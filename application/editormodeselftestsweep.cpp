/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodebuildtools.h"
#include "editor/editorSettings.hpp"
#include "editor/editorGeometry.hpp"
#include "application/application.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"

#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "scene/scenelayers.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Track.h"
#include "world/TrackRails.h"
#include "world/MemCell.h"
#include "world/Event.h"
#include "vehicle/DynObj.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "world/Sweep.h"
#include "input/keyboardinput.h"
#include "launcher/textures_scanner.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "scene/sceneterrain.h"
#include "editor/editorSpeed.hpp"

#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <fstream>
#include <map>
#include <span>
#include <sstream>
#include <string_view>
#include "application/editormodeselftest.h"

namespace
{

using selftest_words = std::map<std::string, std::string, std::less<>>;

// the text put into the buffer of a field, cut to its size
void copy_text(std::span<char> const Buffer, std::string_view const Text)
{
	*std::format_to_n(Buffer.data(), static_cast<std::ptrdiff_t>(Buffer.size()) - 1, "{}", Text).out = '\0';
}

// the anchor named: 1 to 3 in the order of the names, 0 for another name
int anchor_of(std::string_view const Name, std::array<std::string_view, 3> const &Names)
{
	auto const found{std::ranges::find(Names, Name)};
	return found == Names.end() ? 0 : static_cast<int>(found - Names.begin()) + 1;
}

// how long the model is laid: as the original, the whole stretch, or as long as given
int length_mode_of(std::string_view const Length)
{
	if (Length == "original")
		return 0;
	if (Length == "all")
		return 2;
	return 1;
}

// the point given as three numbers apart by commas
glm::dvec3 vector_of(std::string Text)
{
	std::ranges::replace(Text, ',', ' ');
	std::istringstream numbers(Text);
	glm::dvec3 result{0.0};
	numbers >> result.x >> result.y >> result.z;
	return result;
}

// the place of the name among the names, their count for another name
int index_of(std::string const &Text, std::vector<std::string> const &Names)
{
	return static_cast<int>(std::ranges::find(Names, Text) - Names.begin());
}

// the reference point of a bent model
struct bend_reference
{
	int along{0};
	int side{0};
	int height{0};
	glm::dvec3 point{0.0};
};

// the reference point the command gives, its parts not given as they are
bend_reference reference_of(selftest_words &Values, bend_reference const &Current)
{
	bend_reference result{Current};
	if (Values.contains("along"))
		result.along = index_of(Values["along"], {"start", "origin", "middle", "end", "point"});
	if (Values.contains("side"))
		result.side = index_of(Values["side"], {"origin", "near", "far", "middle", "point"});
	if (Values.contains("height"))
		result.height = index_of(Values["height"], {"origin", "bottom", "top", "edge", "point"});
	if (Values.contains("point"))
		result.point = vector_of(Values["point"]);
	return result;
}

// the name of the track, a dash for none
std::string name_or_dash(TTrack const *Track)
{
	return Track != nullptr ? Track->name() : std::string{"-"};
}

// whether the track is there: none, removed, present
char const *presence_of(TTrack const *Track)
{
	if (Track == nullptr)
		return "none";
	return Track->m_editorremoved ? "removed" : "present";
}

} // namespace

editor_selftest_result sweep_editor::selftest_sweep_lay(std::istringstream &words)
{
	auto values{selftest_values(words)};
	auto &tool{m_sweep};
	auto const get = [&values](char const *Key, std::string const &Default) {
		auto const found{values.find(Key)};
		return found != values.end() ? found->second : Default;
	};
	tool.edited = nullptr;
	tool.curve_for = nullptr;
	copy_text(tool.model, get("model", ""));
	tool.parameters.clear();
	std::stringstream parameters(get("params", ""));
	std::string parameter;
	while (std::getline(parameters, parameter, ','))
	{
		tool.parameters.emplace_back();
		copy_text(tool.parameters.back(), parameter);
	}
	tool.parameters_for = tool.model.data();
	auto &settings{tool.settings};
	settings.lateral = std::stod(get("lateral", "0"));
	settings.height = std::stod(get("height", "0"));
	settings.side_anchor = anchor_of(get("side", "origin"), {"near", "far", "centre"});
	settings.height_anchor = anchor_of(get("top", "origin"), {"bottom", "top", "edge"});
	settings.bend = get("mode", "bend") != "repeat";
	settings.step = std::stod(get("step", "0"));
	settings.widen = get("widen", "0") == "1";
	settings.platform = get("platform", "0") == "1";
	settings.face = get("face", "1") == "1";
	settings.tilt = get("tilt", "0") == "1";
	settings.from = std::stod(get("from", "0"));
	settings.to = std::stod(get("to", "-1"));
	auto const length{get("length", "all")};
	tool.length_mode = length_mode_of(length);
	if (tool.length_mode == 1)
		tool.length = std::stod(length);
	if (values.contains("edge"))
		EditorSettings.platform_edge(std::stod(values["edge"]));
	tool.measured.clear();
	tool.open = true;
	m_build.sweep_curve();
	sweep_measure();
	if (values.contains("widen"))
		settings.widen = get("widen", "0") == "1";
	if (values.contains("platform"))
		settings.platform = get("platform", "0") == "1";
	if (get("create", "1") == "1")
	{
		sweep_create();
		WriteLog("SELFTEST sweep: " + tool.status);
	}
	return selftest_result::handled;
}

editor_selftest_result sweep_editor::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "sweep")
		return selftest_sweep_lay(Words);
	if (Command == "drag")
	{
		glm::dvec3 from{0.0};
		glm::dvec3 to{0.0};
		Words >> from.x >> from.z >> to.x >> to.z;
		m_sweep.open = true;
		if (false == sweep_press_at(from, false))
		{
			WriteLog("SELFTEST drag: the press wasn't taken");
			return selftest_result::handled;
		}
		sweep_drag_at(to);
		sweep_release();
		WriteLog("SELFTEST drag: " + m_sweep.status);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_sweep(std::string const &command, std::istringstream &words)
{
	if (command == "spread")
	{
		std::string name;
		words >> name >> m_spread.stop >> m_spread.reach;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr)
			return selftest_result::handled;
		std::string names;
		for (auto const *other : track_spread_from(*track))
			names += ' ' + other->name();
		WriteLog("SELFTEST spread " + name + ":" + names + " | prev " + name_or_dash(track->trPrev) + " next " + name_or_dash(track->trNext));
		return selftest_result::handled;
	}
	if (command == "turntable")
	{
		glm::dvec3 centre{0.0};
		double yaw{0.0};
		words >> centre.x >> centre.y >> centre.z >> yaw;
		turntable_place(centre, yaw);
		for (double angle; words >> angle;)
		{
			m_turntable.heading = m_turntable.table_yaw + angle;
			m_turntable.exit_length = 10.0;
			track_intent intent;
			intent.what = track_intent::kind::turntable_exit;
			turntable_press(intent);
		}
		WriteLog("SELFTEST turntable: " + (m_turntable.error.empty() ? m_turntable.status : m_turntable.error));
		return selftest_result::handled;
	}
	return selftest_bend(command, words);
}

editor_selftest_result bend_editor::selftest_bend_lay(std::istringstream &words)
{
	auto values{selftest_values(words)};
	auto &tool{m_bend};
	auto const reference{reference_of(values, {tool.along_anchor, tool.side_anchor, tool.height_anchor, tool.point})};
	tool.edited = nullptr;
	tool.along_anchor = reference.along;
	tool.side_anchor = reference.side;
	tool.height_anchor = reference.height;
	tool.point = reference.point;
	tool.axis = values.contains("axis") ? index_of(values["axis"], {"auto", "z", "x"}) : 0;
	tool.turned = values["turned"] == "1";
	bend_source source;
	if (false == m_build.bend_source_of_selection(source))
	{
		std::string reason;
		auto *const node{m_build.current_node()};
		auto const editable{node != nullptr && scene::Layers.editable(node, &reason)};
		WriteLog(format("SELFTEST bend: nothing to bend (node %p template %d editable %d %s, instance %d)", static_cast<void *>(node), node != nullptr && node->from_template() ? 1 : 0, editable ? 1 : 0, reason.c_str(), static_cast<int>(m_build.current_instance())));
		return selftest_result::handled;
	}
	tool.tracks_for = nullptr;
	bend_find_tracks(glm::dvec3(source.transform[3]), source.model != nullptr ? static_cast<void const *>(source.model) : std::bit_cast<void const *>(static_cast<std::uintptr_t>(source.instance)));
	tool.track = values.contains("track") ? std::stoi(values["track"]) : 0;
	if (values.contains("pick"))
		bend_pick(vector_of(values["pick"]));
	if (tool.tracks.empty())
	{
		WriteLog("SELFTEST bend: no track nearby");
		return selftest_result::handled;
	}
	sweep_node::state state;
	std::vector<TTrack *> tracks;
	bend_definition(source, *tool.tracks[tool.track].first, state, tracks);
	WriteLog(format("SELFTEST bend: track %s, %zu paths, anchor before %.3f %.3f %.3f, lateral %.3f height %.3f from %.3f to %.3f, axis %s flip %d; %s", tool.tracks[tool.track].first->name().c_str(), tracks.size(),
	                tool.marker.x, tool.marker.y, tool.marker.z, state.lateral, state.height, state.from, state.to, state.along_x ? "x" : "z", state.flip ? 1 : 0, tool.status.c_str()));
	if (values["go"] == "1")
	{
		bend_selection();
		WriteLog("SELFTEST bend: " + tool.status);
	}
	return selftest_result::handled;
}

editor_selftest_result bend_editor::selftest_bend_set(std::istringstream &words)
{
	auto values{selftest_values(words)};
	auto &tool{m_bend};
	auto const reference{reference_of(values, {tool.along_anchor, tool.side_anchor, tool.height_anchor, tool.point})};
	auto const *sweep{tool.edited};
	if (sweep == nullptr)
		return selftest_result::handled;
	auto state{sweep->definition()};
	tool.along_anchor = reference.along;
	if (reference.side != tool.side_anchor || reference.height != tool.height_anchor || reference.point != tool.point)
	{
		tool.side_anchor = reference.side;
		tool.height_anchor = reference.height;
		tool.point = reference.point;
		bend_reanchor(state, reference.side, reference.height, reference.point);
	}
	if (values.contains("lateral"))
		state.lateral = std::stod(values["lateral"]);
	if (values.contains("up"))
		state.height = std::stod(values["up"]);
	if (values.contains("widen"))
		state.widen = values["widen"] == "1";
	if (values.contains("pick"))
	{
		bend_pick(vector_of(values["pick"]));
		state = sweep->definition();
	}
	if (values.contains("length"))
	{
		bend_stretch(*sweep, bend_fixed(*sweep, state.point), std::stod(values["length"]), state.point, state);
	}
	bend_edit(state);
	WriteLog("SELFTEST bendset done");
	return selftest_result::handled;
}

editor_selftest_result bend_editor::selftest_bend_probe() const
{
	auto const *sweep{m_bend.edited};
	if (sweep == nullptr)
	{
		WriteLog("SELFTEST bendprobe: nothing bent");
		return selftest_result::handled;
	}
	glm::dvec3 low;
	glm::dvec3 high;
	glm::dvec3 shift;
	sweep->bounds(low, high, shift);
	auto const &state{sweep->definition()};
	auto const marker{bend_marker(*sweep)};
	glm::dvec3 boxlow{std::numeric_limits<double>::max()};
	glm::dvec3 boxhigh{-std::numeric_limits<double>::max()};
	std::size_t vertices{0};
	for (auto const &shape : sweep->create_shapes())
		for (auto const &vertex : shape.data().vertices)
		{
			boxlow = glm::min(boxlow, vertex.position);
			boxhigh = glm::max(boxhigh, vertex.position);
			++vertices;
		}
	WriteLog(format("SELFTEST bendprobe %s: stations %.3f - %.3f, along the reference line %.3f m, model %.3f m, lateral %.3f height %.3f anchors %d/%d point %.3f %.3f %.3f, marker %.3f %.3f %.3f, box %.2f %.2f %.2f - %.2f %.2f %.2f, %zu vertices, removed %d",
	                sweep->name().c_str(), sweep->start(), sweep->end(), sweep->distance_at(sweep->end()) - sweep->distance_at(sweep->start()), high.x - low.x, state.lateral, state.height, state.side_anchor, state.height_anchor, state.point.x,
	                state.point.y, state.point.z, marker.x, marker.y, marker.z, boxlow.x, boxlow.y, boxlow.z, boxhigh.x, boxhigh.y, boxhigh.z, vertices, sweep->m_editorremoved ? 1 : 0));
	return selftest_result::handled;
}

editor_selftest_result bend_editor::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "bend")
		return selftest_bend_lay(Words);
	if (Command == "bendset")
		return selftest_bend_set(Words);
	if (Command == "bendprobe")
		return selftest_bend_probe();
	if (Command == "bendpanel")
	{
		m_build.build_ui()->expand_bend();
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_bend(std::string const &command, std::istringstream &words)
{
	if (command == "lineside")
	{
		std::string tab;
		words >> tab;
		show_track_tab(tab == "parallel" ? track_tab::parallel : track_tab::lineside);
		if (tab == "vehicle")
			m_vehicle.tab().expand = true;
		else if (tab == "hekto")
			m_hekto.tab().expand = true;
		else if (tab == "fouling")
			m_fouling.tab().expand = true;
		else
			m_parallel.tab().expand = true;
		return selftest_result::handled;
	}
	if (command == "ttincludes")
	{
		std::string name;
		words >> name;
		auto const *track{simulation::Paths.find(name)};
		WriteLog(format("SELFTEST ttincludes %s: track %s, includes %d", name.c_str(), presence_of(track), track != nullptr ? static_cast<int>(turntable_includes(*track).size()) : -1));
		return selftest_result::handled;
	}
	if (command == "checkspans")
	{
		WriteLog("SELFTEST spans: " + scene::Layers.check_sources());
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
