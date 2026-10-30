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

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include "application/editormodeselftest.h"

editor_mode::selftest_result editor_mode::selftest_switch(std::string const &command, std::istringstream &words)
{
	// each switch: the tips of its blades and its drive from the joint before them; drivetips moves the drives there first
	if (command == "bladetips" || command == "drivetips")
	{
		if (command == "drivetips")
			WriteLog(format("SELFTEST drivetips: %zu moved", drives_to_blade_tips()));
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->m_paths.size() < 2)
				continue;
			auto const start{track->m_paths[0].points[segment_data::point::start]};
			std::string text{format("SELFTEST drive of %s: blades from %.3f m", track->name().c_str(), editor_track::blade_tip_of(*track))};
			for (int path = 0; path < 2; ++path)
				if (auto const &shape{track->blade(path)}; shape.valid)
					text += format(", blade %d %.3f - %.3f m", path, shape.tip, shape.heel);
			if (auto const found{find_switch_drive(*track)})
				text += format(", drive %.3f m from the joint", geometry::plan_distance(found->standing.location, start));
			else
				text += ", no drive";
			// how far apart the rail of the blade and the one it lies at are along the path, where the blades of a slip are shaped
			if (track->DoubleSlip())
				for (int path = 0; path < 2; ++path)
				{
					auto const &shape{track->blade(path)};
					auto const gap = [&](double const At) {
						auto const &gauge{shape.gauge};
						auto const next{std::find_if(gauge.begin(), gauge.end(), [&](glm::dvec2 const &Sample) { return Sample.x >= At; })};
						return next == gauge.end() ? (gauge.empty() ? -1.0 : gauge.back().y) : next->y;
					};
					text += format(", path %d %s side %d gaps", path, shape.valid ? "shaped" : "not shaped", track->SwitchExtension->RightSwitch == (path == 0) ? 1 : -1);
					for (auto const at : {0.354, 1.0, 1.248, 2.0, 4.0, 6.0, 8.0, 9.4})
						text += format(" %.1f:%.4f", at, gap(at));
				}
			WriteLog(text);
		}
		return selftest_result::handled;
	}
	if (command == "curvedswitch")
	{
		std::string name;
		std::string label;
		glm::dvec3 at{0.0};
		glm::dvec3 toward{0.0};
		words >> name >> at.x >> at.z >> toward.x >> toward.z >> label;
		std::replace(label.begin(), label.end(), '_', ' ');
		m_node = simulation::Paths.find(name);
		ui()->set_node(m_node);
		show_track_tab(track_tab::turnout);
		arm_switch();
		auto &tool{m_switch};
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			if (false == label.empty() && tool.templates[i].label.find(label) != std::string::npos)
			{
				tool.armed = i;
				break;
			}
		m_cursor_override = at;
		auto const started{start_switch_placement()};
		m_cursor_override.reset();
		if (false == started)
		{
			WriteLog("SELFTEST curvedswitch: not started, " + tool.status);
			return selftest_result::handled;
		}
		tool.mouse = glm::dvec3{toward.x, tool.point.y, toward.z};
		finish_switch_placement();
		WriteLog("SELFTEST curvedswitch " + (tool.armed >= 0 ? tool.templates[tool.armed].label : std::string{}) + ": " + tool.status);
		return selftest_result::handled;
	}
	if (command == "designer")
	{
		std::string list;
		int index{0};
		auto &designer{m_switch_designer};
		designer.pointed_test = -1;
		words >> list >> index >> designer.pointed_test;
		designer.open = list != "close";
		designer.from_catalogue = list != "own";
		designer.selected = index;
		switch_designs_load();
		auto const &list_of{designer.from_catalogue ? designer.catalogue : designer.own};
		if (designer.open && index >= 0 && index < static_cast<int>(list_of.size()))
		{
			std::string problem;
			auto const shape{switch_design::build(list_of[index], problem)};
			auto text{format("SELFTEST designer %s: a %.3f b %.3f 1:%.3f sharpest R %.1f, %.3f m beside the main one, %.3f m long, end %.3f; parts", list_of[index].name.c_str(), shape.a, shape.b, shape.ratio, shape.radius, shape.offset, shape.length, shape.paths.size() > 1 ? shape.paths[1].points[segment_data::point::end].z : 0.0)};
			for (auto const &part : list_of[index].parts)
				text += format(" %.3f/R%.1f", part.length, part.radius1);
			WriteLog(text + " " + problem);
		}
		return selftest_result::handled;
	}
	if (command == "designerpick")
	{
		// the points of a switch clicked off the view for the template editor, by x z; finish: the template made of them
		auto &designer{m_switch_designer};
		designer.open = true;
		designer.picking = true;
		for (glm::dvec2 point; words >> point.x >> point.y;)
			switch_designer_click({point.x, 0.0, point.y});
		words.clear();
		std::string finish;
		words >> finish;
		if (finish == "finish")
		{
			switch_designer_finish();
			WriteLog("SELFTEST designerpick: " + designer.status);
			if (false == designer.from_catalogue && designer.selected >= 0 && designer.selected < static_cast<int>(designer.own.size()))
			{
				auto text{std::string{"SELFTEST designerpick parts"}};
				for (auto const &part : designer.own[designer.selected].parts)
					text += format(" %.3f/R%.1f..%.1f", part.length, part.radius1, part.radius2);
				WriteLog(text + format(", lead %.3f", designer.own[designer.selected].lead));
			}
		}
		return selftest_result::handled;
	}
	if (command == "release")
	{
		m_switch.placing = false;
		lay_cancel();
		m_cursor_override.reset();
		return selftest_result::handled;
	}
	if (command == "hover")
	{
		glm::dvec3 at{0.0};
		if (words >> at.x >> at.z)
			m_cursor_override = at;
		else
			m_cursor_override.reset();
		return selftest_result::handled;
	}
	if (command == "layhold")
	{
		int layout{0};
		words >> layout >> m_lay.radius;
		m_track_layout = static_cast<track_layout>(std::clamp(layout, 0, 3));
		m_lay.active = true;
		lay_cancel();
		std::vector<glm::dvec3> points;
		for (glm::dvec3 point{0.0}; words >> point.x >> point.z;)
			points.push_back(point);
		for (std::size_t i = 0; i + 1 < points.size(); ++i)
		{
			auto point{points[i]};
			point.y = 5000.0;
			std::vector<char> found;
			auto const ground{ground_heights({point}, true, &found)};
			point.y = found[0] != 0 ? ground[0] : m_lay.points.empty() ? 0.0 : m_lay.points.back().y;
			m_lay.mouse_snap = editor_track::find_free_end(nullptr, 1, point, 3.0, {});
			m_lay.mouse = m_lay.mouse_snap.track != nullptr ? point : lay_bed(point);
			lay_click();
		}
		if (false == points.empty())
			m_cursor_override = points.back();
		WriteLog("SELFTEST layhold: " + m_lay.status);
		return selftest_result::handled;
	}
	if (command == "switchkind")
	{
		// what the switch tool places, as chosen in its panel: single, slip, crossing or connection with the kind of it
		std::string word;
		int sub{0};
		words >> word >> sub;
		auto const kind{word == "slip" ? switch_kind::slip : word == "crossing" ? switch_kind::crossing : word == "connection" ? switch_kind::connection : switch_kind::single};
		m_switch.connection_kind = sub;
		choose_switch_kind(kind);
		WriteLog(format("SELFTEST switchkind %s: kind %d, connection %d, armed %d", word.c_str(), static_cast<int>(current_switch_kind()), m_switch.connection, m_switch.armed));
		return selftest_result::handled;
	}
	if (command == "connection" || command == "connectionplan" || command == "connectionhold")
	{
		std::string kindword;
		std::string label;
		std::string second;
		glm::dvec3 at{0.0};
		glm::dvec3 toward{0.0};
		words >> kindword >> at.x >> at.z >> toward.x >> toward.z >> label >> second;
		connection::kind kind{connection::kind::crossover};
		if (kindword != "single" && false == connection::parse(kindword, kind))
		{
			WriteLog("SELFTEST connection: unknown kind " + kindword);
			return selftest_result::handled;
		}
		auto &tool{m_switch};
		for (std::string key; words >> key;)
		{
			if (key == "radius")
				words >> tool.connection_radius;
			else if (key == "gap")
				words >> tool.connection_gap;
			else if (key == "together")
				words >> tool.connection_together;
			else if (key == "stub")
				words >> tool.connection_stub;
			else if (key == "spacings")
			{
				std::string value;
				words >> value;
				std::snprintf(tool.connection_spacings, sizeof(tool.connection_spacings), "%s", value.c_str());
			}
		}
		show_track_tab(track_tab::turnout);
		arm_switch();
		auto const find = [&](std::string Label) {
			std::replace(Label.begin(), Label.end(), '_', ' ');
			for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
				if (false == Label.empty() && Label != "-" && false == tool.templates[i].double_slip && false == tool.templates[i].twofold() && tool.templates[i].label.find(Label) != std::string::npos)
					return i;
			return -1;
		};
		auto const first{find(label)};
		if (first < 0)
		{
			WriteLog("SELFTEST connection: no template " + label);
			return selftest_result::handled;
		}
		tool.armed = first;
		tool.connection_second = find(second);
		tool.connection = kindword == "single" ? -1 : static_cast<int>(kind);
		auto const line{straight_near(at, {}, 3.0)};
		TTrack *start{line.tracks.empty() ? nullptr : line.tracks.front()};
		if (start == nullptr)
		{
			// along a curve: the track nearest to the press
			auto best{3.0};
			for (auto *track : simulation::Paths.sequence())
			{
				if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
					continue;
				auto const distance{geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, at)), at)};
				if (distance < best)
				{
					best = distance;
					start = track;
				}
			}
		}
		if (start == nullptr)
		{
			WriteLog("SELFTEST connection: no track at the start");
			return selftest_result::handled;
		}
		m_node = start;
		ui()->set_node(m_node);
		m_cursor_override = at;
		auto const started{start_switch_placement()};
		m_cursor_override.reset();
		if (false == started)
		{
			WriteLog("SELFTEST connection: not started, " + tool.status);
			return selftest_result::handled;
		}
		tool.mouse = glm::dvec3{toward.x, tool.point.y, toward.z};
		if (tool.curved)
		{
			finish_switch_placement();
			WriteLog("SELFTEST connection along the curve: " + tool.status);
			return selftest_result::handled;
		}
		if (command == "connectionhold")
		{
			m_cursor_override = toward;
			WriteLog("SELFTEST connectionhold: " + tool.status);
			return selftest_result::handled;
		}
		connection::request request;
		connection::result plan;
		auto const valid{plan_connection(request, plan)};
		WriteLog("SELFTEST connection " + kindword + ": " + (valid ? connection_readout(plan) : "invalid, " + plan.error));
		for (auto const &entry : plan.switches)
		{
			auto const &start{entry.paths[0].points[segment_data::point::start]};
			auto const &end{entry.paths[1].points[segment_data::point::end]};
			WriteLog(format("SELFTEST connection switch on %d at %.3f, forward %d side %d%s: %.3f %.3f %.3f -> branch end %.3f %.3f %.3f", entry.line, entry.along, entry.forward, entry.side, entry.second ? " (second)" : "", start.x, start.y, start.z, end.x, end.y,
			                end.z));
		}
		for (auto const &insert : plan.inserts)
		{
			auto const &start{insert.points[segment_data::point::start]};
			auto const &end{insert.points[segment_data::point::end]};
			WriteLog(format("SELFTEST connection insert %.3f %.3f %.3f -> %.3f %.3f %.3f, %.3f m, R %.1f", start.x, start.y, start.z, end.x, end.y, end.z, geometry::bezier{insert}.plan_length(), std::abs(insert.radius)));
		}
		if (command == "connectionplan" || false == valid)
		{
			tool.placing = false;
			return selftest_result::handled;
		}
		insert_connection();
		WriteLog("SELFTEST connection built: " + tool.status);
		for (auto const &entry : switch_store::records())
			if (entry.kind == "connection")
			{
				std::string parts;
				for (auto const &part : entry.parts)
					parts += ' ' + part;
				WriteLog(format("SELFTEST connection record %s %s: spacing %.3f, insert %.3f, parts%s", entry.type.c_str(), entry.name.c_str(), entry.a, entry.b, parts.c_str()));
			}
		return selftest_result::handled;
	}
	if (command == "conecheck")
	{
		std::string name;
		words >> name;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr || track->m_paths.size() < 2)
		{
			WriteLog("SELFTEST conecheck: no switch " + name);
			return selftest_result::handled;
		}
		geometry::bezier const main{track->m_paths[0]};
		geometry::bezier const other{track->m_paths[1]};
		std::string text;
		for (int i = 1; i <= 8; ++i)
		{
			auto const t{i * 0.125};
			auto const point{other.point(t)};
			double best{std::numeric_limits<double>::max()};
			double along{0.0};
			for (int k = 0; k <= 2000; ++k)
				if (auto const distance{geometry::plan_distance(main.point(k / 2000.0), point)}; distance < best)
				{
					best = distance;
					along = k / 2000.0;
				}
			auto const base{main.point(along)};
			auto const direction{glm::normalize(geometry::plan_of(main.first(along)))};
			auto const lateral{(point.x - base.x) * -direction.y + (point.z - base.z) * direction.x};
			auto const &path{track->m_paths[0]};
			auto const roll{glm::radians(path.rolls[0] + (path.rolls[1] - path.rolls[0]) * along)};
			auto const tangent{main.first(along)};
			auto const beyond{(point.x - base.x) * direction.x + (point.z - base.z) * direction.y};
			auto const cone{base.y + beyond * tangent.y / glm::length(geometry::plan_of(tangent)) - lateral * std::tan(roll)};
			text += format(" [t %.3f off %.3f dy %.4f]", t, lateral, point.y - cone);
		}
		WriteLog("SELFTEST conecheck " + name + ":" + text);
		return selftest_result::handled;
	}
	// a walk along the tracks from the plain one nearest the point as an axle runs: along it (1) or against it (-1), on the path of a switch
	// it's set to, or the one come onto from its end; each track, the path of a switch and the way, END where no track goes on
	if (command == "pathwalk")
	{
		glm::dvec3 point{0.0};
		double direction{1.0};
		int steps{8};
		words >> point.x >> point.z >> direction >> steps;
		TTrack *track{nullptr};
		double nearest{1e9};
		for (auto *candidate : simulation::Paths.sequence())
		{
			if (candidate == nullptr || candidate->m_editorremoved || candidate->eType != tt_Normal || candidate->m_paths.empty())
				continue;
			if (auto const d{geometry::plan_distance(editor_track::point_at(*candidate, editor_track::nearest_parameter(*candidate, point)), point)}; d < nearest)
			{
				nearest = d;
				track = candidate;
			}
		}
		std::string text{"SELFTEST pathwalk:"};
		int path{-1};
		for (int step = 0; step < steps && track != nullptr; ++step)
		{
			text += " " + track->name() + (path >= 0 ? "[" + std::to_string(path) + "]" : std::string{}) + (direction > 0 ? ">" : "<");
			auto const side{direction > 0 ? 1 : -1};
			auto const switched{track->eType == tt_Switch && path >= 0};
			auto const joined{switched ? (side > 0 ? track->SwitchExtension->iNextDirection[path] : track->SwitchExtension->iPrevDirection[path]) : (side > 0 ? track->iNextDirection : track->iPrevDirection)};
			auto *next{track->Connected(side, direction, switched ? path : -1)};
			if (next == nullptr)
			{
				text += " END";
				break;
			}
			path = next->eType == tt_Switch ? ((joined & 1) ? joined >> 1 : next->SwitchExtension->CurrentIndex) : -1;
			track = next;
		}
		WriteLog(text);
		return selftest_result::handled;
	}
	// the key H as pressed over the view: the selected switch thrown over
	if (command == "keyh")
	{
		auto const handled{track_shortcut(GLFW_KEY_H)};
		auto const *track{selected_track()};
		auto const switched{track != nullptr && track->eType == tt_Switch && track->SwitchExtension != nullptr};
		WriteLog(format("SELFTEST keyh: %s, %s set to %d, blades at %.3f of %.3f", handled ? "thrown" : "not taken", track != nullptr ? track->name().c_str() : "-", switched ? track->SwitchExtension->CurrentIndex : -1, switched ? track->SwitchExtension->fOffset : 0.f,
		                switched ? track->SwitchExtension->fDesiredOffset : 0.f));
		return selftest_result::handled;
	}
	// where the blades of the switch are on their way over
	if (command == "blades")
	{
		std::string name;
		words >> name;
		auto const *track{simulation::Paths.find(name)};
		if (track != nullptr && track->eType == tt_Switch && track->SwitchExtension != nullptr)
			WriteLog(format("SELFTEST blades %s: %.3f of %.3f", name.c_str(), track->SwitchExtension->fOffset, track->SwitchExtension->fDesiredOffset));
		return selftest_result::handled;
	}
	if (command == "throw")
	{
		std::string name;
		int state{0};
		words >> name >> state;
		auto *track{simulation::Paths.find(name)};
		if (track != nullptr && track->eType == tt_Switch)
		{
			track->Switch(state, 0.f, 0.f);
			// the blades go over at once, the editor doesn't run the animations of the switches
			track->SwitchExtension->fOffset = track->SwitchExtension->fDesiredOffset;
			track->RaAnimate();
		}
		WriteLog(format("SELFTEST throw %s %d%s", name.c_str(), state, track != nullptr && track->eType == tt_Switch ? "" : ": no such switch"));
		return selftest_result::handled;
	}
	if (command == "switchdrive")
	{
		int index{-1};
		int side{0};
		words >> index >> side;
		scan_switch_drives();
		m_switch.drive = index;
		m_switch.drive_side = side;
		WriteLog("SELFTEST switchdrive: " + (index >= 0 && index < static_cast<int>(m_switch.drives.size()) ? m_switch.drives[index].name : std::string{"none"}));
		return selftest_result::handled;
	}
	if (command == "drivechoice")
	{
		int index{-1};
		int side{0};
		words >> index >> side;
		scan_switch_drives();
		m_switch.drive = index;
		m_switch.drive_side = side;
		m_switch.status.clear();
		auto *track{selected_track()};
		if (track != nullptr && track->eType == tt_Switch && false == track->m_editorremoved)
			apply_switch_drive_choice(*track);
		WriteLog("SELFTEST drivechoice: " + (track != nullptr ? track->name() : std::string{"no switch"}) + ", " + m_switch.status);
		return selftest_result::handled;
	}
	if (command == "drives")
	{
		std::string name;
		words >> name;
		scan_switch_drives();
		std::string files;
		for (auto const &standing : standing_templates([](std::string const &File) { return File.find("napedy/") != std::string::npos; }))
			if (false == standing.values.empty() && standing.values[0] == name)
			{
				files += " " + standing.file;
				for (std::size_t i = 1; i < standing.values.size(); ++i)
					files += (i == 1 ? " (" : " ") + standing.values[i] + (i + 1 == standing.values.size() ? ")" : "");
			}
		for (scene::instance_handle handle = 1; handle <= scene::Layers.instance_count(); ++handle)
			if (scene::Layers.tracked(handle) && scene::Layers.instance(handle).file != nullptr && scene::Layers.instance(handle).file->find("napedy") != std::string::npos)
				files += format(" [%d %s removed %d dead %d]", static_cast<int>(handle), scene::Layers.instance(handle).file->c_str(), scene::Layers.instance(handle).removed ? 1 : 0, scene::Layers.instance(handle).dead ? 1 : 0);
		WriteLog("SELFTEST drives " + name + ": chosen " + (m_switch.drive >= 0 ? m_switch.drives[m_switch.drive].name : std::string{"none"}) + ", side " + std::to_string(m_switch.drive_side) + ", standing" + files + ", steps " + std::to_string(m_history.size()));
		return selftest_result::handled;
	}
	if (command == "driveflip")
	{
		std::string name;
		words >> name;
		auto *track{simulation::Paths.find(name)};
		auto const done{track != nullptr && flip_switch_drive(*track)};
		WriteLog("SELFTEST driveflip " + name + ": " + (done ? "done " : "failed ") + m_switch.status);
		return selftest_result::handled;
	}
	if (command == "driveput" || command == "drivefamily")
	{
		std::string name;
		int family{-1};
		words >> name >> family;
		auto *track{simulation::Paths.find(name)};
		auto const done{track != nullptr && (command == "driveput" ? place_switch_drive(*track) : replace_switch_drive(*track, family, false))};
		WriteLog("SELFTEST " + command + " " + name + ": " + (done ? "done " : "failed ") + m_switch.status);
		return selftest_result::handled;
	}
	if (command == "drivetilt")
	{
		std::string name;
		words >> name;
		auto *model{simulation::Instances.find(name)};
		auto *track{simulation::Paths.find(name.substr(0, name.find('_')))};
		if (model == nullptr || track == nullptr)
		{
			WriteLog("SELFTEST drivetilt: no " + name);
			return selftest_result::handled;
		}
		auto const &path{track->m_paths[0]};
		auto const transform{glm::dmat3(model->rotation_scale())};
		auto const across{transform * glm::dvec3{1.0, 0.0, 0.0}};
		auto const forward{transform * glm::dvec3{0.0, 0.0, 1.0}};
		auto const direction{glm::normalize(glm::dvec2{forward.x, forward.z})};
		auto const lateral{across.x * -direction.y + across.z * direction.x};
		auto const cone{-lateral * std::tan(glm::radians(static_cast<double>(path.rolls[0])))};
		auto const control{path.points[segment_data::point::control1]};
		auto const grade{control.y / std::hypot(control.x, control.z)};
		WriteLog(format("SELFTEST drivetilt %s: angles %.3f %.3f %.3f, +x rises %.4f (cone %.4f), +z rises %.4f (grade %.4f), origin %.3f %.3f %.3f, points %.3f", name.c_str(), model->Angles().x, model->Angles().y, model->Angles().z, across.y, cone, forward.y, grade,
		                model->location().x, model->location().y, model->location().z, path.points[segment_data::point::start].y));
		return selftest_result::handled;
	}
	if (command == "switchfree")
	{
		// a switch with no track: pressed at the point, the direction of its main track toward the second one (none at a free end), let
		// go at the third one, whose side it takes; past its end from a free end it's bent, one: one-sided (Tab), shift: at a free end the
		// diverging track leads into it
		std::string label;
		glm::dvec3 press{0.0};
		glm::dvec3 heading{0.0};
		glm::dvec3 release{0.0};
		words >> label >> press.x >> press.z >> heading.x >> heading.z >> release.x >> release.z;
		std::replace(label.begin(), label.end(), '_', ' ');
		auto oneside{false};
		auto shift{false};
		for (std::string flag; words >> flag;)
		{
			oneside |= flag == "one";
			shift |= flag == "shift";
		}
		auto &tool{m_switch};
		show_track_tab(track_tab::turnout);
		tool.templates = editor_track::standard_switch_templates();
		tool.collected = true;
		tool.connection = -1;
		tool.armed = -1;
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			if (tool.templates[i].label.rfind(label, 0) == 0)
			{
				tool.armed = i;
				break;
			}
		// the free end taken where it was pressed, not by the mouse on the screen
		m_cursor_override = press;
		start_free_switch(press);
		m_cursor_override.reset();
		if (false == tool.placing)
		{
			WriteLog("SELFTEST switchfree: not started, no template " + label);
			return selftest_result::handled;
		}
		if (tool.free_end.track == nullptr)
			tool.free_heading = glm::normalize(geometry::plan_of(heading - tool.point));
		tool.mouse = glm::dvec3{release.x, tool.point.y, release.z};
		tool.one_sided = oneside;
		// the free end the cursor is at found where it was let go, not by the mouse on the screen; Shift held as it's let go
		m_cursor_override = tool.mouse;
		auto &io{ImGui::GetIO()};
		auto const shifted{io.KeyShift};
		io.KeyShift = shift;
		tool.placing = false;
		finish_switch_placement();
		io.KeyShift = shifted;
		m_cursor_override.reset();
		WriteLog("SELFTEST switchfree" + (tool.free_end.track != nullptr ? " from " + tool.free_end.track->name() : std::string{}) + ": " + tool.status);
		if (auto const *track{selected_track()}; track != nullptr && track->eType == tt_Switch)
			for (std::size_t i = 0; i < track->m_paths.size(); ++i)
			{
				auto const &a{track->m_paths[i].points[segment_data::point::start]};
				auto const &b{track->m_paths[i].points[segment_data::point::end]};
				// the radius the path turns with from end to end, signed the way it turns
				geometry::bezier const curve{track->m_paths[i]};
				auto const turn{geometry::signed_angle(geometry::plan_of(curve.first(0.0)), geometry::plan_of(curve.first(1.0)))};
				WriteLog(format("SELFTEST switchfree path %s/%zu: %.3f %.3f %.3f -> %.3f %.3f %.3f, R %.1f, turning R %.1f", track->name().c_str(), i, a.x, a.y, a.z, b.x, b.y, b.z, static_cast<double>(track->m_paths[i].radius),
				                std::abs(turn) > 1e-6 ? curve.plan_length() / turn : 0.0));
			}
		return selftest_result::handled;
	}
	if (command == "switchplace" || command == "switchcurve" || command == "slipplace")
	{
		std::string label;
		glm::dvec3 point{0.0};
		int direction{1};
		int side{1};
		words >> label >> point.x >> point.z >> direction >> side;
		// snap: the start of a switch into a curve drawn to the ends of the cant ramps and to the joints, as a drag lets it go
		std::string snap;
		words >> snap;
		std::replace(label.begin(), label.end(), '_', ' ');
		auto &tool{m_switch};
		tool.templates = editor_track::standard_switch_templates();
		tool.collected = true;
		tool.armed = -1;
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			if (tool.templates[i].label.rfind(label, 0) == 0)
			{
				tool.armed = i;
				break;
			}
		TTrack *nearest{nullptr};
		double distance{1e9};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
				continue;
			auto const d{geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, point)), point)};
			if (d < distance)
			{
				distance = d;
				nearest = track;
			}
		}
		if (tool.armed < 0 || nearest == nullptr)
		{
			WriteLog("SELFTEST " + command + ": no template " + label + " or no path");
			return selftest_result::handled;
		}
		tool.status.clear();
		if (command == "switchcurve")
		{
			editor_track::curve curve;
			editor_track::chain chain;
			std::string error;
			if (false == editor_track::find_curve(*nearest, m_straights.tolerance, m_route.design.norms.gauge, curve) || false == editor_track::find_chain(curve.from, curve.to, chain, error, true))
			{
				WriteLog("SELFTEST switchcurve: no curve");
				return selftest_result::handled;
			}
			tool.frame = sample_chain(chain);
			tool.frame_tracks = chain.tracks;
			tool.curved = true;
			tool.along = nearest_station(tool.frame, point);
			tool.pressed = tool.along;
			if (snap == "snap")
			{
				std::string what;
				tool.along = curved_switch_start(direction, &what);
				WriteLog(format("SELFTEST switchcurve start %.3f, pressed %.3f%s", tool.along, tool.pressed, what.empty() ? "" : (": " + what).c_str()));
				// the tracks of the curve and their ends along it, where the frame has them and where they are
				for (auto const *track : tool.frame_tracks)
					for (auto const index : {segment_data::point::start, segment_data::point::end})
					{
						auto const &at{track->m_paths.front().points[index]};
						auto const station{nearest_station(tool.frame, at)};
						auto const framed{sample_at(tool.frame, station).position};
						WriteLog(format("SELFTEST switchcurve frame %s.%s: station %.3f, at %.3f %.3f, frame %.3f %.3f", track->name().c_str(), index == segment_data::point::start ? "start" : "end", station, at.x, at.z, framed.x, framed.z));
					}
			}
			// a slip bent into the curve with its crossing track, its paths and its record logged
			if (tool.templates[tool.armed].double_slip)
			{
				insert_curved_slip(direction, side);
				if (auto const *part{selected_track()}; part != nullptr && part->DoubleSlip())
					if (auto const known{switch_store::named(*part)})
					{
						WriteLog(format("SELFTEST switchcurve slip %s: %s, %zu parts", known->name.c_str(), switch_store::label(*known).c_str(), known->parts.size()));
						for (auto const &name : known->parts)
							if (auto const *track{simulation::Paths.find(name)})
								for (std::size_t i = 0; i < track->m_paths.size(); ++i)
								{
									auto const &path{track->m_paths[i]};
									auto const &a{path.points[segment_data::point::start]};
									auto const &b{path.points[segment_data::point::end]};
									WriteLog(format("SELFTEST switchcurve slip path %s/%zu: %.3f %.3f %.3f -> %.3f %.3f %.3f, R %.1f, %zu pieces, rolls %.2f %.2f", name.c_str(), i, a.x, a.y, a.z, b.x, b.y, b.z, static_cast<double>(path.radius), path.pieces.size(),
									                static_cast<double>(path.rolls[0]), static_cast<double>(path.rolls[1])));
								}
					}
			}
			else
				insert_curved_switch(direction, side);
		}
		else
		{
			auto const line{editor_track::find_straight(*nearest, m_straights.tolerance)};
			if (command == "slipplace")
			{
				WriteLog(format("SELFTEST slipplace: straight from %s, %zu paths, start %.3f %.3f, %.3f m", nearest->name().c_str(), line.tracks.size(), line.start.x, line.start.z, line.length));
				insert_double_slip(line, point, tool.templates[tool.armed]);
			}
			else
				insert_switch(line, std::clamp(line.along(point), 0.0, line.length), direction, side);
		}
		WriteLog("SELFTEST " + command + ": " + tool.status + " " + m_straights.status);
		return selftest_result::handled;
	}
	if (command == "switchlink" || command == "branchlink")
	{
		std::string label;
		glm::dvec3 point{0.0};
		glm::dvec3 target{0.0};
		if (command == "switchlink")
			words >> label;
		words >> point.x >> point.z >> target.x >> target.z;
		std::replace(label.begin(), label.end(), '_', ' ');
		auto const log_link = [&](editor_track::branch_link const &Link, std::string const &Reason) {
			WriteLog(format("SELFTEST %s plan: valid %d, R %.1f m, before %.3f m, after %.3f m, trim %.3f m, shortfall %.3f m, %zu pieces%s", command.c_str(), Link.valid ? 1 : 0, Link.radius, Link.before, Link.after, Link.trim,
			                Link.shortfall, Link.pieces.size(), Reason.empty() ? "" : (", " + Reason).c_str()));
			for (auto const &piece : Link.pieces)
			{
				auto const &a{piece.points[segment_data::point::start]};
				auto const &b{piece.points[segment_data::point::end]};
				WriteLog(format("SELFTEST %s piece: %.3f %.3f %.3f -> %.3f %.3f %.3f, L %.3f m, R %.1f m", command.c_str(), a.x, a.y, a.z, b.x, b.y, b.z, geometry::bezier{piece}.plan_length(), static_cast<double>(piece.radius)));
			}
		};
		if (command == "switchlink")
		{
			show_track_tab(track_tab::turnout);
			auto &tool{m_switch};
			tool.templates = editor_track::standard_switch_templates();
			tool.collected = true;
			tool.armed = -1;
			tool.connection = -1;
			for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
				if (tool.templates[i].label.rfind(label, 0) == 0)
				{
					tool.armed = i;
					break;
				}
			TTrack *nearest{nullptr};
			double distance{1e9};
			for (auto *track : simulation::Paths.sequence())
			{
				if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
					continue;
				auto const d{geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, point)), point)};
				if (d < distance)
				{
					distance = d;
					nearest = track;
				}
			}
			if (tool.armed < 0 || nearest == nullptr)
			{
				WriteLog("SELFTEST switchlink: no template " + label + " or no path");
				return selftest_result::handled;
			}
			m_node = nearest;
			ui()->set_node(m_node);
			m_cursor_override = point;
			if (false == start_switch_placement())
			{
				m_cursor_override.reset();
				WriteLog("SELFTEST switchlink: not started, " + tool.status);
				return selftest_result::handled;
			}
			tool.mouse = glm::dvec3{target.x, tool.point.y, target.z};
			switch_link plan;
			plan_switch_link(plan);
			log_link(plan.link, plan.reason);
			finish_switch_placement();
			m_cursor_override.reset();
			WriteLog("SELFTEST switchlink: " + tool.status);
			return selftest_result::handled;
		}
		TTrack *own{nullptr};
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved && track->eType == tt_Switch && track->m_paths.size() == 2 && geometry::plan_distance(track->m_paths[1].points[segment_data::point::end], point) < 3.0)
				own = track;
		if (own == nullptr)
		{
			WriteLog("SELFTEST branchlink: no switch branch end there");
			return selftest_result::handled;
		}
		editor_track::snap_target end;
		end.track = own;
		end.point = {1, editor_track::point_kind::end};
		end.position = own->m_paths[1].points[segment_data::point::end];
		if (false == start_extend(end))
		{
			WriteLog("SELFTEST branchlink: the branch end isn't free");
			return selftest_result::handled;
		}
		m_extend.pressed = {-10000.f, -10000.f};
		m_extend.mouse = glm::dvec3{target.x, m_extend.point.y, target.z};
		m_extend.snap = editor_track::find_free_end(own, own->iCategoryFlag & 15, m_extend.mouse, 3.0, {own});
		editor_track::branch_link link;
		std::string reason;
		plan_branch_link(link, reason);
		log_link(link, reason);
		finish_extend();
		WriteLog(format("SELFTEST branchlink: switch now %.3f %.3f -> %.3f %.3f", own->m_paths[0].points[segment_data::point::start].x, own->m_paths[0].points[segment_data::point::start].z, own->m_paths[0].points[segment_data::point::end].x,
		                own->m_paths[0].points[segment_data::point::end].z));
		return selftest_result::handled;
	}
	if (command == "switchslide" || command == "switchreplace")
	{
		std::string name;
		words >> name;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr || track->eType != tt_Switch || track->m_paths.size() < 2)
		{
			WriteLog("SELFTEST " + command + ": no switch " + name);
			return selftest_result::handled;
		}
		if (command == "switchslide")
		{
			// back, ahead: one bent into a curve as far as the far end of the track next to it
			double distance{0.0};
			std::string word;
			words >> word;
			std::vector<curve_sample> frame;
			std::array<TTrack *, 2> sides;
			double station{0.0};
			if ((word == "back" || word == "ahead") && bent_switch_frame(*track, frame, sides, station))
				distance = word == "back" ? -station : frame.back().station - station - (nearest_station(frame, track->m_paths.front().points[segment_data::point::end]) - station);
			else
				distance = std::atof(word.c_str());
			double low{0.0};
			double high{0.0};
			std::string reason;
			auto const set{double_of(*track)};
			double length{0.0};
			auto const room{set.has_value() ? double_room(*set, low, high, length, reason) : switch_room(*track, low, high, reason)};
			auto const done{slide_switch(*track, distance)};
			WriteLog(format("SELFTEST switchslide %s %.3f: room %d %.3f .. %.3f, done %d, %s", name.c_str(), distance, room ? 1 : 0, low, high, done ? 1 : 0, m_switch.status.c_str()));
			// how the switch meets the tracks at its ends: the turn between them, the gap, the radius of the one next to it
			for (auto const &joint : editor_track::joints(*track))
			{
				geometry::bezier const mine{track->m_paths[joint.own.path]};
				auto const atend{joint.own.kind == editor_track::point_kind::end};
				auto const out{geometry::plan_of(atend ? mine.first(1.0) : -mine.first(0.0))};
				// the direction of the joint points out of the other track at its end
				auto const kink{glm::length(out) > 1e-9 && glm::length(joint.direction) > 1e-9 ? glm::degrees(std::acos(std::clamp(-glm::dot(glm::normalize(out), glm::normalize(joint.direction)), -1.0, 1.0))) : 0.0};
				auto const at{editor_track::point_position(*track, joint.own)};
				geometry::bezier const theirs{joint.other->m_paths[joint.theirs.path]};
				auto const turn{geometry::signed_angle(geometry::plan_of(theirs.first(0.0)), geometry::plan_of(theirs.first(1.0)))};
				WriteLog(format("SELFTEST switchslide joint %d.%s %.3f %.3f - %s: kink %.4f deg, gap %.4f m, its R %.1f, L %.2f m", joint.own.path, atend ? "end" : "start", at.x, at.z, joint.other->name().c_str(), kink,
				                glm::distance(at, editor_track::point_position(*joint.other, joint.theirs)), std::abs(turn) > 1e-6 ? theirs.plan_length() / turn : 0.0, theirs.plan_length()));
			}
			return selftest_result::handled;
		}
		std::string label;
		int side{1};
		words >> label >> side;
		std::replace(label.begin(), label.end(), '_', ' ');
		auto const templates{editor_track::standard_switch_templates()};
		auto const found{std::find_if(templates.begin(), templates.end(), [&](editor_track::switch_template const &Template) { return Template.label.rfind(label, 0) == 0; })};
		if (found == templates.end())
		{
			WriteLog("SELFTEST switchreplace: no template " + label);
			return selftest_result::handled;
		}
		auto const set{double_of(*track)};
		if (set.has_value())
			track = set->first;
		auto const &main{track->m_paths[0]};
		auto const origin{main.points[segment_data::point::start]};
		auto const chord{geometry::plan_of(main.points[segment_data::point::end] - origin)};
		auto const length{glm::length(chord)};
		auto paths{editor_track::place_switch(*found, origin, chord / length, side, (main.points[segment_data::point::end].y - origin.y) / length)};
		std::string status;
		auto const done{set.has_value() ? found->twofold() && paths.size() >= 5 && move_double_paths(*set, paths, status) : paths.size() >= 2 && move_switch_paths(*track, paths, status)};
		if (done && set.has_value())
		{
			record_double(*set->first, *set->second, *found);
			switch_store::follow();
			records_to_history();
			straight_refresh();
		}
		else if (done)
		{
			record_switch(*track, *found);
			records_to_history();
			straight_refresh();
		}
		WriteLog(format("SELFTEST switchreplace %s %s: done %d, %s", name.c_str(), found->label.c_str(), done ? 1 : 0, status.c_str()));
		return selftest_result::handled;
	}
	if (command == "switchpieces" || command == "switchbreak")
	{
		double radius{0.0};
		double ratio{0.0};
		std::string name;
		if (command == "switchpieces")
			words >> radius >> ratio;
		words >> name;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->m_paths.size() < 2 || track->SwitchExtension == nullptr)
				continue;
			if (false == name.empty() ? track->name() != name : track->m_paths[1].pieces.empty())
				continue;
			if (command == "switchbreak")
			{
				auto &pieces{track->m_paths[1].pieces};
				if (pieces.size() < 2)
					continue;
				pieces[0].points[segment_data::point::end].x += 0.3;
				editor_track::commit({track});
				WriteLog(format("SELFTEST switchbreak %s: path 2 %.3f m, %zu pieces in use, %zu kept", track->name().c_str(), track->SwitchExtension->Segments[1]->GetLength(), track->SwitchExtension->Segments[1]->PieceCount(), pieces.size()));
				return selftest_result::handled;
			}
			auto const &segment{*track->SwitchExtension->Segments[1]};
			auto const &main{track->m_paths[0]};
			auto const &branch{track->m_paths[1]};
			auto const origin{segment.FastGetPoint(0.0)};
			auto const plan{glm::normalize(geometry::plan_of(main.points[segment_data::point::end] - main.points[segment_data::point::start]))};
			glm::dvec3 const along{plan.x, 0.0, plan.y};
			auto const reach{branch.points[segment_data::point::end] - branch.points[segment_data::point::start]};
			auto const hand{along.x * reach.z - along.z * reach.x > 0.0 ? 1.0 : -1.0};
			auto const normal{glm::dvec3{-along.z, 0.0, along.x} * hand};
			auto const centre{origin + normal * radius};
			auto const angle{std::atan(1.0 / ratio)};
			auto const arc{radius * angle};
			auto const heading0{-std::atan2(along.x, along.z)};
			glm::dvec3 position;
			glm::vec3 rotation;
			double worst{0.0};
			for (int i = 0; i <= 10; ++i)
			{
				segment.RaPositionGet(arc * i / 10.0, position, rotation);
				worst = std::max(worst, std::abs(geometry::plan_distance(position, centre) - radius));
			}
			segment.RaPositionGet(arc, position, rotation);
			auto const expected{centre - normal * radius * std::cos(angle) + along * radius * std::sin(angle)};
			auto const turn{std::abs(std::remainder(rotation.z - heading0, glm::two_pi<double>()))};
			double low{1e9};
			double high{-1e9};
			for (int i = 0; i <= 10; ++i)
			{
				segment.RaPositionGet(arc + 0.05 + (segment.GetLength() - arc - 0.1) * i / 10.0, position, rotation);
				low = std::min(low, static_cast<double>(rotation.z));
				high = std::max(high, static_cast<double>(rotation.z));
			}
			segment.RaPositionGet(arc, position, rotation);
			WriteLog(format("SELFTEST switchpieces %s: path 2 %.3f m in %zu pieces (%zu kept), arc %.3f m, arc end off %.4f m, turn there %.5f (angle %.5f), axis off the arc %.4f m, straight heading spread %.6f, end %.3f %.3f", track->name().c_str(),
			                segment.GetLength(), segment.PieceCount(), branch.pieces.size(), arc, geometry::plan_distance(position, expected), turn, angle, worst, high - low, segment.FastGetPoint(1.0).x, segment.FastGetPoint(1.0).z));
		}
		return selftest_result::handled;
	}
	if (command == "switchlinks")
	{
		auto const name_of = [](TTrack const *Track) { return Track == nullptr ? std::string{"-"} : Track->name().empty() ? std::string{"(noname)"} : Track->name(); };
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr)
				continue;
			auto const &extension{*track->SwitchExtension};
			WriteLog(format("SELFTEST switchlinks %s: prev %s / %s, next %s / %s, lengths %.3f / %.3f m", track->name().c_str(), name_of(extension.pPrevs[0]).c_str(), name_of(extension.pPrevs[1]).c_str(), name_of(extension.pNexts[0]).c_str(), name_of(extension.pNexts[1]).c_str(),
			                extension.Segments[0]->GetLength(), extension.Segments[1]->GetLength()));
		}
		return selftest_result::handled;
	}
	if (command == "switchtemplates")
	{
		for (auto const &entry : editor_track::standard_switch_templates())
		{
			if (entry.double_slip)
				continue;
			auto const paths{editor_track::place_switch(entry, glm::dvec3{0.0}, glm::dvec2{0.0, 1.0}, 1, 0.0)};
			double gap{0.0};
			double kink{0.0};
			std::size_t pieces{0};
			for (auto const &path : paths)
			{
				pieces += path.pieces.size();
				for (std::size_t i = 0; i + 1 < path.pieces.size(); ++i)
				{
					auto const &one{path.pieces[i]};
					auto const &two{path.pieces[i + 1]};
					gap = std::max(gap, glm::distance(one.points[segment_data::point::end], two.points[segment_data::point::start]));
					geometry::bezier const a{one};
					geometry::bezier const b{two};
					auto const ta{glm::normalize(geometry::plan_of(a.first(1.0)))};
					auto const tb{glm::normalize(geometry::plan_of(b.first(0.0)))};
					kink = std::max(kink, std::abs(geometry::signed_angle(ta, tb)));
				}
				if (false == path.pieces.empty())
					gap = std::max({gap, glm::distance(path.pieces.front().points[segment_data::point::start], path.points[segment_data::point::start]), glm::distance(path.pieces.back().points[segment_data::point::end], path.points[segment_data::point::end])});
			}
			std::string ends;
			for (auto const &path : paths)
				ends += format(" (%.3f %.3f)", path.points[segment_data::point::end].x, path.points[segment_data::point::end].z);
			std::string starts;
			for (std::size_t i = 1; i < paths.size() && i <= 3; i += 2)
			{
				geometry::bezier const first{paths[i].pieces.empty() ? paths[i] : paths[i].pieces.front()};
				auto const tangent{glm::normalize(geometry::plan_of(first.first(0.0)))};
				starts += format(" %.4f", glm::degrees(std::abs(geometry::signed_angle(glm::dvec2{0.0, 1.0}, tangent))));
			}
			WriteLog(format("SELFTEST switchtemplate %s: %s%s, L %.3f 1:%.3f R %.1f a %.3f b %.3f stagger %.3f, %zu paths, %zu pieces, gap %.6f m, kink %.6f deg, start deg%s, ends%s", entry.label.c_str(), entry.type.c_str(), entry.own ? " own" : "", entry.length, entry.ratio, entry.radius, entry.a, entry.b, entry.stagger, paths.size(), pieces, gap,
			                glm::degrees(kink), starts.c_str(), ends.c_str()));
		}
		return selftest_result::handled;
	}
	if (command == "switches")
	{
		for (auto const &entry : switch_store::records())
		{
			std::string parts;
			for (auto const &part : entry.parts)
				parts += part + " ";
			WriteLog(format("SELFTEST switch record %s %s: %s, R %.3f 1:%.3f hand %c a %.3f b %.3f main R %.3f, branch R %.3f, km %s, number %s, %s, parts %s", entry.kind.c_str(), entry.name.c_str(), switch_store::label(entry).c_str(), entry.radius, entry.ratio, entry.hand, entry.a,
			                entry.b, entry.main_radius, switch_store::branch_radius(entry), entry.km.has_value() ? format("%.3f", *entry.km / 1000.0).c_str() : "-", entry.number.c_str(), entry.source.c_str(), parts.c_str()));
		}
		speed_check::options options;
		options.norms = m_route.design.norms;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->m_paths.size() < 2)
				continue;
			auto const known{switch_store::of(*track)};
			geometry::bezier const branch{track->m_paths[1]};
			auto const first{branch.first(0.3)};
			auto const second{branch.second(0.3)};
			auto const planar{std::abs(first.x * second.z - first.z * second.x) / std::pow(first.x * first.x + first.z * first.z, 1.5)};
			auto const verdict{speed_check::check(*track, 1, options)};
			WriteLog(format("SELFTEST switch %s: %s, branch R at 0.3 %.3f, path R %.3f, branch speed %.1f km/h at R %.1f", track->name().c_str(), known.has_value() ? switch_store::label(*known).c_str() : "no record", planar > 1e-9 ? 1.0 / planar : 0.0,
			                track->m_paths[1].radius, verdict.safe, verdict.radius));
		}
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
