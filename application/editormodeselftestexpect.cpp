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
#include "application/editormodeselftest.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorSettings.hpp"
#include "editor/editorIncludeInfo.hpp"
#include "editor/editorSwitchStore.hpp"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "world/Track.h"
#include "imgui/imgui.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

// the expectations of a script: each logged as SELFTEST PASS <what> or SELFTEST FAIL <what>: got <value>, expected <value>, the failed
// ones counted, so the run ends with an error code at quit (editormodeselftest.cpp)
namespace
{

// the end of another track nearest to the end of a path is taken as the one joined there when it's this near, m
double constexpr kJointReach{1.0};

// the conversion of a roll [deg] to the cant [mm] the inspector shows
using cant_conversion = float (*)(float);

void expectation(bool const Passed, std::string const &What, std::string const &Got, std::string const &Wanted)
{
	++editor_selftest::expectations();
	if (Passed)
	{
		WriteLog("SELFTEST PASS " + What);
		return;
	}
	++editor_selftest::failures();
	WriteLog("SELFTEST FAIL " + What + ": got " + Got + ", expected " + Wanted);
}

void expect_near(std::string const &What, double const Got, double const Wanted, double const Tolerance)
{
	expectation(std::abs(Got - Wanted) <= Tolerance, What, format("%.4f", Got), format("%.4f +- %.4f", Wanted, Tolerance));
}

// the arguments of the expectation weren't all there, or weren't numbers where numbers go: it fails
bool malformed(std::string const &What, std::istringstream const &Words)
{
	if (false == Words.fail())
		return false;
	expectation(false, What, "malformed", "the arguments the expectation takes");
	return true;
}

// an optional number after the arguments: Fallback when there's none. Read apart, as a failed read gives 0
double optional_number(std::istringstream &Words, double const Fallback)
{
	if (double given{0.0}; Words >> given)
		return given;
	return Fallback;
}

// the end of the first path of the track named, as the expectations name it: <track> <start|end>
struct track_end
{
	TTrack const *track{nullptr};
	editor_track::point_ref point;
	std::string error;
};

track_end read_end(std::istringstream &Words)
{
	std::string name;
	std::string side;
	Words >> name >> side;
	track_end result;
	result.track = simulation::Paths.find(name);
	result.point = {0, side == "end" ? editor_track::point_kind::end : editor_track::point_kind::start};
	if (result.track == nullptr || result.track->m_editorremoved || result.track->m_paths.empty())
		result.error = "no track " + name;
	else if (side != "start" && side != "end")
		result.error = "no end " + side;
	return result;
}

bool failed_end(std::string const &What, track_end const &End)
{
	if (End.error.empty())
		return false;
	expectation(false, What, End.error, "the end of a track");
	return true;
}

// the end of a path of another track nearest to a point, within the reach of a joint
struct joined_end
{
	TTrack const *track{nullptr};
	editor_track::point_ref point;
	double distance{kJointReach};

	void take(TTrack const &Track, glm::dvec3 const &Point)
	{
		for (int path = 0; path < static_cast<int>(Track.m_paths.size()); ++path)
			for (auto const kind : {editor_track::point_kind::start, editor_track::point_kind::end})
				if (auto const gap{glm::distance(editor_track::point_position(Track, {path, kind}), Point)}; gap < distance)
				{
					distance = gap;
					track = &Track;
					point = {path, kind};
				}
	}
};

// expect cant <track> <start|end> <mm> <tolerance mm>
void expect_cant(std::string const &What, std::istringstream &Words, cant_conversion const Cant)
{
	auto const end{read_end(Words)};
	double wanted{0.0};
	double tolerance{0.0};
	Words >> wanted >> tolerance;
	if (malformed(What, Words) || failed_end(What, end))
		return;
	auto const &path{end.track->m_paths.front()};
	expect_near(What, Cant(end.point.kind == editor_track::point_kind::start ? path.rolls[0] : path.rolls[1]), wanted, tolerance);
}

// expect joint <track> <start|end> <gap m> <angle deg> <cant step mm>
void expect_joint(std::string const &What, std::istringstream &Words, cant_conversion const Cant)
{
	auto const end{read_end(Words)};
	double gap{0.0};
	double angle{0.0};
	double cant{0.0};
	Words >> gap >> angle >> cant;
	if (malformed(What, Words) || failed_end(What, end))
		return;
	auto const at{editor_track::point_position(*end.track, end.point)};
	joined_end other;
	for (auto const *track : simulation::Paths.sequence())
		if (track != nullptr && track != end.track && false == track->m_editorremoved)
			other.take(*track, at);
	if (other.track == nullptr)
	{
		expectation(false, What, "no end of another track", format("one within %.1f m", kJointReach));
		return;
	}
	auto const out{glm::normalize(editor_track::outward(*end.track, end.point))};
	auto const in{-glm::normalize(editor_track::outward(*other.track, other.point))};
	auto const kink{glm::degrees(std::acos(std::clamp(glm::dot(out, in), -1.0, 1.0)))};
	// the rolls of the two ends are opposite where the joint doesn't step, as editor_track::match_rolls makes them
	auto const own{Cant(static_cast<float>(editor_track::joint_roll(*end.track, end.point)))};
	auto const theirs{Cant(static_cast<float>(editor_track::joint_roll(*other.track, other.point)))};
	auto const step{std::abs(static_cast<double>(own) + static_cast<double>(theirs))};
	auto const side{other.point.kind == editor_track::point_kind::start ? "start" : "end"};
	expectation(other.distance <= gap && kink <= angle && step <= cant, What, format("gap %.4f m, kink %.4f deg, cant step %.1f mm to %s[%d] %s", other.distance, kink, step, other.track->name().c_str(), other.point.path, side),
	            format("at most %.4f m, %.4f deg, %.1f mm", gap, angle, cant));
}

// expect records <name> <count> [kind]
void expect_records(std::string const &What, std::istringstream &Words)
{
	std::string name;
	std::size_t wanted{0};
	Words >> name >> wanted;
	if (malformed(What, Words))
		return;
	std::string kind;
	Words >> kind;
	// the records of the switch, slip or crossing of the name, and the connections having the switch among theirs
	auto const counted = [&name, &kind](switch_store::record const &Entry) {
		auto const among{Entry.kind == "connection" && std::ranges::find(Entry.parts, name) != Entry.parts.end()};
		return (Entry.name == name || among) && (kind.empty() || Entry.kind == kind);
	};
	auto const count{static_cast<std::size_t>(std::ranges::count_if(switch_store::records(), counted))};
	expectation(count == wanted, What, std::to_string(count), std::to_string(wanted));
}

// expect shaped <switch> <yes|no>: the switch keeps the shape its record says, or was bent out of it
void expect_shaped(std::string const &What, std::istringstream &Words)
{
	std::string name;
	std::string wanted;
	Words >> name >> wanted;
	if (malformed(What, Words))
		return;
	auto const *track{simulation::Paths.find(name)};
	if (track == nullptr || track->m_editorremoved)
	{
		expectation(false, What, "no track " + name, "a switch");
		return;
	}
	auto const shaped{switch_store::of(*track).has_value()};
	expectation(shaped == (wanted == "yes"), What, shaped ? "yes" : "no", wanted);
}

// expect speed <track> <km/h> [tolerance], expect gauge <track> <m> [tolerance]
void expect_track_value(std::string const &What, std::string const &Kind, std::istringstream &Words)
{
	std::string name;
	double wanted{0.0};
	Words >> name >> wanted;
	if (malformed(What, Words))
		return;
	auto const speed{Kind == "speed"};
	auto const tolerance{optional_number(Words, speed ? 0.01 : 0.0005)};
	auto const *track{simulation::Paths.find(name)};
	if (track == nullptr || track->m_editorremoved)
	{
		expectation(false, What, "no track " + name, "the track");
		return;
	}
	expect_near(What, speed ? editor_track::velocity(*track) : static_cast<double>(track->fTrackWidth), wanted, tolerance);
}

// expect track <name> <present|removed>
void expect_track(std::string const &What, std::istringstream &Words)
{
	std::string name;
	std::string state;
	Words >> name >> state;
	if (malformed(What, Words))
		return;
	if (state != "present" && state != "removed")
	{
		expectation(false, What, "no state " + state, "present or removed");
		return;
	}
	auto const *track{simulation::Paths.find(name)};
	auto const present{track != nullptr && false == track->m_editorremoved};
	expectation(present == (state == "present"), What, present ? "present" : "removed", state);
}

// expect route <track> <in|out>
void expect_route(std::string const &What, std::istringstream &Words, std::vector<TTrack *> const &Line)
{
	std::string name;
	std::string state;
	Words >> name >> state;
	if (malformed(What, Words))
		return;
	if (state != "in" && state != "out")
	{
		expectation(false, What, "no state " + state, "in or out");
		return;
	}
	auto const *track{simulation::Paths.find(name)};
	auto const in{track != nullptr && std::ranges::find(Line, track) != Line.end()};
	expectation(in == (state == "in"), What, in ? "in" : "out", state);
}

// expect norms gauge <mm> [tolerance]
void expect_norm(std::string const &What, std::istringstream &Words, alignment::limits const &Norms)
{
	std::string key;
	double wanted{0.0};
	Words >> key >> wanted;
	if (malformed(What, Words))
		return;
	if (key != "gauge")
	{
		expectation(false, What, "no norm " + key, "gauge");
		return;
	}
	expect_near(What, Norms.gauge, wanted, optional_number(Words, 0.5));
}

// expect findings <note|warning|block|any> <least> <most>: how many the last check of the norms found of the tier
void expect_findings(std::string const &What, std::istringstream &Words, std::vector<norms_report::entry> const &Findings)
{
	std::string tier;
	int least{0};
	int most{0};
	Words >> tier >> least >> most;
	if (malformed(What, Words))
		return;
	std::map<std::string, norms::tier, std::less<>> const tiers{{"note", norms::tier::prefer}, {"warning", norms::tier::warn}, {"block", norms::tier::block}};
	auto const wanted{tiers.find(tier)};
	if (tier != "any" && wanted == tiers.end())
	{
		expectation(false, What, "no tier " + tier, "note, warning, block or any");
		return;
	}
	auto const count{std::ranges::count_if(Findings, [&](norms_report::entry const &Entry) { return tier == "any" || Entry.finding.level == wanted->second; })};
	expectation(count >= least && count <= most, What, std::to_string(count), format("%d to %d", least, most));
}

// expect height <track> <start|end> <m> <tolerance>, expect lift <track> <start|end> <label> <m> <tolerance>
void expect_height(std::string const &What, std::string const &Kind, std::istringstream &Words, std::map<std::string, double> const &Marks)
{
	auto const end{read_end(Words)};
	std::string label;
	if (Kind == "lift")
		Words >> label;
	double wanted{0.0};
	double tolerance{0.0};
	Words >> wanted >> tolerance;
	if (malformed(What, Words) || failed_end(What, end))
		return;
	auto height{editor_track::point_position(*end.track, end.point).y};
	if (Kind == "lift")
	{
		auto const mark{Marks.find(label)};
		if (mark == Marks.end())
		{
			expectation(false, What, "no mark " + label, "a height marked");
			return;
		}
		height -= mark->second;
	}
	expect_near(What, height, wanted, tolerance);
}

// the include is in the scenery, not deleted, and the name is among the values of its parameters: the model and the events of a
// turntable name its track so
bool names_track(scene::instance_handle const Handle, std::string const &Name)
{
	if (false == scene::Layers.tracked(Handle))
		return false;
	auto const &included{scene::Layers.instance(Handle)};
	if (included.removed || included.dead || included.file == nullptr)
		return false;
	std::string file;
	std::vector<std::string> values;
	return editor_includes::parse_directive(scene::Layers.directive(Handle), file, values) && std::ranges::any_of(values, [&Name](std::string const &Value) { return ToLower(Value) == Name; });
}

// expect includes <track> <count>
void expect_includes(std::string const &What, std::istringstream &Words)
{
	std::string name;
	std::size_t wanted{0};
	Words >> name >> wanted;
	if (malformed(What, Words))
		return;
	std::size_t count{0};
	for (scene::instance_handle handle = 1; handle <= scene::Layers.instance_count(); ++handle)
		if (names_track(handle, name))
			++count;
	expectation(count == wanted, What, std::to_string(count), std::to_string(wanted));
}

// expect status <text>: the status the switch tool shows in its panel holds the text, the rest of the line
void expect_status(std::string const &What, std::istringstream &Words, std::string const &Status)
{
	std::string wanted;
	std::getline(Words >> std::ws, wanted);
	if (wanted.empty())
	{
		expectation(false, What, "malformed", "the text the status holds");
		return;
	}
	expectation(Status.find(wanted) != std::string::npos, What, "\"" + Status + "\"", "a status holding \"" + wanted + "\"");
}

// expect statuslacks <text>: the status of the switch tool doesn't hold the text, the rest of the line
void expect_status_lacks(std::string const &What, std::istringstream &Words, std::string const &Status)
{
	std::string unwanted;
	std::getline(Words >> std::ws, unwanted);
	if (unwanted.empty())
	{
		expectation(false, What, "malformed", "the text the status lacks");
		return;
	}
	expectation(Status.find(unwanted) == std::string::npos, What, "\"" + Status + "\"", "a status without \"" + unwanted + "\"");
}

// expect refusal <holds|lacks> <text>, expect statusline <holds|lacks> <text>: what the cursor tells while it's shown, or the status
// line of the editor, holds the text, the rest of the line, or doesn't
void expect_told(std::string const &What, std::istringstream &Words, std::string const &Told)
{
	std::string state;
	std::string text;
	Words >> state;
	std::getline(Words >> std::ws, text);
	if ((state != "holds" && state != "lacks") || text.empty())
	{
		expectation(false, What, "malformed", "holds or lacks and the text");
		return;
	}
	auto const holds{Told.find(text) != std::string::npos};
	expectation(holds == (state == "holds"), What, "\"" + Told + "\"", (state == "holds" ? "a text holding \"" : "a text without \"") + text + "\"");
}

// expect laycursor <x> <z> <tolerance>: the point the line of the lay tool goes to from its last point clicked, in the plan
void expect_lay_cursor(std::string const &What, std::istringstream &Words, glm::dvec3 const &Mouse)
{
	double x{0.0};
	double z{0.0};
	double tolerance{0.0};
	Words >> x >> z >> tolerance;
	if (malformed(What, Words))
		return;
	auto const distance{glm::length(glm::dvec2{Mouse.x - x, Mouse.z - z})};
	expectation(distance <= tolerance, What, format("%.3f %.3f", Mouse.x, Mouse.z), format("%.3f %.3f within %.3f", x, z, tolerance));
}

// expect terrain <grow|area|prompt> <0|1>: the terrain model made under the tracks as they're laid, or around the camera within the
// download area, the question of the terrain of a new scenery open
void expect_terrain(std::string const &What, std::istringstream &Words, std::map<std::string, bool> const &States)
{
	std::string key;
	int wanted{0};
	Words >> key >> wanted;
	if (malformed(What, Words))
		return;
	auto const state{States.find(key)};
	if (state == States.end())
	{
		std::string keys;
		for (auto const &[name, value] : States)
			keys += (keys.empty() ? "" : ", ") + name;
		expectation(false, What, "no state " + key, keys);
		return;
	}
	expectation(state->second == (wanted != 0), What, state->second ? "1" : "0", std::to_string(wanted));
}

// the name of the tool of the surroundings in use, as the expectation names it
std::string tool_name(nodebank_panel::edit_mode const Mode)
{
	switch (Mode)
	{
	case nodebank_panel::COPY: return "copy";
	case nodebank_panel::ADD: return "insert";
	case nodebank_panel::BRUSH: return "brush";
	case nodebank_panel::FILL: return "fill";
	case nodebank_panel::TRACK: return "track";
	default: return "select";
	}
}

// expect tool <name>: the tool of the surroundings in use, or track while the field of work of the tracks is
void expect_tool(std::string const &What, std::istringstream &Words, std::string const &Tool)
{
	std::string wanted;
	Words >> wanted;
	if (malformed(What, Words))
		return;
	expectation(Tool == wanted, What, Tool, wanted);
}

// expect window <name> <0|1>: a window of the editor open: shortcuts, settings, weather, orthophoto
void expect_window(std::string const &What, std::istringstream &Words, std::map<std::string, bool> const &Windows)
{
	std::string name;
	int wanted{0};
	Words >> name >> wanted;
	if (malformed(What, Words))
		return;
	auto const window{Windows.find(name)};
	if (window == Windows.end())
	{
		expectation(false, What, "no window " + name, "shortcuts, settings, weather or orthophoto");
		return;
	}
	expectation(window->second == (wanted != 0), What, window->second ? "1" : "0", std::to_string(wanted));
}

// expect selected <x> <y> <z> <tolerance>: the node selected stands at the point
void expect_selected(std::string const &What, std::istringstream &Words, std::optional<glm::dvec3> const &Selected)
{
	glm::dvec3 wanted{0.0};
	double tolerance{0.0};
	Words >> wanted.x >> wanted.y >> wanted.z >> tolerance;
	if (malformed(What, Words))
		return;
	if (false == Selected.has_value())
	{
		expectation(false, What, "nothing selected", format("%.3f %.3f %.3f", wanted.x, wanted.y, wanted.z));
		return;
	}
	expectation(glm::distance(*Selected, wanted) <= tolerance, What, format("%.3f %.3f %.3f", Selected->x, Selected->y, Selected->z), format("%.3f %.3f %.3f within %.3f", wanted.x, wanted.y, wanted.z, tolerance));
}

// expect laypanel <state> <shown|folded>, expect switchpanel <state>: the state the panel of the tool is in, as the step at its top
// tells it, and whether the settings of the lay tool are shown or folded to the line of their summary
void expect_panel(std::string const &What, std::istringstream &Words, std::string const &Got)
{
	std::string wanted;
	for (std::string word; Words >> word;)
		wanted += (wanted.empty() ? "" : " ") + word;
	if (wanted.empty())
	{
		expectation(false, What, "malformed", "the state of the panel");
		return;
	}
	expectation(Got == wanted, What, Got, wanted);
}

// what the editor shows, for the expectations of what the user sees
struct shown_state
{
	std::string status; // of the switch tool
	std::string refusal; // by the cursor, while it's shown
	std::string status_line;
	glm::dvec3 lay_cursor{0.0};
	std::map<std::string, bool> terrain; // grow, area, prompt, loading, question
	std::map<std::string, bool> photo; // on, chosen, zoom4k, tracksover
	std::string tool;
	std::map<std::string, bool> windows; // shortcuts, settings, weather, orthophoto
	std::optional<glm::dvec3> selected; // where the node selected stands
	std::string lay_panel; // the state of the panel of the lay tool and whether its settings are shown
	std::string switch_panel; // the state of the panel of the switch tool
};

// the expectations of what the editor shows; false: no expectation of the kind
bool expect_shown(std::string const &Kind, std::string const &What, std::istringstream &Words, shown_state const &Shown)
{
	if (Kind == "statuslacks")
		expect_status_lacks(What, Words, Shown.status);
	else if (Kind == "refusal")
		expect_told(What, Words, Shown.refusal);
	else if (Kind == "statusline")
		expect_told(What, Words, Shown.status_line);
	else if (Kind == "laycursor")
		expect_lay_cursor(What, Words, Shown.lay_cursor);
	else if (Kind == "terrain")
		expect_terrain(What, Words, Shown.terrain);
	else if (Kind == "photo")
		expect_terrain(What, Words, Shown.photo);
	else if (Kind == "tool")
		expect_tool(What, Words, Shown.tool);
	else if (Kind == "window")
		expect_window(What, Words, Shown.windows);
	else if (Kind == "selected")
		expect_selected(What, Words, Shown.selected);
	else if (Kind == "laypanel")
		expect_panel(What, Words, Shown.lay_panel);
	else if (Kind == "switchpanel")
		expect_panel(What, Words, Shown.switch_panel);
	else
		return false;
	return true;
}

// expect steps <count>: the steps Ctrl+Z takes back, the ones made together counted once
void expect_steps(std::string const &What, std::istringstream &Words, std::size_t const Steps)
{
	std::size_t wanted{0};
	Words >> wanted;
	if (malformed(What, Words))
		return;
	expectation(Steps == wanted, What, std::to_string(Steps), std::to_string(wanted));
}

} // namespace

editor_mode::selftest_result editor_mode::selftest_expect(std::string const &command, std::istringstream &words)
{
	if (command == "mark")
	{
		std::string label;
		words >> label;
		auto const end{read_end(words)};
		if (failed_end("mark " + label, end))
			return selftest_result::handled;
		auto const height{editor_track::point_position(*end.track, end.point).y};
		m_selftest.marks[label] = height;
		WriteLog(format("SELFTEST mark %s: %.4f m", label.c_str(), height));
		return selftest_result::handled;
	}
	if (command != "expect")
		return selftest_result::unhandled;
	// what is expected as the script says it, for the log
	auto what{words.str()};
	what.erase(0, std::min(what.size(), what.find("expect") + 6));
	what.erase(0, std::min(what.size(), what.find_first_not_of(' ')));
	std::string kind;
	words >> kind;
	if (kind == "cant")
		expect_cant(what, words, &editor_mode::cant_of_roll);
	else if (kind == "joint")
		expect_joint(what, words, &editor_mode::cant_of_roll);
	else if (kind == "records")
		expect_records(what, words);
	else if (kind == "shaped")
		expect_shaped(what, words);
	else if (kind == "speed" || kind == "gauge")
		expect_track_value(what, kind, words);
	else if (kind == "track")
		expect_track(what, words);
	else if (kind == "route")
		expect_route(what, words, m_route.chain.tracks);
	else if (kind == "norms")
		expect_norm(what, words, laying_norms());
	else if (kind == "height" || kind == "lift")
		expect_height(what, kind, words, m_selftest.marks);
	else if (kind == "includes")
		expect_includes(what, words);
	else if (kind == "status")
		expect_status(what, words, m_switch.status);
	else if (kind == "findings")
		expect_findings(what, words, m_norms.results());
	else if (kind == "steps")
		expect_steps(what, words, static_cast<std::size_t>(std::ranges::count_if(m_history, [](EditorSnapshot const &Step) { return false == Step.joined; })));
	else
	{
		shown_state shown{m_switch.status, ui()->refusal_shown(), ui()->status_line(), m_lay.mouse,
		                  {{"grow", m_nmt_state.grow},
		                   {"area", nmt_area()},
		                   {"prompt", m_nmt_state.prompt},
		                   {"loading", m_nmt_state.offer_loading},
		                   {"question", ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId)}},
		                  {{"on", m_orthophoto.enabled()}, {"chosen", m_orthophoto_chosen}, {"zoom4k", m_orthophoto.zoom_hires()}, {"tracksover", m_track_lines_drawn > 0}},
		                  tool_name(ui()->mode()),
		                  {{"shortcuts", m_shortcuts_open}, {"settings", m_settings_open}, {"weather", m_environment_open}, {"orthophoto", m_orthophoto_window}}, std::nullopt,
		                  shown_lay_panel(), shown_switch_panel()};
		if (m_node != nullptr)
			shown.selected = m_node->location();
		if (false == expect_shown(kind, what, words, shown))
			expectation(false, what, "no expectation " + kind,
			            "cant, joint, records, shaped, speed, gauge, track, route, norms, findings, height, lift, includes, status, statuslacks, refusal, statusline, laycursor, terrain, tool, window, "
			            "selected, laypanel, switchpanel or steps");
	}
	return selftest_result::handled;
}

std::string editor_mode::shown_lay_panel() const
{
	static char const *const states[]{"ready", "clicking", "laid", "extending"};
	auto const panel{lay_panel_state()};
	// the settings are shown before the first point and after the line is laid, folded to their line while the points are clicked
	auto const shown{panel == lay_state::ready || panel == lay_state::laid || m_lay.settings_open};
	return std::string{states[static_cast<int>(panel)]} + (shown ? " shown" : " folded");
}

std::string editor_mode::shown_switch_panel() const
{
	if (m_switch.placing)
		return "dragging";
	auto const *track{selected_track()};
	if (track == nullptr || track->eType != tt_Switch)
		return "ready";
	return just_laid(track) ? "placed" : "selected";
}
