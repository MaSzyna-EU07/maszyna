/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// The track geometry edited through the model (editor-tracks-domain.md §3, §4, §8, §9, §10; stage E4), with the setting "Editing on the
// model (trial)" on. The project of the map is opened with the scenery (made of its //$ lines the first time) and written with it; a line
// laid by the lay tool in the straights, or a line of the model the route design is applied to, goes through the commands of the model,
// its tracks made by the generator and marked as the line's; a refused command changes nothing and tells why, one kept is one step of the
// history; a line whose tracks were changed by hand is written over only when the user says so; a track the scenarios of the map name goes
// only when the user says so. The old tracks stay as they are, the old tools edit them.

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodemodel.h"
#include "application/editormodeselftest.h"
#include "application/editormodeshared.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorSettings.hpp"
#include "editor/project/projectGenerated.hpp"
#include "editor/project/projectSession.hpp"
#include "editor/project/scenarioIndexCache.hpp"
#include "editor/project/sceneryText.hpp"
#include "editor/layout/layoutTurnout.hpp"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"
#include "world/Track.h"
#include "application/scriptedinput.h"

#include "imgui/imgui.h"

#include <algorithm>
#include <bit>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <regex>
#include <sstream>

namespace
{

std::string line_name(layout::line_id const Line)
{
	return format("L%u", layout::number(Line));
}

// the text of the track as the scenery save writes it: its own export
std::string export_of(TTrack const &Track)
{
	std::string text;
	Track.export_as_text(text);
	text.erase(text.find_last_not_of(" \t\r\n") + 1);
	return text;
}

std::vector<std::string> words_of(std::string const &Text)
{
	std::istringstream stream{Text};
	std::vector<std::string> result;
	for (std::string word; stream >> word;)
		result.push_back(std::move(word));
	return result;
}

// what the entry of the track has between its length and its path: the width, the friction, the sound distance, the quality, the damage,
// the environment and the visibility with the materials; empty where it isn't a plain track
std::string surface_of(TTrack const &Track)
{
	auto const words{words_of(export_of(Track))};
	// node <range> <range> <name> track normal <length> <width> ... <environment> <vis|unvis> [<rail> <length> <ballast> <height> <width> <slope>]
	if (words.size() < 14 || words[4] != "track" || words[5] != "normal")
		return {};
	auto const count{words[13] == "vis" ? std::size_t{13} : std::size_t{7}};
	if (words.size() < 7 + count)
		return {};
	std::string result;
	for (std::size_t i = 7; i < 7 + count; ++i)
		result += (result.empty() ? "" : " ") + words[i];
	return result;
}

bool bits_equal(glm::dvec3 const &A, glm::dvec3 const &B)
{
	return std::bit_cast<std::uint64_t>(A.x) == std::bit_cast<std::uint64_t>(B.x) && std::bit_cast<std::uint64_t>(A.y) == std::bit_cast<std::uint64_t>(B.y) && std::bit_cast<std::uint64_t>(A.z) == std::bit_cast<std::uint64_t>(B.z);
}

// the setting as a self-test sets it for its run, not kept in the settings of the editor; else the one of the settings
std::optional<bool> &run_setting()
{
	static std::optional<bool> value;
	return value;
}

bool model_setting()
{
	return run_setting().value_or(EditorSettings.model_editing());
}

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

} // namespace

std::string model_surface_of(TTrack const &Track)
{
	return surface_of(Track);
}

void editor_model_deleter::operator()(editor_model *Model) const
{
	delete Model;
}

editor_model &editor_mode::model() const
{
	if (m_model == nullptr)
		m_model.reset(new editor_model);
	return *m_model;
}

bool editor_mode::model_refused() const
{
	return model().refused;
}

std::string editor_mode::model_message() const
{
	return model().message;
}

std::vector<std::string> editor_mode::model_notices() const
{
	return model().shown_notices;
}

std::string editor_mode::model_error() const
{
	return model().error;
}

layout::network const &editor_mode::model_network() const
{
	return model().session.network();
}

std::string editor_mode::model_shown() const
{
	return model().shown;
}

namespace
{

// the format of the catalogue of the editor with {1}, {2}... put in its place: the words of the message the model made
std::string filled(std::string Format, std::vector<std::string> const &Arguments)
{
	for (std::size_t i = 0; i < Arguments.size(); ++i)
	{
		auto const mark{"{" + std::to_string(i + 1) + "}"};
		for (auto at{Format.find(mark)}; at != std::string::npos; at = Format.find(mark, at + Arguments[i].size()))
			Format.replace(at, mark.size(), Arguments[i]);
	}
	return Format;
}

// a message of the model as the editor shows it: the English of the model matched against its forms, the form of the catalogue of the
// editor (STR) filled with the numbers and the names; the parts the forms leave as {sub} shown so in turn
struct message_form
{
	std::regex pattern;
	char const *format; // with {1}, {2}... for the groups of the pattern
	std::vector<bool> nested; // the group is a message of its own, shown so
};

std::vector<message_form> const &message_forms()
{
	static std::vector<message_form> const forms{
	    // the norms: a BLOCK limit along a line, a finding
	    {std::regex{R"(^line (\d+) at ([-\d.]+) m: (.+)$)"}, STRN("line {1} at {2} m: {3}"), {false, false, true}},
	    {std::regex{R"(^(.+?) ([-\d.e+]+), the limit ([-\d.e+]+) \((.+)\)$)"}, STRN("{1} {2}, the limit {3} ({4})"), {true, false, false, false}},
	    {std::regex{R"(^(.+?) ([-\d.e+]+), the limit ([-\d.e+]+)$)"}, STRN("{1} {2}, the limit {3}"), {true, false, false}},
	    // the turnouts
	    {std::regex{R"(^turnout (\d+) \((.+?)\): (.+)$)"}, STRN("turnout {1} ({2}): {3}"), {false, false, true}},
	    {std::regex{R"(^bent into R (\d+) m in a main track, where no switch of 1:9\.403 nor steeper than 1:9 is bent \(11\.4 pkt 4\)$)"},
	     STRN("bent into R {1} m in a main track, where no switch of 1:9.403 nor steeper than 1:9 is bent (11.4 pkt 4)"), {false}},
	    {std::regex{R"(^(.+) is ([\d.]+) m long: it doesn't fit into line (\d+) \(([\d.]+) m long\) with its start at ([-\d.]+) m$)"},
	     STRN("{1} is {2} m long: it doesn't fit into line {3} ({4} m long) with its start at {5} m"), {false, false, false, false, false}},
	    {std::regex{R"(^line (\d+) is a track of turnout (\d+): move it, change it or take it out$)"}, STRN("line {1} is a track of turnout {2}: move it, change it or take it out"), {false, false}},
	    // the crossovers
	    {std::regex{R"(^the second turnout doesn't fit into line (\d+) at ([-\d.]+) m$)"}, STRN("the second turnout doesn't fit into line {1} at {2} m"), {false, false}},
	    {std::regex{R"(^the line between the turnouts is ([\d.]+) m long, the norms ask for ([\d.]+) m between two turnouts at ([\d.]+) km/h \(ST-T1-A6 11\.3 pkt 7\)$)"},
	     STRN("the line between the turnouts is {1} m long, the norms ask for {2} m between two turnouts at {3} km/h (ST-T1-A6 11.3 pkt 7)"), {false, false, false}},
	    {std::regex{R"(^the lines are ([\d.]+) m apart along the crossover \(to ([\d.]+) m\), (.+) needs ([\d.]+) m \(ST-T1-A6 tab\. 11\.2 and the straight between its turnouts\)$)"},
	     STRN("the lines are {1} m apart along the crossover (to {2} m), {3} needs {4} m (ST-T1-A6 tab. 11.2 and the straight between its turnouts)"), {false, false, false, false}},
	    {std::regex{R"(^no ordinary turnout of the catalogue fits lines ([\d.]+) m apart; (.+) needs them ([\d.]+) m apart$)"},
	     STRN("no ordinary turnout of the catalogue fits lines {1} m apart; {2} needs them {3} m apart"), {false, false, false}},
	    {std::regex{R"(^(.+) fits lines ([\d.]+) m apart, at ([\d.]+) km/h; (.+) needs them ([\d.]+) m apart$)"}, STRN("{1} fits lines {2} m apart, at {3} km/h; {4} needs them {5} m apart"),
	     {false, false, false, false, false}},
	    {std::regex{R"(^proposed: (.+)$)"}, STRN("proposed: {1}"), {true}},
	    // the lines and the nodes
	    {std::regex{R"(^line (\d+): (.+)$)"}, STRN("line {1}: {2}"), {false, true}},
	    {std::regex{R"(^vertex (\d+): (.+)$)"}, STRN("vertex {1}: {2}"), {false, true}},
	    {std::regex{R"(^([-\d.]+) m is inside a transition curve whose parts wouldn't be curves of its kind; split on a straight or an arc$)"},
	     STRN("{1} m is inside a transition curve whose parts wouldn't be curves of its kind; split on a straight or an arc"), {false}},
	    {std::regex{R"(^([-\d.]+) m is inside the vertical curve at ([-\d.]+) m$)"}, STRN("{1} m is inside the vertical curve at {2} m"), {false, false}},
	    {std::regex{R"(^line (\d+) meets node (\d+) at an angle of ([\d.]+) deg$)"}, STRN("line {1} meets node {2} at an angle of {3} deg"), {false, false, false}},
	    {std::regex{R"(^the grade breaks by ([\d.]+) per mille at node (\d+) with no vertical curve$)"}, STRN("the grade breaks by {1} per mille at node {2} with no vertical curve"), {false, false}},
	    {std::regex{R"(^the cant steps by ([\d.]+) mm at node (\d+) where line (\d+) meets it$)"}, STRN("the cant steps by {1} mm at node {2} where line {3} meets it"), {false, false, false}},
	    {std::regex{R"(^the grade break at ([-\d.]+) m isn't between the one before it and the end of the line, ([\d.]+) m long$)"},
	     STRN("the grade break at {1} m isn't between the one before it and the end of the line, {2} m long"), {false, false}},
	    {std::regex{R"(^the vertical curves at ([-\d.]+) m and ([-\d.]+) m overlap by ([\d.]+) m$)"}, STRN("the vertical curves at {1} m and {2} m overlap by {3} m"), {false, false, false}},
	    {std::regex{R"(^the curves of vertex (\d+) and vertex (\d+) overlap by ([\d.]+) m$)"}, STRN("the curves of vertex {1} and vertex {2} overlap by {3} m"), {false, false, false}},
	    {std::regex{R"(^the transition curves turn ([\d.]+) rad, more than the ([\d.]+) rad the tangents do$)"}, STRN("the transition curves turn {1} rad, more than the {2} rad the tangents do"),
	     {false, false}},
	    {std::regex{R"(^node (\d+) isn't the free end of a line$)"}, STRN("node {1} isn't the free end of a line"), {false}},
	    // of the editor's own
	    {std::regex{R"(^the end of the line at ([-\d.]+) m, its node at ([-\d.]+) m: a line of the model keeps the heights of its nodes$)"},
	     STRN("the end of the line at {1} m, its node at {2} m: a line of the model keeps the heights of its nodes"), {false, false}},
	    {std::regex{R"(^the spacing in the curve at vertex (\d+) isn't widened by (\d+) mm: its transitions can't take it$)"},
	     STRN("the spacing in the curve at vertex {1} isn't widened by {2} mm: its transitions can't take it"), {false, false}},
	    {std::regex{R"(^the curve at vertex (\d+) would be of a radius of ([-\d.]+) m alongside$)"}, STRN("the curve at vertex {1} would be of a radius of {2} m alongside"), {false, false}},
	    {std::regex{R"(^a spacing of ([\d.]+) m$)"}, STRN("a spacing of {1} m"), {false}},
	};
	return forms;
}

} // namespace

std::string editor_mode::model_tr(std::string const &Text)
{
	if (Text.empty())
		return Text;
	// a whole message the catalogue of the editor has
	if (auto const whole{STR(Text)}; whole != Text)
		return whole;
	for (auto const &form : message_forms())
	{
		std::smatch found;
		if (false == std::regex_match(Text, found, form.pattern))
			continue;
		std::vector<std::string> arguments;
		for (std::size_t i = 1; i < found.size(); ++i)
			arguments.push_back(i - 1 < form.nested.size() && form.nested[i - 1] ? model_tr(found[i].str()) : found[i].str());
		return filled(STR(form.format), arguments);
	}
	return Text;
}

std::string editor_mode::model_reasons_shown(layout::outcome const &Outcome)
{
	std::string result;
	for (auto const &reason : Outcome.reasons)
		result += (result.empty() ? "" : "\n") + model_tr(reason);
	return result;
}

std::string editor_mode::model_notice_shown(layout::notice const &Notice)
{
	return model_tr(Notice.text);
}

void editor_mode::model_forget(scene::basic_node const *Node)
{
	if (m_model != nullptr)
		m_model->tracks.erase(Node);
}

void editor_mode::model_plan_drop()
{
	model().plan = {};
}

bool editor_mode::model_on() const
{
	return model_setting() && model().session.is_open();
}

void editor_mode::model_update()
{
	if (false == model_setting())
		return;
	if (false == model().tried && Global.editor_session && false == scene::Layers.empty())
		model_open();
	if (false == model_on())
		return;
	// the index built is taken, and kept between the sessions
	auto const index{model().index.ready()};
	auto &question{model().question};
	if (question.what != model_question::kind::waiting || index == nullptr)
		return;
	question.users = project::users_of(*index, question.names);
	WriteLog(format("Editor: the scenarios of the map looked through, %zu of %zu name the tracks %s", question.users.size(), index->scenarios.size(), question.names.empty() ? "" : question.names.front().c_str()));
	if (false == question.users.empty())
	{
		question.what = model_question::kind::scenarios;
		question.opened = false;
		return;
	}
	auto const answer{std::move(question.answer)};
	question = {};
	if (answer)
		answer(true);
}

void editor_mode::model_open()
{
	model().tried = true;
	model().error.clear();
	auto const map{scene::Layers.layer(scene::layer_handle{1}).name};
	std::filesystem::path const folder{paths::scenery};
	auto opened{model().session.open(folder, map)};
	if (false == opened.done)
	{
		model().error = opened.error;
		ui()->set_status(format(STR_C("The project of the map can't be opened: %s. The tracks are edited the old way"), opened.error.c_str()), true);
		WriteLog("Editor: the project of the map " + map + " can't be opened: " + opened.error, logtype::generic);
		return;
	}
	WriteLog(format("Editor: the project of the map %s %s, %zu lines of the model", map.c_str(), opened.migrated ? "made of its //$ lines" : "read", model().session.network().lines().size()), logtype::generic);
	for (auto const &note : opened.notes)
		WriteLog("Editor: project: " + note, logtype::generic);
	for (auto const &change : opened.hand_changes)
		WriteLog(format("Editor: project: the track %s of line %s in %s %s outside the editor", change.name.c_str(), line_name(change.line).c_str(), change.file.c_str(), change.missing ? "is gone" : "was changed"), logtype::generic);
	// the tracks the generator wrote, found in the scene by their names; written again as the files have them while nothing changes them
	model().tracks.clear();
	std::map<std::string, std::optional<std::string>> files;
	std::map<layout::line_id, std::size_t> counts;
	for (auto const &node : model().session.project().generated)
	{
		if (node.kind != "track" && node.kind != "switch")
			continue;
		auto *track{simulation::Paths.find(node.name)};
		if (track == nullptr || track->m_editorremoved)
			continue;
		auto found{files.find(node.file)};
		if (found == files.end())
		{
			std::ifstream stream{folder / node.file, std::ios::binary};
			found = files.emplace(node.file, stream.is_open() ? std::optional<std::string>{std::string{std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()}} : std::nullopt).first;
		}
		managed_track entry;
		entry.line = node.line;
		entry.index = counts[node.line]++;
		entry.kind = node.kind;
		if (found->second.has_value())
			if (auto const text{scenery_text::node_text(*found->second, node.name)})
				entry.text = *text;
		if (auto const *source{scene::Layers.source(track)})
			entry.offset = source->context.offset;
		entry.baseline = export_of(*track);
		model().tracks[track] = std::move(entry);
	}
	scene::Layers.path_writer = [this](TTrack const &Track, glm::dvec3 const &Offset) { return model_path_text(Track, Offset); };
	// the scenario index of the map built in the background, kept between the sessions (not for the self-tests, which leave nothing behind)
	auto &store{model().store};
	store.clear();
	if (selftest_script("EU07_EDITOR_SELFTEST") == nullptr)
	{
		auto name{map};
		std::replace_if(name.begin(), name.end(), [](char const C) { return C == '/' || C == '\\' || C == ':'; }, '_');
		store = user_cache_path("scenario_index/" + name + ".txt");
	}
	model().index.request(folder, map, store);
	ui()->set_status(format(STR_C("Editing on the model: the project of the map is %s, %zu lines of the model"), project::path_of(map).generic_string().c_str(), model().session.network().lines().size()));
}

editor_mode::managed_track const *editor_mode::model_track(TTrack const *Track) const
{
	if (Track == nullptr || false == model_on())
		return nullptr;
	auto const found{model().tracks.find(Track)};
	if (found == model().tracks.end() || model().session.network().find(found->second.line) == nullptr)
		return nullptr;
	return &found->second;
}

std::vector<TTrack *> editor_mode::model_line_tracks(layout::line_id const Line) const
{
	std::vector<std::pair<std::size_t, TTrack *>> found;
	for (auto const &[node, entry] : model().tracks)
	{
		auto *track{const_cast<TTrack *>(dynamic_cast<TTrack const *>(node))};
		if (entry.line == Line && track != nullptr && false == track->m_editorremoved)
			found.emplace_back(entry.index, track);
	}
	std::ranges::sort(found, [](auto const &A, auto const &B) { return A.first < B.first; });
	std::vector<TTrack *> result;
	for (auto const &entry : found)
		result.push_back(entry.second);
	return result;
}

std::set<layout::line_id> editor_mode::model_seen_changed() const
{
	std::set<layout::line_id> result;
	for (auto const &[node, entry] : model().tracks)
	{
		auto const *track{dynamic_cast<TTrack const *>(node)};
		if (track != nullptr && false == track->m_editorremoved && false == result.contains(entry.line) && export_of(*track) != entry.baseline)
			result.insert(entry.line);
	}
	return result;
}

std::string editor_mode::model_label(TTrack const *Track) const
{
	auto const *managed{model_track(Track)};
	if (managed == nullptr)
		return {};
	return format(STR_C("Line %s (managed)"), line_name(managed->line).c_str());
}

void editor_mode::model_told(layout::outcome const &Outcome, bool const Refused, std::string const &What)
{
	model().refused = Refused;
	model().notices.clear();
	model().shown_notices.clear();
	model().proposal.clear();
	if (Refused)
	{
		model().message = "Refused, nothing changed:";
		for (auto const &reason : Outcome.reasons)
			model().message += "\n" + reason;
		model().shown = STR("Refused, nothing changed:") + "\n" + model_reasons_shown(Outcome);
		WriteLog("Editor: model: " + editor_selftest::one_line(model().message), logtype::generic);
		return;
	}
	model().message = What;
	model().shown = What;
	for (auto const &notice : Outcome.notices)
		if (std::ranges::find(model().notices, notice.text) == model().notices.end())
		{
			model().notices.push_back(notice.text);
			model().shown_notices.push_back(model_notice_shown(notice));
		}
	WriteLog("Editor: model: " + What + (model().notices.empty() ? std::string{} : " · " + std::to_string(model().notices.size()) + " notices of the norms"), logtype::generic);
}

void editor_mode::model_run(std::vector<layout::command> Commands, model_context Context)
{
	std::vector<TTrack *> created;
	auto &session{model().session};
	while (false == Commands.empty())
	{
		auto const &command{Commands.front()};
		auto const effect{session.trial(command)};
		if (false == effect.outcome.done)
		{
			model_told(effect.outcome, true);
			return;
		}
		// a line changed by hand is written over only when the user says so
		auto hand{session.hand_changed(effect, model_seen_changed())};
		std::erase_if(hand, [&](layout::line_id const Line) { return Context.overwrite.contains(Line); });
		if (false == hand.empty())
		{
			auto &question{model().question};
			question = {};
			question.what = model_question::kind::hand;
			question.lines = hand;
			question.answer = [this, Commands, Context, hand](bool const Overwrite) mutable {
				if (false == Overwrite)
				{
					layout::outcome kept;
					kept.reasons.push_back(STR("The change by hand is kept: the line isn't laid anew"));
					model_told(kept, true);
					return;
				}
				Context.overwrite.insert(hand.begin(), hand.end());
				model_run(std::move(Commands), std::move(Context));
			};
			WriteLog(format("Editor: model: the line %s was changed by hand, the user asked", line_name(*hand.begin()).c_str()), logtype::generic);
			return;
		}
		std::set<layout::line_id> overwrite;
		for (auto const *lines : {&effect.changed, &effect.removed})
			for (auto const line : *lines)
				if (Context.overwrite.contains(line))
					overwrite.insert(line);
		auto const before{session.network()};
		auto const applied{session.execute(command, overwrite)};
		if (false == applied.outcome.done)
		{
			model_told(applied.outcome, true);
			return;
		}
		created = model_tracks_after(applied.token, before, applied.changed, applied.removed, Context);
		std::string what{format(STR_C("%s: done"), STR_C(layout::command_name(command)))};
		if (false == applied.changed.empty())
			what = format(STR_P("%s: line %s, %zu track", "%s: line %s, %zu tracks", static_cast<long long>(created.size())), STR_C(layout::command_name(command)), line_name(*applied.changed.begin()).c_str(), created.size());
		model_told(applied.outcome, false, what);
		Commands.erase(Commands.begin());
	}
	if (Context.done)
		Context.done(created);
}

std::vector<TTrack *> editor_mode::model_tracks_after(std::uint64_t const Token, layout::network const &Before, std::set<layout::line_id> const &Changed, std::set<layout::line_id> const &Removed, model_context const &Context)
{
	auto const &after{model().session.network()};
	// the tracks which go: of the lines changed or gone; of a turnout one of whose tracks is, its switch tracks, kept by its main track,
	// before and after the change. Made again: the lines of their own changed, the turnouts there are now
	std::set<layout::line_id> going;
	std::set<layout::line_id> own;
	std::set<layout::turnout_id> turnouts;
	for (auto const *lines : {&Changed, &Removed})
		for (auto const line : *lines)
		{
			going.insert(line);
			if (auto const *earlier{layout::owner_of(Before, line)})
				going.insert(earlier->main);
			if (auto const *now{layout::owner_of(after, line)})
			{
				going.insert(now->main);
				turnouts.insert(now->id);
			}
			else if (lines == &Changed && after.find(line) != nullptr)
				own.insert(line);
		}
	// a turnout whose main track was there before and goes now is made again whole
	for (auto const &[id, turnout] : after.turnouts())
		if (going.contains(turnout.main))
			turnouts.insert(id);
	std::vector<TTrack *> removed;
	auto surface{Context.surface};
	for (auto const line : going)
		for (auto *track : model_line_tracks(line))
		{
			if (surface.empty() && track->eType == tt_Normal)
				surface = surface_of(*track);
			removed.push_back(track);
		}
	// the old tracks the line takes the place of
	for (auto *track : Context.replaced)
		if (false == track->m_editorremoved && std::ranges::find(removed, track) == removed.end())
			removed.push_back(track);
	// the old tracks go first: the new ones take their names
	for (auto *track : removed)
		editor_track::retire(*track);
	if (surface.empty())
		surface = "1.435 0.15 25 20 0 flat vis " + m_lay.style.rail + " 6 " + m_lay.style.ballast + " 0.2 0.5 1.1";
	model().session.options().surface = surface;
	std::vector<TTrack *> created;
	std::vector<std::pair<TTrack *, managed_track>> entries;
	std::map<layout::line_id, std::size_t> indices;
	// an entry of the generator put into the scene, the track marked as the line's
	auto const put = [&](layout::line_id const Line, std::string const &Text, std::string const &Name, std::string const &Kind) {
		auto const at{Text.find(" track ")};
		if (at == std::string::npos)
			return;
		auto *track{editor_track::load_entry(Text.substr(at + 7), Name)};
		if (track == nullptr)
			return;
		managed_track entry;
		entry.line = Line;
		entry.index = indices[Line]++;
		entry.text = Text;
		entry.kind = Kind;
		if (track->name() != Name)
		{
			// another track has the name: the entry written under the one it got
			WriteLog("Editor: model: the name " + Name + " is taken, the track is " + track->name(), logtype::generic);
			entry.text = "node -1 0 " + track->name() + Text.substr(at);
		}
		created.push_back(track);
		entries.emplace_back(track, std::move(entry));
	};
	for (auto const line : own)
	{
		auto const made{model().session.generate(line)};
		for (auto const &error : made.errors)
			WriteLog("Editor: model: the tracks of line " + line_name(line) + ": " + error, logtype::generic);
		for (auto const &piece : made.pieces)
			put(line, piece.text, piece.name, "track");
	}
	for (auto const id : turnouts)
	{
		auto const *turnout{after.find(id)};
		if (turnout == nullptr)
			continue;
		auto const made{model().session.generate(id)};
		for (auto const &error : made.errors)
			WriteLog(format("Editor: model: the tracks of turnout %u: %s", layout::number(id), error.c_str()), logtype::generic);
		if (made.parts.empty())
			put(turnout->main, made.text, made.name, "switch");
		for (auto const &part : made.parts)
		{
			if (part.is_switch)
			{
				put(turnout->main, part.text, part.name, "switch");
				continue;
			}
			for (auto const &piece : part.pieces[0])
				put(turnout->main, piece.text, piece.name, "track");
		}
	}
	editor_track::commit(created);
	for (auto &[track, entry] : entries)
	{
		entry.baseline = export_of(*track);
		model().tracks[track] = std::move(entry);
	}
	auto const steps{m_history.size()};
	auto const serial{m_history.empty() ? std::uint64_t{0} : m_history.back().serial};
	push_track_snapshot({}, created, removed);
	if (m_history.size() == steps && (m_history.empty() || m_history.back().serial == serial))
	{
		// nothing of the scene changed: the step is the model's alone
		trim_history();
		EditorSnapshot step;
		step.action = EditorSnapshot::Action::Other;
		m_history.push_back(std::move(step));
		g_redo.clear();
	}
	m_history.back().model_token = Token;
	if (std::ranges::any_of(removed, [&](TTrack const *Track) { return Track == m_node; }))
	{
		m_node = created.empty() ? nullptr : created.back();
		ui()->set_node(m_node);
	}
	return created;
}

void editor_mode::model_history(std::uint64_t const Token, bool const Undo)
{
	if (Token == 0 || false == model().session.is_open())
		return;
	auto const done{Undo ? model().session.undo(Token) : model().session.redo(Token)};
	if (false == done)
		WriteLog(format("Editor: model: the step %llu can't be %s: the history of the model went out of step with the editor's", static_cast<unsigned long long>(Token), Undo ? "taken back" : "brought again"), logtype::generic);
	model().message.clear();
	model().notices.clear();
	model().shown.clear();
	model().shown_notices.clear();
	model().refused = false;
}

std::optional<std::string> editor_mode::model_path_text(TTrack const &Track, glm::dvec3 const &Offset)
{
	if (false == model_on() || Track.m_editorremoved)
		return std::nullopt;
	auto const found{model().tracks.find(&Track)};
	if (found == model().tracks.end() || found->second.text.empty())
		return std::nullopt;
	auto const &entry{found->second};
	// changed by the other tools since it was made: its own export, a change by hand
	if (export_of(Track) != entry.baseline)
		return std::nullopt;
	if (Offset != entry.offset)
	{
		WriteLog("Editor: model: the track " + Track.name() + " goes into a file of another placement, written by its own export", logtype::generic);
		return std::nullopt;
	}
	model().written.emplace_back(&Track, entry.text);
	return entry.text;
}

void editor_mode::model_saved(bool const Success)
{
	if (false == model_on())
	{
		model().written.clear();
		return;
	}
	if (false == Success)
	{
		model().written.clear();
		return;
	}
	// the tracks written, by their lines, each with the file it went into
	std::map<layout::line_id, std::vector<project::session::written_piece>> lines;
	for (auto const &[track, text] : model().written)
	{
		auto const found{model().tracks.find(track)};
		auto const *source{scene::Layers.source(track)};
		if (found == model().tracks.end() || source == nullptr)
			continue;
		lines[found->second.line].push_back({track->name(), text, scene::Layers.layer(source->layer).name, found->second.kind});
	}
	model().written.clear();
	for (auto const &[line, pieces] : lines)
		model().session.record_written(line, pieces);
	for (auto const line : model_seen_changed())
		model().session.mark_hand_changed(line);
	std::string error;
	if (false == model().session.save(error))
	{
		ui()->set_status(format(STR_C("The project of the map isn't saved: %s"), error.c_str()), true);
		WriteLog("Editor: the project of the map isn't saved: " + error, logtype::generic);
		return;
	}
	WriteLog(format("Editor: the project of the map saved, %zu lines, %zu tracks of them written now", model().session.network().lines().size(), [&] {
		std::size_t count{0};
		for (auto const &entry : lines)
			count += entry.second.size();
		return count;
	}()), logtype::generic);
}

bool editor_mode::model_lay(lay_line_plan const &Plan, editor_track::snap_target const &End, std::vector<TTrack *> &Created, std::string &Why)
{
	if (false == model_on())
		return false;
	auto const &tool{m_lay};
	if (std::ranges::any_of(tool.modes, [](lay_mode const Mode) { return Mode != lay_mode::straights; }))
	{
		Why = STR("A curve traced is laid as an old track: the model lays the straights for now");
		return false;
	}
	auto const &design{Plan.design};
	if (false == Plan.shape.valid)
		return false;
	if (design.start_curved || design.end_curved)
	{
		Why = STR("A curve going on from the curve of a free end is laid as an old track: the model doesn't lay it yet");
		return false;
	}
	layout::create_line line;
	line.start = design.start;
	line.end = design.end;
	for (std::size_t i = 0; i < design.vertices.size(); ++i)
	{
		auto const &vertex{design.vertices[i]};
		if (vertex.compound || vertex.spirals || vertex.own_ramps())
		{
			Why = STR("A compound curve, or one with cant ramps of its own, is laid as an old track: the model doesn't lay it yet");
			return false;
		}
		layout::vertex made;
		made.point = i < Plan.shape.vertices.size() ? Plan.shape.vertices[i] : vertex.position;
		made.radius = vertex.radius;
		made.transition_in = vertex.transition_in;
		made.transition_out = vertex.transition_out;
		made.cant = vertex.cant;
		line.vertices.push_back(made);
	}
	line.transition = design.shape == alignment::transition_shape::clothoid ? layout::transition_shape::clothoid : layout::transition_shape::cubic_parabola;
	line.category.vmax = design.speed;
	model_context context;
	auto const *style{tool.start.track != nullptr ? tool.start.track : End.track};
	if (style == nullptr && tool.copy_selected)
		style = const_cast<editor_mode *>(this)->selected_track();
	if (style != nullptr)
		context.surface = surface_of(*style);
	context.done = [&Created](std::vector<TTrack *> const &Made) { Created = Made; };
	model_run({line}, context);
	return true;
}

layout::line_id editor_mode::model_plan_line() const
{
	auto const line{model().plan.line};
	if (line == layout::line_id::none || false == model_on() || m_route.chain.tracks.empty())
		return layout::line_id::none;
	auto const tracks{model_line_tracks(line)};
	if (tracks.empty() || m_route.from != tracks.front() || m_route.to != tracks.back())
		return layout::line_id::none;
	return line;
}

bool editor_mode::route_from_model(layout::line_id const Line)
{
	auto const *line{model().session.network().find(Line)};
	auto const tracks{model_line_tracks(Line)};
	model().plan = {};
	// a track of a turnout is edited through its turnout, not in the route design
	if (line == nullptr || tracks.empty() || line->turnout != layout::turnout_id::none)
		return false;
	auto &route{m_route};
	route.from = tracks.front();
	route.to = tracks.back();
	route.single = false;
	route_reset();
	if (route.chain.tracks.empty())
		return false;
	auto &design{route.design};
	design.vertices.clear();
	design.shape = line->shape.transition == layout::transition_shape::clothoid ? alignment::transition_shape::clothoid : alignment::transition_shape::cubic_parabola;
	design.speed = line->category.vmax;
	std::vector<layout::vertex_id> ids;
	for (auto const &each : line->shape.vertices)
	{
		alignment::vertex vertex;
		vertex.position = each.point;
		vertex.radius = each.radius;
		vertex.transition_in = each.transition_in;
		vertex.transition_out = each.transition_out;
		vertex.cant = each.cant;
		design.vertices.push_back(vertex);
		ids.push_back(each.id);
	}
	if (false == design.vertices.empty())
	{
		design.vertices.front().offset = std::max(0.0, glm::dot(design.vertices.front().position - glm::dvec2{design.start.x, design.start.z}, design.start_direction));
		design.vertices.back().offset = std::max(0.0, glm::dot(glm::dvec2{design.end.x, design.end.z} - design.vertices.back().position, design.end_direction));
	}
	route.designed = static_cast<int>(design.vertices.size());
	route.clicks.clear();
	route_update();
	model().plan.line = Line;
	model().plan.vertices = std::move(ids);
	model().plan.original = design;
	route.status = format(STR_P("Line %s of the model, %.0f m, %zu curve", "Line %s of the model, %.0f m, %zu curves", static_cast<long long>(design.vertices.size())), line_name(Line).c_str(), line->solved.length, design.vertices.size());
	return true;
}

bool editor_mode::model_plan_commands(layout::line_id const Line, std::vector<layout::command> &Commands, std::string &Why) const
{
	auto const &plan{model().plan};
	auto const &design{m_route.design};
	if (design.vertices.size() != plan.vertices.size() || plan.original.vertices.size() != plan.vertices.size())
	{
		Why = "Curves added or taken out here aren't applied to a line of the model: add a curve or take one out with the buttons of the line";
		return false;
	}
	// where the vertex is: the first and the last of two or more along the tangents of the ends, by their offsets, as the route design
	// draws and drags them; the others where they are
	auto const place = [](alignment::design const &Design, std::size_t const Index) {
		auto const &vertex{Design.vertices[Index]};
		auto const count{Design.vertices.size()};
		if (count >= 2 && Index == 0 && glm::length(Design.start_direction) > 1e-9)
			return glm::dvec2{Design.start.x, Design.start.z} + glm::normalize(Design.start_direction) * vertex.offset;
		if (count >= 2 && Index + 1 == count && glm::length(Design.end_direction) > 1e-9)
			return glm::dvec2{Design.end.x, Design.end.z} - glm::normalize(Design.end_direction) * vertex.offset;
		return vertex.position;
	};
	for (std::size_t i = 0; i < design.vertices.size(); ++i)
	{
		auto was{plan.original.vertices[i]};
		auto now{design.vertices[i]};
		was.position = place(plan.original, i);
		now.position = place(design, i);
		if (now.compound || now.spirals || now.own_ramps())
		{
			Why = "A compound curve, or one with cant ramps of its own, isn't laid by the model yet";
			return false;
		}
		// a vertex moved: move_vertex; its curve changed besides: one set_vertex with its place, one step for the vertex
		auto const moved{glm::distance(now.position, was.position) > 1e-4};
		layout::set_vertex set{Line, plan.vertices[i]};
		if (now.radius != was.radius)
			set.radius = now.radius;
		if (now.transition_in != was.transition_in)
			set.transition_in = now.transition_in;
		if (now.transition_out != was.transition_out)
			set.transition_out = now.transition_out;
		if (now.cant != was.cant)
			set.cant = now.cant;
		if (set.radius || set.transition_in || set.transition_out || set.cant)
		{
			if (moved)
				set.point = now.position;
			Commands.push_back(set);
		}
		else if (moved)
			Commands.push_back(layout::move_vertex{Line, plan.vertices[i], now.position});
	}
	return true;
}

bool editor_mode::model_plan_apply()
{
	auto const line{model_plan_line()};
	if (line == layout::line_id::none)
		return false;
	model().preview = {};
	std::vector<layout::command> commands;
	if (std::string why; false == model_plan_commands(line, commands, why))
	{
		layout::outcome refusal;
		refusal.reasons.push_back(why);
		model_told(refusal, true);
		return true;
	}
	if (commands.empty())
	{
		model_told({}, false, STR("Nothing changed"));
		return true;
	}
	model_context context;
	context.done = [this, line](std::vector<TTrack *> const &) {
		auto const message{model().message};
		auto const notices{model().notices};
		auto const shown{model().shown};
		auto const shown_notices{model().shown_notices};
		route_from_model(line);
		model().message = message;
		model().notices = notices;
		model().shown = shown;
		model().shown_notices = shown_notices;
	};
	model_run(std::move(commands), std::move(context));
	return true;
}

bool editor_mode::model_plan_convert(std::string &Why)
{
	if (false == model_on() || model_plan_line() != layout::line_id::none)
		return false;
	auto &route{m_route};
	if (false == route.result.valid || route.chain.tracks.empty())
		return false;
	auto const &design{route.design};
	auto const &chain{route.chain};
	auto const events = [](TTrack const &Track) {
		return false == Track.m_events0.empty() || false == Track.m_events1.empty() || false == Track.m_events2.empty() || false == Track.m_events0all.empty() || false == Track.m_events1all.empty() ||
		       false == Track.m_events2all.empty() || false == Track.Isolated.empty();
	};
	if (false == chain.switches.empty())
		Why = STR("A line through switches is applied to the old tracks: the switches go through the model later");
	else if (design.start_curved || design.end_curved)
		Why = STR("A curve going on from the curve of the track beyond is applied to the old tracks: the model doesn't lay it yet");
	else if (std::ranges::any_of(design.vertices, [](alignment::vertex const &Vertex) { return Vertex.compound || Vertex.spirals || Vertex.own_ramps(); }))
		Why = STR("A compound curve, or one with cant ramps of its own, is applied to the old tracks: the model doesn't lay it yet");
	else if (std::ranges::any_of(chain.tracks, [&](TTrack const *Track) { return Track->eType != tt_Normal || model_track(Track) != nullptr || events(*Track); }))
		Why = STR("Tracks with events or in isolated circuits are applied as old tracks: their events would be lost");
	if (false == Why.empty())
		return false;
	// the grade line is the model's in a later stage: an even grade from end to end, as the tracks have it, is taken over
	{
		auto const rise{design.end.y - design.start.y};
		auto previous{design.start};
		double along{0.0};
		auto const off = [&](glm::dvec3 const &Point) { return std::abs(Point.y - (design.start.y + rise * (chain.length > 0.0 ? std::clamp(along / chain.length, 0.0, 1.0) : 0.0))); };
		for (auto const *track : chain.tracks)
		{
			auto const &path{track->m_paths.front()};
			auto from{path.points[segment_data::point::start]};
			auto to{path.points[segment_data::point::end]};
			if (glm::distance(to, previous) < glm::distance(from, previous))
				std::swap(from, to);
			auto const first{off(from)};
			along += track->Length();
			if (std::max(first, off(to)) > 0.01)
			{
				Why = STR("A line with a grade line of its own is applied to the old tracks: the model takes over the grade line later");
				return false;
			}
			previous = to;
		}
	}
	layout::create_line line;
	line.start = design.start;
	line.end = design.end;
	for (std::size_t i = 0; i < design.vertices.size(); ++i)
	{
		auto const &vertex{design.vertices[i]};
		layout::vertex made;
		made.point = i < route.result.vertices.size() ? route.result.vertices[i] : vertex.position;
		made.radius = vertex.radius;
		made.transition_in = vertex.transition_in;
		made.transition_out = vertex.transition_out;
		made.cant = vertex.cant;
		line.vertices.push_back(made);
	}
	line.transition = design.shape == alignment::transition_shape::clothoid ? layout::transition_shape::clothoid : layout::transition_shape::cubic_parabola;
	line.category.vmax = design.speed;
	auto const tracks{chain.tracks};
	std::vector<std::string> names;
	for (auto const *track : tracks)
		if (false == track->name().empty() && track->name() != "none")
			names.push_back(track->name());
	// the old tracks named by a scenario go only when the user says so
	model_check_users(names, [this, line, tracks]() {
		if (std::ranges::any_of(tracks, [](TTrack const *Track) { return Track->m_editorremoved; }))
			return;
		model_context context;
		context.replaced = tracks;
		context.surface = surface_of(*tracks.front());
		context.done = [this](std::vector<TTrack *> const &Created) {
			auto const message{model().message};
			auto const notices{model().notices};
			auto const shown{model().shown};
			auto const shown_notices{model().shown_notices};
			if (auto const *managed{Created.empty() ? nullptr : model_track(Created.front())})
				route_from_model(managed->line);
			else
				route_drop();
			model().message = message;
			model().notices = notices;
			model().shown = shown;
			model().shown_notices = shown_notices;
		};
		model_run({line}, context);
	});
	return true;
}

void editor_mode::render_model_plan()
{
	auto const line{model_plan_line()};
	auto const *shape{model().session.network().find(line)};
	if (shape == nullptr)
		return;
	ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.55f, 1.0f), "%s", format(STR_C("Line %s (managed)"), line_name(line).c_str()).c_str());
	item_tooltip("A line of the model of the track geometry: its changes are applied as commands of the model, checked against the norms, its tracks made anew");
	if (false == model().shown.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		if (model().refused)
			ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", model().shown.c_str());
		else
			ImGui::TextDisabled("%s", model().shown.c_str());
		ImGui::PopTextWrapPos();
	}
	// the quality line: what the norms warn of, or would have otherwise
	if (false == model().notices.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.25f, 1.0f), "%s", format(STR_P("The norms: %zu note", "The norms: %zu notes", static_cast<long long>(model().notices.size())), model().notices.size()).c_str());
		for (auto const &notice : model().shown_notices)
			ImGui::BulletText("%s", notice.c_str());
		ImGui::PopTextWrapPos();
	}
	// split in two at the chainage, joined from a free end to the nearest free end of another line, deleted
	auto &split{model().plan.split};
	if (split < 0.0 || split > shape->solved.length)
		split = std::round(shape->solved.length * 0.5);
	ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.0f);
	ImGui::InputDouble("##modelsplit", &split, 0.0, 0.0, "%.1f m");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Split the line here")))
	{
		model_context context;
		context.done = [this, line](std::vector<TTrack *> const &) { route_from_model(line); };
		model_run({layout::split_line{line, split}}, context);
		return;
	}
	item_tooltip("The line cut in two at the distance from its start, the track kept as it is");
	auto const &network{model().session.network()};
	for (auto const end : {shape->start, shape->end})
	{
		auto const *node{network.find(end)};
		if (node == nullptr || node->ends.size() != 1)
			continue;
		layout::node const *nearest{nullptr};
		double distance{300.0};
		for (auto const &[id, other] : network.nodes())
			if (id != end && other.ends.size() == 1 && other.ends.front().line != line && glm::distance(other.position, node->position) < distance)
			{
				distance = glm::distance(other.position, node->position);
				nearest = &other;
			}
		if (nearest == nullptr)
			continue;
		ImGui::PushID(static_cast<int>(layout::number(end)));
		if (ImGui::Button(format(STR_C("Join the %s to line %s (%.0f m)"), end == shape->start ? STR_C("start") : STR_C("end"), line_name(nearest->ends.front().line).c_str(), distance).c_str()))
		{
			model_context context;
			context.done = [this, line](std::vector<TTrack *> const &) { route_from_model(line); };
			model_run({layout::join_ends{end, nearest->id}}, context);
			ImGui::PopID();
			return;
		}
		item_tooltip("A new line laid between the free end and the free end of the other line, its curves as the norms prefer");
		ImGui::PopID();
	}
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.18f, 0.15f, 1.0f));
	if (ImGui::Button(format(STR_C("Delete line %s"), line_name(line).c_str()).c_str()))
	{
		ImGui::PopStyleColor();
		model_delete_line(line);
		return;
	}
	ImGui::PopStyleColor();
	item_tooltip("The line and its tracks taken away; the scenarios naming its tracks are looked through first. Ctrl+Z brings it back");
}

bool editor_mode::model_delete(TTrack *Track)
{
	if (false == model_on() || Track == nullptr || m_model_delete_confirmed)
		return false;
	if (auto const *managed{model_track(Track)})
	{
		// a track of a turnout: the turnout taken out
		if (auto const *turnout{layout::owner_of(model_network(), managed->line)})
			model_turnout_remove(turnout->id);
		else
			model_delete_line(managed->line);
		return true;
	}
	if (Track->name().empty() || Track->name() == "none")
		return false;
	model_check_users({Track->name()}, [this, Track]() {
		if (Track->m_editorremoved)
			return;
		m_node = Track;
		ui()->set_node(m_node);
		m_model_delete_confirmed = true;
		delete_selected_track();
		m_model_delete_confirmed = false;
	});
	return true;
}

bool editor_mode::model_delete_set(std::vector<TTrack *> const &Tracks)
{
	if (false == model_on() || m_model_delete_confirmed)
		return false;
	if (std::ranges::any_of(Tracks, [&](TTrack const *Track) { return model_track(Track) != nullptr; }))
	{
		ui()->set_status(STR_C("A line of the model goes by Del on one of its tracks, not in a set"), true);
		return true;
	}
	std::vector<std::string> names;
	for (auto const *track : Tracks)
		if (false == track->name().empty() && track->name() != "none")
			names.push_back(track->name());
	if (names.empty())
		return false;
	model_check_users(names, [this, Tracks]() {
		m_track_set.clear();
		for (auto *track : Tracks)
			if (false == track->m_editorremoved)
				m_track_set.push_back(track);
		if (m_track_set.empty())
			return;
		m_node = m_track_set.back();
		ui()->set_node(m_node);
		m_model_delete_confirmed = true;
		track_set_delete();
		m_model_delete_confirmed = false;
	});
	return true;
}

void editor_mode::model_delete_line(layout::line_id const Line)
{
	std::vector<std::string> names;
	for (auto const *track : model_line_tracks(Line))
		names.push_back(track->name());
	model_check_users(names, [this, Line]() {
		model_context context;
		context.done = [this, Line](std::vector<TTrack *> const &) {
			if (model().plan.line == Line)
			{
				model().plan = {};
				route_drop();
			}
			m_node = nullptr;
			ui()->set_node(nullptr);
			m_track_point = {};
		};
		model_run({layout::delete_line{Line}}, context);
	});
}

void editor_mode::model_check_users(std::vector<std::string> Names, std::function<void()> Then, bool const Rename)
{
	if (Names.empty())
	{
		Then();
		return;
	}
	auto &question{model().question};
	question = {};
	question.what = model_question::kind::waiting;
	question.names = std::move(Names);
	question.rename = Rename;
	question.answer = [Then](bool const Go) {
		if (Go)
			Then();
	};
	model().index.request(model().session.folder(), model().session.map(), model().store);
	// the index ready already: answered now
	model_update();
}

void editor_mode::render_model_windows()
{
	auto &question{model().question};
	if (question.what == model_question::kind::none)
		return;
	char const *ids[]{"", "###modelhand", "###modelwaiting", "###modelusers"};
	auto const id{ids[static_cast<int>(question.what)]};
	if (false == question.opened)
	{
		ImGui::OpenPopup(id);
		question.opened = true;
	}
	auto const answer = [&](bool const Yes) {
		auto const given{std::move(question.answer)};
		question = {};
		ImGui::CloseCurrentPopup();
		if (given)
			given(Yes);
	};
	ImGui::SetNextWindowSize(ImVec2(460.0f * Global.ui_scale, 0.0f), ImGuiCond_Appearing);
	switch (question.what)
	{
	case model_question::kind::hand:
	{
		std::string lines;
		for (auto const line : question.lines)
			lines += (lines.empty() ? "" : ", ") + line_name(line);
		if (ImGui::BeginPopupModal((std::string{STR_C("Changed by hand")} + id).c_str(), nullptr, ImGuiWindowFlags_NoSavedSettings))
		{
			ImGui::PushTextWrapPos(0.0f);
			ImGui::TextUnformatted(format(STR_C("The tracks of line %s were changed by hand since the editor made them."), lines.c_str()).c_str());
			ImGui::TextDisabled("%s", STR_C("Overwritten, they're made anew of the line as the model lays it and the change by hand is lost. Kept, the edit isn't made."));
			ImGui::PopTextWrapPos();
			if (ImGui::Button(STR_C("Keep the change by hand")))
				answer(false);
			ImGui::SameLine();
			if (ImGui::Button(STR_C("Overwrite")))
				answer(true);
			ImGui::EndPopup();
		}
		break;
	}
	case model_question::kind::waiting:
		if (ImGui::BeginPopupModal((std::string{STR_C("Checking the scenarios")} + id).c_str(), nullptr, ImGuiWindowFlags_NoSavedSettings))
		{
			ImGui::PushTextWrapPos(0.0f);
			ImGui::TextUnformatted(question.rename ? STR_C("Looking through the scenarios which use the map, before the track is renamed. On a big map it takes a few minutes, once.")
			                                       : STR_C("Looking through the scenarios which use the map, before the track goes. On a big map it takes a few minutes, once."));
			ImGui::PopTextWrapPos();
			if (ImGui::Button(STR_C("Cancel")))
				answer(false);
			ImGui::SameLine();
			if (ImGui::Button(question.rename ? STR_C("Rename without checking") : STR_C("Delete without checking")))
				answer(true);
			ImGui::EndPopup();
		}
		break;
	case model_question::kind::scenarios:
		if (ImGui::BeginPopupModal((format(STR_P("Track used by %zu scenario", "Track used by %zu scenarios", static_cast<long long>(question.users.size())), question.users.size()) + id).c_str(), nullptr, ImGuiWindowFlags_NoSavedSettings))
		{
			ImGui::PushTextWrapPos(0.0f);
			ImGui::TextUnformatted(question.rename ? STR_C("Renamed, the scenarios lose what they name in the map:") : STR_C("Taken away, the scenarios lose what they name in the map:"));
			for (auto const &use : question.users)
			{
				ImGui::BulletText("%s", use.scenario.c_str());
				ImGui::Indent();
				for (auto const &reference : use.references)
					ImGui::TextDisabled("%s", format(STR_C("%s: %s %s (%s:%zu)"), reference.name.c_str(), STR_C(project::use_name(reference.use)), reference.from.c_str(), reference.file.c_str(), reference.line).c_str());
				ImGui::Unindent();
			}
			ImGui::PopTextWrapPos();
			if (ImGui::Button(STR_C("Cancel")))
				answer(false);
			ImGui::SameLine();
			if (ImGui::Button(question.rename ? STR_C("Rename anyway") : STR_C("Delete anyway")))
				answer(true);
			ImGui::EndPopup();
		}
		break;
	default: break;
	}
}

editor_mode::selftest_result editor_mode::selftest_model(std::string const &command, std::istringstream &words)
{
	// a track by its name, or @<line>.<index>: the track of the line of the model at the index from its start
	auto const resolve = [&](std::string const &Name) -> TTrack * {
		if (false == Name.empty() && Name.front() == '@')
		{
			unsigned line{0};
			std::size_t index{0};
			if (std::sscanf(Name.c_str() + 1, "%u.%zu", &line, &index) != 2)
				return nullptr;
			auto const tracks{model_line_tracks(static_cast<layout::line_id>(line))};
			return index < tracks.size() ? tracks[index] : nullptr;
		}
		auto *track{simulation::Paths.find(Name)};
		return track != nullptr && false == track->m_editorremoved ? track : nullptr;
	};
	if (command == "modelediting")
	{
		int on{0};
		words >> on;
		// for this run, not kept in the settings of the editor
		run_setting() = on != 0;
		if (on != 0 && false == model().tried && false == scene::Layers.empty())
			model_open();
		WriteLog(format("SELFTEST modelediting %d: the project %s, %zu lines", on, model().session.is_open() ? "open" : "not open", model().session.network().lines().size()));
		return selftest_result::handled;
	}
	if (command == "modelwait")
	{
		// the scenario index looked through for the question waiting on it
		model_update();
		return model().question.what == model_question::kind::waiting ? selftest_result::again : selftest_result::handled;
	}
	if (command == "modelanswer")
	{
		std::string answer;
		words >> answer;
		auto &question{model().question};
		WriteLog(format("SELFTEST modelanswer %s: the question %d", answer.c_str(), static_cast<int>(question.what)));
		if (question.what == model_question::kind::none)
			return selftest_result::handled;
		auto const given{std::move(question.answer)};
		question = {};
		if (given)
			given(answer == "yes");
		return selftest_result::handled;
	}
	if (command == "modeltracks")
	{
		for (auto const &[id, line] : model().session.network().lines())
		{
			std::string names;
			for (auto const *track : model_line_tracks(id))
				names += " " + track->name();
			WriteLog(format("SELFTEST modeltracks %s: %.3f m, %zu vertices, tracks%s", line_name(id).c_str(), line.solved.length, line.shape.vertices.size(), names.c_str()));
		}
		return selftest_result::handled;
	}
	if (command == "modelmark")
	{
		std::string label;
		words >> label;
		model().marks[label] = model().session.network();
		return selftest_result::handled;
	}
	if (command == "modelplan")
	{
		// the route design opened on the line of the track, as a click on it with the tool does
		std::string name;
		words >> name;
		auto *track{resolve(name)};
		show_track_tab(track_tab::route);
		m_track_tab = track_tab::route;
		m_route.single = false;
		auto const loaded{track != nullptr && route_load(*track)};
		WriteLog(format("SELFTEST modelplan %s: %s, line %s, %zu vertices", name.c_str(), loaded ? "loaded" : "failed", line_name(model_plan_line()).c_str(), m_route.design.vertices.size()));
		return selftest_result::handled;
	}
	if (command == "modelselect" || command == "modeldel")
	{
		// the track selected, as a click on it selects it; deleted as Del deletes it
		std::string name;
		words >> name;
		auto *track{resolve(name)};
		if (track != nullptr)
		{
			m_node = track;
			m_track_point = {};
			ui()->set_node(m_node);
			if (command == "modeldel")
				delete_selected_track();
		}
		WriteLog(format("SELFTEST %s %s: %s", command.c_str(), name.c_str(), track != nullptr ? track->name().c_str() : "no track"));
		return selftest_result::handled;
	}
	if (command == "modelrename")
	{
		// the track renamed as the inspector renames it
		std::string name, to;
		words >> name >> to;
		auto *track{resolve(name)};
		std::string reason;
		if (track != nullptr && editor_track::name_valid(*track, to, reason))
			track_rename(*track, to);
		WriteLog(format("SELFTEST modelrename %s %s: %s", name.c_str(), to.c_str(), track != nullptr ? reason.c_str() : "no track"));
		return selftest_result::handled;
	}
	if (command == "routeradius" || command == "routecant")
	{
		// the field of the curve in the route design typed into, the rest of it as it is
		int index{0};
		double value{0.0};
		words >> index >> value;
		if (index >= 0 && index < static_cast<int>(m_route.design.vertices.size()))
		{
			(command == "routeradius" ? m_route.design.vertices[index].radius : m_route.design.vertices[index].cant) = value;
			route_update();
		}
		WriteLog(format("SELFTEST %s %d %.1f: valid %d", command.c_str(), index, value, m_route.result.valid ? 1 : 0));
		return selftest_result::handled;
	}
	if (command == "routemove")
	{
		int index{0};
		glm::dvec2 point{0.0};
		words >> index >> point.x >> point.y;
		if (index >= 0 && index < static_cast<int>(m_route.design.vertices.size()))
		{
			m_route.design.vertices[index].position = point;
			route_update();
		}
		WriteLog(format("SELFTEST routemove %d: valid %d", index, m_route.result.valid ? 1 : 0));
		return selftest_result::handled;
	}
	if (command == "modelgrade")
	{
		// the profile opened on the line of the track, a break put into its grade line at the chainage from the start of the route with
		// its height and vertical radius, applied as the button applies it
		std::string name;
		double chainage{0.0}, elevation{0.0}, radius{0.0};
		words >> name >> chainage >> elevation >> radius;
		auto *track{resolve(name)};
		if (track == nullptr)
		{
			WriteLog("SELFTEST modelgrade: no track " + name);
			return selftest_result::handled;
		}
		profile_open_run(*track);
		auto &state{m_profile};
		auto &points{state.line.points};
		if (points.size() >= 2 && chainage > points.front().chainage && chainage < points.back().chainage)
		{
			auto const at{std::ranges::find_if(points, [&](profile::pvi const &Point) { return Point.chainage > chainage; })};
			points.insert(at, profile::pvi{chainage, elevation, radius, false, false});
			state.changed = true;
			profile_check();
			profile_apply(false);
		}
		WriteLog(format("SELFTEST modelgrade %s %.1f %.3f R%.0f: %zu points, %s %s", name.c_str(), chainage, elevation, radius, points.size(), editor_selftest::one_line(state.status).c_str(), editor_selftest::one_line(state.error).c_str()));
		return selftest_result::handled;
	}
	if (command == "modelturnoutside")
	{
		// the turnout of the switch track given the other side, as the inspector's button gives it
		std::string name;
		words >> name;
		auto *track{resolve(name)};
		auto const *turnout{model_turnout(track)};
		if (turnout != nullptr)
			model_turnout_run(layout::change_turnout{turnout->id, turnout->type, -turnout->hand});
		WriteLog(format("SELFTEST modelturnoutside %s: %s", name.c_str(), turnout != nullptr ? editor_selftest::one_line(model().message).c_str() : "no turnout"));
		return selftest_result::handled;
	}
	if (command == "modelsplit" || command == "modeldelete" || command == "modeljoin")
	{
		auto const line{model_plan_line()};
		if (line == layout::line_id::none)
		{
			WriteLog("SELFTEST " + command + ": no line of the model open in the route design");
			return selftest_result::handled;
		}
		model_context context;
		context.done = [this, line](std::vector<TTrack *> const &) { route_from_model(line); };
		if (command == "modelsplit")
		{
			double chainage{0.0};
			words >> chainage;
			model_run({layout::split_line{line, chainage}}, context);
		}
		else if (command == "modeldelete")
			model_delete_line(line);
		else
		{
			// from the free end of the line at the point to the free end of another one nearest the other point
			glm::dvec2 from{0.0}, to{0.0};
			words >> from.x >> from.y >> to.x >> to.y;
			auto const nearest = [&](glm::dvec2 const &Point, bool const Own) {
				layout::node const *found{nullptr};
				for (auto const &[id, node] : model().session.network().nodes())
					if (node.ends.size() == 1 && (node.ends.front().line == line) == Own &&
					    (found == nullptr || glm::distance(glm::dvec2{node.position.x, node.position.z}, Point) < glm::distance(glm::dvec2{found->position.x, found->position.z}, Point)))
						found = &node;
				return found;
			};
			auto const *start{nearest(from, true)};
			auto const *end{nearest(to, false)};
			if (start != nullptr && end != nullptr)
				model_run({layout::join_ends{start->id, end->id}}, context);
		}
		WriteLog("SELFTEST " + command + ": " + editor_selftest::one_line(model().message));
		return selftest_result::handled;
	}
	if (command != "expect")
		return selftest_result::unhandled;
	auto const start{words.tellg()};
	std::string what;
	words >> what;
	if (what == "managed")
	{
		std::string name, line;
		words >> name >> line;
		auto const *track{resolve(name)};
		auto const *managed{model_track(track)};
		auto const got{track == nullptr ? std::string{"no track"} : managed == nullptr ? std::string{"none"} : std::to_string(layout::number(managed->line))};
		auto const label{model_label(track)};
		expectation(line == "any" ? managed != nullptr : got == line, "managed " + name, got + (label.empty() ? "" : " (" + label + ")"), line);
		return selftest_result::handled;
	}
	if (what == "modellines")
	{
		std::size_t count{0};
		words >> count;
		auto const got{model().session.network().lines().size()};
		expectation(model_on() && got == count, "modellines", std::to_string(got) + (model_on() ? "" : " (no model)"), std::to_string(count));
		return selftest_result::handled;
	}
	if (what == "projectfile")
	{
		std::string wanted;
		std::size_t lines{0};
		words >> wanted;
		auto const file{project::path_of(std::filesystem::path{paths::scenery} / scene::Layers.layer(scene::layer_handle{1}).name)};
		auto const there{std::filesystem::exists(file)};
		if (wanted == "lines")
		{
			// the lines of the project as the file has them
			words >> lines;
			auto const read{project::load(file)};
			auto const got{read.project.has_value() ? read.project->network.lines().size() : std::size_t{0}};
			expectation(read.project.has_value() && got == lines, "projectfile lines", read.project.has_value() ? std::to_string(got) : read.error, std::to_string(lines));
			return selftest_result::handled;
		}
		expectation(there == (wanted == "present"), "projectfile " + file.generic_string(), there ? "present" : "absent", wanted);
		return selftest_result::handled;
	}
	if (what == "modelends")
	{
		// the tracks of each line from end to end with the numbers of its nodes, and from track to track, to the bit
		std::string problems;
		std::size_t checked{0};
		auto const &network{model().session.network()};
		for (auto const &[id, line] : network.lines())
		{
			auto const *start{network.find(line.start)};
			auto const *end{network.find(line.end)};
			// a track of a turnout: the path of its switch track from the start of the turnout, at the numbers of the nodes at its ends
			if (line.turnout != layout::turnout_id::none)
			{
				auto const *turnout{network.find(line.turnout)};
				if (turnout == nullptr || start == nullptr || end == nullptr || layout::kind_of(*turnout) != layout::turnout_kind::ordinary)
					continue;
				TTrack const *switched{nullptr};
				for (auto const *track : model_line_tracks(turnout->main))
					if (track->eType == tt_Switch && track->m_paths.size() >= 2)
						switched = track;
				auto const path{id == turnout->main ? 0 : id == turnout->diverging ? 1 : -1};
				if (switched == nullptr || path < 0)
				{
					problems += " " + line_name(id) + " has no switch track";
					continue;
				}
				auto const &points{switched->m_paths[path].points};
				auto const from{points[segment_data::point::start]};
				auto const to{points[segment_data::point::end]};
				if (false == ((bits_equal(from, start->position) && bits_equal(to, end->position)) || (bits_equal(from, end->position) && bits_equal(to, start->position))))
					problems += format(" %s: the path %d of %s off its nodes", line_name(id).c_str(), path, switched->name().c_str());
				++checked;
				continue;
			}
			auto const tracks{model_line_tracks(id)};
			if (tracks.empty() || start == nullptr || end == nullptr)
			{
				problems += " " + line_name(id) + " has no tracks";
				continue;
			}
			auto const first{tracks.front()->m_paths.front().points[segment_data::point::start]};
			auto const last{tracks.back()->m_paths.front().points[segment_data::point::end]};
			if (false == bits_equal(first, start->position))
				problems += format(" %s start %.17g %.17g %.17g against %.17g %.17g %.17g", line_name(id).c_str(), first.x, first.y, first.z, start->position.x, start->position.y, start->position.z);
			if (false == bits_equal(last, end->position))
				problems += format(" %s end %.17g %.17g %.17g against %.17g %.17g %.17g", line_name(id).c_str(), last.x, last.y, last.z, end->position.x, end->position.y, end->position.z);
			for (std::size_t i = 1; i < tracks.size(); ++i)
				if (false == bits_equal(tracks[i - 1]->m_paths.front().points[segment_data::point::end], tracks[i]->m_paths.front().points[segment_data::point::start]))
					problems += " " + line_name(id) + " between " + tracks[i - 1]->name() + " and " + tracks[i]->name();
			++checked;
		}
		expectation(problems.empty() && checked > 0, "modelends", problems.empty() ? std::to_string(checked) + " lines" : problems, "the ends at the nodes and between the tracks the same to the bit");
		return selftest_result::handled;
	}
	if (what == "question")
	{
		std::string kind;
		words >> kind;
		char const *names[]{"none", "hand", "waiting", "scenarios"};
		auto const &question{model().question};
		auto got{std::string{names[static_cast<int>(question.what)]}};
		if (question.what == model_question::kind::scenarios)
			got += " " + std::to_string(question.users.size());
		if (question.what == model_question::kind::hand)
			for (auto const line : question.lines)
				got += " " + line_name(line);
		std::string count;
		words >> count;
		auto const wanted{count.empty() ? kind : kind + " " + count};
		expectation(got == wanted || (count.empty() && got.starts_with(kind)), "question " + wanted, got, wanted);
		return selftest_result::handled;
	}
	if (what == "modelmessage")
	{
		std::string mode, text;
		words >> mode;
		std::getline(words, text);
		text.erase(0, text.find_first_not_of(' '));
		// as the model told it, or as the user reads it in the language of the editor
		auto message{model().message};
		for (auto const &notice : model().notices)
			message += "\n" + notice;
		auto shown{model().shown};
		for (auto const &notice : model().shown_notices)
			shown += "\n" + notice;
		auto const holds{message.find(text) != std::string::npos || shown.find(text) != std::string::npos};
		expectation(holds == (mode == "holds"), "modelmessage " + mode + " " + text, editor_selftest::one_line(message), (mode == "holds" ? "holding " : "lacking ") + text);
		return selftest_result::handled;
	}
	if (what == "handlines")
	{
		// the lines taken for changed by hand: in the project, or seen changed by the other tools since they were made
		std::size_t wanted{0};
		words >> wanted;
		auto marked{model_seen_changed()};
		marked.insert(model().session.project().hand_changed.begin(), model().session.project().hand_changed.end());
		std::string lines;
		for (auto const line : marked)
			lines += " " + line_name(line);
		expectation(marked.size() == wanted, "handlines", std::to_string(marked.size()) + lines, std::to_string(wanted));
		return selftest_result::handled;
	}
	if (what == "modelgrades")
	{
		// the breaks of the grade line of the line of the model
		unsigned line{0};
		std::size_t wanted{0};
		words >> line >> wanted;
		auto const *found{model_network().find(static_cast<layout::line_id>(line))};
		auto const got{found != nullptr ? found->shape.grades.size() : std::size_t{0}};
		std::string breaks;
		if (found != nullptr)
			for (auto const &each : found->shape.grades)
				breaks += format(" %.1f/%.3f/R%.0f", each.chainage, each.elevation, each.vertical_radius);
		expectation(found != nullptr && got == wanted, format("modelgrades %u", line), std::to_string(got) + breaks, std::to_string(wanted));
		return selftest_result::handled;
	}
	if (what == "modelvertexat")
	{
		// where the vertex of the line of the model is, in the plan
		unsigned line{0};
		std::size_t index{0};
		glm::dvec2 wanted{0.0};
		double tolerance{0.01};
		words >> line >> index >> wanted.x >> wanted.y >> tolerance;
		auto const *found{model_network().find(static_cast<layout::line_id>(line))};
		glm::dvec2 got{std::numeric_limits<double>::quiet_NaN()};
		if (found != nullptr && index < found->shape.vertices.size())
			got = found->shape.vertices[index].point;
		expectation(glm::distance(got, wanted) <= tolerance, format("modelvertexat %u %zu", line, index), format("%.3f %.3f", got.x, got.y), format("%.3f %.3f within %.3f", wanted.x, wanted.y, tolerance));
		return selftest_result::handled;
	}
	if (what == "modelpreview")
	{
		// a handle of the route design dragged on a line of the model: the line as the model would lay it tried, and told
		std::string wanted;
		words >> wanted;
		auto const &preview{model().preview};
		auto const got{preview.key.empty() ? std::string{"none"} : std::string{"shown"}};
		expectation(got == wanted, "modelpreview " + wanted, got + (preview.key.empty() ? "" : (preview.refused ? " refused: " : ": ") + editor_selftest::one_line(preview.text)), wanted);
		return selftest_result::handled;
	}
	if (what == "modelproposal")
	{
		// the solver's best proposal for the crossover refused last
		std::string mode, text;
		words >> mode;
		std::getline(words, text);
		text.erase(0, text.find_first_not_of(' '));
		auto const holds{false == model().proposal.empty() && model().proposal.find(text) != std::string::npos};
		expectation(holds == (mode == "holds"), "modelproposal " + mode + " " + text, model().proposal.empty() ? std::string{"none"} : model().proposal, (mode == "holds" ? "holding " : "lacking ") + text);
		return selftest_result::handled;
	}
	if (what == "hand")
	{
		unsigned line{0};
		int wanted{0};
		words >> line >> wanted;
		auto const id{static_cast<layout::line_id>(line)};
		auto const marked{model().session.project().hand_changed.contains(id) || model_seen_changed().contains(id)};
		expectation(marked == (wanted != 0), format("hand %u", line), marked ? "1" : "0", std::to_string(wanted));
		return selftest_result::handled;
	}
	if (what == "modelrefused")
	{
		int wanted{0};
		words >> wanted;
		expectation(model().refused == (wanted != 0), "modelrefused", (model().refused ? "1: " : "0: ") + editor_selftest::one_line(model().message), std::to_string(wanted));
		return selftest_result::handled;
	}
	if (what == "modelvertex")
	{
		// a value of the vertex of the line of the model: radius, in, out, cant
		unsigned line{0};
		std::size_t index{0};
		std::string key;
		double wanted{0.0};
		words >> line >> index >> key >> wanted;
		auto const *found{model().session.network().find(static_cast<layout::line_id>(line))};
		auto got{std::numeric_limits<double>::quiet_NaN()};
		if (found != nullptr && index < found->shape.vertices.size())
		{
			auto const &vertex{found->shape.vertices[index]};
			got = key == "radius" ? vertex.radius : key == "in" ? vertex.transition_in : key == "out" ? vertex.transition_out : vertex.cant;
		}
		expectation(std::abs(got - wanted) <= 1e-6, format("modelvertex %u %zu %s", line, index, key.c_str()), format("%.6f", got), format("%.6f", wanted));
		return selftest_result::handled;
	}
	if (what == "modelsame")
	{
		std::string label;
		words >> label;
		auto const found{model().marks.find(label)};
		auto const same{found != model().marks.end() && found->second.same(model().session.network())};
		expectation(same, "modelsame " + label, same ? "the same" : "another network", "the network marked " + label);
		return selftest_result::handled;
	}
	words.clear();
	words.seekg(start);
	return selftest_result::unhandled;
}
