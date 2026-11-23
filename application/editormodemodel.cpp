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
#include "application/editormodeselftest.h"
#include "application/editormodeshared.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorSettings.hpp"
#include "editor/project/projectGenerated.hpp"
#include "editor/project/projectSession.hpp"
#include "editor/project/sceneryText.hpp"
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

struct editor_model
{
	project::session session;
	bool tried{false}; // the project opened for the scenery, or failed to
	std::string error;
	std::unordered_map<scene::basic_node const *, editor_mode::managed_track> tracks;
	std::string message; // what the last command came to: why it was refused, or what the norms tell of it
	bool refused{false};
	std::vector<std::string> notices;
	std::vector<std::pair<TTrack const *, std::string>> written; // by the save under way, the texts written
};

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
	return model().notices;
}

std::string editor_mode::model_error() const
{
	return model().error;
}

void editor_mode::model_forget(scene::basic_node const *Node)
{
	if (m_model != nullptr)
		m_model->tracks.erase(Node);
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
		if (node.kind != "track")
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
		if (found->second.has_value())
			if (auto const text{scenery_text::node_text(*found->second, node.name)})
				entry.text = *text;
		if (auto const *source{scene::Layers.source(track)})
			entry.offset = source->context.offset;
		entry.baseline = export_of(*track);
		model().tracks[track] = std::move(entry);
	}
	scene::Layers.path_writer = [this](TTrack const &Track, glm::dvec3 const &Offset) { return model_path_text(Track, Offset); };
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
	if (Refused)
	{
		model().message = STR("Refused, nothing changed:");
		for (auto const &reason : Outcome.reasons)
			model().message += "\n" + reason;
		WriteLog("Editor: model: " + editor_selftest::one_line(model().message), logtype::generic);
		return;
	}
	model().message = What;
	for (auto const &notice : Outcome.notices)
		if (std::ranges::find(model().notices, notice.text) == model().notices.end())
			model().notices.push_back(notice.text);
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
		std::set<layout::line_id> overwrite;
		for (auto const *lines : {&effect.changed, &effect.removed})
			for (auto const line : *lines)
				if (Context.overwrite.contains(line))
					overwrite.insert(line);
		auto const applied{session.execute(command, overwrite)};
		if (false == applied.outcome.done)
		{
			model_told(applied.outcome, true);
			return;
		}
		created = model_tracks_after(applied.token, applied.changed, applied.removed, Context);
		std::string what{format(STR_C("%s: done"), STR_C(layout::command_name(command)))};
		if (false == applied.changed.empty())
			what = format(STR_P("%s: line %s, %zu track", "%s: line %s, %zu tracks", static_cast<long long>(created.size())), STR_C(layout::command_name(command)), line_name(*applied.changed.begin()).c_str(), created.size());
		model_told(applied.outcome, false, what);
		Commands.erase(Commands.begin());
	}
	if (Context.done)
		Context.done(created);
}

std::vector<TTrack *> editor_mode::model_tracks_after(std::uint64_t const Token, std::set<layout::line_id> const &Changed, std::set<layout::line_id> const &Removed, model_context const &Context)
{
	std::vector<TTrack *> removed;
	auto surface{Context.surface};
	for (auto const *lines : {&Changed, &Removed})
		for (auto const line : *lines)
			for (auto *track : model_line_tracks(line))
			{
				if (surface.empty())
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
	for (auto const line : Changed)
	{
		auto const made{model().session.generate(line)};
		for (auto const &error : made.errors)
			WriteLog("Editor: model: the tracks of line " + line_name(line) + ": " + error, logtype::generic);
		std::size_t index{0};
		for (auto const &piece : made.pieces)
		{
			auto const at{piece.text.find(" track ")};
			if (at == std::string::npos)
				continue;
			auto *track{editor_track::load_entry(piece.text.substr(at + 7), piece.name)};
			if (track == nullptr)
				continue;
			managed_track entry;
			entry.line = line;
			entry.index = index++;
			entry.text = piece.text;
			if (track->name() != piece.name)
			{
				// another track has the name: the entry written under the one it got
				WriteLog("Editor: model: the name " + piece.name + " is taken, the track is " + track->name(), logtype::generic);
				entry.text = "node -1 0 " + track->name() + piece.text.substr(at);
			}
			created.push_back(track);
			entries.emplace_back(track, std::move(entry));
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
	// the tracks written, by their lines and the files they went into
	std::map<std::pair<layout::line_id, std::string>, std::vector<project::session::written_piece>> lines;
	for (auto const &[track, text] : model().written)
	{
		auto const found{model().tracks.find(track)};
		auto const *source{scene::Layers.source(track)};
		if (found == model().tracks.end() || source == nullptr)
			continue;
		lines[{found->second.line, scene::Layers.layer(source->layer).name}].push_back({track->name(), text});
	}
	model().written.clear();
	for (auto const &[key, pieces] : lines)
		model().session.record_written(key.first, pieces, key.second);
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

bool editor_mode::model_plan_convert(std::string &Why)
{
	if (false == model_on())
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
	{
		model_context context;
		context.replaced = tracks;
		context.surface = surface_of(*tracks.front());
		context.done = [this](std::vector<TTrack *> const &) { route_drop(); };
		model_run({line}, context);
	}
	return true;
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
			auto const tracks{model_line_tracks(id)};
			auto const *start{network.find(line.start)};
			auto const *end{network.find(line.end)};
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
	words.clear();
	words.seekg(start);
	return selftest_result::unhandled;
}
