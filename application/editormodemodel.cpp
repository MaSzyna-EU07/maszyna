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

// what the entry of the track has between its length and its path: the width, the friction, the sound distance, the quality, the damage,
// the environment and the visibility with the materials; empty where it isn't a plain track
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

editor_mode::selftest_result editor_mode::selftest_model(std::string const &command, std::istringstream &words)
{
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
	if (command != "expect")
		return selftest_result::unhandled;
	auto const start{words.tellg()};
	std::string what;
	words >> what;
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
	words.clear();
	words.seekg(start);
	return selftest_result::unhandled;
}
