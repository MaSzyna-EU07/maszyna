/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/project/projectSession.hpp"
#include "editor/project/projectGenerated.hpp"
#include "editor/project/projectMigration.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

namespace project
{

void differences(layout::network const &Before, layout::network const &After, std::set<layout::line_id> &Changed, std::set<layout::line_id> &Removed)
{
	for (auto const &[id, line] : After.lines())
		if (auto const *earlier{Before.find(id)}; earlier == nullptr || false == (*earlier == line))
			Changed.insert(id);
	for (auto const &[id, line] : Before.lines())
		if (After.find(id) == nullptr)
			Removed.insert(id);
}

session::open_result session::open(std::filesystem::path const &Folder, std::string const &Map, norms::area_map const &Areas)
{
	close();
	open_result result;
	auto opened{open_project(Folder, Map, Areas)};
	result.migrated = opened.migrated;
	result.notes = std::move(opened.notes);
	if (false == opened.project.project.has_value())
	{
		result.error = opened.project.error;
		return result;
	}
	result.notes.insert(result.notes.end(), opened.project.notes.begin(), opened.project.notes.end());
	m_project = std::move(*opened.project.project);
	m_folder = Folder;
	m_map = Map;
	// the tracks written by the generator, as the files have them now: one changed or gone outside the editor marks its line
	result.hand_changes = find_hand_changes(m_project, [&](std::string const &File) -> std::optional<std::string> {
		std::ifstream stream{m_folder / File, std::ios::binary};
		if (false == stream.is_open())
			return std::nullopt;
		return std::string{std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
	});
	mark_hand_changes(m_project, result.hand_changes);
	m_model = layout::model{m_project.network};
	m_open = true;
	result.done = true;
	// a map edited on the model has its project file from the start, also with nothing to migrate, of which open_project() writes none
	if (std::error_code missing; false == std::filesystem::exists(file(), missing))
		if (std::string error; false == save(error))
			result.notes.push_back("the project file isn't written: " + error);
	return result;
}

void session::close()
{
	m_open = false;
	m_folder.clear();
	m_map.clear();
	m_project = {};
	m_model = {};
	m_done.clear();
	m_undone.clear();
}

std::filesystem::path session::file() const
{
	return path_of(m_folder / m_map);
}

session::effect session::trial(layout::command const &Command, layout::edit_policy const Policy) const
{
	effect result;
	layout::model copy{m_model.state()};
	result.outcome = copy.execute(Command, Policy);
	if (result.outcome.done)
		differences(m_model.state(), copy.state(), result.changed, result.removed);
	return result;
}

std::set<layout::line_id> session::hand_changed(effect const &Effect, std::set<layout::line_id> const &Seen) const
{
	std::set<layout::line_id> result;
	for (auto const *lines : {&Effect.changed, &Effect.removed})
		for (auto const line : *lines)
			if (false == may_overwrite(m_project, line) || Seen.contains(line))
				result.insert(line);
	return result;
}

session::applied session::execute(layout::command const &Command, std::set<layout::line_id> const &Overwrite, layout::edit_policy const Policy)
{
	applied result;
	auto const before{m_model.state()};
	result.outcome = m_model.execute(Command, Policy);
	if (false == result.outcome.done)
		return result;
	differences(before, m_model.state(), result.changed, result.removed);
	step kept;
	kept.token = ++m_last_token;
	kept.hand_before = m_project.hand_changed;
	for (auto const line : Overwrite)
		m_project.hand_changed.erase(line);
	kept.hand_after = m_project.hand_changed;
	m_done.push_back(std::move(kept));
	// the model forgets the steps taken back once a command is kept: so do the tokens
	m_undone.clear();
	result.token = m_last_token;
	return result;
}

bool session::undo(std::uint64_t const Token, effect *Effect)
{
	if (m_done.empty() || m_done.back().token != Token)
		return false;
	auto const before{m_model.state()};
	if (false == m_model.undo())
		return false;
	auto taken{std::move(m_done.back())};
	m_done.pop_back();
	m_project.hand_changed = taken.hand_before;
	if (Effect != nullptr)
	{
		*Effect = {};
		Effect->outcome.done = true;
		differences(before, m_model.state(), Effect->changed, Effect->removed);
	}
	m_undone.push_back(std::move(taken));
	return true;
}

bool session::redo(std::uint64_t const Token, effect *Effect)
{
	if (m_undone.empty() || m_undone.back().token != Token)
		return false;
	auto const before{m_model.state()};
	if (false == m_model.redo())
		return false;
	auto taken{std::move(m_undone.back())};
	m_undone.pop_back();
	m_project.hand_changed = taken.hand_after;
	if (Effect != nullptr)
	{
		*Effect = {};
		Effect->outcome.done = true;
		differences(before, m_model.state(), Effect->changed, Effect->removed);
	}
	m_done.push_back(std::move(taken));
	return true;
}

layout::generated_line session::generate(layout::line_id const Line) const
{
	return layout::generate(m_model.state(), Line, m_options);
}

void session::record_written(layout::line_id const Line, std::vector<written_piece> const &Pieces, std::string const &File)
{
	layout::generated_line written;
	written.line = Line;
	for (auto const &piece : Pieces)
	{
		layout::generated_piece entry;
		entry.name = piece.name;
		entry.text = piece.text;
		written.pieces.push_back(std::move(entry));
	}
	// the scenery writes only the tracks changed since it was saved: the records of the other tracks the line still has stay as they
	// were, or after a track moved and moved back the line would keep a record of that one track alone, and own only it when opened
	std::set<std::string> current;
	for (auto const &piece : generate(Line).pieces)
		current.insert(piece.name);
	auto const written_now = [&Pieces](std::string const &Name) { return std::ranges::any_of(Pieces, [&Name](written_piece const &Piece) { return Piece.name == Name; }); };
	std::vector<generated_node> kept;
	for (auto const &node : m_project.generated)
		if (node.kind == "track" && node.line == Line && current.contains(node.name) && false == written_now(node.name))
			kept.push_back(node);
	record_generated(m_project, written, File);
	m_project.generated.insert(m_project.generated.end(), kept.begin(), kept.end());
}

void session::mark_hand_changed(layout::line_id const Line)
{
	m_project.hand_changed.insert(Line);
}

bool session::save(std::string &Error)
{
	if (false == m_open)
	{
		Error = "no project is open";
		return false;
	}
	auto const &now{m_model.state()};
	// the tracks of the lines gone went from the scenery with them
	std::erase_if(m_project.generated, [&](generated_node const &Node) { return now.find(Node.line) == nullptr; });
	for (auto line{m_project.hand_changed.begin()}; line != m_project.hand_changed.end();)
		line = now.find(*line) == nullptr ? m_project.hand_changed.erase(line) : std::next(line);
	m_project.network = now;
	return project::save(m_project, file(), Error);
}

} // namespace project
