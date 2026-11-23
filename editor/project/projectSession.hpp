/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutCommands.hpp"
#include "editor/layout/layoutGenerator.hpp"
#include "editor/project/projectFile.hpp"
#include "editor/project/projectGenerated.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

// The project of the map open in the editor, edited through the commands of the model (editor-tracks-domain.md §3, §4, §9; stage E4):
// the project file read when the map is opened (made of its //$ lines the first time) and written with the scenery, the model of the
// track geometry its commands edit, and what the editor needs around them: which lines a command lays anew or takes away, so their tracks
// are made again or go; which lines were changed by hand, so they aren't written over unasked; the steps of the model's history, which
// the editor's history takes back and brings again by their tokens, one step of the editor a command. No scene: the editor puts the tracks
// the generator makes into its scene and tells which it wrote into which file.
namespace project
{

// the lines which differ between two states of the network: Changed, in the second one and not the same there (laid anew, or new), and
// Removed, gone from it
void differences(layout::network const &Before, layout::network const &After, std::set<layout::line_id> &Changed, std::set<layout::line_id> &Removed);

class session
{
  public:
	struct open_result
	{
		bool done{false};
		bool migrated{false}; // there was no project file: made of the //$ lines of the files of the map and written
		std::string error; // why there's no project: a file of a newer schema, one which can't be read or written
		std::vector<std::string> notes; // of the migration, and the lines the file has which can't be laid as written
		std::vector<hand_change> hand_changes; // the tracks of the generator changed in the files since they were written
	};
	// the project of the map (relative to the folder of the sceneries): its file read, or made of the //$ lines of the map; the tracks the
	// generator wrote compared with the files as they are now, the lines of the ones changed outside the editor marked as changed by hand
	open_result open(std::filesystem::path const &Folder, std::string const &Map, norms::area_map const &Areas = {});
	void close();
	bool is_open() const { return m_open; }
	std::filesystem::path const &folder() const { return m_folder; }
	std::string const &map() const { return m_map; }
	// <map>.eu07proj next to the map
	std::filesystem::path file() const;

	layout::network const &network() const { return m_model.state(); }
	// the generated tracks, the lines changed by hand, the //$ lines kept; its network is the one at the last open or save, network() is now
	document const &project() const { return m_project; }

	// what a command does
	struct effect
	{
		layout::outcome outcome;
		std::set<layout::line_id> changed; // laid anew or new: their tracks are made again
		std::set<layout::line_id> removed; // gone: their tracks go
	};
	// the command tried on a copy of the network, nothing of the session changed: refused with the reasons, or the lines it changes
	effect trial(layout::command const &Command, layout::edit_policy const Policy = layout::edit_policy::refuse_block) const;
	// the lines of the effect, changed or taken away, which may not be written over unasked: the ones the project marks as changed by hand
	// (found so in the files) and the ones given (whose tracks the editor sees changed by other tools since they were made)
	std::set<layout::line_id> hand_changed(effect const &Effect, std::set<layout::line_id> const &Seen = {}) const;

	struct applied : effect
	{
		std::uint64_t token{0}; // of the step of the history kept; 0 for a command refused
	};
	// the command carried out: the lines of Overwrite may be written over, their marks of a change by hand taken off with it. A command
	// kept is a step of the history, the marks with it; refused, nothing changes
	applied execute(layout::command const &Command, std::set<layout::line_id> const &Overwrite = {}, layout::edit_policy const Policy = layout::edit_policy::refuse_block);
	// the step of the token taken back, or brought again; only the latest one kept (undo) or taken back (redo) can be: false for another one,
	// nothing changed. The lines it changes, as the effect of a command tells them
	bool undo(std::uint64_t const Token, effect *Effect = nullptr);
	bool redo(std::uint64_t const Token, effect *Effect = nullptr);
	std::size_t undoable() const { return m_done.size(); }
	std::size_t redoable() const { return m_undone.size(); }

	// the tracks of the line as the generator makes them now, named as the options name them
	layout::generated_line generate(layout::line_id const Line) const;
	// the tracks of the simulator the turnout is made of now: its switch tracks, and the plain tracks of a double switch, a slip or a crossing
	layout::generated_turnout generate(layout::turnout_id const Turnout) const;
	layout::generator_options &options() { return m_options; }
	layout::generator_options const &options() const { return m_options; }

	// a piece written into a scenery file: its name and the text written, the coordinates those of the file
	struct written_piece
	{
		std::string name;
		std::string text;
		std::string file; // where it went, of the folder of the sceneries; empty: the one given with the pieces
		std::string kind{"track"}; // "switch" for the switch track of a turnout, kept by its main track
	};
	// the tracks of the line written with the scenery into the file, as they were written: the project records them, by the checksums of
	// their text, in the place of the ones it had of the line, and doesn't take the line for changed by hand any more
	void record_written(layout::line_id const Line, std::vector<written_piece> const &Pieces, std::string const &File);
	// the same, each piece with the file it went into: the tracks of a line may go into more files than one
	void record_written(layout::line_id const Line, std::vector<written_piece> const &Pieces);
	// the line's tracks were changed by other tools: so marked in the project, not written over unasked from now on
	void mark_hand_changed(layout::line_id const Line);
	// the project written next to the map, with the network as it is now and the records of the generated tracks of the lines gone taken out
	bool save(std::string &Error);
	// the assumptions of the project (editorProjectSetup.hpp), the line of their record, kept in the project from now on, written with it;
	// the profile of the scenery the model judges the lines by with them, a change of it not a step of the history
	void setup(std::string Line, norms::profile_id const Profile);

  private:
	struct step
	{
		std::uint64_t token{0};
		std::set<layout::line_id> hand_before;
		std::set<layout::line_id> hand_after;
	};
	bool m_open{false};
	std::filesystem::path m_folder;
	std::string m_map;
	document m_project;
	layout::model m_model;
	layout::generator_options m_options;
	std::vector<step> m_done;
	std::vector<step> m_undone;
	std::uint64_t m_last_token{0};
};

} // namespace project
