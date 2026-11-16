/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// what the Lua states of the game share to run scripts sandboxed: the scenery scripts (scripting/lua.cpp, lua in the
// .scn) and the script screens (scripting/luaprogram.cpp). a sandboxed script reaches no files, no other programs, no
// ffi and no debug library, and loads no code of its own: no require, dofile, load or precompiled chunks

#include <lua.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace lua_sandbox
{

// the largest script file a scenery loads
constexpr std::uintmax_t max_script_file{16 * 1024 * 1024};

// the libraries a sandboxed script gets: base, table, string, math and bit, without dofile, loadfile, load,
// loadstring, module, require and string.dump
void open_safe_libraries(lua_State *L);

// the os table with only what reads the clock: os.time, os.clock, os.difftime and os.date, the last refusing the
// conversions the C library doesn't know (the Microsoft one ends the program on them)
void open_clock_library(lua_State *L);

// loads Code as text, under the name Name: precompiled code is refused. leaves the chunk, or an error message, on the
// stack
bool load_text_chunk(lua_State *L, std::string_view Code, std::string const &Name);

// Path with each '\' turned into '/', as the game reads the paths in its data (replace_slashes): a scenery made on
// Windows may name its folders either way, and on Linux only '/' separates them
std::string with_forward_slashes(std::string_view Path);

// the file Path of Folder, when Path is a relative path inside Folder (no '..', no drive, no leading separator) and
// names a regular file of at most max_script_file bytes. nothing otherwise. '\' separates folders in Path as '/' does,
// on every system
std::optional<std::string> read_script_file(std::string const &Folder, std::string_view Path);

// reads the script Path of Folder (as read_script_file), loads it as text and runs it in protected mode. nothing when
// it ran, otherwise why it didn't
std::optional<std::string> run_script_file(lua_State *L, std::string const &Folder, std::string_view Path);

// the files a scenery script reads and writes itself (eu07.read_file, eu07.write_file, eu07.read_data_file): it reads
// the files of the scenery folder, and writes only into a data folder of its scenery, outside the game

// the largest file a script writes, all the files of a data folder together, and how many files a data folder holds
constexpr std::uintmax_t max_data_file{1024 * 1024};
constexpr std::uintmax_t max_data_folder{16 * 1024 * 1024};
constexpr std::size_t max_data_files{256};
// the longest path a script writes to, and how many folders deep it goes
constexpr std::size_t max_data_path{200};
constexpr std::size_t max_data_depth{8};

// the file Path of Folder, when Path is a relative path inside Folder (as read_script_file) and names a regular file of
// at most Limit bytes. nothing otherwise
std::optional<std::string> read_file(std::filesystem::path const &Folder, std::string_view Path, std::uintmax_t Limit);

// true for a path a script may write to: a relative path inside the folder (is_safe_relative_path), '/' or '\' between
// the folders, at most max_data_path bytes and max_data_depth folders deep, each name portable: not empty, not '.' or
// '..', none of the characters Windows refuses (: < > " | ? * and the control characters), no '.' or ' ' at the end,
// and not a device of Windows (con, prn, aux, nul, com0-9, lpt0-9, conin$, conout$, with an extension or without)
bool is_safe_write_path(std::string_view Path);

// writes Contents to the file Path of Folder (a data folder), replacing what was there, and makes Folder and the folders
// of Path. refused for a path that isn't safe (is_safe_write_path), for more than max_data_file bytes, when Folder would
// hold more than max_data_files files or max_data_folder bytes, when Path names something else than a file, and when it
// leads out of Folder through a link. nothing when written, otherwise why not
std::optional<std::string> write_data_file(std::filesystem::path const &Folder, std::string_view Path, std::string_view Contents);

// the data folder of the scenery Scenery (its .scn file, relative to the scenery folder) in Root: Root and the file
// without ".scn" (td.scn: Root/td). empty when Root is, or the name isn't a safe path (is_safe_write_path)
std::filesystem::path scenery_data_folder(std::filesystem::path const &Root, std::string_view Scenery);

// where the file functions of a scenery script read and write
struct file_folders
{
	// the scenery folder, which eu07.read_file reads, ending with a separator. empty: nothing is read
	std::string scenery;
	// the data folder of the scenery, which eu07.write_file writes and eu07.read_data_file reads. empty: nothing is
	// written there or read from it
	std::filesystem::path data;
};

// adds eu07.read_file(path), eu07.write_file(path, text) and eu07.read_data_file(path) to the state, on the folders of
// Folders, read on every call: Folders is to outlive the state
void open_file_library(lua_State *L, file_folders const *Folders);

// the objects of the simulation a scenery script holds (eu07.events): each kind is a userdata of a metatable of its
// own, so a function given the handle of another kind, a light userdata or anything else raises a Lua error, and never
// takes one object for another. the objects outlive the state (the scenery's events, tracks, memcells and isolated
// track circuits, and its vehicles, which the game takes off the tracks but doesn't delete)
enum class handle_kind
{
	event,
	track,
	isolated,
	vehicle,
	memcell
};

// the name of the metatable of the kind, as the errors give it ("eu07.memcell expected, got userdata")
char const *handle_name(handle_kind Kind);

// makes the metatables of the handles; once, before the first handle. a script can't reach them (getmetatable gives
// false)
void open_handles(lua_State *L);

// pushes the handle of Object, of the kind, or nil for nullptr. one object has one handle while the script holds it:
// the handles of an object found twice are equal, and the same key of a table
void push_handle(lua_State *L, handle_kind Kind, void *Object);

// the object of the handle at Index: nullptr for nil or nothing (what the find functions give for a missing object),
// a Lua error (bad argument) for anything else than a handle of the kind
void *check_handle(lua_State *L, int Index, handle_kind Kind);

template <typename Type_> Type_ *check_handle(lua_State *L, int const Index, handle_kind const Kind)
{
	return static_cast<Type_ *>(check_handle(L, Index, Kind));
}

} // namespace lua_sandbox
