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

#include <cstdint>
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

// the file Path of Folder, when Path is a relative path inside Folder (no '..', no drive, no leading separator) and
// names a regular file of at most max_script_file bytes. nothing otherwise
std::optional<std::string> read_script_file(std::string const &Folder, std::string_view Path);

// reads the script Path of Folder (as read_script_file), loads it as text and runs it in protected mode. nothing when
// it ran, otherwise why it didn't
std::optional<std::string> run_script_file(lua_State *L, std::string const &Folder, std::string_view Path);

} // namespace lua_sandbox
