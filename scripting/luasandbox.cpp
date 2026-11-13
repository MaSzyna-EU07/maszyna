/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scripting/luasandbox.h"

#include "scripting/scriptscreenrules.h"

#include <filesystem>
#include <fstream>
#include <system_error>

namespace lua_sandbox
{

namespace
{

void open_library(lua_State *L, lua_CFunction Open, char const *Name)
{
	lua_pushcfunction(L, Open);
	lua_pushstring(L, Name);
	lua_call(L, 1, 0);
}

// the conversions of strftime in C89, which every C library knows. LuaJIT hands each one to strftime on its own
constexpr std::string_view date_conversions{"aAbBcdHIjmMpSUwWxXyYZ%"};

// os.date(format, time), the library's own (the upvalue) for a format of known conversions only: the Microsoft
// C library ends the program on one it doesn't know ("%Q", "%#")
int api_date(lua_State *L)
{
	std::size_t length{0};
	auto const *format{luaL_optlstring(L, 1, "%c", &length)};
	std::string_view const text{format, length};
	for (std::size_t idx = 0; idx + 1 < text.size(); ++idx)
	{
		if (text[idx] != '%')
		{
			continue;
		}
		++idx;
		if (date_conversions.find(text[idx]) == std::string_view::npos)
		{
			return luaL_error(L, "os.date: unknown conversion '%%%c'", text[idx]);
		}
	}
	lua_pushvalue(L, lua_upvalueindex(1));
	lua_insert(L, 1);
	lua_call(L, lua_gettop(L) - 1, LUA_MULTRET);
	return lua_gettop(L);
}

} // namespace

void open_safe_libraries(lua_State *L)
{
	open_library(L, luaopen_base, "");
	open_library(L, luaopen_table, LUA_TABLIBNAME);
	open_library(L, luaopen_string, LUA_STRLIBNAME);
	open_library(L, luaopen_math, LUA_MATHLIBNAME);
	open_library(L, luaopen_bit, LUA_BITLIBNAME);
	// what reaches files or loads code
	for (auto const *name : {"dofile", "loadfile", "load", "loadstring", "module", "require"})
	{
		lua_pushnil(L);
		lua_setglobal(L, name);
	}
	lua_getglobal(L, LUA_STRLIBNAME);
	lua_pushnil(L);
	lua_setfield(L, -2, "dump");
	lua_pop(L, 1);
}

void open_clock_library(lua_State *L)
{
	// the whole library, of which a new table takes what only reads the clock
	open_library(L, luaopen_os, LUA_OSLIBNAME);
	lua_getglobal(L, LUA_OSLIBNAME);
	lua_newtable(L);
	for (auto const *name : {"time", "clock", "difftime"})
	{
		lua_getfield(L, -2, name);
		lua_setfield(L, -2, name);
	}
	lua_getfield(L, -2, "date");
	lua_pushcclosure(L, api_date, 1);
	lua_setfield(L, -2, "date");
	// the new table in place of the library: the global, and the one the libraries know as loaded, which
	// luaL_openlibs fills up again rather than make a new global
	lua_pushvalue(L, -1);
	lua_setglobal(L, LUA_OSLIBNAME);
	lua_getfield(L, LUA_REGISTRYINDEX, "_LOADED");
	lua_pushvalue(L, -2);
	lua_setfield(L, -2, LUA_OSLIBNAME);
	// _LOADED, the new table, the library
	lua_pop(L, 3);
}

bool load_text_chunk(lua_State *L, std::string_view const Code, std::string const &Name)
{
	if (false == Code.empty() && Code.front() == '\x1b')
	{
		lua_pushfstring(L, "%s: precompiled scripts aren't loaded", Name.c_str());
		return false;
	}
	return 0 == luaL_loadbufferx(L, Code.data(), Code.size(), ("@" + Name).c_str(), "t");
}

std::optional<std::string> read_script_file(std::string const &Folder, std::string_view const Path)
{
	if (false == script_screens::is_safe_relative_path(Path))
	{
		return std::nullopt;
	}
	// a folder or a device (CON, NUL, COM1...) isn't a regular file. in the code page of the system, as the game opens
	// its files
	std::filesystem::path const path{Folder + std::string{Path}};
	std::error_code error;
	if (false == std::filesystem::is_regular_file(path, error))
	{
		return std::nullopt;
	}
	auto const size{std::filesystem::file_size(path, error)};
	if (error || size > max_script_file)
	{
		return std::nullopt;
	}
	std::ifstream file{path, std::ios::binary};
	if (false == file.is_open())
	{
		return std::nullopt;
	}
	std::string contents(static_cast<std::size_t>(size), '\0');
	file.read(contents.data(), static_cast<std::streamsize>(contents.size()));
	// what was read, should the file have shrunk meanwhile
	contents.resize(static_cast<std::size_t>(file.gcount()));
	return contents;
}

std::optional<std::string> run_script_file(lua_State *L, std::string const &Folder, std::string_view const Path)
{
	auto const name{Folder + std::string{Path}};
	auto const code{read_script_file(Folder, Path)};
	if (false == code.has_value())
	{
		return name + ": not found, not a file, larger than " + std::to_string(max_script_file) +
		       " bytes, or outside the folder (an absolute path, a drive or '..')";
	}
	if (false == load_text_chunk(L, *code, name) || 0 != lua_pcall(L, 0, 0, 0))
	{
		std::size_t length{0};
		auto const *message{lua_tolstring(L, -1, &length)};
		std::string result{message != nullptr ? std::string{message, length} : name + ": unknown error"};
		lua_pop(L, 1);
		return result;
	}
	return std::nullopt;
}

} // namespace lua_sandbox
