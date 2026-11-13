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

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <utility>
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

std::string with_forward_slashes(std::string_view const Path)
{
	std::string result{Path};
	std::ranges::replace(result, '\\', '/');
	return result;
}

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

std::optional<std::string> read_file(std::filesystem::path const &Folder, std::string_view const Path, std::uintmax_t const Limit)
{
	// a NUL would end the name the system sees
	if (false == script_screens::is_safe_relative_path(Path) || Path.find('\0') != std::string_view::npos)
	{
		return std::nullopt;
	}
	// a folder or a device (CON, NUL, COM1...) isn't a regular file. in the code page of the system, as the game opens
	// its files
	auto const path{Folder / with_forward_slashes(Path)};
	std::error_code error;
	if (false == std::filesystem::is_regular_file(path, error))
	{
		return std::nullopt;
	}
	auto const size{std::filesystem::file_size(path, error)};
	if (error || size > Limit)
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

std::optional<std::string> read_script_file(std::string const &Folder, std::string_view const Path)
{
	return read_file(std::filesystem::path{Folder}, Path, max_script_file);
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

namespace
{

char lower_case(char const Character)
{
	return Character >= 'A' && Character <= 'Z' ? static_cast<char>(Character - 'A' + 'a') : Character;
}

// the names Windows opens as its devices, whatever the extension
bool is_device_name(std::string_view Name)
{
	// the extension, and the spaces before it, don't count ("nul.txt", "con .log")
	Name = Name.substr(0, Name.find('.'));
	while (false == Name.empty() && Name.back() == ' ')
	{
		Name.remove_suffix(1);
	}
	std::string name;
	std::ranges::transform(Name, std::back_inserter(name), lower_case);
	if (name == "con" || name == "prn" || name == "aux" || name == "nul" || name == "conin$" || name == "conout$")
	{
		return true;
	}
	return name.size() == 4 && (name.starts_with("com") || name.starts_with("lpt")) && name[3] >= '0' && name[3] <= '9';
}

// a name of a file or a folder that every system takes as it is
bool is_portable_name(std::string_view const Name)
{
	if (Name.empty() || Name == "." || Name == ".." || Name.back() == '.' || Name.back() == ' ')
	{
		return false;
	}
	for (auto const character : Name)
	{
		if (static_cast<unsigned char>(character) < 0x20 || character == 0x7f ||
		    std::string_view{"<>:\"|?*"}.find(character) != std::string_view::npos)
		{
			return false;
		}
	}
	return false == is_device_name(Name);
}

// true when Path, a canonical path, is Folder, a canonical path, or inside it
bool is_inside(std::filesystem::path const &Path, std::filesystem::path const &Folder)
{
	return std::mismatch(Folder.begin(), Folder.end(), Path.begin(), Path.end()).first == Folder.end();
}

file_folders const &folders_of(lua_State *L)
{
	return *static_cast<file_folders const *>(lua_touserdata(L, lua_upvalueindex(1)));
}

std::string_view check_text(lua_State *L, int const Index)
{
	std::size_t length{0};
	auto const *text{luaL_checklstring(L, Index, &length)};
	return {text, length};
}

void push_contents(lua_State *L, std::optional<std::string> const &Contents)
{
	if (Contents.has_value())
	{
		lua_pushlstring(L, Contents->data(), Contents->size());
	}
	else
	{
		lua_pushnil(L);
	}
}

// eu07.read_file(path): the contents of a file of the scenery folder, or nil
int api_read_file(lua_State *L)
{
	auto const path{check_text(L, 1)};
	auto const &folders{folders_of(L)};
	push_contents(L, folders.scenery.empty() ? std::nullopt : read_file(folders.scenery, path, max_script_file));
	return 1;
}

// eu07.read_data_file(path): the contents of a file of the scenery's data folder, or nil
int api_read_data_file(lua_State *L)
{
	auto const path{check_text(L, 1)};
	auto const &folders{folders_of(L)};
	push_contents(L, folders.data.empty() ? std::nullopt : read_file(folders.data, path, max_data_file));
	return 1;
}

// eu07.write_file(path, text): writes the file of the scenery's data folder. true, or nil and why not
int api_write_file(lua_State *L)
{
	auto const path{check_text(L, 1)};
	auto const contents{check_text(L, 2)};
	if (auto const error{write_data_file(folders_of(L).data, path, contents)}; error.has_value())
	{
		lua_pushnil(L);
		lua_pushlstring(L, error->data(), error->size());
		return 2;
	}
	lua_pushboolean(L, 1);
	return 1;
}

} // namespace

bool is_safe_write_path(std::string_view const Path)
{
	if (Path.size() > max_data_path || false == script_screens::is_safe_relative_path(Path))
	{
		return false;
	}
	std::size_t names{0};
	std::size_t start{0};
	while (start <= Path.size())
	{
		auto const end{std::min(Path.find_first_of("/\\", start), Path.size())};
		if (false == is_portable_name(Path.substr(start, end - start)))
		{
			return false;
		}
		++names;
		start = end + 1;
	}
	// the folders, and the file in them
	return names <= max_data_depth + 1;
}

std::optional<std::string> write_data_file(std::filesystem::path const &Folder, std::string_view const Path, std::string_view const Contents)
{
	namespace fs = std::filesystem;
	std::string const name{Path};
	if (Folder.empty())
	{
		return name + ": no folder to write to (the game knows no folder of the player)";
	}
	if (false == is_safe_write_path(Path))
	{
		return name + ": not a path to write to (a relative path of portable names, at most " + std::to_string(max_data_path) +
		       " bytes and " + std::to_string(max_data_depth) + " folders deep, without '..')";
	}
	if (Contents.size() > max_data_file)
	{
		return name + ": larger than " + std::to_string(max_data_file) + " bytes";
	}
	auto const target{Folder / with_forward_slashes(Path)};
	std::error_code error;
	// a link, a folder or anything else that isn't a file stays as it is
	auto const status{fs::symlink_status(target, error)};
	auto const exists{status.type() != fs::file_type::not_found};
	if (exists && status.type() != fs::file_type::regular)
	{
		return name + ": not a file";
	}
	std::uintmax_t replaced{0};
	if (exists)
	{
		replaced = fs::file_size(target, error);
		if (error)
		{
			return name + ": can't be replaced";
		}
	}
	// what the folder holds, and will hold with the file in place of what was there
	std::uintmax_t held{0};
	std::size_t files{0};
	if (fs::is_directory(Folder, error))
	{
		for (fs::recursive_directory_iterator entry{Folder, error}, end; !error && entry != end; entry.increment(error))
		{
			std::error_code entryerror;
			if (entry->symlink_status(entryerror).type() == fs::file_type::regular)
			{
				auto const size{entry->file_size(entryerror)};
				held += entryerror ? 0 : size;
				++files;
			}
		}
		if (error)
		{
			return name + ": the data folder can't be read";
		}
	}
	if (files + (exists ? 0 : 1) > max_data_files)
	{
		return name + ": the data folder holds " + std::to_string(max_data_files) + " files already";
	}
	if (held - std::min(replaced, held) + Contents.size() > max_data_folder)
	{
		return name + ": the data folder would hold more than " + std::to_string(max_data_folder) + " bytes";
	}
	fs::create_directories(target.parent_path(), error);
	if (error)
	{
		return name + ": its folder can't be made";
	}
	// still inside the data folder, should a link in it lead elsewhere
	auto const root{fs::canonical(Folder, error)};
	auto const parent{error ? fs::path{} : fs::canonical(target.parent_path(), error)};
	if (error || false == is_inside(parent, root))
	{
		return name + ": outside the data folder";
	}
	std::ofstream file{parent / target.filename(), std::ios::binary | std::ios::trunc};
	if (false == file.is_open())
	{
		return name + ": can't be opened to write";
	}
	file.write(Contents.data(), static_cast<std::streamsize>(Contents.size()));
	file.close();
	if (file.fail())
	{
		return name + ": couldn't be written whole";
	}
	return std::nullopt;
}

std::filesystem::path scenery_data_folder(std::filesystem::path const &Root, std::string_view const Scenery)
{
	auto name{with_forward_slashes(Scenery)};
	if (name.size() >= 4)
	{
		std::string extension;
		std::ranges::transform(std::string_view{name}.substr(name.size() - 4), std::back_inserter(extension), lower_case);
		if (extension == ".scn")
		{
			name.resize(name.size() - 4);
		}
	}
	if (Root.empty() || false == is_safe_write_path(name))
	{
		return {};
	}
	return Root / name;
}

void open_file_library(lua_State *L, file_folders const *Folders)
{
	// eu07, which the scenery scripts have for eu07.events, made when there is none
	lua_getglobal(L, "eu07");
	if (false == lua_istable(L, -1))
	{
		lua_pop(L, 1);
		lua_newtable(L);
		lua_pushvalue(L, -1);
		lua_setglobal(L, "eu07");
	}
	for (auto const &[name, function] : {std::pair{"read_file", api_read_file}, std::pair{"write_file", api_write_file},
	                                     std::pair{"read_data_file", api_read_data_file}})
	{
		lua_pushlightuserdata(L, const_cast<file_folders *>(Folders));
		lua_pushcclosure(L, function, 1);
		lua_setfield(L, -2, name);
	}
	lua_pop(L, 1);
}

} // namespace lua_sandbox
