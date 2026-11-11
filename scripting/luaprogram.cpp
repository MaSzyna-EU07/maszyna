/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the Lua screens: scripts run by LuaJIT, each screen in a state of its own, with the eu07 table of
// script_screens_doc.md. the state has no io, os, ffi, debug or require, reads files only through eu07 and below the
// script's folder, takes no precompiled code, and runs without the JIT compiler, so that the instruction count hook
// can stop a script stuck in a loop (LuaJIT calls no hooks in compiled code)

#include "stdafx.h"
#include "scripting/scriptscreenprogram.h"
#include "scripting/scriptscreendrawing.h"
#include "scripting/scriptscreenkernels.h"

#include "utilities/Logs.h"
#include "utilities/dictionary.h"
#include "utilities/utilities.h"

#include "stb/stb_image.h"

#include <lua.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <format>
#include <fstream>
#include <iterator>
#include <new>
#include <string_view>
#include <type_traits>
#include <utility>

namespace script_screens
{

namespace
{

constexpr int hook_interval{1000};
constexpr char const *image_metatable{"eu07.image"};
constexpr char const *font_metatable{"eu07.font"};
constexpr char const *sprite_metatable{"eu07.sprite"};
// the address is the key of the program in the registry of its state
constexpr char program_key{0};

class lua_program final : public program
{
  public:
	lua_program(std::string const &Script, script_screen_kind const Kind) : kind{Kind}, script{Script}, folder{substr_path(Script)} {}
	~lua_program() override
	{
		if (m_state != nullptr)
		{
			lua_close(m_state);
		}
	}
	lua_program(lua_program const &) = delete;
	lua_program &operator=(lua_program const &) = delete;

	bool start();
	bool render(render_call &Call) override;

	script_screen_kind const kind;
	std::string const script;
	std::string const folder;
	render_call *call{nullptr}; // the refresh in progress
	bool commands_ignored{false}; // a screen outside the cab tried to send a command, and it was logged
	std::int64_t budget{0}; // instructions left to the call in progress

  private:
	// calls what's on the stack, with the instruction limit. logs a failure
	bool run(int Arguments, int Results, std::string_view What);
	void open_libraries();
	void register_api();

	lua_State *m_state{nullptr};
	int m_screen{LUA_NOREF}; // the table the script returned
};

lua_program *program_of(lua_State *L)
{
	lua_pushlightuserdata(L, const_cast<char *>(&program_key)); // NOSONAR: the address is only a key
	lua_rawget(L, LUA_REGISTRYINDEX);
	auto *self{static_cast<lua_program *>(lua_touserdata(L, -1))};
	lua_pop(L, 1);
	return self;
}

void count_hook(lua_State *L, lua_Debug * /*Debug*/)
{
	auto *self{program_of(L)};
	if (self == nullptr)
	{
		return;
	}
	self->budget -= hook_interval;
	if (self->budget < 0)
	{
		luaL_error(L, "instruction limit exceeded");
	}
}

// objects of the api -------------------------------------------------------------------------------------------------

// puts the object in a new userdata with the metatable, left on the stack: an rvalue is moved in, an lvalue copied.
// what's stored is the plain type (no reference, no const), the one collect<> and check_object<> are given
template <typename Type_> std::remove_cvref_t<Type_> *push_object(lua_State *L, Type_ &&Object, char const *Metatable)
{
	using object_type = std::remove_cvref_t<Type_>;
	auto *memory{lua_newuserdata(L, sizeof(object_type))};
	auto *object{new (memory) object_type(std::forward<Type_>(Object))};
	luaL_getmetatable(L, Metatable);
	lua_setmetatable(L, -2);
	return object;
}

template <typename Type_> int collect(lua_State *L)
{
	static_cast<Type_ *>(lua_touserdata(L, 1))->~Type_();
	return 0;
}

// the object at the index when it has the metatable, otherwise nullptr
template <typename Type_> Type_ *test_object(lua_State *L, int const Index, char const *Metatable)
{
	auto *object{lua_touserdata(L, Index)};
	if (object == nullptr || 0 == lua_getmetatable(L, Index))
	{
		return nullptr;
	}
	luaL_getmetatable(L, Metatable);
	auto const matches{lua_rawequal(L, -1, -2) != 0};
	lua_pop(L, 2);
	return matches ? static_cast<Type_ *>(object) : nullptr;
}

template <typename Type_> Type_ &check_object(lua_State *L, int const Index, char const *Metatable)
{
	auto *object{test_object<Type_>(L, Index, Metatable)};
	if (object == nullptr)
	{
		luaL_typerror(L, Index, Metatable);
	}
	return *object;
}

// an image of a script: what was drawn on it, and the post-processing (image:led_post, image:resize) asked for and not
// run yet. the steps of the image render returns run in the queue, on the gpu when the renderer can; the steps of an
// image the script goes on to read or draw on (or with) run here, on the cpu, when it does
struct lua_image
{
	canvas drawn;
	std::vector<post_step> steps;

	int width() const
	{
		auto result{drawn.width()};
		for (auto const &step : steps)
		{
			result = width_after(step, result);
		}
		return result;
	}
	int height() const
	{
		auto result{drawn.height()};
		for (auto const &step : steps)
		{
			result = height_after(step, result);
		}
		return result;
	}
	// the image with its post-processing done
	canvas &done()
	{
		if (false == steps.empty())
		{
			post_image image{drawn.pixels(), drawn.width(), drawn.height()};
			// the sizes were checked when the steps were asked for
			apply_post(image, steps);
			drawn = canvas{image.width, image.height, std::move(image.pixels)};
			steps.clear();
		}
		return drawn;
	}
	// the image, a step further
	lua_image then(post_step const &Step) const
	{
		lua_image result{drawn, steps};
		result.steps.push_back(Step);
		return result;
	}
};

lua_image &check_lua_image(lua_State *L, int const Index)
{
	return check_object<lua_image>(L, Index, image_metatable);
}

// the image, to read or draw on: its post-processing is done first
canvas &check_image(lua_State *L, int const Index)
{
	return check_lua_image(L, Index).done();
}

int check_int(lua_State *L, int const Index)
{
	return static_cast<int>(luaL_checkinteger(L, Index));
}

int optional_int(lua_State *L, int const Index, int const Default)
{
	return static_cast<int>(luaL_optinteger(L, Index, Default));
}

// a colour: { r, g, b }, each 0-255
rgb check_color(lua_State *L, int const Index)
{
	luaL_checktype(L, Index, LUA_TTABLE);
	rgb color{};
	for (int channel = 0; channel < 3; ++channel)
	{
		lua_rawgeti(L, Index, channel + 1);
		if (0 == lua_isnumber(L, -1))
		{
			luaL_argerror(L, Index, "a colour is { r, g, b }");
		}
		color[channel] = static_cast<std::uint8_t>(std::clamp<lua_Integer>(lua_tointeger(L, -1), 0, 255));
		lua_pop(L, 1);
	}
	return color;
}

std::string_view check_text(lua_State *L, int const Index)
{
	std::size_t length{0};
	auto const *text{luaL_checklstring(L, Index, &length)};
	return {text, length};
}

struct decoded_image
{
	std::vector<std::uint8_t> pixels;
	int width{0};
	int height{0};
};

// a .png of the screen's folder or below, as RGBA
std::optional<decoded_image> read_png(std::string const &Folder, std::string_view const Path)
{
	auto const file{read_screen_file(Folder, Path)};
	if (false == file.has_value())
	{
		return std::nullopt;
	}
	int width{0};
	int height{0};
	int channels{0};
	// the texture loader turns every image bottom row first, for all threads (model/Texture.cpp): fonts and sprites are
	// read top row first, as led_util reads them
	stbi_set_flip_vertically_on_load_thread(0);
	auto *pixels{stbi_load_from_memory(reinterpret_cast<stbi_uc const *>(file->data()), static_cast<int>(file->size()), &width, &height, &channels, 4)};
	if (pixels == nullptr)
	{
		return std::nullopt;
	}
	decoded_image result{std::vector<std::uint8_t>(pixels, pixels + image_size(width, height)), width, height};
	stbi_image_free(pixels);
	return result;
}

// eu07 ---------------------------------------------------------------------------------------------------------------

// eu07.image(width, height [, color]): a new image, black or in the colour
int api_image(lua_State *L)
{
	auto const width{check_int(L, 1)};
	auto const height{check_int(L, 2)};
	luaL_argcheck(L, valid_image_side(width), 1, "between 1 and 4096");
	luaL_argcheck(L, valid_image_side(height), 2, "between 1 and 4096");
	auto const color{lua_isnoneornil(L, 3) ? rgb{0, 0, 0} : check_color(L, 3)};
	push_object(L, lua_image{canvas{width, height, color}, {}}, image_metatable);
	return 1;
}

// eu07.font(path): the font of path .. ".png" and path .. ".txt", below the script's folder
int api_font(lua_State *L)
{
	auto const path{check_text(L, 1)};
	auto const *self{program_of(L)};
	auto loaded{false};
	{
		auto const image{read_png(self->folder, std::string{path} + ".png")};
		auto const description{read_screen_file(self->folder, std::string{path} + ".txt")};
		if (image.has_value() && description.has_value())
		{
			if (auto font{bitmap_font::parse(image->pixels, image->width, image->height, *description)}; font.has_value())
			{
				push_object(L, std::move(*font), font_metatable);
				loaded = true;
			}
		}
	}
	if (false == loaded)
	{
		return luaL_error(L, "font \"%s\" not found below the script's folder", lua_tostring(L, 1));
	}
	return 1;
}

// eu07.sprite(path): the sprite of path .. ".png", below the script's folder
int api_sprite(lua_State *L)
{
	auto const path{check_text(L, 1)};
	auto const *self{program_of(L)};
	auto loaded{false};
	{
		if (auto const image{read_png(self->folder, std::string{path} + ".png")}; image.has_value())
		{
			push_object(L, bitmap_font::sprite(image->pixels, image->width, image->height), sprite_metatable);
			loaded = true;
		}
	}
	if (false == loaded)
	{
		return luaL_error(L, "sprite \"%s\" not found below the script's folder", lua_tostring(L, 1));
	}
	return 1;
}

int api_time(lua_State *L)
{
	lua_pushnumber(L, screen_time());
	return 1;
}

// eu07.log(...): the values, joined with spaces, as print() does
int api_log(lua_State *L)
{
	auto const count{lua_gettop(L)};
	for (int idx = 1; idx <= count; ++idx)
	{
		if (idx > 1)
		{
			lua_pushliteral(L, " ");
		}
		lua_getglobal(L, "tostring");
		lua_pushvalue(L, idx);
		lua_call(L, 1, 1);
	}
	if (count == 0)
	{
		lua_pushliteral(L, "");
	}
	else
	{
		lua_concat(L, count * 2 - 1);
	}
	auto const *self{program_of(L)};
	auto const *text{lua_tostring(L, -1)};
	WriteLog(std::format("Lua screen \"{}\": {}", self->script, text != nullptr ? text : ""));
	return 0;
}

// eu07.command(name, param1, param2): sends a command to the vehicle of the cab
int api_command(lua_State *L)
{
	auto const name{check_text(L, 1)};
	auto const param1{optional_int(L, 2, 0)};
	auto const param2{optional_int(L, 3, 0)};
	auto *self{program_of(L)};
	if (self->call == nullptr)
	{
		return 0;
	}
	if (self->kind != script_screen_kind::cab)
	{
		if (false == self->commands_ignored)
		{
			self->commands_ignored = true;
			WriteLog(std::format("Lua screen \"{}\": commands are only taken from screens in the cab", self->script));
		}
		return 0;
	}
	self->call->commands.emplace_back(std::string{name}, param1, param2, self->call->vehicle);
	return 0;
}

// eu07.read_file(path): the contents of a file below the script's folder, or nil
int api_read_file(lua_State *L)
{
	auto const path{check_text(L, 1)};
	auto const *self{program_of(L)};
	if (auto const contents{read_screen_file(self->folder, path)}; contents.has_value())
	{
		lua_pushlstring(L, contents->data(), contents->size());
	}
	else
	{
		lua_pushnil(L);
	}
	return 1;
}

// loads a script as text: precompiled code is refused. leaves the chunk, or an error message, on the stack
bool load_chunk(lua_State *L, std::string_view const Code, std::string const &Name)
{
	if (false == Code.empty() && Code.front() == '\x1b')
	{
		lua_pushfstring(L, "%s: precompiled scripts aren't loaded", Name.c_str());
		return false;
	}
	return 0 == luaL_loadbufferx(L, Code.data(), Code.size(), ("@" + Name).c_str(), "t");
}

// eu07.include(path): runs another script below the script's folder and returns what it returns
int api_include(lua_State *L)
{
	auto const path{check_text(L, 1)};
	auto const *self{program_of(L)};
	auto loaded{false};
	{
		if (auto const code{read_screen_file(self->folder, path)}; code.has_value())
		{
			loaded = load_chunk(L, *code, self->folder + std::string{path});
		}
		else
		{
			lua_pushfstring(L, "script \"%s\" not found below the script's folder", lua_tostring(L, 1));
		}
	}
	if (false == loaded)
	{
		return lua_error(L);
	}
	lua_call(L, 0, 1);
	return 1;
}

// eu07.image methods -------------------------------------------------------------------------------------------------

int image_width(lua_State *L)
{
	lua_pushinteger(L, check_lua_image(L, 1).width());
	return 1;
}

int image_height(lua_State *L)
{
	lua_pushinteger(L, check_lua_image(L, 1).height());
	return 1;
}

int image_fill(lua_State *L)
{
	auto &image{check_image(L, 1)};
	image.fill(check_color(L, 2));
	return 0;
}

int image_set(lua_State *L)
{
	auto &image{check_image(L, 1)};
	auto const x{check_int(L, 2)};
	auto const y{check_int(L, 3)};
	image.set(x, y, check_color(L, 4));
	return 0;
}

int image_get(lua_State *L)
{
	auto const &image{check_image(L, 1)};
	auto const color{image.get(check_int(L, 2), check_int(L, 3))};
	for (auto const channel : color)
	{
		lua_pushinteger(L, channel);
	}
	return 3;
}

int image_fill_rect(lua_State *L)
{
	auto &image{check_image(L, 1)};
	auto const x{check_int(L, 2)};
	auto const y{check_int(L, 3)};
	auto const width{check_int(L, 4)};
	auto const height{check_int(L, 5)};
	image.fill_rect(x, y, width, height, check_color(L, 6));
	return 0;
}

enum class alignment
{
	left,
	right,
	center
};

// image:draw_text(font, x, y, text, color [, spacing]): returns the x after the text
int draw_text_aligned(lua_State *L, alignment const Alignment)
{
	auto &image{check_image(L, 1)};
	auto const &font{check_object<bitmap_font>(L, 2, font_metatable)};
	auto x{check_int(L, 3)};
	auto const y{check_int(L, 4)};
	auto const text{check_text(L, 5)};
	auto const color{check_color(L, 6)};
	auto const spacing{optional_int(L, 7, 1)};
	if (Alignment == alignment::right)
	{
		x -= font.text_width(text, spacing);
	}
	else if (Alignment == alignment::center)
	{
		x -= font.text_width(text, spacing) / 2;
	}
	lua_pushinteger(L, image.draw_text(font, x, y, text, color, spacing));
	return 1;
}

int image_draw_text(lua_State *L)
{
	return draw_text_aligned(L, alignment::left);
}

int image_draw_text_right(lua_State *L)
{
	return draw_text_aligned(L, alignment::right);
}

int image_draw_text_centered(lua_State *L)
{
	return draw_text_aligned(L, alignment::center);
}

int image_draw_sprite(lua_State *L)
{
	auto &image{check_image(L, 1)};
	auto const &sprite{check_object<glyph>(L, 2, sprite_metatable)};
	auto const x{check_int(L, 3)};
	auto const y{check_int(L, 4)};
	image.draw_glyph(sprite, x, y, check_color(L, 5));
	return 0;
}

int image_draw_image(lua_State *L)
{
	auto &image{check_image(L, 1)};
	auto const &source{check_image(L, 2)};
	image.draw_image(source, check_int(L, 3), check_int(L, 4));
	return 0;
}

// image:led_post([bloom]): led_util's render_post, a new image (2w + 2) x (2h + 2). done when the image is shown or used
int image_led_post(lua_State *L)
{
	auto const &image{check_lua_image(L, 1)};
	auto const bloom{lua_isnoneornil(L, 2) || lua_toboolean(L, 2) != 0};
	luaL_argcheck(L, valid_image_side(image.width() * 2 + 2) && valid_image_side(image.height() * 2 + 2), 1, "too large");
	post_step step;
	step.type = post_step::kind::led;
	step.bloom = bloom;
	push_object(L, image.then(step), image_metatable);
	return 1;
}

// image:resize(width, height): scaled as Pillow's resize(..., LANCZOS), a new image. done when the image is shown or used
int image_resize(lua_State *L)
{
	auto const &image{check_lua_image(L, 1)};
	auto const width{check_int(L, 2)};
	auto const height{check_int(L, 3)};
	luaL_argcheck(L, valid_image_side(width), 2, "between 1 and 4096");
	luaL_argcheck(L, valid_image_side(height), 3, "between 1 and 4096");
	post_step step;
	step.type = post_step::kind::resize;
	step.width = width;
	step.height = height;
	push_object(L, image.then(step), image_metatable);
	return 1;
}

int image_copy(lua_State *L)
{
	auto const &image{check_lua_image(L, 1)};
	push_object(L, lua_image{image}, image_metatable);
	return 1;
}

// eu07.font and eu07.sprite methods ----------------------------------------------------------------------------------

int font_height(lua_State *L)
{
	lua_pushinteger(L, check_object<bitmap_font>(L, 1, font_metatable).height());
	return 1;
}

int font_text_width(lua_State *L)
{
	auto const &font{check_object<bitmap_font>(L, 1, font_metatable)};
	auto const text{check_text(L, 2)};
	lua_pushinteger(L, font.text_width(text, optional_int(L, 3, 1)));
	return 1;
}

int sprite_width(lua_State *L)
{
	lua_pushinteger(L, check_object<glyph>(L, 1, sprite_metatable).width());
	return 1;
}

int sprite_height(lua_State *L)
{
	lua_pushinteger(L, check_object<glyph>(L, 1, sprite_metatable).height());
	return 1;
}

constexpr std::array image_methods{
    luaL_Reg{"width", image_width},
    luaL_Reg{"height", image_height},
    luaL_Reg{"fill", image_fill},
    luaL_Reg{"set", image_set},
    luaL_Reg{"get", image_get},
    luaL_Reg{"fill_rect", image_fill_rect},
    luaL_Reg{"draw_text", image_draw_text},
    luaL_Reg{"draw_text_right", image_draw_text_right},
    luaL_Reg{"draw_text_centered", image_draw_text_centered},
    luaL_Reg{"draw_sprite", image_draw_sprite},
    luaL_Reg{"draw_image", image_draw_image},
    luaL_Reg{"led_post", image_led_post},
    luaL_Reg{"resize", image_resize},
    luaL_Reg{"copy", image_copy},
    luaL_Reg{nullptr, nullptr},
};

constexpr std::array font_methods{
    luaL_Reg{"height", font_height},
    luaL_Reg{"text_width", font_text_width},
    luaL_Reg{nullptr, nullptr},
};

constexpr std::array sprite_methods{
    luaL_Reg{"width", sprite_width},
    luaL_Reg{"height", sprite_height},
    luaL_Reg{nullptr, nullptr},
};

constexpr std::array api_functions{
    luaL_Reg{"image", api_image},
    luaL_Reg{"font", api_font},
    luaL_Reg{"sprite", api_sprite},
    luaL_Reg{"time", api_time},
    luaL_Reg{"log", api_log},
    luaL_Reg{"command", api_command},
    luaL_Reg{"read_file", api_read_file},
    luaL_Reg{"include", api_include},
    luaL_Reg{nullptr, nullptr},
};

template <std::size_t Size_> void make_metatable(lua_State *L, char const *Name, std::array<luaL_Reg, Size_> const &Methods, lua_CFunction Collect)
{
	luaL_newmetatable(L, Name);
	lua_newtable(L);
	for (auto const &method : Methods)
	{
		if (method.name == nullptr)
		{
			break;
		}
		lua_pushcfunction(L, method.func);
		lua_setfield(L, -2, method.name);
	}
	lua_setfield(L, -2, "__index");
	lua_pushcfunction(L, Collect);
	lua_setfield(L, -2, "__gc");
	lua_pop(L, 1);
}

// the state of a refresh as a table, as the python screens get it in a dictionary; a list of points (touches) is a
// list of { x, y }
void push_state(lua_State *L, dictionary_source const &Input)
{
	lua_createtable(L, 0, static_cast<int>(Input.floats.size() + Input.integers.size() + Input.bools.size() + Input.strings.size() + Input.vec2_lists.size()));
	for (auto const &[key, value] : Input.floats)
	{
		lua_pushnumber(L, value);
		lua_setfield(L, -2, key.c_str());
	}
	for (auto const &[key, value] : Input.integers)
	{
		lua_pushnumber(L, value);
		lua_setfield(L, -2, key.c_str());
	}
	for (auto const &[key, value] : Input.bools)
	{
		lua_pushboolean(L, value ? 1 : 0);
		lua_setfield(L, -2, key.c_str());
	}
	for (auto const &[key, value] : Input.strings)
	{
		auto const text{utf8_text(value)};
		lua_pushlstring(L, text.data(), text.size());
		lua_setfield(L, -2, key.c_str());
	}
	for (auto const &[key, points] : Input.vec2_lists)
	{
		lua_createtable(L, static_cast<int>(points.size()), 0);
		for (std::size_t idx = 0; idx < points.size(); ++idx)
		{
			lua_createtable(L, 2, 0);
			lua_pushnumber(L, points[idx].x);
			lua_rawseti(L, -2, 1);
			lua_pushnumber(L, points[idx].y);
			lua_rawseti(L, -2, 2);
			lua_rawseti(L, -2, static_cast<int>(idx + 1));
		}
		lua_setfield(L, -2, key.c_str());
	}
}

void open_library(lua_State *L, lua_CFunction Open, char const *Name)
{
	lua_pushcfunction(L, Open);
	lua_pushstring(L, Name);
	lua_call(L, 1, 0);
}

void lua_program::open_libraries()
{
	open_library(m_state, luaopen_base, "");
	open_library(m_state, luaopen_table, LUA_TABLIBNAME);
	open_library(m_state, luaopen_string, LUA_STRLIBNAME);
	open_library(m_state, luaopen_math, LUA_MATHLIBNAME);
	open_library(m_state, luaopen_bit, LUA_BITLIBNAME);
	// what reaches files or loads code
	for (auto const *name : {"dofile", "loadfile", "load", "loadstring", "module", "require"})
	{
		lua_pushnil(m_state);
		lua_setglobal(m_state, name);
	}
	lua_getglobal(m_state, LUA_STRLIBNAME);
	lua_pushnil(m_state);
	lua_setfield(m_state, -2, "dump");
	lua_pop(m_state, 1);
}

void lua_program::register_api()
{
	auto *L{m_state};
	lua_pushlightuserdata(L, const_cast<char *>(&program_key)); // NOSONAR: the address is only a key
	lua_pushlightuserdata(L, this);
	lua_rawset(L, LUA_REGISTRYINDEX);

	make_metatable(L, image_metatable, image_methods, collect<lua_image>);
	make_metatable(L, font_metatable, font_methods, collect<bitmap_font>);
	make_metatable(L, sprite_metatable, sprite_methods, collect<glyph>);

	lua_newtable(L);
	for (auto const &function : api_functions)
	{
		if (function.name == nullptr)
		{
			break;
		}
		lua_pushcfunction(L, function.func);
		lua_setfield(L, -2, function.name);
	}
	lua_setglobal(L, "eu07");
	// print() writes to the log
	lua_pushcfunction(L, api_log);
	lua_setglobal(L, "print");
}

bool lua_program::run(int const Arguments, int const Results, std::string_view const What)
{
	budget = instruction_limit;
	if (0 == lua_pcall(m_state, Arguments, Results, 0))
	{
		return true;
	}
	auto const *message{lua_tostring(m_state, -1)};
	ErrorLog(std::format("Lua screen \"{}\": {} failed: {}", script, What, message != nullptr ? message : "unknown error"));
	lua_pop(m_state, 1);
	return false;
}

bool lua_program::start()
{
	m_state = luaL_newstate();
	if (m_state == nullptr)
	{
		ErrorLog(std::format("Lua screen \"{}\": failed to create the state", script));
		return false;
	}
	// no compiled code: LuaJIT calls no hooks in it, and the hook is what stops a script stuck in a loop
	luaJIT_setmode(m_state, 0, LUAJIT_MODE_ENGINE | LUAJIT_MODE_OFF);
	open_libraries();
	register_api();
	lua_sethook(m_state, count_hook, LUA_MASKCOUNT, hook_interval);

	std::ifstream file{script, std::ios::binary};
	if (false == file.is_open())
	{
		ErrorLog(std::format("Lua screen \"{}\": file not found", script));
		return false;
	}
	std::string const code{std::istreambuf_iterator<char>{file}, std::istreambuf_iterator<char>{}};
	if (false == load_chunk(m_state, code, script))
	{
		ErrorLog(std::format("Lua screen \"{}\": {}", script, lua_tostring(m_state, -1)));
		return false;
	}
	if (false == run(0, 1, "the script"))
	{
		return false;
	}
	if (false == lua_istable(m_state, -1))
	{
		ErrorLog(std::format("Lua screen \"{}\": the script has to return a table with a render function", script));
		return false;
	}
	lua_getfield(m_state, -1, "render");
	auto const renders{lua_isfunction(m_state, -1)};
	lua_pop(m_state, 1);
	if (false == renders)
	{
		ErrorLog(std::format("Lua screen \"{}\": the script has to return a table with a render function", script));
		return false;
	}
	m_screen = luaL_ref(m_state, LUA_REGISTRYINDEX);

	lua_rawgeti(m_state, LUA_REGISTRYINDEX, m_screen);
	lua_getfield(m_state, -1, "init");
	lua_remove(m_state, -2);
	if (lua_isfunction(m_state, -1))
	{
		if (false == run(0, 0, "init"))
		{
			return false;
		}
	}
	else
	{
		lua_pop(m_state, 1);
	}
	WriteLog(std::format("Lua screen \"{}\": ready", script));
	return true;
}

bool lua_program::render(render_call &Call)
{
	call = &Call;
	lua_rawgeti(m_state, LUA_REGISTRYINDEX, m_screen);
	lua_getfield(m_state, -1, "render");
	lua_remove(m_state, -2);
	push_state(m_state, Call.input);
	auto const rendered{run(1, 1, "render")};
	call = nullptr;
	if (false == rendered)
	{
		return false;
	}
	// nil: the screen didn't change, keep the image there is
	auto succeeded{true};
	if (auto const *image{test_object<lua_image>(m_state, -1, image_metatable)}; image != nullptr)
	{
		Call.pixels = image->drawn.pixels();
		Call.width = image->drawn.width();
		Call.height = image->drawn.height();
		Call.post = image->steps;
	}
	else if (false == lua_isnil(m_state, -1))
	{
		ErrorLog(std::format("Lua screen \"{}\": render has to return an image or nil", script));
		succeeded = false;
	}
	lua_pop(m_state, 1);
	// the images the refreshes left behind
	lua_gc(m_state, LUA_GCSTEP, 0);
	return succeeded;
}

class lua_runtime_host final : public runtime
{
  public:
	std::unique_ptr<program> load(std::string const &Script, script_screen_kind const Kind) override
	{
		auto loaded{std::make_unique<lua_program>(Script, Kind)};
		if (false == loaded->start())
		{
			return nullptr;
		}
		return loaded;
	}
};

} // namespace

std::unique_ptr<runtime> make_lua_runtime()
{
	return std::make_unique<lua_runtime_host>();
}

} // namespace script_screens
