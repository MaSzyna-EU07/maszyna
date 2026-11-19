/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the Lua screens: scripts run by LuaJIT, each screen in a state of its own, with the eu07 table of
// script_screens_doc.md. the state has no io, os, ffi, debug, require or newproxy, reads files only through eu07 and
// below the script's folder, takes no precompiled code, and runs without the JIT compiler, so that the instruction
// count hook can stop a script stuck in a loop (LuaJIT calls no hooks in compiled code). what runs in C without the
// hook is held to limits of its own: the pattern functions to the steps they may take, what makes large strings and
// images to the memory of the screen. the game reads the script's table in protected mode, as its metamethods may
// fail

#include "stdafx.h"
#include "scripting/scriptscreenprogram.h"
#include "scripting/scriptscreendrawing.h"
#include "scripting/scriptscreenkernels.h"
#include "scripting/scriptscreenrules.h"
#include "scripting/luasandbox.h"

#include "utilities/Logs.h"
#include "utilities/dictionary.h"
#include "utilities/utilities.h"

#include "stb/stb_image.h"

#include <lua.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <format>
#include <new>
#include <optional>
#include <string_view>
#include <type_traits>
#include <utility>

namespace script_screens
{

namespace
{

constexpr int hook_interval{1000};
// the longest string string.rep makes
constexpr double max_repeat{1024 * 1024};
// arrays, to be arguments of collect<>
constexpr char image_metatable[]{"eu07.image"};
constexpr char font_metatable[]{"eu07.font"};
constexpr char sprite_metatable[]{"eu07.sprite"};
// the address is the key of the program in the registry of its state
constexpr char program_key{0};

// the memory of a screen: what its Lua heap holds, when the allocator counts it, and the pixels of its images, fonts
// and sprites, which the objects keep outside the heap
struct screen_memory
{
	std::size_t heap{0};
	std::size_t objects{0};
	bool counted{false}; // the state was made with limited_alloc
	bool enforced{false}; // a call into the script is in progress: the allocator refuses what goes over the limit
	// what the screen holds when it is checked next (lua_program::over_memory) with a full collection
	std::size_t collect_at{max_screen_memory};
};

// the allocator of a screen's state (lua_newstate), which refuses a block that would take the screen over twice
// max_screen_memory (what it holds, and its garbage between the collections) while a call into the script is in
// progress: LuaJIT raises "not enough memory" there, in the protected call. the game's own work between the calls isn't
// refused, as it isn't protected
void *limited_alloc(void *Data, void *Block, std::size_t Old, std::size_t New)
{
	auto &memory{*static_cast<screen_memory *>(Data)};
	if (Block == nullptr)
	{
		Old = 0;
	}
	if (New == 0)
	{
		std::free(Block);
		memory.heap -= Old;
		return nullptr;
	}
	if (New > Old && memory.enforced && memory.heap - Old + New + memory.objects > 2 * max_screen_memory)
	{
		return nullptr;
	}
	auto *result{std::realloc(Block, New)};
	if (result != nullptr)
	{
		memory.heap = memory.heap - Old + New;
	}
	return result;
}

class lua_program final : public program
{
  public:
	lua_program(std::string const &Script, script_screen_kind const Kind)
	    : kind{Kind}, script{Script}, folder{substr_path(Script)}, output{std::format("Lua screen \"{}\"", Script)}
	{
	}
	~lua_program() override
	{
		if (m_state != nullptr)
		{
			// the finalizers lua_close runs aren't counted: the budget of the last call may be spent
			lua_sethook(m_state, nullptr, 0, 0);
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
	output_limits output;
	std::int64_t budget{0}; // instructions left to the call in progress
	char const *stopped{nullptr}; // why the call in progress is being stopped: the hook raises it at every instruction
	screen_memory memory;

	// true when the screen would hold more than max_screen_memory with Adding bytes more. checked with a full collection
	// once it has made a quarter of the limit since the last one, so a screen near its limit isn't collected over and
	// over
	bool over_memory(lua_State *L, std::size_t Adding);

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

bool lua_program::over_memory(lua_State *L, std::size_t const Adding)
{
	auto const used{[this, L] {
		auto const heap{memory.counted ? memory.heap
		                               : static_cast<std::size_t>(lua_gc(L, LUA_GCCOUNT, 0)) * 1024 + static_cast<std::size_t>(lua_gc(L, LUA_GCCOUNTB, 0))};
		return heap + memory.objects;
	}};
	if (used() + Adding <= memory.collect_at)
	{
		return false;
	}
	// what is garbage doesn't count
	lua_gc(L, LUA_GCCOLLECT, 0);
	auto const held{used()};
	memory.collect_at = std::max(max_screen_memory, held + max_screen_memory / 4);
	return held + Adding > max_screen_memory;
}

void count_hook(lua_State *L, lua_Debug * /*Debug*/)
{
	auto *self{program_of(L)};
	if (self == nullptr)
	{
		return;
	}
	if (self->stopped == nullptr)
	{
		self->budget -= hook_interval;
		if (self->budget < 0)
		{
			self->stopped = "instruction limit exceeded";
		}
		else if (self->over_memory(L, 0))
		{
			// what the allocator doesn't refuse (where the screen has none of its own) is caught here
			self->stopped = "memory limit exceeded";
		}
	}
	if (self->stopped != nullptr)
	{
		// from now on the hook runs at every instruction, and raises the error again. a script that catches it (pcall,
		// xpcall, coroutine.resume) gets it again at its next instruction, so the error comes out of every level of the
		// script in turn, and the call ends. run() sets the interval back for the next call. the hook is the state's,
		// shared by its coroutines
		lua_sethook(L, count_hook, LUA_MASKCOUNT, 1);
		luaL_error(L, "%s", self->stopped);
	}
}

// raises a Lua error when the screen can't take Bytes more: called before an object of the api is made
void reserve_memory(lua_State *L, std::size_t const Bytes)
{
	if (auto *self{program_of(L)}; self != nullptr && self->over_memory(L, Bytes))
	{
		luaL_error(L, "memory limit exceeded (%d MB)", static_cast<int>(max_screen_memory / (1024 * 1024)));
	}
}

// objects of the api -------------------------------------------------------------------------------------------------

// where the userdata of an object keeps the bytes it counts in the screen's memory, after the object
template <typename Type_> constexpr std::size_t charge_offset{(sizeof(Type_) + alignof(std::size_t) - 1) / alignof(std::size_t) * alignof(std::size_t)};

// puts the object in a new userdata with the metatable, left on the stack: an rvalue is moved in, an lvalue copied.
// what's stored is the plain type (no reference, no const), the one collect<> and check_object<> are given. Bytes: what
// the object holds outside the heap (its pixels), counted in the screen's memory until it is collected, and checked
// with reserve_memory() before the object was made
template <typename Type_> std::remove_cvref_t<Type_> *push_object(lua_State *L, Type_ &&Object, char const *Metatable, std::size_t const Bytes)
{
	using object_type = std::remove_cvref_t<Type_>;
	auto *memory{static_cast<char *>(lua_newuserdata(L, charge_offset<object_type> + sizeof(std::size_t)))};
	auto *object{new (memory) object_type(std::forward<Type_>(Object))};
	std::memcpy(memory + charge_offset<object_type>, &Bytes, sizeof(Bytes));
	if (auto *self{program_of(L)}; self != nullptr)
	{
		self->memory.objects += Bytes;
	}
	luaL_getmetatable(L, Metatable);
	lua_setmetatable(L, -2);
	return object;
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

// __gc. the metatables are locked (make_metatable), so only the collector calls it; all the same it destroys only an
// object of its own type, and once: the object loses its metatable, which its methods check for
template <typename Type_, char const *Metatable_> int collect(lua_State *L)
{
	if (auto *object{test_object<Type_>(L, 1, Metatable_)}; object != nullptr)
	{
		std::size_t bytes{0};
		std::memcpy(&bytes, reinterpret_cast<char const *>(object) + charge_offset<Type_>, sizeof(bytes));
		object->~Type_();
		lua_pushnil(L);
		lua_setmetatable(L, 1);
		if (auto *self{program_of(L)}; self != nullptr)
		{
			self->memory.objects -= std::min(bytes, self->memory.objects);
		}
	}
	return 0;
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

// the pixels an image holds: as drawn, and as its post-processing leaves it
std::size_t image_bytes(int const Width, int const Height, int const Finalwidth, int const Finalheight)
{
	auto const drawn{static_cast<std::size_t>(Width) * static_cast<std::size_t>(Height)};
	auto const done{Width == Finalwidth && Height == Finalheight ? 0 : static_cast<std::size_t>(Finalwidth) * static_cast<std::size_t>(Finalheight)};
	return (drawn + done) * 4;
}

std::size_t image_bytes(lua_image const &Image)
{
	return image_bytes(Image.drawn.width(), Image.drawn.height(), Image.width(), Image.height());
}

// a new image, Image a step further, counted in the screen's memory
void push_image_then(lua_State *L, lua_image const &Image, post_step const &Step)
{
	reserve_memory(L, image_bytes(Image.drawn.width(), Image.drawn.height(), width_after(Step, Image.width()), height_after(Step, Image.height())));
	auto next{Image.then(Step)};
	auto const bytes{image_bytes(next)};
	push_object(L, std::move(next), image_metatable, bytes);
}

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
	// the size first: a small file may declare an image of gigabytes
	if (0 == stbi_info_from_memory(reinterpret_cast<stbi_uc const *>(file->data()), static_cast<int>(file->size()), &width, &height, &channels) ||
	    false == valid_image_side(width) || false == valid_image_side(height))
	{
		return std::nullopt;
	}
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
	auto const bytes{image_bytes(width, height, width, height)};
	reserve_memory(L, bytes);
	push_object(L, lua_image{canvas{width, height, color}, {}}, image_metatable, bytes);
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
			// the glyphs keep a bit of each pixel; a byte is counted
			auto const bytes{static_cast<std::size_t>(image->width) * static_cast<std::size_t>(image->height)};
			reserve_memory(L, bytes);
			if (auto font{bitmap_font::parse(image->pixels, image->width, image->height, *description)}; font.has_value())
			{
				push_object(L, std::move(*font), font_metatable, bytes);
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
			auto const bytes{static_cast<std::size_t>(image->width) * static_cast<std::size_t>(image->height)};
			reserve_memory(L, bytes);
			push_object(L, bitmap_font::sprite(image->pixels, image->width, image->height), sprite_metatable, bytes);
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
	auto *self{program_of(L)};
	std::size_t length{0};
	auto const *text{lua_tolstring(L, -1, &length)};
	if (auto const line{self->output.take_log(text != nullptr ? std::string_view{text, length} : std::string_view{})}; line.has_value())
	{
		WriteLog(*line);
	}
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
	if (self->output.take_command(*self->call, name))
	{
		self->call->commands.emplace_back(std::string{name}, param1, param2, self->call->vehicle);
	}
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
	return lua_sandbox::load_text_chunk(L, Code, Name);
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
	push_image_then(L, image, step);
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
	push_image_then(L, image, step);
	return 1;
}

int image_copy(lua_State *L)
{
	auto const &image{check_lua_image(L, 1)};
	auto const bytes{image_bytes(image)};
	reserve_memory(L, bytes);
	push_object(L, lua_image{image}, image_metatable, bytes);
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

// string.rep, which makes no string longer than max_repeat. a separator (a third argument, where LuaJIT takes one)
// is counted as well
int api_repeat(lua_State *L)
{
	std::size_t length{0};
	luaL_checklstring(L, 1, &length);
	auto const count{luaL_checknumber(L, 2)};
	std::size_t separator{0};
	luaL_optlstring(L, 3, "", &separator);
	// as a double: no overflow for any count, and a count of NaN fails both comparisons
	if (false == (count <= 0) && false == (static_cast<double>(length) * count + static_cast<double>(separator) * (count - 1) <= max_repeat))
	{
		return luaL_error(L, "string.rep: the result would be over %d bytes", static_cast<int>(max_repeat));
	}
	lua_pushvalue(L, lua_upvalueindex(1));
	lua_insert(L, 1);
	lua_call(L, lua_gettop(L) - 1, 1);
	return 1;
}

// the functions of the libraries that make a string in one call, around the library's own (the upvalue): refused when
// the result may be over the memory of a screen. what they make otherwise is counted with the rest of the heap
int too_large(lua_State *L, char const *Function)
{
	return luaL_error(L, "%s: the result may be over the memory of a screen (%d MB)", Function, static_cast<int>(max_screen_memory / (1024 * 1024)));
}

// calls the library's own function, the upvalue, with the arguments, and returns what it returns
int call_library(lua_State *L)
{
	lua_pushvalue(L, lua_upvalueindex(1));
	lua_insert(L, 1);
	lua_call(L, lua_gettop(L) - 1, LUA_MULTRET);
	return lua_gettop(L);
}

// string.format: no longer than the format and its arguments, each string as %q writes it (a character as up to 4,
// "\0" as "\000"), anything else in at most 512 characters
int api_format(lua_State *L)
{
	std::size_t length{0};
	luaL_checklstring(L, 1, &length);
	auto total{static_cast<double>(length)};
	for (int idx = 2; idx <= lua_gettop(L); ++idx)
	{
		total += lua_type(L, idx) == LUA_TSTRING ? 4.0 * static_cast<double>(lua_objlen(L, idx)) : 512.0;
	}
	if (total > static_cast<double>(max_screen_memory))
	{
		return too_large(L, "string.format");
	}
	return call_library(L);
}

// table.concat: the items and the separators between them. the items are read as the library reads them, up to the
// first one that isn't a string or a number, on which the library raises its error
int api_concat(lua_State *L)
{
	luaL_checktype(L, 1, LUA_TTABLE);
	std::size_t separator{0};
	luaL_optlstring(L, 2, "", &separator);
	auto const first{luaL_optinteger(L, 3, 1)};
	auto const last{lua_isnoneornil(L, 4) ? static_cast<lua_Integer>(lua_objlen(L, 1)) : luaL_checkinteger(L, 4)};
	auto total{0.0};
	for (auto idx = first; idx <= last; ++idx)
	{
		lua_rawgeti(L, 1, static_cast<int>(idx));
		auto const type{lua_type(L, -1)};
		// a number in at most 32 characters
		total += (type == LUA_TSTRING ? static_cast<double>(lua_objlen(L, -1)) : 32.0) + (idx > first ? static_cast<double>(separator) : 0.0);
		lua_pop(L, 1);
		if (type != LUA_TSTRING && type != LUA_TNUMBER)
		{
			break;
		}
		if (total > static_cast<double>(max_screen_memory))
		{
			return too_large(L, "table.concat");
		}
	}
	return call_library(L);
}

// the pattern functions, upvalue 2 telling which
enum class pattern_function
{
	find,
	match,
	gmatch,
	gsub
};

constexpr std::array<char const *, 4> pattern_function_names{"string.find", "string.match", "string.gmatch", "string.gsub"};

// the longest text a replacement of gsub puts in place of the matches: a string (or a number) as it is, with each %0-%9
// as long as a match, a table by its longest value. nothing when it can't be told (a function, a table with a
// metatable): what those give is made in Lua, under the hook
std::optional<double> replacement_length(lua_State *L, int const Index, double const Subject, double const Matches)
{
	if (lua_type(L, Index) == LUA_TSTRING || lua_type(L, Index) == LUA_TNUMBER)
	{
		std::size_t length{0};
		auto const *text{lua_tolstring(L, Index, &length)};
		auto const captures{static_cast<double>(std::count(text, text + length, '%'))};
		// the matches don't overlap: a capture of each takes at most the subject; a position capture is a number
		return Matches * static_cast<double>(length) + captures * (Subject + 24.0 * Matches);
	}
	if (lua_type(L, Index) == LUA_TTABLE)
	{
		if (lua_getmetatable(L, Index) != 0)
		{
			// pushed: off the stack again, or the library gets it as the 4th argument
			lua_pop(L, 1);
			return std::nullopt;
		}
		auto longest{0.0};
		lua_pushnil(L);
		while (lua_next(L, Index) != 0)
		{
			longest = std::max(longest, lua_type(L, -1) == LUA_TSTRING ? static_cast<double>(lua_objlen(L, -1)) : 32.0);
			lua_pop(L, 1);
		}
		return Matches * longest;
	}
	return std::nullopt;
}

// string.find, match, gmatch and gsub, refused for a pattern that may take too long on the subject: they run in C,
// without the instruction hook (see lua_pattern_cost() and max_pattern_cost). gsub with a string or a table is refused
// as well when its result may be over the memory of a screen
int api_pattern(lua_State *L)
{
	auto const function{static_cast<pattern_function>(lua_tointeger(L, lua_upvalueindex(2)))};
	auto const *name{pattern_function_names[static_cast<std::size_t>(function)]};
	std::size_t subject{0};
	auto const *text{luaL_checklstring(L, 1, &subject)};
	std::size_t length{0};
	auto const *pattern{luaL_checklstring(L, 2, &length)};
	auto const plain{function == pattern_function::find && lua_toboolean(L, 4) != 0};
	if (lua_pattern_cost(std::string_view{pattern, length}, std::string_view{text, subject}, plain, function != pattern_function::gmatch) > max_pattern_cost)
	{
		return luaL_error(L, "%s: the pattern may take too long on a subject of %d bytes", name, static_cast<int>(subject));
	}
	if (function == pattern_function::gsub)
	{
		auto const size{static_cast<double>(subject)};
		auto const matches{lua_isnumber(L, 4) ? std::clamp(static_cast<double>(lua_tonumber(L, 4)), 0.0, size + 1.0) : size + 1.0};
		if (auto const replaced{replacement_length(L, 3, size, matches)}; replaced.has_value() && size + *replaced > static_cast<double>(max_screen_memory))
		{
			return too_large(L, name);
		}
	}
	return call_library(L);
}

// xpcall(f, handler, ...), in Lua on top of pcall: LuaJIT calls the handler of its own xpcall where the error was
// raised, before the stack unwinds, which for the error of the instruction hook is while the hook still runs, when no
// hook is called, so a handler that loops would never be stopped. here the handler is called once the error has left
// f, under pcall as well, and a script has no debug library to look at the stack with. made in Lua, a coroutine yields
// across it as across pcall
constexpr std::string_view xpcall_source{R"lua(
local pcall, select, error = pcall, select, error
local function finish(handler, ok, ...)
	if ok then
		return true, ...
	end
	local _, handled = pcall(handler, (...))
	return false, handled
end
return function(...)
	if select("#", ...) < 2 then
		error("bad argument #2 to 'xpcall' (value expected)", 2)
	end
	local f, handler = ...
	return finish(handler, pcall(f, select(3, ...)))
end
)lua"};

// what the screens get of the libraries in place of the library's own functions, which run in C without the hook:
// string.rep, string.format and table.concat with a cap on the length of the result, the pattern functions with a cap
// on the steps they take, and xpcall made of pcall. strings share the string table as their methods
void limit_libraries(lua_State *L)
{
	lua_getglobal(L, LUA_STRLIBNAME);
	lua_getfield(L, -1, "rep");
	lua_pushcclosure(L, api_repeat, 1);
	lua_setfield(L, -2, "rep");
	lua_getfield(L, -1, "format");
	lua_pushcclosure(L, api_format, 1);
	lua_setfield(L, -2, "format");
	for (std::size_t idx = 0; idx < pattern_function_names.size(); ++idx)
	{
		// "string.find": the name in the table is past the dot
		auto const *name{pattern_function_names[idx] + sizeof("string.") - 1};
		lua_getfield(L, -1, name);
		lua_pushinteger(L, static_cast<lua_Integer>(idx));
		lua_pushcclosure(L, api_pattern, 2);
		lua_setfield(L, -2, name);
	}
	lua_pop(L, 1);

	lua_getglobal(L, LUA_TABLIBNAME);
	lua_getfield(L, -1, "concat");
	lua_pushcclosure(L, api_concat, 1);
	lua_setfield(L, -2, "concat");
	lua_pop(L, 1);

	if (0 == luaL_loadbuffer(L, xpcall_source.data(), xpcall_source.size(), "=xpcall") && 0 == lua_pcall(L, 0, 1, 0))
	{
		lua_setglobal(L, "xpcall");
	}
	else
	{
		// a script then gets no xpcall rather than the one the hook doesn't reach
		lua_pop(L, 1);
		lua_pushnil(L);
		lua_setglobal(L, "xpcall");
	}
}

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
	// getmetatable() of an object gives this instead of the table
	lua_pushboolean(L, 0);
	lua_setfield(L, -2, "__metatable");
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

void lua_program::open_libraries()
{
	// base, table, string, math and bit, without what reaches files or loads code
	lua_sandbox::open_safe_libraries(m_state);
	// what runs in C without the hook is held to limits of its own
	limit_libraries(m_state);
}

void lua_program::register_api()
{
	auto *L{m_state};
	lua_pushlightuserdata(L, const_cast<char *>(&program_key)); // NOSONAR: the address is only a key
	lua_pushlightuserdata(L, this);
	lua_rawset(L, LUA_REGISTRYINDEX);

	make_metatable(L, image_metatable, image_methods, collect<lua_image, image_metatable>);
	make_metatable(L, font_metatable, font_methods, collect<bitmap_font, font_metatable>);
	make_metatable(L, sprite_metatable, sprite_methods, collect<glyph, sprite_metatable>);

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
	stopped = nullptr;
	// the interval, which the hook shortens once the budget of a call is spent
	lua_sethook(m_state, count_hook, LUA_MASKCOUNT, hook_interval);
	output.start_call();
	memory.enforced = true;
	auto const status{lua_pcall(m_state, Arguments, Results, 0)};
	memory.enforced = false;
	if (0 == status)
	{
		return true;
	}
	auto const *message{lua_tostring(m_state, -1)};
	ErrorLog(std::format("Lua screen \"{}\": {} failed: {}", script, What, message != nullptr ? message : "unknown error"));
	lua_pop(m_state, 1);
	// what the call left behind, a heap run over the limit
	lua_gc(m_state, LUA_GCCOLLECT, 0);
	return false;
}

// render and init of the table the script returned (the argument), read in protected mode: the table may have a
// metatable of the script's, whose __index runs the script and may fail
int screen_functions(lua_State *L)
{
	lua_getfield(L, 1, "render");
	lua_getfield(L, 1, "init");
	return 2;
}

// a refresh: render of the screen's table (its reference, the second argument) called with the state of the refresh
// (the dictionary_source, the first). in protected mode, the reading of render and the making of the state as well
int call_render(lua_State *L)
{
	auto const &input{*static_cast<dictionary_source const *>(lua_touserdata(L, 1))};
	auto const screen{static_cast<int>(lua_tointeger(L, 2))};
	lua_settop(L, 0);
	lua_rawgeti(L, LUA_REGISTRYINDEX, screen);
	lua_getfield(L, 1, "render");
	push_state(L, input);
	lua_call(L, 1, 1);
	return 1;
}

// true when a screen's state can be made with an allocator of its own: LuaJIT 2.0 on x64, and 2.1 on x64 without GC64,
// refuse one (lua_newstate gives nullptr, and writes so to stderr: 2.0 isn't asked)
bool custom_allocator_works()
{
#if LUAJIT_VERSION_NUM < 20100
	if constexpr (sizeof(void *) == 8)
	{
		return false;
	}
#endif
	static bool const works{[] {
		screen_memory probe;
		auto *state{lua_newstate(limited_alloc, &probe)};
		if (state == nullptr)
		{
			return false;
		}
		lua_close(state);
		return true;
	}()};
	return works;
}

bool lua_program::start()
{
	// the screen's own allocator counts its memory, where LuaJIT takes one
	if (custom_allocator_works())
	{
		m_state = lua_newstate(limited_alloc, &memory);
		memory.counted = m_state != nullptr;
	}
	if (m_state == nullptr)
	{
		m_state = luaL_newstate();
	}
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

	auto const code{read_regular_file(script)};
	if (false == code.has_value())
	{
		ErrorLog(std::format("Lua screen \"{}\": file not found, or larger than {} bytes", script, max_screen_file));
		return false;
	}
	if (false == load_chunk(m_state, *code, script))
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
	lua_pushcfunction(m_state, screen_functions);
	lua_pushvalue(m_state, -2);
	if (false == run(1, 2, "reading render and init"))
	{
		return false;
	}
	// the table, render, init
	if (false == lua_isfunction(m_state, -2))
	{
		ErrorLog(std::format("Lua screen \"{}\": the script has to return a table with a render function", script));
		return false;
	}
	lua_remove(m_state, -2);
	lua_insert(m_state, -2);
	// init, the table
	m_screen = luaL_ref(m_state, LUA_REGISTRYINDEX);
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
	lua_pushcfunction(m_state, call_render);
	lua_pushlightuserdata(m_state, const_cast<dictionary_source *>(&Call.input));
	lua_pushinteger(m_state, m_screen);
	auto const rendered{run(2, 1, "render")};
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

bool lua_screen_memory_allocator()
{
	return custom_allocator_works();
}

std::unique_ptr<runtime> make_lua_runtime()
{
	return std::make_unique<lua_runtime_host>();
}

} // namespace script_screens
