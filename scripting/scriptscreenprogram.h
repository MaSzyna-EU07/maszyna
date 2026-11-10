/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the inside of the script screens: what the queue (scripting/scriptscreens.cpp) asks of a language runtime
// (scripting/luaprogram.cpp, scripting/wasmprogram.cpp), and the helpers they share

#include "scripting/scriptscreens.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace script_screens
{

// the largest side of a screen image, in pixels
constexpr int max_image_side{4096};
// the most instructions one call into a screen may run, so that a script stuck in a loop can't hang the worker
constexpr std::int64_t instruction_limit{200'000'000};

struct pending_command
{
	std::string name;
	std::int32_t param1{0};
	std::int32_t param2{0};
	std::uint32_t vehicle{0};
};

// one refresh of a screen: what it gets and what it gives back
struct render_call
{
	dictionary_source const &input;
	std::uint32_t vehicle{0}; // the target of the commands
	std::vector<pending_command> commands; // sent during the call
	// the image drawn, RGBA, from the top row down; empty when the screen didn't change
	std::vector<std::uint8_t> pixels;
	int width{0};
	int height{0};
	// the post-processing still to run on the image (image:led_post, image:resize of the Lua screens)
	std::vector<post_step> post;
};

// a running script: a Lua state, a WebAssembly instance. made, called and destroyed on the worker thread, or by the
// queue once the worker stopped
class program
{
  public:
	program() = default;
	virtual ~program() = default;
	program(program const &) = delete;
	program &operator=(program const &) = delete;
	// runs a refresh. false when the screen can't run (any more); the reason is in the log
	virtual bool render(render_call &Call) = 0;
};

// a language: started and stopped with the queue
class runtime
{
  public:
	runtime() = default;
	virtual ~runtime() = default;
	runtime(runtime const &) = delete;
	runtime &operator=(runtime const &) = delete;
	// loads the script of a screen and runs its setup. nullptr when it fails; the reason is in the log
	virtual std::unique_ptr<program> load(std::string const &Script, script_screen_kind Kind) = 0;
	// called on the worker thread when it starts and before it ends
	virtual void attach_thread() {}
	virtual void detach_thread() {}
};

std::unique_ptr<runtime> make_lua_runtime();
std::unique_ptr<runtime> make_wasm_runtime();

// the text as UTF-8: names of sceneries and vehicles may come in windows-1250
std::string utf8_text(std::string const &Text);
// a file of the screen's folder or below; nothing for a path leading outside, or no such file
std::optional<std::string> read_screen_file(std::string const &Folder, std::string_view Path);
// seconds of a steady clock, the same for every screen, for animations that don't depend on how often the screen is
// refreshed
double screen_time();
bool valid_image_side(int Side);

} // namespace script_screens
