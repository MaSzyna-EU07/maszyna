/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scripting/scriptscreenrules.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

struct ITexture;
struct dictionary_source;

// screens drawn by scripts, the counterpart of the python screens (scripting/PyInt.h): Lua scripts run by LuaJIT
// (luascreen:, luadestinationsign:) and WebAssembly modules run by WAMR (wasmscreen:, wasmdestinationsign:). a script
// gets the state a python screen gets, returns an RGBA image and, in the cab, may send commands. the interface is
// described in script_screens_doc.md.
// unlike the python screens, which refresh after a time, a script screen refreshes every so many rendered frames:
// screens.cabframes in eu07.ini for the screens in the cab, screens.externalframes for the screens outside
// (destination signs)

enum class script_screen_kind
{
	cab, // in the cab of the driven vehicle: may send commands to it
	external // on the outside of a vehicle, such as a destination sign: shows only
};

enum class script_language
{
	lua,
	wasm
};

// the language of a script, by its extension: .lua or .wasm
inline std::optional<script_language> script_language_of(std::string_view const Script)
{
	if (Script.ends_with(".lua"))
	{
		return script_language::lua;
	}
	if (Script.ends_with(".wasm"))
	{
		return script_language::wasm;
	}
	return std::nullopt;
}

// one screen: a running script with its own state, and the image it drew last
class script_screen;

class script_screen_queue
{
  public:
	struct screen_request
	{
		script_screen_kind kind{script_screen_kind::cab};
		script_language language{script_language::lua};
		std::string script; // the .lua script or .wasm module
		std::string key; // requests with the same key get the same screen, e.g. the destination signs of one train
		ITexture *texture{nullptr}; // where the screen is drawn; none for a screen that only sends commands
		int frames{0}; // the screen's own refresh interval from the .mmd, used when longer than the eu07.ini one
	};

	script_screen_queue();
	~script_screen_queue();
	script_screen_queue(script_screen_queue const &) = delete;
	script_screen_queue &operator=(script_screen_queue const &) = delete;

	// starts the runtimes switched on in eu07.ini and the worker thread. returns true when any runtime started
	bool init();
	// stops the worker and releases every screen, also the ones still held by vehicles
	void exit();
	// true when screens in the language can run
	bool runs(script_language Language) const;
	// the screen for the request, made on the first request with its key. nullptr when its language doesn't run
	std::shared_ptr<script_screen> acquire(screen_request const &Request);
	// true once per refresh interval of the screen. the caller then gathers the screen's input and submit()s it
	bool due(script_screen &Screen) const;
	// queues a refresh of the screen with the given input, replacing a refresh of it still waiting.
	// Vehicle: the vehicle that gets the commands the screen sends; cab screens only
	void submit(std::shared_ptr<script_screen> const &Screen, std::shared_ptr<dictionary_source> Input, std::uint32_t Vehicle);
	// main thread, once per rendered frame: uploads the finished images, runs the commands the screens sent and
	// counts the frame
	void update();

  private:
	struct implementation;
	std::unique_ptr<implementation> m_impl;
};
