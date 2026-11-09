/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scripting/wasmscreenrules.h"

#include <cstdint>
#include <memory>
#include <string>

struct ITexture;
struct dictionary_source;

// screens drawn by WebAssembly modules, the counterpart of the python screens (scripting/PyInt.h).
// a module gets the same state the python screens get, returns an RGBA image and may send commands. the
// interface between the two is described in wasm_screens_doc.md.
// unlike the python screens, which refresh after a time, a wasm screen refreshes every so many rendered
// frames: wasm.cabframes in eu07.ini for the screens in the cab, wasm.externalframes for the screens outside
// (destination signs)

enum class wasm_screen_kind
{
	cab, // in the cab of the driven vehicle: may send commands to it
	external // on the outside of a vehicle, such as a destination sign: shows only
};

// one screen: a module instance with its own memory, and the image it drew last
class wasm_screen;

class wasm_screen_queue
{
  public:
	struct screen_request
	{
		wasm_screen_kind kind{wasm_screen_kind::cab};
		std::string script; // the .wasm file
		std::string key; // requests with the same key get the same screen, e.g. the destination signs of one train
		ITexture *texture{nullptr}; // where the screen is drawn; none for a screen that only sends commands
		int frames{0}; // the screen's own refresh interval from the .mmd, used when longer than the eu07.ini one
	};

	wasm_screen_queue();
	~wasm_screen_queue();
	wasm_screen_queue(wasm_screen_queue const &) = delete;
	wasm_screen_queue &operator=(wasm_screen_queue const &) = delete;

	// starts the runtime and its worker thread. returns true on success
	bool init();
	// stops the worker and releases every screen, also the ones still held by vehicles
	void exit();
	// the screen for the request, made on the first request with its key. nullptr when the wasm screens are off
	std::shared_ptr<wasm_screen> acquire(screen_request const &Request);
	// true once per refresh interval of the screen. the caller then gathers the screen's input and submit()s it
	bool due(wasm_screen &Screen) const;
	// queues a refresh of the screen with the given input, replacing a refresh of it still waiting.
	// Vehicle: the vehicle that gets the commands the screen sends; cab screens only
	void submit(std::shared_ptr<wasm_screen> const &Screen, std::shared_ptr<dictionary_source> Input, std::uint32_t Vehicle);
	// main thread, once per rendered frame: uploads the finished images, runs the commands the screens sent and
	// counts the frame
	void update();

  private:
	struct implementation;
	std::unique_ptr<implementation> m_impl;
};
