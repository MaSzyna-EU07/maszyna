/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the wasm screens of a build without WITH_WASM: there are none, and the vehicles fall back to what they had before

#include "stdafx.h"
#include "scripting/wasmscreens.h"

struct wasm_screen_queue::implementation
{
};

wasm_screen_queue::wasm_screen_queue() : m_impl{std::make_unique<implementation>()} {}

wasm_screen_queue::~wasm_screen_queue() = default;

bool wasm_screen_queue::init()
{
	return false;
}

void wasm_screen_queue::exit()
{
	// nothing was started
}

std::shared_ptr<wasm_screen> wasm_screen_queue::acquire(screen_request const & /*Request*/)
{
	return nullptr;
}

bool wasm_screen_queue::due(wasm_screen & /*Screen*/) const
{
	return false;
}

void wasm_screen_queue::submit(std::shared_ptr<wasm_screen> const & /*Screen*/, std::shared_ptr<dictionary_source> /*Input*/, std::uint32_t /*Vehicle*/)
{
	// nothing to refresh
}

void wasm_screen_queue::update()
{
	// nothing to upload
}
