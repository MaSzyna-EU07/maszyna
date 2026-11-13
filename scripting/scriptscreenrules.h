/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

class cParser;

// the parts of the script screens (Lua, WebAssembly) that don't need a runtime: the .mmd entries, the frame
// schedule and the file access rules. kept apart from scripting/scriptscreens.cpp so the unit tests can check them
namespace script_screens
{

// a script screen as the .mmd file describes it:
//   luascreen: { script.lua target: submodel frames: 4 parameters: key=value&key=value }
//   luadestinationsign: { script.lua frames: 30 instance: name parameters: ... background: texture }
// and the same with wasmscreen:, wasmdestinationsign: and a .wasm module
// every key but the script is optional, and an unknown key is skipped along with its value
struct screen_config
{
	std::string script; // the .lua script or .wasm module, relative to the vehicle folder when it has no path of its own
	std::string target{"none"}; // cab screens: the submodel the screen is drawn on
	std::string parameters; // extra key=value pairs passed to the screen with the simulation state
	std::string instancing; // destination signs: "name" or "type", one sign per vehicle or per vehicle type
	std::string background; // destination signs: the texture shown while the sign is off
	int frames{0}; // the screen's own refresh interval in frames, used when longer than the one set in eu07.ini

	void deserialize(cParser &Input);

  private:
	bool deserialize_mapping(cParser &Input);
};

// how often a screen gets fresh data, counted in rendered frames
class frame_schedule
{
  public:
	frame_schedule() = default;
	// Start: the frame the screen was created on. the first refresh comes Phase frames later
	frame_schedule(std::uint64_t Start, int Interval, int Phase);

	// the interval of a screen: the one set in eu07.ini, or the screen's own one when that is longer. at least 1
	static int effective_interval(int Configured, int Requested);
	// a fixed offset in [0, Interval) taken from the screen's name, so that the screens sharing an interval
	// don't all refresh on the same frame
	static int phase(std::string_view Key, int Interval);

	// true when the screen is due on Frame, and then not again until Interval frames later. a screen that
	// wasn't asked for a while (a vehicle out of sight) is due on the first frame it is asked again
	bool poll(std::uint64_t Frame);

	int interval() const
	{
		return m_interval;
	}

  private:
	int m_interval{1};
	std::uint64_t m_next{0};
};

// true for a file a screen may read: a relative path inside the screen's folder, without '..', without a
// drive or a leading separator
bool is_safe_relative_path(std::string_view Path);

// the path the game opens the script of a screen by: a name without a folder is in the vehicle's folder (Folder), a
// path is from the game's folder. nothing for a path that leads outside the game's folder (as for the files a screen
// reads: absolute, with a drive, or with ".."), since the script's folder is the one the screen then reads files from
std::optional<std::string> screen_script_path(std::string const &Folder, std::string const &Script);

} // namespace script_screens
