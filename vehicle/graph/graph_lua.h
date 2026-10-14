/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "vehicle/graph/graph_types.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct lua_State;

// lua function blocks of the vehicle graph. each vehicle gets a lua state of its own, separate from the scenery's
// one, and each block an environment of its own within it. scripts run in lua 5.1 (luajit) with a restricted set of
// libraries: no files, operating system, modules, ffi or bytecode loading. the jit compiler is turned off for these
// states, so the instruction budget which stops endless loops works reliably
namespace vgraph
{

// messages of the lua blocks (print, errors). the simulator directs them to its log, the command line tools to the console
using log_sink = std::function<void(std::string const &)>;
void set_log_sink(log_sink Sink);
void log_message(std::string const &Message);

// whether this build of the simulator can run lua blocks (WITH_LUA)
bool lua_available();

// interface of a lua block, read from its script:
// block "name" { inputs = { a = "bool" }, outputs = { q = "float" }, params = { k = 1.0 }, state = { ... }, rate = 20,
//                update = function(self, i, o, dt) ... end }
struct lua_block_definition
{
	std::string name;
	std::vector<port_def> ports; // inputs then outputs, each sorted by key so the order doesn't depend on the lua hash
	std::vector<param_def> params; // numeric parameters of the script as "lua.<key>", with the script's values as defaults
	double rate{0.0}; // updates per second; 0 updates the block in every step
};

// runs the script in a temporary state and reads the block interface. returns false with the reason on failure
bool parse_lua_block(std::string const &Script, std::string const &Chunkname, lua_block_definition &Definition, std::string &Error);

// lua state shared by the lua blocks of one vehicle
class lua_host
{
public:
	lua_host();
	~lua_host();
	lua_host(lua_host const &) = delete;
	lua_host &operator=(lua_host const &) = delete;

	lua_State *state() const
	{
		return m_state;
	}
	// runs the script of a block in a new environment and returns the registry reference of the block definition table,
	// or a negative value with the reason on failure. the environment's random generator is seeded from the node id
	int load_block(std::string const &Script, std::string const &Nodeid, std::string &Error);
	// calls the function placed on the stack below its arguments, within the instruction budget. returns false with the reason on failure
	bool call(int Arguments, std::string &Error);
	void release(int Reference);

private:
	lua_State *m_state{nullptr};
};

// random generator used by math.random of the lua blocks; the external editor uses the same one, so previews match
struct block_random
{
	std::uint64_t state;

	explicit block_random(std::uint64_t Seed);
	double next(); // [0, 1)
	static std::uint64_t seed_from(std::string const &Text);
};

} // namespace vgraph
