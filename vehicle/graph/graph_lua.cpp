/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_lua.h"

#include <algorithm>
#include <cmath>

#ifdef WITH_LUA
#include <lua.hpp>
#endif

namespace vgraph
{

namespace
{

log_sink &sink()
{
	static log_sink instance;
	return instance;
}

} // namespace

void set_log_sink(log_sink Sink)
{
	sink() = std::move(Sink);
}

void log_message(std::string const &Message)
{
	if (sink())
	{
		sink()(Message);
	}
}

// block_random

block_random::block_random(std::uint64_t const Seed) : state(Seed != 0 ? Seed : 0x9E3779B97F4A7C15ull) {}

double block_random::next()
{
	// xorshift64*: small, fast, and easy to reproduce exactly in the editor
	state ^= state >> 12;
	state ^= state << 25;
	state ^= state >> 27;
	return static_cast<double>((state * 0x2545F4914F6CDD1Dull) >> 11) * (1.0 / 9007199254740992.0);
}

std::uint64_t block_random::seed_from(std::string const &Text)
{
	// fnv-1a
	std::uint64_t hash{0xcbf29ce484222325ull};
	for (auto const character : Text)
	{
		hash ^= static_cast<unsigned char>(character);
		hash *= 0x100000001b3ull;
	}
	return hash;
}

#ifdef WITH_LUA

namespace
{

// keys of values kept out of reach of the scripts: light userdata can't be created from lua
char const block_definition_key{0};
char const block_name_key{0};

// instructions between checks of the budget, and the number of checks allowed in a single call
int constexpr hook_interval{1000};
int constexpr call_budget{2000}; // 2 million instructions; far more than any block needs in one step

thread_local int budget_remaining{0};

void budget_hook(lua_State *L, lua_Debug *)
{
	if (--budget_remaining <= 0)
	{
		budget_remaining = call_budget; // leave room for the error handling
		luaL_error(L, "instruction budget exceeded, endless loop?");
	}
}

void open_library(lua_State *L, lua_CFunction Open, char const *Name)
{
	lua_pushcfunction(L, Open);
	lua_pushstring(L, Name);
	lua_call(L, 1, 0);
}

// copies the fields of the table at Source into a new table pushed on the stack
void push_copy(lua_State *L, int const Source)
{
	auto const source{Source < 0 ? lua_gettop(L) + Source + 1 : Source};
	lua_newtable(L);
	lua_pushnil(L);
	while (lua_next(L, source) != 0)
	{
		lua_pushvalue(L, -2);
		lua_insert(L, -2);
		lua_rawset(L, -4);
	}
}

// math.random and math.randomseed with the generator in the closure's upvalue, the same semantics as in lua 5.1
int script_random(lua_State *L)
{
	auto *random{static_cast<block_random *>(lua_touserdata(L, lua_upvalueindex(1)))};
	auto const r{random->next()};
	switch (lua_gettop(L))
	{
	case 0:
	{
		lua_pushnumber(L, r);
		return 1;
	}
	case 1:
	{
		auto const upper{luaL_checknumber(L, 1)};
		luaL_argcheck(L, 1.0 <= upper, 1, "interval is empty");
		lua_pushnumber(L, std::floor(r * std::floor(upper)) + 1.0);
		return 1;
	}
	case 2:
	{
		auto const lower{std::floor(luaL_checknumber(L, 1))};
		auto const upper{std::floor(luaL_checknumber(L, 2))};
		luaL_argcheck(L, lower <= upper, 2, "interval is empty");
		lua_pushnumber(L, std::floor(r * (upper - lower + 1.0)) + lower);
		return 1;
	}
	default:
	{
		return luaL_error(L, "wrong number of arguments");
	}
	}
}

int script_randomseed(lua_State *L)
{
	auto *random{static_cast<block_random *>(lua_touserdata(L, lua_upvalueindex(1)))};
	// splitmix64 finalizer spreads small seeds over the whole state
	auto x{static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(luaL_checknumber(L, 1))))};
	x += 0x9E3779B97F4A7C15ull;
	x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
	x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
	x ^= x >> 31;
	*random = block_random{x};
	return 0;
}

// print sends the message to the log, prefixed with the node id; limited, so a script printing in every step can't flood the log
int script_print(lua_State *L)
{
	auto const count{static_cast<int>(lua_tointeger(L, lua_upvalueindex(2)))};
	if (count >= 50)
	{
		return 0;
	}
	lua_pushinteger(L, count + 1);
	lua_replace(L, lua_upvalueindex(2));
	std::string message{"lua ["};
	message += lua_tostring(L, lua_upvalueindex(1));
	message += "]:";
	auto const arguments{lua_gettop(L)};
	lua_getglobal(L, "tostring");
	for (int idx = 1; idx <= arguments; ++idx)
	{
		lua_pushvalue(L, -1);
		lua_pushvalue(L, idx);
		lua_call(L, 1, 1);
		auto const *text{lua_tostring(L, -1)};
		message += ' ';
		message += (text != nullptr ? text : "?");
		lua_pop(L, 1);
	}
	lua_pop(L, 1);
	if (count + 1 == 50)
	{
		message += " (further messages of this block are suppressed)";
	}
	log_message(message);
	return 0;
}

// block "name" returns a function, which takes the definition table and stores it in the environment
int script_block_definition(lua_State *L)
{
	luaL_checktype(L, 1, LUA_TTABLE);
	lua_pushvalue(L, lua_upvalueindex(1)); // environment
	lua_pushlightuserdata(L, const_cast<char *>(&block_definition_key));
	lua_pushvalue(L, 1);
	lua_rawset(L, -3);
	lua_pushlightuserdata(L, const_cast<char *>(&block_name_key));
	lua_pushvalue(L, lua_upvalueindex(2));
	lua_rawset(L, -3);
	return 0;
}

int script_block(lua_State *L)
{
	if (lua_type(L, 1) == LUA_TTABLE)
	{
		// block { ... } without a name
		lua_pushstring(L, "");
		lua_insert(L, 1);
	}
	luaL_checkstring(L, 1);
	lua_pushvalue(L, lua_upvalueindex(1)); // environment
	lua_pushvalue(L, 1); // name
	lua_pushcclosure(L, script_block_definition, 2);
	if (lua_gettop(L) >= 3 && lua_type(L, 2) == LUA_TTABLE)
	{
		// block("name", { ... })
		lua_pushvalue(L, 2);
		lua_call(L, 1, 0);
		return 0;
	}
	return 1;
}

// libraries available to the scripts. everything else (io, os, package, debug, ffi, jit, loading code) stays out
char const *const base_functions[]{"assert", "error",   "ipairs", "next",         "pairs",        "pcall",  "select",   "tonumber",
                                   "tostring", "type", "unpack", "xpcall", "rawequal", "rawget", "rawset", "setmetatable",
                                   "getmetatable", "_VERSION"};
char const *const libraries[]{"string", "table", "math", "bit", "coroutine"};

lua_State *create_state()
{
	auto *L{luaL_newstate()};
	if (L == nullptr)
	{
		return nullptr;
	}
	// interpreter only: instruction count hooks don't run inside compiled traces, and the budget has to stop endless loops
	luaJIT_setmode(L, 0, LUAJIT_MODE_ENGINE | LUAJIT_MODE_OFF);
	open_library(L, luaopen_base, "");
	open_library(L, luaopen_table, LUA_TABLIBNAME);
	open_library(L, luaopen_string, LUA_STRLIBNAME);
	open_library(L, luaopen_math, LUA_MATHLIBNAME);
	open_library(L, luaopen_bit, LUA_BITLIBNAME);
	// string.dump would give access to bytecode
	lua_getglobal(L, "string");
	lua_pushnil(L);
	lua_setfield(L, -2, "dump");
	lua_pop(L, 1);
	// the metatable of strings leads to the shared string library; hiding it keeps the blocks from changing it for others
	lua_pushstring(L, "");
	if (lua_getmetatable(L, -1) != 0)
	{
		lua_pushboolean(L, 0);
		lua_setfield(L, -2, "__metatable");
		lua_pop(L, 1);
	}
	lua_pop(L, 1);
	lua_sethook(L, budget_hook, LUA_MASKCOUNT, hook_interval);
	return L;
}

// pushes a new environment for the block with the specified node id
void push_environment(lua_State *L, std::string const &Nodeid)
{
	lua_newtable(L);
	auto const environment{lua_gettop(L)};
	for (auto const *name : base_functions)
	{
		lua_getglobal(L, name);
		lua_setfield(L, environment, name);
	}
	for (auto const *name : libraries)
	{
		lua_getglobal(L, name);
		if (lua_istable(L, -1))
		{
			// copies, so changes made by one block don't leak to the others
			push_copy(L, -1);
			lua_setfield(L, environment, name);
		}
		lua_pop(L, 1);
	}
	// random generator of the block, seeded from the node id
	lua_getfield(L, environment, "math");
	auto *random{static_cast<block_random *>(lua_newuserdata(L, sizeof(block_random)))};
	*random = block_random{block_random::seed_from(Nodeid)};
	lua_pushvalue(L, -1);
	lua_pushcclosure(L, script_random, 1);
	lua_setfield(L, -3, "random");
	lua_pushcclosure(L, script_randomseed, 1);
	lua_setfield(L, -2, "randomseed");
	lua_pop(L, 1);
	// print to the log
	lua_pushstring(L, Nodeid.c_str());
	lua_pushinteger(L, 0);
	lua_pushcclosure(L, script_print, 2);
	lua_setfield(L, environment, "print");
	lua_pushvalue(L, environment);
	lua_setfield(L, environment, "_G");
	// block definition
	lua_pushvalue(L, environment);
	lua_pushcclosure(L, script_block, 1);
	lua_setfield(L, environment, "block");
}

// runs the chunk on top of the stack within the budget; returns false with the message on failure
bool protected_call(lua_State *L, int const Arguments, std::string &Error)
{
	budget_remaining = call_budget;
	if (lua_pcall(L, Arguments, 0, 0) != 0)
	{
		auto const *message{lua_tostring(L, -1)};
		Error = (message != nullptr ? message : "unknown error");
		lua_pop(L, 1);
		return false;
	}
	return true;
}

// loads and runs the script in the environment on top of the stack; leaves the definition table on the stack, or returns false
bool run_script(lua_State *L, std::string const &Script, std::string const &Chunkname, std::string &Error)
{
	auto const environment{lua_gettop(L)};
	auto const chunkname{"=" + Chunkname};
	if (luaL_loadbuffer(L, Script.data(), Script.size(), chunkname.c_str()) != 0)
	{
		Error = lua_tostring(L, -1);
		lua_pop(L, 1);
		return false;
	}
	lua_pushvalue(L, environment);
	lua_setfenv(L, -2);
	if (false == protected_call(L, 0, Error))
	{
		return false;
	}
	lua_pushlightuserdata(L, const_cast<char *>(&block_definition_key));
	lua_rawget(L, environment);
	if (false == lua_istable(L, -1))
	{
		lua_pop(L, 1);
		Error = "the script doesn't call block \"name\" { ... }";
		return false;
	}
	lua_getfield(L, -1, "update");
	auto const hasupdate{lua_isfunction(L, -1)};
	lua_pop(L, 1);
	if (false == hasupdate)
	{
		lua_pop(L, 1);
		Error = "the block has no update(self, i, o, dt) function";
		return false;
	}
	return true;
}

bool is_identifier(std::string const &Text)
{
	if (Text.empty() || (Text[0] >= '0' && Text[0] <= '9'))
	{
		return false;
	}
	return std::all_of(std::begin(Text), std::end(Text), [](char const Char) { return (Char >= 'a' && Char <= 'z') || (Char >= 'A' && Char <= 'Z') || (Char >= '0' && Char <= '9') || Char == '_'; });
}

// reads inputs or outputs of the definition table on top of the stack
bool read_ports(lua_State *L, char const *Field, port_direction const Direction, std::vector<port_def> &Ports, std::string &Error)
{
	lua_getfield(L, -1, Field);
	if (lua_isnil(L, -1))
	{
		lua_pop(L, 1);
		return true;
	}
	if (false == lua_istable(L, -1))
	{
		lua_pop(L, 1);
		Error = std::string{"\""} + Field + "\" has to be a table";
		return false;
	}
	std::vector<port_def> ports;
	lua_pushnil(L);
	while (lua_next(L, -2) != 0)
	{
		if (lua_type(L, -2) != LUA_TSTRING || lua_type(L, -1) != LUA_TSTRING)
		{
			lua_pop(L, 3);
			Error = std::string{"entries of \""} + Field + "\" have to look like name = \"bool\" | \"int\" | \"float\"";
			return false;
		}
		std::string const key{lua_tostring(L, -2)};
		std::string type{lua_tostring(L, -1)};
		std::transform(std::begin(type), std::end(type), std::begin(type), [](unsigned char const Char) { return static_cast<char>(std::tolower(Char)); });
		lua_pop(L, 1);
		if (false == is_identifier(key))
		{
			lua_pop(L, 2);
			Error = "port name \"" + key + "\" has to be made of letters, digits and '_'";
			return false;
		}
		port_def port;
		port.key = key;
		port.direction = Direction;
		port.dom = domain::signal;
		port.optional = true;
		if (type == "bool")
		{
			port.type = value_type::boolean;
		}
		else if (type == "int")
		{
			port.type = value_type::integer;
		}
		else if (type == "float")
		{
			port.type = value_type::real;
		}
		else
		{
			lua_pop(L, 2);
			Error = "port \"" + key + "\" has unknown type \"" + type + "\", expected bool, int or float";
			return false;
		}
		ports.push_back(std::move(port));
	}
	lua_pop(L, 1);
	std::sort(std::begin(ports), std::end(ports), [](port_def const &Left, port_def const &Right) { return Left.key < Right.key; });
	Ports.insert(std::end(Ports), std::begin(ports), std::end(ports));
	return true;
}

} // namespace

bool lua_available()
{
	return true;
}

bool parse_lua_block(std::string const &Script, std::string const &Chunkname, lua_block_definition &Definition, std::string &Error)
{
	lua_host host;
	auto *L{host.state()};
	if (L == nullptr)
	{
		Error = "can't create lua state";
		return false;
	}
	push_environment(L, Chunkname);
	if (false == run_script(L, Script, Chunkname, Error))
	{
		return false;
	}
	// definition table on top of the stack, environment below
	lua_pushlightuserdata(L, const_cast<char *>(&block_name_key));
	lua_rawget(L, -3);
	Definition.name = (lua_isstring(L, -1) ? lua_tostring(L, -1) : "");
	lua_pop(L, 1);
	if (false == read_ports(L, "inputs", port_direction::in, Definition.ports, Error) || false == read_ports(L, "outputs", port_direction::out, Definition.ports, Error))
	{
		return false;
	}
	for (std::size_t first = 0; first < Definition.ports.size(); ++first)
	{
		for (auto second = first + 1; second < Definition.ports.size(); ++second)
		{
			if (Definition.ports[first].key == Definition.ports[second].key)
			{
				Error = "\"" + Definition.ports[first].key + "\" is both an input and an output";
				return false;
			}
		}
	}
	lua_getfield(L, -1, "params");
	if (lua_istable(L, -1))
	{
		lua_pushnil(L);
		while (lua_next(L, -2) != 0)
		{
			// numeric parameters can be set per vehicle in the graph; others stay as the script defines them
			if (lua_type(L, -2) == LUA_TSTRING && lua_type(L, -1) == LUA_TNUMBER)
			{
				param_def param;
				param.key = std::string{"lua."} + lua_tostring(L, -2);
				param.type = param_type::number;
				param.default_value = param_value::make_number(lua_tonumber(L, -1));
				Definition.params.push_back(std::move(param));
			}
			lua_pop(L, 1);
		}
	}
	lua_pop(L, 1);
	std::sort(std::begin(Definition.params), std::end(Definition.params), [](param_def const &Left, param_def const &Right) { return Left.key < Right.key; });
	lua_getfield(L, -1, "rate");
	if (false == lua_isnil(L, -1))
	{
		if (lua_type(L, -1) != LUA_TNUMBER || lua_tonumber(L, -1) < 0.0)
		{
			Error = "\"rate\" has to be a number of updates per second, 0 or more";
			return false;
		}
		Definition.rate = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);
	return true;
}

lua_host::lua_host() : m_state(create_state()) {}

lua_host::~lua_host()
{
	if (m_state != nullptr)
	{
		lua_close(m_state);
	}
}

int lua_host::load_block(std::string const &Script, std::string const &Nodeid, std::string &Error)
{
	if (m_state == nullptr)
	{
		Error = "can't create lua state";
		return LUA_NOREF;
	}
	auto *L{m_state};
	auto const top{lua_gettop(L)};
	push_environment(L, Nodeid);
	if (false == run_script(L, Script, Nodeid, Error))
	{
		lua_settop(L, top);
		return LUA_NOREF;
	}
	auto const reference{luaL_ref(L, LUA_REGISTRYINDEX)};
	lua_settop(L, top);
	return reference;
}

bool lua_host::call(int const Arguments, std::string &Error)
{
	return protected_call(m_state, Arguments, Error);
}

void lua_host::release(int const Reference)
{
	if (m_state != nullptr && Reference != LUA_NOREF && Reference != LUA_REFNIL)
	{
		luaL_unref(m_state, LUA_REGISTRYINDEX, Reference);
	}
}

#else

bool lua_available()
{
	return false;
}

bool parse_lua_block(std::string const &, std::string const &, lua_block_definition &, std::string &Error)
{
	Error = "this build of the simulator has no lua support (WITH_LUA)";
	return false;
}

lua_host::lua_host() = default;
lua_host::~lua_host() = default;

int lua_host::load_block(std::string const &, std::string const &, std::string &Error)
{
	Error = "this build of the simulator has no lua support (WITH_LUA)";
	return -1;
}

bool lua_host::call(int, std::string &Error)
{
	Error = "this build of the simulator has no lua support (WITH_LUA)";
	return false;
}

void lua_host::release(int) {}

#endif

} // namespace vgraph
