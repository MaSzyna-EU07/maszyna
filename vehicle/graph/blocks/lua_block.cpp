/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_catalog.h"
#include "vehicle/graph/graph_format.h"
#include "vehicle/graph/graph_lua.h"

#include <cmath>

#ifdef WITH_LUA
#include <lua.hpp>
#endif

// function block defined by a lua script. its ports and numeric parameters come from the script
namespace vgraph
{

namespace
{

bool resolve_lua_block(node_desc const &Node, std::vector<port_def> &Ports, std::vector<param_def> &Params, std::string &Error)
{
	if (false == lua_available())
	{
		Error = "lua blocks need a simulator built with lua support (WITH_LUA)";
		return false;
	}
	if (Node.script.empty())
	{
		Error = "lua block without a script";
		return false;
	}
	lua_block_definition definition;
	if (false == parse_lua_block(Node.script, Node.id, definition, Error))
	{
		Error = "lua: " + Error;
		return false;
	}
	Ports = std::move(definition.ports);
	Params = std::move(definition.params);
	return true;
}

#ifdef WITH_LUA

class lua_block final : public block
{
public:
	explicit lua_block(block_setup const &Setup) : block(Setup), m_host(Setup.lua), m_nodeid(Setup.node_id)
	{
		for (std::size_t idx = 0; idx < Setup.ports.size(); ++idx)
		{
			auto const &port{Setup.ports[idx]};
			(port.direction == port_direction::in ? m_inputs : m_outputs).push_back({idx, port.key, port.type});
		}
		if (m_host == nullptr || m_host->state() == nullptr)
		{
			fail("no lua state");
			return;
		}
		std::string error;
		m_definition = m_host->load_block(Setup.script, Setup.node_id, error);
		if (m_definition < 0)
		{
			fail(error);
			return;
		}
		auto *L{m_host->state()};
		auto const top{lua_gettop(L)};
		lua_rawgeti(L, LUA_REGISTRYINDEX, m_definition);
		auto const definition{lua_gettop(L)};
		lua_getfield(L, definition, "rate");
		auto const rate{lua_isnumber(L, -1) ? lua_tonumber(L, -1) : 0.0};
		lua_pop(L, 1);
		m_period = (rate > 0.0 ? 1.0 / rate : 0.0);
		// self: copy of the state table, and the parameters with values from the graph for the numeric ones
		lua_newtable(L);
		auto const self{lua_gettop(L)};
		lua_getfield(L, definition, "state");
		if (lua_istable(L, -1))
		{
			lua_pushnil(L);
			while (lua_next(L, -2) != 0)
			{
				lua_pushvalue(L, -2);
				lua_insert(L, -2);
				lua_rawset(L, self);
			}
		}
		lua_pop(L, 1);
		lua_getfield(L, definition, "params");
		if (lua_istable(L, -1))
		{
			lua_pushnil(L);
			while (lua_next(L, -2) != 0)
			{
				if (lua_type(L, -2) == LUA_TSTRING && lua_type(L, -1) == LUA_TNUMBER)
				{
					auto const paramindex{find_param(Setup.param_defs, std::string{"lua."} + lua_tostring(L, -2))};
					if (paramindex >= 0)
					{
						lua_pop(L, 1);
						lua_pushnumber(L, Setup.params[paramindex].number);
					}
				}
				lua_pushvalue(L, -2);
				lua_insert(L, -2);
				lua_rawset(L, self);
			}
		}
		lua_pop(L, 1);
		m_self = luaL_ref(L, LUA_REGISTRYINDEX);
		lua_newtable(L);
		m_in = luaL_ref(L, LUA_REGISTRYINDEX);
		lua_newtable(L);
		m_out = luaL_ref(L, LUA_REGISTRYINDEX);
		lua_settop(L, top);
	}

	~lua_block() override
	{
		if (m_host != nullptr)
		{
			m_host->release(m_definition);
			m_host->release(m_self);
			m_host->release(m_in);
			m_host->release(m_out);
		}
	}

	void update(frame &Frame) override
	{
		if (m_fault)
		{
			// outputs keep their last values
			return;
		}
		auto dt{Frame.dt};
		if (m_period > 0.0)
		{
			m_accumulated += Frame.dt;
			if (m_accumulated + 1e-9 < m_period)
			{
				return;
			}
			dt = m_accumulated;
			m_accumulated = 0.0;
		}
		auto *L{m_host->state()};
		auto const top{lua_gettop(L)};
		lua_rawgeti(L, LUA_REGISTRYINDEX, m_in);
		for (auto const &input : m_inputs)
		{
			auto const value{get(Frame, input.port)};
			switch (input.type)
			{
			case value_type::boolean:
				lua_pushboolean(L, value > 0.5);
				break;
			case value_type::integer:
				lua_pushnumber(L, std::round(value));
				break;
			default:
				lua_pushnumber(L, value);
				break;
			}
			lua_setfield(L, -2, input.key.c_str());
		}
		lua_pop(L, 1);
		lua_rawgeti(L, LUA_REGISTRYINDEX, m_definition);
		lua_getfield(L, -1, "update");
		lua_remove(L, -2);
		lua_rawgeti(L, LUA_REGISTRYINDEX, m_self);
		lua_rawgeti(L, LUA_REGISTRYINDEX, m_in);
		lua_rawgeti(L, LUA_REGISTRYINDEX, m_out);
		lua_pushnumber(L, dt);
		std::string error;
		if (false == m_host->call(4, error))
		{
			lua_settop(L, top);
			fail(error);
			return;
		}
		lua_rawgeti(L, LUA_REGISTRYINDEX, m_out);
		for (auto const &output : m_outputs)
		{
			lua_getfield(L, -1, output.key.c_str());
			auto value{0.0};
			if (lua_isboolean(L, -1))
			{
				value = (lua_toboolean(L, -1) ? 1.0 : 0.0);
			}
			else if (lua_type(L, -1) == LUA_TNUMBER)
			{
				value = lua_tonumber(L, -1);
				if (output.type == value_type::integer)
				{
					value = std::round(value);
				}
			}
			lua_pop(L, 1);
			set(Frame, output.port, value);
		}
		lua_settop(L, top);
	}

	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "fault";
	}
	double state_value(std::size_t) const override
	{
		return m_fault ? 1.0 : 0.0;
	}

private:
	struct port_entry
	{
		std::size_t port;
		std::string key;
		value_type type;
	};

	void fail(std::string const &Reason)
	{
		// a failing block stops, and the rest of the vehicle keeps working
		m_fault = true;
		log_message("lua [" + m_nodeid + "]: block stopped: " + Reason);
	}

	lua_host *m_host;
	std::string m_nodeid;
	std::vector<port_entry> m_inputs;
	std::vector<port_entry> m_outputs;
	int m_definition{-1};
	int m_self{-1};
	int m_in{-1};
	int m_out{-1};
	double m_period{0.0};
	double m_accumulated{0.0};
	bool m_fault{false};
};

#endif

} // namespace

void register_script_blocks(catalog &Catalog)
{
	block_type type;
	type.id = "lua.block";
	type.category = "script";
	type.name = "Lua block";
	type.description = "Function block defined by a lua 5.1 script: block \"name\" { inputs, outputs, params, state, rate, update = function(self, i, o, dt) ... end }.";
	type.origin = block_origin::script;
	type.dynamic_ports = true;
	type.resolve = resolve_lua_block;
#ifdef WITH_LUA
	type.create = make_block<lua_block>;
#else
	// graphs with lua blocks are rejected when loaded, so blocks of this type are never created
	type.create = nullptr;
#endif
	Catalog.add(std::move(type));
}

} // namespace vgraph
