#include "stdafx.h"
#include "scripting/lua.h"
#include "world/Event.h"
#include "utilities/Logs.h"
#include "world/MemCell.h"
#include "vehicle/Driver.h"
#include "simulation/simulation.h"
#include "scripting/luasandbox.h"

lua::lua()
{
    state = luaL_newstate();
    if (!state)
        throw std::runtime_error("cannot create lua state");
    lua_atpanic(state, atpanic);
    // sandboxed as the script screens are: no io, ffi, debug, require or load*, and of os only the clock.
    // lua.fulllibraries in eu07.ini opens the rest when the scenery loads its first script (interpret())
    lua_sandbox::open_safe_libraries(state);
    lua_sandbox::open_clock_library(state);

	static constexpr luaL_Reg api[] =
	{
		{"event_create", scriptapi_event_create},
		{"event_find", scriptapi_event_find},
		{"event_exists", scriptapi_event_exists},
		{"event_getname", scriptapi_event_getname},
		{"event_dispatch", scriptapi_event_dispatch},
		{"event_dispatch_n", scriptapi_event_dispatch_n},
		{"track_find", scriptapi_track_find},
		{"track_isoccupied", scriptapi_track_isoccupied},
		{"track_isoccupied_n", scriptapi_track_isoccupied_n},
		{"isolated_find", scriptapi_isolated_find},
		{"isolated_isoccupied", scriptapi_isolated_isoccupied},
		{"isolated_isoccupied_n", scriptapi_isolated_isoccupied_n},
		{"train_getname", scriptapi_train_getname},
		{"dynobj_putvalues", scriptapi_dynobj_putvalues},
		{"memcell_find", scriptapi_memcell_find},
		{"memcell_read", scriptapi_memcell_read},
		{"memcell_read_n", scriptapi_memcell_read_n},
		{"memcell_update", scriptapi_memcell_update},
		{"memcell_update_n", scriptapi_memcell_update_n},
		{"random", scriptapi_random},
		{"writelog", scriptapi_writelog},
		{"writeerrorlog", scriptapi_writeerrorlog},
		{nullptr, nullptr}
	};
	luaL_register(state, "eu07.events", api);
}

lua::~lua()
{
    lua_close(state);
    state = nullptr;
}

std::string lua::get_error() const
{
	auto const *error = lua_tostring(state, -1);
	return error ? error : "unknown error";
}

void lua::interpret(const std::string &folder, const std::string &file)
{
	if (Global.lua_fulllibraries && !full_libraries) {
		// all of LuaJIT's libraries, as before the sandbox: the scripts may then reach files, programs and ffi
		luaL_openlibs(state);
		full_libraries = true;
		WriteLog("lua: lua.fulllibraries is on, the scenery scripts run with all libraries (io, os, ffi, require...)", logtype::lua);
	}
	// a relative path inside the scenery folder, a file of text, run in protected mode: a script that fails stops
	// there, and the scenery goes on loading
	if (auto const error = lua_sandbox::run_script_file(state, folder, file)) {
		ErrorLog("lua: " + *error, logtype::lua);
	}
}

int lua::atpanic(lua_State *s)
{
	auto const *err = lua_tostring(s, -1);
	ErrorLog(std::string("lua: Runtime error: ") + (err ? err : "unknown error"), logtype::lua);
    return 0;
}

/// Dereferences the function handler from the Lua registry.
void lua::unref(lua_State *L, const int ref)
{
	luaL_unref(L, LUA_REGISTRYINDEX, ref);
}

/// Executes the callback function of an event created with `scriptapi_event_create`.
void lua::dispatch_event(lua_State *L, const int handler, basic_event *event, const TDynamicObject *activator)
{
	lua_rawgeti(L, LUA_REGISTRYINDEX, handler);
	lua_pushlightuserdata(L, event);
	lua_pushlightuserdata(L, const_cast<TDynamicObject *>(activator));
	// in protected mode: a handler that fails is logged, and the simulation goes on
	if (lua_pcall(L, 2, 0, 0) != 0) {
		auto const *error = lua_tostring(L, -1);
		ErrorLog("lua: event " + (event ? event->m_name : std::string("?")) + ": " + (error ? error : "unknown error"), logtype::lua);
		lua_pop(L, 1);
	}
}

void lua::push_memcell_values(lua_State *L, const TMemCell *mc)
{
	lua_createtable(L, 0, 3);
	lua_pushstring(L, "str");
	lua_pushstring(L, mc ? mc->Text().c_str() : nullptr);
	lua_settable(L, -3);
	lua_pushstring(L, "num1");
	lua_pushnumber(L, mc ? mc->Value1() : 0.0);
	lua_settable(L, -3);
	lua_pushstring(L, "num2");
	lua_pushnumber(L, mc ? mc->Value2() : 0.0);
	lua_settable(L, -3);
}

memcell_values lua::get_memcell_values(lua_State *L, int idx)
{
	// For negative indices, we need to account for the extra item that is the key name/value extracted from the table.
	if (idx < 0)
		idx--;
	// no text in the table is an empty one: the memcell and the vehicle take the text as a string
	memcell_values mc{"", 0.0, 0.0};
	lua_pushstring(L, "str");
	lua_gettable(L, idx);
	if (lua_isstring(L, -1))
		mc.str = lua_tostring(L, -1);
	lua_pop(L, 1);
	lua_pushstring(L, "num1");
	lua_gettable(L, idx);
	if (lua_isnumber(L, -1))
		mc.num1 = lua_tonumber(L, -1);
	lua_pop(L, 1);
	lua_pushstring(L, "num2");
	lua_gettable(L, idx);
	if (lua_isnumber(L, -1))
		mc.num2 = lua_tonumber(L, -1);
	lua_pop(L, 1);
	return mc;
}

int lua::scriptapi_event_create(lua_State *L)
{
	std::string name = luaL_checkstring(L, 1);
	double delay = lua_tonumber(L, 2);
	double randomdelay = lua_tonumber(L, 3);
	lua_pushvalue(L, 4);
	int funcRef = luaL_ref(L, LUA_REGISTRYINDEX);

	basic_event *event = new lua_event(L, funcRef);
	event->m_name = name;
	event->m_delay = delay;
	event->m_delayrandom = randomdelay;
	if (simulation::Events.insert(event))
	{
		lua_pushlightuserdata(L, event);
		return 1;
	}
	return 0;
}

int lua::scriptapi_event_find(lua_State *L)
{
	std::string name = luaL_checkstring(L, 1);
	basic_event *foundevent = simulation::Events.FindEvent(name);
	if (foundevent)
	{
		lua_pushlightuserdata(L, foundevent);
		return 1;
	}
	ErrorLog("lua: missing event: " + name);
	return 0;
}

int lua::scriptapi_event_exists(lua_State *L)
{
	std::string name = luaL_checkstring(L, 1);
	basic_event *foundevent = simulation::Events.FindEvent(name);
	lua_pushboolean(L, foundevent != nullptr);
	return 1;
}

int lua::scriptapi_event_getname(lua_State *L)
{
	auto *event = static_cast<basic_event *>(lua_touserdata(L, 1));
	if (event)
	{
		lua_pushstring(L, event->m_name.c_str());
		return 1;
	}
	return 0;
}

int lua::scriptapi_event_dispatch(lua_State *L)
{
	auto *event = static_cast<basic_event *>(lua_touserdata(L, 1));
	auto *activator = static_cast<TDynamicObject *>(lua_touserdata(L, 2));
	double delay = lua_tonumber(L, 3);
	if (event)
		simulation::Events.AddToQuery(event, activator, delay);
	return 0;
}

int lua::scriptapi_event_dispatch_n(lua_State *L)
{
	std::string name = luaL_checkstring(L, 1);
	auto *activator = static_cast<TDynamicObject *>(lua_touserdata(L, 2));
	double delay = lua_tonumber(L, 3);
	basic_event *foundevent = simulation::Events.FindEvent(name);
	if (foundevent)
		simulation::Events.AddToQuery(foundevent, activator, delay);
	else
		ErrorLog("lua: missing event: " + name);
	return 0;
}

int lua::scriptapi_track_find(lua_State *L)
{
	std::string name = luaL_checkstring(L, 1);
	TTrack *track = simulation::Paths.find(name);
	if (track)
	{
		lua_pushlightuserdata(L, track);
		return 1;
	}
	ErrorLog("lua: missing track: " + name);
	return 0;
}

int lua::scriptapi_track_isoccupied(lua_State *L)
{
	auto *track = static_cast<TTrack *>(lua_touserdata(L, 1));
	if (track)
	{
		lua_pushboolean(L, !track->IsEmpty());
		return 1;
	}
	return 0;
}

int lua::scriptapi_track_isoccupied_n(lua_State *L)
{
	std::string name = luaL_checkstring(L, 1);
	TTrack *track = simulation::Paths.find(name);
	if (track)
	{
		lua_pushboolean(L, !track->IsEmpty());
		return 1;
	}
	ErrorLog("lua: missing track: " + name);
	return 0;
}

int lua::scriptapi_isolated_find(lua_State *L)
{
	std::string name = luaL_checkstring(L, 1);
	TIsolated *isolated = TIsolated::Find(name);
	if (isolated)
	{
		lua_pushlightuserdata(L, isolated);
		return 1;
	}
	ErrorLog("lua: missing isolated: " + name);
	return 0;
}

int lua::scriptapi_isolated_isoccupied(lua_State *L)
{
	auto *isolated = static_cast<TIsolated *>(lua_touserdata(L, 1));
	if (isolated)
	{
		lua_pushboolean(L, isolated->Busy());
		return 1;
	}
	return 0;
}

int lua::scriptapi_isolated_isoccupied_n(lua_State *L)
{
	std::string name = luaL_checkstring(L, 1);
	TIsolated *isolated = TIsolated::Find(name);
	if (isolated)
	{
		lua_pushboolean(L, isolated->Busy());
		return 1;
	}
	ErrorLog("lua: missing isolated: " + name);
	return 0;
}

int lua::scriptapi_train_getname(lua_State *L)
{
	auto *dyn = static_cast<TDynamicObject *>(lua_touserdata(L, 1));
	if (dyn && dyn->Mechanik)
	{
		lua_pushstring(L, dyn->Mechanik->TrainName().c_str());
		return 1;
	}
	return 0;
}

int lua::scriptapi_dynobj_putvalues(lua_State *L)
{
	auto *dyn = static_cast<TDynamicObject *>(lua_touserdata(L, 1));
	auto [str, num1, num2] = get_memcell_values(L, 2);
	if (!dyn)
		return 0;
	TLocation loc{};
	if (dyn->Mechanik)
		dyn->Mechanik->PutCommand(str, num1, num2, loc);
	else
		dyn->MoverParameters->PutCommand(str, num1, num2, loc);
	return 0;
}

int lua::scriptapi_memcell_find(lua_State *L)
{
	std::string str = luaL_checkstring(L, 1);
	TMemCell *mc = simulation::Memory.find(str);
	if (mc)
	{
		lua_pushlightuserdata(L, mc);
		return 1;
	}
	ErrorLog("lua: missing memcell: " + str);
	return 0;
}

int lua::scriptapi_memcell_read(lua_State *L)
{
	auto *mc = static_cast<TMemCell *>(lua_touserdata(L, 1));
	push_memcell_values(L, mc);
	return 1;
}

int lua::scriptapi_memcell_read_n(lua_State *L)
{
	std::string str = luaL_checkstring(L, 1);
	TMemCell *mc = simulation::Memory.find(str);
	push_memcell_values(L, mc);
	if (!mc)
		ErrorLog("lua: missing memcell: " + str);
	return 1;
}

int lua::scriptapi_memcell_update(lua_State *L)
{
	auto *mc = static_cast<TMemCell *>(lua_touserdata(L, 1));
	auto [str, num1, num2] = get_memcell_values(L, 2);
	if (mc)
		mc->UpdateValues(str, num1, num2,
					 basic_event::flags::text | basic_event::flags::value1 | basic_event::flags::value2);
	return 0;
}

int lua::scriptapi_memcell_update_n(lua_State *L)
{
	std::string mstr = luaL_checkstring(L, 1);
	auto [str, num1, num2] = get_memcell_values(L, 2);
	TMemCell *mc = simulation::Memory.find(mstr);
	if (mc)
		mc->UpdateValues(str, num1, num2,
					 basic_event::flags::text | basic_event::flags::value1 | basic_event::flags::value2);
	else
		ErrorLog("lua: missing memcell: " + mstr);
	return 0;
}

int lua::scriptapi_random(lua_State *L)
{
	double a = lua_tonumber(L, 1);
	double b = lua_tonumber(L, 2);
	lua_pushnumber(L, Random(a, b));
	return 1;
}

int lua::scriptapi_writelog(lua_State *L)
{
	std::string txt = luaL_checkstring(L, 1);
	WriteLog("lua: log: " + txt, logtype::lua);
	return 0;
}

int lua::scriptapi_writeerrorlog(lua_State *L)
{
	std::string txt = luaL_checkstring(L, 1);
	ErrorLog("lua: log: " + txt, logtype::lua);
	return 0;
}
