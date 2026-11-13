#pragma once
#include <lua.hpp>
#include <stdafx.h>
#include "scripting/luasandbox.h"

class basic_event;
class TMemCell;
class TDynamicObject;
struct memcell_values { const char *str; double num1; double num2; };

class lua
{
    lua_State *state;
	bool full_libraries = false; // lua.fulllibraries: all of LuaJIT's libraries opened
	// what eu07.read_file reads (the scenery folder) and eu07.write_file writes (the scenery's data folder), known
	// when the scenery loads its scripts (interpret())
	lua_sandbox::file_folders files;

    static int atpanic(lua_State *s);

	static int scriptapi_event_create(lua_State *L);
	static int scriptapi_event_find(lua_State *L);
	static int scriptapi_event_exists(lua_State *L);
	static int scriptapi_event_getname(lua_State *L);
	static int scriptapi_event_dispatch(lua_State *L);
	static int scriptapi_event_dispatch_n(lua_State *L);
	static int scriptapi_track_find(lua_State *L);
	static int scriptapi_track_isoccupied(lua_State *L);
	static int scriptapi_track_isoccupied_n(lua_State *L);
	static int scriptapi_isolated_find(lua_State *L);
	static int scriptapi_isolated_isoccupied(lua_State *L);
	static int scriptapi_isolated_isoccupied_n(lua_State *L);
	static int scriptapi_train_getname(lua_State *L);
	static int scriptapi_dynobj_putvalues(lua_State *L);
	static int scriptapi_memcell_find(lua_State *L);
	static int scriptapi_memcell_read(lua_State *L);
	static int scriptapi_memcell_read_n(lua_State *L);
	static int scriptapi_memcell_update(lua_State *L);
	static int scriptapi_memcell_update_n(lua_State *L);
	static int scriptapi_random(lua_State *L);
	static int scriptapi_writelog(lua_State *L);
	static int scriptapi_writeerrorlog(lua_State *L);

public:
    lua();
    ~lua();

	std::string get_error() const;
	// runs the script file, a relative path inside folder (the scenery folder), sandboxed (scripting/luasandbox.h).
	// a script that can't be read or fails is logged
	void interpret(const std::string &folder, const std::string &file);
	static void unref(lua_State *L, int ref);
	static void dispatch_event(lua_State *L, int handler, basic_event *event, const TDynamicObject *activator);
	static void push_memcell_values(lua_State *L, const TMemCell *mc);
	static memcell_values get_memcell_values(lua_State *L, int idx);
};
