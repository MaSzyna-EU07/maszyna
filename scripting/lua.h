#pragma once
#include <lua.hpp>
#include <stdafx.h>

class basic_event;
class TMemCell;
class TDynamicObject;
struct memcell_values { const char *str; double num1; double num2; };

class lua
{
    lua_State *state;

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
    void interpret(const std::string& file) const;
	static void unref(lua_State *L, int ref);
	static void dispatch_event(lua_State *L, int handler, basic_event *event, const TDynamicObject *activator);
	static void push_memcell_values(lua_State *L, const TMemCell *mc);
	static memcell_values get_memcell_values(lua_State *L, int idx);
};
