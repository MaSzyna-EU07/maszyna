#include "stdafx.h"
#include "doctest.h"

// the sandbox the Lua states of the game share (scripting/luasandbox.cpp): the scenery scripts (lua in the .scn,
// scripting/lua.cpp) and the script screens (scripting/luaprogram.cpp, tested with the screens in
// test_script_screen_host.cpp). a script gets base, table, string, math and bit, and of os only the clock; it reaches
// no files, no programs, no ffi, loads no code of its own and no precompiled chunk, and a scenery reads its scripts
// only from its own folder

#ifdef WITH_LUA

#include "scripting/luasandbox.h"
#include "scratch.h"

#include <format>
#include <optional>
#include <string>
#include <string_view>

namespace {

// a state with the libraries of the scenery scripts
class sandboxed_state {
  public:
	sandboxed_state() : m_state{luaL_newstate()} {
		lua_sandbox::open_safe_libraries(m_state);
		lua_sandbox::open_clock_library(m_state);
	}
	~sandboxed_state() { lua_close(m_state); }
	sandboxed_state(sandboxed_state const &) = delete;
	sandboxed_state &operator=(sandboxed_state const &) = delete;

	lua_State *get() const { return m_state; }

	// runs the code, which returns one value, and gives that value as text ("nil", "true", a number, a string), or
	// "error: " and the message
	std::string run(std::string_view const Code) const {
		auto const top{lua_gettop(m_state)};
		std::string result;
		if (false == lua_sandbox::load_text_chunk(m_state, Code, "test") || 0 != lua_pcall(m_state, 0, 1, 0)) {
			result = std::string{"error: "} + lua_tostring(m_state, -1);
		} else if (lua_isnil(m_state, -1)) {
			result = "nil";
		} else if (lua_isboolean(m_state, -1)) {
			result = lua_toboolean(m_state, -1) ? "true" : "false";
		} else {
			auto const *text{lua_tostring(m_state, -1)};
			result = text != nullptr ? text : "?";
		}
		lua_settop(m_state, top);
		return result;
	}

  private:
	lua_State *m_state;
};

bool contains(std::string_view const Text, std::string_view const Part) {
	return Text.find(Part) != std::string_view::npos;
}

} // namespace

TEST_CASE("lua sandbox: no library that reaches files, programs, ffi or the debugger, and nothing that loads code") {
	sandboxed_state const lua;
	for (auto const *name : {"io", "ffi", "debug", "package", "jit", "require", "module", "dofile", "loadfile", "load", "loadstring",
	                         "string.dump", "os.execute", "os.exit", "os.remove", "os.rename", "os.getenv", "os.tmpname",
	                         "os.setlocale"}) {
		CAPTURE(name);
		CHECK(lua.run(std::format("return {}", name)) == "nil");
	}
	// what the scripts did with them fails, as a Lua error
	CHECK(contains(lua.run("return io.open('eu07.ini')"), "attempt to index global 'io' (a nil value)"));
	CHECK(contains(lua.run("return os.execute('cmd')"), "attempt to call field 'execute' (a nil value)"));
	CHECK(contains(lua.run("return require('ffi')"), "attempt to call global 'require' (a nil value)"));
}

TEST_CASE("lua sandbox: the libraries a script gets work") {
	sandboxed_state const lua;
	CHECK(lua.run("return math.floor(2.5) + #string.rep('a', 3) + bit.band(7, 4) + #table.concat({ 'a', 'b' })") == "11");
	CHECK(lua.run("return ('%d-%s'):format(7, 'x')") == "7-x");
	CHECK(lua.run("local ok, error = pcall(function() error('boom') end) return (not ok) and error:find('boom') ~= nil") == "true");
	CHECK(lua.run("local n = 0 for _, v in ipairs({ 1, 2, 3 }) do n = n + v end return n") == "6");
	CHECK(lua.run("return tostring(tonumber('12') + select('#', 1, 2))") == "14");
}

TEST_CASE("lua sandbox: os reads the clock, and nothing else") {
	sandboxed_state const lua;
	CHECK(lua.run("return type(os.time()) == 'number' and os.time() > 0") == "true");
	CHECK(lua.run("return type(os.clock()) == 'number'") == "true");
	CHECK(lua.run("return os.difftime(10, 4)") == "6");
	CHECK(lua.run("return os.time({ year = 2020, month = 1, day = 2, hour = 12 }) > 0") == "true");
	CHECK(lua.run("return os.date('!%Y-%m-%d', 0)") == "1970-01-01");
	CHECK(lua.run("return os.date('!*t', 86400 * 365).year") == "1971");
	CHECK(lua.run("return #os.date()") != "0");
	// a lone % at the end and %% are text
	CHECK(lua.run("return os.date('!100%%', 0)") == "100%");
	CHECK(lua.run("return os.date('!100%', 0)") == "100%");
	// conversions the Microsoft C library doesn't know end the program there: refused before
	CHECK(contains(lua.run("return os.date('%Q')"), "os.date: unknown conversion '%Q'"));
	CHECK(contains(lua.run("return os.date('%#c')"), "os.date: unknown conversion '%#'"));
	CHECK(contains(lua.run("return os.date('%Ey')"), "os.date: unknown conversion '%E'"));
	// the table holds only the clock
	CHECK(lua.run("local n = 0 for _ in pairs(os) do n = n + 1 end return n") == "4");
}

TEST_CASE("lua sandbox: all libraries opened over the sandbox are whole again (lua.fulllibraries)") {
	sandboxed_state const lua;
	luaL_openlibs(lua.get());
	for (auto const *name : {"io.open", "os.execute", "os.getenv", "os.date", "require", "loadstring", "dofile", "string.dump", "debug.traceback"}) {
		CAPTURE(name);
		CHECK(lua.run(std::format("return type({})", name)) == "function");
	}
	CHECK(lua.run("return type(require('ffi'))") == "table");
	CHECK(lua.run("return rawequal(os, package.loaded.os)") == "true");
}

TEST_CASE("lua sandbox: precompiled code isn't loaded") {
	// real LuaJIT bytecode, from a state with all libraries
	auto *full{luaL_newstate()};
	luaL_openlibs(full);
	// luaL_dostring is a || of two calls, a bool in C++
	REQUIRE_FALSE(luaL_dostring(full, "return string.dump(function() return 42 end)"));
	std::size_t length{0};
	auto const *bytes{lua_tolstring(full, -1, &length)};
	std::string const bytecode{bytes, length};
	lua_close(full);
	REQUIRE(false == bytecode.empty());
	CHECK(bytecode.front() == '\x1b');

	sandboxed_state const lua;
	auto const top{lua_gettop(lua.get())};
	CHECK_FALSE(lua_sandbox::load_text_chunk(lua.get(), bytecode, "compiled.lua"));
	CHECK(contains(lua_tostring(lua.get(), -1), "compiled.lua: precompiled scripts aren't loaded"));
	lua_settop(lua.get(), top);
	// nor in a text that only starts with the marker
	CHECK_FALSE(lua_sandbox::load_text_chunk(lua.get(), std::string_view{"\x1bLJ\x02 return 1"}, "fake.lua"));
	lua_settop(lua.get(), top);
}

TEST_CASE("lua sandbox: a scenery reads its scripts only from its own folder") {
	test::scratch_folder const root{"lua_sandbox_paths"};
	root.write("outside.lua", "return 1");
	root.write("scenery/test.lua", "return 2");
	root.write("scenery/scripts/module.lua", "return 3");
	root.write("scenery/folder.lua/inner.lua", "return 4");
	auto const folder{root.path("scenery") + "/"};

	CHECK(lua_sandbox::read_script_file(folder, "test.lua") == std::optional<std::string>{"return 2"});
	CHECK(lua_sandbox::read_script_file(folder, "scripts/module.lua") == std::optional<std::string>{"return 3"});
	// '\' separates the folders on Linux too, as in the rest of the scenery's paths
	CHECK(lua_sandbox::with_forward_slashes("a\\b/c\\d.lua") == "a/b/c/d.lua");
	CHECK(lua_sandbox::read_script_file(folder, "scripts\\module.lua") == std::optional<std::string>{"return 3"});
	for (auto const &path : {std::string{"../outside.lua"}, std::string{"scripts/../../outside.lua"}, std::string{"..\\outside.lua"},
	                         std::string{"scripts\\..\\..\\outside.lua"}, std::string{"scripts/..\\../outside.lua"},
	                         root.path("outside.lua"), std::string{"/outside.lua"}, std::string{"\\outside.lua"}, std::string{"C:outside.lua"},
	                         std::string{"missing.lua"}, std::string{"folder.lua"}, std::string{"scripts"}, std::string{}}) {
		CAPTURE(path);
		CHECK_FALSE(lua_sandbox::read_script_file(folder, path).has_value());
	}
}

TEST_CASE("lua sandbox: a script larger than the limit isn't read") {
	test::scratch_folder const root{"lua_sandbox_size"};
	root.write("scenery/limit.lua", std::string(static_cast<std::size_t>(lua_sandbox::max_script_file), ' '));
	root.write("scenery/over.lua", std::string(static_cast<std::size_t>(lua_sandbox::max_script_file) + 1, ' '));
	auto const folder{root.path("scenery") + "/"};
	CHECK(lua_sandbox::read_script_file(folder, "limit.lua").has_value());
	CHECK_FALSE(lua_sandbox::read_script_file(folder, "over.lua").has_value());
}

TEST_CASE("lua sandbox: a scenery script runs in protected mode, and its errors name the file") {
	test::scratch_folder const root{"lua_sandbox_run"};
	root.write("scenery/good.lua", "ran = (ran or 0) + 1");
	root.write("scenery/fails.lua", "ran = 10\nlocal x = nil\nx.field = 1\nran = 20");
	root.write("scenery/files.lua", "local f = io.open('eu07.ini')");
	root.write("scenery/system.lua", "os.execute('cmd /c echo pwned')");
	root.write("scenery/syntax.lua", "this is not lua");
	root.write("scenery/compiled.lua", std::string_view{"\x1bLJ\x02 bytes"});
	auto const folder{root.path("scenery") + "/"};

	sandboxed_state const lua;
	auto *L{lua.get()};
	auto const top{lua_gettop(L)};

	CHECK_FALSE(lua_sandbox::run_script_file(L, folder, "good.lua").has_value());
	CHECK(lua.run("return ran") == "1");

	auto const fails{lua_sandbox::run_script_file(L, folder, "fails.lua")};
	REQUIRE(fails.has_value());
	CHECK(contains(*fails, "fails.lua:3:"));
	// it stopped at the error
	CHECK(lua.run("return ran") == "10");

	auto const files{lua_sandbox::run_script_file(L, folder, "files.lua")};
	REQUIRE(files.has_value());
	CHECK(contains(*files, "files.lua:1: attempt to index global 'io' (a nil value)"));

	auto const system{lua_sandbox::run_script_file(L, folder, "system.lua")};
	REQUIRE(system.has_value());
	CHECK(contains(*system, "system.lua:1: attempt to call field 'execute' (a nil value)"));

	auto const syntax{lua_sandbox::run_script_file(L, folder, "syntax.lua")};
	REQUIRE(syntax.has_value());
	CHECK(contains(*syntax, "syntax.lua:1:"));

	auto const compiled{lua_sandbox::run_script_file(L, folder, "compiled.lua")};
	REQUIRE(compiled.has_value());
	CHECK(contains(*compiled, "compiled.lua: precompiled scripts aren't loaded"));

	auto const outside{lua_sandbox::run_script_file(L, folder, "../good.lua")};
	REQUIRE(outside.has_value());
	CHECK(contains(*outside, "outside the folder"));

	// nothing left on the stack, and the state still works
	CHECK(lua_gettop(L) == top);
	CHECK_FALSE(lua_sandbox::run_script_file(L, folder, "good.lua").has_value());
	CHECK(lua.run("return ran") == "11");
}

#endif
