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

#include <filesystem>
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

TEST_CASE("lua sandbox: eu07.read_file reads a file of the scenery folder, and nothing outside it") {
	test::scratch_folder const root{"lua_sandbox_read"};
	root.write("outside.txt", "secret");
	root.write("scenery/data.txt", "line 1\nline 2");
	root.write("scenery/sub/more.txt", "more");
	root.write("scenery/folder/inner.txt", "inner");
	auto const folder{std::filesystem::path{root.path("scenery")}};

	CHECK(lua_sandbox::read_file(folder, "data.txt", 100) == std::optional<std::string>{"line 1\nline 2"});
	CHECK(lua_sandbox::read_file(folder, "sub/more.txt", 100) == std::optional<std::string>{"more"});
	CHECK(lua_sandbox::read_file(folder, "sub\\more.txt", 100) == std::optional<std::string>{"more"});
	// the limit is the caller's
	CHECK(lua_sandbox::read_file(folder, "sub/more.txt", 4).has_value());
	CHECK_FALSE(lua_sandbox::read_file(folder, "sub/more.txt", 3).has_value());
	for (auto const &path : {std::string{"../outside.txt"}, std::string{"sub/../../outside.txt"}, std::string{"..\\outside.txt"},
	                         std::string{"sub\\..\\..\\outside.txt"}, root.path("outside.txt"), std::string{"/outside.txt"},
	                         std::string{"\\outside.txt"}, std::string{"C:outside.txt"}, std::string{"missing.txt"},
	                         std::string{"folder"}, std::string{}, std::string{"data.txt\0x", 10}}) {
		CAPTURE(path);
		CHECK_FALSE(lua_sandbox::read_file(folder, path, 100).has_value());
	}
}

TEST_CASE("lua sandbox: the paths a script may write to") {
	for (auto const *path : {"state.txt", "saves/state.txt", "saves\\state.txt", "a/b/c/d/e/f/g/h/file.txt", "con_log.txt", "comx.txt",
	                         "com10.txt", "nul_state", ".hidden", "stan poci\xc4\x85gu.txt", "a..b"}) {
		CAPTURE(path);
		CHECK(lua_sandbox::is_safe_write_path(path));
	}
	for (auto const &path : {std::string{}, std::string{".."}, std::string{"../x.txt"}, std::string{"a/../../x.txt"}, std::string{"..\\x.txt"},
	                         std::string{"/x.txt"}, std::string{"\\x.txt"}, std::string{"C:x.txt"}, std::string{"C:\\x.txt"},
	                         std::string{"x.txt:stream"}, std::string{"a//b.txt"}, std::string{"a/"}, std::string{"./x.txt"},
	                         std::string{"a/./x.txt"}, std::string{"con"}, std::string{"CON.txt"}, std::string{"saves/nul.dat"},
	                         std::string{"Com1"}, std::string{"lpt9.log"}, std::string{"aux"}, std::string{"prn.x.y"},
	                         std::string{"conin$"}, std::string{"CONOUT$.txt"}, std::string{"con .txt"}, std::string{"a<b"},
	                         std::string{"a>b"}, std::string{"a\"b"}, std::string{"a|b"}, std::string{"a?b"}, std::string{"a*b"},
	                         std::string{"x."}, std::string{"x "}, std::string{"dir./x"}, std::string{"a\tb"}, std::string{"a\nb"},
	                         std::string{"a\0b", 3}, std::string{"a/b/c/d/e/f/g/h/i/file.txt"},
	                         std::string(lua_sandbox::max_data_path + 1, 'x')}) {
		CAPTURE(path);
		CHECK_FALSE(lua_sandbox::is_safe_write_path(path));
	}
	CHECK(lua_sandbox::is_safe_write_path(std::string(lua_sandbox::max_data_path, 'x')));
}

TEST_CASE("lua sandbox: a script writes into its data folder only, and reads back what it wrote") {
	test::scratch_folder const root{"lua_sandbox_write"};
	root.write("outside.txt", "kept");
	auto const data{root.root() / "data" / "td"};

	// the folder is made with the first file
	CHECK_FALSE(lua_sandbox::write_data_file(data, "state.txt", "runs=1").has_value());
	CHECK(lua_sandbox::read_file(data, "state.txt", lua_sandbox::max_data_file) == std::optional<std::string>{"runs=1"});
	// replaced, not appended
	CHECK_FALSE(lua_sandbox::write_data_file(data, "state.txt", "runs=2").has_value());
	CHECK(lua_sandbox::read_file(data, "state.txt", lua_sandbox::max_data_file) == std::optional<std::string>{"runs=2"});
	// any bytes, and the folders of the path
	std::string const bytes{"a\0b\r\n\xff", 6};
	CHECK_FALSE(lua_sandbox::write_data_file(data, "saves\\day 1/bin.dat", bytes).has_value());
	CHECK(lua_sandbox::read_file(data, "saves/day 1/bin.dat", lua_sandbox::max_data_file) == std::optional<std::string>{bytes});
	CHECK_FALSE(lua_sandbox::write_data_file(data, "empty.txt", "").has_value());
	CHECK(lua_sandbox::read_file(data, "empty.txt", lua_sandbox::max_data_file) == std::optional<std::string>{""});

	// nothing outside the folder
	for (auto const *path : {"../outside.txt", "../../outside.txt", "..\\outside.txt", "saves/../../outside.txt", "nul", "con.txt", "x:y"}) {
		CAPTURE(path);
		auto const error{lua_sandbox::write_data_file(data, path, "overwritten")};
		REQUIRE(error.has_value());
		CHECK(contains(*error, "not a path to write to"));
	}
	CHECK(lua_sandbox::write_data_file(data, root.path("outside.txt"), "overwritten").has_value());
	CHECK(lua_sandbox::read_file(root.root(), "outside.txt", 100) == std::optional<std::string>{"kept"});
	// a folder isn't replaced by a file
	auto const folder{lua_sandbox::write_data_file(data, "saves", "x")};
	REQUIRE(folder.has_value());
	CHECK(contains(*folder, "not a file"));
	CHECK(std::filesystem::is_directory(data / "saves"));
	// no folder (the game knows no folder of the player), nothing written
	CHECK(lua_sandbox::write_data_file(std::filesystem::path{}, "state.txt", "x").has_value());
}

TEST_CASE("lua sandbox: the limits of a data folder") {
	test::scratch_folder const root{"lua_sandbox_limits"};
	auto const data{root.root() / "data"};
	std::string const full(static_cast<std::size_t>(lua_sandbox::max_data_file), 'x');

	SUBCASE("a file") {
		CHECK_FALSE(lua_sandbox::write_data_file(data, "full.txt", full).has_value());
		CHECK(lua_sandbox::read_file(data, "full.txt", lua_sandbox::max_data_file).has_value());
		auto const over{lua_sandbox::write_data_file(data, "full.txt", full + "x")};
		REQUIRE(over.has_value());
		CHECK(contains(*over, "larger than"));
		// the file stays as it was
		CHECK(std::filesystem::file_size(data / "full.txt") == lua_sandbox::max_data_file);
	}
	SUBCASE("the bytes of the folder") {
		auto const files{static_cast<int>(lua_sandbox::max_data_folder / lua_sandbox::max_data_file)};
		for (int idx = 0; idx < files; ++idx) {
			REQUIRE_FALSE(lua_sandbox::write_data_file(data, std::format("part{}.dat", idx), full).has_value());
		}
		auto const over{lua_sandbox::write_data_file(data, "one_more.dat", "x")};
		REQUIRE(over.has_value());
		CHECK(contains(*over, "more than"));
		CHECK_FALSE(std::filesystem::exists(data / "one_more.dat"));
		// a file replaced counts with its new size: as large again fits, and a smaller one makes room
		CHECK_FALSE(lua_sandbox::write_data_file(data, "part0.dat", full).has_value());
		CHECK_FALSE(lua_sandbox::write_data_file(data, "part0.dat", "small").has_value());
		CHECK_FALSE(lua_sandbox::write_data_file(data, "one_more.dat", "x").has_value());
	}
	SUBCASE("the files of the folder") {
		for (std::size_t idx = 0; idx < lua_sandbox::max_data_files; ++idx) {
			REQUIRE_FALSE(lua_sandbox::write_data_file(data, std::format("sub{}/file{}.txt", idx % 4, idx), "x").has_value());
		}
		auto const over{lua_sandbox::write_data_file(data, "one_more.txt", "x")};
		REQUIRE(over.has_value());
		CHECK(contains(*over, "files already"));
		CHECK_FALSE(std::filesystem::exists(data / "one_more.txt"));
		// one that is there is still replaced
		CHECK_FALSE(lua_sandbox::write_data_file(data, "sub0/file0.txt", "y").has_value());
	}
}

TEST_CASE("lua sandbox: a link in the data folder doesn't lead a write out of it") {
	test::scratch_folder const root{"lua_sandbox_link"};
	root.write("outside/kept.txt", "kept");
	auto const data{root.root() / "data"};
	std::filesystem::create_directories(data);
	std::error_code error;
	std::filesystem::create_directory_symlink(root.root() / "outside", data / "link", error);
	if (error) {
		// Windows makes links only for an administrator or in developer mode
		MESSAGE("no folder link made (" << error.message() << "), not tested");
		return;
	}
	auto const written{lua_sandbox::write_data_file(data, "link/kept.txt", "overwritten")};
	REQUIRE(written.has_value());
	CHECK(contains(*written, "outside the data folder"));
	CHECK(lua_sandbox::read_file(root.root(), "outside/kept.txt", 100) == std::optional<std::string>{"kept"});
}

TEST_CASE("lua sandbox: the data folder of a scenery") {
	std::filesystem::path const root{"user/scenery_data"};
	CHECK(lua_sandbox::scenery_data_folder(root, "td.scn") == root / "td");
	CHECK(lua_sandbox::scenery_data_folder(root, "Td.SCN") == root / "Td");
	CHECK(lua_sandbox::scenery_data_folder(root, "lines\\l107.scn") == root / "lines/l107");
	CHECK(lua_sandbox::scenery_data_folder(root, "test.scm") == root / "test.scm");
	for (auto const *scenery : {"", ".scn", "../td.scn", "/td.scn", "C:td.scn", "con.scn"}) {
		CAPTURE(scenery);
		CHECK(lua_sandbox::scenery_data_folder(root, scenery).empty());
	}
	CHECK(lua_sandbox::scenery_data_folder(std::filesystem::path{}, "td.scn").empty());
}

TEST_CASE("lua sandbox: the file functions of a scenery script") {
	test::scratch_folder const root{"lua_sandbox_api"};
	root.write("outside.txt", "secret");
	root.write("scenery/settings.txt", "speed=40");
	lua_sandbox::file_folders folders{root.path("scenery") + "/", root.root() / "data" / "td"};

	sandboxed_state const lua;
	// next to eu07.events, as the scenery state has it
	REQUIRE_FALSE(luaL_dostring(lua.get(), "eu07 = { events = { marker = 1 } }"));
	lua_sandbox::open_file_library(lua.get(), &folders);
	auto const top{lua_gettop(lua.get())};

	CHECK(lua.run("return eu07.events.marker") == "1");
	CHECK(lua.run("return eu07.read_file('settings.txt')") == "speed=40");
	CHECK(lua.run("return eu07.read_file('../outside.txt')") == "nil");
	CHECK(lua.run("return eu07.read_file('missing.txt')") == "nil");
	CHECK(lua.run("return eu07.read_data_file('state.txt')") == "nil");
	CHECK(lua.run("return eu07.write_file('state.txt', 'runs=' .. 1)") == "true");
	CHECK(lua.run("return eu07.read_data_file('state.txt')") == "runs=1");
	// the scenery folder and the data folder are apart
	CHECK(lua.run("return eu07.read_file('state.txt')") == "nil");
	CHECK(lua.run("return eu07.read_data_file('../../scenery/settings.txt')") == "nil");
	// a number is written as its text
	CHECK(lua.run("return eu07.write_file('number.txt', 42)") == "true");
	CHECK(lua.run("return eu07.read_data_file('number.txt')") == "42");
	// a refused write gives nil and the reason
	CHECK(lua.run("local ok, reason = eu07.write_file('../outside.txt', 'x') return tostring(ok) .. ' ' .. reason") ==
	      "nil ../outside.txt: not a path to write to (a relative path of portable names, at most 200 bytes and 8 folders deep, without '..')");
	CHECK(lua.run("local ok, reason = eu07.write_file('big.txt', string.rep('x', 1024 * 1024 + 1)) return reason") ==
	      "big.txt: larger than 1048576 bytes");
	CHECK(lua_sandbox::read_file(root.root(), "outside.txt", 100) == std::optional<std::string>{"secret"});
	// the arguments are checked
	CHECK(contains(lua.run("return eu07.write_file('x.txt')"), "bad argument #2"));
	CHECK(contains(lua.run("return eu07.read_file()"), "bad argument #1"));
	CHECK(contains(lua.run("return eu07.write_file({}, 'x')"), "bad argument #1"));

	// the folders are read on every call: without a data folder nothing is written or read there
	folders.data.clear();
	CHECK(lua.run("local ok, reason = eu07.write_file('state.txt', 'x') return tostring(ok) .. ' ' .. reason") ==
	      "nil state.txt: no folder to write to (the game knows no folder of the player)");
	CHECK(lua.run("return eu07.read_data_file('state.txt')") == "nil");
	CHECK(lua_gettop(lua.get()) == top);
}

TEST_CASE("lua sandbox: the file functions make eu07 when there is none") {
	lua_sandbox::file_folders const folders;
	sandboxed_state const lua;
	lua_sandbox::open_file_library(lua.get(), &folders);
	CHECK(lua.run("return type(eu07.read_file) .. type(eu07.write_file) .. type(eu07.read_data_file)") == "functionfunctionfunction");
	CHECK(lua.run("return eu07.read_file('x.txt')") == "nil");
}

#endif
