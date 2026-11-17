#include "stdafx.h"
#include "doctest.h"

#include "scripting/scriptscreenrules.h"
#include "utilities/parser.h"

// the parts of the script screens that don't need the runtime: the .mmd entries, the refresh schedule counted in
// frames and the files a screen may read

using script_screens::frame_schedule;
using script_screens::screen_config;

TEST_CASE("script screens: a cab screen entry with every key") {
	cParser parser("{ ekran.wasm target: screen01 frames: 6 parameters: id=1&mode=full } next:", cParser::buffer_TEXT);
	screen_config config;
	config.deserialize(parser);

	CHECK(config.script == "ekran.wasm");
	CHECK(config.target == "screen01");
	CHECK(config.frames == 6);
	CHECK(config.parameters == "id=1&mode=full");
	// the parser stops at the end of the block
	CHECK(parser.getToken<std::string>() == "next:");
}

TEST_CASE("script screens: the optional keys keep their defaults, an unknown key is skipped with its value") {
	cParser parser("{ tablica.wasm later: 12 instance: name background: tablica_off }", cParser::buffer_TEXT);
	screen_config config;
	config.deserialize(parser);

	CHECK(config.script == "tablica.wasm");
	CHECK(config.target == "none");
	CHECK(config.frames == 0);
	CHECK(config.parameters.empty());
	CHECK(config.instancing == "name");
	CHECK(config.background == "tablica_off");
}

TEST_CASE("script screens: the interval set in eu07.ini, or the screen's own when it is longer") {
	CHECK(frame_schedule::effective_interval(4, 0) == 4);
	CHECK(frame_schedule::effective_interval(4, 10) == 10);
	CHECK(frame_schedule::effective_interval(4, 2) == 4);
	// never less than every frame
	CHECK(frame_schedule::effective_interval(0, 0) == 1);
	CHECK(frame_schedule::effective_interval(-3, -1) == 1);
}

TEST_CASE("script screens: a screen is due once per interval") {
	frame_schedule schedule{100, 3, 0};
	CHECK(schedule.poll(100));
	// asked again in the same frame (the vehicle drawn in more than one pass)
	CHECK_FALSE(schedule.poll(100));
	CHECK_FALSE(schedule.poll(101));
	CHECK_FALSE(schedule.poll(102));
	CHECK(schedule.poll(103));
	CHECK_FALSE(schedule.poll(105));
	CHECK(schedule.poll(106));
}

TEST_CASE("script screens: a screen out of sight for a while is due on the first frame it is asked again") {
	frame_schedule schedule{0, 5, 0};
	CHECK(schedule.poll(0));
	CHECK(schedule.poll(42));
	CHECK_FALSE(schedule.poll(46));
	CHECK(schedule.poll(47));
}

TEST_CASE("script screens: the phase spreads the screens over the frames of an interval") {
	auto const interval{30};
	auto const phase{frame_schedule::phase("make:dynamic/pkp/ed72/tablica.wasm?$timetable=ed72-001", interval)};
	CHECK(phase >= 0);
	CHECK(phase < interval);
	// the same name always gets the same phase
	CHECK(phase == frame_schedule::phase("make:dynamic/pkp/ed72/tablica.wasm?$timetable=ed72-001", interval));
	// a screen refreshed every frame has no phase
	CHECK(frame_schedule::phase("anything", 1) == 0);

	frame_schedule schedule{1000, interval, phase};
	auto first{std::uint64_t{0}};
	for (auto frame = std::uint64_t{1000}; frame < 1000 + interval; ++frame) {
		if (schedule.poll(frame)) {
			first = frame;
			break;
		}
	}
	CHECK(first == 1000 + static_cast<std::uint64_t>(phase));
}

TEST_CASE("script screens: a screen reads files only below its own folder") {
	CHECK(script_screens::is_safe_relative_path("font.png"));
	CHECK(script_screens::is_safe_relative_path("data/font.png"));
	CHECK(script_screens::is_safe_relative_path("data\\font.png"));
	CHECK(script_screens::is_safe_relative_path("data/..font.png"));

	CHECK_FALSE(script_screens::is_safe_relative_path(""));
	CHECK_FALSE(script_screens::is_safe_relative_path("../eu07.ini"));
	CHECK_FALSE(script_screens::is_safe_relative_path("data/../../eu07.ini"));
	CHECK_FALSE(script_screens::is_safe_relative_path("data\\..\\x"));
	CHECK_FALSE(script_screens::is_safe_relative_path(".."));
	CHECK_FALSE(script_screens::is_safe_relative_path("/etc/passwd"));
	CHECK_FALSE(script_screens::is_safe_relative_path("\\windows"));
	CHECK_FALSE(script_screens::is_safe_relative_path("c:/windows/win.ini"));
}

TEST_CASE("script screens: the script of a screen is in the vehicle's folder, or from the game's, and not outside it") {
	using script_screens::screen_script_path;
	std::string const vehicle{"dynamic/pkp/ep07_v1/"};
	CHECK(screen_script_path(vehicle, "screen.lua") == "dynamic/pkp/ep07_v1/screen.lua");
	CHECK(screen_script_path(vehicle, "scripts/screen.lua") == "scripts/screen.lua");
	// a backslash isn't a folder to the game (substr_path), so the name goes in the vehicle's folder
	CHECK(screen_script_path(vehicle, "lua\\screen.lua") == "dynamic/pkp/ep07_v1/lua\\screen.lua");

	CHECK_FALSE(screen_script_path(vehicle, "../../../screen.lua").has_value());
	CHECK_FALSE(screen_script_path(vehicle, "scripts/../../screen.lua").has_value());
	CHECK_FALSE(screen_script_path(vehicle, "..\\..\\screen.lua").has_value());
	CHECK_FALSE(screen_script_path(vehicle, "/screen.lua").has_value());
	CHECK_FALSE(screen_script_path(vehicle, "c:/games/screen.lua").has_value());
	CHECK_FALSE(screen_script_path("../outside/", "screen.lua").has_value());
}

TEST_CASE("script screens: the steps a Lua pattern function may take at worst") {
	using script_screens::lua_pattern_cost;
	using script_screens::max_pattern_cost;
	// a plain search: the subject (and one past it) times the pattern
	CHECK(lua_pattern_cost("abc", 999, true, true) == doctest::Approx(1000.0 * 3));
	CHECK(lua_pattern_cost(".-b", 999, true, true) == doctest::Approx(1000.0 * 3));
	// a pattern without special characters is searched plainly as well
	CHECK(lua_pattern_cost("abc", 999, false, true) == doctest::Approx(1000.0 * 3));
	CHECK(lua_pattern_cost("", 999, false, true) == doctest::Approx(1000.0));
	// from every position, times the subject for each repetition but a last one
	CHECK(lua_pattern_cost(".-b", 999, false, true) == doctest::Approx(1000.0 * 1000 * 3));
	CHECK(lua_pattern_cost("a*", 999, false, true) == doctest::Approx(1000.0 * 2));
	CHECK(lua_pattern_cost("(a+)", 999, false, true) == doctest::Approx(1000.0 * 4));
	CHECK(lua_pattern_cost("a+$", 999, false, true) == doctest::Approx(1000.0 * 1000 * 3));
	CHECK(lua_pattern_cost("(%d+):(%d+)", 999, false, true) == doctest::Approx(1000.0 * 1000 * 11));
	CHECK(lua_pattern_cost("[^\n]+", 999, false, false) == doctest::Approx(1000.0 * 5));
	CHECK(lua_pattern_cost("[%]x]*y", 999, false, true) == doctest::Approx(1000.0 * 1000 * 7));
	CHECK(lua_pattern_cost("%bxyz", 999, false, true) == doctest::Approx(1000.0 * 1000 * 5));
	CHECK(lua_pattern_cost("(a)%1x", 999, false, true) == doctest::Approx(1000.0 * 1000 * 6));
	CHECK(lua_pattern_cost("%f[%w]%w+", 999, false, true) == doctest::Approx(1000.0 * 9));
	// '?' doubles
	CHECK(lua_pattern_cost("a?a?b", 999, false, true) == doctest::Approx(1000.0 * 4 * 5));
	// '^' anchors find, match and gsub to the first position; gmatch takes it as a character
	CHECK(lua_pattern_cost("^.-b", 999, false, true) == doctest::Approx(1000.0 * 4));
	CHECK(lua_pattern_cost("^.-b", 999, false, false) == doctest::Approx(1000.0 * 1000 * 4));
	// a malformed pattern is the library's to refuse
	CHECK(lua_pattern_cost("[a", 999, false, true) > 0.0);
	CHECK(lua_pattern_cost("%", 999, false, true) > 0.0);
	CHECK(lua_pattern_cost("%b", 999, false, true) > 0.0);

	// the review's case, and what a screen does
	CHECK(lua_pattern_cost(".-b", 1'000'000, false, true) > max_pattern_cost);
	CHECK(lua_pattern_cost("[^\n]+", 1'000'000, false, false) < max_pattern_cost);
	CHECK(lua_pattern_cost("^(%w+)=(%d+)$", 1'000, false, true) < max_pattern_cost);
	CHECK(lua_pattern_cost("%s+$", 4'000, false, true) < max_pattern_cost);
	std::string optional;
	for (int idx = 0; idx < 30; ++idx) {
		optional += "a?";
	}
	CHECK(lua_pattern_cost(optional + std::string(30, 'a'), 100, false, true) > max_pattern_cost);
}

TEST_CASE("script screens: a repetition followed by what always matches doesn't multiply") {
	using script_screens::lua_pattern_cost;
	// the first %s* of the trim: past it, (.-)%s*$ matches from wherever it stopped, so it's tried once
	CHECK(lua_pattern_cost("^%s*(.-)%s*$", 999, false, true) == doctest::Approx(1000.0 * 1000 * 12));
	CHECK(lua_pattern_cost("^%s*(.*)", 999, false, true) == doctest::Approx(8.0));
	CHECK(lua_pattern_cost("a*.-$", 999, false, true) == doctest::Approx(1000.0 * 1000 * 5));
	// what may not match still multiplies
	CHECK(lua_pattern_cost("^%s*(.*%S)", 999, false, true) == doctest::Approx(1000.0 * 1000 * 10));
	CHECK(lua_pattern_cost("^%s*x%s*$", 999, false, true) == doctest::Approx(1000.0 * 1000 * 9));
	// '$' is the end only at the end of the pattern
	CHECK(lua_pattern_cost("a*$b", 999, false, true) == doctest::Approx(1000.0 * 1000 * 4));
}

TEST_CASE("script screens: a repetition right before the final $ counts its longest run in the subject") {
	using script_screens::lua_pattern_cost;
	using script_screens::max_pattern_cost;
	std::string words;
	while (words.size() < 10'000) {
		words += "word  next\tone ";
	}
	// a line of 10 KB trimmed: the subject times the longest run of spaces (2), where the length alone gives the
	// subject squared
	CHECK(lua_pattern_cost("^%s*(.-)%s*$", std::string_view{words}, false, true) == doctest::Approx((words.size() + 1.0) * 3 * 12));
	CHECK(lua_pattern_cost("^%s*(.-)%s*$", std::string_view{words}, false, true) < max_pattern_cost);
	CHECK(lua_pattern_cost("^%s*(.-)%s*$", words.size(), false, true) > max_pattern_cost);
	CHECK(lua_pattern_cost("%s+$", std::string_view{words}, false, true) < max_pattern_cost);
	CHECK(lua_pattern_cost("%d*$", std::string_view{words}, false, true) == doctest::Approx((words.size() + 1.0) * 1 * 4));
	CHECK(lua_pattern_cost("x*$", std::string_view{"axxxa"}, false, true) == doctest::Approx(6.0 * 4 * 3));
	CHECK(lua_pattern_cost("%%*$", std::string_view{"a%%%a"}, false, true) == doctest::Approx(6.0 * 4 * 4));
	// a long run is still the subject squared: "x", 10 000 spaces, "x" takes the trim about 10^8 steps, and a
	// megabyte of them hours
	auto const spaces{"x" + std::string(10'000, ' ') + "x"};
	CHECK(lua_pattern_cost("^%s*(.-)%s*$", std::string_view{spaces}, false, true) > max_pattern_cost);
	auto const megabyte{"x" + std::string(1'000'000, ' ') + "x"};
	CHECK(lua_pattern_cost("^%s*(.-)%s*$", std::string_view{megabyte}, false, true) > max_pattern_cost);
	CHECK(lua_pattern_cost("%s*$", std::string_view{megabyte}, false, true) > max_pattern_cost);
	// a set or '.' isn't gone through: the whole subject
	CHECK(lua_pattern_cost("[ ]*$", std::string_view{words}, false, true) == doctest::Approx((words.size() + 1.0) * (words.size() + 1.0) * 5));
	CHECK(lua_pattern_cost(".-b", std::string(1'000'000, 'a'), false, true) > max_pattern_cost);
}
