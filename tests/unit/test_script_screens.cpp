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
