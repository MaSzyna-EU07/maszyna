#include "stdafx.h"
#include "doctest.h"
#include "application/hudlayout.h"

#include <array>

// where the parts of the hud of the driver stand: held at the nearer edges of the screen, snapped to the edges near them, kept
// in hud_layout.ini
namespace
{
using hud_layout::anchor;
using hud_layout::rect;
using hud_layout::size;

size const screen{1920.f, 1080.f};
} // namespace

TEST_CASE("a part is held at the edges of the screen nearer to it")
{
	auto const topleft{hud_layout::anchor_of(rect{16.f, 40.f, 300.f, 100.f}, screen)};
	CHECK_FALSE(topleft.right);
	CHECK(topleft.x == doctest::Approx(16.f));
	CHECK_FALSE(topleft.bottom);
	CHECK(topleft.y == doctest::Approx(40.f));

	auto const bottomright{hud_layout::anchor_of(rect{1600.f, 900.f, 300.f, 100.f}, screen)};
	CHECK(bottomright.right);
	CHECK(bottomright.x == doctest::Approx(20.f));
	CHECK(bottomright.bottom);
	CHECK(bottomright.y == doctest::Approx(80.f));
}

TEST_CASE("a part keeps its corner on a screen of another size")
{
	auto const held{hud_layout::anchor_of(rect{1600.f, 900.f, 300.f, 100.f}, screen)};
	auto const placed{hud_layout::place(held, size{300.f, 100.f}, size{1280.f, 720.f})};
	CHECK(placed.left == doctest::Approx(960.f));
	CHECK(placed.top == doctest::Approx(540.f));
}

TEST_CASE("a part snaps to the edges of the screen and of the other parts")
{
	SUBCASE("to the edges of the screen")
	{
		auto const snapped{hud_layout::snap(rect{6.f, 1072.f - 100.f, 300.f, 100.f}, {}, screen, 12.f)};
		CHECK(snapped.left == doctest::Approx(0.f));
		CHECK(snapped.top == doctest::Approx(980.f));
	}
	SUBCASE("to the side of another part, and in line with it")
	{
		std::array const others{rect{100.f, 200.f, 300.f, 100.f}};
		auto const snapped{hud_layout::snap(rect{408.f, 195.f, 200.f, 80.f}, others, screen, 12.f)};
		CHECK(snapped.left == doctest::Approx(400.f));
		CHECK(snapped.top == doctest::Approx(200.f));
	}
	SUBCASE("not from farther than the range")
	{
		auto const snapped{hud_layout::snap(rect{50.f, 50.f, 300.f, 100.f}, {}, screen, 12.f)};
		CHECK(snapped.left == doctest::Approx(50.f));
		CHECK(snapped.top == doctest::Approx(50.f));
	}
	SUBCASE("held within the screen")
	{
		auto const snapped{hud_layout::snap(rect{1800.f, -40.f, 300.f, 100.f}, {}, screen, 12.f)};
		CHECK(snapped.left == doctest::Approx(1620.f));
		CHECK(snapped.top == doctest::Approx(0.f));
	}
}

TEST_CASE("the layout is written and read back")
{
	hud_layout::layout layout;
	layout.insert_or_assign("aid", anchor{false, 16.f, true, 16.f});
	layout.insert_or_assign("timetable", anchor{true, 20.f, false, 60.f});
	auto const read{hud_layout::parse(hud_layout::write(layout))};
	REQUIRE(read.size() == 2);
	CHECK_FALSE(read.at("aid").right);
	CHECK(read.at("aid").bottom);
	CHECK(read.at("aid").y == doctest::Approx(16.f));
	CHECK(read.at("timetable").right);
	CHECK(read.at("timetable").x == doctest::Approx(20.f));
}

TEST_CASE("lines of hud_layout.ini which aren't places are passed over")
{
	auto const read{hud_layout::parse("// comment\n\naid middle 16 top 4\nscenario left 10\nmap left 5 top 6\n")};
	CHECK(read.size() == 1);
	CHECK(read.contains("map"));
}
