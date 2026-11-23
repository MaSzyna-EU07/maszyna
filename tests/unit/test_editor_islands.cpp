#include "stdafx.h"
#include "doctest.h"
#include "editor/editorIslands.hpp"

#include <array>
#include <utility>

namespace {

using editor_islands::rect;

editor_islands::input window(float const Width, float const Height) {
	editor_islands::input in;
	in.work_min = {0.0f, 20.0f}; // under the menu
	in.work_max = {Width, 20.0f + Height};
	in.capsule_width = 600.0f;
	return in;
}

bool inside(rect const &Inner, rect const &Outer) {
	// an island not shown has no area, wherever it is
	if (Inner.size().x <= 0.0f || Inner.size().y <= 0.0f)
		return true;
	return Inner.min.x >= Outer.min.x && Inner.min.y >= Outer.min.y && Inner.max.x <= Outer.max.x && Inner.max.y <= Outer.max.y;
}

bool overlap(rect const &A, rect const &B) {
	// rectangles without an area overlap nothing
	if (A.size().x <= 0.0f || A.size().y <= 0.0f || B.size().x <= 0.0f || B.size().y <= 0.0f)
		return false;
	return A.min.x < B.max.x && B.min.x < A.max.x && A.min.y < B.max.y && B.min.y < A.max.y;
}

bool well_formed(rect const &R) {
	return R.max.x >= R.min.x && R.max.y >= R.min.y;
}

// the islands the layout gives a size to, which have to stay apart
std::array<std::pair<char const *, rect>, 7> sized(editor_islands::layout const &Layout) {
	return {{{"capsule", Layout.capsule}, {"rail", Layout.rail}, {"tool", Layout.tool}, {"inspector", Layout.inspector}, {"drawer", Layout.drawer}, {"bank", Layout.bank}, {"sets", Layout.sets}}};
}

void check_apart(editor_islands::input const &In) {
	auto const layout{editor_islands::compute(In)};
	rect const work{In.work_min, In.work_max};
	auto const islands{sized(layout)};
	for (std::size_t a = 0; a < islands.size(); ++a) {
		CAPTURE(islands[a].first);
		CHECK(well_formed(islands[a].second));
		CHECK(inside(islands[a].second, work));
		for (std::size_t b = a + 1; b < islands.size(); ++b) {
			CAPTURE(islands[b].first);
			CHECK_FALSE(overlap(islands[a].second, islands[b].second));
		}
	}
	CHECK(well_formed(layout.view));
	CHECK(inside(layout.view, work));
}

} // namespace

TEST_CASE("editor islands: a full hd window keeps the sizes of the design") {
	auto in{window(1920.0f, 1060.0f)};
	auto const layout{editor_islands::compute(in)};
	CHECK(layout.capsule.min == glm::vec2{12.0f, 32.0f});
	CHECK(layout.rail.min == glm::vec2{12.0f, 32.0f + 52.0f + 10.0f});
	CHECK(layout.tool.size().x == doctest::Approx(340.0f));
	CHECK(layout.tool.min == glm::vec2{1920.0f - 12.0f - 340.0f, 32.0f});
	CHECK(layout.tool.size().y == doctest::Approx(300.0f)); // before its content was measured
	CHECK(layout.inspector.min.y == doctest::Approx(layout.tool.max.y + 10.0f));
	CHECK(layout.inspector.max.y == doctest::Approx(1080.0f - 12.0f));
	CHECK(layout.drawer.size().x == doctest::Approx(300.0f));
	CHECK_FALSE(layout.column_below_capsule);
	CHECK_FALSE(layout.tray_full_width);
	// the view between the rail and the column, down to the bottom margin
	CHECK(layout.view.min == glm::vec2{12.0f + 48.0f + 10.0f, 32.0f + 52.0f + 10.0f});
	CHECK(layout.view.max == glm::vec2{layout.tool.min.x - 10.0f, 1080.0f - 12.0f});
	check_apart(in);
}

TEST_CASE("editor islands: the tray takes the bottom of the view, the model sets share it") {
	auto in{window(1920.0f, 1060.0f)};
	in.tray = true;
	in.sets = true;
	auto const layout{editor_islands::compute(in)};
	CHECK(layout.bank.size().y == doctest::Approx(250.0f));
	CHECK(layout.bank.max.y == doctest::Approx(1080.0f - 12.0f));
	CHECK(layout.sets.min.x == doctest::Approx(layout.bank.max.x + 10.0f));
	CHECK(layout.sets.max.x == doctest::Approx(layout.tool.min.x - 10.0f));
	CHECK(layout.view.max.y == doctest::Approx(layout.bank.min.y - 10.0f));
	CHECK(layout.drawer.max.y <= layout.bank.min.y - 10.0f);
	check_apart(in);
}

TEST_CASE("editor islands: the tool is as high as its content, within its share of the column") {
	auto in{window(1920.0f, 1060.0f)};
	in.tool_content = 180.0f;
	CHECK(editor_islands::compute(in).tool.size().y == doctest::Approx(180.0f));
	in.tool_content = 40.0f;
	CHECK(editor_islands::compute(in).tool.size().y == doctest::Approx(140.0f));
	in.tool_content = 5000.0f;
	auto const layout{editor_islands::compute(in)};
	CHECK(layout.tool.size().y == doctest::Approx((1080.0f - 32.0f - 12.0f) * 0.6f));
	CHECK(layout.inspector.size().y >= 120.0f);
}

TEST_CASE("editor islands: a capsule too wide for the column at its side puts the column under it") {
	auto in{window(1366.0f, 748.0f)};
	in.scale = 1.25f;
	in.capsule_width = 1100.0f;
	auto const layout{editor_islands::compute(in)};
	CHECK(layout.column_below_capsule);
	CHECK(layout.tool.min.y >= layout.capsule.max.y);
	check_apart(in);
}

TEST_CASE("editor islands: in a narrow window the tray goes under the column, which stops above it") {
	auto in{window(480.0f, 600.0f)};
	in.tray = true;
	auto const layout{editor_islands::compute(in)};
	CHECK(layout.tray_full_width);
	CHECK(layout.bank.max.x == doctest::Approx(480.0f - 12.0f));
	CHECK(layout.inspector.max.y <= layout.bank.min.y - 10.0f);
	check_apart(in);
}

TEST_CASE("editor islands: the islands stay apart and in the window at any size and scale") {
	for (auto const scale : {1.0f, 1.5f, 2.0f})
		for (auto const width : {320.0f, 640.0f, 1024.0f, 1366.0f, 2560.0f})
			for (auto const height : {200.0f, 480.0f, 768.0f, 1440.0f})
				for (auto const tray : {false, true})
					for (auto const sets : {false, true}) {
						CAPTURE(scale);
						CAPTURE(width);
						CAPTURE(height);
						CAPTURE(tray);
						CAPTURE(sets);
						auto in{window(width, height)};
						in.scale = scale;
						in.capsule_width = 500.0f * scale;
						in.tool_content = 260.0f * scale;
						in.tray = tray;
						in.sets = tray && sets;
						check_apart(in);
					}
}

TEST_CASE("editor islands: a minimised window leaves every island and the view empty, not turned inside out") {
	auto in{window(0.0f, 0.0f)};
	in.tray = true;
	in.sets = true;
	auto const layout{editor_islands::compute(in)};
	for (auto const &[name, island] : sized(layout)) {
		CAPTURE(name);
		CHECK(well_formed(island));
		CHECK(island.size().x * island.size().y == doctest::Approx(0.0f));
	}
	CHECK(well_formed(layout.view));
}
