// Esc in the road window (GH#348): the first one gives up what is going on, the next one goes back to the selection

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorRoad.hpp"

namespace {

constexpr int select_tool{0};
constexpr int build_tool{1};
constexpr int place_tool{2};

} // namespace

TEST_CASE("Esc with the build tool and no road being drawn goes back to the selection") {
	CHECK(editor_road::tool_after_escape(build_tool, false, false) == select_tool);
}

TEST_CASE("Esc while a road is being drawn or led out of a road or a junction keeps the build tool") {
	CHECK(editor_road::tool_after_escape(build_tool, true, false) == build_tool);
}

TEST_CASE("Esc with the place tool goes back to the selection, the tool has nothing being drawn") {
	CHECK(editor_road::tool_after_escape(place_tool, false, false) == select_tool);
}

TEST_CASE("Esc while the selected points are dragged up or down keeps the tool, the drag is given up") {
	CHECK(editor_road::tool_after_escape(select_tool, false, true) == select_tool);
	CHECK(editor_road::tool_after_escape(build_tool, false, true) == build_tool);
}

TEST_CASE("Esc with the selection tool stays with it") {
	CHECK(editor_road::tool_after_escape(select_tool, false, false) == select_tool);
}

TEST_CASE("two Esc presses from the build tool in the middle of a road: the road given up, then the selection") {
	auto tool{build_tool};
	bool drawing{true};
	tool = editor_road::tool_after_escape(tool, drawing, false);
	drawing = false; // what the first Esc gives up
	CHECK(tool == build_tool);
	tool = editor_road::tool_after_escape(tool, drawing, false);
	CHECK(tool == select_tool);
}
