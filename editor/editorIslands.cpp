/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorIslands.hpp"

#include <algorithm>
#include <cmath>

namespace editor_islands
{

namespace
{

// the sizes at the scale 1, in pixels
constexpr float margin{12.0f}; // from the edges of the window
constexpr float gap{10.0f}; // between two islands
constexpr float column{340.0f}; // the tool and the selection on the right
constexpr float column_least{220.0f}; // narrower only when the window is
constexpr float column_share{0.4f}; // of the width of the window at most, above its least width
constexpr float capsule_height{52.0f};
constexpr float rail_width{48.0f};
constexpr float rail_height{128.0f}; // three buttons of the rail with its padding, the window sizes itself by them
constexpr float drawer_width{300.0f};
constexpr float drawer_most{560.0f};
constexpr float tray_height{250.0f};
constexpr float tray_share{0.4f}; // of the height of the window at most
constexpr float tray_least_width{200.0f}; // narrower between the rail and the column, the tray goes under the column
constexpr float bank_share{0.58f}; // of the tray, when the model sets share it
constexpr float tool_least{140.0f};
constexpr float tool_first{300.0f}; // before its content was measured
constexpr float tool_share{0.6f}; // of the height of the column at most
constexpr float inspector_least{120.0f};

rect make(glm::vec2 const Min, glm::vec2 const Size)
{
	return {Min, Min + glm::max(Size, glm::vec2{0.0f})};
}

// the edges of the space the islands are laid in: the window less its margin, never turned inside out
struct frame
{
	float s;
	float gap;
	float left;
	float top;
	float right;
	float bottom;

	explicit frame(input const &In)
	    : s{In.scale}, gap{editor_islands::gap * In.scale}, left{In.work_min.x + margin * In.scale}, top{In.work_min.y + margin * In.scale}, right{std::max(left, In.work_max.x - margin * In.scale)},
	      bottom{std::max(top, In.work_max.y - margin * In.scale)}
	{
	}
	float width() const
	{
		return right - left;
	}
	float height() const
	{
		return bottom - top;
	}
};

// never over the rail, the column gives way to it in a window as narrow as that
float column_width(frame const &F, float const Besiderail)
{
	auto const wanted{std::min(column * F.s, std::max(column_least * F.s, F.width() * column_share))};
	return std::max(0.0f, std::min(wanted, F.right - Besiderail));
}

// the tool on top of the column, as high as its content up to a share of the column; the selection under it has the rest
void lay_column(layout &Layout, input const &In, frame const &F, glm::vec2 const Top, float const Width, float const Bottom)
{
	auto const height{std::max(0.0f, Bottom - Top.y)};
	auto const least{std::min(tool_least * F.s, height)};
	auto const most{std::max(least, std::min(height * tool_share, height - F.gap - inspector_least * F.s))};
	auto const wanted{In.tool_content > 0.0f ? In.tool_content : tool_first * F.s};
	auto const tool{std::clamp(wanted, least, most)};
	Layout.tool = make(Top, {Width, tool});
	auto const inspector{Top.y + tool + F.gap};
	Layout.inspector = make({Top.x, std::min(inspector, Bottom)}, {Width, Bottom - inspector});
}

// the node bank, and the model sets by its side when they are shown; never up into the capsule, Ceiling is under it
void lay_tray(layout &Layout, input const &In, frame const &F, glm::vec2 const Span, float const Ceiling)
{
	auto const left{Span.x};
	auto const right{Span.y};
	auto const height{std::max(0.0f, std::min({tray_height * F.s, F.height() * tray_share, F.bottom - Ceiling}))};
	auto const top{F.bottom - height};
	auto const width{std::max(0.0f, right - left)};
	if (false == In.sets)
	{
		Layout.bank = make({left, top}, {width, height});
		return;
	}
	auto const bank{std::floor(std::max(0.0f, width - F.gap) * bank_share)};
	Layout.bank = make({left, top}, {bank, height});
	auto const sets{left + bank + F.gap};
	Layout.sets = make({std::min(sets, right), top}, {right - sets, height});
}

} // namespace

layout compute(input const &In)
{
	frame const f{In};
	layout result;
	// the capsule at the top left, the rail under it
	auto const capsule{std::min(capsule_height * f.s, f.height())};
	result.capsule = make({f.left, f.top}, {std::min(In.capsule_width, f.width()), capsule});
	auto const belowcapsule{std::min(f.top + capsule + f.gap, f.bottom)};
	result.rail = make({f.left, belowcapsule}, {std::min(rail_width * f.s, f.width()), std::min(rail_height * f.s, f.bottom - belowcapsule)});
	auto const besiderail{std::min(result.rail.max.x + f.gap, f.right)};
	// the column on the right, under the capsule if the capsule would reach into it
	auto const width{column_width(f, besiderail)};
	auto const columnleft{f.right - width};
	result.column_below_capsule = result.capsule.max.x + f.gap > columnleft;
	auto const columntop{result.column_below_capsule ? belowcapsule : f.top};
	// the tray between the rail and the column; where that's too narrow, under the column too, the column stopping above it
	auto const betweenright{std::max(besiderail, columnleft - f.gap)};
	result.tray_full_width = In.tray && betweenright - besiderail < tray_least_width * f.s;
	auto columnbottom{f.bottom};
	auto freebottom{f.bottom};
	if (In.tray)
	{
		lay_tray(result, In, f, {besiderail, result.tray_full_width ? f.right : betweenright}, belowcapsule);
		freebottom = std::max(belowcapsule, result.bank.min.y - f.gap);
		if (result.tray_full_width)
			columnbottom = std::max(columntop, freebottom);
	}
	lay_column(result, In, f, {columnleft, columntop}, width, columnbottom);
	// the scene or the layers as a drawer by the rail, down to the tray, never into the column
	auto const drawer{std::min(drawer_width * f.s, betweenright - besiderail)};
	auto const drawerheight{std::min(drawer_most * f.s, freebottom - belowcapsule)};
	result.drawer = make({besiderail, belowcapsule}, {drawer, drawerheight});
	// the 3d view between the rail, the capsule, the column and the tray
	result.view = make({besiderail, belowcapsule}, {betweenright - besiderail, freebottom - belowcapsule});
	return result;
}

} // namespace editor_islands
