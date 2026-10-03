/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "world/Road.h"
#include "simulation/simulation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{

// the same helper the track tools use to draw over the viewport
class screen_projection
{
  public:
	screen_projection()
	{
		ImGuiIO const &io = ImGui::GetIO();
		m_size = io.DisplaySize;
		m_camera = GfxRenderer->Camera_Position();
		float const aspect = m_size.y > 0.0f ? m_size.x / m_size.y : 1.0f;
		m_viewprojection = editor_mode::projection_matrix(aspect) * GfxRenderer->Camera_View_Matrix();
	}

	glm::vec4 clip(glm::dvec3 const &Point) const
	{
		return m_viewprojection * glm::vec4(glm::vec3(Point - m_camera), 1.0f);
	}
	ImVec2 screen(glm::vec4 const &Clip) const
	{
		return ImVec2((Clip.x / Clip.w * 0.5f + 0.5f) * m_size.x, (0.5f - Clip.y / Clip.w * 0.5f) * m_size.y);
	}
	bool project(glm::dvec3 const &Point, ImVec2 &Screen) const
	{
		auto const c = clip(Point);
		if (c.w < kNear)
			return false;
		Screen = screen(c);
		return true;
	}
	void line(ImDrawList *Drawlist, glm::dvec3 const &A, glm::dvec3 const &B, ImU32 const Color, float const Thickness) const
	{
		glm::vec4 a = clip(A);
		glm::vec4 b = clip(B);
		if (a.w < kNear && b.w < kNear)
			return;
		if (a.w < kNear)
			a = glm::mix(a, b, (kNear - a.w) / (b.w - a.w));
		else if (b.w < kNear)
			b = glm::mix(b, a, (kNear - b.w) / (a.w - b.w));
		Drawlist->AddLine(screen(a), screen(b), Color, Thickness);
	}

  private:
	static constexpr float kNear{0.1f};
	ImVec2 m_size;
	glm::dvec3 m_camera;
	glm::mat4 m_viewprojection;
};

glm::dvec3 path_point(segment_data const &Path, double const T)
{
	auto const &p0 = Path.points[segment_data::point::start];
	auto const &p3 = Path.points[segment_data::point::end];
	auto const &cp1 = Path.points[segment_data::point::control1];
	auto const &cp2 = Path.points[segment_data::point::control2];
	if (cp1 == glm::dvec3{0.0} && cp2 == glm::dvec3{0.0})
		return glm::mix(p0, p3, T);
	auto const p1 = p0 + cp1;
	auto const p2 = p3 + cp2;
	double const u = 1.0 - T;
	return u * u * u * p0 + 3.0 * u * u * T * p1 + 3.0 * u * T * T * p2 + T * T * T * p3;
}

bool is_curved(segment_data const &Path)
{
	return Path.points[segment_data::point::control1] != glm::dvec3{0.0} || Path.points[segment_data::point::control2] != glm::dvec3{0.0};
}

void copy_text(char *Buffer, std::size_t const Size, std::string const &Text)
{
	std::strncpy(Buffer, Text.c_str(), Size - 1);
	Buffer[Size - 1] = 0;
}

std::string lower_case(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return Text;
}

double const kSnapRadius{4.0}; // an end of a road this close to the cursor is what the click is taken to mean
double const kReach{3000.0}; // clicks landing further than this are taken for misses
double const kLaneRange{600.0}; // lanes are drawn for the roads this close to the camera
double const kLongestStretch{2000.0}; // a single click doesn't lead a road further than this
// lengths of the pieces a road following the ground is tried with, from the longest; the first one to fit the ground is taken
double const kProfileSteps[]{50.0, 25.0, 12.5};
double const kSunkTolerance{0.05}; // how far under the height it's built at the road can get between the points it's led through
double const kRaisedTolerance{0.5}; // and how far over it

ImU32 const kForwardColor{IM_COL32(70, 230, 110, 230)};
ImU32 const kBackwardColor{IM_COL32(255, 170, 50, 230)};
ImU32 const kTurnColor{IM_COL32(210, 210, 210, 150)};
ImU32 const kWayColor{IM_COL32(120, 200, 255, 170)};
ImU32 const kSelectedColor{IM_COL32(40, 220, 255, 230)};
ImU32 const kPreviewColor{IM_COL32(255, 210, 60, 255)};
ImU32 const kRefusedColor{IM_COL32(240, 60, 60, 255)};
ImU32 const kSnapColor{IM_COL32(255, 60, 255, 255)};

}

void editor_mode::render_road_menu()
{
	if (ImGui::BeginMenu("Roads"))
	{
		ImGui::MenuItem("Road editor", nullptr, &m_roadtool.window);
		ImGui::MenuItem("Show lanes", nullptr, &m_roadtool.lanes);
		ImGui::EndMenu();
	}
}

void editor_mode::update_road_tool()
{
	auto &tool{m_roadtool};
	if (tool.selected != nullptr && tool.selected->m_editorremoved)
		tool.selected = nullptr;
	if (tool.junction != nullptr && tool.junction->m_editorremoved)
		tool.junction = nullptr;
	tool.preview.clear();
	tool.hasjunction = false;
	tool.previewsnap = {};
	tool.previewerror.clear();
	if (false == tool.window)
	{
		tool.chain = false;
		return;
	}
	glm::dvec3 const camera{Global.pCamera.Pos};
	tool.mouse = camera + GfxRenderer->Mouse_Position();
	if (glm::distance(tool.mouse, camera) > kReach)
	{
		tool.previewerror = "The cursor isn't over anything near enough";
		return;
	}
	// what a click would make at the moment, to be drawn over the view
	if (tool.tool == 1 && tool.chain)
		tool.preview = road_preview(tool.previewsnap, tool.previewerror, false);
	else if (tool.tool == 2)
		tool.hasjunction = junction_preview(tool.previewjunction, tool.previewsnap, tool.previewerror);
}

void editor_mode::road_select(road_node *Road)
{
	auto &tool{m_roadtool};
	tool.selected = Road;
	if (Road != nullptr)
	{
		tool.junction = nullptr;
		tool.settings = Road->definition();
		std::string reason;
		tool.status = "Selected: " + (Road->name().empty() ? std::string{"(unnamed road)"} : Road->name());
		if (false == editor_road::can_edit(*Road, &reason))
			tool.status += ". It can't be changed: " + reason;
	}
	copy_text(tool.surface, sizeof(tool.surface), tool.settings.surface);
	for (int side = 0; side < 2; ++side)
		copy_text(tool.sides[side], sizeof(tool.sides[side]), tool.settings.sides[side].material);
}

void editor_mode::junction_select(junction_node *Junction)
{
	auto &tool{m_roadtool};
	tool.junction = Junction;
	if (Junction == nullptr)
		return;
	tool.selected = nullptr;
	std::string reason;
	tool.status = "Selected: " + Junction->name();
	if (false == editor_road::can_edit(*Junction, &reason))
		tool.status += ". It can't be removed: " + reason;
}

void editor_mode::road_cancel()
{
	m_roadtool.chain = false;
}

std::vector<segment_data> editor_mode::road_preview(editor_road::loose_end &Snap, std::string &Error, bool const Fresh)
{
	auto const &tool{m_roadtool};
	Snap = editor_road::find_end(tool.mouse, kSnapRadius);
	// the end the road is being drawn from isn't somewhere to lead it to
	if (Snap.valid() && glm::distance(Snap.position, tool.point) < 0.5)
		Snap = {};
	glm::dvec2 enddirection{0.0, 1.0};
	if (Snap.valid())
	{
		glm::dvec2 const outwards{Snap.outwards.x, Snap.outwards.z};
		if (glm::length(outwards) > 1e-6)
			enddirection = -glm::normalize(outwards);
		else
			Snap = {};
	}
	// in the open the road is put over what's under the cursor
	auto const target{Snap.valid() ? Snap.position : tool.mouse + glm::dvec3{0.0, tool.offset, 0.0}};
	if (glm::distance(tool.point, target) > kLongestStretch)
	{
		Error = "Too far from the last point";
		return {};
	}
	if (Snap.junction != nullptr)
	{
		// the junction has its ways through made for a set of lanes at each arm
		auto const arriving{tool.reversed ? tool.settings.backward : tool.settings.forward};
		auto const leaving{tool.reversed ? tool.settings.forward : tool.settings.backward};
		if (arriving != Snap.backward || leaving != Snap.forward || std::abs(tool.settings.lanewidth - Snap.width) > 0.01f)
		{
			char width[32];
			std::snprintf(width, sizeof(width), "%.2f", Snap.width);
			Error = "This arm of the junction takes a road with " + std::to_string(Snap.backward) + " lane(s) leading into it and " + std::to_string(Snap.forward) + " out of it, " + width + " m wide each";
			return {};
		}
	}
	auto pieces{editor_road::plan(tool.point, tool.hasdirection ? &tool.direction : nullptr, target, Snap.valid() ? &enddirection : nullptr, Global.ctrlState, Error)};
	road_profile(pieces, Snap, Fresh);
	return pieces;
}

// gives the pieces laid out by the planner their heights: over the ground if the road is to follow it, with the slopes
// of the pieces matched where they meet, and with what the road is attached to
void editor_mode::road_profile(std::vector<segment_data> &Pieces, editor_road::loose_end const &Snap, bool const Fresh)
{
	auto const &tool{m_roadtool};
	if (Pieces.empty())
		return;
	// an end of a road or an arm of a junction is reached with the slope it has
	double endgrade{0.0};
	if (Snap.valid())
	{
		auto const run{glm::length(glm::dvec2{Snap.outwards.x, Snap.outwards.z})};
		endgrade = (run > 1e-6 ? -Snap.outwards.y / run : 0.0);
	}
	auto const *firstgrade{tool.hasgrade ? &tool.grade : nullptr};
	auto const *lastgrade{Snap.valid() ? &endgrade : nullptr};
	if (false == tool.follow)
	{
		// straight from a point to the next one, the way a ramp or a viaduct goes
		std::vector<double> heights;
		for (auto const &piece : Pieces)
			heights.emplace_back(piece.points[segment_data::point::start].y);
		heights.emplace_back(Pieces.back().points[segment_data::point::end].y);
		editor_road::profile(Pieces, heights, firstgrade, lastgrade);
		return;
	}
	auto const planned{Pieces};
	bool fresh{Fresh};
	for (auto const step : kProfileSteps)
	{
		Pieces = editor_road::refine(planned, step);
		auto const count{Pieces.size()};
		// the ground is looked up where the pieces meet, and in the middle of each to see how the result fits it
		std::vector<glm::dvec3> points;
		points.reserve(2 * count + 1);
		for (auto const &piece : Pieces)
			points.emplace_back(piece.points[segment_data::point::start]);
		points.emplace_back(Pieces.back().points[segment_data::point::end]);
		for (auto const &piece : Pieces)
			points.emplace_back(path_point(piece, 0.5));
		// the planner leads the road between its ends, which are over the ground already
		for (auto &point : points)
			point.y -= tool.offset;
		auto ground{ground_heights(points, fresh)};
		fresh = false;
		for (auto &height : ground)
			height += tool.offset;
		std::vector<double> heights(ground.begin(), ground.begin() + count + 1);
		// the ends are where they were put
		heights.front() = Pieces.front().points[segment_data::point::start].y;
		heights.back() = Pieces.back().points[segment_data::point::end].y;
		editor_road::profile(Pieces, heights, firstgrade, lastgrade);
		// shorter pieces are called for if the ground shows through the road, or the road hangs over it.
		// the pieces attached to something are let off, as they have to get to its height one way or another
		bool fits{true};
		for (std::size_t i = 0; i < count; ++i)
		{
			if ((i == 0 && firstgrade != nullptr) || (i + 1 == count && lastgrade != nullptr))
				continue;
			auto const over{path_point(Pieces[i], 0.5).y - ground[count + 1 + i]};
			if (over < -kSunkTolerance || over > kRaisedTolerance)
			{
				fits = false;
				break;
			}
		}
		if (fits)
			break;
	}
}

// lays out the junction a click would make. returns: false if there's none to make, with the reason in Error unless
// it's a matter of the first click being still ahead
bool editor_mode::junction_preview(junction_node::state &State, editor_road::loose_end &Snap, std::string &Error) const
{
	auto const &tool{m_roadtool};
	auto const kind{static_cast<editor_road::junction_kind>(std::clamp(tool.kind, 0, 3))};
	Snap = {};
	junction_node::state state;
	// the look goes after the road the junction is made for
	auto const dress = [&state](road_node::state const &Layout) {
		state.surface = Layout.surface;
		state.texturelength = Layout.texturelength;
		state.markings = Layout.markings;
		state.friction = Layout.friction;
		state.sounddistance = Layout.sounddistance;
		state.quality = Layout.quality;
		state.environment = Layout.environment;
	};
	if (false == tool.chain)
	{
		// at a loose end of a road the junction is made for that road, and put right past its end
		Snap = editor_road::find_end(tool.mouse, kSnapRadius, false);
		if (Snap.road == nullptr)
			return false;
		auto const &layout{Snap.road->definition()};
		glm::dvec2 const outwards{Snap.outwards.x, Snap.outwards.z};
		if (glm::length(outwards) < 1e-6 || layout.lanes.empty())
		{
			Error = "There's no telling which way this road goes at its end";
			return false;
		}
		// the ways through the junction start and end where the lanes of the road are expected, which takes lanes of the same width
		auto const width{layout.lanes.front().width};
		for (auto const &lane : layout.lanes)
		{
			if (std::abs(lane.width - width) > 0.01f)
			{
				Error = "Lanes of this road differ in width, and a junction takes roads with lanes of the same width";
				return false;
			}
		}
		state = editor_road::junction(kind, Snap.position, glm::normalize(outwards), Snap.atend ? layout.forward : layout.backward, Snap.atend ? layout.backward : layout.forward, width);
		dress(layout);
	}
	else
	{
		// in the open the centre is set already, and the cursor shows where the first road comes from
		glm::dvec2 const away{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
		if (glm::length(away) < 1.0)
		{
			Error = "Move the cursor away from the centre to turn the junction";
			return false;
		}
		auto const heading{-glm::normalize(away)};
		auto const &layout{tool.settings};
		auto const reach{editor_road::junction_reach(layout.forward + layout.backward, layout.lanewidth)};
		glm::dvec3 const entry{tool.point.x - heading.x * reach, tool.point.y, tool.point.z - heading.y * reach};
		state = editor_road::junction(kind, entry, heading, layout.forward, layout.backward, layout.lanewidth);
		dress(layout);
	}
	state.velocity = tool.crossingspeed;
	state.normalize();
	State = state;
	return true;
}

void editor_mode::junction_click()
{
	auto &tool{m_roadtool};
	junction_node::state state;
	editor_road::loose_end snap;
	std::string error;
	if (false == junction_preview(state, snap, error))
	{
		if (false == error.empty())
		{
			tool.status = "Can't make the junction: " + error;
		}
		else if (false == tool.chain)
		{
			// away from the roads the first click sets the centre, the second one turns the junction
			tool.point = tool.mouse + glm::dvec3{0.0, tool.offset, 0.0};
			tool.chain = true;
			tool.status = "Centre set";
		}
		return;
	}
	std::string reason;
	if (snap.valid() && false == editor_road::can_join(snap, &reason))
	{
		tool.status = "Can't make the junction here: " + reason;
		return;
	}
	auto const created{editor_road::create(std::vector<junction_node::state>{state})};
	if (created.empty())
	{
		tool.status = "The junction wasn't created";
		return;
	}
	push_road_snapshot({}, {}, {}, created, {});
	tool.chain = false;
	tool.status = "\"" + created.front()->name() + "\" made" + (snap.road != nullptr ? " at the end of \"" + snap.road->name() + "\"" : std::string{}) + ". Lead roads out of its arms with the Build tool";
}

void editor_mode::road_click()
{
	auto &tool{m_roadtool};
	glm::dvec3 const camera{Global.pCamera.Pos};
	glm::dvec3 const ground{camera + GfxRenderer->Mouse_Position()};
	if (glm::distance(ground, camera) > kReach)
	{
		tool.status = "The click didn't land on anything near enough";
		return;
	}
	tool.mouse = ground;
	if (tool.tool == 0)
	{
		auto *road{editor_road::nearest(ground, 1.5)};
		auto *junction{road == nullptr ? editor_road::nearest_junction(ground) : nullptr};
		if (road == nullptr && junction == nullptr)
			tool.status = "No road here";
		road_select(road);
		junction_select(junction);
		return;
	}

	std::string reason;
	if (false == editor_road::available(&reason))
	{
		tool.status = reason;
		return;
	}
	if (tool.tool == 2)
	{
		junction_click();
		return;
	}
	if (false == tool.chain)
	{
		// first click sets where the road starts. started from a loose end of a road it carries that road on,
		// started from an arm of a junction it's the road the junction is made for there
		auto const end{editor_road::find_end(ground, kSnapRadius)};
		glm::dvec2 const outwards{end.outwards.x, end.outwards.z};
		if (end.valid() && glm::length(outwards) > 1e-6)
		{
			if (false == editor_road::can_join(end, &reason))
			{
				tool.status = "Can't build from here: " + reason;
				return;
			}
			tool.point = end.position;
			tool.direction = glm::normalize(outwards);
			tool.hasdirection = true;
			tool.grade = end.outwards.y / glm::length(outwards);
			tool.hasgrade = true;
			tool.junction = nullptr;
			if (end.road != nullptr)
			{
				// carried on from its start the road keeps the direction of its axis, so the lanes stay what they were
				tool.reversed = (false == end.atend);
				tool.settings = end.road->definition();
				tool.status = "Carrying on \"" + end.road->name() + "\" with its layout";
			}
			else
			{
				tool.reversed = false;
				tool.settings.forward = end.forward;
				tool.settings.backward = end.backward;
				tool.settings.lanewidth = end.width;
				tool.settings.lanes.clear();
				tool.settings.changes.clear();
				tool.settings.normalize();
				tool.status = "Leading a road out of \"" + end.junction->name() + "\", with the lanes the junction has there";
			}
			road_select(nullptr);
		}
		else
		{
			tool.point = ground + glm::dvec3{0.0, tool.offset, 0.0};
			tool.hasdirection = false;
			tool.hasgrade = false;
			tool.reversed = false;
			tool.selected = nullptr;
			tool.junction = nullptr;
			tool.status = "Start set";
		}
		tool.chain = true;
		return;
	}

	editor_road::loose_end snap;
	std::string error;
	auto pieces{road_preview(snap, error, true)};
	if (pieces.empty())
	{
		tool.status = "Can't build this: " + error;
		return;
	}
	if (snap.valid() && false == editor_road::can_join(snap, &reason))
	{
		tool.status = "Can't join here: " + reason;
		return;
	}
	// where the next piece is going to carry on from
	auto const &last{pieces.back()};
	auto const endpoint{last.points[segment_data::point::end]};
	auto const tangent{last.points[segment_data::point::control2] != glm::dvec3{0.0} ? -last.points[segment_data::point::control2] : last.points[segment_data::point::end] - last.points[segment_data::point::start]};
	auto const grade{editor_road::end_grade(last)};
	if (tool.reversed)
	{
		for (auto &piece : pieces)
		{
			std::swap(piece.points[segment_data::point::start], piece.points[segment_data::point::end]);
			std::swap(piece.points[segment_data::point::control1], piece.points[segment_data::point::control2]);
		}
		std::reverse(pieces.begin(), pieces.end());
	}
	std::vector<road_node::state> states;
	for (auto const &piece : pieces)
	{
		auto state{tool.settings};
		state.axis = piece;
		states.emplace_back(state);
	}
	auto const created{editor_road::create(states)};
	if (created.empty())
	{
		tool.status = "The road wasn't created";
		return;
	}
	push_road_snapshot({}, created, {});
	if (snap.valid())
	{
		tool.chain = false;
		tool.status = "Joined with \"" + (snap.road != nullptr ? snap.road->name() : snap.junction->name()) + "\"";
		return;
	}
	glm::dvec2 const planar{tangent.x, tangent.z};
	tool.point = endpoint;
	if (glm::length(planar) > 1e-6)
	{
		tool.direction = glm::normalize(planar);
		tool.hasdirection = true;
	}
	tool.grade = grade;
	tool.hasgrade = true;
	tool.status = std::to_string(created.size()) + (created.size() == 1 ? " piece added" : " pieces added");
}

void editor_mode::road_apply()
{
	auto &tool{m_roadtool};
	if (tool.selected == nullptr)
		return;
	tool.settings.normalize();
	auto const pieces{tool.whole ? editor_road::chain(*tool.selected) : std::vector<road_node *>{tool.selected}};
	std::vector<std::pair<road_node *, road_node::state>> before;
	std::vector<std::pair<road_node *, road_node::state>> changes;
	for (auto *road : pieces)
	{
		std::string reason;
		if (false == editor_road::can_edit(*road, &reason))
		{
			tool.status = "\"" + road->name() + "\" can't be changed: " + reason;
			// what the window shows goes back to what the road is like
			road_select(tool.selected);
			tool.status = "\"" + road->name() + "\" can't be changed: " + reason;
			return;
		}
		auto state{tool.settings};
		state.axis = road->definition().axis;
		if (road != tool.selected && road->definition().changes.size() == state.changes.size())
		{
			// where the lanes can be changed is set for each piece on its own
			state.changes = road->definition().changes;
		}
		before.emplace_back(road, road->definition());
		changes.emplace_back(road, state);
	}
	editor_road::apply(changes);
	push_road_snapshot(before, {}, {});
	tool.settings = tool.selected->definition();
	tool.status = (pieces.size() == 1 ? std::string{"Changed"} : "Changed " + std::to_string(pieces.size()) + " pieces");
}

bool editor_mode::road_delete()
{
	auto &tool{m_roadtool};
	if (false == tool.window || (tool.selected == nullptr && tool.junction == nullptr))
		return false;
	std::string reason;
	if (tool.junction != nullptr)
	{
		auto *junction{tool.junction};
		if (false == editor_road::can_edit(*junction, &reason))
		{
			tool.status = "Can't delete: " + reason;
			return true;
		}
		editor_road::remove(std::vector<junction_node *>{junction});
		push_road_snapshot({}, {}, {}, {}, {junction});
		tool.junction = nullptr;
		tool.status = "\"" + junction->name() + "\" deleted, Ctrl+Z brings it back";
		return true;
	}
	auto *road{tool.selected};
	if (false == editor_road::can_edit(*road, &reason))
	{
		tool.status = "Can't delete: " + reason;
		return true;
	}
	editor_road::remove(std::vector<road_node *>{road});
	push_road_snapshot({}, {}, {road});
	tool.selected = nullptr;
	tool.status = "\"" + road->name() + "\" deleted, Ctrl+Z brings it back";
	return true;
}

bool editor_mode::road_split()
{
	auto &tool{m_roadtool};
	if (false == tool.window || tool.selected == nullptr)
		return false;
	auto *road{tool.selected};
	std::string reason;
	if (false == editor_road::can_edit(*road, &reason))
	{
		tool.status = "Can't split: " + reason;
		return true;
	}
	auto const before{road->definition()};
	auto *created{editor_road::split(*road, editor_road::nearest_parameter(before, tool.mouse))};
	if (created == nullptr)
	{
		tool.status = "Can't split this close to the end";
		return true;
	}
	push_road_snapshot({{road, before}}, {created}, {});
	road_select(road);
	tool.status = "Split in two: \"" + road->name() + "\" and \"" + created->name() + "\"";
	return true;
}

void editor_mode::push_road_snapshot(std::vector<std::pair<road_node *, road_node::state>> States, std::vector<road_node *> Created, std::vector<road_node *> Removed, std::vector<junction_node *> Junctionscreated,
                                     std::vector<junction_node *> Junctionsremoved)
{
	if (States.empty() && Created.empty() && Removed.empty() && Junctionscreated.empty() && Junctionsremoved.empty())
		return;

	if (m_max_history_size >= 0 && (int)m_history.size() >= m_max_history_size)
		m_history.erase(m_history.begin(), m_history.begin() + ((int)m_history.size() - m_max_history_size + 1));

	// the entry is listed under the name of the first thing it's about
	scene::basic_node const *node{nullptr};
	if (false == States.empty())
		node = States.front().first;
	else if (false == Created.empty())
		node = Created.front();
	else if (false == Removed.empty())
		node = Removed.front();
	else if (false == Junctionscreated.empty())
		node = Junctionscreated.front();
	else
		node = Junctionsremoved.front();
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::RoadEdit;
	snap.node_name = node->name();
	snap.position = node->location();
	snap.roads = std::move(States);
	snap.roads_created = std::move(Created);
	snap.roads_removed = std::move(Removed);
	snap.junctions_created = std::move(Junctionscreated);
	snap.junctions_removed = std::move(Junctionsremoved);
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::restore_road_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo)
{
	if (Snapshot.roads.empty() && Snapshot.roads_created.empty() && Snapshot.roads_removed.empty() && Snapshot.junctions_created.empty() && Snapshot.junctions_removed.empty())
		return;

	// what the roads are like now is what the opposite operation brings back
	EditorSnapshot current{Snapshot};
	current.roads.clear();
	for (auto const &entry : Snapshot.roads)
		current.roads.emplace_back(entry.first, entry.first->definition());
	Opposite.push_back(std::move(current));

	if (Undo)
	{
		editor_road::remove(Snapshot.roads_created);
		editor_road::remove(Snapshot.junctions_created);
		editor_road::revive(Snapshot.junctions_removed);
		editor_road::revive(Snapshot.roads_removed);
	}
	else
	{
		editor_road::revive(Snapshot.roads_created);
		editor_road::revive(Snapshot.junctions_created);
		editor_road::remove(Snapshot.junctions_removed);
		editor_road::remove(Snapshot.roads_removed);
	}
	editor_road::apply(Snapshot.roads);

	auto &tool{m_roadtool};
	tool.chain = false;
	if (tool.selected != nullptr)
		road_select(tool.selected->m_editorremoved ? nullptr : tool.selected);
	if (tool.junction != nullptr && tool.junction->m_editorremoved)
		tool.junction = nullptr;
}

// draws controls for the layout of a road. returns: true if the layout was changed in a way which calls for the road to be made anew
bool editor_mode::render_road_layout(road_node::state &State)
{
	auto &tool{m_roadtool};
	bool changed{false};
	// typed values are taken when confirmed, so the road isn't made anew with each digit
	auto const number = [](char const *Label, float &Value, float const Step, char const *Format) {
		ImGui::SetNextItemWidth(120.0f);
		return ImGui::InputFloat(Label, &Value, Step, Step * 4.0f, Format, ImGuiInputTextFlags_EnterReturnsTrue);
	};
	auto const material = [&tool](char const *Label, char *Buffer, std::size_t const Size, std::string &Value) {
		bool picked{false};
		ImGui::PushID(Label);
		ImGui::SetNextItemWidth(200.0f);
		if (ImGui::InputText("##name", Buffer, Size, ImGuiInputTextFlags_EnterReturnsTrue) || ImGui::IsItemDeactivatedAfterEdit())
		{
			auto const name{lower_case(Buffer)};
			if (name != Value && name.find(' ') == std::string::npos)
			{
				Value = name.empty() ? std::string{"none"} : name;
				picked = true;
			}
			copy_text(Buffer, Size, Value);
		}
		ImGui::SameLine();
		if (ImGui::Button("..."))
			ImGui::OpenPopup("pick");
		ImGui::SameLine();
		ImGui::TextUnformatted(Label);
		if (ImGui::BeginPopup("pick"))
		{
			ImGui::TextDisabled("Materials and images found in the texture folder");
			ImGui::SetNextItemWidth(200.0f);
			ImGui::InputText("Filter", tool.filter, sizeof(tool.filter));
			auto const filter{lower_case(tool.filter)};
			std::vector<std::string const *> listed;
			for (auto const &name : editor_road::materials())
				if (filter.empty() || name.find(filter) != std::string::npos)
					listed.emplace_back(&name);
			ImGui::BeginChild("list", ImVec2(340.0f, 280.0f), true);
			ImGuiListClipper clipper(static_cast<int>(listed.size()));
			while (clipper.Step())
			{
				for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i)
				{
					if (ImGui::Selectable(listed[i]->c_str(), *listed[i] == Value))
					{
						Value = *listed[i];
						copy_text(Buffer, Size, Value);
						picked = true;
						ImGui::CloseCurrentPopup();
					}
				}
			}
			ImGui::EndChild();
			ImGui::EndPopup();
		}
		ImGui::PopID();
		return picked;
	};

	// lanes
	ImGui::SetNextItemWidth(120.0f);
	if (ImGui::InputInt("Lanes along the axis", &State.forward))
		changed = true;
	ImGui::SetNextItemWidth(120.0f);
	if (ImGui::InputInt("Lanes against the axis", &State.backward))
		changed = true;
	if (changed)
	{
		// a different set of lanes starts with the common width and speed limit, and the default rules for changing lanes
		State.lanes.clear();
		State.changes.clear();
		State.normalize();
	}
	if (number("Lane width [m]", State.lanewidth, 0.25f, "%.2f"))
	{
		State.lanewidth = std::clamp(State.lanewidth, 1.0f, 10.0f);
		for (auto &lane : State.lanes)
			lane.width = State.lanewidth;
		changed = true;
	}
	float limit{State.velocity > 0.0f ? State.velocity : 0.0f};
	if (number("Speed limit [km/h], 0: none", limit, 10.0f, "%.0f"))
	{
		State.velocity = (limit > 0.0f ? limit : -1.0f);
		for (auto &lane : State.lanes)
			lane.velocity = State.velocity;
		changed = true;
	}

	// appearance
	ImGui::Separator();
	char const *colours[]{"none", "white", "orange"};
	int colour{State.markings == road_node::marking_colour::white ? 1 : State.markings == road_node::marking_colour::orange ? 2 : 0};
	ImGui::SetNextItemWidth(120.0f);
	if (ImGui::Combo("Markings", &colour, colours, 3))
	{
		State.markings = (colour == 1 ? road_node::marking_colour::white : colour == 2 ? road_node::marking_colour::orange : road_node::marking_colour::none);
		changed = true;
	}
	if (material("Surface", tool.surface, sizeof(tool.surface), State.surface))
		changed = true;
	if (number("Texture length [m]", State.texturelength, 0.5f, "%.2f"))
	{
		State.texturelength = std::max(0.1f, State.texturelength);
		changed = true;
	}
	char const *sidenames[]{"Left side", "Right side"};
	char const *sidetypes[]{"none", "shoulder", "sidewalk"};
	for (int side = 0; side < 2; ++side)
	{
		ImGui::PushID(side);
		auto &data{State.sides[side]};
		int type{data.type == road_node::side_type::shoulder ? 1 : data.type == road_node::side_type::sidewalk ? 2 : 0};
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::Combo(sidenames[side], &type, sidetypes, 3))
		{
			data.type = (type == 1 ? road_node::side_type::shoulder : type == 2 ? road_node::side_type::sidewalk : road_node::side_type::none);
			if (data.type != road_node::side_type::none && data.width <= 0.0f)
				data.width = 1.5f;
			changed = true;
		}
		if (data.type != road_node::side_type::none)
		{
			ImGui::Indent();
			if (number("Width [m]", data.width, 0.25f, "%.2f"))
			{
				data.width = std::clamp(data.width, 0.1f, 20.0f);
				changed = true;
			}
			if (material("Material", tool.sides[side], sizeof(tool.sides[side]), data.material))
				changed = true;
			if (data.material.empty() || data.material == "none")
				ImGui::TextDisabled("Pick a material to have the side drawn");
			ImGui::Unindent();
		}
		ImGui::PopID();
	}
	if (State.sides[0].type == road_node::side_type::sidewalk || State.sides[1].type == road_node::side_type::sidewalk)
	{
		if (number("Kerb height [m]", State.kerbheight, 0.01f, "%.2f"))
		{
			State.kerbheight = std::clamp(State.kerbheight, 0.0f, 1.0f);
			changed = true;
		}
	}
	if (State.sides[0].type == road_node::side_type::shoulder || State.sides[1].type == road_node::side_type::shoulder)
	{
		if (number("Bank width [m]", State.slope.x, 0.25f, "%.2f"))
		{
			State.slope.x = std::clamp(State.slope.x, 0.0f, 20.0f);
			changed = true;
		}
		if (number("Bank drop [m]", State.slope.y, 0.1f, "%.2f"))
		{
			State.slope.y = std::clamp(State.slope.y, 0.0f, 20.0f);
			changed = true;
		}
	}

	// single lanes, and the rules for changing between them
	if (ImGui::CollapsingHeader("Lanes and lane changes", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::TextDisabled("Left to right when facing along the axis. f: along the axis, b: against it");
		for (std::size_t lane = 0; lane < State.lanes.size(); ++lane)
		{
			ImGui::PushID(static_cast<int>(lane));
			auto &data{State.lanes[lane]};
			auto const id{State.lane_id(lane)};
			ImGui::TextColored(lane >= static_cast<std::size_t>(State.backward) ? ImVec4(0.3f, 0.9f, 0.45f, 1.0f) : ImVec4(1.0f, 0.67f, 0.2f, 1.0f), "%-3s", id.c_str());
			ImGui::SameLine();
			if (number("m##width", data.width, 0.25f, "%.2f"))
			{
				data.width = std::clamp(data.width, 1.0f, 10.0f);
				changed = true;
			}
			ImGui::SameLine();
			float lanelimit{data.velocity > 0.0f ? data.velocity : 0.0f};
			if (number("km/h##velocity", lanelimit, 10.0f, "%.0f"))
			{
				data.velocity = (lanelimit > 0.0f ? lanelimit : -1.0f);
				changed = true;
			}
			if (lane + 1 < State.lanes.size() && lane < State.changes.size())
			{
				// the entries are in the order of the stored values: none, to the right, to the left, both
				auto const next{State.lane_id(lane + 1)};
				auto const toright{"only " + id + " to " + next};
				auto const toleft{"only " + next + " to " + id};
				char const *modes[]{"no lane change", toright.c_str(), toleft.c_str(), "lane change both ways"};
				ImGui::Indent();
				ImGui::SetNextItemWidth(200.0f);
				if (ImGui::Combo("##change", &State.changes[lane], modes, 4))
					changed = true;
				ImGui::Unindent();
			}
			ImGui::PopID();
		}
	}
	return changed;
}

void editor_mode::render_road_window()
{
	auto &tool{m_roadtool};
	if (false == tool.window)
		return;
	ImGui::SetNextWindowSize(ImVec2(440.0f, 620.0f), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin("Roads###roadeditor", &tool.window))
	{
		ImGui::End();
		return;
	}
	std::string reason;
	if (false == editor_road::available(&reason))
		ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", reason.c_str());

	auto const previoustool{tool.tool};
	ImGui::RadioButton("Select", &tool.tool, 0);
	ImGui::SameLine();
	ImGui::RadioButton("Build", &tool.tool, 1);
	ImGui::SameLine();
	ImGui::RadioButton("Junction", &tool.tool, 2);
	if (tool.tool != previoustool)
	{
		tool.chain = false;
		if (tool.tool != 0)
		{
			tool.selected = nullptr;
			tool.junction = nullptr;
		}
	}
	ImGui::SameLine();
	ImGui::Checkbox("Show lanes", &tool.lanes);

	// how the things being built are put on the ground
	auto const placement = [&tool](bool const Road) {
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::InputFloat("Height above the ground [m]", &tool.offset, 0.05f, 0.25f, "%.2f"))
			tool.offset = std::clamp(tool.offset, -10.0f, 100.0f);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "How far over the ground under the cursor the points are put.\nA few centimetres keep the surface from flickering where it lies on the ground.");
		if (false == Road)
			return;
		ImGui::Checkbox("Follow the ground", &tool.follow);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "On: the road is led over the ground between the clicked points, in pieces as short as the ground calls for,\nwith the profile rounded so the slope doesn't change abruptly.\nOff: the road goes straight from a point to the next one, like a ramp or a viaduct.");
	};
	if (tool.tool == 1)
	{
		ImGui::TextDisabled("LMB: start, then each next point. Ctrl: straight ahead. Esc: finish");
		ImGui::TextDisabled("Start or end on a loose end of a road or an arm of a junction to join it");
		if (tool.chain)
		{
			if (ImGui::Button("Finish (Esc)"))
				tool.chain = false;
		}
		placement(true);
		ImGui::Separator();
		// what's set here is what the next pieces get
		render_road_layout(tool.settings);
	}
	else if (tool.tool == 2)
	{
		ImGui::TextDisabled("LMB on a loose end of a road: junction for that road, right past its end");
		ImGui::TextDisabled("LMB elsewhere: the centre, then the side the first road comes from. Esc: start over");
		char const *kinds[]{"Crossroads", "T: the road ends, roads to both sides", "T: the road goes on, a road to the left", "T: the road goes on, a road to the right"};
		ImGui::SetNextItemWidth(280.0f);
		ImGui::Combo("Kind", &tool.kind, kinds, 4);
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::InputFloat("Speed limit on the junction [km/h]", &tool.crossingspeed, 5.0f, 10.0f, "%.0f"))
			tool.crossingspeed = std::clamp(tool.crossingspeed, 5.0f, 200.0f);
		placement(false);
		ImGui::Separator();
		ImGui::TextWrapped("Away from the roads the junction is made for roads with the lanes set in the Build tool: %d + %d, %.2f m wide. Its surface and markings go after the road it's made for.", tool.settings.forward, tool.settings.backward,
		                   tool.settings.lanewidth);
		ImGui::TextWrapped("Roads are led out of the arms with the Build tool, and get the lanes the junction has there.");
	}
	else if (tool.junction != nullptr)
	{
		auto const &junction{*tool.junction};
		ImGui::Text("%s, %d arms, %d ways through", junction.name().c_str(), static_cast<int>(junction.definition().arms.size()), static_cast<int>(junction.movements().size()));
		ImGui::TextDisabled("Speed limit %.0f km/h. The ways through are made for the roads attached at the moment", junction.definition().velocity);
		if (ImGui::Button("Delete (Del)"))
			road_delete();
	}
	else if (tool.selected != nullptr)
	{
		auto const &road{*tool.selected};
		ImGui::Text("%s, %.1f m, %d + %d lanes", road.name().c_str(), road.definition().length(), road.definition().forward, road.definition().backward);
		if (ImGui::Button("Delete (Del)"))
			road_delete();
		ImGui::SameLine();
		ImGui::TextDisabled("K: split at the cursor");
		ImGui::Checkbox("Changes go to the whole road", &tool.whole);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "A road is a string of pieces. With this on a change of the layout is made to all of them,\nexcept for the lane change rules, which are always set for the selected piece alone.\nSplit a piece (K) to get a zone with rules of its own.");
		ImGui::Separator();
		if (tool.selected != nullptr && render_road_layout(tool.settings))
			road_apply();
	}
	else
	{
		ImGui::TextDisabled("LMB: select a road piece or a junction");
	}
	if (false == tool.status.empty())
	{
		ImGui::Separator();
		ImGui::TextWrapped("%s", tool.status.c_str());
	}
	ImGui::End();
}

void editor_mode::draw_road_overlay() const
{
	auto const &tool{m_roadtool};
	if (false == tool.lanes && false == tool.window)
		return;

	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList();
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto const drawpath = [&](segment_data const &Path, ImU32 const Color, float const Thickness, int const Samples) {
		auto previous{path_point(Path, 0.0)};
		for (int i = 1; i <= Samples; ++i)
		{
			auto const next{path_point(Path, static_cast<double>(i) / Samples)};
			projection.line(drawlist, previous, next, Color, Thickness);
			previous = next;
		}
	};

	// lanes, as the vehicles see them: a line for each, with an arrow pointing where the traffic goes
	if (tool.lanes)
	{
		for (auto const *road : simulation::Roads.sequence())
		{
			if (road == nullptr || road->m_editorremoved)
				continue;
			auto const range{glm::distance(road->location(), camera)};
			if (range > kLaneRange + 0.5 * road->definition().length())
				continue;
			auto const samples{is_curved(road->definition().axis) ? (range < 150.0 ? 12 : 6) : 1};
			auto const against{static_cast<std::size_t>(road->definition().backward)};
			auto const &tracks{road->tracks()};
			for (std::size_t lane = 0; lane < tracks.size(); ++lane)
			{
				if (tracks[lane] == nullptr || tracks[lane]->m_paths.empty())
					continue;
				auto const &path{tracks[lane]->m_paths.front()};
				auto const color{lane >= against ? kForwardColor : kBackwardColor};
				drawpath(path, color, 2.0f, samples);
				if (range > 300.0)
					continue;
				auto const middle{path_point(path, 0.5)};
				auto direction{path_point(path, 0.52) - middle};
				if (glm::length(direction) < 1e-6)
					continue;
				direction = glm::normalize(direction);
				glm::dvec3 const across{direction.z, 0.0, -direction.x};
				auto const tip{middle + direction * 1.2};
				projection.line(drawlist, tip, middle - direction * 0.6 + across * 0.6, color, 2.0f);
				projection.line(drawlist, tip, middle - direction * 0.6 - across * 0.6, color, 2.0f);
			}
			for (auto const *turn : road->turns())
			{
				if (turn != nullptr && false == turn->m_paths.empty())
					drawpath(turn->m_paths.front(), kTurnColor, 1.5f, 8);
			}
		}
		// ways through the junctions
		for (auto const *junction : simulation::Junctions.sequence())
		{
			if (junction == nullptr || junction->m_editorremoved || glm::distance(junction->location(), camera) > kLaneRange)
				continue;
			for (auto const &way : junction->movements())
				drawpath(way, kWayColor, 1.5f, 8);
		}
	}
	if (false == tool.window)
		return;

	// outline of a road piece: its axis and the edges of its surface
	auto const drawroad = [&](road_node::state const &State, ImU32 const Color) {
		auto const samples{is_curved(State.axis) ? 12 : 1};
		auto const halfwidth{0.5 * State.width()};
		drawpath(State.axis, Color, 3.0f, samples);
		glm::dvec3 previous[2];
		for (int i = 0; i <= samples; ++i)
		{
			auto const t{static_cast<double>(i) / samples};
			auto const position{path_point(State.axis, t)};
			auto const tangent{State.tangent(t)};
			glm::dvec3 across{tangent.z, 0.0, -tangent.x};
			if (glm::length(across) > 1e-6)
				across = glm::normalize(across);
			glm::dvec3 const edges[2]{position + across * halfwidth, position - across * halfwidth};
			if (i > 0)
			{
				projection.line(drawlist, previous[0], edges[0], Color, 1.5f);
				projection.line(drawlist, previous[1], edges[1], Color, 1.5f);
			}
			previous[0] = edges[0];
			previous[1] = edges[1];
		}
	};
	auto const drawpoint = [&](glm::dvec3 const &Point, ImU32 const Color, float const Radius, bool const Filled) {
		ImVec2 screen;
		if (false == projection.project(Point, screen))
			return;
		if (Filled)
			drawlist->AddCircleFilled(screen, Radius, Color);
		else
			drawlist->AddCircle(screen, Radius, Color, 20, 3.0f);
	};

	// outline of a junction: the edge of its surface, with the ends of the roads it's made for
	auto const drawjunction = [&](junction_node::state const &State, ImU32 const Color) {
		auto const outline{State.outline()};
		for (std::size_t i = 0; i < outline.size(); ++i)
			projection.line(drawlist, outline[i], outline[(i + 1) % outline.size()], Color, 2.0f);
		for (auto const &arm : State.arms)
			drawpoint(arm.position, Color, 5.0f, true);
	};

	if (tool.selected != nullptr && false == tool.selected->m_editorremoved)
	{
		auto const &state{tool.selected->definition()};
		drawroad(state, kSelectedColor);
		drawpoint(state.axis.points[segment_data::point::start], kSelectedColor, 6.0f, true);
		drawpoint(state.axis.points[segment_data::point::end], kSelectedColor, 6.0f, true);
	}
	if (tool.junction != nullptr && false == tool.junction->m_editorremoved)
		drawjunction(tool.junction->definition(), kSelectedColor);

	std::string hint;
	if (tool.tool == 1)
	{
		if (tool.chain)
		{
			drawpoint(tool.point, kPreviewColor, 6.0f, true);
			if (tool.preview.empty())
			{
				projection.line(drawlist, tool.point, tool.mouse, kRefusedColor, 2.0f);
				hint = tool.previewerror + "   ";
			}
			for (auto const &piece : tool.preview)
			{
				auto state{tool.settings};
				state.axis = piece;
				drawroad(state, kPreviewColor);
			}
			if (tool.previewsnap.valid())
				drawpoint(tool.previewsnap.position, kSnapColor, 14.0f, false);
			hint += "LMB: build up to the cursor   Ctrl: straight ahead   Esc: finish   Ctrl+Z: undo the last piece";
		}
		else
		{
			auto const end{editor_road::find_end(tool.mouse, kSnapRadius)};
			if (end.valid())
				drawpoint(end.position, kSnapColor, 14.0f, false);
			hint = "LMB: set where the road starts; on a loose end of a road or an arm of a junction (ring) to carry on from there";
		}
	}
	else if (tool.tool == 2)
	{
		if (tool.chain)
			drawpoint(tool.point, kPreviewColor, 6.0f, true);
		if (tool.hasjunction)
			drawjunction(tool.previewjunction, kPreviewColor);
		else if (false == tool.previewerror.empty())
			hint = tool.previewerror + "   ";
		if (tool.previewsnap.valid())
			drawpoint(tool.previewsnap.position, kSnapColor, 14.0f, false);
		hint += (tool.chain ? "LMB: make the junction, with the first road coming from the side of the cursor   Esc: start over" :
		         tool.previewsnap.valid() ? "LMB: make the junction at the end of this road" :
		                                    "LMB: on a loose end of a road (ring) to make its junction there; elsewhere to set the centre of one");
	}
	else
	{
		hint = tool.selected != nullptr ? "LMB: select a road piece or a junction   K: split at the cursor   Del: delete   Ctrl+Z: undo" :
		       tool.junction != nullptr ? "LMB: select a road piece or a junction   Del: delete   Ctrl+Z: undo" :
		                                  "LMB: select a road piece or a junction";
	}
	// shown above the place the track tools put their hints at
	ImGuiIO const &io = ImGui::GetIO();
	ImVec2 const position{12.0f, io.DisplaySize.y - 56.0f};
	auto const size{ImGui::CalcTextSize(hint.c_str())};
	drawlist->AddRectFilled(ImVec2(position.x - 6.0f, position.y - 4.0f), ImVec2(position.x + size.x + 6.0f, position.y + size.y + 4.0f), IM_COL32(0, 0, 0, 150), 4.0f);
	drawlist->AddText(position, IM_COL32(255, 255, 255, 230), hint.c_str());
}
