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
#include "world/RoadPoint.h"
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
	// true while there's room in the draw list for more. the user interface is drawn with 16 bit indices, so a list
	// can't hold more than 65536 vertices; past that the lines come out as triangles strewn all over the screen,
	// which takes the frame rate down with it. the limit doesn't apply if the renderer can deal with larger lists
	bool room(ImDrawList const *Drawlist) const
	{
		return (Drawlist->Flags & ImDrawListFlags_AllowVtxOffset) != 0 || Drawlist->VtxBuffer.Size < limit;
	}
	// true if a line between two points on the screen can't be seen
	bool outside(ImVec2 const &A, ImVec2 const &B) const
	{
		return (A.x < 0.0f && B.x < 0.0f) || (A.y < 0.0f && B.y < 0.0f) || (A.x > m_size.x && B.x > m_size.x) || (A.y > m_size.y && B.y > m_size.y);
	}
	void line(ImDrawList *Drawlist, glm::dvec3 const &A, glm::dvec3 const &B, ImU32 const Color, float const Thickness) const
	{
		if (false == room(Drawlist))
			return;
		glm::vec4 a = clip(A);
		glm::vec4 b = clip(B);
		if (a.w < kNear && b.w < kNear)
			return;
		if (a.w < kNear)
			a = glm::mix(a, b, (kNear - a.w) / (b.w - a.w));
		else if (b.w < kNear)
			b = glm::mix(b, a, (kNear - b.w) / (a.w - b.w));
		auto const from{screen(a)};
		auto const to{screen(b)};
		if (outside(from, to))
			return;
		Drawlist->AddLine(from, to, Color, Thickness);
	}
	// members
	int limit{kVertexLimit}; // vertices the draw list can hold before the drawing stops

	static constexpr int kVertexLimit{56000}; // leaves room for what's drawn after the overlay
	static constexpr int kLaneVertexLimit{36000}; // the lanes give way to the tools, which are drawn after them

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
double const kPointRadius{2.5}; // a point where pieces meet this close to the click is what gets selected, instead of the piece
double const kPointRange{400.0}; // the points are drawn for the roads this close to the camera
double const kSideMargin{1.0}; // how far past the edge of a road the cursor still counts as being on it
double const kMarkerRadius{2.0}; // a level crossing or a traffic point this close to the click is what gets selected
double const kRailReach{40.0}; // rails crossing the road this close to the cursor are taken for a single level crossing
double const kLaneReach{2.5}; // a lane with its middle this close to the cursor is the one a traffic point is put on
double const kFullTurn{6.283185307179586};
double const kJoinClearance{15.0}; // a road isn't led into the side of another this close to the point it's led from
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
ImU32 const kPointColor{IM_COL32(255, 255, 255, 200)};
ImU32 const kSelectedColor{IM_COL32(40, 220, 255, 230)};
ImU32 const kPreviewColor{IM_COL32(255, 210, 60, 255)};
ImU32 const kRefusedColor{IM_COL32(240, 60, 60, 255)};
ImU32 const kSnapColor{IM_COL32(255, 60, 255, 255)};
ImU32 const kCrossingColor{IM_COL32(255, 220, 60, 230)};
ImU32 const kSpawnColor{IM_COL32(90, 230, 120, 230)};
ImU32 const kDespawnColor{IM_COL32(255, 110, 110, 230)};
ImU32 const kCrosswalkColor{IM_COL32(235, 235, 235, 230)};
double const kBankReach{30.0}; // how far from the road the ground is looked for, for a bank to be led to it
double const kBankStep{0.5};

roadpoint_node::kind_type point_kind(int const Index)
{
	return Index == 1 ? roadpoint_node::kind_type::spawn : Index == 2 ? roadpoint_node::kind_type::despawn : Index == 3 ? roadpoint_node::kind_type::crosswalk : roadpoint_node::kind_type::crossing;
}

char const *point_label(roadpoint_node::kind_type const Kind)
{
	return Kind == roadpoint_node::kind_type::spawn     ? "spawn point" :
	       Kind == roadpoint_node::kind_type::despawn   ? "removal point" :
	       Kind == roadpoint_node::kind_type::crosswalk ? "pedestrian crossing" :
	                                                      "level crossing";
}

// gathers new definitions for the pieces which meet or end at provided points.
// Change: receives a definition, which end of it is at the point, and the number of the point
template <class Change_>
std::vector<std::pair<road_node *, road_node::state>> point_changes(std::vector<glm::dvec3> const &Points, Change_ Change)
{
	std::vector<std::pair<road_node *, road_node::state>> changes;
	for (std::size_t index = 0; index < Points.size(); ++index)
	{
		for (auto const &end : editor_road::ends_at(Points[index]))
		{
			// a piece with both its ends among the points gets both changed
			auto entry{std::find_if(changes.begin(), changes.end(), [&end](std::pair<road_node *, road_node::state> const &Listed) { return Listed.first == end.road; })};
			if (entry == changes.end())
			{
				changes.emplace_back(end.road, end.road->definition());
				entry = std::prev(changes.end());
			}
			Change(entry->second, end.atend, index);
		}
	}
	return changes;
}

// direction of a path for specified value of its curve parameter. like path_point() it's cheap enough to be asked for a lot each frame
glm::dvec3 path_direction(segment_data const &Path, double const T)
{
	auto const &p0 = Path.points[segment_data::point::start];
	auto const &p3 = Path.points[segment_data::point::end];
	auto const &cp1 = Path.points[segment_data::point::control1];
	auto const &cp2 = Path.points[segment_data::point::control2];
	if (cp1 == glm::dvec3{0.0} && cp2 == glm::dvec3{0.0})
		return p3 - p0;
	auto const p1 = p0 + cp1;
	auto const p2 = p3 + cp2;
	double const u = 1.0 - T;
	return 3.0 * u * u * (p1 - p0) + 6.0 * u * T * (p2 - p1) + 3.0 * T * T * (p3 - p2);
}

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
	if (tool.marker != nullptr && tool.marker->m_editorremoved)
		tool.marker = nullptr;
	if ((tool.branchroad != nullptr && tool.branchroad->m_editorremoved) || (tool.branchjunction != nullptr && tool.branchjunction->m_editorremoved))
	{
		// what the road was to be led out of is gone
		tool.branchroad = nullptr;
		tool.branchjunction = nullptr;
		tool.chain = false;
	}
	// so are the points whose pieces were removed or changed behind the back of the selection
	tool.points.erase(std::remove_if(tool.points.begin(), tool.points.end(), [](glm::dvec3 const &Point) { return editor_road::ends_at(Point).empty(); }), tool.points.end());
	tool.plan = road_plan{};
	tool.hashover = false;
	if (false == tool.window || tool.tool != 2)
	{
		tool.hastarget = false;
		tool.aimedkind = -1;
	}
	if (false == tool.window)
	{
		tool.chain = false;
		return;
	}
	glm::dvec3 const camera{Global.pCamera.Pos};
	tool.mouse = camera + GfxRenderer->Mouse_Position();
	if (glm::distance(tool.mouse, camera) > kReach)
	{
		tool.plan.error = "The cursor isn't over anything near enough";
		tool.hastarget = false;
		tool.aimedkind = -1;
		return;
	}
	if (tool.tool == 2)
	{
		// the point a click would make at the moment, to be drawn over the view
		roadpoint_aim(false);
		return;
	}
	if (tool.tool != 1)
		return;
	if (tool.ring)
	{
		// a click puts a roundabout around the cursor, there's nothing to lead from a point to another
		tool.chain = false;
		return;
	}
	if (tool.chain)
	{
		// what a click would make at the moment, to be drawn over the view
		tool.plan = road_preview(false);
		return;
	}
	// where a click would lead a road out of the side of a road, or out of a junction
	if (editor_road::find_end(tool.mouse, kSnapRadius).valid())
		return;
	if (auto const *road{editor_road::nearest(tool.mouse, kSideMargin)}; road != nullptr)
	{
		tool.hover = road->definition().point(editor_road::nearest_parameter(road->definition(), tool.mouse));
		tool.hashover = true;
	}
	else if (auto const *junction{editor_road::nearest_junction(tool.mouse)}; junction != nullptr)
	{
		tool.hover = junction->definition().centre;
		tool.hashover = true;
	}
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
	else
	{
		// the layout stays for the build tool, which makes pieces with the same lanes all the way
		tool.settings.missing = {};
	}
	tool.varied = tool.settings.changing();
	copy_text(tool.surface, sizeof(tool.surface), tool.settings.surface);
	for (int side = 0; side < 2; ++side)
		copy_text(tool.sides[side], sizeof(tool.sides[side]), tool.settings.sides[side].material);
	copy_text(tool.kerbtext, sizeof(tool.kerbtext), tool.settings.kerbmaterial);
	copy_text(tool.mediantext, sizeof(tool.mediantext), tool.settings.median.material);
	copy_text(tool.banktext, sizeof(tool.banktext), tool.settings.bankmaterial);
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
		tool.status += ". It can't be changed: " + reason;
	// the fields for the names of materials are shared with the roads
	copy_text(tool.surface, sizeof(tool.surface), Junction->definition().surface);
	copy_text(tool.sides[0], sizeof(tool.sides[0]), Junction->definition().side.material);
	copy_text(tool.kerbtext, sizeof(tool.kerbtext), Junction->definition().kerbmaterial);
	copy_text(tool.banktext, sizeof(tool.banktext), Junction->definition().bankmaterial);
	copy_text(tool.mediantext, sizeof(tool.mediantext), Junction->definition().medianmaterial);
}

void editor_mode::roadpoint_select(roadpoint_node *Point)
{
	auto &tool{m_roadtool};
	tool.marker = Point;
	if (Point == nullptr)
		return;
	std::string reason;
	tool.status = "Selected: " + Point->name();
	if (false == editor_road::can_edit(*Point, &reason))
		tool.status += ". It can't be changed: " + reason;
}

// works out the point a click of the place tool would make with the cursor where it is
void editor_mode::roadpoint_aim(bool const Fresh)
{
	auto &tool{m_roadtool};
	if (false == Fresh && tool.aimedkind == tool.placekind && glm::distance(tool.aimed, tool.mouse) < 0.1)
	{
		// the cursor didn't move, so neither did what it points at
		return;
	}
	tool.aimed = tool.mouse;
	tool.aimedkind = tool.placekind;
	auto const kind{point_kind(tool.placekind)};
	tool.target = tool.placing[tool.placekind];
	tool.target.kind = kind;
	tool.hastarget = false;
	tool.targetnote.clear();
	tool.targetdirection = glm::dvec3{0.0};
	if (kind == roadpoint_node::kind_type::crosswalk)
	{
		auto const *road{editor_road::nearest(tool.mouse, kSideMargin)};
		if (road == nullptr)
		{
			tool.targetnote = "Point at a road";
			return;
		}
		// the stripes go across the road, whichever part of it the cursor is over
		auto const t{editor_road::nearest_parameter(road->definition(), tool.mouse)};
		tool.target.position = road->definition().point(t);
		tool.targetdirection = road->definition().tangent(t);
		tool.hastarget = true;
		return;
	}
	if (kind == roadpoint_node::kind_type::crossing)
	{
		auto const *road{editor_road::nearest(tool.mouse, kSideMargin)};
		if (road == nullptr)
		{
			tool.targetnote = "Point at the place a road crosses the rails";
			return;
		}
		if (road->definition().changing())
		{
			tool.targetnote = "The road gains or loses lanes on this piece; a level crossing goes on a piece which keeps its lanes";
			return;
		}
		glm::dvec3 middle{0.0};
		double halfspan{0.0};
		if (editor_road::rails_across(*road, tool.mouse, kRailReach, middle, halfspan))
		{
			// the crossing goes halfway between the outermost tracks. the distance is rounded up, so it reads well in the scenery file
			tool.target.position = middle;
			tool.target.clearance = static_cast<float>(std::ceil((halfspan + tool.stopmargin) * 2.0) * 0.5);
		}
		else
		{
			tool.target.position = road->definition().point(editor_road::nearest_parameter(road->definition(), tool.mouse));
			tool.target.clearance = std::max(tool.stopmargin, 2.0f);
			tool.targetnote = "No rails cross the road here; a crossing put here won't close until some do";
		}
		tool.hastarget = true;
		return;
	}
	if (false == editor_road::nearest_lane(tool.mouse, kLaneReach, tool.target.position, tool.targetdirection))
	{
		tool.targetnote = "Point at a lane of a road";
		return;
	}
	tool.hastarget = true;
}

// puts down the point the place tool is set for, as a step which can be taken back
void editor_mode::roadpoint_place()
{
	auto &tool{m_roadtool};
	roadpoint_aim(true);
	if (false == tool.hastarget)
	{
		tool.status = tool.targetnote;
		return;
	}
	auto const created{editor_road::create(std::vector<roadpoint_node::state>{tool.target})};
	if (created.empty())
	{
		tool.status = "No scenery is loaded";
		return;
	}
	auto const *point{created.front()};
	auto const &state{point->definition()};
	editor_road::record record;
	record.points_created = created;
	push_road_snapshot(std::move(record));
	char text[64];
	switch (state.kind)
	{
	case roadpoint_node::kind_type::crossing:
	{
		std::snprintf(text, sizeof(text), "%.1f", state.clearance);
		tool.status = "Level crossing made: " + std::to_string(point->stops().size()) + " lanes stop " + text + " m from its middle, ";
		tool.status += std::to_string(point->rails()) + " tracks are watched for trains.";
		break;
	}
	case roadpoint_node::kind_type::spawn:
	{
		tool.status = (state.vehicles.empty() ? std::string{"Spawn point made, with no vehicles: nothing appears there until it's given some (Select, click the point)."} :
		                                        "Spawn point made, with up to " + std::to_string(state.count) + " vehicles on the roads at a time. They appear while the simulation runs.");
		break;
	}
	case roadpoint_node::kind_type::despawn:
	{
		tool.status = "Removal point made, it takes vehicles from " + std::to_string(point->lanes().size()) + (point->lanes().size() == 1 ? " lane." : " lanes.");
		break;
	}
	case roadpoint_node::kind_type::crosswalk:
	{
		tool.status = "Pedestrian crossing painted.";
		break;
	}
	}
	if (false == tool.targetnote.empty())
		tool.status += " " + tool.targetnote + ".";
}

// replaces the definition of a level crossing or a traffic point, as a step which can be taken back
void editor_mode::roadpoint_apply(roadpoint_node &Point, roadpoint_node::state const &State)
{
	auto &tool{m_roadtool};
	std::string reason;
	if (false == editor_road::can_edit(Point, &reason))
	{
		tool.status = "\"" + Point.name() + "\" can't be changed: " + reason;
		return;
	}
	editor_road::record record;
	record.points.emplace_back(&Point, Point.definition());
	editor_road::apply(std::vector<std::pair<roadpoint_node *, roadpoint_node::state>>{{&Point, State}});
	push_road_snapshot(std::move(record));
	tool.status = "Changed";
}

// draws controls for a level crossing or a traffic point. returns: true if something was changed
bool editor_mode::render_roadpoint_layout(roadpoint_node::state &State, roadpoint_node const *Point)
{
	auto &tool{m_roadtool};
	bool changed{false};
	// values typed for a point which is there already are taken when confirmed, so the point isn't made anew with each digit
	auto const flags{Point != nullptr ? ImGuiInputTextFlags_EnterReturnsTrue : ImGuiInputTextFlags_None};
	auto const number = [flags](char const *Label, float &Value, float const Step, char const *Format) {
		ImGui::SetNextItemWidth(120.0f);
		return ImGui::InputFloat(Label, &Value, Step, Step * 4.0f, Format, flags);
	};
	ImVec4 const warning{1.0f, 0.6f, 0.3f, 1.0f};
	switch (State.kind)
	{
	case roadpoint_node::kind_type::crossing:
	{
		if (Point != nullptr)
		{
			ImGui::Text("%d lanes stop here, %d tracks are watched for trains", static_cast<int>(Point->stops().size()), static_cast<int>(Point->rails()));
			if (Point->rails() == 0)
				ImGui::TextColored(warning, "No rails pass through it, so it never closes");
			// the crossing is tied to what's there when it's made or changed, and when the roads change
			if (ImGui::Button("Look for the rails again"))
				changed = true;
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", "For when the tracks were laid or changed after the crossing was made.");
			if (number("Stop this far from the middle [m]", State.clearance, 0.5f, "%.1f"))
				changed = true;
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", "Where the vehicles stop, measured along the road from the middle of the crossing.\n"
				                        "The tracks passing within that distance from the middle are the ones the crossing is closed for.");
		}
		else
		{
			if (number("Stop this far from the outermost track [m]", tool.stopmargin, 0.5f, "%.1f"))
				tool.stopmargin = std::clamp(tool.stopmargin, 2.0f, 50.0f);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", "Where the vehicles stop, measured along the road from the axis of the first track they'd get to.");
		}
		if (number("Closes with a train this far away [m]", State.warning, 50.0f, "%.0f"))
			changed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "Measured along the tracks, both ways, with both ways of each switch counted.\n"
			                        "A rail vehicle within that distance which gets nearer closes the crossing, and so does one right at it.\n"
			                        "It opens a few seconds after the last one is gone.");
		if (ImGui::Checkbox("Paint the stop lines", &State.stoplines))
			changed = true;
		break;
	}
	case roadpoint_node::kind_type::spawn:
	{
		if (Point != nullptr)
		{
			if (Point->lanes().empty())
				ImGui::TextColored(warning, "It isn't on a lane, so nothing appears here");
			else if (Point->vehicles() == 0)
				ImGui::TextDisabled("The vehicles are made when the simulation runs");
			else
				ImGui::Text("%d vehicles made, %d of them wait for their turn", static_cast<int>(Point->vehicles()), static_cast<int>(Point->waiting()));
		}
		if (number("A vehicle every [s]", State.interval, 1.0f, "%.0f"))
			changed = true;
		if (number("Uneven by [0-1]", State.variation, 0.1f, "%.2f"))
			changed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "Part of the time between vehicles which is drawn at random each time.\n"
			                        "0: they come like clockwork, 1: anything from one right after another to twice the time.");
		if (number("Speed [km/h]", State.velocity, 5.0f, "%.0f"))
			changed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "The speed the vehicles appear with. From there on the drivers go by the speed limits of the lanes.");
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::InputInt("Vehicles on the roads at a time", &State.count, 1, 5, flags))
			changed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "How many vehicles of this point can be on the scenery at once. That many are made along with the scenery,\n"
			                        "drawn from the listed ones in turn; one of those which wait is put on the road each time its turn comes,\n"
			                        "until all of them are out. A vehicle taken by a removal point goes back to wait for its next turn,\n"
			                        "and so does one which stood still for 15 seconds, other than at a closed level crossing, or went backwards.");
		// the set
		ImGui::Separator();
		ImGui::Text("Vehicles of the set: %d", static_cast<int>(State.vehicles.size()));
		int removed{-1};
		for (std::size_t index = 0; index < State.vehicles.size(); ++index)
		{
			auto const &vehicle{State.vehicles[index]};
			ImGui::PushID(static_cast<int>(index));
			if (ImGui::SmallButton("x"))
				removed = static_cast<int>(index);
			ImGui::SameLine();
			if (vehicle.load > 0)
				ImGui::Text("%s / %s, %s, %d %s", vehicle.folder.c_str(), vehicle.type.c_str(), vehicle.skin.c_str(), vehicle.load, vehicle.loadtype.c_str());
			else
				ImGui::Text("%s / %s, %s", vehicle.folder.c_str(), vehicle.type.c_str(), vehicle.skin.c_str());
			ImGui::PopID();
		}
		if (removed >= 0)
		{
			State.vehicles.erase(State.vehicles.begin() + removed);
			changed = true;
		}
		if (State.vehicles.empty())
			ImGui::TextColored(warning, "No vehicles, so nothing appears here");
		ImGui::InputTextMultiline("##vehicles", tool.vehicles, sizeof(tool.vehicles), ImVec2(-1.0f, ImGui::GetTextLineHeight() * 5.0f));
		if (ImGui::Button("Add to the set"))
		{
			auto const parsed{roadpoint_node::parse_vehicles(tool.vehicles)};
			if (parsed.empty())
			{
				tool.status = "No vehicle found in the text";
			}
			else
			{
				State.vehicles.insert(State.vehicles.end(), parsed.begin(), parsed.end());
				tool.vehicles[0] = 0;
				changed = true;
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Empty the set") && false == State.vehicles.empty())
		{
			State.vehicles.clear();
			changed = true;
		}
		ImGui::TextDisabled("Paste the vehicles the way they're put in a scenery:");
		ImGui::TextDisabled("node -1 0 none dynamic <folder> <skin> <type> <path>");
		ImGui::TextDisabled("  <offset> <driver> <velocity> <load> enddynamic");
		ImGui::TextDisabled("or as <folder> <skin> <type>, one vehicle a line.");
		ImGui::TextDisabled("The path, the offset, the driver and the velocity don't matter.");
		break;
	}
	case roadpoint_node::kind_type::despawn:
	{
		if (Point != nullptr)
			ImGui::Text("Takes vehicles from %d %s", static_cast<int>(Point->lanes().size()), Point->lanes().size() == 1 ? "lane" : "lanes");
		if (number("Takes vehicles this close [m]", State.radius, 0.5f, "%.1f"))
			changed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "Road vehicles on the lanes passing within this distance from the point are taken off the road when they get here,\n"
			                        "except for the one the user drives. Vehicles which came from a spawn point go back to wait for their next turn.\n"
			                        "The distance it starts with takes in a single lane.");
		break;
	}
	case roadpoint_node::kind_type::crosswalk:
	{
		if (number("Length along the road [m]", State.length, 0.5f, "%.1f"))
			changed = true;
		ImGui::TextDisabled("Stripes painted from one edge of the road to the other.");
		ImGui::TextDisabled("The drivers don't stop for it: there's nobody to cross.");
		break;
	}
	}
	if (changed && Point != nullptr)
		State.normalize();
	return changed;
}

// replaces the definition of a junction, as a step which can be taken back
void editor_mode::junction_apply(junction_node &Junction, junction_node::state const &State)
{
	auto &tool{m_roadtool};
	std::string reason;
	if (false == editor_road::can_edit(Junction, &reason))
	{
		tool.status = "\"" + Junction.name() + "\" can't be changed: " + reason;
		// what the window shows goes back to what the junction is like
		junction_select(&Junction);
		tool.status = "\"" + Junction.name() + "\" can't be changed: " + reason;
		return;
	}
	editor_road::record record;
	record.junctions.emplace_back(&Junction, Junction.definition());
	editor_road::apply(std::vector<std::pair<junction_node *, junction_node::state>>{{&Junction, State}});
	push_road_snapshot(std::move(record));
	tool.status = "Changed";
}

// draws controls for what a junction is like, and makes the changes as they're made
void editor_mode::render_junction_layout(junction_node &Junction)
{
	auto &tool{m_roadtool};
	auto state{Junction.definition()};
	bool changed{false};
	// typed values are taken when confirmed, so the junction isn't made anew with each digit
	auto const number = [](char const *Label, float &Value, float const Step, char const *Format) {
		ImGui::SetNextItemWidth(120.0f);
		return ImGui::InputFloat(Label, &Value, Step, Step * 4.0f, Format, ImGuiInputTextFlags_EnterReturnsTrue);
	};

	auto limit{state.velocity > 0.f ? state.velocity : 0.f};
	if (number("Speed limit [km/h], 0: none", limit, 5.0f, "%.0f"))
	{
		state.velocity = (limit > 0.f ? limit : -1.f);
		changed = true;
	}
	auto centre{static_cast<float>(state.centre.y)};
	if (number("Height of the middle [m]", centre, 0.1f, "%.2f"))
	{
		state.centre.y = centre;
		changed = true;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", "The surface is spanned between the ends of the roads and this point.\nThe ends of the roads are raised or lowered as points: select the white dot at the end of a road.");

	// appearance
	ImGui::Separator();
	char const *colours[]{"none", "white", "orange"};
	int colour{state.markings == road_node::marking_colour::white ? 1 : state.markings == road_node::marking_colour::orange ? 2 : 0};
	ImGui::SetNextItemWidth(120.0f);
	if (ImGui::Combo("Markings", &colour, colours, 3))
	{
		state.markings = (colour == 1 ? road_node::marking_colour::white : colour == 2 ? road_node::marking_colour::orange : road_node::marking_colour::none);
		changed = true;
	}
	if (render_road_material("Surface", tool.surface, sizeof(tool.surface), state.surface))
		changed = true;
	if (number("Texture length [m]", state.texturelength, 0.5f, "%.2f"))
	{
		state.texturelength = std::max(0.1f, state.texturelength);
		changed = true;
	}
	// what the corners between the roads are lined with
	if (ImGui::Checkbox("Kerb around the corners", &state.kerbs))
		changed = true;
	if (state.kerbs)
	{
		ImGui::Indent();
		if (number("Kerb width [m]", state.kerbwidth, 0.05f, "%.2f"))
		{
			state.kerbwidth = std::clamp(state.kerbwidth, 0.05f, 2.0f);
			changed = true;
		}
		if (render_road_material("Kerb material", tool.kerbtext, sizeof(tool.kerbtext), state.kerbmaterial))
			changed = true;
		if (state.side.type != road_node::side_type::sidewalk && number("Kerb height [m]", state.kerbheight, 0.01f, "%.2f"))
		{
			state.kerbheight = std::clamp(state.kerbheight, 0.0f, 1.0f);
			changed = true;
		}
		ImGui::Unindent();
	}
	char const *sidetypes[]{"none", "shoulder", "sidewalk"};
	int type{state.side.type == road_node::side_type::shoulder ? 1 : state.side.type == road_node::side_type::sidewalk ? 2 : 0};
	ImGui::SetNextItemWidth(120.0f);
	if (ImGui::Combo("Sides of the corners", &type, sidetypes, 3))
	{
		state.side.type = (type == 1 ? road_node::side_type::shoulder : type == 2 ? road_node::side_type::sidewalk : road_node::side_type::none);
		if (state.side.type != road_node::side_type::none && state.side.width <= 0.0f)
			state.side.width = 1.5f;
		changed = true;
	}
	if (state.side.type != road_node::side_type::none)
	{
		ImGui::Indent();
		if (number("Width [m]", state.side.width, 0.25f, "%.2f"))
		{
			state.side.width = std::clamp(state.side.width, 0.1f, 20.0f);
			changed = true;
		}
		if (render_road_material("Material", tool.sides[0], sizeof(tool.sides[0]), state.side.material))
			changed = true;
		if (state.side.material.empty() || state.side.material == "none")
			ImGui::TextDisabled("Pick a material to have the sides drawn");
		if (state.side.type == road_node::side_type::sidewalk)
		{
			if (number("Kerb height [m]", state.kerbheight, 0.01f, "%.2f"))
			{
				state.kerbheight = std::clamp(state.kerbheight, 0.0f, 1.0f);
				changed = true;
			}
		}
		else
		{
			if (number("Bank width [m]", state.slope.x, 0.25f, "%.2f"))
			{
				state.slope.x = std::clamp(state.slope.x, 0.0f, 20.0f);
				changed = true;
			}
			if (number("Bank drop [m]", state.slope.y, 0.1f, "%.2f"))
			{
				state.slope.y = std::clamp(state.slope.y, 0.0f, 20.0f);
				changed = true;
			}
		}
		ImGui::Unindent();
	}

	// banks leading from the corners to the ground
	if (ImGui::CollapsingHeader("Banks to the ground"))
	{
		if (render_road_material("Bank material", tool.banktext, sizeof(tool.banktext), state.bankmaterial))
			changed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "What the banks are covered with next to a sidewalk, a kerb or the bare edge.\nA bank of a shoulder is covered with the material of the shoulder.");
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::InputFloat("Run for a metre of drop [m]", &tool.bankgrade, 0.25f, 0.5f, "%.2f"))
			tool.bankgrade = std::clamp(tool.bankgrade, 0.25f, 10.0f);
		if (ImGui::Button("Lead the banks to the ground"))
		{
			if (changed)
				junction_apply(Junction, state);
			junction_bank_to_ground(Junction);
			return;
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "Looks for the ground past each corner, at both roads the corner runs between,\n"
			                        "and sets the bank of the corner to reach it. Done for the roads too, the banks meet at the ends of the roads.");
		auto banked{false};
		for (auto const &arm : state.arms)
			banked = banked || arm.bank.set;
		if (banked)
		{
			ImGui::SameLine();
			if (ImGui::Button("Remove them"))
			{
				for (auto &arm : state.arms)
					arm.bank = road_node::bank_data{};
				changed = true;
			}
		}
	}
	if (state.arms.size() == 2)
	{
		// where a road changes its lanes: what's between its two directions, if the roads keep them apart
		ImGui::Separator();
		char const *mediantypes[]{"nothing, the ground shows", "surface closed to the traffic", "island"};
		int median{state.median == road_node::median_type::gap ? 0 : state.median == road_node::median_type::island ? 2 : 1};
		ImGui::SetNextItemWidth(220.0f);
		if (ImGui::Combo("Between the directions", &median, mediantypes, 3))
		{
			state.median = (median == 0 ? road_node::median_type::gap : median == 2 ? road_node::median_type::island : road_node::median_type::painted);
			changed = true;
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "Shows where a road which keeps its two directions apart meets this stretch:\n"
			                        "what's between the directions is led on to the other road, where it closes or carries on.");
		if (state.median == road_node::median_type::island && render_road_material("Island material", tool.mediantext, sizeof(tool.mediantext), state.medianmaterial))
			changed = true;
	}

	// the roads
	if (ImGui::CollapsingHeader("Roads of the junction", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::TextDisabled("The lanes are taken from the roads: change them on the road, the junction follows.");
		ImGui::TextDisabled("The roads are numbered in the view. Lanes are counted from the middle of the road;");
		ImGui::TextDisabled("automatic: the inner lane turns left, the outer one right, all go straight.");
		for (std::size_t arm = 0; arm < state.arms.size(); ++arm)
		{
			ImGui::PushID(static_cast<int>(arm));
			auto &data{state.arms[arm]};
			ImGui::Text("%d: %d in, %d out, %.2f m a lane", static_cast<int>(arm) + 1, data.incoming, data.outgoing, data.width);
			if (state.arms.size() > 2)
			{
				ImGui::SameLine();
				if (ImGui::Checkbox("Stop line", &data.stopline))
					changed = true;
				// stripes for the pedestrians, across the whole road where it meets the junction
				ImGui::SameLine();
				bool zebra{data.crosswalk > 0.f};
				if (ImGui::Checkbox("Pedestrian crossing", &zebra))
				{
					data.crosswalk = (zebra ? 4.f : 0.f);
					changed = true;
				}
				if (zebra)
				{
					ImGui::SameLine();
					ImGui::SetNextItemWidth(90.0f);
					if (ImGui::InputFloat("m##crosswalk", &data.crosswalk, 0.5f, 1.0f, "%.1f", ImGuiInputTextFlags_EnterReturnsTrue))
					{
						data.crosswalk = std::clamp(data.crosswalk, 1.0f, 20.0f);
						changed = true;
					}
				}
			}
			if (data.incoming > 0 && state.arms.size() > 2)
			{
				ImGui::Indent();
				// the entries are in the order of the stored values
				char const *priorities[]{"no sign: gives way to the right", "has the right of way", "gives way", "stops, then gives way"};
				int priority{static_cast<int>(data.priority)};
				ImGui::SetNextItemWidth(220.0f);
				if (ImGui::Combo("##priority", &priority, priorities, 4))
				{
					data.priority = static_cast<junction_node::right_of_way>(std::clamp(priority, 0, 3));
					changed = true;
				}
				// ways each lane can be left by
				char const *ways[]{"automatic", "left", "straight", "left, straight", "right", "left, right", "straight, right", "any"};
				for (int lane = 0; lane < data.incoming; ++lane)
				{
					ImGui::PushID(100 + lane);
					int turns{static_cast<std::size_t>(lane) < data.turns.size() ? data.turns[lane] & 7 : 0};
					auto const label{"lane " + std::to_string(lane + 1) + " goes"};
					ImGui::SetNextItemWidth(140.0f);
					if (ImGui::Combo(label.c_str(), &turns, ways, 8))
					{
						data.turns.resize(std::max(data.turns.size(), static_cast<std::size_t>(lane) + 1), 0);
						data.turns[lane] = turns;
						changed = true;
					}
					ImGui::PopID();
				}
				ImGui::Unindent();
			}
			ImGui::PopID();
		}
	}
	if (changed)
		junction_apply(Junction, state);
}

void editor_mode::road_cancel()
{
	auto &tool{m_roadtool};
	tool.chain = false;
	tool.branchroad = nullptr;
	tool.branchjunction = nullptr;
}

// works out what a click of the build tool would make with the cursor where it is
editor_mode::road_plan editor_mode::road_preview(bool const Fresh)
{
	auto const &tool{m_roadtool};
	auto const &layout{tool.settings};
	road_plan plan;
	// lanes of the road being built which lead to its far end, and the ones leading back from there
	auto const arriving{tool.reversed ? layout.backward : layout.forward};
	auto const leaving{tool.reversed ? layout.forward : layout.backward};
	float lanewidth{0.f};
	auto const evenlanes{editor_road::lane_width(layout, 0.0, lanewidth)};
	char const *unevenlanes{"Lanes of the road being built differ in width, and a junction takes roads with lanes of the same width"};

	// where the road is led to. a loose end of a road or an arm of a junction near the cursor comes first
	plan.snap = editor_road::find_end(tool.mouse, kSnapRadius);
	// the end the road is being drawn from isn't somewhere to lead it to
	if (plan.snap.valid() && glm::distance(plan.snap.position, tool.point) < 0.5)
		plan.snap = {};
	auto target{tool.mouse + glm::dvec3{0.0, tool.offset, 0.0}};
	glm::dvec2 enddirection{0.0, 1.0};
	double endgrade{0.0};
	bool tied{false}; // the road has to arrive at the target in set direction, with set slope
	if (plan.snap.valid())
	{
		glm::dvec2 const outwards{plan.snap.outwards.x, plan.snap.outwards.z};
		if (glm::length(outwards) > 1e-6)
		{
			target = plan.snap.position;
			enddirection = -glm::normalize(outwards);
			endgrade = -plan.snap.outwards.y / glm::length(outwards);
			tied = true;
		}
		else
		{
			plan.snap = {};
		}
	}
	if (plan.snap.junction != nullptr || plan.snap.half != 0)
	{
		// the junction has its ways through made for a set of lanes at each arm; a single roadway of a road takes the lanes it has
		if (false == evenlanes || arriving != plan.snap.backward || leaving != plan.snap.forward || std::abs(lanewidth - plan.snap.width) > 0.01f)
		{
			char width[32];
			std::snprintf(width, sizeof(width), "%.2f", plan.snap.width);
			plan.error = std::string{plan.snap.junction != nullptr ? "This arm of the junction" : "This roadway"} + " takes a road with ";
			plan.error += std::to_string(plan.snap.backward) + " lane(s) leading to it and " + std::to_string(plan.snap.forward) + " away from it, " + width + " m wide each";
			return plan;
		}
	}
	else if (false == plan.snap.valid() && glm::distance(tool.mouse, tool.point) > kJoinClearance)
	{
		// then the side of a road, or a junction between its roads: the road is taken in with a junction made there, or with another arm of the one which is there
		auto *road{editor_road::nearest(tool.mouse, kSideMargin)};
		auto *junction{road == nullptr ? editor_road::nearest_junction(tool.mouse) : nullptr};
		if (road != nullptr && road == tool.branchroad)
			road = nullptr;
		if (junction != nullptr && junction == tool.branchjunction)
			junction = nullptr;
		if (road != nullptr || junction != nullptr)
		{
			if (false == evenlanes)
			{
				plan.error = unevenlanes;
				return plan;
			}
			auto const t{road != nullptr ? editor_road::nearest_parameter(road->definition(), tool.mouse) : 0.0};
			auto const centre{road != nullptr ? road->definition().point(t) : junction->definition().centre};
			// seen from the junction the road comes from where it's being led from
			glm::dvec2 const from{tool.point.x - centre.x, tool.point.z - centre.z};
			if (false == (road != nullptr ? editor_road::branch_from(*road, t, from, arriving, leaving, lanewidth, plan.end, plan.error) : editor_road::branch_from(*junction, from, arriving, leaving, lanewidth, plan.end, plan.error)))
				return plan;
			plan.hasend = true;
			auto const &arm{plan.end.layout.arms[plan.end.arm]};
			target = arm.position;
			enddirection = -arm.direction;
			tied = true;
		}
	}

	// where the road starts
	auto start{tool.point};
	auto direction{tool.direction};
	auto hasdirection{tool.hasdirection};
	auto startgrade{tool.grade};
	auto hasgrade{tool.hasgrade};
	if (tool.branchroad != nullptr || tool.branchjunction != nullptr)
	{
		// out of the side of a road, or out of a junction, in the direction of where it's led to
		if (false == evenlanes)
		{
			plan.error = unevenlanes;
			return plan;
		}
		glm::dvec2 const away{target.x - tool.point.x, target.z - tool.point.z};
		if (glm::length(away) < 1.0)
		{
			plan.error = "Move the cursor to where the road is to go";
			return plan;
		}
		// the axis of the road goes away from the junction, so the lanes going against it are the ones leading into the junction
		if (false == (tool.branchroad != nullptr ? editor_road::branch_from(*tool.branchroad, tool.branchat, away, layout.backward, layout.forward, lanewidth, plan.start, plan.error) :
		                                           editor_road::branch_from(*tool.branchjunction, away, layout.backward, layout.forward, lanewidth, plan.start, plan.error)))
			return plan;
		plan.hasstart = true;
		auto const &arm{plan.start.layout.arms[plan.start.arm]};
		start = arm.position;
		direction = arm.direction;
		hasdirection = true;
		startgrade = 0.0;
		hasgrade = true;
	}
	if (plan.hasstart && plan.hasend)
	{
		// the two junctions are laid out for the roads as they are now, which stops being true for the second once the first is made
		bool shared{plan.start.junction != nullptr && plan.start.junction == plan.end.junction};
		auto const touches = [](editor_road::branch const &Branch, road_node const *Road) {
			return std::any_of(Branch.changes.begin(), Branch.changes.end(), [Road](std::pair<road_node *, road_node::state> const &Change) { return Change.first == Road; }) ||
			       std::find(Branch.removed.begin(), Branch.removed.end(), Road) != Branch.removed.end();
		};
		for (auto const &change : plan.start.changes)
			shared = shared || touches(plan.end, change.first);
		for (auto const *road : plan.start.removed)
			shared = shared || touches(plan.end, road);
		if (shared)
		{
			plan.error = "The junctions at both ends would take the same piece of road; lead the road somewhere else first";
			return plan;
		}
	}
	if (glm::distance(start, target) > kLongestStretch)
	{
		plan.error = "Too far from the last point";
		return plan;
	}
	plan.pieces = editor_road::plan(start, hasdirection ? &direction : nullptr, target, tied ? &enddirection : nullptr, Global.ctrlState, plan.error);
	road_profile(plan.pieces, hasgrade ? &startgrade : nullptr, tied ? &endgrade : nullptr, Fresh);
	return plan;
}

// gives the pieces laid out by the planner their heights: over the ground if the road is to follow it, with the slopes
// of the pieces matched where they meet, and with what the road is attached to
void editor_mode::road_profile(std::vector<segment_data> &Pieces, double const *Firstgrade, double const *Lastgrade, bool const Fresh)
{
	auto const &tool{m_roadtool};
	if (Pieces.empty())
		return;
	if (false == tool.follow)
	{
		// straight from a point to the next one, the way a ramp or a viaduct goes
		std::vector<double> heights;
		for (auto const &piece : Pieces)
			heights.emplace_back(piece.points[segment_data::point::start].y);
		heights.emplace_back(Pieces.back().points[segment_data::point::end].y);
		editor_road::profile(Pieces, heights, Firstgrade, Lastgrade);
		return;
	}
	auto const planned{Pieces};
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
		auto ground{ground_heights(points, Fresh)};
		for (auto &height : ground)
			height += tool.offset;
		std::vector<double> heights(ground.begin(), ground.begin() + count + 1);
		// the ends are where they were put
		heights.front() = Pieces.front().points[segment_data::point::start].y;
		heights.back() = Pieces.back().points[segment_data::point::end].y;
		editor_road::profile(Pieces, heights, Firstgrade, Lastgrade);
		// shorter pieces are called for if the ground shows through the road, or the road hangs over it.
		// the pieces attached to something are let off, as they have to get to its height one way or another
		bool fits{true};
		for (std::size_t i = 0; i < count; ++i)
		{
			if ((i == 0 && Firstgrade != nullptr) || (i + 1 == count && Lastgrade != nullptr))
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
	if (tool.tool == 2)
	{
		roadpoint_place();
		return;
	}
	if (tool.tool == 0)
	{
		if (auto *marker{editor_road::nearest_point(ground, kMarkerRadius)}; marker != nullptr)
		{
			// a level crossing or a traffic point
			tool.points.clear();
			tool.junction = nullptr;
			road_select(nullptr);
			roadpoint_select(marker);
			return;
		}
		tool.marker = nullptr;
		glm::dvec3 joint;
		if (editor_road::nearest_joint(ground, kPointRadius, joint))
		{
			// a point where pieces meet or end. with Shift it's added to the selected ones, or taken out of them
			auto const listed{std::find_if(tool.points.begin(), tool.points.end(), [&joint](glm::dvec3 const &Point) { return glm::distance(Point, joint) < 0.25; })};
			if (false == Global.shiftState)
				tool.points.assign(1, joint);
			else if (listed != tool.points.end())
				tool.points.erase(listed);
			else
				tool.points.emplace_back(joint);
			tool.selected = nullptr;
			tool.junction = nullptr;
			tool.status = std::to_string(tool.points.size()) + (tool.points.size() == 1 ? " point selected" : " points selected");
			return;
		}
		tool.points.clear();
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
	if (tool.ring)
	{
		road_roundabout(ground);
		return;
	}
	if (false == tool.chain)
	{
		// first click sets where the road starts
		tool.branchroad = nullptr;
		tool.branchjunction = nullptr;
		tool.selected = nullptr;
		tool.junction = nullptr;
		tool.points.clear();
		auto const end{editor_road::find_end(ground, kSnapRadius)};
		glm::dvec2 const outwards{end.outwards.x, end.outwards.z};
		if (end.valid() && glm::length(outwards) > 1e-6)
		{
			// started from a loose end of a road it carries that road on, started from an arm of a junction it's the road the junction is made for there
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
			if (end.road != nullptr && end.half == 0)
			{
				// carried on from its start the road keeps the direction of its axis, so the lanes stay what they were
				tool.reversed = (false == end.atend);
				// with the lanes it has at that end
				tool.settings = end.road->definition().section(end.atend ? 1 : 0);
				// and it stays as wide as it is at that end, with its directions as far apart
				auto const scale{tool.settings.taper[end.atend ? 1 : 0]};
				tool.settings.taper = {scale, scale};
				auto const gap{tool.settings.median.width[end.atend ? 1 : 0]};
				tool.settings.median.width = {gap, gap};
				// the banks were made for the ground where they are
				tool.settings.banks = {};
				tool.status = "Carrying on \"" + end.road->name() + "\" with its layout";
			}
			else
			{
				tool.reversed = false;
				if (end.road != nullptr)
				{
					// one of the roadways of a road which keeps its directions apart is carried on as a road of its own, which looks like the road it leaves
					tool.settings = end.road->definition();
					tool.settings.sides[end.half > 0 ? 0 : 1] = road_node::side_data{};
					tool.settings.banks = {};
					tool.settings.missing = {};
				}
				tool.settings.forward = end.forward;
				tool.settings.backward = end.backward;
				tool.settings.lanewidth = end.width;
				tool.settings.taper = {1.f, 1.f};
				tool.settings.median.width = {0.f, 0.f};
				tool.settings.lanes.clear();
				tool.settings.changes.clear();
				tool.settings.normalize();
				tool.status = (end.junction != nullptr ? "Leading a road out of \"" + end.junction->name() + "\", with the lanes the junction has there" :
				                                         "Carrying on one of the roadways of \"" + end.road->name() + "\" as a one-way road");
			}
			road_select(nullptr);
		}
		else
		{
			tool.hasdirection = false;
			tool.hasgrade = false;
			tool.reversed = false;
			tool.settings.taper = {1.f, 1.f};
			if (auto *road{editor_road::nearest(ground, kSideMargin)}; road != nullptr)
			{
				// started from the side of a road it leaves that road at a junction, made along with its first piece
				tool.branchroad = road;
				tool.branchat = editor_road::nearest_parameter(road->definition(), ground);
				tool.point = road->definition().point(tool.branchat);
				tool.status = "Leading a road out of the side of \"" + road->name() + "\": click where it goes";
			}
			else if (auto *junction{editor_road::nearest_junction(ground)}; junction != nullptr)
			{
				// started from a junction it's another road of that junction
				if (junction->definition().arms.size() >= 4)
				{
					tool.status = "\"" + junction->name() + "\" has four roads already";
					return;
				}
				tool.branchjunction = junction;
				tool.point = junction->definition().centre;
				tool.status = "Leading another road out of \"" + junction->name() + "\": click where it goes";
			}
			else
			{
				tool.point = ground + glm::dvec3{0.0, tool.offset, 0.0};
				tool.status = "Start set";
			}
		}
		tool.chain = true;
		return;
	}

	auto plan{road_preview(true)};
	if (plan.pieces.empty())
	{
		tool.status = "Can't build this: " + plan.error;
		return;
	}
	if (plan.snap.valid() && false == editor_road::can_join(plan.snap, &reason))
	{
		tool.status = "Can't join here: " + reason;
		return;
	}
	auto pieces{plan.pieces};
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
	// the junctions the road starts or ends with are made first, so the road finds them when it's made
	editor_road::record record;
	if (plan.hasstart)
	{
		if (plan.start.junction == nullptr)
			plan.start.layout.velocity = tool.crossingspeed;
		editor_road::carry_out(plan.start, record);
	}
	if (plan.hasend)
	{
		if (plan.end.junction == nullptr)
			plan.end.layout.velocity = tool.crossingspeed;
		editor_road::carry_out(plan.end, record);
	}
	std::vector<road_node::state> states;
	// what changes from the start of a piece to its end is spread over the whole stretch being built,
	// so the pieces don't each go through all of it
	std::vector<double> stations{0.0};
	for (auto const &piece : pieces)
		stations.emplace_back(stations.back() + editor_road::planar_length(piece));
	auto const along = [&stations](std::size_t const Index) {
		auto const fraction{stations.back() > 0.0 ? stations[Index] / stations.back() : 0.0};
		return static_cast<float>(fraction * fraction * (3.0 - 2.0 * fraction));
	};
	for (std::size_t index = 0; index < pieces.size(); ++index)
	{
		auto state{tool.settings};
		state.axis = pieces[index];
		state.missing = {};
		auto const from{along(index)};
		auto const to{along(index + 1)};
		auto const &median{tool.settings.median};
		state.median.width = {glm::mix(median.width[0], median.width[1], from), glm::mix(median.width[0], median.width[1], to)};
		// an island is rounded where the stretch starts and ends
		state.median.round = {index == 0 ? median.round[0] : 0.f, index + 1 == pieces.size() ? median.round[1] : 0.f};
		for (std::size_t side = 0; side < state.banks.size(); ++side)
		{
			auto const &bank{tool.settings.banks[side]};
			state.banks[side].width = {glm::mix(bank.width[0], bank.width[1], from), glm::mix(bank.width[0], bank.width[1], to)};
			state.banks[side].drop = {glm::mix(bank.drop[0], bank.drop[1], from), glm::mix(bank.drop[0], bank.drop[1], to)};
		}
		states.emplace_back(state);
	}
	auto const created{editor_road::create(states)};
	record.roads_created.insert(record.roads_created.end(), created.begin(), created.end());
	push_road_snapshot(std::move(record));
	tool.branchroad = nullptr;
	tool.branchjunction = nullptr;
	if (created.empty())
	{
		tool.chain = false;
		tool.status = "The road wasn't created";
		return;
	}
	if (plan.snap.valid() || plan.hasend)
	{
		tool.chain = false;
		tool.status = (plan.snap.valid() ? "Joined with \"" + (plan.snap.road != nullptr ? plan.snap.road->name() : plan.snap.junction->name()) + "\"" :
		               plan.end.junction != nullptr ? "Led into \"" + plan.end.junction->name() + "\" as another of its roads" :
		                                              std::string{"Led into the side of the road, with a junction made there"});
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
	if (plan.hasstart)
		tool.status += (plan.start.junction != nullptr ? ", as another road of the junction" : ", with a junction where the road leaves");
}

void editor_mode::road_apply()
{
	auto &tool{m_roadtool};
	if (tool.selected == nullptr)
		return;
	tool.settings.normalize();
	// other lanes at the end are something the selected piece has for itself; the rest of the road keeps the lanes it has
	auto const pieces{tool.whole && false == tool.settings.changing() ? editor_road::chain(*tool.selected) : std::vector<road_node *>{tool.selected}};
	editor_road::record record;
	std::vector<std::pair<road_node *, road_node::state>> changes;
	// the banks are kept for each piece as they are, unless it's the banks which were just changed
	auto const banked{tool.settings.banks != tool.selected->definition().banks};
	for (auto *road : pieces)
	{
		std::string reason;
		if (false == editor_road::can_edit(*road, &reason))
		{
			// what the window shows goes back to what the road is like
			road_select(tool.selected);
			tool.status = "\"" + road->name() + "\" can't be changed: " + reason;
			return;
		}
		auto state{tool.settings};
		state.axis = road->definition().axis;
		// how wide the road is at its points is set at these points, for each piece on its own
		state.taper = road->definition().taper;
		if (road != tool.selected)
		{
			// and so is how far apart the two directions are led, and the rounded ends of what's between them
			state.median.width = road->definition().median.width;
			state.median.round = road->definition().median.round;
			// banks led to the ground differ from piece to piece, so a change of something else leaves them alone
			if (false == banked)
				state.banks = road->definition().banks;
		}
		if (road != tool.selected && road->definition().changes.size() == state.changes.size())
		{
			// where the lanes can be changed is set for each piece on its own
			state.changes = road->definition().changes;
		}
		record.roads.emplace_back(road, road->definition());
		changes.emplace_back(road, state);
	}
	editor_road::apply(changes);
	// what's next to the changed pieces is fitted to them: junctions take the lanes they have now, the pieces of the road
	// which stay as they were get wider or narrower towards them, or give up their ends for the lanes to change
	std::string notes;
	for (auto *road : pieces)
		editor_road::settle(*road, record, notes);
	auto const fitted{record.roads.size() + record.roads_removed.size() + record.junctions.size() + record.junctions_created.size() - pieces.size()};
	push_road_snapshot(std::move(record));
	tool.settings = tool.selected->definition();
	tool.status = (pieces.size() == 1 ? std::string{"Changed."} : "Changed " + std::to_string(pieces.size()) + " pieces.");
	if (fitted > 0)
		tool.status += " What's next to it was fitted to it.";
	if (false == notes.empty())
		tool.status += " " + notes;
}

bool editor_mode::road_delete()
{
	auto &tool{m_roadtool};
	if (false == tool.window)
		return false;
	std::string reason;
	editor_road::record record;
	if (tool.marker != nullptr)
	{
		auto *point{tool.marker};
		if (false == editor_road::can_edit(*point, &reason))
		{
			tool.status = "Can't delete: " + reason;
			return true;
		}
		editor_road::remove(std::vector<roadpoint_node *>{point});
		record.points_removed.emplace_back(point);
		push_road_snapshot(std::move(record));
		tool.marker = nullptr;
		tool.status = "\"" + point->name() + "\" deleted, Ctrl+Z brings it back";
		return true;
	}
	if (false == tool.points.empty())
		return road_points_delete();
	if (tool.selected == nullptr && tool.junction == nullptr)
		return false;
	if (tool.junction != nullptr)
	{
		auto *junction{tool.junction};
		if (false == editor_road::can_edit(*junction, &reason))
		{
			tool.status = "Can't delete: " + reason;
			return true;
		}
		editor_road::remove(std::vector<junction_node *>{junction});
		record.junctions_removed.emplace_back(junction);
		push_road_snapshot(std::move(record));
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
	record.roads_removed.emplace_back(road);
	push_road_snapshot(std::move(record));
	tool.selected = nullptr;
	tool.status = "\"" + road->name() + "\" deleted, Ctrl+Z brings it back";
	return true;
}

// puts a roundabout around specified point
void editor_mode::road_roundabout(glm::dvec3 const &Ground)
{
	auto &tool{m_roadtool};
	std::string reason;
	if (false == editor_road::available(&reason))
	{
		tool.status = reason;
		return;
	}
	auto const states{editor_road::roundabout(Ground + glm::dvec3{0.0, tool.offset, 0.0}, tool.ringradius, tool.settings, tool.island)};
	if (states.empty())
	{
		tool.status = "The roundabout is too small for its lanes: give it a larger radius";
		return;
	}
	for (auto const &state : states)
	{
		for (auto const t : {0.0, 0.5})
		{
			if (editor_road::nearest(state.point(t), 0.0) != nullptr || editor_road::nearest_junction(state.point(t)) != nullptr)
			{
				tool.status = "There's a road in the way. Make the roundabout next to it, then lead the road into its side";
				return;
			}
		}
	}
	auto const created{editor_road::create(states)};
	if (created.empty())
	{
		tool.status = "The roundabout wasn't created";
		return;
	}
	editor_road::record record;
	record.roads_created = created;
	push_road_snapshot(std::move(record));
	tool.status = "Roundabout made. Untick Roundabout and lead roads out of its side, or into it: the traffic joining it gives way to the traffic going around.";
}

// how far from specified point, and how far down, a slope of the grade set for the banks gets to the ground
std::pair<float, float> editor_mode::bank_reach(glm::dvec3 const &Edge, glm::dvec3 const &Outwards)
{
	auto const grade{std::clamp(static_cast<double>(m_roadtool.bankgrade), 0.25, 10.0)};
	auto const steps{static_cast<int>(kBankReach / kBankStep)};
	// the ground further and further from the edge, without the roads themselves
	std::vector<glm::dvec3> probes;
	for (int step = 1; step <= steps; ++step)
		probes.emplace_back(Edge + Outwards * (step * kBankStep));
	auto const ground{ground_heights(probes, true)};
	// the bank ends where a slope of the set grade, going down or up, gets to the ground
	auto const rising{ground.front() > Edge.y};
	auto width{kBankReach};
	auto drop{Edge.y - ground.back()};
	for (int step = 1; step <= steps; ++step)
	{
		auto const reach{step * kBankStep};
		auto const height{ground[step - 1]};
		if (rising ? height <= Edge.y + reach / grade : height >= Edge.y - reach / grade)
		{
			width = reach;
			drop = Edge.y - height;
			break;
		}
	}
	return {static_cast<float>(width), static_cast<float>(drop)};
}

// leads the banks of the corners of a junction to the ground beside it
void editor_mode::junction_bank_to_ground(junction_node &Junction)
{
	auto state{Junction.definition()};
	auto const order{state.arm_order()};
	auto const sided{state.side.type != road_node::side_type::none && state.side.width > 0.f};
	// where the banks start: past the kerb and whatever lines the corners
	auto const lining{(state.kerbs ? state.kerbwidth : 0.f) + (sided ? state.side.width : 0.f)};
	auto const raised{sided && state.side.type == road_node::side_type::sidewalk ? state.kerbheight : 0.f};
	for (std::size_t idx = 0; idx < order.size(); ++idx)
	{
		// a corner starts at the left edge of a road, seen from the junction, and ends at the right edge of the next one
		auto &arm{state.arms[order[idx]]};
		auto const &next{state.arms[order[(idx + 1) % order.size()]]};
		glm::dvec3 const outwards[2]{glm::dvec3{arm.direction.y, 0.0, -arm.direction.x}, -glm::dvec3{next.direction.y, 0.0, -next.direction.x}};
		glm::dvec3 const edges[2]{arm.position + outwards[0] * (0.5 * state.arm_width(order[idx]) + lining) + glm::dvec3{0.0, raised, 0.0},
		                          next.position + outwards[1] * (0.5 * state.arm_width(order[(idx + 1) % order.size()]) + lining) + glm::dvec3{0.0, raised, 0.0}};
		arm.bank.set = true;
		for (int end = 0; end < 2; ++end)
		{
			auto const reach{bank_reach(edges[end], outwards[end])};
			arm.bank.width[end] = reach.first;
			arm.bank.drop[end] = reach.second;
		}
	}
	junction_apply(Junction, state);
	auto &tool{m_roadtool};
	if (tool.status == "Changed")
	{
		tool.status = "Banks of the junction led to the ground.";
		if ((false == sided || state.side.type != road_node::side_type::shoulder) && (state.bankmaterial.empty() || state.bankmaterial == "none"))
			tool.status += " Pick a bank material to have them drawn.";
	}
}

// leads the banks of the selected piece, or of the whole road, to the ground beside it
void editor_mode::road_bank_to_ground()
{
	auto &tool{m_roadtool};
	if (tool.selected == nullptr)
		return;
	auto const pieces{tool.whole ? editor_road::chain(*tool.selected) : std::vector<road_node *>{tool.selected}};
	std::string reason;
	for (auto const *road : pieces)
	{
		if (false == editor_road::can_edit(*road, &reason))
		{
			tool.status = "\"" + road->name() + "\" can't be changed: " + reason;
			return;
		}
	}
	std::vector<std::pair<road_node *, road_node::state>> changes;
	for (auto *road : pieces)
	{
		auto state{road->definition()};
		for (int side = 0; side < 2; ++side)
		{
			auto const &data{state.sides[side]};
			auto const sided{data.type != road_node::side_type::none && data.width > 0.f};
			for (int end = 0; end < 2; ++end)
			{
				auto const t{static_cast<double>(end)};
				// where the bank starts: past the roadway, the kerb and whatever lies beside it
				auto const tangent{state.tangent(t)};
				glm::dvec3 across{tangent.z, 0.0, -tangent.x};
				if (glm::length(across) < 1e-9)
					continue;
				across = glm::normalize(across) * (side == 0 ? 1.0 : -1.0);
				auto edge{state.point(t) + across * (0.5 * state.span(t) + (state.kerbs[side] ? state.kerbwidth : 0.f) + (sided ? data.width : 0.f))};
				if (sided && data.type == road_node::side_type::sidewalk)
					edge.y += state.kerbheight;
				auto const reach{bank_reach(edge, across)};
				state.banks[side].set = true;
				state.banks[side].width[end] = reach.first;
				state.banks[side].drop[end] = reach.second;
			}
		}
		changes.emplace_back(road, state);
	}
	editor_road::record record;
	for (auto const &change : changes)
		record.roads.emplace_back(change.first, change.first->definition());
	editor_road::apply(changes);
	push_road_snapshot(std::move(record));
	road_select(tool.selected);
	tool.status = "Banks of " + std::to_string(changes.size()) + (changes.size() == 1 ? " piece" : " pieces") + " led to the ground.";
	auto const &layout{tool.selected->definition()};
	if ((layout.sides[0].type != road_node::side_type::shoulder || layout.sides[1].type != road_node::side_type::shoulder) && (layout.bankmaterial.empty() || layout.bankmaterial == "none"))
		tool.status += " Pick a bank material to have them drawn where the road has no shoulder.";
}

// takes the selected points out of their roads
bool editor_mode::road_points_delete()
{
	auto &tool{m_roadtool};
	editor_road::record record;
	std::string notes;
	int taken{0};
	for (auto const &point : tool.points)
	{
		std::string error;
		if (editor_road::dissolve(point, record, error))
			++taken;
		else if (notes.find(error) == std::string::npos)
			notes += " A point was left: " + error + ".";
	}
	push_road_snapshot(std::move(record));
	// the points which stayed are still selected, so it's clear which ones these are
	tool.points.erase(std::remove_if(tool.points.begin(), tool.points.end(), [](glm::dvec3 const &Point) { return editor_road::ends_at(Point).empty(); }), tool.points.end());
	tool.status = (taken == 0 ? std::string{"Nothing deleted."} : taken == 1 ? std::string{"Point deleted, Ctrl+Z brings it back."} :
	                                                                 std::to_string(taken) + " points deleted, Ctrl+Z brings them back.");
	tool.status += notes;
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
	if (road->definition().changing())
	{
		tool.status = "Can't split a piece which gains or loses lanes; give it the same lanes at both ends first";
		return true;
	}
	editor_road::record record;
	record.roads.emplace_back(road, road->definition());
	auto *created{editor_road::split(*road, editor_road::nearest_parameter(road->definition(), tool.mouse))};
	if (created == nullptr)
	{
		tool.status = "Can't split this close to the end";
		return true;
	}
	record.roads_created.emplace_back(created);
	push_road_snapshot(std::move(record));
	road_select(road);
	tool.status = "Split in two: \"" + road->name() + "\" and \"" + created->name() + "\". Click the point between them to set the width or the height of the road there";
	return true;
}

// makes the changes gathered for the selected points. returns: true if they were made
bool editor_mode::road_points_apply(std::vector<std::pair<road_node *, road_node::state>> const &Changes, std::vector<std::pair<junction_node *, junction_node::state>> const &Junctions)
{
	auto &tool{m_roadtool};
	if (Changes.empty() && Junctions.empty())
	{
		tool.status = "There's nothing to change at the selected points";
		return false;
	}
	editor_road::record record;
	std::string reason;
	for (auto const &change : Changes)
	{
		if (false == editor_road::can_edit(*change.first, &reason))
		{
			tool.status = "\"" + change.first->name() + "\" can't be changed: " + reason;
			return false;
		}
		record.roads.emplace_back(change.first, change.first->definition());
	}
	for (auto const &change : Junctions)
	{
		if (false == editor_road::can_edit(*change.first, &reason))
		{
			tool.status = "\"" + change.first->name() + "\" can't be changed: " + reason;
			return false;
		}
		record.junctions.emplace_back(change.first, change.first->definition());
	}
	editor_road::apply(Changes);
	editor_road::apply(Junctions);
	// junctions the changed pieces lead to are made for the roads as they are now
	std::string notes;
	for (auto const &change : Changes)
		editor_road::settle(*change.first, record, notes);
	push_road_snapshot(std::move(record));
	tool.status = (Changes.size() == 1 ? std::string{"Changed 1 piece."} : "Changed " + std::to_string(Changes.size()) + " pieces.");
	if (false == notes.empty())
		tool.status += " " + notes;
	return true;
}

void editor_mode::road_point_width(float const Width)
{
	// the pieces on both sides of a point get to that width there, and from there go back to their own along their length
	auto const changes{point_changes(m_roadtool.points, [Width](road_node::state &State, bool const Atend, std::size_t const) {
		auto const width{State.width(Atend ? 1 : 0)};
		if (width > 0.0)
			State.taper[Atend ? 1 : 0] = std::clamp(static_cast<float>(Width / width), 0.25f, 4.f);
	})};
	road_points_apply(changes, {});
}

// Heights: one for each of the selected points
void editor_mode::road_point_heights(std::vector<double> const &Heights)
{
	auto &tool{m_roadtool};
	if (Heights.size() != tool.points.size())
		return;
	// the control points are kept relative to the ends, so the slopes the pieces have at a point stay what they are
	auto const changes{point_changes(tool.points, [&Heights](road_node::state &State, bool const Atend, std::size_t const Point) { State.axis.points[Atend ? segment_data::point::end : segment_data::point::start].y = Heights[Point]; })};
	// an arm of a junction goes up or down with the end of the road it's made for
	std::vector<std::pair<junction_node *, junction_node::state>> junctions;
	for (std::size_t index = 0; index < tool.points.size(); ++index)
	{
		for (auto const &arm : editor_road::arms_at(tool.points[index]))
		{
			auto entry{std::find_if(junctions.begin(), junctions.end(), [&arm](std::pair<junction_node *, junction_node::state> const &Listed) { return Listed.first == arm.first; })};
			if (entry == junctions.end())
			{
				junctions.emplace_back(arm.first, arm.first->definition());
				entry = std::prev(junctions.end());
			}
			entry->second.arms[arm.second].position.y = Heights[index];
		}
	}
	if (false == road_points_apply(changes, junctions))
		return;
	// the selection follows what it's made of
	for (std::size_t index = 0; index < tool.points.size(); ++index)
		tool.points[index].y = Heights[index];
}

void editor_mode::push_road_snapshot(editor_road::record Record)
{
	if (Record.empty())
		return;

	if (m_max_history_size >= 0 && (int)m_history.size() >= m_max_history_size)
		m_history.erase(m_history.begin(), m_history.begin() + ((int)m_history.size() - m_max_history_size + 1));

	// the entry is listed under the name of the first thing it's about
	scene::basic_node const *node{nullptr};
	if (false == Record.roads.empty())
		node = Record.roads.front().first;
	else if (false == Record.roads_created.empty())
		node = Record.roads_created.front();
	else if (false == Record.roads_removed.empty())
		node = Record.roads_removed.front();
	else if (false == Record.junctions.empty())
		node = Record.junctions.front().first;
	else if (false == Record.junctions_created.empty())
		node = Record.junctions_created.front();
	else if (false == Record.junctions_removed.empty())
		node = Record.junctions_removed.front();
	else if (false == Record.points.empty())
		node = Record.points.front().first;
	else if (false == Record.points_created.empty())
		node = Record.points_created.front();
	else
		node = Record.points_removed.front();
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::RoadEdit;
	snap.node_name = node->name();
	snap.position = node->location();
	snap.roads = std::move(Record);
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::restore_road_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo)
{
	auto const &record{Snapshot.roads};
	if (record.empty())
		return;

	// what the roads are like now is what the opposite operation brings back
	EditorSnapshot current{Snapshot};
	for (auto &entry : current.roads.roads)
		entry.second = entry.first->definition();
	for (auto &entry : current.roads.junctions)
		entry.second = entry.first->definition();
	for (auto &entry : current.roads.points)
		entry.second = entry.first->definition();
	Opposite.push_back(std::move(current));

	// things are taken out before the rest is changed and put in after it, so nothing gets joined with what's on its way out
	if (Undo)
	{
		editor_road::remove(record.roads_created);
		editor_road::remove(record.junctions_created);
		editor_road::remove(record.points_created);
	}
	else
	{
		editor_road::remove(record.roads_removed);
		editor_road::remove(record.junctions_removed);
		editor_road::remove(record.points_removed);
	}
	editor_road::apply(record.roads);
	editor_road::apply(record.junctions);
	editor_road::apply(record.points);
	if (Undo)
	{
		editor_road::revive(record.roads_removed);
		editor_road::revive(record.junctions_removed);
		editor_road::revive(record.points_removed);
	}
	else
	{
		editor_road::revive(record.roads_created);
		editor_road::revive(record.junctions_created);
		editor_road::revive(record.points_created);
	}

	auto &tool{m_roadtool};
	road_cancel();
	if (tool.selected != nullptr)
		road_select(tool.selected->m_editorremoved ? nullptr : tool.selected);
	if (tool.junction != nullptr)
		junction_select(tool.junction->m_editorremoved ? nullptr : tool.junction);
	if (tool.marker != nullptr && tool.marker->m_editorremoved)
		tool.marker = nullptr;
	tool.aimedkind = -1;
}

// a field for the name of a material, with a list of the ones in the texture folder to pick from. returns: true if the material was changed
bool editor_mode::render_road_material(char const *Label, char *Buffer, std::size_t const Size, std::string &Value)
{
	auto &tool{m_roadtool};
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
	auto const material = [this](char const *Label, char *Buffer, std::size_t const Size, std::string &Value) { return render_road_material(Label, Buffer, Size, Value); };

	// lanes. a piece of a road which is there already can end with other lanes than it starts with
	{
		auto const selecting{tool.tool == 0 && tool.selected != nullptr};
		int counts[2][2]{{State.lanes_at(0, true), State.lanes_at(0, false)}, {State.lanes_at(1, true), State.lanes_at(1, false)}};
		bool relaned{false};
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::InputInt(selecting && tool.varied ? "Lanes along the axis, at the start" : "Lanes along the axis", &counts[0][0]))
			relaned = true;
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::InputInt(selecting && tool.varied ? "Lanes against the axis, at the start" : "Lanes against the axis", &counts[0][1]))
			relaned = true;
		if (selecting)
		{
			// ticked it only shows the lanes of the end to be set; unticked it gives the end the lanes of the start
			if (ImGui::Checkbox("Other lanes at the end of the piece", &tool.varied) && false == tool.varied && State.changing())
				relaned = true;
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", "The road gains or loses lanes along this piece: it gets wider or narrower, a lane it gains branches off the outermost lane\n"
				                        "of its direction (two at most), a lane it loses joins the outermost one left.\n"
				                        "What's next to the piece is fitted to the lanes the piece has at that end.\n"
				                        "The axis goes the way the lanes drawn in green do: the start of the piece is where these come from.\n"
				                        "Split the piece (K) first to have the lanes change over a part of it.");
			if (tool.varied)
			{
				ImGui::SetNextItemWidth(120.0f);
				if (ImGui::InputInt("Lanes along the axis, at the end", &counts[1][0]))
					relaned = true;
				ImGui::SetNextItemWidth(120.0f);
				if (ImGui::InputInt("Lanes against the axis, at the end", &counts[1][1]))
					relaned = true;
			}
		}
		// the traffic going along the axis gets on the piece at its start, the one going against it at its end
		if (State.lanes_at(1, true) - State.lanes_at(0, true) > 2 || State.lanes_at(0, false) - State.lanes_at(1, false) > 2)
			ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "A lane forks into three at most: lanes past the second one gained can't be got on");
		if (relaned)
		{
			if (false == selecting || false == tool.varied)
			{
				counts[1][0] = counts[0][0];
				counts[1][1] = counts[0][1];
			}
			// a different set of lanes starts with the common width and speed limit, and the default rules for changing lanes
			State.lanes_between(counts[0][0], counts[0][1], counts[1][0], counts[1][1]);
			changed = true;
		}
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
	// kerbs
	if (ImGui::Checkbox("Kerb on the left", &State.kerbs[0]))
		changed = true;
	ImGui::SameLine();
	if (ImGui::Checkbox("on the right", &State.kerbs[1]))
		changed = true;
	auto const island{State.divided() && State.median.type == road_node::median_type::island};
	if (State.kerbs[0] || State.kerbs[1] || island)
	{
		ImGui::Indent();
		if (number("Kerb width [m]", State.kerbwidth, 0.05f, "%.2f"))
		{
			State.kerbwidth = std::clamp(State.kerbwidth, 0.05f, 2.0f);
			changed = true;
		}
		if (material("Kerb material", tool.kerbtext, sizeof(tool.kerbtext), State.kerbmaterial))
			changed = true;
		if ((State.kerbs[0] || State.kerbs[1]) && (State.kerbmaterial.empty() || State.kerbmaterial == "none"))
			ImGui::TextDisabled("Pick a material to have the kerbs drawn");
		ImGui::Unindent();
	}
	if (State.kerbs[0] || State.kerbs[1] || island || State.sides[0].type == road_node::side_type::sidewalk || State.sides[1].type == road_node::side_type::sidewalk)
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
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "The bank every shoulder of the road is closed with, unless the side has a bank of its own set below.");
		if (number("Bank drop [m]", State.slope.y, 0.1f, "%.2f"))
		{
			State.slope.y = std::clamp(State.slope.y, 0.0f, 20.0f);
			changed = true;
		}
	}

	// banks leading to the ground
	if (ImGui::CollapsingHeader("Banks to the ground"))
	{
		ImGui::TextDisabled("A slope from the edge of the road down, or up, to the ground.");
		ImGui::TextDisabled("Set here it goes to every piece the changes go to; the button below");
		ImGui::TextDisabled("works out each piece on its own.");
		char const *banknames[]{"Bank on the left", "Bank on the right"};
		for (int side = 0; side < 2; ++side)
		{
			ImGui::PushID(100 + side);
			auto &bank{State.banks[side]};
			if (ImGui::Checkbox(banknames[side], &bank.set))
			{
				if (bank.set && bank.width[0] <= 0.f && bank.width[1] <= 0.f)
				{
					bank.width = {State.slope.x, State.slope.x};
					bank.drop = {State.slope.y, State.slope.y};
				}
				changed = true;
			}
			if (bank.set)
			{
				ImGui::Indent();
				if (number("Width at the start [m]", bank.width[0], 0.5f, "%.2f"))
					changed = true;
				if (number("Drop at the start [m]", bank.drop[0], 0.25f, "%.2f"))
					changed = true;
				if (number("Width at the end [m]", bank.width[1], 0.5f, "%.2f"))
					changed = true;
				if (number("Drop at the end [m]", bank.drop[1], 0.25f, "%.2f"))
					changed = true;
				ImGui::Unindent();
			}
			ImGui::PopID();
		}
		if (material("Bank material", tool.banktext, sizeof(tool.banktext), State.bankmaterial))
			changed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "What a bank is covered with next to a sidewalk, a kerb or the bare edge of the road.\nA bank of a shoulder is covered with the material of the shoulder.");
		if (tool.selected != nullptr)
		{
			ImGui::SetNextItemWidth(120.0f);
			if (ImGui::InputFloat("Run for a metre of drop [m]", &tool.bankgrade, 0.25f, 0.5f, "%.2f"))
				tool.bankgrade = std::clamp(tool.bankgrade, 0.25f, 10.0f);
			if (ImGui::Button("Lead the banks to the ground"))
			{
				road_bank_to_ground();
				// the layout shown was replaced with what the road is like now
				return false;
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", "Looks for the ground on both sides of the road, at the start and at the end of each piece,\n"
				                        "and sets the banks to reach it with the slope given above: down where the road is over the ground, up where it's cut into it.\n"
				                        "Done for the selected piece, or for every piece of the road if the changes go to the whole road.");
		}
	}

	// the two directions kept apart
	if (State.forward > 0 && State.backward > 0 && ImGui::CollapsingHeader("Between the directions"))
	{
		char const *mediantypes[]{"nothing, the ground shows", "surface closed to the traffic", "island"};
		int type{State.median.type == road_node::median_type::gap ? 0 : State.median.type == road_node::median_type::island ? 2 : 1};
		ImGui::SetNextItemWidth(220.0f);
		if (ImGui::Combo("##median", &type, mediantypes, 3))
		{
			State.median.type = (type == 0 ? road_node::median_type::gap : type == 2 ? road_node::median_type::island : road_node::median_type::painted);
			changed = true;
		}
		if (number("Apart at the start [m]", State.median.width[0], 0.5f, "%.2f"))
			changed = true;
		if (number("Apart at the end [m]", State.median.width[1], 0.5f, "%.2f"))
			changed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "How far apart the two directions are led at each end of the piece; 0 at one end and more at the other makes them part along it.\n"
			                        "Set for each piece on its own. The pieces next to it follow it where they meet, and so do the junctions.\n"
			                        "A road with its directions apart can be carried on as two one-way roads: start the Build tool\n"
			                        "on the middle of one of its roadways, at an end of the road with nothing attached.");
		if (State.median.type == road_node::median_type::island)
		{
			if (material("Island material", tool.mediantext, sizeof(tool.mediantext), State.median.material))
				changed = true;
			ImGui::TextDisabled("The island is as high as the kerbs; with a kerb material set it gets kerbs of its own.");
			if (number("Rounded at the start [m]", State.median.round[0], 0.5f, "%.2f"))
				changed = true;
			if (number("Rounded at the end [m]", State.median.round[1], 0.5f, "%.2f"))
				changed = true;
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", "Length of the rounded end of the island at that end of the piece; half of the width of the island makes it a half circle.\n"
				                        "0: the island is cut straight, with a face. Use it where the next piece has no island.");
		}
	}

	// traffic
	if (number("Traffic share, 100: like any other", State.weight, 10.0f, "%.0f"))
	{
		State.weight = std::clamp(State.weight, 0.0f, 10000.0f);
		changed = true;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", "How willing the drivers are to take this road when they leave a junction, next to the other roads they can take there.\n"
		                        "With 100 for the main road and 20 for a dirt road leaving it, one vehicle in six turns into the dirt road.\n"
		                        "0: nobody takes it, unless there's no other way.");
	if (ImGui::Checkbox("Part of a roundabout", &State.roundabout))
		changed = true;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", "A junction made on this road gives the right of way to the traffic going along it,\nand tells the traffic of the road joining it to give way.");

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
	ImGui::RadioButton("Place", &tool.tool, 2);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", "Level crossings, and the points where vehicles appear on the roads or are taken off them");
	if (tool.tool != previoustool)
	{
		road_cancel();
		if (tool.tool != 0)
		{
			tool.junction = nullptr;
			tool.points.clear();
			tool.marker = nullptr;
			// the fields for the names of materials show the layout for the roads to be built again
			road_select(nullptr);
		}
	}
	ImGui::SameLine();
	ImGui::Checkbox("Show lanes", &tool.lanes);

	// typed values are taken when confirmed, so nothing is made anew with each digit
	auto const number = [](char const *Label, float &Value, float const Step, char const *Format) {
		ImGui::SetNextItemWidth(120.0f);
		return ImGui::InputFloat(Label, &Value, Step, Step * 4.0f, Format, ImGuiInputTextFlags_EnterReturnsTrue);
	};
	if (tool.tool == 1)
	{
		if (ImGui::Checkbox("Roundabout", &tool.ring))
			road_cancel();
		if (tool.ring)
		{
			ImGui::TextDisabled("LMB: put a roundabout around the cursor. It's a one-way road going around,");
			ImGui::TextDisabled("with the lanes set below as the lanes along the axis.");
			ImGui::TextDisabled("Then untick this and lead the roads out of its side, or into it.");
			ImGui::SetNextItemWidth(120.0f);
			if (ImGui::InputFloat("Radius of the middle of its road [m]", &tool.ringradius, 1.0f, 5.0f, "%.1f"))
				tool.ringradius = std::clamp(tool.ringradius, 8.0f, 200.0f);
			render_road_material("Island in the middle", tool.islandtext, sizeof(tool.islandtext), tool.island);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", "What the middle is covered with; it's raised on a kerb as high as the kerbs of the road.\nnone: the middle is left as it is.");
		}
		else
		{
			ImGui::TextDisabled("LMB: start, then each next point. Ctrl: straight ahead. Esc: finish");
			ImGui::TextDisabled("Start or end on a loose end of a road to join it.");
			ImGui::TextDisabled("Start or end on the side of a road to make a junction there,");
			ImGui::TextDisabled("or on a junction to give it another road.");
		}
		if (tool.chain)
		{
			if (ImGui::Button("Finish (Esc)"))
				road_cancel();
		}
		// how the road is put on the ground
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::InputFloat("Height above the ground [m]", &tool.offset, 0.05f, 0.25f, "%.2f"))
			tool.offset = std::clamp(tool.offset, -10.0f, 100.0f);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "How far over the ground under the cursor the points are put.\nA few centimetres keep the surface from flickering where it lies on the ground.");
		ImGui::Checkbox("Follow the ground", &tool.follow);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "On: the road is led over the ground between the clicked points, in pieces as short as the ground calls for,\nwith the profile rounded so the slope doesn't change abruptly.\nOff: the road goes straight from a point to the next one, like a ramp or a viaduct.");
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::InputFloat("Speed limit on new junctions [km/h]", &tool.crossingspeed, 5.0f, 10.0f, "%.0f"))
			tool.crossingspeed = std::clamp(tool.crossingspeed, 5.0f, 200.0f);
		ImGui::Separator();
		// what's set here is what the next pieces get
		render_road_layout(tool.settings);
	}
	else if (tool.tool == 2)
	{
		ImGui::RadioButton("Level crossing", &tool.placekind, 0);
		ImGui::SameLine();
		ImGui::RadioButton("Spawn point", &tool.placekind, 1);
		ImGui::SameLine();
		ImGui::RadioButton("Removal point", &tool.placekind, 2);
		ImGui::RadioButton("Pedestrian crossing", &tool.placekind, 3);
		tool.placekind = std::clamp(tool.placekind, 0, 3);
		switch (tool.placekind)
		{
		case 0:
			ImGui::TextDisabled("LMB on a road where the rails cross it. The vehicles stop ahead");
			ImGui::TextDisabled("of the rails for as long as a train is coming.");
			break;
		case 1:
			ImGui::TextDisabled("LMB on a lane. The vehicles of the set appear there one by one");
			ImGui::TextDisabled("and drive on the way the lane goes.");
			break;
		case 3:
			ImGui::TextDisabled("LMB on a road: stripes are painted across it there.");
			break;
		default:
			ImGui::TextDisabled("LMB on a lane. The vehicles which get there are taken off the road;");
			ImGui::TextDisabled("put one where the road leaves the scenery.");
			break;
		}
		ImGui::Separator();
		// what's set here is what the next points get
		auto &placing{tool.placing[tool.placekind]};
		placing.kind = point_kind(tool.placekind);
		render_roadpoint_layout(placing, nullptr);
	}
	else if (tool.marker != nullptr)
	{
		auto &point{*tool.marker};
		auto state{point.definition()};
		ImGui::Text("%s, %s", point.name().c_str(), point_label(state.kind));
		if (render_roadpoint_layout(state, &point))
			roadpoint_apply(point, state);
		if (ImGui::Button("Delete (Del)"))
			road_delete();
	}
	else if (false == tool.points.empty())
	{
		// the road at the selected points
		ImGui::Text("%d %s selected", static_cast<int>(tool.points.size()), tool.points.size() == 1 ? "point" : "points");
		ImGui::TextDisabled("Shift+LMB: add a point, or take it out. K on a selected piece: make a point at the cursor");
		auto const ends{editor_road::ends_at(tool.points.front())};
		if (false == ends.empty())
		{
			auto const &layout{ends.front().road->definition()};
			auto width{static_cast<float>(layout.width(ends.front().atend ? 1 : 0) * layout.taper[ends.front().atend ? 1 : 0])};
			if (number("Road width here [m]", width, 0.25f, "%.2f"))
				road_point_width(std::clamp(width, 1.0f, 100.0f));
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", "Combined width of the lanes at this point; each lane takes its share of it.\nThe pieces on both sides get wider or narrower along their length to meet it.\nSelect several points to keep the width over the pieces between them.");
			// heights: set outright, moved by a step, or taken from the ground
			auto height{static_cast<float>(tool.points.front().y)};
			if (number("Height [m]", height, 0.1f, "%.2f"))
				road_point_heights(std::vector<double>(tool.points.size(), height));
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", "Height of the axis of the road at the point; with several points selected all are put at this height.\nThe slopes of the pieces at the point are kept, so the road stays smooth.");
			std::vector<double> heights;
			for (auto const &point : tool.points)
				heights.emplace_back(point.y);
			auto const moved{ImGui::Button("Lower") ? -1.0 : 0.0};
			ImGui::SameLine();
			auto const raised{ImGui::Button("Raise") ? 1.0 : 0.0};
			ImGui::SameLine();
			ImGui::SetNextItemWidth(80.0f);
			if (ImGui::InputFloat("by [m]", &tool.heightstep, 0.0f, 0.0f, "%.2f"))
				tool.heightstep = std::clamp(tool.heightstep, 0.01f, 50.0f);
			if (moved + raised != 0.0)
			{
				// each point moves from where it is, so the shape of the road between them is kept
				for (auto &entry : heights)
					entry += (moved + raised) * tool.heightstep;
				road_point_heights(heights);
			}
			if (ImGui::Button("Put on the ground"))
			{
				// a point with nothing under it comes back with the height it has, and is left where it is
				auto const ground{ground_heights(tool.points, true)};
				for (std::size_t index = 0; index < heights.size(); ++index)
					if (ground[index] != tool.points[index].y)
						heights[index] = ground[index] + tool.offset;
				road_point_heights(heights);
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Puts each point %.2f m over the ground under it; the height above the ground is set in the Build tool.", tool.offset);
		}
		if (ImGui::Button(tool.points.size() == 1 ? "Delete the point (Del)" : "Delete the points (Del)"))
			road_points_delete();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "Takes the point out of the road: the pieces on its two sides become one piece,\n"
			                        "led from the far end of one to the far end of the other the way the road goes there.\n"
			                        "A point the road ends at takes the last piece of the road with it.");
	}
	else if (tool.junction != nullptr)
	{
		auto &junction{*tool.junction};
		if (junction.definition().arms.size() == 2)
		{
			ImGui::Text("%s, a stretch where the road changes its lanes", junction.name().c_str());
			ImGui::TextDisabled("It's drawn as a part of the road, and leads the lanes of one side to the lanes of the other.");
		}
		else
		{
			ImGui::Text("%s, %d roads, %d ways through", junction.name().c_str(), static_cast<int>(junction.definition().arms.size()), static_cast<int>(junction.movements().size()));
			ImGui::TextDisabled("The ways through are made for the roads attached at the moment.");
			ImGui::TextDisabled("Build tool, LMB on the junction: lead another road out of it");
		}
		render_junction_layout(junction);
		if (ImGui::Button("Delete (Del)"))
			road_delete();
	}
	else if (tool.selected != nullptr)
	{
		auto const &road{*tool.selected};
		auto const &layout{road.definition()};
		if (layout.changing())
			ImGui::Text("%s, %.1f m, %d + %d to %d + %d lanes", road.name().c_str(), layout.length(), layout.lanes_at(0, true), layout.lanes_at(0, false), layout.lanes_at(1, true),
			            layout.lanes_at(1, false));
		else
			ImGui::Text("%s, %.1f m, %d + %d lanes", road.name().c_str(), layout.length(), layout.forward, layout.backward);
		if (ImGui::Button("Delete (Del)"))
			road_delete();
		ImGui::SameLine();
		ImGui::TextDisabled("K: split at the cursor");
		if (tool.settings.changing())
			ImGui::TextDisabled("This piece has lanes of its own at its end, so it's changed alone");
		else
			ImGui::Checkbox("Changes go to the whole road", &tool.whole);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", "A road is a string of pieces. With this on a change of the layout is made to all of them,\nexcept for the lane change rules, which are always set for the selected piece alone.\nWith this off the selected piece alone is changed, and the road next to it is fitted to it:\nmade wider or narrower towards it, or given a stretch where the lanes change if their number differs.\nSplit a piece (K) to get a piece of the length you need.");
		ImGui::Separator();
		if (tool.selected != nullptr && render_road_layout(tool.settings))
			road_apply();
	}
	else
	{
		ImGui::TextDisabled("LMB: select a road piece, a junction, a point where pieces meet,");
		ImGui::TextDisabled("a level crossing or a traffic point");
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

	screen_projection projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList();
	glm::dvec3 const camera{Global.pCamera.Pos};
	// pieces of a line needed to draw a path: one for a straight, more the longer a bend is and the nearer it is
	auto const pieces = [&camera](segment_data const &Path, glm::dvec3 const &Location) {
		auto const range{glm::distance(Location, camera)};
		auto const length{glm::distance(Path.points[segment_data::point::start], Path.points[segment_data::point::end])};
		return std::clamp(static_cast<int>(length / (range < 100.0 ? 3.0 : range < 300.0 ? 8.0 : 20.0)), 2, 12);
	};
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
		projection.limit = screen_projection::kLaneVertexLimit;
		for (auto const *road : simulation::Roads.sequence())
		{
			if (road == nullptr || road->m_editorremoved)
				continue;
			auto const range{glm::distance(road->location(), camera)};
			if (range > kLaneRange + 0.5 * glm::distance(road->definition().axis.points[segment_data::point::start], road->definition().axis.points[segment_data::point::end]))
				continue;
			if (false == projection.room(drawlist))
				break;
			auto const against{static_cast<std::size_t>(road->definition().backward)};
			// a lane of a piece which gains or loses lanes is a path at each end of the piece; the ways between these are drawn below
			auto const lanecount{road->definition().lanes.size()};
			for (std::size_t slot = 0; slot < 2 * lanecount; ++slot)
			{
				auto const lane{slot / 2};
				auto const *track{road->lane_track(lane, static_cast<int>(slot % 2))};
				if (track == nullptr || track->m_paths.empty() || (slot % 2 == 1 && track == road->lane_track(lane, 0)))
					continue;
				auto const &path{track->m_paths.front()};
				auto const color{lane >= against ? kForwardColor : kBackwardColor};
				drawpath(path, color, 2.0f, is_curved(path) ? pieces(path, road->location()) : 1);
				if (range > 300.0 || road->definition().changing())
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
				if (turn != nullptr && false == turn->m_paths.empty() && range < 300.0)
					drawpath(turn->m_paths.front(), kTurnColor, 1.5f, 8);
			}
			// ways from the lanes at one end to the lanes at the other
			for (auto const &way : road->ways())
				drawpath(way, kWayColor, 1.5f, pieces(way, road->location()));
		}
		// ways through the junctions
		for (auto const *junction : simulation::Junctions.sequence())
		{
			if (junction == nullptr || junction->m_editorremoved || glm::distance(junction->location(), camera) > kLaneRange)
				continue;
			for (auto const &way : junction->movements())
				drawpath(way, kWayColor, 1.5f, pieces(way, junction->location()));
		}
		projection.limit = screen_projection::kVertexLimit;
	}
	if (false == tool.window)
		return;

	// outline of a road piece: its axis and the edges of its surface
	auto const drawroad = [&](road_node::state const &State, ImU32 const Color) {
		auto const samples{is_curved(State.axis) || State.taper[0] != State.taper[1] ? pieces(State.axis, State.axis.points[segment_data::point::start]) : 1};
		drawpath(State.axis, Color, 3.0f, samples);
		glm::dvec3 previous[2];
		for (int i = 0; i <= samples; ++i)
		{
			auto const t{static_cast<double>(i) / samples};
			auto const position{path_point(State.axis, t)};
			auto const tangent{path_direction(State.axis, t)};
			glm::dvec3 across{tangent.z, 0.0, -tangent.x};
			if (glm::length(across) > 1e-6)
				across = glm::normalize(across);
			auto const halfwidth{0.5 * State.span(t)};
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
		if (false == projection.room(drawlist) || false == projection.project(Point, screen) || projection.outside(screen, screen))
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

	// a circle lying on the ground
	auto const drawring = [&](glm::dvec3 const &Centre, double const Radius, ImU32 const Color) {
		int const pieces{24};
		glm::dvec3 previous{Centre + glm::dvec3{Radius, 0.0, 0.0}};
		for (int i = 1; i <= pieces; ++i)
		{
			auto const angle{kFullTurn * i / pieces};
			glm::dvec3 const next{Centre + glm::dvec3{Radius * std::cos(angle), 0.0, Radius * std::sin(angle)}};
			projection.line(drawlist, previous, next, Color, 2.0f);
			previous = next;
		}
	};
	auto const drawlabel = [&](glm::dvec3 const &Point, ImU32 const Color, char const *Text) {
		ImVec2 screen;
		if (false == projection.room(drawlist) || false == projection.project(Point, screen) || projection.outside(screen, screen))
			return;
		drawlist->AddText(ImVec2(screen.x + 14.0f, screen.y - 8.0f), Color, Text);
	};
	// a level crossing or a traffic point. Point: the node it's drawn for, nullptr for one which isn't made yet
	auto const drawmarker = [&](roadpoint_node::state const &State, roadpoint_node const *Point, ImU32 const Color) {
		drawpoint(State.position, Color, 6.0f, true);
		auto const labelled{glm::distance(State.position, camera) < 200.0};
		switch (State.kind)
		{
		case roadpoint_node::kind_type::crossing:
		{
			// the tracks within the circle are the ones it's closed for, the lines are where the vehicles stop
			drawring(State.position, State.clearance, Color);
			auto const closed{Point != nullptr && Point->closed()};
			if (Point != nullptr)
			{
				for (auto const &stop : Point->stops())
				{
					glm::dvec3 across{stop.direction.z, 0.0, -stop.direction.x};
					if (glm::length(across) < 1e-6)
						continue;
					across = glm::normalize(across) * (0.5 * stop.width);
					projection.line(drawlist, stop.position - across, stop.position + across, closed ? kRefusedColor : Color, 4.0f);
				}
			}
			if (labelled)
				drawlabel(State.position, Color, closed ? "crossing, closed" : "crossing");
			break;
		}
		case roadpoint_node::kind_type::spawn:
		{
			drawpoint(State.position, Color, 11.0f, false);
			if (labelled)
				drawlabel(State.position, Color, "spawn");
			break;
		}
		case roadpoint_node::kind_type::despawn:
		{
			drawring(State.position, State.radius, Color);
			if (labelled)
				drawlabel(State.position, Color, "removal");
			break;
		}
		case roadpoint_node::kind_type::crosswalk:
		{
			drawpoint(State.position, Color, 11.0f, false);
			if (labelled)
				drawlabel(State.position, Color, "pedestrians");
			break;
		}
		}
	};
	for (auto const *point : simulation::Roadpoints.sequence())
	{
		if (point == nullptr || point->m_editorremoved || glm::distance(point->location(), camera) > kPointRange)
			continue;
		auto const kind{point->definition().kind};
		drawmarker(point->definition(), point,
		           point == tool.marker                         ? kSelectedColor :
		           kind == roadpoint_node::kind_type::spawn     ? kSpawnColor :
		           kind == roadpoint_node::kind_type::despawn   ? kDespawnColor :
		           kind == roadpoint_node::kind_type::crosswalk ? kCrosswalkColor :
		                                                          kCrossingColor);
		if (point == tool.marker)
			drawpoint(point->definition().position, kSelectedColor, 15.0f, false);
	}

	if (tool.selected != nullptr && false == tool.selected->m_editorremoved)
	{
		auto const &state{tool.selected->definition()};
		drawroad(state, kSelectedColor);
		drawpoint(state.axis.points[segment_data::point::start], kSelectedColor, 6.0f, true);
		drawpoint(state.axis.points[segment_data::point::end], kSelectedColor, 6.0f, true);
	}
	if (tool.junction != nullptr && false == tool.junction->m_editorremoved)
	{
		drawjunction(tool.junction->definition(), kSelectedColor);
		// the roads are numbered the way the window lists them
		auto const &arms{tool.junction->definition().arms};
		for (std::size_t arm = 0; arm < arms.size(); ++arm)
			drawlabel(arms[arm].position, kSelectedColor, std::to_string(arm + 1).c_str());
	}

	std::string hint;
	if (tool.tool == 1 && tool.ring)
	{
		// the road going around, as wide as its lanes
		auto layout{tool.settings};
		layout.backward = 0;
		layout.forward = std::max(1, layout.forward);
		layout.lanes.clear();
		layout.normalize();
		auto const centre{tool.mouse + glm::dvec3{0.0, tool.offset, 0.0}};
		drawring(centre, tool.ringradius + 0.5 * layout.width(), kPreviewColor);
		drawring(centre, std::max(0.0, tool.ringradius - 0.5 * layout.width()), kPreviewColor);
		hint = "LMB: put a roundabout here";
	}
	else if (tool.tool == 1)
	{
		if (tool.chain)
		{
			auto const &plan{tool.plan};
			drawpoint(tool.point, kPreviewColor, 6.0f, true);
			if (plan.pieces.empty())
			{
				projection.line(drawlist, tool.point, tool.mouse, kRefusedColor, 2.0f);
				hint = plan.error + "   ";
			}
			else
			{
				// junctions the road would start or end with
				if (plan.hasstart)
					drawjunction(plan.start.layout, kPreviewColor);
				if (plan.hasend)
					drawjunction(plan.end.layout, kPreviewColor);
			}
			for (auto const &piece : plan.pieces)
			{
				auto state{tool.settings};
				state.axis = piece;
				drawroad(state, kPreviewColor);
			}
			if (plan.snap.valid())
				drawpoint(plan.snap.position, kSnapColor, 14.0f, false);
			hint += (tool.branchroad != nullptr ? "LMB: lead the road out to the cursor, with a junction where it leaves   Esc: give up" :
			         tool.branchjunction != nullptr ? "LMB: lead the road out to the cursor, as another road of the junction   Esc: give up" :
			                                          "LMB: build up to the cursor   Ctrl: straight ahead   Esc: finish   Ctrl+Z: undo the last piece");
		}
		else
		{
			auto const end{editor_road::find_end(tool.mouse, kSnapRadius)};
			if (end.valid())
			{
				drawpoint(end.position, kSnapColor, 14.0f, false);
				hint = "LMB: carry on from this end";
			}
			else if (tool.hashover)
			{
				drawpoint(tool.hover, kSnapColor, 5.0f, true);
				drawpoint(tool.hover, kSnapColor, 14.0f, false);
				hint = "LMB: lead a road out of here, with a junction";
			}
			else
			{
				hint = "LMB: set where the road starts; on a loose end of a road to carry it on, on the side of a road or on a junction to lead a road out of it";
			}
		}
	}
	else if (tool.tool == 2)
	{
		if (tool.hastarget)
		{
			drawmarker(tool.target, nullptr, kPreviewColor);
			if (glm::length(tool.targetdirection) > 1e-6)
			{
				// the way the traffic goes on the lane
				auto const direction{glm::normalize(tool.targetdirection)};
				glm::dvec3 const across{direction.z, 0.0, -direction.x};
				auto const tip{tool.target.position + direction * 5.0};
				projection.line(drawlist, tool.target.position, tip, kPreviewColor, 2.0f);
				projection.line(drawlist, tip, tip - direction * 1.2 + across * 0.8, kPreviewColor, 2.0f);
				projection.line(drawlist, tip, tip - direction * 1.2 - across * 0.8, kPreviewColor, 2.0f);
			}
			hint = std::string{"LMB: put a "} + point_label(tool.target.kind) + " here";
			if (false == tool.targetnote.empty())
				hint += "   " + tool.targetnote;
		}
		else
		{
			hint = (tool.targetnote.empty() ? std::string{"LMB: put a "} + point_label(point_kind(tool.placekind)) + " on a road" : tool.targetnote);
		}
	}
	else
	{
		// points where the pieces meet or end, to pick from
		for (auto const *road : simulation::Roads.sequence())
		{
			if (road == nullptr || road->m_editorremoved || glm::distance(road->location(), camera) > kPointRange)
				continue;
			drawpoint(road->definition().axis.points[segment_data::point::start], kPointColor, 3.5f, true);
			drawpoint(road->definition().axis.points[segment_data::point::end], kPointColor, 3.5f, true);
		}
		for (auto const &point : tool.points)
		{
			drawpoint(point, kSelectedColor, 6.0f, true);
			drawpoint(point, kSelectedColor, 11.0f, false);
		}
		hint = tool.marker != nullptr       ? "LMB: select a piece, a junction, a point or a marker   Del: delete   Ctrl+Z: undo" :
		       false == tool.points.empty() ? "LMB: select a point   Shift+LMB: add a point or take it out   Del: delete the points   Ctrl+Z: undo" :
		       tool.selected != nullptr     ? "LMB: select a piece, a junction or a point (white dot)   K: split at the cursor   Del: delete   Ctrl+Z: undo" :
		       tool.junction != nullptr     ? "LMB: select a piece, a junction or a point (white dot)   Del: delete   Ctrl+Z: undo" :
		                                      "LMB: select a piece, a junction, a point (white dot) or a marker";
	}
	// shown above the place the track tools put their hints at
	ImGuiIO const &io = ImGui::GetIO();
	ImVec2 const position{12.0f, io.DisplaySize.y - 56.0f};
	auto const size{ImGui::CalcTextSize(hint.c_str())};
	drawlist->AddRectFilled(ImVec2(position.x - 6.0f, position.y - 4.0f), ImVec2(position.x + size.x + 6.0f, position.y + size.y + 4.0f), IM_COL32(0, 0, 0, 150), 4.0f);
	drawlist->AddText(position, IM_COL32(255, 255, 255, 230), hint.c_str());
}
