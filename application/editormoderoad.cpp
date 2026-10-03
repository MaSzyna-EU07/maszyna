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

ImU32 const kForwardColor{IM_COL32(70, 230, 110, 230)};
ImU32 const kBackwardColor{IM_COL32(255, 170, 50, 230)};
ImU32 const kTurnColor{IM_COL32(210, 210, 210, 150)};
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
	if (false == tool.window)
	{
		tool.chain = false;
		return;
	}
	tool.mouse = Global.pCamera.Pos + GfxRenderer->Mouse_Position();
}

void editor_mode::road_select(road_node *Road)
{
	auto &tool{m_roadtool};
	tool.selected = Road;
	if (Road != nullptr)
	{
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

void editor_mode::road_cancel()
{
	m_roadtool.chain = false;
}

std::vector<segment_data> editor_mode::road_preview(editor_road::loose_end &Snap, std::string &Error) const
{
	auto const &tool{m_roadtool};
	Snap = editor_road::find_end(tool.mouse, kSnapRadius);
	// the end the road is being drawn from isn't somewhere to lead it to
	if (Snap.road != nullptr && glm::distance(Snap.position, tool.point) < 0.5)
		Snap = {};
	glm::dvec2 enddirection{0.0, 1.0};
	if (Snap.road != nullptr)
	{
		glm::dvec2 const outwards{Snap.outwards.x, Snap.outwards.z};
		if (glm::length(outwards) > 1e-6)
			enddirection = -glm::normalize(outwards);
		else
			Snap = {};
	}
	return editor_road::plan(tool.point, tool.hasdirection ? &tool.direction : nullptr, Snap.road != nullptr ? Snap.position : tool.mouse, Snap.road != nullptr ? &enddirection : nullptr, Global.ctrlState, Error);
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
		if (road == nullptr)
			tool.status = "No road here";
		road_select(road);
		return;
	}

	std::string reason;
	if (false == editor_road::available(&reason))
	{
		tool.status = reason;
		return;
	}
	if (false == tool.chain)
	{
		// first click sets where the road starts. started from a loose end of a road it carries that road on
		auto const end{editor_road::find_end(ground, kSnapRadius)};
		glm::dvec2 const outwards{end.outwards.x, end.outwards.z};
		if (end.road != nullptr && glm::length(outwards) > 1e-6)
		{
			tool.point = end.position;
			tool.direction = glm::normalize(outwards);
			tool.hasdirection = true;
			// carried on from its start the road keeps the direction of its axis, so the lanes stay what they were
			tool.reversed = (false == end.atend);
			auto const layout{end.road->definition()};
			tool.selected = nullptr;
			tool.settings = layout;
			road_select(nullptr);
			tool.status = "Carrying on \"" + end.road->name() + "\" with its layout";
		}
		else
		{
			tool.point = ground;
			tool.hasdirection = false;
			tool.reversed = false;
			tool.selected = nullptr;
			tool.status = "Start set";
		}
		tool.chain = true;
		return;
	}

	editor_road::loose_end snap;
	std::string error;
	auto pieces{road_preview(snap, error)};
	if (pieces.empty())
	{
		tool.status = "Can't build this: " + error;
		return;
	}
	// where the next piece is going to carry on from
	auto const &last{pieces.back()};
	auto const endpoint{last.points[segment_data::point::end]};
	auto const tangent{last.points[segment_data::point::control2] != glm::dvec3{0.0} ? -last.points[segment_data::point::control2] : last.points[segment_data::point::end] - last.points[segment_data::point::start]};
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
	if (snap.road != nullptr)
	{
		tool.chain = false;
		tool.status = "Joined with \"" + snap.road->name() + "\"";
		return;
	}
	glm::dvec2 const planar{tangent.x, tangent.z};
	tool.point = endpoint;
	if (glm::length(planar) > 1e-6)
	{
		tool.direction = glm::normalize(planar);
		tool.hasdirection = true;
	}
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
	if (false == tool.window || tool.selected == nullptr)
		return false;
	auto *road{tool.selected};
	std::string reason;
	if (false == editor_road::can_edit(*road, &reason))
	{
		tool.status = "Can't delete: " + reason;
		return true;
	}
	editor_road::remove({road});
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

void editor_mode::push_road_snapshot(std::vector<std::pair<road_node *, road_node::state>> States, std::vector<road_node *> Created, std::vector<road_node *> Removed)
{
	if (States.empty() && Created.empty() && Removed.empty())
		return;

	if (m_max_history_size >= 0 && (int)m_history.size() >= m_max_history_size)
		m_history.erase(m_history.begin(), m_history.begin() + ((int)m_history.size() - m_max_history_size + 1));

	auto const *road = false == States.empty() ? States.front().first : false == Created.empty() ? Created.front() : Removed.front();
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::RoadEdit;
	snap.node_name = road->name();
	snap.position = road->location();
	snap.roads = std::move(States);
	snap.roads_created = std::move(Created);
	snap.roads_removed = std::move(Removed);
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::restore_road_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo)
{
	if (Snapshot.roads.empty() && Snapshot.roads_created.empty() && Snapshot.roads_removed.empty())
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
		editor_road::revive(Snapshot.roads_removed);
	}
	else
	{
		editor_road::revive(Snapshot.roads_created);
		editor_road::remove(Snapshot.roads_removed);
	}
	editor_road::apply(Snapshot.roads);

	auto &tool{m_roadtool};
	tool.chain = false;
	if (tool.selected != nullptr)
		road_select(tool.selected->m_editorremoved ? nullptr : tool.selected);
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
	if (tool.tool != previoustool)
	{
		tool.chain = false;
		if (tool.tool == 1)
			tool.selected = nullptr;
	}
	ImGui::SameLine();
	ImGui::Checkbox("Show lanes", &tool.lanes);

	if (tool.tool == 1)
	{
		ImGui::TextDisabled("LMB: start, then each next point. Ctrl: straight ahead. Esc: finish");
		ImGui::TextDisabled("Start or end on a loose end of a road to join it");
		if (tool.chain)
		{
			if (ImGui::Button("Finish (Esc)"))
				tool.chain = false;
		}
		ImGui::Separator();
		// what's set here is what the next pieces get
		render_road_layout(tool.settings);
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
		ImGui::TextDisabled("LMB: select a road piece");
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

	if (tool.selected != nullptr && false == tool.selected->m_editorremoved)
	{
		auto const &state{tool.selected->definition()};
		drawroad(state, kSelectedColor);
		drawpoint(state.axis.points[segment_data::point::start], kSelectedColor, 6.0f, true);
		drawpoint(state.axis.points[segment_data::point::end], kSelectedColor, 6.0f, true);
	}

	std::string hint;
	if (tool.tool == 1)
	{
		if (tool.chain)
		{
			editor_road::loose_end snap;
			std::string error;
			auto const pieces{road_preview(snap, error)};
			drawpoint(tool.point, kPreviewColor, 6.0f, true);
			if (pieces.empty())
			{
				projection.line(drawlist, tool.point, tool.mouse, kRefusedColor, 2.0f);
				hint = error + "   ";
			}
			for (auto const &piece : pieces)
			{
				auto state{tool.settings};
				state.axis = piece;
				drawroad(state, kPreviewColor);
			}
			if (snap.road != nullptr)
				drawpoint(snap.position, kSnapColor, 14.0f, false);
			hint += "LMB: build up to the cursor   Ctrl: straight ahead   Esc: finish   Ctrl+Z: undo the last piece";
		}
		else
		{
			auto const end{editor_road::find_end(tool.mouse, kSnapRadius)};
			if (end.road != nullptr)
				drawpoint(end.position, kSnapColor, 14.0f, false);
			hint = "LMB: set where the road starts; on a loose end of a road (ring) to carry it on";
		}
	}
	else
	{
		hint = tool.selected != nullptr ? "LMB: select a road piece   K: split at the cursor   Del: delete   Ctrl+Z: undo" : "LMB: select a road piece";
	}
	// shown above the place the track tools put their hints at
	ImGuiIO const &io = ImGui::GetIO();
	ImVec2 const position{12.0f, io.DisplaySize.y - 56.0f};
	auto const size{ImGui::CalcTextSize(hint.c_str())};
	drawlist->AddRectFilled(ImVec2(position.x - 6.0f, position.y - 4.0f), ImVec2(position.x + size.x + 6.0f, position.y + size.y + 4.0f), IM_COL32(0, 0, 0, 150), 4.0f);
	drawlist->AddText(position, IM_COL32(255, 255, 255, 230), hint.c_str());
}
