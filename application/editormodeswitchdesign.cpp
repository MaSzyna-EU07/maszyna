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
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorSwitchDesign.hpp"

#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <format>


namespace
{

enum class part_kind
{
	straight,
	arc,
	transition
};

part_kind kind_of(switch_design::part const &Part)
{
	using enum part_kind;
	if (Part.radius1 == 0.0 && Part.radius2 == 0.0)
		return straight;
	return Part.radius1 == Part.radius2 ? arc : transition;
}

// the radius a part keeps as its kind changes
double kept_radius(switch_design::part const &Part)
{
	if (Part.radius2 != 0.0)
		return Part.radius2;
	return Part.radius1 != 0.0 ? Part.radius1 : 300.0;
}

void set_kind(switch_design::part &Part, part_kind const Kind)
{
	using enum part_kind;
	auto const radius{kept_radius(Part)};
	switch (Kind)
	{
	case straight:
		Part.radius1 = 0.0;
		Part.radius2 = 0.0;
		break;
	case arc:
		Part.radius1 = radius;
		Part.radius2 = radius;
		break;
	case transition:
		Part.radius1 = 0.0;
		Part.radius2 = radius;
		break;
	}
}

// which value of the template the mouse is over, for the drawing to show it
namespace value
{
int constexpr radius{1};
int constexpr ratio{2};
int constexpr a{3};
int constexpr b{4};
int constexpr lead{5};
int constexpr entry{6};
int constexpr tip{7};
int constexpr heel{8};
int constexpr blade_throw{9};
int constexpr main_track{10};
int constexpr stagger{11};
int constexpr part{100};
}

void mark(int &Pointing, int const Value)
{
	if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) || ImGui::IsItemActive())
		Pointing = Value;
}

// the radii of a part of the kind, in the row of the part; Value: of the part, for the drawing to show it. True when one changed
bool edit_part_radii(switch_design::part &Part, part_kind const Kind, int &Pointing, int const Value)
{
	bool changed{false};
	switch (Kind)
	{
	case part_kind::arc:
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		if (ImGui::InputDouble(STR_C("R##radius"), &Part.radius1, 0.0, 0.0, "%.1f"))
		{
			Part.radius2 = Part.radius1;
			changed = true;
		}
		mark(Pointing, Value);
		item_tooltip("Radius of the arc; negative turns back towards the main track");
		if (Part.radius1 != 0.0)
		{
			ImGui::SameLine();
			ImGui::TextDisabled("%.3f°", glm::degrees(Part.length / Part.radius1));
		}
		break;
	case part_kind::transition:
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		changed |= ImGui::InputDouble(STR_C("R from##radius1"), &Part.radius1, 0.0, 0.0, "%.1f");
		item_tooltip("Radius at the start of the transition curve, 0: from a straight");
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		changed |= ImGui::InputDouble(STR_C("to##radius2"), &Part.radius2, 0.0, 0.0, "%.1f");
		item_tooltip("Radius at the end of the transition curve, 0: into a straight; the curvature changes evenly along it");
		break;
	default:
		break;
	}
	return changed;
}

// the buttons moving the part I up and removing it, at the end of its row
void part_buttons(int const I, int &Up, int &Erase)
{
	ImGui::SameLine();
	if (ImGui::SmallButton("^") && I > 0)
		Up = I;
	item_tooltip("Move the part up");
	ImGui::SameLine();
	if (ImGui::SmallButton("x"))
		Erase = I;
	item_tooltip("Remove the part");
}

// the buttons adding a part at the end of the diverging track; true when one was added
bool add_part_buttons(std::vector<switch_design::part> &Parts)
{
	bool changed{false};
	auto const radius{Parts.empty() ? 300.0 : std::max({std::abs(Parts.back().radius1), std::abs(Parts.back().radius2), 0.0})};
	if (ImGui::SmallButton(STR_C("+ Straight")))
	{
		Parts.emplace_back(5.0, 0.0, 0.0);
		changed = true;
	}
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("+ Arc")))
	{
		auto const r{radius > 0.0 ? radius : 300.0};
		Parts.emplace_back(20.0, r, r);
		changed = true;
	}
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("+ Transition curve")))
	{
		Parts.emplace_back(10.0, 0.0, radius > 0.0 ? radius : 300.0);
		changed = true;
	}
	return changed;
}

// the parts of the diverging track as a table; true when something changed
bool edit_parts(std::vector<switch_design::part> &Parts, char const *Id, bool const Editable, int &Pointing, int const Pointed)
{
	bool changed{false};
	std::array<char const *, 3> const kinds{STR_C("Straight"), STR_C("Arc"), STR_C("Transition")};
	ImGui::PushID(Id);
	if (false == Editable)
		ImGui::BeginDisabled();
	int erase{-1};
	int up{-1};
	for (int i = 0; i < static_cast<int>(Parts.size()); ++i)
	{
		auto &part{Parts[i]};
		ImGui::PushID(i);
		ImGui::AlignTextToFramePadding();
		ImGui::Text("%d.", i + 1);
		mark(Pointing, Pointed + i);
		ImGui::SameLine();
		auto kind{static_cast<int>(kind_of(part))};
		ImGui::SetNextItemWidth(130.0f);
		if (ImGui::Combo("##kind", &kind, kinds.data(), static_cast<int>(kinds.size())))
		{
			set_kind(part, static_cast<part_kind>(kind));
			changed = true;
		}
		mark(Pointing, Pointed + i);
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		changed |= ImGui::InputDouble(STR_C("m##length"), &part.length, 0.0, 0.0, "%.3f");
		mark(Pointing, Pointed + i);
		item_tooltip("Length of the part, measured along it");
		changed |= edit_part_radii(part, static_cast<part_kind>(kind), Pointing, Pointed + i);
		if (Editable)
			part_buttons(i, up, erase);
		ImGui::PopID();
	}
	if (up > 0)
	{
		std::swap(Parts[up], Parts[up - 1]);
		changed = true;
	}
	if (erase >= 0)
	{
		Parts.erase(Parts.begin() + erase);
		changed = true;
	}
	if (Editable)
		changed |= add_part_buttons(Parts);
	if (false == Editable)
		ImGui::EndDisabled();
	ImGui::PopID();
	return changed;
}

float constexpr kRow{17.0f};
float constexpr kMargin{14.0f};
ImU32 constexpr kActive{IM_COL32(255, 225, 60, 255)};
ImU32 constexpr kPlain{IM_COL32(200, 205, 215, 235)};

// the drawing of a switch in its own space, the main track along the width of the box: where its points go, its labels and its
// dimensions, the one of the value pointed at in yellow
struct schematic
{
	ImDrawList *drawlist{nullptr};
	ImVec2 origin;
	float width{0.0f};
	float scale{1.0f};
	float base{0.0f}; // of the main track, on the screen
	float stretch{1.0f}; // of the offsets from the main track
	int pointed{-1};

	ImVec2 screen(glm::dvec3 const &Point) const { return ImVec2{origin.x + kMargin + static_cast<float>(Point.z) * scale, base - static_cast<float>(Point.x) * scale * stretch}; }
	ImU32 shade(int const Value) const { return pointed == Value ? kActive : kPlain; }
	void label(ImVec2 const &At, std::string const &Text, ImU32 const Colour) const
	{
		auto const size{ImGui::CalcTextSize(Text.c_str())};
		ImVec2 const corner{std::clamp(At.x - size.x * 0.5f, origin.x + 2.0f, std::max(origin.x + 2.0f, origin.x + width - size.x - 12.0f)), At.y - size.y * 0.5f};
		drawlist->AddRectFilled({corner.x - 2.0f, corner.y}, {corner.x + size.x + 2.0f, corner.y + size.y}, IM_COL32(18, 22, 28, 235), 2.0f);
		drawlist->AddText(corner, Colour, Text.c_str());
	}
	// a dimension along the main track from First to Second, in the row under it
	void dimension(double const First, double const Second, int const Row, std::string const &Text, int const Value) const
	{
		auto const y{base + 16.0f + Row * kRow};
		auto const first_x{screen({0.0, 0.0, First}).x};
		auto const second_x{screen({0.0, 0.0, Second}).x};
		auto const colour{shade(Value)};
		drawlist->AddLine({first_x, y}, {second_x, y}, colour, pointed == Value ? 2.0f : 1.0f);
		for (auto const x : {first_x, second_x})
		{
			drawlist->AddLine({x, y - 3.0f}, {x, y + 3.0f}, colour, 1.0f);
			drawlist->AddLine({x, base + 2.0f}, {x, y}, pointed == Value ? kActive : IM_COL32(200, 205, 215, 45), 1.0f);
		}
		label({(first_x + second_x) * 0.5f, y}, Text, colour);
	}
};

// the diverging track sampled along its length, for the parts of it
struct schematic_sample
{
	ImVec2 at;
	double along;
};

// a piece of a path of the switch drawn, its samples kept in Samples unless it's nullptr; Along: the length along the path so far
void draw_schematic_piece(schematic const &View, segment_data const &Piece, ImU32 const Colour, double &Along, std::vector<schematic_sample> *Samples)
{
	geometry::bezier const curve{Piece};
	auto last{curve.point(0.0)};
	auto previous{View.screen(last)};
	if (Samples != nullptr && Samples->empty())
		Samples->push_back({previous, 0.0});
	for (int j = 1; j <= 24; ++j)
	{
		auto const point{curve.point(j / 24.0)};
		auto const next{View.screen(point)};
		View.drawlist->AddLine(previous, next, Colour, 2.0f);
		Along += glm::distance(point, last);
		if (Samples != nullptr)
			Samples->push_back({next, Along});
		last = point;
		previous = next;
	}
}

// the paths of the switch drawn; the samples along the diverging track
std::vector<schematic_sample> draw_schematic_paths(schematic const &View, switch_design::shape const &Shape)
{
	std::array<ImU32, 4> const colours{IM_COL32(200, 200, 200, 255), IM_COL32(90, 200, 255, 255), IM_COL32(200, 200, 200, 255), IM_COL32(255, 170, 60, 255)};
	std::vector<schematic_sample> diverging;
	for (std::size_t i = 0; i < Shape.paths.size(); ++i)
	{
		auto const &path{Shape.paths[i]};
		std::vector<segment_data> pieces{path.pieces};
		if (pieces.empty())
			pieces.push_back(path);
		auto along{0.0};
		for (auto const &piece : pieces)
			draw_schematic_piece(View, piece, colours[i % 4], along, i == 1 ? &diverging : nullptr);
	}
	return diverging;
}

// the first arc of the diverging track: where its middle lies along it, its radius and its part, -1 for none
struct schematic_arc
{
	double at{-1.0};
	double radius{0.0};
	int part{-1};
};

// the diverging track drawn thick from From to To along it
void draw_schematic_span(schematic const &View, std::vector<schematic_sample> const &Samples, double const From, double const To)
{
	for (std::size_t k = 1; k < Samples.size(); ++k)
		if (Samples[k].along >= From && Samples[k - 1].along <= To)
			View.drawlist->AddLine(Samples[k - 1].at, Samples[k].at, kActive, 4.0f);
}

// the part of the diverging track pointed at drawn thick; where the first arc lies
schematic_arc draw_schematic_parts(schematic const &View, switch_design::design const &Entry, std::vector<schematic_sample> const &Samples)
{
	schematic_arc first;
	auto from{Entry.lead};
	for (int i = 0; i < static_cast<int>(Entry.parts.size()); ++i)
	{
		auto const &part{Entry.parts[i]};
		if (first.part < 0 && part.radius1 != 0.0 && part.radius1 == part.radius2)
		{
			first.at = from + part.length * 0.5;
			first.radius = part.radius1;
			first.part = i;
		}
		if (View.pointed == value::part + i)
			draw_schematic_span(View, Samples, from, from + part.length);
		from += part.length;
	}
	return first;
}

// the radius of the first arc of the diverging track written by it
void draw_schematic_arc(schematic const &View, std::vector<schematic_sample> const &Samples, schematic_arc const &Arc, ImVec2 const &End)
{
	auto const near_arc{std::ranges::lower_bound(Samples, Arc.at, std::ranges::less{}, &schematic_sample::along)};
	auto const at{near_arc != Samples.end() ? near_arc->at : End};
	auto const active{View.pointed == value::radius || View.pointed == value::part + Arc.part};
	ImVec2 const text{at.x - 30.0f, at.y - 30.0f};
	View.drawlist->AddLine(at, text, active ? kActive : IM_COL32(200, 205, 215, 140), 1.0f);
	View.label(text, format("R %.0f", std::abs(Arc.radius)), active ? kActive : kPlain);
}

// the dimensions of the switch along the main track, in rows under it
void draw_schematic_dimensions(schematic const &View, switch_design::design const &Entry, switch_design::shape const &Shape)
{
	auto row{0};
	if (Shape.a > 0.0)
		View.dimension(0.0, Shape.a, row, format("a %.3f", Shape.a), value::a);
	if (Shape.b > 0.0)
		View.dimension(Shape.a, Shape.a + Shape.b, row, format("b %.3f", Shape.b), value::b);
	++row;
	if (Entry.lead > 0.0)
		View.dimension(0.0, Entry.lead, row++, format(STR_C("pre-blade %.3f"), Entry.lead), value::lead);
	if (Shape.tip > 0.0)
	{
		auto const at{View.screen({0.0, 0.0, Shape.tip})};
		View.drawlist->AddTriangleFilled({at.x, at.y}, {at.x - 4.0f, at.y + 8.0f}, {at.x + 4.0f, at.y + 8.0f}, IM_COL32(120, 255, 120, 220));
		View.dimension(0.0, Shape.tip, row++, format(STR_C("blade tip %.3f"), Shape.tip), value::tip);
		if (Entry.entry > 0.0 || View.pointed == value::entry)
			View.label({at.x, at.y - 22.0f}, format(STR_C("blade angle %.3f deg"), Entry.entry), View.shade(value::entry));
		if (View.pointed == value::blade_throw)
		{
			View.drawlist->AddLine({at.x, at.y - 18.0f}, {at.x, at.y}, kActive, 2.0f);
			View.label({at.x, at.y - 22.0f}, format(STR_C("blade throw %.3f"), Entry.bladethrow > 0.0 ? Entry.bladethrow : 0.160), kActive);
		}
	}
	if (Entry.heel > 0.0)
		View.dimension(0.0, Entry.heel, row++, format(STR_C("blade heel %.3f"), Entry.heel), value::heel);
	if (Shape.stagger > 0.0)
	{
		View.drawlist->AddCircleFilled(View.screen({0.0, 0.0, Shape.stagger}), 4.0f, IM_COL32(255, 170, 60, 255));
		View.dimension(0.0, Shape.stagger, row++, format(STR_C("second switch %.3f"), Shape.stagger), value::stagger);
	}
	View.dimension(0.0, Shape.length, row, format(STR_C("main track %.3f"), Shape.length), value::main_track);
}

// the switch in its own space with its dimensions, the one of the value pointed at in yellow: the main track along the width of the box
void draw_schematic(switch_design::design const &Entry, switch_design::shape const &Shape, int const Pointed, bool const Stretched)
{
	if (Shape.paths.empty() || Shape.length <= 0.0)
		return;
	auto const rows{2 + (Entry.lead > 0.0 ? 1 : 0) + (Shape.tip > 0.0 ? 1 : 0) + (Entry.heel > 0.0 ? 1 : 0) + (Shape.stagger > 0.0 ? 1 : 0)};
	auto const height{130.0f + rows * kRow};
	auto const width{ImGui::GetContentRegionAvail().x};
	ImGui::BeginChild("##switchshape", ImVec2(0.0f, height), true, ImGuiWindowFlags_NoScrollbar);
	auto *drawlist{ImGui::GetWindowDrawList()};
	auto const origin{ImGui::GetCursorScreenPos()};
	double reach{0.0};
	double across{0.0};
	for (auto const &path : Shape.paths)
	{
		reach = std::max(reach, path.points[segment_data::point::end].z);
		across = std::max(across, std::abs(path.points[segment_data::point::end].x));
	}
	auto const scale{(width - kMargin * 2.0f - 50.0f) / static_cast<float>(std::max(1.0, reach))};
	auto const base{origin.y + height - rows * kRow - 22.0f};
	auto const stretch{Stretched ? std::clamp((base - origin.y - 28.0f) / std::max(0.01f, static_cast<float>(across) * scale), 1.0f, 12.0f) : 1.0f};
	schematic const view{drawlist, origin, width, scale, base, stretch, Pointed};
	auto const diverging{draw_schematic_paths(view, Shape)};
	if (diverging.empty())
	{
		ImGui::EndChild();
		return;
	}
	auto const first_arc{draw_schematic_parts(view, Entry, diverging)};
	auto const start{view.screen({0.0, 0.0, 0.0})};
	auto const end{diverging.back().at};
	drawlist->AddCircleFilled(start, 4.0f, IM_COL32(255, 255, 255, 255));
	// the tangents meeting at the centre of the switch
	if (Shape.a > 0.0)
	{
		auto const centre{view.screen({0.0, 0.0, Shape.a})};
		drawlist->AddLine(centre, end, IM_COL32(255, 255, 255, 110), 1.0f);
		drawlist->AddCircle(centre, 4.0f, IM_COL32(255, 255, 255, 220), 0, 1.5f);
	}
	if (first_arc.at >= 0.0)
		draw_schematic_arc(view, diverging, first_arc, end);
	if (Shape.ratio > 0.0)
		view.label({end.x + 8.0f, end.y - 14.0f}, format("1:%.2f", Shape.ratio), view.shade(value::ratio));
	// how far the end of the diverging track lies from the main one
	{
		ImVec2 const foot{end.x, base};
		drawlist->AddLine(end, foot, IM_COL32(200, 205, 215, 90), 1.0f);
		view.label({end.x + 26.0f, (end.y + base) * 0.5f}, format("%.2f m", Shape.offset), IM_COL32(200, 205, 215, 170));
	}
	draw_schematic_dimensions(view, Entry, Shape);
	if (stretch > 1.01f)
		drawlist->AddText({origin.x + 4.0f, origin.y + 2.0f}, IM_COL32(255, 255, 255, 120), format(STR_C("across x%.1f"), stretch).c_str());
	ImGui::EndChild();
}

// the template shown laid along the clicks: the one measured so far while picking, then the one chosen on the list; nullptr for none
switch_design::design const *shown_design(bool const Picking, switch_design::design const &Measured, bool const MeasuredReady, std::vector<switch_design::design> const &List, int const Selected)
{
	if (Picking)
		return MeasuredReady ? &Measured : nullptr;
	if (Selected >= 0 && Selected < static_cast<int>(List.size()))
		return &List[Selected];
	return nullptr;
}

// where a point of a switch laid along the clicks lies: its main track from the first click towards the second one, the diverging
// track to the side of the third one
struct laid_place
{
	glm::dvec2 start;
	glm::dvec2 main;
	glm::dvec2 across;
	double height;

	glm::dvec3 at(glm::dvec3 const &Point) const
	{
		auto const plan{start + main * Point.z + across * Point.x};
		return glm::dvec3{plan.x, height, plan.y};
	}
};

// the paths of a switch laid in the view
void draw_laid_paths(screen_projection const &Projection, ImDrawList *Drawlist, laid_place const &Place, switch_design::shape const &Shape)
{
	std::array<ImU32, 4> const colours{IM_COL32(255, 255, 255, 240), IM_COL32(255, 90, 200, 255), IM_COL32(255, 255, 255, 240), IM_COL32(255, 170, 60, 255)};
	for (std::size_t i = 0; i < Shape.paths.size(); ++i)
	{
		std::vector<segment_data> pieces{Shape.paths[i].pieces};
		if (pieces.empty())
			pieces.push_back(Shape.paths[i]);
		for (auto const &piece : pieces)
		{
			geometry::bezier const curve{piece};
			auto previous{Place.at(curve.point(0.0))};
			for (int j = 1; j <= 16; ++j)
			{
				auto const next{Place.at(curve.point(j / 16.0))};
				Projection.line(Drawlist, previous, next, colours[i % 4], 2.0f);
				previous = next;
			}
		}
	}
}

// a point along the main track of a switch laid in the view, with its text
void mark_laid(screen_projection const &Projection, ImDrawList *Drawlist, laid_place const &Place, double const Along, std::string const &Text)
{
	ImVec2 at;
	if (false == Projection.project(Place.at({0.0, 0.0, Along}), at))
		return;
	Drawlist->AddCircle(at, 5.0f, IM_COL32(255, 255, 255, 240), 0, 1.5f);
	if (false == Text.empty())
		Drawlist->AddText({at.x + 7.0f, at.y + 4.0f}, IM_COL32(255, 255, 255, 240), Text.c_str());
}

// the switch of Shape laid in the view along the first three clicks
void draw_laid(screen_projection const &Projection, ImDrawList *Drawlist, std::vector<glm::dvec3> const &Clicks, switch_design::shape const &Shape)
{
	auto const way{geometry::plan_of(Clicks[1] - Clicks[0])};
	if (Shape.paths.empty() || false == (glm::dot(way, way) > 1e-6))
		return;
	auto const main{glm::normalize(way)};
	auto const beside{geometry::plan_of(Clicks[2] - Clicks[0])};
	auto const side{main.x * beside.y - main.y * beside.x < 0.0 ? -1.0 : 1.0};
	laid_place const place{geometry::plan_of(Clicks[0]), main, glm::dvec2{-main.y * side, main.x * side}, Clicks[0].y + 0.2};
	draw_laid_paths(Projection, Drawlist, place, Shape);
	if (Shape.a > 0.0)
		mark_laid(Projection, Drawlist, place, Shape.a, format("a %.2f", Shape.a));
	mark_laid(Projection, Drawlist, place, Shape.length, format(STR_C("length %.2f m"), Shape.length));
}

// a value of a template in a row of the fields table
bool input_row(char const *Label, char const *Id, double &Value, char const *Format)
{
	field_row(Label);
	return ImGui::InputDouble(Id, &Value, 0.0, 0.0, Format);
}

// the place of a switch type on the list of the types, the ordinary switch for any other
int type_index(std::string const &Type)
{
	if (Type == "Rpd")
		return 1;
	if (Type == "Rpj")
		return 2;
	return Type == "Rps" ? 3 : 0;
}

// the name and the type of a template, in the fields table
bool edit_name_type(switch_design::design &Entry)
{
	auto changed{false};
	std::array<char, 96> name{};
	Entry.name.copy(name.data(), name.size() - 1);
	field_row("Name##designname");
	if (ImGui::InputText("##designname", name.data(), name.size()))
	{
		Entry.name = name.data();
		changed = true;
	}
	std::array<char const *, 4> const types{STR_C("Rz ordinary"), STR_C("Rpd double, to both sides"), STR_C("Rpj double, to one side"), STR_C("Rps double, symmetric")};
	std::array<char const *, 4> const codes{"Rz", "Rpd", "Rpj", "Rps"};
	int type{type_index(Entry.type)};
	field_row("Type##designtype");
	if (ImGui::Combo("##designtype", &type, types.data(), 4))
	{
		Entry.type = codes[static_cast<std::size_t>(type)];
		if (type > 0 && Entry.stagger <= 0.0)
			Entry.stagger = 5.0;
		changed = true;
	}
	item_tooltip("A double switch is laid as two switches: the first one with its diverging track and the main track up to the second one, which starts there with the other diverging track");
	return changed;
}

// the pre-blade stretch, the blade angle and the tip of the second switch, in the fields table: those of the first one until given
bool edit_second_blades(switch_design::design &Entry)
{
	auto changed{false};
	if (auto lead2{Entry.lead2 >= 0.0 ? Entry.lead2 : Entry.lead}; input_row("Pre-blade stretch, m##designlead2", "##designlead2", lead2, "%.3f"))
	{
		Entry.lead2 = std::max(0.0, lead2);
		changed = true;
	}
	if (auto entry2{Entry.entry2 >= 0.0 ? Entry.entry2 : Entry.entry}; input_row("Blade angle, deg##designentry2", "##designentry2", entry2, "%.4f"))
	{
		Entry.entry2 = std::max(0.0, entry2);
		changed = true;
	}
	if (auto tip2{Entry.tip2 >= 0.0 ? Entry.tip2 : Entry.tip}; input_row("Tip of a tangent blade, m##designtip2", "##designtip2", tip2, "%.3f"))
	{
		Entry.tip2 = std::max(0.0, tip2);
		changed = true;
	}
	return changed;
}

// the second switch of a double one: where it starts, and the diverging track of its own if it has one
bool edit_second_switch(switch_design::design &Entry, bool const Editable, int &Pointing)
{
	auto changed{false};
	ImGui::Separator();
	if (begin_fields("##staggerfields", 0.5f))
	{
		changed |= input_row("Second switch starts at, m##stagger", "##stagger", Entry.stagger, "%.3f");
		mark(Pointing, value::stagger);
		item_tooltip("Distance from the start of the first switch to the start of the second one, along the main track");
		ImGui::EndTable();
	}
	bool own{false == Entry.parts2.empty() && Entry.type != "Rps"};
	if (Entry.type == "Rps")
		ImGui::TextDisabled("%s", STR_C("The second switch is the mirror image of the first one"));
	else if (ImGui::Checkbox(STR_C("The second switch has a diverging track of its own"), &own))
	{
		Entry.parts2 = own ? Entry.parts : std::vector<switch_design::part>{};
		changed = true;
	}
	if (Entry.type != "Rps")
		item_tooltip("Otherwise the second switch diverges the same way as the first, to the other side for Rpd, to the same side for Rpj");
	if (own)
	{
		if (begin_fields("##design2fields", 0.5f))
		{
			changed |= edit_second_blades(Entry);
			ImGui::EndTable();
		}
		int unused{-1};
		changed |= edit_parts(Entry.parts2, "##parts2", Editable, unused, value::part);
	}
	else if (Entry.lead2 >= 0.0 || Entry.entry2 >= 0.0 || Entry.tip2 >= 0.0)
	{
		Entry.lead2 = -1.0;
		Entry.entry2 = -1.0;
		Entry.tip2 = -1.0;
		changed = true;
	}
	return changed;
}

// the rail and the blades of a template, in the fields table
bool edit_blades(switch_design::design &Entry, int &Pointing)
{
	auto changed{false};
	std::array<char, 32> rail{};
	if (Entry.rail != "-")
		Entry.rail.copy(rail.data(), rail.size() - 1);
	field_row("Rail##designrail");
	if (ImGui::InputText("##designrail", rail.data(), rail.size()))
	{
		Entry.rail = rail[0] != '\0' ? std::string{rail.data()} : std::string{"-"};
		changed = true;
	}
	item_tooltip("Rail of the switch kept in its record: 60E1, 49E1...");
	changed |= input_row("Pre-blade stretch, m##designlead", "##designlead", Entry.lead, "%.3f");
	mark(Pointing, value::lead);
	item_tooltip("From the start of the switch (the joint before the blades) to the tip of the blade; the diverging track runs along the main one there");
	changed |= input_row("Blade angle, deg##designentry", "##designentry", Entry.entry, "%.4f");
	mark(Pointing, value::entry);
	item_tooltip("The angle at which the blade meets the stock rail at its tip: the diverging track starts there at this angle instead of tangentially. Curved blades a fraction of a degree, straight blades 1 to 2 degrees; 0: tangential");
	changed |= input_row("Tip of a tangent blade, m##designtip", "##designtip", Entry.tip, "%.3f");
	mark(Pointing, value::tip);
	item_tooltip("With the blade angle 0: the tip of the blade from the start of the switch as its scheme gives it, the diverging track is broken there into pieces; 0: where the blade gets 5 mm thick, less the 125 mm its nose is planed over");
	changed |= input_row("Blade heel, m##designheel", "##designheel", Entry.heel, "%.3f");
	mark(Pointing, value::heel);
	item_tooltip("Where the moving blades end, from the start of the switch, as its scheme gives it; the switch keeps it in the scenery. 0: as long as the flangeway of 58 mm behind them needs");
	changed |= input_row("Blade throw, m##designthrow", "##designthrow", Entry.bladethrow, "%.3f");
	mark(Pointing, value::blade_throw);
	item_tooltip("How far the tips of the blades move over between the two paths; 0: the usual 0.160 m");
	return changed;
}

// the angle of a straight blade from its length and its gap at the heel
bool edit_straight_blade(switch_design::design &Entry, double &BladeLength, double &BladeGap)
{
	ImGui::TextDisabled("%s", STR_C("Straight blade:"));
	if (false == begin_fields("##bladefields", 0.5f))
		return false;
	auto changed{false};
	input_row("l, m##bladelength", "##bladelength", BladeLength, "%.2f");
	input_row("z, m##bladegap", "##bladegap", BladeGap, "%.3f");
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(1);
	if (ImGui::Button(STR_C("Angle from l and z"), ImVec2(-FLT_MIN, 0.0f)) && BladeLength > 0.1 && BladeGap > 0.0 && BladeGap < BladeLength)
	{
		Entry.entry = glm::degrees(std::asin(BladeGap / BladeLength));
		changed = true;
	}
	item_tooltip("sin angle = z / l: l the length of a straight blade, z the gap between the running edges of the stock rail and the blade at its heel");
	ImGui::EndTable();
	return changed;
}

// the main track of a template and the a and b of its catalogue, in the fields table
bool edit_main_track(switch_design::design &Entry, int &Pointing)
{
	auto changed{false};
	changed |= input_row("Main track, m##designmain", "##designmain", Entry.main, "%.3f");
	mark(Pointing, value::main_track);
	item_tooltip("Length of the main track, 0: as far as the diverging tracks reach");
	changed |= input_row("a##designa", "##designa", Entry.a, "%.3f");
	mark(Pointing, value::a);
	item_tooltip("From the start to the point where the tangents of both tracks meet, as the catalogue gives it; 0: from the geometry");
	changed |= input_row("b##designb", "##designb", Entry.b, "%.3f");
	mark(Pointing, value::b);
	item_tooltip("From the point where the tangents meet to the end, as the catalogue gives it; 0: the main track less a");
	return changed;
}

// the settings of a template under More settings
bool edit_more_settings(switch_design::design &Entry, bool const Editable, int &Pointing, double &BladeLength, double &BladeGap)
{
	auto changed{false};
	if (false == Editable)
		ImGui::BeginDisabled();
	if (begin_fields("##designmorefields", 0.5f))
	{
		changed |= edit_blades(Entry, Pointing);
		ImGui::EndTable();
	}
	if (Editable)
		changed |= edit_straight_blade(Entry, BladeLength, BladeGap);
	if (begin_fields("##designmainfields", 0.5f))
	{
		changed |= edit_main_track(Entry, Pointing);
		ImGui::EndTable();
	}
	if (false == Editable)
		ImGui::EndDisabled();
	return changed;
}

// what a template makes, or why it makes nothing; the rest of it in the tooltip
void show_shape(switch_design::design const &Entry, switch_design::shape const &Shape, std::string const &Error)
{
	if (false == Error.empty())
	{
		ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "%s", STR(Error).c_str());
		return;
	}
	ImGui::Text(STR_C("Angle 1:%.3f (%.4f deg), sharpest R %.1f m, a %.3f m, b %.3f m"), Shape.ratio, glm::degrees(std::atan(1.0 / Shape.ratio)), Shape.radius, Shape.a, Shape.b);
	if (false == ImGui::IsItemHovered())
		return;
	auto const length{format(STR_C("Length %.3f m, the diverging track ends %.3f m beside the main one, in %zu pieces"), Shape.length, Shape.offset, std::max<std::size_t>(1, Shape.paths[1].pieces.size()))};
	auto const tip{Entry.entry > 0.0 ? format(STR_C("Tip of the blade %.3f m from the start, the diverging track leaves there at %.4f deg"), Shape.tip, Entry.entry)
	                                 : format(STR_C("Tip of the blade %.3f m from the start, the diverging track leaves tangentially"), Shape.tip)};
	ImGui::SetTooltip("%s\n%s", length.c_str(), tip.c_str());
}

// the own templates saved: whether they were, the status telling how it went
bool save_templates(std::vector<switch_design::design> const &Own, bool &Dirty, std::string &Status)
{
	std::string problem;
	if (false == switch_design::save_own(Own, problem))
	{
		Status = problem;
		return false;
	}
	Dirty = false;
	Status = STR("Saved, the templates are on the list of the switch tool");
	return true;
}

// the own templates saved before a switch of them is placed: the status telling only of a failure
void save_before_placing(std::vector<switch_design::design> const &Own, bool &Dirty, std::string &Status)
{
	std::string problem;
	if (switch_design::save_own(Own, problem))
		Dirty = false;
	else
		Status = problem;
}

} // namespace

void editor_mode::switch_designs_load()
{
	auto &designer{m_switch_designer};
	if (designer.loaded)
		return;
	designer.loaded = true;
	designer.catalogue = switch_design::catalogue();
	designer.own = switch_design::own();
}

void editor_mode::switch_templates_refresh(std::string const &Arm)
{
	auto &tool{m_switch};
	std::vector<editor_track::switch_template> scenery;
	for (auto const &entry : tool.templates)
		if (entry.source != nullptr)
			scenery.push_back(entry);
	std::string armed{Arm};
	if (armed.empty() && tool.armed >= 0 && tool.armed < static_cast<int>(tool.templates.size()))
		armed = tool.templates[tool.armed].label;
	tool.templates = editor_track::standard_switch_templates();
	tool.templates.insert(tool.templates.end(), scenery.begin(), scenery.end());
	tool.collected = true;
	for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		if (false == armed.empty() && tool.templates[i].label.rfind(armed, 0) == 0)
		{
			tool.last = i;
			if (tool.armed >= 0)
				tool.armed = i;
			break;
		}
}

void editor_mode::switch_designer_click(glm::dvec3 const &Point)
{
	m_switch_designer.clicks.push_back(Point);
	switch_designer_measure();
}

// what the points clicked so far make: the shape and the radius keeping closest to the points along the diverging track; too few of them:
// the shape chosen at the radius given
void editor_mode::switch_designer_measure()
{
	auto &designer{m_switch_designer};
	designer.measured_ready = false;
	designer.measured_text.clear();
	if (designer.clicks.size() < 4)
		return;
	std::vector<glm::dvec2> plan;
	for (auto const &click : designer.clicks)
		plan.push_back(geometry::plan_of(click));
	switch_design::measured found;
	std::string problem;
	if (false == switch_design::measure_clicks(plan, found, problem))
	{
		designer.measured_text = STR(problem);
		return;
	}
	switch_design::design entry;
	entry.own = true;
	entry.rail = "S42";
	if (plan.size() >= 7)
	{
		switch_design::clicks_fit fitted;
		std::array<char const *, 4> const shapes{STR_C("straights"), STR_C("transition curve"), STR_C("two arcs"), STR_C("arc, straight, arc")};
		if (switch_design::fit_clicks(entry, plan, static_cast<switch_design::tangent_fit>(designer.fit_kind), designer.quick_first, fitted, problem))
		{
			designer.quick_radius = std::round(fitted.radius);
			found.radius = 0.0;
			designer.fit_kind = static_cast<int>(fitted.kind);
			designer.measured_text = format(STR_C("Along the points clicked: %s, R %.0f m, off them %.2f m at most"), shapes[static_cast<std::size_t>(fitted.kind)], fitted.radius, fitted.worst);
		}
	}
	if (entry.parts.empty() && false == switch_design::fit_tangents(entry, found.radius > 0.0 ? found.radius : designer.quick_radius, found.ratio, found.a, found.b, static_cast<switch_design::tangent_fit>(designer.fit_kind), problem, designer.quick_first))
	{
		designer.measured_text = format(STR_C("Measured a %.2f m, b %.2f m, 1:%.2f: %s"), found.a, found.b, found.ratio, STR(problem).c_str());
		return;
	}
	entry.a = found.a;
	entry.b = found.main - found.a;
	entry.main = found.main;
	if (std::abs(found.past) > 0.5)
		designer.measured_text += (designer.measured_text.empty() ? "" : "\n") + format(STR_C("The end of the diverging track lies %+.2f m along the main track from its end: check points 2 and 3"), found.past);
	designer.quick_a = found.a;
	designer.quick_b = found.b;
	designer.quick_ratio = found.ratio;
	if (found.radius > 0.0)
		designer.quick_radius = found.radius;
	designer.measured = entry;
	designer.measured_ready = true;
}

void editor_mode::switch_designer_finish()
{
	auto &designer{m_switch_designer};
	switch_designs_load();
	switch_designer_measure();
	if (false == designer.measured_ready)
	{
		designer.status = designer.measured_text;
		return;
	}
	auto entry{designer.measured};
	entry.name = format("%s %zu", STR_C("Switch"), designer.own.size() + 1);
	designer.own.push_back(entry);
	designer.from_catalogue = false;
	designer.selected = static_cast<int>(designer.own.size()) - 1;
	designer.dirty = true;
	designer.picking = false;
	first_step_done(first_step::template_clicked);
	designer.status = format(STR_C("Measured a %.2f m, b %.2f m, 1:%.2f"), designer.quick_a, designer.quick_b, designer.quick_ratio);
	if (false == designer.measured_text.empty())
		designer.status += "\n" + designer.measured_text;
}

void editor_mode::draw_switch_designer_clicks() const
{
	auto const &designer{m_switch_designer};
	if (designer.open && designer.picking)
		draw_cursor_ring(designer.clicks.empty() ? cursor_ground() : cursor_level(designer.clicks.front().y));
	if (false == designer.open || designer.clicks.empty())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const &clicks{designer.clicks};
	if (clicks.size() >= 2)
		projection.line(drawlist, clicks[0], clicks[1], IM_COL32(230, 230, 230, 230), 2.0f);
	if (clicks.size() >= 4)
		projection.line(drawlist, clicks[2], clicks[3], IM_COL32(90, 200, 255, 230), 2.0f);
	// the template laid where it was clicked: the one measured so far while clicking, then the one chosen on the list
	if (clicks.size() >= 3 && designer.on_view)
	{
		auto const &list{designer.from_catalogue ? designer.catalogue : designer.own};
		if (auto const *shown{shown_design(designer.picking, designer.measured, designer.measured_ready, list, designer.selected)}; shown != nullptr)
		{
			std::string problem;
			draw_laid(projection, drawlist, clicks, switch_design::build(*shown, problem));
		}
	}
	for (std::size_t i = 0; i < clicks.size(); ++i)
	{
		ImVec2 at;
		if (false == projection.project(clicks[i], at))
			continue;
		drawlist->AddCircleFilled(at, 4.0f, i < 4 ? IM_COL32(255, 225, 60, 255) : IM_COL32(120, 255, 120, 255));
		drawlist->AddText({at.x + 6.0f, at.y - 16.0f}, IM_COL32(255, 255, 255, 255), std::to_string(i + 1).c_str());
	}
}

void editor_mode::render_switch_designer()
{
	auto &designer{m_switch_designer};
	draw_switch_designer_clicks();
	if (false == designer.open)
		return;
	first_step_done(first_step::template_open);
	switch_designs_load();
	ImGui::SetNextWindowSize(ImVec2(820.0f, 560.0f), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin((std::string(STR_C("Switch templates")) + "###switchdesigner").c_str(), &designer.open, ImGuiWindowFlags_NoCollapse))
	{
		ImGui::End();
		return;
	}
	ImGui::BeginChild("##designlist", ImVec2(230.0f, 0.0f), true);
	render_switch_designer_list();
	ImGui::EndChild();
	ImGui::SameLine();
	ImGui::BeginChild("##designedit", ImVec2(0.0f, 0.0f), false);
	render_switch_designer_edit();
	ImGui::EndChild();
	ImGui::End();
}

// the templates on the left of the window, and the switch clicked off the view
void editor_mode::render_switch_designer_list()
{
	auto &designer{m_switch_designer};
	ImGui::TextDisabled("%s", STR_C("Catalogue (read only)"));
	item_tooltip("The catalogue switches start tangentially at the joint before the blades: without the pre-blade stretch and the angle of the blade, which a copy can be given");
	for (int i = 0; i < static_cast<int>(designer.catalogue.size()); ++i)
		if (ImGui::Selectable(std::format("{}##catalogue{}", switch_name_translated(designer.catalogue[i].name), i).c_str(), designer.from_catalogue && designer.selected == i))
		{
			designer.from_catalogue = true;
			designer.selected = i;
		}
	ImGui::Separator();
	ImGui::TextDisabled("%s", STR_C("Own templates"));
	for (int i = 0; i < static_cast<int>(designer.own.size()); ++i)
		if (ImGui::Selectable(std::format("{}##own{}", designer.own[i].name, i).c_str(), false == designer.from_catalogue && designer.selected == i))
		{
			designer.from_catalogue = false;
			designer.selected = i;
		}
	if (designer.own.empty())
		ImGui::TextDisabled("%s", STR_C("(none yet)"));
	if (ImGui::Button(STR_C("New")))
	{
		switch_design::design entry;
		entry.own = true;
		entry.name = format("%s %zu", STR_C("Switch"), designer.own.size() + 1);
		auto const angle{std::atan(1.0 / 9.0)};
		entry.parts = {{190.0 * angle, 190.0, 190.0}, {6.0, 0.0, 0.0}};
		designer.own.push_back(entry);
		designer.from_catalogue = false;
		designer.selected = static_cast<int>(designer.own.size()) - 1;
		designer.dirty = true;
	}
	item_tooltip("A new own template, an Rz 1:9 R190 to start from");
	if (ImGui::Button(designer.picking ? STR_C("Stop clicking") : STR_C("Click off the view...")))
	{
		designer.picking = false == designer.picking;
		if (designer.picking)
			designer.clicks.clear();
	}
	item_tooltip("Measures a switch off the orthophoto: click its points in the view, in the top view (O) they're best seen. The orthophoto is turned on in the layers");
	if (designer.clicks.size() >= 3)
	{
		ImGui::Checkbox(STR_C("Show on the view"), &designer.on_view);
		item_tooltip("The chosen template drawn in the view where the switch was clicked, as it changes: to be compared with the orthophoto under it");
	}
	if (designer.picking)
		render_switch_designer_clicking();
}

// the points of the switch to click off the view, the one to click next lit
void editor_mode::render_switch_designer_clicking()
{
	auto &designer{m_switch_designer};
	std::array<char const *, 5> const steps{STR_C("1. The start of the switch"), STR_C("2. The end of the switch on the main track"), STR_C("3. The end of the switch on the diverging track"), STR_C("4. A point further on the diverging track, for its way"), STR_C("5. Points along the diverging track, 3 at least, as laying a curve from clicks (optional)")};
	auto const at{std::min<std::size_t>(designer.clicks.size(), 4)};
	for (std::size_t i = 0; i < steps.size(); ++i)
		if (i == at)
			ImGui::TextColored(ImVec4(1.0f, 0.88f, 0.25f, 1.0f), "%s", steps[i]);
		else
			ImGui::TextDisabled("%s", steps[i]);
	if (at == 4)
		ImGui::TextDisabled(STR_C("On the arc: %zu"), designer.clicks.size() - 4);
	ImGui::TextDisabled("%s", STR_C("Backspace: back, Esc: stop"));
	if (false == m_orthophoto.enabled())
	{
		if (ImGui::SmallButton(STR_C("Orthophoto...")))
			m_orthophoto_window = true;
		item_tooltip("The photo to click the switch off isn't turned on");
	}
	if (ImGui::SmallButton(STR_C("Back")) && false == designer.clicks.empty())
	{
		designer.clicks.pop_back();
		switch_designer_measure();
	}
	if (false == designer.measured_text.empty())
		ImGui::TextWrapped("%s", designer.measured_text.c_str());
	ImGui::SameLine();
	if (designer.clicks.size() >= 4 && ImGui::SmallButton(STR_C("Make the template")))
		switch_designer_finish();
}

// the template chosen on the list, on the right of the window
void editor_mode::render_switch_designer_edit()
{
	auto &designer{m_switch_designer};
	if (auto const &list{designer.from_catalogue ? designer.catalogue : designer.own}; designer.selected < 0 || designer.selected >= static_cast<int>(list.size()))
	{
		ImGui::TextWrapped("%s", STR_C("Choose a template on the left; copy a catalogue one to own templates to change it"));
		item_tooltip("A template is a straight main track and the diverging track from the start of the switch as a chain of straights, arcs and transition curves; a double switch is two switches, the second one starting on the main track of the first");
		return;
	}
	bool const editable{false == designer.from_catalogue};
	auto &entry{designer.from_catalogue ? designer.catalogue[designer.selected] : designer.own[designer.selected]};
	bool changed{false};
	auto pointing{designer.pointed_test};
	if (false == editable)
	{
		if (ImGui::Button(STR_C("Copy to own templates")))
		{
			auto copy{entry};
			copy.own = true;
			copy.note.clear();
			copy.name = switch_name_translated(copy.name) + STR(" (copy)");
			designer.own.push_back(copy);
			designer.from_catalogue = false;
			designer.selected = static_cast<int>(designer.own.size()) - 1;
			designer.dirty = true;
			return;
		}
		ImGui::SameLine();
		ImGui::TextDisabled("%s", STR_C("A catalogue template can't be changed"));
	}
	if (false == editable)
		ImGui::BeginDisabled();
	if (begin_fields("##designfields", 0.5f))
	{
		changed |= edit_name_type(entry);
		ImGui::EndTable();
	}
	if (false == editable)
		ImGui::EndDisabled();

	{
		std::string problem;
		ImGui::Checkbox(STR_C("Stretch across"), &designer.stretched);
		item_tooltip("Draws the offsets from the main track larger, so the shape of the diverging track can be seen");
		// what's being clicked while it's clicked: the template chosen isn't changed by it
		auto const &drawn{designer.picking && designer.measured_ready ? designer.measured : entry};
		draw_schematic(drawn, switch_design::build(drawn, problem), designer.pointed, designer.stretched);
	}

	ImGui::Separator();
	ImGui::TextDisabled("%s", STR_C("Diverging track from the start of the switch"));
	if (editable && begin_fields("##quickfields", 0.5f))
		changed |= render_switch_designer_quick(entry, pointing);
	changed |= edit_parts(entry.parts, "##parts", editable, pointing, value::part);
	bool const twofold{switch_design::double_switch(entry.type)};
	if (false == editable)
		ImGui::BeginDisabled();
	if (twofold)
		changed |= edit_second_switch(entry, editable, pointing);
	if (false == editable)
		ImGui::EndDisabled();
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		changed |= edit_more_settings(entry, editable, pointing, designer.blade_length, designer.blade_gap);
		ImGui::TreePop();
	}

	std::string error;
	auto const shape{switch_design::build(entry, error)};
	ImGui::Separator();
	show_shape(entry, shape, error);
	designer.pointed = pointing;
	if (changed)
		designer.dirty = true;
	render_switch_designer_actions(entry, editable, error.empty());
	if (false == designer.status.empty())
		ImGui::TextWrapped("%s", designer.status.c_str());
}

// the diverging track of a template filled in from its radius, its angle and its a and b, in the fields table, which it ends
bool editor_mode::render_switch_designer_quick(switch_design::design &Entry, int &Pointing)
{
	auto &designer{m_switch_designer};
	auto changed{false};
	input_row("R##quickradius", "##quickradius", designer.quick_radius, "%.0f");
	mark(Pointing, value::radius);
	input_row("1:n##quickratio", "##quickratio", designer.quick_ratio, "%.2f");
	mark(Pointing, value::ratio);
	input_row("a, m##quicka", "##quicka", designer.quick_a, "%.3f");
	mark(Pointing, value::a);
	item_tooltip("From the start to the point where the tangents of both tracks meet; with a b differing the curve is made of two arcs; shorter than the tangent of the arc: the blade meets the stock rail at an angle. 0: the tangent of the arc");
	input_row("b, m##quickb", "##quickb", designer.quick_b, "%.3f");
	mark(Pointing, value::b);
	std::array<char const *, 4> const fits{STR_C("straights"), STR_C("transition curve"), STR_C("two arcs"), STR_C("arc, straight, arc")};
	field_row("Rest of the tangents##quickkind");
	ImGui::Combo("##quickkind", &designer.fit_kind, fits.data(), 4);
	item_tooltip("What makes up where a or b is longer than the tangent of the arc");
	if (designer.fit_kind == static_cast<int>(switch_design::tangent_fit::arc_straight_arc))
	{
		input_row("R of the first arc##quickfirst", "##quickfirst", designer.quick_first, "%.0f");
		item_tooltip("The radius of the arc before the straight, when it isn't the one after it; 0: the R");
	}
	item_tooltip("From the point where the tangents meet to the end of the switch. 0: the tangent of the arc");
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(1);
	if (ImGui::Button(STR_C("Fill in"), ImVec2(-FLT_MIN, 0.0f)) && designer.quick_radius >= 10.0 && designer.quick_ratio > 1.0)
	{
		std::string problem;
		if (switch_design::fit_tangents(Entry, designer.quick_radius, designer.quick_ratio, designer.quick_a, designer.quick_b, static_cast<switch_design::tangent_fit>(designer.fit_kind), problem, designer.quick_first))
		{
			designer.status.clear();
			changed = true;
		}
		else
			designer.status = problem;
	}
	item_tooltip("Replaces the parts with the curve from the start of the switch up to the angle 1:n fitted to a and b, R the sharpest of it: two arcs where a and b differ, else one arc with the straights they ask for");
	ImGui::EndTable();
	if (designer.quick_radius >= 10.0 && designer.quick_ratio > 1.0)
		ImGui::TextDisabled(STR_C("The tangent of the arc alone is %.3f m"), designer.quick_radius * std::tan(std::atan(1.0 / designer.quick_ratio) * 0.5));
	return changed;
}

// saving, removing and placing the template chosen; Buildable: it makes a switch
void editor_mode::render_switch_designer_actions(switch_design::design const &Entry, bool const Editable, bool const Buildable)
{
	auto &designer{m_switch_designer};
	if (Editable)
	{
		if (ImGui::Button(designer.dirty ? STR_C("Save *") : STR_C("Save")) && save_templates(designer.own, designer.dirty, designer.status))
			switch_templates_refresh();
		item_tooltip("Keeps the own templates in eu07_switch_templates.txt in the configuration directory of the simulator and puts them on the list of the switch tool");
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Delete")))
		{
			designer.own.erase(designer.own.begin() + designer.selected);
			designer.selected = -1;
			designer.dirty = true;
		}
		item_tooltip("Removes the template; Save keeps the change");
	}
	if (false == Buildable)
		return;
	if (Editable)
		ImGui::SameLine();
	if (ImGui::Button(STR_C("Place this switch")))
	{
		if (designer.dirty && Editable)
			save_before_placing(designer.own, designer.dirty, designer.status);
		switch_templates_refresh(Entry.name + "  ");
		show_track_tab(track_tab::turnout);
		arm_switch();
	}
	item_tooltip("Saves the own templates and chooses this one in the switch tool: then click a straight and drag to the side the switch goes");
}
