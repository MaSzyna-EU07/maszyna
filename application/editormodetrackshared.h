/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// what the files of the track mode of the editor share: the sizes of the handles and reaches, and small helpers

#include "editor/editorAlignment.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorTrack.hpp"
#include "utilities/translation.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include <optional>
#include <string>
#include <vector>

namespace track_mode
{

using geometry::bezier;
using geometry::cross;
using geometry::grade_of;
using geometry::plan_distance;
using geometry::plan_of;
using geometry::signed_angle;
using geometry::turned;
using geometry::arc_pieces;
using geometry::arc_through;
using geometry::has_controls;
using geometry::level;
using geometry::reversed;
using geometry::straight_piece;

double constexpr kParallelFindAngle{15.0}; // deg, neighbours counted as beside the straight
double constexpr kParallelSnapAngle{5.0}; // deg, close enough to snap parallel
double constexpr kParallelSnapWindow{0.45}; // m from 4.00 / 4.50 / 4.75 m
double constexpr kParallelHintWindow{1.25}; // m, hint before the snap takes

inline double piece_plan_radius(segment_data const &Path)
{
	if (std::abs(Path.radius) > 0.5f)
		return std::abs(Path.radius);
	bezier const curve{Path};
	auto const begin{plan_of(curve.first(0.0))};
	auto const finish{plan_of(curve.first(1.0))};
	if (glm::length(begin) < 1e-9 || glm::length(finish) < 1e-9)
		return 0.0;
	auto const turn{std::abs(signed_angle(begin, finish))};
	if (turn < 1e-4)
		return 0.0;
	return curve.plan_length() / turn;
}

inline std::string lines_of(std::vector<std::string> const &Texts)
{
	std::string text;
	for (auto const &line : Texts)
		text += (text.empty() ? "" : "\n") + line;
	return text;
}

inline std::string narrowed(std::vector<segment_data> const &Pieces, editor_track::layout const Layout, double const Radius)
{
	if (Layout != editor_track::layout::arc_straight && Layout != editor_track::layout::straight_arc)
		return {};
	double sharpest{0.0};
	for (auto const &piece : Pieces)
		if (auto const radius{piece_plan_radius(piece)}; radius >= 0.5 && (sharpest == 0.0 || radius < sharpest))
			sharpest = radius;
	if (sharpest == 0.0 || sharpest >= Radius * 0.99)
		return {};
	return format(STR_C("R %.0f m doesn't reach the cursor, the arc goes through it with R %.0f m"), Radius, sharpest);
}

inline std::string describe(editor_track::straight const &Line)
{
	return format(STR_C("L %.2f m  azimuth %.4f deg  grade %.2f per mille  %zu paths"), Line.length, Line.azimuth, Line.grade * 1000.0, Line.tracks.size());
}

inline char const *point_label(editor_track::point_kind const Kind)
{
	switch (Kind)
	{
	case editor_track::point_kind::start: return STR_C("Start");
	case editor_track::point_kind::control1: return STR_C("Control 1");
	case editor_track::point_kind::control2: return STR_C("Control 2");
	default: return STR_C("End");
	}
}

editor_track::point_kind const kPointKinds[] = {editor_track::point_kind::start, editor_track::point_kind::control1, editor_track::point_kind::control2, editor_track::point_kind::end};

float const kHandleRadius{10.0f};
double const kLaySnapRadius{5.0};
float const kSnapPixels{16.0f};
double const kSwitchReach{6.0}; // a press farther from the selected track selects another one instead of placing the switch
double const kLineReach{2000.0}; // how far each way the Curve tab follows the line
double const kDigitizeFit{0.5};
double const kDrawnReach{100000.0};
double const kJointRange{400.0}; // the ends of the paths are drawn for the tracks this close to the camera
double const kDotsRange{500.0}; // m, the dots of the ends of the tracks are drawn this close to the camera in the 3D view
ImU32 const kJointColor{IM_COL32(255, 255, 255, 200)};
int const kRailCategory{1};

inline editor_track::branch_end branch_start(segment_data const &Diverging)
{
	auto const tangent{bezier{Diverging}.first(1.0)};
	return {Diverging.points[segment_data::point::end], plan_of(tangent), grade_of(tangent), static_cast<double>(Diverging.rolls[1])};
}

inline editor_track::branch_end branch_target(editor_track::snap_target const &Target)
{
	return {Target.position, plan_of(Target.direction), grade_of(Target.direction), Target.track != nullptr ? editor_track::joint_roll(*Target.track, Target.point) : 0.0};
}

inline double switch_from(editor_track::straight const &Line, double const Length, double const Along, int const Direction, bool const Snap)
{
	auto from{std::clamp(Direction > 0 ? Along : Along - Length, 0.0, Line.length - Length)};
	if (false == Snap)
		return from;
	std::vector<double> joints{0.0, Line.length};
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
	{
		auto const &path{Line.tracks[i]->m_paths[Line.paths[i]]};
		for (auto const index : {segment_data::point::start, segment_data::point::end})
			joints.push_back(Line.along(path.points[index]));
	}
	auto const nearest = [&](double const Value) {
		double best{Value};
		for (auto const joint : joints)
			if (std::abs(joint - Value) < std::abs(best - Value) || best == Value)
				best = std::abs(joint - Value) < 1.0 ? joint : best;
		return best;
	};
	auto const snappedstart{nearest(from)};
	auto const snappedend{nearest(from + Length)};
	if (snappedstart != from)
		from = snappedstart;
	else if (snappedend != from + Length)
		from = snappedend - Length;
	return std::clamp(from, 0.0, Line.length - Length);
}

inline std::string link_summary(editor_track::snap_target const &Target, editor_track::branch_link const &Link, double const Switchradius)
{
	auto const name{Target.track != nullptr && false == Target.track->name().empty() ? Target.track->name() : std::string{STR_C("(no name)")}};
	auto text{Link.reverse ? format(STR_C("The branch leads into %s with reverse curves R %.0f m"), name.c_str(), Link.radius) :
	          Link.radius > 0.0 ? format(STR_C("The branch leads into %s with an R %.0f m curve"), name.c_str(), Link.radius) : format(STR_C("The branch leads straight into %s"), name.c_str())};
	if (Link.insert > 0.0)
		text += format(STR_C(", a straight of %.1f m between them"), Link.insert);
	if (Link.reverse && Link.radius < Switchradius - 0.5)
		text += format(STR_C(", sharper than the switch (R %.0f m)"), Switchradius);
	if (Link.trim > 0.0)
		text += format(STR_C(", %s shortened %.2f m"), name.c_str(), Link.trim);
	else if (Link.trim < 0.0)
		text += format(STR_C(", %s lengthened %.2f m"), name.c_str(), -Link.trim);
	return text;
}

inline std::optional<alignment::join_result> eased_join(alignment::join_request const &Request, editor_track::easing const &In, editor_track::easing const &Out)
{
	if (In.pieces.empty() && Out.pieces.empty())
		return std::nullopt;
	auto middle{Request};
	if (false == In.pieces.empty())
	{
		middle.start.position = In.end;
		middle.start.direction = In.direction;
		middle.start.roll = In.pieces.back().rolls[1];
	}
	if (false == Out.pieces.empty())
	{
		middle.end.position = Out.end;
		middle.end.direction = -Out.direction;
		middle.end.roll = -Out.pieces.back().rolls[1];
	}
	if (false == editor_track::eases_room(In, Out, Request.start.position, Request.start.direction, Request.end.position, Request.end.direction))
		return std::nullopt;
	auto eased{alignment::join(middle)};
	if (eased.kind == alignment::join_kind::none || eased.kind == alignment::join_kind::bezier)
		return std::nullopt;
	eased.length = editor_track::eased_length(In, eased.length, Out);
	eased.pieces = editor_track::eased_pieces(In, eased.pieces, Out);
	return eased;
}

// curvature of the shape at Chainage, positive turning from x towards z
inline double curvature_at(alignment::result const &Shape, double const Chainage)
{
	auto const a{std::clamp(Chainage - 0.5, 0.0, Shape.length)};
	auto const b{std::clamp(Chainage + 0.5, 0.0, Shape.length)};
	if (b - a < 1e-3)
		return 0.0;
	auto const from{plan_of(alignment::evaluate(Shape, a).direction)};
	auto const to{plan_of(alignment::evaluate(Shape, b).direction)};
	return signed_angle(glm::normalize(from), glm::normalize(to)) / (b - a);
}

// circular curve from a point along a direction, cut into pieces of 90 degrees at most. Side: 1 to the left, -1 to the right
inline void compound_from(editor_track::curve const &Curve, alignment::vertex &Vertex)
{
	if (false == Curve.compound || Curve.arcs.size() < 2)
		return;
	Vertex.compound = true;
	Vertex.radius = std::round(Curve.arcs.front().radius);
	Vertex.share = Curve.arcs.front().turn;
	Vertex.arcs.clear();
	for (std::size_t i = 1; i < Curve.arcs.size(); ++i)
		Vertex.arcs.push_back({std::round(Curve.arcs[i].radius), std::round(Curve.arcs[i].transition), Curve.arcs[i].turn});
}

inline char const *type_name(TTrack const &Track)
{
	switch (Track.eType)
	{
	case tt_Normal: return "normal";
	case tt_Switch: return "switch";
	case tt_Cross: return "cross";
	case tt_Table: return "turntable";
	case tt_Tributary: return "tributary";
	default: return "unknown type";
	}
}

} // namespace track_mode
