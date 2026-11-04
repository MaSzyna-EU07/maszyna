/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/

#pragma once

#include "application/editormode.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "imgui/imgui.h"

// what the files of the track mode share
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

inline std::string describe(editor_track::straight const &Line)
{
	return format("L %.2f m  az %.4f deg  i %.2f per mille  %zu paths", Line.length, Line.azimuth, Line.grade * 1000.0, Line.tracks.size());
}

inline char const *point_label(editor_track::point_kind const Kind)
{
	switch (Kind)
	{
	case editor_track::point_kind::start: return "Start";
	case editor_track::point_kind::control1: return "Control 1";
	case editor_track::point_kind::control2: return "Control 2";
	default: return "End";
	}
}

editor_track::point_kind const kPointKinds[] = {editor_track::point_kind::start, editor_track::point_kind::control1, editor_track::point_kind::control2, editor_track::point_kind::end};

float const kHandleRadius{10.0f};
double const kLaySnapRadius{5.0};
float const kSnapPixels{16.0f};
float const kDragPixels{5.0f};
double const kSwitchReach{6.0}; // a press farther from the selected track selects another one instead of placing the switch
double const kLineReach{2000.0}; // how far each way the Curve tab follows the line
double const kJointRange{400.0}; // the ends of the paths are drawn for the tracks this close to the camera
ImU32 const kJointColor{IM_COL32(255, 255, 255, 200)};
int const kRailCategory{1};

inline void item_tooltip(char const *Text)
{
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C(Text));
}

// the curve a track ends with at Joint, as it goes on beyond the joint, and a transition curve leading out of it to the straight
struct easing
{
	double curvature{0.0}; // positive turning from x towards z
	double length{0.0};
	glm::dvec3 end{0.0};
	glm::dvec2 direction{0.0};
	std::vector<segment_data> pieces;
};

inline easing ease_out_of(TTrack const &Neighbour, glm::dvec3 const &Joint, double const Grade, double const Speed, alignment::limits const &Limits)
{
	easing result;
	for (auto const &path : Neighbour.m_paths)
		for (auto const atend : {false, true})
		{
			if (glm::distance(path.points[atend ? segment_data::point::end : segment_data::point::start], Joint) > 0.05)
				continue;
			if (path.points[segment_data::point::control1] == glm::dvec3{} && path.points[segment_data::point::control2] == glm::dvec3{})
				return result;
			bezier const curve{path};
			auto const first{curve.first(atend ? 1.0 : 0.0)};
			auto const second{curve.second(atend ? 1.0 : 0.0)};
			auto const speed{std::hypot(first.x, first.z)};
			if (speed < 1e-9)
				return result;
			auto curvature{(first.x * second.z - first.z * second.x) / (speed * speed * speed)};
			glm::dvec2 direction{first.x / speed, first.z / speed};
			double roll{path.rolls[1]};
			if (false == atend)
			{
				curvature = -curvature;
				direction = -direction;
				roll = -path.rolls[0];
			}
			result.curvature = curvature;
			if (std::abs(curvature) < 1e-4)
				return result;
			result.length = std::max(20.0, alignment::recommend(Speed, 1.0 / std::abs(curvature), Limits).transition);
			auto const count{static_cast<int>(std::ceil(result.length / 10.0))};
			auto const piece{result.length / count};
			glm::dvec3 point{Joint};
			for (int i = 0; i < count; ++i)
			{
				segment_data part;
				part.points[segment_data::point::start] = point;
				auto const from{direction};
				int const steps{static_cast<int>(std::ceil(piece / 0.05))};
				auto const step{piece / steps};
				for (int j = 0; j < steps; ++j)
				{
					auto const along{i * piece + (j + 0.5) * step};
					auto const angle{curvature * (1.0 - along / result.length) * step};
					auto const half{turned(direction, angle * 0.5)};
					point += glm::dvec3{half.x * step, Grade * step, half.y * step};
					direction = turned(direction, angle);
				}
				part.points[segment_data::point::end] = point;
				part.points[segment_data::point::control1] = glm::dvec3{from.x, Grade, from.y} * (piece / 3.0);
				part.points[segment_data::point::control2] = -glm::dvec3{direction.x, Grade, direction.y} * (piece / 3.0);
				part.rolls = {static_cast<float>(roll * (1.0 - i * piece / result.length)), static_cast<float>(roll * (1.0 - (i + 1) * piece / result.length))};
				result.pieces.push_back(part);
			}
			result.end = point;
			result.direction = direction;
			return result;
		}
	return result;
}

inline segment_data turned_around(segment_data const &Path)
{
	segment_data result{Path};
	result.points[segment_data::point::start] = Path.points[segment_data::point::end];
	result.points[segment_data::point::end] = Path.points[segment_data::point::start];
	result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
	result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
	result.rolls = {-Path.rolls[1], -Path.rolls[0]};
	return result;
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

} // namespace track_mode
