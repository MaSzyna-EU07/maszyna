/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorTrack.hpp"
#include "editor/editorGeometry.hpp"

#include "world/Track.h"
#include "utilities/Globals.h"

#include <algorithm>
#include <cmath>

namespace
{

using geometry::arc_pieces;
using geometry::bezier;
using geometry::cross;
using geometry::grade_of;
using geometry::plan_distance;
using geometry::plan_of;
using geometry::reversed;
using geometry::signed_angle;
using geometry::straight_piece;

std::vector<segment_data> scaled(segment_data const &Path, bool const Atend, double const Length, double const Original)
{
	auto const &start{Path.points[segment_data::point::start]};
	auto const &end{Path.points[segment_data::point::end]};
	auto const scale{Length / std::max(Original, 1e-6)};
	segment_data piece{Path};
	piece.points[segment_data::point::end] = start + (end - start) * scale;
	piece.points[segment_data::point::control1] *= scale;
	piece.points[segment_data::point::control2] *= scale;
	piece.radius *= static_cast<float>(scale);
	return {Atend ? piece : reversed(piece)};
}

void spread_rolls(std::vector<segment_data> &Pieces, segment_data const &Path, double const Original, float const Radius)
{
	double chainage{0.0};
	for (auto &piece : Pieces)
	{
		auto const piecelength{bezier{piece}.plan_length()};
		auto const from{static_cast<float>(std::clamp(chainage / Original, 0.0, 1.0))};
		auto const to{static_cast<float>(std::clamp((chainage + piecelength) / Original, 0.0, 1.0))};
		piece.rolls = {glm::mix(Path.rolls[0], Path.rolls[1], from), glm::mix(Path.rolls[0], Path.rolls[1], to)};
		piece.radius = Radius;
		chainage += piecelength;
	}
}

} // namespace

std::vector<segment_data> editor_track::stretched(segment_data const &Path, bool const Atend, double const Length)
{
	auto const path{Atend ? Path : reversed(Path)};
	bezier const curve{path};
	auto const &start{path.points[segment_data::point::start]};
	auto const &end{path.points[segment_data::point::end]};
	auto const first{plan_of(curve.first(0.0))};
	auto const last{plan_of(curve.first(1.0))};
	if (glm::length(first) < 1e-9 || glm::length(last) < 1e-9)
		return {};
	glm::dvec2 const direction{glm::normalize(first)};
	auto const grade{grade_of(curve.first(0.0))};
	auto const original{curve.plan_length()};
	auto const turn{signed_angle(direction, glm::normalize(last))};
	auto const lateral{cross(direction, plan_of(end - start))};
	std::vector<segment_data> pieces;
	float radius{0.0f};
	if (std::abs(turn) < 1e-4 && std::abs(lateral) < 0.01)
		pieces = straight_piece(start, direction, grade, Length);
	else
	{
		auto const arcradius{original / std::abs(turn)};
		int const side{turn > 0.0 ? 1 : -1};
		auto const arc{arc_pieces(start, direction, grade, arcradius, std::abs(turn), side)};
		bool const circular{std::abs(turn) > 1e-4 && plan_distance(arc.back().points[segment_data::point::end], end) < 0.05 && plan_distance(arc.front().points[segment_data::point::start], start) < 1e-6};
		if (false == circular)
			return scaled(path, Atend, Length, original);
		pieces = arc_pieces(start, direction, grade, arcradius, std::min(Length / arcradius, glm::radians(330.0)), side);
		radius = path.radius;
	}
	spread_rolls(pieces, path, original, radius);
	if (false == Atend)
	{
		std::reverse(pieces.begin(), pieces.end());
		for (auto &piece : pieces)
			piece = reversed(piece);
	}
	return pieces;
}

std::optional<editor_track::path_end> editor_track::stretch_end(TTrack const &Track, point_ref const &Point)
{
	if (Track.eType != tt_Normal || false == is_supported(Track) || Track.m_paths.empty())
		return std::nullopt;
	auto const last{static_cast<int>(Track.m_paths.size()) - 1};
	if (false == Point.valid())
	{
		bool const startfree{false == is_connected(Track, {0, point_kind::start})};
		bool const endfree{false == is_connected(Track, {last, point_kind::end})};
		bool const atend{endfree || false == startfree};
		return path_end{atend ? last : 0, atend};
	}
	if (false == is_end(Point.kind) || Point.path > last)
		return std::nullopt;
	auto const position{point_position(Track, Point)};
	for (int i = 0; i <= last; ++i)
		if (i != Point.path && (plan_distance(Track.m_paths[i].points[segment_data::point::start], position) < geometry::same_point || plan_distance(Track.m_paths[i].points[segment_data::point::end], position) < geometry::same_point))
			return std::nullopt;
	return path_end{Point.path, Point.kind == point_kind::end};
}

std::pair<glm::dvec3, glm::dvec3> editor_track::end_axis(segment_data const &Path, bool const Atend)
{
	bezier const curve{Path};
	auto const tangent{Atend ? curve.first(1.0) : -curve.first(0.0)};
	return {Path.points[Atend ? segment_data::point::end : segment_data::point::start], glm::length(tangent) > 1e-9 ? glm::normalize(tangent) : glm::dvec3{}};
}

editor_track::point_ref editor_track::stretch_remap(point_ref Point, int const Path, std::size_t const Pieces)
{
	if (false == Point.valid() || Point.path < Path)
		return Point;
	auto const extra{static_cast<int>(Pieces) - 1};
	if (Point.path > Path || Point.kind == point_kind::end || Point.kind == point_kind::control2)
		Point.path += extra;
	return Point;
}

std::size_t editor_track::driven_paths(TTrack const &Track)
{
	return Track.eType == tt_Switch ? std::min<std::size_t>(1, Track.m_paths.size()) : Track.m_paths.size();
}

editor_track::middle_point editor_track::middle(TTrack const &Track)
{
	auto const count{driven_paths(Track)};
	double total{0.0};
	for (std::size_t i = 0; i < count; ++i)
		total += bezier{Track.m_paths[i]}.plan_length();
	middle_point result{Track.location(), {0.0, 0.0, 1.0}, total * 0.5};
	double along{0.0};
	for (std::size_t i = 0; i < count; ++i)
	{
		bezier const curve{Track.m_paths[i]};
		auto const length{curve.plan_length()};
		if (along + length >= result.chainage || i + 1 == count)
		{
			auto const t{curve.parameter(result.chainage - along)};
			result.point = curve.point(t);
			result.tangent = curve.first(t);
			return result;
		}
		along += length;
	}
	return result;
}

double editor_track::joint_roll(TTrack const &Track, point_ref const &Point)
{
	if (Point.path < 0 || Point.path >= static_cast<int>(Track.m_paths.size()))
		return 0.0;
	auto const &path{Track.m_paths[Point.path]};
	return Point.kind == point_kind::start ? path.rolls[0] : -path.rolls[1];
}

void editor_track::roll_joined(TTrack &Other, point_ref const &Theirs, float const Roll)
{
	auto &theirs{Other.m_paths[Theirs.path]};
	if (Theirs.kind == point_kind::start)
		theirs.rolls[0] = -Roll;
	else
		theirs.rolls[1] = Roll;
}

void editor_track::match_rolls(TTrack const &Track, std::vector<joint> const &Joints)
{
	for (auto const &joint : Joints)
		if (joint.own.path < static_cast<int>(Track.m_paths.size()) && joint.theirs.path < static_cast<int>(joint.other->m_paths.size()))
			roll_joined(*joint.other, joint.theirs, static_cast<float>(joint_roll(Track, joint.own)));
}

glm::dvec3 editor_track::outward(TTrack const &Track, point_ref const &Point)
{
	bezier const curve{Track.m_paths[Point.path]};
	return Point.kind == point_kind::start ? -curve.first(0.0) : curve.first(1.0);
}

editor_track::point_ref editor_track::switch_main(TTrack const &Track, point_ref Point)
{
	if (Track.eType == tt_Switch && Point.valid() && Point.path > 0)
		Point.path = 0;
	return Point;
}

glm::dvec3 editor_track::control_vector(TTrack const &Track, point_ref const &Control)
{
	return point_position(Track, Control) - point_position(Track, end_of(Control));
}

std::optional<editor_track::drive_spot> editor_track::drive_spot_of(TTrack const &Switch)
{
	if (Switch.eType != tt_Switch || Switch.m_paths.size() < 2)
		return std::nullopt;
	auto const &main{Switch.m_paths[0]};
	auto const &diverging{Switch.m_paths[1]};
	auto const start{main.points[segment_data::point::start]};
	auto const &control{main.points[segment_data::point::control1]};
	auto const along{control != glm::dvec3{} ? control : main.points[segment_data::point::end] - start};
	auto heading{plan_of(along)};
	if (glm::length(heading) < 1e-9)
		return std::nullopt;
	heading = glm::normalize(heading);
	glm::dvec2 const right{-heading.y, heading.x};
	drive_spot spot;
	spot.rightswitch = glm::dot(plan_of(diverging.points[segment_data::point::end] - start), right) > 0.0;
	spot.location = start;
	auto const yaw{glm::degrees(std::atan2(heading.x, heading.y))};
	spot.yaw = yaw < 0.0 ? yaw + 360.0 : yaw;
	auto const run{glm::length(plan_of(along))};
	auto const roll{static_cast<double>(main.rolls[0])};
	spot.tilt = {-glm::degrees(std::atan(along.y / run)), roll};
	if (Global.bRollFix)
		spot.location.y += 0.75 * std::abs(std::sin(glm::radians(roll)));
	return spot;
}

double editor_track::straight_axis::along(glm::dvec3 const &Point) const
{
	return glm::dot(plan_of(Point - origin), direction);
}

double editor_track::straight_axis::lateral(glm::dvec3 const &Point) const
{
	return std::abs(cross(direction, plan_of(Point - origin)));
}

std::optional<editor_track::straight_axis> editor_track::main_axis(TTrack const &Switch)
{
	if (Switch.m_paths.empty())
		return std::nullopt;
	auto const &main{Switch.m_paths.front()};
	auto const origin{main.points[segment_data::point::start]};
	auto const axis{plan_of(main.points[segment_data::point::end] - origin)};
	auto const length{glm::length(axis)};
	if (length < 1e-3)
		return std::nullopt;
	return straight_axis{origin, axis / length, length};
}

std::vector<segment_data> editor_track::slid(TTrack const &Switch, double const Distance)
{
	if (Switch.m_paths.empty())
		return {};
	auto const &main{Switch.m_paths.front()};
	auto const run{main.points[segment_data::point::end] - main.points[segment_data::point::start]};
	auto const planlength{glm::length(plan_of(run))};
	if (planlength < 1e-3)
		return {};
	auto const shift{run * (Distance / planlength)};
	auto paths{Switch.m_paths};
	for (auto &path : paths)
	{
		path.points[segment_data::point::start] += shift;
		path.points[segment_data::point::end] += shift;
	}
	return paths;
}

void editor_track::edit::capture(TTrack &Track)
{
	if (std::find(changed.begin(), changed.end(), &Track) != changed.end())
		return;
	states.emplace_back(&Track, editor_track::capture(Track));
	changed.push_back(&Track);
}
