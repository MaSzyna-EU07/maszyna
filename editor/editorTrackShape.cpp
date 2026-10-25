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
#include "editor/editorGroundFollow.hpp"

#include "world/Track.h"
#include "utilities/Globals.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace
{

using geometry::arc_pieces;
using geometry::arc_through;
using geometry::has_controls;
using geometry::bezier;
using geometry::cross;
using geometry::grade_of;
using geometry::plan_distance;
using geometry::plan_of;
using geometry::reversed;
using geometry::signed_angle;
using geometry::straight_piece;
using geometry::turned;

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

std::vector<segment_data> arc_then_straight(glm::dvec3 const &Point, glm::dvec2 const &Direction, double const Grade, glm::dvec3 const &Target, double const Radius, int const Side)
{
	glm::dvec2 const normal{-Direction.y * Side, Direction.x * Side};
	auto const centre{plan_of(Point) + normal * Radius};
	auto const towards{plan_of(Target) - centre};
	auto const distance{glm::length(towards)};
	auto const spread{std::acos(std::clamp(Radius / distance, -1.0, 1.0))};
	auto const radial0{plan_of(Point) - centre};
	double rest{-1.0};
	double angle{0.0};
	for (auto const sign : {1.0, -1.0})
	{
		auto const radial{turned(towards / distance, sign * spread) * Radius};
		auto turn{Side * signed_angle(radial0, radial)};
		if (turn < 0.0)
			turn += 2.0 * glm::pi<double>();
		auto const heading{turned(Direction, Side * turn)};
		auto const remaining{glm::dot(plan_of(Target) - centre - radial, heading)};
		if (remaining > -1e-3 && (rest < 0.0 || turn < angle))
		{
			rest = std::max(0.0, remaining);
			angle = turn;
		}
	}
	if (rest < 0.0 || angle > glm::pi<double>())
		return {};
	auto pieces{arc_pieces(Point, Direction, Grade, Radius, angle, Side)};
	if (rest < 0.01)
		return pieces;
	auto const tail{straight_piece(pieces.back().points[segment_data::point::end], turned(Direction, Side * angle), Grade, rest)};
	pieces.insert(pieces.end(), tail.begin(), tail.end());
	return pieces;
}

std::vector<segment_data> straight_then_arc(glm::dvec3 const &Point, glm::dvec2 const &Direction, double const Grade, glm::dvec2 const &Offset, double const Radius, int const Side)
{
	auto const along{glm::dot(Offset, Direction)};
	auto const lateral{std::abs(cross(Direction, Offset))};
	if (lateral > 2.0 * Radius)
		return arc_through(Point, Direction, Grade, Offset);
	auto const across{Radius - lateral};
	auto const lead{along - std::sqrt(std::max(0.0, Radius * Radius - across * across))};
	if (lead < 0.5)
		return arc_through(Point, Direction, Grade, Offset);
	auto const angle{2.0 * std::atan2(lateral, along - lead)};
	if (angle > glm::pi<double>())
		return {};
	auto pieces{straight_piece(Point, Direction, Grade, lead)};
	auto const arc{arc_pieces(pieces.back().points[segment_data::point::end], Direction, Grade, Radius, angle, Side)};
	pieces.insert(pieces.end(), arc.begin(), arc.end());
	return pieces;
}

void lay_transition(editor_track::easing &Result, glm::dvec3 const &Joint, glm::dvec2 Direction, double const Grade, double const Roll)
{
	auto const curvature{Result.curvature};
	auto const length{Result.length};
	auto const count{static_cast<int>(std::ceil(length / 10.0))};
	auto const piece{length / count};
	glm::dvec3 point{Joint};
	for (int i = 0; i < count; ++i)
	{
		segment_data part;
		part.points[segment_data::point::start] = point;
		auto const from{Direction};
		int const steps{static_cast<int>(std::ceil(piece / 0.05))};
		auto const step{piece / steps};
		for (int j = 0; j < steps; ++j)
		{
			auto const along{i * piece + (j + 0.5) * step};
			auto const angle{curvature * (1.0 - along / length) * step};
			auto const half{turned(Direction, angle * 0.5)};
			point += glm::dvec3{half.x * step, Grade * step, half.y * step};
			Direction = turned(Direction, angle);
		}
		part.points[segment_data::point::end] = point;
		part.points[segment_data::point::control1] = glm::dvec3{from.x, Grade, from.y} * (piece / 3.0);
		part.points[segment_data::point::control2] = -glm::dvec3{Direction.x, Grade, Direction.y} * (piece / 3.0);
		part.rolls = {static_cast<float>(Roll * (1.0 - i * piece / length)), static_cast<float>(Roll * (1.0 - (i + 1) * piece / length))};
		Result.pieces.push_back(part);
	}
	Result.end = point;
	Result.direction = Direction;
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
		path.translate(shift);
	return paths;
}

double editor_track::branch_radius(segment_data const &Diverging)
{
	if (std::abs(Diverging.radius) > 0.5f)
		return std::abs(Diverging.radius);
	bezier const curve{Diverging};
	auto const begin{plan_of(curve.first(0.0))};
	auto const finish{plan_of(curve.first(1.0))};
	if (glm::length(begin) < 1e-9 || glm::length(finish) < 1e-9)
		return 0.0;
	auto const turn{std::abs(signed_angle(begin, finish))};
	return turn > 1e-4 ? curve.plan_length() / turn : 0.0;
}

double editor_track::reverse_insert(double const Speed)
{
	if (Speed <= 60.0)
		return 6.0;
	return Speed <= 100.0 ? 0.15 * Speed : 0.25 * Speed;
}

double editor_track::diverging_speed(double const Radius)
{
	if (Radius < 300.0)
		return 40.0;
	if (Radius < 500.0)
		return 50.0;
	if (Radius < 760.0)
		return 60.0;
	if (Radius < 1200.0)
		return 80.0;
	return Radius < 2500.0 ? 100.0 : 130.0;
}

namespace
{

double constexpr kPi{3.14159265358979323846};
double constexpr kHalfPi{0.5 * kPi};
double constexpr kReverseNoInsert{106.5};

glm::dvec2 unit_chord(glm::dvec2 const &Heading, double const Angle)
{
	glm::dvec2 const left{-Heading.y, Heading.x};
	return Heading * std::sin(std::abs(Angle)) + left * ((Angle > 0.0 ? 1.0 : -1.0) * (1.0 - std::cos(Angle)));
}

struct reverse_fit
{
	double radius{0.0};
	double first{0.0};
	double second{0.0};
};

std::optional<reverse_fit> fit_reverse(glm::dvec2 const &Offset, glm::dvec2 const &Tangent, double const Turn, double const Insert)
{
	auto const residual = [&](double const First, double &Radius) {
		auto const heading{turned(Tangent, First)};
		auto const chord{unit_chord(Tangent, First) + unit_chord(heading, Turn - First)};
		auto const rest{Offset - heading * Insert};
		Radius = glm::dot(chord, rest) / std::max(glm::dot(chord, chord), 1e-12);
		return cross(chord, rest);
	};
	auto const usable = [&](double const First) { return std::abs(First) > 1e-4 && std::abs(Turn - First) > 1e-4 && (First > 0.0) != (Turn - First > 0.0) && std::abs(Turn - First) <= kHalfPi; };
	std::optional<reverse_fit> best;
	int constexpr steps{2000};
	double radius{0.0};
	auto previous{-kHalfPi};
	auto before{residual(previous, radius)};
	for (int i = 1; i <= steps; ++i)
	{
		auto const first{-kHalfPi + kPi * i / steps};
		auto const now{residual(first, radius)};
		if (usable(previous) && usable(first) && (before > 0.0) != (now > 0.0))
		{
			auto low{previous};
			auto high{first};
			auto lowvalue{before};
			for (int k = 0; k < 60; ++k)
			{
				auto const middle{0.5 * (low + high)};
				auto const value{residual(middle, radius)};
				if ((value > 0.0) == (lowvalue > 0.0))
				{
					low = middle;
					lowvalue = value;
				}
				else
					high = middle;
			}
			auto const root{0.5 * (low + high)};
			residual(root, radius);
			if (radius >= 1.0 && (false == best.has_value() || radius > best->radius))
				best = reverse_fit{radius, root, Turn - root};
		}
		previous = first;
		before = now;
	}
	return best;
}

} // namespace

std::optional<double> editor_track::branch_shift(branch_end const &From, glm::dvec2 const &Slide, double const Radius, branch_end const &To, double const Kink)
{
	if (glm::length(From.direction) < 1e-9 || glm::length(To.direction) < 1e-9 || glm::length(Slide) < 1e-9)
		return std::nullopt;
	auto const slide{glm::normalize(Slide)};
	auto const tangent{glm::normalize(From.direction)};
	auto const into{glm::normalize(To.direction)};
	auto const turn{signed_angle(tangent, into)};
	auto const offset{plan_of(To.position) - plan_of(From.position)};
	auto const across{cross(into, slide)};
	if (std::abs(turn) > glm::radians(170.0))
		return std::nullopt;
	if (std::abs(turn) <= Kink)
	{
		if (std::abs(across) < 1e-6)
			return std::nullopt;
		return cross(into, offset) / across;
	}
	if (Radius < 1.0)
		return std::nullopt;
	auto const gap{offset - unit_chord(tangent, turn) * Radius};
	if (std::abs(across) >= std::sin(glm::radians(1.0)))
		return cross(into, gap) / across;
	auto const determinant{cross(tangent, slide)};
	if (std::abs(determinant) < 1e-6 || cross(gap, slide) / determinant < -0.01)
		return std::nullopt;
	return cross(tangent, gap) / determinant;
}

editor_track::branch_link editor_track::link_branch(branch_end const &From, double const Radius, branch_end const &To, double const Kink, double const Insert, bool const Stretch)
{
	branch_link result;
	if (glm::length(From.direction) < 1e-9 || glm::length(To.direction) < 1e-9 || Radius < 1.0)
		return result;
	auto const tangent{glm::normalize(From.direction)};
	auto const into{glm::normalize(To.direction)};
	auto const turn{signed_angle(tangent, into)};
	auto const start{plan_of(From.position)};
	auto const target{plan_of(To.position)};
	auto const offset{target - start};
	if (std::abs(turn) > glm::radians(170.0))
		return result;
	std::vector<std::pair<double, int>> arcs;
	auto const reverse = [&]() {
		auto fit{fit_reverse(offset, tangent, turn, 0.0)};
		if (false == fit.has_value() || 2.0 / fit->radius > 1.0 / kReverseNoInsert)
		{
			fit = fit_reverse(offset, tangent, turn, Insert);
			result.insert = Insert;
		}
		if (false == fit.has_value())
			return false;
		result.reverse = true;
		result.radius = fit->radius;
		result.before = 0.0;
		result.after = 0.0;
		result.shortfall = 0.0;
		arcs = {{std::abs(fit->first), fit->first > 0.0 ? 1 : -1}, {std::abs(fit->second), fit->second > 0.0 ? 1 : -1}};
		return true;
	};
	if (std::abs(turn) <= Kink)
	{
		auto const lateral{cross(tangent, offset)};
		result.after = glm::dot(tangent, offset);
		if (std::abs(lateral) >= 0.005 && false == reverse())
			return result;
	}
	else
	{
		auto const determinant{cross(tangent, into)};
		auto const reach{cross(offset, into) / determinant};
		auto const tangentlength{Radius * std::tan(std::abs(turn) * 0.5)};
		result.before = reach - tangentlength;
		result.after = cross(tangent, offset) / determinant - tangentlength;
		if (reach > 0.0 && result.before >= -0.01)
		{
			result.before = std::max(0.0, result.before);
			arcs = {{std::abs(turn), turn > 0.0 ? 1 : -1}};
			result.radius = Radius;
		}
		else if (false == reverse())
		{
			result.radius = Radius;
			result.shortfall = std::max(0.01, -result.before);
			return result;
		}
	}
	if (result.after < -0.01 || (Stretch && result.after >= 0.01))
	{
		result.trim = -result.after;
		result.after = 0.0;
	}
	auto const line = [](glm::dvec2 const &A, glm::dvec2 const &B) {
		segment_data path;
		path.points[segment_data::point::start] = {A.x, 0.0, A.y};
		path.points[segment_data::point::end] = {B.x, 0.0, B.y};
		return path;
	};
	auto cursor{start};
	auto heading{tangent};
	if (result.before >= 0.01)
	{
		result.pieces.push_back(line(cursor, cursor + tangent * result.before));
		cursor += tangent * result.before;
	}
	for (std::size_t i = 0; i < arcs.size(); ++i)
	{
		auto const [angle, side]{arcs[i]};
		if (i > 0 && result.insert >= 0.01)
		{
			result.pieces.push_back(line(cursor, cursor + heading * result.insert));
			cursor += heading * result.insert;
		}
		auto const pieces{arc_pieces({cursor.x, 0.0, cursor.y}, heading, 0.0, result.radius, angle, side)};
		result.pieces.insert(result.pieces.end(), pieces.begin(), pieces.end());
		cursor = plan_of(pieces.back().points[segment_data::point::end]);
		heading = turned(heading, side * angle);
	}
	if (result.after >= 0.01)
		result.pieces.push_back(line(cursor, target));
	else if (result.trim == 0.0 && false == result.pieces.empty())
		result.pieces.back().points[segment_data::point::end] = {target.x, 0.0, target.y};
	if (false == result.pieces.empty())
	{
		auto const height1{To.position.y + To.grade * result.trim};
		ground_follow::follow(result.pieces, {From.position.y, From.grade, height1, To.grade}, [](std::vector<glm::dvec3> const &Points, std::vector<glm::dvec2> const &) { return std::vector<double>(Points.size(), std::numeric_limits<double>::quiet_NaN()); });
		std::vector<double> lengths;
		double total{0.0};
		for (auto const &piece : result.pieces)
			total += lengths.emplace_back(bezier{piece}.plan_length());
		double chainage{0.0};
		for (std::size_t i = 0; i < result.pieces.size(); ++i)
		{
			auto const from{From.roll + (To.roll - From.roll) * chainage / std::max(total, 1e-6)};
			chainage += lengths[i];
			auto const to{From.roll + (To.roll - From.roll) * chainage / std::max(total, 1e-6)};
			result.pieces[i].rolls = {static_cast<float>(from), static_cast<float>(to)};
		}
	}
	result.landing = result.pieces.empty() ? From.position : result.pieces.back().points[segment_data::point::end];
	result.valid = true;
	return result;
}

void editor_track::edit::capture(TTrack &Track)
{
	if (std::find(changed.begin(), changed.end(), &Track) != changed.end())
		return;
	states.emplace_back(&Track, editor_track::capture(Track));
	changed.push_back(&Track);
}

std::vector<segment_data> editor_track::layout_pieces(layout const Kind, glm::dvec3 const &Point, glm::dvec2 const &Direction, double const Grade, glm::dvec3 const &Target, double const Radius)
{
	glm::dvec2 const offset{Target.x - Point.x, Target.z - Point.z};
	auto const along{glm::dot(offset, Direction)};
	auto const lateral{cross(Direction, offset)};
	auto const chord{glm::length(offset)};
	if (chord < 0.5)
		return {};
	if (Kind == layout::straight)
		return straight_piece(Point, Direction, Grade, along);
	if (Kind == layout::arc || std::abs(lateral) < 0.005 * chord + 0.05)
		return arc_through(Point, Direction, Grade, offset);
	int const side{lateral > 0.0 ? 1 : -1};
	auto const radius{std::max(1.0, Radius)};
	if (radius >= chord * chord / (2.0 * std::abs(lateral)))
		return arc_through(Point, Direction, Grade, offset);
	if (Kind == layout::arc_straight)
		return arc_then_straight(Point, Direction, Grade, Target, radius, side);
	return straight_then_arc(Point, Direction, Grade, offset, radius, side);
}

std::optional<editor_track::bend> editor_track::bend_at(TTrack const &Neighbour, glm::dvec3 const &Joint)
{
	for (auto const &path : Neighbour.m_paths)
		for (auto const atend : {false, true})
		{
			if (glm::distance(path.points[atend ? segment_data::point::end : segment_data::point::start], Joint) > 0.05)
				continue;
			bend result;
			auto const sign{atend ? 1.0 : -1.0};
			result.roll = atend ? static_cast<double>(path.rolls[1]) : -static_cast<double>(path.rolls[0]);
			if (false == has_controls(path))
			{
				auto const chord{plan_of(path.points[segment_data::point::end] - path.points[segment_data::point::start])};
				if (glm::length(chord) < 1e-9)
					return std::nullopt;
				result.direction = glm::normalize(chord) * sign;
				return result;
			}
			bezier const curve{path};
			auto const first{curve.first(atend ? 1.0 : 0.0)};
			auto const second{curve.second(atend ? 1.0 : 0.0)};
			auto const speed{std::hypot(first.x, first.z)};
			if (speed < 1e-9)
				return std::nullopt;
			result.curvature = sign * (first.x * second.z - first.z * second.x) / (speed * speed * speed);
			result.direction = {sign * first.x / speed, sign * first.z / speed};
			return result;
		}
	return std::nullopt;
}

editor_track::easing editor_track::ease_out_of(TTrack const &Neighbour, glm::dvec3 const &Joint, double const Grade, double const Speed, alignment::limits const &Limits)
{
	easing result;
	auto const end{bend_at(Neighbour, Joint)};
	if (false == end.has_value())
		return result;
	result.curvature = end->curvature;
	if (std::abs(end->curvature) < 1e-4)
		return result;
	// the diverging track of a switch has no cant: run no faster than its curve lets with the unbalanced acceleration alone, the
	// transition out of it is the one for that speed
	auto speed{Speed};
	if (Neighbour.eType == tt_Switch)
		speed = std::min(Speed, std::sqrt(12.96 * Limits.unbalanced / std::abs(end->curvature)));
	result.length = std::max(20.0, alignment::recommend(speed, 1.0 / std::abs(end->curvature), Limits).transition);
	lay_transition(result, Joint, end->direction, Grade, end->roll);
	return result;
}

bool editor_track::eases_room(easing const &In, easing const &Out, glm::dvec3 const &Start, glm::dvec2 const &Startdirection, glm::dvec3 const &End, glm::dvec2 const &Enddirection)
{
	auto const from{In.pieces.empty() ? Start : In.end};
	auto const to{Out.pieces.empty() ? End : Out.end};
	auto const gap{plan_of(to) - plan_of(from)};
	return glm::length(gap) > 5.0 && glm::dot(In.pieces.empty() ? Startdirection : In.direction, gap) > 0.0 && glm::dot(Out.pieces.empty() ? Enddirection : -Out.direction, gap) > 0.0;
}

std::vector<segment_data> editor_track::eased_pieces(easing const &In, std::vector<segment_data> const &Middle, easing const &Out)
{
	auto pieces{In.pieces};
	pieces.insert(pieces.end(), Middle.begin(), Middle.end());
	for (auto piece{Out.pieces.rbegin()}; piece != Out.pieces.rend(); ++piece)
		pieces.push_back(reversed(*piece));
	return pieces;
}

double editor_track::eased_length(easing const &In, double const Middle, easing const &Out)
{
	return Middle + (In.pieces.empty() ? 0.0 : In.length) + (Out.pieces.empty() ? 0.0 : Out.length);
}
