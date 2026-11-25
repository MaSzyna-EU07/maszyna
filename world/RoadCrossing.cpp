/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/RoadCrossing.h"

#include "model/vertex.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>

namespace road_crossing
{

namespace
{

glm::dvec2 plan(glm::dvec3 const &Point)
{
	return {Point.x, Point.z};
}

std::array<glm::dvec3, 4> bezier(segment_data const &Path)
{
	auto const &p0{Path.points[segment_data::point::start]};
	auto const &p3{Path.points[segment_data::point::end]};
	if (Path.points[segment_data::point::control1] == glm::dvec3{0.0} && Path.points[segment_data::point::control2] == glm::dvec3{0.0})
		return {p0, p0 + (p3 - p0) / 3.0, p3 - (p3 - p0) / 3.0, p3};
	return {p0, p0 + Path.points[segment_data::point::control1], p3 + Path.points[segment_data::point::control2], p3};
}

glm::dvec3 bezier_point(std::array<glm::dvec3, 4> const &P, double const T)
{
	auto const u{1.0 - T};
	return u * u * u * P[0] + 3.0 * u * u * T * P[1] + 3.0 * u * T * T * P[2] + T * T * T * P[3];
}

glm::dvec3 bezier_derivative(std::array<glm::dvec3, 4> const &P, double const T)
{
	auto const u{1.0 - T};
	return 3.0 * u * u * (P[1] - P[0]) + 6.0 * u * T * (P[2] - P[1]) + 3.0 * T * T * (P[3] - P[2]);
}

glm::dvec2 direction_at(std::array<glm::dvec3, 4> const &P, double const T)
{
	auto direction{plan(bezier_derivative(P, std::clamp(T, 1e-4, 1.0 - 1e-4)))};
	if (glm::length(direction) < 1e-9)
		direction = plan(P[3] - P[0]);
	return glm::length(direction) > 1e-9 ? glm::normalize(direction) : glm::dvec2{0.0, 1.0};
}

} // namespace

namespace
{

double plan_cross(glm::dvec2 const &A, glm::dvec2 const &B)
{
	return A.x * B.y - A.y * B.x;
}

double plan_area(std::vector<glm::dvec2> const &Outline)
{
	double result{0.0};
	for (std::size_t i = 0; i < Outline.size(); ++i)
		result += plan_cross(Outline[i], Outline[(i + 1) % Outline.size()]);
	return 0.5 * result;
}

// the outline of the surface of a road piece in the plan: its left edge forward, its right edge back, every quarter of a metre or so
std::vector<glm::dvec2> footprint(segment_data const &Axis, double const Halfwidth)
{
	auto const points{bezier(Axis)};
	double length{0.0};
	for (int i = 0; i < 16; ++i)
		length += glm::length(plan(bezier_point(points, (i + 1) / 16.0) - bezier_point(points, i / 16.0)));
	auto const count{std::clamp(static_cast<int>(std::ceil(length / 0.25)), 2, 400)};
	std::vector<glm::dvec2> left, right;
	for (int i = 0; i <= count; ++i)
	{
		auto const t{static_cast<double>(i) / count};
		auto const centre{plan(bezier_point(points, t))};
		auto const along{direction_at(points, t)};
		// the left of the road, as surface_at() has it
		glm::dvec2 const side{along.y, -along.x};
		left.push_back(centre + side * Halfwidth);
		right.push_back(centre - side * Halfwidth);
	}
	std::vector<glm::dvec2> result{left};
	result.insert(result.end(), right.rbegin(), right.rend());
	return result;
}

// the part of the outline Subject within the convex outline Window (Sutherland-Hodgman)
std::vector<glm::dvec2> clip(std::vector<glm::dvec2> Subject, std::vector<glm::dvec2> const &Window)
{
	auto const turn{plan_area(Window) >= 0.0 ? 1.0 : -1.0};
	for (std::size_t i = 0; i < Window.size() && false == Subject.empty(); ++i)
	{
		auto const &a{Window[i]};
		auto const &b{Window[(i + 1) % Window.size()]};
		auto const side = [&](glm::dvec2 const &P) { return turn * plan_cross(b - a, P - a); };
		auto const input{std::move(Subject)};
		Subject.clear();
		for (std::size_t j = 0; j < input.size(); ++j)
		{
			auto const &current{input[j]};
			auto const &previous{input[(j + input.size() - 1) % input.size()]};
			auto const here{side(current)}, there{side(previous)};
			auto const meet = [&]() { return previous + (current - previous) * (there / (there - here)); };
			if (here >= 0.0)
			{
				if (there < 0.0)
					Subject.push_back(meet());
				Subject.push_back(current);
			}
			else if (there >= 0.0)
				Subject.push_back(meet());
		}
	}
	return Subject;
}

// the height of the top of a panel at a point of the plan, from its triangles; false off it
bool top_at(panel const &Panel, glm::dvec2 const &Point, double &Height)
{
	auto found{false};
	for (auto const &t : Panel.faces)
	{
		auto const d{(t[1].z - t[2].z) * (t[0].x - t[2].x) + (t[2].x - t[1].x) * (t[0].z - t[2].z)};
		if (std::abs(d) < 1e-14)
			continue;
		auto const u{((t[1].z - t[2].z) * (Point.x - t[2].x) + (t[2].x - t[1].x) * (Point.y - t[2].z)) / d};
		auto const v{((t[2].z - t[0].z) * (Point.x - t[2].x) + (t[0].x - t[2].x) * (Point.y - t[2].z)) / d};
		if (u < -1e-9 || v < -1e-9 || 1.0 - u - v < -1e-9)
			continue;
		auto const y{u * t[0].y + v * t[1].y + (1.0 - u - v) * t[2].y};
		if (false == found || y > Height)
			Height = y;
		found = true;
	}
	if (Panel.faces.empty() && Panel.outline.size() >= 3)
	{
		// no triangles: the top over the outline
		auto const turn{plan_area(Panel.outline) >= 0.0 ? 1.0 : -1.0};
		for (std::size_t i = 0; i < Panel.outline.size(); ++i)
			if (turn * plan_cross(Panel.outline[(i + 1) % Panel.outline.size()] - Panel.outline[i], Point - Panel.outline[i]) < 0.0)
				return false;
		Height = Panel.top;
		return true;
	}
	return found;
}

} // namespace

double flangeway_width(double const Radius)
{
	auto const radius{std::abs(Radius)};
	if (radius <= 0.0 || radius >= 350.0)
		return 0.067;
	if (radius >= 250.0)
		return 0.075;
	return 0.080;
}

double flangeway_gauge_depth(double const Gauge)
{
	return Gauge < 1.3 ? 0.010 : 0.014;
}

std::array<rail_gap, 2> rail_gaps(double const Gauge, double const Head, double const Flangeway)
{
	auto const half{0.5 * Gauge};
	rail_gap right;
	right.side = 1;
	right.flangeway_from = half - Flangeway;
	right.head_from = half;
	right.head_to = half + Head;
	rail_gap left;
	left.side = -1;
	left.flangeway_from = -half + Flangeway;
	left.head_from = -half;
	left.head_to = -half - Head;
	return {left, right};
}

double rail_track::cant() const
{
	return 1500.0 * std::sin(roll);
}

glm::dvec2 rail_track::axis_at(double const Along, glm::dvec2 &Right) const
{
	auto const base{right()};
	auto tangent{along + base * (curvature * Along)};
	tangent = glm::normalize(tangent);
	Right = {-tangent.y, tangent.x};
	return glm::dvec2{point.x, point.z} + along * Along + base * (0.5 * curvature * Along * Along);
}

rail_track track_at(TSegment const &Path, double const Distance)
{
	auto const length{Path.GetLength()};
	auto const s{std::clamp(Distance, 0.0, length)};
	rail_track result;
	result.distance = s;
	glm::dvec3 across;
	Path.plan_frame(s, result.point, across);
	auto const reach{std::min(0.5, 0.5 * length)};
	glm::dvec3 before, after, unused;
	Path.plan_frame(std::max(0.0, s - reach), before, unused);
	Path.plan_frame(std::min(length, s + reach), after, unused);
	auto const run{plan(after) - plan(before)};
	if (glm::length(run) > 1e-9)
	{
		result.along = glm::normalize(run);
		result.grade = (after.y - before.y) / glm::length(run);
	}
	// the bend: how far the direction turns right over the distance
	if (length > 0.1)
	{
		glm::dvec3 unused2;
		auto const span{std::min(1.0, 0.25 * length)};
		auto const a{std::max(0.0, s - span)}, b{std::min(length, s + span)};
		glm::dvec3 p0, p1, p2, p3;
		Path.plan_frame(a, p0, unused2);
		Path.plan_frame(std::min(length, a + 0.05), p1, unused2);
		Path.plan_frame(std::max(0.0, b - 0.05), p2, unused2);
		Path.plan_frame(b, p3, unused2);
		auto const d0{plan(p1) - plan(p0)}, d1{plan(p3) - plan(p2)};
		if (glm::length(d0) > 1e-9 && glm::length(d1) > 1e-9 && b - a > 0.2)
		{
			auto const t0{glm::normalize(d0)}, t1{glm::normalize(d1)};
			glm::dvec2 const right0{-t0.y, t0.x};
			result.curvature = glm::dot(t1 - t0, right0) / (b - a - 0.05);
		}
	}
	result.roll = Path.GetRoll(s);
	result.length = length;
	// a cant ramp: the roll at one end of the path isn't the one at the other
	result.ramp = std::abs(Path.GetRoll(0.0) - Path.GetRoll(length)) > 1e-4;
	return result;
}

head_plane plane_of(rail_track const &Track)
{
	head_plane result;
	result.origin = Track.point;
	// along the track it rises by the grade; across it the rail profiles are turned by the roll about the axis, the right one going
	// down for a roll above 0
	result.gradient = Track.grade * Track.along - std::tan(Track.roll) * Track.right();
	return result;
}

glm::dvec3 head_top(rail_track const &Track, int const Side, double const Along, double const Out)
{
	auto const lateral{0.5 * Track.gauge + Out};
	glm::dvec2 right;
	auto const centre{Track.axis_at(Along, right)};
	auto const point{centre + right * (Side * lateral * std::cos(Track.roll))};
	auto const height{Track.point.y + Track.grade * Along - Side * lateral * std::sin(Track.roll)};
	return {point.x, height, point.y};
}

bool surface_at(segment_data const &Axis, double const Halfwidth, glm::dvec2 const &Point, double &Height)
{
	auto const points{bezier(Axis)};
	// the nearest point of the axis in the plan, then the offset across it
	auto t{0.0};
	auto distance{std::numeric_limits<double>::max()};
	int constexpr samples{64};
	for (int i = 0; i <= samples; ++i)
	{
		auto const u{static_cast<double>(i) / samples};
		auto const d{glm::length(plan(bezier_point(points, u)) - Point)};
		if (d < distance)
		{
			distance = d;
			t = u;
		}
	}
	for (int i = 0; i < 20; ++i)
	{
		auto const along{plan(bezier_derivative(points, t))};
		auto const span{glm::dot(along, along)};
		if (span < 1e-12)
			break;
		auto const next{std::clamp(t + glm::dot(Point - plan(bezier_point(points, t)), along) / span, 0.0, 1.0)};
		if (std::abs(next - t) < 1e-12)
			break;
		t = next;
	}
	auto const axis{bezier_point(points, t)};
	auto const along{direction_at(points, t)};
	auto const offset{Point - plan(axis)};
	// past an end of the piece
	if (std::abs(glm::dot(offset, along)) > 1e-3)
		return false;
	// the left of the road, as the road lays its cross-section (road_axis::frame)
	glm::dvec2 const left{along.y, -along.x};
	auto const across{glm::dot(offset, left)};
	if (std::abs(across) > Halfwidth)
		return false;
	auto const roll{glm::radians(glm::mix(static_cast<double>(Axis.rolls[0]), static_cast<double>(Axis.rolls[1]), t))};
	Height = axis.y + across * std::tan(roll);
	return true;
}

void track_frame(rail_track const &Track, glm::dvec2 const &Point, double &Along, double &Across)
{
	auto along{glm::dot(Point - glm::dvec2{Track.point.x, Track.point.z}, Track.along)};
	glm::dvec2 right;
	for (int i = 0; i < 4; ++i)
	{
		auto const centre{Track.axis_at(along, right)};
		glm::dvec2 const tangent{right.y, -right.x};
		auto const step{glm::dot(Point - centre, tangent)};
		along += step;
		if (std::abs(step) < 1e-9)
			break;
	}
	auto const centre{Track.axis_at(along, right)};
	Along = along;
	Across = glm::dot(Point - centre, right);
}

double band(rail_track const &Track)
{
	return (0.5 * Track.gauge + Track.head) * std::cos(Track.roll);
}

bool facing(rail_track const &A, rail_track const &B, glm::dvec2 const &Road, double const Halfwidth, double *Gap)
{
	if (A.turnout || B.turnout || std::abs(glm::dot(A.along, B.along)) < std::cos(glm::radians(10.0)) || glm::length(Road) < 1e-9)
		return false;
	double along{0.0}, across{0.0};
	track_frame(A, glm::dvec2{B.point.x, B.point.z}, along, across);
	auto const gap{std::abs(across) - band(A) - band(B)};
	if (Gap != nullptr)
		*Gap = gap;
	if (gap < 0.05)
		return false;
	// along the road the fields sides of the heads are the gap over the sine of the angle apart; the line of the road square to it,
	// where its pieces meet, reaches the skew across its width further
	auto const sine{std::max(0.2, std::abs(glm::dot(glm::normalize(Road), A.right())))};
	auto const cosine{std::sqrt(std::max(0.0, 1.0 - sine * sine))};
	return gap / sine < facing_gap + 2.0 * Halfwidth * cosine / sine;
}

bool crossing_surface::at(glm::dvec2 const &Point, double &Height) const
{
	if (tracks.empty())
		return false;
	struct place
	{
		double along{0.0}, across{0.0};
		double beyond{0.0}; // how far past the field side of its heads
		double ahead{0.0}; // how far along the road from the point to its axis
	};
	std::vector<place> places(tracks.size());
	auto inside{tracks.size()};
	for (std::size_t i = 0; i < tracks.size(); ++i)
	{
		auto &entry{places[i]};
		track_frame(tracks[i], Point, entry.along, entry.across);
		entry.beyond = std::abs(entry.across) - band(tracks[i]);
		glm::dvec2 right;
		tracks[i].axis_at(entry.along, right);
		auto const across{glm::dot(road, right)};
		entry.ahead = std::abs(across) > 1e-6 ? -entry.across / across : std::numeric_limits<double>::max();
		if (entry.beyond <= 0.0 && (inside == tracks.size() || entry.beyond < places[inside].beyond))
			inside = i;
	}
	// the plane of the heads at the point where the road crosses the track, as the editor lays the decks of the road into it
	auto const height = [&](std::size_t const I) { return plane_of(tracks[I]).at(Point); };
	// over a track: the plane of its heads
	if (inside < tracks.size())
	{
		Height = height(inside);
		return true;
	}
	// between two: the nearest each way along the road
	auto behind{tracks.size()}, front{tracks.size()};
	for (std::size_t i = 0; i < tracks.size(); ++i)
	{
		if (places[i].ahead < 0.0 && (behind == tracks.size() || places[i].ahead > places[behind].ahead))
			behind = i;
		if (places[i].ahead > 0.0 && places[i].ahead < std::numeric_limits<double>::max() && (front == tracks.size() || places[i].ahead < places[front].ahead))
			front = i;
	}
	if (behind < tracks.size() && front < tracks.size() && facing(tracks[behind], tracks[front], road, halfwidth))
	{
		// straight across from the field side of the head of the one to the field side of the head of the other: from the arc of the
		// field side of the heads of each track (the line over which a point is over the track, above), at the plane of its heads there,
		// in proportion to how far past each the point is. So the surface goes on from the plane of the heads of each track without a
		// step, on a curve too: taken from straight lines along the tracks it stepped where their arcs leave them, by the break times the
		// millimetres between the two. Between two tracks lying apart a little out of parallel, on curves, it's twisted a hair from a
		// plane: the pieces of the road meeting on it meet at the same points along their ends (conform())
		auto const &a{tracks[behind]};
		auto const &b{tracks[front]};
		auto const foot = [&](std::size_t const Index) {
			glm::dvec2 right;
			auto const centre{tracks[Index].axis_at(places[Index].along, right)};
			return centre + right * ((places[Index].across >= 0.0 ? 1.0 : -1.0) * band(tracks[Index]));
		};
		auto const u{std::max(0.0, places[behind].beyond)}, v{std::max(0.0, places[front].beyond)};
		auto const ha{plane_of(a).at(foot(behind))};
		auto const hb{plane_of(b).at(foot(front))};
		Height = ha + (hb - ha) * u / std::max(1e-9, u + v);
		return true;
	}
	// past the outer heads, or between two tracks not facing each other: the plane of the nearer one going on
	auto nearest{std::size_t{0}};
	for (std::size_t i = 1; i < tracks.size(); ++i)
		if (places[i].beyond < places[nearest].beyond)
			nearest = i;
	Height = height(nearest);
	return true;
}

double crossing_surface::slope(glm::dvec2 const &Point, glm::dvec2 const &Direction) const
{
	if (glm::length(Direction) < 1e-9)
		return 0.0;
	auto const step{0.02};
	double here{0.0}, there{0.0};
	if (false == at(Point, here) || false == at(Point + glm::normalize(Direction) * step, there))
		return 0.0;
	return (there - here) / step;
}

crossing_surface surface_of(std::vector<rail_track> const &Tracks, glm::dvec2 const &Road, double const Halfwidth)
{
	crossing_surface result;
	if (glm::length(Road) < 1e-9)
		return result;
	result.road = glm::normalize(Road);
	result.halfwidth = Halfwidth;
	for (auto const &track : Tracks)
	{
		if (track.turnout || std::abs(glm::dot(result.road, track.right())) < std::sin(glm::radians(15.0)))
			continue;
		if (std::any_of(result.tracks.begin(), result.tracks.end(), [&](rail_track const &Other) { return Other.source == track.source && Other.path == track.path; }))
			continue;
		result.tracks.push_back(track);
	}
	return result;
}

bool follows(segment_data const &Axis, crossing_surface const &Surface)
{
	if (Surface.empty())
		return false;
	auto const points{bezier(Axis)};
	auto const length{glm::length(plan(points[3] - points[0]))};
	auto const count{std::clamp(static_cast<int>(std::ceil(length / 0.25)), 4, 400)};
	for (int i = 0; i <= count; ++i)
	{
		auto const point{bezier_point(points, static_cast<double>(i) / count)};
		double height{0.0};
		if (false == Surface.at(plan(point), height) || std::abs(height - point.y) > cut_tolerance)
			return false;
	}
	return true;
}

bool height_on(segment_data const &Axis, double const Halfwidth, crossing_surface const *Surface, glm::dvec2 const &Point, double &Height)
{
	if (false == surface_at(Axis, Halfwidth, Point, Height))
		return false;
	double crossing{0.0};
	if (Surface != nullptr && Surface->at(Point, crossing))
		Height = crossing;
	return true;
}

void warp(std::vector<world_vertex> &Vertices, segment_data const &Axis, double const Halfwidth, crossing_surface const &Surface)
{
	for (auto &vertex : Vertices)
	{
		auto const point{plan(vertex.position)};
		double ruled{0.0}, height{0.0};
		// the edges of the piece too, a hair past its reach by the rounding
		if (false == surface_at(Axis, Halfwidth + 1e-4, point, ruled) || false == Surface.at(point, height))
			continue;
		// of the surface: a corner the cut made on an edge of a twisted piece lies on the chord, off the surface of the piece by up to a
		// couple of centimetres; a kerb stands higher
		if (std::abs(vertex.position.y - ruled) <= 0.05)
			vertex.position.y = height;
		else
			vertex.position.y += height - ruled;
	}
}

void conform(std::vector<world_vertex> &Vertices, segment_data const &Axis, double const Halfwidth, crossing_surface const &Surface)
{
	if (Surface.empty() || Vertices.size() < 3)
		return;
	auto const points{bezier(Axis)};
	// the ends of the piece: each a line across the road through an end of its axis, square to it there
	struct end_line
	{
		glm::dvec2 point, tangent, left;
		std::vector<double> stations; // where its edges are split, along the left from the point
	};
	std::vector<end_line> ends;
	for (auto const end : {0, 1})
	{
		auto const tangent{direction_at(points, end == 0 ? 0.0 : 1.0)};
		if (glm::length(tangent) < 1e-9)
			return;
		end_line line;
		line.point = plan(points[end == 0 ? 0 : 3]);
		line.tangent = tangent;
		line.left = {tangent.y, -tangent.x};
		ends.push_back(line);
	}
	auto const reach{Halfwidth + 0.5};
	// where an end, taken on past the road, passes the field side of the heads of a track on a side (1 its right): the surface bends there
	auto const passes = [&](end_line const &Line, rail_track const &Track, int const Side, double &Station) {
		auto const beyond = [&](double const At) {
			double along{0.0}, across{0.0};
			track_frame(Track, Line.point + Line.left * At, along, across);
			return across - Side * band(Track);
		};
		auto found{false};
		// across the road first, then as far as a track crossing the piece at a skew may pass its other end
		for (auto const range : {reach, reach + 30.0})
		{
			auto previous{beyond(-range)};
			for (double at = -range + 0.25; at <= range + 1e-9; at += 0.25)
			{
				auto const current{beyond(at)};
				if ((previous <= 0.0) != (current <= 0.0))
				{
					auto low{at - 0.25}, high{at};
					auto const below{previous <= 0.0};
					for (int i = 0; i < 60 && high - low > 1e-12; ++i)
					{
						auto const middle{0.5 * (low + high)};
						if ((beyond(middle) <= 0.0) == below)
							low = middle;
						else
							high = middle;
					}
					auto const station{0.5 * (low + high)};
					if (false == found || std::abs(station) < std::abs(Station))
						Station = station;
					found = true;
				}
				previous = current;
			}
			if (found)
				break;
		}
		return found;
	};
	auto const mix = [](world_vertex const &A, world_vertex const &B, double const Share) {
		world_vertex result;
		result.position = glm::mix(A.position, B.position, Share);
		result.normal = glm::mix(A.normal, B.normal, static_cast<float>(Share));
		result.texture = glm::mix(A.texture, B.texture, static_cast<float>(Share));
		return result;
	};
	// the lines the surface bends along, the field sides of the heads, each from where it passes the one end of the piece to where it
	// passes the other (the arc of a track is a hair from that chord over the length of a piece): the triangles cut along the whole of each,
	// so an edge it crosses is split at the same point in the triangles either side of it
	for (auto const &track : Surface.tracks)
		for (auto const side : {-1, 1})
		{
			double first{0.0}, last{0.0};
			if (false == passes(ends[0], track, side, first) || false == passes(ends[1], track, side, last))
				continue;
			if (std::abs(first) <= reach)
				ends[0].stations.push_back(first);
			if (std::abs(last) <= reach)
				ends[1].stations.push_back(last);
			auto const a{ends[0].point + ends[0].left * first}, b{ends[1].point + ends[1].left * last};
			if (glm::length(b - a) < 1e-6)
				continue;
			auto const side_of = [&](world_vertex const &P) { return (b.x - a.x) * (P.position.z - a.y) - (b.y - a.y) * (P.position.x - a.x); };
			std::vector<world_vertex> parts;
			parts.reserve(Vertices.size() + Vertices.size() / 4);
			for (std::size_t i = 0; i + 2 < Vertices.size(); i += 3)
			{
				std::vector<world_vertex> left, right;
				for (int k = 0; k < 3; ++k)
				{
					auto const &current{Vertices[i + k]};
					auto const &next{Vertices[i + (k + 1) % 3]};
					auto const here{side_of(current)}, there{side_of(next)};
					(here >= 0.0 ? left : right).push_back(current);
					if ((here >= 0.0) != (there >= 0.0))
					{
						auto const crossing{mix(current, next, here / (here - there))};
						left.push_back(crossing);
						right.push_back(crossing);
					}
				}
				for (auto const *polygon : {&left, &right})
					for (std::size_t k = 1; k + 1 < polygon->size(); ++k)
						parts.insert(parts.end(), {(*polygon)[0], (*polygon)[k], (*polygon)[k + 1]});
			}
			Vertices = std::move(parts);
		}
	// the joints: every joint_spacing from the axis, the same points for the piece on the other side, whichever way it goes
	for (auto &line : ends)
	{
		auto const count{static_cast<int>(std::ceil(reach / joint_spacing))};
		for (int k = -count; k <= count; ++k)
			line.stations.push_back(k * joint_spacing);
		std::sort(line.stations.begin(), line.stations.end());
	}
	std::vector<world_vertex> result;
	result.reserve(Vertices.size());
	for (std::size_t i = 0; i + 2 < Vertices.size(); i += 3)
	{
		std::array<world_vertex const *, 3> const corners{&Vertices[i], &Vertices[i + 1], &Vertices[i + 2]};
		auto const normal{glm::cross(corners[1]->position - corners[0]->position, corners[2]->position - corners[0]->position)};
		auto split{false};
		// the surface only: lying flat enough, as cut_gaps() takes it
		if (std::abs(normal.y) >= 0.5 * glm::length(normal))
			for (auto const &line : ends)
			{
				for (int e = 0; e < 3 && false == split; ++e)
				{
					auto const &a{*corners[e]};
					auto const &b{*corners[(e + 1) % 3]};
					auto const &c{*corners[(e + 2) % 3]};
					auto const offa{plan(a.position) - line.point}, offb{plan(b.position) - line.point};
					// an edge along the end of the piece
					if (std::abs(glm::dot(offa, line.tangent)) > 1e-5 || std::abs(glm::dot(offb, line.tangent)) > 1e-5)
						continue;
					auto const sa{glm::dot(offa, line.left)}, sb{glm::dot(offb, line.left)};
					if (std::abs(sb - sa) < 1e-6)
						continue;
					std::vector<double> shares;
					for (auto const station : line.stations)
					{
						auto const share{(station - sa) / (sb - sa)};
						// not a sliver beside a corner
						if (share * std::abs(sb - sa) > 1e-5 && (1.0 - share) * std::abs(sb - sa) > 1e-5)
							shares.push_back(share);
					}
					if (shares.empty())
						continue;
					std::sort(shares.begin(), shares.end());
					// a fan from the corner across the edge, wound as the triangle is
					auto previous{a};
					for (auto const share : shares)
					{
						auto const next{mix(a, b, share)};
						result.insert(result.end(), {previous, next, c});
						previous = next;
					}
					result.insert(result.end(), {previous, b, c});
					split = true;
				}
				if (split)
					break;
			}
		if (false == split)
			result.insert(result.end(), {*corners[0], *corners[1], *corners[2]});
	}
	Vertices = std::move(result);
}

// a path laid out along the axis of a road set onto the surface of a level crossing at its ends: there the heights of the surface, its
// slopes going into the path and its slope across it for the rolls; the control vectors keep their length in the plan
void fit(segment_data &Path, crossing_surface const &Crossing)
{
	auto &start{Path.points[segment_data::point::start]};
	auto &end{Path.points[segment_data::point::end]};
	glm::dvec2 const a{start.x, start.z}, b{end.x, end.z};
	if (glm::length(b - a) < 1e-3)
		return;
	double first{0.0}, last{0.0};
	if (false == Crossing.at(a, first) || false == Crossing.at(b, last))
		return;
	auto const &control1{Path.points[segment_data::point::control1]};
	auto const &control2{Path.points[segment_data::point::control2]};
	// a straight path has none: a third of the chord
	auto const out{glm::length(glm::dvec2{control1.x, control1.z}) > 1e-6 ? glm::dvec2{control1.x, control1.z} : (b - a) / 3.0};
	auto const in{glm::length(glm::dvec2{control2.x, control2.z}) > 1e-6 ? glm::dvec2{control2.x, control2.z} : (a - b) / 3.0};
	// rising going into the path from its start; at its end the slope coming to it, from the end looking back into the path
	auto const rise{Crossing.slope(a, out)};
	auto const fall{Crossing.slope(b, in)};
	start.y = first;
	end.y = last;
	Path.points[segment_data::point::control1] = {out.x, rise * glm::length(out), out.y};
	Path.points[segment_data::point::control2] = {in.x, fall * glm::length(in), in.y};
	// the roll: the surface across the path, as the road has it (its left up for a roll above 0)
	auto const roll = [&](glm::dvec2 const &Point, glm::dvec2 const &Along) {
		auto const forward{glm::normalize(Along)};
		glm::dvec2 const left{forward.y, -forward.x};
		double here{0.0}, there{0.0};
		if (false == Crossing.at(Point - left * 0.02, here) || false == Crossing.at(Point + left * 0.02, there))
			return 0.f;
		return static_cast<float>(glm::degrees(std::atan((there - here) / 0.04)));
	};
	Path.rolls = {roll(a, out), roll(b, -in)};
}

void weld(std::vector<world_vertex> &Vertices)
{
	for (auto &vertex : Vertices)
		vertex.position = glm::round(vertex.position * 100000.0) / 100000.0;
}

double crossing_parameter(segment_data const &Axis, rail_track const &Track)
{
	auto const points{bezier(Axis)};
	auto const lateral = [&](double const T) { return glm::dot(plan(bezier_point(points, T)) - plan(Track.point), Track.right()); };
	int constexpr samples{64};
	auto previous{lateral(0.0)};
	auto best{-1.0};
	auto bestdistance{std::numeric_limits<double>::max()};
	for (int i = 1; i <= samples; ++i)
	{
		auto const t{static_cast<double>(i) / samples};
		auto const current{lateral(t)};
		if ((previous <= 0.0) != (current <= 0.0))
		{
			auto a{t - 1.0 / samples}, b{t};
			auto fa{previous};
			for (int k = 0; k < 40 && b - a > 1e-9; ++k)
			{
				auto const m{0.5 * (a + b)};
				auto const fm{lateral(m)};
				if ((fa <= 0.0) == (fm <= 0.0))
				{
					a = m;
					fa = fm;
				}
				else
					b = m;
			}
			auto const at{0.5 * (a + b)};
			auto const d{glm::length(plan(bezier_point(points, at)) - plan(Track.point))};
			if (d < bestdistance)
			{
				bestdistance = d;
				best = at;
			}
		}
		previous = current;
	}
	// the straight taken for the track holds as far as the road reaches across it
	return bestdistance < 50.0 ? best : -1.0;
}

double groove_top_width(double const Flangeway)
{
	// the wall runs out by groove_run over the depth: measured 14 mm under the top, the groove is as wide as the flangeway
	return Flangeway + groove_run * flangeway_gauge_depth(1.435) / groove_depth;
}

road_cut cut(segment_data const &Axis, double const Halfwidth, std::vector<rail_track> const &Tracks, float const Texturelength, std::vector<panel> const &Panels, crossing_surface const *Surface)
{
	road_cut result;
	auto const points{bezier(Axis)};
	auto const tile{Texturelength > 0.f ? static_cast<double>(Texturelength) : 4.0};
	// two triangles A B C, A C D into the list, wound to face the side given
	auto const face = [&](std::vector<world_vertex> &Into, glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C, glm::dvec3 const &D, glm::dvec3 const &Facing) {
		for (auto const &triangle : {std::array<glm::dvec3, 3>{A, B, C}, std::array<glm::dvec3, 3>{A, C, D}})
		{
			auto normal{glm::cross(triangle[1] - triangle[0], triangle[2] - triangle[0])};
			if (glm::length(normal) < 1e-12)
				continue;
			normal = glm::normalize(normal);
			auto const flip{glm::dot(normal, Facing) < 0.0};
			if (flip)
				normal = -normal;
			auto const vertex = [&](glm::dvec3 const &P) {
				world_vertex entry;
				entry.position = P;
				entry.normal = glm::vec3{normal};
				entry.texture = glm::vec2{static_cast<float>(P.x / tile), static_cast<float>(P.z / tile)};
				return entry;
			};
			Into.push_back(vertex(triangle[0]));
			Into.push_back(vertex(flip ? triangle[2] : triangle[1]));
			Into.push_back(vertex(flip ? triangle[1] : triangle[2]));
		}
	};
	for (std::size_t k = 0; k < Tracks.size(); ++k)
	{
		auto const &track{Tracks[k]};
		if (track.turnout)
			continue;
		// where the axis crosses the track; one crossing it past an end of the piece, the rails still under its side, taken along its chord
		auto const t{crossing_parameter(Axis, track)};
		auto const direction{t >= 0.0 ? direction_at(points, t) : plan(points[3] - points[0])};
		if (glm::length(direction) < 1e-9)
			continue;
		// a road running nearly along the track is no crossing
		auto const sine{std::abs(glm::dot(glm::normalize(direction), track.right()))};
		if (sine < std::sin(glm::radians(15.0)))
			continue;
		auto const reach{Halfwidth / sine + 1.0};
		// the surface over the heads, along each rail as far as the piece reaches over it: in their plane. Often enough for a short piece
		// to be looked at too, which a rail crosses over a few centimetres at a skew
		auto const chord{glm::length(plan(points[3] - points[0]))};
		auto const often{std::clamp(0.25 * chord, 0.005, 0.1)};
		auto lies{true};
		auto samples{0};
		for (auto const side : {-1, 1})
			for (double along = -reach; along <= reach && lies; along += often)
			{
				auto const head{head_top(track, side, along, 0.5 * track.head)};
				double height{0.0};
				if (false == height_on(Axis, Halfwidth, Surface, plan(head), height))
					continue;
				lies = std::abs(head.y - height) <= cut_tolerance;
				++samples;
			}
		// a piece laid over the crossing of the track (it follows the surface of the crossing, which has the track) lies at its heads: the
		// plane of the heads where the axis crosses the track is off the heads of a curve by a millimetre or two along it at a skew
		if (Surface != nullptr && samples > 0 &&
		    std::any_of(Surface->tracks.begin(), Surface->tracks.end(), [&](rail_track const &Other) { return Other.source == track.source && Other.path == track.path; }))
			lies = true;
		if (false == lies || samples == 0)
			continue;
		result.tracks.push_back(k);
		auto const flangeway{flangeway_width(track.radius)};
		auto const top{groove_top_width(flangeway)};
		// how far along the track the panels lying on it reach, beside its rails
		double panelfrom{std::numeric_limits<double>::max()}, panelto{-std::numeric_limits<double>::max()};
		for (auto const &entry : Panels)
			for (auto const &triangle : entry.faces)
				for (auto const &corner : triangle)
				{
					auto const offset{plan(corner) - glm::dvec2{track.point.x, track.point.z}};
					auto const along{glm::dot(offset, track.along)};
					auto const across{std::abs(glm::dot(offset, track.right()))};
					if (std::abs(along) > 30.0 || across > 0.5 * track.gauge + track.head + 1.0)
						continue;
					panelfrom = std::min(panelfrom, along);
					panelto = std::max(panelto, along);
				}
		// along a bend the strips go in pieces short enough to stay on the rail within 0.1 mm
		auto const bend{std::abs(track.curvature)};
		auto const piece_length{bend > 1e-9 ? std::sqrt(0.0008 / bend) : 1e9};
		for (auto const &gap : rail_gaps(track.gauge, track.head, flangeway))
		{
			auto const side{gap.side};
			// across the rail from its gauge face, outwards positive: the top edge of the groove, the foot of its wall, its bottom reaching
			// a little under the head, the field side of the head
			auto const edge{-top}, foot{-(top - groove_run)}, under{groove_tuck}, field{track.head};
			auto const corner = [&](double const Along, double const Out, double const Drop) { return head_top(track, side, Along, Out) - glm::dvec3{0.0, Drop, 0.0}; };
			// the stretch of the rail the road lies over: where the top of the groove or the head is on its surface
			auto const on_road = [&](double const Along) {
				double height{0.0};
				for (auto const out : {edge, 0.5 * (edge + under), field})
					if (surface_at(Axis, Halfwidth, plan(corner(Along, out, 0.0)), height))
						return true;
				return false;
			};
			auto low{reach}, high{-reach};
			for (double along = -reach; along <= reach; along += 0.01)
				if (on_road(along))
				{
					low = std::min(low, along);
					high = std::max(high, along);
				}
			if (high <= low)
				continue;
			// to the edge of the road, the last step of the scan past it
			low -= 0.01;
			high += 0.01;
			// and past the road as far as the panels lie along the rail
			auto const from{panelto > panelfrom ? std::min(low, panelfrom - 0.02) : low};
			auto const to{panelto > panelfrom ? std::max(high, panelto + 0.02) : high};
			auto const segments = [&](double const From, double const To) {
				return std::max(1, static_cast<int>(std::ceil((To - From) / piece_length - 1e-9)));
			};
			// the hole: the top of the groove and the head, the surface reaching half a millimetre under the field side of the head, so
			// the chords along a bend leave no slit beside it
			{
				auto const a0{from}, a1{to};
				auto const count{segments(a0, a1)};
				for (int i = 0; i < count; ++i)
				{
					auto const a{a0 + (a1 - a0) * i / count}, b{a0 + (a1 - a0) * (i + 1) / count};
					result.holes.push_back({plan(corner(a, edge, 0.0)), plan(corner(b, edge, 0.0)), plan(corner(b, field - 0.0005, 0.0)), plan(corner(a, field - 0.0005, 0.0))});
				}
			}
			// the groove: its wall going down from the edge of the surface, its bottom closed, its ends closed
			{
				auto const count{segments(from, to)};
				for (int i = 0; i < count; ++i)
				{
					auto const a{from + (to - from) * i / count}, b{from + (to - from) * (i + 1) / count};
					glm::dvec2 right;
					track.axis_at(0.5 * (a + b), right);
					// the wall faces the rail and up, the bottom up
					glm::dvec3 const towards{right.x * side, 1.0, right.y * side};
					face(result.grooves, corner(a, edge, 0.0), corner(b, edge, 0.0), corner(b, foot, groove_depth), corner(a, foot, groove_depth), towards);
					face(result.grooves, corner(a, foot, groove_depth), corner(b, foot, groove_depth), corner(b, under, groove_depth), corner(a, under, groove_depth), glm::dvec3{0.0, 1.0, 0.0});
				}
				for (auto const end : {from, to})
				{
					glm::dvec2 right;
					track.axis_at(end, right);
					glm::dvec2 const tangent{right.y, -right.x};
					auto const outwards{end == from ? -1.0 : 1.0};
					face(result.grooves, corner(end, edge, 0.0), corner(end, foot, groove_depth), corner(end, under, groove_depth), corner(end, under, 0.0),
					     glm::dvec3{tangent.x * outwards, 0.0, tangent.y * outwards});
				}
			}
			// past the road: the strips beside the rail filled at the plane of the heads, inwards from the groove and outwards from the head, as
			// far as a panel might leave open there; the panels cut them where they lie
			for (auto const &[a0, a1] : {std::pair{from, low}, std::pair{high, to}})
			{
				if (a1 - a0 < 0.005)
					continue;
				auto const count{segments(a0, a1)};
				for (int i = 0; i < count; ++i)
				{
					auto const a{a0 + (a1 - a0) * i / count}, b{a0 + (a1 - a0) * (i + 1) / count};
					face(result.fills, corner(a, edge - 0.4, 0.0), corner(b, edge - 0.4, 0.0), corner(b, edge, 0.0), corner(a, edge, 0.0), glm::dvec3{0.0, 1.0, 0.0});
					face(result.fills, corner(a, field, 0.0), corner(b, field, 0.0), corner(b, field + 0.4, 0.0), corner(a, field + 0.4, 0.0), glm::dvec3{0.0, 1.0, 0.0});
				}
			}
		}
	}
	return result;
}

std::vector<world_vertex> skirts(std::vector<world_vertex> const &Surface, std::vector<std::vector<glm::dvec2>> const &Covers, double const Depth)
{
	std::vector<world_vertex> result;
	if (Covers.empty())
		return result;
	// the edges of the surface used by one of its triangles alone
	auto const key = [](glm::dvec3 const &P) { return std::pair<long long, long long>{std::llround(P.x * 20000.0), std::llround(P.z * 20000.0)}; };
	std::map<std::pair<std::pair<long long, long long>, std::pair<long long, long long>>, int> edges;
	for (std::size_t i = 0; i + 2 < Surface.size(); i += 3)
		for (int k = 0; k < 3; ++k)
		{
			auto a{key(Surface[i + k].position)}, b{key(Surface[i + (k + 1) % 3].position)};
			if (b < a)
				std::swap(a, b);
			++edges[{a, b}];
		}
	auto const along_cover = [&](glm::dvec2 const &P) {
		for (auto const &cover : Covers)
			for (std::size_t m = 0; m < cover.size(); ++m)
			{
				auto const &a{cover[m]};
				auto const &b{cover[(m + 1) % cover.size()]};
				auto const ab{b - a};
				auto const length2{glm::dot(ab, ab)};
				if (length2 < 1e-12)
					continue;
				auto const t{std::clamp(glm::dot(P - a, ab) / length2, 0.0, 1.0)};
				if (glm::length(P - (a + ab * t)) < 0.0005)
					return true;
			}
		return false;
	};
	glm::dvec3 const down{0.0, Depth, 0.0};
	for (std::size_t i = 0; i + 2 < Surface.size(); i += 3)
		for (int k = 0; k < 3; ++k)
		{
			auto const &a{Surface[i + k]};
			auto const &b{Surface[i + (k + 1) % 3]};
			auto const &c{Surface[i + (k + 2) % 3]};
			auto ka{key(a.position)}, kb{key(b.position)};
			if (kb < ka)
				std::swap(ka, kb);
			if (edges[{ka, kb}] != 1)
				continue;
			// along a panel over any of its length: a corner of the panel on the edge, where the triangle beside wasn't cut at it (a T),
			// leaves the edge along the panel only from that corner on
			auto along{false};
			for (int m = 0; m <= 8 && false == along; ++m)
				along = along_cover(glm::mix(plan(a.position), plan(b.position), m / 8.0));
			if (false == along)
				continue;
			// facing away from the surface, towards the panel
			auto const edge{plan(b.position) - plan(a.position)};
			glm::dvec2 normal{-edge.y, edge.x};
			if (glm::dot(normal, plan(c.position) - plan(a.position)) > 0.0)
				normal = -normal;
			if (glm::length(normal) < 1e-12)
				continue;
			normal = glm::normalize(normal);
			glm::dvec3 const facing{normal.x, 0.0, normal.y};
			// its foot under the road, away from the panel: a ray slipping down a crack between the edge of the road and the edge of the
			// panel, however thin, meets the side of the panel going one way and the skirt going the other
			auto const foot{-facing * (0.4 * Depth) - down};
			for (auto const &triangle : {std::array<glm::dvec3, 3>{a.position, b.position, b.position + foot}, std::array<glm::dvec3, 3>{a.position, b.position + foot, a.position + foot}})
			{
				auto const n{glm::cross(triangle[1] - triangle[0], triangle[2] - triangle[0])};
				auto const flip{glm::dot(n, facing) < 0.0};
				for (int m : {0, flip ? 2 : 1, flip ? 1 : 2})
					result.push_back({triangle[m], glm::vec3{facing}, a.texture});
			}
		}
	return result;
}

std::vector<std::array<glm::dvec3, 3>> rail_heads(rail_track const &Track, double const From, double const To)
{
	std::vector<std::array<glm::dvec3, 3>> result;
	auto const bend{std::abs(Track.curvature)};
	auto const step{bend > 1e-9 ? std::min(0.5, std::sqrt(0.008 / bend)) : 0.5};
	auto const count{std::max(1, static_cast<int>(std::ceil((To - From) / step)))};
	auto const quad = [&](glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C, glm::dvec3 const &D) {
		result.push_back({A, B, C});
		result.push_back({A, C, D});
	};
	glm::dvec3 const down{0.0, 0.05, 0.0};
	for (auto const side : {-1, 1})
		for (int i = 0; i < count; ++i)
		{
			auto const a{From + (To - From) * i / count}, b{From + (To - From) * (i + 1) / count};
			auto const ga{head_top(Track, side, a, 0.0)}, gb{head_top(Track, side, b, 0.0)};
			auto const fa{head_top(Track, side, a, Track.head)}, fb{head_top(Track, side, b, Track.head)};
			quad(ga, gb, fb, fa);
			quad(ga, gb, gb - down, ga - down);
			quad(fa, fb, fb - down, fa - down);
		}
	return result;
}

namespace
{

// where a triangle is over the point of the plan, its height there
bool height_in(std::array<glm::dvec3, 3> const &T, glm::dvec2 const &P, double &Height)
{
	auto const &a{T[0]};
	auto const &b{T[1]};
	auto const &c{T[2]};
	auto const d{(b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z)};
	if (std::abs(d) < 1e-14)
		return false;
	auto const u{((b.z - c.z) * (P.x - c.x) + (c.x - b.x) * (P.y - c.z)) / d};
	auto const v{((c.z - a.z) * (P.x - c.x) + (a.x - c.x) * (P.y - c.z)) / d};
	auto const w{1.0 - u - v};
	if (u < -1e-9 || v < -1e-9 || w < -1e-9)
		return false;
	Height = u * a.y + v * b.y + w * c.y;
	return true;
}

// the distance along the ray to the triangle, both of its faces; -1 for none
double ray_hit(glm::dvec3 const &Origin, glm::dvec3 const &Direction, std::array<glm::dvec3, 3> const &T)
{
	auto const e1{T[1] - T[0]};
	auto const e2{T[2] - T[0]};
	auto const p{glm::cross(Direction, e2)};
	auto const det{glm::dot(e1, p)};
	if (std::abs(det) < 1e-14)
		return -1.0;
	auto const inverse{1.0 / det};
	auto const s{Origin - T[0]};
	auto const u{glm::dot(s, p) * inverse};
	if (u < -1e-9 || u > 1.0 + 1e-9)
		return -1.0;
	auto const q{glm::cross(s, e1)};
	auto const v{glm::dot(Direction, q) * inverse};
	if (v < -1e-9 || u + v > 1.0 + 1e-9)
		return -1.0;
	return glm::dot(e2, q) * inverse;
}

} // namespace

std::vector<defect> check(built_crossing const &Built, glm::dvec2 const &Low, glm::dvec2 const &High, double const Step, std::size_t const Most, std::size_t *Cracks)
{
	std::vector<defect> result;
	auto const add = [&](glm::dvec3 const &Point, char const *Kind) {
		if (result.size() < Most)
			result.push_back({Point, Kind});
	};
	// the triangles by what they are, in the cells of a grid over the plan
	enum kind : int
	{
		road_kind,
		panel_kind,
		rail_kind
	};
	struct entry
	{
		std::array<glm::dvec3, 3> triangle;
		int kind;
	};
	std::vector<entry> all;
	for (auto const &triangle : Built.road)
		all.push_back({triangle, road_kind});
	for (auto const &triangle : Built.panels)
		all.push_back({triangle, panel_kind});
	for (auto const &triangle : Built.rails)
		all.push_back({triangle, rail_kind});
	double constexpr cellsize{0.25};
	auto const origin{Low - glm::dvec2{5.0}};
	auto const columns{static_cast<int>(std::ceil((High.x - Low.x + 10.0) / cellsize)) + 1};
	auto const rows{static_cast<int>(std::ceil((High.y - Low.y + 10.0) / cellsize)) + 1};
	std::vector<std::vector<std::uint32_t>> cells(static_cast<std::size_t>(columns) * rows);
	auto const column = [&](double const X) { return std::clamp(static_cast<int>(std::floor((X - origin.x) / cellsize)), 0, columns - 1); };
	auto const row = [&](double const Z) { return std::clamp(static_cast<int>(std::floor((Z - origin.y) / cellsize)), 0, rows - 1); };
	for (std::uint32_t i = 0; i < all.size(); ++i)
	{
		auto const &t{all[i].triangle};
		auto const x0{std::min({t[0].x, t[1].x, t[2].x})}, x1{std::max({t[0].x, t[1].x, t[2].x})};
		auto const z0{std::min({t[0].z, t[1].z, t[2].z})}, z1{std::max({t[0].z, t[1].z, t[2].z})};
		if (x1 < origin.x || z1 < origin.y || x0 > origin.x + columns * cellsize || z0 > origin.y + rows * cellsize)
			continue;
		for (int r = row(z0); r <= row(z1); ++r)
			for (int c = column(x0); c <= column(x1); ++c)
				cells[static_cast<std::size_t>(r) * columns + c].push_back(i);
	}
	// the highest of the triangles of a kind (-1: any) over the point; Kind receives what it is
	// the triangles lying flat, for the tops of the panels
	std::vector<char> flat(all.size(), 0);
	for (std::size_t i = 0; i < all.size(); ++i)
	{
		auto const &t{all[i].triangle};
		auto const normal{glm::cross(t[1] - t[0], t[2] - t[0])};
		// facing up: the top of a panel, not its bottom showing in a joint between two of them
		flat[i] = normal.y >= 0.7 * glm::length(normal) ? 1 : 0;
	}
	auto const highest = [&](glm::dvec2 const &P, int const Of, double &Height, int *Kind = nullptr, bool const Flat = false) {
		auto found{false};
		for (auto const index : cells[static_cast<std::size_t>(row(P.y)) * columns + column(P.x)])
		{
			auto const &item{all[index]};
			if (Of >= 0 && item.kind != Of)
				continue;
			if (Flat && flat[index] == 0)
				continue;
			double height{0.0};
			if (height_in(item.triangle, P, height) && (false == found || height > Height))
			{
				Height = height;
				found = true;
				if (Kind != nullptr)
					*Kind = item.kind;
			}
		}
		return found;
	};
	// the footprint of the road: the pieces, with the boxes around them to skip the far ones fast
	struct piece
	{
		segment_data const *axis;
		double halfwidth;
		glm::dvec2 low, high;
	};
	std::vector<piece> pieces;
	for (auto const &[axis, halfwidth] : Built.footprint)
	{
		auto const p{bezier(axis)};
		glm::dvec2 low{std::numeric_limits<double>::max()}, high{-std::numeric_limits<double>::max()};
		for (auto const &point : p)
		{
			low = glm::min(low, plan(point));
			high = glm::max(high, plan(point));
		}
		// the road lays its cross-section turned by the roll: its edges come in by the cosine of it in the plan
		auto const roll{glm::radians(static_cast<double>(std::max(std::abs(axis.rolls[0]), std::abs(axis.rolls[1]))))};
		pieces.push_back({&axis, halfwidth * std::cos(roll), low - glm::dvec2{halfwidth + 0.1}, high + glm::dvec2{halfwidth + 0.1}});
	}
	auto const on_road = [&](glm::dvec2 const &P) {
		for (auto const &entry : pieces)
		{
			if (P.x < entry.low.x || P.y < entry.low.y || P.x > entry.high.x || P.y > entry.high.y)
				continue;
			double height{0.0};
			if (surface_at(*entry.axis, entry.halfwidth, P, height))
				return true;
		}
		return false;
	};
	// a ray straight down at a point meeting nothing through a crack, not a hole: the corners of the road are welded to a grid of 0.01 mm
	// (road_node::create_shapes), and a corner the cuts along the rails and under the panels make on an edge of a triangle, which the
	// triangle beside it doesn't have (a T), goes a few micrometres off that edge with it, a line between them that much wide which a ray
	// right on it slips through. Something under the ray 0.02 mm aside both ways, across the line whichever way it goes (eight ways
	// tried: one of them is within 11 degrees of square to it)
	std::size_t cracks{0};
	auto const crack = [&](glm::dvec2 const &P) {
		double height{0.0};
		for (int k = 0; k < 8; ++k)
		{
			auto const angle{k * glm::pi<double>() / 8.0};
			glm::dvec2 const aside{std::cos(angle) * crack_width, std::sin(angle) * crack_width};
			if (highest(P + aside, -1, height) && highest(P - aside, -1, height))
			{
				++cracks;
				return true;
			}
		}
		return false;
	};

	// straight down over the footprint of the road: something under each ray, and no road over a panel within its footprint
	for (double x = Low.x; x <= High.x + 1e-9; x += Step)
		for (double z = Low.y; z <= High.y + 1e-9; z += Step)
		{
			glm::dvec2 const point{x, z};
			// within the road by 2 mm: a point on its outer edge, or past its end within the rounding, has nothing under it rightly
			if (false == on_road(point) || false == on_road(point + glm::dvec2{0.002, 0.0}) || false == on_road(point - glm::dvec2{0.002, 0.0}) ||
			    false == on_road(point + glm::dvec2{0.0, 0.002}) || false == on_road(point - glm::dvec2{0.0, 0.002}))
				continue;
			double height{0.0};
			if (false == highest(point, -1, height))
			{
				if (false == crack(point))
					add({x, 0.0, z}, "hole");
				continue;
			}
			// the top of a panel: its faces lying flat, not the sides of its ends going down under the joint; of the road the faces lying
			// flat as well, which would fight with it for the same pixels, not the steep wall of a groove going on under its edge
			// a millimetre within the top, not on its edge, where the road cut along it meets it to the rounding
			double panel{0.0}, road{0.0}, unused{0.0};
			auto const within{highest(point + glm::dvec2{0.001, 0.0}, panel_kind, unused, nullptr, true) && highest(point - glm::dvec2{0.001, 0.0}, panel_kind, unused, nullptr, true) &&
			                  highest(point + glm::dvec2{0.0, 0.001}, panel_kind, unused, nullptr, true) && highest(point - glm::dvec2{0.0, 0.001}, panel_kind, unused, nullptr, true)};
			if (within && highest(point, panel_kind, panel, nullptr, true) && highest(point, road_kind, road, nullptr, true) && road > panel - 0.001)
				add({x, road, z}, "over");
		}

	// beside every edge of the surface of the road every centimetre, 3 mm off it: something under the ray, where it isn't the road
	{
		std::map<std::pair<std::pair<long long, long long>, std::pair<long long, long long>>, int> edges;
		auto const key = [](glm::dvec3 const &P) { return std::pair<long long, long long>{std::llround(P.x * 2000.0), std::llround(P.z * 2000.0)}; };
		for (auto const &triangle : Built.road)
			for (int i = 0; i < 3; ++i)
			{
				auto a{key(triangle[i])}, b{key(triangle[(i + 1) % 3])};
				if (b < a)
					std::swap(a, b);
				++edges[{a, b}];
			}
		for (auto const &triangle : Built.road)
			for (int i = 0; i < 3; ++i)
			{
				auto a{key(triangle[i])}, b{key(triangle[(i + 1) % 3])};
				if (b < a)
					std::swap(a, b);
				if (edges[{a, b}] != 1)
					continue;
				auto const from{plan(triangle[i])}, to{plan(triangle[(i + 1) % 3])};
				auto const length{glm::length(to - from)};
				if (length < 0.002)
					continue;
				auto const along{(to - from) / length};
				glm::dvec2 const normal{-along.y, along.x};
				for (double s = 0.005; s < length; s += 0.01)
					for (auto const sign : {-1.0, 1.0})
					{
						auto const probe{from + along * s + normal * (0.003 * sign)};
						if (probe.x < Low.x || probe.y < Low.y || probe.x > High.x || probe.y > High.y || false == on_road(probe))
							continue;
						double height{0.0};
						if (highest(probe, road_kind, height))
							continue;
						if (false == highest(probe, -1, height))
						{
							// a crack under 0.02 mm, as for the holes above: something under the ray 0.02 mm aside each way
							if (crack(probe))
								continue;
							add({probe.x, triangle[i].y, probe.y}, "seam");
						}
					}
			}
	}

	// at slants of 10 and 30 degrees along the roads and along the tracks, both ways, over the panels and the rails: each ray aimed just
	// under the surface meets something before it gets deeper than a groove
	{
		std::vector<glm::dvec2> directions;
		for (auto const &entry : Built.footprint)
		{
			auto const p{bezier(entry.first)};
			auto const d{plan(p[3] - p[0])};
			if (glm::length(d) > 1e-6)
			{
				directions.push_back(glm::normalize(d));
				directions.push_back(-glm::normalize(d));
			}
		}
		for (auto const &track : Built.tracks)
		{
			directions.push_back(track.along);
			directions.push_back(-track.along);
		}
		// the directions differing by less than 5 degrees taken once
		std::vector<glm::dvec2> distinct;
		for (auto const &d : directions)
			if (std::none_of(distinct.begin(), distinct.end(), [&](glm::dvec2 const &Other) { return glm::dot(Other, d) > std::cos(glm::radians(5.0)); }))
				distinct.push_back(d);
		glm::dvec2 low{std::numeric_limits<double>::max()}, high{-std::numeric_limits<double>::max()};
		for (auto const *list : {&Built.panels, &Built.rails})
			for (auto const &triangle : *list)
				for (auto const &corner : triangle)
				{
					low = glm::min(low, plan(corner));
					high = glm::max(high, plan(corner));
				}
		low = glm::max(low - glm::dvec2{0.5}, Low);
		high = glm::min(high + glm::dvec2{0.5}, High);
		auto const step{std::max(Step, 0.05)};
		for (double x = low.x; x <= high.x + 1e-9; x += step)
			for (double z = low.y; z <= high.y + 1e-9; z += step)
			{
				glm::dvec2 const point{x, z};
				double height{0.0};
				if (false == on_road(point) || false == highest(point, -1, height))
					continue;
				glm::dvec3 const target{x, height - 0.002, z};
				for (auto const elevation : {10.0, 30.0})
					for (auto const &d : distinct)
					{
						// from over the road: a ray from past its edge sees under it, as the ground does there
						if (false == on_road(point - d * 0.1))
							continue;
						auto const e{glm::radians(elevation)};
						glm::dvec3 const direction{d.x * std::cos(e), -std::sin(e), d.y * std::cos(e)};
						auto const back{2.0};
						auto const beyond{0.045 / std::sin(e)};
						auto const meets = [&](glm::dvec3 const &Target) {
							auto const origin{Target - direction * back};
							// the cells along the ray in the plan
							std::vector<std::uint32_t> seen;
							// the cells around each point along it, so a ray cutting the corner of a cell meets what's in it too
							std::vector<std::size_t> visited;
							for (double s = 0.0; s <= back + beyond + 0.1; s += 0.1)
							{
								auto const p{origin + direction * s};
								for (int dr = -1; dr <= 1; ++dr)
									for (int dc = -1; dc <= 1; ++dc)
									{
										auto const r{std::clamp(row(p.z) + dr, 0, rows - 1)};
										auto const c{std::clamp(column(p.x) + dc, 0, columns - 1)};
										auto const cell{static_cast<std::size_t>(r) * columns + c};
										if (std::find(visited.begin(), visited.end(), cell) != visited.end())
											continue;
										visited.push_back(cell);
										for (auto const index : cells[cell])
										{
											if (std::find(seen.begin(), seen.end(), index) != seen.end())
												continue;
											seen.push_back(index);
											auto const t{ray_hit(origin, direction, all[index].triangle)};
											if (t >= 0.0 && t <= back + beyond)
												return true;
										}
									}
							}
							return false;
						};
						if (meets(target))
							continue;
						// through a crack under 0.02 mm (see crack() above), not a slit: the ray moved 0.02 mm along the road or the track it goes
						// along, or aside, meets something; through a slit it doesn't, the slit wider than that. Moved so, the ray passes the
						// surface 0.02 mm away, at the same height: a step at a joint lets it through as well
						glm::dvec3 const along{d.x, 0.0, d.y};
						glm::dvec3 const aside{-d.y, 0.0, d.x};
						auto through{false};
						for (auto const &shift : {along, -along, aside, -aside})
							if (meets(target + shift * crack_width))
							{
								through = true;
								break;
							}
						if (through)
							++cracks;
						else
							add(target, "slit");
					}
			}
	}

	// down the middle of each flangeway every centimetre, along the road and the panels beside it: the bottom of the groove at its depth,
	// nothing over it
	for (auto const &track : Built.tracks)
	{
		auto const flangeway{flangeway_width(track.radius)};
		auto const top{groove_top_width(flangeway)};
		auto const middle{0.5 * (-(top - groove_run) + groove_tuck)};
		for (auto const side : {-1, 1})
			for (double along = -20.0; along <= 20.0; along += 0.01)
			{
				auto const centre{head_top(track, side, along, middle)};
				double height{0.0};
				auto const beside{highest(plan(head_top(track, side, along, -0.15)), panel_kind, height) || highest(plan(head_top(track, side, along, track.head + 0.15)), panel_kind, height)};
				if (false == beside && false == on_road(plan(centre)))
					continue;
				int what{-1};
				if (false == highest(plan(centre), -1, height, &what) || what != road_kind || std::abs(height - (centre.y - groove_depth)) > 0.002)
					add({centre.x, centre.y, centre.z}, "flangeway");
			}
	}
	if (Cracks != nullptr)
		*Cracks = cracks;
	return result;
}

std::vector<std::vector<glm::dvec2>> gaps(segment_data const &Axis, double const Halfwidth, std::vector<rail_track> const &Tracks, std::vector<std::size_t> *Cut)
{
	auto result{cut(Axis, Halfwidth, Tracks)};
	if (Cut != nullptr)
		*Cut = result.tracks;
	return std::move(result.holes);
}

std::vector<glm::dvec2> hull(std::vector<glm::dvec2> Points)
{
	std::sort(Points.begin(), Points.end(), [](glm::dvec2 const &A, glm::dvec2 const &B) { return A.x < B.x || (A.x == B.x && A.y < B.y); });
	Points.erase(std::unique(Points.begin(), Points.end(), [](glm::dvec2 const &A, glm::dvec2 const &B) { return glm::length(A - B) < 1e-6; }), Points.end());
	if (Points.size() < 3)
		return Points;
	std::vector<glm::dvec2> result(2 * Points.size());
	std::size_t k{0};
	for (std::size_t i = 0; i < Points.size(); ++i)
	{
		while (k >= 2 && plan_cross(result[k - 1] - result[k - 2], Points[i] - result[k - 2]) <= 0.0)
			--k;
		result[k++] = Points[i];
	}
	for (std::size_t i = Points.size() - 1, t = k + 1; i-- > 0;)
	{
		while (k >= t && plan_cross(result[k - 1] - result[k - 2], Points[i] - result[k - 2]) <= 0.0)
			--k;
		result[k++] = Points[i];
	}
	result.resize(k - 1);
	return result;
}

std::vector<std::vector<glm::dvec2>> covers(segment_data const &Axis, double const Halfwidth, std::vector<panel> const &Panels, crossing_surface const *Surface)
{
	std::vector<std::vector<glm::dvec2>> result;
	auto const outline{footprint(Axis, Halfwidth)};
	glm::dvec2 low{std::numeric_limits<double>::max()}, high{-std::numeric_limits<double>::max()};
	for (auto const &corner : outline)
	{
		low = glm::min(low, corner);
		high = glm::max(high, corner);
	}
	for (auto const &entry : Panels)
	{
		if (entry.outline.size() < 3)
			continue;
		glm::dvec2 a{std::numeric_limits<double>::max()}, b{-std::numeric_limits<double>::max()};
		for (auto const &corner : entry.outline)
		{
			a = glm::min(a, corner);
			b = glm::max(b, corner);
		}
		if (b.x < low.x || a.x > high.x || b.y < low.y || a.y > high.y)
			continue;
		// the part of the panel over the piece, however small: the corner of a panel at a skew reaches over the short piece next to the
		// one the panel lies on mostly
		auto const overlap{clip(outline, entry.outline)};
		if (overlap.size() < 3 || std::abs(plan_area(overlap)) < 1e-8)
			continue;
		glm::dvec2 middle{0.0};
		for (auto const &corner : overlap)
			middle += corner;
		middle /= static_cast<double>(overlap.size());
		std::vector<glm::dvec2> probes{middle};
		for (auto const &corner : overlap)
			probes.push_back(corner + (middle - corner) * 0.2);
		// lying at the surface there: the top of the panel at it or up to 0.3 m over it, a slab, not something standing on the road
		auto covered{false};
		for (auto const &probe : probes)
		{
			double road{0.0}, top{0.0};
			if (false == height_on(Axis, Halfwidth, Surface, probe, road) || false == top_at(entry, probe, top))
				continue;
			if (top >= road - 0.02 && top <= road + 0.3)
			{
				covered = true;
				break;
			}
		}
		if (false == covered)
			continue;
		if (entry.faces.empty())
			result.push_back(entry.outline);
		else
			for (auto const &face : entry.faces)
				result.push_back({plan(face[0]), plan(face[1]), plan(face[2])});
	}
	return result;
}

void cut_gaps(std::vector<world_vertex> &Vertices, std::vector<std::vector<glm::dvec2>> const &Gaps, std::vector<world_vertex> *Bridges)
{
	struct gap
	{
		std::vector<glm::dvec2> outline; // counterclockwise
		glm::dvec2 low, high;
	};
	std::vector<gap> gaps;
	for (auto const &outline : Gaps)
	{
		if (outline.size() < 3)
			continue;
		gap entry;
		entry.outline = outline;
		double area{0.0};
		for (std::size_t i = 0; i < outline.size(); ++i)
		{
			auto const &a{outline[i]};
			auto const &b{outline[(i + 1) % outline.size()]};
			area += a.x * b.y - b.x * a.y;
		}
		if (area < 0.0)
			std::reverse(entry.outline.begin(), entry.outline.end());
		entry.low = entry.high = outline.front();
		for (auto const &point : outline)
		{
			entry.low = glm::min(entry.low, point);
			entry.high = glm::max(entry.high, point);
		}
		gaps.push_back(std::move(entry));
	}
	if (gaps.empty())
		return;
	using polygon = std::vector<world_vertex>;
	// above 0 on the left of the line from A to B, which is inside a counterclockwise outline
	auto const side = [](glm::dvec2 const &A, glm::dvec2 const &B, world_vertex const &P) { return (B.x - A.x) * (P.position.z - A.y) - (B.y - A.y) * (P.position.x - A.x); };
	std::vector<world_vertex> result;
	result.reserve(Vertices.size());
	for (std::size_t i = 0; i + 2 < Vertices.size(); i += 3)
	{
		std::vector<polygon> pieces{{Vertices[i], Vertices[i + 1], Vertices[i + 2]}};
		auto const normal{glm::cross(Vertices[i + 1].position - Vertices[i].position, Vertices[i + 2].position - Vertices[i].position)};
		if (std::abs(normal.y) >= 0.5 * glm::length(normal))
		{
			glm::dvec2 low{Vertices[i].position.x, Vertices[i].position.z};
			auto high{low};
			for (int k = 1; k < 3; ++k)
			{
				glm::dvec2 const point{Vertices[i + k].position.x, Vertices[i + k].position.z};
				low = glm::min(low, point);
				high = glm::max(high, point);
			}
			for (auto const &entry : gaps)
			{
				if (entry.low.x > high.x || entry.high.x < low.x || entry.low.y > high.y || entry.high.y < low.y)
					continue;
				std::vector<polygon> kept;
				for (auto const &piece : pieces)
				{
					auto inside{piece};
					for (std::size_t e = 0; e < entry.outline.size() && inside.size() >= 3; ++e)
					{
						auto const &a{entry.outline[e]};
						auto const &b{entry.outline[(e + 1) % entry.outline.size()]};
						polygon in, out;
						for (std::size_t k = 0; k < inside.size(); ++k)
						{
							auto const &current{inside[k]};
							auto const &next{inside[(k + 1) % inside.size()]};
							auto const here{side(a, b, current)};
							auto const there{side(a, b, next)};
							(here >= 0.0 ? in : out).push_back(current);
							if ((here >= 0.0) != (there >= 0.0))
							{
								auto const share{here / (here - there)};
								world_vertex crossing;
								crossing.position = glm::mix(current.position, next.position, share);
								crossing.normal = glm::mix(current.normal, next.normal, static_cast<float>(share));
								crossing.texture = glm::mix(current.texture, next.texture, static_cast<float>(share));
								in.push_back(crossing);
								out.push_back(crossing);
							}
						}
						// a sliver the rounding leaves along an edge of the gap isn't a part of the surface
						auto area{0.0};
						for (std::size_t m = 0; m < out.size(); ++m)
						{
							auto const &p{out[m].position};
							auto const &q{out[(m + 1) % out.size()].position};
							area += p.x * q.z - q.x * p.z;
						}
						if (out.size() >= 3 && std::abs(area) > 2e-7)
							kept.push_back(std::move(out));
						inside = std::move(in);
					}
					// what's left inside all the lines is the gap: the surface bridging it, for the heights
					if (Bridges != nullptr && inside.size() >= 3)
						for (std::size_t k = 1; k + 1 < inside.size(); ++k)
						{
							Bridges->push_back(inside[0]);
							Bridges->push_back(inside[k]);
							Bridges->push_back(inside[k + 1]);
						}
				}
				pieces = std::move(kept);
			}
		}
		for (auto const &piece : pieces)
			for (std::size_t k = 1; k + 1 < piece.size(); ++k)
			{
				result.push_back(piece[0]);
				result.push_back(piece[k]);
				result.push_back(piece[k + 1]);
			}
	}
	Vertices = std::move(result);
}

crossing_build build(std::vector<std::vector<world_vertex> *> const &Batches, segment_data const &Axis, double const Reach, std::vector<rail_track> const &Rails, float const Texturelength,
                     std::vector<panel> const &Panels, crossing_surface const *Surface)
{
	crossing_build result;
	auto crossing{cut(Axis, Reach, Rails, Texturelength, Panels, Surface)};
	result.tracks = crossing.tracks;
	result.holes = std::move(crossing.holes);
	// onto the surface of the crossing first, split where it bends and at the joints (conform()), so what's cut out of it after, along the
	// rails and under the panels, makes its corners on it: cut first and set onto it after, a corner made on an edge left whole in the
	// triangle beside went off that edge as far as the surface bends along it, a slit beside it
	if (Surface != nullptr)
		for (auto *batch : Batches)
		{
			conform(*batch, Axis, Reach, *Surface);
			warp(*batch, Axis, Reach, *Surface);
		}
	// what's cut out of the surface stays, apart, as the surface over the gaps for the heights
	if (false == result.holes.empty())
		for (auto *batch : Batches)
			cut_gaps(*batch, result.holes, &result.bridges);
	// the crossing panels lying on the road, over all of their footprint: the surface under them left out, the panel is the surface there.
	// A piece next to the rails which a panel reaches over too, not only the one over the rails
	result.covers = covers(Axis, Reach, Panels, Surface);
	if (false == result.covers.empty())
		for (auto *batch : Batches)
		{
			cut_gaps(*batch, result.covers);
			// hanging down from the edges along the panels, so nothing shows under the edge of the road
			auto const skirt{skirts(*batch, result.covers)};
			batch->insert(batch->end(), skirt.begin(), skirt.end());
		}
	// the grooves beside the heads: closed channels along the road and the panels, the panels making the flangeway with their sides over
	// them; the strips beside the rails past the road filled where no panel lies
	result.grooves = std::move(crossing.grooves);
	result.fills = std::move(crossing.fills);
	if (false == result.fills.empty())
	{
		std::vector<std::vector<glm::dvec2>> under;
		for (auto const &entry : Panels)
			for (auto const &face : entry.faces)
				under.push_back({plan(face[0]), plan(face[1]), plan(face[2])});
		if (false == under.empty())
			cut_gaps(result.fills, under);
	}
	return result;
}

} // namespace road_crossing
