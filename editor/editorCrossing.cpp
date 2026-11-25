/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorCrossing.hpp"

#include "world/Segment.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace level_crossing
{

namespace
{

glm::dvec2 plan(glm::dvec3 const &Point)
{
	return {Point.x, Point.z};
}

double cross(glm::dvec2 const &A, glm::dvec2 const &B)
{
	return A.x * B.y - A.y * B.x;
}

// the left of a direction in the plan, as a road has it (road_axis::frame)
glm::dvec2 left_of(glm::dvec2 const &Along)
{
	return {Along.y, -Along.x};
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

bool straight(segment_data const &Path)
{
	return Path.points[segment_data::point::control1] == glm::dvec3{0.0} && Path.points[segment_data::point::control2] == glm::dvec3{0.0};
}

// a part of a path between two values of its curve parameter, the rolls taken along
segment_data subpath(segment_data const &Path, double const From, double const To)
{
	segment_data part{Path};
	part.pieces.clear();
	part.rolls = {static_cast<float>(Path.rolls[0] + (Path.rolls[1] - Path.rolls[0]) * From), static_cast<float>(Path.rolls[0] + (Path.rolls[1] - Path.rolls[0]) * To)};
	auto const points{bezier(Path)};
	if (straight(Path))
	{
		part.points[segment_data::point::start] = glm::mix(points[0], points[3], From);
		part.points[segment_data::point::end] = glm::mix(points[0], points[3], To);
		return part;
	}
	part.points[segment_data::point::start] = bezier_point(points, From);
	part.points[segment_data::point::end] = bezier_point(points, To);
	part.points[segment_data::point::control1] = bezier_derivative(points, From) * ((To - From) / 3.0);
	part.points[segment_data::point::control2] = bezier_derivative(points, To) * (-(To - From) / 3.0);
	return part;
}

int constexpr samples{64};

// the road as one line along its pieces: the distance along it to each point
class road_line
{
  public:
	explicit road_line(std::vector<road_piece> const &Pieces) : m_pieces{Pieces}
	{
		double total{0.0};
		for (auto const &piece : m_pieces)
		{
			auto const points{bezier(piece.axis)};
			std::vector<double> stations(samples + 1, 0.0);
			auto previous{points[0]};
			for (int i = 1; i <= samples; ++i)
			{
				auto const current{bezier_point(points, static_cast<double>(i) / samples)};
				stations[i] = stations[i - 1] + glm::length(plan(current) - plan(previous));
				previous = current;
			}
			m_starts.push_back(total);
			total += stations.back();
			m_stations.push_back(std::move(stations));
			m_points.push_back(points);
		}
		m_length = total;
	}
	double length() const
	{
		return m_length;
	}
	std::size_t count() const
	{
		return m_pieces.size();
	}
	double start(std::size_t const Piece) const
	{
		return m_starts[Piece];
	}
	double end(std::size_t const Piece) const
	{
		return m_starts[Piece] + m_stations[Piece].back();
	}
	// the piece holding the distance; at a joint the one starting there
	std::size_t piece_at(double const Station) const
	{
		for (std::size_t i = m_pieces.size(); i-- > 0;)
			if (Station >= m_starts[i] - 1e-9)
				return i;
		return 0;
	}
	double parameter(std::size_t const Piece, double const Station) const
	{
		auto const &stations{m_stations[Piece]};
		auto const local{Station - m_starts[Piece]};
		if (local <= 0.0)
			return 0.0;
		if (local >= stations.back())
			return 1.0;
		auto const index{static_cast<int>(std::distance(stations.begin(), std::upper_bound(stations.begin(), stations.end(), local)))};
		auto const span{stations[index] - stations[index - 1]};
		return (index - 1 + (span > 0.0 ? (local - stations[index - 1]) / span : 0.0)) / samples;
	}
	double station(std::size_t const Piece, double const T) const
	{
		auto const &stations{m_stations[Piece]};
		auto const position{std::clamp(T, 0.0, 1.0) * samples};
		auto const index{std::min(samples - 1, static_cast<int>(position))};
		return m_starts[Piece] + glm::mix(stations[index], stations[index + 1], position - index);
	}
	glm::dvec3 point(double const Station) const
	{
		auto const piece{piece_at(Station)};
		return bezier_point(m_points[piece], parameter(piece, Station));
	}
	// the direction in the plan
	glm::dvec2 along(double const Station) const
	{
		auto const piece{piece_at(Station)};
		auto const t{std::clamp(parameter(piece, Station), 1e-4, 1.0 - 1e-4)};
		auto direction{plan(bezier_derivative(m_points[piece], t))};
		if (glm::length(direction) < 1e-9)
			direction = plan(m_points[piece][3] - m_points[piece][0]);
		return glm::length(direction) > 1e-9 ? glm::normalize(direction) : glm::dvec2{0.0, 1.0};
	}
	// the slope of the axis along the road, and its roll, degrees
	double grade(double const Station) const
	{
		auto const step{0.05};
		auto const a{std::max(0.0, Station - step)};
		auto const b{std::min(m_length, Station + step)};
		return b > a ? (point(b).y - point(a).y) / (b - a) : 0.0;
	}
	double roll(double const Station) const
	{
		auto const piece{piece_at(Station)};
		auto const t{parameter(piece, Station)};
		auto const &rolls{m_pieces[piece].axis.rolls};
		return rolls[0] + (rolls[1] - rolls[0]) * t;
	}
	double halfwidth(double const Station) const
	{
		return m_pieces[piece_at(Station)].halfwidth;
	}
	segment_data const &axis(std::size_t const Piece) const
	{
		return m_pieces[Piece].axis;
	}

  private:
	std::vector<road_piece> const &m_pieces;
	std::vector<std::vector<double>> m_stations;
	std::vector<std::array<glm::dvec3, 4>> m_points;
	std::vector<double> m_starts;
	double m_length{0.0};
};

// a cubic between two heights and their slopes, over a stretch of the road
struct hermite
{
	double from{0.0}, to{1.0};
	double h0{0.0}, m0{0.0}, h1{0.0}, m1{0.0};
	double height(double const Station) const
	{
		auto const l{to - from};
		if (l <= 0.0)
			return h0;
		auto const s{std::clamp((Station - from) / l, 0.0, 1.0)};
		auto const s2{s * s}, s3{s2 * s};
		return (2 * s3 - 3 * s2 + 1) * h0 + (s3 - 2 * s2 + s) * l * m0 + (-2 * s3 + 3 * s2) * h1 + (s3 - s2) * l * m1;
	}
	double slope(double const Station) const
	{
		auto const l{to - from};
		if (l <= 0.0)
			return m0;
		auto const s{std::clamp((Station - from) / l, 0.0, 1.0)};
		auto const s2{s * s};
		return ((6 * s2 - 6 * s) * h0 + (3 * s2 - 4 * s + 1) * l * m0 + (-6 * s2 + 6 * s) * h1 + (3 * s2 - 2 * s) * l * m1) / l;
	}
};

// a stretch of the road over tracks, in the plane of their heads
struct deck
{
	double from{0.0}, to{0.0};
	// an end facing the deck of the track next to it: where the axis passes the field side of the heads
	bool facing_from{false}, facing_to{false};
	head_plane plane;
	std::vector<std::size_t> tracks;
	double mismatch{0.0};
};

// what the height of the road is made of along a stretch of it
struct section
{
	double from{0.0}, to{0.0};
	int deck{-1}; // the deck it lies on, -1: a cubic
	hermite curve;
};

double degrees(double const Radians)
{
	return Radians * 180.0 / std::numbers::pi;
}

} // namespace

std::vector<rail_type> const &rail_types()
{
	static std::vector<rail_type> const types{
	    {"60E1", 0.072, "EN 13674-1; ArcelorMittal, Szyny transportowe (2023), the table of the flat bottom rails: C 72,00 mm", true},
	    {"49E1", 0.067, "EN 13674-1; ArcelorMittal, Szyny transportowe (2023), the table of the flat bottom rails: C 67,00 mm", true},
	    {"S42", 0.068, "unconfirmed: no sheet of the S42 rail found, the head of the older Polish rails taken", false},
	};
	return types;
}

rail_type const *rail_named(std::string const &Name)
{
	for (auto const &type : rail_types())
		if (Name == type.name)
			return &type;
	return nullptr;
}

head_plane common_plane(std::vector<rail_track> const &Tracks, double &Mismatch)
{
	std::vector<glm::dvec3> heads;
	for (auto const &track : Tracks)
		for (auto const side : {-1, 1})
			for (auto const along : {-2.0, 0.0, 2.0})
				heads.push_back(head_top(track, side, along, 0.5 * track.head));
	head_plane result;
	Mismatch = 0.0;
	if (heads.empty())
		return result;
	glm::dvec3 mean{0.0};
	for (auto const &head : heads)
		mean += head;
	mean /= static_cast<double>(heads.size());
	// h - mean.y = a (x - mean.x) + b (z - mean.z), least squares
	double xx{0.0}, xz{0.0}, zz{0.0}, xh{0.0}, zh{0.0};
	for (auto const &head : heads)
	{
		auto const x{head.x - mean.x}, z{head.z - mean.z}, h{head.y - mean.y};
		xx += x * x;
		xz += x * z;
		zz += z * z;
		xh += x * h;
		zh += z * h;
	}
	auto const det{xx * zz - xz * xz};
	result.origin = mean;
	if (std::abs(det) > 1e-12)
		result.gradient = {(xh * zz - zh * xz) / det, (zh * xx - xh * xz) / det};
	for (auto const &head : heads)
		Mismatch = std::max(Mismatch, std::abs(result.at(plan(head)) - head.y));
	return result;
}

bool touches(std::vector<glm::dvec2> const &Polygon, glm::dvec2 const &A, glm::dvec2 const &B)
{
	if (Polygon.size() < 3)
		return false;
	// either end inside
	auto const inside = [&](glm::dvec2 const &P) {
		bool in{false};
		for (std::size_t i = 0, j = Polygon.size() - 1; i < Polygon.size(); j = i++)
		{
			auto const &a{Polygon[i]};
			auto const &b{Polygon[j]};
			if ((a.y > P.y) != (b.y > P.y) && P.x < (b.x - a.x) * (P.y - a.y) / (b.y - a.y) + a.x)
				in = !in;
		}
		return in;
	};
	if (inside(A) || inside(B))
		return true;
	for (std::size_t i = 0, j = Polygon.size() - 1; i < Polygon.size(); j = i++)
	{
		auto const &c{Polygon[j]};
		auto const &d{Polygon[i]};
		auto const denominator{cross(B - A, d - c)};
		if (std::abs(denominator) < 1e-12)
			continue;
		auto const u{cross(c - A, d - c) / denominator};
		auto const v{cross(c - A, B - A) / denominator};
		if (u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0)
			return true;
	}
	return false;
}

std::vector<double> crossing_stations(design_input const &Input)
{
	road_line const line{Input.pieces};
	std::vector<double> result;
	for (auto const &track : Input.tracks)
	{
		// where the axis of the road crosses the axis of the track, taken straight there: the side of the track the road is on changes
		auto best{-1.0};
		auto bestdistance{std::numeric_limits<double>::max()};
		auto const step{0.25};
		auto const lateral = [&](double const Station) { return glm::dot(plan(line.point(Station)) - plan(track.point), track.right()); };
		auto previous{lateral(0.0)};
		for (double station = step; station <= line.length() + 1e-9; station += step)
		{
			auto const here{std::min(station, line.length())};
			auto const current{lateral(here)};
			if ((previous <= 0.0) != (current <= 0.0))
			{
				// halved down to a millimetre
				auto a{here - step}, b{here};
				auto fa{previous};
				for (int i = 0; i < 30 && b - a > 1e-4; ++i)
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
				auto const distance{glm::length(plan(line.point(at)) - plan(track.point))};
				if (distance < bestdistance)
				{
					bestdistance = distance;
					best = at;
				}
			}
			previous = current;
		}
		result.push_back(bestdistance < 50.0 ? best : -1.0);
	}
	return result;
}

double outer_slope(rail_track const &Own, int const Side, std::vector<rail_track> const &Tracks, glm::dvec2 const &Road, double const Halfwidth)
{
	auto const outwards{Own.right() * static_cast<double>(Side)};
	rail_track const *nearest{nullptr};
	auto best{std::numeric_limits<double>::max()};
	for (auto const &other : Tracks)
	{
		if (other.source == Own.source && other.path == Own.path)
			continue;
		auto const across{glm::dot(glm::dvec2{other.point.x - Own.point.x, other.point.z - Own.point.z}, outwards)};
		double gap{0.0};
		if (across <= 0.0 || false == facing(Own, other, Road, Halfwidth, &gap) || gap >= best)
			continue;
		best = gap;
		nearest = &other;
	}
	if (nearest == nullptr)
		return 0.0;
	// from the field side of the head of the own rail to the head of the rail of the other track facing it, straight
	auto const own{head_top(Own, Side, 0.0, Own.head)};
	auto const towards{glm::dot(glm::dvec2{Own.point.x - nearest->point.x, Own.point.z - nearest->point.z}, nearest->right()) > 0.0 ? 1 : -1};
	auto const along{glm::dot(glm::dvec2{own.x - nearest->point.x, own.z - nearest->point.z}, nearest->along)};
	auto const other{head_top(*nearest, towards, along, nearest->head)};
	auto const run{glm::length(glm::dvec2{other.x - own.x, other.z - own.z})};
	if (run < 1e-3)
		return 0.0;
	auto const line{(other.y - own.y) / run};
	// over the plane of the heads of the own track, going away from the rail
	return line - glm::dot(plane_of(Own).gradient, outwards);
}

double grade_break(rail_track const &A, rail_track const &B, glm::dvec2 const &Road)
{
	if (glm::length(Road) < 1e-9)
		return 0.0;
	// the road going from A to B
	auto road{glm::normalize(Road)};
	glm::dvec2 const a{A.point.x, A.point.z}, b{B.point.x, B.point.z};
	if (glm::dot(b - a, road) < 0.0)
		road = -road;
	// where its axis passes the field side of the head of each, the one facing the other
	auto const edge = [&](rail_track const &Track, glm::dvec2 const &Through, double const Direction) {
		auto const sine{std::max(0.05, std::abs(glm::dot(road, Track.right())))};
		return Through + road * (Direction * road_crossing::band(Track) / sine);
	};
	auto const ea{edge(A, a, 1.0)}, eb{edge(B, b, -1.0)};
	auto const run{glm::dot(eb - ea, road)};
	if (run < 1e-3)
		return 0.0;
	auto const grade{(plane_of(B).at(eb) - plane_of(A).at(ea)) / run};
	return std::max(std::abs(grade - glm::dot(plane_of(A).gradient, road)), std::abs(glm::dot(plane_of(B).gradient, road) - grade));
}

rail_track with_cant(rail_track Track, double const Share)
{
	// the roll about the axis, the sine of it the cant over 1500 mm
	Track.roll = std::asin(std::clamp(std::sin(Track.roll) * Share, -1.0, 1.0));
	return Track;
}

double break_share(rail_track const &A, rail_track const &B, glm::dvec2 const &Road)
{
	if (grade_break(A, B, Road) <= break_limit)
		return 1.0;
	if (grade_break(with_cant(A, 0.0), with_cant(B, 0.0), Road) > break_limit)
		return 0.0;
	auto low{0.0}, high{1.0};
	for (int i = 0; i < 40; ++i)
	{
		auto const middle{0.5 * (low + high)};
		if (grade_break(with_cant(A, middle), with_cant(B, middle), Road) <= break_limit)
			low = middle;
		else
			high = middle;
	}
	return low;
}

bool one_plane(rail_track const &A, rail_track const &B, double &Raise, int &Moved)
{
	// alike: the same cant the same way, the same grade, side by side
	if (std::abs(glm::dot(A.along, B.along)) < std::cos(glm::radians(2.0)))
		return false;
	auto const same{glm::dot(A.along, B.along) > 0.0};
	auto const rollb{same ? B.roll : -B.roll};
	auto const gradeb{same ? B.grade : -B.grade};
	if (std::abs(std::sin(A.roll) - std::sin(rollb)) * 1500.0 > 1.0 || std::abs(A.grade - gradeb) > 0.0005)
		return false;
	// the level of B in the plane of A, or of A in the one of B: the one which goes up
	auto const up{plane_of(A).at(glm::dvec2{B.point.x, B.point.z}) - B.point.y};
	Moved = up >= 0.0 ? 1 : 0;
	Raise = std::abs(up);
	return true;
}

double allowed_cant(double const Angle, double const Grade, bool const Narrow)
{
	auto const limit{Narrow ? narrow_cross_slope_limit : cross_slope_limit};
	auto const angle{glm::radians(std::clamp(Angle, 1.0, 90.0))};
	// the slope of the rails across the track, rise over run, the cant over 1500 mm between the rail axes along the slope
	auto const slope{std::max(0.0, limit - std::abs(Grade * std::cos(angle))) / std::sin(angle)};
	return std::floor(1500.0 * std::sin(std::atan(slope)) + 1e-6);
}

design_result design(design_input const &Input)
{
	design_result result;
	result.crossed_tracks = Input.tracks;
	if (Input.pieces.empty())
	{
		result.reason = "no road";
		return result;
	}
	road_line const line{Input.pieces};
	auto const total{line.length()};
	auto const crossings{crossing_stations(Input)};

	// the decks: over each track, the stretch of the road whose cross-section reaches the rails and their gaps, with the sleepers
	std::vector<deck> decks;
	for (std::size_t k = 0; k < Input.tracks.size(); ++k)
	{
		auto const &track{Input.tracks[k]};
		if (crossings[k] < 0.0)
			continue;
		if (track.turnout)
		{
			result.reason = "a level crossing isn't made within the length of a turnout (Dz.U. 2025 poz. 1105, § 34): the road stays as it is";
			return result;
		}
		auto const angle{std::acos(std::clamp(std::abs(glm::dot(line.along(crossings[k]), track.along)), 0.0, 1.0))};
		if (angle < glm::radians(15.0))
		{
			result.reason = "the road runs along the track rather than across it: the road stays as it is";
			return result;
		}
		result.angle = std::min(result.angle, degrees(angle));
		// no crossing on a cant ramp
		if (Input.limits && track.ramp)
			result.on_ramp.push_back(k);
		// the cant a crossing allows there: the road isn't laid over a track canted more
		auto const allowed{allowed_cant(degrees(angle), track.grade, Input.narrow)};
		if (Input.limits && std::abs(track.cant()) > allowed + 0.5)
			result.over.push_back({k, std::abs(track.cant()), allowed, degrees(angle)});
		auto const reachout{0.5 * track.gauge + track.head + deck_margin};
		auto const covers = [&](double const Station) {
			auto const centre{plan(line.point(Station))};
			auto const left{left_of(line.along(Station)) * line.halfwidth(Station)};
			auto const a{glm::dot(centre + left - plan(track.point), track.right())};
			auto const b{glm::dot(centre - left - plan(track.point), track.right())};
			return std::min(a, b) <= reachout && std::max(a, b) >= -reachout;
		};
		deck stretch;
		auto const step{0.02};
		stretch.from = stretch.to = crossings[k];
		while (stretch.from - step >= 0.0 && covers(stretch.from - step))
			stretch.from -= step;
		while (stretch.to + step <= total && covers(stretch.to + step))
			stretch.to += step;
		stretch.from = std::max(0.0, stretch.from - step);
		stretch.to = std::min(total, stretch.to + step);
		stretch.plane = plane_of(track);
		stretch.tracks = {k};
		decks.push_back(stretch);
	}
	if (decks.empty())
	{
		result.reason = "no rails cross the road";
		return result;
	}
	if (false == result.on_ramp.empty())
	{
		result.reason = "a level crossing isn't made on a cant ramp (Dz.U. 2025 poz. 1105, § 31 ust. 3): move the crossing off the ramp";
		return result;
	}
	if (false == result.over.empty())
	{
		result.reason = "the cant of a track is more than a level crossing allows";
		return result;
	}
	std::sort(decks.begin(), decks.end(), [](deck const &A, deck const &B) { return A.from < B.from; });
	// two tracks side by side: nearly parallel, the field sides of their heads apart
	auto const side_by_side = [&](deck const &A, deck const &B) {
		for (auto const ka : A.tracks)
			for (auto const kb : B.tracks)
			{
				auto const &a{Input.tracks[ka]};
				auto const &b{Input.tracks[kb]};
				if (std::abs(glm::dot(a.along, b.along)) < std::cos(glm::radians(10.0)))
					return false;
				double along{0.0}, across{0.0};
				road_crossing::track_frame(a, plan(b.point), along, across);
				if (std::abs(across) - road_crossing::band(a) - road_crossing::band(b) < 0.05)
					return false;
			}
		return true;
	};
	// decks reaching into each other share a plane, the one fit the best to the heads of all their rails; not the ones of two tracks
	// side by side, crossed at a skew, which keep a plane each, the road between them straight from the one to the other
	for (std::size_t i = 0; i + 1 < decks.size();)
	{
		if (decks[i + 1].from > decks[i].to + 0.5 || side_by_side(decks[i], decks[i + 1]))
		{
			++i;
			continue;
		}
		decks[i].to = std::max(decks[i].to, decks[i + 1].to);
		decks[i].tracks.insert(decks[i].tracks.end(), decks[i + 1].tracks.begin(), decks[i + 1].tracks.end());
		decks.erase(decks.begin() + i + 1);
		std::vector<rail_track> tracks;
		for (auto const k : decks[i].tracks)
			tracks.push_back(Input.tracks[k]);
		decks[i].plane = common_plane(tracks, decks[i].mismatch);
		result.mismatch = std::max(result.mismatch, decks[i].mismatch);
	}

	// two tracks side by side, each in its own plane, facing each other across the road (facing()): between them the road goes straight
	// from the field side of the head of the one to the head of the other, with no hump and no curve between the outer rails (Dz.U. 2025
	// poz. 1105, § 30 ust. 3), and the outer panels lying between them are bent onto that line (outer_slope). Along the axis the decks end
	// where it passes the field side of the heads; at a skew the pieces of the road, which meet along lines square to it, can't bend
	// along the heads across its width, and the road lays its geometry and its lanes onto the surface of the crossing itself
	// (road_crossing::crossing_surface)
	auto const edge_station = [&](rail_track const &Track, double const From, double const Direction) {
		auto const beyond = [&](double const Station) {
			double along{0.0}, across{0.0};
			road_crossing::track_frame(Track, plan(line.point(Station)), along, across);
			return std::abs(across) > road_crossing::band(Track);
		};
		// the last station over the heads, the first past them
		auto over{From}, past{From};
		while (std::abs(past - From) < 30.0 && past > 0.0 && past < total && false == beyond(past))
		{
			over = past;
			past = std::clamp(past + Direction * 0.01, 0.0, total);
		}
		for (int i = 0; i < 40; ++i)
		{
			auto const middle{0.5 * (over + past)};
			if (beyond(middle))
				past = middle;
			else
				over = middle;
		}
		return past;
	};
	for (std::size_t i = 0; i + 1 < decks.size(); ++i)
	{
		if (decks[i].tracks.size() != 1 || decks[i + 1].tracks.size() != 1)
			continue;
		auto const ka{decks[i].tracks.front()}, kb{decks[i + 1].tracks.front()};
		auto const middle{0.5 * (crossings[ka] + crossings[kb])};
		if (false == facing(Input.tracks[ka], Input.tracks[kb], line.along(middle), line.halfwidth(middle)))
			continue;
		decks[i].to = edge_station(Input.tracks[ka], crossings[ka], 1.0);
		decks[i].facing_to = true;
		decks[i + 1].from = edge_station(Input.tracks[kb], crossings[kb], -1.0);
		decks[i + 1].facing_from = true;
	}
	// a crossing panel lying on the track of a deck: the deck reaches over all of it, so the road lies in the plane of the heads, which is
	// the plane of the top of the panel, under the whole of its footprint, its edges and its cut corners too, and meets its edge at its
	// top. The road leaves its surface out under the panel (road_crossing::covers), so it doesn't show through it either
	for (auto &entry : decks)
		for (auto const &panel : Input.panels)
		{
			if (panel.outline.size() < 3)
				continue;
			glm::dvec2 middle{0.0};
			for (auto const &corner : panel.outline)
				middle += corner;
			middle /= static_cast<double>(panel.outline.size());
			auto const plane{entry.plane.at(middle)};
			// a slab at the heads, not something standing on the road nor lying under it
			if (panel.top > plane + 0.3 || panel.top < plane - 0.02)
				continue;
			double from{std::numeric_limits<double>::max()}, to{-1.0};
			for (double station = std::max(0.0, entry.from - 5.0); station <= std::min(total, entry.to + 5.0); station += 0.02)
			{
				auto const centre{plan(line.point(station))};
				auto const left{left_of(line.along(station)) * line.halfwidth(station)};
				if (touches(panel.outline, centre - left, centre + left))
				{
					from = std::min(from, station);
					to = std::max(to, station);
				}
			}
			// the panel of this track: it reaches into the deck
			if (to < from || from > entry.to + 0.5 || to < entry.from - 0.5)
				continue;
			// not on the side facing the next track: the panels there are bent onto the line between the two
			if (false == entry.facing_from)
				entry.from = std::min(entry.from, std::max(0.0, from - 0.04));
			if (false == entry.facing_to)
				entry.to = std::max(entry.to, std::min(total, to + 0.04));
		}
	// the joints of the pieces of the road: an edge or an end near one goes to it, so a stretch laid again comes out as the pieces it's made
	// of already, and nothing is laid anew for a difference of a few millimetres
	std::vector<double> joints;
	for (std::size_t i = 0; i <= line.count(); ++i)
		joints.push_back(i < line.count() ? line.start(i) : total);
	auto const snapped = [&](double const Station, double const Reach) {
		auto result{Station};
		auto nearest{Reach};
		for (auto const joint : joints)
			if (std::abs(joint - Station) < nearest)
			{
				nearest = std::abs(joint - Station);
				result = joint;
			}
		return result;
	};
	// an end facing the next track stays at the head, where the line to the other track starts and the panels bent onto it begin
	for (auto &entry : decks)
	{
		if (false == entry.facing_from)
			entry.from = snapped(entry.from, 0.05);
		if (false == entry.facing_to)
			entry.to = snapped(entry.to, 0.05);
	}

	auto const height_on = [&](deck const &Deck, double const Station) { return Deck.plane.at(plan(line.point(Station))); };
	auto const slope_on = [&](deck const &Deck, double const Station) { return glm::dot(Deck.plane.gradient, line.along(Station)); };
	auto const roll_on = [&](deck const &Deck, double const Station) { return degrees(std::atan(glm::dot(Deck.plane.gradient, left_of(line.along(Station))))); };

	// the stretches laid anew: the decks within 60 m of each other are one, with the approaches on either side
	struct stretch
	{
		double from{0.0}, to{0.0};
		std::vector<std::size_t> decks;
	};
	std::vector<stretch> stretches;
	for (std::size_t i = 0; i < decks.size(); ++i)
	{
		if (stretches.empty() || decks[i].from - decks[stretches.back().decks.back()].to > 2.0 * reach)
			stretches.push_back({});
		stretches.back().decks.push_back(i);
	}

	std::vector<section> sections;
	std::vector<std::pair<double, double>> extents;
	auto cramped{false}; // an approach to the end of the road which can't be within the grade the approaches may have
	for (auto &part : stretches)
	{
		auto const &first{decks[part.decks.front()]};
		auto const &last{decks[part.decks.back()]};
		// an end of the stretch within a metre of a joint goes to the joint: the stretch laid again a little longer or shorter (panels
		// laid on it, a deck reaching further) would leave a sliver of a piece past it otherwise, shorter than two ends of pieces may be
		// apart and still be two joints (editor_road), which breaks the road in two
		part.from = snapped(std::max(0.0, first.from - reach), 1.0);
		part.to = snapped(std::min(total, last.to + reach), 1.0);
		extents.emplace_back(part.from, part.to);
		// an approach: from the edge of the deck out to where the road goes on as it was. Outward: 1 past the last deck, -1 before the first
		auto const approach = [&](deck const &Deck, double const Edge, double const End, int const Outward) {
			auto const length{std::abs(End - Edge)};
			if (length < 0.05)
				return;
			auto const hb{height_on(Deck, Edge)};
			auto const gb{slope_on(Deck, Edge) * Outward}; // rise going outwards
			auto const he{line.point(End).y};
			// the grade of the road as it goes on past the end, taken from that side: the stretch laid again doesn't change it
			auto const outer{std::clamp(End + 0.1 * Outward, 0.0, total)};
			auto const ge{std::abs(outer - End) > 1e-6 ? (line.point(outer).y - he) / std::abs(outer - End) : line.grade(End) * Outward};
			// in the distance outward from the edge of the deck, then turned into the stations of the road
			std::vector<hermite> parts;
			// the road ends within the reach of the approach: the approach goes on to its end, there's nothing past it to lead into. One
			// which would have to be steeper than the approaches may be to get there can't be laid
			auto const ends{length < approach_length + 5.0 && (End <= 1e-6 || End >= total - 1e-6)};
			if (ends && std::abs(he - hb - 0.5 * std::min(ease, 0.5 * length) * gb) > approach_grade * (length - 0.5 * std::min(ease, 0.5 * length)) + 1e-6)
				cramped = true;
			if (length < 3.0 * ease)
			{
				parts.push_back({0.0, length, hb, gb, he, ends ? (he - hb) / length : ge});
			}
			else
			{
				auto const eased{ease};
				// the grade which takes the road from the edge of the deck to the height at the end of the approach, the ease from the
				// slope of the deck counted in; no steeper than the approaches may be
				auto const needed{(he - hb - 0.5 * eased * gb) / (length - 0.5 * eased)};
				auto const ga{std::clamp(needed, -approach_grade, approach_grade)};
				auto const held{ends ? length : length >= approach_length + 5.0 ? approach_length : std::max(eased + 0.5, length * 2.0 / 3.0)};
				auto const h1{hb + eased * 0.5 * (gb + ga)};
				auto const h2{h1 + (held - eased) * ga};
				parts.push_back({0.0, eased, hb, gb, h1, ga});
				parts.push_back({eased, held, h1, ga, h2, ga});
				if (held < length - 1e-6)
					parts.push_back({held, length, h2, ga, he, ge});
			}
			for (auto const &piece : parts)
			{
				section entry;
				if (Outward > 0)
				{
					entry.from = Edge + piece.from;
					entry.to = Edge + piece.to;
					entry.curve = {entry.from, entry.to, piece.h0, piece.m0, piece.h1, piece.m1};
				}
				else
				{
					entry.from = Edge - piece.to;
					entry.to = Edge - piece.from;
					entry.curve = {entry.from, entry.to, piece.h1, -piece.m1, piece.h0, -piece.m0};
				}
				sections.push_back(entry);
			}
		};
		approach(first, first.from, part.from, -1);
		for (std::size_t i = 0; i < part.decks.size(); ++i)
		{
			auto const index{part.decks[i]};
			auto const &here{decks[index]};
			section entry;
			entry.from = here.from;
			entry.to = here.to;
			entry.deck = static_cast<int>(index);
			sections.push_back(entry);
			if (i + 1 < part.decks.size())
			{
				auto const &next{decks[part.decks[i + 1]]};
				section between;
				between.from = here.to;
				between.to = next.from;
				between.curve = {here.to, next.from, height_on(here, here.to), slope_on(here, here.to), height_on(next, next.from), slope_on(next, next.from)};
				if (here.facing_to)
				{
					// straight from the one to the other
					auto const h0{height_on(here, here.to)}, h1{height_on(next, next.from)};
					auto const grade{(h1 - h0) / std::max(1e-6, next.from - here.to)};
					between.curve = {here.to, next.from, h0, grade, h1, grade};
				}
				if (between.to - between.from > 1e-3)
					sections.push_back(between);
			}
		}
		approach(last, last.to, part.to, 1);
	}
	std::sort(sections.begin(), sections.end(), [](section const &A, section const &B) { return A.from < B.from; });

	// the crossing panels lying on the decks: the road isn't lowered under them, it leaves its surface out under a panel as it makes its
	// geometry (road_crossing::covers), so its lanes stay at the top of the panel. They're counted
	for (auto const &panel : Input.panels)
	{
		if (panel.outline.size() < 3)
			continue;
		struct
		{
			double from, to;
		} under{std::numeric_limits<double>::max(), -1.0};
		for (auto const &[from, to] : extents)
			for (double station = from; station <= to; station += 0.02)
			{
				auto const centre{plan(line.point(station))};
				auto const left{left_of(line.along(station)) * line.halfwidth(station)};
				if (touches(panel.outline, centre - left, centre + left))
				{
					under.from = std::min(under.from, station);
					under.to = std::max(under.to, station);
				}
			}
		if (under.to < under.from)
			continue;
		auto const middle{0.5 * (under.from + under.to)};
		auto const *on{static_cast<section const *>(nullptr)};
		for (auto const &entry : sections)
			if (middle >= entry.from && middle <= entry.to)
				on = &entry;
		auto const road{on == nullptr ? line.point(middle).y : on->deck >= 0 ? height_on(decks[on->deck], middle) : on->curve.height(middle)};
		// something standing on the road higher than a slab is no panel of it, nor is one under the road
		if (panel.top > road + 0.3 || panel.top < road - 0.02)
			continue;
		++result.panels;
	}

	auto const section_at = [&](double const Station) -> section const & {
		for (auto const &entry : sections)
			if (Station >= entry.from - 1e-9 && Station <= entry.to + 1e-9)
				return entry;
		return sections.front();
	};
	auto const height_at = [&](double const Station) {
		auto const &entry{section_at(Station)};
		return entry.deck >= 0 ? height_on(decks[entry.deck], Station) : entry.curve.height(Station);
	};
	auto const slope_at = [&](double const Station) {
		auto const &entry{section_at(Station)};
		return entry.deck >= 0 ? slope_on(decks[entry.deck], Station) : entry.curve.slope(Station);
	};
	// the roll: the one of the plane over a deck, between the decks and on the approaches led evenly from one end to the other
	auto const roll_at = [&](double const Station) {
		auto const &entry{section_at(Station)};
		if (entry.deck >= 0)
			return roll_on(decks[entry.deck], Station);
		for (std::size_t s = 0; s < stretches.size(); ++s)
		{
			auto const &part{stretches[s]};
			if (Station < part.from - 1e-9 || Station > part.to + 1e-9)
				continue;
			auto const &first{decks[part.decks.front()]};
			auto const &last{decks[part.decks.back()]};
			if (Station <= first.from)
			{
				auto const span{first.from - part.from};
				return span > 0.0 ? glm::mix(line.roll(part.from), roll_on(first, first.from), (Station - part.from) / span) : roll_on(first, first.from);
			}
			if (Station >= last.to)
			{
				auto const span{part.to - last.to};
				return span > 0.0 ? glm::mix(roll_on(last, last.to), line.roll(part.to), (Station - last.to) / span) : roll_on(last, last.to);
			}
			for (std::size_t i = 0; i + 1 < part.decks.size(); ++i)
			{
				auto const &here{decks[part.decks[i]]};
				auto const &next{decks[part.decks[i + 1]]};
				if (Station >= here.to && Station <= next.from)
				{
					auto const span{next.from - here.to};
					return span > 0.0 ? glm::mix(roll_on(here, here.to), roll_on(next, next.from), (Station - here.to) / span) : roll_on(here, here.to);
				}
			}
		}
		return line.roll(Station);
	};

	// the breaks: the ends of the sections, the joints of the pieces of the road, and no piece longer
	// than 10 m. One near a joint goes to the joint, so a stretch laid again comes out as the pieces it's made of already
	std::vector<double> breaks;
	for (auto const &entry : sections)
	{
		breaks.push_back(entry.from);
		breaks.push_back(entry.to);
	}
	for (auto &station : breaks)
		for (auto const joint : joints)
			if (std::abs(station - joint) < 0.05)
				station = joint;
	// the joints there are kept: a piece laid lies within one of the pieces it's laid on
	for (auto const &[from, to] : extents)
		for (auto const joint : joints)
			if (joint > from && joint < to)
				breaks.push_back(joint);
	std::sort(breaks.begin(), breaks.end());
	breaks.erase(std::unique(breaks.begin(), breaks.end(), [](double A, double B) { return B - A < 0.05; }), breaks.end());
	std::vector<double> stations;
	for (std::size_t i = 0; i < breaks.size(); ++i)
	{
		stations.push_back(breaks[i]);
		if (i + 1 < breaks.size())
		{
			auto const span{breaks[i + 1] - breaks[i]};
			auto const count{static_cast<int>(std::ceil(span / 10.0 - 1e-6))};
			for (int j = 1; j < count; ++j)
				stations.push_back(breaks[i] + span * j / count);
		}
	}

	// at a skew the lanes pass the field side of the heads facing the next track one after another across the width of the road: the
	// pieces there half a metre long at most, the lanes set onto the surface of the crossing at their ends (road_node::fit_lanes), so
	// between them they keep to it within millimetres. The breaks there kept, the stretches between them split evenly, no piece a sliver
	std::vector<std::pair<double, double>> zones;
	for (auto const &entry : decks)
		for (auto const end : {0, 1})
		{
			if (end == 0 ? false == entry.facing_from : false == entry.facing_to)
				continue;
			auto const station{end == 0 ? entry.from : entry.to};
			auto const &track{Input.tracks[entry.tracks.front()]};
			auto const sine{std::max(0.2, std::abs(glm::dot(line.along(station), track.right())))};
			auto const skew{line.halfwidth(station) * std::sqrt(std::max(0.0, 1.0 - sine * sine)) / sine};
			if (skew >= 0.05)
				zones.emplace_back(std::max(0.0, station - skew), std::min(total, station + skew));
		}
	std::sort(zones.begin(), zones.end());
	for (std::size_t i = 0; i + 1 < zones.size();)
		if (zones[i + 1].first <= zones[i].second)
		{
			zones[i].second = std::max(zones[i].second, zones[i + 1].second);
			zones.erase(zones.begin() + static_cast<std::ptrdiff_t>(i) + 1);
		}
		else
			++i;
	for (auto [from, to] : zones)
	{
		// its ends onto a station near them, so no sliver is left beside it
		for (auto *end : {&from, &to})
			for (auto const station : stations)
				if (std::abs(station - *end) < 0.25)
					*end = station;
		std::vector<double> fixed{from, to};
		for (auto const station : breaks)
			if (station > from + 1e-9 && station < to - 1e-9)
				fixed.push_back(station);
		std::sort(fixed.begin(), fixed.end());
		std::erase_if(stations, [&](double const Station) {
			return Station > from + 1e-9 && Station < to - 1e-9 && std::none_of(breaks.begin(), breaks.end(), [&](double const Break) { return std::abs(Break - Station) < 1e-9; });
		});
		for (std::size_t k = 0; k + 1 < fixed.size(); ++k)
		{
			auto const span{fixed[k + 1] - fixed[k]};
			auto const count{std::max(1, static_cast<int>(std::ceil(span / 0.5 - 1e-6)))};
			for (int j = 0; j <= count; ++j)
				stations.push_back(fixed[k] + span * j / count);
		}
	}
	std::sort(stations.begin(), stations.end());
	stations.erase(std::unique(stations.begin(), stations.end(), [](double A, double B) { return B - A < 1e-6; }), stations.end());

	auto const within = [&](double const Station) {
		return std::any_of(extents.begin(), extents.end(), [&](auto const &Extent) { return Station >= Extent.first - 1e-9 && Station <= Extent.second + 1e-9; });
	};
	for (std::size_t i = 0; i + 1 < stations.size(); ++i)
	{
		auto const a{stations[i]}, b{stations[i + 1]};
		auto const middle{0.5 * (a + b)};
		if (b - a < 1e-3 || false == within(middle))
			continue;
		auto const source{line.piece_at(middle)};
		auto const ta{std::abs(a - line.start(source)) < 1e-6 ? 0.0 : line.parameter(source, a)};
		auto const tb{std::abs(b - line.end(source)) < 1e-6 ? 1.0 : line.parameter(source, b)};
		designed_piece piece;
		piece.source = source;
		piece.from = ta;
		piece.to = tb;
		piece.axis = subpath(line.axis(source), ta, tb);
		auto const &entry{section_at(middle)};
		piece.deck = entry.deck >= 0;
		auto &points{piece.axis.points};
		auto const flat{straight(piece.axis)};
		auto controls{bezier(piece.axis)};
		auto const length{b - a};
		auto const h0{height_at(a)}, h1{height_at(b)};
		double y1, y2;
		if (piece.deck)
		{
			// in the plane: the control points in it too, which keeps the whole curve in it
			auto const &plane{decks[entry.deck].plane};
			y1 = plane.at(plan(controls[1]));
			y2 = plane.at(plan(controls[2]));
		}
		else
		{
			y1 = h0 + slope_at(a + 1e-6) * length / 3.0;
			y2 = h1 - slope_at(b - 1e-6) * length / 3.0;
		}
		points[segment_data::point::start].y = h0;
		points[segment_data::point::end].y = h1;
		auto const linear{std::abs(y1 - glm::mix(h0, h1, 1.0 / 3.0)) < 1e-5 && std::abs(y2 - glm::mix(h0, h1, 2.0 / 3.0)) < 1e-5};
		if (false == (flat && linear))
		{
			auto const p0{points[segment_data::point::start]};
			auto const p3{points[segment_data::point::end]};
			auto const c1{flat ? glm::mix(p0, p3, 1.0 / 3.0) : controls[1]};
			auto const c2{flat ? glm::mix(p0, p3, 2.0 / 3.0) : controls[2]};
			points[segment_data::point::control1] = glm::dvec3{c1.x, y1, c1.z} - p0;
			points[segment_data::point::control2] = glm::dvec3{c2.x, y2, c2.z} - p3;
		}
		piece.axis.rolls = {static_cast<float>(roll_at(a + 1e-6)), static_cast<float>(roll_at(b - 1e-6))};
		result.pieces.push_back(piece);
	}

	// what's against the regulation
	for (auto const &part : stretches)
	{
		auto const &first{decks[part.decks.front()]};
		auto const &last{decks[part.decks.back()]};
		for (auto const &[from, to] : {std::pair{std::max(part.from, first.from - approach_length), first.from - ease}, std::pair{last.to + ease, std::min(part.to, last.to + approach_length)}})
			for (double station = from; station <= to; station += 0.1)
				result.steepest = std::max(result.steepest, std::abs(slope_at(station)));
		for (auto const index : part.decks)
			result.cross_slope = std::max(result.cross_slope, std::abs(slope_on(decks[index], 0.5 * (decks[index].from + decks[index].to))));
	}
	result.steep_approach = result.steepest > approach_grade + 1e-4;
	// the breaks of the grade line at the heads of the tracks facing each other: from the plane of the one onto the line to the other
	for (std::size_t i = 0; i + 1 < decks.size(); ++i)
	{
		if (false == decks[i].facing_to)
			continue;
		auto const &here{decks[i]};
		auto const &next{decks[i + 1]};
		auto const run{next.from - here.to};
		auto const grade{run > 1e-6 ? (height_on(next, next.from) - height_on(here, here.to)) / run : 0.0};
		auto const value{std::max(std::abs(grade - slope_on(here, here.to)), std::abs(slope_on(next, next.from) - grade))};
		result.breaks.push_back({here.tracks.front(), next.tracks.front(), value});
		result.largest_break = std::max(result.largest_break, value);
	}
	// the editor doesn't lay an approach steeper than the regulation allows: where the road around doesn't leave room for one within it
	// (it ends too near the rails, or too far off their height), the crossing is refused
	if (Input.limits && (result.steep_approach || cramped))
	{
		result.valid = false;
		result.reason = "the approach can't be kept within 3% over 20 m of the rails (Dz.U. 2025 poz. 1105, annex 5): the road ends too near the rails or too far off their height; lengthen it or bring it nearer the height of the rails";
		return result;
	}
	// over 5% the editor can't lay it: that takes the analysis of how the vehicles get over it (annex 5 pt 3)
	if (Input.limits && result.largest_break > break_limit + 1e-4)
	{
		result.valid = false;
		result.reason = "the grade line of the road breaks at the heads of two tracks side by side by more than 5% (Dz.U. 2025 poz. 1105, annex 5 pt 3)";
		return result;
	}
	result.steep_cross = result.cross_slope > (Input.narrow ? narrow_cross_slope_limit : cross_slope_limit) + 1e-4;
	result.tracks = 0;
	result.cant = 0.0;
	result.cant_low = std::numeric_limits<double>::max();
	for (std::size_t k = 0; k < Input.tracks.size(); ++k)
	{
		if (crossings[k] < 0.0)
			continue;
		++result.tracks;
		result.cant = std::max(result.cant, std::abs(Input.tracks[k].cant()));
		result.cant_low = std::min(result.cant_low, std::abs(Input.tracks[k].cant()));
	}
	if (false == extents.empty())
	{
		result.from = extents.front().first;
		result.to = extents.back().second;
	}
	result.stretches = extents;
	result.starts = joints;
	for (std::size_t k = 0; k < Input.tracks.size(); ++k)
		if (crossings[k] >= 0.0)
			result.crossed.push_back(k);
	result.valid = false == result.pieces.empty();
	return result;
}

bool surface_height(std::vector<designed_piece> const &Pieces, double const Halfwidth, glm::dvec2 const &Point, double &Height)
{
	auto best{std::numeric_limits<double>::max()};
	for (auto const &piece : Pieces)
	{
		auto const points{bezier(piece.axis)};
		// the nearest point of the axis in the plan, then the offset across it
		auto t{0.0};
		auto distance{std::numeric_limits<double>::max()};
		for (int i = 0; i <= 200; ++i)
		{
			auto const u{i / 200.0};
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
			t = std::clamp(t + glm::dot(Point - plan(bezier_point(points, t)), along) / span, 0.0, 1.0);
		}
		auto const axis{bezier_point(points, t)};
		auto const along{glm::normalize(plan(bezier_derivative(points, std::clamp(t, 1e-4, 1.0 - 1e-4))))};
		auto const offset{Point - plan(axis)};
		if (std::abs(glm::dot(offset, along)) > 1e-3 && t > 0.0 && t < 1.0)
			continue;
		if ((t <= 0.0 || t >= 1.0) && std::abs(glm::dot(offset, along)) > 1e-3)
			continue;
		auto const across{glm::dot(offset, left_of(along))};
		if (std::abs(across) > Halfwidth || std::abs(across) >= best)
			continue;
		best = std::abs(across);
		auto const roll{glm::radians(glm::mix(static_cast<double>(piece.axis.rolls[0]), static_cast<double>(piece.axis.rolls[1]), t))};
		Height = axis.y + across * std::tan(roll);
	}
	return best < std::numeric_limits<double>::max();
}

double axis_height(std::vector<designed_piece> const &Pieces, double const Station)
{
	double start{0.0};
	for (auto const &piece : Pieces)
	{
		auto const points{bezier(piece.axis)};
		std::vector<double> stations(samples + 1, 0.0);
		for (int i = 1; i <= samples; ++i)
			stations[i] = stations[i - 1] + glm::length(plan(bezier_point(points, i / double(samples))) - plan(bezier_point(points, (i - 1) / double(samples))));
		if (Station <= start + stations.back() || &piece == &Pieces.back())
		{
			auto const local{std::clamp(Station - start, 0.0, stations.back())};
			auto const index{std::max<std::ptrdiff_t>(1, std::distance(stations.begin(), std::upper_bound(stations.begin(), stations.end(), local)))};
			auto const i{std::min<std::ptrdiff_t>(index, samples)};
			auto const span{stations[i] - stations[i - 1]};
			auto const t{(i - 1 + (span > 0.0 ? (local - stations[i - 1]) / span : 0.0)) / samples};
			return bezier_point(points, t).y;
		}
		start += stations.back();
	}
	return 0.0;
}

double axis_grade(std::vector<designed_piece> const &Pieces, double const Station)
{
	auto const step{0.02};
	return (axis_height(Pieces, Station + step) - axis_height(Pieces, Station - step)) / (2.0 * step);
}

} // namespace level_crossing
