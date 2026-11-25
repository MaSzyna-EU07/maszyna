/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutGenerator.hpp"
#include "editor/layout/layoutTurnout.hpp"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <format>
#include <iterator>
#include <optional>
#include <string>

namespace layout
{

namespace
{

double constexpr kStraightened{0.02}; // m, TTrack::straightened: control vectors this close to the third of the chord are taken for none
double constexpr kShortestPiece{0.1}; // m: the simulation takes a path under 5 cm for one of no length
int constexpr kDeepest{14}; // halvings of a piece, at most

// a point of the line where a piece ends: at a node with the numbers of the node. A piece ending there arrives along the element before it,
// which a secant blade turns off from (arriving: the heading of that element, the same where the line runs on in its heading)
struct piece_end
{
	double chainage{0.0};
	glm::dvec3 point{0.0};
	glm::dvec2 heading{0.0, 1.0};
	glm::dvec2 arriving{0.0, 1.0};
	double grade{0.0};
	double cant{0.0};
};

std::string number_text(double const Value)
{
	return std::format("{}", Value);
}

std::string number_text(float const Value)
{
	return std::format("{}", Value);
}

std::string point_text(glm::dvec3 const &Point)
{
	return std::format("{} {} {}", number_text(Point.x), number_text(Point.y), number_text(Point.z));
}

// a path as an entry has it: the start, its roll, the control vectors, the end, its roll, the radius
std::string path_text(segment_data const &Path)
{
	auto const &p{Path.points};
	return std::format("{} {} {} {} {} {} {}", point_text(p[segment_data::point::start]), number_text(Path.rolls[0]), point_text(p[segment_data::point::control1]),
	                   point_text(p[segment_data::point::control2]), point_text(p[segment_data::point::end]), number_text(Path.rolls[1]), number_text(Path.radius));
}

// the entry of a plain track of the piece: its name, its length, the surface, its path and the attributes
std::string entry_text(generated_piece const &Piece, std::string_view const Surface, std::string_view const Attributes)
{
	return std::format("node -1 0 {} track normal {:.3f} {} {} {} endtrack\n", Piece.name, std::abs(Piece.to - Piece.from), Surface, path_text(Piece.path), Attributes);
}

class cutter
{
  public:
	cutter(network const &Network, line const &Line, generator_options const &Options) : m_line{Line}, m_solved{Line.solved}, m_options{Options}
	{
		for (auto const end : {line_end::start, line_end::end})
		{
			auto const *at{Network.find(Line.node_at(end))};
			auto const *ref{Network.end_of(Line.id, end)};
			auto &kept{end == line_end::start ? m_start : m_end};
			auto const sense{ref != nullptr ? static_cast<double>(ref->sense) : 1.0};
			auto const chainage{end == line_end::start ? 0.0 : m_solved.length};
			kept.chainage = chainage;
			kept.point = at != nullptr ? at->position : m_solved.point(chainage);
			kept.heading = at != nullptr ? at->heading * sense : m_solved.tangent(chainage);
			kept.arriving = kept.heading;
			kept.cant = at != nullptr ? at->cant * sense : m_solved.cant_at(chainage);
			kept.grade = m_solved.grade(chainage);
		}
	}

	std::vector<generated_piece> pieces(std::vector<std::string> &Errors) { return pieces(0.0, m_solved.length, Errors); }

	// the pieces between the chainages: from the node at an end of the line, from the point of the line inside it
	std::vector<generated_piece> pieces(double const From, double const To, std::vector<std::string> &Errors)
	{
		auto cuts{breaks()};
		std::erase_if(cuts, [&](double const Chainage) { return Chainage <= From + kShortestPiece * 0.5 || Chainage >= To - kShortestPiece * 0.5; });
		cuts.insert(cuts.begin(), From);
		cuts.push_back(To);
		std::vector<generated_piece> result;
		auto from{From <= 0.0 ? m_start : end_at(From)};
		auto const last{To >= m_solved.length ? m_end : end_at(To)};
		for (std::size_t i = 1; i < cuts.size(); ++i)
		{
			auto const to{i + 1 == cuts.size() ? last : end_at(cuts[i])};
			// no piece longer than a track may be
			auto const count{std::max(1, static_cast<int>(std::ceil((to.chainage - from.chainage) / geometry::longest_path - 1e-9)))};
			auto start{from};
			for (int k = 1; k <= count; ++k)
			{
				auto const finish{k == count ? to : end_at(from.chainage + (to.chainage - from.chainage) * k / count)};
				cut(start, finish, 0, result, Errors);
				start = finish;
			}
			from = to;
		}
		return result;
	}

  private:
	line const &m_line;
	solved_line const &m_solved;
	generator_options const &m_options;
	piece_end m_start;
	piece_end m_end;

	piece_end end_at(double const Chainage) const
	{
		piece_end result;
		result.chainage = Chainage;
		result.point = m_solved.point(Chainage);
		result.heading = glm::normalize(m_solved.tangent(Chainage));
		result.arriving = result.heading;
		// at the tip of a secant blade the element before heads another way
		auto const &before{m_solved.elements[m_solved.element_at(std::max(0.0, Chainage - 1e-9))]};
		if (auto const arriving{glm::normalize(before.tangent(std::clamp(Chainage - before.chainage, 0.0, before.length)))}; std::abs(geometry::signed_angle(arriving, result.heading)) > 1e-9)
			result.arriving = arriving;
		result.grade = m_solved.grade(Chainage);
		result.cant = m_solved.cant_at(Chainage);
		return result;
	}

	// where the pieces have to end: the elements, the vertical curves, the stretches of the cant; none closer than the shortest piece
	std::vector<double> breaks() const
	{
		std::vector<double> result{0.0, m_solved.length};
		for (auto const &e : m_solved.elements)
			result.push_back(e.chainage);
		for (std::size_t i = 1; i + 1 < m_solved.profile.size(); ++i)
			if (auto const &point{m_solved.profile[i]}; point.tangent > 0.0)
			{
				result.push_back(point.chainage - point.tangent);
				result.push_back(point.chainage + point.tangent);
			}
		for (auto const &stretch : m_solved.cant)
		{
			result.push_back(stretch.from);
			result.push_back(stretch.to);
		}
		auto const hard{hard_points()};
		result.insert(result.end(), hard.begin(), hard.end());
		std::ranges::sort(result);
		std::erase_if(result, [&](double const Chainage) { return Chainage < 0.0 || Chainage > m_solved.length; });
		auto const last{std::unique(result.begin(), result.end(), [](double const A, double const B) { return std::abs(A - B) < 1e-9; })};
		result.erase(last, result.end());
		result.front() = 0.0;
		result.back() = m_solved.length;
		auto const is_hard = [&result, &hard](std::size_t const At) { return At == 0 || At + 1 == result.size() || std::ranges::find(hard, result[At]) != hard.end(); };
		for (std::size_t i = 0; i + 1 < result.size() && result.size() > 2;)
		{
			if (result[i + 1] - result[i] >= kShortestPiece)
			{
				++i;
				continue;
			}
			// the short stretch goes to its neighbour: the break after it goes, or the one before it; a joint of an old track stays, the
			// short piece with it where both ends are
			if (false == is_hard(i + 1))
				result.erase(result.begin() + static_cast<std::ptrdiff_t>(i + 1));
			else if (false == is_hard(i))
				result.erase(result.begin() + static_cast<std::ptrdiff_t>(i));
			else
				++i;
		}
		return result;
	}

	// the joints of the old tracks inside the line
	std::vector<double> hard_points() const
	{
		std::vector<double> result;
		for (auto const &split : m_line.hard_splits)
			if (split.chainage > 0.0 && split.chainage < m_solved.length)
				result.push_back(split.chainage);
		return result;
	}

	// the element the chainage lies in, the curve of a span all in one
	element const *only_element(double const From, double const To) const
	{
		auto const &e{m_solved.elements[m_solved.element_at(From + 1e-9)]};
		return e.end_chainage() >= To - 1e-9 ? &e : nullptr;
	}

	bool level_grade(double const From, double const To) const
	{
		for (std::size_t i = 1; i + 1 < m_solved.profile.size(); ++i)
		{
			auto const &point{m_solved.profile[i]};
			if (point.tangent > 0.0 && point.chainage + point.tangent > From + 1e-9 && point.chainage - point.tangent < To - 1e-9)
				return false;
		}
		return true;
	}

	float radius(double const From, double const To) const
	{
		double sharpest{0.0};
		for (auto const &e : m_solved.elements)
		{
			if (e.end_chainage() <= From + 1e-9 || e.chainage >= To - 1e-9)
				continue;
			auto const a{std::max(From, e.chainage) - e.chainage};
			auto const b{std::min(To, e.end_chainage()) - e.chainage};
			sharpest = std::max({sharpest, std::abs(e.curvature(a)), std::abs(e.curvature(b))});
		}
		return sharpest > 1e-9 ? static_cast<float>(1.0 / sharpest) : 0.f;
	}

	// the cubic parabola of the element as the cubic Bezier curve it is, the part of it between the chainages
	bool parabola_controls(element const &E, double const From, double const To, glm::dvec2 &P1, glm::dvec2 &P2, glm::dvec2 const &P0, glm::dvec2 const &P3) const
	{
		if (E.kind != element_kind::cubic_parabola)
			return false;
		auto const radius{std::abs(E.parabola_radius)};
		auto const x{E.parabola_x};
		std::array<glm::dvec2, 4> const form{glm::dvec2{0.0, 0.0}, glm::dvec2{x / 3.0, 0.0}, glm::dvec2{x * 2.0 / 3.0, 0.0}, glm::dvec2{x, x * x / (6.0 * radius)}};
		// the abscissae of the ends, along the parabola from its tangent point
		auto const abscissa = [&](double const Along) {
			double low{0.0};
			double high{x};
			for (int i = 0; i < 60; ++i)
			{
				auto const middle{0.5 * (low + high)};
				(parabola_length(x, radius, middle) < Along ? low : high) = middle;
			}
			return 0.5 * (low + high);
		};
		auto const a{From - E.chainage};
		auto const b{To - E.chainage};
		glm::dvec2 origin;
		glm::dvec2 along;
		double side{E.parabola_radius < 0.0 ? -1.0 : 1.0};
		double ta{0.0};
		double tb{0.0};
		if (E.entering)
		{
			origin = E.origin;
			along = E.direction;
			ta = abscissa(a) / x;
			tb = abscissa(b) / x;
		}
		else
		{
			// from the tangent point at its end, looking back into the curve, turning the other way
			origin = E.position(E.length);
			along = -glm::normalize(E.tangent(E.length));
			side = -side;
			ta = abscissa(E.length - a) / x;
			tb = abscissa(E.length - b) / x;
		}
		auto const world = [&](glm::dvec2 const &Point) { return origin + along * Point.x + left_of(along) * (side * Point.y); };
		auto const q0{world(blossom(form, ta, ta, ta))};
		auto const q3{world(blossom(form, tb, tb, tb))};
		P1 = P0 + (world(blossom(form, ta, ta, tb)) - q0);
		P2 = P3 + (world(blossom(form, ta, tb, tb)) - q3);
		return true;
	}

	// the curvature at the chainage from inside the piece: of the element after it at the start of the piece, before it at the end
	double curvature_inside(double const Chainage, bool const After) const
	{
		auto const &e{m_solved.elements[m_solved.element_at(After ? Chainage + 1e-9 : Chainage - 1e-9)]};
		return e.curvature(std::clamp(Chainage - e.chainage, 0.0, e.length));
	}

	// the handles of a cubic Bezier curve between the ends and their headings which give it the curvatures of the line at its ends
	// (G2 Hermite interpolation, by Newton's method from those of an arc); the ones of an arc where it finds none near them
	static std::pair<double, double> curvature_handles(glm::dvec2 const &P0, glm::dvec2 const &T0, glm::dvec2 const &P3, glm::dvec2 const &T3, double const K0, double const K3, double const Handle)
	{
		auto const chord{P3 - P0};
		auto const across0{geometry::cross(T0, chord)};
		auto const across3{geometry::cross(chord, T3)};
		auto const turn{geometry::cross(T0, T3)};
		// 3/2 K0 a^2 = (T0 x D) - b (T0 x T3), 3/2 K3 b^2 = (D x T3) - a (T0 x T3)
		double a{Handle};
		double b{Handle};
		for (int i = 0; i < 30; ++i)
		{
			auto const f{1.5 * K0 * a * a - across0 + b * turn};
			auto const g{1.5 * K3 * b * b - across3 + a * turn};
			auto const fa{3.0 * K0 * a};
			auto const fb{turn};
			auto const ga{turn};
			auto const gb{3.0 * K3 * b};
			auto const determinant{fa * gb - fb * ga};
			if (std::abs(determinant) < 1e-15)
				return {Handle, Handle};
			auto const da{(f * gb - fb * g) / determinant};
			auto const db{(fa * g - f * ga) / determinant};
			a -= da;
			b -= db;
			if (std::abs(da) + std::abs(db) < 1e-12 * Handle)
				break;
		}
		if (false == std::isfinite(a) || false == std::isfinite(b) || a < 0.3 * Handle || b < 0.3 * Handle || a > 2.0 * Handle || b > 2.0 * Handle)
			return {Handle, Handle};
		return {a, b};
	}

	double cant_deviation(piece_end const &From, piece_end const &To) const
	{
		double worst{0.0};
		int constexpr samples{16};
		for (int i = 1; i < samples; ++i)
		{
			auto const t{static_cast<double>(i) / samples};
			auto const s{From.chainage + (To.chainage - From.chainage) * t};
			worst = std::max(worst, std::abs(m_solved.cant_at(s) - (From.cant + (To.cant - From.cant) * t)));
		}
		return worst;
	}

	segment_data path_of(piece_end const &From, piece_end const &To) const
	{
		segment_data path;
		path.points[segment_data::point::start] = From.point;
		path.points[segment_data::point::end] = To.point;
		path.rolls = {static_cast<float>(roll_of(From.cant)), static_cast<float>(roll_of(To.cant))};
		path.radius = radius(From.chainage, To.chainage);
		auto const p0{geometry::plan_of(From.point)};
		auto const p3{geometry::plan_of(To.point)};
		auto const turn{geometry::signed_angle(From.heading, To.arriving)};
		auto const *single{only_element(From.chainage, To.chainage)};
		// a straight of an even grade is drawn by the simulator as a straight of no control vectors
		if (single != nullptr && single->kind == element_kind::straight && level_grade(From.chainage, To.chainage))
			return path;
		// the handles of an arc of the turn, 4/3 tan(turn / 4) R; for a transition a start, halved until close enough
		auto const chord{glm::distance(p0, p3)};
		auto const cosine{std::cos(turn / 4.0)};
		auto const handle{chord / (3.0 * cosine * cosine)};
		auto handles{std::make_pair(handle, handle)};
		if (single == nullptr || single->kind != element_kind::arc)
			handles = curvature_handles(p0, From.heading, p3, To.arriving, curvature_inside(From.chainage, true), curvature_inside(To.chainage, false), handle);
		glm::dvec2 p1{p0 + From.heading * handles.first};
		glm::dvec2 p2{p3 - To.arriving * handles.second};
		if (single != nullptr)
			parabola_controls(*single, From.chainage, To.chainage, p1, p2, p0, p3);
		auto const y1{From.point.y + From.grade * glm::distance(p0, p1)};
		auto const y2{To.point.y - To.grade * glm::distance(p2, p3)};
		path.points[segment_data::point::control1] = glm::dvec3{p1.x, y1, p1.y} - From.point;
		path.points[segment_data::point::control2] = glm::dvec3{p2.x, y2, p2.y} - To.point;
		return path;
	}

	std::string name_of(double const From)
	{
		auto const &e{m_solved.elements[m_solved.element_at(From + 1e-9)]};
		std::string key;
		switch (e.role)
		{
		case element_role::straight: key = e.vertex != 0 ? std::format("s{}", e.vertex) : "se"; break;
		case element_role::transition_in: key = std::format("v{}i", e.vertex); break;
		case element_role::arc: key = std::format("v{}a", e.vertex); break;
		case element_role::transition_out: key = std::format("v{}o", e.vertex); break;
		}
		auto const count{++m_counts[key]};
		return std::format("{}{}_{}{}", m_options.prefix, number(m_line.id), key, count);
	}

	void cut(piece_end const &From, piece_end const &To, int const Depth, std::vector<generated_piece> &Into, std::vector<std::string> &Errors)
	{
		auto path{path_of(From, To)};
		if (straightened(path))
		{
			// the simulator would draw it straight: so it's a straight, close enough or halved
			path.points[segment_data::point::control1] = glm::dvec3{0.0};
			path.points[segment_data::point::control2] = glm::dvec3{0.0};
		}
		auto const off{deviation(m_solved, drawn(path), From.chainage, To.chainage)};
		auto const cant_off{cant_deviation(From, To)};
		auto const fits{off <= m_options.tolerance && cant_off <= m_options.cant_tolerance};
		if (false == fits && Depth < kDeepest && To.chainage - From.chainage >= 2.0 * kShortestPiece)
		{
			auto const middle{end_at(0.5 * (From.chainage + To.chainage))};
			cut(From, middle, Depth + 1, Into, Errors);
			cut(middle, To, Depth + 1, Into, Errors);
			return;
		}
		if (false == fits)
			Errors.push_back(std::format("{}: the piece at {:.2f} m lies {:.1f} mm from the line", std::format("line {}", number(m_line.id)), From.chainage, off * 1000.0));
		generated_piece piece;
		piece.name = name_of(From.chainage);
		piece.from = From.chainage;
		piece.to = To.chainage;
		piece.path = path;
		piece.deviation = off;
		piece.text = entry_text(piece, m_options.surface, std::format("velocity {}", number_text(m_line.category.vmax)));
		Into.push_back(std::move(piece));
	}

	std::map<std::string, int> m_counts;
};

// which of the events of an old track a piece of its stretch takes (TTrack::m_events*, TrkFoll.cpp): event0 and eventall0, of a vehicle
// standing on the track, each piece; event2 and eventall2, of one entering the track at its Point1, the piece at that end; event1 and
// eventall1, entering it at its Point2, the piece at the other end
struct piece_events
{
	bool standing{false};
	bool at_point1{false};
	bool at_point2{false};
};

// the attributes of an old track with the events the piece takes; the keywords in any case, as the simulator lowers its tokens
std::string attributes_of_piece(std::string_view const Attributes, piece_events const Kept)
{
	auto const kept = [Kept](std::string_view const Word) -> std::optional<bool> {
		std::string lower;
		std::ranges::transform(Word, std::back_inserter(lower), [](char const C) { return static_cast<char>(std::tolower(static_cast<unsigned char>(C))); });
		if (lower == "event0" || lower == "eventall0")
			return Kept.standing;
		if (lower == "event2" || lower == "eventall2")
			return Kept.at_point1;
		if (lower == "event1" || lower == "eventall1")
			return Kept.at_point2;
		return std::nullopt;
	};
	std::string result;
	std::size_t at{0};
	bool skip{false};
	while (at < Attributes.size())
	{
		auto const start{Attributes.find_first_not_of(' ', at)};
		if (start == std::string_view::npos)
			break;
		auto const end{std::min(Attributes.find(' ', start), Attributes.size())};
		auto const word{Attributes.substr(start, end - start)};
		at = end;
		auto const event{kept(word)};
		if (skip)
			skip = false;
		else if (event.has_value() && false == *event)
			skip = true; // the event and its name after it
		else
			result += (result.empty() ? "" : " ") + std::string{word};
	}
	return result;
}

// the attributes of an old track without its events, for a part of it a split cut off
std::string without_events(std::string_view const Attributes)
{
	return attributes_of_piece(Attributes, {});
}

// the pieces of the stretches of the old tracks named and written as each old track was: its name on the piece it ran into first, with
// its events, <name>_2... on the others, its surface and its attributes, the path the way it ran; a piece out of any stretch as the generator
// names and writes it
void keep_old_tracks(line const &Line, std::vector<generated_piece> &Pieces, generator_options const &Options)
{
	auto const &splits{Line.hard_splits};
	auto const stretch_of = [&splits](generated_piece const &Piece) {
		auto const after{std::ranges::upper_bound(splits, Piece.from + 1e-6, {}, &hard_split::chainage)};
		return after == splits.begin() ? splits.size() : static_cast<std::size_t>(after - splits.begin()) - 1;
	};
	std::size_t first{0};
	while (first < Pieces.size())
	{
		auto const stretch{stretch_of(Pieces[first])};
		auto last{first + 1};
		while (last < Pieces.size() && stretch_of(Pieces[last]) == stretch)
			++last;
		if (stretch < splits.size())
		{
			auto const &old{splits[stretch].track};
			auto const count{last - first};
			for (auto i = first; i < last; ++i)
			{
				auto &piece{Pieces[i]};
				// 0 at the Point1 of the old track
				auto const order{old.reversed ? last - 1 - i : i - first};
				if (false == old.name.empty() && false == old.continued)
					piece.name = order == 0 ? old.name : std::format("{}_{}", old.name, order + 1);
				if (old.reversed)
					piece.path = geometry::reversed(piece.path);
				auto attributes{old.continued ? without_events(old.attributes) : attributes_of_piece(old.attributes, {true, order == 0, order + 1 == count})};
				if (old.attributes.empty())
					attributes = std::format("velocity {}", number_text(Line.category.vmax));
				piece.text = entry_text(piece, old.surface.empty() ? Options.surface : old.surface, attributes);
			}
		}
		first = last;
	}
}

// the path the pieces make, from the start of the first to the end of the last, for the versions of the simulator which know only the path:
// one curve between their headings, the sharpest radius of them
segment_data whole_path(std::vector<generated_piece> const &Pieces)
{
	// a track the cutter gave no pieces of, its errors told: no path
	if (Pieces.empty())
		return {};
	if (Pieces.size() == 1)
		return Pieces.front().path;
	segment_data result;
	auto const &first{Pieces.front().path};
	auto const &last{Pieces.back().path};
	auto const start{first.points[segment_data::point::start]};
	auto const end{last.points[segment_data::point::end]};
	result.points[segment_data::point::start] = start;
	result.points[segment_data::point::end] = end;
	result.rolls = {first.rolls[0], last.rolls[1]};
	auto const heading = [](drawn_curve const &Curve, bool const AtEnd) {
		auto const vector{AtEnd ? Curve.p3 - Curve.p2 : Curve.p1 - Curve.p0};
		return glm::normalize(geometry::plan_of(vector));
	};
	auto const handle{geometry::plan_distance(start, end) / 3.0};
	auto const out{heading(drawn(first), false)};
	auto const in{heading(drawn(last), true)};
	auto const rise{(end.y - start.y) / 3.0};
	result.points[segment_data::point::control1] = {out.x * handle, rise, out.y * handle};
	result.points[segment_data::point::control2] = {-in.x * handle, -rise, -in.y * handle};
	float sharpest{0.f};
	for (auto const &piece : Pieces)
		if (piece.path.radius > 0.f && (sharpest == 0.f || piece.path.radius < sharpest))
			sharpest = piece.path.radius;
	result.radius = sharpest;
	return result;
}

} // namespace

glm::dvec3 drawn_curve::point(double const T) const
{
	auto const u{1.0 - T};
	return u * u * u * p0 + 3.0 * u * u * T * p1 + 3.0 * u * T * T * p2 + T * T * T * p3;
}

bool straightened(segment_data const &Path)
{
	// as TTrack::straightened() takes it, the second of its checks the first control vector again
	auto const &p1{Path.points[segment_data::point::start]};
	auto const &cp1{Path.points[segment_data::point::control1]};
	auto const &p2{Path.points[segment_data::point::end]};
	auto const square = [](glm::dvec3 const &V) { return glm::dot(V, V); };
	return square((p1 + p1 + p2) / 3.0 - p1 - cp1) < kStraightened * kStraightened || square((p1 + p2 + p2) / 3.0 - p2 + cp1) < kStraightened * kStraightened;
}

drawn_curve drawn(segment_data const &Path)
{
	drawn_curve result;
	result.p0 = Path.points[segment_data::point::start];
	result.p3 = Path.points[segment_data::point::end];
	auto const &control1{Path.points[segment_data::point::control1]};
	auto const &control2{Path.points[segment_data::point::control2]};
	if ((control1 == glm::dvec3{} && control2 == glm::dvec3{}) || straightened(Path))
	{
		result.p1 = glm::mix(result.p0, result.p3, 1.0 / 3.0);
		result.p2 = glm::mix(result.p0, result.p3, 2.0 / 3.0);
	}
	else
	{
		result.p1 = result.p0 + control1;
		result.p2 = result.p3 + control2;
	}
	return result;
}

double deviation(solved_line const &Line, drawn_curve const &Curve, double const From, double const To)
{
	auto const length{To - From};
	auto const samples{std::clamp(static_cast<int>(std::ceil(length / 0.5)), 16, 128)};
	double worst{0.0};
	for (int i = 0; i <= samples; ++i)
	{
		auto const t{static_cast<double>(i) / samples};
		auto const here{Curve.point(t)};
		glm::dvec2 const plan{here.x, here.z};
		auto s{From + length * t};
		for (int step = 0; step < 8; ++step)
		{
			auto const &e{Line.elements[Line.element_at(s)]};
			auto const local{s - e.chainage};
			auto const shift{glm::dot(plan - e.position(local), glm::normalize(e.tangent(local)))};
			s = std::clamp(s + shift, std::max(0.0, From - 1.0), std::min(Line.length, To + 1.0));
			if (std::abs(shift) < 1e-10)
				break;
		}
		auto const across{glm::distance(plan, Line.position(s))};
		auto const height{here.y - Line.elevation(s)};
		worst = std::max(worst, std::hypot(across, height));
	}
	return worst;
}

std::string generated_line::text() const
{
	std::string result;
	for (auto const &piece : pieces)
		result += piece.text;
	return result;
}

generated_line generate(network const &Network, line_id const Line, generator_options const &Options)
{
	generated_line result;
	result.line = Line;
	auto const *owner{Network.find(Line)};
	if (owner == nullptr)
	{
		result.errors.push_back(std::format("there's no line {}", number(Line)));
		return result;
	}
	if (false == owner->solved.valid())
	{
		result.errors.push_back(std::format("line {} isn't laid", number(Line)));
		return result;
	}
	if (owner->turnout != turnout_id::none)
	{
		result.errors.push_back(std::format("line {} is a track of turnout {}, made with it", number(Line), number(owner->turnout)));
		return result;
	}
	cutter pieces{Network, *owner, Options};
	result.pieces = pieces.pieces(result.errors);
	if (false == owner->hard_splits.empty())
		keep_old_tracks(*owner, result.pieces, Options);
	return result;
}

namespace
{

// the pieces of a track of the turnout between two chainages of its line, run from the node given: the pieces of a line running the other
// way turned round. A joint inside the turnout is one chainage, the same number for the parts on both sides of it
std::vector<generated_piece> run_of(network const &Network, line const &Track, node_id const From, double const A, double const B, generator_options const &Options, std::vector<std::string> &Errors)
{
	cutter cut{Network, Track, Options};
	auto const forward{Track.start == From};
	auto pieces{cut.pieces(std::min(A, B), std::max(A, B), Errors)};
	if (false == forward)
	{
		std::ranges::reverse(pieces);
		for (auto &piece : pieces)
		{
			piece.path = geometry::reversed(piece.path);
			std::swap(piece.from, piece.to);
		}
	}
	return pieces;
}

// the chainages of a track of the turnout at distances from the node it's run from, its ends exactly where it ends
struct along_track
{
	double length{0.0};
	bool forward{true};
	along_track(line const &Track, node_id const From) : length{Track.solved.length}, forward{Track.start == From} {}
	double at(double const Distance) const { return forward ? Distance : length - Distance; }
	double from_end() const { return forward ? 0.0 : length; }
	double to_end() const { return forward ? length : 0.0; }
};

// a switch track of two paths, each the pieces given
generated_part switch_part(std::string const &Name, std::array<std::vector<generated_piece>, 2> Pieces, double const Length, double const Velocity, turnout_type const &Type, generator_options const &Options)
{
	generated_part result;
	result.name = Name;
	result.is_switch = true;
	result.pieces = std::move(Pieces);
	for (std::size_t i = 0; i < 2; ++i)
		result.paths[i] = whole_path(result.pieces[i]);
	auto text{std::format("node -1 0 {} track switch {:.3f} {} {} {} velocity {} frogs ", Name, Length, Options.surface, path_text(result.paths[0]), path_text(result.paths[1]), number_text(Velocity))};
	if (Type.movable)
		text += "movablenose ";
	if (Type.heel > 0.0 || Type.bladethrow > 0.0)
		text += std::format("blades {} {} ", number_text(Type.heel), number_text(Type.bladethrow));
	for (std::size_t i = 0; i < 2; ++i)
	{
		if (result.pieces[i].size() < 2)
			continue;
		text += std::format("pieces{} {} ", i + 1, result.pieces[i].size());
		for (auto const &piece : result.pieces[i])
			text += path_text(piece.path) + " ";
	}
	result.text = text + "endtrack\n";
	return result;
}

// a plain track of the turnout: each piece a track of its own, the first one of the name given, the next ones numbered on
generated_part plain_part(std::string const &Name, std::vector<generated_piece> Pieces, double const Velocity, generator_options const &Options)
{
	generated_part result;
	result.name = Name;
	for (std::size_t i = 0; i < Pieces.size(); ++i)
	{
		auto &piece{Pieces[i]};
		piece.name = i == 0 ? Name : std::format("{}_{}", Name, i + 1);
		auto const &p{piece.path.points};
		piece.text = std::format("node -1 0 {} track normal {:.3f} {} {} {} {} {} {} {} {} velocity {} endtrack\n", piece.name, std::abs(piece.to - piece.from), Options.surface, point_text(p[segment_data::point::start]),
		                         number_text(piece.path.rolls[0]), point_text(p[segment_data::point::control1]), point_text(p[segment_data::point::control2]), point_text(p[segment_data::point::end]),
		                         number_text(piece.path.rolls[1]), number_text(piece.path.radius), number_text(Velocity));
		result.text += piece.text;
	}
	result.pieces[0] = std::move(Pieces);
	result.paths[0] = whole_path(result.pieces[0]);
	return result;
}

} // namespace

generated_turnout generate(network const &Network, turnout_id const Turnout, generator_options const &Options)
{
	generated_turnout result;
	result.turnout = Turnout;
	auto const *switched{Network.find(Turnout)};
	if (switched == nullptr)
	{
		result.errors.push_back(std::format("there's no turnout {}", number(Turnout)));
		return result;
	}
	auto const type{turnout_type_of(switched->type)};
	if (false == type.has_value())
	{
		result.errors.push_back(std::format("turnout {}: no type {}", number(Turnout), switched->type));
		return result;
	}
	result.name = std::format("{}{}", Options.turnout_prefix, number(Turnout));
	result.velocity = switched->velocity;
	for (auto const id : tracks_of(*switched))
		if (auto const *track{Network.find(id)}; track == nullptr || false == track->solved.valid())
		{
			result.errors.push_back(std::format("turnout {}: its track {} isn't laid", number(Turnout), number(id)));
			return result;
		}
	auto const &main{*Network.find(switched->main)};
	auto const whole = [&](line const &Track, node_id const From) { return run_of(Network, Track, From, 0.0, Track.solved.length, Options, result.errors); };
	auto const run = [&](line const &Track, node_id const From, double const A, double const B) { return run_of(Network, Track, From, A, B, Options, result.errors); };
	switch (type->kind)
	{
	case turnout_kind::twofold:
	{
		// the first switch up to a short plain track before the second one, which the simulation joins to a switch only through a plain track
		auto const &diverging{*Network.find(switched->diverging)};
		auto const &main2{*Network.find(switched->main2)};
		auto const &diverging2{*Network.find(switched->diverging2)};
		along_track const along{main, switched->start};
		auto const stagger{main.solved.length};
		auto const joint{along.at(stagger - std::min(1.0, stagger * 0.5))};
		result.parts.push_back(switch_part(result.name, {run(main, switched->start, along.from_end(), joint), whole(diverging, switched->start)}, stagger, result.velocity, *type, Options));
		result.parts.push_back(plain_part(result.name + "_l", run(main, switched->start, joint, along.to_end()), switched->speed_main, Options));
		result.parts.push_back(switch_part(result.name + "_2", {whole(main2, switched->second), whole(diverging2, switched->second)}, main2.solved.length, result.velocity, *type, Options));
		break;
	}
	case turnout_kind::slip:
	{
		// a switch at each leg a slip curve leaves, the tracks crossing and the middles of the slip curves plain tracks, their ends further
		// from the crossing than the simulation joins ends (build_double_slip of the switch tool)
		auto const &crossing{*Network.find(switched->crossing)};
		auto const &slip{*Network.find(switched->diverging)};
		auto const half{type->half};
		auto const gap{std::max(0.25, 0.06 / std::sin(type->angle * 0.5))};
		auto const slow{switched->speed_diverging};
		along_track const along{main, switched->start};
		auto const main_near{along.at(half - gap)};
		auto const main_far{along.at(half + gap)};
		auto const cross_near{half - gap};
		auto const cross_far{half + gap};
		auto const slip_length{slip.solved.length};
		auto const slip_near{slip_length * 0.5 - gap};
		auto const slip_far{slip_length * 0.5 + gap};
		result.parts.push_back(switch_part(result.name + "_a", {run(main, switched->start, along.from_end(), main_near), run(slip, switched->start, 0.0, slip_near)}, half, result.velocity, *type, Options));
		if (type->slip_curves >= 2)
		{
			// from B back along the main track and the second slip curve; from C along the crossing track and the second slip curve
			auto const &slip2{*Network.find(switched->diverging2)};
			auto const slip2_length{slip2.solved.length};
			auto const slip2_near{slip2_length * 0.5 - gap};
			auto const slip2_far{slip2_length * 0.5 + gap};
			result.parts.push_back(switch_part(result.name + "_b", {run(main, switched->main_end, along.to_end(), main_far), run(slip2, switched->main_end, slip2_length, slip2_far)}, half, result.velocity, *type, Options));
			result.parts.push_back(switch_part(result.name + "_c", {run(crossing, switched->cross_start, 0.0, cross_near), run(slip2, switched->cross_start, 0.0, slip2_near)}, half, result.velocity, *type, Options));
			result.parts.push_back(plain_part(result.name + "_s2", run(slip2, switched->cross_start, slip2_near, slip2_far), slow, Options));
		}
		else
		{
			result.parts.push_back(plain_part(result.name + "_b", run(main, switched->start, main_far, along.to_end()), switched->speed_main, Options));
			result.parts.push_back(plain_part(result.name + "_c", run(crossing, switched->cross_start, 0.0, cross_near), switched->speed_main, Options));
		}
		result.parts.push_back(switch_part(result.name + "_d", {run(crossing, switched->cross_end, crossing.solved.length, cross_far), run(slip, switched->cross_end, slip_length, slip_far)}, half, result.velocity, *type, Options));
		result.parts.push_back(plain_part(result.name + "_m", run(main, switched->start, main_near, main_far), switched->speed_main, Options));
		result.parts.push_back(plain_part(result.name + "_x", run(crossing, switched->cross_start, cross_near, cross_far), switched->speed_main, Options));
		result.parts.push_back(plain_part(result.name + "_s1", run(slip, switched->start, slip_near, slip_far), slow, Options));
		break;
	}
	case turnout_kind::crossing:
		// two plain tracks crossing
		result.parts.push_back(plain_part(result.name + "_1", whole(main, switched->start), switched->speed_main, Options));
		result.parts.push_back(plain_part(result.name + "_2", whole(*Network.find(switched->crossing), switched->cross_start), switched->speed_main, Options));
		break;
	case turnout_kind::ordinary:
	case turnout_kind::symmetric:
	default:
		// the main track and the diverging one, each from the start of the turnout
		result.parts.push_back(switch_part(result.name, {whole(main, switched->start), whole(*Network.find(switched->diverging), switched->start)}, type->length, result.velocity, *type, Options));
		break;
	}
	// the first switch track as the fields of a turnout of one have it
	if (result.parts.front().is_switch)
	{
		result.pieces = result.parts.front().pieces;
		result.paths = result.parts.front().paths;
	}
	for (auto const &part : result.parts)
		result.text += part.text;
	return result;
}

std::string generate_all(network const &Network, generator_options const &Options, std::vector<std::string> &Errors)
{
	std::string result;
	for (auto const &[id, owner] : Network.lines())
	{
		if (owner.turnout != turnout_id::none)
			continue;
		auto const made{generate(Network, id, Options)};
		Errors.insert(Errors.end(), made.errors.begin(), made.errors.end());
		result += made.text();
	}
	for (auto const &[id, each] : Network.turnouts())
	{
		auto const made{generate(Network, id, Options)};
		Errors.insert(Errors.end(), made.errors.begin(), made.errors.end());
		if (made.errors.empty())
			result += made.text;
	}
	return result;
}

} // namespace layout
