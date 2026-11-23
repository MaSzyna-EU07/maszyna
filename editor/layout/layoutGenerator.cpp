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
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace layout
{

namespace
{

double constexpr kStraightened{0.02}; // m, TTrack::straightened: control vectors this close to the third of the chord are taken for none
double constexpr kShortestPiece{0.1}; // m: the simulation takes a path under 5 cm for one of no length
int constexpr kDeepest{14}; // halvings of a piece, at most

// a point of the line where a piece ends: at a node with the numbers of the node
struct piece_end
{
	double chainage{0.0};
	glm::dvec3 point{0.0};
	glm::dvec2 heading{0.0, 1.0};
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
			kept.cant = at != nullptr ? at->cant * sense : m_solved.cant_at(chainage);
			kept.grade = m_solved.grade(chainage);
		}
	}

	std::vector<generated_piece> pieces(std::vector<std::string> &Errors)
	{
		auto const cuts{breaks()};
		std::vector<generated_piece> result;
		auto from{m_start};
		for (std::size_t i = 1; i < cuts.size(); ++i)
		{
			auto const to{i + 1 == cuts.size() ? m_end : end_at(cuts[i])};
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
		std::ranges::sort(result);
		std::erase_if(result, [&](double const Chainage) { return Chainage < 0.0 || Chainage > m_solved.length; });
		auto const last{std::unique(result.begin(), result.end(), [](double const A, double const B) { return std::abs(A - B) < 1e-9; })};
		result.erase(last, result.end());
		result.front() = 0.0;
		result.back() = m_solved.length;
		for (std::size_t i = 0; i + 1 < result.size() && result.size() > 2;)
		{
			if (result[i + 1] - result[i] >= kShortestPiece)
			{
				++i;
				continue;
			}
			// the short stretch goes to its neighbour: the break after it goes, or the one before it at the end
			result.erase(result.begin() + static_cast<std::ptrdiff_t>(i + 1 + 1 < result.size() ? i + 1 : i));
		}
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
		auto const turn{geometry::signed_angle(From.heading, To.heading)};
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
			handles = curvature_handles(p0, From.heading, p3, To.heading, curvature_inside(From.chainage, true), curvature_inside(To.chainage, false), handle);
		glm::dvec2 p1{p0 + From.heading * handles.first};
		glm::dvec2 p2{p3 - To.heading * handles.second};
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
		auto const &p{path.points};
		auto const point = [](glm::dvec3 const &Point) { return std::format("{} {} {}", number_text(Point.x), number_text(Point.y), number_text(Point.z)); };
		piece.text = std::format("node -1 0 {} track normal {:.3f} {} {} {} {} {} {} {} {} velocity {} endtrack\n", piece.name, To.chainage - From.chainage, m_options.surface, point(p[segment_data::point::start]),
		                         number_text(path.rolls[0]), point(p[segment_data::point::control1]), point(p[segment_data::point::control2]), point(p[segment_data::point::end]), number_text(path.rolls[1]),
		                         number_text(path.radius), number_text(m_line.category.vmax));
		Into.push_back(std::move(piece));
	}

	std::map<std::string, int> m_counts;
};

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
	if (auto const hard{Options.hard_splits.find(Line)}; hard != Options.hard_splits.end() && false == hard->second.empty())
	{
		result.errors.push_back(std::format("line {}: the joints of old tracks as hard points of the pieces come with stage E6", number(Line)));
		return result;
	}
	cutter pieces{Network, *owner, Options};
	result.pieces = pieces.pieces(result.errors);
	return result;
}

std::string generate_all(network const &Network, generator_options const &Options, std::vector<std::string> &Errors)
{
	std::string result;
	for (auto const &[id, owner] : Network.lines())
	{
		auto const made{generate(Network, id, Options)};
		Errors.insert(Errors.end(), made.errors.begin(), made.errors.end());
		result += made.text();
	}
	return result;
}

} // namespace layout
