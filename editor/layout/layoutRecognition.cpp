/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutRecognition.hpp"
#include "editor/layout/layoutGenerator.hpp"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>

namespace layout
{

namespace
{

double constexpr kRailBase{1500.0}; // mm between the middles of the rail heads, the base of the roll
int constexpr kTable{256}; // the arc length of a path tabulated at this many parameters
double constexpr kWindow{30.0}; // m the heading is differenced over, to read the curvature through the joints and the noise of the paths
double constexpr kCurved{2e-4}; // 1/m read so: a curve of a radius up to 5 km; the larger ones are found where the line keeps off the paths
double constexpr kReach{2.0}; // m: the straights read from this far off the curves found
double constexpr kKink{5e-3}; // rad: the paths turning more at a joint meet with no curve; less, the noise of the ends of the paths
double constexpr kSmallestTurn{2e-4}; // rad: a curve turning less is the noise of the paths
double constexpr kCantStep{0.5}; // mm, as the line takes it
double constexpr kCompound{0.25}; // a share of the curvature the halves of an arc may differ by and still be one arc
double constexpr kDip{0.35}; // a share of the curvature around: falling under it inside a curve, two curves
double constexpr kSmallTurn{0.035}; // rad, 2 degree: a curve turning less over a window or two may be the jags of old tracks
double constexpr kAstray{1.0}; // m: a line so far off the points was led astray by the curves found
double constexpr kLongestTurn{std::numbers::pi / 2.0}; // rad, a quarter of the circle: a curve turning more is laid as curves of it at most
int constexpr kRefits{6}; // curves added where the line keeps off the paths, at most

double sign_of(double const Value)
{
	return Value < 0.0 ? -1.0 : 1.0;
}

// the Bezier curve of a path as the chain runs it, the arc length in the plan along it
class path_curve
{
  public:
	explicit path_curve(chain_path const &Part) : m_curve{drawn(Part.path)}
	{
		m_rolls = {Part.path.rolls[0], Part.path.rolls[1]};
		if (false == Part.forward)
		{
			std::swap(m_curve.p0, m_curve.p3);
			std::swap(m_curve.p1, m_curve.p2);
			m_rolls = {-Part.path.rolls[1], -Part.path.rolls[0]};
		}
		m_table[0] = 0.0;
		auto previous{plan(0.0)};
		for (int i = 1; i <= kTable; ++i)
		{
			auto const next{plan(static_cast<double>(i) / kTable)};
			m_table[i] = m_table[i - 1] + glm::distance(previous, next);
			previous = next;
		}
	}

	double length() const { return m_table[kTable]; }

	// the parameter of the curve at the arc length
	double parameter(double const S) const
	{
		if (S <= 0.0)
			return 0.0;
		if (S >= length())
			return 1.0;
		auto const next{std::upper_bound(m_table.begin(), m_table.end(), S)};
		auto const i{static_cast<std::size_t>(std::max<std::ptrdiff_t>(1, next - m_table.begin()))};
		auto const span{m_table[i] - m_table[i - 1]};
		auto t{(static_cast<double>(i - 1) + (span > 0.0 ? (S - m_table[i - 1]) / span : 0.0)) / kTable};
		// a step of Newton's method on the chords of the table
		auto const speed{glm::length(first_plan(t))};
		auto const chord_s{m_table[i - 1] + glm::distance(plan(static_cast<double>(i - 1) / kTable), plan(t))};
		if (speed > 1e-9)
			t = std::clamp(t + (S - chord_s) / speed, 0.0, 1.0);
		return t;
	}

	glm::dvec3 point(double const T) const { return m_curve.point(T); }
	glm::dvec3 first(double const T) const
	{
		auto const u{1.0 - T};
		return 3.0 * (u * u * (m_curve.p1 - m_curve.p0) + 2.0 * u * T * (m_curve.p2 - m_curve.p1) + T * T * (m_curve.p3 - m_curve.p2));
	}
	glm::dvec3 second(double const T) const { return 6.0 * ((1.0 - T) * (m_curve.p2 - 2.0 * m_curve.p1 + m_curve.p0) + T * (m_curve.p3 - 2.0 * m_curve.p2 + m_curve.p1)); }
	// mm, the roll turned evenly with the parameter, as the simulator turns it
	double cant(double const T) const { return cant_of(std::lerp(static_cast<double>(m_rolls[0]), static_cast<double>(m_rolls[1]), T)); }

	chain_sample sample(double const S, std::size_t const Part, double const Start) const
	{
		auto const t{parameter(S)};
		chain_sample result;
		result.s = Start + std::clamp(S, 0.0, length());
		result.point = point(t);
		auto const d1{first_plan(t)};
		auto const speed{glm::length(d1)};
		result.heading = speed > 1e-12 ? d1 / speed : chord_heading();
		if (speed > 1e-12)
			result.curvature = geometry::cross(d1, geometry::plan_of(second(t))) / (speed * speed * speed);
		result.cant = cant(t);
		result.part = Part;
		return result;
	}

  private:
	drawn_curve m_curve;
	std::array<float, 2> m_rolls{};
	std::array<double, kTable + 1> m_table{};

	glm::dvec2 plan(double const T) const { return geometry::plan_of(m_curve.point(T)); }
	glm::dvec2 first_plan(double const T) const { return geometry::plan_of(first(T)); }
	glm::dvec2 chord_heading() const
	{
		auto const chord{geometry::plan_of(m_curve.p3 - m_curve.p0)};
		return glm::length(chord) > 1e-12 ? glm::normalize(chord) : glm::dvec2{0.0, 1.0};
	}
};

// the chainage of the line nearest the point, by Newton's method along the tangent from the guess
double project(solved_line const &Line, glm::dvec2 const &Point, double const Guess)
{
	auto s{std::clamp(Guess, 0.0, Line.length)};
	for (int step = 0; step < 12; ++step)
	{
		auto const &e{Line.elements[Line.element_at(s)]};
		auto const local{std::clamp(s - e.chainage, 0.0, e.length)};
		auto const tangent{glm::normalize(e.tangent(local))};
		auto const shift{glm::dot(Point - e.position(local), tangent)};
		auto const next{std::clamp(s + shift, 0.0, Line.length)};
		auto const moved{std::abs(next - s)};
		s = next;
		if (moved < 1e-9)
			break;
	}
	return s;
}

// the unwrapped headings of the samples, the turns at the joints included
std::vector<double> headings(std::vector<chain_sample> const &Samples)
{
	std::vector<double> result(Samples.size(), 0.0);
	if (Samples.empty())
		return result;
	result[0] = std::atan2(Samples[0].heading.y, Samples[0].heading.x);
	for (std::size_t i = 1; i < Samples.size(); ++i)
		result[i] = result[i - 1] + geometry::signed_angle(Samples[i - 1].heading, Samples[i].heading);
	return result;
}

// a value of the samples at the arc length, interpolated
double at_length(std::vector<chain_sample> const &Samples, std::vector<double> const &Values, double const S)
{
	if (S <= Samples.front().s)
		return Values.front();
	if (S >= Samples.back().s)
		return Values.back();
	auto const next{std::upper_bound(Samples.begin(), Samples.end(), S, [](double const Value, chain_sample const &Sample) { return Value < Sample.s; })};
	auto const i{static_cast<std::size_t>(next - Samples.begin())};
	auto const span{Samples[i].s - Samples[i - 1].s};
	auto const t{span > 0.0 ? (S - Samples[i - 1].s) / span : 1.0};
	return std::lerp(Values[i - 1], Values[i], t);
}

std::size_t sample_at(std::vector<chain_sample> const &Samples, double const S)
{
	auto const next{std::lower_bound(Samples.begin(), Samples.end(), S, [](chain_sample const &Sample, double const Value) { return Sample.s < Value; })};
	return std::min(static_cast<std::size_t>(next - Samples.begin()), Samples.size() - 1);
}

// the curvature read off the change of the heading over the reach each way: the joints and their kinks among it, the ripple of the
// Bezier curves and the noise of their ends evened out
std::vector<double> read_curvature(std::vector<chain_sample> const &Samples, std::vector<double> const &Headings, double const Reach)
{
	std::vector<double> result(Samples.size(), 0.0);
	auto const total{Samples.back().s};
	for (std::size_t i = 0; i < Samples.size(); ++i)
	{
		auto const a{std::max(0.0, Samples[i].s - Reach)};
		auto const b{std::min(total, Samples[i].s + Reach)};
		if (b - a > 1e-6)
			result[i] = (at_length(Samples, Headings, b) - at_length(Samples, Headings, a)) / (b - a);
	}
	return result;
}

// the solution of the square system, by elimination with the largest pivot; false where it's singular
bool solve_system(std::vector<double> &Matrix, std::vector<double> &Right, std::size_t const Size)
{
	auto const at = [&Matrix, Size](std::size_t const Row, std::size_t const Column) -> double & { return Matrix[Row * Size + Column]; };
	for (std::size_t column = 0; column < Size; ++column)
	{
		auto pivot{column};
		for (auto row = column + 1; row < Size; ++row)
			if (std::abs(at(row, column)) > std::abs(at(pivot, column)))
				pivot = row;
		if (std::abs(at(pivot, column)) < 1e-300)
			return false;
		if (pivot != column)
		{
			for (std::size_t k = 0; k < Size; ++k)
				std::swap(at(pivot, k), at(column, k));
			std::swap(Right[pivot], Right[column]);
		}
		for (auto row = column + 1; row < Size; ++row)
		{
			auto const factor{at(row, column) / at(column, column)};
			if (factor == 0.0)
				continue;
			for (auto k = column; k < Size; ++k)
				at(row, k) -= factor * at(column, k);
			Right[row] -= factor * Right[column];
		}
	}
	for (auto row = Size; row-- > 0;)
	{
		auto value{Right[row]};
		for (auto k = row + 1; k < Size; ++k)
			value -= at(row, k) * Right[k];
		Right[row] = value / at(row, row);
	}
	return true;
}

// a least squares problem of a few parameters: the residuals of a range of them, the parameters each one moves and their bounds
struct least_squares
{
	std::function<bool(std::vector<double> const &, std::vector<double> &, std::size_t, std::size_t)> residuals; // false: no solution there
	std::size_t count{0}; // of the residuals
	std::vector<double> steps; // of the differences, by parameter
	std::vector<double> lower;
	std::vector<double> upper;
	std::vector<std::pair<std::size_t, std::size_t>> reach; // the residuals each parameter moves
	std::vector<double> stride; // the most a parameter moves in a step; the step shortened as a whole to keep within
};

double square_sum(std::vector<double> const &Values)
{
	return std::accumulate(Values.begin(), Values.end(), 0.0, [](double const Sum, double const Value) { return Sum + Value * Value; });
}

// Levenberg-Marquardt, the Jacobian by forward differences over the residuals each parameter moves; the parameters kept within their bounds
void minimize(least_squares const &Problem, std::vector<double> &Parameters, int const Iterations)
{
	auto const n{Parameters.size()};
	if (n == 0 || Problem.count == 0)
		return;
	std::vector<double> residuals(Problem.count, 0.0);
	if (false == Problem.residuals(Parameters, residuals, 0, Problem.count))
		return;
	auto cost{square_sum(residuals)};
	double lambda{1e-3};
	std::vector<double> jacobian(n * Problem.count, 0.0);
	std::vector<double> moved(Problem.count, 0.0);
	for (int iteration = 0; iteration < Iterations; ++iteration)
	{
		std::ranges::fill(jacobian, 0.0);
		for (std::size_t j = 0; j < n; ++j)
		{
			// a parameter held where it is: no column
			if (Problem.lower[j] == Problem.upper[j])
				continue;
			auto probe{Parameters};
			auto step{Problem.steps[j]};
			if (probe[j] + step > Problem.upper[j])
				step = -step;
			probe[j] += step;
			auto const [from, to]{Problem.reach[j]};
			if (false == Problem.residuals(probe, moved, from, to))
				continue;
			for (auto i = from; i < to; ++i)
				jacobian[j * Problem.count + i] = (moved[i] - residuals[i]) / step;
		}
		std::vector<double> normal(n * n, 0.0);
		std::vector<double> gradient(n, 0.0);
		for (std::size_t a = 0; a < n; ++a)
		{
			auto const [from, to]{Problem.reach[a]};
			for (auto i = from; i < to; ++i)
				gradient[a] += jacobian[a * Problem.count + i] * residuals[i];
			for (std::size_t b = a; b < n; ++b)
			{
				auto const low{std::max(from, Problem.reach[b].first)};
				auto const high{std::min(to, Problem.reach[b].second)};
				double sum{0.0};
				for (auto i = low; i < high; ++i)
					sum += jacobian[a * Problem.count + i] * jacobian[b * Problem.count + i];
				normal[a * n + b] = sum;
				normal[b * n + a] = sum;
			}
		}
		bool improved{false};
		for (int attempt = 0; attempt < 8 && false == improved; ++attempt)
		{
			auto system{normal};
			for (std::size_t a = 0; a < n; ++a)
				system[a * n + a] += lambda * std::max(normal[a * n + a], 1e-12);
			auto delta{gradient};
			for (auto &value : delta)
				value = -value;
			if (false == solve_system(system, delta, n))
			{
				lambda *= 10.0;
				continue;
			}
			double shorten{1.0};
			for (std::size_t a = 0; a < n; ++a)
				if (std::abs(delta[a]) * shorten > Problem.stride[a])
					shorten = Problem.stride[a] / std::abs(delta[a]);
			auto candidate{Parameters};
			for (std::size_t a = 0; a < n; ++a)
				candidate[a] = std::clamp(candidate[a] + delta[a] * shorten, Problem.lower[a], Problem.upper[a]);
			std::vector<double> next(Problem.count, 0.0);
			if (Problem.residuals(candidate, next, 0, Problem.count))
			{
				auto const value{square_sum(next)};
				if (value < cost)
				{
					auto const gain{cost - value};
					Parameters = std::move(candidate);
					residuals = std::move(next);
					cost = value;
					lambda = std::max(lambda / 3.0, 1e-9);
					improved = true;
					if (gain < 1e-12 * (cost + 1e-12))
						return;
					continue;
				}
			}
			lambda *= 4.0;
		}
		if (false == improved)
			return;
	}
}

// the plan of the line fitted to the points of the chain: each vertex its point (or the distance along the heading an end is held in),
// the curvature of its arc and the lengths of its transitions
class plan_fit
{
  public:
	plan_fit(std::vector<chain_sample> const &Samples, recognition_options const &Options) : m_samples{Samples}, m_options{Options}
	{
		m_start = Samples.front().point;
		m_end = Samples.back().point;
		m_total = Samples.back().s;
		m_guess.resize(Samples.size());
		for (std::size_t i = 0; i < Samples.size(); ++i)
			m_guess[i] = Samples[i].s;
	}

	// the curves as found, or as the last fit left them
	void start_from(std::vector<vertex> Vertices, std::vector<found_curve> Spans)
	{
		m_vertices = std::move(Vertices);
		m_spans = std::move(Spans);
		m_lowest_in.assign(m_vertices.size(), 0.0);
		m_lowest_out.assign(m_vertices.size(), 0.0);
	}
	// the transitions of the vertex no shorter than these: a cant ramped along them
	void hold_transitions(std::size_t const Vertex, double const In, double const Out)
	{
		m_lowest_in[Vertex] = In;
		m_lowest_out[Vertex] = Out;
		m_vertices[Vertex].transition_in = std::max(m_vertices[Vertex].transition_in, In);
		m_vertices[Vertex].transition_out = std::max(m_vertices[Vertex].transition_out, Out);
	}

	std::vector<vertex> const &vertices() const { return m_vertices; }
	std::vector<found_curve> const &spans() const { return m_spans; }

	solved_line lay(std::vector<vertex> const &Vertices) const
	{
		line_shape shape;
		shape.vertices = Vertices;
		shape.transition = m_options.transition;
		for (std::size_t k = 0; k < shape.vertices.size(); ++k)
		{
			shape.vertices[k].id = static_cast<vertex_id>(k + 1);
			shape.vertices[k].cant = 0.0;
		}
		end_condition start;
		start.position = m_start;
		end_condition end;
		end.position = m_end;
		return solve(shape, start, end);
	}

	// the vertices and the radii first, the transitions held as they are, then all of it: a transition grown early bends a curve which is
	// too sharp out of the way of the points rather than the radius growing
	void fit(int const Iterations)
	{
		if (m_vertices.empty())
			return;
		make_layable();
		for (auto const held : {true, false})
		{
			auto parameters{pack()};
			least_squares problem;
			problem.count = m_samples.size();
			problem.residuals = [this](std::vector<double> const &Parameters, std::vector<double> &Out, std::size_t From, std::size_t To) { return residuals(Parameters, Out, From, To); };
			bounds(problem, held);
			minimize(problem, parameters, Iterations);
			m_vertices = unpack(parameters);
		}
	}

  private:
	std::vector<chain_sample> const &m_samples;
	recognition_options const &m_options;
	glm::dvec3 m_start{0.0};
	glm::dvec3 m_end{0.0};
	double m_total{0.0};
	std::vector<vertex> m_vertices;
	std::vector<found_curve> m_spans;
	std::vector<double> m_lowest_in;
	std::vector<double> m_lowest_out;
	mutable std::vector<double> m_guess;

	// curves as found which can't be laid (their transitions overlapping, a curve reaching past an end): the transitions taken out, the
	// radii made smaller until they can
	void make_layable()
	{
		if (lay(m_vertices).valid())
			return;
		for (std::size_t k = 0; k < m_vertices.size(); ++k)
		{
			m_vertices[k].transition_in = m_lowest_in[k];
			m_vertices[k].transition_out = m_lowest_out[k];
		}
		for (int round = 0; round < 200 && false == lay(m_vertices).valid(); ++round)
			for (auto &v : m_vertices)
				v.radius = std::max(1.0, v.radius * 0.9);
		// a vertex no radius lays (behind an end, behind the one before it): the one turning least taken out, until the rest can be laid
		while (false == m_vertices.empty() && false == lay(m_vertices).valid())
		{
			auto const k{least_turning()};
			m_vertices.erase(m_vertices.begin() + static_cast<std::ptrdiff_t>(k));
			m_spans.erase(m_spans.begin() + static_cast<std::ptrdiff_t>(k));
			m_lowest_in.erase(m_lowest_in.begin() + static_cast<std::ptrdiff_t>(k));
			m_lowest_out.erase(m_lowest_out.begin() + static_cast<std::ptrdiff_t>(k));
		}
	}

	// the vertex whose tangents turn least
	std::size_t least_turning() const
	{
		std::vector<glm::dvec2> polygon{start_plan()};
		for (auto const &v : m_vertices)
			polygon.push_back(v.point);
		polygon.push_back(end_plan());
		std::size_t result{0};
		double least{std::numeric_limits<double>::max()};
		for (std::size_t k = 0; k < m_vertices.size(); ++k)
		{
			auto const in{polygon[k + 1] - polygon[k]};
			auto const out{polygon[k + 2] - polygon[k + 1]};
			auto const turn{glm::length(in) > 0.0 && glm::length(out) > 0.0 ? std::abs(geometry::signed_angle(in, out)) : 0.0};
			if (turn < least)
			{
				least = turn;
				result = k;
			}
		}
		return result;
	}

	bool start_held() const { return m_options.start.held; }
	bool end_held() const { return m_options.end.held; }
	glm::dvec2 start_plan() const { return geometry::plan_of(m_start); }
	glm::dvec2 end_plan() const { return geometry::plan_of(m_end); }
	// the point of the vertex is given by the ends held: the only vertex between two ends held, where their headings cross
	bool fixed_point() const { return m_vertices.size() == 1 && start_held() && end_held(); }
	bool on_start_ray(std::size_t const K) const { return K == 0 && start_held() && false == fixed_point(); }
	bool on_end_ray(std::size_t const K) const { return K + 1 == m_vertices.size() && end_held() && false == fixed_point() && false == on_start_ray(K); }

	std::vector<double> pack() const
	{
		std::vector<double> result;
		for (std::size_t k = 0; k < m_vertices.size(); ++k)
		{
			auto const &v{m_vertices[k]};
			if (on_start_ray(k))
				result.push_back(glm::dot(v.point - start_plan(), m_options.start.heading));
			else if (on_end_ray(k))
				result.push_back(glm::dot(end_plan() - v.point, m_options.end.heading));
			else if (false == fixed_point())
			{
				result.push_back(v.point.x);
				result.push_back(v.point.y);
			}
			result.push_back(1.0 / v.radius);
			// the transitions by their squares: what a transition moves the curve by grows with the square of its length, so a step of
			// a transition of none moves it at all
			result.push_back(v.transition_in * v.transition_in);
			result.push_back(v.transition_out * v.transition_out);
		}
		return result;
	}

	std::vector<vertex> unpack(std::vector<double> const &Parameters) const
	{
		auto result{m_vertices};
		std::size_t i{0};
		for (std::size_t k = 0; k < result.size(); ++k)
		{
			auto &v{result[k]};
			if (on_start_ray(k))
				v.point = start_plan() + m_options.start.heading * Parameters[i++];
			else if (on_end_ray(k))
				v.point = end_plan() - m_options.end.heading * Parameters[i++];
			else if (fixed_point())
			{
				double along{0.0};
				double back{0.0};
				if (geometry::lines_cross(start_plan(), m_options.start.heading, end_plan(), m_options.end.heading, along, back))
					v.point = start_plan() + m_options.start.heading * along;
			}
			else
			{
				v.point.x = Parameters[i++];
				v.point.y = Parameters[i++];
			}
			v.radius = 1.0 / Parameters[i++];
			v.transition_in = std::sqrt(std::max(0.0, Parameters[i++]));
			v.transition_out = std::sqrt(std::max(0.0, Parameters[i++]));
		}
		return result;
	}

	// the samples between the curve before the vertex and the one after it: what its parameters move
	std::pair<std::size_t, std::size_t> reach(std::size_t const K) const
	{
		auto const margin{30.0};
		auto const from{K == 0 ? 0.0 : m_spans[K - 1].from - margin};
		auto const to{K + 1 >= m_spans.size() ? m_total : m_spans[K + 1].to + margin};
		auto const first{from <= 0.0 ? 0 : sample_at(m_samples, from)};
		auto const last{to >= m_total ? m_samples.size() : sample_at(m_samples, to) + 1};
		return {first, std::max(first, last)};
	}

	void bounds(least_squares &Problem, bool const Held) const
	{
		for (std::size_t k = 0; k < m_vertices.size(); ++k)
		{
			auto const span{reach(k)};
			auto const add = [&Problem, &span](double const Step, double const Low, double const High, double const Stride) {
				Problem.stride.push_back(Stride);
				Problem.steps.push_back(Step);
				Problem.lower.push_back(Low);
				Problem.upper.push_back(High);
				Problem.reach.push_back(span);
			};
			auto const limit{std::numeric_limits<double>::max()};
			if (on_start_ray(k) || on_end_ray(k))
				add(0.01, 0.0, limit, 5.0);
			else if (false == fixed_point())
			{
				add(0.01, -limit, limit, 5.0);
				add(0.01, -limit, limit, 5.0);
			}
			auto const curvature{1.0 / m_vertices[k].radius};
			add(std::max(1e-9, curvature * 1e-4), 1e-6, 1.0, curvature * 0.3 + 1e-5);
			for (auto const &[lowest, now] : {std::pair{m_lowest_in[k], m_vertices[k].transition_in}, std::pair{m_lowest_out[k], m_vertices[k].transition_out}})
			{
				if (Held)
					add(1.0, now * now, now * now, 0.0);
				else
					add(1.0, lowest * lowest, limit, 400.0);
			}
		}
	}

	bool residuals(std::vector<double> const &Parameters, std::vector<double> &Out, std::size_t const From, std::size_t const To) const
	{
		auto const line{lay(unpack(Parameters))};
		if (false == line.valid())
			return false;
		auto const scale{m_total > 0.0 ? line.length / m_total : 1.0};
		for (auto i = From; i < To; ++i)
		{
			auto const &sample{m_samples[i]};
			auto const plan{geometry::plan_of(sample.point)};
			auto const s{project(line, plan, m_guess[i] > 0.0 ? m_guess[i] : sample.s * scale)};
			auto const tangent{glm::normalize(line.tangent(s))};
			Out[i] = geometry::cross(tangent, plan - line.position(s));
			if (From == 0 && To == m_samples.size())
				m_guess[i] = s;
		}
		return true;
	}
};

// the curve of a run of the curvature: its turn over the run and around it, its arc where the curvature is half of the arc's or more (the
// area of the curvature over the width between those points is the turn), its transitions where the curvature rises from a tenth of
// the arc's to nine tenths, the window it was read over taken off; a turn at a joint the run is no wider than the window around is a kink
std::optional<found_curve> read_curve(std::vector<chain_sample> const &Samples, std::vector<double> const &Headings, std::vector<double> const &Curvature,
                                      std::pair<std::size_t, std::size_t> const &Run, std::pair<double, double> const &Room)
{
	auto const [first, last]{Run};
	found_curve curve;
	auto const before{std::max(Room.first, Samples[first].s - kWindow * 0.5)};
	auto const after{std::min(Room.second, Samples[last].s + kWindow * 0.5)};
	curve.deflection = at_length(Samples, Headings, after) - at_length(Samples, Headings, before);
	if (std::abs(curve.deflection) < kSmallestTurn)
		return std::nullopt;
	double peak{0.0};
	for (auto k = first; k <= last; ++k)
		peak = std::max(peak, std::abs(Curvature[k]));
	auto plateau{peak};
	double half_in{Samples[first].s};
	double half_out{Samples[last].s};
	for (int pass = 0; pass < 4; ++pass)
	{
		auto k{first};
		while (k < last && std::abs(Curvature[k]) < plateau * 0.5)
			++k;
		half_in = Samples[k].s;
		k = last;
		while (k > first && std::abs(Curvature[k]) < plateau * 0.5)
			--k;
		half_out = Samples[k].s;
		if (half_out < half_in)
			std::swap(half_in, half_out);
		plateau = std::abs(curve.deflection) / std::max(half_out - half_in, 0.5);
	}
	auto const ramp = [&Samples, &Curvature, first, last, plateau](bool const In) {
		auto k{In ? first : last};
		auto const next = [In](std::size_t const At) { return In ? At + 1 : At - 1; };
		auto const inside = [In, first, last](std::size_t const At) { return In ? At < last : At > first; };
		while (inside(k) && std::abs(Curvature[k]) < plateau * 0.1)
			k = next(k);
		auto const low{Samples[k].s};
		while (inside(k) && std::abs(Curvature[k]) < plateau * 0.9)
			k = next(k);
		auto const raw{std::abs(Samples[k].s - low) / 0.8};
		return std::sqrt(std::max(0.0, raw * raw - kWindow * kWindow));
	};
	curve.radius = 1.0 / std::max(plateau, 1e-6);
	curve.transition_in = std::min(ramp(true), std::max(0.0, half_out - half_in));
	curve.transition_out = std::min(ramp(false), std::max(0.0, half_out - half_in));
	curve.from = std::max(before, half_in - curve.transition_in * 0.5);
	curve.to = std::min(after, half_out + curve.transition_out * 0.5);
	// a turn at one joint making up most of the curve, no wider than the window: a kink
	if (Samples[last].s - Samples[first].s <= kWindow + 2.0)
		for (auto k = first; k <= last; ++k)
			if (std::abs(Samples[k].joint_turn) > kKink && std::abs(Samples[k].joint_turn) > 0.5 * std::abs(curve.deflection))
			{
				curve.kink = true;
				curve.from = std::max(0.0, Samples[k].s - 1.0);
				curve.to = std::min(Samples.back().s, Samples[k].s + 1.0);
				curve.radius = std::max(1.0, 2.0 / std::abs(curve.deflection));
				curve.transition_in = 0.0;
				curve.transition_out = 0.0;
				return curve;
			}
	// arcs of different radii: the mean curvature of the halves of the arc, clear of the window around its ends, differing by more than a
	// share of it
	auto const middle_in{half_in + curve.transition_in * 0.5 + kWindow * 0.5};
	auto const middle_out{half_out - curve.transition_out * 0.5 - kWindow * 0.5};
	if (middle_out - middle_in > 40.0)
	{
		auto const middle{0.5 * (middle_in + middle_out)};
		std::array<double, 2> sums{0.0, 0.0};
		std::array<double, 2> counts{0.0, 0.0};
		for (auto k = first; k <= last; ++k)
			if (Samples[k].s >= middle_in && Samples[k].s <= middle_out)
			{
				auto const half{Samples[k].s < middle ? 0u : 1u};
				sums[half] += std::abs(Curvature[k]);
				counts[half] += 1.0;
			}
		if (counts[0] > 0.0 && counts[1] > 0.0)
		{
			auto const a{sums[0] / counts[0]};
			auto const b{sums[1] / counts[1]};
			curve.compound = std::abs(a - b) > kCompound * std::max(a, b);
		}
	}
	return curve;
}

// a run of the curvature of one sign cut where it falls to a third of what it is on both sides of there: two curves turning alike with a
// short straight between them, which the window of the reading runs together
void split_run(std::vector<double> const &Curvature, std::pair<std::size_t, std::size_t> const &Run, std::vector<std::pair<std::size_t, std::size_t>> &Into)
{
	auto const [first, last]{Run};
	double peak{0.0};
	for (auto k = first; k <= last; ++k)
		peak = std::max(peak, std::abs(Curvature[k]));
	// the highest curvature from each point on to the end of the run
	std::vector<double> ahead(last - first + 2, 0.0);
	for (auto k = last + 1; k-- > first;)
		ahead[k - first] = std::max(ahead[k - first + 1], std::abs(Curvature[k]));
	// the lowest point inside the run with as much as 0.6 of the peak on both sides of it
	std::optional<std::size_t> cut;
	double before{0.0};
	for (auto k = first; k <= last; ++k)
	{
		before = std::max(before, std::abs(Curvature[k]));
		auto const after{ahead[k - first]};
		if (before < 0.6 * peak || after < 0.6 * peak || (cut.has_value() && std::abs(Curvature[k]) >= std::abs(Curvature[*cut])))
			continue;
		if (std::abs(Curvature[k]) < kDip * std::min(before, after))
			cut = k;
	}
	if (false == cut.has_value() || *cut == first || *cut == last)
	{
		Into.emplace_back(first, last);
		return;
	}
	split_run(Curvature, {first, *cut - 1}, Into);
	split_run(Curvature, {*cut + 1, last}, Into);
}

// a curve turning by more than a quarter of the circle laid as curves of a quarter at most, one after another (a vertex of the domain
// turns by less than a half): the transitions at the ends of the first and the last
void split_turn(std::vector<chain_sample> const &Samples, std::vector<double> const &Headings, found_curve const &Curve, std::vector<found_curve> &Into)
{
	auto const parts{static_cast<int>(std::ceil(std::abs(Curve.deflection) / kLongestTurn - 1e-9))};
	if (parts <= 1 || Curve.kink)
	{
		Into.push_back(Curve);
		return;
	}
	auto const start{at_length(Samples, Headings, Curve.from)};
	auto const share{Curve.deflection / parts};
	auto from{Curve.from};
	for (int part = 1; part <= parts; ++part)
	{
		// where the heading has turned by the share so far
		auto to{Curve.to};
		if (part < parts)
		{
			auto k{sample_at(Samples, from)};
			while (k + 1 < Samples.size() && Samples[k].s < Curve.to && std::abs(Headings[k] - start) < std::abs(share * part))
				++k;
			to = Samples[k].s;
		}
		found_curve piece{Curve};
		piece.from = from;
		piece.to = to;
		piece.deflection = at_length(Samples, Headings, to) - at_length(Samples, Headings, from);
		piece.transition_in = part == 1 ? Curve.transition_in : 0.0;
		piece.transition_out = part == parts ? Curve.transition_out : 0.0;
		Into.push_back(piece);
		from = to;
	}
}

// the straight through the samples between the arc lengths, by least squares across it: a point of it and its heading as the chain runs;
// none where there are too few of them or they're too short
std::optional<std::pair<glm::dvec2, glm::dvec2>> straight_through(std::vector<chain_sample> const &Samples, double const From, double const To)
{
	if (To - From < 5.0)
		return std::nullopt;
	glm::dvec2 mean{0.0};
	std::size_t count{0};
	for (auto i = sample_at(Samples, From); i < Samples.size() && Samples[i].s <= To; ++i)
	{
		mean += geometry::plan_of(Samples[i].point);
		++count;
	}
	if (count < 3)
		return std::nullopt;
	mean /= static_cast<double>(count);
	double xx{0.0};
	double xy{0.0};
	double yy{0.0};
	for (auto i = sample_at(Samples, From); i < Samples.size() && Samples[i].s <= To; ++i)
	{
		auto const d{geometry::plan_of(Samples[i].point) - mean};
		xx += d.x * d.x;
		xy += d.x * d.y;
		yy += d.y * d.y;
	}
	auto const angle{0.5 * std::atan2(2.0 * xy, xx - yy)};
	glm::dvec2 direction{std::cos(angle), std::sin(angle)};
	auto const along{geometry::plan_of(Samples[sample_at(Samples, To)].point - Samples[sample_at(Samples, From)].point)};
	if (glm::dot(direction, along) < 0.0)
		direction = -direction;
	return std::make_pair(mean, direction);
}

// the tangent of the chain before the curve (Before) or after it: the straight through the samples between it and the curve next to it,
// or the heading of the chain right at the curve where there's no straight to speak of
std::pair<glm::dvec2, glm::dvec2> tangent_of(std::vector<chain_sample> const &Samples, double const From, double const To, bool const Before)
{
	if (auto const straight{straight_through(Samples, From, To)})
		return *straight;
	auto const &at{Samples[sample_at(Samples, Before ? To : From)]};
	return {geometry::plan_of(at.point), at.heading};
}

// the initial vertex of a curve found: where the tangents before and after it cross
std::optional<vertex> vertex_of(std::vector<chain_sample> const &Samples, found_curve const &Curve, std::pair<double, double> const &Before, std::pair<double, double> const &After)
{
	auto const [a, in]{tangent_of(Samples, Before.first, Before.second, true)};
	auto const [b, out]{tangent_of(Samples, After.first, After.second, false)};
	double along{0.0};
	double back{0.0};
	if (false == geometry::lines_cross(a, in, b, out, along, back))
		return std::nullopt;
	// the tangents crossing ahead of the one before the curve and behind the one after it, no farther off than the stretch of the chain
	// they're read over, turning the way the curve does: else they tell nothing of a curve (it turns the long way, they run alike)
	auto const reach{After.second - Before.first};
	if (along < -kReach || back > kReach || along > reach || -back > reach || geometry::cross(in, out) * Curve.deflection < 0.0)
		return std::nullopt;
	vertex result;
	result.point = a + in * along;
	result.radius = Curve.radius;
	result.transition_in = Curve.transition_in;
	result.transition_out = Curve.transition_out;
	return result;
}

// the stretches of the chain the tangents of the curve are read from: the straights between it and the curves next to it
std::pair<std::pair<double, double>, std::pair<double, double>> straights_around(std::vector<found_curve> const &Curves, std::size_t const K, double const Total)
{
	auto const &curve{Curves[K]};
	auto const low{K == 0 ? 0.0 : Curves[K - 1].to + kReach};
	auto const high{K + 1 == Curves.size() ? Total : Curves[K + 1].from - kReach};
	auto const from{std::max(0.0, curve.from - kReach)};
	auto const to{std::min(Total, curve.to + kReach)};
	return {{std::min(low, from), from}, {to, std::max(high, to)}};
}

// the vertices of the curves found, each one from the straights around it; a held end puts the first or the last one on its heading
std::vector<vertex> initial_vertices(std::vector<chain_sample> const &Samples, std::vector<found_curve> &Curves, recognition_options const &Options)
{
	std::vector<vertex> result;
	std::vector<found_curve> kept;
	auto const total{Samples.back().s};
	for (std::size_t k = 0; k < Curves.size(); ++k)
	{
		auto const [before, after]{straights_around(Curves, k, total)};
		auto const made{vertex_of(Samples, Curves[k], before, after)};
		if (false == made.has_value())
			continue;
		result.push_back(*made);
		kept.push_back(Curves[k]);
	}
	Curves = std::move(kept);
	if (result.empty())
		return result;
	auto const start{geometry::plan_of(Samples.front().point)};
	auto const end{geometry::plan_of(Samples.back().point)};
	if (Options.start.held)
		result.front().point = start + Options.start.heading * std::max(0.1, glm::dot(result.front().point - start, Options.start.heading));
	if (Options.end.held && (result.size() > 1 || false == Options.start.held))
		result.back().point = end - Options.end.heading * std::max(0.1, glm::dot(end - result.back().point, Options.end.heading));
	return result;
}

// the cant of each vertex fitted to the rolls along its curve: held over the arc, ramped over the transitions
void fit_cant(solved_line const &Line, std::vector<chain_sample> const &Samples, std::vector<vertex> &Vertices, ramp_shape const Ramp, std::vector<std::string> &Findings)
{
	auto const count{Vertices.size()};
	std::vector<double> weighted(count, 0.0);
	std::vector<double> squares(count, 0.0);
	std::vector<double> sides(count, 1.0);
	auto const scale{Samples.back().s > 0.0 ? Line.length / Samples.back().s : 1.0};
	for (auto const &sample : Samples)
	{
		auto const s{project(Line, geometry::plan_of(sample.point), sample.s * scale)};
		auto const &e{Line.elements[Line.element_at(s)]};
		if (e.role == element_role::straight || e.vertex == 0 || e.vertex > count)
			continue;
		auto const k{e.vertex - 1};
		auto const t{e.length > 0.0 ? std::clamp((s - e.chainage) / e.length, 0.0, 1.0) : 1.0};
		double weight{1.0};
		if (e.role == element_role::transition_in)
			weight = Ramp == ramp_shape::bloss ? bloss_share(t) : t;
		else if (e.role == element_role::transition_out)
			weight = Ramp == ramp_shape::bloss ? bloss_share(1.0 - t) : 1.0 - t;
		sides[k] = sign_of(e.curvature(e.length * 0.5));
		weighted[k] += weight * sample.cant;
		squares[k] += weight * weight;
	}
	for (std::size_t k = 0; k < count; ++k)
	{
		auto &v{Vertices[k]};
		auto const cant{squares[k] > 0.0 ? sides[k] * weighted[k] / squares[k] : 0.0};
		v.cant = std::max(0.0, std::round(cant * 10.0) / 10.0);
		if (v.cant > kCantStep && (v.transition_in <= 0.0 || v.transition_out <= 0.0))
		{
			Findings.push_back(std::format("the curve of R {:.0f} m has a cant of {:.0f} mm and no transition curve to ramp it along: laid with none", v.radius, v.cant));
			v.cant = 0.0;
		}
		else if (v.cant <= kCantStep)
			v.cant = 0.0;
	}
}

// the breaks of the grade line: the heights of the chain as straight grades within half the tolerance (Douglas and Peucker), the
// heights of the breaks by least squares, each vertical curve the largest of the usual radii which keeps within it
std::vector<grade_break> fit_grades(std::vector<std::pair<double, double>> const &Heights, double const Length, double const Tolerance)
{
	std::vector<grade_break> result;
	auto const count{Heights.size()};
	if (count < 3)
		return result;
	std::vector<bool> kept(count, false);
	kept.front() = true;
	kept.back() = true;
	std::vector<std::pair<std::size_t, std::size_t>> open{{0, count - 1}};
	while (false == open.empty())
	{
		auto const [a, b]{open.back()};
		open.pop_back();
		auto const &pa{Heights[a]};
		auto const &pb{Heights[b]};
		double worst{0.0};
		std::size_t at{a};
		for (auto i = a + 1; i < b; ++i)
		{
			auto const t{pb.first > pa.first ? (Heights[i].first - pa.first) / (pb.first - pa.first) : 0.0};
			auto const off{std::abs(Heights[i].second - std::lerp(pa.second, pb.second, t))};
			if (off > worst)
			{
				worst = off;
				at = i;
			}
		}
		if (worst > Tolerance * 0.5 && at > a && at < b)
		{
			kept[at] = true;
			open.emplace_back(a, at);
			open.emplace_back(at, b);
		}
	}
	// the knots: the ends at the ends of the line, the breaks between them no closer than a metre to each other or to the ends
	std::vector<double> knots{0.0};
	for (std::size_t i = 1; i + 1 < count; ++i)
		if (kept[i] && Heights[i].first > knots.back() + 1.0 && Heights[i].first < Length - 1.0)
			knots.push_back(Heights[i].first);
	knots.push_back(Length);
	auto const inner{knots.size() - 2};
	if (inner == 0)
		return result;
	// the heights of the inner knots by least squares, the ends as the nodes have them
	auto const first_height{Heights.front().second};
	auto const last_height{Heights.back().second};
	std::vector<double> normal(inner * inner, 0.0);
	std::vector<double> right(inner, 0.0);
	for (auto const &[s, y] : Heights)
	{
		auto const next{std::upper_bound(knots.begin(), knots.end(), s)};
		auto const j{std::clamp<std::size_t>(static_cast<std::size_t>(next - knots.begin()), 1, knots.size() - 1)};
		auto const t{(std::clamp(s, knots[j - 1], knots[j]) - knots[j - 1]) / (knots[j] - knots[j - 1])};
		// y = (1 - t) h[j - 1] + t h[j]; the knots 0 and the last fixed
		std::array<std::pair<std::size_t, double>, 2> const terms{std::pair{j - 1, 1.0 - t}, std::pair{j, t}};
		auto value{y};
		for (auto const &[knot, weight] : terms)
		{
			if (knot == 0)
				value -= weight * first_height;
			else if (knot == knots.size() - 1)
				value -= weight * last_height;
		}
		for (auto const &[a, wa] : terms)
		{
			if (a == 0 || a == knots.size() - 1)
				continue;
			right[a - 1] += wa * value;
			for (auto const &[b, wb] : terms)
				if (b != 0 && b != knots.size() - 1)
					normal[(a - 1) * inner + (b - 1)] += wa * wb;
		}
	}
	for (std::size_t i = 0; i < inner; ++i)
		normal[i * inner + i] += 1e-9;
	if (false == solve_system(normal, right, inner))
		return result;
	std::vector<double> elevation{first_height};
	elevation.insert(elevation.end(), right.begin(), right.end());
	elevation.push_back(last_height);
	// the vertical curves: the largest radius of these which keeps the curve within half the tolerance of the heights and clear of the
	// curves of the breaks next to it
	static constexpr std::array<double, 24> radii{20000.0, 15000.0, 10000.0, 8000.0, 6000.0, 5000.0, 4000.0, 3000.0, 2500.0, 2000.0, 1500.0, 1000.0,
	                                              800.0,   600.0,   500.0,   400.0,  300.0,  200.0,  100.0,  50.0,   20.0,   10.0,   5.0,    2.0};
	std::vector<double> tangents(knots.size(), 0.0);
	for (std::size_t j = 1; j + 1 < knots.size(); ++j)
	{
		auto const before{(elevation[j] - elevation[j - 1]) / (knots[j] - knots[j - 1])};
		auto const after{(elevation[j + 1] - elevation[j]) / (knots[j + 1] - knots[j])};
		auto const change{std::abs(after - before)};
		double chosen{radii.back()};
		for (auto const radius : radii)
		{
			auto const tangent{radius * change * 0.5};
			if (tangent + tangents[j - 1] > knots[j] - knots[j - 1] - 1e-6 || tangent > (knots[j + 1] - knots[j]) * 0.5)
				continue;
			double worst{0.0};
			for (auto const &[s, y] : Heights)
			{
				auto const distance{std::abs(s - knots[j])};
				if (distance >= tangent)
					continue;
				auto const line{elevation[j] + (s < knots[j] ? before : after) * (s - knots[j])};
				auto const offset{(tangent - distance) * (tangent - distance) / (2.0 * radius)};
				worst = std::max(worst, std::abs(y - (line + (after > before ? offset : -offset))));
			}
			if (worst <= Tolerance * 0.5)
			{
				chosen = radius;
				break;
			}
		}
		tangents[j] = chosen * change * 0.5;
		result.push_back({knots[j], elevation[j], chosen});
	}
	return result;
}

// where the line keeps farthest off the chain between its curves, as far as the tolerance or more: the stretch of the chain a curve is
// missing in, none where it keeps within
std::optional<found_curve> missing_curve(solved_line const &Line, std::vector<chain_sample> const &Samples, std::vector<found_curve> const &Curves, double const Tolerance)
{
	auto const scale{Samples.back().s > 0.0 ? Line.length / Samples.back().s : 1.0};
	double worst{Tolerance};
	std::optional<std::size_t> at;
	for (std::size_t i = 0; i < Samples.size(); ++i)
	{
		auto const s{Samples[i].s};
		if (std::ranges::any_of(Curves, [&](found_curve const &Curve) { return s > Curve.from - 10.0 && s < Curve.to + 10.0; }))
			continue;
		auto const plan{geometry::plan_of(Samples[i].point)};
		auto const off{glm::distance(plan, Line.position(project(Line, plan, s * scale)))};
		if (off > worst)
		{
			worst = off;
			at = i;
		}
	}
	if (false == at.has_value())
		return std::nullopt;
	// the stretch between the curves around it
	auto const s{Samples[*at].s};
	double from{0.0};
	double to{Samples.back().s};
	for (auto const &curve : Curves)
	{
		if (curve.to <= s)
			from = std::max(from, curve.to + kReach);
		if (curve.from >= s)
			to = std::min(to, curve.from - kReach);
	}
	if (to - from < 4.0 * kReach)
		return std::nullopt;
	auto const turned{headings(Samples)};
	found_curve result;
	// an arc over the stretch between the curves around it, turning as the chain turns over it; its vertex where the headings at the ends
	// of the stretch cross
	result.from = from;
	result.to = to;
	result.deflection = at_length(Samples, turned, to) - at_length(Samples, turned, from);
	if (std::abs(result.deflection) < 1e-7)
		return std::nullopt;
	result.radius = std::max(100.0, (to - from) / std::abs(result.deflection));
	return result;
}

// the plan fitted from the curves found: their vertices fitted by least squares, the curves the curvature didn't show (a radius of tens of
// kilometres, a turn spread over the joints) added where the line keeps off the points
struct fitted_plan
{
	std::vector<vertex> vertices;
	std::vector<found_curve> spans;
	solved_line line;
	double off{std::numeric_limits<double>::max()}; // the farthest a point lies off the line
};

fitted_plan fit_plan(std::vector<chain_sample> const &Samples, std::vector<found_curve> Curves, recognition_options const &Options)
{
	plan_fit fit{Samples, Options};
	auto vertices{initial_vertices(Samples, Curves, Options)};
	fit.start_from(std::move(vertices), Curves);
	fit.fit(60);
	auto line{fit.lay(fit.vertices())};
	for (int round = 0; round < kRefits && line.valid(); ++round)
	{
		auto missing{missing_curve(line, Samples, fit.spans(), Options.tolerance.plan)};
		if (false == missing.has_value())
			break;
		auto spans{fit.spans()};
		auto more{fit.vertices()};
		auto const at{static_cast<std::size_t>(std::ranges::upper_bound(spans, missing->from, {}, &found_curve::from) - spans.begin())};
		auto const made{vertex_of(Samples, *missing, {missing->from, missing->from}, {missing->to, missing->to})};
		if (false == made.has_value())
			break;
		spans.insert(spans.begin() + static_cast<std::ptrdiff_t>(at), *missing);
		more.insert(more.begin() + static_cast<std::ptrdiff_t>(at), *made);
		plan_fit refit{Samples, Options};
		refit.start_from(more, spans);
		refit.fit(60);
		auto const relaid{refit.lay(refit.vertices())};
		if (false == relaid.valid() || measure(relaid, Samples).plan >= measure(line, Samples).plan)
			break;
		fit.start_from(refit.vertices(), refit.spans());
		line = relaid;
	}
	fitted_plan result{fit.vertices(), fit.spans(), line};
	if (line.valid())
		result.off = measure(line, Samples).plan;
	return result;
}

// the samples the curves are fitted to and those the deviation is measured at
struct chain_samples
{
	std::vector<chain_sample> const &fit;
	std::vector<chain_sample> const &measure;
};

// the line of the vertices of the plan fitted: the cant fitted to the rolls, the grade line to the heights along it, the line laid, the
// deviation measured, the joints of the paths along it
void complete(recognized_line &Result, std::vector<vertex> Vertices, chain_samples const &Samples, std::vector<chain_path> const &Chain, recognition_options const &Options)
{
	auto const &samples{Samples.fit};
	for (std::size_t k = 0; k < Vertices.size(); ++k)
	{
		Vertices[k].id = static_cast<vertex_id>(k + 1);
		Vertices[k].cant = 0.0;
	}
	end_condition start;
	start.position = Result.start;
	end_condition end;
	end.position = Result.end;
	Result.shape.vertices = Vertices;
	Result.shape.transition = Options.transition;
	auto const line{solve(Result.shape, start, end)};
	if (false == line.valid())
	{
		Result.error = line.errors.empty() ? std::string{"the curves found can't be laid"} : line.errors.front();
		return;
	}
	fit_cant(line, samples, Result.shape.vertices, Result.shape.ramp, Result.findings);
	std::vector<std::pair<double, double>> heights;
	auto const scale{samples.back().s > 0.0 ? line.length / samples.back().s : 1.0};
	for (auto const &sample : samples)
		heights.emplace_back(project(line, geometry::plan_of(sample.point), sample.s * scale), sample.point.y);
	heights.front().first = 0.0;
	heights.back().first = line.length;
	Result.shape.grades = fit_grades(heights, line.length, Options.tolerance.height);
	Result.solved = solve(Result.shape, start, end);
	if (false == Result.solved.valid() && false == Result.shape.grades.empty())
	{
		Result.findings.emplace_back("the grade line read off the heights can't be laid: an even grade between the ends");
		Result.shape.grades.clear();
		Result.solved = solve(Result.shape, start, end);
	}
	if (false == Result.solved.valid())
	{
		Result.error = Result.solved.errors.empty() ? std::string{"the line can't be laid"} : Result.solved.errors.front();
		return;
	}
	Result.measured = measure(Result.solved, Samples.measure);
	// the joints between the paths along the line
	auto const lengths{part_lengths(Chain)};
	double along{0.0};
	Result.joints = {0.0};
	for (std::size_t p = 0; p + 1 < lengths.size(); ++p)
	{
		along += lengths[p];
		auto const &path{Chain[p].path};
		auto const joint{path.points[Chain[p].forward ? segment_data::point::end : segment_data::point::start]};
		Result.joints.push_back(project(Result.solved, geometry::plan_of(joint), along * Result.solved.length / samples.back().s));
	}
	Result.joints.push_back(Result.solved.length);
}

// a curve of the line whose rolls ramp a cant up with no transition curve to ramp it along: the lengths of the ramps of the rolls before its
// arc and after it, where it has no transition
struct cant_ramp
{
	std::size_t vertex{0};
	double in{0.0};
	double out{0.0};
};

// the length the rolls take to ramp from a tenth of the cant to nine tenths of it, from the end of the arc (At) away from it
double ramp_length(std::vector<chain_sample> const &Samples, double const Scale, double const At, double const Cant, bool const Before)
{
	auto i{sample_at(Samples, At / Scale)};
	auto const step = [Before](std::size_t const Index) { return Before ? Index - 1 : Index + 1; };
	auto const inside = [Before, &Samples](std::size_t const Index) { return Before ? Index > 0 : Index + 1 < Samples.size(); };
	while (inside(i) && std::abs(Samples[i].cant) > 0.9 * Cant)
		i = step(i);
	auto const high{Samples[i].s};
	while (inside(i) && std::abs(Samples[i].cant) > 0.1 * Cant)
		i = step(i);
	return std::max(1.0, std::abs(Samples[i].s - high) * Scale / 0.8);
}

std::vector<cant_ramp> cant_ramps(solved_line const &Line, std::vector<chain_sample> const &Samples, std::vector<vertex> const &Vertices, double const Tolerance)
{
	std::vector<cant_ramp> result;
	auto const scale{Samples.back().s > 0.0 ? Line.length / Samples.back().s : 1.0};
	for (std::size_t k = 0; k < Vertices.size(); ++k)
	{
		auto const &v{Vertices[k]};
		if (v.transition_in > 0.0 && v.transition_out > 0.0)
			continue;
		auto const arc{std::ranges::find_if(Line.elements, [k](element const &E) { return E.vertex == k + 1 && E.role == element_role::arc; })};
		if (arc == Line.elements.end())
			continue;
		// the cant of the rolls along the arc
		double sum{0.0};
		double count{0.0};
		for (auto const &sample : Samples)
			if (sample.s * scale >= arc->chainage && sample.s * scale <= arc->end_chainage())
			{
				sum += std::abs(sample.cant);
				count += 1.0;
			}
		auto const cant{count > 0.0 ? sum / count : 0.0};
		if (cant <= Tolerance)
			continue;
		cant_ramp ramp;
		ramp.vertex = k;
		ramp.in = v.transition_in > 0.0 ? 0.0 : ramp_length(Samples, scale, arc->chainage, cant, true);
		ramp.out = v.transition_out > 0.0 ? 0.0 : ramp_length(Samples, scale, arc->end_chainage(), cant, false);
		result.push_back(ramp);
	}
	return result;
}

} // namespace

double cant_of(double const Roll)
{
	return kRailBase * std::sin(glm::radians(Roll));
}

std::vector<double> part_lengths(std::vector<chain_path> const &Chain)
{
	std::vector<double> result;
	for (auto const &part : Chain)
		result.push_back(path_curve{part}.length());
	return result;
}

std::vector<chain_sample> sample_chain(std::vector<chain_path> const &Chain, double const Step)
{
	std::vector<chain_sample> result;
	double start{0.0};
	glm::dvec2 leaving{0.0};
	for (std::size_t p = 0; p < Chain.size(); ++p)
	{
		path_curve const curve{Chain[p]};
		auto const length{curve.length()};
		auto const count{std::max(1, static_cast<int>(std::ceil(length / std::max(Step, 0.01) - 1e-9)))};
		for (int k = 0; k < count; ++k)
			result.push_back(curve.sample(length * k / count, p, start));
		auto const last{curve.sample(length, p, start)};
		if (p > 0)
			result[result.size() - static_cast<std::size_t>(count)].joint_turn = geometry::signed_angle(leaving, result[result.size() - static_cast<std::size_t>(count)].heading);
		leaving = last.heading;
		if (p + 1 == Chain.size())
			result.push_back(last);
		start += length;
	}
	return result;
}

double measured_deviation::share(takeover_tolerance const &Tolerance) const
{
	return std::max({plan / Tolerance.plan, height / Tolerance.height, cant / Tolerance.cant});
}

void measured_deviation::add(measured_deviation const &Other)
{
	auto const total{samples + Other.samples};
	auto const mean = [this, &Other, total](double const A, double const B) { return total > 0 ? std::sqrt((A * A * samples + B * B * Other.samples) / total) : 0.0; };
	plan_rms = mean(plan_rms, Other.plan_rms);
	height_rms = mean(height_rms, Other.height_rms);
	cant_rms = mean(cant_rms, Other.cant_rms);
	if (Other.plan > plan)
	{
		plan = Other.plan;
		plan_at = Other.plan_at;
	}
	if (Other.height > height)
	{
		height = Other.height;
		height_at = Other.height_at;
	}
	if (Other.cant > cant)
	{
		cant = Other.cant;
		cant_at = Other.cant_at;
	}
	samples = total;
}

measured_deviation measure(solved_line const &Line, std::vector<chain_sample> const &Samples)
{
	measured_deviation result;
	if (false == Line.valid() || Samples.empty())
		return result;
	auto const scale{Samples.back().s > 0.0 ? Line.length / Samples.back().s : 1.0};
	double plan{0.0};
	double height{0.0};
	double cant{0.0};
	for (auto const &sample : Samples)
	{
		auto const at{geometry::plan_of(sample.point)};
		auto const s{project(Line, at, sample.s * scale)};
		auto const off{glm::distance(at, Line.position(s))};
		auto const up{std::abs(sample.point.y - Line.elevation(s))};
		auto const roll{std::abs(sample.cant - Line.cant_at(s))};
		plan += off * off;
		height += up * up;
		cant += roll * roll;
		if (off > result.plan)
		{
			result.plan = off;
			result.plan_at = sample.point;
		}
		if (up > result.height)
		{
			result.height = up;
			result.height_at = sample.point;
		}
		if (roll > result.cant)
		{
			result.cant = roll;
			result.cant_at = sample.point;
		}
	}
	auto const count{static_cast<double>(Samples.size())};
	result.samples = Samples.size();
	result.plan_rms = std::sqrt(plan / count);
	result.height_rms = std::sqrt(height / count);
	result.cant_rms = std::sqrt(cant / count);
	return result;
}

std::vector<found_curve> find_curves(std::vector<chain_sample> const &Samples)
{
	std::vector<found_curve> result;
	if (Samples.size() < 2)
		return result;
	auto const turned{headings(Samples)};
	auto const curvature{read_curvature(Samples, turned, kWindow * 0.5)};
	auto const total{Samples.back().s};
	// the runs of the curvature of one sign, past what the noise of the paths turns them by
	std::vector<std::pair<std::size_t, std::size_t>> runs;
	std::size_t i{0};
	while (i < Samples.size())
	{
		if (std::abs(curvature[i]) <= kCurved)
		{
			++i;
			continue;
		}
		auto const side{sign_of(curvature[i])};
		auto const first{i};
		while (i < Samples.size() && std::abs(curvature[i]) > kCurved && sign_of(curvature[i]) == side)
			++i;
		split_run(curvature, {first, i - 1}, runs);
	}
	for (std::size_t r = 0; r < runs.size(); ++r)
	{
		auto const [first, last]{runs[r]};
		auto const lower{r == 0 ? 0.0 : 0.5 * (Samples[runs[r - 1].second].s + Samples[first].s)};
		auto const upper{r + 1 == runs.size() ? total : 0.5 * (Samples[last].s + Samples[runs[r + 1].first].s)};
		if (auto curve{read_curve(Samples, turned, curvature, {first, last}, {lower, upper})})
			split_turn(Samples, turned, *curve, result);
	}
	// the turns at the joints with no curve around them: kinks
	for (auto const &sample : Samples)
	{
		if (std::abs(sample.joint_turn) <= kKink || std::ranges::any_of(result, [&](found_curve const &Curve) { return sample.s >= Curve.from && sample.s <= Curve.to; }))
			continue;
		found_curve kink;
		kink.from = std::max(0.0, sample.s - 1.0);
		kink.to = std::min(total, sample.s + 1.0);
		kink.deflection = sample.joint_turn;
		kink.radius = std::max(1.0, 2.0 / std::abs(sample.joint_turn));
		kink.kink = true;
		result.insert(std::ranges::upper_bound(result, kink.from, {}, &found_curve::from), kink);
	}
	return result;
}

recognized_line recognize(std::vector<chain_path> const &Chain, recognition_options const &Options)
{
	recognized_line result;
	if (Chain.empty())
	{
		result.error = "no paths";
		return result;
	}
	auto const samples{sample_chain(Chain, Options.step)};
	auto const measured{sample_chain(Chain, Options.measure_step)};
	result.start = samples.front().point;
	result.end = samples.back().point;
	if (samples.back().s < 0.2)
	{
		result.error = "the paths have no length";
		return result;
	}
	auto curves{find_curves(samples)};
	for (auto const &curve : curves)
	{
		auto const where{std::format("{:.1f} m along", curve.from)};
		if (curve.kink)
			result.findings.push_back(std::format("the paths turn by {:.3f} degree at a joint {}, with no curve", glm::degrees(std::abs(curve.deflection)), where));
		if (curve.compound)
			result.findings.push_back(std::format("a compound curve {}: arcs of more than one radius laid as one", where));
	}
	auto plan{fit_plan(samples, curves, Options)};
	if (plan.off > Options.tolerance.plan)
	{
		// the small turns read off old tracks laid jagged, joint by joint: the plan fitted again with the curves which turn by more, kept
		// where it keeps nearer
		auto significant{curves};
		std::erase_if(significant, [](found_curve const &Curve) { return false == Curve.kink && std::abs(Curve.deflection) < kSmallTurn && Curve.to - Curve.from < 2.0 * kWindow; });
		if (significant.size() < curves.size())
		{
			auto other{fit_plan(samples, significant, Options)};
			if (other.off < plan.off)
				plan = std::move(other);
		}
	}
	// curves found which lead the least squares astray (a turn read twice, the tangents of a jagged stretch): the plan of the curves added where
	// the line keeps off the points alone, kept where it keeps nearer
	if (plan.off > kAstray)
	{
		auto bare{fit_plan(samples, {}, Options)};
		if (bare.off < plan.off)
			plan = std::move(bare);
	}
	auto const &line{plan.line};
	if (false == line.valid())
	{
		result.error = line.errors.empty() ? std::string{"the curves found can't be laid"} : line.errors.front();
		return result;
	}
	chain_samples const both{samples, measured};
	auto const found{result.findings};
	complete(result, plan.vertices, both, Chain, Options);
	// the curves whose rolls ramp a cant up with no transition curve under it: fitted again with transitions as long as the ramps, kept
	// where the line keeps nearer the old tracks so (the arc moved in by them, the cant ramped where the rolls ramp it)
	auto const ramps{cant_ramps(line, samples, plan.vertices, Options.tolerance.cant)};
	if (result.valid() && false == ramps.empty())
	{
		plan_fit again{samples, Options};
		again.start_from(plan.vertices, plan.spans);
		for (auto const &[k, in, out] : ramps)
			again.hold_transitions(k, in, out);
		again.fit(60);
		recognized_line ramped;
		ramped.start = result.start;
		ramped.end = result.end;
		ramped.findings = found;
		complete(ramped, again.vertices(), both, Chain, Options);
		if (ramped.valid() && ramped.measured.share(Options.tolerance) < result.measured.share(Options.tolerance))
			result = std::move(ramped);
	}
	return result;
}

} // namespace layout
