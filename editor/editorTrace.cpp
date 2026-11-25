/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorTrace.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <utility>

namespace trace
{

namespace
{

constexpr double kPi{3.14159265358979323846};
constexpr double kGravity{9.81};
// a point further from the line than this many of its sigmas doesn't keep to it
constexpr double kWithin{3.0};
// what a parameter more has to bring down the sum of the squares of the departures in sigmas, for a shape of more of them to be taken
constexpr double kPenalty{9.0};
// the transitions keep to the ones of the norms unless the points show otherwise: a departure of this many metres weighs as one sigma
constexpr double kTransitionScale{20.0};
constexpr double kRadiusLeast{15.0};
constexpr double kRadiusMost{50000.0};
constexpr double kShortestArc{10.0};
constexpr double kNearlyStraight{5000.0};
// a gentler curve isn't one: a straight the noise of the points bent
constexpr double kRadiusGentlest{30000.0};
// the transitions of a fit this close to the ones of the norms are the ones of the norms
constexpr double kTransitionSnap{5.0};
// the arcs of a compound curve differ this many times at most: a flatter one is a straight between two curves
constexpr double kCompoundSpread{2.5};
// a point further off than this many of its sigmas, a click gone astray, pulls the line less than the square of its departure would
constexpr double kRobust{kWithin};

double sign_of(double const Value)
{
	return Value < 0.0 ? -1.0 : 1.0;
}

double cross(glm::dvec2 const &A, glm::dvec2 const &B)
{
	return A.x * B.y - A.y * B.x;
}

glm::dvec2 unit(double const Angle)
{
	return {std::cos(Angle), std::sin(Angle)};
}

glm::dvec2 left_of(double const Angle)
{
	return {-std::sin(Angle), std::cos(Angle)};
}

double angle_of(glm::dvec2 const &Direction)
{
	return std::atan2(Direction.y, Direction.x);
}

double wrapped(double const Angle)
{
	return std::remainder(Angle, 2.0 * kPi);
}

// --- the norms, not rounded: the fits need them smooth ---

double allowed_speed(double const Speed, double const Radius, alignment::limits const &Norms)
{
	return std::max(1.0, std::min(Speed, alignment::speed_limit(Radius, Norms)));
}

// mm the norms ask for in an arc of the radius at the speed, as alignment::recommend gives it, not rounded up
double norm_cant(double const Speed, double const Radius, alignment::limits const &Norms)
{
	if (Radius <= 0.0)
		return 0.0;
	auto const factor{Norms.gauge / (kGravity * 12.96)};
	auto const allowance{Norms.gauge * Norms.unbalanced / kGravity};
	auto const equilibrium{factor * Speed * Speed / Radius};
	return std::min(Norms.cant_max, std::max(std::max(0.0, equilibrium - allowance), equilibrium * 2.0 / 3.0));
}

// m of the transition from the curvature and the cant of one arc to those of the next one, as alignment::transition_between gives it,
// not rounded up; the curvatures not signed, the same side
double norm_between(double const Speed, double const Curvature0, double const Cant0, double const Curvature1, double const Cant1, alignment::limits const &Norms)
{
	auto const cant{std::abs(Cant1 - Cant0)};
	double length{0.0};
	if (cant > 0.0)
		length = std::max(Speed * cant / (3.6 * Norms.cant_rate), cant / 1000.0 * Norms.ramp_factor * Speed);
	auto const unbalanced = [&](double const Curvature, double const Cant) { return Speed * Speed / 12.96 * Curvature - kGravity * Cant / Norms.gauge; };
	return std::max(length, std::abs(unbalanced(Curvature1, Cant1) - unbalanced(Curvature0, Cant0)) * Speed / (3.6 * Norms.jerk));
}

// the cant the transitions of the length ramp at the speed, of the one the norms ask for at most, in steps of 5 mm
double ramped_cant(double const Cant, double const Length, double const Speed, alignment::limits const &Norms)
{
	if (Length <= 0.0)
		return 0.0;
	auto const speed{std::max(1.0, Speed)};
	auto const ramp{std::min({Cant, Length * 1000.0 / (Norms.ramp_factor * speed), Norms.cant_rate * 3.6 * Length / speed})};
	return ramp < 5.0 ? 0.0 : std::floor(ramp / 5.0) * 5.0;
}

// --- the line as a chain of segments: the curvature of each changing evenly from its start to its end ---

struct segment
{
	double from{0.0}; // 1/m, positive turning left
	double to{0.0};
	double length{0.0};
};

struct pose
{
	glm::dvec2 position{0.0};
	double heading{0.0};
};

struct local
{
	double x{0.0};
	double y{0.0};
	double angle{0.0};
};

// the point Distance along the segment in the frame of its start; an arc or a straight carried on back for a Distance below 0
local along(segment const &Segment, double const Distance)
{
	auto const change{Segment.length > 1e-12 ? (Segment.to - Segment.from) / Segment.length : 0.0};
	if (std::abs(Segment.to - Segment.from) < 1e-15)
	{
		auto const k{Segment.from};
		auto const angle{k * Distance};
		if (std::abs(angle) < 1e-7)
			return {Distance - k * k * Distance * Distance * Distance / 6.0, 0.5 * k * Distance * Distance, angle};
		return {std::sin(angle) / k, (1.0 - std::cos(angle)) / k, angle};
	}
	// Gauss-Legendre over pieces turning 0.3 rad at most: of 4 points where they turn 0.1 rad at most, within a micrometre, else of 8
	static constexpr std::array<double, 4> nodes{0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363};
	static constexpr std::array<double, 4> weights{0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763};
	static constexpr std::array<double, 2> nodes4{0.3399810435848563, 0.8611363115940526};
	static constexpr std::array<double, 2> weights4{0.6521451548625461, 0.3478548451374539};
	auto const heading = [&](double const S) { return Segment.from * S + 0.5 * change * S * S; };
	auto const distance{std::max(0.0, Distance)};
	auto const turn{std::max(std::abs(Segment.from), std::abs(Segment.from + change * distance)) * distance};
	int const pieces{std::max(1, static_cast<int>(std::ceil(turn / 0.3)))};
	auto const h{distance / pieces};
	auto const gentle{turn < 0.1};
	double x{0.0};
	double y{0.0};
	for (int p = 0; p < pieces; ++p)
	{
		auto const middle{h * (p + 0.5)};
		auto const sum = [&](auto const &Nodes, auto const &Weights) {
			for (std::size_t i = 0; i < Nodes.size(); ++i)
				for (auto const side : {-1.0, 1.0})
				{
					auto const a{heading(middle + side * Nodes[i] * 0.5 * h)};
					x += Weights[i] * std::cos(a);
					y += Weights[i] * std::sin(a);
				}
		};
		if (gentle)
			sum(nodes4, weights4);
		else
			sum(nodes, weights);
	}
	return {x * 0.5 * h, y * 0.5 * h, heading(distance)};
}

pose moved(pose const &From, local const &By)
{
	auto const t{unit(From.heading)};
	auto const n{left_of(From.heading)};
	return {From.position + t * By.x + n * By.y, From.heading + By.angle};
}

double curvature_at(segment const &Segment, double const Distance)
{
	return Segment.length > 1e-12 ? Segment.from + (Segment.to - Segment.from) * std::clamp(Distance / Segment.length, 0.0, 1.0) : Segment.from;
}

// a segment set in the plan
struct placed
{
	segment shape;
	pose start;
	pose end;
	double chainage{0.0};
};

struct chain
{
	std::vector<placed> pieces;
	// the first piece carried on back before its start, the last one on beyond its end
	bool open_before{true};
	bool open_after{true};
};

chain chain_of(pose const &Start, std::vector<segment> const &Segments)
{
	chain line;
	line.pieces.reserve(Segments.size());
	auto at{Start};
	double chainage{0.0};
	for (auto const &each : Segments)
	{
		placed piece{each, at, at, chainage};
		piece.end = moved(at, along(each, each.length));
		at = piece.end;
		chainage += each.length;
		line.pieces.push_back(piece);
	}
	return line;
}

struct projection
{
	bool found{false};
	double offset{0.0}; // positive to the left
	std::size_t piece{0};
	double distance{0.0}; // along the piece
};

// the point set square onto the piece, where it lies abreast of it
projection onto(chain const &Line, std::size_t const Index, glm::dvec2 const &Point)
{
	projection result;
	auto const &piece{Line.pieces[Index]};
	auto const first{Index == 0 && Line.open_before};
	auto const last{Index + 1 == Line.pieces.size() && Line.open_after};
	auto const t0{unit(piece.start.heading)};
	auto const t1{unit(piece.end.heading)};
	if (false == first && glm::dot(Point - piece.start.position, t0) < 0.0)
		return result;
	if (false == last && glm::dot(Point - piece.end.position, t1) > 0.0)
		return result;
	auto const &shape{piece.shape};
	result.piece = Index;
	if (std::abs(shape.to - shape.from) < 1e-15)
	{
		auto const k{shape.from};
		if (std::abs(k) < 1e-12)
		{
			result.found = true;
			result.offset = cross(t0, Point - piece.start.position);
			result.distance = glm::dot(Point - piece.start.position, t0);
			return result;
		}
		auto const centre{piece.start.position + left_of(piece.start.heading) / k};
		auto const radial{Point - centre};
		auto const start{piece.start.position - centre};
		auto const turned{std::atan2(cross(start, radial), glm::dot(start, radial)) * sign_of(k)};
		// an arc carried on only half a turn either way
		auto const reach{std::abs(k) * shape.length};
		if (turned < (first ? -0.5 * kPi : -1e-9) || turned > (last ? reach + 0.5 * kPi : reach + 1e-9))
			return result;
		result.found = true;
		result.offset = sign_of(k) * (1.0 / std::abs(k) - glm::length(radial));
		result.distance = turned / std::abs(k);
		return result;
	}
	// a transition: Newton from the point abreast along the start
	auto s{std::clamp(glm::dot(Point - piece.start.position, t0), 0.0, shape.length)};
	auto on{moved(piece.start, along(shape, s))};
	for (int i = 0; i < 4; ++i)
	{
		auto const v{Point - on.position};
		auto const g{glm::dot(v, unit(on.heading))};
		auto const bend{1.0 - curvature_at(shape, s) * glm::dot(v, left_of(on.heading))};
		auto const next{std::clamp(s + g / std::max(0.2, bend), 0.0, shape.length)};
		if (std::abs(next - s) < 1e-5)
			break;
		s = next;
		on = moved(piece.start, along(shape, s));
	}
	result.found = true;
	result.offset = glm::dot(Point - on.position, left_of(on.heading));
	result.distance = s;
	return result;
}

// the point set square onto the line: the nearest of the pieces it lies abreast of
projection project(chain const &Line, glm::dvec2 const &Point)
{
	projection best;
	for (std::size_t i = 0; i < Line.pieces.size(); ++i)
		if (auto const here{onto(Line, i, Point)}; here.found && (false == best.found || std::abs(here.offset) < std::abs(best.offset)))
			best = here;
	if (false == best.found && false == Line.pieces.empty())
	{
		// past a joint where the pieces don't quite meet: the nearest joint
		auto nearest{std::numeric_limits<double>::max()};
		for (std::size_t i = 0; i < Line.pieces.size(); ++i)
			for (auto const *end : {&Line.pieces[i].start, &Line.pieces[i].end})
				if (auto const distance{glm::distance(Point, end->position)}; distance < nearest)
				{
					nearest = distance;
					best.found = true;
					best.piece = i;
					best.distance = end == &Line.pieces[i].start ? 0.0 : Line.pieces[i].shape.length;
					best.offset = cross(unit(end->heading), Point - end->position);
				}
	}
	return best;
}

// --- least squares within a budget ---

struct budget
{
	std::size_t left{0};
	std::size_t used{0};
	bool spent{false};
	bool take()
	{
		if (left == 0)
		{
			spent = true;
			return false;
		}
		--left;
		++used;
		return true;
	}
};

using residuals_of = std::function<bool(std::vector<double> const &, std::vector<double> &)>;

double squares(std::vector<double> const &Values)
{
	double sum{0.0};
	for (auto const value : Values)
		sum += value * value;
	return sum;
}

// Levenberg-Marquardt; none where the residuals fail where it starts or the budget runs out
std::optional<double> minimize(std::vector<double> &Parameters, residuals_of const &Residuals, budget &Budget, int const Iterations = 60)
{
	std::vector<double> residuals;
	auto const evaluate = [&](std::vector<double> const &At, std::vector<double> &Into) { return Budget.take() && Residuals(At, Into); };
	if (false == evaluate(Parameters, residuals))
		return std::nullopt;
	auto cost{squares(residuals)};
	auto const n{Parameters.size()};
	auto const m{residuals.size()};
	if (n == 0)
		return cost;
	double damping{1e-3};
	std::vector<double> trial;
	std::vector<double> jacobian(n * m, 0.0);
	for (int iteration = 0; iteration < Iterations && cost > 1e-14; ++iteration)
	{
		for (std::size_t j = 0; j < n; ++j)
		{
			auto parameters{Parameters};
			auto const step{1e-6 * std::max(1.0, std::abs(Parameters[j]))};
			parameters[j] += step;
			auto sign{1.0};
			if (false == evaluate(parameters, trial) || trial.size() != m)
			{
				if (Budget.spent)
					return std::nullopt;
				parameters[j] = Parameters[j] - step;
				sign = -1.0;
				if (false == evaluate(parameters, trial) || trial.size() != m)
				{
					if (Budget.spent)
						return std::nullopt;
					std::fill(jacobian.begin() + j * m, jacobian.begin() + (j + 1) * m, 0.0);
					continue;
				}
			}
			for (std::size_t i = 0; i < m; ++i)
				jacobian[j * m + i] = sign * (trial[i] - residuals[i]) / step;
		}
		std::vector<double> normal(n * n, 0.0);
		std::vector<double> gradient(n, 0.0);
		for (std::size_t j = 0; j < n; ++j)
		{
			for (std::size_t i = 0; i < m; ++i)
				gradient[j] += jacobian[j * m + i] * residuals[i];
			for (std::size_t k = j; k < n; ++k)
			{
				double sum{0.0};
				for (std::size_t i = 0; i < m; ++i)
					sum += jacobian[j * m + i] * jacobian[k * m + i];
				normal[j * n + k] = normal[k * n + j] = sum;
			}
		}
		auto better{false};
		auto settled{false};
		for (int attempt = 0; attempt < 10 && false == better; ++attempt)
		{
			auto system{normal};
			std::vector<double> step(n);
			for (std::size_t j = 0; j < n; ++j)
			{
				system[j * n + j] += damping * std::max(normal[j * n + j], 1e-9);
				step[j] = -gradient[j];
			}
			// Gauss with partial pivoting
			for (std::size_t c = 0; c < n; ++c)
			{
				auto pivot{c};
				for (std::size_t r = c + 1; r < n; ++r)
					if (std::abs(system[r * n + c]) > std::abs(system[pivot * n + c]))
						pivot = r;
				if (std::abs(system[pivot * n + c]) < 1e-300)
					break;
				for (std::size_t k = 0; k < n; ++k)
					std::swap(system[c * n + k], system[pivot * n + k]);
				std::swap(step[c], step[pivot]);
				for (std::size_t r = c + 1; r < n; ++r)
				{
					auto const factor{system[r * n + c] / system[c * n + c]};
					for (std::size_t k = c; k < n; ++k)
						system[r * n + k] -= factor * system[c * n + k];
					step[r] -= factor * step[c];
				}
			}
			for (std::size_t c = n; c-- > 0;)
			{
				for (std::size_t k = c + 1; k < n; ++k)
					step[c] -= system[c * n + k] * step[k];
				step[c] = std::abs(system[c * n + c]) > 1e-300 ? step[c] / system[c * n + c] : 0.0;
			}
			auto candidate{Parameters};
			for (std::size_t j = 0; j < n; ++j)
				candidate[j] += step[j];
			if (evaluate(candidate, trial) && trial.size() == m && squares(trial) < cost)
			{
				auto const lower{squares(trial)};
				// a thousandth of a sigma squared less is no better: the fit is as close as it gets
				settled = cost - lower < std::max(1e-6 * cost, 1e-3);
				Parameters = candidate;
				residuals = trial;
				cost = lower;
				damping = std::max(1e-9, damping * 0.3);
				better = true;
			}
			else
			{
				if (Budget.spent)
					return std::nullopt;
				damping *= 8.0;
			}
		}
		if (false == better || settled)
			break;
	}
	return cost;
}

// --- the shape of the open end of the line and its parameters ---

// a curve of the open end: where it begins, of how many radii, whether it ends on a straight
struct curve_form
{
	int side{1}; // 1 turning left, -1 right
	int radii{1};
	bool open{true}; // the line ends within its last arc
	bool inside{false}; // begins within its first arc, at the start of the line
	bool held{false}; // begins at the curvature the line is held to at its start: no straight before it, the transition in from it by the norms
	bool fixed{false}; // and its first arc of that curvature, with no transition (a switch)

	bool operator==(curve_form const &) const = default;
};

struct form
{
	bool free{false}; // the start of the line fitted: across the first point, and its heading
	bool slide{false}; // the last arc of the last curve frozen lengthened or shortened
	bool slide_out{false}; // and its transition out too: the straight after it moved across
	std::vector<curve_form> curves;

	bool operator==(form const &) const = default;
};

// a curve as the parameters make it
struct curve_made
{
	curve_form shape;
	std::vector<double> curvatures; // signed
	std::vector<double> arcs; // m; the last one of an open curve as far as the line goes
	std::vector<double> between; // m, the transitions between the arcs
	double lead{0.0}; // the straight before it
	double in{0.0};
	double out{0.0};
	double from{0.0}; // the curvature its transition in starts from
	double speed{0.0}; // km/h the norms are asked at in the fit
};

// the line the parameters make from where the open end begins
struct made
{
	bool valid{false};
	pose start; // of the chain of the fit
	std::vector<segment> segments; // the arc sliding and its transition out first, where the last curve frozen slides
	std::vector<curve_made> curves;
	std::vector<std::pair<std::size_t, std::size_t>> spans; // of the segments of each curve, the last one past it
	std::vector<double> priors; // residuals
	std::size_t first_tail{0}; // the first segment of the open end
};

// what the open end goes on from: the start of the line, held or fitted, or the end of the curve frozen last
struct anchor
{
	pose start; // held, or the one of the line as fitted last
	double curvature{0.0}; // held at the start
	double cant{0.0};
	bool fixed{false};
	bool held{false}; // the start of the line held to a free end
	// the last curve frozen sliding: the pose before its last arc, the arc and the transition out of it
	bool slides{false};
	pose before;
	segment arc;
	segment out;
	glm::dvec2 first{0.0}; // the first point of the line, where a start fitted is abreast of
};

struct model
{
	form shape;
	std::vector<double> parameters;
};

std::size_t parameter_count(form const &Shape)
{
	std::size_t count{(Shape.free ? 2u : 0u) + (Shape.slide ? 1u : 0u) + (Shape.slide_out ? 1u : 0u)};
	for (auto const &curve : Shape.curves)
	{
		if (false == curve.inside && false == curve.held)
			count += 2; // the straight before, the transition in
		for (int i = 0; i < curve.radii; ++i)
		{
			if (false == (curve.fixed && i == 0))
				++count; // the radius
			if (false == (curve.open && i + 1 == curve.radii))
				++count; // the length of the arc
		}
		if (false == curve.open)
			++count; // the transition out
	}
	return count;
}

double length_of(double const Parameter)
{
	return Parameter * Parameter;
}

double parameter_of(double const Length)
{
	return std::sqrt(std::max(0.0, Length));
}

double curvature_of(double const Parameter, int const Side)
{
	return Side / std::exp(std::clamp(Parameter, std::log(kRadiusLeast), std::log(kRadiusMost)));
}

double radius_parameter(double const Curvature)
{
	return std::log(std::clamp(1.0 / std::max(std::abs(Curvature), 1e-12), kRadiusLeast, kRadiusMost));
}

// the cants of the arcs in the fit: the norms' at the speed the sharpest allows, the others in proportion to the curvature
std::vector<double> fit_cants(std::vector<double> const &Curvatures, double const Speed, alignment::limits const &Norms)
{
	double sharpest{0.0};
	for (auto const k : Curvatures)
		sharpest = std::max(sharpest, std::abs(k));
	auto const radius{1.0 / std::max(sharpest, 1e-12)};
	auto const cant{norm_cant(Speed, radius, Norms)};
	std::vector<double> cants;
	for (auto const k : Curvatures)
		cants.push_back(cant * std::abs(k) / std::max(sharpest, 1e-12));
	return cants;
}

made make(anchor const &Anchor, form const &Shape, std::vector<double> const &X, settings const &Settings)
{
	made line;
	std::size_t at{0};
	auto const next = [&]() { return at < X.size() ? X[at++] : 0.0; };
	if (Shape.free)
	{
		auto const across{next()};
		auto const heading{next()};
		line.start = {Anchor.first + left_of(heading) * across, heading};
	}
	else if (Anchor.slides)
		line.start = Anchor.before;
	else
		line.start = Anchor.start;
	if (Anchor.slides)
	{
		auto arc{Anchor.arc};
		auto const slide{Shape.slide ? next() : 0.0};
		arc.length = Anchor.arc.length + slide;
		if (arc.length < 1.0)
			return line;
		line.segments.push_back(arc);
		if (Anchor.out.length > 0.0)
		{
			auto out{Anchor.out};
			if (Shape.slide_out)
			{
				out.length = length_of(next());
				line.priors.push_back((out.length - Anchor.out.length) / kTransitionScale);
			}
			line.segments.push_back(out);
		}
	}
	line.first_tail = line.segments.size();
	auto const &norms{Settings.norms};
	for (std::size_t c = 0; c < Shape.curves.size(); ++c)
	{
		auto const &form{Shape.curves[c]};
		curve_made curve;
		curve.shape = form;
		auto const first{line.segments.size()};
		if (false == form.inside && false == form.held)
		{
			curve.lead = length_of(next());
			curve.in = length_of(next());
		}
		for (int i = 0; i < form.radii; ++i)
		{
			if (form.fixed && i == 0)
				curve.curvatures.push_back(Anchor.curvature);
			else
				curve.curvatures.push_back(curvature_of(next(), form.side));
			curve.arcs.push_back(form.open && i + 1 == form.radii ? 0.0 : length_of(next()));
		}
		if (false == form.open)
			curve.out = length_of(next());
		// the curve turns one way: a held curvature the other way, or none, begins it by a transition
		double sharpest{0.0};
		for (auto const k : curve.curvatures)
			sharpest = std::max(sharpest, std::abs(k));
		curve.speed = allowed_speed(Settings.speed, 1.0 / std::max(sharpest, 1e-12), norms);
		auto const cants{fit_cants(curve.curvatures, curve.speed, norms)};
		for (std::size_t i = 1; i < curve.curvatures.size(); ++i)
			curve.between.push_back(norm_between(curve.speed, std::abs(curve.curvatures[i - 1]), cants[i - 1], std::abs(curve.curvatures[i]), cants[i], norms) + 2.5);
		if (form.held)
		{
			curve.from = Anchor.curvature;
			auto const same{Anchor.curvature * form.side > 0.0};
			curve.in = form.fixed ? 0.0 : norm_between(curve.speed, same ? std::abs(Anchor.curvature) : 0.0, same ? Anchor.cant : 0.0, std::abs(curve.curvatures.front()), cants.front(), norms);
			// a curvature changing by 5 % at most and the cant by less than 5 mm goes on with none, as the norms take it
			if (same && std::abs(std::abs(curve.curvatures.front()) - std::abs(Anchor.curvature)) <= 0.05 * std::abs(Anchor.curvature))
				curve.in = 0.0;
			else if (curve.in > 0.0)
				curve.in += 2.5;
		}
		// the straight before the curve, the transition into it
		if (curve.lead > 0.0)
			line.segments.push_back({0.0, 0.0, curve.lead});
		if (curve.in > 0.0)
			line.segments.push_back({curve.from, curve.curvatures.front(), curve.in});
		for (std::size_t i = 0; i < curve.curvatures.size(); ++i)
		{
			if (i > 0 && curve.between[i - 1] > 0.0)
				line.segments.push_back({curve.curvatures[i - 1], curve.curvatures[i], curve.between[i - 1]});
			line.segments.push_back({curve.curvatures[i], curve.curvatures[i], curve.arcs[i]});
		}
		if (false == form.open && curve.out > 0.0)
			line.segments.push_back({curve.curvatures.back(), 0.0, curve.out});
		line.spans.emplace_back(first, line.segments.size());
		// the transitions in and out keep to the ones of the norms where the points don't show otherwise
		if (false == form.inside && false == form.held)
			line.priors.push_back((curve.in - (norm_between(curve.speed, 0.0, 0.0, std::abs(curve.curvatures.front()), cants.front(), norms) + 2.5)) / kTransitionScale);
		if (false == form.open)
			line.priors.push_back((curve.out - (norm_between(curve.speed, std::abs(curve.curvatures.back()), cants.back(), 0.0, 0.0, norms) + 2.5)) / kTransitionScale);
		line.curves.push_back(std::move(curve));
	}
	// a straight after the last curve ending on one, or the whole open end a straight
	if (Shape.curves.empty() || false == Shape.curves.back().open)
		line.segments.push_back({0.0, 0.0, 0.0});
	// a curve turning round; a compound one of an arc this many times flatter than its sharpest one is a straight between two curves
	for (auto const &curve : line.curves)
	{
		double sharpest{0.0};
		auto flattest{std::numeric_limits<double>::max()};
		for (auto const k : curve.curvatures)
		{
			sharpest = std::max(sharpest, std::abs(k));
			flattest = std::min(flattest, std::abs(k));
		}
		if (curve.curvatures.size() > 1 && sharpest > kCompoundSpread * flattest)
			return line;
		// a radius run off past the gentlest curve a line is laid with: no curve the points show
		if (flattest < 1.0 / kRadiusGentlest)
			return line;
		double turn{0.0};
		for (std::size_t i = 0; i < curve.curvatures.size(); ++i)
			turn += std::abs(curve.curvatures[i]) * curve.arcs[i];
		if (turn > 0.95 * kPi)
			return line;
	}
	line.valid = true;
	return line;
}

// how far the points are from the line, in their sigmas, weighed robustly beyond kRobust
struct departures
{
	std::vector<double> offsets; // m
	std::vector<std::size_t> pieces;
	double cost{0.0};
	double worst{0.0}; // in sigmas
};

double robust(double const Normalized)
{
	auto const a{std::abs(Normalized)};
	return a <= kRobust ? Normalized : std::copysign(std::sqrt(2.0 * kRobust * a - kRobust * kRobust), Normalized);
}

// --- the state after each click ---

// what the norms make of a curve: its transitions, its cant and the speed it's laid for
struct settled_curve
{
	double in{0.0};
	double out{0.0};
	std::vector<double> between;
	double cant{0.0};
	double speed{0.0}; // km/h below the line's, else 0
	bool refused{false};
};

// a curve frozen, as it was fitted
struct frozen_curve
{
	curve_made curve;
	std::size_t first{0}; // of the segments of the frozen line, its straight before it first
	std::size_t last{0}; // past it
	std::optional<settled_curve> settled;
};

struct state
{
	// the start of the line: held at a free end, or fitted (then kept from when the first curve froze)
	pose start;
	bool started{false}; // the start known: held, or fitted from two points at least
	std::vector<segment> frozen;
	std::vector<frozen_curve> frozen_curves;
	// the open end
	anchor from;
	std::size_t tail{0}; // the first point of it
	std::vector<std::size_t> sliding; // the points of the curve frozen last which its slide moves
	std::size_t slide_segment{std::numeric_limits<std::size_t>::max()}; // of the frozen line
	model fit;
	double worst{0.0}; // in sigmas, of the points of the open end
	std::vector<char> misfit; // of all the points
	result line;
};

// the curve alone between long straights, or from the start or to the end of the line where it begins or ends within an arc: what the
// norms are asked of it with
alignment::design alone(alignment::design const &Design, alignment::vertex const &Vertex, glm::dvec2 const &In, glm::dvec2 const &Out, bool const StartsCurved, bool const EndsCurved)
{
	constexpr double kFar{50000.0};
	auto one{Design};
	one.vertices = {Vertex};
	if (false == StartsCurved)
	{
		auto const start{Vertex.position - In * kFar};
		one.start = {start.x, 0.0, start.y};
		one.start_direction = In;
		one.start_curved = false;
		one.start_radius = 0.0;
		one.start_turn = 0;
	}
	if (false == EndsCurved)
	{
		auto const end{Vertex.position + Out * kFar};
		one.end = {end.x, 0.0, end.y};
		one.end_direction = Out;
		one.end_curved = false;
		one.end_radius = 0.0;
		one.end_turn = 0;
	}
	one.start_reserve = one.end_reserve = 0.0;
	return one;
}

} // namespace

double click_sigma(double const MetresPerPixel)
{
	return std::max(0.05, 2.0 * std::abs(MetresPerPixel));
}

double curve::sharpest() const
{
	double least{std::numeric_limits<double>::max()};
	for (auto const radius : radii)
		least = std::min(least, radius);
	return radii.empty() ? 0.0 : least;
}

std::string radii_text(curve const &Curve)
{
	std::string text;
	auto const kilometres{Curve.sharpest() >= kNearlyStraight};
	for (auto const radius : Curve.radii)
	{
		char part[32];
		if (kilometres)
			std::snprintf(part, sizeof(part), radius >= 9950.0 ? "%.0f" : "%.1f", radius / 1000.0);
		else
			std::snprintf(part, sizeof(part), "%.0f", radius);
		text += (text.empty() ? "" : "/") + std::string{part};
	}
	return text + (kilometres ? " km" : " m");
}

struct tracer::impl
{
	settings options;
	std::optional<hold> held;
	std::vector<glm::dvec2> points;
	std::vector<double> sigmas;
	std::vector<state> states; // after each point; the first one before any

	impl(settings const &Settings, std::optional<hold> const &Start) : options{Settings}, held{Start}
	{
		state first;
		if (held.has_value())
		{
			first.start = {held->position, angle_of(held->heading)};
			first.started = true;
			first.from.start = first.start;
			first.from.curvature = held->curvature;
			first.from.cant = held->cant;
			first.from.fixed = held->fixed;
			first.from.held = true;
			// from a curved end, the curve goes on from it
			if (std::abs(held->curvature) > 1e-12)
			{
				curve_form curve;
				curve.side = held->curvature > 0.0 ? 1 : -1;
				curve.held = true;
				curve.fixed = held->fixed;
				curve.open = true;
				first.fit.shape.curves.push_back(curve);
				if (false == curve.fixed)
					first.fit.parameters.push_back(radius_parameter(held->curvature));
			}
		}
		build(first);
		states.push_back(std::move(first));
	}

	// --- fitting the open end ---

	// the points of the open end, those the slide moves first
	std::vector<std::size_t> fitted_points(state const &State) const
	{
		std::vector<std::size_t> indices;
		if (State.fit.shape.slide)
			indices = State.sliding;
		for (auto i{State.tail}; i < points.size(); ++i)
			indices.push_back(i);
		return indices;
	}

	departures measure(chain const &Line, std::vector<std::size_t> const &Indices) const
	{
		departures result;
		for (auto const i : Indices)
		{
			auto const at{project(Line, points[i])};
			result.offsets.push_back(at.offset);
			result.pieces.push_back(at.piece);
			auto const normalized{at.offset / sigmas[i]};
			result.worst = std::max(result.worst, std::abs(normalized));
			auto const weighed{robust(normalized)};
			result.cost += weighed * weighed;
		}
		return result;
	}

	// how far the line ends off the end held: across its last straight, its heading, the end not behind it
	static bool end_residuals(chain const &Laid, hold const &End, std::vector<double> &Residuals)
	{
		constexpr double kEnd{0.005};
		constexpr double kLever{100.0};
		if (Laid.pieces.empty())
			return false;
		auto const &last{Laid.pieces.back()};
		if (std::abs(last.shape.from) > 1e-12 || std::abs(last.shape.to) > 1e-12)
			return false;
		auto const t{unit(last.start.heading)};
		auto const offset{End.position - last.start.position};
		Residuals.push_back(cross(t, offset) / kEnd);
		Residuals.push_back(std::sin(wrapped(last.start.heading - angle_of(End.heading))) * kLever / kEnd);
		Residuals.push_back(std::min(0.0, glm::dot(t, offset)) / kEnd);
		return true;
	}

	struct candidate
	{
		model fit;
		bool valid{false};
		double cost{0.0}; // robust, the priors with it
		double worst{0.0}; // in sigmas
		std::size_t parameters{0};
		double score() const { return cost + kPenalty * static_cast<double>(parameters); }
	};

	candidate fitted(state const &State, model const &Start, budget &Budget, std::vector<std::size_t> const &Indices, std::optional<hold> const &End = {}) const
	{
		candidate result;
		auto const &anchor{State.from};
		auto const &shape{Start.shape};
		residuals_of const residuals{[&](std::vector<double> const &X, std::vector<double> &R) {
			auto const line{make(anchor, shape, X, options)};
			if (false == line.valid)
				return false;
			auto const laid{chain_of(line.start, line.segments)};
			R.clear();
			for (auto const i : Indices)
			{
				auto const at{project(laid, points[i])};
				if (false == at.found)
					return false;
				R.push_back(robust(at.offset / sigmas[i]));
			}
			R.insert(R.end(), line.priors.begin(), line.priors.end());
			return false == End.has_value() || end_residuals(laid, *End, R);
		}};
		auto parameters{Start.parameters};
		auto const cost{minimize(parameters, residuals, Budget)};
		if (false == cost.has_value())
			return result;
		auto const line{make(anchor, shape, parameters, options)};
		if (false == line.valid)
			return result;
		// an arc of a compound curve too short to be any: a kink the points made
		for (auto const &curve : line.curves)
			for (std::size_t i = 0; i < curve.arcs.size(); ++i)
				if (false == (curve.shape.open && i + 1 == curve.arcs.size()) && curve.arcs[i] < kShortestArc && curve.curvatures.size() > 1)
					return result;
		auto const laid{chain_of(line.start, line.segments)};
		auto const measured{measure(laid, Indices)};
		result.fit = {shape, parameters};
		result.valid = true;
		result.cost = *cost;
		result.worst = measured.worst;
		result.parameters = parameter_count(shape);
		return result;
	}

	// the best within the noise, the fewest parameters weighed in
	static std::optional<candidate> choose(std::vector<candidate> const &Candidates)
	{
		std::optional<candidate> best;
		auto const any_within{std::any_of(Candidates.begin(), Candidates.end(), [](candidate const &C) { return C.valid && C.worst <= kWithin; })};
		for (auto const &c : Candidates)
			if (c.valid && (false == any_within || c.worst <= kWithin) && (false == best.has_value() || c.score() < best->score()))
				best = c;
		return best;
	}

	// the open end laid by the model: the chain, its curves, the points measured
	struct laid_end
	{
		made line;
		chain laid;
		departures measured;
	};
	laid_end lay_end(state const &State, model const &Fit) const
	{
		laid_end result;
		result.line = make(State.from, Fit.shape, Fit.parameters, options);
		if (result.line.valid)
		{
			result.laid = chain_of(result.line.start, result.line.segments);
			result.measured = measure(result.laid, fitted_points(State));
		}
		return result;
	}

	// the parameters of the curve C begin where in the vector
	static std::size_t offset_of(form const &Shape, std::size_t const Curve)
	{
		std::size_t at{(Shape.free ? 2u : 0u) + (Shape.slide ? 1u : 0u) + (Shape.slide_out ? 1u : 0u)};
		for (std::size_t c = 0; c < Curve; ++c)
		{
			form one;
			one.curves = {Shape.curves[c]};
			at += parameter_count(one);
		}
		return at;
	}

	// the parameters of one curve in the order make() reads them
	struct curve_parameters
	{
		std::optional<double> lead;
		std::optional<double> in;
		std::vector<std::optional<double>> radii;
		std::vector<std::optional<double>> arcs;
		std::optional<double> out;
	};
	static curve_parameters read(form const &Shape, std::vector<double> const &X, std::size_t const Curve)
	{
		auto const &form{Shape.curves[Curve]};
		auto at{offset_of(Shape, Curve)};
		curve_parameters result;
		if (false == form.inside && false == form.held)
		{
			result.lead = X[at++];
			result.in = X[at++];
		}
		for (int i = 0; i < form.radii; ++i)
		{
			result.radii.push_back(form.fixed && i == 0 ? std::nullopt : std::optional<double>{X[at++]});
			result.arcs.push_back(form.open && i + 1 == form.radii ? std::nullopt : std::optional<double>{X[at++]});
		}
		if (false == form.open)
			result.out = X[at++];
		return result;
	}
	static void write(curve_form const &Form, curve_parameters const &Values, std::vector<double> &X)
	{
		if (false == Form.inside && false == Form.held)
		{
			X.push_back(Values.lead.value_or(1.0));
			X.push_back(Values.in.value_or(4.0));
		}
		for (int i = 0; i < Form.radii; ++i)
		{
			if (false == (Form.fixed && i == 0))
				X.push_back(i < static_cast<int>(Values.radii.size()) && Values.radii[i].has_value() ? *Values.radii[i] : std::log(1000.0));
			if (false == (Form.open && i + 1 == Form.radii))
				X.push_back(i < static_cast<int>(Values.arcs.size()) && Values.arcs[i].has_value() ? *Values.arcs[i] : parameter_of(50.0));
		}
		if (false == Form.open)
			X.push_back(Values.out.value_or(Values.in.value_or(parameter_of(40.0))));
	}

	// the parameters before the curve C, the curve given anew, the ones after it as they were
	static std::vector<double> replaced(form const &Shape, std::vector<double> const &X, std::size_t const Curve, curve_form const &Form, curve_parameters const &Values)
	{
		std::vector<double> result(X.begin(), X.begin() + static_cast<std::ptrdiff_t>(offset_of(Shape, Curve)));
		write(Form, Values, result);
		if (Curve + 1 < Shape.curves.size())
			result.insert(result.end(), X.begin() + static_cast<std::ptrdiff_t>(offset_of(Shape, Curve + 1)), X.end());
		return result;
	}

	// a curve opened at the end of the straight the open end ends with, turning towards the point clicked last
	std::vector<model> opened(state const &State) const
	{
		auto const &shape{State.fit.shape};
		if (false == shape.curves.empty() && shape.curves.back().open)
			return {};
		return open_after(State, State.fit);
	}

	// the straight of the points of a line not held keeping to one from the first: the longest run of them within the noise, two at least
	model straight_run() const
	{
		auto count{std::min<std::size_t>(2, points.size())};
		auto fit{first_straight(count)};
		while (count < points.size())
		{
			auto const longer{first_straight(count + 1)};
			auto const t{unit(longer.parameters[1])};
			auto const origin{points.front() + left_of(longer.parameters[1]) * longer.parameters[0]};
			auto within{true};
			for (std::size_t i = 0; i <= count && within; ++i)
				within = std::abs(cross(t, points[i] - origin)) <= kWithin * sigmas[i];
			if (false == within)
				break;
			fit = longer;
			++count;
		}
		return fit;
	}

	// the last curve of the open end, an open one, begun afresh: the straight before it through the points keeping to one, the curve
	// through the rest
	std::vector<model> restarted(state const &State) const
	{
		auto const &shape{State.fit.shape};
		if (shape.curves.empty() || false == shape.curves.back().open || shape.curves.back().held)
			return {};
		model base;
		base.shape = shape;
		base.shape.curves.pop_back();
		base.parameters.assign(State.fit.parameters.begin(), State.fit.parameters.begin() + static_cast<std::ptrdiff_t>(offset_of(shape, shape.curves.size() - 1)));
		if (base.shape.free && base.shape.curves.empty())
			base = straight_run();
		return open_after(State, base);
	}

	std::vector<model> open_after(state const &State, model const &Base) const
	{
		std::vector<model> guesses;
		auto const &shape{Base.shape};
		auto const current{lay_end(State, Base)};
		if (false == current.line.valid || current.laid.pieces.empty())
			return guesses;
		auto const &last{current.laid.pieces.back()};
		auto const origin{last.start.position};
		auto const heading{last.start.heading};
		auto const t{unit(heading)};
		auto const n{left_of(heading)};
		auto const &point{points.back()};
		auto const ahead{glm::dot(point - origin, t)};
		auto const aside{glm::dot(point - origin, n)};
		if (ahead <= 1.0 || std::abs(aside) < 1e-6)
			return guesses;
		int const side{aside > 0.0 ? 1 : -1};
		// where the points leave the straight: after the last of them keeping to it
		double leaving{0.0};
		auto const indices{fitted_points(State)};
		for (std::size_t k = 0; k + 1 < indices.size(); ++k)
		{
			auto const i{indices[k]};
			if (current.measured.pieces[k] + 1 != current.laid.pieces.size())
				continue;
			auto const along{glm::dot(points[i] - origin, t)};
			if (along > 0.0 && along < ahead && std::abs(glm::dot(points[i] - origin, n)) <= 1.5 * sigmas[i])
				leaving = std::max(leaving, along);
		}
		for (auto const share : {0.0, 0.5})
		{
			auto const centre{leaving + (ahead - leaving) * share};
			auto const run{std::max(1.0, ahead - centre)};
			auto const radius{std::clamp(run * run / (2.0 * std::abs(aside)), kRadiusLeast * 2.0, kRadiusMost * 0.9)};
			auto const allowed{allowed_speed(options.speed, radius, options.norms)};
			auto const in{std::clamp(norm_between(allowed, 0.0, 0.0, 1.0 / radius, norm_cant(allowed, radius, options.norms), options.norms) + 2.5, 5.0, std::max(5.0, 0.8 * run))};
			auto with{shape};
			curve_form form;
			form.side = side;
			auto parameters{Base.parameters};
			parameters.push_back(parameter_of(std::max(0.5, centre - 0.5 * in)));
			parameters.push_back(parameter_of(in));
			parameters.push_back(radius_parameter(1.0 / radius));
			with.curves.push_back(form);
			guesses.push_back({with, parameters});
		}
		return guesses;
	}

	// the open last curve closed: its last arc ending so that the straight after it heads for the target, by bisection
	std::vector<model> closed(state const &State, glm::dvec2 const &Target) const
	{
		std::vector<model> guesses;
		auto const &shape{State.fit.shape};
		if (shape.curves.empty() || false == shape.curves.back().open)
			return guesses;
		auto const c{shape.curves.size() - 1};
		auto const values{read(shape, State.fit.parameters, c)};
		auto form{shape.curves.back()};
		form.open = false;
		auto const current{lay_end(State, State.fit)};
		if (false == current.line.valid || current.laid.pieces.empty())
			return guesses;
		auto const &curve{current.line.curves.back()};
		auto const k{std::abs(curve.curvatures.back())};
		auto const allowed{allowed_speed(options.speed, 1.0 / k, options.norms)};
		auto const out{norm_between(allowed, k, norm_cant(allowed, 1.0 / k, options.norms), 0.0, 0.0, options.norms) + 2.5};
		auto const at_length = [&](double const Length) {
			auto values_now{values};
			values_now.arcs.back() = parameter_of(Length);
			values_now.out = parameter_of(out);
			auto with{shape};
			with.curves.back() = form;
			return model{with, replaced(shape, State.fit.parameters, c, form, values_now)};
		};
		auto const across = [&](double const Length) -> std::optional<double> {
			auto const guess{at_length(Length)};
			auto const line{make(State.from, guess.shape, guess.parameters, options)};
			if (false == line.valid)
				return std::nullopt;
			auto const laid{chain_of(line.start, line.segments)};
			auto const &straight{laid.pieces.back()};
			return cross(unit(straight.start.heading), Target - straight.start.position);
		};
		// the arc as far as abreast of the target at most, the straight after it then passes the target on the side the curve turns to
		auto const reach{onto(current.laid, current.laid.pieces.size() - 1, Target)};
		auto const side{static_cast<double>(form.side)};
		auto low{1.0};
		auto high{std::max(2.0, reach.found ? reach.distance : 1.0)};
		auto const at_low{across(low)};
		auto const at_high{across(high)};
		if (at_low.has_value() && at_high.has_value() && *at_low * side > 0.0 && *at_high * side < 0.0)
		{
			for (int i = 0; i < 40 && high - low > 0.01; ++i)
			{
				auto const middle{0.5 * (low + high)};
				auto const value{across(middle)};
				if (false == value.has_value())
					break;
				(*value * side > 0.0 ? low : high) = middle;
			}
			guesses.push_back(at_length(0.5 * (low + high)));
		}
		else
			guesses.push_back(at_length(std::max(1.0, 0.5 * high)));
		return guesses;
	}

	// the last arc of the open last curve split: after the point before the last one, of the curvature the last points show
	std::vector<model> split(state const &State) const
	{
		std::vector<model> guesses;
		auto const &shape{State.fit.shape};
		if (shape.curves.empty() || false == shape.curves.back().open || shape.curves.back().radii >= 3 || points.size() < 3)
			return guesses;
		auto const c{shape.curves.size() - 1};
		auto const current{lay_end(State, State.fit)};
		if (false == current.line.valid || current.laid.pieces.empty())
			return guesses;
		auto const before{onto(current.laid, current.laid.pieces.size() - 1, points[points.size() - 2])};
		if (false == before.found || before.distance < kShortestArc)
			return guesses;
		auto const values{read(shape, State.fit.parameters, c)};
		auto form{shape.curves.back()};
		auto const k{current.line.curves.back().curvatures.back()};
		// the circle through the last three points
		auto const &a{points[points.size() - 3]};
		auto const &b{points[points.size() - 2]};
		auto const &p{points.back()};
		auto const twice{2.0 * cross(b - a, p - a)};
		std::vector<double> curvatures;
		if (std::abs(twice) > 1e-9 && sign_of(twice) == sign_of(k))
			curvatures.push_back(sign_of(k) * 2.0 * std::abs(cross(b - a, p - a)) / (glm::length(b - a) * glm::length(p - b) * glm::length(p - a)));
		curvatures.push_back(k * 1.4);
		curvatures.push_back(k * 0.7);
		++form.radii;
		for (auto const next : curvatures)
		{
			auto values_now{values};
			values_now.arcs.back() = parameter_of(before.distance);
			values_now.radii.push_back(radius_parameter(next));
			values_now.arcs.push_back(std::nullopt);
			auto with{shape};
			with.curves.back() = form;
			guesses.push_back({with, replaced(shape, State.fit.parameters, c, form, values_now)});
		}
		return guesses;
	}

	// the closed curve C of a radius more than it has: its longest arc cut in two, one part sharper than the other
	std::vector<model> more_radii(state const &State, model const &Fit, std::size_t const C) const
	{
		std::vector<model> guesses;
		auto const &shape{Fit.shape};
		auto form{shape.curves[C]};
		if (form.radii >= 3 || form.open)
			return guesses;
		auto const values{read(shape, Fit.parameters, C)};
		auto const line{make(State.from, shape, Fit.parameters, options)};
		if (false == line.valid)
			return guesses;
		auto const &curve{line.curves[C]};
		std::size_t longest{0};
		for (std::size_t i = 0; i < curve.arcs.size(); ++i)
			if (curve.arcs[i] > curve.arcs[longest])
				longest = i;
		if (curve.arcs[longest] < 3.0 * kShortestArc)
			return guesses;
		++form.radii;
		for (auto const sharper : {false, true})
		{
			curve_parameters values_now;
			values_now.lead = values.lead;
			values_now.in = values.in;
			values_now.out = values.out;
			for (std::size_t i = 0; i < curve.arcs.size(); ++i)
			{
				if (i != longest)
				{
					values_now.radii.push_back(values.radii[i]);
					values_now.arcs.push_back(values.arcs[i]);
					continue;
				}
				auto const radius{1.0 / std::abs(curve.curvatures[i])};
				auto const half{parameter_of(0.5 * curve.arcs[i])};
				values_now.radii.push_back(form.fixed && i == 0 ? std::nullopt : std::optional<double>{std::log(radius * (sharper ? 0.85 : 1.15))});
				values_now.arcs.push_back(half);
				values_now.radii.push_back(std::log(radius * (sharper ? 1.15 : 0.85)));
				values_now.arcs.push_back(half);
			}
			auto with{shape};
			with.curves[C] = form;
			guesses.push_back({with, replaced(shape, Fit.parameters, C, form, values_now)});
		}
		return guesses;
	}

	// the open end begun as one open curve from the first point of the line, the start within its arc
	std::vector<model> from_inside(state const &State) const
	{
		std::vector<model> guesses;
		if (false == State.fit.shape.free || false == State.fit.shape.curves.empty() || points.size() < 3 || State.tail != 0)
			return guesses;
		// the circle through the first, the middle and the last point
		auto const &a{points.front()};
		auto const &b{points[points.size() / 2]};
		auto const &c{points.back()};
		auto const ab{b - a};
		auto const ac{c - a};
		auto const twice{2.0 * cross(ab, ac)};
		if (std::abs(twice) < 1e-9)
			return guesses;
		auto const centre{a + glm::dvec2{ac.y * glm::dot(ab, ab) - ab.y * glm::dot(ac, ac), ab.x * glm::dot(ac, ac) - ac.x * glm::dot(ab, ab)} / twice};
		auto const radius{glm::distance(centre, a)};
		int const side{twice > 0.0 ? 1 : -1};
		auto const radial{a - centre};
		auto const heading{angle_of(glm::dvec2{-radial.y, radial.x} * static_cast<double>(side))};
		form shape;
		shape.free = true;
		curve_form curve;
		curve.side = side;
		curve.inside = true;
		shape.curves.push_back(curve);
		guesses.push_back({shape, {0.0, heading, radius_parameter(1.0 / radius)}});
		return guesses;
	}

	// the closed last curve going on still: its exit taken back
	std::vector<model> reopened(state const &State) const
	{
		auto const &shape{State.fit.shape};
		if (shape.curves.empty() || shape.curves.back().open)
			return {};
		auto const c{shape.curves.size() - 1};
		auto values{read(shape, State.fit.parameters, c)};
		auto form{shape.curves.back()};
		form.open = true;
		values.arcs.back() = std::nullopt;
		values.out.reset();
		auto with{shape};
		with.curves.back() = form;
		return {model{with, replaced(shape, State.fit.parameters, c, form, values)}};
	}

	// the curve C of an arc less than it has, the first or the last one dropped, the one next to it taking over its length; closed where
	// Close says so
	std::vector<model> fewer_radii(state const &State, model const &Fit, std::size_t const C, bool const Close) const
	{
		std::vector<model> guesses;
		auto const &shape{Fit.shape};
		auto form{shape.curves[C]};
		if (form.radii < 2)
			return guesses;
		auto const values{read(shape, Fit.parameters, C)};
		auto const line{make(State.from, shape, Fit.parameters, options)};
		if (false == line.valid)
			return guesses;
		auto const &curve{line.curves[C]};
		auto const open{form.open && false == Close};
		for (auto const last : {true, false})
		{
			if (form.fixed && false == last)
				continue;
			auto fewer{form};
			--fewer.radii;
			fewer.open = open;
			curve_parameters values_now;
			values_now.lead = values.lead;
			values_now.in = values.in;
			auto const n{curve.arcs.size()};
			auto const keep_from{last ? std::size_t{0} : std::size_t{1}};
			auto const keep_to{last ? n - 1 : n};
			for (auto i{keep_from}; i < keep_to; ++i)
			{
				values_now.radii.push_back(values.radii[i]);
				auto length{curve.arcs[i]};
				// the arc dropped goes into the one next to it
				if ((last && i + 1 == keep_to) || (false == last && i == keep_from))
					length += curve.arcs[last ? n - 1 : 0];
				values_now.arcs.push_back(i + 1 == keep_to && open ? std::nullopt : std::optional<double>{parameter_of(std::max(1.0, length))});
			}
			if (false == open)
				values_now.out = values.out.value_or(values.in.value_or(parameter_of(40.0)));
			auto with{shape};
			with.curves[C] = fewer;
			guesses.push_back({with, replaced(shape, Fit.parameters, C, fewer, values_now)});
		}
		return guesses;
	}

	// the closed last curve going on still, flatter or sharper: its exit taken back, an arc more after its last one, of the curvature the
	// last points show
	std::vector<model> reopened_more(state const &State) const
	{
		std::vector<model> guesses;
		auto const &shape{State.fit.shape};
		if (shape.curves.empty() || shape.curves.back().open || shape.curves.back().radii >= 3 || points.size() < 3)
			return guesses;
		auto const c{shape.curves.size() - 1};
		auto const current{lay_end(State, State.fit)};
		if (false == current.line.valid)
			return guesses;
		auto values{read(shape, State.fit.parameters, c)};
		auto form{shape.curves.back()};
		form.open = true;
		++form.radii;
		values.out.reset();
		auto const k{current.line.curves.back().curvatures.back()};
		auto const &a{points[points.size() - 3]};
		auto const &b{points[points.size() - 2]};
		auto const &p{points.back()};
		auto const twice{2.0 * cross(b - a, p - a)};
		std::vector<double> curvatures;
		if (std::abs(twice) > 1e-9 && sign_of(twice) == sign_of(k))
			curvatures.push_back(sign_of(k) * 2.0 * std::abs(cross(b - a, p - a)) / (glm::length(b - a) * glm::length(p - b) * glm::length(p - a)));
		curvatures.push_back(k * 0.7);
		curvatures.push_back(k * 1.4);
		for (auto const next : curvatures)
		{
			auto values_now{values};
			values_now.radii.push_back(radius_parameter(next));
			values_now.arcs.push_back(std::nullopt);
			auto with{shape};
			with.curves.back() = form;
			guesses.push_back({with, replaced(shape, State.fit.parameters, c, form, values_now)});
		}
		return guesses;
	}

	// a curve closed before the end held, where the straight the open end ends with doesn't lead into it
	std::vector<model> into(state const &State, hold const &End) const
	{
		std::vector<model> guesses;
		auto const &shape{State.fit.shape};
		if (false == shape.curves.empty() && shape.curves.back().open)
			return guesses;
		auto const current{lay_end(State, State.fit)};
		if (false == current.line.valid || current.laid.pieces.empty())
			return guesses;
		auto const &last{current.laid.pieces.back()};
		auto const deflection{wrapped(angle_of(End.heading) - last.start.heading)};
		if (std::abs(deflection) < 1e-4)
			return guesses;
		auto const t{unit(last.start.heading)};
		auto const e{glm::normalize(End.heading)};
		auto const determinant{cross(t, e)};
		if (std::abs(determinant) < 1e-9)
			return guesses;
		auto const a{cross(End.position - last.start.position, e) / determinant};
		auto const b{cross(End.position - last.start.position, t) / determinant};
		if (a <= 1.0 || b >= -1.0)
			return guesses;
		auto const tangent{0.6 * std::min(a, -b)};
		auto const radius{std::clamp(tangent / std::tan(0.5 * std::abs(deflection)), kRadiusLeast * 2.0, kRadiusMost * 0.9)};
		auto const allowed{allowed_speed(options.speed, radius, options.norms)};
		auto const in{std::clamp(norm_between(allowed, 0.0, 0.0, 1.0 / radius, norm_cant(allowed, radius, options.norms), options.norms) + 2.5, 5.0, std::max(5.0, 0.4 * radius * std::abs(deflection)))};
		curve_form form;
		form.side = deflection > 0.0 ? 1 : -1;
		form.open = false;
		auto with{shape};
		with.curves.push_back(form);
		auto parameters{State.fit.parameters};
		parameters.push_back(parameter_of(std::max(0.5, a - tangent - 0.5 * in)));
		parameters.push_back(parameter_of(in));
		parameters.push_back(radius_parameter(1.0 / radius));
		parameters.push_back(parameter_of(std::max(1.0, radius * std::abs(deflection) - in)));
		parameters.push_back(parameter_of(in));
		guesses.push_back({with, parameters});
		return guesses;
	}

	// the shapes tried for the point clicked last, from the one fitted before it
	std::vector<model> guesses(state const &State) const
	{
		std::vector<model> result{State.fit};
		auto const &shape{State.fit.shape};
		auto const add = [&](std::vector<model> const &More) { result.insert(result.end(), More.begin(), More.end()); };
		// from the diverging track of a switch held at the start, a straight right at it too
		if (State.from.held && State.from.fixed && false == State.from.slides && shape.curves.size() == 1 && shape.curves.front().held)
			result.push_back(model{});
		if (shape.curves.empty() || false == shape.curves.back().open)
		{
			// the last a straight: a curve begins on it, or the line begins within one, or the curve closed goes on still
			add(opened(State));
			add(from_inside(State));
			add(reopened(State));
			add(reopened_more(State));
		}
		else
		{
			add(closed(State, points.back()));
			add(split(State));
			add(restarted(State));
			// a compound curve whose last arc only began the straight after it: closed after the arc before it
			if (shape.curves.back().radii > 1)
				add(fewer_radii(State, State.fit, shape.curves.size() - 1, true));
		}
		return result;
	}

	// the straight of the first points of a line not held, as the least squares across have it
	model first_straight(std::size_t Count = 0) const
	{
		model fit;
		fit.shape.free = true;
		auto const count{Count == 0 ? points.size() : std::min(Count, points.size())};
		glm::dvec2 mean{0.0};
		for (std::size_t i = 0; i < count; ++i)
			mean += points[i];
		mean /= static_cast<double>(count);
		double xx{0.0}, xy{0.0}, yy{0.0};
		for (std::size_t i = 0; i < count; ++i)
		{
			auto const d{points[i] - mean};
			xx += d.x * d.x;
			xy += d.x * d.y;
			yy += d.y * d.y;
		}
		auto heading{0.5 * std::atan2(2.0 * xy, xx - yy)};
		if (glm::dot(unit(heading), points[count - 1] - points.front()) < 0.0)
			heading += kPi;
		auto const across{glm::dot(mean - points.front(), left_of(heading))};
		fit.parameters = {across, heading};
		return fit;
	}

	// --- one click ---

	state click(state const &Before, budget &Budget) const
	{
		state next{Before};
		next.misfit.resize(points.size(), 0);
		auto const count{points.size()};
		// a line not held: the start from the first point, the straight to the second
		if (false == next.started)
		{
			if (count >= 2)
			{
				next.started = true;
				next.from.first = points.front();
				next.from.start = {points.front(), angle_of(points[1] - points[0])};
				next.fit = first_straight();
			}
			build(next);
			return next;
		}
		auto const indices{fitted_points(next)};
		std::vector<candidate> tried;
		auto const all{guesses(Before)};
		for (std::size_t g = 0; g < all.size(); ++g)
		{
			auto const &guess{all[g]};
			// the straight of the start fitted afresh: its least squares
			if (guess.shape.free && guess.shape.curves.empty())
				tried.push_back(fitted(next, first_straight(), Budget, indices));
			else
				tried.push_back(fitted(next, guess, Budget, indices));
			if (Budget.spent)
				break;
			// the shape of before keeping to the point clicked within twice its noise, the others within the noise too: no other
			// shape of more parameters could bring the sum of the squares down enough to be taken
			if (g == 0 && tried.front().valid && tried.front().worst <= kWithin && Before.worst <= kWithin)
			{
				auto const line{make(next.from, tried.front().fit.shape, tried.front().fit.parameters, options)};
				auto const at{project(chain_of(line.start, line.segments), points.back())};
				if (at.found && std::abs(at.offset) <= 2.0 * sigmas.back())
					break;
			}
		}
		auto const best{choose(tried)};
		// none, the budget spent, or none keeping to the points within the noise where the line did before: the line stays, the point told
		// two points running which don't fit: the line goes on with the best of them, rather than staying behind for good
		auto const stuck{count >= 2 && Before.misfit.size() >= count - 1 && Before.misfit[count - 2] != 0};
		if (Budget.spent || false == best.has_value() || (best->worst > kWithin && Before.worst <= kWithin && false == stuck))
		{
			next.misfit.back() = 1;
			build(next);
			return next;
		}
		next.fit = best->fit;
		next.worst = best->worst;
		for (auto const i : indices)
			next.misfit[i] = 0;
		freeze(next, Budget);
		build(next);
		return next;
	}

	// the curves of the open end with two points on the straight after them frozen, the straight turning about the end of the last one;
	// the radii of the last one frozen chosen among 1, 2 and 3 first
	void freeze(state &State, budget &Budget) const
	{
		auto end{lay_end(State, State.fit)};
		if (false == end.line.valid)
			return;
		auto indices{fitted_points(State)};
		// the last closed curve with two points on the straight after it: abreast of the lead of the next curve or of the last straight
		auto const frozen_to = [&](laid_end const &End, form const &Shape) -> std::optional<std::size_t> {
			std::optional<std::size_t> upto;
			for (std::size_t c = 0; c < Shape.curves.size(); ++c)
			{
				if (Shape.curves[c].open)
					break;
				auto const from{End.line.spans[c].second};
				auto const to{c + 1 < Shape.curves.size() ? End.line.spans[c + 1].first + (End.line.curves[c + 1].lead > 0.0 ? 1 : 0) : End.laid.pieces.size()};
				std::size_t on{0};
				for (std::size_t k = 0; k < indices.size(); ++k)
					if (indices[k] >= State.tail && End.measured.pieces[k] >= from && End.measured.pieces[k] < to && std::abs(End.measured.offsets[k]) <= kWithin * sigmas[indices[k]])
						++on;
				if (on >= 2)
					upto = c;
			}
			// three curves in the open end: the first one frozen whatever comes after it
			if (false == upto.has_value() && Shape.curves.size() >= 3 && false == Shape.curves.front().open)
				upto = 0;
			return upto;
		};
		auto upto{frozen_to(end, State.fit.shape)};
		if (false == upto.has_value())
			return;
		// the radii of the curves frozen: a radius more where it keeps clearly closer to the points
		auto improved{fitted(State, State.fit, Budget, indices)};
		for (std::size_t c = 0; c <= *upto && improved.valid && false == Budget.spent; ++c)
			for (auto const fewer : {true, false})
				for (;;)
				{
					std::vector<candidate> more;
					for (auto const &guess : fewer ? fewer_radii(State, improved.fit, c, true) : more_radii(State, improved.fit, c))
						more.push_back(fitted(State, guess, Budget, indices));
					auto const better{choose(more)};
					if (false == better.has_value() || better->score() >= improved.score() || (better->worst > kWithin && improved.worst <= kWithin))
						break;
					improved = *better;
				}
		if (improved.valid && false == Budget.spent)
		{
			State.fit = improved.fit;
			State.worst = improved.worst;
			end = lay_end(State, State.fit);
			upto = frozen_to(end, State.fit.shape);
			if (false == upto.has_value())
				return;
		}
		auto const c{*upto};
		auto const &shape{State.fit.shape};
		auto const &line{end.line};
		// the slide of the curve frozen before goes into its arc for good
		if (State.from.slides && State.slide_segment < State.frozen.size())
		{
			auto &curve{State.frozen_curves.back()};
			State.frozen[State.slide_segment].length = line.segments.front().length;
			if (false == curve.curve.arcs.empty())
				curve.curve.arcs.back() = line.segments.front().length;
			if (State.from.out.length > 0.0 && line.segments.size() > 1 && State.slide_segment + 1 < State.frozen.size())
				curve.curve.out = State.frozen[State.slide_segment + 1].length = line.segments[1].length;
			curve.settled.reset();
		}
		// the start of the line is kept from now on
		if (false == State.from.slides)
			State.start = line.start;
		auto const base{State.frozen.size()};
		auto const first{line.first_tail};
		for (std::size_t s = first; s < line.spans[c].second; ++s)
			State.frozen.push_back(line.segments[s]);
		for (std::size_t k = 0; k <= c; ++k)
		{
			frozen_curve curve;
			curve.curve = line.curves[k];
			curve.first = base + (line.spans[k].first - first);
			curve.last = base + (line.spans[k].second - first);
			State.frozen_curves.push_back(curve);
		}
		// the new anchor: the pose before the last arc of the curve frozen, the arc and its transition out
		auto const &last{line.curves[c]};
		auto const arc_index{line.spans[c].second - (last.out > 0.0 ? 2 : 1)};
		State.slide_segment = base + (arc_index - first);
		State.from.slides = true;
		State.from.before = end.laid.pieces[arc_index].start;
		State.from.arc = line.segments[arc_index];
		State.from.out = last.out > 0.0 ? line.segments[arc_index + 1] : segment{};
		State.from.held = false;
		// the points the slide moves: those abreast of the arc and of its transition out; the open end from the first one after them
		std::vector<std::size_t> sliding;
		auto tail{points.size()};
		for (std::size_t k = 0; k < indices.size(); ++k)
		{
			auto const piece{end.measured.pieces[k]};
			if (piece == arc_index || (last.out > 0.0 && piece == arc_index + 1))
				sliding.push_back(indices[k]);
			if (piece >= line.spans[c].second && indices[k] >= State.tail)
				tail = std::min(tail, indices[k]);
		}
		State.sliding = sliding;
		State.tail = tail;
		form rest;
		rest.slide = true;
		rest.slide_out = last.out > 0.0;
		rest.curves.assign(shape.curves.begin() + static_cast<std::ptrdiff_t>(c + 1), shape.curves.end());
		std::vector<double> x{0.0};
		if (rest.slide_out)
			x.push_back(parameter_of(last.out));
		if (c + 1 < shape.curves.size())
			x.insert(x.end(), State.fit.parameters.begin() + static_cast<std::ptrdiff_t>(offset_of(shape, c + 1)), State.fit.parameters.end());
		State.fit = {rest, x};
	}

	// --- the line as the design lays it ---

	// the result of the state: its line built, the norms asked of the curves frozen kept with them
	void build(state &State) const
	{
		State.line = built(State, std::nullopt);
	}

	result built(state &State, std::optional<hold> const &End) const
	{
		result out;
		out.sigmas = sigmas;
		out.misfit = State.misfit;
		out.misfit.resize(points.size(), 0);
		out.frozen = State.frozen_curves.size();
		if (points.size() + (held.has_value() ? 1u : 0u) + (End.has_value() ? 1u : 0u) < 2 || false == State.started)
		{
			out.error = "points";
			return out;
		}
		auto const end{lay_end(State, State.fit)};
		if (false == end.line.valid)
		{
			out.error = "fit";
			return out;
		}
		// the whole line: the frozen segments from the start, the slide applied to the arc of the last frozen curve, the open end
		std::vector<segment> segments{State.frozen};
		std::vector<curve_made> curves;
		std::vector<std::pair<std::size_t, std::size_t>> spans;
		for (auto const &curve : State.frozen_curves)
		{
			curves.push_back(curve.curve);
			spans.emplace_back(curve.first, curve.last);
		}
		auto const &line{end.line};
		if (State.from.slides && State.slide_segment < segments.size())
		{
			segments[State.slide_segment].length = line.segments.front().length;
			auto &arcs{curves.back().arcs};
			if (false == arcs.empty())
				arcs.back() = line.segments.front().length;
			if (State.from.out.length > 0.0 && line.segments.size() > 1 && State.slide_segment + 1 < segments.size())
				curves.back().out = segments[State.slide_segment + 1].length = line.segments[1].length;
		}
		auto const base{segments.size()};
		for (std::size_t s = line.first_tail; s < line.segments.size(); ++s)
			segments.push_back(line.segments[s]);
		for (std::size_t c = 0; c < line.curves.size(); ++c)
		{
			curves.push_back(line.curves[c]);
			spans.emplace_back(base + line.spans[c].first - line.first_tail, base + line.spans[c].second - line.first_tail);
		}
		auto const start{State.frozen.empty() && false == State.from.slides ? line.start : State.start};
		auto whole{chain_of(start, segments)};
		if (whole.pieces.empty())
		{
			out.error = "fit";
			return out;
		}
		// the line ends at the end held, or abreast of the last point it was fitted to, within its last piece carried on
		auto last_point{points.size()};
		while (last_point > 0 && State.misfit.size() >= last_point && State.misfit[last_point - 1] != 0)
			--last_point;
		auto const target{End.has_value() ? End->position : last_point > 0 ? points[last_point - 1] : start.position};
		auto const tip{onto(whole, whole.pieces.size() - 1, target)};
		segments.back().length = std::max(0.0, tip.found ? tip.distance : 0.0);
		if (false == curves.empty() && curves.back().shape.open && spans.back().second == segments.size())
			curves.back().arcs.back() = segments.back().length;
		whole = chain_of(start, segments);
		auto const &tail{whole.pieces.back()};
		alignment::design design;
		design.speed = options.speed;
		design.norms = options.norms;
		design.shape = options.shape;
		design.start = {start.position.x, 0.0, start.position.y};
		design.start_direction = unit(start.heading);
		auto const finish{End.has_value() ? End->position : tail.end.position};
		design.end = {finish.x, 0.0, finish.y};
		design.end_direction = End.has_value() ? glm::normalize(End->heading) : unit(tail.end.heading);
		// the vertices: where the straights either side of each curve cross
		std::vector<curve> told;
		std::vector<std::size_t> made_of; // the curve of the fit each one is
		std::vector<std::pair<glm::dvec2, glm::dvec2>> ways;
		for (std::size_t c = 0; c < curves.size(); ++c)
		{
			auto const &made{curves[c]};
			auto const [first, past]{spans[c]};
			if (first >= whole.pieces.size() || past == 0 || past > whole.pieces.size())
				continue;
			auto const begins{first + (made.lead > 0.0 ? 1 : 0)};
			if (begins >= past)
				continue;
			auto const entry{whole.pieces[begins].start};
			auto const exit{whole.pieces[past - 1].end};
			auto const in{unit(entry.heading)};
			auto const outgoing{unit(exit.heading)};
			auto const determinant{cross(in, outgoing)};
			// a curve turning hardly at all is laid as a straight
			if (std::abs(determinant) < 1e-7 && glm::dot(in, outgoing) > 0.0)
				continue;
			auto const along{cross(exit.position - entry.position, outgoing) / determinant};
			alignment::vertex vertex;
			vertex.position = entry.position + in * along;
			vertex.radius = 1.0 / std::abs(made.curvatures.front());
			vertex.transition_in = made.in;
			vertex.transition_out = made.shape.open ? 0.0 : made.out;
			if (made.curvatures.size() > 1)
			{
				vertex.compound = true;
				vertex.share = std::abs(made.curvatures.front()) * made.arcs.front();
				for (std::size_t i = 1; i < made.curvatures.size(); ++i)
				{
					alignment::compound_arc arc;
					arc.radius = 1.0 / std::abs(made.curvatures[i]);
					arc.transition = made.between[i - 1];
					arc.share = std::abs(made.curvatures[i]) * made.arcs[i];
					vertex.arcs.push_back(arc);
				}
			}
			curve one;
			one.vertex = vertex;
			for (auto const k : made.curvatures)
				one.radii.push_back(1.0 / std::abs(k));
			one.open = made.shape.open && past == segments.size();
			one.inside = begins == 0 && (made.shape.inside || made.shape.held);
			one.frozen = c < State.frozen_curves.size();
			one.nearly_straight = one.sharpest() >= kNearlyStraight;
			told.push_back(one);
			made_of.push_back(c);
			ways.emplace_back(in, outgoing);
		}
		for (auto const &one : told)
			design.vertices.push_back(one.vertex);
		// a line beginning within an arc, at a curved end held or anywhere, goes on with its curve from there; one ending within one goes on
		// into it
		if (false == told.empty() && told.front().inside)
		{
			auto const &first{curves[made_of.front()]};
			design.start_curved = true;
			design.start_radius = std::abs(first.from) > 1e-12 ? 1.0 / std::abs(first.from) : told.front().radii.front();
			design.start_turn = first.shape.side;
			design.start_held = first.shape.fixed;
			if (held.has_value() && std::abs(held->curvature) > 1e-12)
				design.start_cant = std::max(0.0, held->cant);
		}
		else if (held.has_value() && std::abs(held->curvature) > 1e-12)
		{
			// a straight right from a curved end: the curvature jumps there, as at the end of a switch
			design.start_radius = 1.0 / std::abs(held->curvature);
			design.start_turn = held->curvature > 0.0 ? 1 : -1;
			design.start_held = held->fixed;
		}
		if (false == told.empty() && told.back().open)
		{
			design.end_curved = true;
			design.end_radius = told.back().radii.back();
			design.end_turn = curves[made_of.back()].shape.side;
		}
		// the curves laid with the transitions of the norms may need a little more room than the fit took: the straights at the ends of the
		// line carried on, not one before a free end held
		design.end_reserve = End.has_value() ? 0.0 : 1000.0;
		design.start_reserve = held.has_value() || design.start_curved ? 0.0 : 1000.0;
		if (false == told.empty() && told.back().open && told.size() == 1 && design.start_curved)
			design.end_reserve = 1.0;
		if (design.vertices.size() >= 2)
		{
			design.vertices.front().offset = glm::dot(design.vertices.front().position - start.position, unit(start.heading));
			design.vertices.back().offset = glm::dot(finish - design.vertices.back().position, glm::normalize(design.end_direction));
		}
		// the norms: the transitions of the norms where the fit keeps close to them, the cants, the speed of each curve; those of the
		// curves frozen once
		for (std::size_t k = 0; k < told.size(); ++k)
		{
			auto const c{made_of[k]};
			// the curve frozen last settled anew while its end slides
			auto const sliding{State.fit.shape.slide && c + 1 == State.frozen_curves.size()};
			auto *frozen{c < State.frozen_curves.size() && false == sliding ? &State.frozen_curves[c] : nullptr};
			settled_curve settled;
			if (frozen != nullptr && frozen->settled.has_value())
				settled = *frozen->settled;
			else
			{
				settled = settle(design, k, told[k], ways[k].first, ways[k].second);
				if (frozen != nullptr)
					frozen->settled = settled;
			}
			auto &vertex{design.vertices[k]};
			vertex.transition_in = settled.in;
			vertex.transition_out = settled.out;
			for (std::size_t i = 0; i < vertex.arcs.size() && i < settled.between.size(); ++i)
				vertex.arcs[i].transition = settled.between[i];
			vertex.cant = settled.cant;
			vertex.speed = settled.speed;
			auto &one{told[k]};
			one.vertex = vertex;
			one.speed = settled.speed > 0.0 ? settled.speed : options.speed;
			one.restricted = settled.speed > 0.0;
			one.refused = settled.refused;
			if (k + 1 == told.size() && design.end_curved)
				design.end_cant = vertex.compound ? alignment::compound_cants(vertex).back() : vertex.cant;
			if (k == 0 && design.start_curved && false == held.has_value())
				design.start_cant = vertex.compound ? alignment::compound_cants(vertex).front() : vertex.cant;
		}
		out.design = design;
		out.curves = told;
		out.laid = alignment::compute(design);
		if (false == out.laid.valid)
		{
			// the transitions as the fit has them, clothoids
			auto again{design};
			again.shape = alignment::transition_shape::clothoid;
			if (auto const laid{alignment::compute(again)}; laid.valid)
			{
				out.design = again;
				out.laid = laid;
			}
		}
		if (false == out.laid.valid)
		{
			out.error = out.laid.errors.empty() ? std::string{"fit"} : out.laid.errors.front();
			return out;
		}
		// where each curve lies along the line
		for (std::size_t k = 0; k < out.curves.size(); ++k)
		{
			auto &one{out.curves[k]};
			one.from = out.laid.length;
			one.to = 0.0;
			for (auto const &e : out.laid.elements)
				if (e.vertex == static_cast<int>(k) && e.kind != alignment::element_kind::straight && e.length > 1e-6)
				{
					one.from = std::min(one.from, e.chainage);
					one.to = std::max(one.to, e.chainage + e.length);
				}
		}
		out.valid = true;
		// the points measured against the line laid
		auto const projections{alignment::project(out.laid, points)};
		double sum{0.0};
		for (std::size_t i = 0; i < points.size(); ++i)
		{
			auto const offset{i < projections.size() ? projections[i].offset : 0.0};
			out.offsets.push_back(offset);
			sum += offset * offset;
			out.worst = std::max(out.worst, std::abs(offset));
			auto straight{true};
			if (i < projections.size())
				for (auto const &one : out.curves)
					if (projections[i].chainage > one.from && projections[i].chainage < one.to)
						straight = false;
			out.on_straight.push_back(straight ? 1 : 0);
		}
		out.rms = points.empty() ? 0.0 : std::sqrt(sum / static_cast<double>(points.size()));
		for (auto const &one : out.curves)
			if (one.refused)
			{
				out.refused = true;
				char text[160];
				std::snprintf(text, sizeof(text), "R %.0f m < %.0f m", one.sharpest(), options.radius_least);
				out.error = text;
			}
		return out;
	}

	// the transitions, the cant and the speed of the curve K of the design as the norms ask for at the speed it's laid for: the highest at
	// which its radius and its transitions meet them, the line's where they do
	settled_curve settle(alignment::design const &Design, std::size_t const K, curve const &One, glm::dvec2 const &In, glm::dvec2 const &Out) const
	{
		auto const &norms{options.norms};
		settled_curve result;
		auto vertex{Design.vertices[K]};
		auto const sharpest{One.sharpest()};
		auto const starts_curved{K == 0 && Design.start_curved};
		auto const ends_curved{K + 1 == Design.vertices.size() && Design.end_curved};
		// the transitions of the norms at the speed the radius allows, where the fit has them about so long
		auto const allowed{allowed_speed(options.speed, sharpest, norms)};
		auto const snapped = [&](double const Fitted, double const Norm) { return Fitted <= 0.0 ? 0.0 : std::abs(Fitted - Norm) <= kTransitionSnap ? Norm : std::round(Fitted); };
		auto const sharpest_cant{alignment::recommend(allowed, sharpest, norms).cant};
		auto const end_cant = [&](double const Radius) { return sharpest_cant * sharpest / std::max(1.0, Radius); };
		if (false == starts_curved)
			vertex.transition_in = snapped(vertex.transition_in, alignment::transition_length(allowed, One.radii.front(), end_cant(One.radii.front()), norms));
		else
			vertex.transition_in = vertex.transition_in > 0.0 ? std::ceil(vertex.transition_in / 5.0 - 1e-9) * 5.0 : 0.0;
		if (false == ends_curved)
			vertex.transition_out = snapped(vertex.transition_out, alignment::transition_length(allowed, One.radii.back(), end_cant(One.radii.back()), norms));
		for (auto &arc : vertex.arcs)
			arc.transition = std::ceil(arc.transition / 5.0 - 1e-9) * 5.0;
		result.in = vertex.transition_in;
		result.out = vertex.transition_out;
		for (auto const &arc : vertex.arcs)
			result.between.push_back(arc.transition);
		result.refused = options.radius_least > 0.0 && sharpest < options.radius_least - 1e-6;
		// the transitions the cant is ramped along: in and out, one of them where the curve begins or ends within its arc
		auto ramp{std::min(starts_curved ? vertex.transition_out : vertex.transition_in, ends_curved ? vertex.transition_in : vertex.transition_out)};
		if (starts_curved && ends_curved)
			ramp = 0.0;
		else if (ramp <= 0.0)
			ramp = std::max(starts_curved ? 0.0 : vertex.transition_in, ends_curved ? 0.0 : vertex.transition_out);
		// the cant a held curve goes on with
		auto const keeps_cant{starts_curved && held.has_value() && std::abs(held->curvature) > 1e-12 && std::abs(One.radii.front() - 1.0 / std::abs(held->curvature)) <= 0.05 / std::abs(held->curvature)};
		auto speed{options.speed};
		for (;;)
		{
			auto const at{allowed_speed(speed, sharpest, norms)};
			auto laid{vertex};
			laid.cant = keeps_cant ? std::max(0.0, held->cant) : starts_curved && ends_curved ? alignment::recommend(at, sharpest, norms).cant : ramped_cant(alignment::recommend(at, sharpest, norms).cant, ramp, at, norms);
			laid.speed = speed < options.speed - 1e-6 ? speed : 0.0;
			auto trial{alone(Design, laid, In, Out, starts_curved, ends_curved)};
			if (ends_curved)
				trial.end_cant = laid.cant;
			if (starts_curved && false == held.has_value())
				trial.start_cant = laid.cant;
			auto meets{false};
			if (starts_curved && ends_curved)
			{
				// the line within the arc from end to end: its transitions lie beyond it, the radius and the cant tell
				auto const recommended{alignment::recommend(speed, sharpest, norms)};
				meets = sharpest >= recommended.radius_min - 1e-6 && alignment::unbalanced_acceleration(speed, sharpest, laid.cant, norms) <= norms.unbalanced + 1e-6 && laid.cant <= norms.cant_max + 1e-6;
			}
			else
			{
				auto const computed{alignment::compute(trial)};
				meets = computed.valid && false == computed.curves.empty() && alignment::meets_norms(computed.curves.front(), trial);
			}
			if (meets || speed <= options.speed_least + 1e-6)
			{
				result.cant = laid.cant;
				result.speed = laid.speed;
				break;
			}
			// the next speed down, a whole step
			auto const step{std::max(1.0, options.speed_step)};
			speed = std::max(options.speed_least, std::min(std::ceil(speed / step - 1e-9) * step - step, speed - 1e-6));
		}
		return result;
	}

	// the line ending at the end held
	result ended(hold const &End) const
	{
		auto State{states.back()};
		if (false == State.started)
		{
			// one point and the end: the straight from it
			if (points.size() == 1)
			{
				State.started = true;
				State.from.first = points.front();
				State.fit.shape.free = true;
				auto const way{End.position - points.front()};
				State.fit.parameters = {0.0, angle_of(way)};
			}
			else
				return State.line;
		}
		budget spend{options.budget};
		auto const indices{fitted_points(State)};
		std::vector<model> starts;
		if (State.fit.shape.curves.empty() || false == State.fit.shape.curves.back().open)
		{
			starts.push_back(State.fit);
			auto const more{into(State, End)};
			starts.insert(starts.end(), more.begin(), more.end());
		}
		else
		{
			auto const more{closed(State, End.position)};
			starts.insert(starts.end(), more.begin(), more.end());
		}
		std::vector<candidate> tried;
		for (auto const &guess : starts)
			tried.push_back(fitted(State, guess, spend, indices, End));
		// the end held weighs the most: the one ending at it
		std::optional<candidate> best;
		for (auto const &c : tried)
			if (c.valid && (false == best.has_value() || c.score() < best->score()))
				best = c;
		if (false == best.has_value())
			return State.line;
		State.fit = best->fit;
		auto line{built(State, End)};
		line.evaluations = spend.used;
		return line;
	}
};

tracer::tracer() : m_impl{new impl{settings{}, std::nullopt}}
{
}

tracer::tracer(settings const &Settings, std::optional<hold> const &Start) : m_impl{new impl{Settings, Start}}
{
}

tracer::tracer(tracer const &Other) : m_impl{new impl{*Other.m_impl}}
{
}

tracer &tracer::operator=(tracer const &Other)
{
	if (this != &Other)
	{
		auto *copy{new impl{*Other.m_impl}};
		delete m_impl;
		m_impl = copy;
	}
	return *this;
}

tracer::~tracer()
{
	delete m_impl;
}

void tracer::add(glm::dvec2 const &Point, double const Sigma)
{
	auto &self{*m_impl};
	auto const started{std::chrono::steady_clock::now()};
	self.points.push_back(Point);
	self.sigmas.push_back(std::max(0.05, Sigma));
	budget spend{self.options.budget};
	auto next{self.click(self.states.back(), spend)};
	next.line.evaluations = spend.used;
	next.line.budget_spent = spend.spent;
	next.line.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
	self.states.push_back(std::move(next));
}

void tracer::pop()
{
	auto &self{*m_impl};
	if (self.points.empty())
		return;
	self.points.pop_back();
	self.sigmas.pop_back();
	self.states.pop_back();
}

std::size_t tracer::size() const
{
	return m_impl->points.size();
}

std::vector<glm::dvec2> const &tracer::points() const
{
	return m_impl->points;
}

settings const &tracer::options() const
{
	return m_impl->options;
}

std::optional<hold> const &tracer::start() const
{
	return m_impl->held;
}

result const &tracer::now() const
{
	return m_impl->states.back().line;
}

result tracer::ended(hold const &End) const
{
	return m_impl->ended(End);
}

result traced(std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, settings const &Settings, std::optional<hold> const &Start)
{
	tracer line{Settings, Start};
	for (std::size_t i = 0; i < Points.size(); ++i)
		line.add(Points[i], i < Sigmas.size() ? Sigmas[i] : 0.1);
	return line.now();
}

spacing spacing_to(std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, std::vector<glm::dvec2> const &Neighbour)
{
	spacing result;
	if (Neighbour.size() < 2)
		return result;
	std::vector<double> offsets;
	double noise{0.0};
	for (std::size_t i = 0; i < Points.size(); ++i)
	{
		// square to the nearest segment of the neighbour the point lies abreast of
		auto best{std::numeric_limits<double>::max()};
		std::optional<double> offset;
		for (std::size_t k = 0; k + 1 < Neighbour.size(); ++k)
		{
			auto const along{Neighbour[k + 1] - Neighbour[k]};
			auto const length{glm::length(along)};
			if (length < 1e-9)
				continue;
			auto const t{along / length};
			auto const s{glm::dot(Points[i] - Neighbour[k], t)};
			if (s < -1e-6 || s > length + 1e-6)
				continue;
			auto const across{cross(t, Points[i] - Neighbour[k])};
			if (std::abs(across) < best)
			{
				best = std::abs(across);
				offset = across;
			}
		}
		if (false == offset.has_value())
			continue;
		offsets.push_back(*offset);
		noise += std::pow(i < Sigmas.size() ? Sigmas[i] : 0.1, 2.0);
	}
	result.points = offsets.size();
	if (offsets.size() < 3)
		return result;
	double mean{0.0};
	for (auto const offset : offsets)
		mean += offset;
	mean /= static_cast<double>(offsets.size());
	double spread{0.0};
	for (auto const offset : offsets)
		spread += (offset - mean) * (offset - mean);
	result.mean = mean;
	result.spread = std::sqrt(spread / static_cast<double>(offsets.size()));
	result.parallel = result.spread <= std::sqrt(noise / static_cast<double>(offsets.size())) * 1.5;
	return result;
}

} // namespace trace
