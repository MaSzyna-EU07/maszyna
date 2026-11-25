/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorTraceStation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "editor/editorGeometry.hpp"

namespace trace_station
{

namespace
{

double constexpr kBeyond{12.0}; // m of the straight beyond the shortest turnout its points tell the shapes apart by
double constexpr kCap{16.0}; // a point farther off than 4 sigma counts as one 4 sigma off
double constexpr kGain{9.0}; // a point within 3 sigma of a turnout explains it, one farther off counts against it
double constexpr kPrefer{2.0}; // the preferred shape where two fit alike
double constexpr kOffMain{3.0}; // sigmas off the main track a point is to tell the shape
double constexpr kCoarse{1.0}; // m, the steps the start is sought in, then refined
double constexpr kFine{0.05};
double constexpr kSigma{0.1}; // m, the noise of a point given none
double constexpr kAnchor{3.0}; // m, how far from the first point clicked on the main track the start of the turnout is, as a sigma

double cross(glm::dvec2 const &A, glm::dvec2 const &B)
{
	return A.x * B.y - A.y * B.x;
}

glm::dvec2 left(glm::dvec2 const &V)
{
	return {-V.y, V.x};
}

double to_segment(glm::dvec2 const &P, glm::dvec2 const &A, glm::dvec2 const &B)
{
	auto const ab{B - A};
	auto const length2{glm::dot(ab, ab)};
	auto const t{length2 > 1e-12 ? std::clamp(glm::dot(P - A, ab) / length2, 0.0, 1.0) : 0.0};
	return glm::distance(P, A + ab * t);
}

glm::dvec2 end_heading(turnout_shape const &Shape)
{
	auto const &line{Shape.diverging};
	if (line.size() < 2)
		return {1.0, 0.0};
	auto const last{line.back() - line[line.size() - 2]};
	return glm::length(last) > 1e-9 ? glm::normalize(last) : glm::dvec2{1.0, 0.0};
}

// of a point in the frame of the turnout from its diverging track, and from the straight going on from its end. The track goes on along x,
// so the segments abreast of the point are found by halving
double off_diverging(turnout_shape const &Shape, glm::dvec2 const &P)
{
	auto const &line{Shape.diverging};
	if (line.empty())
		return std::numeric_limits<double>::max();
	auto best{std::numeric_limits<double>::max()};
	auto const above{std::upper_bound(line.begin(), line.end(), P.x, [](double const X, glm::dvec2 const &Each) { return X < Each.x; })};
	auto const at{static_cast<std::ptrdiff_t>(above - line.begin())};
	auto const count{static_cast<std::ptrdiff_t>(line.size())};
	for (auto i{std::max<std::ptrdiff_t>(0, at - 3)}; i + 1 < count && i <= at + 2; ++i)
		best = std::min(best, to_segment(P, line[i], line[i + 1]));
	if (count == 1)
		best = glm::distance(P, line.front());
	auto const heading{end_heading(Shape)};
	if (glm::dot(P - line.back(), heading) > 0.0)
		best = std::min(best, std::abs(cross(heading, P - line.back())));
	return best;
}

struct trial
{
	double score{std::numeric_limits<double>::max()};
	double start{0.0};
	std::size_t told{0};
	double rms{0.0};
};

// the end of the diverging track of the shape put at Start the way and to the side given; the points along it counted
void place_end(straight const &Main, turnout_shape const &Shape, std::vector<glm::dvec2> const &Points, turnout_fit &Fit)
{
	auto const x{Main.direction * static_cast<double>(Fit.direction)};
	auto const y{left(x) * static_cast<double>(Fit.side)};
	auto const origin{Main.point(Fit.start)};
	auto const &end{Shape.diverging.back()};
	auto const heading{end_heading(Shape)};
	Fit.end = origin + x * end.x + y * end.y;
	Fit.heading = glm::normalize(x * heading.x + y * heading.y);
	Fit.curvature = Shape.end_curvature * static_cast<double>(Fit.side);
	// the points along the turnout itself: the line goes on from its end with the ones after them
	Fit.used = std::min<std::size_t>(1, Points.size());
	while (Fit.used < Points.size() && glm::dot(Points[Fit.used] - origin, x) <= Shape.length + 0.5)
		++Fit.used;
}

} // namespace

double straight::along(glm::dvec2 const &Point) const
{
	return glm::dot(Point - origin, direction);
}

double straight::lateral(glm::dvec2 const &Point) const
{
	return cross(direction, Point - origin);
}

glm::dvec2 straight::point(double const Along, double const Lateral) const
{
	return origin + direction * Along + left(direction) * Lateral;
}

turnout_shape shape_of(segment_data const &Diverging, double const Length, double const EndCurvature, double const Step)
{
	turnout_shape result;
	result.length = Length;
	result.end_curvature = EndCurvature;
	// in the frame of the fit: along the main track, to the side it diverges to
	auto const framed = [](glm::dvec3 const &Point) { return glm::dvec2{Point.z, Point.x}; };
	std::vector<segment_data> pieces{Diverging.pieces};
	if (pieces.empty())
		pieces.push_back(Diverging);
	for (auto const &piece : pieces)
	{
		geometry::bezier const curve{piece};
		auto const steps{std::clamp(static_cast<int>(std::ceil(curve.plan_length() / std::max(0.05, Step))), 1, 2000)};
		for (int i = result.diverging.empty() ? 0 : 1; i <= steps; ++i)
		{
			auto const point{framed(curve.point(static_cast<double>(i) / steps))};
			// x grows along the track: a point going back, a rounding at a joint of the pieces, is left out
			if (false == result.diverging.empty() && point.x <= result.diverging.back().x)
				continue;
			result.diverging.push_back(point);
		}
	}
	return result;
}

turnout_fit fit_turnout(straight const &Main, std::vector<turnout_shape> const &Shapes, std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, int const Preferred)
{
	turnout_fit result;
	if (Points.size() < 2 || Shapes.empty())
		return result;
	auto const sigma = [&](std::size_t const Index) { return std::max(Index < Sigmas.size() ? Sigmas[Index] : kSigma, 0.01); };
	auto const clicked{Main.along(Points.front())};
	// the first point on the main track is where the turnout starts, give or take a few metres; one off it is a point of its diverging track,
	// told apart from the main track as the others are, the start anywhere within the length of the turnout
	auto const anchored{std::abs(Main.lateral(Points.front())) <= kOffMain * sigma(0)};
	auto const preferred{Preferred >= 0 && Preferred < static_cast<int>(Shapes.size()) && Shapes[Preferred].diverging.size() >= 2 ? Preferred : 0};
	// a start of the shape, its direction and side: what the points make of it. The points along it and along the straight just beyond it
	// count, the ones farther on lie on whatever the line goes on with: a curve, as often as not. A longer shape so has more points to
	// explain, which it gains by where it does and loses by where it doesn't
	auto const judge = [&](int const Shape, int const Direction, int const Side, double const Start) {
		trial trial;
		trial.start = Start;
		auto const &shape{Shapes[Shape]};
		auto const x{Main.direction * static_cast<double>(Direction)};
		auto const y{left(x) * static_cast<double>(Side)};
		auto const origin{Main.point(Start)};
		double score{0.0};
		double squares{0.0};
		for (std::size_t i = anchored ? 1 : 0; i < Points.size(); ++i)
		{
			glm::dvec2 const local{glm::dot(Points[i] - origin, x), glm::dot(Points[i] - origin, y)};
			if (local.x > shape.length + kBeyond)
				continue;
			auto const s{sigma(i)};
			auto const main{std::abs(local.y) / s};
			auto const diverging{off_diverging(shape, local) / s};
			auto const off{std::min(main, diverging)};
			score += std::min(off * off, kCap) - kGain;
			if (main > kOffMain)
			{
				++trial.told;
				squares += diverging * s * diverging * s;
			}
		}
		// the start kept near the first point clicked, more so where it was clicked on the main track
		auto const drift{(Start - clicked) / (anchored ? kAnchor : std::max(shape.length, 1.0))};
		trial.score = score + drift * drift - (Shape == preferred ? kPrefer : 0.0);
		trial.rms = trial.told > 0 ? std::sqrt(squares / static_cast<double>(trial.told)) : 0.0;
		return trial;
	};
	trial best;
	int bestshape{-1};
	int bestdirection{1};
	int bestside{1};
	for (int shape = 0; shape < static_cast<int>(Shapes.size()); ++shape)
	{
		if (Shapes[shape].diverging.size() < 2 || Shapes[shape].length <= 0.0)
			continue;
		auto const length{Shapes[shape].length};
		for (int const direction : {1, -1})
			for (int const side : {1, -1})
			{
				// coarse along the track either way of the first point, then fine about the best of it
				trial found;
				for (double u = -length; u <= length + 1e-9; u += kCoarse)
					if (auto const each{judge(shape, direction, side, clicked + u * direction)}; each.score < found.score)
						found = each;
				auto const around{found.start};
				for (double u = -kCoarse; u <= kCoarse + 1e-9; u += kFine)
					if (auto const each{judge(shape, direction, side, around + u)}; each.score < found.score)
						found = each;
				if (found.score < best.score)
				{
					best = found;
					bestshape = shape;
					bestdirection = direction;
					bestside = side;
				}
			}
	}
	if (bestshape < 0)
		return result;
	result.told = best.told;
	// no point off the main track yet: the preferred shape at the first point, the way and to the side the next point goes
	if (best.told == 0)
	{
		auto const next{Points[1] - Main.point(clicked)};
		bestshape = preferred;
		bestdirection = glm::dot(next, Main.direction) >= 0.0 ? 1 : -1;
		bestside = cross(Main.direction * static_cast<double>(bestdirection), next) >= 0.0 ? 1 : -1;
		best.start = clicked;
		best.rms = 0.0;
		result.preferred = true;
	}
	result.valid = true;
	result.shape = bestshape;
	result.start = best.start;
	result.direction = bestdirection;
	result.side = bestside;
	result.rms = best.rms;
	place_end(Main, Shapes[bestshape], Points, result);
	return result;
}

turnout_fit turnout_at(straight const &Main, std::vector<turnout_shape> const &Shapes, turnout_fit const &Fit, double const Start, std::vector<glm::dvec2> const &Points)
{
	auto result{Fit};
	if (false == Fit.valid || Fit.shape < 0 || Fit.shape >= static_cast<int>(Shapes.size()) || Shapes[Fit.shape].diverging.size() < 2)
		return result;
	result.start = Start;
	place_end(Main, Shapes[Fit.shape], Points, result);
	return result;
}

double round_spacing(double const Measured)
{
	auto const sign{Measured < 0.0 ? -1.0 : 1.0};
	auto const size{std::abs(Measured)};
	auto nearest{-1.0};
	for (double const usual : {4.0, 4.5, 4.75, 5.0})
		if (std::abs(size - usual) <= 0.1 + 1e-9 && (nearest < 0.0 || std::abs(size - usual) < std::abs(size - nearest)))
			nearest = usual;
	return sign * (nearest >= 0.0 ? nearest : std::round(size / 0.05) * 0.05);
}

abreast abreast_of(std::vector<glm::dvec2> const &Axis, glm::dvec2 const &Point)
{
	// the nearest point of the line: on the outside of a bend of it a point may lie abreast of neither segment meeting there, but of the
	// point where they meet. Beyond the ends of the line it's abreast of none
	abreast result;
	auto best{std::numeric_limits<double>::max()};
	double travelled{0.0};
	std::size_t const last{Axis.size() >= 2 ? Axis.size() - 2 : 0};
	for (std::size_t k = 0; k + 1 < Axis.size(); ++k)
	{
		auto const along{Axis[k + 1] - Axis[k]};
		auto const length{glm::length(along)};
		if (length < 1e-9)
			continue;
		auto const t{along / length};
		auto const raw{glm::dot(Point - Axis[k], t)};
		auto const s{std::clamp(raw, 0.0, length)};
		auto const foot{Axis[k] + t * s};
		if (auto const distance{glm::distance(Point, foot)}; distance < best)
		{
			best = distance;
			result.found = (k > 0 || raw >= -1e-6) && (k < last || raw <= length + 1e-6);
			result.along = travelled + s;
			result.foot = foot;
			result.direction = t;
			result.lateral = (cross(t, Point - foot) >= 0.0 ? 1.0 : -1.0) * distance;
		}
		travelled += length;
	}
	return result;
}

double off_line(std::vector<glm::dvec2> const &Line, glm::dvec2 const &Point)
{
	if (Line.size() == 1)
		return glm::distance(Line.front(), Point);
	auto best{std::numeric_limits<double>::max()};
	for (std::size_t k = 0; k + 1 < Line.size(); ++k)
		best = std::min(best, to_segment(Point, Line[k], Line[k + 1]));
	return best;
}

parallel_run find_parallel(std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, std::vector<glm::dvec2> const &Neighbour, double const Least)
{
	parallel_run result;
	auto const count{Points.size()};
	std::vector<abreast> at(count);
	for (std::size_t i = 0; i < count; ++i)
		at[i] = abreast_of(Neighbour, Points[i]);
	auto const sigma = [&](std::size_t const Index) { return std::max(Index < Sigmas.size() ? Sigmas[Index] : kSigma, 0.01); };
	std::size_t bestcount{0};
	double bestlength{0.0};
	for (std::size_t first = 0; first < count; ++first)
		for (std::size_t last = first + 2; last < count; ++last)
		{
			if (false == at[last].found || false == at[first].found)
				break;
			double mean{0.0};
			for (auto i{first}; i <= last; ++i)
				mean += at[i].lateral;
			mean /= static_cast<double>(last - first + 1);
			// on the track itself, or across it, is no run alongside
			if (std::abs(mean) < 1.0)
				continue;
			bool keeps{true};
			for (auto i{first}; i <= last && keeps; ++i)
				keeps = at[i].found && std::abs(at[i].lateral - mean) <= 2.5 * sigma(i) + 0.02;
			if (false == keeps)
				break;
			auto const length{std::abs(at[last].along - at[first].along)};
			auto const points{last - first + 1};
			if (length < Least || points < bestcount || (points == bestcount && length <= bestlength))
				continue;
			bestcount = points;
			bestlength = length;
			result.found = true;
			result.first = first;
			result.last = last;
			result.measured = mean;
		}
	if (result.found)
		result.spacing = round_spacing(result.measured);
	return result;
}

glm::dvec2 alongside(std::vector<glm::dvec2> const &Neighbour, glm::dvec2 const &Point, double const Spacing)
{
	auto const at{abreast_of(Neighbour, Point)};
	return at.found ? at.foot + left(at.direction) * Spacing : Point;
}

joining_arc join_alongside(glm::dvec2 const &From, glm::dvec2 const &Heading, straight const &Alongside, std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, double const Preferred)
{
	joining_arc result;
	auto const heading{glm::normalize(Heading)};
	auto direction{Alongside.direction};
	if (glm::dot(direction, heading) < 0.0)
		direction = -direction;
	// where the straight from From meets the one alongside, and the angle they bend by there
	auto const across{cross(heading, direction)};
	if (std::abs(across) < std::sin(0.002))
		return result;
	auto const ahead{cross(Alongside.origin - From, direction) / across};
	if (ahead <= 0.0)
		return result;
	auto const vertex{From + heading * ahead};
	auto const bend{std::acos(std::clamp(glm::dot(heading, direction), -1.0, 1.0))};
	auto const most{ahead / std::tan(bend * 0.5)};
	if (most < 50.0)
		return result;
	auto const turn{across > 0.0 ? 1.0 : -1.0};
	// the line of the arc of the radius: the straight to it, the arc sampled every metre or so, the straight alongside on from it
	auto const line_of = [&](double const Radius) {
		auto const tangent{Radius * std::tan(bend * 0.5)};
		auto const first{vertex - heading * tangent};
		auto const centre{first + left(heading) * (Radius * turn)};
		std::vector<glm::dvec2> line{From, first};
		auto const steps{std::clamp(static_cast<int>(std::ceil(Radius * bend)), 2, 2000)};
		for (int i = 1; i <= steps; ++i)
		{
			auto const angle{bend * i / steps * turn};
			auto const radial{first - centre};
			line.push_back(centre + glm::dvec2{radial.x * std::cos(angle) - radial.y * std::sin(angle), radial.x * std::sin(angle) + radial.y * std::cos(angle)});
		}
		line.push_back(line.back() + direction * 2000.0);
		return line;
	};
	auto const score = [&](double const Radius, double &Rms) {
		auto const line{line_of(Radius)};
		double sum{0.0};
		double squares{0.0};
		for (std::size_t i = 0; i < Points.size(); ++i)
		{
			auto const off{off_line(line, Points[i])};
			auto const s{std::max(i < Sigmas.size() ? Sigmas[i] : kSigma, 0.01)};
			sum += std::min((off / s) * (off / s), kCap);
			squares += off * off;
		}
		Rms = Points.empty() ? 0.0 : std::sqrt(squares / static_cast<double>(Points.size()));
		// the radius preferred where the points tell no other one
		return sum + 0.01 * std::abs(std::log(Radius / std::max(1.0, Preferred)));
	};
	auto const least{std::min(100.0, most)};
	auto best{std::numeric_limits<double>::max()};
	if (Points.empty())
		result.radius = std::min(Preferred > 0.0 ? Preferred : most, most);
	else
		for (int i = 0; i <= 200; ++i)
		{
			auto const radius{least * std::pow(most / least, i / 200.0)};
			double rms{0.0};
			if (auto const each{score(radius, rms)}; each < best)
			{
				best = each;
				result.radius = radius;
				result.rms = rms;
			}
		}
	result.valid = result.radius > 0.0;
	result.vertex = vertex;
	return result;
}

head_place place_in_head(double const From, double const Length, double const Low, double const High, bool const Before, bool const After, double const Least)
{
	head_place result;
	auto const lowest{Low + (Before ? Least : 0.0)};
	auto const highest{High - Length - (After ? Least : 0.0)};
	result.from = From;
	if (lowest > highest + 1e-9)
		return result;
	result.fits = true;
	result.from = std::clamp(From, lowest, highest);
	result.moved = result.from - From;
	return result;
}

} // namespace trace_station
