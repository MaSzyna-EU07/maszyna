/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorGroundFollow.hpp"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>

namespace ground_follow
{

namespace
{

using geometry::bezier;
using geometry::has_controls;
using geometry::part_of;
using geometry::plan_of;

double constexpr kStep{2.0};
double constexpr kShortest{5.0}; // m, the shortest piece the line cuts off

struct samples
{
	std::vector<double> chainages;
	std::vector<glm::dvec3> points;
	std::vector<glm::dvec2> directions;
};

samples sample(std::vector<segment_data> const &Pieces, std::vector<double> const &Lengths, double const Total, ends const &Ends)
{
	samples result;
	double reached{0.0};
	for (std::size_t i = 0; i < Pieces.size(); ++i)
	{
		bezier const curve{Pieces[i]};
		auto const count{std::max(1, static_cast<int>(std::ceil(Lengths[i] / kStep)))};
		for (int k = (i == 0 ? 1 : 0); k < count; ++k)
		{
			auto const along{Lengths[i] * k / count};
			auto const t{curve.parameter(along)};
			auto const chainage{reached + along};
			auto point{curve.point(t)};
			point.y = Ends.height0 + (Ends.height1 - Ends.height0) * chainage / Total + 50.0;
			result.chainages.push_back(chainage);
			result.points.push_back(point);
			result.directions.push_back(plan_of(curve.first(t)));
		}
		reached += Lengths[i];
	}
	return result;
}

bool grade_line(samples const &Points, std::vector<double> const &Targets, double const Total, ends const &Ends, grading const &Grading, profile::line &Line)
{
	auto const none{std::numeric_limits<double>::quiet_NaN()};
	std::vector<double> chainages{0.0};
	std::vector<double> ground{none};
	chainages.insert(chainages.end(), Points.chainages.begin(), Points.chainages.end());
	ground.insert(ground.end(), Targets.begin(), Targets.end());
	chainages.push_back(Total);
	ground.push_back(none);
	if (std::none_of(ground.begin(), ground.end(), [](double const Height) { return std::isfinite(Height); }))
		return false;
	profile::context context;
	context.length = Total;
	context.start_elevation = Ends.height0;
	context.end_elevation = Ends.height1;
	context.start_joined = false == std::isnan(Ends.grade0);
	context.end_joined = false == std::isnan(Ends.grade1);
	context.start_grade = context.start_joined ? Ends.grade0 : 0.0;
	context.end_grade = context.end_joined ? Ends.grade1 : 0.0;
	// the start of the track stays where it's laid from, a free end goes where the line takes it
	auto fit{Grading.fit};
	if (false == context.start_joined)
		fit.bounds.push_back({0.0, Ends.height0, Ends.height0});
	auto settings{Grading.settings};
	settings.join_start = true;
	settings.join_end = true;
	Line = profile::fit_ground(chainages, ground, context, settings, fit, {}, std::vector<double>(chainages.size(), Grading.reach));
	if (Line.points.size() < 2)
		return false;
	profile::complete(Line, context);
	if (false == context.start_joined)
		Line.points.front().elevation = Ends.height0;
	return true;
}

std::vector<segment_data> cut(std::vector<segment_data> const &Pieces, std::vector<double> const &Lengths, std::vector<double> const &Knots, std::vector<std::pair<double, double>> &Spans)
{
	std::vector<segment_data> result;
	double from{0.0};
	for (std::size_t i = 0; i < Pieces.size(); ++i)
	{
		bezier const curve{Pieces[i]};
		auto const to{from + Lengths[i]};
		double cut{from};
		double parameter{0.0};
		for (auto const knot : Knots)
		{
			if (knot < cut + kShortest || knot > to - kShortest)
				continue;
			auto const next{curve.parameter(knot - from)};
			result.push_back(part_of(Pieces[i], parameter, next));
			Spans.emplace_back(cut, knot);
			cut = knot;
			parameter = next;
		}
		result.push_back(parameter > 0.0 ? part_of(Pieces[i], parameter, 1.0) : Pieces[i]);
		Spans.emplace_back(cut, to);
		from = to;
	}
	return result;
}

void lift(segment_data &Piece, std::pair<double, double> const &Span, std::function<double(double, double &)> const &At)
{
	auto &points{Piece.points};
	auto const [a, b]{Span};
	auto const length{b - a};
	double begin{0.0};
	double end{0.0};
	auto &start{points[segment_data::point::start]};
	auto &finish{points[segment_data::point::end]};
	auto &control1{points[segment_data::point::control1]};
	auto &control2{points[segment_data::point::control2]};
	start.y = At(a, begin);
	finish.y = At(b, end);
	if (has_controls(Piece))
	{
		control1.y = begin * glm::length(plan_of(control1));
		control2.y = -end * glm::length(plan_of(control2));
		return;
	}
	auto const linear{(finish.y - start.y) / std::max(length, 1e-6)};
	if (length < 1e-3 || (std::abs(begin - linear) < 1e-6 && std::abs(end - linear) < 1e-6))
		return;
	auto const along{glm::normalize(plan_of(finish - start)) * (length / 3.0)};
	control1 = {along.x, begin * length / 3.0, along.y};
	control2 = {-along.x, -end * length / 3.0, -along.y};
}

} // namespace

void knot_line::refresh()
{
	auto const last{knots.size() - 1};
	slopes.assign(knots.size(), 0.0);
	for (std::size_t i = 0; i <= last; ++i)
	{
		auto const a{i == 0 ? 0 : i - 1};
		auto const b{i == last ? last : i + 1};
		slopes[i] = (heights[b] - heights[a]) / (knots[b] - knots[a]);
	}
	if (false == std::isnan(grade0))
		slopes.front() = grade0;
	if (false == std::isnan(grade1))
		slopes.back() = grade1;
}

double knot_line::at(double const Chainage, double &Grade) const
{
	auto const next{static_cast<std::size_t>(std::upper_bound(knots.begin(), knots.end(), Chainage) - knots.begin())};
	auto const i{std::clamp<std::size_t>(next, 1, knots.size() - 1) - 1};
	auto const length{knots[i + 1] - knots[i]};
	return geometry::hermite(length, heights[i], slopes[i], heights[i + 1], slopes[i + 1], std::clamp((Chainage - knots[i]) / length, 0.0, 1.0), Grade);
}

void follow(std::vector<segment_data> &Pieces, ends const &Ends, bed_query const &Bed, grading const &Grading)
{
	std::vector<double> lengths;
	double total{0.0};
	for (auto const &piece : Pieces)
		total += lengths.emplace_back(bezier{piece}.plan_length(256));
	if (total < 1e-3)
		return;
	auto const points{sample(Pieces, lengths, total, Ends)};
	auto const targets{Bed(points.points, points.directions)};
	std::vector<std::pair<double, double>> spans;
	if (profile::line graded; grade_line(points, targets, total, Ends, Grading, graded))
	{
		// the ends of the track go on in the height and the grade of the tracks they join, whatever the line found
		auto result{cut(Pieces, lengths, profile::breaks(graded), spans)};
		for (std::size_t i = 0; i < result.size(); ++i)
			lift(result[i], spans[i], [&](double const Chainage, double &Grade) {
				if (Chainage < 1e-6 && false == std::isnan(Ends.grade0))
				{
					Grade = Ends.grade0;
					return Ends.height0;
				}
				if (Chainage > total - 1e-6 && false == std::isnan(Ends.grade1))
				{
					Grade = Ends.grade1;
					return Ends.height1;
				}
				Grade = profile::grade(graded, Chainage);
				return profile::elevation(graded, Chainage);
			});
		Pieces = std::move(result);
		return;
	}
	knot_line line{{0.0, total}, {Ends.height0, Ends.height1}, {}, Ends.grade0, Ends.grade1};
	line.refresh();
	auto result{cut(Pieces, lengths, line.knots, spans)};
	for (std::size_t i = 0; i < result.size(); ++i)
		lift(result[i], spans[i], [&](double const Chainage, double &Grade) { return line.at(Chainage, Grade); });
	Pieces = std::move(result);
}

} // namespace ground_follow
