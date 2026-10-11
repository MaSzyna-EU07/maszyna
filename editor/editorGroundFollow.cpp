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
#include <iterator>

namespace ground_follow
{

namespace
{

using geometry::bezier;
using geometry::has_controls;
using geometry::part_of;
using geometry::plan_of;

double constexpr kStep{2.0};
double constexpr kTolerance{0.1};
double constexpr kSpacing{10.0};

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

void fit(knot_line &Line, std::vector<double> const &Chainages, std::vector<double> const &Targets)
{
	for (std::size_t pass = 0; pass < Chainages.size(); ++pass)
	{
		auto worst{kTolerance};
		auto pick{Chainages.size()};
		for (std::size_t j = 0; j < Chainages.size(); ++j)
		{
			if (std::isnan(Targets[j]))
				continue;
			double grade{0.0};
			auto const miss{std::abs(Line.at(Chainages[j], grade) - Targets[j])};
			if (miss <= worst || false == Line.room(Chainages[j], kSpacing))
				continue;
			worst = miss;
			pick = j;
		}
		if (pick == Chainages.size())
			return;
		Line.insert(Chainages[pick], Targets[pick]);
	}
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
			if (knot < cut + 0.5 || knot > to - 0.5)
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

void lift(segment_data &Piece, std::pair<double, double> const &Span, knot_line const &Line)
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
	start.y = Line.at(a, begin);
	finish.y = Line.at(b, end);
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

void knot_line::insert(double const Chainage, double const Height)
{
	auto const where{std::upper_bound(knots.begin(), knots.end(), Chainage) - knots.begin()};
	knots.insert(knots.begin() + where, Chainage);
	heights.insert(heights.begin() + where, Height);
	refresh();
}

double knot_line::at(double const Chainage, double &Grade) const
{
	auto const next{static_cast<std::size_t>(std::upper_bound(knots.begin(), knots.end(), Chainage) - knots.begin())};
	auto const i{std::clamp<std::size_t>(next, 1, knots.size() - 1) - 1};
	auto const length{knots[i + 1] - knots[i]};
	return geometry::hermite(length, heights[i], slopes[i], heights[i + 1], slopes[i + 1], std::clamp((Chainage - knots[i]) / length, 0.0, 1.0), Grade);
}

bool knot_line::room(double const Chainage, double const Spacing) const
{
	auto const next{std::upper_bound(knots.begin(), knots.end(), Chainage)};
	return next != knots.end() && *next - Chainage >= Spacing && Chainage - *std::prev(next) >= Spacing;
}

void follow(std::vector<segment_data> &Pieces, ends const &Ends, bed_query const &Bed)
{
	std::vector<double> lengths;
	double total{0.0};
	for (auto const &piece : Pieces)
		total += lengths.emplace_back(bezier{piece}.plan_length(256));
	if (total < 1e-3)
		return;
	auto const points{sample(Pieces, lengths, total, Ends)};
	auto const targets{Bed(points.points, points.directions)};
	knot_line line{{0.0, total}, {Ends.height0, Ends.height1}, {}, Ends.grade0, Ends.grade1};
	line.refresh();
	fit(line, points.chainages, targets);
	std::vector<std::pair<double, double>> spans;
	auto result{cut(Pieces, lengths, line.knots, spans)};
	for (std::size_t i = 0; i < result.size(); ++i)
		lift(result[i], spans[i], line);
	Pieces = std::move(result);
}

} // namespace ground_follow
