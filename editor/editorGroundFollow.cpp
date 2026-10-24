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

bool same(double const A, double const B)
{
	return A == B || (std::isnan(A) && std::isnan(B));
}

bool same(std::vector<double> const &A, std::vector<double> const &B)
{
	return A.size() == B.size() && std::equal(A.begin(), A.end(), B.begin(), [](double const X, double const Y) { return same(X, Y); });
}

bool same(ends const &A, ends const &B)
{
	return same(A.height0, B.height0) && same(A.grade0, B.grade0) && same(A.height1, B.height1) && same(A.grade1, B.grade1);
}

bool same(grading const &A, grading const &B)
{
	auto const &a{A.settings};
	auto const &b{B.settings};
	auto const &an{a.norms};
	auto const &bn{b.norms};
	auto const &af{A.fit};
	auto const &bf{B.fit};
	// the laying tells no points, humps nor bounds: a grading with any isn't taken for another
	return a.points.empty() && b.points.empty() && a.speed == b.speed && a.join_start == b.join_start && a.join_end == b.join_end && an.grade_max == bn.grade_max && an.grade_switch_max == bn.grade_switch_max &&
	       an.radius_factor == bn.radius_factor && an.radius_min == bn.radius_min && an.curve_threshold == bn.curve_threshold && an.element_min == bn.element_min && an.fixed_tolerance == bn.fixed_tolerance &&
	       af.tolerance == bf.tolerance && af.grade_max == bf.grade_max && af.follow_length == bf.follow_length && af.follow_height == bf.follow_height && af.follow_steepest == bf.follow_steepest &&
	       af.follow_tolerance == bf.follow_tolerance && af.humps.empty() && bf.humps.empty() && af.hump_radius == bf.hump_radius && af.bounds.empty() && bf.bounds.empty() && af.hold_grades == bf.hold_grades &&
	       A.reach == B.reach;
}

bool grade_line(request const &Asked, profile::line &Line)
{
	auto const Total{Asked.total};
	auto const &Ends{Asked.heights};
	auto const &Grading{Asked.grades};
	auto const none{std::numeric_limits<double>::quiet_NaN()};
	std::vector<double> chainages{0.0};
	std::vector<double> ground{none};
	chainages.insert(chainages.end(), Asked.chainages.begin(), Asked.chainages.end());
	ground.insert(ground.end(), Asked.targets.begin(), Asked.targets.end());
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

// the pieces cut where the grade of the line breaks and lifted onto it, the chainages of the line Scale times those of the
// pieces; with no line the heights lead from one end to the other
void lay_on(std::vector<segment_data> &Pieces, std::vector<double> const &Lengths, double const Total, ends const &Ends, profile::line const *Graded, double const Scale)
{
	std::vector<std::pair<double, double>> spans;
	if (Graded != nullptr)
	{
		auto knots{profile::breaks(*Graded)};
		for (auto &knot : knots)
			knot /= Scale;
		// the ends of the track go on in the height and the grade of the tracks they join, whatever the line found
		auto result{cut(Pieces, Lengths, knots, spans)};
		for (std::size_t i = 0; i < result.size(); ++i)
			lift(result[i], spans[i], [&](double const Chainage, double &Grade) {
				if (Chainage < 1e-6 && false == std::isnan(Ends.grade0))
				{
					Grade = Ends.grade0;
					return Ends.height0;
				}
				if (Chainage > Total - 1e-6 && false == std::isnan(Ends.grade1))
				{
					Grade = Ends.grade1;
					return Ends.height1;
				}
				Grade = profile::grade(*Graded, Chainage * Scale) * Scale;
				return profile::elevation(*Graded, Chainage * Scale);
			});
		Pieces = std::move(result);
		return;
	}
	knot_line line{{0.0, Total}, {Ends.height0, Ends.height1}, {}, Ends.grade0, Ends.grade1};
	line.refresh();
	auto result{cut(Pieces, Lengths, line.knots, spans)};
	for (std::size_t i = 0; i < result.size(); ++i)
		lift(result[i], spans[i], [&](double const Chainage, double &Grade) { return line.at(Chainage, Grade); });
	Pieces = std::move(result);
}

// the lengths of the pieces in plan and the ground sampled along them; returns the length of them all
double measure(std::vector<segment_data> const &Pieces, ends const &Ends, bed_query const &Bed, grading const &Grading, std::vector<double> &Lengths, request &Asked)
{
	double total{0.0};
	for (auto const &piece : Pieces)
		total += Lengths.emplace_back(bezier{piece}.plan_length(256));
	if (total < 1e-3)
		return total;
	auto points{sample(Pieces, Lengths, total, Ends)};
	Asked.targets = Bed(points.points, points.directions);
	Asked.chainages = std::move(points.chainages);
	Asked.total = total;
	Asked.heights = Ends;
	Asked.grades = Grading;
	return total;
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

bool request::operator==(request const &Other) const
{
	return total == Other.total && same(heights, Other.heights) && chainages == Other.chainages && same(targets, Other.targets) && same(grades, Other.grades);
}

fitter::~fitter()
{
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		m_stop = true;
	}
	m_wake.notify_all();
	if (m_thread.joinable())
		m_thread.join();
}

bool fitter::done(request const &Request, profile::line &Line, bool &Graded) const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	if (false == m_result.has_value() || false == (m_result->asked == Request))
		return false;
	Line = m_result->line;
	Graded = m_result->graded;
	return true;
}

bool fitter::latest(request &Request, profile::line &Line, bool &Graded) const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	if (false == m_result.has_value())
		return false;
	Request = m_result->asked;
	Line = m_result->line;
	Graded = m_result->graded;
	return true;
}

void fitter::ask(request Request)
{
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		if ((m_asked.has_value() && *m_asked == Request) || (m_fitting.has_value() && *m_fitting == Request) || (m_result.has_value() && m_result->asked == Request))
			return;
		m_asked = std::move(Request);
		if (false == m_thread.joinable())
			m_thread = std::thread{[this]() { work(); }};
	}
	m_wake.notify_one();
}

void fitter::work()
{
	std::unique_lock<std::mutex> lock{m_mutex};
	for (;;)
	{
		m_wake.wait(lock, [this]() { return m_stop || m_asked.has_value(); });
		if (m_stop)
			return;
		m_fitting = std::move(m_asked);
		m_asked.reset();
		result fitted{*m_fitting};
		lock.unlock();
		fitted.graded = grade_line(fitted.asked, fitted.line);
		lock.lock();
		m_result = std::move(fitted);
		m_fitting.reset();
	}
}

void follow(std::vector<segment_data> &Pieces, ends const &Ends, bed_query const &Bed, grading const &Grading, fitter const *Fitted)
{
	std::vector<double> lengths;
	request asked;
	auto const total{measure(Pieces, Ends, Bed, Grading, lengths, asked)};
	if (total < 1e-3)
		return;
	profile::line graded;
	bool found{false};
	if (Fitted == nullptr || false == Fitted->done(asked, graded, found))
		found = grade_line(asked, graded);
	lay_on(Pieces, lengths, total, Ends, found ? &graded : nullptr, 1.0);
}

void follow_meanwhile(std::vector<segment_data> &Pieces, ends const &Ends, bed_query const &Bed, grading const &Grading, fitter &Fitter)
{
	std::vector<double> lengths;
	request asked;
	auto const total{measure(Pieces, Ends, Bed, Grading, lengths, asked)};
	if (total < 1e-3)
		return;
	profile::line graded;
	bool found{false};
	if (Fitter.done(asked, graded, found))
	{
		lay_on(Pieces, lengths, total, Ends, found ? &graded : nullptr, 1.0);
		return;
	}
	// the cursor moves on a little from frame to frame: the line fitted for where it was a moment ago is near enough, as long
	// as the track goes on from the same end
	request last;
	auto const stretched{Fitter.latest(last, graded, found) && found && last.total > 1e-3 && same(last.heights.height0, Ends.height0) && same(last.heights.grade0, Ends.grade0)};
	Fitter.ask(std::move(asked));
	lay_on(Pieces, lengths, total, Ends, stretched ? &graded : nullptr, stretched ? last.total / total : 1.0);
}

void follow_points(std::vector<segment_data> &Pieces, std::vector<glm::dvec3> const &Points, ends const &Ends)
{
	std::vector<double> lengths;
	double total{0.0};
	for (auto const &piece : Pieces)
		total += lengths.emplace_back(bezier{piece}.plan_length(256));
	if (total < 1e-3)
		return;
	// the plan of the pieces as a line of short chords, each point of it with its chainage
	std::vector<glm::dvec2> line;
	std::vector<double> chainages;
	{
		double from{0.0};
		for (std::size_t i = 0; i < Pieces.size(); ++i)
		{
			bezier const curve{Pieces[i]};
			auto const steps{std::max(4, static_cast<int>(std::ceil(lengths[i] / kStep)))};
			for (int j = (i == 0 ? 0 : 1); j <= steps; ++j)
			{
				line.push_back(plan_of(curve.point(curve.parameter(lengths[i] * j / steps))));
				chainages.push_back(from + lengths[i] * j / steps);
			}
			from += lengths[i];
		}
	}
	auto const chainage_of = [&](glm::dvec2 const &Point) {
		auto best{std::numeric_limits<double>::max()};
		double chainage{0.0};
		for (std::size_t i = 0; i + 1 < line.size(); ++i)
		{
			auto const chord{line[i + 1] - line[i]};
			auto const length2{glm::dot(chord, chord)};
			auto const t{length2 > 1e-18 ? std::clamp(glm::dot(Point - line[i], chord) / length2, 0.0, 1.0) : 0.0};
			auto const offset{Point - (line[i] + chord * t)};
			if (auto const distance2{glm::dot(offset, offset)}; distance2 < best)
			{
				best = distance2;
				chainage = chainages[i] + (chainages[i + 1] - chainages[i]) * t;
			}
		}
		return chainage;
	};
	// the knots: the ends, and between them the points in their order along the track, none too near the one before
	std::vector<double> knots{0.0};
	std::vector<double> heights{Ends.height0};
	for (auto const &point : Points)
	{
		auto const chainage{chainage_of(plan_of(point))};
		if (chainage < knots.back() + 1.0 || chainage > total - 1.0)
			continue;
		knots.push_back(chainage);
		heights.push_back(point.y);
	}
	knots.push_back(total);
	heights.push_back(Ends.height1);
	auto const last{knots.size() - 1};
	// the height at the chainage on the stretch Middle is on, an end of it taken at a knot
	auto const at = [&](double const Chainage, double const Middle, double &Grade) {
		std::size_t stretch{0};
		while (stretch + 1 < last && Middle > knots[stretch + 1])
			++stretch;
		auto const span{knots[stretch + 1] - knots[stretch]};
		auto const even{(heights[stretch + 1] - heights[stretch]) / span};
		auto const from{stretch == 0 && false == std::isnan(Ends.grade0) ? Ends.grade0 : even};
		auto const to{stretch + 1 == last && false == std::isnan(Ends.grade1) ? Ends.grade1 : even};
		// a cubic of Hermite, a straight grade where both ends keep the even one
		auto const t{std::clamp((Chainage - knots[stretch]) / span, 0.0, 1.0)};
		auto const t2{t * t};
		auto const t3{t2 * t};
		Grade = ((6.0 * t2 - 6.0 * t) * heights[stretch] + (3.0 * t2 - 4.0 * t + 1.0) * from * span + (6.0 * t - 6.0 * t2) * heights[stretch + 1] + (3.0 * t2 - 2.0 * t) * to * span) / span;
		return (2.0 * t3 - 3.0 * t2 + 1.0) * heights[stretch] + (t3 - 2.0 * t2 + t) * from * span + (3.0 * t2 - 2.0 * t3) * heights[stretch + 1] + (t3 - t2) * to * span;
	};
	std::vector<std::pair<double, double>> spans;
	auto result{cut(Pieces, lengths, std::vector<double>(knots.begin() + 1, knots.end() - 1), spans)};
	for (std::size_t i = 0; i < result.size(); ++i)
	{
		auto const middle{(spans[i].first + spans[i].second) * 0.5};
		lift(result[i], spans[i], [&](double const Chainage, double &Grade) { return at(Chainage, middle, Grade); });
	}
	Pieces = std::move(result);
}

} // namespace ground_follow
