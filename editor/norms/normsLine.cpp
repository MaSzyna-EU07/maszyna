/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/norms/norms.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

// a line read from its samples as elements: where the curvature keeps still an arc or a straight, where it changes evenly a
// transition, a jump at a joint where it changes at once. The pieces are found by keeping the samples a straight line between two kept
// ones misses by more than a tolerance (Douglas and Peucker), the same for the cant and for the grade
namespace norms
{

namespace
{

double constexpr kSame{1e-6}; // m
double constexpr kJumpCurvature{2e-5}; // 1/m: a smaller change at a joint, about R 50 km, is taken for the curves meeting not quite
double constexpr kStraight{1.0 / 30000.0}; // 1/m: flatter is a straight
double constexpr kCurvatureTolerance{1e-5}; // 1/m, and 3 % of the curvature
double constexpr kCurvatureShare{0.03};
double constexpr kJumpCant{1.0}; // mm
double constexpr kCantTolerance{0.5}; // mm
double constexpr kJumpGrade{0.0005};
double constexpr kGradeTolerance{0.0002};
double constexpr kPrecision{0.002}; // of the values measured on a line read from its samples

using value_of = double (*)(sample const &);

double curvature_of(sample const &Sample)
{
	return Sample.curvature;
}

double cant_of(sample const &Sample)
{
	return Sample.cant;
}

double grade_of(sample const &Sample)
{
	return Sample.grade;
}

// the runs of samples with no jump between them: [first, last] each
std::vector<std::pair<std::size_t, std::size_t>> runs(std::vector<sample> const &Samples, value_of const Value, double const Jump, bool const Features)
{
	std::vector<std::pair<std::size_t, std::size_t>> result;
	std::size_t first{0};
	for (std::size_t i = 1; i < Samples.size(); ++i)
	{
		auto const &before{Samples[i - 1]};
		auto const &after{Samples[i]};
		if (after.chainage - before.chainage > kSame)
			continue;
		if (auto const jump{std::abs(Value(after) - Value(before)) > Jump || (Features && after.features != before.features)}; false == jump)
			continue;
		result.emplace_back(first, i - 1);
		first = i;
	}
	result.emplace_back(first, Samples.size() - 1);
	return result;
}

// the samples kept of the run: the ones the line between the kept ones would miss by more than the tolerance
std::vector<std::size_t> simplified(std::vector<sample> const &Samples, std::pair<std::size_t, std::size_t> const Run, value_of const Value, double const Tolerance)
{
	std::vector<std::size_t> kept{Run.first, Run.second};
	std::vector<std::pair<std::size_t, std::size_t>> pending{Run};
	while (false == pending.empty())
	{
		auto const [first, last]{pending.back()};
		pending.pop_back();
		if (last <= first + 1)
			continue;
		auto const &a{Samples[first]};
		auto const &b{Samples[last]};
		auto const run{b.chainage - a.chainage};
		double furthest{0.0};
		std::size_t at{first};
		for (auto i = first + 1; i < last; ++i)
		{
			auto const share{run > kSame ? (Samples[i].chainage - a.chainage) / run : 0.5};
			auto const miss{std::abs(Value(Samples[i]) - std::lerp(Value(a), Value(b), share))};
			if (miss > furthest)
			{
				furthest = miss;
				at = i;
			}
		}
		if (furthest <= Tolerance)
			continue;
		kept.push_back(at);
		pending.emplace_back(first, at);
		pending.emplace_back(at, last);
	}
	std::ranges::sort(kept);
	auto const repeated{std::ranges::unique(kept)};
	kept.erase(repeated.begin(), repeated.end());
	return kept;
}

// the mean of the value over the samples, weighted by the length
double mean(std::vector<sample> const &Samples, std::size_t const First, std::size_t const Last, value_of const Value)
{
	double sum{0.0};
	double length{0.0};
	for (auto i = First + 1; i <= Last; ++i)
	{
		auto const run{Samples[i].chainage - Samples[i - 1].chainage};
		sum += (Value(Samples[i]) + Value(Samples[i - 1])) * 0.5 * run;
		length += run;
	}
	return length > kSame ? sum / length : Value(Samples[First]);
}

struct piece
{
	std::size_t first;
	std::size_t last;
	std::size_t run;
};

element element_of(std::vector<sample> const &Samples, piece const &Piece)
{
	auto const &a{Samples[Piece.first]};
	auto const &b{Samples[Piece.last]};
	element result;
	result.start = a.chainage;
	result.length = b.chainage - a.chainage;
	result.from = a.at;
	result.to = b.at;
	result.features = a.features;
	double sharpest{0.0};
	for (auto i = Piece.first; i <= Piece.last; ++i)
	{
		sharpest = std::max(sharpest, std::abs(Samples[i].curvature));
		result.speed = std::max(result.speed, Samples[i].speed);
	}
	auto const tolerance{std::max(kCurvatureTolerance, kCurvatureShare * std::max(std::abs(a.curvature), std::abs(b.curvature)))};
	if (sharpest < kStraight)
		result.kind = element_kind::straight;
	else if (std::abs(b.curvature - a.curvature) <= tolerance && a.curvature * b.curvature > 0.0)
	{
		result.kind = element_kind::arc;
		result.curvature_start = mean(Samples, Piece.first, Piece.last, &curvature_of);
		result.curvature_end = result.curvature_start;
	}
	else
	{
		result.kind = element_kind::transition;
		result.curvature_start = std::abs(a.curvature) < kStraight ? 0.0 : a.curvature;
		result.curvature_end = std::abs(b.curvature) < kStraight ? 0.0 : b.curvature;
	}
	return result;
}

// the neighbours alike made one: straights, and arcs of about one curvature
bool alike(element const &A, element const &B)
{
	if (A.kind != B.kind || A.features != B.features)
		return false;
	if (A.kind == element_kind::straight)
		return true;
	if (A.kind != element_kind::arc)
		return false;
	return A.curvature_end * B.curvature_start > 0.0 && std::abs(A.curvature_end - B.curvature_start) <= kCurvatureShare * std::max(std::abs(A.curvature_end), std::abs(B.curvature_start));
}

void merge(element &Into, element const &Next)
{
	if (Into.kind == element_kind::arc)
	{
		auto const total{Into.length + Next.length};
		auto const curvature{total > kSame ? (Into.curvature_start * Into.length + Next.curvature_start * Next.length) / total : Into.curvature_start};
		Into.curvature_start = curvature;
		Into.curvature_end = curvature;
	}
	Into.length = Next.end() - Into.start;
	Into.to = Next.to;
	Into.speed = std::max(Into.speed, Next.speed);
}

// where two elements meet with no jump the curvature goes on: the transition ends at the curvature of the arc or the straight
void join(element &Before, element &After)
{
	if (Before.kind == element_kind::transition && After.kind != element_kind::transition)
		Before.curvature_end = After.curvature_start;
	else if (After.kind == element_kind::transition && Before.kind != element_kind::transition)
		After.curvature_start = Before.curvature_end;
	else if (Before.kind == element_kind::transition)
	{
		auto const middle{(Before.curvature_end + After.curvature_start) * 0.5};
		Before.curvature_end = middle;
		After.curvature_start = middle;
	}
}

std::vector<element> plan(std::vector<sample> const &Samples)
{
	std::vector<element> result;
	std::vector<std::size_t> run_of;
	auto const all{runs(Samples, &curvature_of, kJumpCurvature, true)};
	for (std::size_t r = 0; r < all.size(); ++r)
	{
		double sharpest{0.0};
		for (auto i = all[r].first; i <= all[r].second; ++i)
			sharpest = std::max(sharpest, std::abs(Samples[i].curvature));
		auto const kept{simplified(Samples, all[r], &curvature_of, std::max(kCurvatureTolerance, kCurvatureShare * sharpest))};
		for (std::size_t k = 1; k < kept.size(); ++k)
		{
			if (Samples[kept[k]].chainage - Samples[kept[k - 1]].chainage <= kSame)
				continue;
			auto next{element_of(Samples, {kept[k - 1], kept[k], r})};
			if (false == result.empty() && run_of.back() == r && alike(result.back(), next))
			{
				merge(result.back(), next);
				continue;
			}
			result.push_back(next);
			run_of.push_back(r);
		}
	}
	for (std::size_t i = 1; i < result.size(); ++i)
		if (run_of[i] == run_of[i - 1])
			join(result[i - 1], result[i]);
	// the length of an element cut by the end of the line or by a switch isn't its own
	if (false == result.empty())
	{
		result.front().open_start = true;
		result.back().open_end = true;
	}
	for (std::size_t i = 1; i < result.size(); ++i)
		if ((result[i].features & feature::switch_track) != (result[i - 1].features & feature::switch_track))
		{
			result[i - 1].open_end = true;
			result[i].open_start = true;
		}
	return result;
}

std::vector<cant_point> cant(std::vector<sample> const &Samples)
{
	std::vector<cant_point> result;
	for (auto const &run : runs(Samples, &cant_of, kJumpCant, false))
		for (auto const index : simplified(Samples, run, &cant_of, kCantTolerance))
			result.push_back({Samples[index].chainage, Samples[index].cant});
	return result;
}

std::vector<grade_element> grades(std::vector<sample> const &Samples)
{
	std::vector<grade_element> result;
	for (auto const &run : runs(Samples, &grade_of, kJumpGrade, false))
	{
		auto const kept{simplified(Samples, run, &grade_of, kGradeTolerance)};
		for (std::size_t k = 1; k < kept.size(); ++k)
		{
			auto const &a{Samples[kept[k - 1]]};
			auto const &b{Samples[kept[k]]};
			if (b.chainage - a.chainage <= kSame)
				continue;
			grade_element next{a.chainage, b.chainage - a.chainage, a.grade, b.grade};
			if (std::abs(b.grade - a.grade) <= kGradeTolerance)
			{
				auto const held{mean(Samples, kept[k - 1], kept[k], &grade_of)};
				next.grade_start = held;
				next.grade_end = held;
			}
			// a held grade going on in the same run as the one before it
			if (auto const continued{false == result.empty() && k > 1 && result.back().grade_start == result.back().grade_end && next.grade_start == next.grade_end &&
			                         std::abs(result.back().grade_end - next.grade_start) <= kGradeTolerance};
			    continued)
			{
				auto &last{result.back()};
				auto const total{last.length + next.length};
				auto const held{(last.grade_start * last.length + next.grade_start * next.length) / total};
				last.length = total;
				last.grade_start = held;
				last.grade_end = held;
				continue;
			}
			result.push_back(next);
		}
	}
	// within a run the grade goes on from one piece to the next: a held grade sets the end of a curve next to it
	for (std::size_t i = 1; i < result.size(); ++i)
	{
		auto &before{result[i - 1]};
		auto &after{result[i]};
		auto const before_held{before.grade_start == before.grade_end};
		auto const after_held{after.grade_start == after.grade_end};
		if (std::abs(after.start - before.start - before.length) > kSame || std::abs(after.grade_start - before.grade_end) > kJumpGrade || (before_held && after_held))
			continue;
		if (after_held)
			before.grade_end = after.grade_start;
		else if (before_held)
			after.grade_start = before.grade_end;
		else
		{
			auto const middle{(before.grade_end + after.grade_start) * 0.5};
			before.grade_end = middle;
			after.grade_start = middle;
		}
	}
	return result;
}

} // namespace

line describe(std::vector<sample> const &Samples, std::string Name)
{
	line result;
	result.name = std::move(Name);
	if (Samples.size() < 2)
		return result;
	result.precision = kPrecision;
	result.elements = plan(Samples);
	result.cant = cant(Samples);
	result.grades = grades(Samples);
	return result;
}

} // namespace norms
