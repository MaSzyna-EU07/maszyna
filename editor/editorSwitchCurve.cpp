/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorSwitchCurve.hpp"
#include "editor/editorFormat.hpp"
#include "utilities/translation.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace switch_curve
{

namespace
{

// 1/m: a curvature below it is a straight, the curvature measured over a few metres never quite 0
constexpr double kStraight{2e-5};
// m: an end of a ramp this close to an end of the switch is at it
constexpr double kAtEnd{0.3};
// m: how far off the switch a ramp keeps, 9.6 pkt 16
constexpr double kRampClear{6.0};

// the curved switches bent from the basic ones, table 11.9: the radius of the basic switch, the limiting curvature of the
// diverging track and the least radius of the main track, the values allowed by way of exception in the brackets, 0 for none
struct bent_limit
{
	double radius;
	double branch;
	double branch_exception;
	double main;
	double main_exception;
};
constexpr bent_limit kBent[]{
    {300.0, 4.000e-3, 4.673e-3, 1505.522, 749.722}, {500.0, 4.000e-3, 4.673e-3, 501.730, 375.638}, {760.0, 3.333e-3, 0.0, 497.250, 0.0}, {1200.0, 2.169e-3, 0.0, 750.001, 0.0}, {2500.0, 1.063e-3, 0.0, 1510.0, 0.0}};

double heading_at(std::vector<track_point> const &Track, double const Station)
{
	if (Station <= Track.front().station)
		return Track.front().heading;
	if (Station >= Track.back().station)
		return Track.back().heading;
	auto const next{std::lower_bound(Track.begin(), Track.end(), Station, [](track_point const &Point, double const Value) { return Point.station < Value; })};
	auto const previous{std::prev(next)};
	auto const span{std::max(1e-9, next->station - previous->station)};
	return previous->heading + (next->heading - previous->heading) * (Station - previous->station) / span;
}

double cant_at(std::vector<track_point> const &Track, double const Station)
{
	if (Station <= Track.front().station)
		return Track.front().cant;
	if (Station >= Track.back().station)
		return Track.back().cant;
	auto const next{std::lower_bound(Track.begin(), Track.end(), Station, [](track_point const &Point, double const Value) { return Point.station < Value; })};
	auto const previous{std::prev(next)};
	auto const span{std::max(1e-9, next->station - previous->station)};
	return previous->cant + (next->cant - previous->cant) * (Station - previous->station) / span;
}

} // namespace

double curvature_at(std::vector<track_point> const &Track, double const Station)
{
	if (Track.size() < 2)
		return 0.0;
	auto const from{std::max(Track.front().station, Station - 1.0)};
	auto const to{std::min(Track.back().station, Station + 1.0)};
	if (to - from < 0.2)
		return 0.0;
	return (heading_at(Track, to) - heading_at(Track, from)) / (to - from);
}

std::vector<double> ramp_ends(std::vector<track_point> const &Track)
{
	std::vector<double> ends;
	auto ramping{false};
	for (std::size_t i = 0; i + 1 < Track.size(); ++i)
	{
		auto const span{Track[i + 1].station - Track[i].station};
		if (span < 1e-6)
			continue;
		// a change of the cant of 1 mm over 20 m and more
		auto const changing{std::abs(Track[i + 1].cant - Track[i].cant) / span > 0.05};
		if (changing == ramping)
			continue;
		ramping = changing;
		// a ramp going on after a break of under a metre, where the paths meet, is the same ramp
		if (changing && false == ends.empty() && Track[i].station - ends.back() < 1.0)
			ends.pop_back();
		else
			ends.push_back(Track[i].station);
	}
	if (ramping)
		ends.push_back(Track.back().station);
	return ends;
}

switch_report check(std::vector<track_point> const &Main, std::vector<track_point> const &Branch, std::vector<track_point> const &Line, double const From, double const To, double const Radius,
                    double const Ratio)
{
	switch_report report;
	if (Main.size() < 2 || Branch.size() < 2 || Line.size() < 2)
		return report;
	// the curvatures counted positive the way the line turns, as the switch goes along it
	auto const direction{To >= From ? 1.0 : -1.0};
	auto const sign{(Line.back().heading - Line.front().heading < 0.0 ? -1.0 : 1.0) * direction};
	struct span
	{
		double first{0.0};
		double last{0.0};
		double low{std::numeric_limits<double>::max()};
		double high{std::numeric_limits<double>::lowest()};
		double sharpest{0.0};
	};
	auto const along = [&](std::vector<track_point> const &Track) {
		span result;
		auto const length{Track.back().station - Track.front().station};
		auto const margin{std::min(1.0, length * 0.25)};
		result.first = sign * curvature_at(Track, Track.front().station + margin);
		result.last = sign * curvature_at(Track, Track.back().station - margin);
		for (auto s{Track.front().station + margin}; s <= Track.back().station - margin + 1e-9; s += 0.5)
		{
			auto const curvature{sign * curvature_at(Track, s)};
			result.low = std::min(result.low, curvature);
			result.high = std::max(result.high, curvature);
			result.sharpest = std::max(result.sharpest, std::abs(curvature));
		}
		return result;
	};
	auto const main{along(Main)};
	auto const branch{along(Branch)};
	report.main_start = main.first;
	report.main_end = main.last;
	report.branch_start = branch.first;
	report.branch_end = branch.last;
	report.cant_start = std::abs(cant_at(Line, From));
	report.cant_end = std::abs(cant_at(Line, To));
	report.curved = main.sharpest > kStraight;
	report.transition = report.curved && std::abs(main.last - main.first) > std::max(2e-6, 0.05 * std::max(std::abs(main.first), std::abs(main.last)));

	if (report.transition)
		report.notes.emplace_back(STR_C("On a transition curve: only by way of exception, by consent of the Zaklad Linii Kolejowych (9.7 pkt 12); the plan gives the radii of both tracks at the start and at the end of the switch"));
	// the curvature of each track of one sign all along it, the cases KZW I and KZW III of 9.7 pkt 12, not KZW II
	if (branch.low < -kStraight && branch.high > kStraight)
		report.warnings.emplace_back(STR_C("The curvature of the diverging track turns from one side to the other within the switch (KZW II), which isn't allowed (9.7 pkt 12)"));
	if (main.low < -kStraight && main.high > kStraight)
		report.warnings.emplace_back(STR_C("The curvature of the main track turns from one side to the other within the switch, which isn't allowed (9.7 pkt 12)"));
	if (report.curved)
	{
		auto const limit{std::find_if(std::begin(kBent), std::end(kBent), [&](bent_limit const &Limit) { return std::abs(Limit.radius - Radius) < 2.0; })};
		if (limit == std::end(kBent))
			report.notes.emplace_back(format(STR_C("The basic switch of R %.0f m isn't in table 11.9, its limiting curvature isn't checked"), Radius));
		else
		{
			if (branch.sharpest > limit->branch + 1e-9)
				report.warnings.emplace_back(limit->branch_exception > 0.0 && branch.sharpest <= limit->branch_exception + 1e-9 ?
				                                 format(STR_C("The curvature of the diverging track %.3f 1/km is over the limiting %.3f 1/km of table 11.9, up to %.3f 1/km only by consent of the Zaklad Linii Kolejowych"), branch.sharpest * 1000.0,
				                                        limit->branch * 1000.0, limit->branch_exception * 1000.0) :
				                                 format(STR_C("The curvature of the diverging track %.3f 1/km is over the limiting %.3f 1/km for the switch of R %.0f m (table 11.9)"), branch.sharpest * 1000.0, limit->branch * 1000.0, Radius));
			if (main.sharpest > kStraight && 1.0 / main.sharpest < limit->main - 1e-6)
				report.warnings.emplace_back(limit->main_exception > 0.0 && 1.0 / main.sharpest >= limit->main_exception - 1e-6 ?
				                                 format(STR_C("The main track bent to R %.0f m, below the least %.0f m of table 11.9, down to %.0f m only by consent of the Zaklad Linii Kolejowych"), 1.0 / main.sharpest, limit->main, limit->main_exception) :
				                                 format(STR_C("The main track bent to R %.0f m, below the least %.0f m for the switch of R %.0f m (table 11.9)"), 1.0 / main.sharpest, limit->main, Radius));
		}
		if (Ratio > 0.0 && Ratio < 9.41)
			report.warnings.emplace_back(format(STR_C("A switch of 1:%.3f isn't bent in the running and the main tracks: none of 1:9.403 nor steeper than 1:9 (11.4 pkt 4)"), Ratio));
	}

	// the cant: the same all along the switch, a ramp off it by 6 m at least or ending at its ends (11.4 pkt 7, 9.6 pkt 16-17)
	if (std::abs(report.cant_end - report.cant_start) > 1.0)
		report.warnings.emplace_back(format(STR_C("The cant changes within the switch, %.0f to %.0f mm: it's to stay the same over it (11.4 pkt 7); a straight ramp through the switch only by consent of the Zaklad Linii Kolejowych"),
		                                    report.cant_start, report.cant_end));
	auto const low{std::min(From, To)};
	auto const high{std::max(From, To)};
	for (auto const end : ramp_ends(Line))
	{
		if (end > low + kAtEnd && end < high - kAtEnd)
			report.warnings.emplace_back(format(STR_C("A cant ramp starts or ends within the switch, %.1f m from its start: at the start or the end of the switch, or 6 m off it at least (9.6 pkt 16-17)"), std::abs(end - From)));
		else if (std::abs(end - low) <= kAtEnd || std::abs(end - high) <= kAtEnd)
			report.notes.emplace_back(STR_C("A cant ramp starts or ends at an end of the switch"));
		else if (auto const off{end < low ? low - end : end - high}; off < kRampClear)
			report.warnings.emplace_back(format(STR_C("A cant ramp ends %.1f m off the switch, 6 m at least (9.6 pkt 16)"), off));
	}
	auto const cant{std::max(report.cant_start, report.cant_end)};
	if (report.curved && cant > 100.0 + 1e-6)
		report.warnings.emplace_back(format(STR_C("Cant %.0f mm in a bent switch, over 100 mm (table 9.6)"), cant));
	else if (report.curved && cant > 60.0 + 1e-6)
		report.notes.emplace_back(format(STR_C("Cant %.0f mm: over the 60 mm recommended for a bent switch, up to 100 mm (table 9.6)"), cant));
	else if (false == report.curved && cant > 1.0)
		report.notes.emplace_back(format(STR_C("Cant %.0f mm in a straight switch: none recommended, up to 100 mm (table 9.6)"), cant));
	return report;
}

double snapped_start(std::vector<double> const &Ramps, double const Pressed, double const Length, int const Direction, double const Reach, std::string *What)
{
	auto const d{Direction < 0 ? -1.0 : 1.0};
	auto best{Pressed};
	auto nearest{Reach};
	std::string name;
	auto const consider = [&](double const Start, char const *Name) {
		if (auto const distance{std::abs(Start - Pressed)}; distance <= nearest)
		{
			nearest = distance;
			best = Start;
			name = Name;
		}
	};
	for (auto const end : Ramps)
	{
		consider(end, STR_C("the switch starts at an end of the cant ramp"));
		consider(end - d * Length, STR_C("the switch ends at an end of the cant ramp"));
		consider(end + d * kRampClear, STR_C("the switch starts 6 m off the cant ramp"));
		consider(end - d * (kRampClear + Length), STR_C("the switch ends 6 m off the cant ramp"));
	}
	if (What != nullptr)
		*What = name;
	return best;
}

} // namespace switch_curve
