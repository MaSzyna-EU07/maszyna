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
// m: the least radius of a side track, table 9.2
constexpr double kSideRadius{180.0};

// the curved switches bent from the basic ones, table 11.9: the radius of the basic switch, the limiting curvature of the
// diverging track and the least radius of the main track; the values in its brackets, allowed only by way of exception, are past
// them as well
struct bent_limit
{
	double radius;
	double branch;
	double main;
};
constexpr bent_limit kBent[]{{300.0, 4.000e-3, 1505.522}, {500.0, 4.000e-3, 501.730}, {760.0, 3.333e-3, 497.250}, {1200.0, 2.169e-3, 750.001}, {2500.0, 1.063e-3, 1510.0}};

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

// the cant of the line over a switch From to To along it: the same all along it, a ramp off it by 6 m at least or ending at its ends
// (11.4 pkt 7, 9.6 pkt 16-17), as much as table 9.6 lets a switch have, Curved or straight
void check_cant(std::vector<track_point> const &Line, double const From, double const To, bool const Curved, double &Start, double &End, std::vector<std::string> &Notes, std::vector<std::string> &Warnings)
{
	Start = std::abs(cant_at(Line, From));
	End = std::abs(cant_at(Line, To));
	if (std::abs(End - Start) > 1.0)
		Warnings.emplace_back(format(STR_C("The cant changes within the switch, %.0f to %.0f mm: it's to stay the same over it (11.4 pkt 7)"), Start, End));
	auto const low{std::min(From, To)};
	auto const high{std::max(From, To)};
	for (auto const end : ramp_ends(Line))
	{
		if (end > low + kAtEnd && end < high - kAtEnd)
			Warnings.emplace_back(format(STR_C("A cant ramp starts or ends within the switch, %.1f m from its start: at the start or the end of the switch, or 6 m off it at least (9.6 pkt 16-17)"), std::abs(end - From)));
		else if (std::abs(end - low) <= kAtEnd || std::abs(end - high) <= kAtEnd)
			Notes.emplace_back(STR_C("A cant ramp starts or ends at an end of the switch"));
		else if (auto const off{end < low ? low - end : end - high}; off < kRampClear)
			Warnings.emplace_back(format(STR_C("A cant ramp ends %.1f m off the switch, 6 m at least (9.6 pkt 16)"), off));
	}
	auto const cant{std::max(Start, End)};
	if (Curved && cant > 100.0 + 1e-6)
		Warnings.emplace_back(format(STR_C("Cant %.0f mm in a bent switch, over 100 mm (table 9.6)"), cant));
	else if (Curved && cant > 60.0 + 1e-6)
		Notes.emplace_back(format(STR_C("Cant %.0f mm: over the 60 mm recommended for a bent switch, up to 100 mm (table 9.6)"), cant));
	else if (false == Curved && cant > 1.0)
		Notes.emplace_back(format(STR_C("Cant %.0f mm in a straight switch: none recommended, up to 100 mm (table 9.6)"), cant));
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
                    double const Ratio, double const Straight)
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
	// of the part bent: up to the frog of a switch bent up to it
	auto const along = [&](std::vector<track_point> const &Track) {
		span result;
		auto const finish{Track.back().station - (Straight > 0.5 ? Straight : 0.0)};
		auto const length{finish - Track.front().station};
		auto const margin{std::min(1.0, length * 0.25)};
		result.first = sign * curvature_at(Track, Track.front().station + margin);
		result.last = sign * curvature_at(Track, finish - margin);
		for (auto s{Track.front().station + margin}; s <= finish - margin + 1e-9; s += 0.5)
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
	report.curved = main.sharpest > kStraight;
	report.transition = report.curved && std::abs(main.last - main.first) > std::max(2e-6, 0.05 * std::max(std::abs(main.first), std::abs(main.last)));

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
				report.warnings.emplace_back(format(STR_C("The curvature of the diverging track %.3f 1/km is over the limiting %.3f 1/km for the switch of R %.0f m (table 11.9)"), branch.sharpest * 1000.0, limit->branch * 1000.0, Radius));
			if (main.sharpest > kStraight && 1.0 / main.sharpest < limit->main - 1e-6)
				report.warnings.emplace_back(format(STR_C("The main track bent to R %.0f m, below the least %.0f m for the switch of R %.0f m (table 11.9)"), 1.0 / main.sharpest, limit->main, Radius));
		}
		// 1:9 itself is bent, as the Rz 300-1:9 of table 11.9; the ratio may come from the angle of the tracks, a bit off the catalogue,
		// 1:9.403 told from 1:9.51 by half of what keeps them apart
		if (Ratio > 0.0 && (Ratio < 8.95 || std::abs(Ratio - 9.403) < 0.05))
			report.warnings.emplace_back(format(STR_C("A switch of 1:%.3f isn't bent in the running and the main tracks: none of 1:9.403 nor steeper than 1:9 (11.4 pkt 4)"), Ratio));
		// the basic switches of table 11.9 have the curve of the diverging track running through the frog; one with a straight frog is
		// bent up to the frog, the frog and both tracks over it straight, the curvature jumping there
		if (Straight > 0.5)
			report.warnings.emplace_back(format(STR_C("Bent up to the frog, as the DB bends the switches with a straight frog: both tracks keep straight over its last %.1f m, the curvature jumps there; table 11.9 lists no such bent switch"), Straight));
	}

	check_cant(Line, From, To, report.curved, report.cant_start, report.cant_end, report.notes, report.warnings);
	return report;
}

slip_report check_slip(std::vector<std::vector<track_point>> const &Tracks, std::vector<std::vector<track_point>> const &Curves, std::vector<track_point> const &Line, double const From, double const To)
{
	slip_report report;
	if (Tracks.size() < 2 || Line.size() < 2 || std::any_of(Tracks.begin(), Tracks.end(), [](std::vector<track_point> const &Track) { return Track.size() < 2; }))
		return report;
	// the curvatures counted positive the way the line turns, as the first track goes along it
	auto const sign{(Line.back().heading - Line.front().heading < 0.0 ? -1.0 : 1.0) * (To >= From ? 1.0 : -1.0)};
	struct span
	{
		double low{std::numeric_limits<double>::max()};
		double high{std::numeric_limits<double>::lowest()};
		double sharpest{0.0};
	};
	auto const along = [&](std::vector<track_point> const &Track) {
		span result;
		auto const margin{std::min(1.0, (Track.back().station - Track.front().station) * 0.25)};
		for (auto s{Track.front().station + margin}; s <= Track.back().station - margin + 1e-9; s += 0.5)
		{
			auto const curvature{sign * curvature_at(Track, s)};
			result.low = std::min(result.low, curvature);
			result.high = std::max(result.high, curvature);
			result.sharpest = std::max(result.sharpest, std::abs(curvature));
		}
		return result;
	};
	auto const middle = [&](std::vector<track_point> const &Track) { return sign * curvature_at(Track, 0.5 * (Track.front().station + Track.back().station)); };
	report.first = middle(Tracks[0]);
	report.second = middle(Tracks[1]);
	// the curvature of each track through it of one sign all along it, as in the main track of a switch (9.7 pkt 12); the slip curves
	// begin with the leads along the stock rails, which bend with the line even where the curve turns against it
	auto turning{false};
	for (auto const &track : Tracks)
	{
		auto const curvatures{along(track)};
		report.curved = report.curved || curvatures.sharpest > kStraight;
		turning = turning || (curvatures.low < -kStraight && curvatures.high > kStraight);
	}
	if (turning)
		report.warnings.emplace_back(STR_C("The curvature of a track through the slip turns from one side to the other within it, which isn't allowed (9.7 pkt 12)"));
	for (auto const &curve : Curves)
		if (curve.size() >= 2)
			report.curves.push_back(along(curve).sharpest);
	if (report.curved)
	{
		report.warnings.emplace_back(STR_C("A slip isn't bent in the running nor in the main tracks, a bent one goes into a side track only (11.4 pkt 4b)"));
		// bent the way the curve of the slip turns it gets sharper: no sharper than the side tracks take
		for (auto const curvature : report.curves)
			if (curvature > 1.0 / kSideRadius + 1e-9)
				report.warnings.emplace_back(format(STR_C("A slip curve bent to R %.0f m, below the least %.0f m of a side track (table 9.2)"), 1.0 / curvature, kSideRadius));
	}
	check_cant(Line, From, To, report.curved, report.cant_start, report.cant_end, report.notes, report.warnings);
	if (report.curved && std::max(report.cant_start, report.cant_end) > 1.0)
		report.notes.emplace_back(STR_C("In a side track the layouts needing no cant are recommended (9.3 pkt 5)"));
	return report;
}

exit_report check_exit(double const Curvature, double const Straight, std::vector<std::array<double, 3>> const &Line, double const Speed, double const Gauge)
{
	exit_report report;
	// table 9.15 and table 9.16, track connections
	report.allowed = Speed <= 60.0 ? 130.0 : Speed <= 100.0 ? 100.0 : Speed <= 160.0 ? 80.0 : 60.0;
	report.element = Speed <= 60.0 ? 6.0 : Speed <= 100.0 ? 0.15 * Speed : 0.25 * Speed;
	// mm of the cant deficiency the change of the curvature makes at once: 11.8 V^2 / R at the standard gauge
	auto const sudden = [&](double const Before, double const After) { return Gauge * Speed * Speed / (9.81 * 3.6 * 3.6) * std::abs(After - Before); };
	// 1/m, a change of the curvature as large at once is a sudden one
	constexpr double kSudden{1e-4};
	struct change
	{
		double along; // from the end of the switch
		double before;
		double after;
	};
	std::vector<change> changes;
	auto curvature{Curvature};
	if (Straight > 0.0 && std::abs(Curvature) > kSudden)
	{
		changes.push_back({-Straight, Curvature, 0.0});
		curvature = 0.0;
	}
	double along{0.0};
	for (auto const &[length, start, end] : Line)
	{
		if (std::abs(start - curvature) > kSudden)
			changes.push_back({along, curvature, start});
		curvature = end;
		along += length;
	}
	// the curve of the switch going on into the line: nothing of the switch to judge
	if (changes.empty() || changes.front().along > 1e-6)
		return report;
	auto const &first{changes.front()};
	auto const alone = [&](change const &Change) {
		auto const value{sudden(Change.before, Change.after)};
		report.change = std::max(report.change, value);
		if (value > report.allowed)
			report.warnings.emplace_back(format(STR_C("Behind the switch the curvature changes at once by a sudden change of the cant deficiency of %.0f mm, over the %.0f mm allowed at %.0f km/h (table 9.15)"), value, report.allowed, Speed));
	};
	if (changes.size() > 1)
	{
		auto const &next{changes[1]};
		report.apart = next.along - first.along;
		if (report.apart < report.element)
		{
			// taken together, from before the first one to after the next one (9.11 pkt 8b, 9)
			report.change = sudden(first.before, next.after);
			if (report.change > report.allowed)
				report.warnings.emplace_back(format(STR_C("The curve behind the switch begins %.1f m after the end of its curve, closer than the %.1f m which take the sudden changes of the cant deficiency apart: together %.0f mm, over the %.0f mm allowed at %.0f km/h (tables 9.15, 9.16)"),
				                                    report.apart, report.element, report.change, report.allowed, Speed));
			return report;
		}
		alone(next);
	}
	// the end of the curve of the switch at its end, not within it over a straight frog: that one is the switch's own
	if (first.along > -1e-6)
		alone(first);
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
