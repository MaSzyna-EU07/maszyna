/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorSpeed.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorTrack.hpp"
#include "editor/editorSwitchStore.hpp"

#include "utilities/translation.h"
#include "world/Track.h"

#include <algorithm>
#include <cmath>

namespace speed_check
{

namespace
{

double constexpr kGravity{9.81};

struct sample
{
	double chainage;
	double curvature; // signed, 1/m
	double cant; // m, signed as the curvature: raise of the outer rail
};

double value_at(std::vector<sample> const &Samples, double const Chainage, double sample::*Field)
{
	auto const next{std::lower_bound(Samples.begin(), Samples.end(), Chainage, [](sample const &Sample, double const Value) { return Sample.chainage < Value; })};
	if (next == Samples.begin())
		return (*next).*Field;
	if (next == Samples.end())
		return Samples.back().*Field;
	auto const &before{*std::prev(next)};
	auto const run{next->chainage - before.chainage};
	return run > 1e-9 ? before.*Field + ((*next).*Field - before.*Field) * (Chainage - before.chainage) / run : before.*Field;
}

// highest speed up to Top at which the unbalanced acceleration, changing by Curvature and Cant per metre, changes no faster than Jerk
double jerk_limit(double const Curvature, double const Cant, double const Gauge, double const Jerk, double const Top)
{
	auto const rate = [&](double const Speed) { return std::abs(Speed * Speed / 12.96 * Curvature - kGravity * Cant / Gauge) * Speed / 3.6; };
	if (rate(Top) <= Jerk)
		return Top;
	double low{0.0};
	for (int i = 1; i * 10.0 <= Top; ++i)
	{
		auto const speed{i * 10.0};
		if (rate(speed) > Jerk)
		{
			double high{speed};
			for (int i = 0; i < 12; ++i)
			{
				auto const middle{(low + high) * 0.5};
				(rate(middle) > Jerk ? high : low) = middle;
			}
			return low;
		}
		low = speed;
	}
	return Top;
}

// track the simulation joined to the start or the end of the path
TTrack *linked(TTrack const &Track, int const Path, bool const Atend)
{
	if (Track.SwitchExtension && Path < 2)
		return Atend ? Track.SwitchExtension->pNexts[Path] : Track.SwitchExtension->pPrevs[Path];
	return Atend ? Track.trNext : Track.trPrev;
}

// paths which continue the path through the point, run from or to it. only the joints the trains take count:
// in a double slip the ends of the other diagonal lie a few centimetres away, but aren't joined to this path
std::vector<editor_track::route_span> continuations(TTrack &Track, int const Path, bool const Atend, glm::dvec3 const &Direction, bool const Arriving)
{
	std::vector<editor_track::route_span> result;
	auto *other{linked(Track, Path, Atend)};
	if (other == nullptr || other == &Track || other->m_editorremoved)
		return result;
	auto const &point{Track.m_paths[Path].points[Atend ? segment_data::point::end : segment_data::point::start]};
	for (int path = 0; path < static_cast<int>(other->m_paths.size()); ++path)
	{
		auto const &candidate{other->m_paths[path]};
		auto const atstart{glm::distance(candidate.points[segment_data::point::start], point) < geometry::same_point};
		if (false == atstart && glm::distance(candidate.points[segment_data::point::end], point) >= geometry::same_point)
			continue;
		if (linked(*other, path, false == atstart) != &Track)
			continue;
		geometry::bezier const curve{candidate};
		// direction of travel through the other path at the point
		auto const along{atstart ? curve.first(0.0) : -curve.first(1.0)};
		auto const travel{Arriving ? -along : along};
		if (glm::dot(geometry::plan_of(travel), geometry::plan_of(Direction)) <= 0.0)
			continue;
		result.push_back({other, path, Arriving ? false == atstart : atstart});
	}
	return result;
}

} // namespace

char const *describe(cause const Cause)
{
	switch (Cause)
	{
	case cause::radius: return "radius";
	case cause::cant_rate: return "cant change rate";
	case cause::cant_ramp: return "cant ramp";
	case cause::jerk: return "no transition";
	case cause::branch: return "switch branch";
	default: return "-";
	}
}

bool verdict::violated(options const &Options) const
{
	if (hard >= Options.top)
		return false;
	if (set <= 0.0)
		return true;
	return set > hard + 0.5;
}

bool verdict::warned(options const &Options) const
{
	if (safe >= Options.top || violated(Options))
		return false;
	return set <= 0.0 || set > safe + 0.5;
}

std::string explain(verdict const &Verdict, options const &Options)
{
	if (Verdict.safe >= Options.top)
		return format(STR_C("The geometry allows any speed up to %.0f km/h"), Options.top);
	auto const gauge{Options.norms.gauge / 1000.0};
	// the numbers are given at the speed set, or at the safe one when none is set
	auto const speed{Verdict.set > 0.0 ? Verdict.set : Verdict.safe};
	auto const radius_text = [](double const Radius) {
		return Radius == 0.0 ? std::string{STR_C("a straight")} : format("R %.0f m%s", std::abs(Radius), Radius < 0.0 ? STR_C(" the other way") : "");
	};
	auto const unbalanced = [&](double const Radius, double const Cant) { return (Radius != 0.0 ? speed * speed / (12.96 * Radius) : 0.0) - kGravity * Cant / 1000.0 / gauge; };
	std::string text;
	if (Verdict.warned(Options))
		text = STR_C("Warning only, the limit of a switch branch: ");
	if (Verdict.set <= 0.0)
		text += format(STR_C("No speed limit is set, so the trains run at the line speed, while the geometry allows %.0f km/h.\n"), Verdict.safe);
	else if (Verdict.set > Verdict.safe + 0.5)
		text += format(STR_C("The speed set, %.0f km/h, is above the %.0f km/h the geometry allows.\n"), Verdict.set, Verdict.safe);
	else
		text += format(STR_C("The speed set, %.0f km/h, is within the %.0f km/h the geometry allows.\n"), Verdict.set, Verdict.safe);
	if (Verdict.why == cause::branch)
		text += Verdict.hard < Options.top ? format(STR_C("Without the switch branch limit the geometry allows %.0f km/h (%s).\n"), Verdict.hard, STR_C(describe(Verdict.hard_why)))
		                                   : std::string{STR_C("Without the switch branch limit the geometry allows any speed.\n")};
	switch (Verdict.why)
	{
	case cause::radius:
	case cause::branch:
	{
		bool const branch{Verdict.why == cause::branch};
		text += format(STR_C("%s, %s, cant %.0f mm.\n"), branch ? STR_C("Diverging track of the switch") : STR_C("Curve"), radius_text(Verdict.radius).c_str(), Verdict.cant);
		text += format(STR_C("Unbalanced acceleration V²/(12.96 R) - g h/s at %.0f km/h: %.2f m/s², the limit is %.2f m/s²%s.\n"), speed, unbalanced(Verdict.radius, Verdict.cant),
		               branch ? Options.branch_unbalanced : Options.norms.unbalanced, branch ? STR_C(" for the switch branches, which have no cant") : "");
		text += branch ? STR_C("A switch of a larger radius would allow more") : STR_C("More cant or a larger radius would allow more");
		break;
	}
	case cause::cant_rate:
		text += format(STR_C("The cant changes from %.0f to %.0f mm over %.0f m, %.2f mm per m.\n"), Verdict.cant_from, Verdict.cant_to, Options.reference_length, Verdict.cant_slope);
		text += format(STR_C("A train at %.0f km/h feels it change by %.0f mm/s, the limit is %.0f mm/s.\n"), speed, Verdict.cant_slope * speed / 3.6, Options.norms.cant_rate);
		text += STR_C("A longer cant ramp, along a longer transition curve, would allow more");
		break;
	case cause::cant_ramp:
		text += format(STR_C("The cant changes from %.0f to %.0f mm over %.0f m, a ramp of 1:%.0f.\n"), Verdict.cant_from, Verdict.cant_to, Options.reference_length, 1000.0 / std::max(1e-6, Verdict.cant_slope));
		text += format(STR_C("At %.0f km/h the ramp can't be steeper than 1:(k V) = 1:%.0f, k = %.1f.\n"), speed, Options.norms.ramp_factor * speed, Options.norms.ramp_factor);
		text += STR_C("A longer cant ramp would allow more");
		break;
	case cause::jerk:
	{
		auto const from{unbalanced(Verdict.radius_from, Verdict.cant_from)};
		auto const to{unbalanced(Verdict.radius_to, Verdict.cant_to)};
		auto const time{Options.reference_length / std::max(1.0, speed / 3.6)};
		text += format(STR_C("Within %.0f m the track goes from %s, cant %.0f mm, to %s, cant %.0f mm.\n"), Options.reference_length, radius_text(Verdict.radius_from).c_str(), Verdict.cant_from,
		               radius_text(Verdict.radius_to).c_str(), Verdict.cant_to);
		text += format(STR_C("At %.0f km/h the unbalanced acceleration changes from %.2f to %.2f m/s² in %.1f s: %.2f m/s³, the limit is %.2f m/s³.\n"), speed, from, to, time,
		               std::abs(to - from) / time, Options.norms.jerk);
		text += STR_C("The curve lacks a transition, or it is too short");
		break;
	}
	default: break;
	}
	return text;
}

double suggested(double const Safe)
{
	if (Safe >= 40.0)
		return std::floor(Safe / 10.0) * 10.0;
	return std::max(5.0, std::floor(Safe / 5.0) * 5.0);
}

double path_velocity(TTrack const &Track, int const Path)
{
	auto const velocity{editor_track::velocity(Track)};
	if (Track.eType == tt_Switch && velocity <= -2.0)
		return Path == 0 ? -1.0 : -velocity;
	return velocity > 0.0 ? velocity : -1.0;
}

double switch_velocity(double const Main, double const Diverging)
{
	if (Main <= 0.0)
		return Diverging > 0.0 ? -std::max(2.0, Diverging) : -1.0;
	return Diverging > 0.0 ? std::min(Main, Diverging) : Main;
}

verdict check(TTrack &Track, int const Path, options const &Options)
{
	verdict result;
	result.track = &Track;
	result.path = Path;
	result.set = path_velocity(Track, Path);
	result.safe = Options.top;
	result.hard = Options.top;
	if (Path < 0 || Path >= static_cast<int>(Track.m_paths.size()))
		return result;

	auto const gauge{Options.norms.gauge / 1000.0};
	bool const branch{Track.eType == tt_Switch && Path == 1};
	double kept{0.0};
	if (branch)
		if (auto const known{switch_store::of(Track)})
			kept = switch_store::branch_radius(*known);
	auto const &own{Track.m_paths[Path]};
	geometry::bezier const curve{own};
	auto const before{continuations(Track, Path, false, curve.first(0.0), true)};
	auto const after{continuations(Track, Path, true, curve.first(1.0), false)};
	std::vector<editor_track::route_span const *> previous{nullptr}, next{nullptr};
	if (false == before.empty())
		previous.clear();
	for (auto const &span : before)
		previous.push_back(&span);
	if (false == after.empty())
		next.clear();
	for (auto const &span : after)
		next.push_back(&span);

	auto const half{Options.reference_length * 0.5};
	for (auto const *first : previous)
	{
		for (auto const *last : next)
		{
			editor_track::route route;
			if (first != nullptr)
				route.spans.push_back(*first);
			route.spans.push_back({&Track, Path, true});
			if (last != nullptr)
				route.spans.push_back(*last);
			auto const own_span{first != nullptr ? std::size_t{1} : std::size_t{0}};
			auto const sampled{editor_track::sample_route(route, Options.step)};
			std::vector<sample> samples;
			samples.reserve(sampled.size());
			for (auto const &entry : sampled)
			{
				auto const sign{entry.curvature < 0.0 ? -1.0 : 1.0};
				samples.push_back({entry.chainage, entry.curvature, entry.cant * sign});
			}
			auto const from{route.spans[own_span].from};
			auto const to{route.spans[own_span].to};
			for (std::size_t i = 0; i < sampled.size(); ++i)
			{
				if (sampled[i].span != own_span)
					continue;
				auto const &here{samples[i]};
				auto const curvature{kept > 0.0 ? std::min(std::abs(here.curvature), 1.0 / kept) : std::abs(here.curvature)};
				auto const cant{std::abs(here.cant)};
				double safe{Options.top};
				cause why{cause::none};
				auto const limit = [&](double const Speed, cause const Cause) {
					if (Speed < safe)
					{
						safe = Speed;
						why = Cause;
					}
					if (Cause != cause::branch && Speed < result.hard)
					{
						result.hard = Speed;
						result.hard_why = Cause;
					}
				};
				if (curvature > 1e-7)
				{
					auto const allowed{(branch ? Options.branch_unbalanced : Options.norms.unbalanced) + kGravity * cant / gauge};
					limit(3.6 * std::sqrt(std::max(0.0, allowed) / curvature), branch ? cause::branch : cause::radius);
				}
				auto const start{std::max(0.0, here.chainage - half)};
				auto const end{std::min(route.length, here.chainage + half)};
				auto const run{end - start};
				double cantchange{0.0};
				if (run > Options.step * 0.5)
				{
					cantchange = (value_at(samples, end, &sample::cant) - value_at(samples, start, &sample::cant)) / run;
					auto const curvaturechange{(value_at(samples, end, &sample::curvature) - value_at(samples, start, &sample::curvature)) / run};
					if (std::abs(cantchange) > 1e-7)
					{
						limit(3.6 * Options.norms.cant_rate / 1000.0 / std::abs(cantchange), cause::cant_rate);
						limit(1.0 / (Options.norms.ramp_factor * std::abs(cantchange)), cause::cant_ramp);
					}
					// switches have no transitions by design, their limits cover it
					auto const regular{std::all_of(route.spans.begin(), route.spans.end(), [&](editor_track::route_span const &Span) { return Span.to < start || Span.from > end || Span.track->eType == tt_Normal; })};
					if (regular && (std::abs(curvaturechange) > 1e-9 || std::abs(cantchange) > 1e-9))
						limit(jerk_limit(curvaturechange, cantchange, gauge, Options.norms.jerk, Options.top), cause::jerk);
				}
				if (safe < result.safe)
				{
					auto const share{to > from ? (here.chainage - from) / (to - from) : 0.0};
					result.safe = safe;
					result.why = why;
					result.position = curve.point(std::clamp(curve.parameter(share * curve.plan_length()), 0.0, 1.0));
					result.radius = curvature > 1e-7 ? 1.0 / curvature : 0.0;
					result.cant = cant * 1000.0;
					// signed as the curvature here, a reverse curve getting the other sign
					auto const sign{here.curvature < 0.0 ? -1.0 : 1.0};
					auto const radius_at = [&](double const Chainage) {
						auto const value{value_at(samples, Chainage, &sample::curvature) * sign};
						return std::abs(value) > 1e-7 ? 1.0 / value : 0.0;
					};
					result.radius_from = radius_at(start);
					result.radius_to = radius_at(end);
					result.cant_from = value_at(samples, start, &sample::cant) * sign * 1000.0;
					result.cant_to = value_at(samples, end, &sample::cant) * sign * 1000.0;
					result.cant_slope = std::abs(cantchange) * 1000.0;
				}
			}
		}
	}
	return result;
}

} // namespace speed_check
