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
#include "editor/norms/normsTables.hpp"

#include <algorithm>
#include <array>
#include <cmath>

// the limits of the profiles from their tables, the tier of a value against them, the formulas of the cant
namespace norms
{

namespace
{

double constexpr kC{11.8}; // mm m h^2 / km^2: the cant balancing a curve, 11.8 V^2 / R (ST-T1-A6 9.7, EN 13803-1 5.2.1)

// the strictest of the limits of one tier where several rows hold: the lowest of the greatest values, the highest of the least
// ones; at the same value a confirmed one
void stricter(std::optional<bound> &Kept, bound const &Candidate, bool const Minimum)
{
	if (false == Kept.has_value())
	{
		Kept = Candidate;
		return;
	}
	auto const better{Minimum ? Candidate.value > Kept->value : Candidate.value < Kept->value};
	if (better || (Candidate.value == Kept->value && Candidate.confirmed && false == Kept->confirmed))
		Kept = Candidate;
}

thresholds gathered(std::span<tables::row const> const Rows, std::string_view const Document, parameter const What, query const &Where)
{
	thresholds result;
	auto const least{minimum(What)};
	for (auto const &row : Rows)
	{
		if (row.what != What || false == tables::applies(row, Where))
			continue;
		source const from{row.document.empty() ? Document : row.document, row.place};
		auto const take = [&Where, &from, least](tables::cell const &Cell, std::optional<bound> &Kept) {
			if (Cell.present)
				stricter(Kept, {tables::value(Cell.limit, Where.speed, Where.radius), from, Cell.confirmed}, least);
		};
		take(row.prefer, result.prefer);
		take(row.warn, result.warn);
		take(row.block, result.block);
	}
	return result;
}

bool beyond(parameter const What, double const Value, bound const &Bound, double const Precision)
{
	auto const margin{std::max(1e-9 * std::max(1.0, std::abs(Bound.value)), Precision * std::abs(Bound.value))};
	return minimum(What) ? Value < Bound.value - margin : Value > Bound.value + margin;
}

} // namespace

bool minimum(parameter const What)
{
	switch (What)
	{
	case parameter::element_length:
	case parameter::transition_length:
	case parameter::transition_shift:
	case parameter::radius:
	case parameter::cant_min:
	case parameter::vertical_radius:
	case parameter::intermediate_length: return true;
	default: return false;
	}
}

profile const &profile_of(profile_id const Id)
{
	static std::array<profile, static_cast<std::size_t>(profile_id::count)> const profiles{{
	    {profile_id::plk, "PLK ST-T1-A6", "PLK", "ST-T1-A6"},
	    {profile_id::tsi, "TSI INF", "TSI INF", "TSI INF"},
	    {profile_id::en13803, "EN 13803", "EN 13803", "EN 13803-1"},
	    {profile_id::ch, "CH AB-EBV", "AB-EBV", "AB-EBV"},
	}};
	return profiles[std::min(static_cast<std::size_t>(Id), profiles.size() - 1)];
}

double slow_speed(traffic const Kind, double const Vmax)
{
	struct line_type
	{
		traffic kind;
		double vmax;
		double vmin;
	};
	// ST-T1-A6 tab. 8.1: P250 P200 M200 P160 M160 P120 M120 T120 P80 M80 T80 T40
	static std::array<line_type, 12> constexpr types{{
	    {traffic::passenger, 80, 60},
	    {traffic::passenger, 120, 80},
	    {traffic::passenger, 160, 120},
	    {traffic::passenger, 200, 120},
	    {traffic::passenger, 250, 120},
	    {traffic::mixed, 80, 40},
	    {traffic::mixed, 120, 60},
	    {traffic::mixed, 160, 80},
	    {traffic::mixed, 200, 80},
	    {traffic::freight, 40, 40},
	    {traffic::freight, 80, 40},
	    {traffic::freight, 120, 60},
	}};
	double result{-1.0};
	double fastest{-1.0};
	for (auto const &type : types)
	{
		if (type.kind != Kind)
			continue;
		if (type.vmax >= Vmax - 1e-9)
		{
			result = type.vmin;
			break;
		}
		fastest = type.vmin;
	}
	if (result < 0.0)
		result = fastest;
	return std::min(result, Vmax);
}

thresholds limits(profile_id const Profile, parameter const What, query const &Where)
{
	return gathered(tables::rows(Profile), profile_of(Profile).document, What, Where);
}

thresholds hard_limits(parameter const What, query const &Where)
{
	return gathered(tables::hard_rows(), profile_of(profile_id::tsi).document, What, Where);
}

assessment assess(parameter const What, double const Value, thresholds const &Profile, thresholds const &Hard, double const Precision)
{
	assessment result;
	auto const passed = [What, Value, Precision](std::optional<bound> const &Bound) { return Bound.has_value() && beyond(What, Value, *Bound, Precision); };
	// the extended limit of the profile when confirmed, the hard one in any case
	auto const profile_block{passed(Profile.block) && Profile.block->confirmed};
	auto const hard_block{passed(Hard.block)};
	if (profile_block || hard_block)
	{
		result.level = tier::block;
		result.passed = profile_block ? *Profile.block : *Hard.block;
		result.hard = hard_block && false == profile_block;
		return result;
	}
	// an extended limit not confirmed only warns
	if (passed(Profile.block))
	{
		result.level = tier::warn;
		result.passed = *Profile.block;
		return result;
	}
	if (passed(Profile.warn))
	{
		result.level = tier::warn;
		result.passed = *Profile.warn;
		return result;
	}
	if (passed(Profile.prefer))
	{
		result.level = tier::prefer;
		result.passed = *Profile.prefer;
	}
	return result;
}

double equilibrium_cant(double const Speed, double const Radius)
{
	return Radius > 0.0 ? kC * Speed * Speed / Radius : 0.0;
}

double regular_cant(double const Speed, double const Radius)
{
	return Radius > 0.0 ? 6.5 * Speed * Speed / Radius : 0.0;
}

double cant_deficiency(double const Speed, double const Radius, double const Cant)
{
	return equilibrium_cant(Speed, Radius) - Cant;
}

double cant_excess(double const Speed, double const Radius, double const Cant)
{
	return Cant - equilibrium_cant(Speed, Radius);
}

double least_cant(double const Speed, double const Radius, double const Deficiency)
{
	return equilibrium_cant(Speed, Radius) - Deficiency;
}

double greatest_cant(double const Speed, double const Radius, double const Excess)
{
	return equilibrium_cant(Speed, Radius) + Excess;
}

double least_radius(double const Speed, double const Cant, double const Deficiency)
{
	return Cant + Deficiency > 0.0 ? kC * Speed * Speed / (Cant + Deficiency) : 0.0;
}

double greatest_speed(double const Radius, double const Cant, double const Deficiency)
{
	return std::sqrt(std::max(0.0, (Cant + Deficiency) * Radius / kC));
}

double speed_for_deficiencies(double const Radius, double const Fast, double const Slow, double const Vmin)
{
	return std::sqrt(std::max(0.0, (Fast - Slow) * Radius / kC + Vmin * Vmin));
}

} // namespace norms
