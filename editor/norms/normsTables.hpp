/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/norms/norms.hpp"

#include <span>
#include <string_view>

// the rows of the tables of the profiles (normsTables.cpp): a row holds the limits of the three tiers of one parameter where its
// conditions hold, with the place in the document they come from. Where several rows hold at once, the strictest limit of each tier
// is the one (a track at a platform keeps the stricter of the platform row and the main track row)
namespace norms::tables
{

// a limit: constant + per_speed V + per_speed2 V^2 + per_radius R, not below floor
struct formula
{
	double constant{0.0};
	double per_speed{0.0};
	double per_speed2{0.0};
	double per_radius{0.0};
	double floor{-1e300};
};
double value(formula const &Formula, double const Speed, double const Radius);

struct cell
{
	formula limit;
	bool present{false};
	bool confirmed{true};
};

inline constexpr unsigned main_track{1u};
inline constexpr unsigned side_track{2u};
inline constexpr unsigned any_track{main_track | side_track};
inline constexpr unsigned passenger{1u};
inline constexpr unsigned mixed{2u};
inline constexpr unsigned freight{4u};
inline constexpr unsigned any_traffic{passenger | mixed | freight};
inline constexpr double unbounded{1e9};

// the row holds where the query has all the features it needs and none it excludes, for the tracks and the traffic it names, and
// above each lower bound up to each upper one: speed_above < V <= speed_to and so on. A row of radii (radius_above >= 0) holds on
// curves only, the radius of a straight being 0
struct row
{
	parameter what{parameter::count};
	cell prefer;
	cell warn;
	cell block;
	std::string_view place;
	unsigned needs{0};
	unsigned excludes{0};
	unsigned tracks{any_track};
	unsigned traffics{any_traffic};
	double speed_above{-1.0};
	double speed_to{unbounded};
	double radius_above{-1.0};
	double radius_to{unbounded};
	double load_above{-1.0};
	double load_to{unbounded};
	std::string_view document{}; // the profile's own when empty
};

bool applies(row const &Row, query const &Where);
// the rows of the profile, and the global hard ones
std::span<row const> rows(profile_id const Profile);
std::span<row const> hard_rows();

} // namespace norms::tables
