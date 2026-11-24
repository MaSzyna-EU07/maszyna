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

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// The assumptions of the project ("Co rysujesz?"): what the user draws, said once at the start of a scenery and changed later in a
// window of their own: the norms, the speed of the line, the traffic, a main or a side track, the gauge, the rail and the look of the
// tracks. They're kept with the map, in its project file where it has one, else in the scenery as the line //$u setup ..., and they're
// the defaults of the laying, the plan, the switches, the parallel tool and the profile. There's no "no norms": every scenery has a
// profile, a fictional one too, PLK where nothing was said; the TSI alone are the loosest
namespace project_setup
{

struct setup
{
	norms::profile_id profile{norms::profile_id::plk};
	double speed{100.0}; // km/h of the line
	norms::traffic traffic{norms::traffic::mixed};
	norms::track_kind track{norms::track_kind::main};
	int gauge{1435}; // mm
	std::string rail{"60E1"}; // the switches of this rail first
	std::string rails; // the texture of the rails of the tracks laid; empty: as the lay tool has it
	std::string ballast; // the texture of the trackbed; empty: as the lay tool has it

	bool operator==(setup const &) const = default;
};

inline constexpr std::array<int, 5> gauges{1435, 1520, 1000, 750, 600};
inline constexpr std::array<std::string_view, 3> rail_kinds{"60E1", "49E1", "S42"};
inline constexpr std::array<double, 8> speeds{40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0, 200.0};

// the words the profiles are written with, as in the project file: plk, tsi, en13803, ch
std::string_view profile_key(norms::profile_id const Profile);
std::optional<norms::profile_id> profile_of_key(std::string_view const Key);

// the class of the line the norms judge the tracks laid by: the traffic and the track of the setup, the speed given (the one of the
// line laid), or the setup's own
norms::line_class line_class(setup const &Setup, double const Speed = 0.0);
// mm between the axes of the rails the cant is measured on: the gauge and the head of a rail, 1500 for 1435
double rail_axes(setup const &Setup);
// km/h through the diverging track of the switches laid by default: as the line and the track have it
double diverging_speed(setup const &Setup);

// one line: "setup norms plk speed 100 traffic mixed track main gauge 1435 rail 60E1", the textures after it where given
std::string record(setup const &Setup);
// the line of record(); none where it isn't one. The words it doesn't know are passed over, a value it can't read keeps the default
std::optional<setup> read(std::string_view const Line);
// the first line of the //$u lines (the mark taken off) which reads
std::optional<setup> read(std::vector<std::string> const &Lines);

} // namespace project_setup
