/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// The norms of the geometry of the tracks: the profiles of the limits as tables, each value with the document and the table or the
// paragraph it comes from, and one engine checking a line described as elements against any of them. It stands on its own, with no
// scene, no tracks and no user interface (editor-tracks-domain.md §3 and §6); the texts of what it finds are made by its users.
//
// Three tiers (§6.2): PREFER, the values the editor would choose itself (PLK P0); WARN, beyond the normal limits, allowed with a
// message (beyond P1); BLOCK, beyond the extended limits capped by the TSI (beyond P2), which no operation may reach. The global hard
// limits of §6.3 hold in every profile. A value the tables mark unconfirmed can warn, never block.
namespace norms
{

enum class profile_id
{
	plk, // PKP PLK ST-T1-A6 v1.0 (2017): P0 PREFER, P1 WARN, P2 BLOCK
	tsi, // TSI INF, Reg. (EU) 1299/2014 as amended to 2023/1694: the legal minimum, BLOCK only
	en13803, // EN 13803-1:2010: normal WARN, exceptional BLOCK
	ch, // Swiss AB-EBV, AB 17 (standard gauge, 1.7.2024): Grenzwert im Normalfall WARN, maximaler Grenzwert BLOCK
	count
};

enum class tier
{
	none,
	prefer, // beyond the value the editor would choose
	warn,
	block
};

enum class parameter
{
	cant_deficiency, // I at the highest speed, mm
	cant_excess, // E at the speed of the slow trains, mm
	deficiency_jump, // ΔI at an abrupt change of the curvature or of the cant, mm
	cant_gradient, // dD/ds of a ramp, mm/m
	cant_rate, // dD/dt along a ramp, mm/s
	deficiency_rate, // dI/dt along a transition or a ramp, mm/s
	element_length, // of a straight or an arc, m, at least
	transition_length, // m, at least
	transition_shift, // of the arc moved in by its transition, m, at least
	radius, // m, at least
	cant, // mm, at most
	cant_min, // mm: a cant given at all is at least this
	vertical_radius, // m, at least; a break of the grade with no vertical curve is 0
	grade, // per mille, at most
	intermediate_length, // m: abrupt changes of the cant deficiency further apart are taken one by one (ST-T1-A6 tab. 9.16)
	profile_boundary, // an arc running across the boundary of two profiles: to be settled (§6.4)
	count
};

// the limit is a least value, not a greatest one
bool minimum(parameter const What);

enum class traffic
{
	passenger,
	mixed,
	freight
};

enum class track_kind
{
	main, // running lines and main tracks of the stations (tory szlakowe, główne zasadnicze i dodatkowe; Zuggleise)
	side // sidings (tory boczne; Rangiergleise)
};

// what the element is part of or what the query is about, flags
namespace feature
{
inline constexpr unsigned switch_track{1u << 0}; // a switch or a crossing: a connection of tracks (połączenie torów)
inline constexpr unsigned movable_frog{1u << 1};
inline constexpr unsigned curved_switch{1u << 2}; // a switch bent into a curve (rozjazd łukowy)
inline constexpr unsigned crossing{1u << 3}; // a crossing or a slip: double, twofold and threefold frogs
inline constexpr unsigned expansion{1u << 4}; // a rail expansion device (przyrząd wyrównawczy)
inline constexpr unsigned diverging{1u << 5}; // the diverging track of a switch
inline constexpr unsigned platform{1u << 6}; // a track at a platform
inline constexpr unsigned station{1u << 7}; // the useful length of a station track
inline constexpr unsigned ballastless{1u << 8};
inline constexpr unsigned suburban{1u << 9}; // passengers may stand (ST-T1-A6 tab. 10.2)
inline constexpr unsigned tilting{1u << 10}; // the trains tilt: the cant deficiency is theirs, not the TSI's
// set by the engine for a query
inline constexpr unsigned crest{1u << 11};
inline constexpr unsigned sag{1u << 12};
inline constexpr unsigned short_stretch{1u << 13}; // a grade held for less than 500 m (ST-T1-A6 tab. 10.1 footnote 2)
inline constexpr unsigned low_deficiency{1u << 14}; // I ≤ 75 mm on both sides of the abrupt change (TSI 4.2.4.4(2))
inline constexpr unsigned abrupt{1u << 15}; // an arc with no transition curve at an end (AB-EBV AB 17 5.2.2)
} // namespace feature

// the class of the line (§2): the highest speed, the slow trains, the traffic, main or side track, and what holds along all of it
struct line_class
{
	double vmax{100.0}; // km/h, where the tracks have no speed of their own
	double vmin{0.0}; // km/h of the slow trains; 0: by ST-T1-A6 tab. 8.1 for the traffic and the highest speed
	traffic kind{traffic::mixed};
	track_kind track{track_kind::main};
	double load{5.0}; // Tg a year, for the cant excess (ST-T1-A6 tab. 9.8)
	unsigned features{0}; // platform, station, ballastless, suburban, tilting

	bool operator==(line_class const &) const = default;
};

// the speed of the slow trains of a line of the traffic and the highest speed given: the line type of ST-T1-A6 tab. 8.1 with the
// lowest highest speed not below it (P passenger, M mixed, T freight), no faster than Vmax
double slow_speed(traffic const Kind, double const Vmax);

// what a limit is asked for
struct query
{
	double speed{0.0}; // km/h
	double radius{0.0}; // m, 0 on a straight
	double load{5.0}; // Tg a year
	traffic kind{traffic::mixed};
	track_kind track{track_kind::main};
	unsigned features{0};
};

struct source
{
	std::string_view document; // ST-T1-A6, TSI INF, EN 13803-1, AB-EBV
	std::string_view place; // the table, the formula or the paragraph
};

struct bound
{
	double value{0.0};
	source from;
	bool confirmed{true};
};

struct thresholds
{
	std::optional<bound> prefer;
	std::optional<bound> warn;
	std::optional<bound> block;
};

// a profile: its tables of limits
struct profile
{
	profile_id id{profile_id::plk};
	std::string_view name; // "PLK ST-T1-A6"
	std::string_view label; // "PLK", as in a message
	std::string_view document; // "ST-T1-A6", the one its tables come from
};
profile const &profile_of(profile_id const Id);

// the limits of the profile for the parameter where the query says, the strictest of the rows which apply; empty tiers where the
// profile sets none
thresholds limits(profile_id const Profile, parameter const What, query const &Where);
// the global hard limits of §6.3 kept in every profile: the TSI, and the cant gradient of EN 13803-1 (track twist)
thresholds hard_limits(parameter const What, query const &Where);

// the tier of the value against the limits of the profile and the hard ones: beyond a confirmed block bound or a hard one, block; beyond
// an unconfirmed block bound or the warn one, warn; beyond the prefer one, prefer. The bound passed is returned in Passed
struct assessment
{
	tier level{tier::none};
	bound passed;
	bool hard{false}; // the hard limit decided
};
// Precision: the share of a limit the value is known within, a value no further beyond it isn't beyond it
assessment assess(parameter const What, double const Value, thresholds const &Profile, thresholds const &Hard, double const Precision = 0.0);

// the formulas of ST-T1-A6 9.3 - 9.5 (and EN 13803-1 5.2): speeds km/h, radii m, cant mm
double equilibrium_cant(double const Speed, double const Radius); // Deq = 11.8 V^2 / R (9.7)
double regular_cant(double const Speed, double const Radius); // Dreg = 6.5 V^2 / R (9.6)
double cant_deficiency(double const Speed, double const Radius, double const Cant); // I = Deq - D (9.9)
double cant_excess(double const Speed, double const Radius, double const Cant); // E = D - Deq(Vmin) (9.10)
double least_cant(double const Speed, double const Radius, double const Deficiency); // Dmin (9.4)
double greatest_cant(double const Speed, double const Radius, double const Excess); // Dmax (9.5)
double least_radius(double const Speed, double const Cant, double const Deficiency); // Rmin (9.1)
double greatest_speed(double const Radius, double const Cant, double const Deficiency); // V at which I reaches the deficiency
// the highest speed through an arc keeping I at the fast trains and I at the slow ones as given (ST-T1-A6 załącznik 1)
double speed_for_deficiencies(double const Radius, double const Fast, double const Slow, double const Vmin);

// a plan point, x and z of the scenery
struct plan_point
{
	double x{0.0};
	double z{0.0};
};

enum class element_kind
{
	straight,
	transition,
	arc
};

// an element of the line in the plan: the curvature changes evenly along it from its start to its end, signed, positive turning left
struct element
{
	element_kind kind{element_kind::straight};
	double start{0.0}; // m, chainage
	double length{0.0};
	double curvature_start{0.0}; // 1/m
	double curvature_end{0.0};
	double speed{0.0}; // km/h, the highest on it; 0: the one of the line class
	unsigned features{0}; // switch_track, movable_frog, curved_switch, crossing, expansion, diverging, platform, station
	// the element is cut by the end of the line or by a switch: its length isn't its own
	bool open_start{false};
	bool open_end{false};
	plan_point from;
	plan_point to;

	double end() const { return start + length; }
};

// the cant along the line, mm, signed: above 0 leaning into a turn to the left; changing evenly between the points, two points at one
// chainage being a step
struct cant_point
{
	double chainage{0.0};
	double cant{0.0};
};

// the grade line: along each element the grade changes evenly, a vertical curve where it does, a break between two elements whose
// grades differ; grades as ratios, above 0 rising onward
struct grade_element
{
	double start{0.0};
	double length{0.0};
	double grade_start{0.0};
	double grade_end{0.0};
};

struct line
{
	std::string name;
	std::vector<element> elements;
	std::vector<cant_point> cant;
	std::vector<grade_element> grades;
	// the share of a value measured on the line it is known within: a line read from the curves of the tracks (an arc of a Bezier curve
	// isn't quite one, its roll is written to a few decimals) isn't taken beyond a limit it reaches within that
	double precision{0.0};
};

// what the line is read from: points along it, a few metres apart, two at a joint, the end of the one path and the start of the next
struct sample
{
	double chainage{0.0};
	plan_point at;
	double grade{0.0};
	double curvature{0.0}; // 1/m, signed, positive turning left
	double cant{0.0}; // mm, signed, above 0 leaning left
	double speed{0.0}; // km/h, 0: none set
	unsigned features{0};
};

// the line described as elements: straights, transitions and arcs where the curvature keeps still or changes evenly, the cant as its
// points, the grade line as its elements
line describe(std::vector<sample> const &Samples, std::string Name = {});

// the profiles in force by place (§6.4): the default of the scenery, areas outlined in the plan, lines by name; a line's own wins
struct area
{
	std::string name;
	profile_id profile{profile_id::plk};
	std::vector<plan_point> outline;
};
struct area_map
{
	profile_id scenery{profile_id::plk};
	std::vector<area> areas;
	std::vector<std::pair<std::string, profile_id>> lines;

	profile_id at(plan_point const &Point, std::string_view Line) const;
};

struct finding
{
	parameter what{parameter::count};
	tier level{tier::none};
	double value{0.0};
	double limit{0.0};
	profile_id profile{profile_id::plk}; // the profile checked, the one whose limit was passed unless hard
	source from;
	bool confirmed{true};
	bool hard{false}; // a global hard limit of §6.3
	double chainage{0.0};
	double length{0.0}; // m along the line it holds over, 0 at a point
	std::size_t element{0};
	double speed{0.0}; // km/h the value was found for
	profile_id other{profile_id::plk}; // profile_boundary: the second profile
};

// the line checked at the line class, each element against the profile of its place, the stricter one of the two at its ends
// on a boundary; the findings in the order of the line: of the parameters checked over an element one each, the worst; of the jumps,
// the ramps and the vertical curves one each
std::vector<finding> evaluate(line const &Line, line_class const &Class, area_map const &Areas);

} // namespace norms
