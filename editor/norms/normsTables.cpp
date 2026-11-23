/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/norms/normsTables.hpp"

#include <algorithm>
#include <array>

// The tables of the profiles. Each row names the table or the paragraph of its document. The sources, as read for this module:
//
// PLK: Standardy Techniczne ... ST-T1-A6 "Układy geometryczne torów", v1.0, PKP PLK 2017 (uchwała 1086/2017), chapters 8 - 10. The
//   cells spanning two or three columns of the tables (9.1, 9.6, 9.7, 9.13 - 9.15, 10.1, 10.2) were read on the pages of the PDF
//   (editor-tracks-domain.md §6.5 pkt 4). Two rows stand out of line with the rows around them as printed and are kept unconfirmed:
//   tab. 9.8, 5 < q <= 10, P0 95 mm (above the P0 of q <= 5), and tab. 9.7, movable frog 230 < V <= 250, P2 110 mm (above its P1).
//   Table 9.4 (the straight between reverse curves without transitions) is a picture with no text and isn't here.
// TSI: Commission Regulation (EU) No 1299/2014 (TSI INF), consolidated text of 28.9.2023 (amendments 2019/776 and 2023/1694), 4.2.3
//   and 4.2.4. The cant deficiency of 153 mm spans both columns up to 300 km/h in table 8 (§6.5 pkt 3: not 100 mm above 160 km/h).
// EN 13803: EN 13803-1:2010, 5.2.1 - 5.2.5 (the part of the standard published as its preview: radius, cant, cant deficiency, cant
//   excess, cant gradient). Its rate of change of the cant and of the cant deficiency, its element lengths and its vertical curves
//   aren't in that part and aren't here. The plain line values of the abrupt change of the cant deficiency are EN 13803-2:2006+A1:2009
//   7.1.3, read from a preview whose table is garbled, so unconfirmed.
// CH: AB-EBV (Ausführungsbestimmungen zur Eisenbahnverordnung, SR 742.141.11) as of 1.7.2024, AB 17 "Trassierungselemente",
//   Normalspur, VR <= 250 km/h. It has no limit of the grade of a line, nor of the cant below which none is given.
// DB (Ril 800.0110) isn't here: it isn't published, and its values are known only from secondary sources.
namespace norms::tables
{

double value(formula const &Formula, double const Speed, double const Radius)
{
	return std::max(Formula.floor, Formula.constant + Formula.per_speed * Speed + Formula.per_speed2 * Speed * Speed + Formula.per_radius * Radius);
}

bool applies(row const &Row, query const &Where)
{
	auto const track{Where.track == track_kind::main ? main_track : side_track};
	auto traffic_bit{mixed};
	if (Where.kind == traffic::passenger)
		traffic_bit = passenger;
	else if (Where.kind == traffic::freight)
		traffic_bit = freight;
	return (Where.features & Row.needs) == Row.needs && (Where.features & Row.excludes) == 0 && (Row.tracks & track) != 0 && (Row.traffics & traffic_bit) != 0 && Where.speed > Row.speed_above &&
	       Where.speed <= Row.speed_to && Where.radius > Row.radius_above && Where.radius <= Row.radius_to && Where.load > Row.load_above && Where.load <= Row.load_to;
}

namespace
{

using namespace norms::feature;

constexpr cell at(double const Value)
{
	cell result;
	result.limit.constant = Value;
	result.present = true;
	return result;
}

constexpr cell linear(double const Constant, double const PerSpeed)
{
	auto result{at(Constant)};
	result.limit.per_speed = PerSpeed;
	return result;
}

// PerSpeed V, not below Floor
constexpr cell share(double const PerSpeed, double const Floor = -1e300)
{
	auto result{at(0.0)};
	result.limit.per_speed = PerSpeed;
	result.limit.floor = Floor;
	return result;
}

// PerSpeed2 V^2, not below Floor
constexpr cell square(double const PerSpeed2, double const Floor)
{
	auto result{at(0.0)};
	result.limit.per_speed2 = PerSpeed2;
	result.limit.floor = Floor;
	return result;
}

// the length run in the time at the speed
constexpr cell travel(double const Seconds)
{
	return share(Seconds / 3.6);
}

// (R - 50) / 1.5: the cant of the small radii, for the stiff freight wagons
constexpr cell small_radius()
{
	auto result{at(-50.0 / 1.5)};
	result.limit.per_radius = 1.0 / 1.5;
	return result;
}

constexpr cell unconfirmed(cell Cell)
{
	Cell.confirmed = false;
	return Cell;
}

constexpr cell none{};

using enum parameter;

// clang-format off
constexpr std::array plk{
	// tab. 9.7, the cant deficiency; footnote 2 of P2: 150 mm for selected passenger vehicles only
	row{.what = cant_deficiency, .prefer = at(110), .warn = at(130), .block = at(150), .place = "tab. 9.7", .excludes = switch_track | expansion, .tracks = main_track},
	row{.what = cant_deficiency, .prefer = at(80), .warn = at(100), .block = at(100), .place = "tab. 9.7", .excludes = switch_track | expansion, .tracks = side_track},
	row{.what = cant_deficiency, .prefer = at(80), .warn = at(100), .block = at(100), .place = "tab. 9.7", .excludes = switch_track | expansion, .radius_above = 200, .radius_to = 250},
	row{.what = cant_deficiency, .prefer = at(50), .warn = at(70), .block = at(70), .place = "tab. 9.7", .excludes = switch_track | expansion, .radius_above = 0, .radius_to = 200},
	row{.what = cant_deficiency, .prefer = at(90), .warn = at(110), .block = at(110), .place = "tab. 9.7", .needs = switch_track, .excludes = movable_frog | crossing, .speed_to = 160},
	row{.what = cant_deficiency, .prefer = at(60), .warn = at(90), .block = at(90), .place = "tab. 9.7", .needs = switch_track, .excludes = movable_frog | crossing, .speed_above = 160, .speed_to = 200},
	row{.what = cant_deficiency, .prefer = at(100), .warn = at(130), .block = at(130), .place = "tab. 9.7", .needs = switch_track | movable_frog, .excludes = crossing, .speed_to = 230},
	row{.what = cant_deficiency, .prefer = at(80), .warn = at(80), .block = unconfirmed(at(110)), .place = "tab. 9.7", .needs = switch_track | movable_frog, .excludes = crossing, .speed_above = 230, .speed_to = 250},
	row{.what = cant_deficiency, .prefer = at(0), .warn = at(0), .block = at(80), .place = "tab. 9.7", .needs = crossing, .speed_to = 120},
	row{.what = cant_deficiency, .prefer = at(80), .warn = at(100), .block = at(100), .place = "tab. 9.7", .needs = expansion, .speed_to = 160},
	row{.what = cant_deficiency, .prefer = at(0), .warn = at(80), .block = at(80), .place = "tab. 9.7", .needs = expansion, .speed_above = 160, .speed_to = 200},
	row{.what = cant_deficiency, .prefer = at(0), .warn = at(60), .block = at(60), .place = "tab. 9.7", .needs = expansion, .speed_above = 200, .speed_to = 250},
	// tab. 9.8, the cant excess by the load of the track; in switches, crossings and expansion devices also their row of tab. 9.7 (9.5 pkt 4)
	row{.what = cant_excess, .prefer = at(90), .warn = at(110), .block = at(110), .place = "tab. 9.8", .load_to = 5},
	row{.what = cant_excess, .prefer = unconfirmed(at(95)), .warn = at(95), .block = at(110), .place = "tab. 9.8", .load_above = 5, .load_to = 10},
	row{.what = cant_excess, .prefer = at(80), .warn = at(95), .block = at(110), .place = "tab. 9.8", .load_above = 10, .load_to = 15},
	row{.what = cant_excess, .prefer = at(65), .warn = at(80), .block = at(110), .place = "tab. 9.8", .load_above = 15, .load_to = 20},
	row{.what = cant_excess, .prefer = at(50), .warn = at(80), .block = at(110), .place = "tab. 9.8", .load_above = 20},
	row{.what = cant_excess, .prefer = at(90), .warn = at(110), .block = at(110), .place = "9.5 pkt 4, tab. 9.7", .needs = switch_track, .excludes = movable_frog | crossing, .speed_to = 160},
	row{.what = cant_excess, .prefer = at(60), .warn = at(90), .block = at(90), .place = "9.5 pkt 4, tab. 9.7", .needs = switch_track, .excludes = movable_frog | crossing, .speed_above = 160, .speed_to = 200},
	row{.what = cant_excess, .prefer = at(100), .warn = at(130), .block = at(130), .place = "9.5 pkt 4, tab. 9.7", .needs = switch_track | movable_frog, .excludes = crossing, .speed_to = 230},
	row{.what = cant_excess, .prefer = at(80), .warn = at(80), .block = unconfirmed(at(110)), .place = "9.5 pkt 4, tab. 9.7", .needs = switch_track | movable_frog, .excludes = crossing, .speed_above = 230, .speed_to = 250},
	row{.what = cant_excess, .prefer = at(0), .warn = at(0), .block = at(80), .place = "9.5 pkt 4, tab. 9.7", .needs = crossing, .speed_to = 120},
	row{.what = cant_excess, .prefer = at(80), .warn = at(100), .block = at(100), .place = "9.5 pkt 4, tab. 9.7", .needs = expansion, .speed_to = 160},
	row{.what = cant_excess, .prefer = at(0), .warn = at(80), .block = at(80), .place = "9.5 pkt 4, tab. 9.7", .needs = expansion, .speed_above = 160, .speed_to = 200},
	row{.what = cant_excess, .prefer = at(0), .warn = at(60), .block = at(60), .place = "9.5 pkt 4, tab. 9.7", .needs = expansion, .speed_above = 200, .speed_to = 250},
	// tab. 9.15, the abrupt change of the cant deficiency: connections of tracks, running lines and main tracks, sidings
	row{.what = deficiency_jump, .prefer = at(130), .warn = at(130), .block = at(130), .place = "tab. 9.15", .needs = switch_track, .speed_to = 60},
	row{.what = deficiency_jump, .prefer = at(100), .warn = at(100), .block = at(125), .place = "tab. 9.15", .needs = switch_track, .speed_above = 60, .speed_to = 100},
	row{.what = deficiency_jump, .prefer = at(80), .warn = at(80), .block = linear(160, -0.35), .place = "tab. 9.15", .needs = switch_track, .speed_above = 100, .speed_to = 160},
	row{.what = deficiency_jump, .prefer = at(60), .warn = at(60), .block = linear(160, -0.35), .place = "tab. 9.15", .needs = switch_track, .speed_above = 160, .speed_to = 200},
	row{.what = deficiency_jump, .prefer = at(50), .warn = at(50), .block = at(60), .place = "tab. 9.15", .excludes = switch_track, .tracks = main_track, .speed_to = 70},
	row{.what = deficiency_jump, .prefer = at(0), .warn = linear(64, -0.2), .block = at(50), .place = "tab. 9.15", .excludes = switch_track, .tracks = main_track, .speed_above = 70, .speed_to = 160},
	row{.what = deficiency_jump, .prefer = at(0), .warn = at(30), .block = at(50), .place = "tab. 9.15", .excludes = switch_track, .tracks = main_track, .speed_above = 160, .speed_to = 200},
	row{.what = deficiency_jump, .prefer = at(0), .warn = at(25), .block = at(25), .place = "tab. 9.15", .excludes = switch_track, .tracks = main_track, .speed_above = 200, .speed_to = 250},
	row{.what = deficiency_jump, .prefer = at(50), .warn = at(100), .block = at(100), .place = "tab. 9.15", .excludes = switch_track, .tracks = side_track, .speed_to = 40},
	// tab. 9.12 the gradient of a cant ramp (twist 8 / 10 / 12.5 mm on a base of 5 m), tab. 9.13 the rate of change of the cant and tab. 9.14
	// that of the cant deficiency, straight ramps
	row{.what = cant_gradient, .prefer = at(1.6), .warn = at(2.0), .block = at(2.5), .place = "tab. 9.12", .speed_to = 250},
	row{.what = cant_rate, .prefer = at(35), .warn = at(50), .block = at(60), .place = "tab. 9.13"},
	row{.what = deficiency_rate, .prefer = at(55), .warn = at(70), .block = at(80), .place = "tab. 9.14", .speed_to = 200},
	row{.what = deficiency_rate, .prefer = at(55), .warn = at(55), .block = at(70), .place = "tab. 9.14", .speed_above = 200, .speed_to = 250},
	// tab. 9.1, the shortest straights and arcs of the running lines and main tracks, outside the connections of tracks
	row{.what = element_length, .prefer = share(0.35, 30), .warn = share(0.35, 30), .block = share(0.25, 20), .place = "tab. 9.1", .excludes = switch_track, .tracks = main_track, .speed_to = 120},
	row{.what = element_length, .prefer = share(0.40), .warn = share(0.40), .block = share(0.30), .place = "tab. 9.1", .excludes = switch_track, .tracks = main_track, .speed_above = 120, .speed_to = 200},
	row{.what = element_length, .prefer = share(0.65), .warn = share(0.65), .block = share(0.55), .place = "tab. 9.1", .excludes = switch_track, .tracks = main_track, .speed_above = 200, .speed_to = 250},
	// tab. 9.10, the transition curves: at least 30 m and moving the arc in by 0.02 m (9.14) at P0 and P1, only the kinematic terms
	// at P2, checked as the cant gradient and the rates of change
	row{.what = transition_length, .prefer = at(30), .warn = at(30), .block = none, .place = "tab. 9.10", .excludes = switch_track},
	row{.what = transition_shift, .prefer = at(0.02), .warn = at(0.02), .block = none, .place = "9.7 pkt 10, tab. 9.10", .excludes = switch_track},
	// tab. 9.2, the least radii
	row{.what = radius, .prefer = at(600), .warn = at(300), .block = at(180), .place = "tab. 9.2", .excludes = switch_track, .tracks = main_track},
	row{.what = radius, .prefer = at(500), .warn = at(500), .block = at(300), .place = "tab. 9.2", .needs = platform, .excludes = switch_track},
	row{.what = radius, .prefer = at(450), .warn = at(180), .block = at(180), .place = "tab. 9.2", .excludes = switch_track, .tracks = side_track},
	// tab. 9.6, the cant; below R 300 m also (R - 50) / 1.5 (9.8)
	row{.what = cant, .prefer = at(150), .warn = at(150), .block = at(150), .place = "tab. 9.6", .excludes = switch_track | ballastless, .tracks = main_track},
	row{.what = cant, .prefer = at(150), .warn = at(150), .block = at(160), .place = "tab. 9.6", .needs = ballastless, .excludes = switch_track, .tracks = main_track},
	row{.what = cant, .prefer = at(60), .warn = at(100), .block = at(110), .place = "tab. 9.6", .needs = platform},
	row{.what = cant, .prefer = at(0), .warn = at(60), .block = at(100), .place = "tab. 9.6", .excludes = switch_track, .tracks = side_track},
	row{.what = cant, .prefer = at(0), .warn = at(100), .block = at(150), .place = "tab. 9.6", .excludes = switch_track, .radius_above = 200, .radius_to = 300},
	row{.what = cant, .prefer = at(0), .warn = at(60), .block = at(100), .place = "tab. 9.6", .excludes = switch_track, .radius_above = 0, .radius_to = 200},
	row{.what = cant, .prefer = none, .warn = none, .block = small_radius(), .place = "9.8", .excludes = switch_track, .radius_above = 0, .radius_to = 299.999},
	row{.what = cant, .prefer = at(0), .warn = at(100), .block = at(110), .place = "tab. 9.6", .needs = switch_track, .excludes = curved_switch | crossing},
	row{.what = cant, .prefer = at(60), .warn = at(100), .block = at(110), .place = "tab. 9.6", .needs = switch_track | curved_switch, .excludes = crossing},
	row{.what = cant, .prefer = at(0), .warn = at(0), .block = at(0), .place = "tab. 9.6", .needs = crossing},
	row{.what = cant_min, .prefer = at(20), .warn = at(20), .block = at(20), .place = "tab. 9.6"},
	// tab. 10.2, the least radii of the vertical curves
	row{.what = vertical_radius, .prefer = square(0.77, 5000), .warn = square(0.35, 5000), .block = square(0.25, 2000), .place = "tab. 10.2", .excludes = switch_track | suburban},
	row{.what = vertical_radius, .prefer = square(0.77, 5000), .warn = square(0.77, 5000), .block = square(0.25, 2000), .place = "tab. 10.2", .needs = suburban, .excludes = switch_track},
	row{.what = vertical_radius, .prefer = square(0.35, 5000), .warn = square(0.35, 5000), .block = square(0.25, 3000), .place = "tab. 10.2", .needs = switch_track | crest},
	row{.what = vertical_radius, .prefer = square(0.35, 3000), .warn = square(0.35, 3000), .block = square(0.25, 2000), .place = "tab. 10.2", .needs = switch_track | sag},
	// tab. 10.1, the steepest grades: running lines and main tracks of the lines of type P, of types M and T (in brackets, footnote 2:
	// on stretches shorter than 500 m), the useful length of station tracks, tracks at platforms
	row{.what = grade, .prefer = at(6.0), .warn = at(17.5), .block = at(35.0), .place = "tab. 10.1", .excludes = station, .tracks = main_track, .traffics = passenger},
	row{.what = grade, .prefer = at(6.0), .warn = at(10.0), .block = at(20.0), .place = "tab. 10.1", .excludes = station | short_stretch, .tracks = main_track, .traffics = mixed | freight},
	row{.what = grade, .prefer = at(6.0), .warn = at(17.5), .block = at(35.0), .place = "tab. 10.1", .needs = short_stretch, .excludes = station, .tracks = main_track, .traffics = mixed | freight},
	row{.what = grade, .prefer = at(2.0), .warn = at(2.0), .block = at(2.5), .place = "tab. 10.1", .needs = station},
	row{.what = grade, .prefer = at(2.5), .warn = at(6.0), .block = at(10.0), .place = "tab. 10.1", .needs = platform},
	// tab. 9.16, the shortest element between abrupt changes of the cant deficiency for them to be taken one by one: on the lines tab. 9.1
	// at P2 (for a transition between them tab. 9.10 at P2 would do, the one of 9.1 stands in for it)
	row{.what = intermediate_length, .prefer = none, .warn = none, .block = share(0.25, 20), .place = "tab. 9.16", .excludes = switch_track, .tracks = main_track, .speed_to = 120},
	row{.what = intermediate_length, .prefer = none, .warn = none, .block = share(0.30), .place = "tab. 9.16", .excludes = switch_track, .tracks = main_track, .speed_above = 120, .speed_to = 200},
	row{.what = intermediate_length, .prefer = none, .warn = none, .block = share(0.55), .place = "tab. 9.16", .excludes = switch_track, .tracks = main_track, .speed_above = 200, .speed_to = 250},
	row{.what = intermediate_length, .prefer = none, .warn = none, .block = at(6.0), .place = "tab. 9.16", .needs = switch_track, .speed_to = 60},
	row{.what = intermediate_length, .prefer = none, .warn = none, .block = share(0.15), .place = "tab. 9.16", .needs = switch_track, .speed_above = 60, .speed_to = 100},
	row{.what = intermediate_length, .prefer = none, .warn = none, .block = share(0.25), .place = "tab. 9.16", .needs = switch_track, .speed_above = 100, .speed_to = 200},
	row{.what = intermediate_length, .prefer = none, .warn = none, .block = share(0.10), .place = "tab. 9.16", .excludes = switch_track, .tracks = side_track, .speed_to = 40},
};

// the TSI rows the profile "TSI only" and the global hard limits share
constexpr std::array tsi_rows{
	row{.what = cant_deficiency, .block = at(153), .place = "4.2.4.3, tab. 8", .excludes = tilting, .speed_to = 300},
	row{.what = cant_deficiency, .block = at(100), .place = "4.2.4.3, tab. 8", .excludes = tilting, .speed_above = 300},
	row{.what = cant_deficiency, .block = at(130), .place = "4.2.4.3, tab. 8", .traffics = freight, .speed_to = 160},
	row{.what = cant, .block = at(160), .place = "4.2.4.2, tab. 7", .excludes = ballastless, .traffics = mixed | freight},
	row{.what = cant, .block = at(180), .place = "4.2.4.2, tab. 7", .excludes = ballastless, .traffics = passenger},
	row{.what = cant, .block = at(170), .place = "4.2.4.2, tab. 7", .needs = ballastless, .traffics = mixed | freight},
	row{.what = cant, .block = at(180), .place = "4.2.4.2, tab. 7", .needs = ballastless, .traffics = passenger},
	row{.what = cant, .block = at(110), .place = "4.2.4.2(2)", .needs = platform},
	row{.what = cant, .block = small_radius(), .place = "4.2.4.2(3)", .traffics = mixed | freight, .radius_above = 0, .radius_to = 304.999},
	row{.what = deficiency_jump, .block = at(130), .place = "4.2.4.4(1)", .excludes = low_deficiency, .speed_to = 40},
	row{.what = deficiency_jump, .block = at(150), .place = "4.2.4.4(2)", .needs = low_deficiency, .speed_to = 40},
	row{.what = deficiency_jump, .block = at(130), .place = "4.2.4.4(1)", .speed_above = 40, .speed_to = 60},
	row{.what = deficiency_jump, .block = at(125), .place = "4.2.4.4(1)", .speed_above = 60, .speed_to = 200},
	row{.what = deficiency_jump, .block = at(85), .place = "4.2.4.4(1)", .speed_above = 200, .speed_to = 230},
	row{.what = deficiency_jump, .block = at(25), .place = "4.2.4.4(1)", .speed_above = 230},
	row{.what = radius, .block = at(150), .place = "4.2.3.4(1)", .excludes = switch_track},
	row{.what = vertical_radius, .block = at(500), .place = "4.2.3.5(1)", .needs = crest},
	row{.what = vertical_radius, .block = at(900), .place = "4.2.3.5(1)", .needs = sag},
};

template <std::size_t First, std::size_t Second>
constexpr std::array<row, First + Second> joined(std::array<row, First> const &A, std::array<row, Second> const &B)
{
	std::array<row, First + Second> result{};
	std::ranges::copy(A, result.begin());
	std::ranges::copy(B, result.begin() + First);
	return result;
}

// 4.2.3.3: up to 35 per mille on the main tracks of new P1 lines for passengers alone, within the envelope of (3) a and b; 2.5 per
// mille through the platforms where vehicles are attached or detached
constexpr auto tsi{joined(tsi_rows, std::array{
	row{.what = grade, .block = at(35.0), .place = "4.2.3.3(3)", .tracks = main_track, .traffics = passenger},
	row{.what = grade, .block = at(2.5), .place = "4.2.3.3(1)", .needs = station},
})};

// in every profile (editor-tracks-domain.md 6.3) the TSI, and the cant gradient of 2.5 mm/m, the exceptional limit of EN 13803-1,
// set for the twist of the track; the TSI rows of (R - 50) / 1.5 leave out the condition of a ramp steeper than 1 mm/m
constexpr auto hard{joined(tsi_rows, std::array{
	row{.what = cant_gradient, .block = at(2.5), .place = "5.2.5", .document = "EN 13803-1"},
})};

constexpr std::array en{
	row{.what = radius, .warn = at(190), .block = at(150), .place = "5.2.1"},
	row{.what = radius, .prefer = at(500), .place = "5.2.1 NOTE 1", .needs = platform},
	row{.what = cant, .warn = at(160), .block = at(180), .place = "5.2.2"},
	row{.what = cant, .prefer = at(110), .place = "5.2.2 NOTE", .needs = platform},
	row{.what = cant, .warn = small_radius(), .place = "5.2.2", .radius_above = 0, .radius_to = 319.999},
	row{.what = cant_deficiency, .warn = at(130), .block = at(183), .place = "5.2.3, tab. 1", .excludes = tilting, .speed_above = 80, .speed_to = 200},
	row{.what = cant_deficiency, .warn = at(130), .block = at(168), .place = "5.2.3, tab. 1", .excludes = tilting, .speed_above = 200, .speed_to = 230},
	row{.what = cant_deficiency, .warn = at(130), .block = at(153), .place = "5.2.3, tab. 1", .excludes = tilting, .speed_above = 230, .speed_to = 250},
	row{.what = cant_deficiency, .warn = at(100), .block = at(130), .place = "5.2.3, tab. 1", .excludes = tilting, .speed_above = 250, .speed_to = 300},
	row{.what = cant_deficiency, .warn = at(275), .block = at(306), .place = "5.2.3, tab. 1", .needs = tilting, .speed_above = 79.999, .speed_to = 260},
	row{.what = cant_excess, .warn = at(110), .place = "5.2.4"},
	row{.what = cant_gradient, .warn = at(2.25), .block = at(2.5), .place = "5.2.5"},
	row{.what = deficiency_jump, .warn = unconfirmed(at(50)), .place = "7.1.3", .excludes = switch_track, .speed_to = 70, .document = "EN 13803-2"},
	row{.what = deficiency_jump, .warn = unconfirmed(at(40)), .place = "7.1.3", .excludes = switch_track, .speed_above = 70, .speed_to = 170, .document = "EN 13803-2"},
	row{.what = deficiency_jump, .warn = unconfirmed(at(30)), .place = "7.1.3", .excludes = switch_track, .speed_above = 170, .speed_to = 230, .document = "EN 13803-2"},
};

constexpr std::array ch{
	// 3.2, the shortest straight and arc, 1 s (0.7 s) of the travel; 3.3 the least radius, R >= 500 m to be aimed for at platforms; 5.2.2
	// 185 m for an arc without a transition in the tracks of the trains
	row{.what = element_length, .warn = travel(1.0), .block = travel(0.7), .place = "AB 17 3.2", .excludes = switch_track, .tracks = main_track},
	row{.what = radius, .warn = at(150), .place = "AB 17 3.3.1"},
	row{.what = radius, .prefer = at(500), .place = "AB 17 3.3.2", .needs = platform},
	row{.what = radius, .warn = at(185), .place = "AB 17 5.2.2", .needs = abrupt, .excludes = switch_track, .tracks = main_track},
	// 3.4 the cant, 100 mm to be aimed for at platforms; 6.3.1.1 in curved switches
	row{.what = cant, .warn = at(160), .block = at(180), .place = "AB 17 3.4.1"},
	row{.what = cant, .warn = small_radius(), .place = "AB 17 3.4.2.1", .radius_above = 0},
	row{.what = cant, .prefer = at(100), .place = "AB 17 3.4.2.2", .needs = platform},
	row{.what = cant, .warn = at(120), .block = at(150), .place = "AB 17 6.3.1.1", .needs = switch_track | curved_switch},
	// 3.6.2.1 the cant deficiency by the radius, 200 < VR <= 250 by footnote 2; 6.3.3.1 the main track of the switches, 6.2.1 their
	// diverging track, between 100 and 200 km/h evenly between the values; 8.3.2 the tilting trains
	row{.what = cant_deficiency, .warn = at(122), .block = at(130), .place = "AB 17 3.6.2.1", .excludes = switch_track | tilting, .speed_to = 200, .radius_above = 0, .radius_to = 349.999},
	row{.what = cant_deficiency, .warn = at(130), .block = at(150), .place = "AB 17 3.6.2.1", .excludes = switch_track | tilting, .speed_to = 200, .radius_above = 349.999, .radius_to = 649.999},
	row{.what = cant_deficiency, .warn = at(150), .block = at(150), .place = "AB 17 3.6.2.1", .excludes = switch_track | tilting, .speed_to = 200, .radius_above = 649.999},
	row{.what = cant_deficiency, .warn = at(130), .block = at(150), .place = "AB 17 3.6.2.1", .excludes = switch_track | tilting, .speed_above = 200, .speed_to = 250},
	row{.what = cant_deficiency, .warn = at(110), .block = at(130), .place = "AB 17 6.3.3.1", .needs = switch_track, .excludes = diverging | movable_frog, .speed_to = 140},
	row{.what = cant_deficiency, .warn = at(100), .block = at(130), .place = "AB 17 6.3.3.1", .needs = switch_track, .excludes = diverging | movable_frog, .speed_above = 140, .speed_to = 160},
	row{.what = cant_deficiency, .warn = at(80), .block = at(130), .place = "AB 17 6.3.3.1", .needs = switch_track, .excludes = diverging | movable_frog, .speed_above = 160, .speed_to = 250},
	row{.what = cant_deficiency, .warn = at(130), .block = at(130), .place = "AB 17 6.3.3.1", .needs = switch_track | movable_frog, .excludes = diverging},
	row{.what = cant_deficiency, .warn = at(100), .block = at(120), .place = "AB 17 6.2.1", .needs = switch_track | diverging, .speed_to = 100},
	row{.what = cant_deficiency, .warn = linear(130, -0.3), .block = linear(155, -0.35), .place = "AB 17 6.2.1", .needs = switch_track | diverging, .speed_above = 100, .speed_to = 200},
	row{.what = cant_deficiency, .block = at(275), .place = "AB 17 8.3.2", .needs = tilting, .radius_above = 249.999},
	row{.what = cant_excess, .warn = at(110), .block = at(130), .place = "AB 17 3.7.1"},
	// 4.2.1 the twist of the ramps, of the tracks of the trains and of the shunting tracks; 4.3 the rates of change
	row{.what = cant_gradient, .warn = at(2.0), .block = at(2.5), .place = "AB 17 4.2.1", .tracks = main_track},
	row{.what = cant_gradient, .warn = at(3.0), .place = "AB 17 4.2.1", .tracks = side_track},
	row{.what = cant_rate, .warn = at(50), .block = at(60), .place = "AB 17 4.3.2"},
	row{.what = deficiency_rate, .warn = at(55), .block = at(90), .place = "AB 17 4.3.1", .speed_to = 200},
	row{.what = deficiency_rate, .warn = at(55), .block = at(75), .place = "AB 17 4.3.1", .speed_above = 200},
	// 5.3 the abrupt change of the cant deficiency: an arc without a transition in the tracks of the trains (between 80 and 160 km/h evenly
	// from 50 to 25 mm), switches and connections, shunting tracks and sidings
	row{.what = deficiency_jump, .warn = at(50), .place = "AB 17 5.3.1", .excludes = switch_track, .tracks = main_track, .speed_to = 80},
	row{.what = deficiency_jump, .warn = linear(75, -0.3125), .place = "AB 17 5.3.1", .excludes = switch_track, .tracks = main_track, .speed_above = 80, .speed_to = 160},
	row{.what = deficiency_jump, .warn = at(0), .place = "AB 17 5.3.1", .excludes = switch_track, .tracks = main_track, .speed_above = 160},
	row{.what = deficiency_jump, .warn = at(102), .block = at(120), .place = "AB 17 5.3.2", .needs = switch_track, .tracks = main_track},
	row{.what = deficiency_jump, .warn = at(120), .block = at(120), .place = "AB 17 5.3.2", .tracks = side_track},
	// 5.4.1 the calming element between abrupt changes of the curvature, case a
	row{.what = intermediate_length, .block = travel(0.7), .place = "AB 17 5.4.1.1", .tracks = main_track, .speed_to = 65},
	row{.what = intermediate_length, .block = travel(1.0), .place = "AB 17 5.4.1.1", .tracks = main_track, .speed_above = 65},
	row{.what = intermediate_length, .block = travel(0.7), .place = "AB 17 5.4.1.4", .tracks = side_track},
	// 7.2 - 7.4 the vertical curves: in the tracks of the trains, in their switches, in shunting tracks
	row{.what = vertical_radius, .warn = square(0.35, 3000), .block = square(0.25, 500), .place = "AB 17 7.2", .needs = crest, .excludes = switch_track, .tracks = main_track},
	row{.what = vertical_radius, .warn = square(0.35, 2000), .block = square(0.175, 900), .place = "AB 17 7.2", .needs = sag, .excludes = switch_track, .tracks = main_track},
	row{.what = vertical_radius, .warn = square(0.35, 5000), .block = square(0.25, 500), .place = "AB 17 7.3", .needs = switch_track | crest, .tracks = main_track},
	row{.what = vertical_radius, .warn = square(0.35, 3000), .block = square(0.175, 900), .place = "AB 17 7.3", .needs = switch_track | sag, .tracks = main_track},
	row{.what = vertical_radius, .warn = at(1000), .block = at(500), .place = "AB 17 7.4", .needs = crest, .excludes = switch_track, .tracks = side_track},
	row{.what = vertical_radius, .warn = at(1000), .block = at(900), .place = "AB 17 7.4", .needs = sag, .excludes = switch_track, .tracks = side_track},
	row{.what = vertical_radius, .warn = at(2000), .block = at(500), .place = "AB 17 7.4", .needs = switch_track | crest, .tracks = side_track},
	row{.what = vertical_radius, .warn = at(2000), .block = at(900), .place = "AB 17 7.4", .needs = switch_track | sag, .tracks = side_track},
};
// clang-format on

} // namespace

std::span<row const> rows(profile_id const Profile)
{
	switch (Profile)
	{
	case profile_id::plk: return plk;
	case profile_id::tsi: return tsi;
	case profile_id::en13803: return en;
	case profile_id::ch: return ch;
	default: return {};
	}
}

std::span<row const> hard_rows()
{
	return hard;
}

} // namespace norms::tables
