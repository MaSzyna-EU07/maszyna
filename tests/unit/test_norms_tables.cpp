// the tables of the profiles of the norms (editor/norms): every value of the PLK tables of ST-T1-A6, P0 / P1 / P2, and of the TSI, at
// the edges of their bands too (a speed on the edge of two bands, R 300, 250 and 200 m), the table in the name of each test; a few of
// EN 13803-1 and of AB-EBV

#include "stdafx.h"
#include "doctest.h"
#include "editor/norms/norms.hpp"

#include <optional>
#include <string>

namespace {

using norms::parameter;
using norms::profile_id;
using norms::track_kind;
using norms::traffic;
namespace feature = norms::feature;

std::optional<double> constexpr none{};

norms::query at(double const Speed, double const Radius = 0.0, unsigned const Features = 0, track_kind const Track = track_kind::main, traffic const Kind = traffic::mixed, double const Load = 5.0) {
	norms::query query;
	query.speed = Speed;
	query.radius = Radius;
	query.features = Features;
	query.track = Track;
	query.kind = Kind;
	query.load = Load;
	return query;
}

void tier(std::optional<norms::bound> const &Bound, std::optional<double> const Expected, char const *Name) {
	CAPTURE(Name);
	REQUIRE(Bound.has_value() == Expected.has_value());
	if (Expected.has_value()) {
		CHECK(Bound->value == doctest::Approx(*Expected).epsilon(1e-9));
	}
}

// the limits of the three tiers, PREFER / WARN / BLOCK, empty where the profile sets none
void tiers(norms::thresholds const &Limits, std::optional<double> const Prefer, std::optional<double> const Warn, std::optional<double> const Block) {
	tier(Limits.prefer, Prefer, "prefer");
	tier(Limits.warn, Warn, "warn");
	tier(Limits.block, Block, "block");
}

norms::thresholds plk(parameter const What, norms::query const &Where) {
	return norms::limits(profile_id::plk, What, Where);
}

norms::thresholds tsi(parameter const What, norms::query const &Where) {
	return norms::limits(profile_id::tsi, What, Where);
}

std::string place(norms::thresholds const &Limits) {
	return Limits.block.has_value() ? std::string{Limits.block->from.place} : std::string{};
}

} // namespace

TEST_CASE("norms PLK tab. 9.7: the cant deficiency of the running lines, the sidings, the small radii, the switches, the crossings and the expansion devices") {
	tiers(plk(parameter::cant_deficiency, at(160)), 110.0, 130.0, 150.0);
	tiers(plk(parameter::cant_deficiency, at(80, 0.0, 0, track_kind::side)), 80.0, 100.0, 100.0);
	// R 250 m is the band 200 < R <= 250, a little more the running line; R 200 m the band R <= 200
	tiers(plk(parameter::cant_deficiency, at(80, 250.0)), 80.0, 100.0, 100.0);
	tiers(plk(parameter::cant_deficiency, at(80, 250.01)), 110.0, 130.0, 150.0);
	tiers(plk(parameter::cant_deficiency, at(80, 200.0)), 50.0, 70.0, 70.0);
	tiers(plk(parameter::cant_deficiency, at(80, 200.01)), 80.0, 100.0, 100.0);
	// switches with a fixed frog: 160 km/h is the band V <= 160
	tiers(plk(parameter::cant_deficiency, at(160, 0.0, feature::switch_track)), 90.0, 110.0, 110.0);
	tiers(plk(parameter::cant_deficiency, at(160.5, 0.0, feature::switch_track)), 60.0, 90.0, 90.0);
	tiers(plk(parameter::cant_deficiency, at(200, 0.0, feature::switch_track)), 60.0, 90.0, 90.0);
	// with a movable frog; P2 of 230 < V <= 250 is above its P1 as printed, kept unconfirmed
	tiers(plk(parameter::cant_deficiency, at(230, 0.0, feature::switch_track | feature::movable_frog)), 100.0, 130.0, 130.0);
	auto const fast_nose{plk(parameter::cant_deficiency, at(240, 0.0, feature::switch_track | feature::movable_frog))};
	tiers(fast_nose, 80.0, 80.0, 110.0);
	CHECK_FALSE(fast_nose.block->confirmed);
	tiers(plk(parameter::cant_deficiency, at(120, 0.0, feature::switch_track | feature::crossing)), 0.0, 0.0, 80.0);
	tiers(plk(parameter::cant_deficiency, at(160, 0.0, feature::expansion)), 80.0, 100.0, 100.0);
	tiers(plk(parameter::cant_deficiency, at(200, 0.0, feature::expansion)), 0.0, 80.0, 80.0);
	tiers(plk(parameter::cant_deficiency, at(250, 0.0, feature::expansion)), 0.0, 60.0, 60.0);
	CHECK(place(plk(parameter::cant_deficiency, at(160))) == "tab. 9.7");
}

TEST_CASE("norms PLK tab. 9.8: the cant excess by the load of the track, the switches by tab. 9.7 too (9.5 pkt 4)") {
	tiers(plk(parameter::cant_excess, at(80, 0.0, 0, track_kind::main, traffic::mixed, 5.0)), 90.0, 110.0, 110.0);
	auto const second{plk(parameter::cant_excess, at(80, 0.0, 0, track_kind::main, traffic::mixed, 10.0))};
	tiers(second, 95.0, 95.0, 110.0);
	CHECK_FALSE(second.prefer->confirmed);
	tiers(plk(parameter::cant_excess, at(80, 0.0, 0, track_kind::main, traffic::mixed, 15.0)), 80.0, 95.0, 110.0);
	tiers(plk(parameter::cant_excess, at(80, 0.0, 0, track_kind::main, traffic::mixed, 20.0)), 65.0, 80.0, 110.0);
	tiers(plk(parameter::cant_excess, at(80, 0.0, 0, track_kind::main, traffic::mixed, 20.5)), 50.0, 80.0, 110.0);
	// the stricter of tab. 9.8 and the row of the switch in tab. 9.7
	tiers(plk(parameter::cant_excess, at(180, 0.0, feature::switch_track, track_kind::main, traffic::mixed, 15.0)), 60.0, 90.0, 90.0);
	tiers(plk(parameter::cant_excess, at(100, 0.0, feature::switch_track | feature::crossing)), 0.0, 0.0, 80.0);
}

TEST_CASE("norms PLK tab. 9.15: the abrupt change of the cant deficiency in the connections of tracks, the running lines and the sidings") {
	auto const connection{feature::switch_track};
	tiers(plk(parameter::deficiency_jump, at(60, 0.0, connection)), 130.0, 130.0, 130.0);
	tiers(plk(parameter::deficiency_jump, at(60.5, 0.0, connection)), 100.0, 100.0, 125.0);
	tiers(plk(parameter::deficiency_jump, at(100, 0.0, connection)), 100.0, 100.0, 125.0);
	tiers(plk(parameter::deficiency_jump, at(160, 0.0, connection)), 80.0, 80.0, 160.0 - 0.35 * 160.0);
	tiers(plk(parameter::deficiency_jump, at(200, 0.0, connection)), 60.0, 60.0, 160.0 - 0.35 * 200.0);
	tiers(plk(parameter::deficiency_jump, at(70)), 50.0, 50.0, 60.0);
	tiers(plk(parameter::deficiency_jump, at(100)), 0.0, 64.0 - 0.2 * 100.0, 50.0);
	tiers(plk(parameter::deficiency_jump, at(160)), 0.0, 64.0 - 0.2 * 160.0, 50.0);
	tiers(plk(parameter::deficiency_jump, at(200)), 0.0, 30.0, 50.0);
	tiers(plk(parameter::deficiency_jump, at(250)), 0.0, 25.0, 25.0);
	tiers(plk(parameter::deficiency_jump, at(40, 0.0, 0, track_kind::side)), 50.0, 100.0, 100.0);
	tiers(plk(parameter::deficiency_jump, at(50, 0.0, 0, track_kind::side)), none, none, none);
}

TEST_CASE("norms PLK tab. 9.12, 9.13, 9.14: the cant gradient, the rate of change of the cant and of the cant deficiency") {
	tiers(plk(parameter::cant_gradient, at(250)), 1.6, 2.0, 2.5);
	tiers(plk(parameter::cant_rate, at(160)), 35.0, 50.0, 60.0);
	tiers(plk(parameter::deficiency_rate, at(200)), 55.0, 70.0, 80.0);
	tiers(plk(parameter::deficiency_rate, at(201)), 55.0, 55.0, 70.0);
	CHECK(place(plk(parameter::cant_gradient, at(100))) == "tab. 9.12");
}

TEST_CASE("norms PLK tab. 9.1: the shortest straights and arcs") {
	tiers(plk(parameter::element_length, at(80)), 30.0, 30.0, 20.0);
	tiers(plk(parameter::element_length, at(120)), 42.0, 42.0, 30.0);
	tiers(plk(parameter::element_length, at(160)), 64.0, 64.0, 48.0);
	tiers(plk(parameter::element_length, at(250)), 162.5, 162.5, 137.5);
	// not in the connections of tracks
	tiers(plk(parameter::element_length, at(80, 0.0, feature::switch_track)), none, none, none);
}

TEST_CASE("norms PLK tab. 9.10: the transition curves, 30 m and the shift of the arc by 0.02 m (9.14) at P0 and P1") {
	tiers(plk(parameter::transition_length, at(100)), 30.0, 30.0, none);
	tiers(plk(parameter::transition_shift, at(100)), 0.02, 0.02, none);
}

TEST_CASE("norms PLK tab. 9.2: the least radii") {
	tiers(plk(parameter::radius, at(100, 500.0)), 600.0, 300.0, 180.0);
	tiers(plk(parameter::radius, at(40, 500.0, 0, track_kind::side)), 450.0, 180.0, 180.0);
	// at a platform the stricter of its row, 500 / 500 / 300, and the one of the track
	tiers(plk(parameter::radius, at(40, 500.0, feature::platform, track_kind::side)), 500.0, 500.0, 300.0);
	tiers(plk(parameter::radius, at(100, 500.0, feature::platform)), 600.0, 500.0, 300.0);
}

TEST_CASE("norms PLK tab. 9.6 and 9.8: the cant, (R - 50) / 1.5 below R 300 m") {
	tiers(plk(parameter::cant, at(100)), 150.0, 150.0, 150.0);
	tiers(plk(parameter::cant, at(100, 0.0, feature::ballastless)), 150.0, 150.0, 160.0);
	tiers(plk(parameter::cant, at(100, 0.0, feature::platform)), 60.0, 100.0, 110.0);
	tiers(plk(parameter::cant, at(40, 0.0, 0, track_kind::side)), 0.0, 60.0, 100.0);
	// R 300 m is the band 200 < R <= 300, with no (R - 50) / 1.5 yet; R 250 m with it; R 200 m the band R <= 200, where it's 100 mm too
	tiers(plk(parameter::cant, at(80, 300.0)), 0.0, 100.0, 150.0);
	auto const tight{plk(parameter::cant, at(80, 250.0))};
	tiers(tight, 0.0, 100.0, (250.0 - 50.0) / 1.5);
	CHECK(place(tight) == "9.8");
	tiers(plk(parameter::cant, at(80, 200.0)), 0.0, 60.0, 100.0);
	tiers(plk(parameter::cant, at(80, 300.01)), 150.0, 150.0, 150.0);
	tiers(plk(parameter::cant, at(80, 0.0, feature::switch_track)), 0.0, 100.0, 110.0);
	tiers(plk(parameter::cant, at(80, 0.0, feature::switch_track | feature::curved_switch)), 60.0, 100.0, 110.0);
	tiers(plk(parameter::cant, at(80, 0.0, feature::switch_track | feature::crossing)), 0.0, 0.0, 0.0);
	tiers(plk(parameter::cant_min, at(80)), 20.0, 20.0, 20.0);
}

TEST_CASE("norms PLK tab. 10.2: the least radii of the vertical curves") {
	tiers(plk(parameter::vertical_radius, at(100, 0.0, feature::crest)), 7700.0, 5000.0, 2500.0);
	tiers(plk(parameter::vertical_radius, at(160, 0.0, feature::sag)), 0.77 * 160 * 160, 0.35 * 160 * 160, 0.25 * 160 * 160);
	tiers(plk(parameter::vertical_radius, at(60, 0.0, feature::sag)), 5000.0, 5000.0, 2000.0);
	tiers(plk(parameter::vertical_radius, at(100, 0.0, feature::suburban | feature::crest)), 7700.0, 7700.0, 2500.0);
	tiers(plk(parameter::vertical_radius, at(100, 0.0, feature::switch_track | feature::crest)), 5000.0, 5000.0, 3000.0);
	tiers(plk(parameter::vertical_radius, at(100, 0.0, feature::switch_track | feature::sag)), 3500.0, 3500.0, 2500.0);
}

TEST_CASE("norms PLK tab. 10.1: the steepest grades") {
	tiers(plk(parameter::grade, at(100, 0.0, 0, track_kind::main, traffic::passenger)), 6.0, 17.5, 35.0);
	tiers(plk(parameter::grade, at(100, 0.0, 0, track_kind::main, traffic::mixed)), 6.0, 10.0, 20.0);
	tiers(plk(parameter::grade, at(100, 0.0, 0, track_kind::main, traffic::freight)), 6.0, 10.0, 20.0);
	tiers(plk(parameter::grade, at(100, 0.0, feature::short_stretch, track_kind::main, traffic::mixed)), 6.0, 17.5, 35.0);
	tiers(plk(parameter::grade, at(40, 0.0, feature::station)), 2.0, 2.0, 2.5);
	tiers(plk(parameter::grade, at(40, 0.0, feature::platform, track_kind::main, traffic::passenger)), 2.5, 6.0, 10.0);
}

TEST_CASE("norms PLK tab. 9.16: the shortest intermediate element between abrupt changes of the cant deficiency") {
	tiers(plk(parameter::intermediate_length, at(100)), none, none, 25.0);
	tiers(plk(parameter::intermediate_length, at(60)), none, none, 20.0);
	tiers(plk(parameter::intermediate_length, at(160)), none, none, 48.0);
	tiers(plk(parameter::intermediate_length, at(250)), none, none, 137.5);
	tiers(plk(parameter::intermediate_length, at(60, 0.0, feature::switch_track)), none, none, 6.0);
	// ST-T1-A6 przykład Z3-3: at 70 km/h on a connection 0.15 V = 10.5 m
	tiers(plk(parameter::intermediate_length, at(70, 0.0, feature::switch_track)), none, none, 10.5);
	tiers(plk(parameter::intermediate_length, at(150, 0.0, feature::switch_track)), none, none, 37.5);
	tiers(plk(parameter::intermediate_length, at(40, 0.0, 0, track_kind::side)), none, none, 4.0);
}

TEST_CASE("norms PLK tab. 8.1: the speed of the slow trains of the types of the lines") {
	CHECK(norms::slow_speed(traffic::passenger, 250) == 120);
	CHECK(norms::slow_speed(traffic::passenger, 200) == 120);
	CHECK(norms::slow_speed(traffic::mixed, 200) == 80);
	CHECK(norms::slow_speed(traffic::passenger, 160) == 120);
	CHECK(norms::slow_speed(traffic::mixed, 160) == 80);
	CHECK(norms::slow_speed(traffic::passenger, 120) == 80);
	CHECK(norms::slow_speed(traffic::mixed, 120) == 60);
	CHECK(norms::slow_speed(traffic::freight, 120) == 60);
	CHECK(norms::slow_speed(traffic::passenger, 80) == 60);
	CHECK(norms::slow_speed(traffic::mixed, 80) == 40);
	CHECK(norms::slow_speed(traffic::freight, 80) == 40);
	CHECK(norms::slow_speed(traffic::freight, 40) == 40);
	// between the types the next faster one, never above the highest speed itself
	CHECK(norms::slow_speed(traffic::passenger, 100) == 80);
	CHECK(norms::slow_speed(traffic::freight, 30) == 30);
}

TEST_CASE("norms TSI 4.2.4.3 tab. 8: the cant deficiency, 153 mm up to 300 km/h, 130 mm for the freight wagons") {
	tiers(tsi(parameter::cant_deficiency, at(160)), none, none, 153.0);
	tiers(tsi(parameter::cant_deficiency, at(300)), none, none, 153.0);
	tiers(tsi(parameter::cant_deficiency, at(301)), none, none, 100.0);
	tiers(tsi(parameter::cant_deficiency, at(160, 0.0, 0, track_kind::main, traffic::freight)), none, none, 130.0);
	tiers(tsi(parameter::cant_deficiency, at(160, 0.0, feature::tilting, track_kind::main, traffic::passenger)), none, none, none);
	CHECK(place(tsi(parameter::cant_deficiency, at(160))) == "4.2.4.3, tab. 8");
}

TEST_CASE("norms TSI 4.2.4.2 tab. 7: the cant, at the platforms, (R - 50) / 1.5 below R 305 m for mixed and freight traffic") {
	tiers(tsi(parameter::cant, at(100)), none, none, 160.0);
	tiers(tsi(parameter::cant, at(100, 0.0, 0, track_kind::main, traffic::passenger)), none, none, 180.0);
	tiers(tsi(parameter::cant, at(100, 0.0, feature::ballastless)), none, none, 170.0);
	tiers(tsi(parameter::cant, at(100, 0.0, feature::ballastless, track_kind::main, traffic::passenger)), none, none, 180.0);
	tiers(tsi(parameter::cant, at(100, 0.0, feature::platform, track_kind::main, traffic::passenger)), none, none, 110.0);
	tiers(tsi(parameter::cant, at(60, 300.0)), none, none, 160.0);
	tiers(tsi(parameter::cant, at(60, 250.0)), none, none, 200.0 / 1.5);
	tiers(tsi(parameter::cant, at(60, 250.0, 0, track_kind::main, traffic::passenger)), none, none, 180.0);
}

TEST_CASE("norms TSI 4.2.4.4: the abrupt change of the cant deficiency, 150 mm at 40 km/h below 75 mm on both sides") {
	tiers(tsi(parameter::deficiency_jump, at(40)), none, none, 130.0);
	tiers(tsi(parameter::deficiency_jump, at(40, 0.0, feature::low_deficiency)), none, none, 150.0);
	tiers(tsi(parameter::deficiency_jump, at(60, 0.0, feature::low_deficiency)), none, none, 130.0);
	tiers(tsi(parameter::deficiency_jump, at(60.5)), none, none, 125.0);
	tiers(tsi(parameter::deficiency_jump, at(200)), none, none, 125.0);
	tiers(tsi(parameter::deficiency_jump, at(230)), none, none, 85.0);
	tiers(tsi(parameter::deficiency_jump, at(230.5)), none, none, 25.0);
}

TEST_CASE("norms TSI 4.2.3.3, 4.2.3.4, 4.2.3.5: the grades, the least radius, the vertical curves") {
	tiers(tsi(parameter::radius, at(100, 300.0)), none, none, 150.0);
	tiers(tsi(parameter::vertical_radius, at(100, 0.0, feature::crest)), none, none, 500.0);
	tiers(tsi(parameter::vertical_radius, at(100, 0.0, feature::sag)), none, none, 900.0);
	tiers(tsi(parameter::grade, at(100, 0.0, 0, track_kind::main, traffic::passenger)), none, none, 35.0);
	tiers(tsi(parameter::grade, at(100, 0.0, 0, track_kind::main, traffic::mixed)), none, none, none);
	tiers(tsi(parameter::grade, at(40, 0.0, feature::station)), none, none, 2.5);
}

TEST_CASE("norms hard limits of every profile: the TSI and the cant gradient of 2.5 mm/m of EN 13803-1") {
	auto const gradient{norms::hard_limits(parameter::cant_gradient, at(100))};
	tiers(gradient, none, none, 2.5);
	CHECK(gradient.block->from.document == "EN 13803-1");
	auto const deficiency{norms::hard_limits(parameter::cant_deficiency, at(100))};
	tiers(deficiency, none, none, 153.0);
	CHECK(deficiency.block->from.document == "TSI INF");
	tiers(norms::hard_limits(parameter::radius, at(100, 300.0)), none, none, 150.0);
	// the grades are the profiles' own
	tiers(norms::hard_limits(parameter::grade, at(100, 0.0, 0, track_kind::main, traffic::passenger)), none, none, none);
}

TEST_CASE("norms EN 13803-1 5.2 and AB-EBV AB 17: the normal and the exceptional or maximal limits") {
	auto const en = [](parameter const What, norms::query const &Where) { return norms::limits(profile_id::en13803, What, Where); };
	auto const ch = [](parameter const What, norms::query const &Where) { return norms::limits(profile_id::ch, What, Where); };
	tiers(en(parameter::cant_deficiency, at(150)), none, 130.0, 183.0);
	tiers(en(parameter::cant_deficiency, at(80)), none, none, none);
	tiers(en(parameter::cant_gradient, at(100)), none, 2.25, 2.5);
	tiers(en(parameter::cant, at(100, 500.0)), none, 160.0, 180.0);
	tiers(en(parameter::radius, at(100, 500.0)), none, 190.0, 150.0);
	CHECK_FALSE(en(parameter::deficiency_jump, at(100)).warn->confirmed);
	tiers(ch(parameter::cant_deficiency, at(100, 300.0)), none, 122.0, 130.0);
	tiers(ch(parameter::cant_deficiency, at(100, 500.0)), none, 130.0, 150.0);
	tiers(ch(parameter::cant_deficiency, at(150, 100.0, feature::switch_track | feature::diverging)), none, 130.0 - 0.3 * 150.0, 155.0 - 0.35 * 150.0);
	tiers(ch(parameter::cant_rate, at(100)), none, 50.0, 60.0);
	tiers(ch(parameter::element_length, at(72)), none, 20.0, 14.0);
	tiers(ch(parameter::deficiency_jump, at(120)), none, 75.0 - 0.3125 * 120.0, none);
	tiers(ch(parameter::vertical_radius, at(100, 0.0, feature::sag)), none, 3500.0, 1750.0);
	tiers(ch(parameter::vertical_radius, at(40, 0.0, feature::sag)), none, 2000.0, 900.0);
}
