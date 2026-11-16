#include "stdafx.h"
#include "doctest.h"
#include "world/TrackCheck.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// physics.trackcheck: the geometry of the rail tracks of a scenery the criteria of derailment find suspicious, see
// world/TrackCheck.h

namespace tc = track_check;

namespace {

bool has(tc::findings const &Found, tc::kind const Kind) {
	return std::find(Found.kinds.begin(), Found.kinds.end(), Kind) != Found.kinds.end();
}

tc::track curve(double const Radius, double const Velocity, double const Cant = 0.0) {
	tc::track track;
	track.name = "t1";
	track.radius = Radius;
	track.velocity = Velocity;
	track.cant = {Cant, Cant};
	return track;
}

double overturning(double const Radius, double const Cant = 0.0) {
	auto const &eu07{tc::default_references().front().vehicle};
	return derailment::overturning_speed(Radius, std::asin(Cant / 1500.0), eu07.centre_of_gravity, 1.435) * 3.6;
}

} // namespace

TEST_CASE("trackcheck: the default vehicles") {
	auto const &references{tc::default_references()};
	REQUIRE(references.size() == 2);
	CHECK(references[0].name == "EU07");
	CHECK(references[0].vehicle.centre_of_gravity == doctest::Approx(1.65).epsilon(0.01));
	CHECK(references[1].vehicle.centre_of_gravity == doctest::Approx(1.42).epsilon(0.01));
	// the EU07 overturns at 108 km/h in a curve of 300 m without cant
	CHECK(overturning(300.0) == doctest::Approx(108.0).epsilon(0.02));
}

TEST_CASE("trackcheck: the speed limit of a curve") {
	auto const critical{overturning(300.0)};
	SUBCASE("under the speeds of the vehicles nothing") {
		CHECK(tc::check(curve(300.0, critical - 10.0)).empty());
	}
	SUBCASE("over the overturning speed of the EU07") {
		auto const found{tc::check(curve(300.0, critical + 5.0))};
		CHECK(has(found, tc::kind::overturning));
		CHECK_FALSE(has(found, tc::kind::unlimited_curve));
		CHECK(found.defective());
		CHECK(found.text.find("over the overturning speed of EU07") != std::string::npos);
		CHECK(found.text.find("speed limit 113 km/h") != std::string::npos);
	}
	SUBCASE("the cant raises it") {
		CHECK(tc::check(curve(300.0, critical + 5.0, 100.0)).empty());
	}
	SUBCASE("without a speed limit of its own it's no defect, the speed it takes is told") {
		auto const found{tc::check(curve(300.0, -1.0))};
		CHECK(has(found, tc::kind::unlimited_curve));
		CHECK_FALSE(has(found, tc::kind::overturning));
		CHECK_FALSE(found.defective());
		CHECK_FALSE(tc::is_defect(tc::kind::unlimited_curve));
		CHECK(found.unlimited_speed == doctest::Approx(critical));
		CHECK(found.text.starts_with("no speed limit set (not a defect): the curve takes at most "));
		CHECK(found.text.find(" km/h, the overturning speed of EU07") != std::string::npos);
		CHECK(found.text.find("speed limit none") != std::string::npos);
	}
	SUBCASE("nor anything when it takes them at the top speed of the EU07") {
		CHECK(tc::check(curve(600.0, -1.0)).empty());
	}
	SUBCASE("a tram loop of 30 m without a speed limit is no tight curve too fast") {
		// the tram tracks of Tomaszewo (tramwaj106...): no category of their own, no speed limit
		auto const found{tc::check(curve(30.0, -1.0))};
		CHECK(found.kinds == std::vector{tc::kind::unlimited_curve});
		CHECK(found.unlimited_speed == doctest::Approx(overturning(30.0)));
		CHECK(found.unlimited_speed < 40.0);
	}
	SUBCASE("a closed track (speed 0) is not checked") {
		CHECK(tc::check(curve(100.0, 0.0)).empty());
	}
	SUBCASE("nor the speed of a switch") {
		auto track{curve(150.0, 100.0)};
		track.plain = false;
		CHECK(tc::check(track).empty());
	}
	SUBCASE("the cant at the end with the less of it") {
		auto track{curve(300.0, critical + 5.0, 100.0)};
		track.cant[0] = 0.0;
		CHECK(has(tc::check(track), tc::kind::overturning));
	}
}

TEST_CASE("trackcheck: the cant") {
	SUBCASE("the same cant at both ends of a straight track") {
		auto const found{tc::check(curve(0.0, 100.0, 60.0))};
		CHECK(has(found, tc::kind::cant_on_straight));
		CHECK(found.text.starts_with("cant 60 / 60 mm on a straight track"));
	}
	SUBCASE("a transition has it at one end only") {
		auto track{curve(0.0, 100.0, 60.0)};
		track.cant[0] = 0.0;
		CHECK(tc::check(track).empty());
	}
	SUBCASE("a few mm are left alone") {
		CHECK(tc::check(curve(0.0, 100.0, 4.0)).empty());
	}
	SUBCASE("over 160 mm") {
		auto const found{tc::check(curve(1000.0, 60.0, 170.0))};
		CHECK(has(found, tc::kind::cant_excess));
		CHECK(found.kinds.size() == 1);
		CHECK(tc::check(curve(1000.0, 60.0, 150.0)).empty());
	}
}

TEST_CASE("trackcheck: a tight curve") {
	CHECK(has(tc::check(curve(30.0, 50.0)), tc::kind::tight_curve));
	CHECK_FALSE(has(tc::check(curve(30.0, 30.0)), tc::kind::tight_curve));
	CHECK_FALSE(has(tc::check(curve(60.0, 50.0)), tc::kind::tight_curve));
}

TEST_CASE("trackcheck: the unevenness") {
	auto track{curve(0.0, 100.0)};
	track.uneven.value = 14.f;
	track.uneven.side = 20.f;
	track.uneven.cross = 20.f;
	CHECK(tc::check(track).empty());
	track.uneven.side = 40.f;
	track.uneven.cross = 26.f;
	auto const found{tc::check(track)};
	CHECK(has(found, tc::kind::uneven));
	CHECK(found.text.starts_with("uneven side 40, cross 26 mm out of range"));
}

TEST_CASE("trackcheck: the scan of a scenery") {
	std::vector<tc::track> tracks;
	tracks.push_back(curve(300.0, 140.0));
	tracks.back().name = "fast";
	tracks.push_back(curve(0.0, 100.0));
	tracks.back().name = "worn";
	tracks.back().damage = 31;
	tracks.push_back(curve(0.0, 100.0, 50.0));
	tracks.back().name = "leaning";
	tracks.back().damage = 31;
	tracks.push_back(curve(0.0, 100.0));
	tracks.back().name = "fine";
	tracks.back().damage = 2;
	tracks.push_back(curve(30.0, -1.0));
	tracks.back().name = "tram_1";
	tracks.push_back(curve(150.0, -1.0));
	tracks.back().name = "siding_1";

	std::vector<std::string> log;
	auto const summary{tc::scan(tracks, [&](std::string const &Line) { log.push_back(Line); })};
	CHECK(summary.tracks == 6);
	// the curves without a speed limit are told apart from the defects
	CHECK(summary.suspicious == 2);
	CHECK(summary.unlimited == 2);
	CHECK(summary.slowest == doctest::Approx(overturning(30.0)));
	CHECK(summary.count(tc::kind::overturning) == 1);
	CHECK(summary.count(tc::kind::cant_on_straight) == 1);
	CHECK(summary.count(tc::kind::tight_curve) == 0);
	CHECK(summary.count(tc::kind::unlimited_curve) == 2);
	// the damage flags are counted, not told track by track
	CHECK(summary.damage.at(31) == 2);
	CHECK(summary.damage.at(2) == 1);
	REQUIRE(log.size() == 7);
	CHECK(log[0].starts_with("trackcheck: fast: speed limit 140 km/h over the overturning speed of EU07"));
	CHECK(log[1].starts_with("trackcheck: leaning: cant 50 / 50 mm on a straight track"));
	CHECK(log[1].ends_with("damage 31)"));
	CHECK(log[2].starts_with("trackcheck: tram_1: no speed limit set (not a defect): the curve takes at most "));
	CHECK(log[3].starts_with("trackcheck: siding_1: no speed limit set (not a defect)"));
	CHECK(log[4] == "trackcheck: 6 rail tracks checked, 2 suspicious: overturning 1; flange climbing 0; cant on a straight 1; cant too high 0; tight curve too fast 0; uneven out of range 0");
	CHECK(log[5].starts_with("trackcheck: curves without a speed limit: 2 taking the reference vehicles under 125 km/h (not a defect), the slowest at "));
	CHECK(log[6] == "trackcheck: damage flags: 2 on 1; 31 on 2");
	for (auto const &line : log) {
		CHECK(line.starts_with("trackcheck: "));
	}
}
