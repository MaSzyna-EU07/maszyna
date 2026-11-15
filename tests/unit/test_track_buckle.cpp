#include "stdafx.h"
#include "doctest.h"
#include "world/TrackBuckle.h"

#include <cmath>

// the buckling of welded track in the heat, world/TrackBuckle.h: the shape the track takes, the force and the temperature of the rails,
// the limits it buckles at and the draws, repeated with the same seed

namespace {

double constexpr pi_squared{9.869604401089358};

int count_buckled(double const Rise, track_buckle::limits const &Limits, std::uint32_t const Seed, int const Passes)
{
	auto count{0};
	for (int pass = 1; pass <= Passes; ++pass)
	{
		auto const draw{track_buckle::draw(track_buckle::name_hash("tor_1234"), Seed, track_buckle::pass, static_cast<std::uint64_t>(pass))};
		if (track_buckle::decide(Rise, Limits, true, draw) == track_buckle::verdict::under_train)
			++count;
	}
	return count;
}

} // namespace

TEST_CASE("the force in a rail held from growing longer is E*A*alpha*dT")
{
	// PLK ST-T1-A8 designs for 35 K, 665 kN in a 60E1 rail, with its own alpha of about 1.18e-5
	auto const force{track_buckle::thermal_force(track_buckle::area_60e1, 35.0)};
	CHECK(force == doctest::Approx(650e3).epsilon(0.03));
	CHECK(force == doctest::Approx(665e3).epsilon(0.03));
	// Kish, Samavedam: about 2.5 kips a rail for each degree F in a 136 lb rail (13.35 sq in), 20.0 kN/K
	CHECK(track_buckle::thermal_force(13.35 * 6.4516e-4, 1.0) == doctest::Approx(20.0e3).epsilon(0.05));
	// lighter rails press less
	CHECK(track_buckle::thermal_force(track_buckle::area_49e1, 35.0) < force);
	CHECK(track_buckle::rail_area("49E1_rail") == track_buckle::area_49e1);
	CHECK(track_buckle::rail_area("S42") == track_buckle::area_s42);
	CHECK(track_buckle::rail_area("") == track_buckle::area_60e1);
}

TEST_CASE("a rail is as warm as the air at night or under clouds, 17 K warmer in full sun")
{
	CHECK(track_buckle::rail_temperature(30.0, -10.0, 0.0) == doctest::Approx(30.0));
	CHECK(track_buckle::rail_temperature(30.0, 90.0, 0.0) == doctest::Approx(47.0));
	CHECK(track_buckle::rail_temperature(30.0, 90.0, 1.0) == doctest::Approx(30.0));
	// rain cools as much as clouds
	CHECK(track_buckle::rail_temperature(30.0, 90.0, 2.0) == doctest::Approx(30.0));
	CHECK(track_buckle::rail_temperature(30.0, 30.0, 0.0) == doctest::Approx(38.5));
	CHECK(track_buckle::rail_temperature(30.0, 60.0, 0.5) == doctest::Approx(30.0 + 17.0 * std::sin(60.0 * 3.141592653589793 / 180.0) * 0.5));
}

TEST_CASE("the limits of the grades meet the heat the tracks of PLK deform in")
{
	track_buckle::cwr track;
	auto const average{track_buckle::limits_of(track, 0.0, 0.0)};
	// at the neutral 26 C under a train from rails at 58 C, by itself from 71 C: PLK, deformations with the rails above 70 C
	CHECK(track.neutral + average.low == doctest::Approx(58.0));
	CHECK(track.neutral + average.high == doctest::Approx(71.0));
	// Kish, Samavedam: the chance of a buckle of a straight track grows fast from rails at 63 C with the neutral at 27 C, 36 K
	CHECK(36.0 > average.low);
	CHECK(36.0 < average.high);
	// the design of PLK, 35 K, is short of where a good track buckles under a train
	track.grade = track_buckle::grade::good;
	CHECK(track_buckle::limits_of(track, 0.0, 0.0).low > 35.0);
	// worked lately: weak whatever the grade
	track.worked = true;
	auto const worked{track_buckle::limits_of(track, 0.0, 0.0)};
	CHECK(worked.low == doctest::Approx(22.0));
	CHECK(worked.high == doctest::Approx(32.0));
	// curves and an uneven line lower them
	track = {};
	CHECK(track_buckle::limits_of(track, 500.0, 0.0).low == doctest::Approx(29.0));
	CHECK(track_buckle::limits_of(track, -250.0, 0.0).low == doctest::Approx(26.0));
	CHECK(track_buckle::limits_of(track, 2000.0, 0.0).low == doctest::Approx(32.0));
	CHECK(track_buckle::limits_of(track, 0.0, 20.0).high == doctest::Approx(42.0));
	// FRA, the dynamic test: a 7.5 degree curve (233 m) with a 19 mm misalignment buckled under the train at 34 K
	auto const fra{track_buckle::limits_of(track, 233.0, 19.0)};
	CHECK(34.0 > fra.low);
	CHECK(34.0 < fra.high);
}

TEST_CASE("below the lower limit a track never buckles, at the upper one it buckles by itself")
{
	track_buckle::limits const limits{32.0, 45.0};
	for (auto const draw : {0.0, 0.3, 0.999})
	{
		CHECK(track_buckle::decide(31.9, limits, true, draw) == track_buckle::verdict::none);
		CHECK(track_buckle::decide(31.9, limits, false, draw) == track_buckle::verdict::none);
		CHECK(track_buckle::decide(45.0, limits, false, draw) == track_buckle::verdict::by_itself);
		// in between only under a train
		CHECK(track_buckle::decide(44.0, limits, false, draw) == track_buckle::verdict::none);
	}
	CHECK(track_buckle::chance(32.0, limits) == doctest::Approx(0.0));
	CHECK(track_buckle::chance(38.5, limits) == doctest::Approx(0.5));
	CHECK(track_buckle::chance(50.0, limits) == doctest::Approx(1.0));
	// a train going slowly and not braking doesn't stir it up
	CHECK_FALSE(track_buckle::stirs(30.0, false));
	CHECK(track_buckle::stirs(30.0, true));
	CHECK(track_buckle::stirs(60.0, false));
	CHECK_FALSE(track_buckle::stirs(3.0, true));
}

TEST_CASE("the share of the passes that buckle a track grows with the heat, and repeats with the seed")
{
	track_buckle::limits const limits{32.0, 45.0};
	auto const passes{4000};
	auto const quarter{count_buckled(32.0 + 0.25 * 13.0, limits, 7, passes)};
	auto const most{count_buckled(32.0 + 0.75 * 13.0, limits, 7, passes)};
	CHECK(quarter == doctest::Approx(0.25 * passes).epsilon(0.1));
	CHECK(most == doctest::Approx(0.75 * passes).epsilon(0.05));
	CHECK(count_buckled(31.0, limits, 7, passes) == 0);
	// the same seed and track, the same passes
	CHECK(count_buckled(38.0, limits, 7, passes) == count_buckled(38.0, limits, 7, passes));
	auto const key{track_buckle::name_hash("tor_1234")};
	CHECK(track_buckle::draw(key, 7, track_buckle::pass, 3) == track_buckle::draw(key, 7, track_buckle::pass, 3));
	// another seed, other draws
	auto differ{0};
	for (std::uint64_t pass = 1; pass <= 100; ++pass)
		if (track_buckle::draw(key, 7, track_buckle::pass, pass) != track_buckle::draw(key, 8, track_buckle::pass, pass))
			++differ;
	CHECK(differ == 100);
	// and another track
	CHECK(track_buckle::draw(track_buckle::name_hash("tor_1235"), 7, track_buckle::pass, 3) != track_buckle::draw(key, 7, track_buckle::pass, 3));
	// the draws spread over 0 to 1
	auto low{0};
	for (std::uint64_t pass = 1; pass <= 1000; ++pass)
	{
		auto const draw{track_buckle::draw(key, 7, track_buckle::pass, pass)};
		CHECK(draw >= 0.0);
		CHECK(draw < 1.0);
		if (draw < 0.5)
			++low;
	}
	CHECK(low == doctest::Approx(500).epsilon(0.1));
}

TEST_CASE("the heat buckles a track at one place in about 10 km, and wherever the bed was worked on")
{
	auto sites{0};
	auto const tracks{20000};
	for (int track = 0; track < tracks; ++track)
		if (track_buckle::weak_spot(track_buckle::name_hash("t" + std::to_string(track)), 3, 500.0, false))
			++sites;
	// 500 m each, 10 000 km together: about 1000
	CHECK(sites == doctest::Approx(1000).epsilon(0.1));
	CHECK(track_buckle::weak_spot(track_buckle::name_hash("t1"), 3, 1.0, true));
}

TEST_CASE("the buckle has its amplitude in the middle, side lobes the other way and ends smoothly")
{
	track_buckle::shape const shape{0.4, 14.0, 60.0};
	CHECK(track_buckle::offset(shape, 60.0) == doctest::Approx(0.4));
	CHECK(track_buckle::slope(shape, 60.0) == doctest::Approx(0.0));
	// the main lobe crosses the line one length apart
	CHECK(track_buckle::offset(shape, 60.0 - 7.0) == doctest::Approx(0.0));
	CHECK(track_buckle::offset(shape, 60.0 + 7.0) == doctest::Approx(0.0));
	// the side lobes the other way, a quarter as far out
	CHECK(track_buckle::offset(shape, 60.0 + 14.0) == doctest::Approx(-0.1));
	CHECK(track_buckle::offset(shape, 60.0 - 14.0) == doctest::Approx(-0.1));
	// into the straight track at the ends with no displacement, slope or curvature
	CHECK(track_buckle::reach(shape) == doctest::Approx(21.0));
	for (auto const at : {-21.0 + 1e-6, 21.0 - 1e-6})
	{
		CAPTURE(at);
		CHECK(track_buckle::offset(shape, 60.0 + at) == doctest::Approx(0.0));
		CHECK(track_buckle::slope(shape, 60.0 + at) == doctest::Approx(0.0));
		CHECK(track_buckle::bend(shape, 60.0 + at) == doctest::Approx(0.0));
	}
	// nothing beyond
	CHECK(track_buckle::offset(shape, 60.0 + 21.5) == 0.0);
	CHECK(track_buckle::bend(shape, 60.0 - 30.0) == 0.0);
	// symmetric
	for (auto const at : {1.0, 4.0, 7.5, 12.0, 16.0, 20.0})
		CHECK(track_buckle::offset(shape, 60.0 + at) == doctest::Approx(track_buckle::offset(shape, 60.0 - at)));
	// the largest curvature 11/9 pi^2 a / L^2 in the middle, bending back towards the line
	auto const peak{track_buckle::peak_bend(0.4, 14.0)};
	CHECK(peak == doctest::Approx(11.0 / 9.0 * pi_squared * 0.4 / (14.0 * 14.0)));
	CHECK(track_buckle::bend(shape, 60.0) == doctest::Approx(-peak));
	// none larger anywhere, and the curvature with no jumps
	auto previous{track_buckle::bend(shape, 35.0)};
	for (auto step = 0; step <= 5000; ++step)
	{
		auto const at{35.0 + 0.01 * step};
		auto const bend{track_buckle::bend(shape, at)};
		CHECK(std::abs(bend) <= peak * 1.0001);
		CHECK(std::abs(bend - previous) < 0.01 * peak);
		previous = bend;
	}
	// the curvature as the displacement has it
	auto const step{0.01};
	for (auto const at : {42.0, 50.5, 55.0, 61.0, 66.0, 73.0, 79.0})
	{
		CAPTURE(at);
		auto const slope{(track_buckle::offset(shape, at + step) - track_buckle::offset(shape, at - step)) / (2.0 * step)};
		auto const bend{(track_buckle::offset(shape, at + step) - 2.0 * track_buckle::offset(shape, at) + track_buckle::offset(shape, at - step)) / (step * step)};
		CHECK(track_buckle::slope(shape, at) == doctest::Approx(slope).epsilon(1e-4));
		CHECK(track_buckle::bend(shape, at) == doctest::Approx(bend).epsilon(1e-3));
	}
	// to the left
	track_buckle::shape const left{-0.4, 14.0, 60.0};
	CHECK(track_buckle::offset(left, 60.0) == doctest::Approx(-0.4));
	CHECK(track_buckle::bend(left, 60.0) == doctest::Approx(peak));
	// none
	CHECK(track_buckle::offset(track_buckle::shape{}, 60.0) == 0.0);
	CHECK_FALSE(track_buckle::active(track_buckle::shape{}));
}

TEST_CASE("a bogie feels the curvature of the buckle averaged over its wheelbase, a body that of its pivots")
{
	track_buckle::shape const shape{0.3, 20.0, 50.0};
	auto const peak{track_buckle::peak_bend(0.3, 20.0)};
	CHECK(track_buckle::mean_bend(shape, 50.0, 0.0) == doctest::Approx(track_buckle::bend(shape, 50.0)));
	// a bogie 2.5 m long, almost the whole of it
	CHECK(std::abs(track_buckle::mean_bend(shape, 50.0, 2.5)) > 0.95 * peak);
	// the mean of the curvature over the stretch, anywhere
	for (auto const at : {32.0, 42.0, 50.0, 57.5, 64.0})
	{
		CAPTURE(at);
		auto sum{0.0};
		auto const parts{2500};
		for (int part = 0; part < parts; ++part)
			sum += track_buckle::bend(shape, at - 1.25 + (part + 0.5) * 2.5 / parts);
		CHECK(track_buckle::mean_bend(shape, at, 2.5) == doctest::Approx(sum / parts).epsilon(1e-3));
	}
	// a body on the pivots 0 apart is a bogie
	CHECK(track_buckle::body_bend(shape, 47.0, 0.0, 2.5) == doctest::Approx(track_buckle::mean_bend(shape, 47.0, 2.5)));
	// a coach with its pivots 17 m apart over the main lobe: its pivots are on the flanks, which bend the other way
	auto const across{track_buckle::body_bend(shape, 50.0, 17.0, 2.5)};
	CHECK(std::abs(across) < 0.5 * peak);
	// the most the body feels, at any place, is less than a bogie does
	auto most{0.0};
	for (auto step = 0; step <= 1200; ++step)
		most = std::max(most, std::abs(track_buckle::body_bend(shape, 20.0 + 0.05 * step, 17.0, 2.5)));
	CHECK(most < 0.95 * peak);
	CHECK(most > 0.3 * peak);
}

TEST_CASE("the equivalent radius of a buckle")
{
	// the default ones as sharp as the raised cosine lobes of the research note: 0.4 m by 18 m on a straight 41 m, 0.2 by 14 in a curve 50 m
	CHECK(1.0 / track_buckle::peak_bend(track_buckle::straight_amplitude, track_buckle::straight_length) == doctest::Approx(41.0).epsilon(0.02));
	CHECK(1.0 / track_buckle::peak_bend(track_buckle::curve_amplitude, track_buckle::curve_length) == doctest::Approx(49.6).epsilon(0.02));
	CHECK(1.0 / track_buckle::peak_bend(0.3, 20.0) == doctest::Approx(110.5).epsilon(0.01));
}

TEST_CASE("the entries buckle and cwr are read and written back")
{
	auto const buckle{track_buckle::read_buckle("400;18;60")};
	CHECK(buckle.amplitude == 400.f);
	CHECK(buckle.length == 18.f);
	CHECK(buckle.centre == 60.f);
	CHECK(buckle.side == 0);
	CHECK(track_buckle::write_buckle(buckle) == "buckle 400;18;60 ");
	auto const left{track_buckle::read_buckle("250;;;-1")};
	CHECK(left.amplitude == 250.f);
	CHECK(left.length < 0.f);
	CHECK(left.centre < 0.f);
	CHECK(left.side == -1);
	CHECK(track_buckle::write_buckle(left) == "buckle 250;;;-1 ");
	CHECK(track_buckle::read_buckle(track_buckle::write_buckle(left).substr(7)).side == -1);
	// kept within bounds
	auto const wild{track_buckle::read_buckle("5000;100;-5;3")};
	CHECK(wild.amplitude == 1000.f);
	CHECK(wild.length == 60.f);
	CHECK(wild.centre < 0.f);
	CHECK(wild.side == 1);
	CHECK(track_buckle::read_buckle("300;1").length == 4.f);
	// none
	CHECK_FALSE(track_buckle::given(track_buckle::read_buckle("0;18;60")));
	CHECK_FALSE(track_buckle::given(track_buckle::read_buckle("x")));
	CHECK(track_buckle::write_buckle(track_buckle::declared{}).empty());

	auto const cwr{track_buckle::read_cwr("20;2;1")};
	CHECK(cwr.neutral == 20.f);
	CHECK(cwr.grade == track_buckle::grade::weak);
	CHECK(cwr.worked);
	CHECK(track_buckle::write_cwr(cwr) == "cwr 20;2;1 ");
	auto const partial{track_buckle::read_cwr("22")};
	CHECK(partial.neutral == 22.f);
	CHECK(partial.grade == track_buckle::grade::average);
	CHECK_FALSE(partial.worked);
	CHECK(track_buckle::write_cwr(partial) == "cwr 22;1 ");
	auto const bounded{track_buckle::read_cwr("99;7")};
	CHECK(bounded.neutral == 60.f);
	CHECK(bounded.grade == 2);
	// the default isn't written
	CHECK(track_buckle::write_cwr(track_buckle::read_cwr("26;1;0")).empty());
}

TEST_CASE("a buckle is laid into its track away from the ends, out of a curve")
{
	auto const values{track_buckle::read_buckle("400;18;60")};
	auto const straight{track_buckle::resolve(values, 200.0, 0, 0.0, 0.5, 0.2)};
	CHECK(straight.centre == 60.0);
	CHECK(straight.length == 18.0);
	// the side drawn on a straight
	CHECK(straight.amplitude == doctest::Approx(-0.4));
	CHECK(track_buckle::resolve(values, 200.0, 0, 0.0, 0.5, 0.7).amplitude == doctest::Approx(0.4));
	// out of a curve, whatever the draw
	CHECK(track_buckle::resolve(values, 200.0, 1, 400.0, 0.5, 0.2).amplitude == doctest::Approx(0.4));
	CHECK(track_buckle::resolve(values, 200.0, -1, 400.0, 0.5, 0.7).amplitude == doctest::Approx(-0.4));
	// the side given wins
	CHECK(track_buckle::resolve(track_buckle::read_buckle("400;18;60;-1"), 200.0, 1, 400.0, 0.5, 0.7).amplitude == doctest::Approx(-0.4));
	// the centre kept away from the ends by the reach, 1.5 lengths, and a margin
	CHECK(track_buckle::resolve(track_buckle::read_buckle("400;18;5"), 200.0, 0, 0.0, 0.5, 0.2).centre == doctest::Approx(29.0));
	CHECK(track_buckle::resolve(track_buckle::read_buckle("400;18;195"), 200.0, 0, 0.0, 0.5, 0.2).centre == doctest::Approx(171.0));
	// drawn where not given
	CHECK(track_buckle::resolve(track_buckle::read_buckle("400;18"), 200.0, 0, 0.0, 0.0, 0.2).centre == doctest::Approx(29.0));
	CHECK(track_buckle::resolve(track_buckle::read_buckle("400;18"), 200.0, 0, 0.0, 1.0, 0.2).centre == doctest::Approx(171.0));
	// the length by the shape of the track: a curve goes out less far and shorter
	CHECK(track_buckle::resolve(track_buckle::read_buckle("200"), 200.0, 1, 400.0, 0.5, 0.2).length == track_buckle::curve_length);
	CHECK(track_buckle::resolve(track_buckle::read_buckle("400"), 200.0, 0, 0.0, 0.5, 0.2).length == track_buckle::straight_length);
	CHECK(track_buckle::resolve(track_buckle::read_buckle("400"), 200.0, 1, 3000.0, 0.5, 0.2).length == track_buckle::straight_length);
	// too short a track
	CHECK_FALSE(track_buckle::active(track_buckle::resolve(values, 57.9, 0, 0.0, 0.5, 0.2)));
	CHECK(track_buckle::active(track_buckle::resolve(values, 58.0, 0, 0.0, 0.5, 0.2)));
	// the natural ones
	CHECK(track_buckle::natural(true).amplitude == 200.f);
	CHECK(track_buckle::natural(false).length == 14.f);
}
