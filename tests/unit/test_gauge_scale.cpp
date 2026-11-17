#include "stdafx.h"
#include "doctest.h"
#include "consist.h"

#include "utilities/parser.h"
#include "vehicle/gaugescale.h"
#include "vehicle/mainctrlact.h"

// the controller position marker of the diesel locomotives (MaSzyna-EU07/maszyna#74): the mainctrlact: gauge of
// the ST43 follows its controller, on a scale whose marks are not evenly spaced (scaletable:)

using test::consist;
using test::definition;

namespace {

gauge_scale::table read(std::string const &Text) {
	cParser parser(Text, cParser::buffer_TEXT);
	return gauge_scale::read(parser);
}

} // namespace

TEST_CASE("gauge scale: a table is read with or without commas, the brackets touching the numbers or not") {
	for (auto const *text : {"[ 0 0, 1 0.5, 2 0.6 ]", "[0 0 1 0.5 2 0.6]", "[ 0 0,1 0.5,2 0.6]"}) {
		CAPTURE(text);
		auto const table{read(text)};
		CHECK(table.closed);
		CHECK_FALSE(table.block_end);
		CHECK_FALSE(table.malformed);
		REQUIRE(table.points.size() == 3);
		CHECK(table.points[1].value == doctest::Approx(1.0));
		CHECK(table.points[1].scaled == doctest::Approx(0.5));
	}
}

TEST_CASE("gauge scale: the table leaves the rest of the gauge's block to be read") {
	cParser parser("[ 0 0, 1 0.5 ] soundinc: x.wav }", cParser::buffer_TEXT);
	CHECK(gauge_scale::read(parser).closed);
	CHECK(parser.getToken<std::string>(true, "\n\r\t ,;") == "soundinc:");
}

TEST_CASE("gauge scale: a table without its closing bracket stops at the end of the block") {
	auto const table{read("[ 0 0, 1 0.5 }")};
	CHECK_FALSE(table.closed);
	CHECK(table.block_end);
	CHECK(table.points.size() == 2);
}

TEST_CASE("gauge scale: a value without what the gauge shows for it, or a word, makes a malformed table") {
	CHECK(read("[ 0 0, 1 ]").malformed);
	CHECK(read("[ 0 0, 1 x ]").malformed);
	CHECK(read("[ 0 0, 1 x ]").points.size() == 1);
}

TEST_CASE("gauge scale: the points are put in the order of their values") {
	auto const table{read("[ 2 0.6, 0 0, 1 0.5 ]")};
	REQUIRE(table.points.size() == 3);
	CHECK(table.points[0].value == doctest::Approx(0.0));
	CHECK(table.points[2].value == doctest::Approx(2.0));
}

TEST_CASE("gauge scale: the gauge shows what the table gives, interpolated between its points") {
	auto const table{read("[ 0 0, 1 -0.05, 2 -0.07, 4 -0.08 ]")};
	CHECK(gauge_scale::interpolate(table.points, 0.f) == doctest::Approx(0.0));
	CHECK(gauge_scale::interpolate(table.points, 1.f) == doctest::Approx(-0.05));
	CHECK(gauge_scale::interpolate(table.points, 0.5f) == doctest::Approx(-0.025)); // a marker on its way
	CHECK(gauge_scale::interpolate(table.points, 3.f) == doctest::Approx(-0.075));
	CHECK(gauge_scale::interpolate(table.points, 4.f) == doctest::Approx(-0.08));
	// past the ends the marker stays at them
	CHECK(gauge_scale::interpolate(table.points, -1.f) == doctest::Approx(0.0));
	CHECK(gauge_scale::interpolate(table.points, 9.f) == doctest::Approx(-0.08));
}

TEST_CASE("gauge scale: two points of the same value make a step") {
	auto const table{read("[ 0 0, 1 0.1, 1 0.5, 2 0.6 ]")};
	CHECK(gauge_scale::interpolate(table.points, 0.99f) == doctest::Approx(0.099));
	CHECK(gauge_scale::interpolate(table.points, 1.f) == doctest::Approx(0.5));
	CHECK(gauge_scale::interpolate(table.points, 1.5f) == doctest::Approx(0.55));
}

TEST_CASE("main ctrl act: the marker of a diesel locomotive with a scaletable: follows its controller (MaSzyna-EU07/maszyna#74)") {
	// nothing sets the actual position of a diesel engine's controller, the marker stayed at 0
	for (auto const *path : {"st44_v2/m62-1151", "sm42_v1/6d1", "su45_v2/301d"}) {
		CAPTURE(path);
		consist train;
		auto &locomotive{train.add(definition::fiz(path), 1).mover()};
		REQUIRE((locomotive.EngineType == TEngineType::DieselElectric || locomotive.EngineType == TEngineType::DieselEngine));
		REQUIRE(locomotive.EIMCtrlType == 0);
		locomotive.MainCtrlPos = 5;
		CHECK(locomotive.MainCtrlActualPos == 0);
		CHECK(main_ctrl_act::position(locomotive, true) == doctest::Approx(5.0));
		// one with a controller of the EIM kind keeps its actual position itself
		locomotive.EIMCtrlType = 2;
		CHECK(main_ctrl_act::position(locomotive, true) == doctest::Approx(0.0));
	}
}

TEST_CASE("main ctrl act: the marker of a diesel locomotive without a scaletable: stays still, as before") {
	// the shipped ST43 cabs define mainctrlact: with a linear scale nobody has seen working; it is not opted in
	for (auto const *path : {"st44_v2/m62-1151", "sm42_v1/6d1", "su45_v2/301d"}) {
		CAPTURE(path);
		consist train;
		auto &locomotive{train.add(definition::fiz(path), 1).mover()};
		REQUIRE(locomotive.EIMCtrlType == 0);
		locomotive.MainCtrlPos = 5;
		locomotive.ScndCtrlPos = 1;
		for (auto const coupled : {false, true}) {
			CAPTURE(coupled);
			locomotive.CoupledCtrl = coupled;
			CHECK(main_ctrl_act::position(locomotive, false) == doctest::Approx(0.0));
		}
	}
}

TEST_CASE("main ctrl act: the marker of an electric locomotive shows its actual position, as before") {
	for (auto const scaletable : {false, true}) {
		CAPTURE(scaletable);
		consist train;
		auto &locomotive{train.add(definition::fiz("et22_v3/201e-w"), 1).mover()};
		locomotive.MainCtrlPos = 10;
		locomotive.MainCtrlActualPos = 3;
		locomotive.ScndCtrlPos = 2;
		locomotive.ScndCtrlActualPos = 1;
		locomotive.CoupledCtrl = false;
		CHECK(main_ctrl_act::position(locomotive, scaletable) == doctest::Approx(3.0));
		locomotive.CoupledCtrl = true;
		CHECK(main_ctrl_act::position(locomotive, scaletable) == doctest::Approx(4.0));
	}
}
