#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "drive.h"

using test::definition;
using test::vehicle;

TEST_CASE("definition: the wagon from data/wagon.json loads with its values") {
	vehicle wagon{definition::load("wagon")};

	CHECK(wagon->Mass == doctest::Approx(42200.0));
	CHECK(wagon->Vmax == doctest::Approx(120.0));
	CHECK(wagon->Dim.L == doctest::Approx(26.8));
	CHECK(wagon->WheelDiameter == doctest::Approx(0.96));
	CHECK(wagon->NAxles == 4);
	CHECK(wagon->MaxBrakePress[3] == doctest::Approx(3.8));
	CHECK(wagon->BrakeSystem == TBrakeSystem::Pneumatic);
}

TEST_CASE("definition: keys set by the test reach the vehicle") {
	vehicle heavy{definition::load("wagon").set("Param", "M", 60000).set("Wheels", "D", 1.0)};

	CHECK(heavy->Mass == doctest::Approx(60000.0));
	CHECK(heavy->WheelDiameter == doctest::Approx(1.0));
}

TEST_CASE("definition: an unknown test vehicle is an error, not a silently empty one") {
	CHECK_THROWS(definition::load("no such vehicle"));
}

TEST_CASE("definition: a misspelt section is an error, not silently skipped by the loader") {
	CHECK_THROWS(definition::load("wagon").set("Brakes", "MBF", 30));
}

TEST_CASE("definition: rows of a table section reach the vehicle") {
	auto const pc1{light::headlight_left | light::headlight_upper | light::headlight_right};
	vehicle wagon{definition::load("wagon")
	                  .set("LightsList", "Size", 2)
	                  .set("LightsList", "Default", 1)
	                  .set("LightsList", "rows", {{0, 0}, {pc1, 0}})};

	CHECK(wagon->LightsPosNo == 2);
	CHECK(wagon->LightsPos == 1);
	CHECK(wagon->Lights[end::front][1] == pc1);
}

TEST_CASE("definition: lines the loader skips or trims: comments, a hash anywhere, leading spaces, carriage returns") {
	auto text{definition::load("wagon").text()};
	// a commented line and one with a hash set the mass the wagon doesn't get; the rest, indented and ended with
	// a carriage return, still load
	auto const param{text.find("Param")};
	REQUIRE(param != std::string::npos);
	text.insert(param, "// Param: M=11111\nParam: M=22222 # a note\n   ");
	std::string crlf;
	for (auto const c : text) {
		if (c == '\n') {
			crlf += '\r';
		}
		crlf += c;
	}
	TMoverParameters wagon(0.0, "wagon", "test", 0);
	REQUIRE(wagon.LoadFIZ_Text(crlf));
	CHECK(wagon.Mass == doctest::Approx(42200.0));
	CHECK(wagon.Vmax == doctest::Approx(120.0));
	CHECK(wagon.MaxBrakePress[3] == doctest::Approx(3.8));
}

TEST_CASE("definition: a compressor list, whose positions allow the compressor and scale its speed and limits") {
	// none of the game's vehicles has one. its rows are the positions after the 0th, which changes nothing: whether
	// the compressor may run (2 yes, 1 no, 0 as the switch says), its speed, and its lower and upper limits. the
	// table holds whole numbers (int CompressorList[4][9]), so a factor of 0.9 is read as 0, and both limits
	// drop to nothing
	auto eu07{definition::fiz("4e_v1/4e")
	              .set("CompressorList", "Size", 3)
	              .set("CompressorList", "Default", 1)
	              .set("CompressorList", "rows", {{2, 2, 1, 1}, {1, 1, 1, 1}, {2, 1, 0.9, 0.9}})};
	vehicle real{eu07, 0.0, "test", 1};
	CHECK(real->CompressorListPosNo == 3);
	CHECK(real->CompressorList[3][3] == 0);
	auto driven{test::driving(real)};
	test::track const line{0.15, 0.0, test::catenary_3kv};
	REQUIRE(test::start_up(driven, line));
	CHECK(real->CompressorListPos == 1);
	test::drive(driven, line, 60.0);
	CHECK(real->CompressorAllow);
	CHECK(real->Compressor <= real->MaxCompressor + 0.05);
	CHECK(real->Compressor >= real->MinCompressor - 0.05);
	// the second position doesn't let it run
	CHECK(real->ChangeCompressorPreset(2));
	test::drive(driven, line, 2.0);
	CHECK_FALSE(real->CompressorAllow);
	CHECK_FALSE(real->CompressorFlag);
	// the third lets it, under limits of nothing: with no span between them for the governor, the reservoir over
	// the upper limit gets vented while the compressor runs on
	CHECK(real->ChangeCompressorPreset(3));
	auto const full{real->Compressor};
	test::drive(driven, line, 10.0);
	CHECK(real->CompressorAllow);
	CHECK(real->CompressorFlag);
	CHECK(real->Compressor < full - 1.0);
	// no further than the last position
	CHECK_FALSE(real->ChangeCompressorPreset(5));
	CHECK(real->CompressorListPos == 3);
}

TEST_CASE("definition: the compressor's limits set apart for the rear cab") {
	auto eu07{definition::fiz("4e_v1/4e").set("Brake", "MinCP_B", 5.0).set("Brake", "MaxCP_B", 6.0)};
	for (auto const cab : {1, -1}) {
		CAPTURE(cab);
		vehicle real{eu07, 0.0, "test", cab};
		auto driven{test::driving(real)};
		test::track const line{0.15, 0.0, test::catenary_3kv};
		REQUIRE(test::start_up(driven, line));
		test::drive(driven, line, 120.0);
		CHECK(real->CabDependentCompressor);
		CHECK(real->MaxCompressor == doctest::Approx(cab > 0 ? 8.7 : 6.0));
		CHECK(real->MinCompressor == doctest::Approx(cab > 0 ? 7.0 : 5.0));
		CHECK(real->Compressor <= real->MaxCompressor + 0.05);
		CHECK(real->Compressor >= real->MinCompressor - 0.05);
	}
}

TEST_CASE("definition: the highest power of an induction motor drive by the speed (PmaxList)") {
	// the drive's power limit (eimc_p_Pmax) follows the table, between its rows and, by the last two, past its end
	auto eu47{definition::fiz("e186_v2/eu47").set("PmaxList", "Size", 3).set("PmaxList", "rows", {{0, 1000}, {50, 3000}, {100, 5600}})};
	auto const table{[](double Speed) { return Speed < 50.0 ? 1000.0 + 40.0 * Speed : 3000.0 + 52.0 * (Speed - 50.0); }};
	for (auto const speed : {25.0, 75.0, 150.0}) {
		CAPTURE(speed);
		vehicle real{eu47, speed, "test", 1};
		auto driven{test::driving(real)};
		test::track const line{0.0, 0.0, test::catenary_3kv};
		REQUIRE(test::start_up(driven, line));
		CHECK(real->eimc[eimc_p_Pmax] == doctest::Approx(table(real->Vel)).epsilon(0.01));
	}
}

TEST_CASE("definition: the colour and the brightness of the headlights") {
	// only the 214M's own file has the section, which the definitions here can't hold (see data/fiz/README.md)
	vehicle railbus{definition::fiz("21xm_v1/214ma")
	                    .set("Headlights", "LampRed", 248)
	                    .set("Headlights", "LampGreen", 221)
	                    .set("Headlights", "LampBlue", 111)
	                    .set("Headlights", "DimmedMultiplier", 0.4)
	                    .set("Headlights", "NormalMultiplier", 0.9)};
	CHECK(railbus->refR == doctest::Approx(248.0));
	CHECK(railbus->refG == doctest::Approx(221.0));
	CHECK(railbus->refB == doctest::Approx(111.0));
	CHECK(railbus->dimMultiplier == doctest::Approx(0.4));
	CHECK(railbus->normMultiplier == doctest::Approx(0.9));
}
