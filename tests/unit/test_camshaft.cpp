#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// eu07/maszyna#393: running backwards, a camshaft which reached the last position of the series connection (the
// next one already joins the motors in more branches than BackwardsBranchesAllowed) stayed there whatever the driver
// did with the master controller, with the line contactors closed and the motors pulling, until the line contactors
// were opened by hand

namespace {

// the starting resistances of the Skoda 181 (cd/181_v2): 6 motors in series up to position 22, then two branches of
// 3 up to 34, then three branches of 2 up to 43; columns: position, resistance, branches, motors in a branch, auto
nlohmann::ordered_json skoda_rlist() {
	nlohmann::ordered_json rows{nlohmann::ordered_json::array({0, 0, 0, 0, 0}), nlohmann::ordered_json::array({1, 999999, 1, 6, 0})};
	auto const series{std::vector<double>{15.87, 12.32, 10.03, 8.54, 6.98, 5.84, 5.35, 4.87, 3.88, 2.92, 2.68, 2.43, 2.19, 1.95, 1.70, 1.46, 1.22, 0.97, 0.49, 0.24, 0.00}};
	auto const seriesparallel{std::vector<double>{4.50, 3.63, 3.11, 2.42, 2.08, 1.73, 1.38, 1.04, 0.69, 0.35, 0.17, 0.00}};
	auto const parallel{std::vector<double>{2.78, 2.29, 1.80, 1.41, 1.02, 0.73, 0.44, 0.24, 0.00}};
	auto position{2};
	for (auto const resistance : series) {
		rows.push_back(nlohmann::ordered_json::array({position++, resistance, 1, 6, 0}));
	}
	for (auto const resistance : seriesparallel) {
		rows.push_back(nlohmann::ordered_json::array({position++, resistance, 2, 3, 0}));
	}
	for (auto const resistance : parallel) {
		rows.push_back(nlohmann::ordered_json::array({position++, resistance, 3, 2, 0}));
	}
	return rows;
}

// an electric locomotive with a camshaft, made after the Skoda 181
definition skoda() {
	auto result{definition::load("wagon")
	                .set("Param", "M", 124200)
	                .set("Param", "Vmax", 90)
	                .set("Dimensions", "L", 18.8)
	                .set("Wheels", "D", 1.25)
	                .set("Wheels", "Axle", "Co'Co'")
	                .set("Engine", "EngineType", "ElectricSeriesMotor")
	                .set("Engine", "Volt", 967)
	                .set("Engine", "Trans", "21:89")
	                .set("Engine", "WindingRes", 0.084)
	                .set("Engine", "nmax", 1870)
	                .set("Circuit", "CircuitRes", 0.01)
	                .set("Circuit", "ImaxLo", 550)
	                .set("Circuit", "ImaxHi", 550)
	                .set("Circuit", "IminLo", 307)
	                .set("Circuit", "IminHi", 332)
	                .set("Cntrl", "MCPN", 43)
	                .set("Cntrl", "SCPN", 3)
	                .set("Cntrl", "CoupledCtrl", "No")
	                .set("Cntrl", "IniCDelay", 0.2)
	                .set("Cntrl", "SCDelay", 0.18)
	                .set("Cntrl", "SCDDelay", 0.18)
	                .set("Cntrl", "Camshaft", "Yes")
	                .set("MotorParamTable0", "rows",
	                     nlohmann::ordered_json::array({nlohmann::ordered_json::array({0, 33.75, 57, 0.4, 225.70, 57, 0.4}), nlohmann::ordered_json::array({1, 31.66, 69, 0.4, 217.10, 69, 0.4}),
	                                                    nlohmann::ordered_json::array({2, 30.52, 90, 0.4, 212.80, 90, 0.4}), nlohmann::ordered_json::array({3, 28.74, 107, 0.4, 199.55, 107, 0.4})}))
	                .set("RList", "Size", 43)
	                .set("RList", "rows", skoda_rlist())};
	return result;
}

// the locomotive running with the line contactors closed and the camshaft at Position, the master controller moved
// to Controller, in the given direction; the time since the last step of the camshaft long enough for the next one
void running(TMoverParameters &Locomotive, int const Direction, int const Position, int const Controller) {
	Locomotive.CabActive = 1;
	Locomotive.DirActive = Direction;
	Locomotive.Mains = true;
	Locomotive.PipePress = 5.0; // bar, the pressure switch lets the line contactors close
	Locomotive.BrakePress = 0.0;
	Locomotive.StLinFlag = true;
	Locomotive.MainCtrlActualPos = Position;
	Locomotive.MainCtrlPos = Controller;
	Locomotive.LastRelayTime = 10.0;
}

// steps the camshaft as the physics does, each step once the delay since the previous one passed; returns where it
// ended up
int run_camshaft(TMoverParameters &Locomotive, int const Steps) {
	for (int step = 0; step < Steps; ++step) {
		Locomotive.LastRelayTime = 10.0;
		Locomotive.AutoRelayCheck();
	}
	return Locomotive.MainCtrlActualPos;
}

} // namespace

TEST_CASE("camshaft: running backwards, the camshaft goes back down from the last series position (eu07/maszyna#393)") {
	vehicle locomotive{skoda()};
	auto &mover{locomotive.mover()};
	REQUIRE(mover.HasCamshaft);
	REQUIRE(mover.RlistSize == 43);
	REQUIRE(mover.RList[22].Bn == 1);
	REQUIRE(mover.RList[23].Bn == 2);

	SUBCASE("the controller turned back a few positions") {
		running(mover, -1, 22, 15);
		CHECK(run_camshaft(mover, 1) == 21); // it used to stay at 22
		CHECK(run_camshaft(mover, 10) == 15);
	}
	SUBCASE("the controller turned back to zero: the camshaft runs down and the line contactors open") {
		running(mover, -1, 22, 0);
		run_camshaft(mover, 30);
		CHECK(mover.MainCtrlActualPos == 0);
		CHECK_FALSE(mover.StLinFlag); // the motors don't pull any more
	}
}

TEST_CASE("camshaft: running backwards, the camshaft still doesn't go past the series connection") {
	vehicle locomotive{skoda()};
	auto &mover{locomotive.mover()};
	running(mover, -1, 20, 43);
	CHECK(run_camshaft(mover, 10) == 22);

	SUBCASE("unless more branches are allowed backwards") {
		mover.BackwardsBranchesAllowed = 2;
		CHECK(run_camshaft(mover, 20) == 34);
	}
}

TEST_CASE("camshaft: running forwards, the camshaft goes through all the connections and back") {
	vehicle locomotive{skoda()};
	auto &mover{locomotive.mover()};
	running(mover, 1, 20, 43);
	CHECK(run_camshaft(mover, 30) == 43);

	mover.MainCtrlPos = 10;
	CHECK(run_camshaft(mover, 40) == 10);
}
