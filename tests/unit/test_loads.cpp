#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

// characterisation of the loads of vehicles besides the loading by commands (test_commands.cpp): the load
// assigned to a vehicle (AssignLoad), with the pantographs of an electric vehicle set up by the "pantstate"
// load of a scenario, and the steps of loading and unloading (LoadingDone)

using test::definition;
using test::vehicle;

TEST_CASE("loads: the pantographs of an electric vehicle set up by the amount of its pantstate load") {
	// bit 0 raises the front pantograph, bit 1 the rear one, bit 2 turns the vehicle round in the train first
	// (DoubleTr), so the bits name the pantographs the other way round
	struct setup {
		int amount;
		bool front;
		bool rear;
	};
	for (auto const &[amount, front, rear] : {setup{0, false, false}, setup{1, true, false}, setup{2, false, true}, setup{3, true, true}, setup{5, false, true},
	                                          setup{6, true, false}}) {
		CAPTURE(amount);
		vehicle et42{definition::fiz("et42_v2/112e-a")};
		CHECK(et42->AssignLoad("pantstate", static_cast<float>(amount)));
		CHECK(et42->DoubleTr == ((amount & 4) != 0 ? -1 : 1));
		CHECK(et42->Pantographs[end::front].valve.is_enabled == front);
		CHECK(et42->Pantographs[end::rear].valve.is_enabled == rear);
		// the load itself stays as it was
		CHECK(et42->LoadAmount == doctest::Approx(0.0));
	}
	// a vehicle without a current collector takes no such load
	vehicle wagon{definition::fiz("3wl_v2/3wl")};
	CHECK_FALSE(wagon->AssignLoad("pantstate", 3.f));
}

TEST_CASE("loads: a load assigned, of one kind at a time, up to the vehicle's capacity, and emptied") {
	vehicle wagon{definition::fiz("3wl_v2/3wl")};
	auto const empty{wagon->TotalMass};
	REQUIRE(wagon->AssignLoad("coal", 10.f));
	CHECK(wagon->LoadTypeChange);
	CHECK(wagon->LoadType.name == "coal");
	CHECK(wagon->TotalMass > empty);
	// more of the same: the amount set, not added, and no more than the vehicle takes
	wagon->LoadTypeChange = false;
	REQUIRE(wagon->AssignLoad("coal", 1000.f));
	CHECK_FALSE(wagon->LoadTypeChange);
	CHECK(wagon->LoadAmount == doctest::Approx(wagon->MaxLoad));
	// another kind can't be added to a load, nor can an unknown one be loaded
	CHECK_FALSE(wagon->AssignLoad("ruda", 5.f));
	CHECK(wagon->LoadType.name == "coal");
	// emptied, with no name
	REQUIRE(wagon->AssignLoad(""));
	CHECK(wagon->LoadTypeChange);
	CHECK(wagon->LoadAmount == doctest::Approx(0.0));
	CHECK(wagon->LoadType.name.empty());
	CHECK_FALSE(wagon->AssignLoad("unknown", 5.f));
	CHECK(wagon->AssignLoad("ruda", 5.f));
}

TEST_CASE("loads: the steps of loading and unloading") {
	// most of the game's wagons may be overloaded (OverLoadFactor), the 3WL not
	vehicle wagon{definition::fiz("3wl_v2/3wl").set("Load", "OverLoadFactor", 1.5)};
	REQUIRE(wagon->AssignLoad("coal", 10.f));
	auto const capacity{wagon->MaxLoad * (1.0 + wagon->OverLoadFactor)};

	SUBCASE("no speed of the change: done at once") {
		CHECK(wagon->LoadingDone(0.0, "coal"));
		CHECK(wagon->LoadStatus == 4);
	}
	SUBCASE("no load, or another one, named: done when it was") {
		wagon->LoadStatus = 1;
		CHECK_FALSE(wagon->LoadingDone(1.0, ""));
		CHECK_FALSE(wagon->LoadingDone(1.0, "ruda"));
		wagon->LoadStatus = 4;
		CHECK(wagon->LoadingDone(1.0, ""));
	}
	SUBCASE("unloading, step by step, what was asked or all of it, the name going with the last of it") {
		wagon->CommandIn.Value1 = 4.0; // asked to unload
		wagon->LastLoadChangeTime = 2.0; // at 1 a second
		CHECK_FALSE(wagon->LoadingDone(-1.0, "coal"));
		CHECK(wagon->LoadStatus == 2);
		CHECK(wagon->LoadAmount == doctest::Approx(8.0));
		CHECK(wagon->LastLoadChangeTime == doctest::Approx(0.0));
		// nothing done since the last step
		CHECK_FALSE(wagon->LoadingDone(-1.0, "coal"));
		CHECK(wagon->LoadAmount == doctest::Approx(8.0));
		wagon->LastLoadChangeTime = 2.0;
		CHECK(wagon->LoadingDone(-1.0, "coal"));
		CHECK(wagon->LoadAmount == doctest::Approx(6.0));
		wagon->CommandIn.Value1 = 100.0;
		wagon->LastLoadChangeTime = 10.0;
		CHECK(wagon->LoadingDone(-1.0, "coal"));
		CHECK(wagon->LoadAmount == doctest::Approx(0.0));
		CHECK(wagon->LoadType.name.empty());
	}
	SUBCASE("loading, step by step, up to what was asked or what the vehicle takes, slower when overloaded") {
		wagon->CommandIn.Value1 = 5.0;
		wagon->LastLoadChangeTime = 2.0;
		CHECK_FALSE(wagon->LoadingDone(1.0, "coal"));
		CHECK(wagon->LoadStatus == 1);
		CHECK(wagon->LoadAmount == doctest::Approx(12.0));
		wagon->LastLoadChangeTime = 3.0;
		CHECK(wagon->LoadingDone(1.0, "coal"));
		CHECK(wagon->LoadAmount == doctest::Approx(15.0));
		wagon->CommandIn.Value1 = 1000.0;
		wagon->LastLoadChangeTime = 1000.0;
		CHECK(wagon->LoadingDone(1.0, "coal"));
		CHECK(wagon->LoadAmount == doctest::Approx(capacity));
		CHECK(capacity == doctest::Approx(2.5 * wagon->MaxLoad));
		// over the vehicle's own capacity, the change goes at half the speed
		wagon->CommandIn.Value1 = 1000.0;
		wagon->LastLoadChangeTime = 4.0;
		CHECK_FALSE(wagon->LoadingDone(-1.0, "coal"));
		CHECK(wagon->LoadAmount == doctest::Approx(capacity - 2.0));
	}
}
