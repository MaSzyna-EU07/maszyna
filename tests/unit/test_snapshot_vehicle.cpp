#include "stdafx.h"
#include "doctest.h"

#include "consist.h"
#include "drive.h"
#include "McZapkie/hamulce.h"
#include "real_vehicles.h"
#include "simulation/simulationsnapshot.h"
#include "simulation/simulationsnapshotmover.h"
#include "vehicle.h"

#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

// the state of a vehicle in a simulation snapshot (simulation/simulationsnapshotmover.cpp): saved from a vehicle which
// has been running, put on a vehicle fresh from its definition and saved again it's the same, and the fresh vehicle
// carries on the way the first one does

namespace {

namespace snapshot = simulation::snapshot;
using test::definition;
using test::vehicle;

test::track const wired{.voltage = test::catenary_3kv};

snapshot::record state_of(TMoverParameters const &Mover)
{
	snapshot::record entry{"vehicle", Mover.Name};
	snapshot::capture_mover(entry, Mover);
	return entry;
}

// the entry as a reader of the file gets it
snapshot::record through_text(snapshot::record const &Entry)
{
	snapshot::document document;
	document.add("snapshot", "1");
	document.records.emplace_back(Entry);
	auto const read{snapshot::parse(snapshot::to_text(document))};
	REQUIRE(read.problems.empty());
	REQUIRE(read.records.size() == 2);
	return read.records[1];
}

// the keys whose values differ between the entries, with the values: "pipe 5 4.9, "
std::string differences(snapshot::record const &Left, snapshot::record const &Right)
{
	std::string text;
	for (auto const &[key, value] : Left.fields)
	{
		if (auto const other{Right.text(key, "(none)")}; other != value)
			text += key + ' ' + value + ' ' + other + ", ";
	}
	if (Left.fields.size() != Right.fields.size())
		text += "the number of keys";
	return text;
}

// an electric locomotive started up, with the brakes off, notched up and run for a while
void run_locomotive(vehicle &Locomotive, int const Notches, double const Seconds)
{
	auto const driven{test::driving(Locomotive)};
	REQUIRE(test::start_up(driven, wired));
	test::release_brakes(Locomotive.mover());
	test::drive(driven, wired, 20.0);
	for (auto notch{0}; notch < Notches; ++notch)
	{
		Locomotive->IncMainCtrl(1);
		test::drive(driven, wired, 2.0);
	}
	test::drive(driven, wired, Seconds);
}

// runs the vehicle on with its driver acknowledging the vigilance device, as the AI does
void drive_on(vehicle &Vehicle, double const Seconds, test::track const &Track = wired)
{
	test::drive(test::driving(Vehicle), Track, Seconds);
}

} // namespace

TEST_CASE("snapshot: the state of every real vehicle, saved and put on a fresh one, saves the same")
{
	for (auto const &path : test::real_vehicles())
	{
		CAPTURE(path);
		test::reseed_random();
		vehicle first{definition::fiz(path), 0.0, "test", 1};
		// something for the snapshot to hold: the battery on, the brakes handled, a few seconds of air moving about
		first->BatterySwitch(true);
		first->CabActivisation();
		first->BrakeLevelSet(first->Handle->GetPos(bh_RP));
		first.run(3.0, wired);
		first->BrakeLevelSet(first->Handle->GetPos(bh_FB));
		first.run(2.0, wired);
		auto const saved{through_text(state_of(first.mover()))};

		test::reseed_random();
		vehicle second{definition::fiz(path), 0.0, "test", 1};
		snapshot::apply_mover(second.mover(), saved);
		auto const restored{state_of(second.mover())};
		CHECK_MESSAGE(restored == saved, differences(saved, restored));
	}
}

TEST_CASE("snapshot: the state of a vehicle with a load and one without")
{
	// the first real vehicle made to carry something
	std::string path;
	for (auto const &candidate : test::real_vehicles())
	{
		if (vehicle probe{definition::fiz(candidate)}; probe->MaxLoad > 0.f && false == probe->LoadAttributes.empty())
		{
			path = candidate;
			break;
		}
	}
	REQUIRE_FALSE(path.empty());
	CAPTURE(path);
	auto const wagon{definition::fiz(path)};
	vehicle loaded{wagon};
	auto const loadname{loaded->LoadAttributes.front().name};
	REQUIRE(loaded->AssignLoad(loadname, 0.5f * loaded->MaxLoad));
	loaded->ComputeMass();
	auto const saved{through_text(state_of(loaded.mover()))};
	CHECK(saved.text("loadtype") == loadname);

	vehicle empty{wagon};
	auto const emptymass{empty->TotalMass};
	snapshot::apply_mover(empty.mover(), saved);
	CHECK(empty->LoadAmount == loaded->LoadAmount);
	CHECK(empty->LoadType.name == loadname);
	CHECK(empty->TotalMass == doctest::Approx(loaded->TotalMass));
	CHECK(empty->TotalMass > emptymass);

	// and the other way: a vehicle loaded by the scenery, empty in the snapshot
	vehicle unloaded{wagon};
	auto const emptied{through_text(state_of(unloaded.mover()))};
	snapshot::apply_mover(empty.mover(), emptied);
	CHECK(empty->LoadAmount == 0.f);
	CHECK(empty->LoadType.name.empty());
	CHECK(empty->TotalMass == doctest::Approx(emptymass));
}

TEST_CASE("snapshot: an entry without the state of the machinery leaves the vehicle as it is")
{
	vehicle locomotive{definition::fiz("4e_v1/4e"), 0.0, "test", 1};
	run_locomotive(locomotive, 5, 5.0);
	auto const before{state_of(locomotive.mover())};
	// what an older file, or one written by hand, may hold
	snapshot::record entry{"vehicle", "test"};
	entry.set("unknown", 12).set("mainctrl", locomotive->MainCtrlPos);
	snapshot::apply_mover(locomotive.mover(), entry);
	auto const after{state_of(locomotive.mover())};
	CHECK_MESSAGE(after == before, differences(before, after));
}

TEST_CASE("snapshot: a locomotive restored on the move runs on like the one it was saved from")
{
	test::reseed_random();
	vehicle first{definition::fiz("4e_v1/4e"), 0.0, "test", 1};
	run_locomotive(first, 12, 20.0);
	REQUIRE(first->Mains);
	REQUIRE(first.speed() > 20.0);
	auto const saved{through_text(state_of(first.mover()))};

	test::reseed_random();
	vehicle second{definition::fiz("4e_v1/4e"), 0.0, "test", 1};
	snapshot::apply_mover(second.mover(), saved);
	CHECK(second.speed() == first.speed());
	CHECK(second->Mains);
	CHECK(second->MainCtrlPos == first->MainCtrlPos);

	// both carry on under power for a minute
	auto const firststart{first->DistCounter};
	auto const secondstart{second->DistCounter};
	drive_on(first, 60.0);
	drive_on(second, 60.0);
	CAPTURE(first.speed());
	CAPTURE(second.speed());
	CHECK(second->Mains);
	CHECK(second->ConverterFlag == first->ConverterFlag);
	CHECK(second.speed() == doctest::Approx(first.speed()).epsilon(0.02));
	CHECK(second->DistCounter - secondstart == doctest::Approx(first->DistCounter - firststart).epsilon(0.02));
	CHECK(second->PipePress == doctest::Approx(first->PipePress).epsilon(0.02));
	CHECK(second->ScndPipePress == doctest::Approx(first->ScndPipePress).epsilon(0.05));

	// then both brake to a stand
	for (auto *locomotive : {&first, &second})
	{
		while ((*locomotive)->DecMainCtrl(1))
		{
			// power off
		}
		(*locomotive)->BrakeLevelSet((*locomotive)->Handle->GetPos(bh_FB));
	}
	auto const firstbraking{first->DistCounter};
	auto const secondbraking{second->DistCounter};
	drive_on(first, 120.0);
	drive_on(second, 120.0);
	CHECK(first.speed() == doctest::Approx(0.0).epsilon(0.01));
	CHECK(second.speed() == doctest::Approx(0.0).epsilon(0.01));
	CHECK(second->DistCounter - secondbraking == doctest::Approx(first->DistCounter - firstbraking).epsilon(0.03));
	CHECK(second->BrakePress == doctest::Approx(first->BrakePress).epsilon(0.03));
}

TEST_CASE("snapshot: a locomotive restored in the middle of braking stops where the one it was saved from stops")
{
	test::reseed_random();
	vehicle first{definition::fiz("4e_v1/4e"), 0.0, "test", 1};
	run_locomotive(first, 14, 40.0);
	REQUIRE(first.speed() > 40.0);
	while (first->DecMainCtrl(1))
	{
		// power off
	}
	first->BrakeLevelSet(first->Handle->GetPos(bh_FB));
	// the pressure in the cylinders still rising
	drive_on(first, 2.0);
	REQUIRE(first->BrakePress > 0.1);
	REQUIRE(first.speed() > 20.0);
	auto const saved{through_text(state_of(first.mover()))};

	test::reseed_random();
	vehicle second{definition::fiz("4e_v1/4e"), 0.0, "test", 1};
	snapshot::apply_mover(second.mover(), saved);
	CHECK(second->BrakePress == first->BrakePress);
	CHECK(second->Hamulec->GetBCP() == first->Hamulec->GetBCP());
	CHECK(second->Hamulec->GetCRP() == first->Hamulec->GetCRP());
	CHECK(second->Hamulec->GetBRP() == first->Hamulec->GetBRP());

	auto const firststart{first->DistCounter};
	auto const secondstart{second->DistCounter};
	drive_on(first, 120.0);
	drive_on(second, 120.0);
	CHECK(first.speed() == doctest::Approx(0.0).epsilon(0.01));
	CHECK(second.speed() == doctest::Approx(0.0).epsilon(0.01));
	CAPTURE(first->DistCounter - firststart);
	CAPTURE(second->DistCounter - secondstart);
	CHECK(second->DistCounter - secondstart == doctest::Approx(first->DistCounter - firststart).epsilon(0.03));
	CHECK(second->BrakePress == doctest::Approx(first->BrakePress).epsilon(0.03));
	CHECK(second->PipePress == doctest::Approx(first->PipePress).epsilon(0.03));
}

namespace {

// a train of the given vehicles, the first one with the driver
void make_train(test::consist &Train, std::vector<char const *> const &Vehicles)
{
	test::reseed_random();
	for (auto const *path : Vehicles)
		Train.add(definition::fiz(path), Train.size() == 0 ? 1 : 0);
}

// puts the state of each vehicle of a train on the same vehicle of another, and checks that it took
void restore_train(test::consist &To, test::consist &From)
{
	REQUIRE(To.size() == From.size());
	for (std::size_t index{0}; index < From.size(); ++index)
		snapshot::apply_mover(To[index].mover(), through_text(state_of(From[index].mover())));
	for (std::size_t index{0}; index < From.size(); ++index)
	{
		CAPTURE(index);
		auto const saved{state_of(From[index].mover())};
		auto const restored{state_of(To[index].mover())};
		CHECK_MESSAGE(restored == saved, differences(saved, restored));
	}
}

} // namespace

TEST_CASE("snapshot: a multiple unit restored on the move runs on like the one it was saved from")
{
	std::vector<char const *> const cars{"ed72_v1/5bs-ra", "ed72_v1/6bs-sa", "ed72_v1/6bs-sb", "ed72_v1/5bs-rb"};
	test::consist first;
	make_train(first, cars);
	auto const driven{test::driving(first)};
	REQUIRE(test::start_up(driven, wired));
	test::release_brakes(first[0].mover());
	test::drive(driven, wired, 20.0);
	for (auto notch{0}; notch < 2; ++notch)
	{
		first[0]->IncMainCtrl(1);
		test::drive(driven, wired, 2.0);
	}
	test::drive(driven, wired, 40.0);
	REQUIRE(first[0].speed() > 20.0);

	test::consist second;
	make_train(second, cars);
	restore_train(second, first);

	test::drive(driven, wired, 60.0);
	test::drive(test::driving(second), wired, 60.0);
	CAPTURE(first[0].speed());
	CAPTURE(second[0].speed());
	for (std::size_t index{0}; index < first.size(); ++index)
	{
		CAPTURE(index);
		CHECK(second[index]->Mains == first[index]->Mains);
		CHECK(second[index]->ConverterFlag == first[index]->ConverterFlag);
		CHECK(second[index].speed() == doctest::Approx(first[index].speed()).epsilon(0.02));
		CHECK(second[index]->PipePress == doctest::Approx(first[index]->PipePress).epsilon(0.02));
		CHECK(second[index]->ScndPipePress == doctest::Approx(first[index]->ScndPipePress).epsilon(0.03));
	}
}

TEST_CASE("snapshot: a locomotive with wagons restored while it brakes stops like the train it was saved from")
{
	std::vector<char const *> const vehicles{"4e_v1/4e", "3wl_v2/3wl", "11xa_v2/93a", "bdhpumn_v3/bdhpumn", "z1_v2/152a"};
	test::consist first;
	make_train(first, vehicles);
	auto const driven{test::driving(first)};
	REQUIRE(test::start_up(driven, wired));
	test::release_brakes(first[0].mover());
	test::drive(driven, wired, 30.0);
	for (auto notch{0}; notch < 14; ++notch)
	{
		first[0]->IncMainCtrl(1);
		test::drive(driven, wired, 2.0);
	}
	test::drive(driven, wired, 40.0);
	REQUIRE(first[0].speed() > 30.0);
	while (first[0]->DecMainCtrl(1))
	{
		// power off
	}
	first[0]->BrakeLevelSet(first[0]->Handle->GetPos(bh_FB));
	// the brake pipe still emptying along the train, the cylinders filling
	test::drive(driven, wired, 3.0);
	REQUIRE(first[4]->BrakePress > 0.05);

	test::consist second;
	make_train(second, vehicles);
	restore_train(second, first);

	auto const firststart{first[0]->DistCounter};
	auto const secondstart{second[0]->DistCounter};
	test::drive(driven, wired, 120.0);
	test::drive(test::driving(second), wired, 120.0);
	CHECK(first[0].speed() == doctest::Approx(0.0).epsilon(0.01));
	CHECK(second[0].speed() == doctest::Approx(0.0).epsilon(0.01));
	CAPTURE(first[0]->DistCounter - firststart);
	CAPTURE(second[0]->DistCounter - secondstart);
	CHECK(second[0]->DistCounter - secondstart == doctest::Approx(first[0]->DistCounter - firststart).epsilon(0.02));
	for (std::size_t index{0}; index < first.size(); ++index)
	{
		CAPTURE(index);
		CHECK(second[index]->BrakePress == doctest::Approx(first[index]->BrakePress).epsilon(0.02).scale(4.0));
		CHECK(second[index]->PipePress == doctest::Approx(first[index]->PipePress).epsilon(0.02).scale(5.0));
		CHECK(second[index]->Hamulec->GetCRP() == doctest::Approx(first[index]->Hamulec->GetCRP()).epsilon(0.02).scale(5.0));
	}
}

TEST_CASE("snapshot: a diesel locomotive restored with its engine running keeps it running")
{
	test::reseed_random();
	vehicle first{definition::fiz("sm42_v1/6d1"), 0.0, "test", 1};
	auto const driven{test::driving(first)};
	REQUIRE(test::start_up(driven, test::track{}));
	test::release_brakes(first.mover());
	drive_on(first, 30.0, test::track{});
	REQUIRE(first->Mains);
	REQUIRE(first->enrot > 0.0);
	auto const saved{through_text(state_of(first.mover()))};

	test::reseed_random();
	vehicle second{definition::fiz("sm42_v1/6d1"), 0.0, "test", 1};
	snapshot::apply_mover(second.mover(), saved);
	drive_on(first, 30.0, test::track{});
	drive_on(second, 30.0, test::track{});
	CHECK(second->Mains);
	CHECK(second->enrot == doctest::Approx(first->enrot).epsilon(0.02));
	CHECK(second->dizel_heat.Ts == doctest::Approx(first->dizel_heat.Ts).epsilon(0.02));
	CHECK(second->ScndPipePress == doctest::Approx(first->ScndPipePress).epsilon(0.05));
}

TEST_CASE("snapshot: every real vehicle which starts up, restored while it runs, runs on like the one it was saved from")
{
	for (auto const &path : test::real_vehicles())
	{
		CAPTURE(path);
		test::reseed_random();
		vehicle first{definition::fiz(path), 0.0, "test", 1};
		auto const track{first->EnginePowerSource.SourceType == TPowerSource::CurrentCollector ? wired : test::track{}};
		if (false == test::is_powered(first.mover()) || false == test::start_up(test::driving(first), track))
			continue;
		test::release_brakes(first.mover());
		for (auto notch{0}; notch < 3; ++notch)
		{
			first->IncMainCtrl(1);
			drive_on(first, 3.0, track);
		}
		drive_on(first, 10.0, track);
		auto const saved{through_text(state_of(first.mover()))};

		test::reseed_random();
		vehicle second{definition::fiz(path), 0.0, "test", 1};
		snapshot::apply_mover(second.mover(), saved);
		drive_on(first, 20.0, track);
		drive_on(second, 20.0, track);
		CHECK(second->Mains == first->Mains);
		CHECK(second->ConverterFlag == first->ConverterFlag);
		CHECK(second->CompressorFlag == first->CompressorFlag);
		CHECK(second.speed() == doctest::Approx(first.speed()).epsilon(0.02).scale(10.0));
		CHECK(second->enrot == doctest::Approx(first->enrot).epsilon(0.02).scale(1.0));
		CHECK(second->PipePress == doctest::Approx(first->PipePress).epsilon(0.01).scale(5.0));
		CHECK(second->BrakePress == doctest::Approx(first->BrakePress).epsilon(0.01).scale(5.0));
		CHECK(second->ScndPipePress == doctest::Approx(first->ScndPipePress).epsilon(0.01).scale(8.0));
	}
}

TEST_CASE("snapshot: what a brake valve keeps beside its air comes back on a fresh vehicle")
{
	for (auto const &path : test::real_vehicles())
	{
		CAPTURE(path);
		test::reseed_random();
		vehicle first{definition::fiz(path), 0.0, "test", 1};
		REQUIRE(first->Hamulec != nullptr);
		// every latch the other way than a fresh valve has it, every level another one
		auto latches{first->Hamulec->latches()};
		REQUIRE(latches.size() >= 2);
		for (auto &value : latches)
			value = (value == 0.0 ? 1.0 : 0.0);
		first->Hamulec->latches(latches);
		REQUIRE(first->Hamulec->latches() == latches);
		// and a flow its auxiliary reservoir has yet to take in
		auto const reservoirs{first->Hamulec->reservoirs()};
		REQUIRE(reservoirs.size() >= 3);
		REQUIRE(reservoirs[1] != nullptr);
		reservoirs[1]->Flow(0.125);
		auto const saved{through_text(state_of(first.mover()))};

		test::reseed_random();
		vehicle second{definition::fiz(path), 0.0, "test", 1};
		CHECK(second->Hamulec->latches() != latches);
		snapshot::apply_mover(second.mover(), saved);
		CHECK(second->Hamulec->latches() == latches);
		CHECK(second->Hamulec->reservoirs()[1]->GetPendingFlow() == 0.125);
	}
}

TEST_CASE("snapshot: positions of the controllers past the ranges of the vehicle are brought within them")
{
	snapshot::take_refusals();
	vehicle locomotive{definition::fiz("4e_v1/4e"), 0.0, "test", 1};
	REQUIRE(locomotive->MainCtrlPosNo > 0);
	auto entry{state_of(locomotive.mover())};
	// what an edited file, or one saved with a definition of more positions than the vehicle has now, may hold
	entry.set("mainctrl", 999).set("mainctrlactual", -4).set("scndctrl", locomotive->ScndCtrlPosNo + 5).set("scndctrlactual", 1000);
	entry.set("brakectrl", 99).set("manualbrake", -3).set("lightspos", 500).set("compressorlistpos", 40).set("wiperswitchpos", 16);
	entry.set("caboccupied", 7).set("diractive", -2).set("dimmer", 1000);
	snapshot::apply_mover(locomotive.mover(), through_text(entry));
	CHECK(locomotive->MainCtrlPos == locomotive->MainCtrlPosNo);
	CHECK(locomotive->MainCtrlActualPos == 0);
	CHECK(locomotive->ScndCtrlPos == locomotive->ScndCtrlPosNo);
	CHECK(locomotive->ScndCtrlActualPos == locomotive->ScndCtrlPosNo);
	CHECK(locomotive->BrakeCtrlPos == std::max(locomotive->BrakeCtrlPosNo, 0));
	CHECK(locomotive->ManualBrakePos == 0);
	CHECK(locomotive->LightsPos <= 18);
	CHECK(locomotive->LightsPos == std::min(locomotive->LightsPosNo, 18));
	CHECK(locomotive->CompressorListPos <= 8);
	CHECK(locomotive->wiperSwitchPos == 15);
	CHECK(locomotive->CabOccupied == 1);
	CHECK(locomotive->DirActive == -1);
	CHECK(locomotive->modernDimmerPosition == static_cast<int>(locomotive->dimPositions.size()) - 1);
	auto const refusals{snapshot::take_refusals()};
	CAPTURE(refusals.size());
	CHECK(refusals.size() >= 10);

	// a value which isn't a whole number at all isn't taken, and the position stays as it was
	auto const before{locomotive->MainCtrlPos};
	snapshot::record broken{"vehicle", "test"};
	broken.fields = {{"mainctrl", "1e18"}, {"mainctrlactual", "nan"}, {"scndctrl", "2.5"}};
	snapshot::apply_mover(locomotive.mover(), broken);
	CHECK(locomotive->MainCtrlPos == before);
	CHECK(snapshot::take_refusals().size() == 3);
}

TEST_CASE("snapshot: the pullee mode and the timer of the release of the holding brake come back")
{
	// nobody in it: a vehicle with a cab taken isn't prepared for being hauled
	vehicle first{definition::fiz("4e_v1/4e")};
	REQUIRE_FALSE(first->PulleeMode);
	REQUIRE(first->PulleeModeSwitch(true));
	first->StopBrakeReleaseTimer = 1.25;
	auto const saved{through_text(state_of(first.mover()))};
	CHECK(saved.text("pulleemode") == "1");
	CHECK(saved.text("stopbrakereleasetimer") == "1.25");

	vehicle second{definition::fiz("4e_v1/4e")};
	snapshot::apply_mover(second.mover(), saved);
	CHECK(second->PulleeMode);
	CHECK(second->StopBrakeReleaseTimer == 1.25);
	auto const restored{state_of(second.mover())};
	CHECK_MESSAGE(restored == saved, differences(saved, restored));
}

TEST_CASE("snapshot: a load which can't be one is refused")
{
	snapshot::take_refusals();
	std::string path;
	for (auto const &candidate : test::real_vehicles())
	{
		if (vehicle probe{definition::fiz(candidate)}; probe->MaxLoad > 0.f && false == probe->LoadAttributes.empty())
		{
			path = candidate;
			break;
		}
	}
	REQUIRE_FALSE(path.empty());
	vehicle wagon{definition::fiz(path)};
	auto const loadname{wagon->LoadAttributes.front().name};
	for (auto const *amount : {"nan", "-1", "inf", "1e30"})
	{
		CAPTURE(amount);
		snapshot::record entry{"vehicle", "test"};
		entry.fields = {{"load", amount}, {"loadtype", loadname}};
		snapshot::apply_mover(wagon.mover(), entry);
		CHECK(wagon->LoadAmount == 0.f);
		CHECK(wagon->LoadType.name.empty());
	}
	// a load is named by its model, which is looked up in the folder of the vehicle
	snapshot::record entry{"vehicle", "test"};
	entry.fields = {{"load", "1"}, {"loadtype", "../../../models/elsewhere"}};
	snapshot::apply_mover(wagon.mover(), entry);
	CHECK(wagon->LoadType.name.empty());
	CHECK(snapshot::take_refusals().size() == 5);
}

namespace {

// the pressures a driver's handle of the kind gives to a snapshot: each of its variables once, as a second copy of
// one would be saved twice and put back twice
template <typename Handle>
void pressures_listed_once(char const *Kind)
{
	CAPTURE(Kind);
	Handle handle;
	auto const pressures{handle.pressures()};
	std::set<double const *> const distinct(pressures.begin(), pressures.end());
	CHECK(distinct.size() == pressures.size());
}

} // namespace

TEST_CASE("snapshot: a driver's handle lists each of its pressures once")
{
	pressures_listed_once<TFV4a>("FV4a");
	pressures_listed_once<TFV4aM>("FV4aM");
	pressures_listed_once<TMHZ_EN57>("MHZ_EN57");
	pressures_listed_once<TMHZ_K5P>("MHZ_K5P");
	pressures_listed_once<TMHZ_6P>("MHZ_6P");
	pressures_listed_once<TM394>("M394");
	pressures_listed_once<TH14K1>("H14K1");
	pressures_listed_once<TSt113>("St113");
	pressures_listed_once<Ttest>("test");
	pressures_listed_once<TFD1>("FD1");
	pressures_listed_once<TH1405>("H1405");
	pressures_listed_once<TFVel6>("FVel6");
	pressures_listed_once<TFVE408>("FVE408");
}

TEST_CASE("snapshot: the pressures of the Knorr St113 come back on a fresh vehicle")
{
	test::reseed_random();
	vehicle first{definition::fiz("en57_v1/6bai"), 0.0, "test", 1};
	REQUIRE(first->BrakeHandle == TBrakeHandle::St113);
	REQUIRE(first->Handle != nullptr);
	// the control chamber, the reductor and the EP intensity, none of them what a fresh handle has
	auto const pressures{first->Handle->pressures()};
	std::vector<double> const values{4.625, 0.125, -1.0};
	REQUIRE(pressures.size() == values.size());
	for (std::size_t index{0}; index < values.size(); ++index)
		*pressures[index] = values[index];
	CHECK(first->Handle->GetCP() == values[0]);
	CHECK(first->Handle->GetRP() == 5.0 + values[1]);
	CHECK(first->Handle->GetEP() == values[2]);
	auto const saved{through_text(state_of(first.mover()))};
	CHECK(snapshot::text_numbers(saved.text("handle")) == values);

	test::reseed_random();
	vehicle second{definition::fiz("en57_v1/6bai"), 0.0, "test", 1};
	REQUIRE(second->Handle != nullptr);
	CHECK(second->Handle->GetCP() != values[0]);
	snapshot::apply_mover(second.mover(), saved);
	CHECK(second->Handle->GetCP() == values[0]);
	CHECK(second->Handle->GetRP() == 5.0 + values[1]);
	CHECK(second->Handle->GetEP() == values[2]);
	auto const restored{state_of(second.mover())};
	CHECK_MESSAGE(restored == saved, differences(saved, restored));
}
