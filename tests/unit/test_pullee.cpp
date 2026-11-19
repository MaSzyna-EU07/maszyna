#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "consist.h"
#include "drive.h"
#include "real_vehicles.h"

using test::definition;
using test::vehicle;

// the pullee mode ("jazda ciągniona", MaSzyna-EU07/maszyna#64): a vehicle prepared for being hauled dead in a
// train, TMoverParameters::PulleeModeSwitch(). the trains here are an EU07 and the vehicle behind it, coupled with
// the hook and the brake hose alone, as with coupling 3 of a scenery. every test has both sides: what the vehicle
// does as it is (unchanged by the mode being there), and what it does prepared for hauling.
// pressures in bar, the brake pipe runs at 5.0

namespace {

constexpr int hook_and_brake_hose{coupling::coupler | coupling::brakehose};

// an EU07 with nobody in it, its brake pipe held by the test the way a driver's valve would hold it, and the
// vehicle it hauls. Velocity as of a trainset of a scenery [km/h]: not 0, the vehicles are ready to depart,
// 0, they stand braked (brake pipe at 3.5, hand and spring brakes on, batteries off)
struct hauled_train {
	test::consist train;
	vehicle &locomotive;
	vehicle &hauled;

	hauled_train(definition const &Hauled, double const Velocity, bool const Pullee)
	    : locomotive{train.add(definition::fiz("4e_v1/4e"), 0, test::consist::full_coupling, Velocity)}, hauled{train.add(Hauled, 0, hook_and_brake_hose, Velocity)} {
		if (Pullee) {
			REQUIRE(hauled->PulleeModeSwitch(true));
		}
		locomotive.hold_brake_pipe(5.0);
	}

	void run(double const Seconds) { train.run(Seconds); }
};

// no air in the main reservoir and the feed pipe, as after standing dead for some hours
void drain_feed_air(TMoverParameters &Vehicle) {
	Vehicle.Pipe2->CreatePress(0.0);
	Vehicle.Pipe2->Act();
	Vehicle.CompressedVolume = 0.0;
}

// whether the real vehicle can go in a train at all: the road vehicles can't, nor can the draisine, which has no coupler
bool can_be_hauled(std::string const &Path) {
	vehicle probe{definition::fiz(Path)};
	return (probe->CategoryFlag & 1) != 0 && probe->Couplers[end::front].CouplerType != TCouplerType::NoCoupler;
}

} // namespace

TEST_CASE("pullee: a dead Vectron vents the brake pipe of the train hauling it, prepared for hauling it doesn't") {
	// the Vectron applies the emergency brake while none of its cabs is active (InactiveCabFlag bit 1), which a
	// vehicle hauled with the hook and the brake hose never has
	auto const vectron{definition::fiz("vectron_v1/wariant_7w")};
	SUBCASE("as it is") {
		hauled_train train{vectron, 60.0, false};
		REQUIRE((train.hauled->InactiveCabFlag & activation::emergencybrake) != 0);
		REQUIRE_FALSE(train.hauled->PulleeMode);
		train.run(120.0);
		CHECK(train.hauled->EmergencyValveFlow > 0.0);
		CHECK(train.locomotive->PipePress < 4.95); // the pipe sags although the locomotive keeps feeding it
		CHECK(train.hauled->BrakePress > 0.4);
		CHECK(train.hauled.speed() < 30.0);
	}
	SUBCASE("prepared for hauling") {
		hauled_train train{vectron, 60.0, true};
		train.run(120.0);
		CHECK(train.hauled->EmergencyValveFlow == 0.0);
		CHECK(train.locomotive->PipePress > 4.99);
		CHECK(train.hauled->BrakePress < 0.1);
		CHECK(train.hauled->Fb < 1.0);
		CHECK(train.hauled.speed() > 40.0); // running resistance alone
	}
}

TEST_CASE("pullee: an EU07 with a driver can't release the brakes of a dead Vectron, and hauls one prepared for hauling") {
	for (auto const pullee : {false, true}) {
		CAPTURE(pullee);
		test::reseed_random();
		test::consist train;
		// a trainset ready to depart, as with velocity 0.1 in a scenery
		auto &locomotive{train.add(definition::fiz("4e_v1/4e"), 1, test::consist::full_coupling, 0.1)};
		auto &hauled{train.add(definition::fiz("vectron_v1/wariant_7w"), 0, hook_and_brake_hose, 0.1)};
		if (pullee) {
			REQUIRE(hauled->PulleeModeSwitch(true));
		}
		test::track const track{0.15, 0.0, test::catenary_3kv};
		auto driven{test::driving(train)};
		REQUIRE(test::start_up(driven, track, 600.0));
		test::release_brakes(driven.cab);
		// the driver of a train switches on the batteries of the vehicles not controlled from the cab, TController::CheckVehicles()
		hauled->BatterySwitch(true);
		test::drive(driven, track, 60.0);
		if (false == pullee) {
			// the Vectron has emptied the brake pipe, and both locomotives stand on an emergency application
			CHECK(hauled->Battery);
			CHECK(locomotive->PipePress < 1.0);
			CHECK(locomotive->BrakePress > 3.0);
			CHECK(hauled->BrakePress > 3.0);
			continue;
		}
		CHECK_FALSE(hauled->Battery);
		CHECK(locomotive->PipePress > 4.9);
		CHECK(locomotive->BrakePress < 0.1);
		CHECK(hauled->BrakePress < 0.1);
		// the driver notches up watching the ammeter
		for (int i = 0; i < 3 && driven.cab.DirActive <= 0; ++i) {
			driven.cab.DirectionForward();
		}
		for (int second = 0; second < 60; second += 5) {
			if (std::abs(driven.controlling.Im) < 350.0 && false == driven.controlling.IsMainCtrlMaxPowerPos()) {
				driven.controlling.IncMainCtrl(1);
			}
			test::drive(driven, track, 5.0);
		}
		CHECK(hauled.speed() > 20.0);
	}
}

TEST_CASE("pullee: the spring brake of a dead locomotive without air stops the train, prepared for hauling it gives no force") {
	// the spring brake is held off by the air of the feed pipe, which on a dead locomotive is what its main
	// reservoir has left. the EU47 and the EP09 have the spring brake alone, the Vectron the emergency brake too
	for (auto const *path : {"e186_v2/eu47", "ep09_v2/104e", "vectron_v1/wariant_7w"}) {
		for (auto const pullee : {false, true}) {
			CAPTURE(path);
			CAPTURE(pullee);
			test::reseed_random();
			hauled_train train{definition::fiz(path), 60.0, pullee};
			auto const &hauled{train.hauled.mover()};
			REQUIRE(hauled.SpringBrake.MaxBrakeForce > 0.0);
			drain_feed_air(train.hauled.mover());
			train.run(60.0);
			// no air came from anywhere: the cylinder of the spring brake is empty either way
			REQUIRE(hauled.SpringBrake.SBP < 0.1);
			if (pullee) {
				CHECK(hauled.Fb < 1.0);
				CHECK(train.hauled.speed() > 45.0);
			}
			else {
				CHECK(hauled.Fb > 30000.0);
				CHECK(std::abs(train.hauled.speed()) < 1.0);
			}
		}
	}
}

TEST_CASE("pullee: a dead EU47 starts to drag once its main reservoir has leaked down, prepared for hauling it never does") {
	// from a standing start the reservoir has 4.4 bar, and the spring brake comes on below 3.5 (MinFP)
	for (auto const pullee : {false, true}) {
		CAPTURE(pullee);
		test::reseed_random();
		hauled_train train{definition::fiz("e186_v2/eu47"), 0.0, pullee};
		auto &hauled{train.hauled.mover()};
		// what the driver of the train does on its inspection, TController::AutoRewident()
		hauled.DecManualBrakeLevel(ManualBrakePosNo);
		hauled.SpringBrake.Activate = false;
		train.run(180.0);
		// charged and released: nothing holds it yet
		CHECK(hauled.BrakePress < 0.1);
		CHECK(hauled.Fb < 1.0);
		train.run(1020.0); // 20 minutes since the start
		CHECK(hauled.ScndPipePress < hauled.SpringBrake.MinForcePressure);
		if (pullee) {
			CHECK(hauled.Fb < 1.0);
		}
		else {
			CHECK(hauled.Fb > 5000.0);
		}
	}
}

TEST_CASE("pullee: a vehicle prepared for hauling brakes and releases with the train, and its spring brake stays off after") {
	// the distributor stays on the brake pipe. with air in the feed pipe and without: the cylinders fill from the
	// auxiliary reservoir, and the spring brake, with no air to hold it off, still gives nothing after the release
	for (auto const *path : {"vectron_v1/wariant_7w", "e6act_v2/e6acta", "ep09_v2/104e", "e186_v2/eu47"}) {
		for (auto const feedair : {true, false}) {
			CAPTURE(path);
			CAPTURE(feedair);
			test::reseed_random();
			hauled_train train{definition::fiz(path), 60.0, true};
			auto const &hauled{train.hauled.mover()};
			if (false == feedair) {
				drain_feed_air(train.hauled.mover());
			}
			train.run(30.0);
			CHECK(hauled.BrakePress < 0.1);
			// a full service application
			train.locomotive.hold_brake_pipe(3.5);
			train.run(20.0);
			CHECK(hauled.BrakePress > 2.0);
			CHECK(hauled.Fb > 30000.0);
			train.locomotive.hold_brake_pipe(5.0);
			train.run(90.0);
			CHECK(hauled.BrakePress < 0.1);
			CHECK(hauled.Fb < 1.0);
			CHECK(hauled.EmergencyValveFlow == 0.0);
		}
	}
}

TEST_CASE("pullee: the spring brake of an Impuls prepared for hauling doesn't call the emergency brake") {
	// SpringBrakeDriveEmergencyVel=2: above 2 km/h an applied spring brake vents the brake pipe. the emergency
	// brake of the inactive cab (InactiveCabFlag) is taken out of the definition, so that it's the spring brake's
	auto const impuls{definition::fiz("impuls_v1/31we-a_pr").set("Cntrl", "InactiveCabFlag", 0)};
	for (auto const pullee : {false, true}) {
		CAPTURE(pullee);
		test::reseed_random();
		hauled_train train{impuls, 60.0, pullee};
		auto const &hauled{train.hauled.mover()};
		REQUIRE(hauled.InactiveCabFlag == 0);
		REQUIRE(hauled.SpringBrakeDriveEmergencyVel == doctest::Approx(2.0));
		drain_feed_air(train.hauled.mover());
		train.run(10.0);
		REQUIRE(hauled.Vel > 2.0);
		REQUIRE(hauled.SpringBrake.IsActive);
		if (pullee) {
			CHECK(hauled.EmergencyValveFlow == 0.0);
			CHECK(hauled.BrakePress < 0.1);
		}
		else {
			CHECK(hauled.EmergencyValveFlow > 0.0);
		}
	}
}

TEST_CASE("pullee: the brake valves in the cab of a vehicle prepared for hauling move no air") {
	// with somebody in the cab: nobody is put there by a scenery, but the valves are cut out whoever sits down.
	// the Elf has the MHZ_K5P driver's valve, the EU47 the MHZ_6P, the Vectron the MHZ_K8P, the EP09 and the EU07 the FV4a
	for (auto const *path : {"elf_v1/21we-a", "e186_v2/eu47", "vectron_v1/wariant_7w", "ep09_v2/104e", "4e_v1/4e"}) {
		for (auto const pullee : {false, true}) {
			CAPTURE(path);
			CAPTURE(pullee);
			test::reseed_random();
			vehicle driven{definition::fiz(path), 0.1, "test", 1};
			// the emergency brake of the inactive cab is another matter, see above
			driven->InactiveCabFlag = 0;
			if (pullee) {
				REQUIRE(driven->PulleeModeSwitch(true));
			}
			// the local brake a driver keeps a standing vehicle on is let go
			driven->DecLocalBrakeLevel(LocalBrakePosNo);
			driven.run(20.0);
			REQUIRE(driven->BrakePress < 0.1);
			auto const charged{driven->PipePress};
			// full service braking with the driver's valve
			driven->BrakeLevelSet(driven->Handle->GetPos(bh_FB));
			driven.run(10.0);
			if (pullee) {
				CHECK(driven->dpMainValve == 0.0);
				CHECK(driven->PipePress > charged - 0.1);
				CHECK(driven->BrakePress < 0.1);
			}
			else {
				CHECK(driven->PipePress < charged - 0.5);
				CHECK(driven->BrakePress > 1.0);
			}
			// and the emergency position
			driven->BrakeLevelSet(driven->Handle->GetPos(bh_EB));
			driven.run(10.0);
			if (pullee) {
				CHECK(driven->dpMainValve == 0.0);
				CHECK(driven->PipePress > charged - 0.1);
			}
			else {
				CHECK(driven->PipePress < 1.0);
			}
		}
	}
}

TEST_CASE("pullee: the local brake of a vehicle prepared for hauling stays released") {
	for (auto const *path : {"vectron_v1/wariant_7w", "4e_v1/4e", "ep09_v2/104e"}) {
		for (auto const pullee : {false, true}) {
			CAPTURE(path);
			CAPTURE(pullee);
			test::reseed_random();
			vehicle driven{definition::fiz(path), 0.1, "test", 1};
			driven->InactiveCabFlag = 0;
			driven.hold_brake_pipe(5.0);
			if (pullee) {
				REQUIRE(driven->PulleeModeSwitch(true));
			}
			// applied with the lever, as on a vehicle left on standby, and asked for by the brake computer of a unit
			// (what the MED of TDynamicObject writes; the lever of a Vectron does nothing without it)
			driven->IncLocalBrakeLevel(LocalBrakePosNo);
			driven->LocalBrakePosAEIM = 1.0;
			driven.run(20.0);
			if (pullee) {
				CHECK(driven->BrakePress < 0.1);
			}
			else {
				CHECK(driven->BrakePress > 1.0);
			}
		}
	}
}

TEST_CASE("pullee: entering the mode lets go of the hand brake and the spring brake switch, and switches the battery off") {
	SUBCASE("a locomotive standing braked, its spring brake switched on") {
		vehicle vectron{definition::fiz("vectron_v1/wariant_7w")};
		REQUIRE(vectron->SpringBrake.Activate);
		REQUIRE_FALSE(vectron->PulleeMode);
		CHECK(vectron->PulleeModeSwitch(true));
		CHECK(vectron->PulleeMode);
		CHECK_FALSE(vectron->SpringBrake.Activate);
		// already in it
		CHECK_FALSE(vectron->PulleeModeSwitch(true));
	}
	SUBCASE("a unit standing with its hand brake on") {
		vehicle en57{definition::fiz("en57_v1/5bgb")};
		REQUIRE(en57->ManualBrakePos > 0);
		CHECK(en57->PulleeModeSwitch(true));
		CHECK(en57->ManualBrakePos == 0);
	}
	SUBCASE("a locomotive ready to depart, its battery on") {
		vehicle vectron{definition::fiz("vectron_v1/wariant_7w"), 0.1};
		REQUIRE(vectron->Battery);
		CHECK(vectron->PulleeModeSwitch(true));
		CHECK_FALSE(vectron->Battery);
	}
}

TEST_CASE("pullee: the battery and the cabs of a vehicle prepared for hauling stay off, until it leaves the mode") {
	vehicle vectron{definition::fiz("vectron_v1/wariant_7w"), 0.0, "test", 1};
	REQUIRE(vectron->PulleeModeSwitch(true));
	CHECK_FALSE(vectron->BatterySwitch(true));
	CHECK_FALSE(vectron->Battery);
	CHECK_FALSE(vectron->CabActivisation());
	CHECK_FALSE(vectron->CabActivisation(true));
	CHECK(vectron->CabActive == 0);
	vectron.run(5.0);
	// the vigilance device and the cab signalling of a cab never activated stay out of it
	CHECK_FALSE(vectron->SecuritySystem.is_braking());
	CHECK(vectron->EmergencyValveFlow == 0.0);

	REQUIRE(vectron->PulleeModeSwitch(false));
	CHECK_FALSE(vectron->PulleeMode);
	CHECK(vectron->BatterySwitch(true));
	CHECK(vectron->Battery);
	CHECK(vectron->CabActivisation());
	CHECK(vectron->CabActive == 1);
	// a vehicle driven from isn't one to haul
	CHECK_FALSE(vectron->PulleeModeSwitch(true));
	CHECK_FALSE(vectron->PulleeMode);
}

TEST_CASE("pullee: a cab activated on a dead battery calls the emergency brake, which the mode keeps from happening") {
	// the vigilance device of an activated cab brakes when it has no power
	for (auto const pullee : {false, true}) {
		CAPTURE(pullee);
		test::reseed_random();
		vehicle vectron{definition::fiz("vectron_v1/wariant_7w"), 0.0, "test", 1};
		vectron->InactiveCabFlag = 0;
		if (pullee) {
			REQUIRE(vectron->PulleeModeSwitch(true));
		}
		CHECK(vectron->CabActivisation() == (false == pullee));
		vectron.run(0.5);
		CHECK(vectron->SecuritySystem.is_braking() == (false == pullee));
		CHECK((vectron->EmergencyValveFlow > 0.0) == (false == pullee));
	}
}

TEST_CASE("pullee: the battery and cab commands of the train's driver pass a vehicle prepared for hauling by") {
	// coupled with the control cable too, which a hauled vehicle normally isn't
	for (auto const pullee : {false, true}) {
		CAPTURE(pullee);
		test::reseed_random();
		test::consist train;
		auto &locomotive{train.add(definition::fiz("4e_v1/4e"), 1)};
		auto &hauled{train.add(definition::fiz("4e_v1/4e"))};
		auto &last{train.add(definition::fiz("4e_v1/4e"))};
		if (pullee) {
			REQUIRE(hauled->PulleeModeSwitch(true));
		}
		REQUIRE_FALSE(hauled->Battery);
		locomotive->BatterySwitch(true);
		locomotive->CabActivisation();
		train.run(2.0);
		CHECK(hauled->Battery == (false == pullee));
		CHECK(hauled->CabActive == (pullee ? 0 : 1));
		// the vehicle behind it gets them either way
		CHECK(last->Battery);
		CHECK(last->CabActive == 1);
	}
}

TEST_CASE("pullee: every real vehicle prepared for hauling runs free behind a locomotive, with no air of its own") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		if (false == can_be_hauled(path)) {
			continue;
		}
		test::reseed_random();
		hauled_train train{definition::fiz(path), 60.0, true};
		auto const &hauled{train.hauled.mover()};
		drain_feed_air(train.hauled.mover());
		train.run(60.0);
		CHECK(hauled.EmergencyValveFlow == 0.0);
		CHECK(hauled.BrakePress < 0.1);
		CHECK(hauled.Fb < 1.0);
		CHECK(train.locomotive->PipePress > 4.99);
		CHECK(train.hauled.speed() > 40.0);
	}
}

TEST_CASE("pullee: every real vehicle standing braked releases once prepared for hauling and charged") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		if (false == can_be_hauled(path)) {
			continue;
		}
		test::reseed_random();
		// hand brakes on, spring brakes switched on, cylinders half full, the pipe at 3.5
		hauled_train train{definition::fiz(path), 0.0, true};
		auto const &hauled{train.hauled.mover()};
		CHECK(hauled.ManualBrakePos == 0);
		CHECK_FALSE(hauled.SpringBrake.Activate);
		CHECK_FALSE(hauled.Battery);
		train.run(180.0);
		CHECK(hauled.EmergencyValveFlow == 0.0);
		CHECK(hauled.BrakePress < 0.1);
		// the cylinders of a few are still letting out the last of their air (6Dg, SM42, 810: 15-72 N)
		CHECK(hauled.Fb < 500.0);
	}
}
