#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"
#include "golden.h"

#include "physics/brakes.h"

// the genset of a dual-mode vehicle (eu07/maszyna#536): a second source of the traction circuit of an induction
// motor vehicle next to its pantographs, which the traction selector picks. the vehicle is the game's E6ACT
// (Dragon 2) with the genset of the E6ACTad "last mile" set in the test: 520 kW, the rest assumed

using test::consist;
using test::definition;
using test::track;
using test::vehicle;
using traction_mode = TMoverParameters::traction_mode;

namespace {

definition dragon() {
	return definition::fiz("e6act_v2/e6acta");
}

definition dragon_genset() {
	return dragon()
	    .set("Power", "GensetPower", 520)
	    .set("Power", "GensetVoltage", 3000)
	    .set("Power", "GensetIdleRPM", 700)
	    .set("Power", "GensetMaxRPM", 1800)
	    .set("Power", "GensetStartTime", 8)
	    .set("Power", "GensetSpinUpTime", 3)
	    .set("Power", "GensetPmaxRatio", 0.85);
}

// the power the traction of dragon_genset() may take at full revolutions [kW]
double const genset_traction_power{520.0 * 0.85};

track const no_wire{0.15, 0.0, 0.0};
track const wire{0.15, 0.0, test::catenary_3kv};

// the battery, the cab and a direction, as the driver begins
void wake(TMoverParameters &Cab) {
	Cab.BatterySwitch(true);
	Cab.CabActivisation();
	for (int i = 0; i < 3 && Cab.DirActive <= 0; ++i) {
		Cab.DirectionForward();
	}
}

// the vigilance and the cab signalling acknowledged, as test::drive() does once a second
void attend(TMoverParameters &Cab) {
	if (Cab.SecuritySystem.is_cabsignal_blinking() && Cab.SecuritySystem.has_separate_acknowledge()) {
		Cab.SecuritySystem.cabsignal_reset();
	}
	if (Cab.SecuritySystem.is_blinking()) {
		Cab.SecuritySystemReset();
	}
}

void run(vehicle &Vehicle, double const Seconds, track const &Track = no_wire) {
	test::drive(test::driving(Vehicle), Track, Seconds);
}

// the line breaker closed, which it does only with the controller at zero, and after the relays are reset
void close_line_breaker(TMoverParameters &Cab) {
	while (Cab.DecMainCtrl(2)) {
		// down to zero
	}
	Cab.RelayReset(relay_t::maincircuitground | relay_t::tractionnmotoroverload | relay_t::primaryconverteroverload);
	Cab.MainSwitch(true);
}

// gets the vehicle ready to run on its genset, as test::start_up() does on the pantographs: the selector on the
// genset, the genset started, the line breaker closed once it runs, the converter and the compressor, the air, the
// brakes released. one round of the driver's actions a second; returns whether the vehicle got ready
bool start_on_genset(vehicle &Vehicle, track const &Track = no_wire, double const Seconds = 300.0) {
	auto &cab{Vehicle.mover()};
	for (auto second = 0; second < Seconds; ++second) {
		wake(cab);
		cab.TractionModeSwitch(traction_mode::genset);
		cab.GensetSwitch(true);
		if (false == cab.Mains) {
			if (cab.MainSwitchCheck()) {
				close_line_breaker(cab);
			}
		}
		else {
			cab.ConverterSwitch(true);
			cab.CompressorSwitch(true);
		}
		run(Vehicle, 1.0, Track);
		if (cab.Mains && cab.ConverterFlag && cab.ScndPipePress >= 4.5) {
			// and the brakes let go
			test::release_brakes(cab);
			for (auto wait = 0; wait < 120 && cab.BrakePress > 0.05; ++wait) {
				run(Vehicle, 1.0, Track);
			}
			return cab.BrakePress <= 0.05;
		}
	}
	return false;
}

// the main controller to its full power position
void notch_up(TMoverParameters &Cab) {
	for (int i = 0; i < 60 && false == Cab.IsMainCtrlMaxPowerPos(); ++i) {
		Cab.IncMainCtrl(1);
	}
}

// the power at the wheels [kW]
double wheel_power(TMoverParameters const &Vehicle) {
	return std::abs(Vehicle.Ft * Vehicle.V) / 1000.0;
}

// one step of 0.01 s, acknowledging the vigilance once a second
void step(vehicle &Vehicle, long const Step, track const &Track = no_wire) {
	if (Step % 100 == 0) {
		attend(Vehicle.mover());
	}
	Vehicle.step(Track);
}

} // namespace

TEST_CASE("genset: a vehicle without the keys has none and stays on the catenary") {
	vehicle dragon6{dragon(), 0.0, "plain", 1};
	auto &cab{dragon6.mover()};
	CHECK(cab.Genset.power == 0.0);
	CHECK_FALSE(cab.Genset.fitted());
	CHECK_FALSE(cab.TractionModeSwitch(traction_mode::genset));
	CHECK(cab.TractionMode == traction_mode::catenary);
	CHECK_FALSE(cab.GensetSelected());
	REQUIRE(test::start_up(test::driving(dragon6), wire));
	CHECK(cab.PantographVoltage > 2900.0);
	CHECK(cab.TractionSupplyVoltage() == std::max(cab.GetTrainsetHighVoltage(), cab.PantographVoltage));
	CHECK(cab.TractionSupplyMinVoltage() == cab.EnginePowerSource.CollectorParameters.MinV);
	CHECK(cab.GensetTractionPowerLimit() == std::numeric_limits<double>::max());
	// a start request does nothing without a genset
	cab.GensetSwitch(true);
	run(dragon6, 20.0, wire);
	CHECK_FALSE(cab.Genset.is_active);
	CHECK(cab.Genset.voltage == 0.0);
	CHECK(cab.TractionMode == traction_mode::catenary);
}

TEST_CASE("genset: the keys reach the vehicle") {
	vehicle dragon6{dragon_genset()};
	auto const &genset{dragon6->Genset};
	CHECK(genset.fitted());
	CHECK(genset.power == 520.0);
	CHECK(genset.voltage_nominal == 3000.0);
	CHECK(genset.revolutions_idle == doctest::Approx(700.0 / 60.0));
	CHECK(genset.revolutions_max == doctest::Approx(30.0));
	CHECK(genset.start_time == 8.0);
	CHECK(genset.spinup_time == 3.0);
	CHECK(genset.traction_share == 0.85);
	CHECK(genset.start_type == start_t::manual);
	CHECK(dragon6->TractionMode == traction_mode::catenary);
	// the pantographs stay what they were
	CHECK(dragon6->EnginePowerSource.SourceType == TPowerSource::CurrentCollector);
	CHECK(dragon6->EnginePowerSource.CollectorParameters.MinV == 2000.0);

	SUBCASE("started on its own") {
		vehicle automatic{dragon_genset().set("Cntrl", "GensetStart", "Automatic")};
		CHECK(automatic->Genset.start_type == start_t::automatic);
	}
	SUBCASE("the defaults") {
		vehicle defaults{dragon().set("Power", "GensetPower", 520)};
		auto const &plain{defaults->Genset};
		CHECK(plain.fitted());
		CHECK(plain.voltage_nominal == doctest::Approx(0.65 * 4600.0)); // of the pantographs' MaxVoltage
		CHECK(plain.revolutions_idle == doctest::Approx(10.0));
		CHECK(plain.revolutions_max == doctest::Approx(30.0));
		CHECK(plain.start_time == 8.0);
		CHECK(plain.spinup_time == 3.0);
		CHECK(plain.traction_share == 1.0);
	}
}

TEST_CASE("genset: the keys are ignored on other than induction motor vehicles") {
	for (auto const *path : {"4e_v1/4e", "br285_v1/br285"}) {
		CAPTURE(path);
		vehicle other{definition::fiz(path).set("Power", "GensetPower", 520)};
		CHECK_FALSE(other->Genset.fitted());
		CHECK_FALSE(other->TractionModeSwitch(traction_mode::genset));
		CHECK(other->TractionMode == traction_mode::catenary);
	}
}

TEST_CASE("genset: the selector moves only with the line breaker open") {
	vehicle dragon6{dragon_genset(), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	REQUIRE(test::start_up(test::driving(dragon6), wire));
	REQUIRE(cab.Mains);
	CHECK_FALSE(cab.TractionModeSwitch(traction_mode::genset));
	CHECK(cab.TractionMode == traction_mode::catenary);
	run(dragon6, 2.0, wire);
	CHECK(cab.Mains);
	cab.MainSwitch(false);
	REQUIRE_FALSE(cab.Mains);
	CHECK(cab.TractionModeSwitch(traction_mode::genset));
	CHECK(cab.GensetSelected());
	CHECK(cab.TractionModeSwitch(traction_mode::catenary));
	CHECK(cab.TractionMode == traction_mode::catenary);
}

TEST_CASE("genset: started, it closes the line breaker without a catenary") {
	vehicle dragon6{dragon_genset(), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	wake(cab);
	REQUIRE(cab.TractionModeSwitch(traction_mode::genset));
	cab.GensetSwitch(true);
	run(dragon6, 7.0);
	CHECK(cab.Genset.is_cranking);
	CHECK_FALSE(cab.Genset.is_active);
	CHECK(cab.Genset.voltage == 0.0);
	CHECK_FALSE(cab.MainSwitchCheck());
	run(dragon6, 2.0);
	CHECK(cab.Genset.is_active);
	CHECK_FALSE(cab.Genset.is_cranking);
	CHECK(cab.Genset.revolutions == cab.Genset.revolutions_idle);
	CHECK(cab.Genset.voltage == 3000.0);
	// the line breaker gets ready MainInitTime after the voltage came
	for (auto second = 0; second < 10 && false == cab.Mains; ++second) {
		if (cab.MainSwitchCheck()) {
			close_line_breaker(cab);
		}
		run(dragon6, 1.0);
	}
	REQUIRE(cab.Mains);
	CHECK(cab.PantographVoltage == 0.0);
	dragon6.step(no_wire);
	CHECK(cab.EngineVoltage == 3000.0);
	CHECK(cab.MainCircuitPowered());
	run(dragon6, 5.0);
	CHECK(cab.Mains);
}

TEST_CASE("genset: on the catenary position the genset feeds nothing") {
	vehicle dragon6{dragon_genset(), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	wake(cab);
	cab.GensetSwitch(true);
	run(dragon6, 20.0);
	REQUIRE(cab.Genset.is_active);
	CHECK(cab.TractionMode == traction_mode::catenary);
	CHECK_FALSE(cab.MainSwitchCheck());
	close_line_breaker(cab);
	CHECK_FALSE(cab.Mains);
	dragon6.step(no_wire);
	CHECK(cab.EngineVoltage == 0.0);
	CHECK(cab.TractionSupplyVoltage() == 0.0);
	// running isolated, at idle
	CHECK(cab.Genset.revolutions == cab.Genset.revolutions_idle);
}

TEST_CASE("genset: on the genset position the catenary feeds nothing") {
	vehicle dragon6{dragon_genset(), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	wake(cab);
	REQUIRE(cab.TractionModeSwitch(traction_mode::genset));
	// the pantographs up under the wire, the genset stopped: the line breaker never closes
	CHECK_FALSE(test::start_up(test::driving(dragon6), wire, 120.0));
	REQUIRE(cab.PantographVoltage > 2900.0);
	CHECK_FALSE(cab.Mains);
	CHECK_FALSE(cab.MainSwitchCheck());
	dragon6.step(wire);
	CHECK(cab.EngineVoltage == 0.0);
	CHECK(cab.TractionSupplyVoltage() == 0.0);
}

TEST_CASE("genset: the power at the wheels stays within the genset's") {
	vehicle dragon6{dragon_genset(), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	REQUIRE(start_on_genset(dragon6));
	notch_up(cab);
	auto peak{0.0};
	auto revolutions{0.0};
	auto over{0L};
	auto step_count{0L};
	for (; step_count < 60000 && cab.Vel < 80.0 && cab.Mains; ++step_count) {
		step(dragon6, step_count);
		auto const power{wheel_power(cab)};
		peak = std::max(peak, power);
		revolutions = std::max(revolutions, cab.Genset.revolutions);
		if (power > genset_traction_power * 1.02) {
			++over;
		}
	}
	CAPTURE(step_count);
	CAPTURE(peak);
	CHECK(cab.Mains);
	CHECK(cab.Vel >= 80.0);
	CHECK(over == 0);
	CHECK(peak <= genset_traction_power * 1.02);
	CHECK(peak >= genset_traction_power * 0.95); // the limit bites
	CHECK(revolutions >= 0.95 * cab.Genset.revolutions_max);
}

TEST_CASE("genset: the revolutions follow the demand, so the power builds up") {
	vehicle dragon6{dragon_genset(), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	REQUIRE(start_on_genset(dragon6));
	// at idle with nothing asked of it
	CHECK(cab.Genset.revolutions == doctest::Approx(cab.Genset.revolutions_idle));
	CHECK(cab.GensetTractionPowerLimit() == doctest::Approx(genset_traction_power * 700.0 / 1800.0).epsilon(0.001));
	notch_up(cab);
	// the demand of the traction builds up at the pace of the inverters, the revolutions follow it
	auto step_count{0L};
	for (; step_count < 3000 && cab.eimv_pr < 0.99; ++step_count) {
		step(dragon6, step_count);
	}
	CAPTURE(step_count);
	REQUIRE(cab.eimv_pr >= 0.99);
	run(dragon6, 3.0);
	auto const after3{cab.Genset.available_power()};
	CAPTURE(after3);
	// first order: 1 - exp(-1) of the way from idle to full in the time constant at the least
	CHECK(after3 > 520.0 * 0.75);
	run(dragon6, 9.0);
	CHECK(cab.Genset.available_power() > 520.0 * 0.95);
	CHECK(cab.GensetTractionPowerLimit() == doctest::Approx(cab.Genset.available_power() * 0.85));
	// notched down it goes back to idle
	while (cab.DecMainCtrl(2)) {
		// down to zero
	}
	run(dragon6, 20.0);
	CHECK(cab.Genset.revolutions == doctest::Approx(cab.Genset.revolutions_idle).epsilon(0.01));
}

TEST_CASE("genset: running from the catenary is the same with the keys") {
	// the values of a run of each one, every step
	struct values {
		std::vector<double> force;
		std::vector<double> speed;
		std::vector<double> voltage;
		std::vector<double> current;
		std::vector<double> demand;
	};
	auto const record = [](values &Values) {
		return std::vector<test::channel>{
		    {"Ft", [&Values](TMoverParameters const &Vehicle) { Values.force.push_back(Vehicle.Ft); return Vehicle.Ft; }},
		    {"V", [&Values](TMoverParameters const &Vehicle) { Values.speed.push_back(Vehicle.V); return Vehicle.V; }},
		    {"EngineVoltage", [&Values](TMoverParameters const &Vehicle) { Values.voltage.push_back(Vehicle.EngineVoltage); return Vehicle.EngineVoltage; }},
		    {"Itot", [&Values](TMoverParameters const &Vehicle) { Values.current.push_back(Vehicle.Itot); return Vehicle.Itot; }},
		    {"eimv_pr", [&Values](TMoverParameters const &Vehicle) { Values.demand.push_back(Vehicle.eimv_pr); return Vehicle.eimv_pr; }},
		};
	};
	auto const drive = [&record](definition const &Definition, values &Values) {
		test::reseed_random();
		vehicle dragon6{Definition, 0.0, "test", 1};
		// not checked against a recorded one: the runs are compared with each other
		test::trace trace{"genset/catenary", record(Values), 1, false};
		dragon6.record(trace);
		test::drive_cycle(test::driving(dragon6), wire, trace);
	};
	values plain;
	values genset;
	drive(dragon(), plain);
	drive(dragon_genset(), genset);
	REQUIRE(plain.speed.size() > 10000);
	REQUIRE(genset.speed.size() == plain.speed.size());
	CHECK(*std::ranges::max_element(plain.speed) > 10.0);
	// the first step they differ in, if any
	auto const first_difference = [](std::vector<double> const &Left, std::vector<double> const &Right) {
		return std::ranges::mismatch(Left, Right).in1 - Left.begin();
	};
	auto const size{static_cast<std::ptrdiff_t>(plain.speed.size())};
	CHECK(first_difference(plain.force, genset.force) == size);
	CHECK(first_difference(plain.speed, genset.speed) == size);
	CHECK(first_difference(plain.voltage, genset.voltage) == size);
	CHECK(first_difference(plain.current, genset.current) == size);
	CHECK(first_difference(plain.demand, genset.demand) == size);
}

TEST_CASE("genset: stopped, it opens the line breaker") {
	vehicle dragon6{dragon_genset(), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	REQUIRE(start_on_genset(dragon6));
	REQUIRE(cab.Genset.revolutions == doctest::Approx(cab.Genset.revolutions_idle));
	cab.GensetSwitch(false);
	auto step_count{0L};
	for (; step_count < 1000 && cab.Mains; ++step_count) {
		step(dragon6, step_count);
	}
	// coasting down from idle, the voltage falls to half, the undervoltage limit, in spinup_time * ln 2
	auto const seconds{step_count * 0.01};
	CAPTURE(seconds);
	CHECK_FALSE(cab.Mains);
	CHECK(seconds == doctest::Approx(3.0 * std::log(2.0)).epsilon(0.05));
	CHECK_FALSE(cab.Genset.is_active);
	// and it doesn't close again
	for (auto second = 0; second < 20; ++second) {
		close_line_breaker(cab);
		run(dragon6, 1.0);
	}
	CHECK_FALSE(cab.Mains);
	// coasting down to a stop, with the time constant spinup_time
	CHECK(cab.Genset.revolutions < 0.01 * cab.Genset.revolutions_idle);
	CHECK(cab.Genset.voltage < 0.01 * cab.Genset.voltage_nominal);
}

TEST_CASE("genset: automatic start follows the selector") {
	vehicle dragon6{dragon_genset().set("Cntrl", "GensetStart", "Automatic"), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	wake(cab);
	run(dragon6, 10.0);
	CHECK_FALSE(cab.Genset.is_cranking);
	CHECK_FALSE(cab.Genset.is_active);
	REQUIRE(cab.TractionModeSwitch(traction_mode::genset));
	run(dragon6, 1.0);
	CHECK(cab.Genset.is_cranking);
	run(dragon6, 8.0);
	CHECK(cab.Genset.is_active);
	// the start switch does nothing to it
	CHECK_FALSE(cab.GensetSwitch(false));
	run(dragon6, 1.0);
	CHECK(cab.Genset.is_active);
	REQUIRE(cab.TractionModeSwitch(traction_mode::catenary));
	dragon6.step(no_wire);
	CHECK_FALSE(cab.Genset.is_active);
}

TEST_CASE("genset: no battery, no start") {
	vehicle dragon6{dragon_genset(), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	cab.BatterySwitch(false);
	REQUIRE(cab.TractionModeSwitch(traction_mode::genset));
	cab.GensetSwitch(true);
	run(dragon6, 20.0);
	CHECK_FALSE(cab.Power24vIsAvailable);
	CHECK_FALSE(cab.Genset.is_cranking);
	CHECK_FALSE(cab.Genset.is_active);
	CHECK(cab.Genset.revolutions == 0.0);
	// and with it, it starts
	wake(cab);
	run(dragon6, 10.0);
	CHECK(cab.Genset.is_active);
}

TEST_CASE("genset: the pantographs' pressure switch leaves the genset's line breaker alone") {
	SUBCASE("on the genset") {
		vehicle dragon6{dragon_genset(), 0.0, "test", 1};
		auto &cab{dragon6.mover()};
		REQUIRE(start_on_genset(dragon6));
		cab.PantPressSwitchActive = true;
		cab.PantPress = 0.0;
		cab.PantPressureSwitchCheck();
		CHECK(cab.Mains);
	}
	SUBCASE("on the pantographs it opens the line breaker, the check the genset is spared") {
		vehicle dragon6{dragon_genset(), 0.0, "test", 1};
		auto &cab{dragon6.mover()};
		REQUIRE(test::start_up(test::driving(dragon6), wire));
		REQUIRE(cab.Mains);
		// the pressure switch looks at the voltage through the couplers, not the pantographs'
		cab.PantPressSwitchActive = true;
		cab.PantPress = 0.0;
		cab.PantPressureSwitchCheck();
		CHECK_FALSE(cab.Mains);
	}
}

namespace {

// what braking showed of the current of the traction circuit
struct braking {
	double lowest_current{0.0}; // [A], negative going back to the source
	bool electrodynamic{false}; // whether the electrodynamic brake took part
	double speed_lost{0.0}; // [km/h] in the 20 s of braking
};

// up to 60 km/h at full power, then braking with the train brake for 20 s, which the E6ACT blends with the
// electrodynamic one
braking brake_from_60(vehicle &Dragon, track const &Track) {
	auto &cab{Dragon.mover()};
	notch_up(cab);
	for (auto step_count = 0L; step_count < 60000 && cab.Vel < 60.0; ++step_count) {
		step(Dragon, step_count, Track);
	}
	REQUIRE(cab.Vel >= 60.0);
	while (cab.DecMainCtrl(2)) {
		// down to zero
	}
	run(Dragon, 2.0, Track);
	cab.DynamicBrakeSwitch(true);
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_FB));
	braking result;
	auto const speed{cab.Vel};
	for (auto step_count = 0L; step_count < 2000; ++step_count) {
		step(Dragon, step_count, Track);
		result.lowest_current = std::min(result.lowest_current, cab.Itot);
		result.electrodynamic |= cab.eimv_pr < 0.0 && cab.eimv[eimv_Ipoj] < 0.0;
	}
	result.speed_lost = speed - cab.Vel;
	return result;
}

} // namespace

TEST_CASE("genset: braking electrodynamically on the genset sends nothing back") {
	SUBCASE("on the genset") {
		vehicle dragon6{dragon_genset(), 0.0, "test", 1};
		REQUIRE(start_on_genset(dragon6));
		auto const braked{brake_from_60(dragon6, no_wire)};
		REQUIRE(braked.electrodynamic);
		CHECK(braked.lowest_current >= -1e-6);
		CHECK(braked.speed_lost > 5.0);
		CHECK(dragon6->Mains);
	}
	SUBCASE("on the catenary the same braking feeds the line, the check the genset is spared") {
		vehicle dragon6{dragon_genset(), 0.0, "test", 1};
		REQUIRE(test::start_up(test::driving(dragon6), wire));
		test::release_brakes(dragon6.mover());
		run(dragon6, 5.0, wire);
		auto const braked{brake_from_60(dragon6, wire)};
		REQUIRE(braked.electrodynamic);
		CHECK(braked.lowest_current < -10.0);
		CHECK(braked.speed_lost > 5.0);
	}
}

TEST_CASE("genset: one genset feeds the next car through the high voltage coupling") {
	// both able to join the control and the high voltage lines: 35 of the E6ACT, + control 4 + high voltage 8
	consist train;
	auto &first{train.add(dragon_genset().set("BuffCoupl", "AllowedFlag", 47), 1).mover()};
	auto &second{train.add(dragon().set("BuffCoupl", "AllowedFlag", 47)).mover()};
	REQUIRE(first.Couplers[end::rear].CouplingFlag.HasAny(Coupling::highVoltage));
	wake(first);
	REQUIRE(first.TractionModeSwitch(traction_mode::genset));
	first.GensetSwitch(true);
	train.run(5.0, no_wire);
	CHECK(second.GetTrainsetHighVoltage() == 0.0);
	train.run(10.0, no_wire);
	REQUIRE(first.Genset.is_active);
	CHECK(second.GetTrainsetHighVoltage() > 2900.0);
	CHECK(second.PantographVoltage == 0.0);
	// the second one, on its catenary position with the pantographs down, takes it as the catenary's
	second.BatterySwitch(true);
	train.run(10.0, no_wire);
	second.RelayReset(relay_t::maincircuitground | relay_t::tractionnmotoroverload | relay_t::primaryconverteroverload, range_t::local);
	CHECK(second.MainSwitchCheck());
	second.MainSwitch(true, range_t::local);
	CHECK(second.Mains);
	// stopped, the genset takes the voltage away
	first.GensetSwitch(false);
	train.run(30.0, no_wire);
	CHECK(second.GetTrainsetHighVoltage() < 1.0);
	CHECK_FALSE(second.Mains);
}

TEST_CASE("genset: the selector command travels along the train") {
	// two locomotives in multiple: the control line, no permanent coupling
	auto const coupling{Coupling::coupler | Coupling::brakeHose | Coupling::mainHose | Coupling::control};
	consist train;
	auto &first{train.add(dragon_genset().set("BuffCoupl", "AllowedFlag", 47), 1).mover()};
	auto &second{train.add(dragon_genset().set("BuffCoupl", "AllowedFlag", 47), 0, coupling).mover()};
	REQUIRE(first.Couplers[end::rear].CouplingFlag.HasAny(Coupling::control));
	wake(first);
	train.run(1.0, no_wire);
	CHECK(first.TractionModeSwitch(traction_mode::genset, range_t::consist));
	CHECK(first.GensetSelected());
	CHECK(second.GensetSelected());
	first.GensetSwitch(true, range_t::consist);
	CHECK(first.Genset.is_enabled);
	CHECK(second.Genset.is_enabled);
	first.TractionModeSwitch(traction_mode::catenary, range_t::consist);
	CHECK_FALSE(second.GensetSelected());
	// the unit range keeps to the permanent couplings, which locomotives in multiple don't have
	first.TractionModeSwitch(traction_mode::genset, range_t::unit);
	CHECK(first.GensetSelected());
	CHECK_FALSE(second.GensetSelected());
}

TEST_CASE("genset: a selector command refused with the line breaker closed isn't passed on") {
	auto const coupling{Coupling::coupler | Coupling::brakeHose | Coupling::mainHose | Coupling::control};
	consist train;
	auto &first{train.add(dragon_genset().set("BuffCoupl", "AllowedFlag", 47), 1).mover()};
	auto &second{train.add(dragon_genset().set("BuffCoupl", "AllowedFlag", 47), 0, coupling).mover()};
	REQUIRE(test::start_up(test::driving(train), wire));
	REQUIRE(first.Mains);
	// the second one's line breaker opened by itself, the lead's is closed
	second.MainSwitch(false, range_t::local);
	REQUIRE_FALSE(second.Mains);
	CHECK_FALSE(first.TractionModeSwitch(traction_mode::genset, range_t::consist));
	CHECK_FALSE(first.GensetSelected());
	CHECK_FALSE(second.GensetSelected());
	// with the lead's open it goes along
	first.MainSwitch(false, range_t::local);
	CHECK(first.TractionModeSwitch(traction_mode::genset, range_t::consist));
	CHECK(first.GensetSelected());
	CHECK(second.GensetSelected());
}

TEST_CASE("genset: a lead which starts its genset on its own passes the start switch on") {
	auto const coupling{Coupling::coupler | Coupling::brakeHose | Coupling::mainHose | Coupling::control};
	consist train;
	auto &first{train.add(dragon_genset().set("BuffCoupl", "AllowedFlag", 47).set("Cntrl", "GensetStart", "Automatic"), 1).mover()};
	auto &second{train.add(dragon_genset().set("BuffCoupl", "AllowedFlag", 47), 0, coupling).mover()};
	wake(first);
	train.run(1.0, no_wire);
	// ignored by the lead, so its cab switch doesn't move, but the car behind starts on it
	CHECK_FALSE(first.GensetSwitch(true, range_t::consist));
	CHECK_FALSE(first.Genset.is_enabled);
	CHECK(second.Genset.is_enabled);
	CHECK_FALSE(first.GensetSwitch(false, range_t::consist));
	CHECK_FALSE(second.Genset.is_enabled);
}

TEST_CASE("genset: on the genset the pantographs carry no traction current") {
	SUBCASE("on the genset") {
		vehicle dragon6{dragon_genset(), 0.0, "test", 1};
		auto &cab{dragon6.mover()};
		REQUIRE(start_on_genset(dragon6));
		notch_up(cab);
		run(dragon6, 10.0);
		REQUIRE(cab.Itot > 50.0);
		CHECK(cab.CatenaryCurrent() == 0.0);
		// only a heating fed straight from the pantographs takes from the wire
		cab.TotalCurrent = 40.0;
		cab.HeatingPowerSource.SourceType = TPowerSource::CurrentCollector;
		CHECK(cab.CatenaryCurrent() == 40.0);
		cab.HeatingPowerSource.SourceType = TPowerSource::Main;
		CHECK(cab.CatenaryCurrent() == 0.0);
	}
	SUBCASE("on the catenary the traction current goes through them, the check the genset is spared") {
		vehicle dragon6{dragon_genset(), 0.0, "test", 1};
		auto &cab{dragon6.mover()};
		REQUIRE(test::start_up(test::driving(dragon6), wire));
		test::release_brakes(cab);
		run(dragon6, 5.0, wire);
		notch_up(cab);
		for (auto step_count = 0L; step_count < 60000 && cab.Vel < 30.0; ++step_count) {
			step(dragon6, step_count, wire);
		}
		CAPTURE(cab.Vel);
		REQUIRE(cab.Itot > 50.0);
		CHECK(cab.CatenaryCurrent() == std::abs(cab.Itot) * cab.IsVehicleEIMBrakingFactor() + cab.TotalCurrent);
		CHECK(cab.CatenaryCurrent() > 50.0);
	}
}

TEST_CASE("genset: a scenery command reaches the selector") {
	vehicle dragon6{dragon_genset(), 0.0, "test", 1};
	auto &cab{dragon6.mover()};
	wake(cab);
	// as a scenery's PutValues event gives it, one at a time
	cab.PutCommand("TractionMode", 1, 0, cab.Loc);
	dragon6.step(no_wire);
	CHECK(cab.GensetSelected());
	CHECK(cab.CommandIn.Command == VehicleCommand::none);
	cab.PutCommand("GensetSwitch", 1, 0, cab.Loc);
	dragon6.step(no_wire);
	CHECK(cab.Genset.is_enabled);
	run(dragon6, 10.0);
	CHECK(cab.Genset.is_active);
	cab.PutCommand("TractionMode", 0, 0, cab.Loc);
	dragon6.step(no_wire);
	CHECK_FALSE(cab.GensetSelected());
}
