#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

#include "physics/brakes.h"

#include <format>
#include <vector>

using test::definition;
using test::vehicle;

// the fill and release times of the brake cylinder of an ESt3 distributor with the AL2 load relay (TNESt3 with
// TPrzekCiagly), in G and in P, with the empty/loaded lever in each of its three positions. bb43f559e made the
// relay's impulse chamber 1 l and scaled the cylinder nozzles to the times of table 3-9 of the ESt3 data; before it
// they were set by eye. this pins what the valve does today, so a change of the relay or of the nozzles shows

namespace {

// a Fas 413S four-axle freight wagon (pkp/413s_v1/413s.fiz of the game data): ESt3AL2, 100 l reservoir, one
// 16" cylinder, the empty/medium/loaded lever at 1.5/2.6/3.8 bar, brake delays G and P. not among the converted
// real vehicles of tests/unit/data/fiz, so its lines are set over the test wagon
definition fas_413s() {
	return definition::load("wagon")
	    .set("Param", "M", 25300)
	    .set("Param", "Vmax", 100)
	    .set("Load", "MaxLoad", 56)
	    .set("Load", "LoadQ", "tonns")
	    .set("Load", "LoadAccepted", "coal,gravel")
	    .set("Dimensions", "L", 14.4)
	    .set("Wheels", "D", 0.92)
	    .set("Wheels", "Axle", "2'2'")
	    .set("Wheels", "Ad", 2.0)
	    .set("Wheels", "Bd", 8.5)
	    .set("Brake", "BrakeValve", "ESt3AL2")
	    .set("Brake", "Size", 100)
	    .set("Brake", "MBF", 144)
	    .set("Brake", "MaxBP", 3.8)
	    .set("Brake", "TareMaxBP", 1.5)
	    .set("Brake", "MedMaxBP", 2.6)
	    .set("Brake", "BCN", 1)
	    .set("Brake", "BCR", 0.2032)
	    .set("Brake", "BCD", 0.14)
	    .set("Brake", "BCM", 11.75)
	    .set("Brake", "NBpA", 4)
	    .set("Brake", "BM", "P10-Bg")
	    .set("Brake", "BCS", 2.7)
	    .set("Brake", "BSA", 16.0)
	    .set("Brake", "BVV", 100)
	    .set("Brake", "BRE", 0.8)
	    .set("Cntrl", "BrakeDelays", "GP")
	    .remove("Cntrl", "MaxBPMass");
}

struct times {
	double charged; // in the cylinder after charging [bar]
	double applied; // in the cylinder 60 s after the emergency application [bar]
	double fill; // from the pipe emptied to 95% of applied [s]
	double release; // from the pipe back at 5 bar to 0.4 bar in the cylinder [s]
};

// charges the wagon, empties the pipe at once (an emergency application) and fills it again at once, stepping
// the physics in 0.01 s steps as the game does
times measure(int const Delay, int const Lever) {
	test::reseed_random();
	vehicle wagon{fas_413s()};
	REQUIRE(wagon->BrakeValve == TBrakeValve::ESt3AL2);
	if (wagon->BrakeDelayFlag != Delay) {
		REQUIRE(wagon->BrakeDelaySwitch(Delay));
	}
	// an empty wagon starts with the lever at empty (TMoverParameters::CheckLocomotiveParameters)
	REQUIRE(wagon->LoadFlag == 1);
	while (wagon->LoadFlag < Lever) {
		REQUIRE(wagon->IncBrakeMult());
	}
	REQUIRE(wagon->LoadFlag == Lever);

	times result{};
	wagon.hold_brake_pipe(5.0);
	wagon.run(180.0);
	result.charged = wagon->BrakePress;

	double constexpr step{0.01};
	std::vector<double> cylinder;
	wagon.hold_brake_pipe(0.0);
	for (int i = 0; i < 6000; ++i) {
		wagon.step();
		cylinder.push_back(wagon->BrakePress);
	}
	result.applied = cylinder.back();
	result.fill = -1.0;
	for (std::size_t i = 0; i < cylinder.size(); ++i) {
		if (cylinder[i] >= 0.95 * result.applied) {
			result.fill = static_cast<double>(i + 1) * step;
			break;
		}
	}

	wagon.hold_brake_pipe(5.0);
	result.release = -1.0;
	for (int i = 0; i < 18000; ++i) {
		wagon.step();
		if (wagon->BrakePress <= 0.4) {
			result.release = static_cast<double>(i + 1) * step;
			break;
		}
	}
	return result;
}

} // namespace

TEST_CASE("ESt3AL2: fill and release times of the cylinder of a 413S wagon, G and P, empty, medium and loaded") {
	struct expected {
		char const *name;
		int delay;
		int lever; // LoadFlag: 1 empty, 2 medium, 3 loaded
		double applied; // [bar]
		double fill; // [s]
		double release; // [s]
	};
	// measured on staging 5a24a1048, held to 1% (the pressure) and 5% (the times). the fill time is to 95% of the
	// pressure 60 s after the pipe was emptied, the release time down to 0.4 bar. both count from a step of the pipe,
	// 5 to 0 bar and back at once, not from the rate of a driver's brake valve or of a test bench. for comparison only,
	// UIC 540 asks for G fill 18-30 s, release 45-60 s, P fill 3-5 s, release 15-20 s: the fill times are within, the
	// release times shorter. the emergency application gives the cylinder 0.19-0.27 bar more than the lever's
	// TareMaxBP, MedMaxBP and MaxBP (1.5, 2.6, 3.8 bar)
	for (auto const &[name, delay, lever, applied, fill, release] : {
	         expected{"G empty", bdelay_G, 1, 1.736, 22.81, 33.89},
	         expected{"G medium", bdelay_G, 2, 2.870, 21.98, 40.82},
	         expected{"G loaded", bdelay_G, 3, 3.991, 21.15, 43.98},
	         expected{"P empty", bdelay_P, 1, 1.737, 3.84, 10.98},
	         expected{"P medium", bdelay_P, 2, 2.871, 3.71, 13.18},
	         expected{"P loaded", bdelay_P, 3, 3.995, 3.62, 14.19},
	     }) {
		CAPTURE(name);
		auto const measured{measure(delay, lever)};
		INFO(std::format("{}: charged {:.3f} bar, applied {:.3f} bar, fill {:.2f} s, release {:.2f} s", name, measured.charged, measured.applied, measured.fill,
		                    measured.release));
		CHECK(measured.charged < 0.05);
		CHECK(measured.applied == doctest::Approx(applied).epsilon(0.01));
		CHECK(measured.fill == doctest::Approx(fill).epsilon(0.05));
		CHECK(measured.release == doctest::Approx(release).epsilon(0.05));
	}
}
