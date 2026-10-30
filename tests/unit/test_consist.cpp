#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"
#include "golden.h"

#include "McZapkie/hamulce.h"

using test::consist;
using test::definition;

// characterisation of trains of coupled vehicles: the real electric units driven whole, a locomotive with
// wagons, the air, the control and the coupler forces between the vehicles, in golden traces
// (data/golden/consist/), each vehicle of the train in its own

namespace {

struct unit {
	char const *name;
	std::vector<char const *> cars; // from the front, the first one driven
};

std::vector<unit> const units{
    {"en57", {"en57_v1/5bgaiii_led_spot", "en57_v1/6ba_840", "en57_v1/5bgb"}},
    {"ed72", {"ed72_v1/5bs-ra", "ed72_v1/6bs-sa", "ed72_v1/6bs-sb", "ed72_v1/5bs-rb"}},
    {"ew58", {"ew58_v1/ew58-sa", "ew58_v1/ew58-d", "ew58_v1/ew58-sb"}},
    {"elf", {"elf_v1/21we-a", "elf_v1/21we-b", "elf_v1/21we-c"}},
    {"impuls", {"impuls_v1/31we-a_pr", "impuls_v1/31we-b_pr", "impuls_v1/31we-c_pr", "impuls_v1/31we-d_pr"}},
};

std::vector<test::channel> car_channels() {
	using namespace test::channels;
	return motion() + traction() + air() + brakes();
}

// a trace for each vehicle of the train, recording it
std::vector<std::unique_ptr<test::trace>> record(consist &Train, std::string const &Name) {
	std::vector<std::unique_ptr<test::trace>> traces;
	for (std::size_t i = 0; i < Train.size(); ++i) {
		traces.push_back(std::make_unique<test::trace>("consist/" + Name + "/" + std::to_string(i), car_channels(), 100));
		Train[i].record(*traces.back());
	}
	return traces;
}

// the events go to every trace
struct all_traces {
	std::vector<std::unique_ptr<test::trace>> &traces;
	void mark(std::string const &Event) {
		for (auto &trace : traces) {
			trace->mark(Event);
		}
	}
};

} // namespace

TEST_CASE("consist: every real electric unit driven whole from its front cab") {
	for (auto const &unit : units) {
		CAPTURE(unit.name);
		test::reseed_random();
		consist train;
		for (auto const *car : unit.cars) {
			train.add(definition::fiz(car), train.size() == 0 ? 1 : 0);
		}
		auto traces{record(train, unit.name)};
		// the drive cycle marks the events in the trace it gets; the first car's carries them
		test::drive_cycle(test::driving(train), test::track{0.15, 0.0, test::catenary_3kv}, *traces.front());
		for (auto &trace : traces) {
			trace->check();
		}
	}
}

TEST_CASE("consist: a locomotive with wagons, driven, braked by its handle and pulled apart by the couplers") {
	test::reseed_random();
	consist train;
	train.add(definition::fiz("4e_v1/4e"), 1);
	for (auto const *wagon : {"3wl_v2/3wl", "11xa_v2/93a", "bdhpumn_v3/bdhpumn", "z1_v2/152a"}) {
		train.add(definition::fiz(wagon));
	}
	auto traces{record(train, "eu07 with wagons")};
	test::drive_cycle(test::driving(train), test::track{0.15, 0.0, test::catenary_3kv}, *traces.front());
	for (auto &trace : traces) {
		trace->check();
	}
	// the couplers kept the train together
	for (std::size_t i = 0; i + 1 < train.size(); ++i) {
		CAPTURE(i);
		CHECK(std::abs(train.gap(i)) < 0.5);
	}
}

namespace {

// what the cab's ammeters show (Train.cpp, the driver's panel) and the brake force the AI expects of the
// vehicle (Driver.cpp), read of the driven vehicle. BrakeForceR() changes nothing, though it isn't const
std::vector<test::channel> gauges() {
	std::vector<test::channel> result;
	for (int ammeter = 0; ammeter < 4; ++ammeter) {
		result.push_back({"ShowCurrent" + std::to_string(ammeter), [ammeter](auto const &M) { return M.ShowCurrent(ammeter); }});
	}
	result.push_back({"BrakeForceR0.25", [](auto const &M) { return const_cast<TMoverParameters &>(M).BrakeForceR(0.25, M.Vel); }});
	result.push_back({"BrakeForceR1", [](auto const &M) { return const_cast<TMoverParameters &>(M).BrakeForceR(1.0, M.Vel); }});
	result.push_back({"BrakeForceR1at20", [](auto const &M) { return const_cast<TMoverParameters &>(M).BrakeForceR(1.0, 20.0); }});
	using test::channels::operator+;
	return result + test::channels::traction();
}

} // namespace

TEST_CASE("consist: the gauges of units and locomotives in multiple, driven") {
	auto trains{units};
	trains.push_back({"eu07 pair", {"4e_v1/4e", "4e_v1/4e"}});
	trains.push_back({"et22 pair", {"et22_v3/201e-w", "et22_v3/201e-rn"}});
	trains.push_back({"ep09 pair", {"ep09_v2/104e", "ep09_v2/104e"}});
	trains.push_back({"sn61 three", {"sn61_v2/sn61", "sn61_v2/sn61", "sn61_v2/sn61"}});
	trains.push_back({"su45 pair", {"su45_v2/301d", "su45_v2/301d"}});
	for (auto const &train : trains) {
		CAPTURE(train.name);
		test::reseed_random();
		consist vehicles;
		for (auto const *car : train.cars) {
			vehicles.add(definition::fiz(car), vehicles.size() == 0 ? 1 : 0);
		}
		test::trace trace{std::string{"gauges/"} + train.name, gauges(), 100};
		vehicles[0].record(trace);
		test::drive_cycle(test::driving(vehicles), test::track{0.15, 0.0, test::catenary_3kv}, trace);
		trace.check();
	}
}
