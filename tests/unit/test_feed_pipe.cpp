#include "stdafx.h"
#include "doctest.h"

#include "McZapkie/TMoverParameters.h"
#include "McZapkie/hamulce.h"
#include "consist.h"
#include "vehicle.h"

// the air exchange through the main reservoir hoses between the vehicles of a train (TMoverParameters::UpdatePipeCouplers()).
// taken in one explicit step of 0.01 s, a hose carried about 1.5 times the air which evens out the pipes on its ends,
// and the vehicles' own main reservoirs fed the pipe in the same step: the pipes of the middle vehicles of a multiple
// unit, or of locomotives in multiple traction, kept jumping 0.8 bar to and fro between the steps, with the main
// reservoirs all at the same pressure

namespace {

using test::definition;

// an EN57 of Units units, driven from the front cab
void en57(test::consist &Train, int const Units) {
	for (int i = 0; i < Units; ++i) {
		Train.add(definition::fiz("en57_v1/5bgb"), i == 0 ? 1 : 0);
		Train.add(definition::fiz("en57_v1/6ba_840"));
		Train.add(definition::fiz("en57_v1/5bgb"));
	}
}

// Count SM40 shunters in multiple traction, driven from the first
void sm40(test::consist &Train, int const Count) {
	for (int i = 0; i < Count; ++i) {
		Train.add(definition::fiz("sm40_v1/sm40"), i == 0 ? 1 : 0);
	}
}

// a locomotive and Cars passenger cars, whose main reservoir pipe only the locomotive feeds
void passenger_train(test::consist &Train, int const Cars) {
	Train.add(definition::fiz("ep09_v2/104e"));
	for (int i = 0; i < Cars; ++i) {
		Train.add(definition::fiz("11xa_v2/93a"));
	}
	Train[0].hold_brake_pipe(5.0);
}

// the largest difference [bar] of a main reservoir pipe of the train from Pressure, over Seconds of steps of Dt
double off_pressure(test::consist &Train, double const Pressure, double const Seconds, double const Dt = 0.01) {
	auto off{0.0};
	auto const steps{static_cast<int>(std::lround(Seconds / Dt))};
	for (int step = 0; step < steps; ++step) {
		Train.step({}, Dt);
		for (std::size_t i = 0; i < Train.size(); ++i) {
			off = std::max(off, std::abs(Train[i]->ScndPipePress - Pressure));
		}
	}
	return off;
}

struct evening_out {
	double time{60.0}; // [s] until the main reservoirs of the train's ends are within 0.05 bar
	double zigzag{0.0}; // [bar] the largest step of a main reservoir pipe back and forth, between steps the other way
};

// the train standing, the main reservoir of its first vehicle Pressure [bar], those of the rest 1 bar below, stepping
// by Dt until they even out through the main reservoir pipe
template <typename Builder>
evening_out even_out(Builder Train, int const Count, double const Pressure, double const Dt) {
	test::consist train;
	Train(train, Count);
	for (std::size_t i = 0; i < train.size(); ++i) {
		auto &mover{train[i].mover()};
		REQUIRE(mover.VeselVolume > 0.0);
		auto const pressure{i == 0 ? Pressure : Pressure - 1.0};
		mover.CompressedVolume = pressure * mover.VeselVolume;
		mover.Compressor = pressure;
		mover.Pipe2->CreatePress(pressure);
		mover.Pipe2->Act();
		mover.ScndPipePress = pressure;
	}
	evening_out result;
	auto const &first{train[0].mover()};
	auto const &last{train[train.size() - 1].mover()};
	std::vector<double> previous(train.size());
	std::vector<double> change(train.size());
	for (std::size_t i = 0; i < train.size(); ++i) {
		previous[i] = train[i]->ScndPipePress;
	}
	auto time{0.0};
	while (time < 60.0) {
		train.step({}, Dt);
		time += Dt;
		for (std::size_t i = 0; i < train.size(); ++i) {
			auto const step{train[i]->ScndPipePress - previous[i]};
			if (step * change[i] < 0.0) {
				// turned back
				result.zigzag = std::max(result.zigzag, std::min(std::abs(step), std::abs(change[i])));
			}
			change[i] = step;
			previous[i] = train[i]->ScndPipePress;
		}
		if (std::abs(first.Compressor - last.Compressor) < 0.05) {
			result.time = time;
			break;
		}
	}
	return result;
}

// the time [s] until the main reservoir pipe of the train's last vehicle, emptied with all the others which have no
// main reservoir of their own, is within 0.2 bar of the first vehicle's, which holds its main reservoir at Pressure [bar]
template <typename Builder>
double charging_time(Builder Train, int const Count, double const Pressure, double const Dt) {
	test::consist train;
	Train(train, Count);
	train[0].hold_main_reservoir(Pressure);
	for (std::size_t i = 1; i < train.size(); ++i) {
		auto &mover{train[i].mover()};
		REQUIRE(mover.VeselVolume == 0.0);
		mover.Pipe2->CreatePress(0.0);
		mover.Pipe2->Act();
		mover.ScndPipePress = 0.0;
	}
	auto const &first{train[0].mover()};
	auto const &last{train[train.size() - 1].mover()};
	auto time{0.0};
	while ((last.ScndPipePress < 1.0 || std::abs(last.ScndPipePress - first.ScndPipePress) > 0.2) && time < 60.0) {
		train.step({}, Dt);
		time += Dt;
	}
	return time;
}

} // namespace

TEST_CASE("main reservoir pipe: the pipes of a three unit EN57 stand at the pressure of its main reservoirs") {
	test::consist train;
	en57(train, 3);
	for (std::size_t i = 0; i < train.size(); ++i) {
		train[i].hold_main_reservoir(7.0);
	}
	// the pipes filling up from the reservoirs
	train.run(60.0);
	CHECK(off_pressure(train, 7.0, 1.0) < 0.01);
	// the exchange through the hoses of the 21 m cars takes 2 sub-steps of a step of 0.01 s
	CHECK(train[1]->ScndPipeCouplersSubsteps(0.01) == 2);
}

TEST_CASE("main reservoir pipe: the pipes of locomotives in multiple traction stand at the pressure of their main reservoirs") {
	test::consist train;
	sm40(train, 3);
	for (std::size_t i = 0; i < train.size(); ++i) {
		train[i].hold_main_reservoir(7.5);
	}
	train.run(60.0);
	CHECK(off_pressure(train, 7.5, 1.0) < 0.01);
}

TEST_CASE("main reservoir pipe: the main reservoirs of a multiple unit even out as with a short step") {
	auto const converged{even_out(en57, 3, 7.0, 0.001)};
	auto const usual{even_out(en57, 3, 7.0, 0.01)};
	CAPTURE(converged.time);
	CAPTURE(usual.time);
	CAPTURE(usual.zigzag);
	CHECK(converged.time < 60.0);
	CHECK(usual.time == doctest::Approx(converged.time).epsilon(0.1));
	// the pipes go along with the reservoirs, without jumping to and fro between the steps
	CHECK(usual.zigzag < 0.05);
}

TEST_CASE("main reservoir pipe: a passenger train, whose exchange was stable in one step, fills its pipe as with a short step") {
	auto const converged{charging_time(passenger_train, 10, 9.0, 0.001)};
	auto const usual{charging_time(passenger_train, 10, 9.0, 0.01)};
	CAPTURE(converged);
	CAPTURE(usual);
	CHECK(converged < 60.0);
	CHECK(usual == doctest::Approx(converged).epsilon(0.05));
}
