#include "stdafx.h"
#include "invariants.h"

#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <utility>

namespace test {

namespace {

std::string values(std::initializer_list<std::pair<char const *, double>> Values) {
	std::string result;
	for (auto const &[name, value] : Values) {
		char text[64];
		std::snprintf(text, sizeof(text), "%s%s=%.6g", result.empty() ? "" : " ", name, value);
		result += text;
	}
	return result;
}

bool electric(TMoverParameters const &M) {
	return M.EngineType == TEngineType::ElectricSeriesMotor || M.EngineType == TEngineType::ElectricInductionMotor;
}

double max_brake_press(TMoverParameters const &M) {
	auto result{0.0};
	for (auto const press : M.MaxBrakePress) {
		result = std::max(result, press);
	}
	return result;
}

// the margin over a limit which a step of the simulation may overshoot it by
constexpr double pressure_margin{0.05};
// the highest acceleration of a vehicle, 5 g, which a collision goes well over
constexpr double highest_acceleration{5.0 * 9.81};

} // namespace

std::vector<invariant> const &invariants() {
	static std::vector<invariant> const rules{
	    {"every value a number",
	     [](auto const &M, double, auto const *) {
		     for (auto const value : {M.V, M.AccS, M.Ft, M.Fb, M.Im, M.Itot, M.EngineVoltage, M.enrot, M.PipePress, M.BrakePress, M.LocBrakePress,
		                              M.ScndPipePress, M.Compressor, M.BatteryVoltage}) {
			     if (false == std::isfinite(value)) {
				     return false;
			     }
		     }
		     return true;
	     },
	     [](auto const &M) {
		     return values({{"V", M.V}, {"AccS", M.AccS}, {"Ft", M.Ft}, {"Fb", M.Fb}, {"Im", M.Im}, {"Itot", M.Itot}, {"EngineVoltage", M.EngineVoltage},
		                    {"enrot", M.enrot}, {"PipePress", M.PipePress}, {"BrakePress", M.BrakePress}, {"LocBrakePress", M.LocBrakePress},
		                    {"ScndPipePress", M.ScndPipePress}, {"Compressor", M.Compressor}, {"BatteryVoltage", M.BatteryVoltage}});
	     }},
	    {"brake cylinders between empty and the vehicle's highest pressure",
	     [](auto const &M, double, auto const *) {
		     // a step overshoots the setting a little: the EU07's LSt fills its cylinders to 4.0 bar of its 3.8
		     auto const limit{1.1 * max_brake_press(M) + pressure_margin};
		     return M.BrakePress >= -pressure_margin && M.BrakePress <= limit && M.LocBrakePress >= -pressure_margin && M.LocBrakePress <= limit;
	     },
	     [](auto const &M) { return values({{"BrakePress", M.BrakePress}, {"LocBrakePress", M.LocBrakePress}, {"max(MaxBrakePress)", max_brake_press(M)}}); }},
	    {"pipes between empty and the highest governor setting",
	     [](auto const &M, double, auto const *) {
		     // all the air comes from the main reservoirs, which the game sets up to 10 bar, some (the Impuls) to 12;
		     // the driver's valve in its filling position joins the brake pipe to them
		     auto const highest{1.01 * std::max(10.0, M.MaxCompressor) + pressure_margin};
		     for (auto const press : {M.PipePress, M.ScndPipePress, M.Compressor}) {
			     if (press < -pressure_margin || press > highest) {
				     return false;
			     }
		     }
		     return true;
	     },
	     [](auto const &M) { return values({{"PipePress", M.PipePress}, {"ScndPipePress", M.ScndPipePress}, {"Compressor", M.Compressor}}); }},
	    {"main reservoir no higher than the compressor's governor",
	     [](auto const &M, double, auto const *) { return M.MaxCompressor <= 0.0 || M.Compressor <= 1.01 * M.MaxCompressor + pressure_margin; },
	     [](auto const &M) { return values({{"Compressor", M.Compressor}, {"MaxCompressor", M.MaxCompressor}}); }},
	    {"no motor current with the line breaker open",
	     [](auto const &M, double, auto const *) { return M.Mains || false == electric(M) || std::abs(M.Im) < 1.0; },
	     [](auto const &M) { return values({{"Mains", M.Mains}, {"Im", M.Im}, {"Itot", M.Itot}}); }},
	    {"no motor current with the line contactors open, but for the dynamic brake",
	     [](auto const &M, double, auto const *) {
		     // the dynamic brake closes its own circuit of the motors and the resistors
		     return M.StLinFlag || M.DynamicBrakeFlag || M.EngineType != TEngineType::ElectricSeriesMotor || std::abs(M.Im) < 1.0;
	     },
	     [](auto const &M) {
		     return values({{"StLinFlag", M.StLinFlag}, {"DynamicBrakeFlag", M.DynamicBrakeFlag}, {"Im", M.Im}, {"MainCtrlActualPos", M.MainCtrlActualPos}});
	     }},
	    {"no tractive force without power",
	     [](auto const &M, double, auto const *) { return M.Power >= 0.01 || std::abs(M.Ft) < 1.0; },
	     [](auto const &M) { return values({{"Power", M.Power}, {"Ft", M.Ft}}); }},
	    {"engine fill between none and full",
	     [](auto const &M, double, auto const *) { return M.dizel_fill >= 0.0 && M.dizel_fill <= 1.0; },
	     [](auto const &M) { return values({{"dizel_fill", M.dizel_fill}}); }},
	    {"diesel engine turning forward, no faster than its top speed",
	     [](auto const &M, double, auto const *) {
		     // the governor lets it overshoot a little (the SN61 by 1%)
		     return M.EngineType != TEngineType::DieselEngine || (M.enrot >= 0.0 && M.enrot <= 1.05 * M.dizel_nmax);
	     },
	     [](auto const &M) { return values({{"enrot", M.enrot}, {"dizel_nmax", M.dizel_nmax}}); }},
	    {"battery no higher than its nominal voltage",
	     [](auto const &M, double, auto const *) { return M.NominalBatteryVoltage <= 0.0 || M.BatteryVoltage <= 1.05 * M.NominalBatteryVoltage; },
	     [](auto const &M) { return values({{"BatteryVoltage", M.BatteryVoltage}, {"NominalBatteryVoltage", M.NominalBatteryVoltage}}); }},
	    {"distance counted as the speed runs",
	     [](auto const &M, double Time, auto const *Before) {
		     if (Before == nullptr) {
			     return true;
		     }
		     // the speed of the step is between the speeds at its ends, or a bit off it, as the movement is integrated;
		     // through a standstill, down to none
		     auto const dt{Time - Before->time};
		     auto const covered{(M.DistCounter - Before->DistCounter) * 1000.0};
		     auto const lowest{M.V * Before->V > 0.0 ? std::min(std::abs(M.V), std::abs(Before->V)) : 0.0};
		     auto const highest{std::max(std::abs(M.V), std::abs(Before->V))};
		     return covered >= lowest * dt * 0.9 - 0.01 && covered <= highest * dt * 1.1 + 0.01;
	     },
	     [](auto const &M) { return values({{"V", M.V}, {"DistCounter", M.DistCounter}}); }},
	    {"no acceleration over 5 g",
	     [](auto const &M, double Time, auto const *Before) {
		     return Before == nullptr || std::abs(M.V - Before->V) <= highest_acceleration * (Time - Before->time) + 0.01;
	     },
	     [](auto const &M) { return values({{"V", M.V}, {"AccS", M.AccS}}); }},
	    {"hand brake applied, the wheels braked on the move",
	     [](auto const &M, double, auto const *) { return false == M.MBrake || M.ManualBrakePos == 0 || std::abs(M.V) < 0.5 || std::abs(M.Fb) > 0.0; },
	     [](auto const &M) { return values({{"ManualBrakePos", M.ManualBrakePos}, {"V", M.V}, {"Fb", M.Fb}}); }},
	};
	return rules;
}

namespace {

struct excuse {
	char const *trace; // a part of the trace's name
	char const *invariant;
	char const *reason;
};

excuse const excuses[]{
    // what the tests do
    {"couplers/collision/", "no acceleration over 5 g", "vehicles run into each other"},
    {"controls/collision/", "no acceleration over 5 g", "vehicles run into each other"},
    {"couplers/collision/", "distance counted as the speed runs", "the buffers throw the vehicle back within a step"},
    {"switches_live/", "no acceleration over 5 g", "sampled by hand, a second apart in the trace for 2 s run"},
    {"switches_live/", "distance counted as the speed runs", "sampled by hand, a second apart in the trace for 2 s run"},
    {"brake handles/", "main reservoir no higher than the compressor's governor", "the test holds the main reservoir at 8 bar"},
    {"local brake/", "main reservoir no higher than the compressor's governor", "the test holds the main reservoir at 8 bar"},
    {"brake cycle/at speed/", "brake cylinders between empty and the vehicle's highest pressure", "the rapid step of the valves at speed"},
    // the vehicles as they are
    {"elf/2", "main reservoir no higher than the compressor's governor",
     "the units' reservoirs share the supply pipe: the A car's governor stops at 9 bar, the C car's at 8.5"},
    {"sm03_v1/", "brake cylinders between empty and the vehicle's highest pressure",
     "its K valve fills the cylinders to the auxiliary reservoir's pressure (4.27 bar), over MaxBP 3.5"},
    {"eu04_v1/", "brake cylinders between empty and the vehicle's highest pressure",
     "its K valve fills the cylinders to the auxiliary reservoir's pressure (3.19 bar), over MaxBP 2.8"},
    {"sn61", "brake cylinders between empty and the vehicle's highest pressure",
     "its ESt3d_PZZ valve fills the cylinders to the auxiliary reservoir's pressure in an emergency (4.6 bar), over MaxBP 3.8"},
    {"sm03_v1/", "diesel engine turning forward, no faster than its top speed", "a lower gear at speed drives the engine over its top speed, by 10%"},
    {"wmb10_v1/", "diesel engine turning forward, no faster than its top speed", "a lower gear at speed drives the engine over its top speed, by 25%"},
    {"controls/full physics/", "no acceleration over 5 g", "the couplers' slack taken up as the train starts throws the wagons about at up to 5.7 g"},
    {"controls/full physics/", "distance counted as the speed runs", "the couplers' slack taken up as the train starts throws the wagons back within a step"},
};

} // namespace

bool excused(std::string const &Trace, std::string const &Invariant) {
	for (auto const &excuse : excuses) {
		if (excuse.invariant == Invariant && Trace.find(excuse.trace) != std::string::npos) {
			return true;
		}
	}
	return false;
}

} // namespace test
