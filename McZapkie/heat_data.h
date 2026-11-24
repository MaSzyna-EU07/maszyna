#pragma once

#include "core.h"
#include "fluid_circuit_t.h"

namespace mover
{
struct heat_data
{
	// input, state of relevant devices
	bool cooling{false}; // TODO: user controlled device, implement
	//    bool okienko { true }; // window in the engine compartment
	// system configuration
	bool auxiliary_water_circuit{false}; // cooling system has an extra water circuit
	double fan_speed{0.075}; // cooling fan rpm; either fraction of engine rpm, or absolute value if negative
	// heat exchange factors
	double kw{0.35};
	double kv{0.6};
	double kfe{1.0};
	double kfs{80.0};
	double kfo{25.0};
	double kfo2{25.0};
	// system parts

	fluid_circuit_t water;
	fluid_circuit_t water_aux;
	fluid_circuit_t oil;
	fluid_circuit_t engine;
	// output, state of affected devices
	bool PA{false}; // malfunction flag
	float rpmw{0.0}; // current main circuit fan revolutions
	float rpmwz{0.0}; // desired main circuit fan revolutions
	bool zaluzje1{false};
	float rpmw2{0.0}; // current auxiliary circuit fan revolutions
	float rpmwz2{0.0}; // desired auxiliary circuit fan revolutions
	bool zaluzje2{false};
	// output, temperatures
	float Te{15.0}; // ambient temperature TODO: get it from environment data
	// NOTE: by default the engine is initialized in warm, startup-ready state
	float Ts{50.0}; // engine temperature
	float To{45.0}; // oil temperature
	float Tsr{50.0}; // main circuit radiator temperature (?)
	float Twy{50.0}; // main circuit water temperature
	float Tsr2{40.0}; // secondary circuit radiator temperature (?)
	float Twy2{40.0}; // secondary circuit water temperature
	float temperatura1{50.0};
	float temperatura2{40.0};
	float powerfactor{1.0}; // coefficient of heat generation for engines other than su45
	// engine overheat threshold
	float engine_max_temp{-1}; // maximum acceptable engine temperature, triggers overheat lamp when exceeded
	bool engine_is_hot{false}; // engine temperature crossed cooling threshold
};

} // namespace mover
