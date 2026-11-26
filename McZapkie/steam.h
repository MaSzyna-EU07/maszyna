/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <string_view>

// the steam locomotive: its boiler, fire and cylinders, lumped into a few states updated every step of the
// physics. it stands apart from the vehicle (TMoverParameters drives it, see the steam_* methods there), so it
// can be tested on its own. the defaults are those of the PKP Ty2 (DRB class 52); a vehicle definition sets its
// own in the optional Steam: section, see load()
//
// units: pressures in Pa, absolute, unless named gauge; masses in kg, flows in kg/s, temperatures in °C
namespace steam {

inline constexpr double atmosphere{101'325.0}; // Pa
inline constexpr double air_pump_pressure{3.0e5}; // gauge Pa of the boiler, under which the air pump stops

// saturated water and steam, from the steam tables
namespace saturation {
// the pressure of steam over water of the given temperature
double pressure(double Temperature);
// the temperature of water boiling at the given pressure
double temperature(double Pressure);
// the heat turning a kg of water of the given temperature into steam, J/kg
double latent_heat(double Temperature);
// the specific heat of water of the given temperature, J/(kg K)
double specific_heat(double Temperature);
} // namespace saturation

struct parameters {
	// cylinders: two simple-expansion ones, cranked 90° apart
	int cylinders{2};
	double bore{0.600}; // m, CylD
	double stroke{0.660}; // m, CylS
	double clearance{0.09}; // the dead space, of the swept volume, CylClearance
	double cutoff_max{0.80}; // the longest cutoff of the valve gear, of the stroke, CutoffMax
	double cutoff_min{0.10}; // the shortest one, CutoffMin
	double cutoff_max_reverse{0.0}; // full gear running backwards, CutoffMaxReverse; 0: the same as forwards
	double diagram_factor{0.95}; // the indicator diagram's area against the ideal one (release, compression, throttling), DiagramFactor
	double mechanical_efficiency{0.85}; // from the cylinders to the rims, MechEff
	double wire_drawing{1.0 / 55.0}; // loss of admission pressure per sqrt(rpm), through the ports and passages, WireDrawing
	double back_pressure{0.04e5}; // Pa per (kg/s)² of exhaust, from the blast pipe, BackPressure
	double condensation{0.08}; // the steam lost to the cylinder walls at short cutoffs, of the steam admitted, Condensation
	double regulator_flow{7.0}; // kg/s through the regulator wide open, at working pressure and a chest pressure of half of it, RegulatorFlow
	double cocks_flow{0.05}; // kg/s per bar of admission pressure through the open cylinder cocks, CocksFlow
	double torque_pulsation{1.0}; // how much of the swing of the torque over a revolution reaches the wheels at low speed, 0..1, TorquePulsation
	double wheel_diameter{1.400}; // m, of the driving wheels, given by the Wheels: section
	double equaliser_pressure{0.5e5}; // gauge Pa of steam chest pressure under which the pressure equalisers open, EqualiserPressure

	// boiler
	double working_pressure{16.0e5}; // gauge Pa, where the safety valves open, BoilerPressure
	double safety_valve_flow{5.0}; // kg/s the safety valves pass wide open, 3% over working pressure, SafetyValveFlow
	double water_mass{9000.0}; // kg of water with the level in the middle of the glass, BoilerWater
	double glass_mass{1200.0}; // kg of water between the bottom and the top of the glass, GlassWater
	double crown_mass{700.0}; // kg of water under the bottom of the glass down to the firebox crown, CrownWater
	double swell{0.03}; // how much higher the level shows per kg/s of steam taken, of the glass, Swell
	double steel_heat{12.0e6}; // J/K of the boiler's steel heated with its water, BoilerSteelHeat
	double heat_loss{500.0}; // W/K lost by the boiler to its surroundings, BoilerHeatLoss
	double superheat_max{160.0}; // K the superheater adds over saturation at the nominal steam rate, Superheat
	double steam_rate{2.9}; // kg/s, the nominal steam rate of the boiler, SteamRate

	// fire
	double grate_area{3.89}; // m², Grate
	double fire_mass{600.0}; // kg of the right fire on the grate, FireMass
	double firing_rate_max{550.0}; // kg/(m² h) of coal burned with the strongest draught, FiringRateMax
	double coal_heat{29.0e6}; // J/kg, CoalHeat
	double efficiency_max{0.80}; // of the heat of the coal reaching the water, firing gently, BoilerEfficiency
	double efficiency_min{0.50}; // and firing at the highest rate
	// m² of the boiler's evaporative heating surface (the firebox and the tubes and flues, on the side of the gases),
	// HeatingSurface; optional. given, the efficiency falls with the coal burned per m² of it, reaching efficiency_min at
	// the rate of the Ty2 at its fullest firing, so a boiler of more surface to its grate keeps more of the heat; without it,
	// with the firing rate of the grate alone
	double heating_surface{0.0};
	double fire_delay{25.0}; // s from burning the coal to heating the water, FireDelay
	double exhaust_draught{1.0}; // the draught of the nominal exhaust, of the full one, ExhaustDraught
	double blower_draught{0.35}; // the draught of the blower fully open, BlowerDraught
	double blower_flow{0.25}; // kg/s of steam the blower takes fully open, BlowerFlow
	double natural_draught{0.04}; // the draught of a standing locomotive, the blower and dampers aside

	// feed
	std::array<double, 2> injector_flow{4.17, 2.9}; // kg/s of water each injector delivers wide open, InjectorFlow1, InjectorFlow2
	double feed_temperature{10.0}; // °C of the water in the tender
	double tender_water{30000.0}; // kg, TenderWater
	double tender_coal{10000.0}; // kg, TenderCoal
	double air_pump_flow{0.1}; // kg/s of steam the air pump takes running, AirPumpFlow

	double swept_volume() const; // m³ swept by all the pistons in a revolution of the wheels
};

// what the crew does with the controls of the engine
struct controls {
	double regulator{0.0}; // 0..1
	double cutoff{0.0}; // of the stroke, 0 in mid gear
	double blower{0.0}; // 0..1
	bool cylinder_cocks{false};
	// the fireman: shovelling coal and working the injectors himself, or leaving it to the automatic one
	bool automatic_fireman{true};
	double firing{0.0}; // kg/s of coal shovelled, by hand
	std::array<double, 2> injectors{0.0, 0.0}; // 0..1 open, by hand
	double auxiliary_flow{0.0}; // kg/s of steam taken by the air pump, the generator, train heating
};

struct state {
	// the states
	double water_temperature{20.0}; // °C
	double water_mass{9000.0}; // kg
	double fire_mass{0.0}; // kg
	double fire_heat{0.0}; // W, of the fire, reaching the water
	double superheat{0.0}; // K over saturation
	double exhaust_flow{0.0}; // kg/s, out of the blast pipe
	double tender_water{30000.0}; // kg
	double tender_coal{10000.0}; // kg
	bool safety_valves_open{false};
	bool fusible_plug_melted{false};
	// what the last update gave, for the gauges, the sounds and the smoke
	double boiler_pressure{atmosphere}; // Pa
	double chest_pressure{atmosphere}; // Pa
	double cylinder_flow{0.0}; // kg/s, into the cylinders
	double safety_valve_flow{0.0}; // kg/s
	double burn_rate{0.0}; // kg/s of coal
	double draught{0.0}; // of the full one
	double priming{0.0}; // of the steam that is water carried over, 0..1
	double force{0.0}; // N at the rims, unsigned
	double power{0.0}; // W indicated
	std::array<double, 2> injectors{0.0, 0.0}; // 0..1, as worked
	double blower{0.0}; // 0..1, as worked

	// the gauge pressure of the boiler
	double pressure() const { return boiler_pressure - atmosphere; }
	// the level in the glass: 0 at the bottom, 1 at the top, under 0 out of sight
	double water_level(parameters const &Parameters) const;
};

} // namespace steam

// the steam engine of a locomotive: its boiler, fire and cylinders, and the controls the crew works them with. the
// vehicle sets the controls and turns the driving wheels, the engine gives the force at their rims
class SteamEngine {
  public:
	steam::parameters Parameters;
	steam::controls Controls;
	steam::state State;
	double CrankAngle{0.0}; // rad, of the cranks of the driving wheels

	// reads the keys of the Steam: section of a vehicle definition over the defaults
	void load(std::string_view Line);
	// in steam, ready to go: the boiler a little under its working pressure, the fire made up, the glass half full
	void raise_steam();
	// cold, the boiler filled and the fire not lit
	void cool_down(double Temperature);
	// advances the engine by the time step at the revolutions of the driving wheels per second (unsigned): the cranks
	// turn with them, and the force at the rims, in State.force, pushes the way the valve gear is set
	void update(double Revolutions, double Deltatime);
	// the force at the rims of the given cutoff and admission pressure, at a standstill and not limited by the boiler, N
	double starting_force(double Cutoff, double Pressure) const;
	// the gauge pressure of the boiler, Pa
	double pressure() const
	{
		return State.pressure();
	}
	// the level in the glass: 0 at the bottom, 1 at the top, under 0 out of sight
	double water_level() const
	{
		return State.water_level(Parameters);
	}
	// whether its fire or its boiler is alive, so that it needs its physics with no one in the cab
	bool in_steam() const
	{
		return State.fire_mass > 0.0 || State.pressure() > 0.0;
	}
};
