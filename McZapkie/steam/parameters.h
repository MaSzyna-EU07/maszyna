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

// the figures of a steam locomotive for the crank-angle model (McZapkie/steam/). the defaults are those of the PKP
// Pt31 (1'D1' h2), read from its works drawings (Arch DT 84, the digital archive of Stacja Muzeum); the drawing a
// figure comes from is named beside it. a figure marked "assumed" is in no drawing read so far and is an estimate
// or a typical value, one marked "literature" comes from Pokropiński, Parowozy normalnotorowe produkcji polskiej
// (2007), p. 254
//
// units: m, m², m³, kg, s, N, J, W; pressures in Pa, gauge where named so; temperatures in K
namespace steam::detailed
{

struct boiler_parameters
{
	double working_pressure{14.71e5}; // gauge, 15 at, drawings 19.01 and 4.20
	double volume{14.0}; // of water and steam together, assumed from the barrel of drawings 2.07 and 1.04
	double water_volume_normal{9.3}; // assumed, the level some 0.16 m over the firebox crown
	double water_volume_min{8.5}; // assumed, the lowest level allowed
	double water_volume_max{10.6}; // assumed, the top of the glass
	double water_surface{15.0}; // m², assumed
	double steel_mass{28000.0}; // heated with the water, assumed
	double steel_heat{480.0}; // J/(kg K)
	double grate_area{4.51}; // literature
	double firebox_surface{17.3}; // m², from the drawing of the firebox (volume IV, no number) and 2.12
	double tube_length{6.2}; // between the tube plates, drawings 2.14, 2.16 and 6.02
	int tubes{112}; // drawing 2.14
	double tube_bore{0.055};
	int flues{38}; // drawing 2.16
	double flue_bore{0.135};
	int safety_valves{2}; // drawing 3.57
	double safety_valve_seat{0.0888}; // m, drawing 4.20
	double safety_valve_spring{651000.0}; // N/m, 930 kG in 14 mm, drawing 4.20
	double safety_valve_blowdown{0.5e5}; // assumed, a pop valve
	double lagging_transmittance{2.2}; // W/(m² K), assumed
	double lagging_surface{62.0}; // m², assumed
};

struct superheater_parameters
{
	int elements{38}; // Schmidt, in the flues, drawing 6.02
	double pipe_outer{0.036};
	double pipe_bore{0.029};
	std::array<double, 4> passes{5.05, 4.85, 4.58, 4.38}; // how far each of the four runs of an element reaches from the smokebox tube plate
	double steam_section{0.0251}; // m², one run an element
	double steam_path{21.3}; // m an element
	double pipe_mass{2260.0};
	double pipe_heat{480.0}; // J/(kg K)
	double steam_volume{0.9}; // the pipes, the header and the steam pipes, assumed
};

struct regulator_parameters
{
	double main_valve{0.205}; // m, a poppet valve in the dome, drawing 3.43
	double lift{0.077};
	double pilot_valve{0.060}; // assumed
	double pilot_travel{0.15}; // of the handle, opening the pilot valve alone
	double steam_pipe{0.180}; // drawing 3.43
	double steam_chest_volume{0.085}; // a cylinder, with its steam pipe, assumed
};

struct draught_parameters
{
	double nozzle_area{0.01685}; // the blast pipe ø155 less its bridge, drawing 5.15
	double chimney_throat{0.500}; // drawing 5.20
	double chimney_top{0.595}; // drawing 5.06
	double exhaust_volume{0.16}; // the exhaust passages and the breeches pipe, assumed
	double grate_resistance{20.0}; // of the clean grate, set for a smokebox vacuum of 1.5 to 2.5 kPa
	double bed_resistance{4000.0}; // more of it per m of fuel and clinker
	double ashpan_resistance{1.6};
	double damper_area{0.75}; // all the ashpan dampers open, assumed from drawing 7.01
	double gas_inertia{0.3}; // s, the gases of the boiler following the draught
	double blower_flow{0.12}; // kg/s of steam, the blower wide open at working pressure, assumed
	double diffuser_efficiency{0.55}; // of the tapered chimney, assumed
};

// the coal as fired: the mass fractions of its elements, moisture and ash
struct fuel_parameters
{
	double heating_value{27.2e6}; // J/kg, the lower one, assumed
	double carbon{0.70};
	double hydrogen{0.045};
	double oxygen{0.09};
	double sulphur{0.008};
	double ash{0.10};
	double bulk_density{800.0}; // kg/m³
	double flame_emissivity{0.6};
	double secondary_air{0.12}; // of the air through the grate, past the door and the leaks
	double unburnt_loss{0.00042}; // of the coal, per kg/(m² h) of firing rate (Lawford Fry)
};

struct cylinder_parameters
{
	double bore{0.630}; // drawing 21.04
	double stroke{0.700};
	double crank{0.350};
	double connecting_rod{3.800};
	double front_rod{0.070}; // the tail rod, drawing 19.28
	double back_rod{0.105}; // the piston rod
	double clearance{0.08}; // of the swept volume, assumed from drawings 19.01 and L20.01
	double crank_angle{90.0}; // degrees the left crank is behind the right one
	double reciprocating_mass{578.0}; // a side: the piston and rods of drawing L20.01, a third of the connecting rod of 20.10, the crosshead assumed
	double relief_pressure{16.2e5}; // gauge, of the cylinder relief valves, assumed
	double drain_cock{0.022}; // m, assumed
	double wall_mass{950.0}; // of cast iron taking part, a cylinder, assumed
	double wall_heat{540.0}; // J/(kg K)
};

// Heusinger (Walschaerts) valve gear with inside-admission piston valves, drawing 21.04. the origin is in the
// driving axle, x forwards, y up
struct valve_gear_parameters
{
	double return_crank{0.180};
	double return_crank_angle{96.4594}; // degrees behind the crank, 96°27'34"
	double eccentric_rod{2.55252};
	double link_x{2.550};
	double link_y{0.736};
	double link_foot{0.450}; // the radius of the pin the eccentric rod drives
	double link_foot_setback{0.020};
	double link_radius{2.000};
	double radius_rod{2.000};
	double radius_rod_tail{0.250}; // behind the die block, to the lifting link
	double combination_lever_upper{0.124};
	double combination_lever_lower{0.840};
	double valve_height{0.620}; // of the valve spindle over the cylinder
	double valve_centre{4.550}; // x of the valve spindle pin with the valve central
	double union_link{0.75052};
	double crosshead_arm{0.192}; // of the union link's pin under the cylinder
	double weighshaft_x{1.820};
	double weighshaft_y{1.230};
	double lifting_arm{0.500};
	double lifting_link{0.500};
	double reach_rod_arm{0.570};
	double screw_forward{0.293}; // the travel of the reverser's nut to full forward gear
	double screw_reverse{0.247};
	double steam_lap{0.040}; // locomotives 1 to 3; from number 4 on: 0.041, 0.006 and a port 0.720 long
	double exhaust_lap{0.002};
	double port_width{0.052};
	double port_length{0.700}; // round the valve
	double port_discharge{0.7}; // assumed
};

struct auxiliary_parameters
{
	std::array<double, 2> injector_flow{4.2, 3.9}; // kg/s of water: the live steam injector, the exhaust steam one (Metcalfe); assumed
	double air_pump_flow{0.05}; // kg/s of steam, running
	double generator_flow{0.02};
	double tender_water{32000.0}; // kg, tender 32D29, literature
	double tender_coal{10000.0};
	double feed_temperature{12.0 + 273.15};
	double empty_mass{96600.0}; // of the locomotive with no water, fuel, sand or crew, assumed
	double crew_mass{400.0}; // with their tools
	double tender_empty_mass{26500.0}; // literature
	double sand_capacity{300.0}; // kg, assumed
	double sanding_rate{0.03}; // kg/s
};

struct ambient_parameters
{
	double pressure{1.01325e5};
	double temperature{15.0 + 273.15};
};

struct parameters
{
	boiler_parameters boiler;
	superheater_parameters superheater;
	regulator_parameters regulator;
	draught_parameters draught;
	fuel_parameters fuel;
	cylinder_parameters cylinders;
	valve_gear_parameters valve_gear;
	auxiliary_parameters auxiliaries;
	ambient_parameters ambient;
	double wheel_diameter{1.850}; // of the coupled wheels, drawings 21.04 and 11.01
};

} // namespace steam::detailed
