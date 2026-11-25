/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// water and steam for the crank-angle model of the steam locomotive (McZapkie/steam/): IAPWS-IF97, regions 1, 2
// and 4, tabulated once on a grid of ln p (0.2..25 bar, 400 nodes) by superheat (0..330 K every 5 K) and read by
// linear and bilinear interpolation. the grid and the interpolation are those of the reference model the engine
// was worked out with, so both give the same figures
//
// units: Pa absolute, K, m³/kg, J/kg, J/(kg K), kg/s
namespace steam::detailed
{

inline constexpr double bar{1.0e5}; // Pa
inline constexpr double technical_atmosphere{0.980665e5}; // Pa, the at (kG/cm²) of the drawings
inline constexpr double kelvin{273.15};
inline constexpr double gravity{9.80665}; // m/s²
inline constexpr double stefan_boltzmann{5.670374e-8}; // W/(m² K⁴)

// boiling water (liquid) and dry saturated steam (vapour) of one pressure
struct saturated
{
	double temperature;
	double liquid_volume;
	double vapour_volume;
	double liquid_energy; // internal
	double vapour_energy;
	double liquid_enthalpy;
	double vapour_enthalpy;
	double liquid_entropy;
	double vapour_entropy;
};

struct superheated
{
	double volume;
	double enthalpy;
	double entropy;
	double heat_capacity; // at constant pressure
	double energy; // internal
};

// a fluid in equilibrium: wet steam is a mixture of one pressure and temperature, of the given dryness
struct fluid
{
	double pressure;
	double temperature;
	double dryness; // 1 for superheated steam
	double enthalpy;
};

// the saturation line at the given pressure
saturated saturation(double Pressure);
double saturation_temperature(double Pressure);
double saturation_pressure(double Temperature);
// superheated steam of the given pressure and temperature; at or under saturation it is dry saturated steam
superheated superheated_steam(double Pressure, double Temperature);
// the equilibrium state of the given specific internal energy and specific volume. Guess is the pressure to start
// the search for superheated steam from, 0 for none
fluid state_from_energy_and_volume(double Energy, double Volume, double Guess = 0.0);

// the isentropic exponent: wet steam after Zeuner, superheated 1.30
double isentropic_exponent(double Dryness);

// an orifice between two pressures
struct orifice
{
	double area; // m²
	double discharge{1.0}; // the discharge coefficient
};
// the steam upstream of an orifice
struct upstream
{
	double pressure;
	double volume; // specific
	double dryness{1.0};
};
// the flow through the orifice down to the given pressure: isentropic (de Saint-Venant and Wantzel), choked under
// the critical pressure ratio; none against the pressure
double orifice_flow(orifice const &Orifice, upstream const &Upstream, double Downstream);

} // namespace steam::detailed
