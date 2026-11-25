/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "McZapkie/steam/properties.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <span>
#include <vector>

namespace steam::detailed
{

namespace
{

// IAPWS-IF97, the Revised Release of 2007: the exponents and coefficients of the Gibbs energy of region 1 (water,
// table 2) and of region 2 (steam: the ideal gas part, table 10, and the residual part, table 11)
constexpr std::array region1_i{
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 8, 8, 21, 23, 29, 30, 31, 32,
};
constexpr std::array region1_j{
    -2, -1, 0, 1, 2, 3, 4, 5, -9, -7, -1, 0, 1, 3, -3, 0, 1, 3, 17, -4, 0, 6, -5, -2, 10, -8, -11, -6, -29, -31, -38, -39, -40, -41,
};
constexpr std::array region1_n{
    1.4632971213167e-01,  -8.4548187169114e-01, -3.7563603672040e+00, 3.3855169168385e+00,  -9.5791963387872e-01, 1.5772038513228e-01,  -1.6616417199501e-02,
    8.1214629983568e-04,  2.8319080123804e-04,  -6.0706301565874e-04, -1.8990068218419e-02, -3.2529748770505e-02, -2.1841717175414e-02, -5.2838357969930e-05,
    -4.7184321073267e-04, -3.0001780793026e-04, 4.7661393906987e-05,  -4.4141845330846e-06, -7.2694996297594e-16, -3.1679644845054e-05, -2.8270797985312e-06,
    -8.5205128120103e-10, -2.2425281908000e-06, -6.5171222895601e-07, -1.4341729937924e-13, -4.0516996860117e-07, -1.2734301741641e-09, -1.7424871230634e-10,
    -6.8762131295531e-19, 1.4478307828521e-20,  2.6335781662795e-23,  -1.1947622640071e-23, 1.8228094581404e-24,  -9.3537087292458e-26,
};
constexpr std::array region2_ideal_j{
    0, 1, -5, -4, -3, -2, -1, 2, 3,
};
constexpr std::array region2_ideal_n{
    -9.6927686500217e+00, 1.0086655968018e+01, -5.6087911283020e-03, 7.1452738081455e-02, -4.0710498223928e-01, 1.4240819171444e+00, -4.3839511319450e+00, -2.8408632460772e-01, 2.1268463753307e-02,
};
constexpr std::array region2_i{
    1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 5, 6, 6, 6, 7, 7, 7, 8, 8, 9, 10, 10, 10, 16, 16, 18, 20, 20, 20, 21, 22, 23, 24, 24, 24,
};
constexpr std::array region2_j{
    0, 1, 2, 3, 6, 1, 2, 4, 7, 36, 0, 1, 3, 6, 35, 1, 2, 3, 7, 3, 16, 35, 0, 11, 25, 8, 36, 13, 4, 10, 14, 29, 50, 57, 20, 35, 48, 21, 53, 39, 26, 40, 58,
};
constexpr std::array region2_n{
    -1.7731742473213e-03, -1.7834862292358e-02, -4.5996013696365e-02, -5.7581259083432e-02, -5.0325278727930e-02, -3.3032641670203e-05, -1.8948987516315e-04, -3.9392777243355e-03,
    -4.3797295650573e-02, -2.6674547914087e-05, 2.0481737692309e-08,  4.3870667284435e-07,  -3.2277677238570e-05, -1.5033924542148e-03, -4.0668253562649e-02, -7.8847309559367e-10,
    1.2790717852285e-08,  4.8225372718507e-07,  2.2922076337661e-06,  -1.6714766451061e-11, -2.1171472321355e-03, -2.3895741934104e+01, -5.9059564324270e-18, -1.2621808899101e-06,
    -3.8946842435739e-02, 1.1256211360459e-11,  -8.2311340897998e+00, 1.9809712802088e-08,  1.0406965210174e-19,  -1.0234747095929e-13, -1.0018179379511e-09, -8.0882908646985e-11,
    1.0693031879409e-01,  -3.3662250574171e-01, 8.9185845355421e-25,  3.0629316876232e-13,  -4.2002467698208e-06, -5.9056029685639e-26, 3.7826947613457e-06,  -1.2768608934681e-15,
    7.3087610595061e-29,  5.5414715350778e-17,  -9.4369707241210e-07,
};

constexpr double gas_constant{461.526}; // J/(kg K)
constexpr double megapascal{1.0e6};

constexpr std::size_t pressure_nodes{400};
constexpr std::size_t superheat_nodes{67};
constexpr double pressure_min{0.2e5};
constexpr double pressure_max{25.0e5};
constexpr double superheat_step{5.0};
constexpr double superheat_max{superheat_step * (superheat_nodes - 1)};
constexpr double temperature_max{1073.0}; // of region 2

// x to the power of a whole exponent
double power(double const Base, int const Exponent)
{
	return std::pow(Base, static_cast<double>(Exponent));
}

// the powers 0..Count-1 of a number
template <std::size_t Count> std::array<double, Count> powers(double const Base)
{
	std::array<double, Count> result{};
	result[0] = 1.0;
	for (std::size_t index{1}; index < Count; ++index)
	{
		result[index] = result[index - 1] * Base;
	}
	return result;
}

// what the tables keep of a point
struct point
{
	double volume;
	double enthalpy;
	double entropy;
	double heat_capacity;
};

// region 4: the saturation temperature of the pressure, equation 31
double if97_saturation_temperature(double const Pressure)
{
	constexpr std::array n{0.0,
	                       0.11670521452767e+04,
	                       -0.72421316703206e+06,
	                       -0.17073846940092e+02,
	                       0.12020824702470e+05,
	                       -0.32325550322333e+07,
	                       0.14915108613530e+02,
	                       -0.48232657361591e+04,
	                       0.40511340542057e+06,
	                       -0.23855557567849e+00,
	                       0.65017534844798e+03};
	auto const beta{std::pow(Pressure / megapascal, 0.25)};
	auto const e{beta * beta + n[3] * beta + n[6]};
	auto const f{n[1] * beta * beta + n[4] * beta + n[7]};
	auto const g{n[2] * beta * beta + n[5] * beta + n[8]};
	auto const d{2.0 * g / (-f - std::sqrt(f * f - 4.0 * e * g))};
	return (n[10] + d - std::sqrt((n[10] + d) * (n[10] + d) - 4.0 * (n[9] + n[10] * d))) / 2.0;
}

// region 1, water: equation 7 and its derivatives
point if97_water(double const Temperature, double const Pressure)
{
	auto const tau{1386.0 / Temperature - 1.222};
	auto const pi{7.1 - Pressure / 16.53e6};
	auto g{0.0};
	auto gp{0.0};
	auto gt{0.0};
	auto gtt{0.0};
	for (std::size_t term{0}; term < region1_n.size(); ++term)
	{
		auto const i{region1_i[term]};
		auto const j{region1_j[term]};
		auto const n{region1_n[term]};
		g += n * power(pi, i) * power(tau, j);
		gp -= n * i * power(pi, i - 1) * power(tau, j);
		gt += n * j * power(pi, i) * power(tau, j - 1);
		gtt += n * j * (j - 1) * power(pi, i) * power(tau, j - 2);
	}
	auto const reducedtemperature{1386.0 / Temperature};
	return {gp * gas_constant * Temperature / 16.53e6, reducedtemperature * gt * gas_constant * Temperature, gas_constant * (reducedtemperature * gt - g),
	        -gas_constant * reducedtemperature * reducedtemperature * gtt};
}

// the Gibbs energy of region 2 and the derivatives the tables need
struct gibbs
{
	double g{0.0};
	double gp{0.0};
	double gt{0.0};
	double gtt{0.0};
};

// the ideal gas part, equation 16
gibbs if97_steam_ideal(double const Tau, double const Pi)
{
	gibbs ideal{std::log(Pi), 1.0 / Pi, 0.0, 0.0};
	for (std::size_t term{0}; term < region2_ideal_n.size(); ++term)
	{
		auto const j{region2_ideal_j[term]};
		auto const n{region2_ideal_n[term]};
		ideal.g += n * power(Tau, j);
		ideal.gt += n * j * power(Tau, j - 1);
		ideal.gtt += n * j * (j - 1) * power(Tau, j - 2);
	}
	return ideal;
}

// the residual part, equation 17. the exponents are whole and not negative, so the powers come from tables; a
// power of an exponent under zero is only ever multiplied by zero
gibbs if97_steam_residual(double const Tau, double const Pi)
{
	auto const pis{powers<25>(Pi)};
	auto const taus{powers<59>(Tau - 0.5)};
	gibbs residual;
	for (std::size_t term{0}; term < region2_n.size(); ++term)
	{
		auto const i{static_cast<std::size_t>(region2_i[term])};
		auto const j{static_cast<std::size_t>(region2_j[term])};
		auto const n{region2_n[term]};
		auto const tau1{j >= 1 ? taus[j - 1] : 0.0};
		auto const tau2{j >= 2 ? taus[j - 2] : 0.0};
		auto const exponent{static_cast<double>(j)};
		residual.g += n * pis[i] * taus[j];
		residual.gp += n * static_cast<double>(i) * pis[i - 1] * taus[j];
		residual.gt += n * exponent * pis[i] * tau1;
		residual.gtt += n * exponent * (exponent - 1.0) * pis[i] * tau2;
	}
	return residual;
}

// region 2, steam: equation 15 and its derivatives
point if97_steam(double const Temperature, double const Pressure)
{
	auto const tau{540.0 / Temperature};
	auto const pi{Pressure / megapascal};
	auto const ideal{if97_steam_ideal(tau, pi)};
	auto const residual{if97_steam_residual(tau, pi)};
	auto const gt{ideal.gt + residual.gt};
	return {pi * (ideal.gp + residual.gp) * gas_constant * Temperature / Pressure, tau * gt * gas_constant * Temperature, gas_constant * (tau * gt - (ideal.g + residual.g)),
	        -gas_constant * tau * tau * (ideal.gtt + residual.gtt)};
}

using pressure_column = std::array<double, pressure_nodes>;

// the tables: the saturation line by the pressure, superheated steam by the pressure and the superheat
struct steam_tables
{
	pressure_column log_pressure{};
	pressure_column temperature{};
	pressure_column liquid_volume{};
	pressure_column vapour_volume{};
	pressure_column liquid_energy{};
	pressure_column vapour_energy{};
	pressure_column liquid_enthalpy{};
	pressure_column vapour_enthalpy{};
	pressure_column liquid_entropy{};
	pressure_column vapour_entropy{};
	pressure_column vapour_log_density{}; // rising with the pressure, to find the pressure of a given volume of saturated steam
	std::vector<double> volume;
	std::vector<double> enthalpy;
	std::vector<double> entropy;
	std::vector<double> heat_capacity;

	void fill_saturation(std::size_t Row);
	void fill_superheated(std::size_t Row);
};

void steam_tables::fill_saturation(std::size_t const Row)
{
	auto const pressure{std::exp(log_pressure[Row])};
	auto const boiling{if97_saturation_temperature(pressure)};
	auto const water{if97_water(boiling, pressure)};
	auto const vapour{if97_steam(boiling, pressure)};
	temperature[Row] = boiling;
	liquid_volume[Row] = water.volume;
	vapour_volume[Row] = vapour.volume;
	liquid_enthalpy[Row] = water.enthalpy;
	vapour_enthalpy[Row] = vapour.enthalpy;
	liquid_energy[Row] = water.enthalpy - pressure * water.volume;
	vapour_energy[Row] = vapour.enthalpy - pressure * vapour.volume;
	liquid_entropy[Row] = water.entropy;
	vapour_entropy[Row] = vapour.entropy;
	vapour_log_density[Row] = -std::log(vapour.volume);
}

void steam_tables::fill_superheated(std::size_t const Row)
{
	auto const pressure{std::exp(log_pressure[Row])};
	for (std::size_t column{0}; column < superheat_nodes; ++column)
	{
		auto const superheat{superheat_step * static_cast<double>(column)};
		auto const steam{if97_steam(std::min(temperature[Row] + superheat, temperature_max), pressure)};
		auto const cell{Row * superheat_nodes + column};
		volume[cell] = steam.volume;
		enthalpy[cell] = steam.enthalpy;
		entropy[cell] = steam.entropy;
		heat_capacity[cell] = steam.heat_capacity;
	}
}

steam_tables build_tables()
{
	steam_tables result;
	auto const cells{pressure_nodes * superheat_nodes};
	result.volume.resize(cells);
	result.enthalpy.resize(cells);
	result.entropy.resize(cells);
	result.heat_capacity.resize(cells);
	auto const first{std::log(pressure_min)};
	auto const last{std::log(pressure_max)};
	for (std::size_t row{0}; row < pressure_nodes; ++row)
	{
		result.log_pressure[row] = first + (last - first) * static_cast<double>(row) / static_cast<double>(pressure_nodes - 1);
		result.fill_saturation(row);
		result.fill_superheated(row);
	}
	return result;
}

steam_tables const &tables()
{
	static steam_tables const built{build_tables()};
	return built;
}

// where a value falls between the nodes of a table: the lower node and how far towards the next one, 0..1
struct node
{
	std::size_t index;
	double weight;
};

node pressure_node(double const Pressure)
{
	auto const &logs{tables().log_pressure};
	auto const position{(std::log(std::clamp(Pressure, pressure_min, pressure_max)) - logs.front()) / (logs.back() - logs.front()) * static_cast<double>(pressure_nodes - 1)};
	auto const index{std::min(static_cast<std::size_t>(position), pressure_nodes - 2)};
	return {index, position - static_cast<double>(index)};
}

node superheat_node(double const Superheat)
{
	auto const position{std::clamp(Superheat, 0.0, superheat_max) / superheat_step};
	auto const index{std::min(static_cast<std::size_t>(position), superheat_nodes - 2)};
	return {index, position - static_cast<double>(index)};
}

double along(pressure_column const &Column, node const &Node)
{
	return Column[Node.index] * (1.0 - Node.weight) + Column[Node.index + 1] * Node.weight;
}

double bilinear(std::vector<double> const &Table, node const &Row, node const &Column)
{
	auto const cell{Row.index * superheat_nodes + Column.index};
	return Table[cell] * (1.0 - Row.weight) * (1.0 - Column.weight) + Table[cell + superheat_nodes] * Row.weight * (1.0 - Column.weight) + Table[cell + 1] * (1.0 - Row.weight) * Column.weight +
	       Table[cell + superheat_nodes + 1] * Row.weight * Column.weight;
}

// y of x on a line through the points of rising Xs, held at its ends
double interpolate(std::span<double const> const Xs, std::span<double const> const Ys, double const X)
{
	if (X <= Xs.front())
	{
		return Ys.front();
	}
	if (X >= Xs.back())
	{
		return Ys.back();
	}
	auto const upper{static_cast<std::size_t>(std::distance(Xs.begin(), std::ranges::upper_bound(Xs, X)))};
	auto const weight{(X - Xs[upper - 1]) / (Xs[upper] - Xs[upper - 1])};
	return Ys[upper - 1] + weight * (Ys[upper] - Ys[upper - 1]);
}

saturated saturation_at(node const &Node)
{
	auto const &table{tables()};
	return {along(table.temperature, Node),     along(table.liquid_volume, Node),   along(table.vapour_volume, Node),  along(table.liquid_energy, Node), along(table.vapour_energy, Node),
	        along(table.liquid_enthalpy, Node), along(table.vapour_enthalpy, Node), along(table.liquid_entropy, Node), along(table.vapour_entropy, Node)};
}

// superheated steam of the given pressure and superheat
superheated superheated_by(double const Pressure, double const Superheat)
{
	auto const &table{tables()};
	auto const row{pressure_node(Pressure)};
	auto const column{superheat_node(Superheat)};
	auto const volume{bilinear(table.volume, row, column)};
	auto const enthalpy{bilinear(table.enthalpy, row, column)};
	return {volume, enthalpy, bilinear(table.entropy, row, column), bilinear(table.heat_capacity, row, column), enthalpy - Pressure * volume};
}

// how far superheated steam of the pressure and superheat is from the volume and energy sought
struct residuals
{
	double volume;
	double energy;
};

residuals superheated_residuals(double const Pressure, double const Superheat, double const Energy, double const Volume)
{
	auto const steam{superheated_by(Pressure, Superheat)};
	return {steam.volume / Volume - 1.0, (steam.energy - Energy) / 1.0e5};
}

// superheated steam of the given energy and volume: Newton's method on the logarithm of the pressure and the superheat
fluid superheated_from_energy_and_volume(double const Energy, double const Volume, double const Guess)
{
	constexpr int iterations{40};
	constexpr double pressure_probe{0.001};
	constexpr double superheat_probe{0.5};
	auto pressure{Guess > 0.0 ? Guess : std::clamp(461.5 * 500.0 / Volume, pressure_min, pressure_max)};
	auto superheat{50.0};
	auto converged{false};
	for (auto iteration{0}; iteration < iterations && !converged; ++iteration)
	{
		auto const here{superheated_residuals(pressure, superheat, Energy, Volume)};
		converged = std::abs(here.volume) < 1.0e-9 && std::abs(here.energy) < 1.0e-7;
		if (converged)
		{
			continue;
		}
		auto const bypressure{superheated_residuals(pressure * (1.0 + pressure_probe), superheat, Energy, Volume)};
		auto const bysuperheat{superheated_residuals(pressure, superheat + superheat_probe, Energy, Volume)};
		// the Jacobian, and the step solving it
		auto const a{(bypressure.volume - here.volume) / pressure_probe};
		auto const b{(bysuperheat.volume - here.volume) / superheat_probe};
		auto const c{(bypressure.energy - here.energy) / pressure_probe};
		auto const d{(bysuperheat.energy - here.energy) / superheat_probe};
		auto const determinant{a * d - b * c};
		auto const pressurestep{(here.volume * d - b * here.energy) / determinant};
		auto const superheatstep{(a * here.energy - here.volume * c) / determinant};
		pressure = std::clamp(pressure * std::exp(-std::clamp(pressurestep, -0.5, 0.5)), pressure_min, pressure_max);
		superheat = std::clamp(superheat - superheatstep, 0.0, superheat_max);
	}
	return {pressure, saturation_temperature(pressure) + superheat, 1.0, Energy + pressure * Volume};
}

// wet steam of the given volume at a pressure: its dryness, and how far its energy is from the one sought
struct mixture
{
	double excess;
	double dryness;
	double temperature;
};

mixture mixture_at(double const LogPressure, double const Energy, double const Volume)
{
	auto const line{saturation(std::exp(LogPressure))};
	auto const dryness{(Volume - line.liquid_volume) / (line.vapour_volume - line.liquid_volume)};
	return {line.liquid_energy + dryness * (line.vapour_energy - line.liquid_energy) - Energy, dryness, line.temperature};
}

// wet steam of the given energy and volume, found by bisection on the logarithm of the pressure, the energy of the
// mixture rising with it. dryness under 0 where there is none
fluid wet_from_energy_and_volume(double const Energy, double const Volume)
{
	constexpr int bisections{48};
	auto const &logs{tables().log_pressure};
	auto low{logs.front()};
	auto high{logs.back()};
	auto atlow{mixture_at(low, Energy, Volume).excess};
	if (atlow * mixture_at(high, Energy, Volume).excess >= 0.0)
	{
		return {0.0, 0.0, -1.0, 0.0};
	}
	for (auto bisection{0}; bisection < bisections; ++bisection)
	{
		auto const middle{0.5 * (low + high)};
		auto const excess{mixture_at(middle, Energy, Volume).excess};
		if ((excess > 0.0) == (atlow > 0.0))
		{
			low = middle;
			atlow = excess;
		}
		else
		{
			high = middle;
		}
	}
	auto const found{mixture_at(0.5 * (low + high), Energy, Volume)};
	if (found.dryness < 0.0 || found.dryness > 1.0)
	{
		return {0.0, 0.0, -1.0, 0.0};
	}
	auto const pressure{std::exp(0.5 * (low + high))};
	return {pressure, found.temperature, found.dryness, Energy + pressure * Volume};
}

} // namespace

saturated saturation(double const Pressure)
{
	return saturation_at(pressure_node(Pressure));
}

double saturation_temperature(double const Pressure)
{
	return along(tables().temperature, pressure_node(Pressure));
}

double saturation_pressure(double const Temperature)
{
	auto const &table{tables()};
	return std::exp(interpolate(table.temperature, table.log_pressure, Temperature));
}

superheated superheated_steam(double const Pressure, double const Temperature)
{
	return superheated_by(Pressure, Temperature - saturation_temperature(Pressure));
}

fluid state_from_energy_and_volume(double const Energy, double const Volume, double const Guess)
{
	auto const &table{tables()};
	// the energy of saturated steam of this very volume tells superheated steam from wet at once
	auto const saturatedlog{interpolate(table.vapour_log_density, table.log_pressure, -std::log(Volume))};
	auto const saturatedenergy{interpolate(table.log_pressure, table.vapour_energy, saturatedlog)};
	auto const withintables{Volume > table.vapour_volume.back() && Volume < table.vapour_volume.front()};
	if (Energy >= saturatedenergy && withintables)
	{
		return superheated_from_energy_and_volume(Energy, Volume, Guess > 0.0 ? Guess : std::exp(saturatedlog));
	}
	if (auto const wet{wet_from_energy_and_volume(Energy, Volume)}; wet.dryness >= 0.0)
	{
		return wet;
	}
	return superheated_from_energy_and_volume(Energy, Volume, Guess);
}

double isentropic_exponent(double const Dryness)
{
	return Dryness >= 1.0 ? 1.30 : 1.035 + 0.1 * Dryness;
}

double orifice_flow(orifice const &Orifice, upstream const &Upstream, double const Downstream)
{
	if (Orifice.area <= 0.0 || Upstream.pressure <= Downstream)
	{
		return 0.0;
	}
	auto const exponent{isentropic_exponent(Upstream.dryness)};
	auto const critical{std::pow(2.0 / (exponent + 1.0), exponent / (exponent - 1.0))};
	auto const ratio{std::max(Downstream / Upstream.pressure, critical)};
	auto const outflow{std::sqrt(std::max(2.0 * exponent / (exponent - 1.0) * (std::pow(ratio, 2.0 / exponent) - std::pow(ratio, (exponent + 1.0) / exponent)), 0.0))};
	return Orifice.discharge * Orifice.area * outflow * std::sqrt(Upstream.pressure / Upstream.volume);
}

} // namespace steam::detailed
