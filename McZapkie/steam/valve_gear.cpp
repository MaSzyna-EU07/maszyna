/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "McZapkie/steam/valve_gear.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <numeric>
#include <utility>
#include <vector>

namespace steam::detailed
{

namespace
{

constexpr double millimetres{1000.0};
constexpr double turn{2.0 * std::numbers::pi};

struct vec2
{
	double x;
	double y;
};

vec2 operator+(vec2 const &Left, vec2 const &Right)
{
	return {Left.x + Right.x, Left.y + Right.y};
}

vec2 operator-(vec2 const &Left, vec2 const &Right)
{
	return {Left.x - Right.x, Left.y - Right.y};
}

vec2 operator*(vec2 const &Vector, double const Scale)
{
	return {Vector.x * Scale, Vector.y * Scale};
}

double dot(vec2 const &Left, vec2 const &Right)
{
	return Left.x * Right.x + Left.y * Right.y;
}

vec2 rotated(vec2 const &Vector, double const Angle)
{
	auto const cosine{std::cos(Angle)};
	auto const sine{std::sin(Angle)};
	return {cosine * Vector.x - sine * Vector.y, sine * Vector.x + cosine * Vector.y};
}

double radians(double const Degrees)
{
	return Degrees * std::numbers::pi / 180.0;
}

using vector3 = std::array<double, 3>;
using matrix3 = std::array<vector3, 3>;

// the row of the largest element of the column, from the diagonal down
std::size_t pivot_row(matrix3 const &Matrix, std::size_t const Column)
{
	auto best{Column};
	for (auto row{Column + 1}; row < 3; ++row)
	{
		if (std::abs(Matrix[row][Column]) > std::abs(Matrix[best][Column]))
		{
			best = row;
		}
	}
	return best;
}

// takes the pivot row of the column out of the rows under it
void eliminate(matrix3 &Matrix, vector3 &Right, std::size_t const Column)
{
	for (auto row{Column + 1}; row < 3; ++row)
	{
		auto const factor{Matrix[row][Column] / Matrix[Column][Column]};
		for (auto element{Column}; element < 3; ++element)
		{
			Matrix[row][element] -= factor * Matrix[Column][element];
		}
		Right[row] -= factor * Right[Column];
	}
}

// solves Matrix * x = Right by Gaussian elimination with partial pivoting
vector3 solve3(matrix3 Matrix, vector3 Right)
{
	for (std::size_t column{0}; column < 3; ++column)
	{
		auto const pivot{pivot_row(Matrix, column)};
		std::swap(Matrix[column], Matrix[pivot]);
		std::swap(Right[column], Right[pivot]);
		eliminate(Matrix, Right, column);
	}
	vector3 result{};
	for (std::size_t back{0}; back < 3; ++back)
	{
		auto const row{2 - back};
		auto sum{Right[row]};
		for (auto element{row + 1}; element < 3; ++element)
		{
			sum -= Matrix[row][element] * result[element];
		}
		result[row] = sum / Matrix[row][row];
	}
	return result;
}

// one of the two curves of a valve event against the piston's way: where it crosses zero going up, and going down
struct crossings
{
	double rising{std::numeric_limits<double>::quiet_NaN()};
	double falling{std::numeric_limits<double>::quiet_NaN()};
};

// the first crossings of zero of the curve round the revolution, as the piston's way (per cent of the stroke)
// there, interpolated between the steps
crossings first_crossings(std::vector<double> const &Curve, std::vector<double> const &Way)
{
	crossings result;
	auto const count{Curve.size()};
	for (std::size_t step{0}; step < count; ++step)
	{
		auto const here{Curve[step]};
		auto const next{Curve[(step + 1) % count]};
		auto const up{here < 0.0 && next >= 0.0};
		auto const down{here >= 0.0 && next < 0.0};
		if (!up && !down)
		{
			continue;
		}
		auto const way{std::lerp(Way[step], Way[(step + 1) % count], here / (here - next))};
		if (up && std::isnan(result.rising))
		{
			result.rising = way;
		}
		if (down && std::isnan(result.falling))
		{
			result.falling = way;
		}
	}
	return result;
}

// a revolution of the gear, step by step
struct revolution
{
	std::vector<double> valve; // mm
	std::vector<double> die_block; // mm
	std::vector<double> way; // of the piston from the front, per cent of the stroke
};

revolution run_revolution(valve_gear &Gear, double const Weighshaft, int const Steps)
{
	revolution result;
	auto const count{static_cast<std::size_t>(Steps)};
	result.valve.resize(count);
	result.die_block.resize(count);
	result.way.resize(count);
	Gear.reset();
	// twice round, for the solution to settle on its continuous branch
	for (auto pass{0}; pass < 2; ++pass)
	{
		for (std::size_t step{0}; step < count; ++step)
		{
			auto const crank{turn * static_cast<double>(step) / static_cast<double>(count)};
			auto const position{Gear.solve(crank, Weighshaft)};
			result.valve[step] = position.valve * millimetres;
			result.die_block[step] = position.die_block * millimetres;
			result.way[step] = Gear.piston(crank) / Gear.stroke() * 100.0;
		}
	}
	return result;
}

double mean(std::vector<double> const &Values)
{
	return std::accumulate(Values.begin(), Values.end(), 0.0) / static_cast<double>(Values.size());
}

// the events of one end of the cylinder: the admission and exhaust openings of its port round the revolution
struct cylinder_end
{
	std::vector<double> admission;
	std::vector<double> exhaust;
	bool back;
};

void end_events(valve_events &Events, cylinder_end const &End, revolution const &Revolution, double const PortWidth)
{
	auto const index{End.back ? std::size_t{1} : std::size_t{0}};
	// the working stroke of the front end takes the piston from 0 to 100 per cent, that of the back end from 100 to 0
	auto const worked{[&End](double const Way) { return End.back ? 100.0 - Way : Way; }};
	auto const admission{first_crossings(End.admission, Revolution.way)};
	auto const exhaust{first_crossings(End.exhaust, Revolution.way)};
	Events.cutoff[index] = worked(admission.falling);
	Events.admission[index] = 100.0 - worked(admission.rising);
	Events.release[index] = worked(exhaust.rising);
	Events.compression[index] = 100.0 - worked(exhaust.falling);
	Events.admission_opening[index] = std::min(*std::ranges::max_element(End.admission), PortWidth) / millimetres;
	Events.exhaust_opening[index] = std::min(*std::ranges::max_element(End.exhaust), PortWidth) / millimetres;
}

} // namespace

valve_gear::valve_gear(valve_gear_parameters const &Gear, cylinder_parameters const &Cylinders)
    : m_geometry{Cylinders.crank * millimetres,
                 Cylinders.connecting_rod * millimetres,
                 Gear.return_crank * millimetres,
                 radians(Gear.return_crank_angle),
                 Gear.eccentric_rod * millimetres,
                 Gear.link_x * millimetres,
                 Gear.link_y * millimetres,
                 Gear.link_foot * millimetres,
                 Gear.link_foot_setback * millimetres,
                 Gear.link_radius * millimetres,
                 Gear.radius_rod * millimetres,
                 Gear.radius_rod_tail * millimetres,
                 Gear.combination_lever_upper * millimetres,
                 Gear.combination_lever_lower * millimetres,
                 Gear.valve_height * millimetres,
                 Gear.valve_centre * millimetres,
                 Gear.union_link * millimetres,
                 Gear.crosshead_arm * millimetres,
                 Gear.weighshaft_x * millimetres,
                 Gear.weighshaft_y * millimetres,
                 Gear.lifting_arm * millimetres,
                 Gear.lifting_link * millimetres,
                 Gear.steam_lap * millimetres,
                 Gear.exhaust_lap * millimetres,
                 Gear.port_width * millimetres}
{
}

double valve_gear::crosshead(double const Crank) const
{
	auto const &g{m_geometry};
	auto const offset{g.crank * std::sin(Crank)};
	return g.crank * std::cos(Crank) + std::sqrt(g.connecting_rod * g.connecting_rod - offset * offset);
}

double valve_gear::piston(double const Crank) const
{
	auto const &g{m_geometry};
	return (g.connecting_rod + g.crank - crosshead(Crank)) / millimetres;
}

double valve_gear::piston_rate(double const Crank) const
{
	auto const &g{m_geometry};
	auto const sine{std::sin(Crank)};
	auto const cosine{std::cos(Crank)};
	auto const offset{g.crank * sine};
	return (g.crank * sine + g.crank * g.crank * sine * cosine / std::sqrt(g.connecting_rod * g.connecting_rod - offset * offset)) / millimetres;
}

double valve_gear::stroke() const
{
	return 2.0 * m_geometry.crank / millimetres;
}

double valve_gear::steam_lap() const
{
	return m_geometry.steam_lap / millimetres;
}

double valve_gear::exhaust_lap() const
{
	return m_geometry.exhaust_lap / millimetres;
}

double valve_gear::port_width() const
{
	return m_geometry.port_width / millimetres;
}

// the angle of the link, its foot held by the eccentric rod to the return crank: Newton's method from the last angle
double valve_gear::link_angle(double const Crank)
{
	constexpr int iterations{30};
	auto const &g{m_geometry};
	auto const eccentric{Crank - g.return_crank_angle};
	vec2 const returncrank{g.return_crank * std::cos(eccentric), -g.return_crank * std::sin(eccentric)};
	vec2 const pivot{g.link_x, g.link_y};
	vec2 const foot{-g.link_foot_setback, -std::sqrt(g.link_foot * g.link_foot - g.link_foot_setback * g.link_foot_setback)};
	auto angle{m_link};
	auto step{1.0};
	for (auto iteration{0}; iteration < iterations && std::abs(step) >= 1.0e-13; ++iteration)
	{
		auto const rod{pivot + rotated(foot, angle) - returncrank};
		auto const swing{rotated({-foot.y, foot.x}, angle)};
		step = (dot(rod, rod) - g.eccentric_rod * g.eccentric_rod) / (2.0 * dot(rod, swing));
		angle -= step;
	}
	m_link = angle;
	return angle;
}

// how far the union link, the radius rod and the lifting link are from their lengths (squared), for the given
// place of the valve spindle pin, angle of the combination lever and place of the die block
valve_gear::unknowns valve_gear::residuals(unknowns const &Unknowns, double const Link, double const Crosshead, double const Weighshaft) const
{
	auto const &g{m_geometry};
	vec2 const along{std::sin(Unknowns[1]), std::cos(Unknowns[1])};
	vec2 const spindle{Unknowns[0], g.valve_height};
	auto const leverhead{spindle + along * g.lever_upper};
	auto const leverfoot{spindle - along * g.lever_lower};
	// the die block on the arc of the link, the arc's centre being the head of the lever with the valve central
	vec2 const centre{g.valve_centre - g.link_x, g.valve_height + g.lever_upper - g.link_y};
	auto const arc{std::asin(-centre.y / g.link_radius) + Unknowns[2] / g.link_radius};
	auto const block{vec2{g.link_x, g.link_y} + rotated(centre + vec2{-std::cos(arc), std::sin(arc)} * g.link_radius, Link)};
	auto const tail{block + (block - leverhead) * (g.radius_rod_tail / g.radius_rod)};
	vec2 const crossheadpin{Crosshead, -g.crosshead_arm};
	vec2 const lift{g.weighshaft_x + g.lifting_arm * std::cos(Weighshaft), g.weighshaft_y + g.lifting_arm * std::sin(Weighshaft)};
	auto const unionlink{leverfoot - crossheadpin};
	auto const radiusrod{leverhead - block};
	auto const liftinglink{tail - lift};
	return {dot(unionlink, unionlink) - g.union_link * g.union_link, dot(radiusrod, radiusrod) - g.radius_rod * g.radius_rod, dot(liftinglink, liftinglink) - g.lifting_link * g.lifting_link};
}

// the step of Newton's method, the Jacobian taken by central differences
valve_gear::unknowns valve_gear::newton_step(unknowns const &Unknowns, double const Link, double const Crosshead, double const Weighshaft) const
{
	constexpr unknowns probes{1.0e-4, 1.0e-7, 1.0e-4};
	matrix3 jacobian{};
	for (std::size_t column{0}; column < 3; ++column)
	{
		auto ahead{Unknowns};
		auto behind{Unknowns};
		ahead[column] += probes[column];
		behind[column] -= probes[column];
		auto const up{residuals(ahead, Link, Crosshead, Weighshaft)};
		auto const down{residuals(behind, Link, Crosshead, Weighshaft)};
		for (std::size_t row{0}; row < 3; ++row)
		{
			jacobian[row][column] = (up[row] - down[row]) / (2.0 * probes[column]);
		}
	}
	return solve3(jacobian, residuals(Unknowns, Link, Crosshead, Weighshaft));
}

valve_position valve_gear::solve(double const Crank, double const Weighshaft)
{
	constexpr int iterations{60};
	auto const &g{m_geometry};
	auto const link{link_angle(Crank)};
	auto const crossheadx{crosshead(Crank)};
	auto x{m_solved ? m_unknowns : unknowns{g.valve_centre, 0.0, 0.0}};
	auto converged{false};
	for (auto iteration{0}; iteration < iterations && !converged; ++iteration)
	{
		auto const step{newton_step(x, link, crossheadx, Weighshaft)};
		for (std::size_t index{0}; index < 3; ++index)
		{
			x[index] -= step[index];
		}
		converged = std::max({std::abs(step[0]), std::abs(step[1]) / 1.0e-3, std::abs(step[2])}) < 1.0e-9;
	}
	m_unknowns = x;
	m_solved = true;
	m_crank = Crank;
	m_weighshaft = Weighshaft;
	return {(x[0] - g.valve_centre) / millimetres, x[2] / millimetres, link, x[1], crossheadx / millimetres};
}

valve_position valve_gear::follow(double const Crank, double const Weighshaft)
{
	constexpr double near_crank{0.2}; // rad
	constexpr double near_weighshaft{0.02};
	constexpr double stride{std::numbers::pi / 180.0};
	if (m_solved && std::abs(std::remainder(Crank - m_crank, turn)) < near_crank && std::abs(Weighshaft - m_weighshaft) < near_weighshaft)
	{
		return solve(Crank, Weighshaft);
	}
	// from the crank pointing forwards once round and on to where it is, on the continuous branch of the solution
	reset();
	auto const target{turn + std::fmod(std::fmod(Crank, turn) + turn, turn)};
	auto const strides{static_cast<int>(std::ceil(target / stride))};
	for (auto step{0}; step < strides; ++step)
	{
		solve(target * static_cast<double>(step) / static_cast<double>(strides), Weighshaft);
	}
	auto const position{solve(target, Weighshaft)};
	m_crank = Crank;
	return position;
}

void valve_gear::reset()
{
	m_solved = false;
	m_link = 0.0;
}

port_openings valve_gear::openings(double const Valve) const
{
	auto const &g{m_geometry};
	auto const travel{Valve * millimetres};
	auto const open{[&g](double const Opening) { return std::clamp(Opening, 0.0, g.port_width) / millimetres; }};
	return {open(travel - g.steam_lap), open(-travel - g.exhaust_lap), open(-travel - g.steam_lap), open(travel - g.exhaust_lap)};
}

double mid_gear(valve_gear &Gear)
{
	constexpr int bisections{60};
	constexpr int steps{72};
	auto low{radians(-20.0)};
	auto high{radians(20.0)};
	for (auto bisection{0}; bisection < bisections; ++bisection)
	{
		auto const middle{0.5 * (low + high)};
		Gear.reset();
		auto sum{0.0};
		for (auto step{0}; step < steps; ++step)
		{
			sum += Gear.solve(turn * static_cast<double>(step) / static_cast<double>(steps), middle).die_block;
		}
		if (sum > 0.0)
		{
			high = middle;
		}
		else
		{
			low = middle;
		}
	}
	Gear.reset();
	return 0.5 * (low + high);
}

double weighshaft_angle(valve_gear_parameters const &Parameters, double const MidGear, double const Reverser)
{
	auto const travel{Reverser * (Reverser >= 0.0 ? Parameters.screw_forward : Parameters.screw_reverse)};
	return MidGear - std::asin(std::clamp(travel / Parameters.reach_rod_arm, -0.999, 0.999));
}

double reverser_position(valve_gear_parameters const &Parameters, double const MidGear, double const Weighshaft)
{
	auto const travel{std::sin(MidGear - Weighshaft) * Parameters.reach_rod_arm};
	return travel / (travel >= 0.0 ? Parameters.screw_forward : Parameters.screw_reverse);
}

valve_events valve_timing(valve_gear &Gear, double const Weighshaft, int const Steps)
{
	auto revolution{run_revolution(Gear, Weighshaft, Steps)};
	valve_events events{};
	events.forward = mean(revolution.die_block) < 0.0;
	if (!events.forward)
	{
		// running backwards the crank angle falls: the revolution is gone through the other way
		std::ranges::reverse(revolution.valve);
		std::ranges::reverse(revolution.die_block);
		std::ranges::reverse(revolution.way);
	}
	auto const [blocklow, blockhigh]{std::ranges::minmax(revolution.die_block)};
	auto const [valvelow, valvehigh]{std::ranges::minmax(revolution.valve)};
	events.die_block_mean = mean(revolution.die_block) / millimetres;
	events.die_block_max = std::max(std::abs(blocklow), std::abs(blockhigh)) / millimetres;
	events.die_block_slip = (blockhigh - blocklow) / millimetres;
	events.valve_travel_forward = valvehigh / millimetres;
	events.valve_travel_back = -valvelow / millimetres;
	auto const steamlap{Gear.steam_lap() * millimetres};
	auto const exhaustlap{Gear.exhaust_lap() * millimetres};
	cylinder_end front{{}, {}, false};
	cylinder_end back{{}, {}, true};
	for (auto const travel : revolution.valve)
	{
		front.admission.push_back(travel - steamlap);
		front.exhaust.push_back(-travel - exhaustlap);
		back.admission.push_back(-travel - steamlap);
		back.exhaust.push_back(travel - exhaustlap);
	}
	auto const portwidth{Gear.port_width() * millimetres};
	end_events(events, front, revolution, portwidth);
	end_events(events, back, revolution, portwidth);
	Gear.reset();
	return events;
}

double weighshaft_for_cutoff(valve_gear &Gear, double const Cutoff, bool const Forward)
{
	constexpr int bisections{40};
	constexpr int steps{720};
	auto const middlegear{mid_gear(Gear)};
	auto low{Forward ? middlegear - radians(40.0) : middlegear + radians(0.5)};
	auto high{Forward ? middlegear - radians(0.5) : middlegear + radians(40.0)};
	auto const excess{[&Gear, Cutoff](double const Weighshaft) { return valve_timing(Gear, Weighshaft, steps).cutoff[0] - Cutoff; }};
	auto atlow{excess(low)};
	for (auto bisection{0}; bisection < bisections; ++bisection)
	{
		auto const middle{0.5 * (low + high)};
		auto const atmiddle{excess(middle)};
		if ((atmiddle > 0.0) == (atlow > 0.0))
		{
			low = middle;
			atlow = atmiddle;
		}
		else
		{
			high = middle;
		}
	}
	return 0.5 * (low + high);
}

} // namespace steam::detailed
