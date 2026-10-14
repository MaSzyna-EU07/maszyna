/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_catalog.h"
#include "vehicle/graph/graph_body.h"
#include "McZapkie/MOVER.h"

#include <cmath>

// combustion engine drive: engine -> clutch and gearbox / hydraulic transmission / main generator.
//
// the mechanical connections keep the convention of the electric drives: torque forward, rotational speed back. the
// engine is a torque source like the electric motors: it reads the speed of its shaft from the back channel and gives the
// torque. the speed of the engine shaft is a state of the block connected to the engine (drive.gearbox, drive.hydro or
// drive.generator), the same way the speed of the wheels is a state of the vehicle body behind drive.axle: that block
// holds the inertia of the engine side (crankshaft, flywheel, generator rotor), integrates the engine speed with the engine
// torque and the torque of its coupling (slipping clutch, torque converter, generator load), and returns it to the engine.
// when the coupling is locked (clutch engaged without slip, lockup clutch of the converter) the engine follows the speed
// of the wheels, as in TMoverParameters::dizel_Momentum, so the chain is rigid and stays stable at any step.
// the engine shaft has to be connected to exactly one such block; its speed is in rad/s, positive, in the frame of the
// engine. the direction of the vehicle is set after the coupling (dir inputs of the gearbox and the hydraulic transmission)
namespace vgraph
{

namespace
{

double constexpr two_pi{2.0 * M_PI};
double constexpr rpm_to_rad{two_pi / 60.0};

TMoverParameters &vehicle(frame const &Frame)
{
	return *Frame.body->mover;
}

block_type make_type(std::string Id, std::string Name, std::string Description, block_origin Origin)
{
	block_type type;
	type.id = std::move(Id);
	type.category = "drive";
	type.name = std::move(Name);
	type.description = std::move(Description);
	type.origin = Origin;
	return type;
}

// direction of the drive in the frame of the vehicle: -1, 0 or 1 (as for the electric motors)
int absolute_direction(frame const &Frame, double const Dir)
{
	auto const dir{Dir > 0.5 ? 1 : Dir < -0.5 ? -1 : 0};
	auto const cab{Frame.body->cab};
	return dir * (cab != 0 ? cab : 1);
}

double sign(double const Value)
{
	return Value > 0.0 ? 1.0 : Value < 0.0 ? -1.0 : 0.0;
}

// clamp which accepts the bounds in any order (safe_clamp of the Mover)
double clamp_any(double const Value, double const Bound1, double const Bound2)
{
	return std::clamp(Value, std::min(Bound1, Bound2), std::max(Bound1, Bound2));
}

// linear interpolation in the rows of a table (first column the argument, sorted); the end rows outside of it
double interpolate(std::vector<std::vector<double>> const &Rows, std::size_t const Column, double const Argument)
{
	if (Rows.empty())
	{
		return 0.0;
	}
	if (Argument <= Rows.front()[0])
	{
		return Rows.front()[Column];
	}
	for (std::size_t idx = 1; idx < Rows.size(); ++idx)
	{
		auto const &upper{Rows[idx]};
		if (Argument <= upper[0])
		{
			auto const &lower{Rows[idx - 1]};
			auto const span{upper[0] - lower[0]};
			return span > 0.0 ? lower[Column] + (upper[Column] - lower[Column]) * (Argument - lower[0]) / span : upper[Column];
		}
	}
	return Rows.back()[Column];
}

// rows of a table parameter with at least the specified number of columns, sorted by the first one
std::vector<std::vector<double>> sorted_rows(std::vector<std::vector<double>> const &Rows, std::size_t const Columns)
{
	std::vector<std::vector<double>> result;
	for (auto const &row : Rows)
	{
		if (row.size() >= Columns)
		{
			result.push_back(row);
		}
	}
	std::stable_sort(std::begin(result), std::end(result), [](auto const &Left, auto const &Right) { return Left[0] < Right[0]; });
	return result;
}

// combustion engine with a speed governor and a starter (adapted from TMoverParameters::dizel_Momentum, dizel_fillcheck
// and dizel_Update)
class drive_diesel final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			start,
			stop,
			fuel,
			demand,
			shaft,
			running,
			starting,
			rpm,
			torque,
			fill,
			power,
			fuel_rate
		};
	};
	struct param
	{
		enum : std::size_t
		{
			idle_rpm,
			max_rpm,
			governor,
			droop,
			fill_rate,
			curve,
			curve_type,
			m_max,
			rpm_m_max,
			m_nmax,
			friction,
			starter_torque,
			start_time,
			fuel_consumption,
			main_engine
		};
	};

	explicit drive_diesel(block_setup const &Setup)
	    : block(Setup), m_idle(std::max(1.0, number(Setup, param::idle_rpm))), m_max(std::max(m_idle + 1.0, number(Setup, param::max_rpm))),
	      m_allspeed(choice(Setup, param::governor) == 0), m_band(std::max(1.0, std::max(0.001, number(Setup, param::droop)) * m_max)),
	      m_fillrate(std::max(0.0, number(Setup, param::fill_rate))), m_curve(sorted_rows(table(Setup, param::curve), 2)), m_curvepower(choice(Setup, param::curve_type) == 1),
	      m_mmax(number(Setup, param::m_max)), m_nmmax(number(Setup, param::rpm_m_max)), m_mnmax(number(Setup, param::m_nmax)), m_friction(std::max(0.0, number(Setup, param::friction))),
	      m_startertorque(std::max(0.0, number(Setup, param::starter_torque))), m_starttime(std::max(0.0, number(Setup, param::start_time))),
	      m_mainengine(boolean(Setup, param::main_engine))
	{
		if (m_curve.size() < 2)
		{
			m_curve.clear();
		}
		if (m_nmmax == m_max)
		{
			// as in LoadFIZ_DList: guards the division in the torque parabola
			m_nmmax = m_max - 1.0;
		}
		// fuel used per hour at the full power, as dizel_FuelConsumption of LoadFIZ_DList: g/kWh * kW / (850 g/l)
		auto const ratedpower{std::max(0.0, full_torque(m_max) - m_friction) * m_max * rpm_to_rad / 1000.0};
		m_fuelperhour = std::max(0.0, number(Setup, param::fuel_consumption)) * ratedpower / 850.0;
	}

	void update(frame &Frame) override
	{
		auto const omega{back_out(Frame, port::shaft)};
		auto const rpm{omega / rpm_to_rad};
		auto const start{get_bool(Frame, port::start)};
		auto const stop{get_bool(Frame, port::stop)};
		auto const fuel{get_bool(Frame, port::fuel)};
		auto const startpressed{start && false == m_startprevious};
		m_startprevious = start;

		// start procedure (dizel_Update): the starter cranks the engine for start_time, then the ignition; the starter
		// keeps helping until the engine reaches its idle speed (dizel_spinup)
		if (false == m_running && false == m_cranking && startpressed && fuel && false == stop)
		{
			m_cranking = true;
			m_cranktime = 0.0;
		}
		if (m_cranking)
		{
			if (stop || false == fuel)
			{
				m_cranking = false;
			}
			else
			{
				m_cranktime += Frame.dt;
				if (m_cranktime >= m_starttime)
				{
					m_cranking = false;
					m_running = true;
					m_spinup = true;
				}
			}
		}
		if (m_running && (stop || false == fuel))
		{
			// knocked out without fuel, as dizel_Update does when the fuel pump stops
			m_running = false;
		}
		m_spinup = m_spinup && m_running && rpm < 0.95 * m_idle;
		if (m_running && false == m_spinup && rpm < 0.25 * m_idle)
		{
			// stalled, e.g. by engaging the clutch at standstill
			m_running = false;
		}

		// fuel injection set by the governor (dizel_fillcheck), reached at fill_rate (dizel_Update: fillspeed)
		auto target{0.0};
		if (m_running)
		{
			auto const demand{std::clamp(get(Frame, port::demand), 0.0, 1.0)};
			if (m_allspeed)
			{
				// all speed governor: the demand sets the speed between the idle and the maximum one
				auto const setspeed{m_idle + demand * (m_max - m_idle)};
				target = std::clamp((setspeed - rpm) / m_band, 0.0, 1.0);
			}
			else
			{
				// the demand is the injection; the governor keeps the idle speed and cuts the fuel at the maximum one
				target = std::max(demand, std::clamp((m_idle - rpm) / m_band, 0.0, 1.0));
				target = std::min(target, std::clamp((m_max - rpm) / m_band, 0.0, 1.0));
			}
		}
		m_fill += (target - m_fill) * std::min(1.0, m_fillrate * Frame.dt);

		// torque (dizel_Momentum): the curve at full injection times the injection, less the motion resistance of
		// the engine (dizel_Mstand), which acts against the rotation and fades out at standstill
		m_combustion = rpm > 0.0 ? full_torque(rpm) * m_fill : 0.0;
		m_resistance = m_friction * std::clamp(rpm / (0.1 * m_idle), -1.0, 1.0);
		auto starter{0.0};
		if (m_cranking || m_spinup)
		{
			// starter, falling with the speed as in dizel_Momentum
			starter = m_startertorque * 0.3 / (0.3 + std::max(0.0, rpm / m_idle));
		}
		m_torque = m_combustion - m_resistance + starter;
		m_omega = omega;
		m_netpower = (m_combustion - m_resistance) * omega / 1000.0;
		m_fuelrate = m_running ? m_fuelperhour * std::max(0.0, rpm / m_max) * m_fill : 0.0;
		m_fuelused += m_fuelrate * Frame.dt / 3600.0;

		set_forward(Frame, port::shaft, m_torque);
		set_bool(Frame, port::running, m_running);
		set_bool(Frame, port::starting, m_cranking || m_spinup);
		set(Frame, port::rpm, rpm);
		set(Frame, port::torque, m_torque);
		set(Frame, port::fill, m_fill);
		set(Frame, port::power, m_netpower);
		set(Frame, port::fuel_rate, m_fuelrate);
	}

	// the fields of the vehicle body are written after all blocks were updated, so the engine speed replaces the
	// revolutions written by the traction motors of a diesel-electric vehicle (for the diesel vehicles enrot is the
	// engine speed: engine sounds, cab indicators)
	void late_update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		// for the diesel vehicles Mains is the running engine; with more engines (or line breakers) any of them
		body.Mains = Frame.body->mains.any(Frame.step, m_running);
		if (false == m_mainengine)
		{
			return;
		}
		body.enrot = m_omega / two_pi;
		body.dizel_fill = m_fill;
		body.Mm = m_combustion - m_resistance;
		body.dizel_Torque = m_torque;
		body.dizel_Power = m_netpower;
		// as in TractionForce: (dizel_Mstand + Mm) * enrot * 2pi / 1000
		body.EnginePower = m_combustion * m_omega / 1000.0;
		body.dizel_startup = m_cranking;
		body.dizel_spinup = m_spinup;
		body.dizel_FuelConsumptionActual = m_fuelrate;
		body.dizel_FuelConsumptedTotal = m_fuelused;
		// speeds the sounds and indicators scale with (EngineIdleRPM, EngineMaxRPM, EngineRPMRatio), rev/s
		body.dizel_nmin = m_idle / 60.0;
		body.dizel_nmax = m_max / 60.0;
		body.nmax = m_max / 60.0;
	}

	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "fuel_used";
	}
	double state_value(std::size_t) const override
	{
		return m_fuelused;
	}

	static block_type describe()
	{
		auto type{make_type("drive.diesel", "Diesel engine",
		                    "Combustion engine with a speed governor and a starter (dizel_Momentum). start begins the start procedure: the starter cranks the engine for "
		                    "start_time, then the fuel is injected. demand 0..1 sets the speed (all_speed governor) or the injection (min_max governor, idle and maximum "
		                    "speed kept). The torque is the curve at full injection (table rpm -> Nm or kW, or the Mover parabola m_max, rpm_m_max, m_nmax) times the "
		                    "injection, less the friction. The shaft has to go to one drive.gearbox, drive.hydro or drive.generator, which holds the engine speed.",
		                    block_origin::extracted)};
		type.ports = {signal_in("start", value_type::boolean),
		              signal_in("stop", value_type::boolean, 0.0, "", true),
		              signal_in("fuel", value_type::boolean, 1.0, "", true),
		              signal_in("demand", value_type::real),
		              physical_out("shaft", domain::mechanical),
		              signal_out("running", value_type::boolean),
		              signal_out("starting", value_type::boolean),
		              signal_out("rpm", value_type::real, "rpm"),
		              signal_out("torque", value_type::real, "Nm"),
		              signal_out("fill", value_type::real),
		              signal_out("power", value_type::real, "kW"),
		              signal_out("fuel_rate", value_type::real, "l/h")};
		type.params = {number_param("idle_rpm", 600.0, "rpm"),
		               number_param("max_rpm", 1500.0, "rpm"),
		               choice_param("governor", {"all_speed", "min_max"}, "all_speed"),
		               number_param("droop", 0.05),
		               number_param("fill_rate", 10.0, "1/s"),
		               table_param("curve", {"rpm", "torque"}),
		               choice_param("curve_type", {"torque", "power"}, "torque"),
		               number_param("m_max", 3000.0, "Nm"),
		               number_param("rpm_m_max", 1100.0, "rpm"),
		               number_param("m_nmax", 2600.0, "Nm"),
		               number_param("friction", 300.0, "Nm"),
		               number_param("starter_torque", 600.0, "Nm"),
		               number_param("start_time", 2.0, "s"),
		               number_param("fuel_consumption", 220.0, "g/kWh"),
		               boolean_param("main_engine", true)};
		type.late = true;
		type.create = make_block<drive_diesel>;
		return type;
	}

private:
	// torque at full injection, Nm
	double full_torque(double const Rpm) const
	{
		if (false == m_curve.empty())
		{
			auto const value{interpolate(m_curve, 1, Rpm)};
			// power curve in kW: torque = P / omega, the speed held at the first row to keep it finite
			return m_curvepower ? value * 1000.0 / (std::max(Rpm, std::max(1.0, m_curve.front()[0])) * rpm_to_rad) : value;
		}
		// parabola of dizel_Momentum: Mmax at nMmax, Mnmax at nmax
		auto const x{(Rpm - m_nmmax) / (m_nmmax - m_max)};
		return std::max(0.0, m_mmax - (m_mmax - m_mnmax) * x * x);
	}

	double const m_idle; // rpm
	double const m_max; // rpm
	bool const m_allspeed;
	double const m_band; // rpm
	double const m_fillrate;
	std::vector<std::vector<double>> m_curve;
	bool const m_curvepower;
	double const m_mmax;
	double m_nmmax;
	double const m_mnmax;
	double const m_friction;
	double const m_startertorque;
	double const m_starttime;
	bool const m_mainengine;
	double m_fuelperhour{0.0}; // l/h at full power and maximum speed
	bool m_startprevious{false};
	bool m_cranking{false};
	bool m_running{false};
	bool m_spinup{false};
	double m_cranktime{0.0};
	double m_fill{0.0};
	double m_combustion{0.0};
	double m_resistance{0.0};
	double m_torque{0.0};
	double m_omega{0.0};
	double m_netpower{0.0};
	double m_fuelrate{0.0};
	double m_fuelused{0.0};
};

// friction clutch and mechanical gearbox (adapted from TMoverParameters::dizel_Momentum without the converter,
// dizel_EngageChange and dizel_AutoGearCheck). holds the speed of the engine: slipping, the clutch passes its friction
// torque and the engine speed follows its own torque; synchronous, the engine follows the gearbox input and its whole
// torque passes on
class drive_gearbox final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			gear,
			clutch,
			dir,
			out,
			actual,
			engage,
			slip,
			shifting
		};
	};
	struct param
	{
		enum : std::size_t
		{
			gears,
			clutch_torque,
			engage_rate,
			release_rate,
			auto_clutch,
			inertia,
			eff
		};
	};

	explicit drive_gearbox(block_setup const &Setup)
	    : block(Setup), m_clutchtorque(std::max(0.0, number(Setup, param::clutch_torque))), m_engagerate(std::max(0.0, number(Setup, param::engage_rate))),
	      m_releaserate(std::max(0.0, number(Setup, param::release_rate))), m_autoclutch(boolean(Setup, param::auto_clutch)),
	      m_inertia(std::max(0.01, number(Setup, param::inertia))), m_efficiency(std::clamp(number(Setup, param::eff), 0.01, 1.0))
	{
		for (auto const &row : table(Setup, param::gears))
		{
			if (row.size() >= 2)
			{
				m_gears.emplace_back(static_cast<int>(std::lround(row[0])), row[1]);
			}
		}
	}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const moment{forward_in(Frame, port::in)};
		auto const requestedgear{static_cast<int>(std::lround(get(Frame, port::gear)))};
		auto const requesteddir{absolute_direction(Frame, get(Frame, port::dir))};

		// a change of the gear or the direction waits for the released clutch; with auto_clutch the gearbox releases it
		m_shifting = (requestedgear != m_gear || requesteddir != m_dir);
		auto const demand{std::clamp(get(Frame, port::clutch), 0.0, 1.0)};
		m_engagestate = (m_shifting && m_autoclutch) ? 0.0 : demand;
		// dizel_EngageChange: the pressure approaches the set one, and jumps to it when close
		if (std::abs(m_engage - m_engagestate) < 0.11)
		{
			m_engage = m_engagestate;
		}
		else
		{
			m_engage += (m_engage > m_engagestate ? m_releaserate : m_engagerate) * Frame.dt * (m_engagestate - m_engage);
		}
		if (m_shifting && m_engage < 0.1)
		{
			m_gear = requestedgear;
			m_dir = requesteddir;
			m_shifting = false;
		}

		// ratio from the engine to the output shaft, in the frame of the vehicle; 0 in neutral
		auto ratio{0.0};
		for (auto const &gear : m_gears)
		{
			if (gear.first == m_gear && m_gear != 0)
			{
				ratio = gear.second * m_dir;
				break;
			}
		}

		auto gearmoment{0.0};
		m_deltaomega = 0.0;
		m_slipping = false;
		if (ratio == 0.0)
		{
			// neutral: the input shaft turns with the engine, which runs free
			auto const newomega{m_omega + moment / m_inertia * Frame.dt};
			m_omega = (newomega < 0.0 && m_omega >= 0.0) ? 0.0 : newomega;
			m_previous = m_omega;
		}
		else
		{
			// dizel_Momentum, in rad/s and kg m2 (the .fiz AIM is the inertia times 2pi)
			auto const n{back_out(Frame, port::out) * ratio}; // speed of the gearbox input
			auto const torquec{m_clutchtorque * m_engage};
			if (std::abs(moment) > torquec || std::abs(m_previous - m_omega) > 0.1 * two_pi)
			{
				// slipping, because of the speed difference or the torque above the friction of the clutch
				m_slipping = true;
				m_deltaomega = m_omega - m_previous;
				gearmoment = torquec * sign(m_deltaomega);
				auto const eps{(moment - gearmoment) / m_inertia};
				auto newomega{m_omega + eps * Frame.dt};
				if ((newomega - n) * (m_omega - m_previous) < 0.0 && torquec > 0.1)
				{
					// the slip changed its sign: the clutch grips
					newomega = n;
				}
				if (newomega * m_omega <= 0.0 && eps * m_omega < 0.0)
				{
					// no passing through zero speed
					newomega = 0.0;
				}
				m_omega = newomega;
			}
			else
			{
				// synchronous: the engine follows the gearbox, within what the clutch can hold
				gearmoment = moment;
				m_omega = clamp_any(n, m_omega + (moment - torquec) / m_inertia * Frame.dt, m_omega + (moment + torquec) / m_inertia * Frame.dt);
			}
			m_previous = n;
		}

		set_forward(Frame, port::out, gearmoment * ratio * m_efficiency);
		add_back(Frame, port::in, m_omega);
		set(Frame, port::actual, m_gear);
		set(Frame, port::engage, m_engage);
		set_bool(Frame, port::slip, m_slipping && m_engage > 0.1 && std::abs(m_deltaomega) > 0.2 * two_pi);
		set_bool(Frame, port::shifting, m_shifting);
		// clutch state for the sounds and the cab indicators (rsEngageSlippery, clutch gauges)
		body.dizel_engage = m_engage;
		body.dizel_engagestate = m_engagestate;
		body.dizel_engagedeltaomega = m_deltaomega / two_pi;
		body.dizel_automaticgearstatus = m_shifting ? sign(requestedgear - m_gear) : 0.0;
	}

	static block_type describe()
	{
		auto type{make_type("drive.gearbox", "Gearbox with clutch",
		                    "Friction clutch and mechanical gearbox (dizel_Momentum, dizel_EngageChange). gear selects the row of the gears table (gear number, ratio; "
		                    "0 or a missing number is neutral, reverse gears can have negative ratios), dir reverses. A change of the gear or the direction waits until "
		                    "the clutch is released; with auto_clutch the gearbox releases it itself. Holds the speed of the engine (inertia of the engine side).",
		                    block_origin::extracted)};
		type.ports = {physical_in("in", domain::mechanical),
		              signal_in("gear", value_type::integer, 1.0, "", true),
		              signal_in("clutch", value_type::real, 1.0, "", true),
		              signal_in("dir", value_type::real, 1.0, "", true),
		              physical_out("out", domain::mechanical),
		              signal_out("actual", value_type::integer),
		              signal_out("engage", value_type::real),
		              signal_out("slip", value_type::boolean),
		              signal_out("shifting", value_type::boolean)};
		type.params = {table_param("gears", {"gear", "ratio"}),
		               number_param("clutch_torque", 1500.0, "Nm"),
		               number_param("engage_rate", 0.5, "1/s"),
		               number_param("release_rate", 0.9, "1/s"),
		               boolean_param("auto_clutch", true),
		               number_param("inertia", 10.0, "kg m2"),
		               number_param("eff", 0.95)};
		type.create = make_block<drive_gearbox>;
		return type;
	}

private:
	double const m_clutchtorque;
	double const m_engagerate;
	double const m_releaserate;
	bool const m_autoclutch;
	double const m_inertia;
	double const m_efficiency;
	std::vector<std::pair<int, double>> m_gears;
	int m_gear{0};
	int m_dir{0};
	bool m_shifting{false};
	bool m_slipping{false};
	double m_engage{0.0};
	double m_engagestate{0.0};
	double m_omega{0.0}; // engine speed, rad/s
	double m_previous{0.0}; // gearbox input speed in the previous step (dizel_n_old)
	double m_deltaomega{0.0};
};

// hydraulic transmission: torque converter with filling, fluid coupling range and lockup clutch (adapted from the hydro_TC
// part of TMoverParameters::dizel_Momentum). holds the speed of the engine like the gearbox
class drive_hydro final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			power,
			running,
			dir,
			out,
			fill,
			lockup,
			ratio
		};
	};
	struct param
	{
		enum : std::size_t
		{
			tc_tmmax,
			tc_cp,
			tc_table,
			tc_tii,
			tc_tio,
			tc_too,
			fill_rate,
			drain_rate,
			lockup_torque,
			lockup_speed,
			unlock_speed,
			idle_rpm,
			ans_speed,
			inertia
		};
	};

	explicit drive_hydro(block_setup const &Setup)
	    : block(Setup), m_tmmax(number(Setup, param::tc_tmmax)), m_couplingpoint(std::clamp(number(Setup, param::tc_cp), 0.01, 0.99)),
	      m_table(sorted_rows(table(Setup, param::tc_table), 2)), m_torqueinin(number(Setup, param::tc_tii)), m_torqueinout(number(Setup, param::tc_tio)),
	      m_torqueoutout(number(Setup, param::tc_too)), m_fillrate(std::max(0.0, number(Setup, param::fill_rate))), m_drainrate(std::max(0.0, number(Setup, param::drain_rate))),
	      m_lockuptorque(std::max(0.0, number(Setup, param::lockup_torque))), m_lockupspeed(number(Setup, param::lockup_speed)), m_unlockspeed(number(Setup, param::unlock_speed)),
	      m_idle(std::max(1.0, number(Setup, param::idle_rpm)) * rpm_to_rad), m_ansspeed(number(Setup, param::ans_speed)),
	      m_inertia(std::max(0.01, number(Setup, param::inertia)))
	{
		if (m_table.size() < 2)
		{
			m_table.clear();
		}
	}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const moment{forward_in(Frame, port::in)};
		auto const dir{absolute_direction(Frame, get(Frame, port::dir))};
		auto const power{get_bool(Frame, port::power)};
		// the engine running (Mains of dizel_Momentum); a stopped engine drains the converter and releases the lockup clutch
		auto const running{get_bool(Frame, port::running)};
		auto const vel{std::abs(Frame.body->velocity) * 3.6};
		// speed of the output shaft in the frame of the engine; without a direction it turns with the engine
		auto const n{dir != 0 ? back_out(Frame, port::out) * dir : m_omega};

		// filling and draining of the converter, engaging and releasing of the lockup clutch
		if (power && running && m_omega > 0.9 * m_idle)
		{
			m_fill += m_fillrate * Frame.dt;
		}
		if ((false == power && vel < m_ansspeed) || false == running || m_omega < 0.8 * m_idle)
		{
			m_fill -= m_drainrate * Frame.dt;
		}
		m_fill = std::clamp(m_fill, 0.0, 1.0);
		if (vel > m_lockupspeed && running && m_omega > 0.9 * m_idle && power)
		{
			m_lockup = true;
			m_lockuprate += m_fillrate * Frame.dt;
		}
		if (vel < (power ? m_lockupspeed : m_unlockspeed) || false == running || m_omega < 0.8 * m_idle)
		{
			m_lockup = false;
			m_lockuprate -= m_drainrate * Frame.dt;
		}
		m_lockuprate = std::clamp(m_lockuprate, 0.0, 1.0);

		// torques of the converter; its constants are for the speeds in rev/s, as in the .fiz files
		auto const nin{m_omega / two_pi};
		auto const nout{m_previous / two_pi};
		auto const hydrotorque{nin * nin * m_torqueinin + (nin - nout) * m_torqueinout + nout * nout * m_torqueoutout};
		auto const speedratio{nout / std::max(0.01, nin)};
		double torquein;
		double torqueout;
		if (false == m_table.empty())
		{
			m_tmratio = interpolate(m_table, 1, speedratio);
			torqueout = hydrotorque * m_fill * m_tmratio;
			torquein = hydrotorque * m_fill * std::min(1.0, m_tmratio);
		}
		else if (speedratio < m_couplingpoint)
		{
			// converter range: the torque is multiplied
			auto const x{1.0 - speedratio / m_couplingpoint};
			m_tmratio = 1.0 + (m_tmmax - 1.0) * x * x;
			torquein = hydrotorque * m_fill;
			torqueout = hydrotorque * m_fill * m_tmratio;
		}
		else
		{
			// fluid coupling range
			m_tmratio = (1.0 - speedratio) / (1.0 - m_couplingpoint);
			torquein = hydrotorque * m_fill * m_tmratio;
			torqueout = torquein;
		}
		auto const torquelock{m_lockuptorque * m_lockuprate};

		// the main clutch of dizel_Momentum is taken as always engaged; the converter is drained to disconnect the engine
		double gearmoment;
		auto const capacity{torquelock + std::abs(torquein)};
		if (std::abs(moment) > capacity || std::abs(m_previous - m_omega) > 0.1 * two_pi)
		{
			auto const deltaomega{m_omega - m_previous};
			// NOTE: dizel_Momentum adds the lockup torque without the sign of the slip; here it acts against the slip too
			gearmoment = (torquelock + std::abs(torqueout)) * sign(deltaomega);
			auto const eps{(moment - capacity * sign(deltaomega)) / m_inertia};
			auto newomega{m_omega + eps * Frame.dt};
			if ((newomega - n) * deltaomega < 0.0 && torquelock > 0.1)
			{
				// the lockup clutch grips
				newomega = n;
			}
			if (newomega * m_omega <= 0.0 && eps * m_omega < 0.0)
			{
				newomega = 0.0;
			}
			m_omega = newomega;
		}
		else
		{
			gearmoment = moment;
			m_omega = clamp_any(n, m_omega + (moment - capacity) / m_inertia * Frame.dt, m_omega + (moment + capacity) / m_inertia * Frame.dt);
		}
		m_previous = n;

		set_forward(Frame, port::out, gearmoment * dir);
		add_back(Frame, port::in, m_omega);
		set(Frame, port::fill, m_fill);
		set_bool(Frame, port::lockup, m_lockup);
		// the ratio of an empty converter means nothing (and grows without limit when the engine stops)
		set(Frame, port::ratio, m_fill > 0.0 ? m_tmratio : 0.0);
		body.hydro_TC_Fill = m_fill;
		body.hydro_TC_Lockup = m_lockup;
		body.hydro_TC_LockupRate = m_lockuprate;
		body.hydro_TC_TMRatio = m_tmratio;
		body.hydro_TC_TorqueIn = torquein;
		body.hydro_TC_TorqueOut = gearmoment;
		body.hydro_TC_nIn = nin;
		body.hydro_TC_nOut = nout;
	}

	static block_type describe()
	{
		auto type{make_type("drive.hydro", "Hydraulic transmission",
		                    "Torque converter with filling, fluid coupling range above the coupling point and a lockup clutch, as hydro_TC of the .fiz files (dizel_Momentum). "
		                    "power fills the converter (with the engine above 0.9 of idle_rpm); without it the converter drains below ans_speed. The lockup clutch engages "
		                    "above lockup_speed with power and releases below it (below unlock_speed without power). running (the engine's) false drains the converter and "
		                    "releases the lockup clutch. Holds the speed of the engine (inertia of the engine side).",
		                    block_origin::extracted)};
		type.ports = {physical_in("in", domain::mechanical),
		              signal_in("power", value_type::boolean),
		              signal_in("running", value_type::boolean, 1.0, "", true),
		              signal_in("dir", value_type::real, 1.0, "", true),
		              physical_out("out", domain::mechanical),
		              signal_out("fill", value_type::real),
		              signal_out("lockup", value_type::boolean),
		              signal_out("ratio", value_type::real)};
		type.params = {number_param("tc_tmmax", 2.0),
		               number_param("tc_cp", 0.85),
		               table_param("tc_table", {"speed_ratio", "torque_ratio"}),
		               number_param("tc_tii", 4.5),
		               number_param("tc_tio", 0.0),
		               number_param("tc_too", 0.0),
		               number_param("fill_rate", 1.0, "1/s"),
		               number_param("drain_rate", 1.0, "1/s"),
		               number_param("lockup_torque", 3000.0, "Nm"),
		               number_param("lockup_speed", 40.0, "km/h"),
		               number_param("unlock_speed", 30.0, "km/h"),
		               number_param("idle_rpm", 600.0, "rpm"),
		               number_param("ans_speed", 3.0, "km/h"),
		               number_param("inertia", 10.0, "kg m2")};
		type.create = make_block<drive_hydro>;
		return type;
	}

private:
	double const m_tmmax;
	double const m_couplingpoint;
	std::vector<std::vector<double>> m_table;
	double const m_torqueinin;
	double const m_torqueinout;
	double const m_torqueoutout;
	double const m_fillrate;
	double const m_drainrate;
	double const m_lockuptorque;
	double const m_lockupspeed;
	double const m_unlockspeed;
	double const m_idle; // rad/s
	double const m_ansspeed;
	double const m_inertia;
	double m_fill{0.0};
	bool m_lockup{false};
	double m_lockuprate{0.0};
	double m_tmratio{1.0};
	double m_omega{0.0}; // engine speed, rad/s
	double m_previous{0.0}; // output speed in the previous step, rad/s
};

// main generator of a diesel-electric vehicle: separately excited dc generator (adapted from the DieselElectric part of
// TMoverParameters::TractionForce: the ellipse of the generator characteristic and the power available at the engine
// speed). holds the speed of the engine
class drive_generator final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			excitation,
			out,
			u,
			i,
			p
		};
	};
	struct param
	{
		enum : std::size_t
		{
			u_max,
			i_max,
			p_max,
			nominal_rpm,
			eff,
			field_time,
			inertia
		};
	};

	explicit drive_generator(block_setup const &Setup)
	    : block(Setup), m_umax(std::max(0.0, number(Setup, param::u_max))), m_imax(std::max(1.0, number(Setup, param::i_max))),
	      m_pmax(std::max(0.0, number(Setup, param::p_max)) * 1000.0), m_nominal(std::max(1.0, number(Setup, param::nominal_rpm)) * rpm_to_rad),
	      m_efficiency(std::clamp(number(Setup, param::eff), 0.01, 1.0)), m_fieldtime(std::max(0.0, number(Setup, param::field_time))),
	      m_inertia(std::max(0.01, number(Setup, param::inertia)))
	{
	}

	void update(frame &Frame) override
	{
		auto const moment{forward_in(Frame, port::in)};
		auto const current{back_out(Frame, port::out)}; // drawn by the load in the previous step
		auto const excitation{std::clamp(get(Frame, port::excitation), 0.0, 1.0)};
		auto const speedratio{std::clamp(m_omega / m_nominal, 0.0, 1.0)};
		// the voltage at no load grows with the excitation and the speed; under load it follows the ellipse of the
		// separately excited generator (zero at i_max) and the hyperbola of the power available at this speed
		auto const noload{m_umax * excitation * speedratio};
		auto const load{std::min(1.0, std::abs(current) / m_imax)};
		auto target{noload * std::sqrt(std::max(0.0, 1.0 - load * load))};
		if (std::abs(current) > 1.0)
		{
			target = std::min(target, m_pmax * speedratio / std::abs(current));
		}
		// the field of the generator follows with its time constant, which also keeps the loop with the motors stable
		m_voltage += (target - m_voltage) * (m_fieldtime > 0.0 ? std::min(1.0, Frame.dt / m_fieldtime) : 1.0);
		m_power = m_voltage * current;
		// load of the engine: the electric power and the losses
		auto const loadtorque{m_power / m_efficiency / std::max(m_omega, 1.0)};
		m_omega = std::max(0.0, m_omega + (moment - loadtorque) / m_inertia * Frame.dt);

		set_forward(Frame, port::out, m_voltage);
		add_back(Frame, port::in, m_omega);
		set(Frame, port::u, m_voltage);
		set(Frame, port::i, current);
		set(Frame, port::p, m_power / 1000.0);
	}

	static block_type describe()
	{
		auto type{make_type("drive.generator", "Main generator",
		                    "Main generator of a diesel-electric vehicle. The voltage at no load is u_max times the excitation (0..1), proportional to the engine speed "
		                    "below nominal_rpm; under load it falls on an ellipse to zero at i_max, and the power is limited to p_max scaled with the engine speed "
		                    "(DieselElectric of TractionForce). The electric power loads the engine. Holds the speed of the engine (inertia of the engine and the rotor).",
		                    block_origin::extracted)};
		type.ports = {physical_in("in", domain::mechanical), signal_in("excitation", value_type::real), physical_out("out", domain::high_voltage),
		              signal_out("u", value_type::real, "V"),     signal_out("i", value_type::real, "A"),  signal_out("p", value_type::real, "kW")};
		type.params = {number_param("u_max", 750.0, "V"),
		               number_param("i_max", 2400.0, "A"),
		               number_param("p_max", 330.0, "kW"),
		               number_param("nominal_rpm", 1500.0, "rpm"),
		               number_param("eff", 0.93),
		               number_param("field_time", 0.5, "s"),
		               number_param("inertia", 20.0, "kg m2")};
		type.create = make_block<drive_generator>;
		return type;
	}

private:
	double const m_umax;
	double const m_imax;
	double const m_pmax; // W
	double const m_nominal; // rad/s
	double const m_efficiency;
	double const m_fieldtime;
	double const m_inertia;
	double m_voltage{0.0};
	double m_power{0.0};
	double m_omega{0.0}; // engine speed, rad/s
};

} // namespace

void register_diesel_blocks(catalog &Catalog)
{
	Catalog.add(drive_diesel::describe());
	Catalog.add(drive_gearbox::describe());
	Catalog.add(drive_hydro::describe());
	Catalog.add(drive_generator::describe());
}

} // namespace vgraph
