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

// drive chain: motor -> gear -> axle. the mechanical connections pass the torque forward and the rotational speed back.
// both are in the frame of the vehicle: positive torque pushes the vehicle towards its front (cab 1), positive speed
// is the rotation when moving that way. the dir input of the motors is the position of the reverser in the occupied cab,
// so the motors turn it to the frame of the vehicle with the occupied cab
namespace vgraph
{

namespace
{

double constexpr two_pi{2.0 * M_PI};

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

// direction of the drive in the frame of the vehicle: -1, 0 or 1
int absolute_direction(frame const &Frame, double const Dir)
{
	auto const dir{Dir > 0.5 ? 1 : Dir < -0.5 ? -1 : 0};
	auto const cab{Frame.body->cab};
	return dir * (cab != 0 ? cab : 1);
}

// generic motor with a torque and power limit; a stand-in for the drives not extracted yet
class drive_simple final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			wn,
			demand,
			dir,
			shaft,
			i,
			p
		};
	};
	struct param
	{
		enum : std::size_t
		{
			t_max,
			p_max,
			eff,
			min_voltage
		};
	};

	explicit drive_simple(block_setup const &Setup)
	    : block(Setup), m_supplied(is_connected(Setup, port::wn)), m_tmax(number(Setup, param::t_max)), m_pmax(number(Setup, param::p_max)),
	      m_efficiency(std::clamp(number(Setup, param::eff), 0.01, 1.0)), m_minvoltage(number(Setup, param::min_voltage))
	{
	}

	void update(frame &Frame) override
	{
		auto const voltage{m_supplied ? forward_in(Frame, port::wn) : 0.0};
		auto const powered{false == m_supplied || (voltage > 0.0 && voltage >= m_minvoltage)};
		auto const dir{absolute_direction(Frame, get(Frame, port::dir))};
		auto const demand{std::clamp(get(Frame, port::demand), -1.0, 1.0)};
		auto const speed{back_out(Frame, port::shaft)}; // rad/s
		auto torque{0.0};
		if (powered && dir != 0)
		{
			if (demand >= 0.0)
			{
				// traction, limited by the power above the base speed
				auto const limit{std::abs(speed) > 1e-3 ? std::min(m_tmax, m_pmax / std::abs(speed)) : m_tmax};
				torque = dir * demand * limit;
			}
			else
			{
				// electric brake, against the rotation
				auto const limit{std::abs(speed) > 1e-3 ? std::min(m_tmax, m_pmax / std::abs(speed)) : 0.0};
				torque = (speed >= 0.0 ? -1.0 : 1.0) * (-demand) * limit * std::min(1.0, std::abs(speed));
			}
		}
		auto const mechanical{torque * speed}; // W, positive when driving
		auto const electrical{mechanical >= 0.0 ? mechanical / m_efficiency : mechanical * m_efficiency};
		auto const current{m_supplied && voltage > 1.0 ? electrical / voltage : 0.0};
		if (m_supplied)
		{
			add_back(Frame, port::wn, current);
		}
		set_forward(Frame, port::shaft, torque);
		set(Frame, port::i, current);
		set(Frame, port::p, electrical);
		auto &body{vehicle(Frame)};
		body.Im = current;
		body.EngineVoltage = voltage * dir;
		body.enrot = speed / two_pi;
	}

	static block_type describe()
	{
		auto type{make_type("drive.simple", "Simple motor",
		                    "Motor with a torque limit (t_max) and a power limit (p_max): demand 0..1 drives, -1..0 brakes electrically. A stand-in for prototypes.",
		                    block_origin::native)};
		type.ports = {physical_in("wn", domain::high_voltage, true), signal_in("demand", value_type::real), signal_in("dir", value_type::real),
		              physical_out("shaft", domain::mechanical),      signal_out("i", value_type::real, "A"), signal_out("p", value_type::real, "W")};
		type.params = {number_param("t_max", 5000.0, "Nm"), number_param("p_max", 500000.0, "W"), number_param("eff", 0.9), number_param("min_voltage", 0.0, "V")};
		type.create = make_block<drive_simple>;
		return type;
	}

private:
	bool const m_supplied;
	double const m_tmax;
	double const m_pmax;
	double const m_efficiency;
	double const m_minvoltage;
};

class drive_gear final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			out
		};
	};

	explicit drive_gear(block_setup const &Setup) : block(Setup), m_ratio(number(Setup, 0)), m_efficiency(number(Setup, 1)) {}

	void update(frame &Frame) override
	{
		set_forward(Frame, port::out, forward_in(Frame, port::in) * m_ratio * m_efficiency);
		add_back(Frame, port::in, back_out(Frame, port::out) * m_ratio);
	}

	static block_type describe()
	{
		auto type{make_type("drive.gear", "Gear", "Transmission: torque times ratio and efficiency, speed of the input times ratio. ratio = wheel teeth / motor teeth.",
		                    block_origin::extracted)};
		type.ports = {physical_in("in", domain::mechanical), physical_out("out", domain::mechanical)};
		type.params = {number_param("ratio", 1.0), number_param("eff", 1.0)};
		type.create = make_block<drive_gear>;
		return type;
	}

private:
	double const m_ratio;
	double const m_efficiency;
};

// wheelsets driven by one shaft: the torque becomes the tractive force of the vehicle
class drive_axle final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			shaft,
			force,
			slip
		};
	};

	explicit drive_axle(block_setup const &Setup) : block(Setup), m_axles(std::max(0.0, number(Setup, 0))) {}

	void update(frame &Frame) override
	{
		auto const &body{vehicle(Frame)};
		auto const diameter{body.WheelDiameter > 0.0 ? body.WheelDiameter : 1.0};
		auto const force{forward_in(Frame, port::shaft) * 2.0 / diameter * m_axles};
		Frame.body->tractive_force += force;
		add_back(Frame, port::shaft, Frame.body->wheel_speed * two_pi);
		set(Frame, port::force, force);
		set_bool(Frame, port::slip, Frame.body->slipping);
	}

	static block_type describe()
	{
		auto type{make_type("drive.axle", "Driven axles",
		                    "Driven wheelsets: the torque of one shaft (of one motor, after the gear) times powered_axles gives the force at the wheel rim. All axle forces are "
		                    "summed into the tractive force of the vehicle.",
		                    block_origin::native)};
		type.ports = {physical_in("shaft", domain::mechanical), signal_out("force", value_type::real, "N"), signal_out("slip", value_type::boolean)};
		type.params = {number_param("powered_axles", 1.0)};
		type.create = make_block<drive_axle>;
		return type;
	}

private:
	double const m_axles;
};

// series dc traction motors with resistor starting (TMoverParameters::Current, Momentum)
class drive_dc_series final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			wn,
			position,
			shunt,
			dir,
			high,
			reset,
			shaft,
			i,
			u_motor,
			overload
		};
	};
	struct param
	{
		enum : std::size_t
		{
			rlist,
			mpt,
			volt,
			winding_res,
			circuit_res,
			imax_lo,
			imax_hi,
			imin_lo,
			imin_hi,
			trip_delay
		};
	};

	explicit drive_dc_series(block_setup const &Setup)
	    : block(Setup), m_windingres(number(Setup, param::winding_res)), m_circuitres(number(Setup, param::circuit_res)), m_imaxlo(number(Setup, param::imax_lo)),
	      m_imaxhi(number(Setup, param::imax_hi)), m_tripdelay(number(Setup, param::trip_delay))
	{
		for (auto const &row : table(Setup, param::rlist))
		{
			// pos, r, bn, mn, auto
			scheme entry;
			entry.r = row[1];
			entry.bn = static_cast<int>(std::lround(row[2]));
			entry.mn = static_cast<int>(std::lround(row[3]));
			m_schemes.push_back(entry);
		}
		for (auto const &row : table(Setup, param::mpt))
		{
			// pos, mfi, mIsat, mfi0, fi, Isat, fi0
			TMotorParameters motor;
			motor.mfi = row[1];
			motor.mIsat = row[2];
			motor.mfi0 = row[3];
			motor.fi = row[4];
			motor.Isat = row[5];
			motor.fi0 = row[6];
			m_motors.push_back(motor);
		}
		if (m_schemes.empty())
		{
			m_schemes.emplace_back();
		}
		if (m_motors.empty())
		{
			m_motors.emplace_back();
		}
	}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const dir{absolute_direction(Frame, get(Frame, port::dir))};
		auto const voltage{forward_in(Frame, port::wn) * dir};
		auto const position{std::clamp<int>(static_cast<int>(std::lround(get(Frame, port::position))), 0, static_cast<int>(m_schemes.size()) - 1)};
		auto const shunt{std::clamp<int>(static_cast<int>(std::lround(get(Frame, port::shunt))), 0, static_cast<int>(m_motors.size()) - 1)};
		auto const &scheme{m_schemes[position]};
		auto const &motor{m_motors[shunt]};
		auto const revolutions{back_out(Frame, port::shaft) / two_pi}; // rev/s

		if (get_bool(Frame, port::reset))
		{
			m_overload = false;
		}
		auto current{0.0};
		if (scheme.bn != 0 && std::abs(voltage) > 1.0 && false == m_overload)
		{
			auto const resistance{scheme.r + m_circuitres};
			current = series_motor_current(voltage, revolutions, scheme.mn, scheme.mn * m_windingres + resistance, motor);
		}
		// overload relay, as in TractionForce: trips when the current stays above the limit
		auto const imax{get_bool(Frame, port::high) ? m_imaxhi : m_imaxlo};
		if (imax > 0.0 && std::abs(current) > imax)
		{
			m_overtime += Frame.dt;
			if (m_overtime > m_tripdelay)
			{
				m_overload = true;
			}
		}
		else
		{
			m_overtime = 0.0;
		}
		auto const torque{series_motor_momentum(current, motor)};
		auto const linecurrent{current * scheme.bn};
		add_back(Frame, port::wn, std::abs(linecurrent));
		set_forward(Frame, port::shaft, torque);
		set(Frame, port::i, current);
		set(Frame, port::u_motor, scheme.mn > 0 ? (std::abs(voltage) - std::abs(current) * (scheme.r + m_circuitres)) / scheme.mn : 0.0);
		set_bool(Frame, port::overload, m_overload);
		body.Im = current;
		body.EngineVoltage = voltage;
		body.enrot = revolutions;
		body.MainCtrlActualPos = position;
	}

	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "overload";
	}
	double state_value(std::size_t) const override
	{
		return m_overload ? 1.0 : 0.0;
	}

	static block_type describe()
	{
		auto type{make_type("drive.dc_series", "Series DC motors",
		                    "Series wound DC traction motors with resistor starting, computed as for the .fiz vehicles. position selects the row of rlist (resistance, "
		                    "branches, motors in series), shunt the row of mpt (motor characteristics). The shaft carries the torque of one motor. high selects imax_hi "
		                    "for the overload relay; reset clears it.",
		                    block_origin::extracted)};
		type.ports = {physical_in("wn", domain::high_voltage),
		              signal_in("position", value_type::integer),
		              signal_in("shunt", value_type::integer, 0.0, "", true),
		              signal_in("dir", value_type::real),
		              signal_in("high", value_type::boolean, 0.0, "", true),
		              signal_in("reset", value_type::boolean, 0.0, "", true),
		              physical_out("shaft", domain::mechanical),
		              signal_out("i", value_type::real, "A"),
		              signal_out("u_motor", value_type::real, "V"),
		              signal_out("overload", value_type::boolean)};
		type.params = {table_param("rlist", {"pos", "r", "bn", "mn", "auto"}),
		               table_param("mpt", {"pos", "mfi", "misat", "mfi0", "fi", "isat", "fi0"}),
		               number_param("volt", 1500.0, "V"),
		               number_param("winding_res", 0.0, "ohm"),
		               number_param("circuit_res", 0.0, "ohm"),
		               number_param("imax_lo", 0.0, "A"),
		               number_param("imax_hi", 0.0, "A"),
		               number_param("imin_lo", 0.0, "A"),
		               number_param("imin_hi", 0.0, "A"),
		               number_param("trip_delay", 0.05, "s")};
		type.create = make_block<drive_dc_series>;
		return type;
	}

private:
	struct scheme
	{
		double r{0.0};
		int bn{0};
		int mn{0};
	};

	double const m_windingres;
	double const m_circuitres;
	double const m_imaxlo;
	double const m_imaxhi;
	double const m_tripdelay;
	std::vector<scheme> m_schemes;
	std::vector<TMotorParameters> m_motors;
	double m_overtime{0.0};
	bool m_overload{false};
};

// induction motors fed by inverters (the ElectricInductionMotor drive of TMoverParameters::TractionForce)
class drive_async final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			wn,
			demand,
			dir,
			shaft,
			i,
			f,
			ed_active,
			ed
		};
	};
	// in the order of the eimc constants they're copied to
	struct param
	{
		enum : std::size_t
		{
			dfic,
			dfmax,
			p,
			cfu,
			cim,
			icif,
			uzmax,
			uzh,
			du,
			i0,
			fcfu,
			fcfuh,
			f0,
			a1,
			pmax,
			fh,
			ph,
			vh0,
			vh1,
			imax,
			abed,
			edep,
			flat,
			motors,
			ratio,
			eff,
			log_force
		};
	};

	explicit drive_async(block_setup const &Setup)
	    : block(Setup), m_flat(boolean(Setup, param::flat)), m_motors(static_cast<int>(std::lround(number(Setup, param::motors)))), m_ratio(number(Setup, param::ratio)),
	      m_efficiency(number(Setup, param::eff)), m_logforce(boolean(Setup, param::log_force))
	{
		std::fill(std::begin(m_eimc), std::end(m_eimc), 0.0);
		std::fill(std::begin(m_eimv), std::end(m_eimv), 0.0);
		m_eimc[eimc_s_dfic] = number(Setup, param::dfic);
		m_eimc[eimc_s_dfmax] = number(Setup, param::dfmax);
		m_eimc[eimc_s_p] = number(Setup, param::p);
		m_eimc[eimc_s_cfu] = number(Setup, param::cfu);
		m_eimc[eimc_s_cim] = number(Setup, param::cim);
		m_eimc[eimc_s_icif] = number(Setup, param::icif);
		m_eimc[eimc_f_Uzmax] = number(Setup, param::uzmax);
		m_eimc[eimc_f_Uzh] = number(Setup, param::uzh);
		m_eimc[eimc_f_DU] = number(Setup, param::du);
		m_eimc[eimc_f_I0] = number(Setup, param::i0);
		m_eimc[eimc_f_cfu] = number(Setup, param::fcfu);
		// as in LoadFIZ_Engine: fcfuH defaults to fcfu
		m_eimc[eimc_f_cfuH] = number(Setup, param::fcfuh) > 0.0 ? number(Setup, param::fcfuh) : m_eimc[eimc_f_cfu];
		m_eimc[eimc_p_F0] = number(Setup, param::f0);
		m_eimc[eimc_p_a1] = number(Setup, param::a1);
		m_eimc[eimc_p_Pmax] = number(Setup, param::pmax);
		m_eimc[eimc_p_Fh] = number(Setup, param::fh);
		m_eimc[eimc_p_Ph] = number(Setup, param::ph);
		m_eimc[eimc_p_Vh0] = number(Setup, param::vh0);
		m_eimc[eimc_p_Vh1] = number(Setup, param::vh1);
		m_eimc[eimc_p_Imax] = number(Setup, param::imax);
		m_eimc[eimc_p_abed] = number(Setup, param::abed);
		m_eimc[eimc_p_eped] = number(Setup, param::edep);
	}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const voltage{forward_in(Frame, port::wn)};
		auto const dir{absolute_direction(Frame, get(Frame, port::dir))};
		auto const enrot{back_out(Frame, port::shaft) / two_pi};
		auto const demand{std::clamp(get(Frame, port::demand), -1.0, 1.0)};
		if (voltage < 1.0 || dir == 0)
		{
			// without supply the inverters stop, as for the vehicles without the main circuit (Mains)
			std::fill(std::begin(m_eimv), std::end(m_eimv), 0.0);
			m_eimvpr = 0.0;
			m_dynamicbrake = false;
			add_back(Frame, port::wn, 0.0);
			publish(Frame, body, 0.0, 0.0, 0.0, voltage * dir, enrot);
			return;
		}
		auto const vel{body.Vel};
		// set value of the force and the rate of its ramp, as in TractionForce
		double posratio;
		double rate;
		if (demand < -0.001)
		{
			m_dynamicbrake = true;
			auto const vhramp{std::clamp((vel - m_eimc[eimc_p_Vh0]) / std::max(0.001, m_eimc[eimc_p_Vh1] - m_eimc[eimc_p_Vh0]), 0.0, 1.0)};
			posratio = demand * vhramp;
			rate = 5.0;
		}
		else
		{
			m_dynamicbrake = false;
			posratio = std::max(demand, 0.0);
			if (m_flat && m_eimc[eimc_p_F0] * m_eimv[eimv_Fful] > 0)
			{
				posratio = std::min(posratio * m_eimc[eimc_p_F0] / m_eimv[eimv_Fful], 1.0);
			}
			rate = 4.0;
		}
		if (body.SlippingWheels)
		{
			// anti-slip: the set value drops quickly
			posratio = 0.0;
			rate = 10.0;
		}
		induction_motor_inputs input;
		input.demand = posratio;
		input.rate = rate;
		input.dynamic_brake = m_dynamicbrake;
		input.engine_voltage = voltage;
		input.enrot = enrot;
		input.vel = vel;
		input.v = body.V;
		input.dir_absolute = dir;
		input.dir_active = dir;
		input.powered_axles = m_motors;
		input.inverters_ratio = 1.0;
		input.ratio = m_ratio;
		input.efficiency = m_efficiency;
		input.wheel_diameter = body.WheelDiameter > 0.0 ? body.WheelDiameter : 1.0;
		input.log_force = m_logforce;
		input.max_voltage = body.EnginePowerSource.CollectorParameters.MaxV;
		input.dt = Frame.dt;
		induction_motor_outputs output;
		induction_motor_update(input, m_eimc, m_eimv, m_eimvpr, m_vadd, output);

		add_back(Frame, port::wn, output.Itot);
		// share of the requested braking force the motors deliver, for the blending with the pneumatic brake
		auto edshare{0.0};
		if (m_dynamicbrake && m_eimc[eimc_p_Fh] > 0.0 && demand < -0.001)
		{
			edshare = std::clamp(std::abs(m_eimv[eimv_Fr]) / (m_eimc[eimc_p_Fh] * -demand), 0.0, 1.0);
		}
		publish(Frame, body, output.Mm, output.Im, edshare, voltage, enrot);
	}

	std::size_t state_count() const override
	{
		return 2;
	}
	char const *state_name(std::size_t Index) const override
	{
		return Index == 0 ? "set_value" : "force_kn";
	}
	double state_value(std::size_t Index) const override
	{
		return Index == 0 ? m_eimvpr : m_eimv[eimv_Fr];
	}

	static block_type describe()
	{
		auto type{make_type("drive.async", "Induction motors with inverters",
		                    "Induction traction motors fed by inverters, computed as for the .fiz vehicles (Engine: ElectricInductionMotor). demand 0..1 drives, -1..0 brakes "
		                    "electrically (faded out between vh1 and vh0). The shaft carries the torque of one motor; ratio and eff have to match the gear, and motors the "
		                    "driven axles. ed is the share of the requested braking force the motors deliver, for the ed input of the distributor.",
		                    block_origin::extracted)};
		type.ports = {physical_in("wn", domain::high_voltage),       signal_in("demand", value_type::real),     signal_in("dir", value_type::real),
		              physical_out("shaft", domain::mechanical),      signal_out("i", value_type::real, "A"),    signal_out("f", value_type::real, "Hz"),
		              signal_out("ed_active", value_type::boolean),   signal_out("ed", value_type::real)};
		type.params = {number_param("dfic", 0.0),
		               number_param("dfmax", 0.0),
		               number_param("p", 2.0),
		               number_param("cfu", 0.0),
		               number_param("cim", 0.0),
		               number_param("icif", 0.0),
		               number_param("uzmax", 0.0, "V"),
		               number_param("uzh", 0.0, "V"),
		               number_param("du", 0.0, "V"),
		               number_param("i0", 0.0, "A"),
		               number_param("fcfu", 0.0),
		               number_param("fcfuh", 0.0),
		               number_param("f0", 0.0, "kN"),
		               number_param("a1", 0.0),
		               number_param("pmax", 0.0, "kW"),
		               number_param("fh", 0.0, "kN"),
		               number_param("ph", 0.0, "kW"),
		               number_param("vh0", 0.0, "km/h"),
		               number_param("vh1", 0.0, "km/h"),
		               number_param("imax", 0.0, "A"),
		               number_param("abed", 1.0),
		               number_param("edep", 1.5),
		               boolean_param("flat", false),
		               number_param("motors", 4.0),
		               number_param("ratio", 1.0),
		               number_param("eff", 1.0),
		               boolean_param("log_force", false)};
		type.create = make_block<drive_async>;
		return type;
	}

private:
	void publish(frame &Frame, TMoverParameters &Body, double const Torque, double const Current, double const Edshare, double const Voltage, double const Revolutions)
	{
		set_forward(Frame, port::shaft, Torque);
		set(Frame, port::i, Current);
		set(Frame, port::f, std::abs(m_eimv[eimv_fp]));
		set_bool(Frame, port::ed_active, m_dynamicbrake);
		set(Frame, port::ed, Edshare);
		Body.Im = Current;
		Body.EngineVoltage = Voltage;
		Body.enrot = Revolutions;
		Body.DynamicBrakeFlag = m_dynamicbrake;
		Body.InverterFrequency = std::abs(m_eimv[eimv_fp]);
		std::copy(std::begin(m_eimv), std::end(m_eimv), Body.eimv);
	}

	bool const m_flat;
	int const m_motors;
	double const m_ratio;
	double const m_efficiency;
	bool const m_logforce;
	double m_eimc[26];
	double m_eimv[21];
	double m_eimvpr{0.0};
	double m_vadd{0.0};
	bool m_dynamicbrake{false};
};

} // namespace

void register_drive_blocks(catalog &Catalog)
{
	Catalog.add(drive_simple::describe());
	Catalog.add(drive_dc_series::describe());
	Catalog.add(drive_async::describe());
	Catalog.add(drive_gear::describe());
	Catalog.add(drive_axle::describe());
}

} // namespace vgraph
