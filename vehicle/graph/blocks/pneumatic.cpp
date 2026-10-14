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

// air supply and brakes. the blocks drive the brake equipment of the vehicle body (TMoverParameters): its brake pipe and
// feed pipe, which exchange air with the neighbouring vehicles, and the distributor and driver's brake valves created
// by CheckLocomotiveParameters from the parameters of these blocks, with the same code as for the .fiz vehicles.
// the sequence of the operations follows TMoverParameters::UpdatePipePressure and UpdateScndPipePressure
namespace vgraph
{

namespace
{

// cross-section of the feed pipe connection, as in TMoverParameters::UpdateScndPipePressure
double constexpr feed_pipe_section{0.5067};

TMoverParameters &vehicle(frame const &Frame)
{
	return *Frame.body->mover;
}

block_type make_type(std::string Id, std::string Category, std::string Name, std::string Description, block_origin Origin)
{
	block_type type;
	type.id = std::move(Id);
	type.category = std::move(Category);
	type.name = std::move(Name);
	type.description = std::move(Description);
	type.origin = Origin;
	return type;
}

// compressor of the main reservoir
class pn_compressor final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			power,
			power_hv,
			on,
			air,
			run
		};
	};
	struct param
	{
		enum : std::size_t
		{
			flow,
			max_pressure,
			min_voltage,
			power
		};
	};

	explicit pn_compressor(block_setup const &Setup)
	    : block(Setup), m_aux(is_connected(Setup, port::power)), m_hv(is_connected(Setup, port::power_hv)), m_flow(number(Setup, param::flow)),
	      m_maxpressure(number(Setup, param::max_pressure)), m_minvoltage(number(Setup, param::min_voltage)), m_power(number(Setup, param::power))
	{
	}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		// without supply connections the compressor is assumed to be powered
		auto powered{false == m_aux && false == m_hv};
		auto const auxvoltage{m_aux ? forward_in(Frame, port::power) : 0.0};
		auto const hvvoltage{m_hv ? forward_in(Frame, port::power_hv) : 0.0};
		powered = powered || (m_aux && auxvoltage >= m_minvoltage && auxvoltage > 0.0) || (m_hv && hvvoltage >= m_minvoltage && hvvoltage > 0.0);
		auto const run{powered && get_bool(Frame, port::on) && body.VeselVolume > 0.0 && m_maxpressure > 0.0};
		body.CompressorAllow = get_bool(Frame, port::on);
		body.CompressorFlag = run;
		if (run)
		{
			// TMoverParameters::CompressorCheck, compressor working at steady pace
			body.CompressedVolume += m_flow * (2.0 * m_maxpressure - body.Compressor) / m_maxpressure * Frame.dt;
			if (m_aux && auxvoltage > 0.0)
			{
				add_back(Frame, port::power, m_power);
			}
			else if (m_hv && hvvoltage > 0.0)
			{
				add_back(Frame, port::power_hv, m_power / hvvoltage);
			}
		}
		set_forward(Frame, port::air, body.Compressor);
		set_bool(Frame, port::run, run);
	}

	static block_type describe()
	{
		auto type{make_type("pn.compressor", "pneumatics", "Compressor",
		                    "Compressor of the main reservoir. Without a supply connected it's assumed to be powered; with both connected either one is enough.",
		                    block_origin::extracted)};
		type.ports = {physical_in("power", domain::aux_ac, true), physical_in("power_hv", domain::high_voltage, true), signal_in("on", value_type::boolean),
		              physical_out("air", domain::air), signal_out("run", value_type::boolean)};
		type.params = {fiz(number_param("flow", 0.05, "m3/s"), "Brake:", "CompressorSpeed"), number_param("max_pressure", 10.0, "bar"), number_param("min_voltage", 0.0, "V"),
		               number_param("power", 10000.0, "W")};
		type.single = true;
		type.create = make_block<pn_compressor>;
		return type;
	}

private:
	bool const m_aux;
	bool const m_hv;
	double const m_flow;
	double const m_maxpressure;
	double const m_minvoltage;
	double const m_power;
};

// pressure switch of the compressor: asks for air below the lower pressure, until the upper one is reached
class pn_governor final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			p,
			need
		};
	};
	struct param
	{
		enum : std::size_t
		{
			min,
			max
		};
	};

	explicit pn_governor(block_setup const &Setup) : block(Setup), m_min(number(Setup, param::min)), m_max(number(Setup, param::max)) {}

	void update(frame &Frame) override
	{
		auto const pressure{get(Frame, port::p)};
		if (pressure < m_min)
		{
			m_need = true;
		}
		else if (pressure > m_max)
		{
			m_need = false;
		}
		vehicle(Frame).CompressorGovernorLock = false == m_need;
	}
	void commit(frame &Frame) override
	{
		set_bool(Frame, port::need, m_need);
	}
	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "need";
	}
	double state_value(std::size_t) const override
	{
		return m_need ? 1.0 : 0.0;
	}

	static block_type describe()
	{
		auto type{make_type("pn.governor", "pneumatics", "Pressure governor", "Pressure switch of the compressor: on below min, off above max.", block_origin::primitive)};
		type.ports = {signal_in("p", value_type::real, 0.0, "bar"), signal_out("need", value_type::boolean)};
		type.params = {fiz(number_param("min", 7.5, "bar"), "Brake:", "MinCP"), fiz(number_param("max", 8.5, "bar"), "Brake:", "MaxCP")};
		type.deferred = true;
		type.create = make_block<pn_governor>;
		return type;
	}

private:
	double const m_min;
	double const m_max;
	bool m_need{true};
};

// main reservoir of the vehicle, and its connection with the feed pipe
class pn_reservoir final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			out,
			pz,
			p
		};
	};

	explicit pn_reservoir(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const dt{Frame.dt};
		if (body.VeselVolume > 0.0)
		{
			// TMoverParameters::CompressorCheck: safety valve and leaks
			body.Compressor = body.CompressedVolume / body.VeselVolume;
			body.EmergencyValveOpen = body.Compressor > (body.EmergencyValveOpen ? body.EmergencyValveOff : body.EmergencyValveOn);
			if (body.EmergencyValveOpen)
			{
				body.CompressedVolume -= PF(0, body.Compressor, body.EmergencyValveArea) * dt;
			}
			body.CompressedVolume = std::max(0.0, body.CompressedVolume - dt * body.AirLeakRate * 0.1);
			body.Compressor = body.CompressedVolume / body.VeselVolume;
		}
		else
		{
			body.Compressor = 0.0;
		}
		set_forward(Frame, port::out, body.Compressor);
		set_forward(Frame, port::pz, body.ScndPipePress);
		set(Frame, port::p, body.Compressor);
	}

	// the feed pipe, after the other blocks took their air from it (TMoverParameters::UpdateScndPipePressure)
	void late_update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const dt{Frame.dt};
		// the end cocks (if.pz) are checked by the exchange
		auto const exchange{body.ScndPipeExchange(dt)};
		if (body.VeselVolume > 0.0)
		{
			auto const flow{PF(body.Compressor, body.ScndPipePress, feed_pipe_section) * dt};
			body.CompressedVolume += flow / 1000.0;
			body.Pipe2->Flow(-flow);
		}
		body.Pipe2->Flow(exchange);
		body.Pipe2->Act();
		body.ScndPipePress = body.Pipe2->P();
		if (body.ScndPipePress < -1.0)
		{
			body.ScndPipePress = -1.0;
			body.Pipe2->CreatePress(-1.0);
			body.Pipe2->Act();
		}
		if (body.CompressedVolume < 0.0)
		{
			body.CompressedVolume = 0.0;
		}
	}

	static block_type describe()
	{
		auto type{make_type("pn.reservoir", "pneumatics", "Main reservoir",
		                    "The main reservoir of the vehicle. It feeds the feed pipe (pz), which the brake valves and the neighbouring vehicles take air from.",
		                    block_origin::simulator)};
		type.ports = {physical_in("in", domain::air, true), physical_out("out", domain::air), physical_out("pz", domain::feed_pipe), signal_out("p", value_type::real, "bar")};
		type.params = {fiz(number_param("volume", 0.0, "m3"), "Brake:", "Vv")};
		type.single = true;
		type.late = true;
		type.create = make_block<pn_reservoir>;
		return type;
	}
};

// brake pipe of the vehicle
class br_pipe final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			vent,
			dist,
			couplers,
			p
		};
	};

	explicit br_pipe(block_setup const &Setup) : block(Setup), m_trainlength(std::max(0.0, number(Setup, 0))) {}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const dt{Frame.dt};
		if (Frame.body->standalone && m_trainlength > 0.0 && false == m_extended)
		{
			// without the simulator there are no neighbours: the pipe gets the volume of the train instead
			auto const pressure{body.Pipe->P()};
			body.Pipe->CreateCap((std::max(body.Dim.L, 14.0) + 0.5 + m_trainlength) * body.Spg);
			body.Pipe->CreatePress(pressure);
			m_extended = true;
		}
		// emergency valve: the vent input, and the emergency brakes the simulator applies to every vehicle
		body.EmergencyValveFlow = 0.0;
		if (get_bool(Frame, port::vent) || body.RadioStopFlag || body.AlarmChainFlag)
		{
			body.EmergencyValveFlow = PF(0, body.PipePress, 0.15) * dt;
		}
		body.Pipe->Flow(-body.EmergencyValveFlow);
		body.Pipe->Flow(-body.PipePress * 0.001 * dt);
		body.Pipe->Act();
		body.PipePress = body.Pipe->P();
		publish(Frame, body.PipePress);
	}

	// exchange with the neighbours, after the distributor took its air
	void late_update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		// the end cocks (if.pg) are checked by the exchange
		body.Pipe->Flow(body.GetDVc(Frame.dt));
		body.Pipe->Act();
		body.PipePress = body.Pipe->P();
		if (body.PipePress < -1.0)
		{
			body.PipePress = -1.0;
			body.Pipe->CreatePress(-1.0);
			body.Pipe->Act();
		}
	}

	static block_type describe()
	{
		auto type{make_type("br.pipe", "brakes", "Brake pipe",
		                    "The brake pipe of the vehicle, connected through the couplers with the neighbouring vehicles. vent opens the emergency valve. "
		                    "preview_train_length adds the pipe of a train of that length when the graph runs without the simulator.",
		                    block_origin::native)};
		type.ports = {physical_in("in", domain::brake_pipe, true), signal_in("vent", value_type::boolean, 0.0, "", true), physical_out("dist", domain::brake_pipe),
		              physical_out("couplers", domain::brake_pipe), signal_out("p", value_type::real, "bar")};
		type.params = {number_param("preview_train_length", 0.0, "m")};
		type.single = true;
		type.late = true;
		type.create = make_block<br_pipe>;
		return type;
	}

private:
	double const m_trainlength; // without the simulator: length of the brake pipe of the train behind the vehicle
	bool m_extended{false};

	void publish(frame &Frame, double const Pressure)
	{
		set_forward(Frame, port::dist, Pressure);
		set_forward(Frame, port::couplers, Pressure);
		set(Frame, port::p, Pressure);
	}
};

// driver's brake valve
class br_handle final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			zg,
			pos,
			emerg,
			reductor,
			pg,
			p_ctl
		};
	};

	explicit br_handle(block_setup const &Setup) : block(Setup), m_reductor(is_connected(Setup, port::reductor)) {}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const dt{Frame.dt};
		auto position{get(Frame, port::pos)};
		if (get_bool(Frame, port::emerg))
		{
			position = body.Handle->GetPos(bh_EB);
		}
		body.BrakeCtrlPosR = position;
		body.BrakeCtrlPos = static_cast<int>(std::floor(position + 0.5));
		// brake pipe lock in the emergency state, per the .fiz parameters (LPOn, LPOff, HandlePipeUnlockPos, EmergencyCutsOffHandle)
		body.LockPipe = body.PipePress < (body.LockPipe ? body.LockPipeOff : body.LockPipeOn);
		auto const lock{(body.LockPipe && false == body.UnlockPipe && position > body.HandleUnlock) || (body.EmergencyCutsOffHandle && body.EmergencyValveFlow > 0)};
		auto const supply{lock ? body.PipePress + 0.00001 : body.ScndPipePress};
		body.Handle->SetReductor(m_reductor ? get(Frame, port::reductor) : body.BrakeCtrlPos2);
		auto const flow{body.Handle->GetPF(position, body.PipePress, supply, dt, body.EqvtPipePress)};
		if (flow < 0)
		{
			if (body.Compressor > body.ScndPipePress)
			{
				body.CompressedVolume += flow / 1500.0;
				body.Pipe2->Flow(flow / 3.0);
			}
			else
			{
				body.Pipe2->Flow(flow);
			}
		}
		body.Pipe->Flow(-flow);
		// normalised as in UpdatePipePressure, for the sounds of the valve
		body.dpMainValve = flow / (std::max(body.Dim.L, 1.0) * std::max(body.Spg, 0.01) * 20) / (100.0 * dt);
		set_forward(Frame, port::pg, body.PipePress);
		set(Frame, port::p_ctl, body.Handle->GetCP());
	}

	static block_type describe()
	{
		auto type{make_type("br.handle", "brakes", "Driver's brake valve",
		                    "Driver's brake valve (TDriverHandle). pos is the position of the handle in the valve's own scale, e.g. -1..6 for FV4a; emerg moves it to the "
		                    "emergency position. It takes air from the feed pipe of the main reservoir.",
		                    block_origin::simulator)};
		type.ports = {physical_in("zg", domain::air),
		              signal_in("pos", value_type::real),
		              signal_in("emerg", value_type::boolean, 0.0, "", true),
		              signal_in("reductor", value_type::real, 0.0, "bar", true),
		              physical_out("pg", domain::brake_pipe),
		              signal_out("p_ctl", value_type::real, "bar")};
		type.params = {
		    fiz(choice_param("type", {"FV4a", "MHZ_EN57", "MHZ_K5P", "MHZ_K8P", "MHZ_6P", "M394", "Knorr", "St113", "FVel6", "FVE408", "D2", "Westinghouse", "test"}, "FV4a"), "Cntrl.",
		        "BrakeHandle"),
		    fiz(boolean_param("auto_overload", false), "Cntrl.", "HAO"),
		    fiz(boolean_param("manual_overload", false), "Cntrl.", "HMO"),
		    fiz(number_param("gdp1", 0.0), "Cntrl.", "HGDP1"),
		    fiz(number_param("gdp2", 0.0), "Cntrl.", "HGDP2"),
		    fiz(number_param("overload_max", 0.0, "bar"), "Cntrl.", "OMP"),
		    fiz(number_param("overload_decrease", 0.0, "bar/s"), "Cntrl.", "OPD"),
		    fiz(number_param("lock_on", -1.0, "bar"), "Brake:", "LPOn"),
		    fiz(number_param("lock_off", -1.0, "bar"), "Brake:", "LPOff"),
		    fiz(number_param("unlock_pos", -3.0), "Brake:", "HandlePipeUnlockPos"),
		    fiz(boolean_param("emergency_cuts_off", false), "Brake:", "EmergencyCutsOffHandle"),
		};
		type.single = true;
		type.create = make_block<br_handle>;
		return type;
	}

private:
	bool const m_reductor;
};

// independent (local) brake valve
class br_local_handle final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			zg,
			pos,
			bc
		};
	};

	explicit br_local_handle(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const position{std::clamp(get(Frame, port::pos), 0.0, 1.0)};
		body.LocalBrakePosA = position;
		body.dpLocalValve = body.LocHandle->GetPF(position, body.Hamulec->GetBCP(), body.ScndPipePress, Frame.dt, 0);
		body.LocBrakePress = body.LocHandle->GetCP();
		set_forward(Frame, port::bc, body.LocBrakePress);
	}

	static block_type describe()
	{
		auto type{make_type("br.local_handle", "brakes", "Independent brake valve",
		                    "Independent (local) brake valve of the locomotive. pos 0..1 is the position of the handle; bc is the pressure it sets for the distributor.",
		                    block_origin::simulator)};
		type.ports = {physical_in("zg", domain::air), signal_in("pos", value_type::real), physical_out("bc", domain::air)};
		type.params = {fiz(choice_param("type", {"FD1", "Knorr", "Westinghouse"}, "FD1"), "Cntrl.", "LocBrakeHandle")};
		type.single = true;
		type.create = make_block<br_local_handle>;
		return type;
	}
};

// distributor (brake valve) of the vehicle, with its auxiliary reservoir and brake cylinders
class br_distributor final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			pg,
			feed,
			local,
			ed,
			release,
			cyl,
			bc,
			aux,
			crp
		};
	};

	explicit br_distributor(block_setup const &Setup)
	    : block(Setup), m_local(is_connected(Setup, port::local)), m_ed(is_connected(Setup, port::ed)), m_release(is_connected(Setup, port::release))
	{
	}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const dt{Frame.dt};
		auto &brake{*body.Hamulec};
		brake.ForceLeak(dt * body.AirLeakRate * 0.25);
		if (m_local)
		{
			body.LocBrakePress = forward_in(Frame, port::local);
			brake.SetLBP(body.LocBrakePress);
		}
		// load dependent brake force, as in TMoverParameters::UpdatePipePressure
		switch (body.BrakeValve)
		{
		case TBrakeValve::K:
		case TBrakeValve::W:
		case TBrakeValve::KE:
		case TBrakeValve::EStED:
		case TBrakeValve::ESt3AL2:
		case TBrakeValve::NESt3:
		case TBrakeValve::ESt4:
		case TBrakeValve::ESt3:
		{
			brake.PLC(body.MBPM < 2 ? body.MaxBrakePress[body.LoadFlag] : body.TotalMass - body.Mred);
			break;
		}
		case TBrakeValve::EP2:
		case TBrakeValve::EP1:
		{
			brake.PLC(body.TotalMass - body.Mred);
			break;
		}
		default:
		{
			break;
		}
		}
		if (m_ed)
		{
			brake.SetED(std::clamp(get(Frame, port::ed), 0.0, 1.0));
		}
		if (m_release)
		{
			brake.Releaser(get_bool(Frame, port::release) ? 1 : 0);
		}
		body.CntrlPipePress = brake.GetVRP();
		// the brake pipe
		auto const connected{(brake.GetBrakeStatus() & b_dmg) == b_dmg ? 0.0 : 1.0};
		body.Pipe->Flow(connected * brake.GetPF(connected * body.PipePress, dt, body.Vel));
		if (body.ASBType == 128)
		{
			brake.ASB(int(body.SlippingWheels && body.Vel > 1) * (1 + 2 * int(body.nrot_eps < -0.01)));
		}
		// the feed pipe
		body.Pipe2->Flow(brake.GetHPFlow(body.ScndPipePress, dt));

		body.BrakePress = brake.GetBCP();
		body.Volume = brake.GetBRP();
		set_forward(Frame, port::cyl, body.BrakePress);
		set(Frame, port::bc, body.BrakePress);
		set(Frame, port::aux, body.Volume);
		set(Frame, port::crp, brake.GetCRP());
	}

	static block_type describe()
	{
		auto type{make_type("br.distributor", "brakes", "Distributor",
		                    "Distributor of the automatic brake (TBrake), with its auxiliary reservoir and the brake cylinders. local is the pressure of the independent "
		                    "brake valve; ed (0..1) is the share of the electrodynamic brake for the blending of the EStED type; release operates the releaser.",
		                    block_origin::simulator)};
		type.ports = {physical_in("pg", domain::brake_pipe),
		              physical_in("feed", domain::feed_pipe, true),
		              physical_in("local", domain::air, true),
		              signal_in("ed", value_type::real, 0.0, "", true),
		              signal_in("release", value_type::boolean, 0.0, "", true),
		              physical_out("cyl", domain::air),
		              signal_out("bc", value_type::real, "bar"),
		              signal_out("aux", value_type::real, "bar"),
		              signal_out("crp", value_type::real, "bar")};
		type.params = {
		    fiz(choice_param("type", {"W", "W_Lu_L", "W_Lu_XR", "W_Lu_VI", "K", "Kg", "Kp", "Kss", "Kkg", "Kkp", "Kks", "Hikp1", "Hikss", "Hikg1", "KE", "SW", "EStED", "NESt3", "ESt3",
		                                  "LSt", "ESt4", "ESt3AL2", "EP1", "EP2", "M483", "CV1_L_TR", "CV1", "CV1_R"},
		                     "ESt3"),
		        "Brake:", "BrakeValve"),
		    fiz(choice_param("mode", {"G", "P", "R", "GP", "PR", "GPR", "GPR+Mg", "PR+Mg"}, "GP"), "Cntrl.", "BrakeDelays"),
		    fiz(number_param("max_bp", 3.8, "bar"), "Brake:", "MaxBP"),
		    fiz(number_param("max_lbp", 0.0, "bar"), "Brake:", "MaxLBP"),
		    fiz(number_param("tare_max_bp", 0.0, "bar"), "Brake:", "TareMaxBP"),
		    fiz(number_param("max_asbp", 0.0, "bar"), "Brake:", "MaxASBP"),
		    fiz(number_param("cyl_count", 2.0), "Brake:", "BCN"),
		    fiz(number_param("cyl_radius", 0.178, "m"), "Brake:", "BCR"),
		    fiz(number_param("cyl_dist", 0.1, "m"), "Brake:", "BCD"),
		    fiz(number_param("volume", 0.0, "l"), "Brake:", "BVV"),
		    fiz(number_param("rm", 1.0), "Brake:", "RM"),
		    fiz(number_param("rv", 55.0, "km/h"), "Brake:", "RV"),
		    fiz(number_param("valve_size", 0.0), "Brake:", "Size"),
		    fiz(number_param("delay1", 0.0, "s"), "Cntrl.", "BDelay1"),
		    fiz(number_param("delay2", 0.0, "s"), "Cntrl.", "BDelay2"),
		    fiz(number_param("delay3", 0.0, "s"), "Cntrl.", "BDelay3"),
		    fiz(number_param("delay4", 0.0, "s"), "Cntrl.", "BDelay4"),
		    fiz(choice_param("asb", {"none", "manual", "automatic", "yes"}, "none"), "Cntrl.", "ASB"),
		};
		type.single = true;
		type.create = make_block<br_distributor>;
		return type;
	}

private:
	bool const m_local;
	bool const m_ed;
	bool const m_release;
};

// brake rigging: from the cylinder pressure to the brake force at the wheels
class br_rigging final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			bc,
			spring,
			force
		};
	};
	struct param
	{
		enum : std::size_t
		{
			bcm,
			bcs,
			bsa,
			bre,
			nbpa,
			material
		};
	};

	explicit br_rigging(block_setup const &Setup) : block(Setup), m_material(make_friction_material(material_id(text(Setup, param::material)))) {}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		// the cylinders are described by the distributor (BCR, BCN), the rigging by this block; both are in the body
		brake_rigging_setup setup;
		setup.press_to_force = body.P2FTrans;
		setup.cylinder_spring = body.BrakeCylSpring;
		setup.cylinder_mult = body.BrakeCylMult[0];
		setup.slack_adjuster = body.BrakeSlckAdj;
		setup.rigging_efficiency = body.BrakeRigEff;
		setup.cylinder_count = body.BrakeCylNo;
		setup.axles = body.NAxles;
		setup.blocks_per_axle = body.NBpA;
		auto const shoeforce{brake_rigging_shoe_force(setup, std::max(0.0, get(Frame, port::spring)), get(Frame, port::bc), body.Ntotal)};
		auto const friction{m_material->GetFC(shoeforce, body.Vel)};
		body.UnitBrakeForce = friction * shoeforce * 1000.0;
		auto const force{body.UnitBrakeForce * body.NAxles * std::max(1, body.NBpA)};
		Frame.body->brake_force += force;
		set(Frame, port::force, force);
	}

	static block_type describe()
	{
		auto type{make_type("br.rigging", "brakes", "Brake rigging",
		                    "Brake rigging and shoes or discs: the brake force from the cylinder pressure (bc) and the force of the spring brake (spring, kN). The force is "
		                    "applied at the wheels of the vehicle.",
		                    block_origin::extracted)};
		type.ports = {signal_in("bc", value_type::real, 0.0, "bar"), signal_in("spring", value_type::real, 0.0, "kN", true), signal_out("force", value_type::real, "N")};
		type.params = {
		    fiz(number_param("bcm", 1.0), "Brake:", "BCM"),
		    fiz(number_param("bcs", 0.0, "kN"), "Brake:", "BCS"),
		    fiz(number_param("bsa", 0.0, "kN"), "Brake:", "BSA"),
		    fiz(number_param("bre", 1.0), "Brake:", "BRE"),
		    fiz(number_param("nbpa", 1.0), "Brake:", "NBpA"),
		    fiz(choice_param("material", {"P10", "P10-Bg", "P10-Bgu", "P10yBg", "P10yBgu", "FR513", "FR510", "Cosid", "Disk1", "Disk1+Mg", "Disk2"}, "P10"), "Brake:", "BM"),
		};
		type.single = true;
		type.create = make_block<br_rigging>;
		return type;
	}

private:
	static int material_id(std::string const &Material)
	{
		// as in TMoverParameters::LoadFIZ_Brake, without the magnetic track brake
		if (Material == "P10-Bg")
			return bp_P10Bg;
		if (Material == "P10-Bgu")
			return bp_P10Bgu;
		if (Material == "FR513")
			return bp_FR513;
		if (Material == "FR510")
			return bp_FR510;
		if (Material == "Cosid")
			return bp_Cosid;
		if (Material == "P10yBg")
			return bp_P10yBg;
		if (Material == "P10yBgu")
			return bp_P10yBgu;
		if (Material == "Disk1" || Material == "Disk1+Mg")
			return bp_D1;
		if (Material == "Disk2")
			return bp_D2;
		return bp_P10;
	}

	std::shared_ptr<TFricMat> m_material;
};

// spring (parking) brake
class pn_spring_brake final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			release,
			feed,
			force,
			applied
		};
	};

	explicit pn_spring_brake(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto &spring{body.SpringBrake};
		spring.Activate = false == get_bool(Frame, port::release);
		body.UpdateSpringBrake(Frame.dt);
		// the force the spring adds to the shoes, as in TMoverParameters::BrakeForce
		auto const force{spring.IsReady ? std::max(0.0, spring.MinForcePressure - spring.Cylinder->P()) * spring.MaxBrakeForce : 0.0};
		set(Frame, port::force, force);
		set_bool(Frame, port::applied, spring.IsActive);
	}

	static block_type describe()
	{
		auto type{make_type("pn.spring_brake", "brakes", "Spring brake",
		                    "Spring (parking) brake, released with air from the feed pipe. force (kN) is meant for the spring input of the brake rigging.",
		                    block_origin::extracted)};
		type.ports = {signal_in("release", value_type::boolean), physical_in("feed", domain::air, true), signal_out("force", value_type::real, "kN"),
		              signal_out("applied", value_type::boolean)};
		type.params = {
		    fiz(number_param("volume", 1.0, "l"), "SpringBrake:", "Volume"),
		    fiz(number_param("mbf", 0.0, "kN"), "SpringBrake:", "MBF"),
		    fiz(number_param("max_sp", 0.0, "bar"), "SpringBrake:", "MaxSP"),
		    fiz(number_param("reset_p", 0.0, "bar"), "SpringBrake:", "ResetP"),
		    fiz(number_param("min_fp", 0.1, "bar"), "SpringBrake:", "MinFP"),
		    fiz(number_param("press_off", -1.0, "bar"), "SpringBrake:", "PressOff"),
		    fiz(number_param("press_on", -2.0, "bar"), "SpringBrake:", "PressOn"),
		    fiz(number_param("valve_on_area", 0.0), "SpringBrake:", "ValveOnArea"),
		    fiz(number_param("valve_off_area", 0.0), "SpringBrake:", "ValveOffArea"),
		    fiz(number_param("valve_pnb_area", 0.0), "SpringBrake:", "ValvePNBArea"),
		};
		type.single = true;
		type.create = make_block<pn_spring_brake>;
		return type;
	}
};

} // namespace

void register_pneumatic_blocks(catalog &Catalog)
{
	Catalog.add(pn_compressor::describe());
	Catalog.add(pn_governor::describe());
	Catalog.add(pn_reservoir::describe());
	Catalog.add(br_handle::describe());
	Catalog.add(br_local_handle::describe());
	Catalog.add(br_pipe::describe());
	Catalog.add(br_distributor::describe());
	Catalog.add(br_rigging::describe());
	Catalog.add(pn_spring_brake::describe());
}

} // namespace vgraph
