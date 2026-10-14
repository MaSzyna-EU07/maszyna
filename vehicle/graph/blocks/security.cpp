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

// vigilance device and cab signalling (CA/SHP). the block drives the TSecuritySystem of the vehicle body, configured by
// TSecuritySystem::load from the parameters of the block, so the simulator's own uses of it (cab signal magnets of the
// track, the reset commands of the cab, the AI) keep working with graph vehicles
namespace vgraph
{

namespace
{

class sec_ca final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			v,
			power,
			reset,
			reset_shp,
			active,
			lamp,
			buzzer,
			brake,
			shp_lamp
		};
	};
	struct param
	{
		enum : std::size_t
		{
			aware,
			cab_signal,
			separate_ack,
			sifa,
			aware_delay,
			sound_delay,
			emergency_delay,
			min_speed,
			max_hold_time,
			radio_stop,
			cab_dependent
		};
	};

	explicit sec_ca(block_setup const &Setup) : block(Setup), m_separate(boolean(Setup, param::separate_ack)) {}

	void update(frame &Frame) override
	{
		auto &security{Frame.body->mover->SecuritySystem};
		auto const active{get_bool(Frame, port::active)};
		if (m_first || active != m_active)
		{
			security.set_enabled(active);
			m_active = active;
			m_first = false;
		}
		auto const reset{get_bool(Frame, port::reset)};
		if (reset && false == m_reset)
		{
			security.acknowledge_press();
		}
		else if (false == reset && m_reset)
		{
			security.acknowledge_release();
		}
		m_reset = reset;
		auto const resetshp{get_bool(Frame, port::reset_shp)};
		if (resetshp && false == m_resetshp)
		{
			security.cabsignal_reset();
		}
		m_resetshp = resetshp;

		security.update(Frame.dt, get(Frame, port::v), get_bool(Frame, port::power), Frame.body->cab);

		set_bool(Frame, port::lamp, m_separate ? security.is_vigilance_blinking() : security.is_blinking());
		set_bool(Frame, port::buzzer, security.is_beeping() || security.is_cabsignal_beeping());
		set_bool(Frame, port::brake, security.is_braking());
		set_bool(Frame, port::shp_lamp, security.is_cabsignal_blinking());
	}

	static block_type describe()
	{
		block_type type;
		type.id = "sec.ca";
		type.category = "security";
		type.name = "Vigilance device (CA/SHP)";
		type.description = "Vigilance device and cab signalling (TSecuritySystem). v in km/h; brake is meant for the vent input of the brake pipe. The cab signal is "
		                   "set by the magnets of the track, as for the .fiz vehicles.";
		type.origin = block_origin::simulator;
		type.ports = {signal_in("v", value_type::real, 0.0, "km/h"),
		              signal_in("power", value_type::boolean),
		              signal_in("reset", value_type::boolean),
		              signal_in("reset_shp", value_type::boolean, 0.0, "", true),
		              signal_in("active", value_type::boolean, 1.0, "", true),
		              signal_out("lamp", value_type::boolean),
		              signal_out("buzzer", value_type::boolean),
		              signal_out("brake", value_type::boolean),
		              signal_out("shp_lamp", value_type::boolean)};
		// the first four make AwareSystem; a negative min_speed leaves the default of 10% of vmax
		type.params = {
		    boolean_param("aware", true),
		    boolean_param("cab_signal", false),
		    boolean_param("separate_ack", false),
		    boolean_param("sifa", false),
		    fiz(number_param("aware_delay", 30.0, "s"), "Security:", "AwareDelay"),
		    fiz(number_param("sound_delay", 5.0, "s"), "Security:", "SoundSignalDelay"),
		    fiz(number_param("emergency_delay", 5.0, "s"), "Security:", "EmergencyBrakeDelay"),
		    number_param("min_speed", -1.0, "km/h"),
		    fiz(number_param("max_hold_time", 1.5, "s"), "Security:", "MaxHoldTime"),
		    fiz(boolean_param("radio_stop", false), "Security:", "RadioStop"),
		    fiz(boolean_param("cab_dependent", false), "Security:", "CabDependent"),
		};
		type.single = true;
		type.create = make_block<sec_ca>;
		return type;
	}

private:
	bool const m_separate;
	bool m_first{true};
	bool m_active{false};
	bool m_reset{false};
	bool m_resetshp{false};
};

} // namespace

void register_security_blocks(catalog &Catalog)
{
	Catalog.add(sec_ca::describe());
}

} // namespace vgraph
