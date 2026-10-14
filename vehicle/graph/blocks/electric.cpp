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

// high voltage circuit and auxiliary supplies. voltage goes forward through the connections, current comes back; the
// back channel lags one step, so a breaker sees the current its load drew in the previous step
namespace vgraph
{

namespace
{

TMoverParameters &vehicle(frame const &Frame)
{
	return *Frame.body->mover;
}

block_type make_type(std::string Id, std::string Name, std::string Description, block_origin Origin)
{
	block_type type;
	type.id = std::move(Id);
	type.category = "electric";
	type.name = std::move(Name);
	type.description = std::move(Description);
	type.origin = Origin;
	return type;
}

// current collector. in the simulator its contact with the catenary is the pantograph of the vehicle model
class el_pantograph final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			raise,
			air,
			wn,
			up,
			height,
			u
		};
	};
	struct param
	{
		enum : std::size_t
		{
			index,
			raise_time,
			lower_time,
			min_pressure,
			preview_voltage
		};
	};

	explicit el_pantograph(block_setup const &Setup)
	    : block(Setup), m_air(is_connected(Setup, port::air)), m_index(std::clamp(static_cast<int>(std::lround(number(Setup, param::index))), 0, 1)),
	      m_raisetime(std::max(0.01, number(Setup, param::raise_time))), m_lowertime(std::max(0.01, number(Setup, param::lower_time))),
	      m_minpressure(number(Setup, param::min_pressure)), m_previewvoltage(number(Setup, param::preview_voltage))
	{
	}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const pressure{m_air ? forward_in(Frame, port::air) : body.EnginePowerSource.CollectorParameters.MaxPress};
		auto const raise{get_bool(Frame, port::raise) && pressure >= m_minpressure};
		auto &pantograph{body.Pantographs[m_index]};
		pantograph.is_active = raise;
		// the model raises the pantograph at a speed depending on this pressure
		body.PantPress = pressure;
		m_height = std::clamp(m_height + (raise ? Frame.dt / m_raisetime : -Frame.dt / m_lowertime), 0.0, 1.0);
		double voltage;
		if (Frame.body->standalone)
		{
			voltage = m_height >= 1.0 ? m_previewvoltage : 0.0;
		}
		else
		{
			voltage = std::abs(m_index == 0 ? body.PantFrontVolt : body.PantRearVolt);
		}
		pantograph.voltage = voltage;
		set_forward(Frame, port::wn, voltage);
		// current drawn by the vehicle through this pantograph, in the previous step
		Frame.body->current += back_out(Frame, port::wn);
		set_bool(Frame, port::up, m_height >= 1.0);
		set(Frame, port::height, m_height);
		set(Frame, port::u, voltage);
	}

	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "height";
	}
	double state_value(std::size_t) const override
	{
		return m_height;
	}

	static block_type describe()
	{
		auto type{make_type("el.pantograph", "Pantograph",
		                    "Current collector: index 0 is the front pantograph of the vehicle model, 1 the rear one. Raising needs the air pressure (when connected). "
		                    "Without the simulator the catenary has the preview voltage.",
		                    block_origin::extracted)};
		type.ports = {signal_in("raise", value_type::boolean),          physical_in("air", domain::air, true), physical_out("wn", domain::high_voltage),
		              signal_out("up", value_type::boolean),            signal_out("height", value_type::real), signal_out("u", value_type::real, "V")};
		type.params = {number_param("index", 0.0), number_param("raise_time", 4.0, "s"), number_param("lower_time", 1.0, "s"), number_param("min_pressure", 3.5, "bar"),
		               number_param("preview_voltage", 3000.0, "V")};
		type.create = make_block<el_pantograph>;
		return type;
	}

private:
	bool const m_air;
	int const m_index;
	double const m_raisetime;
	double const m_lowertime;
	double const m_minpressure;
	double const m_previewvoltage;
	double m_height{0.0};
};

// joins two supplies: the higher voltage wins, the current goes back to the source(s) supplying it
class el_junction final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			a,
			b,
			out
		};
	};

	explicit el_junction(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		auto const a{forward_in(Frame, port::a)};
		auto const b{forward_in(Frame, port::b)};
		set_forward(Frame, port::out, std::max(a, b));
		auto const current{back_out(Frame, port::out)};
		if (std::abs(a - b) < 1.0)
		{
			add_back(Frame, port::a, 0.5 * current);
			add_back(Frame, port::b, 0.5 * current);
		}
		else
		{
			add_back(Frame, a > b ? port::a : port::b, current);
		}
	}

	static block_type describe()
	{
		auto type{make_type("el.junction", "Junction", "Joins two high voltage supplies, e.g. two pantographs.", block_origin::primitive)};
		type.ports = {physical_in("a", domain::high_voltage), physical_in("b", domain::high_voltage, true), physical_out("out", domain::high_voltage)};
		type.create = make_block<el_junction>;
		return type;
	}
};

// line breaker (WS). its state is the Mains of the vehicle body, which the sounds and the AI watch; with more breakers
// (multi-system vehicles) the Mains is closed when any of them is
class el_hsb final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			close,
			open,
			hold,
			out,
			closed,
			i,
			tripped
		};
	};
	struct param
	{
		enum : std::size_t
		{
			trip_current,
			close_time,
			min_voltage,
			init_time
		};
	};

	explicit el_hsb(block_setup const &Setup)
	    : block(Setup), m_tripcurrent(number(Setup, param::trip_current)), m_closetime(number(Setup, param::close_time)), m_minvoltage(number(Setup, param::min_voltage)),
	      m_inittime(number(Setup, param::init_time)), m_initcountdown(m_inittime)
	{
	}

	void update(frame &Frame) override
	{
		m_voltage = forward_in(Frame, port::in);
		m_current = back_out(Frame, port::out);
		// the current of the previous step passes to the supply
		add_back(Frame, port::in, m_current);
		auto const voltageok{m_voltage > 0.0 && m_voltage >= m_minvoltage};
		// the breaker can be closed after the voltage was present for the init time
		m_initcountdown = voltageok ? m_initcountdown - Frame.dt : m_inittime;
		if (m_closed)
		{
			auto const overload{m_tripcurrent > 0.0 && std::abs(m_current) > m_tripcurrent};
			if (overload || false == voltageok || get_bool(Frame, port::open) || false == get_bool(Frame, port::hold))
			{
				m_closed = false;
				m_tripped = m_tripped || overload;
			}
		}
		else if (get_bool(Frame, port::close) && voltageok && m_initcountdown <= 0.0 && get_bool(Frame, port::hold) && false == get_bool(Frame, port::open))
		{
			m_closing += Frame.dt;
			if (m_closing >= m_closetime)
			{
				m_closed = true;
				m_tripped = false;
				m_closing = 0.0;
			}
		}
		else
		{
			m_closing = 0.0;
		}
		// the vehicle can have more breakers, e.g. one for each supply system; its Mains is closed when any of them is
		vehicle(Frame).Mains = Frame.body->mains.any(Frame.step, m_closed);
	}
	void commit(frame &Frame) override
	{
		set_forward(Frame, port::out, m_closed ? m_voltage : 0.0);
		set_bool(Frame, port::closed, m_closed);
		set(Frame, port::i, m_current);
		set_bool(Frame, port::tripped, m_tripped);
	}
	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "closed";
	}
	double state_value(std::size_t) const override
	{
		return m_closed ? 1.0 : 0.0;
	}

	static block_type describe()
	{
		auto type{make_type("el.hsb", "Line breaker",
		                    "Main (line) breaker. Closes after close is held for close_time, with enough voltage present for init_time; opens with open, without hold, "
		                    "below min_voltage or above trip_current (then tripped is set until the next closing).",
		                    block_origin::extracted)};
		type.ports = {physical_in("in", domain::high_voltage),
		              signal_in("close", value_type::boolean),
		              signal_in("open", value_type::boolean, 0.0, "", true),
		              signal_in("hold", value_type::boolean, 1.0, "", true),
		              physical_out("out", domain::high_voltage),
		              signal_out("closed", value_type::boolean),
		              signal_out("i", value_type::real, "A"),
		              signal_out("tripped", value_type::boolean)};
		type.params = {number_param("trip_current", 0.0, "A"), number_param("close_time", 0.2, "s"), number_param("min_voltage", 0.0, "V"), number_param("init_time", 0.0, "s")};
		type.deferred = true;
		type.create = make_block<el_hsb>;
		return type;
	}

private:
	double const m_tripcurrent;
	double const m_closetime;
	double const m_minvoltage;
	double const m_inittime;
	double m_initcountdown;
	double m_closing{0.0};
	double m_voltage{0.0};
	double m_current{0.0};
	bool m_closed{false};
	bool m_tripped{false};
};

class el_contactor final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			on,
			out,
			closed
		};
	};

	explicit el_contactor(block_setup const &Setup) : block(Setup), m_delay(number(Setup, 0)) {}

	void update(frame &Frame) override
	{
		m_voltage = forward_in(Frame, port::in);
		add_back(Frame, port::in, back_out(Frame, port::out));
		if (get_bool(Frame, port::on))
		{
			m_timer += Frame.dt;
			m_closed = m_timer >= m_delay;
		}
		else
		{
			m_timer = 0.0;
			m_closed = false;
		}
	}
	void commit(frame &Frame) override
	{
		set_forward(Frame, port::out, m_closed ? m_voltage : 0.0);
		set_bool(Frame, port::closed, m_closed);
	}

	static block_type describe()
	{
		auto type{make_type("el.contactor", "Contactor", "Connects the output to the input after on was held for the delay; disconnects at once.", block_origin::primitive)};
		type.ports = {physical_in("in", domain::high_voltage), signal_in("on", value_type::boolean), physical_out("out", domain::high_voltage),
		              signal_out("closed", value_type::boolean)};
		type.params = {number_param("delay", 0.0, "s")};
		type.deferred = true;
		type.create = make_block<el_contactor>;
		return type;
	}

private:
	double const m_delay;
	double m_timer{0.0};
	double m_voltage{0.0};
	bool m_closed{false};
};

// static converter or motor generator: auxiliary supply from the high voltage, and optionally the onboard low voltage
// (charger). its state is the ConverterFlag of the body; with more converters the flag is set when any of them runs
class el_converter final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			wn,
			on,
			aux,
			run,
			nn
		};
	};
	struct param
	{
		enum : std::size_t
		{
			power,
			start_time,
			min_voltage,
			out_voltage,
			nn_voltage,
			aux_kind
		};
	};

	explicit el_converter(block_setup const &Setup)
	    : block(Setup), m_idlepower(number(Setup, param::power) * 0.05), m_starttime(number(Setup, param::start_time)), m_minvoltage(number(Setup, param::min_voltage)),
	      m_outvoltage(number(Setup, param::out_voltage)), m_nnvoltage(std::max(0.0, number(Setup, param::nn_voltage)))
	{
	}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const voltage{forward_in(Frame, port::wn)};
		auto const on{get_bool(Frame, port::on)};
		if (on && voltage > 0.0 && voltage >= m_minvoltage)
		{
			m_timer += Frame.dt;
		}
		else
		{
			m_timer = 0.0;
		}
		auto const run{on && voltage > 0.0 && voltage >= m_minvoltage && m_timer >= m_starttime};
		body.ConverterAllow = Frame.body->converter_allow.any(Frame.step, on);
		body.ConverterFlag = Frame.body->converter.any(Frame.step, run);
		set_forward(Frame, port::aux, run ? m_outvoltage : 0.0);
		set_forward(Frame, port::nn, run ? m_nnvoltage : 0.0);
		if (run)
		{
			// power taken by the auxiliaries and the low voltage loads in the previous step, and the losses
			auto const nnpower{m_nnvoltage * std::max(0.0, back_out(Frame, port::nn))};
			add_back(Frame, port::wn, (back_out(Frame, port::aux) + nnpower + m_idlepower) / voltage);
		}
		set_bool(Frame, port::run, run);
	}

	static block_type describe()
	{
		auto type{make_type("el.converter", "Converter",
		                    "Converter of the auxiliary supply. Runs start_time after it was switched on with enough voltage; draws the power of its loads from the high "
		                    "voltage. nn is the onboard low voltage made by its charger (off when nn_voltage is 0). aux_kind only names the auxiliary circuit.",
		                    block_origin::extracted)};
		type.ports = {physical_in("wn", domain::high_voltage), signal_in("on", value_type::boolean), physical_out("aux", domain::aux_ac), signal_out("run", value_type::boolean),
		              physical_out("nn", domain::low_voltage)};
		// aux_kind describes the auxiliary circuit for the editor (three phase, single phase or direct current); the simulator ignores it
		type.params = {number_param("power", 50000.0, "W"),   number_param("start_time", 2.0, "s"), number_param("min_voltage", 0.0, "V"),
		               number_param("out_voltage", 400.0, "V"), number_param("nn_voltage", 0.0, "V"), choice_param("aux_kind", {"ac3", "ac1", "dc"}, "ac3")};
		type.create = make_block<el_converter>;
		return type;
	}

private:
	double const m_idlepower;
	double const m_starttime;
	double const m_minvoltage;
	double const m_outvoltage;
	double const m_nnvoltage;
	double m_timer{0.0};
};

// voltage relay: tells whether the supply is within a range, e.g. to recognize the supply system of a multi-system
// vehicle. the simulator gives the vehicles only the voltage of the catenary, so the systems are told apart by it
class el_voltage_relay final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			ok,
			u
		};
	};
	struct param
	{
		enum : std::size_t
		{
			min_voltage,
			max_voltage,
			delay
		};
	};

	explicit el_voltage_relay(block_setup const &Setup)
	    : block(Setup), m_minvoltage(number(Setup, param::min_voltage)), m_maxvoltage(number(Setup, param::max_voltage)), m_delay(number(Setup, param::delay))
	{
	}

	void update(frame &Frame) override
	{
		// the relay draws no current
		auto const voltage{forward_in(Frame, port::in)};
		auto const inrange{voltage > 0.0 && voltage >= m_minvoltage && (m_maxvoltage <= 0.0 || voltage <= m_maxvoltage)};
		m_timer = inrange ? m_timer + Frame.dt : 0.0;
		set_bool(Frame, port::ok, inrange && m_timer >= m_delay);
		set(Frame, port::u, voltage);
	}

	static block_type describe()
	{
		auto type{make_type("el.voltage_relay", "Voltage relay",
		                    "ok is set when the voltage was within min_voltage and max_voltage (0: no upper limit) for the delay. Draws no current; used e.g. to recognize "
		                    "the supply system.",
		                    block_origin::primitive)};
		type.ports = {physical_in("in", domain::high_voltage), signal_out("ok", value_type::boolean), signal_out("u", value_type::real, "V")};
		type.params = {number_param("min_voltage", 2200.0, "V"), number_param("max_voltage", 4000.0, "V"), number_param("delay", 0.5, "s")};
		type.create = make_block<el_voltage_relay>;
		return type;
	}

private:
	double const m_minvoltage;
	double const m_maxvoltage;
	double const m_delay;
	double m_timer{0.0};
};

// main transformer of an ac vehicle, or of the ac branch of a multi-system one, with the rectifier behind it implied.
// the output voltage is the input voltage times the ratio; the current drawn from the input follows from the power
class el_transformer final : public block
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
	struct param
	{
		enum : std::size_t
		{
			ratio,
			eff
		};
	};

	explicit el_transformer(block_setup const &Setup)
	    : block(Setup), m_ratio(std::max(0.0, number(Setup, param::ratio))), m_efficiency(std::clamp(number(Setup, param::eff), 0.01, 1.0))
	{
	}

	void update(frame &Frame) override
	{
		set_forward(Frame, port::out, forward_in(Frame, port::in) * m_ratio);
		add_back(Frame, port::in, back_out(Frame, port::out) * m_ratio / m_efficiency);
	}

	static block_type describe()
	{
		auto type{make_type("el.transformer", "Transformer",
		                    "Output voltage is the input voltage times the ratio (out/in); the input current is the output current times ratio / eff. For ac vehicles "
		                    "and the ac branches of multi-system vehicles, with the rectifier implied.",
		                    block_origin::primitive)};
		type.ports = {physical_in("in", domain::high_voltage), physical_out("out", domain::high_voltage)};
		type.params = {number_param("ratio", 0.12, ""), number_param("eff", 0.97, "")};
		type.create = make_block<el_transformer>;
		return type;
	}

private:
	double const m_ratio;
	double const m_efficiency;
};

// battery of the low voltage circuits; its switch is the Battery of the body
class el_battery final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			on,
			charge,
			nn,
			ok,
			u
		};
	};
	struct param
	{
		enum : std::size_t
		{
			voltage,
			capacity,
			min_voltage
		};
	};

	explicit el_battery(block_setup const &Setup)
	    : block(Setup), m_charger(is_connected(Setup, port::charge)), m_voltage(number(Setup, param::voltage)), m_capacity(std::max(0.1, number(Setup, param::capacity))),
	      m_minvoltage(number(Setup, param::min_voltage)), m_charge(m_capacity)
	{
	}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const on{get_bool(Frame, port::on)};
		if (on)
		{
			// current of the loads in the previous step, A
			m_charge -= std::max(0.0, back_out(Frame, port::nn)) * Frame.dt / 3600.0;
		}
		if (m_charger && forward_in(Frame, port::charge) > 0.0)
		{
			// charged at a tenth of the capacity per hour
			m_charge += 0.1 * m_capacity * Frame.dt / 3600.0;
		}
		m_charge = std::clamp(m_charge, 0.0, m_capacity);
		// open circuit voltage drops with the charge
		auto const voltage{on ? m_voltage * (0.85 + 0.15 * m_charge / m_capacity) : 0.0};
		body.Battery = on;
		body.BatteryVoltage = voltage;
		set_forward(Frame, port::nn, voltage);
		set_bool(Frame, port::ok, on && voltage >= m_minvoltage);
		set(Frame, port::u, voltage);
	}
	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "charge";
	}
	double state_value(std::size_t) const override
	{
		return m_charge;
	}

	static block_type describe()
	{
		auto type{make_type("el.battery", "Battery",
		                    "Battery of the low voltage circuits. ok tells whether its voltage is above min_voltage; it's charged when the charge input has voltage.",
		                    block_origin::extracted)};
		type.ports = {signal_in("on", value_type::boolean), physical_in("charge", domain::aux_ac, true), physical_out("nn", domain::low_voltage),
		              signal_out("ok", value_type::boolean), signal_out("u", value_type::real, "V")};
		type.params = {number_param("voltage", 110.0, "V"), number_param("capacity", 100.0, "Ah"), number_param("min_voltage", 80.0, "V")};
		type.single = true;
		type.create = make_block<el_battery>;
		return type;
	}

private:
	bool const m_charger;
	double const m_voltage;
	double const m_capacity;
	double const m_minvoltage;
	double m_charge;
};

} // namespace

void register_electric_blocks(catalog &Catalog)
{
	Catalog.add(el_pantograph::describe());
	Catalog.add(el_junction::describe());
	Catalog.add(el_hsb::describe());
	Catalog.add(el_contactor::describe());
	Catalog.add(el_converter::describe());
	Catalog.add(el_battery::describe());
	Catalog.add(el_voltage_relay::describe());
	Catalog.add(el_transformer::describe());
}

} // namespace vgraph
