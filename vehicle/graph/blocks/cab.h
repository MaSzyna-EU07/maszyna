/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "vehicle/graph/graph_block.h"

#include <string>
#include <vector>

// cab controls and indicators. the blocks hold the logic and the values; the cab (TTrain) creates the animated
// submodels from their parameters and binds them to the values these blocks expose
namespace vgraph
{

// switch, button or lever of the cab. its behaviour and animation are defined in the graph, and its value
// is available to the other blocks
class cab_switch final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			value, // "pos": value of the current position, between min and max
			index // number of the current position, from 0
		};
	};
	struct param
	{
		enum : std::size_t
		{
			behaviour,
			positions,
			min,
			max,
			default_value,
			spring,
			labels,
			animation,
			step,
			offset,
			smoothing,
			submodel,
			sound_inc,
			sound_dec,
			sound_positions,
			key_inc,
			key_dec,
			command,
			hw_name,
			cab
		};
	};
	enum class behaviour_type
	{
		bistable, // stays where it was put
		monostable, // returns to the default position when released, e.g. a push button
		multi // several positions; with the spring enabled it returns to the default position when released
	};
	enum class animation_type
	{
		rotate,
		move
	};

	explicit cab_switch(block_setup const &Setup);

	void update(frame &Frame) override;
	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "position";
	}
	double state_value(std::size_t) const override
	{
		return m_position;
	}

	// operations of the player, AI or hardware. they take effect in the next update
	void increase();
	void decrease();
	void toggle();
	void press();
	void release();
	void set_position(int Position);
	// moves to the position with the value closest to the specified one, e.g. for hardware levers
	void set_value(double Value);

	int position() const
	{
		return m_position;
	}
	int position_count() const
	{
		return m_positioncount;
	}
	int default_position() const
	{
		return m_defaultposition;
	}
	double value_at(int Position) const;
	behaviour_type behaviour() const
	{
		return m_behaviour;
	}
	bool spring() const
	{
		return m_spring;
	}
	// description of the specified position, from the labels parameter; empty if not specified
	std::string label(int Position) const;
	// animation of the submodel: offset + step * position, in degrees or millimetres
	animation_type animation() const
	{
		return m_animation;
	}
	double step() const
	{
		return m_step;
	}
	double offset() const
	{
		return m_offset;
	}
	double smoothing() const
	{
		return m_smoothing;
	}
	std::string const &submodel() const
	{
		return m_submodel;
	}
	// sounds played by the animated submodel when the position increases or decreases, in the format of
	// soundinc: and sounddec: of the .mmd controls
	std::string const &sound_increase() const
	{
		return m_soundincrease;
	}
	std::string const &sound_decrease() const
	{
		return m_sounddecrease;
	}
	// sounds of single positions, as soundN: entries of the .mmd controls with N the number of the position
	std::string const &sound_positions() const
	{
		return m_soundpositions;
	}
	// bindings: keys, existing simulator command, name for the hardware desk
	std::string const &key_increase() const
	{
		return m_keyincrease;
	}
	std::string const &key_decrease() const
	{
		return m_keydecrease;
	}
	std::string const &command() const
	{
		return m_command;
	}
	std::string const &hardware_name() const
	{
		return m_hardwarename;
	}
	// cab whose model holds the submodel: 0 any, 1 or 2
	int cab() const
	{
		return m_cab;
	}
	// stable address of the value animating the submodel (the position), for TGauge::AssignFloat
	float *display_state()
	{
		return &m_display;
	}

	static block_type describe();

private:
	int clamp_position(int Position) const;

	behaviour_type const m_behaviour;
	int const m_positioncount;
	double const m_min;
	double const m_max;
	int m_defaultposition{0};
	bool const m_spring;
	std::vector<std::string> m_labels;
	animation_type const m_animation;
	double const m_step;
	double const m_offset;
	double const m_smoothing;
	std::string const m_submodel;
	std::string const m_soundincrease;
	std::string const m_sounddecrease;
	std::string const m_soundpositions;
	std::string const m_keyincrease;
	std::string const m_keydecrease;
	std::string const m_command;
	std::string const m_hardwarename;
	int const m_cab;
	int m_position{0};
	float m_display{0.f};
};

// indicator lamp. when the graph simulates low voltage circuits and the supply input is connected, the lamp lights
// only with enough voltage, and draws current from the supply
class cab_lamp final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			on,
			supply
		};
	};
	struct param
	{
		enum : std::size_t
		{
			color,
			blink,
			blink_period,
			min_voltage,
			power,
			submodel,
			sound_on,
			sound_off,
			cab
		};
	};

	explicit cab_lamp(block_setup const &Setup);

	void update(frame &Frame) override;
	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "lit";
	}
	double state_value(std::size_t) const override
	{
		return m_lit ? 1.0 : 0.0;
	}

	// stable address of the state, for TButton::AssignBool
	bool const *lit_state() const
	{
		return &m_lit;
	}
	std::string const &submodel() const
	{
		return m_submodel;
	}
	// sounds played when the lamp lights and goes out, as soundinc: and sounddec: of the .mmd lamps
	std::string const &sound_on() const
	{
		return m_soundon;
	}
	std::string const &sound_off() const
	{
		return m_soundoff;
	}
	int cab() const
	{
		return m_cab;
	}

	static block_type describe();

private:
	bool const m_supplied; // whether the supply input is connected
	bool const m_blink;
	double const m_blinkperiod;
	double const m_minvoltage;
	double const m_power;
	std::string const m_submodel;
	std::string const m_soundon;
	std::string const m_soundoff;
	int const m_cab;
	bool m_lit{false};
};

// cab given by the "cab" parameter of a cab block: 0 any, 1 or 2, -1 the engine room
int cab_of(block_setup const &Setup);

// needle gauge with one or two needles: voltmeter, ammeter, speedometer, pressure gauge
class cab_gauge final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			value,
			value2
		};
	};
	struct param
	{
		enum : std::size_t
		{
			min,
			max,
			unit,
			angle_min,
			angle_max,
			smoothing,
			submodel,
			submodel2
		};
	};

	cab_gauge(block_setup const &Setup, bool Twoneedles);

	void update(frame &Frame) override;
	std::size_t state_count() const override
	{
		return m_twoneedles ? 2 : 1;
	}
	char const *state_name(std::size_t Index) const override
	{
		return Index == 0 ? "value" : "value2";
	}
	double state_value(std::size_t Index) const override
	{
		return m_values[Index];
	}

	// stable addresses of the values shown by the needles, clamped to the scale, for TGauge::AssignFloat
	float *value_state(std::size_t Needle)
	{
		return &m_values[Needle];
	}
	bool two_needles() const
	{
		return m_twoneedles;
	}
	double min() const
	{
		return m_min;
	}
	double max() const
	{
		return m_max;
	}
	double angle_min() const
	{
		return m_anglemin;
	}
	double angle_max() const
	{
		return m_anglemax;
	}
	double smoothing() const
	{
		return m_smoothing;
	}
	std::string const &submodel(std::size_t Needle) const
	{
		return Needle == 0 ? m_submodel : m_submodel2;
	}
	int cab() const
	{
		return m_cab;
	}

	static block_type describe(bool Twoneedles);

private:
	bool const m_twoneedles;
	double const m_min;
	double const m_max;
	double const m_anglemin;
	double const m_anglemax;
	double const m_smoothing;
	std::string const m_submodel;
	std::string const m_submodel2;
	int const m_cab;
	float m_values[2]{0.f, 0.f};
};

// sound of the vehicle or of its cab. sounds of a graph vehicle are defined in the graph instead of the .mmd file
class vehicle_sound final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			play,
			volume,
			pitch
		};
	};
	struct param
	{
		enum : std::size_t
		{
			file,
			volume,
			pitch,
			loop,
			placement,
			range,
			offset_x,
			offset_y,
			offset_z,
			cab
		};
	};
	// the same placements as sound_placement of the audio subsystem
	enum class placement_type
	{
		general,
		internal,
		engine,
		external,
		external_ambient
	};

	explicit vehicle_sound(block_setup const &Setup);

	void update(frame &Frame) override;
	std::size_t state_count() const override
	{
		return 3;
	}
	char const *state_name(std::size_t Index) const override;
	double state_value(std::size_t Index) const override;

	bool playing() const
	{
		return m_playing;
	}
	// volume and pitch: parameter multiplied by the input
	float volume() const
	{
		return m_volume;
	}
	float pitch() const
	{
		return m_pitch;
	}
	std::string const &file() const
	{
		return m_file;
	}
	bool loop() const
	{
		return m_loop;
	}
	placement_type placement() const
	{
		return m_placement;
	}
	double range() const
	{
		return m_range;
	}
	double offset_x() const
	{
		return m_offset[0];
	}
	double offset_y() const
	{
		return m_offset[1];
	}
	double offset_z() const
	{
		return m_offset[2];
	}
	// for the sounds placed in the cab: the cab they belong to, 0 any
	int cab() const
	{
		return m_cab;
	}

	static block_type describe();

private:
	std::string const m_file;
	double const m_basevolume;
	double const m_basepitch;
	bool const m_loop;
	placement_type const m_placement;
	double const m_range;
	double const m_offset[3];
	int const m_cab;
	bool m_playing{false};
	float m_volume{0.f};
	float m_pitch{1.f};
};

// value published for the python screens of the cab (TTrain::GetTrainState), under the specified name
class cab_export final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			value
		};
	};
	enum class kind_type
	{
		real,
		integer,
		boolean
	};

	explicit cab_export(block_setup const &Setup);

	void update(frame &Frame) override;

	std::string const &name() const
	{
		return m_name;
	}
	kind_type kind() const
	{
		return m_kind;
	}
	double value() const
	{
		return m_value;
	}

	static block_type describe();

private:
	std::string const m_name;
	kind_type const m_kind;
	double m_value{0.0};
};

} // namespace vgraph
