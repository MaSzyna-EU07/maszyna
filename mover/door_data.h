#pragma once

#include "mover/core.h"
#include "mover/basic_door.h"

namespace mover
{
struct door_data
{
	// config
	control_t open_control{control_t::passenger};
	float open_rate{1.f};
	float open_delay{0.f};
	control_t close_control{control_t::passenger};
	float close_rate{1.f};
	float close_delay{0.f};
	int type{2};
	float range{0.f}; // extent of primary move/rotation
	float range_out{0.f}; // extent of shift outward, applicable for plug doors
	int step_type{2};
	float step_rate{0.5f};
	float step_range{0.f};
	bool has_lock{false};
	bool has_warning{false};
	bool has_autowarning{false};
	float auto_duration{-1.f}; // automatic door closure delay period
	float auto_velocity{-1.f}; // automatic door closure velocity threshold
	bool auto_include_remote{false}; // automatic door closure applies also to remote control
	bool permit_needed{false};
	bool closure_signal{false}; // the door closure lamp (SOD) of the cab signals the doors of the whole consist in three states
	std::vector<int> permit_presets; // permit presets selectable with preset switch
	float voltage{0.f}; // power type required for door movement
	// ld inputs
	bool lock_enabled{true};
	bool step_enabled{true};
	bool remote_only{false}; // door ignores local control signals
	// internal data
	int permit_preset{-1}; // curent position of preset selection switch
	// vehicle parts
	std::array<basic_door, 2> instances; // door on the right and left side of the vehicle
	// ld outputs
	bool is_locked{false};
	double doorLockSpeed = 10.0; // predkosc przy ktorej wyzwalana jest blokada drzwi
};

} // namespace mover
