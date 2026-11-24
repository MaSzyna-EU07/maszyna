#pragma once

#include "mover/core.h"

namespace mover
{
struct basic_door
{
	// config
	// ld inputs
	bool open_permit{false}; // door can be opened
	bool local_open{false}; // local attempt to open the door
	bool local_close{false}; // local attempt to close the door
	bool remote_open{false}; // received remote signal to open the door
	bool remote_close{false}; // received remote signal to close the door
	// internal data
	float auto_timer{-1.f}; // delay between activation of open state and closing state for automatic doors
	float close_delay{0.f}; // delay between activation of closing state and actual closing
	float open_delay{0.f}; // delay between activation of opening state and actual opening
	float position{0.f}; // current shift of the door from the closed position
	float step_position{0.f}; // current shift of the movable step from the retracted position
	// ld outputs
	bool is_closed{true}; // the door is fully closed
	bool is_door_closed{true}; // the door is fully closed, step doesn't matter
	bool is_closing{false}; // the door is currently closing
	bool is_opening{false}; // the door is currently opening
	bool is_open{false}; // the door is fully open
	bool step_folding{false}; // the doorstep is currently closing
	bool step_unfolding{false}; // the doorstep is currently opening
};

} // namespace mover
