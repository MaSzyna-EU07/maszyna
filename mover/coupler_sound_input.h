#pragma once

namespace mover::detail
{
struct coupler_sound_input
{
	double gap{0.0}; // between the ends [m]
	double previous_gap{0.0};
	double gap_rate{0.0}; // [m/s]
	double draw_gear{0.0}; // deflection [m]
	double previous_draw_gear{0.0};
	bool coupled{false};
	bool adapter{false};
	bool moving{false};
};

} // namespace mover::detail
