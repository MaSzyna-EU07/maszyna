#pragma once

#include <string>
#include <vector>

#include "McZapkie/MOVER.h"

namespace test {

// the key figures of a vehicle's run, for each part of it between two events of the test: the speeds, the
// distance, the time to stop, the brake pipe's lowest pressure and the time it took to get back, the brake
// cylinders', the motor current's and the tractive force's peaks. a golden trace keeps them next to its
// samples, in data/golden/<Name>.figures, which a person reads:
//
//   [emergency]
//   duration = 40.00 s
//   speed at the start = 80.0 km/h
//   speed at the end = 0.0 km/h
//   distance = 412.3 m
//   stopped after = 38.21 s
//   brake cylinders peak = 3.800 bar
//
// a figure the vehicle doesn't have (no speed, no current) is left out of all the parts. they're compared
// within 0.5%, loose enough for any compiler, so a change which alters the behaviour on purpose shows in their
// diff in a few readable lines, where the samples change in thousands
class figures {
public:
	// called on every step of the vehicle
	void sample(double Time, TMoverParameters const &Vehicle);
	// a part of the run starts with the event
	void mark(std::string const &Event);

	// the figures as the lines of the file
	std::vector<std::string> lines() const;
	// the differences of the lines from the recorded ones, beyond the tolerance, each told in a line
	static std::vector<std::string> compare(std::vector<std::string> const &Recorded, std::vector<std::string> const &Now);

private:
	struct part {
		std::string event;
		bool sampled{false};
		double start{0.0};
		double end{0.0};
		double speed_start{0.0};
		double speed_end{0.0};
		double speed_peak{0.0};
		double distance_start{0.0};
		double distance_end{0.0};
		double stopped{-1.0}; // after the start, if it started moving and stopped
		double pipe_lowest{0.0};
		double pipe_end{0.0};
		double pipe_back{-1.0}; // after the start, at 4.9 bar again after a drop under 4.8
		double cylinders_peak{0.0};
		double current_peak{0.0};
		double force_peak{0.0};
	};

	std::vector<part> m_parts{part{"start"}};
	double m_last_time{-1.0};
	double m_last_speed{0.0};
	double m_last_distance{0.0};
	// the figures the vehicle has, shown in every part
	bool m_moved{false};
	bool m_piped{false};
	bool m_braked{false};
	bool m_current{false};
	bool m_force{false};
};

} // namespace test
