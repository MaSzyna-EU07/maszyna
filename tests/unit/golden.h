#pragma once

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "McZapkie/MOVER.h"
#include "figures.h"
#include "invariants.h"

namespace test {

// one value of the vehicle a trace follows
struct channel {
	std::string name;
	std::function<double(TMoverParameters const &)> value;
};

// sets of channels for the usual subjects of a test, which can be put together
namespace channels {
std::vector<channel> motion(); // speed, distance, forces
std::vector<channel> traction(); // controllers, current, voltage, engine
std::vector<channel> brakes(); // pressures of the brake pipe, cylinders and reservoirs
std::vector<channel> air(); // compressor and main reservoir
std::vector<channel> handles(); // the driver's brake handle and the local brake handle, with their sounds
std::vector<channel> doors(); // both sides' doors and steps, their permits and the lock
std::vector<channel> parameters(); // what the vehicle definition sets, as the loader read it
std::vector<channel> operator+(std::vector<channel> Left, std::vector<channel> const &Right);
} // namespace channels

// characterisation of the simulation: the values of the channels over a run of a vehicle, compared with the
// ones the code gave when the trace got recorded, in tests/unit/data/golden/<Name>.csv (spaces as underscores). a difference fails the
// test, naming the channel and the time it shows up first. the samples are tens of megabytes and only mean
// something against the code before a change, so they stay out of the repository: running the tests with
// EU07_UNIT_TESTS_UPDATE_GOLDEN=1 records them on the code before the change, and after it they're compared.
// a trace with no samples recorded compares only its key figures.
//
// values are compared within a relative tolerance of 1e-9 (the CI builds with GCC and glibc, whose maths
// functions can differ from MSVC's in the last bits), or exactly with EU07_UNIT_TESTS_GOLDEN_EXACT=1, which is the check to
// run for a change which shouldn't alter anything at all, and which fails a trace with no samples recorded.
//
// besides the samples, a trace keeps the key figures of the run (figures.h) in <Name>.figures, in the
// repository, and checks the invariants (invariants.h) on every step
//
//   test::trace trace{"eu07 notch up", test::channels::motion() + test::channels::traction()};
//   eu07.record(trace);
//   ... run the vehicle ...
//   trace.check();
class trace {
public:
	// samples every Every-th step of the vehicle: 100, a second at the usual step, is plenty for a run of minutes,
	// as a difference stays once the simulation has taken it in
	// Figures: whether to keep the key figures, which a trace sampled by hand, at times of its own, has no use for
	trace(std::string Name, std::vector<channel> Channels, int Every = 10, bool Figures = true);

	// called by the vehicle after every step
	void sample(double Time, TMoverParameters const &Vehicle);
	// adds an event of the test to the trace, so a difference can be told apart from the moment of the action
	void mark(std::string const &Event);

	// compares the trace and its key figures (figures.h) with the recorded ones, or records them, and reports the
	// invariants broken
	void check() const;
	// the invariants broken in the trace (see invariants.h), the first time each, excused or not
	std::vector<std::string> const &violations() const { return m_violations; }

private:
	void check_figures(std::filesystem::path const &Path) const;

	std::string m_name;
	std::vector<channel> m_channels;
	int m_every;
	long m_step{0};
	std::vector<std::string> m_rows; // formatted, as in the file
	std::string m_event; // marked since the last row
	std::string m_last_event; // the last one marked, for the invariants broken after it
	snapshot m_before{};
	bool m_sampled{false};
	std::vector<std::string> m_broken; // the names of the invariants broken so far
	std::vector<std::string> m_violations;
	bool m_keep_figures;
	figures m_figures;
};

} // namespace test
