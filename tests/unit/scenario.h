#pragma once

#include <memory>
#include <string>
#include <vector>

#include "consist.h"
#include "drive.h"
#include "golden.h"

namespace test {

// a scenario of a test: a train of real vehicles (one vehicle is a train too), a driver in the front one,
// and a golden trace for each vehicle (data/golden/<Name>/<number of the vehicle>.csv) with the events of
// the scenario in them:
//   test::scenario scenario{"doors/elf", test::channels::motion() + test::channels::doors()};
//   scenario.add("elf_v1/21we-a", 1).add("elf_v1/21we-b").add("elf_v1/21we-c");
//   scenario.start_up();
//   scenario.act("open left");
//   scenario.cab().OperateDoors(side::left, true);
//   scenario.wait(10.0);
//   scenario.check();
class scenario {
public:
	scenario(std::string Name, std::vector<channel> Channels, track Track = {0.15, 0.0, catenary_3kv});

	// adds a real vehicle at the back, see consist::add()
	scenario &add(std::string const &Path, int Cab = 0, int Coupling = consist::full_coupling, double Velocity = 0.0, double Gap = 0.0);

	consist &train() { return m_train; }
	TMoverParameters &vehicle(std::size_t Index) { return m_train[Index].mover(); }
	TMoverParameters &cab() { return m_train[0].mover(); }
	driven driver() { return driving(m_train); }
	track &line() { return m_track; }

	// marks the event in the traces, the action follows it
	void act(std::string const &Event);
	// runs the train with the driver attending to it
	void wait(double Seconds);
	// gets the train ready to run, see test::start_up(); returns whether it did
	bool start_up();
	// releases the brakes and sets the direction forward, for a train to move off
	void get_going();
	// checks the traces
	void check();

private:
	void record();

	std::string m_name;
	std::vector<channel> m_channels;
	track m_track;
	consist m_train;
	std::vector<std::unique_ptr<trace>> m_traces;
};

} // namespace test
