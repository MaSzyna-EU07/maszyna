#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "vehicle.h"

class TDynamicObject;

namespace test {

// vehicles coupled into a train on a straight track, all facing forward, from the front one back, stepped the
// way vehicle_table::update() steps the vehicles of the game: the constants and neighbours of all of them,
// their forces, then their movement. couplers carry the forces, the air, the control and the power between
// the vehicles as in the game:
//   test::consist en57;
//   en57.add(definition::fiz("en57_v1/5bgb"), 1); // driven from the front cab
//   en57.add(definition::fiz("en57_v1/6ba_840"));
//   en57.add(definition::fiz("en57_v1/5bgb"));
class consist {
public:
	// everything a coupler can join, as two vehicles of a train coupled by a crew
	static constexpr int full_coupling{coupling::coupler | coupling::brakehose | coupling::mainhose | coupling::control | coupling::gangway | coupling::heating |
	                                   coupling::permanent | coupling::power110v | coupling::power24v | coupling::highvoltage};

	consist();
	~consist();
	consist(consist const &) = delete;
	consist &operator=(consist const &) = delete;

	// a vehicle at the back of the train, its front coupler joined to the rear one of the last vehicle with
	// the given coupling (coupling flags), Gap [m] between their buffers. Cab as for a vehicle. a vehicle
	// with no coupling (coupling::faux) is free on the same track, as one about to be coupled or hit
	vehicle &add(definition const &Definition, int Cab = 0, int Coupling = full_coupling, double Velocity = 0.0, double Gap = 0.0);

	vehicle &operator[](std::size_t Index) { return *m_vehicles[Index]; }
	std::size_t size() const { return m_vehicles.size(); }

	void step(track const &Track = {}, double Dt = 0.01);
	void run(double Seconds, track const &Track = {}, double Dt = 0.01);

	// distance between the couplers of the vehicle and the one behind it [m], negative when the buffers press
	double gap(std::size_t Index) const;

private:
	void update_neighbours();

	std::vector<std::unique_ptr<vehicle>> m_vehicles;
	std::vector<double> m_positions; // of the vehicles' middles along the track
	// the physics reaches a neighbouring vehicle through its TDynamicObject, for its MoverParameters alone.
	// the game's object can't be made without the rest of the game, so it's a stand-in: storage of its size
	// with that one member set, never constructed, which only the physics ever looks into
	std::vector<std::unique_ptr<std::byte[]>> m_objects;
};

} // namespace test
