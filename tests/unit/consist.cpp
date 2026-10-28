#include "stdafx.h"
#include "consist.h"

#include "vehicle/DynObj.h"

namespace test {

consist::consist() = default;
consist::~consist() = default;

vehicle &consist::add(definition const &Definition, int Cab, int Coupling, double Velocity, double Gap) {
	auto added{std::make_unique<vehicle>(Definition, Velocity, "test" + std::to_string(m_vehicles.size()), Cab)};
	auto &mover{added->mover()};

	auto object{std::make_unique<std::byte[]>(sizeof(TDynamicObject))};
	reinterpret_cast<TDynamicObject *>(object.get())->MoverParameters = &mover; // NOSONAR see m_objects

	auto position{0.0};
	if (false == m_vehicles.empty()) {
		auto &last{m_vehicles.back()->mover()};
		position = m_positions.back() - 0.5 * (last.Dim.L + mover.Dim.L) - Gap;
		mover.Loc = TLocation{position, 0.0, 0.0};
		if (Coupling != coupling::faux) {
			last.Attach(end::rear, end::front, &mover, Coupling, true, false);
			mover.Attach(end::front, end::rear, &last, Coupling, true, false);
		}
	}
	else {
		mover.Loc = TLocation{position, 0.0, 0.0};
	}
	m_vehicles.push_back(std::move(added));
	m_positions.push_back(position);
	m_objects.push_back(std::move(object));
	update_neighbours();
	return *m_vehicles.back();
}

void consist::update_neighbours() {
	// TDynamicObject::update_neighbours(), for vehicles coupled in a line
	for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
		auto &mover{m_vehicles[i]->mover()};
		for (int end = end::front; end <= end::rear; ++end) {
			auto &neighbour{mover.Neighbours[end]};
			auto const other{end == end::front ? static_cast<std::ptrdiff_t>(i) - 1 : static_cast<std::ptrdiff_t>(i) + 1};
			if (other < 0 || other >= static_cast<std::ptrdiff_t>(m_vehicles.size())) {
				neighbour = neighbour_data();
				continue;
			}
			auto &othermover{m_vehicles[other]->mover()};
			auto const otherend{end == end::front ? end::rear : end::front};
			auto const distance{TMoverParameters::CouplerDist(&mover, &othermover) - mover.Couplers[end].adapter_length -
			                    othermover.Couplers[otherend].adapter_length};
			// a vehicle not coupled is a collision source when it's within the range the game scans
			if (mover.Couplers[end].Connected == nullptr && distance > std::max(10.0, std::abs(mover.V)) + 40.0) {
				neighbour = neighbour_data();
				continue;
			}
			neighbour.vehicle = reinterpret_cast<TDynamicObject *>(m_objects[other].get()); // NOSONAR see m_objects
			neighbour.vehicle_end = otherend;
			neighbour.distance = static_cast<float>(distance);
		}
	}
}

void consist::step(track const &Track, double Dt) {
	for (auto &vehicle : m_vehicles) {
		vehicle->begin_step();
	}
	update_neighbours();
	for (int iteration = 1; iteration < Track.iterations; ++iteration) {
		for (auto &vehicle : m_vehicles) {
			vehicle->forces(Dt);
		}
		for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
			m_positions[i] += m_vehicles[i]->fast_move(Track, Dt, TLocation{m_positions[i], 0.0, 0.0});
		}
	}
	for (auto &vehicle : m_vehicles) {
		vehicle->forces(Dt);
	}
	// the average pressure in the train's brake pipe weighed by the vehicles' lengths, TDynamicObject::GetEPP(),
	// over the whole train (the game's stops at a vehicle without the brake hose coupled)
	auto pipe{0.0};
	auto length{0.0};
	for (auto &vehicle : m_vehicles) {
		pipe += vehicle->mover().PipePress * vehicle->mover().Dim.L;
		length += vehicle->mover().Dim.L;
	}
	pipe /= length;
	for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
		m_positions[i] += m_vehicles[i]->move(Track, Dt, TLocation{m_positions[i], 0.0, 0.0}, pipe);
	}
}

void consist::run(double Seconds, track const &Track, double Dt) {
	auto const steps{std::lround(Seconds / (Dt * Track.iterations))};
	for (long i = 0; i < steps; ++i) {
		step(Track, Dt);
	}
}

double consist::gap(std::size_t Index) const {
	return TMoverParameters::CouplerDist(&m_vehicles[Index]->mover(), &m_vehicles[Index + 1]->mover());
}

} // namespace test
