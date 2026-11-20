/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <vector>

#include "vehicle/cameraviews.h"

class TCamera;
class TDynamicObject;

// The outside views of the user's train that go with one of its vehicles (eu07/maszyna#551), after the four views
// [Shift]+[F4] had before: the orbit around a vehicle, the shunting view from above, the view beside a coupling
// and the trackside view of a railfan. [ and ] pick the vehicle (the coupling) the view goes with. The camera
// follows from cameraviews.h; this class hands over the vehicles of the train and keeps what the user set.
class consist_views {

public:
	enum class kind { orbit, shunting, coupling, trackside };

	// starts the View of the train of the Driven vehicle, with the vehicle (or the coupling) picked before when it's
	// still in the train; returns the vehicle the camera goes with, nullptr when there's no train
	TDynamicObject *enter(kind View, TDynamicObject *Driven);
	// picks the vehicle Step places towards the rear of the train (towards its front for a negative Step); the
	// coupling in the coupling view
	void select(int Step);
	// places the Camera for this frame of Seconds (real time): the movement keys come from its Velocity, the mouse of
	// the orbit from its Angle. returns the vehicle the camera goes with, nullptr when the train is gone
	TDynamicObject *update(TCamera &Camera, TDynamicObject *Driven, double Seconds);

private:
	struct coupling_place {
		TDynamicObject *vehicle{nullptr}; // the vehicle the coupler is on
		cameraviews::coupler coupler;
		bool other{false};
		glm::dvec3 otherposition{}; // the facing coupler of the vehicle beyond, coupled or coming closer
		double gap{-1.0}; // at an end of the train: the distance to a vehicle coming closer, negative for none
	};

	// the vehicles of the train of the Driven vehicle, from its front to its rear
	static std::vector<TDynamicObject *> train_of(TDynamicObject *Driven);
	// the vehicle the camera goes with in the Train: the one picked before, or the Driven one
	TDynamicObject *pick(std::vector<TDynamicObject *> const &Train, TDynamicObject *Driven);
	// the Index coupling of the Train, 0 its front end, Train.size() its rear end
	static coupling_place coupling_at(std::vector<TDynamicObject *> const &Train, int Index);
	// the distance to a vehicle not coupled beyond the End of the Vehicle, negative for none
	static double gap_beyond(TDynamicObject const *Vehicle, int End);
	// keeps up the way the train goes and works out where its ends are, for the trackside view
	cameraviews::train_motion motion_of(std::vector<TDynamicObject *> const &Train, TDynamicObject const *Driven);

	cameraviews::placement place_orbit(TCamera &Camera, double Seconds);
	cameraviews::placement place_shunting(TCamera const &Camera, std::vector<TDynamicObject *> const &Train, TDynamicObject const *Driven, double Seconds);
	cameraviews::placement place_coupling(TCamera const &Camera, std::vector<TDynamicObject *> const &Train, double Seconds);
	cameraviews::placement place_trackside(std::vector<TDynamicObject *> const &Train, TDynamicObject const *Driven);

	kind m_view{kind::orbit};
	TDynamicObject *m_vehicle{nullptr}; // the vehicle picked; only compared with the vehicles of the train, never followed
	int m_step{0}; // vehicles to step over in the next update, from [ and ]
	int m_coupling{-1}; // the coupling picked in the coupling view, -1 not yet
	cameraviews::orbit m_orbit;
	cameraviews::topdown m_topdown;
	cameraviews::coupler_view m_coupler;
	cameraviews::trackside m_trackside;
	glm::dvec3 m_lastposition{}; // of the vehicle picked, where the trackside view last saw it move
	glm::dvec3 m_direction{0.0, 0.0, -1.0}; // the way the train goes, for the trackside view
	bool m_moved{false}; // m_lastposition is known
};
