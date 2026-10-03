/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "utilities/Classes.h"
#include "utilities/Names.h"
#include "scene/scenenode.h"
#include "world/Road.h"

class TDynamicObject;
class basic_event;

// a place on a road where something happens to the traffic, or is painted for it:
// - a level crossing: vehicles are stopped ahead of the rails for as long as a train is near,
// - a point where vehicles appear, drawn from a set made ready while the scenery is loaded,
// - a point where vehicles are taken away. the ones which came from a point of the previous kind go back to its set,
// - a pedestrian crossing: stripes painted across the road.
// scenery entries:
// node <max> <min> <name> crossing <position> [clearance <m>] [warning <m>] [stoplines <yes|no>] endcrossing
// node <max> <min> <name> spawn <position> [interval <s>] [variation <0..1>] [velocity <km/h>] [count <n>]
//     [node <max> <min> <name> dynamic <folder> <skin> <type> <path> <offset> <driver> <velocity> <load> [<load type>] enddynamic]... endspawn
// node <max> <min> <name> despawn <position> [radius <m>] enddespawn
// node <max> <min> <name> crosswalk <position> [length <m>] endcrosswalk
// the vehicles of a spawn point are given the way vehicles are put in a scenery; where they're put and how they're driven
// is up to the point, so the path, the offset, the driver and the velocity of these entries don't matter. the listed vehicles
// are what the point draws from to have <count> vehicles of its own on the roads; one which gets stuck is taken off the road
class roadpoint_node : public scene::basic_node
{

  public:
	// types
	enum class kind_type
	{
		crossing,
		spawn,
		despawn,
		crosswalk
	};
	struct vehicle_data
	{
		std::string folder; // where the vehicle is defined, in the folder of the vehicles
		std::string skin;
		std::string type; // name of its definition files
		int load{0};
		std::string loadtype;

		bool operator==(vehicle_data const &Other) const
		{
			return folder == Other.folder && skin == Other.skin && type == Other.type && load == Other.load && loadtype == Other.loadtype;
		}
	};
	struct state
	{
		kind_type kind{kind_type::crossing};
		glm::dvec3 position{0.0};
		// level crossing
		float clearance{6.f}; // distance along the road from the point to the places the vehicles stop at
		float warning{800.f}; // how far along the rails a train coming closes the crossing
		bool stoplines{true}; // lines are painted where the vehicles stop
		// point where vehicles appear
		float interval{20.f}; // seconds from a vehicle to the next
		float variation{0.5f}; // part of the interval which is drawn at random
		float velocity{50.f}; // speed the vehicles appear with, and are told to keep, km/h
		int count{6}; // how many vehicles of the point can be on the roads at a time. that many are made ready, drawn from the listed ones
		std::vector<vehicle_data> vehicles;
		// point where vehicles are taken away
		float radius{2.f}; // vehicles on lanes this close to the point are taken
		// pedestrian crossing
		float length{4.f}; // how much of the road the stripes take along it

		// brings the content to a usable form
		void normalize();
	};
	// place the vehicles of a lane stop at while a crossing is closed
	struct stop_data
	{
		TTrack *track{nullptr}; // the lane it's on
		glm::dvec3 position{0.0};
		glm::dvec3 direction{0.0, 0.0, 1.0}; // the way the lane goes there
		float width{3.5f}; // of the lane
	};
	// constructors
	explicit roadpoint_node(scene::node_data const &Nodedata);
	// methods
	// restores content of the node from provided input stream; reads up to and including the closing statement
	void import(cParser &Input, glm::dvec3 const &Offset);
	state const &definition() const
	{
		return m_state;
	}
	// replaces what the node knows about itself. what it's tied to stays, it's up to the caller to bind it again.
	// a spawn point given another set of vehicles makes them anew; the ones it had stay on the road until they're taken away
	void define(state const &State);
	// scenery keyword for specified kind of point, and the statement closing its entry
	static std::string keyword(kind_type const Kind);
	static bool is_keyword(std::string const &Type);
	// reads vehicles out of a text. each is given the way vehicles are put in a scenery, or, if there's no such entry
	// in the text, as "<folder> <skin> <type> [<load> <load type>]" on a line of its own
	static std::vector<vehicle_data> parse_vehicles(std::string const &Text);
	// ties the point to the lanes and the rails as they are at the moment, or lets go of them
	void bind();
	void unbind();
	// makes the vehicles of a spawn point, once. it loads their models, which takes a while
	void prepare();
	// to be called with each step of the simulation
	void update(double const Deltatime);
	// state for display
	bool closed() const
	{
		return m_closed;
	}
	std::vector<stop_data> const &stops() const
	{
		return m_stops;
	}
	std::vector<TTrack *> const &lanes() const
	{
		return m_lanes;
	}
	std::size_t rails() const
	{
		return m_rails.size();
	}
	// vehicles made for a spawn point, and how many of them wait for their turn
	std::size_t vehicles() const
	{
		return m_pool.size();
	}
	std::size_t waiting() const;
	// true if specified vehicle stands in the line of vehicles held by the crossing, which is closed
	bool holds(TDynamicObject const &Vehicle) const;
	// generates geometry of the stop lines
	std::vector<scene::shape_node> create_shapes() const;
	// puts geometry of the point in the scene, or takes it back
	void show();
	void hide();
	bool shown() const
	{
		return m_shapes.shown();
	}
	// members
	bool m_editorremoved{false}; // removed in the editor; kept around so the removal can be undone

  private:
	// types
	struct pooled_vehicle
	{
		TDynamicObject *vehicle{nullptr};
		double velocity{0.0}; // what its speed was when it was made, to be given back each time it's put on the road
		double standing{0.0}; // how long it's been standing still on the road
	};
	// methods
	float radius_() override;
	void serialize_(std::ostream &Output) const override;
	void deserialize_(std::istream &Input) override;
	void export_as_text_(std::ostream &Output) const override;
	void bind_crossing();
	void update_crossing(double const Deltatime);
	void update_spawn(double const Deltatime);
	void update_despawn();
	void close(bool const Closed);
	// members
	state m_state;
	bool m_bound{false};
	// level crossing
	std::vector<stop_data> m_stops;
	std::vector<basic_event *> m_events; // orders for the drivers, one for each stop. these are never freed, as a driver can hold on to one
	std::vector<TTrack *> m_rails; // rails a train closes the crossing from
	std::vector<TTrack *> m_queue; // lanes the vehicles held by the crossing wait on
	std::map<TDynamicObject const *, double> m_seen; // rail vehicles near, with how far each was the last time
	bool m_closed{false};
	double m_hold{0.0}; // time left for the crossing to stay closed after the last train was seen
	double m_scan{0.0}; // time left to the next look at the rails
	// spawn and despawn
	std::vector<TTrack *> m_lanes; // spawn: the lane the vehicles are put on. despawn: lanes the vehicles are taken from
	double m_offset{0.0}; // spawn: distance of the point from the start of that lane
	std::vector<pooled_vehicle> m_pool;
	bool m_prepared{false};
	int m_made{0}; // number of vehicles made so far, for their names
	double m_timer{0.0}; // time left to the next vehicle
	// pedestrian crossing: the road at the point
	bool m_onroad{false};
	glm::dvec3 m_along{0.0, 0.0, 1.0}; // the way the road goes
	double m_halfwidth{0.0}; // of the road
	owned_shapes m_shapes;
};

// collection of the road points present in the scene
class roadpoint_table : public basic_table<roadpoint_node>
{

  public:
	// legacy style initialization, to be performed when the lanes of the roads are tied together
	void InitRoadpoints();
	// lets go of the lanes and the rails, ahead of a change of the roads, and ties all points anew when it's done
	void unbind();
	void bind();
	// puts geometry of the points in the scene
	void create_geometry();
	// to be called with each step of the simulation
	void update(double const Deltatime);
	// true if specified vehicle stands in the line of vehicles held by a closed level crossing
	bool holds(TDynamicObject const &Vehicle) const;

  private:
	bool m_geometry{false}; // the scene is ready to take geometry of the points
};

//---------------------------------------------------------------------------
