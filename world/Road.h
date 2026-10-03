/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "utilities/Classes.h"
#include "utilities/Names.h"
#include "scene/scenenode.h"
#include "world/Segment.h"

namespace scene
{
struct scratch_data;
class basic_section;
}

// geometry a node puts in the scene as shapes of its own, apart from the shapes of the scenery, so it can take it back
class owned_shapes
{
  public:
	// puts provided shapes in the section of the scene holding specified point
	void show(std::vector<scene::shape_node> Shapes, glm::dvec3 const &Location);
	// takes the shapes back
	void hide();
	bool shown() const
	{
		return m_section != nullptr;
	}
	// true if specified geometry was put in the scene by any object of this class
	static bool owns(gfx::geometry_handle const &Geometry);

  private:
	gfx::geometrybank_handle m_bank{0, 0};
	std::vector<gfx::geometry_handle> m_geometry;
	scene::basic_section *m_section{nullptr};
};

// a piece of a road for wheeled traffic: an axis, and a cross-section which stays the same along it.
// the node itself is neither driven on nor drawn. it produces the lanes, which are ordinary invisible one-way tracks,
// and the surface, the sides and the markings, which are ordinary shapes. this way the vehicles, the ai and the renderers
// deal with a road without knowing about it.
// scenery entry:
// node <max> <min> <name> road <axis, laid out like a path of a track> [<property> <values>]... endroad
class road_node : public scene::basic_node
{

  public:
	// types
	enum class side_type
	{
		none,
		shoulder, // level with the surface, closed with a bank
		sidewalk // raised on a kerb
	};
	enum class marking_colour
	{
		none,
		white,
		orange
	};
	// which of two neighbouring lanes can be left across the line they share; the lanes are taken left to right, facing along the axis
	enum lane_change : int
	{
		change_none = 0,
		change_toright = 1, // from the left lane to the right one
		change_toleft = 2, // from the right lane to the left one
		change_both = 3
	};
	struct lane_data
	{
		float width{3.5f};
		float velocity{-1.f}; // speed limit, none if negative
	};
	struct side_data
	{
		side_type type{side_type::none};
		float width{0.f};
		std::string material;
	};
	// everything the scenery says about the road. the lanes go left to right when facing along the axis:
	// the ones going against it, outermost first, then the ones going along it
	struct state
	{
		segment_data axis;
		int forward{1}; // number of lanes going along the axis
		int backward{1}; // number of lanes going against the axis
		float lanewidth{3.5f}; // what a lane gets unless it's told otherwise
		float velocity{-1.f};
		std::vector<lane_data> lanes;
		std::vector<int> changes; // permissions to change the lane, one for each pair of neighbouring lanes
		std::string surface{"asphaltdark1"};
		float texturelength{4.f};
		std::array<side_data, 2> sides; // left, right
		float kerbheight{0.12f};
		glm::vec2 slope{1.f, 0.4f}; // width and drop of the bank which closes a shoulder
		marking_colour markings{marking_colour::white};
		float friction{0.85f};
		float sounddistance{25.f};
		int quality{15};
		std::string environment{"flat"};

		// brings the content to a usable form: values within their limits, an entry for each lane and for each pair of neighbours
		void normalize();
		// identifier of a lane: f1..fn go along the axis and b1..bn against it, both counted outwards from where the directions meet
		std::string lane_id(std::size_t const Lane) const;
		// number of a lane in the left to right order, -1 if there's no lane with such identifier
		int lane_index(std::string const &Id) const;
		// permission to change the lane a pair of neighbouring lanes gets if the scenery doesn't say otherwise
		int default_change(std::size_t const Boundary) const;
		// distance of the middle of a lane from the axis, positive to the left when facing along the axis
		double lane_offset(std::size_t const Lane) const;
		// combined width of the lanes
		double width() const;
		// shape of the middle of a lane, laid out in the direction of travel
		segment_data lane_path(std::size_t const Lane) const;
		// length of the axis
		double length() const;
		// position on the axis and direction of the axis, for specified value of the curve parameter
		glm::dvec3 point(double const T) const;
		glm::dvec3 tangent(double const T) const;
	};
	// constructors
	explicit road_node(scene::node_data const &Nodedata);
	// methods
	// restores content of the node from provided input stream; reads up to and including the closing 'endroad'
	void import(cParser &Input, glm::dvec3 const &Offset);
	state const &definition() const
	{
		return m_state;
	}
	// replaces what the node knows about the road. the lanes and the geometry stay the way they were made,
	// it's up to the caller to take them down beforehand and make them again
	void define(state const &State);
	// creates paths of the lanes and hands them over to the simulation
	void create_lanes();
	// links lanes of opposite directions at an end of the road which leads nowhere, so the vehicles can turn back
	void close_ends();
	// gives up the paths of the lanes, or the ones made to close the loose ends. the paths belong to the path table,
	// the caller is expected to take them out of use
	std::vector<TTrack *> release_lanes();
	std::vector<TTrack *> release_turns();
	// paths of the lanes, in the order of the lanes, and the ones made to close the loose ends
	std::vector<TTrack *> const &tracks() const
	{
		return m_tracks;
	}
	std::vector<TTrack *> const &turns() const
	{
		return m_turns;
	}
	// true if there's a vehicle on any path of the road
	bool occupied() const;
	// generates geometry of the surface, the sides and the markings
	std::vector<scene::shape_node> create_shapes() const;
	// puts geometry of the road in the scene as shapes of its own, which can be taken back
	void show();
	// takes back geometry put in the scene by show()
	void hide();
	// marks geometry of the road as mixed with other shapes of the scenery; such road can't be changed anymore
	void merged(bool const Merged)
	{
		m_merged = Merged;
	}
	bool merged() const
	{
		return m_merged;
	}
	// members
	bool m_editorremoved{false}; // removed in the editor; kept around so the removal can be undone

  private:
	// methods
	// radius() subclass details, calculates node's bounding radius
	float radius_() override;
	// serialize() subclass details, sends content of the subclass to provided stream
	void serialize_(std::ostream &Output) const override;
	// deserialize() subclass details, restores content of the subclass from provided stream
	void deserialize_(std::istream &Input) override;
	// export() subclass details, sends basic content of the class in legacy (text) format to provided stream
	void export_as_text_(std::ostream &Output) const override;
	// creates a path for the vehicles and registers it with the simulation
	TTrack *create_track(std::string const &Name, segment_data const &Path, float const Width, float const Velocity);
	// members
	state m_state;
	std::vector<TTrack *> m_tracks;
	std::vector<TTrack *> m_turns;
	bool m_merged{false};
	owned_shapes m_shapes; // geometry put in the scene by show()
};

// a place where roads meet. each arm of the junction is where a road ends: a point in the middle of that end, the way
// the road leaves, and the lanes it has there. the junction produces the surface between the arms, and paths leading
// the vehicles from each lane coming in to the lanes going out of the other arms. a lane with more than one way out
// gets a legacy crossroads path, which is what the vehicles and their drivers already know how to pick a way through.
// scenery entry:
// node <max> <min> <name> junction <centre> arm <position> <direction x z> <lanes in> <lanes out> <lane width> ... [<property> <values>]... endjunction
class junction_node : public scene::basic_node
{

  public:
	// types
	struct arm_data
	{
		glm::dvec3 position{0.0}; // middle of the end of the road
		glm::dvec2 direction{0.0, 1.0}; // the way out of the junction, seen from above (x, z)
		int incoming{1}; // lanes leading into the junction
		int outgoing{1}; // lanes leading out of it
		float width{3.5f}; // width of a lane
	};
	struct state
	{
		glm::dvec3 centre{0.0};
		std::vector<arm_data> arms;
		std::string surface{"asphaltdark1"};
		float texturelength{4.f};
		road_node::marking_colour markings{road_node::marking_colour::white};
		float velocity{30.f}; // speed limit on the way through
		float friction{0.85f};
		float sounddistance{25.f};
		int quality{15};
		std::string environment{"flat"};

		// brings the content to a usable form
		void normalize();
		// width of the road at specified arm
		double arm_width(std::size_t const Arm) const;
		// where a lane of an arm meets the junction; the lanes of each kind are counted from the middle of the road, starting with 1
		glm::dvec3 lane_point(std::size_t const Arm, bool const Incoming, int const Lane) const;
		// arms in the order they're met going around the junction
		std::vector<std::size_t> arm_order() const;
		// edge of the surface: ends of the arms joined with rounded corners, going around the junction.
		// Corners, if provided, receives the points of each corner apart
		std::vector<glm::dvec3> outline(std::vector<std::vector<glm::dvec3>> *Corners = nullptr) const;
	};
	// constructors
	explicit junction_node(scene::node_data const &Nodedata);
	// methods
	// restores content of the node from provided input stream; reads up to and including the closing 'endjunction'
	void import(cParser &Input, glm::dvec3 const &Offset);
	state const &definition() const
	{
		return m_state;
	}
	void define(state const &State);
	// makes paths leading through the junction, between the lanes of the roads attached to it at the moment
	void create_links();
	// gives up the paths leading through the junction; the caller is expected to take them out of use
	std::vector<TTrack *> release_links();
	std::vector<TTrack *> const &links() const
	{
		return m_links;
	}
	// shapes of the ways through the junction, for display
	std::vector<segment_data> const &movements() const
	{
		return m_movements;
	}
	// true if there's a vehicle on any path of the junction
	bool occupied() const;
	// generates geometry of the surface
	std::vector<scene::shape_node> create_shapes() const;
	// puts geometry of the junction in the scene as shapes of its own, or takes it back
	void show();
	void hide();
	void merged(bool const Merged)
	{
		m_merged = Merged;
	}
	bool merged() const
	{
		return m_merged;
	}
	// members
	bool m_editorremoved{false};

  private:
	// methods
	float radius_() override;
	void serialize_(std::ostream &Output) const override;
	void deserialize_(std::istream &Input) override;
	void export_as_text_(std::ostream &Output) const override;
	// creates a path through the junction from provided definition and registers it with the simulation
	TTrack *create_track(std::string const &Definition, std::size_t const Index);
	// members
	state m_state;
	std::vector<TTrack *> m_links;
	std::vector<segment_data> m_movements;
	bool m_merged{false};
	owned_shapes m_shapes;
};

// collection of roads present in the scene
class road_table : public basic_table<road_node>
{

  public:
	// legacy style initialization, to be performed when the tracks are already joined
	void InitRoads();
	// generates geometry of the roads and puts it in the scene
	void create_geometry(scene::scratch_data &Scratchpad);
};

// collection of road junctions present in the scene
class junction_table : public basic_table<junction_node>
{

  public:
	// legacy style initialization, to be performed when the tracks are joined, ahead of the roads
	void InitJunctions();
	// generates geometry of the junctions and puts it in the scene
	void create_geometry(scene::scratch_data &Scratchpad);
};

//---------------------------------------------------------------------------
