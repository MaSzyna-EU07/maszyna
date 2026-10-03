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
}

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
		TTrack *track{nullptr}; // the generated path; owned by the path table, like every other track
	};
	struct side_data
	{
		side_type type{side_type::none};
		float width{0.f};
		std::string material;
	};
	// constructors
	explicit road_node(scene::node_data const &Nodedata);
	// methods
	// restores content of the node from provided input stream; reads up to and including the closing 'endroad'
	void import(cParser &Input, glm::dvec3 const &Offset);
	// creates paths of the lanes and hands them over to the simulation
	void create_lanes();
	// links lanes of opposite directions at an end of the road which leads nowhere, so the vehicles can turn back
	void close_ends();
	// generates geometry of the surface, the sides and the markings
	std::vector<scene::shape_node> create_shapes() const;
	// identifier of a lane: f1..fn go along the axis and b1..bn against it, both counted outwards from where the directions meet
	std::string lane_id(std::size_t const Lane) const;
	// distance of the middle of a lane from the axis, positive to the left when facing along the axis
	double lane_offset(std::size_t const Lane) const;
	// combined width of the lanes
	double width() const;
	// lanes, left to right when facing along the axis: the ones going against it, outermost first, then the ones going along it
	std::vector<lane_data> const &lanes() const
	{
		return m_lanes;
	}
	// permissions to change the lane, one for each pair of neighbouring lanes
	std::vector<int> const &lane_changes() const
	{
		return m_changes;
	}
	segment_data const &axis() const
	{
		return m_axis;
	}

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
	// number of a lane in the left to right order, -1 if the road has no lane with such identifier
	int lane_index(std::string const &Id) const;
	// permission to change the lane a pair of neighbouring lanes gets if the scenery doesn't say otherwise
	int default_change(std::size_t const Boundary) const;
	// shape of the middle of a lane, laid out in the direction of travel
	segment_data lane_path(std::size_t const Lane) const;
	// creates a path for the vehicles and registers it with the simulation
	TTrack *create_track(std::string const &Name, segment_data const &Path, float const Width, float const Velocity);
	// members
	segment_data m_axis;
	int m_forward{1}; // number of lanes going along the axis
	int m_backward{1}; // number of lanes going against the axis
	float m_lanewidth{3.5f};
	float m_velocity{-1.f};
	std::vector<lane_data> m_lanes;
	std::vector<int> m_changes;
	std::string m_surface{"asphaltdark1"};
	float m_texturelength{4.f};
	std::array<side_data, 2> m_sides; // left, right
	float m_kerbheight{0.12f};
	glm::vec2 m_slope{1.f, 0.4f}; // width and drop of the bank which closes a shoulder
	marking_colour m_markings{marking_colour::white};
	float m_friction{0.85f};
	float m_sounddistance{25.f};
	int m_quality{15};
	std::string m_environment{"flat"};
	std::vector<TTrack *> m_turns; // paths made to close loose ends of the road
};

// collection of roads present in the scene
class road_table : public basic_table<road_node>
{

  public:
	// legacy style initialization, to be performed when the tracks are already joined
	void InitRoads();
	// generates geometry of the roads and inserts it in the region
	void create_geometry(scene::scratch_data &Scratchpad);
};

//---------------------------------------------------------------------------
