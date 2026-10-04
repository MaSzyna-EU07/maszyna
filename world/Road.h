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
class road_order;

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

// a piece of a road for wheeled traffic: an axis, and a cross-section which stays the same along it, save for getting
// evenly wider or narrower if it's told to.
// the node itself is neither driven on nor drawn. it produces the lanes, which are ordinary invisible one-way tracks,
// and the surface, the sides and the markings, which are ordinary shapes. this way the vehicles, the ai and the renderers
// deal with a road without knowing about it.
// scenery entry:
// node <max> <min> <name> road <axis, laid out like a path of a track> [<property> <values>]... endroad
// properties, past the ones of the lanes and the look: median <gap|painted|island> <width at the start> <width at the end> <material>,
// medianround <length of the rounded end of an island at the start> <at the end>,
// kerbs <left|right|both> <width> <material>, bank <left|right|both> <width at the start> <drop at the start> <width at the end> <drop at the end>,
// bankmaterial <material>, weight <value>, roundabout,
// lanesto <along the axis> <against the axis>: lanes the piece has at the end of its axis, where these aren't the ones it starts with
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
	// what keeps the two directions of a road apart, where they're led away from each other
	enum class median_type
	{
		gap, // nothing: the ground shows between two roadways
		painted, // the surface carries on, marked as closed to the traffic
		island // raised on a kerb
	};
	struct median_data
	{
		median_type type{median_type::painted};
		std::array<float, 2> width{0.f, 0.f}; // at the start and at the end of the axis
		std::string material{"none"}; // what an island is covered with
		std::array<float, 2> round{0.f, 0.f}; // length of the rounded end of an island, at the start and at the end of the axis. 0: the island is cut straight
	};
	// slope leading from the edge of the road down, or up, to the ground
	struct bank_data
	{
		bool set{false}; // false: a shoulder gets the bank the road gives to all its shoulders, anything else gets none
		std::array<float, 2> width{0.f, 0.f}; // at the start and at the end of the axis
		std::array<float, 2> drop{0.f, 0.f}; // how far below the edge of the road it ends; negative if it rises

		bool operator==(bank_data const &Other) const
		{
			return set == Other.set && width == Other.width && drop == Other.drop;
		}
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
		std::array<float, 2> taper{1.f, 1.f}; // what the widths of the lanes are multiplied by at the start and at the end of the axis
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
		float weight{100.f}; // how willing the drivers are to take the road when leaving a junction, next to the other roads they can take there
		median_data median;
		std::array<bool, 2> kerbs{false, false}; // left, right: a kerb runs along the edge of the surface, as high as the kerb of a sidewalk
		float kerbwidth{0.15f};
		std::string kerbmaterial{"none"};
		std::array<bank_data, 2> banks; // left, right
		std::string bankmaterial{"none"}; // what a bank is covered with, other than one of a shoulder, which goes with the shoulder
		bool roundabout{false}; // the piece is a part of a roundabout: traffic on it goes ahead of the traffic joining it
		// lanes the piece is short of at an end of its axis: [end][0: the ones going along the axis, 1: the ones going against it].
		// these are the outermost lanes of the direction, which the road gains or loses along the piece;
		// forward and backward say how many lanes there are where the piece has the most of them
		std::array<std::array<int, 2>, 2> missing{};

		// brings the content to a usable form: values within their limits, an entry for each lane and for each pair of neighbours
		void normalize();
		// true if the piece has other lanes at its end than at its start
		bool changing() const;
		// sets the lanes the piece has at the start and at the end of its axis
		void lanes_between(int const Startforward, int const Startbackward, int const Endforward, int const Endbackward);
		// number of lanes at an end of the axis (0: the start, 1: the end): of the ones going along it, or of the ones going against it
		int lanes_at(int const End, bool const Along) const;
		// true if the piece has the lane at specified end of the axis
		bool lane_at(std::size_t const Lane, int const End) const;
		// the same road with the lanes it has at specified end of the axis all the way
		state section(int const End) const;
		// identifier of a lane: f1..fn go along the axis and b1..bn against it, both counted outwards from where the directions meet
		std::string lane_id(std::size_t const Lane) const;
		// number of a lane in the left to right order, -1 if there's no lane with such identifier
		int lane_index(std::string const &Id) const;
		// permission to change the lane a pair of neighbouring lanes gets if the scenery doesn't say otherwise
		int default_change(std::size_t const Boundary) const;
		// distance of the middle of a lane from the axis, positive to the left when facing along the axis
		double lane_offset(std::size_t const Lane) const;
		// the same at an end of the axis, where the road is centred on the axis with the lanes it has there.
		// a lane the road is short of at that end is where the lane it comes out of, or goes into, is
		double lane_offset(std::size_t const Lane, int const End) const;
		// distance of the middle of a lane from the axis at specified part of the length of the axis, with the taper and what's between the directions counted in
		double lane_position(std::size_t const Lane, double const Fraction) const;
		// combined width of the lanes, before the taper
		double width() const;
		// the same for the lanes the piece has at an end of the axis
		double width(int const End) const;
		// combined width of the lanes at specified part of the length of the axis, with the taper counted in
		double breadth(double const Fraction) const;
		// what the widths of the lanes are multiplied by at specified value of the curve parameter
		double scale(double const T) const;
		// true if the two directions of the road are kept apart anywhere along it
		bool divided() const;
		// how far apart the two directions are at specified value of the curve parameter
		double median_width(double const T) const;
		// distance from the axis of the line the two directions meet at, on a road which doesn't keep them apart
		double divide() const;
		// the same for the lanes the piece has at an end of the axis
		double divide(int const End) const;
		// the same at specified part of the length of the axis, with the taper counted in
		double middle(double const Fraction) const;
		// width of the whole roadway at specified value of the curve parameter: the lanes and what's between the directions
		double span(double const T) const;
		// shape of the middle of a lane, laid out in the direction of travel
		segment_data lane_path(std::size_t const Lane) const;
		// shape of the way from the middle of a lane to the middle of another over a part of the axis, laid out in the direction of travel.
		// From, To: values of the curve parameter of the axis, in the order of the axis
		segment_data lane_path(std::size_t const Lane, std::size_t const Tolane, double const From, double const To) const;
		// part of the axis, as values of its curve parameter, over which a piece which gains or loses lanes leads the lanes it has
		// at one end to the lanes it has at the other. before it and past it there's a short path for each lane the piece has at that end
		std::array<double, 2> change_range() const;
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
	// makes the paths which lead the lanes the piece has at one end to the lanes it has at the other, where these differ.
	// to be called once the lanes are joined with the lanes of the neighbours
	void create_links();
	// links lanes of opposite directions at an end of the road which leads nowhere, so the vehicles can turn back
	void close_ends();
	// gives up the paths of the lanes, or the ones made to close the loose ends. the paths belong to the path table,
	// the caller is expected to take them out of use
	std::vector<TTrack *> release_lanes();
	std::vector<TTrack *> release_turns();
	// paths of the lanes, and the ones made to close the loose ends. the former go in the order of the lanes;
	// on a piece which gains or loses lanes these are the short paths left of each such lane at the ends of the piece
	std::vector<TTrack *> const &tracks() const
	{
		return m_tracks;
	}
	// path of a lane at an end of the axis (0: the start, 1: the end); nullptr if the piece doesn't have the lane there
	TTrack *lane_track(std::size_t const Lane, int const End) const
	{
		return Lane < m_lanetracks[End].size() ? m_lanetracks[End][Lane] : nullptr;
	}
	// paths leading from the lanes at one end to the lanes at the other, on a piece which gains or loses lanes
	std::vector<TTrack *> const &links() const
	{
		return m_links;
	}
	// shapes of the ways these make, for display
	std::vector<segment_data> const &ways() const
	{
		return m_ways;
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
	// the same for a path put together by the caller, the way the scenery defines one
	TTrack *create_track(std::string const &Name, std::string const &Definition);
	// members
	state m_state;
	std::vector<TTrack *> m_tracks;
	std::array<std::vector<TTrack *>, 2> m_lanetracks; // path of each lane at the start and at the end of the axis
	std::vector<TTrack *> m_links;
	std::vector<segment_data> m_ways;
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
// properties: surface <material>, texlength <m>, markings <white|orange|none>, side <none|shoulder|sidewalk> <width> <material>, kerb <height>,
// slope <width> <drop>, stopline <number of an arm, counted from 1>, velocity <km/h>, friction <value>, environment <name>,
// priority <arm> <none|main|yield|stop>, turns <arm> <lane leading in, counted from the middle of the road> <letters out of l, s, r>,
// armmedian <arm> <width of what keeps the two directions of the road apart there>, kerbs <width> <material>,
// cornerbank <arm> <width> <drop> <width> <drop> (at that road, then at the next one), bankmaterial <material>,
// crosswalk <arm> <length>, median <gap|painted|island> <material>
// a junction of two roads is drawn as a stretch of road, on which the lanes of one are led to the lanes of the other.
// that's what a road which changed its lanes used to take; such a junction found in a scenery is loaded as a piece of road instead, see as_road()
// the arms don't have to be level with each other: the surface is spanned between their ends and the centre.
// the junction also tells the vehicles when to wait: the ones which have to give way wait for the ones with the right of way,
// and where neither has it, for the ones coming from their right, or from the opposite side when turning left across their way
class junction_node : public scene::basic_node
{

  public:
	// types
	// what the vehicles coming by a road are to do about the ones coming by the other roads
	enum class right_of_way
	{
		none, // the rule of the right hand
		priority, // the others give way
		yield, // gives way
		stop // stops, then gives way
	};
	// ways a lane can be left by
	enum turn_flags : int
	{
		turn_left = 1,
		turn_straight = 2,
		turn_right = 4
	};
	struct arm_data
	{
		glm::dvec3 position{0.0}; // middle of the end of the road
		glm::dvec2 direction{0.0, 1.0}; // the way out of the junction, seen from above (x, z)
		int incoming{1}; // lanes leading into the junction
		int outgoing{1}; // lanes leading out of it
		float width{3.5f}; // width of a lane
		bool stopline{false}; // a line is painted across the lanes leading into the junction
		float median{0.f}; // how far apart the two directions of the road are there
		float crosswalk{0.f}; // length of the pedestrian crossing painted across the road where it meets the junction; 0: none
		// slope leading to the ground from the corner which starts at the left edge of this road, seen from the junction, and goes on
		// to the next road. it starts at this road and ends at the next one
		road_node::bank_data bank;
		right_of_way priority{right_of_way::none};
		// ways each lane leading into the junction can be left by, a sum of turn_flags; the lanes are counted from the middle of the road.
		// a lane with no entry, or with 0, gets what the junction works out: the inner lane takes the left turns, the outer one the right turns
		std::vector<int> turns;
	};
	// a way through the junction from a lane leading into it
	struct way_data
	{
		std::size_t exit{0}; // arm it leaves by
		int kind{turn_straight}; // one of turn_flags
		int code{0}; // what a driver calls it, once they picked it
		TTrack *track{nullptr}; // lane it leads to
		segment_data path;
		std::vector<std::pair<std::size_t, std::size_t>> conflicts; // ways it can't be taken along with: lane leading in, and a way from that lane
	};
	// a lane leading into the junction, with what its traffic is told
	struct gate_data
	{
		std::size_t arm{0};
		int lane{1};
		TTrack *entry{nullptr}; // the lane
		TTrack *link{nullptr}; // path, or crossroads of paths, leading on from it
		std::vector<way_data> ways;
		std::vector<TTrack *> approach; // lanes the vehicles heading for the junction are looked for on, nearest first
		// for each of these: the way a vehicle on it has to have picked to be heading for the junction; 0 if there's no choice there.
		// it's set for a path on which a road forks into more lanes
		std::vector<int> wishes;
		glm::dvec3 line{0.0}; // where the lane ends
		road_order *order{nullptr};
		// what's going on at the moment
		TDynamicObject *first{nullptr}; // vehicle nearest to the junction
		double distance{0.0}; // of that vehicle from the end of the lane
		double speed{0.0};
		double halflength{0.0};
		unsigned taken{0}; // ways that vehicle may take, a bit for each
		unsigned inside{0}; // ways with a vehicle on them
		bool active{false}; // the vehicle is near enough to matter
		bool blocked{false}; // it's told to wait
		bool committed{false}; // it was let through, the others wait for it
		bool stopped{false}; // it made its stop at the stop sign
		double waited{0.0}; // how long it's been standing there
		double idle{0.0}; // how long it's been standing there since it was let through
	};
	struct state
	{
		glm::dvec3 centre{0.0};
		std::vector<arm_data> arms;
		std::string surface{"asphaltdark1"};
		float texturelength{4.f};
		road_node::marking_colour markings{road_node::marking_colour::white};
		road_node::side_data side; // what the corners between the roads are lined with, the way the sides of a road are
		bool kerbs{false}; // a kerb runs around the corners, the way it runs along the edges of a road
		float kerbwidth{0.15f};
		std::string kerbmaterial{"none"};
		float kerbheight{0.12f};
		glm::vec2 slope{1.f, 0.4f}; // width and drop of the bank which closes a shoulder
		std::string bankmaterial{"none"}; // what a bank of a corner is covered with, other than one of a shoulder
		// a junction of two roads, where a road changes its lanes: what's between the two directions where they're apart
		road_node::median_type median{road_node::median_type::painted};
		std::string medianmaterial{"none"}; // what an island is covered with
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
	// a junction of two roads is a stretch where a road changes its lanes, which a road can do by itself.
	// Road: receives the piece of road to take the place of the junction. returns: false if the junction isn't of that kind
	bool as_road(road_node::state &Road) const;
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
	// tells the vehicles heading for the junction whether they can drive on. to be called with each step of the simulation
	void update(double const Deltatime);
	// lanes leading into the junction, with what their traffic is told at the moment
	std::vector<gate_data> const &gates() const
	{
		return m_gates;
	}
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
	// geometry of a junction of two roads
	std::vector<scene::shape_node> create_transition_shapes() const;
	// sets up what the traffic of each lane leading in is told, once the ways through are made
	void create_gates();
	// members
	state m_state;
	std::vector<TTrack *> m_links;
	std::vector<segment_data> m_movements;
	std::vector<gate_data> m_gates;
	std::vector<road_order *> m_orders; // orders for the drivers, used again each time the gates are made. never freed, as a driver can hold on to one
	double m_scan{0.0}; // time left to the next look at the traffic
	bool m_quiet{false}; // there was no traffic the last time
	bool m_merged{false};
	owned_shapes m_shapes;
};

// collection of roads present in the scene
class road_table : public basic_table<road_node>
{

  public:
	// legacy style initialization, to be performed when the tracks are already joined: ties up the lanes of the pieces
	// which gain or lose them. the junctions go by these, so it comes ahead of their initialization
	void InitLanes();
	// the same, to be performed once the junctions are done: closes the ends of the roads which lead nowhere
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
	// to be called with each step of the simulation
	void update(double const Deltatime);
	// generates geometry of the junctions and puts it in the scene
	void create_geometry(scene::scratch_data &Scratchpad);
};

//---------------------------------------------------------------------------
