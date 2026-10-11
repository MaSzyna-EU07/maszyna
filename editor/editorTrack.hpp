/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/

#pragma once

#include <array>
#include <functional>
#include <string>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "world/Segment.h"
#include "utilities/Classes.h"

class TTrack;
class basic_event;

class editor_track
{
  public:
	enum class point_kind { start, control1, control2, end };
	static bool is_end(point_kind const Kind) { return Kind == point_kind::start || Kind == point_kind::end; }
	struct point_ref
	{
		int path{-1};
		point_kind kind{point_kind::start};
		bool valid() const { return path >= 0; }
		bool operator==(point_ref const &Other) const { return path == Other.path && kind == Other.kind; }
	};

	struct state
	{
		std::vector<segment_data> paths;
		double velocity{-1.0};
		float switchvelocity{-1.f};
		float width{1.435f};
		float friction{0.15f};
		float sounddistance{-1.f};
		int quality{20};
		int damage{0};
		int environment{0};
		material_handle material1{0};
		material_handle material2{0};
		material_handle material3{0};
		float texlength{4.f};
		float texheight{0.6f};
		float texwidth{0.9f};
		float texslope{0.9f};
		std::vector<std::string> isolated;
		float overhead{-1.f};
		bool sleepers{false};
		float sleeper_frequency{0.6f};
		std::string sleeper_model;
		std::string sleeper_skin;
		glm::vec3 sleeper_offset{0.f};
		float sleeper_ballast{0.f};
		std::array<std::vector<std::pair<std::string, basic_event *>>, 6> events;
		std::string name;
	};
	static state capture(TTrack const &Track);
	static void apply(TTrack &Track, state const &State);

	static bool name_valid(TTrack const &Track, std::string &Name, std::string &Reason, TTrack **Owner = nullptr);
	struct name_uses
	{
		int cells{0};
		int includes{0};
		std::vector<std::string> events;
		std::vector<std::string> loose;
	};
	static name_uses uses_of_name(TTrack const &Track);
	static void rename(TTrack &Track, std::string const &Name);

	// lists of the events of the path, in the order of event_keyword(): event0, event1, event2, eventall0, eventall1, eventall2
	static std::vector<std::pair<std::string, basic_event *>> &events(TTrack &Track, int const Index);
	static char const *event_keyword(int const Index);
	// looks the events up by their names; the names of those which don't exist are given in Missing
	static void bind_events(TTrack &Track, std::string &Missing);
	// track circuits the path belongs to, made when they don't exist yet
	static void isolated(TTrack &Track, std::vector<std::string> const &Names);
	// -1: normal, 0: no current, above 0: speed limit with the pantographs lowered
	static void overhead(TTrack &Track, float const Overhead);

	static bool can_edit_geometry(TTrack const &Track, std::string &Reason);
	static bool is_supported(TTrack const &Track);

	static void commit(std::vector<TTrack *> const &Tracks);
	static void commit_parameters(TTrack &Track);

	static glm::dvec3 point_position(TTrack const &Track, point_ref const &Point);
	static void move_point(TTrack &Track, point_ref const &Point, glm::dvec3 const &Position);
	static void translate(TTrack &Track, glm::dvec3 const &Offset);
	static void rotate_y(TTrack &Track, glm::dvec3 const &Pivot, double const Angle);
	static glm::dvec3 pivot(TTrack const &Track);

	static std::vector<TTrack *> neighbours(TTrack const &Track);
	static std::vector<std::pair<TTrack *, point_ref>> connected_points(TTrack const &Track, glm::dvec3 const &Position);
	static bool is_connected(TTrack const &Track, point_ref const &Point);
	// end of a regular neighbouring path joined to an end of the track, as it was when taken
	struct joint
	{
		point_ref own;
		TTrack *other{nullptr};
		point_ref theirs;
		glm::dvec2 direction{0.0, 1.0}; // of the track at its end, into it
		glm::dvec3 control{0.0}; // of the neighbour at its end, never zero
	};
	// joints the editor can change: switches and paths which can't be edited aren't taken
	static std::vector<joint> joints(TTrack const &Track);
	// moves the joined ends to where the ends of the track are now, turning them as the track turned there
	static void follow(TTrack const &Track, std::vector<joint> const &Joints);

	struct snap_target
	{
		TTrack *track{nullptr};
		point_ref point;
		glm::dvec3 position{0.0};
		glm::dvec3 direction{0.0};
		double distance{0.0};
	};
	static snap_target find_snap_target(TTrack const &Track, glm::dvec3 const &Position, double const Radius, std::vector<TTrack const *> const &Exclude);
	// nearest end of a path of the category which nothing else joins; Self may be null
	static snap_target find_free_end(TTrack const *Self, int const Category, glm::dvec3 const &Position, double const Radius, std::vector<TTrack const *> const &Exclude);
	// all of them within the radius, nearest first
	static std::vector<snap_target> free_ends(TTrack const *Self, int const Category, glm::dvec3 const &Position, double const Radius, std::vector<TTrack const *> const &Exclude);
	static void snap_point(TTrack &Track, point_ref const &Point, snap_target const &Target, bool const Aligntangent);

	struct chain
	{
		std::vector<TTrack *> tracks;
		std::vector<bool> forward;
		glm::dvec3 start{0.0};
		glm::dvec3 end{0.0};
		glm::dvec3 start_direction{0.0};
		glm::dvec3 end_direction{0.0};
		double start_radius{0.0};
		double end_radius{0.0};
		double length{0.0};
		double velocity{-1.0};
		double radius{0.0};
		// switches the chain runs through along their main track, each before the regular path of the index given
		struct passage
		{
			TTrack *track{nullptr};
			bool forward{true};
			std::size_t before{0};
		};
		std::vector<passage> switches;
	};
	// with Switches the chain may run through the switches along their main tracks; its ends are regular paths
	static bool find_chain(TTrack *From, TTrack *To, chain &Chain, std::string &Error, bool const Switches = false);
	// the regular parts of a chain between its switches, empty ones skipped
	static std::vector<chain> chain_parts(chain const &Chain);
	// moves the track as a whole, From going to To, turned by Angle in the plan
	static void place(TTrack &Track, glm::dvec3 const &From, glm::dvec3 const &To, double const Angle);

	struct straight
	{
		std::vector<TTrack *> tracks;
		std::vector<int> paths;
		glm::dvec3 start{0.0};
		glm::dvec3 end{0.0};
		glm::dvec2 direction{0.0, 1.0};
		double length{0.0};
		double grade{0.0};
		double azimuth{0.0};
		// distance of the point from the start, measured along the line in the plan
		double along(glm::dvec3 const &Point) const { return (Point.x - start.x) * direction.x + (Point.z - start.z) * direction.y; }
	};
	struct straight_tolerance
	{
		double angle{0.0002};
		double offset{0.02};
		double radius{20000.0};
	};
	static bool is_straight(TTrack const &Track, straight_tolerance const &Tolerance);
	static straight find_straight(TTrack &Track, straight_tolerance const &Tolerance);
	static std::vector<straight> find_straights(double const Minimumlength, straight_tolerance const &Tolerance);
	static std::vector<TTrack *> straight_affected(std::vector<straight> const &Lines);
	static TTrack *outside_neighbour(straight const &Line, bool const Atend);
	static bool touches(TTrack const &Track, glm::dvec3 const &Point);
	struct curve
	{
		TTrack *from{nullptr};
		TTrack *to{nullptr};
		double turn{0.0};
		int reversals{0};
		double radius{0.0};
		double transition_in{0.0};
		double transition_out{0.0};
		double cant{0.0};
		bool compound{false};
		struct arc_part
		{
			double radius{0.0};
			double turn{0.0};
			double transition{0.0};
		};
		std::vector<arc_part> arcs;
	};
	static bool find_curve(TTrack &Track, straight_tolerance const &Tolerance, double const Gauge, curve &Curve);
	// curve made of the given paths, each with the flag of being run from its start
	static bool analyse_curve(std::vector<std::pair<TTrack *, bool>> const &Run, double const Gauge, curve &Curve);
	// regular paths in line through the track, up to switches, free ends or about Limit metres each way
	static bool find_run(TTrack &Track, double const Limit, chain &Chain, std::string &Error);
	// the chain read as an alignment: a vertex at the intersection of the tangents of each curve
	struct line_vertex
	{
		glm::dvec2 position{0.0};
		curve shape;
		bool kink{false}; // the straights meet with no curve between them
	};
	static std::vector<line_vertex> recognize_line(chain const &Chain, straight_tolerance const &Tolerance, double const Gauge);
	// regular straight paths continuing a chain end in line, which the chain can take over
	struct straight_run
	{
		std::vector<TTrack *> tracks;
		std::vector<bool> outward;
		double length{0.0};
	};
	static straight_run straight_beyond(chain const &Chain, bool const Atend, straight_tolerance const &Tolerance);
	static double run_reserve(straight_run const &Run);
	static TTrack *take_straight(straight_run const &Run, TTrack *Edge, glm::dvec3 const &Joint, glm::dvec3 const &Cut, std::vector<std::pair<TTrack *, state>> &States, std::vector<TTrack *> &Created);
	// sequence of paths, switches included, which a train can run along
	struct route_span
	{
		TTrack *track{nullptr};
		int path{0};
		bool forward{true};
		double from{0.0}; // chainage, measured in the plan
		double to{0.0};
	};
	struct route
	{
		std::vector<route_span> spans;
		double length{0.0};
	};
	static bool find_route(TTrack *From, TTrack *To, route &Route, std::string &Error);
	// continues both ways from the path, through switches along their path 0
	static route run_route(TTrack &Track, double const Maximum);
	static route route_of(chain const &Chain);
	struct route_sample
	{
		double chainage{0.0};
		glm::dvec3 position{0.0};
		glm::dvec2 direction{0.0, 1.0};
		double grade{0.0};
		double curvature{0.0}; // signed, 1/m, positive turning left
		double cant{0.0}; // m, raise of the outer rail
		std::size_t span{0};
	};
	static std::vector<route_sample> sample_route(route &Route, double const Step);
	static double sampled_elevation(std::vector<route_sample> const &Samples, double const Chainage);
	static double sampled_grade(std::vector<route_sample> const &Samples, double const Chainage);
	static glm::dvec3 sampled_position(std::vector<route_sample> const &Samples, double const Chainage);
	// grade of the path which adjoins the end of the route, positive rising along the route
	static bool adjoining_grade(route const &Route, bool const Atend, double &Grade);
	// path off the route whose end no longer met the adjoining path in height
	struct height_gap
	{
		TTrack *track{nullptr};
		TTrack *neighbour{nullptr};
		double gap{0.0}; // neighbour - track
		bool closed{false}; // the end of the neighbour was brought to the track
	};
	// sets elevations of the paths of the route; regular paths are split at the chainages in Breaks,
	// switches are tilted as a whole in the plane of their grade, branches included, and the regular paths
	// off the route get their ends joined to the switches brought to the height there
	static std::vector<height_gap> apply_profile(route &Route, std::function<double(double)> const &Elevation, std::function<double(double)> const &Grade, std::vector<double> const &Breaks, std::vector<std::pair<TTrack *, state>> &States, std::vector<TTrack *> &Created);
	// carries elevations of the chain over to the new pieces, in proportion of the length
	static void keep_heights(chain const &Chain, std::vector<segment_data> &Pieces);

	struct switch_template
	{
		TTrack *source{nullptr};
		std::array<segment_data, 2> local;
		double length{0.0};
		double ratio{0.0};
		double radius{0.0};
		int count{0};
		std::string label;
		double a{0.0};
		double b{0.0};
		bool double_slip{false};
	};
	static std::vector<switch_template> standard_switch_templates();
	static std::vector<switch_template> find_switch_templates();
	static TTrack *load_path(std::string const &Text, TTrack const &Template, std::string const &Name = {});
	static TTrack *create_path(TTrack const &Style, segment_data const &Path);
	// parameters of a path laid where there's no other path to copy them from
	struct path_style
	{
		double velocity{100.0};
		std::string rail{"rail_screw_used1"};
		std::string ballast{"1435mm/tpbps-new2"};
		std::string rail_profile;
	};
	// the path goes to the layer receiving the items created in the editor
	static TTrack *create_path(path_style const &Style, segment_data const &Path);
	static TTrack *split_path(TTrack &Track, double const T);
	static double nearest_parameter(TTrack const &Track, glm::dvec3 const &Point);
	// point of the first path at the parameter of nearest_parameter()
	static glm::dvec3 point_at(TTrack const &Track, double const T);
	static std::vector<segment_data> place_switch(switch_template const &Template, glm::dvec3 const &Origin, glm::dvec2 const &Direction, int const Side, double const Grade);
	static TTrack *create_switch(switch_template const &Template, std::vector<segment_data> const &Paths, TTrack const &Style, std::string const &Name = {});
	static TTrack *create_turntable(segment_data const &Path, TTrack const *Style, std::string const &Name);
	static void move_straights(std::vector<straight> const &Lines, std::vector<std::pair<glm::dvec3, glm::dvec3>> const &Ends);
	static std::vector<TTrack *> relay(chain const &Chain, std::vector<segment_data> const &Pieces);
	static void retire(TTrack &Track);

	static std::string material_name(material_handle const Material);
	static material_handle fetch_material(std::string const &Name);

	static float texture_height(TTrack const &Track);
	static void texture_height(TTrack &Track, float const Height);
	static double velocity(TTrack const &Track);
	static void velocity(TTrack &Track, double const Velocity);
	static void damage(TTrack &Track, int const Damage);

	// told about the changes of the paths, to let the objects which belong to them follow
	class observer
	{
	  public:
		virtual ~observer() = default;
		virtual void track_captured(TTrack const &Track) = 0;
		virtual void tracks_committed(std::vector<TTrack *> const &Tracks) = 0;
		virtual void track_retired(TTrack &Track) = 0;
		// called before the commit; Original keeps the beginning of the path, given in First
		virtual void track_split(TTrack &Original, TTrack &Created, segment_data const &First) = 0;
	};
	static void observe(observer *Observer) { s_observer = Observer; }

  private:
	static observer *s_observer;
	// sets the length, the ends and their directions of the chain of the tracks given
	static bool complete_chain(chain &Chain, std::string &Error);
	static void disconnect(TTrack &Track);
	static void join(TTrack &Track);
	static void connect(TTrack &Track, bool const Prevside, TTrack *Other, int const Endpointid);
	static void update_transition(TTrack &Track);
	static void bind_switch_path(TTrack &Switch, int const Path);
	static void store_switch_path(TTrack &Switch, int const Path);
	static void rebuild_geometry(TTrack &Track);
	static TTrack *clone(TTrack const &Template);
	static TTrack *load_text(std::string const &Text, TTrack const *Template, std::string const &Name);
	static void move_straight(straight const &Line, glm::dvec3 const &Start, glm::dvec3 const &End, std::function<bool(TTrack const *)> const &Member);
};
