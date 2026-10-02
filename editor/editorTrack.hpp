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

class editor_track
{
  public:
	enum class point_kind { start, control1, control2, end };
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
	};
	static state capture(TTrack const &Track);
	static void apply(TTrack &Track, state const &State);

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

	struct snap_target
	{
		TTrack *track{nullptr};
		point_ref point;
		glm::dvec3 position{0.0};
		glm::dvec3 direction{0.0};
		double distance{0.0};
	};
	static snap_target find_snap_target(TTrack const &Track, glm::dvec3 const &Position, double const Radius, std::vector<TTrack const *> const &Exclude);
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
	};
	static bool find_chain(TTrack *From, TTrack *To, chain &Chain, std::string &Error);

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
	};
	struct straight_tolerance
	{
		double angle{0.0002};
		double offset{0.02};
	};
	static bool is_straight(TTrack const &Track, straight_tolerance const &Tolerance);
	static straight find_straight(TTrack &Track, straight_tolerance const &Tolerance);
	static std::vector<straight> find_straights(double const Minimumlength, straight_tolerance const &Tolerance);
	static std::vector<TTrack *> straight_affected(std::vector<straight> const &Lines);
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
	};
	static bool find_curve(TTrack &Track, straight_tolerance const &Tolerance, double const Gauge, curve &Curve);
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
	};
	static std::vector<switch_template> standard_switch_templates();
	static std::vector<switch_template> find_switch_templates();
	static std::vector<segment_data> place_switch(switch_template const &Template, glm::dvec3 const &Origin, glm::dvec2 const &Direction, int const Side, double const Grade);
	static TTrack *create_switch(switch_template const &Template, std::vector<segment_data> const &Paths, TTrack const &Style);
	static void move_straights(std::vector<straight> const &Lines, std::vector<std::pair<glm::dvec3, glm::dvec3>> const &Ends);
	static std::vector<TTrack *> relay(chain const &Chain, std::vector<segment_data> const &Pieces);
	static void retire(TTrack &Track);
	static void revive(TTrack &Track);

	static std::string material_name(material_handle const Material);
	static material_handle fetch_material(std::string const &Name);

	static float texture_height(TTrack const &Track);
	static void texture_height(TTrack &Track, float const Height);
	static double velocity(TTrack const &Track);
	static void velocity(TTrack &Track, double const Velocity);
	static void damage(TTrack &Track, int const Damage);

  private:
	static void disconnect(TTrack &Track);
	static void join(TTrack &Track);
	static void connect(TTrack &Track, bool const Prevside, TTrack *Other, int const Endpointid);
	static void update_transition(TTrack &Track);
	static void bind_switch_path(TTrack &Switch, int const Path);
	static void store_switch_path(TTrack &Switch, int const Path);
	static void rebuild_geometry(TTrack &Track);
	static TTrack *clone(TTrack const &Template);
	static TTrack *load_path(std::string const &Text, TTrack const &Template);
	static void move_straight(straight const &Line, glm::dvec3 const &Start, glm::dvec3 const &End, std::function<bool(TTrack const *)> const &Member);
};
