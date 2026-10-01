/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "world/Segment.h"
#include "utilities/Classes.h"

class TTrack;

// Editing of paths (tracks, roads, rivers) loaded from the scenery.
//
// The source of truth for path geometry is TTrack::m_paths (the same data the scenery export writes).
// Edits modify m_paths and then commit() rebuilds everything derived from it: the segments, placement
// in the scene cells, connections with neighbouring paths and the render geometry.
class editor_track
{
  public:
	// identifies an editable point of a path
	enum class point_kind { start, control1, control2, end };
	struct point_ref
	{
		int path{-1}; // index into TTrack::m_paths
		point_kind kind{point_kind::start};
		bool valid() const { return path >= 0; }
		bool operator==(point_ref const &Other) const { return path == Other.path && kind == Other.kind; }
	};

	// snapshot of the editable state of a path, used by the undo history
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
	// restores provided state; follow with commit() to rebuild the path
	static void apply(TTrack &Track, state const &State);

	// checks whether geometry of the path can be modified. returns: true if it can, otherwise false and the reason
	static bool can_edit_geometry(TTrack const &Track, std::string &Reason);
	// checks whether the path type is supported by the editor at all
	static bool is_supported(TTrack const &Track);

	// rebuilds provided paths after modification of their source data (m_paths): segments, placement in the region,
	// connections with neighbours and render geometry of the paths and all their old and new neighbours
	static void commit(std::vector<TTrack *> const &Tracks);
	// rebuilds the path after change of parameters which don't affect its course (width, textures, etc)
	static void commit_parameters(TTrack &Track);

	// world position of specified point. for control points it's the absolute position of the handle;
	// a zero (straight) control vector is presented at 1/3 of the chord so it can be grabbed
	static glm::dvec3 point_position(TTrack const &Track, point_ref const &Point);
	// moves specified point to provided world position. moving an end point carries its control vector along.
	// switches keep the start points of both their paths together
	static void move_point(TTrack &Track, point_ref const &Point, glm::dvec3 const &Position);
	// moves the whole path by provided offset
	static void translate(TTrack &Track, glm::dvec3 const &Offset);
	// rotates the whole path around vertical axis going through provided pivot point
	static void rotate_y(TTrack &Track, glm::dvec3 const &Pivot, double const Angle);
	// pivot used for rotation of the whole path
	static glm::dvec3 pivot(TTrack const &Track);

	// lists paths currently connected to provided path
	static std::vector<TTrack *> neighbours(TTrack const &Track);
	// lists end points of other paths connected to the end point located at provided position,
	// as pairs of the neighbour and its matching point
	static std::vector<std::pair<TTrack *, point_ref>> connected_points(TTrack const &Track, glm::dvec3 const &Position);
	// checks whether end point located at provided position is connected to another path
	static bool is_connected(TTrack const &Track, point_ref const &Point);

	// candidate to connect a dragged end point with
	struct snap_target
	{
		TTrack *track{nullptr};
		point_ref point;
		glm::dvec3 position{0.0};
		glm::dvec3 direction{0.0}; // direction from the point into the target path
		double distance{0.0};
	};
	// finds the nearest free end point of another path of the same category as provided path, within specified radius.
	// paths listed in Exclude are skipped, and connections to them don't count as occupying an end point
	static snap_target find_snap_target(TTrack const &Track, glm::dvec3 const &Position, double const Radius, std::vector<TTrack const *> const &Exclude);
	// places the point at the target and optionally aligns the control vector with the target path
	static void snap_point(TTrack &Track, point_ref const &Point, snap_target const &Target, bool const Aligntangent);

	// names of the materials used by the path; "none" for missing ones
	static std::string material_name(material_handle const Material);
	static material_handle fetch_material(std::string const &Name);

	// height of the trackbed edge as written in the scenery (without roll fix adjustment)
	static float texture_height(TTrack const &Track);
	static void texture_height(TTrack &Track, float const Height);
	static double velocity(TTrack const &Track);
	static void velocity(TTrack &Track, double const Velocity);
	static void damage(TTrack &Track, int const Damage);

  private:
	// unlinks the path from all its neighbours
	static void disconnect(TTrack &Track);
	// links the path with paths whose free end points match its own free end points
	static void join(TTrack &Track);
	// links specified end of a regular path with matching end of another path
	static void connect(TTrack &Track, bool const Prevside, TTrack *Other, int const Endpointid);
	// recalculates the trackbed transition flag
	static void update_transition(TTrack &Track);
	// sets active connections of a switch to specified path, without moving the blades
	static void bind_switch_path(TTrack &Switch, int const Path);
	static void store_switch_path(TTrack &Switch, int const Path);
	// rebuilds render geometry of the path in the bank of the section it's placed in
	static void rebuild_geometry(TTrack &Track);
};
