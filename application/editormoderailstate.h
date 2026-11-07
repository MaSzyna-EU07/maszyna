/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "utilities/Classes.h"
#include "application/editortool.h"
#include "editor/editorTrack.hpp"
#include "world/Segment.h"

class TTrack;

// the state of the rail tools kept by editor_mode, which the other tools read too: the drag of the track gizmo, the turntable, the
// signals. editor_mode names each type as it did when they were nested in it
namespace rail_state
{

// ImGuizmo-based transform gizmo for the selected node
enum class gizmo_operation { translate, rotate, scale };

// a drag of the track gizmo
struct track_gizmo_drag
{
	// where the drag grabbed the gizmo
	struct grab_point
	{
		glm::dvec3 origin{0.0};
		glm::dvec3 axis{0.0};
		double grabbed{0.0};
	};
	// the frame the track is turned and tilted in
	struct tilt_frame
	{
		glm::dmat3 basis{1.0};
		glm::dvec3 pivot{0.0};
		double grade{0.0};
		double roll{0.0};
		double roll0{0.0};
	};
	// the chainage of the pivot, and of the points of the paths dragged
	struct chain_points
	{
		double pivot_chainage{0.0};
		std::vector<double> chainages;
		std::vector<double> lengths;
	};
	struct switch_joint
	{
		TTrack *track{nullptr};
		editor_track::point_ref own;
		editor_track::point_ref theirs;
		glm::dvec3 point{0.0};
		glm::dvec3 tangent{0.0};
		double roll{0.0};
		bool moved{false};
		std::vector<std::pair<TTrack *, std::vector<segment_data>>> cluster;
		std::vector<std::pair<TTrack *, std::vector<editor_track::joint>>> joints;
	};
	gizmo_operation operation{gizmo_operation::translate};
	bool on_control{false};
	double angle{0.0};
	int path{-1};
	bool atend{true};
	std::size_t pieces{1};
	std::vector<segment_data> paths;
	editor_track::point_ref point;
	grab_point grab;
	double length{0.0};
	double stretch{0.0};
	std::vector<std::pair<TTrack *, editor_track::point_ref>> partners;
	glm::dvec3 fixed{0.0};
	std::vector<std::pair<TTrack *, std::vector<int>>> held;
	std::vector<TTrack *> extra;
	int ring{-1};
	tilt_frame tilt;
	chain_points chain;
	std::vector<switch_joint> switches;
};

struct turntable_template
{
	std::string file;
	std::string name;
	double length{0.0};
	int angles{0};
	int positions{0};
	int track{0};
	int yaw{0};
	std::vector<double> fixed;
};
struct turntable_fit
{
	editor_track::snap_target end;
	double heading{0.0};
	double radius{0.0};
	double length{0.0};
	std::vector<segment_data> pieces;
	std::string issue;
	bool chosen{true};
	TTrack *replaces{nullptr};
};
struct turntable_tool
{
	// the templates of the turntables, the one chosen and the name of the next one
	struct template_bank
	{
		std::vector<turntable_template> templates;
		int revision{-1}; // of the include bank the templates were read from
		int chosen{-1};
		std::array<char, 64> name{};
	};
	// where the turntable placed goes, as the mouse pressed it
	struct placement
	{
		glm::dvec3 centre{0.0};
		double yaw{0.0};
		glm::vec2 pressed{0.f};
		editor_track::snap_target snap;
	};
	// the reach of the tracks fitted to the exits, their radius, their stubs and the step of the exits
	struct exit_shape
	{
		float reach{80.f};
		float radius{150.f};
		float stub{25.f};
		float step{0.f};
	};
	template_bank bank;
	bool placing{false};
	placement place;
	TTrack *table{nullptr};
	scene::instance_handle include{0};
	int kind{-1};
	double length{0.0};
	double table_yaw{0.0};
	std::vector<double> exits;
	exit_shape shape;
	std::vector<turntable_fit> fits;
	int hovered{-1};
	double heading{0.0};
	double exit_length{0.0};
	std::string status;
	std::string error;
};

struct signal_template
{
	std::string file;
	std::string name;
	std::string description;
	std::string kind;
	std::string mount;
	std::string lamps;
	std::string lean;
	std::string read;
	int parameters{0};
	bool plate{false};
	bool linked{false};
	std::vector<int> speeds;
	int aspects{0}; // of a block signal, 3 or 4
	int arms{0}; // of a semaphore, 1 or 2, from its description
	// the aspects its description lists, by name (Sr1, Od2...) with the event giving each; empty for a template not described so
	std::vector<std::pair<std::string, std::string>> aspect_events;
	// of a warning disc: its event for each speed class of the signal after it, stop, vmax, 100, 60, 40
	std::vector<std::pair<std::string, std::string>> repeats;
};
struct signal_spot
{
	TTrack *track{nullptr};
	int path{0};
	glm::dvec3 axis{0.0};
	glm::dvec2 travel{0.0, 1.0};
	int side{1};
	int list{2};
	double offset{0.0};
	glm::dvec3 position{0.0};
	double yaw{0.0};
};
struct signal_standing
{
	standing_template include;
	int kind{-1};
	std::string name;
	std::string read;
	TTrack *track{nullptr};
	int list{0};
};
struct signal_tool
{
	// the name, the plate and the linked signal typed for the signal placed
	struct signal_labels
	{
		std::array<char, 64> name{"A"};
		std::array<char, 64> plate{};
		std::array<char, 64> linked{"none"};
	};
	// the signals standing in the scenery, as read at the step of the history and the time kept
	struct standing_signals
	{
		std::vector<signal_standing> standing;
		std::size_t standing_history{static_cast<std::size_t>(-1)};
		std::size_t standing_redo{static_cast<std::size_t>(-1)};
		double standing_time{-1.0};
	};
	// the signal moved: where it was grabbed, where it stood and where it goes
	struct signal_drag
	{
		glm::dvec2 grab{0.0};
		signal_spot home;
		bool home_valid{false};
		signal_spot moved;
		bool moved_valid{false};
	};
	std::vector<signal_template> templates;
	int revision{-1};
	int kind{0};
	std::array<int, 7> chosen{-1, -1, -1, -1, -1, -1, -1};
	bool armed{false}; // a click by a track places the chosen template
	std::array<char, 64> filter{};
	signal_labels labels;
	float height{0.0f};
	bool placing{false};
	signal_spot press;
	glm::vec2 pressed{0.f};
	int facing{0};
	standing_signals known;
	int hover{-1};
	scene::instance_handle selected{0};
	bool moving{false};
	signal_drag drag;
	std::string status;
};

} // namespace rail_state
