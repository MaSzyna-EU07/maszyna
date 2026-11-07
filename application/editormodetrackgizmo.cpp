/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "editor/editorGeometry.hpp"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <array>
#include <cmath>

namespace
{

using geometry::bezier;
using geometry::level;
using geometry::plan_of;
using geometry::turned;

bool mouse_along(glm::dvec3 const &Origin, glm::dvec3 const &Axis, double &Along)
{
	screen_projection const projection;
	glm::dvec3 origin;
	glm::dvec3 direction;
	projection.ray(ImGui::GetIO().MousePos, origin, direction);
	auto const b{glm::dot(Axis, direction)};
	auto const denominator{1.0 - b * b};
	if (denominator < 1e-6)
		return false;
	auto const w{Origin - origin};
	Along = (b * glm::dot(direction, w) - glm::dot(Axis, w)) / denominator;
	return true;
}

glm::mat4 gizmo_along(glm::dvec3 const &Position, glm::dvec3 const &Axis)
{
	auto gizmo{glm::translate(glm::mat4(1.0f), glm::vec3(Position))};
	if (glm::length(Axis) < 1e-9)
		return gizmo;
	auto const z{glm::normalize(Axis)};
	auto x{glm::cross(glm::dvec3{0.0, 1.0, 0.0}, z)};
	x = glm::length(x) > 1e-6 ? glm::normalize(x) : glm::dvec3{1.0, 0.0, 0.0};
	auto const y{glm::cross(z, x)};
	gizmo[0] = glm::vec4(glm::vec3(x), 0.0f);
	gizmo[1] = glm::vec4(glm::vec3(y), 0.0f);
	gizmo[2] = glm::vec4(glm::vec3(z), 0.0f);
	return gizmo;
}

void add_once(std::vector<TTrack *> &Tracks, TTrack *Track)
{
	if (std::ranges::find(Tracks, Track) == Tracks.end())
		Tracks.emplace_back(Track);
}

bool editable(TTrack const &Track)
{
	std::string reason;
	return editor_track::can_edit_geometry(Track, reason);
}

double snapped_grade(double const Grade)
{
	return std::round(Grade * 1000.0) / 1000.0;
}

double snapped_cant(double const Roll0, double const Roll)
{
	auto const cant{std::round(1500.0 * std::sin(glm::radians(Roll0 + Roll)) / 5.0) * 5.0 / 1500.0};
	return glm::degrees(std::asin(std::clamp(cant, -1.0, 1.0))) - Roll0;
}

// the turn and the tilt the paths of a switch are carried by with the joint they hang on: by the cosine and the sine of the turn
// in the plan, toward the direction the joint has now, up the grade along it
struct carry_motion
{
	double c;
	double s;
	glm::dvec3 to;
	double grade;

	glm::dvec3 turn(glm::dvec3 const &Vector) const { return glm::dvec3{Vector.x * c - Vector.z * s, Vector.y, Vector.z * c + Vector.x * s}; }
	double tilt(glm::dvec3 const &Vector) const { return (Vector.x * to.x + Vector.z * to.z) * grade; }
	// a path turned about the joint as it was and moved with it to the point
	void carry(segment_data &Path, glm::dvec3 const &Joint, glm::dvec3 const &Point) const
	{
		for (auto const index : {segment_data::point::start, segment_data::point::end})
		{
			auto const moved{turn(Path.points[index] - Joint)};
			Path.points[index] = Point + moved + glm::dvec3{0.0, tilt(moved), 0.0};
		}
		for (auto const index : {segment_data::point::control1, segment_data::point::control2})
		{
			auto &lever{Path.points[index]};
			if (lever == glm::dvec3{})
				continue;
			lever = turn(lever);
			lever.y += tilt(lever);
		}
	}
};

// the direction the control vector of the edited end is held along by a partner which doesn't move: the one of its own control
// vector, or of its chord when it has none
glm::dvec3 held_direction(TTrack const &Other, editor_track::point_ref const &Theirs)
{
	auto const &path{Other.m_paths[Theirs.path]};
	bool const start{Theirs.kind == editor_track::point_kind::start};
	auto const lever{path.points[start ? segment_data::point::control1 : segment_data::point::control2]};
	auto const chord{path.points[start ? segment_data::point::end : segment_data::point::start] - path.points[start ? segment_data::point::start : segment_data::point::end]};
	return lever != glm::dvec3{} ? lever : chord;
}

// the ends of all the paths of the track, the start and the end of each in turn
std::vector<editor_track::point_ref> path_ends(TTrack const &Track)
{
	std::vector<editor_track::point_ref> ends;
	for (int i = 0; i < static_cast<int>(Track.m_paths.size()); ++i)
		for (auto const kind : {editor_track::point_kind::start, editor_track::point_kind::end})
			ends.push_back({i, kind});
	return ends;
}

// the switches going with the first one: the ones joined to it, and to those, which can move with it
template <typename Movable>
std::vector<TTrack *> cluster_of(TTrack *First, Movable const &Moves)
{
	std::vector members{First};
	std::size_t k{0};
	while (k < members.size())
	{
		for (auto *next : editor_track::neighbours(*members[k]))
			if (Moves(next) && std::ranges::find(members, next) == members.end())
				members.push_back(next);
		++k;
	}
	return members;
}

} // namespace

editor_mode::gizmo_operation editor_mode::track_gizmo_operation(TTrack const &Track, bool const Pointmode) const
{
	using enum rail_state::gizmo_operation;
	if (m_gizmo_op == translate || (Pointmode && false == editor_track::is_end(m_track_point.kind)))
		return m_gizmo_op;
	if (m_gizmo_op == rotate)
		return rotate;
	auto const picked{Pointmode ? m_track_point : editor_track::point_ref{}};
	return editor_track::stretch_end(Track, picked).has_value() ? scale : translate;
}

glm::mat4 editor_mode::track_gizmo_frame(TTrack const &Track, bool const Pointmode, gizmo_operation const Operation) const
{
	auto anchor{Pointmode ? editor_track::point_position(Track, m_track_point) : editor_track::pivot(Track)};
	bool const controlmode{Pointmode && false == editor_track::is_end(m_track_point.kind)};
	glm::dvec3 axis{0.0};
	if (Operation == gizmo_operation::scale)
	{
		if (controlmode)
			axis = editor_track::control_vector(Track, m_track_point);
		else if (auto const end{editor_track::stretch_end(Track, Pointmode ? m_track_point : editor_track::point_ref{})})
			std::tie(anchor, axis) = editor_track::end_axis(Track.m_paths[end->path], end->atend);
	}
	else if (Operation == gizmo_operation::rotate)
	{
		if (controlmode)
			axis = level(editor_track::control_vector(Track, m_track_point));
		else if (Pointmode)
			axis = level(bezier{Track.m_paths[m_track_point.path]}.first(m_track_point.kind == editor_track::point_kind::start ? 0.0 : 1.0));
		else
		{
			auto const middle{editor_track::middle(Track)};
			anchor = middle.point;
			axis = level(middle.tangent);
		}
	}
	return gizmo_along(anchor - glm::dvec3{GfxRenderer->Camera_Position()}, axis);
}

void editor_mode::track_gizmo_hold()
{
	auto &drag{m_track_gizmo_drag};
	drag.held.clear();
	for (auto *track : m_track_drag)
	{
		if (drag.operation == gizmo_operation::scale && false == drag.on_control && track == selected_track())
			continue;
		auto held{editor_track::hold_controls(*track)};
		if (false == held.empty())
			drag.held.emplace_back(track, std::move(held));
	}
}

void editor_mode::track_gizmo_settle()
{
	for (auto const &[track, held] : m_track_gizmo_drag.held)
		editor_track::settle_controls(*track, held);
	m_track_gizmo_drag.held.clear();
}

void editor_mode::track_gizmo_conform(TTrack &Track) const
{
	if (Track.eType != tt_Switch)
		return;
	editor_track::conform_switch(Track);
	editor_track::follow(Track, m_track_joints);
	editor_track::follow_grade(Track, m_track_joints);
}

bool editor_mode::track_gizmo_clustered(TTrack const *Track) const
{
	return std::ranges::any_of(m_track_gizmo_drag.switches, [Track](auto const &Link) { return std::ranges::any_of(Link.cluster, [Track](auto const &Member) { return Member.first == Track; }); });
}

void editor_mode::track_gizmo_stretch(TTrack &Track, double const Length)
{
	auto &drag{m_track_gizmo_drag};
	auto const pieces{editor_track::stretched(drag.paths[drag.path], drag.atend, Length)};
	if (pieces.empty())
		return;
	auto paths{drag.paths};
	paths.erase(paths.begin() + drag.path);
	paths.insert(paths.begin() + drag.path, pieces.begin(), pieces.end());
	Track.m_paths = std::move(paths);
	drag.pieces = pieces.size();
	drag.stretch = Length;
	if (drag.point.valid())
		m_track_point = editor_track::stretch_remap(drag.point, drag.path, drag.pieces);
	auto joints{m_track_joints};
	for (auto &joint : joints)
		joint.own = editor_track::stretch_remap(joint.own, drag.path, drag.pieces);
	editor_track::follow(Track, joints);
	track_gizmo_spill(Track, pieces.size());
	m_track_dirty = true;
}

void editor_mode::track_gizmo_spill(TTrack &Track, std::size_t const Pieces)
{
	auto &drag{m_track_gizmo_drag};
	std::vector<segment_data> rest;
	if (Track.eType == tt_Normal && Pieces > 1)
	{
		auto const first{Track.m_paths.begin() + drag.path};
		auto const keep{drag.atend ? first : first + (Pieces - 1)};
		for (auto piece = first; piece != first + Pieces; ++piece)
			if (piece != keep)
				rest.push_back(*piece);
		// the piece kept, taken out before the pieces are erased
		auto kept{std::move(*keep)};
		Track.m_paths.erase(first, first + Pieces);
		Track.m_paths.insert(Track.m_paths.begin() + drag.path, std::move(kept));
		if (drag.point.valid())
			m_track_point = drag.point;
	}
	while (drag.extra.size() > rest.size())
	{
		auto *surplus{drag.extra.back()};
		drag.extra.pop_back();
		std::erase(m_track_drag, surplus);
		editor_track::retire(*surplus);
	}
	for (std::size_t i = 0; i < rest.size(); ++i)
	{
		if (i < drag.extra.size())
		{
			drag.extra[i]->m_paths = {rest[i]};
			continue;
		}
		auto *created{editor_track::create_path(Track, rest[i])};
		drag.extra.push_back(created);
		m_track_drag.push_back(created);
		editor_track::commit({created});
	}
}

void editor_mode::track_gizmo_control(TTrack &Track, glm::dvec3 Control)
{
	auto const &drag{m_track_gizmo_drag};
	if (drag.fixed != glm::dvec3{})
		Control = drag.fixed * std::max(0.5, glm::dot(Control, drag.fixed));
	if (glm::length(Control) < 0.5)
		Control = glm::length(Control) > 1e-9 ? glm::normalize(Control) * 0.5 : drag.grab.axis * 0.5;
	editor_track::move_point(Track, drag.point, editor_track::point_position(Track, editor_track::end_of(drag.point)) + Control);
	editor_track::align_controls(Track, drag.point, drag.partners);
	track_gizmo_conform(Track);
	m_track_dirty = true;
}

void editor_mode::track_gizmo_point(TTrack &Track, glm::dvec3 const &Position) const
{
	editor_track::move_point(Track, m_track_point, Position);
	track_gizmo_conform(Track);
	auto const moved{editor_track::point_position(Track, m_track_point)};
	for (auto const &[track, point] : m_track_drag_points)
		if (false == track_gizmo_clustered(track))
			editor_track::move_point(*track, point, moved);
}

void editor_mode::track_gizmo_carry(track_gizmo_drag::switch_joint const &Link, glm::dvec3 const &Point, glm::dvec3 const &From, glm::dvec3 const &To, double const Grade) const
{
	auto const c{glm::dot(From, To)};
	auto const s{From.x * To.z - From.z * To.x};
	carry_motion const motion{c, s, To, Grade};
	for (auto const &[member, paths] : Link.cluster)
	{
		member->m_paths = paths;
		for (auto &path : member->m_paths)
			motion.carry(path, Link.point, Point);
	}
}

void editor_mode::track_gizmo_roll_switch(track_gizmo_drag::switch_joint const &Link, double const Roll) const
{
	auto &main{Link.track->m_paths[0]};
	bool const start{Link.theirs.kind == editor_track::point_kind::start};
	auto value{start ? -Roll : Roll};
	if (Link.theirs.path != 0)
	{
		auto const t{start ? 0.0 : 1.0};
		if (glm::dot(plan_of(bezier{Link.track->m_paths[Link.theirs.path]}.first(t)), plan_of(bezier{main}.first(t))) < 0.0)
			value = -value;
	}
	main.rolls = {static_cast<float>(value), static_cast<float>(value)};
	editor_track::conform_switch(*Link.track);
}

void editor_mode::track_gizmo_switches(TTrack const &Track)
{
	for (auto &link : m_track_gizmo_drag.switches)
	{
		if (link.own.path >= static_cast<int>(Track.m_paths.size()))
			continue;
		auto const point{editor_track::point_position(Track, link.own)};
		auto const tangent{editor_track::outward(Track, link.own)};
		auto const roll{editor_track::joint_roll(Track, link.own)};
		auto const from{level(link.tangent)};
		auto const to{level(tangent)};
		auto const grade{geometry::grade_of(tangent) - geometry::grade_of(link.tangent)};
		if (bool const changed{glm::distance(point, link.point) > 1e-9 || glm::distance(from, to) > 1e-12 || std::abs(grade) > 1e-12 || std::abs(roll - link.roll) > 1e-6}; false == changed && false == link.moved)
			continue;
		link.moved = true;
		track_gizmo_carry(link, point, from, to, grade);
		if (std::abs(roll - link.roll) > 1e-6)
			track_gizmo_roll_switch(link, roll);
		for (auto const &[member, joints] : link.joints)
		{
			editor_track::follow(*member, joints);
			editor_track::follow_grade(*member, joints);
			editor_track::match_rolls(*member, joints);
		}
	}
}

void editor_mode::track_gizmo_take_joints(TTrack const &Track)
{
	m_track_joints = editor_track::joints(Track);
	for (auto const &joint : m_track_joints)
		add_once(m_track_drag, joint.other);
}

void editor_mode::track_gizmo_take_points(TTrack const &Track)
{
	for (auto const &connection : editor_track::connected_points(Track, editor_track::point_position(Track, m_track_point)))
	{
		if (false == editable(*connection.first))
			continue;
		m_track_drag_points.emplace_back(connection);
		add_once(m_track_drag, connection.first);
	}
}

void editor_mode::track_gizmo_take_partners(TTrack &Track)
{
	auto &drag{m_track_gizmo_drag};
	auto const end{editor_track::end_of(m_track_point)};
	for (auto const &partner : editor_track::partners(Track, end))
	{
		auto const &[other, theirs]{partner};
		bool const own{other == &Track};
		if (bool const fixed{own ? m_track_point.path != 0 : other->eType != tt_Normal || false == editable(*other)}; fixed)
		{
			if (auto const along{held_direction(*other, theirs)}; glm::length(along) > 1e-9)
				drag.fixed = (own && theirs.kind == end.kind ? 1.0 : -1.0) * glm::normalize(along);
			continue;
		}
		if (false == m_track_drag_connected && false == own)
			continue;
		drag.partners.push_back(partner);
		add_once(m_track_drag, other);
	}
}

void editor_mode::track_gizmo_take_switch_joints(TTrack const &Track, bool const Controlmode)
{
	auto const edited{editor_track::point_position(Track, editor_track::end_of(m_track_point))};
	for (auto const &joint : editor_track::joints(Track))
	{
		if (Controlmode && glm::distance(editor_track::point_position(Track, joint.own), edited) < geometry::same_point)
			continue;
		m_track_joints.push_back(joint);
		add_once(m_track_drag, joint.other);
	}
}

editor_mode::track_gizmo_drag::switch_joint editor_mode::track_gizmo_link(TTrack const &Track, editor_track::point_ref const &Own, std::pair<TTrack *, editor_track::point_ref> const &Connection, std::vector<TTrack *> const &Members)
{
	track_gizmo_drag::switch_joint link{Connection.first, Own, Connection.second, editor_track::point_position(Track, Own), editor_track::outward(Track, Own), editor_track::joint_roll(Track, Own)};
	for (auto *member : Members)
	{
		link.cluster.emplace_back(member, std::vector<segment_data>{});
		auto joints{editor_track::joints(*member)};
		std::erase_if(joints, [&Track](editor_track::joint const &Joint) { return Joint.other == &Track; });
		add_once(m_track_drag, member);
		for (auto const &joint : joints)
			add_once(m_track_drag, joint.other);
		link.joints.emplace_back(member, std::move(joints));
	}
	return link;
}

void editor_mode::track_gizmo_take_switches(TTrack &Track)
{
	auto &drag{m_track_gizmo_drag};
	auto const movable = [this, &Track](TTrack const *Other) { return Other != &Track && Other->eType == tt_Switch && Other->m_paths.size() == 2 && editable(*Other) && false == track_gizmo_clustered(Other); };
	for (auto const &own : path_ends(Track))
		for (auto const &connection : editor_track::connected_points(Track, editor_track::point_position(Track, own)))
			if (movable(connection.first))
				drag.switches.push_back(track_gizmo_link(Track, own, connection, cluster_of(connection.first, movable)));
}

void editor_mode::track_gizmo_take_double(TTrack &Track)
{
	auto const set{double_of(Track)};
	if (false == set.has_value() || Track.eType != tt_Switch)
		return;
	auto const *other{&Track == set->first ? set->second : set->first};
	if (false == editable(*set->link) || false == editable(*other) || track_gizmo_clustered(set->link))
		return;
	editor_track::point_ref const own{0, &Track == set->first ? editor_track::point_kind::end : editor_track::point_kind::start};
	auto const position{editor_track::point_position(Track, own)};
	for (auto const &connection : editor_track::connected_points(Track, position))
	{
		if (connection.first != set->link)
			continue;
		std::erase_if(m_track_joints, [&set](editor_track::joint const &Joint) { return Joint.other == set->link; });
		auto const movable = [&set, other](TTrack const *Each) { return Each == set->link || Each == other; };
		m_track_gizmo_drag.switches.push_back(track_gizmo_link(Track, own, connection, cluster_of(connection.first, movable)));
		return;
	}
}

void editor_mode::track_gizmo_measure_rotation(TTrack &Track, bool const Pointmode)
{
	auto &drag{m_track_gizmo_drag};
	auto const frame{track_gizmo_frame(Track, Pointmode, gizmo_operation::rotate)};
	drag.tilt.basis = glm::dmat3(glm::mat3(frame));
	drag.tilt.pivot = glm::dvec3{GfxRenderer->Camera_Position()} + glm::dvec3(frame[3]);
	double along{0.0};
	for (auto const &path : Track.m_paths)
	{
		drag.chain.chainages.push_back(along);
		drag.chain.lengths.push_back(bezier{path}.plan_length());
		along += drag.chain.lengths.back();
	}
	auto const end{editor_track::switch_main(Track, editor_track::end_of(drag.point))};
	if (Track.eType == tt_Switch && false == Track.m_paths.empty())
	{
		drag.chain.pivot_chainage = editor_track::switch_station(Track, drag.tilt.pivot);
		drag.tilt.roll0 = end.valid() && end.kind == editor_track::point_kind::end ? Track.m_paths[0].rolls[1] : Track.m_paths[0].rolls[0];
	}
	else if (end.valid() && end.path < static_cast<int>(drag.chain.chainages.size()))
	{
		drag.chain.pivot_chainage = drag.chain.chainages[end.path] + (end.kind == editor_track::point_kind::end ? drag.chain.lengths[end.path] : 0.0);
		drag.tilt.roll0 = end.kind == editor_track::point_kind::start ? Track.m_paths[end.path].rolls[0] : Track.m_paths[end.path].rolls[1];
	}
	else
	{
		drag.chain.pivot_chainage = along * 0.5;
		drag.tilt.roll0 = Track.m_paths.empty() ? 0.0 : Track.m_paths.front().rolls[0];
	}
}

void editor_mode::track_gizmo_measure(TTrack &Track, bool const Pointmode, gizmo_operation const Operation)
{
	auto &drag{m_track_gizmo_drag};
	if (drag.on_control)
	{
		drag.grab.origin = editor_track::point_position(Track, m_track_point);
		auto const lever{editor_track::control_vector(Track, m_track_point)};
		drag.length = glm::length(lever);
		drag.grab.axis = drag.length > 1e-9 ? lever / drag.length : glm::dvec3{0.0, 0.0, 1.0};
	}
	else if (auto const end{Operation == gizmo_operation::scale ? editor_track::stretch_end(Track, drag.point) : std::nullopt})
	{
		drag.path = end->path;
		drag.atend = end->atend;
		drag.paths = Track.m_paths;
		std::tie(drag.grab.origin, drag.grab.axis) = editor_track::end_axis(Track.m_paths[drag.path], drag.atend);
		drag.length = bezier{Track.m_paths[drag.path]}.plan_length();
	}
	drag.stretch = drag.length;
	if (Operation == gizmo_operation::rotate)
		track_gizmo_measure_rotation(Track, Pointmode);
}

void editor_mode::track_gizmo_begin(TTrack &Track, bool const Pointmode, gizmo_operation const Operation)
{
	bool const controlmode{Pointmode && false == editor_track::is_end(m_track_point.kind)};
	auto &drag{m_track_gizmo_drag};
	m_track_gizmo_using = true;
	m_track_pivot = editor_track::pivot(Track);
	m_track_drag = {&Track};
	m_track_drag_points.clear();
	m_track_joints.clear();
	m_track_snap = {};
	drag = {};
	drag.operation = Operation;
	drag.on_control = controlmode;
	drag.point = Pointmode ? m_track_point : editor_track::point_ref{};
	bool const whole{false == Pointmode || (false == controlmode && Operation != gizmo_operation::translate)};
	if (whole && m_track_drag_connected)
		track_gizmo_take_joints(Track);
	if (Pointmode && false == controlmode && Operation == gizmo_operation::translate && m_track_drag_connected)
		track_gizmo_take_points(Track);
	if (controlmode)
		track_gizmo_take_partners(Track);
	if (false == whole && Track.eType == tt_Switch && m_track_drag_connected)
		track_gizmo_take_switch_joints(Track, controlmode);
	if (whole)
		track_gizmo_take_double(Track);
	if (m_track_drag_connected)
		track_gizmo_take_switches(Track);
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *dragged : m_track_drag)
		states.emplace_back(dragged, editor_track::capture(*dragged));
	push_track_snapshot(std::move(states));
	records_watch_begin();
	track_gizmo_hold();
	for (auto &link : drag.switches)
		for (auto &[member, paths] : link.cluster)
			paths = member->m_paths;
	track_gizmo_measure(Track, Pointmode, Operation);
}

void editor_mode::track_gizmo_turn_direction(TTrack &Track, double const Angle)
{
	auto &drag{m_track_gizmo_drag};
	auto target{Angle};
	if (Global.ctrlState)
		target = glm::radians(std::round(glm::degrees(target) / 5.0) * 5.0);
	auto const step{std::remainder(target - drag.angle, 2.0 * glm::pi<double>())};
	if (std::abs(step) < 1e-9)
		return;
	drag.angle += step;
	if (drag.on_control)
	{
		auto const lever{editor_track::control_vector(Track, drag.point)};
		auto const plan{turned(plan_of(lever), -step)};
		track_gizmo_control(Track, {plan.x, lever.y, plan.y});
		return;
	}
	editor_track::rotate_y(Track, drag.tilt.pivot, step);
	editor_track::follow(Track, m_track_joints);
	m_track_dirty = true;
}

void editor_mode::track_gizmo_turn_grade(TTrack &Track, double const Angle)
{
	auto &drag{m_track_gizmo_drag};
	auto target{-std::tan(std::clamp(Angle, -1.4, 1.4))};
	if (Global.ctrlState)
		target = snapped_grade(target);
	auto const step{target - drag.tilt.grade};
	if (std::abs(step) < 1e-9)
		return;
	drag.tilt.grade = target;
	if (drag.on_control)
	{
		auto const lever{editor_track::control_vector(Track, drag.point)};
		auto const run{glm::length(plan_of(lever))};
		auto const along{glm::dot(level(lever), drag.tilt.basis[2])};
		track_gizmo_control(Track, {lever.x, lever.y + step * run * (along >= 0.0 ? 1.0 : -1.0), lever.z});
		return;
	}
	auto const count{editor_track::driven_paths(Track)};
	for (std::size_t i = 0; i < count && i < drag.chain.chainages.size(); ++i)
	{
		auto &path{Track.m_paths[i]};
		auto const from{drag.chain.chainages[i] - drag.chain.pivot_chainage};
		auto const to{from + drag.chain.lengths[i]};
		path.points[segment_data::point::start].y += step * from;
		path.points[segment_data::point::end].y += step * to;
		auto &control1{path.points[segment_data::point::control1]};
		auto &control2{path.points[segment_data::point::control2]};
		control1.y += step * glm::length(plan_of(control1));
		control2.y -= step * glm::length(plan_of(control2));
	}
	editor_track::conform_switch(Track);
	editor_track::follow(Track, m_track_joints);
	editor_track::follow_grade(Track, m_track_joints);
	m_track_dirty = true;
}

void editor_mode::track_gizmo_turn_cant(TTrack &Track, double const Angle)
{
	auto &drag{m_track_gizmo_drag};
	auto const end{editor_track::end_of(drag.point)};
	auto const canted{editor_track::switch_main(Track, end)};
	auto target{glm::degrees(drag.point.kind == editor_track::point_kind::control2 ? -Angle : Angle)};
	if (Global.ctrlState)
		target = snapped_cant(drag.tilt.roll0, target);
	auto const step{static_cast<float>(target - drag.tilt.roll)};
	if (std::abs(step) < 1e-6f)
		return;
	drag.tilt.roll = target;
	for (int i = 0; i < static_cast<int>(Track.m_paths.size()); ++i)
	{
		auto &path{Track.m_paths[i]};
		if (false == canted.valid() || (i == canted.path && canted.kind == editor_track::point_kind::start))
			path.rolls[0] += step;
		if (false == canted.valid() || (i == canted.path && canted.kind == editor_track::point_kind::end))
			path.rolls[1] += step;
	}
	track_gizmo_conform(Track);
	editor_track::match_rolls(Track, m_track_joints);
	for (auto const &[other, theirs] : drag.partners)
		if (other != &Track)
			editor_track::roll_joined(*other, theirs, static_cast<float>(editor_track::joint_roll(Track, end)));
	m_track_dirty = true;
}

void editor_mode::track_gizmo_turn(TTrack &Track, int const Axis, double const Angle)
{
	if (auto const &drag{m_track_gizmo_drag}; drag.on_control && drag.fixed != glm::dvec3{})
		return;
	if (Axis == 1)
		track_gizmo_turn_direction(Track, Angle);
	else if (Axis == 0)
		track_gizmo_turn_grade(Track, Angle);
	else
		track_gizmo_turn_cant(Track, Angle);
}

void editor_mode::track_gizmo_length(TTrack &Track, double Length)
{
	auto &drag{m_track_gizmo_drag};
	Length = std::max(drag.on_control ? 0.5 : 1.0, Length);
	if (std::abs(Length - drag.stretch) < 1e-6)
		return;
	if (drag.on_control)
	{
		drag.stretch = Length;
		track_gizmo_control(Track, drag.grab.axis * Length);
	}
	else if (drag.path >= 0)
		track_gizmo_stretch(Track, Length);
}

void editor_mode::track_gizmo_end(TTrack &Track)
{
	if (m_track_snap.track != nullptr && m_track_gizmo_drag.operation == gizmo_operation::translate && m_track_point.valid())
		editor_track::snap_point(Track, m_track_point, m_track_snap, m_track_align_tangent);
	track_gizmo_settle();
	m_track_dirty = true;
	m_track_snap = {};
	commit_track_drag(true);
	if (false == m_track_gizmo_drag.extra.empty() && false == m_history.empty())
	{
		auto &created{m_history.back().created};
		created.insert(created.end(), m_track_gizmo_drag.extra.begin(), m_track_gizmo_drag.extra.end());
		m_track_gizmo_drag.extra.clear();
	}
	records_watch_end();
	m_track_gizmo_using = false;
	m_track_drag.clear();
	m_track_drag_points.clear();
	m_track_joints.clear();
}

void editor_mode::track_gizmo_rotated(TTrack &Track)
{
	auto &drag{m_track_gizmo_drag};
	auto const turn{glm::transpose(drag.tilt.basis) * glm::dmat3(glm::mat3(m_track_gizmo))};
	std::array<double, 3> const angles{std::atan2(turn[1][2], turn[1][1]), std::atan2(turn[2][0], turn[0][0]), std::atan2(turn[0][1], turn[0][0])};
	if (drag.ring < 0)
	{
		auto const largest{std::ranges::max_element(angles, [](double const A, double const B) { return std::abs(A) < std::abs(B); })};
		if (std::abs(*largest) > 1e-5)
			drag.ring = static_cast<int>(largest - angles.begin());
	}
	if (drag.ring >= 0)
		track_gizmo_turn(Track, drag.ring, angles[drag.ring]);
}

void editor_mode::track_gizmo_scaled(TTrack &Track)
{
	auto const &drag{m_track_gizmo_drag};
	double along{0.0};
	if (false == mouse_along(drag.grab.origin, drag.grab.axis, along))
		return;
	auto length{drag.length + along - drag.grab.grabbed};
	if (Global.ctrlState && m_gizmo_snap > 0.0f)
		length = std::round(length / m_gizmo_snap) * m_gizmo_snap;
	track_gizmo_length(Track, length);
}

void editor_mode::track_gizmo_moved_point(TTrack &Track, glm::dvec3 const &Position)
{
	track_gizmo_point(Track, Position);
	m_track_dirty = true;
	if (false == editor_track::is_end(m_track_point.kind) || false == m_track_drag_points.empty())
		return;
	std::vector<TTrack const *> const exclude(m_track_drag.begin(), m_track_drag.end());
	screen_projection const projection;
	ImVec2 screen;
	auto const position{editor_track::point_position(Track, m_track_point)};
	m_track_snap = projection.project(position, screen) ? snap_free_end(position, {screen.x, screen.y}, &Track, Track.iCategoryFlag & 15, exclude) : editor_track::snap_target{};
}

void editor_mode::render_track_gizmo()
{
	using enum rail_state::gizmo_operation;
	auto *track = selected_track();
	if (track == nullptr || false == editable(*track))
	{
		m_track_gizmo_using = false;
		return;
	}
	bool const pointmode = ui()->mode() == nodebank_panel::TRACK && m_track_point.valid() && m_track_point.path < static_cast<int>(track->m_paths.size());
	bool const controlmode = pointmode && false == editor_track::is_end(m_track_point.kind);

	gizmo_frame const frame;
	auto &drag{m_track_gizmo_drag};
	auto const operation{m_track_gizmo_using ? drag.operation : track_gizmo_operation(*track, pointmode)};

	if (false == m_track_gizmo_using)
		m_track_gizmo = track_gizmo_frame(*track, pointmode, operation);

	glm::mat4 delta(1.0f);
	auto gizmo{operation == scale ? ImGuizmo::SCALE_Z : ImGuizmo::TRANSLATE};
	if (operation == rotate)
		gizmo = ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y | ImGuizmo::ROTATE_Z;
	ImGuizmo::AllowAxisFlip(operation != scale);
	if (m_point_drag.active)
		ImGuizmo::Enable(false);
	frame.manipulate(gizmo, m_track_gizmo, operation == translate ? m_gizmo_snap : 0.0f, &delta, operation == translate ? ImGuizmo::WORLD : ImGuizmo::LOCAL);
	ImGuizmo::Enable(true);
	ImGuizmo::AllowAxisFlip(true);

	if (false == ImGuizmo::IsUsing())
	{
		if (m_track_gizmo_using)
			track_gizmo_end(*track);
		return;
	}
	if (false == m_track_gizmo_using)
	{
		track_gizmo_begin(*track, pointmode, operation);
		if (operation == scale && false == mouse_along(drag.grab.origin, drag.grab.axis, drag.grab.grabbed))
			drag.grab.grabbed = 0.0;
	}
	if (operation == rotate)
		track_gizmo_rotated(*track);
	else if (operation == scale)
		track_gizmo_scaled(*track);
	else if (controlmode)
		track_gizmo_control(*track, frame.camera + glm::dvec3(m_track_gizmo[3]) - editor_track::point_position(*track, editor_track::end_of(m_track_point)));
	else if (pointmode)
		track_gizmo_moved_point(*track, frame.camera + glm::dvec3(m_track_gizmo[3]));
	else if (glm::dvec3 const offset{delta[3]}; glm::length(offset) > 1e-9)
	{
		editor_track::translate(*track, offset);
		editor_track::follow(*track, m_track_joints);
		m_track_dirty = true;
	}
	track_gizmo_switches(*track);
	commit_track_drag(false);
}
