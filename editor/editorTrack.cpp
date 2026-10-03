/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/

#include "stdafx.h"
#include "editor/editorTrack.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "world/Track.h"
#include "vehicle/DynObj.h"
#include "scene/scene.h"
#include "simulation/simulation.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "utilities/parser.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <deque>
#include <map>
#include <numeric>
#include <functional>
#include <sstream>
#include <tuple>
#include <unordered_set>

namespace
{

using geometry::bezier;
using geometry::chord_length;
using geometry::cross;
using geometry::plan_of;
using geometry::signed_angle;
using geometry::turned;

double const kSamePoint{geometry::same_point};

segment_data::point point_index(editor_track::point_kind const Kind)
{
	switch (Kind)
	{
	case editor_track::point_kind::start: return segment_data::point::start;
	case editor_track::point_kind::control1: return segment_data::point::control1;
	case editor_track::point_kind::control2: return segment_data::point::control2;
	default: return segment_data::point::end;
	}
}

bool has_switch_paths(TTrack const &Track)
{
	return Track.SwitchExtension != nullptr && (Track.eType == tt_Switch || Track.eType == tt_Cross);
}

void add_unique(std::vector<TTrack *> &Tracks, TTrack *Track)
{
	if (Track != nullptr && std::find(Tracks.begin(), Tracks.end(), Track) == Tracks.end())
		Tracks.emplace_back(Track);
}

glm::dvec3 path_tangent(segment_data const &Path, bool const Atend)
{
	auto const &start{Path.points[segment_data::point::start]};
	auto const &end{Path.points[segment_data::point::end]};
	auto const &control1{Path.points[segment_data::point::control1]};
	auto const &control2{Path.points[segment_data::point::control2]};
	if (false == Atend)
		return control1 != glm::dvec3{} ? control1 : (end + control2) - start;
	return control2 != glm::dvec3{} ? -control2 : end - (start + control1);
}

double path_radius(segment_data const &Path, double const T)
{
	if (Path.points[segment_data::point::control1] == glm::dvec3{} && Path.points[segment_data::point::control2] == glm::dvec3{})
		return 0.0;
	bezier const curve{Path};
	auto const first{curve.first(T)};
	auto const second{curve.second(T)};
	auto const speed{std::hypot(first.x, first.z)};
	auto const curvature{std::abs(first.x * second.z - first.z * second.x) / (speed * speed * speed)};
	return curvature > 1e-7 ? 1.0 / curvature : 0.0;
}

segment_data reversed(segment_data const &Path)
{
	segment_data result{Path};
	result.points[segment_data::point::start] = Path.points[segment_data::point::end];
	result.points[segment_data::point::end] = Path.points[segment_data::point::start];
	result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
	result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
	result.rolls = {-Path.rolls[1], -Path.rolls[0]};
	return result;
}

bool adjoining_direction(TTrack const *Neighbour, glm::dvec3 const &Joint, bool const Leaving, glm::dvec3 &Direction, double &Radius)
{
	if (Neighbour == nullptr)
		return false;
	for (auto const &path : Neighbour->m_paths)
	{
		for (auto const atend : {false, true})
		{
			auto const &point{path.points[atend ? segment_data::point::end : segment_data::point::start]};
			if (glm::distance(point, Joint) > kSamePoint)
				continue;
			auto const tangent{glm::normalize(path_tangent(path, atend))};
			Direction = (Leaving != atend) ? tangent : -tangent;
			Radius = path_radius(path, atend ? 1.0 : 0.0);
			return true;
		}
	}
	return false;
}

}

editor_track::observer *editor_track::s_observer{nullptr};

editor_track::state editor_track::capture(TTrack const &Track)
{
	if (s_observer != nullptr)
		s_observer->track_captured(Track);
	state result;
	result.paths = Track.m_paths;
	result.velocity = Track.fVelocity;
	result.switchvelocity = Track.SwitchExtension ? Track.SwitchExtension->fVelocity : -1.f;
	result.width = Track.fTrackWidth;
	result.friction = Track.fFriction;
	result.sounddistance = Track.fSoundDistance;
	result.quality = Track.iQualityFlag;
	result.damage = Track.iDamageFlag;
	result.environment = static_cast<int>(Track.eEnvironment);
	result.material1 = Track.m_material1;
	result.material2 = Track.m_material2;
	result.material3 = Track.SwitchExtension ? Track.SwitchExtension->m_material3 : null_handle;
	result.texlength = Track.fTexLength;
	result.texheight = texture_height(Track);
	result.texwidth = Track.fTexWidth;
	result.texslope = Track.fTexSlope;
	return result;
}

void editor_track::apply(TTrack &Track, state const &State)
{
	Track.m_paths = State.paths;
	Track.fVelocity = State.velocity;
	if (Track.SwitchExtension)
	{
		Track.SwitchExtension->fVelocity = State.switchvelocity;
		Track.SwitchExtension->m_material3 = State.material3;
	}
	Track.fTrackWidth = State.width;
	Track.fTrackWidth2 = State.width;
	Track.fFriction = State.friction;
	Track.fSoundDistance = State.sounddistance;
	Track.iQualityFlag = State.quality;
	damage(Track, State.damage);
	Track.eEnvironment = static_cast<TEnvironmentType>(State.environment);
	Track.m_material1 = State.material1;
	Track.m_material2 = State.material2;
	Track.fTexLength = State.texlength;
	texture_height(Track, State.texheight);
	Track.fTexWidth = State.texwidth;
	Track.fTexSlope = State.texslope;
}

bool editor_track::is_supported(TTrack const &Track)
{
	if ((Track.iCategoryFlag & 0x80) != 0)
		return false;
	if (Track.eType == tt_Normal)
		return Track.Segment != nullptr;
	if (Track.eType == tt_Switch)
		return Track.SwitchExtension != nullptr && Track.m_paths.size() >= 2;
	return false;
}

bool editor_track::can_edit_geometry(TTrack const &Track, std::string &Reason)
{
	if (false == is_supported(Track))
	{
		Reason = "Editing of this path type isn't supported";
		return false;
	}
	if (Global.NvRenderer)
	{
		Reason = "Path geometry can't be rebuilt with the experimental renderer";
		return false;
	}
	Reason.clear();
	return true;
}

void editor_track::commit(std::vector<TTrack *> const &Tracks)
{
	if (Tracks.empty())
		return;

	std::vector<TTrack *> affected;
	for (auto *track : Tracks)
	{
		add_unique(affected, track);
		for (auto *neighbour : neighbours(*track))
			add_unique(affected, neighbour);
	}
	for (auto *track : Tracks)
	{
		disconnect(*track);
		simulation::Region->erase_and_unregister(track);
	}
	for (auto *track : Tracks)
	{
		track->init_segments(false);
		track->update_location();
		track->m_area.radius = -1.f;
		simulation::Region->insert_and_register(track);
	}
	for (auto *track : Tracks)
		join(*track);
	for (auto *track : Tracks)
		for (auto *neighbour : neighbours(*track))
			add_unique(affected, neighbour);
	for (auto *track : affected)
		update_transition(*track);
	for (auto *track : affected)
		rebuild_geometry(*track);
	for (auto *track : Tracks)
		track->mark_dirty();
	for (auto *track : affected)
		for (auto *vehicle : track->Dynamics)
			vehicle->Move(0.000001);
	if (s_observer != nullptr)
		s_observer->tracks_committed(Tracks);
}

void editor_track::commit_parameters(TTrack &Track)
{
	Track.fTrackWidth2 = Track.fTrackWidth;
	Track.init_segments(false);

	std::vector<TTrack *> affected{&Track};
	for (auto *neighbour : neighbours(Track))
		add_unique(affected, neighbour);
	for (auto *track : affected)
		update_transition(*track);
	for (auto *track : affected)
		rebuild_geometry(*track);
	Track.mark_dirty();
}

glm::dvec3 editor_track::point_position(TTrack const &Track, point_ref const &Point)
{
	if (Point.path < 0 || Point.path >= static_cast<int>(Track.m_paths.size()))
		return Track.location();

	auto const &path{Track.m_paths[Point.path]};
	auto const &start{path.points[segment_data::point::start]};
	auto const &end{path.points[segment_data::point::end]};
	switch (Point.kind)
	{
	case point_kind::start: return start;
	case point_kind::end: return end;
	case point_kind::control1:
	{
		auto const &control{path.points[segment_data::point::control1]};
		return control == glm::dvec3{} ? start + (end - start) / 3.0 : start + control;
	}
	case point_kind::control2:
	{
		auto const &control{path.points[segment_data::point::control2]};
		return control == glm::dvec3{} ? end + (start - end) / 3.0 : end + control;
	}
	}
	return start;
}

void editor_track::move_point(TTrack &Track, point_ref const &Point, glm::dvec3 const &Position)
{
	if (Point.path < 0 || Point.path >= static_cast<int>(Track.m_paths.size()))
		return;

	auto &path{Track.m_paths[Point.path]};
	switch (Point.kind)
	{
	case point_kind::start:
	case point_kind::end:
	{
		auto &point{path.points[point_index(Point.kind)]};
		auto const oldposition{point};
		point = Position;
		for (auto &otherpath : Track.m_paths)
		{
			if (&otherpath == &path)
				continue;
			for (auto const index : {segment_data::point::start, segment_data::point::end})
				if (glm::distance(otherpath.points[index], oldposition) < kSamePoint)
					otherpath.points[index] = Position;
		}
		break;
	}
	case point_kind::control1:
	{
		auto control{Position - path.points[segment_data::point::start]};
		path.points[segment_data::point::control1] = (glm::length(control) < 0.01 ? glm::dvec3{} : control);
		break;
	}
	case point_kind::control2:
	{
		auto control{Position - path.points[segment_data::point::end]};
		path.points[segment_data::point::control2] = (glm::length(control) < 0.01 ? glm::dvec3{} : control);
		break;
	}
	}
}

void editor_track::translate(TTrack &Track, glm::dvec3 const &Offset)
{
	for (auto &path : Track.m_paths)
	{
		path.points[segment_data::point::start] += Offset;
		path.points[segment_data::point::end] += Offset;
	}
}

void editor_track::rotate_y(TTrack &Track, glm::dvec3 const &Pivot, double const Angle)
{
	auto const c{std::cos(Angle)};
	auto const s{std::sin(Angle)};
	auto const rotate = [&](glm::dvec3 const &Vector) { return glm::dvec3{Vector.x * c + Vector.z * s, Vector.y, -Vector.x * s + Vector.z * c}; };

	for (auto &path : Track.m_paths)
	{
		for (auto const index : {segment_data::point::start, segment_data::point::end})
			path.points[index] = Pivot + rotate(path.points[index] - Pivot);
		for (auto const index : {segment_data::point::control1, segment_data::point::control2})
			path.points[index] = rotate(path.points[index]);
	}
}

glm::dvec3 editor_track::pivot(TTrack const &Track)
{
	return Track.location();
}

std::vector<TTrack *> editor_track::neighbours(TTrack const &Track)
{
	std::vector<TTrack *> result;
	add_unique(result, Track.trPrev);
	add_unique(result, Track.trNext);
	if (has_switch_paths(Track))
	{
		for (int i = 0; i < 2; ++i)
		{
			add_unique(result, Track.SwitchExtension->pPrevs[i]);
			add_unique(result, Track.SwitchExtension->pNexts[i]);
		}
	}
	result.erase(std::remove(result.begin(), result.end(), &Track), result.end());
	return result;
}

std::vector<std::pair<TTrack *, editor_track::point_ref>> editor_track::connected_points(TTrack const &Track, glm::dvec3 const &Position)
{
	std::vector<std::pair<TTrack *, point_ref>> result;
	for (auto *neighbour : neighbours(Track))
	{
		for (int i = 0; i < static_cast<int>(neighbour->m_paths.size()); ++i)
		{
			for (auto const kind : {point_kind::start, point_kind::end})
			{
				if (glm::distance(neighbour->m_paths[i].points[point_index(kind)], Position) < kSamePoint)
					result.emplace_back(neighbour, point_ref{i, kind});
			}
		}
	}
	return result;
}

bool editor_track::is_connected(TTrack const &Track, point_ref const &Point)
{
	if (false == editor_track::is_end(Point.kind))
		return false;
	return false == connected_points(Track, point_position(Track, Point)).empty();
}

editor_track::snap_target editor_track::find_snap_target(TTrack const &Track, glm::dvec3 const &Position, double const Radius, std::vector<TTrack const *> const &Exclude)
{
	snap_target result;
	result.distance = std::numeric_limits<double>::max();

	auto const excluded = [&](TTrack const *Other) { return std::find(Exclude.begin(), Exclude.end(), Other) != Exclude.end(); };
	auto const category{Track.iCategoryFlag & 15};
	auto const sections{simulation::Region->sections(Position, static_cast<float>(Radius))};
	for (auto *section : sections)
	{
		for (auto const &cell : section->m_cells)
		{
			for (auto *other : cell.m_directories.paths)
			{
				if (other == &Track || excluded(other) || (other->iCategoryFlag & 15) != category || false == is_supported(*other))
					continue;
				for (int i = 0; i < static_cast<int>(other->m_paths.size()); ++i)
				{
					auto const &path{other->m_paths[i]};
					for (auto const kind : {point_kind::start, point_kind::end})
					{
						auto const &point{path.points[point_index(kind)]};
						auto const distance{glm::distance(point, Position)};
						if (distance > Radius || distance >= result.distance)
							continue;
						auto const connections{connected_points(*other, point)};
						if (std::any_of(connections.begin(), connections.end(), [&](auto const &Connection) { return false == excluded(Connection.first); }))
							continue;

						glm::dvec3 direction;
						if (kind == point_kind::start)
						{
							auto const &control{path.points[segment_data::point::control1]};
							direction = control != glm::dvec3{} ? control : path.points[segment_data::point::end] - point;
						}
						else
						{
							auto const &control{path.points[segment_data::point::control2]};
							direction = control != glm::dvec3{} ? control : path.points[segment_data::point::start] - point;
						}
						if (glm::length(direction) < 1e-6)
							continue;

						result.track = other;
						result.point = point_ref{i, kind};
						result.position = point;
						result.direction = glm::normalize(direction);
						result.distance = distance;
					}
				}
			}
		}
	}
	return result;
}

void editor_track::snap_point(TTrack &Track, point_ref const &Point, snap_target const &Target, bool const Aligntangent)
{
	if (Target.track == nullptr || false == editor_track::is_end(Point.kind))
		return;

	move_point(Track, Point, Target.position);
	if (false == Aligntangent)
		return;

	auto &path{Track.m_paths[Point.path]};
	auto const &start{path.points[segment_data::point::start]};
	auto const &end{path.points[segment_data::point::end]};
	auto &control{path.points[Point.kind == point_kind::start ? segment_data::point::control1 : segment_data::point::control2]};
	auto const length{control != glm::dvec3{} ? glm::length(control) : glm::distance(start, end) / 3.0};
	control = -Target.direction * length;
}

std::string editor_track::material_name(material_handle const Material)
{
	if (Material == null_handle)
		return "none";
	auto name{GfxRenderer->Material(Material)->GetName()};
	if (name.find(paths::textures) == 0)
		name.erase(0, std::string{paths::textures}.size());
	return name;
}

material_handle editor_track::fetch_material(std::string const &Name)
{
	if (Name.empty() || Name == "none")
		return null_handle;
	auto name{Name};
	replace_slashes(name);
	return GfxRenderer->Fetch_Material(name);
}

float editor_track::texture_height(TTrack const &Track)
{
	return (Track.fTexHeight1 - Track.fTexHeightOffset) * (Track.iCategoryFlag & 4 ? -1.f : 1.f);
}

void editor_track::texture_height(TTrack &Track, float const Height)
{
	Track.fTexHeight1 = Height * (Track.iCategoryFlag & 4 ? -1.f : 1.f) + Track.fTexHeightOffset;
}

double editor_track::velocity(TTrack const &Track)
{
	if (Track.SwitchExtension && Track.eType == tt_Switch)
		return Track.SwitchExtension->fVelocity;
	return Track.fVelocity;
}

void editor_track::velocity(TTrack &Track, double const Velocity)
{
	if (Track.SwitchExtension && Track.eType == tt_Switch)
	{
		Track.SwitchExtension->fVelocity = static_cast<float>(Velocity);
		if (Velocity <= -2.0)
			Track.fVelocity = Track.SwitchExtension->CurrentIndex ? -Velocity : -1.0;
		else
			Track.fVelocity = Velocity;
		return;
	}
	Track.fVelocity = Velocity;
}

void editor_track::damage(TTrack &Track, int const Damage)
{
	Track.iDamageFlag = Damage;
	if (Damage & 128)
		Track.iAction |= 0x80;
	else
		Track.iAction &= ~0x80;
}

void editor_track::disconnect(TTrack &Track)
{
	for (auto *neighbour : neighbours(Track))
	{
		if (has_switch_paths(*neighbour))
		{
			auto &extension{*neighbour->SwitchExtension};
			for (int i = 0; i < 2; ++i)
			{
				if (extension.pPrevs[i] == &Track)
					extension.pPrevs[i] = nullptr;
				if (extension.pNexts[i] == &Track)
					extension.pNexts[i] = nullptr;
			}
			bind_switch_path(*neighbour, extension.CurrentIndex);
		}
		else
		{
			if (neighbour->trPrev == &Track)
				neighbour->trPrev = nullptr;
			if (neighbour->trNext == &Track)
				neighbour->trNext = nullptr;
		}
	}
	Track.trPrev = nullptr;
	Track.trNext = nullptr;
	if (has_switch_paths(Track))
	{
		auto &extension{*Track.SwitchExtension};
		for (int i = 0; i < 2; ++i)
		{
			extension.pPrevs[i] = nullptr;
			extension.pNexts[i] = nullptr;
		}
	}
}

void editor_track::join(TTrack &Track)
{
	TTrack *other;
	int endpointid;
	switch (Track.eType)
	{
	case tt_Normal:
	{
		if (Track.trPrev == nullptr)
		{
			std::tie(other, endpointid) = simulation::Region->find_path(Track.Segment->FastGetPoint_0(), &Track);
			connect(Track, true, other, endpointid);
		}
		if (Track.trNext == nullptr)
		{
			std::tie(other, endpointid) = simulation::Region->find_path(Track.Segment->FastGetPoint_1(), &Track);
			connect(Track, false, other, endpointid);
		}
		break;
	}
	case tt_Switch:
	{
		for (int i = 0; i < 2; ++i)
		{
			auto const &segment{Track.SwitchExtension->Segments[i]};
			for (auto const &point : {segment->FastGetPoint_0(), segment->FastGetPoint_1()})
			{
				std::tie(other, endpointid) = simulation::Region->find_path(point, &Track);
				if (other != nullptr && other->eType == tt_Normal)
					join(*other);
			}
		}
		break;
	}
	default:
		break;
	}
}

void editor_track::connect(TTrack &Track, bool const Prevside, TTrack *Other, int const Endpointid)
{
	if (Other == nullptr)
		return;

	switch (Endpointid)
	{
	case 0:
		if (Prevside)
			Track.ConnectPrevPrev(Other, 0);
		else
			Track.ConnectNextPrev(Other, 0);
		break;
	case 1:
		if (Prevside)
			Track.ConnectPrevNext(Other, 1);
		else
			Track.ConnectNextNext(Other, 1);
		break;
	case 2:
	case 3:
	case 4:
	case 5:
	{
		if (false == has_switch_paths(*Other))
			break;
		auto const path{(Endpointid - 2) / 2};
		auto const atend{((Endpointid - 2) % 2) != 0};
		bind_switch_path(*Other, path);
		if (Prevside)
		{
			if (atend)
				Track.ConnectPrevNext(Other, path ? 3 : 1);
			else
				Track.ConnectPrevPrev(Other, path ? 2 : 0);
		}
		else
		{
			if (atend)
				Track.ConnectNextNext(Other, path ? 3 : 1);
			else
				Track.ConnectNextPrev(Other, path ? 2 : 0);
		}
		store_switch_path(*Other, path);
		bind_switch_path(*Other, Other->SwitchExtension->CurrentIndex);
		break;
	}
	default:
		break;
	}
}

void editor_track::update_transition(TTrack &Track)
{
	Track.iTrapezoid &= ~2;
	auto const *next{Track.trNext};
	if (next == nullptr || Track.iNextDirection != 0)
		return;
	if (Track.m_visible && next->m_visible && Track.eType == tt_Normal && next->eType == tt_Normal)
	{
		if (Track.fTrackWidth != next->fTrackWidth || Track.fTexHeight1 != next->fTexHeight1 || Track.fTexWidth != next->fTexWidth || Track.fTexSlope != next->fTexSlope)
			Track.iTrapezoid |= 2;
	}
}

void editor_track::bind_switch_path(TTrack &Switch, int const Path)
{
	auto &extension{*Switch.SwitchExtension};
	auto const path{Path & 1};
	Switch.Segment = extension.Segments[path];
	Switch.trNext = extension.pNexts[path];
	Switch.trPrev = extension.pPrevs[path];
	Switch.iNextDirection = extension.iNextDirection[path];
	Switch.iPrevDirection = extension.iPrevDirection[path];
}

void editor_track::store_switch_path(TTrack &Switch, int const Path)
{
	auto &extension{*Switch.SwitchExtension};
	auto const path{Path & 1};
	extension.pNexts[path] = Switch.trNext;
	extension.pPrevs[path] = Switch.trPrev;
	extension.iNextDirection[path] = Switch.iNextDirection;
	extension.iPrevDirection[path] = Switch.iPrevDirection;
	if (Switch.eType == tt_Switch)
	{
		extension.pPrevs[path ^ 1] = Switch.trPrev;
		extension.iPrevDirection[path ^ 1] = Switch.iPrevDirection;
	}
}

bool editor_track::find_chain(TTrack *From, TTrack *To, chain &Chain, std::string &Error)
{
	Chain = {};
	if (From == nullptr || To == nullptr)
	{
		Error = "Select the first and the last path of the fragment";
		return false;
	}
	auto const regular = [](TTrack const *Track) { return Track->eType == tt_Normal && is_supported(*Track); };
	if (false == regular(From) || false == regular(To))
	{
		Error = "The fragment has to consist of regular paths, without switches";
		return false;
	}

	Chain.tracks = {From};
	Chain.forward = {true};
	if (From != To)
	{
		bool found{false};
		for (auto const throughend : {true, false})
		{
			Chain.tracks = {From};
			Chain.forward = {throughend};
			TTrack *previous{From};
			TTrack *current{throughend ? From->trNext : From->trPrev};
			while (current != nullptr && current != From && Chain.tracks.size() < 10000 && regular(current))
			{
				auto const entersatstart{current->trPrev == previous};
				Chain.tracks.push_back(current);
				Chain.forward.push_back(entersatstart);
				if (current == To)
				{
					found = true;
					break;
				}
				previous = current;
				current = entersatstart ? current->trNext : current->trPrev;
			}
			if (found)
				break;
		}
		if (false == found)
		{
			Chain = {};
			Error = "The selected paths aren't connected by a chain of regular paths (switches end the chain)";
			return false;
		}
	}

	auto const category{From->iCategoryFlag & 15};
	for (auto const *track : Chain.tracks)
	{
		if ((track->iCategoryFlag & 15) != category)
		{
			Error = "The fragment mixes different kinds of paths";
			Chain = {};
			return false;
		}
		Chain.length += track->Length();
		Chain.velocity = std::max(Chain.velocity, velocity(*track));
		auto radius{path_radius(track->m_paths.front(), 0.5)};
		if (radius == 0.0 && track->m_paths.front().radius != 0.f)
			radius = std::abs(track->m_paths.front().radius);
		if (radius > 0.0 && (Chain.radius == 0.0 || radius < Chain.radius))
			Chain.radius = radius;
	}

	auto const &first{From->m_paths.front()};
	auto const firstforward{Chain.forward.front()};
	Chain.start = first.points[firstforward ? segment_data::point::start : segment_data::point::end];
	Chain.start_direction = firstforward ? path_tangent(first, false) : -path_tangent(first, true);
	adjoining_direction(firstforward ? From->trPrev : From->trNext, Chain.start, false, Chain.start_direction, Chain.start_radius);
	Chain.start_direction = glm::normalize(Chain.start_direction);

	auto const &last{To->m_paths.front()};
	auto const lastforward{Chain.forward.back()};
	Chain.end = last.points[lastforward ? segment_data::point::end : segment_data::point::start];
	Chain.end_direction = lastforward ? path_tangent(last, true) : -path_tangent(last, false);
	adjoining_direction(lastforward ? To->trNext : To->trPrev, Chain.end, true, Chain.end_direction, Chain.end_radius);
	Chain.end_direction = glm::normalize(Chain.end_direction);

	Error.clear();
	return true;
}

namespace
{

bool plan_straight(segment_data const &Path, editor_track::straight_tolerance const &Tolerance)
{
	auto const chord{plan_of(Path.points[segment_data::point::end] - Path.points[segment_data::point::start])};
	auto const length{glm::length(chord)};
	if (length < 1e-3)
		return false;
	auto const &control1{Path.points[segment_data::point::control1]};
	auto const &control2{Path.points[segment_data::point::control2]};
	if (Path.radius != 0.f && std::abs(Path.radius) < Tolerance.radius)
		return false;
	if (control1 == glm::dvec3{} && control2 == glm::dvec3{})
		return true;
	auto const direction{chord / length};
	for (auto const index : {segment_data::point::control1, segment_data::point::control2})
		if (std::abs(cross(direction, plan_of(Path.points[index]))) > Tolerance.offset)
			return false;
	auto const tangentstart{plan_of(path_tangent(Path, false))};
	auto const tangentend{plan_of(path_tangent(Path, true))};
	if (glm::length(tangentstart) < 1e-9 || glm::length(tangentend) < 1e-9)
		return false;
	auto const a{glm::normalize(tangentstart)};
	auto const b{glm::normalize(tangentend)};
	auto const turn{std::abs(std::atan2(cross(a, b), glm::dot(a, b)))};
	auto const deviation{std::max(std::abs(cross(direction, a)), std::abs(cross(direction, b)))};
	return turn * Tolerance.radius <= length && deviation * Tolerance.radius <= length;
}

glm::dvec2 direction_at(TTrack const &Track, glm::dvec3 const &Joint, bool const Arriving, bool &Found)
{
	Found = false;
	for (auto const &path : Track.m_paths)
	{
		for (auto const atend : {false, true})
		{
			if (glm::distance(path.points[atend ? segment_data::point::end : segment_data::point::start], Joint) > kSamePoint)
				continue;
			auto const tangent{plan_of(path_tangent(path, atend))};
			if (glm::length(tangent) < 1e-9)
				continue;
			Found = true;
			auto const unit{glm::normalize(tangent)};
			return (atend == Arriving) ? unit : -unit;
		}
	}
	return {0.0, 1.0};
}

TTrack *neighbour_at(TTrack const &Track, int const Path, bool const Atend)
{
	if (has_switch_paths(Track))
		return Atend ? Track.SwitchExtension->pNexts[Path & 1] : Track.SwitchExtension->pPrevs[Path & 1];
	return Atend ? Track.trNext : Track.trPrev;
}

void rigid_move(TTrack &Track, glm::dvec3 const &Pivot, glm::dvec3 const &Target, double const Angle)
{
	for (auto &path : Track.m_paths)
	{
		for (auto const index : {segment_data::point::start, segment_data::point::end})
		{
			auto &point{path.points[index]};
			auto const moved{plan_of(Target) + turned(plan_of(point - Pivot), Angle)};
			point = {moved.x, point.y + Target.y - Pivot.y, moved.y};
		}
		for (auto const index : {segment_data::point::control1, segment_data::point::control2})
		{
			auto &control{path.points[index]};
			auto const moved{turned(plan_of(control), Angle)};
			control = {moved.x, control.y, moved.y};
		}
	}
}

void follow_joint(TTrack &Track, glm::dvec3 const &From, glm::dvec3 const &To, double const Angle)
{
	for (auto &path : Track.m_paths)
	{
		for (auto const atend : {false, true})
		{
			auto &point{path.points[atend ? segment_data::point::end : segment_data::point::start]};
			if (glm::distance(point, From) > kSamePoint)
				continue;
			point = To;
			auto &control{path.points[atend ? segment_data::point::control2 : segment_data::point::control1]};
			auto const moved{turned(plan_of(control), Angle)};
			control = {moved.x, control.y, moved.y};
		}
	}
}

} // namespace

bool editor_track::is_straight(TTrack const &Track, straight_tolerance const &Tolerance)
{
	if (false == is_supported(Track) || Track.m_editorremoved)
		return false;
	if (false == std::any_of(Track.m_paths.begin(), Track.m_paths.end(), [&](segment_data const &Path) { return plan_straight(Path, Tolerance); }))
		return false;
	if (Track.eType != tt_Normal || Track.trPrev == nullptr || Track.trNext == nullptr)
		return true;
	auto const &path{Track.m_paths.front()};
	auto const &start{path.points[segment_data::point::start]};
	auto const &end{path.points[segment_data::point::end]};
	auto const length{glm::length(plan_of(end - start))};
	auto const direction{glm::normalize(plan_of(end - start))};
	bool foundbefore, foundafter;
	auto const before{direction_at(*Track.trPrev, start, true, foundbefore)};
	auto const after{direction_at(*Track.trNext, end, false, foundafter)};
	if (false == foundbefore || false == foundafter)
		return true;
	auto const kinkbefore{signed_angle(before, direction)};
	auto const kinkafter{signed_angle(direction, after)};
	if (std::abs(kinkbefore) <= Tolerance.angle || std::abs(kinkafter) <= Tolerance.angle || (kinkbefore > 0.0) != (kinkafter > 0.0))
		return true;
	return length / ((std::abs(kinkbefore) + std::abs(kinkafter)) * 0.5) >= Tolerance.radius;
}

editor_track::straight editor_track::find_straight(TTrack &Track, straight_tolerance const &Tolerance)
{
	straight result;
	if (false == is_straight(Track, Tolerance))
		return result;

	int firstpath{0};
	while (false == plan_straight(Track.m_paths[firstpath], Tolerance))
		++firstpath;
	auto const &first{Track.m_paths[firstpath]};
	auto const origin{first.points[segment_data::point::start]};
	auto const direction{glm::normalize(plan_of(first.points[segment_data::point::end] - origin))};
	auto const lateral = [&](glm::dvec3 const &Point) { return std::abs(cross(direction, plan_of(Point - origin))); };
	auto const accepts = [&](TTrack const *Other, glm::dvec3 const &Joint) {
		if (Other == nullptr || false == is_supported(*Other) || Other->m_editorremoved)
			return -1;
		for (int i = 0; i < static_cast<int>(Other->m_paths.size()); ++i)
		{
			auto const &path{Other->m_paths[i]};
			if (false == plan_straight(path, Tolerance))
				continue;
			auto const &start{path.points[segment_data::point::start]};
			auto const &end{path.points[segment_data::point::end]};
			if (glm::distance(start, Joint) > kSamePoint && glm::distance(end, Joint) > kSamePoint)
				continue;
			auto const otherdirection{glm::normalize(plan_of(end - start))};
			if (std::abs(cross(direction, otherdirection)) > Tolerance.angle)
				continue;
			if (lateral(start) > Tolerance.offset || lateral(end) > Tolerance.offset)
				continue;
			return i;
		}
		return -1;
	};

	std::deque<std::pair<TTrack *, int>> run{{&Track, firstpath}};
	for (auto const forward : {true, false})
	{
		TTrack *current{&Track};
		int path{firstpath};
		bool atend{forward};
		while (true)
		{
			auto const &exit{current->m_paths[path].points[atend ? segment_data::point::end : segment_data::point::start]};
			auto *next{neighbour_at(*current, path, atend)};
			if (std::any_of(run.begin(), run.end(), [&](auto const &Member) { return Member.first == next; }))
				break;
			auto const nextpath{accepts(next, exit)};
			if (nextpath < 0)
				break;
			if (forward)
				run.emplace_back(next, nextpath);
			else
				run.emplace_front(next, nextpath);
			atend = glm::distance(next->m_paths[nextpath].points[segment_data::point::start], exit) <= kSamePoint;
			current = next;
			path = nextpath;
		}
	}

	double low{0.0}, high{0.0};
	glm::dvec3 lowpoint{origin}, highpoint{origin};
	for (auto const &member : run)
	{
		result.tracks.push_back(member.first);
		result.paths.push_back(member.second);
		for (auto const index : {segment_data::point::start, segment_data::point::end})
		{
			auto const &point{member.first->m_paths[member.second].points[index]};
			auto const along{glm::dot(plan_of(point - origin), direction)};
			if (along < low)
			{
				low = along;
				lowpoint = point;
			}
			if (along > high)
			{
				high = along;
				highpoint = point;
			}
		}
	}
	result.start = lowpoint;
	result.end = highpoint;
	result.direction = direction;
	result.length = high - low;
	result.grade = result.length > 0.0 ? (highpoint.y - lowpoint.y) / result.length : 0.0;
	auto azimuth{glm::degrees(std::atan2(direction.x, direction.y))};
	if (azimuth < 0.0)
		azimuth += 360.0;
	result.azimuth = azimuth;
	return result;
}

std::vector<editor_track::straight> editor_track::find_straights(double const Minimumlength, straight_tolerance const &Tolerance)
{
	std::vector<straight> result;
	std::unordered_set<TTrack const *> visited;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || visited.count(track) > 0 || false == is_straight(*track, Tolerance))
			continue;
		auto line{find_straight(*track, Tolerance)};
		for (auto const *member : line.tracks)
			visited.insert(member);
		if (line.length >= Minimumlength)
			result.push_back(std::move(line));
	}
	std::sort(result.begin(), result.end(), [](straight const &A, straight const &B) { return A.length > B.length; });
	return result;
}

bool editor_track::find_curve(TTrack &Track, straight_tolerance const &Tolerance, double const Gauge, curve &Curve)
{
	Curve = {};
	auto const curved = [&](TTrack const *Other) { return Other != nullptr && Other->eType == tt_Normal && is_supported(*Other) && false == Other->m_editorremoved && false == is_straight(*Other, Tolerance); };
	if (false == curved(&Track))
		return false;

	std::deque<std::pair<TTrack *, bool>> run{{&Track, true}};
	for (auto const throughend : {true, false})
	{
		TTrack *previous{&Track};
		TTrack *current{throughend ? Track.trNext : Track.trPrev};
		while (curved(current) && std::none_of(run.begin(), run.end(), [&](auto const &Member) { return Member.first == current; }))
		{
			auto const entersatstart{current->trPrev == previous};
			if (throughend)
				run.emplace_back(current, entersatstart);
			else
				run.emplace_front(current, false == entersatstart);
			previous = current;
			current = entersatstart ? current->trNext : current->trPrev;
		}
	}

	std::vector<double> turns;
	std::vector<double> lengths;
	std::vector<bool> varying;
	std::vector<double> curvatures;
	auto const incoming = [&](std::size_t const Index) {
		auto const &path{run[Index].first->m_paths.front()};
		auto const tangent{glm::normalize(plan_of(run[Index].second ? path_tangent(path, false) : -path_tangent(path, true)))};
		return tangent;
	};
	auto const outgoing = [&](std::size_t const Index) {
		auto const &path{run[Index].first->m_paths.front()};
		auto const tangent{glm::normalize(plan_of(run[Index].second ? path_tangent(path, true) : -path_tangent(path, false)))};
		return tangent;
	};
	auto const joint = [&](std::size_t const Index, bool const Atend) {
		auto const &path{run[Index].first->m_paths.front()};
		return path.points[(run[Index].second == Atend) ? segment_data::point::end : segment_data::point::start];
	};
	std::vector<double> kinks(run.size() + 1, 0.0);
	{
		auto *before{run.front().second ? run.front().first->trPrev : run.front().first->trNext};
		bool found{false};
		if (before != nullptr)
		{
			auto const direction{direction_at(*before, joint(0, false), true, found)};
			if (found)
				kinks.front() = signed_angle(direction, incoming(0));
		}
		auto *after{run.back().second ? run.back().first->trNext : run.back().first->trPrev};
		if (after != nullptr)
		{
			auto const direction{direction_at(*after, joint(run.size() - 1, true), false, found)};
			if (found)
				kinks.back() = signed_angle(outgoing(run.size() - 1), direction);
		}
		for (std::size_t i = 1; i < run.size(); ++i)
			kinks[i] = signed_angle(outgoing(i - 1), incoming(i));
	}
	for (std::size_t i = 0; i < run.size(); ++i)
	{
		auto const &path{run[i].first->m_paths.front()};
		auto const length{run[i].first->Length()};
		auto const internal{signed_angle(incoming(i), outgoing(i))};
		turns.push_back(internal + kinks[i] * 0.5 + kinks[i + 1] * 0.5);
		lengths.push_back(length);
		auto const radiusstart{path_radius(path, 0.0)};
		auto const radiusend{path_radius(path, 1.0)};
		auto const curvaturestart{radiusstart > 0.0 ? 1.0 / radiusstart : 0.0};
		auto const curvatureend{radiusend > 0.0 ? 1.0 / radiusend : 0.0};
		auto const middle{path_radius(path, 0.5)};
		double curvature{middle > 0.0 ? 1.0 / middle : 0.0};
		if (curvature == 0.0 && path.radius != 0.f)
			curvature = 1.0 / std::abs(path.radius);
		if (curvature == 0.0 && length > 1e-3)
			curvature = std::abs(turns.back()) / length;
		curvatures.push_back(curvature);
		varying.push_back(std::abs(curvaturestart - curvatureend) > 0.1 * std::max(curvaturestart, curvatureend));
		for (auto const roll : path.rolls)
			Curve.cant = std::max(Curve.cant, Gauge * std::sin(std::abs(glm::radians(static_cast<double>(roll)))));
	}
	auto const peak{*std::max_element(curvatures.begin(), curvatures.end())};
	for (std::size_t i = 0; i < run.size(); ++i)
		if (curvatures[i] < 0.9 * peak)
			varying[i] = true;
	double smallest{0.0};
	for (std::size_t i = 0; i < run.size(); ++i)
		if (false == varying[i] && curvatures[i] > 0.0 && (smallest == 0.0 || 1.0 / curvatures[i] < smallest))
			smallest = 1.0 / curvatures[i];
	if (smallest == 0.0 && peak > 0.0)
		smallest = 1.0 / peak;
	int sign{0};
	for (auto const turn : turns)
	{
		Curve.turn += turn;
		int const current{turn > 1e-9 ? 1 : turn < -1e-9 ? -1 : 0};
		if (current != 0 && sign != 0 && current != sign)
			++Curve.reversals;
		if (current != 0)
			sign = current;
	}
	for (std::size_t i = 0; i < run.size() && varying[i]; ++i)
		Curve.transition_in += lengths[i];
	for (std::size_t i = run.size(); i > 0 && varying[i - 1]; --i)
		Curve.transition_out += lengths[i - 1];
	if (Curve.transition_in + Curve.transition_out >= std::accumulate(lengths.begin(), lengths.end(), 0.0))
		Curve.transition_in = Curve.transition_out = 0.0;
	Curve.radius = smallest;
	std::vector<std::pair<double, double>> arcs;
	for (std::size_t i = 0; i < run.size(); ++i)
	{
		if (varying[i] || curvatures[i] <= 0.0)
			continue;
		auto const radius{1.0 / curvatures[i]};
		if (false == arcs.empty() && std::abs(arcs.back().first - radius) < 0.05 * radius)
			arcs.back().second += lengths[i];
		else
			arcs.emplace_back(radius, lengths[i]);
	}
	if (arcs.size() >= 2 && Curve.reversals == 0)
	{
		auto const &first{arcs.front()};
		auto const &last{arcs.back()};
		if (std::abs(first.first - last.first) > 0.1 * std::min(first.first, last.first))
		{
			Curve.compound = true;
			Curve.radius = first.first;
			Curve.radius2 = last.first;
			auto const turnfirst{first.second / first.first};
			auto const turnlast{last.second / last.first};
			Curve.split = turnfirst / std::max(1e-9, turnfirst + turnlast);
			double middle{0.0};
			bool inside{false};
			for (std::size_t i = 0; i < run.size(); ++i)
			{
				if (false == varying[i])
				{
					inside = true;
					continue;
				}
				if (inside && i + 1 < run.size() && std::any_of(varying.begin() + i + 1, varying.end(), [](bool const Varying) { return false == Varying; }))
					middle += lengths[i];
			}
			Curve.transition_middle = middle;
		}
	}
	Curve.from = run.front().first;
	Curve.to = run.back().first;
	return true;
}

namespace
{

// shorter remainders of a straight path are absorbed by moving the joint instead of splitting the path
double split_margin(double const Length)
{
	return std::max(0.5, 0.002 * Length);
}

} // namespace

editor_track::straight_run editor_track::straight_beyond(chain const &Chain, bool const Atend, straight_tolerance const &Tolerance)
{
	straight_run run;
	if (Chain.tracks.empty())
		return run;
	auto *edge{Atend ? Chain.tracks.back() : Chain.tracks.front()};
	bool const forward{Atend ? Chain.forward.back() : Chain.forward.front()};
	TTrack *current{forward == Atend ? edge->trNext : edge->trPrev};
	auto joint{Atend ? Chain.end : Chain.start};
	auto const origin{plan_of(joint)};
	auto direction{glm::normalize(plan_of(Atend ? Chain.end_direction : -Chain.start_direction))};
	while (current != nullptr && run.tracks.size() < 1000)
	{
		std::string reason;
		if (current->eType != tt_Normal || current->m_editorremoved || false == can_edit_geometry(*current, reason))
			break;
		if (std::find(Chain.tracks.begin(), Chain.tracks.end(), current) != Chain.tracks.end() || std::find(run.tracks.begin(), run.tracks.end(), current) != run.tracks.end())
			break;
		auto const &path{current->m_paths.front()};
		if (false == plan_straight(path, Tolerance))
			break;
		bool outward;
		if (glm::distance(path.points[segment_data::point::start], joint) <= kSamePoint)
			outward = true;
		else if (glm::distance(path.points[segment_data::point::end], joint) <= kSamePoint)
			outward = false;
		else
			break;
		auto const &far{path.points[outward ? segment_data::point::end : segment_data::point::start]};
		if (run.tracks.empty())
		{
			auto const own{glm::normalize(plan_of(far - joint))};
			if (std::abs(cross(direction, own)) > 0.01 || glm::dot(direction, own) <= 0.0)
				break;
			direction = own;
		}
		auto const offset{plan_of(far) - origin};
		auto const along{glm::dot(offset, direction)};
		if (along <= run.length + 1e-3 || std::abs(cross(direction, offset)) > Tolerance.offset)
			break;
		run.tracks.push_back(current);
		run.outward.push_back(outward);
		run.length = along;
		joint = far;
		current = outward ? current->trNext : current->trPrev;
	}
	return run;
}

double editor_track::run_reserve(straight_run const &Run)
{
	if (Run.tracks.empty())
		return 0.0;
	return std::max(0.0, Run.length - split_margin(chord_length(Run.tracks.back()->m_paths.front())) - 0.01);
}

TTrack *editor_track::take_straight(straight_run const &Run, TTrack *Edge, glm::dvec3 const &Joint, glm::dvec3 const &Cut, std::vector<std::pair<TTrack *, state>> &States, std::vector<TTrack *> &Created)
{
	auto const length{glm::length(plan_of(Cut - Joint))};
	auto *edge{Edge};
	double covered{0.0};
	for (std::size_t i = 0; i < Run.tracks.size(); ++i)
	{
		auto *track{Run.tracks[i]};
		auto const outward{Run.outward[i]};
		auto const &path{track->m_paths.front()};
		auto const pathlength{chord_length(path)};
		auto const margin{split_margin(pathlength)};
		auto const remaining{length - covered};
		if (remaining <= margin)
		{
			if (std::abs(remaining) > 1e-4)
			{
				States.emplace_back(track, capture(*track));
				move_point(*track, {0, outward ? point_kind::start : point_kind::end}, Cut);
				commit({track});
			}
			break;
		}
		if (remaining < pathlength - margin)
		{
			auto const &nearpoint{path.points[outward ? segment_data::point::start : segment_data::point::end]};
			auto const &farpoint{path.points[outward ? segment_data::point::end : segment_data::point::start]};
			auto const t{nearest_parameter(*track, nearpoint + (farpoint - nearpoint) * (remaining / pathlength))};
			auto const before{capture(*track)};
			auto *created{split_path(*track, t)};
			if (created == nullptr)
				break;
			States.emplace_back(track, before);
			Created.push_back(created);
			edge = outward ? track : created;
			break;
		}
		edge = track;
		covered += pathlength;
	}
	return edge;
}

namespace
{

using route_node = std::tuple<TTrack *, int, bool>;

std::vector<route_node> route_next(route_node const &Node)
{
	std::vector<route_node> result;
	auto *track{std::get<0>(Node)};
	auto const path{std::get<1>(Node)};
	auto const forward{std::get<2>(Node)};
	auto const &exit{track->m_paths[path].points[forward ? segment_data::point::end : segment_data::point::start]};
	auto *next{neighbour_at(*track, path, forward)};
	if (next == nullptr || next == track || next->m_editorremoved || false == editor_track::is_supported(*next) || (next->iCategoryFlag & 15) != (track->iCategoryFlag & 15))
		return result;
	for (int i = 0; i < static_cast<int>(next->m_paths.size()); ++i)
	{
		auto const &candidate{next->m_paths[i]};
		if (glm::distance(candidate.points[segment_data::point::start], exit) <= kSamePoint)
			result.emplace_back(next, i, true);
		else if (glm::distance(candidate.points[segment_data::point::end], exit) <= kSamePoint)
			result.emplace_back(next, i, false);
	}
	return result;
}

} // namespace

bool editor_track::find_route(TTrack *From, TTrack *To, route &Route, std::string &Error)
{
	Route = {};
	if (From == nullptr || To == nullptr || false == is_supported(*From) || false == is_supported(*To))
	{
		Error = "Select the first and the last path of the route";
		return false;
	}
	if (From == To)
	{
		Route.spans.push_back({From, 0, true});
		Error.clear();
		return true;
	}
	std::vector<route_node> nodes;
	std::vector<int> parents;
	std::map<route_node, int> visited;
	for (int i = 0; i < static_cast<int>(From->m_paths.size()); ++i)
		for (auto const forward : {true, false})
		{
			route_node const node{From, i, forward};
			visited.emplace(node, static_cast<int>(nodes.size()));
			nodes.push_back(node);
			parents.push_back(-1);
		}
	int goal{-1};
	for (std::size_t index = 0; index < nodes.size() && goal < 0 && nodes.size() < 200000; ++index)
	{
		for (auto const &next : route_next(nodes[index]))
		{
			if (std::get<0>(next) == From || visited.count(next) > 0)
				continue;
			visited.emplace(next, static_cast<int>(nodes.size()));
			nodes.push_back(next);
			parents.push_back(static_cast<int>(index));
			if (std::get<0>(next) == To)
			{
				goal = static_cast<int>(nodes.size()) - 1;
				break;
			}
		}
	}
	if (goal < 0)
	{
		Error = "No route connects the selected paths";
		return false;
	}
	for (int index = goal; index >= 0; index = parents[index])
		Route.spans.push_back({std::get<0>(nodes[index]), std::get<1>(nodes[index]), std::get<2>(nodes[index])});
	std::reverse(Route.spans.begin(), Route.spans.end());
	Error.clear();
	return true;
}

editor_track::route editor_track::run_route(TTrack &Track, double const Maximum)
{
	route result;
	if (false == is_supported(Track))
		return result;
	auto const walk = [&](route_node Node) {
		std::vector<route_node> nodes;
		double length{0.0};
		while (nodes.size() < 5000 && length < Maximum)
		{
			auto candidates{route_next(Node)};
			if (candidates.empty())
				break;
			auto chosen{candidates.front()};
			for (auto const &candidate : candidates)
				if (std::get<1>(candidate) == 0)
					chosen = candidate;
			auto *track{std::get<0>(chosen)};
			if (track == &Track || std::any_of(nodes.begin(), nodes.end(), [&](route_node const &Other) { return std::get<0>(Other) == track; }))
				break;
			nodes.push_back(chosen);
			length += bezier(track->m_paths[std::get<1>(chosen)]).plan_length(8);
			Node = chosen;
		}
		return nodes;
	};
	auto const ahead{walk({&Track, 0, true})};
	auto const behind{walk({&Track, 0, false})};
	for (auto node = behind.rbegin(); node != behind.rend(); ++node)
		if (std::none_of(ahead.begin(), ahead.end(), [&](route_node const &Other) { return std::get<0>(Other) == std::get<0>(*node); }))
			result.spans.push_back({std::get<0>(*node), std::get<1>(*node), false == std::get<2>(*node)});
	result.spans.push_back({&Track, 0, true});
	for (auto const &node : ahead)
		result.spans.push_back({std::get<0>(node), std::get<1>(node), std::get<2>(node)});
	return result;
}

editor_track::route editor_track::route_of(chain const &Chain)
{
	route result;
	for (std::size_t i = 0; i < Chain.tracks.size(); ++i)
		result.spans.push_back({Chain.tracks[i], 0, Chain.forward[i]});
	return result;
}

std::vector<editor_track::route_sample> editor_track::sample_route(route &Route, double const Step)
{
	std::vector<route_sample> samples;
	double chainage{0.0};
	for (std::size_t i = 0; i < Route.spans.size(); ++i)
	{
		auto &span{Route.spans[i]};
		bezier const curve{span.track->m_paths[span.path]};
		auto const count{std::max(2, static_cast<int>(std::ceil(curve.plan_length(16) / std::max(0.1, Step))))};
		span.from = chainage;
		auto previous{plan_of(curve.point(span.forward ? 0.0 : 1.0))};
		for (int j = (samples.empty() ? 0 : 1); j <= count; ++j)
		{
			auto const t{span.forward ? static_cast<double>(j) / count : 1.0 - static_cast<double>(j) / count};
			auto const position{curve.point(t)};
			chainage += glm::distance(previous, plan_of(position));
			previous = plan_of(position);
			auto derivative{curve.first(t)};
			if (false == span.forward)
				derivative = -derivative;
			auto const second{curve.second(t)};
			auto const planar{std::hypot(derivative.x, derivative.z)};
			route_sample sample;
			sample.chainage = chainage;
			sample.position = position;
			sample.span = i;
			if (planar > 1e-9)
			{
				sample.direction = plan_of(derivative) / planar;
				sample.grade = derivative.y / planar;
				sample.curvature = (derivative.x * second.z - derivative.z * second.x) / (planar * planar * planar);
			}
			samples.push_back(sample);
		}
		span.to = chainage;
	}
	Route.length = chainage;
	return samples;
}

namespace
{

template <typename Getter>
auto interpolate(std::vector<editor_track::route_sample> const &Samples, double const Chainage, Getter const &Get) -> decltype(Get(Samples.front()))
{
	if (Samples.empty())
		return {};
	auto const next{std::lower_bound(Samples.begin(), Samples.end(), Chainage, [](editor_track::route_sample const &Sample, double const Value) { return Sample.chainage < Value; })};
	if (next == Samples.begin())
		return Get(Samples.front());
	if (next == Samples.end())
		return Get(Samples.back());
	auto const &before{*std::prev(next)};
	auto const run{next->chainage - before.chainage};
	return run > 1e-9 ? Get(before) + (Get(*next) - Get(before)) * (Chainage - before.chainage) / run : Get(before);
}

void set_heights(segment_data &Path, double const Startheight, double const Endheight, double const Startgrade, double const Endgrade)
{
	auto &start{Path.points[segment_data::point::start]};
	auto &end{Path.points[segment_data::point::end]};
	auto &control1{Path.points[segment_data::point::control1]};
	auto &control2{Path.points[segment_data::point::control2]};
	if (control1 == glm::dvec3{} && control2 == glm::dvec3{})
	{
		control1 = (end - start) / 3.0;
		control2 = (start - end) / 3.0;
	}
	start.y = Startheight;
	end.y = Endheight;
	control1.y = Startgrade * std::hypot(control1.x, control1.z);
	control2.y = -Endgrade * std::hypot(control2.x, control2.z);
}

// the path lies in the plane through Origin which rises with Grade along Direction (unit, in plan) and is level across it
void plane_heights(segment_data &Path, glm::dvec3 const &Origin, glm::dvec2 const &Direction, double const Grade)
{
	auto &start{Path.points[segment_data::point::start]};
	auto &end{Path.points[segment_data::point::end]};
	auto &control1{Path.points[segment_data::point::control1]};
	auto &control2{Path.points[segment_data::point::control2]};
	if (control1 == glm::dvec3{} && control2 == glm::dvec3{})
	{
		control1 = (end - start) / 3.0;
		control2 = (start - end) / 3.0;
	}
	auto const height = [&](glm::dvec3 const &Point) { return Origin.y + Grade * glm::dot(plan_of(Point - Origin), Direction); };
	auto const first{start + control1};
	auto const second{end + control2};
	start.y = height(start);
	end.y = height(end);
	control1.y = height(first) - start.y;
	control2.y = height(second) - end.y;
}

glm::dvec2 start_direction(segment_data const &Path)
{
	auto const &start{Path.points[segment_data::point::start]};
	auto const &control{Path.points[segment_data::point::control1]};
	glm::dvec2 direction{control.x, control.z};
	if (glm::length(direction) < 1e-6)
		direction = {Path.points[segment_data::point::end].x - start.x, Path.points[segment_data::point::end].z - start.z};
	return glm::length(direction) > 1e-9 ? glm::normalize(direction) : glm::dvec2{1.0, 0.0};
}

} // namespace

double editor_track::sampled_elevation(std::vector<route_sample> const &Samples, double const Chainage)
{
	return interpolate(Samples, Chainage, [](route_sample const &Sample) { return Sample.position.y; });
}

double editor_track::sampled_grade(std::vector<route_sample> const &Samples, double const Chainage)
{
	return interpolate(Samples, Chainage, [](route_sample const &Sample) { return Sample.grade; });
}

glm::dvec3 editor_track::sampled_position(std::vector<route_sample> const &Samples, double const Chainage)
{
	return interpolate(Samples, Chainage, [](route_sample const &Sample) { return Sample.position; });
}

bool editor_track::adjoining_grade(route const &Route, bool const Atend, double &Grade)
{
	if (Route.spans.empty())
		return false;
	auto const &span{Atend ? Route.spans.back() : Route.spans.front()};
	auto const outward{Atend ? span.forward : false == span.forward};
	auto const next{route_next({span.track, span.path, outward})};
	if (next.empty())
		return false;
	auto const &[track, path, forward]{next.front()};
	auto const tangent{path_tangent(track->m_paths[path], false == forward)};
	auto const plan{std::hypot(tangent.x, tangent.z)};
	if (plan < 1e-9)
		return false;
	auto const along{(forward ? 1.0 : -1.0) * tangent.y / plan};
	Grade = Atend ? along : -along;
	return true;
}

std::vector<editor_track::height_gap> editor_track::apply_profile(route &Route, std::function<double(double)> const &Elevation, std::function<double(double)> const &Grade, std::vector<double> const &Breaks, std::vector<std::pair<TTrack *, state>> &States, std::vector<TTrack *> &Created)
{
	double constexpr margin{0.25};
	auto const remember = [&](TTrack *Track) {
		if (std::none_of(States.begin(), States.end(), [&](auto const &Entry) { return Entry.first == Track; }) && std::find(Created.begin(), Created.end(), Track) == Created.end())
			States.emplace_back(Track, capture(*Track));
	};
	sample_route(Route, 1.0);
	std::vector<route_span> spans;
	for (auto const &span : Route.spans)
	{
		if (span.track->eType != tt_Normal)
		{
			spans.push_back(span);
			continue;
		}
		std::vector<double> cuts;
		for (auto const chainage : Breaks)
			if (chainage > span.from + margin && chainage < span.to - margin)
				cuts.push_back(chainage);
		if (cuts.empty())
		{
			spans.push_back(span);
			continue;
		}
		remember(span.track);
		if (span.forward)
		{
			auto *piece{span.track};
			auto origin{span.from};
			spans.push_back({piece, 0, true});
			for (auto const cut : cuts)
			{
				auto const t{bezier(piece->m_paths.front()).parameter(cut - origin)};
				auto *created{split_path(*piece, t)};
				if (created == nullptr)
					continue;
				Created.push_back(created);
				spans.push_back({created, 0, true});
				piece = created;
				origin = cut;
			}
		}
		else
		{
			std::vector<TTrack *> order;
			for (auto cut = cuts.begin(); cut != cuts.end(); ++cut)
			{
				auto const t{bezier(span.track->m_paths.front()).parameter(span.to - *cut)};
				auto *created{split_path(*span.track, t)};
				if (created == nullptr)
					continue;
				Created.push_back(created);
				order.push_back(created);
			}
			for (auto *piece : order)
				spans.push_back({piece, 0, false});
			spans.push_back({span.track, 0, false});
		}
	}
	Route.spans = spans;
	sample_route(Route, 1.0);

	auto const on_route = [&](TTrack const *Track) {
		return std::any_of(Route.spans.begin(), Route.spans.end(), [&](route_span const &Span) { return Span.track == Track; });
	};
	struct joint
	{
		TTrack *track;
		int path;
		bool atend;
		TTrack *neighbour;
	};
	std::vector<joint> joints;
	std::vector<TTrack *> changed;
	for (auto const &span : Route.spans)
	{
		remember(span.track);
		auto const startchainage{span.forward ? span.from : span.to};
		auto const endchainage{span.forward ? span.to : span.from};
		auto const sign{span.forward ? 1.0 : -1.0};
		auto &path{span.track->m_paths[span.path]};
		set_heights(path, Elevation(startchainage), Elevation(endchainage), Grade(startchainage) * sign, Grade(endchainage) * sign);
		add_unique(changed, span.track);
		if (span.track->eType == tt_Normal)
			continue;
		auto const grade{Grade((span.from + span.to) * 0.5) * sign};
		for (int i = 0; i < static_cast<int>(span.track->m_paths.size()); ++i)
		{
			for (auto const atend : {false, true})
			{
				auto *neighbour{neighbour_at(*span.track, i, atend)};
				if (neighbour != nullptr && false == on_route(neighbour) && std::none_of(joints.begin(), joints.end(), [&](joint const &Joint) { return Joint.track == span.track && Joint.neighbour == neighbour; }))
					joints.push_back({span.track, i, atend, neighbour});
			}
			if (i != span.path)
				plane_heights(span.track->m_paths[i], path.points[segment_data::point::start], start_direction(path), grade);
		}
	}
	commit(changed);
	sample_route(Route, 1.0);

	std::vector<height_gap> gaps;
	for (auto const &joint : joints)
	{
		auto const &end{joint.track->m_paths[joint.path].points[joint.atend ? segment_data::point::end : segment_data::point::start]};
		double nearest{std::numeric_limits<double>::max()};
		double gap{0.0};
		for (auto const &path : joint.neighbour->m_paths)
			for (auto const index : {segment_data::point::start, segment_data::point::end})
			{
				auto const &point{path.points[index]};
				auto const distance{std::hypot(point.x - end.x, point.z - end.z)};
				if (distance < nearest)
				{
					nearest = distance;
					gap = point.y - end.y;
				}
			}
		if (nearest <= kSamePoint && std::abs(gap) > 0.001)
			gaps.push_back({joint.track, joint.neighbour, gap});
	}
	return gaps;
}

void editor_track::keep_heights(chain const &Chain, std::vector<segment_data> &Pieces)
{
	auto existing{route_of(Chain)};
	auto const samples{sample_route(existing, 1.0)};
	if (samples.empty() || existing.length <= 1e-6)
		return;
	std::vector<double> lengths;
	double total{0.0};
	for (auto const &piece : Pieces)
	{
		lengths.push_back(bezier(piece).plan_length());
		total += lengths.back();
	}
	if (total <= 1e-6)
		return;
	auto const scale{existing.length / total};
	double chainage{0.0};
	for (std::size_t i = 0; i < Pieces.size(); ++i)
	{
		auto const from{chainage * scale};
		auto const to{(chainage + lengths[i]) * scale};
		set_heights(Pieces[i], sampled_elevation(samples, from), sampled_elevation(samples, to), sampled_grade(samples, from) * scale, sampled_grade(samples, to) * scale);
		chainage += lengths[i];
	}
}

std::vector<editor_track::switch_template> editor_track::find_switch_templates()
{
	std::vector<switch_template> result;
	std::map<std::string, std::size_t> lookup;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->m_paths.size() < 2 || (track->iCategoryFlag & 0x80) != 0)
			continue;
		auto const &main{track->m_paths[0]};
		auto const origin{main.points[segment_data::point::start]};
		auto const chord{plan_of(main.points[segment_data::point::end] - origin)};
		if (glm::length(chord) < 1.0)
			continue;
		auto const axis{glm::normalize(chord)};
		glm::dvec2 const normal{-axis.y, axis.x};
		auto const tolocal = [&](glm::dvec3 const &Vector) {
			auto const planar{plan_of(Vector)};
			return glm::dvec3{glm::dot(planar, normal), Vector.y, glm::dot(planar, axis)};
		};
		switch_template entry;
		entry.source = track;
		for (int i = 0; i < 2; ++i)
		{
			auto const &path{track->m_paths[i]};
			auto &local{entry.local[i]};
			local = path;
			local.points[segment_data::point::start] = tolocal(path.points[segment_data::point::start] - origin);
			local.points[segment_data::point::end] = tolocal(path.points[segment_data::point::end] - origin);
			local.points[segment_data::point::control1] = tolocal(path.points[segment_data::point::control1]);
			local.points[segment_data::point::control2] = tolocal(path.points[segment_data::point::control2]);
		}
		if (glm::length(plan_of(entry.local[1].points[segment_data::point::start])) > kSamePoint)
			continue;
		if (entry.local[1].points[segment_data::point::end].x < 0.0)
		{
			for (auto &path : entry.local)
			{
				for (auto &point : path.points)
					point.x = -point.x;
				path.rolls = {-path.rolls[0], -path.rolls[1]};
			}
		}
		entry.length = entry.local[0].points[segment_data::point::end].z;
		auto const tangent{path_tangent(entry.local[1], true)};
		auto const angle{std::atan2(std::abs(tangent.x), tangent.z)};
		entry.ratio = angle > 1e-6 ? 1.0 / std::tan(angle) : 0.0;
		entry.radius = path_radius(entry.local[1], 0.25);
		auto const key{format("%.2f|%.2f|%.2f|%s|%s", entry.length, entry.local[1].points[segment_data::point::end].x, entry.local[1].points[segment_data::point::end].z, material_name(track->m_material1).c_str(), material_name(track->m_material2).c_str())};
		auto const found{lookup.find(key)};
		if (found != lookup.end())
		{
			++result[found->second].count;
			continue;
		}
		entry.count = 1;
		entry.label = format("L %.2f m  1:%.1f  R %.0f m  %s", entry.length, entry.ratio, entry.radius, material_name(track->m_material1).c_str());
		lookup.emplace(key, result.size());
		result.push_back(entry);
	}
	std::sort(result.begin(), result.end(), [](switch_template const &A, switch_template const &B) { return A.count > B.count; });
	return result;
}

std::vector<segment_data> editor_track::place_switch(switch_template const &Template, glm::dvec3 const &Origin, glm::dvec2 const &Direction, int const Side, double const Grade)
{
	std::vector<segment_data> result;
	glm::dvec2 const normal{-Direction.y, Direction.x};
	auto const side{static_cast<double>(Side)};
	auto const vector = [&](glm::dvec3 const &Local) {
		auto const planar{Direction * Local.z + normal * (Local.x * side)};
		return glm::dvec3{planar.x, Local.y + Grade * Local.z, planar.y};
	};
	for (auto const &local : Template.local)
	{
		segment_data path{local};
		path.points[segment_data::point::start] = Origin + vector(local.points[segment_data::point::start]);
		path.points[segment_data::point::end] = Origin + vector(local.points[segment_data::point::end]);
		path.points[segment_data::point::control1] = vector(local.points[segment_data::point::control1]);
		path.points[segment_data::point::control2] = vector(local.points[segment_data::point::control2]);
		if (Side < 0)
			path.rolls = {-local.rolls[0], -local.rolls[1]};
		result.push_back(path);
	}
	return result;
}

std::vector<editor_track::switch_template> editor_track::standard_switch_templates()
{
	struct definition
	{
		char const *name;
		double radius;
		double ratio;
		double a;
		double b;
	};
	definition const definitions[] = {
		{"Rz 1:9 R190", 190.0, 9.0, 10.523, 16.615},
		{"Rz 1:9 R300", 300.0, 9.0, 16.615, 16.615},
		{"Rz 1:12 R500", 500.0, 12.0, 20.797, 0.0},
		{"Rz 1:14 R760", 760.0, 14.0, 0.0, 0.0},
		{"Rz 1:18,5 R1200", 1200.0, 18.5, 32.409, 34.943},
		{"Rz 1:26,5 R2500", 2500.0, 26.5, 0.0, 0.0},
	};
	std::vector<switch_template> result;
	for (auto const &definition : {std::pair<double, char const *>{190.0, "1:9"}, {300.0, "1:9"}, {500.0, "1:12"}})
	{
		switch_template slip;
		slip.double_slip = true;
		slip.radius = definition.first;
		slip.label = std::string{"Rkpd "} + definition.second + " R" + std::to_string(static_cast<int>(definition.first)) + " double slip (click at a crossing of two straights)";
		result.push_back(slip);
	}
	for (auto const &definition : definitions)
	{
		auto const angle{std::atan(1.0 / definition.ratio)};
		auto const a{definition.a > 0.0 ? definition.a : definition.radius * std::tan(angle * 0.5)};
		auto const estimated{definition.b <= 0.0};
		auto const b{estimated ? 1.87 * definition.ratio : definition.b};
		auto const offset{b * std::tan(angle)};
		switch_template entry;
		entry.a = a;
		entry.b = b;
		entry.length = a + b;
		entry.ratio = definition.ratio;
		entry.radius = definition.radius;
		auto &main{entry.local[0]};
		main.points[segment_data::point::end] = {0.0, 0.0, a + b};
		auto &diverging{entry.local[1]};
		diverging.points[segment_data::point::end] = {offset, 0.0, a + b};
		diverging.points[segment_data::point::control1] = {0.0, 0.0, a * 2.0 / 3.0};
		diverging.points[segment_data::point::control2] = {-offset * 2.0 / 3.0, 0.0, -b * 2.0 / 3.0};
		diverging.radius = static_cast<float>(definition.radius);
		entry.label = std::string{definition.name} + "  a " + std::to_string(a).substr(0, std::to_string(a).find('.') + 4) + " b " + std::to_string(b).substr(0, std::to_string(b).find('.') + 4) + (estimated ? " (b est.)" : "");
		result.push_back(entry);
	}
	return result;
}

TTrack *editor_track::create_switch(switch_template const &Template, std::vector<segment_data> const &Paths, TTrack const &Style, std::string const &Name)
{
	if (Paths.size() < 2)
		return nullptr;
	TTrack *track{nullptr};
	if (Template.source != nullptr)
	{
		track = clone(*Template.source);
	}
	else
	{
		char const *environments[] = {"flat", "mountains", "canyon", "tunnel", "bridge", "bank"};
		auto const environment{Style.eEnvironment >= e_flat && Style.eEnvironment <= e_bank ? environments[Style.eEnvironment] : "flat"};
		std::ostringstream text;
		text.precision(std::numeric_limits<double>::digits10);
		text << "switch " << Template.length << ' ' << Style.fTrackWidth << ' ' << Style.fFriction << ' ' << 10.0 << ' ' << Style.iQualityFlag << ' ' << 0 << ' ' << environment << ' ';
		auto const rails{Style.m_material1};
		auto const trackbed{Style.eType == tt_Switch ? (Style.SwitchExtension ? Style.SwitchExtension->m_material3 : null_handle) : Style.m_material2};
		if (Style.m_visible)
			text << "vis " << material_name(rails) << ' ' << Style.fTexLength << ' ' << material_name(rails) << ' ' << texture_height(Style) << ' ' << Style.fTexWidth << ' ' << Style.fTexSlope << ' ';
		else
			text << "unvis ";
		for (int i = 0; i < 2; ++i)
		{
			auto const &path{Paths[i]};
			auto const &start{path.points[segment_data::point::start]};
			auto const &control1{path.points[segment_data::point::control1]};
			auto const &control2{path.points[segment_data::point::control2]};
			auto const &end{path.points[segment_data::point::end]};
			text << start.x << ' ' << start.y << ' ' << start.z << ' ' << path.rolls[0] << ' ' << control1.x << ' ' << control1.y << ' ' << control1.z << ' ' << control2.x << ' ' << control2.y << ' ' << control2.z << ' ' << end.x << ' ' << end.y << ' ' << end.z << ' ' << path.rolls[1] << ' ' << path.radius << ' ';
		}
		if (trackbed != null_handle)
			text << "trackbed " << material_name(trackbed) << ' ';
		if ((Style.iCategoryFlag & 15) == 1 && false == Style.m_profile1.first.empty())
			text << "railprofile " << Style.m_profile1.first << ' ';
		text << "endtrack\n";
		track = load_path(text.str(), Style, Name);
	}
	track->m_paths.assign(Paths.begin(), Paths.begin() + 2);
	return track;
}

bool editor_track::touches(TTrack const &Track, glm::dvec3 const &Point)
{
	for (auto const &path : Track.m_paths)
		for (auto const index : {segment_data::point::start, segment_data::point::end})
			if (glm::distance(path.points[index], Point) <= kSamePoint)
				return true;
	return false;
}

TTrack *editor_track::outside_neighbour(straight const &Line, bool const Atend)
{
	auto const &point{Atend ? Line.end : Line.start};
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
	{
		auto const &path{Line.tracks[i]->m_paths[Line.paths[i]]};
		for (auto const end : {false, true})
		{
			if (glm::distance(path.points[end ? segment_data::point::end : segment_data::point::start], point) > kSamePoint)
				continue;
			auto *neighbour{neighbour_at(*Line.tracks[i], Line.paths[i], end)};
			if (neighbour != nullptr && std::find(Line.tracks.begin(), Line.tracks.end(), neighbour) == Line.tracks.end())
				return neighbour;
		}
	}
	return nullptr;
}

std::vector<TTrack *> editor_track::straight_affected(std::vector<straight> const &Lines)
{
	std::vector<TTrack *> result;
	for (auto const &line : Lines)
		for (auto *track : line.tracks)
			add_unique(result, track);
	auto const members{result};
	auto const member = [&](TTrack const *Track) { return std::find(members.begin(), members.end(), Track) != members.end(); };
	for (auto *track : members)
	{
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			for (auto const atend : {false, true})
			{
				auto *neighbour{neighbour_at(*track, i, atend)};
				if (neighbour == nullptr || member(neighbour))
					continue;
				add_unique(result, neighbour);
				if (false == has_switch_paths(*neighbour))
					continue;
				for (auto *next : neighbours(*neighbour))
					if (false == member(next))
						add_unique(result, next);
			}
		}
	}
	return result;
}

void editor_track::move_straights(std::vector<straight> const &Lines, std::vector<std::pair<glm::dvec3, glm::dvec3>> const &Ends)
{
	std::vector<TTrack *> members;
	for (auto const &line : Lines)
		for (auto *track : line.tracks)
			add_unique(members, track);
	auto const member = [&](TTrack const *Track) { return std::find(members.begin(), members.end(), Track) != members.end(); };
	for (std::size_t i = 0; i < Lines.size() && i < Ends.size(); ++i)
		move_straight(Lines[i], Ends[i].first, Ends[i].second, member);
}

void editor_track::move_straight(straight const &Line, glm::dvec3 const &Start, glm::dvec3 const &End, std::function<bool(TTrack const *)> const &Member)
{
	if (Line.tracks.empty() || Line.length < 1e-6)
		return;
	auto const &direction{Line.direction};
	auto const newplan{plan_of(End - Start)};
	auto const newlength{glm::length(newplan)};
	if (newlength < 1e-3)
		return;
	auto const newdirection{newplan / newlength};
	auto const angle{std::atan2(cross(direction, newdirection), glm::dot(direction, newdirection))};
	auto const along = [&](glm::dvec3 const &Point) { return glm::dot(plan_of(Point - Line.start), direction); };

	struct interval
	{
		double from, to;
		bool rigid;
	};
	std::vector<interval> intervals;
	double rigidlength{0.0};
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
	{
		auto const &path{Line.tracks[i]->m_paths[Line.paths[i]]};
		auto const a{along(path.points[segment_data::point::start])};
		auto const b{along(path.points[segment_data::point::end])};
		auto const rigid{has_switch_paths(*Line.tracks[i])};
		intervals.push_back({std::min(a, b), std::max(a, b), rigid});
		if (rigid)
			rigidlength += std::abs(b - a);
	}
	std::sort(intervals.begin(), intervals.end(), [](interval const &A, interval const &B) { return A.from < B.from; });
	auto const scale{Line.length - rigidlength > 1e-6 ? (newlength - rigidlength) / (Line.length - rigidlength) : 1.0};
	auto const remap = [&](double const Along) {
		double mapped{0.0}, previous{0.0};
		for (auto const &span : intervals)
		{
			if (Along <= span.from)
				return mapped + (Along - previous) * scale;
			mapped += (span.from - previous) * scale;
			previous = span.from;
			auto const factor{span.rigid ? 1.0 : scale};
			if (Along <= span.to)
				return mapped + (Along - previous) * factor;
			mapped += (span.to - previous) * factor;
			previous = span.to;
		}
		return mapped + (Along - previous) * scale;
	};
	auto const transform = [&](glm::dvec3 const &Point) {
		auto const a{along(Point)};
		auto const offset{cross(direction, plan_of(Point - Line.start))};
		auto const mapped{remap(a)};
		auto const oldheight{Line.start.y + (Line.end.y - Line.start.y) * a / Line.length};
		auto const newheight{Start.y + (End.y - Start.y) * mapped / newlength};
		auto const moved{plan_of(Start) + newdirection * mapped + glm::dvec2{-newdirection.y, newdirection.x} * offset};
		return glm::dvec3{moved.x, Point.y - oldheight + newheight, moved.y};
	};

	auto const &member{Member};
	std::vector<std::pair<glm::dvec3, glm::dvec3>> joints;
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
	{
		auto &track{*Line.tracks[i]};
		auto const before{track.m_paths};
		if (has_switch_paths(track))
		{
			auto const pivot{before[Line.paths[i]].points[segment_data::point::start]};
			rigid_move(track, pivot, transform(pivot), angle);
		}
		else
		{
			for (auto &path : track.m_paths)
			{
				for (auto const index : {segment_data::point::start, segment_data::point::end})
					path.points[index] = transform(path.points[index]);
				for (auto const index : {segment_data::point::control1, segment_data::point::control2})
				{
					auto &control{path.points[index]};
					auto const moved{turned(plan_of(control), angle) * scale};
					control = {moved.x, control.y * scale, moved.y};
				}
			}
		}
		for (std::size_t j = 0; j < before.size(); ++j)
			for (auto const index : {segment_data::point::start, segment_data::point::end})
				joints.emplace_back(before[j].points[index], track.m_paths[j].points[index]);
	}

	std::vector<TTrack *> moved;
	for (auto *track : Line.tracks)
	{
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			for (auto const atend : {false, true})
			{
				auto *neighbour{neighbour_at(*track, i, atend)};
				if (neighbour == nullptr || member(neighbour) || std::find(moved.begin(), moved.end(), neighbour) != moved.end())
					continue;
				moved.push_back(neighbour);
				for (auto const &joint : joints)
				{
					bool touches{false};
					for (auto const &path : neighbour->m_paths)
						for (auto const index : {segment_data::point::start, segment_data::point::end})
							touches |= glm::distance(path.points[index], joint.first) <= kSamePoint;
					if (false == touches)
						continue;
					if (has_switch_paths(*neighbour))
					{
						auto const before{neighbour->m_paths};
						rigid_move(*neighbour, joint.first, joint.second, angle);
						for (auto *next : neighbours(*neighbour))
						{
							if (member(next) || next == neighbour)
								continue;
							for (std::size_t j = 0; j < before.size(); ++j)
								for (auto const index : {segment_data::point::start, segment_data::point::end})
									follow_joint(*next, before[j].points[index], neighbour->m_paths[j].points[index], angle);
						}
					}
					else
					{
						follow_joint(*neighbour, joint.first, joint.second, angle);
					}
					break;
				}
			}
		}
	}
}

std::vector<TTrack *> editor_track::relay(chain const &Chain, std::vector<segment_data> const &Pieces)
{
	std::vector<TTrack *> created;
	auto const count{Chain.tracks.size()};
	if (count == 0 || Pieces.size() < count)
		return created;

	std::vector<TTrack *> tracks;
	std::size_t previous{count};
	for (std::size_t i = 0; i < Pieces.size(); ++i)
	{
		auto const index{i * count / Pieces.size()};
		TTrack *track;
		bool forward;
		if (index != previous)
		{
			track = Chain.tracks[index];
			forward = Chain.forward[index];
		}
		else
		{
			track = clone(*Chain.tracks[index]);
			created.push_back(track);
			forward = true;
		}
		previous = index;
		track->m_paths.resize(1);
		track->m_paths.front() = forward ? Pieces[i] : reversed(Pieces[i]);
		tracks.push_back(track);
	}
	commit(tracks);
	return created;
}

TTrack *editor_track::clone(TTrack const &Template)
{
	std::ostringstream text;
	Template.export_as_text_(text);
	auto content{text.str()};
	auto const type{content.find("track ")};
	if (type == 0)
		content.erase(0, 6);
	auto *track{load_path(content, Template)};
	track->m_friction = Template.m_friction;
	return track;
}

TTrack *editor_track::create_path(TTrack const &Style, segment_data const &Path)
{
	char const *environments[] = {"flat", "mountains", "canyon", "tunnel", "bridge", "bank"};
	auto const environment{Style.eEnvironment >= e_flat && Style.eEnvironment <= e_bank ? environments[Style.eEnvironment] : "flat"};
	auto const category{Style.iCategoryFlag & 15};
	std::ostringstream text;
	text.precision(std::numeric_limits<double>::digits10);
	text << (category == 2 ? "road " : category == 4 ? "river " : "normal ") << glm::distance(Path.points[segment_data::point::start], Path.points[segment_data::point::end]) << ' ' << Style.fTrackWidth << ' ' << Style.fFriction << ' ' << Style.fSoundDistance << ' ' << Style.iQualityFlag << ' ' << 0 << ' ' << environment << ' ';
	auto const ballast{Style.eType == tt_Switch ? (Style.SwitchExtension ? Style.SwitchExtension->m_material3 : null_handle) : Style.m_material2};
	if (Style.m_visible)
		text << "vis " << material_name(Style.m_material1) << ' ' << Style.fTexLength << ' ' << material_name(ballast) << ' ' << texture_height(Style) << ' ' << Style.fTexWidth << ' ' << Style.fTexSlope << ' ';
	else
		text << "unvis ";
	auto const &start{Path.points[segment_data::point::start]};
	auto const &control1{Path.points[segment_data::point::control1]};
	auto const &control2{Path.points[segment_data::point::control2]};
	auto const &end{Path.points[segment_data::point::end]};
	text << start.x << ' ' << start.y << ' ' << start.z << ' ' << Path.rolls[0] << ' ' << control1.x << ' ' << control1.y << ' ' << control1.z << ' ' << control2.x << ' ' << control2.y << ' ' << control2.z << ' ' << end.x << ' ' << end.y << ' ' << end.z << ' ' << Path.rolls[1] << ' ' << Path.radius << ' ';
	auto const speed{velocity(Style)};
	if (speed > 0.0)
		text << "velocity " << speed << ' ';
	if (category == 1 && false == Style.m_profile1.first.empty())
		text << "railprofile " << Style.m_profile1.first << ' ';
	text << "endtrack\n";
	auto *track{load_path(text.str(), Style)};
	track->m_paths = {Path};
	return track;
}

glm::dvec3 editor_track::point_at(TTrack const &Track, double const T)
{
	auto const &path{Track.m_paths.front()};
	auto const p0{path.points[segment_data::point::start]};
	auto const p3{path.points[segment_data::point::end]};
	auto const p1{p0 + path.points[segment_data::point::control1]};
	auto const p2{p3 + path.points[segment_data::point::control2]};
	auto const u{1.0 - T};
	return u * u * u * p0 + 3.0 * u * u * T * p1 + 3.0 * u * T * T * p2 + T * T * T * p3;
}

double editor_track::nearest_parameter(TTrack const &Track, glm::dvec3 const &Point)
{
	if (Track.m_paths.empty())
		return 0.5;
	auto const distance = [&](double const T) { return geometry::plan_distance(point_at(Track, T), Point); };
	double best{0.5};
	for (int i = 0; i <= 200; ++i)
		if (distance(i / 200.0) < distance(best))
			best = i / 200.0;
	double step{1.0 / 200.0};
	for (int i = 0; i < 30; ++i)
	{
		step *= 0.5;
		if (best - step > 0.0 && distance(best - step) < distance(best))
			best -= step;
		else if (best + step < 1.0 && distance(best + step) < distance(best))
			best += step;
	}
	return best;
}

TTrack *editor_track::split_path(TTrack &Track, double const T)
{
	if (Track.eType != tt_Normal || Track.m_paths.empty() || T <= 0.001 || T >= 0.999)
		return nullptr;
	auto const path{Track.m_paths.front()};
	auto const p0{path.points[segment_data::point::start]};
	auto const p3{path.points[segment_data::point::end]};
	segment_data first{path};
	segment_data second{path};
	auto const roll{static_cast<float>(path.rolls[0] + (path.rolls[1] - path.rolls[0]) * T)};
	if (path.points[segment_data::point::control1] == glm::dvec3{} && path.points[segment_data::point::control2] == glm::dvec3{})
	{
		auto const middle{glm::mix(p0, p3, T)};
		first.points[segment_data::point::end] = middle;
		second.points[segment_data::point::start] = middle;
	}
	else
	{
		auto const p1{p0 + path.points[segment_data::point::control1]};
		auto const p2{p3 + path.points[segment_data::point::control2]};
		auto const p01{glm::mix(p0, p1, T)};
		auto const p12{glm::mix(p1, p2, T)};
		auto const p23{glm::mix(p2, p3, T)};
		auto const p012{glm::mix(p01, p12, T)};
		auto const p123{glm::mix(p12, p23, T)};
		auto const middle{glm::mix(p012, p123, T)};
		first.points[segment_data::point::control1] = p01 - p0;
		first.points[segment_data::point::control2] = p012 - middle;
		first.points[segment_data::point::end] = middle;
		second.points[segment_data::point::start] = middle;
		second.points[segment_data::point::control1] = p123 - middle;
		second.points[segment_data::point::control2] = p23 - p3;
	}
	first.rolls[1] = roll;
	second.rolls[0] = roll;
	auto *created{create_path(Track, second)};
	Track.m_paths.front() = first;
	if (s_observer != nullptr)
		s_observer->track_split(Track, *created, first);
	commit({&Track, created});
	return created;
}

TTrack *editor_track::load_path(std::string const &Text, TTrack const &Template, std::string const &Name)
{
	cParser parser(Text, cParser::buffer_TEXT);
	scene::node_data data;
	data.type = "track";
	auto const base{Template.name().empty() || Template.name() == "none" ? std::string{"editor_track"} : Template.name()};
	data.name = Name;
	data.layer = Template.layer();
	for (int i = 1; data.name.empty() || simulation::Paths.find(data.name) != nullptr; ++i)
		data.name = base + "_" + std::to_string(i);
	auto *track = new TTrack(data);
	track->m_rangesquaredmin = Template.m_rangesquaredmin;
	track->m_rangesquaredmax = Template.m_rangesquaredmax;
	track->Load(&parser, glm::dvec3{});
	for (auto *events : {&track->m_events0, &track->m_events1, &track->m_events2, &track->m_events0all, &track->m_events1all, &track->m_events2all})
		events->clear();
	track->m_events = false;
	simulation::Paths.insert(track);
	return track;
}

void editor_track::retire(TTrack &Track)
{
	auto const adjoining{neighbours(Track)};
	disconnect(Track);
	simulation::Region->erase_and_unregister(&Track);
	Track.m_editorremoved = true;
	for (auto *neighbour : adjoining)
	{
		update_transition(*neighbour);
		rebuild_geometry(*neighbour);
	}
	if (s_observer != nullptr)
		s_observer->track_retired(Track);
}

void editor_track::rebuild_geometry(TTrack &Track)
{
	auto &section{simulation::Region->section(Track.location())};
	Track.rebuild_geometry(section.m_geometrycreated ? section.m_geometrybank : gfx::geometrybank_handle{});
}
