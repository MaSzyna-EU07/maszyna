/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorTrack.hpp"

#include "world/Track.h"
#include "scene/scene.h"
#include "simulation/simulation.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

// end points closer than this are treated as the same point
double const kSamePoint{0.05};

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

bool is_end(editor_track::point_kind const Kind)
{
	return Kind == editor_track::point_kind::start || Kind == editor_track::point_kind::end;
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

} // namespace

editor_track::state editor_track::capture(TTrack const &Track)
{
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
		return false; // auto-generated helper path
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
	if (false == Track.Dynamics.empty())
	{
		Reason = "There are vehicles placed on this path";
		return false;
	}
	Reason.clear();
	return true;
}

void editor_track::commit(std::vector<TTrack *> const &Tracks)
{
	if (Tracks.empty())
		return;

	// paths whose render geometry depends on the modified ones
	std::vector<TTrack *> affected;
	for (auto *track : Tracks)
	{
		add_unique(affected, track);
		for (auto *neighbour : neighbours(*track))
			add_unique(affected, neighbour);
	}
	// take the paths out of the scene while their ends still match the old segments...
	for (auto *track : Tracks)
	{
		disconnect(*track);
		simulation::Region->erase_and_unregister(track);
	}
	// ...rebuild the segments and put the paths back in their new place...
	for (auto *track : Tracks)
	{
		track->init_segments(false);
		track->update_location();
		track->m_area.radius = -1.f; // forces radius recalculation
		simulation::Region->insert_and_register(track);
	}
	// ...connect them with whatever they touch now...
	for (auto *track : Tracks)
		join(*track);
	for (auto *track : Tracks)
		for (auto *neighbour : neighbours(*track))
			add_unique(affected, neighbour);
	// ...and update the visuals
	for (auto *track : affected)
		update_transition(*track);
	for (auto *track : affected)
		rebuild_geometry(*track);
	for (auto *track : Tracks)
		track->mark_dirty();
}

void editor_track::commit_parameters(TTrack &Track)
{
	Track.fTrackWidth2 = Track.fTrackWidth;
	// segments are re-created from unchanged data, but road texture proportions and roll fix depend on parameters
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
		// control vectors are relative to their end points, so they follow them automatically.
		// other paths of the same piece sharing this point (the common start of a switch) follow as well
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
	if (false == is_end(Point.kind))
		return false;
	return false == connected_points(Track, point_position(Track, Point)).empty();
}

editor_track::snap_target editor_track::find_snap_target(TTrack const &Track, glm::dvec3 const &Position, double const Radius, std::vector<TTrack const *> const &Exclude)
{
	snap_target result;
	result.distance = std::numeric_limits<double>::max();

	auto const excluded = [&](TTrack const *Other) { return std::find(Exclude.begin(), Exclude.end(), Other) != Exclude.end(); };
	auto const category{Track.iCategoryFlag & 15};
	// NOTE: copy, the region returns its scratchpad
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
						// skip ends already connected with something which stays in place
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
	if (Target.track == nullptr || false == is_end(Point.kind))
		return;

	move_point(Track, Point, Target.position);
	if (false == Aligntangent)
		return;

	// the path continues the target, so it leaves the shared point in the opposite direction
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
		// same rules as TTrack::Switch()
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
		// like during scenery load, switches are connected from the side of the regular paths
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
		// same connections as in path_table::InitTracks(), but made without moving the switch blades
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
	// same rule as TTrack::ConnectNextPrev(): transition is drawn by the path whose end 2 meets start of a different path
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
	// same as TTrack::SetConnections(), without switching back to the base position
	auto &extension{*Switch.SwitchExtension};
	auto const path{Path & 1};
	extension.pNexts[path] = Switch.trNext;
	extension.pPrevs[path] = Switch.trPrev;
	extension.iNextDirection[path] = Switch.iNextDirection;
	extension.iPrevDirection[path] = Switch.iPrevDirection;
	if (Switch.eType == tt_Switch)
	{
		// switch paths share their start point, so they share the path connected there as well
		extension.pPrevs[path ^ 1] = Switch.trPrev;
		extension.iPrevDirection[path ^ 1] = Switch.iPrevDirection;
	}
}

void editor_track::rebuild_geometry(TTrack &Track)
{
	auto &section{simulation::Region->section(Track.location())};
	// sections which weren't displayed yet create geometry of their content on their own
	Track.rebuild_geometry(section.m_geometrycreated ? section.m_geometrybank : gfx::geometrybank_handle{});
}
