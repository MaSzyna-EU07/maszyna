/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormoderide.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorKilometrage.hpp"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "vehicle/Camera.h"
#include "world/Segment.h"
#include "world/Track.h"
#include "imgui/imgui.h"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace track_ride_walk
{

namespace
{

double plan_length(glm::dvec3 const &Vector)
{
	return std::hypot(Vector.x, Vector.z);
}

glm::dvec3 plan_unit(glm::dvec3 const &Vector)
{
	auto const length{plan_length(Vector)};
	return length > 1e-9 ? glm::dvec3{Vector.x / length, 0.0, Vector.z / length} : glm::dvec3{0.0};
}

glm::dvec3 point_of(TSegment const &Segment, double const Distance)
{
	glm::dvec3 point{0.0};
	glm::vec3 rotation{0.0f};
	Segment.RaPositionGet(std::clamp(Distance, 0.0, Segment.GetLength()), point, rotation);
	return point;
}

// the track goes on there for the camera: there is one, it wasn't removed, it isn't a crossroads of roads
bool passable(TTrack const *Track)
{
	return Track != nullptr && false == Track->m_editorremoved && Track->eType != tt_Cross && segment({const_cast<TTrack *>(Track), Track->eType == tt_Switch ? 0 : -1, 0.0}) != nullptr;
}

// the path of a switch entered from its frog: the one whose end at Point2 is nearest to where the camera came from
int path_from_frog(TTrack const &Switch, glm::dvec3 const &From)
{
	auto best{0};
	auto nearest{std::numeric_limits<double>::max()};
	for (int path = 0; path < Switch.path_count(); ++path)
	{
		auto const *segment{Switch.path_segment(path)};
		if (segment == nullptr)
			continue;
		if (auto const distance{glm::distance(point_of(*segment, segment->GetLength()), From)}; distance < nearest)
		{
			nearest = distance;
			best = path;
		}
	}
	return best;
}

// the next track over the joint at the end the place is moving to (Direction 1: Point2, -1: Point1); Place becomes the
// start of it and Direction the way along it. False where the track ends
bool cross_joint(place &Place, int &Direction, int const Choice, bool &Facing)
{
	auto const *segment{track_ride_walk::segment(Place)};
	if (segment == nullptr)
		return false;
	auto const exit{point_of(*segment, Direction > 0 ? segment->GetLength() : 0.0)};
	double way{static_cast<double>(Direction)};
	auto *next{Place.track->Connected(Direction, way, Place.track->eType == tt_Switch ? Place.path : -1)};
	if (false == passable(next))
		return false;
	auto const direction{way > 0.0 ? 1 : -1};
	auto path{-1};
	if (next->eType == tt_Switch)
	{
		// both paths of a switch start at its Point1, at the blades
		if (direction > 0)
		{
			path = switch_path(*next, Choice);
			Facing = true;
		}
		else
			path = path_from_frog(*next, exit);
	}
	Place.track = next;
	Place.path = path;
	Direction = direction;
	auto const *entered{track_ride_walk::segment(Place)};
	Place.s = direction > 0 ? 0.0 : entered->GetLength();
	return true;
}

} // namespace

TSegment *segment(place const &Place)
{
	if (Place.track == nullptr)
		return nullptr;
	if (Place.track->eType == tt_Switch)
		return Place.track->SwitchExtension != nullptr ? Place.track->path_segment(std::clamp(Place.path, 0, 1)) : nullptr;
	return Place.track->Segment.get();
}

bool frame(place const &Place, glm::dvec3 &Point, glm::dvec3 &Forward)
{
	auto const *path{segment(Place)};
	if (path == nullptr)
		return false;
	auto const length{path->GetLength()};
	Point = point_of(*path, Place.s);
	// the direction from two points around the place, the same on any kind of path
	auto const step{std::min(0.5, std::max(length * 0.5, 1e-3))};
	auto const from{std::max(0.0, Place.s - step)};
	auto const to{std::min(length, Place.s + step)};
	Forward = to > from ? point_of(*path, to) - point_of(*path, from) : glm::dvec3{0.0, 0.0, 1.0};
	auto const size{glm::length(Forward)};
	Forward = size > 1e-9 ? Forward / size : glm::dvec3{0.0, 0.0, 1.0};
	return true;
}

int switch_path(TTrack const &Switch, int const Choice)
{
	if (Switch.eType != tt_Switch || Switch.SwitchExtension == nullptr)
		return -1;
	// how much each path turns away from the way in at the blades
	double turn[2]{0.0, 0.0};
	for (int path = 0; path < 2; ++path)
	{
		auto const *segment{Switch.path_segment(path)};
		if (segment == nullptr || segment->GetLength() <= 0.0)
		{
			turn[path] = std::numeric_limits<double>::max();
			continue;
		}
		auto const length{segment->GetLength()};
		auto const in{plan_unit(point_of(*segment, std::min(1.0, length * 0.5)) - point_of(*segment, 0.0))};
		auto const out{plan_unit(point_of(*segment, length) - point_of(*segment, std::max(0.0, length - 1.0)))};
		turn[path] = std::acos(std::clamp(glm::dot(in, out), -1.0, 1.0));
	}
	auto const straighter{turn[1] < turn[0] ? 1 : 0};
	return (Choice % 2) == 0 ? straighter : 1 - straighter;
}

halt walk(place &Place, double const Distance, int const Choice, int &Flips, bool &Facing)
{
	Flips = 0;
	Facing = false;
	if (segment(Place) == nullptr)
		return halt::end;
	Place.s = std::clamp(Place.s, 0.0, segment(Place)->GetLength());
	auto direction{Distance >= 0.0 ? 1 : -1};
	auto left{std::abs(Distance)};
	// the joints passed in one step; a ring of tracks of no length would hold it here for ever
	for (int joint = 0; joint < 10000; ++joint)
	{
		auto const length{segment(Place)->GetLength()};
		auto const target{Place.s + direction * left};
		if (target >= 0.0 && target <= length)
		{
			Place.s = target;
			return halt::none;
		}
		left = std::max(0.0, left - (direction > 0 ? length - Place.s : Place.s));
		auto const before{direction};
		if (false == cross_joint(Place, direction, Choice, Facing))
		{
			Place.s = direction > 0 ? length : 0.0;
			return halt::end;
		}
		if (direction != before)
			++Flips;
	}
	return halt::end;
}

TTrack *facing_switch_ahead(place Place, int Direction, double const Reach, double &Distance)
{
	auto travelled{0.0};
	for (int joint = 0; joint < 1000; ++joint)
	{
		auto const *path{segment(Place)};
		if (path == nullptr)
			return nullptr;
		travelled += Direction > 0 ? path->GetLength() - Place.s : Place.s;
		if (travelled > Reach)
			return nullptr;
		auto facing{false};
		if (false == cross_joint(Place, Direction, 0, facing))
			return nullptr;
		if (facing)
		{
			Distance = travelled;
			return Place.track;
		}
	}
	return nullptr;
}

double nearest_distance(TSegment const &Segment, glm::dvec3 const &Point)
{
	auto const length{Segment.GetLength()};
	if (length <= 0.0)
		return 0.0;
	auto const off = [&](double const Distance) { return plan_length(point_of(Segment, Distance) - Point); };
	auto const steps{std::clamp(static_cast<int>(std::ceil(length / 2.0)), 8, 4000)};
	auto const step{length / steps};
	auto best{0.0};
	auto nearest{std::numeric_limits<double>::max()};
	for (int i = 0; i <= steps; ++i)
	{
		if (auto const distance{off(i * step)}; distance < nearest)
		{
			nearest = distance;
			best = i * step;
		}
	}
	// golden section within a step on either side
	auto low{std::max(0.0, best - step)};
	auto high{std::min(length, best + step)};
	constexpr double ratio{0.6180339887498949};
	for (int i = 0; i < 40 && high - low > 1e-4; ++i)
	{
		auto const a{high - ratio * (high - low)};
		auto const b{low + ratio * (high - low)};
		if (off(a) < off(b))
			high = b;
		else
			low = a;
	}
	return 0.5 * (low + high);
}

} // namespace track_ride_walk

namespace
{

constexpr double glide_time{0.35}; // s
constexpr double side_reach{20.0}; // m to the track alongside
constexpr double switch_reach{400.0}; // m ahead the switch is shown

glm::dvec3 plan_right(glm::dvec3 const &Forward)
{
	auto const length{std::hypot(Forward.x, Forward.z)};
	return length > 1e-9 ? glm::dvec3{-Forward.z / length, 0.0, Forward.x / length} : glm::dvec3{1.0, 0.0, 0.0};
}

// which way a path of a switch turns away from the other one, seen from the blades
char const *turn_name(TTrack const &Switch, int const Path)
{
	auto const *path{Switch.path_segment(Path)};
	auto const *other{Switch.path_segment(1 - Path)};
	if (path == nullptr || other == nullptr)
		return STR_C("straight on");
	glm::dvec3 start, forward;
	track_ride_walk::frame({const_cast<TTrack *>(&Switch), Path, 0.0}, start, forward);
	glm::dvec3 end, ignored;
	track_ride_walk::frame({const_cast<TTrack *>(&Switch), Path, path->GetLength()}, end, ignored);
	glm::dvec3 otherend;
	track_ride_walk::frame({const_cast<TTrack *>(&Switch), 1 - Path, other->GetLength()}, otherend, ignored);
	if (track_ride_walk::switch_path(Switch, 0) == Path)
		return STR_C("straight on");
	auto const right{plan_right(forward)};
	return glm::dot(end - otherend, right) > 0.0 ? STR_C("to the right") : STR_C("to the left");
}

} // namespace

bool track_ride::enter(TTrack *Track, int const Path, glm::dvec3 const &Near, glm::dvec3 const &Look, bool const Orient)
{
	if (Track == nullptr || Track->m_editorremoved || Track->eType == tt_Cross)
		return false;
	track_ride_walk::place place{Track, Track->eType == tt_Switch ? std::clamp(Path, 0, 1) : -1, 0.0};
	auto const *path{track_ride_walk::segment(place)};
	if (path == nullptr)
		return false;
	place.s = track_ride_walk::nearest_distance(*path, Near);
	m_place = place;
	m_direction = 1;
	m_direction = look_direction(Look);
	m_speed = 0.0;
	m_choice = 0;
	m_forward = m_back = m_up = m_down = false;
	m_glide_pending = true;
	m_orient_pending = Orient;
	m_km.clear();
	m_km_age = 1e9;
	m_switch = nullptr;
	return true;
}

void track_ride::leave()
{
	m_place = {};
	m_speed = 0.0;
	m_switch = nullptr;
	m_forward = m_back = m_up = m_down = false;
}

int track_ride::look_direction(glm::dvec3 const &Look) const
{
	glm::dvec3 point, forward;
	if (false == track_ride_walk::frame(m_place, point, forward))
		return m_direction;
	auto const along{forward.x * Look.x + forward.z * Look.z};
	// looking across the track keeps the way it was
	if (std::abs(along) < 0.15 * std::hypot(Look.x, Look.z))
		return m_direction;
	return along >= 0.0 ? 1 : -1;
}

glm::dvec3 track_ride::rail_point() const
{
	glm::dvec3 point{0.0}, forward;
	track_ride_walk::frame(m_place, point, forward);
	return point;
}

void track_ride::cycle_choice()
{
	m_choice = (m_choice + 1) % 2;
}

bool track_ride::key(int const Key, int const Action, int const Mods, glm::dvec3 const &Look, std::string &Status)
{
	m_shift = (Mods & GLFW_MOD_SHIFT) != 0;
	m_ctrl = (Mods & GLFW_MOD_CONTROL) != 0;
	if (Key == GLFW_KEY_LEFT_SHIFT || Key == GLFW_KEY_RIGHT_SHIFT)
		m_shift = Action != GLFW_RELEASE;
	if (Key == GLFW_KEY_LEFT_CONTROL || Key == GLFW_KEY_RIGHT_CONTROL)
		m_ctrl = Action != GLFW_RELEASE;
	if (false == active())
		return false;
	auto const held{Action != GLFW_RELEASE};
	auto const press{Action == GLFW_PRESS};
	// with Ctrl or Alt the ride keeps only Ctrl+W and Ctrl+arrows, the much faster drive; the shortcuts of the editor get the
	// rest (Ctrl+S saves, Ctrl+Z and Ctrl+Y undo and redo). Releases go on to the editor too, it may have seen the key go down
	// before the ride
	auto const shortcut{(Mods & GLFW_MOD_ALT) != 0 || (m_ctrl && Key != GLFW_KEY_W && Key != GLFW_KEY_UP && Key != GLFW_KEY_DOWN)};
	auto const hold = [&](bool &Flag) {
		Flag = held && false == shortcut;
		return Flag;
	};
	switch (Key)
	{
	case GLFW_KEY_W:
	case GLFW_KEY_UP: return hold(m_forward);
	case GLFW_KEY_S:
	case GLFW_KEY_DOWN: return hold(m_back);
	case GLFW_KEY_E:
	case GLFW_KEY_PAGE_UP: return hold(m_up);
	case GLFW_KEY_Q:
	case GLFW_KEY_PAGE_DOWN: return hold(m_down);
	case GLFW_KEY_A:
	case GLFW_KEY_LEFT:
	case GLFW_KEY_D:
	case GLFW_KEY_RIGHT:
		if (shortcut)
			return false;
		if (press && false == jump_side(Key == GLFW_KEY_A || Key == GLFW_KEY_LEFT ? -1 : 1, Look))
			Status = STR("No track alongside");
		return held;
	case GLFW_KEY_Z:
		if (shortcut)
			return false;
		if (press)
			cycle_choice();
		return held;
	case GLFW_KEY_ESCAPE:
		// Esc closes an open menu or popup first, as it does in the rest of the editor
		if (ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
			return false;
		[[fallthrough]];
	case GLFW_KEY_Y:
		if (shortcut)
			return false;
		if (press)
			leave();
		return held;
	default: return false;
	}
}

void track_ride::wheel(double const Notches)
{
	m_height = std::clamp(m_height * std::pow(0.85, -Notches), 0.5, 500.0);
}

void track_ride::advance(TCamera &Camera, double const Deltatime)
{
	if (false == active())
		return;
	if (m_place.track->m_editorremoved || track_ride_walk::segment(m_place) == nullptr)
	{
		leave();
		return;
	}
	auto const dt{std::clamp(Deltatime, 0.0, 0.1)};
	// the window takes the keys, their releases among them: none is held for the ride then
	if (ImGui::GetIO().WantCaptureKeyboard)
		m_forward = m_back = m_up = m_down = false;
	// the path may have got shorter, an undo or a change of the track under the ride
	m_place.s = std::clamp(m_place.s, 0.0, track_ride_walk::segment(m_place)->GetLength());
	glm::dvec3 const look{-std::sin(Camera.Angle.y), 0.0, -std::cos(Camera.Angle.y)};
	// standing, the way ahead is the one the camera looks; a camera about to be turned along the track looks there already
	if (std::abs(m_speed) < 0.5 && false == m_orient_pending)
		m_direction = look_direction(look);

	auto const top{m_ctrl ? 250.0 : m_shift ? 60.0 : 20.0};
	auto const wanted{static_cast<double>((m_forward ? 1 : 0) - (m_back ? 1 : 0)) * top};
	auto const rate{(wanted == 0.0 || wanted * m_speed < 0.0 ? 2.5 : 1.2) * std::max(top, std::abs(m_speed))};
	m_speed = m_speed < wanted ? std::min(wanted, m_speed + rate * dt) : std::max(wanted, m_speed - rate * dt);

	if (m_speed != 0.0)
	{
		int flips{0};
		bool facing{false};
		auto const halted{track_ride_walk::walk(m_place, m_direction * m_speed * dt, m_choice, flips, facing)};
		if (flips % 2 != 0)
			m_direction = -m_direction;
		if (facing)
			m_choice = 0;
		if (halted == track_ride_walk::halt::end)
			m_speed = 0.0;
	}
	if (m_up != m_down)
		m_height = std::clamp(m_height + (m_up ? 1.0 : -1.0) * std::clamp(m_height, 2.0, 50.0) * (m_shift ? 3.0 : 1.0) * dt, 0.5, 500.0);

	// the switch ahead
	m_switch = track_ride_walk::facing_switch_ahead(m_place, m_speed < 0.0 ? -m_direction : m_direction, switch_reach, m_switch_distance);
	m_km_age += dt;
	if (m_km_age > 0.5)
	{
		m_km_age = 0.0;
		auto const at{rail_point()};
		double km{0.0};
		auto const line{kilometrage::line_at(at)};
		m_km = false == line.empty() && kilometrage::km_at(line, at, km) ? kilometrage::text(km) : std::string{};
	}

	if (m_orient_pending)
	{
		// along the way ahead, a little down to the rails
		m_orient_pending = false;
		glm::dvec3 point, forward;
		track_ride_walk::frame(m_place, point, forward);
		auto const ahead{forward * static_cast<double>(m_direction)};
		Camera.Angle.y = static_cast<float>(std::atan2(-ahead.x, -ahead.z));
		Camera.Angle.x = -0.12f;
		Camera.Angle.z = 0.0f;
	}
	auto const target{rail_point() + glm::dvec3{0.0, m_height, 0.0}};
	if (m_glide_pending)
	{
		m_glide_from = Camera.Pos;
		m_glide = 0.0;
		m_glide_pending = false;
	}
	if (m_glide < 1.0)
	{
		m_glide = std::min(1.0, m_glide + dt / glide_time);
		auto const eased{m_glide * m_glide * (3.0 - 2.0 * m_glide)};
		Camera.Pos = glm::mix(m_glide_from, target, eased);
	}
	else
		Camera.Pos = target;
	Camera.Velocity = glm::dvec3{0.0};
}

bool track_ride::jump_side(int const Side, glm::dvec3 const &Look)
{
	glm::dvec3 here, forward;
	if (false == track_ride_walk::frame(m_place, here, forward))
		return false;
	auto const ahead{forward * static_cast<double>(look_direction(Look))};
	auto const right{plan_right(ahead)};
	auto const category{m_place.track->iCategoryFlag & 15};
	auto const parallel{std::cos(30.0 * M_PI / 180.0)};
	track_ride_walk::place best;
	auto nearest{side_reach};
	auto bestforward{forward};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track == m_place.track || track->m_editorremoved || track->eType == tt_Cross || (track->iCategoryFlag & 15) != category)
			continue;
		for (int path = 0; path < track->path_count(); ++path)
		{
			track_ride_walk::place candidate{track, track->eType == tt_Switch ? path : -1, 0.0};
			auto const *segment{track_ride_walk::segment(candidate)};
			if (segment == nullptr)
				continue;
			// far off tracks out at once: no point of the path nearer than the reach
			auto const length{segment->GetLength()};
			auto const first{segment->FastGetPoint_0()};
			auto const last{segment->FastGetPoint_1()};
			if (std::min(std::hypot(first.x - here.x, first.z - here.z), std::hypot(last.x - here.x, last.z - here.z)) > 0.6 * length + side_reach + 1.0)
				continue;
			candidate.s = track_ride_walk::nearest_distance(*segment, here);
			glm::dvec3 point, along;
			track_ride_walk::frame(candidate, point, along);
			auto const offset{point - here};
			auto const across{(offset.x * right.x + offset.z * right.z) * Side};
			auto const lengthwise{offset.x * ahead.x + offset.z * ahead.z};
			// alongside on that side, about across from the camera, running the same way
			if (across < 1.5 || across >= nearest || std::abs(lengthwise) > 2.0 + 0.25 * across || std::abs(along.x * ahead.x + along.z * ahead.z) < parallel)
				continue;
			nearest = across;
			best = candidate;
			bestforward = along;
		}
	}
	if (best.track == nullptr)
		return false;
	auto const speed{m_speed};
	auto const goingahead{m_direction == look_direction(Look)};
	m_place = best;
	// the way ahead stays the same on the new track
	auto const aheadnow{bestforward.x * ahead.x + bestforward.z * ahead.z >= 0.0 ? 1 : -1};
	m_direction = goingahead ? aheadnow : -aheadnow;
	m_speed = speed;
	m_glide_pending = true;
	m_choice = 0;
	return true;
}

void track_ride::forget(scene::basic_node const *Node)
{
	if (Node != nullptr && Node == m_place.track)
		leave();
}

void track_ride::draw_hud() const
{
	if (false == active())
		return;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const pad{6.0f};
	auto const height{ImGui::GetTextLineHeight()};

	std::string line{m_place.track->name().empty() ? std::string{"?"} : m_place.track->name()};
	if (false == m_km.empty())
		line += "   km " + m_km;
	line += format("   %.0f km/h   %s %.1f m", std::abs(m_speed) * 3.6, STR_C("height"), m_height);
	std::string ahead;
	if (m_switch != nullptr)
		ahead = format(STR_C("Switch in %.0f m: %s   (Z changes)"), m_switch_distance, turn_name(*m_switch, track_ride_walk::switch_path(*m_switch, m_choice)));
	// the keys only standing, out of the way while driving
	std::string keys;
	if (std::abs(m_speed) < 0.5)
		keys = STR("W/S drive   A/D next track   Q/E height   Esc leave");

	// over the bottom of the view, above the bar of the keys of the tools; the rows keep their places as the others come and go
	auto const centre{0.5f * (editor_ui::view_min().x + editor_ui::view_max().x)};
	auto y{editor_ui::view_max().y - 12.0f - height * 2.5f - 3.0f * (height + pad * 1.5f)};
	auto const row = [&](std::string const &Text, ImU32 const Background, ImU32 const Foreground) {
		if (false == Text.empty())
		{
			auto const size{ImGui::CalcTextSize(Text.c_str())};
			ImVec2 const from{centre - 0.5f * size.x - pad, y - pad * 0.5f};
			ImVec2 const to{centre + 0.5f * size.x + pad, y + size.y + pad * 0.5f};
			drawlist->AddRectFilled(from, to, Background, 4.0f);
			drawlist->AddText(ImVec2(from.x + pad, y), Foreground, Text.c_str());
		}
		y += height + pad * 1.5f;
	};
	row(line, IM_COL32(20, 60, 110, 215), IM_COL32(255, 255, 255, 255));
	row(ahead, m_choice != 0 ? IM_COL32(150, 100, 10, 225) : IM_COL32(0, 0, 0, 170), IM_COL32(255, 255, 255, 240));
	row(keys, IM_COL32(0, 0, 0, 120), IM_COL32(220, 220, 220, 210));
}

editor_selftest_result track_ride::selftest(std::string const &Command, std::istringstream &Words)
{
	auto const report = [this](char const *What) {
		if (false == active())
		{
			WriteLog(format("SELFTEST %s: not riding", What));
			return;
		}
		auto const at{rail_point()};
		WriteLog(format("SELFTEST %s: %s path %d s %.3f direction %d speed %.2f choice %d at %.3f %.3f %.3f", What, m_place.track->name().c_str(), m_place.path, m_place.s, m_direction, m_speed, m_choice, at.x, at.y, at.z));
	};
	if (Command == "ride")
	{
		// ride <track> [path] [s]: onto the track at the distance, looking towards the growing distance
		std::string name;
		int path{0};
		double s{0.0};
		Words >> name >> path >> s;
		auto *track{simulation::Paths.find(name)};
		track_ride_walk::place place{track, track != nullptr && track->eType == tt_Switch ? path : -1, 0.0};
		auto const *segment{track_ride_walk::segment(place)};
		if (segment == nullptr || false == enter(track, path, glm::dvec3{0.0}, glm::dvec3{0.0, 0.0, 1.0}, true))
		{
			WriteLog("SELFTEST ride " + name + ": no such track");
			return editor_selftest_result::handled;
		}
		m_place.s = std::clamp(s, 0.0, segment->GetLength());
		m_direction = 1;
		report("ride");
		return editor_selftest_result::handled;
	}
	if (Command == "ridewalk")
	{
		// ridewalk <metres> [choice]: the place moved ahead by the distance (negative: back), the switches and tracks logged
		// before and after, they mustn't change
		double metres{0.0};
		int choice{0};
		Words >> metres >> choice;
		if (false == active())
		{
			report("ridewalk");
			return editor_selftest_result::handled;
		}
		auto const states = [] {
			std::string text;
			for (auto *track : simulation::Paths.sequence())
				if (track != nullptr && track->eType == tt_Switch && track->SwitchExtension != nullptr)
					text += std::to_string(track->SwitchExtension->CurrentIndex);
			return text;
		};
		auto const before{states()};
		int flips{0};
		bool facing{false};
		auto const halted{track_ride_walk::walk(m_place, m_direction * metres, choice, flips, facing)};
		if (flips % 2 != 0)
			m_direction = -m_direction;
		WriteLog(format("SELFTEST ridewalk %.3f choice %d: flips %d facing %d %s, switches %s", metres, choice, flips, facing ? 1 : 0, halted == track_ride_walk::halt::end ? "halted at an end" : "on the way", before == states() ? "kept" : "THROWN"));
		report("ridewalk");
		return editor_selftest_result::handled;
	}
	if (Command == "ridejump")
	{
		// ridejump <-1|1>: to the track alongside on the left or the right of the way ahead
		int side{1};
		Words >> side;
		glm::dvec3 point, forward;
		if (false == track_ride_walk::frame(m_place, point, forward))
		{
			report("ridejump");
			return editor_selftest_result::handled;
		}
		auto const jumped{jump_side(side, forward * static_cast<double>(m_direction))};
		WriteLog(format("SELFTEST ridejump %d: %s", side, jumped ? "jumped" : "no track alongside"));
		report("ridejump");
		return editor_selftest_result::handled;
	}
	if (Command == "rideahead")
	{
		// rideahead: the switch ahead entered from its blades and the path the camera would take there
		double distance{0.0};
		auto *ahead{active() ? track_ride_walk::facing_switch_ahead(m_place, m_direction, switch_reach, distance) : nullptr};
		if (ahead == nullptr)
			WriteLog("SELFTEST rideahead: no switch ahead");
		else
			WriteLog(format("SELFTEST rideahead: %s in %.2f m, straight path %d", ahead->name().c_str(), distance, track_ride_walk::switch_path(*ahead, 0)));
		return editor_selftest_result::handled;
	}
	if (Command == "rideprobe")
	{
		report("rideprobe");
		return editor_selftest_result::handled;
	}
	if (Command == "rideleave")
	{
		leave();
		WriteLog("SELFTEST rideleave");
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}
