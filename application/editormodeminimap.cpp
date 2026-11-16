/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodeminimap.h"
#include "application/editormoderide.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorKilometrage.hpp"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"
#include "vehicle/Camera.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "world/Segment.h"
#include "world/Track.h"
#include "imgui/imgui.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <unordered_set>
#include <sstream>

namespace
{

constexpr char const *stop_prefix{"PassengerStopPoint:"};

// the name of the station out of the text of a stop point, cut at '#' which only makes the text unique
std::string stop_name(std::string const &Text)
{
	auto name{Text.substr(std::string{stop_prefix}.size())};
	if (auto const hash{name.find('#')}; hash != std::string::npos)
		name.erase(hash);
	while (false == name.empty() && std::isspace(static_cast<unsigned char>(name.back())))
		name.pop_back();
	return name;
}

// lower case without the polish letters, of a text in windows-1250
std::string folded(std::string Text)
{
	win1250_to_ascii(Text);
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return Text;
}

// the way the camera looks in the plan
glm::dvec3 camera_look()
{
	return {-std::sin(Global.pCamera.Angle.y), 0.0, -std::cos(Global.pCamera.Angle.y)};
}

// the path of the track nearest to the point and the point of it, for a track and a point given
int path_at(TTrack const &Track, glm::dvec3 const &Point, glm::dvec3 &On)
{
	return kilometrage::path_near(Track, Point, On);
}

} // namespace

bool minimap_tool::stale(std::size_t &Seen) const
{
	auto const now{m_history.history_revision() * 1000003u ^ simulation::Paths.sequence().size() * 7919u ^ simulation::Memory.sequence().size() * 104729u ^ simulation::Events.sequence().size()};
	if (now == Seen)
		return false;
	Seen = now;
	return true;
}

void minimap_tool::gather_lines()
{
	if (false == stale(m_lines_seen))
		return;
	m_lines.clear();
	m_low = glm::dvec2{std::numeric_limits<double>::max()};
	m_high = glm::dvec2{std::numeric_limits<double>::lowest()};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 1) == 0 || track->eType == tt_Cross)
			continue;
		for (int path = 0; path < track->path_count(); ++path)
		{
			track_ride_walk::place const place{track, track->eType == tt_Switch ? path : -1, 0.0};
			auto const *segment{track_ride_walk::segment(place)};
			if (segment == nullptr || segment->GetLength() <= 0.0)
				continue;
			line drawn;
			drawn.track = track;
			drawn.low = glm::dvec2{std::numeric_limits<double>::max()};
			drawn.high = glm::dvec2{std::numeric_limits<double>::lowest()};
			auto const count{std::clamp(static_cast<int>(segment->GetLength() / 10.0), 1, 32)};
			for (int i = 0; i <= count; ++i)
			{
				auto const point{segment->FastGetPoint(static_cast<double>(i) / count)};
				glm::dvec2 const plan{point.x, point.z};
				drawn.points.push_back(plan);
				drawn.low = glm::min(drawn.low, plan);
				drawn.high = glm::max(drawn.high, plan);
			}
			m_low = glm::min(m_low, drawn.low);
			m_high = glm::max(m_high, drawn.high);
			m_lines.push_back(std::move(drawn));
		}
	}
}

void minimap_tool::gather_stations()
{
	if (false == stale(m_stations_seen))
		return;
	std::map<std::string, station> found;
	// the texts are in windows-1250; the ones of the events lost their polish letters as they were read
	auto const add = [&](std::string const &Text, bool const Letters, glm::dvec3 const &Point, TTrack *Track) {
		auto const name{stop_name(Text)};
		if (name.empty())
			return;
		auto const key{folded(name)};
		auto &entry{found[key]};
		entry.key = key;
		// the timetables join the words with underscores
		if (entry.name.empty() || Letters)
		{
			entry.name = win1250_to_utf8(name);
			std::replace(entry.name.begin(), entry.name.end(), '_', ' ');
		}
		if (Track == nullptr)
		{
			int path{0};
			Track = kilometrage::track_near(Point, 15.0, path);
		}
		entry.stops.push_back({Point, Track});
	};
	for (auto *cell : simulation::Memory.sequence())
		if (cell != nullptr && cell->Text().starts_with(stop_prefix))
			add(cell->Text(), true, cell->location(), cell->Track);
	for (auto *event : simulation::Events.sequence())
		if (auto const *values{dynamic_cast<putvalues_event const *>(event)}; values != nullptr && values->input_command() == TCommandType::cm_PassengerStopPoint)
			add(values->input_text(), false, values->input_location(), nullptr);
	std::vector<station> stations;
	for (auto &entry : found)
		stations.push_back(std::move(entry.second));
	// the next stop kept for the stations still there
	for (auto &station : stations)
		for (auto const &old : m_stations)
			if (old.key == station.key)
				station.next = old.next;
	m_stations = std::move(stations);
}

std::vector<minimap_tool::station> const &minimap_tool::stations()
{
	gather_stations();
	return m_stations;
}

void minimap_tool::go_to(station &Station)
{
	if (Station.stops.empty())
		return;
	auto const &stop{Station.stops[Station.next % Station.stops.size()]};
	++Station.next;
	m_marked = Station.key;
	// a map not going with the camera goes over to the stop
	if (false == m_follow)
		m_centre = {stop.point.x, stop.point.z};
	glm::dvec3 on{stop.point};
	auto const path{stop.track != nullptr ? path_at(*stop.track, stop.point, on) : 0};
	if (m_ride.active())
	{
		if (stop.track != nullptr && m_ride.enter(stop.track, path, stop.point, camera_look(), true))
			return;
		m_ride.leave();
	}
	// from behind and above the stop, looking along the track and down to it
	auto ahead{camera_look()};
	if (stop.track != nullptr)
	{
		track_ride_walk::place place{stop.track, stop.track->eType == tt_Switch ? path : -1, 0.0};
		if (auto const *segment{track_ride_walk::segment(place)})
		{
			place.s = track_ride_walk::nearest_distance(*segment, stop.point);
			glm::dvec3 point, forward;
			if (track_ride_walk::frame(place, point, forward))
			{
				on = point;
				ahead = glm::dot(glm::dvec2{forward.x, forward.z}, glm::dvec2{ahead.x, ahead.z}) >= 0.0 ? forward : -forward;
			}
		}
	}
	auto const plan{std::hypot(ahead.x, ahead.z)};
	glm::dvec3 const look{glm::normalize(glm::dvec3{ahead.x / std::max(plan, 1e-9) * 0.9, -0.42, ahead.z / std::max(plan, 1e-9) * 0.9})};
	m_view.fly_to(on, look, 45.0, 0.8);
}

void minimap_tool::go_to_point(glm::dvec3 const &Point, double const Reach)
{
	int path{0};
	auto *track{kilometrage::track_near(Point, Reach, path)};
	glm::dvec3 on{Point};
	if (track != nullptr)
		kilometrage::path_near(*track, Point, on);
	if (m_ride.active())
	{
		if (track != nullptr && m_ride.enter(track, path, on, camera_look(), false))
			return;
		m_ride.leave();
	}
	auto const ahead{camera_look()};
	glm::dvec3 const look{glm::normalize(glm::dvec3{ahead.x * 0.8, -0.6, ahead.z * 0.8})};
	if (track != nullptr)
		m_view.fly_to(on, look, 60.0, 0.6);
	else
		m_view.fly_to(glm::dvec3{Point.x, m_view.camera_position().y, Point.z}, look, 0.0, 0.6);
}

void minimap_tool::forget(scene::basic_node const *Node)
{
	if (Node == nullptr)
		return;
	for (auto &station : m_stations)
		for (auto &stop : station.stops)
			if (stop.track == Node)
				stop.track = nullptr;
	for (auto &drawn : m_lines)
		if (drawn.track == Node)
			drawn.track = nullptr;
}

void minimap_tool::render_search()
{
	gather_stations();
	ImGui::SetNextItemWidth(-1.0f);
	// enter goes to the first one found
	auto const entered{ImGui::InputTextWithHint("##stationsearch", STR_C("Find a station"), m_search, sizeof(m_search), ImGuiInputTextFlags_EnterReturnsTrue)};
	if (m_stations.empty())
	{
		ImGui::TextDisabled("%s", STR_C("No W4 in the scenery"));
		return;
	}
	auto const wanted{folded(utf8_to_win1250(m_search))};
	std::vector<int> shown;
	for (int i = 0; i < static_cast<int>(m_stations.size()); ++i)
		if (wanted.empty() || m_stations[i].key.find(wanted) != std::string::npos)
			shown.push_back(i);
	auto const rows{std::min<int>(static_cast<int>(shown.size()), 6)};
	if (shown.empty())
	{
		ImGui::TextDisabled("%s", STR_C("Nothing found"));
		return;
	}
	if (entered)
	{
		// the field lets the keys go, they drive the ride from there; a click on the station goes on to its next stop
		go_to(m_stations[shown.front()]);
	}
	if (ImGui::BeginChild("##stations", ImVec2(0.0f, (rows + 0.3f) * ImGui::GetTextLineHeightWithSpacing()), ImGuiChildFlags_Borders))
	{
		for (auto const index : shown)
		{
			auto &station{m_stations[index]};
			auto const label{station.stops.size() > 1 ? format("%s  (%d)##%d", station.name.c_str(), static_cast<int>(station.stops.size()), index) : format("%s##%d", station.name.c_str(), index)};
			if (ImGui::Selectable(label.c_str(), station.key == m_marked))
				go_to(station);
			if (ImGui::IsItemHovered() && station.stops.size() > 1)
				ImGui::SetTooltip("%s", STR_C("Click again for the next W4"));
		}
	}
	ImGui::EndChild();
}

void minimap_tool::render_map(float const Height)
{
	gather_lines();
	auto const width{ImGui::GetContentRegionAvail().x};
	ImVec2 const size{std::max(width, 50.0f), std::max(Height, 80.0f)};
	auto const origin{ImGui::GetCursorScreenPos()};
	auto const camera{m_view.camera_position()};
	if (false == m_fitted && false == m_lines.empty())
	{
		m_fitted = true;
		m_centre = {camera.x, camera.z};
		m_scale = 2.0;
	}
	if (m_follow)
		m_centre = {camera.x, camera.z};

	// a window of its own: its own list of what's drawn, which mustn't go over 65536 vertices on the renderers without offsets
	if (false == ImGui::BeginChild("##minimapcanvas", size, ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
	{
		ImGui::EndChild();
		return;
	}
	ImGui::InvisibleButton("##minimap", size);
	auto const hovered{ImGui::IsItemHovered()};
	auto const &io{ImGui::GetIO()};
	glm::vec2 const mouse{io.MousePos.x, io.MousePos.y};
	glm::dvec2 const middle{origin.x + size.x * 0.5, origin.y + size.y * 0.5};
	// x to the right, z down: as the top view looking north along -z
	auto const to_world = [&](glm::vec2 const &Screen) { return glm::dvec2{m_centre.x + (Screen.x - middle.x) * m_scale, m_centre.y + (Screen.y - middle.y) * m_scale}; };
	auto const to_screen = [&](glm::dvec2 const &World) { return ImVec2(static_cast<float>(middle.x + (World.x - m_centre.x) / m_scale), static_cast<float>(middle.y + (World.y - m_centre.y) / m_scale)); };
	if (hovered && io.MouseWheel != 0.0f)
	{
		auto const under{to_world(mouse)};
		m_scale = std::clamp(m_scale * std::pow(0.8, static_cast<double>(io.MouseWheel)), 0.05, 500.0);
		if (false == m_follow)
			m_centre = {under.x - (mouse.x - middle.x) * m_scale, under.y - (mouse.y - middle.y) * m_scale};
	}
	if (ImGui::IsItemActivated())
	{
		m_pressed = mouse;
		m_dragging = false;
	}
	if (ImGui::IsItemActive())
	{
		if (glm::length(mouse - m_pressed) > 4.0f)
			m_dragging = true;
		if (m_dragging && (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f))
		{
			m_follow = false;
			m_centre.x -= io.MouseDelta.x * m_scale;
			m_centre.y -= io.MouseDelta.y * m_scale;
		}
	}
	if (ImGui::IsItemDeactivated() && false == m_dragging)
	{
		auto const world{to_world(mouse)};
		go_to_point(glm::dvec3{world.x, camera.y, world.y}, std::max(20.0, 12.0 * m_scale));
	}

	auto *drawlist{ImGui::GetWindowDrawList()};
	ImVec2 const corner{origin.x + size.x, origin.y + size.y};
	drawlist->PushClipRect(origin, corner, true);
	drawlist->AddRectFilled(origin, corner, IM_COL32(28, 32, 36, 255));
	auto const low{to_world({origin.x, origin.y})};
	auto const high{to_world({corner.x, corner.y})};
	auto const *ridden{m_ride.track()};
	// what's left of the vertices for the tracks, some kept for the stations, the camera and the scale
	auto const budget{drawlist->VtxBuffer.Size + 56000};
	auto full{false};
	std::vector<ImVec2> points;
	std::unordered_set<std::uint64_t> dots;
	for (auto const &drawn : m_lines)
	{
		if (drawn.track == nullptr || drawn.high.x < low.x || drawn.low.x > high.x || drawn.high.y < low.y || drawn.low.y > high.y)
			continue;
		if (drawlist->VtxBuffer.Size > budget)
		{
			full = true;
			break;
		}
		auto const colour{drawn.track == ridden ? IM_COL32(255, 170, 40, 255) : drawn.track->eType == tt_Switch ? IM_COL32(150, 200, 255, 230) : IM_COL32(200, 200, 200, 210)};
		auto const extent{std::max(drawn.high.x - drawn.low.x, drawn.high.y - drawn.low.y) / m_scale}; // px
		// a track a pixel long a dot, one dot a pixel
		if (extent < 1.5)
		{
			auto const at{to_screen(drawn.low)};
			if (dots.insert((static_cast<std::uint64_t>(static_cast<std::uint32_t>(static_cast<std::int32_t>(std::floor(at.x)))) << 32) | static_cast<std::uint32_t>(static_cast<std::int32_t>(std::floor(at.y)))).second)
				drawlist->AddRectFilled(at, ImVec2(at.x + 1.0f, at.y + 1.0f), colour);
			continue;
		}
		auto const width{drawn.track == ridden ? 3.0f : 1.0f};
		// a short one from end to end, the longer ones through their points a few pixels apart
		if (extent < 16.0 && drawn.track != ridden)
		{
			drawlist->AddLine(to_screen(drawn.points.front()), to_screen(drawn.points.back()), colour, width);
			continue;
		}
		points.clear();
		for (auto const &point : drawn.points)
		{
			auto const at{to_screen(point)};
			if (points.empty() || &point == &drawn.points.back() || std::abs(at.x - points.back().x) + std::abs(at.y - points.back().y) > 4.0f)
				points.push_back(at);
		}
		drawlist->AddPolyline(points.data(), static_cast<int>(points.size()), colour, ImDrawFlags_None, width);
	}
	// the stations: a dot at each stop, the name at the first one in sight
	for (int i = 0; i < static_cast<int>(m_stations.size()); ++i)
	{
		auto const &station{m_stations[i]};
		auto const marked{station.key == m_marked};
		bool named{false};
		for (auto const &stop : station.stops)
		{
			auto const at{to_screen({stop.point.x, stop.point.z})};
			if (at.x < origin.x - 50.0f || at.x > corner.x + 50.0f || at.y < origin.y - 20.0f || at.y > corner.y + 20.0f)
				continue;
			// the stops stop short of the limit of the list too, a few hundred vertices kept for the camera and the scale
			if (drawlist->VtxBuffer.Size > 64500)
			{
				full = true;
				break;
			}
			drawlist->AddCircleFilled(at, marked ? 5.0f : 3.5f, marked ? IM_COL32(255, 80, 60, 255) : IM_COL32(255, 220, 60, 255));
			if (false == named && (marked || m_scale < 25.0))
			{
				named = true;
				drawlist->AddText(ImVec2(at.x + 6.0f, at.y - ImGui::GetTextLineHeight() * 0.5f), IM_COL32(255, 235, 150, 255), station.name.c_str());
			}
		}
	}
	// the camera, a triangle the way it looks
	{
		auto const at{to_screen({camera.x, camera.z})};
		auto const look{camera_look()};
		ImVec2 const forward{static_cast<float>(look.x), static_cast<float>(look.z)};
		ImVec2 const side{-forward.y, forward.x};
		auto const point = [&](float const Ahead, float const Across) { return ImVec2(at.x + forward.x * Ahead + side.x * Across, at.y + forward.y * Ahead + side.y * Across); };
		drawlist->AddTriangleFilled(point(11.0f, 0.0f), point(-6.0f, 6.0f), point(-6.0f, -6.0f), IM_COL32(255, 60, 60, 255));
		drawlist->AddTriangle(point(11.0f, 0.0f), point(-6.0f, 6.0f), point(-6.0f, -6.0f), IM_COL32(0, 0, 0, 255), 1.0f);
	}
	// the scale bar
	{
		auto metres{100.0 * m_scale};
		auto const magnitude{std::pow(10.0, std::floor(std::log10(metres)))};
		metres = metres / magnitude >= 5.0 ? 5.0 * magnitude : metres / magnitude >= 2.0 ? 2.0 * magnitude : magnitude;
		auto const pixels{static_cast<float>(metres / m_scale)};
		ImVec2 const from{origin.x + 8.0f, corner.y - 10.0f};
		drawlist->AddLine(from, ImVec2(from.x + pixels, from.y), IM_COL32(255, 255, 255, 220), 2.0f);
		auto const text{metres >= 1000.0 ? format("%g km", metres / 1000.0) : format("%g m", metres)};
		drawlist->AddText(ImVec2(from.x, from.y - ImGui::GetTextLineHeight() - 2.0f), IM_COL32(255, 255, 255, 220), text.c_str());
	}
	if (full)
	{
		auto const *text{STR_C("Zoom in to see all the tracks")};
		drawlist->AddText(ImVec2(origin.x + 8.0f, origin.y + 6.0f), IM_COL32(255, 200, 120, 255), text);
	}
	drawlist->PopClipRect();
	ImGui::EndChild();
}

void minimap_tool::render_windows()
{
	if (false == m_open)
		return;
	// first in the top right corner of the view, over the scene rather than over the panels
	ImVec2 const size{340.0f * Global.ui_scale, 460.0f * Global.ui_scale};
	ImGui::SetNextWindowPos(ImVec2(editor_ui::view_max().x - size.x - 60.0f, editor_ui::view_min().y + 50.0f), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(size, ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowBgAlpha(0.97f);
	if (false == ImGui::Begin((std::string(STR_C("Minimap")) + "###minimap").c_str(), &m_open))
	{
		ImGui::End();
		return;
	}
	render_search();
	ImGui::Checkbox(STR_C("Follow the camera"), &m_follow);
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Whole scenery")) && false == m_lines.empty())
	{
		m_fitted = true;
		m_follow = false;
		m_centre = (m_low + m_high) * 0.5;
		auto const room{ImGui::GetContentRegionAvail()};
		m_scale = std::max((m_high.x - m_low.x) / std::max(room.x, 50.0f), (m_high.y - m_low.y) / std::max(room.y, 80.0f)) * 1.05;
	}
	render_map(ImGui::GetContentRegionAvail().y);
	ImGui::End();
}

editor_selftest_result minimap_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "minimap")
	{
		// minimap <0|1|2>: the window shown or hidden, 2: shown with the whole scenery in it, as its button does
		int open{1};
		Words >> open;
		m_open = open != 0;
		if (open == 2)
		{
			gather_lines();
			m_fitted = true;
			m_follow = false;
			m_centre = (m_low + m_high) * 0.5;
			m_scale = std::max(m_high.x - m_low.x, m_high.y - m_low.y) / 300.0;
		}
		return editor_selftest_result::handled;
	}
	if (Command == "stations")
	{
		// stations: logs the stations the W4 name, with their stops
		gather_stations();
		WriteLog(format("SELFTEST stations: %d", static_cast<int>(m_stations.size())));
		for (auto const &station : m_stations)
			for (auto const &stop : station.stops)
				WriteLog(format("SELFTEST station %s at %.2f %.2f %.2f track %s", station.name.c_str(), stop.point.x, stop.point.y, stop.point.z, stop.track != nullptr ? stop.track->name().c_str() : "-"));
		return editor_selftest_result::handled;
	}
	if (Command == "station")
	{
		// station <text>: the camera to the next stop of the first station whose name holds the text, as a click on the list
		std::string text;
		std::getline(Words >> std::ws, text);
		gather_stations();
		auto const wanted{folded(utf8_to_win1250(text))};
		auto const found{std::find_if(m_stations.begin(), m_stations.end(), [&](station const &Station) { return Station.key.find(wanted) != std::string::npos; })};
		if (found == m_stations.end())
			WriteLog("SELFTEST station " + text + ": none");
		else
		{
			go_to(*found);
			WriteLog("SELFTEST station " + text + ": " + found->name);
		}
		return editor_selftest_result::handled;
	}
	if (Command == "minimapclick")
	{
		// minimapclick <x> <z>: as a click on the map there
		double x{0.0};
		double z{0.0};
		Words >> x >> z;
		go_to_point(glm::dvec3{x, m_view.camera_position().y, z}, 30.0);
		WriteLog(format("SELFTEST minimapclick %.2f %.2f", x, z));
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}
