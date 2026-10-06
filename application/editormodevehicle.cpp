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
#include "application/application.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "launcher/textures_scanner.h"

#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "vehicle/DynObj.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <cctype>
#include <optional>
#include <limits>

namespace
{

using geometry::bezier;

std::size_t const kVehicleList{300};
double const kHalfVehicle{8.0}; // m, the vehicle is placed about centred on the point

std::string lower(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return Text;
}

std::string skin_label(ui::vehicle_desc const &Vehicle, ui::skin_set const &Skin)
{
	auto label{Skin.meta != nullptr && false == Skin.meta->name.empty() ? Skin.meta->name : Skin.skin};
	return label + "  (" + Vehicle.path.generic_string() + ")";
}

int path_under(TTrack const &Track, glm::dvec3 const &Point)
{
	int result{0};
	auto best{std::numeric_limits<double>::max()};
	for (int i = 0; i < static_cast<int>(Track.m_paths.size()); ++i)
	{
		bezier const curve{Track.m_paths[i]};
		for (int k = 0; k <= 64; ++k)
		{
			auto const distance{geometry::plan_distance(curve.point(k / 64.0), Point)};
			if (distance < best)
			{
				best = distance;
				result = i;
			}
		}
	}
	return result;
}

double nearest_on(TTrack const &Track, int const Path, glm::dvec3 const &Point)
{
	bezier const curve{Track.m_paths[Path]};
	auto parameter{0.0};
	auto best{std::numeric_limits<double>::max()};
	for (int k = 0; k <= 256; ++k)
	{
		auto const distance{geometry::plan_distance(curve.point(k / 256.0), Point)};
		if (distance < best)
		{
			best = distance;
			parameter = k / 256.0;
		}
	}
	return parameter;
}

glm::dvec2 heading_at(TTrack const &Track, int const Path, glm::dvec3 const &Point)
{
	auto const tangent{bezier{Track.m_paths[Path]}.first(nearest_on(Track, Path, Point))};
	glm::dvec2 const planar{tangent.x, tangent.z};
	return glm::length(planar) > 1e-9 ? glm::normalize(planar) : glm::dvec2{0.0, 1.0};
}

// distance along the path of the track from its start to the point nearest to the one given
double distance_along(TTrack const &Track, int const Path, glm::dvec3 const &Point)
{
	bezier const curve{Track.m_paths[Path]};
	auto const parameter{nearest_on(Track, Path, Point)};
	int const count{64};
	double length{0.0}, total{0.0};
	auto previous{curve.point(0.0)};
	for (int k = 1; k <= count; ++k)
	{
		auto const t{static_cast<double>(k) / count};
		auto const next{curve.point(t)};
		auto const step{glm::distance(previous, next)};
		total += step;
		if (t <= parameter)
			length += step;
		else if (t - 1.0 / count < parameter)
			length += step * (parameter - (t - 1.0 / count)) * count;
		previous = next;
	}
	return std::clamp(length * Track.Length() / std::max(total, 1e-6), 0.0, Track.Length());
}

} // namespace

void editor_mode::vehicle_start(TTrack &Track, glm::dvec3 const &Point)
{
	m_vehicle.track = &Track;
	m_vehicle.point = Point;
	m_vehicle.expand = true;
	m_vehicle.status.clear();
	auto const heading{heading_at(Track, Track.eType == tt_Switch ? path_under(Track, Point) : 0, Point)};
	glm::dvec2 const away{Point.x - Global.pCamera.Pos.x, Point.z - Global.pCamera.Pos.z};
	m_vehicle.facing = glm::dot(heading, away) >= 0.0 ? 1 : -1;
}

bool editor_mode::vehicle_place()
{
	auto &tool{m_vehicle};
	if (tool.track == nullptr || tool.track->m_editorremoved)
		return false;
	auto train{tool.consist};
	if (train.empty())
	{
		if (tool.vehicle == nullptr || tool.skin == nullptr)
			return false;
		train.push_back({tool.vehicle, tool.skin, false});
	}
	auto const path{tool.track->eType == tt_Switch ? path_under(*tool.track, tool.point) : 0};
	if (tool.track->eType == tt_Switch && tool.track->GetSwitchState() != path)
		tool.track->Switch(path);
	auto const along{distance_along(*tool.track, path, tool.point)};
	auto const against{tool.facing < 0};
	std::vector<std::string> names;
	std::string nodes;
	std::string head;
	for (std::size_t i = 0; i < train.size(); ++i)
	{
		auto const &item{train[i]};
		auto const stem{item.vehicle->path.stem().generic_string()};
		std::string name;
		for (int index = 1; index < 10000; ++index)
		{
			name = lower("editor_" + stem + "_" + std::to_string(index));
			if (simulation::Vehicles.find(name) == nullptr && std::find(names.begin(), names.end(), name) == names.end())
				break;
		}
		names.push_back(name);
		auto const leads{i == 0};
		if (leads)
			head = name;
		nodes += "node -1 0 " + name + " dynamic " + item.vehicle->path.parent_path().generic_string() + " " + item.skin->skin + " " + stem + " " + (item.turned ? "-1" : "0") + " " + (leads && tool.driver == 0 ? "headdriver" : "nobody") + " 3 0 enddynamic\n";
	}
	auto const offset{against ? along - kHalfVehicle : along + kHalfVehicle};
	auto const vehicles{simulation::State.insert_trainset(head, tool.track, offset, nodes, against)};
	if (vehicles.empty())
	{
		tool.status = STR_C("The vehicle couldn't be placed, the log says why");
		return false;
	}
	tool.placed = head;
	tool.status = format(STR_C("%s and %d vehicle(s) behind it stand on %s. It isn't saved with the scenery"), head.c_str(), static_cast<int>(vehicles.size()) - 1, tool.track->name().empty() ? "(noname)" : tool.track->name().c_str());
	WriteLog("Editor: " + tool.status, logtype::generic);
	return true;
}

void editor_mode::render_vehicle_ui()
{
	auto &tool{m_vehicle};
	auto *selected{selected_track()};
	if (selected != nullptr && selected != tool.track)
		vehicle_start(*selected, bezier{selected->m_paths.front()}.point(0.5));
	if (tool.track == nullptr)
		return;
	if (tool.bank == nullptr)
	{
		tool.bank = std::make_shared<ui::vehicles_bank>();
		tool.bank->scan_textures();
	}
	ImGui::TextDisabled("%s", STR_C("A vehicle to drive, placed where RMB on the track was clicked, or in the middle of the path.\nIt isn't written to the scenery files"));
	ImGui::SetNextItemWidth(-1.0f);
	ImGui::InputTextWithHint("##vehiclesearch", STR_C("search: name, series, skin"), tool.search, sizeof(tool.search));
	ImGui::Checkbox(STR_C("Only the ones with a cab"), &tool.controllable_only);
	std::string const text{lower(tool.search)};
	std::vector<std::pair<std::shared_ptr<ui::vehicle_desc>, std::shared_ptr<ui::skin_set>>> listed;
	std::size_t total{0};
	for (auto const &[path, vehicle] : tool.bank->vehicles)
	{
		if (vehicle == nullptr || (tool.controllable_only && false == vehicle->controllable))
			continue;
		for (auto const &skin : vehicle->matching_skinsets)
		{
			if (skin == nullptr)
				continue;
			if (false == text.empty() && lower(skin_label(*vehicle, *skin)).find(text) == std::string::npos && (skin->meta == nullptr || skin->meta->search_lowered.find(text) == std::string::npos))
				continue;
			++total;
			if (listed.size() < kVehicleList)
				listed.emplace_back(vehicle, skin);
		}
	}
	ImGui::BeginChild("##vehicles", ImVec2(0.0f, ImGui::GetTextLineHeightWithSpacing() * 8.0f + 6.0f), true);
	for (std::size_t i = 0; i < listed.size(); ++i)
	{
		auto const &[vehicle, skin] = listed[i];
		if (ImGui::Selectable(format("%s##vehicle%zu", skin_label(*vehicle, *skin).c_str(), i).c_str(), tool.skin == skin))
		{
			tool.vehicle = vehicle;
			tool.skin = skin;
		}
	}
	ImGui::EndChild();
	if (total > listed.size())
		ImGui::TextDisabled(STR_C("%zu found, the first %zu listed"), total, listed.size());
	if (tool.vehicle != nullptr && tool.skin != nullptr && ImGui::Button(STR_C("Add to the train")))
		tool.consist.push_back({tool.vehicle, tool.skin, false});
	if (false == tool.consist.empty())
	{
		ImGui::SameLine();
		ImGui::TextDisabled(STR_C("Train: %d vehicle(s), the first one leads"), static_cast<int>(tool.consist.size()));
		std::optional<std::size_t> remove;
		std::optional<std::size_t> raise;
		for (std::size_t i = 0; i < tool.consist.size(); ++i)
		{
			auto &item{tool.consist[i]};
			auto const vehicle{item.skin->vehicle.lock()};
			ImGui::PushID(static_cast<int>(i));
			ImGui::Checkbox("##turned", &item.turned);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Turned around in the train"));
			ImGui::SameLine();
			ImGui::Text("%2d. %s", static_cast<int>(i + 1), vehicle != nullptr ? skin_label(*vehicle, *item.skin).c_str() : item.skin->skin.c_str());
			ImGui::SameLine();
			if (i > 0 && ImGui::SmallButton(STR_C("up")))
				raise = i;
			ImGui::SameLine();
			if (ImGui::SmallButton("x"))
				remove = i;
			ImGui::PopID();
		}
		if (raise)
			std::swap(tool.consist[*raise], tool.consist[*raise - 1]);
		if (remove)
			tool.consist.erase(tool.consist.begin() + static_cast<std::ptrdiff_t>(*remove));
		if (ImGui::SmallButton(STR_C("Empty the train")))
			tool.consist.clear();
	}
	if (ImGui::Button(STR_C("Turn the train around")))
		tool.facing = -tool.facing;
	ImGui::SameLine();
	ImGui::TextDisabled("%s", STR_C("the head goes where the arrow at the marker points"));
	ImGui::RadioButton(STR_C("With a driver, to drive"), &tool.driver, 0);
	ImGui::SameLine();
	ImGui::RadioButton(STR_C("Nobody, parked"), &tool.driver, 1);
	if (tool.consist.empty() && (tool.vehicle == nullptr || tool.skin == nullptr))
		ImGui::TextDisabled("%s", STR_C("Pick a vehicle from the list"));
	else
	{
		if (ImGui::Button(STR_C("Place")))
			vehicle_place();
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Place and drive")) && vehicle_place())
		{
			Global.editor_enter_vehicle = tool.placed;
			tool.leave = true;
		}
	}
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void editor_mode::draw_vehicle_marker() const
{
	auto const &tool{m_vehicle};
	if (false == tool.open || tool.track == nullptr)
		return;
	screen_projection const projection;
	ImVec2 screen;
	if (false == projection.project(tool.point, screen))
		return;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	auto const colour{IM_COL32(255, 220, 60, 235)};
	drawlist->AddCircle(screen, 9.0f, colour, 16, 2.5f);
	if (tool.track->m_editorremoved || tool.track->m_paths.empty())
		return;
	auto const heading{heading_at(*tool.track, tool.track->eType == tt_Switch ? path_under(*tool.track, tool.point) : 0, tool.point) * static_cast<double>(tool.facing)};
	glm::dvec3 const tip{tool.point.x + heading.x * 8.0, tool.point.y, tool.point.z + heading.y * 8.0};
	ImVec2 end;
	if (false == projection.project(tip, end))
		return;
	drawlist->AddLine(screen, end, colour, 3.0f);
	auto const dx{end.x - screen.x};
	auto const dy{end.y - screen.y};
	auto const size{std::sqrt(dx * dx + dy * dy)};
	if (size < 1.0f)
		return;
	ImVec2 const unit{dx / size, dy / size};
	ImVec2 const side{-unit.y, unit.x};
	drawlist->AddTriangleFilled(ImVec2(end.x + unit.x * 10.0f, end.y + unit.y * 10.0f), ImVec2(end.x + side.x * 7.0f, end.y + side.y * 7.0f), ImVec2(end.x - side.x * 7.0f, end.y - side.y * 7.0f), colour);
}
