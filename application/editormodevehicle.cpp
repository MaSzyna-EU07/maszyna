/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodevehicle.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
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

std::string skin_name(ui::skin_set const &Skin)
{
	return Skin.meta != nullptr && false == Skin.meta->name.empty() ? Skin.meta->name : Skin.skin;
}

// side view of the vehicle from textures/mini, 0 while there's none; reading one takes a moment, a few of them go in a frame
ImTextureID vehicle_mini(ui::skin_set &Skin, std::set<ui::skin_set const *> &Asked, int &Loads)
{
	if (false == static_cast<bool>(Skin.mini))
		return 0;
	if (Asked.count(&Skin) == 0)
	{
		if (Loads >= 4)
			return 0;
		++Loads;
		Asked.insert(&Skin);
	}
	auto const id{Skin.mini.get()};
	return id == deferred_image::invalid_id ? 0 : static_cast<ImTextureID>(id);
}

// the side view fitted into the box, kept in proportion and standing on its bottom; mirrored for a vehicle turned around
void draw_mini(ImDrawList &Draw, ImTextureID const Image, glm::ivec2 const Size, ImVec2 const Low, ImVec2 const High, bool const Turned)
{
	if (Image == 0 || Size.x <= 0 || Size.y <= 0)
	{
		Draw.AddRect(Low, High, ImGui::GetColorU32(ImGuiCol_Border), 3.0f);
		return;
	}
	auto const scale{std::min((High.x - Low.x) / Size.x, (High.y - Low.y) / Size.y)};
	auto const width{std::round(Size.x * scale)};
	auto const height{std::round(Size.y * scale)};
	ImVec2 const low{std::round(Low.x + (High.x - Low.x - width) * 0.5f), High.y - height};
	Draw.AddImage(Image, low, ImVec2(low.x + width, High.y), ImVec2(Turned ? 1.0f : 0.0f, 1.0f), ImVec2(Turned ? 0.0f : 1.0f, 0.0f));
}

std::string skin_label(ui::vehicle_desc const &Vehicle, ui::skin_set const &Skin)
{
	return skin_name(Skin) + "  (" + Vehicle.path.generic_string() + ")";
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

void vehicle_tool::start(TTrack &Track, glm::dvec3 const &Point)
{
	m_vehicle.track = &Track;
	m_vehicle.point = Point;
	m_tab.expand = true;
	m_vehicle.status.clear();
	auto const heading{heading_at(Track, Track.eType == tt_Switch ? path_under(Track, Point) : 0, Point)};
	glm::dvec2 const away{Point.x - Global.pCamera.Pos.x, Point.z - Global.pCamera.Pos.z};
	m_vehicle.facing = glm::dot(heading, away) >= 0.0 ? 1 : -1;
}

bool vehicle_tool::place()
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
	m_services.first_step_done(editor_first_step::drive);
	auto const trackname{tool.track->name().empty() ? std::string{STR("(no name)")} : tool.track->name()};
	auto const behind{static_cast<int>(vehicles.size()) - 1};
	tool.status = behind > 0 ? format(STR_P("%s and %d vehicle behind it stand on %s", "%s and %d vehicles behind it stand on %s", behind), head.c_str(), behind, trackname.c_str()) : format(STR_C("%s stands on %s"), head.c_str(), trackname.c_str());
	if (tool.write && tool.track->name().empty())
		tool.status += ". " + std::string{STR_C("The track has no name, the trainset can't be written to the scenery")};
	else if (tool.write)
	{
		std::string timetable{tool.timetable};
		timetable.erase(std::remove_if(timetable.begin(), timetable.end(), [](unsigned char const Character) { return std::isspace(Character) != 0; }), timetable.end());
		vehicle_state::trainset entry{head, tool.track->name(), {}};
		entry.lines.push_back("trainset " + (timetable.empty() ? std::string{"none"} : timetable) + " " + tool.track->name() + " " + to_string(offset, 2) + " " + to_string(tool.velocity, 0));
		if (against)
			entry.lines.push_back("reversed");
		std::istringstream lines{nodes};
		for (std::string line; std::getline(lines, line);)
			if (false == line.empty())
				entry.lines.push_back(line);
		entry.lines.push_back("endtrainset");
		tool.trainsets.push_back(std::move(entry));
		tool.status += ". " + std::string{STR_C("Written to the scenery as a trainset on save (Ctrl+S)")};
	}
	else
		tool.status += ". " + std::string{STR_C("Only to drive, it isn't written to the scenery")};
	WriteLog("Editor: " + tool.status, logtype::generic);
	return true;
}

void vehicle_tool::render_options()
{
	auto &tool{m_vehicle};
	auto *selected{m_services.selected_track()};
	auto const &hover{m_services.hovered_track()};
	if (selected != nullptr && selected == tool.track && hover.track == selected && ImGui::IsMouseClicked(0) && false == ImGui::GetIO().WantCaptureMouse)
		start(*selected, hover.point);
	else if (selected != nullptr && selected != tool.track)
		start(*selected, hover.track == selected ? hover.point : bezier{selected->m_paths.front()}.point(0.5));
	if (tool.track == nullptr)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", STR_C("LMB on a track: where the vehicle stands"));
		return;
	}
	if (tool.bank == nullptr)
	{
		tool.bank = std::make_shared<ui::vehicles_bank>();
		tool.bank->scan_textures();
	}
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
	int loads{0};
	auto const line{ImGui::GetTextLineHeight()};
	auto const box{ImVec2(std::round(ImGui::GetFontSize() * 7.0f), std::round(line * 2.0f))};
	ImGui::BeginChild("##vehicles", ImVec2(0.0f, (box.y + ImGui::GetStyle().ItemSpacing.y) * 5.5f + 6.0f), ImGuiChildFlags_Borders);
	for (std::size_t i = 0; i < listed.size(); ++i)
	{
		auto const &[vehicle, skin] = listed[i];
		ImGui::PushID(static_cast<int>(i));
		auto const low{ImGui::GetCursorScreenPos()};
		if (ImGui::Selectable("##vehicle", tool.skin == skin, ImGuiSelectableFlags_AllowDoubleClick, ImVec2(0.0f, box.y)))
		{
			tool.vehicle = vehicle;
			tool.skin = skin;
			if (ImGui::IsMouseDoubleClicked(0))
				tool.consist.push_back({vehicle, skin, false});
		}
		if (ImGui::IsItemVisible())
		{
			auto &draw{*ImGui::GetWindowDrawList()};
			draw_mini(draw, vehicle_mini(*skin, tool.minis, loads), skin->mini.size(), ImVec2(low.x + 2.0f, low.y + 1.0f), ImVec2(low.x + box.x, low.y + box.y - 1.0f), false);
			auto const left{low.x + box.x + ImGui::GetStyle().ItemSpacing.x};
			auto const right{ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x};
			draw.PushClipRect(ImVec2(left, low.y), ImVec2(right, low.y + box.y), true);
			draw.AddText(ImVec2(left, low.y), ImGui::GetColorU32(ImGuiCol_Text), skin_name(*skin).c_str());
			draw.AddText(ImVec2(left, low.y + line), ImGui::GetColorU32(ImGuiCol_TextDisabled), vehicle->path.generic_string().c_str());
			draw.PopClipRect();
		}
		ImGui::PopID();
	}
	ImGui::EndChild();
	if (total > listed.size())
		ImGui::TextDisabled(STR_C("%zu found, the first %zu listed"), total, listed.size());
	if (tool.vehicle != nullptr && tool.skin != nullptr)
	{
		if (ImGui::Button(format(STR_C("Add %s to the train"), skin_name(*tool.skin).c_str()).c_str(), ImVec2(-1.0f, 0.0f)))
			tool.consist.push_back({tool.vehicle, tool.skin, false});
		item_tooltip("A double click on the list adds it too");
	}

	// the train as a picture, the head on the left; a click on a vehicle shows its buttons
	ImGui::SeparatorText(STR_C("Train"));
	if (tool.consist.empty())
		ImGui::TextDisabled("%s", STR_C("No vehicles yet: pick one from the list and add it"));
	else
	{
		auto const height{std::round(line * 1.8f)};
		auto const width_of = [&](ui::skin_set const &Skin) {
			auto const size{Skin.mini.size()};
			return size.y > 0 ? std::round(height * size.x / size.y) : height * 3.0f;
		};
		float total_width{0.0f};
		for (auto const &item : tool.consist)
			total_width += width_of(*item.skin) + 2.0f;
		auto const scrolled{total_width > ImGui::GetContentRegionAvail().x};
		ImGui::BeginChild("##train", ImVec2(0.0f, height + 6.0f + (scrolled ? ImGui::GetStyle().ScrollbarSize : 0.0f)), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);
		auto &draw{*ImGui::GetWindowDrawList()};
		for (std::size_t i = 0; i < tool.consist.size(); ++i)
		{
			auto &item{tool.consist[i]};
			auto const width{width_of(*item.skin)};
			if (i > 0)
				ImGui::SameLine(0.0f, 2.0f);
			ImGui::PushID(static_cast<int>(i));
			auto const low{ImGui::GetCursorScreenPos()};
			if (ImGui::InvisibleButton("##car", ImVec2(width, height + 4.0f)))
				tool.picked = tool.picked == static_cast<int>(i) ? -1 : static_cast<int>(i);
			if (ImGui::IsItemHovered())
			{
				auto const vehicle{item.skin->vehicle.lock()};
				ImGui::SetTooltip("%d. %s", static_cast<int>(i + 1), vehicle != nullptr ? skin_label(*vehicle, *item.skin).c_str() : item.skin->skin.c_str());
			}
			if (tool.picked == static_cast<int>(i) || ImGui::IsItemHovered())
				draw.AddRectFilled(low, ImVec2(low.x + width, low.y + height + 4.0f), tool.picked == static_cast<int>(i) ? family_colour(0, 0.45f) : ImGui::GetColorU32(ImGuiCol_FrameBgHovered), 3.0f);
			draw_mini(draw, vehicle_mini(*item.skin, tool.minis, loads), item.skin->mini.size(), ImVec2(low.x, low.y + 2.0f), ImVec2(low.x + width, low.y + height + 2.0f), item.turned);
			ImGui::PopID();
		}
		ImGui::EndChild();
		if (tool.picked >= static_cast<int>(tool.consist.size()))
			tool.picked = -1;
		if (tool.picked >= 0)
		{
			auto const i{static_cast<std::size_t>(tool.picked)};
			auto &item{tool.consist[i]};
			ImGui::Text("%d. %s", tool.picked + 1, skin_name(*item.skin).c_str());
			auto const button = [](char const *Label, bool const Enabled, char const *Tooltip) {
				ImGui::BeginDisabled(false == Enabled);
				auto const pressed{ImGui::Button(STR_C(Label))};
				ImGui::EndDisabled();
				if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
					ImGui::SetTooltip("%s", STR_C(Tooltip));
				ImGui::SameLine();
				return pressed;
			};
			auto const forward{button("Forward", i > 0, "Moves it one place towards the head")};
			auto const back{button("Back", i + 1 < tool.consist.size(), "Moves it one place towards the end")};
			auto const turn{button("Turn around", true, "Turned around in the train")};
			auto const remove{button("Remove", true, "Takes it out of the train")};
			ImGui::NewLine();
			if (forward)
			{
				std::swap(tool.consist[i], tool.consist[i - 1]);
				--tool.picked;
			}
			else if (back)
			{
				std::swap(tool.consist[i], tool.consist[i + 1]);
				++tool.picked;
			}
			else if (turn)
				item.turned = !item.turned;
			else if (remove)
			{
				tool.consist.erase(tool.consist.begin() + static_cast<std::ptrdiff_t>(i));
				tool.picked = -1;
			}
		}
		else
			ImGui::TextDisabled(STR_P("%d vehicle, the head on the left: click it to move it, turn it or remove it", "%d vehicles, the head on the left: click one to move it, turn it or remove it", static_cast<long long>(tool.consist.size())), static_cast<int>(tool.consist.size()));
		if (ImGui::SmallButton(STR_C("Empty the train")))
		{
			tool.consist.clear();
			tool.picked = -1;
		}
	}

	ImGui::SeparatorText(STR_C("On the track"));
	if (ImGui::Button(STR_C("Turn the train around")))
		tool.facing = -tool.facing;
	item_tooltip("The head goes where the arrow at the marker points");
	segmented("driver", tool.driver,
	          {{0, STR_C("With a driver, to drive"), "The train can be driven or run by the AI at once"}, {1, STR_C("Nobody, parked"), "The vehicles stand braked, without a crew"}});
	toggle_chips("vehiclewrite", {{STR_C("Write to the scenery as a trainset"), &tool.write, "On: the trainset is written to the scenery on save (Ctrl+S). Off: only to drive now, gone when the scenery is loaded again"}});
	if (tool.write && ImGui::TreeNode(STR_C("More settings")))
	{
		if (begin_fields("##vehiclefields", 0.45f))
		{
			field_row("Timetable##trainset");
			ImGui::InputText("##vehicletimetable", tool.timetable, sizeof(tool.timetable));
			item_tooltip("The timetable the train runs by; none: no timetable, it waits for orders");
			field_row("Speed at the start (km/h)");
			ImGui::DragScalar("##vehiclevelocity", ImGuiDataType_Double, &tool.velocity, 1.0f, nullptr, nullptr, "%.0f");
			tool.velocity = std::clamp(tool.velocity, 0.0, 400.0);
			ImGui::EndTable();
		}
		ImGui::TreePop();
	}
	if (false == tool.consist.empty() || (tool.vehicle != nullptr && tool.skin != nullptr))
	{
		auto const half{std::floor((ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f)};
		auto const tall{std::round(ImGui::GetFrameHeight() * 1.3f)};
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.55f, 0.25f, 1.0f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.65f, 0.3f, 1.0f));
		if (ImGui::Button(STR_C("Place"), ImVec2(half, tall)))
			place();
		item_tooltip("A trainset on the track, placed where the track was clicked, or in the middle of the path");
		ImGui::PopStyleColor(2);
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Place and drive"), ImVec2(-1.0f, tall)) && place())
		{
			Global.editor_enter_vehicle = tool.placed;
			tool.leave = true;
		}
	}
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
	if (tool.trainsets.empty())
		return;
	ImGui::Separator();
	ImGui::TextDisabled(STR_P("%d trainset to write on save:", "%d trainsets to write on save:", static_cast<long long>(tool.trainsets.size())), static_cast<int>(tool.trainsets.size()));
	std::optional<std::size_t> drop;
	for (std::size_t i = 0; i < tool.trainsets.size(); ++i)
	{
		ImGui::PushID(static_cast<int>(i));
		if (ImGui::SmallButton("x"))
			drop = i;
		item_tooltip("Not written to the scenery; the vehicles stay until the scenery is loaded again");
		ImGui::SameLine();
		ImGui::Text("%s  (%s)", tool.trainsets[i].head.c_str(), tool.trainsets[i].track.c_str());
		ImGui::PopID();
	}
	if (drop)
		tool.trainsets.erase(tool.trainsets.begin() + static_cast<std::ptrdiff_t>(*drop));
}

void vehicle_tool::draw_overlay() const
{
	auto const &tool{m_vehicle};
	if (false == m_tab.open || tool.track == nullptr)
		return;
	screen_projection const projection;
	ImVec2 screen;
	if (false == projection.project(tool.point, screen))
		return;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
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

bool vehicle_tool::take_leave()
{
	return std::exchange(m_vehicle.leave, false);
}

std::size_t vehicle_tool::trainset_lines(std::vector<std::string> &Lines)
{
	std::erase_if(m_vehicle.trainsets, [](auto const &Trainset) {
		auto const *track{simulation::Paths.find(Trainset.track)};
		return track == nullptr || track->m_editorremoved;
	});
	for (auto const &trainset : m_vehicle.trainsets)
		Lines.insert(Lines.end(), trainset.lines.begin(), trainset.lines.end());
	return m_vehicle.trainsets.size();
}

void vehicle_tool::trainsets_written()
{
	m_vehicle.status = format(STR_C("%d trainset(s) written to the scenery"), static_cast<int>(m_vehicle.trainsets.size()));
	m_vehicle.trainsets.clear();
}

editor_selftest_result vehicle_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "trainset")
	{
		std::string name;
		std::string vehicle;
		std::string skin;
		int facing{1};
		Words >> name >> vehicle >> skin >> facing >> m_vehicle.velocity;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr)
			return editor_selftest_result::handled;
		auto desc{std::make_shared<ui::vehicle_desc>()};
		desc->path = vehicle;
		auto set{std::make_shared<ui::skin_set>()};
		set->skin = skin;
		m_vehicle.consist.clear();
		m_vehicle.vehicle = desc;
		m_vehicle.skin = set;
		start(*track, geometry::bezier{track->m_paths.front()}.point(0.5));
		m_vehicle.facing = facing;
		auto const placed{place()};
		WriteLog(format("SELFTEST trainset %s placed %d pending %d: %s", name.c_str(), placed ? 1 : 0, static_cast<int>(m_vehicle.trainsets.size()), m_vehicle.status.c_str()));
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}
