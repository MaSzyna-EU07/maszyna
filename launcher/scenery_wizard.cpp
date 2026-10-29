/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "launcher/scenery_wizard.h"
#include "application/application.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeoMap.hpp"
#include "editor/editorOrthophoto.hpp"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "rendering/renderer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace
{

bool valid_name(std::string const &Name)
{
	return false == Name.empty() && std::all_of(Name.begin(), Name.end(), [](char const Character) { return (Character >= 'a' && Character <= 'z') || (Character >= '0' && Character <= '9') || Character == '_' || Character == '-'; });
}

} // namespace

ui::scenerywizard_panel::scenerywizard_panel() : ui_panel(STR("New scenery"), false) {}

ui::scenerywizard_panel::~scenerywizard_panel() = default;

void ui::scenerywizard_panel::render_contents()
{
	auto const available{ImGui::GetContentRegionAvail()};
	auto const form{360.f * Global.ui_scale};
	glm::vec2 const map{std::max(200.f, available.x - form - ImGui::GetStyle().ItemSpacing.x), std::max(200.f, available.y)};
	ImGui::BeginChild("##wizardmap", ImVec2(map.x, map.y), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
	if (m_map.render(map - glm::vec2{16.f}))
		pick(m_map.point());
	ImGui::EndChild();
	ImGui::SameLine();
	ImGui::BeginChild("##wizardform", ImVec2(0.f, map.y), true);
	render_form();
	ImGui::EndChild();
}

void ui::scenerywizard_panel::pick(glm::dvec2 const &Point)
{
	m_map.pick(Point);
	auto const degrees{editor_geo::to_degrees(Point)};
	std::snprintf(m_latlon, sizeof(m_latlon), "%.6f, %.6f", degrees.x, degrees.y);
}

void ui::scenerywizard_panel::render_form()
{
	ImGui::TextUnformatted(STR_C("New scenery"));
	ImGui::Separator();
	if (ImGui::RadioButton(STR_C("Realistic"), m_realistic))
		m_realistic = true;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("A real place: the centre is picked on the map, the editor opens with the orthophoto under it, seen from above"));
	ImGui::SameLine();
	if (ImGui::RadioButton(STR_C("Fictional"), false == m_realistic))
		m_realistic = false;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("No place on the map, no orthophoto: an empty plane seen from above"));
	ImGui::Spacing();

	ImGui::SetNextItemWidth(-1.f);
	ImGui::InputTextWithHint("##name", STR_C("file name, e.g. my_line"), m_filename, sizeof(m_filename), ImGuiInputTextFlags_CharsNoBlank);
	std::string name{m_filename};
	std::transform(name.begin(), name.end(), name.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	auto const path{Global.asCurrentSceneryPath + name + ".scn"};
	auto const named{valid_name(name)};
	auto const exists{named && std::filesystem::exists(path)};
	if (name.empty())
		ImGui::TextDisabled("%s", STR_C("scenery/<name>.scn"));
	else if (false == named)
		ImGui::TextColored(ImVec4(1.f, 0.45f, 0.35f, 1.f), "%s", STR_C("Lower case letters, digits, _ and - only"));
	else if (exists)
		ImGui::TextColored(ImVec4(1.f, 0.45f, 0.35f, 1.f), STR_C("%s exists already"), path.c_str());
	else
		ImGui::TextDisabled("%s", path.c_str());

	ImGui::SetNextItemWidth(-1.f);
	ImGui::InputTextWithHint("##title", STR_C("title, written in the file"), m_title, sizeof(m_title));
	ImGui::Spacing();

	auto const placed{m_map.picked() || false == m_realistic};
	auto const ready{named && false == exists && placed};
	if (false == ready)
	{
		ImGui::TextDisabled("%s", placed ? STR_C("Give the file a name") : STR_C("Click the centre on the map"));
		if (m_realistic && ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The wheel zooms, dragging moves the map. A click sets the centre of the scenery, its point 0 0 0"));
	}
	if (ImGui::Button(STR_C("Create and open in the editor"), ImVec2(-1.f, 0.f)) && ready)
		create_scenery();
	if (false == m_status.empty())
		ImGui::TextWrapped("%s", m_status.c_str());
	ImGui::Spacing();

	if (ImGui::TreeNode(STR_C("More settings")))
	{
		ImGui::BeginDisabled(false == m_realistic);
		ImGui::TextUnformatted(STR_C("Centre of the scenery, PUWG 1992"));
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("PUWG 1992: the coordinate system of the maps of Poland, in km"));
		double east{m_map.point().x / 1000.0};
		double north{m_map.point().y / 1000.0};
		ImGui::SetNextItemWidth(130.f);
		auto changed{ImGui::InputDouble(STR_C("E, km##east"), &east, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)};
		ImGui::SetNextItemWidth(130.f);
		changed |= ImGui::InputDouble(STR_C("N, km##north"), &north, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue);
		if (changed)
		{
			pick({east * 1000.0, north * 1000.0});
			m_map.show(m_map.point(), 1000.0);
		}
		ImGui::SetNextItemWidth(-90.f);
		auto const entered{ImGui::InputTextWithHint("##latlon", STR_C("latitude, longitude"), m_latlon, sizeof(m_latlon), ImGuiInputTextFlags_EnterReturnsTrue)};
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Show")) || entered)
		{
			std::string text{m_latlon};
			std::replace(text.begin(), text.end(), ',', ' ');
			std::istringstream words(text);
			double latitude{0.0}, longitude{0.0};
			if (words >> latitude >> longitude && latitude > 45.0 && latitude < 58.0 && longitude > 10.0 && longitude < 28.0)
			{
				pick(editor_geo::to_puwg(latitude, longitude));
				m_map.show(m_map.point());
				m_status.clear();
			}
			else
				m_status = STR("Give the latitude and the longitude in degrees, e.g. 52.2297, 21.0122");
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Degrees, as given by the online maps; the map moves there"));
		if (m_map.picked())
		{
			auto const degrees{editor_geo::to_degrees(m_map.point())};
			ImGui::TextDisabled(STR_C("%.5f N, %.5f E"), degrees.x, degrees.y);
		}
		ImGui::Spacing();
		ImGui::TextWrapped("%s", STR_C("The scenery keeps the centre as its geographic reference (//$g PUWG1992): the orthophoto of the editor lies under it, in place."));
		ImGui::EndDisabled();
		ImGui::TreePop();
	}
	ImGui::Spacing();
	ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
	ImGui::TextWrapped("%s", STR_C("Map: (c) OpenStreetMap contributors, served by geoportal.gov.pl"));
	ImGui::PopStyleColor();
}

bool ui::scenerywizard_panel::create_scenery()
{
	std::string name{m_filename};
	std::transform(name.begin(), name.end(), name.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	auto const file{name + ".scn"};
	std::ofstream output(Global.asCurrentSceneryPath + file, std::ios::binary);
	if (false == output.is_open())
	{
		m_status = STR("The file can't be written");
		return false;
	}
	auto const now{std::time(nullptr)};
	char date[32];
	std::strftime(date, sizeof(date), "%Y-%m-%d", std::localtime(&now));
	char reference[96];
	std::snprintf(reference, sizeof(reference), "//$g PUWG1992 %.3f %.3f", m_map.point().x / 1000.0, m_map.point().y / 1000.0);
	if (m_realistic)
	{
		// the orthophoto on; it's shown as the settings of the editor have it, its origin is the one of //$g
		output << reference << "\r\n";
		output << "//$o " << editor_orthophoto::mark(editor_orthophoto::config{}, true, false) << "\r\n";
	}
	else
		std::snprintf(reference, sizeof(reference), "fictional");
	if (m_title[0] != '\0')
		output << "// " << m_title << "\r\n";
	output << "// created in the editor " << date << "\r\n\r\n";
	output << "atmo 0.423 0.702 1.0 25000 25000 0.70 0.80 0.9 0 endatmo\r\n";
	output << "time 12:00 5:00 20:00 endtime\r\n\r\n";
	output << "FirstInit\r\n";
	output.close();
	WriteLog("Scenery wizard: created " + file + " at " + reference);

	Global.SceneryFile = file;
	Global.local_start_vehicle = "ghostview";
	Global.editor_session = true;
	Application.pop_mode();
	Application.push_mode(eu07_application::mode::scenarioloader);
	return true;
}
