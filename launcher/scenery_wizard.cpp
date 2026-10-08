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
#include "editor/editorOrthophoto.hpp"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "stb/stb_image.h"
#include <glad/glad.h>

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

double constexpr kSemiMajor{6378137.0};
double constexpr kFlattening{1.0 / 298.257222101};
double constexpr kScale{0.9993};
double constexpr kFalseEasting{500000.0};
double constexpr kFalseNorthing{-5300000.0};
double constexpr kCentralMeridian{19.0};
glm::dvec2 const kPolandLow{140000.0, 120000.0};
glm::dvec2 const kPolandHigh{890000.0, 800000.0};
std::string const kMapService{"https://mapy.geoportal.gov.pl/wss/ext/OSM/BaseMap/service"};

double seconds()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

glm::dvec2 to_puwg(double const Latitude, double const Longitude)
{
	auto const e2{2.0 * kFlattening - kFlattening * kFlattening};
	auto const ep2{e2 / (1.0 - e2)};
	auto const phi{glm::radians(Latitude)};
	auto const sine{std::sin(phi)};
	auto const cosine{std::cos(phi)};
	auto const n{kSemiMajor / std::sqrt(1.0 - e2 * sine * sine)};
	auto const t{std::tan(phi) * std::tan(phi)};
	auto const c{ep2 * cosine * cosine};
	auto const a{glm::radians(Longitude - kCentralMeridian) * cosine};
	auto const m{kSemiMajor * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * phi - (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(2.0 * phi) +
	                          (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * phi) - (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * phi))};
	auto const x{kScale * n * (a + (1.0 - t + c) * std::pow(a, 3) / 6.0 + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * std::pow(a, 5) / 120.0)};
	auto const y{kScale * (m + n * std::tan(phi) * (a * a / 2.0 + (5.0 - t + 9.0 * c + 4.0 * c * c) * std::pow(a, 4) / 24.0 + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * std::pow(a, 6) / 720.0))};
	return {kFalseEasting + x, y + kFalseNorthing};
}

glm::dvec2 to_degrees(glm::dvec2 const &Point)
{
	auto const e2{2.0 * kFlattening - kFlattening * kFlattening};
	auto const ep2{e2 / (1.0 - e2)};
	auto const x{Point.x - kFalseEasting};
	auto const m{(Point.y - kFalseNorthing) / kScale};
	auto const mu{m / (kSemiMajor * (1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0))};
	auto const e1{(1.0 - std::sqrt(1.0 - e2)) / (1.0 + std::sqrt(1.0 - e2))};
	auto const phi1{mu + (3.0 * e1 / 2.0 - 27.0 * std::pow(e1, 3) / 32.0) * std::sin(2.0 * mu) + (21.0 * e1 * e1 / 16.0 - 55.0 * std::pow(e1, 4) / 32.0) * std::sin(4.0 * mu) +
	                (151.0 * std::pow(e1, 3) / 96.0) * std::sin(6.0 * mu) + (1097.0 * std::pow(e1, 4) / 512.0) * std::sin(8.0 * mu)};
	auto const sine{std::sin(phi1)};
	auto const c1{ep2 * std::cos(phi1) * std::cos(phi1)};
	auto const t1{std::tan(phi1) * std::tan(phi1)};
	auto const n1{kSemiMajor / std::sqrt(1.0 - e2 * sine * sine)};
	auto const r1{kSemiMajor * (1.0 - e2) / std::pow(1.0 - e2 * sine * sine, 1.5)};
	auto const d{x / (n1 * kScale)};
	auto const phi{phi1 - (n1 * std::tan(phi1) / r1) * (d * d / 2.0 - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1 * c1 - 9.0 * ep2) * std::pow(d, 4) / 24.0 +
	                                                     (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1 * t1 - 252.0 * ep2 - 3.0 * c1 * c1) * std::pow(d, 6) / 720.0)};
	auto const lambda{(d - (1.0 + 2.0 * t1 + c1) * std::pow(d, 3) / 6.0 + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1 * c1 + 8.0 * ep2 + 24.0 * t1 * t1) * std::pow(d, 5) / 120.0) / std::cos(phi1)};
	return {glm::degrees(phi), kCentralMeridian + glm::degrees(lambda)};
}

bool valid_name(std::string const &Name)
{
	return false == Name.empty() && std::all_of(Name.begin(), Name.end(), [](char const Character) { return (Character >= 'a' && Character <= 'z') || (Character >= '0' && Character <= '9') || Character == '_' || Character == '-'; });
}

} // namespace

ui::scenerywizard_panel::scenerywizard_panel() : ui_panel(STR("New scenery"), false) {}

ui::scenerywizard_panel::~scenerywizard_panel()
{
	if (m_request.valid())
		m_request.wait();
	if (m_texture != 0)
		glDeleteTextures(1, &m_texture);
}

void ui::scenerywizard_panel::render_contents()
{
	upload_map();
	auto const available{ImGui::GetContentRegionAvail()};
	auto const form{360.f * Global.ui_scale};
	glm::vec2 const map{std::max(200.f, available.x - form - ImGui::GetStyle().ItemSpacing.x), std::max(200.f, available.y)};
	ImGui::BeginChild("##wizardmap", ImVec2(map.x, map.y), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
	render_map(map - glm::vec2{16.f});
	ImGui::EndChild();
	ImGui::SameLine();
	ImGui::BeginChild("##wizardform", ImVec2(0.f, map.y), true);
	render_form();
	ImGui::EndChild();
}

void ui::scenerywizard_panel::render_map(glm::vec2 const &Size)
{
	auto const origin{ImGui::GetCursorScreenPos()};
	if (false == m_fitted)
	{
		m_fitted = true;
		m_centre = (kPolandLow + kPolandHigh) * 0.5;
		m_scale = std::max((kPolandHigh.x - kPolandLow.x) / Size.x, (kPolandHigh.y - kPolandLow.y) / Size.y);
	}
	ImGui::InvisibleButton("##map", ImVec2(Size.x, Size.y));
	auto const hovered{ImGui::IsItemHovered()};
	auto const active{ImGui::IsItemActive()};
	auto const &io{ImGui::GetIO()};
	glm::vec2 const mouse{io.MousePos.x, io.MousePos.y};
	glm::vec2 const middle{origin.x + Size.x * 0.5f, origin.y + Size.y * 0.5f};
	auto const to_world = [&](glm::vec2 const &Screen) { return glm::dvec2{m_centre.x + (Screen.x - middle.x) * m_scale, m_centre.y - (Screen.y - middle.y) * m_scale}; };
	auto const to_screen = [&](glm::dvec2 const &World) { return ImVec2(middle.x + static_cast<float>((World.x - m_centre.x) / m_scale), middle.y - static_cast<float>((World.y - m_centre.y) / m_scale)); };

	if (hovered && io.MouseWheel != 0.f)
	{
		auto const under{to_world(mouse)};
		m_scale = std::clamp(m_scale * std::pow(0.8, static_cast<double>(io.MouseWheel)), 0.25, 2000.0);
		m_centre = {under.x - (mouse.x - middle.x) * m_scale, under.y + (mouse.y - middle.y) * m_scale};
		m_dirty = true;
		m_changed = seconds();
	}
	if (ImGui::IsItemActivated())
	{
		m_pressed = mouse;
		m_dragging = false;
	}
	if (active)
	{
		if (glm::length(mouse - m_pressed) > 4.f)
			m_dragging = true;
		if (m_dragging && (io.MouseDelta.x != 0.f || io.MouseDelta.y != 0.f))
		{
			m_centre.x -= io.MouseDelta.x * m_scale;
			m_centre.y += io.MouseDelta.y * m_scale;
			m_dirty = true;
			m_changed = seconds();
		}
	}
	if (ImGui::IsItemDeactivated() && false == m_dragging)
		pick(to_world(mouse));

	if (m_dirty && seconds() - m_changed > 0.25 && false == m_request.valid())
		request_map(Size);

	auto *drawlist{ImGui::GetWindowDrawList()};
	drawlist->PushClipRect(origin, ImVec2(origin.x + Size.x, origin.y + Size.y), true);
	drawlist->AddRectFilled(origin, ImVec2(origin.x + Size.x, origin.y + Size.y), IM_COL32(200, 214, 222, 255));
	if (m_texture != 0)
		drawlist->AddImage(reinterpret_cast<ImTextureID>(static_cast<std::intptr_t>(m_texture)), to_screen({m_image_low.x, m_image_high.y}), to_screen({m_image_high.x, m_image_low.y}));
	if (m_picked)
	{
		auto const at{to_screen(m_point)};
		auto const colour{IM_COL32(220, 30, 30, 255)};
		drawlist->AddCircle(at, 10.f, colour, 24, 2.5f);
		drawlist->AddLine(ImVec2(at.x - 16.f, at.y), ImVec2(at.x + 16.f, at.y), colour, 2.f);
		drawlist->AddLine(ImVec2(at.x, at.y - 16.f), ImVec2(at.x, at.y + 16.f), colour, 2.f);
	}
	std::string info;
	if (hovered)
	{
		auto const world{to_world(mouse)};
		auto const degrees{to_degrees(world)};
		info = format(STR_C("E %.3f km, N %.3f km  (%.5f N, %.5f E)"), world.x / 1000.0, world.y / 1000.0, degrees.x, degrees.y);
	}
	if (m_request.valid())
		info += (info.empty() ? "" : "   ") + std::string{STR_C("loading the map...")};
	else if (false == m_map_error.empty())
		info += (info.empty() ? "" : "   ") + m_map_error;
	if (false == info.empty())
	{
		auto const size{ImGui::CalcTextSize(info.c_str())};
		drawlist->AddRectFilled(ImVec2(origin.x + 4.f, origin.y + 4.f), ImVec2(origin.x + 12.f + size.x, origin.y + 8.f + size.y), IM_COL32(0, 0, 0, 170), 4.f);
		drawlist->AddText(ImVec2(origin.x + 8.f, origin.y + 6.f), IM_COL32(255, 255, 255, 240), info.c_str());
	}
	drawlist->PopClipRect();
}

void ui::scenerywizard_panel::request_map(glm::vec2 const &Size)
{
	m_dirty = false;
	auto const width{std::clamp(static_cast<int>(Size.x), 64, 2048)};
	auto const height{std::clamp(static_cast<int>(Size.y), 64, 2048)};
	glm::dvec2 const half{width * 0.5 * m_scale, height * 0.5 * m_scale};
	auto const low{m_centre - half};
	auto const high{m_centre + half};
	char bbox[160];
	std::snprintf(bbox, sizeof(bbox), "%.1f,%.1f,%.1f,%.1f", low.x, low.y, high.x, high.y);
	auto const url{kMapService + "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&LAYERS=osm&STYLES=&SRS=EPSG:2180&BBOX=" + bbox + "&WIDTH=" + std::to_string(width) + "&HEIGHT=" + std::to_string(height) + "&FORMAT=image/png"};
	m_request = std::async(std::launch::async, [url, low, high, width, height]() {
		map_result result;
		result.low = low;
		result.high = high;
		std::vector<std::uint8_t> data;
		if (false == editor_http_get(url, data, result.error))
			return result;
		int w{0}, h{0}, components{0};
		auto *pixels{stbi_load_from_memory(data.data(), static_cast<int>(data.size()), &w, &h, &components, 4)};
		if (pixels == nullptr)
		{
			result.error = STR("The map service sent no image");
			return result;
		}
		result.width = w;
		result.height = h;
		result.pixels.assign(pixels, pixels + static_cast<std::size_t>(w) * h * 4);
		stbi_image_free(pixels);
		return result;
	});
}

void ui::scenerywizard_panel::upload_map()
{
	if (false == m_request.valid() || m_request.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
		return;
	auto result{m_request.get()};
	if (false == result.error.empty() || result.pixels.empty())
	{
		m_map_error = result.error.empty() ? std::string{STR("The map service sent no image")} : STR("No map: ") + result.error;
		return;
	}
	m_map_error.clear();
	if (m_texture == 0)
		glGenTextures(1, &m_texture);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, result.width, result.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, result.pixels.data());
	glBindTexture(GL_TEXTURE_2D, 0);
	m_image_low = result.low;
	m_image_high = result.high;
}

void ui::scenerywizard_panel::pick(glm::dvec2 const &Point)
{
	m_point = Point;
	m_picked = true;
	auto const degrees{to_degrees(Point)};
	std::snprintf(m_latlon, sizeof(m_latlon), "%.6f, %.6f", degrees.x, degrees.y);
}

void ui::scenerywizard_panel::render_form()
{
	ImGui::TextUnformatted(STR_C("New scenery"));
	ImGui::Separator();
	ImGui::TextWrapped("%s", STR_C("Find the place on the map: the wheel zooms, dragging moves the map. A click sets the centre of the scenery, its point 0 0 0."));
	ImGui::Spacing();

	ImGui::SetNextItemWidth(-1.f);
	ImGui::InputTextWithHint("##name", STR_C("file name, e.g. my_line"), m_name, sizeof(m_name), ImGuiInputTextFlags_CharsNoBlank);
	std::string name{m_name};
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

	ImGui::TextUnformatted(STR_C("Centre of the scenery, PUWG 1992"));
	double east{m_point.x / 1000.0};
	double north{m_point.y / 1000.0};
	ImGui::SetNextItemWidth(130.f);
	auto changed{ImGui::InputDouble(STR_C("E, km##east"), &east, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)};
	ImGui::SetNextItemWidth(130.f);
	changed |= ImGui::InputDouble(STR_C("N, km##north"), &north, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue);
	if (changed)
	{
		pick({east * 1000.0, north * 1000.0});
		m_centre = m_point;
		m_dirty = true;
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
			pick(to_puwg(latitude, longitude));
			m_centre = m_point;
			m_scale = std::min(m_scale, 4.0);
			m_dirty = true;
			m_status.clear();
		}
		else
			m_status = STR("Give the latitude and the longitude in degrees, e.g. 52.2297, 21.0122");
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Degrees, as given by the online maps; the map moves there"));
	if (m_picked)
	{
		auto const degrees{to_degrees(m_point)};
		ImGui::TextDisabled(STR_C("%.5f N, %.5f E"), degrees.x, degrees.y);
	}
	ImGui::Spacing();
	ImGui::TextWrapped("%s", STR_C("The scenery keeps the centre as its geographic reference (//$g PUWG1992): the orthophoto of the editor lies under it, in place."));
	ImGui::Spacing();

	auto const ready{named && false == exists && m_picked};
	if (false == ready)
		ImGui::TextDisabled("%s", m_picked ? STR_C("Give the file a name") : STR_C("Click the centre on the map"));
	if (ImGui::Button(STR_C("Create and open in the editor"), ImVec2(-1.f, 0.f)) && ready)
		create_scenery();
	if (false == m_status.empty())
		ImGui::TextWrapped("%s", m_status.c_str());
	ImGui::Spacing();
	ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
	ImGui::TextWrapped("%s", STR_C("Map: (c) OpenStreetMap contributors, served by geoportal.gov.pl"));
	ImGui::PopStyleColor();
}

bool ui::scenerywizard_panel::create_scenery()
{
	std::string name{m_name};
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
	std::snprintf(reference, sizeof(reference), "//$g PUWG1992 %.3f %.3f", m_point.x / 1000.0, m_point.y / 1000.0);
	output << reference << "\r\n";
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
