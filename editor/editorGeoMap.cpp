/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorGeoMap.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorOrthophoto.hpp"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "rendering/renderer.h"
#include "stb/stb_image.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

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

} // namespace

namespace editor_geo
{

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

bool in_poland(glm::dvec2 const &Point)
{
	return Point.x >= kPolandLow.x && Point.x <= kPolandHigh.x && Point.y >= kPolandLow.y && Point.y <= kPolandHigh.y;
}

bool parse_place(std::string const &Text, glm::dvec2 &Point)
{
	// the letters and the signs around the numbers out, as of "52.2297° N, 21.0122° E" or "X: 456787 Y: 579897"
	std::string text;
	for (unsigned char const letter : Text)
		text += std::isdigit(letter) || letter == '.' || letter == ',' || letter == '-' ? static_cast<char>(letter) : ' ';
	std::vector<std::string> words;
	for (std::size_t at{0}; at < text.size();)
	{
		auto const begin{text.find_first_not_of(" ,", at)};
		if (begin == std::string::npos)
			break;
		auto end{text.find(' ', begin)};
		if (end == std::string::npos)
			end = text.size();
		auto word{text.substr(begin, end - begin)};
		while (false == word.empty() && word.back() == ',')
			word.pop_back();
		words.push_back(word);
		at = end;
	}
	std::vector<std::string> numbers;
	if (words.size() == 1)
	{
		// one word: the comma parts the numbers, or the numbers and their decimals
		std::vector<std::string> parts;
		for (std::size_t at{0};;)
		{
			auto const comma{words[0].find(',', at)};
			parts.push_back(words[0].substr(at, comma == std::string::npos ? std::string::npos : comma - at));
			if (comma == std::string::npos)
				break;
			at = comma + 1;
		}
		if (parts.size() == 2)
			numbers = parts;
		else if (parts.size() == 4)
			numbers = {parts[0] + "." + parts[1], parts[2] + "." + parts[3]};
	}
	else if (words.size() == 2)
		for (auto word : words)
		{
			std::replace(word.begin(), word.end(), ',', '.');
			numbers.push_back(word);
		}
	if (numbers.size() != 2)
		return false;
	double values[2];
	for (int i = 0; i < 2; ++i)
	{
		char *end{nullptr};
		values[i] = std::strtod(numbers[i].c_str(), &end);
		if (end == numbers[i].c_str() || *end != '\0')
			return false;
	}
	auto const latitude = [](double const Value) { return Value >= 48.0 && Value <= 56.0; };
	auto const longitude = [](double const Value) { return Value >= 13.0 && Value <= 25.0; };
	if (latitude(values[0]) && longitude(values[1]))
		Point = to_puwg(values[0], values[1]);
	else if (longitude(values[0]) && latitude(values[1]))
		Point = to_puwg(values[1], values[0]);
	else if (values[0] > 1000.0 && values[1] > 1000.0)
		Point = {values[1], values[0]};
	else
		return false;
	return in_poland(Point);
}

} // namespace editor_geo

editor_map::~editor_map()
{
	if (m_request.valid())
		m_request.wait();
	release();
}

void editor_map::release()
{
	if (m_texture != 0 && GfxRenderer && GfxRenderer->GetImguiRenderer() != nullptr)
		GfxRenderer->GetImguiRenderer()->Release_Image(m_texture);
	m_texture = 0;
}

void editor_map::pick(glm::dvec2 const &Point)
{
	m_point = Point;
	m_picked = true;
}

void editor_map::show(glm::dvec2 const &Point, double const Scale)
{
	m_fitted = true;
	m_centre = Point;
	m_scale = std::min(m_scale, Scale);
	m_dirty = true;
}

bool editor_map::render(glm::vec2 const &Size)
{
	upload();
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
	auto const clicked{ImGui::IsItemDeactivated() && false == m_dragging};
	if (clicked)
		pick(to_world(mouse));

	if (m_dirty && seconds() - m_changed > 0.25 && false == m_request.valid())
		request(Size);

	auto *drawlist{ImGui::GetWindowDrawList()};
	drawlist->PushClipRect(origin, ImVec2(origin.x + Size.x, origin.y + Size.y), true);
	drawlist->AddRectFilled(origin, ImVec2(origin.x + Size.x, origin.y + Size.y), IM_COL32(200, 214, 222, 255));
	if (m_texture != 0)
		drawlist->AddImage(static_cast<ImTextureID>(m_texture), to_screen({m_image_low.x, m_image_high.y}), to_screen({m_image_high.x, m_image_low.y}), ImVec2(0, 0), ImVec2(1, 1));
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
		auto const degrees{editor_geo::to_degrees(world)};
		info = format(STR_C("E %.3f km, N %.3f km  (%.5f N, %.5f E)"), world.x / 1000.0, world.y / 1000.0, degrees.x, degrees.y);
	}
	if (m_request.valid())
		info += (info.empty() ? "" : "   ") + std::string{STR_C("loading the map...")};
	else if (false == m_error.empty())
		info += (info.empty() ? "" : "   ") + m_error;
	if (false == info.empty())
	{
		auto const size{ImGui::CalcTextSize(info.c_str())};
		drawlist->AddRectFilled(ImVec2(origin.x + 4.f, origin.y + 4.f), ImVec2(origin.x + 12.f + size.x, origin.y + 8.f + size.y), IM_COL32(0, 0, 0, 170), 4.f);
		drawlist->AddText(ImVec2(origin.x + 8.f, origin.y + 6.f), IM_COL32(255, 255, 255, 240), info.c_str());
	}
	drawlist->PopClipRect();
	return clicked;
}

void editor_map::request(glm::vec2 const &Size)
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
	m_request = std::async(std::launch::async, [url, low, high]() {
		image result;
		result.low = low;
		result.high = high;
		std::vector<std::uint8_t> data;
		if (false == editor_http_get(url, data, result.error))
			return result;
		int w{0}, h{0}, components{0};
		// the texture loader turns the stb flip on for the whole process only once it loads its first texture: this thread
		// keeps the north edge in the first row whatever it is by then
		stbi_set_flip_vertically_on_load_thread(0);
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

void editor_map::upload()
{
	if (false == m_request.valid() || m_request.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
		return;
	auto result{m_request.get()};
	if (false == result.error.empty() || result.pixels.empty())
	{
		m_error = result.error.empty() ? std::string{STR("The map service sent no image")} : STR("No map: ") + result.error;
		return;
	}
	m_error.clear();
	release();
	auto *renderer{GfxRenderer ? GfxRenderer->GetImguiRenderer() : nullptr};
	m_texture = renderer != nullptr ? renderer->Create_Image(result.pixels.data(), result.width, result.height) : 0;
	m_image_low = result.low;
	m_image_high = result.high;
}
