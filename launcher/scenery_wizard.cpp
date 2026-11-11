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
#include "editor/editorGeometry.hpp"
#include "editor/editorOrthophoto.hpp"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>

namespace
{

bool valid_name(std::string const &Name)
{
	return false == Name.empty() && std::all_of(Name.begin(), Name.end(), [](char const Character) { return (Character >= 'a' && Character <= 'z') || (Character >= '0' && Character <= '9') || Character == '_' || Character == '-'; });
}

// the sides of the square around the centre offered, km
std::array<int, 4> constexpr sides{1, 2, 5, 10};
// km², above it the download is told to take long
double constexpr large_range{50.0};
// m a pixel: zoomed out more than this the centre clicked may be off by hundreds of metres
double constexpr coarse_scale{60.0};

ImVec4 const warning_colour{1.f, 0.78f, 0.3f, 1.f};
ImVec4 const error_colour{1.f, 0.45f, 0.35f, 1.f};

void wrapped(ImVec4 const &Colour, std::string const &Text)
{
	ImGui::PushStyleColor(ImGuiCol_Text, Colour);
	ImGui::TextWrapped("%s", Text.c_str());
	ImGui::PopStyleColor();
}

void wrapped_disabled(std::string const &Text)
{
	wrapped(ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled), Text);
}

void bounds(std::vector<glm::dvec2> const &Points, glm::dvec2 &Low, glm::dvec2 &High)
{
	Low = High = Points.front();
	for (auto const &point : Points)
	{
		Low = glm::min(Low, point);
		High = glm::max(High, point);
	}
}

} // namespace

ui::scenerywizard_panel::scenerywizard_panel() : ui_panel(STR("New scenery"), false) {}

ui::scenerywizard_panel::~scenerywizard_panel() = default;

void ui::scenerywizard_panel::render_contents()
{
	auto const available{ImGui::GetContentRegionAvail()};
	auto const form{380.f * Global.ui_scale};
	glm::vec2 const map{std::max(200.f, available.x - form - ImGui::GetStyle().ItemSpacing.x), std::max(200.f, available.y)};

	// what the map shows and what a click on it does
	if (false == m_realistic)
	{
		m_map.outline({}, false);
		m_map.picking(false);
		m_map.hint(STR("A fictional scenery needs no place on the map"));
	}
	else if (m_drawing)
	{
		m_map.outline(m_drawn, false);
		m_map.picking(false);
		m_map.hint(m_drawn.size() < 3 ? STR("Click the corners of the area, one after another around it") : STR("Click the next corner, or Done when the area is whole"));
	}
	else
	{
		m_map.outline(range(), true);
		m_map.picking(true);
		m_map.hint(m_map.picked() ? STR("A click moves the centre of the scenery") : STR("Click the centre of the scenery. The wheel zooms the map, dragging moves it"));
	}

	ImGui::BeginChild("##wizardmap", ImVec2(map.x, map.y), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
	if (m_map.render(map - glm::vec2{16.f}) && m_realistic)
	{
		if (m_drawing)
			m_drawn.push_back(m_map.clicked());
		else
			pick(m_map.point());
	}
	ImGui::EndChild();
	// the keys of the drawing as in the editor, unless a field is being typed in
	if (m_drawing && false == ImGui::GetIO().WantTextInput)
	{
		if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false))
			stop_drawing(true);
		else if (ImGui::IsKeyPressed(ImGuiKey_Escape, false))
			stop_drawing(false);
		else if (ImGui::IsKeyPressed(ImGuiKey_Backspace) && false == m_drawn.empty())
			m_drawn.pop_back();
	}
	ImGui::SameLine();
	ImGui::BeginChild("##wizardform", ImVec2(0.f, map.y), true);
	render_form();
	ImGui::EndChild();
}

void ui::scenerywizard_panel::pick(glm::dvec2 const &Point)
{
	auto const first{false == m_map.picked() || m_map.scale() > coarse_scale};
	m_status.clear();
	m_map.pick(Point);
	auto const degrees{editor_geo::to_degrees(Point)};
	std::snprintf(m_place, sizeof(m_place), "%.6f, %.6f", degrees.x, degrees.y);
	// picked on the map of the whole country: the map goes to the place, to see the range around it and to put the centre exactly
	if (first)
		show_range();
}

void ui::scenerywizard_panel::start_drawing()
{
	m_before = m_drawn;
	m_drawn.clear();
	m_drawing = true;
}

void ui::scenerywizard_panel::stop_drawing(bool const Keep)
{
	if (false == m_drawing || (Keep && m_drawn.size() < 3))
		return;
	m_drawing = false;
	if (Keep)
	{
		show_range();
		return;
	}
	// given up: the area there was before, or the square when there was none
	m_drawn = m_before;
	if (m_drawn.size() < 3)
		m_shape = range_shape::square;
}

void ui::scenerywizard_panel::show_range()
{
	auto const area{range()};
	if (area.empty())
	{
		m_map.show(m_map.point(), 8.0);
		return;
	}
	glm::dvec2 low, high;
	bounds(area, low, high);
	m_map.fit(low, high);
}

std::vector<glm::dvec2> ui::scenerywizard_panel::range() const
{
	if (false == m_realistic)
		return {};
	if (m_shape == range_shape::drawn)
		return m_drawn.size() >= 3 ? m_drawn : std::vector<glm::dvec2>{};
	if (false == m_map.picked())
		return {};
	auto const half{m_side * 500.0};
	auto const &centre{m_map.point()};
	return {{centre.x - half, centre.y - half}, {centre.x + half, centre.y - half}, {centre.x + half, centre.y + half}, {centre.x - half, centre.y + half}};
}

std::string ui::scenerywizard_panel::missing(bool const Named, bool const Exists) const
{
	if (false == Named)
		return STR("Give the file a name (step 1)");
	if (Exists)
		return STR("A scenery of this name exists already: give another name (step 1)");
	if (false == m_realistic)
		return {};
	if (false == m_map.picked())
		return STR("Click the centre of the scenery on the map (step 2)");
	if (false == editor_geo::in_poland(m_map.point()))
		return STR("The centre is outside Poland: the aerial photos cover Poland only (step 2)");
	if (m_drawing)
		return STR("Finish drawing the area: Done (step 3)");
	auto const area{range()};
	if (area.empty())
		return STR("Draw the area of the aerial photos on the map (step 3)");
	if (geometry::polygon_crosses_itself(area))
		return STR("The edges of the area cross each other: draw it again (step 3)");
	return {};
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
	{
		// an area being drawn is given up, as with Cancel
		stop_drawing(false);
		m_realistic = false;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("No place on the map, no orthophoto: an empty plane seen from above"));

	ImGui::SeparatorText(STR_C("1. Name"));
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
		ImGui::TextColored(error_colour, "%s", STR_C("Lower case letters, digits, _ and - only"));
	else if (exists)
		ImGui::TextColored(error_colour, STR_C("%s exists already"), path.c_str());
	else
		ImGui::TextDisabled("%s", path.c_str());
	ImGui::SetNextItemWidth(-1.f);
	ImGui::InputTextWithHint("##title", STR_C("title, written in the file"), m_title, sizeof(m_title));

	if (m_realistic)
	{
		render_place();
		render_range();
	}

	ImGui::Spacing();
	ImGui::Separator();
	auto const reason{missing(named, exists)};
	ImGui::BeginDisabled(false == reason.empty());
	if (ImGui::Button(STR_C("Create and open in the editor"), ImVec2(-1.f, ImGui::GetFrameHeight() * 1.4f)))
		create_scenery();
	ImGui::EndDisabled();
	if (false == reason.empty())
		wrapped_disabled(reason);
	if (false == m_status.empty())
		wrapped(error_colour, m_status);

	ImGui::Spacing();
	wrapped_disabled(STR("Map: (c) OpenStreetMap contributors, served by geoportal.gov.pl"));
}

void ui::scenerywizard_panel::render_place()
{
	ImGui::SeparatorText(STR_C("2. Where the scenery lies"));
	wrapped_disabled(STR("Click its centre on the map, or type the place and press Enter"));
	ImGui::SetNextItemWidth(-1.f);
	if (ImGui::InputTextWithHint("##place", STR_C("e.g. 52.2297, 21.0122"), m_place, sizeof(m_place), ImGuiInputTextFlags_EnterReturnsTrue))
	{
		glm::dvec2 typed;
		if (editor_geo::parse_place(m_place, typed))
		{
			m_status.clear();
			m_map.pick(typed);
			// the square goes with the centre and is shown whole; an area of the user's stays, the map goes to the centre
			if (m_shape == range_shape::square)
				show_range();
			else
				m_map.show(typed, 8.0);
		}
		else
			m_status = STR("Not a place in Poland: two numbers, degrees or the X and Y of PUWG 1992");
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Degrees as the online maps copy them (latitude, longitude), or the X and Y of PUWG 1992, the coordinates of the Polish maps of geoportal.gov.pl"));
	if (false == m_map.picked())
		return;
	auto const degrees{editor_geo::to_degrees(m_map.point())};
	ImGui::TextDisabled(STR_C("%.5f N, %.5f E"), degrees.x, degrees.y);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip(STR_C("X %.0f, Y %.0f in PUWG 1992; the point 0 0 0 of the scenery"), m_map.point().y, m_map.point().x);
	if (m_map.scale() > coarse_scale)
		wrapped(warning_colour, STR("Zoom the map in with the wheel, to put the centre exactly"));
}

void ui::scenerywizard_panel::render_range()
{
	ImGui::SeparatorText(STR_C("3. Range of the aerial photos"));
	wrapped_disabled(STR("The editor downloads the aerial photos and the shape of the ground only inside this area, blue on the map. Outside it there are no photos."));
	// the area is given around the centre: without it the clicks on the map would go to the corners of an area, not to the centre
	auto const placed{m_map.picked()};
	if (false == placed)
		wrapped(warning_colour, STR("Click the centre on the map first (step 2)"));
	ImGui::BeginDisabled(false == placed);
	ImGui::BeginDisabled(m_drawing);
	if (ImGui::RadioButton(STR_C("A square around the centre"), m_shape == range_shape::square))
	{
		m_shape = range_shape::square;
		if (m_map.picked())
			show_range();
	}
	ImGui::EndDisabled();
	if (m_shape == range_shape::square)
	{
		ImGui::Indent();
		for (auto const side : sides)
		{
			if (side != sides.front())
				ImGui::SameLine();
			if (ImGui::RadioButton(format(STR_C("%d x %d km##side"), side, side).c_str(), m_side == side))
			{
				m_side = side;
				if (m_map.picked())
					show_range();
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip(STR_C("%d km from edge to edge, the centre of the scenery in its middle, %.1f km from each edge"), side, side * 0.5);
		}
		ImGui::Unindent();
	}
	if (ImGui::RadioButton(STR_C("My own area, drawn on the map"), m_shape == range_shape::drawn) && m_shape != range_shape::drawn)
	{
		m_shape = range_shape::drawn;
		// nothing drawn yet: the clicks on the map go to it straight away
		if (m_drawn.size() < 3)
			start_drawing();
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("For a line: a long band along it, which downloads much less than a square around it"));
	if (m_shape == range_shape::drawn)
	{
		ImGui::Indent();
		if (m_drawing)
		{
			ImGui::Text(STR_C("%zu corners"), m_drawn.size());
			ImGui::BeginDisabled(m_drawn.size() < 3);
			if (ImGui::Button(STR_C("Done")))
				stop_drawing(true);
			ImGui::EndDisabled();
			ImGui::SameLine();
			ImGui::BeginDisabled(m_drawn.empty());
			if (ImGui::Button(STR_C("Undo the corner")))
				m_drawn.pop_back();
			ImGui::EndDisabled();
			ImGui::SameLine();
			if (ImGui::Button(STR_C("Cancel")))
				stop_drawing(false);
			ImGui::TextDisabled("%s", STR_C("Enter: done, Backspace: undo the corner, Esc: cancel"));
		}
		else if (ImGui::Button(STR_C("Draw again")))
			start_drawing();
		ImGui::Unindent();
	}
	ImGui::EndDisabled();

	auto const area{range()};
	if (area.empty())
		return;
	auto const size{geometry::polygon_area(area) / 1.0e6};
	auto const tiles{geometry::squares_in(area, editor_orthophoto::tile_size).size()};
	ImGui::TextDisabled(STR_C("About %.1f sq km, %zu photos of 256 x 256 m"), size, tiles);
	// while it's drawn the area isn't whole yet: what it holds is told once it's done
	if (m_drawing)
		return;
	if (geometry::polygon_crosses_itself(area))
		wrapped(error_colour, STR("The edges of the area cross each other: draw it again, clicking the corners in turn around it"));
	if (m_map.picked() && false == geometry::point_in_polygon(area, m_map.point()))
		wrapped(warning_colour, STR("The centre of the scenery is outside the area: there will be no photos under it"));
	if (size > large_range)
		wrapped(warning_colour, STR("A large area: the download takes long and needs a lot of disk space. It goes on in the background, the editor can be used meanwhile."));
	wrapped_disabled(STR("It can be changed later in the editor: Map > Orthophoto settings... > Download area"));
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
	auto const area{range()};
	if (m_realistic)
	{
		// the orthophoto on; it's shown as the settings of the editor have it, its origin is the one of //$g
		output << reference << "\r\n";
		output << "//$o " << editor_orthophoto::mark(editor_orthophoto::config{}, true, false) << "\r\n";
		// the range of the photos, the download area of the editor
		for (auto const &line : editor_orthophoto::area_lines(area))
			output << "//$z " << line << "\r\n";
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
	if (m_realistic)
		WriteLog(format("Scenery wizard: created %s at %s, the range of the photos %zu points, %.1f km2", file.c_str(), reference, area.size(), geometry::polygon_area(area) / 1.0e6));
	else
		WriteLog("Scenery wizard: created " + file + ", fictional");

	Global.SceneryFile = file;
	Global.local_start_vehicle = "ghostview";
	Global.editor_session = true;
	Application.pop_mode();
	Application.push_mode(eu07_application::mode::scenarioloader);
	return true;
}
