/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormoderailtools.h"
#include "application/editoruilayer.h"

rail_tools::rail_tools(editor_services &Services, editor_rail_services &Rail)
    : speed{Services, Services, Services, Services}, srk{Services, Services, Services, Services, Services}, km{Services, Services, Services, Services}, switch_designer{Services, Services, Rail}, track_sheet{Services, Services, Services, Rail}, track_style{Services, Rail, track_sheet}
{
}

void editor_mode::make_rail_tools()
{
	editor_services &services{*this};
	editor_rail_services &rail{*this};
	m_rail_tools = std::make_unique<rail_tools>(services, rail);
	m_tools.insert(m_tools.end(), {&m_rail_tools->speed, &m_rail_tools->srk, &m_rail_tools->km, &m_rail_tools->switch_designer, &m_rail_tools->track_sheet, &m_rail_tools->track_style});
}

void editor_mode::arm_switch_tool()
{
	arm_switch();
}

glm::dvec3 editor_mode::cursor_on_ground() const
{
	return cursor_ground();
}

glm::dvec3 editor_mode::cursor_on_level(double const Height) const
{
	return cursor_level(Height);
}

void editor_mode::draw_ring(glm::dvec3 const &Point) const
{
	draw_cursor_ring(Point);
}

bool editor_mode::orthophoto_shown() const
{
	return m_orthophoto.enabled();
}

void editor_mode::open_orthophoto_window()
{
	m_orthophoto_window = true;
}

void editor_mode::show_status(std::string const &Text, bool const Warning)
{
	ui()->set_status(Text, Warning);
}

std::size_t editor_mode::history_size() const
{
	return m_history.size();
}

void editor_mode::save_scenery()
{
	save();
}

bool editor_mode::pick_track_texture(int const Kind, std::string const &Current, bool const Mixed, std::string &Chosen)
{
	return track_texture_picker(Kind, Current, Mixed, Chosen);
}

bool editor_mode::take_dropped_material(std::string &Name)
{
	return material_drop(Name);
}

std::vector<TTrack *> editor_mode::selected_track_list()
{
	return selected_tracks();
}

void editor_mode::select_track_list(std::vector<TTrack *> const &Tracks)
{
	select_tracks(Tracks);
}

void editor_mode::grow_selection(std::vector<TTrack *> const &From)
{
	if (m_track_set.empty() && false == From.empty())
		m_track_set = From;
	track_set_grow();
	select_tracks(m_track_set);
}

void editor_mode::toggle_in_selection(TTrack &Track)
{
	track_set_toggle(Track);
}
