/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the brush of models of the Surroundings: one model where clicked, or many in a circle along the path of the cursor

#include "stdafx.h"

#include "application/editormode.h"
#include "application/editorprojection.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorIncludeInfo.hpp"
#include "model/AnimModel.h"
#include "simulation/simulation.h"
#include "utilities/translation.h"

#include <array>
#include <cmath>
#include <numbers>

namespace
{

ImU32 constexpr kCrossColour{IM_COL32(255, 220, 70, 255)};
ImU32 constexpr kCircleColour{IM_COL32(255, 220, 70, 170)};
ImU32 constexpr kShadowColour{IM_COL32(0, 0, 0, 140)};

// the last press of the left button landed in the viewport, not on a window (see editormode.cpp)
bool viewport_click()
{
	return false == ImGui::GetIO().MouseDownOwned[0];
}

} // namespace

bool editor_mode::brush_ground(glm::dvec3 &Point)
{
	if (false == cursor_on_ground(Point))
		return false;
	// no further than things are placed at all
	auto const offset{Point - glm::dvec3{Camera.Pos}};
	Point = placement_on_ground(glm::dvec3{Camera.Pos} + clamp_mouse_offset_to_max(offset));
	return std::isfinite(Point.x) && std::isfinite(Point.y) && std::isfinite(Point.z);
}

double editor_mode::brush_height(glm::dvec2 const &Point, double const Around)
{
	if (double height{Around}; m_streamer.active() && m_streamer.height_at(Point.x, Point.y, height))
		return height;
	return placement_on_ground({Point.x, Around, Point.y}).y;
}

TAnimModel *editor_mode::place_brush_model(glm::dvec3 const &At)
{
	auto const *source{ui()->get_active_node_template()};
	// the scenery templates are put one at a time with the insert mode
	if (source == nullptr || source->starts_with(editor_includes::directive_mark))
		return nullptr;
	// NOTE: no name, as with thousands of nodes the names take their share of the time the scenery needs to load. a node which
	// needs one, to be referred to by an event, gets it in the node properties
	auto *model{simulation::State.create_model(*source, std::string{}, At)};
	if (model == nullptr)
		return nullptr;
	auto const look{scatter::pick(ui()->brush().look, m_brush_random)};
	m_editor.rotate(model, look.angles, 0.0f);
	model->Scale(model->Scale() * look.scale);
	add_to_hierarchy(model);
	return model;
}

void editor_mode::brush_press()
{
	mouseHold = true;
	m_brush_has_last = false;
	m_brush_taken.clear();
	if (ui()->brush().way != brush_object_list::kind::single)
	{
		// one step of the history for the whole stroke, the models added to it as they're put
		trim_history();
		EditorSnapshot stroke;
		stroke.action = EditorSnapshot::Action::Array;
		m_history.push_back(std::move(stroke));
		g_redo.clear();
		m_brush_stroke = true;
		return;
	}
	// the point is read once the renderer has the depth under the cursor of the press
	GfxRenderer->Pick_Node_Callback([this](scene::basic_node * /*node*/) {
		if (false == viewport_click())
			return;
		glm::dvec3 at;
		if (false == brush_ground(at))
			return;
		auto *model{place_brush_model(at)};
		if (model == nullptr)
			return;
		std::string text;
		model->export_as_text(text);
		push_snapshot(model, EditorSnapshot::Action::Add, text);
		m_node = model;
		ui()->set_node(m_node);
	});
}

void editor_mode::handle_brush_mouse_hold(int /*Action*/, int /*Button*/)
{
	if (ui()->mode() != nodebank_panel::BRUSH || false == m_brush_stroke)
		return;
	GfxRenderer->Pick_Node_Callback([this](scene::basic_node * /*node*/) { brush_dab(); });
}

void editor_mode::brush_dab()
{
	// picks are resolved a few frames later; drop the ones queued before the button went up, or for a press that turned out
	// to belong to the UI
	if (false == mouseHold || false == m_brush_stroke || false == viewport_click() || m_history.empty())
		return;
	glm::dvec3 centre;
	if (false == brush_ground(centre))
		return;
	auto const &settings{ui()->brush().dab};
	// the next dab once the cursor went far enough, measured on the ground, whatever the camera does meanwhile
	if (m_brush_has_last && glm::distance(glm::dvec2{centre.x, centre.z}, glm::dvec2{oldPos.x, oldPos.z}) < scatter::dab_step(settings))
		return;
	oldPos = centre;
	m_brush_has_last = true;
	auto &stroke{m_history.back()};
	for (auto const &point : scatter::dab({centre.x, centre.z}, settings, m_brush_taken, m_brush_random))
	{
		auto *model{place_brush_model({point.x, brush_height(point, centre.y), point.y})};
		// the place is taken either way, so a model which failed isn't tried there on every move
		m_brush_taken.push_back(point);
		if (model == nullptr)
			continue;
		EditorSnapshot::array_copy copy;
		copy.model = model;
		copy.uuid = model->uuid;
		stroke.layer = model->layer();
		stroke.copies.push_back(std::move(copy));
	}
}

void editor_mode::end_brush_stroke()
{
	if (false == m_brush_stroke)
		return;
	m_brush_stroke = false;
	if (m_history.empty() || m_history.back().action != EditorSnapshot::Action::Array)
		return;
	auto const count{m_history.back().copies.size()};
	if (count == 0)
	{
		// nothing put: no empty step in the history
		m_history.pop_back();
		return;
	}
	ui()->set_status(format(STR_C("Brush: %zu models put; Ctrl+Z takes the whole stroke back"), count));
}

void editor_mode::draw_brush_cursor()
{
	glm::dvec3 centre;
	if (false == brush_ground(centre))
		return;
	screen_projection const view;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const &settings{ui()->brush()};
	auto const many{settings.way == brush_object_list::kind::multi};
	// the cross on the ground, as large as a tenth of the circle (a metre for the single model), between 0.5 and 3 m
	auto const arm{std::clamp(many ? 0.1 * settings.dab.radius : 1.0, 0.5, 3.0)};
	auto const ground = [&](double const X, double const Z) { return glm::dvec3{X, brush_height({X, Z}, centre.y) + 0.05, Z}; };
	std::array<std::pair<glm::dvec3, glm::dvec3>, 2> const arms{std::make_pair(ground(centre.x - arm, centre.z), ground(centre.x + arm, centre.z)),
	                                                             std::make_pair(ground(centre.x, centre.z - arm), ground(centre.x, centre.z + arm))};
	for (auto const &[a, b] : arms)
	{
		view.line(drawlist, a, b, kShadowColour, 4.0f);
		view.line(drawlist, a, b, kCrossColour, 2.0f);
	}
	if (false == many)
		return;
	// the circle, laid on the ground
	int constexpr pieces{64};
	auto const radius{std::max(0.1, settings.dab.radius)};
	auto previous{ground(centre.x + radius, centre.z)};
	for (int i = 1; i <= pieces; ++i)
	{
		auto const angle{2.0 * std::numbers::pi * i / pieces};
		auto const next{ground(centre.x + radius * std::cos(angle), centre.z + radius * std::sin(angle))};
		view.line(drawlist, previous, next, kShadowColour, 3.5f);
		view.line(drawlist, previous, next, kCircleColour, 1.8f);
		previous = next;
	}
}
