#include "stdafx.h"
#include "widgets/map.h"
#include "widgets/map_objects.h"
#include "vehicle/Train.h"
#include "vehicle/Camera.h"
#include "simulation/simulation.h"
#include "vehicle/Driver.h"
#include "model/AnimModel.h"
#include "application/application.h"
#include "editor/editorTerrainStreamer.hpp"
#include "world/Sweep.h"

namespace
{

int constexpr map_vertex_limit = 48000;

bool map_has_vertex_budget(ImDrawList const *draw_list, int const required = 12)
{
	return draw_list->VtxBuffer.Size + required < map_vertex_limit;
}

bool clip_map_line(ImVec2 &a, ImVec2 &b, ImVec2 origin, glm::vec2 surface_size)
{
	float const left = origin.x;
	float const top = origin.y;
	float const right = origin.x + surface_size.x;
	float const bottom = origin.y + surface_size.y;
	float const dx = b.x - a.x;
	float const dy = b.y - a.y;
	float t_min = 0.0f;
	float t_max = 1.0f;
	const std::array<float, 4> p{-dx, dx, -dy, dy};
	const std::array<float, 4> q{a.x - left, right - a.x, a.y - top, bottom - a.y};
	for (std::size_t i = 0; i < p.size(); ++i)
	{
		if (p[i] == 0.0f)
		{
			if (q[i] < 0.0f)
				return false;
			continue;
		}

		float const t = q[i] / p[i];
		if (p[i] < 0.0f)
			t_min = std::max(t_min, t);
		else
			t_max = std::min(t_max, t);
		if (t_min > t_max)
			return false;
	}

	ImVec2 const start = a;
	a = ImVec2(start.x + t_min * dx, start.y + t_min * dy);
	b = ImVec2(start.x + t_max * dx, start.y + t_max * dy);
	return true;
}

double nice_distance_step(double target)
{
	if (!(target > 0.0) || !std::isfinite(target))
		return 1.0;

	double const magnitude = std::pow(10.0, std::floor(std::log10(target)));
	double const normalized = target / magnitude;
	return (normalized <= 1.0 ? 1.0 : normalized <= 2.0 ? 2.0 : normalized <= 5.0 ? 5.0 : 10.0) * magnitude;
}

ImU32 speed_limit_color(double speed)
{
	if (speed <= 40.0)
		return IM_COL32(245, 85, 75, 235);
	if (speed <= 80.0)
		return IM_COL32(245, 165, 65, 235);
	if (speed <= 120.0)
		return IM_COL32(235, 220, 75, 235);
	return IM_COL32(100, 220, 125, 235);
}

}

ui::map_panel::map_panel() : ui_panel(STR_C("Map"), false)
{
	size_min = {200, 200};
	size_max = {fb_size, fb_size};
	window_flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
	init_done = true;
}

float ui::map_panel::get_vehicle_rotation()
{
	const TDynamicObject *vehicle = simulation::Train->Dynamic();
	glm::vec3 front = glm::dvec3(vehicle->VectorFront()) * (vehicle->DirectionGet() > 0 ? 1.0 : -1.0);
	glm::vec2 north_ptr(0.0f, 1.0f);
	return glm::atan(front.z, front.x) - glm::atan(north_ptr.y, north_ptr.x);
}

void ui::map_panel::collect_map_geometry(glm::mat4 transform)
{
	cFrustum frustum;
	frustum.calculate(transform, glm::mat4());

	m_colored_paths.switches.clear();
	m_colored_paths.occupied.clear();
	m_colored_paths.future.clear();
	m_colored_paths.roads.clear();
    m_colored_paths.highlighted.clear();
	m_section_handles.clear();

	for (int row = 0; row < scene::EU07_REGIONSIDESECTIONCOUNT; row++)
	{
		for (int column = 0; column < scene::EU07_REGIONSIDESECTIONCOUNT; column++)
		{
			scene::basic_section *section = simulation::Region->get_section(row * scene::EU07_REGIONSIDESECTIONCOUNT + column);
			if (section && frustum.sphere_inside(section->area().center, section->area().radius) > 0.f)
			{
				const gfx::geometrybank_handle handle = section->get_map_geometry();
				if (handle != null_handle)
					m_section_handles.push_back(handle);
				section->get_map_active_paths(m_colored_paths);
			}
		}
	}

	for (TTrack *track : simulation::Paths.sequence()) {
		track->get_map_future_paths(m_colored_paths);
	}

    for (std::pair<TTrack*, int> &entry : highlighted_switches) {
        if (entry.first)
            entry.first->get_map_paths_for_state(m_colored_paths, entry.second);
    }
}

void ui::map_panel::render_grid(ImDrawList *draw_list, ImVec2 origin, glm::vec2 surface_size, glm::mat4 transform)
{
	const auto to_screen = [&](glm::dvec3 const &position) {
		glm::vec4 const ndc = transform * glm::vec4(glm::vec3(position), 1.0f);
		return ImVec2(origin.x + (ndc.x + 1.0f) * 0.5f * surface_size.x,
		              origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y);
	};
	const auto world_at = [&](float x, float z) {
		glm::vec4 const position = glm::inverse(transform) * glm::vec4(x, 0.0f, z, 1.0f);
		return glm::dvec2(position.x, position.z);
	};

	glm::dvec2 const world_x = world_at(1.0f, 0.0f) - world_at(0.0f, 0.0f);
	glm::dvec2 const world_z = world_at(0.0f, 1.0f) - world_at(0.0f, 0.0f);
	ImVec2 const screen_origin = to_screen(glm::dvec3(0.0));
	ImVec2 const screen_x = to_screen(glm::dvec3(world_x.x, 0.0, world_x.y));
	ImVec2 const screen_z = to_screen(glm::dvec3(world_z.x, 0.0, world_z.y));
	float const x_pixels_per_meter = glm::length(glm::vec2(screen_x.x - screen_origin.x, screen_x.y - screen_origin.y));
	float const z_pixels_per_meter = glm::length(glm::vec2(screen_z.x - screen_origin.x, screen_z.y - screen_origin.y));
	float const pixels_per_meter = std::min(x_pixels_per_meter, z_pixels_per_meter);
	if (!(pixels_per_meter > 0.0f) || !std::isfinite(pixels_per_meter))
		return;

	double const spacing = nice_distance_step(std::max(100.0, 70.0 / pixels_per_meter));
	std::array<glm::dvec2, 4> corners{
		world_at(-1.0f, -1.0f), world_at(-1.0f, 1.0f),
		world_at(1.0f, -1.0f), world_at(1.0f, 1.0f)};
	glm::dvec2 minimum(std::numeric_limits<double>::max());
	glm::dvec2 maximum(std::numeric_limits<double>::lowest());
	for (auto const &corner : corners)
	{
		minimum = glm::min(minimum, corner);
		maximum = glm::max(maximum, corner);
	}

	ImU32 const color = IM_COL32(110, 125, 135, 55);
	for (double x = std::floor(minimum.x / spacing) * spacing; x <= maximum.x; x += spacing)
	{
		ImVec2 a = to_screen(glm::dvec3(x, 0.0, minimum.y));
		ImVec2 b = to_screen(glm::dvec3(x, 0.0, maximum.y));
		if (map_has_vertex_budget(draw_list) && clip_map_line(a, b, origin, surface_size))
			draw_list->AddLine(a, b, color);
	}
	for (double z = std::floor(minimum.y / spacing) * spacing; z <= maximum.y; z += spacing)
	{
		ImVec2 a = to_screen(glm::dvec3(minimum.x, 0.0, z));
		ImVec2 b = to_screen(glm::dvec3(maximum.x, 0.0, z));
		if (map_has_vertex_budget(draw_list) && clip_map_line(a, b, origin, surface_size))
			draw_list->AddLine(a, b, color);
	}
}

void ui::map_panel::render_map_lines(ImDrawList *draw_list, ImVec2 origin, glm::vec2 surface_size, glm::mat4 transform)
{
	const auto to_screen = [&](glm::vec3 const &position) {
		glm::vec4 ndc = transform * glm::vec4(position, 1.0f);
		return ImVec2(origin.x + (ndc.x + 1.0f) * 0.5f * surface_size.x,
		              origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y);
	};

	const auto draw_geometry = [&](gfx::geometrybank_handle const &handle, ImU32 color, float thickness = 1.0f) {
		if (handle == null_handle || !map_has_vertex_budget(draw_list))
			return;

		auto const &vertices = GfxRenderer->Vertices(handle);
		for (std::size_t idx = 0; idx + 1 < vertices.size(); idx += 2)
		{
			if (!map_has_vertex_budget(draw_list))
				break;
			ImVec2 a = to_screen(vertices[idx].position);
			ImVec2 b = to_screen(vertices[idx + 1].position);
			if (clip_map_line(a, b, origin, surface_size))
				draw_list->AddLine(a, b, color, thickness);
		}
	};

	for (auto const &body : EditorTerrain.water())
	{
		if (body.outline.size() < 3 || !map_has_vertex_budget(draw_list, 2048))
			continue;

		std::vector<ImVec2> outline;
		outline.reserve(std::min<std::size_t>(body.outline.size(), 256));
		for (auto const &point : body.outline)
		{
			ImVec2 const screen = to_screen(glm::vec3(point.x, body.level, point.y));
			if (outline.empty() || glm::length(glm::vec2(screen.x - outline.back().x, screen.y - outline.back().y)) >= 3.0f)
				outline.push_back(screen);
		}
		if (outline.size() > 256)
		{
			std::vector<ImVec2> reduced;
			reduced.reserve(256);
			for (std::size_t i = 0; i < 256; ++i)
				reduced.push_back(outline[i * outline.size() / 256]);
			outline = std::move(reduced);
		}

		if (outline.size() < 3)
			continue;

		ImVec2 minimum(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		ImVec2 maximum(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
		for (auto const &point : outline)
		{
			minimum.x = std::min(minimum.x, point.x);
			minimum.y = std::min(minimum.y, point.y);
			maximum.x = std::max(maximum.x, point.x);
			maximum.y = std::max(maximum.y, point.y);
		}
		if (maximum.x < origin.x || minimum.x > origin.x + surface_size.x ||
		    maximum.y < origin.y || minimum.y > origin.y + surface_size.y)
			continue;

		int const estimated_fill_vertices = static_cast<int>(outline.size()) * 6;
		if (map_has_vertex_budget(draw_list, estimated_fill_vertices))
			draw_list->AddConcavePolyFilled(outline.data(), static_cast<int>(outline.size()), IM_COL32(35, 115, 165, 80));

		for (std::size_t idx = 0; idx < outline.size() && map_has_vertex_budget(draw_list); ++idx)
		{
			ImVec2 a = outline[idx];
			ImVec2 b = outline[(idx + 1) % outline.size()];
			if (clip_map_line(a, b, origin, surface_size))
				draw_list->AddLine(a, b, IM_COL32(80, 190, 230, 230), 2.0f);
		}
	}

	float const pixels_per_meter = glm::length(glm::vec2(
		to_screen(glm::vec3(1.0f, 0.0f, 0.0f)).x - to_screen(glm::vec3(0.0f)).x,
		to_screen(glm::vec3(1.0f, 0.0f, 0.0f)).y - to_screen(glm::vec3(0.0f)).y));
	double const river_step = std::clamp(4.0 / std::max(static_cast<double>(pixels_per_meter), 1e-6), 5.0, 50.0);
	for (TTrack *track : simulation::Paths.sequence())
	{
		if (!track || (track->iCategoryFlag & 15) != 4 || !track->Segment)
			continue;

		TSegment const &segment = *track->Segment;
		double const length = segment.GetLength();
		if (!(length > 0.0))
			continue;

		double const half_width = 0.5 * std::abs(track->fTrackWidth);
		glm::dvec3 previous_center, previous_across;
		segment.plan_frame(0.0, previous_center, previous_across);
		glm::dvec3 previous_left = previous_center + previous_across * half_width;
		glm::dvec3 previous_right = previous_center - previous_across * half_width;
		for (double distance = std::min(river_step, length);; distance = std::min(distance + river_step, length))
		{
			if (!map_has_vertex_budget(draw_list))
				break;
			glm::dvec3 center, across;
			segment.plan_frame(distance, center, across);
			glm::dvec3 const left = center + across * half_width;
			glm::dvec3 const right = center - across * half_width;
			ImVec2 left_start = to_screen(glm::vec3(previous_left));
			ImVec2 left_end = to_screen(glm::vec3(left));
			if (clip_map_line(left_start, left_end, origin, surface_size))
				draw_list->AddLine(left_start, left_end, IM_COL32(80, 190, 230, 230), 1.5f);
			ImVec2 right_start = to_screen(glm::vec3(previous_right));
			ImVec2 right_end = to_screen(glm::vec3(right));
			if (map_has_vertex_budget(draw_list) && clip_map_line(right_start, right_end, origin, surface_size))
				draw_list->AddLine(right_start, right_end, IM_COL32(80, 190, 230, 230), 1.5f);
			previous_left = left;
			previous_right = right;
			if (distance >= length)
				break;
		}
	}

	for (auto const &handle : m_section_handles)
		draw_geometry(handle, IM_COL32(190, 195, 200, 255));

	if (m_show_speed_limits)
	{
		for (TTrack *track : simulation::Paths.sequence())
		{
			if (!track || track->iCategoryFlag != 1 || track->VelocityGet() < 0.0)
				continue;

			ImU32 const color = speed_limit_color(track->VelocityGet());
			if (track->eType == tt_Switch && track->SwitchExtension)
			{
				int const state = track->GetSwitchState();
				if (state >= 0 && state < 2)
					draw_geometry(track->SwitchExtension->map_geometry[state], color, 3.0f);
			}
			else
				draw_geometry(track->extra_map_geometry, color, 3.0f);
		}
	}

	for (TTrack *track : simulation::Paths.sequence())
	{
		if (!track || track->iCategoryFlag != 1)
			continue;

		ImU32 color;
		if (track->eEnvironment == e_tunnel)
			color = IM_COL32(190, 125, 235, 230);
		else if (track->eEnvironment == e_bridge)
			color = IM_COL32(70, 205, 225, 230);
		else
			continue;

		const auto draw_segment = [&](TSegment const *segment) {
			if (!segment)
				return;
			std::vector<gfx::basic_vertex> vertices;
			segment->render_lines(vertices, 0.5f);
			for (std::size_t idx = 0; idx + 1 < vertices.size(); idx += 2)
			{
				if (!map_has_vertex_budget(draw_list))
					break;
				ImVec2 a = to_screen(vertices[idx].position);
				ImVec2 b = to_screen(vertices[idx + 1].position);
				if (clip_map_line(a, b, origin, surface_size))
					draw_list->AddLine(a, b, color, 3.0f);
			}
		};

		if (track->eType == tt_Switch && track->SwitchExtension)
		{
			draw_segment(track->SwitchExtension->Segments[0].get());
			draw_segment(track->SwitchExtension->Segments[1].get());
		}
		else if (track->eType == tt_Normal)
			draw_segment(track->Segment.get());
	}

	for (auto const &handle : m_colored_paths.roads)
		draw_geometry(handle, IM_COL32(235, 165, 65, 255));
	for (auto const &handle : m_colored_paths.future)
		draw_geometry(handle, IM_COL32(180, 180, 65, 255));
	for (auto const &handle : m_colored_paths.switches)
		draw_geometry(handle, IM_COL32(0, 255, 0, 255));
	for (auto const &handle : m_colored_paths.occupied)
		draw_geometry(handle, IM_COL32(255, 0, 0, 255));
	for (auto const &handle : m_colored_paths.highlighted)
		draw_geometry(handle, IM_COL32(75, 75, 255, 255));

	for (auto const &object : map::Objects.entries)
	{
		if (!map_has_vertex_budget(draw_list, 16))
			break;
		ImU32 color = IM_COL32(255, 220, 70, 255);
		if (std::dynamic_pointer_cast<map::launcher>(object))
			color = IM_COL32(255, 150, 50, 255);
		else if (std::dynamic_pointer_cast<map::track_switch>(object))
			color = IM_COL32(80, 190, 255, 255);
		else if (std::dynamic_pointer_cast<map::obstacle>(object))
			color = IM_COL32(255, 80, 80, 255);
		draw_list->AddCircleFilled(to_screen(object->location), 3.0f, color);
	}

}

void ui::map_panel::render_labels(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size)
{
	ImDrawList *draw_list = ImGui::GetWindowDrawList();
	std::vector<std::string> station_names;
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.85f, 0.9f, 1.0f, 1.0f));
	for (basic_event *event : simulation::Events.sequence())
	{
		if (!event || event->input_command() != TCommandType::cm_PassengerStopPoint)
			continue;

		std::string name = event->input_text();
		constexpr std::string_view stop_point_prefix = "PassengerStopPoint:";
		if (!name.starts_with(stop_point_prefix))
			continue;

		name = Bezogonkow(name.substr(stop_point_prefix.size()), true);
		if (name.empty() || std::find(station_names.begin(), station_names.end(), name) != station_names.end())
			continue;

		glm::vec4 ndc_pos = transform * glm::vec4(glm::vec3(event->input_location()), 1.0f);
		if (glm::abs(ndc_pos.x) > 1.0f || glm::abs(ndc_pos.z) > 1.0f)
			continue;
		if (!map_has_vertex_budget(draw_list, static_cast<int>(name.size() * 8 + 16)))
			break;
		station_names.push_back(name);

		ImVec2 const text_size = ImGui::CalcTextSize(name.c_str());
		glm::vec2 const gui_pos =
			(glm::vec2(ndc_pos.x, -ndc_pos.z) / 2.0f + 0.5f) * glm::vec2(surface_size.x, surface_size.y);
		ImGui::SetCursorPos(ImVec2(
			origin.x + gui_pos.x - text_size.x / 2.0f,
			origin.y + gui_pos.y - text_size.y / 2.0f));
		ImGui::TextUnformatted(name.c_str());
	}
	ImGui::PopStyleColor();

	ImVec2 const window_position = ImGui::GetWindowPos();
	ImVec2 const map_screen_origin(window_position.x + origin.x, window_position.y + origin.y);
	draw_list->PushClipRect(
		map_screen_origin,
		ImVec2(map_screen_origin.x + surface_size.x, map_screen_origin.y + surface_size.y),
		true);
	for (sweep_node *sweep : simulation::Sweeps.sequence())
	{
		if (!map_has_vertex_budget(draw_list))
			break;
		if (!sweep || !sweep->definition().platform)
			continue;

		glm::dvec3 low, high, shift;
		if (!sweep->bounds(low, high, shift))
			continue;

		auto const &definition = sweep->definition();
		double const near_side = definition.lateral >= 0.0 ? low.y : high.y;
		double const far_side = definition.lateral >= 0.0 ? high.y : low.y;
		double const start = std::clamp(sweep->start(), 0.0, sweep->length());
		double const end = std::clamp(sweep->end(), start, sweep->length());
		if (end - start < 0.1)
			continue;

		auto const edge_position = [&](double distance, double side) {
			auto const frame = sweep->frame_at(distance);
			double const offset = definition.lateral + sweep->setback(frame) + side + shift.y;
			return frame.position + frame.left * offset;
		};
		auto const to_screen = [&](glm::dvec3 const &position) {
			glm::vec4 const ndc = transform * glm::vec4(glm::vec3(position), 1.0f);
			return ImVec2(
				map_screen_origin.x + (ndc.x + 1.0f) * 0.5f * surface_size.x,
				map_screen_origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y);
		};

		double distance = start;
		auto near_previous = edge_position(distance, near_side);
		auto far_previous = edge_position(distance, far_side);
		while (distance < end)
		{
			if (!map_has_vertex_budget(draw_list, 16))
				break;
			distance = std::min(distance + 3.0, end);
			auto const near_current = edge_position(distance, near_side);
			auto const far_current = edge_position(distance, far_side);
			ImVec2 near_a = to_screen(near_previous);
			ImVec2 near_b = to_screen(near_current);
			if (clip_map_line(near_a, near_b, map_screen_origin, surface_size))
				draw_list->AddLine(near_a, near_b, IM_COL32(255, 190, 90, 255), 1.5f);
			ImVec2 far_a = to_screen(far_previous);
			ImVec2 far_b = to_screen(far_current);
			if (map_has_vertex_budget(draw_list) && clip_map_line(far_a, far_b, map_screen_origin, surface_size))
				draw_list->AddLine(far_a, far_b, IM_COL32(220, 155, 75, 220), 1.0f);
			near_previous = near_current;
			far_previous = far_current;
		}
		ImVec2 start_a = to_screen(edge_position(start, near_side));
		ImVec2 start_b = to_screen(edge_position(start, far_side));
		if (map_has_vertex_budget(draw_list) && clip_map_line(start_a, start_b, map_screen_origin, surface_size))
			draw_list->AddLine(start_a, start_b, IM_COL32(255, 225, 160, 255), 1.0f);
		ImVec2 end_a = to_screen(near_previous);
		ImVec2 end_b = to_screen(far_previous);
		if (map_has_vertex_budget(draw_list) && clip_map_line(end_a, end_b, map_screen_origin, surface_size))
			draw_list->AddLine(end_a, end_b, IM_COL32(255, 225, 160, 255), 1.0f);
	}
	draw_list->PopClipRect();

	for (TDynamicObject *vehicle : simulation::Vehicles.sequence())
	{
		if (!map_has_vertex_budget(draw_list, 16))
			break;
		if (!vehicle->MyTrack || (vehicle->MyTrack->iCategoryFlag & 15) != 1)
			continue;

		glm::vec4 ndc_pos = transform * glm::vec4(glm::vec3(vehicle->GetPosition()), 1.0f);
		if (glm::abs(ndc_pos.x) > 1.0f || glm::abs(ndc_pos.z) > 1.0f)
			continue;

		bool const is_train_front = vehicle->Prev() == nullptr;
		glm::dvec3 front = vehicle->VectorFront();
		front.y = 0.0;
		double const front_length = glm::length(front);
		if (!(front_length > 0.0) || !std::isfinite(front_length))
			continue;
		front /= front_length;
		glm::dvec3 const side(-front.z, 0.0, front.x);
		double const half_length = vehicle->GetLength() * 0.5;
		double const half_width = vehicle->GetWidth() * 0.5;
		if (!(half_length > 0.0) || !(half_width > 0.0)
		    || !std::isfinite(half_length) || !std::isfinite(half_width))
			continue;

		auto const to_screen = [transform, map_screen_origin, surface_size](glm::dvec3 const &position) {
			glm::vec4 const point = transform * glm::vec4(glm::vec3(position), 1.0f);
			return ImVec2(
			    map_screen_origin.x + (point.x + 1.0f) * 0.5f * surface_size.x,
			    map_screen_origin.y + (1.0f - point.z) * 0.5f * surface_size.y);
		};
		glm::dvec3 const center = vehicle->GetPosition();
		ImVec2 const front_left = to_screen(center + front * half_length + side * half_width);
		ImVec2 const rear_left = to_screen(center - front * half_length + side * half_width);
		ImVec2 const rear_right = to_screen(center - front * half_length - side * half_width);
		ImVec2 const front_right = to_screen(center + front * half_length - side * half_width);
		ImU32 const fill = is_train_front ? IM_COL32(130, 255, 150, 255) : IM_COL32(100, 220, 125, 255);
		draw_list->AddQuadFilled(front_left, rear_left, rear_right, front_right, fill);
		draw_list->AddQuad(front_left, rear_left, rear_right, front_right, IM_COL32(20, 35, 25, 255), 1.0f);
	}

	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 1.0f, 0.7f, 1.0f));
	for (TDynamicObject *vehicle : simulation::Vehicles.sequence())
	{
		if (!vehicle->MyTrack || (vehicle->MyTrack->iCategoryFlag & 15) != 1 || vehicle->Prev() || !vehicle->Mechanik)
			continue;

		std::string label = vehicle->Mechanik->TrainName();
		if (label.empty() || label == "none")
			label = ToUpper(vehicle->name());

		glm::vec4 ndc_pos = transform * glm::vec4(glm::vec3(vehicle->GetPosition()), 1.0f);
		if (glm::abs(ndc_pos.x) > 1.0f || glm::abs(ndc_pos.z) > 1.0f)
			continue;

		glm::vec2 gui_pos = (glm::vec2(ndc_pos.x, -ndc_pos.z) / 2.0f + 0.5f) * glm::vec2(surface_size.x, surface_size.y);

		ImVec2 textsize = ImGui::CalcTextSize(label.c_str());
		ImGui::SetCursorPos(ImVec2(origin.x + gui_pos.x - textsize.x / 2.0f, origin.y + gui_pos.y - textsize.y / 2.0f));
		ImGui::TextUnformatted(label.c_str());
	}
	ImGui::PopStyleColor();
}

void ui::map_panel::render_contents()
{
    if (!init_done) {
        is_open = false;
        return;
    }

	bool const zoom_in = ImGui::Button("+");
	ImGui::SameLine();
	bool const zoom_out = ImGui::Button("-");
	ImGui::SameLine();

	if (!simulation::Train && mode == MODE_VEHICLE)
		mode = MODE_MANUAL;

	glm::mat4 transform;
	transform[0][0] = -1.0f;

    ImGui::RadioButton(STR_C("Pan"), (int *)&mode, 0);
	ImGui::SameLine();
    ImGui::RadioButton(STR_C("Follow camera"), (int *)&mode, 1);
	ImGui::SameLine();
	ImGui::BeginDisabled(simulation::Train == nullptr);
    ImGui::RadioButton(STR_C("Follow vehicle"), (int *)&mode, 2);
	ImGui::EndDisabled();
	ImGui::SameLine();
	ImGui::Checkbox(STR_C("Speed limits"), &m_show_speed_limits);
	if (simulation::Train)
	{
		ImGui::SameLine();
		ImGui::Text("%.0f km/h", std::abs(simulation::Train->Dynamic()->GetVelocity()));
	}
	if (m_show_speed_limits)
	{
		ImGui::TextColored(ImVec4(0.96f, 0.33f, 0.29f, 1.0f), "<=40");
		ImGui::SameLine();
		ImGui::TextColored(ImVec4(0.96f, 0.65f, 0.25f, 1.0f), "<=80");
		ImGui::SameLine();
		ImGui::TextColored(ImVec4(0.92f, 0.86f, 0.29f, 1.0f), "<=120");
		ImGui::SameLine();
		ImGui::TextColored(ImVec4(0.39f, 0.86f, 0.49f, 1.0f), ">120 km/h");
	}

	ImVec2 surface_size_im = ImGui::GetContentRegionAvail();
	glm::vec2 surface_size(surface_size_im.x, surface_size_im.y);

	glm::mat4 aspect_transform;
	float aspect = surface_size.y / surface_size.x;

	if (aspect > 1.0f / aspect)
		aspect_transform = glm::scale(glm::mat4(), glm::vec3(aspect, 1.0f, 1.0f));
	else
		aspect_transform = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f / aspect));

	transform *= aspect_transform;

	TDynamicObject const *followed_vehicle = nullptr;
	glm::dvec3 follow_position{};
	glm::dvec3 follow_direction{};
	if (mode == MODE_VEHICLE && simulation::Train)
	{
		followed_vehicle = simulation::Train->Dynamic();
		float rotation = get_vehicle_rotation();

		transform = glm::rotate(transform, rotation, glm::vec3(0.0f, 1.0f, 0.0f));

		follow_position = followed_vehicle->GetPosition();
		follow_direction = followed_vehicle->VectorFront() * (followed_vehicle->DirectionGet() > 0 ? 1.0 : -1.0);
		double const direction_length = glm::length(follow_direction);
		if (direction_length > 0.0)
			follow_direction /= direction_length;
	}
	if (mode == MODE_CAMERA)
	{
		float initial_rotation;
		if (!FreeFlyModeFlag)
			initial_rotation = get_vehicle_rotation();
		else
			initial_rotation = glm::pi<float>();

		float rotation = initial_rotation - Global.pCamera.Angle.y;

		transform = glm::rotate(transform, rotation, glm::vec3(0.0f, 1.0f, 0.0f));

		glm::dvec3 position = Global.pCamera.Pos;
		translate = glm::vec2(position.x, position.z) * -zoom;
	}

	glm::vec4 const screen_x = transform * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
	float const base_pixels_per_meter = glm::length(glm::vec2(
		screen_x.x * surface_size.x * 0.5f,
		screen_x.z * surface_size.y * -0.5f));
	if (base_pixels_per_meter > 0.0f && std::isfinite(base_pixels_per_meter))
	{
		float const min_zoom = (110.0f / 2000.0f) / base_pixels_per_meter;
		float const max_zoom = (110.0f / 50.0f) / base_pixels_per_meter;
		float const previous_zoom = zoom;
		zoom = std::clamp(zoom, min_zoom, max_zoom);
		if (zoom_in)
			zoom = std::min(zoom * 1.25f, max_zoom);
		else if (zoom_out)
			zoom = std::max(zoom / 1.25f, min_zoom);
		translate *= zoom / previous_zoom;
	}

	if (followed_vehicle)
	{
		double lookahead = 0.0;
		double const direction_length = glm::length(follow_direction);
		if (direction_length > 0.0)
		{
			glm::vec4 const screen_direction = transform * glm::vec4(glm::vec3(follow_direction), 0.0f);
			double max_lookahead = std::numeric_limits<double>::infinity();
			double const screen_margin = std::min(16.0, 0.1 * std::min(surface_size.x, surface_size.y));
			double const available_width = std::max(0.0, surface_size.x * 0.5 - screen_margin);
			double const available_height = std::max(0.0, surface_size.y * 0.5 - screen_margin);
			double const pixels_per_meter_x = std::abs(screen_direction.x) * surface_size.x * 0.5 * zoom;
			double const pixels_per_meter_y = std::abs(screen_direction.z) * surface_size.y * 0.5 * zoom;
			if (pixels_per_meter_x > 1e-6)
				max_lookahead = std::min(max_lookahead, available_width / pixels_per_meter_x);
			if (pixels_per_meter_y > 1e-6)
				max_lookahead = std::min(max_lookahead, available_height / pixels_per_meter_y);

			double const desired_lookahead = std::abs(followed_vehicle->GetVelocity()) * 10.0;
			lookahead = std::min(desired_lookahead, max_lookahead);
		}
		follow_position += follow_direction * lookahead;
		translate = glm::vec2(follow_position.x, follow_position.z) * -zoom;
	}

	transform = glm::translate(transform, glm::vec3(translate.x, 0.0f, translate.y));
	transform = glm::scale(transform, glm::vec3(zoom));

	collect_map_geometry(transform);

	ImVec2 window_origin = ImGui::GetCursorPos();
	ImVec2 screen_origin = ImGui::GetCursorScreenPos();

	ImDrawList *draw_list = ImGui::GetWindowDrawList();
	draw_list->PushClipRect(screen_origin, ImVec2(screen_origin.x + surface_size.x, screen_origin.y + surface_size.y), true);
	draw_list->AddRectFilled(
		screen_origin,
		ImVec2(screen_origin.x + surface_size.x, screen_origin.y + surface_size.y),
		IM_COL32(18, 22, 26, 255));
	render_grid(draw_list, screen_origin, surface_size, transform);
	render_map_lines(draw_list, screen_origin, surface_size, transform);
	if (map_has_vertex_budget(draw_list, 64))
	{
		TCamera const &camera = Global.pCamera;
		double yaw = camera.Angle.y;
		if (camera.m_owner != nullptr)
		{
			glm::dvec3 owner_forward = camera.m_owner->VectorFront();
			if (simulation::Train && simulation::Train->Dynamic() == camera.m_owner && simulation::Train->Occupied())
				owner_forward *= simulation::Train->Occupied()->CabOccupied < 0 ? -1.0 : 1.0;
			yaw += std::atan2(-owner_forward.x, -owner_forward.z);
		}
		glm::dvec3 forward(-std::sin(yaw), 0.0, -std::cos(yaw));
		double const forward_length = glm::length(forward);
		glm::vec4 const camera_ndc = transform * glm::vec4(glm::vec3(camera.Pos), 1.0f);
		if (forward_length > 0.0 && glm::abs(camera_ndc.x) <= 1.0f && glm::abs(camera_ndc.z) <= 1.0f)
		{
			forward /= forward_length;
			glm::dvec3 const side(-forward.z, 0.0, forward.x);
			double const half_fov = glm::radians(
				static_cast<double>(Global.FieldOfView / std::max(Global.ZoomFactor, 0.01f))) * 0.5;
			double const view_length = 300.0;
			glm::dvec3 const left = camera.Pos + (forward * std::cos(half_fov) - side * std::sin(half_fov)) * view_length;
			glm::dvec3 const right = camera.Pos + (forward * std::cos(half_fov) + side * std::sin(half_fov)) * view_length;
			auto const to_screen = [&](glm::dvec3 const &position) {
				glm::vec4 const ndc = transform * glm::vec4(glm::vec3(position), 1.0f);
				return ImVec2(
					screen_origin.x + (ndc.x + 1.0f) * 0.5f * surface_size.x,
					screen_origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y);
			};
			ImVec2 const origin = to_screen(camera.Pos);
			ImVec2 const left_screen = to_screen(left);
			ImVec2 const right_screen = to_screen(right);
			draw_list->AddTriangleFilled(origin, left_screen, right_screen, IM_COL32(100, 190, 255, 35));
			draw_list->AddLine(origin, left_screen, IM_COL32(100, 190, 255, 160), 1.0f);
			draw_list->AddLine(origin, right_screen, IM_COL32(100, 190, 255, 160), 1.0f);
			draw_list->AddCircleFilled(origin, 5.0f, IM_COL32(100, 190, 255, 255));
			draw_list->AddCircle(origin, 5.0f, IM_COL32(15, 25, 35, 255), 0, 1.5f);
			ImVec2 const tip = to_screen(camera.Pos + forward * 9.0);
			draw_list->AddLine(origin, tip, IM_COL32(15, 25, 35, 255), 2.0f);
		}
	}
	draw_list->PopClipRect();
	ImGui::InvisibleButton("##map", surface_size_im);

	if (m_camera_click_pending && ImGui::GetTime() >= m_camera_click_deadline)
	{
		Global.FreeCameraInit[0] = m_camera_click_position;
		Global.FreeCameraTeleport = m_camera_click_position;
		Global.pCamera.Pos = m_camera_click_position;
		m_camera_click_pending = false;
	}

	if (ImGui::IsItemHovered())
	{
		const ImVec2 screen_pos = ImGui::GetMousePos();
		const glm::vec2 surface_pos(screen_pos.x - screen_origin.x, screen_pos.y - screen_origin.y);
		const glm::vec2 ndc_pos = surface_pos / surface_size * 2.0f - 1.0f;
		glm::vec3 const world_pos = glm::inverse(transform) * glm::vec4(ndc_pos.x, 0.0f, -ndc_pos.y, 1.0f);
		const ImGuiIO &io = ImGui::GetIO();
		const auto relocate_camera = [&] {
			glm::vec3 const world_pos = glm::inverse(transform) * glm::vec4(ndc_pos.x, 0.0f, -ndc_pos.y, 1.0f);
			glm::dvec3 const camera_position(world_pos.x, Global.pCamera.Pos.y, world_pos.z);
			Global.FreeCameraInit[0] = camera_position;
			Global.FreeCameraTeleport = camera_position;
			Global.pCamera.Pos = camera_position;
		};
		if (ImGui::IsMouseDoubleClicked(0))
		{
			m_camera_click_pending = false;
			Global.FreeCameraMoveTarget = glm::dvec3(world_pos.x, Global.pCamera.Pos.y, world_pos.z);
		}

		if (io.MouseWheel != 0.0f &&
		    base_pixels_per_meter > 0.0f &&
		    std::isfinite(base_pixels_per_meter))
		{
			float const previous_zoom = zoom;
			float const min_zoom = (110.0f / 2000.0f) / base_pixels_per_meter;
			float const max_zoom = (110.0f / 50.0f) / base_pixels_per_meter;
			zoom = std::clamp(
				zoom * std::pow(1.05f, io.MouseWheel),
				min_zoom,
				max_zoom);
			translate += (previous_zoom - zoom) * glm::vec2(world_pos.x, world_pos.z);

			glm::vec2 surface_screen_center = glm::vec2(screen_origin.x, screen_origin.y) + surface_size / 2.0f;
			Application.set_cursor_pos(surface_screen_center.x, surface_screen_center.y);
		}

		if (FreeFlyModeFlag && Global.pCamera.m_owner == nullptr && ImGui::IsMouseClicked(2))
			relocate_camera();
		if (FreeFlyModeFlag && Global.pCamera.m_owner == nullptr
		    && ImGui::IsMouseReleased(0)
		    && io.MouseDragMaxDistanceSqr[0] < io.MouseDragThreshold * io.MouseDragThreshold)
		{
			m_camera_click_position = glm::dvec3(world_pos.x, Global.pCamera.Pos.y, world_pos.z);
			m_camera_click_deadline = ImGui::GetTime() + io.MouseDoubleClickTime;
			m_camera_click_pending = true;
		}
		if (mode == 0 && ImGui::IsMouseDragging(0))
		{
			ImVec2 delta_im = ImGui::GetMouseDragDelta();
			ImGui::ResetMouseDragDelta();

			glm::vec2 delta(delta_im.x, delta_im.y);

			translate -= delta / surface_size * 2.0f;
		}
		else
		{
			map::sorted_object_list objects = map::Objects.find_in_range(glm::vec3(world_pos.x, NAN, world_pos.z), 0.03f / zoom);

			if (ImGui::IsMouseClicked(1))
			{
				if (objects.size() > 1)
					register_popup(std::make_unique<ui::disambiguation_popup>(*this, std::move(objects)));
				else if (objects.size() == 1)
					handle_map_object_click(*this, objects.begin()->second);
				else
				{
					glm::vec3 nearest = simulation::Region->find_nearest_track_point(world_pos);
					if (!glm::isnan(nearest.x)
					        && glm::distance(glm::vec2(world_pos.x, world_pos.z), glm::vec2(nearest.x, nearest.z)) < 60.0f)
						register_popup(std::make_unique<obstacle_insert_window>(*this, std::move(nearest)));
				}
			}
			else if (!objects.empty())
			{
				handle_map_object_hover(objects.begin()->second);
			}
		}

		if (m_show_speed_limits)
		{
			TTrack *nearest_track = nullptr;
			float nearest_screen_distance = 8.0f;
			for (TTrack *track : simulation::Paths.sequence())
			{
				if (!track || track->iCategoryFlag != 1 || !track->Segment)
					continue;

				glm::vec3 const position = track->Segment->get_nearest_point(world_pos);
				glm::vec4 const track_ndc = transform * glm::vec4(position, 1.0f);
				float const dx = (track_ndc.x - ndc_pos.x) * 0.5f * surface_size.x;
				float const dy = (track_ndc.z - ndc_pos.y) * -0.5f * surface_size.y;
				float const screen_distance = std::sqrt(dx * dx + dy * dy);
				if (screen_distance < nearest_screen_distance)
				{
					nearest_screen_distance = screen_distance;
					nearest_track = track;
				}
			}

			if (nearest_track)
			{
				ImGui::BeginTooltip();
				ImGui::Text("%s", nearest_track->name().c_str());
				double const speed = nearest_track->VelocityGet();
				if (speed < 0.0)
					ImGui::TextUnformatted(STR_C("No posted speed limit"));
				else
					ImGui::Text("%.0f km/h", speed);
				ImGui::EndTooltip();
			}
		}
	}

	render_labels(transform, window_origin, surface_size);

	if (surface_size.x >= 40.0f && surface_size.y >= 40.0f)
	{
		const auto to_screen = [&](glm::dvec3 const &position) {
			glm::vec4 const ndc = transform * glm::vec4(glm::vec3(position), 1.0f);
			return ImVec2(
				screen_origin.x + (ndc.x + 1.0f) * 0.5f * surface_size.x,
				screen_origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y);
		};
		ImVec2 const scale_origin = to_screen(glm::dvec3(0.0));
		ImVec2 const scale_unit = to_screen(glm::dvec3(1.0, 0.0, 0.0));
		float const pixels_per_meter = glm::length(glm::vec2(scale_unit.x - scale_origin.x, scale_unit.y - scale_origin.y));
		if (pixels_per_meter > 0.0f && std::isfinite(pixels_per_meter))
		{
			double const scale_distance = nice_distance_step(110.0 / pixels_per_meter);
			float const scale_width = static_cast<float>(scale_distance * pixels_per_meter);
			char label[32];
			if (scale_distance >= 1000.0)
				std::snprintf(label, sizeof(label), "%.1f km", scale_distance / 1000.0);
			else if (scale_distance >= 10.0)
				std::snprintf(label, sizeof(label), "%.0f m", scale_distance);
			else
				std::snprintf(label, sizeof(label), "%.1f m", scale_distance);

			ImVec2 const text_size = ImGui::CalcTextSize(label);
			ImVec2 const scale_start(screen_origin.x + 12.0f, screen_origin.y + surface_size.y - 16.0f);
			float const scale_y = scale_start.y - text_size.y - 7.0f;
			ImDrawList *draw_list = ImGui::GetWindowDrawList();
			draw_list->PushClipRect(screen_origin, ImVec2(screen_origin.x + surface_size.x, screen_origin.y + surface_size.y), true);
			draw_list->AddRectFilled(
				ImVec2(scale_start.x - 6.0f, scale_y - 4.0f),
				ImVec2(scale_start.x + scale_width + 6.0f, scale_start.y + 5.0f),
				IM_COL32(18, 22, 26, 190), 3.0f);
			draw_list->AddLine(
				ImVec2(scale_start.x, scale_y),
				ImVec2(scale_start.x + scale_width, scale_y),
				IM_COL32(245, 245, 245, 230), 2.0f);
			draw_list->AddLine(
				ImVec2(scale_start.x, scale_y - 3.0f),
				ImVec2(scale_start.x, scale_y + 3.0f),
				IM_COL32(245, 245, 245, 230), 2.0f);
			draw_list->AddLine(
				ImVec2(scale_start.x + scale_width, scale_y - 3.0f),
				ImVec2(scale_start.x + scale_width, scale_y + 3.0f),
				IM_COL32(245, 245, 245, 230), 2.0f);
			draw_list->AddText(
				ImVec2(scale_start.x, scale_y - text_size.y - 2.0f),
				IM_COL32(245, 245, 245, 255), label);

			ImVec2 const north_origin = to_screen(glm::dvec3(0.0));
			ImVec2 const north_unit = to_screen(glm::dvec3(0.0, 0.0, 1.0));
			glm::vec2 north_direction(north_unit.x - north_origin.x, north_unit.y - north_origin.y);
			float const north_length = glm::length(north_direction);
			if (north_length > 0.0f && std::isfinite(north_length))
			{
				north_direction /= north_length;
				glm::vec2 const perpendicular(-north_direction.y, north_direction.x);
				ImVec2 const compass_center(
					screen_origin.x + surface_size.x - 34.0f,
					screen_origin.y + surface_size.y - 34.0f);
				glm::vec2 const center(compass_center.x, compass_center.y);
				glm::vec2 const tip = center + north_direction * 11.0f;
				glm::vec2 const base = center - north_direction * 7.0f;
				draw_list->AddRectFilled(
					ImVec2(compass_center.x - 26.0f, compass_center.y - 26.0f),
					ImVec2(compass_center.x + 26.0f, compass_center.y + 26.0f),
					IM_COL32(18, 22, 26, 190), 5.0f);
				draw_list->AddLine(
					ImVec2(base.x, base.y),
					ImVec2(tip.x, tip.y),
					IM_COL32(245, 245, 245, 235), 2.0f);
				draw_list->AddTriangleFilled(
					ImVec2(tip.x + north_direction.x * 4.0f, tip.y + north_direction.y * 4.0f),
					ImVec2(tip.x - north_direction.x * 3.0f + perpendicular.x * 4.0f,
					       tip.y - north_direction.y * 3.0f + perpendicular.y * 4.0f),
					ImVec2(tip.x - north_direction.x * 3.0f - perpendicular.x * 4.0f,
					       tip.y - north_direction.y * 3.0f - perpendicular.y * 4.0f),
					IM_COL32(255, 95, 85, 255));
				ImVec2 const north_label(
					compass_center.x + north_direction.x * 19.0f - ImGui::CalcTextSize("N").x * 0.5f,
					compass_center.y + north_direction.y * 19.0f - ImGui::CalcTextSize("N").y * 0.5f);
				draw_list->AddText(north_label, IM_COL32(245, 245, 245, 255), "N");
			}
			draw_list->PopClipRect();
		}
	}
}

void ui::handle_map_object_click(ui_panel &parent, std::shared_ptr<map::map_object> &obj)
{
	if (auto sem = std::dynamic_pointer_cast<map::semaphore>(obj))
	{
		parent.register_popup(std::make_unique<semaphore_window>(parent, std::move(sem)));
	}
	else if (auto track = std::dynamic_pointer_cast<map::launcher>(obj))
	{
		parent.register_popup(std::make_unique<launcher_window>(parent, std::move(track)));
	}
    else if (auto track_switch = std::dynamic_pointer_cast<map::track_switch>(obj))
    {
        parent.register_popup(std::make_unique<track_switch_window>(parent, std::move(track_switch)));
    }
	else if (auto obstacle = std::dynamic_pointer_cast<map::obstacle>(obj))
	{
		parent.register_popup(std::make_unique<obstacle_remove_window>(parent, std::move(obstacle)));
	}
	else if (auto obstacle = std::dynamic_pointer_cast<map::vehicle>(obj))
	{
		parent.register_popup(std::make_unique<vehicle_click_window>(parent, std::move(obstacle)));
	}
}

void ui::handle_map_object_hover(std::shared_ptr<map::map_object> &obj)
{
	ImGui::BeginTooltip();

	if (auto sem = std::dynamic_pointer_cast<map::semaphore>(obj))
	{
		for (auto &model : sem->models)
		{
			ImGui::PushID(model);
			ImGui::TextUnformatted(model->name().c_str());

			for (int i = 0; i < iMaxNumLights; i++)
			{
				auto state = model->LightGet(i);
				if (!state)
					continue;

				glm::vec3 current_color = std::get<2>(*state).value_or(glm::vec3(0.5f)) * std::get<1>(*state);
				if (std::get<0>(*state) < 0.0f)
					continue;

				ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(current_color.x, current_color.y, current_color.z, 1.0f));

				std::string res = "  ##" + std::to_string(i);
				ImGui::Button(res.c_str());

				ImGui::PopStyleColor();
			}

			ImGui::PopID();
		}
	}
	else if (auto sw = std::dynamic_pointer_cast<map::map_object>(obj))
	{
		ImGui::TextUnformatted(sw->name.c_str());
	}

	ImGui::EndTooltip();
}

ui::disambiguation_popup::disambiguation_popup(ui_panel &panel, map::sorted_object_list &&list) : popup(panel), m_list(list) {}

void ui::disambiguation_popup::render_content()
{
	for (auto &item : m_list)
	{
		if (ImGui::Button(item.second->name.c_str()))
		{
			ImGui::CloseCurrentPopup();

			handle_map_object_click(m_parent, item.second);
		}
	}
}

ui::semaphore_window::semaphore_window(ui_panel &panel, std::shared_ptr<map::semaphore> &&sem) : popup(panel), m_sem(sem) {}

void ui::semaphore_window::render_content()
{
	for (auto &model : m_sem->models)
	{
		ImGui::PushID(model);
		ImGui::TextUnformatted(model->name().c_str());

		for (int i = 0; i < iMaxNumLights; i++)
		{
			auto state = model->LightGet(i);
			if (!state)
				continue;

			glm::vec3 lamp_color = std::get<2>(*state).value_or(glm::vec3(0.5f));
			glm::vec3 current_color = lamp_color * std::get<1>(*state);
			float level = std::get<0>(*state);

			if (level < 0.0f)
				continue;

			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(lamp_color.x, lamp_color.y, lamp_color.z, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(current_color.x, current_color.y, current_color.z, 1.0f));

			std::string res = "  ##" + std::to_string(i);
			if (ImGui::Button(res.c_str()))
			{
				level += 1.0f;
				if (level >= 3.0f)
					level = 0.0f;

				m_relay.post(user_command::setlight, (double)i, level, GLFW_PRESS, 0, glm::vec3(0.0f), &model->name());
			}

			ImGui::PopStyleColor(2);
		}

		ImGui::PopID();
	}

	ImGui::Separator();

	for (auto &item : m_sem->events)
	{
		std::string displayname = item->name().substr(m_sem->name.size());

		if (displayname.size() < 2)
			continue;

		displayname[1] = std::toupper(displayname[1]);

		if (ImGui::Button(displayname.c_str()))
		{
			m_relay.post(user_command::queueevent, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(0.0f), &item->name());
		}
	}
}

ui::launcher_window::launcher_window(ui_panel &panel, std::shared_ptr<map::launcher> &&sw) : popup(panel), m_switch(sw) {}

void ui::launcher_window::render_content()
{
	ImGui::TextUnformatted(m_switch->name.c_str());

	const std::string &open_label =
	        m_switch->type == map::launcher::track_switch
	        ? STR("Straight |") : STR("Open |");

	const std::string &close_label =
	        m_switch->type == map::launcher::track_switch
	        ? STR("Divert /") : STR("Close -");

	if (ImGui::Button(open_label.c_str()))
	{
		m_relay.post(user_command::queueevent, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(0.0f), &m_switch->first_event->name());
		ImGui::CloseCurrentPopup();
	}

	if (ImGui::Button(close_label.c_str()))
	{
		m_relay.post(user_command::queueevent, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(0.0f), &m_switch->second_event->name());
		ImGui::CloseCurrentPopup();
	}
}

ui::track_switch_window::track_switch_window(ui_panel &panel, std::shared_ptr<map::track_switch> &&sw) : popup(panel), m_switch(sw) {}

bool ui::track_switch_window::render()
{
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.5f);
    bool ret = popup::render();
    ImGui::PopStyleVar();
    return ret;
}

void ui::track_switch_window::render_content()
{
    auto &highlight = dynamic_cast<map_panel&>(m_parent).highlighted_switches;

    ImGui::TextUnformatted(m_switch->name.c_str());

    highlight.clear();

    std::array<std::string, 4> names = { "ac", "ad", "bc", "bd" };

    for (size_t i = 0; i < 4; i++) {
        if (m_switch->action[i]) {
            if (ImGui::Button(names[i].c_str()))
                m_relay.post(user_command::queueevent, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(0.0f), &m_switch->action[i]->name());
            if (ImGui::IsItemHovered()) {
                for (size_t j = 0; j < 4; j++) {
                    if ((names[i][0] - 'a') != j && (names[i][1] - 'a') != j)
                        continue;
                    highlight.push_back(std::make_pair(m_switch->track[j], m_switch->preview[i][j] == '1' ? 1 : 0));
                }
            }
            ImGui::SameLine();
        }
    }
}

ui::obstacle_insert_window::obstacle_insert_window(ui_panel &panel, glm::dvec3 const &pos) : popup(panel), m_position(pos)
{
	std::ifstream file;
	file.open("obstaclebank.txt", std::ios_base::in | std::ios_base::binary);

	if (!file.is_open())
		return;

	std::string line;
	while (std::getline(file, line))
	{
		std::istringstream entry(line);

		std::string name;
		std::string data;
		std::getline(entry, name, ':');
		std::getline(entry, data, ':');

		m_obstacles.push_back(std::make_pair(name, data));
	}
}

void ui::obstacle_insert_window::render_content()
{
	if (m_obstacles.empty()) {
		ImGui::CloseCurrentPopup();
		return;
	}

	ImGui::TextUnformatted(STR_C("Insert obstacle:"));
	for (auto const &entry : m_obstacles)
	{
		if (ImGui::Button(entry.first.c_str()))
		{
			std::string name("obstacle_" + std::to_string(LocalRandom(0.0, 100000.0)));

			std::string payload(name + ':' + entry.second);
			m_relay.post(user_command::insertmodel, 0.0, 0.0, GLFW_PRESS, 0, m_position, &payload);

			auto obstacle = std::make_shared<map::obstacle>();
			obstacle->name = entry.first;
			obstacle->location = m_position;
			obstacle->model_name = name;

			map::Objects.entries.push_back(std::move(obstacle));

			ImGui::CloseCurrentPopup();
		}
	}
}

ui::obstacle_remove_window::obstacle_remove_window(ui_panel &panel, std::shared_ptr<map::obstacle> &&obstacle)
    : popup(panel), m_obstacle(obstacle) { }

void ui::obstacle_remove_window::render_content()
{
	if (ImGui::Button(STR_C("Delete obstacle"))) {
		m_relay.post(user_command::deletemodel, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(), &m_obstacle->model_name);

		auto &entries = map::Objects.entries;
		for (auto it = entries.rbegin(); it != entries.rend(); it++) {
			if (*it == m_obstacle) {
				entries.erase(std::next(it).base());
				break;
			}
		}

		ImGui::CloseCurrentPopup();
	}
}

ui::vehicle_click_window::vehicle_click_window(ui_panel &panel, std::shared_ptr<map::vehicle> &&obstacle)
    : popup(panel), m_vehicle(obstacle) { }

void ui::vehicle_click_window::render_content()
{
	ImGui::TextUnformatted(STR_C("Light signal:"));

	if (ImGui::Button(STR_C("none"))) {
		std::string name = m_vehicle->name + "%SetSignal";
		m_relay.post(user_command::sendaicommand, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(), &name);

		ImGui::CloseCurrentPopup();
	}
	if (ImGui::Button(STR_C("Pc6"))) {
		std::string name = m_vehicle->name + "%SetSignal";
		m_relay.post(user_command::sendaicommand, Signal_Pc6, 1.0, GLFW_PRESS, 0, glm::vec3(), &name);

		ImGui::CloseCurrentPopup();
	}
	if (ImGui::Button(STR_C("A1"))) {
		std::string name = m_vehicle->name + "%SetSignal";
		m_relay.post(user_command::sendaicommand, Signal_A1, 1.0, GLFW_PRESS, 0, glm::vec3(), &name);

		ImGui::CloseCurrentPopup();
	}

	if (DebugModeFlag) {
		ImGui::NewLine();
		ImGui::TextUnformatted(STR_C("Send command:"));
		ImGui::InputText(STR_C("Command"), command_buf.data(), command_buf.size());

		static std::vector<std::string> possible_commands =
		    {"CabSignal","Overhead","Emergency_brake","Timetable:","SetVelocity","SetProximityVelocity","ShuntVelocity",
		    "Wait_for_orders","Prepare_engine","Change_direction","Obey_train","Bank","Shunt","Loose_shunt",
		    "Jump_to_first_order","Jump_to_order","Warning_signal","Radio_channel","SetLights","SetSignal"};

		ImGui::SameLine();
		if (ImGui::BeginCombo("##cmd_select", nullptr, ImGuiComboFlags_NoPreview | ImGuiComboFlags_HeightLarge))
		{
			for (size_t i = 0; i < possible_commands.size(); i++)
				if (ImGui::Selectable(possible_commands[i].c_str(), false))
					memcpy(command_buf.data(), possible_commands[i].c_str(), possible_commands[i].size() + 1);
			ImGui::EndCombo();
		}

		ImGui::InputInt(STR_C("Param 1"), &command_param1);
		ImGui::InputInt(STR_C("Param 2"), &command_param2);
		if (ImGui::Button(STR_C("Send"))) {
			std::string name = m_vehicle->name + "%" + std::string(command_buf.data());
			m_relay.post(user_command::sendaicommand, command_param1, command_param2, GLFW_PRESS, 0, glm::vec3(), &name);

			ImGui::CloseCurrentPopup();
		}
	}
}
