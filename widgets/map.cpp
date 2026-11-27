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
#include "widgets/map_icons.h"
#include "world/Sweep.h"
#include <format>
#include <ranges>
#include <tuple>
#include <cstdint>

namespace
{

int constexpr map_vertex_limit = 48000;
int constexpr map_static_vertex_limit = 28000;
int constexpr map_symbol_vertex_limit = 40000;
int constexpr map_platform_vertex_limit = 42000;
int constexpr map_vehicle_vertex_limit = 47000;

bool map_has_vertex_budget(
	ImDrawList const *draw_list, int const required = 12, int const limit = map_vertex_limit)
{
	return draw_list->VtxBuffer.Size + required < limit;
}

float map_signal_light_opacity(std::tuple<float, float, std::optional<glm::vec3>> const &state)
{
	if (std::abs(std::get<0>(state)) < 0.01f)
		return 0.0f;
	return std::clamp(std::get<1>(state), 0.0f, 1.0f);
}

bool map_point_in_rect(ImVec2 point, ImVec2 minimum, ImVec2 maximum)
{
	return point.x >= minimum.x && point.x <= maximum.x &&
	       point.y >= minimum.y && point.y <= maximum.y;
}

bool map_world_bounds_visible(
	glm::dvec2 minimum, glm::dvec2 maximum, glm::mat4 const &transform,
	ImVec2 origin, glm::vec2 surface_size, float world_y, float screen_margin = 0.0f)
{
	std::array<glm::dvec2, 4> const corners {
		minimum, glm::dvec2(minimum.x, maximum.y),
		glm::dvec2(maximum.x, minimum.y), maximum};
	ImVec2 screen_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	ImVec2 screen_max(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
	for (glm::dvec2 const &corner : corners)
	{
		glm::vec4 const ndc = transform * glm::vec4(glm::vec3(corner.x, world_y, corner.y), 1.0f);
		ImVec2 const screen(
			origin.x + (ndc.x + 1.0f) * 0.5f * surface_size.x,
			origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y);
		screen_min.x = std::min(screen_min.x, screen.x);
		screen_min.y = std::min(screen_min.y, screen.y);
		screen_max.x = std::max(screen_max.x, screen.x);
		screen_max.y = std::max(screen_max.y, screen.y);
	}

	return screen_max.x >= origin.x - screen_margin &&
	       screen_min.x <= origin.x + surface_size.x + screen_margin &&
	       screen_max.y >= origin.y - screen_margin &&
	       screen_min.y <= origin.y + surface_size.y + screen_margin;
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

bool map_toolbar_icon_button(
	char const *id, char const *tooltip, map_icons::icon icon, bool selected, bool &hovered)
{
	ImGui::PushID(id);
	ImGui::PushStyleColor(
		ImGuiCol_Button,
		selected ? ImVec4(0.25f, 0.45f, 0.65f, 1.0f) : ImGui::GetStyleColorVec4(ImGuiCol_Button));
	float const button_size = ImGui::GetFrameHeight();
	bool const clicked = ImGui::Button("##icon", ImVec2(button_size, button_size));
	hovered = ImGui::IsItemHovered();
	ImVec2 const minimum = ImGui::GetItemRectMin();
	ImVec2 const maximum = ImGui::GetItemRectMax();
	ImVec2 const center((minimum.x + maximum.x) * 0.5f, (minimum.y + maximum.y) * 0.5f);
	map_icons::draw(ImGui::GetWindowDrawList(), icon, center, ImGui::GetColorU32(ImGuiCol_Text));

	if (hovered)
	{
		ImGui::BeginTooltip();
		ImGui::TextUnformatted(tooltip);
		ImGui::EndTooltip();
	}
	ImGui::PopStyleColor();
	ImGui::PopID();
	return clicked;
}

}

ui::map_panel::map_panel() : ui_panel(STR_C("Map"), false)
{
	size_min = {200, 200};
	size_max = {fb_size, fb_size};
	window_flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
	               ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus |
	               ImGuiWindowFlags_NoFocusOnAppearing;
	init_done = true;
}

void ui::map_panel::prepare_static_scene_cache()
{
	if (m_static_scene_region == simulation::Region)
		return;

	m_water_outlines.clear();
	m_river_lines.clear();
	m_environment_lines.clear();
	m_platform_lines.clear();
	m_station_labels.clear();
	m_switch_tracks.clear();
	m_signal_tracks.clear();
	m_signal_track_tree.clear();
	m_signal_caches.clear();
	cache_static_water();
	for (TTrack *track : simulation::Paths.sequence())
		cache_static_track_lines(track);
	if (!m_signal_tracks.empty())
		build_signal_track_tree(0, m_signal_tracks.size());
	cache_static_station_labels();
	cache_static_platforms();
	cache_signal_directions();
	m_static_scene_region = simulation::Region;
	m_static_view_cache_valid = false;
}

void ui::map_panel::cache_static_water()
{
	for (auto const &body : EditorTerrain.water())
	{
		if (body.outline.size() < 3)
			continue;
		cached_water_outline cached_body;
		cached_body.level = body.level;
		cached_body.minimum = glm::dvec2(std::numeric_limits<double>::max());
		cached_body.maximum = glm::dvec2(std::numeric_limits<double>::lowest());
		cached_body.points.reserve(body.outline.size());
		for (auto const &point : body.outline)
		{
			glm::dvec2 const planar_point(point.x, point.y);
			cached_body.points.push_back(planar_point);
			cached_body.minimum = glm::min(cached_body.minimum, planar_point);
			cached_body.maximum = glm::max(cached_body.maximum, planar_point);
		}
		m_water_outlines.push_back(std::move(cached_body));
	}
}

void ui::map_panel::append_cached_line(
	cached_world_lines &lines, glm::vec3 const &first, glm::vec3 const &second,
	ImU32 color, float thickness)
{
	lines.vertices.push_back(first);
	lines.vertices.push_back(second);
	if (color != 0 || thickness != 1.0f)
	{
		lines.line_colors.push_back(color);
		lines.line_thicknesses.push_back(thickness);
	}
	glm::dvec2 const first_planar(first.x, first.z);
	glm::dvec2 const second_planar(second.x, second.z);
	lines.minimum = glm::min(lines.minimum, glm::min(first_planar, second_planar));
	lines.maximum = glm::max(lines.maximum, glm::max(first_planar, second_planar));
}

void ui::map_panel::cache_static_track_lines(TTrack *track)
{
	if (!track)
		return;

	if ((track->iCategoryFlag & 15) == 1 && track->Segment)
	{
		signal_track_entry entry;
		entry.track = track;
		entry.sequence_order = m_signal_tracks.size();
		track->Segment->plan_bounds(entry.minimum, entry.maximum);
		m_signal_tracks.push_back(entry);
	}

	if (track->iCategoryFlag == 1 && track->eType == tt_Switch && track->SwitchExtension)
		m_switch_tracks.push_back(track);

	if ((track->iCategoryFlag & 15) == 4 && track->Segment)
	{
		TSegment const &segment = *track->Segment;
		double const length = segment.GetLength();
		if (length <= 0.0)
			return;

		cached_world_lines cached_river;
		double const half_width = 0.5 * std::abs(track->fTrackWidth);
		segment.plan_bounds(cached_river.minimum, cached_river.maximum);
		cached_river.minimum -= glm::dvec2(half_width);
		cached_river.maximum += glm::dvec2(half_width);
		glm::dvec3 previous_center;
		glm::dvec3 previous_across;
		segment.plan_frame(0.0, previous_center, previous_across);
		glm::dvec3 previous_left = previous_center + previous_across * half_width;
		glm::dvec3 previous_right = previous_center - previous_across * half_width;
		for (double distance = std::min(5.0, length);;)
		{
			glm::dvec3 center;
			glm::dvec3 across;
			segment.plan_frame(distance, center, across);
			glm::dvec3 const left = center + across * half_width;
			glm::dvec3 const right = center - across * half_width;
			append_cached_line(cached_river, glm::vec3(previous_left), glm::vec3(left), 0, 1.0f);
			append_cached_line(cached_river, glm::vec3(previous_right), glm::vec3(right), 0, 1.0f);
			previous_left = left;
			previous_right = right;
			if (distance >= length)
				break;
			distance = std::min(distance + 5.0, length);
		}
		m_river_lines.push_back(std::move(cached_river));
		return;
	}

	if (track->iCategoryFlag != 1 ||
	    (track->eEnvironment != e_tunnel && track->eEnvironment != e_bridge))
		return;

	auto cache_environment_segment = [&](TSegment const *segment) {
		if (!segment)
			return;
		cached_world_lines cached_lines;
		cached_lines.minimum = glm::dvec2(std::numeric_limits<double>::max());
		cached_lines.maximum = glm::dvec2(std::numeric_limits<double>::lowest());
		cached_lines.color = track->eEnvironment == e_tunnel ?
			IM_COL32(190, 125, 235, 230) : IM_COL32(70, 205, 225, 230);
		std::vector<gfx::basic_vertex> vertices;
		segment->render_lines(vertices, 0.5f);
		for (std::size_t idx = 0; idx + 1 < vertices.size(); idx += 2)
			append_cached_line(cached_lines, vertices[idx].position, vertices[idx + 1].position, 0, 1.0f);
		if (!cached_lines.vertices.empty())
			m_environment_lines.push_back(std::move(cached_lines));
	};

	if (track->eType == tt_Switch && track->SwitchExtension)
	{
		cache_environment_segment(track->SwitchExtension->Segments[0].get());
		cache_environment_segment(track->SwitchExtension->Segments[1].get());
	}
	else if (track->eType == tt_Normal)
		cache_environment_segment(track->Segment.get());
}

int ui::map_panel::build_signal_track_tree(std::size_t const first, std::size_t const last)
{
	signal_track_node node;
	node.first = first;
	node.last = last;
	node.minimum = glm::dvec2(std::numeric_limits<double>::max());
	node.maximum = glm::dvec2(std::numeric_limits<double>::lowest());
	for (std::size_t idx = first; idx < last; ++idx)
	{
		node.minimum = glm::min(node.minimum, m_signal_tracks[idx].minimum);
		node.maximum = glm::max(node.maximum, m_signal_tracks[idx].maximum);
	}

	int const node_index = static_cast<int>(m_signal_track_tree.size());
	m_signal_track_tree.push_back(node);
	if (last - first <= 8)
		return node_index;

	glm::dvec2 const extent = node.maximum - node.minimum;
	int const axis = extent.x >= extent.y ? 0 : 1;
	std::size_t const middle = first + (last - first) / 2;
	std::nth_element(
		m_signal_tracks.begin() + static_cast<std::ptrdiff_t>(first),
		m_signal_tracks.begin() + static_cast<std::ptrdiff_t>(middle),
		m_signal_tracks.begin() + static_cast<std::ptrdiff_t>(last),
		[axis](signal_track_entry const &lhs, signal_track_entry const &rhs) {
			return lhs.minimum[axis] + lhs.maximum[axis] <
			       rhs.minimum[axis] + rhs.maximum[axis];
		});

	int const left = build_signal_track_tree(first, middle);
	int const right = build_signal_track_tree(middle, last);
	m_signal_track_tree[static_cast<std::size_t>(node_index)].left = left;
	m_signal_track_tree[static_cast<std::size_t>(node_index)].right = right;
	return node_index;
}

void ui::map_panel::find_nearest_signal_track(
	int const node_index, glm::dvec3 const &position, TTrack *&nearest_track,
	double &nearest_distance2, std::size_t &nearest_order) const
{
	if (node_index < 0)
		return;

	auto const &node = m_signal_track_tree[static_cast<std::size_t>(node_index)];
	glm::dvec2 const point(position.x, position.z);
	glm::dvec2 const offset = glm::max(glm::max(node.minimum - point, point - node.maximum), glm::dvec2(0.0));
	if (glm::dot(offset, offset) > nearest_distance2)
		return;

	if (node.left < 0 || node.right < 0)
	{
		for (std::size_t idx = node.first; idx < node.last; ++idx)
		{
			TTrack *track = m_signal_tracks[idx].track;
			if (!track || !track->Segment)
				continue;
			glm::dvec3 const nearest_point = track->Segment->get_nearest_point(position);
			glm::dvec2 const track_offset(position.x - nearest_point.x, position.z - nearest_point.z);
			double const distance2 = glm::dot(track_offset, track_offset);
			if (distance2 < nearest_distance2 ||
			    (distance2 == nearest_distance2 && m_signal_tracks[idx].sequence_order < nearest_order))
			{
				nearest_distance2 = distance2;
				nearest_track = track;
				nearest_order = m_signal_tracks[idx].sequence_order;
			}
		}
		return;
	}

	auto const bounds_distance2 = [this, &point](int const child_index) {
		auto const &child = m_signal_track_tree[static_cast<std::size_t>(child_index)];
		glm::dvec2 const child_offset =
			glm::max(glm::max(child.minimum - point, point - child.maximum), glm::dvec2(0.0));
		return glm::dot(child_offset, child_offset);
	};
	int first_child = node.left;
	int second_child = node.right;
	if (bounds_distance2(second_child) < bounds_distance2(first_child))
		std::swap(first_child, second_child);
	find_nearest_signal_track(first_child, position, nearest_track, nearest_distance2, nearest_order);
	find_nearest_signal_track(second_child, position, nearest_track, nearest_distance2, nearest_order);
}

glm::dvec2 ui::map_panel::map_signal_direction(glm::dvec3 const &position, float const facing_angle) const
{
	TTrack *nearest_track = nullptr;
	double nearest_distance2 = std::numeric_limits<double>::max();
	std::size_t nearest_order = std::numeric_limits<std::size_t>::max();
	if (!m_signal_track_tree.empty())
		find_nearest_signal_track(0, position, nearest_track, nearest_distance2, nearest_order);
	if (!nearest_track || !nearest_track->Segment)
		return glm::dvec2(1.0, 0.0);

	double const path_length = nearest_track->Segment->GetLength();
	if (!(path_length > 0.0) || !std::isfinite(path_length))
		return glm::dvec2(1.0, 0.0);

	double const segment_distance = nearest_track->Segment->find_nearest_point(position) * path_length;
	glm::dvec3 const track_direction = nearest_track->Segment->GetDirection(segment_distance);
	glm::dvec2 direction(track_direction.x, track_direction.z);
	if (glm::length2(direction) <= 1e-8)
		return glm::dvec2(1.0, 0.0);

	direction = glm::normalize(direction);
	float const yaw = glm::radians(facing_angle);
	glm::dvec2 const signal_facing(-std::sin(yaw), -std::cos(yaw));
	if (glm::dot(direction, signal_facing) < 0.0)
		direction = -direction;
	return direction;
}

void ui::map_panel::cache_signal_directions()
{
	for (auto const &object : map::Objects.entries)
	{
		auto const semaphore = std::dynamic_pointer_cast<map::semaphore>(object);
		if (!semaphore)
			continue;

		cached_semaphore &cache = m_signal_caches[semaphore.get()];
		cache.lights.resize(semaphore->models.size());
		for (std::size_t idx = 0; idx < semaphore->models.size(); ++idx)
		{
			TAnimModel *model = semaphore->models[idx];
			cached_signal_light &light = cache.lights[idx];
			light.model = model;
			light.direction = map_signal_direction(
				model ? glm::dvec3(model->location()) : glm::dvec3(semaphore->location),
				model ? model->Angles().y : 0.0f);
			light.direction_valid = true;
		}
		if (semaphore->models.empty())
		{
			cache.fallback.direction =
				map_signal_direction(glm::dvec3(semaphore->location), 0.0f);
			cache.fallback.direction_valid = true;
		}
	}
}

void ui::map_panel::cache_static_station_labels()
{
	for (basic_event *event : simulation::Events.sequence())
	{
		if (!event || event->input_command() != TCommandType::cm_PassengerStopPoint)
			continue;
		std::string name = event->input_text();
		constexpr std::string_view stop_point_prefix = "PassengerStopPoint:";
		if (!name.starts_with(stop_point_prefix))
			continue;
		name = Bezogonkow(name.substr(stop_point_prefix.size()), true);
		if (!name.empty())
			m_station_labels.push_back({std::move(name), event->input_location()});
	}
}

void ui::map_panel::cache_static_platforms()
{
	for (sweep_node *sweep : simulation::Sweeps.sequence())
	{
		if (!sweep || !sweep->definition().platform)
			continue;

		glm::dvec3 low;
		glm::dvec3 high;
		glm::dvec3 shift;
		if (!sweep->bounds(low, high, shift))
			continue;

		auto const &definition = sweep->definition();
		double const near_side = definition.lateral >= 0.0 ? low.y : high.y;
		double const far_side = definition.lateral >= 0.0 ? high.y : low.y;
		double const start = std::clamp(sweep->start(), 0.0, sweep->length());
		double const end = std::clamp(sweep->end(), start, sweep->length());
		if (end - start < 0.1)
			continue;

		cached_world_lines cached_platform;
		cached_platform.minimum = glm::dvec2(std::numeric_limits<double>::max());
		cached_platform.maximum = glm::dvec2(std::numeric_limits<double>::lowest());
		auto const edge_position = [&](double distance, double side) {
			auto const frame = sweep->frame_at(distance);
			double const offset = definition.lateral + sweep->setback(frame) + side + shift.y;
			return frame.position + frame.left * offset;
		};
		double distance = start;
		auto near_previous = edge_position(distance, near_side);
		auto far_previous = edge_position(distance, far_side);
		while (distance < end)
		{
			distance = std::min(distance + 3.0, end);
			auto const near_current = edge_position(distance, near_side);
			auto const far_current = edge_position(distance, far_side);
			append_cached_line(cached_platform, glm::vec3(near_previous), glm::vec3(near_current),
			                   IM_COL32(255, 190, 90, 255), 1.5f);
			append_cached_line(cached_platform, glm::vec3(far_previous), glm::vec3(far_current),
			                   IM_COL32(220, 155, 75, 220), 1.0f);
			near_previous = near_current;
			far_previous = far_current;
		}
		append_cached_line(cached_platform, glm::vec3(edge_position(start, near_side)),
		                   glm::vec3(edge_position(start, far_side)), IM_COL32(255, 225, 160, 255), 1.0f);
		append_cached_line(cached_platform, glm::vec3(near_previous), glm::vec3(far_previous),
		                   IM_COL32(255, 225, 160, 255), 1.0f);
		if (!cached_platform.vertices.empty())
			m_platform_lines.push_back(std::move(cached_platform));
	}
}

float ui::map_panel::get_vehicle_rotation()
{
	const TDynamicObject *vehicle = simulation::Train->Dynamic();
	glm::vec3 front = glm::dvec3(vehicle->VectorFront()) * (vehicle->DirectionGet() > 0 ? 1.0 : -1.0);
	glm::vec2 north_ptr(0.0f, 1.0f);
	return glm::atan(front.z, front.x) - glm::atan(north_ptr.y, north_ptr.x);
}

void ui::map_panel::prepare_static_view_cache(glm::mat4 const &transform)
{
	if (m_static_view_cache_valid &&
	    m_cached_region == simulation::Region &&
	    m_cached_view_transform == transform)
		return;

	m_section_handles.clear();
	m_colored_paths.roads.clear();
	m_visible_switch_tracks.clear();
	m_cached_section_lines.clear();
	m_cached_road_lines.clear();

	for (auto const &section : map::Objects.section_geometry)
	{
		if (map_world_bounds_visible(
			    section.minimum, section.maximum, transform, ImVec2(0.0f, 0.0f),
			    glm::vec2(2.0f), 0.0f, 0.25f))
			m_section_handles.push_back(section.geometry);
	}
	for (auto const &section : map::Objects.road_geometry)
	{
		if (map_world_bounds_visible(
			    section.minimum, section.maximum, transform, ImVec2(0.0f, 0.0f),
			    glm::vec2(2.0f), 0.0f, 0.25f))
			m_colored_paths.roads.push_back(section.geometry);
	}

	ImVec2 const origin(0.0f, 0.0f);
	glm::vec2 const surface_size(2.0f, 2.0f);
	auto segment_visible = [&](TSegment const *segment) {
		if (!segment)
			return false;
		glm::dvec2 minimum;
		glm::dvec2 maximum;
		segment->plan_bounds(minimum, maximum);
		return map_world_bounds_visible(minimum, maximum, transform, origin, surface_size, 0.0f, 0.25f);
	};

	for (TTrack *track : m_switch_tracks) {
		if (!track || track->iCategoryFlag != 1 || track->eType != tt_Switch || !track->SwitchExtension)
			continue;
		if (segment_visible(track->SwitchExtension->Segments[0].get()) ||
		    segment_visible(track->SwitchExtension->Segments[1].get()))
			m_visible_switch_tracks.push_back(track);
	}

	auto cache_geometry_lines = [&](std::vector<gfx::geometrybank_handle> const &handles,
	                                std::vector<glm::vec4> &lines) {
		for (gfx::geometrybank_handle const &handle : handles)
		{
			if (handle == null_handle)
				continue;
			auto const &vertices = GfxRenderer->Vertices(handle);
			for (std::size_t idx = 0; idx + 1 < vertices.size(); idx += 2)
			{
				glm::vec4 const first = transform * glm::vec4(vertices[idx].position, 1.0f);
				glm::vec4 const second = transform * glm::vec4(vertices[idx + 1].position, 1.0f);
				lines.emplace_back(first.x, first.z, second.x, second.z);
			}
		}
	};
	cache_geometry_lines(m_section_handles, m_cached_section_lines);
	cache_geometry_lines(m_colored_paths.roads, m_cached_road_lines);

	m_cached_region = simulation::Region;
	m_cached_view_transform = transform;
	m_static_view_cache_valid = true;
}

void ui::map_panel::collect_map_geometry(glm::mat4 transform)
{
	prepare_static_view_cache(transform);

	m_colored_paths.switches.clear();
	m_colored_paths.future.clear();
	m_colored_paths.highlighted.clear();

	for (TTrack *track : m_visible_switch_tracks)
		track->get_map_active_paths(m_colored_paths);

	for (TTrack *track : simulation::Paths.sequence())
	{
		if (track && track->iCategoryFlag == 1 && !track->Dynamics.empty())
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
	std::int64_t const first_x = static_cast<std::int64_t>(std::floor(minimum.x / spacing));
	std::int64_t const last_x = static_cast<std::int64_t>(std::floor(maximum.x / spacing));
	for (std::int64_t x_index = first_x; x_index <= last_x; ++x_index)
	{
		double const x = static_cast<double>(x_index) * spacing;
		ImVec2 a = to_screen(glm::dvec3(x, 0.0, minimum.y));
		ImVec2 b = to_screen(glm::dvec3(x, 0.0, maximum.y));
		if (map_has_vertex_budget(draw_list) && clip_map_line(a, b, origin, surface_size))
			draw_list->AddLine(a, b, color);
	}
	std::int64_t const first_z = static_cast<std::int64_t>(std::floor(minimum.y / spacing));
	std::int64_t const last_z = static_cast<std::int64_t>(std::floor(maximum.y / spacing));
	for (std::int64_t z_index = first_z; z_index <= last_z; ++z_index)
	{
		double const z = static_cast<double>(z_index) * spacing;
		ImVec2 a = to_screen(glm::dvec3(minimum.x, 0.0, z));
		ImVec2 b = to_screen(glm::dvec3(maximum.x, 0.0, z));
		if (map_has_vertex_budget(draw_list) && clip_map_line(a, b, origin, surface_size))
			draw_list->AddLine(a, b, color);
	}
}

void ui::map_panel::render_map_lines(
	ImDrawList *draw_list, ImVec2 origin, glm::vec2 surface_size, glm::mat4 transform, double const scale_length)
{
	m_hovered_map_object.reset();
	ImVec2 const mouse_position = ImGui::GetMousePos();
	bool const mouse_over_map =
		map_point_in_rect(mouse_position, origin, ImVec2(origin.x + surface_size.x, origin.y + surface_size.y));

	const auto to_screen = [&](glm::vec3 const &position) {
		glm::vec4 ndc = transform * glm::vec4(position, 1.0f);
		return ImVec2(origin.x + (ndc.x + 1.0f) * 0.5f * surface_size.x,
		              origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y);
	};

	const auto draw_geometry = [&](gfx::geometrybank_handle const &handle, ImU32 color, float thickness = 1.0f) {
		if (handle == null_handle || !map_has_vertex_budget(draw_list, 12, map_static_vertex_limit))
			return;

		auto const &vertices = GfxRenderer->Vertices(handle);
		for (std::size_t idx = 0; idx + 1 < vertices.size(); idx += 2)
		{
			if (!map_has_vertex_budget(draw_list, 12, map_static_vertex_limit))
				break;
			ImVec2 a = to_screen(vertices[idx].position);
			ImVec2 b = to_screen(vertices[idx + 1].position);
			if (clip_map_line(a, b, origin, surface_size))
				draw_list->AddLine(a, b, color, thickness);
		}
	};

	const auto draw_cached_lines = [&](std::vector<glm::vec4> const &lines, ImU32 color) {
		for (glm::vec4 const &line : lines)
		{
			if (!map_has_vertex_budget(draw_list, 12, map_static_vertex_limit))
				break;
			ImVec2 const first(
				origin.x + (line.x + 1.0f) * 0.5f * surface_size.x,
				origin.y + (1.0f - line.y) * 0.5f * surface_size.y);
			ImVec2 const second(
				origin.x + (line.z + 1.0f) * 0.5f * surface_size.x,
				origin.y + (1.0f - line.w) * 0.5f * surface_size.y);
			ImVec2 a = first;
			ImVec2 b = second;
			if (clip_map_line(a, b, origin, surface_size))
				draw_list->AddLine(a, b, color);
		}
	};

	for (auto const &body : m_water_outlines)
	{
		if (!map_has_vertex_budget(draw_list, 2048, map_static_vertex_limit))
			continue;
		if (!map_world_bounds_visible(
			    body.minimum, body.maximum, transform, origin, surface_size, static_cast<float>(body.level)))
			continue;

		std::vector<ImVec2> outline;
		outline.reserve(std::min<std::size_t>(body.points.size(), 256));
		for (auto const &point : body.points)
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

		int const estimated_fill_vertices = static_cast<int>(outline.size()) * 6;
		if (map_has_vertex_budget(draw_list, estimated_fill_vertices, map_static_vertex_limit))
			draw_list->AddConcavePolyFilled(outline.data(), static_cast<int>(outline.size()), IM_COL32(35, 115, 165, 80));

		for (std::size_t idx = 0;
		     idx < outline.size() && map_has_vertex_budget(draw_list, 12, map_static_vertex_limit); ++idx)
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
	std::size_t const river_stride = static_cast<std::size_t>(std::ceil(river_step / 5.0));
	for (auto const &river : m_river_lines)
	{
		if (!map_world_bounds_visible(
			    river.minimum, river.maximum, transform, origin, surface_size, 0.0f, 2.0f))
			continue;
		std::size_t const segment_count = river.vertices.size() / 4;
		for (std::size_t first_segment = 0;
		     first_segment < segment_count &&
			     map_has_vertex_budget(draw_list, 12, map_static_vertex_limit);
		     first_segment += river_stride)
		{
			std::size_t const last_segment = std::min(first_segment + river_stride, segment_count) - 1;
			std::size_t const first_vertex = first_segment * 4;
			std::size_t const last_vertex = last_segment * 4;
			ImVec2 left_start = to_screen(river.vertices[first_vertex]);
			ImVec2 left_end = to_screen(river.vertices[last_vertex + 1]);
			if (clip_map_line(left_start, left_end, origin, surface_size))
				draw_list->AddLine(left_start, left_end, IM_COL32(80, 190, 230, 230), 1.5f);
			ImVec2 right_start = to_screen(river.vertices[first_vertex + 2]);
			ImVec2 right_end = to_screen(river.vertices[last_vertex + 3]);
			if (map_has_vertex_budget(draw_list, 12, map_static_vertex_limit) &&
			    clip_map_line(right_start, right_end, origin, surface_size))
				draw_list->AddLine(right_start, right_end, IM_COL32(80, 190, 230, 230), 1.5f);
		}
	}

	for (auto const &lines : m_environment_lines)
	{
		if (lines.vertices.empty() || !map_has_vertex_budget(draw_list, 12, map_static_vertex_limit))
			continue;
		if (!map_world_bounds_visible(
			    lines.minimum, lines.maximum, transform, origin, surface_size, 0.0f, 3.0f))
			continue;
		for (std::size_t idx = 0; idx + 1 < lines.vertices.size(); idx += 2)
		{
			if (!map_has_vertex_budget(draw_list, 12, map_static_vertex_limit))
				break;
			ImVec2 a = to_screen(lines.vertices[idx]);
			ImVec2 b = to_screen(lines.vertices[idx + 1]);
			if (clip_map_line(a, b, origin, surface_size))
				draw_list->AddLine(a, b, lines.color, 3.0f);
		}
	}

	draw_cached_lines(m_cached_road_lines, IM_COL32(235, 165, 65, 255));
	draw_cached_lines(m_cached_section_lines, IM_COL32(190, 195, 200, 255));
	for (auto const &handle : m_colored_paths.future)
		draw_geometry(handle, IM_COL32(180, 180, 65, 255), 1.5f);
	for (auto const &handle : m_colored_paths.switches)
		draw_geometry(handle, IM_COL32(0, 255, 0, 255));
	for (auto const &handle : m_colored_paths.highlighted)
		draw_geometry(handle, IM_COL32(75, 75, 255, 255));

	struct ordered_map_object
	{
		int signal_layer;
		float screen_y;
		std::shared_ptr<map::map_object> object;
	};
	std::vector<ordered_map_object> visible_map_objects;
	visible_map_objects.reserve(map::Objects.entries.size());
	for (auto const &object : map::Objects.entries)
	{
		glm::vec4 const ndc = transform * glm::vec4(object->location, 1.0f);
		if (std::abs(ndc.x) > 1.2f || std::abs(ndc.z) > 1.2f)
			continue;
		float const screen_y = origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y;
		int signal_layer = 0;
		if (auto const semaphore = std::dynamic_pointer_cast<map::semaphore>(object))
		{
			auto const cache_it = m_signal_caches.find(semaphore.get());
			if (cache_it != m_signal_caches.end())
			{
				glm::dvec2 const direction = cache_it->second.lights.empty()
					? cache_it->second.fallback.direction
					: cache_it->second.lights.front().direction;
				glm::vec4 const transformed_direction =
					transform * glm::vec4(static_cast<float>(direction.x), 0.0f,
					                      static_cast<float>(direction.y), 0.0f);
				signal_layer = -transformed_direction.z > 0.0f ? 1 : 2;
			}
		}
		visible_map_objects.push_back({signal_layer, screen_y, object});
	}
	std::ranges::stable_sort(
		visible_map_objects, [](auto const &lhs, auto const &rhs) {
			return std::tie(lhs.signal_layer, lhs.screen_y) <
			       std::tie(rhs.signal_layer, rhs.screen_y);
		});

	for (auto const &entry : visible_map_objects)
	{
		auto const &object = entry.object;
		if (!map_has_vertex_budget(draw_list, 16, map_symbol_vertex_limit))
			break;

		if (auto sem = std::dynamic_pointer_cast<map::semaphore>(object))
		{
			cached_semaphore &signal_cache = m_signal_caches[sem.get()];
			signal_cache.lights.resize(sem->models.size());
			auto const light_cache_now = std::chrono::steady_clock::now();
			auto draw_signal_head = [this, &sem, draw_list, &to_screen, &transform, &surface_size, scale_length,
			                         mouse_over_map, mouse_position, light_cache_now,
			                         origin](TAnimModel *model, glm::vec3 const &location,
			                                 cached_signal_light &cache) {
				ImVec2 const center = to_screen(location);
				float constexpr signal_cull_margin = 80.0f;
				if (center.x < origin.x - signal_cull_margin ||
				    center.x > origin.x + surface_size.x + signal_cull_margin ||
				    center.y < origin.y - signal_cull_margin ||
				    center.y > origin.y + surface_size.y + signal_cull_margin)
					return;

				if (cache.model != model)
				{
					cache.model = model;
					cache.direction_valid = false;
					cache.valid = false;
				}
				if (model && (!cache.valid || cache.model != model ||
				              light_cache_now - cache.refreshed_at >= std::chrono::milliseconds(100))) {
					cache.colors.clear();
					for (int i = 0; i < iMaxNumLights; ++i) {
						auto state = model->LightGet(i);
						if (!state)
							continue;

						glm::vec3 const base_color = std::get<2>(*state).value_or(glm::vec3(0.5f));
						float const opacity = map_signal_light_opacity(*state);
						auto const color_component = [](float component) {
							return static_cast<int>(std::clamp(component * 255.0f, 0.0f, 255.0f));
						};
						cache.colors.resize(static_cast<std::size_t>(i) + 1, 0);
						cache.colors[static_cast<std::size_t>(i)] = static_cast<std::uint32_t>(IM_COL32(
							color_component(base_color.x),
							color_component(base_color.y),
							color_component(base_color.z),
							static_cast<int>(opacity * 255.0f)));
					}
					cache.model = model;
					cache.refreshed_at = light_cache_now;
					cache.valid = true;
				}

				if (sem->shunting)
				{
					glm::dvec2 const world_direction = map_signal_direction(
						glm::dvec3(location),
						model ? model->Angles().y : 0.0f
						);
					glm::vec4 const transformed_direction =transform * glm::vec4(
						static_cast<float>(world_direction.x),
						0.0f,
						static_cast<float>(world_direction.y),
						0.0f
						);
					glm::vec2 post_direction(
						transformed_direction.x * surface_size.x * 0.5f,
						-transformed_direction.z * surface_size.y * 0.5f
						);
					if (glm::length2(post_direction) <= 1e-8f || !std::isfinite(glm::length2(post_direction)))
					{
						post_direction = glm::vec2(1.0f, 0.0f);
					}
					else
					{
						post_direction = glm::normalize(post_direction);
					}
					float const shunt_post_length = std::lerp(20.0f,10.0f, std::clamp(static_cast<float>(scale_length) / 100.0f, 0.0f, 1.0f));
					ImVec2 const head_start(
						center.x + post_direction.x * shunt_post_length,
						center.y + post_direction.y * shunt_post_length
						);
					glm::vec2 const head_axis(post_direction.x, post_direction.y);
					glm::vec2 const head_normal(-head_axis.y, head_axis.x);
					float const lamp_spacing = std::lerp(10.0f, 0.0f, std::clamp(static_cast<float>(scale_length) / 100.0f, 0.0f, 1.0f));
					float const head_length = 10.0f + (cache.colors.size() - 1.0f) * lamp_spacing;
					float constexpr head_half_width = 5.0f;
					std::array<ImVec2, 4> const head_corners
					{
						ImVec2(
							head_start.x - head_normal.x * head_half_width,
							head_start.y - head_normal.y * head_half_width
							),
						ImVec2(
							head_start.x + head_axis.x * head_length - head_normal.x * head_half_width,
							head_start.y + head_axis.y * head_length - head_normal.y * head_half_width
							),
						ImVec2(
							head_start.x + head_axis.x * head_length + head_normal.x * head_half_width,
							head_start.y + head_axis.y * head_length + head_normal.y * head_half_width
							),
						ImVec2(
							head_start.x + head_normal.x * head_half_width,
							head_start.y + head_normal.y * head_half_width
							)
					};
					draw_list->AddLine(center, head_start, IM_COL32(150, 150, 150, 255), 5.0f);
					glm::vec2 const mouse_from_head(mouse_position.x - head_start.x, mouse_position.y - head_start.y);
					bool const shunt_hovered = mouse_over_map &&
						((std::abs(glm::dot(mouse_from_head, head_axis)) <= head_length &&
						  std::abs(glm::dot(mouse_from_head, head_normal)) <= head_half_width) ||
						 glm::length2(glm::vec2(mouse_position.x - center.x, mouse_position.y - center.y)) <= 100.0f);
					if (shunt_hovered)
					{
						m_hovered_map_object = sem;
						draw_list->AddLine(center, head_start, IM_COL32(120, 200, 255, 255), 8.0f);
					}
					ImU32 const head_color = shunt_hovered
						? IM_COL32(120, 200, 255, 255)
						: IM_COL32(0, 0, 0, 255);
					draw_list->AddQuadFilled(head_corners[0], head_corners[1], head_corners[2], head_corners[3], head_color);
					if (shunt_hovered)
					{
						for (std::size_t corner = 0; corner < head_corners.size(); ++corner)
						{
							draw_list->AddLine(head_corners[corner], head_corners[(corner + 1) % head_corners.size()], head_color, 3.0f);
						}
					}
					if (cache.colors.size() > 0)
					{
						for (std::size_t idx = 0; idx < cache.colors.size(); ++idx)
						{
							ImU32 color = cache.colors[cache.colors.size() - 1 - idx];
							if ((color & IM_COL32_A_MASK) == 0)
							{
								continue;
							}
							float const lamp_offset = 5.0f + static_cast<float>(cache.colors.size() - 1 - idx) * lamp_spacing;
							ImVec2 const lamp_position(
								head_start.x + head_axis.x * lamp_offset,
								head_start.y + head_axis.y * lamp_offset
								);
							draw_list->AddCircleFilled(lamp_position, 2.5f, color);
						}
					}
					return;
				}

				if (!cache.direction_valid) {
					cache.direction = map_signal_direction(
						glm::dvec3(location), model ? model->Angles().y : 0.0f);
					cache.direction_valid = true;
				}
				glm::dvec2 const &world_direction = cache.direction;
				glm::vec4 const transformed_direction =
					transform * glm::vec4(static_cast<float>(world_direction.x), 0.0f,
					                     static_cast<float>(world_direction.y), 0.0f);
				glm::vec2 dir(
					transformed_direction.x * surface_size.x * 0.5f,
					-transformed_direction.z * surface_size.y * 0.5f);
				if (glm::length2(dir) <= 1e-8f || !std::isfinite(glm::length2(dir)))
					dir = glm::vec2(1.0f, 0.0f);
				else
					dir = glm::normalize(dir);
				glm::vec2 const post_direction(dir.x, dir.y);
				float const post_length = std::lerp(50.0f, 10.0f, std::clamp(static_cast<float>(scale_length) / 100.0f, 0.0f, 1.0f));
				float const post_segment_length = post_length / 6.0f;
				ImVec2 const post_start(center.x, center.y);
				ImVec2 const post_end(
					center.x + post_direction.x * post_length,
					center.y + post_direction.y * post_length);
				glm::vec2 const mouse_from_post(
					mouse_position.x - center.x,
					mouse_position.y - center.y);
				float const post_projection = glm::dot(
					mouse_from_post,
					glm::vec2(post_direction.x, post_direction.y));
				float const post_projection_clamped = std::clamp(post_projection, 0.0f, post_length);
				glm::vec2 const post_nearest(
					center.x + post_direction.x * post_projection_clamped,
					center.y + post_direction.y * post_projection_clamped);
				glm::vec2 const post_distance(
					mouse_position.x - post_nearest.x,
					mouse_position.y - post_nearest.y);
				bool const post_hovered =
					mouse_over_map && glm::length2(post_distance) <= 49.0f;
				float const lamp_spacing = std::lerp(10.0f, 0.0f, std::clamp(static_cast<float>(scale_length) / 100.0f, 0.0f, 1.0f));
				if (cache.colors.empty())
				{
					bool const icon_hovered = mouse_over_map &&
						(glm::length2(glm::vec2(mouse_position.x - post_end.x, mouse_position.y - post_end.y)) <= 100.0f ||
						 post_hovered);
					if (icon_hovered || post_hovered)
						m_hovered_map_object = sem;
					draw_list->AddLine(post_start, post_end, IM_COL32(255, 255, 255, 255), 3.0f);
					for (int segment = 0; segment < 6; segment += 2)
					{
						ImVec2 const segment_start(
							center.x + post_direction.x * post_segment_length * static_cast<float>(segment),
							center.y + post_direction.y * post_segment_length * static_cast<float>(segment));
						ImVec2 const segment_end(
							center.x + post_direction.x * post_segment_length * static_cast<float>(segment + 1),
							center.y + post_direction.y * post_segment_length * static_cast<float>(segment + 1));
						draw_list->AddLine(segment_start, segment_end, IM_COL32(255, 0, 0, 255), 3.0f);
					}
					if (icon_hovered || post_hovered)
						draw_list->AddLine(post_start, post_end, IM_COL32(120, 200, 255, 255), 5.0f);
					draw_list->AddCircleFilled(post_end, 5.0f, IM_COL32(255, 210, 70, 255));
					if (icon_hovered || post_hovered)
						draw_list->AddCircle(post_end, 8.0f, IM_COL32(120, 200, 255, 255), 0, 3.0f);
					return;
				}

				glm::vec2 const head_axis(post_direction.x, post_direction.y);
				float const head_half_length = std::max(5.0f, (static_cast<float>(cache.colors.size()) - 1.0f) * lamp_spacing * 0.5f + 4.5f);
				ImVec2 const head_center(post_end.x+head_axis.x * head_half_length, post_end.y + head_axis.y * head_half_length);
				glm::vec2 const head_normal(-head_axis.y, head_axis.x);
				float constexpr head_half_width = 5.0f;
				float const head_body_half_length = std::max(0.0f, head_half_length - head_half_width);
				glm::vec2 const mouse_from_head(mouse_position.x - head_center.x, mouse_position.y - head_center.y);
				bool const icon_hovered = mouse_over_map &&
					((std::abs(glm::dot(mouse_from_head, head_axis)) <= head_half_length &&
					  std::abs(glm::dot(mouse_from_head, head_normal)) <= head_half_width) ||
					 post_hovered);
				bool const signal_hovered = icon_hovered;
				if (signal_hovered)
					m_hovered_map_object = sem;

				draw_list->AddLine(post_start, post_end, IM_COL32(245, 245, 245, 255), 3.0f);
				for (int segment = 0; segment < 6; segment += 2)
				{
					ImVec2 const segment_start(
						center.x + post_direction.x * post_segment_length * static_cast<float>(segment),
						center.y + post_direction.y * post_segment_length * static_cast<float>(segment));
					ImVec2 const segment_end(
						center.x + post_direction.x * post_segment_length * static_cast<float>(segment + 1),
						center.y + post_direction.y * post_segment_length * static_cast<float>(segment + 1));
					draw_list->AddLine(segment_start, segment_end, IM_COL32(220, 55, 55, 255), 3.0f);
				}
				if (signal_hovered)
					draw_list->AddLine(post_start, post_end, IM_COL32(120, 200, 255, 255), 5.0f);
				ImVec2 const front_cap_center(
					head_center.x + head_axis.x * head_body_half_length,
					head_center.y + head_axis.y * head_body_half_length);
				ImVec2 const rear_cap_center(
					head_center.x - head_axis.x * head_body_half_length,
					head_center.y - head_axis.y * head_body_half_length);
				auto const add_head_shape = [draw_list, front_cap_center, rear_cap_center](
						ImU32 color, float thickness) {
					float const radius = thickness * 0.5f;
					draw_list->AddLine(rear_cap_center, front_cap_center, color, thickness);
					draw_list->AddCircleFilled(front_cap_center, radius, color);
					draw_list->AddCircleFilled(rear_cap_center, radius, color);
				};
				if (signal_hovered)
				{
					ImU32 constexpr highlight_color = IM_COL32(120, 200, 255, 255);
					add_head_shape(highlight_color, head_half_width * 2.0f + 3.0f);
				}
				else
					add_head_shape(IM_COL32(0, 0, 0, 255), head_half_width * 2.0f);
				for (std::size_t idx = cache.colors.size(); idx-- > 0;)
				{
					if ((cache.colors[idx] & IM_COL32_A_MASK) == 0)
						continue;
					float const offset =
						static_cast<float>(cache.colors.size() - idx - 1) -
						(static_cast<float>(cache.colors.size()) - 1.0f) * 0.5f;
					ImVec2 const lamp_position(
						head_center.x + head_axis.x * offset * lamp_spacing,
						head_center.y + head_axis.y * offset * lamp_spacing);
					draw_list->AddCircleFilled(lamp_position, 3.6f, static_cast<ImU32>(cache.colors[idx]));
				}
			};

			if (sem->models.empty())
				draw_signal_head(nullptr, sem->location, signal_cache.fallback);
			else
				for (std::size_t idx = 0; idx < sem->models.size(); ++idx)
					draw_signal_head(
						sem->models[idx], glm::vec3(sem->models[idx]->location()), signal_cache.lights[idx]);
			continue;
		}

		ImVec2 const object_position = to_screen(object->location);
		if (object_position.x < origin.x - 10.0f || object_position.x > origin.x + surface_size.x + 10.0f ||
		    object_position.y < origin.y - 10.0f || object_position.y > origin.y + surface_size.y + 10.0f)
			continue;

		bool const object_hovered = mouse_over_map &&
			glm::length2(glm::vec2(mouse_position.x - object_position.x, mouse_position.y - object_position.y)) <= 100.0f;
		if (object_hovered)
			m_hovered_map_object = object;

		if (auto const launcher = std::dynamic_pointer_cast<map::launcher>(object);
		    launcher && launcher->type == map::launcher::level_crossing)
		{
			draw_list->AddCircleFilled(object_position, 8.0f, IM_COL32(35, 40, 45, 240));
			draw_list->AddLine(
				ImVec2(object_position.x - 5.0f, object_position.y - 5.0f),
				ImVec2(object_position.x + 5.0f, object_position.y + 5.0f),
				IM_COL32(245, 245, 245, 255), 4.0f);
			draw_list->AddLine(
				ImVec2(object_position.x - 5.0f, object_position.y + 5.0f),
				ImVec2(object_position.x + 5.0f, object_position.y - 5.0f),
				IM_COL32(245, 245, 245, 255), 4.0f);
			draw_list->AddLine(
				ImVec2(object_position.x - 5.0f, object_position.y - 5.0f),
				ImVec2(object_position.x + 5.0f, object_position.y + 5.0f),
				IM_COL32(220, 55, 55, 255), 1.5f);
			draw_list->AddLine(
				ImVec2(object_position.x - 5.0f, object_position.y + 5.0f),
				ImVec2(object_position.x + 5.0f, object_position.y - 5.0f),
				IM_COL32(220, 55, 55, 255), 1.5f);
			if (object_hovered)
				draw_list->AddCircle(object_position, 11.0f, IM_COL32(120, 200, 255, 255), 0, 2.0f);
			continue;
		}

		ImU32 color = IM_COL32(255, 220, 70, 255);
		if (std::dynamic_pointer_cast<map::launcher>(object))
			color = IM_COL32(255, 150, 50, 255);
		else if (std::dynamic_pointer_cast<map::track_switch>(object))
			color = IM_COL32(80, 190, 255, 255);
		else if (std::dynamic_pointer_cast<map::obstacle>(object))
			color = IM_COL32(255, 80, 80, 255);
		draw_list->AddCircleFilled(object_position, object_hovered ? 5.0f : 3.0f, color);
		if (object_hovered)
			draw_list->AddCircle(object_position, 9.0f, IM_COL32(120, 200, 255, 255), 0, 2.0f);
	}

}

void ui::map_panel::render_labels(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size)
{
	ImDrawList *draw_list = ImGui::GetWindowDrawList();
	std::vector<std::string> station_names;
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.85f, 0.9f, 1.0f, 1.0f));
	for (auto const &station : m_station_labels)
	{
		if (station.name.empty() ||
		    std::find(station_names.begin(), station_names.end(), station.name) != station_names.end())
			continue;

		glm::vec4 ndc_pos = transform * glm::vec4(glm::vec3(station.position), 1.0f);
		if (glm::abs(ndc_pos.x) > 1.0f || glm::abs(ndc_pos.z) > 1.0f)
			continue;
		if (!map_has_vertex_budget(
			    draw_list, static_cast<int>(station.name.size() * 8 + 16), map_platform_vertex_limit))
			break;
		station_names.push_back(station.name);

		ImVec2 const text_size = ImGui::CalcTextSize(station.name.c_str());
		glm::vec2 const gui_pos =
			(glm::vec2(ndc_pos.x, -ndc_pos.z) / 2.0f + 0.5f) * glm::vec2(surface_size.x, surface_size.y);
		ImGui::SetCursorPos(ImVec2(
			origin.x + gui_pos.x - text_size.x / 2.0f,
			origin.y + gui_pos.y - text_size.y / 2.0f));
		ImGui::TextUnformatted(station.name.c_str());
	}
	ImGui::PopStyleColor();

	ImVec2 const window_position = ImGui::GetWindowPos();
	ImVec2 const map_screen_origin(window_position.x + origin.x, window_position.y + origin.y);
	draw_list->PushClipRect(
		map_screen_origin,
		ImVec2(map_screen_origin.x + surface_size.x, map_screen_origin.y + surface_size.y),
		true);
	auto const to_screen = [&](glm::vec3 const &position) {
		glm::vec4 const ndc = transform * glm::vec4(position, 1.0f);
		return ImVec2(
			map_screen_origin.x + (ndc.x + 1.0f) * 0.5f * surface_size.x,
			map_screen_origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y);
	};
	for (auto const &platform : m_platform_lines)
	{
		if (!map_has_vertex_budget(draw_list, 12, map_platform_vertex_limit) || platform.vertices.empty())
			break;
		if (!map_world_bounds_visible(
			    platform.minimum, platform.maximum, transform, map_screen_origin, surface_size, 0.0f, 3.0f))
			continue;
		for (std::size_t idx = 0; idx + 1 < platform.vertices.size(); idx += 2)
		{
			if (!map_has_vertex_budget(draw_list, 16, map_platform_vertex_limit))
				break;
			ImVec2 a = to_screen(platform.vertices[idx]);
			ImVec2 b = to_screen(platform.vertices[idx + 1]);
			if (clip_map_line(a, b, map_screen_origin, surface_size))
				draw_list->AddLine(a, b, platform.line_colors[idx / 2], platform.line_thicknesses[idx / 2]);
		}
	}
	draw_list->PopClipRect();

	std::vector<std::pair<float, TDynamicObject *>> visible_vehicles;
	visible_vehicles.reserve(simulation::Vehicles.sequence().size());
	for (TDynamicObject *vehicle : simulation::Vehicles.sequence())
	{
		if (!vehicle->MyTrack || (vehicle->MyTrack->iCategoryFlag & 15) != 1)
			continue;

		glm::vec4 const ndc_pos = transform * glm::vec4(glm::vec3(vehicle->GetPosition()), 1.0f);
		if (glm::abs(ndc_pos.x) > 1.0f || glm::abs(ndc_pos.z) > 1.0f)
			continue;
		float const screen_y = map_screen_origin.y + (1.0f - ndc_pos.z) * 0.5f * surface_size.y;
		visible_vehicles.emplace_back(screen_y, vehicle);
	}
	std::ranges::stable_sort(
		visible_vehicles, [](auto const &lhs, auto const &rhs) { return lhs.first < rhs.first; });

	TDynamicObject *const controlled_vehicle =
		simulation::Train ? simulation::Train->Dynamic() : nullptr;
	for (auto const &entry : visible_vehicles)
	{
		TDynamicObject *const vehicle = entry.second;
		if (!map_has_vertex_budget(draw_list, 16, map_vehicle_vertex_limit) &&
		    vehicle != controlled_vehicle)
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

		std::string train_name = vehicle->Mechanik->TrainName();
		if (train_name.empty() || train_name == "none")
			train_name = ToUpper(vehicle->name());
		std::string const label = std::format("{} | {:.0f} km/h", train_name, std::abs(vehicle->GetVelocity()));

		if (!map_has_vertex_budget(
			    draw_list, static_cast<int>(label.size() * 8 + 16), map_vehicle_vertex_limit))
			break;
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

void ui::map_panel::render_camera_frustum(
	ImDrawList *draw_list, ImVec2 screen_origin, glm::vec2 surface_size, glm::mat4 transform)
{
	if (!map_has_vertex_budget(draw_list, 64))
		return;

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
	if (!(forward_length > 0.0))
		return;

	forward /= forward_length;
	glm::dvec3 const side(-forward.z, 0.0, forward.x);
	double const half_fov = glm::radians(
		static_cast<double>(Global.FieldOfView / std::max(Global.ZoomFactor, 0.01f))) * 0.5;
	double constexpr view_length = 300.0;
	glm::dvec3 const left =
		camera.Pos + (forward * std::cos(half_fov) - side * std::sin(half_fov)) * view_length;
	glm::dvec3 const right =
		camera.Pos + (forward * std::cos(half_fov) + side * std::sin(half_fov)) * view_length;
	auto const to_screen = [transform, screen_origin, surface_size](glm::dvec3 const &position) {
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

void ui::map_panel::render_contents()
{
    if (!init_done) {
        is_open = false;
        return;
    }

	if (!simulation::Train && mode == MODE_VEHICLE)
		mode = MODE_MANUAL;

	ImVec2 const padding = ImGui::GetStyle().WindowPadding;
	ImVec2 const cursor_origin = ImGui::GetCursorPos();
	ImVec2 const cursor_screen_origin = ImGui::GetCursorScreenPos();
	ImVec2 const available = ImGui::GetContentRegionAvail();
	ImVec2 const window_origin(cursor_origin.x - padding.x, cursor_origin.y - padding.y);
	ImVec2 const screen_origin(cursor_screen_origin.x - padding.x, cursor_screen_origin.y - padding.y);
	ImVec2 const surface_size_im(available.x + padding.x * 2.0f, available.y + padding.y * 2.0f);
	glm::vec2 const surface_size(surface_size_im.x, surface_size_im.y);
	ImGui::SetCursorScreenPos(screen_origin);
	ImGui::SetNextItemAllowOverlap();
	ImGui::InvisibleButton("##map", surface_size_im);
	bool const map_hovered = ImGui::IsItemHovered();

	ImDrawList *draw_list = ImGui::GetWindowDrawList();
	draw_list->ChannelsSplit(2);
	draw_list->ChannelsSetCurrent(1);
	ImGui::SetCursorScreenPos(ImVec2(screen_origin.x + 10.0f, screen_origin.y + 10.0f));

	bool toolbar_hovered = false;
	bool zoom_in_hovered = false;
	bool const zoom_in = map_toolbar_icon_button(
		"zoom-in", STR_C("Zoom in"), map_icons::icon::zoom_in, false, zoom_in_hovered);
	toolbar_hovered = toolbar_hovered || zoom_in_hovered;
	ImGui::SameLine();
	bool zoom_out_hovered = false;
	bool const zoom_out = map_toolbar_icon_button(
		"zoom-out", STR_C("Zoom out"), map_icons::icon::zoom_out, false, zoom_out_hovered);
	toolbar_hovered = toolbar_hovered || zoom_out_hovered;
	ImGui::SameLine();

	glm::mat4 transform;
	transform[0][0] = -1.0f;

	bool pan_hovered = false;
	if (map_toolbar_icon_button(
		    "pan", STR_C("Pan"), map_icons::icon::pan, mode == MODE_MANUAL, pan_hovered))
		mode = MODE_MANUAL;
	toolbar_hovered = toolbar_hovered || pan_hovered;
	ImGui::SameLine();
	bool camera_hovered = false;
	if (map_toolbar_icon_button(
		    "camera", STR_C("Follow camera"), map_icons::icon::camera, mode == MODE_CAMERA, camera_hovered))
		mode = MODE_CAMERA;
	toolbar_hovered = toolbar_hovered || camera_hovered;
	ImGui::SameLine();
	ImGui::BeginDisabled(simulation::Train == nullptr);
	bool vehicle_hovered = false;
	if (map_toolbar_icon_button(
		    "vehicle", STR_C("Follow vehicle"), map_icons::icon::vehicle, mode == MODE_VEHICLE, vehicle_hovered))
		mode = MODE_VEHICLE;
	ImGui::EndDisabled();
	toolbar_hovered = toolbar_hovered || vehicle_hovered;
	ImGui::SameLine();
	bool reset_heading_hovered = false;
	if (map_toolbar_icon_button(
		    "reset-heading", STR_C("Reset map rotation"), map_icons::icon::rotate, false, reset_heading_hovered))
		m_manual_rotation = 0.0f;
	toolbar_hovered = toolbar_hovered || reset_heading_hovered;

	glm::mat4 aspect_transform;
	float aspect = surface_size.y / surface_size.x;

	if (aspect > 1.0f / aspect)
		aspect_transform = glm::scale(glm::mat4(), glm::vec3(aspect, 1.0f, 1.0f));
	else
		aspect_transform = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f / aspect));

	transform *= aspect_transform;

	if (mode == MODE_MANUAL)
		transform = glm::rotate(transform, m_manual_rotation, glm::vec3(0.0f, 1.0f, 0.0f));

	TDynamicObject const *followed_vehicle = nullptr;
	glm::vec3 follow_position{};
	glm::vec3 follow_direction{};
	glm::vec2 follow_screen_offset{};
	float map_rotation = m_manual_rotation;
	if (mode == MODE_VEHICLE && simulation::Train)
	{
		followed_vehicle = simulation::Train->Dynamic();
		map_rotation = get_vehicle_rotation() + m_manual_rotation;

		transform = glm::rotate(transform, map_rotation, glm::vec3(0.0f, 1.0f, 0.0f));

		follow_position = followed_vehicle->GetPosition();
		follow_direction = followed_vehicle->VectorFront() * (followed_vehicle->DirectionGet() > 0 ? 1.0 : -1.0);
		double const direction_length = glm::length(follow_direction);
		if (direction_length > 0.0)
			follow_direction /= direction_length;
		follow_direction = glm::rotateY(follow_direction, -m_manual_rotation);
	}
	if (mode == MODE_CAMERA)
	{
		float initial_rotation;
		if (!FreeFlyModeFlag)
			initial_rotation = get_vehicle_rotation();
		else
			initial_rotation = glm::pi<float>();

		map_rotation = initial_rotation - Global.pCamera.Angle.y + m_manual_rotation;

		transform = glm::rotate(transform, map_rotation, glm::vec3(0.0f, 1.0f, 0.0f));

		glm::dvec3 position = Global.pCamera.Pos;
		translate = glm::vec2(position.x, position.z) * -zoom;
	}

	glm::vec4 const screen_x = transform * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
	float const base_pixels_per_meter = glm::length(glm::vec2(
		screen_x.x * surface_size.x * 0.5f,
		screen_x.z * surface_size.y * -0.5f));
	if (base_pixels_per_meter > 0.0f && std::isfinite(base_pixels_per_meter))
	{
		float const min_zoom = (110.0f / 4000.0f) / base_pixels_per_meter;
		float const max_zoom = (110.0f / 12.5f) / base_pixels_per_meter;
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
		translate = glm::vec2(follow_position.x, follow_position.z) * -zoom;
		glm::vec4 const screen_direction = transform * glm::vec4(glm::vec3(follow_direction), 0.0f);
		follow_screen_offset = glm::vec2(screen_direction.x, screen_direction.z) * static_cast<float>(-lookahead * zoom);
	}

	transform = glm::translate(transform, glm::vec3(translate.x, 0.0f, translate.y));
	transform = glm::scale(transform, glm::vec3(zoom));
	transform = glm::translate(glm::mat4(1.0f), glm::vec3(follow_screen_offset.x, 0.0f, follow_screen_offset.y)) * transform;
	double const scale_length = 110.0 / (base_pixels_per_meter * zoom);
	double const scale_distance = nice_distance_step(scale_length);

	prepare_static_scene_cache();
	collect_map_geometry(transform);

	draw_list->ChannelsSetCurrent(0);
	draw_list->PushClipRect(screen_origin, ImVec2(screen_origin.x + surface_size.x, screen_origin.y + surface_size.y), true);
	draw_list->AddRectFilled(
		screen_origin,
		ImVec2(screen_origin.x + surface_size.x, screen_origin.y + surface_size.y),
		IM_COL32(18, 22, 26, static_cast<int>(Global.UIBgOpacity * 255.0f)));
	render_grid(draw_list, screen_origin, surface_size, transform);
	render_map_lines(draw_list, screen_origin, surface_size, transform, scale_length);
	draw_list->PopClipRect();

	if (map_hovered && !toolbar_hovered)
	{
		const ImVec2 screen_pos = ImGui::GetMousePos();
		const glm::vec2 surface_pos(screen_pos.x - screen_origin.x, screen_pos.y - screen_origin.y);
		const glm::vec2 ndc_pos = surface_pos / surface_size * 2.0f - 1.0f;
		glm::vec3 const world_pos = glm::inverse(transform) * glm::vec4(ndc_pos.x, 0.0f, -ndc_pos.y, 1.0f);
		const ImGuiIO &io = ImGui::GetIO();

		if (io.MouseWheel != 0.0f &&
		    base_pixels_per_meter > 0.0f &&
		    std::isfinite(base_pixels_per_meter))
		{
			float const previous_zoom = zoom;
			float const min_zoom = (110.0f / 4000.0f) / base_pixels_per_meter;
			float const max_zoom = (110.0f / 12.5f) / base_pixels_per_meter;
			zoom = std::clamp(
				zoom * std::pow(1.05f, io.MouseWheel),
				min_zoom,
				max_zoom);
			translate += (previous_zoom - zoom) * glm::vec2(world_pos.x, world_pos.z);
		}

		if (mode != MODE_MANUAL && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
		{
			glm::vec4 const world_center = glm::inverse(transform) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
			translate = -glm::vec2(world_center.x, world_center.z) * zoom;
			m_manual_rotation = map_rotation;
			mode = MODE_MANUAL;
		}
		if (ImGui::IsMouseDragging(ImGuiMouseButton_Right))
		{
			ImVec2 const delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
			ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
			m_manual_rotation += delta.x * 0.01f;
		}
		if (mode == MODE_MANUAL && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
		{
			ImVec2 const delta_im = ImGui::GetMouseDragDelta();
			ImGui::ResetMouseDragDelta();

			glm::vec4 const screen_delta(
				delta_im.x * 2.0f / surface_size.x,
				0.0f,
				-delta_im.y * 2.0f / surface_size.y,
				0.0f);
			glm::vec4 const map_delta = glm::inverse(transform) * screen_delta;
			translate += glm::vec2(map_delta.x, map_delta.z) * zoom;
		}
		else
		{
			float const map_pick_radius = std::max(
				0.03f / zoom,
				24.0f / (std::min(surface_size.x, surface_size.y) * zoom));
			map::sorted_object_list objects =
				map::Objects.find_in_range(glm::vec3(world_pos.x, NAN, world_pos.z), map_pick_radius);

			if (ImGui::IsMouseReleased(0) &&
			    io.MouseDragMaxDistanceSqr[0] < io.MouseDragThreshold * io.MouseDragThreshold)
			{
				std::shared_ptr<map::map_object> clicked_object;
				if (m_hovered_map_object &&
				    (std::dynamic_pointer_cast<map::semaphore>(m_hovered_map_object) ||
				     std::dynamic_pointer_cast<map::track_switch>(m_hovered_map_object)))
					clicked_object = m_hovered_map_object;
				else if (auto clickable_object = std::find_if(
					         objects.begin(), objects.end(), [](auto const &entry) {
						         return std::dynamic_pointer_cast<map::semaphore>(entry.second) ||
						                std::dynamic_pointer_cast<map::track_switch>(entry.second);
					         });
				         clickable_object != objects.end())
					clicked_object = clickable_object->second;

				if (clicked_object)
				{
					handle_map_object_click(*this, clicked_object);
				}
			}

			if (ImGui::IsMouseReleased(ImGuiMouseButton_Right) &&
			    io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Right] <
			        io.MouseDragThreshold * io.MouseDragThreshold)
			{
				if (m_hovered_map_object &&
				    (std::dynamic_pointer_cast<map::semaphore>(m_hovered_map_object) ||
				     std::dynamic_pointer_cast<map::track_switch>(m_hovered_map_object)))
					handle_map_object_click(*this, m_hovered_map_object);
				else if (objects.size() > 1)
					register_popup(std::make_unique<ui::disambiguation_popup>(*this, std::move(objects)));
				else if (objects.size() == 1)
					handle_map_object_click(*this, objects.begin()->second);
				else
				{
					glm::vec3 nearest = simulation::Region->find_nearest_track_point(world_pos);
					if (!glm::isnan(nearest.x)
					        && glm::distance(glm::vec2(world_pos.x, world_pos.z), glm::vec2(nearest.x, nearest.z)) < 60.0f)
						register_popup(std::make_unique<obstacle_insert_popup>(*this, std::move(nearest)));
				}
			}
			else if (m_hovered_map_object)
			{
				handle_map_object_hover(m_hovered_map_object);
			}
			else if (!objects.empty())
			{
				handle_map_object_hover(objects.begin()->second);
			}
		}

	}

	render_labels(transform, window_origin, surface_size);

	if (surface_size.x >= 40.0f && surface_size.y >= 40.0f)
	{
		draw_list->PushClipRect(
			screen_origin,
			ImVec2(screen_origin.x + surface_size.x, screen_origin.y + surface_size.y),
			true);
		render_camera_frustum(draw_list, screen_origin, surface_size, transform);
		draw_list->PopClipRect();

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
	ImGui::SetCursorScreenPos(ImVec2(screen_origin.x, screen_origin.y + surface_size.y));
	draw_list->ChannelsMerge();
}

void ui::handle_map_object_click(ui_panel &parent, std::shared_ptr<map::map_object> &obj)
{
	if (auto sem = std::dynamic_pointer_cast<map::semaphore>(obj))
	{
		parent.register_popup(std::make_unique<semaphore_popup>(parent, std::move(sem)));
	}
	else if (auto track = std::dynamic_pointer_cast<map::launcher>(obj))
	{
		parent.register_popup(std::make_unique<event_launcher_popup>(parent, std::move(track)));
	}
    else if (auto track_switch = std::dynamic_pointer_cast<map::track_switch>(obj))
    {
        parent.register_popup(std::make_unique<track_switch_popup>(parent, std::move(track_switch)));
    }
	else if (auto obstacle = std::dynamic_pointer_cast<map::obstacle>(obj))
	{
		parent.register_popup(std::make_unique<obstacle_remove_popup>(parent, std::move(obstacle)));
	}
	else if (auto obstacle = std::dynamic_pointer_cast<map::vehicle>(obj))
	{
		parent.register_popup(std::make_unique<vehicle_popup>(parent, std::move(obstacle)));
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

				glm::vec3 const lamp_color = std::get<2>(*state).value_or(glm::vec3(0.5f));
				float const opacity = map_signal_light_opacity(*state);
				if (std::get<0>(*state) < 0.0f)
					continue;

				ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(lamp_color.x, lamp_color.y, lamp_color.z, opacity));

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

ui::semaphore_popup::semaphore_popup(ui_panel &panel, std::shared_ptr<map::semaphore> &&sem) : popup(panel), m_sem(sem) {}

void ui::semaphore_popup::render_content()
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
			float const opacity = map_signal_light_opacity(*state);
			float level = std::get<0>(*state);

			if (level < 0.0f)
				continue;

			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(lamp_color.x, lamp_color.y, lamp_color.z, opacity));
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(lamp_color.x, lamp_color.y, lamp_color.z, opacity));

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

ui::event_launcher_popup::event_launcher_popup(ui_panel &panel, std::shared_ptr<map::launcher> &&sw) : popup(panel), m_switch(sw) {}

void ui::event_launcher_popup::render_content()
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

ui::track_switch_popup::track_switch_popup(ui_panel &panel, std::shared_ptr<map::track_switch> &&sw) : popup(panel), m_switch(sw) {}

bool ui::track_switch_popup::render()
{
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.5f);
    bool ret = popup::render();
    ImGui::PopStyleVar();
    return ret;
}

void ui::track_switch_popup::render_content()
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

ui::obstacle_insert_popup::obstacle_insert_popup(ui_panel &panel, glm::dvec3 const &pos) : popup(panel), m_position(pos)
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

void ui::obstacle_insert_popup::render_content()
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

ui::obstacle_remove_popup::obstacle_remove_popup(ui_panel &panel, std::shared_ptr<map::obstacle> &&obstacle)
    : popup(panel), m_obstacle(obstacle) { }

void ui::obstacle_remove_popup::render_content()
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

ui::vehicle_popup::vehicle_popup(ui_panel &panel, std::shared_ptr<map::vehicle> &&obstacle)
    : popup(panel), m_vehicle(obstacle) { }

void ui::vehicle_popup::render_content()
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
