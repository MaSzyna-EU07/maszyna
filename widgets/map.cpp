#include "stdafx.h"
#include "widgets/map.h"
#include "widgets/map_objects.h"
#include "vehicle/Train.h"
#include "vehicle/Camera.h"
#include "simulation/simulation.h"
#include "vehicle/Driver.h"
#include "model/AnimModel.h"
#include "application/application.h"
#include "world/Sweep.h"

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

void ui::map_panel::render_map_lines(ImDrawList *draw_list, ImVec2 origin, glm::vec2 surface_size, glm::mat4 transform)
{
	const auto to_screen = [&](glm::vec3 const &position) {
		glm::vec4 ndc = transform * glm::vec4(position, 1.0f);
		return ImVec2(origin.x + (ndc.x + 1.0f) * 0.5f * surface_size.x,
		              origin.y + (1.0f - ndc.z) * 0.5f * surface_size.y);
	};

	const auto draw_geometry = [&](gfx::geometrybank_handle const &handle, ImU32 color) {
		if (handle == null_handle)
			return;

		auto const &vertices = GfxRenderer->Vertices(handle);
		for (std::size_t idx = 0; idx + 1 < vertices.size(); idx += 2)
			draw_list->AddLine(to_screen(vertices[idx].position), to_screen(vertices[idx + 1].position), color);
	};

	for (auto const &handle : m_section_handles)
		draw_geometry(handle, IM_COL32(190, 195, 200, 255));
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
			distance = std::min(distance + 3.0, end);
			auto const near_current = edge_position(distance, near_side);
			auto const far_current = edge_position(distance, far_side);
			draw_list->AddLine(to_screen(near_previous), to_screen(near_current), IM_COL32(255, 190, 90, 255), 1.5f);
			draw_list->AddLine(to_screen(far_previous), to_screen(far_current), IM_COL32(220, 155, 75, 220), 1.0f);
			near_previous = near_current;
			far_previous = far_current;
		}
		draw_list->AddLine(to_screen(edge_position(start, near_side)), to_screen(edge_position(start, far_side)), IM_COL32(255, 225, 160, 255), 1.0f);
		draw_list->AddLine(to_screen(near_previous), to_screen(far_previous), IM_COL32(255, 225, 160, 255), 1.0f);
	}
	draw_list->PopClipRect();

	for (TDynamicObject *vehicle : simulation::Vehicles.sequence())
	{
		if (!vehicle->MyTrack || (vehicle->MyTrack->iCategoryFlag & 15) != 1)
			continue;

		glm::vec4 ndc_pos = transform * glm::vec4(glm::vec3(vehicle->GetPosition()), 1.0f);
		if (glm::abs(ndc_pos.x) > 1.0f || glm::abs(ndc_pos.z) > 1.0f)
			continue;

		ImVec2 const vehicle_pos(
		    map_screen_origin.x + (ndc_pos.x + 1.0f) * 0.5f * surface_size.x,
		    map_screen_origin.y + (1.0f - ndc_pos.z) * 0.5f * surface_size.y);
		bool const is_train_front = vehicle->Prev() == nullptr;
		float const marker_radius = is_train_front ? 4.0f : 3.0f;
		draw_list->AddCircleFilled(vehicle_pos, marker_radius, IM_COL32(100, 255, 130, 255));
		draw_list->AddCircle(vehicle_pos, marker_radius, IM_COL32(20, 35, 25, 255), 0, 1.0f);
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

	{
		float prev_zoom = zoom;

		if (ImGui::Button("+"))
			zoom *= 2.0f;
		ImGui::SameLine();

		if (ImGui::Button("-"))
			zoom /= 2.0f;
		ImGui::SameLine();

		float x = zoom / prev_zoom;
		translate *= x;
	}

	glm::mat4 transform;
	transform[0][0] = -1.0f;

    ImGui::RadioButton(STR_C("Pan"), (int *)&mode, 0);
	ImGui::SameLine();
    ImGui::RadioButton(STR_C("Follow camera"), (int *)&mode, 1);
	ImGui::SameLine();
    ImGui::RadioButton(STR_C("Follow vehicle"), (int *)&mode, 2);

	ImVec2 surface_size_im = ImGui::GetContentRegionAvail();
	glm::vec2 surface_size(surface_size_im.x, surface_size_im.y);

	glm::mat4 aspect_transform;
	float aspect = surface_size.y / surface_size.x;

	if (aspect > 1.0f / aspect)
		aspect_transform = glm::scale(glm::mat4(), glm::vec3(aspect, 1.0f, 1.0f));
	else
		aspect_transform = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f / aspect));

	transform *= aspect_transform;

	if (mode == MODE_VEHICLE && simulation::Train)
	{
		float rotation = get_vehicle_rotation();

		transform = glm::rotate(transform, rotation, glm::vec3(0.0f, 1.0f, 0.0f));

		glm::dvec3 position = simulation::Train->Dynamic()->GetPosition();
		translate = glm::vec2(position.x, position.z) * -zoom;
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
	render_map_lines(draw_list, screen_origin, surface_size, transform);
	draw_list->PopClipRect();
	ImGui::InvisibleButton("##map", surface_size_im);

	if (ImGui::IsItemHovered())
	{
		const ImVec2 screen_pos = ImGui::GetMousePos();
		const glm::vec2 surface_pos(screen_pos.x - screen_origin.x, screen_pos.y - screen_origin.y);
		const glm::vec2 ndc_pos = surface_pos / surface_size * 2.0f - 1.0f;
		const ImGuiIO &io = ImGui::GetIO();
		const auto relocate_camera = [&] {
			glm::vec3 const world_pos = glm::inverse(transform) * glm::vec4(ndc_pos.x, 0.0f, -ndc_pos.y, 1.0f);
			glm::dvec3 const camera_position(world_pos.x, Global.pCamera.Pos.y, world_pos.z);
			Global.FreeCameraInit[0] = camera_position;
			Global.FreeCameraTeleport = camera_position;
			Global.pCamera.Pos = camera_position;
		};

		if (io.MouseWheel != 0.0f)
		{
			float prev_zoom = zoom;
			zoom *= std::pow(1.2, io.MouseWheel);
			float x = zoom / prev_zoom;

			glm::vec4 corrected = glm::inverse(aspect_transform) * glm::vec4(ndc_pos.x, 0.0f, ndc_pos.y, 1.0f);
			translate += glm::vec2(corrected.x, corrected.z);
			translate *= x;

			glm::vec2 surface_screen_center = glm::vec2(screen_origin.x, screen_origin.y) + surface_size / 2.0f;
			Application.set_cursor_pos(surface_screen_center.x, surface_screen_center.y);
		}

		if (FreeFlyModeFlag && Global.pCamera.m_owner == nullptr
			&& ImGui::IsMouseReleased(0)
			&& io.MouseDragMaxDistanceSqr[0] < io.MouseDragThreshold * io.MouseDragThreshold)
			relocate_camera();
		if (FreeFlyModeFlag && Global.pCamera.m_owner == nullptr && ImGui::IsMouseClicked(2))
			relocate_camera();
		if (mode == 0 && ImGui::IsMouseDragging(0))
		{
			ImVec2 delta_im = ImGui::GetMouseDragDelta();
			ImGui::ResetMouseDragDelta();

			glm::vec2 delta(delta_im.x, delta_im.y);

			translate -= delta / surface_size * 2.0f;
		}
		else
		{
			glm::vec3 world_pos = glm::inverse(transform) * glm::vec4(ndc_pos.x, 0.0f, -ndc_pos.y, 1.0f);

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
	}

	render_labels(transform, window_origin, surface_size);
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
