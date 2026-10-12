#include "stdafx.h"
#include "widgets/map.h"
#include "widgets/map_objects.h"
#include "utilities/Logs.h"
#include "vehicle/Train.h"
#include "vehicle/Camera.h"
#include "simulation/simulation.h"
#include "vehicle/Driver.h"
#include "model/AnimModel.h"
#include "application/application.h"
#include "world/station.h"

ui::map_panel::map_panel() : ui_panel(STR_C("Map"), false)
{
    size_min = {200, 200};
    size_max = {fb_size, fb_size};
    window_flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

#ifdef WITH_OPENGL_MODERN
    opengl33_renderer *gl33 = dynamic_cast<opengl33_renderer *>(GfxRenderer.get());
    if (!gl33)
    {
        ErrorLog("map not supported on old renderer");
        return;
    }
#else
    ErrorLog("map not supported on old renderer"); return;
#endif

    gl::shader vert("map.vert");
    gl::shader frag("map.frag");

    m_track_shader = std::unique_ptr<gl::program>(new gl::program({vert, frag}));
    if (!Global.gfx_usegles || GLAD_GL_EXT_geometry_shader)
    {
        gl::shader poi_frag("map_poi.frag");
        gl::shader poi_geom("map_poi.geom");
        m_poi_shader = std::unique_ptr<gl::program>(new gl::program({vert, poi_frag, poi_geom}));
        m_icon_atlas = GfxRenderer->Fetch_Texture("map_icons");
    }

    m_tex = std::make_unique<opengl_texture>();
    m_tex->alloc_rendertarget(GL_RGB8, GL_RGB, fb_size, fb_size);

    m_fb = std::make_unique<gl::framebuffer>();
    m_fb->attach(*m_tex, GL_COLOR_ATTACHMENT0);

    m_fb->setup_drawing(1);

    if (!m_fb->is_complete())
    {
        ErrorLog("map framebuffer incomplete");
        return;
    }

    if (Global.iMultisampling)
    {
        m_msaa_rb = std::make_unique<gl::renderbuffer>();
        m_msaa_rb->alloc(GL_RGB8, fb_size, fb_size, 1 << Global.iMultisampling);

        m_msaa_fb = std::make_unique<gl::framebuffer>();
        m_msaa_fb->attach(*m_msaa_rb, GL_COLOR_ATTACHMENT0);

        m_msaa_fb->setup_drawing(1);

        if (!m_msaa_fb->is_complete())
        {
            ErrorLog("map multisampling framebuffer incomplete");
            return;
        }
    }

    glGetError();
    glLineWidth(2.0f);
    if (!glGetError())
    {
        m_widelines_supported = true;
    }

    scene_ubo = std::make_unique<gl::ubo>(sizeof(gl::scene_ubs), 0);

    init_done = true;
}

float ui::map_panel::get_vehicle_rotation()
{
    const TDynamicObject *vehicle = simulation::Train->Dynamic();
    glm::vec3 front = glm::dvec3(vehicle->VectorFront()) * (vehicle->DirectionGet() > 0 ? 1.0 : -1.0);
    glm::vec2 north_ptr(0.0f, 1.0f);
    return glm::atan(front.z, front.x) - glm::atan(north_ptr.y, north_ptr.x);
}

glm::dvec3 ui::map_panel::get_vehicle_velocity()
{
    const TDynamicObject *vehicle = simulation::Train->Dynamic();
    glm::dvec3 front = vehicle->VectorFront() * (vehicle->DirectionGet() > 0 ? 1.0 : -1.0);
    return front * std::clamp(vehicle->GetVelocity(), 0.0, 90.0);
}


void ui::map_panel::render_map_texture(glm::mat4 transform, glm::vec2 surface_size)
{
    cFrustum frustum;
    frustum.calculate(transform, glm::mat4());

    m_colored_paths.switches.clear();
    m_colored_paths.occupied.clear();
    m_colored_paths.future.clear();
    m_colored_paths.highlighted.clear();
    m_colored_paths.roads.clear();
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
                {
                    m_section_handles.push_back(handle);
                    section->get_map_active_paths(m_colored_paths);
                    section->get_map_road_paths(m_colored_paths);
                }
            }
        }
    }

    for (TTrack *track: simulation::Paths.sequence())
    {
        track->get_map_future_paths(m_colored_paths);
    }

    for (std::pair<TTrack *, int> &entry: highlighted_switches)
    {
        if (entry.first)
        {
            entry.first->get_map_paths_for_state(m_colored_paths, entry.second);
        }
    }

    glDisable(GL_DEPTH_TEST);
    if (Global.iMultisampling)
    {
        m_msaa_fb->clear(GL_COLOR_BUFFER_BIT);
        m_msaa_fb->bind();
    }
    else
    {
        m_fb->clear(GL_COLOR_BUFFER_BIT);
        m_fb->bind();
    }

    m_track_shader->bind();
    if (m_widelines_supported)
    {
        glLineWidth(1.5f);
    }
    glViewport(0, 0, surface_size.x, surface_size.y);

    scene_ubs.projection = transform;
    scene_ubs.cascade_end = glm::vec4(0.5f, 0.5f, 0.5f, 0.0f);
    scene_ubo->update(scene_ubs);
    scene_ubo->bind_uniform();

#ifdef WITH_OPENGL_MODERN
    opengl33_renderer *gl33 = dynamic_cast<opengl33_renderer *>(GfxRenderer.get());

    gl33->Draw_Geometry(m_section_handles.begin(), m_section_handles.end());

    if (m_widelines_supported)
    {
        glLineWidth(2.5f);
    }
    scene_ubs.cascade_end = glm::vec4(0.4f, 0.4f, 0.4f, 0.0f);
    scene_ubo->update(scene_ubs);
    gl33->Draw_Geometry(m_colored_paths.roads.begin(), m_colored_paths.roads.end());

    if (m_widelines_supported)
    {
        glLineWidth(4.5f);
    }
    scene_ubs.cascade_end = glm::vec4(0.7f, 0.7f, 0.0f, 0.0f);
    scene_ubo->update(scene_ubs);
    gl33->Draw_Geometry(m_colored_paths.future.begin(), m_colored_paths.future.end());

    if (m_widelines_supported)
    {
        glLineWidth(3.0f);
    }
    scene_ubs.cascade_end = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
    scene_ubo->update(scene_ubs);
    gl33->Draw_Geometry(m_colored_paths.switches.begin(), m_colored_paths.switches.end());

    if (m_widelines_supported)
    {
        glLineWidth(1.5f);
    }
    scene_ubs.cascade_end = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
    scene_ubo->update(scene_ubs);
    gl33->Draw_Geometry(m_colored_paths.occupied.begin(), m_colored_paths.occupied.end());

    if (m_widelines_supported)
    {
        glLineWidth(4.0f);
    }
    scene_ubs.cascade_end = glm::vec4(0.3f, 0.3f, 1.0f, 0.0f);
    scene_ubo->update(scene_ubs);
    gl33->Draw_Geometry(m_colored_paths.highlighted.begin(), m_colored_paths.highlighted.end());

    if (!Global.gfx_usegles || GLAD_GL_EXT_geometry_shader)
    {
        gl33->Bind_Texture(0, m_icon_atlas);
        m_poi_shader->bind();
        scene_ubs.cascade_end = glm::vec4(1.0f / (surface_size / 200.0f), 1.0f, 0.0f);
    }

    if (map::Objects.poi_dirty)
    {
        map::Objects.poi_dirty = false;
        simulation::Region->update_poi_geometry();
    }

    scene_ubo->update(scene_ubs);
    gl33->Draw_Geometry(simulation::Region->get_map_poi_geometry());
#endif

    if (Global.iMultisampling)
    {
        m_fb->blit_from(m_msaa_fb.get(), surface_size.x, surface_size.y, GL_COLOR_BUFFER_BIT, GL_COLOR_ATTACHMENT0);
    }
    gl::framebuffer::unbind();
    gl::program::unbind();
}

void ui::map_panel::render_grid(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size)
{
    float grid_size;

    if (zoom > 1.0f)
        grid_size = 10.0f;
    else if (zoom > 0.1f)
        grid_size = 100.0f;
    else if (zoom > 0.01f)
        grid_size = 1000.0f;
    else if (zoom > 0.001f)
        grid_size = 10000.0f;
    else
        return;

    // Odwracamy transform tylko raz.
    const glm::mat4 inverse_transform = glm::inverse(transform);

    // Cztery narożniki viewportu w NDC.
    const glm::vec4 ndc_corners[] = {
        {-10.0f, 0.0f, -1.0f, 1.0f},
        { 10.0f, 0.0f, -1.0f, 1.0f},
        { 10.0f, 0.0f,  1.0f, 1.0f},
        {-10.0f, 0.0f,  1.0f, 1.0f}
    };

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const glm::vec4 &ndc : ndc_corners)
    {
        const glm::vec4 world = inverse_transform * ndc;

        min_x = std::min(min_x, world.x);
        max_x = std::max(max_x, world.x);
        min_z = std::min(min_z, world.z);
        max_z = std::max(max_z, world.z);
    }

    // Pierwsza linia siatki widoczna w zakresie.
    const float first_x = std::floor(min_x / grid_size) * grid_size;
    const float first_z = std::floor(min_z / grid_size) * grid_size;

    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    const ImU32 color = IM_COL32(255, 255, 255, 40);

    auto world_to_screen = [&](float x, float z)
    {
        const glm::vec4 ndc =
            transform * glm::vec4(x, 0.0f, z, 1.0f);

        const glm::vec2 pos =
            (glm::vec2(ndc.x, -ndc.z) * 0.5f + 0.5f) * surface_size;

        return ImVec2(
            origin.x + pos.x,
            origin.y + pos.y
        );
    };

    // Południki - X stałe, Z zmienne.
    for (float x = first_x; x <= max_x; x += grid_size)
    {
        draw_list->AddLine(
            world_to_screen(x, min_z),
            world_to_screen(x, max_z),
            color
        );
    }

    // Równoleżniki - Z stałe, X zmienne.
    for (float z = first_z; z <= max_z; z += grid_size)
    {
        draw_list->AddLine(
            world_to_screen(min_x, z),
            world_to_screen(max_x, z),
            color
        );
    }
}

void ui::map_panel::render_vehicles_labels(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size)
{
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 1.0f, 0.7f, 1.0f));
    for (TDynamicObject *vehicle: simulation::Vehicles.sequence())
    {
        if (vehicle->Prev() || !vehicle->Mechanik)
        {
            continue;
        }
        std::string label = vehicle->Mechanik->TrainName();
        if (label.empty() || label == "none")
            label = ToUpper(vehicle->name());

        glm::vec4 ndc_pos = transform * glm::vec4(glm::vec3(vehicle->GetPosition()), 1.0f);
        if (glm::abs(ndc_pos.x) > 1.0f || glm::abs(ndc_pos.z) > 1.0f)
        {
            continue;
        }
        glm::vec2 gui_pos = (glm::vec2(ndc_pos.x, -ndc_pos.z) / 2.0f + 0.5f) * glm::vec2(surface_size.x, surface_size.y);

        TDynamicObject *veh = vehicle;

        ImVec2 textsize = ImGui::CalcTextSize(label.c_str());
        ImGui::SetCursorPos(ImVec2(origin.x + gui_pos.x - textsize.x / 2.0f, origin.y + gui_pos.y - textsize.y / 2.0f));
        ImGui::TextUnformatted(label.c_str());
    }
    ImGui::PopStyleColor();
}

void ui::map_panel::render_stops_labels(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size)
{
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.7f, 0.7f, 1.0f));
    ImGui::SetWindowFontScale(1.5f);

    std::map<std::string, std::pair<glm::vec3, int>> stop_positions;
    for (basic_event* event : simulation::Events.sequence())
    {
        std::string event_name = event->name();
        if (!event_name.ends_with("_stopinfo"))
        {
            continue;
        }

        std::string stop_name = ToUpper(event_name.substr(0, event_name.find("#")));

        stop_positions[stop_name].first += event->input_location();
        stop_positions[stop_name].second++;
    }

    for (const auto& [stop_name, position] : stop_positions)
    {
        glm::vec3 stop_position = position.first / static_cast<float>(position.second);

        glm::vec4 ndc_pos = transform * glm::vec4(stop_position, 1.0f);

        if (glm::abs(ndc_pos.x) > 1.0f || glm::abs(ndc_pos.z) > 1.0f)
        {
            continue;
        }

        glm::vec2 gui_pos =(glm::vec2(ndc_pos.x, -ndc_pos.z) / 2.0f + 0.5f) *
            glm::vec2(surface_size.x, surface_size.y);

        ImVec2 textsize = ImGui::CalcTextSize(stop_name.c_str());

        ImGui::SetCursorPos(ImVec2(
            origin.x + gui_pos.x - textsize.x / 2.0f,
            origin.y + gui_pos.y - textsize.y / 2.0f
        ));

        ImGui::TextUnformatted(stop_name.c_str());
    }

    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopStyleColor();
}

void ui::map_panel::render_labels(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size)
{
    render_grid(transform,origin,surface_size);
    render_vehicles_labels(transform, origin, surface_size);
    render_stops_labels(transform, origin, surface_size);
}

void ui::map_panel::render_contents()
{
    if (!init_done)
    {
        is_open = false;
        return;
    }

    {
        translate /= zoom;

        if (ImGui::Button("+") || ImGui::IsItemActive())
        {
            zoom *= 1.05f;
        }
        ImGui::SameLine();

        if (ImGui::Button("-") || ImGui::IsItemActive())
        {
            zoom /= 1.05f;
        }
        ImGui::SameLine();

        translate *= zoom;
    }

    glm::mat4 transform;
    transform[0][0] = -1.0f;

    ImGui::RadioButton(STR_C("Pan"), (int *) &mode, 0);
    ImGui::SameLine();
    ImGui::RadioButton(STR_C("Follow camera"), (int *) &mode, 1);
    ImGui::SameLine();
    ImGui::RadioButton(STR_C("Follow vehicle"), (int *) &mode, 2);

    ImVec2 surface_size_im = ImGui::GetContentRegionAvail();
    glm::vec2 surface_size(surface_size_im.x, surface_size_im.y);

    glm::mat4 aspect_transform;
    float aspect = surface_size.y / surface_size.x;

    if (aspect > 1.0f / aspect)
    {
        aspect_transform = glm::scale(glm::mat4(), glm::vec3(aspect, 1.0f, 1.0f));
    }
    else
    {
        aspect_transform = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f / aspect));
    }

    transform *= aspect_transform;

    if (mode == MODE_VEHICLE && simulation::Train)
    {
        float rotation = get_vehicle_rotation();
        glm::dvec3 vel = get_vehicle_velocity();

        transform = glm::rotate(transform, rotation, glm::vec3(0.0f, 1.0f, 0.0f));

        glm::dvec3 position = simulation::Train->Dynamic()->GetPosition() + vel * (0.01 / zoom);
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

    render_map_texture(transform, surface_size);

    ImVec2 window_origin = ImGui::GetCursorPos();
    ImVec2 screen_origin = ImGui::GetCursorScreenPos();

    // Specify alpha for the map image tint so the map window
    // honors the ui.bg.opacity setting like other windows.
    // Make the button frame fully transparent so ImageButton's
    // ImGuiCol_Button background (green and opaque on hover) doesn't
    // show behind the tinted image.
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::ImageButton((ImTextureID) (intptr_t) (m_tex->id), surface_size_im, ImVec2(0, surface_size.y / fb_size), ImVec2(surface_size.x / fb_size, 0), 0, ImVec4(0.0f, 0.0f, 0.0f, 0.0f), ImVec4(1.0f, 1.0f, 1.0f, Global.UIBgOpacity));
    ImGui::PopStyleColor(3);

    if (ImGui::IsItemHovered())
    {
        const ImVec2 screen_pos = ImGui::GetMousePos();
        const glm::vec2 surface_pos(screen_pos.x - screen_origin.x, screen_pos.y - screen_origin.y);
        const glm::vec2 ndc_pos = surface_pos / surface_size * 2.0f - 1.0f;
        const ImGuiIO &io = ImGui::GetIO();

        if (io.MouseWheel != 0.0f)
        {
            translate /= zoom;
            zoom *= std::pow(1.05f, io.MouseWheel);
            translate *= zoom;
        }

        if (FreeFlyModeFlag && ImGui::IsMouseClicked(2))
        {
            glm::vec3 world_pos = glm::inverse(transform) * glm::vec4(ndc_pos.x, 0.0f, -ndc_pos.y, 1.0f);

            Global.FreeCameraInit[0].x = world_pos.x;
            Global.FreeCameraInit[0].y = Global.pCamera.Pos.y;
            Global.FreeCameraInit[0].z = world_pos.z;
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
                    if (!glm::isnan(nearest.x) && glm::distance(glm::vec2(world_pos.x, world_pos.z), glm::vec2(nearest.x, nearest.z)) < 60.0f)
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

    if (const auto sem = std::dynamic_pointer_cast<map::semaphore>(obj))
    {
        for (const auto &model: sem->models)
        {
            ImGui::PushID(model);
            ImGui::TextUnformatted(model->name().c_str());

            for (int i = 0; i < iMaxNumLights; i++)
            {
#ifdef WITH_OPENGL_MODERN
                dynamic_cast<opengl33_renderer *>(GfxRenderer.get())->Update_AnimModel(model); // update lamp opacities
#endif
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

ui::disambiguation_popup::disambiguation_popup(ui_panel &panel, map::sorted_object_list &&list) : popup(panel), m_list(list)
{
}

void ui::disambiguation_popup::render_content()
{
    for (auto &item: m_list)
    {
        if (ImGui::Button(item.second->name.c_str()))
        {
            ImGui::CloseCurrentPopup();

            handle_map_object_click(m_parent, item.second);
        }
    }
}

ui::semaphore_window::semaphore_window(ui_panel &panel, std::shared_ptr<map::semaphore> &&sem) : popup(panel), m_sem(sem)
{
}

void ui::semaphore_window::render_content()
{
    for (auto &model: m_sem->models)
    {
        ImGui::PushID(model);
        ImGui::TextUnformatted(model->name().c_str());

        for (int i = 0; i < iMaxNumLights; i++)
        {
#ifdef WITH_OPENGL_MODERN
            dynamic_cast<opengl33_renderer *>(GfxRenderer.get())->Update_AnimModel(model); // update lamp opacities
#endif
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

                m_relay.post(user_command::setlight, (double) i, level, GLFW_PRESS, 0, glm::vec3(0.0f), &model->name());
            }

            ImGui::PopStyleColor(2);
        }

        ImGui::PopID();
    }

    ImGui::Separator();

    for (auto &item: m_sem->events)
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

ui::launcher_window::launcher_window(ui_panel &panel, std::shared_ptr<map::launcher> &&sw) : popup(panel), m_switch(sw)
{
}

void ui::launcher_window::render_content()
{
    ImGui::TextUnformatted(m_switch->name.c_str());

    const std::string &open_label = m_switch->type == map::launcher::track_switch ? STR("Straight |") : STR("Open |");

    const std::string &close_label = m_switch->type == map::launcher::track_switch ? STR("Divert /") : STR("Close -");

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

ui::track_switch_window::track_switch_window(ui_panel &panel, std::shared_ptr<map::track_switch> &&sw) : popup(panel), m_switch(sw)
{
}

bool ui::track_switch_window::render()
{
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.5f);
    bool ret = popup::render();
    ImGui::PopStyleVar();
    return ret;
}

void ui::track_switch_window::render_content()
{
    auto &highlight = dynamic_cast<map_panel &>(m_parent).highlighted_switches;

    ImGui::TextUnformatted(m_switch->name.c_str());

    highlight.clear();

    std::array<std::string, 4> names = {"ac", "ad", "bc", "bd"};

    for (size_t i = 0; i < 4; i++)
    {
        if (m_switch->action[i])
        {
            if (ImGui::Button(names[i].c_str()))
                m_relay.post(user_command::queueevent, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(0.0f), &m_switch->action[i]->name());
            if (ImGui::IsItemHovered())
            {
                for (size_t j = 0; j < 4; j++)
                {
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
    if (m_obstacles.empty())
    {
        ImGui::CloseCurrentPopup();
        return;
    }

    ImGui::TextUnformatted(STR_C("Insert obstacle:"));
    for (auto const &entry: m_obstacles)
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

            std::vector<gfx::basic_vertex> vertices;
            vertices.emplace_back(std::move(obstacle->vertex()));
            gfx::userdata_array userdata{};
            GfxRenderer->Append(vertices, userdata, simulation::Region->get_map_poi_geometry(), GL_POINTS);

            map::Objects.entries.push_back(std::move(obstacle));

            ImGui::CloseCurrentPopup();
        }
    }
}

ui::obstacle_remove_window::obstacle_remove_window(ui_panel &panel, std::shared_ptr<map::obstacle> &&obstacle) : popup(panel), m_obstacle(obstacle)
{
}

void ui::obstacle_remove_window::render_content()
{
    if (ImGui::Button(STR_C("Delete obstacle")))
    {
        m_relay.post(user_command::deletemodel, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(), &m_obstacle->model_name);

        auto &entries = map::Objects.entries;
        for (auto it = entries.rbegin(); it != entries.rend(); it++)
        {
            if (*it == m_obstacle)
            {
                entries.erase(std::next(it).base());
                break;
            }
        }

        map::Objects.poi_dirty = true;

        ImGui::CloseCurrentPopup();
    }
}

ui::vehicle_click_window::vehicle_click_window(ui_panel &panel, std::shared_ptr<map::vehicle> &&obstacle) : popup(panel), m_vehicle(obstacle)
{
}

void ui::vehicle_click_window::render_content()
{
    ImGui::TextUnformatted(STR_C("Light signal:"));

    if (ImGui::Button(STR_C("none")))
    {
        std::string name = m_vehicle->name + "%SetSignal";
        m_relay.post(user_command::sendaicommand, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(), &name);

        ImGui::CloseCurrentPopup();
    }
    if (ImGui::Button(STR_C("Pc6")))
    {
        std::string name = m_vehicle->name + "%SetSignal";
        m_relay.post(user_command::sendaicommand, Signal_Pc6, 1.0, GLFW_PRESS, 0, glm::vec3(), &name);

        ImGui::CloseCurrentPopup();
    }
    if (ImGui::Button(STR_C("A1")))
    {
        std::string name = m_vehicle->name + "%SetSignal";
        m_relay.post(user_command::sendaicommand, Signal_A1, 1.0, GLFW_PRESS, 0, glm::vec3(), &name);

        ImGui::CloseCurrentPopup();
    }

    if (DebugModeFlag)
    {
        ImGui::NewLine();
        ImGui::TextUnformatted(STR_C("Send command:"));
        ImGui::InputText(STR_C("Command"), command_buf.data(), command_buf.size());

        static std::vector<std::string> possible_commands = {"CabSignal", "Overhead", "Emergency_brake", "Timetable:", "SetVelocity", "SetProximityVelocity", "ShuntVelocity", "Wait_for_orders", "Prepare_engine", "Change_direction", "Obey_train", "Bank", "Shunt", "Loose_shunt", "Jump_to_first_order", "Jump_to_order", "Warning_signal", "Radio_channel", "SetLights", "SetSignal"};

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
        if (ImGui::Button(STR_C("Send")))
        {
            std::string name = m_vehicle->name + "%" + std::string(command_buf.data());
            m_relay.post(user_command::sendaicommand, command_param1, command_param2, GLFW_PRESS, 0, glm::vec3(), &name);

            ImGui::CloseCurrentPopup();
        }
    }
}
