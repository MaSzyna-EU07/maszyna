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

#include <array>
#include <limits>

namespace
{

// what an object of the map is, for the map of RmlUi to tell of it
ui::map_panel::object_info describe(map::map_object const &object)
{
	ui::map_panel::object_info info;
	info.name = object.name;
	info.kind = ui::map_actions::kind(object);
	if (auto const *semaphore = dynamic_cast<map::semaphore const *>(&object))
	{
		for (auto *model : semaphore->models)
		{
			std::vector<glm::vec3> colours;
			for (auto const &lamp : ui::map_actions::lamps(model))
				colours.push_back(lamp.colour);
			info.heads.emplace_back(model->name(), std::move(colours));
		}
	}
	return info;
}

} // namespace

std::string ui::map_actions::kind(map::map_object const &object)
{
	if (dynamic_cast<map::semaphore const *>(&object) != nullptr)
		return "Signal";
	if (auto const *launcher = dynamic_cast<map::launcher const *>(&object))
		return launcher->type == map::launcher::track_switch ? "Switch" : "Level crossing";
	if (dynamic_cast<map::track_switch const *>(&object) != nullptr)
		return "Switch";
	if (dynamic_cast<map::obstacle const *>(&object) != nullptr)
		return "Obstacle";
	if (dynamic_cast<map::vehicle const *>(&object) != nullptr)
		return "Vehicle";
	return "Object";
}

ui::map_panel::map_panel() : ui_panel(STR_C("Map"), false)
{
	size_min = {200, 200};
	size_max = {fb_size, fb_size};
	window_flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

#ifdef WITH_OPENGL_MODERN
    // the map is drawn with the geometry banks and the framebuffers of the opengl renderer
    opengl46_renderer *renderer = dynamic_cast<opengl46_renderer*>(GfxRenderer.get());
    if (!renderer) {
        WriteLog("map: drawn by the opengl renderer only");
        return;
    }
#else
    ErrorLog("map not supported on old renderer");
    return;
#endif

	gl::shader vert("map.vert");
	gl::shader frag("map.frag");

	m_track_shader = std::unique_ptr<gl::program>(new gl::program({vert, frag}));
	gl::shader poi_frag("map_poi.frag");
	gl::shader poi_geom("map_poi.geom");
	m_poi_shader = std::unique_ptr<gl::program>(new gl::program({vert, poi_frag, poi_geom}));
    m_icon_atlas = GfxRenderer->Fetch_Texture("map_icons");

	if (!make_targets(fb_size))
		return;

	scene_ubo = std::make_unique<gl::ubo>(sizeof(gl::scene_ubs), 0);

	init_done = true;
}

bool ui::map_panel::make_targets(int const Size)
{
	m_tex = std::make_unique<opengl_texture>();
    m_tex->alloc_rendertarget(GL_RGB8, GL_RGB, Size, Size);

	m_fb = std::make_unique<gl::framebuffer>();
	m_fb->attach(*m_tex, GL_COLOR_ATTACHMENT0);

	m_fb->setup_drawing(1);

	if (!m_fb->is_complete())
	{
		ErrorLog("map framebuffer incomplete");
		return false;
	}

	if (Global.iMultisampling)
	{
		m_msaa_rb = std::make_unique<gl::renderbuffer>();
		m_msaa_rb->alloc(GL_RGB8, Size, Size, 1 << Global.iMultisampling);

		m_msaa_fb = std::make_unique<gl::framebuffer>();
		m_msaa_fb->attach(*m_msaa_rb, GL_COLOR_ATTACHMENT0);

		m_msaa_fb->setup_drawing(1);

		if (!m_msaa_fb->is_complete())
		{
			ErrorLog("map multisampling framebuffer incomplete");
			return false;
		}
	}

	fb_size = Size;
	return true;
}

float ui::map_panel::get_vehicle_rotation()
{
	return get_vehicle_rotation(simulation::Train->Dynamic());
}

float ui::map_panel::get_vehicle_rotation(TDynamicObject const *vehicle)
{
	glm::vec3 front = glm::dvec3(vehicle->VectorFront()) * (vehicle->DirectionGet() > 0 ? 1.0 : -1.0);
	glm::vec2 north_ptr(0.0f, 1.0f);
	return glm::atan(front.z, front.x) - glm::atan(north_ptr.y, north_ptr.x);
}

TDynamicObject const *ui::map_panel::followed_vehicle() const
{
	if (!m_followed.empty())
		return simulation::Vehicles.find(m_followed);
	return simulation::Train ? simulation::Train->Dynamic() : nullptr;
}

void ui::map_panel::draw_paths(std::vector<gfx::geometrybank_handle> &paths, glm::vec3 const &colour, glm::mat4 const &transform, glm::vec2 surface_size, int thickness)
{
#ifdef WITH_OPENGL_MODERN
    opengl46_renderer *renderer = dynamic_cast<opengl46_renderer*>(GfxRenderer.get());

	scene_ubs.cascade_end = glm::vec4(colour, 0.0f);
	// a thick line is drawn again moved by a pixel to each side, the map shader draws thin lines only
	for (int x = -thickness; x <= thickness; ++x)
	{
		for (int y = -thickness; y <= thickness; ++y)
		{
			if (thickness > 0 && std::abs(x) + std::abs(y) > thickness)
				continue;
			glm::vec3 const offset(x * 2.0f / surface_size.x, 0.0f, y * 2.0f / surface_size.y);
			scene_ubs.projection = glm::translate(glm::mat4(1.0f), offset) * transform;
			scene_ubo->update(scene_ubs);
			renderer->Draw_Geometry(paths.begin(), paths.end());
		}
	}
	scene_ubs.projection = transform;
#endif
}

void ui::map_panel::render_map_texture(glm::mat4 transform, glm::vec2 surface_size)
{
	render_map_texture(transform, surface_size, style{});
}

void ui::map_panel::render_map_texture(glm::mat4 transform, glm::vec2 surface_size, style const &Style)
{
	cFrustum frustum;
	frustum.calculate(transform, glm::mat4());

	m_colored_paths.switches.clear();
	m_colored_paths.occupied.clear();
	m_colored_paths.future.clear();
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
				{
					m_section_handles.push_back(handle);
					section->get_map_active_paths(m_colored_paths);
				}
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

	glDisable(GL_DEPTH_TEST);
	// the background of the style, the clear colour of the renderer left as it was
	std::array<GLfloat, 4> clearcolour{};
	if (Style.background)
	{
		glGetFloatv(GL_COLOR_CLEAR_VALUE, clearcolour.data());
		glClearColor(Style.background->r, Style.background->g, Style.background->b, 1.0f);
	}
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
	if (Style.background)
		glClearColor(clearcolour[0], clearcolour[1], clearcolour[2], clearcolour[3]);

	m_track_shader->bind();
	glViewport(0, 0, surface_size.x, surface_size.y);

	scene_ubs.projection = transform;
	scene_ubo->bind_uniform();

	draw_paths(m_section_handles, Style.track, transform, surface_size, Style.thickness);
	draw_paths(m_colored_paths.future, Style.future, transform, surface_size, Style.thickness);
	draw_paths(m_colored_paths.switches, Style.switches, transform, surface_size, Style.thickness);
	draw_paths(m_colored_paths.occupied, Style.occupied, transform, surface_size, Style.thickness);
	draw_paths(m_colored_paths.highlighted, Style.highlighted, transform, surface_size, Style.thickness);

#ifdef WITH_OPENGL_MODERN
	if (Style.icons)
	{
		opengl46_renderer *renderer = dynamic_cast<opengl46_renderer*>(GfxRenderer.get());
		renderer->Bind_Texture(0, m_icon_atlas);
		m_poi_shader->bind();
		scene_ubs.cascade_end = glm::vec4(1.0f / (surface_size / 200.0f), 1.0f, 0.0f);

		if (map::Objects.poi_dirty) {
			map::Objects.poi_dirty = false;
			simulation::Region->update_poi_geometry();
		}

		scene_ubo->update(scene_ubs);
		renderer->Draw_Geometry(simulation::Region->get_map_poi_geometry());
	}
#endif

	if (Global.iMultisampling)
		m_fb->blit_from(m_msaa_fb.get(), surface_size.x, surface_size.y, GL_COLOR_BUFFER_BIT, GL_COLOR_ATTACHMENT0);

	gl::framebuffer::unbind();
	gl::program::unbind();
}

std::vector<ui::map_panel::label> ui::map_panel::collect_labels(glm::mat4 const &transform, glm::vec2 surface_size) const
{
	std::vector<label> labels;
	for (TDynamicObject *vehicle : simulation::Vehicles.sequence())
	{
		if (vehicle->Prev() || !vehicle->Mechanik)
			continue;

		std::string text = vehicle->Mechanik->TrainName();
		if (text.empty() || text == "none")
			text = ToUpper(vehicle->name());

		glm::vec4 ndc_pos = transform * glm::vec4(glm::vec3(vehicle->GetPosition()), 1.0f);
		if (glm::abs(ndc_pos.x) > 1.0f || glm::abs(ndc_pos.z) > 1.0f)
			continue;

		glm::vec2 gui_pos = (glm::vec2(ndc_pos.x, -ndc_pos.z) / 2.0f + 0.5f) * glm::vec2(surface_size.x, surface_size.y);
		labels.emplace_back(std::move(text), gui_pos);
	}
	return labels;
}

void ui::map_panel::render_labels(glm::mat4 transform, ImVec2 origin, glm::vec2 surface_size)
{
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 1.0f, 0.7f, 1.0f));
	for (auto const &label : collect_labels(transform, surface_size))
	{
		ImVec2 textsize = ImGui::CalcTextSize(label.text.c_str());
		ImGui::SetCursorPos(ImVec2(origin.x + label.position.x - textsize.x / 2.0f, origin.y + label.position.y - textsize.y / 2.0f));
		ImGui::TextUnformatted(label.text.c_str());
	}
	ImGui::PopStyleColor();
}

bool ui::map_panel::fit_targets(glm::vec2 surface_size)
{
	// the map over the whole screen is bigger than the image a window has
	if (auto const needed{static_cast<int>(std::ceil(std::max(surface_size.x, surface_size.y)))}; needed > fb_size && !make_targets((needed + 255) / 256 * 256))
	{
		init_done = false;
		return false;
	}
	return true;
}

ui::map_panel::view ui::map_panel::view_of(glm::vec2 surface_size)
{
	view result;
	result.transform[0][0] = -1.0f;

	float aspect = surface_size.y / surface_size.x;

	if (aspect > 1.0f / aspect)
		result.aspect = glm::scale(glm::mat4(), glm::vec3(aspect, 1.0f, 1.0f));
	else
		result.aspect = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f / aspect));

	result.transform *= result.aspect;

	// the vehicle driven, or the one chosen on the map of RmlUi
	if (auto const *followed{mode == MODE_VEHICLE ? followed_vehicle() : nullptr}; followed != nullptr)
	{
		float rotation = get_vehicle_rotation(followed);

		result.transform = glm::rotate(result.transform, rotation, glm::vec3(0.0f, 1.0f, 0.0f));

		glm::dvec3 position = followed->GetPosition();
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

		result.transform = glm::rotate(result.transform, rotation, glm::vec3(0.0f, 1.0f, 0.0f));

		glm::dvec3 position = Global.pCamera.Pos;
		translate = glm::vec2(position.x, position.z) * -zoom;
	}

	result.transform = glm::translate(result.transform, glm::vec3(translate.x, 0.0f, translate.y));
	result.transform = glm::scale(result.transform, glm::vec3(zoom));
	return result;
}

glm::vec3 ui::map_panel::world_at(view const &View, glm::vec2 ndc_pos)
{
	return glm::inverse(View.transform) * glm::vec4(ndc_pos.x, 0.0f, -ndc_pos.y, 1.0f);
}

void ui::map_panel::open_objects(glm::vec3 const &world_pos, map::sorted_object_list &&objects)
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

    ImGui::RadioButton(STR_C("Pan"), (int *)&mode, 0);
	ImGui::SameLine();
    ImGui::RadioButton(STR_C("Follow camera"), (int *)&mode, 1);
	ImGui::SameLine();
    ImGui::RadioButton(STR_C("Follow vehicle"), (int *)&mode, 2);

	ImVec2 surface_size_im = ImGui::GetContentRegionAvail();
	glm::vec2 surface_size(surface_size_im.x, surface_size_im.y);

	if (!fit_targets(surface_size))
	{
		is_open = false;
		return;
	}

	auto const view{view_of(surface_size)};

	render_map_texture(view.transform, surface_size);

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
	// imgui 1.89+ takes the frame padding of the image button from the style
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
	ImGui::ImageButton("##map",
					   (ImTextureID)(intptr_t)(m_tex->id),
					   surface_size_im,
					   ImVec2(0, surface_size.y / fb_size),
					   ImVec2(surface_size.x / fb_size, 0),
					   ImVec4(0.0f, 0.0f, 0.0f, 0.0f),
					   ImVec4(1.0f, 1.0f, 1.0f, Global.UIBgOpacity));
	ImGui::PopStyleVar();
	ImGui::PopStyleColor(3);

	if (ImGui::IsItemHovered())
	{
		const ImVec2 screen_pos = ImGui::GetMousePos();
		const glm::vec2 surface_pos(screen_pos.x - screen_origin.x, screen_pos.y - screen_origin.y);
		const glm::vec2 ndc_pos = surface_pos / surface_size * 2.0f - 1.0f;
		const ImGuiIO &io = ImGui::GetIO();

		if (io.MouseWheel != 0.0f)
		{
			float prev_zoom = zoom;
			zoom *= std::pow(2.0, io.MouseWheel);
			float x = zoom / prev_zoom;

			glm::vec4 corrected = glm::inverse(view.aspect) * glm::vec4(ndc_pos.x, 0.0f, ndc_pos.y, 1.0f);
			translate += glm::vec2(corrected.x, corrected.z);
			translate *= x;

			glm::vec2 surface_screen_center = glm::vec2(screen_origin.x, screen_origin.y) + surface_size / 2.0f;
			Application.set_cursor_pos(surface_screen_center.x, surface_screen_center.y);
		}

        if (FreeFlyModeFlag && ImGui::IsMouseClicked(2)) {
            glm::vec3 world_pos = world_at(view, ndc_pos);

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
			glm::vec3 world_pos = world_at(view, ndc_pos);

			map::sorted_object_list objects = map::Objects.find_in_range(glm::vec3(world_pos.x, NAN, world_pos.z), 0.03f / zoom);

			if (ImGui::IsMouseClicked(1))
			{
				open_objects(world_pos, std::move(objects));
			}
			else if (!objects.empty())
			{
				handle_map_object_hover(objects.begin()->second);
			}
		}
	}

	render_labels(view.transform, window_origin, surface_size);
}

std::optional<ui::map_panel::image> ui::map_panel::draw(glm::vec2 surface_size, style const &Style)
{
	if (!init_done || surface_size.x < 1.0f || surface_size.y < 1.0f || !fit_targets(surface_size))
		return std::nullopt;
	m_view = view_of(surface_size);
	m_surface = surface_size;
	render_map_texture(m_view.transform, surface_size, Style);
	return image{(ImTextureID)(intptr_t)(m_tex->id), ImVec2(0, surface_size.y / fb_size), ImVec2(surface_size.x / fb_size, 0)};
}

glm::vec2 ui::map_panel::to_surface(glm::dvec3 const &world) const
{
	glm::vec4 ndc_pos = m_view.transform * glm::vec4(glm::vec3(world), 1.0f);
	return (glm::vec2(ndc_pos.x, -ndc_pos.z) / 2.0f + 0.5f) * m_surface;
}

std::vector<ui::map_panel::train> ui::map_panel::trains() const
{
	std::vector<train> result;
	// the vehicles the drivers of the consists are in
	for (TDynamicObject const *vehicle : simulation::Vehicles.sequence())
	{
		if (!vehicle->Mechanik || !vehicle->Mechanik->primary())
			continue;

		std::string name = vehicle->Mechanik->TrainName();
		if (name.empty() || name == "none")
			name = ToUpper(vehicle->name());

		glm::dvec3 const position = vehicle->GetPosition();
		glm::vec2 const at = to_surface(position);
		bool const shown = at.x >= 0.0f && at.y >= 0.0f && at.x <= m_surface.x && at.y <= m_surface.y;
		glm::dvec3 const front = glm::dvec3(vehicle->VectorFront()) * (vehicle->DirectionGet() > 0 ? 1.0 : -1.0);
		glm::vec2 const ahead = to_surface(position + front * 10.0);
		float const heading = glm::degrees(std::atan2(ahead.x - at.x, at.y - ahead.y));
		result.emplace_back(vehicle, std::move(name), at, shown, heading);
	}
	return result;
}

float ui::map_panel::pixels_per_metre() const
{
	glm::dvec3 const centre = glm::dvec3(-translate.x / zoom, 0.0, -translate.y / zoom);
	return glm::length(to_surface(centre + glm::dvec3(1000.0, 0.0, 0.0)) - to_surface(centre)) / 1000.0f;
}

std::vector<ui::map_panel::segment> ui::map_panel::grid(float step) const
{
	std::vector<segment> lines;
	if (step <= 0.0f || m_surface.x < 1.0f || m_surface.y < 1.0f)
		return lines;
	// the part of the world seen, from the corners of the surface
	glm::vec2 low(std::numeric_limits<float>::max());
	glm::vec2 high(std::numeric_limits<float>::lowest());
	for (glm::vec2 const corner : {glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, -1.0f), glm::vec2(-1.0f, 1.0f), glm::vec2(1.0f, 1.0f)})
	{
		glm::vec3 const world = world_at(m_view, corner);
		low = glm::min(low, glm::vec2(world.x, world.z));
		high = glm::max(high, glm::vec2(world.x, world.z));
	}
	int const firstx = static_cast<int>(std::ceil(low.x / step));
	int const lastx = std::min(static_cast<int>(std::floor(high.x / step)), firstx + 100);
	for (int line = firstx; line <= lastx; ++line)
		lines.emplace_back(to_surface(glm::dvec3(line * step, 0.0, low.y)), to_surface(glm::dvec3(line * step, 0.0, high.y)));
	int const firstz = static_cast<int>(std::ceil(low.y / step));
	int const lastz = std::min(static_cast<int>(std::floor(high.y / step)), firstz + 100);
	for (int line = firstz; line <= lastz; ++line)
		lines.emplace_back(to_surface(glm::dvec3(low.x, 0.0, line * step)), to_surface(glm::dvec3(high.x, 0.0, line * step)));
	return lines;
}

std::optional<ui::map_panel::object_info> ui::map_panel::object_at(glm::vec2 surface_pos) const
{
	if (m_surface.x < 1.0f || m_surface.y < 1.0f)
		return std::nullopt;
	glm::vec3 world_pos = world_at(m_view, surface_pos / m_surface * 2.0f - 1.0f);
	map::sorted_object_list objects = map::Objects.find_in_range(glm::vec3(world_pos.x, NAN, world_pos.z), 0.03f / zoom);
	if (objects.empty())
		return std::nullopt;
	return describe(*objects.begin()->second);
}

map::sorted_object_list ui::map_panel::objects_at(glm::vec2 surface_pos) const
{
	if (m_surface.x < 1.0f || m_surface.y < 1.0f)
		return {};
	glm::vec3 world_pos = world_at(m_view, surface_pos / m_surface * 2.0f - 1.0f);
	return map::Objects.find_in_range(glm::vec3(world_pos.x, NAN, world_pos.z), 0.03f / zoom);
}

std::shared_ptr<map::semaphore> ui::map_panel::semaphore_at(glm::dvec3 const &location)
{
	for (auto const &entry : map::Objects.entries)
	{
		auto semaphore = std::dynamic_pointer_cast<map::semaphore>(entry);
		if (semaphore && glm::distance(glm::dvec3(semaphore->location), location) < 1.0)
			return semaphore;
	}
	return nullptr;
}

std::optional<glm::dvec3> ui::map_panel::track_point_at(glm::vec2 surface_pos) const
{
	if (m_surface.x < 1.0f || m_surface.y < 1.0f)
		return std::nullopt;
	glm::vec3 world_pos = world_at(m_view, surface_pos / m_surface * 2.0f - 1.0f);
	glm::vec3 nearest = simulation::Region->find_nearest_track_point(world_pos);
	if (glm::isnan(nearest.x) || glm::distance(glm::vec2(world_pos.x, world_pos.z), glm::vec2(nearest.x, nearest.z)) >= 60.0f)
		return std::nullopt;
	return glm::dvec3(nearest);
}

std::vector<ui::map_panel::vehicle_mark> ui::map_panel::vehicles(float shortest) const
{
	std::vector<vehicle_mark> marks;
	auto const scale = pixels_per_metre();
	if (scale <= 0.0f)
		return marks;
	// the consist driven, from the vehicle driven to both ends
	std::vector<TDynamicObject const *> own;
	if (auto const *driven = (simulation::Train ? simulation::Train->Dynamic() : nullptr))
	{
		for (auto const *vehicle = driven; vehicle != nullptr; vehicle = vehicle->Prev())
			own.push_back(vehicle);
		for (auto const *vehicle = driven->Next(); vehicle != nullptr; vehicle = vehicle->Next())
			own.push_back(vehicle);
	}
	for (TDynamicObject const *vehicle : simulation::Vehicles.sequence())
	{
		auto const length = static_cast<float>(vehicle->MoverParameters->Dim.L) * scale;
		if (length < shortest)
			continue;
		glm::dvec3 const position = vehicle->GetPosition();
		glm::vec2 const at = to_surface(position);
		// those off the surface left out, with a margin of their length
		if (at.x < -length || at.y < -length || at.x > m_surface.x + length || at.y > m_surface.y + length)
			continue;
		glm::vec2 const ahead = to_surface(position + glm::dvec3(vehicle->VectorFront()) * 10.0);
		vehicle_mark mark;
		mark.position = at;
		mark.angle = glm::degrees(std::atan2(ahead.y - at.y, ahead.x - at.x));
		mark.length = length;
		mark.width = std::max(static_cast<float>(vehicle->MoverParameters->Dim.W) * scale, 3.0f);
		mark.powered = vehicle->MoverParameters->Power > 0.0;
		mark.own = std::ranges::find(own, vehicle) != own.end();
		marks.push_back(mark);
	}
	return marks;
}

void ui::map_panel::follow(std::string const &vehicle)
{
	mode = MODE_VEHICLE;
	m_followed = vehicle;
}

void ui::map_panel::set_view_mode(int const Mode)
{
	mode = static_cast<decltype(mode)>(std::clamp(Mode, 0, 2));
	m_followed.clear();
}

std::vector<ui::map_panel::label> ui::map_panel::labels() const
{
	return collect_labels(m_view.transform, m_surface);
}

void ui::map_panel::zoom_by(float steps, glm::vec2 surface_pos)
{
	if (m_surface.x < 1.0f || m_surface.y < 1.0f)
		return;
	// between about 240 m and about 190 km across a screen 1920 pixels wide
	constexpr float closest = 8.0f; // pixels a metre
	constexpr float farthest = 0.01f;
	auto const scale = pixels_per_metre();
	auto factor = std::pow(2.0f, steps);
	if (scale > 0.0f)
		factor = std::clamp(factor, std::min(1.0f, farthest / scale), std::max(1.0f, closest / scale));
	float prev_zoom = zoom;
	zoom *= factor;
	float x = zoom / prev_zoom;
	// the point under the cursor stays where it is
	glm::vec2 const ndc_pos = surface_pos / m_surface * 2.0f - 1.0f;
	glm::vec4 corrected = glm::inverse(m_view.aspect) * glm::vec4(ndc_pos.x, 0.0f, ndc_pos.y, 1.0f);
	translate = (translate + glm::vec2(corrected.x, corrected.z)) * x - glm::vec2(corrected.x, corrected.z);
}

void ui::map_panel::drag_by(glm::vec2 delta)
{
	if (mode == MODE_MANUAL && m_surface.x >= 1.0f && m_surface.y >= 1.0f)
		translate -= delta / m_surface * 2.0f;
}

void ui::map_panel::click(int button, glm::vec2 surface_pos)
{
	if (m_surface.x < 1.0f || m_surface.y < 1.0f)
		return;
	glm::vec2 const ndc_pos = surface_pos / m_surface * 2.0f - 1.0f;
	glm::vec3 world_pos = world_at(m_view, ndc_pos);
	if (button == 2 && FreeFlyModeFlag)
	{
		Global.FreeCameraInit[0].x = world_pos.x;
		Global.FreeCameraInit[0].y = Global.pCamera.Pos.y;
		Global.FreeCameraInit[0].z = world_pos.z;
	}
	if (button == 1)
		open_objects(world_pos, map::Objects.find_in_range(glm::vec3(world_pos.x, NAN, world_pos.z), 0.03f / zoom));
}

void ui::map_panel::render_popups()
{
	if (popups.empty())
		return;
	// the windows of the objects clicked are opened from a window of no size of their own
	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(ImVec2(1.0f, 1.0f), ImGuiCond_Always);
	auto const flags{ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
	                 ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoDocking};
	if (ImGui::Begin("###mapobjects", nullptr, flags))
		popups.remove_if([](const std::unique_ptr<ui::popup> &popup) { return popup->render(); });
	ImGui::End();
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

			for (auto const &lamp : map_actions::lamps(model))
			{
				glm::vec3 const &current_color = lamp.colour;
				ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(current_color.x, current_color.y, current_color.z, 1.0f));

				std::string res = "  ##" + std::to_string(lamp.index);
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

std::vector<ui::map_actions::lamp> ui::map_actions::lamps(TAnimModel *model)
{
#ifdef WITH_OPENGL_MODERN
	if (auto *renderer = dynamic_cast<opengl46_renderer *>(GfxRenderer.get()))
		renderer->Update_AnimModel(model); // update lamp opacities
#endif
	std::vector<lamp> result;
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
		result.emplace_back(i, current_color, lamp_color, level);
	}
	return result;
}

void ui::map_actions::cycle_lamp(TAnimModel *model, lamp const &Lamp)
{
	float level = Lamp.level + 1.0f;
	if (level >= 3.0f)
		level = 0.0f;

	command_relay relay;
	relay.post(user_command::setlight, (double)Lamp.index, level, GLFW_PRESS, 0, glm::vec3(0.0f), &model->name());
}

std::vector<ui::map_actions::choice> ui::map_actions::aspects(map::semaphore const &semaphore)
{
	std::vector<choice> result;
	for (auto *item : semaphore.events)
	{
		std::string displayname = item->name().substr(semaphore.name.size());

		if (displayname.size() < 2)
			continue;

		displayname[1] = std::toupper(displayname[1]);
		result.emplace_back(std::move(displayname), item);
	}
	return result;
}

std::vector<ui::map_actions::choice> ui::map_actions::positions(map::launcher const &launcher)
{
	const std::string &open_label =
	        launcher.type == map::launcher::track_switch
	        ? STR("Straight |") : STR("Open |");

	const std::string &close_label =
	        launcher.type == map::launcher::track_switch
	        ? STR("Divert /") : STR("Close -");

	std::vector<choice> result;
	result.emplace_back(open_label, launcher.first_event);
	result.emplace_back(close_label, launcher.second_event);
	return result;
}

std::vector<ui::map_actions::switch_choice> ui::map_actions::positions(map::track_switch const &track_switch)
{
	std::array<std::string, 4> names = { "ac", "ad", "bc", "bd" };

	std::vector<switch_choice> result;
	for (size_t i = 0; i < 4; i++) {
		if (!track_switch.action[i])
			continue;
		switch_choice position{choice{names[i], track_switch.action[i]}, {}};
		for (size_t j = 0; j < 4; j++) {
			if ((names[i][0] - 'a') != j && (names[i][1] - 'a') != j)
				continue;
			position.preview.push_back(std::make_pair(track_switch.track[j], track_switch.preview[i][j] == '1' ? 1 : 0));
		}
		result.push_back(std::move(position));
	}
	return result;
}

void ui::map_actions::run(basic_event const &event)
{
	command_relay relay;
	relay.post(user_command::queueevent, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(0.0f), &event.name());
}

std::vector<std::pair<std::string, std::string>> ui::map_actions::obstacle_bank()
{
	std::vector<std::pair<std::string, std::string>> obstacles;
	std::ifstream file;
	file.open("obstaclebank.txt", std::ios_base::in | std::ios_base::binary);

	if (!file.is_open())
		return obstacles;

	std::string line;
	while (std::getline(file, line))
	{
		std::istringstream entry(line);

		std::string name;
		std::string data;
		std::getline(entry, name, ':');
		std::getline(entry, data, ':');

		obstacles.push_back(std::make_pair(name, data));
	}
	return obstacles;
}

void ui::map_actions::insert_obstacle(glm::dvec3 const &position, std::pair<std::string, std::string> const &entry)
{
	std::string name("obstacle_" + std::to_string(LocalRandom(0.0, 100000.0)));

	std::string payload(name + ':' + entry.second);
	command_relay relay;
	relay.post(user_command::insertmodel, 0.0, 0.0, GLFW_PRESS, 0, position, &payload);

	auto obstacle = std::make_shared<map::obstacle>();
	obstacle->name = entry.first;
	obstacle->location = position;
	obstacle->model_name = name;

	std::vector<gfx::basic_vertex> vertices;
	vertices.emplace_back(std::move(obstacle->vertex()));
	gfx::userdata_array userdata{};
	GfxRenderer->Append(vertices, userdata, simulation::Region->get_map_poi_geometry(), GL_POINTS);

	map::Objects.entries.push_back(std::move(obstacle));
}

void ui::map_actions::remove_obstacle(std::shared_ptr<map::obstacle> const &obstacle)
{
	command_relay relay;
	relay.post(user_command::deletemodel, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(), &obstacle->model_name);

	auto &entries = map::Objects.entries;
	for (auto it = entries.rbegin(); it != entries.rend(); it++) {
		if (*it == obstacle) {
			entries.erase(std::next(it).base());
			break;
		}
	}

	map::Objects.poi_dirty = true;
}

void ui::map_actions::set_vehicle_signal(std::string const &vehicle, int const signal)
{
	std::string name = vehicle + "%SetSignal";
	command_relay relay;
	if (signal == 0)
		relay.post(user_command::sendaicommand, 0.0, 0.0, GLFW_PRESS, 0, glm::vec3(), &name);
	else
		relay.post(user_command::sendaicommand, signal, 1.0, GLFW_PRESS, 0, glm::vec3(), &name);
}

ui::semaphore_window::semaphore_window(ui_panel &panel, std::shared_ptr<map::semaphore> &&sem) : popup(panel), m_sem(sem) {}

void ui::semaphore_window::render_content()
{
	for (auto &model : m_sem->models)
	{
		ImGui::PushID(model);
		ImGui::TextUnformatted(model->name().c_str());

		for (auto const &lamp : map_actions::lamps(model))
		{
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(lamp.lit.x, lamp.lit.y, lamp.lit.z, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(lamp.colour.x, lamp.colour.y, lamp.colour.z, 1.0f));

			std::string res = "  ##" + std::to_string(lamp.index);
			if (ImGui::Button(res.c_str()))
				map_actions::cycle_lamp(model, lamp);

			ImGui::PopStyleColor(2);
		}

		ImGui::PopID();
	}

	ImGui::Separator();

	for (auto const &aspect : map_actions::aspects(*m_sem))
	{
		if (ImGui::Button(aspect.name.c_str()))
			map_actions::run(*aspect.event);
	}
}

ui::launcher_window::launcher_window(ui_panel &panel, std::shared_ptr<map::launcher> &&sw) : popup(panel), m_switch(sw) {}

void ui::launcher_window::render_content()
{
	ImGui::TextUnformatted(m_switch->name.c_str());

	for (auto const &position : map_actions::positions(*m_switch))
	{
		if (ImGui::Button(position.name.c_str()))
		{
			map_actions::run(*position.event);
			ImGui::CloseCurrentPopup();
		}
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

    for (auto const &position : map_actions::positions(*m_switch)) {
        if (ImGui::Button(position.position.name.c_str()))
            map_actions::run(*position.position.event);
        if (ImGui::IsItemHovered())
            highlight.insert(highlight.end(), position.preview.begin(), position.preview.end());
        ImGui::SameLine();
    }
}

ui::obstacle_insert_window::obstacle_insert_window(ui_panel &panel, glm::dvec3 const &pos) : popup(panel), m_position(pos), m_obstacles(map_actions::obstacle_bank())
{
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
			map_actions::insert_obstacle(m_position, entry);

			ImGui::CloseCurrentPopup();
		}
	}
}

ui::obstacle_remove_window::obstacle_remove_window(ui_panel &panel, std::shared_ptr<map::obstacle> &&obstacle)
    : popup(panel), m_obstacle(obstacle) { }

void ui::obstacle_remove_window::render_content()
{
	if (ImGui::Button(STR_C("Delete obstacle"))) {
		map_actions::remove_obstacle(m_obstacle);

		ImGui::CloseCurrentPopup();
	}
}

ui::vehicle_click_window::vehicle_click_window(ui_panel &panel, std::shared_ptr<map::vehicle> &&obstacle)
    : popup(panel), m_vehicle(obstacle) { }

void ui::vehicle_click_window::render_content()
{
	ImGui::TextUnformatted(STR_C("Light signal:"));

	if (ImGui::Button(STR_C("none"))) {
		map_actions::set_vehicle_signal(m_vehicle->name, 0);

		ImGui::CloseCurrentPopup();
	}
	if (ImGui::Button(STR_C("Pc6"))) {
		map_actions::set_vehicle_signal(m_vehicle->name, Signal_Pc6);

		ImGui::CloseCurrentPopup();
	}
	if (ImGui::Button(STR_C("A1"))) {
		map_actions::set_vehicle_signal(m_vehicle->name, Signal_A1);

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
