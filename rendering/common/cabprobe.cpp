/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/cabprobe.h"

#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "vehicle/DynObj.h"

namespace gfx
{

namespace
{

// a change of the light of the cab lamp larger than this share of it (the lamp switched, dimmed, the daylight it fades in) has the
// probe drawn anew at once; smaller ones wait for the next update
float const LAMP_CHANGED{0.05f};

float max_component(glm::vec3 const &Color)
{
	return std::max(Color.r, std::max(Color.g, Color.b));
}

} // namespace

int cab_probe::size_of(int const Level)
{
	switch (Level)
	{
	case 1:
		return 64;
	case 2:
		return 128;
	case 3:
		return 256;
	default:
		return 0;
	}
}

void cab_probe::allocate(int const Size)
{
	release();
	if (Size <= 0)
		return;
	if (false == gpu_allocate(Size))
	{
		ErrorLog("cab probe framebuffer setup failed, no cab probe");
		release();
		Global.gfx_cabprobe = 0;
		m_level = 0;
		return;
	}
	m_size = Size;
	WriteLog("cab probe: " + std::to_string(Size) + " x " + std::to_string(Size) + " cube");
}

void cab_probe::release()
{
	gpu_release();
	m_size = 0;
	m_valid = false;
}

bool cab_probe::update_wanted(TDynamicObject const *Vehicle, TModel3d const *Cab, glm::vec3 const &Center, glm::vec3 const &Extent, gfx::local_lights::light const *Lamp, unsigned int const Framestamp)
{
	auto const level{std::clamp(Global.gfx_cabprobe, 0, 3)};
	if (level != m_level)
	{
		m_level = level;
		auto const size{size_of(level)};
		if (size != m_size)
			allocate(size);
		m_valid = false;
	}
	if ((m_size == 0) || (Vehicle == nullptr) || (Cab == nullptr) || (Extent == glm::vec3{0.f}))
	{
		// with no cab drawn nothing reads the probe; once there is one again it may be another
		m_valid = false;
		return false;
	}
	auto const lamp{Lamp != nullptr ? Lamp->color : glm::vec3{0.f}};
	auto const lampchange{glm::length(lamp - m_lamp)};
	auto const lampchanged{(lampchange > 0.f) && ((max_component(m_lamp) <= 0.f) || (max_component(lamp) <= 0.f) || (lampchange > LAMP_CHANGED * glm::length(m_lamp)))};
	if ((Vehicle != m_vehicle) || (Cab != m_cab) || (Center != m_center) || (Extent != m_extent) || lampchanged)
	{
		m_valid = false;
	}
	// the moving parts of the cab and the lights of the desk are followed every few frames
	if (m_valid && (Framestamp - m_drawnframe < static_cast<unsigned int>(std::max(Global.gfx_cabprobe_interval, 1))))
	{
		return false;
	}
	m_vehicle = Vehicle;
	m_cab = Cab;
	m_center = Center;
	m_extent = Extent;
	m_lamp = lamp;
	m_drawnframe = Framestamp;
	return true;
}

void cab_probe::setup_drawing(gl::scene_ubs &Scene, gfx::local_lights::light const *Lamp, glm::vec3 const &Lamplocal, bool const Shadows, glm::vec4 const &Shadowweights) const
{
	if (Lamp == nullptr)
	{
		Scene.cab_probe_lamp = glm::vec4{0.f};
		Scene.cab_probe_lamp_color = glm::vec4{0.f};
		Scene.cab_probe_lamp_shadow = glm::vec4{0.f};
		return;
	}
	// as the light buffer has it (Write_Local_Light()): the shading takes the light to end at 1 / sqrt(quadratic)
	Scene.cab_probe_lamp = glm::vec4{Lamplocal - m_center, 1.f / sq(std::max(Lamp->range, 0.01f))};
	Scene.cab_probe_lamp_color = glm::vec4{Lamp->color, Shadows ? 1.f : 0.f};
	Scene.cab_probe_lamp_shadow = Shadowweights;
}

void cab_probe::begin()
{
	m_cpustart = std::chrono::steady_clock::now();
	gpu_begin();
}

void cab_probe::end(std::uint32_t const Drawcalls)
{
	gpu_end();
	m_valid = true;
	m_drawcalls = Drawcalls;
	++m_updatecount;
	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_cpustart).count()};
	m_cputime = (m_cputime == 0.0 ? elapsed : m_cputime * 0.9 + elapsed * 0.1);
}

glm::dmat4 cab_probe::model_to_probe() const
{
	return glm::translate(glm::dmat4{1.0}, -glm::dvec3{m_center});
}

void cab_probe::setup_scene(gl::scene_ubs &Scene, TDynamicObject const *Vehicle, glm::dvec3 const &Camera, glm::mat3 const &View) const
{
	if (false == ready(Vehicle))
	{
		clear_scene(Scene);
		return;
	}
	// view space to the space of the probe: the space of the cab model, with the origin in the middle of its box. the vehicle is where
	// it is now, the probe stays right as it moves
	auto const toworld{glm::dmat4{glm::transpose(glm::dmat3{View})}};
	auto const toprobe{model_to_probe() * glm::inverse(Vehicle->mMatrix) * glm::translate(glm::dmat4{1.0}, Camera - Vehicle->GetPosition()) * toworld};
	Scene.cab_probe_view = glm::mat4{toprobe};
	Scene.cab_probe_box = glm::vec4{m_extent, 1.f};
	auto const levels{static_cast<float>(std::floor(std::log2(static_cast<float>(std::max(m_size, 1)))))};
	Scene.cab_probe_params = glm::vec4{Global.gfx_cabprobe_bounce, Global.gfx_cabprobe_reflections ? 1.f : 0.f, levels, 0.f};
}

void cab_probe::clear_scene(gl::scene_ubs &Scene)
{
	Scene.cab_probe_view = glm::mat4{1.f};
	Scene.cab_probe_box = glm::vec4{0.f};
	Scene.cab_probe_params = glm::vec4{0.f};
}

int cab_probe::sh_level() const
{
	auto level{0};
	while ((m_size >> level) > static_cast<int>(gl::CAB_PROBE_SH_SIZE))
		++level;
	return level;
}

int cab_probe::sh_size() const
{
	return std::max(m_size >> sh_level(), 1);
}

std::string cab_probe::info() const
{
	if (m_size == 0)
		return "cab probe: off";
	return "cab probe: " + std::to_string(m_size) + " px, " + (m_valid ? "ready" : "none") + ", draw calls " + std::to_string(m_drawcalls) + ", cpu " + to_string(m_cputime, 2) + " ms, updates " +
	       std::to_string(m_updatecount);
}

void cab_probe::draw_debug_ui()
{
	char const *levels[]{"off", "low (64)", "medium (128)", "high (256)"};
	ImGui::Combo("cab probe (gfx.cabprobe)", &Global.gfx_cabprobe, levels, IM_ARRAYSIZE(levels));
	if (Global.gfx_cabprobe <= 0)
		return;
	ImGui::SliderInt("frames between its updates (gfx.cabprobe.interval)", &Global.gfx_cabprobe_interval, 1, 60);
	ImGui::SliderFloat("light bounced around the cab (gfx.cabprobe.bounce)", &Global.gfx_cabprobe_bounce, 0.f, 4.f);
	ImGui::Checkbox("the cab reflects itself (gfx.cabprobe.reflections)", &Global.gfx_cabprobe_reflections);
	ImGui::TextUnformatted(info().c_str());
}

} // namespace gfx
