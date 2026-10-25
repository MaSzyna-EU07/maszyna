/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/localshadows.h"

#include "rendering/openglcamera.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"

namespace gfx
{

namespace
{

// a lamp which moved farther than this (an animated submodel) has its scenery drawn anew, metres
double const LAMP_MOVED{0.01};
// lamps with a shorter reach aren't worth a map, metres
float const LAMP_RANGE_MIN{1.f};

} // namespace

local_shadows::level_setup local_shadows::setup(int const Level)
{
	switch (Level)
	{
	case 1:
		return {512, 4, 0.f, 1, 1024, 2, 8};
	case 2:
		return {1024, 2, 1.f, 1, 1024, 1, 4};
	case 3:
		return {1024, 1, 2.f, 2, 2048, 1, 2};
	default:
		return {0, 0, 0.f, 0, 0, 0, 0};
	}
}

void local_shadows::allocate(int const Size)
{
	release();
	if (Size <= 0)
		return;
	if (false == gpu_allocate(Size))
	{
		ErrorLog("lamp shadow map framebuffer setup failed, the lamps have no shadows");
		release();
		Global.gfx_lights_shadows = 0;
		return;
	}
	m_size = Size;
	WriteLog("lamp shadow maps: " + std::to_string(Size) + " x " + std::to_string(Size) + " cube");
}

void local_shadows::release()
{
	gpu_release();
	m_size = 0;
	m_valid = false;
}

void local_shadows::allocate_lamps(int const Count, int const Size)
{
	release_lamps();
	if ((Count <= 0) || (Size <= 0))
		return;
	// the scenery parts and the working copies
	if (false == gpu_allocate_lamps(Count, Size))
	{
		ErrorLog("scenery lamp shadow map framebuffer setup failed, the scenery lamps have no shadows");
		release_lamps();
		Global.gfx_lights_shadows_lamps = 0;
		return;
	}
	m_lampssize = Size;
	m_slots.assign(static_cast<std::size_t>(Count), lamp_slot{});
	WriteLog("scenery lamp shadow maps: " + std::to_string(Count) + " x " + std::to_string(Size) + " x " + std::to_string(Size) + " cube (" +
	         std::to_string(2 * Count * 6 * Size / 1024 * Size * 4 / 1024) + " MB)");
}

void local_shadows::release_lamps()
{
	gpu_release_lamps();
	m_lampssize = 0;
	m_slots.clear();
	m_wanted.clear();
}

void local_shadows::allocate_headlights(int const Count, int const Size)
{
	release_headlights();
	if ((Count <= 0) || (Size <= 0))
		return;
	if (false == gpu_allocate_headlights(Count, Size))
	{
		ErrorLog("headlight shadow map framebuffer setup failed, the headlights have no shadows");
		release_headlights();
		Global.gfx_lights_shadows_headlights = 0;
		return;
	}
	m_headlightssize = Size;
	m_headlightslots.assign(static_cast<std::size_t>(Count), headlight_slot{});
	WriteLog("headlight shadow maps: " + std::to_string(Count) + " x " + std::to_string(Size) + " x " + std::to_string(Size) + " (" +
	         std::to_string(Count * Size / 1024 * Size * 4 / 1024) + " MB)");
}

void local_shadows::release_headlights()
{
	gpu_release_headlights();
	m_headlightssize = 0;
	m_headlightslots.clear();
	m_headlightswanted.clear();
}

bool local_shadows::update_wanted(TDynamicObject const *Vehicle, TModel3d const *Cab, glm::vec3 const &Lamp, unsigned int const Framestamp)
{
	auto const level{std::clamp(Global.gfx_lights_shadows, 0, 3)};
	if (level != m_level)
	{
		m_level = level;
		auto const size{setup(level).size};
		if (size != m_size)
			allocate(size);
		m_valid = false;
	}
	if ((m_size == 0) || (Vehicle == nullptr) || (Cab == nullptr))
	{
		// with the lamp off nothing reads the map; once it is on again the cab may have changed
		m_valid = false;
		return false;
	}
	if ((false == m_valid) || (Vehicle != m_vehicle) || (Cab != m_cab) || (Lamp != m_lamp))
	{
		m_valid = false;
	}
	// the moving parts of the cab are followed every few frames
	if (m_valid && (Framestamp - m_drawnframe < setup(level).interval))
	{
		return false;
	}
	m_vehicle = Vehicle;
	m_cab = Cab;
	m_lamp = Lamp;
	m_drawnframe = Framestamp;
	return true;
}

void local_shadows::begin()
{
	m_cpustart = std::chrono::steady_clock::now();
	gpu_begin_cab();
}

void local_shadows::end(std::uint32_t const Drawcalls)
{
	gpu_end_cab();
	m_valid = true;
	m_drawcalls = Drawcalls;
	++m_updatecount;
	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_cpustart).count()};
	m_cputime = (m_cputime == 0.0 ? elapsed : m_cputime * 0.9 + elapsed * 0.1);
}

glm::vec4 local_shadows::weights() const
{
	// a face spans twice the distance from the lamp
	return glm::vec4{static_cast<float>(cab_slot), shadow_near, 2.f / static_cast<float>(std::max(m_size, 1)), setup(m_level).radius};
}

void local_shadows::update_lamps(std::vector<gfx::local_lights::light> const &Lights, glm::dvec3 const &Camera, std::uint64_t const Version, unsigned int const Framestamp)
{
	m_frame = Framestamp;
	m_wanted.clear();
	auto const level{std::clamp(Global.gfx_lights_shadows, 0, 3)};
	auto const count{level > 0 ? Global.gfx_lights_shadows_lamps : 0};
	auto const size{count > 0 ? Global.gfx_lights_shadows_lamps_size : 0};
	if ((static_cast<std::size_t>(count) != m_slots.size()) || (size != m_lampssize))
	{
		allocate_lamps(count, size);
	}
	if (m_slots.empty())
	{
		return;
	}
	m_sceneryleft = setup(level).sceneryperframe;
	auto const reach{static_cast<double>(Global.gfx_lights_shadows_lamps_range)};
	// the lights come the most important first: the first ones within reach get the slots
	std::vector<gfx::local_lights::light const *> picked;
	for (auto const &light : Lights)
	{
		if (picked.size() >= m_slots.size())
		{
			break;
		}
		if ((light.owner == nullptr) || (light.range < LAMP_RANGE_MIN) || (glm::length(light.position - Camera) > reach))
		{
			continue;
		}
		picked.emplace_back(&light);
	}
	// lamps which already have a slot keep it
	std::vector<gfx::local_lights::light const *> unplaced;
	for (auto const *light : picked)
	{
		auto placed{false};
		for (std::size_t idx = 0; idx < m_slots.size(); ++idx)
		{
			auto &slot{m_slots[idx]};
			if ((slot.owner == light->owner) && (slot.part == light->part) && (slot.wantedframe != Framestamp))
			{
				if (glm::length(slot.position - light->position) > LAMP_MOVED)
				{
					slot.scenery = false;
					slot.vehicles = false;
				}
				slot.position = light->position;
				slot.range = light->range;
				slot.wantedframe = Framestamp;
				m_wanted.emplace_back(idx);
				placed = true;
				break;
			}
		}
		if (false == placed)
		{
			unplaced.emplace_back(light);
		}
	}
	// the others take the slots of lamps no longer picked
	for (auto const *light : unplaced)
	{
		for (std::size_t idx = 0; idx < m_slots.size(); ++idx)
		{
			auto &slot{m_slots[idx]};
			if (slot.wantedframe == Framestamp)
			{
				continue;
			}
			slot = lamp_slot{};
			slot.owner = light->owner;
			slot.part = light->part;
			slot.position = light->position;
			slot.range = light->range;
			slot.wantedframe = Framestamp;
			m_wanted.emplace_back(idx);
			break;
		}
	}
	// the scene changed (events, the editor): the scenery parts are out of date
	for (auto &slot : m_slots)
	{
		if (slot.scenery && (slot.version != Version))
		{
			slot.scenery = false;
			slot.vehicles = false;
		}
	}
}

void local_shadows::reset_lamps()
{
	for (auto &slot : m_slots)
	{
		slot = lamp_slot{};
	}
	m_wanted.clear();
}

bool local_shadows::scenery_wanted(std::size_t const Slot)
{
	if (m_slots[Slot].scenery || (m_sceneryleft <= 0))
	{
		return false;
	}
	--m_sceneryleft;
	return true;
}

bool local_shadows::vehicles_wanted(std::size_t const Slot, unsigned int const Framestamp) const
{
	auto const &slot{m_slots[Slot]};
	return slot.scenery && ((false == slot.vehicles) || (Framestamp - slot.vehiclesframe >= setup(m_level).interval));
}

void local_shadows::begin_lamp(std::size_t const Slot, bool const Vehicles, std::uint64_t const Version, unsigned int const Framestamp)
{
	auto &slot{m_slots[Slot]};
	auto const scenerycube{static_cast<int>(Slot)};
	auto const workingcube{static_cast<int>(m_slots.size() + Slot)};
	if (Vehicles)
	{
		// the vehicles go over the scenery
		slot.vehicles = true;
		slot.vehiclesframe = Framestamp;
		++m_lampsvehicles;
	}
	else
	{
		slot.scenery = true;
		slot.version = Version;
		slot.vehicles = false;
		++m_lampsscenery;
	}
	gpu_begin_lamp(scenerycube, workingcube, Vehicles);
}

void local_shadows::end_lamp(std::uint32_t const Drawcalls)
{
	gpu_end_lamp();
	m_lampsframedrawcalls += Drawcalls;
}

void local_shadows::lamps_timer_begin()
{
	m_lampsframedrawcalls = 0;
	m_cpustart = std::chrono::steady_clock::now();
	gpu_lamps_timer(true);
	m_lampstiming = true;
}

void local_shadows::lamps_timer_end()
{
	if (false == m_lampstiming)
	{
		return;
	}
	gpu_lamps_timer(false);
	m_lampstiming = false;
	m_lampsdrawcalls = m_lampsframedrawcalls;
	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_cpustart).count()};
	m_lampscputime = (m_lampscputime == 0.0 ? elapsed : m_lampscputime * 0.9 + elapsed * 0.1);
}

local_shadows::lamp_slot const *local_shadows::lamp(gfx::local_lights::light const &Light, std::size_t &Slot) const
{
	if (Light.owner == nullptr)
	{
		return nullptr;
	}
	for (auto const idx : m_wanted)
	{
		auto const &slot{m_slots[idx]};
		if ((slot.owner == Light.owner) && (slot.part == Light.part))
		{
			if (false == slot.scenery)
			{
				return nullptr;
			}
			Slot = idx;
			return &slot;
		}
	}
	return nullptr;
}

glm::vec4 local_shadows::lamp_weights(std::size_t const Slot) const
{
	auto const cube{m_slots[Slot].vehicles ? m_slots.size() + Slot : Slot};
	return glm::vec4{static_cast<float>(cube), shadow_near, 2.f / static_cast<float>(std::max(m_lampssize, 1)), setup(m_level).radius};
}

std::vector<TDynamicObject *> const &local_shadows::lamp_vehicles(glm::dvec3 const &Camera)
{
	m_lampvehicles.clear();
	auto const vehiclesrange{static_cast<double>(Global.gfx_lights_shadows_vehicles_range)};
	if (vehiclesrange > 0.0)
	{
		for (auto *vehicle : simulation::Vehicles.sequence())
		{
			if (glm::length(vehicle->GetPosition() - Camera) <= vehiclesrange + vehicle->radius())
				m_lampvehicles.emplace_back(vehicle);
		}
	}
	return m_lampvehicles;
}

std::vector<TDynamicObject *> const &local_shadows::slot_vehicles(lamp_slot const &Slot)
{
	m_slotvehicles.clear();
	for (auto *vehicle : m_lampvehicles)
	{
		if (glm::length(vehicle->GetPosition() - Slot.position) <= Slot.range + vehicle->radius())
			m_slotvehicles.emplace_back(vehicle);
	}
	return m_slotvehicles;
}

std::vector<light_array::light_record const *> const &local_shadows::pick_headlights(light_array const &Lights, opengl_camera const &Camera)
{
	m_headlightpicked.clear();
	auto const count{(Global.gfx_lights_headlights && (std::clamp(Global.gfx_lights_shadows, 0, 3) > 0)) ? std::clamp(Global.gfx_lights_shadows_headlights, 0, static_cast<int>(gl::MAX_HEADLIGHT_SHADOWS)) : 0};
	if (count > 0)
	{
		m_headlightcandidates.clear();
		auto const camera{Camera.position()};
		auto const range{static_cast<double>(Global.gfx_lights_shadows_headlights_range)};
		for (auto const &light : Lights.data)
		{
			if (light.intensity == 0.f)
				continue;
			// as in the light buffers: by day the light around takes the beam down to nothing
			auto luminance{static_cast<float>(Global.fLuminance)};
			if (light.owner->fShade > 0.f)
				luminance *= light.owner->fShade;
			auto const beam{light.color - glm::vec3{luminance}};
			if (std::max(beam.r, std::max(beam.g, beam.b)) <= 0.f)
				continue;
			// the part of the beam the shadows are cast in: from the end of the vehicle to the reach ahead of it
			auto const &dimensions{light.owner->MoverParameters->Dim};
			auto const radius{range * 0.5 + std::max(dimensions.W, dimensions.H) + 2.0};
			auto const centre{light.position + glm::dvec3{light.direction} * (range * 0.5 + 2.0)};
			if (glm::length(centre - camera) - radius > range)
				continue;
			if (false == Camera.visible(scene::bounding_area{centre, static_cast<float>(radius)}))
				continue;
			m_headlightcandidates.emplace_back(static_cast<float>(glm::length2(camera - light.position)) / light.intensity, &light);
		}
		auto const picked{std::min(m_headlightcandidates.size(), static_cast<std::size_t>(count))};
		std::partial_sort(std::begin(m_headlightcandidates), std::begin(m_headlightcandidates) + picked, std::end(m_headlightcandidates),
		                  [](auto const &Left, auto const &Right) { return Left.first < Right.first; });
		for (std::size_t idx = 0; idx < picked; ++idx)
		{
			m_headlightpicked.emplace_back(m_headlightcandidates[idx].second);
		}
	}
	return m_headlightpicked;
}

void local_shadows::update_headlights(std::vector<light_array::light_record const *> const &Lights, unsigned int const Framestamp)
{
	m_headlightswanted.clear();
	auto const level{std::clamp(Global.gfx_lights_shadows, 0, 3)};
	auto const count{level > 0 ? std::clamp(Global.gfx_lights_shadows_headlights, 0, static_cast<int>(gl::MAX_HEADLIGHT_SHADOWS)) : 0};
	auto const size{count > 0 ? setup(level).headlightsize : 0};
	if ((static_cast<std::size_t>(count) != m_headlightslots.size()) || (size != m_headlightssize))
	{
		allocate_headlights(count, size);
	}
	if (m_headlightslots.empty())
	{
		return;
	}
	// sets which already have a slot keep it
	std::vector<light_array::light_record const *> unplaced;
	for (auto const *light : Lights)
	{
		if (m_headlightswanted.size() + unplaced.size() >= m_headlightslots.size())
		{
			break;
		}
		auto placed{false};
		for (std::size_t idx = 0; idx < m_headlightslots.size(); ++idx)
		{
			auto &slot{m_headlightslots[idx]};
			if ((slot.owner == light->owner) && (slot.index == light->index) && (slot.wantedframe != Framestamp))
			{
				slot.wantedframe = Framestamp;
				m_headlightswanted.emplace_back(idx);
				placed = true;
				break;
			}
		}
		if (false == placed)
		{
			unplaced.emplace_back(light);
		}
	}
	// the others take the slots of sets no longer picked
	for (auto const *light : unplaced)
	{
		for (std::size_t idx = 0; idx < m_headlightslots.size(); ++idx)
		{
			auto &slot{m_headlightslots[idx]};
			if (slot.wantedframe == Framestamp)
			{
				continue;
			}
			slot = headlight_slot{};
			slot.owner = light->owner;
			slot.index = light->index;
			slot.wantedframe = Framestamp;
			m_headlightswanted.emplace_back(idx);
			break;
		}
	}
}

void local_shadows::reset_headlights()
{
	for (auto &slot : m_headlightslots)
	{
		slot = headlight_slot{};
	}
	m_headlightswanted.clear();
}

bool local_shadows::headlight_wanted(std::size_t const Slot, bool const Moved, unsigned int const Framestamp) const
{
	auto const &slot{m_headlightslots[Slot]};
	auto const level{setup(m_level)};
	return (false == slot.drawn) || (Framestamp - slot.drawnframe >= (Moved ? level.headlightmoving : level.headlightstill));
}

void local_shadows::begin_headlight(std::size_t const Slot, glm::dvec3 const &Projector, glm::mat3 const &Rotation, glm::mat4 const &Projection, glm::vec3 const &Direction, unsigned int const Framestamp)
{
	auto &slot{m_headlightslots[Slot]};
	slot.drawn = true;
	slot.drawnframe = Framestamp;
	slot.projector = Projector;
	slot.rotation = Rotation;
	slot.projection = Projection;
	slot.direction = Direction;
	++m_headlightsdrawn;
	gpu_begin_headlight(Slot);
}

void local_shadows::end_headlight(std::uint32_t const Drawcalls)
{
	gpu_end_headlight();
	m_headlightsframedrawcalls += Drawcalls;
}

void local_shadows::headlights_timer_begin()
{
	m_headlightsframedrawcalls = 0;
	m_headlightscpustart = std::chrono::steady_clock::now();
	gpu_headlights_timer(true);
	m_headlightstiming = true;
}

void local_shadows::headlights_timer_end()
{
	if (false == m_headlightstiming)
	{
		return;
	}
	gpu_headlights_timer(false);
	m_headlightstiming = false;
	m_headlightsdrawcalls = m_headlightsframedrawcalls;
	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_headlightscpustart).count()};
	m_headlightscputime = (m_headlightscputime == 0.0 ? elapsed : m_headlightscputime * 0.9 + elapsed * 0.1);
}

int local_shadows::headlight_map(TDynamicObject const *Owner, int const Index) const
{
	for (auto const idx : m_headlightswanted)
	{
		auto const &slot{m_headlightslots[idx]};
		if ((slot.owner == Owner) && (slot.index == Index))
		{
			return (slot.drawn ? static_cast<int>(idx) : -1);
		}
	}
	return -1;
}

glm::vec4 local_shadows::headlight_params(std::size_t const Slot) const
{
	// a map spans 2 tan(fov / 2) one metre from the projector, and the projection holds 1 / tan(fov / 2)
	auto const &projection{m_headlightslots[Slot].projection};
	return glm::vec4{2.f / (std::max(projection[1][1], 1e-6f) * static_cast<float>(std::max(m_headlightssize, 1))), setup(m_level).radius, 0.f, 0.f};
}

TDynamicObject *local_shadows::shadow_cab()
{
	if (FreeFlyModeFlag || (false == Global.render_cab) || (simulation::Train == nullptr))
		return nullptr;
	auto *cab{simulation::Train->Dynamic()};
	if ((cab != nullptr) && ((false == cab->bDisplayCab) || (cab->mdKabina == nullptr) || (cab->mdKabina == cab->mdModel)))
		cab = nullptr;
	return cab;
}

std::string local_shadows::info() const
{
	std::string text;
	if (m_size != 0)
	{
		text += "\nlamp shadows: " + std::to_string(m_size) + " cube, " + (m_valid ? "cab lamp" : "none") + ", " + std::to_string(m_drawcalls) + " drawcalls, cpu " + to_string(m_cputime, 2) +
		        " ms";
	}
	if (false == m_slots.empty())
	{
		auto withvehicles{0};
		auto ready{0};
		for (auto const idx : m_wanted)
		{
			ready += (m_slots[idx].scenery ? 1 : 0);
			withvehicles += (m_slots[idx].vehicles ? 1 : 0);
		}
		text += "\nscenery lamp shadows: " + std::to_string(m_wanted.size()) + " of " + std::to_string(m_slots.size()) + " slots (" + std::to_string(ready) + " ready, " +
		        std::to_string(withvehicles) + " with vehicles), drawn " + std::to_string(m_lampsscenery) + " scenery + " + std::to_string(m_lampsvehicles) + " vehicles, " +
		        std::to_string(m_lampsdrawcalls) + " drawcalls, cpu " + to_string(m_lampscputime, 2) + " ms";
	}
	if (false == m_headlightslots.empty())
	{
		auto ready{0};
		for (auto const idx : m_headlightswanted)
		{
			ready += (m_headlightslots[idx].drawn ? 1 : 0);
		}
		text += "\nheadlight shadows: " + std::to_string(m_headlightswanted.size()) + " of " + std::to_string(m_headlightslots.size()) + " slots (" + std::to_string(ready) + " ready), " +
		        std::to_string(m_headlightssize) + " map, drawn " + std::to_string(m_headlightsdrawn) + " times, " + std::to_string(m_headlightsdrawcalls) + " drawcalls, cpu " +
		        to_string(m_headlightscputime, 2) + " ms";
	}
	return text;
}

void local_shadows::draw_debug_ui()
{
	char const *levels[]{"no", "low (512, every 4th frame)", "medium (1024, every 2nd frame)", "high (1024, every frame, softer)"};
	ImGui::Combo("lamp shadows (gfx.lights.shadows)", &Global.gfx_lights_shadows, levels, IM_ARRAYSIZE(levels));
	if (m_size != 0)
	{
		ImGui::Text("cab: cube %d x %d, %s, drawn %u times, %u drawcalls, %.2f ms (cpu)", m_size, m_size, m_valid ? "cab lamp" : "no lamp", m_updatecount, m_drawcalls, m_cputime);
	}
	ImGui::SliderInt("scenery lamps with shadows (gfx.lights.shadows.lamps)", &Global.gfx_lights_shadows_lamps, 0, 32);
	ImGui::SliderFloat("their reach from the camera, m (.lamps.range)", &Global.gfx_lights_shadows_lamps_range, 0.f, 500.f);
	ImGui::SliderFloat("vehicles casting them, from the camera, m (.vehicles.range)", &Global.gfx_lights_shadows_vehicles_range, 0.f, 500.f);
	if (false == m_slots.empty())
	{
		auto const text{info()};
		auto const line{text.find("\nscenery lamp shadows:")};
		auto const end{(line == std::string::npos) ? std::string::npos : text.find('\n', line + 1)};
		if (line != std::string::npos)
			ImGui::TextUnformatted(text.substr(line + 1, end == std::string::npos ? std::string::npos : end - line - 1).c_str());
	}
	ImGui::SliderInt("headlights with shadows (gfx.lights.shadows.headlights)", &Global.gfx_lights_shadows_headlights, 0, static_cast<int>(gl::MAX_HEADLIGHT_SHADOWS));
	ImGui::SliderFloat("their reach ahead of the vehicle, m (.headlights.range)", &Global.gfx_lights_shadows_headlights_range, 10.f, 1000.f);
	if (false == m_headlightslots.empty())
	{
		auto const text{info()};
		auto const line{text.rfind('\n')};
		ImGui::TextUnformatted(text.c_str() + (line == std::string::npos ? 0 : line + 1));
	}
}

} // namespace gfx
