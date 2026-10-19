/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "rendering/opengl46/opengl46locallights.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>

#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "scene/scene.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"
#include "simulation/simulation.h"
#include "rendering/geometrybank.h"

namespace
{
// a light ends where its contribution falls under this. it is above the limit of the culling of the clusters, as there are
// many more of these lights than headlights, and each of them lights only its surroundings
float const LIGHT_THRESHOLD{0.01f};
// no light of a model reaches farther than this, metres
float const LIGHT_RANGE_MAX{60.f};
// the self-illumination threshold of the lamps of scenery models (Light_On, ls_Dark) and full darkness: between them
// the lights of models and compartments fade in, as they would be lost in the daylight anyway
float const LIGHT_DUSK_LUMINANCE{0.325f};
float const LIGHT_DAY_LUMINANCE{1.0f};

glm::vec3 to_linear(glm::vec3 const &Color)
{
	return glm::pow(glm::max(Color, glm::vec3{0.f}), glm::vec3{2.2f});
}

float max_component(glm::vec3 const &Color)
{
	return std::max(Color.r, std::max(Color.g, Color.b));
}

// distance at which a light of the colour falls under the threshold with the square of the distance
float light_range(glm::vec3 const &Color)
{
	return std::min(std::sqrt(max_component(Color) / LIGHT_THRESHOLD), LIGHT_RANGE_MAX);
}

// model space transform of the parent of the submodel, from the static transforms of the submodels above it; false if the
// submodel is not in the tree
bool find_parent_transform(TSubModel const *Submodel, TSubModel const *Target, glm::mat4 const &Parent, glm::mat4 &Result)
{
	for (auto const *submodel{Submodel}; submodel != nullptr; submodel = submodel->Next)
	{
		if (submodel == Target)
		{
			Result = Parent;
			return true;
		}
		auto transform{Parent};
		if ((submodel->iFlags & 0xC000) && (submodel->fMatrix != nullptr))
		{
			transform *= glm::make_mat4(submodel->fMatrix->readArray());
		}
		if (find_parent_transform(submodel->Child, Target, transform, Result))
		{
			return true;
		}
	}
	return false;
}

// bounds of the geometry of the submodel, of its children and, with Siblings, of the submodels after it, in the space of Transform
void extend_bounds(TSubModel const *Submodel, glm::mat4 const &Transform, gfx::geometrybank_manager const &Geometry, bool const Siblings, glm::vec3 &Min, glm::vec3 &Max,
                   bool &Lightpoints)
{
	for (auto const *submodel{Submodel}; submodel != nullptr; submodel = (Siblings ? submodel->Next : nullptr))
	{
		auto transform{Transform};
		if ((submodel->iFlags & 0xC000) && (submodel->fMatrix != nullptr))
		{
			transform *= glm::make_mat4(submodel->fMatrix->readArray());
		}
		if (submodel->eType == TP_FREESPOTLIGHT)
		{
			Lightpoints = true;
		}
		else if ((submodel->eType < TP_ROTATOR) && (submodel->m_geometry.handle != null_handle))
		{
			for (auto const &vertex : Geometry.vertices(submodel->m_geometry.handle))
			{
				auto const point{glm::vec3{transform * glm::vec4{vertex.position, 1.f}}};
				Min = glm::min(Min, point);
				Max = glm::max(Max, point);
			}
		}
		extend_bounds(submodel->Child, transform, Geometry, true, Min, Max, Lightpoints);
	}
}
} // namespace

void opengl46_local_lights::gather(glm::dvec3 const &Camera, scene::basic_region &Region, gfx::geometrybank_manager const &Geometry, unsigned int const Framestamp)
{
	auto const start{std::chrono::steady_clock::now()};

	m_lights.clear();
	m_cablampon = false;
	m_scenerycount = m_vehiclecount = m_compartmentcount = 0;

	// the editor adds, moves and removes instances in the cells. it can also do it in the frame it is turned off in
	auto const rebuild{EditorModeFlag || m_editorseen};
	m_editorseen = EditorModeFlag;

	if (Global.gfx_lights_clustered)
	{
		if (Global.gfx_lights_cab)
		{
			gather_cab_lamp(Geometry);
		}

		// lamps are lost in the daylight; this also spares the work of gathering them in the middle of the day
		auto const darkness{1.f - glm::smoothstep(LIGHT_DUSK_LUMINANCE, LIGHT_DAY_LUMINANCE, static_cast<float>(Global.fLuminance))};
		auto const range{Global.gfx_lights_range};

		if ((darkness > 0.f) && (range > 0.f) && Global.gfx_lights_scenery)
		{
			auto const freespotbrightness{Global.gfx_lights_freespot_intensity * darkness};
			// sections of the region within the range, the same way the render picks them
			auto const camerax{static_cast<int>(std::floor(Camera.x / scene::EU07_SECTIONSIZE + scene::EU07_REGIONSIDESECTIONCOUNT / 2))};
			auto const cameraz{static_cast<int>(std::floor(Camera.z / scene::EU07_SECTIONSIZE + scene::EU07_REGIONSIDESECTIONCOUNT / 2))};
			auto const reach{static_cast<int>(std::ceil(range / scene::EU07_SECTIONSIZE))};
			for (int row = std::max(0, cameraz - reach); row <= std::min(scene::EU07_REGIONSIDESECTIONCOUNT - 1, cameraz + reach); ++row)
			{
				for (int column = std::max(0, camerax - reach); column <= std::min(scene::EU07_REGIONSIDESECTIONCOUNT - 1, camerax + reach); ++column)
				{
					auto *section{Region.m_sections[row * scene::EU07_REGIONSIDESECTIONCOUNT + column]};
					if (section == nullptr)
					{
						continue;
					}
					for (auto &cell : section->m_cells)
					{
						if ((false == cell.m_active) || (glm::length(cell.m_area.center - Camera) > range + cell.m_area.radius))
						{
							continue;
						}
						for (auto *instance : cell_instances(cell, rebuild).instances)
						{
							gather_instance(instance, Camera, freespotbrightness, Framestamp);
						}
					}
				}
			}
		}

		if ((darkness > 0.f) && (range > 0.f) && Global.gfx_lights_vehicles)
		{
			auto const freespotbrightness{Global.gfx_lights_freespot_intensity * darkness};
			auto const compartmentbrightness{Global.gfx_lights_compartment_intensity * darkness};
			for (auto *vehicle : simulation::Vehicles.sequence())
			{
				if (glm::length(vehicle->GetPosition() - Camera) > range + vehicle->radius())
				{
					continue;
				}
				gather_vehicle(vehicle, Camera, Geometry, freespotbrightness, compartmentbrightness);
			}
		}

		// the most important lights first: when the light buffer or the list of a cluster fills up, the others are left out
		std::sort(std::begin(m_lights), std::end(m_lights), [&Camera](light const &Left, light const &Right) {
			return glm::length2(Left.position - Camera) / std::max(max_component(Left.color), 1e-6f) < glm::length2(Right.position - Camera) / std::max(max_component(Right.color), 1e-6f);
		});
	}

	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()};
	m_gathertime = (m_gathertime == 0.0 ? elapsed : m_gathertime * 0.95 + elapsed * 0.05);
}

void opengl46_local_lights::reset()
{
	m_lights.clear();
	m_cablampon = false;
	m_modellights.clear();
	m_cells.clear();
	m_sections.clear();
	m_models.clear();
	m_scenerycount = m_vehiclecount = m_compartmentcount = m_placed = m_dropped = 0;
}

void opengl46_local_lights::gather_instance(TAnimModel *Instance, glm::dvec3 const &Camera, float const Brightness, unsigned int const Framestamp)
{
	if ((false == Instance->m_visible) || (Instance->pModel == nullptr))
	{
		return;
	}
	auto const offset{Instance->location() - Camera};
	if (glm::length(offset) > Global.gfx_lights_range + Instance->radius())
	{
		return;
	}
	// the same distance the render picks the lods and the range of the instance by
	auto const distancesquared{glm::length2(offset / static_cast<double>(Global.ZoomFactor)) / Global.fDistanceFactor};
	if ((distancesquared < Instance->m_rangesquaredmin) || (distancesquared >= Instance->m_rangesquaredmax))
	{
		return;
	}

	Instance->RaAnimate(Framestamp);
	Instance->RaPrepare();
	TSubModel::fSquareDist = static_cast<float>(distancesquared);

	auto const count{m_lights.size()};
	auto const placement{glm::translate(glm::dmat4{1.0}, Instance->location()) * glm::dmat4{Instance->rotation_scale()}};
	m_walkowner = Instance;
	walk(Instance->pModel->Root, glm::mat4{1.f}, placement, Brightness);
	m_walkowner = nullptr;
	m_scenerycount += static_cast<std::uint32_t>(m_lights.size() - count);
}

void opengl46_local_lights::gather_vehicle(TDynamicObject *Vehicle, glm::dvec3 const &Camera, gfx::geometrybank_manager const &Geometry, float const Freespotbrightness,
                                           float const Compartmentbrightness)
{
	if ((false == Global.render_cab) && (Global.pCamera.m_owner == Vehicle))
	{
		// the render leaves the vehicle out as well
		return;
	}
	// light points: the models with any are gone through after the setup the render makes before it draws the vehicle
	std::array<TModel3d *, 2> models{Vehicle->mdModel, Vehicle->mdLowPolyInt};
	auto const haslightpoints = [this](TModel3d *Model) { return (Model != nullptr) && has_light_points(Model); };
	if (std::any_of(std::begin(models), std::end(models), haslightpoints) || std::any_of(std::begin(Vehicle->mdAttachments), std::end(Vehicle->mdAttachments), haslightpoints))
	{
		TSubModel::iInstance = reinterpret_cast<std::uintptr_t>(Vehicle);
		TSubModel::iVariantSeed = Vehicle->variant_seed();
		auto const squaredistance{glm::length2(glm::vec3{Vehicle->vPosition - Camera} / Global.ZoomFactor)};
		Vehicle->ABuLittleUpdate(squaredistance);

		auto const placement{glm::translate(glm::dmat4{1.0}, Vehicle->vPosition) * Vehicle->mMatrix};
		auto const count{m_lights.size()};
		auto const walkmodel = [&](TModel3d *Model) {
			if (haslightpoints(Model))
			{
				TSubModel::fSquareDist = squaredistance;
				walk(Model->Root, glm::mat4{1.f}, placement, Freespotbrightness);
			}
		};
		walkmodel(Vehicle->mdModel);
		for (auto *attachment : Vehicle->mdAttachments)
		{
			walkmodel(attachment);
		}
		walkmodel(Vehicle->mdLowPolyInt);
		m_vehiclecount += static_cast<std::uint32_t>(m_lights.size() - count);

		if (Vehicle->btnOn)
		{
			Vehicle->TurnOff(); // the state of the shared model goes back to the default, as after the render
		}
	}

	// lit compartments, the way TDynamicObject::Update() works out their self-illumination: cab sections come first and are lit at
	// the level of their lamp, the others at the level of the compartment lights
	if ((Vehicle->mdLowPolyInt != nullptr) && (Compartmentbrightness > 0.f))
	{
		auto const owncab{m_cablampon && (simulation::Train != nullptr) && (simulation::Train->Dynamic() == Vehicle)};
		auto cabsection{true};
		for (auto const &section : Vehicle->Sections)
		{
			auto const &name{section.compartment->pName};
			cabsection &= (name.size() >= 4) && name.starts_with("cab");
			auto const level{section.light_level * (cabsection ? 1.f : Vehicle->MoverParameters->CompartmentLights.intensity)};
			if ((level <= 0.f) || (Global.fLuminance > section.compartment->fLight) || (section.compartment->iVisible == 0))
			{
				continue;
			}
			if (cabsection && owncab)
			{
				// the lamp of the occupied cab is a light of its own
				continue;
			}
			auto const &data{this->section(Vehicle->mdLowPolyInt, section.compartment, Geometry)};
			if (data.has_light_points || (data.extent == glm::vec3{0.f}))
			{
				continue;
			}
			light compartmentlight;
			compartmentlight.position = Vehicle->GetWorldPosition(glm::dvec3{data.center});
			compartmentlight.color = to_linear(Vehicle->InteriorLight) * (level * Compartmentbrightness);
			// the light goes out of the windows a little; there are no shadows to keep it in, so it doesn't reach far
			compartmentlight.range = std::clamp(glm::length(data.extent) * 1.5f, 2.f, std::min(light_range(compartmentlight.color), 15.f));
			m_lights.emplace_back(compartmentlight);
			++m_compartmentcount;
		}
	}
}

void opengl46_local_lights::gather_cab_lamp(gfx::geometrybank_manager const &Geometry)
{
	// the cab is drawn, Render_cab() conditions
	if (FreeFlyModeFlag || (false == Global.render_cab) || (simulation::Train == nullptr))
	{
		return;
	}
	auto const *vehicle{simulation::Train->Dynamic()};
	if ((vehicle == nullptr) || (false == vehicle->bDisplayCab) || (vehicle->mdKabina == nullptr) || (vehicle->mdKabina == vehicle->mdModel))
	{
		return;
	}
	if (vehicle->InteriorLightLevel <= 0.f)
	{
		return;
	}
	// as bright as the ambient light it used to add to the cab: lost in the daylight
	auto const luminance{Global.fLuminance * (vehicle->fShade > 0.0f ? vehicle->fShade : 1.0f)};
	auto const level{vehicle->InteriorLightLevel * std::clamp(1.25f - static_cast<float>(luminance), 0.f, 1.f)};
	if (level <= 0.f)
	{
		return;
	}
	auto const &bounds{model_bounds(vehicle->mdKabina, Geometry)};
	if (bounds.extent == glm::vec3{0.f})
	{
		return;
	}
	// under the ceiling, in the middle of the cab
	auto const lamp{glm::vec3{bounds.center.x, std::max(bounds.center.y, bounds.center.y + bounds.extent.y - 0.25f), bounds.center.z}};
	m_cablamp = light{};
	m_cablamp.position = vehicle->GetWorldPosition(glm::dvec3{lamp});
	m_cablampvehicle = vehicle;
	m_cablamplocal = lamp;
	m_cablamp.color = to_linear(vehicle->InteriorLight) * (level * Global.gfx_lights_cab_intensity);
	// within the cab: without shadows (gfx.lights.shadows) the lamp would light the track below and the body of the vehicle around it
	m_cablamp.range = std::clamp(glm::length(bounds.extent) * 1.25f, 2.5f, 8.f);
	m_cablampon = (max_component(m_cablamp.color) > 0.f);
}

void opengl46_local_lights::walk(TSubModel *Submodel, glm::mat4 const &Transform, glm::dmat4 const &Placement, float const Brightness)
{
	// visibility, lod ranges, transforms and animations as in opengl46_renderer::Render_Alpha(TSubModel *)
	for (auto *submodel{Submodel}; submodel != nullptr; submodel = submodel->Next)
	{
		if ((submodel->iVisible == 0) || (TSubModel::fSquareDist < submodel->fSquareMinDist) || (TSubModel::fSquareDist >= submodel->fSquareMaxDist))
		{
			continue;
		}
		auto transform{Transform};
		if (submodel->iFlags & 0xC000)
		{
			if (submodel->fMatrix != nullptr)
			{
				transform *= glm::make_mat4(submodel->fMatrix->readArray());
			}
			if (submodel->b_aAnim != TAnimType::at_None)
			{
				submodel->RaAnimation(transform, submodel->b_aAnim);
			}
		}
		if (submodel->eType == TP_FREESPOTLIGHT)
		{
			add_light_point(*submodel, transform, Placement, Brightness);
		}
		if (submodel->Child != nullptr)
		{
			walk(submodel->Child, transform, Placement, Brightness);
		}
	}
}

void opengl46_local_lights::add_light_point(TSubModel const &Submodel, glm::mat4 const &Transform, glm::dmat4 const &Placement, float const Brightness)
{
	// lit points: with self-illumination, past its threshold. the compartments of vehicles set it with their lights
	if ((Submodel.f4Emision.a <= 0.f) || (Global.fLuminance >= Submodel.fLight) || (Submodel.fVisible <= 0.f))
	{
		return;
	}
	auto const diffuse{Submodel.DiffuseOverride.r < 0.f ? // -1 indicates no override
	                       glm::vec3{Submodel.f4Diffuse} :
	                       Submodel.DiffuseOverride};
	auto const color{to_linear(diffuse) * (Brightness * Submodel.fHotspotPower * Submodel.fVisible * std::min(Submodel.f4Emision.a, 1.f))};
	if (max_component(color) <= LIGHT_THRESHOLD)
	{
		return;
	}

	light lightpoint;
	lightpoint.position = glm::dvec3{Placement * glm::dvec4{glm::dvec3{Transform[3]}, 1.0}};
	auto const axis{glm::vec3{glm::dmat3{Placement} * glm::dvec3{-glm::vec3{Transform[2]}}}};
	lightpoint.direction = (glm::length2(axis) > 0.f ? glm::normalize(axis) : glm::vec3{0.f, -1.f, 0.f});
	lightpoint.color = color;
	// the cone of the glare: where the point is seen from is where it shines. the glare goes from no brightness at the falloff
	// angle to full at the hotspot angle, which some models have the wrong way round
	lightpoint.cos_outer = std::clamp(Submodel.fCosFalloffAngle, -1.f, 0.999f);
	lightpoint.cos_inner = std::clamp(std::max(Submodel.fCosHotspotAngle, lightpoint.cos_outer + 0.05f), lightpoint.cos_outer + 0.001f, 1.f);
	lightpoint.range = light_range(color);
	if (m_walkowner != nullptr)
	{
		lightpoint.owner = m_walkowner;
		lightpoint.part = &Submodel;
	}
	m_lights.emplace_back(lightpoint);
}

bool opengl46_local_lights::has_light_points(TModel3d *Model)
{
	auto lookup{m_modellights.find(Model)};
	if (lookup != m_modellights.end())
	{
		return lookup->second;
	}
	auto found{false};
	std::vector<TSubModel const *> pending{Model->Root};
	while ((false == found) && (false == pending.empty()))
	{
		auto const *submodel{pending.back()};
		pending.pop_back();
		for (; submodel != nullptr; submodel = submodel->Next)
		{
			if (submodel->eType == TP_FREESPOTLIGHT)
			{
				found = true;
				break;
			}
			if (submodel->Child != nullptr)
			{
				pending.emplace_back(submodel->Child);
			}
		}
	}
	m_modellights.emplace(Model, found);
	return found;
}

opengl46_local_lights::cell_data const &opengl46_local_lights::cell_instances(scene::basic_cell &Cell, bool const Rebuild)
{
	// light points are drawn in the translucent phase, so the instances which have them are on the translucent list of the cell
	auto &instances{Cell.m_instancetranslucent};
	auto &data{m_cells[&Cell]};
	if (Rebuild || (data.source_size != instances.size()) || (data.source_data != static_cast<void const *>(instances.data())))
	{
		data.instances.clear();
		for (auto *instance : instances)
		{
			if ((instance->pModel != nullptr) && has_light_points(instance->pModel))
			{
				data.instances.emplace_back(instance);
			}
		}
		data.source_size = instances.size();
		data.source_data = instances.data();
	}
	return data;
}

opengl46_local_lights::section_data const &opengl46_local_lights::section(TModel3d *Model, TSubModel *Section, gfx::geometrybank_manager const &Geometry)
{
	auto lookup{m_sections.find(Section)};
	if (lookup != m_sections.end())
	{
		return lookup->second;
	}
	section_data data;
	glm::mat4 transform{1.f};
	if (find_parent_transform(Model->Root, Section, glm::mat4{1.f}, transform))
	{
		glm::vec3 min{std::numeric_limits<float>::max()};
		glm::vec3 max{std::numeric_limits<float>::lowest()};
		extend_bounds(Section, transform, Geometry, false, min, max, data.has_light_points);
		if (min.x <= max.x)
		{
			data.center = 0.5f * (min + max);
			data.extent = 0.5f * (max - min);
		}
	}
	return m_sections.emplace(Section, data).first->second;
}

opengl46_local_lights::section_data const &opengl46_local_lights::model_bounds(TModel3d *Model, gfx::geometrybank_manager const &Geometry)
{
	auto lookup{m_models.find(Model)};
	if (lookup != m_models.end())
	{
		return lookup->second;
	}
	section_data data;
	glm::vec3 min{std::numeric_limits<float>::max()};
	glm::vec3 max{std::numeric_limits<float>::lowest()};
	extend_bounds(Model->Root, glm::mat4{1.f}, Geometry, true, min, max, data.has_light_points);
	if (min.x <= max.x)
	{
		data.center = 0.5f * (min + max);
		data.extent = 0.5f * (max - min);
	}
	return m_models.emplace(Model, data).first->second;
}

std::string opengl46_local_lights::info() const
{
	return "local lights: " + std::to_string(m_lights.size()) + " (scenery " + std::to_string(m_scenerycount) + ", vehicles " + std::to_string(m_vehiclecount) + ", compartments " +
	       std::to_string(m_compartmentcount) + ")" + (m_cablampon ? " + cab lamp" : "") + ", placed " + std::to_string(m_placed) + ", over capacity " + std::to_string(m_dropped) +
	       ", gather " + to_string(m_gathertime, 2) + " ms";
}

void opengl46_local_lights::draw_debug_ui()
{
	ImGui::TextUnformatted("local lights (need clustered lights)");
	ImGui::Checkbox("light points of scenery models", &Global.gfx_lights_scenery);
	ImGui::Checkbox("light points and compartments of vehicles", &Global.gfx_lights_vehicles);
	ImGui::Checkbox("cab lamp as a light", &Global.gfx_lights_cab);
	ImGui::SliderFloat("light point brightness", &Global.gfx_lights_freespot_intensity, 0.f, 50.f);
	ImGui::SliderFloat("compartment brightness", &Global.gfx_lights_compartment_intensity, 0.f, 10.f);
	ImGui::SliderFloat("cab lamp brightness", &Global.gfx_lights_cab_intensity, 0.f, 10.f);
	ImGui::SliderFloat("cab ambient kept", &Global.gfx_lights_cab_bounce, 0.f, 1.f);
	ImGui::SliderFloat("range, m", &Global.gfx_lights_range, 0.f, 1000.f);
	ImGui::Text("gathered %u: scenery %u, vehicles %u, compartments %u%s", static_cast<unsigned>(m_lights.size()), m_scenerycount, m_vehiclecount, m_compartmentcount,
	            m_cablampon ? ", cab lamp" : "");
	ImGui::Text("in the light buffer %u, over capacity %u", m_placed, m_dropped);
	ImGui::Text("gather %.2f ms (cpu)", m_gathertime);
}
