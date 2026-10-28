/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace scene
{
class basic_region;
class basic_cell;
} // namespace scene
namespace gfx
{
class geometrybank_manager;
}
class TModel3d;
class TSubModel;
class TAnimModel;
class TDynamicObject;

// lights of the scene other than the sun and the vehicle headlights, which go to the light clusters of the renderers:
// - the light points (freespot lights) of scenery models and of vehicles, taken the way the experimental renderer takes them:
//   a point which is visible, has self-illumination (emission) and is past its self-illumination threshold shines as a spot
//   light along the -z axis of its submodel, with the cone of its glare. lamps of scenery models switched with Light_On / Light_Off
//   and the self-illumination of vehicle compartments (TDynamicObject::Sections) work through the same state
// - the lit compartments of vehicles (low poly interior sections) without light points of their own: a point light in the middle
// - the lights of the occupied cab, in place of the ambient light its lamp used to add: the light points of the cab model (mdKabina),
//   any of them, switched by the visibility of the submodels they're in (children of i-cablight:, i-dashboardlight: ...). a cab
//   model without light points gets a point light under its top instead, switched and dimmed with the lamp of the cab
// gathered once per frame around the camera, written to the light buffers by the renderers (Update_Lights())
namespace gfx
{

class local_lights
{
  public:
	struct light
	{
		glm::dvec3 position; // world space
		glm::vec3 direction{0.f, -1.f, 0.f}; // world space, unit; axis of the cone
		glm::vec3 color; // linear, the brightness included
		float cos_outer{-1.f}; // cosine of the half angle of the cone; not above 0: the light is not limited to a cone
		float cos_inner{-1.f}; // cosine of the half angle within which the light is at its full brightness; above cos_outer
		float range{0.f}; // distance at which the light ends, metres
		// light points: the radius of the sphere the light shines from (metres, 0: a point), and its light falling off as
		// 1 / (d^2 + r^2) (the soft falloff of blender) instead of the way the surface of the sphere gives it
		float radius{0.f};
		bool soft_falloff{false};
		// the light casts shadows (the shadow maps of the lamps, the contact shadows), and its shadows are cast also by the model
		// it comes from (the instance, owner)
		bool cast_shadows{true};
		bool shade_self{false};
		// light point of a scenery model: the instance and the submodel it comes from, which name the lamp from frame to frame
		// (opengl46_local_shadows keeps the shadow maps of scenery lamps by them). nullptr for the other lights
		void const *owner{nullptr};
		void const *part{nullptr};
	};

	local_lights() = default;
	local_lights(const local_lights &) = delete;
	local_lights &operator=(const local_lights &) = delete;

	// collects the lights within Global.gfx_lights_range of Camera, for the frame about to be drawn. it sets the state of the
	// shared models of the instances and vehicles it goes through, as the render does before it draws each of them.
	// Geometry: the geometry the models were loaded to, for the bounds of compartments and cabs; Framestamp: the number of the frame
	void gather(glm::dvec3 const &Camera, scene::basic_region &Region, gfx::geometrybank_manager const &Geometry, unsigned int const Framestamp);
	// forgets the lights and everything kept about the scene, which is about to be replaced (a scenery being loaded)
	void reset();
	// the lights of models and vehicles, the most important (closest and brightest) first
	std::vector<light> const &lights() const
	{
		return m_lights;
	}
	// a light of the occupied cab: the light (part names it from frame to frame: the light point, or the vehicle for the lamp put
	// under the top of the cab) and its position in the space of the cab model (mdKabina)
	struct cab_light
	{
		light source;
		glm::vec3 local{0.f};
	};
	// the lights of the occupied cab which are on, when the cab is drawn, the brightest first; empty otherwise
	std::vector<cab_light> const &cab_lights() const
	{
		return m_cablights;
	}
	// the brightest light of the cab, which lights its probe; nullptr when there is none
	cab_light const *cab_principal() const
	{
		return m_cablights.empty() ? nullptr : &m_cablights.front();
	}
	// the vehicle of the lights of the cab, when any of them is on
	TDynamicObject const *cab_lamp_vehicle() const
	{
		return m_cablampon ? m_cablampvehicle : nullptr;
	}
	// the box of the cab model Cab (model space): its middle and half its size; false when the model has no geometry
	bool cab_bounds(TModel3d *Cab, gfx::geometrybank_manager const &Geometry, glm::vec3 &Center, glm::vec3 &Extent);
	// counts of the last pass which wrote the lights to the light buffer: placed, and left out for the lack of room
	void placed(std::uint32_t const Placed, std::uint32_t const Dropped)
	{
		m_placed = Placed;
		m_dropped = Dropped;
	}
	// line for the statistics of the debug panel
	std::string info() const;
	// controls and counters for the debug window
	void draw_debug_ui();

  private:
	struct section_data
	{
		glm::vec3 center{0.f}; // middle of the bounds, model space
		glm::vec3 extent{0.f}; // half the size of the bounds
		bool has_light_points{false}; // the section holds light points, which light it in place of the one made for it
	};
	struct cell_data
	{
		std::vector<TAnimModel *> instances; // instances of models with light points
		// what the list was made from: when the instance list of the cell changes, the list is made anew
		std::size_t source_size{0};
		void const *source_data{nullptr};
	};

	// lights of the light points of the submodel tree, gone through the way the render goes through it. Transform: model space of
	// Submodel's parent, Placement: model space to world space, Brightness: of a light point of white colour
	void walk(TSubModel *Submodel, glm::mat4 const &Transform, glm::dmat4 const &Placement, float const Brightness);
	void add_light_point(TSubModel const &Submodel, glm::mat4 const &Transform, glm::dmat4 const &Placement, float Brightness);
	void gather_instance(TAnimModel *Instance, glm::dvec3 const &Camera, float const Brightness, unsigned int const Framestamp);
	void gather_vehicle(TDynamicObject *Vehicle, glm::dvec3 const &Camera, gfx::geometrybank_manager const &Geometry, float const Freespotbrightness, float const Compartmentbrightness);
	void gather_cab_lights(gfx::geometrybank_manager const &Geometry);
	bool has_light_points(TModel3d *Model);
	cell_data const &cell_instances(scene::basic_cell &Cell, bool const Rebuild);
	section_data const &section(TModel3d *Model, TSubModel *Section, gfx::geometrybank_manager const &Geometry);
	section_data const &model_bounds(TModel3d *Model, gfx::geometrybank_manager const &Geometry);

	std::vector<light> m_lights;
	std::vector<cab_light> m_cablights;
	bool m_cablampon{false};
	TDynamicObject const *m_cablampvehicle{nullptr};
	// walk() goes through the cab model: its light points shine while they're visible, whatever the self-illumination threshold
	bool m_walkcab{false};
	std::unordered_map<TModel3d const *, bool> m_modellights; // whether the model has light points
	std::unordered_map<scene::basic_cell const *, cell_data> m_cells;
	std::unordered_map<TSubModel const *, section_data> m_sections; // compartments, by their submodel
	std::unordered_map<TModel3d const *, section_data> m_models; // bounds of cab models
	TAnimModel const *m_walkowner{nullptr}; // scenery instance whose light points walk() goes through; nullptr for vehicles
	bool m_editorseen{false}; // the editor was on in the last frame: the instance lists of the cells may have changed
	// statistics
	std::uint32_t m_scenerycount{0};
	std::uint32_t m_vehiclecount{0};
	std::uint32_t m_compartmentcount{0};
	std::uint32_t m_placed{0};
	std::uint32_t m_dropped{0};
	double m_gathertime{0.0}; // milliseconds, smoothed
};

} // namespace gfx
