/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "rendering/common/locallights.h"
#include "rendering/lightarray.h"

class TDynamicObject;
class TModel3d;
class opengl_camera;

namespace gfx
{

// the shadow maps of the lamps (Global.gfx_lights_shadows), the part shared by the renderers (stage P4 of
// claude/vulkan14-renderer-plan.md): which lamps and headlights get maps, when they are drawn, what the shading reads; the
// backend (opengl46_local_shadows, the vulkan renderer) keeps the maps and draws to them through the gpu_* methods.
// - the lights of the occupied cab (gfx::local_lights::cab_lights(): its lamps, the lights of its desk), the brightest
//   Global.gfx_lights_shadows_cab of them, with the shadows cast by the cab (the model drawn by Render_cab()). the map of a light is
//   kept in the space of the cab model, centred on the light: it stays right while the vehicle moves and turns, and is drawn anew only
//   every few frames (for the moving parts of the cab: levers, doors, wipers) or when the light, the cab or the level change. a light
//   keeps its map (slot) while it's among those picked;
// - the lamps of scenery models closest to the camera (Global.gfx_lights_shadows_lamps of them, within
//   Global.gfx_lights_shadows_lamps_range). the map of such a lamp is kept in world axes, centred on the lamp, and has two parts:
//   the scenery (drawn once, when the lamp gets its slot or the scene changes) and a working copy of it, with the vehicles near
//   the camera (within Global.gfx_lights_shadows_vehicles_range) and the occupied cab added, made every few frames while there
//   are any by the lamp. the shading reads the working copy then, the scenery part otherwise.
// the six faces of a map are drawn at once, so a model costs one draw call per submodel, as in the colour pass. the depth is
// n / d (d the distance along the axis of the face), see local_light_shadow() in light_common.glsl, which reads the map for the
// light with the layer in its headlight_weights.x, from the array of scenery lamps when its linear is 1 and from the one of the
// cab lamp otherwise.
// - the headlights of the vehicles (Global.gfx_lights_shadows_headlights sets of them, the most important ones): a plain map,
//   drawn from the projector of the beam with its projection (the beam is a narrow frustum aimed along the vehicle), with what is
//   within Global.gfx_lights_shadows_headlights_range ahead of the vehicle, but not the vehicle itself. the map keeps the projector
//   it was drawn with, in world space: one a few frames old is still right for what doesn't move. read by headlight_beam_shadow()
//   in light_common.glsl, for the headlights with the map in their headlight_weights.w (1 + the layer) and the projection to it in
//   gl::scene_ubs::headlight_shadow
class local_shadows
{
  public:
	// near plane of the maps, metres. surfaces closer to the lamp (its own fitting) cast no shadow
	static constexpr float shadow_near{0.15f};
	// lights of the cab with shadow maps, at most
	static constexpr int cab_slots_max{8};

	local_shadows() = default;
	virtual ~local_shadows() = default;
	local_shadows(const local_shadows &) = delete;
	local_shadows &operator=(const local_shadows &) = delete;

	// the lights of the cab

	// a light of the cab with a slot (a cube of the array of the cab)
	struct cab_slot
	{
		void const *part{nullptr}; // the light it is, see gfx::local_lights::cab_light
		glm::vec3 lamp{0.f}; // position of the light, in the space of the cab model
		float nearplane{shadow_near}; // near plane of the map, see lamp_slot
		bool valid{false}; // the map holds the shadows of the light where it is
		unsigned int drawnframe{0}; // framestamp the map was last drawn in
		unsigned int wantedframe{~0u}; // framestamp it was last among the lights given shadows
	};
	// picks the lights of the cab which get shadows in this frame, and the maps to be drawn in it: Vehicle and Cab the occupied
	// vehicle and its cab model (nullptr when the cab has no lights), Lights its lights, the brightest first. (re)makes the maps when
	// the level, the size or the number wanted changed
	void update_cab(TDynamicObject const *Vehicle, TModel3d const *Cab, std::vector<gfx::local_lights::cab_light> const &Lights, unsigned int const Framestamp);
	// the slots whose maps are to be drawn in this frame
	std::vector<std::size_t> const &cab_draws() const
	{
		return m_cabdraws;
	}
	cab_slot const &cab(std::size_t const Slot) const
	{
		return m_cabslots[Slot];
	}
	// sets up the drawing to the map of the slot: the target, viewport, cleared faces, the state of the faces, its near plane. the
	// geometry is to be drawn with the programs of the cube maps and a modelview from the space of the model to the space of the map
	// (the light of the slot at the origin)
	void begin(std::size_t const Slot);
	// ends the drawing to the map
	void end(std::uint32_t const Drawcalls);
	// the slot of the light of the cab of Vehicle with its shadows ready, -1 if it has none
	int cab_slot_of(TDynamicObject const *Vehicle, gfx::local_lights::cab_light const &Light) const;
	// the headlight_weights of the light which reads the map of the slot: layer, near plane, size of a texel one metre from the lamp,
	// radius of the filter in texels
	glm::vec4 weights(std::size_t const Slot) const;

	// lamps of scenery models

	// a lamp with a slot
	struct lamp_slot
	{
		void const *owner{nullptr}; // the light it is, see gfx::local_lights::light
		void const *part{nullptr};
		glm::dvec3 position{0.0}; // world space
		float range{0.f}; // of the light, metres
		// near plane of the map, metres: what is closer to the lamp casts no shadow. the light of a lamp with a radius comes from
		// the surface of its sphere, what is within it (the glass of the lamp the light point is in) doesn't hold it back
		float nearplane{shadow_near};
		bool shade_self{false}; // the instance of the lamp casts its shadows (local_lights::light::shade_self)
		unsigned int wantedframe{~0u}; // framestamp it was last among the lamps given shadows
		bool scenery{false}; // the scenery part of the map is drawn
		std::uint64_t version{0}; // of the scene the scenery part was drawn for
		bool vehicles{false}; // the working copy is drawn, with vehicles, and is what the shading reads
		unsigned int vehiclesframe{0}; // framestamp the working copy was last drawn in
	};
	// picks the lamps which get shadows in this frame from the gathered lights (the most important first): the scenery lamps
	// within reach of Camera. a lamp keeps its slot while it is picked; one which got a slot or moved, and all of them when the
	// scene changes (Version), need their scenery drawn. (re)makes the maps when their count or size changed
	void update_lamps(std::vector<gfx::local_lights::light> const &Lights, glm::dvec3 const &Camera, std::uint64_t const Version, unsigned int const Framestamp);
	// forgets the lamps: the scene they come from is about to be replaced (a scenery being loaded)
	void reset_lamps();
	// slots of the lamps picked in this frame
	std::vector<std::size_t> const &lamps() const
	{
		return m_wanted;
	}
	lamp_slot const &slot(std::size_t const Slot) const
	{
		return m_slots[Slot];
	}
	// whether the scenery part of the map of the slot is to be drawn in this frame. a few per frame at most, by the level
	bool scenery_wanted(std::size_t const Slot);
	// whether the working copy of the slot, with vehicles, is to be drawn in this frame (every few frames, by the level)
	bool vehicles_wanted(std::size_t const Slot, unsigned int const Framestamp) const;
	// the slot has no vehicles by it: the shading reads the scenery part
	void no_vehicles(std::size_t const Slot)
	{
		m_slots[Slot].vehicles = false;
	}
	// sets up the drawing to the scenery part (cleared) or to the working copy (a copy of the scenery part) of the map of the slot.
	// the geometry is to be drawn with the programs of the cube maps and a modelview to world axes centred on the lamp, with the
	// near plane of the slot
	void begin_lamp(std::size_t const Slot, bool const Vehicles, std::uint64_t const Version, unsigned int const Framestamp);
	// ends the drawing to the map of a lamp
	void end_lamp(std::uint32_t const Drawcalls);
	// the timing of the drawing of the maps of the lamps in this frame
	void lamps_timer_begin();
	void lamps_timer_end();
	// the slot of the lamp the light is, if it has its shadows ready, nullptr otherwise
	lamp_slot const *lamp(gfx::local_lights::light const &Light, std::size_t &Slot) const;
	// the headlight_weights of the light of the lamp in the slot, see weights()
	glm::vec4 lamp_weights(std::size_t const Slot) const;
	// the vehicles near the camera, which cast the shadows of the lamps (Global.gfx_lights_shadows_vehicles_range), and those by
	// the lamp of a slot, within its reach; valid until the next call
	std::vector<TDynamicObject *> const &lamp_vehicles(glm::dvec3 const &Camera);
	std::vector<TDynamicObject *> const &slot_vehicles(lamp_slot const &Slot);
	std::vector<TDynamicObject *> const &slot_vehicles() const
	{
		return m_slotvehicles;
	}

	// headlights of vehicles

	// a set of headlights (one end of a vehicle) with a slot
	struct headlight_slot
	{
		TDynamicObject const *owner{nullptr}; // light_array::light_record::owner and index
		int index{-1};
		unsigned int wantedframe{~0u}; // framestamp it was last among the headlights given shadows
		bool drawn{false}; // the map holds the shadows of the headlights
		unsigned int drawnframe{0};
		// the projector the map was drawn with: world space position, world axes to its axes, projection
		glm::dvec3 projector{0.0};
		glm::mat3 rotation{1.f};
		glm::mat4 projection{1.f};
		glm::vec3 direction{0.f}; // of the light when the map was drawn
	};
	// the headlights which shine, the most important first (in the order the light buffers take them), whose beam may be seen
	// from the camera of the colour pass within the reach of the shadows, as many as there are to be slots; valid until the next call
	std::vector<light_array::light_record const *> const &pick_headlights(light_array const &Lights, opengl_camera const &Camera);
	// picks the headlights which get shadows in this frame: Lights, the most important first, as many as there are slots
	// (Global.gfx_lights_shadows_headlights). a set keeps its slot while it is picked. (re)makes the maps when their count or size changed
	void update_headlights(std::vector<light_array::light_record const *> const &Lights, unsigned int const Framestamp);
	// forgets the headlights: the vehicles may be about to go (a scenery being loaded)
	void reset_headlights();
	// slots of the headlights picked in this frame
	std::vector<std::size_t> const &headlights() const
	{
		return m_headlightswanted;
	}
	headlight_slot const &headlight(std::size_t const Slot) const
	{
		return m_headlightslots[Slot];
	}
	// whether the map of the slot is to be drawn in this frame: one never drawn, then every few frames by the level, more often
	// while the headlights Moved (with the vehicle)
	bool headlight_wanted(std::size_t const Slot, bool const Moved, unsigned int const Framestamp) const;
	// sets up the drawing to the map of the slot (cleared), drawn from the projector given (see headlight_slot). the geometry is
	// to be drawn by the shadow pass with this projection, back faces only
	void begin_headlight(std::size_t const Slot, glm::dvec3 const &Projector, glm::mat3 const &Rotation, glm::mat4 const &Projection, glm::vec3 const &Direction, unsigned int const Framestamp);
	// ends the drawing to the map of a set of headlights
	void end_headlight(std::uint32_t const Drawcalls);
	// the timing of the drawing of the maps of the headlights in this frame
	void headlights_timer_begin();
	void headlights_timer_end();
	// the draws of the maps of lamps and of headlights the walkers of the vulkan renderer made in the frame (M2), counted once they're done
	void add_drawcalls(std::uint32_t const Lamps, std::uint32_t const Headlights)
	{
		m_lampsdrawcalls += Lamps;
		m_headlightsdrawcalls += Headlights;
	}
	// the slot of the headlights with their map ready in this frame, -1 if there is none
	int headlight_map(TDynamicObject const *Owner, int const Index) const;
	// the headlight_shadow_params of the map of the slot, see gl::scene_ubs
	glm::vec4 headlight_params(std::size_t const Slot) const;

	// the occupied vehicle whose cab casts the shadows of the lamps and headlights, nullptr when it's not drawn
	static TDynamicObject *shadow_cab();

	// lines for the statistics of the debug panel
	std::string info() const;
	// controls for the debug window
	void draw_debug_ui();

  protected:
	// size, refresh interval (frames), filter radius (texels, 0: the 2x2 filter of the hardware) of a level, the number of
	// scenery parts of lamp maps drawn per frame, and the size and refresh intervals (frames, while the vehicle moves and while
	// it doesn't) of the maps of headlights
	struct level_setup
	{
		int size;
		unsigned int interval;
		float radius;
		int sceneryperframe;
		int headlightsize;
		unsigned int headlightmoving;
		unsigned int headlightstill;
	};
	static level_setup setup(int const Level);

	// what the backend does with the maps: makes them (false: they can't be made) and lets them go; the lights of the cab: Count
	// cubes, the lamps: 2 x Count cubes (the scenery parts, then the working copies), the headlights: Count layers
	virtual bool gpu_allocate(int const Count, int const Size) = 0;
	virtual void gpu_release() = 0;
	virtual bool gpu_allocate_lamps(int const Count, int const Size) = 0;
	virtual void gpu_release_lamps() = 0;
	virtual bool gpu_allocate_headlights(int const Count, int const Size) = 0;
	virtual void gpu_release_headlights() = 0;
	// the drawing to the cube of a light of the cab (with the near plane of the slot), to a cube of the lamps (Vehicles: the working
	// copy, from a copy of the scenery part Scenerycube; the scenery part cleared otherwise) and to a layer of the headlights
	virtual void gpu_begin_cab(std::size_t const Slot) = 0;
	virtual void gpu_end_cab() = 0;
	virtual void gpu_begin_lamp(int const Scenerycube, int const Workingcube, bool const Vehicles) = 0;
	virtual void gpu_end_lamp() = 0;
	virtual void gpu_begin_headlight(std::size_t const Slot) = 0;
	virtual void gpu_end_headlight() = 0;
	// the gpu times of the drawings of a frame
	virtual void gpu_lamps_timer(bool const Begin) = 0;
	virtual void gpu_headlights_timer(bool const Begin) = 0;

	void allocate(int const Count, int const Size);
	void release();
	void allocate_lamps(int const Count, int const Size);
	void release_lamps();
	void allocate_headlights(int const Count, int const Size);
	void release_headlights();

	int m_level{-1}; // level the maps were made for
	// lights of the cab: a cube per slot
	int m_size{0}; // of a face of the maps, 0 when there are none
	std::vector<cab_slot> m_cabslots;
	std::vector<std::size_t> m_cabwanted; // slots of the lights picked in this frame
	std::vector<std::size_t> m_cabdraws; // and of those whose maps are drawn in it
	std::size_t m_cabdrawing{0}; // slot begin() set up the drawing to
	TDynamicObject const *m_cabvehicle{nullptr}; // the vehicle and the cab model the maps of the slots are of
	TModel3d const *m_cabmodel{nullptr};
	// scenery lamps: the scenery parts of the slots first, then their working copies
	int m_lampssize{0};
	std::vector<lamp_slot> m_slots;
	float m_lampnear{shadow_near}; // near plane of the map of a lamp begin_lamp() set up the drawing to
	std::vector<std::size_t> m_wanted;
	unsigned int m_frame{0}; // framestamp of the last update_lamps()
	int m_sceneryleft{0}; // scenery parts which may still be drawn in this frame
	bool m_lampstiming{false};
	std::vector<TDynamicObject *> m_lampvehicles;
	std::vector<TDynamicObject *> m_slotvehicles;
	// headlights: a layer per slot
	int m_headlightssize{0};
	std::vector<headlight_slot> m_headlightslots;
	std::vector<std::size_t> m_headlightswanted;
	bool m_headlightstiming{false};
	std::chrono::steady_clock::time_point m_headlightscpustart;
	std::vector<std::pair<float, light_array::light_record const *>> m_headlightcandidates;
	std::vector<light_array::light_record const *> m_headlightpicked;
	// statistics
	std::uint32_t m_drawcalls{0}; // of the drawings of the cab maps in the last frame with any
	std::uint32_t m_framedrawcalls{0};
	std::uint32_t m_updatecount{0}; // drawings of the cab maps so far
	double m_cputime{0.0}; // milliseconds, smoothed
	std::chrono::steady_clock::time_point m_cpustart;
	std::uint32_t m_lampsscenery{0}; // scenery parts drawn so far
	std::uint32_t m_lampsvehicles{0}; // working copies drawn so far
	std::uint32_t m_lampsdrawcalls{0}; // in the drawings of the lamp maps in the last frame with any
	std::uint32_t m_lampsframedrawcalls{0};
	double m_lampscputime{0.0}; // milliseconds per frame, smoothed
	std::uint32_t m_headlightsdrawn{0}; // maps of headlights drawn so far
	std::uint32_t m_headlightsdrawcalls{0}; // in the drawings of the maps of headlights in the last frame with any
	std::uint32_t m_headlightsframedrawcalls{0};
	double m_headlightscputime{0.0}; // milliseconds per frame, smoothed
};

} // namespace gfx
