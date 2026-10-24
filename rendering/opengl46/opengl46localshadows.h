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
#include <memory>
#include <string>
#include <vector>

#include "gl/shader.h"
#include "gl/query.h"
#include "rendering/common/locallights.h"
#include "rendering/lightarray.h"

class TDynamicObject;
class TModel3d;

// cube shadow maps of the lamps of the opengl 4.6 renderer (Global.gfx_lights_shadows):
// - the lamp of the occupied cab, with the shadows cast by the cab (the model drawn by Render_cab()). the map is kept in the space
//   of the cab model, centred on the lamp: it stays right while the vehicle moves and turns, and is drawn anew only every few frames
//   (for the moving parts of the cab: levers, doors, wipers) or when the lamp, the cab or the level change;
// - the lamps of scenery models closest to the camera (Global.gfx_lights_shadows_lamps of them, within
//   Global.gfx_lights_shadows_lamps_range). the map of such a lamp is kept in world axes, centred on the lamp, and has two parts:
//   the scenery (drawn once, when the lamp gets its slot or the scene changes) and a working copy of it, with the vehicles near
//   the camera (within Global.gfx_lights_shadows_vehicles_range) and the occupied cab added, made every few frames while there
//   are any by the lamp. the shading reads the working copy then, the scenery part otherwise.
// the six faces of a map are drawn at once, by a geometry shader (shadowcube.vert/.geom), so a model costs one draw call per
// submodel, as in the colour pass. the depth is n / d (d the distance along the axis of the face), see local_light_shadow() in
// light_common.glsl, which reads the map for the light with the layer in its headlight_weights.x, from the array of scenery
// lamps when its linear is 1 and from the one of the cab lamp otherwise.
// - the headlights of the vehicles (Global.gfx_lights_shadows_headlights sets of them, the most important ones): a plain map,
//   drawn from the projector of the beam with its projection (the beam is a narrow frustum aimed along the vehicle, see
//   opengl46_renderer::Headlight_Projector()), with what is within Global.gfx_lights_shadows_headlights_range ahead of the
//   vehicle, but not the vehicle itself. the map keeps the projector it was drawn with, in world space: one a few frames old is
//   still right for what doesn't move. read by headlight_beam_shadow() in light_common.glsl, for the headlights with the map in their
//   headlight_weights.w (1 + the layer) and the projection to it in gl::scene_ubs::headlight_shadow
class opengl46_local_shadows
{
  public:
	opengl46_local_shadows() = default;
	~opengl46_local_shadows();
	opengl46_local_shadows(const opengl46_local_shadows &) = delete;
	opengl46_local_shadows &operator=(const opengl46_local_shadows &) = delete;

	// creates the programs and the stand-in maps. throws gl::shader_exception
	void init();
	// puts the maps (or the stand-ins, which shadow nothing) on their texture units, gl::LOCAL_SHADOW_TEX and
	// gl::LOCAL_SHADOW_LAMPS_TEX. once per context and frame
	void bind();
	// shadow program of the cube maps, of the alpha tested materials when Alpha
	gl::program &program(bool const Alpha)
	{
		return Alpha ? *m_alpha_program : *m_program;
	}
	// the same for the multi draws of the instances drawn from the gpu culling, which read the model block of a draw by its
	// number (MODEL_DRAWS). nullptr when they couldn't be made: the draws are made one by one with program() then
	gl::program *multidraw_program(bool const Alpha)
	{
		return Alpha ? m_alpha_multidraw_program.get() : m_multidraw_program.get();
	}

	// the cab lamp

	// whether the map of the cab lamp is to be drawn in this frame: Vehicle and Cab the occupied vehicle and its cab model,
	// Lamp the position of the lamp in the space of the cab model; Vehicle nullptr when the lamp is off. (re)makes the map to
	// the size of the level when the level changed
	bool update_wanted(TDynamicObject const *Vehicle, TModel3d const *Cab, glm::vec3 const &Lamp, unsigned int const Framestamp);
	// sets up the drawing to the map: the framebuffer, viewport, cleared faces, the state of the faces. the geometry is to be
	// drawn with program() and a modelview from the space of the model to the space of the map
	void begin();
	// ends the drawing to the map, leaves the default framebuffer bound and the culling of faces on
	void end(std::uint32_t const Drawcalls);
	// whether the map holds the shadows of the lamp of Vehicle, the occupied vehicle
	bool ready(TDynamicObject const *Vehicle) const
	{
		return m_valid && (Vehicle != nullptr) && (Vehicle == m_vehicle);
	}
	// position of the lamp the map was drawn for, in the space of the cab model
	glm::vec3 const &lamp() const
	{
		return m_lamp;
	}
	// the headlight_weights of the light which reads the map: layer, near plane, size of a texel one metre from the lamp,
	// radius of the filter in texels
	glm::vec4 weights() const;

	// lamps of scenery models

	// a lamp with a slot
	struct lamp_slot
	{
		void const *owner{nullptr}; // the light it is, see gfx::local_lights::light
		void const *part{nullptr};
		glm::dvec3 position{0.0}; // world space
		float range{0.f}; // of the light, metres
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
	// the geometry is to be drawn with program() and a modelview to world axes centred on the lamp
	void begin_lamp(std::size_t const Slot, bool const Vehicles, std::uint64_t const Version, unsigned int const Framestamp);
	// ends the drawing to the map of a lamp
	void end_lamp(std::uint32_t const Drawcalls);
	// gpu timer of the drawing of the maps of the lamps in this frame
	void lamps_timer_begin();
	void lamps_timer_end();
	// the slot of the lamp the light is, if it has its shadows ready, nullptr otherwise
	lamp_slot const *lamp(gfx::local_lights::light const &Light, std::size_t &Slot) const;
	// the headlight_weights of the light of the lamp in the slot, see weights()
	glm::vec4 lamp_weights(std::size_t const Slot) const;

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
	// ends the drawing to the map of a set of headlights, leaves the default framebuffer bound
	void end_headlight(std::uint32_t const Drawcalls);
	// gpu timer of the drawing of the maps of the headlights in this frame
	void headlights_timer_begin();
	void headlights_timer_end();
	// the slot of the headlights with their map ready in this frame, -1 if there is none
	int headlight_map(TDynamicObject const *Owner, int const Index) const;
	// the headlight_shadow_params of the map of the slot, see gl::scene_ubs
	glm::vec4 headlight_params(std::size_t const Slot) const;

	// lines for the gpu times and statistics of the debug panel
	std::string info_times();
	std::string info() const;
	// controls for the debug window
	void draw_debug_ui();

  private:
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
	void allocate(int const Size);
	void release();
	void allocate_lamps(int const Count, int const Size);
	void release_lamps();
	void allocate_headlights(int const Count, int const Size);
	void release_headlights();
	void set_layer(int const Layer);

	std::unique_ptr<gl::program> m_program;
	std::unique_ptr<gl::program> m_alpha_program;
	std::unique_ptr<gl::program> m_multidraw_program;
	std::unique_ptr<gl::program> m_alpha_multidraw_program;
	GLuint m_empty{0}; // stand-in: a cube map array of 1x1 faces with the farthest depth
	GLuint m_emptyheadlights{0}; // the same, a 2d array of one 1x1 layer
	int m_level{-1}; // level the maps were made for
	// cab lamp
	GLuint m_texture{0}; // cube map array, one cube
	GLuint m_framebuffer{0};
	int m_size{0}; // of a face of m_texture, 0 when there is none
	bool m_valid{false}; // m_texture holds the map of m_vehicle
	TDynamicObject const *m_vehicle{nullptr};
	TModel3d const *m_cab{nullptr};
	glm::vec3 m_lamp{0.f};
	unsigned int m_drawnframe{0}; // framestamp the map was last drawn in
	gl::gpu_timer m_timer;
	// scenery lamps: cube map array, the scenery parts of the slots first, then their working copies
	GLuint m_lampstexture{0};
	GLuint m_lampsframebuffer{0};
	int m_lampssize{0};
	std::vector<lamp_slot> m_slots;
	std::vector<std::size_t> m_wanted;
	unsigned int m_frame{0}; // framestamp of the last update_lamps()
	int m_sceneryleft{0}; // scenery parts which may still be drawn in this frame
	gl::gpu_timer m_lampstimer;
	bool m_lampstiming{false};
	// headlights: 2d array, a layer per slot
	GLuint m_headlightstexture{0};
	GLuint m_headlightsframebuffer{0};
	int m_headlightssize{0};
	std::vector<headlight_slot> m_headlightslots;
	std::vector<std::size_t> m_headlightswanted;
	gl::gpu_timer m_headlightstimer;
	bool m_headlightstiming{false};
	std::chrono::steady_clock::time_point m_headlightscpustart;
	// statistics
	std::uint32_t m_drawcalls{0}; // of the last drawing of the cab map
	std::uint32_t m_updatecount{0}; // drawings of the cab map so far
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
