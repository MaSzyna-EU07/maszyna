/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46localshadows.h"

#include "gl/framebuffer.h"
#include "gl/ubo.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

namespace
{

// near plane of the maps, metres. surfaces closer to the lamp (its own fitting) cast no shadow
float const SHADOW_NEAR{0.15f};
// uniform locations of shadowcube.geom
GLint const NEAR_LOCATION{0};
GLint const LAYER_LOCATION{1};
// layer of the map of the cab lamp in the array
int const CAB_SLOT{0};
// a lamp which moved farther than this (an animated submodel) has its scenery drawn anew, metres
double const LAMP_MOVED{0.01};
// lamps with a shorter reach aren't worth a map, metres
float const LAMP_RANGE_MIN{1.f};

// cube map array of depth maps compared to the reference, one level, Cubes cubes of Size x Size faces
GLuint make_cube_array(int const Size, int const Cubes)
{
	GLuint texture{0};
	glCreateTextures(GL_TEXTURE_CUBE_MAP_ARRAY, 1, &texture);
	glTextureStorage3D(texture, 1, GL_DEPTH_COMPONENT32F, Size, Size, 6 * Cubes);
	// one level: the lookups need no derivatives, also in the loops over the lights the fragments of a quad don't go through together
	glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTextureParameteri(texture, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
	// 1 (shadow) where the map holds n / d at least as large as the reference: a surface at most as far from the lamp
	glTextureParameteri(texture, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	return texture;
}

// 2d array of depth maps compared to the reference, one level, Layers layers of Size x Size
GLuint make_array(int const Size, int const Layers)
{
	GLuint texture{0};
	glCreateTextures(GL_TEXTURE_2D_ARRAY, 1, &texture);
	glTextureStorage3D(texture, 1, GL_DEPTH_COMPONENT32F, Size, Size, Layers);
	// one level, as the cube maps above
	glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTextureParameteri(texture, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
	glTextureParameteri(texture, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	return texture;
}

// framebuffer drawing to all layers of the depth texture (the geometry shader picks the face); 0 if it can't be made
GLuint make_framebuffer(GLuint const Texture)
{
	GLuint framebuffer{0};
	glCreateFramebuffers(1, &framebuffer);
	glNamedFramebufferTexture(framebuffer, GL_DEPTH_ATTACHMENT, Texture, 0);
	glNamedFramebufferDrawBuffer(framebuffer, GL_NONE);
	glNamedFramebufferReadBuffer(framebuffer, GL_NONE);
	if (glCheckNamedFramebufferStatus(framebuffer, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		glDeleteFramebuffers(1, &framebuffer);
		return 0;
	}
	return framebuffer;
}

// the state of the drawing to a cube map
void setup_drawing(GLuint const Framebuffer, int const Size)
{
	// the framebuffer is bound past the cache of gl::framebuffer, which the end of the drawing resets
	gl::framebuffer::unbind();
	glBindFramebuffer(GL_FRAMEBUFFER, Framebuffer);
	glViewport(0, 0, Size, Size);
	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	// one sided walls (of cabs, of buildings) face the lamp, they are to cast shadows as well
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

} // namespace

opengl46_local_shadows::~opengl46_local_shadows()
{
	release();
	release_lamps();
	release_headlights();
	if (m_empty != 0)
		glDeleteTextures(1, &m_empty);
	if (m_emptyheadlights != 0)
		glDeleteTextures(1, &m_emptyheadlights);
}

void opengl46_local_shadows::init()
{
	gl::shader vertex("shadowcube.vert");
	gl::shader geometry("shadowcube.geom");
	{
		gl::shader fragment("shadowmap.frag");
		m_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, geometry, fragment});
	}
	{
		gl::shader fragment("alphashadowmap.frag");
		m_alpha_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, geometry, fragment});
	}
	glProgramUniform1f(*m_program, NEAR_LOCATION, SHADOW_NEAR);
	glProgramUniform1f(*m_alpha_program, NEAR_LOCATION, SHADOW_NEAR);
	set_layer(CAB_SLOT * 6);

	// a shadow sampler with no depth texture to read from is undefined, also where the shading doesn't read it
	glCreateTextures(GL_TEXTURE_CUBE_MAP_ARRAY, 1, &m_empty);
	glTextureStorage3D(m_empty, 1, GL_DEPTH_COMPONENT32F, 1, 1, 6);
	glTextureParameteri(m_empty, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTextureParameteri(m_empty, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTextureParameteri(m_empty, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
	glTextureParameteri(m_empty, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	float const farthest{0.f};
	glClearTexImage(m_empty, 0, GL_DEPTH_COMPONENT, GL_FLOAT, &farthest);
	m_emptyheadlights = make_array(1, 1);
	glTextureParameteri(m_emptyheadlights, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTextureParameteri(m_emptyheadlights, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glClearTexImage(m_emptyheadlights, 0, GL_DEPTH_COMPONENT, GL_FLOAT, &farthest);
}

void opengl46_local_shadows::set_layer(int const Layer)
{
	glProgramUniform1i(*m_program, LAYER_LOCATION, Layer);
	glProgramUniform1i(*m_alpha_program, LAYER_LOCATION, Layer);
}

opengl46_local_shadows::level_setup opengl46_local_shadows::setup(int const Level)
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

void opengl46_local_shadows::allocate(int const Size)
{
	release();
	if (Size <= 0)
		return;
	m_texture = make_cube_array(Size, CAB_SLOT + 1);
	m_framebuffer = make_framebuffer(m_texture);
	if (m_framebuffer == 0)
	{
		ErrorLog("lamp shadow map framebuffer setup failed, the lamps have no shadows");
		release();
		Global.gfx_lights_shadows = 0;
		return;
	}
	m_size = Size;
	WriteLog("lamp shadow maps: " + std::to_string(Size) + " x " + std::to_string(Size) + " cube");
}

void opengl46_local_shadows::release()
{
	if (m_framebuffer != 0)
		glDeleteFramebuffers(1, &m_framebuffer);
	if (m_texture != 0)
		glDeleteTextures(1, &m_texture);
	m_framebuffer = 0;
	m_texture = 0;
	m_size = 0;
	m_valid = false;
}

void opengl46_local_shadows::allocate_lamps(int const Count, int const Size)
{
	release_lamps();
	if ((Count <= 0) || (Size <= 0))
		return;
	// the scenery parts and the working copies
	m_lampstexture = make_cube_array(Size, 2 * Count);
	m_lampsframebuffer = make_framebuffer(m_lampstexture);
	if (m_lampsframebuffer == 0)
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

void opengl46_local_shadows::release_lamps()
{
	if (m_lampsframebuffer != 0)
		glDeleteFramebuffers(1, &m_lampsframebuffer);
	if (m_lampstexture != 0)
		glDeleteTextures(1, &m_lampstexture);
	m_lampsframebuffer = 0;
	m_lampstexture = 0;
	m_lampssize = 0;
	m_slots.clear();
	m_wanted.clear();
}

void opengl46_local_shadows::allocate_headlights(int const Count, int const Size)
{
	release_headlights();
	if ((Count <= 0) || (Size <= 0))
		return;
	m_headlightstexture = make_array(Size, Count);
	glCreateFramebuffers(1, &m_headlightsframebuffer);
	glNamedFramebufferTextureLayer(m_headlightsframebuffer, GL_DEPTH_ATTACHMENT, m_headlightstexture, 0, 0);
	glNamedFramebufferDrawBuffer(m_headlightsframebuffer, GL_NONE);
	glNamedFramebufferReadBuffer(m_headlightsframebuffer, GL_NONE);
	if (glCheckNamedFramebufferStatus(m_headlightsframebuffer, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
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

void opengl46_local_shadows::release_headlights()
{
	if (m_headlightsframebuffer != 0)
		glDeleteFramebuffers(1, &m_headlightsframebuffer);
	if (m_headlightstexture != 0)
		glDeleteTextures(1, &m_headlightstexture);
	m_headlightsframebuffer = 0;
	m_headlightstexture = 0;
	m_headlightssize = 0;
	m_headlightslots.clear();
	m_headlightswanted.clear();
}

void opengl46_local_shadows::bind()
{
	glBindTextureUnit(gl::LOCAL_SHADOW_TEX, (m_texture != 0) ? m_texture : m_empty);
	glBindTextureUnit(gl::LOCAL_SHADOW_LAMPS_TEX, (m_lampstexture != 0) ? m_lampstexture : m_empty);
	glBindTextureUnit(gl::HEADLIGHT_SHADOW_TEX, (m_headlightstexture != 0) ? m_headlightstexture : m_emptyheadlights);
}

bool opengl46_local_shadows::update_wanted(TDynamicObject const *Vehicle, TModel3d const *Cab, glm::vec3 const &Lamp, unsigned int const Framestamp)
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

void opengl46_local_shadows::begin()
{
	m_cpustart = std::chrono::steady_clock::now();
	m_timer.begin();
	set_layer(CAB_SLOT * 6);
	setup_drawing(m_framebuffer, m_size);
	float const farthest{0.f};
	glClearNamedFramebufferfv(m_framebuffer, GL_DEPTH, 0, &farthest);
}

void opengl46_local_shadows::end(std::uint32_t const Drawcalls)
{
	glEnable(GL_CULL_FACE);
	gl::framebuffer::unbind();
	m_timer.end();
	m_valid = true;
	m_drawcalls = Drawcalls;
	++m_updatecount;
	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_cpustart).count()};
	m_cputime = (m_cputime == 0.0 ? elapsed : m_cputime * 0.9 + elapsed * 0.1);
}

glm::vec4 opengl46_local_shadows::weights() const
{
	// a face spans twice the distance from the lamp
	return glm::vec4{static_cast<float>(CAB_SLOT), SHADOW_NEAR, 2.f / static_cast<float>(std::max(m_size, 1)), setup(m_level).radius};
}

void opengl46_local_shadows::update_lamps(std::vector<opengl46_local_lights::light> const &Lights, glm::dvec3 const &Camera, std::uint64_t const Version, unsigned int const Framestamp)
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
	std::vector<opengl46_local_lights::light const *> picked;
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
	std::vector<opengl46_local_lights::light const *> unplaced;
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

void opengl46_local_shadows::reset_lamps()
{
	for (auto &slot : m_slots)
	{
		slot = lamp_slot{};
	}
	m_wanted.clear();
}

bool opengl46_local_shadows::scenery_wanted(std::size_t const Slot)
{
	if (m_slots[Slot].scenery || (m_sceneryleft <= 0))
	{
		return false;
	}
	--m_sceneryleft;
	return true;
}

bool opengl46_local_shadows::vehicles_wanted(std::size_t const Slot, unsigned int const Framestamp) const
{
	auto const &slot{m_slots[Slot]};
	return slot.scenery && ((false == slot.vehicles) || (Framestamp - slot.vehiclesframe >= setup(m_level).interval));
}

void opengl46_local_shadows::begin_lamp(std::size_t const Slot, bool const Vehicles, std::uint64_t const Version, unsigned int const Framestamp)
{
	auto &slot{m_slots[Slot]};
	auto const scenerycube{static_cast<int>(Slot)};
	auto const workingcube{static_cast<int>(m_slots.size() + Slot)};
	if (Vehicles)
	{
		// the vehicles go over the scenery
		glCopyImageSubData(m_lampstexture, GL_TEXTURE_CUBE_MAP_ARRAY, 0, 0, 0, scenerycube * 6, m_lampstexture, GL_TEXTURE_CUBE_MAP_ARRAY, 0, 0, 0, workingcube * 6, m_lampssize, m_lampssize, 6);
		slot.vehicles = true;
		slot.vehiclesframe = Framestamp;
		++m_lampsvehicles;
	}
	else
	{
		float const farthest{0.f};
		glClearTexSubImage(m_lampstexture, 0, 0, 0, scenerycube * 6, m_lampssize, m_lampssize, 6, GL_DEPTH_COMPONENT, GL_FLOAT, &farthest);
		slot.scenery = true;
		slot.version = Version;
		slot.vehicles = false;
		++m_lampsscenery;
	}
	set_layer((Vehicles ? workingcube : scenerycube) * 6);
	setup_drawing(m_lampsframebuffer, m_lampssize);
}

void opengl46_local_shadows::end_lamp(std::uint32_t const Drawcalls)
{
	glEnable(GL_CULL_FACE);
	gl::framebuffer::unbind();
	set_layer(CAB_SLOT * 6);
	m_lampsframedrawcalls += Drawcalls;
}

void opengl46_local_shadows::lamps_timer_begin()
{
	m_lampsframedrawcalls = 0;
	m_cpustart = std::chrono::steady_clock::now();
	m_lampstimer.begin();
	m_lampstiming = true;
}

void opengl46_local_shadows::lamps_timer_end()
{
	if (false == m_lampstiming)
	{
		return;
	}
	m_lampstimer.end();
	m_lampstiming = false;
	m_lampsdrawcalls = m_lampsframedrawcalls;
	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_cpustart).count()};
	m_lampscputime = (m_lampscputime == 0.0 ? elapsed : m_lampscputime * 0.9 + elapsed * 0.1);
}

opengl46_local_shadows::lamp_slot const *opengl46_local_shadows::lamp(opengl46_local_lights::light const &Light, std::size_t &Slot) const
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

glm::vec4 opengl46_local_shadows::lamp_weights(std::size_t const Slot) const
{
	auto const cube{m_slots[Slot].vehicles ? m_slots.size() + Slot : Slot};
	return glm::vec4{static_cast<float>(cube), SHADOW_NEAR, 2.f / static_cast<float>(std::max(m_lampssize, 1)), setup(m_level).radius};
}

void opengl46_local_shadows::update_headlights(std::vector<light_array::light_record const *> const &Lights, unsigned int const Framestamp)
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

void opengl46_local_shadows::reset_headlights()
{
	for (auto &slot : m_headlightslots)
	{
		slot = headlight_slot{};
	}
	m_headlightswanted.clear();
}

bool opengl46_local_shadows::headlight_wanted(std::size_t const Slot, bool const Moved, unsigned int const Framestamp) const
{
	auto const &slot{m_headlightslots[Slot]};
	auto const level{setup(m_level)};
	return (false == slot.drawn) || (Framestamp - slot.drawnframe >= (Moved ? level.headlightmoving : level.headlightstill));
}

void opengl46_local_shadows::begin_headlight(std::size_t const Slot, glm::dvec3 const &Projector, glm::mat3 const &Rotation, glm::mat4 const &Projection, glm::vec3 const &Direction, unsigned int const Framestamp)
{
	auto &slot{m_headlightslots[Slot]};
	slot.drawn = true;
	slot.drawnframe = Framestamp;
	slot.projector = Projector;
	slot.rotation = Rotation;
	slot.projection = Projection;
	slot.direction = Direction;
	++m_headlightsdrawn;

	gl::framebuffer::unbind();
	glNamedFramebufferTextureLayer(m_headlightsframebuffer, GL_DEPTH_ATTACHMENT, m_headlightstexture, 0, static_cast<GLint>(Slot));
	glBindFramebuffer(GL_FRAMEBUFFER, m_headlightsframebuffer);
	glViewport(0, 0, m_headlightssize, m_headlightssize);
	float const farthest{0.f};
	glClearNamedFramebufferfv(m_headlightsframebuffer, GL_DEPTH, 0, &farthest);
	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	// as in the shadows of the sun: the back faces, so the lit faces (the ground most of all, which the beam skims) don't shadow themselves
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void opengl46_local_shadows::end_headlight(std::uint32_t const Drawcalls)
{
	gl::framebuffer::unbind();
	m_headlightsframedrawcalls += Drawcalls;
}

void opengl46_local_shadows::headlights_timer_begin()
{
	m_headlightsframedrawcalls = 0;
	m_headlightscpustart = std::chrono::steady_clock::now();
	m_headlightstimer.begin();
	m_headlightstiming = true;
}

void opengl46_local_shadows::headlights_timer_end()
{
	if (false == m_headlightstiming)
	{
		return;
	}
	m_headlightstimer.end();
	m_headlightstiming = false;
	m_headlightsdrawcalls = m_headlightsframedrawcalls;
	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_headlightscpustart).count()};
	m_headlightscputime = (m_headlightscputime == 0.0 ? elapsed : m_headlightscputime * 0.9 + elapsed * 0.1);
}

int opengl46_local_shadows::headlight_map(TDynamicObject const *Owner, int const Index) const
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

glm::vec4 opengl46_local_shadows::headlight_params(std::size_t const Slot) const
{
	// a map spans 2 tan(fov / 2) one metre from the projector, and the projection holds 1 / tan(fov / 2)
	auto const &projection{m_headlightslots[Slot].projection};
	return glm::vec4{2.f / (std::max(projection[1][1], 1e-6f) * static_cast<float>(std::max(m_headlightssize, 1))), setup(m_level).radius, 0.f, 0.f};
}

std::string opengl46_local_shadows::info_times()
{
	std::string text;
	if (auto const time = m_timer.result())
		text += " lamp shadows: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	if (auto const time = m_lampstimer.result())
		text += " scenery lamp shadows: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	if (auto const time = m_headlightstimer.result())
		text += " headlight shadows: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	return text;
}

std::string opengl46_local_shadows::info() const
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

void opengl46_local_shadows::draw_debug_ui()
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
