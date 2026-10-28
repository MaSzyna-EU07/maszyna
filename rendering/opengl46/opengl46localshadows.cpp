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

// near plane of the maps (gfx::local_shadows)
float const SHADOW_NEAR{gfx::local_shadows::shadow_near};
// uniform locations of shadowcube.geom
GLint const NEAR_LOCATION{0};
GLint const LAYER_LOCATION{1};
// layer of the map of the cab lamp in the array
int const CAB_SLOT{gfx::local_shadows::cab_slot};

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
	gpu_release();
	gpu_release_lamps();
	gpu_release_headlights();
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
	// the variants for the multi draws of the instances drawn from the gpu culling (the scenery parts of the maps of lamps); the
	// fragment stages read nothing of the model block
	try
	{
		gl::shader multidrawvertex("shadowcube.vert", "#define MODEL_DRAWS 1\n");
		gl::shader fragment("shadowmap.frag");
		gl::shader alphafragment("alphashadowmap.frag");
		m_multidraw_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{multidrawvertex, geometry, fragment});
		m_alpha_multidraw_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{multidrawvertex, geometry, alphafragment});
		glProgramUniform1f(*m_multidraw_program, NEAR_LOCATION, SHADOW_NEAR);
		glProgramUniform1f(*m_alpha_multidraw_program, NEAR_LOCATION, SHADOW_NEAR);
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("multi draws of the lamp shadow maps not available: " + std::string(e.what()));
		m_multidraw_program.reset();
		m_alpha_multidraw_program.reset();
	}
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

void opengl46_local_shadows::set_near(float const Near)
{
	glProgramUniform1f(*m_program, NEAR_LOCATION, Near);
	glProgramUniform1f(*m_alpha_program, NEAR_LOCATION, Near);
	if (m_multidraw_program)
		glProgramUniform1f(*m_multidraw_program, NEAR_LOCATION, Near);
	if (m_alpha_multidraw_program)
		glProgramUniform1f(*m_alpha_multidraw_program, NEAR_LOCATION, Near);
}

void opengl46_local_shadows::set_layer(int const Layer)
{
	glProgramUniform1i(*m_program, LAYER_LOCATION, Layer);
	glProgramUniform1i(*m_alpha_program, LAYER_LOCATION, Layer);
	if (m_multidraw_program)
		glProgramUniform1i(*m_multidraw_program, LAYER_LOCATION, Layer);
	if (m_alpha_multidraw_program)
		glProgramUniform1i(*m_alpha_multidraw_program, LAYER_LOCATION, Layer);
}

bool opengl46_local_shadows::gpu_allocate(int const Size)
{
	m_texture = make_cube_array(Size, CAB_SLOT + 1);
	m_framebuffer = make_framebuffer(m_texture);
	return (m_framebuffer != 0);
}

void opengl46_local_shadows::gpu_release()
{
	if (m_framebuffer != 0)
		glDeleteFramebuffers(1, &m_framebuffer);
	if (m_texture != 0)
		glDeleteTextures(1, &m_texture);
	m_framebuffer = 0;
	m_texture = 0;
}

bool opengl46_local_shadows::gpu_allocate_lamps(int const Count, int const Size)
{
	// the scenery parts and the working copies
	m_lampstexture = make_cube_array(Size, 2 * Count);
	m_lampsframebuffer = make_framebuffer(m_lampstexture);
	return (m_lampsframebuffer != 0);
}

void opengl46_local_shadows::gpu_release_lamps()
{
	if (m_lampsframebuffer != 0)
		glDeleteFramebuffers(1, &m_lampsframebuffer);
	if (m_lampstexture != 0)
		glDeleteTextures(1, &m_lampstexture);
	m_lampsframebuffer = 0;
	m_lampstexture = 0;
}

bool opengl46_local_shadows::gpu_allocate_headlights(int const Count, int const Size)
{
	m_headlightstexture = make_array(Size, Count);
	glCreateFramebuffers(1, &m_headlightsframebuffer);
	glNamedFramebufferTextureLayer(m_headlightsframebuffer, GL_DEPTH_ATTACHMENT, m_headlightstexture, 0, 0);
	glNamedFramebufferDrawBuffer(m_headlightsframebuffer, GL_NONE);
	glNamedFramebufferReadBuffer(m_headlightsframebuffer, GL_NONE);
	return (glCheckNamedFramebufferStatus(m_headlightsframebuffer, GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
}

void opengl46_local_shadows::gpu_release_headlights()
{
	if (m_headlightsframebuffer != 0)
		glDeleteFramebuffers(1, &m_headlightsframebuffer);
	if (m_headlightstexture != 0)
		glDeleteTextures(1, &m_headlightstexture);
	m_headlightsframebuffer = 0;
	m_headlightstexture = 0;
}

void opengl46_local_shadows::bind()
{
	glBindTextureUnit(gl::LOCAL_SHADOW_TEX, (m_texture != 0) ? m_texture : m_empty);
	glBindTextureUnit(gl::LOCAL_SHADOW_LAMPS_TEX, (m_lampstexture != 0) ? m_lampstexture : m_empty);
	glBindTextureUnit(gl::HEADLIGHT_SHADOW_TEX, (m_headlightstexture != 0) ? m_headlightstexture : m_emptyheadlights);
}

void opengl46_local_shadows::gpu_begin_cab()
{
	m_timer.begin();
	set_layer(CAB_SLOT * 6);
	set_near(SHADOW_NEAR);
	setup_drawing(m_framebuffer, m_size);
	float const farthest{0.f};
	glClearNamedFramebufferfv(m_framebuffer, GL_DEPTH, 0, &farthest);
}

void opengl46_local_shadows::gpu_end_cab()
{
	glEnable(GL_CULL_FACE);
	gl::framebuffer::unbind();
	m_timer.end();
}

void opengl46_local_shadows::gpu_begin_lamp(int const Scenerycube, int const Workingcube, bool const Vehicles)
{
	if (Vehicles)
	{
		// the vehicles go over the scenery
		glCopyImageSubData(m_lampstexture, GL_TEXTURE_CUBE_MAP_ARRAY, 0, 0, 0, Scenerycube * 6, m_lampstexture, GL_TEXTURE_CUBE_MAP_ARRAY, 0, 0, 0, Workingcube * 6, m_lampssize, m_lampssize, 6);
	}
	else
	{
		float const farthest{0.f};
		glClearTexSubImage(m_lampstexture, 0, 0, 0, Scenerycube * 6, m_lampssize, m_lampssize, 6, GL_DEPTH_COMPONENT, GL_FLOAT, &farthest);
	}
	set_layer((Vehicles ? Workingcube : Scenerycube) * 6);
	// the near plane of the lamp, which the shading reads the map with (lamp_weights())
	set_near(m_lampnear);
	setup_drawing(m_lampsframebuffer, m_lampssize);
}

void opengl46_local_shadows::gpu_end_lamp()
{
	glEnable(GL_CULL_FACE);
	gl::framebuffer::unbind();
	set_layer(CAB_SLOT * 6);
}

void opengl46_local_shadows::gpu_lamps_timer(bool const Begin)
{
	if (Begin)
		m_lampstimer.begin();
	else
		m_lampstimer.end();
}

void opengl46_local_shadows::gpu_begin_headlight(std::size_t const Slot)
{
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

void opengl46_local_shadows::gpu_end_headlight()
{
	gl::framebuffer::unbind();
}

void opengl46_local_shadows::gpu_headlights_timer(bool const Begin)
{
	if (Begin)
		m_headlightstimer.begin();
	else
		m_headlightstimer.end();
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
