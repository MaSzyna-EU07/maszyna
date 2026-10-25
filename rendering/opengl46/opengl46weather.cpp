/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46weather.h"

#include "rendering/opengl46/opengl46sky.h"
#include "environment/weather.h"
#include "simulation/simulationenvironment.h"
#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

namespace
{

// sizes of the noises (clouds_noise.comp)
int const SHAPE_NOISE_SIZE{128};
int const DETAIL_NOISE_SIZE{32};
int const COVERAGE_NOISE_SIZE{512};
std::unique_ptr<gl::program> make_compute(std::string const &Name)
{
	gl::shader shader(Name);
	return std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
}

std::unique_ptr<gl::program> make_program(std::string const &Vertex, std::string const &Fragment)
{
	gl::shader vertex(Vertex);
	gl::shader fragment(Fragment);
	return std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, fragment});
}

GLuint groups(int const Size)
{
	return static_cast<GLuint>((Size + static_cast<int>(gl::WEATHER_GROUP_SIZE) - 1) / static_cast<int>(gl::WEATHER_GROUP_SIZE));
}

} // namespace

opengl46_weather::~opengl46_weather()
{
	for (auto *texture : {&m_shape_noise, &m_detail_noise, &m_coverage_noise, &m_shadow_map, &m_clouds_color, &m_clouds_depth, &m_clouds_history[0], &m_clouds_history[1], &m_panorama,
	                      &m_fog_scattering[0], &m_fog_scattering[1], &m_fog_integrated})
		release(*texture);
}

void opengl46_weather::release(GLuint &Texture)
{
	if (Texture != 0)
		glDeleteTextures(1, &Texture);
	Texture = 0;
}

GLuint opengl46_weather::create_texture(GLenum const Target, GLenum const Format, glm::ivec3 const &Size, GLint const Filter, GLint const Wrap)
{
	GLuint texture{0};
	glCreateTextures(Target, 1, &texture);
	if (Target == GL_TEXTURE_3D)
		glTextureStorage3D(texture, 1, Format, Size.x, Size.y, Size.z);
	else
		glTextureStorage2D(texture, 1, Format, Size.x, Size.y);
	glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, Filter);
	glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, Filter);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_S, Wrap);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_T, Wrap);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_R, Wrap);
	return texture;
}

void opengl46_weather::init()
{
	m_noise_program = make_compute("clouds_noise.comp");
	m_march_program = make_compute("clouds_march.comp");
	m_temporal_program = make_compute("clouds_temporal.comp");
	m_shadow_program = make_compute("clouds_shadow.comp");
	m_horizon_program = make_compute("fog_horizon.comp");
	m_inject_program = make_compute("fog_inject.comp");
	m_integrate_program = make_compute("fog_integrate.comp");
	m_sky_program = make_program("quad.vert", "sky_weather.frag");
	m_stars_program = make_program("stars.vert", "stars.frag");

	m_weather_ubo = std::make_unique<gl::ubo>(sizeof(gl::weather_ubs), gl::WEATHER_UBO_BINDING);
	m_weather_ubo->update(m_state.block());

	make_noise();

	auto const stars{gfx::weather_state::star_vertices()};
	m_stars_count = static_cast<int>(stars.size() / 2);
	m_stars_vao.emplace();
	m_stars_vao->bind();
	m_stars_buffer.emplace();
	m_stars_buffer->allocate(gl::buffer::ARRAY_BUFFER, static_cast<GLsizeiptr>(stars.size() * sizeof(glm::vec4)), GL_STATIC_DRAW);
	m_stars_buffer->upload(gl::buffer::ARRAY_BUFFER, stars.data(), 0, static_cast<GLsizeiptr>(stars.size() * sizeof(glm::vec4)));
	m_stars_vao->setup_attrib(*m_stars_buffer, 0, 4, GL_FLOAT, 2 * sizeof(glm::vec4), 0);
	m_stars_vao->setup_attrib(*m_stars_buffer, 1, 4, GL_FLOAT, 2 * sizeof(glm::vec4), sizeof(glm::vec4));
	gl::vao::unbind();
}

void opengl46_weather::make_noise()
{
	auto const mipmapped = [](GLenum const Target, GLenum const Format, glm::ivec3 const &Size) {
		GLuint texture{0};
		glCreateTextures(Target, 1, &texture);
		auto const levels{static_cast<GLsizei>(std::floor(std::log2(static_cast<float>(std::max(Size.x, Size.y))))) + 1};
		if (Target == GL_TEXTURE_3D)
			glTextureStorage3D(texture, levels, Format, Size.x, Size.y, Size.z);
		else
			glTextureStorage2D(texture, levels, Format, Size.x, Size.y);
		glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTextureParameteri(texture, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTextureParameteri(texture, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTextureParameteri(texture, GL_TEXTURE_WRAP_R, GL_REPEAT);
		return texture;
	};
	m_shape_noise = mipmapped(GL_TEXTURE_3D, GL_RGBA8, glm::ivec3{SHAPE_NOISE_SIZE});
	m_detail_noise = mipmapped(GL_TEXTURE_3D, GL_RGBA8, glm::ivec3{DETAIL_NOISE_SIZE});
	m_coverage_noise = mipmapped(GL_TEXTURE_2D, GL_RGBA8, glm::ivec3{COVERAGE_NOISE_SIZE, COVERAGE_NOISE_SIZE, 1});

	m_noise_program->bind();
	auto const make = [&](int const Kind, GLuint const Volume, int const Size) {
		glBindImageTexture(0, Volume, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);
		glBindImageTexture(1, m_coverage_noise, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
		glProgramUniform1i(*m_noise_program, 0, Kind);
		glProgramUniform1i(*m_noise_program, 1, Size);
		m_noise_program->dispatch(groups(Size), groups(Size), Kind == 2 ? 1 : static_cast<GLuint>(Size));
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	};
	make(0, m_shape_noise, SHAPE_NOISE_SIZE);
	make(1, m_detail_noise, DETAIL_NOISE_SIZE);
	make(2, m_shape_noise, COVERAGE_NOISE_SIZE);
	glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
	glBindImageTexture(1, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
	gl::program::unbind();
	glGenerateTextureMipmap(m_shape_noise);
	glGenerateTextureMipmap(m_detail_noise);
	glGenerateTextureMipmap(m_coverage_noise);
}

void opengl46_weather::bind()
{
	m_weather_ubo->bind_uniform();
	if (m_shadow_map != 0)
		glBindTextureUnit(gl::CLOUD_SHADOW_TEX, m_shadow_map);
	if (m_fog_integrated != 0)
		glBindTextureUnit(gl::FOG_VOLUME_TEX, m_fog_integrated);
}

void opengl46_weather::bind_noise() const
{
	glBindTextureUnit(gl::CLOUD_SHAPE_TEX, m_shape_noise);
	glBindTextureUnit(gl::CLOUD_DETAIL_TEX, m_detail_noise);
	glBindTextureUnit(gl::CLOUD_COVERAGE_TEX, m_coverage_noise);
	if (m_shadow_map != 0)
		glBindTextureUnit(gl::CLOUD_SHADOW_TEX, m_shadow_map);
}

void opengl46_weather::upload_header()
{
	// the colours of the horizon are left alone when the gpu writes them
	auto const size{m_state.horizon_on_gpu() ? offsetof(gl::weather_ubs, fog_horizon) : sizeof(gl::weather_ubs)};
	m_weather_ubo->update(reinterpret_cast<uint8_t const *>(&m_state.block()), 0, static_cast<GLsizeiptr>(size));
	m_weather_ubo->bind_uniform();
}

void opengl46_weather::update(opengl46_sky const &Sky, glm::vec3 const &Keylight, glm::vec3 const &Keycolor, glm::vec3 const &Ambient, glm::vec3 const &Fogcolor, glm::dvec3 const &Camera)
{
	m_state.update(Sky.atmosphere_weight(), Sky.skyview_table() != 0, Keylight, Keycolor, Ambient, Fogcolor, Camera);
	auto const shadows{m_state.shadows_wanted()};
	if (shadows)
	{
		if (m_shadow_size != Global.gfx_clouds_shadows_size)
		{
			release(m_shadow_map);
			m_shadow_size = Global.gfx_clouds_shadows_size;
			m_shadow_map = create_texture(GL_TEXTURE_2D, GL_R16F, glm::ivec3{m_shadow_size, m_shadow_size, 1}, GL_LINEAR, GL_CLAMP_TO_EDGE);
		}
		m_state.place_shadows(m_shadow_size, Camera);
	}
	upload_header();

	if (m_state.horizon_on_gpu())
	{
		glBindTextureUnit(gl::SKY_VIEW_TEX, Sky.skyview_table());
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, gl::WEATHER_SCRATCH_SSBO_BINDING, *m_weather_ubo);
		glProgramUniform1i(*m_horizon_program, 0, static_cast<GLint>(offsetof(gl::weather_ubs, fog_horizon) / sizeof(glm::vec4)));
		glProgramUniform4f(*m_horizon_program, 1, Sky.atmosphere_weight(), Sky.sky_scale(), 0.5f * Sky.overcast(), 64.f);
		auto const &balance{Sky.white_balance()};
		glProgramUniform3f(*m_horizon_program, 2, balance.r, balance.g, balance.b);
		glProgramUniform3f(*m_horizon_program, 3, Fogcolor.r, Fogcolor.g, Fogcolor.b);
		m_horizon_program->dispatch(1);
		glMemoryBarrier(GL_UNIFORM_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, gl::WEATHER_SCRATCH_SSBO_BINDING, 0);
	}

	if (shadows)
	{
		m_shadow_timer.begin();
		bind_noise();
		glBindImageTexture(0, m_shadow_map, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R16F);
		glProgramUniform1i(*m_shadow_program, 1, m_shadow_size);
		glProgramUniform3f(*m_shadow_program, 2, m_state.block().cloud_shadow_params.x, m_state.block().cloud_shadow_params.y, Global.gfx_clouds_shadows_range);
		glProgramUniform1i(*m_shadow_program, 3, 8);
		m_shadow_program->dispatch(groups(m_shadow_size), groups(m_shadow_size), 1);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R16F);
		m_shadow_timer.end();
		m_state.shadows_drawn(true);
		upload_header();
	}
	else
		m_state.shadows_drawn(false);

	// a band of the panorama of the clouds, all of it the first time
	if (m_state.clouds_drawn())
	{
		auto const level{gfx::weather_state::quality(Global.gfx_clouds, Global.gfx_fog_quality)};
		if (m_panorama_size != level.panorama)
		{
			release(m_panorama);
			m_panorama_size = level.panorama;
			m_panorama = create_texture(GL_TEXTURE_2D, GL_RGBA16F, glm::ivec3{m_panorama_size, 1}, GL_LINEAR, GL_CLAMP_TO_EDGE);
			glTextureParameteri(m_panorama, GL_TEXTURE_WRAP_S, GL_REPEAT);
			m_state.panorama_resized();
		}
		int first, rows;
		m_state.panorama_band(m_panorama_size, first, rows);
		m_panorama_timer.begin();
		bind_noise();
		glBindImageTexture(0, m_panorama, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		// the depth isn't written in this mode, but something has to be bound
		glBindImageTexture(1, m_panorama, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glProgramUniform1i(*m_march_program, 0, 1);
		glProgramUniform2i(*m_march_program, 1, m_panorama_size.x, m_panorama_size.y);
		glProgramUniform1i(*m_march_program, 4, m_state.frame());
		// the panorama is seen blurred: fewer samples
		glProgramUniform4f(*m_march_program, 5, static_cast<float>(std::max(level.steps / 2, 24)), static_cast<float>(std::max(level.light_steps - 1, 2)), Global.gfx_clouds_distance, 0.f);
		glProgramUniform2i(*m_march_program, 6, first, rows);
		m_march_program->dispatch(groups(m_panorama_size.x), groups(rows), 1);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glBindImageTexture(1, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		m_panorama_timer.end();
	}
	gl::program::unbind();
}

void opengl46_weather::prepare_view(glm::ivec2 const &Size, quality_level const &Level)
{
	auto const cloudsize{glm::max((Size + Level.divisor - 1) / Level.divisor, glm::ivec2{1})};
	if (m_state.clouds_drawn() && (cloudsize != m_view_size || m_clouds_color == 0))
	{
		for (auto *texture : {&m_clouds_color, &m_clouds_depth, &m_clouds_history[0], &m_clouds_history[1]})
			release(*texture);
		m_view_size = cloudsize;
		m_clouds_color = create_texture(GL_TEXTURE_2D, GL_RGBA16F, glm::ivec3{m_view_size, 1}, GL_LINEAR, GL_CLAMP_TO_EDGE);
		m_clouds_depth = create_texture(GL_TEXTURE_2D, GL_R32F, glm::ivec3{m_view_size, 1}, GL_NEAREST, GL_CLAMP_TO_EDGE);
		for (auto &history : m_clouds_history)
			history = create_texture(GL_TEXTURE_2D, GL_RGBA16F, glm::ivec3{m_view_size, 1}, GL_LINEAR, GL_CLAMP_TO_EDGE);
		m_state.previous_clouds.valid = false;
	}
	if ((Global.gfx_fog == 1) && (Level.fog_grid != m_fog_grid || m_fog_integrated == 0))
	{
		for (auto *texture : {&m_fog_scattering[0], &m_fog_scattering[1], &m_fog_integrated})
			release(*texture);
		m_fog_grid = Level.fog_grid;
		for (auto &scattering : m_fog_scattering)
			scattering = create_texture(GL_TEXTURE_3D, GL_RGBA16F, m_fog_grid, GL_LINEAR, GL_CLAMP_TO_EDGE);
		// one slice more: the fog through the whole volume without the clouds, which the sky takes (fog_integrate.comp)
		m_fog_integrated = create_texture(GL_TEXTURE_3D, GL_RGBA16F, m_fog_grid + glm::ivec3{0, 0, 1}, GL_LINEAR, GL_CLAMP_TO_EDGE);
		m_state.previous_fog.valid = false;
	}
	m_view_level = Level;
}

void opengl46_weather::render_view(glm::mat4 const &Projection, glm::mat4 const &View, glm::dvec3 const &Camera, glm::ivec2 const &Size, int const Cablamp)
{
	auto const level{gfx::weather_state::quality(Global.gfx_clouds, Global.gfx_fog_quality)};
	prepare_view(Size, level);
	auto const rotation{glm::mat4{glm::mat3{View}}};
	auto const inverseprojection{glm::inverse(Projection)};
	auto const viewtoworld{glm::transpose(rotation)};
	auto const projectionview{Projection * rotation};
	auto &previousclouds{m_state.previous_clouds};
	auto &previousfog{m_state.previous_fog};

	if (m_state.clouds_drawn())
	{
		m_clouds_timer.begin();
		bind_noise();
		glBindImageTexture(0, m_clouds_color, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glBindImageTexture(1, m_clouds_depth, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
		glProgramUniform1i(*m_march_program, 0, 0);
		glProgramUniform2i(*m_march_program, 1, m_view_size.x, m_view_size.y);
		glProgramUniformMatrix4fv(*m_march_program, 2, 1, GL_FALSE, glm::value_ptr(inverseprojection));
		glProgramUniformMatrix4fv(*m_march_program, 3, 1, GL_FALSE, glm::value_ptr(viewtoworld));
		glProgramUniform1i(*m_march_program, 4, m_state.frame());
		glProgramUniform4f(*m_march_program, 5, static_cast<float>(level.steps), static_cast<float>(level.light_steps), Global.gfx_clouds_distance, level.detail ? 1.f : 0.f);
		glProgramUniform2i(*m_march_program, 6, 0, m_view_size.y);
		m_march_program->dispatch(groups(m_view_size.x), groups(m_view_size.y), 1);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

		auto const next{1 - m_clouds_current};
		glBindTextureUnit(gl::CLOUD_TEX, m_clouds_color);
		glBindTextureUnit(gl::CLOUD_SHAPE_TEX, m_clouds_depth);
		glBindTextureUnit(gl::CLOUD_HISTORY_TEX, m_clouds_history[m_clouds_current]);
		glBindImageTexture(0, m_clouds_history[next], 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glBindImageTexture(1, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
		glProgramUniform2i(*m_temporal_program, 1, m_view_size.x, m_view_size.y);
		glProgramUniformMatrix4fv(*m_temporal_program, 2, 1, GL_FALSE, glm::value_ptr(inverseprojection));
		glProgramUniformMatrix4fv(*m_temporal_program, 3, 1, GL_FALSE, glm::value_ptr(viewtoworld));
		glProgramUniformMatrix4fv(*m_temporal_program, 4, 1, GL_FALSE, glm::value_ptr(previousclouds.projection_view));
		auto const previous{glm::vec3{previousclouds.camera - Camera}};
		glProgramUniform3f(*m_temporal_program, 5, previous.x, previous.y, previous.z);
		glProgramUniform1f(*m_temporal_program, 6, gfx::weather_state::continuity(previousclouds, Camera) ? 0.15f : 1.f);
		m_temporal_program->dispatch(groups(m_view_size.x), groups(m_view_size.y), 1);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		// the shape noise back where the clouds of the fog read it
		glBindTextureUnit(gl::CLOUD_SHAPE_TEX, m_shape_noise);
		m_clouds_current = next;
		m_clouds_timer.end();
		previousclouds = {projectionview, Camera, {}, true};
		m_state.clouds_made();
	}
	else
		previousclouds.valid = false;

	if (Global.gfx_fog == 1)
	{
		m_fog_timer.begin();
		auto const temporal{Global.gfx_fog_temporal && gfx::weather_state::continuity(previousfog, Camera)};
		auto const jitter{m_state.fog_jitter(temporal)};
		auto const slicing{gfx::weather_state::fog_slicing(m_fog_grid.z, jitter)};
		bind_noise();
		auto const next{1 - m_fog_current};
		glBindImageTexture(0, m_fog_scattering[next], 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glBindTextureUnit(gl::CLOUD_HISTORY_TEX, m_fog_scattering[m_fog_current]);
		glProgramUniform3i(*m_inject_program, 0, m_fog_grid.x, m_fog_grid.y, m_fog_grid.z);
		glProgramUniformMatrix4fv(*m_inject_program, 1, 1, GL_FALSE, glm::value_ptr(inverseprojection));
		glProgramUniformMatrix4fv(*m_inject_program, 2, 1, GL_FALSE, glm::value_ptr(viewtoworld));
		glProgramUniform4fv(*m_inject_program, 3, 1, glm::value_ptr(slicing));
		glProgramUniformMatrix4fv(*m_inject_program, 4, 1, GL_FALSE, glm::value_ptr(previousfog.projection_view));
		auto const previous{glm::vec3{previousfog.camera - Camera}};
		glProgramUniform3f(*m_inject_program, 5, previous.x, previous.y, previous.z);
		auto const sunshadows{Global.gfx_fog_shadows && Global.gfx_shadowmap_enabled};
		auto const share{temporal ? std::clamp(Global.gfx_fog_temporal_share, 0.02f, 1.f) : 1.f};
		glProgramUniform4f(*m_inject_program, 6, share, Global.gfx_fog_lights, sunshadows ? 1.f : 0.f, Global.gfx_fog_shadows ? 1.f : 0.f);
		glProgramUniform1i(*m_inject_program, 7, Cablamp);
		glProgramUniform2f(*m_inject_program, 8, previousfog.slicing.x, previousfog.slicing.z);
		glProgramUniform2f(*m_inject_program, 9, jitter.x, jitter.y);
		m_inject_program->dispatch(groups(m_fog_grid.x), groups(m_fog_grid.y), static_cast<GLuint>(m_fog_grid.z));
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

		glBindTextureUnit(gl::CLOUD_HISTORY_TEX, m_fog_scattering[next]);
		glBindImageTexture(0, m_fog_integrated, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glProgramUniform3i(*m_integrate_program, 0, m_fog_grid.x, m_fog_grid.y, m_fog_grid.z);
		glProgramUniformMatrix4fv(*m_integrate_program, 1, 1, GL_FALSE, glm::value_ptr(inverseprojection));
		glProgramUniformMatrix4fv(*m_integrate_program, 2, 1, GL_FALSE, glm::value_ptr(viewtoworld));
		glProgramUniform4fv(*m_integrate_program, 3, 1, glm::value_ptr(slicing));
		m_integrate_program->dispatch(groups(m_fog_grid.x), groups(m_fog_grid.y), 1);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		m_fog_current = next;
		m_fog_timer.end();
		previousfog = {projectionview, Camera, slicing, true};
		m_state.fog_made(slicing, m_fog_grid);
	}
	else
		previousfog.valid = false;
	gl::program::unbind();
}

void opengl46_weather::setup_pass(bool const Volume, glm::ivec2 const &Size)
{
	m_state.setup_pass(Volume, Size);
	upload_header();
	bind();
}

void opengl46_weather::draw_stars(glm::mat4 const &View, float const Sunangle, double const Siderealtime, double const Latitude)
{
	if (!m_stars_vao || m_stars_count == 0)
		return;
	glm::mat4 toworld, toview;
	glm::vec4 params;
	if (false == m_state.stars(Sunangle, Siderealtime, Latitude, View, toworld, toview, params))
		return;
	m_stars_program->bind();
	glProgramUniformMatrix4fv(*m_stars_program, 0, 1, GL_FALSE, glm::value_ptr(toworld));
	glProgramUniformMatrix4fv(*m_stars_program, 1, 1, GL_FALSE, glm::value_ptr(toview));
	glProgramUniform4fv(*m_stars_program, 2, 1, glm::value_ptr(params));
	glEnable(GL_PROGRAM_POINT_SIZE);
	glBlendFunc(GL_ONE, GL_ONE);
	m_stars_vao->bind();
	glDrawArrays(GL_POINTS, 0, m_stars_count);
	glDisable(GL_PROGRAM_POINT_SIZE);
	gl::vao::unbind();
}

void opengl46_weather::draw_sky(glm::mat4 const &Projection, glm::mat4 const &View, bool const Mainview)
{
	int mode{0};
	if (m_state.clouds_drawn())
	{
		if (Mainview && m_state.clouds_view_valid())
		{
			mode = 1;
			glBindTextureUnit(gl::CLOUD_TEX, m_clouds_history[m_clouds_current]);
		}
		else if (m_panorama != 0)
		{
			mode = 2;
			glBindTextureUnit(gl::CLOUD_TEX, m_panorama);
		}
	}
	auto const inverseprojection{glm::inverse(Projection)};
	auto const viewtoworld{glm::transpose(glm::mat4{glm::mat3{View}})};
	m_sky_program->bind();
	glProgramUniformMatrix4fv(*m_sky_program, 0, 1, GL_FALSE, glm::value_ptr(inverseprojection));
	glProgramUniformMatrix4fv(*m_sky_program, 1, 1, GL_FALSE, glm::value_ptr(viewtoworld));
	glProgramUniform1i(*m_sky_program, 2, mode);

	GLboolean depthmask;
	glGetBooleanv(GL_DEPTH_WRITEMASK, &depthmask);
	auto const depthtest{glIsEnabled(GL_DEPTH_TEST)};
	auto const cull{glIsEnabled(GL_CULL_FACE)};
	glDepthMask(GL_FALSE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_SRC_ALPHA);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthMask(depthmask);
	if (depthtest)
		glEnable(GL_DEPTH_TEST);
	if (cull)
		glEnable(GL_CULL_FACE);
}

std::string opengl46_weather::info_times()
{
	std::string text;
	auto const line = [&text](char const *Name, gl::gpu_timer &Timer) {
		if (auto const time = Timer.result())
			text += std::string{" "} + Name + ": " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	};
	line("clouds", m_clouds_timer);
	line("clouds panorama", m_panorama_timer);
	line("cloud shadows", m_shadow_timer);
	line("fog volume", m_fog_timer);
	return text;
}

void opengl46_weather::draw_debug_ui()
{
	m_state.draw_debug_ui();
}
