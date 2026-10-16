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
#include "environment/starcatalog.h"
#include "environment/weather.h"
#include "simulation/simulationenvironment.h"
#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "utilities/utilities.h"
#include "widgets/weatherpanel.h"

namespace
{

// sizes of the noises (clouds_noise.comp)
int const SHAPE_NOISE_SIZE{128};
int const DETAIL_NOISE_SIZE{32};
int const COVERAGE_NOISE_SIZE{512};
// how many times the cell of a genus fits in the tile of the shape noise, and its spread in the tile of the coverage noise
double const SHAPE_TILE_CELLS{4.0};
double const COVERAGE_TILE_SPREADS{4.0};
// size of the patches of the ground fog
float const FOG_PATCH_SIZE{300.f};
// the haze the atmosphere draws in the sky already, as an extinction: about the haze of a day with 30 km of visibility
float const SKY_HAZE_EXTINCTION{3.f / 30000.f};
// near end of the fog volume (view depth, metres)
float const FOG_VOLUME_NEAR{0.5f};
// bands the panorama is drawn in, one a frame
int const PANORAMA_BANDS{8};
// brightness of a star of magnitude 0 in the units of the scene
float const STAR_BRIGHTNESS{4.f};
// stars dimmer than this aren't drawn
float const STAR_LIMIT{6.5f};

float luminance(glm::vec3 const &Color)
{
	return glm::dot(Color, glm::vec3{0.2126f, 0.7152f, 0.0722f});
}

// the position in a tiling noise, of a period in metres, kept small for the single precision of the shaders
double wrap(double const Value, double const Period)
{
	return Value - Period * std::floor(Value / Period);
}

// the colour of a star of a colour temperature, linear, of luminance 1, somewhat paler than the black body: the eye sees most
// stars white
glm::vec3 star_color(float const Temperature)
{
	auto const t{std::clamp(Temperature, 1500.f, 40000.f) / 100.f};
	glm::vec3 color;
	color.r = (t <= 66.f) ? 255.f : 329.698727446f * std::pow(t - 60.f, -0.1332047592f);
	color.g = (t <= 66.f) ? 99.4708025861f * std::log(t) - 161.1195681661f : 288.1221695283f * std::pow(t - 60.f, -0.0755148492f);
	color.b = (t >= 66.f) ? 255.f : (t <= 19.f) ? 0.f : 138.5177312231f * std::log(t - 10.f) - 305.0447927307f;
	color = glm::pow(glm::clamp(color / 255.f, glm::vec3{0.f}, glm::vec3{1.f}), glm::vec3{2.2f});
	color /= std::max(luminance(color), 1e-3f);
	return glm::mix(color, glm::vec3{1.f}, 0.5f);
}

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

opengl46_weather::quality_level opengl46_weather::quality(int const Clouds, int const Fog)
{
	quality_level level;
	switch (std::clamp(Clouds, 1, 4))
	{
	case 1: level = {4, 32, 3, false, {512, 128}, {}}; break;
	case 2: level = {2, 48, 4, true, {1024, 256}, {}}; break;
	case 3: level = {2, 80, 6, true, {1024, 256}, {}}; break;
	default: level = {2, 128, 8, true, {2048, 512}, {}}; break;
	}
	switch (std::clamp(Fog, 1, 4))
	{
	case 1: level.fog_grid = {80, 45, 48}; break;
	case 2: level.fog_grid = {160, 90, 64}; break;
	case 3: level.fog_grid = {160, 90, 96}; break;
	default: level.fog_grid = {240, 135, 128}; break;
	}
	return level;
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

	// a grey fog and no clouds until the first update
	for (auto &color : m_weather.fog_horizon)
		color = glm::vec4{0.5f, 0.5f, 0.5f, 1.f};
	m_weather.fog_haze = glm::vec4{3.f / 10000.f, 1.f / 800.f, SKY_HAZE_EXTINCTION, 1.f};
	m_weather.fog_light = glm::vec4{0.f, 1.f, 0.6f, 1.f / 5000.f};
	m_weather_ubo = std::make_unique<gl::ubo>(sizeof(gl::weather_ubs), gl::WEATHER_UBO_BINDING);
	m_weather_ubo->update(m_weather);

	make_noise();

	// the stars, in the equatorial frame
	std::vector<glm::vec4> stars;
	auto const *catalogue{star_catalogue::stars()};
	for (std::size_t i = 0; i < star_catalogue::size(); ++i)
	{
		auto const &star{catalogue[i]};
		auto const magnitude{static_cast<float>(star.magnitude) * 0.01f};
		if (magnitude > STAR_LIMIT)
			continue;
		auto const ascension{glm::radians(static_cast<float>(star.right_ascension) * 0.01f)};
		auto const declination{glm::radians(static_cast<float>(star.declination) * 0.01f)};
		stars.emplace_back(std::cos(declination) * std::cos(ascension), std::cos(declination) * std::sin(ascension), std::sin(declination), magnitude);
		// a phase of the twinkling which doesn't repeat from star to star
		auto const phase{std::fmod(static_cast<float>(i) * 0.6180339887f, 1.f)};
		stars.emplace_back(star_color(static_cast<float>(star.temperature) * 10.f), phase);
	}
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
	auto const size{m_horizon_on_gpu ? offsetof(gl::weather_ubs, fog_horizon) : sizeof(gl::weather_ubs)};
	m_weather_ubo->update(reinterpret_cast<uint8_t const *>(&m_weather), 0, static_cast<GLsizeiptr>(size));
	m_weather_ubo->bind_uniform();
}

void opengl46_weather::update(opengl46_sky const &Sky, glm::vec3 const &Keylight, glm::vec3 const &Keycolor, glm::vec3 const &Ambient, glm::vec3 const &Fogcolor, glm::dvec3 const &Camera)
{
	auto const &weather{simulation::Environment.weather()};
	auto const &conditions{weather.conditions()};
	m_fog_volume_valid = false;
	m_clouds_view_valid = false;
	++m_frame;

	auto const height{static_cast<float>(Camera.y) - weather.ground()};
	auto const time{weather.time()};
	auto const sun{glm::normalize(Keylight)};

	// the fog. the visibility is where a dark object fades to 5%: the extinction is 3 over it
	auto const haze{3.f / conditions.visibility};
	auto const hazerate{1.f / conditions.haze_height};
	m_weather.fog_haze = glm::vec4{haze, hazerate, SKY_HAZE_EXTINCTION, 1.f};
	if (conditions.groundfog_thickness > 0.f)
	{
		auto const thickness{conditions.groundfog_thickness};
		m_weather.fog_ground = glm::vec4{std::max(3.f / conditions.groundfog_visibility - haze, 0.f), 1.f / (0.25f * thickness), 0.75f * thickness, conditions.fog_noise};
	}
	else
		m_weather.fog_ground = glm::vec4{0.f, 1.f, 0.f, conditions.fog_noise};
	// the patches drift with the wind, slower than the clouds
	auto const drift{glm::dvec2{weather.cloud_offset()} * 0.3};
	auto const patch{static_cast<double>(FOG_PATCH_SIZE)};
	m_weather.fog_noise = glm::vec4{static_cast<float>(wrap(Camera.x - drift.x, patch)), static_cast<float>(wrap(Camera.y, patch)), static_cast<float>(wrap(Camera.z - drift.y, patch)), 1.f / FOG_PATCH_SIZE};
	// the extinction at the height of the camera tells how far things are hidden
	auto const extinction{haze * std::exp(-hazerate * std::max(height, 0.f)) +
	                      m_weather.fog_ground.x * std::exp(-m_weather.fog_ground.y * std::max(height - m_weather.fog_ground.z, 0.f))};
	// 2% left at 1.3 visibilities; the draw range is twice this
	m_fog_range = std::clamp(3.f / std::max(extinction, 1e-7f) / 1.5f, 5.f, 1e6f);

	// the share of the light of the fog the sun gives, which the shadows hold back
	auto const sunlight{luminance(Keycolor) * 1.5f * std::max(sun.y, 0.f)};
	auto const skylight{luminance(Ambient) * 0.65f};
	m_sun_share = (sunlight + skylight > 1e-6f) ? sunlight / (sunlight + skylight) : 0.f;

	// clouds
	auto const layers{std::min(conditions.clouds.size(), gl::MAX_CLOUD_LAYERS)};
	m_clouds_drawn = (Global.gfx_clouds > 0) && (layers > 0);
	m_average_transmittance = 1.f;
	auto const velocity{weather.cloud_velocity()};
	auto const speed{glm::length(velocity)};
	auto const wind{speed > 1e-3f ? glm::dvec2{velocity / speed} : glm::dvec2{1.0, 0.0}};
	m_weather.cloud_wind = glm::vec4{static_cast<float>(wind.x), static_cast<float>(wind.y), speed, 0.f};
	for (std::size_t i = 0; i < gl::MAX_CLOUD_LAYERS; ++i)
	{
		auto &target{m_weather.cloud_layers[i]};
		if (i >= layers)
		{
			target = gl::cloud_layer_ubs{};
			continue;
		}
		auto const &layer{conditions.clouds[i]};
		auto const &genus{cloud_genus_info(layer.type)};
		target.heights = glm::vec4{layer.base, layer.top, 1.f / std::max(layer.top - layer.base, 1.f), static_cast<float>(layer.type)};
		auto const shapetile{static_cast<double>(genus.cell) * SHAPE_TILE_CELLS};
		auto const coveragetile{static_cast<double>(genus.spread) * COVERAGE_TILE_SPREADS};
		target.shape = glm::vec4{layer.coverage, genus.extinction * layer.density, static_cast<float>(1.0 / shapetile), genus.erosion};
		target.profile = glm::vec4{genus.bottom, genus.top, genus.top_variation, genus.anvil};
		// the higher, the faster the wind carries them
		auto const offset{glm::dvec2{weather.cloud_offset()} * (1.0 + static_cast<double>(layer.base) / 8000.0)};
		// the camera's place in the noise, in the space stretched along the wind as the shaders take it
		auto const place{glm::dvec2{Camera.x, Camera.z} - offset};
		auto const along{glm::dot(place, wind)};
		auto const stretched{(place - wind * along) + wind * (along / std::max(static_cast<double>(genus.stretch), 1.0))};
		target.motion = glm::vec4{static_cast<float>(wrap(stretched.x, shapetile)), static_cast<float>(wrap(stretched.y, shapetile)), static_cast<float>(1.0 / coveragetile), genus.stretch};
		// the clouds change their shapes slowly; the seed moves the layers apart in the noise
		auto const evolution{std::fmod(static_cast<double>(conditions.seed) * 0.6180339887 + static_cast<double>(i) * 0.37 + time * 0.00002, 1.0)};
		target.extra = glm::vec4{genus.flat ? 1.f : 0.f, static_cast<float>(evolution), static_cast<float>(wrap(stretched.x, coveragetile)), static_cast<float>(wrap(stretched.y, coveragetile))};
		m_average_transmittance *= 1.f - std::clamp(layer.coverage * genus.opacity * std::min(layer.density, 1.5f), 0.f, 1.f);
	}
	if (Global.gfx_clouds <= 0)
		m_average_transmittance = 1.f;
	m_weather.fog_light = glm::vec4{m_sun_share, m_average_transmittance, 0.6f, fog_density()};
	m_weather.weather_camera = glm::vec4{height, static_cast<float>(std::fmod(time, 100000.0)), m_clouds_drawn ? 1.f : 0.f, m_clouds_drawn ? static_cast<float>(layers) : 0.f};
	m_weather.weather_sun_direction = glm::vec4{sun, sun.y > -0.02f ? 1.f : 0.f};
	m_weather.weather_sun_color = glm::vec4{Keycolor, 0.f};
	m_weather.weather_ambient = glm::vec4{Ambient * 1.3f, 0.f};
	m_weather.fog_volume_params = glm::vec4{0.f, FOG_VOLUME_NEAR, 1.f, 1.f};
	m_weather.fog_volume_target = glm::vec4{1.f, 1.f, Global.gfx_fog_range, 1.f - Sky.atmosphere_weight()};

	// the colours of the horizon: from the atmosphere on the gpu, the old fog colour otherwise
	m_horizon_on_gpu = (Sky.atmosphere_weight() > 0.f) && (Sky.skyview_table() != 0);
	if (false == m_horizon_on_gpu)
	{
		for (auto &color : m_weather.fog_horizon)
			color = glm::vec4{Fogcolor, 1.f};
	}

	// the shadows of the clouds, on a map snapped to its texels so it doesn't crawl as the camera moves
	auto const shadows{m_clouds_drawn && Global.gfx_clouds_shadows && (sun.y > 0.02f)};
	m_weather.cloud_shadow_params = glm::vec4{0.f};
	if (shadows)
	{
		if (m_shadow_size != Global.gfx_clouds_shadows_size)
		{
			release(m_shadow_map);
			m_shadow_size = Global.gfx_clouds_shadows_size;
			m_shadow_map = create_texture(GL_TEXTURE_2D, GL_R16F, glm::ivec3{m_shadow_size, m_shadow_size, 1}, GL_LINEAR, GL_CLAMP_TO_EDGE);
		}
		auto const extent{static_cast<double>(Global.gfx_clouds_shadows_range)};
		auto const texel{extent / static_cast<double>(m_shadow_size)};
		auto const centre{glm::dvec2{std::round(Camera.x / texel) * texel - Camera.x, std::round(Camera.z / texel) * texel - Camera.z}};
		m_weather.cloud_shadow_params = glm::vec4{static_cast<float>(centre.x), static_cast<float>(centre.y), static_cast<float>(1.0 / extent), 0.f};
	}
	upload_header();

	if (m_horizon_on_gpu)
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
		glProgramUniform3f(*m_shadow_program, 2, m_weather.cloud_shadow_params.x, m_weather.cloud_shadow_params.y, Global.gfx_clouds_shadows_range);
		glProgramUniform1i(*m_shadow_program, 3, 8);
		m_shadow_program->dispatch(groups(m_shadow_size), groups(m_shadow_size), 1);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R16F);
		m_shadow_timer.end();
		m_shadows_valid = true;
		m_weather.cloud_shadow_params.w = 1.f;
		upload_header();
	}
	else
		m_shadows_valid = false;

	// a band of the panorama of the clouds, all of it the first time
	if (m_clouds_drawn)
	{
		auto const level{quality(Global.gfx_clouds, Global.gfx_fog_quality)};
		if (m_panorama_size != level.panorama)
		{
			release(m_panorama);
			m_panorama_size = level.panorama;
			m_panorama = create_texture(GL_TEXTURE_2D, GL_RGBA16F, glm::ivec3{m_panorama_size, 1}, GL_LINEAR, GL_CLAMP_TO_EDGE);
			glTextureParameteri(m_panorama, GL_TEXTURE_WRAP_S, GL_REPEAT);
			m_panorama_full = false;
			m_panorama_band = 0;
		}
		auto const bandrows{(m_panorama_size.y + PANORAMA_BANDS - 1) / PANORAMA_BANDS};
		auto const first{m_panorama_full ? m_panorama_band * bandrows : 0};
		auto const rows{m_panorama_full ? bandrows : m_panorama_size.y};
		m_panorama_timer.begin();
		bind_noise();
		glBindImageTexture(0, m_panorama, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		// the depth isn't written in this mode, but something has to be bound
		glBindImageTexture(1, m_panorama, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glProgramUniform1i(*m_march_program, 0, 1);
		glProgramUniform2i(*m_march_program, 1, m_panorama_size.x, m_panorama_size.y);
		glProgramUniform1i(*m_march_program, 4, m_frame);
		// the panorama is seen blurred: fewer samples
		glProgramUniform4f(*m_march_program, 5, static_cast<float>(std::max(level.steps / 2, 24)), static_cast<float>(std::max(level.light_steps - 1, 2)), Global.gfx_clouds_distance, 0.f);
		glProgramUniform2i(*m_march_program, 6, first, rows);
		m_march_program->dispatch(groups(m_panorama_size.x), groups(rows), 1);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glBindImageTexture(1, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		m_panorama_timer.end();
		m_panorama_band = m_panorama_full ? (m_panorama_band + 1) % PANORAMA_BANDS : 0;
		m_panorama_full = true;
	}
	gl::program::unbind();
	m_report = to_metar(conditions);
}

void opengl46_weather::prepare_view(glm::ivec2 const &Size, quality_level const &Level)
{
	auto const cloudsize{glm::max((Size + Level.divisor - 1) / Level.divisor, glm::ivec2{1})};
	if (m_clouds_drawn && (cloudsize != m_view_size || m_clouds_color == 0))
	{
		for (auto *texture : {&m_clouds_color, &m_clouds_depth, &m_clouds_history[0], &m_clouds_history[1]})
			release(*texture);
		m_view_size = cloudsize;
		m_clouds_color = create_texture(GL_TEXTURE_2D, GL_RGBA16F, glm::ivec3{m_view_size, 1}, GL_LINEAR, GL_CLAMP_TO_EDGE);
		m_clouds_depth = create_texture(GL_TEXTURE_2D, GL_R32F, glm::ivec3{m_view_size, 1}, GL_NEAREST, GL_CLAMP_TO_EDGE);
		for (auto &history : m_clouds_history)
			history = create_texture(GL_TEXTURE_2D, GL_RGBA16F, glm::ivec3{m_view_size, 1}, GL_LINEAR, GL_CLAMP_TO_EDGE);
		m_previous_clouds.valid = false;
	}
	if ((Global.gfx_fog == 1) && (Level.fog_grid != m_fog_grid || m_fog_integrated == 0))
	{
		for (auto *texture : {&m_fog_scattering[0], &m_fog_scattering[1], &m_fog_integrated})
			release(*texture);
		m_fog_grid = Level.fog_grid;
		for (auto &scattering : m_fog_scattering)
			scattering = create_texture(GL_TEXTURE_3D, GL_RGBA16F, m_fog_grid, GL_LINEAR, GL_CLAMP_TO_EDGE);
		m_fog_integrated = create_texture(GL_TEXTURE_3D, GL_RGBA16F, m_fog_grid, GL_LINEAR, GL_CLAMP_TO_EDGE);
		m_previous_fog.valid = false;
	}
	m_view_level = Level;
}

void opengl46_weather::render_view(glm::mat4 const &Projection, glm::mat4 const &View, glm::dvec3 const &Camera, glm::ivec2 const &Size, int const Cablamp)
{
	auto const level{quality(Global.gfx_clouds, Global.gfx_fog_quality)};
	prepare_view(Size, level);
	auto const rotation{glm::mat4{glm::mat3{View}}};
	auto const inverseprojection{glm::inverse(Projection)};
	auto const viewtoworld{glm::transpose(rotation)};
	auto const projectionview{Projection * rotation};
	// a jump of the camera (another vehicle, the free camera) leaves nothing to blend with
	auto const continuity = [&](view_state const &Previous) { return Previous.valid && glm::length(Previous.camera - Camera) < 50.0; };

	if (m_clouds_drawn)
	{
		m_clouds_timer.begin();
		bind_noise();
		glBindImageTexture(0, m_clouds_color, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glBindImageTexture(1, m_clouds_depth, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
		glProgramUniform1i(*m_march_program, 0, 0);
		glProgramUniform2i(*m_march_program, 1, m_view_size.x, m_view_size.y);
		glProgramUniformMatrix4fv(*m_march_program, 2, 1, GL_FALSE, glm::value_ptr(inverseprojection));
		glProgramUniformMatrix4fv(*m_march_program, 3, 1, GL_FALSE, glm::value_ptr(viewtoworld));
		glProgramUniform1i(*m_march_program, 4, m_frame);
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
		glProgramUniformMatrix4fv(*m_temporal_program, 4, 1, GL_FALSE, glm::value_ptr(m_previous_clouds.projection_view));
		auto const previous{glm::vec3{m_previous_clouds.camera - Camera}};
		glProgramUniform3f(*m_temporal_program, 5, previous.x, previous.y, previous.z);
		glProgramUniform1f(*m_temporal_program, 6, continuity(m_previous_clouds) ? 0.15f : 1.f);
		m_temporal_program->dispatch(groups(m_view_size.x), groups(m_view_size.y), 1);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		// the shape noise back where the clouds of the fog read it
		glBindTextureUnit(gl::CLOUD_SHAPE_TEX, m_shape_noise);
		m_clouds_current = next;
		m_clouds_timer.end();
		m_previous_clouds = {projectionview, Camera, {}, true};
		m_clouds_view_valid = true;
	}
	else
		m_previous_clouds.valid = false;

	if (Global.gfx_fog == 1)
	{
		m_fog_timer.begin();
		auto const farthest{std::max(Global.gfx_fog_range, FOG_VOLUME_NEAR * 4.f)};
		// the cells are sampled at another depth in every frame, in the Halton sequence of 2
		auto const jitter{[](int Frame) {
			float value{0.f}, fraction{0.5f};
			for (Frame = (Frame % 64) + 1; Frame > 0; Frame /= 2, fraction *= 0.5f)
				value += fraction * static_cast<float>(Frame % 2);
			return value;
		}(m_frame)};
		auto const temporal{Global.gfx_fog_temporal && continuity(m_previous_fog)};
		glm::vec4 const slicing{FOG_VOLUME_NEAR, farthest, static_cast<float>(m_fog_grid.z) / std::log(farthest / FOG_VOLUME_NEAR), temporal ? jitter : 0.5f};
		bind_noise();
		auto const next{1 - m_fog_current};
		glBindImageTexture(0, m_fog_scattering[next], 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glBindTextureUnit(gl::CLOUD_HISTORY_TEX, m_fog_scattering[m_fog_current]);
		glProgramUniform3i(*m_inject_program, 0, m_fog_grid.x, m_fog_grid.y, m_fog_grid.z);
		glProgramUniformMatrix4fv(*m_inject_program, 1, 1, GL_FALSE, glm::value_ptr(inverseprojection));
		glProgramUniformMatrix4fv(*m_inject_program, 2, 1, GL_FALSE, glm::value_ptr(viewtoworld));
		glProgramUniform4fv(*m_inject_program, 3, 1, glm::value_ptr(slicing));
		glProgramUniformMatrix4fv(*m_inject_program, 4, 1, GL_FALSE, glm::value_ptr(m_previous_fog.projection_view));
		auto const previous{glm::vec3{m_previous_fog.camera - Camera}};
		glProgramUniform3f(*m_inject_program, 5, previous.x, previous.y, previous.z);
		auto const sunshadows{Global.gfx_fog_shadows && Global.gfx_shadowmap_enabled};
		glProgramUniform4f(*m_inject_program, 6, temporal ? 0.25f : 1.f, Global.gfx_fog_lights, sunshadows ? 1.f : 0.f, Global.gfx_fog_shadows ? 1.f : 0.f);
		glProgramUniform1i(*m_inject_program, 7, Cablamp);
		glProgramUniform2f(*m_inject_program, 8, m_previous_fog.slicing.x, m_previous_fog.slicing.z);
		m_inject_program->dispatch(groups(m_fog_grid.x), groups(m_fog_grid.y), static_cast<GLuint>(m_fog_grid.z));
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

		glBindTextureUnit(gl::CLOUD_HISTORY_TEX, m_fog_scattering[next]);
		glBindImageTexture(0, m_fog_integrated, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glProgramUniform3i(*m_integrate_program, 0, m_fog_grid.x, m_fog_grid.y, m_fog_grid.z);
		glProgramUniformMatrix4fv(*m_integrate_program, 1, 1, GL_FALSE, glm::value_ptr(inverseprojection));
		glProgramUniform4fv(*m_integrate_program, 3, 1, glm::value_ptr(slicing));
		m_integrate_program->dispatch(groups(m_fog_grid.x), groups(m_fog_grid.y), 1);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		m_fog_current = next;
		m_fog_slicing = slicing;
		m_fog_timer.end();
		m_previous_fog = {projectionview, Camera, slicing, true};
		m_fog_volume_valid = true;
	}
	else
		m_previous_fog.valid = false;
	gl::program::unbind();
}

void opengl46_weather::setup_pass(bool const Volume, glm::ivec2 const &Size)
{
	auto const volume{Volume && m_fog_volume_valid && (Global.gfx_fog == 1)};
	m_weather.fog_volume_params = volume ? glm::vec4{1.f, m_fog_slicing.x, m_fog_slicing.z, static_cast<float>(m_fog_grid.z)} : glm::vec4{0.f, FOG_VOLUME_NEAR, 1.f, 1.f};
	m_weather.fog_volume_target.x = 1.f / static_cast<float>(std::max(Size.x, 1));
	m_weather.fog_volume_target.y = 1.f / static_cast<float>(std::max(Size.y, 1));
	m_weather.fog_volume_target.z = volume ? m_fog_slicing.y : Global.gfx_fog_range;
	upload_header();
	bind();
}

void opengl46_weather::draw_stars(glm::mat4 const &View, float const Sunangle, double const Siderealtime, double const Latitude)
{
	if (!Global.gfx_stars || !m_stars_vao || m_stars_count == 0)
		return;
	// the stars come out between the civil and the nautical twilight
	auto const night{1.f - glm::smoothstep(-12.f, -3.f, Sunangle)};
	if (night <= 0.f)
		return;
	// the equatorial frame (z to the pole, x to the vernal equinox) to the sky of the observer (x west, y up, z north)
	auto const theta{glm::radians(Siderealtime)};
	auto const phi{glm::radians(Latitude)};
	auto const st{static_cast<float>(std::sin(theta))}, ct{static_cast<float>(std::cos(theta))};
	auto const sp{static_cast<float>(std::sin(phi))}, cp{static_cast<float>(std::cos(phi))};
	glm::mat3 equatorial;
	equatorial[0] = glm::vec3{st, cp * ct, -sp * ct};
	equatorial[1] = glm::vec3{-ct, cp * st, -sp * st};
	equatorial[2] = glm::vec3{0.f, sp, cp};
	auto const toworld{glm::mat4{equatorial}};
	auto const toview{glm::mat4{glm::mat3{View}}};
	m_stars_program->bind();
	glProgramUniformMatrix4fv(*m_stars_program, 0, 1, GL_FALSE, glm::value_ptr(toworld));
	glProgramUniformMatrix4fv(*m_stars_program, 1, 1, GL_FALSE, glm::value_ptr(toview));
	glProgramUniform4f(*m_stars_program, 2, STAR_BRIGHTNESS * Global.gfx_stars_brightness * night, static_cast<float>(std::fmod(Timer::GetTime(), 10000.0)), 2.5f, 0.f);
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
	if (m_clouds_drawn)
	{
		if (Mainview && m_clouds_view_valid)
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
	char const *fogmodes[]{"analytic (along the ray, as the old fog)", "volumetric"};
	ImGui::Combo("fog (gfx.fog)", &Global.gfx_fog, fogmodes, IM_ARRAYSIZE(fogmodes));
	char const *levels[]{"low", "medium", "high", "ultra"};
	auto fogquality{std::clamp(Global.gfx_fog_quality, 1, 4) - 1};
	if (ImGui::Combo("fog volume quality (gfx.fog.quality)", &fogquality, levels, IM_ARRAYSIZE(levels)))
		Global.gfx_fog_quality = fogquality + 1;
	ImGui::SliderFloat("fog volume depth, m (gfx.fog.range)", &Global.gfx_fog_range, 100.f, 10000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	ImGui::SliderFloat("lamps in the fog (gfx.fog.lights)", &Global.gfx_fog_lights, 0.f, 4.f);
	ImGui::Checkbox("shadows of the sun and headlights in the fog (gfx.fog.shadows)", &Global.gfx_fog_shadows);
	ImGui::Checkbox("fog blended over frames (gfx.fog.temporal)", &Global.gfx_fog_temporal);
	char const *cloudlevels[]{"none", "low", "medium", "high", "ultra"};
	ImGui::Combo("clouds (gfx.clouds)", &Global.gfx_clouds, cloudlevels, IM_ARRAYSIZE(cloudlevels));
	ImGui::SliderFloat("cloud distance, m (gfx.clouds.distance)", &Global.gfx_clouds_distance, 5000.f, 200000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	ImGui::Checkbox("cloud shadows (gfx.clouds.shadows)", &Global.gfx_clouds_shadows);
	ImGui::SliderFloat("cloud shadow map, m (gfx.clouds.shadows.range)", &Global.gfx_clouds_shadows_range, 1000.f, 100000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	ImGui::Checkbox("stars (gfx.stars)", &Global.gfx_stars);
	ImGui::SliderFloat("star brightness", &Global.gfx_stars_brightness, 0.f, 8.f);
	ImGui::Text("fog range %.0f m, sun share of the fog light %.2f, clouds let through %.2f on average", m_fog_range, m_sun_share, m_average_transmittance);
	ImGui::Separator();
	ui::weather_controls(simulation::Environment.weather());
}
