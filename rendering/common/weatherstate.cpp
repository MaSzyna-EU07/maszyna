/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/weatherstate.h"

#include "environment/starcatalog.h"
#include "environment/weather.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "widgets/weatherpanel.h"

namespace
{

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

} // namespace

namespace gfx
{

weather_state::quality_level weather_state::quality(int const Clouds, int const Fog)
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

weather_state::weather_state()
{
	// a grey fog and no clouds until the first update
	for (auto &color : m_weather.fog_horizon)
		color = glm::vec4{0.5f, 0.5f, 0.5f, 1.f};
	m_weather.fog_haze = glm::vec4{3.f / 10000.f, 1.f / 800.f, SKY_HAZE_EXTINCTION, 1.f};
	m_weather.fog_light = glm::vec4{0.f, 1.f, 0.6f, 1.f / 5000.f};
}

void weather_state::update(float const Atmosphereweight, bool const Skyview, glm::vec3 const &Keylight, glm::vec3 const &Keycolor, glm::vec3 const &Ambient, glm::vec3 const &Fogcolor, glm::dvec3 const &Camera)
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
	m_weather.cloud_wind = glm::vec4{static_cast<float>(wind.x), static_cast<float>(wind.y), speed, m_cloud_detail_scale};
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
	m_weather.weather_sun_color = glm::vec4{Keycolor, m_cloud_sun_scale};
	m_weather.weather_ambient = glm::vec4{Ambient * 1.3f, m_cloud_sky_scale};
	m_weather.fog_volume_params = glm::vec4{0.f, FOG_VOLUME_NEAR, 1.f, 1.f};
	m_weather.fog_volume_target = glm::vec4{1.f, 1.f, Global.gfx_fog_range, 1.f - Atmosphereweight};

	// the colours of the horizon: from the atmosphere on the gpu, the old fog colour otherwise
	m_horizon_on_gpu = (Atmosphereweight > 0.f) && Skyview;
	if (false == m_horizon_on_gpu)
	{
		for (auto &color : m_weather.fog_horizon)
			color = glm::vec4{Fogcolor, 1.f};
	}

	// the shadows of the clouds
	m_shadows_wanted = m_clouds_drawn && Global.gfx_clouds_shadows && (sun.y > 0.02f);
	m_weather.cloud_shadow_params = glm::vec4{0.f};
	m_report = to_metar(conditions);
}

void weather_state::place_shadows(int const Size, glm::dvec3 const &Camera)
{
	// a map snapped to its texels so it doesn't crawl as the camera moves
	auto const extent{static_cast<double>(Global.gfx_clouds_shadows_range)};
	auto const texel{extent / static_cast<double>(Size)};
	auto const centre{glm::dvec2{std::round(Camera.x / texel) * texel - Camera.x, std::round(Camera.z / texel) * texel - Camera.z}};
	m_weather.cloud_shadow_params = glm::vec4{static_cast<float>(centre.x), static_cast<float>(centre.y), static_cast<float>(1.0 / extent), 0.f};
}

void weather_state::shadows_drawn(bool const Drawn)
{
	m_shadows_valid = Drawn;
	if (Drawn)
		m_weather.cloud_shadow_params.w = 1.f;
}

void weather_state::panorama_resized()
{
	m_panorama_full = false;
	m_panorama_band = 0;
}

void weather_state::panorama_band(glm::ivec2 const &Size, int &First, int &Rows)
{
	auto const bandrows{(Size.y + PANORAMA_BANDS - 1) / PANORAMA_BANDS};
	First = m_panorama_full ? m_panorama_band * bandrows : 0;
	Rows = m_panorama_full ? bandrows : Size.y;
	m_panorama_band = m_panorama_full ? (m_panorama_band + 1) % PANORAMA_BANDS : 0;
	m_panorama_full = true;
}

bool weather_state::continuity(view_state const &Previous, glm::dvec3 const &Camera)
{
	// a jump of the camera (another vehicle, the free camera) leaves nothing to blend with
	return Previous.valid && glm::length(Previous.camera - Camera) < 50.0;
}

glm::vec3 weather_state::fog_jitter(bool const Temporal) const
{
	// the cells are sampled at another point in every frame: along the depth in the Halton sequence of 2, across the view in
	// those of 3 and 5 (fog_inject.comp moves the point of every cell by a value of its own)
	auto const halton = [](int Index, int const Base) {
		float value{0.f}, fraction{1.f};
		for (; Index > 0; Index /= Base)
		{
			fraction /= static_cast<float>(Base);
			value += fraction * static_cast<float>(Index % Base);
		}
		return value;
	};
	auto const sample{(m_frame % 64) + 1};
	return Temporal ? glm::vec3{halton(sample, 3), halton(sample, 5), halton(sample, 2)} : glm::vec3{0.5f};
}

glm::vec4 weather_state::fog_slicing(int const Slices, glm::vec3 const &Jitter)
{
	auto const farthest{std::max(Global.gfx_fog_range, FOG_VOLUME_NEAR * 4.f)};
	return glm::vec4{FOG_VOLUME_NEAR, farthest, static_cast<float>(Slices) / std::log((farthest + gl::FOG_VOLUME_BIAS) / (FOG_VOLUME_NEAR + gl::FOG_VOLUME_BIAS)), Jitter.z};
}

void weather_state::fog_made(glm::vec4 const &Slicing, glm::ivec3 const &Grid)
{
	m_fog_slicing = Slicing;
	m_fog_grid = Grid;
	m_fog_volume_valid = true;
}

void weather_state::setup_pass(bool const Volume, glm::ivec2 const &Size)
{
	auto const volume{Volume && m_fog_volume_valid && (Global.gfx_fog == 1)};
	m_weather.fog_volume_params = volume ? glm::vec4{1.f, m_fog_slicing.x, m_fog_slicing.z, static_cast<float>(m_fog_grid.z)} : glm::vec4{0.f, FOG_VOLUME_NEAR, 1.f, 1.f};
	m_weather.fog_volume_target.x = 1.f / static_cast<float>(std::max(Size.x, 1));
	m_weather.fog_volume_target.y = 1.f / static_cast<float>(std::max(Size.y, 1));
	m_weather.fog_volume_target.z = volume ? m_fog_slicing.y : Global.gfx_fog_range;
}

std::vector<glm::vec4> weather_state::star_vertices()
{
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
	return stars;
}

bool weather_state::stars(float const Sunangle, double const Siderealtime, double const Latitude, glm::mat4 const &View, glm::mat4 &Toworld, glm::mat4 &Toview, glm::vec4 &Params) const
{
	if (!Global.gfx_stars)
		return false;
	// the stars come out between the civil and the nautical twilight
	auto const night{1.f - glm::smoothstep(-12.f, -3.f, Sunangle)};
	if (night <= 0.f)
		return false;
	// the equatorial frame (z to the pole, x to the vernal equinox) to the sky of the observer (x west, y up, z north)
	auto const theta{glm::radians(Siderealtime)};
	auto const phi{glm::radians(Latitude)};
	auto const st{static_cast<float>(std::sin(theta))}, ct{static_cast<float>(std::cos(theta))};
	auto const sp{static_cast<float>(std::sin(phi))}, cp{static_cast<float>(std::cos(phi))};
	glm::mat3 equatorial;
	equatorial[0] = glm::vec3{st, cp * ct, -sp * ct};
	equatorial[1] = glm::vec3{-ct, cp * st, -sp * st};
	equatorial[2] = glm::vec3{0.f, sp, cp};
	Toworld = glm::mat4{equatorial};
	Toview = glm::mat4{glm::mat3{View}};
	Params = glm::vec4{STAR_BRIGHTNESS * Global.gfx_stars_brightness * night, static_cast<float>(std::fmod(Timer::GetTime(), 10000.0)), 2.5f, 0.f};
	return true;
}

void weather_state::draw_debug_ui()
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
	ImGui::SliderFloat("share of a new frame in the fog (gfx.fog.temporal.share)", &Global.gfx_fog_temporal_share, 0.02f, 1.f);
	char const *cloudlevels[]{"none", "low", "medium", "high", "ultra"};
	ImGui::Combo("clouds (gfx.clouds)", &Global.gfx_clouds, cloudlevels, IM_ARRAYSIZE(cloudlevels));
	ImGui::SliderFloat("cloud distance, m (gfx.clouds.distance)", &Global.gfx_clouds_distance, 5000.f, 200000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	ImGui::Checkbox("cloud shadows (gfx.clouds.shadows)", &Global.gfx_clouds_shadows);
	ImGui::SliderFloat("cloud shadow map, m (gfx.clouds.shadows.range)", &Global.gfx_clouds_shadows_range, 1000.f, 100000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	ImGui::SliderFloat("cloud sunlight (tuning, not saved)", &m_cloud_sun_scale, 0.25f, 3.f);
	ImGui::SliderFloat("cloud sky light (tuning, not saved)", &m_cloud_sky_scale, 0.f, 3.f);
	ImGui::SliderFloat("cloud edge detail (tuning, not saved)", &m_cloud_detail_scale, 0.f, 3.f);
	ImGui::Checkbox("stars (gfx.stars)", &Global.gfx_stars);
	ImGui::SliderFloat("star brightness", &Global.gfx_stars_brightness, 0.f, 8.f);
	ImGui::Text("fog range %.0f m, sun share of the fog light %.2f, clouds let through %.2f on average", m_fog_range, m_sun_share, m_average_transmittance);
	ImGui::Separator();
	ui::weather_controls(simulation::Environment.weather());
}

} // namespace gfx
