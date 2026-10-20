/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46grass.h"

#include "rendering/openglcamera.h"
#include "model/Texture.h"
#include "simulation/simulationenvironment.h"
#include "simulation/simulationtime.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"

namespace
{

int constexpr max_blades{65536}; // drawn in a tile at most
float constexpr blade_reach{1.5f}; // metres the bounds of a tile reach above its ground, for the blades

// the share of the blades drawn at specified distance: fewer further away, for speed (from Thinning), and fewer and fewer from
// Fade to Reach, where the grass ends; the same as lod_share() of grass.vert
float lod_share(float const Distance, float const Thinning, float const Fade, float const Reach)
{
	auto const share{Distance <= Thinning ? 1.f : std::pow(Thinning / Distance, 1.5f)};
	auto const t{std::clamp((Distance - Fade) / std::max(Reach - Fade, 1e-3f), 0.f, 1.f)};
	return share * (1.f - t * t * (3.f - 2.f * t));
}

// the grass through the year, by the day of the year: x how dry it is, y how fresh (the green of spring), z how grown (share
// of the height). the turns follow the seasons of the textures (world_environment::compute_season(): winter till day 65,
// spring till 158, summer till 252, autumn till 341): dormant and dry in winter, greening up and growing through spring,
// fresh in May, drying out from August, the dry grass lying lower towards winter
glm::vec3 grass_season(int const Yearday)
{
	struct point
	{
		float day;
		glm::vec3 value;
	};
	static point const year[]{{1.f, {0.80f, 0.f, 0.55f}},    {65.f, {0.80f, 0.f, 0.55f}},   {100.f, {0.45f, 0.3f, 0.70f}}, {135.f, {0.05f, 1.f, 0.90f}},
	                          {170.f, {0.f, 0.4f, 1.f}},     {200.f, {0.10f, 0.f, 1.f}},    {240.f, {0.35f, 0.f, 1.f}},    {280.f, {0.50f, 0.f, 0.95f}},
	                          {320.f, {0.65f, 0.f, 0.80f}},  {341.f, {0.80f, 0.f, 0.60f}},  {367.f, {0.80f, 0.f, 0.55f}}};
	auto const day{std::clamp(static_cast<float>(Yearday), 1.f, 366.f)};
	for (std::size_t i = 1; i < std::size(year); ++i)
		if (day <= year[i].day)
		{
			auto const t{(day - year[i - 1].day) / (year[i].day - year[i - 1].day)};
			return glm::mix(year[i - 1].value, year[i].value, t * t * (3.f - 2.f * t));
		}
	return year[std::size(year) - 1].value;
}

GLuint make_texture(GLenum const Format, int const Width, int const Height, GLenum const Filter)
{
	GLuint texture{0};
	::glCreateTextures(GL_TEXTURE_2D, 1, &texture);
	::glTextureStorage2D(texture, 1, Format, Width, Height);
	::glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, Filter);
	::glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, Filter);
	::glTextureParameteri(texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	::glTextureParameteri(texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	return texture;
}

} // namespace

opengl46_grass::~opengl46_grass()
{
	// the textures go with the context; deleted here only if it's still there
	if (::glfwGetCurrentContext() != nullptr)
		clear();
}

bool opengl46_grass::init()
{
	if (m_state != 0)
		return m_state > 0;
	m_state = -1;
	try
	{
		gl::shader const vertex("grass.vert");
		gl::shader const fragment("grass.frag");
		m_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, fragment});
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("heightmap terrain: can't use the grass shader: " + std::string(e.what()));
		return false;
	}
	m_vao = std::make_unique<gl::vao>();
	auto const location = [&](char const *Name) { return ::glGetUniformLocation(*m_program, Name); };
	m_uniforms.tile = location("grass_tile");
	m_uniforms.part = location("grass_part");
	m_uniforms.chunk = location("grass_chunk");
	m_uniforms.camera = location("grass_camera");
	m_uniforms.lod = location("grass_lod");
	m_uniforms.wind = location("grass_wind");
	m_uniforms.base = location("grass_base");
	m_uniforms.tip = location("grass_tip");
	m_uniforms.shape = location("grass_shape");
	m_uniforms.share = location("grass_share");
	m_uniforms.dry = location("grass_dry");
	m_uniforms.season = location("grass_season");
	m_state = 1;
	return true;
}

void opengl46_grass::release(chunk &Chunk)
{
	for (auto *texture : {&Chunk.heights, &Chunk.density, &Chunk.holes})
		if (*texture != 0)
		{
			::glDeleteTextures(1, texture);
			*texture = 0;
		}
}

void opengl46_grass::clear()
{
	for (auto &entry : m_chunks)
		release(entry.second);
	m_chunks.clear();
}

void opengl46_grass::upload(chunk &Chunk, gfx::grass_chunk const &Source)
{
	release(Chunk);
	Chunk.version = Source.version;
	Chunk.corner = Source.corner;
	Chunk.size = Source.size;
	Chunk.cells = Source.cells;
	Chunk.samples = Source.samples;
	Chunk.kinds = Source.kinds;

	auto const side{Source.cells + 1};
	Chunk.heights = make_texture(GL_R32F, side, side, GL_NEAREST);
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	::glTextureSubImage2D(Chunk.heights, 0, 0, 0, side, side, GL_RED, GL_FLOAT, Source.heights);

	// the densities of the kinds side by side, as the channels of a texel
	auto const samples{Source.samples + 1};
	auto const plane{static_cast<std::size_t>(samples) * samples};
	std::size_t kinds{0};
	while (kinds < Source.kinds.size() && Source.kinds[kinds] >= 0)
		++kinds;
	std::vector<std::uint8_t> texels(plane * 4, 0);
	for (std::size_t kind = 0; kind < kinds; ++kind)
		for (std::size_t i = 0; i < plane; ++i)
			texels[i * 4 + kind] = Source.density[kind * plane + i];
	Chunk.density = make_texture(GL_RGBA8, samples, samples, GL_LINEAR);
	::glTextureSubImage2D(Chunk.density, 0, 0, 0, samples, samples, GL_RGBA, GL_UNSIGNED_BYTE, texels.data());

	std::uint8_t const none{0};
	auto const holeside{(Source.holes > 0 && Source.hole_mask != nullptr) ? Source.holes + 1 : 1};
	Chunk.holes = make_texture(GL_R8, holeside, holeside, GL_LINEAR);
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	::glTextureSubImage2D(Chunk.holes, 0, 0, 0, holeside, holeside, GL_RED, GL_UNSIGNED_BYTE, holeside > 1 ? Source.hole_mask : &none);
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

	// the ground and the densest place of every kind of each tile, to skip the empty tiles and to draw as many blades as needed
	auto const spacing{static_cast<double>(Source.size) / Source.cells};
	auto const step{static_cast<double>(Source.size) / Source.samples};
	auto const tilesize{static_cast<double>(Source.size) / tiles};
	for (int tz = 0; tz < tiles; ++tz)
		for (int tx = 0; tx < tiles; ++tx)
		{
			auto &part{Chunk.parts[static_cast<std::size_t>(tz) * tiles + tx]};
			auto const range = [](double const From, double const To, double const Step, int const Last) {
				return std::make_pair(std::clamp(static_cast<int>(std::floor(From / Step)), 0, Last), std::clamp(static_cast<int>(std::ceil(To / Step)), 0, Last));
			};
			auto const [gx0, gx1]{range(tx * tilesize, (tx + 1) * tilesize, spacing, Source.cells)};
			auto const [gz0, gz1]{range(tz * tilesize, (tz + 1) * tilesize, spacing, Source.cells)};
			part.low = std::numeric_limits<float>::max();
			part.high = -std::numeric_limits<float>::max();
			for (int z = gz0; z <= gz1; ++z)
				for (int x = gx0; x <= gx1; ++x)
				{
					auto const height{Source.heights[static_cast<std::size_t>(z) * side + x]};
					part.low = std::min(part.low, height);
					part.high = std::max(part.high, height);
				}
			part.most.fill(0.f);
			auto const [sx0, sx1]{range(tx * tilesize, (tx + 1) * tilesize, step, Source.samples)};
			auto const [sz0, sz1]{range(tz * tilesize, (tz + 1) * tilesize, step, Source.samples)};
			for (std::size_t kind = 0; kind < kinds; ++kind)
			{
				std::uint8_t most{0};
				for (int z = sz0; z <= sz1; ++z)
					for (int x = sx0; x <= sx1; ++x)
						most = std::max(most, Source.density[kind * plane + static_cast<std::size_t>(z) * samples + x]);
				part.most[kind] = most / 255.f;
			}
		}
}

void opengl46_grass::update(std::vector<gfx::grass_kind> const &Kinds, std::vector<gfx::grass_chunk> const &Chunks)
{
	m_kinds = Kinds;
	for (auto &entry : m_chunks)
		entry.second.listed = false;
	if (false == Global.gfx_grass)
	{
		clear();
		return;
	}
	for (auto const &source : Chunks)
	{
		if (source.heights == nullptr || source.density == nullptr || source.cells <= 0 || source.samples <= 0)
			continue;
		auto &entry{m_chunks[source.id]};
		entry.listed = true;
		if (entry.heights == 0 || entry.version != source.version)
			upload(entry, source);
	}
	// what's no longer near goes
	for (auto it = m_chunks.begin(); it != m_chunks.end();)
	{
		if (it->second.listed)
		{
			++it;
			continue;
		}
		release(it->second);
		it = m_chunks.erase(it);
	}
}

std::size_t opengl46_grass::render(opengl_camera const &Camera, glm::mat4 const &View, std::function<void(glm::mat4 const &)> const &Modelview)
{
	if (false == Global.gfx_grass || m_chunks.empty() || m_kinds.empty() || false == init())
		return 0;

	// whole up to the distance, thinning out to the fading distance
	auto const fade{std::max(5.f, Global.gfx_grass_distance)};
	auto const reach{std::max(fade + 1.f, Global.gfx_grass_fadedistance)};
	auto const thinning{std::max(3.f, fade * 0.2f)};
	auto const multiplier{std::max(0.f, Global.gfx_grass_density)};
	auto const camera{Camera.position()};

	m_program->bind();
	m_vao->bind();
	::glDisable(GL_CULL_FACE);
	::glUniform4f(m_uniforms.lod, thinning, fade, reach, 0.f);
	auto const season{grass_season(simulation::Time.year_day())};
	::glUniform4f(m_uniforms.season, season.x, season.y, season.z, 0.f);
	// the wind sways the blades along it, harder in the gusts
	auto const wind{glm::vec3{simulation::Environment.wind()}};
	auto const windspeed{glm::length(glm::vec2{wind.x, wind.z})};
	auto const winddirection{windspeed > 0.01f ? glm::vec2{wind.x, wind.z} / windspeed : glm::vec2{1.f, 0.f}};
	::glUniform4f(m_uniforms.wind, winddirection.x, winddirection.y, std::clamp(windspeed / 15.f, 0.f, 1.f), static_cast<float>(std::fmod(Timer::GetTime(), 3600.0)));

	std::size_t triangles{0};
	for (auto &entry : m_chunks)
	{
		auto &chunk{entry.second};
		// the kinds of the chunk, and the share of the blades of each against its densest
		std::array<glm::vec4, 4> base{}, tip{}, shape{}, dry{};
		glm::vec4 share{0.f};
		float densest{0.f};
		for (std::size_t i = 0; i < chunk.kinds.size(); ++i)
			if (chunk.kinds[i] >= 0 && chunk.kinds[i] < static_cast<int>(m_kinds.size()))
				densest = std::max(densest, m_kinds[chunk.kinds[i]].density);
		if (densest <= 0.f)
			continue;
		for (std::size_t i = 0; i < chunk.kinds.size(); ++i)
		{
			if (chunk.kinds[i] < 0 || chunk.kinds[i] >= static_cast<int>(m_kinds.size()))
				continue;
			auto const &kind{m_kinds[chunk.kinds[i]]};
			base[i] = glm::vec4{kind.base, 0.f};
			tip[i] = glm::vec4{kind.tip, 0.f};
			shape[i] = glm::vec4{kind.height, kind.variation, kind.width, kind.bend};
			dry[i] = glm::vec4{kind.dry, kind.seasonal};
			share[i] = kind.density / densest;
		}
		auto const tallest{std::max({shape[0].x, shape[1].x, shape[2].x, shape[3].x}) * 1.5f + blade_reach};

		auto const tilesize{chunk.size / tiles};
		bool set{false};
		for (int tz = 0; tz < tiles; ++tz)
			for (int tx = 0; tx < tiles; ++tx)
			{
				auto const &part{chunk.parts[static_cast<std::size_t>(tz) * tiles + tx]};
				auto const fill{std::min(1.f, glm::dot(glm::vec4{part.most[0], part.most[1], part.most[2], part.most[3]}, share))};
				if (fill <= 0.f)
					continue;
				// distance to the box of the tile
				glm::dvec3 const low{chunk.corner.x + tx * tilesize, part.low, chunk.corner.y + tz * tilesize};
				glm::dvec3 const high{low.x + tilesize, part.high + tallest, low.z + tilesize};
				auto const nearest{glm::clamp(camera, low, high)};
				auto const distance{static_cast<float>(glm::length(nearest - camera))};
				if (distance > reach)
					continue;
				if (false == Camera.visible(scene::bounding_area{(low + high) * 0.5, static_cast<float>(glm::length(high - low) * 0.5)}))
					continue;
				auto const full{tilesize * tilesize * densest * multiplier * fill};
				auto const count{std::min(max_blades, static_cast<int>(std::ceil(full * lod_share(distance, thinning, fade, reach))))};
				if (count <= 0)
					continue;
				if (false == set)
				{
					// the chunk is drawn relative to its corner, the heights as they are
					set = true;
					Modelview(glm::translate(View, glm::vec3{glm::dvec3{chunk.corner.x - camera.x, -camera.y, chunk.corner.y - camera.z}}));
					::glUniform4f(m_uniforms.chunk, chunk.size, static_cast<float>(chunk.cells), static_cast<float>(chunk.samples), 0.f);
					::glUniform4f(m_uniforms.camera, static_cast<float>(camera.x - chunk.corner.x), static_cast<float>(camera.y), static_cast<float>(camera.z - chunk.corner.y), 0.f);
					::glUniform4fv(m_uniforms.base, 4, glm::value_ptr(base[0]));
					::glUniform4fv(m_uniforms.tip, 4, glm::value_ptr(tip[0]));
					::glUniform4fv(m_uniforms.shape, 4, glm::value_ptr(shape[0]));
					::glUniform4fv(m_uniforms.share, 1, glm::value_ptr(share));
					::glUniform4fv(m_uniforms.dry, 4, glm::value_ptr(dry[0]));
					GLuint const textures[]{chunk.heights, chunk.density, chunk.holes};
					::glBindTextures(0, 3, textures);
				}
				// fewer segments further away
				auto const segments{distance < thinning * 1.5f ? 3 : distance < fade * 0.5f ? 2 : 1};
				// the number of the tile in the world, the seed of its blades
				auto const seedx{static_cast<float>(std::floor(chunk.corner.x / tilesize + 0.5) + tx)};
				auto const seedz{static_cast<float>(std::floor(chunk.corner.y / tilesize + 0.5) + tz)};
				::glUniform4f(m_uniforms.tile, tx * tilesize, tz * tilesize, tilesize, std::max(1.f, full));
				::glUniform4f(m_uniforms.part, fill, static_cast<float>(segments), seedx, seedz);
				::glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, segments * 2 + 1, count);
				triangles += static_cast<std::size_t>(count) * (segments * 2 - 1);
			}
	}

	::glEnable(GL_CULL_FACE);
	gl::vao::unbind();
	// the units of the textures were taken behind the back of the texture cache
	opengl_texture::reset_unit_cache();
	return triangles;
}
