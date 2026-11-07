/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/grassfield.h"

#include "simulation/simulationenvironment.h"
#include "simulation/simulationtime.h"
#include "utilities/Globals.h"
#include "utilities/Timer.h"

namespace gfx
{

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

} // namespace

void grass_field::release(std::uint64_t const Id, chunk &Chunk)
{
	if (Chunk.uploaded)
		gpu_release(Id);
	Chunk.uploaded = false;
}

void grass_field::clear()
{
	for (auto &entry : m_chunks)
		release(entry.first, entry.second);
	m_chunks.clear();
}

void grass_field::upload(std::uint64_t const Id, chunk &Chunk, gfx::grass_chunk const &Source)
{
	release(Id, Chunk);
	Chunk.version = Source.version;
	Chunk.corner = Source.corner;
	Chunk.size = Source.size;
	Chunk.cells = Source.cells;
	Chunk.samples = Source.samples;
	Chunk.kinds = Source.kinds;
	gpu_upload(Id, Source);
	Chunk.uploaded = true;

	auto const side{Source.cells + 1};
	auto const samples{Source.samples + 1};
	auto const plane{static_cast<std::size_t>(samples) * samples};
	std::size_t kinds{0};
	while (kinds < Source.kinds.size() && Source.kinds[kinds] >= 0)
		++kinds;

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

void grass_field::update(std::vector<gfx::grass_kind> const &Kinds, std::vector<gfx::grass_chunk> const &Chunks)
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
		if (false == entry.uploaded || entry.version != source.version)
			upload(source.id, entry, source);
	}
	// what's no longer near goes
	for (auto it = m_chunks.begin(); it != m_chunks.end();)
	{
		if (it->second.listed)
		{
			++it;
			continue;
		}
		release(it->first, it->second);
		it = m_chunks.erase(it);
	}
}

bool grass_field::prepare(glm::dvec3 const &Camera, std::function<bool(scene::bounding_area const &)> const &Visible)
{
	if (false == Global.gfx_grass || m_chunks.empty() || m_kinds.empty())
		return false;

	m_chunkdraws.clear();
	m_tiledraws.clear();
	m_triangles = 0;
	// whole up to the distance, thinning out to the fading distance
	auto const fade{std::max(5.f, Global.gfx_grass_distance)};
	auto const reach{std::max(fade + 1.f, Global.gfx_grass_fadedistance)};
	auto const thinning{std::max(3.f, fade * 0.2f)};
	auto const multiplier{std::max(0.f, Global.gfx_grass_density)};
	auto const camera{Camera};

	m_frame.lod = glm::vec4{thinning, fade, reach, 0.f};
	auto const season{grass_season(simulation::Time.year_day())};
	m_frame.season = glm::vec4{season.x, season.y, season.z, 0.f};
	// the wind sways the blades along it, harder in the gusts
	auto const wind{glm::vec3{simulation::Environment.wind()}};
	auto const windspeed{glm::length(glm::vec2{wind.x, wind.z})};
	auto const winddirection{windspeed > 0.01f ? glm::vec2{wind.x, wind.z} / windspeed : glm::vec2{1.f, 0.f}};
	m_frame.wind = glm::vec4{winddirection.x, winddirection.y, std::clamp(windspeed / 15.f, 0.f, 1.f), static_cast<float>(std::fmod(Timer::GetTime(), 3600.0))};

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
				if (false == Visible(scene::bounding_area{(low + high) * 0.5, static_cast<float>(glm::length(high - low) * 0.5)}))
					continue;
				auto const full{tilesize * tilesize * densest * multiplier * fill};
				auto const count{std::min(max_blades, static_cast<int>(std::ceil(full * lod_share(distance, thinning, fade, reach))))};
				if (count <= 0)
					continue;
				if (false == set)
				{
					// the chunk is drawn relative to its corner, the heights as they are
					set = true;
					chunk_draw draw;
					draw.id = entry.first;
					draw.offset = glm::dvec3{chunk.corner.x - camera.x, -camera.y, chunk.corner.y - camera.z};
					draw.chunk = glm::vec4{chunk.size, static_cast<float>(chunk.cells), static_cast<float>(chunk.samples), 0.f};
					draw.camera = glm::vec4{static_cast<float>(camera.x - chunk.corner.x), static_cast<float>(camera.y), static_cast<float>(camera.z - chunk.corner.y), 0.f};
					draw.base = base;
					draw.tip = tip;
					draw.shape = shape;
					draw.share = share;
					draw.dry = dry;
					draw.first = m_tiledraws.size();
					m_chunkdraws.push_back(draw);
				}
				// fewer segments further away
				auto const segments{distance < thinning * 1.5f ? 3 : distance < fade * 0.5f ? 2 : 1};
				// the number of the tile in the world, the seed of its blades
				auto const seedx{static_cast<float>(std::floor(chunk.corner.x / tilesize + 0.5) + tx)};
				auto const seedz{static_cast<float>(std::floor(chunk.corner.y / tilesize + 0.5) + tz)};
				tile_draw draw;
				draw.tile = glm::vec4{tx * tilesize, tz * tilesize, tilesize, std::max(1.f, full)};
				draw.part = glm::vec4{fill, static_cast<float>(segments), seedx, seedz};
				draw.count = count;
				draw.segments = segments;
				m_tiledraws.push_back(draw);
				++m_chunkdraws.back().count;
				m_triangles += static_cast<std::size_t>(count) * (segments * 2 - 1);
			}
	}
	return true;
}

} // namespace gfx
