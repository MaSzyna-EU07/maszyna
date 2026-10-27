/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/skyocclusion.h"

#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "scene/scene.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "vehicle/DynObj.h"

namespace gfx
{

namespace
{

// the heights of a texel nothing covers: no top (lower than any scenery), no ceiling
float const NONE_TOP{-1.0e4f};
float const NONE_CEILING{1.0e4f};
bool has_top(float const Top)
{
	return Top > 0.5f * NONE_TOP;
}
bool has_ceiling(float const Ceiling)
{
	return Ceiling < 0.5f * NONE_CEILING;
}
// the limits of the heights between which the key light is let through, relative to the top: lit always, never
float const LIT_ABOVE_ALWAYS{-1000.f};
float const LIT_BELOW_ALWAYS{1000.f};
float const LIT_NEVER_BELOW{-1000.f};
// how high above the top the share of the sky above it is worked out (a surface on the ground isn't held back by the slope it's on),
// and how far below the ceiling the share under it (the middle of the height under a vault of a tunnel or a deck of a bridge)
float const ABOVE_TOP{1.f};
float const BELOW_CEILING{2.5f};
// a fragment this far below the top of its texel is under it, fully this much further. as in light_common.glsl, sky_occlusion()
float const COVER_START{0.75f};
float const COVER_SPAN{1.f};
// half the height over which the key light comes in at the edge of a shadow, metres. as in light_common.glsl
float const SUN_SOFTNESS{0.6f};
// a key light moving more than this (the cosine of the angle) has the occlusion of the key light worked out anew
double const KEYLIGHT_MOVED{0.99999}; // ~0.26 degree

sky_occlusion *g_current{nullptr};

int wrap(int const Value, int const Size)
{
	auto const result{Value % Size};
	return (result < 0 ? result + Size : result);
}

double wrap(double const Value, double const Size)
{
	return Value - std::floor(Value / Size) * Size;
}

std::uint64_t hash_mix(std::uint64_t Hash, std::uint64_t const Value)
{
	Hash ^= Value + 0x9e3779b97f4a7c15ull + (Hash << 6) + (Hash >> 2);
	return Hash;
}

std::uint64_t hash_mix_double(std::uint64_t Hash, double const Value)
{
	std::uint64_t bits;
	std::memcpy(&bits, &Value, sizeof(bits));
	return hash_mix(Hash, bits);
}

// the share of the sky of an elevation band, weighed by the cosine on a surface facing up, from the tangents of its ends
float sky_share(float const Tangent)
{
	if (Tangent <= 0.f)
		return 0.f;
	if (Tangent >= 1.0e6f)
		return 1.f;
	auto const squared{Tangent * Tangent};
	return squared / (1.f + squared);
}

float smoothstep(float const Edge0, float const Edge1, float const Value)
{
	auto const t{std::clamp((Value - Edge0) / (Edge1 - Edge0), 0.f, 1.f)};
	return t * t * (3.f - 2.f * t);
}

// the occlusion of a point at Height, over a texel of the given top and occlusion values: the share of the sky and of the key
// light. the same as light_common.glsl, sky_occlusion_texel()
glm::vec2 texel_occlusion(float const Top, glm::vec4 const &Occlusion, float const Height)
{
	auto const cover{std::clamp((Top - Height - COVER_START) / COVER_SPAN, 0.f, 1.f)};
	auto const sky{Occlusion.x + (Occlusion.y - Occlusion.x) * cover};
	auto const suntop{smoothstep(-SUN_SOFTNESS, SUN_SOFTNESS, Height - (Top + Occlusion.z))};
	auto const sununder{smoothstep(-SUN_SOFTNESS, SUN_SOFTNESS, (Top + Occlusion.w) - Height)};
	return {sky, suntop + (sununder - suntop) * cover};
}

// the geometry of a section made (and its binary terrain loaded) as the walk of the scene makes it when the section is drawn first,
// so what is gathered and signed is the scenery as it's drawn. the radii of the shapes are worked out before, from their vertices,
// which go to the geometry bank (a radius asked for after that would be 0)
void ready_section(scene::basic_section *Section)
{
	if (false == Section->m_geometrycreated)
	{
		for (auto &shape : Section->m_shapes)
			shape.radius();
		for (auto &cell : Section->m_cells)
		{
			for (auto &shape : cell.m_shapesopaque)
				shape.radius();
		}
	}
	Section->create_geometry();
}

// the reach of a shape for the tests against a tile: none known (0, the radius worked out too late) means the shape is tested
// triangle by triangle
double shape_reach(scene::shape_node &Shape)
{
	auto const radius{static_cast<double>(Shape.radius())};
	return (radius > 0.0 ? radius : std::numeric_limits<double>::max());
}

} // namespace

sky_occlusion::layout sky_occlusion::layout_of(int const Level)
{
	layout result;
	switch (Level)
	{
	case 1:
		// 256 texels of 1 m, the occlusion of the sky searched 32 m around
		result.tile = 32.f;
		result.tile_texels = 32;
		result.azimuths = 8;
		result.steps = 8;
		result.reach = 32.f;
		result.sun_steps = 12;
		result.sun_reach = 64.f;
		break;
	case 3:
		// 1024 texels of 0.5 m
		result.tile = 64.f;
		result.tile_texels = 128;
		result.azimuths = 16;
		result.steps = 12;
		result.reach = 64.f;
		result.sun_steps = 24;
		result.sun_reach = 128.f;
		break;
	default:
		// 512 texels of 1 m
		result.tile = 64.f;
		result.tile_texels = 64;
		result.azimuths = 12;
		result.steps = 10;
		result.reach = 48.f;
		result.sun_steps = 16;
		result.sun_reach = 128.f;
		break;
	}
	result.tiles = 8;
	return result;
}

sky_occlusion::~sky_occlusion()
{
	// the backend lets go of its textures itself, it's gone already
	if (m_thread.joinable())
	{
		{
			std::lock_guard<std::mutex> lock{m_mutex};
			m_quit = true;
		}
		m_wake.notify_all();
		m_thread.join();
	}
	if (g_current == this)
		g_current = nullptr;
}

sky_occlusion const *sky_occlusion::current()
{
	return (g_current != nullptr && g_current->active() ? g_current : nullptr);
}

void sky_occlusion::start(int const Level)
{
	m_layout = layout_of(Level);
	auto const side{m_layout.side()};
	if (false == gpu_allocate(side))
	{
		ErrorLog("sky occlusion: the maps can't be made, no occlusion of the sky");
		gpu_release();
		Global.gfx_skyvisibility = 0;
		m_level = 0;
		m_side = 0;
		return;
	}
	m_side = side;
	m_level = Level;
	m_sun = Global.gfx_skyvisibility_sun;
	auto const count{static_cast<std::size_t>(m_layout.tiles * m_layout.tiles)};
	m_slots.assign(count, slot{});
	auto const texels{static_cast<std::size_t>(side) * side};
	m_heights.assign(texels, NONE_TOP);
	m_occlusion.assign(texels, glm::vec4{1.f, 1.f, LIT_ABOVE_ALWAYS, LIT_BELOW_ALWAYS});
	auto const tiletexels{static_cast<std::size_t>(m_layout.tile_texels) * m_layout.tile_texels};
	m_neutralheights.assign(tiletexels, NONE_TOP);
	m_neutralocclusion.resize(tiletexels * 4);
	for (std::size_t i = 0; i < tiletexels; ++i)
	{
		m_neutralocclusion[i * 4 + 0] = glm::packHalf1x16(1.f);
		m_neutralocclusion[i * 4 + 1] = glm::packHalf1x16(1.f);
		m_neutralocclusion[i * 4 + 2] = glm::packHalf1x16(LIT_ABOVE_ALWAYS);
		m_neutralocclusion[i * 4 + 3] = glm::packHalf1x16(LIT_BELOW_ALWAYS);
	}
	for (int index = 0; index < static_cast<int>(count); ++index)
	{
		gpu_upload((index % m_layout.tiles) * m_layout.tile_texels, (index / m_layout.tiles) * m_layout.tile_texels, m_layout.tile_texels, m_neutralheights.data(), m_neutralocclusion.data());
	}
	m_rawtop.assign(texels, NONE_TOP);
	m_rawceiling.assign(texels, NONE_CEILING);
	m_top.assign(texels, NONE_TOP);
	m_ceiling.assign(texels, NONE_CEILING);
	m_workertiles.assign(count, glm::ivec2{std::numeric_limits<int>::min()});
	m_workergenerations.assign(count, 0);
	m_workerraster.assign(count, false);
	m_workerdirty.assign(count, false);
	m_jobs.clear();
	m_results.clear();
	m_quit = false;
	m_workersun = m_sun;
	m_keylight = glm::vec3{0.f, 1.f, 0.f};
	m_workerkeylight = m_keylight;
	m_checkslot = 0;
	m_thread = std::thread{[this]() { work(); }};
	g_current = this;
	WriteLog("sky occlusion: " + std::to_string(side) + "x" + std::to_string(side) + " texels of " + to_string(m_layout.texel(), 2) + " m, tiles of " + to_string(m_layout.tile, 0) + " m");
}

void sky_occlusion::stop()
{
	if (m_thread.joinable())
	{
		{
			std::lock_guard<std::mutex> lock{m_mutex};
			m_quit = true;
		}
		m_wake.notify_all();
		m_thread.join();
	}
	if (m_side > 0)
		gpu_release();
	m_side = 0;
	m_level = 0;
	m_slots.clear();
	m_heights.clear();
	m_occlusion.clear();
	m_rawtop.clear();
	m_rawceiling.clear();
	m_top.clear();
	m_ceiling.clear();
	m_jobs.clear();
	m_results.clear();
	if (g_current == this)
		g_current = nullptr;
}

void sky_occlusion::clear()
{
	// once, until a scenery is there again
	if ((m_side <= 0) || (m_region == nullptr))
		return;
	auto const level{m_level};
	stop();
	start(level);
	m_region = nullptr;
}

int sky_occlusion::slot_of(glm::ivec2 const &Tile) const
{
	return wrap(Tile.y, m_layout.tiles) * m_layout.tiles + wrap(Tile.x, m_layout.tiles);
}

void sky_occlusion::neutral(int const Slot)
{
	auto const px{m_layout.tile_texels};
	auto const x0{(Slot % m_layout.tiles) * px};
	auto const z0{(Slot / m_layout.tiles) * px};
	for (int z = 0; z < px; ++z)
	{
		auto const row{static_cast<std::size_t>(z0 + z) * m_side + x0};
		std::fill_n(m_heights.begin() + row, px, NONE_TOP);
		std::fill_n(m_occlusion.begin() + row, px, glm::vec4{1.f, 1.f, LIT_ABOVE_ALWAYS, LIT_BELOW_ALWAYS});
	}
	gpu_upload(x0, z0, px, m_neutralheights.data(), m_neutralocclusion.data());
	++m_uploaded;
}

void sky_occlusion::apply(tile_result &Result)
{
	auto &target{m_slots[Result.slot]};
	if (target.generation != Result.generation)
		return;
	target.state = slot_state::ready;
	auto const px{m_layout.tile_texels};
	auto const x0{(Result.slot % m_layout.tiles) * px};
	auto const z0{(Result.slot / m_layout.tiles) * px};
	for (int z = 0; z < px; ++z)
	{
		auto const row{static_cast<std::size_t>(z0 + z) * m_side + x0};
		for (int x = 0; x < px; ++x)
		{
			auto const source{static_cast<std::size_t>(z) * px + x};
			m_heights[row + x] = Result.heights[source];
			m_occlusion[row + x] = glm::vec4{glm::unpackHalf1x16(Result.occlusion[source * 4 + 0]), glm::unpackHalf1x16(Result.occlusion[source * 4 + 1]),
			                                 glm::unpackHalf1x16(Result.occlusion[source * 4 + 2]), glm::unpackHalf1x16(Result.occlusion[source * 4 + 3])};
		}
	}
	gpu_upload(x0, z0, px, Result.heights.data(), Result.occlusion.data());
	++m_uploaded;
}

void sky_occlusion::update(scene::basic_region *Region, glm::dvec3 const &Camera, glm::vec3 const &Keylight)
{
	auto const level{std::clamp(Global.gfx_skyvisibility, 0, 3)};
	if ((level != m_level) || ((m_side > 0) && (m_sun != Global.gfx_skyvisibility_sun)))
	{
		stop();
		if (level > 0)
			start(level);
	}
	if ((m_side <= 0) || (Region == nullptr))
		return;
	if (Region != m_region)
	{
		clear();
		m_region = Region;
		if (m_side <= 0)
			return;
	}

	// what the thread worked out
	{
		std::deque<tile_result> results;
		{
			std::lock_guard<std::mutex> lock{m_mutex};
			results.swap(m_results);
		}
		for (auto &result : results)
		{
			apply(result);
		}
	}

	auto const tiles{m_layout.tiles};
	auto const tilesize{static_cast<double>(m_layout.tile)};
	glm::ivec2 const cameratile{static_cast<int>(std::floor(Camera.x / tilesize)), static_cast<int>(std::floor(Camera.z / tilesize))};
	m_cameratile = cameratile;
	m_camera = Camera;
	// the tiles of the square around the camera: the slot of each holds the one of the tiles of the square it falls on
	glm::ivec2 const first{cameratile.x - tiles / 2, cameratile.y - tiles / 2};
	bool reassigned{false};
	for (int index = 0; index < tiles * tiles; ++index)
	{
		glm::ivec2 const wanted{first.x + wrap((index % tiles) - first.x, tiles), first.y + wrap((index / tiles) - first.y, tiles)};
		auto &target{m_slots[index]};
		if (target.tile == wanted)
			continue;
		target.tile = wanted;
		++target.generation;
		target.state = slot_state::empty;
		target.regather = false;
		target.signature = 0;
		neutral(index);
		reassigned = true;
	}

	// the key light, when it moved enough
	auto keylight{glm::length(Keylight) > 0.f ? glm::normalize(Keylight) : glm::vec3{0.f, 1.f, 0.f}};
	auto const keylightmoved{glm::dot(glm::dvec3{keylight}, glm::dvec3{m_keylight}) < KEYLIGHT_MOVED};
	if (keylightmoved)
		m_keylight = keylight;

	// the tiles which came in or whose scenery changed, the nearest first, one in a frame so the frame isn't held up; all at once when
	// the square is mostly empty (the start, a jump of the camera), so the thread lays them all out before it works out the occlusion
	// (which reads the tiles around) and does it once for each
	std::vector<std::pair<int, int>> wanted;
	auto empty{0};
	for (int index = 0; index < tiles * tiles; ++index)
	{
		auto const &candidate{m_slots[index]};
		if (candidate.state == slot_state::empty)
			++empty;
		if ((candidate.state == slot_state::empty) || candidate.regather)
		{
			auto const offset{candidate.tile - cameratile};
			wanted.emplace_back(offset.x * offset.x + offset.y * offset.y, index);
		}
	}
	std::sort(wanted.begin(), wanted.end());
	int queued{0};
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		queued = static_cast<int>(m_jobs.size());
	}
	auto const bulk{empty > tiles * tiles / 2};
	auto const budget{bulk ? tiles * tiles : 1};
	std::vector<raster_job> jobs;
	for (auto const &entry : wanted)
	{
		if ((static_cast<int>(jobs.size()) >= budget) || ((false == bulk) && (queued + static_cast<int>(jobs.size()) >= 8)))
			break;
		auto &target{m_slots[entry.second]};
		auto const gatherstart{std::chrono::steady_clock::now()};
		raster_job job;
		job.slot = entry.second;
		job.generation = target.generation;
		target.signature = signature(Region, target.tile);
		gather(Region, target.tile, job.triangles);
		m_gathertime += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - gatherstart).count();
		m_triangles = static_cast<std::uint32_t>(job.triangles.size());
		if (target.regather)
			++m_regathered;
		++m_gathered;
		if (target.state == slot_state::empty)
			target.state = slot_state::queued;
		target.regather = false;
		jobs.emplace_back(std::move(job));
	}
	// the scenery of a tile worked out already, checked for changes a little at a time (the editor, the streamed terrain, the
	// models shown and hidden by the events)
	if ((jobs.empty()) && (false == m_slots.empty()))
	{
		for (int check = 0; check < 2; ++check)
		{
			m_checkslot = (m_checkslot + 1) % m_slots.size();
			auto &target{m_slots[m_checkslot]};
			if ((target.state == slot_state::ready) && (false == target.regather))
			{
				if (signature(Region, target.tile) != target.signature)
					target.regather = true;
			}
		}
	}

	{
		std::lock_guard<std::mutex> lock{m_mutex};
		if (reassigned)
		{
			for (int index = 0; index < tiles * tiles; ++index)
			{
				if ((m_workertiles[index] == m_slots[index].tile) && (m_workergenerations[index] == m_slots[index].generation))
					continue;
				m_workertiles[index] = m_slots[index].tile;
				m_workergenerations[index] = m_slots[index].generation;
				// the thread clears what it kept of the tile which left, before anything else
				m_workerraster[index] = false;
				m_workerdirty[index] = true;
			}
		}
		for (auto &job : jobs)
		{
			m_jobs.emplace_back(std::move(job));
		}
		m_workercamera = cameratile;
		if (keylightmoved || (m_workersun != m_sun))
		{
			m_workerkeylight = m_keylight;
			m_workersun = m_sun;
			for (std::size_t index = 0; index < m_workerdirty.size(); ++index)
			{
				if (m_workerraster[index])
					m_workerdirty[index] = true;
			}
		}
		m_pending = static_cast<int>(m_jobs.size());
	}
	m_wake.notify_one();
}

void sky_occlusion::setup_scene(gl::scene_ubs &Scene, glm::dvec3 const &Camera) const
{
	if (m_side <= 0)
	{
		clear_scene(Scene);
		return;
	}
	auto const size{static_cast<double>(m_layout.tile) * m_layout.tiles};
	// w: 1 with the maps, 2 when the fog reads them too
	Scene.sky_occlusion_origin = glm::vec4{static_cast<float>(wrap(Camera.x, size)), static_cast<float>(wrap(Camera.z, size)), static_cast<float>(Camera.y), Global.gfx_skyvisibility_fog ? 2.f : 1.f};
	// the square reaches at least this far from the camera it follows on every side
	auto const reach{m_layout.tile * static_cast<float>(m_layout.tiles / 2 - 1)};
	Scene.sky_occlusion_params = glm::vec4{static_cast<float>(size), static_cast<float>(m_side), reach * 0.8f, reach * 0.2f};
	// the camera of the pass may be another one (an extra view): what is away from the camera the maps follow fades out
	Scene.sky_occlusion_center = glm::vec4{static_cast<float>(m_camera.x - Camera.x), static_cast<float>(m_camera.z - Camera.z), 0.f, 0.f};
}

void sky_occlusion::clear_scene(gl::scene_ubs &Scene)
{
	Scene.sky_occlusion_origin = glm::vec4{0.f};
	Scene.sky_occlusion_params = glm::vec4{0.f};
	Scene.sky_occlusion_center = glm::vec4{0.f};
}

glm::vec2 sky_occlusion::sample(glm::dvec3 const &Position) const
{
	if (m_side <= 0)
		return {1.f, 1.f};
	// faded out towards the edges of the square, as the shading does
	auto const reach{m_layout.tile * static_cast<float>(m_layout.tiles / 2 - 1)};
	auto const distance{static_cast<float>(std::max(std::abs(Position.x - m_camera.x), std::abs(Position.z - m_camera.z)))};
	auto const fade{std::clamp((distance - reach * 0.8f) / (reach * 0.2f), 0.f, 1.f)};
	if (fade >= 1.f)
		return {1.f, 1.f};
	auto const texel{static_cast<double>(m_layout.texel())};
	auto const u{Position.x / texel - 0.5};
	auto const v{Position.z / texel - 0.5};
	auto const i0{static_cast<std::int64_t>(std::floor(u))};
	auto const j0{static_cast<std::int64_t>(std::floor(v))};
	auto const fu{static_cast<float>(u - static_cast<double>(i0))};
	auto const fv{static_cast<float>(v - static_cast<double>(j0))};
	auto const height{static_cast<float>(Position.y)};
	glm::vec2 result{0.f};
	for (int j = 0; j < 2; ++j)
	{
		for (int i = 0; i < 2; ++i)
		{
			auto const x{static_cast<int>(((i0 + i) % m_side + m_side) % m_side)};
			auto const z{static_cast<int>(((j0 + j) % m_side + m_side) % m_side)};
			auto const index{static_cast<std::size_t>(z) * m_side + x};
			auto const weight{(i == 0 ? 1.f - fu : fu) * (j == 0 ? 1.f - fv : fv)};
			result += texel_occlusion(m_heights[index], m_occlusion[index], height) * weight;
		}
	}
	return glm::mix(result, glm::vec2{1.f}, fade);
}

// what the scenery over a tile is made of, without its triangles: the geometry of the shapes, the instances with their places and
// whether they're shown. sections and cells are the ones gather() goes through
std::uint64_t sky_occlusion::signature(scene::basic_region *Region, glm::ivec2 const &Tile)
{
	auto const tilesize{static_cast<double>(m_layout.tile)};
	glm::dvec2 const corner{Tile.x * tilesize, Tile.y * tilesize};
	glm::dvec2 const far{corner + glm::dvec2{tilesize}};
	glm::dvec3 const center{corner.x + 0.5 * tilesize, 0.0, corner.y + 0.5 * tilesize};
	auto const overlaps = [&](scene::bounding_area const &Area, double const Radius) {
		return (Area.center.x + Radius >= corner.x) && (Area.center.x - Radius <= far.x) && (Area.center.z + Radius >= corner.y) && (Area.center.z - Radius <= far.y);
	};
	std::uint64_t hash{0x5ca1ab1e};
	auto const &found{Region->sections(center, static_cast<float>(tilesize * 0.75))};
	std::vector<scene::basic_section *> const sections{found.begin(), found.end()};
	for (auto *section : sections)
	{
		ready_section(section);
		hash = hash_mix(hash, static_cast<std::uint64_t>(section->m_shapes.size()));
		for (auto &shape : section->m_shapes)
		{
			auto const &data{shape.data()};
			if (overlaps(data.area, shape_reach(shape)))
				hash = hash_mix(hash, static_cast<std::uint64_t>(data.geometry) ^ static_cast<std::uint64_t>(data.visible ? 1 : 0));
		}
		for (auto &cell : section->m_cells)
		{
			if (false == overlaps(cell.area(), static_cast<double>(cell.area().radius)))
				continue;
			hash = hash_mix(hash, static_cast<std::uint64_t>(cell.m_shapesopaque.size()));
			hash = hash_mix(hash, static_cast<std::uint64_t>(cell.m_instancesopaque.size()));
			for (auto &shape : cell.m_shapesopaque)
			{
				auto const &data{shape.data()};
				if (overlaps(data.area, shape_reach(shape)))
					hash = hash_mix(hash, static_cast<std::uint64_t>(data.geometry) ^ static_cast<std::uint64_t>(data.visible ? 1 : 0));
			}
			for (auto const *instance : cell.m_instancesopaque)
			{
				hash = hash_mix(hash, static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(instance)) ^ static_cast<std::uint64_t>(instance->visible() ? 1 : 0));
				auto const location{instance->location()};
				hash = hash_mix_double(hash, location.x);
				hash = hash_mix_double(hash, location.y);
				hash = hash_mix_double(hash, location.z);
				hash = hash_mix_double(hash, static_cast<double>(instance->vAngle.y));
			}
		}
	}
	return hash;
}

void sky_occlusion::gather(scene::basic_region *Region, glm::ivec2 const &Tile, std::vector<triangle> &Triangles)
{
	auto const tilesize{static_cast<double>(m_layout.tile)};
	glm::dvec2 const corner{Tile.x * tilesize, Tile.y * tilesize};
	glm::dvec2 const far{corner + glm::dvec2{tilesize}};
	glm::dvec3 const center{corner.x + 0.5 * tilesize, 0.0, corner.y + 0.5 * tilesize};
	auto const overlaps = [&](scene::bounding_area const &Area, double const Radius) {
		return (Area.center.x + Radius >= corner.x) && (Area.center.x - Radius <= far.x) && (Area.center.z + Radius >= corner.y) && (Area.center.z - Radius <= far.y);
	};
	auto const add = [&](glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C, glm::vec3 const &Normal) {
		// the bounds of the triangle against the tile
		auto const minx{std::min(A.x, std::min(B.x, C.x))};
		auto const maxx{std::max(A.x, std::max(B.x, C.x))};
		auto const minz{std::min(A.z, std::min(B.z, C.z))};
		auto const maxz{std::max(A.z, std::max(B.z, C.z))};
		if ((maxx < corner.x) || (minx > far.x) || (maxz < corner.y) || (minz > far.y))
			return;
		// whether it faces down: the side of the normals of its vertices (the side it's seen from), its winding taken where they
		// don't tell
		auto const geometric{glm::cross(glm::vec3{B - A}, glm::vec3{C - A})};
		auto down{false};
		if (std::abs(Normal.y) > 0.1f)
			down = (Normal.y < 0.f);
		else
			down = (geometric.y < 0.f);
		Triangles.push_back(triangle{glm::vec3{static_cast<float>(A.x - corner.x), static_cast<float>(A.y), static_cast<float>(A.z - corner.y)},
		                             glm::vec3{static_cast<float>(B.x - corner.x), static_cast<float>(B.y), static_cast<float>(B.z - corner.y)},
		                             glm::vec3{static_cast<float>(C.x - corner.x), static_cast<float>(C.y), static_cast<float>(C.z - corner.y)}, down});
	};
	auto const shape = [&](scene::shape_node &Shape) {
		auto const &data{Shape.data()};
		if ((false == data.visible) || data.translucent)
			return;
		if (false == overlaps(data.area, shape_reach(Shape)))
			return;
		if (data.geometry != gfx::geometry_handle{0, 0})
		{
			auto const &vertices{GfxRenderer->Vertices(data.geometry)};
			auto const &indices{GfxRenderer->Indices(data.geometry)};
			if ((GfxRenderer->Primitive(data.geometry) != GL_TRIANGLES) && (GfxRenderer->Primitive(data.geometry) != -1))
				return;
			auto const vertex = [&](std::size_t const Index) -> gfx::basic_vertex const & { return vertices[Index]; };
			auto const count{indices.empty() ? vertices.size() : indices.size()};
			for (std::size_t i = 0; i + 2 < count; i += 3)
			{
				auto const &a{vertex(indices.empty() ? i : indices[i])};
				auto const &b{vertex(indices.empty() ? i + 1 : indices[i + 1])};
				auto const &c{vertex(indices.empty() ? i + 2 : indices[i + 2])};
				add(data.origin + glm::dvec3{a.position}, data.origin + glm::dvec3{b.position}, data.origin + glm::dvec3{c.position}, a.normal + b.normal + c.normal);
			}
		}
		else
		{
			// not made into geometry yet: the source of it, in world space
			for (std::size_t i = 0; i + 2 < data.vertices.size(); i += 3)
			{
				add(data.vertices[i].position, data.vertices[i + 1].position, data.vertices[i + 2].position, data.vertices[i].normal + data.vertices[i + 1].normal + data.vertices[i + 2].normal);
			}
		}
	};
	auto const &sections{Region->sections(center, static_cast<float>(tilesize * 0.75))};
	std::vector<scene::basic_section *> const sectionlist{sections.begin(), sections.end()};
	for (auto *section : sectionlist)
	{
		ready_section(section);
		for (auto &entry : section->m_shapes)
			shape(entry);
		for (auto &cell : section->m_cells)
		{
			if (false == overlaps(cell.area(), static_cast<double>(cell.area().radius)))
				continue;
			for (auto &entry : cell.m_shapesopaque)
				shape(entry);
			for (auto *instance : cell.m_instancesopaque)
			{
				if ((instance == nullptr) || (false == instance->visible()) || (instance->pModel == nullptr))
					continue;
				if (false == overlaps(instance->m_area, static_cast<double>(instance->radius())))
					continue;
				// placed as the walk of the scene places it: translated, turned about y, x, z, scaled
				auto transform{glm::translate(glm::dmat4{1.0}, instance->location())};
				auto const &angle{instance->vAngle};
				if (angle.y != 0.f)
					transform = glm::rotate(transform, glm::radians(static_cast<double>(angle.y)), glm::dvec3{0.0, 1.0, 0.0});
				if (angle.x != 0.f)
					transform = glm::rotate(transform, glm::radians(static_cast<double>(angle.x)), glm::dvec3{1.0, 0.0, 0.0});
				if (angle.z != 0.f)
					transform = glm::rotate(transform, glm::radians(static_cast<double>(angle.z)), glm::dvec3{0.0, 0.0, 1.0});
				transform = glm::scale(transform, glm::dvec3{instance->Scale()});
				auto alpha{static_cast<std::uint32_t>(instance->Material() != nullptr ? instance->Material()->textures_alpha : 0x30300030)};
				alpha ^= 0x0F0F000F;
				gather_model(instance->pModel, alpha, transform, corner, far, Triangles);
			}
		}
	}
}

void sky_occlusion::gather_model(TModel3d *Model, std::uint32_t const Alpha, glm::dmat4 const &Transform, glm::dvec2 const &Corner, glm::dvec2 const &Far, std::vector<triangle> &Triangles)
{
	if ((Model == nullptr) || (Model->Root == nullptr))
		return;
	if (0 == (Alpha & Model->iFlags & 0x1F1F001F))
		return;
	gather_submodel(Model->Root, Alpha, Transform, Corner, Far, Triangles);
}

void sky_occlusion::gather_submodel(TSubModel *Submodel, std::uint32_t const Alpha, glm::dmat4 const &Transform, glm::dvec2 const &Corner, glm::dvec2 const &Far, std::vector<triangle> &Triangles)
{
	for (auto *part{Submodel}; part != nullptr; part = part->Next)
	{
		// the nearest level of detail, as it stands without its animations
		if ((0 == part->iVisible) || (part->fSquareMinDist > 0.f))
			continue;
		auto transform{Transform};
		if ((part->iFlags & 0xC000) && (part->fMatrix != nullptr))
			transform = transform * glm::dmat4{glm::make_mat4(part->fMatrix->readArray())};
		if ((part->eType == GL_TRIANGLES) && (Alpha & part->iFlags & 0x1F) && (part->m_geometry.handle != gfx::geometry_handle{0, 0}) && (part->fVisible > 0.f))
		{
			auto const &vertices{GfxRenderer->Vertices(part->m_geometry.handle)};
			auto const &indices{GfxRenderer->Indices(part->m_geometry.handle)};
			auto const count{indices.empty() ? vertices.size() : indices.size()};
			glm::dmat3 const normalmatrix{glm::transpose(glm::inverse(glm::dmat3{transform}))};
			for (std::size_t i = 0; i + 2 < count; i += 3)
			{
				auto const &a{vertices[indices.empty() ? i : indices[i]]};
				auto const &b{vertices[indices.empty() ? i + 1 : indices[i + 1]]};
				auto const &c{vertices[indices.empty() ? i + 2 : indices[i + 2]]};
				glm::dvec3 const pa{transform * glm::dvec4{glm::dvec3{a.position}, 1.0}};
				glm::dvec3 const pb{transform * glm::dvec4{glm::dvec3{b.position}, 1.0}};
				glm::dvec3 const pc{transform * glm::dvec4{glm::dvec3{c.position}, 1.0}};
				auto const minx{std::min(pa.x, std::min(pb.x, pc.x))};
				auto const maxx{std::max(pa.x, std::max(pb.x, pc.x))};
				auto const minz{std::min(pa.z, std::min(pb.z, pc.z))};
				auto const maxz{std::max(pa.z, std::max(pb.z, pc.z))};
				if ((maxx < Corner.x) || (minx > Far.x) || (maxz < Corner.y) || (minz > Far.y))
					continue;
				glm::vec3 const normal{normalmatrix * glm::dvec3{a.normal + b.normal + c.normal}};
				auto const geometric{glm::cross(glm::vec3{pb - pa}, glm::vec3{pc - pa})};
				auto const down{std::abs(normal.y) > 0.1f ? (normal.y < 0.f) : (geometric.y < 0.f)};
				Triangles.push_back(triangle{glm::vec3{static_cast<float>(pa.x - Corner.x), static_cast<float>(pa.y), static_cast<float>(pa.z - Corner.y)},
				                             glm::vec3{static_cast<float>(pb.x - Corner.x), static_cast<float>(pb.y), static_cast<float>(pb.z - Corner.y)},
				                             glm::vec3{static_cast<float>(pc.x - Corner.x), static_cast<float>(pc.y), static_cast<float>(pc.z - Corner.y)}, down});
			}
		}
		if (part->Child != nullptr)
			gather_submodel(part->Child, Alpha, transform, Corner, Far, Triangles);
	}
}

// the thread

bool sky_occlusion::worker_slot_valid(glm::ivec2 const &Tile) const
{
	auto const index{slot_of(Tile)};
	return m_workerraster[index] && (m_workertiles[index] == Tile);
}

void sky_occlusion::work()
{
	std::unique_lock<std::mutex> lock{m_mutex};
	while (false == m_quit)
	{
		m_wake.wait(lock, [this]() {
			if (m_quit || (false == m_jobs.empty()))
				return true;
			for (std::size_t index = 0; index < m_workerdirty.size(); ++index)
			{
				if (m_workerdirty[index])
					return true;
			}
			return false;
		});
		if (m_quit)
			break;
		// the tiles which left: what was kept of them goes
		for (std::size_t index = 0; index < m_workerdirty.size(); ++index)
		{
			if (m_workerdirty[index] && (false == m_workerraster[index]))
			{
				m_workerdirty[index] = false;
				auto const px{m_layout.tile_texels};
				auto const x0{static_cast<int>(index % m_layout.tiles) * px};
				auto const z0{static_cast<int>(index / m_layout.tiles) * px};
				for (int z = 0; z < px; ++z)
				{
					auto const row{static_cast<std::size_t>(z0 + z) * m_side + x0};
					std::fill_n(m_rawtop.begin() + row, px, NONE_TOP);
					std::fill_n(m_rawceiling.begin() + row, px, NONE_CEILING);
					std::fill_n(m_top.begin() + row, px, NONE_TOP);
					std::fill_n(m_ceiling.begin() + row, px, NONE_CEILING);
				}
			}
		}
		if (false == m_jobs.empty())
		{
			// the scenery of a tile
			auto job{std::move(m_jobs.front())};
			m_jobs.pop_front();
			if (job.generation != m_workergenerations[job.slot])
				continue;
			lock.unlock();
			auto const start{std::chrono::steady_clock::now()};
			rasterize(job);
			lock.lock();
			if (job.generation != m_workergenerations[job.slot])
				continue;
			m_workerraster[job.slot] = true;
			// the opening of the tile and of the ones around it, which reach into it, and their occlusion worked out anew
			auto const tile{m_workertiles[job.slot]};
			std::vector<int> around;
			for (int dz = -1; dz <= 1; ++dz)
			{
				for (int dx = -1; dx <= 1; ++dx)
				{
					auto const neighbour{tile + glm::ivec2{dx, dz}};
					if (worker_slot_valid(neighbour))
						around.push_back(slot_of(neighbour));
				}
			}
			lock.unlock();
			for (auto const index : around)
				filter(index);
			lock.lock();
			for (auto const index : around)
			{
				if (m_workerraster[index])
					m_workerdirty[index] = true;
			}
			m_workertime += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
			continue;
		}
		// the occlusion of the dirty tile nearest to the camera
		auto best{-1};
		auto bestdistance{std::numeric_limits<int>::max()};
		for (std::size_t index = 0; index < m_workerdirty.size(); ++index)
		{
			if (false == m_workerdirty[index])
				continue;
			auto const offset{m_workertiles[index] - m_workercamera};
			auto const distance{offset.x * offset.x + offset.y * offset.y};
			if (distance < bestdistance)
			{
				bestdistance = distance;
				best = static_cast<int>(index);
			}
		}
		if (best < 0)
			continue;
		m_workerdirty[best] = false;
		tile_result result;
		result.slot = best;
		result.generation = m_workergenerations[best];
		lock.unlock();
		auto const start{std::chrono::steady_clock::now()};
		occlude(best, result);
		auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()};
		lock.lock();
		m_workertime += elapsed;
		++m_workertiles_done;
		if (result.generation == m_workergenerations[best])
			m_results.emplace_back(std::move(result));
	}
}

// the triangles laid out over the texels of the tile from above: the highest of them and the lowest of those facing down
void sky_occlusion::rasterize(raster_job &Job)
{
	auto const px{m_layout.tile_texels};
	auto const texel{m_layout.texel()};
	auto const x0{(Job.slot % m_layout.tiles) * px};
	auto const z0{(Job.slot / m_layout.tiles) * px};
	for (int z = 0; z < px; ++z)
	{
		auto const row{static_cast<std::size_t>(z0 + z) * m_side + x0};
		std::fill_n(m_rawtop.begin() + row, px, NONE_TOP);
		std::fill_n(m_rawceiling.begin() + row, px, NONE_CEILING);
	}
	for (auto const &entry : Job.triangles)
	{
		// texel space, the centres of the texels at whole numbers
		glm::vec2 const a{entry.a.x / texel - 0.5f, entry.a.z / texel - 0.5f};
		glm::vec2 const b{entry.b.x / texel - 0.5f, entry.b.z / texel - 0.5f};
		glm::vec2 const c{entry.c.x / texel - 0.5f, entry.c.z / texel - 0.5f};
		auto const area{(b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)};
		if (std::abs(area) < 1e-6f)
			continue; // seen edge on
		auto const minx{std::max(0, static_cast<int>(std::ceil(std::min(a.x, std::min(b.x, c.x)))))};
		auto const maxx{std::min(px - 1, static_cast<int>(std::floor(std::max(a.x, std::max(b.x, c.x)))))};
		auto const minz{std::max(0, static_cast<int>(std::ceil(std::min(a.y, std::min(b.y, c.y)))))};
		auto const maxz{std::min(px - 1, static_cast<int>(std::floor(std::max(a.y, std::max(b.y, c.y)))))};
		if ((minx > maxx) || (minz > maxz))
			continue;
		auto const inverse{1.f / area};
		auto const tolerance{-1e-4f};
		for (int z = minz; z <= maxz; ++z)
		{
			auto const row{static_cast<std::size_t>(z0 + z) * m_side + x0};
			for (int x = minx; x <= maxx; ++x)
			{
				glm::vec2 const p{static_cast<float>(x), static_cast<float>(z)};
				auto const w0{((b.x - p.x) * (c.y - p.y) - (b.y - p.y) * (c.x - p.x)) * inverse};
				auto const w1{((c.x - p.x) * (a.y - p.y) - (c.y - p.y) * (a.x - p.x)) * inverse};
				auto const w2{1.f - w0 - w1};
				if ((w0 < tolerance) || (w1 < tolerance) || (w2 < tolerance))
					continue;
				auto const height{w0 * entry.a.y + w1 * entry.b.y + w2 * entry.c.y};
				auto &top{m_rawtop[row + x]};
				top = std::max(top, height);
				if (entry.down)
				{
					auto &ceiling{m_rawceiling[row + x]};
					ceiling = std::min(ceiling, height);
				}
			}
		}
	}
}

// the opening of the top and of the ceiling of a tile: what is narrower than three texels (poles, masts, fences, beams) goes, the
// rest keeps its shape. the texels of the tiles around it are read where they hold their tiles, nothing elsewhere
void sky_occlusion::filter(int const Slot)
{
	glm::ivec2 tile;
	std::vector<glm::ivec2> tiles;
	std::vector<bool> raster;
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		tile = m_workertiles[Slot];
		tiles = m_workertiles;
		raster = m_workerraster;
	}
	auto const px{m_layout.tile_texels};
	auto const margin{2};
	auto const span{px + 2 * margin};
	std::vector<float> top(static_cast<std::size_t>(span) * span), ceiling(top.size());
	for (int z = -margin; z < px + margin; ++z)
	{
		for (int x = -margin; x < px + margin; ++x)
		{
			glm::ivec2 const world{tile.x * px + x, tile.y * px + z};
			glm::ivec2 const worldtile{static_cast<int>(std::floor(static_cast<float>(world.x) / px)), static_cast<int>(std::floor(static_cast<float>(world.y) / px))};
			auto const slot{slot_of(worldtile)};
			auto const index{static_cast<std::size_t>(z + margin) * span + (x + margin)};
			if (raster[slot] && (tiles[slot] == worldtile))
			{
				auto const map{static_cast<std::size_t>(wrap(world.y, m_side)) * m_side + wrap(world.x, m_side)};
				top[index] = m_rawtop[map];
				ceiling[index] = m_rawceiling[map];
			}
			else
			{
				top[index] = NONE_TOP;
				ceiling[index] = NONE_CEILING;
			}
		}
	}
	// erosion then dilation of the top (the least of the largest), dilation then erosion of the ceiling
	std::vector<float> top2(top.size(), NONE_TOP), ceiling2(top.size(), NONE_CEILING);
	for (int z = 1; z < span - 1; ++z)
	{
		for (int x = 1; x < span - 1; ++x)
		{
			auto lowest{std::numeric_limits<float>::max()};
			auto highest{-std::numeric_limits<float>::max()};
			for (int dz = -1; dz <= 1; ++dz)
			{
				for (int dx = -1; dx <= 1; ++dx)
				{
					auto const index{static_cast<std::size_t>(z + dz) * span + (x + dx)};
					lowest = std::min(lowest, top[index]);
					highest = std::max(highest, ceiling[index]);
				}
			}
			top2[static_cast<std::size_t>(z) * span + x] = lowest;
			ceiling2[static_cast<std::size_t>(z) * span + x] = highest;
		}
	}
	auto const x0{(Slot % m_layout.tiles) * px};
	auto const z0{(Slot / m_layout.tiles) * px};
	for (int z = 0; z < px; ++z)
	{
		for (int x = 0; x < px; ++x)
		{
			auto highest{-std::numeric_limits<float>::max()};
			auto lowest{std::numeric_limits<float>::max()};
			for (int dz = -1; dz <= 1; ++dz)
			{
				for (int dx = -1; dx <= 1; ++dx)
				{
					auto const index{static_cast<std::size_t>(z + margin + dz) * span + (x + margin + dx)};
					highest = std::max(highest, top2[index]);
					lowest = std::min(lowest, ceiling2[index]);
				}
			}
			auto const map{static_cast<std::size_t>(z0 + z) * m_side + (x0 + x)};
			// the opening keeps what it keeps of the raster as it was (never higher), the ceiling under the top
			auto const opened{std::min(highest, m_rawtop[map])};
			m_top[map] = opened;
			m_ceiling[map] = (has_top(opened) && has_ceiling(lowest)) ? std::min(std::max(lowest, m_rawceiling[map]), opened) : NONE_CEILING;
		}
	}
}

// the occlusion of the texels of a tile, from the opened heights around it
void sky_occlusion::occlude(int const Slot, tile_result &Result)
{
	glm::ivec2 tile;
	std::vector<glm::ivec2> tiles;
	std::vector<bool> raster;
	glm::vec3 keylight;
	bool sun;
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		tile = m_workertiles[Slot];
		tiles = m_workertiles;
		raster = m_workerraster;
		keylight = m_workerkeylight;
		sun = m_workersun;
	}
	auto const px{m_layout.tile_texels};
	auto const texel{m_layout.texel()};
	auto const margin{static_cast<int>(std::ceil(std::max(m_layout.reach, m_layout.sun_reach) / texel)) + 1};
	auto const span{px + 2 * margin};
	// the heights around the tile, nothing where the tiles aren't known
	std::vector<float> top(static_cast<std::size_t>(span) * span), ceiling(top.size());
	for (int z = -margin; z < px + margin; ++z)
	{
		for (int x = -margin; x < px + margin; ++x)
		{
			glm::ivec2 const world{tile.x * px + x, tile.y * px + z};
			glm::ivec2 const worldtile{static_cast<int>(std::floor(static_cast<float>(world.x) / px)), static_cast<int>(std::floor(static_cast<float>(world.y) / px))};
			auto const slot{slot_of(worldtile)};
			auto const index{static_cast<std::size_t>(z + margin) * span + (x + margin)};
			if (raster[slot] && (tiles[slot] == worldtile))
			{
				auto const map{static_cast<std::size_t>(wrap(world.y, m_side)) * m_side + wrap(world.x, m_side)};
				top[index] = m_top[map];
				ceiling[index] = m_ceiling[map];
			}
			else
			{
				top[index] = NONE_TOP;
				ceiling[index] = NONE_CEILING;
			}
		}
	}
	auto const at = [&](int const X, int const Z) { return static_cast<std::size_t>(Z + margin) * span + (X + margin); };

	// the directions and the distances of the samples, in texels; the first one past the texel itself
	std::vector<glm::vec2> directions(m_layout.azimuths);
	for (int a = 0; a < m_layout.azimuths; ++a)
	{
		auto const angle{(static_cast<float>(a) + 0.5f) * glm::two_pi<float>() / static_cast<float>(m_layout.azimuths)};
		directions[a] = glm::vec2{std::cos(angle), std::sin(angle)};
	}
	auto const first{std::max(2.f, 1.5f * texel)};
	std::vector<float> distances(m_layout.steps);
	for (int k = 0; k < m_layout.steps; ++k)
	{
		auto const share{static_cast<float>(k + 1) / static_cast<float>(m_layout.steps)};
		distances[k] = first + (m_layout.reach - first) * share * share;
	}
	std::vector<float> sundistances(m_layout.sun_steps);
	for (int k = 0; k < m_layout.sun_steps; ++k)
	{
		auto const share{static_cast<float>(k + 1) / static_cast<float>(m_layout.sun_steps)};
		sundistances[k] = first + (m_layout.sun_reach - first) * share * share;
	}
	// the key light: its direction across the ground, and the rise of a ray towards it per metre across
	auto const across{std::sqrt(keylight.x * keylight.x + keylight.z * keylight.z)};
	auto const keyup{keylight.y > 0.01f};
	auto const overhead{across < 1e-4f};
	glm::vec2 const keydirection{overhead ? glm::vec2{1.f, 0.f} : glm::vec2{keylight.x, keylight.z} / std::max(across, 1e-4f)};
	auto const rise{overhead ? 1e6f : keylight.y / across};

	// the share of the sky seen from the height Origin over a texel, the search around it. a ceiling above the origin lets the sky
	// through below its underside, what has none blocks it from the horizon up to its top
	auto const sky = [&](int const X, int const Z, float const Origin) {
		float sum{0.f};
		for (auto const &direction : directions)
		{
			float low{0.f};
			float high{1.0e7f};
			for (auto const distance : distances)
			{
				auto const sx{X + static_cast<int>(std::lround(direction.x * distance / texel))};
				auto const sz{Z + static_cast<int>(std::lround(direction.y * distance / texel))};
				auto const index{at(sx, sz)};
				auto const t{top[index]};
				auto const c{ceiling[index]};
				if (has_ceiling(c) && (c > Origin + 0.05f))
					high = std::min(high, (c - Origin) / distance);
				else if (t > Origin)
					low = std::max(low, (t - Origin) / distance);
				if (low >= high)
					break;
			}
			sum += std::max(0.f, sky_share(high) - sky_share(low));
		}
		return sum / static_cast<float>(directions.size());
	};

	Result.heights.resize(static_cast<std::size_t>(px) * px);
	Result.occlusion.resize(static_cast<std::size_t>(px) * px * 4);
	for (int z = 0; z < px; ++z)
	{
		for (int x = 0; x < px; ++x)
		{
			auto const index{at(x, z)};
			auto const t0{top[index]};
			auto const c0{ceiling[index]};
			glm::vec4 value{1.f, 1.f, LIT_ABOVE_ALWAYS, LIT_BELOW_ALWAYS};
			if (has_top(t0))
			{
				auto const origin{t0 + ABOVE_TOP};
				value.x = sky(x, z, origin);
				// under a top with no ceiling is the inside of what is solid (a hill, a wall seen edge on): no sky, no sun
				value.y = 0.f;
				// the key light above the top: the height above which the rays towards it pass over all
				if (false == sun)
					value.z = LIT_ABOVE_ALWAYS;
				else if (false == keyup)
					value.z = LIT_BELOW_ALWAYS;
				else if (false == overhead)
				{
					auto shadow{NONE_TOP};
					for (auto const distance : sundistances)
					{
						auto const sx{x + static_cast<int>(std::lround(keydirection.x * distance / texel))};
						auto const sz{z + static_cast<int>(std::lround(keydirection.y * distance / texel))};
						auto const sample{at(sx, sz)};
						auto const drop{distance * rise};
						// under a ceiling the ray passes
						if (has_ceiling(ceiling[sample]) && (origin + drop < ceiling[sample]))
							continue;
						shadow = std::max(shadow, top[sample] - drop);
					}
					value.z = has_top(shadow) ? std::clamp(shadow - t0, LIT_ABOVE_ALWAYS, LIT_BELOW_ALWAYS) : LIT_ABOVE_ALWAYS;
				}
				else
					value.z = LIT_ABOVE_ALWAYS;
				if (has_ceiling(c0))
				{
					// under the ceiling
					auto const under{c0 - BELOW_CEILING};
					value.y = sky(x, z, under);
					if (false == sun)
						value.w = LIT_BELOW_ALWAYS;
					else if (false == keyup)
						value.w = LIT_NEVER_BELOW;
					else
					{
						// the height below which the rays towards the key light pass under the ceilings, out to the open
						auto limit{c0};
						auto out{false};
						for (auto const distance : sundistances)
						{
							auto const sx{x + static_cast<int>(std::lround(keydirection.x * distance / texel))};
							auto const sz{z + static_cast<int>(std::lround(keydirection.y * distance / texel))};
							auto const sample{at(sx, sz)};
							auto const drop{distance * rise};
							if (top[sample] <= under + drop)
							{
								out = true;
								break;
							}
							if (has_ceiling(ceiling[sample]) && (under + drop < ceiling[sample]))
								limit = std::min(limit, ceiling[sample] - drop);
							else
							{
								limit = NONE_TOP;
								break; // into the solid
							}
						}
						// the rays which don't come out within the reach stay in the tunnel
						value.w = (out && has_top(limit)) ? std::clamp(limit - t0, LIT_NEVER_BELOW, LIT_BELOW_ALWAYS) : LIT_NEVER_BELOW;
					}
				}
				else
					value.w = (sun ? LIT_NEVER_BELOW : LIT_BELOW_ALWAYS);
			}
			auto const target{static_cast<std::size_t>(z) * px + x};
			Result.heights[target] = t0;
			Result.occlusion[target * 4 + 0] = glm::packHalf1x16(value.x);
			Result.occlusion[target * 4 + 1] = glm::packHalf1x16(value.y);
			Result.occlusion[target * 4 + 2] = glm::packHalf1x16(value.z);
			Result.occlusion[target * 4 + 3] = glm::packHalf1x16(value.w);
		}
	}
}

std::string sky_occlusion::info() const
{
	if (m_side <= 0)
		return "sky occlusion: off";
	int ready{0};
	for (auto const &entry : m_slots)
	{
		if (entry.state == slot_state::ready)
			++ready;
	}
	double workertime;
	std::uint32_t done;
	{
		auto &mutex{const_cast<std::mutex &>(m_mutex)};
		std::lock_guard<std::mutex> lock{mutex};
		workertime = m_workertime;
		done = m_workertiles_done;
	}
	return "sky occlusion: " + std::to_string(m_side) + "x" + std::to_string(m_side) + " of " + to_string(m_layout.texel(), 2) + " m, tiles " + std::to_string(ready) + "/" +
	       std::to_string(m_slots.size()) + " ready, " + std::to_string(m_pending) + " queued; gathered " + std::to_string(m_gathered) + " (" + std::to_string(m_regathered) + " again, last " +
	       std::to_string(m_triangles) + " triangles), " + to_string(m_gathertime / std::max<std::uint32_t>(m_gathered, 1), 2) + " ms each on the main thread; " + std::to_string(done) +
	       " worked out in the background, " + to_string(workertime / std::max<std::uint32_t>(done, 1), 1) + " ms each; " + std::to_string(m_uploaded) + " uploads";
}

void sky_occlusion::draw_debug_ui()
{
	char const *levels[]{"off (no occlusion)", "low (256 m, 1 m texels)", "medium (512 m, 1 m texels)", "high (512 m, 0.5 m texels)"};
	ImGui::Combo("occlusion of the sky by the scenery (gfx.skyvisibility)", &Global.gfx_skyvisibility, levels, IM_ARRAYSIZE(levels));
	ImGui::Checkbox("the sun held back too (gfx.skyvisibility.sun)", &Global.gfx_skyvisibility_sun);
	ImGui::Checkbox("in the volumetric fog (gfx.skyvisibility.fog)", &Global.gfx_skyvisibility_fog);
	ImGui::Checkbox("environment flags of the tracks darken as before (gfx.tunnelflags)", &Global.gfx_tunnelflags);
	if (m_side <= 0)
		return;
	ImGui::TextWrapped("%s", info().c_str());
	if (ImGui::Button("work the tiles out again"))
		clear();
	auto const light{sample(Global.pCamera.Pos)};
	ImGui::Text("at the camera: sky %.2f, key light %.2f", light.x, light.y);
	// a view of the square: the share of the sky, the key light or the top relative to the camera
	char const *modes[]{"no view", "sky above the top", "sky under the ceilings", "key light on the top", "height of the top"};
	ImGui::Combo("view", &m_previewmode, modes, IM_ARRAYSIZE(modes));
	if ((m_previewmode <= 0) || (GfxRenderer == nullptr) || (GfxRenderer->GetImguiRenderer() == nullptr))
		return;
	// the square as the camera sees it from above, the camera in the middle, north (+z) up
	auto const size{std::min(m_side, 256)};
	auto const step{static_cast<double>(m_layout.tile) * m_layout.tiles / size};
	m_previewrgba.resize(static_cast<std::size_t>(size) * size * 4);
	auto const camera{Global.pCamera.Pos};
	auto const texel{static_cast<double>(m_layout.texel())};
	for (int row = 0; row < size; ++row)
	{
		for (int column = 0; column < size; ++column)
		{
			auto const wx{camera.x + (column - size / 2) * step};
			auto const wz{camera.z + (size / 2 - row) * step};
			auto const x{wrap(static_cast<int>(std::floor(wx / texel)), m_side)};
			auto const z{wrap(static_cast<int>(std::floor(wz / texel)), m_side)};
			auto const index{static_cast<std::size_t>(z) * m_side + x};
			auto const top{m_heights[index]};
			auto const &occlusion{m_occlusion[index]};
			float value{0.f};
			switch (m_previewmode)
			{
			case 1:
				value = occlusion.x;
				break;
			case 2:
				value = occlusion.y;
				break;
			case 3:
				value = texel_occlusion(top, occlusion, top + 0.1f).y;
				break;
			default:
				value = has_top(top) ? std::clamp(0.5f + static_cast<float>(top - camera.y) / 40.f, 0.f, 1.f) : 0.f;
				break;
			}
			auto const level{static_cast<std::uint8_t>(std::clamp(value, 0.f, 1.f) * 255.f)};
			auto *pixel{&m_previewrgba[(static_cast<std::size_t>(row) * size + column) * 4]};
			pixel[0] = level;
			pixel[1] = level;
			pixel[2] = (has_top(top) || (m_previewmode != 4)) ? level : static_cast<std::uint8_t>(96);
			pixel[3] = 255;
			if ((row == size / 2) && (column == size / 2))
			{
				pixel[0] = 255;
				pixel[1] = 0;
				pixel[2] = 0;
			}
		}
	}
	imgui_renderer::image_options options;
	options.srgb = false;
	options.nearest = true;
	m_previewimage = GfxRenderer->GetImguiRenderer()->Update_Image(m_previewimage, m_previewrgba.data(), size, size, options);
	if (m_previewimage != 0)
		ImGui::Image(static_cast<ImTextureID>(m_previewimage), ImVec2{static_cast<float>(size), static_cast<float>(size)});
}

float vehicle_shade(TDynamicObject const *Vehicle)
{
	if ((Vehicle == nullptr) || (false == Global.gfx_tunnelflags))
		return 0.f;
	return Vehicle->fShade;
}

float vehicle_light_level(TDynamicObject const *Vehicle)
{
	if (Vehicle == nullptr)
		return 1.f;
	if (Global.gfx_tunnelflags)
		return (Vehicle->fShade > 0.f ? Vehicle->fShade : 1.f);
	auto const *occlusion{sky_occlusion::current()};
	if (occlusion == nullptr)
		return 1.f;
	// at the height of the windows
	auto const light{occlusion->sample(Vehicle->GetPosition() + glm::dvec3{0.0, 1.5, 0.0})};
	return std::clamp(0.35f * light.x + 0.65f * light.y, 0.f, 1.f);
}

} // namespace gfx
