/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorTerrainStreamer.hpp"

#include "model/vertex.h"
#include "rendering/renderer.h"
#include "scene/scene.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>

terrain_streamer EditorTerrain;

namespace
{

int constexpr worker_count{2};
std::size_t constexpr max_loading{12}; // chunks asked from the workers at a time
std::size_t constexpr max_meshing{12}; // meshes asked from the workers at a time
double constexpr upload_budget{0.004}; // seconds of a frame for taking over finished work
int constexpr materials_per_frame{6};
double constexpr look_ahead{8.0}; // seconds; chunks are loaded around where the camera will be by then
double constexpr look_ahead_limit{2000.0}; // metres
double constexpr region_half{scene::EU07_REGIONSIDESECTIONCOUNT * scene::EU07_SECTIONSIZE * 0.5 - heightmap::chunk_size};
double constexpr kPi{3.14159265358979323846};

double steady_seconds()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::uint64_t geometry_key(gfx::geometry_handle const &Geometry)
{
	return (static_cast<std::uint64_t>(Geometry.bank) << 32) | Geometry.chunk;
}

bool in_region(glm::dvec2 const &Point)
{
	return std::abs(Point.x) < region_half && std::abs(Point.y) < region_half;
}

glm::dvec3 chunk_centre(heightmap::chunk_key const &Key)
{
	auto const low{heightmap::chunk_corner(Key)};
	return {low.x + heightmap::chunk_size * 0.5, 0.0, low.y + heightmap::chunk_size * 0.5};
}

// index range of the grid points (spacing Step from Origin, Count points) within [Low, High]
std::pair<int, int> index_range(double const Origin, double const Step, int const Count, double const Low, double const High)
{
	auto const first{std::max(0, static_cast<int>(std::ceil((Low - Origin) / Step - 1e-9)))};
	auto const last{std::min(Count - 1, static_cast<int>(std::floor((High - Origin) / Step + 1e-9)))};
	return {first, last};
}

double falloff(double const Distance, double const Radius)
{
	// smooth cosine falloff: full strength at the centre, zero at the rim
	return 0.5 * (std::cos(kPi * Distance / Radius) + 1.0);
}

void erase_shape(std::vector<scene::shape_node> &Shapes, gfx::geometry_handle const &Geometry)
{
	for (auto it = Shapes.begin(); it != Shapes.end(); ++it)
	{
		auto const &geometry{it->data().geometry};
		if (geometry.bank == Geometry.bank && geometry.chunk == Geometry.chunk)
		{
			Shapes.erase(it);
			return;
		}
	}
}

} // namespace

terrain_streamer::~terrain_streamer()
{
	stop_workers();
}

std::string terrain_streamer::pack_path(heightmap::pack_key const &Key) const
{
	return m_directory + heightmap::pack_file_name(Key);
}

bool terrain_streamer::open(std::string const &Name)
{
	close();
	m_name = Name;
	m_directory = "terrain/" + Name + "/";
	m_manifest = heightmap::manifest();
	m_manifestchanged = false;
	auto const manifestpath{m_directory + "terrain.yaml"};
	std::error_code ec;
	if (std::filesystem::exists(manifestpath, ec))
	{
		std::string error;
		if (false == heightmap::load_manifest(manifestpath, m_manifest, &error))
			ErrorLog("Bad file: heightmap terrain description \"" + manifestpath + "\": " + error, logtype::file);
	}
	else if (false == std::filesystem::exists(m_directory, ec))
		WriteLog("Heightmap terrain \"" + m_directory + "\" has no files yet", logtype::generic);
	m_active = true;
	m_waterchanged = true;
	start_workers();
	WriteLog("Heightmap terrain: " + m_directory, logtype::generic);
	return true;
}

void terrain_streamer::close()
{
	stop_workers();
	if (simulation::Region != nullptr)
	{
		for (auto &entry : m_chunks)
			undraw(*entry.second);
		clear_water(true);
	}
	clear_water(false);
	m_chunks.clear();
	m_packs.clear();
	m_loading.clear();
	m_removed.clear();
	m_palette.clear();
	m_recording = 0;
	m_change = change();
	m_active = false;
	m_name.clear();
	m_directory.clear();
}

void terrain_streamer::reset()
{
	// the sections the shapes were put in are gone with the old region, they aren't to be touched
	stop_workers();
	for (auto &entry : m_chunks)
	{
		entry.second->m_section = nullptr;
		if (entry.second->m_material != null_handle && GfxRenderer)
			GfxRenderer->Terrain_Release(entry.second->m_material);
	}
	for (auto &slot : m_slots)
		slot.used = false;
	m_chunks.clear();
	m_packs.clear();
	m_loading.clear();
	m_removed.clear();
	m_palette.clear();
	clear_water(false);
	m_recording = 0;
	m_change = change();
	m_active = false;
	m_name.clear();
	m_directory.clear();
	m_hascamera = false;
}

// workers

void terrain_streamer::start_workers()
{
	if (false == m_workers.empty())
		return;
	m_stop = false;
	for (int i = 0; i < worker_count; ++i)
		m_workers.emplace_back(&terrain_streamer::worker, this);
}

void terrain_streamer::stop_workers()
{
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		m_stop = true;
		m_jobs.clear();
	}
	m_wakeup.notify_all();
	for (auto &worker : m_workers)
		if (worker.joinable())
			worker.join();
	m_workers.clear();
	std::lock_guard<std::mutex> lock{m_mutex};
	m_results.clear();
	m_busy = 0;
	m_stop = false;
	for (auto &entry : m_chunks)
		entry.second->m_building = false;
}

void terrain_streamer::submit(job Job)
{
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		m_jobs.push_back(std::move(Job));
	}
	m_wakeup.notify_one();
}

void terrain_streamer::worker()
{
	while (true)
	{
		job task;
		{
			std::unique_lock<std::mutex> lock{m_mutex};
			m_wakeup.wait(lock, [this]() { return m_stop || false == m_jobs.empty(); });
			if (m_stop)
				return;
			// the nearest work first
			auto const next{std::min_element(m_jobs.begin(), m_jobs.end(), [](job const &A, job const &B) { return A.priority < B.priority; })};
			task = std::move(*next);
			m_jobs.erase(next);
			++m_busy;
		}
		result done;
		done.type = task.type;
		done.pack = task.pack;
		done.chunk = task.chunk;
		done.generation = task.generation;
		done.stride = task.stride;
		done.version = task.version;
		switch (task.type)
		{
		case job::kind::pack:
		{
			auto const path{pack_path(task.pack)};
			std::error_code ec;
			if (std::filesystem::exists(path, ec))
			{
				auto file{std::make_shared<heightmap::pack_file>()};
				std::lock_guard<std::mutex> lock{m_filemutex};
				if (file->open(path, &done.error))
					done.file = file;
			}
			break;
		}
		case job::kind::chunk:
		{
			std::lock_guard<std::mutex> lock{m_filemutex};
			done.data = task.file->read(task.chunk, &done.error);
			break;
		}
		case job::kind::mesh:
		{
			auto const &neighbours{task.neighbours};
			heightmap::build_mesh(*task.data, task.stride, task.origin,
			                      [&neighbours](double const X, double const Z, double &Height) {
				                      for (auto const &neighbour : neighbours)
					                      if (neighbour != nullptr && neighbour->contains(X, Z))
					                      {
						                      Height = neighbour->height_at(X, Z);
						                      return true;
					                      }
				                      return false;
			                      },
			                      done.mesh);
			break;
		}
		}
		std::lock_guard<std::mutex> lock{m_mutex};
		m_results.push_back(std::move(done));
		--m_busy;
	}
}

void terrain_streamer::process(result &Result)
{
	switch (Result.type)
	{
	case job::kind::pack:
	{
		auto &pack{m_packs[Result.pack]};
		if (Result.generation != pack.generation)
			break; // the file was written meanwhile, and is known already
		if (Result.file != nullptr)
		{
			pack.file = std::move(Result.file);
			pack.status = pack_state::state::ready;
		}
		else
		{
			if (false == Result.error.empty())
				ErrorLog("Bad file: " + Result.error, logtype::file);
			pack.status = pack_state::state::missing;
		}
		break;
	}
	case job::kind::chunk:
	{
		m_loading.erase(Result.chunk);
		auto const pack{m_packs.find(heightmap::pack_of(Result.chunk))};
		if (pack == m_packs.end() || pack->second.generation != Result.generation)
			break; // read from a file which was written anew since
		if (Result.data == nullptr)
		{
			ErrorLog("Bad file: " + Result.error, logtype::file);
			break;
		}
		if (m_chunks.count(Result.chunk) > 0 || m_removed.count(Result.chunk) > 0)
			break;
		auto chunk{std::make_unique<editor_terrain>(std::move(Result.data))};
		auto const [low, high]{std::minmax_element(chunk->heights().begin(), chunk->heights().end())};
		auto const corner{chunk->data().corner()};
		chunk->m_low = {corner.x, *low, corner.y};
		chunk->m_high = {corner.x + heightmap::chunk_size, *high, corner.y + heightmap::chunk_size};
		m_chunks.emplace(Result.chunk, std::move(chunk));
		touch_neighbours(Result.chunk);
		break;
	}
	case job::kind::mesh:
	{
		auto const lookup{m_chunks.find(Result.chunk)};
		if (lookup == m_chunks.end())
			break;
		auto &chunk{*lookup->second};
		chunk.m_building = false;
		if (Result.version < chunk.m_builtversion)
			break;
		upload(chunk, Result.mesh, Result.stride);
		chunk.m_builtversion = Result.version;
		break;
	}
	}
}

// drawing

material_handle terrain_streamer::palette_material(std::uint16_t const Layer)
{
	auto const index{Layer < m_manifest.layers.size() ? Layer : std::uint16_t{0}};
	auto const lookup{m_palette.find(index)};
	if (lookup != m_palette.end())
		return lookup->second;
	auto const material{m_manifest.layers.empty() ? null_handle : GfxRenderer->Fetch_Material(m_manifest.layers[index].material)};
	m_palette.emplace(index, material);
	return material;
}

void terrain_streamer::update_material(editor_terrain &Chunk)
{
	auto const &data{Chunk.data()};
	std::vector<gfx::terrain_layer> layers;
	auto const count{std::max<std::size_t>(1, data.layer_count())};
	for (std::size_t slot = 0; slot < count; ++slot)
	{
		auto const layer{data.layers[slot] == heightmap::no_layer ? std::uint16_t{0} : data.layers[slot]};
		auto const size{layer < m_manifest.layers.size() ? m_manifest.layers[layer].size : 0.f};
		layers.push_back({palette_material(layer), size});
	}
	auto const corner{data.corner()};
	auto const packcorner{heightmap::pack_corner(heightmap::pack_of(data.key))};
	glm::vec3 const placement{static_cast<float>(corner.x - packcorner.x), static_cast<float>(corner.y - packcorner.y), static_cast<float>(heightmap::chunk_size)};
	auto const painted{data.paint > 0 && count > 1};
	auto const material{GfxRenderer->Terrain_Material(Chunk.m_material, layers, painted ? data.paint : 0, painted ? data.weights.data() : nullptr, placement)};
	if (Chunk.m_material != null_handle && Chunk.m_material != material)
		GfxRenderer->Terrain_Release(Chunk.m_material);
	Chunk.m_material = material;
	Chunk.m_paintedversion = Chunk.m_paintversion;
}

void terrain_streamer::upload(editor_terrain &Chunk, heightmap::mesh &Mesh, int const Stride)
{
	if (simulation::Region == nullptr || Mesh.vertices.empty())
		return;
	auto const cells{Chunk.cells()};
	auto const triangles{Mesh.indices.size() / 3};
	gfx::userdata_array nouserdata;

	// the mesh goes to a geometry slot made for its grid; a slot of the same kind is refilled in place
	auto fits = [&](geometry_slot const &Slot) { return Slot.cells == cells && Slot.stride == Stride; };
	if (Chunk.m_slot < 0 || false == fits(m_slots[Chunk.m_slot]))
	{
		if (Chunk.m_slot >= 0)
			m_slots[Chunk.m_slot].used = false;
		Chunk.m_slot = -1;
		for (std::size_t i = 0; i < m_slots.size(); ++i)
			if (false == m_slots[i].used && fits(m_slots[i]))
			{
				Chunk.m_slot = static_cast<int>(i);
				break;
			}
		if (Chunk.m_slot < 0)
		{
			geometry_slot slot;
			slot.bank = GfxRenderer->Create_Bank();
			slot.cells = cells;
			slot.stride = Stride;
			slot.geometry = GfxRenderer->Insert(Mesh.indices, Mesh.vertices, nouserdata, slot.bank, GL_TRIANGLES);
			if (slot.geometry.chunk == 0)
			{
				// the shape can't keep the geometry it had, it's given back already
				undraw(Chunk);
				return;
			}
			slot.used = true;
			Chunk.m_slot = static_cast<int>(m_slots.size());
			m_slotofgeometry[geometry_key(slot.geometry)] = Chunk.m_slot;
			m_slots.push_back(slot);
		}
		else
		{
			m_slots[Chunk.m_slot].used = true;
			GfxRenderer->Replace(Mesh.vertices, nouserdata, m_slots[Chunk.m_slot].geometry, GL_TRIANGLES);
		}
	}
	else
	{
		GfxRenderer->Replace(Mesh.vertices, nouserdata, m_slots[Chunk.m_slot].geometry, GL_TRIANGLES);
	}
	auto const &slot{m_slots[Chunk.m_slot]};

	if (Chunk.m_material == null_handle || Chunk.m_paintedversion != Chunk.m_paintversion)
		update_material(Chunk);

	// the shape gets just two bounding vertices: they give it its area, and leave no triangles behind for code which
	// reads the source vertices of the section shapes; the terrain answers questions about the ground itself
	auto const centre{chunk_centre(Chunk.key())};
	auto &section{simulation::Region->section(centre)};
	// the section makes the geometry of its own shapes once, before it's drawn; done first, it leaves ours alone
	section.create_geometry();
	auto const low{Mesh.low};
	auto const high{Mesh.high};
	std::vector<world_vertex> bounds(2);
	bounds[0].position = low;
	bounds[1].position = high;
	scene::shape_node shape;
	shape.make_terrain(Chunk.m_material, std::move(bounds), section.m_area.center);
	shape.geometry(slot.geometry);

	auto replaced{false};
	if (Chunk.m_section != nullptr)
	{
		for (auto &existing : Chunk.m_section->m_shapes)
		{
			auto const &geometry{existing.data().geometry};
			if (geometry.bank == Chunk.m_geometry.bank && geometry.chunk == Chunk.m_geometry.chunk)
			{
				existing = shape;
				replaced = true;
				break;
			}
		}
	}
	if (false == replaced)
		section.m_shapes.emplace_back(shape);
	// the section reaches far enough for the chunk not to be culled at its edges
	section.m_area.radius = std::max(section.m_area.radius, static_cast<float>(glm::length(section.m_area.center - Mesh.centre) + Mesh.radius));
	Chunk.m_section = &section;
	Chunk.m_geometry = slot.geometry;
	Chunk.m_triangles = triangles;
	Chunk.m_stride = Stride;
	Chunk.m_low = low;
	Chunk.m_high = high;
}

void terrain_streamer::undraw(editor_terrain &Chunk)
{
	if (Chunk.m_section != nullptr && simulation::Region != nullptr)
		erase_shape(Chunk.m_section->m_shapes, Chunk.m_geometry);
	if (Chunk.m_slot >= 0)
		m_slots[Chunk.m_slot].used = false;
	if (Chunk.m_material != null_handle && GfxRenderer)
		GfxRenderer->Terrain_Release(Chunk.m_material);
	Chunk.m_slot = -1;
	Chunk.m_material = null_handle;
	Chunk.m_section = nullptr;
	Chunk.m_geometry = gfx::geometry_handle{0, 0};
	Chunk.m_triangles = 0;
	Chunk.m_stride = 0;
	Chunk.m_builtversion = 0;
	Chunk.m_paintedversion = 0;
}

heightmap::chunk_ptr terrain_streamer::neighbour(chunk_key const &Key) const
{
	auto const lookup{m_chunks.find(Key)};
	return lookup != m_chunks.end() ? lookup->second->shared() : nullptr;
}

void terrain_streamer::touch_neighbours(chunk_key const &Key)
{
	for (auto const &offset : {chunk_key{-1, 0}, chunk_key{1, 0}, chunk_key{0, -1}, chunk_key{0, 1}})
	{
		auto const lookup{m_chunks.find({Key.first + offset.first, Key.second + offset.second})};
		if (lookup != m_chunks.end())
			lookup->second->m_neighbours = true;
	}
}

int terrain_streamer::wanted_stride(editor_terrain const &Chunk, glm::dvec3 const &Camera) const
{
	// distance from the camera to the box of the chunk
	glm::dvec3 const offset{std::max({Chunk.m_low.x - Camera.x, 0.0, Camera.x - Chunk.m_high.x}), std::max({Chunk.m_low.y - Camera.y, 0.0, Camera.y - Chunk.m_high.y}),
	                        std::max({Chunk.m_low.z - Camera.z, 0.0, Camera.z - Chunk.m_high.z})};
	auto const distance{glm::length(offset)};
	auto const spacing{Chunk.data().spacing()};
	auto const limit{heightmap::max_stride(Chunk.cells())};
	auto const stride_for = [&](double const Distance) {
		auto const edge{Distance / m_detail};
		int stride{1};
		while (stride * 2 <= limit && stride * 2 * spacing <= edge)
			stride *= 2;
		return stride;
	};
	// a margin around the distances the stride changes at, so a camera standing at one doesn't flip it to and fro
	if (Chunk.m_stride > 0)
	{
		auto const finer{stride_for(distance * 0.85)};
		auto const coarser{stride_for(distance * 1.15)};
		if (Chunk.m_stride >= finer && Chunk.m_stride <= coarser)
			return Chunk.m_stride;
	}
	return stride_for(distance);
}

void terrain_streamer::request_mesh(editor_terrain &Chunk, int const Stride, double const Priority)
{
	auto const centre{chunk_centre(Chunk.key())};
	job task;
	task.type = job::kind::mesh;
	task.chunk = Chunk.key();
	task.data = Chunk.shared();
	auto const key{Chunk.key()};
	task.neighbours = {neighbour({key.first - 1, key.second}), neighbour({key.first + 1, key.second}), neighbour({key.first, key.second - 1}), neighbour({key.first, key.second + 1})};
	task.stride = Stride;
	task.origin = simulation::Region->section(centre).m_area.center;
	task.version = Chunk.m_shapeversion;
	task.priority = Priority;
	Chunk.m_building = true;
	Chunk.m_neighbours = false;
	submit(std::move(task));
}

void terrain_streamer::update(glm::dvec3 const &Camera)
{
	if (false == m_active || simulation::Region == nullptr)
		return;

	// where the camera is heading
	auto const now{steady_seconds()};
	if (m_hascamera)
	{
		auto const elapsed{now - m_cameratime};
		if (elapsed > 1e-3)
		{
			auto velocity{(Camera - m_camera) / elapsed};
			if (glm::length(velocity) > 300.0)
				velocity = glm::dvec3{0.0}; // a jump, not a movement
			m_velocity = glm::mix(m_velocity, velocity, std::min(1.0, elapsed * 2.0));
		}
	}
	m_camera = Camera;
	m_cameratime = now;
	m_hascamera = true;
	auto ahead{m_velocity * look_ahead};
	if (glm::length(ahead) > look_ahead_limit)
		ahead *= look_ahead_limit / glm::length(ahead);
	auto const target{Camera + ahead};

	if (m_waterchanged)
		build_water();

	// finished work, within a few milliseconds
	while (steady_seconds() - now < upload_budget)
	{
		result done;
		{
			std::lock_guard<std::mutex> lock{m_mutex};
			if (m_results.empty())
				break;
			done = std::move(m_results.front());
			m_results.pop_front();
		}
		process(done);
	}

	// chunks wanted around the camera and around where it's heading
	auto const range{std::max(m_radius * heightmap::chunk_size, static_cast<double>(Global.BaseDrawRange) * std::max(1.0, static_cast<double>(Global.fDistanceFactor))) + heightmap::chunk_size * 0.5};
	auto const radius{std::min(64, static_cast<int>(std::ceil(range / heightmap::chunk_size)))};
	if (m_ringradius != radius)
	{
		m_ring.clear();
		for (int dz = -radius; dz <= radius; ++dz)
			for (int dx = -radius; dx <= radius; ++dx)
				if (dx * dx + dz * dz <= (radius + 1) * (radius + 1))
					m_ring.emplace_back(dx, dz);
		std::sort(m_ring.begin(), m_ring.end(), [](chunk_key const &A, chunk_key const &B) { return A.first * A.first + A.second * A.second < B.first * B.first + B.second * B.second; });
		m_ringradius = radius;
	}
	auto const here{heightmap::chunk_at(Camera.x, Camera.z)};
	auto const there{heightmap::chunk_at(target.x, target.z)};
	std::vector<chunk_key> centres{here};
	if (there != here)
		centres.push_back(there);
	for (auto const &centre : centres)
	{
		for (auto const &offset : m_ring)
		{
			if (m_loading.size() >= max_loading)
				break;
			chunk_key const key{centre.first + offset.first, centre.second + offset.second};
			if (m_chunks.count(key) > 0 || m_loading.count(key) > 0 || m_removed.count(key) > 0)
				continue;
			auto const packkey{heightmap::pack_of(key)};
			auto &pack{m_packs[packkey]};
			auto const distance{std::sqrt(static_cast<double>(offset.first * offset.first + offset.second * offset.second))};
			if (pack.status == pack_state::state::unknown)
			{
				pack.status = pack_state::state::loading;
				job task;
				task.type = job::kind::pack;
				task.pack = packkey;
				task.generation = pack.generation;
				task.priority = distance;
				submit(std::move(task));
				continue;
			}
			if (pack.status != pack_state::state::ready || false == pack.file->has(key))
				continue;
			job task;
			task.type = job::kind::chunk;
			task.chunk = key;
			task.pack = packkey;
			task.file = pack.file;
			task.generation = pack.generation;
			task.priority = distance;
			m_loading.insert(key);
			submit(std::move(task));
		}
	}

	// chunks gone out of range are dropped, except edited ones, which wait in memory to be saved
	auto const outside = [&](chunk_key const &Key, int const Margin) {
		auto const within = [&](chunk_key const &Centre) {
			auto const dx{Key.first - Centre.first}, dz{Key.second - Centre.second};
			return dx * dx + dz * dz <= (radius + Margin) * (radius + Margin);
		};
		return false == within(here) && false == within(there);
	};
	for (auto it = m_chunks.begin(); it != m_chunks.end();)
	{
		auto &chunk{*it->second};
		if (outside(it->first, 2))
		{
			if (chunk.m_section != nullptr || chunk.m_slot >= 0)
				undraw(chunk);
			if (false == chunk.modified() && false == chunk.m_building)
			{
				it = m_chunks.erase(it);
				continue;
			}
		}
		++it;
	}

	// meshes of the grid each chunk's distance asks for, the nearest first
	std::size_t meshing{0};
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		meshing = m_busy;
		for (auto const &queued : m_jobs)
			if (queued.type == job::kind::mesh)
				++meshing;
	}
	std::vector<std::pair<double, editor_terrain *>> wanted;
	int materials{0};
	for (auto &entry : m_chunks)
	{
		auto &chunk{*entry.second};
		if (outside(entry.first, 1))
			continue;
		if (false == in_region(glm::dvec2{chunk_centre(entry.first).x, chunk_centre(entry.first).z}))
			continue;
		// paint changes go to the material straight away
		if (chunk.m_section != nullptr && chunk.m_paintedversion != chunk.m_paintversion && materials < materials_per_frame)
		{
			update_material(chunk);
			upload_shape_material(chunk);
			++materials;
		}
		if (chunk.m_building)
			continue;
		auto const stride{wanted_stride(chunk, Camera)};
		if (chunk.m_stride != stride || chunk.m_builtversion != chunk.m_shapeversion || chunk.m_neighbours)
		{
			auto const centre{chunk_centre(entry.first)};
			wanted.emplace_back(glm::length(glm::dvec2{centre.x - Camera.x, centre.z - Camera.z}), &chunk);
		}
	}
	std::sort(wanted.begin(), wanted.end(), [](auto const &A, auto const &B) { return A.first < B.first; });
	for (auto const &entry : wanted)
	{
		if (meshing >= max_meshing)
			break;
		request_mesh(*entry.second, wanted_stride(*entry.second, Camera), entry.first / heightmap::chunk_size);
		++meshing;
	}
}

void terrain_streamer::upload_shape_material(editor_terrain &Chunk)
{
	if (Chunk.m_section == nullptr)
		return;
	for (auto &existing : Chunk.m_section->m_shapes)
	{
		auto const &geometry{existing.data().geometry};
		if (geometry.bank == Chunk.m_geometry.bank && geometry.chunk == Chunk.m_geometry.chunk)
		{
			// the shape is made anew with the new material, the rest of it stays
			std::vector<world_vertex> bounds(2);
			bounds[0].position = Chunk.m_low;
			bounds[1].position = Chunk.m_high;
			scene::shape_node shape;
			shape.make_terrain(Chunk.m_material, std::move(bounds), existing.data().origin);
			shape.geometry(Chunk.m_geometry);
			existing = shape;
			return;
		}
	}
}

// queries

void terrain_streamer::collect(std::vector<editor_terrain *> &Out) const
{
	for (auto const &entry : m_chunks)
		Out.push_back(entry.second.get());
}

editor_terrain *terrain_streamer::terrain_at(double X, double Z) const
{
	auto const lookup{m_chunks.find(heightmap::chunk_at(X, Z))};
	if (lookup != m_chunks.end() && lookup->second->contains(X, Z))
		return lookup->second.get();
	return nullptr;
}

bool terrain_streamer::height_at(double X, double Z, double &Height) const
{
	auto const *chunk{terrain_at(X, Z)};
	if (chunk == nullptr)
		return false;
	Height = chunk->height_at(X, Z);
	return true;
}

void terrain_streamer::gather_triangles(glm::dvec2 const &Min, glm::dvec2 const &Max, std::vector<std::array<glm::dvec3, 3>> &Out) const
{
	auto const first{heightmap::chunk_at(Min.x, Min.y)};
	auto const last{heightmap::chunk_at(Max.x, Max.y)};
	for (int cz = first.second; cz <= last.second; ++cz)
		for (int cx = first.first; cx <= last.first; ++cx)
		{
			auto const lookup{m_chunks.find({cx, cz})};
			if (lookup == m_chunks.end())
				continue;
			auto const &data{lookup->second->data()};
			auto const low{data.corner()};
			auto const step{data.spacing()};
			// the cells overlapping the rectangle
			auto const xs{index_range(low.x, step, data.cells, Min.x - step, Max.x)};
			auto const zs{index_range(low.y, step, data.cells, Min.y - step, Max.y)};
			auto const point = [&](int const Ix, int const Iz) { return glm::dvec3{low.x + Ix * step, data.height(Ix, Iz), low.y + Iz * step}; };
			for (int iz = zs.first; iz <= zs.second && iz < data.cells; ++iz)
				for (int ix = xs.first; ix <= xs.second && ix < data.cells; ++ix)
				{
					auto const v00{point(ix, iz)}, v10{point(ix + 1, iz)}, v01{point(ix, iz + 1)}, v11{point(ix + 1, iz + 1)};
					Out.push_back({v00, v01, v10});
					Out.push_back({v11, v10, v01});
				}
		}
}

bool terrain_streamer::owns(gfx::geometry_handle const &Geometry) const
{
	if (Geometry.bank == 0 && Geometry.chunk == 0)
		return false;
	if (m_slotofgeometry.count(geometry_key(Geometry)) > 0)
		return true;
	for (auto const &water : m_watershapes)
		if (water.geometry.bank == Geometry.bank && water.geometry.chunk == Geometry.chunk)
			return true;
	return false;
}

bool terrain_streamer::exists(chunk_key const &Key) const
{
	if (m_chunks.count(Key) > 0)
		return true;
	if (m_removed.count(Key) > 0)
		return false;
	auto const pack{m_packs.find(heightmap::pack_of(Key))};
	return pack != m_packs.end() && pack->second.status == pack_state::state::ready && pack->second.file->has(Key);
}

// editing

editor_terrain *terrain_streamer::load_now(chunk_key const &Key)
{
	auto const lookup{m_chunks.find(Key)};
	if (lookup != m_chunks.end())
		return lookup->second.get();
	if (m_removed.count(Key) > 0)
		return nullptr;
	auto const packkey{heightmap::pack_of(Key)};
	auto &pack{m_packs[packkey]};
	if (pack.status != pack_state::state::ready && pack.status != pack_state::state::missing)
	{
		// the index is read here and now; a worker reading it meanwhile is ignored by the generation
		++pack.generation;
		pack.status = pack_state::state::missing;
		auto const path{pack_path(packkey)};
		std::error_code ec;
		if (std::filesystem::exists(path, ec))
		{
			auto file{std::make_shared<heightmap::pack_file>()};
			std::string error;
			std::lock_guard<std::mutex> lock{m_filemutex};
			if (file->open(path, &error))
			{
				pack.file = file;
				pack.status = pack_state::state::ready;
			}
			else
				ErrorLog("Bad file: " + error, logtype::file);
		}
	}
	if (pack.status != pack_state::state::ready || false == pack.file->has(Key))
		return nullptr;
	std::shared_ptr<heightmap::chunk_data> data;
	std::string error;
	{
		std::lock_guard<std::mutex> lock{m_filemutex};
		data = pack.file->read(Key, &error);
	}
	if (data == nullptr)
	{
		ErrorLog("Bad file: " + error, logtype::file);
		return nullptr;
	}
	auto chunk{std::make_unique<editor_terrain>(std::move(data))};
	auto const [low, high]{std::minmax_element(chunk->heights().begin(), chunk->heights().end())};
	auto const corner{chunk->data().corner()};
	chunk->m_low = {corner.x, *low, corner.y};
	chunk->m_high = {corner.x + heightmap::chunk_size, *high, corner.y + heightmap::chunk_size};
	auto *result{chunk.get()};
	m_chunks.emplace(Key, std::move(chunk));
	m_loading.erase(Key);
	touch_neighbours(Key);
	return result;
}

std::vector<editor_terrain *> terrain_streamer::chunks_in(glm::dvec2 const &Min, glm::dvec2 const &Max, bool const Load)
{
	std::vector<editor_terrain *> result;
	auto const first{heightmap::chunk_at(Min.x, Min.y)};
	auto const last{heightmap::chunk_at(Max.x, Max.y)};
	for (int cz = first.second; cz <= last.second; ++cz)
		for (int cx = first.first; cx <= last.first; ++cx)
		{
			auto const lookup{m_chunks.find({cx, cz})};
			if (lookup != m_chunks.end())
				result.push_back(lookup->second.get());
			else if (Load)
			{
				if (auto *chunk{load_now({cx, cz})})
					result.push_back(chunk);
			}
		}
	return result;
}

bool terrain_streamer::add_chunk(int Cx, int Cz, float Spacing, float Height, std::uint16_t const Layer)
{
	return add_chunk(Cx, Cz, Spacing, [Height](double, double) { return Height; }, Layer);
}

bool terrain_streamer::add_chunk(int Cx, int Cz, float Spacing, std::function<float(double X, double Z)> const &Height, std::uint16_t const Layer)
{
	chunk_key const key{Cx, Cz};
	if (false == m_active || load_now(key) != nullptr || m_chunks.count(key) > 0)
		return false;
	if (false == in_region(glm::dvec2{chunk_centre(key).x, chunk_centre(key).z}))
		return false;
	auto data{heightmap::chunk_data::make_flat(key, Spacing, 0.f)};
	data->layers[0] = Layer < m_manifest.layers.size() ? Layer : std::uint16_t{0};
	auto const low{data->corner()};
	auto const step{data->spacing()};
	for (int iz = 0; iz <= data->cells; ++iz)
		for (int ix = 0; ix <= data->cells; ++ix)
			data->heights[static_cast<std::size_t>(iz) * data->side() + ix] = Height(low.x + ix * step, low.y + iz * step);
	return add_chunk(std::move(data));
}

bool terrain_streamer::add_chunk(std::shared_ptr<heightmap::chunk_data> Data)
{
	if (Data == nullptr || false == Data->valid())
		return false;
	auto const key{Data->key};
	if (false == m_active || load_now(key) != nullptr || m_chunks.count(key) > 0)
		return false;
	if (false == in_region(glm::dvec2{chunk_centre(key).x, chunk_centre(key).z}))
		return false;
	for (auto &layer : Data->layers)
		if (layer != heightmap::no_layer && layer >= m_manifest.layers.size())
			layer = 0;
	auto data{std::move(Data)};
	auto const low{data->corner()};
	remember(key, nullptr);
	auto chunk{std::make_unique<editor_terrain>(std::move(data))};
	chunk->m_modified = true;
	auto const [lowest, highest]{std::minmax_element(chunk->heights().begin(), chunk->heights().end())};
	chunk->m_low = {low.x, *lowest, low.y};
	chunk->m_high = {low.x + heightmap::chunk_size, *highest, low.y + heightmap::chunk_size};
	auto &added{*chunk};
	m_chunks.emplace(key, std::move(chunk));
	m_removed.erase(key);
	m_loading.erase(key);
	apply_modifiers(added, low, low + glm::dvec2{heightmap::chunk_size});
	touch_neighbours(key);
	return true;
}

void terrain_streamer::remove_chunk(int Cx, int Cz)
{
	chunk_key const key{Cx, Cz};
	if (recording())
	{
		// the data is needed to bring the chunk back
		if (auto const *chunk{load_now(key)})
			remember(*chunk);
	}
	auto const lookup{m_chunks.find(key)};
	if (lookup != m_chunks.end())
	{
		undraw(*lookup->second);
		m_chunks.erase(lookup);
	}
	m_removed.insert(key);
	m_loading.erase(key);
	touch_neighbours(key);
}

bool terrain_streamer::spacing(int Cx, int Cz, float Spacing, int Paint)
{
	auto *chunk{load_now({Cx, Cz})};
	if (chunk == nullptr)
		return false;
	auto const &current{chunk->data()};
	auto const repaint{Paint > 0 && current.paint > 0 && Paint != current.paint};
	if (heightmap::cells_for(Spacing) == current.cells && false == repaint)
		return false;
	remember(*chunk);
	chunk->replace(current.resampled(Spacing, Paint), true, repaint);
	auto const low{chunk->data().corner()};
	apply_modifiers(*chunk, low, low + glm::dvec2{heightmap::chunk_size});
	touch_neighbours(chunk->key());
	return true;
}

bool terrain_streamer::reshape(glm::dvec2 const &Min, glm::dvec2 const &Max, std::function<bool(double X, double Z, float &Height)> const &Shaper)
{
	bool changed{false};
	for (auto *chunk : chunks_in(Min, Max, true))
	{
		auto const &current{chunk->data()};
		auto const low{current.corner()};
		auto const step{current.spacing()};
		auto const xs{index_range(low.x, step, current.side(), Min.x, Max.x)};
		auto const zs{index_range(low.y, step, current.side(), Min.y, Max.y)};
		if (xs.first > xs.second || zs.first > zs.second)
			continue;
		if (current.base.empty() || false == m_touchup)
		{
			// a copy is shaped, and taken only if anything changed
			auto heights{current.base.empty() ? current.heights : current.base};
			bool shaped{false};
			for (int iz = zs.first; iz <= zs.second; ++iz)
				for (int ix = xs.first; ix <= xs.second; ++ix)
					shaped |= Shaper(low.x + ix * step, low.y + iz * step, heights[static_cast<std::size_t>(iz) * current.side() + ix]);
			if (false == shaped)
				continue;
			remember(*chunk);
			auto &data{chunk->edit(true, false)};
			(data.base.empty() ? data.heights : data.base) = std::move(heights);
			if (false == data.base.empty())
				apply_modifiers(*chunk, Min, Max);
			changed = true;
			continue;
		}
		// the terrain as it's seen is shaped. where a modifier shapes the ground (or a touch-up is there already) the change
		// is a touch-up over it, elsewhere it's the ground under the modifiers, which may then reach further
		auto heights{current.heights};
		auto base{current.base};
		auto adjust{current.adjust.empty() ? std::vector<float>(heights.size(), 0.f) : current.adjust};
		bool shaped{false}, touched{false};
		for (int iz = zs.first; iz <= zs.second; ++iz)
			for (int ix = xs.first; ix <= xs.second; ++ix)
			{
				auto const index{static_cast<std::size_t>(iz) * current.side() + ix};
				auto height{heights[index]};
				if (false == Shaper(low.x + ix * step, low.y + iz * step, height))
					continue;
				shaped = true;
				auto const change{height - heights[index]};
				auto const shapedbymodifier{std::abs(heights[index] - adjust[index] - base[index]) > 0.0005f};
				if (shapedbymodifier || adjust[index] != 0.f)
				{
					adjust[index] += change;
					touched = true;
				}
				else
					base[index] += change;
				heights[index] = height;
			}
		if (false == shaped)
			continue;
		remember(*chunk);
		auto &data{chunk->edit(true, false)};
		data.base = std::move(base);
		if (touched)
			data.adjust = std::move(adjust);
		data.heights = std::move(heights);
		apply_modifiers(*chunk, Min, Max);
		changed = true;
	}
	return changed;
}

bool terrain_streamer::restore_stitching(double X, double Z, double Radius, double Amount)
{
	if (Radius <= 0.0 || Amount <= 0.0)
		return false;
	glm::dvec2 const low{X - Radius, Z - Radius}, high{X + Radius, Z + Radius};
	bool changed{false};
	for (auto *chunk : chunks_in(low, high, false))
	{
		auto const &current{chunk->data()};
		if (current.adjust.empty())
			continue;
		auto const corner{current.corner()};
		auto const step{current.spacing()};
		auto const xs{index_range(corner.x, step, current.side(), low.x, high.x)};
		auto const zs{index_range(corner.y, step, current.side(), low.y, high.y)};
		auto adjust{current.adjust};
		bool restored{false};
		for (int iz = zs.first; iz <= zs.second; ++iz)
			for (int ix = xs.first; ix <= xs.second; ++ix)
			{
				auto const vx{corner.x + ix * step}, vz{corner.y + iz * step};
				auto const distance{std::sqrt((vx - X) * (vx - X) + (vz - Z) * (vz - Z))};
				auto &value{adjust[static_cast<std::size_t>(iz) * current.side() + ix]};
				if (distance > Radius || value == 0.f)
					continue;
				value *= static_cast<float>(1.0 - std::min(1.0, Amount * falloff(distance, Radius)));
				// what's below the step of the stored heights is gone
				if (std::abs(value) < heightmap::height_step * 0.5)
					value = 0.f;
				restored = true;
			}
		if (false == restored)
			continue;
		remember(*chunk);
		auto &data{chunk->edit(true, false)};
		if (std::all_of(adjust.begin(), adjust.end(), [](float const Value) { return Value == 0.f; }))
			data.adjust.clear();
		else
			data.adjust = std::move(adjust);
		apply_modifiers(*chunk, low, high);
		changed = true;
	}
	return changed;
}

std::size_t terrain_streamer::touched_up() const
{
	std::size_t count{0};
	for (auto const &entry : m_chunks)
	{
		auto const &adjust{entry.second->data().adjust};
		count += static_cast<std::size_t>(std::count_if(adjust.begin(), adjust.end(), [](float const Value) { return Value != 0.f; }));
	}
	return count;
}

bool terrain_streamer::sculpt(double X, double Z, double Radius, double Strength)
{
	if (Radius <= 0.0)
		return false;
	return reshape(glm::dvec2{X - Radius, Z - Radius}, glm::dvec2{X + Radius, Z + Radius}, [&](double const Vx, double const Vz, float &Height) {
		auto const distance{std::sqrt((Vx - X) * (Vx - X) + (Vz - Z) * (Vz - Z))};
		if (distance > Radius)
			return false;
		Height += static_cast<float>(Strength * falloff(distance, Radius));
		return true;
	});
}

bool terrain_streamer::smooth(double X, double Z, double Radius, double Amount)
{
	if (Radius <= 0.0 || Amount <= 0.0)
		return false;
	glm::dvec2 const low{X - Radius, Z - Radius}, high{X + Radius, Z + Radius};
	// every chunk the brush touches is smoothed from the same ground, so the edges they share stay together
	std::vector<std::pair<editor_terrain const *, std::vector<float>>> before;
	for (auto *chunk : chunks_in(low - glm::dvec2{6.0}, high + glm::dvec2{6.0}, false))
		before.emplace_back(chunk, (chunk->data().base.empty() || m_touchup) ? chunk->heights() : chunk->data().base);
	auto const ground = [&](double const Gx, double const Gz, double const Fallback) {
		for (auto const &entry : before)
			if (entry.first->contains(Gx, Gz))
				return entry.first->height_in(entry.second, Gx, Gz);
		return Fallback;
	};
	auto const finest{std::accumulate(before.begin(), before.end(), heightmap::chunk_size, [](double Value, auto const &Entry) { return std::min(Value, Entry.first->data().spacing()); })};
	return reshape(low, high, [&](double const Vx, double const Vz, float &Height) {
		auto const distance{std::sqrt((Vx - X) * (Vx - X) + (Vz - Z) * (Vz - Z))};
		if (distance > Radius)
			return false;
		auto const own{static_cast<double>(Height)};
		auto const step{finest};
		// the neighbours a cell away, read from the ground as it was before the stroke, whichever chunk holds them
		auto const average{(ground(Vx - step, Vz, own) + ground(Vx + step, Vz, own) + ground(Vx, Vz - step, own) + ground(Vx, Vz + step, own) + own * 4.0) / 8.0};
		auto const smoothed{own + (average - own) * std::min(1.0, Amount * falloff(distance, Radius))};
		if (std::abs(smoothed - own) < 1e-5)
			return false;
		Height = static_cast<float>(smoothed);
		return true;
	});
}

bool terrain_streamer::level(double X, double Z, double Radius, double Target, double Amount, int const Mode)
{
	if (Radius <= 0.0 || Amount <= 0.0)
		return false;
	return reshape(glm::dvec2{X - Radius, Z - Radius}, glm::dvec2{X + Radius, Z + Radius}, [&](double const Vx, double const Vz, float &Height) {
		auto const distance{std::sqrt((Vx - X) * (Vx - X) + (Vz - Z) * (Vz - Z))};
		if (distance > Radius)
			return false;
		auto const own{static_cast<double>(Height)};
		if ((Mode == 1 && own >= Target) || (Mode == 2 && own <= Target))
			return false;
		// the middle of the brush gets there, the rim follows less; what's within the step of the stored heights is there already
		auto const levelled{own + (Target - own) * std::min(1.0, Amount * falloff(distance, Radius))};
		auto const result{std::abs(Target - levelled) < heightmap::height_step ? Target : levelled};
		if (std::abs(result - own) < 1e-5)
			return false;
		Height = static_cast<float>(result);
		return true;
	});
}

bool terrain_streamer::paint(double X, double Z, double Radius, double Strength, std::uint16_t Layer)
{
	if (Radius <= 0.0 || Strength <= 0.0)
		return false;
	bool changed{false};
	for (auto *chunk : chunks_in(glm::dvec2{X - Radius, Z - Radius}, glm::dvec2{X + Radius, Z + Radius}, false))
	{
		auto const &current{chunk->data()};
		auto const samples{current.paint > 0 ? current.paint : heightmap::default_paint_samples};
		auto const step{heightmap::chunk_size / samples};
		auto const low{current.corner()};
		auto const xs{index_range(low.x, step, samples + 1, X - Radius, X + Radius)};
		auto const zs{index_range(low.y, step, samples + 1, Z - Radius, Z + Radius)};
		if (xs.first > xs.second || zs.first > zs.second)
			continue;
		if (current.paint <= 0 && current.layers[0] == Layer)
			continue; // the chunk is all of this material already
		auto data{std::make_shared<heightmap::chunk_data>(current)};
		data->ensure_paint(samples);
		auto slot{data->slot_of(Layer)};
		if (slot < 0)
		{
			data->compact_layers();
			data->ensure_paint(samples);
			auto const count{data->layer_count()};
			if (count >= heightmap::max_chunk_layers)
			{
				WriteLog("Editor: chunk " + std::to_string(current.key.first) + "," + std::to_string(current.key.second) + " has " + std::to_string(count) + " materials already, no room for another",
				         logtype::generic);
				continue;
			}
			slot = static_cast<int>(count);
			data->layers[slot] = Layer;
			data->weights.resize(data->weights.size() + static_cast<std::size_t>(samples + 1) * (samples + 1), 0);
		}
		auto const plane{static_cast<std::size_t>(samples + 1) * (samples + 1)};
		auto const count{data->layer_count()};
		bool painted{false};
		for (int iz = zs.first; iz <= zs.second; ++iz)
			for (int ix = xs.first; ix <= xs.second; ++ix)
			{
				auto const px{low.x + ix * step}, pz{low.y + iz * step};
				auto const distance{std::sqrt((px - X) * (px - X) + (pz - Z) * (pz - Z))};
				if (distance > Radius)
					continue;
				auto const sample{static_cast<std::size_t>(iz) * (samples + 1) + ix};
				auto &own{data->weights[slot * plane + sample]};
				auto const amount{std::clamp(Strength * falloff(distance, Radius), 0.0, 1.0)};
				auto const target{static_cast<int>(std::lround(own + (255 - own) * amount))};
				if (target == own)
					continue;
				// the other layers give way in proportion to what they have, the weights still adding up to 255
				auto const others{255 - static_cast<int>(own)};
				auto const left{255 - target};
				int given{0};
				for (std::size_t other = 0; other < count; ++other)
				{
					if (static_cast<int>(other) == slot)
						continue;
					auto &weight{data->weights[other * plane + sample]};
					auto const scaled{others > 0 ? static_cast<int>(weight) * left / others : 0};
					weight = static_cast<std::uint8_t>(scaled);
					given += scaled;
				}
				own = static_cast<std::uint8_t>(255 - given);
				painted = true;
			}
		if (false == painted)
			continue;
		data->compact_layers();
		remember(*chunk);
		chunk->replace(std::move(data), false, true);
		changed = true;
	}
	return changed;
}

std::uint16_t terrain_streamer::layer(std::string const &Material, float const Size)
{
	for (std::size_t i = 0; i < m_manifest.layers.size(); ++i)
		if (m_manifest.layers[i].material == Material && (Size <= 0.f || m_manifest.layers[i].size == Size))
			return static_cast<std::uint16_t>(i);
	remember_manifest();
	m_manifest.layers.push_back({Material, std::max(0.f, Size)});
	m_manifestchanged = true;
	return static_cast<std::uint16_t>(m_manifest.layers.size() - 1);
}

void terrain_streamer::layer_size(std::uint16_t const Layer, float const Size)
{
	if (Layer >= m_manifest.layers.size() || m_manifest.layers[Layer].size == Size)
		return;
	remember_manifest();
	m_manifest.layers[Layer].size = std::max(0.f, Size);
	m_manifestchanged = true;
	// the materials of the chunks using it are made again
	for (auto &entry : m_chunks)
		if (entry.second->data().slot_of(Layer) >= 0)
			entry.second->m_paintedversion = 0;
}

void terrain_streamer::layer_material(std::uint16_t const Layer, std::string const &Material)
{
	if (Layer >= m_manifest.layers.size() || Material.empty() || m_manifest.layers[Layer].material == Material)
		return;
	remember_manifest();
	m_manifest.layers[Layer].material = Material;
	m_manifestchanged = true;
	m_palette.erase(Layer);
	for (auto &entry : m_chunks)
		if (entry.second->data().slot_of(Layer) >= 0 || (Layer == 0 && entry.second->data().layer_count() == 0))
			entry.second->m_paintedversion = 0;
}

void terrain_streamer::compress(bool const State)
{
	if (m_manifest.compress == State)
		return;
	m_manifest.compress = State;
	m_manifestchanged = true;
}

void terrain_streamer::default_spacing(float const Spacing)
{
	auto const spacing{heightmap::valid_spacing(Spacing)};
	if (m_manifest.spacing == spacing)
		return;
	m_manifest.spacing = spacing;
	m_manifestchanged = true;
}

void terrain_streamer::water(std::vector<heightmap::water_body> Water)
{
	remember_manifest();
	m_manifest.water = std::move(Water);
	m_manifestchanged = true;
	m_waterchanged = true;
}

void terrain_streamer::water_material(std::string const &Material)
{
	if (Material.empty() || Material == m_manifest.water_material)
		return;
	remember_manifest();
	m_manifest.water_material = Material;
	m_manifestchanged = true;
	m_waterchanged = true;
}

void terrain_streamer::apply_modifiers(editor_terrain &Chunk, glm::dvec2 const &Min, glm::dvec2 const &Max)
{
	auto const &current{Chunk.data()};
	auto const low{current.corner()};
	auto const high{low + glm::dvec2{heightmap::chunk_size}};
	std::vector<heightmap::modifier_shape> shapes;
	shapes.reserve(m_manifest.modifiers.size());
	bool reachesany{false};
	for (auto const &modifier : m_manifest.modifiers)
	{
		heightmap::modifier_shape shape{modifier};
		if (false == shape.reaches(low, high))
			continue;
		reachesany = true;
		if (shape.reaches(Min, Max))
			shapes.push_back(std::move(shape));
	}
	if (false == reachesany)
	{
		// no modifier reaches the chunk (any more): its own heights are what's left, with the touch-ups made over the modifiers
		if (false == current.base.empty())
		{
			remember(Chunk);
			auto &data{Chunk.edit(true, false)};
			data.heights = std::move(data.base);
			if (data.adjust.size() == data.heights.size())
				for (std::size_t index = 0; index < data.heights.size(); ++index)
					data.heights[index] += data.adjust[index];
			data.base.clear();
			data.adjust.clear();
		}
		return;
	}
	remember(Chunk);
	auto &data{Chunk.edit(true, false)};
	if (data.base.empty())
	{
		data.base = data.heights;
		data.adjust.clear();
	}
	auto const touchups{data.adjust.size() == data.heights.size()};
	auto const step{data.spacing()};
	auto const xs{index_range(low.x, step, data.side(), std::max(Min.x, low.x), std::min(Max.x, high.x))};
	auto const zs{index_range(low.y, step, data.side(), std::max(Min.y, low.y), std::min(Max.y, high.y))};
	for (int iz = zs.first; iz <= zs.second; ++iz)
		for (int ix = xs.first; ix <= xs.second; ++ix)
		{
			auto const index{static_cast<std::size_t>(iz) * data.side() + ix};
			auto height{data.base[index]};
			auto const x{low.x + ix * step}, z{low.y + iz * step};
			for (auto const &shape : shapes)
				shape.apply(x, z, height);
			data.heights[index] = touchups ? height + data.adjust[index] : height;
		}
}

void terrain_streamer::apply_modifiers(glm::dvec2 const &Min, glm::dvec2 const &Max)
{
	for (auto *chunk : chunks_in(Min, Max, true))
		apply_modifiers(*chunk, Min, Max);
}

void terrain_streamer::modifier(heightmap::modifier Modifier)
{
	remember_manifest();
	auto bounds{Modifier.bounds()};
	auto const existing{std::find_if(m_manifest.modifiers.begin(), m_manifest.modifiers.end(), [&](heightmap::modifier const &Item) { return Item.name == Modifier.name; })};
	if (existing != m_manifest.modifiers.end())
	{
		// the ground it changed before is led anew as well
		auto const old{existing->bounds()};
		bounds.first = glm::min(bounds.first, old.first);
		bounds.second = glm::max(bounds.second, old.second);
		*existing = std::move(Modifier);
	}
	else
		m_manifest.modifiers.push_back(std::move(Modifier));
	m_manifestchanged = true;
	apply_modifiers(bounds.first, bounds.second);
}

bool terrain_streamer::remove_modifier(std::string const &Name)
{
	auto const existing{std::find_if(m_manifest.modifiers.begin(), m_manifest.modifiers.end(), [&](heightmap::modifier const &Item) { return Item.name == Name; })};
	if (existing == m_manifest.modifiers.end())
		return false;
	remember_manifest();
	auto const bounds{existing->bounds()};
	m_manifest.modifiers.erase(existing);
	m_manifestchanged = true;
	apply_modifiers(bounds.first, bounds.second);
	return true;
}

bool terrain_streamer::bake_modifier(std::string const &Name)
{
	auto const existing{std::find_if(m_manifest.modifiers.begin(), m_manifest.modifiers.end(), [&](heightmap::modifier const &Item) { return Item.name == Name; })};
	if (existing == m_manifest.modifiers.end())
		return false;
	remember_manifest();
	auto const baked{*existing};
	auto const bounds{baked.bounds()};
	heightmap::modifier_shape const shape{baked};
	// the ground under the modifiers takes the shape this one gives it; the other modifiers and the touch-ups go on top
	for (auto *chunk : chunks_in(bounds.first, bounds.second, true))
	{
		auto const &current{chunk->data()};
		if (current.base.empty())
			continue;
		auto const low{current.corner()};
		auto const step{current.spacing()};
		auto const xs{index_range(low.x, step, current.side(), bounds.first.x, bounds.second.x)};
		auto const zs{index_range(low.y, step, current.side(), bounds.first.y, bounds.second.y)};
		auto base{current.base};
		bool shaped{false};
		for (int iz = zs.first; iz <= zs.second; ++iz)
			for (int ix = xs.first; ix <= xs.second; ++ix)
				shaped |= shape.apply(low.x + ix * step, low.y + iz * step, base[static_cast<std::size_t>(iz) * current.side() + ix]);
		if (false == shaped)
			continue;
		remember(*chunk);
		chunk->edit(true, false).base = std::move(base);
	}
	m_manifest.modifiers.erase(std::find_if(m_manifest.modifiers.begin(), m_manifest.modifiers.end(), [&](heightmap::modifier const &Item) { return Item.name == Name; }));
	m_manifestchanged = true;
	apply_modifiers(bounds.first, bounds.second);
	return true;
}

bool terrain_streamer::modified() const
{
	if (m_manifestchanged || false == m_removed.empty())
		return true;
	return std::any_of(m_chunks.begin(), m_chunks.end(), [](auto const &Entry) { return Entry.second->modified(); });
}

// undo

void terrain_streamer::begin_change()
{
	if (m_recording++ == 0)
		m_change = change();
}

terrain_streamer::change terrain_streamer::end_change()
{
	if (m_recording == 0)
		return change();
	if (--m_recording > 0)
		return change(); // the outer change goes on
	change result{std::move(m_change)};
	m_change = change();
	return result;
}

void terrain_streamer::remember(chunk_key const &Key, heightmap::chunk_ptr const &Data)
{
	if (m_recording > 0)
		m_change.chunks.emplace(Key, Data); // only the first state counts
}

void terrain_streamer::remember_manifest()
{
	if (m_recording == 0 || m_change.manifest)
		return;
	m_change.manifest = true;
	m_change.layers = m_manifest.layers;
	m_change.water = m_manifest.water;
	m_change.water_material = m_manifest.water_material;
	m_change.modifiers = m_manifest.modifiers;
}

terrain_streamer::change terrain_streamer::restore(change const &Change)
{
	change opposite;
	if (false == m_active)
		return opposite;
	for (auto const &[key, data] : Change.chunks)
	{
		auto *current{load_now(key)};
		opposite.chunks.emplace(key, current != nullptr ? current->shared() : nullptr);
		if (data == nullptr)
		{
			if (current != nullptr)
			{
				undraw(*current);
				m_chunks.erase(key);
				m_removed.insert(key);
				touch_neighbours(key);
			}
			continue;
		}
		if (current != nullptr)
		{
			current->replace(data, true, true);
		}
		else
		{
			auto chunk{std::make_unique<editor_terrain>(data)};
			chunk->m_modified = true;
			auto const [low, high]{std::minmax_element(chunk->heights().begin(), chunk->heights().end())};
			auto const corner{data->corner()};
			chunk->m_low = {corner.x, *low, corner.y};
			chunk->m_high = {corner.x + heightmap::chunk_size, *high, corner.y + heightmap::chunk_size};
			m_chunks.emplace(key, std::move(chunk));
			m_removed.erase(key);
			m_loading.erase(key);
		}
		touch_neighbours(key);
	}
	if (Change.manifest)
	{
		opposite.manifest = true;
		opposite.layers = m_manifest.layers;
		opposite.water = m_manifest.water;
		opposite.water_material = m_manifest.water_material;
		opposite.modifiers = m_manifest.modifiers;
		auto const samelayers{std::equal(Change.layers.begin(), Change.layers.end(), m_manifest.layers.begin(), m_manifest.layers.end(),
		                                 [](heightmap::layer_def const &A, heightmap::layer_def const &B) { return A.material == B.material && A.size == B.size; })};
		m_manifest.layers = Change.layers;
		m_manifest.modifiers = Change.modifiers;
		m_manifest.water = Change.water;
		if (false == Change.water_material.empty())
			m_manifest.water_material = Change.water_material;
		m_manifestchanged = true;
		m_waterchanged = true;
		if (false == samelayers)
		{
			// the materials of the chunks are made anew from the palette as it was
			m_palette.clear();
			for (auto &entry : m_chunks)
				entry.second->m_paintedversion = 0;
		}
	}
	return opposite;
}

bool terrain_streamer::save(std::string *Error)
{
	if (false == m_active)
		return true;
	auto const fail = [&](std::string const &Message) {
		if (Error != nullptr)
			*Error = Message;
		ErrorLog("Heightmap terrain: " + Message);
		return false;
	};
	std::error_code ec;
	auto const manifestpath{m_directory + "terrain.yaml"};
	if (m_manifestchanged || modified() || false == std::filesystem::exists(manifestpath, ec))
	{
		std::string error;
		if (false == heightmap::save_manifest(manifestpath, m_manifest, &error))
			return fail(error);
		m_manifestchanged = false;
	}

	// the packs holding changed chunks are written whole
	std::set<heightmap::pack_key> packs;
	for (auto const &entry : m_chunks)
		if (entry.second->modified())
			packs.insert(heightmap::pack_of(entry.first));
	for (auto const &key : m_removed)
		packs.insert(heightmap::pack_of(key));

	for (auto const &packkey : packs)
	{
		auto const path{pack_path(packkey)};
		std::vector<heightmap::chunk_ptr> chunks;
		std::set<chunk_key> taken;
		// what's in the file now, unless it's in memory or removed
		std::shared_ptr<heightmap::pack_file> file;
		if (std::filesystem::exists(path, ec))
		{
			file = std::make_shared<heightmap::pack_file>();
			std::string error;
			std::lock_guard<std::mutex> lock{m_filemutex};
			if (false == file->open(path, &error))
				return fail(error);
			for (auto const &key : file->chunks())
			{
				if (m_removed.count(key) > 0 || m_chunks.count(key) > 0)
					continue;
				auto data{file->read(key, &error)};
				if (data == nullptr)
					return fail(error);
				chunks.push_back(std::move(data));
				taken.insert(key);
			}
		}
		for (auto const &entry : m_chunks)
			if (heightmap::pack_of(entry.first) == packkey && taken.count(entry.first) == 0)
				chunks.push_back(entry.second->shared());
		{
			std::lock_guard<std::mutex> lock{m_filemutex};
			if (chunks.empty())
				std::filesystem::remove(path, ec);
			else
			{
				std::string error;
				if (false == heightmap::write_pack(path, packkey, chunks, m_manifest.compress, &error))
					return fail(error);
			}
		}
		// the file is known anew; what the workers read from the old one is ignored
		auto &pack{m_packs[packkey]};
		++pack.generation;
		pack.file.reset();
		pack.status = pack_state::state::missing;
		if (false == chunks.empty())
		{
			auto reopened{std::make_shared<heightmap::pack_file>()};
			std::string error;
			std::lock_guard<std::mutex> lock{m_filemutex};
			if (reopened->open(path, &error))
			{
				pack.file = reopened;
				pack.status = pack_state::state::ready;
			}
		}
		for (auto it = m_removed.begin(); it != m_removed.end();)
			it = (heightmap::pack_of(*it) == packkey ? m_removed.erase(it) : std::next(it));
		for (auto const &entry : m_chunks)
			if (heightmap::pack_of(entry.first) == packkey)
				entry.second->clear_modified();
	}
	WriteLog("Heightmap terrain saved: " + m_directory + " (" + std::to_string(packs.size()) + " pack files written)", logtype::generic);
	return true;
}

// water

void terrain_streamer::clear_water(bool const Touch)
{
	if (Touch && simulation::Region != nullptr)
		for (auto const &water : m_watershapes)
		{
			if (water.cell != nullptr)
				erase_shape(water.cell->m_shapestranslucent, water.geometry);
			else if (water.section != nullptr)
				erase_shape(water.section->m_shapes, water.geometry);
		}
	// the geometry stays with the renderer, to be filled anew
	for (auto const &water : m_watershapes)
		m_waterfree.push_back(water.geometry);
	m_watershapes.clear();
}

void terrain_streamer::build_water()
{
	m_waterchanged = false;
	clear_water(true);
	if (simulation::Region == nullptr)
		return;
	for (auto const &body : m_manifest.water)
	{
		if (body.outline.size() < 3)
			continue;
		auto const indices{heightmap::triangulate(body.outline)};
		if (indices.empty())
			continue;
		glm::dvec2 centroid{0.0};
		for (auto const &point : body.outline)
			centroid += point;
		centroid /= static_cast<double>(body.outline.size());
		if (false == in_region(centroid))
			continue;
		auto const material{GfxRenderer->Fetch_Material(body.material.empty() ? m_manifest.water_material : body.material)};
		auto const *materialdata{GfxRenderer->Material(material)};
		auto const repeat{body.size > 0.f ? static_cast<double>(body.size) : (materialdata != nullptr && materialdata->GetSize().x > 0.f) ? static_cast<double>(materialdata->GetSize().x) : 8.0};
		glm::dvec3 const centre{centroid.x, body.level, centroid.y};
		auto &section{simulation::Region->section(centre)};
		// the section makes the geometry of its own shapes once, before it's drawn; done first, it leaves ours alone
		section.create_geometry();
		// translucent water goes with the translucent geometry of the cell around its middle, drawn relative to the cell
		auto const translucent{materialdata != nullptr && materialdata->is_translucent()};
		auto &cell{section.cell(centre)};
		auto const origin{translucent ? cell.m_area.center : section.m_area.center};
		gfx::vertex_array vertices;
		vertices.reserve(indices.size());
		double reach{0.0};
		// the outline is turned counterclockwise in the plan, the triangles go the other way round to face up
		for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
			for (auto const index : {indices[i + 2], indices[i + 1], indices[i]})
			{
				auto const &point{body.outline[index]};
				gfx::basic_vertex vertex;
				vertex.position = glm::vec3(glm::dvec3{point.x, body.level, point.y} - origin);
				vertex.normal = {0.f, 1.f, 0.f};
				vertex.texture = {static_cast<float>((point.x - centroid.x) / repeat), static_cast<float>((centroid.y - point.y) / repeat)};
				vertex.tangent = {1.f, 0.f, 0.f, 1.f};
				vertices.push_back(vertex);
				reach = std::max(reach, glm::length(point - centroid));
			}
		// the geometry of water made before is filled anew, so editing the water doesn't pile up geometry
		gfx::userdata_array nouserdata;
		water_shape entry;
		if (false == m_waterfree.empty())
		{
			entry.geometry = m_waterfree.back();
			m_waterfree.pop_back();
			GfxRenderer->Replace(vertices, nouserdata, entry.geometry, GL_TRIANGLES);
		}
		else
		{
			if (m_waterbank.bank == 0 && m_waterbank.chunk == 0)
				m_waterbank = GfxRenderer->Create_Bank();
			entry.geometry = GfxRenderer->Insert(vertices, nouserdata, m_waterbank, GL_TRIANGLES);
		}
		if (entry.geometry.chunk == 0)
			continue;
		// two bounding vertices give the shape its area, the triangles are in the geometry
		std::vector<world_vertex> bounds(2);
		bounds[0].position = glm::dvec3{centroid.x - reach, body.level, centroid.y - reach};
		bounds[1].position = glm::dvec3{centroid.x + reach, body.level, centroid.y + reach};
		scene::shape_node shape;
		shape.make_terrain(material, std::move(bounds), origin);
		shape.geometry(entry.geometry);
		if (translucent)
		{
			cell.m_shapestranslucent.emplace_back(std::move(shape));
			cell.m_area.radius = std::max(cell.m_area.radius, static_cast<float>(glm::length(cell.m_area.center - centre) + reach));
			cell.m_active = true;
			entry.cell = &cell;
		}
		else
		{
			section.m_shapes.emplace_back(std::move(shape));
		}
		section.m_area.radius = std::max(section.m_area.radius, static_cast<float>(glm::length(section.m_area.center - centre) + reach));
		entry.section = &section;
		m_watershapes.push_back(entry);
	}
}
