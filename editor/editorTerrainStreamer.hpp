/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <glm/glm.hpp>

#include "editor/editorTerrain.hpp"

struct world_vertex;
namespace scene { class basic_section; class basic_cell; }

// the heightmap terrain of the scenery (scene/heightmapterrain.h), loaded around the camera.
//
// the pack files are read and the meshes made by worker threads; the main thread only puts finished meshes on the gpu,
// within a few milliseconds of each frame. chunks are loaded around the camera and around the place the camera is heading
// to, so a moving train finds its terrain loaded. each chunk is drawn with a grid as coarse as its distance allows.
//
// the editor changes the chunks in memory; they're written to the pack files when the scenery is saved, until then the
// edited chunks stay in memory, also when they're far from the camera
class terrain_streamer
{
  public:
	using chunk_key = heightmap::chunk_key;

	terrain_streamer() = default;
	~terrain_streamer();

	// opens the terrain kept in terrain/<Name>/ (the folder may not exist yet). returns: true if there's terrain to show
	bool open(std::string const &Name);
	// stops showing the terrain
	void close();
	bool active() const { return m_active; }
	std::string const &name() const { return m_name; }
	// the folder of the terrain, with a trailing slash
	std::string const &directory() const { return m_directory; }

	// loads and unloads the chunks around the camera, and puts finished meshes on the gpu
	void update(glm::dvec3 const &Camera);
	// hard reset used when a new scenery loads: drops everything WITHOUT touching scene sections
	// (the old region is being torn down, so its section pointers are already dangling)
	void reset();

	// drawn detail: metres of distance per metre of grid spacing (larger: finer grid further away)
	void detail(float const Detail) { m_detail = std::max(10.f, Detail); }
	float detail() const { return m_detail; }
	// smallest distance (chunks) the terrain is loaded to, whatever the draw range
	void radius(int const Radius) { m_radius = std::max(0, Radius); }
	int radius() const { return m_radius; }

	// chunks held in memory
	void collect(std::vector<editor_terrain *> &Out) const;
	// chunk covering (X,Z), or nullptr
	editor_terrain *terrain_at(double X, double Z) const;
	// height of the terrain at (X,Z); false where there's no chunk in memory
	bool height_at(double X, double Z, double &Height) const;
	// triangles of the full grid of the chunks in memory overlapping the rectangle (x, z)
	void gather_triangles(glm::dvec2 const &Min, glm::dvec2 const &Max, std::vector<std::array<glm::dvec3, 3>> &Out) const;
	// true if the geometry belongs to the terrain (or its water)
	bool owns(gfx::geometry_handle const &Geometry) const;
	std::size_t resident() const { return m_chunks.size(); }
	// chunks in the pack files and in memory, less the removed ones; the files known so far
	bool exists(chunk_key const &Key) const;

	// editing; the chunks changed are written to the files by save()
	heightmap::manifest const &manifest() const { return m_manifest; }
	// a flat chunk at (Cx, Cz) of specified spacing and height, covered with the material of palette entry Layer;
	// returns false if there's a chunk already
	bool add_chunk(int Cx, int Cz, float Spacing, float Height, std::uint16_t Layer = 0);
	// a chunk at (Cx, Cz), heights given for each grid point by Height (from its world position)
	bool add_chunk(int Cx, int Cz, float Spacing, std::function<float(double X, double Z)> const &Height, std::uint16_t Layer = 0);
	// a chunk made elsewhere, at the place its key says; its layers are indices of the palette
	bool add_chunk(std::shared_ptr<heightmap::chunk_data> Data);
	void remove_chunk(int Cx, int Cz);
	// changes the point spacing of the chunk at (Cx, Cz), heights resampled; Paint: paint samples along a side less one
	// the chunk gets, 0 keeps the ones it has. returns: true if the chunk changed
	bool spacing(int Cx, int Cz, float Spacing, int Paint = 0);
	// raises (Strength > 0) or lowers the ground within Radius of (X,Z), with a smooth falloff
	bool sculpt(double X, double Z, double Radius, double Strength);
	// evens out the ground within Radius of (X,Z), Amount (0..1) of the way at the centre
	bool smooth(double X, double Z, double Radius, double Amount);
	// leads the ground within Radius of (X,Z) towards the height Target, Amount (0..1) of the way at the centre;
	// Mode: 0 up and down, 1 only raises it, 2 only lowers it
	bool level(double X, double Z, double Radius, double Target, double Amount, int Mode = 0);
	// gives every grid point within the rectangle to Shaper, with its world position and height; Shaper returns true if it
	// changed the height. where the modifiers shape the ground the change is a touch-up kept over them (see touch_up()),
	// elsewhere it changes the ground under them
	bool reshape(glm::dvec2 const &Min, glm::dvec2 const &Max, std::function<bool(double X, double Z, float &Height)> const &Shaper);
	// true: the brushes shape the terrain as it's seen, also where the modifiers (tracks) shape it, the changes kept there
	// as touch-ups over the modifiers; false: the brushes shape only the ground under the modifiers, which keep their shape
	void touch_up(bool const State) { m_touchup = State; }
	bool touch_up() const { return m_touchup; }
	// takes the touch-ups within Radius of (X,Z) back, Amount (0..1) of the way at the centre: the terrain goes back to the
	// shape the modifiers give it
	bool restore_stitching(double X, double Z, double Radius, double Amount);
	// grid points with touch-ups over the modifiers, in the chunks in memory
	std::size_t touched_up() const;
	// paints the material of the palette entry Layer within Radius of (X,Z), Strength (0..1) of the way at the centre
	bool paint(double X, double Z, double Radius, double Strength, std::uint16_t Layer);
	// fills the area within Outline (x, z) with the material of the palette entry Layer, Strength (0..1) of the way.
	// Feather: metres across the outline the material blends in over, half of them inside; 0: a sharp edge, as sharp as
	// the paint samples of the chunks go. the chunks are loaded where needed
	bool fill(std::vector<glm::dvec2> const &Outline, double Feather, std::uint16_t Layer, double Strength = 1.0);
	// palette entry of the material (repeated at Size, 0: any, turned by Rotation degrees), added to the palette if needed
	std::uint16_t layer(std::string const &Material, float Size = 0.f, float Rotation = 0.f);
	void layer_size(std::uint16_t Layer, float Size);
	// turns the textures of the palette entry Layer by Rotation degrees, on every chunk painted with it
	void layer_rotation(std::uint16_t Layer, float Rotation);
	// gives the palette entry Layer another material; the chunks painted with it change with it
	void layer_material(std::uint16_t Layer, std::string const &Material);
	void compress(bool State);
	void default_spacing(float Spacing);
	// bodies of water
	std::vector<heightmap::water_body> const &water() const { return m_manifest.water; }
	void water(std::vector<heightmap::water_body> Water);
	// material of the bodies of water which don't name their own
	void water_material(std::string const &Material);
	// modifiers: added or replaced by name
	void modifier(heightmap::modifier Modifier);
	bool remove_modifier(std::string const &Name);
	// the ground takes the shape the modifier gives it for good, and the modifier is removed: from then on the
	// terrain there is shaped by hand like anywhere else
	bool bake_modifier(std::string const &Name);
	std::vector<heightmap::modifier> const &modifiers() const { return m_manifest.modifiers; }
	// changed since the last save
	bool modified() const;

	// undo: the state of what a change touched, from before it
	struct change
	{
		std::map<chunk_key, heightmap::chunk_ptr> chunks; // the data the chunks had; null: there was no chunk
		bool manifest{false}; // the palette, the water (and its material) and the modifiers below were changed
		std::vector<heightmap::layer_def> layers;
		std::vector<heightmap::water_body> water;
		std::string water_material;
		std::vector<heightmap::modifier> modifiers;

		bool empty() const { return chunks.empty() && false == manifest; }
	};
	// from now on the state of what's changed is kept, until end_change(); nested calls join the change in progress
	void begin_change();
	// returns: the state from before the changes made since begin_change() (empty if nothing changed)
	change end_change();
	bool recording() const { return m_recording > 0; }
	// puts the terrain back in the state recorded. returns: the state it replaced, to go forward again
	change restore(change const &Change);

	// writes the description and the changed pack files. returns: true on success, Error describing the failure
	bool save(std::string *Error = nullptr);

  private:
	struct pack_state
	{
		enum class state
		{
			unknown,
			loading,
			ready,
			missing
		} status{state::unknown};
		std::shared_ptr<heightmap::pack_file const> file;
		std::uint32_t generation{0};
	};
	struct job
	{
		enum class kind
		{
			pack,
			chunk,
			mesh
		} type{kind::pack};
		heightmap::pack_key pack{0, 0};
		chunk_key chunk{0, 0};
		std::shared_ptr<heightmap::pack_file const> file;
		std::uint32_t generation{0};
		heightmap::chunk_ptr data;
		std::array<heightmap::chunk_ptr, 4> neighbours; // -x, +x, -z, +z
		int stride{1};
		glm::dvec3 origin{0.0};
		std::uint64_t version{0};
		double priority{0.0};
	};
	struct result
	{
		job::kind type{job::kind::pack};
		heightmap::pack_key pack{0, 0};
		chunk_key chunk{0, 0};
		std::uint32_t generation{0};
		std::shared_ptr<heightmap::pack_file const> file;
		std::shared_ptr<heightmap::chunk_data> data;
		heightmap::mesh mesh;
		int stride{1};
		std::uint64_t version{0};
		std::string error;
	};
	struct geometry_slot
	{
		gfx::geometrybank_handle bank{0, 0};
		gfx::geometry_handle geometry{0, 0};
		int cells{0};
		int stride{0};
		bool used{false};
	};
	struct water_shape
	{
		gfx::geometry_handle geometry{0, 0};
		scene::basic_section *section{nullptr};
		scene::basic_cell *cell{nullptr};
	};

	void start_workers();
	void stop_workers();
	void worker();
	void submit(job Job);
	void process(result &Result);
	// the chunk is drawn: its shape put in the scene, or put back with a new mesh
	void upload(editor_terrain &Chunk, heightmap::mesh &Mesh, int const Stride);
	void update_material(editor_terrain &Chunk);
	// puts the current material of the chunk in its shape
	void upload_shape_material(editor_terrain &Chunk);
	// takes the chunk out of the scene, its geometry slot and material given back
	void undraw(editor_terrain &Chunk);
	void request_mesh(editor_terrain &Chunk, int const Stride, double const Priority);
	int wanted_stride(editor_terrain const &Chunk, glm::dvec3 const &Camera) const;
	heightmap::chunk_ptr neighbour(chunk_key const &Key) const;
	void touch_neighbours(chunk_key const &Key);
	material_handle palette_material(std::uint16_t const Layer);
	// paints the material of the palette entry Layer over the paint samples within the rectangle, Amount (0..1) of the way
	// at each, as Amount gives it for the place of the sample. Load: chunks not in memory are loaded
	bool paint_area(glm::dvec2 const &Min, glm::dvec2 const &Max, std::uint16_t Layer, bool Load, std::function<double(double X, double Z)> const &Amount);
	// chunk loaded right away from its pack file, for editing far from the camera
	editor_terrain *load_now(chunk_key const &Key);
	// chunks in memory or in the files overlapping the rectangle, loaded if needed
	std::vector<editor_terrain *> chunks_in(glm::dvec2 const &Min, glm::dvec2 const &Max, bool const Load);
	// leads the heights of the chunk from its base heights through the modifiers, within the rectangle
	void apply_modifiers(editor_terrain &Chunk, glm::dvec2 const &Min, glm::dvec2 const &Max);
	void apply_modifiers(glm::dvec2 const &Min, glm::dvec2 const &Max);
	void build_water();
	void clear_water(bool const Touch);
	std::string pack_path(heightmap::pack_key const &Key) const;
	// the state of a chunk (null: no chunk) or of the description is kept for the change in progress, if it's the first time
	void remember(chunk_key const &Key, heightmap::chunk_ptr const &Data);
	void remember(editor_terrain const &Chunk) { remember(Chunk.key(), Chunk.shared()); }
	void remember_manifest();

	bool m_active{false};
	std::string m_name;
	std::string m_directory;
	heightmap::manifest m_manifest;
	bool m_manifestchanged{false};
	float m_detail{120.f};
	bool m_touchup{true};
	int m_radius{2};

	std::map<heightmap::pack_key, pack_state> m_packs;
	std::map<chunk_key, std::unique_ptr<editor_terrain>> m_chunks;
	std::set<chunk_key> m_loading; // asked from a worker
	std::set<chunk_key> m_removed; // removed in the editor, still in the files until saved
	std::vector<geometry_slot> m_slots;
	std::map<std::uint64_t, int> m_slotofgeometry; // geometry key -> slot
	std::map<std::uint16_t, material_handle> m_palette;
	std::vector<water_shape> m_watershapes;
	std::vector<gfx::geometry_handle> m_waterfree; // geometry of the water shown before, to be filled anew
	gfx::geometrybank_handle m_waterbank{0, 0};
	bool m_waterchanged{false};

	int m_recording{0};
	change m_change;

	glm::dvec3 m_camera{0.0};
	glm::dvec3 m_velocity{0.0};
	double m_cameratime{0.0};
	bool m_hascamera{false};
	std::vector<chunk_key> m_ring; // offsets of the chunks around a point, the nearest first
	int m_ringradius{-1};

	// shared with the workers, guarded by m_mutex
	std::mutex m_mutex;
	std::condition_variable m_wakeup;
	std::vector<job> m_jobs;
	std::deque<result> m_results;
	std::size_t m_busy{0}; // jobs taken by the workers and not finished yet
	bool m_stop{false};
	std::vector<std::thread> m_workers;
	// the pack files are read by the workers and written by save()
	std::mutex m_filemutex;
};

// single, simulation-level instance shared by the editor (authoring) and the scenery loader, so the terrain
// renders in every mode. opened by the `heightmap_terrain` scenery directive and updated each frame with the active camera.
extern terrain_streamer EditorTerrain;
