/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "rendering/geometrybank.h"

// heightmap terrain: the ground of a scenery as a grid of square chunks of heights, kept in the folder
// terrain/<name>/ of the simulator: a description of the terrain (terrain.yaml) and pack files (.tch, terrain chunks)
// holding 16 x 16 chunks each. the chunks are loaded around the camera as they're needed.
//
// a chunk has its own point spacing (0.5, 1, 2 or 5 m), heights of its grid points including the points of its edges
// (shared with the neighbouring chunk), and weights of the materials painted over it, up to 8 of them per chunk.
namespace heightmap
{

double constexpr chunk_size{250.0}; // metres, side of a chunk; whole cells of every allowed spacing, and of the scene
int constexpr pack_side{16}; // chunks along the side of a pack file
double constexpr height_step{0.005}; // metres, quantum of the stored heights
std::size_t constexpr max_chunk_layers{8}; // materials painted over a single chunk
std::uint16_t constexpr no_layer{0xffff};
int constexpr default_paint_samples{250}; // paint samples along the side of a chunk, less one: a sample per metre

// point spacings a chunk can have, finest first
std::array<float, 4> constexpr spacings{0.5f, 1.f, 2.f, 5.f};
float constexpr default_spacing{2.f};
// cells along the side of a chunk with specified point spacing
int cells_for(float const Spacing);
// the allowed spacing closest to specified one
float valid_spacing(float const Spacing);

using chunk_key = std::pair<int, int>;
using pack_key = std::pair<int, int>;

// chunk covering specified point
chunk_key chunk_at(double const X, double const Z);
// corner of the chunk with the lowest coordinates
glm::dvec2 chunk_corner(chunk_key const &Key);
// pack holding specified chunk
pack_key pack_of(chunk_key const &Key);
// corner of the pack with the lowest coordinates; texture coordinates of the terrain start over at it
glm::dvec2 pack_corner(pack_key const &Key);
// name of the file of specified pack
std::string pack_file_name(pack_key const &Key);

struct chunk_data
{
	chunk_key key{0, 0};
	int cells{0}; // quads along a side; (cells + 1)^2 grid points
	std::vector<float> heights; // world y of the grid points, rows of growing z, from the lowest x
	std::vector<float> base; // the heights before the modifiers were applied; empty where no modifier reaches the chunk
	int paint{0}; // paint samples along a side, less one. 0: the chunk is covered with its first layer alone
	std::array<std::uint16_t, max_chunk_layers> layers; // indices of the materials in the terrain palette, no_layer for unused
	std::vector<std::uint8_t> weights; // planar, (paint + 1)^2 per used layer, rows like the heights; weights of a sample add up to 255

	chunk_data() { layers.fill(no_layer); }

	int side() const { return cells + 1; }
	double spacing() const { return chunk_size / cells; }
	glm::dvec2 corner() const { return chunk_corner(key); }
	bool valid() const { return cells > 0 && heights.size() == static_cast<std::size_t>(side()) * side(); }
	// number of layers in use, the used ones come first
	std::size_t layer_count() const;
	float height(int const Ix, int const Iz) const { return heights[static_cast<std::size_t>(Iz) * side() + Ix]; }
	// height of the surface at specified world point, the way the full grid is triangulated; points outside are clamped to the chunk
	double height_at(double const X, double const Z) const;
	// as above, from provided heights of the grid points of the chunk
	double height_in(std::vector<float> const &Heights, double const X, double const Z) const;
	bool contains(double const X, double const Z) const;
	// a flat chunk of specified spacing at specified height, covered with the first material of the palette
	static std::shared_ptr<chunk_data> make_flat(chunk_key const &Key, float const Spacing, float const Height);
	// the same chunk with the grid of another spacing, heights and paint taken over
	std::shared_ptr<chunk_data> resampled(float const Spacing) const;
	// weight of specified layer slot at paint sample (Ix, Iz)
	std::uint8_t weight(std::size_t const Slot, int const Ix, int const Iz) const;
	// layer slot of specified palette index, or -1
	int slot_of(std::uint16_t const Layer) const;
	// layer slot of the material covering the most of the chunk
	std::size_t dominant_slot() const;
	// gives the chunk paint samples (with its first layer as the only one painted) if it has none yet
	void ensure_paint(int const Samples = default_paint_samples);
	// drops the layers left without weight anywhere, and the paint samples altogether when a single layer is left
	void compact_layers();
};

using chunk_ptr = std::shared_ptr<chunk_data const>;

// body of water: a level surface within an outline
struct water_body
{
	std::string name;
	double level{0.0};
	std::string material; // empty: the default water material
	std::vector<glm::dvec2> outline; // x, z
};

// modifier of the terrain: a corridor along a line (axis of a track or a road) with a formation of specified
// height and width, and slopes from its edges to the ground
struct corridor_point
{
	glm::dvec2 position{0.0}; // x, z
	double formation{0.0}; // height of the formation
	double half_width{3.5}; // from the axis to the edge of the formation
};

struct modifier
{
	std::string name;
	double slope{1.5}; // horizontal run of the slopes per metre of height
	double reach{60.0}; // from the edge of the formation, where the slopes end even if they don't get to the ground
	double rounding{1.5}; // over which the slopes bend into the formation and into the ground
	std::vector<corridor_point> points;

	// rectangle (x, z) the modifier can change the ground in
	std::pair<glm::dvec2, glm::dvec2> bounds() const;
};

// modifier prepared for many questions
class modifier_shape
{
  public:
	explicit modifier_shape(modifier const &Modifier);
	// leads Height at specified point the way the modifier does; returns: true if the height changed
	bool apply(double const X, double const Z, float &Height) const;
	bool reaches(glm::dvec2 const &Min, glm::dvec2 const &Max) const;

  private:
	modifier const &m_modifier;
	double m_widest{0.0};
	glm::dvec2 m_min{0.0}, m_max{0.0};
	double m_cell{16.0};
	std::vector<std::pair<std::uint64_t, std::uint32_t>> m_grid; // (cell key, segment), sorted
};

// material of the palette, and the size (metres) its textures repeat at; 0: the size given by the material, or 8 m
struct layer_def
{
	std::string material;
	float size{0.f};
};

// description of the terrain, terrain.yaml
struct manifest
{
	int version{1};
	std::vector<layer_def> layers{{"grass", 0.f}}; // the palette of materials, the first covers new chunks
	std::string water_material{"water"}; // of the bodies of water which don't name their own
	bool compress{true}; // pack files store their data compressed with zstd
	float spacing{default_spacing}; // point spacing of new chunks
	std::vector<water_body> water;
	std::vector<modifier> modifiers;
};

bool load_manifest(std::string const &Path, manifest &Manifest, std::string *Error = nullptr);
bool save_manifest(std::string const &Path, manifest const &Manifest, std::string *Error = nullptr);

// pack file (.tch), little endian:
//
//   header, 64 bytes: 'TCH1', uint32 version, int32 pack x, int32 pack z, uint32 chunks along a side, float chunk size,
//                     uint64 offset of the chunk index, reserved
//   chunk index: per chunk (rows of growing z): uint64 offset of its slice table (0: no chunk), uint32 slice count, uint32 reserved
//   slice tables: per slice: uint32 tag, uint32 codec (0: stored, 1: zstd), uint64 offset, uint32 stored size, uint32 size,
//                 uint32 crc-32 of the data, uint32 reserved
//   slice data
//
// slices of a chunk:
//   'CHNK' uint16 cells, uint16 paint samples less one, uint16 layers[8]
//   'HGHT' heights: uint8 encoding (0: int32 base and uint16 steps of height_step above it, delta coded in row order;
//          1: float32), then the values
//   'HBAS' heights before the modifiers, as 'HGHT'
//   'SPLT' paint weights: uint8 per sample per used layer, planar
class pack_file
{
  public:
	// reads the header and the index of specified file. returns: true on success
	bool open(std::string const &Path, std::string *Error = nullptr);
	bool has(chunk_key const &Key) const;
	// chunks the file holds
	std::vector<chunk_key> chunks() const;
	// reads specified chunk. returns: the chunk, or null on failure
	std::shared_ptr<chunk_data> read(chunk_key const &Key, std::string *Error = nullptr) const;
	pack_key key() const { return m_key; }

  private:
	struct slice
	{
		std::uint32_t tag{0};
		std::uint32_t codec{0};
		std::uint64_t offset{0};
		std::uint32_t stored{0};
		std::uint32_t size{0};
		std::uint32_t crc{0};
	};
	std::string m_path;
	pack_key m_key{0, 0};
	std::array<std::vector<slice>, pack_side * pack_side> m_slices;
};

// writes specified chunks (all of the same pack) to the pack file. returns: true on success
bool write_pack(std::string const &Path, pack_key const &Key, std::vector<chunk_ptr> const &Chunks, bool const Compress, std::string *Error = nullptr);

// triangles (indices of the points, three to a triangle) covering a simple polygon in the XZ plane
std::vector<std::uint32_t> triangulate(std::vector<glm::dvec2> const &Outline);

// samples the height of the ground outside of a chunk; returns false where it doesn't know
using height_source = std::function<bool(double const X, double const Z, double &Height)>;

// geometry of a chunk at specified level of detail: grid points taken every Stride points of the grid, with a skirt
// along the edges hiding the gaps to the neighbours drawn with another grid. positions relative to Origin
struct mesh
{
	gfx::vertex_array vertices;
	gfx::index_array indices;
	float radius{0.f}; // from the centre of the vertices
	glm::dvec3 centre{0.0};
	glm::dvec3 low{0.0}, high{0.0}; // world bounds of the grid, the skirts included
};
// strides the grid of a chunk with specified cells can be drawn with
int max_stride(int const Cells);
void build_mesh(chunk_data const &Chunk, int const Stride, glm::dvec3 const &Origin, height_source const &Outside, mesh &Out);
// indices of the mesh of a chunk with specified cells and stride; the same for every chunk of these
void build_indices(int const Cells, int const Stride, gfx::index_array &Out);
// vertices the mesh of a chunk with specified cells and stride has
std::size_t mesh_vertex_count(int const Cells, int const Stride);

} // namespace heightmap
