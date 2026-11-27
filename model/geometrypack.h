/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// E3D1: the e3d model with its geometry kept apart from the rest of the model, so that the copies of one part
// (a bogie, a seat, a cab) which every variant of a vehicle carries are stored once.
//
// the file is the E3D0 one with the IDX and VNT chunks replaced:
//   E3D1 <size>
//     SUB0 / TRA0 / TEX0 / NAM0  as in E3D0, byte for byte (the geometry offsets of SUB0 are left as written, unused)
//     GRF0  one record per sub-model, in the order of SUB0: how its geometry is stored and where to find it
//     GRI0  the geometry blocks kept in the file itself (the small ones, or all of them without a pack)
// a geometry block is the vertex data of one sub-model in the VNT format of the record, followed by its indices
// in the narrowest width (1, 2 or 4 bytes) which holds the largest of them. the blocks not kept in the file
// are found by their content hash in the geometry pack, one file shared by the models of a vehicle family:
//   E3DP <version>  then the blocks, then the index (hash, offset, size; sorted by hash), then the footer
// the vertex and index data are decoded the same way for both formats (decode_vertices(), decode_indices()).

#include "rendering/geometrybank.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace e3d {

// 128-bit content hash of a geometry block (MurmurHash3 x64 128)
struct hash128 {
	std::uint64_t low{0};
	std::uint64_t high{0};

	bool operator==(hash128 const &Right) const { return low == Right.low && high == Right.high; }
	bool operator<(hash128 const &Right) const { return high != Right.high ? high < Right.high : low < Right.low; }
	std::string to_string() const;
};
hash128 murmur3_128(void const *Data, std::size_t Size, std::uint32_t Seed = 0);

struct hash128_hasher {
	std::size_t operator()(hash128 const &Hash) const { return static_cast<std::size_t>(Hash.low ^ (Hash.high * 0x9E3779B97F4A7C15ull)); }
};

// the size in bytes of one vertex of the VNT format (the digit of the chunk: 0 legacy, 1 packed, 2 expanded, +4 user data)
std::size_t vertex_stride(int Format);
// the vertex format carries tangents
inline bool has_tangents(int Format) { return (Format & 3) > 0; }
inline bool has_userdata(int Format) { return (Format & 4) != 0; }

// vertex data of the given format into the arrays given, sized beforehand. returns true if a legacy (format 0) vertex
// with a normal vector of other than unit length was found and Checknormals was set
bool decode_vertices(unsigned char const *Data, std::size_t Count, int Format, gfx::basic_vertex *Vertices, gfx::vertex_userdata *Userdata, bool Checknormals);
// index data of the given width (1, 2 or 4 bytes) into the array given, sized beforehand
void decode_indices(unsigned char const *Data, std::size_t Count, int Size, gfx::basic_index *Indices);
// appends the indices in the narrowest width able to hold them, returns the width (0 if there are none)
int encode_indices(gfx::basic_index const *Indices, std::size_t Count, std::string &Output);

// the same bits in every component: -0.0 differs from 0.0 and a NaN equals a NaN with the same payload
template <glm::length_t Length, glm::qualifier Qualifier>
bool same_bits(glm::vec<Length, float, Qualifier> const &Left, glm::vec<Length, float, Qualifier> const &Right)
{
	for (glm::length_t i = 0; i < Length; ++i)
		if (std::bit_cast<std::uint32_t>(Left[i]) != std::bit_cast<std::uint32_t>(Right[i]))
			return false;
	return true;
}
inline bool same_bits(gfx::basic_vertex const &Left, gfx::basic_vertex const &Right)
{
	return same_bits(Left.position, Right.position) && same_bits(Left.normal, Right.normal) && same_bits(Left.texture, Right.texture) &&
	       same_bits(Left.tangent, Right.tangent);
}
inline bool same_bits(gfx::vertex_userdata const &Left, gfx::vertex_userdata const &Right) { return same_bits(Left.data, Right.data); }
// the arrays have the same size and their elements the same bits
template <typename Type>
bool same_bits(std::vector<Type> const &Left, std::vector<Type> const &Right)
{
	return Left.size() == Right.size() && std::equal(Left.begin(), Left.end(), Right.begin(), [](Type const &L, Type const &R) { return same_bits(L, R); });
}

// how the geometry of a sub-model is stored, one record of the GRF0 chunk
struct geometry_record {
	static constexpr std::size_t disk_size{40};
	enum flag : std::uint8_t {
		inline_data = 0x1, // the block is in the GRI0 chunk of the file, at offset
	};
	std::uint8_t vertex_format{0};
	std::uint8_t index_size{0}; // 0 when the sub-model has no indices
	std::uint8_t flags{0};
	std::uint32_t vertex_count{0};
	std::uint32_t index_count{0};
	std::uint64_t offset{0}; // of the block in GRI0, for an inline one
	std::uint32_t size{0}; // of the block, bytes
	hash128 hash; // of the block content, prefixed with the format, the index width and the counts

	bool empty() const { return vertex_count == 0 && index_count == 0; }
	std::size_t vertex_bytes() const { return vertex_count * vertex_stride(vertex_format); }
	std::size_t index_bytes() const { return std::size_t{index_count} * index_size; }
};

// the geometry of one sub-model, as handed to the writer
struct geometry_block {
	int vertex_format{0};
	int index_size{0};
	std::uint32_t vertex_count{0};
	std::uint32_t index_count{0};
	std::string data; // vertex bytes, then index bytes

	bool empty() const { return vertex_count == 0 && index_count == 0; }
};
hash128 block_hash(int Vertexformat, int Indexsize, std::uint32_t Vertexcount, std::uint32_t Indexcount, std::string_view Data);

// the parts of a model the E3D1 writer puts together. the chunks are complete, with their headers
struct e3d1_content {
	std::string sub; // SUB0 (or SUB1)
	std::string tra; // TRA0 or TRA1
	std::string tex; // TEX0, may be empty
	std::string nam; // NAM0, may be empty
	std::vector<geometry_block> blocks; // one per sub-model, in the order of SUB
};

struct writer_options {
	std::size_t inline_threshold{4096}; // blocks smaller than this stay in the model file
	bool use_pack{true}; // without the pack every block stays in the file
	std::filesystem::path pack; // the pack file, the model file stores its path relative to its own folder
};

struct write_stats {
	std::size_t blocks{0}; // non-empty blocks of the model
	std::size_t inline_blocks{0};
	std::size_t packed_blocks{0}; // blocks referenced in the pack
	std::size_t new_blocks{0}; // of which appended to the pack now
	std::uint64_t new_bytes{0}; // appended to the pack
	std::uint64_t file_bytes{0}; // of the model file
	double hash_seconds{0}; // spent computing the hashes of the blocks
};

// writes an E3D1 model file and adds the blocks it doesn't keep to the pack; throws std::runtime_error on failure
write_stats write_e3d1(std::filesystem::path const &File, e3d1_content const &Content, writer_options const &Options);
// cuts the content of an E3D0 file in memory into the parts of an E3D1 one, without changing a byte of the
// vertex data. returns false with the reason given when the file can't be converted as is
bool split_e3d0(unsigned char const *Data, std::size_t Size, e3d1_content &Content, std::string &Error);

// geometry pack -----------------------------------------------------------------------------------------------

// footer: index offset (u64), entry count (u64), version (u32), reserved (u32), magic "E3DPACK1"
constexpr std::uint32_t pack_version{1};
constexpr std::size_t pack_header_size{16};
constexpr std::size_t pack_footer_size{32};
constexpr std::size_t pack_entry_size{32}; // hash (16), offset (u64), size (u64)

struct pack_entry {
	hash128 hash;
	std::uint64_t offset{0};
	std::uint64_t size{0};
};

// appends blocks to a pack; the index and the footer are rewritten past the data by flush()
class pack_writer {
  public:
	explicit pack_writer(std::filesystem::path const &Path);
	~pack_writer();
	pack_writer(pack_writer const &) = delete;
	pack_writer &operator=(pack_writer const &) = delete;
	// adds the block unless the pack has it already, returns true if it was appended. a block of the same hash already
	// in the pack is compared byte for byte, a difference (a hash collision) throws
	bool add(hash128 const &Hash, std::string_view Data);
	void flush();
	std::size_t size() const { return m_entries.size(); }

  private:
	std::filesystem::path m_path;
	std::FILE *m_file{nullptr};
	std::uint64_t m_dataend{pack_header_size};
	std::unordered_map<hash128, pack_entry, hash128_hasher> m_entries;
	bool m_dirty{false};
};
// the writer of a pack, one per pack for the process
pack_writer &pack_writer_for(std::filesystem::path const &Path);
// writes the index of every pack written to, and closes them
void flush_pack_writers();

// the read side of a pack: its index read once, the blocks read on demand
class pack_reader {
  public:
	// the reader of a pack, opened once for the process; nullptr (with the reason given) if it can't be opened
	static std::shared_ptr<pack_reader> open(std::filesystem::path const &Path, std::string &Error);
	// forgets the packs opened; for tests and the benchmark
	static void close_all();
	~pack_reader();
	pack_reader(pack_reader const &) = delete;
	pack_reader &operator=(pack_reader const &) = delete;

	pack_entry const *find(hash128 const &Hash) const;
	// reads the given blocks, the ones lying close together in one go. Blocks[i] gets the data of Entries[i].
	// returns false (with the reason given) on a read error
	bool read(std::vector<pack_entry const *> const &Entries, std::vector<unsigned char const *> &Blocks, std::vector<unsigned char> &Storage, std::string &Error) const;
	std::filesystem::path const &path() const { return m_path; }

  private:
	pack_reader() = default;
	std::filesystem::path m_path;
	std::FILE *m_file{nullptr};
	std::uint64_t m_filesize{0};
	std::vector<pack_entry> m_index; // sorted by hash
	mutable std::mutex m_lock;
};

// bytes read from the packs since the start (index and blocks); for the benchmark
std::uint64_t pack_bytes_read();

// the geometry of an E3D1 file, located but not decoded yet
struct e3d1_geometry {
	std::vector<geometry_record> records;
	std::string pack; // path of the pack relative to the folder of the model file, empty if none is used
	unsigned char const *inline_data{nullptr};
	std::size_t inline_size{0};
};
// reads the GRF0 chunk content (without the chunk header); false with the reason given if it is malformed
bool parse_grf0(unsigned char const *Data, std::size_t Size, e3d1_geometry &Geometry, std::string &Error);

// the data of every block of a model, found in the file or read from the pack
struct resolved_geometry {
	std::vector<unsigned char const *> blocks; // per record; nullptr for an empty record or one not found
	std::vector<std::string> errors; // why blocks are missing
	std::vector<unsigned char> storage; // of the blocks read from the pack
};
void resolve_geometry(e3d1_geometry const &Geometry, std::filesystem::path const &Modelfolder, resolved_geometry &Resolved);

} // namespace e3d
