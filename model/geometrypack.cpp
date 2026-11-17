/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "model/geometrypack.h"

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <stdexcept>

#include <glm/gtc/packing.hpp>

namespace e3d {

namespace {

constexpr std::uint32_t id4(char a, char b, char c, char d)
{
	return (std::uint32_t(std::uint8_t(d)) << 24) | (std::uint32_t(std::uint8_t(c)) << 16) | (std::uint32_t(std::uint8_t(b)) << 8) | std::uint32_t(std::uint8_t(a));
}
constexpr char pack_magic[8]{'E', '3', 'D', 'P', 'A', 'C', 'K', '1'};

std::atomic<std::uint64_t> g_packbytesread{0};

// little endian values, read the way sn_utils reads them from a stream
std::uint16_t get_u16(unsigned char const *&Data)
{
	std::uint16_t const value = static_cast<std::uint16_t>(Data[1] << 8 | Data[0]);
	Data += 2;
	return value;
}
std::uint32_t get_u32(unsigned char const *&Data)
{
	std::uint32_t const value = std::uint32_t{Data[3]} << 24 | std::uint32_t{Data[2]} << 16 | std::uint32_t{Data[1]} << 8 | std::uint32_t{Data[0]};
	Data += 4;
	return value;
}
std::uint64_t get_u64(unsigned char const *&Data)
{
	std::uint64_t const low{get_u32(Data)};
	std::uint64_t const high{get_u32(Data)};
	return high << 32 | low;
}
float get_f32(unsigned char const *&Data)
{
	return std::bit_cast<float>(get_u32(Data));
}
glm::vec3 get_vec3(unsigned char const *&Data)
{
	glm::vec3 value;
	value.x = get_f32(Data);
	value.y = get_f32(Data);
	value.z = get_f32(Data);
	return value;
}
glm::vec4 get_vec4(unsigned char const *&Data)
{
	glm::vec4 value;
	value.x = get_f32(Data);
	value.y = get_f32(Data);
	value.z = get_f32(Data);
	value.w = get_f32(Data);
	return value;
}
std::int32_t get_i32_at(unsigned char const *Data)
{
	return static_cast<std::int32_t>(get_u32(Data));
}

void put_u8(std::string &Output, std::uint8_t Value)
{
	Output.push_back(static_cast<char>(Value));
}
void put_u16(std::string &Output, std::uint16_t Value)
{
	put_u8(Output, Value & 0xFF);
	put_u8(Output, Value >> 8);
}
void put_u32(std::string &Output, std::uint32_t Value)
{
	put_u16(Output, Value & 0xFFFF);
	put_u16(Output, Value >> 16);
}
void put_u64(std::string &Output, std::uint64_t Value)
{
	put_u32(Output, static_cast<std::uint32_t>(Value));
	put_u32(Output, static_cast<std::uint32_t>(Value >> 32));
}

// the vertex data of basic_vertex::deserialize() and basic_vertex::deserialize_packed()
void vertex(unsigned char const *&Data, gfx::basic_vertex &Vertex, bool const Tangent)
{
	Vertex.position = get_vec3(Data);
	Vertex.normal = get_vec3(Data);
	Vertex.texture.x = get_f32(Data);
	Vertex.texture.y = get_f32(Data);
	if (Tangent)
	{
		Vertex.tangent = get_vec4(Data);
	}
}
void vertex_packed(unsigned char const *&Data, gfx::basic_vertex &Vertex, bool const Tangent)
{
	Vertex.position = glm::unpackHalf4x16(get_u64(Data));
	Vertex.normal = glm::unpackSnorm3x10_1x2(get_u32(Data));
	Vertex.texture.x = glm::unpackHalf1x16(get_u16(Data));
	Vertex.texture.y = glm::unpackHalf1x16(get_u16(Data));
	if (Tangent)
	{
		Vertex.tangent = glm::unpackSnorm3x10_1x2(get_u32(Data));
	}
}

// MurmurHash3 x64 128, by Austin Appleby, placed in the public domain
inline std::uint64_t rotl64(std::uint64_t x, int r)
{
	return (x << r) | (x >> (64 - r));
}
inline std::uint64_t fmix64(std::uint64_t k)
{
	k ^= k >> 33;
	k *= 0xff51afd7ed558ccdull;
	k ^= k >> 33;
	k *= 0xc4ceb9fe1a85ec53ull;
	k ^= k >> 33;
	return k;
}
inline std::uint64_t load64(unsigned char const *p)
{
	std::uint64_t v;
	std::memcpy(&v, p, 8); // little endian hosts only, as the rest of the e3d code
	return v;
}

std::FILE *open_file(std::filesystem::path const &Path, char const *Mode)
{
#ifdef _WIN32
	std::wstring const mode(Mode, Mode + std::strlen(Mode));
	return _wfopen(Path.c_str(), mode.c_str());
#else
	return std::fopen(Path.c_str(), Mode);
#endif
}
bool seek(std::FILE *File, std::uint64_t Offset)
{
#ifdef _WIN32
	return _fseeki64(File, static_cast<__int64>(Offset), SEEK_SET) == 0;
#else
	return fseeko(File, static_cast<off_t>(Offset), SEEK_SET) == 0;
#endif
}
bool read_at(std::FILE *File, std::uint64_t Offset, void *Data, std::size_t Size)
{
	if (false == seek(File, Offset))
		return false;
	auto const read{std::fread(Data, 1, Size, File)};
	g_packbytesread += read;
	return read == Size;
}

std::string path_key(std::filesystem::path const &Path)
{
	std::error_code error;
	auto canonical{std::filesystem::weakly_canonical(Path, error)};
	auto key{(error ? Path : canonical).lexically_normal().generic_string()};
#ifdef _WIN32
	std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
#endif
	return key;
}

// a gap between two blocks of a model read in the same go, rather than seeking past it
constexpr std::uint64_t coalesce_gap{256 * 1024};

} // namespace

std::string hash128::to_string() const
{
	char text[33];
	std::snprintf(text, sizeof(text), "%016llx%016llx", static_cast<unsigned long long>(high), static_cast<unsigned long long>(low));
	return text;
}

hash128 murmur3_128(void const *Data, std::size_t Size, std::uint32_t Seed)
{
	auto const *data{static_cast<unsigned char const *>(Data)};
	std::size_t const nblocks{Size / 16};
	std::uint64_t h1{Seed};
	std::uint64_t h2{Seed};
	std::uint64_t const c1{0x87c37b91114253d5ull};
	std::uint64_t const c2{0x4cf5ad432745937full};

	for (std::size_t i = 0; i < nblocks; ++i)
	{
		std::uint64_t k1{load64(data + i * 16)};
		std::uint64_t k2{load64(data + i * 16 + 8)};
		k1 *= c1;
		k1 = rotl64(k1, 31);
		k1 *= c2;
		h1 ^= k1;
		h1 = rotl64(h1, 27);
		h1 += h2;
		h1 = h1 * 5 + 0x52dce729;
		k2 *= c2;
		k2 = rotl64(k2, 33);
		k2 *= c1;
		h2 ^= k2;
		h2 = rotl64(h2, 31);
		h2 += h1;
		h2 = h2 * 5 + 0x38495ab5;
	}

	unsigned char const *tail{data + nblocks * 16};
	std::uint64_t k1{0};
	std::uint64_t k2{0};
	switch (Size & 15)
	{
	case 15: k2 ^= std::uint64_t(tail[14]) << 48; [[fallthrough]];
	case 14: k2 ^= std::uint64_t(tail[13]) << 40; [[fallthrough]];
	case 13: k2 ^= std::uint64_t(tail[12]) << 32; [[fallthrough]];
	case 12: k2 ^= std::uint64_t(tail[11]) << 24; [[fallthrough]];
	case 11: k2 ^= std::uint64_t(tail[10]) << 16; [[fallthrough]];
	case 10: k2 ^= std::uint64_t(tail[9]) << 8; [[fallthrough]];
	case 9:
		k2 ^= std::uint64_t(tail[8]);
		k2 *= c2;
		k2 = rotl64(k2, 33);
		k2 *= c1;
		h2 ^= k2;
		[[fallthrough]];
	case 8: k1 ^= std::uint64_t(tail[7]) << 56; [[fallthrough]];
	case 7: k1 ^= std::uint64_t(tail[6]) << 48; [[fallthrough]];
	case 6: k1 ^= std::uint64_t(tail[5]) << 40; [[fallthrough]];
	case 5: k1 ^= std::uint64_t(tail[4]) << 32; [[fallthrough]];
	case 4: k1 ^= std::uint64_t(tail[3]) << 24; [[fallthrough]];
	case 3: k1 ^= std::uint64_t(tail[2]) << 16; [[fallthrough]];
	case 2: k1 ^= std::uint64_t(tail[1]) << 8; [[fallthrough]];
	case 1:
		k1 ^= std::uint64_t(tail[0]);
		k1 *= c1;
		k1 = rotl64(k1, 31);
		k1 *= c2;
		h1 ^= k1;
		break;
	default: break;
	}

	h1 ^= Size;
	h2 ^= Size;
	h1 += h2;
	h2 += h1;
	h1 = fmix64(h1);
	h2 = fmix64(h2);
	h1 += h2;
	h2 += h1;
	return {h1, h2};
}

std::size_t vertex_stride(int const Format)
{
	switch (Format & 3)
	{
	case 0: return has_userdata(Format) ? 48 : 32;
	case 1: return has_userdata(Format) ? 28 : 20;
	case 2: return has_userdata(Format) ? 64 : 48;
	default: return 0;
	}
}

bool decode_vertices(unsigned char const *Data, std::size_t const Count, int const Format, gfx::basic_vertex *Vertices, gfx::vertex_userdata *Userdata, bool const Checknormals)
{
	bool const tangents{has_tangents(Format)};
	bool const userdata{has_userdata(Format)};
	bool nonunitnormal{false};
	switch (Format & 3)
	{
	case 0:
	{
		// legacy vnt0 format
		for (std::size_t i = 0; i < Count; ++i)
		{
			vertex(Data, Vertices[i], tangents);
			if (userdata)
				Userdata[i].data = get_vec4(Data);
			// normal vectors debug routine
			if (Checknormals && false == nonunitnormal && std::abs(glm::length2(Vertices[i].normal) - 1.0f) > 0.01f)
				nonunitnormal = true;
		}
		break;
	}
	case 1:
	{
		for (std::size_t i = 0; i < Count; ++i)
		{
			vertex_packed(Data, Vertices[i], tangents);
			if (userdata)
				Userdata[i].data = glm::unpackHalf4x16(get_u64(Data));
		}
		break;
	}
	case 2:
	{
		for (std::size_t i = 0; i < Count; ++i)
		{
			vertex(Data, Vertices[i], tangents);
			if (userdata)
				Userdata[i].data = get_vec4(Data);
		}
		break;
	}
	default: break;
	}
	return nonunitnormal;
}

void decode_indices(unsigned char const *Data, std::size_t const Count, int const Size, gfx::basic_index *Indices)
{
	switch (Size)
	{
	case 1:
		for (std::size_t i = 0; i < Count; ++i)
			Indices[i] = *Data++;
		break;
	case 2:
		for (std::size_t i = 0; i < Count; ++i)
			Indices[i] = get_u16(Data);
		break;
	case 4:
		for (std::size_t i = 0; i < Count; ++i)
			Indices[i] = get_u32(Data);
		break;
	default: break;
	}
}

int encode_indices(gfx::basic_index const *Indices, std::size_t const Count, std::string &Output)
{
	if (Count == 0)
		return 0;
	auto const largest{*std::max_element(Indices, Indices + Count)};
	int const size{largest < (1u << 8) ? 1 : largest < (1u << 16) ? 2 : 4};
	Output.reserve(Output.size() + Count * size);
	for (std::size_t i = 0; i < Count; ++i)
	{
		switch (size)
		{
		case 1: put_u8(Output, static_cast<std::uint8_t>(Indices[i])); break;
		case 2: put_u16(Output, static_cast<std::uint16_t>(Indices[i])); break;
		default: put_u32(Output, Indices[i]); break;
		}
	}
	return size;
}

hash128 block_hash(int const Vertexformat, int const Indexsize, std::uint32_t const Vertexcount, std::uint32_t const Indexcount, std::string_view const Data)
{
	auto const content{murmur3_128(Data.data(), Data.size())};
	// the layout of the block goes into its hash, the same bytes cut another way are another block
	std::string key;
	put_u64(key, content.low);
	put_u64(key, content.high);
	put_u32(key, static_cast<std::uint32_t>(Vertexformat));
	put_u32(key, static_cast<std::uint32_t>(Indexsize));
	put_u32(key, Vertexcount);
	put_u32(key, Indexcount);
	return murmur3_128(key.data(), key.size(), 0xE3D1);
}

// writer ------------------------------------------------------------------------------------------------------

write_stats write_e3d1(std::filesystem::path const &File, e3d1_content const &Content, writer_options const &Options)
{
	write_stats stats;
	std::string pack;
	if (Options.use_pack)
	{
		pack = Options.pack.lexically_relative(File.parent_path()).generic_string();
		if (pack.empty())
			pack = Options.pack.generic_string();
	}
	std::string records;
	std::string inlinedata;
	for (auto const &block : Content.blocks)
	{
		geometry_record record;
		record.vertex_format = static_cast<std::uint8_t>(block.vertex_format);
		record.index_size = static_cast<std::uint8_t>(block.index_size);
		record.vertex_count = block.vertex_count;
		record.index_count = block.index_count;
		if (false == block.empty())
		{
			if (block.data.size() != record.vertex_bytes() + record.index_bytes())
				throw std::runtime_error("e3d1: size of the geometry block doesn't match its counts");
			record.size = static_cast<std::uint32_t>(block.data.size());
			auto const hashstart{std::chrono::steady_clock::now()};
			record.hash = block_hash(block.vertex_format, block.index_size, block.vertex_count, block.index_count, block.data);
			stats.hash_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - hashstart).count();
			++stats.blocks;
			if (false == Options.use_pack || block.data.size() < Options.inline_threshold)
			{
				record.flags |= geometry_record::inline_data;
				record.offset = inlinedata.size();
				inlinedata += block.data;
				++stats.inline_blocks;
			}
			else
			{
				++stats.packed_blocks;
				if (pack_writer_for(Options.pack).add(record.hash, block.data))
				{
					++stats.new_blocks;
					stats.new_bytes += block.data.size();
				}
			}
		}
		put_u8(records, record.vertex_format);
		put_u8(records, record.index_size);
		put_u8(records, record.flags);
		put_u8(records, 0);
		put_u32(records, record.vertex_count);
		put_u32(records, record.index_count);
		put_u32(records, record.size);
		put_u64(records, record.offset);
		put_u64(records, record.hash.low);
		put_u64(records, record.hash.high);
	}

	std::string grf;
	put_u32(grf, id4('G', 'R', 'F', '0'));
	put_u32(grf, 0);
	put_u32(grf, static_cast<std::uint32_t>(Content.blocks.size()));
	put_u32(grf, static_cast<std::uint32_t>(geometry_record::disk_size));
	put_u32(grf, static_cast<std::uint32_t>(pack.size()));
	grf += pack;
	while (grf.size() % 4)
		grf.push_back('\0');
	grf += records;
	auto const setsize = [](std::string &Chunk) {
		auto const size{static_cast<std::uint32_t>(Chunk.size())};
		for (int i = 0; i < 4; ++i)
			Chunk[4 + i] = static_cast<char>((size >> (8 * i)) & 0xFF);
	};
	setsize(grf);
	std::string gri;
	if (false == inlinedata.empty())
	{
		put_u32(gri, id4('G', 'R', 'I', '0'));
		put_u32(gri, 0);
		gri += inlinedata;
		setsize(gri);
	}

	std::string file;
	put_u32(file, id4('E', '3', 'D', '1'));
	put_u32(file, 0);
	file += Content.sub;
	file += Content.tra;
	file += grf;
	file += gri;
	file += Content.tex;
	file += Content.nam;
	setsize(file);

	std::error_code error;
	if (File.has_parent_path())
		std::filesystem::create_directories(File.parent_path(), error);
	auto *output{open_file(File, "wb")};
	if (output == nullptr)
		throw std::runtime_error("e3d1: can't write \"" + File.generic_string() + "\"");
	auto const written{std::fwrite(file.data(), 1, file.size(), output)};
	std::fclose(output);
	if (written != file.size())
		throw std::runtime_error("e3d1: failed to write \"" + File.generic_string() + "\"");
	stats.file_bytes = file.size();
	return stats;
}

bool split_e3d0(unsigned char const *Data, std::size_t const Size, e3d1_content &Content, std::string &Error)
{
	Content = {};
	if (Size < 8)
	{
		Error = "file too short";
		return false;
	}
	unsigned char const *cursor{Data};
	if (get_u32(cursor) != id4('E', '3', 'D', '0'))
	{
		Error = "not an E3D0 file";
		return false;
	}
	auto const mainsize{get_u32(cursor)};
	if (mainsize < 8 || mainsize > Size)
	{
		Error = "bad size of the main chunk";
		return false;
	}
	unsigned char const *const end{Data + mainsize};
	unsigned char const *sub{nullptr};
	std::size_t subsize{0}, recordsize{0};
	unsigned char const *vnt{nullptr};
	std::size_t vntsize{0};
	int vertexformat{-1};
	unsigned char const *idx{nullptr};
	std::size_t idxsize{0};
	int indexsize{0};
	while (cursor + 8 <= end)
	{
		unsigned char const *const chunk{cursor};
		auto const type{get_u32(cursor)};
		auto const chunksize{get_u32(cursor)};
		if (chunksize < 8 || chunk + chunksize > end)
		{
			Error = "bad chunk size";
			return false;
		}
		unsigned char const *const payload{chunk + 8};
		std::size_t const payloadsize{chunksize - 8};
		auto const kind{type & 0x00FFFFFF};
		int const digit{static_cast<int>((type >> 24) & 0xFF) - '0'};
		std::string_view const whole(reinterpret_cast<char const *>(chunk), chunksize);
		if (kind == (id4('S', 'U', 'B', 0) & 0x00FFFFFF))
		{
			if (sub != nullptr)
			{
				Error = "duplicated SUB chunk";
				return false;
			}
			sub = payload;
			subsize = payloadsize;
			recordsize = 256 + 64 * static_cast<std::size_t>(std::max(digit, 0));
			Content.sub = whole;
		}
		else if (kind == (id4('V', 'N', 'T', 0) & 0x00FFFFFF))
		{
			if (vnt != nullptr || vertex_stride(digit) == 0)
			{
				Error = "unsupported or duplicated VNT chunk";
				return false;
			}
			vnt = payload;
			vntsize = payloadsize;
			vertexformat = digit;
		}
		else if (kind == (id4('I', 'D', 'X', 0) & 0x00FFFFFF))
		{
			if (idx != nullptr || (digit != 1 && digit != 2 && digit != 4))
			{
				Error = "unsupported or duplicated IDX chunk";
				return false;
			}
			idx = payload;
			idxsize = payloadsize;
			indexsize = digit;
		}
		else if (type == id4('T', 'R', 'A', '0') || type == id4('T', 'R', 'A', '1'))
			Content.tra = whole;
		else if (type == id4('T', 'E', 'X', '0'))
			Content.tex = whole;
		else if (type == id4('N', 'A', 'M', '0'))
			Content.nam = whole;
		else
		{
			Error = "unknown chunk";
			return false;
		}
		cursor = chunk + chunksize;
	}
	if (sub == nullptr)
	{
		Error = "no SUB chunk";
		return false;
	}
	std::size_t const count{subsize / recordsize};
	Content.blocks.resize(count);
	std::vector<gfx::basic_index> indices;
	for (std::size_t i = 0; i < count; ++i)
	{
		unsigned char const *const record{sub + i * recordsize};
		auto const vertexcount{get_i32_at(record + 28)};
		auto const vertexoffset{get_i32_at(record + 32)};
		auto const indexcount{get_i32_at(record + 156)};
		auto const indexoffset{get_i32_at(record + 160)};
		auto &block{Content.blocks[i]};
		if (vertexcount > 0)
		{
			auto const stride{vnt != nullptr ? vertex_stride(vertexformat) : 0};
			std::uint64_t const begin{static_cast<std::uint64_t>(std::max(vertexoffset, 0)) * stride};
			std::uint64_t const bytes{static_cast<std::uint64_t>(vertexcount) * stride};
			if (vnt == nullptr || vertexoffset < 0 || begin + bytes > vntsize)
			{
				Error = "vertex data of sub-model " + std::to_string(i) + " out of the VNT chunk";
				return false;
			}
			block.vertex_format = vertexformat;
			block.vertex_count = static_cast<std::uint32_t>(vertexcount);
			block.data.assign(reinterpret_cast<char const *>(vnt + begin), static_cast<std::size_t>(bytes));
		}
		else if (vnt != nullptr)
			block.vertex_format = vertexformat;
		if (indexcount > 0)
		{
			std::uint64_t const begin{static_cast<std::uint64_t>(std::max(indexoffset, 0)) * indexsize};
			std::uint64_t const bytes{static_cast<std::uint64_t>(indexcount) * indexsize};
			if (idx == nullptr || indexoffset < 0 || begin + bytes > idxsize)
			{
				Error = "index data of sub-model " + std::to_string(i) + " out of the IDX chunk";
				return false;
			}
			indices.resize(static_cast<std::size_t>(indexcount));
			decode_indices(idx + begin, indices.size(), indexsize, indices.data());
			block.index_count = static_cast<std::uint32_t>(indexcount);
			block.index_size = encode_indices(indices.data(), indices.size(), block.data);
		}
	}
	return true;
}

// pack writer -------------------------------------------------------------------------------------------------

pack_writer::pack_writer(std::filesystem::path const &Path) : m_path(Path)
{
	std::error_code error;
	if (std::filesystem::exists(Path, error))
	{
		m_file = open_file(Path, "r+b");
		if (m_file == nullptr)
			throw std::runtime_error("e3dpack: can't open \"" + Path.generic_string() + "\" for writing");
		auto const filesize{std::filesystem::file_size(Path, error)};
		unsigned char footer[pack_footer_size];
		if (filesize < pack_header_size + pack_footer_size || false == read_at(m_file, filesize - pack_footer_size, footer, sizeof(footer)) ||
		    std::memcmp(footer + 24, pack_magic, 8) != 0)
			throw std::runtime_error("e3dpack: \"" + Path.generic_string() + "\" is not a geometry pack");
		unsigned char const *cursor{footer};
		auto const indexoffset{get_u64(cursor)};
		auto const entrycount{get_u64(cursor)};
		std::vector<unsigned char> index(static_cast<std::size_t>(entrycount * pack_entry_size));
		if (false == read_at(m_file, indexoffset, index.data(), index.size()))
			throw std::runtime_error("e3dpack: can't read the index of \"" + Path.generic_string() + "\"");
		cursor = index.data();
		for (std::uint64_t i = 0; i < entrycount; ++i)
		{
			pack_entry entry;
			entry.hash.low = get_u64(cursor);
			entry.hash.high = get_u64(cursor);
			entry.offset = get_u64(cursor);
			entry.size = get_u64(cursor);
			m_entries.emplace(entry.hash, entry);
		}
		m_dataend = indexoffset;
	}
	else
	{
		if (Path.has_parent_path())
			std::filesystem::create_directories(Path.parent_path(), error);
		m_file = open_file(Path, "w+b");
		if (m_file == nullptr)
			throw std::runtime_error("e3dpack: can't create \"" + Path.generic_string() + "\"");
		std::string header;
		put_u32(header, id4('E', '3', 'D', 'P'));
		put_u32(header, pack_version);
		put_u64(header, 0);
		std::fwrite(header.data(), 1, header.size(), m_file);
		m_dataend = pack_header_size;
		m_dirty = true;
	}
}

pack_writer::~pack_writer()
{
	try
	{
		flush();
	}
	catch (...)
	{
	}
	if (m_file != nullptr)
		std::fclose(m_file);
}

bool pack_writer::add(hash128 const &Hash, std::string_view const Data)
{
	auto const found{m_entries.find(Hash)};
	if (found != m_entries.end())
	{
		// the same hash: make sure it is the same block
		std::string existing(static_cast<std::size_t>(found->second.size), '\0');
		if (found->second.size != Data.size() || false == read_at(m_file, found->second.offset, existing.data(), existing.size()) || existing != Data)
			throw std::runtime_error("e3dpack: hash collision or damaged block " + Hash.to_string() + " in \"" + m_path.generic_string() + "\"");
		return false;
	}
	if (false == seek(m_file, m_dataend) || std::fwrite(Data.data(), 1, Data.size(), m_file) != Data.size())
		throw std::runtime_error("e3dpack: failed to append to \"" + m_path.generic_string() + "\"");
	m_entries.emplace(Hash, pack_entry{Hash, m_dataend, Data.size()});
	m_dataend += Data.size();
	m_dirty = true;
	return true;
}

void pack_writer::flush()
{
	if (false == m_dirty || m_file == nullptr)
		return;
	std::vector<pack_entry> entries;
	entries.reserve(m_entries.size());
	for (auto const &entry : m_entries)
		entries.push_back(entry.second);
	std::sort(entries.begin(), entries.end(), [](pack_entry const &Left, pack_entry const &Right) { return Left.hash < Right.hash; });
	std::string tail;
	tail.reserve(entries.size() * pack_entry_size + pack_footer_size);
	for (auto const &entry : entries)
	{
		put_u64(tail, entry.hash.low);
		put_u64(tail, entry.hash.high);
		put_u64(tail, entry.offset);
		put_u64(tail, entry.size);
	}
	put_u64(tail, m_dataend);
	put_u64(tail, entries.size());
	put_u32(tail, pack_version);
	put_u32(tail, 0);
	tail.append(pack_magic, 8);
	// the data end only grows and so does the index, so the new tail covers the old one whole
	if (false == seek(m_file, m_dataend) || std::fwrite(tail.data(), 1, tail.size(), m_file) != tail.size() || std::fflush(m_file) != 0)
		throw std::runtime_error("e3dpack: failed to write the index of \"" + m_path.generic_string() + "\"");
	m_dirty = false;
}

namespace {
std::map<std::string, std::unique_ptr<pack_writer>> &pack_writers()
{
	static std::map<std::string, std::unique_ptr<pack_writer>> writers;
	return writers;
}
} // namespace

pack_writer &pack_writer_for(std::filesystem::path const &Path)
{
	auto &writers{pack_writers()};
	auto const key{path_key(Path)};
	auto found{writers.find(key)};
	if (found == writers.end())
		found = writers.emplace(key, std::make_unique<pack_writer>(Path)).first;
	return *found->second;
}

void flush_pack_writers()
{
	auto &writers{pack_writers()};
	for (auto &writer : writers)
		writer.second->flush();
	writers.clear();
	// a pack read before is stale now
	pack_reader::close_all();
}

// pack reader -------------------------------------------------------------------------------------------------

namespace {
std::mutex g_readerslock;
std::map<std::string, std::shared_ptr<pack_reader>> &pack_readers()
{
	static std::map<std::string, std::shared_ptr<pack_reader>> readers;
	return readers;
}
} // namespace

std::shared_ptr<pack_reader> pack_reader::open(std::filesystem::path const &Path, std::string &Error)
{
	std::scoped_lock lock(g_readerslock);
	auto &readers{pack_readers()};
	auto const key{path_key(Path)};
	if (auto const found{readers.find(key)}; found != readers.end())
		return found->second;

	std::shared_ptr<pack_reader> reader(new pack_reader());
	reader->m_path = Path;
	reader->m_file = open_file(Path, "rb");
	if (reader->m_file == nullptr)
	{
		Error = "geometry pack \"" + Path.generic_string() + "\" not found";
		return nullptr;
	}
	std::error_code error;
	reader->m_filesize = std::filesystem::file_size(Path, error);
	unsigned char footer[pack_footer_size];
	if (error || reader->m_filesize < pack_header_size + pack_footer_size || false == read_at(reader->m_file, reader->m_filesize - pack_footer_size, footer, sizeof(footer)) ||
	    std::memcmp(footer + 24, pack_magic, 8) != 0)
	{
		Error = "\"" + Path.generic_string() + "\" is not a geometry pack";
		return nullptr;
	}
	unsigned char const *cursor{footer};
	auto const indexoffset{get_u64(cursor)};
	auto const entrycount{get_u64(cursor)};
	auto const version{get_u32(cursor)};
	if (version != pack_version || indexoffset + entrycount * pack_entry_size + pack_footer_size != reader->m_filesize)
	{
		Error = "geometry pack \"" + Path.generic_string() + "\" of unknown version or damaged";
		return nullptr;
	}
	std::vector<unsigned char> index(static_cast<std::size_t>(entrycount * pack_entry_size));
	if (false == read_at(reader->m_file, indexoffset, index.data(), index.size()))
	{
		Error = "can't read the index of geometry pack \"" + Path.generic_string() + "\"";
		return nullptr;
	}
	cursor = index.data();
	reader->m_index.resize(static_cast<std::size_t>(entrycount));
	for (auto &entry : reader->m_index)
	{
		entry.hash.low = get_u64(cursor);
		entry.hash.high = get_u64(cursor);
		entry.offset = get_u64(cursor);
		entry.size = get_u64(cursor);
		if (entry.offset < pack_header_size || entry.offset + entry.size > indexoffset)
		{
			Error = "geometry pack \"" + Path.generic_string() + "\" has a block out of its data";
			return nullptr;
		}
	}
	if (false == std::is_sorted(reader->m_index.begin(), reader->m_index.end(), [](pack_entry const &Left, pack_entry const &Right) { return Left.hash < Right.hash; }))
	{
		Error = "the index of geometry pack \"" + Path.generic_string() + "\" isn't sorted";
		return nullptr;
	}
	readers.emplace(key, reader);
	return reader;
}

void pack_reader::close_all()
{
	std::scoped_lock lock(g_readerslock);
	pack_readers().clear();
}

pack_reader::~pack_reader()
{
	if (m_file != nullptr)
		std::fclose(m_file);
}

pack_entry const *pack_reader::find(hash128 const &Hash) const
{
	auto const found{std::lower_bound(m_index.begin(), m_index.end(), Hash, [](pack_entry const &Entry, hash128 const &Value) { return Entry.hash < Value; })};
	return (found != m_index.end() && found->hash == Hash) ? &*found : nullptr;
}

bool pack_reader::read(std::vector<pack_entry const *> const &Entries, std::vector<unsigned char const *> &Blocks, std::vector<unsigned char> &Storage, std::string &Error) const
{
	Blocks.assign(Entries.size(), nullptr);
	// blocks in the order they lie in the pack; the ones close together are read in one go
	std::vector<std::size_t> order;
	for (std::size_t i = 0; i < Entries.size(); ++i)
		if (Entries[i] != nullptr)
			order.push_back(i);
	std::sort(order.begin(), order.end(), [&](std::size_t Left, std::size_t Right) { return Entries[Left]->offset < Entries[Right]->offset; });
	struct span {
		std::uint64_t begin, end;
		std::size_t storage;
	};
	std::vector<span> spans;
	std::vector<std::size_t> spanof(Entries.size());
	std::size_t total{0};
	auto const gap{coalesce_gap};
	for (auto const i : order)
	{
		auto const begin{Entries[i]->offset};
		auto const end{begin + Entries[i]->size};
		if (spans.empty() || begin > spans.back().end + gap)
			spans.push_back({begin, end, 0});
		else
			spans.back().end = std::max(spans.back().end, end);
		spanof[i] = spans.size() - 1;
	}
	for (auto &s : spans)
	{
		s.storage = total;
		total += static_cast<std::size_t>(s.end - s.begin);
	}
	Storage.resize(total);
	{
		std::scoped_lock lock(m_lock);
		for (auto const &s : spans)
		{
			if (false == read_at(m_file, s.begin, Storage.data() + s.storage, static_cast<std::size_t>(s.end - s.begin)))
			{
				Error = "read error in geometry pack \"" + m_path.generic_string() + "\"";
				return false;
			}
		}
	}
	for (auto const i : order)
	{
		auto const &s{spans[spanof[i]]};
		Blocks[i] = Storage.data() + s.storage + (Entries[i]->offset - s.begin);
	}
	return true;
}

std::uint64_t pack_bytes_read()
{
	return g_packbytesread.load();
}

// loader ------------------------------------------------------------------------------------------------------

bool parse_grf0(unsigned char const *Data, std::size_t const Size, e3d1_geometry &Geometry, std::string &Error)
{
	Geometry.records.clear();
	Geometry.pack.clear();
	if (Size < 12)
	{
		Error = "GRF0 chunk too short";
		return false;
	}
	unsigned char const *cursor{Data};
	auto const count{get_u32(cursor)};
	auto const recordsize{get_u32(cursor)};
	auto const pathsize{get_u32(cursor)};
	std::size_t const header{(12 + std::size_t{pathsize} + 3) & ~std::size_t{3}};
	if (recordsize < geometry_record::disk_size || header > Size || (Size - header) / recordsize < count)
	{
		Error = "GRF0 chunk malformed";
		return false;
	}
	Geometry.pack.assign(reinterpret_cast<char const *>(cursor), pathsize);
	Geometry.records.resize(count);
	for (std::size_t i = 0; i < count; ++i)
	{
		cursor = Data + header + i * recordsize;
		auto &record{Geometry.records[i]};
		record.vertex_format = *cursor++;
		record.index_size = *cursor++;
		record.flags = *cursor++;
		++cursor;
		record.vertex_count = get_u32(cursor);
		record.index_count = get_u32(cursor);
		record.size = get_u32(cursor);
		record.offset = get_u64(cursor);
		record.hash.low = get_u64(cursor);
		record.hash.high = get_u64(cursor);
		if (false == record.empty() && (vertex_stride(record.vertex_format) == 0 || (record.index_count > 0 && record.index_size != 1 && record.index_size != 2 && record.index_size != 4) ||
		                                record.size != record.vertex_bytes() + record.index_bytes()))
		{
			Error = "GRF0 record " + std::to_string(i) + " malformed";
			return false;
		}
	}
	return true;
}

void resolve_geometry(e3d1_geometry const &Geometry, std::filesystem::path const &Modelfolder, resolved_geometry &Resolved)
{
	auto const count{Geometry.records.size()};
	Resolved.blocks.assign(count, nullptr);
	Resolved.errors.clear();
	std::vector<pack_entry const *> entries(count, nullptr);
	bool needpack{false};
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const &record{Geometry.records[i]};
		if (record.empty())
			continue;
		if (record.flags & geometry_record::inline_data)
		{
			if (record.offset + record.size > Geometry.inline_size)
				Resolved.errors.push_back("geometry of sub-model " + std::to_string(i) + " lies past the end of the GRI0 chunk");
			else
				Resolved.blocks[i] = Geometry.inline_data + record.offset;
		}
		else
			needpack = true;
	}
	if (false == needpack)
		return;
	if (Geometry.pack.empty())
	{
		Resolved.errors.push_back("geometry kept in a pack, but no pack named");
		return;
	}
	std::string error;
	auto const pack{pack_reader::open(Modelfolder / std::filesystem::u8path(Geometry.pack), error)};
	if (pack == nullptr)
	{
		Resolved.errors.push_back(error);
		return;
	}
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const &record{Geometry.records[i]};
		if (record.empty() || (record.flags & geometry_record::inline_data))
			continue;
		auto const *entry{pack->find(record.hash)};
		if (entry == nullptr || entry->size != record.size)
		{
			Resolved.errors.push_back("geometry block " + record.hash.to_string() + " of sub-model " + std::to_string(i) + " not found in pack \"" + pack->path().generic_string() + "\"");
			continue;
		}
		entries[i] = entry;
	}
	std::vector<unsigned char const *> blocks;
	if (false == pack->read(entries, blocks, Resolved.storage, error))
	{
		Resolved.errors.push_back(error);
		return;
	}
	for (std::size_t i = 0; i < count; ++i)
		if (entries[i] != nullptr)
			Resolved.blocks[i] = blocks[i];
}

} // namespace e3d
