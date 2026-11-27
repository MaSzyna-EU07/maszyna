#pragma once

// small e3d models put together byte by byte, for the tests of the E3D1 format (test_e3d1*.cpp)

#include <bit>
#include <cstdint>
#include <string>
#include <vector>

namespace e3d_samples {

inline void put_u32(std::string &Output, std::uint32_t const Value)
{
	for (int i = 0; i < 4; ++i)
		Output.push_back(static_cast<char>((Value >> (8 * i)) & 0xFF));
}
inline void put_f32(std::string &Output, float const Value)
{
	put_u32(Output, std::bit_cast<std::uint32_t>(Value));
}
inline void put_at(std::string &Output, std::size_t const Offset, std::uint32_t const Value)
{
	for (int i = 0; i < 4; ++i)
		Output[Offset + i] = static_cast<char>((Value >> (8 * i)) & 0xFF);
}
inline std::uint32_t get_u32(std::string const &Data, std::size_t const Offset)
{
	std::uint32_t value{0};
	for (int i = 0; i < 4; ++i)
		value |= std::uint32_t(static_cast<unsigned char>(Data[Offset + i])) << (8 * i);
	return value;
}
constexpr std::uint32_t id4(char a, char b, char c, char d)
{
	return (std::uint32_t(std::uint8_t(d)) << 24) | (std::uint32_t(std::uint8_t(c)) << 16) | (std::uint32_t(std::uint8_t(b)) << 8) | std::uint32_t(std::uint8_t(a));
}
inline std::string chunk(std::uint32_t const Id, std::string const &Content)
{
	std::string output;
	put_u32(output, Id);
	put_u32(output, static_cast<std::uint32_t>(Content.size() + 8));
	return output + Content;
}

// a sub-model of the sample, its geometry made up of a grid
struct part {
	std::string name;
	int vertices{0}; // 0 for none
	int seed{0}; // the same seed gives the same geometry
};

// an expanded (VNT2, 48 bytes per vertex) indexed model of the given parts, siblings under the first one.
// the indices are stored 4 bytes wide whatever their values, as SaveToBinFile() does for big models
inline std::string model(std::vector<part> const &Parts)
{
	std::string sub;
	std::string vnt;
	std::string idx;
	std::string nam;
	int vertexoffset{0};
	int indexoffset{0};
	for (std::size_t i = 0; i < Parts.size(); ++i)
	{
		auto const &p{Parts[i]};
		int const indexcount{p.vertices > 2 ? 3 * (p.vertices - 2) : 0};
		std::string record(256, '\0');
		// next, child: the parts after the first are its children, in a chain
		put_at(record, 0, (i >= 1 && i + 1 < Parts.size()) ? static_cast<std::uint32_t>(i + 1) : 0xFFFFFFFFu);
		put_at(record, 4, (i == 0 && Parts.size() > 1) ? 1u : 0xFFFFFFFFu);
		put_at(record, 8, 4); // GL_TRIANGLES
		put_at(record, 12, static_cast<std::uint32_t>(i)); // name
		put_at(record, 20, 0x10); // opaque
		put_at(record, 24, 0); // transform
		put_at(record, 28, static_cast<std::uint32_t>(p.vertices));
		put_at(record, 32, static_cast<std::uint32_t>(vertexoffset));
		put_at(record, 36, 0); // no texture: colored
		// diffuse, so it's not divided by its multiplier to nothing
		auto const onebits{std::bit_cast<std::uint32_t>(1.0f)};
		for (int c = 0; c < 4; ++c)
			put_at(record, 64 + 4 * c, onebits);
		put_at(record, 156, static_cast<std::uint32_t>(indexcount));
		put_at(record, 160, static_cast<std::uint32_t>(indexoffset));
		put_at(record, 164, onebits); // diffuse multiplier
		sub += record;
		for (int v = 0; v < p.vertices; ++v)
		{
			float const x{static_cast<float>(v % 17) * 0.25f + static_cast<float>(p.seed)};
			float const y{static_cast<float>(v / 17) * 0.5f};
			put_f32(vnt, x);
			put_f32(vnt, y);
			put_f32(vnt, 0.125f * p.seed);
			put_f32(vnt, 0.0f);
			put_f32(vnt, 0.0f);
			put_f32(vnt, 1.0f); // normal
			put_f32(vnt, x / 4.0f);
			put_f32(vnt, y / 8.0f); // uv
			put_f32(vnt, 1.0f);
			put_f32(vnt, 0.0f);
			put_f32(vnt, 0.0f);
			put_f32(vnt, 1.0f); // tangent
		}
		for (int t = 0; t < p.vertices - 2; ++t)
		{
			put_u32(idx, static_cast<std::uint32_t>(t));
			put_u32(idx, static_cast<std::uint32_t>(t + 1 + (t % 2)));
			put_u32(idx, static_cast<std::uint32_t>(t + 2 - (t % 2)));
		}
		vertexoffset += p.vertices;
		indexoffset += indexcount;
		nam += p.name;
		nam.push_back('\0');
	}
	std::string tra;
	for (int r = 0; r < 4; ++r)
		for (int c = 0; c < 4; ++c)
			put_f32(tra, r == c ? 1.0f : 0.0f);
	std::string tex(1, '\0');
	std::string content{chunk(id4('S', 'U', 'B', '0'), sub) + chunk(id4('T', 'R', 'A', '0'), tra) + chunk(id4('I', 'D', 'X', '4'), idx) + chunk(id4('V', 'N', 'T', '2'), vnt) +
	                    chunk(id4('T', 'E', 'X', '0'), tex) + chunk(id4('N', 'A', 'M', '0'), nam)};
	return chunk(id4('E', '3', 'D', '0'), content);
}

// the chunks of a model file, id and content
struct chunk_view {
	std::uint32_t id;
	std::string content;
};
inline std::vector<chunk_view> chunks(std::string const &File)
{
	std::vector<chunk_view> result;
	std::size_t const end{std::min<std::size_t>(File.size(), get_u32(File, 4))};
	std::size_t position{8};
	while (position + 8 <= end)
	{
		auto const size{get_u32(File, position + 4)};
		result.emplace_back(get_u32(File, position), File.substr(position + 8, size - 8));
		position += size;
	}
	return result;
}
inline std::string find_chunk(std::string const &File, std::uint32_t const Id)
{
	for (auto const &c : chunks(File))
		if (c.id == Id)
			return c.content;
	return {};
}

} // namespace e3d_samples
