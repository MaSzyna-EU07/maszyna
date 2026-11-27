// the E3D1 model format with shared geometry (model/geometrypack.h): the e3d file cut into geometry blocks, the blocks
// shared through the pack of the vehicle family, and found again byte for byte

#include "stdafx.h"
#include "model/geometrypack.h"
#include "e3d_samples.h"

#include "doctest.h"

#include <filesystem>
#include <fstream>

using namespace e3d_samples;

namespace {

std::filesystem::path scratch_folder(std::string const &Name)
{
	auto const folder{std::filesystem::path(EU07_UNIT_TESTS_SCRATCH) / "e3d1" / Name};
	e3d::pack_reader::close_all();
	std::error_code error;
	std::filesystem::remove_all(folder, error);
	std::filesystem::create_directories(folder);
	return folder;
}

std::string read_file(std::filesystem::path const &Path)
{
	std::ifstream input(Path, std::ios::binary);
	return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

// the geometry of one sub-model, decoded
struct geometry {
	gfx::vertex_array vertices;
	gfx::userdata_array userdata;
	gfx::index_array indices;
};

bool same(geometry const &Left, geometry const &Right)
{
	return e3d::same_bits(Left.vertices, Right.vertices) && Left.indices == Right.indices && Left.userdata.size() == Right.userdata.size();
}

// the geometry of every sub-model of an E3D0 file, the way TModel3d::deserialize() cuts it out of the VNT and IDX chunks
std::vector<geometry> decode_e3d0(std::string const &File)
{
	auto const sub{find_chunk(File, id4('S', 'U', 'B', '0'))};
	std::string vnt, idx;
	int format{0}, indexsize{0};
	for (auto const &c : chunks(File))
	{
		if ((c.id & 0x00FFFFFF) == (id4('V', 'N', 'T', 0) & 0x00FFFFFF))
		{
			vnt = c.content;
			format = static_cast<int>(c.id >> 24) - '0';
		}
		if ((c.id & 0x00FFFFFF) == (id4('I', 'D', 'X', 0) & 0x00FFFFFF))
		{
			idx = c.content;
			indexsize = static_cast<int>(c.id >> 24) - '0';
		}
	}
	std::vector<geometry> result(sub.size() / 256);
	for (std::size_t i = 0; i < result.size(); ++i)
	{
		auto const vertexcount{static_cast<int>(get_u32(sub, i * 256 + 28))};
		auto const vertexoffset{get_u32(sub, i * 256 + 32)};
		auto const indexcount{static_cast<int>(get_u32(sub, i * 256 + 156))};
		auto const indexoffset{get_u32(sub, i * 256 + 160)};
		auto &g{result[i]};
		if (vertexcount > 0)
		{
			g.vertices.resize(vertexcount);
			if (e3d::has_userdata(format))
				g.userdata.resize(vertexcount);
			e3d::decode_vertices(reinterpret_cast<unsigned char const *>(vnt.data()) + vertexoffset * e3d::vertex_stride(format), vertexcount, format, g.vertices.data(), g.userdata.data(), false);
		}
		if (indexcount > 0)
		{
			g.indices.resize(indexcount);
			e3d::decode_indices(reinterpret_cast<unsigned char const *>(idx.data()) + indexoffset * indexsize, indexcount, indexsize, g.indices.data());
		}
	}
	return result;
}

// the geometry of every sub-model of an E3D1 file, found in the file or in its pack
std::vector<geometry> decode_e3d1(std::filesystem::path const &Path, std::vector<std::string> &Errors)
{
	auto const file{read_file(Path)};
	REQUIRE(get_u32(file, 0) == id4('E', '3', 'D', '1'));
	auto const grf{find_chunk(file, id4('G', 'R', 'F', '0'))};
	auto const gri{find_chunk(file, id4('G', 'R', 'I', '0'))};
	e3d::e3d1_geometry layout;
	std::string error;
	REQUIRE(e3d::parse_grf0(reinterpret_cast<unsigned char const *>(grf.data()), grf.size(), layout, error));
	layout.inline_data = reinterpret_cast<unsigned char const *>(gri.data());
	layout.inline_size = gri.size();
	e3d::resolved_geometry resolved;
	e3d::resolve_geometry(layout, Path.parent_path(), resolved);
	Errors = resolved.errors;
	std::vector<geometry> result(layout.records.size());
	for (std::size_t i = 0; i < result.size(); ++i)
	{
		auto const &record{layout.records[i]};
		auto *block{resolved.blocks[i]};
		if (record.empty() || block == nullptr)
			continue;
		auto &g{result[i]};
		g.vertices.resize(record.vertex_count);
		if (e3d::has_userdata(record.vertex_format))
			g.userdata.resize(record.vertex_count);
		e3d::decode_vertices(block, record.vertex_count, record.vertex_format, g.vertices.data(), g.userdata.data(), false);
		g.indices.resize(record.index_count);
		e3d::decode_indices(block + record.vertex_bytes(), record.index_count, record.index_size, g.indices.data());
	}
	return result;
}

e3d::e3d1_content split(std::string const &File)
{
	e3d::e3d1_content content;
	std::string error;
	REQUIRE_MESSAGE(e3d::split_e3d0(reinterpret_cast<unsigned char const *>(File.data()), File.size(), content, error), error);
	return content;
}

// a bogie twice (as two includes of one part give it), a small lamp and the root without geometry
std::vector<part> const twin_bogies{{"root", 0, 0}, {"bogie1", 300, 1}, {"bogie2", 300, 1}, {"lamp", 10, 2}};

} // namespace

TEST_CASE("an E3D1 model keeps one copy of a part it has twice, and gives back the same geometry as E3D0")
{
	auto const folder{scratch_folder("twins")};
	auto const e3d0{model(twin_bogies)};
	auto const content{split(e3d0)};
	e3d::writer_options options;
	options.pack = folder / "geometry.e3dpack";
	auto const stats{e3d::write_e3d1(folder / "twins.e3d", content, options)};
	e3d::flush_pack_writers();

	// the bogie (300 vertices of 48 bytes) goes to the pack once, the lamp (10 vertices) stays in the file
	CHECK(stats.blocks == 3);
	CHECK(stats.inline_blocks == 1);
	CHECK(stats.packed_blocks == 2);
	CHECK(stats.new_blocks == 1);
	CHECK(stats.new_bytes == 300 * 48 + 3 * 298 * 2); // the indices of the bogie fit 2 bytes

	std::vector<std::string> errors;
	auto const loaded{decode_e3d1(folder / "twins.e3d", errors)};
	CHECK(errors.empty());
	auto const original{decode_e3d0(e3d0)};
	REQUIRE(loaded.size() == original.size());
	for (std::size_t i = 0; i < original.size(); ++i)
	{
		CAPTURE(i);
		CHECK(same(loaded[i], original[i]));
	}
	CHECK(loaded[1].vertices.size() == 300);
	CHECK(loaded[3].indices.size() == 24);

	// the rest of the model is carried over as it was
	auto const e3d1{read_file(folder / "twins.e3d")};
	for (auto const id : {id4('S', 'U', 'B', '0'), id4('T', 'R', 'A', '0'), id4('T', 'E', 'X', '0'), id4('N', 'A', 'M', '0')})
		CHECK(find_chunk(e3d1, id) == find_chunk(e3d0, id));
	// and nothing of the E3D0 geometry chunks is left, a loader of E3D0 has nothing it could half read
	for (auto const &c : chunks(e3d1))
	{
		CHECK((c.id & 0x00FFFFFF) != (id4('V', 'N', 'T', 0) & 0x00FFFFFF));
		CHECK((c.id & 0x00FFFFFF) != (id4('I', 'D', 'X', 0) & 0x00FFFFFF));
	}
	CHECK(e3d1.size() < e3d0.size() / 2);
}

TEST_CASE("the models of one family share the blocks of their pack, and are written the same each time")
{
	auto const folder{scratch_folder("family")};
	e3d::writer_options options;
	options.pack = folder / "geometry.e3dpack";
	// a variant of the model with another lamp and the same bogie, in a subfolder
	auto const first{model(twin_bogies)};
	auto const second{model({{"root", 0, 0}, {"bogie1", 300, 1}, {"bogie2", 300, 1}, {"lamp", 10, 5}})};
	e3d::write_e3d1(folder / "a.e3d", split(first), options);
	auto const stats{e3d::write_e3d1(folder / "variant" / "b.e3d", split(second), options)};
	CHECK(stats.packed_blocks == 2);
	CHECK(stats.new_blocks == 0);
	e3d::flush_pack_writers();
	auto const packsize{std::filesystem::file_size(folder / "geometry.e3dpack")};

	// the second model names the pack of the folder above
	std::vector<std::string> errors;
	auto const loaded{decode_e3d1(folder / "variant" / "b.e3d", errors)};
	CHECK(errors.empty());
	auto const original{decode_e3d0(second)};
	for (std::size_t i = 0; i < original.size(); ++i)
		CHECK(same(loaded[i], original[i]));

	// once more into the same pack: the same file, the pack doesn't grow
	auto const before{read_file(folder / "a.e3d")};
	e3d::write_e3d1(folder / "a.e3d", split(first), options);
	e3d::flush_pack_writers();
	CHECK(read_file(folder / "a.e3d") == before);
	CHECK(std::filesystem::file_size(folder / "geometry.e3dpack") == packsize);
}

TEST_CASE("without its pack an E3D1 model says what is missing, and keeps the geometry stored in the file")
{
	auto const folder{scratch_folder("missing")};
	e3d::writer_options options;
	options.pack = folder / "geometry.e3dpack";
	e3d::write_e3d1(folder / "twins.e3d", split(model(twin_bogies)), options);
	e3d::flush_pack_writers();

	SUBCASE("no pack at all")
	{
		e3d::pack_reader::close_all();
		std::filesystem::remove(folder / "geometry.e3dpack");
		std::vector<std::string> errors;
		auto const loaded{decode_e3d1(folder / "twins.e3d", errors)};
		REQUIRE(errors.size() == 1);
		CHECK(errors[0].find("not found") != std::string::npos);
		CHECK(loaded[1].vertices.empty());
		CHECK(loaded[2].vertices.empty());
		CHECK(loaded[3].vertices.size() == 10); // inline
	}
	SUBCASE("a pack without the block")
	{
		e3d::pack_reader::close_all();
		std::filesystem::remove(folder / "geometry.e3dpack");
		e3d::write_e3d1(folder / "other.e3d", split(model({{"root", 0, 0}, {"seat", 300, 7}})), options);
		e3d::flush_pack_writers();
		std::vector<std::string> errors;
		auto const loaded{decode_e3d1(folder / "twins.e3d", errors)};
		CHECK(errors.size() == 2); // both bogies
		CHECK(loaded[1].vertices.empty());
		CHECK(loaded[3].vertices.size() == 10);
	}
	SUBCASE("not a pack")
	{
		e3d::pack_reader::close_all();
		std::ofstream(folder / "geometry.e3dpack", std::ios::binary | std::ios::trunc) << "E3D0 something else entirely, long enough to have a footer";
		std::vector<std::string> errors;
		decode_e3d1(folder / "twins.e3d", errors);
		REQUIRE(errors.size() == 1);
		CHECK(errors[0].find("not a geometry pack") != std::string::npos);
	}
}

TEST_CASE("an E3D0 file with geometry outside its chunks isn't converted")
{
	auto file{model(twin_bogies)};
	// the vertex count of the lamp made to reach past the end of the VNT chunk
	auto const sub{file.find("SUB0")};
	put_at(file, sub + 8 + 3 * 256 + 28, 10000);
	e3d::e3d1_content content;
	std::string error;
	CHECK_FALSE(e3d::split_e3d0(reinterpret_cast<unsigned char const *>(file.data()), file.size(), content, error));
	CHECK(error.find("sub-model 3") != std::string::npos);
}

TEST_CASE("the indices of a block are stored in the narrowest width which holds them")
{
	std::string data;
	gfx::basic_index const narrow[]{0, 1, 255};
	CHECK(e3d::encode_indices(narrow, 3, data) == 1);
	CHECK(data.size() == 3);
	gfx::basic_index const middle[]{0, 256, 65535};
	data.clear();
	CHECK(e3d::encode_indices(middle, 3, data) == 2);
	gfx::basic_index const wide[]{0, 65536};
	data.clear();
	CHECK(e3d::encode_indices(wide, 2, data) == 4);
	gfx::basic_index decoded[2];
	e3d::decode_indices(reinterpret_cast<unsigned char const *>(data.data()), 2, 4, decoded);
	CHECK(decoded[1] == 65536);
}

TEST_CASE("the content hash is MurmurHash3 x64 128")
{
	// the reference values of the original implementation (MurmurHash3_x64_128, seed 0)
	CHECK(e3d::murmur3_128("", 0).low == 0);
	CHECK(e3d::murmur3_128("", 0).high == 0);
	auto const hash{e3d::murmur3_128("The quick brown fox jumps over the lazy dog", 43)};
	CHECK(hash.low == 0xe34bbc7bbc071b6cull);
	CHECK(hash.high == 0x7a433ca9c49a9347ull);
}
