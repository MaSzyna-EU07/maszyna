// TModel3d with the E3D1 format: a model loaded from E3D1 and its pack has the geometry of the same model loaded from
// E3D0, sub-model by sub-model. the geometry goes to a renderer which only keeps it, as the null renderer of the game

#include "stdafx.h"
#include "model/Model3d.h"
#include "model/geometrypack.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "e3d_samples.h"

#include "doctest.h"

#include <filesystem>
#include <fstream>

using namespace e3d_samples;

namespace {

// keeps the geometry handed to it, does nothing else
class keeping_renderer final : public gfx_renderer
{
  public:
	bool Init(GLFWwindow *) override { return true; }
	bool AddViewport(const global_settings::extraviewport_config &) override { return false; }
	void Shutdown() override {}
	bool Render() override { return true; }
	void SwapBuffers() override {}
	float Framerate() override { return 10.0f; }
	gfx::geometrybank_handle Create_Bank() override { return {1, 0}; }
	gfx::geometry_handle Insert(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &, int const) override
	{
		m_chunks.push_back({});
		m_chunks.back().indices.swap(Indices);
		m_chunks.back().vertices.swap(Vertices);
		m_chunks.back().userdata.swap(Userdata);
		return {1, static_cast<std::uint32_t>(m_chunks.size())};
	}
	gfx::geometry_handle Insert(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Bank, int const Type) override
	{
		gfx::index_array none;
		return Insert(none, Vertices, Userdata, Bank, Type);
	}
	bool Replace(gfx::vertex_array &, gfx::userdata_array &, gfx::geometry_handle const &, int const, const std::size_t) override { return false; }
	bool Append(gfx::vertex_array &, gfx::userdata_array &, gfx::geometry_handle const &, int const) override { return false; }
	gfx::index_array const &Indices(gfx::geometry_handle const &Geometry) const override { return m_chunks.at(Geometry.chunk - 1).indices; }
	gfx::vertex_array const &Vertices(gfx::geometry_handle const &Geometry) const override { return m_chunks.at(Geometry.chunk - 1).vertices; }
	gfx::userdata_array const &UserData(gfx::geometry_handle const &Geometry) const override { return m_chunks.at(Geometry.chunk - 1).userdata; }
	material_handle Fetch_Material(std::string const &, bool const) override { return null_handle; }
	void Bind_Material(material_handle const, TSubModel const *, lighting_data const *) override {}
	IMaterial const *Material(material_handle const) const override { return IMaterial::null_material(); }
	std::shared_ptr<gfx::shader_program> Fetch_Shader(std::string const &) override { return nullptr; }
	texture_handle Fetch_Texture(std::string const &, bool const, GLint) override { return null_handle; }
	void Bind_Texture(texture_handle const) override {}
	void Bind_Texture(std::size_t const, texture_handle const) override {}
	ITexture &Texture(texture_handle const) override { return *ITexture::null_texture(); }
	ITexture const &Texture(texture_handle const) const override { return *ITexture::null_texture(); }
	void Pick_Control_Callback(std::function<void(TSubModel const *, const glm::vec2)>) override {}
	void Pick_Node_Callback(std::function<void(scene::basic_node *)>) override {}
	TSubModel const *Pick_Control() const override { return nullptr; }
	scene::basic_node const *Pick_Node() const override { return nullptr; }
	glm::dvec3 Mouse_Position() const override { return {}; }
	void Update(double const) override {}
	void Update_Pick_Control() override {}
	void Update_Pick_Node() override {}
	glm::dvec3 Update_Mouse_Position() override { return {}; }
	bool Debug_Ui_State(std::optional<bool>) override { return false; }
	std::string const &info_times() const override { return m_empty; }
	std::string const &info_stats() const override { return m_empty; }
	imgui_renderer *GetImguiRenderer() override { return nullptr; }
	void MakeScreenshot() override {}

  private:
	struct chunk {
		gfx::index_array indices;
		gfx::vertex_array vertices;
		gfx::userdata_array userdata;
	};
	std::vector<chunk> m_chunks;
	std::string m_empty;
};

std::filesystem::path scratch_folder(std::string const &Name)
{
	auto const folder{std::filesystem::path(EU07_UNIT_TESTS_SCRATCH) / "e3d1_model" / Name};
	e3d::pack_reader::close_all();
	std::error_code error;
	std::filesystem::remove_all(folder, error);
	std::filesystem::create_directories(folder);
	return folder;
}

void write_file(std::filesystem::path const &Path, std::string const &Data)
{
	std::ofstream output(Path, std::ios::binary | std::ios::trunc);
	output.write(Data.data(), static_cast<std::streamsize>(Data.size()));
}

std::string read_file(std::filesystem::path const &Path)
{
	std::ifstream input(Path, std::ios::binary);
	return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::string model_name(std::filesystem::path const &File)
{
	auto name{File.generic_string()};
	return name.substr(0, name.size() - 4);
}

// the renderer in place for a test, the one before put back after it
struct renderer_scope {
	renderer_scope() : previous(std::move(GfxRenderer)) { GfxRenderer = std::make_unique<keeping_renderer>(); }
	~renderer_scope() { GfxRenderer = std::move(previous); }
	std::unique_ptr<gfx_renderer> previous;
};

gfx::vertex_array const &vertices_of(TSubModel const &Submodel)
{
	static gfx::vertex_array const none;
	return Submodel.m_geometry.handle != null_handle ? GfxRenderer->Vertices(Submodel.m_geometry.handle) : none;
}
gfx::index_array const &indices_of(TSubModel const &Submodel)
{
	static gfx::index_array const none;
	return Submodel.m_geometry.handle != null_handle ? GfxRenderer->Indices(Submodel.m_geometry.handle) : none;
}

bool same_vertices(gfx::vertex_array const &Left, gfx::vertex_array const &Right)
{
	return e3d::same_bits(Left, Right);
}

std::vector<part> const twin_bogies{{"root", 0, 0}, {"bogie1", 300, 1}, {"bogie2", 300, 1}, {"lamp", 10, 2}};

// E3D0 of the sample and its E3D1 conversion with the pack, in the folder given
void write_both(std::filesystem::path const &Folder)
{
	auto const e3d0{model(twin_bogies)};
	write_file(Folder / "twins_e3d0.e3d", e3d0);
	e3d::e3d1_content content;
	std::string error;
	REQUIRE(e3d::split_e3d0(reinterpret_cast<unsigned char const *>(e3d0.data()), e3d0.size(), content, error));
	e3d::writer_options options;
	options.pack = Folder / "geometry.e3dpack";
	e3d::write_e3d1(Folder / "twins_e3d1.e3d", content, options);
	e3d::flush_pack_writers();
}

} // namespace

TEST_CASE("TModel3d loads the same geometry from E3D1 and its pack as from E3D0")
{
	renderer_scope renderer;
	auto const folder{scratch_folder("twins")};
	write_both(folder);

	TModel3d e3d0;
	TModel3d e3d1;
	REQUIRE(e3d0.LoadFromFile(model_name(folder / "twins_e3d0.e3d"), true));
	REQUIRE(e3d1.LoadFromFile(model_name(folder / "twins_e3d1.e3d"), true));
	REQUIRE(e3d0.SubModelsCount() == 4);
	REQUIRE(e3d1.SubModelsCount() == 4);
	CHECK(e3d1.m_vertexcount == e3d0.m_vertexcount);
	CHECK(e3d1.m_indexcount == e3d0.m_indexcount);
	for (int i = 0; i < 4; ++i)
	{
		CAPTURE(i);
		auto const &left{e3d0.Root[i]};
		auto const &right{e3d1.Root[i]};
		CHECK(right.pName == left.pName);
		CHECK(right.m_geometry.vertex_count == left.m_geometry.vertex_count);
		CHECK(right.m_geometry.index_count == left.m_geometry.index_count);
		CHECK(same_vertices(vertices_of(right), vertices_of(left)));
		CHECK(indices_of(right) == indices_of(left));
	}
	// both copies of the bogie came from the one block of the pack
	CHECK(vertices_of(e3d1.Root[1]).size() == 300);
	CHECK(same_vertices(vertices_of(e3d1.Root[1]), vertices_of(e3d1.Root[2])));
	CHECK(indices_of(e3d1.Root[3]).size() == 24);
}

TEST_CASE("an E3D1 model without its pack loads with the reason logged, without the geometry kept there")
{
	renderer_scope renderer;
	auto const folder{scratch_folder("nopack")};
	write_both(folder);
	e3d::pack_reader::close_all();
	std::filesystem::remove(folder / "geometry.e3dpack");

	TModel3d e3d1;
	// the model is there, the bogies aren't: no crash, no geometry from elsewhere
	CHECK(e3d1.LoadFromFile(model_name(folder / "twins_e3d1.e3d"), true));
	REQUIRE(e3d1.SubModelsCount() == 4);
	CHECK(vertices_of(e3d1.Root[1]).empty());
	CHECK(vertices_of(e3d1.Root[2]).empty());
	CHECK(e3d1.Root[1].m_geometry.vertex_count == 0);
	CHECK(vertices_of(e3d1.Root[3]).size() == 10);
}

TEST_CASE("a loader which knows only E3D0 turns E3D1 away at the main chunk")
{
	renderer_scope renderer;
	auto const folder{scratch_folder("unknown")};
	write_both(folder);
	// the E3D1 file with the main chunk of a format yet to come takes the path an E3D0 only loader takes for E3D1:
	// "unknown main chunk", no sub-models, a model which failed to load
	auto file{read_file(folder / "twins_e3d1.e3d")};
	file[3] = '9';
	write_file(folder / "future.e3d", file);
	TModel3d model;
	CHECK_FALSE(model.LoadFromFile(model_name(folder / "future.e3d"), true));
	CHECK(model.SubModelsCount() == 0);
	CHECK(model.GetSMRoot() == nullptr);
}

TEST_CASE("SaveToBinFile stores a model as E3D1 with bit 32 of convertmodels, and it loads back the same")
{
	renderer_scope renderer;
	auto const folder{scratch_folder("save")};
	write_both(folder);
	auto const flags{Global.iConvertModels};

	TModel3d original;
	REQUIRE(original.LoadFromFile(model_name(folder / "twins_e3d0.e3d"), true));
	Global.iConvertModels = 8; // expanded vertices, as the sample
	original.SaveToBinFile((folder / "saved_e3d0.e3d").generic_string());
	Global.iConvertModels = 8 | 32;
	original.SaveToBinFile((folder / "out" / "saved_e3d1.e3d").generic_string(), (folder / "out" / "geometry.e3dpack").generic_string());
	Global.iConvertModels = flags;

	auto const saved{read_file(folder / "out" / "saved_e3d1.e3d")};
	REQUIRE(saved.size() > 8);
	CHECK(get_u32(saved, 0) == id4('E', '3', 'D', '1'));
	CHECK(std::filesystem::exists(folder / "out" / "geometry.e3dpack"));

	TModel3d e3d0;
	TModel3d e3d1;
	REQUIRE(e3d0.LoadFromFile(model_name(folder / "saved_e3d0.e3d"), true));
	REQUIRE(e3d1.LoadFromFile(model_name(folder / "out" / "saved_e3d1.e3d"), true));
	REQUIRE(e3d1.SubModelsCount() == e3d0.SubModelsCount());
	for (int i = 0; i < e3d0.SubModelsCount(); ++i)
	{
		CAPTURE(i);
		CHECK(same_vertices(vertices_of(e3d1.Root[i]), vertices_of(e3d0.Root[i])));
		CHECK(indices_of(e3d1.Root[i]) == indices_of(e3d0.Root[i]));
	}
}
