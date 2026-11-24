// stand-ins for the parts of the engine which the tracks reach for (world/Track.cpp and the sources linked with it, see
// CMakeLists.txt), so test_track_text.cpp can read a track from its text and write it back with the game's own TTrack::Load()
// and export_as_text(), with no scene. what reading and writing a track does use works: the log, the materials kept by their
// names (unit_tests_material_renderer()). the rest is the simulation of a scene the tests don't make: the events, the vehicles
// on the tracks, the regions, the geometry of the rails. it's never reached by reading or writing a track, and a test which
// strays into it stops with the name of what it reached, rather than passing on a stand-in which does nothing

#include "stdafx.h"
#include "world/Track.h"
#include "editor/editorTrack.hpp"
#include "world/TrackRails.h"
#include "world/TrackSleepers.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "model/AnimModel.h"
#include "model/MdlMngr.h"
#include "vehicle/DynObj.h"
#include "vehicle/Driver.h"
#include "scene/scene.h"
#include "simulation/simulation.h"
#include "rendering/renderer.h"
#include "input/messaging.h"
#include "utilities/Logs.h"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

[[noreturn]] void absent(char const *What) {
	std::cerr << "unit tests: " << What << " isn't a part of the tests, see tests/unit/track_stubs.cpp\n";
	std::abort();
}

// a material known by its name alone
struct named_material final : IMaterial {
	explicit named_material(std::string Name) : name{std::move(Name)} {}
	void finalize(bool) override { /* nothing to load */ }
	bool update() override { return false; }
	float get_or_guess_opacity() const override { return 1.f; }
	bool is_translucent() const override { return false; }
	glm::vec2 GetSize() const override { return {-1.f, -1.f}; }
	std::string GetName() const override { return name; }
	std::optional<float> GetSelfillum() const override { return std::nullopt; }
	int GetShadowRank() const override { return 0; }
	texture_handle GetTexture(int) const override { return null_handle; }
	std::string name;
};

// a renderer which draws nothing and keeps the materials asked for by their names, as they're given: the game's material
// manager reads the files and names them after where it found them, which takes a scenery and its textures
class material_renderer final : public gfx_renderer {
  public:
	auto Fetch_Material(std::string const &Filename, bool const) -> material_handle override {
		for (std::size_t index = 0; index < m_materials.size(); ++index) {
			if (m_materials[index].name == Filename) {
				return static_cast<material_handle>(index + 1);
			}
		}
		m_materials.emplace_back(Filename);
		return static_cast<material_handle>(m_materials.size());
	}
	auto Material(material_handle const Material) const -> IMaterial const * override {
		return Material > 0 && static_cast<std::size_t>(Material) <= m_materials.size() ? &m_materials[Material - 1] : IMaterial::null_material();
	}

	auto Init(GLFWwindow *) -> bool override { absent("renderer: Init"); }
	bool AddViewport(global_settings::extraviewport_config const &) override { absent("renderer: AddViewport"); }
	void Shutdown() override { absent("renderer: Shutdown"); }
	auto Render() -> bool override { absent("renderer: Render"); }
	void SwapBuffers() override { absent("renderer: SwapBuffers"); }
	auto Framerate() -> float override { absent("renderer: Framerate"); }
	auto Create_Bank() -> gfx::geometrybank_handle override { absent("renderer: Create_Bank"); }
	auto Insert(gfx::index_array &, gfx::vertex_array &, gfx::userdata_array &, gfx::geometrybank_handle const &, int const) -> gfx::geometry_handle override { absent("renderer: Insert"); }
	auto Insert(gfx::vertex_array &, gfx::userdata_array &, gfx::geometrybank_handle const &, int const) -> gfx::geometry_handle override { absent("renderer: Insert"); }
	auto Replace(gfx::vertex_array &, gfx::userdata_array &, gfx::geometry_handle const &, int const, std::size_t const) -> bool override { absent("renderer: Replace"); }
	auto Append(gfx::vertex_array &, gfx::userdata_array &, gfx::geometry_handle const &, int const) -> bool override { absent("renderer: Append"); }
	auto Indices(gfx::geometry_handle const &) const -> gfx::index_array const & override { absent("renderer: Indices"); }
	auto Vertices(gfx::geometry_handle const &) const -> gfx::vertex_array const & override { absent("renderer: Vertices"); }
	auto UserData(gfx::geometry_handle const &) const -> gfx::userdata_array const & override { absent("renderer: UserData"); }
	void Bind_Material(material_handle const, TSubModel const *, lighting_data const *) override { absent("renderer: Bind_Material"); }
	auto Fetch_Shader(std::string const &) -> std::shared_ptr<gl::program> override { absent("renderer: Fetch_Shader"); }
	auto Fetch_Texture(std::string const &, bool const, GLint) -> texture_handle override { absent("renderer: Fetch_Texture"); }
	void Bind_Texture(texture_handle const) override { absent("renderer: Bind_Texture"); }
	void Bind_Texture(std::size_t const, texture_handle const) override { absent("renderer: Bind_Texture"); }
	auto Texture(texture_handle const) -> ITexture & override { absent("renderer: Texture"); }
	auto Texture(texture_handle const) const -> ITexture const & override { absent("renderer: Texture"); }
	void Pick_Control_Callback(std::function<void(TSubModel const *, const glm::vec2)>) override { absent("renderer: Pick_Control_Callback"); }
	void Pick_Node_Callback(std::function<void(scene::basic_node *)>) override { absent("renderer: Pick_Node_Callback"); }
	auto Pick_Control() const -> TSubModel const * override { absent("renderer: Pick_Control"); }
	auto Pick_Node() const -> scene::basic_node const * override { absent("renderer: Pick_Node"); }
	auto Mouse_Position() const -> glm::dvec3 override { absent("renderer: Mouse_Position"); }
	void Update(double const) override { absent("renderer: Update"); }
	void Update_Pick_Control() override { absent("renderer: Update_Pick_Control"); }
	void Update_Pick_Node() override { absent("renderer: Update_Pick_Node"); }
	auto Update_Mouse_Position() -> glm::dvec3 override { absent("renderer: Update_Mouse_Position"); }
	bool Debug_Ui_State(std::optional<bool>) override { absent("renderer: Debug_Ui_State"); }
	auto info_times() const -> std::string const & override { absent("renderer: info_times"); }
	auto info_stats() const -> std::string const & override { absent("renderer: info_stats"); }
	imgui_renderer *GetImguiRenderer() override { absent("renderer: GetImguiRenderer"); }
	void MakeScreenshot() override { absent("renderer: MakeScreenshot"); }

  private:
	std::vector<named_material> m_materials;
};

} // namespace

// for the tests: puts a renderer in place which keeps the materials by their names and draws nothing (true), or takes it away (false)
void unit_tests_material_renderer(bool const Enabled) {
	GfxRenderer = Enabled ? std::make_unique<material_renderer>() : nullptr;
}

// the renderer -----------------------------------------------------------------------------------------------------------

std::unique_ptr<gfx_renderer> GfxRenderer; // NOSONAR the engine's extern global, rendering/renderer.h

// the key function of the class for GCC and Clang, which put its vtable where it's defined
auto gfx_renderer::Terrain_Material(material_handle const, std::vector<gfx::terrain_layer> const &, int const, std::uint8_t const *, glm::vec3 const &) -> material_handle { // NOSONAR signature from rendering/renderer.h
	absent("gfx_renderer::Terrain_Material");
}

// the log --------------------------------------------------------------------------------------------------------------

void Error(std::string const &asMessage, bool const) { // NOSONAR signature from utilities/Logs.h
	ErrorLog(asMessage);
}

// the simulation of the scene ------------------------------------------------------------------------------------------

namespace simulation {

event_manager Events; // NOSONAR the engine's extern global, simulation/simulation.h
memory_table Memory; // NOSONAR the engine's extern global, simulation/simulation.h
path_table Paths; // NOSONAR the engine's extern global, simulation/simulation.h
instance_table Instances; // NOSONAR the engine's extern global, simulation/simulation.h
scene::basic_region *Region{nullptr}; // NOSONAR the engine's extern global, simulation/simulation.h

} // namespace simulation

event_manager::~event_manager() = default; // the tests put no events in it

basic_event *event_manager::FindEvent(std::string const &) { absent("event_manager::FindEvent"); }
bool event_manager::AddToQuery(basic_event *, TDynamicObject const *, double) { absent("event_manager::AddToQuery"); }

namespace scene {

bool basic_cell::RaTrackAnimAdd(TTrack *) { absent("basic_cell::RaTrackAnimAdd"); }
void basic_cell::insert(TTrack *) { absent("basic_cell::insert"); }
basic_cell &basic_section::cell(glm::dvec3 const &, glm::ivec2 const &) { absent("basic_section::cell"); }
bool basic_region::point_inside(glm::dvec3 const &) { absent("basic_region::point_inside"); }
basic_section &basic_region::section(glm::dvec3 const &) { absent("basic_region::section"); }
std::tuple<TTrack *, int> basic_region::find_path(glm::dvec3 const &, TTrack const *) { absent("basic_region::find_path"); }
void basic_region::TrackJoin(TTrack *) { absent("basic_region::TrackJoin"); }

} // namespace scene

void multiplayer::WyslijString(std::string const &, int) { absent("multiplayer::WyslijString"); }

// the vehicles and the models ------------------------------------------------------------------------------------------

void TDynamicObject::Move(double) { absent("TDynamicObject::Move"); }
void TDynamicObject::RenderSounds() { absent("TDynamicObject::RenderSounds"); }
void TDynamicObject::RadioStop() { absent("TDynamicObject::RadioStop"); }
void TDynamicObject::OverheadTrack(float) { absent("TDynamicObject::OverheadTrack"); }
std::string const &TController::TrainName() const { absent("TController::TrainName"); }
bool TController::PutCommand(std::string, double, double, glm::dvec3 const *, TStopReason) { absent("TController::PutCommand"); }
std::shared_ptr<TAnimContainer> TAnimModel::GetContainer(std::string const &) { absent("TAnimModel::GetContainer"); }
void TAnimContainer::EventAssign(basic_event *) { absent("TAnimContainer::EventAssign"); }
TModel3d *TModelsManager::GetModel(std::string const &, bool const, bool const, int) { absent("TModelsManager::GetModel"); }

// the buckling of the rails by the heat, world/TrackBuckle.cpp: the tests read the tracks with it off (Global.track_buckling)

track_buckle::shape TTrack::resolve_buckle(track_buckle::declared const &, std::uint32_t const) const { absent("TTrack::resolve_buckle"); }
void TTrack::buckle_pass_enter(TDynamicObject const &) { absent("TTrack::buckle_pass_enter"); }
void TTrack::buckle_pass_leave() { absent("TTrack::buckle_pass_leave"); }

// the geometry of the rails and the sleepers, world/Rail*.cpp and world/Sleeper*.cpp ------------------------------------

namespace track_rails {

// what's kept of the rails around: nothing, with no rails laid
void forget() { /* nothing kept */ }
rail_size measure_rail(gfx::vertex_array const &) { absent("track_rails::measure_rail"); }
int blade_steps(TTrack const &, int const) { absent("track_rails::blade_steps"); }
double blade_length(TTrack const &, int const) { absent("track_rails::blade_length"); }
TTrack const *runon_track(TTrack const &, int const) { absent("track_rails::runon_track"); }
double blade_runon(TTrack const &, int const) { absent("track_rails::blade_runon"); }
material_handle cut_material(material_handle const) { absent("track_rails::cut_material"); }
void cap_loft(gfx::vertex_array const &, std::size_t const, bool const, bool const, gfx::vertex_array &) { absent("track_rails::cap_loft"); }
gfx::userdata_array worn_userdata(std::size_t const, bool const) { absent("track_rails::worn_userdata"); }
void cut_faces(gfx::vertex_array const &, std::size_t const, bool const, bool const, gfx::geometrybank_handle const &, TTrack::geometryhandle_sequence &, bool const) { absent("track_rails::cut_faces"); }
reused_chunks::reused_chunks(chunk_pool *) { absent("track_rails::reused_chunks"); }
reused_chunks::~reused_chunks() = default;
gfx::geometry_handle insert_chunk(gfx::vertex_array &, gfx::userdata_array &, gfx::geometrybank_handle const &, int const) { absent("track_rails::insert_chunk"); }

} // namespace track_rails

track_rails::switch_blade const &TTrack::blade(int const) const { absent("TTrack::blade"); }
double TTrack::blades_throw() const { absent("TTrack::blades_throw"); }
track_rails::rail_layout TTrack::plan_rails(bool const, bool const) { absent("TTrack::plan_rails"); }
TTrack::rail_drawing TTrack::make_rail_drawing(gfx::geometrybank_handle const &) const { absent("TTrack::make_rail_drawing"); }
bool TTrack::lay_rail(track_rails::rail_layout const &, rail_drawing const &, gfx::vertex_array const &, track_rails::rail_course const &, geometryhandle_sequence &, geometryhandle_sequence &) { absent("TTrack::lay_rail"); }

namespace track_sleepers {

bool lays(TTrack const &) { absent("track_sleepers::lays"); }
void unsettle_around(TTrack const &) { absent("track_sleepers::unsettle_around"); }

} // namespace track_sleepers

// the track editor, editor/editorTrack.cpp: the tests take the rolls of the joints of editor/editorTrackShape.cpp alone ----------------

editor_track::point_ref editor_track::end_of(point_ref const &) { absent("editor_track::end_of"); }
glm::dvec3 editor_track::point_position(TTrack const &, point_ref const &) { absent("editor_track::point_position"); }
bool editor_track::is_connected(TTrack const &, point_ref const &) { absent("editor_track::is_connected"); }
bool editor_track::is_supported(TTrack const &) { absent("editor_track::is_supported"); }
std::vector<std::pair<TTrack *, editor_track::point_ref>> editor_track::connected_points(TTrack const &, glm::dvec3 const &) { absent("editor_track::connected_points"); }
editor_track::state editor_track::capture(TTrack const &) { absent("editor_track::capture"); }
