/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "rendering/geometrybank.h"
#include "interfaces/IMaterial.h"
#include "interfaces/ITexture.h"
#include "utilities/Globals.h"

struct lighting_data;

namespace gl
{
class program;
}

namespace gfx
{
// material painted over the heightmap terrain, and the size (metres) its textures are repeated at; 0: the size given by the material, or 8 m
struct terrain_layer
{
	material_handle material{null_handle};
	float size{0.f};
};
} // namespace gfx

class gfx_renderer {

public:
// types
// constructors
// destructor
    virtual ~gfx_renderer() {}
// methods
    virtual auto Init( GLFWwindow *Window ) -> bool = 0;
    virtual bool AddViewport(const global_settings::extraviewport_config &conf) = 0;
    virtual void Shutdown() = 0;
    // main draw call. returns false on error
    virtual auto Render() -> bool = 0;
    virtual void SwapBuffers() = 0;
    virtual auto Framerate() -> float = 0;
    // geometry methods
    // NOTE: hands-on geometry management is exposed as a temporary measure; ultimately all visualization data should be generated/handled automatically by the renderer itself
    // creates a new geometry bank. returns: handle to the bank or NULL
    virtual auto Create_Bank() -> gfx::geometrybank_handle = 0;
    // creates a new indexed geometry chunk of specified type from supplied data, in specified bank. returns: handle to the chunk or NULL
    virtual auto Insert(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type) -> gfx::geometry_handle = 0;
    // creates a new geometry chunk of specified type from supplied data, in specified bank. returns: handle to the chunk or NULL
    virtual auto Insert(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type) -> gfx::geometry_handle = 0;
    // replaces data of specified chunk with the supplied vertex data, starting from specified offset
    virtual auto Replace(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type, const std::size_t Offset = 0) -> bool = 0;
    // adds supplied vertex data at the end of specified chunk
    virtual auto Append(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type) -> bool = 0;
    // provides direct access to index data of specfied chunk
    virtual auto Indices( gfx::geometry_handle const &Geometry ) const->gfx::index_array const & = 0;
	// provides direct access to vertex data of specfied chunk
	virtual auto Vertices( gfx::geometry_handle const &Geometry ) const ->gfx::vertex_array const & = 0;
	// provides direct access to vertex user data of specfied chunk
	virtual auto UserData( gfx::geometry_handle const &Geometry ) const ->gfx::userdata_array const & = 0;
    // material methods
    virtual auto Fetch_Material( std::string const &Filename, bool const Loadnow = true ) -> material_handle = 0;
    virtual void Bind_Material( material_handle const Material, TSubModel const *sm = nullptr, lighting_data const *lighting = nullptr ) = 0;
    virtual auto Material( material_handle const Material ) const -> IMaterial const * = 0;
    // heightmap terrain: material of a chunk blending the materials of up to 8 layers by their weights.
    // Weights: (Samples + 1)^2 values per layer, planar, rows of growing z, the weights of a sample adding up to 255; null with a single layer.
    // Placement: corner of the chunk (x, z) in the texture coordinates of its mesh, and the size of the chunk.
    // Reuse: a material made by this call before, to be filled anew, or null_handle.
    // returns: the material. backends which can't blend give back the material of the layer covering the most
    virtual auto Terrain_Material( material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement ) -> material_handle;
    // gives back a material made by Terrain_Material, once its chunk is gone
    virtual void Terrain_Release( material_handle const Material ) {}
    // shader methods
    virtual auto Fetch_Shader( std::string const &name ) -> std::shared_ptr<gl::program> = 0;
    // texture methods
    virtual auto Fetch_Texture( std::string const &Filename, bool const Loadnow = true, GLint format_hint = GL_SRGB_ALPHA ) -> texture_handle = 0;
    virtual void Bind_Texture( texture_handle const Texture ) = 0;
    virtual void Bind_Texture( std::size_t const Unit, texture_handle const Texture ) = 0;
    virtual auto Texture( texture_handle const Texture ) -> ITexture & = 0;
    virtual auto Texture( texture_handle const Texture ) const -> ITexture const & = 0;
    // utility methods
    virtual void Pick_Control_Callback( std::function<void( TSubModel const *, const glm::vec2 )> Callback ) = 0;
    virtual void Pick_Node_Callback( std::function<void( scene::basic_node * )> Callback ) = 0;
    virtual auto Pick_Control() const -> TSubModel const * = 0;
    virtual auto Pick_Node() const -> scene::basic_node const * = 0;

    virtual auto Mouse_Position() const -> glm::dvec3 = 0;
    virtual auto Mouse_Hit() const -> bool { return true; }
    // editor helpers: matrices/position of the most recent color pass camera.
    // the view matrix is camera-relative (rotation only, camera at origin), matching the
    // camera-relative rendering used by the engine; build object matrices relative to Camera_Position().
    // default implementations return identity so backends without a usable camera still compile.
    virtual auto Camera_View_Matrix() const -> glm::mat4 { return glm::mat4( 1.f ); }
    virtual auto Camera_Projection_Matrix() const -> glm::mat4 { return glm::mat4( 1.f ); }
    virtual auto Camera_Position() const -> glm::dvec3 { return glm::dvec3( 0.0 ); }
    // maintenance methods
    virtual void Update( double const Deltatime ) = 0;
    virtual void Update_Pick_Control() = 0;
    virtual void Update_Pick_Node() = 0;
    virtual auto Update_Mouse_Position() -> glm::dvec3 = 0;
    virtual bool Debug_Ui_State(std::optional<bool>) = 0;
    // debug methods
    virtual auto info_times() const -> std::string const & = 0;
    virtual auto info_stats() const -> std::string const & = 0;
    // imgui renderer
	  virtual class imgui_renderer *GetImguiRenderer() = 0;
	  virtual void MakeScreenshot() = 0;
    // draws the model of a scenery instance on its own into a square image of Size pixels on white background, the model
    // seen in perspective from the front, side and above and filling the image less Margin pixels at each edge.
    // Image receives rgb rows, top row first. returns false if the backend can't make previews or there's nothing to draw
    virtual auto Render_Preview( TAnimModel *Instance, int const Size, int const Margin, bool const Shadows, std::vector<std::uint8_t> &Image ) -> bool { return false; }
};

class gfx_renderer_factory
{
public:
    using create_method = std::unique_ptr<gfx_renderer>(*)();

    bool register_backend(const std::string &backend, create_method func);
    std::unique_ptr<gfx_renderer> create(const std::string &name);

    static gfx_renderer_factory* get_instance();

private:
    std::unordered_map<std::string, create_method> backends;
    static gfx_renderer_factory *instance;
};

class imgui_renderer
{
  public:
	virtual bool Init() = 0;
	virtual void Shutdown() = 0;
	virtual void BeginFrame() = 0;
	virtual void Render() = 0;
	// images of the user interface itself (e.g. the previews of the node bank), apart from the textures of the scene: rgba, rows
	// from the top, in srgb. returns: id for ImGui::Image, 0 if the renderer can't make them
	virtual std::uint64_t Create_Image(std::uint8_t const *Rgba, int const Width, int const Height)
	{
		return 0;
	}
	virtual void Release_Image(std::uint64_t const Image) {}
};

extern std::unique_ptr<gfx_renderer> GfxRenderer;

//---------------------------------------------------------------------------
