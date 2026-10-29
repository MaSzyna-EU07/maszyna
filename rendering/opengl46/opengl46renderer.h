/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "rendering/renderer.h"
#include "rendering/openglcamera.h"
#include "rendering/opengl46/opengl46light.h"
#include "rendering/opengl46/opengl46particles.h"
#include "rendering/opengl46/opengl46gpuparticles.h"
#include "rendering/opengl46/opengl46water.h"
#include "rendering/opengl46/opengl46skydome.h"
#include "rendering/opengl46/opengl46precipitation.h"
#include "simulation/simulationenvironment.h"
#include "scene/scene.h"
#include "world/MemCell.h"
#include "world/TrackSleepers.h"
#include "rendering/lightarray.h"
#include "vr/vr_interface.h"
#include "gl/ubo.h"
#include "gl/ssbo.h"
#include "gl/streambuffer.h"
#include "gl/cluster_grid.h"
#include "gl/framebuffer.h"
#include "gl/renderbuffer.h"
#include "gl/postfx.h"
#include "gl/shader.h"
#include "gl/cubemap.h"
#include "gl/glsl_common.h"
#include "gl/pbo.h"
#include "gl/query.h"
#include "rendering/opengl46/opengl46terrain.h"
#include "rendering/opengl46/opengl46grass.h"
#include "rendering/opengl46/opengl46sky.h"
#include "rendering/opengl46/opengl46weather.h"
#include "rendering/opengl46/opengl46exposure.h"
#include "rendering/opengl46/opengl46ao.h"
#include "rendering/common/locallights.h"
#include "rendering/opengl46/opengl46localshadows.h"
#include "rendering/opengl46/opengl46cabprobe.h"
#include "rendering/opengl46/opengl46skyocclusion.h"
#include "rendering/opengl46/opengl46instancecull.h"
#include "rendering/opengl46/opengl46hiz.h"
#include "rendering/opengl46/opengl46shadowfit.h"
#include "rendering/common/scenewalk.h"
#include "rendering/common/temporalaa.h"
#include "rendering/common/screenreflections.h"

// bare-bones render controller, in lack of anything better yet
class opengl46_renderer : public gfx_renderer, public gfx::scene_walk {
  public:
// constructors
    opengl46_renderer() = default;
// destructor
    ~opengl46_renderer() {}
// methods
    bool
        Init( GLFWwindow *Window ) override;
    void
        Shutdown() override;
    bool
        AddViewport(const global_settings::extraviewport_config &conf) override;
    // main draw call. returns false on error
    bool
        Render() override;
    void
        SwapBuffers() override;
    inline
    float
        Framerate() override { return m_framerate; }
    // geometry methods
    // NOTE: hands-on geometry management is exposed as a temporary measure; ultimately all visualization data should be generated/handled automatically by the renderer itself
    // creates a new geometry bank. returns: handle to the bank or NULL
    gfx::geometrybank_handle
        Create_Bank() override;
    // creates a new indexed geometry chunk of specified type from supplied data, in specified bank. returns: handle to the chunk or NULL
	gfx::geometry_handle Insert(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type) override;
    // creates a new geometry chunk of specified type from supplied data, in specified bank. returns: handle to the chunk or NULL
	gfx::geometry_handle Insert(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type) override;
    // replaces data of specified chunk with the supplied vertex data, starting from specified offset
	bool Replace(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type, const std::size_t Offset = 0) override;
	// replaces the index and the vertex data of specified indexed chunk with the supplied data of the same counts
	bool Replace_Indexed(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry) override;
    // adds supplied vertex data at the end of specified chunk
	bool Append(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type) override;
	void Erase( gfx::geometry_handle const &Geometry ) override;
    // provides direct access to index data of specfied chunk
    gfx::index_array const &
        Indices( gfx::geometry_handle const &Geometry ) const override;
    // provides direct access to vertex data of specfied chunk
    gfx::vertex_array const &
        Vertices( gfx::geometry_handle const &Geometry ) const override;
	// provides direct access to vertex data of specfied chunk
	gfx::userdata_array const &
		UserData( gfx::geometry_handle const &Geometry ) const override;
	int
		Primitive( gfx::geometry_handle const &Geometry ) const override;
    // material methods
    material_handle
        Fetch_Material( std::string const &Filename, bool const Loadnow = true ) override;
    material_handle
        Matte_Material( material_handle const Material ) override;
    material_handle
        Tinted_Material( material_handle const Material, glm::vec3 const &Colour ) override;
    material_handle
        Blend_Material( material_handle const Material, std::array<material_handle, 3> const &Others ) override;
    bool
        Blendable_Material( material_handle const Material ) override;
    void
        Bind_Material( material_handle const Material, TSubModel const *sm = nullptr, lighting_data const *lighting = nullptr ) override;
    IMaterial const *
        Material( material_handle const Material ) const override;
    material_handle
        Terrain_Material( material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement,
                          int const HoleSamples = 0, std::uint8_t const *Holes = nullptr ) override;
    void
        Terrain_Release( material_handle const Material ) override;
    void
        Terrain_Grass( std::vector<gfx::grass_kind> const &Kinds, std::vector<gfx::grass_chunk> const &Chunks ) override;
    // shader methods
    auto Fetch_Shader( std::string const &name ) -> std::shared_ptr<gfx::shader_program> override;
    auto Shader_Name( std::shared_ptr<gfx::shader_program> const &Program ) const -> std::string override;
    // texture methods
    texture_handle
        Fetch_Texture( std::string const &Filename, bool const Loadnow = true, GLint format_hint = GL_SRGB_ALPHA ) override;
    void
        Bind_Texture( texture_handle const Texture ) override;
    void
        Bind_Texture( std::size_t const Unit, texture_handle const Texture ) override;
    ITexture &
        Texture( texture_handle const Texture ) override;
    ITexture const &
        Texture( texture_handle const Texture ) const override;
    // utility methods
    void
        Pick_Control_Callback( std::function<void( TSubModel const *, const glm::vec2 )> Callback ) override;
    void
        Pick_Node_Callback( std::function<void( scene::basic_node * )> Callback ) override;
    TSubModel const *
        Pick_Control() const override { return m_pickcontrolitem; }
    scene::basic_node const *
        Pick_Node() const override { return m_picksceneryitem; }
    void
        Forget_Node( scene::basic_node const *Node ) override;
    glm::dvec3
        Mouse_Position() const override { return m_worldmousecoordinates; }
    bool
        Mouse_Hit() const override { return m_mousehit; }
    glm::mat4
        Camera_View_Matrix() const override { return glm::mat4( glm::mat3( m_colorpass.pass_camera.modelview() ) ); }
    glm::mat4
        Camera_Projection_Matrix() const override { return m_colorpass.pass_camera.projection(); }
    glm::dvec3
        Camera_Position() const override { return m_colorpass.pass_camera.position(); }
    // maintenance methods
    void
        Update( double const Deltatime ) override;
    bool
        Debug_Ui_State(std::optional<bool>) override;
    void
        Update_Pick_Control() override;
    void
        Update_Pick_Node() override;
    glm::dvec3
        Update_Mouse_Position() override;
    // debug methods
    std::string const &
        info_times() const override;
    std::string const &
        info_stats() const override;
    std::vector<info_section> const &
        info_sections() const override;
	  void MakeScreenshot() override;
    bool Render_Preview( TAnimModel *Instance, int const Size, int const Margin, bool const Shadows, std::vector<std::uint8_t> &Image ) override;



    opengl_material & Material( material_handle const Material );
    opengl_material const & Material( TSubModel const * Submodel ) const;
    // draws supplied geometry handles
    void Draw_Geometry(std::vector<gfx::geometrybank_handle>::iterator begin, std::vector<gfx::geometrybank_handle>::iterator end);
	void Draw_Geometry(const gfx::geometrybank_handle &handle);
	// material methods
    void Bind_Material_Shadow(material_handle const Material);
	void Update_AnimModel(TAnimModel *model);

	// members
    GLenum static const sunlight{0};

    static std::unique_ptr<gfx_renderer> create_func();
    // how the windows of the renderer are made (opengl46backend.cpp)
    static gfx_backend_desc backend_desc();

  private:
	using gfx::scene_walk::Render;
	using gfx::scene_walk::Render_Alpha;
	// the draws of the shared walk of the scene
	void walk_material(material_handle const Material, TSubModel const *Submodel = nullptr, lighting_data const *Lighting = nullptr) override;
	void walk_shadow_material(material_handle const Material) override;
	opengl_material const &walk_material_data(material_handle const Material) const override;
	opengl_material const &walk_material_data(TSubModel const *Submodel) const override;
	void walk_program(walk_shader const Program) override;
	void walk_texture(std::size_t const Unit, texture_handle const Texture) override;
	void walk_blending(bool const Alpha) override;
	void walk_sunlight(float const Factor = 1.f) override;
	void walk_environment(TEnvironmentType const Environment = e_flat) override;
	void walk_ambient_save() override;
	void walk_ambient_add(glm::vec3 const &Addition) override;
	void walk_ambient_restore() override;
	glm::vec4 walk_sun_ambient() const override;
	void walk_draw(gfx::geometry_handle const &Geometry) override;
	void walk_draw(std::vector<gfx::geometrybank_handle>::iterator First, std::vector<gfx::geometrybank_handle>::iterator Last) override;
	std::uint32_t walk_instances(glm::mat4 const *Modelviews, std::size_t const Count) override;
	void walk_cull_face(bool const Enabled) override;
	void walk_glare(bool const Enabled) override;
	void walk_glare_draw(TSubModel *Submodel, gfx::geometry_handle const &Geometry) override;
	void walk_light_query(TSubModel *Submodel, bool const Begin) override;
	// the gpu culling of the instances, models and sleepers, the depth pass of the whole scene
	void walk_region_begin() override;
	void walk_editor_cursor() override;
	bool walk_instances_begin() override;
	void walk_instances_end() override;
	gfx::geometrybank_manager &walk_geometry() override;
	bool walk_blending_enabled() const override;

	// types
	struct viewport_config {
		int width;
		int height;

		float draw_range;

		bool main = false;

        GLFWwindow *window = nullptr; // ogl window context
        bool real_window = true; // whether we need to blit onto GLFWwindow surface
        bool custom_backbuffer = false; // whether we want to render to our offscreen LDR backbuffer (pipeline required)
        bool shadow = false; // include this viewport area in shadowmap

        enum vp_type {
            normal,
            custom,
            vr_left,
            vr_right
        } proj_type;
		viewport_proj_config projection;

        // main msaa render target for pipeline mode
		std::unique_ptr<gl::framebuffer> msaa_fb;
		std::unique_ptr<gl::renderbuffer> msaa_rbc;
		std::unique_ptr<gl::renderbuffer> msaa_rbv;
		std::unique_ptr<gl::renderbuffer> msaa_rbd;

        // msaa resolve buffer (when using motion blur)
		std::unique_ptr<gl::framebuffer> main_fb;
		std::unique_ptr<opengl_texture> main_texv;
		std::unique_ptr<opengl_texture> main_texd;
		std::unique_ptr<opengl_texture> main_tex;

        // final HDR buffer (also serving as msaa resolve buffer when not using motion blur)
		std::unique_ptr<gl::framebuffer> main2_fb;
		std::unique_ptr<opengl_texture> main2_tex;

		// ambient occlusion, half size: raw and smoothed (gtao), or the ssao and its blur (older ssao)
		std::unique_ptr<opengl_texture>  ssao_tex;
		std::unique_ptr<opengl_texture>  ssao_blur_tex;
		std::unique_ptr<gl::framebuffer> ssao_fb;
		std::unique_ptr<gl::framebuffer> ssao_blur_fb;
		// main_tex alone, for the ambient occlusion to be applied to it before the motion blur (main_fb holds the depth it reads)
		std::unique_ptr<gl::framebuffer> ao_apply_fb;

		// the temporal anti-aliasing (main viewport only): the two images of the history, written in turn
		std::array<std::unique_ptr<opengl_texture>, 2> taa_tex;
		std::array<std::unique_ptr<gl::framebuffer>, 2> taa_fb;

		// the screen space reflections (main viewport only): the finished image of the last frame with its mipmaps, which they take
		// their colour from (Ssr_History_Target()), and the number of its levels. made when it is first needed
		std::unique_ptr<opengl_texture> ssr_tex;
		std::unique_ptr<gl::framebuffer> ssr_fb;
		int ssr_levels { 0 };

        // LDR backbuffer for offscreen rendering
        std::unique_ptr<gl::framebuffer> backbuffer_fb;
        std::unique_ptr<opengl_texture> backbuffer_tex;

        // levels of the bloom of the post processing; null without the pipeline
        std::unique_ptr<opengl46_exposure::bloom_chain> bloom;

        // light cluster grid of the viewport
        std::unique_ptr<gl::cluster_grid> clusters;
        // depth of the scene resolved from the msaa buffer (Resolved_Depth_Target()): of the finished scene for the cluster heatmap,
        // and of the depth pass of the scene for the contact shadows, read while the colour is drawn. made when it is first needed
        std::unique_ptr<gl::framebuffer> clusters_depth_fb;
        std::unique_ptr<opengl_texture> clusters_depth_tex;

        bool initialized = false;
	};

	viewport_config *m_current_viewport = nullptr;

	// methods
    std::unique_ptr<gl::program> make_shader(std::string v, std::string f);
	bool Init_caps();
	void setup_pass(viewport_config &Viewport, renderpass_config &Config, rendermode const Mode, float const Znear = 0.f, float const Zfar = 1.f, bool const Ignoredebug = false);
	void setup_matrices();
    void setup_drawing(bool const Alpha = false);
    void setup_shadow_empty_map();
    void setup_shadow_bind_map();
    void setup_shadow_color( glm::vec4 const &Shadowcolor );
    void setup_env_map(gl::cubemap *tex);
	void setup_environment_light(TEnvironmentType const Environment = e_flat);
    void setup_sunlight_intensity( float const Factor = 1.f);
    // sends the header of the light buffer and the sun to the gpu
    void upload_light_header();
	// runs jobs needed to generate graphics for specified render pass
	void Render_pass(viewport_config &vp, rendermode const Mode);
	// creates dynamic environment cubemap
	bool Render_reflections(viewport_config &vp);
	// draws the cube shadow maps of the lamps which need it in this frame (the cab lamp: the cab)
	void Render_local_shadows();
	// the occupied vehicle whose cab is drawn in the colour pass, nullptr when there's none (the conditions of Render_cab())
	TDynamicObject *Cab_Probe_Vehicle() const;
	// draws the probe of the occupied cab when it needs it in this frame
	void Render_cab_probe();
	// draws the parts of the cube shadow maps of the scenery lamps which need it in this frame
	void Render_lamp_shadows();
	// one part of the map of a scenery lamp: the scenery, or the vehicles by it (and Cab, the occupied cab) over a copy of it
	void Render_lamp_shadow(std::size_t const Slot, bool const Vehicles, TDynamicObject *Cab);
	// picks the headlights which get shadow maps in this frame and draws the maps which need it
	void Render_headlight_shadows();
	// the map of the headlights in the slot: the scene within reach ahead of them (and Cab, the occupied cab), without their vehicle
	void Render_headlight_shadow(std::size_t const Slot, light_array::light_record const &Light, TDynamicObject *Cab);
	// the culling of one of the maps of lamps and headlights (Global.gfx_gpucull_shadows_headlights, _lamps), the one at Index of
	// Culls, made when it's first needed; nullptr when the instances of the map go through the cpu
	opengl46_instance_cull *Light_Shadow_Cull(std::vector<std::unique_ptr<opengl46_instance_cull>> &Culls, std::size_t const Index, bool const Enabled);
	// the rows of these cullings for the debug panel: their timings (Times) or their counts
	std::string Light_Shadow_Cull_Info(bool const Times);
	// the projector of the beam of a set of headlights: a narrow frustum along the vehicle, from behind it, its near plane at the
	// end of the vehicle. the beam texture is projected with it (Update_Lights()) and the shadow map of the headlights drawn with it.
	// Position in world space, Rotation from world axes to its axes; Near is the distance of the near plane from the projector
	void Headlight_Projector(light_array::light_record const &Light, glm::dvec3 &Position, glm::mat3 &Rotation, glm::mat4 &Projection, float &Near, float &Fieldofview);
	bool Render(world_environment *Environment);
	// places supplied modelview matrices in the instance buffer of the frame. returns: base instance of the first of them
	std::uint32_t Upload_Instance_Modelviews( glm::mat4 const *Modelviews, std::size_t const Count );
	// binds the instance buffer of the frame for the vertex shaders (INSTANCE_SSBO_BINDING)
	void Bind_Instance_Stream() const;
	// draws the instances queued in the pass from the result of the gpu culling (Global.gfx_gpucull 1): makes the depth
	// pyramid of the occlusion test, runs the culling, walks the submodel trees of the groups once per lod band to record
	// the draws (m_indirectdraws, m_indirectcommands), has their instances filled in, then draws them with indirect draws,
	// their depth first (Global.gfx_depthprepass). inside the depth pass of the whole scene only their depth is drawn, their
	// colour waits for the colour pass (Render_Instances_Indirect_Colour()). in two phases
	// (Global.gfx_gpucull_occlusion_instances) the instances seen in the previous frame are drawn first (their depth, or
	// their colour without the prepass), the pyramid is made again and the other instances are culled against it
	void Render_Instances_Indirect();
	// the same for the passes culled on the gpu other than the main view (Global.gfx_gpucull_shadows, _reflections,
	// _shadows_headlights, _shadows_lamps): the cascades of the sun shadows and the maps of headlights and lamps, drawn with the
	// shadow programs (the cube ones for the lamps), and the faces of the environment map, drawn with the programs of the
	// materials like the main view. no occlusion test, no depth prepass, one phase
	void Render_Instances_Indirect_Pass();
	// the walk of the submodel trees of the groups culled by Cull, once per lod band, which records their draws
	// (m_indirectdraws, m_indirectcommands; Walk_Indirect_Draws()), puts them in order for multi draws and has Cull fill in their
	// commands
	void Record_Indirect_Draws( opengl46_instance_cull &Cull );
	// the culling the pass under way feeds (Begin_Pass_Cull()), always one of this renderer
	opengl46_instance_cull &Pass_Cull() { return static_cast<opengl46_instance_cull &>( *m_passcull ); }
	// draws of the instances made, and the gl calls which made them
	struct indirect_counts {
		std::uint32_t draws { 0 };
		std::uint32_t calls { 0 };
		indirect_counts &operator+=( indirect_counts const &Other ) {
			draws += Other.draws;
			calls += Other.calls;
			return *this; }
	};
	// the recorded draws of the instances, with the commands of the given set: their depth with the programs of the depth
	// prepass, with the state of the prepass already set, the same with the state set for it and the time measured outside
	// the depth pass of the whole scene, and their colour. the command buffer and the instances are bound by the caller.
	// with multi draws (m_indirectmultidraw) the draws which follow one another with the same state go in one call
	indirect_counts Render_Instances_Indirect_Depth( opengl46_instance_cull::command_set const Set );
	indirect_counts Render_Instances_Indirect_Prepass( opengl46_instance_cull::command_set const Set, gl::gpu_timer &Timer );
	indirect_counts Render_Instances_Indirect_Colour( opengl46_instance_cull::command_set const Set );
	// the recorded draws of a shadow pass with the shadow programs, the way Bind_Material_Shadow() picks them
	indirect_counts Render_Instances_Indirect_Shadow( opengl46_instance_cull::command_set const Set );
	// multi draws of the recorded draws (Global.gfx_gpucull_multidraw), put in order by Order_Indirect_Draws(): places their
	// model blocks in the buffer of the frame (m_drawblockstream), in the same order
	void Upload_Indirect_Blocks();
	// binds the model blocks of Count recorded draws from First for the programs of the multi draws (MODEL_DRAW_SSBO_BINDING)
	void Bind_Indirect_Blocks( std::size_t const First, std::size_t const Count );
	// the program Bind_Material() binds for the material, nullptr when it binds none
	gl::program const *Indirect_Program( material_handle Material ) const;
	// the variant of a material program for multi draws (built with MODEL_DRAWS), made the first time it's asked for.
	// nullptr if there's none (a program not fetched by Fetch_Shader(), or one which didn't compile)
	gl::program *Multidraw_Program( gl::program const *Program );
	// how the depth prepass treats the material: 0 left out, 1 drawn without alpha test, 2 with the alpha test of the diffuse texture
	int Prepass_Class( material_handle const Material );
	// depth of the whole opaque scene of the colour pass, before its colour (Global.gfx_depthprepass_scene): the colour pass
	// walks the scene a second time, Bind_Material() binds the programs of the depth prepass and draw() leaves out what they
	// can't draw. with Global.gfx_depthprepass_replay the walk of the colour pass is made here, once: its opaque draws are
	// recorded (Record_Scene_Draw()), their depth is drawn from the record (Render_Scene_Prepass()) and the colour pass draws
	// them from it (Render_Scene_Colour()). Future: the motion matrix the colour pass draws the region with. Contactonly: the
	// depth is drawn only for the contact shadows, which need it within their reach
	void Render_Scene_Depth( glm::mat4 const &Future, bool const Contactonly );
	// notes a draw of the walk recorded by Render_Scene_Depth(): the chunks drawn, with the state the walk left for them
	void Record_Scene_Draw( gfx::geometry_handle const *First, std::size_t const Count, bool const Batch );
	// draws the depth of the recorded draws not drawn yet, in runs of the same program, bank and texture of the alpha test,
	// with multi draws when there are any. the depth pyramid of the occlusion test and the cursor of the editor take the depth
	// drawn so far, and the end of the walk the rest
	void Render_Scene_Prepass();
	// the colour of the recorded draws, in the order of the walk, with the programs of their materials
	void Render_Scene_Colour();
	// the colour of the instances culled and recorded by the depth pass of the whole scene (m_indirectcolourpending)
	void Render_Instances_Indirect_Pending();
	// the bounding sphere of a geometry chunk in the space of its vertices, w the radius (negative if there's none)
	glm::vec4 const &Chunk_Sphere( gfx::geometry_handle const &Geometry );
	// whether the colour pass of the viewport gets contact shadows, which need the depth pass of the scene
	bool Contact_Shadows_Wanted( viewport_config const &Viewport ) const;
	// resolves the depth drawn by Render_Scene_Depth() for the contact shadows and sets them up for the colour pass
	void Setup_Contact_Shadows( viewport_config &Viewport );
	// makes the target of the resolved depth of the viewport if it isn't there yet; false if it can't be made
	bool Resolved_Depth_Target( viewport_config &Viewport );
	// whether the colour pass of the viewport takes the ambient occlusion into the light of its surfaces (gfx.postfx.ao.lighting),
	// which needs the depth pass of the scene
	bool Ao_Lighting_Wanted( viewport_config const &Viewport ) const;
	// works out the ambient occlusion of the depth drawn by Render_Scene_Depth() and sets it up for the colour pass
	void Setup_Ao_Lighting( viewport_config &Viewport );
	// resolves the depth drawn so far in the colour pass of the viewport for the soft particles (gfx.particles.soft) and binds it
	// to gl::SCENE_DEPTH_TEX; false when the viewport has no such depth
	bool Setup_Particles_Depth( viewport_config &Viewport );
	// the direction the sun shadows are cast in (the sun, its height capped by gfx.shadow.angle.min), and the rotation from
	// the world to the space of the light of the cascades: x and y across the maps, z along the light
	glm::vec3 Shadow_Light_Vector() const;
	glm::mat3 Shadow_Light_Rotation() const;
	// the distances the fixed cascades of the sun shadows go from and to (shadowtune)
	std::array<float, gl::MAX_CASCADES + 1> Shadow_Fixed_Splits() const;
	// why the cascades of the sun shadows can't be fitted to the depth of the viewport (gfx.shadow.sdsm), null if they can
	char const *Shadow_Fit_Unavailable( viewport_config const &Viewport ) const;
	// the cascades of the frame fitted to the latest measurement of the depth of the view (gfx.shadow.sdsm): the distances
	// they go from and to, and for each cascade the corners of the area its map is to cover, camera-relative world space
	// (none: the whole slice of the view). false: the fixed cascades. fills m_shadowfitstate
	bool Shadow_Fit( viewport_config const &Viewport, std::array<float, gl::MAX_CASCADES + 1> &Splits,
	                 std::array<std::optional<std::array<glm::vec3, 8>>, gl::MAX_CASCADES> &Areas );
	// whether the depth pass of the whole scene draws the material bound last (Bind_Material()) and the geometry
	bool Scene_Depth_Draws( gfx::geometry_handle const &Geometry ) const;
	// releases gl textures used only by vehicles farther than gfx.textures.releasedistance times the draw range
	void Update_Texture_Release();
	// textures used by materials of the sub-models of specified model, replaceable skins excluded
	std::vector<texture_handle> const &Model_Textures( TModel3d *Model );
	// has the data of the textures read ahead of the camera, with gfx.textures.streaming
	void Update_Texture_Prefetch();
	void Render_particles();
	void Render_precipitation();
	// the view of the colour pass being drawn, for the particles of the gpu
	opengl46_gpu_particles::view Particles_View() const;
	// the grass of the heightmap terrain around the camera of the main view; Future: the matrix of the motion vectors
	void Render_grass(glm::mat4 const &Future);
  void Render_vr_models();
	void Render_editor_overlay();
	// writes the lighting of the pass about to be drawn to the light buffer: the sun, the ambient light and the fog, and the headlights of Lights
	// unless Headlights is false, as in the pass which draws the environment map, where lights of vehicles would only get baked into the map.
	// Clustered tells the shading to take the headlights from the lists of the clusters (see Cull_Lights()) and not to go through the first few.
	// with Headlights and Clustered the cab lamp (first) and the lamps of models and vehicles (after the headlights) are written as well
	void Update_Lights(light_array &Lights, bool const Headlights = true, bool const Clustered = false);
	// writes a light of a model or a vehicle, or the cab lamp, to the slot of the light buffer, in the view space of the pass
	void Write_Local_Light(gl::light_element &Light, gfx::local_lights::light const &Source, glm::dvec3 const &Camera, glm::mat4 const &View);
	// brings the light cluster grid of the viewport up to date with the projection of the color pass being drawn.
	// Targetsize is the size in pixels of what the pass draws to, which gl_FragCoord of the shading counts in
	void Update_Clusters(viewport_config &vp, glm::ivec2 const &Targetsize);
	// assigns the lights set up by Update_Lights() to the clusters of the viewport; run after it, before the scene is drawn
	void Cull_Lights(viewport_config &vp);
	// debug view: tints the finished image of the viewport with the cluster of every pixel. Target is the framebuffer
	// holding the image (null for the default one), Targetsize its size
	void Render_cluster_heatmap(viewport_config &vp, gl::framebuffer *Target, glm::ivec2 const &Targetsize);
	// ambient occlusion from the resolved depth (vp.main_texd), multiplied into the image of Target. Mode: Global.gfx_postfx_ao
	void Render_ambient_occlusion(viewport_config &vp, gl::framebuffer &Target, int const Mode);
	// the temporal anti-aliasing of the main view (gfx.antialiasing taa), and whether the viewport has it
	void Render_taa(viewport_config &vp);
	bool Taa_View(viewport_config const &vp) const;
	// the screen space reflections (gfx.ssr): whether the colour pass of the viewport gets them (they need the depth pass of the
	// scene), the depth resolved for them with the history bound and the parameters of the pass set, the finished image of the
	// frame copied to the history (from the colour attachment Attachment of Source), and the target of the history made when it
	// isn't there yet (false if it can't be made)
	bool Ssr_View(viewport_config const &vp) const;
	void Setup_Screen_Reflections(viewport_config &vp, bool const Temporal);
	void Store_Ssr_History(viewport_config &vp, gl::framebuffer &Source, GLenum const Attachment);
	bool Ssr_History_Target(viewport_config &vp);

	bool init_viewport(viewport_config &vp);

    void draw(const gfx::geometry_handle &handle);
    void draw(std::vector<gfx::geometrybank_handle>::iterator begin, std::vector<gfx::geometrybank_handle>::iterator end);

	void draw_debug_ui();

	// model previews
	// extends provided bounds with the geometry of the submodel and its siblings and children visible in a preview
	void Preview_Bounds( TSubModel const *Submodel, glm::dmat4 const &Transform, glm::dvec3 &Min, glm::dvec3 &Max ) const;
	// prepares the offscreen targets of model previews for square images of specified size
	bool Preview_Targets( int const Size );
	// draws the instance on its own with the current render pass settings
	void Preview_Model( TAnimModel *Instance, float const Squaredistance, bool const Alpha );
	// draws the lit instance into the hdr preview target, marking in its stencil the pixels covered by the model
	void Preview_Color( TAnimModel *Instance, float const Squaredistance, int const Size, glm::dvec3 const &Eye, glm::mat4 const &View, glm::mat4 const &Projection, bool const Shadows );

	// members
	GLFWwindow *m_window{nullptr}; // main window
	gfx::geometrybank_manager m_geometry;
	material_manager m_materials;
	texture_manager m_textures;
	opengl46_terrain_materials m_terrainmaterials{m_materials, m_textures};
	opengl46_grass m_grass; // of the heightmap terrain
	opengl46_light m_sunlight;
	// fog colour of the frame: the one of the simulation, or of the atmosphere (see opengl46_sky::update())
	glm::vec3 m_fogcolor{0.f};
	// light of the sky: directional ambient light, atmosphere, prefiltered environment map
	opengl46_sky m_sky;
	// fog, clouds and their shadows, stars
	opengl46_weather m_weather;
	// automatic exposure and bloom of the post processing
	opengl46_exposure m_exposure;
	// screen space ambient occlusion of the post processing
	opengl46_ao m_ao;
	// true while the environment map bound for the shading is the real one (not the empty stand-in of the reflection pass)
	bool m_envmap_bound{false};
	/*
	    float m_sunandviewangle; // cached dot product of sunlight and camera vectors
    */
	texture_handle m_suntexture{-1};
    texture_handle m_moontexture{-1};
	texture_handle m_smoketexture{-1};
    texture_handle m_headlightstexture{-1};

	// main shadowmap resources
	int m_shadowbuffersize{2048};
	glm::mat4 m_shadowtexturematrix; // conversion from camera-centric world space to light-centric clip space

	int m_environmentcubetextureface{0}; // helper, currently processed cube map face
	int m_environmentupdateface{6}; // cube map face the environment map update in progress draws next; 6: no update in progress
	bool m_environmentready{false}; // the environment map was drawn whole at least once
	double m_environmentupdatetime{0}; // time of the most recent environment map update
	glm::dvec3 m_environmentupdatelocation; // coordinates of most recent environment map update
    opengl46_skydome m_skydomerenderer;
    opengl46_precipitation m_precipitationrenderer;
    opengl46_particles m_particlerenderer; // particle visualization subsystem
    opengl46_gpu_particles m_gpuparticles; // the smoke, rain and snow of the gpu (gfx.smoke.gpu, gfx.precipitation.gpu)
    opengl46_windscreen_water m_windscreenwater; // the water on the windscreens of the cab of the train
    bool m_particlesoft{false}; // the depth for the soft particles is bound in the colour pass being drawn

	float m_framerate;
	double m_updateaccumulator{0.0};
	std::string m_debugtimestext;
	std::string m_pickdebuginfo;
	//debug_stats m_debugstats;
	std::string m_debugstatstext;
	std::vector<info_section> m_debugsections; // the same data in sections, for the debug panel
	// what the process and its parts keep in memory, for the debug panel, gathered every 2 s; whether the card tells its memory
	// (GL_NVX_gpu_memory_info), -1 not asked yet
	void Update_Memory_Info();
	std::string m_memorytext;
	std::chrono::steady_clock::time_point m_memorytime {};
	int m_nvxmemory { -1 };
    struct simulation_state {
        std::string weather;
        std::string season;
    } m_simulationstate;

	glm::vec4 m_baseambient{0.0f, 0.0f, 0.0f, 1.0f};
	glm::vec4 m_shadowcolor{colors::shadow};
	//    TEnvironmentType m_environment { e_flat };
	float m_specularopaquescalefactor{1.f};
    float m_speculartranslucentscalefactor{1.f};

	// textures used by vehicle models, for the release of far textures
	std::unordered_map<TModel3d const *, std::vector<texture_handle>> m_modeltextures;
	// the read ahead of textures (Update_Texture_Prefetch()): the cell it goes on with
	std::size_t m_prefetchcursor { 0 };
	std::chrono::steady_clock::time_point m_prefetchvehicletime;
	std::chrono::steady_clock::time_point m_prefetchdroptime;
	unsigned int m_nodestateversion { 0 }; // scene::node_state_version which m_instancerecordsversion was last checked against
	// culling of instances in compute passes, run along the first color pass of the main viewport of a frame
	// when Global.gfx_gpucull is set: the instances of the pass are drawn from its result (1), or the result is compared
	// with the one of the cpu (2)
	opengl46_instance_cull m_instancecull;
	unsigned int m_instancecullframe { ~0u }; // frame it last ran in
	// the cullings of the cascades of the sun shadows (Global.gfx_gpucull_shadows), with records and programs shared with
	// m_instancecull
	std::array<opengl46_instance_cull, 3> m_shadowcull;
	// the cullings of the maps of headlights, by the slot of the map, and of the scenery parts of the maps of lamps, by their order
	// among those drawn in a frame: the passes of a frame don't share the buffers of a culling. made when first needed
	std::vector<std::unique_ptr<opengl46_instance_cull>> m_headlightcull;
	std::vector<std::unique_ptr<opengl46_instance_cull>> m_lampcull;
	opengl46_instance_cull *m_lightshadowcull { nullptr }; // the one of the map of headlights or of a lamp being drawn
	std::size_t m_lampscenerydrawn { 0 }; // scenery parts of the maps of lamps drawn in this frame so far
	opengl46_instance_cull *m_indirectcull { &m_instancecull }; // the one the recorded draws (m_indirectdraws) were culled by
	int m_shadowcascade { -1 }; // the region is drawn for this cascade of the sun shadows
	// depth pyramid for the occlusion test of the gpu culling
	opengl46_hiz m_hiz;
	// the cascades of the sun shadows fitted to the depth of the main view (Global.gfx_shadow_sdsm)
	opengl46_shadow_fit m_shadowfit;
	// with a cascade fitted: the corners of the area its map is to cover, for setup_pass()
	std::array<glm::vec3, 8> const *m_shadowfitarea { nullptr };
	// what the last shadow pass did, for the debug panel
	gfx::shadow_fit::fit_state m_shadowfitstate;
	// the sizes setup_pass() found for the last shadow pass: of the whole slice of the view and as drawn
	float m_shadowslicesize { 0.f };
	float m_shadowdrawnsize { 0.f };
	// cpu time of the parts of the culling and the draws of the instances of the main view (Render_Instances_Indirect() and
	// _Pending()), for the panel: the depth of the recorded walk of the scene drawn before the culling, the depth pyramids, the
	// culling passes, the walks with putting the draws in order, the depth and the colour of the instances
	struct cull_parts {
		double scenedepth { 0.0 };
		double pyramid { 0.0 };
		double culling { 0.0 };
		double record { 0.0 };
		double depth { 0.0 };
		double colour { 0.0 };
	};
	cull_parts m_cullparts;
	// multi draws of the recorded draws: whether they were recorded for them (ordered, their model blocks uploaded), and where
	// the blocks are
	bool m_indirectmultidraw { false };
	GLuint m_indirectblockbuffer { 0 };
	GLintptr m_indirectblockoffset { 0 };
	std::uint64_t m_indirectblockgeneration { 0 };
	// working space of Upload_Indirect_Blocks(), kept between frames
	std::vector<gl::model_ubs> m_indirectblocks;
	// the model blocks of the multi draws, one buffer for each of the last frames like model_ubo (nullptr: no multi draws)
	std::unique_ptr<gl::frame_stream> m_drawblockstream;
	// programs of the multi draws: the vertex shader of the materials, variants of the material programs, of the depth prepass
	std::unique_ptr<gl::shader> m_vertex_multidraw_shader;
	std::unordered_map<gl::program const *, std::unique_ptr<gl::program>> m_multidrawprograms;
	std::unique_ptr<gl::program> m_prepass_multidraw_shader;
	std::unique_ptr<gl::program> m_prepass_alpha_multidraw_shader;
	std::unique_ptr<gl::program> m_shadow_multidraw_shader; // and of the sun shadows
	std::unique_ptr<gl::program> m_alpha_shadow_multidraw_shader;
	bool m_bindmultidraw { false }; // Bind_Material() binds the variant of the material program for multi draws
	// depth prepass: programs and the class of the material programs (see Prepass_Class())
	std::unique_ptr<gl::program> m_prepass_shader;
	std::unique_ptr<gl::program> m_prepass_alpha_shader;
	std::unordered_map<gl::program const *, int> m_prepassclasses;
	gl::gpu_timer m_prepasstimer;
	gl::gpu_timer m_prepassfirsttimer; // of the instances drawn first, in two phases
	// depth pass of the whole scene (Render_Scene_Depth())
	bool m_scenedepthpass { false }; // inside it: Bind_Material() and draw() work for the depth
	bool m_scenedepthskip { true }; // the material bound last isn't drawn in it
	bool m_scenedepthdrawn { false }; // it was drawn for the colour pass under way, which skips what was done in it already
	// with m_indirectcolourpending: the instances were culled in it, the colour pass draws them from the recorded draws
	opengl46_instance_cull::command_set m_indirectcolourset { opengl46_instance_cull::command_set::first }; // with these commands
	debug_stats m_scenedepthstats; // of its last run
	double m_scenedepthcputime { 0.0 }; // ms
	gl::gpu_timer m_scenedepthtimer;
	// the recorded walk of the colour pass (Global.gfx_depthprepass_replay): a draw with the state it was made with
	struct scene_draw {
		gl::model_ubs ubs;
		material_handle material { null_handle }; // bound last before it (Bind_Material())
		TSubModel const *submodel { nullptr };
		std::uint32_t seed { 0 }; // TSubModel::iVariantSeed then
		std::uint32_t variant { 0 }; // TSubModel::VariantId() then: which textures of the material were bound
		std::uint32_t chunkfirst { 0 }; // in m_scenechunks
		std::uint32_t chunkcount { 0 };
		std::uint32_t instancecount { 0 }; // instanced draw (Render_Instanced(), Render_Sleepers()), 0 if not
		std::uint32_t instancebase { 0 };
		GLuint instancebuffer { 0 }; // the instance buffer of the frame the matrices of the instances went to
		std::uint32_t header { 0 }; // the light header (m_sceneheaders) it is lit with
		texture_handle prepasstexture { null_handle }; // of the alpha test of the depth pass
		int prepassclass { 0 }; // Prepass_Class(), 0 as well for what the depth pass leaves out (blending, opacity < 0)
		bool blending { false };
		bool batch { false }; // drawn with draw_batch() (neighbouring tracks)
		bool instances { false }; // no draw: the colour of the instances culled in the walk goes here
	};
	struct scene_chunk {
		gfx::geometry_handle geometry;
		std::uint32_t draw;
		bool depth; // drawn in the depth pass
	};
	using light_header = std::array<std::uint8_t, offsetof( gl::light_data, lights ) + sizeof( gl::light_element )>;
	bool m_scenerecording { false }; // inside the walk of Render_Scene_Depth(): draw(), Bind_Material(), upload_light_header() record
	bool m_scenerecorded { false }; // the colour pass under way draws the record
	bool m_scenereplayed { false }; // the last depth pass of the scene was drawn from a record
	bool m_scenedepthreach { false }; // the depth pass leaves out the draws lying whole farther than m_scenedepthlimit
	float m_scenedepthlimit { 0.f };
	material_handle m_scenematerial { null_handle }; // what Bind_Material() bound last in the walk
	TSubModel const *m_scenesubmodel { nullptr };
	std::uint32_t m_sceneseed { 0 };
	std::uint32_t m_scenevariant { 0 };
	std::vector<scene_draw> m_scenedraws;
	std::vector<scene_chunk> m_scenechunks;
	std::vector<gfx::geometry_handle> m_scenegeometry; // the chunks of m_scenechunks, for draw_batch()
	std::vector<light_header> m_sceneheaders; // the first one is in the light buffer when the walk begins
	std::size_t m_sceneflushed { 0 }; // chunks Render_Scene_Prepass() went through
	// working space of Render_Scene_Prepass(): a chunk drawn and what its run has in common
	struct scene_prepass_item {
		int prepassclass;
		texture_handle texture;
		std::uint32_t bank;
		unsigned int type;
		std::uint32_t indexed;
		GLuint instancebuffer; // 0 for a draw which isn't instanced, which reads the identity every buffer starts with
		std::uint32_t chunk;
		gl::instance_draw_command command;
		auto tie() const {
			return std::tie( prepassclass, texture, bank, type, indexed, instancebuffer, chunk ); }
	};
	std::vector<scene_prepass_item> m_sceneprepassitems;
	std::vector<gl::model_ubs> m_sceneprepassblocks;
	std::vector<gl::instance_draw_command> m_sceneprepasscommands;
	// the commands of the multi draws of the depth pass, one buffer for each of the last frames (nullptr: no multi draws)
	std::unique_ptr<gl::frame_stream> m_scenecommandstream;
	// of the last recorded walk: parts drawn (chunks of geometry; neighbouring tracks are drawn together), those of them drawn in
	// the depth pass and its gl calls, and those left out of it as farther than the reach of the contact shadows
	std::uint32_t m_scenerecordedparts { 0 };
	std::uint32_t m_sceneprepassdraws { 0 };
	std::uint32_t m_sceneprepasscalls { 0 };
	std::uint32_t m_scenebeyondreach { 0 };
	// Chunk_Sphere() of the chunks asked about, dropped when their vertices change (Replace(), Append())
	std::unordered_map<std::uint64_t, glm::vec4> m_chunkspheres;
	// contact shadows: set up for the colour pass under way, and whether the last main view had them
	bool m_contactshadows { false };
	bool m_contactshadowslast { false };
	// the ambient occlusion of the colour pass of the main view under way is in the light of its surfaces (gfx.postfx.ao.lighting),
	// not applied to its image
	bool m_aolighting { false };
	// a vehicle drawn in the colour pass of the last frame moved, and one drawn in this frame did (motion blur, Render_pass())
	bool m_vehiclesmoved { true };
	gl::gpu_timer m_contactshadowstimer; // resolve of the depth
	// the screen space reflections of the main view: the matrices and the parameters, set up for the colour pass under way, begun
	// for the frame (the history written at its end), and the copy of the image to the history with its mipmaps
	gfx::screen_reflections m_ssr;
	bool m_ssractive { false };
	bool m_ssrframe { false };
	gl::gpu_timer m_ssrhistorytimer;
  renderpass_config m_colorpass; // parametrs of most recent color pass
	std::array<renderpass_config, 3> m_shadowpass; // parametrs of most recent shadowmap pass for each of csm stages
	TSubModel const *m_pickcontrolitem{nullptr};
    scene::basic_node *m_picksceneryitem{nullptr};
    glm::vec3 m_worldmousecoordinates { 0.f };
    bool m_mousehit { false };
#ifdef EU07_USE_DEBUG_CAMERA
	renderpass_config m_worldcamera; // debug item
#endif

	std::optional<gl::query> m_timequery;
	GLuint64 m_gllasttime = 0;
	// gpu time of the parts of the frame, marks put one after another through the main view (the context of m_window)
	gl::gpu_timeline m_gputimeline;
	// cpu time of the same parts, marked at the same places: a part lasts from its mark to the next one, the parts of the same
	// name are added up in a frame and averaged over the frames the way Timer::stopwatch averages
	struct cpu_timeline {
		struct part {
			char const *name;
			float time { 0.f }; // average, ms
			float frame { 0.f }; // in the frame under way, ms
		};
		std::vector<part> parts; // in the order they came first
		std::size_t current { 0 };
		std::chrono::steady_clock::time_point mark;
		bool active { false };
		// starts the frame with its first part. the names are kept as pointers: string literals
		void begin( char const *Name );
		// ends the part under way and starts the next one
		void next( char const *Name );
		void end();
		// average of the part of the name, or none
		std::optional<float> time( char const *Name ) const;
	};
	cpu_timeline m_cputimeline;
	// gpu time of the light cluster compute passes; the debug panel shows it once a pass measures it
	gl::gpu_timer m_clusterstimer;
	// gpu time of assigning the lights to the clusters, done every frame
	gl::gpu_timer m_cullstimer;
	// headlights placed in the light buffer by the last Update_Lights(), and those which are on but did not fit in it
	uint32_t m_lights_lit = 0;
	uint32_t m_lights_dropped = 0;
	// lights of models and vehicles and the cab lamp, gathered once per frame for the light clusters
	gfx::local_lights m_locallights;
	// the ambient light of the light buffer while the cab is lit (walk_ambient_save())
	packed_vec3 m_cabambient { 0.f };
	// cube shadow maps of the lamps
	opengl46_local_shadows m_localshadows;
	// the shadow pass draws to a cube shadow map, with the programs of m_localshadows
	bool m_cubeshadows = false;
	// the probe of the occupied cab, and the colour mode draws the cab into it with its programs (Render_cab_probe())
	opengl46_cab_probe m_cabprobe;
	bool m_cabprobepass = false;
	// the occlusion of the sky and of the sun by the scenery around the camera (tunnels, bridges, roofs: the track T)
	opengl46_sky_occlusion m_skyocclusion;

    double m_precipitationrotation;

    glm::mat4 perspective_projection_raw(float fovy, float aspect, float znear, float zfar);
	glm::mat4 perspective_projection(const viewport_proj_config &c, float n, float f, glm::mat4 &frustum);
    glm::mat4 ortho_projection(float left, float right, float bottom, float top, float z_near, float z_far);
    glm::mat4 ortho_frustumtest_projection(float left, float right, float bottom, float top, float z_near, float z_far);
	void editor_ortho_projection(viewport_config const &Viewport, float const Zfar, glm::mat4 &Projection, glm::mat4 &Frustum);

    std::vector<std::function<void(TSubModel const *, glm::vec2)>> m_control_pick_requests;
    std::vector<std::function<void(scene::basic_node *)>> m_node_pick_requests;

	std::unique_ptr<gl::shader> m_vertex_shader;

	std::unique_ptr<gl::ubo> scene_ubo;
	// the model block of every draw, streamed (Global.gfx_bufferstreaming): each update to its own place in the buffer of
	// the frame instead of glBufferSubData of the one block
	std::unique_ptr<gl::stream_ubo> model_ubo;
	std::unique_ptr<gl::ssbo> light_ssbo;
	// per-instance modelview matrices of GPU-instanced draws, one buffer for each of the last frames, used in turn so
	// a frame doesn't write to the buffer the gpu may still read for an earlier one. element 0 of each is identity
	// and never overwritten: draws which aren't instanced (gl_BaseInstance == gl_InstanceID == 0) compute
	// identity*modelview in the vertex shader. Render_Instanced and Render_Sleepers append the camera-space root
	// matrices of their instances past the ones placed earlier in the frame, and draw them from that base instance.
	// persistently mapped with Global.gfx_bufferstreaming
	std::unique_ptr<gl::frame_stream> m_instancestream;
	gl::scene_ubs scene_ubs;
	gl::light_data light_data;

	std::unordered_map<std::string, std::shared_ptr<gl::program>> m_shaders;

	std::unique_ptr<gl::program> m_line_shader;
	gfx::geometrybank_handle m_editor_overlay_bank;
	std::vector<gfx::geometry_handle> m_editor_overlay_geometry;
	unsigned int m_editor_overlay_revision{0};
	std::unique_ptr<gl::program> m_freespot_shader;
    std::unique_ptr<gl::program> m_billboard_shader;
    std::unique_ptr<gl::program> m_celestial_shader;
    std::unique_ptr<gl::program> m_hiddenarea_shader;
    std::unique_ptr<gl::program> m_copy_shader;
    std::unique_ptr<gl::program> m_cluster_build_shader;
    std::unique_ptr<gl::program> m_cluster_cull_shader;
    std::unique_ptr<gl::program> m_cluster_heatmap_shader;

    std::unique_ptr<gl::vao> m_empty_vao;

	std::vector<std::unique_ptr<viewport_config>> m_viewports;

	std::unique_ptr<gl::postfx> m_pfx_motionblur;
	std::unique_ptr<gl::postfx> m_pfx_taa;
	gfx::temporal_aa m_taa;
	std::unique_ptr<gl::postfx> m_pfx_tonemapping;
  std::unique_ptr<gl::postfx> m_pfx_chromaticaberration;

	// postfx ssao
	std::unique_ptr<gl::postfx>      m_pfx_ssao;
	std::unique_ptr<gl::postfx>      m_pfx_ssao_blur;
	std::unique_ptr<opengl_texture> m_ssao_noise_tex;
	glm::vec3                        m_ssao_kernel[32];

	std::unique_ptr<gl::program> m_shadow_shader;
	std::unique_ptr<gl::program> m_alpha_shadow_shader;

	std::unique_ptr<gl::framebuffer> m_pick_fb;
	std::unique_ptr<opengl_texture> m_pick_tex;
	std::unique_ptr<gl::renderbuffer> m_pick_rb;
  std::unique_ptr<gl::program> m_pick_surface_shader;
	std::unique_ptr<gl::program> m_pick_shader;

	std::unique_ptr<gl::cubemap> m_empty_cubemap;

	std::unique_ptr<gl::framebuffer> m_env_fb;
	std::unique_ptr<gl::renderbuffer> m_env_rb;
	std::unique_ptr<gl::cubemap> m_env_tex;

	std::unique_ptr<gl::framebuffer> m_shadow_fb;
	std::unique_ptr<opengl_texture> m_shadow_tex;
	std::unique_ptr<opengl_texture> m_shadow_empty_tex; // what the shadow sampler reads when there is no shadow map in place

    std::unique_ptr<gl::pbo> m_picking_pbo;
    std::unique_ptr<gl::pbo> m_picking_node_pbo;

    std::unique_ptr<gl::pbo> m_depth_pointer_pbo;
    std::unique_ptr<gl::framebuffer> m_depth_pointer_fb;
    std::unique_ptr<gl::renderbuffer> m_depth_pointer_rb;

	material_handle m_invalid_material;

    bool m_blendingenabled;

	struct headlight_config_s
	{
		float in_cutoff = 1.005f;
		float out_cutoff = 0.993f;

		float falloff_linear = 0.15f;
		float falloff_quadratic = 0.15f;

		float intensity = 1.0f;
		float ambient = 0.0f;
	};

	headlight_config_s headlight_config;

    std::unique_ptr<vr_interface> vr;
    bool debug_ui_active = false;

	// offscreen targets of model previews
	struct preview_targets
	{
		int size { 0 };
		std::unique_ptr<opengl_texture> hdr_tex;
		std::unique_ptr<gl::renderbuffer> hdr_rbds; // depth and stencil, the stencil marks pixels covered by the model
		std::unique_ptr<gl::framebuffer> hdr_fb;
		std::unique_ptr<opengl_texture> ldr_tex;
		std::unique_ptr<gl::framebuffer> ldr_fb;
	} m_preview;

    static bool renderer_register;

	class opengl46_imgui_renderer : public imgui_renderer
	{
		virtual bool Init() override;
		virtual void Shutdown() override;
		virtual void BeginFrame() override;
		virtual void Render() override;
		using imgui_renderer::Create_Image;
		virtual std::uint64_t Create_Image(std::uint8_t const *Rgba, int const Width, int const Height, image_options const &Options) override;
		virtual std::uint64_t Update_Image(std::uint64_t const Image, std::uint8_t const *Rgba, int const Width, int const Height, image_options const &Options) override;
		virtual void Release_Image(std::uint64_t const Image) override;
	} m_imgui_renderer;

  virtual imgui_renderer* GetImguiRenderer() override {
	  return &m_imgui_renderer;
  }
};

//---------------------------------------------------------------------------
