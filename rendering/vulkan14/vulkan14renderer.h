/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the vulkan 1.4 renderer, picked with gfxrenderer vulkan (claude/vulkan14-renderer-plan.md): the gpu, the windows with their
// swapchains, the frames in flight, the memory and the uploads, the shaders and the pipelines, the pushed descriptors, the passes
// with their barriers and gpu times, the textures and the user interface (imgui) with the launcher (the track R of the plan), and
// the scene (the track D): the geometry, the materials, the passes of the colour of the scene with the sun shadows, the
// environment map and the post-processing, the picking, the screenshots, the extra viewports and the light clusters. the scene
// is walked by the walk shared with the opengl renderer (gfx::scene_walk), whose draws this renderer records.
//
// the vulkan headers stay in the files of rendering/vulkan14: the objects of the api live in vulkan14_renderer::context

#pragma once

#include "rendering/renderer.h"
#include "rendering/geometrybank.h"
#include "rendering/light.h"
#include "rendering/lightarray.h"
#include "rendering/common/scenewalk.h"
#include "rendering/common/locallights.h"
#include "rendering/common/skylight.h"
#include "rendering/common/weatherstate.h"
#include "rendering/common/shadowfit.h"
#include "rendering/common/gpuparticles.h"
#include "rendering/common/temporalaa.h"
#include "rendering/common/screenreflections.h"
#include "rendering/particles.h"
#include "model/material.h"
#include "vehicle/Camera.h"
#include "gl/ubo.h"

class world_environment;
struct VkRenderingInfo;
namespace vk14
{
struct scene_program;
struct target_formats;
class image;
class compute_kernel;
struct draw_constants;
enum class blend_mode;
class instance_cull;
struct image_use;
struct ray_draw;
} // namespace vk14

class vulkan14_renderer : public gfx_renderer, public gfx::scene_walk
{
  public:
	vulkan14_renderer();
	~vulkan14_renderer() override;

	// lifecycle
	bool Init(GLFWwindow *Window) override;
	bool AddViewport(global_settings::extraviewport_config const &conf) override;
	void Shutdown() override;
	bool Render() override;
	void SwapBuffers() override;
	float Framerate() override;
	// geometry
	gfx::geometrybank_handle Create_Bank() override;
	gfx::geometry_handle Insert(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type) override;
	gfx::geometry_handle Insert(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Geometry, int const Type) override;
	bool Replace(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type, std::size_t const Offset = 0) override;
	bool Replace_Indexed(gfx::index_array &Indices, gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry) override;
	bool Append(gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometry_handle const &Geometry, int const Type) override;
	gfx::index_array const &Indices(gfx::geometry_handle const &Geometry) const override;
	gfx::vertex_array const &Vertices(gfx::geometry_handle const &Geometry) const override;
	gfx::userdata_array const &UserData(gfx::geometry_handle const &Geometry) const override;
	int Primitive(gfx::geometry_handle const &Geometry) const override;
	// materials
	material_handle Fetch_Material(std::string const &Filename, bool const Loadnow = true) override;
	material_handle Matte_Material(material_handle const Material) override;
	material_handle Tinted_Material(material_handle const Material, glm::vec3 const &Colour) override;
	material_handle Blend_Material(material_handle const Material, std::array<material_handle, 3> const &Others) override;
	bool Blendable_Material(material_handle const Material) override;
	void Bind_Material(material_handle const Material, TSubModel const *sm = nullptr, lighting_data const *lighting = nullptr) override;
	IMaterial const *Material(material_handle const Material) const override;
	material_handle Terrain_Material(material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement,
	                                 int const HoleSamples = 0, std::uint8_t const *Holes = nullptr) override;
	void Terrain_Release(material_handle const Material) override;
	void Terrain_Grass(std::vector<gfx::grass_kind> const &Kinds, std::vector<gfx::grass_chunk> const &Chunks) override;
	// shaders
	std::shared_ptr<gfx::shader_program> Fetch_Shader(std::string const &name) override;
	std::string Shader_Name(std::shared_ptr<gfx::shader_program> const &Program) const override;
	// textures
	texture_handle Fetch_Texture(std::string const &Filename, bool const Loadnow = true, GLint format_hint = GL_SRGB_ALPHA) override;
	void Bind_Texture(texture_handle const Texture) override;
	void Bind_Texture(std::size_t const Unit, texture_handle const Texture) override;
	ITexture &Texture(texture_handle const Texture) override;
	ITexture const &Texture(texture_handle const Texture) const override;
	// picking
	void Pick_Control_Callback(std::function<void(TSubModel const *, glm::vec2 const)> Callback) override;
	void Pick_Node_Callback(std::function<void(scene::basic_node *)> Callback) override;
	TSubModel const *Pick_Control() const override;
	scene::basic_node const *Pick_Node() const override;
	glm::dvec3 Mouse_Position() const override;
	bool Mouse_Hit() const override;
	glm::mat4 Camera_View_Matrix() const override;
	glm::mat4 Camera_Projection_Matrix() const override;
	glm::dvec3 Camera_Position() const override;
	// maintenance
	void Update(double const Deltatime) override;
	void Update_Pick_Control() override;
	void Update_Pick_Node() override;
	glm::dvec3 Update_Mouse_Position() override;
	bool Debug_Ui_State(std::optional<bool> State) override;
	// debug
	std::string const &info_times() const override;
	std::string const &info_stats() const override;
	// ui
	imgui_renderer *GetImguiRenderer() override;
	void MakeScreenshot() override;
	bool Render_Preview(TAnimModel *Instance, int const Size, int const Margin, bool const Shadows, std::vector<std::uint8_t> &Image) override;

	static std::unique_ptr<gfx_renderer> create_func();
	// how the application makes the windows of the renderer, and whether it can run on the machine
	static gfx_backend_desc backend_desc();

  private:
	// the objects of the vulkan api (vulkan14context.h)
	struct context;

	// a viewport the scene is drawn to: the main window, an extra viewport (a window of its own or the main window drawn with
	// another projection)
	struct viewport_config
	{
		int width{0};
		int height{0};
		float draw_range{1.f};
		bool main{false};
		bool shadow{false}; // its view is taken into the sun shadows
		std::size_t window{0}; // the swapchain it goes to (vulkan14_renderer::context::windows)
		enum vp_type
		{
			normal,
			custom
		} proj_type{normal};
		viewport_proj_config projection{};
	};
	// the sun of the frame, with the factor of the place drawn (tunnels)
	struct sun_light : public basic_light
	{
		float factor{1.f};
	};

	using gfx::scene_walk::Render;
	using gfx::scene_walk::Render_Alpha;

	// the draws of the shared walk of the scene, recorded
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
	void walk_editor_cursor() override;
	gfx::geometrybank_manager &walk_geometry() override;
	bool walk_blending_enabled() const override;
	void walk_region_begin() override;
	bool walk_instances_begin() override;
	void walk_instances_end() override;

	// the gpu culling of the instances (vulkan14indirect.cpp, the stage P5; opengl46_renderer::Render_Instances_Indirect() and the
	// rest): the cullings of the passes and the depth pyramid, made when the renderer starts
	bool init_cull();
	void destroy_cull();
	// the rendering of the pass under way, kept to be begun again where it was: it's ended for the compute passes of the culling
	// in the middle of the walk, and begun again with what it drew loaded, with the state of the draws as it was
	void remember_rendering(VkRenderingInfo const &Info, std::vector<vk14::image_use> const &Uses);
	void suspend_rendering();
	void resume_rendering();
	// the pass sets with the output of the culling as the instance buffer, made when the pass sets or the output changed; bound by
	// the draws until they're dropped (nullptr)
	void use_cull_pass_sets(vk14::instance_cull *Cull);
	// writes the pass sets (Cull null) or those of the culling
	void write_pass_sets(vk14::instance_cull *Cull);
	// draws of the instances made, and the calls which made them
	struct indirect_counts
	{
		std::uint32_t draws{0};
		std::uint32_t calls{0};
		indirect_counts &operator+=(indirect_counts const &Other)
		{
			draws += Other.draws;
			calls += Other.calls;
			return *this;
		}
	};
	using command_set = gfx::instance_cull::command_set;
	// the instances of the main view drawn from its culling: the depth pyramid, the culling, the walks, the depth and the colour,
	// in two phases with the occlusion test of the instances drawn first; inside the depth pass of the scene the depth alone,
	// the colour waits for the colour pass (render_instances_indirect_pending())
	void render_instances_indirect();
	// the same for the passes other than the main view: no occlusion test, no depth prepass, one phase
	void render_instances_indirect_pass();
	void render_instances_indirect_pending();
	// the walks of the culled groups (gfx::scene_walk::Walk_Indirect_Draws()), the draws put in order for the multi draws, and the
	// commands filled in by the culling
	void record_indirect_draws(vk14::instance_cull &Cull, bool const Cascade);
	// the recorded draws with the commands of the set: their colour with the programs of their materials, their depth with the
	// programs of the depth prepass, the shadows with the programs of the shadows
	indirect_counts draw_indirect_colour(vk14::instance_cull &Cull, command_set const Set);
	indirect_counts draw_indirect_depth(vk14::instance_cull &Cull, command_set const Set);
	indirect_counts draw_indirect_shadow(vk14::instance_cull &Cull, command_set const Set);
	// Count recorded draws from First with the program (a multi draw with more than one) and the state of the draws
	bool draw_indirect(vk14::instance_cull &Cull, command_set const Set, std::size_t const First, std::uint32_t const Count, vk14::scene_program &Program, vk14::blend_mode const Blend);
	// the variant of a program for the multi draws, nullptr when there's none
	vk14::scene_program *multidraw_program(vk14::scene_program &Program);
	vk14::instance_cull &pass_cull();
	// the shadow maps of the lamps (vulkan14lamps.cpp, the stage P4; opengl46_renderer::Render_local_shadows() and the rest): the cab
	// lamp, the scenery lamps, one of them, the headlights, one set of them; the culling of a map (nullptr when its instances go
	// through the cpu); the rendering of the map set up by the maps; the maps put in the layout the shading reads them in
	void render_local_shadows();
	void render_lamp_shadows();
	void render_lamp_shadow(std::size_t const Slot, bool const Vehicles, TDynamicObject *Cab);
	void render_headlight_shadows();
	void render_headlight_shadow(std::size_t const Slot, light_array::light_record const &Light, TDynamicObject *Cab);
	vk14::instance_cull *light_shadow_cull(std::vector<std::unique_ptr<vk14::instance_cull>> &Culls, std::size_t const Index, bool const Enabled);
	void begin_local_shadow_rendering();
	void end_local_shadow_rendering();
	void ready_lamp_maps();
	bool m_cubeshadows{false}; // the shadow pass draws to a cube map, with the programs of the cube maps
	vk14::instance_cull *m_lightshadowcull{nullptr}; // the culling of the map of headlights or of a lamp being drawn
	std::size_t m_lampscenerydrawn{0}; // scenery parts of the maps of lamps drawn in this frame so far
	unsigned int m_instancecullframe{~0u}; // the frame the culling of the main view ran in last
	int m_shadowcascade{-1}; // the cascade of the sun shadows being drawn
	bool m_indirectmultidraw{false}; // the recorded draws are in order for the multi draws
	vk14::instance_cull *m_indirectcull{nullptr}; // the culling the recorded draws were made for
	command_set m_indirectcolourset{command_set::first}; // the commands the colour pass draws the pending instances with
	// the cascades of the sun shadows fitted to the depth of the main view (gfx.shadow.sdsm): the cascades of the frame fitted to the
	// latest measurement (opengl46_renderer::Shadow_Fit()), the measurement of the depth of the finished view, the state for the panel,
	// the area of the cascade being set up and the sizes setup_pass() found for it
	bool shadow_fit(viewport_config const &Viewport, std::array<float, gl::MAX_CASCADES + 1> &Splits, gfx::shadow_fit::areas &Areas);
	char const *shadow_fit_unavailable(viewport_config const &Viewport) const;
	void measure_shadow_fit(std::size_t const Viewport);
	gfx::shadow_fit::fit_state m_shadowfitstate;
	std::array<glm::vec3, 8> const *m_shadowfitarea{nullptr};
	float m_shadowslicesize{0.f};
	float m_shadowdrawnsize{0.f};

	// the scene (vulkan14scene.cpp)
	bool init_scene(std::string &Reason);
	// what the renderer prepares for the frames once it's there: the viewport of the main window, the common textures and geometry
	void init_frame_data();
	void destroy_scene();
	// the scene of the frame: the passes of the viewports
	void render_scene();
	void setup_pass(viewport_config &Viewport, renderpass_config &Config, rendermode const Mode, float const Znear = 0.f, float const Zfar = 1.f, bool const Ignoredebug = false);
	void setup_matrices();
	// the colour of a viewport, with the passes it needs first: the sun shadows, the environment map, the light clusters
	void render_viewport(std::size_t const Viewport);
	void render_shadows(viewport_config &Viewport);
	bool render_reflections(viewport_config &Viewport);
	void render_reflection_face(viewport_config &Viewport);
	// the ray traced reflections (gfx.raytracing, vulkan14raytracing.h): whether the colour pass of the viewport gets them, and the
	// scenery around the camera gathered by the walk in its reflection mode (the draws noted instead of made) and built into the
	// acceleration structures of the frame; a draw noted, with the state of the walk
	bool raytracing_wanted(viewport_config const &Viewport) const;
	void render_ray_scene(viewport_config &Viewport);
	void gather_ray_draw(gfx::geometry_handle const &Geometry);
	bool m_rtgather{false};
	bool m_raytracingon{true}; // switched off in the debug window
	bool m_rtfoliage{true}; // the alpha tested geometry in the traced reflections, switched off in the debug window
	double m_rtwalkms{0.0}; // the processor time of the walk and of the build of the ray tracing, averaged
	double m_rtbuildms{0.0};
	std::vector<vk14::ray_draw> m_rtdraws;
	std::vector<glm::mat4> m_rtinstances; // the modelviews of the instanced draws noted (walk_instances())
	void render_picking(rendermode const Mode);
	// the sky dome and the sun and the moon
	void render_environment(world_environment *Environment);
	void render_particles();
	void render_precipitation();
	void render_editor_overlay();
	// the post-processing of a viewport: the motion blur, the tonemapping, the chromatic aberration
	void render_post(std::size_t const Viewport, bool const Motionblur);
	// the output of a viewport to its window, with the user interface over it for the main window
	void present_viewport(std::size_t const Viewport);
	// the light buffer of the pass: the sun, the ambient light, the fog, the headlights, the lamps (the clustered lights)
	void update_lights(light_array &Lights, bool const Headlights = true, bool const Clustered = false);
	void write_local_light(gl::light_element &Light, gfx::local_lights::light const &Source, glm::dvec3 const &Camera, glm::mat4 const &View);
	void headlight_projector(light_array::light_record const &Light, glm::dvec3 &Position, glm::mat3 &Rotation, glm::mat4 &Projection, float &Near, float &Fieldofview);
	// the light buffer and its header written for the draws which follow
	void upload_light_header();
	void upload_lights();
	void upload_scene_block();
	void setup_sunlight_intensity(float const Factor = 1.f);
	void setup_shadow_color(glm::vec4 const &Shadowcolor);
	void setup_shadow_matrices();
	// the light clusters of the pass: the grid and the lists of the lights
	void update_clusters(glm::ivec2 const &Targetsize);
	// which program of the depth prepass stands for the program of the material: 0 none, 1 without the alpha test, 2 with it
	int prepass_class(material_handle const Material);
	// the depth of the whole opaque scene of the colour pass of the main viewport before its colour, for the contact shadows
	// (Contactonly: with them alone) or for itself (gfx.depthprepass.scene), and the contact shadows set up to read it
	void render_scene_depth(std::size_t const Viewport, glm::mat4 const &Future);
	bool contact_shadows_wanted(viewport_config const &Viewport) const;
	void setup_contact_shadows(std::size_t const Viewport);
	bool m_contactshadows{false};
	// the screen space reflections (gfx.ssr) of the main viewport: whether its colour pass gets them (they need the depth of the
	// scene drawn before the colour), the parameters of the pass set for them, the finished image of the frame (before the motion
	// blur) copied to the history with its mipmaps; the matrices and the state, begun for the frame and set up for its colour pass
	bool ssr_wanted(viewport_config const &Viewport) const;
	void setup_screen_reflections(std::size_t const Viewport, bool const Temporal);
	void store_ssr_history(std::size_t const Viewport, vk14::image &Source);
	gfx::screen_reflections m_ssr;
	bool m_ssrframe{false};
	bool m_ssractive{false};
	// the materials
	material_handle invalid_material() const
	{
		return m_invalid_material;
	}
	// the draws: the pipeline of the program for the state of the walk, made when it's first needed
	void draw_chunk(gfx::geometry_handle const &Geometry, bool const Newblock);
	// a pass of the scene recorded with the walk: its targets, its size, its pass set (vk14::scene_state::pass_sets)
	void start_draws(vk14::target_formats const &Targets, int const Width, int const Height, int const Set, bool const Shadows);
	void end_draws();
	// the rendering of the colour of a viewport, begun with the images cleared or again where it was
	void begin_colour_rendering(std::size_t const Viewport, bool const Clear);
	void push_constants(void const *Constants);
	// a pipeline of the post-processing (a VkPipeline), nullptr when the shader isn't there. Encode: the colour is encoded by the
	// shader (OUTPUT_GAMMA), -1: as gfx.shadergamma says
	void *post_pipeline(char const *Fragment, int const Format, int const Encode = -1);
	// the screenshot copied with a frame the gpu is done with, saved
	void update_screenshot();
	void bind_pass_set();
	void update_pass_sets();
	// the pass set of the colour of the main view bound for the compute kernels, before the dispatches of Kernel (the kernels share
	// the layout of set 0 and the push constants, so the set stays bound for the ones which follow)
	void bind_compute_set(vk14::compute_kernel const &Kernel);
	// the light of the sky (vulkan14sky.cpp): the resources, their first state, the light of the frame worked out before its
	// passes, the environment map prefiltered with the GGX lobe, the stars of the sky pass
	bool init_sky(std::string &Reason);
	void destroy_sky();
	void clear_sky_images();
	void update_sky();
	void prefilter_environment();
	void render_stars(glm::mat4 const &View);
	// the automatic exposure of the image of the main view, and its bloom chain; the result of the chain, null without it
	void measure_exposure(vk14::image &Image);
	vk14::image *render_bloom(std::size_t const Viewport, vk14::image &Image);
	// the weather (vulkan14weather.cpp): the resources, the noises of the clouds, the block of the pass, what the weather is for the
	// frame (before its passes), the clouds and the fog volume of the colour pass of the main view, the clouds and the fog over the sky
	bool init_weather(std::string &Reason);
	void destroy_weather();
	void make_cloud_noise();
	void upload_weather_block();
	void update_weather();
	void render_weather_view(glm::mat4 const &Projection, glm::mat4 const &View, glm::dvec3 const &Camera, glm::ivec2 const &Size, int const Cablamp);
	void render_sky_weather(glm::mat4 const &Projection, bool const Mainview);
	// the ambient occlusion (vulkan14ao.cpp): the resources, and the occlusion of a viewport multiplied into its colour
	bool init_ao(std::string &Reason);
	void destroy_ao();
	void render_ao(std::size_t const Viewport, int const Mode);
	// the particles of the gpu: what the colour pass under way draws them with, and the depth of the scene drawn so far copied for
	// the soft particles (false: they aren't soft)
	gfx::gpu_particles::view particles_view() const;
	bool setup_particles_depth(std::size_t const Viewport);
	bool m_particlesoft{false};
	// the grass of the heightmap terrain seen from the main view, over the opaque scene (opengl46_renderer::Render_grass())
	void render_grass(glm::mat4 const &Future);
	// a pass of the post-processing: the fragment shader drawn over the whole of the target, reading the images (Pass: a pass of
	// its own, for the gpu times). Repeated: a bit for each input read texel by texel and repeated (the noise of the ssao)
	void post_pass(char const *Fragment, vk14::image &Target, std::vector<vk14::image *> const &Inputs, vk14::draw_constants const &Constants, char const *Pass = nullptr, unsigned const Repeated = 0);
	bool bind_scene_pipeline(vk14::scene_program &Program, vk14::target_formats const &Targets, vk14::blend_mode const Blend, int const Topology);
	void apply_draw_state();
	void push_textures();
	std::uint32_t push_draw_block();
	void open_instance_window();
	void update_texture_release();

	std::unique_ptr<context> m_context;
	// the light of the sky and the weather of the frame, shared with the opengl renderer
	gfx::sky_light m_skylight;
	gfx::weather_state m_weatherstate;
	// the environment map prefiltered with the GGX lobe is bound for the pass (light_data.env_ggx)
	bool m_envmap_bound{false};
	// the data of the geometry, which the simulation reads back, and the banks with it on the gpu
	gfx::geometrybank_manager m_geometry;
	std::vector<gfx::geometry_bank *> m_banks;
	material_manager m_materials;
	material_handle m_invalid_material{null_handle};
	std::vector<viewport_config> m_viewports;
	viewport_config *m_current_viewport{nullptr};
	renderpass_config m_colorpass; // of the latest colour pass
	std::array<renderpass_config, 3> m_shadowpass; // of the latest cascades of the sun shadows
	sun_light m_sunlight;
	glm::vec3 m_fogcolor{0.f};
	glm::vec4 m_shadowcolor{colors::shadow};
	gl::scene_ubs scene_ubs{};
	gl::light_data light_data{};
	std::uint32_t m_lightcount{1}; // lights written to light_data
	// the copies of the light buffer in the ring of the frame since the lights were last written, by the header and the sun
	std::vector<std::pair<std::array<std::uint8_t, offsetof(gl::light_data, lights) + sizeof(gl::light_element)>, std::uint32_t>> m_lightheaders;
	packed_vec3 m_cabambient{0.f};
	gfx::local_lights m_locallights;
	std::uint32_t m_lights_lit{0};
	std::uint32_t m_lights_dropped{0};
	// the reflections: the face drawn, and the place and the time of the last update
	int m_environmentcubetextureface{0};
	int m_environmentupdateface{6};
	bool m_environmentready{false};
	double m_environmentupdatetime{0.0};
	glm::dvec3 m_environmentupdatelocation{0.0};
	bool m_vehiclesmoved{true};
	// the temporal anti-aliasing of the main viewport (gfx.antialiasing taa)
	gfx::temporal_aa m_taa;
	texture_handle m_suntexture{-1};
	texture_handle m_moontexture{-1};
	texture_handle m_smoketexture{-1};
	texture_handle m_headlightstexture{-1};
	texture_handle m_precipitationtexture{-1};
	float m_precipitationovercast{-1.f};
	double m_precipitationrotation{0.0};
	// the editor overlay
	gfx::geometrybank_handle m_editor_overlay_bank{};
	std::vector<gfx::geometry_handle> m_editor_overlay_geometry;
	unsigned int m_editor_overlay_revision{0};
	unsigned int m_nodestateversion{0};
	// picking
	std::vector<std::function<void(TSubModel const *, glm::vec2)>> m_control_pick_requests;
	std::vector<std::function<void(scene::basic_node *)>> m_node_pick_requests;
	TSubModel const *m_pickcontrolitem{nullptr};
	scene::basic_node *m_picksceneryitem{nullptr};
	bool m_pickcontrolswanted{false};
	bool m_picknodewanted{false};
	glm::ivec2 m_pickposition{0};
	glm::vec3 m_worldmousecoordinates{0.f};
	bool m_mousehit{false};
	// the state of the draws of the walk being recorded
	struct draw_state;
	std::unique_ptr<draw_state> m_draw;
	// textures used by vehicle models, for the release of far textures
	std::unordered_map<TModel3d const *, std::vector<texture_handle>> m_modeltextures;
	std::vector<texture_handle> const &Model_Textures(TModel3d *Model);

	// the season and the weather the materials were last given
	struct simulation_state
	{
		std::string weather;
		std::string season;
	} m_simulationstate;

	float m_framerate{0.f};
	double m_updateaccumulator{0.0};
	bool m_debug_ui_active{false};
	std::string m_infotimes;
	std::string m_infostats;
	debug_stats m_laststats;

	void update_info();
	// the debug windows (vulkan14debug.cpp, opengl46_renderer::draw_debug_ui()), the lines of the cullings and of the scene for them
	// and for the statistics, the counters of the light clusters read back, and the heatmap of the clusters over the output of a
	// viewport (opengl46_renderer::Render_cluster_heatmap())
	void draw_debug_ui();
	std::string cull_info();
	std::string scene_info();
	void collect_cluster_stats();
	void render_cluster_heatmap(std::size_t const Viewport);
	// the headlights (opengl46_renderer::headlight_config_s), set by the debug window
	struct headlight_config
	{
		float in_cutoff{1.005f};
		float out_cutoff{0.993f};
		float falloff_linear{0.15f};
		float falloff_quadratic{0.15f};
		float intensity{1.0f};
		float ambient{0.0f};
	} m_headlightconfig;
	// the device was lost: what was on the gpu goes to the log
	void report_device_lost(char const *Where);

	static bool renderer_register;
};
