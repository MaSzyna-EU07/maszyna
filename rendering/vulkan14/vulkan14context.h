/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the objects of the vulkan api of the vulkan renderer (vulkan14_renderer::context), for the files of the renderer only:
// the core (vulkan14renderer.cpp) and the scene (vulkan14scene.cpp)

#pragma once

#include "rendering/vulkan14/vulkan14renderer.h"
#include "rendering/vulkan14/vulkan14common.h"
#include "rendering/vulkan14/vulkan14instance.h"
#include "rendering/vulkan14/vulkan14device.h"
#include "rendering/vulkan14/vulkan14swapchain.h"
#include "rendering/vulkan14/vulkan14memory.h"
#include "rendering/vulkan14/vulkan14shaders.h"
#include "rendering/vulkan14/vulkan14descriptors.h"
#include "rendering/vulkan14/vulkan14passes.h"
#include "rendering/vulkan14/vulkan14texture.h"
#include "rendering/vulkan14/vulkan14imgui.h"
#include "rendering/vulkan14/vulkan14bindings.h"
#include "rendering/vulkan14/vulkan14geometry.h"
#include "rendering/vulkan14/vulkan14compute.h"
#include "rendering/vulkan14/vulkan14cull.h"
#include "rendering/vulkan14/vulkan14shadows.h"
#include "rendering/vulkan14/vulkan14terrain.h"
#include "rendering/vulkan14/vulkan14grass.h"
#include "rendering/vulkan14/vulkan14particles.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "utilities/workerpool.h"
#include "model/material.h"
#include "gl/ubo.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace vk14
{

// how the vertices of a program of the scene come
enum class vertex_kind : std::uint8_t
{
	none, // made by the vertex shader (the celestial bodies, the full screen quads)
	scene, // gfx::basic_vertex and the user data, the geometry banks
	colour, // position and colour, the sky dome
	star, // direction with the magnitude and colour with the phase of the twinkling in one stream, the stars
	particle, // position, colour and texture coordinates in one stream, the smoke
	position_uv, // position and texture coordinates in two streams, the precipitation
	particle_instance // three vec4 of gl::particle_draw for each instance, the particles of the gpu
};

// the push constants of the pipelines of the scene and of the post-processing (shaders/vk14/common.glsl, draw_constants_block)
struct draw_constants
{
	std::uint32_t draw{0};
	std::uint32_t flags{0};
	std::uint32_t pad0{0};
	std::uint32_t pad1{0};
	glm::vec4 data[6]{};
};
static_assert(sizeof(draw_constants) == bindings::PUSH_CONSTANTS_SIZE, "the push constants don't match the shaders");

// a program of the scene: the shaders and the pipelines made of them for the targets and the states they were drawn with
struct scene_program : public gfx::shader_program
{
	std::shared_ptr<shader_code const> vertex;
	std::shared_ptr<shader_code const> fragment;
	// the fragment shader of the passes without the velocity target (NO_MOTION_OUTPUT), made when it's first needed
	std::string fragment_name;
	std::shared_ptr<shader_code const> fragment_single;
	// the vertex shader, and the definitions both stages were made with
	std::string vertex_name;
	std::string defines;
	// the variant of the multi draws of the instances drawn from the gpu culling (MODEL_DRAWS), made when it's first needed
	std::shared_ptr<scene_program> multidraw;
	bool multidraw_tried{false};
	vertex_kind vertices{vertex_kind::scene};
	std::unordered_map<std::uint64_t, VkPipeline> pipelines;
};

// what a pass of the scene draws to; the pipelines are made for these
struct target_formats
{
	std::array<VkFormat, 2> colour{VK_FORMAT_UNDEFINED, VK_FORMAT_UNDEFINED};
	std::uint32_t colours{0};
	VkFormat depth{VK_FORMAT_UNDEFINED};
	VkSampleCountFlagBits samples{VK_SAMPLE_COUNT_1_BIT};
	// the views drawn at once (multiview), 0: none
	std::uint32_t view_mask{0};
	// the number the pipelines of the targets are known by
	std::uint32_t id{0};
};

// the images a viewport draws its scene to, made for its size and the settings
struct viewport_targets
{
	VkExtent2D extent{0, 0};
	VkSampleCountFlagBits samples{VK_SAMPLE_COUNT_1_BIT};
	bool motion{false}; // the velocity is drawn, for the motion blur
	bool aberration{false};
	// with more than one sample: the images drawn to, resolved to the ones below
	std::shared_ptr<image> msaa_colour;
	std::shared_ptr<image> msaa_motion;
	std::shared_ptr<image> msaa_depth;
	// the colour of the scene (hdr), the velocity and the depth: the resolved ones, or the ones drawn to with one sample
	std::shared_ptr<image> colour;
	std::shared_ptr<image> motion_image;
	std::shared_ptr<image> depth;
	// the colour after the motion blur (hdr)
	std::shared_ptr<image> blurred;
	// the tonemapped image, before the chromatic aberration, and the one which goes to the window
	std::shared_ptr<image> tonemapped;
	std::shared_ptr<image> output;
	// the depth under the cursor of the editor: one sample of the depth of the ground resolved to it, and the buffer it's read back with
	std::shared_ptr<image> cursor_depth;
	// the depth of the opaque scene drawn before its colour, which the contact shadows read (the stage P3), and the formats of the
	// pass drawing it (the depth alone)
	std::shared_ptr<image> scene_depth;
	target_formats depth_formats;
	// the ambient occlusion (gfx.postfx.ao, the stage P3), half the size: the raw one and the smoothed one; made when it's on
	std::shared_ptr<image> ssao;
	std::shared_ptr<image> ssao_blur;
	// the formats of the passes drawn over the colour of the scene after it (the ambient occlusion): the colour alone
	target_formats post_formats;
	// the bloom chain (gfx.postfx.bloom, the stage P1): the levels down, half the size each, and up, one fewer
	std::vector<std::shared_ptr<image>> bloom_down;
	std::vector<std::shared_ptr<image>> bloom_up;
	std::vector<glm::ivec2> bloom_sizes;
	target_formats formats;
	target_formats output_formats;
	bool srgb{true}; // the output encodes the colour (gfx.shadergamma no)
};

// the ring of a frame some data of the draws goes through, the descriptor of which reads a window of it: the window starts again
// further on when it's full, and the pass set is bound again with the new offset
struct window_ring
{
	frame_ring ring;
	VkDeviceSize window{0}; // the size of the window, bytes
	VkDeviceSize stride{0};
	VkDeviceSize base{0}; // where the current window starts
	bool open{false};
};

// the light clusters (gl::cluster_grid), of the colour pass of each viewport in turn
struct cluster_state
{
	buffer aabbs;
	buffer ranges;
	buffer indices;
	buffer counters;
	gl::cluster_ubs params{};
	bool stale{true}; // the boxes have to be made anew
	VkDeviceSize params_offset{0}; // in the ring of the frame, for the pass under way
	std::uint32_t lights_total{0};
	// the counters of the cullings read back, for the debug panel (gl::cluster_grid::cull_stats): entries of the index list in
	// use, lights dropped from full clusters, the most of them seen
	readback_ring counters_readback;
	struct
	{
		std::uint32_t index_count{0};
		std::uint32_t overflow_count{0};
		std::uint32_t overflow_peak{0};
		bool valid{false};
	} stats;
};

// a picking or a reading of a value of the depth, waiting for the gpu to be done with the frame it was made in
struct readback
{
	std::shared_ptr<buffer> data;
	std::uint64_t value{0}; // of the timeline
	bool pending{false};
	glm::ivec2 position{0};
};

// the light of the sky on the gpu (the stage P1, opengl46_sky): the buffers the shading reads (the directional ambient light,
// the parameters of the atmosphere), the look-up tables of the atmosphere with the kernels making them and the result read
// back, the environment map prefiltered with the GGX lobe, the stars
struct sky_state
{
	buffer sky; // gl::sky_data
	buffer atmosphere; // gl::atmosphere_ubs
	buffer result; // gl::atmosphere_result, written by atmosphere_ambient.comp
	readback_ring result_readback;
	std::shared_ptr<image> transmittance;
	std::shared_ptr<image> multiscattering;
	std::shared_ptr<image> skyview;
	bool tables_ready{false}; // the tables were made once (they are black before)
	compute_kernel transmittance_kernel;
	compute_kernel multiscattering_kernel;
	compute_kernel skyview_kernel;
	compute_kernel ambient_kernel;
	// the prefiltered environment map, a cube view of each of its levels for the prefilter to write
	std::shared_ptr<image> ggx;
	std::vector<VkImageView> ggx_levels;
	bool ggx_ready{false};
	compute_kernel prefilter_kernel;
	// what the buffers were last given
	gl::sky_data uploaded{};
	bool sky_uploaded{false};
	// the dome drawn with the atmosphere, the stars of the catalogue
	std::shared_ptr<scene_program> atmosphere_dome;
	std::shared_ptr<scene_program> stars;
	std::shared_ptr<buffer> star_vertices;
	std::uint32_t star_count{0};
};

// the automatic exposure (the stage P1, opengl46_exposure): the histogram and the exposure on the gpu (gl::exposure_data), the
// kernels, the values read back for the debug window
struct exposure_state
{
	buffer data;
	compute_kernel histogram;
	compute_kernel average;
	readback_ring readback;
	float values[4]{1.f, 1.f, 1.f, 0.f};
	bool cleared{false};
};

// the weather on the gpu (the stage P2, opengl46_weather): the noises of the clouds, the map of their shadows, the clouds of the
// main view and their history, the panorama of the clouds for the other views, the fog volume of the main view and its
// history, with the kernels making them, and the program laying the clouds and the fog over the sky
struct weather_resources
{
	std::shared_ptr<image> shape_noise;
	std::shared_ptr<image> detail_noise;
	std::shared_ptr<image> coverage_noise;
	bool noise_made{false};
	std::shared_ptr<image> shadow_map;
	int shadow_size{0};
	// the clouds of the main view, at a part of its resolution
	glm::ivec2 view_size{0};
	std::shared_ptr<image> clouds_color;
	std::shared_ptr<image> clouds_depth;
	std::array<std::shared_ptr<image>, 2> clouds_history;
	int clouds_current{0}; // the history holding the latest result
	std::shared_ptr<image> panorama;
	glm::ivec2 panorama_size{0};
	// what the panorama is marched with as its depth, which it doesn't write
	std::shared_ptr<image> depth_dummy;
	// the fog volume: the scattering of the cells and its history, the fog integrated along the view
	glm::ivec3 fog_grid{0};
	std::array<std::shared_ptr<image>, 2> fog_scattering;
	int fog_current{0};
	std::shared_ptr<image> fog_integrated;
	compute_kernel noise;
	compute_kernel march;
	compute_kernel temporal;
	compute_kernel shadow;
	compute_kernel horizon;
	compute_kernel inject;
	compute_kernel integrate;
	std::shared_ptr<scene_program> sky;
};

// the gpu culling of the instances (the stage P5, Global.gfx_gpucull): the cullings of the passes, which share the records and
// the kernels with the one of the main view, and the depth pyramid of its occlusion test
struct cull_resources
{
	instance_cull main;
	std::array<instance_cull, 3> cascades;
	instance_cull reflections;
	// the maps of headlights, by the slot of the map, and the scenery parts of the maps of lamps, by their order among those drawn
	// in a frame: the passes of a frame don't share the buffers of a culling. made when first needed
	std::vector<std::unique_ptr<instance_cull>> headlights;
	std::vector<std::unique_ptr<instance_cull>> lamps;
	hiz_pyramid hiz;
	bool ready{false};
	// the cascades of the sun shadows fitted to the depth of the main view (gfx.shadow.sdsm), made when they're first needed
	shadow_fitter shadowfit;
	bool shadowfit_tried{false};
};

// what the scene is drawn with
struct scene_state
{
	// the layouts: of the pass set (set 0), the pushed set of the materials (set 1), of the pipelines of the scene, the
	// post-processing and the compute passes of the clusters
	VkDescriptorSetLayout pass_layout{VK_NULL_HANDLE};
	push_set material_set;
	VkPipelineLayout scene_layout{VK_NULL_HANDLE};
	push_set post_set;
	VkPipelineLayout post_layout{VK_NULL_HANDLE};
	push_set cluster_set;
	VkPipelineLayout cluster_layout{VK_NULL_HANDLE};
	// the pass sets: made with the pool when the images they read are made anew
	VkDescriptorPool pass_pool{VK_NULL_HANDLE};
	enum pass_set_kind
	{
		set_colour, // the shadow map and the environment map
		set_reflections, // the shadow map and the empty environment map
		set_plain, // the empty maps (shadows, picking)
		set_count
	};
	std::array<VkDescriptorSet, set_count> pass_sets{};
	bool pass_sets_stale{true};
	// the pass sets were made anew this many times (the ones of the cullings follow them)
	std::uint64_t pass_generation{0};
	// the rings of the draws: the blocks of the models (gl::model_ubs) and the modelview matrices of the instances
	window_ring draws;
	window_ring instances;
	// the images every pass may read: the stand-ins, the shadow maps, the environment map
	std::shared_ptr<image> white; // a texture of a material which isn't there
	std::shared_ptr<image> shadow_empty;
	std::shared_ptr<image> shadow_map;
	std::vector<VkImageView> shadow_layers; // a view of each cascade
	std::uint32_t shadow_size{0};
	std::shared_ptr<image> env_empty;
	std::shared_ptr<image> env_map;
	std::shared_ptr<image> env_depth;
	std::array<VkImageView, 6> env_faces{};
	std::shared_ptr<image> pick_colour;
	std::shared_ptr<image> pick_depth;
	// the stand-ins of the images of the stages P the pass sets hold until there are real ones: a volume (the fog volume, the noises
	// of the clouds), the cube shadow maps of the lamps, the shadow maps of the headlights, the depth of the scene
	std::shared_ptr<image> volume_empty;
	std::shared_ptr<image> cube_shadow_empty;
	std::shared_ptr<image> headlight_shadow_empty;
	std::shared_ptr<image> depth_empty;
	// the colours of the fog towards the horizon (the stage P2), a block of the pass sets
	buffer weather_horizon;
	VkImageView headlights_view{VK_NULL_HANDLE}; // the one the pass sets were made with
	VkSampler headlights_sampler{VK_NULL_HANDLE};
	// the views of the images of the bindings of each pass set it was made with (vulkan14scene.cpp, pass_set_images())
	std::array<std::array<VkImageView, bindings::PASS_BINDING_COUNT>, 3> pass_images{};
	cluster_state clusters;
	// the ambient occlusion: the noise of the older ssao, the kernels of the ground truth one, the program applying it
	std::shared_ptr<image> ssao_noise;
	compute_kernel gtao_main;
	compute_kernel gtao_denoise;
	std::shared_ptr<scene_program> ao_apply;
	sky_state sky;
	exposure_state exposure;
	weather_resources weather;
	cull_resources cull;
	// the shadow maps of the lamps and the headlights (the stage P4), and the programs drawing the cube maps
	local_shadow_maps lamps;
	std::shared_ptr<scene_program> cube_shadow, cube_alpha_shadow;
	target_formats cube_formats;
	// the materials of the heightmap terrain (the stage P6), made with the bank of the textures
	std::unique_ptr<terrain_materials> terrain;
	// the grass of the heightmap terrain, and its program, made when it's first drawn
	grass_field grass;
	std::shared_ptr<scene_program> grass_program;
	bool grass_tried{false};
	// the smoke and the rain or snow of the gpu
	gpu_particles particles;
	// the heatmap of the light clusters (gfx.lights.clusterheatmap), made when it's first drawn
	std::shared_ptr<scene_program> cluster_heatmap;
	bool cluster_heatmap_tried{false};
	// the targets of the viewports, [0] the main window
	std::vector<viewport_targets> targets;
	// the format of the levels of the bloom chains: no alpha, no negative values, half the memory of rgba16f, where the device
	// draws to it
	VkFormat bloom_format{VK_FORMAT_B10G11R11_UFLOAT_PACK32};
	// the formats of the passes which don't depend on the viewport
	target_formats shadow_formats;
	target_formats reflection_formats;
	target_formats pick_formats;
	std::uint32_t formats_count{0};
	// the programs: of the materials (by the name Fetch_Shader() is given), and the ones of the renderer
	std::unordered_map<std::string, std::shared_ptr<scene_program>> materials;
	std::shared_ptr<scene_program> shadow, alpha_shadow, pick, pick_surface, line, billboard, freespot, celestial, skydome, smoke, precipitation;
	// the depth prepass of the scene (the stage P3): without and with the alpha test, and which of them a program of a material takes
	std::shared_ptr<scene_program> prepass, prepass_alpha;
	std::unordered_map<gfx::shader_program const *, int> prepass_classes;
	// the post-processing, by the fragment shader
	std::unordered_map<std::string, std::shared_ptr<shader_code const>> post_fragments;
	std::shared_ptr<shader_code const> quad;
	VkPipeline cluster_build{VK_NULL_HANDLE};
	VkPipeline cluster_cull{VK_NULL_HANDLE};
	// the reading of the colour under the cursor of the picking passes and of the depth under the cursor of the editor
	readback pick_controls;
	readback pick_scenery;
	readback cursor;
	// the screenshot asked for, copied with the frame
	std::shared_ptr<buffer> screenshot;
	std::uint64_t screenshot_value{0};
	VkExtent2D screenshot_extent{0, 0};
	bool screenshot_pending{false};
	bool screenshot_wanted{false};
	// the occlusion queries of the points of light of the lamps, a pool for each frame in flight with the submodels they were made
	// for; the glare of a lamp is drawn where its point was seen in the frame the gpu was done with last
	std::vector<VkQueryPool> occlusion_pools;
	std::vector<std::vector<TSubModel const *>> occlusion_submodels;
	std::unordered_map<TSubModel const *, bool> glare_visible;
	static constexpr std::uint32_t occlusion_queries{1024};
	// the images of the scene were put in their layouts (the stand-ins cleared)
	bool images_ready{false};
};

} // namespace vk14

// the objects of the vulkan api. destroyed in the reverse order: the swapchains and the frames, then the device, then the instance
struct vulkan14_renderer::context
{
	struct frame
	{
		VkCommandPool pool{VK_NULL_HANDLE};
		// the uploads of the frame go to the queue first, then the commands of the frame
		VkCommandBuffer uploads{VK_NULL_HANDLE};
		// the calls made while the frame is made, recorded into the command buffers once it's done (M1): the uploads into theirs,
		// the commands chunk by chunk by the workers
		vk14::command_stream upload_stream;
		vk14::command_stream commands;
		// a command pool for each worker, with the command buffers it made
		struct worker_commands
		{
			VkCommandPool pool{VK_NULL_HANDLE};
			std::vector<VkCommandBuffer> buffers;
			std::size_t used{0};
		};
		std::vector<worker_commands> workers;
		// the command buffers of the chunks of the frame, in their order
		std::vector<VkCommandBuffer> chunks;
		// the value of the timeline semaphore the gpu reaches when it's done with the frame; 0: never submitted
		std::uint64_t done{0};
		// the number of the frame, for the report of a lost device
		std::uint64_t number{0};
	};

	vk14::instance instance;
	vk14::device device;
	vk14::allocator memory;
	vk14::deletion_queue deletions;
	vk14::uploader uploads;
	vk14::image_table images;
	vk14::samplers sampling;
	vk14::frame_ring ring;
	vk14::shader_library shaders;
	vk14::pipeline_library pipelines;
	vk14::pass_list passes;
	vk14::pending_textures pending;
	vk14::resources resources;
	vk14::imgui_backend imgui;
	vk14::scene_state scene;
	// the textures of the files; made after the objects above and gone before them
	std::unique_ptr<texture_bank> textures;
	// [0]: the main window, then the windows of the extra viewports
	std::vector<std::unique_ptr<vk14::swapchain>> windows;
	std::vector<frame> frames;
	// one semaphore counts the frames done by the gpu; the binary ones are only between the swapchains and the queue
	VkSemaphore timeline{VK_NULL_HANDLE};
	std::uint64_t timelinevalue{0};
	// frames recorded since the start
	std::uint64_t framecount{0};
	// the windows with an image acquired in the frame being made
	std::vector<vk14::swapchain *> acquired;
	bool devicelost{false};
	// the frame being recorded: its index among the frames in flight, its commands
	std::uint32_t slot{0};
	vk14::command_stream *commands{nullptr};
	// the threads recording the command buffers of the frames: a pool of its own, the one of the application is for the simulation
	worker_pool workers;
	// the cpu time the command buffers of the last frame took to record, in milliseconds; its calls and chunks
	std::atomic<double> record_time{0.0};
	std::atomic<std::size_t> record_calls{0};
	std::atomic<std::size_t> record_chunks{0};
	std::atomic<std::size_t> record_threads{0};
	// what's left of a frame made by the main thread: recording its command buffers, submitting them, presenting its windows
	struct render_job
	{
		frame *target{nullptr};
		bool uploaded{false};
		std::vector<VkSemaphoreSubmitInfo> waits;
		std::vector<VkSemaphoreSubmitInfo> signals;
		// the windows presented once the frame is submitted (the render thread)
		std::vector<vk14::swapchain *> windows;
	};
	// the render thread and the job it's given
	std::thread render_thread;
	std::mutex render_mutex;
	std::condition_variable render_wake;
	std::condition_variable render_idle;
	render_job render_pending;
	bool render_queued{false}; // render_pending holds a job not done yet
	bool render_stop{false};
	char const *render_failure{nullptr};
	// how long the main thread waited for the render thread at the start of the last frame, in milliseconds
	double render_wait{0.0};

	~context()
	{
		destroy();
	}

	bool create_frames(std::uint32_t const Count, std::string &Reason);
	// the streams of Frame recorded into its command buffers; false: no command buffer for them
	bool record(frame &Frame, bool Uploads);

	// the job done; false: Failure says where it failed
	bool execute(render_job &Job, char const *&Failure);
	// the windows of the frame submitted last presented; false: the device or a surface is lost
	bool present(std::vector<vk14::swapchain *> const &Windows);
	// the render thread (gfx.vulkan.renderthread, M4): the frames are recorded, submitted and presented by it while the main
	// thread goes on with the simulation of the next one. the main thread waits for it to be done before it begins a frame and
	// before anything else touches the queue, the swapchains or the device as a whole
	void start_render_thread();
	void stop_render_thread();
	bool render_threaded() const
	{
		return render_thread.joinable();
	}
	// the job given to the render thread
	void hand_over(render_job &&Job);
	// waits until the render thread is done with the jobs given to it; where it failed, if it did
	char const *drain();
	// the parts of the renderer made once the device is there
	bool create_core(std::uint32_t const Frames, std::string &Reason);
	// the value of the timeline the gpu reached, 0 when it can't be read
	std::uint64_t completed() const;
	// waits until the gpu reached the value of the timeline; false: the device is lost
	bool wait(std::uint64_t const Value) const;
	void destroy();
};

// the state of the draws of the walk being recorded into a pass of the scene
struct vulkan14_renderer::draw_state
{
	vk14::command_stream *commands{nullptr};
	vk14::target_formats const *targets{nullptr};
	VkExtent2D extent{0, 0};
	vk14::scene_state::pass_set_kind set{vk14::scene_state::set_plain};
	bool rendering{false}; // inside a rendering of a pass of the scene
	bool shadows{false}; // a pass of the sun shadows: the back faces are drawn
	// the offsets of the pass set in the rings of the frame
	std::uint32_t scene_offset{0};
	std::uint32_t light_offset{0};
	std::uint32_t cluster_offset{0};
	std::uint32_t weather_offset{0};
	bool set_dirty{true};
	// the program and the textures for the next draw (walk_material(), walk_program(), walk_texture())
	vk14::scene_program *program{nullptr};
	std::array<texture_handle, gl::MAX_TEXTURES> textures{};
	bool textures_dirty{true};
	// the block of the draw last pushed, for the chunks drawn together
	std::uint32_t block{0};
	// what is bound
	VkPipeline pipeline{VK_NULL_HANDLE};
	VkBuffer vertices{VK_NULL_HANDLE};
	VkBuffer userdata{VK_NULL_HANDLE};
	VkBuffer indices{VK_NULL_HANDLE};
	VkPrimitiveTopology topology{VK_PRIMITIVE_TOPOLOGY_MAX_ENUM};
	VkCullModeFlags cull{VK_CULL_MODE_FLAG_BITS_MAX_ENUM};
	int depthtest{-1};
	int depthwrite{-1};
	std::uint32_t pushed_draw{~0u};
	// the state of the walk (setup_drawing() and the glare of the opengl renderer)
	bool blending{false};
	bool cullface{true};
	bool glare{false};
	bool depthmask_off{false}; // the editor overlay: no depth written whatever the blending
	// the occlusion queries of the glare of the lamps in the frame: the submodels, by the index of their query
	std::vector<TSubModel const *> queries;
	std::uint32_t query{~0u}; // the query under way
	// the draws made in the pass, for the debug panel
	std::uint32_t draws{0};
	// the depth of the scene before its colour (the stage P3): the walk draws with the programs of the depth prepass, and what they
	// don't draw is left out (depthskip, set by walk_material())
	bool depthpass{false};
	bool depthskip{true};
	// the pass sets of a culling, the instance buffer its output, for the indirect draws of its instances; null: the pass sets
	VkDescriptorSet const *cull_sets{nullptr};
	// the rendering under way, begun again where it was after it was ended for the compute passes of the culling
	struct rendering_resume
	{
		std::array<VkRenderingAttachmentInfo, 2> colours{};
		VkRenderingAttachmentInfo depth{};
		bool has_depth{false};
		VkRenderingInfo info{};
		std::vector<vk14::image_use> uses;
		bool valid{false};
	} resume;
};
