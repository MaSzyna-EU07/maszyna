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
#include "model/material.h"
#include "gl/ubo.h"

#include <array>
#include <memory>
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
	particle, // position, colour and texture coordinates in one stream, the smoke
	position_uv // position and texture coordinates in two streams, the precipitation
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
};

// a picking or a reading of a value of the depth, waiting for the gpu to be done with the frame it was made in
struct readback
{
	std::shared_ptr<buffer> data;
	std::uint64_t value{0}; // of the timeline
	bool pending{false};
	glm::ivec2 position{0};
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
	VkImageView headlights_view{VK_NULL_HANDLE}; // the one the pass sets were made with
	cluster_state clusters;
	// the targets of the viewports, [0] the main window
	std::vector<viewport_targets> targets;
	// the formats of the passes which don't depend on the viewport
	target_formats shadow_formats;
	target_formats reflection_formats;
	target_formats pick_formats;
	std::uint32_t formats_count{0};
	// the programs: of the materials (by the name Fetch_Shader() is given), and the ones of the renderer
	std::unordered_map<std::string, std::shared_ptr<scene_program>> materials;
	std::shared_ptr<scene_program> shadow, alpha_shadow, pick, pick_surface, line, billboard, freespot, celestial, skydome, smoke, precipitation;
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
		VkCommandBuffer commands{VK_NULL_HANDLE};
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
	VkCommandBuffer commands{VK_NULL_HANDLE};

	~context()
	{
		destroy();
	}

	bool create_frames(std::uint32_t const Count, std::string &Reason);
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
	VkCommandBuffer commands{VK_NULL_HANDLE};
	vk14::target_formats const *targets{nullptr};
	VkExtent2D extent{0, 0};
	vk14::scene_state::pass_set_kind set{vk14::scene_state::set_plain};
	bool rendering{false}; // inside a rendering of a pass of the scene
	bool shadows{false}; // a pass of the sun shadows: the back faces are drawn
	// the offsets of the pass set in the rings of the frame
	std::uint32_t scene_offset{0};
	std::uint32_t light_offset{0};
	std::uint32_t cluster_offset{0};
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
};
