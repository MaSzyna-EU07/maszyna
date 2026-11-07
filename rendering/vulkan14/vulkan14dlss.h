/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// nvidia dlss (the super resolution of the ngx sdk, built with WITH_DLSS) of the main viewport, gfx.antialiasing dlss: the colour of
// the scene with its depth and velocities, drawn at the size gfx.buffer.percent gives, anti-aliased and upscaled to the size of the
// window. dlaa is the same at the full size. the sdk isn't part of the repository (tools/fetch_dlss_sdk.cmake, DLSS_SDK_ROOT); without
// it dlss is never available and the renderer takes taa.
//
// the frame generation of dlss (gfx.framegeneration x2|x3|x4, with dlss or dlaa) makes one to three frames between the last frame
// and this one, from the images of both with the depth and the velocities; they are shown before this one, the presents spaced
// over the time of a frame (vulkan14_renderer::context::present())

#pragma once

#include "rendering/vulkan14/vulkan14common.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace vk14
{

class command_stream;
class image;
class deletion_queue;

class dlss
{
  public:
	// what a frame gives it, the images in the states the evaluation takes (the inputs read by the shaders, the output a storage
	// image in the general layout)
	struct frame_inputs
	{
		image *colour{nullptr}; // hdr, linear, before the exposure
		image *depth{nullptr}; // reverse z
		image *motion{nullptr}; // the velocities to the last frame, in parts of the image (uv), without the offset of the frame
		image *exposure{nullptr}; // 1x1, the exposure the tonemapping applies; null: dlss works it out itself
		image *output{nullptr}; // the size of the window
		// the offset of the projection of the frame in pixels of the colour, along its columns and rows
		glm::vec2 jitter{0.f};
		bool reset{false}; // the last frames have nothing to do with this one
		float frame_time{0.f}; // milliseconds
	};

	// what the frame generation is given, the images in the general layout: the backbuffer and the hudless the size and the format
	// of the outputs (what the window shows, the rows from the top down), the depth and the velocities the size the scene is drawn
	// at, turned over to the same way up (postfx_fg_flip.frag)
	struct generation_inputs
	{
		image *backbuffer{nullptr}; // the frame as the window shows it, the user interface with it
		image *hudless{nullptr}; // the same without the user interface
		image *depth{nullptr}; // r32, reverse z
		image *motion{nullptr}; // the velocities dlss takes, in parts of the image (uv) along the rows of the images of the renderer
		std::vector<image *> outputs; // the frames generated, in the order they are shown
		// the matrices without the offset of the frame, for the clip space of the image of the renderer (the rows from the bottom up)
		glm::mat4 view_to_clip{1.f};
		glm::mat4 clip_to_previous_clip{1.f};
		glm::vec3 position{0.f}; // the camera in the world, and its axes
		glm::vec3 up{0.f, 1.f, 0.f};
		glm::vec3 right{1.f, 0.f, 0.f};
		glm::vec3 forward{0.f, 0.f, -1.f};
		float znear{0.1f};
		float zfar{1000.f};
		float fov{1.f}; // vertical, in radians
		float aspect{1.f};
		glm::vec2 jitter{0.f}; // as the frame_inputs, in pixels of the depth
		bool reset{false};
	};

	dlss();
	~dlss();
	dlss(dlss const &) = delete;
	dlss &operator=(dlss const &) = delete;

	// the program was built with the sdk
	static bool compiled();
	// the extensions of the instance and of the device ngx needs; asked for before the instance and the device are made, they are
	// taken where the driver has them
	static void instance_extensions(std::vector<std::string> &Extensions);
	static void device_extensions(VkInstance Instance, VkPhysicalDevice Physical, std::vector<std::string> &Extensions);

	// ngx on the device, and whether the driver and the gpu give dlss. false: Reason tells why
	bool init(VkInstance Instance, VkPhysicalDevice Physical, VkDevice Device, std::string &Reason);
	// the gpu is done with everything (the device is idle)
	void shutdown();
	bool available() const;
	// the size the scene is drawn at for the window of the size Output and the part Scale of it (1: dlaa), kept within what dlss takes
	// for the mode it picks for them. false: dlss can't upscale to Output
	bool plan(glm::ivec2 const &Output, float Scale, glm::ivec2 &Render);
	// for the sizes planned: the bias of the mipmaps of the textures of the scene, the number of the offsets the jitter goes through
	float lod_bias() const;
	std::uint32_t jitter_phases() const;
	// the evaluation recorded to Commands (the replay does it on the command buffer of the frame). the feature is made at the first
	// one and again when the sizes or the settings change; the old one goes once the gpu is past Value of the timeline
	void evaluate(command_stream &Commands, frame_inputs const &Inputs, deletion_queue &Deletions, std::uint64_t Value);
	// what the replay of the last frame left (the feature made, an error) taken over for the main thread, once the replay is done
	// (after the wait for the render thread); what went wrong, the first time, or empty
	std::string sync();
	// the feature couldn't be made or evaluated: dlss stays off from then on (as of the last sync())
	bool failed() const;
	// the most frames the frame generation makes between two frames drawn (1 for x2, 3 for x4), 0 where it isn't available (not
	// asked for at the start, an older gpu than the rtx 40 series, the driver)
	std::uint32_t generation_max() const;
	// the frames between the last frame and this one recorded to Commands (Inputs.outputs.size() of them); where the feature can't be
	// made or evaluated the outputs get the backbuffer and the frame generation stays off from then on (generation_failed())
	void generate(command_stream &Commands, generation_inputs const &Inputs, deletion_queue &Deletions, std::uint64_t Value);
	bool generation_failed() const;
	// the mode and the sizes, for the statistics (as of the last sync())
	std::string info() const;
	// the part of the debug window
	void draw_debug_ui();

	struct state;

  private:
	std::unique_ptr<state> m_state;
};

} // namespace vk14
