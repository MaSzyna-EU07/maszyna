/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <optional>
#include <vector>

#include "gl/ubo.h"

class opengl_camera;

namespace gfx
{

// the cpu side of the cascades of the sun shadows fitted to the depth of the main view (sample distribution shadow maps,
// gfx.shadow.sdsm), shared by the renderers (opengl46_shadow_fit, the vulkan renderer). at the end of the colour pass of the main
// view two compute passes go over its resolved depth: the nearest and the farthest distance of the pixels which aren't the sky
// (shadow_fit_depth.comp), then the distances split between the cascades and, for each cascade, the box of the pixels it takes
// in the space of the light (shadow_fit_bounds.comp). the result comes back to the cpu a frame or two later without waiting,
// with the camera and the light it was measured with, and the cascades of the frame cover the boxes (fit()) instead of the
// whole slices of the view between the fixed distances of shadowtune
class shadow_fit
{
  public:
	// the view and the light a measurement was made with
	struct view
	{
		glm::dvec3 camera; // position of the camera
		glm::mat4 projection; // projection of the view
		glm::mat3 rotation; // world to view space, rotation only
		glm::mat3 light; // world to the space of the light, rotation only
		float range{0.f}; // the farthest distance the cascades may reach
	};
	// a measurement which came back
	struct result
	{
		gl::shadow_fit_data data;
		view source;
		unsigned int frame{0}; // frame of the renderer it was made in
	};
	// what the last shadow pass did, for the debug panel
	struct fit_state
	{
		bool fitted{false};
		char const *reason{nullptr}; // why not, with gfx.shadow.sdsm on
		unsigned int age{0}; // frames since the measurement
		float depthmin{0.f};
		float depthmax{0.f};
		float turn{0.f}; // of the view since the measurement, degrees
		std::array<float, gl::MAX_CASCADES + 1> splits{};
		// size of the area of the map of each cascade: of the whole slice of the view, and as drawn; metres
		std::array<float, gl::MAX_CASCADES> slice{};
		std::array<float, gl::MAX_CASCADES> drawn{};
	};
	using areas = std::array<std::optional<std::array<glm::vec3, 8>>, gl::MAX_CASCADES>;

	// a measurement made: its number in the data the passes start from (no pixels, the nearest distance infinite, the bounds
	// empty), kept with the view until it comes back
	gl::shadow_fit_data begin_measurement(view const &View, unsigned int const Frame);
	// the parameters of shadow_fit_bounds.comp: the range, the nearest end of the first split, the least far end of the last
	// one, the split between the logarithmic and the even splits; and the margins of the bounds
	static glm::vec4 bounds_params(view const &View);
	static glm::vec4 margin_params();
	// a measurement which came back from the gpu
	void arrived(gl::shadow_fit_data const &Data);
	// the newest measurement taken in so far
	result const *latest() const
	{
		return m_latest ? &(*m_latest) : nullptr;
	}
	// the ends of the bounds, as the passes store them (shadow_fit_order() in shadow_fit_bounds.comp)
	static float unorder(std::uint32_t const Value);

	// the cascades of the frame fitted to the latest measurement: the distances they go from and to, and for each cascade the
	// corners of the area its map is to cover, camera-relative world space (none: the whole slice of the view). false: the fixed
	// cascades. Camera: of the colour pass the shadows are made for; Unavailable: why the cascades can't be fitted, or null
	bool fit(unsigned int const Frame, opengl_camera const &Camera, char const *Unavailable, std::array<float, gl::MAX_CASCADES + 1> &Splits, areas &Areas, fit_state &State) const;

	// the direction the sun shadows are cast in (the sun, its height capped by gfx.shadow.angle.min), and the rotation from the
	// world to the space of the light of the cascades: x and y across the maps, z along the light
	static glm::vec3 light_vector(glm::vec3 const &Sun);
	static glm::mat3 light_rotation(glm::vec3 const &Sun);
	// the distances the fixed cascades of the sun shadows go from and to (shadowtune)
	static std::array<float, gl::MAX_CASCADES + 1> fixed_splits();
	// why the cascades can't be fitted to the depth of a viewport, null if they can. Postprocessing: the viewport has the images
	// of the post-processing (the resolved depth); Shadow: its view is taken into the shadows; Shadowviews: the viewports which are
	static char const *unavailable(bool const Postprocessing, bool const Shadow, std::size_t const Shadowviews);
	// the points of the slice of the view a shadow pass covers (setup_pass()), in its place the corners of the area fitted to the
	// depth, when there is one (and it lies within the slice); Slicesize: the size of the slice across the light. returns the step
	// the bounds of the map are quantized to, 0 when the area isn't fitted
	static float fit_slice(std::vector<glm::vec4> &Points, std::array<glm::vec3, 8> const *Area, glm::mat3 const &Lightrotation, float &Slicesize);

  private:
	// a measurement on its way back, kept by its number
	struct pending
	{
		std::uint32_t measurement{0}; // 0: none
		view source;
		unsigned int frame{0};
	};
	static constexpr std::size_t PENDING = 8; // more than the copies the readbacks keep in flight

	std::array<pending, PENDING> m_pending;
	std::uint32_t m_measurement{0}; // number of the last measurement
	std::optional<result> m_latest;
};

} // namespace gfx
