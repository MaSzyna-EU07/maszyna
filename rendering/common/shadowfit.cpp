/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/shadowfit.h"

#include <cstring>

#include "rendering/openglcamera.h"
#include "utilities/Globals.h"

namespace gfx
{

namespace
{

// the nearest distance the first cascade may start from and the least one the last reaches, metres: the cascades don't
// get any closer to the camera than this, however close the pixels are
float const SPLIT_NEAR_MIN{0.5f};
float const SPLIT_FAR_MIN{16.f};
// how far past its distances a cascade takes the pixels, this many metres and this share of the distance: the camera moves
// in the frame or two before the cascades are drawn from the measurement, and a pixel which came into another cascade since
// has to be within its map
float const MARGIN_METRES{1.f};
float const MARGIN_SHARE{0.05f};

} // namespace

gl::shadow_fit_data shadow_fit::begin_measurement(view const &View, unsigned int const Frame)
{
	// numbers start from 1, 0 is no measurement
	if (++m_measurement == 0)
		++m_measurement;
	gl::shadow_fit_data data{};
	data.measurement = m_measurement;
	data.depthmin = std::numeric_limits<float>::infinity();
	for (std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade)
	{
		for (std::size_t axis = 0; axis < 4; ++axis)
		{
			data.bounds[cascade * 8 + axis] = 0xFFFFFFFFu;
			data.bounds[cascade * 8 + 4 + axis] = 0u;
		}
	}
	m_pending[m_measurement % PENDING] = pending{m_measurement, View, Frame};
	return data;
}

glm::vec4 shadow_fit::bounds_params(view const &View)
{
	auto const farmin{std::min(SPLIT_FAR_MIN, View.range)};
	return glm::vec4{View.range, std::min(SPLIT_NEAR_MIN, farmin * 0.25f), farmin, std::clamp(Global.gfx_shadow_sdsm_split, 0.f, 1.f)};
}

glm::vec4 shadow_fit::margin_params()
{
	return glm::vec4{MARGIN_METRES, MARGIN_SHARE, 0.f, 0.f};
}

void shadow_fit::arrived(gl::shadow_fit_data const &Data)
{
	auto const &source{m_pending[Data.measurement % PENDING]};
	if ((Data.measurement != 0) && (source.measurement == Data.measurement))
		m_latest = result{Data, source.source, source.frame};
}

float shadow_fit::unorder(std::uint32_t const Value)
{
	auto const bits{((Value & 0x80000000u) != 0u) ? (Value & 0x7FFFFFFFu) : ~Value};
	float result;
	std::memcpy(&result, &bits, sizeof(result));
	return result;
}

glm::vec3 shadow_fit::light_vector(glm::vec3 const &Sun)
{
	// cap the vertical angle to keep shadows from getting too long
	return glm::normalize(glm::vec3{Sun.x, std::min(Sun.y, Global.gfx_shadow_angle_min), Sun.z});
}

glm::mat3 shadow_fit::light_rotation(glm::vec3 const &Sun)
{
	// the rotation of the view of the shadow pass (setup_pass())
	return glm::mat3{glm::lookAt(glm::vec3{0.f}, light_vector(Sun), glm::vec3{0.f, 1.f, 0.f})};
}

std::array<float, gl::MAX_CASCADES + 1> shadow_fit::fixed_splits()
{
	static_assert(gl::MAX_CASCADES == 3, "the fixed cascades are made for 3 of them");
	std::array<float, gl::MAX_CASCADES + 1> splits{0.0f, Global.shadowtune.range / 32, Global.shadowtune.range / 8, Global.shadowtune.range};
	if (Global.shadowtune.map_size > 2048)
	{
		// increase coverage if our shadow map is large enough to produce decent results
		for (std::size_t idx = 1; idx < splits.size(); ++idx)
			splits[idx] *= Global.shadowtune.map_size / 2048;
	}
	return splits;
}

char const *shadow_fit::unavailable(bool const Postprocessing, bool const Shadow, std::size_t const Shadowviews)
{
	if (false == Global.gfx_shadowmap_enabled)
		return "no shadow maps";
	if (Global.bWireFrame)
		return "wireframe";
	// the shadows follow the camera, the view shows another one
	if (DebugCameraFlag)
		return "debug camera";
	if (false == Postprocessing)
		return "no post processing";
	if (EditorModeFlag && Global.EditorOrtho)
		return "orthographic view";
	if ((false == Shadow) || (Shadowviews != 1))
		return "the shadows cover more than one view";
	return nullptr;
}

bool shadow_fit::fit(unsigned int const Frame, opengl_camera const &Camera, char const *Unavailable, std::array<float, gl::MAX_CASCADES + 1> &Splits, areas &Areas, fit_state &State) const
{
	// a measurement older than this many frames, made with the camera this many metres away or the view turned this many
	// degrees from the one now, isn't used
	constexpr unsigned int AGE_MAX{6};
	constexpr float SHIFT_MAX{25.f};
	constexpr float TURN_MAX{20.f};
	// metres an area grows by on each side: the vehicles move in the frames since the measurement too
	constexpr float MARGIN{0.25f};

	auto &state{State};
	state = fit_state{};
	if (false == Global.gfx_shadow_sdsm)
		return false;
	if ((state.reason = Unavailable) != nullptr)
		return false;
	auto const *latest{this->latest()};
	if (latest == nullptr)
	{
		state.reason = "no measurement yet";
		return false;
	}
	auto const &data{latest->data};
	auto const &source{latest->source};
	state.age = Frame - latest->frame;
	state.depthmin = data.depthmin;
	state.depthmax = data.depthmax;
	if (state.age > AGE_MAX)
	{
		state.reason = "the measurement is too old";
		return false;
	}
	if (data.samples == 0)
	{
		state.reason = "only the sky in the view";
		return false;
	}
	if (source.range != fixed_splits().back())
	{
		state.reason = "the range of the shadows changed";
		return false;
	}
	// the shadows of this frame are made for its colour pass
	auto const &camera{Camera};
	auto const shift{glm::vec3{source.camera - camera.position()}};
	if (glm::length(shift) > SHIFT_MAX)
	{
		state.reason = "the camera jumped";
		return false;
	}
	auto const turnmatrix{glm::mat3{camera.modelview()} * glm::transpose(source.rotation)};
	auto const turn{std::acos(std::clamp((turnmatrix[0][0] + turnmatrix[1][1] + turnmatrix[2][2] - 1.f) * 0.5f, -1.f, 1.f))};
	state.turn = glm::degrees(turn);
	if (state.turn > TURN_MAX)
	{
		state.reason = "the view turned too fast";
		return false;
	}
	auto const &projection{camera.projection()};
	if ((std::abs(projection[0][0] - source.projection[0][0]) > 0.02f * std::abs(projection[0][0])) || (std::abs(projection[1][1] - source.projection[1][1]) > 0.02f * std::abs(projection[1][1])))
	{
		state.reason = "the field of view changed";
		return false;
	}

	// the first cascade from the camera, like the fixed ones. setup_pass() takes distances up to 1 m as shares of the far
	// end, so the others start past that
	Splits[0] = 0.f;
	for (std::size_t split = 1; split < Splits.size(); ++split)
		Splits[split] = std::max(data.splits[split], Splits[split - 1] + 1.5f);
	if (false == std::isfinite(Splits.back()))
	{
		state.reason = "invalid measurement";
		return false;
	}

	auto const inverselight{glm::transpose(source.light)};
	// the scenery stays where it was while the camera moved, the cab and the train the camera rides with moved along. an area
	// covers both
	auto const lightshift{source.light * shift};
	auto const chord{2.f * std::sin(turn * 0.5f)};
	for (std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade)
	{
		// with nothing of the view in a cascade, its map covers the whole slice
		if (data.counts[cascade] == 0)
			continue;
		auto const *bounds{&data.bounds[cascade * 8]};
		glm::vec3 low{unorder(bounds[0]), unorder(bounds[1]), unorder(bounds[2])};
		glm::vec3 high{unorder(bounds[4]), unorder(bounds[5]), unorder(bounds[6])};
		if ((false == std::isfinite(low.x + low.y + high.x + high.y)) || (low.x > high.x) || (low.y > high.y))
			continue;
		low = glm::min(low, low + lightshift);
		high = glm::max(high, high + lightshift);
		// and what came into the view as it turned, out to the reach of the cascade
		auto const reach{Splits[cascade + 1] + static_cast<float>(gl::MAX_CASCADES - cascade)};
		auto const margin{reach * chord + MARGIN + 0.01f * std::max(high.x - low.x, high.y - low.y)};
		low -= glm::vec3{margin};
		high += glm::vec3{margin};
		auto &area{Areas[cascade].emplace()};
		for (int corner = 0; corner < 8; ++corner)
			area[corner] = inverselight * glm::vec3{(corner & 1) ? high.x : low.x, (corner & 2) ? high.y : low.y, (corner & 4) ? high.z : low.z};
	}
	state.fitted = true;
	return true;
}

float shadow_fit::fit_slice(std::vector<glm::vec4> &Points, std::array<glm::vec3, 8> const *Area, glm::mat3 const &Lightrotation, float &Slicesize)
{
	// a cascade fitted to the depth of the view (gfx.shadow.sdsm) covers the area of the pixels it takes, within the slice of the
	// view. the areas are compared across the light, the depth along it stays the one of the slice
	auto quantizationstep{0.f};
	glm::vec3 slicemin{std::numeric_limits<float>::max()}, slicemax{std::numeric_limits<float>::lowest()};
	for (auto const &point : Points)
	{
		auto const lightpoint{Lightrotation * glm::vec3{point}};
		slicemin = glm::min(slicemin, lightpoint);
		slicemax = glm::max(slicemax, lightpoint);
	}
	Slicesize = std::max(slicemax.x - slicemin.x, slicemax.y - slicemin.y);
	if (Area != nullptr)
	{
		glm::vec3 areamin{std::numeric_limits<float>::max()}, areamax{std::numeric_limits<float>::lowest()};
		for (auto const &point : *Area)
		{
			auto const lightpoint{Lightrotation * point};
			areamin = glm::min(areamin, lightpoint);
			areamax = glm::max(areamax, lightpoint);
		}
		auto const fitmin{glm::vec2{glm::max(glm::vec2{slicemin}, glm::vec2{areamin})}};
		auto const fitmax{glm::vec2{glm::min(glm::vec2{slicemax}, glm::vec2{areamax})}};
		if ((fitmin.x < fitmax.x) && (fitmin.y < fitmax.y))
		{
			// the size changes in steps of a power of two of about 1/16 of it, so that the texels of the map keep their size until
			// the area grows or shrinks by a step
			auto const size{std::max(fitmax.x - fitmin.x, fitmax.y - fitmin.y)};
			quantizationstep = std::exp2(std::ceil(std::log2(std::max(size, 1.f) / 16.f)));
			Points.clear();
			auto const inverserotation{glm::transpose(Lightrotation)};
			for (int corner = 0; corner < 8; ++corner)
			{
				auto const lightpoint{glm::vec3{(corner & 1) ? fitmax.x : fitmin.x, (corner & 2) ? fitmax.y : fitmin.y, (corner & 4) ? slicemax.z : slicemin.z}};
				Points.emplace_back(inverserotation * lightpoint, 1.f);
			}
		}
	}
	return quantizationstep;
}

} // namespace gfx
