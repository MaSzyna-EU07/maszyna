/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/temporalaa.h"

#include "vehicle/DynObj.h"

namespace
{

// the camera moved farther than this from one frame to the next (another camera, another vehicle, a jump): the history is dropped
constexpr double CAMERA_JUMP{50.0};

// the halton sequence: the index written in the base, mirrored behind the point
float halton(std::uint32_t Index, std::uint32_t const Base)
{
	auto fraction{1.f};
	auto result{0.f};
	while (Index > 0)
	{
		fraction /= static_cast<float>(Base);
		result += fraction * static_cast<float>(Index % Base);
		Index /= Base;
	}
	return result;
}

} // namespace

glm::mat4 gfx::temporal_aa::begin_frame(glm::mat4 const &Projection, glm::dmat3 const &View, glm::dvec3 const &Camera, TDynamicObject const *Cab, glm::ivec2 const &Size)
{
	// the offsets go round m_phases frames: enough to cover a pixel evenly, few enough for the history to hold all of them
	m_phase = (m_phase + 1) % m_phases;
	// the first element of the sequence is 0 in every base; from the second one on they are spread over the pixel
	m_jitter = glm::vec2{halton(m_phase + 1, 2), halton(m_phase + 1, 3)} - 0.5f;
	glm::dvec2 const size{static_cast<double>(std::max(Size.x, 1)), static_cast<double>(std::max(Size.y, 1))};
	// the clip space is two units across the image
	auto const offset{glm::translate(glm::dmat4{1.0}, glm::dvec3{2.0 * glm::dvec2{m_jitter} / size, 0.0})};

	frame current;
	current.projection = glm::dmat4{Projection};
	current.view = View;
	current.camera = Camera;
	current.vehicle = Cab;
	if (Cab != nullptr)
	{
		// as Render_cab() places it
		current.cab = glm::translate(glm::dmat4{1.0}, Cab->GetPosition() - Camera) * Cab->mMatrix;
	}
	current.size = Size;
	current.valid = true;

	auto const projection{offset * current.projection};
	m_historyvalid = m_previous.valid && (m_previous.size == Size) && (m_previous.vehicle == Cab) && (glm::length(Camera - m_previous.camera) < CAMERA_JUMP);
	if (m_historyvalid)
	{
		auto const inverseprojection{glm::inverse(projection)};
		// the view of this frame to the world around the camera, the world around the camera of the last frame, its view
		auto const toworld{glm::dmat4{glm::inverse(current.view)}};
		auto const world{glm::dmat4{m_previous.view} * glm::translate(glm::dmat4{1.0}, Camera - m_previous.camera) * toworld};
		// the cab goes with its vehicle: the view of this frame to the cab, to the world around the camera of the last frame
		auto const cab{glm::dmat4{m_previous.view} * m_previous.cab * glm::inverse(current.cab) * toworld};
		// the shaders project the future with the projection of this frame (the velocity is the place in the future less the place
		// now, both with the offset of this frame): the projection of the last frame with the offset of this one is put in front of
		// it in the future. with the projection the same in both frames this is the identity, and the velocity holds just the motion
		auto const previous{offset * m_previous.projection};
		auto const projectionfix{inverseprojection * previous};
		m_worldfuture = glm::mat4{projectionfix * world};
		m_cabfuture = glm::mat4{projectionfix * cab};
		m_reprojection = glm::mat4{previous * world * inverseprojection};
		m_reprojectionnooffset = glm::mat4{m_previous.projection * world * glm::inverse(current.projection)};
	}
	else
	{
		m_worldfuture = glm::mat4{1.f};
		m_cabfuture = glm::mat4{1.f};
		m_reprojection = glm::mat4{1.f};
		m_reprojectionnooffset = glm::mat4{1.f};
	}
	m_target = 1 - m_target;
	m_previous = current;
	return glm::mat4{projection};
}
