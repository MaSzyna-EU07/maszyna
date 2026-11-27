/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/screenreflections.h"

#include "vehicle/DynObj.h"
#include "gl/ubo.h"
#include "utilities/Globals.h"

namespace
{

// the camera moved farther than this since the frame of the history (another camera, another vehicle, a jump): the history is
// no good for the reflections
constexpr double CAMERA_JUMP{50.0};
// the box around the vehicle of the cab is this much larger than the vehicle on each side, metres
constexpr double CAB_BOX_MARGIN{0.5};

} // namespace

void gfx::screen_reflections::begin_frame(glm::mat4 const &Projection, glm::dmat3 const &View, glm::dvec3 const &Camera, TDynamicObject const *Cab, glm::ivec2 const &Size, int const Levels)
{
	++m_frame;
	m_levels = std::max(Levels, 1);
	m_current.projection = glm::dmat4{Projection};
	m_current.view = View;
	m_current.camera = Camera;
	m_current.vehicle = Cab;
	m_current.cab = glm::dmat4{1.0};
	if (Cab != nullptr)
	{
		// as Render_cab() places it
		m_current.cab = glm::translate(glm::dmat4{1.0}, Cab->GetPosition() - Camera) * Cab->mMatrix;
	}
	m_current.size = Size;
	m_current.valid = true;

	m_historyvalid = m_history.valid && (m_history.size == Size) && (glm::length(Camera - m_history.camera) < CAMERA_JUMP);
	m_hascab = false;
	if (false == m_historyvalid)
	{
		m_reprojection = glm::mat4{1.f};
		m_reprojectioncab = glm::mat4{1.f};
		m_cabbox = glm::mat4{1.f};
		return;
	}
	// the view of this frame to the world around the camera, the world around the camera of the history, its view and projection
	auto const toworld{glm::dmat4{glm::inverse(View)}};
	m_reprojection = glm::mat4{m_history.projection * glm::dmat4{m_history.view} * glm::translate(glm::dmat4{1.0}, Camera - m_history.camera) * toworld};
	// the cab goes with its vehicle: the view of this frame to the cab, to the world around the camera of the history
	if ((Cab != nullptr) && (Cab == m_history.vehicle))
	{
		m_reprojectioncab = glm::mat4{m_history.projection * glm::dmat4{m_history.view} * m_history.cab * glm::inverse(m_current.cab) * toworld};
		// the vehicle stands on the origin of its model, along z, up to its height
		auto const &dimensions{Cab->MoverParameters->Dim};
		glm::dvec3 const lower{-0.5 * dimensions.W - CAB_BOX_MARGIN, -CAB_BOX_MARGIN, -0.5 * dimensions.L - CAB_BOX_MARGIN};
		glm::dvec3 const upper{0.5 * dimensions.W + CAB_BOX_MARGIN, dimensions.H + CAB_BOX_MARGIN, 0.5 * dimensions.L + CAB_BOX_MARGIN};
		auto const centre{0.5 * (lower + upper)};
		auto const halfsize{glm::max(0.5 * (upper - lower), glm::dvec3{0.1})};
		auto const tobox{glm::scale(glm::dmat4{1.0}, 1.0 / halfsize) * glm::translate(glm::dmat4{1.0}, -centre)};
		m_cabbox = glm::mat4{tobox * glm::inverse(m_current.cab) * toworld};
		m_hascab = true;
	}
	else
	{
		m_reprojectioncab = m_reprojection;
		m_cabbox = glm::mat4{1.f};
	}
}

void gfx::screen_reflections::end_frame()
{
	if (m_current.valid)
	{
		m_history = m_current;
	}
}

void gfx::screen_reflections::setup(gl::scene_ubs &Scene, float const Steps, bool const Raytracing, float const Raytracingrange, bool const Temporal) const
{
	clear(Scene);
	auto const screen{m_historyvalid && (Steps > 0.f)};
	if ((false == screen) && (false == Raytracing))
	{
		return;
	}
	Scene.ssr_reprojection = m_reprojection;
	Scene.ssr_reprojection_cab = m_reprojectioncab;
	Scene.ssr_cab_box = m_cabbox;
	Scene.ssr_params = glm::vec4{screen ? Steps : 0.f, Global.gfx_ssr_range, Global.gfx_ssr_roughness, Temporal ? static_cast<float>(m_frame % 16) : 0.f};
	Scene.ssr_params2 = glm::vec4{static_cast<float>(m_levels - 1), m_hascab ? 1.f : 0.f, Raytracing ? 1.f : 0.f, Raytracingrange};
}

void gfx::screen_reflections::clear(gl::scene_ubs &Scene)
{
	Scene.ssr_reprojection = glm::mat4{1.f};
	Scene.ssr_reprojection_cab = glm::mat4{1.f};
	Scene.ssr_cab_box = glm::mat4{1.f};
	Scene.ssr_params = glm::vec4{0.f};
	Scene.ssr_params2 = glm::vec4{0.f};
}

float gfx::screen_reflections::steps(int const Level)
{
	switch (Level)
	{
	case 1: return 16.f;
	case 2: return 32.f;
	case 3: return 64.f;
	default: return 0.f;
	}
}

void gfx::screen_reflections::draw_debug_ui(char const *Raytracing) const
{
	char const *levels[]{"no", "low (16 steps)", "medium (32 steps)", "high (64 steps)"};
	ImGui::Combo("screen space reflections (gfx.ssr)", &Global.gfx_ssr, levels, IM_ARRAYSIZE(levels));
	ImGui::SliderFloat("longest ray, m (gfx.ssr.range)", &Global.gfx_ssr_range, 10.f, 2000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	ImGui::SliderFloat("roughest surface reflecting the scene (gfx.ssr.roughness)", &Global.gfx_ssr_roughness, 0.f, 1.f, "%.2f");
	if (Global.gfx_ssr > 0)
	{
		ImGui::Text("image of the last frame: %s", m_historyvalid ? "in use" : "none (first frame, camera jump or new size)");
	}
	if (Raytracing != nullptr)
	{
		ImGui::TextUnformatted(Raytracing);
	}
}
