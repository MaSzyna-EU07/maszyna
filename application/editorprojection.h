/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editormode.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"

#include <limits>

namespace overlay_color
{
ImU32 constexpr marked{IM_COL32(255, 210, 60, 255)};
ImU32 constexpr grip{IM_COL32(60, 230, 90, 255)};
ImU32 constexpr selected{IM_COL32(40, 220, 255, 255)};
ImU32 constexpr invalid{IM_COL32(255, 60, 60, 255)};
ImU32 constexpr highlight{IM_COL32(255, 255, 255, 255)};
} // namespace overlay_color

// world to screen projection of the current camera, for overlays drawn with ImGui
class screen_projection
{
  public:
	screen_projection()
	{
		ImGuiIO const &io = ImGui::GetIO();
		m_size = io.DisplaySize;
		m_camera = GfxRenderer->Camera_Position();
		float const aspect = m_size.y > 0.0f ? m_size.x / m_size.y : 1.0f;
		m_viewprojection = editor_mode::projection_matrix(aspect) * GfxRenderer->Camera_View_Matrix();
	}

	glm::vec4 clip(glm::dvec3 const &Point) const
	{
		return m_viewprojection * glm::vec4(glm::vec3(Point - m_camera), 1.0f);
	}
	ImVec2 screen(glm::vec4 const &Clip) const
	{
		return ImVec2((Clip.x / Clip.w * 0.5f + 0.5f) * m_size.x, (0.5f - Clip.y / Clip.w * 0.5f) * m_size.y);
	}
	bool project(glm::dvec3 const &Point, ImVec2 &Screen) const
	{
		auto const c = clip(Point);
		if (c.w < kNear)
			return false;
		Screen = screen(c);
		return true;
	}
	void line(ImDrawList *Drawlist, glm::dvec3 const &A, glm::dvec3 const &B, ImU32 const Color, float const Thickness) const
	{
		glm::vec4 a = clip(A);
		glm::vec4 b = clip(B);
		if (a.w < kNear && b.w < kNear)
			return;
		if (a.w < kNear)
			a = glm::mix(a, b, (kNear - a.w) / (b.w - a.w));
		else if (b.w < kNear)
			b = glm::mix(b, a, (kNear - b.w) / (a.w - b.w));
		Drawlist->AddLine(screen(a), screen(b), Color, Thickness);
	}
	// squared distance on the screen from the mouse cursor to the point, the largest float for points behind the camera
	float mouse_distance2(glm::dvec3 const &Point) const
	{
		ImVec2 position;
		if (false == project(Point, position))
			return std::numeric_limits<float>::max();
		auto const &mouse{ImGui::GetIO().MousePos};
		return (position.x - mouse.x) * (position.x - mouse.x) + (position.y - mouse.y) * (position.y - mouse.y);
	}

  private:
	static constexpr float kNear{0.1f};
	ImVec2 m_size;
	glm::dvec3 m_camera;
	glm::mat4 m_viewprojection;
};

// starts a gizmo frame over the whole display, the view and the projection are those of the current camera
struct gizmo_frame
{
	glm::mat4 view;
	glm::mat4 projection;
	glm::dvec3 camera;

	gizmo_frame()
	{
		ImGuizmo::BeginFrame();
		ImGuizmo::SetOrthographic(Global.EditorOrtho);
		ImGuiIO const &io = ImGui::GetIO();
		ImGuizmo::SetRect(0.0f, 0.0f, io.DisplaySize.x, io.DisplaySize.y);
		view = GfxRenderer->Camera_View_Matrix();
		camera = GfxRenderer->Camera_Position();
		projection = editor_mode::projection_matrix(io.DisplaySize.y > 0.0f ? io.DisplaySize.x / io.DisplaySize.y : 1.0f);
	}
	// moves the gizmo, placed at the point given in world space, with snapping to the step when ctrl is held
	bool manipulate(ImGuizmo::OPERATION const Operation, glm::mat4 &Gizmo, float const Snap, glm::mat4 *Delta = nullptr) const
	{
		glm::vec3 snapvalue(Snap);
		float const *snap = Global.ctrlState && Snap > 0.0f ? &snapvalue.x : nullptr;
		return ImGuizmo::Manipulate(&view[0][0], &projection[0][0], Operation, ImGuizmo::WORLD, &Gizmo[0][0], Delta != nullptr ? &(*Delta)[0][0] : nullptr, snap);
	}
};
