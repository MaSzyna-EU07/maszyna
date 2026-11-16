/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>

#include <RmlUi/Core/RenderInterface.h>

#include "imgui/imgui.h"

namespace rmlui
{

// RmlUi drawn into a draw list of imgui: its triangles become those of imgui, its textures the images of the user interface of
// the renderer (imgui_renderer::Create_Image). so any renderer which draws imgui draws the documents too, in the same pass and
// with the colours mixed the same way. what would need a renderer of its own isn't done: clip masks (the content of a rounded
// box with overflow hidden isn't rounded off), layers, filters and shaders (gradients, box shadows)
class imgui_render_interface final : public Rml::RenderInterface
{
  public:
	// the draws of a frame go to List, the top left corner of the documents at Origin in the coordinates of imgui
	void begin_frame(ImDrawList *List, ImVec2 Origin);
	void end_frame();
	// the images of the textures still held are released, while the renderer is still there
	void release_all();

	// Rml::RenderInterface
	Rml::CompiledGeometryHandle CompileGeometry(Rml::Span<Rml::Vertex const> Vertices, Rml::Span<int const> Indices) override;
	void RenderGeometry(Rml::CompiledGeometryHandle Geometry, Rml::Vector2f Translation, Rml::TextureHandle Texture) override;
	void ReleaseGeometry(Rml::CompiledGeometryHandle Geometry) override;
	Rml::TextureHandle LoadTexture(Rml::Vector2i &Dimensions, Rml::String const &Source) override;
	Rml::TextureHandle GenerateTexture(Rml::Span<Rml::byte const> Source, Rml::Vector2i Dimensions) override;
	void ReleaseTexture(Rml::TextureHandle Texture) override;
	void EnableScissorRegion(bool Enable) override;
	void SetScissorRegion(Rml::Rectanglei Region) override;
	void SetTransform(Rml::Matrix4f const *Transform) override;

  private:
	// the vertices and the indices stay where RmlUi keeps them until it releases the geometry
	struct geometry
	{
		Rml::Span<Rml::Vertex const> vertices;
		Rml::Span<int const> indices;
	};

	// an image of the renderer made of straight rgba pixels, rows from the top
	Rml::TextureHandle make_texture(std::uint8_t const *Rgba, Rml::Vector2i Dimensions);
	// where a vertex of RmlUi lands in the draw list
	ImVec2 place(Rml::Vector2f Position, Rml::Vector2f Translation) const;

	ImDrawList *m_list{nullptr};
	ImVec2 m_origin{0.f, 0.f};
	bool m_scissor{false};
	Rml::Rectanglei m_scissorregion;
	std::optional<Rml::Matrix4f> m_transform;
	std::unordered_map<Rml::CompiledGeometryHandle, geometry> m_geometries;
	Rml::CompiledGeometryHandle m_nextgeometry{1};
	// the textures of RmlUi and the images of the renderer they are
	std::unordered_map<Rml::TextureHandle, std::uint64_t> m_textures;
	Rml::TextureHandle m_nexttexture{1};
};

} // namespace rmlui
