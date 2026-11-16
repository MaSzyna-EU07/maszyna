/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/rmluirender.h"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <RmlUi/Core/Vertex.h>

#include "application/rmluipixels.h"
#include "rendering/renderer.h"
#include "stb/stb_image.h"
#include "utilities/Logs.h"

namespace rmlui
{
namespace
{

imgui_renderer *images()
{
	return GfxRenderer != nullptr ? GfxRenderer->GetImguiRenderer() : nullptr;
}

// the indices of imgui are 16 bit, one draw can't have more vertices. the draws together can, the renderers start the vertices of a
// draw list over at an offset (ImGuiBackendFlags_RendererHasVtxOffset)
bool fits_draw_list(std::size_t const Vertices)
{
	return Vertices < (std::size_t{1} << 16U);
}

} // namespace

void imgui_render_interface::begin_frame(ImDrawList *List, ImVec2 const Origin)
{
	m_list = List;
	m_origin = Origin;
	m_vertices = 0;
	m_scissor = false;
	m_transform.reset();
}

void imgui_render_interface::end_frame()
{
	m_list = nullptr;
}

void imgui_render_interface::release_all()
{
	if (auto *renderer{images()}; renderer != nullptr)
	{
		for (auto const &texture : m_textures)
			renderer->Release_Image(texture.second);
	}
	m_textures.clear();
	m_geometries.clear();
}

Rml::CompiledGeometryHandle imgui_render_interface::CompileGeometry(Rml::Span<Rml::Vertex const> const Vertices, Rml::Span<int const> const Indices)
{
	auto const handle{m_nextgeometry++};
	m_geometries.try_emplace(handle, geometry{Vertices, Indices});
	return handle;
}

ImVec2 imgui_render_interface::place(Rml::Vector2f const Position, Rml::Vector2f const Translation) const
{
	auto point{Position + Translation};
	if (m_transform)
	{
		auto const projected{*m_transform * Rml::Vector4f{point.x, point.y, 0.f, 1.f}};
		if (std::abs(projected.w) > std::numeric_limits<float>::epsilon())
			point = Rml::Vector2f{projected.x / projected.w, projected.y / projected.w};
	}
	return ImVec2{m_origin.x + point.x, m_origin.y + point.y};
}

void imgui_render_interface::RenderGeometry(Rml::CompiledGeometryHandle const Geometry, Rml::Vector2f const Translation, Rml::TextureHandle const Texture)
{
	auto const found{m_geometries.find(Geometry)};
	if (m_list == nullptr || found == m_geometries.end())
		return;
	auto const &[vertices, indices]{found->second};
	if (vertices.size() == 0 || indices.size() == 0 || false == fits_draw_list(vertices.size()))
		return;
	auto const image{m_textures.find(Texture)};
	auto const textured{image != m_textures.end()};

	if (m_scissor)
		m_list->PushClipRect(ImVec2{m_origin.x + static_cast<float>(m_scissorregion.Left()), m_origin.y + static_cast<float>(m_scissorregion.Top())},
		                     ImVec2{m_origin.x + static_cast<float>(m_scissorregion.Right()), m_origin.y + static_cast<float>(m_scissorregion.Bottom())}, true);
	if (textured)
		m_list->PushTextureID(static_cast<ImTextureID>(image->second));

	m_list->PrimReserve(static_cast<int>(indices.size()), static_cast<int>(vertices.size()));
	m_vertices += static_cast<int>(vertices.size());
	// read after the reservation, which may start the vertices of a new draw at 0
	auto const base{m_list->_VtxCurrentIdx};
	auto const white{ImGui::GetFontTexUvWhitePixel()};
	for (auto const &vertex : vertices)
	{
		auto const &colour{vertex.colour};
		m_list->PrimWriteVtx(place(vertex.position, Translation), textured ? ImVec2{vertex.tex_coord.x, vertex.tex_coord.y} : white,
		                     straight_colour(colour.red, colour.green, colour.blue, colour.alpha));
	}
	for (auto const index : indices)
		m_list->PrimWriteIdx(static_cast<ImDrawIdx>(base + static_cast<unsigned>(index)));

	if (textured)
		m_list->PopTextureID();
	if (m_scissor)
		m_list->PopClipRect();
}

void imgui_render_interface::ReleaseGeometry(Rml::CompiledGeometryHandle const Geometry)
{
	m_geometries.erase(Geometry);
}

Rml::TextureHandle imgui_render_interface::make_texture(std::uint8_t const *Rgba, Rml::Vector2i const Dimensions)
{
	auto *renderer{images()};
	if (renderer == nullptr)
		return 0;
	auto const image{renderer->Create_Image(Rgba, Dimensions.x, Dimensions.y)};
	if (image == 0)
		return 0;
	auto const handle{m_nexttexture++};
	m_textures.try_emplace(handle, image);
	return handle;
}

Rml::TextureHandle imgui_render_interface::LoadTexture(Rml::Vector2i &Dimensions, Rml::String const &Source)
{
	// the loader of the textures of the scene flips the images it reads for all the threads, the ui wants the top row first
	stbi_set_flip_vertically_on_load_thread(0);
	int width{0};
	int height{0};
	int components{0};
	std::unique_ptr<stbi_uc, void (*)(void *)> const pixels{stbi_load(Source.c_str(), &width, &height, &components, 4), &stbi_image_free};
	if (pixels == nullptr)
	{
		ErrorLog("rmlui: can't read the image " + Source);
		return 0;
	}
	Dimensions = Rml::Vector2i{width, height};
	return make_texture(pixels.get(), Dimensions);
}

Rml::TextureHandle imgui_render_interface::GenerateTexture(Rml::Span<Rml::byte const> const Source, Rml::Vector2i const Dimensions)
{
	std::vector<std::uint8_t> rgba(Source.begin(), Source.end());
	unpremultiply(rgba);
	return make_texture(rgba.data(), Dimensions);
}

void imgui_render_interface::ReleaseTexture(Rml::TextureHandle const Texture)
{
	auto const found{m_textures.find(Texture)};
	if (found == m_textures.end())
		return;
	if (auto *renderer{images()}; renderer != nullptr)
		renderer->Release_Image(found->second);
	m_textures.erase(found);
}

void imgui_render_interface::EnableScissorRegion(bool const Enable)
{
	m_scissor = Enable;
}

void imgui_render_interface::SetScissorRegion(Rml::Rectanglei const Region)
{
	m_scissorregion = Region;
}

void imgui_render_interface::SetTransform(Rml::Matrix4f const *Transform)
{
	if (Transform != nullptr)
		m_transform = *Transform;
	else
		m_transform.reset();
}

} // namespace rmlui
