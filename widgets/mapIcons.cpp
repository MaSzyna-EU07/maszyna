#include "stdafx.h"
#include "widgets/mapIcons.h"

#include "rendering/renderer.h"
#include "stb/stb_image.h"
#include "utilities/Globals.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace map_icons
{

namespace
{

unsigned char constexpr png[] = {
#include "map_icons_png.inc"
};

int constexpr cell = 64;
int constexpr columns = 8;
int constexpr rows = 1;

struct atlas
{
	std::uint64_t texture { 0 };
	bool failed { false };
};

atlas g_atlas;

bool prepare()
{
	if (g_atlas.texture != 0)
		return true;
	if (g_atlas.failed)
		return false;

	auto *renderer = GfxRenderer ? GfxRenderer->GetImguiRenderer() : nullptr;
	if (renderer == nullptr)
		return false;

	stbi_set_flip_vertically_on_load_thread(0);
	int width = 0;
	int height = 0;
	int components = 0;
	auto *rgba = stbi_load_from_memory(png, static_cast<int>(sizeof(png)), &width, &height, &components, 4);
	if (rgba == nullptr)
	{
		g_atlas.failed = true;
		return false;
	}

	if (width != columns * cell || height != rows * cell)
	{
		stbi_image_free(rgba);
		g_atlas.failed = true;
		return false;
	}

	g_atlas.texture = renderer->Create_Image(rgba, width, height);
	stbi_image_free(rgba);
	g_atlas.failed = g_atlas.texture == 0;
	return !g_atlas.failed;
}

} // namespace

bool draw(ImDrawList *List, icon const Icon, ImVec2 const &Center, ImU32 const Color)
{
	if (List == nullptr || Icon >= icon::count || !prepare())
		return false;

	int const index = static_cast<int>(Icon);
	float constexpr padding = 1.0f;
	float constexpr atlas_width = static_cast<float>(columns * cell);
	float constexpr atlas_height = static_cast<float>(rows * cell);
	ImVec2 const uv0(
		(index * cell + padding) / atlas_width,
		padding / atlas_height);
	ImVec2 const uv1(
		((index + 1) * cell - padding) / atlas_width,
		(cell - padding) / atlas_height);
	float const size = std::round(20.0f * std::max(1.0f, Global.ui_scale));
	ImVec2 const minimum(std::round(Center.x - size * 0.5f), std::round(Center.y - size * 0.5f));
	List->AddImage(
		static_cast<ImTextureID>(g_atlas.texture),
		minimum,
		ImVec2(minimum.x + size, minimum.y + size),
		uv0,
		uv1,
		Color);
	return true;
}

} // namespace map_icons
