/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorIcons.hpp"

#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "stb/stb_image.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

namespace editor_icons
{

namespace
{

// editor_icons.png, written out by CMake
unsigned char const png[] = {
#include "editor_icons_png.inc"
};

int const cell{64}; // pixels of an icon in the png
int const columns{8};
int const rows{(static_cast<int>(icon::count) + columns - 1) / columns};

// the coverage of the icons, a byte a pixel, as they're in the png. empty if it couldn't be read
std::vector<std::uint8_t> const &coverage()
{
	static std::vector<std::uint8_t> const pixels = []() {
		std::vector<std::uint8_t> result;
		// read in a thread of its own: the texture loader turns the images upside down for the threads which don't say otherwise,
		// and what a thread says stays with it
		std::thread reader([&result]() {
			stbi_set_flip_vertically_on_load_thread(0);
			int width{0}, height{0}, components{0};
			auto *rgba{stbi_load_from_memory(png, static_cast<int>(sizeof(png)), &width, &height, &components, 4)};
			if (rgba == nullptr)
				return;
			if (width == columns * cell && height >= rows * cell)
			{
				result.resize(static_cast<std::size_t>(width) * height);
				for (std::size_t index = 0; index < result.size(); ++index)
					result[index] = rgba[index * 4 + 3];
			}
			stbi_image_free(rgba);
		});
		reader.join();
		return result;
	}();
	return pixels;
}

// the icons scaled down to the size they're drawn at, each with a pixel of empty space around it, so that the filtering
// doesn't pick up a neighbour
struct atlas
{
	std::uint64_t texture{0};
	int size{0}; // pixels of an icon
	bool failed{false};
};
atlas g_atlas;

int stride(int const Size)
{
	return Size + 2;
}

// part of the source pixel Pixel the target pixel Target of the size Scale covers
float overlap(int const Target, float const Scale, int const Pixel)
{
	auto const start{std::max(Target * Scale, static_cast<float>(Pixel))};
	auto const end{std::min((Target + 1) * Scale, static_cast<float>(Pixel + 1))};
	return std::max(0.0f, end - start);
}

std::vector<std::uint8_t> scaled(int const Size)
{
	auto const &source{coverage()};
	auto const width{columns * stride(Size)};
	auto const height{rows * stride(Size)};
	std::vector<std::uint8_t> rgba(static_cast<std::size_t>(width) * height * 4, 255);
	for (std::size_t index = 3; index < rgba.size(); index += 4)
		rgba[index] = 0;
	// each target pixel the average of the source pixels under it, the ones under its edges by the part they're under it
	auto const scale{static_cast<float>(cell) / Size};
	std::vector<float> across(static_cast<std::size_t>(Size) * cell);
	for (int index = 0; index < static_cast<int>(icon::count); ++index)
	{
		auto const sourcex{(index % columns) * cell};
		auto const sourcey{(index / columns) * cell};
		for (int y = 0; y < cell; ++y)
		{
			for (int x = 0; x < Size; ++x)
			{
				float sum{0.0f};
				for (int pixel = static_cast<int>(x * scale); pixel < std::min(cell, static_cast<int>(std::ceil((x + 1) * scale))); ++pixel)
					sum += overlap(x, scale, pixel) * source[static_cast<std::size_t>(sourcey + y) * columns * cell + sourcex + pixel];
				across[static_cast<std::size_t>(y) * Size + x] = sum / scale;
			}
		}
		auto const targetx{(index % columns) * stride(Size) + 1};
		auto const targety{(index / columns) * stride(Size) + 1};
		for (int y = 0; y < Size; ++y)
		{
			for (int x = 0; x < Size; ++x)
			{
				float sum{0.0f};
				for (int pixel = static_cast<int>(y * scale); pixel < std::min(cell, static_cast<int>(std::ceil((y + 1) * scale))); ++pixel)
					sum += overlap(y, scale, pixel) * across[static_cast<std::size_t>(pixel) * Size + x];
				rgba[(static_cast<std::size_t>(targety + y) * width + targetx + x) * 4 + 3] = static_cast<std::uint8_t>(std::clamp(sum / scale + 0.5f, 0.0f, 255.0f));
			}
		}
	}
	return rgba;
}

// the texture of the icons at the size, made again when the size changes
bool prepare(int const Size)
{
	if (g_atlas.failed)
		return false;
	if (g_atlas.texture != 0 && g_atlas.size == Size)
		return true;
	auto *renderer{GfxRenderer ? GfxRenderer->GetImguiRenderer() : nullptr};
	if (renderer == nullptr || coverage().empty())
	{
		g_atlas.failed = true;
		return false;
	}
	if (g_atlas.texture != 0)
		renderer->Release_Image(g_atlas.texture);
	auto const rgba{scaled(Size)};
	g_atlas.texture = renderer->Create_Image(rgba.data(), columns * stride(Size), rows * stride(Size));
	g_atlas.size = Size;
	// a renderer which can't show them gets no more requests
	g_atlas.failed = (g_atlas.texture == 0);
	return false == g_atlas.failed;
}

} // namespace

float size()
{
	return std::round(20.0f * std::max(1.0f, Global.ui_scale));
}

bool draw(ImDrawList *List, icon const Icon, ImVec2 const &Center, ImU32 const Color)
{
	// all at the one size, the texture is made for it
	auto const pixels{static_cast<int>(size())};
	if (List == nullptr || Icon >= icon::count || false == prepare(pixels))
		return false;
	auto const index{static_cast<int>(Icon)};
	auto const width{static_cast<float>(columns * stride(pixels))};
	auto const height{static_cast<float>(rows * stride(pixels))};
	ImVec2 const uv0{((index % columns) * stride(pixels) + 1) / width, ((index / columns) * stride(pixels) + 1) / height};
	ImVec2 const uv1{uv0.x + pixels / width, uv0.y + pixels / height};
	// on whole pixels, the icon was scaled for them
	ImVec2 const min{std::round(Center.x - pixels * 0.5f), std::round(Center.y - pixels * 0.5f)};
	List->AddImage(static_cast<ImTextureID>(g_atlas.texture), min, ImVec2(min.x + pixels, min.y + pixels), uv0, uv1, Color);
	return true;
}

} // namespace editor_icons
