/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorClips.hpp"

#include "rendering/renderer.h"
#include "stb/stb_image.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <thread>

namespace editor_clips
{

clip::~clip()
{
	close();
}

void clip::open(std::string const &Path)
{
	if (Path == m_path && m_frames)
		return;
	close();
	// the loaders done with their clips are joined, those still decoding are kept, so none is waited for here
	std::erase_if(m_loaders, [](loader const &Loader) { return Loader.decoded->done.load(); });
	m_path = Path;
	auto frames{std::make_shared<clip::frames>()};
	m_frames = frames;
	if (std::error_code error; false == std::filesystem::exists(Path, error))
	{
		// no clip for the lesson: told at once, so no empty frame is shown while a thread finds the file missing
		frames->done = true;
		return;
	}
	std::jthread thread([frames, Path] {
		decode(*frames, Path);
		frames->done = true;
	});
	m_loaders.push_back(loader{frames, std::move(thread)});
}

void clip::decode(frames &Frames, std::string const &Path)
{
	stbi_set_flip_vertically_on_load_thread(0);
	std::ifstream file(Path, std::ios::binary);
	std::vector<unsigned char> const bytes{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
	if (bytes.empty())
		return;
	int *delays{nullptr};
	int width{0};
	int height{0};
	int count{0};
	int components{0};
	auto *rgba{stbi_load_gif_from_memory(bytes.data(), static_cast<int>(bytes.size()), &delays, &width, &height, &count, &components, 4)};
	if (rgba != nullptr && width > 0 && height > 0 && count > 0)
	{
		Frames.rgba = std::unique_ptr<std::uint8_t, void (*)(void *)>(rgba, stbi_image_free);
		rgba = nullptr;
		Frames.delays.resize(count);
		for (int index = 0; index < count; ++index)
			Frames.delays[index] = std::max(delays != nullptr ? delays[index] : 100, 20);
		Frames.width = width;
		Frames.height = height;
	}
	if (rgba != nullptr)
		stbi_image_free(rgba);
	if (delays != nullptr)
		stbi_image_free(delays);
}

void clip::close()
{
	if (m_texture != 0 && GfxRenderer && GfxRenderer->GetImguiRenderer() != nullptr)
		GfxRenderer->GetImguiRenderer()->Release_Image(m_texture);
	m_texture = 0;
	m_shown = -1;
	m_frames.reset();
	m_path.clear();
}

bool clip::loading() const
{
	return m_frames && false == m_frames->done;
}

bool clip::missing() const
{
	return m_frames == nullptr || (m_frames->done && m_frames->delays.empty());
}

int clip::width() const
{
	return m_frames && m_frames->done ? m_frames->width : 0;
}

int clip::height() const
{
	return m_frames && m_frames->done ? m_frames->height : 0;
}

std::uint64_t clip::texture(double const Seconds)
{
	if (loading() || missing())
		return 0;
	auto const &frames{*m_frames};
	long long total{0};
	for (auto const delay : frames.delays)
		total += delay;
	if (total <= 0)
		return 0;
	auto time{static_cast<long long>(std::floor(Seconds * 1000.0)) % total};
	int index{0};
	while (index + 1 < static_cast<int>(frames.delays.size()) && time >= frames.delays[index])
		time -= frames.delays[index++];
	if (index == m_shown && m_texture != 0)
		return m_texture;
	auto *renderer{GfxRenderer ? GfxRenderer->GetImguiRenderer() : nullptr};
	if (renderer == nullptr)
		return 0;
	if (m_texture != 0)
		renderer->Release_Image(m_texture);
	m_texture = renderer->Create_Image(frames.rgba.get() +static_cast<std::size_t>(frames.width) * frames.height * 4 * index, frames.width, frames.height);
	m_shown = index;
	return m_texture;
}

} // namespace editor_clips
