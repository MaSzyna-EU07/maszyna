/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorPreviews.hpp"

#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"
#include "stb/stb_image.h"

#include <algorithm>
#include <cctype>

namespace editor_previews
{

std::string const folder{"textures/previews/nodebank/"};

std::string file_name_part(std::string Text)
{
	for (auto &character : Text)
	{
		auto const c{static_cast<unsigned char>(character)};
		if ((false == std::isalnum(c)) && (c != '_') && (c != '-') && (c != '.') && (c != '+') && (c != '(') && (c != ')'))
		{
			character = '_';
		}
	}
	if ((Text.empty()) || (Text == ".") || (Text == ".."))
	{
		Text = "_";
	}
	return Text;
}

std::string preview_path(std::string Model, std::string const &Skin)
{
	replace_slashes(Model);
	erase_extension(Model);
	std::string path;
	std::size_t start{0};
	while (start <= Model.size())
	{
		auto end{Model.find('/', start)};
		if (end == std::string::npos)
		{
			end = Model.size();
		}
		if (end > start)
		{
			if (false == path.empty())
			{
				path += '/';
			}
			path += file_name_part(Model.substr(start, end - start));
		}
		start = end + 1;
	}
	if ((false == Skin.empty()) && (Skin != "none"))
	{
		auto skin{Skin};
		replace_slashes(skin);
		erase_extension(skin);
		std::replace(skin.begin(), skin.end(), '/', '_');
		path += '@' + file_name_part(skin);
	}
	return folder + path;
}

std::string entry_preview_path(std::string const &Entry)
{
	auto const nodestart{Entry.find("node")};
	if (nodestart == std::string::npos)
	{
		return {};
	}
	auto const entry{Entry.substr(nodestart)};
	// node <range max> <range min> <name> <type>
	cParser parser(entry, cParser::buffer_TEXT);
	parser.getTokens(5);
	std::string type;
	for (int index = 0; index < 5; ++index)
	{
		parser >> type;
	}
	if (type != "model")
	{
		return {};
	}
	// the model and skin are the first two tokens after the location and rotation, read as the generator reads them
	cParser names(entry, cParser::buffer_TEXT);
	names.getTokens(9, false);
	auto const model{names.getToken<std::string>()};
	auto const skin{names.getToken<std::string>(false)};
	if (model.empty())
	{
		return {};
	}
	return preview_path(model, skin);
}

std::string variant(std::string const &Path, std::set<std::string> const &Taken)
{
	if (false == Taken.contains(Path))
	{
		return Path;
	}
	auto number{2};
	while (Taken.contains(Path + "~" + std::to_string(number)))
	{
		++number;
	}
	return Path + "~" + std::to_string(number);
}

namespace
{

// images kept at most; at 128 pixels that's 32 MB
std::size_t const image_limit{512};
// uploaded in a frame at most
int const uploads_per_frame{8};

} // namespace

image_cache::image_cache()
{
	// at a bigger ui the images are drawn bigger too
	m_size = std::clamp(static_cast<int>(128.0f * std::max(1.0f, Global.ui_scale)), 128, 256);
	m_thread = std::thread(&image_cache::work, this);
}

image_cache::~image_cache()
{
	{
		std::lock_guard<std::mutex> lock(m_lock);
		m_quit = true;
	}
	m_wake.notify_all();
	if (m_thread.joinable())
	{
		m_thread.join();
	}
	// the textures aren't released: the cache goes when the simulator closes, and the gl context may be gone by then
}

std::uint64_t image_cache::image(std::string const &Path)
{
	auto &entry{m_entries[Path]};
	entry.used = m_frame;
	if (entry.texture != 0 || entry.missing || entry.pending)
	{
		return entry.texture;
	}
	entry.pending = true;
	{
		std::lock_guard<std::mutex> lock(m_lock);
		m_queue.push_front(Path);
	}
	m_wake.notify_one();
	return 0;
}

bool image_cache::missing(std::string const &Path) const
{
	auto const lookup{m_entries.find(Path)};
	return lookup != m_entries.end() && lookup->second.missing;
}

void image_cache::update()
{
	++m_frame;
	std::vector<decoded> done;
	{
		std::lock_guard<std::mutex> lock(m_lock);
		// the queue keeps only what was asked for lately; the rest is asked for again when it's scrolled back into sight
		while (m_queue.size() > image_limit / 2)
		{
			auto const lookup{m_entries.find(m_queue.back())};
			if (lookup != m_entries.end())
			{
				lookup->second.pending = false;
			}
			m_queue.pop_back();
		}
		auto const count{std::min<std::size_t>(m_done.size(), uploads_per_frame)};
		done.assign(std::make_move_iterator(m_done.begin()), std::make_move_iterator(m_done.begin() + count));
		m_done.erase(m_done.begin(), m_done.begin() + count);
	}
	auto *renderer{GfxRenderer ? GfxRenderer->GetImguiRenderer() : nullptr};
	for (auto &image : done)
	{
		if (image.generation != m_generation)
		{
			continue;
		}
		auto const lookup{m_entries.find(image.path)};
		if (lookup == m_entries.end())
		{
			continue;
		}
		auto &entry{lookup->second};
		entry.pending = false;
		if (image.rgba.empty())
		{
			entry.missing = true;
			continue;
		}
		entry.texture = (renderer != nullptr ? renderer->Create_Image(image.rgba.data(), image.width, image.height) : 0);
		// a renderer which can't show them gets no more requests
		entry.missing = (entry.texture == 0);
	}
	// over the limit, the images not asked for the longest are dropped
	std::size_t textures{0};
	for (auto const &entry : m_entries)
	{
		textures += (entry.second.texture != 0 ? 1 : 0);
	}
	if (textures <= image_limit)
	{
		return;
	}
	std::vector<std::pair<std::uint64_t, std::string const *>> loaded;
	loaded.reserve(textures);
	for (auto const &entry : m_entries)
	{
		if (entry.second.texture != 0)
		{
			loaded.emplace_back(entry.second.used, &entry.first);
		}
	}
	std::sort(loaded.begin(), loaded.end());
	std::vector<std::string> dropped;
	for (std::size_t index = 0; index < textures - image_limit; ++index)
	{
		dropped.push_back(*loaded[index].second);
	}
	for (auto const &path : dropped)
	{
		auto const lookup{m_entries.find(path)};
		release(lookup->second);
		m_entries.erase(lookup);
	}
}

void image_cache::clear()
{
	{
		std::lock_guard<std::mutex> lock(m_lock);
		m_queue.clear();
		m_done.clear();
		++m_generation;
	}
	for (auto &entry : m_entries)
	{
		release(entry.second);
	}
	m_entries.clear();
}

void image_cache::release(entry &Entry)
{
	if (Entry.texture != 0 && GfxRenderer && GfxRenderer->GetImguiRenderer() != nullptr)
	{
		GfxRenderer->GetImguiRenderer()->Release_Image(Entry.texture);
	}
	Entry.texture = 0;
}

void image_cache::work()
{
	// the texture loader turns its images upside down for all threads; these are read as they are
	stbi_set_flip_vertically_on_load_thread(0);
	while (true)
	{
		decoded image;
		{
			std::unique_lock<std::mutex> lock(m_lock);
			m_wake.wait(lock, [this]() { return m_quit || false == m_queue.empty(); });
			if (m_quit)
			{
				return;
			}
			image.path = m_queue.front();
			image.generation = m_generation;
			m_queue.pop_front();
		}
		int width{0}, height{0}, components{0};
		auto *pixels{stbi_load(image.path.c_str(), &width, &height, &components, 4)};
		if (pixels != nullptr && width > 0 && height > 0)
		{
			// scaled down by the average of the pixels which make each one, the longer side to the size of the cache
			auto const scale{std::max(1.0f, static_cast<float>(std::max(width, height)) / m_size)};
			image.width = std::max(1, static_cast<int>(width / scale));
			image.height = std::max(1, static_cast<int>(height / scale));
			image.rgba.resize(static_cast<std::size_t>(image.width) * image.height * 4);
			for (int y = 0; y < image.height; ++y)
			{
				auto const y0{static_cast<int>(y * scale)};
				auto const y1{std::max(y0 + 1, std::min(height, static_cast<int>((y + 1) * scale)))};
				for (int x = 0; x < image.width; ++x)
				{
					auto const x0{static_cast<int>(x * scale)};
					auto const x1{std::max(x0 + 1, std::min(width, static_cast<int>((x + 1) * scale)))};
					std::uint32_t sum[4]{0, 0, 0, 0};
					for (int sy = y0; sy < y1; ++sy)
					{
						auto const *row{pixels + (static_cast<std::size_t>(sy) * width + x0) * 4};
						for (int sx = x0; sx < x1; ++sx, row += 4)
						{
							sum[0] += row[0];
							sum[1] += row[1];
							sum[2] += row[2];
							sum[3] += row[3];
						}
					}
					auto const count{static_cast<std::uint32_t>((y1 - y0) * (x1 - x0))};
					auto *target{image.rgba.data() + (static_cast<std::size_t>(y) * image.width + x) * 4};
					for (int channel = 0; channel < 4; ++channel)
					{
						target[channel] = static_cast<std::uint8_t>(sum[channel] / count);
					}
				}
			}
		}
		if (pixels != nullptr)
		{
			stbi_image_free(pixels);
		}
		{
			std::lock_guard<std::mutex> lock(m_lock);
			m_done.emplace_back(std::move(image));
		}
	}
}

} // namespace editor_previews
