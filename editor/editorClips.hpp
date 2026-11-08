/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace editor_clips
{

class clip
{
  public:
	clip() = default;
	~clip();
	clip(clip const &) = delete;
	clip &operator=(clip const &) = delete;

	void open(std::string const &Path);
	void close();
	std::uint64_t texture(double const Seconds);
	bool loading() const;
	bool missing() const;
	int width() const;
	int height() const;
	std::string const &path() const
	{
		return m_path;
	}

  private:
	struct frames
	{
		int width{0};
		int height{0};
		// all the frames one after another, kept as stb_image decoded them and freed by it
		std::unique_ptr<std::uint8_t, void (*)(void *)> rgba{nullptr, nullptr};
		std::vector<int> delays;
		std::atomic<bool> done{false};
	};
	// a thread decoding the frames of a clip, kept until it's done with them and joined then
	struct loader
	{
		std::shared_ptr<frames const> decoded;
		std::jthread thread;
	};
	// the frames of the clip in the file, read and decoded on the thread of a loader
	static void decode(frames &Frames, std::string const &Path);
	std::string m_path;
	std::shared_ptr<frames> m_frames;
	std::uint64_t m_texture{0};
	int m_shown{-1};
	// the loaders of the clips opened, a clip closed while it's decoded keeping its own till it's done; the last member, so
	// joined before the others go
	std::vector<loader> m_loaders;
};

} // namespace editor_clips
