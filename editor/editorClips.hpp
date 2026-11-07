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
		std::vector<std::uint8_t> rgba;
		std::vector<int> delays;
		std::atomic<bool> done{false};
	};
	std::string m_path;
	std::shared_ptr<frames> m_frames;
	std::uint64_t m_texture{0};
	int m_shown{-1};
};

} // namespace editor_clips
