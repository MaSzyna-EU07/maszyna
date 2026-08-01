/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/orthophoto.h"

#include "stb/stb_image.h"
#include "utilities/Logs.h"

#ifdef _WIN32
#include <windows.h>
#include <winhttp.h>
#pragma comment(lib, "winhttp.lib")
#endif

namespace editor
{

namespace
{

int const kWorkers = 4;
// the imagery service sits behind a filter that turns down browser user agents and, even for a
// plain one, fails a request now and then; a handful of immediate retries gets past both
int const kAttempts = 4;
char const *kUserAgent = "curl/8.5.0";

#ifdef _WIN32

// plain HTTPS GET. deliberately small: one request, one answer, no keep-alive - tiles are large
// enough that the connection setup does not dominate, and this way a stuck transfer cannot poison
// anything but itself
bool http_get(std::string const &Url, std::vector<std::uint8_t> &Bytes, std::string &Contenttype)
{
	std::wstring const url(Url.begin(), Url.end());

	URL_COMPONENTS parts{};
	parts.dwStructSize = sizeof(parts);
	parts.dwHostNameLength = 1;
	parts.dwUrlPathLength = 1;
	parts.dwExtraInfoLength = 1;
	if (false == WinHttpCrackUrl(url.c_str(), static_cast<DWORD>(url.size()), 0, &parts))
	{
		return false;
	}

	std::wstring const host(parts.lpszHostName, parts.dwHostNameLength);
	std::wstring const resource = std::wstring(parts.lpszUrlPath, parts.dwUrlPathLength) + std::wstring(parts.lpszExtraInfo, parts.dwExtraInfoLength);

	auto const wideagent = std::wstring(kUserAgent, kUserAgent + std::strlen(kUserAgent));
	HINTERNET session = WinHttpOpen(wideagent.c_str(), WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	if (session == nullptr)
	{
		return false;
	}

	auto const cleanup = [](HINTERNET handle) {
		if (handle != nullptr)
		{
			WinHttpCloseHandle(handle);
		}
	};

	HINTERNET connection = WinHttpConnect(session, host.c_str(), parts.nPort, 0);
	if (connection == nullptr)
	{
		cleanup(session);
		return false;
	}

	DWORD const flags = (parts.nScheme == INTERNET_SCHEME_HTTPS ? WINHTTP_FLAG_SECURE : 0);
	HINTERNET request = WinHttpOpenRequest(connection, L"GET", resource.c_str(), nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, flags);
	if (request == nullptr)
	{
		cleanup(connection);
		cleanup(session);
		return false;
	}

	auto ok = false;
	if (WinHttpSendRequest(request, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) && WinHttpReceiveResponse(request, nullptr))
	{
		DWORD status = 0;
		DWORD statussize = sizeof(status);
		WinHttpQueryHeaders(request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &status, &statussize, WINHTTP_NO_HEADER_INDEX);

		if (status == 200)
		{
			wchar_t typebuffer[128]{};
			DWORD typesize = sizeof(typebuffer);
			if (WinHttpQueryHeaders(request, WINHTTP_QUERY_CONTENT_TYPE, WINHTTP_HEADER_NAME_BY_INDEX, typebuffer, &typesize, WINHTTP_NO_HEADER_INDEX))
			{
				std::wstring const wide(typebuffer);
				Contenttype.assign(wide.begin(), wide.end());
			}

			Bytes.clear();
			DWORD available = 0;
			while (WinHttpQueryDataAvailable(request, &available) && available > 0)
			{
				auto const offset = Bytes.size();
				Bytes.resize(offset + available);
				DWORD read = 0;
				if (false == WinHttpReadData(request, Bytes.data() + offset, available, &read))
				{
					Bytes.clear();
					break;
				}
				Bytes.resize(offset + read);
			}
			ok = (false == Bytes.empty());
		}
	}

	cleanup(request);
	cleanup(connection);
	cleanup(session);

	return ok;
}

#else

bool http_get(std::string const &, std::vector<std::uint8_t> &, std::string &)
{
	return false; // no native transport on this platform yet
}

#endif

} // namespace

orthophoto_source::orthophoto_source(maj0sted::editor::WmsConfig Config, int const Gridzoom, std::string Cachesubdir)
    : m_service([&] {
	      if (Gridzoom >= maj0sted::editor::TileGrid::kCellZoomMin)
	      {
		      Config.tile_pixels = 512; // coarse cells do not need orto's 1024 px
	      }
	      return std::move(Config);
      }()),
      m_gridzoom(Gridzoom), m_cachesubdir(std::move(Cachesubdir))
{
	// the library asks for a URL and expects the answer whenever it arrives; the transfer is queued
	// here and handed back on the thread that drains it, so the service stays single threaded
	m_service.set_fetcher([this](std::string const &Url, maj0sted::editor::TileService::DoneFn Done) {
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_queued.push_back({Url, std::move(Done), false, {}, {}});
		}
		++m_inflight;
		m_wakeup.notify_one();
	});

	m_workers.reserve(kWorkers);
	for (auto i = 0; i < kWorkers; ++i)
	{
		m_workers.emplace_back([this]() { worker(); });
	}
}

orthophoto_source::~orthophoto_source()
{
	m_quitting = true;
	m_wakeup.notify_all();
	for (auto &worker : m_workers)
	{
		if (worker.joinable())
		{
			worker.join();
		}
	}

	for (auto const &texture : m_textures)
	{
		if (texture.second != 0)
		{
			auto const id = texture.second;
			glDeleteTextures(1, &id);
		}
	}
}

void orthophoto_source::worker()
{
	while (false == m_quitting)
	{
		transfer job;
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			m_wakeup.wait(lock, [this]() { return m_quitting || (false == m_queued.empty()); });
			if (m_quitting)
			{
				return;
			}
			job = std::move(m_queued.front());
			m_queued.pop_front();
		}

		for (auto attempt = 0; attempt < kAttempts && false == m_quitting; ++attempt)
		{
			if (http_get(job.url, job.bytes, job.content_type))
			{
				job.ok = true;
				break;
			}
		}

		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_finished.push_back(std::move(job));
		}
	}
}

std::vector<orthophoto_source::ready_tile> orthophoto_source::collect(maj0sted::editor::TileBBox const &View, int const Maxtiles)
{
	// hand finished transfers to the library here, on the caller's thread
	std::deque<transfer> arrived;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		arrived.swap(m_finished);
	}
	for (auto &job : arrived)
	{
		if (job.done)
		{
			job.done(job.ok, std::move(job.bytes), std::move(job.content_type));
		}
		--m_inflight;
	}

	// fetch only what fits the budget for the current view. the window is recentred when the
	// view is larger than the cap, and zoom-to-cursor moves the camera, so the set of keys asked
	// for changes from frame to frame - that must not decide what is drawn
	for (auto const &key : maj0sted::editor::TileGrid::tiles_for_view(View, m_gridzoom, Maxtiles))
	{
		// a tile kept from an earlier session seeds the cache, so no request goes out for it at all
		if (false == m_service.has(key))
		{
			load_from_disk(key);
		}

		auto const status = m_service.request(key);
		if (status == maj0sted::editor::TileStatus::Failed)
		{
			// counted once per tile, not once per frame it stays unavailable
			m_failedkeys.insert(key);
			continue;
		}
		if (status != maj0sted::editor::TileStatus::Ready)
		{
			continue;
		}

		if (m_textures.find(key) == m_textures.end())
		{
			// the first time a tile is ready is also the moment its bytes are known to be good
			save_to_disk(key);
			m_textures.emplace(key, upload(key));
		}
	}

	// draw every texture we already hold that still covers the view. returning only the fetch
	// window made tiles blink out during zoom as soon as they fell outside the cap, then blink
	// back in once the window moved over them again
	auto const overlaps = [](maj0sted::editor::TileBBox const &Left, maj0sted::editor::TileBBox const &Right) {
		return Left.min_x < Right.max_x && Left.max_x > Right.min_x && Left.min_y < Right.max_y && Left.max_y > Right.min_y;
	};

	std::vector<ready_tile> tiles;
	tiles.reserve(m_textures.size());
	for (auto const &[key, texture] : m_textures)
	{
		if (texture == 0)
		{
			continue;
		}
		auto const box = maj0sted::editor::TileGrid::bbox(key);
		if (overlaps(box, View))
		{
			tiles.push_back({box, texture});
		}
	}

	return tiles;
}

unsigned int orthophoto_source::upload(maj0sted::editor::TileKey const &Key)
{
	auto const *image = m_service.peek(Key);
	if (image == nullptr || image->bytes.empty())
	{
		return 0;
	}

	int width = 0;
	int height = 0;
	int channels = 0;
	auto *pixels = stbi_load_from_memory(image->bytes.data(), static_cast<int>(image->bytes.size()), &width, &height, &channels, 3);
	if (pixels == nullptr)
	{
		ErrorLog("Orthophoto: could not decode tile imagery");
		return 0;
	}

	GLuint texture = 0;
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	stbi_image_free(pixels);

	return texture;
}

std::string orthophoto_source::tile_path(maj0sted::editor::TileKey const &Key) const
{
	return "editor/cache/" + m_cachesubdir + "/" + std::to_string(Key.zoom) + "_" + std::to_string(Key.x) + "_" + std::to_string(Key.y) + ".img";
}

bool orthophoto_source::load_from_disk(maj0sted::editor::TileKey const &Key)
{
	std::ifstream file(tile_path(Key), std::ios::binary);
	if (false == file.is_open())
	{
		return false;
	}

	std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
	if (bytes.empty())
	{
		return false;
	}

	m_service.store(Key, std::move(bytes));

	return true;
}

void orthophoto_source::save_to_disk(maj0sted::editor::TileKey const &Key)
{
	auto const *image = m_service.peek(Key);
	if (image == nullptr || image->bytes.empty())
	{
		return;
	}

	auto const path = tile_path(Key);
	std::error_code error;
	std::filesystem::create_directories(std::filesystem::path(path).parent_path(), error);
	if (std::filesystem::exists(path, error))
	{
		return; // came from the disk in the first place
	}

	std::ofstream file(path, std::ios::binary);
	if (file.is_open())
	{
		file.write(reinterpret_cast<char const *>(image->bytes.data()), static_cast<std::streamsize>(image->bytes.size()));
	}
}

std::size_t orthophoto_source::pending() const
{
	return m_inflight.load();
}

std::size_t orthophoto_source::failed() const
{
	return m_failedkeys.size();
}

namespace
{

// decodes encoded image bytes straight onto a fresh texture; 0 when they are not an image
unsigned int upload_image(std::vector<std::uint8_t> const &Bytes)
{
	int width = 0;
	int height = 0;
	int channels = 0;
	auto *pixels = stbi_load_from_memory(Bytes.data(), static_cast<int>(Bytes.size()), &width, &height, &channels, 3);
	if (pixels == nullptr)
	{
		return 0;
	}

	GLuint texture = 0;
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	stbi_image_free(pixels);

	return texture;
}

bool same_box(maj0sted::editor::TileBBox const &Left, maj0sted::editor::TileBBox const &Right)
{
	auto const close = [](double const A, double const B) { return std::abs(A - B) < 1.0; };
	return close(Left.min_x, Right.min_x) && close(Left.min_y, Right.min_y) && close(Left.max_x, Right.max_x) && close(Left.max_y, Right.max_y);
}

} // namespace

wms_image::wms_image(maj0sted::editor::WmsConfig Config) : m_config(std::move(Config))
{
	m_worker = std::thread([this]() { worker(); });
}

wms_image::~wms_image()
{
	m_quitting = true;
	m_wakeup.notify_all();
	if (m_worker.joinable())
	{
		m_worker.join();
	}
	if (m_texture != 0)
	{
		auto const id = m_texture;
		glDeleteTextures(1, &id);
	}
}

void wms_image::worker()
{
	while (false == m_quitting)
	{
		std::string url;
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			m_wakeup.wait(lock, [this]() { return m_quitting || m_haswork; });
			if (m_quitting)
			{
				return;
			}
			url = m_url;
			m_haswork = false;
		}

		std::vector<std::uint8_t> bytes;
		std::string contenttype;
		for (auto attempt = 0; attempt < kAttempts && false == m_quitting; ++attempt)
		{
			if (http_get(url, bytes, contenttype))
			{
				break;
			}
			bytes.clear();
		}

		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_bytes = std::move(bytes);
			m_hasresult = true;
		}
	}
}

unsigned int wms_image::texture_for(maj0sted::editor::TileBBox const &Box, int const Pixels)
{
	// a finished download becomes the texture on this thread, where the graphics API is safe to touch
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_hasresult)
		{
			m_hasresult = false;
			m_loading = false;
			if (false == m_bytes.empty())
			{
				auto const uploaded = upload_image(m_bytes);
				if (uploaded != 0)
				{
					if (m_texture != 0)
					{
						auto const old = m_texture;
						glDeleteTextures(1, &old);
					}
					m_texture = uploaded;
					m_covered = m_inflightbox;
				}
			}
			m_bytes.clear();
		}
	}

	// only chase a genuinely different view, and only once the previous one has landed
	if (false == same_box(Box, m_requested) && false == m_loading.load())
	{
		m_requested = Box;
		m_inflightbox = Box;
		auto config = m_config;
		config.tile_pixels = Pixels;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_url = maj0sted::editor::wms_getmap_url(config, Box);
			m_haswork = true;
		}
		m_loading = true;
		m_wakeup.notify_one();
	}

	return m_texture;
}

} // namespace editor
