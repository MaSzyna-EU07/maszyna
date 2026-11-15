/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorNmt.hpp"
#include "editor/editorHttp.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorGeoTiff.hpp"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>

namespace
{

namespace fs = std::filesystem;

// the model at 1 m of the WCS service of geoportal.gov.pl; the name of the model is read from its capabilities
char const *const service{"https://mapy.geoportal.gov.pl/wss/service/PZGIK/NMT/GRID1/WCS/DigitalTerrainModelFormatTIFF"};
char const *const fallback_coverage{"DTM_PL-KRON86-NH_TIFF"};
char const *const game_cache_folder{"editor/cache/nmt/grid1"}; // in the game folder: older builds kept the squares there
int constexpr worker_count{2}; // the service is quick, but isn't to be hammered
int constexpr border{1}; // pixels of the neighbouring squares fetched around a square, so the heights interpolate across its edges
float constexpr no_data{-1000.f}; // the model gives far lower values outside of the country
double constexpr retry_after{15.0}; // s, a square the service didn't give is asked for again

double seconds()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// among the files kept to be fetched again faster, as the orthophoto: the game folder may not be writable. Without a folder of the
// user's cache, in the game folder
fs::path cache_root()
{
	auto const root{user_cache_path("nmt")};
	return root.empty() ? fs::path{game_cache_folder} : root / "grid1";
}

bool read_file(fs::path const &Path, std::vector<std::uint8_t> &Out)
{
	std::ifstream file{Path, std::ios::binary};
	if (false == file.is_open())
		return false;
	Out.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
	return true;
}

// written under a name of its own and renamed, so a write broken off never leaves a part of a square behind
bool write_file(fs::path const &Path, std::vector<std::uint8_t> const &Data)
{
	std::error_code fault;
	fs::create_directories(Path.parent_path(), fault);
	std::ostringstream suffix;
	suffix << ".part" << std::this_thread::get_id();
	auto temporary{Path};
	temporary += suffix.str();
	{
		std::ofstream file{temporary, std::ios::binary | std::ios::trunc};
		if (false == file.is_open())
			return false;
		file.write(reinterpret_cast<char const *>(Data.data()), static_cast<std::streamsize>(Data.size()));
		if (false == static_cast<bool>(file))
		{
			file.close();
			fs::remove(temporary, fault);
			return false;
		}
	}
	fs::rename(temporary, Path, fault);
	if (fault)
	{
		fs::remove(temporary, fault);
		return false;
	}
	return true;
}

// the name of the model among the names the capabilities of the service list: the first which says it's a terrain model
std::string coverage_of(std::string const &Capabilities)
{
	std::vector<std::string> names;
	for (std::size_t at{0}; (at = Capabilities.find("name>", at)) != std::string::npos;)
	{
		at += 5;
		auto const end{Capabilities.find('<', at)};
		if (end == std::string::npos)
			break;
		names.emplace_back(Capabilities.substr(at, end - at));
	}
	for (auto const &name : names)
	{
		std::string lower{name};
		std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char const Letter) { return static_cast<char>(std::tolower(Letter)); });
		if (lower.find("dtm") != std::string::npos || lower.find("nmt") != std::string::npos || lower.find("terrain") != std::string::npos)
			return name;
	}
	return fallback_coverage;
}

} // namespace

editor_nmt::editor_nmt() = default;

editor_nmt::~editor_nmt()
{
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		m_stop = true;
		m_queue.clear();
	}
	m_wake.notify_all();
	for (auto &worker : m_workers)
		if (worker.joinable())
			worker.join();
}

bool editor_nmt::can_download()
{
	return editor_http::available();
}

void editor_nmt::place(double const North, double const East)
{
	std::lock_guard<std::mutex> lock{m_mutex};
	m_north = North;
	m_east = East;
}

bool editor_nmt::placed() const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	return m_north != 0.0 || m_east != 0.0;
}

editor_nmt::key editor_nmt::key_of(double const East, double const North) const
{
	return {static_cast<int>(std::floor(East / square_size)), static_cast<int>(std::floor(North / square_size))};
}

bool editor_nmt::height(double const X, double const Z, double &Height) const
{
	std::shared_ptr<square const> found;
	double east, north;
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		east = m_east - X;
		north = m_north + Z;
		auto const it{m_squares.find(key_of(east, north))};
		if (it == m_squares.end())
			return false;
		found = it->second;
	}
	auto const &grid{*found};
	if (grid.failed)
		return false;
	// the values lie in the middles of the pixels
	auto const column{(east - grid.east) / grid.pixel - 0.5};
	auto const row{(grid.north - north) / grid.pixel - 0.5};
	auto const c0{std::clamp(static_cast<int>(std::floor(column)), 0, grid.width - 2)};
	auto const r0{std::clamp(static_cast<int>(std::floor(row)), 0, grid.height - 2)};
	auto const fc{std::clamp(column - c0, 0.0, 1.0)};
	auto const fr{std::clamp(row - r0, 0.0, 1.0)};
	auto const at = [&](int const C, int const R) { return grid.heights[static_cast<std::size_t>(R) * grid.width + C]; };
	float const corners[4]{at(c0, r0), at(c0 + 1, r0), at(c0, r0 + 1), at(c0 + 1, r0 + 1)};
	if (std::all_of(std::begin(corners), std::end(corners), [](float const Value) { return std::isfinite(Value) && Value > no_data; }))
	{
		Height = (corners[0] * (1.0 - fc) + corners[1] * fc) * (1.0 - fr) + (corners[2] * (1.0 - fc) + corners[3] * fc) * fr;
		return true;
	}
	// at the edge of the data: the nearest value there is
	auto const nearest{corners[(fr > 0.5 ? 2 : 0) + (fc > 0.5 ? 1 : 0)]};
	if (false == std::isfinite(nearest) || nearest <= no_data)
		return false;
	Height = nearest;
	return true;
}

bool editor_nmt::request(glm::dvec2 const &Min, glm::dvec2 const &Max)
{
	std::vector<key> wanted;
	bool all{true};
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		auto const low{key_of(m_east - Max.x, m_north + Min.y)};
		auto const high{key_of(m_east - Min.x, m_north + Max.y)};
		auto const middle{key_of(m_east - 0.5 * (Min.x + Max.x), m_north + 0.5 * (Min.y + Max.y))};
		auto const now{seconds()};
		for (auto e = low.first; e <= high.first; ++e)
			for (auto n = low.second; n <= high.second; ++n)
			{
				key const square{e, n};
				if (auto const found{m_squares.find(square)}; found != m_squares.end())
				{
					if (false == found->second->failed || found->second->outside || now - found->second->failed_at < retry_after)
						continue;
					m_squares.erase(found);
				}
				all = false;
				if (false == m_queued.contains(square))
					wanted.push_back(square);
			}
		std::sort(wanted.begin(), wanted.end(), [&](key const &Left, key const &Right) {
			auto const distance = [&](key const &Square) { return std::abs(Square.first - middle.first) + std::abs(Square.second - middle.second); };
			return distance(Left) < distance(Right);
		});
		for (auto const &square : wanted)
		{
			m_queued.insert(square);
			m_queue.push_back(square);
		}
		if (false == wanted.empty())
			start_workers();
	}
	if (false == wanted.empty())
		m_wake.notify_all();
	return all;
}

void editor_nmt::start_workers()
{
	if (m_workers.empty())
		for (int i = 0; i < worker_count; ++i)
			m_workers.emplace_back([this]() { work(); });
}

std::filesystem::path editor_nmt::file_of(key const &Key) const
{
	return cache_root() / (std::to_string(Key.first) + "_" + std::to_string(Key.second) + ".tif");
}

void editor_nmt::prefetch(std::vector<glm::dvec2> const &Polygon)
{
	std::vector<key> keys;
	if (can_download())
		for (auto const &square : geometry::squares_in(Polygon, square_size))
			keys.emplace_back(square);
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		m_prefetch = std::move(keys);
		m_prefetch_next = 0;
		m_prefetch_again.clear();
		m_prefetch_cached.clear();
		m_prefetch_failed.clear();
		++m_prefetch_generation;
		if (false == m_prefetch.empty())
			start_workers();
	}
	m_wake.notify_all();
}

void editor_nmt::prefetch_paused(bool const Paused)
{
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		m_prefetch_paused = Paused;
	}
	m_wake.notify_all();
}

editor_nmt::prefetch_state editor_nmt::prefetched() const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	return {static_cast<int>(m_prefetch_cached.size()), static_cast<int>(m_prefetch.size()), static_cast<int>(m_prefetch_failed.size())};
}

std::size_t editor_nmt::waiting() const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	return m_queued.size();
}

bool editor_nmt::retrying(glm::dvec2 const &Min, glm::dvec2 const &Max) const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	auto const low{key_of(m_east - Max.x, m_north + Min.y)};
	auto const high{key_of(m_east - Min.x, m_north + Max.y)};
	for (auto e = low.first; e <= high.first; ++e)
		for (auto n = low.second; n <= high.second; ++n)
			if (auto const found{m_squares.find({e, n})}; found != m_squares.end() && found->second->failed && false == found->second->outside)
				return true;
	return false;
}

void editor_nmt::retry()
{
	std::lock_guard<std::mutex> lock{m_mutex};
	std::erase_if(m_squares, [](auto const &Square) { return Square.second->failed && false == Square.second->outside; });
}

void editor_nmt::cancel()
{
	std::lock_guard<std::mutex> lock{m_mutex};
	for (auto const &square : m_queue)
		m_queued.erase(square);
	m_queue.clear();
}

std::string editor_nmt::error(bool *Again) const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	std::string result;
	bool outside{false};
	if (Again != nullptr)
		*Again = false;
	for (auto const &[key, square] : m_squares)
	{
		if (false == square->failed)
			continue;
		if (Again != nullptr && false == square->outside)
			*Again = true;
		if (result.empty() || (square->outside && false == outside))
		{
			result = square->error;
			outside = square->outside;
		}
	}
	return result;
}

void editor_nmt::clear()
{
	std::lock_guard<std::mutex> lock{m_mutex};
	m_queue.clear();
	m_queued.clear();
	m_squares.clear();
}

void editor_nmt::work()
{
	while (true)
	{
		key square;
		bool prefetching{false};
		std::uint32_t generation{0};
		{
			std::unique_lock<std::mutex> lock{m_mutex};
			auto const prefetchable = [this]() { return false == m_prefetch_paused && (false == m_prefetch_again.empty() || m_prefetch_next < m_prefetch.size()); };
			m_wake.wait(lock, [&]() { return m_stop || false == m_queue.empty() || prefetchable(); });
			if (m_stop)
				return;
			if (false == m_queue.empty())
			{
				square = m_queue.front();
				m_queue.pop_front();
			}
			else
			{
				// a square of the download area, while none is asked for; one broken off before first
				if (false == m_prefetch_again.empty())
				{
					square = m_prefetch_again.front();
					m_prefetch_again.pop_front();
				}
				else
					square = m_prefetch[m_prefetch_next++];
				prefetching = true;
				generation = m_prefetch_generation;
			}
		}
		if (prefetching)
		{
			// put into the disk cache only, the file there not read
			std::error_code fault;
			std::string error;
			bool yielded{false};
			auto const ok{fs::exists(file_of(square), fault) || false == fetch(square, error, &yielded)->failed};
			std::lock_guard<std::mutex> lock{m_mutex};
			if (generation != m_prefetch_generation)
				continue;
			if (yielded)
				m_prefetch_again.push_back(square);
			else
				(ok ? m_prefetch_cached : m_prefetch_failed).insert(square);
			continue;
		}
		std::string error;
		auto fetched{fetch(square, error)};
		{
			std::lock_guard<std::mutex> lock{m_mutex};
			if (false == m_queued.contains(square))
				continue; // cleared meanwhile
			m_queued.erase(square);
			m_squares[square] = std::move(fetched);
		}
	}
}

std::shared_ptr<editor_nmt::square const> editor_nmt::fetch(key const &Key, std::string &Error, bool *Yielded)
{
	auto result{std::make_shared<square>()};
	std::error_code fault;
	auto const file{file_of(Key)};
	std::vector<std::uint8_t> data;
	if (read_file(file, data))
	{
		if (editor_geotiff::read(data, *result, no_data - 1.f))
			return result;
		// damaged: fetched again
		fs::remove(file, fault);
	}
	// a square an older build kept in the game folder is moved to the cache, as it's asked for
	if (auto const old{fs::path{game_cache_folder} / file.filename()}; old != file && read_file(old, data))
	{
		auto const whole{editor_geotiff::read(data, *result, no_data - 1.f)};
		if (whole)
		{
			// moved as it is when the cache is on the same disk, written there and deleted otherwise; one that can't be
			// written there stays where it is
			fs::create_directories(file.parent_path(), fault);
			fs::rename(old, file, fault);
			if (fault && write_file(file, data))
				fs::remove(old, fault);
			return result;
		}
		// damaged
		fs::remove(old, fault);
	}
	if (false == can_download())
	{
		Error = STRN("This build can't fetch from the internet");
		result->failed = true;
		result->outside = true;
		result->error = Error;
		return result;
	}
	editor_http::client client;
	// the download of the area gives way to the squares asked for: the terrain under the track laid isn't kept waiting for it
	auto const yield = [this, Yielded]() {
		std::lock_guard<std::mutex> lock{m_mutex};
		return Yielded != nullptr && false == m_queue.empty();
	};
	auto const stop = [this, &yield]() {
		{
			std::lock_guard<std::mutex> lock{m_mutex};
			if (m_stop)
				return true;
		}
		return yield();
	};
	std::string coverage;
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		coverage = m_coverage;
	}
	if (coverage.empty())
	{
		std::vector<std::uint8_t> capabilities;
		std::string unused;
		if (client.get(std::string{service} + "?SERVICE=WCS&VERSION=1.0.0&REQUEST=GetCapabilities", capabilities, unused, stop))
		{
			coverage = coverage_of(std::string(capabilities.begin(), capabilities.end()));
			std::lock_guard<std::mutex> lock{m_mutex};
			m_coverage = coverage;
		}
		else
		{
			// for this square only: the capabilities are asked for again with the next one
			coverage = fallback_coverage;
		}
	}
	// the square with a pixel of each neighbour around it
	auto const pixels{static_cast<int>(square_size) + 2 * border};
	auto const east{Key.first * square_size - border};
	auto const north{Key.second * square_size - border};
	char bbox[128];
	std::snprintf(bbox, sizeof(bbox), "%.0f,%.0f,%.0f,%.0f", east, north, east + pixels, north + pixels);
	auto const url{std::string{service} + "?SERVICE=WCS&VERSION=1.0.0&REQUEST=GetCoverage&COVERAGE=" + coverage + "&CRS=EPSG:2180&RESPONSE_CRS=EPSG:2180&BBOX=" + bbox +
	               "&WIDTH=" + std::to_string(pixels) + "&HEIGHT=" + std::to_string(pixels) + "&FORMAT=GEOTIFF"};
	for (int attempt = 0; attempt < 3; ++attempt)
	{
		if (client.get(url, data, Error, stop) && editor_geotiff::read(data, *result, no_data - 1.f))
		{
			write_file(file, data);
			Error.clear();
			return result;
		}
		if (stop())
			break;
		if (std::string const text(data.begin(), data.end()); text.find("outside requested coverage") != std::string::npos)
		{
			Error = STRN("The place lies outside of the terrain model of Poland: check the location of the scenery");
			result->outside = true;
			break;
		}
		if (Error.empty())
			Error = STRN("The service gave no terrain model");
	}
	if (Yielded != nullptr && yield())
	{
		// broken off, not failed: downloaded again later
		*Yielded = true;
		result->failed = true;
		return result;
	}
	WriteLog("Editor: NMT square " + std::to_string(Key.first) + "," + std::to_string(Key.second) + " not fetched: " + Error, logtype::generic);
	result->failed = true;
	result->error = Error;
	result->failed_at = seconds();
	return result;
}
