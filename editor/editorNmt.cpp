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
#include "utilities/Logs.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace
{

namespace fs = std::filesystem;

// the model at 1 m of the WCS service of geoportal.gov.pl; the name of the model is read from its capabilities
char const *const service{"https://mapy.geoportal.gov.pl/wss/service/PZGIK/NMT/GRID1/WCS/DigitalTerrainModelFormatTIFF"};
char const *const fallback_coverage{"DTM_PL-KRON86-NH_TIFF"};
char const *const cache_folder{"editor/cache/nmt/grid1/"};
int constexpr worker_count{2}; // the service is quick, but isn't to be hammered
int constexpr border{1}; // pixels of the neighbouring squares fetched around a square, so the heights interpolate across its edges
float constexpr no_data{-1000.f}; // the model gives far lower values outside of the country

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

// a GeoTIFF as the service gives it: one band of 32-bit floats in strips, not compressed, placed on the grid by its tie point
// and pixel scale. false for anything else
template <typename Square>
bool read_geotiff(std::vector<std::uint8_t> const &Data, Square &Out)
{
	if (Data.size() < 8)
		return false;
	bool const little{Data[0] == 'I' && Data[1] == 'I'};
	if (false == little && false == (Data[0] == 'M' && Data[1] == 'M'))
		return false;
	auto const u16 = [&](std::size_t const At) -> std::uint32_t {
		if (At + 2 > Data.size())
			return 0;
		return little ? Data[At] | (Data[At + 1] << 8) : (Data[At] << 8) | Data[At + 1];
	};
	auto const u32 = [&](std::size_t const At) -> std::uint32_t {
		if (At + 4 > Data.size())
			return 0;
		return little ? Data[At] | (Data[At + 1] << 8) | (Data[At + 2] << 16) | (static_cast<std::uint32_t>(Data[At + 3]) << 24) :
		                (static_cast<std::uint32_t>(Data[At]) << 24) | (Data[At + 1] << 16) | (Data[At + 2] << 8) | Data[At + 3];
	};
	auto const f64 = [&](std::size_t const At) {
		std::uint64_t bits{0};
		for (int i = 0; i < 8; ++i)
			bits |= static_cast<std::uint64_t>(At + i < Data.size() ? Data[At + (little ? i : 7 - i)] : 0) << (8 * i);
		double value;
		std::memcpy(&value, &bits, sizeof(value));
		return value;
	};
	if (u16(2) != 42)
		return false;
	struct entry
	{
		std::uint32_t type{0};
		std::uint32_t count{0};
		std::size_t at{0}; // of the values: in the entry when they fit in four bytes
	};
	std::map<std::uint32_t, entry> entries;
	auto const ifd{static_cast<std::size_t>(u32(4))};
	auto const count{u16(ifd)};
	for (std::uint32_t i = 0; i < count; ++i)
	{
		auto const at{ifd + 2 + i * 12};
		entry value{u16(at + 2), u32(at + 4), at + 8};
		auto const size{(value.type == 3 ? 2u : value.type == 12 ? 8u : 4u) * value.count};
		if (size > 4)
			value.at = u32(at + 8);
		entries[u16(at)] = value;
	}
	auto const number = [&](std::uint32_t const Tag, std::uint32_t const Index) -> std::uint32_t {
		auto const found{entries.find(Tag)};
		if (found == entries.end() || Index >= found->second.count)
			return 0;
		return found->second.type == 3 ? u16(found->second.at + Index * 2) : u32(found->second.at + Index * 4);
	};
	auto const real = [&](std::uint32_t const Tag, std::uint32_t const Index) {
		auto const found{entries.find(Tag)};
		return found == entries.end() || Index >= found->second.count ? 0.0 : f64(found->second.at + Index * 8);
	};
	auto const width{static_cast<int>(number(256, 0))};
	auto const height{static_cast<int>(number(257, 0))};
	if (width < 2 || height < 2 || number(258, 0) != 32 || number(339, 0) != 3 || std::max(1u, number(259, 0)) != 1 || false == entries.contains(273) || false == entries.contains(33550) ||
	    false == entries.contains(33922))
		return false;
	auto const rows{std::max(1u, number(278, 0))};
	Out.width = width;
	Out.height = height;
	Out.heights.assign(static_cast<std::size_t>(width) * height, no_data - 1.f);
	auto const strips{entries[273].count};
	for (std::uint32_t strip = 0; strip < strips; ++strip)
	{
		auto const offset{static_cast<std::size_t>(number(273, strip))};
		auto const first{strip * rows};
		for (std::uint32_t row = first; row < std::min<std::uint32_t>(first + rows, height); ++row)
			for (int column = 0; column < width; ++column)
			{
				auto const bits{u32(offset + ((row - first) * width + column) * 4)};
				float value;
				std::memcpy(&value, &bits, sizeof(value));
				Out.heights[static_cast<std::size_t>(row) * width + column] = value;
			}
	}
	Out.pixel = real(33550, 0);
	Out.east = real(33922, 3) - real(33922, 0) * Out.pixel;
	Out.north = real(33922, 4) + real(33922, 1) * Out.pixel;
	return Out.pixel > 0.0;
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
		for (auto e = low.first; e <= high.first; ++e)
			for (auto n = low.second; n <= high.second; ++n)
			{
				key const square{e, n};
				if (m_squares.contains(square))
					continue;
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
		if (false == wanted.empty() && m_workers.empty())
			for (int i = 0; i < worker_count; ++i)
				m_workers.emplace_back([this]() { work(); });
	}
	if (false == wanted.empty())
		m_wake.notify_all();
	return all;
}

std::size_t editor_nmt::waiting() const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	return m_queued.size();
}

std::string editor_nmt::error() const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	return m_error;
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
		{
			std::unique_lock<std::mutex> lock{m_mutex};
			m_wake.wait(lock, [this]() { return m_stop || false == m_queue.empty(); });
			if (m_stop)
				return;
			square = m_queue.front();
			m_queue.pop_front();
		}
		std::string error;
		auto fetched{fetch(square, error)};
		{
			std::lock_guard<std::mutex> lock{m_mutex};
			if (false == m_queued.contains(square))
				continue; // cleared meanwhile
			m_queued.erase(square);
			m_squares[square] = std::move(fetched);
			m_error = error;
		}
	}
}

std::shared_ptr<editor_nmt::square const> editor_nmt::fetch(key const &Key, std::string &Error)
{
	auto result{std::make_shared<square>()};
	std::error_code fault;
	fs::create_directories(cache_folder, fault);
	auto const file{std::string{cache_folder} + std::to_string(Key.first) + "_" + std::to_string(Key.second) + ".tif"};
	std::vector<std::uint8_t> data;
	if (std::ifstream cached{file, std::ios::binary}; cached.is_open())
	{
		data.assign(std::istreambuf_iterator<char>(cached), std::istreambuf_iterator<char>());
		if (read_geotiff(data, *result))
			return result;
		// damaged by a fetch broken off: fetched again
		fs::remove(file, fault);
	}
	if (false == can_download())
	{
		Error = "this build can't fetch from the internet";
		result->failed = true;
		return result;
	}
	editor_http::client client;
	auto const stop = [this]() {
		std::lock_guard<std::mutex> lock{m_mutex};
		return m_stop;
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
		coverage = client.get(std::string{service} + "?SERVICE=WCS&VERSION=1.0.0&REQUEST=GetCapabilities", capabilities, unused, stop) ?
		               coverage_of(std::string(capabilities.begin(), capabilities.end())) :
		               fallback_coverage;
		std::lock_guard<std::mutex> lock{m_mutex};
		m_coverage = coverage;
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
		if (client.get(url, data, Error, stop) && read_geotiff(data, *result))
		{
			if (std::ofstream out{file, std::ios::binary}; out.is_open())
				out.write(reinterpret_cast<char const *>(data.data()), static_cast<std::streamsize>(data.size()));
			Error.clear();
			return result;
		}
		if (stop())
			break;
		if (Error.empty())
			Error = "the service gave no terrain model";
	}
	WriteLog("Editor: NMT square " + std::to_string(Key.first) + "," + std::to_string(Key.second) + " not fetched: " + Error, logtype::generic);
	result->failed = true;
	return result;
}
