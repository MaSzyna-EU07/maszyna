/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorOrthophoto.hpp"

#include "rendering/renderer.h"
#include "scene/scene.h"
#include "simulation/simulation.h"
#include "model/vertex.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "imgui/imgui.h"
#include "stb/stb_image.h"

#if defined(_WIN32)
#include <winhttp.h>
#elif defined(EU07_WITH_CURL)
#include <curl/curl.h>
#endif

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <glad/glad.h>

namespace
{
namespace fs = std::filesystem;

constexpr int standard_pixels{1024}; // ~25 cm/px for a 256 m tile, matches the standard orthophoto
constexpr int hires_pixels{4096};    // ~6 cm/px, the WMS MaxWidth/MaxHeight of the geoportal services
constexpr int probe_pixels{64};      // coverage test of the high-resolution product
constexpr int worker_count{3};       // concurrent requests; the service is slow but shouldn't be hammered
constexpr std::size_t upload_budget{16u << 20}; // bytes of texture data uploaded per frame (always at least one tile)
constexpr std::size_t vertex_budget{60000};     // ImGui uses 16-bit indices and not every backend supports vertex offsets
constexpr float near_w{0.5f};

using stop_check = std::function<bool()>;

double steady_seconds()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ---------------------------------------------------------------------------------------------
// minimal blocking https GET; one instance per worker thread

class http_client
{
  public:
	http_client();
	~http_client();
	http_client(http_client const &) = delete;
	http_client &operator=(http_client const &) = delete;

	bool get(std::string const &Url, std::vector<std::uint8_t> &Out, std::string &Error, stop_check const &Stop);

  private:
#if defined(_WIN32)
	HINTERNET m_session{nullptr};
#elif defined(EU07_WITH_CURL)
	CURL *m_curl{nullptr};
#endif
};

#if defined(_WIN32)

http_client::http_client()
{
	wchar_t const *agent = L"MaSzyna-editor";
#ifdef WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY
	m_session = ::WinHttpOpen(agent, WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
#endif
	if (m_session == nullptr) // automatic proxy needs windows 8.1+
		m_session = ::WinHttpOpen(agent, WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	if (m_session != nullptr)
		::WinHttpSetTimeouts(m_session, 10000, 10000, 15000, 20000);
}

http_client::~http_client()
{
	if (m_session != nullptr)
		::WinHttpCloseHandle(m_session);
}

bool http_client::get(std::string const &Url, std::vector<std::uint8_t> &Out, std::string &Error, stop_check const &Stop)
{
	Out.clear();
	if (m_session == nullptr)
	{
		Error = "WinHttpOpen failed (" + std::to_string(::GetLastError()) + ")";
		return false;
	}

	std::wstring const url(Url.begin(), Url.end()); // the requests are plain ascii
	URL_COMPONENTS parts{};
	parts.dwStructSize = sizeof(parts);
	parts.dwSchemeLength = static_cast<DWORD>(-1);
	parts.dwHostNameLength = static_cast<DWORD>(-1);
	parts.dwUrlPathLength = static_cast<DWORD>(-1);
	parts.dwExtraInfoLength = static_cast<DWORD>(-1);
	if (!::WinHttpCrackUrl(url.c_str(), 0, 0, &parts))
	{
		Error = "invalid url";
		return false;
	}
	std::wstring const host(parts.lpszHostName, parts.dwHostNameLength);
	std::wstring const path = std::wstring(parts.lpszUrlPath, parts.dwUrlPathLength) + std::wstring(parts.lpszExtraInfo, parts.dwExtraInfoLength);

	HINTERNET connection = ::WinHttpConnect(m_session, host.c_str(), parts.nPort, 0);
	HINTERNET request = connection ? ::WinHttpOpenRequest(connection, L"GET", path.c_str(), nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES,
	                                                      parts.nScheme == INTERNET_SCHEME_HTTPS ? WINHTTP_FLAG_SECURE : 0)
	                               : nullptr;
	auto const close = [&]() {
		if (request != nullptr)
			::WinHttpCloseHandle(request);
		if (connection != nullptr)
			::WinHttpCloseHandle(connection);
	};

	if (request == nullptr || !::WinHttpSendRequest(request, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) || !::WinHttpReceiveResponse(request, nullptr))
	{
		Error = "WinHTTP error " + std::to_string(::GetLastError());
		close();
		return false;
	}

	DWORD status{0};
	DWORD statussize{sizeof(status)};
	::WinHttpQueryHeaders(request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &status, &statussize, WINHTTP_NO_HEADER_INDEX);

	for (;;)
	{
		if (Stop())
		{
			Error = "cancelled";
			close();
			return false;
		}
		DWORD available{0};
		if (!::WinHttpQueryDataAvailable(request, &available))
		{
			Error = "WinHTTP error " + std::to_string(::GetLastError());
			close();
			return false;
		}
		if (available == 0)
			break;
		std::size_t const offset = Out.size();
		Out.resize(offset + available);
		DWORD received{0};
		if (!::WinHttpReadData(request, Out.data() + offset, available, &received))
		{
			Error = "WinHTTP error " + std::to_string(::GetLastError());
			close();
			return false;
		}
		Out.resize(offset + received);
	}
	close();

	if (status != 200)
	{
		Error = "HTTP " + std::to_string(status);
		return false;
	}
	return true;
}

#elif defined(EU07_WITH_CURL)

http_client::http_client()
{
	static std::once_flag initialized;
	std::call_once(initialized, []() { ::curl_global_init(CURL_GLOBAL_DEFAULT); });
	m_curl = ::curl_easy_init();
}

http_client::~http_client()
{
	if (m_curl != nullptr)
		::curl_easy_cleanup(m_curl);
}

bool http_client::get(std::string const &Url, std::vector<std::uint8_t> &Out, std::string &Error, stop_check const &Stop)
{
	Out.clear();
	if (m_curl == nullptr)
	{
		Error = "curl_easy_init failed";
		return false;
	}

	auto const write = +[](char *Data, std::size_t Size, std::size_t Count, void *User) -> std::size_t {
		auto *out = static_cast<std::vector<std::uint8_t> *>(User);
		out->insert(out->end(), Data, Data + Size * Count);
		return Size * Count;
	};
	auto const progress = +[](void *User, curl_off_t, curl_off_t, curl_off_t, curl_off_t) -> int { return (*static_cast<stop_check const *>(User))() ? 1 : 0; };

	::curl_easy_reset(m_curl);
	::curl_easy_setopt(m_curl, CURLOPT_URL, Url.c_str());
	::curl_easy_setopt(m_curl, CURLOPT_USERAGENT, "MaSzyna-editor");
	::curl_easy_setopt(m_curl, CURLOPT_FOLLOWLOCATION, 1L);
	::curl_easy_setopt(m_curl, CURLOPT_NOSIGNAL, 1L);
	::curl_easy_setopt(m_curl, CURLOPT_CONNECTTIMEOUT, 10L);
	::curl_easy_setopt(m_curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
	::curl_easy_setopt(m_curl, CURLOPT_LOW_SPEED_TIME, 20L);
	::curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, write);
	::curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, &Out);
	::curl_easy_setopt(m_curl, CURLOPT_NOPROGRESS, 0L);
	::curl_easy_setopt(m_curl, CURLOPT_XFERINFOFUNCTION, progress);
	::curl_easy_setopt(m_curl, CURLOPT_XFERINFODATA, const_cast<stop_check *>(&Stop));

	CURLcode const code = ::curl_easy_perform(m_curl);
	if (code != CURLE_OK)
	{
		Error = ::curl_easy_strerror(code);
		return false;
	}
	long status{0};
	::curl_easy_getinfo(m_curl, CURLINFO_RESPONSE_CODE, &status);
	if (status != 200)
	{
		Error = "HTTP " + std::to_string(status);
		return false;
	}
	return true;
}

#else

http_client::http_client() = default;
http_client::~http_client() = default;

bool http_client::get(std::string const &, std::vector<std::uint8_t> &Out, std::string &Error, stop_check const &)
{
	Out.clear();
	Error = "no http client in this build";
	return false;
}

#endif

// ---------------------------------------------------------------------------------------------
// geoportal requests and the disk cache

struct tile_request
{
	int ix{0}; // easting index
	int iy{0}; // northing index
	int level{standard_pixels};
	int year{0};
	bool hires{false};
};

struct image
{
	int width{0};
	int height{0};
	std::vector<std::uint8_t> rgba;
};

// the archival services return the newest imagery taken up to TIME, but a TIME in the future yields a blank
// image, so the end of the requested year is clamped to the current month
std::string time_parameter(int Year)
{
	using namespace std::chrono;
	year_month_day const today{floor<days>(system_clock::now())};
	int year = Year;
	unsigned month = 12;
	int const thisyear = static_cast<int>(today.year());
	if (year >= thisyear)
	{
		year = thisyear;
		month = static_cast<unsigned>(today.month());
	}
	char buffer[32];
	std::snprintf(buffer, sizeof(buffer), "%04d-%02u-01", year, month);
	return buffer;
}

std::string wms_url(tile_request const &Request, bool Hires, int Pixels, bool Png)
{
	std::string const service = Hires ? (Request.year ? "HighResolutionTime" : "HighResolution") : (Request.year ? "StandardResolutionTime" : "StandardResolution");
	std::string const layer = (Hires && Request.year) ? "Image" : "Raster";
	auto const size = static_cast<long long>(editor_orthophoto::tile_size);
	long long const minn = Request.iy * size;
	long long const mine = Request.ix * size;

	std::string url = "https://mapy.geoportal.gov.pl/wss/service/PZGIK/ORTO/WMS/" + service +
	                  "?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap&CRS=EPSG:2180&STYLES=&LAYERS=" + layer;
	url += Png ? "&FORMAT=image/png&TRANSPARENT=TRUE" : "&FORMAT=image/jpeg";
	url += "&WIDTH=" + std::to_string(Pixels) + "&HEIGHT=" + std::to_string(Pixels);
	// WMS 1.3.0 uses the axis order of the CRS, which for EPSG:2180 is northing first
	url += "&BBOX=" + std::to_string(minn) + "," + std::to_string(mine) + "," + std::to_string(minn + size) + "," + std::to_string(mine + size);
	if (Request.year)
		url += "&TIME=" + time_parameter(Request.year);
	return url;
}

fs::path cache_root()
{
	fs::path root = user_config_path(".cache");
	if (root.empty())
		root = ".cache";
	return root / "geoportal";
}

fs::path cache_file(tile_request const &Request, bool Hires, std::string const &Suffix)
{
	std::string const product = std::string(Hires ? "hr_" : "sr_") + (Request.year ? std::to_string(Request.year) : std::string("latest"));
	std::string const name = std::to_string(static_cast<int>(editor_orthophoto::tile_size)) + "_" + std::to_string(Request.iy) + "_" + std::to_string(Request.ix) + "_" + Suffix;
	return cache_root() / product / name;
}

bool is_image(std::vector<std::uint8_t> const &Data)
{
	if (Data.size() >= 3 && Data[0] == 0xFF && Data[1] == 0xD8 && Data[2] == 0xFF)
		return true; // jpeg
	return Data.size() >= 4 && Data[0] == 0x89 && Data[1] == 'P' && Data[2] == 'N' && Data[3] == 'G';
}

bool read_file(fs::path const &Path, std::vector<std::uint8_t> &Out)
{
	std::ifstream file(Path, std::ios::binary);
	if (!file.is_open())
		return false;
	Out.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
	return !Out.empty();
}

void write_file(fs::path const &Path, std::vector<std::uint8_t> const &Data)
{
	std::error_code ec;
	fs::create_directories(Path.parent_path(), ec);
	// write under a per-thread name and rename, so an interrupted write never leaves a truncated tile behind
	std::ostringstream suffix;
	suffix << ".part" << std::this_thread::get_id();
	fs::path temporary = Path;
	temporary += suffix.str();
	{
		std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
		if (!file.is_open())
			return;
		file.write(reinterpret_cast<char const *>(Data.data()), static_cast<std::streamsize>(Data.size()));
		if (!file)
		{
			file.close();
			fs::remove(temporary, ec);
			return;
		}
	}
	fs::rename(temporary, Path, ec);
	if (ec)
		fs::remove(temporary, ec);
}

// cached file if present, otherwise downloads (with a few retries, the service drops connections and answers 404 now and then)
bool fetch(http_client &Client, std::string const &Url, fs::path const &Path, std::vector<std::uint8_t> &Out, std::string &Error, stop_check const &Stop)
{
	if (read_file(Path, Out) && is_image(Out))
		return true;

	for (int attempt = 0; attempt < 4; ++attempt)
	{
		if (Stop())
		{
			Error = "cancelled";
			return false;
		}
		if (attempt > 0)
		{
			// back off, but stay responsive to shutdown
			for (int wait = 0; wait < attempt * 5 && !Stop(); ++wait)
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		if (!Client.get(Url, Out, Error, Stop))
			continue;
		if (!is_image(Out))
		{
			// service exception report (xml) or similar
			Error = "unexpected response: " + std::string(Out.begin(), Out.begin() + std::min<std::size_t>(Out.size(), 200));
			continue;
		}
		write_file(Path, Out);
		return true;
	}
	return false;
}

bool decode(std::vector<std::uint8_t> const &Data, image &Out, std::string &Error)
{
	int width{0}, height{0}, components{0};
	stbi_uc *pixels = stbi_load_from_memory(Data.data(), static_cast<int>(Data.size()), &width, &height, &components, 4);
	if (pixels == nullptr)
	{
		Error = std::string("image decoding failed: ") + stbi_failure_reason();
		return false;
	}
	Out.width = width;
	Out.height = height;
	Out.rgba.assign(pixels, pixels + static_cast<std::size_t>(width) * height * 4);
	stbi_image_free(pixels);
	return true;
}

// 2x2 box filter
void halve(image &Image)
{
	int const width = std::max(1, Image.width / 2);
	int const height = std::max(1, Image.height / 2);
	std::vector<std::uint8_t> out(static_cast<std::size_t>(width) * height * 4);
	for (int y = 0; y < height; ++y)
		for (int x = 0; x < width; ++x)
			for (int c = 0; c < 4; ++c)
			{
				auto const at = [&](int X, int Y) { return static_cast<unsigned>(Image.rgba[(static_cast<std::size_t>(std::min(Y, Image.height - 1)) * Image.width + std::min(X, Image.width - 1)) * 4 + c]); };
				out[(static_cast<std::size_t>(y) * width + x) * 4 + c] = static_cast<std::uint8_t>((at(2 * x, 2 * y) + at(2 * x + 1, 2 * y) + at(2 * x, 2 * y + 1) + at(2 * x + 1, 2 * y + 1) + 2) / 4);
			}
	Image.width = width;
	Image.height = height;
	Image.rgba = std::move(out);
}

// areas without imagery come back plain white (jpeg) or fully transparent (png)
bool is_blank(image const &Image)
{
	for (std::size_t i = 0; i + 3 < Image.rgba.size(); i += 4)
		if (Image.rgba[i + 3] != 0 && (Image.rgba[i] < 250 || Image.rgba[i + 1] < 250 || Image.rgba[i + 2] < 250))
			return false;
	return true;
}

bool load_standard(http_client &Client, tile_request const &Request, image &Out, std::string &Error, stop_check const &Stop)
{
	std::vector<std::uint8_t> data;
	return fetch(Client, wms_url(Request, false, standard_pixels, false), cache_file(Request, false, std::to_string(standard_pixels) + ".jpg"), data, Error, Stop) && decode(data, Out, Error);
}

// returns false on failure (to be retried later); Empty is set when the place has no imagery
bool load_tile(http_client &Client, tile_request const &Request, image &Out, bool &Empty, std::string &Error, stop_check const &Stop)
{
	Empty = false;
	if (Request.hires && Request.level >= hires_pixels)
	{
		// the high-resolution product covers only parts of the country; a tiny transparent png tells whether this tile is covered
		std::vector<std::uint8_t> data;
		image probe;
		if (!fetch(Client, wms_url(Request, true, probe_pixels, true), cache_file(Request, true, "probe.png"), data, Error, Stop) || !decode(data, probe, Error))
			return false;
		std::size_t covered{0};
		for (std::size_t i = 3; i < probe.rgba.size(); i += 4)
			covered += (probe.rgba[i] != 0);
		std::size_t const total = probe.rgba.size() / 4;

		if (covered == total)
		{
			return fetch(Client, wms_url(Request, true, hires_pixels, false), cache_file(Request, true, std::to_string(hires_pixels) + ".jpg"), data, Error, Stop) && decode(data, Out, Error);
		}
		if (covered > 0)
		{
			// edge of the covered area: transparent png, holes filled from the standard imagery
			if (!fetch(Client, wms_url(Request, true, hires_pixels, true), cache_file(Request, true, std::to_string(hires_pixels) + ".png"), data, Error, Stop) || !decode(data, Out, Error))
				return false;
			image standard;
			std::string ignored;
			if (load_standard(Client, Request, standard, ignored, Stop) && standard.width > 0 && standard.height > 0)
			{
				for (int y = 0; y < Out.height; ++y)
					for (int x = 0; x < Out.width; ++x)
					{
						std::uint8_t *pixel = &Out.rgba[(static_cast<std::size_t>(y) * Out.width + x) * 4];
						if (pixel[3] == 255)
							continue;
						int const sx = std::min(standard.width - 1, x * standard.width / Out.width);
						int const sy = std::min(standard.height - 1, y * standard.height / Out.height);
						std::uint8_t const *source = &standard.rgba[(static_cast<std::size_t>(sy) * standard.width + sx) * 4];
						unsigned const alpha = pixel[3];
						for (int c = 0; c < 3; ++c)
							pixel[c] = static_cast<std::uint8_t>((pixel[c] * alpha + source[c] * (255u - alpha) + 127u) / 255u);
						pixel[3] = 255;
					}
			}
			return true;
		}
		// not covered at all: fall back to the standard imagery
	}

	if (!load_standard(Client, Request, Out, Error, Stop))
		return false;
	if (is_blank(Out))
	{
		Empty = true;
		Out = image{};
		return true;
	}
	while (Out.width > Request.level && Out.width > 1)
		halve(Out);
	return true;
}

// the services cover Poland only (EPSG:2180 extent from their capabilities, plus a margin); no point asking about other places
bool in_service_area(int Ix, int Iy)
{
	double const east = (Ix + 0.5) * editor_orthophoto::tile_size;
	double const north = (Iy + 0.5) * editor_orthophoto::tile_size;
	return north > 100000.0 && north < 900000.0 && east > 100000.0 && east < 900000.0;
}

// homogeneous clip-space vertex with texture coordinates
struct clip_vertex
{
	glm::vec4 position;
	glm::vec2 uv;
};

// Sutherland-Hodgman against a single clip plane; Distance(v) >= 0 is inside
template <typename Distance_> int clip_polygon(clip_vertex const *In, int Count, clip_vertex *Out, Distance_ const &Distance)
{
	int count{0};
	for (int i = 0; i < Count; ++i)
	{
		clip_vertex const &a = In[i];
		clip_vertex const &b = In[(i + 1) % Count];
		float const da = Distance(a.position);
		float const db = Distance(b.position);
		if (da >= 0.0f)
			Out[count++] = a;
		if ((da >= 0.0f) != (db >= 0.0f))
		{
			float const t = da / (da - db);
			Out[count++] = {glm::mix(a.position, b.position, t), glm::mix(a.uv, b.uv, t)};
		}
	}
	return count;
}

// ---------------------------------------------------------------------------------------------
// ground fit and in-scene geometry

constexpr int ground_grid{32};          // ground samples per tile side (8 m) for the draped overlay
constexpr double steepest_ground{0.3};  // minimal normal.y of ground triangles receiving the imagery
constexpr float no_height{-std::numeric_limits<float>::max()};

gfx::geometrybank_handle scene_bank{0, 0}; // created on first use, shared by every tile

std::string slot_name(int Index)
{
	// script-style generated resource: no file behind it, the content is supplied through update_from_memory()
	return "internal_src:editor_orthophoto_" + std::to_string(Index);
}

// clips a polygon in the XZ plane against Point[Axis] <= Limit (Below) or Point[Axis] >= Limit
int clip_xz(glm::dvec3 const *In, int Count, glm::dvec3 *Out, int Axis, double Limit, bool Below)
{
	auto const inside = [&](glm::dvec3 const &Point) { return Below ? Point[Axis] <= Limit : Point[Axis] >= Limit; };
	int count{0};
	for (int i = 0; i < Count; ++i)
	{
		glm::dvec3 const &a = In[i];
		glm::dvec3 const &b = In[(i + 1) % Count];
		bool const ina = inside(a);
		if (ina)
			Out[count++] = a;
		if (ina != inside(b))
			Out[count++] = a + (b - a) * ((Limit - a[Axis]) / (b[Axis] - a[Axis]));
	}
	return count;
}

// height of triangle abc above (X,Z), if the vertical line through it hits the triangle
bool triangle_height(glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C, double X, double Z, double &Out)
{
	double const ux = B.x - A.x, uz = B.z - A.z;
	double const vx = C.x - A.x, vz = C.z - A.z;
	double const den = ux * vz - vx * uz;
	if (std::abs(den) < 1e-9)
		return false;
	double const wx = X - A.x, wz = Z - A.z;
	double const s = (wx * vz - vx * wz) / den;
	double const t = (ux * wz - wx * uz) / den;
	double constexpr epsilon{1e-9};
	if (s < -epsilon || t < -epsilon || s + t > 1.0 + epsilon)
		return false;
	Out = A.y + s * (B.y - A.y) + t * (C.y - A.y);
	return true;
}

// bilinear lookup in a tile heightfield; U runs west->east, V north->south, both 0..1
double sample_heights(std::vector<float> const &Heights, double U, double V)
{
	double const fx = std::clamp(U, 0.0, 1.0) * ground_grid;
	double const fy = std::clamp(V, 0.0, 1.0) * ground_grid;
	int const ix = std::min(static_cast<int>(fx), ground_grid - 1);
	int const iy = std::min(static_cast<int>(fy), ground_grid - 1);
	double const tx = fx - ix, ty = fy - iy;
	auto const at = [&](int X, int Y) { return static_cast<double>(Heights[static_cast<std::size_t>(Y) * (ground_grid + 1) + X]); };
	double const top = at(ix, iy) + (at(ix + 1, iy) - at(ix, iy)) * tx;
	double const bottom = at(ix, iy + 1) + (at(ix + 1, iy + 1) - at(ix, iy + 1)) * tx;
	return top + (bottom - top) * ty;
}

// smooth normal from the heightfield (u grows towards -x, v towards -z)
glm::vec3 heights_normal(std::vector<float> const &Heights, double U, double V)
{
	double constexpr delta{1.0 / ground_grid};
	double const dhdu = (sample_heights(Heights, U + delta, V) - sample_heights(Heights, U - delta, V)) / (2.0 * delta * editor_orthophoto::tile_size);
	double const dhdv = (sample_heights(Heights, U, V + delta) - sample_heights(Heights, U, V - delta)) / (2.0 * delta * editor_orthophoto::tile_size);
	return glm::normalize(glm::vec3(static_cast<float>(dhdu), 1.0f, static_cast<float>(dhdv)));
}
} // namespace

// ---------------------------------------------------------------------------------------------

editor_orthophoto::~editor_orthophoto()
{
	// the scene geometry is left alone: on shutdown the region may already be gone
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_stop = true;
		m_queue.clear();
	}
	m_wakeup.notify_all();
	for (auto &worker : m_workers)
		if (worker.joinable())
			worker.join();
}

bool editor_orthophoto::can_download()
{
#if defined(_WIN32) || defined(EU07_WITH_CURL)
	return true;
#else
	return false;
#endif
}

std::string editor_orthophoto::cache_directory()
{
	auto const path = cache_root().u8string();
	return std::string(path.begin(), path.end());
}

bool editor_orthophoto::owns(gfx::geometry_handle const &Geometry)
{
	return scene_bank.bank != 0 && Geometry.bank == scene_bank.bank;
}

void editor_orthophoto::enabled(bool State)
{
	if (State == m_enabled)
		return;
	m_enabled = State;
	if (m_enabled)
	{
		start_workers();
		return;
	}
	cancel_pending();
	for (auto &entry : m_tiles)
		release_tile(entry.second);
	m_tiles.clear();
	++m_generation; // results still in flight are now stale
	std::lock_guard<std::mutex> lock(m_mutex);
	m_done.clear();
}

void editor_orthophoto::settings(config const &Config)
{
	bool const sourcechanged = (Config.year != m_config.year || Config.hires != m_config.hires);
	bool const shapechanged = (Config.height != m_config.height || Config.drape != m_config.drape || Config.in_scene != m_config.in_scene || Config.lift != m_config.lift);
	m_config = Config;
	m_config.lift = std::clamp(m_config.lift, 0.0f, 50.0f);
	m_config.radius = std::clamp(m_config.radius, 0, max_radius);
	m_config.opacity = std::clamp(m_config.opacity, 0.0f, 1.0f);
	if (shapechanged)
		for (auto &entry : m_tiles)
			entry.second.scene_dirty = true;
	if (!sourcechanged)
		return;
	cancel_pending();
	for (auto &entry : m_tiles)
		release_tile(entry.second);
	m_tiles.clear();
	++m_generation;
}

void editor_orthophoto::refit()
{
	// the current shapes stay until their tile is sampled again, so nothing blinks
	for (auto &entry : m_tiles)
		entry.second.grounded = false;
}

void editor_orthophoto::detach_scene()
{
	for (auto &entry : m_tiles)
	{
		remove_shape(entry.second);
		release_geometry(entry.second);
	}
}

void editor_orthophoto::cancel_pending()
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_queue.clear();
	}
	// jobs already picked up by a worker still deliver their result, which is accepted if the tile is still wanted
	for (auto &entry : m_tiles)
		entry.second.requested = 0;
}

void editor_orthophoto::retry_failed()
{
	for (auto &entry : m_tiles)
	{
		entry.second.failures = 0;
		entry.second.retry_at = 0.0;
	}
}

editor_orthophoto::statistics editor_orthophoto::stats() const
{
	statistics out;
	for (auto const &entry : m_tiles)
	{
		tile const &t = entry.second;
		if (t.level > 0 && !t.empty && t.slot.texture != null_handle)
			++out.resident;
		if (t.requested != 0)
			++out.loading;
		else if (t.failures > 0 && t.level == 0)
			++out.failed;
	}
	return out;
}

int editor_orthophoto::level_for(int Distance, bool Hires)
{
	// coarser imagery further away keeps the memory use of large radii in check
	if (Distance <= 1)
		return Hires ? hires_pixels : standard_pixels;
	if (Distance <= 3)
		return standard_pixels;
	if (Distance <= 6)
		return standard_pixels / 2;
	return standard_pixels / 4;
}

editor_orthophoto::tile_key editor_orthophoto::camera_tile(glm::dvec3 const &Camera) const
{
	double const east = m_config.east - Camera.x;
	double const north = m_config.north + Camera.z;
	return {static_cast<int>(std::floor(east / tile_size)), static_cast<int>(std::floor(north / tile_size))};
}

glm::dvec2 editor_orthophoto::tile_corner(tile_key const &Key) const
{
	// image u runs west->east (towards -x), v north->south (towards -z)
	return {m_config.east - Key.first * tile_size, (Key.second + 1) * tile_size - m_config.north};
}

void editor_orthophoto::start_workers()
{
	if (!m_workers.empty())
		return;
	for (int i = 0; i < worker_count; ++i)
		m_workers.emplace_back(&editor_orthophoto::worker, this);
}

editor_orthophoto::texture_slot editor_orthophoto::acquire_slot()
{
	if (!m_free_slots.empty())
	{
		texture_slot const slot = m_free_slots.back();
		m_free_slots.pop_back();
		return slot;
	}
	texture_slot slot;
	slot.index = m_texture_count++;
	slot.texture = GfxRenderer->Fetch_Texture(slot_name(slot.index), true);
	return slot;
}

void editor_orthophoto::release_tile(tile &Tile)
{
	remove_shape(Tile);
	release_geometry(Tile);
	if (Tile.slot.texture == null_handle)
		return;
	// shrink to a single texel to give the gpu memory back; the slot is reused by the next tile
	std::uint8_t const texel[4] = {0, 0, 0, 0};
	GfxRenderer->Texture(Tile.slot.texture).update_from_memory(1, 1, texel);
	m_free_slots.push_back(Tile.slot);
	Tile.slot = texture_slot{};
}

void editor_orthophoto::schedule(tile_key const &Key, tile &Tile, int Level, std::uint8_t Alpha)
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_queue.erase(std::remove_if(m_queue.begin(), m_queue.end(), [&](job const &Job) { return Job.key == Key; }), m_queue.end());
		m_queue.push_back(job{Key, Level, Alpha, source{m_config.year, m_config.hires}, m_generation.load()});
	}
	Tile.requested = Level;
	Tile.requested_alpha = Alpha;
	m_wakeup.notify_one();
}

std::uint8_t editor_orthophoto::texture_alpha() const
{
	// the overlay applies its opacity when drawing, the scene shapes can only take it from the texture
	return m_config.in_scene ? static_cast<std::uint8_t>(std::lround(m_config.opacity * 255.0f)) : 255;
}

std::size_t editor_orthophoto::apply(result &Result, double Now)
{
	if (Result.generation != m_generation.load())
		return 0;
	auto const lookup = m_tiles.find(Result.key);
	if (lookup == m_tiles.end())
		return 0; // left the area in the meantime; the imagery stays in the disk cache
	tile &t = lookup->second;
	bool const current = (t.requested == Result.level && t.requested_alpha == Result.alpha);
	if (current)
		t.requested = 0;

	if (!Result.ok)
	{
		if (current)
		{
			++t.failures;
			t.retry_at = (t.failures >= 5) ? std::numeric_limits<double>::max() : Now + std::min(120.0, 5.0 * std::pow(2.0, t.failures - 1));
		}
		return 0;
	}

	t.failures = 0;
	t.level = Result.level;
	if ((t.alpha < 255) != (Result.alpha < 255))
		t.scene_dirty = true; // the shape moves between the opaque and the translucent pass
	t.alpha = Result.alpha;
	t.empty = Result.empty;
	if (Result.empty)
	{
		release_tile(t);
		return 0;
	}
	if (t.slot.texture == null_handle)
		t.slot = acquire_slot();
	if (t.slot.texture == null_handle)
	{
		t.level = 0;
		return 0;
	}
	// a shape already in the scene shows the new content right away, it refers to the same texture
	GfxRenderer->Texture(t.slot.texture).update_from_memory(Result.width, Result.height, Result.pixels.data());
	return Result.pixels.size();
}

void editor_orthophoto::sample_ground(tile_key const &Key, tile &Tile)
{
	Tile.grounded = true;
	Tile.scene_dirty = true;
	Tile.heights.clear();
	Tile.surface.clear();

	glm::dvec2 const corner = tile_corner(Key);
	glm::dvec2 const min(corner.x - tile_size, corner.y - tile_size);
	glm::dvec2 const max(corner.x, corner.y);
	std::vector<world_triangle> triangles;
	m_ground(min, max, triangles);

	int constexpr samples{ground_grid + 1};
	double constexpr step{tile_size / ground_grid};
	std::vector<float> heights(static_cast<std::size_t>(samples) * samples, no_height);

	for (auto const &triangle : triangles)
	{
		glm::dvec3 a = triangle[0], b = triangle[1], c = triangle[2];
		glm::dvec3 normal = glm::cross(b - a, c - a);
		if (normal.y < 0.0)
		{
			// keep every triangle facing up, with the winding the editor terrain uses
			std::swap(b, c);
			normal = -normal;
		}
		double const length = glm::length(normal);
		if (length < 1e-9 || normal.y / length < steepest_ground)
			continue; // walls and the like don't receive the imagery

		glm::dvec3 polygon[8] = {a, b, c};
		glm::dvec3 scratch[8];
		int count = clip_xz(polygon, 3, scratch, 0, max.x, true);
		count = clip_xz(scratch, count, polygon, 0, min.x, false);
		count = clip_xz(polygon, count, scratch, 2, max.y, true);
		count = clip_xz(scratch, count, polygon, 2, min.y, false);
		for (int k = 1; k + 1 < count; ++k)
		{
			Tile.surface.push_back(polygon[0]);
			Tile.surface.push_back(polygon[k]);
			Tile.surface.push_back(polygon[k + 1]);
		}

		// heightfield samples under the (whole) triangle, the highest surface wins
		double const xlo = std::min({a.x, b.x, c.x}), xhi = std::max({a.x, b.x, c.x});
		double const zlo = std::min({a.z, b.z, c.z}), zhi = std::max({a.z, b.z, c.z});
		int const i0 = std::clamp(static_cast<int>(std::ceil((corner.x - xhi) / step)), 0, samples - 1);
		int const i1 = std::clamp(static_cast<int>(std::floor((corner.x - xlo) / step)), 0, samples - 1);
		int const j0 = std::clamp(static_cast<int>(std::ceil((corner.y - zhi) / step)), 0, samples - 1);
		int const j1 = std::clamp(static_cast<int>(std::floor((corner.y - zlo) / step)), 0, samples - 1);
		for (int j = j0; j <= j1; ++j)
			for (int i = i0; i <= i1; ++i)
			{
				double y;
				if (triangle_height(a, b, c, corner.x - i * step, corner.y - j * step, y))
				{
					float &sample = heights[static_cast<std::size_t>(j) * samples + i];
					sample = std::max(sample, static_cast<float>(y));
				}
			}
	}

	// holes (places without ground) take the average of their neighbours, spreading inwards
	std::size_t missing = std::count(heights.begin(), heights.end(), no_height);
	if (missing == heights.size())
		return; // no ground at all: the tile stays flat at the layer height
	while (missing > 0)
	{
		std::vector<float> next = heights;
		for (int j = 0; j < samples; ++j)
			for (int i = 0; i < samples; ++i)
			{
				if (heights[static_cast<std::size_t>(j) * samples + i] != no_height)
					continue;
				double sum{0.0};
				int found{0};
				int const offsets[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
				for (auto const &offset : offsets)
				{
					int const x = i + offset[0], y = j + offset[1];
					if (x < 0 || y < 0 || x >= samples || y >= samples)
						continue;
					float const neighbour = heights[static_cast<std::size_t>(y) * samples + x];
					if (neighbour == no_height)
						continue;
					sum += neighbour;
					++found;
				}
				if (found > 0)
				{
					next[static_cast<std::size_t>(j) * samples + i] = static_cast<float>(sum / found);
					--missing;
				}
			}
		heights.swap(next);
	}
	Tile.heights = std::move(heights);
	auto const range = std::minmax_element(Tile.heights.begin(), Tile.heights.end());
	Tile.height_min = *range.first;
	Tile.height_max = *range.second;
}

void editor_orthophoto::remove_shape(tile &Tile)
{
	if (Tile.section == nullptr)
		return;
	if (m_region == simulation::Region)
	{
		auto &shapes = (Tile.cell != nullptr ? Tile.cell->m_shapestranslucent : Tile.section->m_shapes);
		for (auto it = shapes.begin(); it != shapes.end(); ++it)
		{
			auto const &geometry = it->data().geometry;
			if (geometry.bank == Tile.geometry.bank && geometry.chunk == Tile.geometry.chunk)
			{
				shapes.erase(it);
				break;
			}
		}
	}
	Tile.section = nullptr;
	Tile.cell = nullptr;
}

void editor_orthophoto::release_geometry(tile &Tile)
{
	// the chunk stays in the bank and is reused by a tile needing the same capacity
	if (Tile.geometry.chunk != 0)
		m_free_chunks.emplace(Tile.capacity, Tile.geometry);
	Tile.geometry = gfx::geometry_handle{0, 0};
	Tile.capacity = 0;
}

void editor_orthophoto::build_shape(tile_key const &Key, tile &Tile)
{
	remove_shape(Tile);
	Tile.scene_dirty = false;
	if (Tile.level == 0 || Tile.empty || Tile.slot.texture == null_handle || simulation::Region == nullptr)
		return;
	if (Tile.slot.material == null_handle)
		Tile.slot.material = GfxRenderer->Fetch_Material(slot_name(Tile.slot.index), true);

	glm::dvec2 const corner = tile_corner(Key);
	bool const draped = m_config.drape && !Tile.surface.empty();
	auto const uv_of = [&](glm::dvec3 const &Point) { return glm::vec2((corner.x - Point.x) / tile_size, (corner.y - Point.z) / tile_size); };

	std::vector<world_vertex> vertices;
	auto const add = [&](glm::dvec3 const &Point, double Lift) {
		world_vertex vertex;
		vertex.position = Point + glm::dvec3(0.0, Lift, 0.0);
		vertex.texture = uv_of(Point);
		// smooth normals from the heightfield, the clipped triangles alone would look faceted
		vertex.normal = (draped && !Tile.heights.empty()) ? heights_normal(Tile.heights, vertex.texture.x, vertex.texture.y) : glm::vec3(0.0f, 1.0f, 0.0f);
		vertices.push_back(vertex);
	};
	double ylo, yhi;
	if (draped)
	{
		vertices.reserve(Tile.surface.size());
		for (auto const &point : Tile.surface)
			add(point, m_config.lift);
		ylo = Tile.height_min;
		yhi = Tile.height_max + m_config.lift;
	}
	else
	{
		// flat quad at the layer height, triangles facing up like the draped surface
		double const y = m_config.height;
		glm::dvec3 const nw(corner.x, y, corner.y), ne(corner.x - tile_size, y, corner.y);
		glm::dvec3 const sw(corner.x, y, corner.y - tile_size), se(corner.x - tile_size, y, corner.y - tile_size);
		for (auto const &point : {nw, sw, ne, se, ne, sw})
			add(point, 0.0);
		ylo = yhi = y;
	}

	glm::dvec3 const centre(corner.x - tile_size * 0.5, (ylo + yhi) * 0.5, corner.y - tile_size * 0.5);
	scene::basic_section &section = simulation::Region->section(centre);
	section.create_geometry(); // existing section geometry has to be in place before we add ours (idempotent)
	// opaque imagery goes with the section geometry; translucent one has to be drawn in the translucent pass,
	// which only covers the cells. shapes are drawn relative to the centre of their container
	bool const translucent = (Tile.alpha < 255);
	scene::basic_cell *cell = translucent ? &section.cell(centre) : nullptr;
	glm::dvec3 const origin = translucent ? cell->m_area.center : section.m_area.center;

	// chunks are padded to a few fixed sizes (whole triangles), so ones freed by other tiles can be refilled in place
	std::size_t capacity{6};
	while (capacity < vertices.size())
		capacity *= 2;
	gfx::vertex_array gpuvertices;
	gpuvertices.reserve(capacity);
	for (auto const &vertex : vertices)
		gpuvertices.emplace_back(gfx::basic_vertex::convert(vertex, origin));
	world_vertex padding; // degenerate triangles at the origin
	padding.position = origin;
	padding.normal = glm::vec3(0.0f, 1.0f, 0.0f);
	padding.texture = glm::vec2(0.0f);
	gpuvertices.resize(capacity, gfx::basic_vertex::convert(padding, origin));
	gfx::userdata_array userdata;

	if (Tile.geometry.chunk != 0 && Tile.capacity != capacity)
		release_geometry(Tile);
	if (Tile.geometry.chunk == 0)
	{
		auto const reusable = m_free_chunks.find(capacity);
		if (reusable != m_free_chunks.end())
		{
			Tile.geometry = reusable->second;
			Tile.capacity = capacity;
			m_free_chunks.erase(reusable);
		}
	}
	if (Tile.geometry.chunk != 0)
	{
		GfxRenderer->Replace(gpuvertices, userdata, Tile.geometry, GL_TRIANGLES);
	}
	else
	{
		if (scene_bank.bank == 0)
			scene_bank = GfxRenderer->Create_Bank();
		Tile.geometry = GfxRenderer->Insert(gpuvertices, userdata, scene_bank, GL_TRIANGLES);
		Tile.capacity = capacity;
		if (Tile.geometry.chunk == 0)
		{
			Tile.capacity = 0;
			return;
		}
	}

	// the shape gets just two bounding vertices: they give it its area, and leave no triangles behind for
	// code which reads the source vertices of the section shapes (e.g. snapping nodes to the ground)
	std::vector<world_vertex> bounds(2, padding);
	bounds[0].position = glm::dvec3(corner.x - tile_size, ylo, corner.y - tile_size);
	bounds[1].position = glm::dvec3(corner.x, yhi, corner.y);
	scene::shape_node shape;
	shape.make_terrain(Tile.slot.material, std::move(bounds), origin);
	shape.geometry(Tile.geometry);
	// extend the bounds of the containers so the tile isn't culled at its edges
	double const reach = glm::length(glm::dvec3(tile_size * 0.5, (yhi - ylo) * 0.5, tile_size * 0.5));
	section.m_area.radius = std::max(section.m_area.radius, static_cast<float>(glm::length(section.m_area.center - centre) + reach));
	if (cell != nullptr)
	{
		cell->m_shapestranslucent.emplace_back(std::move(shape));
		cell->m_area.radius = std::max(cell->m_area.radius, static_cast<float>(glm::length(cell->m_area.center - centre) + reach));
		cell->m_active = true; // the renderer skips cells which held nothing so far
	}
	else
	{
		section.m_shapes.emplace_back(std::move(shape));
	}

	Tile.section = &section;
	Tile.cell = cell;
	m_region = simulation::Region;
}

void editor_orthophoto::update_scene()
{
	if (m_region != simulation::Region)
	{
		// another scenery was loaded: the sections holding our shapes are gone, forget them without touching
		for (auto &entry : m_tiles)
			entry.second.section = nullptr;
		m_region = simulation::Region;
	}
	if (!m_config.in_scene)
	{
		detach_scene();
		return;
	}
	if (simulation::Region == nullptr)
		return;

	int built{0};
	for (auto &entry : m_tiles)
	{
		tile &t = entry.second;
		if (t.section != nullptr && !t.scene_dirty)
			continue;
		if (t.level == 0 || t.empty)
			continue;
		if (m_config.drape && !t.grounded && m_ground)
			continue; // wait for the ground fit; a shape already in place stays until then
		build_shape(entry.first, t);
		if (++built >= 8)
			break; // the rest next frame
	}
}

void editor_orthophoto::update(glm::dvec3 const &Camera)
{
	double const now = steady_seconds();

	// finished work first, within a per-frame upload budget
	std::size_t uploaded{0};
	while (uploaded < upload_budget)
	{
		result done;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			if (m_done.empty())
				break;
			done = std::move(m_done.front());
			m_done.pop_front();
		}
		if (m_enabled)
			uploaded += apply(done, now);
	}

	if (!m_enabled)
		return;

	tile_key const centre = camera_tile(Camera);
	if (centre != m_camera_tile)
	{
		m_camera_tile = centre;
		std::lock_guard<std::mutex> lock(m_mutex);
		m_focus = centre;
	}
	int const radius = m_config.radius;
	auto const distance = [&](tile_key const &Key) { return std::max(std::abs(Key.first - centre.first), std::abs(Key.second - centre.second)); };

	// opacity of the scene shapes: the tiles are read again with the new alpha once the slider stops moving
	std::uint8_t const alpha = texture_alpha();
	if (alpha != m_alpha_pending)
	{
		m_alpha_pending = alpha;
		m_alpha_changed = now;
	}
	if (m_alpha != m_alpha_pending && now - m_alpha_changed > 0.3)
		m_alpha = m_alpha_pending;

	// tiles which left the area
	bool evicted{false};
	for (auto it = m_tiles.begin(); it != m_tiles.end();)
	{
		if (distance(it->first) > radius)
		{
			release_tile(it->second);
			it = m_tiles.erase(it);
			evicted = true;
		}
		else
			++it;
	}
	if (evicted)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_queue.erase(std::remove_if(m_queue.begin(), m_queue.end(), [&](job const &Job) { return distance(Job.key) > radius; }), m_queue.end());
	}

	// tiles which need (different) imagery
	for (int dy = -radius; dy <= radius; ++dy)
		for (int dx = -radius; dx <= radius; ++dx)
		{
			tile_key const key{centre.first + dx, centre.second + dy};
			tile &t = m_tiles[key];
			if (!in_service_area(key.first, key.second))
				t.empty = true;
			int const level = level_for(std::max(std::abs(dx), std::abs(dy)), m_config.hires);
			if (t.empty || (t.level == level && t.alpha == m_alpha) || (t.requested == level && t.requested_alpha == m_alpha) || now < t.retry_at)
				continue;
			schedule(key, t, level, m_alpha);
		}

	// ground fit, one tile per frame (it walks the scene geometry), nearest first
	if (m_config.drape && m_ground && simulation::Region != nullptr)
	{
		tile *nearest{nullptr};
		tile_key nearestkey{0, 0};
		int nearestdistance{INT_MAX};
		for (auto &entry : m_tiles)
		{
			if (entry.second.grounded || entry.second.empty)
				continue;
			int const d = distance(entry.first);
			if (d < nearestdistance)
			{
				nearestdistance = d;
				nearest = &entry.second;
				nearestkey = entry.first;
			}
		}
		if (nearest != nullptr)
			sample_ground(nearestkey, *nearest);
	}

	update_scene();
}

void editor_orthophoto::worker()
{
	stbi_set_flip_vertically_on_load_thread(0); // the texture loader flips globally; tiles keep the north edge in the first row
	http_client client;
	stop_check const stopping = [this]() { return m_stop.load(); };

	for (;;)
	{
		job current;
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			m_wakeup.wait(lock, [this]() { return m_stop.load() || !m_queue.empty(); });
			if (m_stop)
				return;
			// nearest tile first
			auto best = m_queue.begin();
			long long bestdistance = LLONG_MAX;
			for (auto it = m_queue.begin(); it != m_queue.end(); ++it)
			{
				long long const dx = it->key.first - m_focus.first;
				long long const dy = it->key.second - m_focus.second;
				long long const d = dx * dx + dy * dy;
				if (d < bestdistance)
				{
					bestdistance = d;
					best = it;
				}
			}
			current = *best;
			m_queue.erase(best);
		}

		result out;
		out.key = current.key;
		out.level = current.level;
		out.generation = current.generation;
		out.alpha = current.alpha;

		image picture;
		std::string error;
		tile_request const request{current.key.first, current.key.second, current.level, current.src.year, current.src.hires};
		out.ok = load_tile(client, request, picture, out.empty, error, stopping);
		if (m_stop)
			return;
		if (!out.ok)
			ErrorLog("Editor orthophoto: tile " + std::to_string(current.key.second) + "_" + std::to_string(current.key.first) + " failed: " + error);
		if (out.ok && current.alpha < 255)
			for (std::size_t i = 3; i < picture.rgba.size(); i += 4)
				picture.rgba[i] = static_cast<std::uint8_t>((picture.rgba[i] * current.alpha + 127u) / 255u);
		out.width = picture.width;
		out.height = picture.height;
		out.pixels = std::move(picture.rgba);

		std::lock_guard<std::mutex> lock(m_mutex);
		m_done.push_back(std::move(out));
	}
}

void editor_orthophoto::draw(glm::mat4 const &ViewProjection, glm::dvec3 const &CameraPos, float ScreenWidth, float ScreenHeight) const
{
	if (!m_enabled || m_config.in_scene || m_config.opacity <= 0.0f || ScreenWidth <= 0.0f || ScreenHeight <= 0.0f)
		return;

	ImDrawList *drawlist = ImGui::GetBackgroundDrawList();
	// without vertex offset support in the backend the whole list has to fit 16-bit indices
	std::size_t const vertexbudget = (ImGui::GetIO().BackendFlags & ImGuiBackendFlags_RendererHasVtxOffset) ? vertex_budget * 8 : vertex_budget;
	ImU32 const color = IM_COL32(255, 255, 255, static_cast<int>(m_config.opacity * 255.0f + 0.5f));
	auto const toscreen = [&](glm::vec4 const &Clip) { return ImVec2((Clip.x / Clip.w * 0.5f + 0.5f) * ScreenWidth, (0.5f - Clip.y / Clip.w * 0.5f) * ScreenHeight); };
	// slightly wider than the view so clipped edges never show on screen
	float constexpr guard{1.02f};
	auto const outside = [&](glm::vec4 const *Corners, int Count) {
		auto const all = [&](auto const &Test) {
			for (int i = 0; i < Count; ++i)
				if (!Test(Corners[i]))
					return false;
			return true;
		};
		return all([](glm::vec4 const &v) { return v.w < near_w; }) || all([&](glm::vec4 const &v) { return v.x > v.w * guard; }) ||
		       all([&](glm::vec4 const &v) { return v.x < -v.w * guard; }) || all([&](glm::vec4 const &v) { return v.y > v.w * guard; }) ||
		       all([&](glm::vec4 const &v) { return v.y < -v.w * guard; });
	};
	// clip-space change per metre of height
	glm::vec4 const up = ViewProjection * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);

	for (auto const &entry : m_tiles)
	{
		tile const &t = entry.second;
		if (t.level == 0 || t.empty || t.slot.texture == null_handle)
			continue;
		std::size_t const textureid = GfxRenderer->Texture(t.slot.texture).get_id();
		if (textureid == 0 || textureid == static_cast<std::size_t>(static_cast<std::uint32_t>(-1)))
			continue;

		bool const draped = m_config.drape && !t.heights.empty();
		double const flat = m_config.height;
		double const lift = m_config.lift;
		double const ylo = draped ? t.height_min + lift : flat;
		double const yhi = draped ? t.height_max + lift : flat;

		// world extent of the tile; u runs west->east (-x), v runs north->south (-z), matching the image rows
		glm::dvec2 const corner = tile_corner(entry.first);
		auto const relative = [&](double U, double V) { return glm::vec3(glm::dvec3(corner.x - U * tile_size, 0.0, corner.y - V * tile_size) - CameraPos); };
		// the projection is linear before the divide, so clip positions interpolate exactly: c = c00 + du*u + dv*v + up*height
		glm::vec4 const c00 = ViewProjection * glm::vec4(relative(0.0, 0.0), 1.0f);
		glm::vec4 const du = ViewProjection * glm::vec4(relative(1.0, 0.0), 1.0f) - c00;
		glm::vec4 const dv = ViewProjection * glm::vec4(relative(0.0, 1.0), 1.0f) - c00;
		auto const clip = [&](float U, float V, double Height) { return c00 + du * U + dv * V + up * static_cast<float>(Height); };
		glm::vec4 const box[8] = {clip(0, 0, ylo), clip(1, 0, ylo), clip(1, 1, ylo), clip(0, 1, ylo), clip(0, 0, yhi), clip(1, 0, yhi), clip(1, 1, yhi), clip(0, 1, yhi)};
		if (outside(box, 8))
			continue;

		// ImGui interpolates texture coordinates affinely, so subdivide finer where the camera is close;
		// draped tiles also need enough segments to follow the ground
		double const nearestx = std::clamp(CameraPos.x, corner.x - tile_size, corner.x);
		double const nearestz = std::clamp(CameraPos.z, corner.y - tile_size, corner.y);
		double const nearesty = std::clamp(CameraPos.y, ylo, yhi);
		double const distance = glm::length(glm::dvec3(nearestx, nearesty, nearestz) - CameraPos);
		double const step = std::clamp(distance * 0.2, 4.0, tile_size);
		int segments = std::clamp(static_cast<int>(std::ceil(tile_size / step)), 1, 64);
		if (draped)
			segments = std::max(segments, distance < 300.0 ? ground_grid : distance < 800.0 ? ground_grid / 2 : ground_grid / 4);
		float const segmentsize = 1.0f / static_cast<float>(segments);
		auto const height_at = [&](float U, float V) { return draped ? sample_heights(t.heights, U, V) + lift : flat; };

		drawlist->PushTextureID(reinterpret_cast<ImTextureID>(textureid));
		for (int j = 0; j < segments; ++j)
			for (int i = 0; i < segments; ++i)
			{
				if (static_cast<std::size_t>(drawlist->VtxBuffer.Size) + 16 > vertexbudget)
				{
					drawlist->PopTextureID();
					return;
				}
				float const u0 = i * segmentsize, u1 = (i + 1) * segmentsize;
				float const v0 = j * segmentsize, v1 = (j + 1) * segmentsize;
				clip_vertex quad[4] = {
				    {clip(u0, v0, height_at(u0, v0)), {u0, v0}},
				    {clip(u1, v0, height_at(u1, v0)), {u1, v0}},
				    {clip(u1, v1, height_at(u1, v1)), {u1, v1}},
				    {clip(u0, v1, height_at(u0, v1)), {u0, v1}},
				};
				glm::vec4 const positions[4] = {quad[0].position, quad[1].position, quad[2].position, quad[3].position};
				if (outside(positions, 4))
					continue;

				// clip against the near plane and the (slightly widened) sides of the view
				clip_vertex buffer0[12], buffer1[12];
				int count = clip_polygon(quad, 4, buffer0, [](glm::vec4 const &v) { return v.w - near_w; });
				count = clip_polygon(buffer0, count, buffer1, [&](glm::vec4 const &v) { return v.w * guard - v.x; });
				count = clip_polygon(buffer1, count, buffer0, [&](glm::vec4 const &v) { return v.w * guard + v.x; });
				count = clip_polygon(buffer0, count, buffer1, [&](glm::vec4 const &v) { return v.w * guard - v.y; });
				count = clip_polygon(buffer1, count, buffer0, [&](glm::vec4 const &v) { return v.w * guard + v.y; });
				if (count < 3)
					continue;

				drawlist->PrimReserve((count - 2) * 3, count);
				auto const base = static_cast<ImDrawIdx>(drawlist->_VtxCurrentIdx);
				for (int k = 1; k + 1 < count; ++k)
				{
					drawlist->PrimWriteIdx(base);
					drawlist->PrimWriteIdx(static_cast<ImDrawIdx>(base + k));
					drawlist->PrimWriteIdx(static_cast<ImDrawIdx>(base + k + 1));
				}
				for (int k = 0; k < count; ++k)
					drawlist->PrimWriteVtx(toscreen(buffer0[k].position), ImVec2(buffer0[k].uv.x, buffer0[k].uv.y), color);
			}
		drawlist->PopTextureID();
	}
}
