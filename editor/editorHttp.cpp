/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorHttp.hpp"

#if defined(_WIN32)
#include <winhttp.h>
#elif defined(EU07_WITH_CURL)
#include <curl/curl.h>
#endif

#include <mutex>

namespace editor_http
{

struct client::platform
{
#if defined(_WIN32)
	HINTERNET session{nullptr};
#elif defined(EU07_WITH_CURL)
	CURL *curl{nullptr};
#endif
};


#if defined(_WIN32)

client::client() :
	m_platform(std::make_unique<platform>())
{
	wchar_t const *agent = L"MaSzyna-editor";
#ifdef WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY
	m_platform->session = ::WinHttpOpen(agent, WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
#endif
	if (m_platform->session == nullptr) // automatic proxy needs windows 8.1+
		m_platform->session = ::WinHttpOpen(agent, WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	if (m_platform->session != nullptr)
		::WinHttpSetTimeouts(m_platform->session, 10000, 10000, 15000, 20000);
}

client::~client()
{
	if (m_platform->session != nullptr)
		::WinHttpCloseHandle(m_platform->session);
}

bool client::get(std::string const &Url, std::vector<std::uint8_t> &Out, std::string &Error, stop_check const &Stop)
{
	Out.clear();
	if (m_platform->session == nullptr)
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

	HINTERNET connection = ::WinHttpConnect(m_platform->session, host.c_str(), parts.nPort, 0);
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

client::client() :
	m_platform(std::make_unique<platform>())
{
	static std::once_flag initialized;
	std::call_once(initialized, []() { ::curl_global_init(CURL_GLOBAL_DEFAULT); });
	m_platform->curl = ::curl_easy_init();
}

client::~client()
{
	if (m_platform->curl != nullptr)
		::curl_easy_cleanup(m_platform->curl);
}

bool client::get(std::string const &Url, std::vector<std::uint8_t> &Out, std::string &Error, stop_check const &Stop)
{
	Out.clear();
	if (m_platform->curl == nullptr)
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

	::curl_easy_reset(m_platform->curl);
	::curl_easy_setopt(m_platform->curl, CURLOPT_URL, Url.c_str());
	::curl_easy_setopt(m_platform->curl, CURLOPT_USERAGENT, "MaSzyna-editor");
	::curl_easy_setopt(m_platform->curl, CURLOPT_FOLLOWLOCATION, 1L);
	::curl_easy_setopt(m_platform->curl, CURLOPT_NOSIGNAL, 1L);
	::curl_easy_setopt(m_platform->curl, CURLOPT_CONNECTTIMEOUT, 10L);
	::curl_easy_setopt(m_platform->curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
	::curl_easy_setopt(m_platform->curl, CURLOPT_LOW_SPEED_TIME, 20L);
	::curl_easy_setopt(m_platform->curl, CURLOPT_WRITEFUNCTION, write);
	::curl_easy_setopt(m_platform->curl, CURLOPT_WRITEDATA, &Out);
	::curl_easy_setopt(m_platform->curl, CURLOPT_NOPROGRESS, 0L);
	::curl_easy_setopt(m_platform->curl, CURLOPT_XFERINFOFUNCTION, progress);
	::curl_easy_setopt(m_platform->curl, CURLOPT_XFERINFODATA, const_cast<stop_check *>(&Stop));

	CURLcode const code = ::curl_easy_perform(m_platform->curl);
	if (code != CURLE_OK)
	{
		Error = ::curl_easy_strerror(code);
		return false;
	}
	long status{0};
	::curl_easy_getinfo(m_platform->curl, CURLINFO_RESPONSE_CODE, &status);
	if (status != 200)
	{
		Error = "HTTP " + std::to_string(status);
		return false;
	}
	return true;
}

#else

client::client() = default;
client::~client() = default;

bool client::get(std::string const &, std::vector<std::uint8_t> &Out, std::string &Error, stop_check const &)
{
	Out.clear();
	Error = "no http client in this build";
	return false;
}

#endif

bool available()
{
#if defined(_WIN32) || defined(EU07_WITH_CURL)
	return true;
#else
	return false;
#endif
}

} // namespace editor_http
