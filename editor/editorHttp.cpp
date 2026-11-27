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
// of libcurl only the types and the constants: its functions are looked up at run time (libcurl() below), so the game
// starts on a system without libcurl and only the editor's downloads are off there. no deprecation warnings for the
// options kept for a libcurl older than these headers
#ifndef CURL_DISABLE_DEPRECATION
#define CURL_DISABLE_DEPRECATION
#endif
#include <curl/curl.h>
#include <dlfcn.h>
#include <format>
#include "utilities/Logs.h"
#endif

#include <cstdlib>
#include <filesystem>
#include <string>

namespace editor_http
{

#if defined(_WIN32) || defined(EU07_WITH_CURL)
namespace
{
// the most a response is read into memory: a map or tiles of a service the user typed may be any size
std::size_t constexpr kLargest{64u * 1024u * 1024u};
char const *const kTooLarge{"the response is over 64 MB"};
} // namespace
#endif

struct client::platform
{
#if defined(_WIN32)
	HINTERNET session{nullptr};
#elif defined(EU07_WITH_CURL)
	CURL *curl{nullptr};
#endif
};


#if defined(_WIN32)

namespace
{

DWORD constexpr kStopCheck{50}; // ms between the looks at whether the transfer is to be given up

// what the callback of WinHTTP hands over to the thread waiting for it
struct transfer_state
{
	HANDLE done{::CreateEventW(nullptr, FALSE, FALSE, nullptr)}; // the operation begun finished or failed
	HANDLE closed{::CreateEventW(nullptr, TRUE, FALSE, nullptr)}; // the request is called back no more
	DWORD error{ERROR_SUCCESS}; // of the operation that failed
	DWORD size{0}; // bytes available or read

	transfer_state() = default;
	transfer_state(transfer_state const &) = delete;
	transfer_state &operator=(transfer_state const &) = delete;
	~transfer_state()
	{
		if (done != nullptr)
			::CloseHandle(done);
		if (closed != nullptr)
			::CloseHandle(closed);
	}
};

void CALLBACK on_status(HINTERNET, DWORD_PTR const Context, DWORD const Status, LPVOID const Info, DWORD const Length)
{
	auto *state{reinterpret_cast<transfer_state *>(Context)};
	if (state == nullptr)
		return;
	switch (Status)
	{
	case WINHTTP_CALLBACK_STATUS_SENDREQUEST_COMPLETE:
	case WINHTTP_CALLBACK_STATUS_HEADERS_AVAILABLE:
		break;
	case WINHTTP_CALLBACK_STATUS_DATA_AVAILABLE:
		state->size = *static_cast<DWORD const *>(Info);
		break;
	case WINHTTP_CALLBACK_STATUS_READ_COMPLETE:
		state->size = Length;
		break;
	case WINHTTP_CALLBACK_STATUS_REQUEST_ERROR:
		state->error = static_cast<WINHTTP_ASYNC_RESULT const *>(Info)->dwError;
		break;
	case WINHTTP_CALLBACK_STATUS_HANDLE_CLOSING:
		::SetEvent(state->closed);
		return;
	default:
		return;
	}
	::SetEvent(state->done);
}

// one GET on a connection of its own. the steps of WinHTTP are asynchronous and waited for while the stop is looked at, so a
// transfer is given up at once also while a service keeps the connection waiting: the closing of the editor doesn't wait for
// the timeouts of a service that doesn't answer
class transfer
{
  public:
	transfer(stop_check const &Stop, std::string &Error) :
		m_stop(Stop),
		m_error(Error)
	{
	}
	~transfer()
	{
		close();
	}
	transfer(transfer const &) = delete;
	transfer &operator=(transfer const &) = delete;

	bool open(HINTERNET const Session, std::wstring const &Host, INTERNET_PORT const Port, std::wstring const &Path, bool const Secure)
	{
		if (m_state.done == nullptr || m_state.closed == nullptr)
			return failed(::GetLastError());
		m_connection = ::WinHttpConnect(Session, Host.c_str(), Port, 0);
		if (m_connection == nullptr)
			return failed(::GetLastError());
		m_request = ::WinHttpOpenRequest(m_connection, L"GET", Path.c_str(), nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, Secure ? WINHTTP_FLAG_SECURE : 0);
		if (m_request == nullptr)
			return failed(::GetLastError());
		auto context{reinterpret_cast<DWORD_PTR>(&m_state)};
		if (false == ::WinHttpSetOption(m_request, WINHTTP_OPTION_CONTEXT_VALUE, &context, sizeof(context)))
			return failed(::GetLastError());
		if (::WinHttpSetStatusCallback(m_request, on_status, WINHTTP_CALLBACK_FLAG_ALL_COMPLETIONS | WINHTTP_CALLBACK_FLAG_HANDLES, 0) == WINHTTP_INVALID_STATUS_CALLBACK)
			return failed(::GetLastError());
		m_called_back = true;
		return true;
	}

	bool send()
	{
		return step([this]() { return ::WinHttpSendRequest(m_request, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, reinterpret_cast<DWORD_PTR>(&m_state)); });
	}

	bool receive()
	{
		return step([this]() { return ::WinHttpReceiveResponse(m_request, nullptr); });
	}

	DWORD status() const
	{
		DWORD status{0};
		DWORD size{sizeof(status)};
		::WinHttpQueryHeaders(m_request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &status, &size, WINHTTP_NO_HEADER_INDEX);
		return status;
	}

	// the whole body of the response
	bool read_all(std::vector<std::uint8_t> &Out)
	{
		while (true)
		{
			if (m_stop())
			{
				m_error = "cancelled";
				return false;
			}
			if (false == step([this]() { return ::WinHttpQueryDataAvailable(m_request, nullptr); }))
				return false;
			auto const available{m_state.size};
			if (available == 0)
				return true;
			if (Out.size() + available > kLargest)
			{
				m_error = kTooLarge;
				Out.clear();
				return false;
			}
			auto const offset{Out.size()};
			Out.resize(offset + available);
			auto const read{step([this, &Out, offset, available]() { return ::WinHttpReadData(m_request, Out.data() + offset, available, nullptr); })};
			Out.resize(offset + (read ? m_state.size : 0));
			if (false == read)
				return false;
		}
	}

	void close()
	{
		if (m_request != nullptr)
		{
			::WinHttpCloseHandle(m_request);
			m_request = nullptr;
			// the closing is the last callback: after it the state, and the buffer of a read broken off, are no longer written to
			if (m_called_back)
				::WaitForSingleObject(m_state.closed, INFINITE);
			m_called_back = false;
		}
		if (m_connection != nullptr)
		{
			::WinHttpCloseHandle(m_connection);
			m_connection = nullptr;
		}
	}

  private:
	bool failed(DWORD const Code)
	{
		m_error = "WinHTTP error " + std::to_string(Code);
		return false;
	}

	// begins an operation and waits for it: false when it failed, or a stop was asked for meanwhile
	template <typename Begin> bool step(Begin const &Operation)
	{
		m_state.error = ERROR_SUCCESS;
		if (false == Operation())
			return failed(::GetLastError());
		while (::WaitForSingleObject(m_state.done, kStopCheck) != WAIT_OBJECT_0)
		{
			if (m_stop())
			{
				// closed at once: the operation broken off doesn't write into the buffer any more once this returns
				close();
				m_error = "cancelled";
				return false;
			}
		}
		if (m_state.error != ERROR_SUCCESS)
			return failed(m_state.error);
		return true;
	}

	stop_check const &m_stop;
	std::string &m_error;
	transfer_state m_state;
	HINTERNET m_connection{nullptr};
	HINTERNET m_request{nullptr};
	bool m_called_back{false}; // the closing of the request is called back
};

} // namespace

client::client() :
	m_platform(std::make_unique<platform>())
{
	wchar_t const *agent = L"MaSzyna-editor";
#ifdef WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY
	m_platform->session = ::WinHttpOpen(agent, WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, WINHTTP_FLAG_ASYNC);
#endif
	if (m_platform->session == nullptr) // automatic proxy needs windows 8.1+
		m_platform->session = ::WinHttpOpen(agent, WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, WINHTTP_FLAG_ASYNC);
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

	// the address as the user typed or pasted it, in utf-8
	std::wstring url;
	if (auto const size{::MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, Url.data(), static_cast<int>(Url.size()), nullptr, 0)}; size > 0)
	{
		url.resize(static_cast<std::size_t>(size));
		::MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, Url.data(), static_cast<int>(Url.size()), url.data(), size);
	}
	URL_COMPONENTS parts{};
	parts.dwStructSize = sizeof(parts);
	parts.dwSchemeLength = static_cast<DWORD>(-1);
	parts.dwHostNameLength = static_cast<DWORD>(-1);
	parts.dwUrlPathLength = static_cast<DWORD>(-1);
	parts.dwExtraInfoLength = static_cast<DWORD>(-1);
	if (url.empty() || !::WinHttpCrackUrl(url.c_str(), 0, 0, &parts))
	{
		Error = "invalid url";
		return false;
	}
	std::wstring const host(parts.lpszHostName, parts.dwHostNameLength);
	std::wstring const path = std::wstring(parts.lpszUrlPath, parts.dwUrlPathLength) + std::wstring(parts.lpszExtraInfo, parts.dwExtraInfoLength);

	transfer request{Stop, Error};
	if (false == request.open(m_platform->session, host, parts.nPort, path, parts.nScheme == INTERNET_SCHEME_HTTPS) || false == request.send() || false == request.receive())
		return false;
	auto const status{request.status()};
	if (false == request.read_all(Out))
		return false;
	request.close();

	if (status != 200)
	{
		Error = "HTTP " + std::to_string(status);
		return false;
	}
	return true;
}

#elif defined(EU07_WITH_CURL)

namespace
{
// the file of the certificates https is checked with. a libcurl built elsewhere (as the one once bundled with the game) knows
// only the one of the system it was built on (/etc/ssl/certs/ca-certificates.crt of Debian and Ubuntu), which Fedora, RHEL
// and openSUSE don't have: there every download failed. the file named by the user first, then the ones the distributions
// keep; empty leaves libcurl's own
std::string const &certificates_file()
{
	static std::string const file{[]() {
		std::error_code error;
		for (char const *variable : {"SSL_CERT_FILE", "CURL_CA_BUNDLE"})
		{
			if (char const *value{std::getenv(variable)}; (value != nullptr) && (*value != '\0') && std::filesystem::is_regular_file(value, error))
				return std::string{value};
		}
		for (char const *candidate : {
				 "/etc/ssl/certs/ca-certificates.crt", // Debian, Ubuntu, Arch, SteamOS, Gentoo, NixOS
				 "/etc/pki/tls/certs/ca-bundle.crt", // Fedora, RHEL
				 "/etc/pki/ca-trust/extracted/pem/tls-ca-bundle.pem", // Fedora, RHEL
				 "/etc/ssl/ca-bundle.pem", // openSUSE
				 "/etc/ssl/cert.pem", // Alpine, the BSDs, macOS
			 })
		{
			if (std::filesystem::is_regular_file(candidate, error))
				return std::string{candidate};
		}
		return std::string{};
	}()};
	return file;
}

// the functions of libcurl the client uses, looked up in the system's libcurl. their types are those the headers declare
// them with, so a call through them is the call of the function itself
struct curl_library
{
	decltype(&::curl_global_init) global_init{nullptr};
	decltype(&::curl_easy_init) easy_init{nullptr};
	decltype(&::curl_easy_cleanup) easy_cleanup{nullptr};
	decltype(&::curl_easy_reset) easy_reset{nullptr};
	decltype(&::curl_easy_setopt) easy_setopt{nullptr};
	decltype(&::curl_easy_perform) easy_perform{nullptr};
	decltype(&::curl_easy_getinfo) easy_getinfo{nullptr};
	decltype(&::curl_easy_strerror) easy_strerror{nullptr};
	bool loaded{false}; // all of them found
};

// the system's libcurl: the name of its runtime package (libcurl4 of Debian and Ubuntu, libcurl of Fedora and Arch), that of
// the GnuTLS build of Debian and Ubuntu, then that of its development files. null when there's none
void *open_curl()
{
	for (char const *const name : {"libcurl.so.4", "libcurl-gnutls.so.4", "libcurl.so"})
	{
		if (void *const library{::dlopen(name, RTLD_NOW | RTLD_LOCAL)}; library != nullptr)
			return library;
	}
	return nullptr;
}

// the function Name of the library into Function, false when the library has none
template <typename Function>
bool resolve(void *const Library, char const *const Name, Function &Into)
{
	// dlsym() gives a function's address as void *, which POSIX lets be converted back to the function's type
	Into = reinterpret_cast<Function>(::dlsym(Library, Name));
	if (Into == nullptr)
		WriteLog(std::format("Editor: the libcurl of the system has no {}, the downloads of the editor are off", Name));
	return Into != nullptr;
}

// the system's libcurl, opened and initialised once, when the editor first needs it, and kept for the rest of the run
curl_library load_curl()
{
	curl_library curl;
	void *const library{open_curl()};
	if (library == nullptr)
	{
		WriteLog("Editor: no libcurl (libcurl.so.4) on this system, the downloads of the editor (orthophoto, terrain model, topo map) are off");
		return curl;
	}
	curl.loaded = resolve(library, "curl_global_init", curl.global_init) && resolve(library, "curl_easy_init", curl.easy_init) &&
	              resolve(library, "curl_easy_cleanup", curl.easy_cleanup) && resolve(library, "curl_easy_reset", curl.easy_reset) &&
	              resolve(library, "curl_easy_setopt", curl.easy_setopt) && resolve(library, "curl_easy_perform", curl.easy_perform) &&
	              resolve(library, "curl_easy_getinfo", curl.easy_getinfo) && resolve(library, "curl_easy_strerror", curl.easy_strerror);
	if (false == curl.loaded)
	{
		::dlclose(library);
		return curl_library{};
	}
	curl.global_init(CURL_GLOBAL_DEFAULT);
	return curl;
}

curl_library const &libcurl()
{
	static curl_library const curl{load_curl()};
	return curl;
}
} // namespace

client::client() :
	m_platform(std::make_unique<platform>())
{
	if (auto const &curl{libcurl()}; curl.loaded)
		m_platform->curl = curl.easy_init();
}

client::~client()
{
	// set only with libcurl loaded
	if (m_platform->curl != nullptr)
		libcurl().easy_cleanup(m_platform->curl);
}

bool client::get(std::string const &Url, std::vector<std::uint8_t> &Out, std::string &Error, stop_check const &Stop)
{
	Out.clear();
	auto const &curl{libcurl()};
	if (m_platform->curl == nullptr)
	{
		Error = (curl.loaded ? "curl_easy_init failed" : "no libcurl on this system");
		return false;
	}
	CURL *const handle{m_platform->curl};

	struct sink
	{
		std::vector<std::uint8_t> *out;
		bool over{false};
	} received{&Out};
	auto const write = +[](char *Data, std::size_t Size, std::size_t Count, void *User) -> std::size_t {
		auto *into = static_cast<sink *>(User);
		if (into->out->size() + Size * Count > kLargest)
		{
			// fewer bytes taken than given stops the transfer
			into->over = true;
			return 0;
		}
		into->out->insert(into->out->end(), Data, Data + Size * Count);
		return Size * Count;
	};
	auto const progress = +[](void *User, curl_off_t, curl_off_t, curl_off_t, curl_off_t) -> int { return (*static_cast<stop_check const *>(User))() ? 1 : 0; };

	curl.easy_reset(handle);
	curl.easy_setopt(handle, CURLOPT_URL, Url.c_str());
	curl.easy_setopt(handle, CURLOPT_USERAGENT, "MaSzyna-editor");
	if (auto const &certificates{certificates_file()}; false == certificates.empty())
	{
		curl.easy_setopt(handle, CURLOPT_CAINFO, certificates.c_str());
#if LIBCURL_VERSION_NUM >= 0x073400
		// an https proxy is checked with the same file
		curl.easy_setopt(handle, CURLOPT_PROXY_CAINFO, certificates.c_str());
#endif
	}
	curl.easy_setopt(handle, CURLOPT_FOLLOWLOCATION, 1L);
	curl.easy_setopt(handle, CURLOPT_MAXREDIRS, 10L);
	// http kept for the services the user types; nothing else, also where a redirect leads. the system's libcurl can be older
	// than the headers: one without the options by name (before 7.85.0) refuses them, and takes the bit masks
#if LIBCURL_VERSION_NUM >= 0x075500
	bool const named{(curl.easy_setopt(handle, CURLOPT_PROTOCOLS_STR, "https,http") == CURLE_OK) &&
	                 (curl.easy_setopt(handle, CURLOPT_REDIR_PROTOCOLS_STR, "https,http") == CURLE_OK)};
#else
	bool const named{false};
#endif
	if (false == named)
	{
		curl.easy_setopt(handle, CURLOPT_PROTOCOLS, static_cast<long>(CURLPROTO_HTTPS | CURLPROTO_HTTP));
		curl.easy_setopt(handle, CURLOPT_REDIR_PROTOCOLS, static_cast<long>(CURLPROTO_HTTPS | CURLPROTO_HTTP));
	}
	curl.easy_setopt(handle, CURLOPT_MAXFILESIZE_LARGE, static_cast<curl_off_t>(kLargest));
	curl.easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
	curl.easy_setopt(handle, CURLOPT_CONNECTTIMEOUT, 10L);
	curl.easy_setopt(handle, CURLOPT_LOW_SPEED_LIMIT, 1L);
	curl.easy_setopt(handle, CURLOPT_LOW_SPEED_TIME, 20L);
	curl.easy_setopt(handle, CURLOPT_WRITEFUNCTION, write);
	curl.easy_setopt(handle, CURLOPT_WRITEDATA, &received);
	curl.easy_setopt(handle, CURLOPT_NOPROGRESS, 0L);
	curl.easy_setopt(handle, CURLOPT_XFERINFOFUNCTION, progress);
	curl.easy_setopt(handle, CURLOPT_XFERINFODATA, const_cast<stop_check *>(&Stop));

	CURLcode const code = curl.easy_perform(handle);
	if (code != CURLE_OK)
	{
		Error = (received.over || code == CURLE_FILESIZE_EXCEEDED ? kTooLarge : curl.easy_strerror(code));
		Out.clear();
		return false;
	}
	long status{0};
	curl.easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status);
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
#if defined(_WIN32)
	return true;
#elif defined(EU07_WITH_CURL)
	// libcurl opened here when nothing needed it yet
	return libcurl().loaded;
#else
	return false;
#endif
}

} // namespace editor_http
