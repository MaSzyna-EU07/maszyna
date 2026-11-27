#include "stdafx.h"
#include "doctest.h"
#include "editor/editorHttp.hpp"

#include <asio.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

// a service on this machine. with no reply it takes the connection and the request and never answers, as geoportal.gov.pl
// does now and then
class local_server {
  public:
	explicit local_server(std::string Reply = {}) :
		m_reply(std::move(Reply)) {
		m_acceptor.open(asio::ip::tcp::v4());
		m_acceptor.bind({asio::ip::address_v4::loopback(), 0});
		m_acceptor.listen();
		m_port = m_acceptor.local_endpoint().port();
		m_thread = std::jthread{[this]() { serve(); }};
	}
	~local_server() {
		m_done = true;
		// a connection of its own wakes the server waiting for one
		asio::ip::tcp::socket wake{m_context};
		asio::error_code ignored;
		wake.connect({asio::ip::address_v4::loopback(), m_port}, ignored);
		wake.close(ignored);
		m_thread.join();
	}
	local_server(local_server const &) = delete;
	local_server &operator=(local_server const &) = delete;

	std::string url() const {
		return std::format("http://127.0.0.1:{}/wcs?REQUEST=GetCapabilities", m_port);
	}

  private:
	void serve() {
		while (false == m_done) {
			asio::ip::tcp::socket socket{m_context};
			asio::error_code error;
			m_acceptor.accept(socket, error);
			if (error)
				return;
			if (false == m_reply.empty()) {
				answer(socket);
				continue;
			}
			// the request is left without an answer, the connection open, until the server goes
			while (false == m_done)
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	// the reply once the whole request is read, then the connection closed
	void answer(asio::ip::tcp::socket &Socket) const {
		asio::error_code error;
		asio::streambuf request;
		asio::read_until(Socket, request, "\r\n\r\n", error);
		if (!error)
			asio::write(Socket, asio::buffer(m_reply), error);
		Socket.shutdown(asio::ip::tcp::socket::shutdown_both, error);
	}

	std::string const m_reply;
	asio::io_context m_context;
	asio::ip::tcp::acceptor m_acceptor{m_context};
	unsigned short m_port{0};
	std::atomic<bool> m_done{false};
	std::jthread m_thread;
};

// the requests to the local server would go to a proxy named in the environment, which can't reach it
bool proxied() {
	for (char const *variable : {"http_proxy", "HTTP_PROXY", "all_proxy", "ALL_PROXY"}) {
		if (char const *value{std::getenv(variable)}; value != nullptr && *value != '\0')
			return true;
	}
	return false;
}

} // namespace

TEST_CASE("editor http: a build with an http client has it at run time") {
#if defined(_WIN32) || defined(EU07_WITH_CURL)
	// WinHTTP comes with Windows. libcurl isn't linked, it's loaded when first needed: a build made with its headers runs
	// on a system that has it, so the tests below don't quietly skip a libcurl that failed to load
	CHECK(editor_http::available());
#else
	CHECK_FALSE(editor_http::available());
	editor_http::client client;
	std::vector<std::uint8_t> data;
	std::string error;
	CHECK_FALSE(client.get("http://127.0.0.1/", data, error, []() { return false; }));
	CHECK_FALSE(error.empty());
#endif
}

TEST_CASE("editor http: a request a service doesn't answer is given up as soon as a stop is asked for") {
	if (false == editor_http::available() || proxied())
		return; // a build with no http client, or a proxy in the way
	local_server server;
	auto const start{std::chrono::steady_clock::now()};
	auto const elapsed = [start]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(); };
	editor_http::client client;
	std::vector<std::uint8_t> data;
	std::string error;
	// the stop comes while the client waits for the answer, long before any of its timeouts
	auto const got{client.get(server.url(), data, error, [&elapsed]() { return elapsed() > 0.3; })};
	CHECK_FALSE(got);
	CHECK_FALSE(error.empty());
	// the receive timeout is 20 s; the transfer ends within a moment of the stop (libcurl looks at it about once a second)
	CHECK(elapsed() < 5.0);
}

TEST_CASE("editor http: the whole body of a response is read") {
	if (false == editor_http::available() || proxied())
		return;
	std::string body(100000, '\0');
	for (std::size_t i = 0; i < body.size(); ++i)
		body[i] = static_cast<char>('a' + i % 26);
	local_server server{std::format("HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.size(), body)};
	editor_http::client client;
	std::vector<std::uint8_t> data;
	std::string error;
	CHECK(client.get(server.url(), data, error, []() { return false; }));
	CHECK(error.empty());
	CHECK(std::string(data.begin(), data.end()) == body);
}

TEST_CASE("editor http: the body of an error is kept with the status in the error") {
	if (false == editor_http::available() || proxied())
		return;
	// the terrain model service tells a place outside of its coverage in the body of an error
	std::string const body{"<ServiceException>outside requested coverage</ServiceException>"};
	local_server server{std::format("HTTP/1.1 400 Bad Request\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.size(), body)};
	editor_http::client client;
	std::vector<std::uint8_t> data;
	std::string error;
	CHECK_FALSE(client.get(server.url(), data, error, []() { return false; }));
	CHECK(error == "HTTP 400");
	CHECK(std::string(data.begin(), data.end()) == body);
}
