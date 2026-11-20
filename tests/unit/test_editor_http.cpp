#include "stdafx.h"
#include "doctest.h"
#include "editor/editorHttp.hpp"

#include <asio.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
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
		// a connection of its own wakes the server waiting for one; the one of the client it reads from ends with the client
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
			// the request is read and left without an answer, until the other side closes the connection
			std::array<char, 4096> request{};
			while (!error)
				socket.read_some(asio::buffer(request), error);
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

} // namespace

TEST_CASE("editor http: a request a service doesn't answer is given up as soon as a stop is asked for") {
	if (false == editor_http::available())
		return; // a build with no http client
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
	if (false == editor_http::available())
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
	if (false == editor_http::available())
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
