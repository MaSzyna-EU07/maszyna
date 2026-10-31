/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "audio/radiostream.h"

#ifdef DBG_NEW
#undef new
#endif

#define ASIO_DISABLE_VISIBILITY
#include <asio.hpp>

#define DR_MP3_IMPLEMENTATION
#define DR_MP3_NO_STDIO
#include "ref/dr_libs/dr_mp3.h"

namespace audio
{

namespace
{

auto const EU07_RADIOSTREAM_BUFFERLENGTH{8}; // seconds of decoded audio kept for the owner
auto const EU07_RADIOSTREAM_CONNECTTIMEOUT{std::chrono::seconds{10}};
auto const EU07_RADIOSTREAM_STALLTIMEOUT{std::chrono::seconds{10}}; // max time without any incoming data
auto const EU07_RADIOSTREAM_MAXREDIRECTS{5};
std::size_t const EU07_RADIOSTREAM_MAXHEADERSIZE{32 * 1024};
std::size_t const EU07_RADIOSTREAM_MAXPLAYLISTSIZE{64 * 1024};
// the decoder needs to see the beginning of the next frame to stay in sync, so it's only fed when there's more data than a frame can take.
// we start with the size safe for the largest frames, then scale it down to the frames of the stream,
// otherwise audio of low bitrate streams would be delivered in pieces up to a second long
std::size_t const EU07_RADIOSTREAM_DECODETHRESHOLD{4096};
std::size_t const EU07_RADIOSTREAM_MINDECODETHRESHOLD{512};
// amount of data without a single valid frame after which the stream is deemed not to be mp3
std::size_t const EU07_RADIOSTREAM_MAXJUNKSIZE{512 * 1024};

std::string lowercase(std::string Text)
{
	std::transform(std::begin(Text), std::end(Text), std::begin(Text), [](unsigned char const c) { return static_cast<char>(std::tolower(c)); });
	return Text;
}

std::string trim(std::string const &Text)
{
	auto const first{Text.find_first_not_of(" \t\r\n")};
	if (first == std::string::npos)
	{
		return {};
	}
	auto const last{Text.find_last_not_of(" \t\r\n")};
	return Text.substr(first, last - first + 1);
}

bool starts_with_nocase(std::string const &Text, std::string const &Prefix)
{
	return Text.size() >= Prefix.size() && lowercase(Text.substr(0, Prefix.size())) == Prefix;
}

struct stream_address
{
	std::string host;
	std::string port{"80"};
	std::string target{"/"};

	// host with optional port, as expected by http host header
	std::string authority() const
	{
		auto const name{host.find(':') != std::string::npos ? "[" + host + "]" : host};
		return port == "80" ? name : name + ":" + port;
	}
};

// splits provided address into components. returns: empty string on success, or description of the problem
std::string parse_address(std::string const &Url, stream_address &Address)
{
	auto url{trim(Url)};
	if (starts_with_nocase(url, "https://"))
	{
		return "encrypted (https) streams aren't supported";
	}
	if (starts_with_nocase(url, "http://"))
	{
		url.erase(0, 7);
	}
	else if (url.find("://") != std::string::npos)
	{
		return "unsupported protocol";
	}

	Address = stream_address{};
	auto const hostend{url.find_first_of("/?")};
	auto authority{url.substr(0, hostend)};
	if (hostend != std::string::npos)
	{
		Address.target = url.substr(hostend);
		if (Address.target.front() == '?')
		{
			Address.target.insert(0, "/");
		}
	}
	// credentials aren't supported, but shouldn't prevent the connection
	auto const credentialsend{authority.rfind('@')};
	if (credentialsend != std::string::npos)
	{
		authority.erase(0, credentialsend + 1);
	}
	if (false == authority.empty() && authority.front() == '[')
	{
		// ipv6 literal
		auto const literalend{authority.find(']')};
		if (literalend == std::string::npos)
		{
			return "malformed address";
		}
		Address.host = authority.substr(1, literalend - 1);
		if (literalend + 2 < authority.size() && authority[literalend + 1] == ':')
		{
			Address.port = authority.substr(literalend + 2);
		}
	}
	else
	{
		auto const portstart{authority.rfind(':')};
		Address.host = authority.substr(0, portstart);
		if (portstart != std::string::npos && portstart + 1 < authority.size())
		{
			Address.port = authority.substr(portstart + 1);
		}
	}
	if (Address.host.empty())
	{
		return "malformed address";
	}
	return {};
}

// combines address received in a redirect with the address of the redirecting resource
std::string resolve_redirect(std::string const &Location, stream_address const &Origin)
{
	if (Location.find("://") != std::string::npos)
	{
		return Location;
	}
	if (Location.rfind("//", 0) == 0)
	{
		return "http:" + Location;
	}
	auto const root{"http://" + Origin.authority()};
	if (false == Location.empty() && Location.front() == '/')
	{
		return root + Location;
	}
	auto directory{Origin.target.substr(0, Origin.target.find('?'))};
	directory.erase(directory.rfind('/') + 1);
	return root + directory + Location;
}

// extracts address of the first stream listed in provided m3u or pls playlist. returns: the address, or empty string
std::string parse_playlist(std::string const &Playlist)
{
	std::istringstream lines{Playlist};
	std::string line;
	while (std::getline(lines, line))
	{
		line = trim(line);
		if (line.empty() || line.front() == '#' || line.front() == '[')
		{
			continue;
		}
		// pls entries have form of FileN=address, m3u ones are plain addresses
		if (starts_with_nocase(line, "file"))
		{
			auto const separator{line.find('=')};
			if (separator != std::string::npos)
			{
				line = trim(line.substr(separator + 1));
			}
		}
		if (starts_with_nocase(line, "http://") || starts_with_nocase(line, "https://"))
		{
			return line;
		}
	}
	return {};
}

struct http_response
{
	int status{0};
	std::map<std::string, std::string> headers; // names are stored in lower case
	std::string body; // part of the content which arrived together with the header

	std::string header(std::string const &Name) const
	{
		auto const lookup{headers.find(Name)};
		return lookup != std::end(headers) ? lookup->second : std::string{};
	}
};

// parses provided response header. returns: true on success
bool parse_response(std::string const &Header, http_response &Response)
{
	std::istringstream lines{Header};
	std::string line;
	if (false == static_cast<bool>(std::getline(lines, line)))
	{
		return false;
	}
	// status line, either regular 'HTTP/1.x 200 OK' or 'ICY 200 OK' sent by shoutcast servers
	std::istringstream statusline{line};
	std::string protocol;
	statusline >> protocol >> Response.status;
	if (statusline.fail() || (false == starts_with_nocase(protocol, "http/") && lowercase(protocol) != "icy"))
	{
		return false;
	}
	while (std::getline(lines, line))
	{
		auto const separator{line.find(':')};
		if (separator == std::string::npos)
		{
			continue;
		}
		Response.headers[lowercase(trim(line.substr(0, separator)))] = trim(line.substr(separator + 1));
	}
	return true;
}

// removes http chunked transfer framing from received data, if there's any
class content_decoder
{

  public:
	explicit content_decoder(bool const Chunked) : m_chunked{Chunked} {}
	// appends payload of provided data to the output. returns: false if the data is malformed
	bool feed(std::uint8_t const *Data, std::size_t const Size, std::vector<std::uint8_t> &Output)
	{
		if (false == m_chunked)
		{
			Output.insert(std::end(Output), Data, Data + Size);
			return true;
		}
		std::size_t index{0};
		while (index < Size)
		{
			switch (m_stage)
			{
			case stage::size:
			{
				auto const character{static_cast<char>(Data[index++])};
				if (character == '\n')
				{
					// chunk size is a hex number, optionally followed by extensions we don't care about
					char *numberend{nullptr};
					m_remaining = std::strtoul(m_line.c_str(), &numberend, 16);
					if (numberend == m_line.c_str())
					{
						return false;
					}
					m_line.clear();
					m_stage = (m_remaining > 0 ? stage::payload : stage::done);
				}
				else if (character != '\r')
				{
					m_line += character;
					if (m_line.size() > 256)
					{
						return false;
					}
				}
				break;
			}
			case stage::payload:
			{
				auto const count{std::min<std::size_t>(m_remaining, Size - index)};
				Output.insert(std::end(Output), Data + index, Data + index + count);
				index += count;
				m_remaining -= count;
				if (m_remaining == 0)
				{
					m_stage = stage::separator;
				}
				break;
			}
			case stage::separator:
			{
				// payload of each chunk is followed by a line break
				if (Data[index++] == '\n')
				{
					m_stage = stage::size;
				}
				break;
			}
			case stage::done:
			{
				return true;
			}
			}
		}
		return true;
	}
	bool done() const
	{
		return m_stage == stage::done;
	}

  private:
	enum class stage
	{
		size,
		payload,
		separator,
		done
	};
	bool m_chunked;
	stage m_stage{stage::size};
	std::string m_line;
	unsigned long m_remaining{0};
};

} // namespace

// single connection attempt of the radio stream worker
class radio_session
{

  public:
	enum class result
	{
		stopped, // the owner lost interest in the stream
		failed, // connection didn't work out or was lost, it's worth to try again
		unsupported // there's nothing we can do with this stream
	};
	radio_session(radio_stream &Owner, std::stop_token Stoptoken, std::uint32_t const Generation) : m_owner{Owner}, m_stoptoken{std::move(Stoptoken)}, m_generation{Generation}, m_resolver{m_io}, m_socket{m_io} {}
	~radio_session()
	{
		asio::error_code ignored;
		m_socket.close(ignored);
	}
	// connects with specified stream and feeds the owner with decoded audio until the connection ends
	result play(std::string Url);
	// returns true if the session delivered any audio
	bool delivered() const
	{
		return m_delivered;
	}

  private:
	// runs pending network operation until specified flag is set by its completion handler. returns: false if the operation was aborted
	bool wait(bool const &Done, std::chrono::seconds const Timeout);
	bool connect(stream_address const &Address);
	bool send(std::string const &Data);
	// reads some data from the socket. returns: number of received bytes, or 0 if the connection is done for
	std::size_t receive(std::uint8_t *Data, std::size_t const Size);
	bool receive_response(http_response &Response);
	// retrieves the playlist and replaces provided address with the one found in it. returns: false if it wasn't possible
	bool receive_playlist(http_response const &Response, std::string &Url, result &Failure);
	result receive_audio(http_response const &Response);
	result fail(std::string const &Reason, result const Result = result::failed);

	radio_stream &m_owner;
	std::stop_token m_stoptoken;
	std::uint32_t m_generation;
	asio::io_context m_io;
	asio::ip::tcp::resolver m_resolver;
	asio::ip::tcp::socket m_socket;
	std::string m_error; // description of the most recent network problem
	bool m_delivered{false};
};

bool radio_session::wait(bool const &Done, std::chrono::seconds const Timeout)
{
	auto const deadline{std::chrono::steady_clock::now() + Timeout};
	while (false == Done)
	{
		auto const timedout{std::chrono::steady_clock::now() >= deadline};
		if (timedout || m_stoptoken.stop_requested())
		{
			m_error = (timedout ? "operation timed out" : "");
			// abort the operation, then let its completion handler run as it refers to data of our caller
			asio::error_code ignored;
			m_resolver.cancel();
			m_socket.close(ignored);
			m_io.restart();
			m_io.run();
			return false;
		}
		if (m_io.stopped())
		{
			m_io.restart();
		}
		m_io.run_one_for(std::chrono::milliseconds{100});
	}
	return true;
}

bool radio_session::connect(stream_address const &Address)
{
	asio::error_code error;
	asio::ip::tcp::resolver::results_type endpoints;
	bool done{false};

	m_resolver.async_resolve(Address.host, Address.port, [&](asio::error_code const &Error, asio::ip::tcp::resolver::results_type Results) {
		error = Error;
		endpoints = std::move(Results);
		done = true;
	});
	if (false == wait(done, EU07_RADIOSTREAM_CONNECTTIMEOUT))
	{
		return false;
	}
	if (error)
	{
		m_error = "can't find the server (" + error.message() + ")";
		return false;
	}

	done = false;
	asio::async_connect(m_socket, endpoints, [&](asio::error_code const &Error, asio::ip::tcp::endpoint const &) {
		error = Error;
		done = true;
	});
	if (false == wait(done, EU07_RADIOSTREAM_CONNECTTIMEOUT))
	{
		return false;
	}
	if (error)
	{
		m_error = "can't connect with the server (" + error.message() + ")";
		return false;
	}
	return true;
}

bool radio_session::send(std::string const &Data)
{
	asio::error_code error;
	bool done{false};

	asio::async_write(m_socket, asio::buffer(Data), [&](asio::error_code const &Error, std::size_t) {
		error = Error;
		done = true;
	});
	if (false == wait(done, EU07_RADIOSTREAM_CONNECTTIMEOUT))
	{
		return false;
	}
	if (error)
	{
		m_error = error.message();
		return false;
	}
	return true;
}

std::size_t radio_session::receive(std::uint8_t *Data, std::size_t const Size)
{
	asio::error_code error;
	std::size_t received{0};
	bool done{false};

	m_socket.async_read_some(asio::buffer(Data, Size), [&](asio::error_code const &Error, std::size_t const Count) {
		error = Error;
		received = Count;
		done = true;
	});
	if (false == wait(done, EU07_RADIOSTREAM_STALLTIMEOUT))
	{
		return 0;
	}
	if (error && received == 0)
	{
		m_error = (error == asio::error::eof ? "connection closed by the server" : error.message());
	}
	return received;
}

bool radio_session::receive_response(http_response &Response)
{
	std::string data;
	std::array<std::uint8_t, 2048> chunk;
	while (true)
	{
		// some servers end lines with a bare line feed, so we look for both variants
		auto headerend{data.find("\r\n\r\n")};
		auto headerendsize{std::size_t{4}};
		if (headerend == std::string::npos)
		{
			headerend = data.find("\n\n");
			headerendsize = 2;
		}
		if (headerend != std::string::npos)
		{
			Response = http_response{};
			if (false == parse_response(data.substr(0, headerend), Response))
			{
				m_error = "unrecognized reply from the server";
				return false;
			}
			Response.body = data.substr(headerend + headerendsize);
			return true;
		}
		if (data.size() > EU07_RADIOSTREAM_MAXHEADERSIZE)
		{
			m_error = "unrecognized reply from the server";
			return false;
		}
		auto const received{receive(chunk.data(), chunk.size())};
		if (received == 0)
		{
			return false;
		}
		data.append(reinterpret_cast<char const *>(chunk.data()), received);
	}
}

radio_session::result radio_session::fail(std::string const &Reason, result const Result)
{
	if (m_stoptoken.stop_requested())
	{
		return result::stopped;
	}
	if (false == Reason.empty())
	{
		m_owner.post("radio stream: " + Reason, true);
	}
	return Result;
}

radio_session::result radio_session::play(std::string Url)
{
	for (auto hop{0}; hop <= EU07_RADIOSTREAM_MAXREDIRECTS; ++hop)
	{
		stream_address address;
		auto const addresserror{parse_address(Url, address)};
		if (false == addresserror.empty())
		{
			return fail(addresserror + " \"" + Url + "\"", result::unsupported);
		}

		m_owner.status(radio_stream::state::connecting, m_generation);
		m_owner.post("radio stream: connecting with \"" + Url + "\"");
		{
			// each hop needs a fresh connection
			asio::error_code ignored;
			m_socket.close(ignored);
		}
		if (false == connect(address))
		{
			return fail(m_error);
		}
		// NOTE: we don't ask for the in-band metadata, so the content is pure audio
		auto const request{"GET " + address.target + " HTTP/1.1\r\nHost: " + address.authority() + "\r\nUser-Agent: MaSzyna\r\nAccept: */*\r\nConnection: close\r\n\r\n"};
		http_response response;
		if (false == send(request) || false == receive_response(response))
		{
			return fail(m_error);
		}

		if (response.status >= 300 && response.status < 400)
		{
			auto const location{response.header("location")};
			if (location.empty())
			{
				return fail("server sent a redirect without the target");
			}
			Url = resolve_redirect(location, address);
			continue;
		}
		if (response.status != 200)
		{
			return fail("server replied with status " + std::to_string(response.status));
		}

		auto const contenttype{lowercase(response.header("content-type"))};
		auto const path{lowercase(address.target.substr(0, address.target.find('?')))};
		auto const isplaylist{contenttype.find("mpegurl") != std::string::npos || contenttype.find("scpls") != std::string::npos || path.ends_with(".m3u") || path.ends_with(".pls")};
		if (isplaylist)
		{
			auto failure{result::failed};
			if (false == receive_playlist(response, Url, failure))
			{
				return failure;
			}
			continue;
		}
		for (auto const *format : {"aac", "ogg", "opus", "flac"})
		{
			if (contenttype.find(format) != std::string::npos)
			{
				return fail("unsupported audio format \"" + contenttype + "\", only mp3 streams can be played", result::unsupported);
			}
		}

		auto const name{response.header("icy-name")};
		auto const bitrate{response.header("icy-br")};
		m_owner.post("radio stream: connected" + (name.empty() ? "" : ", station \"" + name + "\"") + (bitrate.empty() ? "" : ", " + bitrate + " kbps"));
		return receive_audio(response);
	}
	return fail("too many redirects");
}

bool radio_session::receive_playlist(http_response const &Response, std::string &Url, result &Failure)
{
	content_decoder content{lowercase(Response.header("transfer-encoding")).find("chunked") != std::string::npos};
	std::vector<std::uint8_t> playlist;
	if (false == content.feed(reinterpret_cast<std::uint8_t const *>(Response.body.data()), Response.body.size(), playlist))
	{
		Failure = fail("malformed reply from the server");
		return false;
	}
	std::array<std::uint8_t, 2048> chunk;
	while (false == content.done() && playlist.size() < EU07_RADIOSTREAM_MAXPLAYLISTSIZE)
	{
		// the playlist is complete when the server closes the connection
		auto const received{receive(chunk.data(), chunk.size())};
		if (received == 0)
		{
			break;
		}
		if (false == content.feed(chunk.data(), received, playlist))
		{
			Failure = fail("malformed reply from the server");
		return false;
		}
	}
	if (m_stoptoken.stop_requested())
	{
		Failure = result::stopped;
		return false;
	}
	auto const address{parse_playlist(std::string{std::begin(playlist), std::end(playlist)})};
	if (address.empty())
	{
		Failure = fail("playlist \"" + Url + "\" doesn't contain any stream address", result::unsupported);
		return false;
	}
	Url = address;
	return true;
}

radio_session::result radio_session::receive_audio(http_response const &Response)
{
	content_decoder content{lowercase(Response.header("transfer-encoding")).find("chunked") != std::string::npos};
	std::vector<std::uint8_t> input;
	input.reserve(4 * EU07_RADIOSTREAM_DECODETHRESHOLD);
	if (false == content.feed(reinterpret_cast<std::uint8_t const *>(Response.body.data()), Response.body.size(), input))
	{
		return fail("malformed reply from the server");
	}

	auto decoder{std::make_unique<drmp3dec>()};
	::drmp3dec_init(decoder.get());
	std::vector<std::int16_t> frame(DRMP3_MAX_SAMPLES_PER_FRAME);
	std::array<std::uint8_t, 4096> chunk;
	std::size_t junksize{0};
	auto threshold{EU07_RADIOSTREAM_DECODETHRESHOLD};

	m_owner.status(radio_stream::state::receiving, m_generation);

	while (true)
	{
		// decode what we have...
		std::size_t position{0};
		while (input.size() - position >= threshold)
		{
			drmp3dec_frame_info info{};
			auto const samplecount{::drmp3dec_decode_frame(decoder.get(), input.data() + position, static_cast<int>(input.size() - position), frame.data(), &info)};
			if (samplecount > 0)
			{
				if (info.channels > 1)
				{
					// mix the channels down, the sound is going to be emitted from a single point in the cab
					for (auto index{0}; index < samplecount; ++index)
					{
						auto sum{0};
						for (auto channel{0}; channel < info.channels; ++channel)
						{
							sum += frame[index * info.channels + channel];
						}
						frame[index] = static_cast<std::int16_t>(sum / info.channels);
					}
				}
				m_owner.store(frame.data(), static_cast<std::size_t>(samplecount), static_cast<unsigned int>(info.sample_rate), m_generation);
				m_delivered = true;
				junksize = 0;
				threshold = std::clamp(static_cast<std::size_t>(info.frame_bytes) * 3, EU07_RADIOSTREAM_MINDECODETHRESHOLD, EU07_RADIOSTREAM_DECODETHRESHOLD);
			}
			else if (info.frame_bytes > 0)
			{
				// data which isn't part of any frame. the decoder is looking for the stream again, which takes more data
				junksize += static_cast<std::size_t>(info.frame_bytes);
				threshold = EU07_RADIOSTREAM_DECODETHRESHOLD;
			}
			else
			{
				// the decoder wants more data
				break;
			}
			position += static_cast<std::size_t>(info.frame_bytes);
		}
		input.erase(std::begin(input), std::begin(input) + static_cast<std::ptrdiff_t>(position));
		if (junksize > EU07_RADIOSTREAM_MAXJUNKSIZE || input.size() > EU07_RADIOSTREAM_MAXJUNKSIZE)
		{
			return fail(m_delivered ? "lost track of audio data in the stream" : "no mp3 audio found in the stream, only mp3 streams can be played", m_delivered ? result::failed : result::unsupported);
		}
		// ...then get some more
		if (content.done())
		{
			return fail("stream ended");
		}
		auto const received{receive(chunk.data(), chunk.size())};
		if (received == 0)
		{
			return fail(m_error);
		}
		if (false == content.feed(chunk.data(), received, input))
		{
			return fail("malformed data from the server");
		}
	}
}

radio_stream::~radio_stream()
{
	close();
	// NOTE: destructors of retired workers wait for their threads to finish
}

// starts reception of specified stream, dropping current one if there's any
void radio_stream::open(std::string const &Url)
{
	close();
	if (Url.empty())
	{
		return;
	}
	m_url = Url;
	std::uint32_t generation;
	{
		std::scoped_lock lock{m_mutex};
		generation = m_generation;
		m_state = state::connecting;
	}
	m_worker.finished = std::make_shared<std::atomic<bool>>(false);
	m_worker.thread = std::jthread{[this](std::stop_token Stoptoken, std::string Address, std::uint32_t const Workergeneration, std::shared_ptr<std::atomic<bool>> Finished) { run(std::move(Stoptoken), std::move(Address), Workergeneration, std::move(Finished)); },
	                               Url, generation, m_worker.finished};
}

// terminates reception of the current stream
void radio_stream::close()
{
	{
		std::scoped_lock lock{m_mutex};
		// from now on the current worker can't deliver anything
		++m_generation;
		m_samplesstart = 0;
		m_samplescount = 0;
		m_state = state::idle;
		m_rate = 0;
	}
	m_url.clear();
	if (m_worker.thread.joinable())
	{
		// the worker may need a moment to shut down its connection. we don't wait for it here, as we're likely called from the main thread
		m_worker.thread.request_stop();
		m_retired.emplace_back(std::move(m_worker));
		m_worker = worker{};
	}
	reap();
}

// disposes of retired workers which are done with their cleanup
void radio_stream::reap()
{
	std::erase_if(m_retired, [](worker const &Worker) { return Worker.finished->load(); });
}

// returns number of decoded samples waiting to be collected
std::size_t radio_stream::available() const
{
	std::scoped_lock lock{m_mutex};
	return m_samplescount;
}

// moves up to specified number of decoded samples to provided buffer. returns: number of samples moved
std::size_t radio_stream::read(std::int16_t *Output, std::size_t const Count)
{
	std::scoped_lock lock{m_mutex};
	auto const count{std::min(Count, m_samplescount)};
	for (std::size_t index{0}; index < count; ++index)
	{
		Output[index] = m_samples[(m_samplesstart + index) % m_samples.size()];
	}
	if (count > 0)
	{
		m_samplesstart = (m_samplesstart + count) % m_samples.size();
		m_samplescount -= count;
	}
	return count;
}

// retrieves next diagnostic message posted by the worker. returns: false if there's no message waiting
bool radio_stream::fetch_message(message &Message)
{
	std::scoped_lock lock{m_messagesmutex};
	if (m_messages.empty())
	{
		return false;
	}
	Message = std::move(m_messages.front());
	m_messages.pop_front();
	return true;
}

void radio_stream::run(std::stop_token Stoptoken, std::string Url, std::uint32_t const Generation, std::shared_ptr<std::atomic<bool>> Finished)
{
	auto failures{0};
	while (false == Stoptoken.stop_requested())
	{
		auto result{radio_session::result::failed};
		auto delivered{false};
		{
			radio_session session{*this, Stoptoken, Generation};
			result = session.play(Url);
			delivered = session.delivered();
		}
		if (result == radio_session::result::stopped || Stoptoken.stop_requested())
		{
			break;
		}
		if (result == radio_session::result::unsupported)
		{
			status(state::failed, Generation);
			break;
		}
		// try again after a pause, which gets longer if the server keeps refusing to cooperate
		failures = (delivered ? 0 : failures + 1);
		auto const pause{std::chrono::seconds{std::min(15, 1 << std::min(failures, 4))}};
		status(state::waiting, Generation);
		post("radio stream: next connection attempt in " + std::to_string(pause.count()) + " s");
		auto const deadline{std::chrono::steady_clock::now() + pause};
		while (false == Stoptoken.stop_requested() && std::chrono::steady_clock::now() < deadline)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds{50});
		}
	}
	Finished->store(true);
}

// stores decoded samples, discarding oldest data if there's no room left. data from outdated workers is ignored
void radio_stream::store(std::int16_t const *Samples, std::size_t const Count, unsigned int const Rate, std::uint32_t const Generation)
{
	std::scoped_lock lock{m_mutex};
	if (Generation != m_generation || Rate == 0)
	{
		return;
	}
	if (Rate != m_rate)
	{
		// (re)initialize the buffer for the new format
		m_samples.assign(static_cast<std::size_t>(Rate) * EU07_RADIOSTREAM_BUFFERLENGTH, 0);
		m_samplesstart = 0;
		m_samplescount = 0;
		m_rate = Rate;
	}
	auto const capacity{m_samples.size()};
	for (std::size_t index{0}; index < Count; ++index)
	{
		m_samples[(m_samplesstart + m_samplescount) % capacity] = Samples[index];
		if (m_samplescount < capacity)
		{
			++m_samplescount;
		}
		else
		{
			// the buffer is full, the oldest sample was just overwritten
			m_samplesstart = (m_samplesstart + 1) % capacity;
		}
	}
}

void radio_stream::status(state const State, std::uint32_t const Generation)
{
	std::scoped_lock lock{m_mutex};
	if (Generation == m_generation)
	{
		m_state = State;
	}
}

void radio_stream::post(std::string Text, bool const Error)
{
	std::scoped_lock lock{m_messagesmutex};
	m_messages.emplace_back(std::move(Text), Error);
	// if nobody collects the messages there's no point in hoarding them
	while (m_messages.size() > 64)
	{
		m_messages.pop_front();
	}
}

} // namespace audio
