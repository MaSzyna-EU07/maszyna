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
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace audio
{

// receiver of an internet radio stream (plain http, mp3 payload).
// owns a worker thread which maintains the connection, decodes received data
// and stores the result as 16-bit mono samples for the owner to collect.
// the worker doesn't touch anything outside of the object, and the owner is never made to wait for the network
class radio_stream
{

	friend class radio_session;

  public:
	// types
	enum class state
	{
		idle, // no stream requested
		connecting, // establishing connection with the server
		receiving, // audio data is coming in
		waiting, // connection was lost or refused, waiting to make another attempt
		failed // stream can't be played, no further attempts will be made
	};
	using message = std::pair<std::string, bool>; // text, is error
	// constructors
	radio_stream() = default;
	radio_stream(radio_stream const &) = delete;
	radio_stream &operator=(radio_stream const &) = delete;
	// destructor
	~radio_stream();
	// methods
	// starts reception of specified stream, dropping current one if there's any
	void open(std::string const &Url);
	// terminates reception of the current stream
	void close();
	// returns address of currently requested stream, or empty string
	std::string const &url() const
	{
		return m_url;
	}
	state status() const
	{
		return m_state.load();
	}
	// returns number of decoded samples waiting to be collected
	std::size_t available() const;
	// returns sample rate of decoded audio, or 0 if nothing was decoded yet
	unsigned int rate() const
	{
		return m_rate.load();
	}
	// moves up to specified number of decoded samples to provided buffer. returns: number of samples moved
	std::size_t read(std::int16_t *Output, std::size_t const Count);
	// retrieves next diagnostic message posted by the worker. returns: false if there's no message waiting
	bool fetch_message(message &Message);

  private:
	// types
	struct worker
	{
		std::jthread thread;
		std::shared_ptr<std::atomic<bool>> finished;
	};
	// methods
	void run(std::stop_token Stoptoken, std::string Url, std::uint32_t const Generation, std::shared_ptr<std::atomic<bool>> Finished);
	// stores decoded samples, discarding oldest data if there's no room left. data from outdated workers is ignored
	void store(std::int16_t const *Samples, std::size_t const Count, unsigned int const Rate, std::uint32_t const Generation);
	void status(state const State, std::uint32_t const Generation);
	void post(std::string Text, bool const Error = false);
	// disposes of retired workers which are done with their cleanup
	void reap();
	// members
	std::string m_url;
	worker m_worker;
	std::vector<worker> m_retired; // workers which were told to stop but may still be busy closing their connection
	std::atomic<state> m_state{state::idle};
	std::atomic<unsigned int> m_rate{0};
	mutable std::mutex m_mutex; // guards the sample buffer and the generation counter
	std::uint32_t m_generation{0}; // identifies the worker allowed to deliver data
	std::vector<std::int16_t> m_samples; // ring buffer
	std::size_t m_samplesstart{0};
	std::size_t m_samplescount{0};
	std::mutex m_messagesmutex;
	std::deque<message> m_messages;
};

} // namespace audio
