/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "environment/liveweather.h"

#include "editor/editorHttp.hpp"

#include <chrono>
#include <vector>


live_weather::~live_weather()
{
	reset();
}

bool live_weather::available()
{
	return editor_http::available();
}

void live_weather::request(glm::dvec2 const &Place)
{
	if (busy() || (false == available()))
		return;
	m_stop = std::make_shared<std::atomic<bool>>(false);
	auto const stop{m_stop};
	m_fetch = std::async(std::launch::async, [Place, stop]() {
		editor_http::client client;
		std::vector<std::uint8_t> body;
		std::string error;
		report result;
		result.place = Place;
		if (false == client.get(url(Place), body, error, [stop]() { return stop->load(); }))
		{
			result.error = error.empty() ? "the fetch failed" : error;
			return result;
		}
		return read(std::string(body.begin(), body.end()), Place);
	});
}

bool live_weather::busy() const
{
	return m_fetch.valid() && (m_fetch.wait_for(std::chrono::seconds(0)) != std::future_status::ready);
}

std::optional<live_weather::report> live_weather::take()
{
	if ((false == m_fetch.valid()) || busy())
		return std::nullopt;
	return m_fetch.get();
}

void live_weather::reset()
{
	if (m_stop)
		m_stop->store(true);
	if (m_fetch.valid())
		m_fetch.wait();
	m_fetch = {};
	m_stop.reset();
}
