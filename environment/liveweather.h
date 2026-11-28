/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the weather as it is now at a place, from Open-Meteo (open-meteo.com: free, no key, the models of the European weather services),
// for weather.mode live: fetched in the background, given as a weather report (METAR) the weather of the simulation reads the way it
// reads any other (parse_metar()), and the depth of the snow lying there

#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <glm/glm.hpp>

class live_weather
{
  public:
	struct report
	{
		std::string metar; // the conditions now, as a weather report
		std::optional<float> snow_depth; // m
		glm::dvec2 place{0.0}; // latitude, longitude the report is of
		std::string error; // empty when the fetch went well
	};

	live_weather() = default;
	~live_weather();
	live_weather(live_weather const &) = delete;
	live_weather &operator=(live_weather const &) = delete;

	// whether this build can fetch anything
	static bool available();
	// whether the weather of the scenery loaded goes live: weather.mode live, the scenery laid on the map, a build which fetches. the
	// clock and the date of the simulation are the ones of the system then (the weather is the one of now)
	static bool active();
	// starts a fetch of the weather now at Place (latitude, longitude), unless one goes on
	void request(glm::dvec2 const &Place);
	bool busy() const;
	// the report of a finished fetch, once
	std::optional<report> take();
	// gives a fetch going on up and forgets it
	void reset();

	// the address the weather of Place is asked at
	static std::string url(glm::dvec2 const &Place);
	// a report out of the answer of Open-Meteo (its "current" block); the error set when the answer doesn't hold it
	static report read(std::string const &Answer, glm::dvec2 const &Place);
	// the weather code of the WMO Open-Meteo gives as a present weather group of a METAR (-RA, +SHSN, FZDZ, TSRA, FG ...); empty
	// for none
	static std::string present_weather(int const Code);

  private:
	// the joining of the thread of a finished fetch, before the next one or when the fetch is given up
	void join();

	std::future<report> m_fetch;
	std::thread m_thread; // of the fetch going on or finished and not joined yet
	std::shared_ptr<std::atomic<bool>> m_stop;
};
