#include "stdafx.h"
#include "widgets/time.h"

#include "environment/climate.h"
#include "environment/weather.h"
#include "simulation/simulationenvironment.h"
#include "simulation/simulationtime.h"
#include "utilities/Globals.h"

#include <array>
#include <format>
#include <string>

namespace
{

constexpr std::array<int, 12> month_lengths{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

bool leap_year(int const Year)
{
	return (Year % 4 == 0 && Year % 100 != 0) || (Year % 400 == 0);
}

int days_in(int const Month, int const Year)
{
	return month_lengths[static_cast<std::size_t>(Month)] + ((Month == 1 && leap_year(Year)) ? 1 : 0);
}

// the day of the year of a day (1..) of a month (0..11)
int year_day(int const Day, int const Month, int const Year)
{
	auto yearday{Day};
	for (int i = 0; i < Month; ++i)
		yearday += days_in(i, Year);
	return yearday;
}

std::string clock_text(int const Minute)
{
	return std::format("{:02}:{:02}", Minute / 60, Minute % 60);
}

// the station the climate of the scenery is taken from
climate::station const &scenery_station(world_weather const &Weather)
{
	auto const location{Weather.location()};
	return location ? climate::stations().nearest(location->x, location->y) : climate::default_station();
}

char const *weather_mode_name()
{
	switch (Global.WeatherMode)
	{
	case global_settings::weather_mode::manual:
		return STR_C("set in eu07.ini (weather.manual)");
	case global_settings::weather_mode::automatic:
		return STR_C("of the climate of the place (weather.mode auto)");
	case global_settings::weather_mode::preset:
		return STR_C("of a preset of the climate (weather.mode preset)");
	case global_settings::weather_mode::live:
		return STR_C("now at the vehicle driven, Open-Meteo (weather.mode live)");
	default:
		return STR_C("of the scenery");
	}
}

} // namespace

ui::time_panel::time_panel() : ui_panel(STR_C("Time and environment"), false)
{
	size.x = 520;
}

void ui::time_panel::render_contents()
{
	// the live weather keeps the clock and the date of the system (weather.live.locktime)
	if (Global.ClockLocked)
		ImGui::TextDisabled("%s", STR_C("Date and time: the ones of the system, for the live weather (weather.live.locktime)"));
	else
		render_time();
	ImGui::Separator();
	render_weather();
	ImGui::Separator();
	render_now();

	if (ImGui::Button(STR_C("Apply")))
	{
		auto const year{static_cast<int>(simulation::Time.data().wYear)};
		if (false == Global.ClockLocked)
			m_relay.post(user_command::setdatetime, static_cast<double>(year_day(day, month, year)), static_cast<double>(minute), 1, 0);
		if (weather_edited || snow_edited || kind > 0)
			apply_weather();
	}
}

void ui::time_panel::render_time()
{
	auto const clock{clock_text(minute)};
	ImGui::SliderInt(STR_C("Time"), &minute, 0, 1439, clock.c_str(), ImGuiSliderFlags_AlwaysClamp);

	std::array<char const *, 12> const months{STR_C("January"), STR_C("February"), STR_C("March"),     STR_C("April"),   STR_C("May"),      STR_C("June"),
	                                          STR_C("July"),    STR_C("August"),   STR_C("September"), STR_C("October"), STR_C("November"), STR_C("December")};
	ImGui::Combo(STR_C("Month"), &month, months.data(), static_cast<int>(months.size()));
	auto const length{days_in(month, static_cast<int>(simulation::Time.data().wYear))};
	day = std::clamp(day, 1, length);
	ImGui::SliderInt(STR_C("Day"), &day, 1, length, "%d", ImGuiSliderFlags_AlwaysClamp);
}

void ui::time_panel::render_weather()
{
	auto const &weather{simulation::Environment.weather()};
	ImGui::Text(STR_C("Weather %s, climate of %s"), weather_mode_name(), scenery_station(weather).name.c_str());
	if (false == weather.live_status().empty())
		ImGui::TextWrapped("%s", weather.live_status().c_str());

	std::array<char const *, static_cast<std::size_t>(climate::preset::count) + 2> kinds{};
	kinds[0] = STR_C("set by hand");
	kinds[1] = STR_C("random, as often as each comes in the month");
	for (int i = 0; i < static_cast<int>(climate::preset::count); ++i)
		kinds[static_cast<std::size_t>(i) + 2] = climate::preset_name(static_cast<climate::preset>(i));
	ImGui::Combo(STR_C("Kind of weather"), &kind, kinds.data(), static_cast<int>(kinds.size()));
	if (kind > 0)
	{
		ImGui::TextDisabled("%s", STR_C("Apply draws a day of this kind for the month of the date"));
		return;
	}

	auto edited{false};
	edited |= ImGui::SliderFloat(STR_C("Wind from, deg"), &wind_direction, 0.f, 360.f, "%.0f");
	edited |= ImGui::SliderFloat(STR_C("Wind, m/s"), &wind_speed, 0.f, 30.f, "%.1f");
	edited |= ImGui::SliderFloat(STR_C("Gusts, m/s"), &wind_gust, 0.f, 45.f, "%.1f");
	edited |= ImGui::DragFloatRange2(STR_C("Temperature of the day, C"), &temperature_min, &temperature_max, 0.1f, -35.f, 45.f, "%.1f", "%.1f", ImGuiSliderFlags_AlwaysClamp);
	edited |= ImGui::SliderFloat(STR_C("Dew point, C"), &dew_point, -40.f, 30.f, "%.1f");
	edited |= ImGui::SliderFloat(STR_C("Cloud cover"), &cover, 0.f, 1.f, "%.2f");
	std::array<char const *, 4> const falls{STR_C("by the temperature and the moisture"), STR_C("none"), STR_C("rain"), STR_C("snow")};
	edited |= ImGui::Combo(STR_C("Precipitation"), &precipitation, falls.data(), static_cast<int>(falls.size()));
	edited |= ImGui::SliderFloat(STR_C("Intensity"), &precipitation_intensity, 0.f, 1.f, "%.2f");
	edited |= ImGui::SliderFloat(STR_C("Visibility, m"), &visibility, 50.f, 50000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	weather_edited |= edited;
	snow_edited |= ImGui::SliderFloat(STR_C("Snow lying, cm"), &snow_depth, 0.f, 150.f, "%.0f");
}

void ui::time_panel::render_now() const
{
	auto const &weather{simulation::Environment.weather()};
	std::array<char const *, 5> const falls{STR_C("nothing"), STR_C("rain"), STR_C("freezing rain"), STR_C("wet snow"), STR_C("snow")};
	ImGui::Text(STR_C("Now: %.1f C, humidity %.0f%%, wet bulb %.1f C, falling: %s"), Global.AirTemperature, weather.humidity() * 100.f, weather.wet_bulb(),
	            falls[static_cast<std::size_t>(weather.falling())]);
	ImGui::Text(STR_C("Wind %.1f m/s, snow lying %.0f cm, textures of the %s"), glm::length(weather.surface_wind()), weather.snow().depth() * 100.f, Global.Season.c_str());
}

void ui::time_panel::apply_weather()
{
	auto &weather{simulation::Environment.weather()};
	auto conditions{weather.target()};
	if (kind > 0)
	{
		auto const &station{scenery_station(weather)};
		auto const seed{static_cast<std::uint32_t>(Random(1, 1000000))};
		auto const preset{kind == 1 ? climate::pick(station, month, static_cast<float>(seed % 9973u) / 9973.f) : static_cast<climate::preset>(kind - 2)};
		conditions = climate::make(station, month, preset, seed);
		conditions.location = weather.target().location;
	}
	else if (weather_edited)
	{
		conditions.wind_described = true;
		conditions.wind_direction = wind_direction;
		conditions.wind_speed = wind_speed;
		conditions.wind_gust = std::max(wind_gust, wind_speed);
		conditions.temperature_min = std::min(temperature_min, temperature_max);
		conditions.temperature_max = std::max(temperature_min, temperature_max);
		conditions.temperature = 0.5f * (temperature_min + temperature_max);
		conditions.dew_point = std::min(dew_point, *conditions.temperature_max);
		conditions.clouds = clouds_from_overcast(cover);
		conditions.precipitation = static_cast<precipitation_type>(precipitation);
		conditions.precipitation_intensity = precipitation_intensity;
		conditions.visibility = visibility;
	}
	if (snow_edited)
		conditions.snow_depth = snow_depth / 100.f;
	weather.apply(conditions, 0.f);
	read_weather();
}

void ui::time_panel::read_weather()
{
	auto const &weather{simulation::Environment.weather()};
	auto const &conditions{weather.target()};
	auto const wind{weather.surface_wind()};
	wind_direction = conditions.wind_direction;
	wind_speed = conditions.wind_described ? conditions.wind_speed : glm::length(wind);
	wind_gust = conditions.wind_described ? conditions.wind_gust : wind_speed;
	temperature_min = conditions.temperature_min.value_or(Global.AirTemperature);
	temperature_max = conditions.temperature_max.value_or(Global.AirTemperature);
	dew_point = weather.dew_point();
	cover = std::clamp(conditions.overcast(), 0.f, 1.f);
	precipitation = static_cast<int>(conditions.precipitation);
	precipitation_intensity = conditions.precipitation_intensity;
	visibility = conditions.visibility;
	snow_depth = weather.snow().depth() * 100.f;
	weather_edited = false;
	snow_edited = false;
}

void ui::time_panel::open()
{
	auto const &data = simulation::Time.data();
	minute = data.wHour * 60 + data.wMinute;
	month = std::clamp(static_cast<int>(data.wMonth), 1, 12) - 1;
	day = static_cast<int>(data.wDay);
	read_weather();
	is_open = true;
}
