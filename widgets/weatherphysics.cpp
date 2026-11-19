/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "widgets/weatherphysics.h"

#include "environment/weather.h"
#include "simulation/simulation.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"

ui::weatherphysics_panel::weatherphysics_panel() : ui_panel(STR_C("Weather physics"), false)
{
	size.x = 480;
}

void ui::weatherphysics_panel::render_contents()
{
	if (ImGui::CollapsingHeader(STR_C("Settings (physics.weather in eu07.ini)"), ImGuiTreeNodeFlags_DefaultOpen))
		render_settings();
	if (ImGui::CollapsingHeader(STR_C("Rails in the open"), ImGuiTreeNodeFlags_DefaultOpen))
		render_rails();
	if (ImGui::CollapsingHeader(STR_C("Vehicle driven"), ImGuiTreeNodeFlags_DefaultOpen))
		render_vehicle();
}

void ui::weatherphysics_panel::render_settings()
{
	auto &settings{Global.physics_weather};
	for (auto const &parameter : weather_physics::bool_parameters)
		ImGui::Checkbox(STR_C(parameter.label), &(settings.*parameter.member));
	ImGui::Separator();
	for (auto const &parameter : weather_physics::float_parameters)
	{
		ImGui::SliderFloat(STR_C(parameter.label), &(settings.*parameter.member), parameter.minimum, parameter.maximum, parameter.format, ImGuiSliderFlags_AlwaysClamp);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", std::string{parameter.key}.c_str());
	}
	if (ImGui::Button(STR_C("Defaults")))
		settings = weather_physics::settings{};
	ImGui::SameLine();
	ImGui::TextDisabled("%s", STR_C("kept in eu07.ini, not by this window"));
}

void ui::weatherphysics_panel::render_rails()
{
	auto &weather{simulation::Environment.weather()};
	auto &rails{weather.rails().state()};
	ImGui::SliderFloat(STR_C("Water"), &rails.water, 0.f, 1.f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
	ImGui::SliderFloat(STR_C("Dirt of a dry spell"), &rails.dirt, 0.f, 1.f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
	ImGui::SliderFloat(STR_C("Ice"), &rails.ice, 0.f, 1.f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
	ImGui::SliderFloat(STR_C("Leaves"), &rails.leaves, 0.f, 1.f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
	// the snow and the rails go by the same clock (world_weather::update_snow())
	ImGui::SliderFloat(STR_C("Time faster"), &weather.snow().time_scale, 1.f, 600.f, "%.0fx", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
	auto const adhesion{weather_physics::adhesion_factor(Global.physics_weather, rails, 0.0, -1.0)};
	ImGui::Text(STR_C("Adhesion of a rail no wheels went over: %.0f%% of the dry one"), adhesion * 100.0);
	ImGui::Text(STR_C("The ai drivers and the roads go by: %.0f%%"), Global.FrictionWeatherFactor * 100.f);
	auto const &snow{weather.snow()};
	using icekind = snow_cover::ice_kind;
	ImGui::Text(STR_C("Ice on the wires: %.1f mm"), snow.wire_ice() * 1000.f);
	ImGui::Text(
	    STR_C("glaze %.1f, hard rime %.1f, soft rime %.1f, wet snow %.1f mm"), snow.wire_ice(icekind::glaze) * 1000.f, snow.wire_ice(icekind::rime_hard) * 1000.f,
	    snow.wire_ice(icekind::rime_soft) * 1000.f, snow.wire_ice(icekind::wet_snow) * 1000.f);
	// a span of 65 m square across the wind as it blows now
	auto const wind{simulation::Environment.wind()};
	auto const across{std::hypot(wind.x, wind.z)};
	auto const density{Global.physics_weather.air_density ? weather_physics::air_density(Global.AirTemperature) : 1.0};
	auto const sway{Global.physics_weather.enabled && Global.physics_weather.wire_wind ? weather_physics::wire_sway(Global.physics_weather, 65.0, across, weather.snow().wire_ice(), density) : 0.0};
	ImGui::Text(STR_C("Wind %.1f m/s blows a span of 65 m across it %.2f m aside"), across, sway);
}

void ui::weatherphysics_panel::render_vehicle()
{
	auto const *train{simulation::Train};
	auto const *vehicle{train != nullptr ? train->Dynamic() : nullptr};
	if (vehicle == nullptr)
	{
		ImGui::TextDisabled("%s", STR_C("no vehicle driven"));
		return;
	}
	auto const &mover{*vehicle->MoverParameters};
	auto const &weather{mover.RunningWeather};
	auto const dry{mover.RunningTrack.friction_dry};
	ImGui::Text("%s", vehicle->name().c_str());
	if (dry > 0.0)
		ImGui::Text(STR_C("Adhesion of the rails under it: %.0f%% of the dry one"), mover.RunningTrack.friction / dry * 100.0);
	ImGui::Text(STR_C("Rain %.2f, snow around the running gear %.2f, %.1f deg C"), weather.rain, weather.snow, weather.temperature);
	ImGui::Text(STR_C("Wind %.1f m/s from the front, %.1f m/s across, air %.0f%%"), -weather.wind_along, weather.wind_across, weather.air_density * 100.0);
	ImGui::Text(STR_C("Drag of the weather: %.2f kN"), mover.WeatherDragForce() * 0.001);
	if (mover.Hamulec != nullptr)
		ImGui::Text(STR_C("Brakes: water %.2f, ice %.2f, friction %.0f%%"), mover.BrakeWeather.water, mover.BrakeWeather.ice, mover.Hamulec->weather_factor() * 100.0);
	ImGui::Text(STR_C("Breaks of the contact of the pantographs: %d"), vehicle->contact_breaks);
	if (vehicle->pants != nullptr)
		for (int index = 0; index < vehicle->iAnimType[ANIM_PANTS]; ++index)
			if (auto const *pantograph{vehicle->pants[index].fParamPants}; (pantograph != nullptr) && (pantograph->arc > 0.f))
				ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.f, 1.f), STR_C("Arc at the pantograph %d: %.0f%%"), index + 1, pantograph->arc * 100.f);
}
