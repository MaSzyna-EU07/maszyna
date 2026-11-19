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
	ImGui::Text(STR_C("Ice on the wires: %.1f mm"), weather.snow().wire_ice() * 1000.f);
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
}
