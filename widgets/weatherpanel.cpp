/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "widgets/weatherpanel.h"

#include "environment/weather.h"

bool ui::weather_controls(world_weather &Weather)
{
	static char input[512]{};
	static float transition{0.f};

	auto conditions{Weather.target()};
	ImGui::TextWrapped("%s: %s", Weather.described() ? "described" : "made from the overcast", to_metar(conditions).c_str());
	ImGui::InputText("##weatherinput", input, sizeof(input));
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("a weather report (METAR), e.g. 24012KT 4000 -RA BR SCT008 BKN015 OVC040 08/06,\nor entries of the weather section, e.g. visibility 800 groundfog 60 300 clouds stratus ovc 150 400");
	ImGui::SameLine();
	auto changed{false};
	if (ImGui::Button("apply##weatherapply"))
		changed = Weather.apply_text(input, transition);
	ImGui::SliderFloat("change over, s##weathertransition", &transition, 0.f, 600.f, "%.0f");

	auto edited{false};
	edited |= ImGui::SliderFloat("visibility, m##weathervisibility", &conditions.visibility, 50.f, 100000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	edited |= ImGui::SliderFloat("haze height, m##weatherhaze", &conditions.haze_height, 50.f, 5000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	edited |= ImGui::SliderFloat("ground fog, m##weathergroundfog", &conditions.groundfog_thickness, 0.f, 500.f, "%.0f");
	edited |= ImGui::SliderFloat("visibility in it, m##weathergroundfogvisibility", &conditions.groundfog_visibility, 20.f, 5000.f, "%.0f", ImGuiSliderFlags_Logarithmic);
	edited |= ImGui::SliderFloat("patchiness##weathernoise", &conditions.fog_noise, 0.f, 1.f);
	char const *precipitations[]{"by the temperature", "none", "rain", "snow"};
	auto precipitation{static_cast<int>(conditions.precipitation)};
	if (ImGui::Combo("precipitation##weatherprecipitation", &precipitation, precipitations, IM_ARRAYSIZE(precipitations)))
	{
		conditions.precipitation = static_cast<precipitation_type>(precipitation);
		edited = true;
	}
	edited |= ImGui::SliderFloat("intensity##weatherintensity", &conditions.precipitation_intensity, 0.f, 1.f);
	edited |= ImGui::SliderFloat("wind from, deg##weatherwinddirection", &conditions.cloud_wind_direction, 0.f, 360.f, "%.0f");
	edited |= ImGui::SliderFloat("wind at the clouds, m/s##weatherwindspeed", &conditions.cloud_wind_speed, 0.f, 60.f, "%.1f");

	std::vector<char const *> genera;
	for (int i = 0; i < static_cast<int>(cloud_type::count); ++i)
		genera.push_back(cloud_genus_info(static_cast<cloud_type>(i)).name);
	int removed{-1};
	for (std::size_t i = 0; i < conditions.clouds.size(); ++i)
	{
		auto &layer{conditions.clouds[i]};
		ImGui::PushID(static_cast<int>(i));
		ImGui::Separator();
		auto type{static_cast<int>(layer.type)};
		if (ImGui::Combo("genus", &type, genera.data(), static_cast<int>(genera.size())))
		{
			// a new genus gets its typical depth
			layer.type = static_cast<cloud_type>(type);
			layer.top = layer.base + cloud_genus_info(layer.type).thickness;
			edited = true;
		}
		auto oktas{layer.coverage * 8.f};
		if (ImGui::SliderFloat("coverage, oktas", &oktas, 0.f, 8.f, "%.1f"))
		{
			layer.coverage = oktas / 8.f;
			edited = true;
		}
		edited |= ImGui::SliderFloat("base, m", &layer.base, 0.f, 12000.f, "%.0f");
		edited |= ImGui::SliderFloat("top, m", &layer.top, 0.f, 15000.f, "%.0f");
		edited |= ImGui::SliderFloat("density", &layer.density, 0.f, 3.f);
		if (ImGui::Button("remove layer"))
			removed = static_cast<int>(i);
		ImGui::PopID();
	}
	if (removed >= 0)
	{
		conditions.clouds.erase(conditions.clouds.begin() + removed);
		edited = true;
	}
	if (conditions.clouds.size() < weather_conditions::max_cloud_layers && ImGui::Button("add layer##weatheradd"))
	{
		auto const &genus{cloud_genus_info(cloud_type::cumulus)};
		conditions.clouds.push_back(cloud_layer{cloud_type::cumulus, 0.4f, genus.base, genus.base + genus.thickness, 1.f});
		edited = true;
	}
	if (edited)
	{
		Weather.apply(conditions, 0.f, true);
		changed = true;
	}
	return changed;
}
