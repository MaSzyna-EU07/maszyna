/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "scenarioloaderuilayer.h"

#include "Globals.h"
#include "translation.h"
#include <nlohmann/json.hpp>
#include "Logs.h"

using json = nlohmann::json;

scenarioloader_ui::scenarioloader_ui()
{
	m_suppress_menu = true;
	load_random_background();
	generate_gradient_tex();
	load_wheel_frames();

	m_trivia = get_random_trivia();
}


std::vector<std::string> scenarioloader_ui::get_random_trivia()
{
	WriteLog("Loading random trivia...");
	std::vector<std::string> trivia = std::vector<std::string>();

	if (!FileExists("lang/trivia.json"))
	{
		ErrorLog("File lang/trivia.json not found!");
		return trivia;
	}
	std::string lang = Global.asLang;

	std::ifstream f("lang/trivia.json");
	json triviaData = json::parse(f);

	// check if lang set exists
	if (triviaData.find(lang) == triviaData.end())
	{
		ErrorLog("No trivia found for language \"" + lang + "\", falling back to English.");
		lang = "en";
	}

	// select random trivia
	int i = Random(0, triviaData[lang].size() - 1);
	std::string triviaStr = triviaData[lang][i]["text"];
	std::string background = triviaData[lang][i]["background"];

	// divide trivia into multiple lines
	const int max_line_length = 100;
	while (triviaStr.length() > max_line_length)
	{
		int split_pos = triviaStr.rfind(' ', max_line_length);
		if (split_pos == std::string::npos)
			split_pos = max_line_length; // no space found, force split
		trivia.push_back(triviaStr.substr(0, split_pos));
		triviaStr = triviaStr.substr(split_pos + 1); // +1 to skip the space
	}

	// if triviaStr is not empty add this as last line
	if (!triviaStr.empty())
		trivia.push_back(triviaStr);

	// now override background if trivia is set
	if (!trivia.empty())
	{
		set_background("textures/ui/backgrounds/" + background);
	}


	return trivia;
}

void scenarioloader_ui::render_()
{
	// For some reason, ImGui windows have some padding. Offset it.
	// TODO: Find out a way to exactly adjust the position.
	constexpr int padding = 12;
	ImGui::SetNextWindowPos(ImVec2(-padding, -padding));
	ImGui::SetNextWindowSize(ImVec2(Global.window_size.x + padding * 2, Global.window_size.y + padding * 2));
	ImGui::Begin("Neo Loading Screen", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground);
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	ImGui::PushFont(font_loading);
	ImGui::SetWindowFontScale(1);
	const float font_scale_mult = 48 / ImGui::GetFontSize();
	
	// Gradient at the lower half of the screen
	const auto tex = reinterpret_cast<ImTextureID>(m_gradient_overlay_tex);
	draw_list->AddImage(tex, ImVec2(0, Global.window_size.y / 2), ImVec2(Global.window_size.x, Global.window_size.y), ImVec2(0, 0), ImVec2(1, 1));
	
	// Loading label
	ImGui::SetWindowFontScale(font_scale_mult * 0.8);
	draw_list->AddText(ImVec2(200, 885), IM_COL32_WHITE, m_progresstext.c_str());
	
	// Loading wheel icon
	const deferred_image* img = &m_loading_wheel_frames[38];
	const auto loading_tex = img->get();
	const auto loading_size = img->size();
	draw_list->AddImage(reinterpret_cast<ImTextureID>(loading_tex), ImVec2(35, 850), ImVec2(35 + loading_size.x, 850 + loading_size.y), ImVec2(0, 0), ImVec2(1, 1));
	
	// Trivia 
	// draw only if we have any trivia loaded
	if (m_trivia.size() > 0)
	{
		// Header
		ImGui::SetWindowFontScale(font_scale_mult * 1);
		draw_list->AddText(ImVec2(1280 - ImGui::CalcTextSize(STR_C("Did you know")).x / 2, 770), IM_COL32_WHITE, STR_C("Did you know"));

		// Content
		ImGui::SetWindowFontScale(font_scale_mult * 0.6);
		for (int i = 0; i < m_trivia.size(); i++)
		{
			std::string line = m_trivia[i];
			draw_list->AddText(ImVec2(1280 - ImGui::CalcTextSize(line.c_str()).x / 2, 825 + i * 25), IM_COL32_WHITE, line.c_str());
		}
	}
	
	// Progress bar at the bottom of the screen
	const auto p1 = ImVec2(0, Global.window_size.y - 2);
	const auto p2 = ImVec2(Global.window_size.x * m_progress, Global.window_size.y);
	draw_list->AddRectFilled(p1, p2, ImColor(40, 210, 60, 255));
	ImGui::PopFont();
	ImGui::End();
}

void scenarioloader_ui::generate_gradient_tex()
{
	constexpr int image_width = 1;
	constexpr int image_height = 256;
	const auto image_data = new char[image_width * image_height * 4];
	for (int x = 0; x < image_width; x++)
		for (int y = 0; y < image_height; y++)
		{
			image_data[(y * image_width + x) * 4] = 0;
			image_data[(y * image_width + x) * 4 + 1] = 0;
			image_data[(y * image_width + x) * 4 + 2] = 0;
			image_data[(y * image_width + x) * 4 + 3] = clamp(static_cast<int>(pow(y / 255.f, 0.7) * 255), 0, 255);
		}

	// Create a OpenGL texture identifier
	GLuint image_texture;
	glGenTextures(1, &image_texture);
	glBindTexture(GL_TEXTURE_2D, image_texture);

	// Setup filtering parameters for display
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Upload pixels into texture
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);

	delete[] image_data;

	m_gradient_overlay_width = image_width;
	m_gradient_overlay_height = image_height;
	m_gradient_overlay_tex = image_texture;
}

void scenarioloader_ui::load_wheel_frames()
{
	for (int i = 0; i < 60; i++)
		m_loading_wheel_frames[i] = deferred_image("ui/loading_wheel/" + std::to_string(i + 1));
}