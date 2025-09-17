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

	if (triviaData[lang].empty())
	{
		ErrorLog("No trivia entries found for language \"" + lang + "\".");
		return trivia;
	}


	// select random trivia
	int i = RandomInt(0, triviaData[lang].size() - 1);
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
	ImVec2 screen_size(Global.window_size.x, Global.window_size.y);
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
	
	// [O] Loading...
	const float margin_left_icon = 35.0f;
	const float margin_bottom_loading = 80.0f;
	const float spacing = 10.0f; // odstęp między ikoną a tekstem

	// Loading icon
	const deferred_image *img = &m_loading_wheel_frames[38];
	const auto loading_tex = img->get();
	const auto loading_size = img->size();

	ImVec2 icon_pos(margin_left_icon, screen_size.y - margin_bottom_loading - loading_size.y);

	// Loading text
	ImGui::SetWindowFontScale(font_scale_mult * 0.8f);
	ImVec2 text_size = ImGui::CalcTextSize(m_progresstext.c_str());

	// Vertical centering of text relative to icon
	float icon_center_y = icon_pos.y + loading_size.y * 0.5f;
	ImVec2 text_pos(icon_pos.x + loading_size.x + spacing, // tuż obok ikony
	                icon_center_y - text_size.y * 0.5f);

	// Draw
	draw_list->AddImage(reinterpret_cast<ImTextureID>(loading_tex), icon_pos, ImVec2(icon_pos.x + loading_size.x, icon_pos.y + loading_size.y), ImVec2(0, 0), ImVec2(1, 1));
	draw_list->AddText(text_pos, IM_COL32_WHITE, m_progresstext.c_str());

	// Trivia 
	// draw only if we have any trivia loaded
	if (m_trivia.size() > 0)
	{
		const float margin_right = 80.0f;
		const float margin_bottom = 80.0f;
		const float line_spacing = 25.0f;
		const float header_gap = 10.0f;

		// Measure width of trivia lines
		ImGui::SetWindowFontScale(font_scale_mult * 0.6f);
		float max_width = 0.0f;
		for (const std::string &line : m_trivia)
		{
			ImVec2 size = ImGui::CalcTextSize(line.c_str());
			if (size.x > max_width)
				max_width = size.x;
		}

		// Measure header width
		ImGui::SetWindowFontScale(font_scale_mult * 1.0f);
		ImVec2 header_size = ImGui::CalcTextSize(STR_C("Did you know..."));
		if (header_size.x > max_width)
			max_width = header_size.x; // blok musi też pomieścić nagłówek

		// Calculate block position
		float content_height = (float)m_trivia.size() * line_spacing;
		float total_height = header_size.y + header_gap + content_height;

		float block_left = screen_size.x - margin_right - max_width;
		float block_top = screen_size.y - margin_bottom - total_height;

		// Draw header
		ImVec2 header_pos(block_left + (max_width - header_size.x) * 0.5f, block_top);
		draw_list->AddText(header_pos, IM_COL32_WHITE, STR_C("Did you know..."));

		// Draw trivia lines
		ImGui::SetWindowFontScale(font_scale_mult * 0.6f);
		for (int i = 0; i < m_trivia.size(); i++)
		{
			const std::string &line = m_trivia[i];
			ImVec2 text_size = ImGui::CalcTextSize(line.c_str());
			ImVec2 text_pos(block_left + (max_width - text_size.x) * 0.5f, block_top + header_size.y + header_gap + i * line_spacing);
			draw_list->AddText(text_pos, IM_COL32_WHITE, line.c_str());
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