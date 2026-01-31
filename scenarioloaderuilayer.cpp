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
#include <sstream>

using json = nlohmann::json;

// Helper function to count UTF-8 characters (not bytes)
size_t utf8_char_count(const std::string& str)
{
	size_t count = 0;
	for (size_t i = 0; i < str.length(); )
	{
		unsigned char c = static_cast<unsigned char>(str[i]);
		if ((c & 0x80) == 0x00)
		{
			// 1-byte character (ASCII)
			i += 1;
		}
		else if ((c & 0xE0) == 0xC0)
		{
			// 2-byte character
			i += 2;
		}
		else if ((c & 0xF0) == 0xE0)
		{
			// 3-byte character (most Chinese characters)
			i += 3;
		}
		else if ((c & 0xF8) == 0xF0)
		{
			// 4-byte character
			i += 4;
		}
		else
		{
			// Invalid UTF-8, skip
			i += 1;
		}
		count++;
	}
	return count;
}

scenarioloader_ui::scenarioloader_ui()
{
	m_suppress_menu = true;
	generate_gradient_tex();
	load_wheel_frames();

	m_trivia = get_random_trivia();
}


std::vector<std::string> scenarioloader_ui::get_random_trivia()
{
	WriteLog("Loading random trivia...");
	std::vector<std::string> trivia = std::vector<std::string>();

	if (!FileExists("lang/trivia_" + Global.asLang + ".json") 
		&& !FileExists("lang/trivia_en.json"))
	{
		ErrorLog("Trivia file not found!");
		return trivia;
	}

	std::string triviaFile = FileExists("lang/trivia_" + Global.asLang + ".json") ? "lang/trivia_" + Global.asLang + ".json" : "lang/trivia_en.json";

	//std::string lang = Global.asLang;
	WriteLog("Selected language: " + Global.asLang);
	// Read file in binary mode to preserve UTF-8 encoding
	std::ifstream f(triviaFile, std::ios::binary);
	std::stringstream buffer;
	buffer << f.rdbuf();
	std::string fileContent = buffer.str();
	json triviaData = json::parse(fileContent);

	// select random
	int i = RandomInt(0, static_cast<int>(triviaData.size()) - 1);
	std::string triviaStr = triviaData[i]["text"];
	std::string background = triviaData[i]["background"];

	// divide trivia into multiple lines - UTF-8 safe implementation
	// Different languages need different character limits due to character width differences
	int max_line_length = 100;  // Default for Latin languages
	if (Global.asLang == "zh")
	{
		max_line_length = 60;  // Reduced for Chinese
	}

	// Find byte position corresponding to character count
	auto get_byte_pos_for_char_count = [&](const std::string& str, size_t char_count) -> size_t {
		size_t byte_pos = 0;
		size_t current_chars = 0;
		for (size_t i = 0; i < str.length() && current_chars < char_count; )
		{
			unsigned char c = static_cast<unsigned char>(str[i]);
			if ((c & 0x80) == 0x00)
			{
				i += 1;
			}
			else if ((c & 0xE0) == 0xC0)
			{
				i += 2;
			}
			else if ((c & 0xF0) == 0xE0)
			{
				i += 3;
			}
			else if ((c & 0xF8) == 0xF0)
			{
				i += 4;
			}
			else
			{
				i += 1;
			}
			current_chars++;
			byte_pos = i;
		}
		return byte_pos;
	};

	// UTF-8 safe find space within character limit
	auto find_space_before_char_count = [&](const std::string& str, size_t max_chars) -> size_t {
		size_t byte_limit = get_byte_pos_for_char_count(str, max_chars);
		size_t search_limit = std::min(str.length(), byte_limit);

		// Search backwards for a space that's not in the middle of a UTF-8 sequence
		for (int idx = static_cast<int>(search_limit); idx >= 0; idx--) {
			// Check if this position is a valid place to split (not in middle of UTF-8 char)
			// A UTF-8 continuation byte has the form 10xxxxxx (0x80-0xBF)
			if (idx == 0 || (static_cast<unsigned char>(str[idx - 1]) & 0xC0) != 0x80) {
				// This is the start of a character (or beginning of string), check if it's a space
				if (idx > 0 && str[idx - 1] == ' ') {
					return idx - 1;  // Position of the space
				}
			}
		}
		return std::string::npos;  // No suitable space found
	};

	// Use character count for proper UTF-8 text wrapping
	while (utf8_char_count(triviaStr) > static_cast<size_t>(max_line_length))
	{
		size_t split_pos = find_space_before_char_count(triviaStr, static_cast<size_t>(max_line_length));
		if (split_pos == std::string::npos)
		{
			// No space found, split at max character count
			split_pos = get_byte_pos_for_char_count(triviaStr, static_cast<size_t>(max_line_length));
		}
		trivia.push_back(triviaStr.substr(0, split_pos));
		triviaStr = triviaStr.substr(split_pos);
		// Skip leading whitespace in the next line
		while (!triviaStr.empty() && triviaStr[0] == ' ')
		{
			triviaStr = triviaStr.substr(1);
		}
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
	if (!Global.NvRenderer)
	{
		const auto tex = reinterpret_cast<ImTextureID>(m_gradient_overlay_tex);
		draw_list->AddImage(tex, ImVec2(0, Global.window_size.y / 2), ImVec2(Global.window_size.x, Global.window_size.y), ImVec2(0, 0), ImVec2(1, 1));
	}
	
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
	//draw_list->AddImage(reinterpret_cast<ImTextureID>(loading_tex), icon_pos, ImVec2(icon_pos.x + loading_size.x, icon_pos.y + loading_size.y), ImVec2(0, 0), ImVec2(1, 1));
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
		ImVec2 header_size = ImGui::CalcTextSize(STR_C("Did you know?"));
		if (header_size.x > max_width)
			max_width = header_size.x; // blok musi też pomieścić nagłówek

		// Calculate block position
		float content_height = (float)m_trivia.size() * line_spacing;
		float total_height = header_size.y + header_gap + content_height;

		float block_left = screen_size.x - margin_right - max_width;
		float block_top = screen_size.y - margin_bottom - total_height;

		// Draw header
		ImVec2 header_pos(block_left + (max_width - header_size.x) * 0.5f, block_top);
		draw_list->AddText(header_pos, IM_COL32_WHITE, STR_C("Did you know?"));

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
	if (Global.NvRenderer)
		return;
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
