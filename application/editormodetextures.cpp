/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "utilities/translation.h"
#include "imgui/imgui.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>

namespace
{

// payload of a material dragged from the texture browser: its name, with the terminating zero
char const *const material_payload{"EU07_MATERIAL"};

// files the browser shows: textures, and the materials, which are kept with them
bool texture_file(std::string const &Extension)
{
	return Extension == ".dds" || Extension == ".tga" || Extension == ".png" || Extension == ".bmp" || Extension == ".jpg" || Extension == ".ktx" || Extension == ".tex";
}

std::string lowercase(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return Text;
}

// the text cut to fit the width, with an ellipsis
std::string fitted(std::string const &Text, float const Width)
{
	if (ImGui::CalcTextSize(Text.c_str()).x <= Width)
		return Text;
	auto result{Text};
	while (false == result.empty() && ImGui::CalcTextSize((result + "...").c_str()).x > Width)
		result.pop_back();
	return result + "...";
}

} // namespace

bool editor_mode::material_drop(std::string &Name)
{
	if (false == ImGui::BeginDragDropTarget())
		return false;
	auto dropped{false};
	if (auto const *payload{ImGui::AcceptDragDropPayload(material_payload)})
	{
		auto const *data{static_cast<char const *>(payload->Data)};
		Name.assign(data, std::find(data, data + payload->DataSize, '\0'));
		dropped = false == Name.empty();
	}
	ImGui::EndDragDropTarget();
	return dropped;
}

bool editor_mode::material_drop(char *Buffer, std::size_t const Size)
{
	std::string name;
	if (Size == 0 || false == material_drop(name))
		return false;
	auto const length{std::min(name.size(), Size - 1)};
	std::memcpy(Buffer, name.data(), length);
	Buffer[length] = '\0';
	return true;
}

std::uint64_t editor_mode::texture_thumbnail(texture_browser::item const &Item)
{
	auto &browser{m_textures};
	auto lookup{browser.thumbnails.find(Item.name)};
	if (lookup == browser.thumbnails.end())
	{
		// reading a texture takes a moment, a few of them go in a frame
		if (browser.loads >= 2)
			return 0;
		++browser.loads;
		texture_handle texture{null_handle};
		if (Item.material)
		{
			auto const material{GfxRenderer->Fetch_Material(Item.name)};
			if (auto const *data{GfxRenderer->Material(material)}; material != null_handle && data != nullptr)
				texture = data->GetTexture(0);
		}
		else
			texture = GfxRenderer->Fetch_Texture(Item.name, true);
		lookup = browser.thumbnails.emplace(Item.name, texture).first;
	}
	if (lookup->second == null_handle)
		return 0;
	auto &texture{GfxRenderer->Texture(lookup->second)};
	if (false == texture.get_is_ready())
		texture.create();
	return texture.get_is_ready() ? static_cast<std::uint64_t>(texture.get_id()) : 0;
}

void editor_mode::render_texture_browser()
{
	auto &browser{m_textures};
	browser.loads = 0;
	if (false == browser.listed)
	{
		browser.listed = true;
		browser.folders.clear();
		browser.items.clear();
		std::map<std::string, texture_browser::item> items;
		std::error_code error;
		for (std::filesystem::directory_iterator entry{std::string{paths::textures} + browser.folder, error}, end; false == static_cast<bool>(error) && entry != end; entry.increment(error))
		{
			auto const path{entry->path()};
			if (entry->is_directory(error))
			{
				browser.folders.push_back(path.filename().string());
				continue;
			}
			auto const extension{lowercase(path.extension().string())};
			auto const material{extension == ".mat"};
			if (false == material && false == texture_file(extension))
				continue;
			auto const stem{path.stem().string()};
			auto &item{items[stem]};
			item.name = browser.folder + stem;
			item.label = stem;
			item.material |= material;
		}
		for (auto &item : items)
			browser.items.push_back(std::move(item.second));
		std::sort(browser.folders.begin(), browser.folders.end());
	}

	// where the browser is, and the folders under it
	ImGui::BeginDisabled(browser.folder.empty());
	if (ImGui::Button(STR_C("Up")))
	{
		auto folder{browser.folder};
		if (false == folder.empty())
			folder.pop_back();
		auto const slash{folder.rfind('/')};
		browser.folder = (slash == std::string::npos ? std::string{} : folder.substr(0, slash + 1));
		browser.listed = false;
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Refresh")))
	{
		browser.listed = false;
		browser.thumbnails.clear();
	}
	ImGui::SameLine();
	ImGui::TextUnformatted((std::string{paths::textures} + browser.folder).c_str());
	ImGui::SetNextItemWidth(std::max(80.0f, ImGui::GetContentRegionAvail().x * 0.5f));
	ImGui::InputTextWithHint("##texturefilter", STR_C("Filter"), browser.filter, IM_ARRAYSIZE(browser.filter));
	ImGui::SameLine();
	ImGui::SetNextItemWidth(std::max(60.0f, ImGui::GetContentRegionAvail().x));
	ImGui::SliderFloat("##texturesize", &browser.size, 48.0f, 256.0f, "%.0f px");
	if (false == browser.folders.empty())
	{
		// the folders flow along the width of the window
		auto const right{ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x};
		for (std::size_t index = 0; index < browser.folders.size(); ++index)
		{
			auto const label{"[" + browser.folders[index] + "]"};
			auto const width{ImGui::CalcTextSize(label.c_str()).x + ImGui::GetStyle().FramePadding.x * 2.0f};
			if (index > 0)
			{
				ImGui::SameLine();
				if (ImGui::GetCursorScreenPos().x + width > right)
					ImGui::NewLine();
			}
			ImGui::PushID(static_cast<int>(index));
			if (ImGui::SmallButton(label.c_str()))
			{
				browser.folder += browser.folders[index] + "/";
				browser.listed = false;
			}
			ImGui::PopID();
		}
		if (false == browser.listed)
		{
			// the list is made anew in the next frame
			return;
		}
	}
	ImGui::Separator();

	std::vector<texture_browser::item const *> shown;
	auto const filter{lowercase(browser.filter)};
	for (auto const &item : browser.items)
		if (filter.empty() || lowercase(item.label).find(filter) != std::string::npos)
			shown.push_back(&item);
	if (shown.empty())
	{
		ImGui::TextDisabled("%s", STR_C("No textures here"));
		return;
	}
	ImGui::TextDisabled("%s", STR_C("Drag a texture onto a field of a material; double click: add it to the palette of the terrain"));

	ImGui::BeginChild("##texturecards");
	auto const &style{ImGui::GetStyle()};
	auto const side{std::max(16.0f, browser.size * std::max(1.0f, Global.ui_scale))};
	auto const cellheight{side + ImGui::GetTextLineHeight() + style.ItemInnerSpacing.y};
	auto const columns{std::max(1, static_cast<int>((ImGui::GetContentRegionAvail().x + style.ItemSpacing.x) / (side + style.ItemSpacing.x)))};
	auto const rows{(static_cast<int>(shown.size()) + columns - 1) / columns};
	auto *drawlist{ImGui::GetWindowDrawList()};
	ImGuiListClipper clipper;
	clipper.Begin(rows, cellheight + style.ItemSpacing.y);
	while (clipper.Step())
	{
		for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
		{
			for (int column = 0; column < columns; ++column)
			{
				auto const index{static_cast<std::size_t>(row) * columns + column};
				if (index >= shown.size())
					break;
				auto const &item{*shown[index]};
				if (column > 0)
					ImGui::SameLine();
				ImGui::PushID(item.name.c_str());
				ImGui::BeginGroup();
				auto const corner{ImGui::GetCursorScreenPos()};
				ImGui::InvisibleButton("##card", ImVec2(side, cellheight));
				auto const hovered{ImGui::IsItemHovered()};
				if (ImGui::IsItemClicked(ImGuiMouseButton_Left))
				{
					browser.selected = item.name;
					std::snprintf(m_terrain_material, IM_ARRAYSIZE(m_terrain_material), "%s", item.name.c_str());
				}
				if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && m_streamer.active())
					m_terrain_layer = m_streamer.layer(item.name, m_terrain_material_size);
				auto const texture{texture_thumbnail(item)};
				if (ImGui::BeginDragDropSource())
				{
					ImGui::SetDragDropPayload(material_payload, item.name.c_str(), item.name.size() + 1);
					if (texture != 0)
						ImGui::Image(static_cast<ImTextureID>(texture), ImVec2(64.0f, 64.0f), ImVec2(0, 1), ImVec2(1, 0));
					ImGui::TextUnformatted(item.name.c_str());
					ImGui::EndDragDropSource();
				}
				ImVec2 const low{corner.x, corner.y}, high{corner.x + side, corner.y + side};
				if (texture != 0)
					drawlist->AddImage(static_cast<ImTextureID>(texture), low, high, ImVec2(0, 1), ImVec2(1, 0));
				else
				{
					drawlist->AddRectFilled(low, high, ImGui::GetColorU32(ImGuiCol_FrameBg));
					drawlist->AddText(ImVec2(low.x + style.FramePadding.x, low.y + style.FramePadding.y), ImGui::GetColorU32(ImGuiCol_TextDisabled), "...");
				}
				if (item.material)
					drawlist->AddText(ImVec2(low.x + style.FramePadding.x, high.y - ImGui::GetTextLineHeight() - style.FramePadding.y), IM_COL32(255, 220, 120, 255), "mat");
				if (browser.selected == item.name || hovered)
					drawlist->AddRect(low, high, ImGui::GetColorU32(browser.selected == item.name ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered), 0.0f, 0, 2.0f);
				drawlist->AddText(ImVec2(low.x, high.y + style.ItemInnerSpacing.y), ImGui::GetColorU32(ImGuiCol_Text), fitted(item.label, side).c_str());
				if (hovered)
				{
					ImGui::BeginTooltip();
					ImGui::TextUnformatted(item.name.c_str());
					ImGui::TextDisabled("%s", item.material ? STR_C("material") : STR_C("texture"));
					ImGui::EndTooltip();
				}
				ImGui::EndGroup();
				ImGui::PopID();
			}
		}
	}
	ImGui::EndChild();
}
