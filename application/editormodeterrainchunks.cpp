/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the list of the chunks of the heightmap terrain: the pack files they're kept in, their grids and triangles, and the camera
// taken to the chunk double clicked

#include "stdafx.h"
#include "application/editormode.h"
#include "editor/editorFormat.hpp"
#include "utilities/Globals.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <tuple>

namespace
{

// how often the list takes in what changed in memory: the chunks loaded, edited and drawn
double constexpr list_refresh{0.5};

enum terrain_chunk_column
{
	column_chunk,
	column_file,
	column_spacing,
	column_triangles,
	column_drawn,
	column_state,
	column_count
};

enum class chunk_state
{
	stored, // in the file, not in memory
	loaded,
	edited, // changed since it was loaded or saved
	added // not in the files yet
};

// a large number with its thousands set apart, to be read at a glance
std::string grouped(std::size_t const Value)
{
	auto digits{std::to_string(Value)};
	for (auto i = static_cast<int>(digits.size()) - 3; i > 0; i -= 3)
		digits.insert(static_cast<std::size_t>(i), " ");
	return digits;
}

std::size_t grid_triangles(int const Cells)
{
	return static_cast<std::size_t>(Cells) * static_cast<std::size_t>(Cells) * 2;
}

} // namespace

void editor_mode::render_terrain_chunks_window()
{
	auto &list{m_chunklist};
	if (false == list.open)
	{
		// read again when it's opened, the files may have changed meanwhile
		list.scanned = false;
		return;
	}
	ImGui::SetNextWindowSize(ImVec2(560.0f * Global.ui_scale, 460.0f * Global.ui_scale), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin((std::string(STR_C("Terrain chunks")) + "###terrainchunks").c_str(), &list.open, ImGuiWindowFlags_NoCollapse))
	{
		ImGui::End();
		return;
	}
	if (false == m_streamer.active())
	{
		ImGui::TextWrapped("%s", STR_C("The scenery has no heightmap terrain yet"));
		list.scanned = false;
		list.rows.clear();
		list.chosen.reset();
		ImGui::End();
		return;
	}

	// the files are read when the list opens, when it's asked to, and after the terrain was saved
	auto const modified{m_streamer.modified()};
	auto const saved{list.modified && false == modified};
	list.modified = modified;
	auto const now{ImGui::GetTime()};
	if (false == list.scanned || saved)
	{
		list.stored = m_streamer.stored_chunks();
		list.scanned = true;
		list.built = -1.0;
	}

	// the chunks of the files with the ones in memory, which may be edited, added since, or of another grid by now
	if (list.built < 0.0 || now - list.built > list_refresh || list.resort)
	{
		std::vector<editor_terrain *> resident;
		m_streamer.collect(resident);
		std::vector<terrain_chunk_row> rows;
		rows.reserve(list.stored.size() + resident.size());
		for (auto const &[key, cells] : list.stored)
			if (false == m_streamer.removed(key))
				rows.push_back({key, cells, true});
		auto const stored{rows.size()};
		for (auto const *chunk : resident)
		{
			auto const key{chunk->key()};
			auto const found{std::lower_bound(rows.begin(), rows.begin() + stored, key, [](terrain_chunk_row const &Row, heightmap::chunk_key const &Key) { return Row.key < Key; })};
			if (found != rows.begin() + stored && found->key == key)
				found->cells = chunk->cells();
			else
				rows.push_back({key, chunk->cells(), false});
		}
		// what the rows stand for, before the filter
		std::vector<heightmap::pack_key> packs;
		list.triangles = 0;
		for (auto const &row : rows)
		{
			list.triangles += grid_triangles(row.cells);
			packs.push_back(heightmap::pack_of(row.key));
		}
		std::sort(packs.begin(), packs.end());
		list.files = static_cast<std::size_t>(std::unique(packs.begin(), packs.end()) - packs.begin());
		list.chunks = rows.size();
		// the filter: a part of the name of the file, or of the coordinates of the chunk
		if (list.filter[0] != '\0')
		{
			std::string const filter{list.filter};
			std::erase_if(rows, [&](terrain_chunk_row const &Row) {
				return heightmap::pack_file_name(heightmap::pack_of(Row.key)).find(filter) == std::string::npos &&
				       format("%d,%d", Row.key.first, Row.key.second).find(filter) == std::string::npos;
			});
		}
		list.rows = std::move(rows);
		list.built = now;
		list.resort = true;
	}

	// the state of a chunk and what's drawn of it, as it's now
	auto const state_of = [&](terrain_chunk_row const &Row, editor_terrain const *Chunk) {
		if (false == Row.stored)
			return chunk_state::added;
		if (Chunk == nullptr)
			return chunk_state::stored;
		return Chunk->modified() ? chunk_state::edited : chunk_state::loaded;
	};
	auto const drawn_of = [](editor_terrain const *Chunk) -> std::size_t { return Chunk != nullptr ? Chunk->triangles() : 0; };

	// summary, of all the chunks whatever the filter
	std::vector<editor_terrain *> resident;
	m_streamer.collect(resident);
	std::size_t drawn{0}, unsaved{0};
	for (auto const *chunk : resident)
	{
		drawn += drawn_of(chunk);
		if (chunk->modified())
			++unsaved;
	}
	auto const area{static_cast<double>(list.chunks) * heightmap::chunk_size * heightmap::chunk_size / 1e6};
	ImGui::Text(STR_C("Chunks: %zu in %zu files, %.2f km2"), list.chunks, list.files, area);
	ImGui::Text(STR_C("In memory: %zu, changed and not saved: %zu"), resident.size(), unsaved);
	ImGui::Text(STR_C("Terrain triangles: %s"), grouped(list.triangles).c_str());
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Of the full grids of all the chunks, as their point spacing makes them; the holes not taken off"));
	ImGui::Text(STR_C("Drawn now: %s"), grouped(drawn).c_str());
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Triangles of the chunks drawn around the camera, with the grids made coarser with the distance"));

	ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12.0f);
	if (ImGui::InputTextWithHint("##chunkfilter", STR_C("Filter: file or x,z"), list.filter, sizeof(list.filter)))
		list.built = -1.0;
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Read the files again")))
	{
		list.scanned = false;
		list.built = -1.0;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("The files are read when the list opens and after the terrain is saved"));
	ImGui::SameLine();
	auto const here{heightmap::chunk_at(Camera.Pos.x, Camera.Pos.z)};
	auto findhere{false};
	if (ImGui::Button(STR_C("Under the camera")))
	{
		list.chosen = here;
		findhere = true;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Shows the chunk the camera is over in the list"));
	ImGui::TextDisabled("%s", STR_C("Double click: the camera goes to the chunk"));

	auto const flags{ImGuiTableFlags_Resizable | ImGuiTableFlags_Sortable | ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit};
	if (ImGui::BeginTable("##terrainchunks", column_count, flags, ImVec2(0.0f, 0.0f)))
	{
		ImGui::TableSetupScrollFreeze(0, 1);
		ImGui::TableSetupColumn(STR_C("Chunk"), ImGuiTableColumnFlags_DefaultSort, 0.0f, column_chunk);
		ImGui::TableSetupColumn(STR_C("File"), 0, 0.0f, column_file);
		ImGui::TableSetupColumn(STR_C("Spacing"), 0, 0.0f, column_spacing);
		ImGui::TableSetupColumn(STR_C("Triangles"), ImGuiTableColumnFlags_PreferSortDescending, 0.0f, column_triangles);
		ImGui::TableSetupColumn(STR_C("Drawn"), ImGuiTableColumnFlags_PreferSortDescending, 0.0f, column_drawn);
		ImGui::TableSetupColumn(STR_C("State"), ImGuiTableColumnFlags_WidthStretch, 0.0f, column_state);
		ImGui::TableHeadersRow();

		if (auto *specs{ImGui::TableGetSortSpecs()}; specs != nullptr && specs->SpecsCount > 0)
		{
			if (specs->SpecsDirty)
			{
				list.sortcolumn = specs->Specs[0].ColumnUserID;
				list.descending = specs->Specs[0].SortDirection == ImGuiSortDirection_Descending;
				specs->SpecsDirty = false;
				list.resort = true;
			}
		}
		if (list.resort)
		{
			// the values which change with the camera are sorted as they were at the time
			auto const value = [&](terrain_chunk_row const &Row) -> std::tuple<long long, long long, long long> {
				switch (list.sortcolumn)
				{
				case column_file:
				{
					auto const pack{heightmap::pack_of(Row.key)};
					return {pack.first, pack.second, 0};
				}
				case column_spacing:
					return {-Row.cells, 0, 0};
				case column_triangles:
					return {static_cast<long long>(grid_triangles(Row.cells)), 0, 0};
				case column_drawn:
					return {static_cast<long long>(drawn_of(m_streamer.chunk(Row.key))), 0, 0};
				case column_state:
					return {static_cast<long long>(state_of(Row, m_streamer.chunk(Row.key))), 0, 0};
				default:
					return {0, 0, 0};
				}
			};
			std::vector<std::pair<std::tuple<long long, long long, long long>, std::size_t>> order;
			order.reserve(list.rows.size());
			for (std::size_t i = 0; i < list.rows.size(); ++i)
				order.emplace_back(value(list.rows[i]), i);
			auto const descending{list.descending};
			auto const &rows{list.rows};
			std::sort(order.begin(), order.end(), [&](auto const &A, auto const &B) {
				if (A.first != B.first)
					return descending ? B.first < A.first : A.first < B.first;
				// the same value: by the place of the chunk, the same way
				auto const &a{rows[A.second].key}, &b{rows[B.second].key};
				return descending ? b < a : a < b;
			});
			std::vector<terrain_chunk_row> sorted;
			sorted.reserve(list.rows.size());
			for (auto const &entry : order)
				sorted.push_back(list.rows[entry.second]);
			list.rows = std::move(sorted);
			list.resort = false;
		}

		auto const height{ImGui::GetTextLineHeightWithSpacing()};
		ImGuiListClipper clipper;
		clipper.Begin(static_cast<int>(list.rows.size()));
		if (findhere)
		{
			auto const found{std::find_if(list.rows.begin(), list.rows.end(), [&](terrain_chunk_row const &Row) { return Row.key == here; })};
			if (found != list.rows.end())
			{
				auto const index{static_cast<int>(found - list.rows.begin())};
				clipper.IncludeItemByIndex(index);
				ImGui::SetScrollY(std::max(0.0f, index * height - ImGui::GetWindowHeight() * 0.5f));
			}
		}
		while (clipper.Step())
		{
			for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i)
			{
				auto const &row{list.rows[static_cast<std::size_t>(i)]};
				auto const *chunk{m_streamer.chunk(row.key)};
				ImGui::TableNextRow();
				if (row.key == here)
					ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, IM_COL32(90, 140, 220, 60));
				ImGui::TableSetColumnIndex(column_chunk);
				auto const chosen{list.chosen && *list.chosen == row.key};
				auto const label{format("%d, %d##chunk%d_%d", row.key.first, row.key.second, row.key.first, row.key.second)};
				if (ImGui::Selectable(label.c_str(), chosen, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick))
				{
					list.chosen = row.key;
					if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
						go_to_terrain_chunk(row.key);
				}
				ImGui::TableSetColumnIndex(column_file);
				ImGui::TextUnformatted(heightmap::pack_file_name(heightmap::pack_of(row.key)).c_str());
				ImGui::TableSetColumnIndex(column_spacing);
				if (row.cells > 0)
					ImGui::Text("%g m", heightmap::chunk_size / row.cells);
				else
					ImGui::TextDisabled("?");
				ImGui::TableSetColumnIndex(column_triangles);
				ImGui::TextUnformatted(grouped(grid_triangles(row.cells)).c_str());
				ImGui::TableSetColumnIndex(column_drawn);
				if (auto const count{drawn_of(chunk)}; count > 0)
					ImGui::TextUnformatted(grouped(count).c_str());
				else
					ImGui::TextDisabled("-");
				ImGui::TableSetColumnIndex(column_state);
				switch (state_of(row, chunk))
				{
				case chunk_state::stored:
					ImGui::TextDisabled("%s", STR_C("in the file"));
					break;
				case chunk_state::loaded:
					ImGui::TextUnformatted(STR_C("loaded"));
					break;
				case chunk_state::edited:
					ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f), "%s", STR_C("changed, not saved"));
					break;
				case chunk_state::added:
					ImGui::TextColored(ImVec4(0.45f, 0.9f, 0.45f, 1.0f), "%s", STR_C("new, not saved"));
					break;
				}
			}
		}
		ImGui::EndTable();
	}
	ImGui::End();
}

void editor_mode::go_to_terrain_chunk(heightmap::chunk_key const &Key)
{
	auto const corner{heightmap::chunk_corner(Key)};
	auto const half{heightmap::chunk_size * 0.5};
	glm::dvec3 centre{corner.x + half, 0.0, corner.y + half};
	// the height of the ground in the middle; a chunk not in memory is read for it
	if (auto const *chunk{m_streamer.chunk(Key, true)})
		centre.y = chunk->height_at(centre.x, centre.z);
	else if (double ground{0.0}; m_streamer.height_at(Camera.Pos.x, Camera.Pos.z, ground))
		centre.y = ground;
	m_chunklist.chosen = Key;
	if (Global.EditorOrtho)
	{
		// the top view only moves over it
		m_focus_active = false;
		Camera.Pos.x = centre.x;
		Camera.Pos.z = centre.z;
		Camera.Velocity = glm::dvec3{0.0};
		Global.pCamera = Camera;
		return;
	}
	// from the side the camera looks from, down at the chunk, from far enough to see the whole of it
	auto const yaw{static_cast<double>(Camera.Angle.y)};
	auto const pitch{glm::radians(35.0)};
	glm::dvec3 const look{-std::sin(yaw) * std::cos(pitch), -std::sin(pitch), -std::cos(yaw) * std::cos(pitch)};
	fly_to(centre, look, heightmap::chunk_size * 1.2);
}
