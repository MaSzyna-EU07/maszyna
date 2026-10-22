/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodesrk.h"
#include "application/editorgestures.h"
#include "application/editormodeshared.h"
#include "application/editorprojection.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorSwitchStore.hpp"

#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_set>

namespace
{

char const *const kMark{"//$s"};
double constexpr kOverlayRange{4000.0};

ImU32 constexpr kPlain{IM_COL32(170, 176, 188, 255)};
ImU32 constexpr kSwitch{IM_COL32(240, 200, 90, 255)};
ImU32 constexpr kApproach{IM_COL32(95, 100, 112, 255)};
ImU32 constexpr kHovered{IM_COL32(255, 255, 255, 255)};

bool is_switch(TTrack const &Track)
{
	return Track.eType != tt_Normal;
}

// a word of the record line: blanks taken out, - for none
std::string word_of(std::string const &Text)
{
	std::string result;
	for (auto const c : Text)
		if (false == std::isspace(static_cast<unsigned char>(c)))
			result += c;
	return result.empty() ? std::string{"-"} : result;
}

std::string shown(std::string const &Word)
{
	return Word == "-" ? std::string{} : Word;
}

} // namespace

void srk_tool::load()
{
	if (m_loaded)
		return;
	m_loaded = true;
	m_records.clear();
	if (scene::Layers.empty())
		return;
	for (auto const layer : scene::Layers.marked_layers(kMark))
		for (auto const &text : scene::Layers.marked(layer, kMark))
		{
			std::istringstream words{text};
			std::string word;
			if ((words >> word).fail() || word != "station")
				continue;
			record entry;
			entry.layer = layer;
			words >> entry.code;
			while (words >> word)
			{
				if (word == "line")
					words >> entry.line;
				else if (word == "anchor")
					words >> entry.anchor;
				else if (word == "name")
				{
					std::getline(words, entry.name);
					entry.name.erase(0, entry.name.find_first_not_of(' '));
					break;
				}
			}
			entry.code = shown(entry.code);
			entry.line = shown(entry.line);
			if (false == entry.anchor.empty())
				m_records.push_back(std::move(entry));
		}
}

void srk_tool::store()
{
	if (false == m_changed || scene::Layers.empty())
		return;
	m_changed = false;
	std::vector<scene::layer_handle> layers{scene::Layers.marked_layers(kMark)};
	for (auto const &entry : m_records)
		if (std::find(layers.begin(), layers.end(), entry.layer) == layers.end())
			layers.push_back(entry.layer);
	for (auto const layer : layers)
	{
		std::vector<std::string> lines;
		for (auto const &entry : m_records)
		{
			if (entry.layer != layer)
				continue;
			if (lines.empty())
				lines.emplace_back("# stations described in the track editor, the simulation skips these lines");
			lines.push_back(format("station %s line %s anchor %s name %s", word_of(entry.code).c_str(), word_of(entry.line).c_str(), entry.anchor.c_str(), entry.name.c_str()));
		}
		scene::Layers.mark(layer, kMark, std::move(lines));
	}
}

void srk_tool::open(TTrack &Track)
{
	m_status.clear();
	auto found{station::find(Track)};
	if (false == found.has_value())
	{
		m_status = STR_C("No switch near this track: it isn't a part of a station");
		return;
	}
	m_layout = std::move(found);
	m_hover = -1;
	m_view_state.exaggeration = 0.0f;
	m_history_size = m_history.history_size();
	// the record of the station, by any of its switches
	m_record = -1;
	for (int index = 0; index < static_cast<int>(m_records.size()) && m_record < 0; ++index)
		if (std::any_of(m_layout->switches.begin(), m_layout->switches.end(), [&](TTrack const *Switch) { return Switch->name() == m_records[index].anchor; }))
			m_record = index;
	m_anchor = m_record >= 0 ? m_records[m_record].anchor : m_layout->switches.front()->name();
}

void srk_tool::refresh()
{
	m_history_size = m_history.history_size();
	auto *anchor{m_anchor.empty() ? nullptr : simulation::Paths.find(m_anchor)};
	if (anchor == nullptr || anchor->m_editorremoved)
	{
		m_layout.reset();
		return;
	}
	auto const view{m_view_state};
	open(*anchor);
	if (m_layout.has_value())
		m_view_state = view;
}

srk_tool::record &srk_tool::record_of_open()
{
	if (m_record < 0)
	{
		record entry;
		auto const *first{m_layout->switches.front()};
		entry.anchor = first->name();
		entry.layer = scene::Layers.resolve(first->layer());
		if (false == scene::Layers.valid(entry.layer) || false == scene::Layers.accepts(entry.layer))
			entry.layer = scene::Layers.active();
		if (false == scene::Layers.valid(entry.layer))
			entry.layer = 1;
		m_records.push_back(std::move(entry));
		m_record = static_cast<int>(m_records.size()) - 1;
		m_anchor = m_records.back().anchor;
	}
	return m_records[m_record];
}

std::string srk_tool::title() const
{
	if (m_record >= 0)
	{
		auto const &entry{m_records[m_record]};
		if (false == entry.name.empty())
			return entry.code.empty() ? entry.name : entry.name + " (" + entry.code + ")";
		if (false == entry.code.empty())
			return entry.code;
	}
	return format(STR_C("Station of %s"), m_anchor.c_str());
}

void srk_tool::find_all()
{
	m_found.clear();
	for (auto *track : station::stations())
		m_found.emplace_back(track->name(), station::switches_of(*track).size());
	std::sort(m_found.begin(), m_found.end(), [](auto const &A, auto const &B) { return A.second > B.second; });
}

void srk_tool::update()
{
	if (false == m_open)
		return;
	load();
	if (m_layout.has_value() && m_history.history_size() != m_history_size)
		refresh();
	else if (false == m_layout.has_value() && false == m_anchor.empty() && m_history.history_size() != m_history_size)
		refresh();
	// a track of another station selected opens it
	auto *track{m_selection.selected_track()};
	if (track == nullptr || track == m_picked)
		return;
	m_picked = track;
	if (m_layout.has_value() && std::any_of(m_layout->paths.begin(), m_layout->paths.end(), [&](station::path const &Path) { return Path.track == track && false == Path.approach; }))
		return;
	open(*track);
}

void srk_tool::forget(scene::basic_node const *Node)
{
	if (Node == m_picked)
		m_picked = nullptr;
	if (m_layout.has_value() && std::any_of(m_layout->paths.begin(), m_layout->paths.end(), [&](station::path const &Path) { return Path.track == Node; }))
	{
		// laid out again from its anchor at the next update
		m_layout.reset();
		m_hover = -1;
		m_history_size = 0;
	}
}

void srk_tool::draw_overlay() const
{
	if (false == m_open || false == m_layout.has_value())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	glm::dvec3 const camera{Global.pCamera.Pos};
	if (m_hover >= 0 && m_hover < static_cast<int>(m_layout->paths.size()))
	{
		auto const &points{m_layout->paths[m_hover].points};
		for (std::size_t i = 1; i < points.size(); ++i)
			projection.line(drawlist, points[i - 1], points[i], kHovered, 4.0f);
	}
	for (auto const *track : m_layout->switches)
	{
		auto const start{track->m_paths.front().points[segment_data::point::start] + glm::dvec3{0.0, 1.0, 0.0}};
		ImVec2 screen;
		if ((false == Global.EditorOrtho && glm::distance(start, camera) > kOverlayRange) || false == projection.project(start, screen))
			continue;
		auto const text{track->name()};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		drawlist->AddRectFilled(ImVec2(screen.x - 3.0f, screen.y - size.y - 4.0f), ImVec2(screen.x + size.x + 3.0f, screen.y), IM_COL32(0, 0, 0, 160), 3.0f);
		drawlist->AddText(ImVec2(screen.x, screen.y - size.y - 2.0f), kSwitch, text.c_str());
	}
}

void srk_tool::render_found()
{
	if (main_button(STR_C("Find the stations of the scenery"), ImVec2(-1.0f, tall_button_height())))
		find_all();
	item_tooltip("Each group of switches joined by tracks shorter than 1.5 km is a station");
	if (m_found.empty())
		return;
	ImGui::TextDisabled("%s", format(STR_P("%zu station", "%zu stations", m_found.size()), m_found.size()).c_str());
	if (false == ImGui::BeginChild("##srkfound", ImVec2(0.0f, std::min(300.0f, ImGui::GetTextLineHeightWithSpacing() * (m_found.size() + 1.0f))), true))
	{
		ImGui::EndChild();
		return;
	}
	for (std::size_t index = 0; index < m_found.size(); ++index)
	{
		auto const &[name, count]{m_found[index]};
		auto const described{std::find_if(m_records.begin(), m_records.end(), [&](record const &Entry) { return Entry.anchor == name; })};
		auto label{format(STR_P("%s: %zu switch", "%s: %zu switches", count), name.c_str(), count)};
		if (described != m_records.end() && false == described->name.empty())
			label = described->name + " - " + label;
		if (ImGui::Selectable((label + "##srkfound" + std::to_string(index)).c_str(), false))
			if (auto *track{simulation::Paths.find(name)}; track != nullptr)
			{
				open(*track);
				m_selection.set_current_node(track);
				m_picked = track;
				m_view.focus_track(*track, 0, track->m_paths.front().points[segment_data::point::start]);
			}
	}
	ImGui::EndChild();
}

void srk_tool::render_options()
{
	if (false == m_layout.has_value())
	{
		ImGui::TextDisabled("%s", STR_C("The signalling of a station: its schematic, the numbers of the switches and the tracks, the track circuits and the signals"));
		ImGui::Spacing();
		gesture_rows({{"LMB on a switch", "opens its station"}});
		if (false == m_status.empty())
			ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", m_status.c_str());
		ImGui::Spacing();
		render_found();
		return;
	}
	auto const &layout{*m_layout};
	ImGui::TextUnformatted(title().c_str());
	std::unordered_set<TTrack const *> tracks;
	for (auto const &path : layout.paths)
		if (false == path.approach)
			tracks.insert(path.track);
	metric_tiles({{format("%zu", layout.switches.size()), STR_C("switches"), kSwitch},
	              {format("%zu", tracks.size() - layout.switches.size()), STR_C("tracks"), kPlain},
	              {format("%.0f m", layout.to - layout.from), STR_C("length"), ImGui::GetColorU32(ImGuiCol_Text)}});
	if (begin_fields("##srkstation"))
	{
		auto const text = [&](char const *Label, char const *Id, std::string const &Value, char const *Hint, auto const &Set) {
			field_row(Label);
			std::array<char, 128> buffer{};
			Value.copy(buffer.data(), buffer.size() - 1);
			if (ImGui::InputTextWithHint(Id, Hint, buffer.data(), buffer.size()))
			{
				Set(std::string{buffer.data()});
				m_changed = true;
			}
		};
		auto const *entry{m_record >= 0 ? &m_records[m_record] : nullptr};
		text("Name", "##srkname", entry != nullptr ? entry->name : std::string{}, "Biała Podlaska", [&](std::string const &Value) { record_of_open().name = Value; });
		text("Code", "##srkcode", entry != nullptr ? entry->code : std::string{}, "BPd", [&](std::string const &Value) { record_of_open().code = word_of(Value) == "-" ? std::string{} : word_of(Value); });
		item_tooltip("The abbreviation of the station: the names of its switches, track circuits and signals begin with it");
		text("Line", "##srkline", entry != nullptr ? entry->line : std::string{}, "2", [&](std::string const &Value) { record_of_open().line = word_of(Value) == "-" ? std::string{} : word_of(Value); });
		item_tooltip("The number of the railway line the station lies on, as the block sections between the stations are named after it");
		ImGui::EndTable();
	}
	ImGui::Spacing();
	if (ImGui::CollapsingHeader(format("%s (%zu)###srkswitches", STR_C("Switches"), layout.switches.size()).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::BeginTable("##srkswitchlist", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY, ImVec2(0.0f, std::min(320.0f, ImGui::GetTextLineHeightWithSpacing() * (layout.switches.size() + 1.5f)))))
		{
			ImGui::TableSetupScrollFreeze(0, 1);
			ImGui::TableSetupColumn(STR_C("Switch"), ImGuiTableColumnFlags_WidthStretch, 0.35f);
			ImGui::TableSetupColumn(STR_C("Type"), ImGuiTableColumnFlags_WidthStretch, 0.45f);
			ImGui::TableSetupColumn(STR_C("km"), ImGuiTableColumnFlags_WidthStretch, 0.2f);
			ImGui::TableHeadersRow();
			auto *selected{m_selection.selected_track()};
			for (std::size_t index = 0; index < layout.switches.size(); ++index)
			{
				auto *track{layout.switches[index]};
				auto const start{track->m_paths.front().points[segment_data::point::start]};
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				if (ImGui::Selectable((track->name() + "##srksw" + std::to_string(index)).c_str(), track == selected, ImGuiSelectableFlags_SpanAllColumns))
				{
					m_selection.set_current_node(track);
					m_picked = track;
					m_view.focus_track(*track, 0, start);
				}
				ImGui::TableNextColumn();
				auto const record{switch_store::named(*track)};
				ImGui::TextUnformatted(record ? switch_store::label(*record).c_str() : track->eType == tt_Switch ? STR_C("switch") : track->eType == tt_Cross ? STR_C("crossing") : STR_C("turntable"));
				ImGui::TableNextColumn();
				auto const along{station::project(layout, start).x};
				if (layout.kilometrage)
					ImGui::Text("%.3f", station::km_of(layout, along));
				else
					ImGui::Text("%.0f m", along);
			}
			ImGui::EndTable();
		}
	}
	ImGui::Spacing();
	if (false == m_status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", m_status.c_str());
	if (ImGui::Button(STR_C("Another station"), ImVec2(-1.0f, 0.0f)))
	{
		m_layout.reset();
		m_anchor.clear();
		m_record = -1;
		m_hover = -1;
	}
	item_tooltip("Leaves this station; LMB on a switch opens another one");
}

void srk_tool::fit_view(float const Width, float const Height)
{
	auto const &layout{*m_layout};
	auto &view{m_view_state};
	auto const span{std::max(50.0, layout.to - layout.from)};
	view.from = layout.from - span * 0.08 - 30.0;
	view.to = layout.to + span * 0.08 + 30.0;
	view.centre = (layout.low + layout.high) * 0.5;
	auto const across{(view.to - view.from) / std::max(1.0f, Width)};
	auto const vertical{(layout.high - layout.low + 20.0) / std::max(1.0f, Height * 0.75f)};
	view.exaggeration = static_cast<float>(std::clamp(across / vertical, 1.0, 500.0));
}

void srk_tool::render_canvas(float const Height)
{
	auto const &layout{*m_layout};
	auto &view{m_view_state};
	auto const &io{ImGui::GetIO()};
	ImVec2 const size{std::max(200.0f, ImGui::GetContentRegionAvail().x), std::max(80.0f, Height)};
	ImVec2 const corner{ImGui::GetCursorScreenPos()};
	ImGui::InvisibleButton("##srkcanvas", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle);
	bool const hovered{ImGui::IsItemHovered()};
	if (view.exaggeration <= 0.0f)
		fit_view(size.x, size.y);
	// the wheel zooms about the mouse, with Ctrl it exaggerates the offsets; a drag moves the plot
	if (hovered && io.MouseWheel != 0.0f)
	{
		if (io.KeyCtrl)
			view.exaggeration = std::clamp(view.exaggeration * std::pow(1.2f, io.MouseWheel), 1.0f, 500.0f);
		else
		{
			auto const at{view.from + (io.MousePos.x - corner.x) / size.x * (view.to - view.from)};
			auto const factor{std::pow(0.8, static_cast<double>(io.MouseWheel))};
			view.from = at - (at - view.from) * factor;
			view.to = at + (view.to - at) * factor;
		}
	}
	auto const across{(view.to - view.from) / size.x};
	auto const vertical{across / view.exaggeration};
	if (ImGui::IsItemActive() && (ImGui::IsMouseDragging(ImGuiMouseButton_Left, kDragPixels) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, kDragPixels)))
	{
		m_dragging = true;
		view.from -= io.MouseDelta.x * across;
		view.to -= io.MouseDelta.x * across;
		view.centre += io.MouseDelta.y * vertical;
	}
	auto const x_of = [&](double const Chainage) { return corner.x + static_cast<float>((Chainage - view.from) / across); };
	auto const y_of = [&](double const Offset) { return corner.y + size.y * 0.5f - static_cast<float>((Offset - view.centre) / vertical); };
	auto const screen_of = [&](glm::dvec2 const &Point) { return ImVec2(x_of(Point.x), y_of(Point.y)); };

	// the path under the mouse
	if (hovered && false == m_dragging)
	{
		m_hover = -1;
		auto nearest{7.0f * 7.0f};
		auto const &mouse{io.MousePos};
		for (int index = 0; index < static_cast<int>(layout.paths.size()); ++index)
		{
			auto const &points{layout.paths[index].schematic};
			for (std::size_t i = 1; i < points.size(); ++i)
			{
				auto const a{screen_of(points[i - 1])};
				auto const b{screen_of(points[i])};
				auto const dx{b.x - a.x};
				auto const dy{b.y - a.y};
				auto const length2{dx * dx + dy * dy};
				auto const t{length2 > 0.0f ? std::clamp(((mouse.x - a.x) * dx + (mouse.y - a.y) * dy) / length2, 0.0f, 1.0f) : 0.0f};
				auto const ex{a.x + t * dx - mouse.x};
				auto const ey{a.y + t * dy - mouse.y};
				if (ex * ex + ey * ey < nearest)
				{
					nearest = ex * ex + ey * ey;
					m_hover = index;
				}
			}
		}
	}
	else if (false == hovered && false == ImGui::IsItemActive())
		m_hover = -1;
	if (ImGui::IsItemDeactivated())
	{
		if (false == m_dragging && m_hover >= 0)
		{
			auto const &path{layout.paths[m_hover]};
			m_selection.set_current_node(path.track);
			m_picked = path.track;
			m_view.focus_track(*path.track, path.index, path.points[path.points.size() / 2]);
		}
		m_dragging = false;
	}

	auto *draw{ImGui::GetWindowDrawList()};
	ImVec2 const corner_end{corner.x + size.x, corner.y + size.y};
	draw->AddRectFilled(corner, corner_end, IM_COL32(22, 24, 28, 255));
	draw->PushClipRect(corner, corner_end, true);
	// the km, or the chainage, along the top
	{
		auto const metres{across * 90.0};
		double step{10.0};
		for (auto const candidate : {10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0})
			if ((step = candidate) >= metres)
				break;
		auto const origin{layout.kilometrage ? layout.km * 1000.0 : 0.0};
		for (auto mark{std::ceil((view.from + origin) / step) * step}; mark <= view.to + origin; mark += step)
		{
			auto const x{x_of(mark - origin)};
			draw->AddLine(ImVec2(x, corner.y), ImVec2(x, corner_end.y), IM_COL32(255, 255, 255, 16));
			auto const label{layout.kilometrage ? format("%.3f", mark / 1000.0) : format("%.0f m", mark)};
			draw->AddText(ImVec2(x + 3.0f, corner.y + 2.0f), IM_COL32(150, 155, 165, 255), label.c_str());
		}
	}
	auto const *selected{m_selection.selected_track()};
	auto const draw_path = [&](station::path const &Path, ImU32 const Colour, float const Width) {
		for (std::size_t i = 1; i < Path.schematic.size(); ++i)
			draw->AddLine(screen_of(Path.schematic[i - 1]), screen_of(Path.schematic[i]), Colour, Width);
	};
	for (auto const &path : layout.paths)
		if (path.approach)
			draw_path(path, kApproach, 2.0f);
	for (auto const &path : layout.paths)
		if (false == path.approach)
			draw_path(path, is_switch(*path.track) ? kSwitch : kPlain, 2.5f);
	for (auto const &path : layout.paths)
		if (path.track == selected)
			draw_path(path, overlay_color::selected, 4.0f);
	if (m_hover >= 0)
		draw_path(layout.paths[m_hover], kHovered, 4.0f);
	// the switches: a dot at the start, the name over or under it in turn
	for (std::size_t index = 0; index < layout.switches.size(); ++index)
	{
		auto const *track{layout.switches[index]};
		auto const at{screen_of(station::project(layout, track->m_paths.front().points[segment_data::point::start]))};
		draw->AddCircleFilled(at, 3.5f, kSwitch);
		auto const text{track->name()};
		auto const extent{ImGui::CalcTextSize(text.c_str())};
		ImVec2 const place{at.x - extent.x * 0.5f, index % 2 == 0 ? at.y - extent.y - 6.0f : at.y + 6.0f};
		draw->AddRectFilled(ImVec2(place.x - 2.0f, place.y - 1.0f), ImVec2(place.x + extent.x + 2.0f, place.y + extent.y + 1.0f), IM_COL32(22, 24, 28, 200), 2.0f);
		draw->AddText(place, kSwitch, text.c_str());
	}
	draw->PopClipRect();
	if (hovered && m_hover >= 0 && false == m_dragging)
	{
		auto const &path{layout.paths[m_hover]};
		auto const along{view.from + (io.MousePos.x - corner.x) * across};
		ImGui::BeginTooltip();
		ImGui::TextUnformatted(path.track->name().empty() ? STR_C("(no name)") : path.track->name().c_str());
		ImGui::TextDisabled("%s, %.1f m", is_switch(*path.track) ? STR_C("switch") : path.approach ? STR_C("line out of the station") : STR_C("track"), path.length);
		if (layout.kilometrage)
			ImGui::TextDisabled("km %.3f", station::km_of(layout, along));
		ImGui::TextDisabled("%s", STR_C("LMB: select it"));
		ImGui::EndTooltip();
	}
}

void srk_tool::render_windows()
{
	if (false == m_open || false == m_layout.has_value())
		return;
	auto const &io{ImGui::GetIO()};
	// along the bottom of the 3d view, as the vertical profile
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	auto const width{std::max(480.0f, high.x - low.x)};
	auto const height{std::clamp((high.y - low.y) * 0.4f, 220.0f, std::max(220.0f, high.y - low.y - 80.0f))};
	ImGui::SetNextWindowPos(ImVec2(low.x, high.y - height), ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Appearing);
	auto const name{std::string{STR_C("Station schematic")} + "  " + title() + "###srkstrip"};
	if (ImGui::Begin(name.c_str(), nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
	{
		auto const position{ImGui::GetWindowPos()};
		auto const size{ImGui::GetWindowSize()};
		if (false == ImGui::IsWindowDocked() && (position.y + size.y > io.DisplaySize.y + 1.0f || position.x + 80.0f > io.DisplaySize.x || position.x + size.x < 80.0f))
			ImGui::SetWindowPos(ImVec2(std::clamp(position.x, 0.0f, std::max(0.0f, io.DisplaySize.x - size.x)), std::max(0.0f, std::min(position.y, io.DisplaySize.y - size.y))));
		if (ImGui::Button(STR_C("Fit")))
			m_view_state.exaggeration = 0.0f;
		item_tooltip("Shows the whole station");
		ImGui::SameLine();
		ImGui::SetNextItemWidth(140.0f);
		ImGui::SliderFloat("##srkexaggeration", &m_view_state.exaggeration, 1.0f, 500.0f, STR_C("offsets x%.0f"), ImGuiSliderFlags_Logarithmic);
		item_tooltip("How much the offsets of the tracks are stretched against the lengths, also Ctrl+wheel over the plot");
		ImGui::SameLine();
		ImGui::TextDisabled("%s", STR_C("Wheel: zoom, drag: move, LMB on a track: select it"));
		render_canvas(ImGui::GetContentRegionAvail().y);
	}
	ImGui::End();
}

editor_selftest_result srk_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "srkopen")
	{
		// srkopen track=<name>: the station of the track, as a click on it opens it
		auto const values{selftest_values(Words)};
		auto const found{values.find("track")};
		auto *track{found != values.end() ? simulation::Paths.find(found->second) : nullptr};
		m_open = true;
		load();
		if (track == nullptr)
		{
			WriteLog("SELFTEST srkopen: no such track");
			return editor_selftest_result::handled;
		}
		open(*track);
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srkopen: " + m_status);
			return editor_selftest_result::handled;
		}
		auto const &layout{*m_layout};
		std::unordered_set<TTrack const *> tracks;
		std::size_t approach{0};
		for (auto const &path : layout.paths)
			if (path.approach)
				++approach;
			else
				tracks.insert(path.track);
		WriteLog(format("SELFTEST srkopen %s: %zu switches, %zu tracks, %zu paths out, axis %.0f m, switches over %.0f..%.0f, offsets %.1f..%.1f, km %s, record %d", m_anchor.c_str(), layout.switches.size(), tracks.size(), approach,
		                layout.chainage.back(), layout.from, layout.to, layout.low, layout.high, layout.kilometrage ? format("%.3f", layout.km).c_str() : "-", m_record));
		for (auto const *track : layout.switches)
		{
			auto const at{station::project(layout, track->m_paths.front().points[segment_data::point::start])};
			WriteLog(format("SELFTEST srkopen switch %s at %.1f offset %.1f", track->name().c_str(), at.x, at.y));
		}
		return editor_selftest_result::handled;
	}
	if (Command == "srkstations")
	{
		load();
		find_all();
		WriteLog(format("SELFTEST srkstations: %zu", m_found.size()));
		for (auto const &[name, count] : m_found)
			WriteLog(format("SELFTEST srkstations %s: %zu switches", name.c_str(), count));
		return editor_selftest_result::handled;
	}
	if (Command == "srkset")
	{
		// srkset code=<code> line=<line> name=<words joined by _>
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srkset: no station open");
			return editor_selftest_result::handled;
		}
		auto const values{selftest_values(Words)};
		auto &entry{record_of_open()};
		if (auto const code{values.find("code")}; code != values.end())
			entry.code = code->second;
		if (auto const line{values.find("line")}; line != values.end())
			entry.line = line->second;
		if (auto const name{values.find("name")}; name != values.end())
		{
			entry.name = name->second;
			std::replace(entry.name.begin(), entry.name.end(), '_', ' ');
		}
		m_changed = true;
		WriteLog("SELFTEST srkset: " + title());
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}
