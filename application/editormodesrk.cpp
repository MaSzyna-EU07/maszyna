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
#include "editor/editorTrack.hpp"

#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <functional>
#include <unordered_set>

namespace
{

char const *const kMark{"//$s"};
double constexpr kOverlayRange{4000.0};

ImU32 constexpr kPlain{IM_COL32(170, 176, 188, 255)};
ImU32 constexpr kSwitch{IM_COL32(240, 200, 90, 255)};
ImU32 constexpr kApproach{IM_COL32(95, 100, 112, 255)};
ImU32 constexpr kHovered{IM_COL32(255, 255, 255, 255)};
ImU32 constexpr kSignal{IM_COL32(240, 95, 85, 255)};

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

// the number of the switch of a unit: of its name once named after the station, of its record, else of its place
std::string unit_number(station::unit const &Unit, std::string const &Stem, std::size_t const Index)
{
	auto const &name{Unit.parts.front()->name()};
	if (name.size() > Stem.size() && name.compare(0, Stem.size(), Stem) == 0)
	{
		auto digits{name.substr(Stem.size())};
		while (false == digits.empty() && std::isalpha(static_cast<unsigned char>(digits.back())))
			digits.pop_back();
		if (false == digits.empty() && std::all_of(digits.begin(), digits.end(), [](char const Digit) { return std::isdigit(static_cast<unsigned char>(Digit)) != 0; }))
			return digits;
	}
	if (auto const record{switch_store::named(*Unit.parts.front())}; record && record->number != "-")
		return record->number;
	return std::to_string(Index + 1);
}

// the number of the station track a circuit is named after: bpd_it5b of track 5; empty for the other names, bpd_ita of the
// circuit before the entry signal A among them
std::string track_of(std::string const &Name, std::string const &Stem)
{
	auto const head{Stem + "it"};
	if (Stem.empty() || Name.size() <= head.size() || Name.compare(0, head.size(), head) != 0)
		return {};
	auto number{Name.substr(head.size())};
	if (std::none_of(number.begin(), number.end(), [](char const Letter) { return std::isdigit(static_cast<unsigned char>(Letter)) != 0; }))
		return {};
	if (number.size() > 1 && std::isalpha(static_cast<unsigned char>(number.back())) && std::isdigit(static_cast<unsigned char>(number[number.size() - 2])))
		number.pop_back();
	return number;
}

// the circuit between an entry signal and the station: bpd_ita
bool entry_circuit(std::string const &Name, std::string const &Stem)
{
	auto const head{Stem + "it"};
	return false == Stem.empty() && Name.size() > head.size() && Name.compare(0, head.size(), head) == 0 &&
	       std::all_of(Name.begin() + head.size(), Name.end(), [](char const Letter) { return std::isalpha(static_cast<unsigned char>(Letter)) != 0; });
}

// the name of a signal as its plate and the schematic show it: bpd_tm3 as Tm3, bpd_f1 as F1
std::string signal_shown(std::string const &Name, std::string const &Stem)
{
	auto text{false == Stem.empty() && Name.size() > Stem.size() && ToLower(Name).compare(0, Stem.size(), Stem) == 0 ? Name.substr(Stem.size()) : Name};
	std::transform(text.begin(), text.end(), text.begin(), [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
	if (text.size() > 2 && text[0] == 'T' && (text[1] == 'M' || text[1] == 'O'))
		text[1] = static_cast<char>(std::tolower(static_cast<unsigned char>(text[1])));
	return text;
}

// the way the trains a signal faces run, in the plan: it stands turned to them
glm::dvec2 travel_of(double const Yaw)
{
	auto const yaw{glm::radians(Yaw + 180.0)};
	return {std::sin(yaw), std::cos(yaw)};
}

// how far the point lies from the tracks of the station, in the plan
double gap_to(station::layout const &Layout, glm::dvec3 const &Point)
{
	auto best{std::numeric_limits<double>::max()};
	for (auto const &path : Layout.paths)
		for (auto const &point : path.points)
			best = std::min(best, glm::distance(glm::dvec2{point.x, point.z}, glm::dvec2{Point.x, Point.z}));
	return best;
}

ImU32 circuit_colour(std::string const &Name)
{
	static std::array<ImU32, 6> const palette{IM_COL32(110, 200, 250, 255), IM_COL32(250, 150, 90, 255), IM_COL32(150, 220, 120, 255), IM_COL32(230, 120, 200, 255), IM_COL32(240, 220, 100, 255), IM_COL32(150, 140, 250, 255)};
	return palette[std::hash<std::string>{}(Name) % palette.size()];
}

template <typename Record_> std::string anchor_of(Record_ const &Entry)
{
	return Entry.track != nullptr && false == Entry.track->m_editorremoved ? Entry.track->name() : Entry.anchor;
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
			entry.track = simulation::Paths.find(entry.anchor);
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
			lines.push_back(format("station %s line %s anchor %s name %s", word_of(entry.code).c_str(), word_of(entry.line).c_str(), anchor_of(entry).c_str(), entry.name.c_str()));
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
	{
		auto &entry{m_records[index]};
		if (entry.track == nullptr)
			entry.track = simulation::Paths.find(entry.anchor);
		if (std::find(m_layout->switches.begin(), m_layout->switches.end(), entry.track) != m_layout->switches.end())
			m_record = index;
	}
	m_anchor = m_record >= 0 ? m_records[m_record].track : m_layout->switches.front();
	survey();
}

void srk_tool::refresh()
{
	m_history_size = m_history.history_size();
	if (m_anchor == nullptr || m_anchor->m_editorremoved)
	{
		m_layout.reset();
		return;
	}
	auto const view{m_view_state};
	auto const status{m_status};
	open(*m_anchor);
	if (m_layout.has_value())
	{
		m_view_state = view;
		m_status = status;
	}
}

srk_tool::record &srk_tool::record_of_open()
{
	if (m_record < 0)
	{
		record entry;
		auto *first{m_layout->switches.front()};
		entry.anchor = first->name();
		entry.track = first;
		entry.layer = scene::Layers.resolve(first->layer());
		if (false == scene::Layers.valid(entry.layer) || false == scene::Layers.accepts(entry.layer))
			entry.layer = scene::Layers.active();
		if (false == scene::Layers.valid(entry.layer))
			entry.layer = 1;
		m_records.push_back(std::move(entry));
		m_record = static_cast<int>(m_records.size()) - 1;
		m_anchor = first;
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
	return format(STR_C("Station of %s"), m_anchor != nullptr ? m_anchor->name().c_str() : "");
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
	// the stations follow their anchors renamed
	for (auto &entry : m_records)
		if (auto const name{anchor_of(entry)}; name != entry.anchor)
		{
			entry.anchor = name;
			m_changed = true;
		}
	if ((m_layout.has_value() || m_anchor != nullptr) && m_history.history_size() != m_history_size)
		refresh();
	else if (m_layout.has_value() && false == m_signals_known)
		survey_signals();
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
	for (auto &entry : m_records)
		if (entry.track == Node)
			entry.track = nullptr;
	if (Node == m_anchor)
	{
		m_anchor = nullptr;
		if (m_layout.has_value())
			for (auto *track : m_layout->switches)
				if (track != Node)
				{
					m_anchor = track;
					break;
				}
	}
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
		auto const described{std::find_if(m_records.begin(), m_records.end(), [&](record const &Entry) { return anchor_of(Entry) == name; })};
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
	step_heading(1, STR_C("Switch numbers"));
	int from{m_reversed ? 1 : 0};
	if (segmented("##srkfrom", from,
	              {{0, STR_C("From the left"), layout.kilometrage ? "From the lower km, as PKP PLK numbers them" : "From the west end of the station"},
	               {1, STR_C("From the right"), layout.kilometrage ? "From the higher km" : "From the east end of the station"}}))
		m_reversed = from == 1;
	if (ImGui::Button(STR_C("Number the switches"), ImVec2(-1.0f, 0.0f)))
		number_switches();
	item_tooltip("Numbers the switches 1, 2, 3... along the station and names them after its code, as on the lines of PKP PLK: bpd_1, "
	             "the parts of a slip bpd_15a to bpd_15d. The drives follow their switches, events written by hand keep the old names. One step of undo");
	step_heading(2, STR_C("Track circuits"));
	if (main_button(STR_C("Set the track circuits"), ImVec2(-1.0f, tall_button_height())))
		set_circuits();
	item_tooltip("Cuts the tracks of the station at the joints of the track circuits and names the circuits, as on the lines of PKP PLK: "
	             "a joint 1.5 m before the start of a switch, beyond the fouling point of each branch where the axes are 4 m apart, in the middle "
	             "of a link too short for both. Each switch gets bpd_iz<number>, each station track bpd_it<number>: 1 and 2 the main tracks, "
	             "track 1 on the right along the km, the odd ones beyond it, the even ones on the other side. Run again after changes, it keeps "
	             "the numbers the tracks have. One step of undo");
	step_heading(3, STR_C("Signals"));
	if (ImGui::Button(STR_C("Set the signals"), ImVec2(-1.0f, 0.0f)))
		set_signals();
	item_tooltip("Places the signals of the station, as on the lines of PKP PLK, and names them after its code: an entry signal 300 m out on "
	             "each line, A, B... from the start of the km, Z, Y... from the end, the circuit from it to the station bpd_ita; an exit "
	             "signal at the end of each station track a train is sent out of, F1, G2 towards the start, R1, P2 towards the end; a "
	             "shunting signal Tm1, Tm2... at the end of a shorter track. A signal standing by the place of one gets its name, none is "
	             "doubled. The kinds of signals chosen in the Signals tool. One step of undo");
	if (auto const *selected{m_selection.selected_track()}; selected != nullptr && false == selected->Isolated.empty())
		if (auto const number{track_of(selected->Isolated.front()->asName, stem_of_open())}; false == number.empty() && begin_fields("##srktrack"))
		{
			field_row("Track number");
			std::array<char, 16> buffer{};
			number.copy(buffer.data(), buffer.size() - 1);
			if (ImGui::InputText("##srktracknumber", buffer.data(), buffer.size(), ImGuiInputTextFlags_EnterReturnsTrue) || ImGui::IsItemDeactivatedAfterEdit())
				renumber_track(*selected, buffer.data());
			item_tooltip("The number of the selected station track: type another one and its track circuits are renamed, bpd_it7a, bpd_it7b. One step of undo");
			ImGui::EndTable();
		}
	if (false == m_status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", m_status.c_str());
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
	if (ImGui::Button(STR_C("Another station"), ImVec2(-1.0f, 0.0f)))
	{
		m_layout.reset();
		m_anchor = nullptr;
		m_record = -1;
		m_hover = -1;
	}
	item_tooltip("Leaves this station; LMB on a switch opens another one");
}

void srk_tool::number_switches()
{
	m_status.clear();
	auto const code{m_record >= 0 ? m_records[m_record].code : std::string{}};
	if (code.empty())
	{
		m_status = STR_C("Give the station its code first: the switches are named after it");
		return;
	}
	auto const units{station::numbering(*m_layout, m_reversed)};
	std::vector<std::pair<TTrack *, std::string>> names;
	std::unordered_set<TTrack const *> own;
	for (std::size_t index = 0; index < units.size(); ++index)
		for (std::size_t part = 0; part < units[index].parts.size(); ++part)
		{
			names.emplace_back(units[index].parts[part], station::switch_name(code, static_cast<int>(index) + 1, part, units[index].parts.size()));
			own.insert(units[index].parts[part]);
		}
	for (auto &[track, name] : names)
	{
		std::string reason;
		TTrack *owner{nullptr};
		if (false == scene::Layers.editable(track, &reason))
		{
			m_status = format(STR_C("%s can't be renamed: %s"), track->name().c_str(), reason.c_str());
			return;
		}
		if (false == editor_track::name_valid(*track, name, reason, &owner) && (owner == nullptr || own.count(owner) == 0))
		{
			m_status = format(STR_C("%s can't be named %s: %s"), track->name().c_str(), name.c_str(), reason.c_str());
			return;
		}
	}
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::map<std::string, std::string> parts;
	std::map<std::string, std::string> bases;
	for (auto const &[track, name] : names)
		if (track->name() != name)
		{
			before.emplace_back(track, editor_track::capture(*track));
			parts[track->name()] = name;
		}
	for (std::size_t index = 0; index < units.size(); ++index)
		if (false == units[index].record.empty())
			bases[units[index].record] = station::switch_name(code, static_cast<int>(index) + 1, 0, 1);
	switch_store::rename(parts, bases);
	// the names go round among the switches: a switch whose name another one gets gives it up first
	for (auto const &entry : names)
		if (entry.first->name() != entry.second && std::any_of(names.begin(), names.end(), [&](auto const &Other) { return Other.first != entry.first && Other.second == entry.first->name(); }))
			editor_track::rename(*entry.first, entry.first->name() + "~");
	for (auto const &[track, name] : names)
		editor_track::rename(*track, name);
	std::size_t numbered{0};
	for (std::size_t index = 0; index < units.size(); ++index)
		if (auto const record{switch_store::named(*units[index].parts.front())}; record && record->number != std::to_string(index + 1))
		{
			switch_store::number(record->name, std::to_string(index + 1));
			++numbered;
		}
	if (before.empty() && numbered == 0)
	{
		m_status = STR_C("The switches have these numbers already");
		return;
	}
	if (before.empty())
		before.emplace_back(units.front().parts.front(), editor_track::capture(*units.front().parts.front()));
	m_history.push_track_snapshot(std::move(before));
	m_status = format(STR_P("%zu switch numbered", "%zu switches numbered", units.size()), units.size());
}

void srk_tool::set_circuits()
{
	m_status.clear();
	auto const code{m_record >= 0 ? m_records[m_record].code : std::string{}};
	if (code.empty())
	{
		m_status = STR_C("Give the station its code first: the track circuits are named after it");
		return;
	}
	auto const units{station::numbering(*m_layout, m_reversed)};
	auto const planned{station::joints(*m_layout, units)};
	for (auto const &entry : planned)
		if (std::string reason; entry.track != nullptr && false == editor_track::can_edit_geometry(*entry.track, reason))
		{
			m_status = format(STR_C("%s can't be cut: %s"), name_of(*entry.track).c_str(), reason.c_str());
			return;
		}
	for (auto const &path : m_layout->paths)
		if (std::string reason; false == path.approach && false == scene::Layers.editable(path.track, &reason))
		{
			m_status = format(STR_C("%s can't be changed: %s"), name_of(*path.track).c_str(), reason.c_str());
			return;
		}
	// the cuts: each track from its end back, so the pieces left to cut stay in the track
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::vector<TTrack *> created;
	std::vector<glm::dvec3> points;
	std::vector<std::pair<TTrack *, std::vector<glm::dvec3>>> cuts;
	for (auto const &entry : planned)
	{
		points.push_back(entry.point);
		if (entry.track == nullptr)
			continue;
		auto found{std::find_if(cuts.begin(), cuts.end(), [&](auto const &Cut) { return Cut.first == entry.track; })};
		if (found == cuts.end())
			found = cuts.insert(cuts.end(), {entry.track, {}});
		found->second.push_back(entry.point);
	}
	for (auto &[track, at] : cuts)
	{
		before.emplace_back(track, editor_track::capture(*track));
		auto *const piece{track};
		std::sort(at.begin(), at.end(), [&](glm::dvec3 const &A, glm::dvec3 const &B) { return editor_track::nearest_parameter(*piece, A) > editor_track::nearest_parameter(*piece, B); });
		for (auto const &point : at)
			if (auto *cut{editor_track::split_path(*track, editor_track::nearest_parameter(*track, point))})
				created.push_back(cut);
	}
	// the circuits of the station as cut
	std::size_t count{0};
	if (auto const found{station::find(*m_anchor)})
	{
		auto const circuits{station::circuits(*found, units, points)};
		auto const numbers{station::track_numbers(*found, circuits)};
		auto const stem{station::prefix(code)};
		std::unordered_set<TTrack const *> captured(created.begin(), created.end());
		for (auto const &[track, state] : before)
			captured.insert(track);
		// a station track keeps the name most of it has, given by hand or so far, unless another one has it too; the others take
		// the numbers left on their side
		std::vector<std::string> names(circuits.size());
		std::map<std::string, std::size_t> held;
		for (std::size_t index = 0; index < circuits.size(); ++index)
		{
			auto const &circuit{circuits[index]};
			if (circuit.unit >= 0)
				continue;
			std::map<std::string, std::size_t> counts;
			for (auto const *track : circuit.tracks)
				if (track->Isolated.size() == 1 && false == track_of(track->Isolated.front()->asName, stem).empty())
					++counts[track->Isolated.front()->asName];
			if (auto const most{std::max_element(counts.begin(), counts.end(), [](auto const &A, auto const &B) { return A.second < B.second; })}; most != counts.end())
				++held[names[index] = most->first];
		}
		std::unordered_set<std::string> taken;
		for (auto &name : names)
			if (false == name.empty() && held[name] > 1)
				name.clear();
			else if (false == name.empty())
				taken.insert(name);
		for (std::size_t index = 0; index < circuits.size(); ++index)
		{
			auto const &circuit{circuits[index]};
			auto &name{names[index]};
			if (circuit.unit >= 0)
				name = stem + "iz" + unit_number(units[circuit.unit], stem, circuit.unit);
			else if (name.empty())
			{
				auto number{numbers[index]};
				while (taken.count(stem + "it" + number) > 0)
				{
					std::size_t digits{0};
					while (digits < number.size() && std::isdigit(static_cast<unsigned char>(number[digits])))
						++digits;
					if (digits == 0)
						break;
					number = std::to_string(std::stoi(number.substr(0, digits)) + 2) + number.substr(digits);
				}
				taken.insert(name = stem + "it" + number);
			}
			for (auto *track : circuit.tracks)
			{
				if (track->Isolated.size() == 1 && track->Isolated.front()->asName == name)
					continue;
				if (captured.insert(track).second)
					before.emplace_back(track, editor_track::capture(*track));
				editor_track::isolated(*track, {name});
			}
		}
		count = circuits.size();
	}
	if (before.empty() && created.empty())
	{
		m_status = STR_C("The track circuits are set already");
		return;
	}
	m_history.push_track_snapshot(std::move(before), created);
	m_status = format(STR_P("%zu track circuit", "%zu track circuits", count), count) + ", " + format(STR_P("%zu cut", "%zu cuts", created.size()), created.size());
}

std::string srk_tool::stem_of_open() const
{
	return m_record >= 0 && false == m_records[m_record].code.empty() ? station::prefix(m_records[m_record].code) : std::string{};
}

void srk_tool::renumber_track(TTrack const &Track, std::string const &Number)
{
	m_status.clear();
	auto const stem{stem_of_open()};
	auto const from{Track.Isolated.empty() ? std::string{} : track_of(Track.Isolated.front()->asName, stem)};
	std::string to;
	for (auto const letter : Number)
		if (std::isalnum(static_cast<unsigned char>(letter)))
			to += static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
	if (from.empty() || to.empty() || to == from)
		return;
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::unordered_set<TTrack const *> seen;
	for (auto const &path : m_layout->paths)
	{
		if (path.track->Isolated.empty() || false == seen.insert(path.track).second)
			continue;
		auto const number{track_of(path.track->Isolated.front()->asName, stem)};
		if (number == to)
		{
			m_status = format(STR_C("The station has a track %s already"), to.c_str());
			return;
		}
		if (number == from)
			before.emplace_back(path.track, editor_track::capture(*path.track));
	}
	for (auto &[track, state] : before)
	{
		auto const &name{track->Isolated.front()->asName};
		editor_track::isolated(*track, {stem + "it" + to + name.substr(stem.size() + 2 + from.size())});
	}
	if (before.empty())
		return;
	m_history.push_track_snapshot(std::move(before));
	m_status = format(STR_C("Track %s is track %s now"), from.c_str(), to.c_str());
}

void srk_tool::set_signals()
{
	m_status.clear();
	auto const stem{stem_of_open()};
	if (stem.empty())
	{
		m_status = STR_C("Give the station its code first: the signals are named after it");
		return;
	}
	auto const &layout{*m_layout};
	// the station tracks by their numbers, the circuits as the tracks have them
	std::map<std::string, std::map<std::string, std::vector<TTrack *>>> numbers;
	std::unordered_set<TTrack const *> inside;
	for (auto const &path : layout.paths)
	{
		if (path.track->Isolated.empty() || inside.count(path.track) > 0)
			continue;
		auto const &name{path.track->Isolated.front()->asName};
		if (name.size() <= stem.size() || name.compare(0, stem.size(), stem) != 0 || entry_circuit(name, stem))
			continue;
		inside.insert(path.track);
		if (auto const number{track_of(name, stem)}; false == number.empty())
			numbers[number][name].push_back(path.track);
	}
	if (numbers.empty())
	{
		m_status = STR_C("Set the track circuits first: the signals stand at their joints");
		return;
	}
	std::vector<station::numbered> tracks;
	for (auto &[number, circuits] : numbers)
	{
		station::numbered entry;
		entry.number = number;
		for (auto &[name, members] : circuits)
			entry.circuits.push_back(std::move(members));
		tracks.push_back(std::move(entry));
	}
	auto plan{station::signals(layout, tracks, [&](TTrack const &Track) { return inside.count(&Track) > 0; })};
	std::vector<standing_signal> standing;
	m_signals.standing_signals(standing, true);
	standing.erase(std::remove_if(standing.begin(), standing.end(), [&](standing_signal const &Signal) { return gap_to(layout, Signal.position) > 8.0; }), standing.end());
	auto const full = [&](station::signal const &Signal) { return stem + ToLower(Signal.name); };
	// the standing ones by their places, then by their names the ones moved away
	std::vector<int> match(plan.size(), -1);
	std::vector<char> used(standing.size(), 0);
	std::vector<std::tuple<double, std::size_t, std::size_t>> candidates;
	for (std::size_t i = 0; i < plan.size(); ++i)
	{
		auto const &planned{plan[i]};
		glm::dvec2 const travel{glm::normalize(glm::dvec2{planned.travel.x, planned.travel.z} + glm::dvec2{1e-12, 0.0})};
		auto const reach{planned.kind == station::signal::role::entry ? 400.0 : planned.kind == station::signal::role::exit ? 60.0 : 40.0};
		for (std::size_t k = 0; k < standing.size(); ++k)
		{
			auto const &signal{standing[k]};
			if ((signal.kind == "shunt") != (planned.kind == station::signal::role::shunting) || (signal.kind != "shunt" && signal.kind != "main") || glm::dot(travel_of(signal.yaw), travel) < 0.87)
				continue;
			glm::dvec2 const offset{signal.position.x - planned.point.x, signal.position.z - planned.point.z};
			auto const along{glm::dot(offset, travel)};
			auto const across{std::abs(offset.x * travel.y - offset.y * travel.x)};
			if (std::abs(along) < reach && across < 3.0)
				candidates.emplace_back(std::abs(along) + across * 10.0, i, k);
		}
	}
	std::sort(candidates.begin(), candidates.end());
	for (auto const &[score, i, k] : candidates)
		if (match[i] < 0 && 0 == used[k])
		{
			match[i] = static_cast<int>(k);
			used[k] = 1;
		}
	for (std::size_t i = 0; i < plan.size(); ++i)
		for (std::size_t k = 0; k < standing.size() && match[i] < 0; ++k)
			if (0 == used[k] && ToLower(standing[k].name) == full(plan[i]))
			{
				match[i] = static_cast<int>(k);
				used[k] = 1;
			}
	// the circuits from the entry signals to the station: the line cut by each one, all of it checked before any is
	struct cut
	{
		std::size_t signal{0};
		TTrack *track{nullptr};
		double parameter{0.0};
		glm::dvec3 point{0.0};
		bool outward{false};
		std::vector<TTrack *> named; // between the station and the cut
	};
	std::vector<cut> cuts;
	for (std::size_t i = 0; i < plan.size(); ++i)
	{
		auto const &planned{plan[i]};
		if (planned.kind != station::signal::role::entry || planned.run.empty())
			continue;
		auto const at{match[i] >= 0 ? standing[match[i]].position : planned.point};
		cut entry;
		entry.signal = i;
		auto best{std::numeric_limits<double>::max()};
		std::size_t nearest{0};
		for (std::size_t k = 0; k < planned.run.size(); ++k)
		{
			auto const t{editor_track::nearest_parameter(*planned.run[k].first, at)};
			auto const point{editor_track::point_at(*planned.run[k].first, t)};
			if (auto const distance{glm::distance(glm::dvec2{point.x, point.z}, glm::dvec2{at.x, at.z})}; distance < best)
			{
				best = distance;
				nearest = k;
				entry.parameter = t;
				entry.point = point;
			}
		}
		for (std::size_t k = 0; k < nearest; ++k)
			entry.named.push_back(planned.run[k].first);
		entry.track = planned.run[nearest].first;
		entry.outward = planned.run[nearest].second;
		auto const &ends{entry.track->m_paths.front().points};
		auto const to_start{glm::distance(entry.point, ends[segment_data::point::start])};
		auto const to_end{glm::distance(entry.point, ends[segment_data::point::end])};
		std::string reason;
		if (std::min(to_start, to_end) <= 0.3)
		{
			// at an end already
			if ((to_end < to_start) == entry.outward)
				entry.named.push_back(entry.track);
			entry.track = nullptr;
		}
		else if (false == editor_track::can_edit_geometry(*entry.track, reason))
		{
			m_status = format(STR_C("%s can't be cut: %s"), name_of(*entry.track).c_str(), reason.c_str());
			return;
		}
		for (auto *part : entry.named)
			if (false == scene::Layers.editable(part, &reason))
			{
				m_status = format(STR_C("%s can't be changed: %s"), name_of(*part).c_str(), reason.c_str());
				return;
			}
		cuts.push_back(std::move(entry));
	}
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::vector<TTrack *> created;
	std::unordered_set<TTrack const *> captured;
	for (auto &entry : cuts)
	{
		auto &planned{plan[entry.signal]};
		if (entry.track != nullptr)
		{
			captured.insert(entry.track);
			before.emplace_back(entry.track, editor_track::capture(*entry.track));
			if (auto *piece{editor_track::split_path(*entry.track, entry.parameter)})
			{
				created.push_back(piece);
				captured.insert(piece);
				entry.named.push_back(entry.outward ? entry.track : piece);
				planned.track = entry.outward ? piece : entry.track;
				planned.point = entry.point;
			}
		}
		auto const name{stem + "it" + ToLower(planned.name)};
		for (auto *part : entry.named)
		{
			if (part->Isolated.size() == 1 && part->Isolated.front()->asName == name)
				continue;
			if (captured.insert(part).second)
				before.emplace_back(part, editor_track::capture(*part));
			editor_track::isolated(*part, {name});
		}
	}
	std::vector<signal_order> placed;
	std::vector<signal_rename> renamed;
	for (std::size_t i = 0; i < plan.size(); ++i)
	{
		auto const &planned{plan[i]};
		auto const plate{ToLower(planned.name)};
		if (match[i] >= 0)
		{
			if (ToLower(standing[match[i]].name) != full(planned))
				renamed.push_back({standing[match[i]].instance, full(planned), plate});
			continue;
		}
		signal_order order;
		order.track = planned.track;
		order.point = planned.point;
		order.facing = planned.facing;
		order.side = planned.side;
		order.kind = planned.kind == station::signal::role::shunting ? "shunt" : "main";
		order.name = full(planned);
		order.plate = plate;
		placed.push_back(std::move(order));
	}
	// the others keep their names, unless one of these takes it
	for (std::size_t k = 0; k < standing.size(); ++k)
		if (0 == used[k] && std::any_of(plan.begin(), plan.end(), [&](station::signal const &Signal) { return full(Signal) == ToLower(standing[k].name); }))
			renamed.push_back({standing[k].instance, ToLower(standing[k].name) + "_old", signal_shown(standing[k].name, stem) + "_old"});
	if (placed.empty() && renamed.empty() && before.empty() && created.empty())
	{
		m_status = STR_C("The signals are set already");
		return;
	}
	std::string error;
	auto const count{m_signals.signal_batch(placed, renamed, std::move(before), std::move(created), error)};
	if (false == error.empty())
		m_status = format(STR_C("The signals couldn't be placed: %s"), error.c_str());
	else
		m_status = format(STR_P("%zu signal placed", "%zu signals placed", count), count) + ", " + format(STR_P("%zu renamed", "%zu renamed", renamed.size()), renamed.size());
}

void srk_tool::survey_signals()
{
	m_signal_marks.clear();
	std::vector<standing_signal> standing;
	m_signals_known = m_layout.has_value() && m_signals.standing_signals(standing, false);
	if (false == m_signals_known)
		return;
	auto const stem{stem_of_open()};
	for (auto const &signal : standing)
	{
		if (gap_to(*m_layout, signal.position) > 8.0)
			continue;
		auto const travel{travel_of(signal.yaw)};
		auto const at{station::project(*m_layout, signal.position)};
		auto const ahead{station::project(*m_layout, signal.position + glm::dvec3{travel.x, 0.0, travel.y} * 5.0)};
		m_signal_marks.push_back({at, ahead.x > at.x, signal_shown(signal.name, stem)});
	}
}

void srk_tool::survey()
{
	m_circuit_of.assign(m_layout.has_value() ? m_layout->paths.size() : 0, -1);
	m_circuits.clear();
	m_ticks.clear();
	if (false == m_layout.has_value())
		return;
	auto const &layout{*m_layout};
	auto const stem{stem_of_open()};
	std::vector<double> weights;
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
	{
		auto const &path{layout.paths[index]};
		if (path.track->Isolated.empty())
			continue;
		// of the lines leading out only the stretches up to the joints of the station
		auto const &name{path.track->Isolated.front()->asName};
		if (path.approach && (stem.empty() || name.compare(0, stem.size(), stem) != 0))
			continue;
		auto found{std::find_if(m_circuits.begin(), m_circuits.end(), [&](circuit_mark const &Mark) { return Mark.text == name; })};
		if (found == m_circuits.end())
		{
			found = m_circuits.insert(m_circuits.end(), {glm::dvec2{0.0}, name, circuit_colour(name)});
			found->labelled = stem.empty() || name.compare(0, stem.size() + 2, stem + "iz") != 0;
			weights.push_back(0.0);
		}
		auto const circuit{static_cast<std::size_t>(found - m_circuits.begin())};
		m_circuit_of[index] = static_cast<int>(circuit);
		for (auto const &point : path.schematic)
		{
			found->at += point;
			weights[circuit] += 1.0;
		}
		found->length += path.length;
	}
	for (std::size_t index = 0; index < m_circuits.size(); ++index)
	{
		auto &mark{m_circuits[index]};
		mark.at /= std::max(1.0, weights[index]);
		// on its track, nearest the middle of its points
		auto nearest{std::numeric_limits<double>::max()};
		glm::dvec2 on{mark.at};
		for (std::size_t path = 0; path < layout.paths.size(); ++path)
			if (m_circuit_of[path] == static_cast<int>(index))
				for (auto const &point : layout.paths[path].schematic)
					if (auto const distance{glm::distance(point, mark.at)}; distance < nearest)
					{
						nearest = distance;
						on = point;
					}
		mark.at = on;
		auto const entry{entry_circuit(mark.text, stem)};
		// bpd_it3 shown as It3
		if (false == stem.empty() && mark.text.compare(0, stem.size(), stem) == 0)
			mark.text.erase(0, stem.size());
		if (false == mark.text.empty())
			mark.text.front() = static_cast<char>(std::toupper(static_cast<unsigned char>(mark.text.front())));
		// bpd_ita, before the entry signal A, as ItA
		if (entry && mark.text.size() > 2)
			std::transform(mark.text.begin() + 2, mark.text.end(), mark.text.begin() + 2, [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
	}
	// the joints: the ends where paths of different circuits meet
	std::vector<std::pair<glm::dvec3, int>> ends;
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
	{
		auto const &path{layout.paths[index]};
		if (path.approach && m_circuit_of[index] < 0)
		{
			for (auto const &point : {path.points.front(), path.points.back()})
				ends.emplace_back(point, -1);
			continue;
		}
		for (auto const &point : {path.points.front(), path.points.back()})
			ends.emplace_back(point, m_circuit_of[index]);
	}
	for (std::size_t i = 0; i < ends.size(); ++i)
		for (std::size_t j = i + 1; j < ends.size(); ++j)
			if (ends[i].second != ends[j].second && (ends[i].second >= 0 || ends[j].second >= 0) && glm::distance(ends[i].first, ends[j].first) < 0.05)
				m_ticks.push_back(station::project(layout, ends[i].first));
	survey_signals();
}

void srk_tool::fit_view(float const Width, float const Height)
{
	auto const &layout{*m_layout};
	auto &view{m_view_state};
	// the switches and the signals out on the lines
	auto from{layout.from};
	auto to{layout.to};
	for (auto const &mark : m_signal_marks)
	{
		from = std::min(from, mark.at.x);
		to = std::max(to, mark.at.x);
	}
	auto const span{std::max(50.0, to - from)};
	view.from = from - span * 0.04 - 30.0;
	view.to = to + span * 0.04 + 30.0;
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
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
		if (auto const &path{layout.paths[index]}; false == path.approach || m_circuit_of[index] >= 0)
			draw_path(path, m_circuit_of[index] >= 0 ? m_circuits[m_circuit_of[index]].colour : is_switch(*path.track) ? kSwitch : kPlain, 2.5f);
	for (auto const &tick : m_ticks)
	{
		auto const at{screen_of(tick)};
		draw->AddLine(ImVec2(at.x, at.y - 5.0f), ImVec2(at.x, at.y + 5.0f), IM_COL32(255, 255, 255, 220), 2.0f);
	}
	for (auto const &path : layout.paths)
		if (path.track == selected)
			draw_path(path, overlay_color::selected, 4.0f);
	if (m_hover >= 0)
		draw_path(layout.paths[m_hover], kHovered, 4.0f);
	// the labels, none over another: the switches first, a dot at the start and the name over or under it in turn, the number
	// alone once named after the station; then the station tracks under or over them, the longest first
	std::vector<ImVec4> placed;
	auto const label = [&](std::string const &Text, ImU32 const Colour, std::initializer_list<ImVec2> Places) {
		auto const extent{ImGui::CalcTextSize(Text.c_str())};
		for (auto const &place : Places)
		{
			ImVec4 const box{place.x - extent.x * 0.5f - 2.0f, place.y - 1.0f, place.x + extent.x * 0.5f + 2.0f, place.y + extent.y + 1.0f};
			if (std::any_of(placed.begin(), placed.end(), [&](ImVec4 const &Other) { return box.x < Other.z && Other.x < box.z && box.y < Other.w && Other.y < box.w; }))
				continue;
			placed.push_back(box);
			draw->AddRectFilled(ImVec2(box.x, box.y), ImVec2(box.z, box.w), IM_COL32(22, 24, 28, 200), 2.0f);
			draw->AddText(ImVec2(place.x - extent.x * 0.5f, place.y), Colour, Text.c_str());
			return;
		}
	};
	auto const stem{stem_of_open()};
	auto const height{ImGui::GetTextLineHeight()};
	for (std::size_t index = 0; index < layout.switches.size(); ++index)
	{
		auto const *track{layout.switches[index]};
		auto const at{screen_of(station::project(layout, track->m_paths.front().points[segment_data::point::start]))};
		draw->AddCircleFilled(at, 3.5f, kSwitch);
		auto text{track->name()};
		if (false == stem.empty() && text.size() > stem.size() && text.compare(0, stem.size(), stem) == 0)
			text.erase(0, stem.size());
		ImVec2 const over{at.x, at.y - height - 6.0f};
		ImVec2 const under{at.x, at.y + 6.0f};
		if (index % 2 == 0)
			label(text, kSwitch, {over, under});
		else
			label(text, kSwitch, {under, over});
	}
	// the signals: a flag pointing the way of the trains they face
	for (auto const &mark : m_signal_marks)
	{
		auto const at{screen_of(mark.at)};
		auto const way{mark.ahead ? 1.0f : -1.0f};
		draw->AddTriangleFilled(ImVec2(at.x - 4.0f * way, at.y - 4.0f), ImVec2(at.x - 4.0f * way, at.y + 4.0f), ImVec2(at.x + 4.0f * way, at.y), kSignal);
		label(mark.text, kSignal, {ImVec2(at.x, at.y - height - 5.0f), ImVec2(at.x, at.y + 5.0f)});
	}
	std::vector<std::size_t> order;
	for (std::size_t index = 0; index < m_circuits.size(); ++index)
		if (m_circuits[index].labelled)
			order.push_back(index);
	std::sort(order.begin(), order.end(), [&](std::size_t A, std::size_t B) { return m_circuits[A].length > m_circuits[B].length; });
	for (auto const index : order)
	{
		auto const &mark{m_circuits[index]};
		auto const at{screen_of(mark.at)};
		label(mark.text, mark.colour, {ImVec2(at.x, at.y + 4.0f), ImVec2(at.x, at.y - height - 4.0f)});
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
		if (false == path.track->Isolated.empty())
			ImGui::TextDisabled(STR_C("track circuit %s"), path.track->Isolated.front()->asName.c_str());
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
		WriteLog(format("SELFTEST srkopen %s: %zu switches, %zu tracks, %zu paths out, axis %.0f m, switches over %.0f..%.0f, offsets %.1f..%.1f, km %s, record %d", m_anchor->name().c_str(), layout.switches.size(), tracks.size(), approach,
		                layout.chainage.back(), layout.from, layout.to, layout.low, layout.high, layout.kilometrage ? format("%.3f", layout.km).c_str() : "-", m_record));
		WriteLog(format("SELFTEST srkopen lines %zu, main %zu tracks, second %zu tracks", layout.lines.size(), layout.main.size(), layout.second.size()));
		for (auto const *track : layout.switches)
		{
			auto const at{station::project(layout, track->m_paths.front().points[segment_data::point::start])};
			WriteLog(format("SELFTEST srkopen switch %s at %.1f offset %.1f", track->name().c_str(), at.x, at.y));
		}
		return editor_selftest_result::handled;
	}
	if (Command == "srknumber")
	{
		// srknumber [from=right]: the switches of the open station numbered
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srknumber: no station open");
			return editor_selftest_result::handled;
		}
		auto const values{selftest_values(Words)};
		auto const from{values.find("from")};
		m_reversed = from != values.end() && from->second == "right";
		number_switches();
		std::string list;
		for (auto const &unit : station::numbering(*m_layout, m_reversed))
			for (auto const *part : unit.parts)
			{
				auto const record{switch_store::named(*part)};
				list += ' ' + part->name() + '/' + (record ? record->number : std::string{"?"});
			}
		WriteLog("SELFTEST srknumber: " + m_status);
		WriteLog("SELFTEST srknumber names:" + list);
		return editor_selftest_result::handled;
	}
	if (Command == "srkcircuits")
	{
		// srkcircuits: the open station cut into its track circuits
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srkcircuits: no station open");
			return editor_selftest_result::handled;
		}
		set_circuits();
		WriteLog("SELFTEST srkcircuits: " + m_status);
		refresh();
		std::map<std::string, std::pair<std::size_t, double>> found;
		for (auto const &path : m_layout->paths)
			if (false == path.track->Isolated.empty())
			{
				auto &entry{found[path.track->Isolated.front()->asName]};
				++entry.first;
				entry.second += path.length;
			}
		std::string list;
		for (auto const &[name, entry] : found)
			list += format(" %s:%zu/%.0f", name.c_str(), entry.first, entry.second);
		WriteLog(format("SELFTEST srkcircuits %zu named, %zu joints:", found.size(), m_ticks.size()) + list);
		return editor_selftest_result::handled;
	}
	if (Command == "srktrack")
	{
		// srktrack circuit=<name> number=<number>: the station track of the circuit renumbered
		auto const values{selftest_values(Words)};
		auto const circuit{values.find("circuit")};
		auto const number{values.find("number")};
		TTrack const *found{nullptr};
		if (m_layout.has_value() && circuit != values.end())
			for (auto const &path : m_layout->paths)
				if (false == path.track->Isolated.empty() && path.track->Isolated.front()->asName == circuit->second)
					found = path.track;
		if (found == nullptr || number == values.end())
		{
			WriteLog("SELFTEST srktrack: no station open, or no such circuit");
			return editor_selftest_result::handled;
		}
		renumber_track(*found, number->second);
		WriteLog("SELFTEST srktrack: " + m_status);
		return editor_selftest_result::handled;
	}
	if (Command == "srksignals")
	{
		// srksignals: the signals of the open station set
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srksignals: no station open");
			return editor_selftest_result::handled;
		}
		set_signals();
		WriteLog("SELFTEST srksignals: " + m_status);
		refresh();
		std::string list;
		for (auto const &mark : m_signal_marks)
			list += format(" %s@%.0f/%.1f%s", mark.text.c_str(), mark.at.x, mark.at.y, mark.ahead ? ">" : "<");
		WriteLog(format("SELFTEST srksignals %zu:", m_signal_marks.size()) + list);
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
