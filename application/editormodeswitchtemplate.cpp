/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the templates of the switches: the one a switch was laid of, another one given to it in place, and the choice of them as the
// switch tool and the inspector offer it, by the speed on the diverging track, then by the skos for the rail of the track

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorSwitchStore.hpp"
#include "simulation/simulation.h"
#include "utilities/translation.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <map>
#include <sstream>
#include <string>

namespace
{

using geometry::has_controls;
using geometry::plan_of;

double speed_of(editor_track::switch_template const &Template)
{
	return Template.radius > 0.0 ? editor_track::diverging_speed(Template.radius) : 0.0;
}

// a slip or a crossing made for the tracks crossing at the angle, rad: to within half of what keeps 1:9.51 and 1:10 of the catalogue
// apart, as slip_skew takes it
bool of_skos(editor_track::switch_template const &Template, double const Angle)
{
	return Template.double_slip && Template.ratio > 0.0 && std::abs(std::atan(1.0 / Template.ratio) - Angle) <= glm::radians(0.14);
}

double angle_of(switch_store::record const &Record)
{
	return Record.ratio > 0.0 ? std::atan(1.0 / Record.ratio) : 0.0;
}

// the words of the title of a template of the catalogue: its type, "rail-R-1:n", what tells it apart from the others of the size
struct title_words
{
	std::string type;
	std::string size;
	std::string rest;
};

title_words words_of(editor_track::switch_template const &Template)
{
	std::istringstream text(Template.title());
	title_words words;
	text >> words.type >> words.size;
	for (std::string word; text >> word;)
		words.rest += (words.rest.empty() ? "" : " ") + word;
	return words;
}

// the skos of the size, as the catalogue writes it: 1:9, 1:18,5; of the first switch of a double one
std::string skos_of(editor_track::switch_template const &Template)
{
	auto const size{words_of(Template).size};
	auto const at{size.find("1:")};
	if (at == std::string::npos)
		return format("1:%.4g", Template.ratio);
	return size.substr(at, size.find('/', at) - at);
}

// the skos the speed is usually laid with
double usual_ratio(double const Speed)
{
	return Speed <= 50.0 ? 9.0 : Speed <= 60.0 ? 12.0 : Speed <= 80.0 ? 14.0 : Speed <= 100.0 ? 18.5 : 26.5;
}

// the name in a list of the templates of one rail and skos: the radius and the speed, the type where it isn't Rz, what tells it
// apart from the others of the size
std::string row_name(editor_track::switch_template const &Template)
{
	if (Template.source != nullptr)
		return Template.title() + "  (" + std::to_string(Template.count) + "x)";
	if (Template.own)
		return Template.title() + (Template.radius > 0.0 ? format("  R %.0f, %.0f km/h", Template.radius, speed_of(Template)) : std::string{});
	auto const words{words_of(Template)};
	std::string name;
	if (words.type != "Rz")
		name = words.type + "  ";
	if (Template.twofold())
		if (auto const at{words.size.find("1:")}; at != std::string::npos)
			name += words.size.substr(at) + "  ";
	if (Template.radius > 0.0)
		name += format("R %.0f, %.0f km/h", Template.radius, speed_of(Template));
	else if (Template.double_slip)
		name += format("a %.3f m", Template.a);
	if (false == words.rest.empty())
		name += (name.empty() ? "" : "  ") + switch_name_translated(words.rest);
	return name.empty() ? switch_name_translated(Template.title()) : name;
}

std::string template_tooltip(editor_track::switch_template const &Template)
{
	if (Template.source != nullptr)
		return Template.label;
	auto text{switch_name_translated(Template.title())};
	if (Template.double_slip)
	{
		text += "\n";
		text += Template.type == "Rkp" ? STR_C("Single slip (Rkp)") : Template.type == "Rkpd" ? STR_C("Double slip (Rkpd)") : STR_C("Crossing (Skrz)");
		text += "\n" + format(STR_C("a %.3f m from the middle of the crossing to the joint"), Template.a) + "\n";
		if (Template.type == "Skrz")
			text += STR_C("Drag at a crossing of two straights");
		else
			text += std::string{STR_C("Drag it out on one straight: its crossing track comes along at its skos, through the quarter of the cursor")} + "\n" +
			        (Template.type == "Rkp" ? STR_C("Or drag at a crossing of two straights at its skos, on the side of the tracks to join") : STR_C("Or drag at a crossing of two straights at its skos"));
		return text;
	}
	text += "\n" + format(STR_C("a %.3f m to the centre of the switch, b %.3f m on to its end"), Template.a, Template.b);
	if (auto const speed{speed_of(Template)}; speed > 0.0)
		text += "\n" + format(STR_C("%.0f km/h on the diverging track"), speed);
	if (auto const note{Template.label.rfind(" (")}; false == Template.own && note != std::string::npos && Template.label.back() == ')')
		text += "\n" + STR(Template.label.substr(note + 1));
	return text;
}

// the angle of the template at a glance, steeper than it is so 1:9 and 1:18.5 differ: the main track and the diverging
// one, or the two crossing tracks of a slip or a crossing
void template_glyph(ImDrawList &Draw, ImVec2 const Low, ImVec2 const High, editor_track::switch_template const &Template, bool const Chosen)
{
	auto const line{ImGui::GetColorU32(ImGuiCol_TextDisabled)};
	auto const made{Chosen ? IM_COL32(255, 255, 255, 255) : IM_COL32(120, 200, 255, 255)};
	auto const ratio{Template.ratio > 0.0 ? Template.ratio : Template.radius > 0.0 && Template.length > 0.0 ? 2.0 * Template.radius / Template.length : 9.0};
	auto const slope{static_cast<float>(std::min(0.9, 3.0 / ratio))};
	auto const width{High.x - Low.x};
	auto const middle{std::round((Low.y + High.y) * 0.5f)};
	if (Template.double_slip)
	{
		auto const rise{std::min(width * slope * 0.5f, (High.y - Low.y) * 0.5f)};
		Draw.AddLine(ImVec2(Low.x, middle), ImVec2(High.x, middle), line, 2.0f);
		Draw.AddLine(ImVec2(Low.x, middle + rise), ImVec2(High.x, middle - rise), made, 2.0f);
		return;
	}
	auto const bottom{std::round(High.y - 2.0f)};
	auto const rise{std::min(width * 0.75f * slope, High.y - Low.y - 4.0f)};
	Draw.AddLine(ImVec2(Low.x, bottom), ImVec2(High.x, bottom), line, 2.0f);
	Draw.AddBezierQuadratic(ImVec2(Low.x + width * 0.15f, bottom), ImVec2(Low.x + width * 0.5f, bottom), ImVec2(High.x, bottom - rise), made, 2.0f);
}

// a template as a row of a list: its glyph and its name; true when clicked
bool template_row(int const Index, editor_track::switch_template const &Template, std::string const &Name, bool const Chosen)
{
	auto const row{std::round(ImGui::GetTextLineHeight() * 1.3f)};
	auto const glyph{std::round(row * 1.8f)};
	ImGui::PushID(Index);
	auto const low{ImGui::GetCursorScreenPos()};
	auto const clicked{ImGui::Selectable("##template", Chosen, ImGuiSelectableFlags_None, ImVec2(0.0f, row))};
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", template_tooltip(Template).c_str());
	if (ImGui::IsItemVisible())
	{
		auto &draw{*ImGui::GetWindowDrawList()};
		template_glyph(draw, ImVec2(low.x + 2.0f, low.y + 2.0f), ImVec2(low.x + glyph, low.y + row - 2.0f), Template, Chosen);
		draw.AddText(ImVec2(low.x + glyph + ImGui::GetStyle().ItemSpacing.x, std::round(low.y + (row - ImGui::GetTextLineHeight()) * 0.5f)), ImGui::GetColorU32(ImGuiCol_Text), Name.c_str());
	}
	ImGui::PopID();
	return clicked;
}

// the height of a list of the rows in its window, as high as the panel lets it
float list_height(int const Rows, int const Headers)
{
	auto const row{std::round(ImGui::GetTextLineHeight() * 1.3f) + ImGui::GetStyle().ItemSpacing.y};
	return std::min(std::round(ImGui::GetFontSize() * 18.0f), (Rows + 0.5f) * row + Headers * ImGui::GetTextLineHeightWithSpacing() + 8.0f);
}

} // namespace

std::string editor_mode::template_name(editor_track::switch_template const &Template)
{
	if (Template.source != nullptr)
		return Template.title();
	auto const speed{Template.double_slip && Template.type == "Skrz" ? 0.0 : speed_of(Template)};
	return switch_name_translated(Template.title()) + (speed > 0.0 ? format(", %.0f km/h", speed) : std::string{});
}

int editor_mode::matching_template(switch_store::record const &Record, bool const Lengths, std::optional<bool> const Movable) const
{
	auto const &templates{m_switch.templates};
	int found{-1};
	for (int i = 0; i < static_cast<int>(templates.size()); ++i)
	{
		auto const &candidate{templates[i]};
		if (candidate.source != nullptr || candidate.double_slip || candidate.twofold() != (Record.kind == "double") || std::abs(candidate.radius - Record.radius) >= 0.5 ||
		    std::abs(candidate.ratio - Record.ratio) >= 0.01 || (Record.rail != "-" && candidate.rail != Record.rail))
			continue;
		if (Lengths && (std::abs(candidate.a - Record.a) > 0.01 || std::abs(candidate.b - Record.b) > 0.01))
			continue;
		if (false == Movable.has_value() || candidate.movable == *Movable)
			return i;
		if (found < 0)
			found = i;
	}
	return found;
}

int editor_mode::switch_template_index(TTrack const &Switch) const
{
	if (Switch.eType != tt_Switch || Switch.m_editorremoved)
		return -1;
	auto known{switch_store::of(Switch)};
	if (false == known.has_value())
		known = switch_store::named(Switch);
	if (false == known.has_value())
	{
		// a switch of a crossover: the record of the crossover keeps the size of its switches
		auto const connections{switch_store::connections(Switch)};
		if (false == connections.empty())
			known = connections.front();
	}
	if (false == known.has_value() || known->source == "measured")
		return -1;
	auto const movable{Switch.SwitchExtension != nullptr && Switch.SwitchExtension->m_movablenose};
	return matching_template(*known, known->kind != "connection", movable);
}

int editor_mode::slip_template_index(switch_store::record const &Record, std::string const &Rail) const
{
	auto const &templates{m_switch.templates};
	int found{-1};
	for (int i = 0; i < static_cast<int>(templates.size()); ++i)
	{
		auto const &candidate{templates[i]};
		if (candidate.type != Record.type || false == of_skos(candidate, angle_of(Record)) || (candidate.type != "Skrz" && std::abs(candidate.radius - Record.radius) >= 0.5))
			continue;
		if (Rail.empty() || candidate.rail == Rail)
			return i;
		if (found < 0)
			found = i;
	}
	return found;
}

std::string editor_mode::template_change_blocked(TTrack const &Switch) const
{
	auto const slip{slip_of(Switch)};
	if (auto const crossing{slip.has_value() ? std::nullopt : crossing_record(Switch)}; slip.has_value() || crossing.has_value())
		return (slip.has_value() ? slip->record : *crossing).main_radius != 0.0 ? STR_C("A slip bent into a curve: delete it and lay it again of another template") : std::string{};
	if (Switch.eType != tt_Switch || Switch.m_paths.size() < 2 || Switch.m_editorremoved)
		return STR_C("Not a switch");
	if (false == switch_store::connections(Switch).empty() && false == crossover_of(Switch).has_value())
		return STR_C("A switch of a connection laid whole: delete the connection and lay it again of another template");
	return {};
}

bool editor_mode::change_slip_template(TTrack &Part, int const Index)
{
	auto &tool{m_switch};
	auto const &shape{tool.templates[Index]};
	auto const fail = [&](std::string const &Status) {
		tool.status = Status;
		ui()->set_status(Status, true);
		return false;
	};
	auto const slip{slip_of(Part)};
	auto const crossing{slip.has_value() ? std::nullopt : crossing_record(Part)};
	if (false == slip.has_value() && false == crossing.has_value())
		return false;
	auto const &record{slip.has_value() ? slip->record : *crossing};
	std::vector<TTrack *> parts;
	if (slip.has_value())
		parts = slip->parts;
	else
		for (auto const &name : record.parts)
			if (auto *part{simulation::Paths.find(name)}; part != nullptr && false == part->m_editorremoved)
				parts.push_back(part);
	for (auto const *part : parts)
		if (false == part->Dynamics.empty())
			return fail(STR_C("The slip can't be laid again while vehicles stand on it"));
	// its drives go with it and the new one gets its own: a drive not described for the editor can't be told apart from them
	if (auto const drive{slip.has_value() ? undescribed_drive(slip->switches) : std::string{}}; false == drive.empty())
		return fail(format(STR_C("The slip has the drive %s, not described for the editor, which may also set the speed or carry its ballast: it isn't laid anew"), drive.c_str()));
	// a single slip laid again joins the pair of the tracks it joined: the point given by them
	auto const &ends{slip.has_value() ? slip->ends : std::array<glm::dvec3, 4>{}};
	auto const pair{slip.has_value() && record.type == "Rkp" ? 0.5 * (ends[0] + ends[3]) : glm::dvec3{0.0}};
	// taken out, the two tracks left crossing in its place; a crossing is those two tracks already
	std::vector<TTrack *> throughs;
	if (slip.has_value())
	{
		throughs = slip_taken_out(*slip, true);
		push_track_snapshot({}, throughs, parts);
		drives_deleted(slip->switches);
	}
	else
	{
		std::vector<std::pair<TTrack *, editor_track::state>> states;
		for (auto *part : parts)
			states.emplace_back(part, editor_track::capture(*part));
		switch_store::forget(record.name);
		push_track_snapshot(std::move(states), {}, {});
		throughs = parts;
	}
	records_to_history();
	auto const undo = [&](std::string const &Status) {
		undo_last();
		return fail(Status);
	};
	if (throughs.size() != 2)
		return undo(STR_C("The tracks of the slip can't be laid crossing in its place"));
	glm::dvec2 directions[2];
	glm::dvec3 starts[2];
	for (int i = 0; i < 2; ++i)
	{
		auto const &path{throughs[i]->m_paths.front()};
		starts[i] = path.points[segment_data::point::start];
		directions[i] = plan_of(path.points[segment_data::point::end] - starts[i]);
		if (glm::length(directions[i]) < 1e-6)
			return undo(STR_C("The tracks of the slip can't be laid crossing in its place"));
		directions[i] = glm::normalize(directions[i]);
	}
	double along{0.0}, alongother{0.0};
	if (false == geometry::lines_cross(plan_of(starts[0]), directions[0], plan_of(starts[1]), directions[1], along, alongother))
		return undo(STR_C("The tracks of the slip don't cross"));
	auto const middle{plan_of(starts[0]) + directions[0] * along};
	auto point{glm::dvec3{middle.x, starts[0].y, middle.y}};
	if (shape.type == "Rkp" && pair != glm::dvec3{0.0})
		if (auto const towards{plan_of(pair) - middle}; glm::length(towards) > 1e-3)
		{
			auto const shift{glm::normalize(towards)};
			point += glm::dvec3{shift.x, 0.0, shift.y};
		}
	tool.keep_slip = record.name;
	auto const steps{m_history.size()};
	auto const laid{insert_double_slip(editor_track::find_straight(*throughs[0], m_straights.tolerance), point, shape, 0)};
	tool.keep_slip.clear();
	if (false == laid || m_history.size() == steps)
		return undo(m_straights.status.empty() ? format(STR_C("%s doesn't fit where the tracks cross"), template_name(shape).c_str()) : m_straights.status);
	join_last_steps();
	records_to_history();
	tool.status = format(STR_C("%s laid again of %s"), record.name.c_str(), template_name(shape).c_str()) + (m_straights.status.empty() ? "" : "\n") + m_straights.status;
	ui()->set_status(tool.status, false);
	WriteLog("Editor: slip " + record.name + " laid again of the template " + shape.title(), logtype::generic);
	// the first part of the new one selected, the slip whole for the inspector
	auto const &records{switch_store::records()};
	if (auto const again{std::find_if(records.begin(), records.end(), [&](switch_store::record const &Entry) { return Entry.name == record.name && false == Entry.parts.empty(); })}; again != records.end())
		if (auto *first{simulation::Paths.find(again->parts.front())}; first != nullptr && false == first->m_editorremoved)
			select_track(first);
	return true;
}

bool editor_mode::apply_switch_template(TTrack &Switch, editor_track::switch_template const &Shape, bool const Flip, std::string &Notes)
{
	Notes.clear();
	auto const set{double_of(Switch)};
	auto &first{set.has_value() ? *set->first : Switch};
	if (first.m_paths.size() < 2 || Shape.twofold() != set.has_value())
		return false;
	auto const &main{first.m_paths[0]};
	auto const &diverging{first.m_paths[1]};
	if (set.has_value() || false == has_controls(main))
	{
		// along the straight: the start and the direction of the switch kept
		auto const origin{main.points[segment_data::point::start]};
		auto const chord{plan_of(main.points[segment_data::point::end] - origin)};
		auto const length{glm::length(chord)};
		if (length < 1e-3)
			return false;
		glm::dvec2 const axis{chord / length};
		glm::dvec2 const normal{-axis.y, axis.x};
		auto const side{(glm::dot(plan_of(diverging.points[segment_data::point::end] - origin), normal) > 0.0) != Flip ? 1 : -1};
		auto paths{editor_track::place_switch(Shape, origin, axis, side, (main.points[segment_data::point::end].y - origin.y) / length)};
		if (set.has_value())
		{
			if (paths.size() < 5)
				return false;
			auto const before{double_paths(*set)};
			for (std::size_t i = 0; i < paths.size(); ++i)
				if (paths[i].rolls[0] == 0.f && paths[i].rolls[1] == 0.f)
					paths[i].rolls = before[i < 4 ? i & ~std::size_t{1} : 4].rolls;
			if (false == move_double_paths(*set, paths, Notes))
				return false;
		}
		else
		{
			if (paths.size() < 2)
				return false;
			for (int i = 0; i < 2; ++i)
				if (paths[i].rolls[0] == 0.f && paths[i].rolls[1] == 0.f)
					paths[i].rolls = first.m_paths[i].rolls;
			if (false == move_switch_paths(first, paths, Notes))
				return false;
		}
	}
	else
	{
		// bent into a curve: bent into it again, the start kept where it is along the curve
		std::vector<curve_sample> frame;
		std::array<TTrack *, 2> sides;
		double station{0.0};
		if (false == bent_switch_frame(first, frame, sides, station))
		{
			Notes = STR_C("A switch bent into a curve takes another template where plain tracks go on from both ends of its main track");
			return false;
		}
		auto const reach{std::abs(Shape.local[0].points[segment_data::point::end].z)};
		if (auto const over{station + reach - frame.back().station}; over > 1e-3)
		{
			Notes = format(STR_C("The template is %.2f m longer than the curve the switch lies in gives it, past the track beyond the switch"), over);
			return false;
		}
		auto const end{diverging.points[segment_data::point::end]};
		auto const at{sample_at(frame, nearest_station(frame, end))};
		auto const side{(glm::dot(plan_of(end) - plan_of(at.position), glm::dvec2{-at.tangent.y, at.tangent.x}) > 0.0) != Flip ? 1 : -1};
		if (false == lay_bent_switch(first, frame, sides, curved_switch_paths(Shape, frame, station, 1, side), Notes))
			return false;
	}
	// the blades and the frog of a template of the catalogue, as a switch laid of it has them
	if (Shape.source == nullptr)
		for (auto *track : {&first, set.has_value() ? set->second : nullptr})
		{
			auto *extension{track != nullptr ? track->SwitchExtension.get() : nullptr};
			if (extension == nullptr || (extension->m_movablenose == Shape.movable && extension->m_bladeheel == Shape.heel && extension->m_bladethrow == Shape.bladethrow))
				continue;
			extension->m_movablenose = Shape.movable;
			extension->m_bladeheel = Shape.heel;
			extension->m_bladethrow = Shape.bladethrow;
			editor_track::commit_parameters(*track);
		}
	if (set.has_value())
		record_double(*set->first, *set->second, Shape);
	else
		record_switch(first, Shape);
	switch_store::follow();
	records_to_history();
	straight_refresh();
	return true;
}

bool editor_mode::change_switch_template(TTrack &Switch, int const Index)
{
	auto &tool{m_switch};
	if (Index < 0 || Index >= static_cast<int>(tool.templates.size()))
		return false;
	// the switch that keeps its template blinks
	auto const fail = [&](std::string const &Status) {
		tool.status = Status;
		ui()->set_status(Status, true);
		flash_switch_problem(Status, Switch.m_paths, {{Switch.location(), ""}});
		return false;
	};
	if (auto const reason{template_change_blocked(Switch)}; false == reason.empty())
		return fail(reason);
	if (slip_of(Switch).has_value() || crossing_record(Switch).has_value())
		return change_slip_template(Switch, Index);
	auto const name{Switch.name()};
	// a crossover taken out and laid again of the template, from where it starts; the tool and the selection as they were
	if (crossover_of(Switch).has_value())
	{
		auto const tab{m_track_tab};
		auto const done{slide_curved_connection(Switch, 0.0, Index)};
		show_track_tab(tab);
		if (auto *again{simulation::Paths.find(name)}; again != nullptr && false == again->m_editorremoved)
			select_track(again);
		return done;
	}
	auto const &shape{tool.templates[Index]};
	std::string notes;
	if (false == apply_switch_template(Switch, shape, false, notes))
		return fail(notes.empty() ? format(STR_C("%s doesn't fit in place of the switch"), template_name(shape).c_str()) : notes);
	tool.status = format(STR_C("Switch %s: %s"), name.c_str(), template_name(shape).c_str()) + (notes.empty() ? "" : "\n") + notes;
	ui()->set_status(tool.status, false == notes.empty());
	WriteLog("Editor: switch " + name + " given the template " + shape.title(), logtype::generic);
	return true;
}

int editor_mode::render_template_picker(char const *Id, std::function<bool(editor_track::switch_template const &)> const &Fits, std::string const &Rail, int const Chosen, bool const Popup)
{
	auto &tool{m_switch};
	auto const &templates{tool.templates};
	auto const count{static_cast<int>(templates.size())};
	int picked{-1};
	ImGui::PushID(Id);
	// the rail: of the track, else the one picked, among the rails the catalogue has templates of the kind for
	auto rail{Rail};
	if (rail.empty())
	{
		std::vector<std::string> rails;
		for (auto const &entry : templates)
			if (Fits(entry) && entry.source == nullptr && false == entry.own && entry.rail != "-" && std::find(rails.begin(), rails.end(), entry.rail) == rails.end())
				rails.push_back(entry.rail);
		if (std::find(rails.begin(), rails.end(), tool.rail_pick) == rails.end() && false == rails.empty())
			tool.rail_pick = rails.front();
		if (rails.size() > 1)
		{
			ImGui::TextDisabled("%s", STR_C("Rails"));
			item_tooltip("The track doesn't tell its rail: the templates of the rail picked are listed, all of them under All templates");
			int pick{static_cast<int>(std::find(rails.begin(), rails.end(), tool.rail_pick) - rails.begin())};
			std::vector<segment_option> options;
			for (std::size_t i = 0; i < rails.size(); ++i)
				options.push_back({static_cast<int>(i), rails[i].c_str()});
			ImGui::SameLine();
			if (choice_chips("##rails", pick, options))
				tool.rail_pick = rails[pick];
		}
		rail = tool.rail_pick;
	}
	// the short list: the catalogue of the rail and the own templates, the one chosen also when it's of neither
	std::vector<int> listed;
	for (int i = 0; i < count; ++i)
	{
		auto const &entry{templates[i]};
		if (Fits(entry) && entry.source == nullptr && (entry.own || entry.rail == rail || entry.rail == "-"))
			listed.push_back(i);
	}
	auto const outside{Chosen >= 0 && Chosen < count && std::find(listed.begin(), listed.end(), Chosen) == listed.end()};
	// the speeds on the diverging track the switches of the rail are made for, each picking the switch usual for it
	std::map<int, int> speeds;
	auto const usual = [&](int const Index) {
		auto const &entry{templates[Index]};
		return std::make_tuple(entry.movable, std::abs(entry.ratio - usual_ratio(speed_of(entry))), false == words_of(entry).rest.empty(), Index);
	};
	for (auto const index : listed)
	{
		auto const &entry{templates[index]};
		if (entry.own || entry.double_slip || entry.twofold() || entry.type != "Rz" || speed_of(entry) <= 0.0)
			continue;
		auto const speed{static_cast<int>(speed_of(entry))};
		if (auto const known{speeds.find(speed)}; known == speeds.end() || usual(index) < usual(known->second))
			speeds[speed] = index;
	}
	if (speeds.size() > 1)
	{
		ImGui::TextDisabled("%s", STR_C("Speed on the diverging track"));
		item_tooltip("Picks the usual switch of the rail for the speed; the others of the same speed are on the list under it");
		int speed{Chosen >= 0 && Chosen < count ? static_cast<int>(speed_of(templates[Chosen])) : -1};
		std::vector<std::string> labels;
		for (auto const &[value, index] : speeds)
			labels.push_back(format("%d km/h", value));
		std::vector<segment_option> options;
		auto label{labels.begin()};
		for (auto const &[value, index] : speeds)
			options.push_back({value, (label++)->c_str()});
		if (choice_chips("##speeds", speed, options))
			picked = speeds[speed];
	}
	// by the skos, the steepest first; the own templates and the one chosen from elsewhere after them
	std::vector<std::pair<double, std::string>> groups;
	auto const group_of = [&](int const Index) -> std::pair<double, std::string> {
		auto const &entry{templates[Index]};
		if (Index == Chosen && outside)
			return {1e9, STR("Chosen")};
		if (entry.own)
			return {1e8, STR("Own templates")};
		return {entry.ratio, skos_of(entry)};
	};
	auto shown{listed};
	if (outside)
		shown.push_back(Chosen);
	for (auto const index : shown)
		if (auto const group{group_of(index)}; std::find(groups.begin(), groups.end(), group) == groups.end())
			groups.push_back(group);
	std::sort(groups.begin(), groups.end());
	if (Popup || ImGui::BeginChild("##short", ImVec2(0.0f, list_height(static_cast<int>(shown.size()), static_cast<int>(groups.size()))), ImGuiChildFlags_Borders))
		for (auto const &group : groups)
		{
			ImGui::SeparatorText(group.second.c_str());
			for (auto const index : shown)
				if (group_of(index) == group && template_row(index, templates[index], row_name(templates[index]), index == Chosen))
					picked = index;
		}
	if (false == Popup)
		ImGui::EndChild();
	// all of them: the switches of the scenery, the own templates, then the catalogue rail by rail
	if (ImGui::TreeNode(STR_C("All templates")))
	{
		auto const whole_group = [](editor_track::switch_template const &Template) {
			return Template.source != nullptr ? std::string{STR("From the scenery")} : Template.own ? std::string{STR("Own templates")} : "PLK " + (Template.rail == "-" ? std::string{} : Template.rail);
		};
		std::vector<std::string> wholes;
		int rows{0};
		for (auto const &entry : templates)
			if (Fits(entry))
			{
				++rows;
				if (auto const group{whole_group(entry)}; std::find(wholes.begin(), wholes.end(), group) == wholes.end())
					wholes.push_back(group);
			}
		if (Popup || ImGui::BeginChild("##all", ImVec2(0.0f, list_height(rows, static_cast<int>(wholes.size()))), ImGuiChildFlags_Borders))
			for (auto const &group : wholes)
			{
				ImGui::SeparatorText(group.c_str());
				for (int i = 0; i < count; ++i)
					if (Fits(templates[i]) && whole_group(templates[i]) == group &&
					    template_row(i, templates[i], templates[i].source != nullptr ? row_name(templates[i]) : switch_name_translated(templates[i].title()), i == Chosen))
						picked = i;
			}
		if (false == Popup)
			ImGui::EndChild();
		ImGui::TreePop();
	}
	ImGui::PopID();
	return picked == Chosen ? -1 : picked;
}

void editor_mode::render_switch_template(TTrack &Track, bool const Combo)
{
	auto &tool{m_switch};
	if (false == tool.collected)
	{
		tool.templates = editor_track::standard_switch_templates();
		tool.collected = true;
	}
	// a slip or a crossing: the templates for the angle its tracks cross at; a switch, a double one: those of its kind
	auto const slip{slip_of(Track)};
	auto const crossing{slip.has_value() ? std::nullopt : crossing_record(Track)};
	auto const set{double_of(Track)};
	auto *first{set.has_value() ? set->first : &Track};
	std::optional<switch_store::record> known;
	std::function<bool(editor_track::switch_template const &)> fits;
	auto rail{editor_track::rail_of(first)};
	int current{-1};
	if (slip.has_value() || crossing.has_value())
	{
		known = slip.has_value() ? slip->record : *crossing;
		auto const angle{angle_of(*known)};
		fits = [angle](editor_track::switch_template const &Template) { return of_skos(Template, angle); };
		current = slip_template_index(*known, rail);
	}
	else if (Track.eType == tt_Switch)
	{
		known = switch_store::named(*first);
		fits = [twofold{set.has_value()}](editor_track::switch_template const &Template) { return false == Template.double_slip && Template.twofold() == twofold; };
		current = switch_template_index(*first);
		if (rail.empty() && known.has_value() && known->rail != "-")
			rail = known->rail;
	}
	else
		return;
	auto const blocked{template_change_blocked(*first)};
	auto const own = [&]() { return known.has_value() ? format(STR_C("Own shape: %s"), switch_store::label(*known).c_str()) : std::string{STR_C("Own shape")}; };
	int picked{-1};
	if (Combo)
	{
		if (begin_fields("##switchtemplate", 0.3f))
		{
			field_row("Template##switchtemplate");
			ImGui::BeginDisabled(false == blocked.empty());
			ImGui::PushItemWidth(-1.0f);
			if (ImGui::BeginCombo("##switchtemplate", current >= 0 ? template_name(tool.templates[current]).c_str() : own().c_str(), ImGuiComboFlags_HeightLargest))
			{
				picked = render_template_picker("##inspector", fits, rail, current, true);
				if (picked >= 0)
					ImGui::CloseCurrentPopup();
				ImGui::EndCombo();
			}
			ImGui::PopItemWidth();
			ImGui::EndDisabled();
			if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
				ImGui::SetTooltip("%s", blocked.empty() ? STR_C("Another template in place of this one: the start and the side of the switch kept, the tracks it joins following it") : blocked.c_str());
			ImGui::EndTable();
		}
	}
	else
	{
		ImGui::TextDisabled("%s", STR_C("Template"));
		ImGui::SameLine();
		ImGui::TextUnformatted(current >= 0 ? template_name(tool.templates[current]).c_str() : own().c_str());
		if (false == blocked.empty())
		{
			ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", blocked.c_str());
			return;
		}
		picked = render_template_picker("##turnout", fits, rail, current, false);
	}
	if (picked >= 0)
		change_switch_template(Track, picked);
}
