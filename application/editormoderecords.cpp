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
#include "application/editoruilayer.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorGeometry.hpp"

#include "world/Track.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <span>
#include <string_view>

namespace
{

using geometry::bezier;
using geometry::plan_of;
using geometry::signed_angle;
using geometry::turned;
using path_ends = std::pair<glm::dvec3, glm::dvec3>;

struct record_drift
{
	int followed{0};
	int departed{0};
	double departure{0.0};
	double cant{0.0};
	std::string layer;
};

glm::dvec2 chord_direction(glm::dvec3 const &Start, glm::dvec3 const &End)
{
	auto const direction{plan_of(End - Start)};
	return glm::length(direction) > 1e-9 ? glm::normalize(direction) : glm::dvec2{0.0, 1.0};
}

path_ends const &ends_before(std::vector<TTrack *> const &Moving, std::vector<path_ends> const &Before, TTrack const *Track)
{
	return Before[static_cast<std::size_t>(std::ranges::find(Moving, Track) - Moving.begin())];
}

bool lies_on(int const Record, TTrack const &Track)
{
	bezier const curve{Track.m_paths.front()};
	std::array<double, 3> constexpr samples{0.0, 0.5, 1.0};
	return std::ranges::all_of(samples, [&](double const T) { return alignment_store::on_record(Record, curve.point(T), alignment_store::tolerance).has_value(); });
}

bool covers_record(int const Record, std::vector<TTrack *> const &Moving)
{
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && track->eType == tt_Normal && false == track->m_paths.empty() && std::ranges::find(Moving, track) == Moving.end() && lies_on(Record, *track))
			return false;
	return true;
}

bool carried_rigidly(int const Record, std::vector<TTrack *> const &Tracks, std::vector<TTrack *> const &Moving, std::vector<path_ends> const &Before, record_drift &Drift)
{
	auto const &[first_start, first_end]{ends_before(Moving, Before, Tracks.front())};
	auto const &path{Tracks.front()->m_paths.front()};
	auto const &start{path.points[segment_data::point::start]};
	auto const from{chord_direction(first_start, first_end)};
	auto const to{chord_direction(start, path.points[segment_data::point::end])};
	auto const angle{signed_angle(from, to)};
	auto const origin{plan_of(first_start)};
	auto const carried = [&start, &origin, &angle](glm::dvec3 const &Old, glm::dvec3 const &Current) { return glm::distance(plan_of(start) + turned(plan_of(Old) - origin, angle), plan_of(Current)) <= 0.01; };
	for (auto const *track : Tracks)
	{
		auto const &[was_start, was_end]{ends_before(Moving, Before, track)};
		auto const &now{track->m_paths.front()};
		if (false == carried(was_start, now.points[segment_data::point::start]) || false == carried(was_end, now.points[segment_data::point::end]))
			return false;
	}
	if (glm::distance(first_start, start) > 1e-4 || std::abs(angle) > 1e-7)
	{
		alignment_store::transform(Record, first_start, from, start, to);
		++Drift.followed;
	}
	return true;
}

void measure_departure(int const Record, std::span<TTrack *const> const Tracks, record_drift &Drift)
{
	auto const &records{alignment_store::records()};
	auto const gauge{records[Record].design.norms.gauge};
	for (auto const *track : Tracks)
	{
		if (track->m_editorremoved)
			continue;
		auto const &path{track->m_paths.front()};
		bezier const curve{path};
		double largest{0.0};
		for (auto const t : {0.0, 0.5, 1.0})
		{
			auto const found{alignment_store::on_record(Record, curve.point(t), 1000.0)};
			largest = std::max(largest, found.has_value() ? found->offset : 1000.0);
		}
		if (largest > alignment_store::tolerance)
		{
			++Drift.departed;
			Drift.departure = std::max(Drift.departure, largest);
			Drift.layer = scene::Layers.layer(records[Record].layer).name;
			continue;
		}
		for (auto const end : {0, 1})
			if (auto const found{alignment_store::on_record(Record, curve.point(end), alignment_store::tolerance)})
			{
				auto const actual{gauge * std::sin(std::abs(glm::radians(static_cast<double>(path.rolls[end]))))};
				Drift.cant = std::max(Drift.cant, std::abs(actual - alignment_store::cant_at(Record, found->chainage)));
			}
	}
}

std::vector<std::string> drift_notes(record_drift const &Drift)
{
	std::vector<std::string> notes;
	if (Drift.followed > 0)
		notes.push_back(STR("The kept design of the line went along with its paths"));
	if (Drift.departed > 0)
		notes.push_back(format(STR_P("%d path left the design kept in %s, by up to %.2f m: Ctrl+Z takes it back, the Plan tool (G) lays it by the design again or forgets the design",
		                             "%d paths left the design kept in %s, by up to %.2f m: Ctrl+Z takes it back, the Plan tool (G) lays them by the design again or forgets it", Drift.departed),
		                       Drift.departed, Drift.layer.c_str(), Drift.departure));
	if (Drift.cant > 5.0)
		notes.push_back(format(STR_C("The cant differs from the kept design by up to %.0f mm"), Drift.cant));
	return notes;
}

switch_store::record switch_entry(editor_track::switch_template const &Shape)
{
	switch_store::record entry;
	if (Shape.source == nullptr)
	{
		entry.type = Shape.type;
		entry.rail = Shape.rail;
		entry.radius = Shape.radius;
		entry.ratio = Shape.ratio;
		entry.a = Shape.a;
		entry.b = Shape.b;
		entry.source = "designed";
	}
	else if (auto const known{switch_store::of(*Shape.source)}; known.has_value() && known->kind == "switch")
	{
		entry = *known;
		entry.number = "-";
	}
	else
	{
		// Rz, Rlj or Rld, as classify finds it by the shape
		entry.type = "Rz";
		entry.radius = Shape.radius;
		entry.ratio = Shape.ratio;
		entry.source = "measured";
	}
	return entry;
}

template <std::size_t Size>
bool edit_dashed(char const *Label, std::string &Value)
{
	// the text cut to the buffer, the rest of it left zero
	std::array<char, Size> text{};
	std::format_to_n(text.data(), static_cast<std::ptrdiff_t>(text.size() - 1), "{}", Value == "-" ? "" : Value.c_str());
	if (false == ImGui::InputText(Label, text.data(), text.size(), ImGuiInputTextFlags_EnterReturnsTrue) && false == ImGui::IsItemDeactivatedAfterEdit())
		return false;
	Value = text[0] != '\0' ? text.data() : "-";
	return true;
}

// the record of the slip the switch is kept as a part of
std::optional<switch_store::record> slip_record_of(TTrack const &Each)
{
	if (Each.eType != tt_Switch || Each.m_editorremoved)
		return std::nullopt;
	auto found{switch_store::named(Each)};
	return found.has_value() && found->kind == "slip" ? found : std::nullopt;
}

// the record of the switch, or of a switch of the slip the track between them is joined to
std::optional<switch_store::record> slip_record_near(TTrack const &Track)
{
	auto known{slip_record_of(Track)};
	if (known.has_value() || Track.eType != tt_Normal || Track.m_paths.size() != 1)
		return known;
	for (auto const index : {segment_data::point::start, segment_data::point::end})
		for (auto const &[other, point] : editor_track::connected_points(Track, Track.m_paths.front().points[index]))
			if (false == known.has_value())
				known = slip_record_of(*other);
	return known;
}

// the switches the record names, none when one of them isn't a switch of the scenery
std::vector<TTrack *> slip_switches(switch_store::record const &Record)
{
	std::vector<TTrack *> switches;
	for (auto const &name : Record.parts)
	{
		auto *part{simulation::Paths.find(name)};
		if (part == nullptr || part->m_editorremoved || part->eType != tt_Switch || part->m_paths.size() < 2)
			return {};
		switches.push_back(part);
	}
	return switches;
}

// the tracks joined to the part at the point, not among the parts yet, added to them
void add_joined_at(TTrack const &Part, glm::dvec3 const &Position, std::vector<TTrack *> &Parts)
{
	for (auto const &[other, point] : editor_track::connected_points(Part, Position))
		if (false == other->m_editorremoved && std::ranges::find(Parts, other) == Parts.end())
			Parts.push_back(other);
}

// the tracks joined to the part at its points inside the slip, near where its straights cross, added to the parts
void add_joined_inside(std::vector<TTrack *> &Parts, std::size_t const Index, glm::dvec2 const &Crossing, double const Inside)
{
	auto const &part{*Parts[Index]};
	for (auto const &path : part.m_paths)
		for (auto const index : {segment_data::point::start, segment_data::point::end})
		{
			auto const &position{path.points[index]};
			if (glm::distance(plan_of(position), Crossing) > Inside)
				continue;
			add_joined_at(part, position, Parts);
		}
}

// all the tracks of the slip: its switches and the tracks joined to them inside it, and to those in turn
std::vector<TTrack *> slip_parts(std::vector<TTrack *> const &Switches, glm::dvec2 const &Crossing, double const Inside)
{
	auto parts{Switches};
	std::size_t index{0};
	while (index < parts.size())
	{
		add_joined_inside(parts, index, Crossing, Inside);
		++index;
	}
	return parts;
}

using slip_end = std::pair<glm::dvec3, glm::dvec2>;

// the way into the slip at the end of the path that is one of its ends, not of unit length
glm::dvec2 way_in(segment_data const &Path, bool const AtEnd, glm::dvec3 const &Position)
{
	auto const &handle{Path.points[AtEnd ? segment_data::point::control2 : segment_data::point::control1]};
	if (handle != glm::dvec3{})
		return plan_of(handle);
	return plan_of(Path.points[AtEnd ? segment_data::point::start : segment_data::point::end] - Position);
}

// the ends of the paths of the part far off the crossing of the slip, each with the way into the slip there, added to the ends
// found so far; false when an end has no way in
bool add_part_ends(TTrack const &Part, glm::dvec2 const &Crossing, double const Inside, std::vector<slip_end> &Ends)
{
	for (auto const &path : Part.m_paths)
		for (auto const atend : {false, true})
		{
			auto const &position{path.points[atend ? segment_data::point::end : segment_data::point::start]};
			if (glm::distance(plan_of(position), Crossing) <= Inside ||
			    std::ranges::any_of(Ends, [&position](auto const &End) { return glm::distance(End.first, position) < geometry::same_point; }))
				continue;
			auto const into{way_in(path, atend, position)};
			if (glm::length(into) < 1e-9)
				return false;
			Ends.emplace_back(position, glm::normalize(into));
		}
	return true;
}

// the ends of the slip, each with the way into the slip there; nothing when an end has no way in
std::optional<std::vector<slip_end>> slip_ends(std::vector<TTrack *> const &Parts, glm::dvec2 const &Crossing, double const Inside)
{
	std::vector<slip_end> ends;
	for (auto const *part : Parts)
		if (false == add_part_ends(*part, Crossing, Inside, ends))
			return std::nullopt;
	return ends;
}

// the place among the ends of the one nearest to the point, in the plan
std::size_t nearest_end(std::vector<slip_end> const &Ends, glm::dvec2 const &Point)
{
	auto const nearest{std::ranges::min_element(Ends, [&Point](auto const &One, auto const &Other) { return glm::distance(plan_of(One.first), Point) < glm::distance(plan_of(Other.first), Point); })};
	return static_cast<std::size_t>(nearest - Ends.begin());
}

} // namespace

void editor_mode::records_watch_begin()
{
	auto &watch{m_record_watch};
	watch = {};
	watch.active = true;
	watch.moving = m_track_drag;
	for (auto const &link : m_track_gizmo_drag.switches)
		for (auto const &[member, member_paths] : link.cluster)
			if (std::ranges::find(watch.moving, member) == watch.moving.end())
				watch.moving.push_back(member);
	for (auto *track : watch.moving)
	{
		auto const &path{track->m_paths.front()};
		watch.before.emplace_back(path.points[segment_data::point::start], path.points[segment_data::point::end]);
		if (track->eType != tt_Normal)
			continue;
		if (auto const found{alignment_store::on_track(*track)})
			watch.on[found->record].push_back(track);
	}
	for (auto const &[record, tracks] : watch.on)
		if (covers_record(record, watch.moving))
			watch.whole.insert(record);
}

// what the drag of the gizmo did to the kept alignments and switches: an alignment all of whose paths were moved or turned as a
// whole goes along with them, paths pulled out of an alignment or given another cant are told about, as are switches bent out
// of the shape of their type; switches moved as a whole take their records along
void editor_mode::records_watch_end()
{
	auto &watch{m_record_watch};
	if (false == watch.active)
		return;
	watch.active = false;
	record_drift drift;
	for (auto const &[record, tracks] : watch.on)
		if (false == watch.whole.contains(record) || false == carried_rigidly(record, tracks, watch.moving, watch.before, drift))
			measure_departure(record, tracks, drift);
	auto notes{drift_notes(drift)};
	switch_store::follow();
	for (auto const *track : watch.moving)
		if (track->eType == tt_Switch && false == track->m_editorremoved)
			if (auto const known{switch_store::named(*track)}; known.has_value() && false == switch_store::of(*track).has_value())
				notes.push_back(format(STR_C("Switch %s lost the shape of its type %s: Ctrl+Z takes it back"), track->name().c_str(), switch_store::label(*known).c_str()));
	records_to_history();
	if (false == notes.empty())
	{
		std::string text;
		for (auto const &note : notes)
			text += (text.empty() ? "" : "\n") + note;
		ui()->set_status(text, drift.departed > 0 || drift.cant > 5.0 || notes.size() > static_cast<std::size_t>(drift.followed > 0 ? 1 : 0));
		WriteLog("Editor: " + text, logtype::generic);
	}
	watch = {};
}

void editor_mode::records_to_history()
{
	auto alignments{alignment_store::take_changes()};
	auto switches{switch_store::take_changes()};
	if (m_history.empty() || m_history.back().action != EditorSnapshot::Action::TrackEdit)
		return;
	auto &last{m_history.back()};
	if (alignments.has_value() && false == last.alignments.has_value())
		last.alignments = std::move(alignments);
	if (switches.has_value() && false == last.switch_records.has_value())
		last.switch_records = std::move(switches);
}

// the switch is kept in the scenery with its type: of the catalogue template, of the switch of the scenery it was copied
// from, or measured; on a curve it is an arc switch, one-sided when it diverges to the side the curve turns to
void editor_mode::record_switch(TTrack const &Track, editor_track::switch_template const &Shape) const
{
	if (Track.eType != tt_Switch || Track.m_paths.size() < 2 || Track.name().empty() || scene::Layers.empty())
		return;
	auto entry{switch_entry(Shape)};
	entry.kind = "switch";
	entry.name = Track.name();
	entry.parts = {entry.name};
	switch_store::place(entry, Track);
	switch_store::classify(entry, Track);
	switch_store::put(entry, alignment_store::layer_of(Track));
}

// a double switch is kept as one record of its two switches, with the type, the size and the side of the first one
void editor_mode::record_double(TTrack const &First, TTrack const &Second, editor_track::switch_template const &Shape) const
{
	if (First.eType != tt_Switch || Second.eType != tt_Switch || First.m_paths.size() < 2 || First.name().empty() || Second.name().empty() || scene::Layers.empty())
		return;
	auto entry{switch_entry(Shape)};
	entry.kind = "double";
	entry.name = First.name();
	entry.parts = {First.name(), Second.name()};
	switch_store::place(entry, First);
	switch_store::classify(entry, First);
	switch_store::put(entry, alignment_store::layer_of(First));
}

void editor_mode::record_connection(connection::kind const Kind, std::vector<TTrack *> const &Switches, connection::result const &Plan, editor_track::switch_template const &Shape) const
{
	if (Switches.empty() || Switches.front()->eType != tt_Switch || Switches.front()->m_paths.size() < 2 || scene::Layers.empty())
		return;
	auto entry{switch_entry(Shape)};
	entry.kind = "connection";
	entry.name = Switches.front()->name();
	for (auto const *track : Switches)
		if (track->name().empty())
			return;
		else
			entry.parts.push_back(track->name());
	switch_store::place(entry, *Switches.front());
	switch_store::classify(entry, *Switches.front());
	entry.type = connection::word(Kind);
	entry.a = Plan.spacing;
	entry.b = Plan.straights.empty() ? 0.0 : Plan.straights.front();
	switch_store::put(entry, alignment_store::layer_of(*Switches.front()));
}

std::optional<editor_mode::double_switch> editor_mode::double_of_switch(TTrack const &Switch)
{
	if (Switch.eType != tt_Switch || Switch.m_editorremoved)
		return std::nullopt;
	auto const known{switch_store::named(Switch)};
	if (false == known.has_value() || known->kind != "double" || known->parts.size() != 2)
		return std::nullopt;
	auto *first{simulation::Paths.find(known->parts[0])};
	auto *second{simulation::Paths.find(known->parts[1])};
	for (auto const *part : {first, second})
		if (part == nullptr || part->m_editorremoved || part->eType != tt_Switch || part->m_paths.size() != 2)
			return std::nullopt;
	if (first == second)
		return std::nullopt;
	auto const start{second->m_paths.front().points[segment_data::point::start]};
	for (auto const &[other, point] : editor_track::connected_points(*first, first->m_paths.front().points[segment_data::point::end]))
	{
		if (other->m_editorremoved || other->eType != tt_Normal || other->m_paths.size() != 1)
			continue;
		auto const &path{other->m_paths.front()};
		auto const opposite{path.points[point.kind == editor_track::point_kind::start ? segment_data::point::end : segment_data::point::start]};
		if (glm::distance(opposite, start) < geometry::same_point)
			return double_switch{first, other, second};
	}
	return std::nullopt;
}

std::optional<editor_mode::double_switch> editor_mode::double_of(TTrack const &Track) const
{
	if (Track.eType == tt_Switch)
		return double_of_switch(Track);
	if (Track.eType != tt_Normal || Track.m_editorremoved || Track.m_paths.size() != 1)
		return std::nullopt;
	for (auto const index : {segment_data::point::start, segment_data::point::end})
		for (auto const &[other, point] : editor_track::connected_points(Track, Track.m_paths.front().points[index]))
		{
			if (other->eType != tt_Switch)
				continue;
			if (auto const found{double_of_switch(*other)}; found.has_value() && found->link == &Track)
				return found;
		}
	return std::nullopt;
}

std::optional<editor_mode::slip_set> editor_mode::slip_of(TTrack const &Track) const
{
	if (Track.m_editorremoved)
		return std::nullopt;
	auto const known{slip_record_near(Track)};
	if (false == known.has_value())
		return std::nullopt;
	slip_set slip;
	slip.record = *known;
	slip.switches = slip_switches(*known);
	if (slip.switches.size() < 2)
		return std::nullopt;
	// where the straights of its first and its last switch meet: the joints of its tracks near it are inside the slip, its ends far off
	auto const &first{slip.switches.front()->m_paths.front()};
	auto const &last{slip.switches.back()->m_paths.front()};
	auto const a{plan_of(first.points[segment_data::point::start])};
	auto const d{plan_of(last.points[segment_data::point::start])};
	auto const da{editor_track::end_direction(first, editor_track::point_kind::start)};
	auto const dd{editor_track::end_direction(last, editor_track::point_kind::start)};
	double alongfirst{0.0};
	double alonglast{0.0};
	if (false == geometry::lines_cross(a, da, d, dd, alongfirst, alonglast))
		return std::nullopt;
	auto const crossing{a + da * alongfirst};
	auto const inside{0.5 * glm::distance(a, crossing)};
	slip.parts = slip_parts(slip.switches, crossing, inside);
	// its ends, each with the way into the slip there
	auto const found{slip_ends(slip.parts, crossing, inside)};
	if (false == found.has_value() || found->size() != 4)
		return std::nullopt;
	auto const &ends{*found};
	// A where its first switch starts, B on along its first track, D where its last switch starts, C the other one
	auto const ia{nearest_end(ends, a)};
	auto const id{nearest_end(ends, d)};
	if (ia == id)
		return std::nullopt;
	std::vector<std::size_t> others;
	for (std::size_t i = 0; i < 4; ++i)
		if (i != ia && i != id)
			others.push_back(i);
	if (others.size() != 2)
		return std::nullopt;
	// of the other two the one farther on along the first track
	auto const ahead = [&](std::size_t const Index) { return glm::dot(plan_of(ends[Index].first) - crossing, da); };
	auto const ib{ahead(others[0]) >= ahead(others[1]) ? others[0] : others[1]};
	auto const ic{ib == others[0] ? others[1] : others[0]};
	std::array<std::size_t, 4> const order{ia, ib, ic, id};
	for (std::size_t i = 0; i < 4; ++i)
	{
		slip.ends[i] = ends[order[i]].first;
		slip.into[i] = ends[order[i]].second;
	}
	return slip;
}

std::vector<TTrack *> editor_mode::slip_taken_out(slip_set const &Slip, bool const Throughs) const
{
	switch_store::forget(Slip.record.name);
	auto const &style{*Slip.switches.front()};
	auto const velocity{speed_check::path_velocity(style, 0)};
	for (auto *part : Slip.parts)
		editor_track::retire(*part);
	std::vector<TTrack *> laid;
	if (false == Throughs)
		return laid;
	for (auto const &path : slip_throughs(Slip))
		if (auto *through{editor_track::create_path(style, path)})
		{
			editor_track::velocity(*through, velocity);
			laid.push_back(through);
		}
	editor_track::commit(laid);
	return laid;
}

std::optional<switch_store::record> editor_mode::crossing_record(TTrack const &Track) const
{
	if (Track.eType != tt_Normal || Track.m_editorremoved)
		return std::nullopt;
	auto const known{switch_store::named(Track)};
	if (false == known.has_value() || known->kind != "crossing" || known->parts.size() != 2)
		return std::nullopt;
	for (auto const &name : known->parts)
		if (auto const *part{simulation::Paths.find(name)}; part == nullptr || part->m_editorremoved || part->eType != tt_Normal || part->m_paths.size() != 1)
			return std::nullopt;
	return known;
}

std::array<segment_data, 2> editor_mode::slip_throughs(slip_set const &Slip)
{
	// from end to end, a straight where the slip is one, else heading out of the ends as the slip does
	auto const through = [&](std::size_t const From, std::size_t const To) {
		segment_data path;
		path.points[segment_data::point::start] = Slip.ends[From];
		path.points[segment_data::point::end] = Slip.ends[To];
		auto const chord{plan_of(Slip.ends[To] - Slip.ends[From])};
		if (auto const length{glm::length(chord)}; length > 1e-6 &&std::abs(geometry::cross(Slip.into[From], chord / length)) + std::abs(geometry::cross(Slip.into[To], chord / length)) > 1e-5)
		{
			auto const handle{length / 3.0};
			auto const rise{Slip.ends[To].y - Slip.ends[From].y};
			path.points[segment_data::point::control1] = {Slip.into[From].x * handle, rise / 3.0, Slip.into[From].y * handle};
			path.points[segment_data::point::control2] = {Slip.into[To].x * handle, -rise / 3.0, Slip.into[To].y * handle};
			path.radius = static_cast<float>(Slip.record.main_radius);
		}
		return path;
	};
	return {through(0, 1), through(2, 3)};
}

std::vector<segment_data> editor_mode::double_paths(double_switch const &Set)
{
	auto link{Set.link->m_paths.front()};
	if (auto const joint{Set.first->m_paths.front().points[segment_data::point::end]}; glm::distance(link.points[segment_data::point::start], joint) > glm::distance(link.points[segment_data::point::end], joint))
		link = geometry::reversed(link);
	return {Set.first->m_paths[0], Set.first->m_paths[1], Set.second->m_paths[0], Set.second->m_paths[1], link};
}

void editor_mode::record_slip(std::span<TTrack *const> const Parts, std::string_view const Kind, std::string_view const Type, double const Radius, double const Angle, std::string_view const Base, double const Middle) const
{
	if (Parts.empty() || scene::Layers.empty())
		return;
	switch_store::record entry;
	entry.kind = Kind;
	entry.name = Base;
	entry.type = Type;
	entry.radius = Radius;
	entry.middle = Middle;
	entry.ratio = Angle > 1e-9 ? 1.0 / std::tan(Angle) : 0.0;
	for (auto const *part : Parts)
		if (part != nullptr && false == part->name().empty() && (Kind == "crossing" || part->eType == tt_Switch))
			entry.parts.push_back(part->name());
	if (entry.parts.empty())
		return;
	auto const *first{simulation::Paths.find(entry.parts.front())};
	if (first == nullptr || first->m_paths.empty())
		return;
	switch_store::place(entry, *first);
	// a slip bent into a curve, as a switch is
	entry.main_radius = switch_store::main_radius_of(*first);
	switch_store::put(entry, alignment_store::layer_of(*first));
}

void editor_mode::render_switch_record(TTrack const &Track) const
{
	auto const known{switch_store::of(Track)};
	if (false == known.has_value())
	{
		ImGui::TextDisabled("%s", STR_C("The scenery keeps no type for this switch, it's read from its shape"));
		return;
	}
	auto entry{*known};
	ImGui::Text(STR_C("Type kept in the scenery: %s"), switch_store::label(entry).c_str());
	if (ImGui::IsItemHovered())
	{
		std::string text;
		if (entry.kind == "switch" && entry.main_radius != 0.0)
			text = format(STR_C("Diverging track R %.0f m: the R %.0f m of the type bent with the main track"), switch_store::branch_radius(entry), entry.radius);
		if (entry.source == "measured")
			text += (text.empty() ? "" : "\n") + std::string{STR_C("The size is measured from the shape of the switch, it's of no template of the catalogue")};
		if (false == text.empty())
			ImGui::SetTooltip("%s", text.c_str());
	}
	bool changed{false};
	if (begin_fields("##switchrecord", 0.4f))
	{
		field_row("Number##switchnumber");
		changed |= edit_dashed<32>("##switchnumber", entry.number);
		item_tooltip("Number of the switch in the station, as the plans of the station give it");
		field_row("Rail##switchrail");
		changed |= edit_dashed<16>("##switchrail", entry.rail);
		item_tooltip("Rail of the switch: 60E1, 49E1...");
		auto km{entry.km.has_value() ? *entry.km / 1000.0 : 0.0};
		field_row("Km of the start##switchkm");
		if (ImGui::InputDouble("##switchkm", &km, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			entry.km = km * 1000.0;
			changed = true;
		}
		item_tooltip("Chainage of the start of the switch; taken from the horizontal alignment kept in the scenery when the switch lies on one");
		ImGui::EndTable();
	}
	if (changed)
	{
		switch_store::put(entry, entry.layer);
		switch_store::settle();
	}
}
