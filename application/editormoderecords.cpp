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
#include <cstdio>

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
	return Before[static_cast<std::size_t>(std::find(Moving.begin(), Moving.end(), Track) - Moving.begin())];
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
		if (track != nullptr && false == track->m_editorremoved && track->eType == tt_Normal && false == track->m_paths.empty() && std::find(Moving.begin(), Moving.end(), track) == Moving.end() && lies_on(Record, *track))
			return false;
	return true;
}

bool carried_rigidly(int const Record, std::vector<TTrack *> const &Tracks, std::vector<TTrack *> const &Moving, std::vector<path_ends> const &Before, record_drift &Drift)
{
	auto const &first{ends_before(Moving, Before, Tracks.front())};
	auto const &path{Tracks.front()->m_paths.front()};
	auto const &start{path.points[segment_data::point::start]};
	auto const from{chord_direction(first.first, first.second)};
	auto const to{chord_direction(start, path.points[segment_data::point::end])};
	auto const angle{signed_angle(from, to)};
	auto const carried = [&](glm::dvec3 const &Old, glm::dvec3 const &Current) { return glm::distance(plan_of(start) + turned(plan_of(Old) - plan_of(first.first), angle), plan_of(Current)) <= 0.01; };
	for (auto const *track : Tracks)
	{
		auto const &was{ends_before(Moving, Before, track)};
		auto const &now{track->m_paths.front()};
		if (false == carried(was.first, now.points[segment_data::point::start]) || false == carried(was.second, now.points[segment_data::point::end]))
			return false;
	}
	if (glm::distance(first.first, start) > 1e-4 || std::abs(angle) > 1e-7)
	{
		alignment_store::transform(Record, first.first, from, start, to);
		++Drift.followed;
	}
	return true;
}

void measure_departure(int const Record, std::vector<TTrack *> const &Tracks, record_drift &Drift)
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
		notes.push_back(format(STR_C("%d path(s) left the design kept in %s, by up to %.2f m: Ctrl+Z takes it back, the route design lays them by the design again or forgets it"), Drift.departed, Drift.layer.c_str(), Drift.departure));
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
		entry.type = "-";
		entry.radius = Shape.radius;
		entry.ratio = Shape.ratio;
		entry.source = "measured";
	}
	return entry;
}

template <std::size_t Size>
bool edit_dashed(char const *Label, std::string &Value)
{
	std::array<char, Size> text{};
	std::snprintf(text.data(), text.size(), "%s", Value == "-" ? "" : Value.c_str());
	if (false == ImGui::InputText(Label, text.data(), text.size(), ImGuiInputTextFlags_EnterReturnsTrue) && false == ImGui::IsItemDeactivatedAfterEdit())
		return false;
	Value = text[0] != '\0' ? text.data() : "-";
	return true;
}

} // namespace

void editor_mode::records_watch_begin()
{
	auto &watch{m_record_watch};
	watch = {};
	watch.active = true;
	watch.moving = m_track_drag;
	for (auto const &link : m_track_gizmo_drag.switches)
		for (auto const &member : link.cluster)
			if (std::find(watch.moving.begin(), watch.moving.end(), member.first) == watch.moving.end())
				watch.moving.push_back(member.first);
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
		if (watch.whole.count(record) == 0 || false == carried_rigidly(record, tracks, watch.moving, watch.before, drift))
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
void editor_mode::record_switch(TTrack &Track, editor_track::switch_template const &Shape)
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

void editor_mode::record_slip(std::vector<TTrack *> const &Parts, std::string const &Kind, std::string const &Type, double const Radius, double const Angle, std::string const &Base)
{
	if (Parts.empty() || scene::Layers.empty())
		return;
	switch_store::record entry;
	entry.kind = Kind;
	entry.name = Base;
	entry.type = Type;
	entry.radius = Radius;
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
	if (entry.kind == "switch" && entry.main_radius != 0.0)
		ImGui::TextDisabled(STR_C("Diverging track R %.0f m: the R %.0f m of the type bent with the main track"), switch_store::branch_radius(entry), entry.radius);
	if (entry.source == "measured")
		ImGui::TextDisabled("%s", STR_C("Copied from a switch of the scenery: the size is measured from its shape"));
	ImGui::PushItemWidth(100.0f);
	bool changed{edit_dashed<32>(STR_C("Number##switchnumber"), entry.number)};
	item_tooltip("Number of the switch in the station, as the plans of the station give it");
	ImGui::SameLine();
	changed |= edit_dashed<16>(STR_C("Rail##switchrail"), entry.rail);
	item_tooltip("Rail of the switch: 60E1, 49E1...");
	auto km{entry.km.has_value() ? *entry.km / 1000.0 : 0.0};
	if (ImGui::InputDouble(STR_C("Km of the start##switchkm"), &km, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		entry.km = km * 1000.0;
		changed = true;
	}
	item_tooltip("Chainage of the start of the switch; taken from the horizontal alignment kept in the scenery when the switch lies on one");
	ImGui::PopItemWidth();
	if (changed)
	{
		switch_store::put(entry, entry.layer);
		switch_store::settle();
	}
}
