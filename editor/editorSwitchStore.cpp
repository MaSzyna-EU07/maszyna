/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorRecords.hpp"

#include "world/Track.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <cstdlib>
#include <set>
#include <sstream>

namespace switch_store
{

namespace
{

using geometry::plan_of;

std::string const kMark{"//$r"};
double const kTolerance{0.02};

std::vector<record> g_records;
std::vector<record> g_base; // as they were before the changes not yet in the undo history
bool g_changed{false};
bool g_loaded{false};

// the lines and their format: editor_records::read_switches() and switch_lines()

glm::dvec2 direction_of(TTrack const &Track)
{
	geometry::bezier const curve{Track.m_paths.front()};
	auto const direction{plan_of(curve.first(0.0))};
	return glm::length(direction) > 1e-12 ? glm::normalize(direction) : glm::dvec2{0.0, 1.0};
}

TTrack *usable(std::string const &Name)
{
	if (Name.empty() || Name == "-")
		return nullptr;
	auto *track{simulation::Paths.find(Name)};
	return track != nullptr && false == track->m_editorremoved && false == track->m_paths.empty() ? track : nullptr;
}

bool in_place(record const &Record, TTrack const &Track)
{
	return geometry::plan_distance(Track.m_paths.front().points[segment_data::point::start], Record.start) <= kTolerance && glm::dot(direction_of(Track), Record.direction) > 0.9999;
}

glm::dvec4 measure(TTrack const &Track)
{
	auto const &main{Track.m_paths.front()};
	auto const start{plan_of(main.points[segment_data::point::start])};
	auto const along{direction_of(Track)};
	glm::dvec2 const across{-along.y, along.x};
	auto const local = [&](glm::dvec3 const &Point) {
		auto const offset{plan_of(Point) - start};
		return glm::dvec2{glm::dot(offset, along), glm::dot(offset, across)};
	};
	auto const mainend{local(main.points[segment_data::point::end])};
	auto const branchend{Track.m_paths.size() > 1 ? local(Track.m_paths[1].points[segment_data::point::end]) : glm::dvec2{0.0}};
	return {mainend.x, mainend.y, branchend.x, branchend.y};
}

bool in_shape(record const &Record, TTrack const &Track)
{
	if (false == Record.shaped)
		return in_place(Record, Track);
	auto const now{measure(Track)};
	for (int i = 0; i < 4; ++i)
		if (std::abs(now[i] - Record.shape[i]) > kTolerance)
			return false;
	return true;
}

void mark(scene::layer_handle const Layer)
{
	if (false == scene::Layers.valid(Layer))
		return;
	std::vector<record> records;
	std::copy_if(g_records.begin(), g_records.end(), std::back_inserter(records), [&](record const &Entry) { return Entry.layer == Layer; });
	scene::Layers.mark(Layer, kMark, editor_records::switch_lines(records, scene::Layers.layer(Layer).context_insert()));
}

} // namespace

void reset()
{
	g_records.clear();
	g_base.clear();
	g_changed = false;
	g_loaded = false;
}

void load()
{
	if (g_loaded || scene::Layers.empty())
		return;
	g_loaded = true;
	g_records.clear();
	for (auto const layer : scene::Layers.marked_layers(kMark))
		for (auto &entry : editor_records::read_switches(scene::Layers.marked(layer, kMark), scene::Layers.layer(layer).context_insert(), layer))
			g_records.push_back(std::move(entry));
	g_base = g_records;
	g_changed = false;
	if (false == g_records.empty())
		WriteLog("Editor: " + std::to_string(g_records.size()) + " switch record(s) read from the scenery", logtype::generic);
}

std::vector<record> const &records()
{
	load();
	return g_records;
}

bool has(std::string const &Name)
{
	auto const &all{records()};
	return std::any_of(all.begin(), all.end(), [&](record const &Record) { return Record.name == Name; });
}

std::optional<record> named(TTrack const &Track)
{
	load();
	auto const &name{Track.name()};
	if (name.empty())
		return std::nullopt;
	for (auto const &entry : g_records)
		if (entry.kind != "connection" && std::find(entry.parts.begin(), entry.parts.end(), name) != entry.parts.end())
			return entry;
	return std::nullopt;
}

std::vector<record> connections(TTrack const &Track)
{
	load();
	std::vector<record> result;
	auto const &name{Track.name()};
	if (name.empty())
		return result;
	for (auto const &entry : g_records)
		if (entry.kind == "connection" && std::find(entry.parts.begin(), entry.parts.end(), name) != entry.parts.end())
			result.push_back(entry);
	return result;
}

std::optional<record> of(TTrack const &Track)
{
	auto const found{named(Track)};
	if (false == found.has_value())
		return std::nullopt;
	auto const *first{usable(found->parts.front())};
	if (first == nullptr || false == in_shape(*found, *first))
		return std::nullopt;
	return found;
}

std::vector<std::string> follow()
{
	load();
	std::vector<std::string> moved;
	std::set<scene::layer_handle> layers;
	for (auto &entry : g_records)
	{
		auto const *first{usable(entry.parts.front())};
		if (first == nullptr || false == entry.shaped || false == in_shape(entry, *first) || in_place(entry, *first))
			continue;
		entry.start = first->m_paths.front().points[segment_data::point::start];
		entry.direction = direction_of(*first);
		entry.km = alignment_store::chainage_at(entry.start);
		moved.push_back(entry.name);
		layers.insert(entry.layer);
	}
	for (auto const layer : layers)
		mark(layer);
	g_changed = g_changed || false == moved.empty();
	return moved;
}

double branch_radius(record const &Record)
{
	if (Record.radius <= 0.0)
		return 0.0;
	auto const side{Record.hand == 'L' ? -1.0 : 1.0};
	auto const curvature{side / Record.radius + (Record.main_radius != 0.0 ? 1.0 / Record.main_radius : 0.0)};
	return std::abs(curvature) > 1e-9 ? 1.0 / std::abs(curvature) : 0.0;
}

void put(record Entry, scene::layer_handle const Layer)
{
	load();
	if (auto const *first{usable(Entry.parts.empty() ? std::string{} : Entry.parts.front())})
	{
		Entry.shape = measure(*first);
		Entry.shaped = true;
	}
	auto const existing{std::find_if(g_records.begin(), g_records.end(), [&](record const &Other) { return Other.name == Entry.name && Other.kind == Entry.kind; })};
	if (existing != g_records.end())
	{
		Entry.layer = existing->layer;
		if (Entry.number == "-")
			Entry.number = existing->number;
		*existing = std::move(Entry);
		g_changed = true;
		mark(existing->layer);
		return;
	}
	Entry.layer = Layer;
	g_records.push_back(std::move(Entry));
	g_changed = true;
	mark(g_records.back().layer);
}

void forget(std::string const &Name)
{
	load();
	std::set<scene::layer_handle> layers;
	for (auto const &entry : g_records)
		if (entry.name == Name)
			layers.insert(entry.layer);
	auto const count{g_records.size()};
	g_records.erase(std::remove_if(g_records.begin(), g_records.end(), [&](record const &Entry) { return Entry.name == Name; }), g_records.end());
	g_changed = g_changed || g_records.size() != count;
	for (auto const layer : layers)
		mark(layer);
}

void rename(std::map<std::string, std::string> const &Parts, std::map<std::string, std::string> const &Bases)
{
	load();
	auto const renamed = [](std::map<std::string, std::string> const &Names, std::string &Name) {
		auto const found{Names.find(Name)};
		if (found == Names.end() || found->second == Name)
			return false;
		Name = found->second;
		return true;
	};
	std::set<scene::layer_handle> layers;
	for (auto &entry : g_records)
	{
		auto changed{renamed(entry.kind == "slip" || entry.kind == "crossing" ? Bases : Parts, entry.name)};
		for (auto &part : entry.parts)
			changed = renamed(Parts, part) || changed;
		if (changed)
			layers.insert(entry.layer);
	}
	for (auto const layer : layers)
		mark(layer);
	g_changed = g_changed || false == layers.empty();
}

void number(std::string const &Name, std::string const &Number)
{
	load();
	for (auto &entry : g_records)
		if (entry.kind != "connection" && entry.name == Name && entry.number != Number)
		{
			entry.number = Number;
			mark(entry.layer);
			g_changed = true;
		}
}

void tidy()
{
	load();
	follow();
	std::set<scene::layer_handle> layers;
	for (auto &entry : g_records)
	{
		if (entry.kind != "switch" || usable(entry.parts.front()) != nullptr)
			continue;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->m_paths.empty() || track->name().empty() || false == in_place(entry, *track))
				continue;
			entry.name = track->name();
			entry.parts = {entry.name};
			layers.insert(entry.layer);
			break;
		}
	}
	auto const gone = [](record const &Entry) { return std::any_of(Entry.parts.begin(), Entry.parts.end(), [](std::string const &Part) { return usable(Part) == nullptr; }); };
	for (auto const &entry : g_records)
		if (gone(entry))
			layers.insert(entry.layer);
	g_records.erase(std::remove_if(g_records.begin(), g_records.end(), gone), g_records.end());
	for (auto const layer : layers)
		mark(layer);
	g_changed = g_changed || false == layers.empty();
}

std::string label(record const &Record)
{
	auto text{Record.type};
	if (Record.rail != "-")
		text += ' ' + Record.rail + '-';
	else
		text += ' ';
	if (Record.radius > 0.0)
		text += Record.middle > 0.0 ? format("%.0f/%.0f-", Record.radius, Record.middle) : format("%.0f-", Record.radius);
	if (Record.ratio > 0.0)
		text += format(std::abs(Record.ratio - std::round(Record.ratio)) < 1e-3 ? "1:%.0f" : "1:%.1f", Record.ratio);
	if (Record.hand == 'L' || Record.hand == 'R')
		text += ' ' + std::string(1, Record.hand == 'L' ? 'L' : 'P');
	if (Record.main_radius != 0.0)
		text += format(", Rg %.0f", std::abs(Record.main_radius));
	return text;
}

std::optional<std::vector<record>> take_changes()
{
	if (false == g_changed)
		return std::nullopt;
	auto before{std::move(g_base)};
	g_base = g_records;
	g_changed = false;
	return before;
}

void settle()
{
	g_base = g_records;
	g_changed = false;
}

void restore(std::vector<record> Records)
{
	std::set<scene::layer_handle> layers;
	for (auto const *list : {&g_records, &Records})
		for (auto const &entry : *list)
			layers.insert(entry.layer);
	g_records = std::move(Records);
	for (auto const layer : layers)
		mark(layer);
	settle();
}

void place(record &Entry, TTrack const &First)
{
	auto const &main{First.m_paths.front()};
	geometry::bezier const curve{main};
	Entry.start = main.points[segment_data::point::start];
	auto const direction{geometry::plan_of(curve.first(0.0))};
	Entry.direction = glm::length(direction) > 1e-12 ? glm::normalize(direction) : glm::dvec2{0.0, 1.0};
	Entry.km = alignment_store::chainage_at(Entry.start);
}

double main_radius_of(TTrack const &Switch)
{
	geometry::bezier const main{Switch.m_paths[0]};
	auto const first{geometry::plan_of(main.first(0.0))};
	auto const last{geometry::plan_of(main.first(1.0))};
	auto const turn{glm::length(first) > 1e-12 && glm::length(last) > 1e-12 ? geometry::signed_angle(glm::normalize(first), glm::normalize(last)) : 0.0};
	auto const radius{std::abs(turn) > 1e-6 ? main.plan_length() / turn : 0.0};
	return std::abs(radius) > 20000.0 ? 0.0 : radius;
}

void classify(record &Entry, TTrack const &Switch)
{
	auto const &diverging{Switch.m_paths[1]};
	Entry.hand = geometry::cross(Entry.direction, geometry::plan_of(diverging.points[segment_data::point::end] - Entry.start)) > 0.0 ? 'R' : 'L';
	Entry.main_radius = main_radius_of(Switch);
	if (Entry.type == "Rz" || Entry.type == "Rlj" || Entry.type == "Rld")
		Entry.type = Entry.main_radius == 0.0 ? "Rz" : (Entry.main_radius > 0.0) == (Entry.hand == 'R') ? "Rlj" : "Rld";
}

} // namespace switch_store
