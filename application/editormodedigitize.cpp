/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the digitization of a scenery made without the editor: its lines kept with their designs, its switches with their types out of
// the catalogue, its crossovers as connections and the objects along the tracks bound to them, at one go

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodeshared.h"
#include "application/editoruilayer.h"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorConnections.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorSwitchStore.hpp"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "world/Track.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace
{

using geometry::bezier;
using geometry::plan_of;
using geometry::signed_angle;

// how far the size of a switch may be from the template to be of it: the length of the main and the diverging track, the angle
double constexpr kMainTolerance{0.10};
double constexpr kBranchTolerance{0.15};
double constexpr kAngleTolerance{0.0025};
// a plain track this short between the ends of two switches is a part of a slip
double constexpr kSlipLink{6.0};
// the longest insert of a crossover
double constexpr kInsertLongest{150.0};
// the tracks a crossover joins lie this far apart
double constexpr kSpacingLeast{2.5};
double constexpr kSpacingMost{15.0};
// radii of the diverging track the switches are made with; a measured one this close to one of them is taken for it
double constexpr kNominalRadii[]{150.0, 190.0, 205.0, 230.0, 265.0, 300.0, 500.0, 760.0, 1200.0};
double constexpr kNominalNear{0.04};

double nominal_radius(double const Radius)
{
	for (auto const nominal : kNominalRadii)
		if (std::abs(Radius - nominal) <= kNominalNear * nominal)
			return nominal;
	return std::round(Radius);
}

std::vector<segment_data const *> parts_of(segment_data const &Path)
{
	std::vector<segment_data const *> parts;
	if (Path.pieces.empty())
		parts.push_back(&Path);
	else
		for (auto const &piece : Path.pieces)
			parts.push_back(&piece);
	return parts;
}

double path_length(segment_data const &Path)
{
	double length{0.0};
	for (auto const *part : parts_of(Path))
		length += bezier{*part}.plan_length();
	return length;
}

glm::dvec2 heading(segment_data const &Path, bool const End)
{
	auto const parts{parts_of(Path)};
	bezier const curve{End ? *parts.back() : *parts.front()};
	auto const direction{plan_of(curve.first(End ? 1.0 : 0.0))};
	if (glm::length(direction) > 1e-9)
		return glm::normalize(direction);
	auto const chord{plan_of(Path.points[segment_data::point::end] - Path.points[segment_data::point::start])};
	return glm::length(chord) > 1e-9 ? glm::normalize(chord) : glm::dvec2{0.0, 1.0};
}

// in the plan, + to the left, at the middle of the longest part of the path
double curvature(segment_data const &Path)
{
	segment_data const *longest{nullptr};
	double length{-1.0};
	for (auto const *part : parts_of(Path))
		if (auto const l{bezier{*part}.plan_length()}; l > length)
		{
			length = l;
			longest = part;
		}
	if (longest == nullptr || false == geometry::has_controls(*longest))
		return 0.0;
	bezier const curve{*longest};
	auto const first{plan_of(curve.first(0.5))};
	auto const second{plan_of(curve.second(0.5))};
	auto const speed{glm::length(first)};
	return speed > 1e-9 ? (first.x * second.y - first.y * second.x) / (speed * speed * speed) : 0.0;
}

struct switch_size
{
	double main{0.0};
	double branch{0.0};
	double angle{0.0}; // between the ends of the two tracks
	double radius{0.0}; // of the diverging track of the straight switch the switch is made of
};

switch_size size_of(segment_data const &Main, segment_data const &Branch)
{
	switch_size size;
	size.main = path_length(Main);
	size.branch = path_length(Branch);
	size.angle = std::abs(signed_angle(heading(Main, true), heading(Branch, true)));
	auto const bend{std::abs(curvature(Branch) - curvature(Main))};
	size.radius = bend > 1e-6 ? 1.0 / bend : 0.0;
	return size;
}

// the template of the catalogue the size is of: the rail of the track and a movable nose decide between the same shapes
int matching(std::vector<editor_track::switch_template> const &Templates, std::vector<std::pair<int, switch_size>> const &Sizes, switch_size const &Size, std::string const &Rail, bool const Movable)
{
	int best{-1};
	auto score{std::numeric_limits<double>::max()};
	for (auto const &[index, size] : Sizes)
	{
		auto const &shape{Templates[index]};
		auto const main{std::abs(size.main - Size.main)};
		auto const branch{std::abs(size.branch - Size.branch)};
		auto const angle{std::abs(size.angle - Size.angle)};
		if (main > kMainTolerance || branch > kBranchTolerance || angle > kAngleTolerance)
			continue;
		auto value{main / kMainTolerance + branch / kBranchTolerance + angle / kAngleTolerance};
		if (false == Rail.empty() && shape.rail != Rail)
			value += 10.0;
		if (shape.movable != Movable)
			value += 5.0;
		if (value < score)
		{
			score = value;
			best = index;
		}
	}
	return best;
}

bool usable_switch(TTrack const *Track)
{
	return Track != nullptr && false == Track->m_editorremoved && Track->eType == tt_Switch && Track->m_paths.size() == 2;
}

bool named(TTrack const &Track)
{
	return false == Track.name().empty() && Track.name() != "none";
}

glm::dvec3 far_end(segment_data const &Path, editor_track::point_kind const Kind)
{
	return Path.points[Kind == editor_track::point_kind::start ? segment_data::point::end : segment_data::point::start];
}

// the end of the path meets an end of another switch, right there or over a short plain track: the paths of a slip do
bool towards_switch(TTrack const &Track, glm::dvec3 const &Point)
{
	for (auto const &[other, point] : editor_track::connected_points(Track, Point))
	{
		if (other == nullptr || other == &Track || other->m_editorremoved)
			continue;
		if (other->eType == tt_Switch && point.kind == editor_track::point_kind::end)
			return true;
		if (other->eType != tt_Normal || other->m_paths.size() != 1 || path_length(other->m_paths.front()) > kSlipLink)
			continue;
		for (auto const &[next, nextpoint] : editor_track::connected_points(*other, far_end(other->m_paths.front(), point.kind)))
			if (next != nullptr && next != other && next != &Track && next->eType == tt_Switch && nextpoint.kind == editor_track::point_kind::end)
				return true;
	}
	return false;
}

bool slip_part(TTrack const &Switch)
{
	return towards_switch(Switch, Switch.m_paths[0].points[segment_data::point::end]) && towards_switch(Switch, Switch.m_paths[1].points[segment_data::point::end]);
}

} // namespace

void editor_mode::digitize_switches(digitize_report &Report, bool const Write)
{
	auto const templates{editor_track::standard_switch_templates()};
	std::vector<std::pair<int, switch_size>> sizes;
	for (int i = 0; i < static_cast<int>(templates.size()); ++i)
	{
		auto const &shape{templates[i]};
		if (shape.source == nullptr && false == shape.double_slip && false == shape.twofold())
			sizes.emplace_back(i, size_of(shape.local[0], shape.local[1]));
	}
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == usable_switch(track))
			continue;
		++Report.switches;
		if (switch_store::named(*track).has_value())
		{
			++Report.known;
			continue;
		}
		if (slip_part(*track))
		{
			++Report.slips;
			continue;
		}
		auto const &main{track->m_paths[0]};
		auto const &branch{track->m_paths[1]};
		if (geometry::plan_distance(main.points[segment_data::point::start], branch.points[segment_data::point::start]) > geometry::same_point)
		{
			++Report.odd;
			Report.notes.push_back(track->name() + ": " + STR("the tracks of the switch don't start at the same point"));
			continue;
		}
		auto const size{size_of(main, branch)};
		auto const movable{track->SwitchExtension != nullptr && track->SwitchExtension->m_movablenose};
		auto const found{matching(templates, sizes, size, editor_track::rail_of(track), movable)};
		editor_track::switch_template shape;
		if (found >= 0)
		{
			shape = templates[found];
			++Report.designed;
			++Report.types[shape.label.substr(0, shape.label.find("  "))];
		}
		else
		{
			shape.source = track;
			shape.radius = nominal_radius(size.radius);
			shape.ratio = size.angle > 1e-6 ? 1.0 / std::tan(size.angle) : 0.0;
			++Report.measured;
			++Report.shapes[format("L %.2f / %.2f  1:%.2f  R %.0f", size.main, size.branch, shape.ratio, shape.radius)];
		}
		if (false == named(*track))
		{
			// the record finds the switch by its name; the names of the scenery stay as they are
			++Report.unnamed;
			continue;
		}
		if (Write && found >= 0)
			record_switch(*track, shape);
		else if (Write)
		{
			switch_store::record entry;
			entry.kind = "switch";
			entry.name = track->name();
			entry.parts = {entry.name};
			entry.radius = shape.radius;
			entry.ratio = shape.ratio;
			entry.source = "measured";
			auto const rail{editor_track::rail_of(track)};
			entry.rail = rail.empty() ? std::string{"-"} : rail;
			switch_store::place(entry, *track);
			switch_store::classify(entry, *track);
			switch_store::put(entry, alignment_store::layer_of(*track));
		}
		++Report.recorded;
	}
}

void editor_mode::digitize_crossovers(digitize_report &Report, bool const Write)
{
	std::set<std::pair<TTrack const *, TTrack const *>> seen;
	for (auto *first : simulation::Paths.sequence())
	{
		if (false == usable_switch(first) || slip_part(*first))
			continue;
		auto const &branch{first->m_paths[1]};
		auto const end{branch.points[segment_data::point::end]};
		// the insert: plain tracks from the end of the diverging track up to the end of the diverging track of another switch
		std::vector<TTrack *> insert;
		TTrack *second{nullptr};
		auto *from{static_cast<TTrack const *>(first)};
		auto point{end};
		double length{0.0};
		bool direct{false};
		while (second == nullptr && length <= kInsertLongest)
		{
			TTrack *next{nullptr};
			editor_track::point_ref at;
			for (auto const &[other, ref] : editor_track::connected_points(*from, point))
			{
				if (other == nullptr || other == from || other->m_editorremoved || std::find(insert.begin(), insert.end(), other) != insert.end())
					continue;
				if (usable_switch(other) && other != first && ref.path == 1 && ref.kind == editor_track::point_kind::end)
				{
					second = other;
					direct = insert.empty();
					break;
				}
				if (other->eType == tt_Normal && other->m_paths.size() == 1)
				{
					next = other;
					at = ref;
				}
			}
			if (second != nullptr || next == nullptr)
				break;
			insert.push_back(next);
			length += path_length(next->m_paths.front());
			point = far_end(next->m_paths.front(), at.kind);
			from = next;
		}
		if (second == nullptr || slip_part(*second))
			continue;
		if (seen.count({second, first}) > 0)
			continue;
		seen.insert({first, second});
		// the two switches face each other from two tracks side by side
		auto const along{heading(first->m_paths[0], false)};
		auto const back{heading(second->m_paths[0], false)};
		if (glm::dot(along, back) > -0.9)
			continue;
		auto const spacing{std::abs(geometry::cross(along, plan_of(second->m_paths[0].points[segment_data::point::start] - first->m_paths[0].points[segment_data::point::start])))};
		if (spacing < kSpacingLeast || spacing > kSpacingMost)
			continue;
		++Report.crossovers;
		auto const name = [](TTrack const *Track) { return name_of(*Track); };
		auto const note = [&](std::string const &Why) { Report.notes.push_back(name(first) + " + " + name(second) + ": " + Why); };
		if (false == switch_store::connections(*first).empty())
		{
			++Report.crossovers_known;
			continue;
		}
		if (direct || insert.size() != 1)
		{
			++Report.crossovers_odd;
			note(direct ? STR("the diverging tracks meet without an insert between them") : format(STR_C("the insert is made of %zu paths"), insert.size()));
			continue;
		}
		auto const one{switch_store::named(*first)};
		auto const other{switch_store::named(*second)};
		if (Write && (false == one.has_value() || false == other.has_value()))
		{
			++Report.crossovers_odd;
			note(STR("a switch of it has no type kept"));
			continue;
		}
		if (one.has_value() && other.has_value() && (std::abs(one->radius - other->radius) > kNominalNear * one->radius || std::abs(one->ratio - other->ratio) > kNominalNear * one->ratio))
		{
			++Report.crossovers_odd;
			note(format(STR_C("the switches differ: %s and %s"), switch_store::label(*one).c_str(), switch_store::label(*other).c_str()));
			continue;
		}
		if (Write)
		{
			auto entry{*one};
			entry.kind = "connection";
			entry.name = first->name();
			entry.parts = {first->name(), second->name()};
			entry.number = "-";
			switch_store::place(entry, *first);
			switch_store::classify(entry, *first);
			entry.type = connection::word(connection::kind::crossover);
			entry.a = spacing;
			entry.b = length;
			switch_store::put(entry, alignment_store::layer_of(*first));
			WriteLog(format("Editor: digitizing - crossover %s + %s kept, %s, spacing %.2f m, insert %.2f m", first->name().c_str(), second->name().c_str(), switch_store::label(entry).c_str(), spacing, length), logtype::generic);
		}
		++Report.crossovers_recorded;
	}
}

std::string editor_mode::digitize_scenery(bool const Write)
{
	digitize_report report;
	if (Write)
		digitize_all(&report.lines, &report.lines_failed);
	digitize_switches(report, Write);
	digitize_crossovers(report, Write);
	if (Write)
	{
		switch_store::settle();
		auto const scope{m_infra.scope};
		m_infra.scope = 3;
		infra_recognize();
		report.found = static_cast<int>(m_infra.candidates.size());
		auto const before{m_bindings.size()};
		infra_bind_chosen();
		report.bound = static_cast<int>(m_bindings.size() - before);
		m_infra.scope = scope;
	}
	auto text{format(STR_C("Switches: %d, kept already %d, of the catalogue %d, measured %d, parts of slips %d, odd %d"), report.switches, report.known, report.designed, report.measured, report.slips, report.odd)};
	if (report.unnamed > 0)
		text += format(STR_C(", without a name, left out %d"), report.unnamed);
	text += "\n" + format(STR_C("Crossovers: %d, kept already %d, recorded %d, left out %d"), report.crossovers, report.crossovers_known, report.crossovers_recorded, report.crossovers_odd);
	if (Write)
	{
		text = format(STR_C("Lines digitized: %d, paths left out: %d"), report.lines, report.lines_failed) + "\n" + text;
		text += "\n" + format(STR_C("Objects by the tracks: %d found, %d bound"), report.found, report.bound);
	}
	WriteLog("Editor: digitizing the scenery\n" + text, logtype::generic);
	std::vector<std::pair<int, std::string>> types;
	for (auto const &[label, count] : report.types)
		types.emplace_back(count, label);
	std::sort(types.rbegin(), types.rend());
	for (auto const &[count, label] : types)
		WriteLog(format("Editor: digitizing - %4d x %s", count, label.c_str()), logtype::generic);
	std::vector<std::pair<int, std::string>> shapes;
	for (auto const &[label, count] : report.shapes)
		shapes.emplace_back(count, label);
	std::sort(shapes.rbegin(), shapes.rend());
	for (std::size_t i = 0; i < std::min<std::size_t>(shapes.size(), 20); ++i)
		WriteLog(format("Editor: digitizing - measured %4d x %s", shapes[i].first, shapes[i].second.c_str()), logtype::generic);
	for (std::size_t i = 0; i < std::min<std::size_t>(report.notes.size(), 30); ++i)
		WriteLog("Editor: digitizing - " + report.notes[i], logtype::generic);
	if (Write)
		ui()->set_status(text, report.odd + report.crossovers_odd + report.lines_failed > 0);
	return text;
}
