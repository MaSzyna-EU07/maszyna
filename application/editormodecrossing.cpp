/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the level crossings: a road changed, or a rail track it crosses, brings the road to the rails, see editor/editorCrossing.hpp

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editormodeselftest.h"
#include "editor/editorFormat.hpp"
#include "editor/norms/norms.hpp"

#include "imgui/imgui.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "world/Road.h"
#include "world/CrossingPanels.h"
#include "world/Track.h"
#include "model/AnimModel.h"
#include "world/Sweep.h"
#include "world/TrackRails.h"

#include <algorithm>
#include <cmath>
#include <fstream>

namespace
{

template <class Type_> void add_once(std::vector<Type_ *> &Items, Type_ *Item)
{
	if (Item != nullptr && std::find(Items.begin(), Items.end(), Item) == Items.end())
		Items.push_back(Item);
}

// the height of the surface of the road at a point of the plan, with the roll counted in. false off the road
bool road_surface(glm::dvec3 const &Point, double &Height)
{
	auto const *road{editor_road::nearest(Point, 0.0)};
	if (road == nullptr)
		return false;
	auto const &state{road->definition()};
	auto const t{editor_road::nearest_parameter(state, Point)};
	auto const axis{state.point(t)};
	auto tangent{state.tangent(t)};
	glm::dvec2 along{tangent.x, tangent.z};
	if (glm::length(along) < 1e-9)
		return false;
	along = glm::normalize(along);
	glm::dvec2 const left{along.y, -along.x};
	auto const across{glm::dot(glm::dvec2{Point.x - axis.x, Point.z - axis.z}, left)};
	auto const roll{glm::radians(glm::mix(static_cast<double>(state.axis.rolls[0]), static_cast<double>(state.axis.rolls[1]), t))};
	Height = axis.y + across * std::tan(roll);
	return true;
}

void crossing_expectation(bool const Passed, std::string const &What, std::string const &Got, std::string const &Wanted)
{
	++editor_selftest::expectations();
	if (Passed)
	{
		WriteLog("SELFTEST PASS " + What);
		return;
	}
	++editor_selftest::failures();
	WriteLog("SELFTEST FAIL " + What + ": got " + Got + ", expected " + Wanted);
}

// the cant changes along the path of the track: a cant ramp
bool on_ramp(TTrack const &Track)
{
	return Track.eType == tt_Normal && false == Track.m_paths.empty() && std::abs(Track.m_paths.front().rolls[0] - Track.m_paths.front().rolls[1]) > 1e-3f;
}

// how far the cant ramp goes on from the end of the track, over the tracks joined there on which the cant changes as well
double ramp_beyond(TTrack const &Track, bool const Atend)
{
	double result{0.0};
	auto const *current{&Track};
	auto leaving{Atend};
	for (int guard = 0; guard < 200; ++guard)
	{
		auto const *next{leaving ? current->trNext : current->trPrev};
		auto const entered{leaving ? current->iNextDirection : current->iPrevDirection};
		if (next == nullptr || next == &Track || next->m_editorremoved || false == on_ramp(*next))
			break;
		result += next->Length();
		current = next;
		// entered by its start it's left by its end
		leaving = entered == 0;
	}
	return result;
}

} // namespace

void editor_mode::crossings_changed(std::vector<TTrack *> const &Tracks)
{
	if (false == m_crossing.armed || m_crossing.busy)
		return;
	auto const empty{m_crossing.roads.empty() && m_crossing.tracks.empty()};
	for (auto *track : Tracks)
		if (track != nullptr && (track->iCategoryFlag & 1) != 0)
			add_once(m_crossing.tracks, track);
	if (empty && false == m_crossing.tracks.empty())
		m_crossing.serial = m_history.empty() ? 0 : m_history.back().serial;
}

void editor_mode::crossings_changed(editor_road::record const &Record)
{
	if (false == m_crossing.armed || m_crossing.busy)
		return;
	auto const empty{m_crossing.roads.empty() && m_crossing.tracks.empty()};
	for (auto const &entry : Record.roads)
		add_once(m_crossing.roads, entry.first);
	for (auto *road : Record.roads_created)
		add_once(m_crossing.roads, road);
	if (empty && false == m_crossing.roads.empty())
		m_crossing.serial = m_history.empty() ? 0 : m_history.back().serial;
}

void editor_mode::crossings_changed(TAnimModel &Model)
{
	if (false == m_crossing.armed || m_crossing.busy)
		return;
	level_crossing::panel panel;
	if (false == editor_road::crossing_panel(Model, panel))
		return;
	auto const empty{m_crossing.roads.empty() && m_crossing.tracks.empty() && m_crossing.panels.empty()};
	m_crossing.panels.emplace_back(Model.location(), &Model);
	if (empty)
		m_crossing.serial = m_history.empty() ? 0 : m_history.back().serial;
}

void editor_mode::crossings_reshow(std::vector<road_node *> const &Roads)
{
	// a road cuts itself along the rails as it makes its geometry
	for (auto *road : Roads)
		if (road != nullptr && false == road->m_editorremoved)
		{
			road->hide();
			road->show();
		}
}

std::vector<road_node *> editor_mode::crossings_near(std::vector<TTrack *> const &Tracks) const
{
	std::vector<road_node *> result;
	for (auto *track : Tracks)
		if (false == track->m_editorremoved)
			for (auto *road : editor_road::roads_across(*track))
				add_once(result, road);
	// the roads cut along a track which moved away or went
	for (auto *road : simulation::Roads.sequence())
		if (road != nullptr && false == road->m_editorremoved)
			for (auto const *track : road->gap_tracks())
				if (std::find(Tracks.begin(), Tracks.end(), track) != Tracks.end())
					add_once(result, road);
	return result;
}

void editor_mode::crossings_after_history()
{
	// what the steps put back isn't a change for the crossings to follow, the steps put the roads back themselves; the roads across the
	// tracks put back are cut along the rails where they are now
	auto const tracks{std::move(m_crossing.tracks)};
	m_crossing.tracks.clear();
	m_crossing.roads.clear();
	m_crossing.panels.clear();
	auto roads{crossings_near(tracks)};
	// the roads at the crossings as well: they leave their surface out under the panels put back or taken away
	for (auto *road : simulation::Roads.sequence())
		if (road != nullptr && false == road->m_editorremoved && (false == road->gaps().empty() || false == road->covers().empty()))
			add_once(roads, road);
	crossings_reshow(roads);
}

namespace
{

// the track brought up by Raise at Point by its grade line: level over Flat either way of it, ramps beyond within the norms of the track
// (PLK ST-T1-A6 tab. 10.1, the grade; tab. 10.2, the vertical curves): a vertical curve, the ramp, a vertical curve, the grade the
// track has added to. The route along the track, the chainage of the point on it, the ramp and the curves; false with why it can't be
struct level_change
{
	editor_track::route route;
	std::vector<editor_track::route_sample> samples;
	double chainage{0.0};
	double raise{0.0};
	double flat{0.0};
	double ramp{0.0}; // m, each, its curves in
	double grade{0.0}; // of the ramp over the grade of the track
	double radius{0.0}; // of the vertical curves
	double curve() const
	{
		return radius * grade;
	}
	// the lift at a chainage, and its rise per metre
	double at(double const Chainage, double &Slope) const
	{
		auto const offset{std::abs(Chainage - chainage)};
		auto const sign{Chainage < chainage ? 1.0 : -1.0};
		Slope = 0.0;
		if (offset <= flat)
			return raise;
		auto const x{flat + ramp - offset}; // from the foot of the ramp up towards the crossing
		if (x <= 0.0)
			return 0.0;
		auto const t{curve()};
		if (x < t)
		{
			Slope = sign * x / radius;
			return x * x / (2.0 * radius);
		}
		if (x > ramp - t)
		{
			Slope = sign * (ramp - x) / radius;
			return raise - (ramp - x) * (ramp - x) / (2.0 * radius);
		}
		Slope = sign * grade;
		return t * t / (2.0 * radius) + grade * (x - t);
	}
	std::vector<double> breaks() const
	{
		std::vector<double> result;
		auto const t{curve()};
		for (auto const side : {-1.0, 1.0})
			for (auto const at : {flat, flat + t, flat + ramp - t, flat + ramp})
				result.push_back(chainage + side * at);
		std::sort(result.begin(), result.end());
		result.erase(std::unique(result.begin(), result.end(), [](double A, double B) { return B - A < 0.05; }), result.end());
		return result;
	}
};

bool plan_level(TTrack &Track, glm::dvec3 const &Point, double const Raise, double const Flat, level_change &Change, std::string &Reason)
{
	Change.route = editor_track::run_route(Track, 3000.0);
	Change.samples = editor_track::sample_route(Change.route, 1.0);
	if (Change.samples.size() < 2)
	{
		Reason = STR("the track is too short");
		return false;
	}
	auto best{std::numeric_limits<double>::max()};
	for (std::size_t i = 0; i + 1 < Change.samples.size(); ++i)
	{
		glm::dvec2 const a{Change.samples[i].position.x, Change.samples[i].position.z};
		glm::dvec2 const b{Change.samples[i + 1].position.x, Change.samples[i + 1].position.z};
		auto const run{b - a};
		auto const t{std::clamp(glm::dot(glm::dvec2{Point.x, Point.z} - a, run) / std::max(1e-12, glm::dot(run, run)), 0.0, 1.0)};
		if (auto const distance{glm::length(glm::dvec2{Point.x, Point.z} - (a + run * t))}; distance < best)
		{
			best = distance;
			Change.chainage = Change.samples[i].chainage + (Change.samples[i + 1].chainage - Change.samples[i].chainage) * t;
		}
	}
	Change.raise = Raise;
	Change.flat = Flat;
	// the norms of the track: the speed most of it is laid for, a main track of mixed traffic
	norms::query where;
	where.speed = editor_track::route_speed(Change.route, 100.0);
	auto const radii{norms::limits(norms::profile_id::plk, norms::parameter::vertical_radius, where)};
	auto const grades{norms::limits(norms::profile_id::plk, norms::parameter::grade, where)};
	Change.radius = radii.warn ? radii.warn->value : radii.prefer ? radii.prefer->value : 5000.0;
	auto const steepest{(grades.warn ? grades.warn->value : grades.block ? grades.block->value : 10.0) / 1000.0};
	// the grade the track has where the ramps go, at most
	double existing{0.0};
	for (auto const &sample : Change.samples)
		if (std::abs(sample.chainage - Change.chainage) <= Flat + 150.0)
			existing = std::max(existing, std::abs(sample.grade));
	auto const room{steepest - existing};
	if (room <= 1e-4)
	{
		Reason = format(STR_C("the track is as steep as the norms allow already (%.1f per mille, PLK ST-T1-A6 tab. 10.1)"), steepest * 1000.0);
		return false;
	}
	// the shortest ramp: its grade the square root of the raise over the radius, no steeper than leaves the track within the norms
	Change.grade = std::min(std::sqrt(Raise / Change.radius), room);
	Change.ramp = Raise / Change.grade + Change.radius * Change.grade;
	auto const reach{Flat + Change.ramp};
	if (Change.chainage - reach < 0.5 || Change.chainage + reach > Change.route.length - 0.5)
	{
		Reason = format(STR_C("the track doesn't go on far enough for the ramps: %.0f m either way of the crossing are needed (%.0f m level, a ramp of %.0f m, vertical curves of R %.0f m)"), reach, Flat,
		                Change.ramp, Change.radius);
		return false;
	}
	for (auto const &span : Change.route.spans)
		if (span.to > Change.chainage - reach && span.from < Change.chainage + reach && span.track->eType != tt_Normal)
		{
			Reason = format(STR_C("switch %s lies where the level would change"), span.track->name().c_str());
			return false;
		}
	return true;
}

} // namespace

void editor_mode::crossings_update()
{
	if (false == m_crossing.armed)
	{
		if (false == simulation::is_ready)
			return;
		m_crossing.armed = true;
		m_crossing.serial = m_history.empty() ? 0 : m_history.back().serial;
		m_crossing.armedserial = m_crossing.serial;
		return;
	}
	if (m_crossing.roads.empty() && m_crossing.tracks.empty() && m_crossing.panels.empty())
		return;
	// a gesture under way: the change isn't done yet
	if (ImGui::GetIO().MouseDown[0] || m_gizmo_using)
		return;
	auto const roads{std::move(m_crossing.roads)};
	auto const tracks{std::move(m_crossing.tracks)};
	auto const panels{std::move(m_crossing.panels)};
	m_crossing.roads.clear();
	m_crossing.tracks.clear();
	m_crossing.panels.clear();
	// a change the user made, a step of the history after the scenery was opened
	if (m_history.empty() || m_history.back().serial < m_crossing.serial || m_history.back().serial <= m_crossing.armedserial)
		return;
	std::vector<road_node *> candidates;
	for (auto *road : roads)
		if (false == road->m_editorremoved)
			add_once(candidates, road);
	auto const around{crossings_near(tracks)};
	for (auto *road : around)
		add_once(candidates, road);
	// the roads near where the panels were and are
	std::vector<glm::dvec3> places;
	for (auto const &[place, model] : panels)
	{
		places.push_back(place);
		auto const &instances{simulation::Instances.sequence()};
		if (std::find(instances.begin(), instances.end(), model) != instances.end())
			places.push_back(model->location());
	}
	for (auto *road : simulation::Roads.sequence())
	{
		if (road == nullptr || road->m_editorremoved)
			continue;
		for (auto const &place : places)
			for (int i = 0; i <= 8; ++i)
			{
				auto const point{road->definition().point(i / 8.0)};
				if (glm::length(glm::dvec2{point.x - place.x, point.z - place.z}) < 0.5 * road->definition().length() + 15.0)
				{
					add_once(candidates, road);
					i = 9;
					break;
				}
			}
	}
	if (candidates.empty())
		return;
	editor_road::record record;
	std::string text;
	bool refusal{false};
	bool warning{false};
	m_crossing.busy = true;
	for (auto *road : candidates)
	{
		if (road->m_editorremoved)
			continue;
		level_crossing::design_result result;
		std::string reason;
		if (editor_road::settle_crossing(*road, record, result, reason))
		{
			text = crossing_summary(result);
			refusal = false;
			// a break of the grade line within the 5% of difficult conditions: allowed, told in the colour of a warning
			warning = result.largest_break > 0.0005;
			m_crossing.fix.reset();
			m_crossing.ramp.reset();
			m_crossing.breaks.reset();
			// the panels offered for a crossing which has none yet
			m_crossing.laid = result.panels > 0 ? nullptr : road;
		}
		else if (false == result.on_ramp.empty() && result.on_ramp.front() < result.crossed_tracks.size())
		{
			// refused for a cant ramp: told, with how far the ramp goes
			auto const &rail{result.crossed_tracks[result.on_ramp.front()]};
			crossing_state::ramp_refusal ramp;
			ramp.track = const_cast<TTrack *>(static_cast<TTrack const *>(rail.source));
			ramp.road = road;
			ramp.back = rail.distance;
			ramp.ahead = rail.length - rail.distance;
			if (ramp.track != nullptr)
			{
				ramp.back += ramp_beyond(*ramp.track, false);
				ramp.ahead += ramp_beyond(*ramp.track, true);
			}
			text = format(STR_C("Level crossing refused: track %s is on a cant ramp here, no level crossing is made on one (Dz.U. 2025 poz. 1105, § 31 ust. 3)"), rail.name.c_str());
			refusal = true;
			m_crossing.fix.reset();
			m_crossing.breaks.reset();
			m_crossing.ramp = ramp;
		}
		else if (false == result.over.empty() && result.over.front().track < result.crossed_tracks.size())
		{
			// refused for the cant: told, with the fix
			auto const &over{result.over.front()};
			auto const &rail{result.crossed_tracks[over.track]};
			crossing_state::cant_fix fix;
			fix.track = const_cast<TTrack *>(static_cast<TTrack const *>(rail.source));
			fix.road = road;
			fix.cant = over.cant;
			fix.allowed = over.allowed;
			fix.radius = rail.radius > 0.0 ? rail.radius : std::abs(rail.curvature) > 1e-6 ? 1.0 / std::abs(rail.curvature) : 0.0;
			if (fix.radius > 0.0)
			{
				norms::query where;
				where.radius = fix.radius;
				where.speed = fix.track != nullptr ? editor_track::velocity(*fix.track) : 0.0;
				auto const limits{norms::limits(norms::profile_id::plk, norms::parameter::cant_deficiency, where)};
				auto const deficiency{limits.warn ? limits.warn->value : limits.block ? limits.block->value : 130.0};
				fix.speed = std::floor(norms::greatest_speed(fix.radius, fix.allowed, deficiency) / 10.0) * 10.0;
			}
			text = format(STR_C("Level crossing refused: track %s has a cant of %.0f mm, a crossing at %.0f degrees allows at most %.0f mm (the road over the crossing at most 8%%, Dz.U. 2025 poz. 1105, § 31 ust. 4)"),
			              rail.name.c_str(), over.cant, over.angle, over.allowed);
			refusal = true;
			m_crossing.fix = fix;
			m_crossing.ramp.reset();
			m_crossing.breaks.reset();
		}
		else if (false == result.breaks.empty() && result.largest_break > level_crossing::break_limit + 1e-4)
		{
			// refused for the break of the grade line at the heads of two tracks side by side: told, with the two fixes
			auto const &worst{*std::max_element(result.breaks.begin(), result.breaks.end(), [](auto const &A, auto const &B) { return A.value < B.value; })};
			auto const &a{result.crossed_tracks[worst.first]};
			auto const &b{result.crossed_tracks[worst.second]};
			crossing_state::break_refusal entry;
			entry.road = road;
			entry.first = const_cast<TTrack *>(static_cast<TTrack const *>(a.source));
			entry.second = const_cast<TTrack *>(static_cast<TTrack const *>(b.source));
			entry.value = worst.value;
			// the road goes from the one to the other: its axis crosses both
			auto const along{glm::dvec2{b.point.x - a.point.x, b.point.z - a.point.z}};
			// the level of the one, or the other, in the plane of the heads of the other
			double raise{0.0};
			int moved{0};
			if (level_crossing::one_plane(a, b, raise, moved))
			{
				auto *track{moved == 1 ? entry.second : entry.first};
				auto const &rail{moved == 1 ? b : a};
				auto const sine{glm::length(along) > 1e-9 ? std::max(0.2, std::abs(glm::dot(glm::normalize(along), rail.right()))) : 1.0};
				auto const flat{editor_road::outer_width(*road) / sine + 5.0};
				level_change change;
				std::string why;
				if (track != nullptr && plan_level(*track, rail.point, raise, flat, change, why))
				{
					entry.moved = track;
					entry.point = rail.point;
					entry.raise = raise;
					entry.flat = flat;
					entry.ramp = change.ramp;
				}
				else
					entry.unlevelled = why;
			}
			else
				entry.unlevelled = STR("their cants or grades differ, their levels alone can't put their heads in one plane");
			// the cants of both lowered alike
			entry.share = level_crossing::break_share(a, b, along);
			entry.cant = std::floor(entry.share * std::max(std::abs(a.cant()), std::abs(b.cant())) + 1e-6);
			for (auto const *rail : {&a, &b})
			{
				auto const radius{rail->radius > 0.0 ? rail->radius : std::abs(rail->curvature) > 1e-6 ? 1.0 / std::abs(rail->curvature) : 0.0};
				auto const *source{static_cast<TTrack const *>(rail->source)};
				if (radius <= 0.0 || source == nullptr)
					continue;
				norms::query where;
				where.radius = radius;
				where.speed = editor_track::velocity(*source);
				auto const limits{norms::limits(norms::profile_id::plk, norms::parameter::cant_deficiency, where)};
				auto const deficiency{limits.warn ? limits.warn->value : limits.block ? limits.block->value : 130.0};
				auto const speed{std::floor(norms::greatest_speed(radius, entry.share * std::abs(rail->cant()), deficiency) / 10.0) * 10.0};
				entry.speed = entry.speed < 0.0 ? speed : std::min(entry.speed, speed);
			}
			text = format(STR_C("Level crossing refused: the grade line of the road breaks by %.1f%% at the heads of tracks %s and %s side by side, over the 5%% Dz.U. 2025 poz. 1105 annex 5 pt 3 allows"),
			              worst.value * 100.0, a.name.c_str(), b.name.c_str());
			refusal = true;
			m_crossing.fix.reset();
			m_crossing.ramp.reset();
			m_crossing.breaks = entry;
		}
		else if (false == reason.empty() && reason != "no rails cross the road")
		{
			WriteLog(format("Editor: crossing of %s refused, the steepest approach %.4f, %d tracks, from %.2f to %.2f of the road", road->name().c_str(), result.steepest, result.tracks, result.from,
			                result.to));
			text = STR(reason);
			refusal = true;
			// a refusal of another kind: the windows of the ones before go
			m_crossing.fix.reset();
			m_crossing.ramp.reset();
			m_crossing.breaks.reset();
		}
	}
	if (false == record.empty())
	{
		push_road_snapshot(record);
		// taken back with the change which made it
		m_history.back().joined = true;
	}
	m_crossing.busy = false;
	// the roads across the tracks changed which weren't laid again: their gaps go where the rails are now, and they leave their surface
	// out under the panels as they lie now
	std::vector<road_node *> reshown;
	for (auto *road : candidates)
		if (std::none_of(record.roads.begin(), record.roads.end(), [road](auto const &Entry) { return Entry.first == road; }) &&
		    std::find(record.roads_created.begin(), record.roads_created.end(), road) == record.roads_created.end())
			reshown.push_back(road);
	crossings_reshow(reshown);
	if (false == text.empty())
	{
		if (warning && false == refusal)
			ui()->set_warning(text);
		else
			ui()->set_status(text, refusal);
		m_crossing.last = text;
		WriteLog("Editor: " + text, logtype::generic);
	}
}

void editor_mode::render_crossing_fix()
{
	if (m_crossing.breaks.has_value())
	{
		auto const &entry{*m_crossing.breaks};
		if (entry.road == nullptr || entry.road->m_editorremoved || entry.first == nullptr || entry.first->m_editorremoved || entry.second == nullptr || entry.second->m_editorremoved)
		{
			m_crossing.breaks.reset();
			return;
		}
		auto const *viewport{ImGui::GetMainViewport()};
		ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f, viewport->WorkPos.y + 120.0f), ImGuiCond_Appearing, ImVec2(0.5f, 0.0f));
		auto open{true};
		if (ImGui::Begin(STR_C("Level crossing##crossingfix"), &open, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse))
		{
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 34.0f);
			ImGui::TextUnformatted(format(STR_C("The grade line of the road breaks by %.1f%% at the heads of tracks %s and %s side by side, where it turns from the plane of the heads of the one onto the line to "
			                                    "the other; Dz.U. 2025 poz. 1105 annex 5 pt 3 allows 5%%, more only with an analysis of how the vehicles get over it. The road stays as it is."),
			                              entry.value * 100.0, entry.first->name().c_str(), entry.second->name().c_str())
			                           .c_str());
			if (entry.moved != nullptr)
			{
				if (ImGui::Button(STR_C("Level the tracks into one plane")))
					ui()->set_status(crossing_level_tracks(), false);
				ImGui::TextDisabled("%s", format(STR_C("Track %s up by %.3f m by its grade line: level %.0f m either way of the crossing, ramps of %.0f m beyond; the road over both without a break (annex 5 pt 1)"),
				                                 entry.moved->name().c_str(), entry.raise, entry.flat, entry.ramp)
				                              .c_str());
			}
			else
				ImGui::TextDisabled("%s", format(STR_C("The tracks can't be levelled into one plane: %s"), entry.unlevelled.c_str()).c_str());
			if (entry.share > 1e-3)
			{
				auto const label{entry.speed > 0.0 ? format(STR_C("Lower the cant at the crossing to %.0f mm (%.0f km/h)"), entry.cant, entry.speed)
				                                   : format(STR_C("Lower the cant at the crossing to %.0f mm"), entry.cant)};
				if (ImGui::Button(label.c_str()))
					ui()->set_status(crossing_lower_for_breaks(), false);
			}
			ImGui::PopTextWrapPos();
		}
		ImGui::End();
		if (false == open)
			m_crossing.breaks.reset();
		return;
	}
	// a crossing laid: its panels offered
	if (false == m_crossing.fix.has_value() && false == m_crossing.ramp.has_value() && m_crossing.laid != nullptr)
	{
		if (m_crossing.laid->m_editorremoved)
		{
			m_crossing.laid = nullptr;
			return;
		}
		auto const *viewport{ImGui::GetMainViewport()};
		ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f, viewport->WorkPos.y + 120.0f), ImGuiCond_Appearing, ImVec2(0.5f, 0.0f));
		auto open{true};
		if (ImGui::Begin(STR_C("Level crossing##crossingfix"), &open, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse))
		{
			ImGui::RadioButton(STR_C("CBP: the concrete panels of PKP PLK, made of the drawings of Id-111"), &m_crossing.system, 0);
			ImGui::RadioButton(STR_C("The models of the game (przejazdy/plyty_przejazd_wew_2_seg, _zew_2_seg)"), &m_crossing.system, 1);
			if (ImGui::Button(STR_C("Lay the crossing panels (inner and outer, bent along the tracks)")))
			{
				auto const done{m_crossing.system == 0 ? crossing_lay_panels(*m_crossing.laid, "cbp", {})
				                                       : crossing_lay_panels(*m_crossing.laid, "przejazdy/plyty_przejazd_wew_2_seg", "przejazdy/plyty_przejazd_zew_2_seg")};
				ui()->set_status(done, false);
				m_crossing.laid = nullptr;
			}
		}
		ImGui::End();
		if (false == open)
			m_crossing.laid = nullptr;
		return;
	}
	if (m_crossing.ramp.has_value())
	{
		auto const &ramp{*m_crossing.ramp};
		if (ramp.track == nullptr || ramp.track->m_editorremoved || ramp.road == nullptr || ramp.road->m_editorremoved)
		{
			m_crossing.ramp.reset();
			return;
		}
		auto const *viewport{ImGui::GetMainViewport()};
		ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f, viewport->WorkPos.y + 120.0f), ImGuiCond_Appearing, ImVec2(0.5f, 0.0f));
		auto open{true};
		if (ImGui::Begin(STR_C("Level crossing##crossingfix"), &open, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse))
		{
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
			ImGui::TextUnformatted(format(STR_C("Track %s changes its cant here, on a cant ramp; no level crossing is made on one (Dz.U. 2025 poz. 1105, § 31 ust. 3). The road stays as it is."),
			                              ramp.track->name().c_str())
			                           .c_str());
			ImGui::TextUnformatted(STR_C("Move the crossing off the cant ramp"));
			ImGui::TextDisabled("%s", format(STR_C("The ramp ends %.0f m back and %.0f m ahead along the track: the crossing is laid as the road gets past either end"), ramp.back, ramp.ahead).c_str());
			ImGui::PopTextWrapPos();
		}
		ImGui::End();
		if (false == open)
			m_crossing.ramp.reset();
		return;
	}
	if (false == m_crossing.fix.has_value())
		return;
	auto const &fix{*m_crossing.fix};
	if (fix.track == nullptr || fix.track->m_editorremoved || fix.road == nullptr || fix.road->m_editorremoved)
	{
		m_crossing.fix.reset();
		return;
	}
	auto const *viewport{ImGui::GetMainViewport()};
	ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f, viewport->WorkPos.y + 120.0f), ImGuiCond_Appearing, ImVec2(0.5f, 0.0f));
	auto open{true};
	if (ImGui::Begin(STR_C("Level crossing##crossingfix"), &open, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse))
	{
		ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
		ImGui::TextUnformatted(format(STR_C("Track %s has a cant of %.0f mm here; a level crossing allows at most %.0f mm (Dz.U. 2025 poz. 1105, § 31 ust. 4). The road stays as it is."),
		                              fix.track->name().c_str(), fix.cant, fix.allowed)
		                           .c_str());
		auto const label{fix.speed > 0.0 ? format(STR_C("Lower the cant of the track at the crossing to %.0f mm (%.0f km/h)"), fix.allowed, fix.speed)
		                                 : format(STR_C("Lower the cant of the track at the crossing to %.0f mm"), fix.allowed)};
		if (ImGui::Button(label.c_str()))
		{
			auto const done{crossing_lower_cant()};
			ui()->set_status(done, false);
		}
		ImGui::TextDisabled("%s", STR_C("Or move the road to where the cant of the track is lower: the crossing is laid as the road gets there"));
		ImGui::PopTextWrapPos();
	}
	ImGui::End();
	if (false == open)
		m_crossing.fix.reset();
}

std::string editor_mode::crossing_lower_cant()
{
	if (false == m_crossing.fix.has_value())
		return {};
	auto const fix{*m_crossing.fix};
	m_crossing.fix.reset();
	if (fix.track == nullptr || fix.track->m_editorremoved || fix.cant <= 0.0)
		return {};
	return crossing_scale_cants({fix.track}, fix.allowed / fix.cant, {fix.speed}, fix.allowed);
}

std::string editor_mode::crossing_scale_cants(std::vector<TTrack *> const &Tracks, double const Share, std::vector<double> const &Speeds, double const Allowed)
{
	// the curve each track lies in: the tracks joined on either side as long as they're canted, the ramps at its ends with them
	auto const canted = [](TTrack const &Track) {
		for (auto const &path : Track.m_paths)
			if (std::abs(path.rolls[0]) > 1e-4f || std::abs(path.rolls[1]) > 1e-4f)
				return true;
		return false;
	};
	std::vector<TTrack *> group;
	std::vector<double> speeds;
	for (std::size_t k = 0; k < Tracks.size(); ++k)
	{
		auto *first{Tracks[k]};
		if (first == nullptr || first->m_editorremoved || std::find(group.begin(), group.end(), first) != group.end())
			continue;
		auto const speed{k < Speeds.size() ? Speeds[k] : -1.0};
		group.push_back(first);
		speeds.push_back(speed);
		for (auto const forward : {false, true})
		{
			auto *current{first};
			for (int guard = 0; guard < 500; ++guard)
			{
				auto *next{forward ? current->trNext : current->trPrev};
				if (next == nullptr || next->m_editorremoved || next->eType != tt_Normal || false == canted(*next) || std::find(group.begin(), group.end(), next) != group.end())
					break;
				group.push_back(next);
				speeds.push_back(speed);
				current = next;
			}
		}
	}
	if (group.empty())
		return {};
	std::string reason;
	if (false == tracks_editable(group, reason))
		return format(STR_C("The cant can't be lowered: %s"), STR(reason).c_str());
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : group)
		states.emplace_back(track, editor_track::capture(*track));
	// the cant scaled along the whole curve, its ramps included, so it goes on as smoothly as before
	auto slowest{-1.0};
	for (std::size_t i = 0; i < group.size(); ++i)
	{
		auto *track{group[i]};
		for (auto &path : track->m_paths)
			path.rolls = {static_cast<float>(glm::degrees(std::asin(std::sin(glm::radians(static_cast<double>(path.rolls[0]))) * Share))),
			              static_cast<float>(glm::degrees(std::asin(std::sin(glm::radians(static_cast<double>(path.rolls[1]))) * Share)))};
		if (speeds[i] > 0.0 && (editor_track::velocity(*track) <= 0.0 || editor_track::velocity(*track) > speeds[i]))
			editor_track::velocity(*track, speeds[i]);
		if (speeds[i] > 0.0)
			slowest = slowest < 0.0 ? speeds[i] : std::min(slowest, speeds[i]);
	}
	push_track_snapshot(std::move(states));
	editor_track::commit(group);
	auto const text{slowest > 0.0 ? format(STR_P("The cant of the curve lowered to %.0f mm over %d track, the speed limited to %.0f km/h; Ctrl+Z takes it back",
	                                             "The cant of the curve lowered to %.0f mm over %d tracks, the speed limited to %.0f km/h; Ctrl+Z takes it back", static_cast<long long>(group.size())),
	                                       Allowed, static_cast<int>(group.size()), slowest)
	                              : format(STR_P("The cant of the curve lowered to %.0f mm over %d track; Ctrl+Z takes it back", "The cant of the curve lowered to %.0f mm over %d tracks; Ctrl+Z takes it back",
	                                             static_cast<long long>(group.size())),
	                                       Allowed, static_cast<int>(group.size()))};
	WriteLog("Editor: " + text, logtype::generic);
	return text;
}

std::string editor_mode::crossing_lower_for_breaks()
{
	if (false == m_crossing.breaks.has_value())
		return {};
	auto const entry{*m_crossing.breaks};
	m_crossing.breaks.reset();
	if (entry.share <= 1e-3 || entry.first == nullptr || entry.second == nullptr)
		return {};
	return crossing_scale_cants({entry.first, entry.second}, entry.share, {entry.speed, entry.speed}, entry.cant);
}

std::string editor_mode::crossing_level_tracks()
{
	if (false == m_crossing.breaks.has_value())
		return {};
	auto const entry{*m_crossing.breaks};
	m_crossing.breaks.reset();
	if (entry.moved == nullptr || entry.moved->m_editorremoved)
		return format(STR_C("The tracks can't be levelled into one plane: %s"), entry.unlevelled.c_str());
	level_change change;
	std::string why;
	if (false == plan_level(*entry.moved, entry.point, entry.raise, entry.flat, change, why))
		return format(STR_C("The tracks can't be levelled into one plane: %s"), why.c_str());
	std::vector<TTrack *> tracks;
	for (auto const &span : change.route.spans)
		tracks.push_back(span.track);
	std::string reason;
	if (false == tracks_editable(tracks, reason))
		return format(STR_C("The tracks can't be levelled into one plane: %s"), STR(reason).c_str());
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	auto const &samples{change.samples};
	editor_track::apply_profile(
	    change.route,
	    [&](double const Chainage) {
		    double slope{0.0};
		    return editor_track::sampled_elevation(samples, Chainage) + change.at(Chainage, slope);
	    },
	    [&](double const Chainage) {
		    double slope{0.0};
		    change.at(Chainage, slope);
		    return editor_track::sampled_grade(samples, Chainage) + slope;
	    },
	    change.breaks(), states, created);
	push_track_snapshot(std::move(states), std::move(created));
	auto const text{format(STR_C("Track %s brought up by %.3f m by its grade line, its heads in the plane of the heads of the track beside it: level %.0f m either way of the crossing, ramps of %.0f m at %.1f per mille with vertical curves of R %.0f m; Ctrl+Z takes it back"),
	                       entry.moved->name().c_str(), entry.raise, entry.flat, change.ramp, change.grade * 1000.0, change.radius)};
	WriteLog("Editor: " + text, logtype::generic);
	return text;
}

std::string editor_mode::crossing_lay_panels(road_node &Road, std::string const &Inner, std::string const &Outer)
{
	auto const rails{editor_road::rails_under(Road)};
	if (rails.empty())
		return STR("No rails cross the road");
	auto const halfwidth{editor_road::outer_width(Road)};
	auto const cbp{Inner == "cbp"};
	std::vector<std::string> notes;
	std::vector<sweep_node *> made;
	static int counter{0};
	auto const lay = [&](TTrack &Track, int const Path, std::string const &Model, double const From, double const To, double const Lateral, int const Side) {
		sweep_node::state state;
		state.model = Model;
		state.bend = true;
		state.step = 0.0;
		state.from = From;
		state.to = To;
		state.lateral = Lateral;
		// the copies stand on the points of the track: the top of the panel goes up to the head as the paths of the track have it, and
		// the road over the crossing (0.18 m over the points; the profile of the rail may be lower, 0.172 m for 60E1, hanging from the head)
		state.height = gauge::rail_height;
		state.side_anchor = Side;
		// the top of the panel at the head of the rail. A panel CBP is made with its top at 0 at the head (an outer one bent rising or
		// falling from there), so its origin goes there; a model of the game by its highest point
		state.height_anchor = cbp ? 0 : 2;
		state.tilt = true;
		state.pieces = {Track.m_paths[Path]};
		scene::node_data data;
		data.type = "sweep";
		data.range_max = -1.0;
		data.layer = Road.layer();
		do
		{
			data.name = "crossing_panel_" + std::to_string(++counter);
		} while (simulation::Sweeps.find(data.name) != nullptr);
		auto *sweep{new sweep_node(data)};
		sweep->define(state);
		sweep->mark_dirty();
		simulation::Sweeps.insert(sweep);
		scene::Layers.count(sweep->layer(), scene::layer_item::model);
		sweep->show();
		sweeps_captured(Track);
		made.push_back(sweep);
	};
	for (auto const &rail : rails)
	{
		auto *track{const_cast<TTrack *>(static_cast<TTrack const *>(rail.source))};
		if (track == nullptr || rail.turnout || rail.path >= static_cast<int>(track->m_paths.size()))
			continue;
		// along the track as far as the road reaches over it at the angle it crosses
		auto sine{1.0};
		glm::dvec2 road{0.0};
		if (auto const *piece{editor_road::nearest(rail.point, 1.0)}; piece != nullptr)
		{
			auto const tangent{piece->definition().tangent(editor_road::nearest_parameter(piece->definition(), rail.point))};
			glm::dvec2 const direction{tangent.x, tangent.z};
			if (glm::length(direction) > 1e-9)
			{
				sine = std::max(0.2, std::abs(glm::dot(glm::normalize(direction), rail.right())));
				road = direction;
			}
		}
		auto const length{track_rails::path_segment(*track, rail.path).GetLength()};
		auto const reach{halfwidth / sine + 0.25};
		if (cbp)
		{
			// the panels CBP: the gauges they're made for, the curves they go on (Id-111 ch. 3)
			auto const gauge{rail.gauge >= 1.5 ? 1520 : rail.gauge >= 1.4 ? 1435 : 0};
			if (gauge == 0)
			{
				notes.push_back(format(STR_C("track %s: CBP is made for the gauges of 1435 and 1520 mm only"), rail.name.c_str()));
				continue;
			}
			auto const radius{rail.radius > 0.0 ? rail.radius : std::abs(rail.curvature) > 1e-6 ? 1.0 / std::abs(rail.curvature) : 0.0};
			if (radius > 0.0 && radius < crossing_panels::minimum_radius - 0.5)
			{
				notes.push_back(format(STR_C("track %s: no CBP on a curve of R %.0f m, it goes on straights and on curves of R 600 m and more (Id-111)"), rail.name.c_str(), radius));
				continue;
			}
			auto const height{crossing_panels::height_for(track->rail_height())};
			// rows of 3 m panels as long as the road reaches over the track and more, centred on it: with room for the shift to the sleepers
			// and for the cut corners of the end panels, so the road ends on whole panels
			auto const count{std::clamp(static_cast<int>(std::ceil((2.0 * reach + 2.0 * (0.5 * crossing_panels::sleeper_spacing + crossing_panels::corner_along)) / crossing_panels::length - 1e-6)), 2,
			                            crossing_panels::most)};
			auto const span{count * crossing_panels::length};
			// the joints over the sleepers: the sleepers half a spacing in from the start of the path, as the track lays them
			auto const spacing{track->m_sleeper_enabled && track->m_sleeper_frequency > 0.01f ? static_cast<double>(track->m_sleeper_frequency) : crossing_panels::sleeper_spacing};
			if (std::abs(spacing - crossing_panels::sleeper_spacing) > 0.01)
				notes.push_back(format(STR_C("track %s: its sleepers are %.2f m apart, CBP lies on sleepers 0.60 m apart with the joints of the panels over them (Id-111): the joints are off the sleepers"),
				                       rail.name.c_str(), spacing));
			auto from{rail.distance - 0.5 * span};
			from = 0.5 * spacing + std::round((from - 0.5 * spacing) / spacing) * spacing;
			auto const to{from + span};
			lay(*track, rail.path, crossing_panels::name({crossing_panels::piece::inner_row, height, gauge, count}), from, to, 0.0, 3);
			auto const outside{0.5 * rail.gauge + rail.head + crossing_panels::outer_gap};
			// between two tracks the outer panels bent onto the line the road goes along from the one to the other
			for (auto const side : {1, -1})
			{
				crossing_panels::spec outer{crossing_panels::piece::outer_row, height, gauge, count};
				outer.slope = level_crossing::outer_slope(rail, side, rails, road, halfwidth);
				WriteLog(format("Editor: crossing panels %s along %s, %+d: %.4f", crossing_panels::name(outer).c_str(), rail.name.c_str(), side, outer.slope));
				lay(*track, rail.path, crossing_panels::name(outer), from, to, side * outside, 1);
			}
			continue;
		}
		auto const from{std::max(0.0, rail.distance - reach)};
		auto const to{std::min(length, rail.distance + reach)};
		// the inner panel between the rails, its middle on the axis; the outer ones from the field side of each head outwards
		lay(*track, rail.path, Inner, from, to, 0.0, 3);
		auto const outside{0.5 * rail.gauge + rail.head};
		lay(*track, rail.path, Outer, from, to, outside, 1);
		lay(*track, rail.path, Outer, from, to, -outside, 1);
	}
	auto noted{std::string{}};
	for (auto const &note : notes)
		noted += "; " + note;
	// the road has to be laid over the panels as they lie: where it can't be (its approaches past them too steep for a road this short),
	// they aren't laid
	if (std::string reason; false == made.empty() && false == editor_road::crossing_possible(Road, reason))
	{
		for (auto *sweep : made)
		{
			sweep->hide();
			sweep->m_editorremoved = true;
		}
		made.clear();
		notes.push_back(STR(reason));
		noted += "; " + STR(reason);
	}
	if (made.empty())
		return notes.empty() ? STR("No rails cross the road") : format(STR_C("No crossing panels laid%s"), noted.c_str());
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = made.front()->name();
	snap.sweeps_toggled = made;
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	// the road leaves its surface out under them: each piece of it
	editor_road::record record;
	for (auto *piece : editor_road::chain(Road))
		record.roads.emplace_back(piece, piece->definition());
	if (record.roads.empty())
		record.roads.emplace_back(&Road, Road.definition());
	crossings_changed(record);
	auto const text{(cbp ? format(STR_P("%zu row of the crossing panels CBP bent along the tracks: %s; Ctrl+Z takes them back",
	                                    "%zu rows of the crossing panels CBP bent along the tracks: %s; Ctrl+Z takes them back", static_cast<long long>(made.size())),
	                              made.size(), made.front()->definition().model.c_str())
	                       : format(STR_P("%zu crossing panel bent along the tracks: inner %s, outer %s; Ctrl+Z takes them back",
	                                      "%zu crossing panels bent along the tracks: inner %s, outer %s; Ctrl+Z takes them back", static_cast<long long>(made.size())),
	                                made.size(), Inner.c_str(), Outer.c_str())) +
	                noted};
	WriteLog("Editor: " + text, logtype::generic);
	return text;
}

std::string editor_mode::crossing_summary(level_crossing::design_result const &Result) const
{
	auto text{format(STR_P("Level crossing: the surface at the rail heads, %d track", "Level crossing: the surface at the rail heads, %d tracks", Result.tracks), Result.tracks)};
	if (Result.cant >= 0.5)
	{
		if (Result.cant - Result.cant_low >= 0.5)
			text += format(STR_C(", cant %.0f-%.0f mm"), Result.cant_low, Result.cant);
		else
			text += format(STR_C(", cant %.0f mm"), Result.cant);
	}
	if (Result.panels > 0)
		text += format(STR_P(", %d crossing panel on top", ", %d crossing panels on top", Result.panels), Result.panels);
	if (Result.steep_cross)
		text += " · " + format(STR_C("the road over the crossing at %.1f%%, over the %.0f%% of annex 5 Dz.U. 2025 poz. 1105"), Result.cross_slope * 100.0,
		                       (Result.tracks > 0 && false == Result.crossed_tracks.empty() && Result.crossed_tracks.front().gauge < 1.3 ? level_crossing::narrow_cross_slope_limit : level_crossing::cross_slope_limit) * 100.0);
	if (Result.steep_approach)
		text += " · " + format(STR_C("an approach of %.1f%% within 20 m of the rails, over the 3%% of annex 5 Dz.U. 2025 poz. 1105"), Result.steepest * 100.0);
	if (Result.angle < 60.0)
		text += " · " + format(STR_C("crossed at %.0f degrees: § 28 ust. 1 Dz.U. 2025 poz. 1105 allows 60-120 for a crossing of category D, 45-135 for the others"), Result.angle);
	if (Result.mismatch > 0.002)
		text += " · " + format(STR_C("the heads of tracks this close along the road are off the plane they share by up to %.0f mm"), Result.mismatch * 1000.0);
	if (Result.largest_break > 0.0005)
		text += " · " + format(STR_C("the grade line breaks by %.1f%% at the heads of the tracks side by side: allowed up to 5%% in difficult conditions only (Dz.U. 2025 poz. 1105, annex 5 pt 3); with their heads in one plane it wouldn't break (pt 1)"),
		                       Result.largest_break * 100.0);
	return text;
}

// roadlay <x0> <z0> <x1> <z1> <height> [lanes each way] [lane width]   a straight road laid as the road tool lays one, one step
// expect roadsurface <x> <z> <height> <tolerance>   the surface of the road at the point of the plan, the roll counted in
// expect railhead <track> <right|left> <distance> <tolerance>   the surface of the road over the head of the rail of the track where the road
//                                      crosses it, against the top of the head, at the middle of the head
// expect crossing <holds|lacks> <text>   what the last crossing laid told
// expect roadgaps <x> <z> <0|1>        the surface of the road at the point is cut away (a gap along a rail) or not
// expect roadpieces <count>            the pieces of the roads in the scenery
// expect crossingrefused <0|1> [cant|ramp|break]   a crossing was refused for the cant of a track, its fix offered, for a cant ramp, or
//                                      for the break of the grade line at the heads of two tracks side by side, its two fixes offered
// expect roadcovered <x> <z> <0|1>     the surface of the road at the point is left out under a crossing panel or not
// expect cbprows <count> [model]       the rows of the panels CBP laid along the tracks, all of them or the ones of the model named
// crossingfix [level|lower]            the fix of the crossing refused for the cant applied, as its button applies it; level: the tracks
//                                      levelled into one plane, lower: their cants lowered, for a crossing refused for the break
// crossinglaypanels <x> <z> [cbp | <inner> <outer>]   the crossing panels laid on the crossing of the road at the point, bent along
//                                      the tracks: the panels CBP made of their drawings, or the models named
// crossingholes <x> <z> [file]         the crossing of the road at the point checked for what shows through it, as built: rays straight
//                                      down every 2 cm over the road 32 m around, beside each edge of its surface, at slants over the
//                                      panels and rails, down each flangeway (road_crossing::check); the triangles and what's found
//                                      written to the file, for the map of it
// expect crossingholes <count>         what the last check found
// crossingprobe                        logs the pieces of the roads, their heights, rolls and gaps
// crossingpanels                       logs the crossing panels of the scenery, their outlines and tops
// roadlanes <0|1>                      the lanes of the roads drawn over the view, or not
// nodebank <0|1>                       the window of the node bank open, or closed
// workarea <surroundings|tracks|roads|terrain>   the field of work chosen
editor_mode::selftest_result editor_mode::selftest_crossing(std::string const &command, std::istringstream &words)
{
	if (command == "roadlay")
	{
		double x0{0.0}, z0{0.0}, x1{0.0}, z1{0.0}, height{0.0};
		int lanes{1};
		float width{3.5f};
		words >> x0 >> z0 >> x1 >> z1 >> height;
		if (words >> lanes)
			words >> width;
		road_node::state state;
		state.forward = state.backward = std::max(1, lanes);
		state.lanewidth = width;
		state.axis.points[segment_data::point::start] = {x0, height, z0};
		state.axis.points[segment_data::point::end] = {x1, height, z1};
		state.normalize();
		std::vector<road_node::state> states;
		for (auto const &piece : editor_road::refine({state.axis}, 50.0))
		{
			auto entry{state};
			entry.axis = piece;
			states.push_back(entry);
		}
		editor_road::record record;
		record.roads_created = editor_road::create(states);
		WriteLog(format("SELFTEST roadlay: %zu pieces from %.1f %.1f to %.1f %.1f at %.2f m", record.roads_created.size(), x0, z0, x1, z1, height));
		push_road_snapshot(std::move(record));
		return selftest_result::handled;
	}
	if (command == "crossingprobe")
	{
		for (auto *road : simulation::Roads.sequence())
		{
			if (road == nullptr || road->m_editorremoved)
				continue;
			auto const &state{road->definition()};
			WriteLog(format("SELFTEST crossingprobe %s: %.3f %.3f %.3f -> %.3f %.3f %.3f, rolls %.3f %.3f, gaps %zu, covers %zu", road->name().c_str(), state.axis.points[0].x, state.axis.points[0].y,
			                state.axis.points[0].z, state.axis.points[3].x, state.axis.points[3].y, state.axis.points[3].z, state.axis.rolls[0], state.axis.rolls[1], road->gaps().size(),
			                road->covers().size()));
		}
		return selftest_result::handled;
	}
	if (command == "crossinglaypanels")
	{
		// the panels laid on the crossing of the road at the point, as its button lays them
		double x{0.0}, z{0.0};
		std::string inner{"cbp"}, outer;
		words >> x >> z;
		if (std::string model; words >> model)
		{
			inner = model;
			if (words >> model)
				outer = model;
		}
		auto *road{editor_road::nearest({x, 0.0, z}, 0.0)};
		auto const done{road != nullptr ? crossing_lay_panels(*road, inner, outer) : std::string{"no road there"}};
		// as the button does: the offer of the panels taken
		m_crossing.laid = nullptr;
		WriteLog("SELFTEST crossinglaypanels: " + done);
		return selftest_result::handled;
	}
	if (command == "crossingreport")
	{
		// crossingreport <x> <z>   what the piece of the road nearest the point makes of the crossing: the tracks under it and around it,
		// whether it follows the surface of the crossing, the tracks it cuts along
		double x{0.0}, z{0.0};
		words >> x >> z;
		if (auto *road{editor_road::nearest({x, 0.0, z}, 0.0)}; road != nullptr)
		{
			WriteLog("SELFTEST crossingreport " + road->crossing_report());
			// and the tracks as the editor laid the road over them
			for (auto const &rail : editor_road::rails_under(*road))
				WriteLog(format("SELFTEST crossingreport   laid over %s@%.9f,%.9f,%.9f grade %.9f roll %.9f along %.9f,%.9f distance %.6f", rail.name.c_str(), rail.point.x, rail.point.y, rail.point.z,
				                rail.grade, rail.roll, rail.along.x, rail.along.y, rail.distance));
			auto const &axis{road->definition().axis};
			WriteLog(format("SELFTEST crossingreport   axis %.6f,%.6f,%.6f -> %.6f,%.6f,%.6f rolls %.6f %.6f", axis.points[0].x, axis.points[0].y, axis.points[0].z, axis.points[3].x, axis.points[3].y,
			                axis.points[3].z, axis.rolls[0], axis.rolls[1]));
		}
		return selftest_result::handled;
	}
	if (command == "crossingcovers")
	{
		// crossingcovers <x> <z>   the panels around the piece of the road at the point and whether it leaves its surface out under each
		double x{0.0}, z{0.0};
		words >> x >> z;
		auto *road{editor_road::nearest({x, 0.0, z}, 0.0)};
		if (road == nullptr)
			return selftest_result::handled;
		auto const &axis{road->definition().axis};
		auto const reach{editor_road::outer_width(*road)};
		auto const &a{axis.points[segment_data::point::start]};
		auto const &b{axis.points[segment_data::point::end]};
		auto const panels{crossing_panels_near(glm::min(glm::dvec2{a.x, a.z}, glm::dvec2{b.x, b.z}) - glm::dvec2{reach + 12.0}, glm::max(glm::dvec2{a.x, a.z}, glm::dvec2{b.x, b.z}) + glm::dvec2{reach + 12.0})};
		for (auto const &panel : panels)
		{
			glm::dvec2 low{1e9}, high{-1e9}, middle{0.0};
			for (auto const &corner : panel.outline)
			{
				low = glm::min(low, corner);
				high = glm::max(high, corner);
				middle += corner;
			}
			middle /= static_cast<double>(std::max<std::size_t>(1, panel.outline.size()));
			double height{0.0};
			auto const on{road_crossing::surface_at(axis, reach, middle, height)};
			auto const covered{false == road_crossing::covers(axis, reach, {panel}).empty()};
			WriteLog(format("SELFTEST crossingcovers %s: panel x %.3f..%.3f z %.3f..%.3f top %.3f, %zu faces, the middle %s at %.3f, covered %d", road->name().c_str(), low.x, high.x, low.y, high.y,
			                panel.top, panel.faces.size(), on ? "on the piece" : "off it", height, covered ? 1 : 0));
		}
		return selftest_result::handled;
	}
	if (command == "nodebank")
	{
		// the window of the node bank open or closed, as its item of the menu of the windows does
		int open{1};
		words >> open;
		ui()->nodebank().is_open = open != 0;
		return selftest_result::handled;
	}
	if (command == "crossingholes")
	{
		double x{0.0}, z{0.0};
		std::string file;
		words >> x >> z >> file;
		m_crossing.defects = -1;
		auto *road{editor_road::nearest({x, 0.0, z}, 0.0)};
		if (road == nullptr)
		{
			WriteLog("SELFTEST crossingholes: no road there");
			return selftest_result::handled;
		}
		road_crossing::built_crossing built;
		std::vector<std::string> pieces;
		built.tracks = editor_road::rails_under(*road);
		glm::dvec2 low{x - 32.0, z - 32.0}, high{x + 32.0, z + 32.0};
		auto const inside = [&](std::array<glm::dvec3, 3> const &T) {
			for (auto const &corner : T)
				if (corner.x >= low.x - 5.0 && corner.x <= high.x + 5.0 && corner.z >= low.y - 5.0 && corner.z <= high.y + 5.0)
					return true;
			return false;
		};
		auto const take = [&](std::vector<scene::shape_node> const &Shapes, std::vector<std::array<glm::dvec3, 3>> &Into) {
			for (auto const &shape : Shapes)
			{
				auto const &vertices{shape.data().vertices};
				for (std::size_t i = 0; i + 2 < vertices.size(); i += 3)
				{
					std::array<glm::dvec3, 3> const triangle{vertices[i].position, vertices[i + 1].position, vertices[i + 2].position};
					if (inside(triangle))
						Into.push_back(triangle);
				}
			}
		};
		// the road as built: the pieces of the roads around, their sides, their grooves and the strips beside the rails
		for (auto *piece : simulation::Roads.sequence())
		{
			if (piece == nullptr || piece->m_editorremoved)
				continue;
			auto const &axis{piece->definition().axis};
			auto const &a{axis.points[segment_data::point::start]};
			auto const &b{axis.points[segment_data::point::end]};
			if (std::max(a.x, b.x) < low.x - 10.0 || std::min(a.x, b.x) > high.x + 10.0 || std::max(a.z, b.z) < low.y - 10.0 || std::min(a.z, b.z) > high.y + 10.0)
				continue;
			built.footprint.emplace_back(piece->definition().axis, editor_road::outer_width(*piece));
			take(piece->create_shapes(), built.road);
			// which piece each triangle of the road comes from, for the map
			pieces.resize(built.road.size(), piece->name());
		}
		// the panels laid along the tracks
		for (auto *sweep : simulation::Sweeps.sequence())
		{
			if (sweep == nullptr || sweep->m_editorremoved)
				continue;
			auto name{sweep->definition().model};
			std::transform(name.begin(), name.end(), name.begin(), [](unsigned char C) { return static_cast<char>(std::tolower(C)); });
			if (name.rfind("@cbp/", 0) != 0 && name.find("przejazd") == std::string::npos && name.find("plyt") == std::string::npos)
				continue;
			take(sweep->create_shapes(), built.panels);
		}
		// the heads of the rails
		for (auto const &rail : built.tracks)
			for (auto const &triangle : road_crossing::rail_heads(rail, -25.0, 25.0))
				built.rails.push_back(triangle);
		std::size_t cracks{0};
		auto const defects{road_crossing::check(built, low, high, 0.02, 100000, &cracks)};
		m_crossing.defects = static_cast<int>(defects.size());
		std::map<std::string, int> kinds;
		for (auto const &entry : defects)
			++kinds[entry.kind];
		std::string summary;
		for (auto const &[kind, count] : kinds)
			summary += " " + kind + " " + std::to_string(count);
		WriteLog(format("SELFTEST crossingholes %.1f %.1f: %zu defects%s; %zu rays through cracks under %.2f mm; %zu triangles of the road, %zu of the panels, %zu of the rails, %zu tracks", x, z,
		                defects.size(), summary.c_str(), cracks, road_crossing::crack_width * 1000.0, built.road.size(), built.panels.size(), built.rails.size(), built.tracks.size()));
		for (std::size_t i = 0; i < defects.size() && i < 40; ++i)
			WriteLog(format("SELFTEST crossingholes defect %s at %.3f %.3f %.3f", defects[i].kind.c_str(), defects[i].point.x, defects[i].point.y, defects[i].point.z));
		if (false == file.empty())
		{
			std::ofstream out{file};
			out.precision(12);
			out << "low," << low.x << ',' << low.y << ",high," << high.x << ',' << high.y << '\n';
			auto const write = [&](char const *Kind, std::vector<std::array<glm::dvec3, 3>> const &Triangles) {
				for (auto const &t : Triangles)
					out << "T," << Kind << ',' << t[0].x << ',' << t[0].z << ',' << t[1].x << ',' << t[1].z << ',' << t[2].x << ',' << t[2].z << ',' << t[0].y << ',' << t[1].y << ',' << t[2].y << '\n';
			};
			for (std::size_t i = 0; i < built.road.size(); ++i)
			{
				auto const &t{built.road[i]};
				out << "T,road," << t[0].x << ',' << t[0].z << ',' << t[1].x << ',' << t[1].z << ',' << t[2].x << ',' << t[2].z << ',' << t[0].y << ',' << t[1].y << ',' << t[2].y << ','
				    << (i < pieces.size() ? pieces[i] : std::string{}) << '\n';
			}
			write("panel", built.panels);
			write("rail", built.rails);
			for (auto const &entry : defects)
				out << "D," << entry.kind << ',' << entry.point.x << ',' << entry.point.z << ',' << entry.point.y << '\n';
		}
		return selftest_result::handled;
	}
	if (command == "crossingfix")
	{
		// the fix of a crossing refused for the cant, as its button applies it; level, lower: the fixes of one refused for the break of
		// the grade line at the heads of two tracks side by side
		std::string which;
		words >> which;
		auto const done{which == "level" ? crossing_level_tracks() : which == "lower" ? crossing_lower_for_breaks() : crossing_lower_cant()};
		WriteLog("SELFTEST crossingfix: " + (done.empty() ? std::string{"nothing to fix"} : done));
		if (false == done.empty())
			ui()->set_status(done, false);
		return selftest_result::handled;
	}
	if (command == "workarea")
	{
		// the field of work, as the row of the fields over the toolbar chooses it
		std::string name;
		words >> name;
		choose_work_area(name == "tracks" ? work_area::tracks : name == "roads" ? work_area::roads : name == "terrain" ? work_area::terrain : work_area::surroundings);
		return selftest_result::handled;
	}
	if (command == "roadlanes")
	{
		// the lanes of the roads drawn over the view, or not (the button of the road tool)
		int shown{1};
		words >> shown;
		m_roadtool.lanes = shown != 0;
		return selftest_result::handled;
	}
	if (command == "crossingpanels")
	{
		for (auto const &panel : editor_road::crossing_panels(glm::dvec2{-1e9}, glm::dvec2{1e9}))
		{
			glm::dvec2 low{1e9}, high{-1e9};
			for (auto const &corner : panel.outline)
			{
				low = glm::min(low, corner);
				high = glm::max(high, corner);
			}
			WriteLog(format("SELFTEST crossingpanel: %zu corners, x %.3f..%.3f, z %.3f..%.3f, top %.3f", panel.outline.size(), low.x, high.x, low.y, high.y, panel.top));
		}
		return selftest_result::handled;
	}
	if (command != "expect")
		return selftest_result::unhandled;
	auto const start{words.tellg()};
	std::string what;
	words >> what;
	if (what == "roadsurface")
	{
		double x{0.0}, z{0.0}, height{0.0}, tolerance{0.001};
		words >> x >> z >> height >> tolerance;
		double got{0.0};
		auto const found{road_surface({x, 0.0, z}, got)};
		crossing_expectation(found && std::abs(got - height) <= tolerance, format("roadsurface %.2f %.2f", x, z), found ? format("%.4f", got) : std::string{"no road"}, format("%.4f +- %.4f", height, tolerance));
		return selftest_result::handled;
	}
	if (what == "railhead")
	{
		std::string name, side;
		double distance{0.0}, tolerance{0.001};
		words >> name >> side >> distance >> tolerance;
		auto const *track{simulation::Paths.find(name)};
		if (track == nullptr)
		{
			crossing_expectation(false, "railhead " + name, "no track", "a track");
			return selftest_result::handled;
		}
		auto rail{level_crossing::track_at(*track->CurrentSegment(), distance)};
		rail.gauge = std::abs(track->fTrackWidth);
		rail.head = track->rail_dimensions().head;
		auto const sign{side == "left" ? -1 : 1};
		auto const head{level_crossing::head_top(rail, sign, 0.0, 0.5 * rail.head)};
		double got{0.0};
		auto const found{road_surface(head, got)};
		// the surface goes through the heads
		auto const wanted{head.y};
		crossing_expectation(found && std::abs(got - wanted) <= tolerance, format("railhead %s %s %.1f", name.c_str(), side.c_str(), distance), found ? format("%.4f", got) : std::string{"no road"},
		                     format("%.4f +- %.4f", wanted, tolerance));
		return selftest_result::handled;
	}
	if (what == "crossing")
	{
		std::string mode, text;
		words >> mode;
		std::getline(words, text);
		text.erase(0, text.find_first_not_of(' '));
		auto const holds{m_crossing.last.find(text) != std::string::npos};
		crossing_expectation(mode == "lacks" ? false == holds : holds, "crossing " + mode + " " + text, m_crossing.last, text);
		return selftest_result::handled;
	}
	if (what == "crossingrefused")
	{
		int wanted{0};
		std::string kind{"cant"};
		words >> wanted >> kind;
		auto const refused{kind == "ramp" ? m_crossing.ramp.has_value() : kind == "break" ? m_crossing.breaks.has_value() : m_crossing.fix.has_value()};
		crossing_expectation(refused == (wanted != 0), "crossingrefused " + kind, refused ? "1" : "0", std::to_string(wanted));
		return selftest_result::handled;
	}
	if (what == "crossingholes")
	{
		int wanted{0};
		words >> wanted;
		crossing_expectation(m_crossing.defects == wanted, "crossingholes", std::to_string(m_crossing.defects), std::to_string(wanted));
		return selftest_result::handled;
	}
	if (what == "roadcovered")
	{
		double x{0.0}, z{0.0};
		int wanted{0};
		words >> x >> z >> wanted;
		auto inside{false};
		for (auto const *road : simulation::Roads.sequence())
			if (road != nullptr && false == road->m_editorremoved)
				for (auto const &cover : road->covers())
					inside |= level_crossing::touches(cover, {x, z}, {x, z});
		crossing_expectation(inside == (wanted != 0), format("roadcovered %.3f %.3f", x, z), inside ? "1" : "0", std::to_string(wanted));
		return selftest_result::handled;
	}
	if (what == "cbprows")
	{
		int wanted{0};
		std::string model;
		words >> wanted >> model;
		int count{0};
		for (auto const *sweep : simulation::Sweeps.sequence())
			if (sweep != nullptr && false == sweep->m_editorremoved && sweep->definition().model.rfind("@cbp/", 0) == 0 && (model.empty() || sweep->definition().model == model))
				++count;
		crossing_expectation(count == wanted, "cbprows " + model, std::to_string(count), std::to_string(wanted));
		return selftest_result::handled;
	}
	if (what == "roadpieces")
	{
		int wanted{0};
		words >> wanted;
		int count{0};
		for (auto const *road : simulation::Roads.sequence())
			if (road != nullptr && false == road->m_editorremoved)
				++count;
		crossing_expectation(count == wanted, "roadpieces", std::to_string(count), std::to_string(wanted));
		return selftest_result::handled;
	}
	if (what == "roadgaps")
	{
		double x{0.0}, z{0.0};
		int wanted{0};
		words >> x >> z >> wanted;
		auto const *road{editor_road::nearest({x, 0.0, z}, 0.0)};
		auto inside{false};
		if (road != nullptr)
			for (auto const &gap : road->gaps())
				inside |= level_crossing::touches(gap, {x, z}, {x, z});
		crossing_expectation(inside == (wanted != 0), format("roadgaps %.3f %.3f", x, z), inside ? "1" : "0", std::to_string(wanted));
		return selftest_result::handled;
	}
	words.clear();
	words.seekg(start);
	return selftest_result::unhandled;
}
