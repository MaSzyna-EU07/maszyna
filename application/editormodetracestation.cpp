/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Stations and heads of turnouts traced on the aerial photo (quirky-crafting-summit, Etap 5): laying from the photo, a click on a track
// starts the line with a turnout of the catalogue into it, fitted to the points clicked along its diverging track, and a click on a track
// ends it with one; from a click on a track to one on a track alongside, with nothing but the turnouts between, a crossover of the solver
// of the connections. A run of the points keeping one spacing from a track beside it is laid exactly alongside it. The turnouts go in by
// the code of the switch tool, the line between them as the lay tool lays one from a free end, all of it one step of the history

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editormodeshared.h"
#include "application/editorflow.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/norms/norms.hpp"
#include "editor/editorSwitchCurve.hpp"

#include "world/Track.h"
#include "scene/scene.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include "application/editormodetrackshared.h"

using namespace track_mode;

namespace
{

double constexpr kStationReach{1.5}; // m from the axis of a track a click starts or ends a line with a turnout into it
double constexpr kStationEndAway{1.0}; // m from a free end: a click there goes on from the end (as kTraceSnap), no turnout
double constexpr kStationApart{80.0}; // m from the first click on the same straight a click is along the turnout, it ends nothing
double constexpr kNeighbourReach{12.0}; // m, a track a run of the points may keep alongside
double constexpr kAlongsideLeast{40.0}; // m, the shortest run alongside a track
double constexpr kCrossoverOff{0.3}; // m, a click between the two of a crossover is on it within this, or 3 sigmas
double constexpr kCrossoverReach{25.0}; // m from the start of the switch on the other track the click on it is

std::vector<glm::dvec2> sampled(segment_data const &Path, double const Step)
{
	std::vector<glm::dvec2> result;
	std::vector<segment_data> pieces{Path.pieces};
	if (pieces.empty())
		pieces.push_back(Path);
	for (auto const &piece : pieces)
	{
		bezier const curve{piece};
		auto const steps{std::clamp(static_cast<int>(std::ceil(curve.plan_length() / Step)), 1, 2000)};
		for (int i = result.empty() ? 0 : 1; i <= steps; ++i)
			result.push_back(plan_of(curve.point(static_cast<double>(i) / steps)));
	}
	return result;
}

// the axis of the run of plain tracks the track is in, up to the switches, in the plan
std::vector<glm::dvec2> run_axis(TTrack &Track)
{
	editor_track::chain run;
	std::string error;
	std::vector<glm::dvec2> result;
	if (false == editor_track::find_run(Track, 5000.0, run, error) || run.tracks.empty())
		return sampled(Track.m_paths.front(), 2.0);
	for (std::size_t i = 0; i < run.tracks.size(); ++i)
	{
		auto points{sampled(run.tracks[i]->m_paths.front(), 2.0)};
		if (false == run.forward[i])
			std::reverse(points.begin(), points.end());
		result.insert(result.end(), points.begin() + (result.empty() ? 0 : 1), points.end());
	}
	return result;
}

} // namespace

void editor_mode::station_templates()
{
	auto &tool{m_switch};
	if (tool.collected)
		return;
	tool.templates = editor_track::standard_switch_templates();
	tool.collected = true;
}

editor_mode::station_catalogue const &editor_mode::station_shapes(std::string const &Rail) const
{
	auto &catalogue{m_station_catalogue};
	auto const &templates{m_switch.templates};
	if (catalogue.rail == Rail && catalogue.templates == templates.size() && false == catalogue.index.empty())
		return catalogue;
	catalogue = {};
	catalogue.rail = Rail;
	catalogue.templates = templates.size();
	for (int i = 0; i < static_cast<int>(templates.size()); ++i)
	{
		auto const &entry{templates[i]};
		// the plain turnouts of the catalogue; one of a shape, the movable nose of a frog not seen on the photo
		if (entry.source != nullptr || entry.own || entry.double_slip || entry.twofold() || entry.type != "Rz" || entry.rail != Rail || entry.movable || entry.radius <= 0.0 || entry.length <= 0.0)
			continue;
		auto const same{std::ranges::any_of(catalogue.index, [&](int const Other) {
			auto const &other{templates[Other]};
			return std::abs(other.radius - entry.radius) < 0.5 && std::abs(other.ratio - entry.ratio) < 0.01 && std::abs(other.length - entry.length) < 0.01;
		})};
		if (same)
			continue;
		catalogue.index.push_back(i);
		catalogue.shapes.push_back(trace_station::shape_of(entry.local[1], entry.length, entry.frog_straight > 0.0 ? 0.0 : 1.0 / entry.radius));
	}
	return catalogue;
}

editor_mode::track_under editor_mode::station_track_at(glm::dvec3 const &Point) const
{
	track_under result;
	TTrack *found{nullptr};
	auto best{kStationReach};
	for (auto *section : simulation::Region->sections(Point, 40.f))
		for (auto const &cell : section->m_cells)
			for (auto *track : cell.m_paths)
			{
				if (track->eType != tt_Normal || track->m_editorremoved || track->m_paths.empty() || (track->iCategoryFlag & 15) != kRailCategory)
					continue;
				auto const distance{plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, Point)), Point)};
				if (distance < best)
				{
					best = distance;
					found = track;
				}
			}
	if (found == nullptr)
		return result;
	// by a free end the line goes on from the end, or ends at it
	if (editor_track::find_free_end(nullptr, kRailCategory, Point, kStationEndAway, {}).track != nullptr)
		return result;
	result.track = found;
	result.straight = editor_track::is_straight(*found, m_straights.tolerance);
	if (result.straight)
		result.line = editor_track::find_straight(*found, m_straights.tolerance);
	return result;
}

bool editor_mode::station_end_at(glm::dvec3 const &Point) const
{
	auto const &tool{m_lay};
	if (false == lay_traced() || tool.points.empty())
		return false;
	auto const under{station_track_at(Point)};
	if (under.track == nullptr)
		return false;
	// a click along the turnout the line starts with: on the straight it starts off, near the first click
	if (tool.start.track == nullptr && plan_distance(Point, tool.points.front()) < kStationApart)
	{
		auto const first{station_track_at(tool.points.front())};
		if (first.track == under.track || std::ranges::find(first.line.tracks, under.track) != first.line.tracks.end())
			return false;
	}
	return true;
}

std::optional<editor_mode::traced_turnout> editor_mode::station_turnout(track_under const &Under, std::vector<glm::dvec3> const &Points, std::vector<double> const &Sigmas, bool const AtEnd, std::string &Refusal,
                                                                         traced_turnout const *Other) const
{
	if (false == Under.straight || Under.line.tracks.empty())
	{
		Refusal = STR("A turnout traced on the photo goes into a straight track. Into a curve, lay it with the turnout tool, then trace on from the end of its diverging track");
		return std::nullopt;
	}
	if (model_on() && model_track(Under.track) != nullptr)
	{
		Refusal = STR("On a line of the model the turnout is laid with the turnout tool: lay it there, then trace on from the end of its diverging track");
		return std::nullopt;
	}
	auto const setup{setup_now()};
	auto const told{editor_track::rail_of(Under.track)};
	auto const rail{told.empty() ? setup.rail : told};
	auto const &catalogue{station_shapes(rail)};
	if (catalogue.shapes.empty())
	{
		Refusal = format(STR_C("The catalogue has no turnouts of the rail %s to fit"), rail.c_str());
		return std::nullopt;
	}
	auto const &line{Under.line};
	trace_station::straight const main{plan_of(line.start), line.direction};
	std::vector<glm::dvec2> points;
	std::vector<double> sigmas;
	for (std::size_t k = 0; k < Points.size(); ++k)
	{
		auto const i{AtEnd ? Points.size() - 1 - k : k};
		points.push_back(plan_of(Points[i]));
		sigmas.push_back(i < Sigmas.size() ? Sigmas[i] : 0.1);
	}
	// where the points don't tell: the one the line starts with (the heads of a station alike), else the one chosen in the switch tool where
	// it's of these, else the usual one of a station
	int preferred{-1};
	auto const shape_of_template = [&](int const Template) {
		auto const found{std::ranges::find(catalogue.index, Template)};
		return found != catalogue.index.end() ? static_cast<int>(found - catalogue.index.begin()) : -1;
	};
	if (Other != nullptr)
		preferred = shape_of_template(Other->shape);
	if (preferred < 0 && m_switch.armed >= 0)
		preferred = shape_of_template(m_switch.armed);
	if (preferred < 0)
		preferred = shape_of_template(usual_switch(rail, 40.0));
	auto fit{trace_station::fit_turnout(main, catalogue.shapes, points, sigmas, preferred)};
	if (false == fit.valid)
		return std::nullopt;
	traced_turnout result;
	result.track = Under.track;
	result.line = line;
	result.shape = catalogue.index[fit.shape];
	result.at_end = AtEnd;
	auto const &shape{m_switch.templates[result.shape]};
	// as the switch tool writes it on a switch laid on a straight
	result.speed = switch_curve::branch_speed(shape.radius, shape.ratio, 0.0, 0.0, shape.movable).speed;
	result.label = format("%s · %.0f km/h", switch_name_translated(shape.title()).c_str(), result.speed);
	if (shape.length > line.length - 0.02)
	{
		Refusal = format(STR_C("The straight is %.1f m long, too short for the %.1f m switch"), line.length, shape.length);
		return std::nullopt;
	}
	// the straight of plain track it's on, between the switches on the line (and the other turnout of the line traced), and the straight
	// the norms ask between turnouts (ST-T1-A6 11.3 pkt 7): moved there, if it's nearer
	auto const length{shape.length};
	auto const from{fit.direction > 0 ? fit.start : fit.start - length};
	auto const middle{from + length * 0.5};
	auto low{0.0};
	auto high{line.length};
	auto before{false};
	auto after{false};
	auto const taken = [&](double const A, double const B) {
		if (B <= middle)
		{
			if (B >= low)
			{
				low = B;
				before = true;
			}
		}
		else if (A >= middle)
		{
			if (A <= high)
			{
				high = A;
				after = true;
			}
		}
		else
		{
			low = high = middle;
			before = after = true;
		}
	};
	for (std::size_t i = 0; i < line.tracks.size(); ++i)
	{
		if (line.tracks[i]->eType != tt_Switch)
			continue;
		auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
		auto const a{line.along(path.points[segment_data::point::start])};
		auto const b{line.along(path.points[segment_data::point::end])};
		taken(std::min(a, b), std::max(a, b));
	}
	for (auto const atend : {false, true})
		if (auto const *outside{editor_track::outside_neighbour(line, atend)}; outside != nullptr && outside->eType == tt_Switch)
			(atend ? after : before) = true;
	if (Other != nullptr && false == Other->paths.empty() && std::ranges::find(line.tracks, Other->track) != line.tracks.end())
	{
		auto const &main{Other->paths.front()};
		auto const a{line.along(main.points[segment_data::point::start])};
		auto const b{line.along(main.points[segment_data::point::end])};
		taken(std::min(a, b), std::max(a, b));
	}
	norms::query where;
	where.speed = result.speed;
	where.kind = setup.traffic;
	where.track = setup.track;
	where.features = setup.speed >= 160.0 ? norms::feature::fast_line : 0u;
	for (auto const &limits : {norms::limits(setup.profile, norms::parameter::switch_straight, where), norms::hard_limits(norms::parameter::switch_straight, where)})
	{
		if (limits.warn.has_value())
			result.least = std::max(result.least, limits.warn->value);
		else if (limits.prefer.has_value())
			result.least = std::max(result.least, limits.prefer->value);
		if (limits.block.has_value())
			result.least = std::max(result.least, limits.block->value);
	}
	auto const place{trace_station::place_in_head(from, length, low, high, before, after, result.least)};
	if (false == place.fits)
	{
		Refusal = format(STR_C("No room for %s here: %.1f m of plain track between the turnouts, it needs %.1f m and %g m of straight to each of them"), result.label.c_str(), std::max(0.0, high - low), length,
		                 result.least);
		return std::nullopt;
	}
	if (std::abs(place.moved) > 1e-6)
	{
		fit = trace_station::turnout_at(main, catalogue.shapes, fit, fit.start + place.moved, points);
		result.moved = place.moved;
		result.note = format(STR_C("moved %.1f m (straight between turnouts at least %g m)"), std::abs(place.moved), result.least);
	}
	result.fit = fit;
	auto const direction{line.direction * static_cast<double>(fit.direction)};
	result.start = line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * fit.start;
	result.paths = editor_track::place_switch(shape, result.start, direction, fit.side, line.grade * fit.direction);
	if (result.paths.size() < 2)
		return std::nullopt;
	result.end = result.paths[1].points[segment_data::point::end];
	result.hold.position = plan_of(result.end);
	result.hold.heading = fit.heading;
	result.hold.curvature = fit.curvature;
	result.hold.fixed = std::abs(fit.curvature) > 1e-9;
	// the line runs into the turnout it ends with
	if (AtEnd)
	{
		result.hold.heading = -fit.heading;
		result.hold.curvature = -fit.curvature;
	}
	return result;
}

editor_mode::traced_station const &editor_mode::traced_station_of(std::vector<glm::dvec3> const &Points) const
{
	auto const &tool{m_lay};
	std::vector<double> key;
	for (auto const &point : Points)
		key.insert(key.end(), {point.x, point.y, point.z});
	auto const setup{setup_now()};
	key.insert(key.end(), {static_cast<double>(m_track_edits), tool.start.track != nullptr ? 1.0 : 0.0, m_station_laying ? 1.0 : 0.0, lay_traced() ? 1.0 : 0.0, static_cast<double>(m_switch.armed),
	                       static_cast<double>(m_switch.templates.size()), setup.speed, static_cast<double>(std::hash<std::string>{}(setup.rail)), tool.sigma_given});
	for (std::size_t i = 0; i < Points.size() && i < tool.sigmas.size(); ++i)
		key.push_back(tool.sigmas[i]);
	if (m_station_memo.has_value() && m_station_memo->first == key)
		return m_station_memo->second;
	traced_station result;
	auto const done = [&]() -> traced_station const & {
		m_station_memo = std::make_pair(std::move(key), std::move(result));
		return m_station_memo->second;
	};
	if (false == lay_traced() || m_station_laying || Points.empty() || m_switch.templates.empty())
		return done();
	std::vector<double> sigmas;
	for (std::size_t i = 0; i < Points.size(); ++i)
		sigmas.push_back(i < tool.sigmas.size() && tool.sigmas[i] > 0.0 ? tool.sigmas[i] : click_sigma_at(Points[i]));
	auto const fromend{tool.start.track != nullptr};
	std::size_t first{fromend ? std::size_t{1} : std::size_t{0}};
	std::size_t last{Points.size()};
	// the line starting on a track: a turnout into it, once there's a point after the first one
	if (false == fromend)
		if (auto const under{station_track_at(Points.front())}; under.track != nullptr)
		{
			result.valid = true;
			std::string refusal;
			if (Points.size() >= 2)
				result.start = station_turnout(under, Points, sigmas, false, refusal, nullptr);
			else if (false == under.straight)
				station_turnout(under, Points, sigmas, false, refusal, nullptr);
			if (false == refusal.empty())
			{
				result.refusal = refusal;
				return done();
			}
			if (result.start.has_value())
				first = result.start->fit.used;
		}
	// ending on a track: a turnout into that one; refused only where it's no crossover either
	std::optional<track_under> ending;
	std::string refused;
	if (Points.size() >= 2 && station_end_at(Points.back()))
		if (auto const under{station_track_at(Points.back())}; under.track != nullptr)
		{
			result.valid = true;
			ending = under;
			result.end = station_turnout(under, Points, sigmas, true, refused, result.start.has_value() ? &*result.start : nullptr);
			if (result.end.has_value())
				last = Points.size() - std::min(Points.size(), result.end->fit.used);
		}
	// from a track to one alongside it, nothing but the turnouts between: a crossover, the solver's
	if (result.start.has_value() && ending.has_value() && ending->straight)
	{
		auto const &a{result.start->line};
		auto const &b{ending->line};
		auto const apart{std::abs(cross(a.direction, plan_of(b.start - a.start)))};
		if (std::abs(cross(a.direction, b.direction)) <= std::sin(glm::radians(1.0)) && apart >= 2.5 && apart <= 15.0 && std::ranges::find(a.tracks, ending->track) == a.tracks.end())
		{
			auto &request{result.request};
			request.type = connection::kind::crossover;
			request.from = connection::of(a);
			request.to = {connection::of(b)};
			request.along = request.from.along(result.start->start);
			request.forward = glm::dot(a.direction * static_cast<double>(result.start->fit.direction), request.from.direction) >= 0.0 ? 1 : -1;
			request.first = request.second = m_switch.templates[result.start->shape];
			request.target = request.to.front().along(Points.back());
			auto const planned{connection::plan(request)};
			auto fits{planned.valid && planned.switches.size() == 2};
			if (fits)
			{
				std::vector<std::vector<glm::dvec2>> lines;
				for (auto const &placed : planned.switches)
					for (auto const &path : placed.paths)
						lines.push_back(sampled(path, 1.0));
				for (auto const &insert : planned.inserts)
					lines.push_back(sampled(insert, 1.0));
				for (std::size_t i = 1; fits && i + 1 < Points.size(); ++i)
				{
					auto off{std::numeric_limits<double>::max()};
					for (auto const &each : lines)
						off = std::min(off, trace_station::off_line(each, plan_of(Points[i])));
					fits = off <= std::max(kCrossoverOff, 3.0 * sigmas[i]);
				}
				auto const &other{planned.switches[1]};
				fits = fits && false == other.paths.empty() &&
				       std::abs(request.to.front().along(Points.back()) - request.to.front().along(other.paths.front().points[segment_data::point::start])) <= kCrossoverReach + request.second.length;
			}
			if (fits)
			{
				result.crossover = true;
				result.connection = planned;
				result.end.reset();
				return done();
			}
		}
	}
	if (false == refused.empty())
	{
		result.refusal = refused;
		return done();
	}
	for (auto i{first}; i < last; ++i)
	{
		result.points.push_back(Points[i]);
		result.sigmas.push_back(sigmas[i]);
		result.clicked.push_back(Points[i]);
		result.indices.push_back(i);
	}
	if (result.start.has_value() && result.end.has_value() && result.points.empty())
	{
		result.refusal = STR("Click points along the track between the turnouts");
		return done();
	}
	// a run alongside a track: of the turnouts, or the one beside the middle of the points
	std::vector<std::pair<std::string, std::vector<glm::dvec2>>> neighbours;
	for (auto const *turnout : {result.start.has_value() ? &*result.start : nullptr, result.end.has_value() ? &*result.end : nullptr})
		if (turnout != nullptr)
			neighbours.emplace_back(name_of(*turnout->track), std::vector<glm::dvec2>{plan_of(turnout->line.start), plan_of(turnout->line.end)});
	if (result.points.size() >= 3)
	{
		auto const &middle{result.points[result.points.size() / 2]};
		TTrack *beside{nullptr};
		auto best{kNeighbourReach};
		for (auto *section : simulation::Region->sections(middle, 40.f))
			for (auto const &cell : section->m_cells)
				for (auto *track : cell.m_paths)
				{
					if (track->eType != tt_Normal || track->m_editorremoved || track->m_paths.empty() || (track->iCategoryFlag & 15) != kRailCategory)
						continue;
					auto const distance{plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, middle)), middle)};
					if (distance >= 1.0 && distance < best)
					{
						best = distance;
						beside = track;
					}
				}
		if (beside != nullptr && std::ranges::none_of(neighbours, [&](auto const &Each) { return Each.first == name_of(*beside); }))
			neighbours.emplace_back(name_of(*beside), run_axis(*beside));
	}
	std::vector<glm::dvec2> plan;
	for (auto const &point : result.points)
		plan.push_back(plan_of(point));
	trace_station::parallel_run best;
	for (auto const &[name, axis] : neighbours)
	{
		auto const run{trace_station::find_parallel(plan, result.sigmas, axis, kAlongsideLeast)};
		if (run.found && (false == best.found || run.last - run.first > best.last - best.first))
		{
			best = run;
			result.alongside = traced_alongside{run.first, run.last, run.spacing, run.measured, name, axis, result.points[(run.first + run.last) / 2]};
		}
	}
	if (result.alongside.has_value())
	{
		result.valid = true;
		auto const &alongside{*result.alongside};
		// moved onto the spacing
		for (auto k{alongside.first}; k <= alongside.last; ++k)
		{
			auto const moved{trace_station::alongside(alongside.axis, plan[k], alongside.spacing)};
			result.points[k].x = moved.x;
			result.points[k].z = moved.y;
		}
	}
	// a station track alongside a straight track, from a turnout or into one: designed, not traced. From the end of the turnout it starts
	// with by the arc joining it onto the straight alongside, along it, by the arc into the turnout it ends with; the radii of the arcs
	// fitted to the points along them, as on the photo; with no turnout at an end, from the first point alongside or to the last one
	if (result.alongside.has_value() && (result.start.has_value() || result.end.has_value()))
	{
		auto const &along{*result.alongside};
		auto const &axis{along.axis};
		auto straight_axis{axis.size() >= 2 && glm::distance(axis.front(), axis.back()) > 1.0};
		for (std::size_t i = 1; straight_axis && i + 1 < axis.size(); ++i)
			straight_axis = trace_station::off_line({axis.front(), axis.back()}, axis[i]) < 0.02;
		auto const starts{result.start.has_value() || (along.first == 0 && false == fromend)};
		auto const ends{result.end.has_value() || along.last + 1 == result.points.size()};
		if (straight_axis && starts && ends)
		{
			auto const direction{glm::normalize(axis.back() - axis.front())};
			trace_station::straight line{axis.front() + glm::dvec2{-direction.y, direction.x} * along.spacing, direction};
			if (glm::dot(plan[along.last] - plan[along.first], line.direction) < 0.0)
				line.direction = -line.direction;
			auto const on_line = [&](glm::dvec3 const &Point) {
				auto const at{line.point(line.along(plan_of(Point)))};
				return glm::dvec3{at.x, Point.y, at.y};
			};
			alignment::design design;
			design.norms = laying_norms();
			design.shape = m_route.design.shape;
			design.speed = std::max(1.0, design_speed(nullptr, nullptr));
			if (result.start.has_value() && result.end.has_value())
				design.speed = std::max(1.0, std::min(design.speed, std::max(result.start->speed, result.end->speed)));
			auto valid{true};
			std::vector<alignment::vertex> vertices;
			std::vector<double> tangents;
			if (result.start.has_value())
			{
				std::vector<glm::dvec2> before(plan.begin(), plan.begin() + static_cast<std::ptrdiff_t>(along.first));
				std::vector<double> noise(result.sigmas.begin(), result.sigmas.begin() + static_cast<std::ptrdiff_t>(along.first));
				auto const arc{trace_station::join_alongside(result.start->hold.position, result.start->hold.heading, line, before, noise, m_switch.templates[result.start->shape].radius)};
				valid = arc.valid;
				design.start = result.start->end;
				design.start_direction = result.start->hold.heading;
				alignment::vertex vertex;
				vertex.position = arc.vertex;
				vertex.radius = arc.radius;
				vertex.offset = glm::distance(plan_of(design.start), arc.vertex);
				vertices.push_back(vertex);
				tangents.push_back(line.along(arc.vertex) + arc.radius * std::tan(0.5 * std::acos(std::clamp(glm::dot(glm::normalize(result.start->hold.heading), line.direction), -1.0, 1.0))));
			}
			else
			{
				design.start = on_line(result.points[along.first]);
				design.start_direction = line.direction;
				tangents.push_back(line.along(plan_of(design.start)));
			}
			if (result.end.has_value())
			{
				std::vector<glm::dvec2> after(plan.rbegin(), plan.rbegin() + static_cast<std::ptrdiff_t>(plan.size() - 1 - along.last));
				std::vector<double> noise(result.sigmas.rbegin(), result.sigmas.rbegin() + static_cast<std::ptrdiff_t>(plan.size() - 1 - along.last));
				trace_station::straight const back{line.origin, -line.direction};
				auto const arc{trace_station::join_alongside(result.end->hold.position, -result.end->hold.heading, back, after, noise, m_switch.templates[result.end->shape].radius)};
				valid = valid && arc.valid;
				design.end = result.end->end;
				design.end_direction = result.end->hold.heading;
				alignment::vertex vertex;
				vertex.position = arc.vertex;
				vertex.radius = arc.radius;
				vertex.offset = glm::distance(arc.vertex, plan_of(design.end));
				vertices.push_back(vertex);
				tangents.push_back(line.along(arc.vertex) - arc.radius * std::tan(0.5 * std::acos(std::clamp(glm::dot(glm::normalize(result.end->hold.heading), line.direction), -1.0, 1.0))));
			}
			else
			{
				design.end = on_line(result.points[along.last]);
				design.end_direction = line.direction;
				tangents.push_back(line.along(plan_of(design.end)));
			}
			// the arcs one after the other along the straight alongside
			valid = valid && tangents.front() < tangents.back() - 1.0;
			if (valid)
			{
				design.vertices = vertices;
				if (auto const shape{alignment::compute(design)}; shape.valid)
					result.design = design;
			}
		}
	}
	return done();
}

void editor_mode::lay_station(traced_station const &Station, editor_track::snap_target const &End)
{
	auto &tool{m_lay};
	tool.status_time = ImGui::GetTime();
	// not laid: why, the points kept to be put right, the click which ended the line on a track or at a free end taken back
	auto const keep = [&](std::string const &Why) {
		tool.laid_now = false;
		tool.status = Why;
		if ((Station.end.has_value() || End.track != nullptr) && false == tool.points.empty())
		{
			tool.points.pop_back();
			lay_trim();
		}
		WriteLog("Editor: " + tool.status, logtype::generic);
	};
	if (false == Station.refusal.empty())
	{
		keep(Station.refusal);
		return;
	}
	if (Station.crossover)
	{
		auto *style{Station.start->track};
		auto const before{m_history.size()};
		if (false == lay_planned_connection(Station.request, Station.connection, style))
		{
			keep(m_switch.status);
			return;
		}
		std::vector<TTrack *> switches;
		for (auto step{before}; step < m_history.size(); ++step)
			for (auto *track : m_history[step].created)
				if (track != nullptr && track->eType == tt_Switch && false == track->m_editorremoved)
					switches.push_back(track);
		tool.status = m_switch.status + "\n" + format(STR_C("Crossover of %s, traced on the photo · Ctrl+Z undoes"), Station.start->label.c_str());
		tool.laid_now = true;
		tool.laid = {};
		tool.laid.station = Station;
		tool.laid.turnouts = switches;
		tool.laid.crossover = true;
		first_step_done(first_step::trace);
		tool.points.clear();
		tool.modes.clear();
		tool.sigmas.clear();
		tool.heights.clear();
		tool.start = {};
		tool.continuing = false;
		tool.laid_step = history_top();
		WriteLog("Editor: " + tool.status, logtype::generic);
		return;
	}
	// the line between the turnouts as shown: not laid at all where it can't be
	{
		double length;
		std::string error;
		lay_line_plan plan;
		auto const pieces{lay_pieces(tool.points, tool.modes, End, length, error, &plan)};
		if (pieces.empty() || (plan.traced.has_value() && plan.traced->refused))
		{
			keep(error.empty() ? std::string{STR_C("Click points along the track between the turnouts")} : error);
			return;
		}
	}
	auto const clicked{tool.points};
	auto const modes{tool.modes};
	auto const sigmas{tool.sigmas};
	auto const heights{tool.heights};
	auto const start{tool.start};
	auto const steps{m_history.size()};
	// the turnouts laid back: the tracks as they were, the points clicked kept
	auto const back = [&](std::string const &Why) {
		while (m_history.size() > steps)
			undo_last();
		straight_refresh();
		tool.points = clicked;
		tool.modes = modes;
		tool.sigmas = sigmas;
		tool.heights = heights;
		tool.start = start;
		keep(Why);
	};
	std::vector<TTrack *> switches;
	std::vector<std::string> told;
	auto const lay_turnout = [&](traced_turnout const &Turnout) -> TTrack * {
		// on the straight as it is now: the turnout laid before may have cut it
		auto const line{straight_through(Turnout.start, Turnout.line.direction)};
		if (line.tracks.empty())
		{
			m_switch.status = STR("The track the turnout goes into isn't a plain straight there any more");
			return nullptr;
		}
		auto const forward{glm::dot(line.direction, Turnout.line.direction * static_cast<double>(Turnout.fit.direction)) >= 0.0 ? 1 : -1};
		std::vector<std::pair<TTrack *, editor_track::state>> states;
		std::vector<TTrack *> created;
		std::vector<TTrack *> removed;
		auto const at{m_history.size()};
		if (false == place_switch_on_straight(line, m_switch.templates[Turnout.shape], nullptr, line.along(Turnout.start), forward, Turnout.fit.side, false, states, created, removed, switch_name_for_new()))
			return nullptr;
		auto const placed{created};
		push_track_snapshot(std::move(states), std::move(created), std::move(removed));
		if (at > steps && m_history.size() > at)
			m_history.back().joined = true;
		straight_refresh();
		auto const found{std::ranges::find_if(placed, [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; })};
		if (found == placed.end())
			return nullptr;
		if (place_switch_drive(**found))
			join_last_steps();
		advance_switch_name();
		told.push_back(format(STR_C("Turnout %s: %s"), (*found)->name().c_str(), Turnout.label.c_str()) + (Turnout.note.empty() ? std::string{} : " · " + Turnout.note));
		return *found;
	};
	// the end of the diverging track of the switch laid, the line going on from it
	auto const branch_end = [&](TTrack *Switch) {
		if (Switch == nullptr || Switch->m_paths.size() < 2)
			return editor_track::snap_target{};
		return editor_track::find_free_end(nullptr, kRailCategory, Switch->m_paths[1].points[segment_data::point::end], 0.05, {});
	};
	editor_track::snap_target from;
	editor_track::snap_target into{End};
	if (Station.start.has_value())
	{
		auto *laid{lay_turnout(*Station.start)};
		from = branch_end(laid);
		if (from.track == nullptr)
			return back(m_switch.status.empty() ? std::string{STR_C("The turnout couldn't be laid")} : m_switch.status);
		switches.push_back(laid);
	}
	if (Station.end.has_value())
	{
		auto *laid{lay_turnout(*Station.end)};
		into = branch_end(laid);
		if (into.track == nullptr)
			return back(m_switch.status.empty() ? std::string{STR_C("The turnout couldn't be laid")} : m_switch.status);
		switches.push_back(laid);
	}
	// the line from the end of the one to the end of the other, through the points between, as from a free end to a free end
	std::vector<glm::dvec3> points;
	std::vector<click_heights::source> sources;
	std::vector<double> noise;
	if (from.track != nullptr)
	{
		points.push_back(from.position);
		sources.push_back(click_heights::source::fixed);
		noise.push_back(0.05);
	}
	else if (start.track != nullptr && false == clicked.empty())
	{
		points.push_back(clicked.front());
		sources.push_back(heights.empty() ? click_heights::source::fixed : heights.front());
		noise.push_back(sigmas.empty() ? 0.05 : sigmas.front());
	}
	for (std::size_t k = 0; k < Station.points.size(); ++k)
	{
		auto const index{Station.indices[k]};
		points.push_back(Station.points[k]);
		sources.push_back(index < heights.size() ? heights[index] : click_heights::source::ground);
		noise.push_back(Station.sigmas[k]);
	}
	if (into.track != nullptr && Station.end.has_value())
	{
		points.push_back(into.position);
		sources.push_back(click_heights::source::fixed);
		noise.push_back(0.05);
	}
	tool.start = from.track != nullptr ? from : start;
	tool.points = points;
	tool.modes.assign(points.size(), lay_mode::traced);
	tool.sigmas = noise;
	tool.heights = sources;
	auto const line_step{m_history.size()};
	m_station_laying = true;
	m_station_design = Station.design;
	lay_finish(into);
	m_station_laying = false;
	m_station_design.reset();
	if (m_history.size() <= line_step)
		return back(tool.status.empty() ? std::string{STR_C("The track between the turnouts couldn't be laid")} : tool.status);
	if (line_step > steps)
		m_history[line_step].joined = true;
	for (auto const &each : told)
		tool.status += "\n" + each;
	if (Station.alongside.has_value())
		tool.status += "\n" + format(STR_C("Alongside %s · %.2f m (measured %.2f m)"), Station.alongside->neighbour.c_str(), std::abs(Station.alongside->spacing), std::abs(Station.alongside->measured));
	tool.laid.station = Station;
	tool.laid.turnouts = switches;
	tool.laid_step = history_top();
	if (false == switches.empty())
		first_step_done(first_step::turnout);
	WriteLog("Editor: " + tool.status, logtype::generic);
}
