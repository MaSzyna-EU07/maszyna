/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/


#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "world/TrackFeatures.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <set>
#include "application/editormodetrackshared.h"

using namespace track_mode;

bool editor_mode::route_active() const
{
	return ui()->mode() == nodebank_panel::TRACK && m_track_tab == track_tab::route && false == m_route.chain.tracks.empty();
}

void editor_mode::route_recommend(alignment::vertex &Vertex) const
{
	auto const recommended{alignment::recommend(m_route.design.speed, Vertex.radius, m_route.design.norms)};
	Vertex.cant = recommended.cant;
	Vertex.transition_in = recommended.transition;
	Vertex.transition_out = recommended.transition;
}

void editor_mode::route_bind_ends()
{
	auto const &chain{m_route.chain};
	auto &design{m_route.design};
	design.start = chain.start;
	design.end = chain.end;
	design.start_direction = glm::normalize(plan_of(chain.start_direction));
	design.end_direction = glm::normalize(plan_of(chain.end_direction));
	design.start_grade = grade_of(chain.start_direction);
	design.end_grade = grade_of(chain.end_direction);
	design.start_radius = chain.start_radius;
	design.end_radius = chain.end_radius;
	design.start_reserve = editor_track::run_reserve(editor_track::straight_beyond(chain, false, m_straights.tolerance));
	design.end_reserve = editor_track::run_reserve(editor_track::straight_beyond(chain, true, m_straights.tolerance));
}

void editor_mode::route_reset()
{
	auto &route{m_route};
	route.chain = {};
	route.error.clear();
	route.status.clear();
	route.vertex = -1;
	route.grip = -1;
	route.result = {};
	if (route.from == nullptr || route.to == nullptr)
		return;
	if (false == editor_track::find_chain(route.from, route.to, route.chain, route.error, true))
		return;
	if (false == route.chain.switches.empty())
		route.status = format(STR_C("The fragment runs through %zu switches along their main tracks. They move as a whole with the line, "
		                      "so they have to fall on its straights"),
		                      route.chain.switches.size());

	auto const &chain{route.chain};
	alignment::design design;
	design.norms = route.design.norms;
	design.shape = route.design.shape;
	design.transition_pieces = route.design.transition_pieces;
	design.speed = chain.velocity > 0.0 ? chain.velocity : 100.0;
	route.design = design;
	route_bind_ends();
	design = route.design;

	glm::dvec2 intersection;
	if (false == alignment::collinear(design))
	{
		alignment::vertex vertex;
		vertex.radius = chain.radius > 0.0 ? std::max(10.0, std::round(chain.radius / 10.0) * 10.0) : 1000.0;
		route_recommend(vertex);
		if (alignment::tangent_intersection(design, intersection))
		{
			route.design.vertices.push_back(vertex);
		}
		else
		{
			auto const distance{plan_distance(design.start, design.end)};
			vertex.offset = distance / 3.0;
			route.design.vertices.push_back(vertex);
			route.design.vertices.push_back(vertex);
		}
	}
	route_update();
}

bool editor_mode::route_from_curve(TTrack &Track)
{
	editor_track::curve curve;
	if (false == editor_track::find_curve(Track, m_straights.tolerance, m_route.design.norms.gauge, curve))
	{
		m_route.chain = {};
		m_route.result = {};
		m_route.vertex = -1;
		m_route.grip = -1;
		m_route.error = STR_C("No curve here: LMB on a curve");
		return false;
	}
	m_route.from = curve.from;
	m_route.to = curve.to;
	route_reset();
	if (m_route.chain.tracks.empty())
		return false;
	auto &design{m_route.design};
	auto vertex{route_vertex_of(curve)};
	m_route.designed = curve.designed.has_value() ? 1 : 0;
	if (curve.reversals == 0)
	{
		design.vertices = {vertex};
	}
	else
	{
		auto const distance{plan_distance(design.start, design.end)};
		vertex.offset = distance / 3.0;
		vertex.compound = false;
		vertex.reverse_turn = false;
		design.vertices = {vertex, vertex};
	}
	route_update();
	m_route.status = STR_C("Curve of ") + std::to_string(m_route.chain.tracks.size()) + STR_C(" paths, turning ") + std::to_string(static_cast<int>(std::round(glm::degrees(curve.turn)))) + STR_C(" deg");
	return true;
}

alignment::vertex editor_mode::route_vertex_of(editor_track::curve const &Curve) const
{
	if (Curve.designed.has_value())
		return *Curve.designed;
	alignment::vertex vertex;
	vertex.radius = Curve.radius > 0.0 ? std::max(1.0, Curve.radius) : 1000.0;
	vertex.transition_in = std::max(0.0, Curve.transition_in);
	vertex.transition_out = std::max(0.0, Curve.transition_out);
	vertex.cant = std::max(0.0, Curve.cant);
	vertex.reverse_turn = std::abs(Curve.turn) > glm::pi<double>();
	compound_from(Curve, vertex);
	return vertex;
}


bool editor_mode::route_load(TTrack &Track)
{
	m_route.designed = 0;
	if (false == (m_route.single ? route_from_curve(Track) : route_from_line(Track)))
		return false;
	route_read_stored(Track);
	return true;
}

// the design kept in the scenery for the fragment takes the place of the one read from the paths; its chainage is carried on
void editor_mode::route_read_stored(TTrack &Track)
{
	auto &route{m_route};
	route.departed = -1;
	route.departure = 0.0;
	route.origin = 0.0;
	if (route.chain.tracks.empty())
		return;
	auto const chain{route.chain};
	bool reversed;
	auto index{alignment_store::find(chain.start, chain.end, reversed)};
	route.departed = alignment_store::departed(chain.tracks, route.departure);
	auto const loose{std::any_of(chain.tracks.begin(), chain.tracks.end(), [](TTrack const *Track) { return false == alignment_store::on_track(*Track).has_value(); })};
	bool digitized{false};
	int parts{0};
	int skipped{0};
	std::string failure;
	if (index < 0 && route.departed < 0 && loose && false == route.single)
	{
		parts = digitize_parts(chain, &failure, skipped);
		if (parts == 1 && skipped == 0)
		{
			alignment_store::settle();
			index = alignment_store::find(chain.start, chain.end, reversed);
			digitized = true;
		}
		else if (parts > 0)
		{
			alignment_store::settle();
			route_from_line(Track);
			route.departed = alignment_store::departed(route.chain.tracks, route.departure);
		}
	}
	if (index >= 0 && false == reversed && false == route.single)
	{
		route_take_stored(index);
	}
	else if (auto const km{alignment_store::chainage_at(chain.start)})
	{
		auto const ahead{chain.start + glm::dvec3{route.design.start_direction.x, 0.0, route.design.start_direction.y}};
		auto const further{alignment_store::chainage_at(ahead)};
		if (further.has_value() && *further > *km)
			route.origin = *km;
	}
	if (digitized && route.designed == 0)
		route.status += STR_C("\nThe line is digitized now, a straight");
	else if (digitized)
		route.status += format(STR_C("\nThe line is digitized now, curves kept in the scenery: %d"), route.designed);
	else if (parts > 0)
		route.status += format(STR_C("\nThe line is digitized in parts: %d, paths left out: %d"), parts, skipped) + (failure.empty() ? std::string{} : " - " + failure);
	else if (skipped > 0 && loose && skipped < static_cast<int>(chain.tracks.size()))
		route.status += format(STR_C("\nPaths left out of the digitized line: %d"), skipped) + (failure.empty() ? std::string{} : " - " + failure);
	else if (false == failure.empty())
		route.status += STR_C("\nThe line can't be digitized: ") + failure;
	else if (route.designed > 0)
		route.status += format(STR_C("\nCurves read from the design kept in the scenery: %d"), route.designed);
	else if (false == route.design.vertices.empty())
		route.status += STR_C("\nCurves recognized from the paths, the scenery keeps no design for them");
}

void editor_mode::route_take_stored(int const Index)
{
	auto &route{m_route};
	auto const &records{alignment_store::records()};
	if (Index < 0 || Index >= static_cast<int>(records.size()))
		return;
	auto const &stored{records[Index]};
	auto &design{route.design};
	design.vertices = stored.design.vertices;
	design.shape = stored.design.shape;
	design.speed = stored.design.speed;
	design.norms = stored.design.norms;
	design.transition_pieces = stored.design.transition_pieces;
	design.arc_piece_angle = stored.design.arc_piece_angle;
	route.designed = static_cast<int>(design.vertices.size());
	route.origin = stored.origin;
	route.vertex = -1;
	route.grip = -1;
	route_update();
}

void editor_mode::render_route_stored()
{
	auto &route{m_route};
	ImGui::PushItemWidth(120.0f);
	auto km{route.origin / 1000.0};
	if (ImGui::InputDouble(STR_C("Km of the start"), &km, 0.0, 0.0, "%.3f"))
		route.origin = km * 1000.0;
	ImGui::PopItemWidth();
	item_tooltip("Chainage of the start of the fragment, kept with the design in the scenery; the vertical profile of these paths counts from it");
	if (ImGui::SmallButton(STR_C("Digitize all the tracks")))
	{
		digitize_all();
		return;
	}
	item_tooltip("Every line of the scenery not digitized yet is kept in it with its straights, arcs and transition curves, the way the drawn tracks are kept; the paths stay as they are");
	auto const &records{alignment_store::records()};
	if (route.departed < 0 || route.departed >= static_cast<int>(records.size()))
		return;
	auto const &layer{scene::Layers.layer(records[route.departed].layer).name};
	ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), STR_C("The design kept in %s for this fragment departs from its paths by up to %.2f m"), layer.c_str(), route.departure);
	if (ImGui::SmallButton(STR_C("Take the kept design")))
	{
		route_take_stored(route.departed);
		route.status = STR_C("The kept design is loaded, applying it lays the paths by it again");
	}
	item_tooltip("Loads the design kept in the scenery in place of the one read from the paths; nothing changes until it's applied");
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Forget the kept design")))
	{
		alignment_store::forget(route.departed);
		alignment_store::settle();
		route.departed = -1;
		route.status = STR_C("The kept design will be removed from ") + layer + STR_C(" on save");
	}
	item_tooltip("The paths stay as they are, the scenery no longer keeps a design for them");
}

void editor_mode::route_store()
{
	auto &route{m_route};
	if (false == route.result.valid || route.chain.tracks.empty() || scene::Layers.empty())
		return;
	alignment_store::store(route.design, route.result, route.origin, alignment_store::layer_of(*route.chain.tracks.front()));
	route.departed = -1;
	route.departure = 0.0;
}

bool editor_mode::digitize_drawn(std::vector<TTrack *> const &Created, bool const Alone)
{
	std::vector<TTrack *> drawn;
	for (auto *track : Created)
		if (track != nullptr && track->eType == tt_Normal && false == track->m_editorremoved && false == track->m_paths.empty())
			drawn.push_back(track);
	if (drawn.empty() || scene::Layers.empty())
		return false;
	editor_track::chain chain;
	std::string error;
	if (editor_track::find_run(*drawn.front(), kDrawnReach, chain, error) && digitize_chain(chain))
		return true;
	if (Alone && editor_track::find_chain(drawn.front(), drawn.back(), chain, error) && chain.tracks.size() == drawn.size() && digitize_chain(chain))
		return true;
	WriteLog("Editor: the drawn paths don't make a line of straights, arcs and transition curves, no design kept for them", logtype::generic);
	return false;
}

void editor_mode::digitize_all()
{
	std::set<TTrack const *> visited;
	int kept{0};
	int failed{0};
	std::string first;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty() || false == editor_track::is_supported(*track) || visited.count(track) > 0)
			continue;
		if (alignment_store::on_track(*track).has_value())
			continue;
		editor_track::chain chain;
		std::string error;
		if (false == editor_track::find_run(*track, kDrawnReach, chain, error) || chain.tracks.empty())
		{
			visited.insert(track);
			continue;
		}
		visited.insert(chain.tracks.begin(), chain.tracks.end());
		std::string reason;
		kept += digitize_parts(chain, &reason, failed);
		if (first.empty())
			first = reason;
	}
	alignment_store::settle();
	auto text{format(STR_C("Lines digitized: %d, paths left out: %d"), kept, failed)};
	if (false == first.empty())
		text += STR_C(", the first: ") + first;
	WriteLog("Editor: " + text, logtype::generic);
	ui()->set_status(text, failed > 0);
	if (false == m_route.chain.tracks.empty())
		route_load(*m_route.chain.tracks[m_route.chain.tracks.size() / 2]);
}

int editor_mode::digitize_parts(editor_track::chain const &Chain, std::string *Reason, int &Skipped)
{
	auto const count{Chain.tracks.size()};
	if (count == 0 || std::all_of(Chain.tracks.begin(), Chain.tracks.end(), [](TTrack const *Track) { return alignment_store::on_track(*Track).has_value(); }))
		return 0;
	TTrack *culprit{nullptr};
	std::string reason;
	if (digitize_chain(Chain, &reason, &culprit))
		return 1;
	if (count == 1)
	{
		++Skipped;
		reason = (Chain.tracks.front()->name().empty() ? std::string{"(noname)"} : Chain.tracks.front()->name()) + ": " + reason;
		WriteLog("Editor: digitizing - " + reason, logtype::generic);
		if (Reason != nullptr && Reason->empty())
			*Reason = reason;
		return 0;
	}
	auto const found{static_cast<std::size_t>(std::find(Chain.tracks.begin(), Chain.tracks.end(), culprit) - Chain.tracks.begin())};
	auto const split{found < count ? std::max<std::size_t>(1, found) : count / 2};
	int parts{0};
	for (auto const &[first, last] : {std::pair<std::size_t, std::size_t>{0, split - 1}, std::pair<std::size_t, std::size_t>{split, count - 1}})
	{
		editor_track::chain part;
		std::string error;
		if (editor_track::find_chain(Chain.tracks[first], Chain.tracks[last], part, error) && part.tracks.size() == last - first + 1)
			parts += digitize_parts(part, Reason, Skipped);
		else
			Skipped += static_cast<int>(last - first + 1);
	}
	return parts;
}

bool editor_mode::digitize_chain(editor_track::chain const &Chain, std::string *Reason, TTrack **Culprit)
{
	if (Chain.tracks.empty())
		return false;
	auto const name = [](TTrack const *Track) { return Track == nullptr || Track->name().empty() ? std::string{"(noname)"} : Track->name(); };
	auto const nearest = [&](glm::dvec2 const &Point) {
		TTrack *found{nullptr};
		double distance{std::numeric_limits<double>::max()};
		for (auto *track : Chain.tracks)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
				if (auto const d{glm::distance(plan_of(track->m_paths.front().points[end]), Point)}; d < distance)
				{
					distance = d;
					found = track;
				}
		return found;
	};
	alignment::design design;
	design.norms = m_route.design.norms;
	design.transition_pieces = m_route.design.transition_pieces;
	design.arc_piece_angle = m_route.design.arc_piece_angle;
	design.speed = Chain.velocity > 0.0 ? Chain.velocity : 100.0;
	design.start = Chain.start;
	design.end = Chain.end;
	if (glm::length(plan_of(Chain.start_direction)) < 1e-9 || glm::length(plan_of(Chain.end_direction)) < 1e-9)
		return false;
	design.start_direction = glm::normalize(plan_of(Chain.start_direction));
	design.end_direction = glm::normalize(plan_of(Chain.end_direction));
	design.start_grade = grade_of(Chain.start_direction);
	design.end_grade = grade_of(Chain.end_direction);
	design.start_radius = Chain.start_radius;
	design.end_radius = Chain.end_radius;
	auto const preferred{m_route.design.shape};
	auto const other{preferred == alignment::transition_shape::clothoid ? alignment::transition_shape::cubic_parabola : alignment::transition_shape::clothoid};
	alignment::result result;
	alignment::design best;
	alignment::result bestresult;
	double closest{std::numeric_limits<double>::max()};
	TTrack *farthest{nullptr};
	bool fits{false};
	for (auto const attempt : {0, 1, 2, 3})
	{
		auto const shape{attempt % 2 == 0 ? preferred : other};
		design.shape = shape;
		if (attempt == 2)
		{
			design.norms.arc_min = 0.0;
			design.norms.arc_min_factor = 0.0;
		}
		design.vertices.clear();
		for (auto const &entry : editor_track::recognize_line(Chain, m_straights.tolerance, design.norms.gauge, shape))
		{
			if (entry.kink)
			{
				if (Reason != nullptr)
					*Reason = format(STR_C("the straights meet with no curve at path %s"), name(nearest(entry.position)).c_str());
				if (Culprit != nullptr)
					*Culprit = nearest(entry.position);
				return false;
			}
			auto vertex{route_vertex_of(entry.shape)};
			vertex.position = entry.position;
			design.vertices.push_back(vertex);
		}
		if (design.vertices.size() >= 2)
		{
			design.vertices.front().offset = std::max(0.0, glm::dot(design.vertices.front().position - plan_of(design.start), design.start_direction));
			design.vertices.back().offset = std::max(0.0, glm::dot(plan_of(design.end) - design.vertices.back().position, design.end_direction));
		}
		result = alignment::compute(design);
		if (alignment_store::follows(result, Chain.tracks))
		{
			fits = true;
			break;
		}
		TTrack *worst{nullptr};
		if (auto const away{alignment_store::deviation(result, Chain.tracks, worst)}; away < closest)
		{
			closest = away;
			farthest = worst;
			best = design;
			bestresult = result;
		}
	}
	double fit{0.0};
	if (false == fits && closest <= kDigitizeFit)
	{
		design = best;
		result = bestresult;
		fit = closest;
		fits = true;
	}
	if (false == fits)
	{
		if (Culprit != nullptr)
			*Culprit = farthest;
		if (Reason != nullptr)
			*Reason = closest < std::numeric_limits<double>::max() ? format(STR_C("the curves read from the paths leave path %s by %.2f m"), name(farthest).c_str(), closest) : std::string{STR_C("the paths don't make a line of straights, arcs and transition curves")};
		return false;
	}
	auto const &records{alignment_store::records()};
	std::set<int> covered;
	for (auto *track : Chain.tracks)
		if (auto const found{alignment_store::on_track(*track)})
			covered.insert(found->record);
	double origin{0.0};
	for (auto const index : covered)
	{
		auto const &old{records[index]};
		auto const start{alignment_store::chainage_on(result, old.design.start)};
		auto const end{alignment_store::chainage_on(result, old.design.end)};
		if (false == start.has_value() || false == end.has_value())
			return false;
		if (index == *covered.begin() && *end > *start)
			origin = old.origin - *start;
	}
	auto const layer{covered.empty() ? alignment_store::layer_of(*Chain.tracks.front()) : records[*covered.begin()].layer};
	for (auto index{covered.rbegin()}; index != covered.rend(); ++index)
		alignment_store::forget(*index);
	alignment_store::store(design, result, origin, layer, "drawn", fit);
	return true;
}

bool editor_mode::route_from_line(TTrack &Track)
{
	auto &route{m_route};
	editor_track::chain run;
	if (false == editor_track::find_run(Track, kLineReach, run, route.error))
		return false;
	route.from = run.tracks.front();
	route.to = run.tracks.back();
	route_reset();
	if (route.chain.tracks.empty())
		return false;
	auto &design{route.design};
	auto const found{editor_track::recognize_line(route.chain, m_straights.tolerance, design.norms.gauge, design.shape)};
	route.designed = 0;
	if (false == found.empty())
	{
		design.vertices.clear();
		for (auto const &entry : found)
		{
			alignment::vertex vertex;
			if (entry.kink)
			{
				vertex.radius = std::max(10.0, std::round(alignment::recommend(design.speed, 1000.0, design.norms).radius_min / 10.0) * 10.0);
				vertex.transition_in = 0.0;
				vertex.transition_out = 0.0;
				vertex.cant = 0.0;
			}
			else
			{
				vertex = route_vertex_of(entry.shape);
				if (entry.shape.designed.has_value())
					++route.designed;
			}
			vertex.position = entry.position;
			design.vertices.push_back(vertex);
		}
		if (design.vertices.size() >= 2)
		{
			design.vertices.front().offset = std::max(0.0, glm::dot(design.vertices.front().position - plan_of(design.start), design.start_direction));
			design.vertices.back().offset = std::max(0.0, glm::dot(plan_of(design.end) - design.vertices.back().position, design.end_direction));
		}
		route_update();
	}
	route.status = format(STR_C("Line of %zu paths, %.0f m, %zu vertices"), route.chain.tracks.size(), route.chain.length, design.vertices.size());
	return true;
}

void editor_mode::route_apply()
{
	auto &route{m_route};
	route.status.clear();
	if (false == route.result.valid || route.chain.tracks.empty())
		return;
	editor_track::chain chain;
	if (false == editor_track::find_chain(route.from, route.to, chain, route.error, true))
		return;
	if (glm::distance(chain.start, route.chain.start) > 1e-3 || glm::distance(chain.end, route.chain.end) > 1e-3)
	{
		route.error = STR_C("The fragment changed since it was selected, select it again");
		return;
	}
	auto involved{chain.tracks};
	for (auto const &passage : chain.switches)
		involved.push_back(passage.track);
	if (false == tracks_editable(involved, route.error))
		return;

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	// takes back what was done before the paths were re-laid
	auto const undo = [&]() {
		for (auto *track : created)
			editor_track::retire(*track);
		for (auto entry = states.rbegin(); entry != states.rend(); ++entry)
		{
			editor_track::apply(*entry->first, entry->second);
			editor_track::commit({entry->first});
		}
	};
	auto *from{route.from};
	auto *to{route.to};
	auto const startextension{route.result.start_extension};
	auto const endextension{route.result.end_extension};
	if (startextension > 0.0 || endextension > 0.0)
	{
		auto const before{chain};
		if (startextension > 0.0)
			from = editor_track::take_straight(editor_track::straight_beyond(before, false, m_straights.tolerance), from, before.start, route.result.start, states, created);
		if (endextension > 0.0)
			to = editor_track::take_straight(editor_track::straight_beyond(before, true, m_straights.tolerance), to, before.end, route.result.end, states, created);
		std::string error;
		auto const found{editor_track::find_chain(from, to, chain, error, true)};
		auto const startoff{found ? plan_distance(chain.start, route.result.start) : 0.0};
		auto const endoff{found ? plan_distance(chain.end, route.result.end) : 0.0};
		if (false == found || startoff > 5.0 || endoff > 5.0)
		{
			auto const startrun{editor_track::straight_beyond(before, false, m_straights.tolerance)};
			auto const endrun{editor_track::straight_beyond(before, true, m_straights.tolerance)};
			WriteLog(format("editor: cutting the straights failed: found %d, start %.2f m off (extension %.2f, run %zu paths %.2f m), end %.2f m off (extension %.2f, run %zu paths %.2f m), chain %zu paths, from %s, to %s, error '%s'", found ? 1 : 0, startoff,
			                startextension, startrun.tracks.size(), startrun.length, endoff, endextension, endrun.tracks.size(), endrun.length, chain.tracks.size(), from != nullptr ? from->name().c_str() : "-", to != nullptr ? to->name().c_str() : "-", error.c_str()));
			undo();
			route.error = error.empty() ? format(STR_C("The adjoining straights can't be cut at the ends of the curve: the start misses by %.1f m, the end by %.1f m"), startoff, endoff) : error;
			return;
		}
		if (false == tracks_editable(chain.tracks, route.error))
		{
			undo();
			return;
		}
	}
	for (auto *track : chain.tracks)
		if (std::none_of(states.begin(), states.end(), [&](auto const &Entry) { return Entry.first == track; }))
			states.emplace_back(track, editor_track::capture(*track));
	std::vector<TTrack *> relaid;
	if (chain.switches.empty())
	{
		auto pieces{alignment::pieces(route.result, route.design, chain.tracks.size())};
		editor_track::keep_heights(chain, pieces);
		relaid = editor_track::relay(chain, pieces, &to);
	}
	else if (false == route_relay_through_switches(chain, states, relaid, to))
	{
		undo();
		return;
	}
	// the straights were cut along their own direction, the curve ends where the design says: the ends are brought together
	auto const added{relaid.size()};
	created.insert(created.end(), relaid.begin(), relaid.end());
	push_track_snapshot(std::move(states), std::move(created));

	route.from = from;
	route.to = to;
	editor_track::find_chain(route.from, route.to, route.chain, route.error, true);
	if (startextension > 0.0 || endextension > 0.0)
	{
		route_bind_ends();
		auto &vertices{route.design.vertices};
		if (vertices.size() >= 2)
		{
			vertices.front().offset += startextension;
			vertices.back().offset += endextension;
		}
		route_update();
	}
	route_store();
	records_to_history();
	route.status = STR_C("Re-laid ") + std::to_string(chain.tracks.size()) + STR_C(" paths") + (added > 0 ? STR_C(", added ") + std::to_string(added) : std::string{});
	if (startextension > 0.0 || endextension > 0.0)
		route.status += format(", the curve took over %.2f / %.2f m of the adjoining straights", startextension, endextension);
	WriteLog("Editor: route design - " + route.status, logtype::generic);
}

// the switches of the chain are moved as a whole onto the designed line, the regular paths between them re-laid
bool editor_mode::route_relay_through_switches(editor_track::chain const &Chain, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, TTrack *&Last)
{
	auto &route{m_route};
	auto const &result{route.result};
	auto const name = [](TTrack const *Track) { return Track->name().empty() ? std::string{"(noname)"} : Track->name(); };
	auto const plan = [](glm::dvec3 const &Vector) { return glm::normalize(plan_of(Vector)); };
	struct placement
	{
		TTrack *track;
		double from, length; // chainage along the new line
		glm::dvec3 entry; // of the main track, where the chain comes in
		double angle;
	};
	// chainages along the old line, the switches keep their distance from the nearer end of the chain
	double before{0.0};
	std::vector<double> starts;
	std::size_t next{0};
	for (std::size_t i = 0; i <= Chain.tracks.size(); ++i)
	{
		for (; next < Chain.switches.size() && Chain.switches[next].before == i; ++next)
		{
			starts.push_back(before);
			before += bezier{Chain.switches[next].track->m_paths.front()}.plan_length();
		}
		if (i < Chain.tracks.size())
			before += bezier{Chain.tracks[i]->m_paths.front()}.plan_length();
	}
	auto const oldlength{before};
	std::vector<placement> placements;
	std::vector<double> breaks;
	for (std::size_t k = 0; k < Chain.switches.size(); ++k)
	{
		auto const &passage{Chain.switches[k]};
		auto const &main{passage.track->m_paths.front()};
		auto const length{bezier{main}.plan_length()};
		auto const from{starts[k] + length * 0.5 < oldlength * 0.5 ? starts[k] : result.length - (oldlength - starts[k])};
		if (from < 0.0 || from + length > result.length || (false == breaks.empty() && from < breaks.back() - 0.01))
		{
			route.error = STR_C("The switch ") + name(passage.track) + STR_C(" doesn't fit on the designed line");
			return false;
		}
		auto const entry{main.points[passage.forward ? segment_data::point::start : segment_data::point::end]};
		auto const exit{main.points[passage.forward ? segment_data::point::end : segment_data::point::start]};
		bezier const curve{main};
		auto const direction{passage.forward ? plan(curve.first(0.0)) : -plan(curve.first(1.0))};
		auto const first{alignment::evaluate(result, from)};
		auto const last{alignment::evaluate(result, from + length)};
		auto const angle{signed_angle(direction, plan(first.direction))};
		// where the other end of the main track goes, set against the line
		auto const moved{plan_of(first.position) + turned(plan_of(exit - entry), angle)};
		auto const departure{glm::distance(moved, plan_of(last.position))};
		if (departure > 0.02)
		{
			route.error = format(STR_C("The switch %s would leave the designed line by %.2f m at its other end: it lies in a curve of the design. "
			                     "Move the vertices so that the switch falls on a straight"),
			                     name(passage.track).c_str(), departure);
			return false;
		}
		placements.push_back({passage.track, from, length, entry, angle});
		breaks.push_back(from);
		breaks.push_back(from + length);
	}
	// the regular parts between the switches, one interval each, the switches taking the ones between
	auto const parts{editor_track::chain_parts(Chain)};
	std::vector<std::size_t> counts;
	std::vector<int> partof;
	std::size_t part{0};
	for (std::size_t k = 0; k <= Chain.switches.size(); ++k)
	{
		auto const first{k == 0 ? std::size_t{0} : Chain.switches[k - 1].before};
		auto const last{k == Chain.switches.size() ? Chain.tracks.size() : Chain.switches[k].before};
		counts.push_back(last - first);
		partof.push_back(last > first ? static_cast<int>(part++) : -1);
		if (k < Chain.switches.size())
			counts.push_back(1);
	}
	if (part != parts.size())
	{
		route.error = STR_C("The paths between the switches can't be read as a chain");
		return false;
	}
	std::vector<std::size_t> intervals;
	auto const pieces{alignment::pieces(result, route.design, 0, breaks, counts, &intervals)};
	std::vector<std::vector<segment_data>> groups(parts.size());
	for (std::size_t i = 0; i < pieces.size(); ++i)
		if (intervals[i] % 2 == 0 && partof[intervals[i] / 2] >= 0)
			groups[partof[intervals[i] / 2]].push_back(pieces[i]);
	for (std::size_t i = 0; i < parts.size(); ++i)
		if (groups[i].size() < parts[i].tracks.size())
		{
			route.error = STR_C("The designed line between the switches is too short for the paths there");
			return false;
		}

	// all checked, the changes go in
	auto const remember = [&](TTrack *Track) {
		if (std::none_of(States.begin(), States.end(), [&](auto const &Entry) { return Entry.first == Track; }))
			States.emplace_back(Track, editor_track::capture(*Track));
	};
	for (auto const &place : placements)
	{
		auto joints{editor_track::joints(*place.track)};
		joints.erase(std::remove_if(joints.begin(), joints.end(), [&](editor_track::joint const &Joint) { return std::find(Chain.tracks.begin(), Chain.tracks.end(), Joint.other) != Chain.tracks.end(); }), joints.end());
		remember(place.track);
		std::vector<TTrack *> changed{place.track};
		for (auto const &joint : joints)
		{
			remember(joint.other);
			if (std::find(changed.begin(), changed.end(), joint.other) == changed.end())
				changed.push_back(joint.other);
		}
		// the heights stay, the paths between the switches keep theirs too
		auto target{alignment::evaluate(result, place.from).position};
		target.y = place.entry.y;
		editor_track::place(*place.track, place.entry, target, place.angle);
		editor_track::follow(*place.track, joints);
		editor_track::commit(changed);
	}
	for (std::size_t i = 0; i < parts.size(); ++i)
	{
		auto group{groups[i]};
		editor_track::keep_heights(parts[i], group);
		TTrack *last{nullptr};
		auto const added{editor_track::relay(parts[i], group, &last)};
		if (last != nullptr && parts[i].tracks.back() == Chain.tracks.back())
			Last = last;
		Created.insert(Created.end(), added.begin(), added.end());
	}
	return true;
}

glm::dvec3 editor_mode::route_vertex_position(int const Vertex) const
{
	auto const &design{m_route.design};
	auto const &result{m_route.result};
	auto const count{static_cast<int>(design.vertices.size())};
	glm::dvec2 const start{design.start.x, design.start.z};
	glm::dvec2 const end{design.end.x, design.end.z};
	glm::dvec2 position;
	if (Vertex < static_cast<int>(result.vertices.size()) && false == result.vertices.empty())
		position = result.vertices[Vertex];
	else if (count == 1)
	{
		if (false == alignment::tangent_intersection(design, position))
			position = (start + end) * 0.5;
	}
	else if (Vertex == 0)
		position = start + glm::normalize(design.start_direction) * design.vertices.front().offset;
	else if (Vertex == count - 1)
		position = end - glm::normalize(design.end_direction) * design.vertices.back().offset;
	else
		position = design.vertices[Vertex].position;

	auto const height{Vertex < static_cast<int>(result.vertex_elevations.size()) ? result.vertex_elevations[Vertex] : design.start.y + (design.end.y - design.start.y) * (Vertex + 1) / (count + 1.0)};
	return {position.x, height, position.y};
}

bool editor_mode::route_hit(int &Vertex, int &Grip) const
{
	if (false == route_active())
		return false;
	screen_projection const projection;
	float best = kHandleRadius * kHandleRadius;
	int hit{-1};
	for (int i = 0; i < static_cast<int>(m_route.design.vertices.size()); ++i)
	{
		if (auto const distance{projection.mouse_distance2(route_vertex_position(i))}; distance < best)
		{
			best = distance;
			hit = i;
		}
	}
	int griphit{-1};
	for (int i = 0; i < static_cast<int>(m_route.result.vertex_chainages.size()) && m_route.result.length > 0.0; ++i)
	{
		if (auto const distance{projection.mouse_distance2(alignment::evaluate(m_route.result, m_route.result.vertex_chainages[i]).position)}; distance < best)
		{
			best = distance;
			griphit = i;
			hit = -1;
		}
	}
	Vertex = hit;
	Grip = griphit;
	return hit >= 0 || griphit >= 0;
}

void editor_mode::draw_route_overlay() const
{
	auto const &route{m_route};
	auto const &design{route.design};
	auto const &result{route.result};
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
	auto const &start{result.valid ? result.start : design.start};
	auto const &end{result.valid ? result.end : design.end};

	std::vector<glm::dvec3> polygon{start};
	for (int i = 0; i < static_cast<int>(design.vertices.size()); ++i)
		polygon.push_back(route_vertex_position(i));
	polygon.push_back(end);
	for (std::size_t i = 0; i + 1 < polygon.size(); ++i)
		projection.line(drawlist, polygon[i], polygon[i + 1], IM_COL32(255, 210, 60, 150), 1.5f);

	if (result.length > 0.0 && result.profile.size() >= 2)
	{
		for (auto const &e : result.elements)
		{
			ImU32 const color = e.kind == alignment::element_kind::straight ? IM_COL32(255, 255, 255, 230) : e.kind == alignment::element_kind::arc ? overlay_color::grip : IM_COL32(255, 140, 30, 255);
			int const steps = std::clamp(static_cast<int>(e.length / 2.0), 1, 400);
			auto previous{alignment::evaluate(result, e.chainage).position};
			for (int j = 1; j <= steps; ++j)
			{
				auto const next{alignment::evaluate(result, e.chainage + e.length * j / steps).position};
				projection.line(drawlist, previous, next, color, 3.0f);
				previous = next;
			}
			ImVec2 screen;
			if (projection.project(alignment::evaluate(result, e.chainage).position, screen))
				drawlist->AddCircleFilled(screen, 3.5f, overlay_color::highlight);
		}
	}

	for (auto const atend : {false, true})
	{
		auto const &point{atend ? end : start};
		auto const direction{atend ? design.end_direction : design.start_direction};
		ImVec2 screen;
		if (projection.project(point, screen))
			drawlist->AddRectFilled(ImVec2(screen.x - 5.0f, screen.y - 5.0f), ImVec2(screen.x + 5.0f, screen.y + 5.0f), IM_COL32(70, 140, 255, 255));
		auto const arrow{glm::dvec3{direction.x, 0.0, direction.y} * (atend ? 15.0 : -15.0)};
		projection.line(drawlist, point, point + arrow, IM_COL32(70, 140, 255, 255), 2.0f);
	}
	for (int i = 0; i < static_cast<int>(design.vertices.size()); ++i)
	{
		ImVec2 screen;
		if (false == projection.project(route_vertex_position(i), screen))
			continue;
		drawlist->AddCircleFilled(screen, 6.0f, overlay_color::marked);
		if (i == route.vertex)
			drawlist->AddCircle(screen, 11.0f, overlay_color::highlight, 16, 2.5f);
		if (i < static_cast<int>(result.vertex_chainages.size()) && result.length > 0.0 && projection.project(alignment::evaluate(result, result.vertex_chainages[i]).position, screen))
		{
			drawlist->AddCircleFilled(screen, 6.0f, overlay_color::grip, 4);
			if (i == route.grip)
				drawlist->AddCircle(screen, 11.0f, overlay_color::highlight, 16, 2.5f);
			auto const *report{i < static_cast<int>(result.curves.size()) ? &result.curves[i] : nullptr};
			auto radius{format("R %.0f", report != nullptr ? report->radius : design.vertices[i].radius)};
			if (report != nullptr && report->radii.size() >= 2)
			{
				radius = "R";
				for (std::size_t k = 0; k < report->radii.size(); ++k)
					radius += format(k == 0 ? " %.0f" : " / %.0f", report->radii[k]);
			}
			drawlist->AddText(ImVec2(screen.x + 9.0f, screen.y + 4.0f), overlay_color::grip, radius.c_str());
		}
		drawlist->AddText(ImVec2(screen.x + 9.0f, screen.y - 18.0f), overlay_color::marked, format("W%d", i + 1).c_str());
	}
}

void editor_mode::render_route_gizmo()
{
	auto &route{m_route};
	auto const count{static_cast<int>(route.design.vertices.size())};
	bool const grip{route.grip >= 0 && route.grip < count && route.grip < static_cast<int>(route.result.vertex_chainages.size())};
	if (false == grip && (route.vertex < 0 || route.vertex >= count))
	{
		m_route_dragging = false;
		return;
	}

	auto const position{grip ? alignment::evaluate(route.result, route.result.vertex_chainages[route.grip]).position : route_vertex_position(route.vertex)};
	glm::dvec3 moved{position};
	if (false == handle_drag_step(moved))
	{
		if (m_route_dragging)
		{
			m_route_dragging = false;
			route_apply();
		}
		return;
	}
	m_route_dragging = true;
	if (grip)
	{
		auto &vertex{route.design.vertices[route.grip]};
		auto const corner{route_vertex_position(route.grip)};
		auto const distance{plan_distance(moved, corner)};
		for (auto const &curve : route.result.curves)
		{
			if (curve.vertex != route.grip || vertex.reverse_turn)
				continue;
			auto const secant{1.0 / std::cos(curve.deflection * 0.5) - 1.0};
			if (secant > 1e-6)
				vertex.radius = std::max(10.0, std::round(distance / secant));
		}
		route_update();
		return;
	}
	auto &vertex{route.design.vertices[route.vertex]};
	glm::dvec2 const point{moved.x, moved.z};
	if (count >= 2)
	{
		if (route.vertex == 0)
			vertex.offset = std::max(0.1, glm::dot(point - plan_of(route.design.start), glm::normalize(route.design.start_direction)));
		else if (route.vertex == count - 1)
			vertex.offset = std::max(0.1, glm::dot(plan_of(route.design.end) - point, glm::normalize(route.design.end_direction)));
		else
			vertex.position = point;
	}
	route_update();
}

void editor_mode::render_route_ui()
{
	auto &route{m_route};
	render_norms_choice();
	ImGui::Separator();
	if (false == route.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", route.error.c_str());
	if (route.chain.tracks.empty())
		return;

	ImGui::TextDisabled(STR_C("%zu paths, %.2f m"), route.chain.tracks.size(), route.chain.length);
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Reset design")))
	{
		auto *track{route.chain.tracks[route.chain.tracks.size() / 2]};
		route_load(*track);
		return;
	}
	item_tooltip("Reads the curves of the line from the scenery again, dropping the changes not applied");
	render_route_stored();
	auto changed{render_route_vertices()};
	if (ImGui::CollapsingHeader(STR_C("Design speed and limits")))
		changed |= render_route_parameters();
	if (changed)
		route_update();

	ImGui::Separator();
	render_route_result();
}

bool editor_mode::render_route_parameters()
{
	auto &design{m_route.design};
	bool changed{false};
	ImGui::PushItemWidth(120.0f);
	changed |= ImGui::InputDouble(STR_C("Design speed (km/h)"), &design.speed, 0.0, 0.0, "%.0f");
	item_tooltip("Speed the cant, the transitions and the recommended radius are designed for");
	design.speed = std::max(1.0, design.speed);
	int shape = static_cast<int>(design.shape);
	char const *shapes[] = {STR_C("Cubic parabola"), STR_C("Clothoid")};
	if (ImGui::Combo(STR_C("Transition curve"), &shape, shapes, IM_ARRAYSIZE(shapes)))
	{
		design.shape = static_cast<alignment::transition_shape>(shape);
		changed = true;
	}
	item_tooltip("Cubic parabola: the usual one on the Polish lines; clothoid: curvature growing linearly with the length");
	ImGui::PopItemWidth();

	if (ImGui::TreeNode(STR_C("Limits")))
	{
		auto &norms{design.norms};
		ImGui::PushItemWidth(100.0f);
		changed |= ImGui::InputDouble(STR_C("Rail axes distance (mm)"), &norms.gauge, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble(STR_C("Unbalanced acceleration (m/s2)"), &norms.unbalanced, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble(STR_C("Maximum cant (mm)"), &norms.cant_max, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble(STR_C("Cant change rate (mm/s)"), &norms.cant_rate, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble(STR_C("Cant ramp 1:(k*V), k"), &norms.ramp_factor, 0.0, 0.0, "%.1f");
		changed |= ImGui::InputDouble(STR_C("Unbalanced acc. change rate (m/s3)"), &norms.jerk, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble(STR_C("Straight between curves (s of travel)"), &norms.tangent_min_time, 0.0, 0.0, "%.1f");
		ImGui::PopItemWidth();
		norms.gauge = std::max(100.0, norms.gauge);
		norms.cant_rate = std::max(1.0, norms.cant_rate);
		norms.jerk = std::max(0.01, norms.jerk);
		ImGui::TreePop();
	}
	return changed;
}

bool editor_mode::render_route_vertices()
{
	bool changed{false};
	auto const count{static_cast<int>(m_route.design.vertices.size())};
	for (int i = 0; i < count; ++i)
	{
		ImGui::PushID(i);
		changed |= render_route_vertex(i);
		ImGui::PopID();
	}
	return changed;
}

bool editor_mode::render_route_vertex(int const Index)
{
	auto &route{m_route};
	auto &design{route.design};
	auto &vertex{design.vertices[Index]};
	auto const count{static_cast<int>(design.vertices.size())};
	bool changed{false};
	if (count > 1)
		ImGui::TextDisabled(STR_C("Curve %d of the S-curve"), Index + 1);
	ImGui::PushItemWidth(110.0f);
	changed |= ImGui::DragScalar(vertex.compound ? STR_C("Radius R1 (m)") : STR_C("Radius R (m)"), ImGuiDataType_Double, &vertex.radius, 5.0f, nullptr, nullptr, "%.0f");
	if (ImGui::Checkbox(STR_C("Compound curve (several radii)"), &vertex.compound))
	{
		changed = true;
		if (vertex.compound && vertex.arcs.empty())
			vertex.arcs.push_back({std::max(10.0, std::round(vertex.radius * 0.6 / 10.0) * 10.0), 0.0, 1.0});
	}
	item_tooltip("Curve made of arcs of different radii, each one its own path, joined with transitions");
	if (vertex.compound)
	{
		auto total{std::max(0.0, vertex.share)};
		for (auto const &arc : vertex.arcs)
			total += std::max(0.0, arc.share);
		auto const percent = [&](double const Share) { return total > 0.0 ? 100.0 * std::max(0.0, Share) / total : 0.0; };
		changed |= ImGui::DragScalar(format(STR_C("Share of R%d"), 1).c_str(), ImGuiDataType_Double, &vertex.share, 0.01f, nullptr, nullptr, "%.2f");
		ImGui::SameLine();
		ImGui::TextDisabled("%.0f%%", percent(vertex.share));
		int removed{-1};
		for (int k = 0; k < static_cast<int>(vertex.arcs.size()); ++k)
		{
			auto &arc{vertex.arcs[k]};
			ImGui::PushID(k);
			ImGui::Separator();
			changed |= ImGui::DragScalar(format(STR_C("Radius R%d (m)"), k + 2).c_str(), ImGuiDataType_Double, &arc.radius, 5.0f, nullptr, nullptr, "%.0f");
			ImGui::SameLine();
			if (ImGui::SmallButton("x"))
				removed = k;
			item_tooltip("Takes this radius out of the curve");
			changed |= ImGui::DragScalar(format(STR_C("Transition R%d-R%d (m)"), k + 1, k + 2).c_str(), ImGuiDataType_Double, &arc.transition, 1.0f, nullptr, nullptr, "%.0f");
			changed |= ImGui::DragScalar(format(STR_C("Share of R%d"), k + 2).c_str(), ImGuiDataType_Double, &arc.share, 0.01f, nullptr, nullptr, "%.2f");
			ImGui::SameLine();
			ImGui::TextDisabled("%.0f%%", percent(arc.share));
			arc.radius = std::max(1.0, arc.radius);
			arc.transition = std::max(0.0, arc.transition);
			arc.share = std::max(0.0, arc.share);
			ImGui::PopID();
		}
		vertex.share = std::max(0.0, vertex.share);
		if (removed >= 0)
		{
			vertex.arcs.erase(vertex.arcs.begin() + removed);
			if (vertex.arcs.empty())
				vertex.compound = false;
			changed = true;
		}
		if (ImGui::SmallButton(STR_C("Add a radius")))
		{
			auto const last{vertex.arcs.empty() ? vertex.radius : vertex.arcs.back().radius};
			vertex.arcs.push_back({std::max(10.0, std::round(last * 0.8 / 10.0) * 10.0), 0.0, 1.0});
			changed = true;
		}
		item_tooltip("Another arc after the last one, the angle of the curve is shared out by the shares");
		ImGui::Separator();
	}
	changed |= ImGui::DragScalar(STR_C("Transition in (m)"), ImGuiDataType_Double, &vertex.transition_in, 1.0f, nullptr, nullptr, "%.0f");
	changed |= ImGui::DragScalar(STR_C("Transition out (m)"), ImGuiDataType_Double, &vertex.transition_out, 1.0f, nullptr, nullptr, "%.0f");
	changed |= ImGui::DragScalar(STR_C("Cant (mm)"), ImGuiDataType_Double, &vertex.cant, 1.0f, nullptr, nullptr, "%.0f");
	item_tooltip("Height of the outer rail above the inner one");
	ImGui::PopItemWidth();
	vertex.radius = std::max(1.0, vertex.radius);
	vertex.transition_in = std::max(0.0, vertex.transition_in);
	vertex.transition_out = std::max(0.0, vertex.transition_out);
	vertex.cant = std::max(0.0, vertex.cant);
	if (false == m_norms_free)
	{
		if (ImGui::SmallButton(STR_C("Apply recommended for V")))
		{
			route_recommend(vertex);
			changed = true;
		}
		item_tooltip("Cant and transitions recommended for the design speed and the radius");
		auto const recommended{alignment::recommend(design.speed, vertex.radius, design.norms)};
		ImGui::TextDisabled(STR_C("For %.0f km/h: R min %.0f m, cant %.0f mm (equilibrium %.0f, min %.0f), transition %.0f m"), design.speed, recommended.radius_min, recommended.cant, recommended.cant_equilibrium, recommended.cant_min, recommended.transition);
	}
	if (ImGui::TreeNode(STR_C("Details")))
	{
		for (auto const &curve : route.result.curves)
		{
			if (curve.vertex != Index)
				continue;
			ImGui::Text(STR_C("Deflection %.4f deg, tangents %.2f / %.2f m, arc %.2f m"), glm::degrees(curve.deflection), curve.tangent_in, curve.tangent_out, curve.arc_length);
			ImGui::Text(STR_C("Unbalanced acc. %.3f m/s2, cant rate %.1f / %.1f mm/s, acc. rate %.3f / %.3f m/s3"), curve.unbalanced, curve.cant_rate_in, curve.cant_rate_out, curve.jerk_in, curve.jerk_out);
		}
		changed |= ImGui::Checkbox(STR_C("Turn the longer way (over 180 deg)"), &vertex.reverse_turn);
		item_tooltip("For the loops: the curve turns by more than a half circle");
		ImGui::TreePop();
	}
	return changed;
}

void editor_mode::render_route_result()
{
	auto const &route{m_route};
	auto const &design{route.design};
	auto const &result{route.result};
	if (result.length > 0.0)
	{
		auto const pieces{alignment::minimum_pieces(result, design)};
		ImGui::Text(STR_C("Length %.2f m (now %.2f m), %zu elements"), result.length, route.chain.length, result.elements.size());
		ImGui::TextDisabled(STR_C("Paths: %zu in the fragment, %zu needed%s"), route.chain.tracks.size(), pieces, pieces > route.chain.tracks.size() ? STR_C(" (copies of the neighbours will be added)") : "");
		if (result.start_extension > 0.0 || result.end_extension > 0.0)
			ImGui::Text(STR_C("Takes over %.2f m of the straight at the start, %.2f m at the end"), result.start_extension, result.end_extension);
	}
	for (auto const &error : result.errors)
		ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", error.c_str());
	for (auto const &warning : result.warnings)
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", warning.c_str());

	if (ImGui::TreeNode(STR_C("Elements")))
	{
		ImGui::TextDisabled(STR_C("Straight available beyond the ends: %.2f / %.2f m"), design.start_reserve, design.end_reserve);
		char const *kinds[] = {"straight", "transition", "curve", "transition", "spiral"};
		for (auto const &e : result.elements)
			ImGui::Text("km %s  %-10s  L %8.2f m%s", profile::format_chainage(m_route.origin + e.chainage).c_str(), kinds[static_cast<int>(e.kind)], e.length, e.kind == alignment::element_kind::straight ? "" : ("  R " + std::to_string(static_cast<int>(std::round(e.radius))) + " m").c_str());
		ImGui::TreePop();
	}

	if (result.valid)
	{
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.55f, 0.25f, 1.0f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.65f, 0.3f, 1.0f));
		if (ImGui::Button(STR_C("Apply to the scenery"), ImVec2(-1.0f, 28.0f)))
			route_apply();
		ImGui::PopStyleColor(2);
	}
	else
	{
		ImGui::TextDisabled(STR_C("Correct the errors to apply the design"));
	}
	if (false == route.status.empty())
		ImGui::TextUnformatted(route.status.c_str());
}

editor_track::straight const &editor_mode::current_straight()
{
	auto *track{selected_track()};
	if (track != m_straights.current_for)
	{
		m_straights.current_for = track;
		m_straights.handle = -1;
		m_straights.current = track != nullptr ? editor_track::find_straight(*track, m_straights.tolerance) : editor_track::straight{};
		find_neighbour_straights();
	}
	return m_straights.current;
}
