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
#include "application/editorgestures.h"
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
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <set>
#include "application/editormodetrackshared.h"

using namespace track_mode;

namespace
{

// how far the points stray from the curve traced through them, as a colour: green within 10 cm, amber within 30 cm, red beyond
ImU32 departure_colour(double const Departure)
{
	if (Departure <= 0.1)
		return IM_COL32(90, 220, 110, 255);
	if (Departure <= 0.3)
		return IM_COL32(255, 200, 60, 255);
	return IM_COL32(255, 90, 70, 255);
}

// the colour an element of the design is drawn in: white straights, green arcs, orange transitions
ImU32 element_colour(alignment::element_kind const Kind)
{
	if (Kind == alignment::element_kind::straight)
		return IM_COL32(255, 255, 255, 230);
	if (Kind == alignment::element_kind::arc)
		return overlay_color::grip;
	return IM_COL32(255, 140, 30, 255);
}

// the way the first curve of the design, or the last one, turns by the legs of the polygon: 1 left, -1 right
int design_turn(alignment::design const &Design, bool const Atend)
{
	auto const count{Design.vertices.size()};
	if (count == 1)
		return cross(Design.start_direction, Design.end_direction) > 0.0 ? 1 : -1;
	glm::dvec2 const in{Atend ? Design.vertices[count - 1].position - Design.vertices[count - 2].position : Design.start_direction};
	glm::dvec2 const out{Atend ? Design.end_direction : Design.vertices[1].position - Design.vertices[0].position};
	return cross(in, out) > 0.0 ? 1 : -1;
}

// how far the paths stray from the line laid along them at most, the most there is when it can't be laid
double design_departure(alignment::result const &Result, std::vector<TTrack *> const &Tracks)
{
	TTrack *worst{nullptr};
	return Result.valid ? alignment_store::deviation(Result, Tracks, worst) : std::numeric_limits<double>::max();
}

// the first vertex of the design and the last one as far into the line from its ends as they lie
void design_end_offsets(alignment::design &Design)
{
	if (Design.vertices.size() < 2)
		return;
	Design.vertices.front().offset = std::max(0.0, glm::dot(Design.vertices.front().position - plan_of(Design.start), Design.start_direction));
	Design.vertices.back().offset = std::max(0.0, glm::dot(plan_of(Design.end) - Design.vertices.back().position, Design.end_direction));
}

// the name of the path as the reasons of the digitization give it
std::string digitized_name(TTrack const *Track)
{
	return Track == nullptr || Track->name().empty() ? std::string{STR_C("(no name)")} : Track->name();
}

// the path of the chain with an end nearest to the point
TTrack *nearest_end(editor_track::chain const &Chain, glm::dvec2 const &Point)
{
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
}

// the design the digitization reads the chain into, the vertices yet to come; none when an end of the chain has no direction
std::optional<alignment::design> digitized_design(editor_track::chain const &Chain, alignment::design const &Route)
{
	alignment::design design;
	design.norms = Route.norms;
	design.transition_pieces = Route.transition_pieces;
	design.arc_piece_angle = Route.arc_piece_angle;
	design.speed = Chain.velocity > 0.0 ? Chain.velocity : 100.0;
	design.start = Chain.start;
	design.end = Chain.end;
	if (glm::length(plan_of(Chain.start_direction)) < 1e-9 || glm::length(plan_of(Chain.end_direction)) < 1e-9)
		return std::nullopt;
	design.start_direction = glm::normalize(plan_of(Chain.start_direction));
	design.end_direction = glm::normalize(plan_of(Chain.end_direction));
	design.start_grade = grade_of(Chain.start_direction);
	design.end_grade = grade_of(Chain.end_direction);
	design.start_radius = Chain.start_radius;
	design.end_radius = Chain.end_radius;
	design.start_turn = Chain.start_turn;
	design.end_turn = Chain.end_turn;
	return design;
}

// the attempt at reading the paths which kept closest to them, of those which don't follow them
struct digitize_best
{
	alignment::design design;
	alignment::result result;
	double closest{std::numeric_limits<double>::max()};
	TTrack *farthest{nullptr};

	void take(alignment::design const &Design, alignment::result const &Result, std::vector<TTrack *> const &Tracks)
	{
		TTrack *worst{nullptr};
		if (auto const away{alignment_store::deviation(Result, Tracks, worst)}; away < closest)
		{
			closest = away;
			farthest = worst;
			design = Design;
			result = Result;
		}
	}
	// why none of the attempts made the line, and the path it strayed farthest from
	void refuse(std::string *Reason, TTrack **Culprit) const
	{
		if (Culprit != nullptr)
			*Culprit = farthest;
		if (Reason == nullptr)
			return;
		if (closest < std::numeric_limits<double>::max())
			*Reason = format(STR_C("the curves read from the paths leave path %s by %.2f m"), digitized_name(farthest).c_str(), closest);
		else
			*Reason = std::string{STR_C("the paths don't make a line of straights, arcs and transition curves")};
	}
};

// the line read from the paths kept in the scenery in place of the lines kept along them, its chainage carried on from the first of
// them; false when one of them doesn't lie along it
bool store_digitized(std::vector<TTrack *> const &Tracks, alignment::design const &Design, alignment::result const &Result, double const Fit)
{
	auto const &records{alignment_store::records()};
	std::set<int> covered;
	for (auto const *track : Tracks)
		if (auto const found{alignment_store::on_track(*track)})
			covered.insert(found->record);
	double origin{0.0};
	for (auto const index : covered)
	{
		auto const &old{records[index]};
		auto const start{alignment_store::chainage_on(Result, alignment_store::kept_start(old))};
		auto const end{alignment_store::chainage_on(Result, alignment_store::kept_end(old))};
		if (false == start.has_value() || false == end.has_value())
			return false;
		if (index == *covered.begin() && *end > *start)
			origin = old.origin + old.from - *start;
	}
	auto const layer{covered.empty() ? alignment_store::layer_of(*Tracks.front()) : records[*covered.begin()].layer};
	for (auto index{covered.rbegin()}; index != covered.rend(); ++index)
		alignment_store::forget(*index);
	alignment_store::store(Design, Result, origin, layer, "drawn", Fit);
	return true;
}

// what was done before the paths were re-laid taken back: the tracks created retired, the ones changed put back as they were
void undo_relay(std::vector<TTrack *> const &Created, std::vector<std::pair<TTrack *, editor_track::state>> const &States)
{
	for (auto *track : Created)
		editor_track::retire(*track);
	for (auto entry = States.rbegin(); entry != States.rend(); ++entry)
	{
		editor_track::apply(*entry->first, entry->second);
		editor_track::commit({entry->first});
	}
}

// the track has an end at the point
bool has_end_at(TTrack const &Track, glm::dvec3 const &Point)
{
	return std::ranges::any_of(std::array{segment_data::point::start, segment_data::point::end}, [&Track, &Point](auto const Which) { return plan_distance(Track.m_paths.front().points[Which], Point) < 0.01; });
}

// a plain track of the scenery with an end at the point, none when there's no such
TTrack *normal_end_at(glm::dvec3 const &Point)
{
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && track->eType == tt_Normal && false == track->m_paths.empty() && has_end_at(*track, Point))
			return track;
	return nullptr;
}

// the status of the line laid: the paths re-laid and added, what the curve took over of the straights adjoining
std::string route_laid_note(std::size_t const Relaid, std::size_t const Added, double const Startextension, double const Endextension)
{
	auto note{Added > 0 ? format(STR_P("Re-laid %zu path, added %zu", "Re-laid %zu paths, added %zu", Relaid), Relaid, Added) : format(STR_P("Re-laid %zu path", "Re-laid %zu paths", Relaid), Relaid)};
	if (Startextension > 0.0 || Endextension > 0.0)
		note += format(STR_C(", the curve took over %.2f / %.2f m of the adjoining straights"), Startextension, Endextension);
	return note;
}


// the track as it is kept among the states, unless it's there already
void remember_state(std::vector<std::pair<TTrack *, editor_track::state>> &States, TTrack *Track)
{
	if (std::ranges::none_of(States, [Track](auto const &Entry) { return Entry.first == Track; }))
		States.emplace_back(Track, editor_track::capture(*Track));
}

// a switch of the chain where it goes along the new line
struct switch_placement
{
	TTrack *track;
	double from; // chainage along the new line
	double length;
	glm::dvec3 entry; // of the main track, where the chain comes in
	double angle;
};

// the chainages of the switches of the chain along the old line, with its Length
std::vector<double> switch_starts(editor_track::chain const &Chain, double &Length)
{
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
	Length = before;
	return starts;
}

// where the switches of the chain go along the new line, each one with its entry and its ends along it; the switches keep their
// distance from the nearer end of the chain. Why one doesn't fit, empty when they all do
std::string place_switches(editor_track::chain const &Chain, alignment::result const &Result, std::vector<switch_placement> &Placements, std::vector<double> &Ends)
{
	auto const plan = [](glm::dvec3 const &Vector) { return glm::normalize(plan_of(Vector)); };
	double oldlength{0.0};
	auto const starts{switch_starts(Chain, oldlength)};
	for (std::size_t k = 0; k < Chain.switches.size(); ++k)
	{
		auto const &passage{Chain.switches[k]};
		auto const &main{passage.track->m_paths.front()};
		auto const length{bezier{main}.plan_length()};
		auto const from{starts[k] + length * 0.5 < oldlength * 0.5 ? starts[k] : Result.length - (oldlength - starts[k])};
		if (from < 0.0 || from + length > Result.length || (false == Ends.empty() && from < Ends.back() - 0.01))
			return format(STR_C("The switch %s doesn't fit on the designed line"), name_of(*passage.track).c_str());
		auto const entry{main.points[passage.forward ? segment_data::point::start : segment_data::point::end]};
		auto const exit{main.points[passage.forward ? segment_data::point::end : segment_data::point::start]};
		bezier const curve{main};
		auto const direction{passage.forward ? plan(curve.first(0.0)) : -plan(curve.first(1.0))};
		auto const first{alignment::evaluate(Result, from)};
		auto const last{alignment::evaluate(Result, from + length)};
		auto const angle{signed_angle(direction, plan(first.direction))};
		// where the other end of the main track goes, set against the line
		auto const moved{plan_of(first.position) + turned(plan_of(exit - entry), angle)};
		if (auto const departure{glm::distance(moved, plan_of(last.position))}; departure > 0.02)
			return format(STR_C("The switch %s would leave the designed line by %.2f m at its other end: it lies in a curve of the design. "
			                    "Move the vertices so that the switch falls on a straight"),
			              name_of(*passage.track).c_str(), departure);
		Placements.emplace_back(passage.track, from, length, entry, angle);
		Ends.push_back(from);
		Ends.push_back(from + length);
	}
	return {};
}

// the switch moved onto the new line, the tracks joined to it off the chain following it; the heights stay
void place_switch(switch_placement const &Place, editor_track::chain const &Chain, alignment::result const &Result, std::vector<std::pair<TTrack *, editor_track::state>> &States)
{
	auto joints{editor_track::joints(*Place.track)};
	std::erase_if(joints, [&Chain](editor_track::joint const &Joint) { return std::ranges::find(Chain.tracks, Joint.other) != Chain.tracks.end(); });
	remember_state(States, Place.track);
	std::vector changed{Place.track};
	for (auto const &joint : joints)
	{
		remember_state(States, joint.other);
		if (std::ranges::find(changed, joint.other) == changed.end())
			changed.push_back(joint.other);
	}
	// the heights stay, the paths between the switches keep theirs too
	auto target{alignment::evaluate(Result, Place.from).position};
	target.y = Place.entry.y;
	editor_track::place(*Place.track, Place.entry, target, Place.angle);
	editor_track::follow(*Place.track, joints);
	editor_track::commit(changed);
}

// the intervals the new line is cut into for the parts of the chain between its switches and for the switches: where it's cut, how
// many paths each one takes, and whose they are: the part, -1 for none, and its path, -1 for any
struct relay_intervals
{
	std::vector<double> breaks;
	std::vector<std::size_t> counts;
	std::vector<std::pair<int, int>> owners;

	// the part of Count paths, cut at the joints of its paths where they follow one another along the line, else one interval
	void add_part(std::size_t const Part, std::size_t const Count, std::vector<double> const &Inner)
	{
		if (Count > 1 && Inner.size() + 1 == Count)
		{
			for (std::size_t j = 0; j < Count; ++j)
			{
				if (j > 0)
					breaks.push_back(Inner[j - 1]);
				counts.push_back(1);
				owners.emplace_back(static_cast<int>(Part), static_cast<int>(j));
			}
			return;
		}
		counts.push_back(Count);
		owners.emplace_back(static_cast<int>(Part), -1);
	}
	// the parts and the switches in their order along the chain; how many of the parts were taken
	std::size_t split(editor_track::chain const &Chain, std::vector<editor_track::chain> const &Parts, alignment::result const &Result, std::vector<double> const &Ends)
	{
		std::size_t part{0};
		for (std::size_t k = 0; k <= Chain.switches.size(); ++k)
		{
			auto const first{k == 0 ? std::size_t{0} : Chain.switches[k - 1].before};
			if (auto const last{k == Chain.switches.size() ? Chain.tracks.size() : Chain.switches[k].before}; last > first && part < Parts.size())
			{
				add_part(part, last - first, alignment::breaks_at(Result, editor_track::chain_joints(Parts[part]), k == 0 ? 0.0 : Ends[2 * k - 1], k == Chain.switches.size() ? Result.length : Ends[2 * k]));
				++part;
			}
			else
			{
				counts.push_back(last - first);
				owners.emplace_back(-1, -1);
			}
			if (k < Chain.switches.size())
			{
				breaks.push_back(Ends[2 * k]);
				breaks.push_back(Ends[2 * k + 1]);
				counts.push_back(1);
				owners.emplace_back(-1, -1);
			}
		}
		return part;
	}
};


// two curvatures the same but for the rounding
bool same_curvature(double const A, double const B)
{
	return std::abs(A - B) <= 1e-6 * std::max(std::abs(A), std::abs(B)) + 1e-12;
}

// the radii of the arcs of the curve of the vertex, of a curve of one arc its radius; none when the line has no such curve
std::vector<double> radii_of(alignment::result const &Result, int const Vertex)
{
	std::vector<double> radii;
	for (auto const &curve : Result.curves)
		if (curve.vertex == Vertex)
			radii = curve.radii.size() > 1 ? curve.radii : std::vector<double>{curve.radius};
	return radii;
}

// the element of the line in steps of about 2 m, each one handed to Step from its start to its end
template <typename Step>
void along_element(alignment::result const &Result, alignment::element const &Element, Step const &Draw)
{
	int const steps = std::clamp(static_cast<int>(Element.length / 2.0), 1, 400);
	auto previous{alignment::evaluate(Result, Element.chainage).position};
	for (int j = 1; j <= steps; ++j)
	{
		auto const next{alignment::evaluate(Result, Element.chainage + Element.length * j / steps).position};
		Draw(previous, next);
		previous = next;
	}
}

// the cant ramps: their ends across the track, the ones moved off the transitions brighter, with how the cant goes along them
void draw_cant_ramps(screen_projection const &Projection, ImDrawList *Drawlist, alignment::result const &Result)
{
	for (auto const &ramp : Result.ramps)
	{
		if (ramp.cant < 0.5)
			continue;
		auto const colour{ramp.own ? IM_COL32(255, 170, 40, 255) : IM_COL32(255, 170, 40, 120)};
		for (auto const ramp_end : {ramp.from, ramp.to})
		{
			auto const station{alignment::evaluate(Result, ramp_end)};
			glm::dvec3 const across{-station.direction.z * 2.0, 0.0, station.direction.x * 2.0};
			Projection.line(Drawlist, station.position - across, station.position + across, colour, 3.0f);
		}
		ImVec2 screen;
		if (ramp.own && Projection.project(alignment::evaluate(Result, (ramp.from + ramp.to) * 0.5).position, screen))
			Drawlist->AddText({screen.x + 8.0f, screen.y + 6.0f}, colour, format(STR_C("ramp %.0f m, %.0f mm"), ramp.to - ramp.from, ramp.cant).c_str());
	}
}

// the ends of the design, each one with an arrow along the track there
void draw_design_ends(screen_projection const &Projection, ImDrawList *Drawlist, alignment::design const &Design, glm::dvec3 const &Start, glm::dvec3 const &End)
{
	for (auto const atend : {false, true})
	{
		auto const &point{atend ? End : Start};
		auto const direction{atend ? Design.end_direction : Design.start_direction};
		if (ImVec2 screen; Projection.project(point, screen))
			Drawlist->AddRectFilled(ImVec2(screen.x - 5.0f, screen.y - 5.0f), ImVec2(screen.x + 5.0f, screen.y + 5.0f), IM_COL32(70, 140, 255, 255));
		auto const arrow{glm::dvec3{direction.x, 0.0, direction.y} * (atend ? 15.0 : -15.0)};
		Projection.line(Drawlist, point, point + arrow, IM_COL32(70, 140, 255, 255), 2.0f);
	}
}

// the radius of the curve by its grip, of a compound curve each one of its radii
std::string radius_label(alignment::curve_report const *Report, double const Radius)
{
	auto radius{format("R %.0f", Report != nullptr ? Report->radius : Radius)};
	if (Report != nullptr && Report->radii.size() >= 2)
	{
		radius = "R";
		for (std::size_t k = 0; k < Report->radii.size(); ++k)
			radius += format(k == 0 ? " %.0f" : " / %.0f", Report->radii[k]);
	}
	return radius;
}


// the curvatures at the ends of the element, to the left above 0
std::pair<double, double> signed_ends(alignment::element const &Element)
{
	auto const sign{static_cast<double>(Element.turn)};
	auto const [start, end]{alignment::end_curvatures(Element)};
	return {sign * start, sign * end};
}

// a label of the band takes the span from Left to Right unless one taken before is too close to it
bool place_label(std::vector<std::pair<float, float>> &Taken, float const Left, float const Right)
{
	for (auto const &[low, high] : Taken)
		if (Left < high + 4.0f && Right > low - 4.0f)
			return false;
	Taken.emplace_back(Left, Right);
	return true;
}

// the radius dragged in the band, to a metre below 1 km, to 10 m above
double rounded_radius(double const Radius)
{
	auto const step{Radius < 1000.0 ? 1.0 : 10.0};
	return std::clamp(std::round(Radius / step) * step, 10.0, 100000.0);
}

// the value of the vertex a drag in the band changes: of the Part 1 the radius of the arc, 2 the transition in, 3 the transition out,
// else the transition into the arc
template <typename Vertex>
auto &band_value(Vertex &Of, int const Part, int const Arc)
{
	auto const later{Arc >= 1 && Arc <= static_cast<int>(Of.arcs.size())};
	switch (Part)
	{
	case 1: return later ? Of.arcs[Arc - 1].radius : Of.radius;
	case 2: return Of.transition_in;
	case 3: return Of.transition_out;
	default: return later ? Of.arcs[Arc - 1].transition : Of.transition_in;
	}
}


// the share of an arc of a compound curve in percent of all of them
double share_percent(double const Share, double const Total)
{
	return Total > 0.0 ? 100.0 * std::max(0.0, Share) / Total : 0.0;
}

// what the curve is made of: an arc between two transitions, two transitions with no arc, or several arcs
bool render_vertex_kind(alignment::vertex &Vertex)
{
	bool changed{false};
	if (ImGui::Checkbox(STR_C("Two transition curves, no arc"), &Vertex.spirals))
	{
		changed = true;
		if (Vertex.spirals)
			Vertex.compound = false;
	}
	item_tooltip("The bend eased the way the lines are drawn: two transition curves meeting at the radius, with no arc between them and no cant; their lengths follow from the radius and the angle");
	if (false == Vertex.spirals && ImGui::Checkbox(STR_C("Compound curve (several radii)"), &Vertex.compound))
	{
		changed = true;
		if (Vertex.compound && Vertex.arcs.empty())
			Vertex.arcs.emplace_back(std::max(10.0, std::round(Vertex.radius * 0.6 / 10.0) * 10.0), 0.0, 1.0);
	}
	if (false == Vertex.spirals)
		item_tooltip("Curve made of arcs of different radii, each one its own path, joined with transitions");
	return changed;
}

} // namespace

bool editor_mode::route_active() const
{
	return ui()->mode() == nodebank_panel::TRACK && m_track_tab == track_tab::route && false == m_route.chain.tracks.empty();
}

void editor_mode::route_recommend(alignment::vertex &Vertex) const
{
	auto sharpest{Vertex.radius};
	if (Vertex.compound)
		for (auto const &arc : Vertex.arcs)
			sharpest = std::min(sharpest, arc.radius);
	auto const recommended{alignment::recommend(m_route.design.speed, sharpest, laying_norms())};
	Vertex.cant = recommended.cant;
	// the cants of the arcs in proportion to their curvature, as the norms recommend them
	Vertex.first_cant = -1.0;
	for (auto &arc : Vertex.arcs)
		arc.cant = -1.0;
	Vertex.transition_in = recommended.transition;
	Vertex.transition_out = recommended.transition;
	alignment::recommend_between(Vertex, m_route.design.speed, laying_norms());
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
	design.start_turn = chain.start_turn;
	design.end_turn = chain.end_turn;
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
	route.trace = {};
	route.clicks.clear();
	route.click_states.clear();
	route.measured = -1;
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
		route_curved_ends();
	}
	else
	{
		auto const distance{plan_distance(design.start, design.end)};
		vertex.offset = distance / 3.0;
		vertex.compound = false;
		vertex.reverse_turn = false;
		design.vertices = {vertex, vertex};
		route_update();
	}
	m_route.status = format(STR_P("Curve of %zu path, turning %d deg", "Curve of %zu paths, turning %d deg", m_route.chain.tracks.size()), m_route.chain.tracks.size(), static_cast<int>(std::round(glm::degrees(curve.turn))));
	return true;
}

void editor_mode::route_curved_ends()
{
	auto &design{m_route.design};
	design.start_curved = false;
	design.end_curved = false;
	if (design.vertices.empty() || m_route.chain.tracks.empty())
	{
		route_update();
		return;
	}
	// a curve beginning right at an end of the fragment at the curvature of the track adjoining there, turning its way, goes on from it,
	// as a curve laid on from the diverging track of a switch does: laid so again, not from a straight there; so next to a switch laid
	// into the curve too. A track of R 10 km or more is a straight there, as the lay tool takes it. The way the first curve and the last
	// one turn by the legs of the polygon
	auto const count{design.vertices.size()};
	design.start_curved = route_goes_on(false);
	design.end_curved = (count > 1 || false == design.start_curved) && route_goes_on(true);
	route_update();
	// a curve read from its paths as going on from the curve adjoining it which the design can't lay along them so, as one whose arc
	// going on is too short to be read as an arc: read from a straight there, whichever of the two keeps closer to its paths
	if (false == design.start_curved && false == design.end_curved)
		return;
	auto const curved{design_departure(m_route.result, m_route.chain.tracks)};
	if (false == (curved > 0.25))
		return;
	auto const kept{design};
	design.start_curved = false;
	design.end_curved = false;
	route_update();
	if (design_departure(m_route.result, m_route.chain.tracks) >= curved)
	{
		design = kept;
		route_update();
	}
}

bool editor_mode::route_goes_on(bool const Atend) const
{
	auto const &design{m_route.design};
	auto const radius{Atend ? design.end_radius : design.start_radius};
	if (radius <= 0.0 || radius >= 10000.0 || (Atend ? design.end_turn : design.start_turn) != design_turn(design, Atend))
		return false;
	// within a tenth: the path of a switch bent into the curve approximates it at its ends, R 860 m at the start of one in R 800 m
	auto const *track{Atend ? m_route.chain.tracks.back() : m_route.chain.tracks.front()};
	auto const bend{editor_track::bend_at(*track, Atend ? m_route.chain.end : m_route.chain.start)};
	return bend.has_value() && std::abs(std::abs(bend->curvature) - 1.0 / radius) <= 0.1 / radius;
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
	// the points clicked along the line when it was laid, wherever they're kept
	m_route.clicks = alignment_store::clicks_along(m_route.result, 3.0);
	m_route.measured = -1;
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
	auto const loose{std::ranges::any_of(chain.tracks, [](TTrack const *Track) { return false == alignment_store::on_track(*Track).has_value(); })};
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
	else if (auto const km{alignment_store::chainage_at(chain.start)}; km.has_value())
	{
		auto const ahead{chain.start + glm::dvec3{route.design.start_direction.x, 0.0, route.design.start_direction.y}};
		auto const further{alignment_store::chainage_at(ahead)};
		if (further.has_value() && *further > *km)
			route.origin = *km;
	}
	route.status += route_stored_note(digitized, parts, skipped, failure, loose && skipped < static_cast<int>(chain.tracks.size()));
}

std::string editor_mode::route_stored_note(bool const Digitized, int const Parts, int const Skipped, std::string const &Failure, bool const Left) const
{
	auto const &route{m_route};
	auto const failure{Failure.empty() ? std::string{} : " - " + Failure};
	if (Digitized && route.designed == 0)
		return STR_C("\nThe line is digitized now, a straight");
	if (Digitized)
		return format(STR_C("\nThe line is digitized now, curves kept in the scenery: %d"), route.designed);
	if (Parts > 0)
		return format(STR_C("\nThe line is digitized in parts: %d, paths left out: %d"), Parts, Skipped) + failure;
	if (Skipped > 0 && Left)
		return format(STR_C("\nPaths left out of the digitized line: %d"), Skipped) + failure;
	if (false == Failure.empty())
		return STR_C("\nThe line can't be digitized: ") + Failure;
	if (route.designed > 0)
		return format(STR_C("\nCurves read from the design kept in the scenery: %d"), route.designed);
	if (false == route.design.vertices.empty())
		return STR_C("\nCurves recognized from the paths, the scenery keeps no design for them");
	return {};
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
	// a curve laid going on from the curve of the track adjoining keeps going on from it, its radius the one which begins it there
	design.start_curved = stored.design.start_curved;
	design.end_curved = stored.design.end_curved;
	design.start_cant = stored.design.start_cant;
	design.end_cant = stored.design.end_cant;
	route.designed = static_cast<int>(design.vertices.size());
	route.origin = stored.origin;
	route.vertex = -1;
	route.grip = -1;
	route_update();
}

void editor_mode::render_route_stored()
{
	auto &route{m_route};
	auto km{route.origin / 1000.0};
	if (begin_fields("##routeorigin"))
	{
		field_row("Km of the start");
		if (ImGui::InputDouble("##routekm", &km, 0.0, 0.0, "%.3f"))
			route.origin = km * 1000.0;
		item_tooltip("Chainage of the start of the fragment, kept with the design in the scenery; the vertical profile of these paths counts from it");
		ImGui::EndTable();
	}
	auto const &records{alignment_store::records()};
	if (route.departed < 0 || route.departed >= static_cast<int>(records.size()) || false == scene::Layers.valid(records[route.departed].layer))
		return;
	auto const &layer{scene::Layers.layer(records[route.departed].layer).name};
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), STR_C("The design kept in %s for this fragment departs from its paths by up to %.2f m"), layer.c_str(), route.departure);
	ImGui::PopTextWrapPos();
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
		route.status = format(STR_C("The kept design will be removed from %s on save"), layer.c_str());
	}
	item_tooltip("The paths stay as they are, the scenery no longer keeps a design for them");
}

void editor_mode::route_store()
{
	auto &route{m_route};
	if (false == route.result.valid || route.chain.tracks.empty() || scene::Layers.empty())
		return;
	alignment_store::store(route.design, route.result, route.origin, alignment_store::layer_of(*route.chain.tracks.front()), "designed", 0.0, route.clicks);
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

void editor_mode::digitize_all(int *Kept, int *Failed)
{
	std::set<TTrack const *> visited;
	int kept{0};
	int failed{0};
	std::string first;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty() || false == editor_track::is_supported(*track) || visited.contains(track))
			continue;
		if (alignment_store::on_track(*track).has_value())
			continue;
		editor_track::chain chain;
		if (std::string error; false == editor_track::find_run(*track, kDrawnReach, chain, error) || chain.tracks.empty())
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
	if (Kept != nullptr)
		*Kept = kept;
	if (Failed != nullptr)
		*Failed = failed;
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
	if (count == 0 || std::ranges::all_of(Chain.tracks, [](TTrack const *Track) { return alignment_store::on_track(*Track).has_value(); }))
		return 0;
	TTrack *culprit{nullptr};
	std::string reason;
	if (digitize_chain(Chain, &reason, &culprit))
		return 1;
	if (count == 1)
	{
		++Skipped;
		reason = name_of(*Chain.tracks.front()) + ": " + reason;
		WriteLog("Editor: digitizing - " + reason, logtype::generic);
		if (Reason != nullptr && Reason->empty())
			*Reason = reason;
		return 0;
	}
	auto const found{static_cast<std::size_t>(std::ranges::find(Chain.tracks, culprit) - Chain.tracks.begin())};
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

bool editor_mode::digitize_chain(editor_track::chain const &Chain, std::string *Reason, TTrack **Culprit) const
{
	if (Chain.tracks.empty() || scene::Layers.empty())
		return false;
	auto read{digitized_design(Chain, m_route.design)};
	if (false == read.has_value())
		return false;
	auto &design{*read};
	auto const preferred{m_route.design.shape};
	auto const other{preferred == alignment::transition_shape::clothoid ? alignment::transition_shape::cubic_parabola : alignment::transition_shape::clothoid};
	alignment::result result;
	digitize_best best;
	bool fits{false};
	for (auto const attempt : {0, 1, 2, 3})
	{
		design.shape = attempt % 2 == 0 ? preferred : other;
		if (attempt == 2)
		{
			design.norms.arc_min = 0.0;
			design.norms.arc_min_factor = 0.0;
		}
		if (false == digitize_vertices(Chain, design, Reason, Culprit))
			return false;
		result = alignment::compute(design);
		fits = alignment_store::follows(result, Chain.tracks);
		if (fits)
			break;
		best.take(design, result, Chain.tracks);
	}
	double fit{0.0};
	if (false == fits && best.closest <= kDigitizeFit)
	{
		design = best.design;
		result = best.result;
		fit = best.closest;
		fits = true;
	}
	if (false == fits)
	{
		best.refuse(Reason, Culprit);
		return false;
	}
	return store_digitized(Chain.tracks, design, result, fit);
}

bool editor_mode::digitize_vertices(editor_track::chain const &Chain, alignment::design &Design, std::string *Reason, TTrack **Culprit) const
{
	Design.vertices.clear();
	for (auto const &entry : editor_track::recognize_line(Chain, m_straights.tolerance, Design.norms.gauge, Design.shape))
	{
		if (entry.kink)
		{
			if (Reason != nullptr)
				*Reason = format(STR_C("the straights meet with no curve at path %s"), digitized_name(nearest_end(Chain, entry.position)).c_str());
			if (Culprit != nullptr)
				*Culprit = nearest_end(Chain, entry.position);
			return false;
		}
		auto vertex{route_vertex_of(entry.shape)};
		vertex.position = entry.position;
		Design.vertices.push_back(vertex);
	}
	design_end_offsets(Design);
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
				route.designed += entry.shape.designed.has_value() ? 1 : 0;
			}
			vertex.position = entry.position;
			design.vertices.push_back(vertex);
		}
		design_end_offsets(design);
		route_curved_ends();
	}
	route.status = format(STR_P("Line of %zu path, %.0f m, %zu vertices", "Line of %zu paths, %.0f m, %zu vertices", route.chain.tracks.size()), route.chain.tracks.size(), route.chain.length, design.vertices.size());
	return true;
}

void editor_mode::route_apply()
{
	m_route.error.clear();
	route_lay();
	if (false == m_route.error.empty())
		refused(m_route.error);
}

void editor_mode::route_lay()
{
	auto &route{m_route};
	route.status.clear();
	m_infra_carried.clear();
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
	if (false == route.relaying && route_lay_crossovers(chain))
		return;

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	auto *from{route.from};
	auto *to{route.to};
	auto const startextension{route.result.start_extension};
	auto const endextension{route.result.end_extension};
	if ((startextension > 0.0 || endextension > 0.0) && false == route_cut_straights(chain, from, to, states, created))
		return;
	for (auto *track : chain.tracks)
		remember_state(states, track);
	std::vector<TTrack *> relaid;
	if (chain.switches.empty())
	{
		std::vector<std::size_t> intervals;
		auto pieces{alignment::pieces_at(route.result, route.design, chain.tracks.size(), editor_track::chain_joints(chain), intervals)};
		editor_track::keep_heights(chain, pieces);
		relaid = editor_track::relay(chain, pieces, &to, &intervals);
	}
	else if (false == route_relay_through_switches(chain, states, relaid, to))
	{
		undo_relay(created, states);
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
		route_extend_ends(startextension, endextension);
	route_store();
	records_to_history();
	route.status = route_laid_note(chain.tracks.size(), added, startextension, endextension);
	route.status += infra_carried_note();
	// the crossovers taken out before which fit the lines now
	if (false == route.relaying && false == route.crossovers_out.empty())
		route_crossovers_again();
	WriteLog("Editor: route design - " + route.status, logtype::generic);
}

void editor_mode::route_extend_ends(double const Startextension, double const Endextension)
{
	route_bind_ends();
	if (auto &vertices{m_route.design.vertices}; vertices.size() >= 2)
	{
		vertices.front().offset += Startextension;
		vertices.back().offset += Endextension;
	}
	route_update();
}

// the crossovers of the line can't be moved as their switches are: they're taken out, the line laid again, and they go back where
// they were, onto the new line
bool editor_mode::route_lay_crossovers(editor_track::chain const &Chain)
{
	auto &route{m_route};
	// a line which ends on a switch of a crossover would be laid again only up to it, the crossover left on a kink
	if (auto const *ending{route_end_crossover(Chain)}; ending != nullptr)
	{
		route.error = format(STR_C("The line ends on the switch %s of a crossover: move the crossover off the curve or take it out, change both lines, then lay it again"), ending->name().c_str());
		return true;
	}
	std::vector<TTrack *> switches;
	for (auto const &passage : Chain.switches)
		switches.push_back(passage.track);
	auto const crossovers{route_crossovers_on(switches)};
	if (crossovers.empty())
		return false;
	auto const tab{m_track_tab};
	route_take_out_crossovers(switches);
	show_track_tab(tab);
	// the line between the same points again: taking the lift back may have cut its paths
	auto *start{normal_end_at(route.chain.start)};
	auto *finish{normal_end_at(route.chain.end)};
	editor_track::chain refreshed;
	if (std::string error; start != nullptr && finish != nullptr && editor_track::find_chain(start, finish, refreshed, error, true))
	{
		route.from = start;
		route.to = finish;
		route.chain = refreshed;
	}
	route.relaying = true;
	auto const steps{m_history.size()};
	route_apply();
	route.relaying = false;
	if (m_history.size() == steps)
	{
		undo_last();
		return true;
	}
	join_last_steps();
	route_crossovers_back(crossovers, tab);
	return true;
}

TTrack const *editor_mode::route_end_crossover(editor_track::chain const &Chain) const
{
	auto const crossing = [this](auto const &Joined) { return Joined.first != nullptr && Joined.first->eType == tt_Switch && crossover_of(*Joined.first).has_value(); };
	for (auto const &end : {Chain.start, Chain.end})
		for (auto const *track : Chain.tracks)
		{
			auto const joined{editor_track::connected_points(*track, end)};
			if (auto const found{std::ranges::find_if(joined, crossing)}; found != joined.end())
				return found->first;
		}
	return nullptr;
}

std::vector<editor_mode::route_design::crossover_out> editor_mode::route_crossovers_on(std::vector<TTrack *> const &Switches)
{
	std::vector<route_design::crossover_out> crossovers;
	std::vector<TTrack const *> seen;
	for (auto const *switchtrack : Switches)
	{
		auto const crossover{crossover_of(*switchtrack)};
		if (false == crossover.has_value() || std::ranges::find(seen, crossover->first) != seen.end())
			continue;
		seen.push_back(crossover->first);
		// the press on this line, the release on the other one
		auto *here{crossover->first == switchtrack ? crossover->first : crossover->second};
		auto *there{here == crossover->first ? crossover->second : crossover->first};
		crossovers.push_back({crossover_template(crossover->record), here->m_paths.front().points[segment_data::point::start], there->m_paths.front().points[segment_data::point::start], crossover->first->name(), {here->name(), there->name()}});
	}
	return crossovers;
}

void editor_mode::route_take_out_crossovers(std::vector<TTrack *> const &Switches)
{
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	// the designs of the outer tracks come down with the lifts taken back, an undo puts them back
	profile_library_load();
	auto library{m_profiles};
	bool lifted{false};
	for (auto const *switchtrack : Switches)
		if (auto const crossover{crossover_of(*switchtrack)})
		{
			TTrack *first{nullptr};
			TTrack *second{nullptr};
			lifted = lifted || crossover->record.lifted();
			take_out_crossover(*crossover, states, created, removed, first, second);
		}
	switch_store::follow();
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	records_to_history();
	if (lifted)
		profiles_to_history(std::move(library));
}

void editor_mode::route_crossovers_back(std::vector<route_design::crossover_out> const &Crossovers, editor_track_tab const Tab)
{
	auto &route{m_route};
	std::string back;
	std::string left_side;
	for (auto const &crossover : Crossovers)
	{
		// the new line moved sideways: the press onto it, the release onto the other line
		if (lay_crossover(crossover.shape, crossover.press, crossover.release, 30.0, crossover.names))
		{
			join_last_steps();
			back += (back.empty() ? "" : ", ") + crossover.name;
		}
		else
		{
			left_side += (left_side.empty() ? "" : ", ") + crossover.name;
			route.crossovers_out.emplace_back(crossover);
		}
	}
	show_track_tab(Tab);
	if (false == back.empty())
		route.status += "\n" + format(STR_C("Crossovers laid again on the new line: %s"), back.c_str());
	if (false == left_side.empty())
		route.status += "\n" + format(STR_C("Crossovers taken out which don't fit the new line, lay them again: %s"), left_side.c_str());
}

bool editor_mode::route_cut_straights(editor_track::chain &Chain, TTrack *&From, TTrack *&To, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created)
{
	auto &route{m_route};
	auto const startextension{route.result.start_extension};
	auto const endextension{route.result.end_extension};
	auto const before{Chain};
	if (startextension > 0.0)
		From = editor_track::take_straight(editor_track::straight_beyond(before, false, m_straights.tolerance), From, before.start, route.result.start, States, Created);
	if (endextension > 0.0)
		To = editor_track::take_straight(editor_track::straight_beyond(before, true, m_straights.tolerance), To, before.end, route.result.end, States, Created);
	std::string error;
	auto const found{editor_track::find_chain(From, To, Chain, error, true)};
	auto const startoff{found && startextension > 0.0 ? plan_distance(Chain.start, route.result.start) : 0.0};
	if (auto const endoff{found && endextension > 0.0 ? plan_distance(Chain.end, route.result.end) : 0.0}; false == found || startoff > 5.0 || endoff > 5.0)
	{
		auto const startrun{editor_track::straight_beyond(before, false, m_straights.tolerance)};
		auto const endrun{editor_track::straight_beyond(before, true, m_straights.tolerance)};
		WriteLog(format("editor: cutting the straights failed: found %d, start %.2f m off (extension %.2f, run %zu paths %.2f m), end %.2f m off (extension %.2f, run %zu paths %.2f m), chain %zu paths, from %s, to %s, error '%s'", found ? 1 : 0, startoff,
		                startextension, startrun.tracks.size(), startrun.length, endoff, endextension, endrun.tracks.size(), endrun.length, Chain.tracks.size(), From != nullptr ? From->name().c_str() : "-", To != nullptr ? To->name().c_str() : "-", error.c_str()));
		undo_relay(Created, States);
		route.error = error.empty() ? format(STR_C("The adjoining straights can't be cut at the ends of the curve: the start misses by %.1f m, the end by %.1f m"), startoff, endoff) : error;
		return false;
	}
	if (false == tracks_editable(Chain.tracks, route.error))
	{
		undo_relay(Created, States);
		return false;
	}
	return true;
}


void editor_mode::route_crossovers_again()
{
	auto &route{m_route};
	auto const tab{m_track_tab};
	std::string back;
	for (auto out{route.crossovers_out.begin()}; out != route.crossovers_out.end();)
		if (lay_crossover(out->shape, out->press, out->release, 30.0, out->names))
		{
			join_last_steps();
			back += (back.empty() ? "" : ", ") + out->name;
			out = route.crossovers_out.erase(out);
		}
		else
			++out;
	show_track_tab(tab);
	if (false == back.empty())
		route.status += "\n" + format(STR_C("Crossovers laid again on the new line: %s"), back.c_str());
}

// the switches of the chain are moved as a whole onto the designed line, the regular paths between them re-laid
bool editor_mode::route_relay_through_switches(editor_track::chain const &Chain, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, TTrack *&Last)
{
	auto &route{m_route};
	auto const &result{route.result};
	std::vector<switch_placement> placements;
	std::vector<double> ends; // of the switches along the new line
	if (auto const error{place_switches(Chain, result, placements, ends)}; false == error.empty())
	{
		route.error = error;
		return false;
	}
	// the regular parts between the switches, cut at the joints of their paths set square onto the new line where they follow one
	// another along it, else one interval each; the switches taking the ones between
	auto const parts{editor_track::chain_parts(Chain)};
	relay_intervals cut;
	if (cut.split(Chain, parts, result, ends) != parts.size())
	{
		route.error = STR_C("The paths between the switches can't be read as a chain");
		return false;
	}
	std::vector<std::size_t> intervals;
	auto const pieces{alignment::pieces(result, route.design, 0, cut.breaks, cut.counts, &intervals)};
	std::vector<std::vector<segment_data>> groups(parts.size());
	std::vector<std::vector<std::size_t>> paths(parts.size()); // of each piece of a part, its path, where its joints cut the line
	for (std::size_t i = 0; i < pieces.size() && i < intervals.size(); ++i)
		if (intervals[i] < cut.owners.size() && cut.owners[intervals[i]].first >= 0)
		{
			auto const &[owner, path]{cut.owners[intervals[i]]};
			groups[owner].push_back(pieces[i]);
			if (path >= 0)
				paths[owner].push_back(static_cast<std::size_t>(path));
		}
	for (std::size_t i = 0; i < parts.size(); ++i)
		if (groups[i].size() < parts[i].tracks.size())
		{
			route.error = STR_C("The designed line between the switches is too short for the paths there");
			return false;
		}

	// all checked, the changes go in
	for (auto const &place : placements)
		place_switch(place, Chain, result, States);
	for (std::size_t i = 0; i < parts.size(); ++i)
	{
		auto group{groups[i]};
		editor_track::keep_heights(parts[i], group);
		TTrack *last{nullptr};
		auto const added{editor_track::relay(parts[i], group, &last, paths[i].size() == group.size() ? &paths[i] : nullptr)};
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

void editor_mode::route_show_curve(double const From, double const To)
{
	auto const &result{m_route.result};
	if (result.elements.empty() || To <= From)
		return;
	glm::dvec3 low{std::numeric_limits<double>::max()};
	glm::dvec3 high{std::numeric_limits<double>::lowest()};
	for (int i = 0; i <= 32; ++i)
	{
		auto const point{alignment::evaluate(result, From + (To - From) * i / 32.0).position};
		low = glm::min(low, point);
		high = glm::max(high, point);
	}
	auto const along{plan_of(alignment::evaluate(result, (From + To) * 0.5).direction)};
	auto const flat{glm::length(along) > 1e-9 ? glm::normalize(along) : glm::dvec2{0.0, 1.0}};
	// from above, leaning a little along the line, the whole curve in sight
	auto const look{glm::normalize(glm::dvec3{flat.x * 0.45, -1.0, flat.y * 0.45})};
	auto const extent{std::max(high.x - low.x, high.z - low.z)};
	fly_to((low + high) * 0.5, look, std::clamp(extent * 1.2, 60.0, 3000.0));
}

std::vector<std::size_t> editor_mode::route_focused() const
{
	using part = route_design::curve_focus::part;
	auto const &focus{m_route.focus};
	auto const &result{m_route.result};
	std::vector<std::size_t> own;
	if (focus.vertex < 0)
		return own;
	for (std::size_t i = 0; i < result.elements.size(); ++i)
		if (result.elements[i].vertex == focus.vertex && result.elements[i].length > 1e-6)
			own.push_back(i);
	if (own.empty() || focus.what == part::whole)
		return own;
	// the arcs of a compound curve found by the curvature at the start and at the end of each element and their radii
	auto const radii{radii_of(result, focus.vertex)};
	std::vector<std::size_t> found;
	for (auto const i : own)
		if (route_focus_takes(i, own, radii))
			found.push_back(i);
	// a part of no length, the arc of two transition curves meeting say, shows the whole curve
	return found.empty() ? own : found;
}

bool editor_mode::route_focus_takes(std::size_t const Element, std::vector<std::size_t> const &Own, std::vector<double> const &Radii) const
{
	using part = route_design::curve_focus::part;
	auto const &focus{m_route.focus};
	auto const [start, end] {alignment::end_curvatures(m_route.result.elements[Element])};
	auto const arc{static_cast<std::size_t>(focus.arc)};
	switch (focus.what)
	{
	case part::entry: return Element == Own.front() && start == 0.0 && end > 0.0;
	case part::exit: return Element == Own.back() && end == 0.0 && start > 0.0;
	case part::radius_arc: return arc < Radii.size() && Radii[arc] > 0.0 && same_curvature(start, 1.0 / Radii[arc]) && same_curvature(end, start);
	case part::between: return arc >= 1 && arc < Radii.size() && Radii[arc] > 0.0 && Radii[arc - 1] > 0.0 && same_curvature(start, 1.0 / Radii[arc - 1]) && same_curvature(end, 1.0 / Radii[arc]);
	default: return false;
	}
}

void editor_mode::route_measure_clicks()
{
	auto &route{m_route};
	if (route.measured == route.revision)
		return;
	route.measured = route.revision;
	route.click_states.clear();
	auto const &result{route.result};
	if (route.clicks.empty() || false == result.valid)
		return;
	std::vector<glm::dvec2> plan;
	for (auto const &click : route.clicks)
		plan.push_back(plan_of(click));
	auto const projections{alignment::project(result, plan)};
	auto const count{static_cast<int>(route.design.vertices.size())};
	for (std::size_t i = 0; i < plan.size() && i < projections.size(); ++i)
	{
		route_design::click_state state;
		state.offset = projections[i].offset;
		// the curve nearest along the line, as the points traced in the plan go to their curves
		for (int k = 0; k < count && k < static_cast<int>(result.vertex_chainages.size()); ++k)
			if (state.vertex < 0 || std::abs(result.vertex_chainages[k] - projections[i].chainage) < std::abs(result.vertex_chainages[state.vertex] - projections[i].chainage))
				state.vertex = k;
		for (int k = 0; k < count && false == state.corner; ++k)
			state.corner = plan_distance(route.clicks[i], route_vertex_position(k)) < 0.05;
		route.click_states.push_back(state);
	}
}

std::vector<glm::dvec2> editor_mode::route_clicks_of(int const Vertex) const
{
	auto const &route{m_route};
	std::vector<glm::dvec2> points;
	for (std::size_t i = 0; i < route.click_states.size() && i < route.clicks.size(); ++i)
	{
		auto const &state{route.click_states[i]};
		// a point clicked far off the line belongs to none of its curves
		if (state.vertex == Vertex && false == state.corner && std::abs(state.offset) <= 3.0)
			points.push_back(plan_of(route.clicks[i]));
	}
	return points;
}

bool editor_mode::route_hold_clicks(int const Vertex)
{
	auto &route{m_route};
	route_measure_clicks();
	auto const points{route_clicks_of(Vertex)};
	if (points.size() < 3 || Vertex < 0 || Vertex >= static_cast<int>(route.design.vertices.size()))
		return false;
	auto design{route.design};
	design.norms.checked = true;
	auto const fit{alignment::refit_radii(design, static_cast<std::size_t>(Vertex), points)};
	if (false == fit.valid)
		return false;
	route.design.vertices[Vertex] = fit.fitted;
	return true;
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
		draw_route_line(projection, drawlist);

	draw_design_ends(projection, drawlist, design, start, end);
	for (int i = 0; i < static_cast<int>(design.vertices.size()); ++i)
		draw_route_vertex(projection, drawlist, i);
	draw_route_clicks(projection, drawlist);
	draw_route_traced(projection, drawlist);
}

void editor_mode::draw_route_line(screen_projection const &Projection, ImDrawList *Drawlist) const
{
	auto const &route{m_route};
	auto const &result{route.result};
	for (auto const &e : result.elements)
	{
		ImU32 const color = element_colour(e.kind);
		along_element(result, e, [&Projection, Drawlist, color](glm::dvec3 const &From, glm::dvec3 const &To) { Projection.line(Drawlist, From, To, color, 3.0f); });
		ImVec2 screen;
		if (Projection.project(alignment::evaluate(result, e.chainage).position, screen))
			Drawlist->AddCircleFilled(screen, 3.5f, overlay_color::highlight);
	}
	// what the mouse points at in the band of the curvature in the panel
	if (route.band.hot >= 0 && route.band.hot < static_cast<int>(result.elements.size()))
		along_element(result, result.elements[route.band.hot], [&Projection, Drawlist](glm::dvec3 const &From, glm::dvec3 const &To) { Projection.line(Drawlist, From, To, IM_COL32(255, 255, 255, 110), 8.0f); });
	draw_cant_ramps(Projection, Drawlist, result);
	// the part of the curve whose fields are pointed at or edited in the panel, lit up and named
	if (auto const focused{route_focused()}; false == focused.empty())
		draw_route_focus(Projection, Drawlist, focused);
	ImVec2 pointed;
	if (route.band.at >= 0.0 && Projection.project(alignment::evaluate(result, std::min(route.band.at, result.length)).position, pointed))
	{
		Drawlist->AddCircleFilled(pointed, 5.0f, overlay_color::highlight);
		Drawlist->AddCircle(pointed, 10.0f, overlay_color::highlight, 20, 2.0f);
	}
}

void editor_mode::draw_route_focus(screen_projection const &Projection, ImDrawList *Drawlist, std::vector<std::size_t> const &Focused) const
{
	auto const &result{m_route.result};
	for (auto const index : Focused)
		along_element(result, result.elements[index], [&Projection, Drawlist](glm::dvec3 const &From, glm::dvec3 const &To) {
			Projection.line(Drawlist, From, To, IM_COL32(60, 230, 255, 120), 11.0f);
			Projection.line(Drawlist, From, To, IM_COL32(200, 250, 255, 255), 3.0f);
		});
	auto const &first{result.elements[Focused.front()]};
	auto const &last{result.elements[Focused.back()]};
	auto const name{route_focus_name(first.length, last.length)};
	ImVec2 middle;
	if (false == name.empty() && Projection.project(alignment::evaluate(result, (first.chainage + last.chainage + last.length) * 0.5).position, middle))
	{
		auto const size{ImGui::GetFontSize() * 1.25f};
		auto const text{ImGui::GetFont()->CalcTextSizeA(size, FLT_MAX, 0.0f, name.c_str())};
		ImVec2 const corner{middle.x - text.x * 0.5f, middle.y - text.y - 22.0f};
		Drawlist->AddRectFilled(ImVec2(corner.x - 6.0f, corner.y - 3.0f), ImVec2(corner.x + text.x + 6.0f, corner.y + text.y + 3.0f), IM_COL32(10, 20, 30, 210), 4.0f);
		Drawlist->AddText(ImGui::GetFont(), size, corner, IM_COL32(200, 250, 255, 255), name.c_str());
		Drawlist->AddLine(ImVec2(middle.x, corner.y + text.y + 3.0f), middle, IM_COL32(200, 250, 255, 200), 1.5f);
	}
}

std::string editor_mode::route_focus_name(double const Firstlength, double const Lastlength) const
{
	using part = route_design::curve_focus::part;
	auto const &focus{m_route.focus};
	alignment::curve_report const *report{nullptr};
	for (auto const &curve : m_route.result.curves)
		if (curve.vertex == focus.vertex)
			report = &curve;
	if (report == nullptr)
		return {};
	auto const compound{report->radii.size() > 1};
	auto const arc{static_cast<std::size_t>(focus.arc)};
	switch (focus.what)
	{
	case part::entry: return format(STR_C("W%d  transition in %.0f m"), focus.vertex + 1, Firstlength);
	case part::exit: return format(STR_C("W%d  transition out %.0f m"), focus.vertex + 1, Lastlength);
	case part::between: return format(STR_C("W%d  transition R%d-R%d %.0f m"), focus.vertex + 1, focus.arc, focus.arc + 1, Firstlength);
	case part::radius_arc:
		if (compound && arc < report->radii.size())
			return format(STR_C("W%d  R%d %.0f m  cant %.0f mm"), focus.vertex + 1, focus.arc + 1, report->radii[arc], arc < report->cants.size() ? report->cants[arc] : report->cant);
		return format(STR_C("W%d  R %.0f m  cant %.0f mm"), focus.vertex + 1, report->radius, report->cant);
	default: return format("W%d  ", focus.vertex + 1) + curve_label(*report, m_route.design.norms);
	}
}

void editor_mode::draw_route_vertex(screen_projection const &Projection, ImDrawList *Drawlist, int const Index) const
{
	auto const &route{m_route};
	auto const &design{route.design};
	auto const &result{route.result};
	ImVec2 screen;
	if (false == Projection.project(route_vertex_position(Index), screen))
		return;
	Drawlist->AddCircleFilled(screen, 6.0f, overlay_color::marked);
	if (Index == route.vertex)
		Drawlist->AddCircle(screen, 11.0f, overlay_color::highlight, 16, 2.5f);
	if (Index < static_cast<int>(result.vertex_chainages.size()) && result.length > 0.0 && Projection.project(alignment::evaluate(result, result.vertex_chainages[Index]).position, screen))
	{
		Drawlist->AddCircleFilled(screen, 6.0f, overlay_color::grip, 4);
		if (Index == route.grip)
			Drawlist->AddCircle(screen, 11.0f, overlay_color::highlight, 16, 2.5f);
		auto const *report{Index < static_cast<int>(result.curves.size()) ? &result.curves[Index] : nullptr};
		Drawlist->AddText(ImVec2(screen.x + 9.0f, screen.y + 4.0f), overlay_color::grip, radius_label(report, design.vertices[Index].radius).c_str());
	}
	Drawlist->AddText(ImVec2(screen.x + 9.0f, screen.y - 18.0f), overlay_color::marked, format("W%d", Index + 1).c_str());
}

// the points clicked when the line was laid, in the colours of their departure; how far each is, along the curve in focus
void editor_mode::draw_route_clicks(screen_projection const &Projection, ImDrawList *Drawlist) const
{
	auto const &route{m_route};
	for (std::size_t i = 0; i < route.clicks.size() && i < route.click_states.size(); ++i)
	{
		auto const &state{route.click_states[i]};
		ImVec2 screen;
		if (state.corner || std::abs(state.offset) > 3.0 || false == Projection.project(route.clicks[i], screen))
			continue;
		auto const departure{std::abs(state.offset)};
		auto const colour{departure_colour(departure)};
		Drawlist->AddCircleFilled(screen, 4.5f, IM_COL32(0, 0, 0, 200), 12);
		Drawlist->AddCircleFilled(screen, 3.0f, colour, 12);
		if (state.vertex == route.focus.vertex)
			Drawlist->AddText(ImVec2(screen.x + 7.0f, screen.y - 7.0f), colour, format("%.0f cm", departure * 100.0).c_str());
	}
}

// the points traced on the orthophoto, in the colours of their departure from the curve fitted to them
void editor_mode::draw_route_traced(screen_projection const &Projection, ImDrawList *Drawlist) const
{
	auto const &route{m_route};
	if (route.trace.points.empty())
		return;
	std::vector<glm::dvec2> plan;
	for (auto const &point : route.trace.points)
		plan.push_back(plan_of(point));
	auto const projections{alignment::project(route.result, plan)};
	auto const hit{route_trace_hit()};
	for (std::size_t i = 0; i < route.trace.points.size(); ++i)
	{
		ImVec2 screen;
		if (false == Projection.project(route.trace.points[i], screen))
			continue;
		auto const departure{i < projections.size() ? std::abs(projections[i].offset) : 0.0};
		auto const colour{departure_colour(departure)};
		Drawlist->AddCircle(screen, 5.0f, IM_COL32(0, 0, 0, 200), 12, 3.5f);
		Drawlist->AddCircle(screen, 5.0f, colour, 12, 2.0f);
		if (static_cast<int>(i) == hit)
			Drawlist->AddCircle(screen, 10.0f, overlay_color::highlight, 16, 2.0f);
		if (i < projections.size())
			Drawlist->AddText(ImVec2(screen.x + 8.0f, screen.y - 7.0f), colour, format("%.0f cm", departure * 100.0).c_str());
	}
}

void editor_mode::render_route_trace()
{
	auto &trace{m_route.trace};
	// the curve is traced on the photo with the laying (Tab), the plan has no button of its own for it
	if (false == trace.active)
		return;
	ImGui::SeparatorText(STR_C("Tracing the curve"));
	if (segmented("traceradii", trace.radii,
	              {{0, STR_C("Auto"), "As few radii as follow the points, up to three"}, {1, STR_C("1 R"), "One radius"}, {2, STR_C("2 R"), "A compound curve of two radii"}, {3, STR_C("3 R"), "A compound curve of three radii"}}))
		route_trace_fit();
	if (trace.points.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("Click points along the middle of the track on the photo, spread over the whole curve; a few on the straights at its ends help too"));
		ImGui::PopTextWrapPos();
	}
	else
	{
		metric_tiles({{format("%zu", trace.points.size()), STR_C("points"), ImGui::GetColorU32(ImGuiCol_Text)},
		              {format("%.0f cm", trace.rms * 100.0), STR_C("departure"), departure_colour(trace.rms)},
		              {format("%.0f cm", trace.worst * 100.0), STR_C("at most"), departure_colour(trace.worst)}});
	}
	if (main_button(STR_C("Apply   Enter"), ImVec2(-1.0f, 0.0f)))
		route_trace_end(true);
	if (ImGui::Button(STR_C("Drop   Esc")))
		route_trace_end(false);
	ImGui::SameLine();
	ImGui::BeginDisabled(trace.points.empty());
	if (ImGui::Button(STR_C("Take out the last point   Backspace")))
	{
		trace.points.pop_back();
		route_trace_fit();
	}
	ImGui::EndDisabled();
}

void editor_mode::route_trace_start()
{
	auto &trace{m_route.trace};
	trace.active = true;
	trace.points.clear();
	trace.before = m_route.design;
	trace.rms = 0.0;
	trace.worst = 0.0;
}

// every point goes to the curve nearest to it along the line, the curves without points keep their shape
void editor_mode::route_trace_fit()
{
	auto &route{m_route};
	auto &trace{route.trace};
	auto design{route.design};
	trace.rms = 0.0;
	trace.worst = 0.0;
	if (trace.points.empty() || design.vertices.empty() || false == route.result.valid)
		return;
	std::vector<glm::dvec2> plan;
	for (auto const &point : trace.points)
		plan.push_back(plan_of(point));
	auto const projections{alignment::project(route.result, plan)};
	std::vector<std::vector<glm::dvec2>> owned(design.vertices.size());
	for (std::size_t i = 0; i < plan.size(); ++i)
	{
		std::size_t nearest{0};
		for (std::size_t k = 1; k < route.result.vertex_chainages.size() && k < owned.size(); ++k)
			if (std::abs(route.result.vertex_chainages[k] - projections[i].chainage) < std::abs(route.result.vertex_chainages[nearest] - projections[i].chainage))
				nearest = k;
		owned[nearest].push_back(plan[i]);
	}
	for (std::size_t k = 0; k < owned.size(); ++k)
	{
		if (owned[k].empty())
			continue;
		auto const fit{alignment::fit_trace(design, k, owned[k], trace.radii)};
		if (fit.valid)
			design.vertices[k] = fit.fitted;
	}
	route.design = design;
	route_update();
	double sum{0.0};
	for (auto const &projection : alignment::project(route.result, plan))
	{
		sum += projection.offset * projection.offset;
		trace.worst = std::max(trace.worst, std::abs(projection.offset));
	}
	trace.rms = std::sqrt(sum / static_cast<double>(plan.size()));
}

void editor_mode::route_trace_end(bool const Keep)
{
	auto &trace{m_route.trace};
	if (false == trace.active)
		return;
	auto const traced{false == trace.points.empty()};
	if (false == Keep)
	{
		m_route.design = trace.before;
		route_update();
	}
	trace = {};
	if (Keep && traced)
		route_apply();
}

// the photo under the cursor where the depth finds it, else the level of the design there
glm::dvec3 editor_mode::route_trace_cursor() const
{
	if (GfxRenderer->Mouse_Hit())
		return cursor_ground();
	auto const &result{m_route.result};
	auto point{cursor_level(alignment::elevation(result, result.length * 0.5))};
	if (auto const projections{alignment::project(result, {plan_of(point)})}; false == projections.empty())
		point = cursor_level(alignment::elevation(result, projections.front().chainage));
	return point;
}

int editor_mode::route_trace_hit() const
{
	screen_projection const projection;
	float best{kHandleRadius * kHandleRadius};
	int hit{-1};
	for (int i = 0; i < static_cast<int>(m_route.trace.points.size()); ++i)
		if (auto const distance{projection.mouse_distance2(m_route.trace.points[i])}; distance < best)
		{
			best = distance;
			hit = i;
		}
	return hit;
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
	if (false == m_route_dragging)
		m_route_drag_from = route.design;
	m_route_dragging = true;
	if (grip)
	{
		route_drag_grip(moved);
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

void editor_mode::route_drag_grip(glm::dvec3 const &Moved)
{
	auto &route{m_route};
	auto &vertex{route.design.vertices[route.grip]};
	auto const corner{route_vertex_position(route.grip)};
	auto const distance{plan_distance(Moved, corner)};
	for (auto const &curve : route.result.curves)
	{
		if (curve.vertex != route.grip || vertex.reverse_turn)
			continue;
		auto const secant{1.0 / std::cos(curve.deflection * 0.5) - 1.0};
		if (secant > 1e-6)
			vertex.radius = std::max(10.0, std::round(distance / secant));
	}
	route_update();
}

void editor_mode::render_route_ui()
{
	auto &route{m_route};
	if (false == route.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", route.error.c_str());
	if (route.chain.tracks.empty())
		return;

	ImGui::AlignTextToFramePadding();
	ImGui::TextDisabled(STR_P("%zu path, %.2f m", "%zu paths, %.2f m", route.chain.tracks.size()), route.chain.tracks.size(), route.chain.length);
	auto const reset{STR_C("Reset design")};
	auto const reset_width{ImGui::CalcTextSize(reset).x + ImGui::GetStyle().FramePadding.x * 2.0f};
	ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - reset_width));
	if (ImGui::Button(reset))
	{
		auto *track{route.chain.tracks[route.chain.tracks.size() / 2]};
		route_load(*track);
		return;
	}
	item_tooltip("Reads the curves of the line from the scenery again, dropping the changes not applied");
	auto changed{render_route_band()};
	render_route_trace();
	render_route_stored();
	changed |= render_route_vertices();
	if (ImGui::CollapsingHeader(STR_C("Design speed and limits")))
		changed |= render_route_parameters();
	if (changed)
		route_update();

	ImGui::Separator();
	render_route_result();

	if (ImGui::CollapsingHeader(STR_C("Digitize the whole scenery")))
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("Keeps the plan of the lines in the scenery as straights, arcs and transitions, so the tools can edit them; the paths stay as they are"));
		ImGui::PopTextWrapPos();
		if (ImGui::Button(STR_C("Digitize all the tracks")))
		{
			digitize_all();
			return;
		}
		item_tooltip("Every line of the scenery not digitized yet is kept in it with its straights, arcs and transition curves, the way the drawn tracks are kept; the paths stay as they are");
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Digitize the scenery")))
		{
			digitize_scenery(true);
			return;
		}
		item_tooltip("All at once: the lines, the switches with their types out of the catalogue (or the size measured from their shape), the crossovers as connections of switches and the signals, signs, poles and catenary bound to the tracks. The paths and the objects stay as they are, the scenery keeps it on save");
	}
}

// the curvature along the fragment, the way the designers draw it: straights on the axis, the transitions as ramps,
// the arcs as plateaus at 1/R, in the colours of the overlay in the view. dragging a plateau changes its radius,
// dragging a ramp sideways the length of the transition, the wheel does both in steps
bool editor_mode::render_route_band()
{
	auto &route{m_route};
	auto &design{route.design};
	auto const &result{route.result};
	auto &band{route.band};
	band.hot = -1;
	band.at = -1.0;
	if (result.elements.empty() || result.length <= 1e-3 || design.vertices.empty())
	{
		band.vertex = -1;
		return false;
	}
	auto const font{ImGui::GetFontSize()};
	auto const width{ImGui::GetContentRegionAvail().x};
	auto const height{std::round(font * 6.0f)};
	auto const top{ImGui::GetCursorScreenPos()};
	ImGui::InvisibleButton("##curvature", ImVec2(width, height));
	ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
	auto const hovered{ImGui::IsItemHovered()};
	auto const active{ImGui::IsItemActive()};
	auto *list{ImGui::GetWindowDrawList()};
	auto view{route_band_layout(top, ImVec2(width, height))};
	list->AddRectFilled(view.top, view.bottom, ImGui::GetColorU32(ImGuiCol_FrameBg), std::round(font * 0.35f));
	std::vector<int> arcs;
	auto const roles{route_band_roles(result, design.vertices.size(), arcs)};
	// the element under the mouse and what of it a drag changes: near the ends of a plateau the transitions either side of it, also the
	// ones of no length yet
	if (hovered && false == view.dragging)
		route_band_pick(view, roles, arcs);
	bool changed{route_band_drag(view, active)};
	changed = route_band_wheel(view) || changed;

	ImGui::PushClipRect(view.top, view.bottom, true);
	draw_route_band(view, list);
	draw_route_band_labels(view, list);
	// the point under the mouse, shown in the view too
	if (hovered || view.dragging)
	{
		auto const at{std::clamp(static_cast<double>((view.mouse.x - view.inner_left) / std::max(1e-6f, view.metres)), 0.0, result.length)};
		band.at = at;
		band.hot = view.dragging ? -1 : view.under;
		if (false == view.dragging)
			list->AddLine(ImVec2(view.mouse.x, view.plot_top), ImVec2(view.mouse.x, view.plot_bottom), IM_COL32(255, 255, 255, 60));
	}
	if (view.dragging && band.vertex >= 0 && band.vertex < static_cast<int>(design.vertices.size()))
		draw_route_band_value(view, list);
	ImGui::PopClipRect();
	if (changed)
		first_step_done(first_step::plan_band);

	if (view.part != 0 && false == view.dragging && ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
		route_band_tooltip(view, arcs);
	return changed;
}

editor_mode::route_band_view editor_mode::route_band_layout(ImVec2 const Top, ImVec2 const Size) const
{
	auto const &result{m_route.result};
	auto const &band{m_route.band};
	route_band_view view;
	view.font = ImGui::GetFontSize();
	view.pad = std::round(view.font * 0.5f);
	view.top = Top;
	view.bottom = ImVec2{Top.x + Size.x, Top.y + Size.y};
	double peak{0.0};
	bool on_left{false};
	bool on_right{false};
	for (auto const &element : result.elements)
	{
		auto const [from, to]{signed_ends(element)};
		peak = std::max({peak, std::abs(from), std::abs(to)});
		on_left |= from > 1e-9 || to > 1e-9;
		on_right |= from < -1e-9 || to < -1e-9;
	}
	view.line = ImGui::GetTextLineHeight();
	view.inner_left = Top.x + view.pad;
	view.inner_width = Size.x - 2.0f * view.pad;
	view.plot_top = Top.y + view.pad * 0.5f + view.line;
	view.plot_bottom = view.bottom.y - view.pad * 0.5f - view.line;
	view.both = on_left && on_right;
	view.axis = view.both ? std::round((view.plot_top + view.plot_bottom) * 0.5f) : view.plot_bottom;
	// the curves to one side only grow upwards
	view.flip = (false == view.both && on_right) ? -1.0f : 1.0f;
	view.dragging = band.vertex >= 0;
	view.metres = view.dragging ? band.metres : view.inner_width / static_cast<float>(result.length);
	if (view.dragging)
		view.curvature = band.curvature;
	else if (peak > 0.0)
		view.curvature = (view.axis - view.plot_top) * 0.85f / static_cast<float>(peak);
	view.mouse = ImGui::GetIO().MousePos;
	return view;
}

// what of its vertex each element is: an arc, the arcs of a compound curve counted from the first one, which are elements of an even
// curvature as an arc is, or a transition, in from the straight, out to it, or between two arcs, which goes on from the arc before it
std::vector<editor_mode::route_band_view::role> editor_mode::route_band_roles(alignment::result const &Result, std::size_t const Vertices, std::vector<int> &Arcs)
{
	auto const count{static_cast<int>(Result.elements.size())};
	std::vector<route_band_view::role> roles(static_cast<std::size_t>(count));
	Arcs.assign(Vertices, 0);
	int vertex{-1};
	int arc{-1};
	double last{0.0};
	for (int i = 0; i < count; ++i)
	{
		auto const &element{Result.elements[i]};
		if (element.vertex != vertex)
		{
			vertex = element.vertex;
			arc = -1;
			last = 0.0;
		}
		if (element.vertex < 0 || element.vertex >= static_cast<int>(Vertices) || element.kind == alignment::element_kind::straight)
			continue;
		auto const [from, to]{signed_ends(element)};
		if (element.kind == alignment::element_kind::arc || (std::abs(from - to) <= 1e-6 * std::abs(from) && std::abs(from) > 1e-9))
		{
			// the next arc where the curvature isn't the one of the arc before, an arc cut in parts staying one
			if (arc < 0 || std::abs(std::abs(from) - last) > 1e-6 * std::abs(from))
				++arc;
			last = std::abs(from);
			roles[i] = {1, arc};
			Arcs[element.vertex] = std::max(Arcs[element.vertex], arc + 1);
		}
		else if (std::abs(from) <= 1e-9)
			roles[i] = route_band_view::role{2, 0};
		else if (std::abs(to) <= 1e-9)
			roles[i] = route_band_view::role{3, 0};
		else
			roles[i] = route_band_view::role{4, arc + 1};
	}
	return roles;
}

void editor_mode::route_band_pick(route_band_view &View, std::vector<route_band_view::role> const &Roles, std::vector<int> const &Arcs) const
{
	auto const &result{m_route.result};
	for (int i = 0; i < static_cast<int>(result.elements.size()); ++i)
	{
		auto const &element{result.elements[i]};
		if (View.mouse.x < View.x_of(element.chainage) || View.mouse.x >= View.x_of(element.chainage + element.length))
			continue;
		View.under = i;
		View.part = Roles[i].part;
		View.arc = Roles[i].arc;
		if (View.part == 1)
		{
			auto const from{View.x_of(element.chainage)};
			auto const to{View.x_of(element.chainage + element.length)};
			auto const edge{std::min(std::round(View.font * 0.5f), (to - from) * 0.25f)};
			if (View.mouse.x < from + edge)
				View.part = View.arc == 0 ? 2 : 4;
			else if (View.mouse.x >= to - edge)
			{
				View.part = View.arc + 1 >= Arcs[element.vertex] ? 3 : 4;
				View.arc += 1;
			}
		}
		return;
	}
}

bool editor_mode::route_band_drag(route_band_view const &View, bool const Active)
{
	auto &design{m_route.design};
	auto &band{m_route.band};
	auto const &result{m_route.result};
	if (View.part != 0 && ImGui::IsItemActivated())
	{
		band.vertex = result.elements[View.under].vertex;
		band.part = View.part;
		band.arc = View.arc;
		band.value = band_value(design.vertices[band.vertex], View.part, View.arc);
		band.press = View.mouse;
		band.metres = View.metres;
		band.curvature = View.curvature;
	}
	if (false == (band.vertex >= 0 && band.vertex < static_cast<int>(design.vertices.size()) && Active))
	{
		if (View.dragging && false == Active)
			band.vertex = -1;
		return false;
	}
	auto &value{band_value(design.vertices[band.vertex], band.part, band.arc)};
	double wanted{0.0};
	if (band.part == 1)
		wanted = route_band_radius(View);
	else
	{
		// the transition in grows to the left, from where it starts; the others to the right
		auto const shift{(View.mouse.x - band.press.x) / std::max(1e-6f, View.metres)};
		wanted = std::max(0.0, std::round(band.value + (band.part == 2 ? -shift : shift)));
	}
	bool changed{false};
	if (wanted != value)
	{
		value = wanted;
		changed = true;
	}
	ImGui::SetMouseCursor(band.part == 1 ? ImGuiMouseCursor_ResizeNS : ImGuiMouseCursor_ResizeEW);
	return changed;
}

double editor_mode::route_band_radius(route_band_view const &View) const
{
	auto const &band{m_route.band};
	int turn{1};
	for (auto const &element : m_route.result.elements)
		if (element.vertex == band.vertex && element.turn != 0)
			turn = element.turn;
	auto const start{View.y_of(turn / std::max(1.0, band.value)) + (View.mouse.y - band.press.y)};
	auto const k{(View.axis - start) * View.flip * turn / std::max(1e-6f, View.curvature)};
	return rounded_radius(1.0 / std::max(1e-5, static_cast<double>(k)));
}

bool editor_mode::route_band_wheel(route_band_view const &View)
{
	if (View.part == 0)
		return false;
	ImGui::SetMouseCursor(View.part == 1 ? ImGuiMouseCursor_ResizeNS : ImGuiMouseCursor_ResizeEW);
	auto const wheel{ImGui::GetIO().MouseWheel};
	if (wheel == 0.0f)
		return false;
	auto &value{band_value(m_route.design.vertices[m_route.result.elements[View.under].vertex], View.part, View.arc)};
	if (View.part == 1)
		value = rounded_radius(value * (wheel > 0.0f ? 0.95 : 1.0 / 0.95));
	else
		value = std::max(0.0, std::round(value / 5.0) * 5.0 + (wheel > 0.0f ? 5.0 : -5.0));
	return true;
}

void editor_mode::draw_route_band(route_band_view const &View, ImDrawList *List) const
{
	auto const &route{m_route};
	auto const &result{route.result};
	auto const disabled{ImGui::GetColorU32(ImGuiCol_TextDisabled)};
	List->AddLine(ImVec2(View.inner_left, View.axis), ImVec2(View.inner_left + View.inner_width, View.axis), ImGui::GetColorU32(ImGuiCol_TextDisabled, 0.5f));
	auto hot_vertex{-1};
	if (View.dragging)
		hot_vertex = route.band.vertex;
	else if (View.under >= 0)
		hot_vertex = result.elements[View.under].vertex;
	for (int i = 0; i < static_cast<int>(result.elements.size()); ++i)
	{
		auto const &element{result.elements[i]};
		auto const [from, to]{signed_ends(element)};
		ImVec2 const a{View.x_of(element.chainage), View.y_of(from)};
		ImVec2 const b{View.x_of(element.chainage + element.length), View.y_of(to)};
		auto const lit{element.vertex >= 0 && element.vertex == hot_vertex};
		auto const tint{element_colour(element.kind)};
		if (element.kind != alignment::element_kind::straight)
			List->AddQuadFilled(ImVec2(a.x, View.axis), a, b, ImVec2(b.x, View.axis), (tint & ~IM_COL32_A_MASK) | (lit ? IM_COL32(0, 0, 0, 90) : IM_COL32(0, 0, 0, 40)));
		List->AddLine(a, b, tint, lit ? 3.0f : 2.0f);
		if (i > 0)
			List->AddLine(ImVec2(a.x, View.axis - 3.0f), ImVec2(a.x, View.axis + 3.0f), disabled);
	}
}

// the labels: the radius over its plateau, the transition lengths along the axis, the km of the ends at the bottom
void editor_mode::draw_route_band_labels(route_band_view const &View, ImDrawList *List) const
{
	auto const &route{m_route};
	auto const &result{route.result};
	auto const disabled{ImGui::GetColorU32(ImGuiCol_TextDisabled)};
	std::vector<std::pair<float, float>> taken;
	auto const start_km{format("km %s", profile::format_chainage(route.origin).c_str())};
	auto const end_km{format("km %s", profile::format_chainage(route.origin + result.length).c_str())};
	auto const label_y{View.label_y()};
	auto const start_size{ImGui::CalcTextSize(start_km.c_str())};
	auto const end_size{ImGui::CalcTextSize(end_km.c_str())};
	place_label(taken, View.inner_left, View.inner_left + start_size.x);
	List->AddText(ImVec2(View.inner_left, label_y), disabled, start_km.c_str());
	if (place_label(taken, View.inner_left + View.inner_width - end_size.x, View.inner_left + View.inner_width))
		List->AddText(ImVec2(View.inner_left + View.inner_width - end_size.x, label_y), disabled, end_km.c_str());
	for (auto const &element : result.elements)
	{
		if (View.dragging && element.vertex == route.band.vertex)
			continue;
		if (element.kind == alignment::element_kind::arc || (element.kind == alignment::element_kind::spiral && element.curvature_start == element.curvature_end && element.radius > 0.0))
			draw_band_radius(View, List, element);
		else if (element.kind == alignment::element_kind::transition_in || element.kind == alignment::element_kind::transition_out)
			draw_band_length(View, List, element, taken);
	}
}

void editor_mode::draw_band_radius(route_band_view const &View, ImDrawList *List, alignment::element const &Element) const
{
	auto const &design{m_route.design};
	auto const from{View.x_of(Element.chainage)};
	auto const to{View.x_of(Element.chainage + Element.length)};
	auto const middle{(from + to) * 0.5f};
	auto const &vertex{design.vertices[std::clamp(Element.vertex, 0, static_cast<int>(design.vertices.size()) - 1)]};
	auto text{format("R %.0f", Element.radius)};
	if (vertex.cant > 0.0)
	{
		auto const longer{text + format(" \xc2\xb7 %.0f mm", vertex.cant)};
		if (ImGui::CalcTextSize(longer.c_str()).x <= to - from)
			text = longer;
	}
	auto const size{ImGui::CalcTextSize(text.c_str())};
	auto const y{signed_ends(Element).first * View.flip >= 0.0 ? View.y_of(signed_ends(Element).first) - size.y - 1.0f : View.y_of(signed_ends(Element).first) + 1.0f};
	if (size.x <= std::max(to - from, size.x * 0.8f))
		List->AddText(ImVec2(std::round(middle - size.x * 0.5f), std::round(std::max(View.top.y + 1.0f, y))), element_colour(Element.kind), text.c_str());
}

void editor_mode::draw_band_length(route_band_view const &View, ImDrawList *List, alignment::element const &Element, std::vector<std::pair<float, float>> &Taken)
{
	auto const from{View.x_of(Element.chainage)};
	auto const to{View.x_of(Element.chainage + Element.length)};
	auto const middle{(from + to) * 0.5f};
	auto const text{format("%.0f m", Element.length)};
	auto const size{ImGui::CalcTextSize(text.c_str())};
	if (View.both)
	{
		auto const y{signed_ends(Element).first + signed_ends(Element).second > 0.0 ? View.axis + 2.0f : View.axis - size.y - 2.0f};
		if (size.x <= to - from + 8.0f)
			List->AddText(ImVec2(std::round(middle - size.x * 0.5f), std::round(y)), element_colour(Element.kind), text.c_str());
	}
	else if (place_label(Taken, middle - size.x * 0.5f, middle + size.x * 0.5f))
		List->AddText(ImVec2(std::round(middle - size.x * 0.5f), View.label_y()), element_colour(Element.kind), text.c_str());
}

void editor_mode::draw_route_band_value(route_band_view const &View, ImDrawList *List) const
{
	auto const &band{m_route.band};
	auto const value{band_value(m_route.design.vertices[band.vertex], band.part, band.arc)};
	auto const text{band.part == 1 ? format("R %.0f m", value) : format("%.0f m", value)};
	auto const size{ImGui::CalcTextSize(text.c_str())};
	ImVec2 const at{std::clamp(View.mouse.x + View.pad, View.top.x, View.bottom.x - size.x - View.pad), std::clamp(View.mouse.y - size.y - View.pad, View.top.y, View.bottom.y - size.y)};
	List->AddRectFilled(ImVec2(at.x - 3.0f, at.y - 1.0f), ImVec2(at.x + size.x + 3.0f, at.y + size.y + 1.0f), IM_COL32(0, 0, 0, 170), 3.0f);
	List->AddText(at, IM_COL32(255, 255, 255, 255), text.c_str());
}

void editor_mode::route_band_tooltip(route_band_view const &View, std::vector<int> const &Arcs) const
{
	auto const part{View.part};
	auto const arc{View.arc};
	auto const index{m_route.result.elements[View.under].vertex};
	auto const &vertex{m_route.design.vertices[index]};
	auto const value{band_value(vertex, part, arc)};
	ImGui::BeginTooltip();
	if (part == 1 && Arcs[index] > 1)
		ImGui::Text(STR_C("Curve %d, arc R%d: %.0f m"), index + 1, arc + 1, value);
	else if (part == 1)
		ImGui::Text(STR_C("Curve %d: R %.0f m, cant %.0f mm"), index + 1, value, vertex.cant);
	else if (part == 4)
		ImGui::Text(STR_C("Curve %d, transition between R%d and R%d: %.0f m"), index + 1, arc, arc + 1, value);
	else if (part == 2)
		ImGui::Text(STR_C("Transition into curve %d: %.0f m"), index + 1, value);
	else
		ImGui::Text(STR_C("Transition out of curve %d: %.0f m"), index + 1, value);
	gesture_rows({{"Drag", part == 1 ? "up or down: the radius" : "sideways: the length"}, {"Wheel", part == 1 ? "the radius by 5%" : "the length by 5 m"}});
	ImGui::EndTooltip();
}

bool editor_mode::render_route_parameters()
{
	auto &design{m_route.design};
	bool changed{false};
	ImGui::TextDisabled("%s", STR_C("Design speed (km/h)"));
	ImGui::SetNextItemWidth(-FLT_MIN);
	changed |= ImGui::InputDouble("##designspeed", &design.speed, 10.0, 20.0, "%.0f");
	item_tooltip("Speed the cant, the transitions and the recommended radius are designed for");
	changed |= value_chips("designspeeds", design.speed, {60.0, 80.0, 100.0, 120.0, 160.0, 200.0}, "%.0f");
	design.speed = std::max(1.0, design.speed);
	ImGui::TextDisabled("%s", STR_C("Transition curve"));
	if (int shape = static_cast<int>(design.shape); segmented("transitionshape", shape,
	              {{static_cast<int>(alignment::transition_shape::cubic_parabola), STR_C("Cubic parabola"), "The usual one on the Polish lines"},
	               {static_cast<int>(alignment::transition_shape::clothoid), STR_C("Clothoid"), "Curvature growing linearly with the length"}}))
	{
		design.shape = static_cast<alignment::transition_shape>(shape);
		changed = true;
	}

	if (ImGui::TreeNode(STR_C("Norms")))
	{
		auto &norms{design.norms};
		if (begin_fields("##routelimits", 0.6f))
		{
			auto const limit = [&](char const *Label, char const *Id, double &Value, char const *Format) {
				field_row(Label);
				changed |= ImGui::InputDouble(Id, &Value, 0.0, 0.0, Format);
			};
			limit("Rail axes distance (mm)", "##gauge", norms.gauge, "%.0f");
			limit("Unbalanced acceleration (m/s2)", "##unbalanced", norms.unbalanced, "%.2f");
			limit("Maximum cant (mm)", "##cantmax", norms.cant_max, "%.0f");
			limit("Cant change rate (mm/s)", "##cantrate", norms.cant_rate, "%.0f");
			limit("Cant ramp 1:(k*V), k", "##ramp", norms.ramp_factor, "%.1f");
			limit("Unbalanced acc. change rate (m/s3)", "##jerk", norms.jerk, "%.2f");
			limit("Straight between curves (s of travel)", "##tangent", norms.tangent_min_time, "%.1f");
			ImGui::EndTable();
		}
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
	m_route.focus = {};
	route_measure_clicks();
	if (false == m_route.clicks.empty())
	{
		ImGui::Checkbox(STR_C("Keep to the points clicked"), &m_route.hold);
		item_tooltip("The scenery keeps the points clicked when the line was laid: a curve whose transitions or cants are changed gets its radii fitted to them again, so it stays where it was seen on the photo");
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Fit all to them")))
		{
			for (int i = 0; i < count; ++i)
				changed |= route_hold_clicks(i);
		}
		item_tooltip("The radii of every curve fitted again to the points clicked along it, the transitions and the cants as they are");
	}
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
	bool changed{false};
	// a transition or a cant changed, the edit done: the curve fitted to the points clicked again, its radii given by them
	bool settled{false};
	using part = route_design::curve_focus::part;
	ImGui::BeginGroup();
	// named as the vertex is in the view, W1, W2..., with where it lies along the line
	changed |= route_vertex_heading(Index);
	// two fields in a row, the name and the unit inside them, a bar in the colour of the element in the band and in the view
	auto const half{std::floor((ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f)};
	changed |= route_vertex_curved_ends(Index);
	changed |= route_vertex_fields(Index, half, settled);
	ImGui::PushItemWidth(110.0f);
	changed |= render_vertex_kind(vertex);
	if (vertex.compound)
		changed |= route_vertex_arcs(Index, settled);
	ImGui::PopItemWidth();
	vertex.radius = std::max(1.0, vertex.radius);
	vertex.transition_in = std::max(0.0, vertex.transition_in);
	vertex.transition_out = std::max(0.0, vertex.transition_out);
	vertex.cant = std::max(0.0, vertex.cant);
	changed |= route_vertex_reshape(Index);
	changed |= route_vertex_ramps(Index, half);
	changed |= route_vertex_details(Index);
	ImGui::EndGroup();
	if (route.focus.vertex != Index && false == route.focus.held && ImGui::IsItemHovered())
		route.focus = {Index, part::whole, 0, false};
	if (settled && route.hold)
		changed |= route_hold_clicks(Index);
	return changed;
}

void editor_mode::route_mark(int const Index, route_design::curve_focus::part const What, int const Arc)
{
	if (ImGui::IsItemActive())
		m_route.focus = {Index, What, Arc, true};
	else if (false == m_route.focus.held && ImGui::IsItemHovered())
		m_route.focus = {Index, What, Arc, false};
}

bool editor_mode::route_vertex_heading(int const Index)
{
	using part = route_design::curve_focus::part;
	auto const &route{m_route};
	auto from{-1.0};
	auto to{-1.0};
	for (auto const &e : route.result.elements)
		if (e.vertex == Index)
		{
			from = from < 0.0 ? e.chainage : from;
			to = e.chainage + e.length;
		}
	auto title{format(STR_C("Curve W%d"), Index + 1)};
	if (from >= 0.0)
		title += format("   km %s - %s", profile::format_chainage(route.origin + from).c_str(), profile::format_chainage(route.origin + to).c_str());
	// the vertex picked in the view stands out
	auto const picked{route.vertex == Index || route.grip == Index};
	if (picked)
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(overlay_color::highlight));
	ImGui::SeparatorText(title.c_str());
	if (picked)
		ImGui::PopStyleColor();
	route_mark(Index, part::whole, 0);
	if (from >= 0.0)
	{
		if (ImGui::SmallButton(STR_C("Show in the view")))
			route_show_curve(from, to);
		item_tooltip("The camera flies over the curve, which is lit up in the view while the mouse is over its fields");
		route_mark(Index, part::whole, 0);
	}
	return route_vertex_clicks(Index);
}

// how far the curve is from the points clicked along it when it was laid
bool editor_mode::route_vertex_clicks(int const Index)
{
	using part = route_design::curve_focus::part;
	std::size_t clicked{0};
	double sum{0.0};
	double worst{0.0};
	for (auto const &state : m_route.click_states)
		if (state.vertex == Index && false == state.corner && std::abs(state.offset) <= 3.0)
		{
			++clicked;
			sum += state.offset * state.offset;
			worst = std::max(worst, std::abs(state.offset));
		}
	if (clicked == 0)
		return false;
	ImGui::SameLine();
	ImVec4 colour{1.0f, 0.35f, 0.27f, 1.0f};
	if (worst <= 0.1)
		colour = ImVec4(0.35f, 0.86f, 0.43f, 1.0f);
	else if (worst <= 0.3)
		colour = ImVec4(1.0f, 0.78f, 0.24f, 1.0f);
	ImGui::TextColored(colour, STR_C("points clicked %zu: %.0f cm on average, at most %.0f cm"), clicked, std::sqrt(sum / static_cast<double>(clicked)) * 100.0, worst * 100.0);
	item_tooltip("The points clicked along the curve when it was laid, kept in the scenery, and how far the curve is from them now");
	route_mark(Index, part::whole, 0);
	if (clicked < 3)
		return false;
	ImGui::SameLine();
	bool changed{false};
	if (ImGui::SmallButton(STR_C("Fit to them")))
		changed = route_hold_clicks(Index);
	item_tooltip("The radii of the curve and their shares fitted again to the points clicked, the transitions and the cants as they are");
	return changed;
}

// the first curve, or the last one, may go on from the curve of the track adjoining at that end: it begins right there, its radius
// the one which begins it there, its entry, or exit, the transition between the two curves
bool editor_mode::route_vertex_curved_ends(int const Index)
{
	auto &design{m_route.design};
	bool changed{false};
	auto const count{static_cast<int>(design.vertices.size())};
	for (auto const atend : {false, true})
	{
		auto const radius{atend ? design.end_radius : design.start_radius};
		if ((atend ? Index + 1 != count : Index != 0) || radius <= 0.0 || radius >= 10000.0 || (atend && count == 1 && design.start_curved))
			continue;
		auto &curved{atend ? design.end_curved : design.start_curved};
		changed |= ImGui::Checkbox((format(atend ? STR_C("Goes into the curve of the track at the end (R %.0f m)") : STR_C("Goes on from the curve of the track at the start (R %.0f m)"), radius) + (atend ? "##curvedend" : "##curvedstart")).c_str(), &curved);
		item_tooltip("The curve begins right at the end of the fragment, at the curvature of the track adjoining there, where it turns the same way; its radius is the one "
		             "which begins it there, the transition at that end goes between the two curves");
	}
	return changed;
}

bool editor_mode::route_vertex_fields(int const Index, float const Half, bool &Settled)
{
	using part = route_design::curve_focus::part;
	auto &vertex{m_route.design.vertices[Index]};
	auto const field = [this, Index, Half, &Settled](char const *Id, double &Value, float const Speed, char const *Format, ImU32 const Colour, char const *Tooltip, part const What) {
		ImGui::SetNextItemWidth(Half);
		auto const edited{ImGui::DragScalar(Id, ImGuiDataType_Double, &Value, Speed, nullptr, nullptr, Format)};
		// the radius is the user's own, the curve no longer fitted after it
		Settled |= What != part::radius_arc && ImGui::IsItemDeactivatedAfterEdit();
		route_mark(Index, What, 0);
		auto const low{ImGui::GetItemRectMin()};
		auto const high{ImGui::GetItemRectMax()};
		ImGui::GetWindowDrawList()->AddRectFilled(low, ImVec2(low.x + 3.0f, high.y), Colour, 2.0f, ImDrawFlags_RoundCornersLeft);
		item_tooltip(Tooltip);
		return edited;
	};
	ImU32 const transition{IM_COL32(255, 140, 30, 255)};
	bool changed{false};
	changed |= field("##radius", vertex.radius, 5.0f, vertex.compound ? STR_C("R1 %.0f m") : STR_C("R %.0f m"), overlay_color::grip, "Radius of the arc: drag sideways, or a double click to type it", part::radius_arc);
	ImGui::SameLine();
	changed |= field("##cant", vertex.cant, 1.0f, STR_C("cant %.0f mm"), IM_COL32(240, 170, 60, 255),
	                 vertex.compound ? "Height of the outer rail above the inner one on the sharpest arc; the arcs with no cant of their own get it in proportion to their curvature"
	                                 : "Height of the outer rail above the inner one",
	                 part::whole);
	changed |= field("##entry", vertex.transition_in, 1.0f, STR_C("entry %.0f m"), transition, "Length of the transition curve leading into the arc", part::entry);
	ImGui::SameLine();
	changed |= field("##exit", vertex.transition_out, 1.0f, STR_C("exit %.0f m"), transition, "Length of the transition curve leading out of the arc", part::exit);
	return changed;
}

bool editor_mode::route_vertex_arcs(int const Index, bool &Settled)
{
	using part = route_design::curve_focus::part;
	auto &vertex{m_route.design.vertices[Index]};
	bool changed{false};
	auto total{std::max(0.0, vertex.share)};
	for (auto const &arc : vertex.arcs)
		total += std::max(0.0, arc.share);
	auto const cants{alignment::compound_cants(vertex)};
	changed |= ImGui::DragScalar(format(STR_C("Share of R%d"), 1).c_str(), ImGuiDataType_Double, &vertex.share, 0.01f, nullptr, nullptr, "%.2f");
	route_mark(Index, part::radius_arc, 0);
	ImGui::SameLine();
	ImGui::TextDisabled("%.0f%%", share_percent(vertex.share, total));
	changed |= route_arc_cant(Index, 0, vertex.first_cant, cants, Settled);
	int removed{-1};
	for (int k = 0; k < static_cast<int>(vertex.arcs.size()); ++k)
		changed |= route_vertex_arc(Index, k, total, cants, Settled, removed);
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
		vertex.arcs.emplace_back(std::max(10.0, std::round(last * 0.8 / 10.0) * 10.0), 0.0, 1.0);
		changed = true;
	}
	item_tooltip("Another arc after the last one, the angle of the curve is shared out by the shares");
	ImGui::Separator();
	return changed;
}

bool editor_mode::route_vertex_arc(int const Index, int const K, double const Total, std::vector<double> const &Cants, bool &Settled, int &Removed)
{
	using part = route_design::curve_focus::part;
	auto &arc{m_route.design.vertices[Index].arcs[K]};
	bool changed{false};
	ImGui::PushID(K);
	ImGui::Separator();
	changed |= ImGui::DragScalar(format(STR_C("Radius R%d (m)"), K + 2).c_str(), ImGuiDataType_Double, &arc.radius, 5.0f, nullptr, nullptr, "%.0f");
	route_mark(Index, part::radius_arc, K + 1);
	ImGui::SameLine();
	if (ImGui::SmallButton("x"))
		Removed = K;
	item_tooltip("Takes this radius out of the curve");
	route_mark(Index, part::radius_arc, K + 1);
	changed |= ImGui::DragScalar(format(STR_C("Transition R%d-R%d (m)"), K + 1, K + 2).c_str(), ImGuiDataType_Double, &arc.transition, 1.0f, nullptr, nullptr, "%.0f");
	Settled |= ImGui::IsItemDeactivatedAfterEdit();
	route_mark(Index, part::between, K + 1);
	changed |= ImGui::DragScalar(format(STR_C("Share of R%d"), K + 2).c_str(), ImGuiDataType_Double, &arc.share, 0.01f, nullptr, nullptr, "%.2f");
	route_mark(Index, part::radius_arc, K + 1);
	ImGui::SameLine();
	ImGui::TextDisabled("%.0f%%", share_percent(arc.share, Total));
	changed |= route_arc_cant(Index, K + 1, arc.cant, Cants, Settled);
	arc.radius = std::max(1.0, arc.radius);
	arc.transition = std::max(0.0, arc.transition);
	arc.share = std::max(0.0, arc.share);
	ImGui::PopID();
	return changed;
}

// the cant of one arc: the one in proportion to the curvature until it's given its own
bool editor_mode::route_arc_cant(int const Index, int const Arc, double &Cant, std::vector<double> const &Cants, bool &Settled)
{
	using part = route_design::curve_focus::part;
	auto value{static_cast<std::size_t>(Arc) < Cants.size() ? Cants[Arc] : std::max(0.0, Cant)};
	auto edited{ImGui::DragScalar(format(STR_C("Cant of R%d (mm)"), Arc + 1).c_str(), ImGuiDataType_Double, &value, 1.0f, nullptr, nullptr, "%.0f")};
	Settled |= ImGui::IsItemDeactivatedAfterEdit();
	route_mark(Index, part::radius_arc, Arc);
	if (edited)
		Cant = std::max(0.0, value);
	item_tooltip("The cant of this arc, ramped along the transition curves to the arcs either side of it; until it's set here, the cant of the curve in proportion to the curvature of the arc");
	ImGui::SameLine();
	if (Cant < 0.0)
		ImGui::TextDisabled("%s", STR_C("auto"));
	else if (ImGui::SmallButton(STR_C("auto")))
	{
		Cant = -1.0;
		edited = true;
		Settled = true;
	}
	item_tooltip("The cant of the arc in proportion to its curvature again");
	return edited;
}

bool editor_mode::route_vertex_reshape(int const Index)
{
	using part = route_design::curve_focus::part;
	auto &route{m_route};
	auto &design{route.design};
	auto &vertex{design.vertices[Index]};
	bool changed{false};
	// the transitions follow the radii, which follow the points clicked: a few rounds settle them
	auto const reshape = [this, Index, &route, &changed](auto const &Shape) {
		for (int pass = 0; pass < 3; ++pass)
		{
			Shape();
			if (false == route.hold || false == route_hold_clicks(Index))
				break;
		}
		changed = true;
	};
	if (ImGui::SmallButton(STR_C("Apply recommended for V")))
		reshape([this, &vertex] { route_recommend(vertex); });
	if (ImGui::IsItemHovered())
	{
		auto const recommended{alignment::recommend(design.speed, vertex.radius, design.norms)};
		ImGui::SetTooltip("%s\n%s", STR_C("Cant and transitions recommended for the design speed and the radius"),
		                  format(STR_C("For %.0f km/h: R min %.0f m, cant %.0f mm (equilibrium %.0f, min %.0f), transition %.0f m"), design.speed, recommended.radius_min, recommended.cant, recommended.cant_equilibrium, recommended.cant_min, recommended.transition).c_str());
	}
	if (false == vertex.spirals)
	{
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Transitions for these cants")))
			reshape([&vertex, &design] { alignment::norm_transitions(vertex, design.speed, design.norms); });
		item_tooltip("The transition curves as long as the norms ask for at these radii and cants, the cants staying: a cant lowered asks for longer ones");
		route_mark(Index, part::whole, 0);
	}
	return changed;
}

// the cant ramps at the ends of the curve: along its transitions, or where they're put, off a switch laid on a transition say
bool editor_mode::route_vertex_ramps(int const Index, float const Half)
{
	if (m_route.design.vertices[Index].spirals || false == ImGui::TreeNode(STR_C("Cant ramps")))
		return false;
	std::vector<alignment::result::cant_ramp> ramps;
	for (auto const &ramp : m_route.result.ramps)
		if (ramp.vertex == Index)
			ramps.push_back(ramp);
	bool changed{route_cant_ramp(Index, false, ramps.empty() ? nullptr : &ramps[0], Half)};
	changed |= route_cant_ramp(Index, true, ramps.size() > 1 ? &ramps[1] : nullptr, Half);
	ImGui::TreePop();
	return changed;
}

bool editor_mode::route_cant_ramp(int const Index, bool const Out, alignment::result::cant_ramp const *Ramp, float const Half)
{
	using part = route_design::curve_focus::part;
	auto &vertex{m_route.design.vertices[Index]};
	auto &offset{Out ? vertex.ramp_out_offset : vertex.ramp_in_offset};
	auto &length{Out ? vertex.ramp_out_length : vertex.ramp_in_length};
	auto const what{Out ? part::exit : part::entry};
	bool changed{false};
	ImGui::PushID(Out ? "out" : "in");
	auto along{length < 0.0};
	if (ImGui::Checkbox(Out ? STR_C("Ramp out along the transition curve") : STR_C("Ramp in along the transition curve"), &along))
	{
		// let go of the transition where it lay along it, to be moved from there
		if (along)
			length = -1.0;
		else if (Ramp != nullptr)
			length = Ramp->to - Ramp->from;
		else
			length = 0.0;
		offset = 0.0;
		changed = true;
	}
	route_mark(Index, what, 0);
	item_tooltip("The norms recommend the cant ramp along the transition curve; off it, it may lie on the straight before the curve or on the arc, a switch on the transition keeping the same cant all along");
	if (false == along)
	{
		ImGui::SetNextItemWidth(Half);
		changed |= ImGui::DragScalar("##offset", ImGuiDataType_Double, &offset, 0.5f, nullptr, nullptr, STR_C("into the curve %.1f m"));
		route_mark(Index, what, 0);
		item_tooltip("How far the outer end of the ramp lies into the curve from its end, below 0 out along the straight before it");
		ImGui::SameLine();
		ImGui::SetNextItemWidth(Half);
		changed |= ImGui::DragScalar("##length", ImGuiDataType_Double, &length, 0.5f, nullptr, nullptr, STR_C("length %.1f m"));
		route_mark(Index, what, 0);
		item_tooltip("The length of the ramp, the cant rising evenly along it");
		length = std::max(0.0, length);
	}
	if (Ramp != nullptr && Ramp->to > Ramp->from)
		ImGui::TextDisabled(STR_C("km %s - %s, %.2f mm/m"), profile::format_chainage(m_route.origin + Ramp->from).c_str(), profile::format_chainage(m_route.origin + Ramp->to).c_str(), Ramp->cant / (Ramp->to - Ramp->from));
	ImGui::PopID();
	return changed;
}

bool editor_mode::route_vertex_details(int const Index)
{
	if (false == ImGui::TreeNode(STR_C("Details")))
		return false;
	for (auto const &curve : m_route.result.curves)
	{
		if (curve.vertex != Index)
			continue;
		ImGui::Text(STR_C("Deflection %.4f deg, tangents %.2f / %.2f m, arc %.2f m"), glm::degrees(curve.deflection), curve.tangent_in, curve.tangent_out, curve.arc_length);
		ImGui::Text(STR_C("Unbalanced acc. %.3f m/s2, cant rate %.1f / %.1f mm/s, acc. rate %.3f / %.3f m/s3"), curve.unbalanced, curve.cant_rate_in, curve.cant_rate_out, curve.jerk_in, curve.jerk_out);
	}
	auto const changed{ImGui::Checkbox(STR_C("Turn the longer way (over 180 deg)"), &m_route.design.vertices[Index].reverse_turn)};
	item_tooltip("For the loops: the curve turns by more than a half circle");
	ImGui::TreePop();
	return changed;
}

void editor_mode::render_route_result()
{
	auto const &route{m_route};
	auto const &design{route.design};
	auto const &result{route.result};
	if (result.length > 0.0 && (result.start_extension > 0.0 || result.end_extension > 0.0))
		ImGui::Text(STR_C("Takes over %.2f m of the straight at the start, %.2f m at the end"), result.start_extension, result.end_extension);
	for (auto const &error : result.errors)
		ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", error.c_str());
	for (auto const &warning : result.warnings)
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", warning.c_str());

	ImGui::PushID("routeresult");
	if (result.length > 0.0 && ImGui::TreeNode(STR_C("Details")))
	{
		auto const pieces{alignment::minimum_pieces(result, design)};
		ImGui::Text(STR_P("Length %.2f m (now %.2f m), %zu element", "Length %.2f m (now %.2f m), %zu elements", result.elements.size()), result.length, route.chain.length, result.elements.size());
		ImGui::TextDisabled(STR_C("Paths: %zu in the fragment, %zu needed%s"), route.chain.tracks.size(), pieces, pieces > route.chain.tracks.size() ? STR_C(" (copies of the neighbours will be added)") : "");
		ImGui::TreePop();
	}
	ImGui::PopID();
	if (ImGui::TreeNode(STR_C("Elements")))
	{
		ImGui::TextDisabled(STR_C("Straight available beyond the ends: %.2f / %.2f m"), design.start_reserve, design.end_reserve);
		static constexpr std::array<char const *, 5> kinds{"straight", "transition", "arc", "transition", "transition between arcs"};
		for (auto const &e : result.elements)
			ImGui::Text("km %s  %-10s  L %8.2f m%s", profile::format_chainage(m_route.origin + e.chainage).c_str(), STR_C(kinds[static_cast<std::size_t>(e.kind)]), e.length, e.kind == alignment::element_kind::straight ? "" : ("  R " + std::to_string(static_cast<int>(std::round(e.radius))) + " m").c_str());
		ImGui::TreePop();
	}

	if (result.valid)
	{
		if (main_button(STR_C("Apply to the scenery"), ImVec2(-1.0f, 28.0f)))
		{
			route_apply();
			first_step_done(first_step::plan_apply);
		}
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
	if (auto *track{selected_track()}; track != m_straights.current_for)
	{
		m_straights.current_for = track;
		m_straights.handle = -1;
		m_straights.current = track != nullptr ? editor_track::find_straight(*track, m_straights.tolerance) : editor_track::straight{};
		find_neighbour_straights();
	}
	return m_straights.current;
}
