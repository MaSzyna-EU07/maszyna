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
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorGeometry.hpp"

#include "world/Track.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <array>
#include <optional>
#include <tuple>

namespace
{

using geometry::bezier;
using geometry::has_controls;
using geometry::cross;
using geometry::grade_of;
using geometry::plan_distance;
using geometry::plan_of;

struct switch_end_move
{
	glm::dvec3 from{0.0};
	glm::dvec3 to{0.0};
	double shift{0.0};
};

std::optional<std::vector<switch_end_move>> switch_end_moves(TTrack const &Track, segment_data const &Main, bool const Online, std::pair<double, double> const &Room, double const Offset, std::string &Status)
{
	auto const axis{editor_track::main_axis(Track)};
	std::vector<switch_end_move> ends;
	for (auto const index : {segment_data::point::start, segment_data::point::end})
	{
		auto const from{Track.m_paths.front().points[index]};
		auto const to{Main.points[index]};
		if (false == Online || glm::distance(from, to) < 0.005 || axis->lateral(to) > Offset)
			continue;
		auto const target{axis->along(to)};
		if (target < Room.first - 0.01 || target > Room.second + 0.01)
		{
			Status = STR_C("The switch would run into another switch or past the end of the straight");
			return std::nullopt;
		}
		ends.emplace_back(from, to, index == segment_data::point::start ? axis->along(from) - target : target - axis->along(from));
	}
	std::ranges::stable_sort(ends, [](switch_end_move const &A, switch_end_move const &B) { return A.shift > B.shift; });
	return ends;
}

std::string switch_rejoin(TTrack const &Track, std::vector<editor_track::joint> const &Joints, std::vector<bool> const &Handled, editor_track::edit &Edit)
{
	std::vector<editor_track::joint> following;
	std::string loose;
	for (std::size_t i = 0; i < Joints.size(); ++i)
	{
		auto const &joint{Joints[i]};
		if (Handled[i] || joint.other->m_editorremoved)
			continue;
		auto const &theirs{joint.other->m_paths[joint.theirs.path]};
		if (auto const shift{glm::distance(editor_track::point_position(Track, joint.own), editor_track::point_position(*joint.other, joint.theirs))}; shift > 0.3 * bezier{theirs}.plan_length())
		{
			loose += (loose.empty() ? "" : ", ") + name_of(*joint.other);
			continue;
		}
		following.push_back(joint);
		Edit.capture(*joint.other);
	}
	editor_track::follow(Track, following);
	return loose;
}

double constexpr kBranchStraight{500.0}; // how far the straight past the curve next to a switch is followed

// direction out of the path at the end
glm::dvec3 end_outward(segment_data const &Path, bool const Atend)
{
	bezier const curve{Path};
	auto direction{Atend ? curve.first(1.0) : -curve.first(0.0)};
	if (glm::length(plan_of(direction)) < 1e-9)
		direction = Atend ? curve.p3 - curve.p0 : curve.p0 - curve.p3;
	return direction;
}

// straight from the start of the line to its first curve
double opening(alignment::result const &Result)
{
	return Result.elements.empty() || Result.elements.front().kind != alignment::element_kind::straight ? 0.0 : Result.elements.front().length;
}

// straight from the last curve of the line to its end
double closing(alignment::result const &Result)
{
	return Result.elements.empty() || Result.elements.back().kind != alignment::element_kind::straight ? 0.0 : Result.elements.back().length;
}

alignment::vertex scaled(alignment::vertex Vertex, double const Factor)
{
	Vertex.radius *= Factor;
	Vertex.transition_in *= Factor;
	Vertex.transition_out *= Factor;
	for (auto &arc : Vertex.arcs)
	{
		arc.radius *= Factor;
		arc.transition *= Factor;
	}
	return Vertex;
}

// where the path of the index leaves the chain
glm::dvec3 exit_of(editor_track::chain const &Chain, std::size_t const Index)
{
	auto const &path{Chain.tracks[Index]->m_paths.front()};
	return path.points[Chain.forward[Index] ? segment_data::point::end : segment_data::point::start];
}

// raises the pieces by Rise at their start, less and less up to nothing at their end
void lift(std::vector<segment_data> &Pieces, double const Rise)
{
	std::vector<double> lengths;
	double total{0.0};
	for (auto const &piece : Pieces)
	{
		lengths.push_back(bezier{piece}.plan_length());
		total += lengths.back();
	}
	if (total <= 1e-6 || std::abs(Rise) <= 1e-6)
		return;
	auto const slope{Rise / total};
	double along{0.0};
	for (std::size_t i = 0; i < Pieces.size(); ++i)
	{
		auto &points{Pieces[i].points};
		auto const controls{has_controls(Pieces[i])};
		points[segment_data::point::start].y += Rise - slope * along;
		points[segment_data::point::end].y += Rise - slope * (along + lengths[i]);
		// the controls are kept from their ends
		if (controls)
		{
			points[segment_data::point::control1].y -= slope * glm::length(plan_of(points[segment_data::point::control1]));
			points[segment_data::point::control2].y += slope * glm::length(plan_of(points[segment_data::point::control2]));
		}
		along += lengths[i];
	}
}

using axis_spans = std::vector<std::tuple<double, double, bool>>;

// how far along the axis the straight goes on past the far end of a switch, Length along it, in plain tracks one after another
double room_ahead(axis_spans const &Spans, double const Length)
{
	double high{Length};
	for (auto const &[from, to, plain] : Spans)
		if (from > Length - 0.01)
		{
			if (false == plain || from > high + 0.01)
				break;
			high = std::max(high, to);
		}
	return high;
}

// how far back along the axis the straight goes on before the start of a switch, in plain tracks one after another
double room_behind(axis_spans const &Spans)
{
	double low{0.0};
	for (auto span{Spans.rbegin()}; span != Spans.rend(); ++span)
	{
		auto const &[from, to, plain] = *span;
		if (to < 0.01)
		{
			if (false == plain || to < low - 0.01)
				break;
			low = std::min(low, from);
		}
	}
	return low;
}

// the track joined to the Current one at its end ahead, Atstart telling whether that's its start, set then for the next one; none
// where the chain ends there
TTrack *next_along(TTrack *Current, bool &Atstart)
{
	auto *next{Atstart ? Current->trNext : Current->trPrev};
	if (next == nullptr || (next->trPrev != Current && next->trNext != Current))
		return nullptr;
	Atstart = next->trPrev == Current;
	return next;
}

// the plain tracks of a branch from the switch on, a straight up to the curve maybe, the curve, the straight past it
struct branch_walk
{
	std::vector<TTrack *> run;
	int phase{0}; // 1 on the curve, 2 on the straight past it
	std::size_t straight{0}; // the first track of the straight past the curve
};

// the branch from the switch on, from First reached at its start or end, Atstart, over the tracks Regular lets it run on, as far as
// kBranchStraight along the straight past the curve
template <typename Regular>
branch_walk walked_branch(TTrack *First, bool Atstart, Regular const &Is_regular, editor_track::straight_tolerance const &Tolerance)
{
	branch_walk walk;
	auto *current{First};
	double past{0.0};
	while (Is_regular(current) && std::ranges::find(walk.run, current) == walk.run.end() && past < kBranchStraight)
	{
		auto const plain{editor_track::is_straight(*current, Tolerance)};
		if (false == plain && walk.phase == 2)
			break;
		if (false == plain)
			walk.phase = 1;
		else if (walk.phase == 1)
		{
			walk.phase = 2;
			walk.straight = walk.run.size();
		}
		if (walk.phase == 2)
			past += current->Length();
		walk.run.push_back(current);
		current = next_along(current, Atstart);
	}
	return walk;
}

// the largest factor of a curve from 1 up to some 5 with which it still fits, Fits_at telling, found by halving; none where even the
// largest tried does
template <typename Fits>
std::optional<double> largest_fitting(Fits const &Fits_at)
{
	double low{1.0};
	double high{1.5};
	while (Fits_at(high) && high < 5.0)
		high *= 1.5;
	if (Fits_at(high))
		return std::nullopt;
	for (int i = 0; i < 40; ++i)
	{
		auto const middle{0.5 * (low + high)};
		(Fits_at(middle) ? low : high) = middle;
	}
	return low;
}

// the vertex of a design made as compute() made its curve, Made, to fit between the ends: scaled where it's compound, else taking the
// radius and the transitions of the curve. Returns the factor the radius was made smaller by
double made_to_fit(alignment::vertex &Vertex, alignment::curve_report const &Made)
{
	auto const reduced{Made.radius / Vertex.radius};
	if (Vertex.compound)
		Vertex = scaled(Vertex, reduced);
	else
	{
		Vertex.radius = Made.radius;
		Vertex.transition_in = Made.transition_in;
		Vertex.transition_out = Made.transition_out;
	}
	return reduced;
}

// the end of a Design at a switch moved to where the line Laid from the switch starts, Atstart: the start of the design
void end_at_switch(alignment::design &Design, alignment::design const &Laid, bool const Atstart)
{
	if (Atstart)
	{
		Design.start = Laid.start;
		Design.start_direction = Laid.start_direction;
		Design.start_grade = Laid.start_grade;
	}
	else
	{
		Design.end = Laid.start;
		Design.end_direction = -Laid.start_direction;
		Design.end_grade = -Laid.start_grade;
	}
}

// the corner K of the Vertices of a design moves along its other leg to the line Laid from the switch, Corners: where the corners lie;
// false where it can't
bool corner_onto_line(std::vector<alignment::vertex> &Vertices, std::vector<glm::dvec2> const &Corners, std::size_t const K, alignment::design const &Laid, bool const Atstart)
{
	if (Vertices.size() < 2 || Corners.size() != Vertices.size())
		return true;
	auto const corner{Corners[K]};
	auto const leg{Corners[Atstart ? 1 : K - 1] - corner};
	auto const determinant{cross(Laid.start_direction, leg)};
	if (std::abs(determinant) < 1e-9)
		return false;
	auto const offset{cross(corner - plan_of(Laid.start), leg) / determinant};
	if (offset <= 0.0)
		return false;
	Vertices[K].offset = offset;
	return true;
}

// the curve next to the switch takes the Shape it was laid with: scaled by the Factor where either is compound
void shaped_as_laid(alignment::vertex &Vertex, alignment::vertex const &Shape, double const Factor, bool const Atstart)
{
	if (Shape.compound || Vertex.compound)
		Vertex = scaled(Vertex, Factor);
	else
	{
		Vertex.radius = Shape.radius;
		Vertex.transition_in = Atstart ? Shape.transition_in : Shape.transition_out;
		Vertex.transition_out = Atstart ? Shape.transition_out : Shape.transition_in;
	}
}

// the design of the record Index of the store, starting at the switch (Atstart) or ending there, follows the line Laid again from it:
// its end and the corner next to it moved onto the line, the curve there taking the shape it was laid with, the km of the joint the
// paths are laid up to kept. Left as it was where that can't be
template <typename Branch>
void record_follows(Branch const &Switch_branch, std::size_t const Index, bool const Atstart)
{
	auto const &record{alignment_store::records()[Index]};
	auto design{record.design};
	auto &vertices{design.vertices};
	if (vertices.empty())
		return;
	auto const k{Atstart ? std::size_t{0} : vertices.size() - 1};
	end_at_switch(design, Switch_branch.laid, Atstart);
	if (false == corner_onto_line(vertices, record.result.vertices, k, Switch_branch.laid, Atstart))
		return;
	shaped_as_laid(vertices[k], Switch_branch.laid.vertices.front(), Switch_branch.factor, Atstart);
	auto const result{alignment::compute(design)};
	if (false == result.valid || false == alignment_store::follows(result, Switch_branch.part.tracks))
		return;
	auto origin{record.origin};
	if (Atstart)
	{
		auto const before{alignment_store::chainage_on(record.result, Switch_branch.part.end)};
		auto const after{alignment_store::chainage_on(result, Switch_branch.part.end)};
		if (before.has_value() && after.has_value())
			origin += *before - *after;
	}
	auto const layer{record.layer};
	auto const source{record.source}; // NOSONAR: a copy, the record goes with forget() before it's stored again
	auto const fit{record.fit};
	alignment_store::forget(static_cast<int>(Index));
	alignment_store::store(design, result, origin, layer, source, fit);
}

// the Text with More after it, the Separator between them where the text isn't empty
std::string joined(std::string Text, std::string_view const Separator, std::string const &More)
{
	if (false == Text.empty())
		Text += Separator;
	Text += More;
	return Text;
}

// an outer end of a double switch moved along its straight
struct outer_end
{
	TTrack *track{nullptr};
	switch_end_move move;
};

// an outer end of a double switch: its track, where it is, where it goes, whether it's the start of the track
struct outer_place
{
	TTrack *track;
	glm::dvec3 from;
	glm::dvec3 to;
	bool atstart;
};

// the moves of the outer ends of a double switch, the Places, along the Axis of its straight, Room: as far along it as the straight
// lets them go; the farthest first. None where one would run into another switch or past the end of the straight, told in Status; no
// moves where the switch isn't on a straight, Online
std::optional<std::vector<outer_end>> outer_moves(bool const Online, editor_track::straight_axis const &Axis, std::array<outer_place, 2> const &Places, std::pair<double, double> const &Room, double const Offset, std::string &Status)
{
	std::vector<outer_end> ends;
	for (auto const &place : Places)
	{
		if (false == Online || glm::distance(place.from, place.to) < 0.005 || Axis.lateral(place.to) > Offset)
			continue;
		auto const target{Axis.along(place.to)};
		if (target < Room.first - 0.01 || target > Room.second + 0.01)
		{
			Status = STR_C("The switch would run into another switch or past the end of the straight");
			return std::nullopt;
		}
		ends.push_back({place.track, {place.from, place.to, place.atstart ? Axis.along(place.from) - target : target - Axis.along(place.from)}});
	}
	std::ranges::stable_sort(ends, [](outer_end const &A, outer_end const &B) { return A.move.shift > B.move.shift; });
	return ends;
}

// the joints of the two switches of a double switch, where they are, the ones with the set itself handled already
struct pair_joints
{
	std::array<std::vector<editor_track::joint>, 2> joints;
	std::array<std::vector<glm::dvec3>, 2> positions;
	std::array<std::vector<bool>, 2> handled;
};

// the joints of the two Switches of a double switch joined by the Link
pair_joints joints_of_pair(std::array<TTrack *, 2> const &Switches, TTrack const *Link)
{
	pair_joints pair;
	for (std::size_t k = 0; k < 2; ++k)
	{
		pair.joints[k] = editor_track::joints(*Switches[k]);
		for (auto const &joint : pair.joints[k])
		{
			pair.positions[k].push_back(editor_track::point_position(*Switches[k], joint.own));
			pair.handled[k].push_back(joint.other == Link || joint.other == Switches[0] || joint.other == Switches[1]);
		}
	}
	return pair;
}

// what's told of a switch moved: the tracks left unjoined, Loose, and the notes on the curves past it
void moved_status(std::string &Status, std::string const &Loose, std::string const &Notes)
{
	if (false == Loose.empty())
		Status = STR_C("Too far from the new ends of the switch, left unjoined: ") + Loose;
	if (false == Notes.empty())
		Status = joined(Status, "\n", Notes);
}

// the joints of the two switches at the Point, by where they were, marked as handled
void handled_at(std::array<std::vector<glm::dvec3>, 2> const &Positions, glm::dvec3 const &Point, std::array<std::vector<bool>, 2> &Handled)
{
	for (std::size_t k = 0; k < 2; ++k)
		for (std::size_t i = 0; i < Positions[k].size(); ++i)
			if (glm::distance(Positions[k][i], Point) < geometry::same_point)
				Handled[k][i] = true;
}

// the joints of a switch on its Path with the Other track marked as handled
void handled_with(std::vector<editor_track::joint> const &Joints, int const Path, TTrack const *Other, std::vector<bool> &Handled)
{
	for (std::size_t i = 0; i < Joints.size(); ++i)
		if (Joints[i].own.path == Path && Joints[i].other == Other)
			Handled[i] = true;
}

// the other end of the path than Point
editor_track::point_ref far_end(editor_track::point_ref const &Point)
{
	return {Point.path, Point.kind == editor_track::point_kind::start ? editor_track::point_kind::end : editor_track::point_kind::start};
}

// whether the Track is a plain one whose end away from Point is free
bool free_beyond(TTrack const &Track, editor_track::point_ref const &Point)
{
	return Track.eType == tt_Normal && Track.m_paths.size() == 1 && editor_track::connected_points(Track, editor_track::point_position(Track, far_end(Point))).empty();
}

// whether the Track is a plain straight along the Shift
bool straight_along(TTrack const &Track, glm::dvec3 const &Shift)
{
	return Track.eType == tt_Normal && Track.m_paths.size() == 1 && false == has_controls(Track.m_paths.front()) &&
	       std::abs(geometry::cross(glm::normalize(plan_of(Track.m_paths.front().points[segment_data::point::end] - Track.m_paths.front().points[segment_data::point::start])), plan_of(Shift))) < 1e-3;
}

// the tracks the crossing track of a slip of the Parts goes on into at its End, each with its far end free, added to the Movers: they
// go along with the slip. False where one is joined on beyond it, told in Reason
bool crossing_movers(std::vector<TTrack *> const &Parts, glm::dvec3 const &End, std::vector<TTrack *> &Movers, std::string &Reason)
{
	for (auto const *part : Parts)
		for (auto const &[other, point] : editor_track::connected_points(*part, End))
		{
			if (std::ranges::find(Parts, other) != Parts.end() || std::ranges::find(Movers, other) != Movers.end())
				continue;
			if (false == free_beyond(*other, point))
			{
				Reason = STR_C("The crossing track of the slip is joined on beyond it: the slip stays where the two tracks cross");
				return false;
			}
			Movers.push_back(other);
		}
	return true;
}

// the straights of the first track of a slip of the Parts out from its End to the joints of its template, Extension long as the
// ones of its crossing track going along, added to the Movers: the Joint at the end goes with them
void straights_along(std::vector<TTrack *> const &Parts, glm::dvec3 const &End, double const Extension, glm::dvec3 const &Shift, std::vector<TTrack *> &Movers, glm::dvec3 &Joint)
{
	for (auto const *part : Parts)
		for (auto const &[other, point] : editor_track::connected_points(*part, End))
			if (std::ranges::find(Parts, other) == Parts.end() && straight_along(*other, Shift) && std::abs(geometry::chord_length(other->m_paths.front()) - Extension) < 0.01 &&
			    std::ranges::find(Movers, other) == Movers.end())
			{
				Movers.push_back(other);
				Joint = editor_track::point_position(*other, far_end(point));
			}
}

// the first track joined to the Movers at the Joint which isn't one of them, with its end there; none where there's none
std::optional<std::pair<TTrack *, editor_track::point_ref>> track_beyond(std::vector<TTrack *> const &Movers, glm::dvec3 const &Joint)
{
	for (auto const *track : Movers)
		for (auto const &[other, point] : editor_track::connected_points(*track, Joint))
			if (std::ranges::find(Movers, other) == Movers.end())
				return std::pair{other, point};
	return std::nullopt;
}

// the ends along the Axis of the tracks of the straights of the Members of a switch which aren't members themselves
std::vector<double> straight_joints(std::vector<TTrack *> const &Members, editor_track::straight_axis const &Axis, editor_track::straight_tolerance const &Tolerance)
{
	std::vector<double> joints;
	for (auto *member : Members)
	{
		if (member->eType != tt_Switch)
			continue;
		auto const line{editor_track::find_straight(*member, Tolerance)};
		for (std::size_t i = 0; i < line.tracks.size(); ++i)
		{
			if (std::ranges::find(Members, line.tracks[i]) != Members.end())
				continue;
			auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
			for (auto const index : {segment_data::point::start, segment_data::point::end})
				joints.push_back(Axis.along(path.points[index]));
		}
	}
	return joints;
}

// the paths of the Movers moved by the Shift
std::vector<segment_data> moved_paths(std::vector<TTrack *> const &Movers, glm::dvec3 const &Shift)
{
	std::vector<segment_data> paths;
	for (auto const *mover : Movers)
		for (auto path : mover->m_paths)
		{
			path.translate(Shift);
			paths.push_back(path);
		}
	return paths;
}

// the Paths moved by the Shift
std::vector<segment_data> shifted(std::vector<segment_data> Paths, glm::dvec3 const &Shift)
{
	for (auto &path : Paths)
		path.translate(Shift);
	return Paths;
}

// a path drawn on the screen as a line through 25 points of it
void draw_path(screen_projection const &Projection, ImDrawList *Drawlist, segment_data const &Path, ImU32 const Color, float const Width)
{
	bezier const curve{Path};
	auto previous{curve.point(0.0)};
	for (int i = 1; i <= 24; ++i)
	{
		auto const next{curve.point(i / 24.0)};
		Projection.line(Drawlist, previous, next, Color, Width);
		previous = next;
	}
}

// how far a switch is slid, by its Start, with the km there
void draw_slide_label(screen_projection const &Projection, ImDrawList *Drawlist, glm::dvec3 const &Start, double const Distance)
{
	ImVec2 at;
	if (false == Projection.project(Start, at))
		return;
	auto const km{alignment_store::chainage_at(Start)};
	auto const label{km.has_value() ? format("%+.2f m  km %.3f", Distance, *km / 1000.0) : format("%+.2f m", Distance)};
	auto const size{ImGui::CalcTextSize(label.c_str())};
	Drawlist->AddRectFilled(ImVec2(at.x + 10.0f, at.y - 8.0f), ImVec2(at.x + 18.0f + size.x, at.y + 8.0f), IM_COL32(0, 0, 0, 180), 3.0f);
	Drawlist->AddText(ImVec2(at.x + 14.0f, at.y - 8.0f), overlay_color::highlight, label.c_str());
}

} // namespace

bool editor_mode::switch_room(TTrack &Track, double &Low, double &High, std::string &Reason) const
{
	Low = 0.0;
	High = 0.0;
	if (Track.eType != tt_Switch || Track.m_paths.size() < 2 || has_controls(Track.m_paths.front()))
		return false;
	auto const axis{editor_track::main_axis(Track)};
	if (false == axis.has_value())
		return false;
	auto const line{editor_track::find_straight(Track, m_straights.tolerance)};
	if (false == tracks_editable(line.tracks, Reason))
		return false;
	axis_spans spans;
	for (std::size_t i = 0; i < line.tracks.size(); ++i)
	{
		if (line.tracks[i] == &Track)
			continue;
		auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
		auto const a{axis->along(path.points[segment_data::point::start])};
		auto const b{axis->along(path.points[segment_data::point::end])};
		spans.emplace_back(std::min(a, b), std::max(a, b), line.tracks[i]->eType == tt_Normal);
	}
	std::ranges::sort(spans);
	High = room_ahead(spans, axis->length);
	Low = room_behind(spans);
	return true;
}

bool editor_mode::switch_vacate(TTrack const &Track, glm::dvec3 const &From, glm::dvec3 const &To, double const Shift, editor_track::straight const &Line, editor_track::edit &Edit, std::string &Status)
{
	if (Shift > 0.0)
	{
		auto const a{Line.along(From)};
		if (auto const b{Line.along(To)}; cut_straight(Line, std::min(a, b), std::max(a, b), Edit.states, Edit.created, Edit.removed, nullptr))
			return true;
		Status = STR_C("The switch would overlap another switch, move it onto plain track");
		return false;
	}
	TTrack const *style{nullptr};
	bool filler{false};
	for (auto const &[other, point] : editor_track::connected_points(Track, From))
	{
		if (other->m_editorremoved)
			continue;
		if (other->eType == tt_Normal && std::ranges::find(Line.tracks, other) != Line.tracks.end())
		{
			Edit.capture(*other);
			editor_track::move_point(*other, point, To);
			continue;
		}
		filler = true;
		if (style == nullptr && other->eType == tt_Normal)
			style = other;
	}
	if (filler)
	{
		segment_data path;
		path.points[segment_data::point::start] = To;
		path.points[segment_data::point::end] = From;
		auto *piece{editor_track::create_path(style != nullptr ? *style : Track, path)};
		Edit.created.push_back(piece);
		Edit.changed.push_back(piece);
	}
	return true;
}

std::optional<editor_mode::switch_branch> editor_mode::take_switch_branch(TTrack const &Track, editor_track::joint const &Joint, std::size_t const Index) const
{
	auto *first{Joint.other};
	if (first == nullptr || first->eType != tt_Normal || first->m_paths.empty())
		return std::nullopt;
	auto const category{first->iCategoryFlag & 15};
	auto const regular = [category](TTrack const *Other) { return Other != nullptr && Other->eType == tt_Normal && false == Other->m_editorremoved && editor_track::is_supported(*Other) && (Other->iCategoryFlag & 15) == category; };
	// from the switch on: a straight up to the curve maybe, the curve, the straight past it
	auto const walk{walked_branch(first, Joint.theirs.kind == editor_track::point_kind::start, regular, m_straights.tolerance)};
	if (walk.phase != 2)
		return std::nullopt;
	auto const &run{walk.run};
	auto const straight{walk.straight};
	switch_branch branch;
	branch.joint = Index;
	branch.straight = straight;
	std::string error;
	auto &chain{branch.chain};
	if (false == editor_track::find_chain(run.front(), run.back(), chain, error) || chain.tracks != run || plan_distance(chain.start, editor_track::point_position(Track, Joint.own)) > geometry::same_point)
		return std::nullopt;
	auto &design{branch.design};
	design.norms = m_route.design.norms;
	design.shape = m_route.design.shape;
	design.transition_pieces = m_route.design.transition_pieces;
	design.speed = chain.velocity > 0.0 ? chain.velocity : 100.0;
	if (auto const hit{alignment_store::on_track(*run[straight - 1])})
	{
		auto const &stored{alignment_store::records()[hit->record].design};
		design.norms = stored.norms;
		design.shape = stored.shape;
		design.transition_pieces = stored.transition_pieces;
		design.arc_piece_angle = stored.arc_piece_angle;
		design.speed = stored.speed;
	}
	design.norms.checked = false;
	auto const found{editor_track::recognize_line(chain, m_straights.tolerance, design.norms.gauge, design.shape)};
	if (found.size() != 1 || found.front().kink)
		return std::nullopt;
	design.start = chain.start;
	design.end = chain.end;
	design.start_direction = glm::normalize(plan_of(chain.start_direction));
	design.end_direction = glm::normalize(plan_of(chain.end_direction));
	design.start_grade = grade_of(chain.start_direction);
	design.end_grade = grade_of(chain.end_direction);
	auto vertex{route_vertex_of(found.front().shape)};
	vertex.position = found.front().position;
	design.vertices = {vertex};
	branch.result = alignment::compute(design);
	if (false == branch.result.valid)
		return std::nullopt;
	return branch;
}

bool editor_mode::plan_switch_branch(segment_data const &Path, editor_track::joint const &Joint, switch_branch &Branch) const
{
	auto const atend{Joint.own.kind == editor_track::point_kind::end};
	auto const start{Path.points[atend ? segment_data::point::end : segment_data::point::start]};
	auto const outward{end_outward(Path, atend)};
	auto laid{Branch.design};
	laid.start = start;
	laid.start_direction = glm::normalize(plan_of(outward));
	laid.start_grade = grade_of(outward);
	auto const original{Branch.design.vertices.front()};
	auto const make = [&](double const Factor) {
		laid.vertices = {scaled(original, Factor)};
		return alignment::compute(laid);
	};
	// compute() makes the curve tighter when it doesn't fit between the ends
	auto const fits = [&](alignment::result const &Result) { return Result.valid && false == Result.curves.empty() && Result.curves.front().radius >= laid.vertices.front().radius * (1.0 - 1e-6); };
	double factor{1.0};
	auto result{make(factor)};
	if (false == result.valid || result.curves.empty())
		return false;
	if (false == fits(result))
	{
		// the design keeps the curve the way it was made to fit
		factor = made_to_fit(laid.vertices.front(), result.curves.front());
		result = alignment::compute(laid);
		if (false == result.valid)
			return false;
	}
	else if (opening(Branch.result) < 0.1 && opening(result) >= 0.1)
	{
		// the curve which adjoined the switch keeps to it: the largest one which fits
		auto const largest{largest_fitting([&make, &fits](double const Factor) { return fits(make(Factor)); })};
		if (false == largest.has_value())
			return false;
		factor = *largest;
		result = make(factor);
		if (false == result.valid)
			return false;
	}
	// the paths are laid up to the first joint on the straight past the old and the new curve
	auto const &chain{Branch.chain};
	auto const end{plan_of(chain.end)};
	auto const behind = [&](glm::dvec3 const &Point) { return glm::dot(end - plan_of(Point), laid.end_direction); };
	auto const curveend{closing(result)};
	auto last{chain.tracks.size()};
	for (auto k{Branch.straight - 1}; k < chain.tracks.size(); ++k)
		if (behind(exit_of(chain, k)) <= curveend - 0.5)
		{
			last = k;
			break;
		}
	if (last == chain.tracks.size())
		return false;
	auto part{laid};
	auto const joint{exit_of(chain, last)};
	part.end = joint;
	part.end_grade = grade_of(end_outward(chain.tracks[last]->m_paths.front(), chain.forward[last]));
	auto const partresult{alignment::compute(part)};
	if (false == partresult.valid)
		return false;
	Branch.part = {};
	Branch.part.tracks.assign(chain.tracks.begin(), chain.tracks.begin() + last + 1);
	Branch.part.forward.assign(chain.forward.begin(), chain.forward.begin() + last + 1);
	Branch.part.start = chain.start;
	Branch.part.end = joint;
	Branch.pieces = alignment::pieces(partresult, part, last + 1);
	if (Branch.pieces.size() < last + 1)
		return false;
	editor_track::keep_heights(Branch.part, Branch.pieces);
	lift(Branch.pieces, start.y - chain.start.y);
	Branch.pieces.front().points[segment_data::point::start] = start;
	Branch.pieces.back().points[segment_data::point::end] = joint;
	Branch.laid = laid;
	Branch.factor = factor;
	return true;
}

void editor_mode::lay_switch_branch(switch_branch const &Branch, editor_track::edit &Edit) const
{
	for (auto *track : Branch.part.tracks)
		Edit.capture(*track);
	auto const created{editor_track::relay(Branch.part, Branch.pieces)};
	Edit.created.insert(Edit.created.end(), created.begin(), created.end());
	// the design kept for the line starting or ending at the switch follows it
	auto const &records{alignment_store::records()};
	auto const from{Branch.chain.start};
	for (std::size_t i = 0; i < records.size(); ++i)
	{
		auto const &record{records[i]};
		if (false == record.whole())
			continue;
		auto const atstart{plan_distance(record.design.start, from) < geometry::same_point};
		if (false == atstart && plan_distance(record.design.end, from) >= geometry::same_point)
			continue;
		record_follows(Branch, i, atstart);
		return;
	}
}

bool editor_mode::move_switch_paths(TTrack &Track, std::vector<segment_data> const &Paths, std::string &Status)
{
	if (Paths.size() < 2 || Track.m_paths.size() < 2 || false == editor_track::main_axis(Track).has_value())
		return false;
	double low{0.0};
	double high{0.0};
	std::string reason;
	bool const online{switch_room(Track, low, high, reason)};
	if (false == online && false == reason.empty())
	{
		Status = reason;
		return false;
	}
	auto const ends{switch_end_moves(Track, Paths.front(), online, {low, high}, m_straights.tolerance.offset, Status)};
	if (false == ends.has_value())
		return false;
	auto const joints{editor_track::joints(Track)};
	std::vector<glm::dvec3> positions;
	for (auto const &joint : joints)
		positions.push_back(editor_track::point_position(Track, joint.own));
	std::vector<bool> handled(joints.size(), false);
	auto branches{switch_branches(Track, joints)};
	editor_track::edit edit;
	edit.capture(Track);
	auto const line{online ? editor_track::find_straight(Track, m_straights.tolerance) : editor_track::straight{}};
	for (auto const &end : *ends)
	{
		for (std::size_t i = 0; i < joints.size(); ++i)
			if (glm::distance(positions[i], end.from) < geometry::same_point)
				handled[i] = true;
		if (false == switch_vacate(Track, end.from, end.to, end.shift, line, edit, Status))
			return false;
	}
	Track.m_paths.assign(Paths.begin(), Paths.begin() + 2);
	auto const notes{plan_switch_branches(Track, joints, branches, handled)};
	auto const loose{switch_rejoin(Track, joints, handled, edit)};
	editor_track::commit(edit.changed);
	for (auto const &branch : branches)
		lay_switch_branch(branch, edit);
	push_track_snapshot(std::move(edit.states), std::move(edit.created), std::move(edit.removed));
	moved_status(Status, loose, notes);
	return true;
}

std::vector<editor_mode::switch_branch> editor_mode::switch_branches(TTrack const &Track, std::vector<editor_track::joint> const &Joints, std::vector<TTrack const *> const &Skip) const
{
	std::vector<switch_branch> branches;
	auto const axis{editor_track::main_axis(Track)};
	if (false == axis.has_value())
		return branches;
	for (std::size_t i = 0; i < Joints.size(); ++i)
	{
		if (std::ranges::find(Skip, Joints[i].other) != Skip.end() || axis->lateral(editor_track::point_position(Track, Joints[i].own)) <= m_straights.tolerance.offset)
			continue;
		if (auto branch{take_switch_branch(Track, Joints[i], i)})
			branches.push_back(std::move(*branch));
	}
	return branches;
}

std::string editor_mode::plan_switch_branches(TTrack const &Track, std::vector<editor_track::joint> const &Joints, std::vector<switch_branch> &Branches, std::vector<bool> &Handled) const
{
	std::string notes;
	for (auto branch{Branches.begin()}; branch != Branches.end();)
	{
		auto const &joint{Joints[branch->joint]};
		if (std::string reason; false == plan_switch_branch(Track.m_paths[joint.own.path], joint, *branch) || false == tracks_editable(branch->part.tracks, reason))
		{
			notes += (notes.empty() ? "" : "\n") + format(STR_C("The curve past the end of the switch can't keep to it, %s follows the switch on its own"), name_of(*joint.other).c_str());
			branch = Branches.erase(branch);
			continue;
		}
		Handled[branch->joint] = true;
		if (std::abs(branch->factor - 1.0) > 1e-4)
			notes += (notes.empty() ? "" : "\n") + format(STR_C("The curve past the end of the switch keeps to it with the radius of %.1f m instead of %.1f m"), branch->laid.vertices.front().radius, branch->design.vertices.front().radius);
		++branch;
	}
	return notes;
}

bool editor_mode::double_room(double_switch const &Set, double &Low, double &High, double &Length, std::string &Reason)
{
	double low{0.0};
	double high{0.0};
	double low2{0.0};
	double high2{0.0};
	auto const axis{editor_track::main_axis(*Set.first)};
	if (false == axis.has_value() || false == switch_room(*Set.first, low, high, Reason) || false == switch_room(*Set.second, low2, high2, Reason))
		return false;
	auto const &main{Set.second->m_paths.front()};
	Low = low;
	High = axis->along(main.points[segment_data::point::start]) + high2;
	Length = axis->along(main.points[segment_data::point::end]);
	return true;
}

bool editor_mode::move_double_paths(double_switch const &Set, std::vector<segment_data> const &Paths, std::string &Status)
{
	auto const axis{editor_track::main_axis(*Set.first)};
	if (Paths.size() < 5 || false == axis.has_value())
		return false;
	double low{0.0};
	double high{0.0};
	double length{0.0};
	std::string reason;
	bool const online{double_room(Set, low, high, length, reason)};
	if (false == online && false == reason.empty())
	{
		Status = reason;
		return false;
	}
	std::array<outer_place, 2> const places{{{Set.first, Set.first->m_paths.front().points[segment_data::point::start], Paths[0].points[segment_data::point::start], true},
	                                         {Set.second, Set.second->m_paths.front().points[segment_data::point::end], Paths[2].points[segment_data::point::end], false}}};
	auto const ends{outer_moves(online, *axis, places, {low, high}, m_straights.tolerance.offset, Status)};
	if (false == ends.has_value())
		return false;
	std::array<TTrack *, 2> const switches{Set.first, Set.second};
	auto pair{joints_of_pair(switches, Set.link)};
	auto const &joints{pair.joints};
	auto const &positions{pair.positions};
	auto &handled{pair.handled};
	auto const joint{Set.first->m_paths.front().points[segment_data::point::end]};
	auto const &link{Set.link->m_paths.front()};
	bool const reversed{glm::distance(link.points[segment_data::point::start], joint) > glm::distance(link.points[segment_data::point::end], joint)};
	std::array<std::vector<switch_branch>, 2> branches;
	for (std::size_t k = 0; k < 2; ++k)
		branches[k] = switch_branches(*switches[k], joints[k], {Set.first, Set.link, Set.second});
	editor_track::edit edit;
	for (auto *track : {Set.first, Set.link, Set.second})
		edit.capture(*track);
	for (auto const &end : *ends)
	{
		handled_at(positions, end.move.from, handled);
		auto const line{editor_track::find_straight(*end.track, m_straights.tolerance)};
		if (false == switch_vacate(*end.track, end.move.from, end.move.to, end.move.shift, line, edit, Status))
			return false;
	}
	Set.first->m_paths.assign(Paths.begin(), Paths.begin() + 2);
	Set.second->m_paths.assign(Paths.begin() + 2, Paths.begin() + 4);
	Set.link->m_paths = {reversed ? geometry::reversed(Paths[4]) : Paths[4]};
	std::string notes;
	for (std::size_t k = 0; k < 2; ++k)
		if (auto const note{plan_switch_branches(*switches[k], joints[k], branches[k], handled[k])}; false == note.empty())
			notes = joined(notes, "\n", note);
	std::string loose;
	for (std::size_t k = 0; k < 2; ++k)
		if (auto const rejoin_problem{switch_rejoin(*switches[k], joints[k], handled[k], edit)}; false == rejoin_problem.empty())
			loose = joined(loose, ", ", rejoin_problem);
	editor_track::commit(edit.changed);
	for (auto const &set : branches)
		for (auto const &branch : set)
			lay_switch_branch(branch, edit);
	push_track_snapshot(std::move(edit.states), std::move(edit.created), std::move(edit.removed));
	moved_status(Status, loose, notes);
	return true;
}

bool editor_mode::slip_room(slip_set const &Slip, slip_slide &Slide, std::string &Reason) const
{
	Slide = slip_slide{};
	Reason.clear();
	// one laid out of straights only, along its first one
	for (auto const *part : Slip.switches)
		if (has_controls(part->m_paths.front()))
		{
			Reason = STR_C("A slip bent into a curve isn't slid along it: delete it and lay it again");
			return false;
		}
	if (false == tracks_editable(Slip.parts, Reason))
		return false;
	auto const chord{Slip.ends[1] - Slip.ends[0]};
	auto const run{glm::length(plan_of(chord))};
	if (run < 1e-6)
		return false;
	Slide.shift = chord / run;
	Slide.movers = Slip.parts;
	// its crossing track goes along where it goes on into tracks ending free, as when it was laid on one straight
	for (std::size_t end = 2; end < 4; ++end)
		if (false == crossing_movers(Slip.parts, Slip.ends[end], Slide.movers, Reason))
			return false;
	// the straights of its first track out to the joints of its template, as long as the ones of its crossing track going along, go
	// along too, the joints with them
	auto const extension{Slide.movers.size() > Slip.parts.size() ? geometry::chord_length(Slide.movers.back()->m_paths.front()) : 0.0};
	std::array<glm::dvec3, 2> joints{Slip.ends[0], Slip.ends[1]};
	for (std::size_t end = 0; end < 2 && extension > 0.0; ++end)
		straights_along(Slip.parts, Slip.ends[end], extension, Slide.shift, Slide.movers, joints[end]);
	Slide.joints = joints;
	// the straights of its first track before it and past it give and take the length, a metre of them kept at least
	std::array room{1e4, 1e4};
	std::array straight{true, true};
	for (std::size_t end = 0; end < 2; ++end)
		if (auto const beyond{track_beyond(Slide.movers, joints[end])})
		{
			Slide.ends[end] = *beyond;
			straight[end] = straight_along(*beyond->first, Slide.shift);
			room[end] = straight[end] ? geometry::chord_length(beyond->first->m_paths.front()) - 1.0 : 0.0;
		}
	// back, it shortens the track before it and lengthens the one past it; on, the other way
	Slide.low = straight[1] ? -std::max(0.0, room[0]) : 0.0;
	Slide.high = straight[0] ? std::max(0.0, room[1]) : 0.0;
	return true;
}

bool editor_mode::slide_slip(slip_set const &Slip, double const Distance)
{
	slip_slide slide;
	if (std::string reason; false == slip_room(Slip, slide, reason))
	{
		m_switch.status = refused(reason);
		ui()->set_status(m_switch.status, true);
		return false;
	}
	auto const distance{std::clamp(Distance, slide.low, slide.high)};
	if (std::abs(distance) < 0.005)
		return false;
	auto const shift{slide.shift * distance};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> changed;
	for (auto *mover : slide.movers)
	{
		states.emplace_back(mover, editor_track::capture(*mover));
		for (auto &path : mover->m_paths)
			path.translate(shift);
		changed.push_back(mover);
	}
	for (std::size_t end = 0; end < 2; ++end)
		if (auto const &[other, point]{slide.ends[end]}; other != nullptr)
		{
			states.emplace_back(other, editor_track::capture(*other));
			editor_track::move_point(*other, point, slide.joints[end] + shift);
			changed.push_back(other);
		}
	editor_track::commit(changed);
	push_track_snapshot(std::move(states), {}, {});
	switch_store::follow();
	records_to_history();
	straight_refresh();
	m_switch.status = format(STR_C("Slip %s moved %.2f m along its track, its crossing track with it"), Slip.record.name.c_str(), distance);
	ui()->set_status(m_switch.status, false);
	return true;
}

bool editor_mode::bent_switch_frame(TTrack &Track, std::vector<curve_sample> &Frame, std::array<TTrack *, 2> &Sides, double &Station) const
{
	Frame.clear();
	Sides = {nullptr, nullptr};
	if (Track.eType != tt_Switch || Track.m_paths.size() != 2 || Track.m_editorremoved || false == has_controls(Track.m_paths.front()) || slip_of(Track).has_value() || double_of(Track).has_value() ||
	    crossover_of(Track).has_value())
		return false;
	// the plain tracks the main track goes on into, each run towards the end of the switch it joins: the one before into its start, the
	// one past it out of its end
	std::array<bool, 2> forward{true, true};
	for (auto const &joint : editor_track::joints(Track))
	{
		if (joint.own.path != 0 || joint.other == nullptr || joint.other->eType != tt_Normal || joint.other->m_editorremoved || joint.other->m_paths.size() != 1)
			continue;
		auto const past{joint.own.kind == editor_track::point_kind::end ? 1 : 0};
		Sides[past] = joint.other;
		forward[past] = (joint.theirs.kind == editor_track::point_kind::end) == (past == 0);
	}
	if (Sides[0] == nullptr || Sides[1] == nullptr || Sides[0] == Sides[1])
		return false;
	editor_track::chain chain;
	chain.tracks = {Sides[0], &Track, Sides[1]};
	chain.forward = {forward[0], true, forward[1]};
	Frame = sample_chain(chain);
	Station = nearest_station(Frame, Track.m_paths.front().points[segment_data::point::start]);
	return Frame.size() >= 2;
}

std::vector<segment_data> editor_mode::bent_switch_paths(TTrack const &Track, std::vector<curve_sample> const &Frame, double const Station, double const Distance) const
{
	// a path as seen from the curve where the switch starts: across it, over it, along it, as frame_path takes a path of a template
	auto const local_end = [&](glm::dvec3 const &World, glm::dvec3 const &Control, glm::dvec3 &Point, glm::dvec3 &Localcontrol) {
		auto const at{nearest_station(Frame, World)};
		auto const sample{sample_at(Frame, at)};
		glm::dvec2 const across{-sample.tangent.y, sample.tangent.x};
		auto const lateral{glm::dot(plan_of(World) - plan_of(sample.position), across)};
		Point = {lateral, World.y - sample.position.y + lateral * std::tan(glm::radians(static_cast<double>(sample.roll))), at - Station};
		auto const planar{plan_of(Control)};
		Localcontrol = glm::length(planar) > 1e-9 ? glm::dvec3{glm::dot(planar, across), Control.y, glm::dot(planar, sample.tangent)} : glm::dvec3{0.0};
	};
	auto const localized = [&](segment_data const &Path) {
		auto local{Path};
		local.pieces.clear();
		local_end(Path.points[segment_data::point::start], Path.points[segment_data::point::control1], local.points[segment_data::point::start], local.points[segment_data::point::control1]);
		local_end(Path.points[segment_data::point::end], Path.points[segment_data::point::control2], local.points[segment_data::point::end], local.points[segment_data::point::control2]);
		return local;
	};
	std::vector<segment_data> result;
	for (std::size_t i = 0; i < Track.m_paths.size(); ++i)
	{
		auto const &own{Track.m_paths[i]};
		auto path{frame_path(localized(own), Frame, Station + Distance, 1, 1)};
		for (auto const &piece : own.pieces)
			path.pieces.push_back(frame_path(localized(piece), Frame, Station + Distance, 1, 1));
		// the main track follows the curve it lies in
		if (i == 0)
			path.radius = own.radius;
		result.push_back(path);
	}
	return result;
}

bool editor_mode::slide_bent_switch(TTrack &Track, double const Distance)
{
	std::vector<curve_sample> frame;
	std::array<TTrack *, 2> sides;
	double station;
	if (false == bent_switch_frame(Track, frame, sides, station))
		return false;
	auto const &main{Track.m_paths.front()};
	auto const length{nearest_station(frame, main.points[segment_data::point::end]) - station};
	// as far as the far end of a track next to it, which goes then, the switch joining what it joined there: P4 of another switch, say
	auto const back{station};
	auto const ahead{frame.back().station - station - length};
	if (Distance < -back - 1e-3 || Distance > ahead + 1e-3)
	{
		m_switch.status = format(STR_C("The switch goes %.2f m back and %.2f m on along the curve at most"), std::max(0.0, back), std::max(0.0, ahead));
		ui()->set_status(m_switch.status, true);
		return false;
	}
	// a track shorter than this left beside it goes too
	constexpr double kRemnant{0.5};
	auto distance{std::clamp(Distance, -back, ahead)};
	if (back + distance < kRemnant)
		distance = -back;
	else if (ahead - distance < kRemnant)
		distance = ahead;
	std::string notes;
	if (false == lay_bent_switch(Track, frame, sides, bent_switch_paths(Track, frame, station, distance), notes))
	{
		m_switch.status = notes;
		ui()->set_status(m_switch.status, true);
		return false;
	}
	switch_store::follow();
	records_to_history();
	straight_refresh();
	m_switch.status = format(STR_C("Switch %s moved %.2f m along the curve"), Track.name().c_str(), distance) + (notes.empty() ? "" : "\n") + notes;
	ui()->set_status(m_switch.status, false == notes.empty());
	return true;
}

bool editor_mode::lay_bent_switch(TTrack &Track, std::vector<curve_sample> const &Frame, std::array<TTrack *, 2> const &Sides, std::vector<segment_data> const &Paths, std::string &Notes)
{
	Notes.clear();
	if (Paths.size() < 2 || Frame.size() < 2)
		return false;
	auto const &main{Paths.front()};
	// where the new ends of the switch fall along the curve, within the tracks next to it
	std::array<double, 2> const ends{nearest_station(Frame, main.points[segment_data::point::start]), nearest_station(Frame, main.points[segment_data::point::end])};
	auto const joints{editor_track::joints(Track)};
	std::vector<bool> handled(joints.size(), false);
	// the line leaving the end of the diverging track: the curve past it laid again up to the straight beyond it
	std::vector<switch_branch> branches;
	for (std::size_t i = 0; i < joints.size(); ++i)
		if (joints[i].own.path == 1)
			if (auto branch{take_switch_branch(Track, joints[i], i)})
				branches.push_back(std::move(*branch));
	editor_track::edit edit;
	edit.capture(Track);
	// the tracks of the curve on both sides laid along it up to the new ends of the switch
	std::vector<std::string> gone;
	for (int k = 0; k < 2; ++k)
	{
		auto &side{*Sides[k]};
		handled_with(joints, 0, &side, handled);
		// taken up whole: the switch takes its place up to its far end
		if ((k == 0 ? ends[0] : Frame.back().station - ends[1]) < 1e-3)
		{
			gone.push_back(side.name());
			editor_track::retire(side);
			edit.removed.push_back(&side);
			continue;
		}
		auto const end{k == 0 ? segment_data::point::start : segment_data::point::end};
		auto const laid{side_along_frame(side.m_paths.front(), Frame, ends[k], Track.m_paths.front().points[end], main.points[end])};
		edit.capture(side);
		side.m_paths.front() = laid;
	}
	Track.m_paths.assign(Paths.begin(), Paths.begin() + 2);
	auto const notes{plan_switch_branches(Track, joints, branches, handled)};
	auto const loose{switch_rejoin(Track, joints, handled, edit)};
	editor_track::commit(edit.changed);
	for (auto const &branch : branches)
		lay_switch_branch(branch, edit);
	push_track_snapshot(std::move(edit.states), std::move(edit.created), std::move(edit.removed));
	for (auto const &name : gone)
		Notes = joined(Notes, "\n", format(STR_C("%s taken up whole, the switch joins what it joined"), name.c_str()));
	if (false == loose.empty())
		Notes = joined(Notes, "\n", std::string{STR_C("Too far from the new ends of the switch, left unjoined: ")} + loose);
	if (false == notes.empty())
		Notes = joined(Notes, "\n", notes);
	return true;
}

segment_data editor_mode::side_along_frame(segment_data const &Path, std::vector<curve_sample> const &Frame, double const End, glm::dvec3 const &Joint, glm::dvec3 const &Corner)
{
	auto const atend{glm::distance(Path.points[segment_data::point::end], Joint) < glm::distance(Path.points[segment_data::point::start], Joint)};
	auto from{nearest_station(Frame, Path.points[segment_data::point::start])};
	auto to{nearest_station(Frame, Path.points[segment_data::point::end])};
	(atend ? to : from) = End;
	auto const a{sample_at(Frame, from)};
	auto const b{sample_at(Frame, to)};
	auto const way{to >= from ? 1.0 : -1.0};
	auto const span{std::abs(to - from)};
	auto const turn{std::abs(geometry::signed_angle(a.tangent, b.tangent))};
	auto const handle{turn > 1e-6 ? 4.0 / 3.0 * std::tan(turn / 4.0) * span / turn : span / 3.0};
	segment_data laid{Path};
	laid.pieces.clear();
	laid.points[segment_data::point::start] = a.position;
	laid.points[segment_data::point::end] = b.position;
	// right at the new end of the switch
	laid.points[atend ? segment_data::point::end : segment_data::point::start] = Corner;
	auto const rise{laid.points[segment_data::point::end].y - laid.points[segment_data::point::start].y};
	laid.points[segment_data::point::control1] = {a.tangent.x * way * handle, rise / 3.0, a.tangent.y * way * handle};
	laid.points[segment_data::point::control2] = {-b.tangent.x * way * handle, -rise / 3.0, -b.tangent.y * way * handle};
	laid.rolls = {static_cast<float>(a.roll * way), static_cast<float>(b.roll * way)};
	return laid;
}

bool editor_mode::slide_switch(TTrack &Track, double const Distance)
{
	if (std::abs(Distance) < 0.005)
		return false;
	// a slip slides whole
	if (auto const slip{slip_of(Track)})
		return slide_slip(*slip, Distance);
	// a switch of a crossover along a curve takes the crossover with it
	if (crossover_of(Track).has_value())
		return slide_curved_connection(Track, Distance);
	// one bent into a curve slides along the curve
	if (Track.m_paths.size() == 2 && has_controls(Track.m_paths.front()))
		return slide_bent_switch(Track, Distance);
	auto const set{double_of(Track)};
	auto &first{set.has_value() ? *set->first : Track};
	if (first.m_paths.size() < 2)
		return false;
	auto paths{editor_track::slid(first, Distance)};
	if (paths.empty())
		return false;
	std::string status;
	bool moved{false};
	if (set.has_value())
	{
		auto const shift{paths.front().points[segment_data::point::start] - first.m_paths.front().points[segment_data::point::start]};
		paths = double_paths(*set);
		for (auto &path : paths)
			path.translate(shift);
		moved = move_double_paths(*set, paths, status);
	}
	else
		moved = move_switch_paths(Track, paths, status);
	if (false == moved)
	{
		m_switch.status = status;
		ui()->set_status(status, true);
		return false;
	}
	switch_store::follow();
	records_to_history();
	straight_refresh();
	m_switch.status = status.empty() ? format(STR_C("Switch %s moved %.2f m along the straight"), first.name().c_str(), Distance) : status;
	ui()->set_status(m_switch.status, false == status.empty());
	return true;
}

bool editor_mode::switch_slides(TTrack &Track) const
{
	if (auto const slip{slip_of(Track)})
	{
		slip_slide room;
		std::string reason;
		return slip_room(*slip, room, reason);
	}
	if (Track.eType == tt_Normal)
	{
		auto const set{double_of(Track)};
		return set.has_value() && switch_slides(*set->first) && switch_slides(*set->second);
	}
	if (Track.eType != tt_Switch || Track.m_paths.size() != 2 || Track.m_editorremoved)
		return false;
	if (false == has_controls(Track.m_paths.front()))
		return true;
	// bent into a curve: along the curve, where it goes on into plain tracks on both sides
	std::vector<curve_sample> frame;
	std::array<TTrack *, 2> sides;
	double station;
	return bent_switch_frame(Track, frame, sides, station);
}

bool editor_mode::switch_slide_start_slip(TTrack const &Track)
{
	auto const slip{slip_of(Track)};
	if (false == slip.has_value())
		return false;
	auto &slide{m_switch.slide};
	std::string reason;
	slip_slide room;
	auto const axis{editor_track::main_axis(*slip->switches.front())};
	if (false == slip_room(*slip, room, reason) || false == axis.has_value())
	{
		if (false == reason.empty())
			ui()->set_status(reason, true);
		return true;
	}
	auto const &mouse{ImGui::GetIO().MousePos};
	slide.active = true;
	slide.track = slip->switches.front();
	slide.pressed = {mouse.x, mouse.y};
	slide.grip = axis->along(cursor_level(axis->origin.y));
	slide.length = 0.0;
	slide.low = room.low;
	slide.high = room.high;
	return true;
}

bool editor_mode::switch_slide_start_bent(TTrack &Track)
{
	auto &slide{m_switch.slide};
	if (std::array<TTrack *, 2> sides; false == bent_switch_frame(Track, slide.frame, sides, slide.station))
		return false;
	auto const &main{Track.m_paths.front()};
	auto const &mouse{ImGui::GetIO().MousePos};
	slide.active = true;
	slide.track = &Track;
	slide.pressed = {mouse.x, mouse.y};
	slide.grip = nearest_station(slide.frame, cursor_level(main.points[segment_data::point::start].y));
	slide.length = nearest_station(slide.frame, main.points[segment_data::point::end]) - slide.station;
	slide.low = -slide.station;
	slide.high = std::max(0.0, slide.frame.back().station - slide.station - slide.length);
	auto const joints{editor_track::joints(Track)};
	for (std::size_t i = 0; i < joints.size(); ++i)
		if (joints[i].own.path == 1)
			if (auto branch{take_switch_branch(Track, joints[i], i)})
				slide.branches.emplace_back(joints[i], 1, std::move(*branch));
	return true;
}

void editor_mode::switch_slide_start(TTrack &Track)
{
	auto &slide{m_switch.slide};
	slide = {};
	// a slip along its first track, from its first switch
	if (switch_slide_start_slip(Track))
		return;
	// bent into a curve: along it, the tracks next to it keeping a metre, the curve past its diverging track drawn as it'll be laid
	if (switch_slide_start_bent(Track))
		return;
	std::string reason;
	auto const set{double_of(Track)};
	auto &first{set.has_value() ? *set->first : Track};
	auto const axis{editor_track::main_axis(first)};
	if (false == axis.has_value())
		return;
	slide.length = axis->length;
	if (false == (set.has_value() ? double_room(*set, slide.low, slide.high, slide.length, reason) : switch_room(first, slide.low, slide.high, reason)))
	{
		if (false == reason.empty())
			ui()->set_status(reason, true);
		return;
	}
	auto const &mouse{ImGui::GetIO().MousePos};
	slide.active = true;
	slide.track = &first;
	slide.pressed = {mouse.x, mouse.y};
	slide.grip = axis->along(cursor_level(axis->origin.y));
	std::vector members{&first};
	if (set.has_value())
		members = {set->first, set->link, set->second};
	slide.joints = straight_joints(members, *axis, m_straights.tolerance);
	std::vector<TTrack const *> const skip(members.begin(), members.end());
	for (auto const *member : members)
	{
		if (member->eType != tt_Switch)
			continue;
		std::size_t const offset{set.has_value() && member == set->second ? 2u : 0u};
		auto const joints{editor_track::joints(*member)};
		for (auto &branch : switch_branches(*member, joints, skip))
		{
			auto const &joint{joints[branch.joint]};
			slide.branches.emplace_back(joint, offset + static_cast<std::size_t>(joint.own.path), std::move(branch));
		}
	}
}

void editor_mode::switch_slide_update()
{
	auto &slide{m_switch.slide};
	if (false == slide.active)
		return;
	// along the curve it's bent into
	if (false == slide.frame.empty())
	{
		if (slide.track == nullptr || slide.track->m_editorremoved)
		{
			slide = {};
			return;
		}
		auto const height{sample_at(slide.frame, slide.station).position.y};
		auto distance{std::clamp(nearest_station(slide.frame, cursor_level(height)) - slide.grip, slide.low, slide.high)};
		// drawn to the far ends of the tracks next to it, where it takes one up whole and joins what it joined
		for (auto const end : {slide.low, slide.high})
			if (std::abs(distance - end) < 1.0)
				distance = end;
		slide.distance = distance;
		return;
	}
	auto const axis{slide.track != nullptr && false == slide.track->m_editorremoved && slide.track->m_paths.size() >= 2 ? editor_track::main_axis(*slide.track) : std::nullopt};
	if (false == axis.has_value())
	{
		slide = {};
		return;
	}
	auto const length{slide.length};
	auto const distance{std::clamp(axis->along(cursor_level(axis->origin.y)) - slide.grip, slide.low, slide.high - length)};
	double best{1.0};
	double snapped{distance};
	for (auto const joint : slide.joints)
		for (auto const end : {0.0, length})
			if (auto const gap{std::abs(joint - (end + distance))}; gap < best && std::abs(joint - end) > 0.01)
			{
				best = gap;
				snapped = joint - end;
			}
	slide.distance = std::clamp(snapped, slide.low, slide.high - length);
}

void editor_mode::switch_slide_finish()
{
	auto &slide{m_switch.slide};
	if (false == slide.active)
		return;
	auto *track{slide.track};
	auto const distance{slide.distance};
	auto const &mouse{ImGui::GetIO().MousePos};
	auto const dragged{glm::length(glm::vec2{mouse.x, mouse.y} - slide.pressed) > kDragPixels};
	slide = {};
	if (track != nullptr && false == track->m_editorremoved && dragged)
		slide_switch(*track, distance);
}

void editor_mode::draw_switch_slide() const
{
	auto const &slide{m_switch.slide};
	if (false == slide.active || slide.track == nullptr || slide.track->m_paths.size() < 2)
		return;
	if (auto const &mouse{ImGui::GetIO().MousePos}; glm::length(glm::vec2{mouse.x, mouse.y} - slide.pressed) <= kDragPixels)
		return;
	auto paths{slide.frame.empty() ? editor_track::slid(*slide.track, slide.distance) : bent_switch_paths(*slide.track, slide.frame, slide.station, slide.distance)};
	if (paths.empty())
		return;
	// a slip with all it takes along
	if (auto const slip{slip_of(*slide.track)})
	{
		slip_slide room;
		std::string reason;
		slip_room(*slip, room, reason);
		paths = moved_paths(room.movers, room.shift * slide.distance);
	}
	else if (auto const set{double_of(*slide.track)})
	{
		auto const shift{paths.front().points[segment_data::point::start] - slide.track->m_paths.front().points[segment_data::point::start]};
		paths = shifted(double_paths(*set), shift);
	}
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	for (auto const &path : paths)
		draw_path(projection, drawlist, path, overlay_color::marked, 3.0f);
	// the curves past the switch, laid again up to the straight beyond them
	for (auto const &line : slide.branches)
	{
		if (line.path >= paths.size())
			continue;
		auto branch{line.branch};
		if (false == plan_switch_branch(paths[line.path], line.joint, branch))
			continue;
		for (auto const &piece : branch.pieces)
			draw_path(projection, drawlist, piece, overlay_color::grip, 2.0f);
	}
	draw_slide_label(projection, drawlist, paths.front().points[segment_data::point::start], slide.distance);
}
