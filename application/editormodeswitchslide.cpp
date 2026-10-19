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

std::string track_label(TTrack const &Track)
{
	return Track.name().empty() ? std::string{STR_C("(no name)")} : Track.name();
}

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
		ends.push_back({from, to, index == segment_data::point::start ? axis->along(from) - target : target - axis->along(from)});
	}
	std::stable_sort(ends.begin(), ends.end(), [](switch_end_move const &A, switch_end_move const &B) { return A.shift > B.shift; });
	return ends;
}

std::string switch_rejoin(TTrack &Track, std::vector<editor_track::joint> const &Joints, std::vector<bool> const &Handled, editor_track::edit &Edit)
{
	std::vector<editor_track::joint> following;
	std::string loose;
	for (std::size_t i = 0; i < Joints.size(); ++i)
	{
		auto const &joint{Joints[i]};
		if (Handled[i] || joint.other->m_editorremoved)
			continue;
		auto const &theirs{joint.other->m_paths[joint.theirs.path]};
		auto const shift{glm::distance(editor_track::point_position(Track, joint.own), editor_track::point_position(*joint.other, joint.theirs))};
		if (shift > 0.3 * bezier{theirs}.plan_length())
		{
			loose += (loose.empty() ? "" : ", ") + track_label(*joint.other);
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

} // namespace

bool editor_mode::switch_room(TTrack &Track, double &Low, double &High, std::string &Reason)
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
	std::vector<std::tuple<double, double, bool>> spans;
	for (std::size_t i = 0; i < line.tracks.size(); ++i)
	{
		if (line.tracks[i] == &Track)
			continue;
		auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
		auto const a{axis->along(path.points[segment_data::point::start])};
		auto const b{axis->along(path.points[segment_data::point::end])};
		spans.emplace_back(std::min(a, b), std::max(a, b), line.tracks[i]->eType == tt_Normal);
	}
	std::sort(spans.begin(), spans.end());
	double low{0.0};
	double high{axis->length};
	for (auto const &[from, to, plain] : spans)
		if (from > axis->length - 0.01)
		{
			if (false == plain || from > high + 0.01)
				break;
			high = std::max(high, to);
		}
	for (auto span{spans.rbegin()}; span != spans.rend(); ++span)
	{
		auto const &[from, to, plain] = *span;
		if (to < 0.01)
		{
			if (false == plain || to < low - 0.01)
				break;
			low = std::min(low, from);
		}
	}
	Low = low;
	High = high;
	return true;
}

bool editor_mode::switch_vacate(TTrack &Track, glm::dvec3 const &From, glm::dvec3 const &To, double const Shift, editor_track::straight const &Line, editor_track::edit &Edit, std::string &Status)
{
	if (Shift > 0.0)
	{
		auto const a{Line.along(From)};
		auto const b{Line.along(To)};
		if (cut_straight(Line, std::min(a, b), std::max(a, b), Edit.states, Edit.created, Edit.removed, nullptr))
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
		if (other->eType == tt_Normal && std::find(Line.tracks.begin(), Line.tracks.end(), other) != Line.tracks.end())
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
	auto const regular = [&](TTrack const *Other) { return Other != nullptr && Other->eType == tt_Normal && false == Other->m_editorremoved && editor_track::is_supported(*Other) && (Other->iCategoryFlag & 15) == category; };
	// from the switch on: a straight up to the curve maybe, the curve, the straight past it
	std::vector<TTrack *> run;
	auto *current{first};
	auto atstart{Joint.theirs.kind == editor_track::point_kind::start};
	int phase{0};
	std::size_t straight{0};
	double past{0.0};
	while (regular(current) && std::find(run.begin(), run.end(), current) == run.end() && past < kBranchStraight)
	{
		auto const plain{editor_track::is_straight(*current, m_straights.tolerance)};
		if (false == plain && phase == 2)
			break;
		if (false == plain)
			phase = 1;
		else if (phase == 1)
		{
			phase = 2;
			straight = run.size();
		}
		if (phase == 2)
			past += current->Length();
		run.push_back(current);
		auto *next{atstart ? current->trNext : current->trPrev};
		if (next == nullptr || (next->trPrev != current && next->trNext != current))
			break;
		atstart = next->trPrev == current;
		current = next;
	}
	if (phase != 2)
		return std::nullopt;
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
		auto &vertex{laid.vertices.front()};
		auto const &report{result.curves.front()};
		auto const reduced{report.radius / vertex.radius};
		if (vertex.compound)
			vertex = scaled(vertex, reduced);
		else
		{
			vertex.radius = report.radius;
			vertex.transition_in = report.transition_in;
			vertex.transition_out = report.transition_out;
		}
		factor = reduced;
		result = alignment::compute(laid);
		if (false == result.valid)
			return false;
	}
	else if (opening(Branch.result) < 0.1 && opening(result) >= 0.1)
	{
		// the curve which adjoined the switch keeps to it: the largest one which fits
		double low{1.0};
		double high{1.5};
		while (fits(make(high)) && high < 5.0)
			high *= 1.5;
		if (fits(make(high)))
			return false;
		for (int i = 0; i < 40; ++i)
		{
			auto const middle{0.5 * (low + high)};
			(fits(make(middle)) ? low : high) = middle;
		}
		factor = low;
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

void editor_mode::lay_switch_branch(switch_branch const &Branch, editor_track::edit &Edit)
{
	for (auto *track : Branch.part.tracks)
		Edit.capture(*track);
	auto const created{editor_track::relay(Branch.part, Branch.pieces)};
	Edit.created.insert(Edit.created.end(), created.begin(), created.end());
	// the design kept for the line starting or ending at the switch follows it
	auto const &records{alignment_store::records()};
	auto const from{Branch.chain.start};
	auto const joint{Branch.part.end};
	for (std::size_t i = 0; i < records.size(); ++i)
	{
		auto const &record{records[i]};
		auto const atstart{plan_distance(record.design.start, from) < geometry::same_point};
		if (false == atstart && plan_distance(record.design.end, from) >= geometry::same_point)
			continue;
		auto design{record.design};
		auto &vertices{design.vertices};
		if (vertices.empty())
			return;
		auto const outward{Branch.laid.start_direction};
		auto const k{atstart ? std::size_t{0} : vertices.size() - 1};
		if (atstart)
		{
			design.start = Branch.laid.start;
			design.start_direction = outward;
			design.start_grade = Branch.laid.start_grade;
		}
		else
		{
			design.end = Branch.laid.start;
			design.end_direction = -outward;
			design.end_grade = -Branch.laid.start_grade;
		}
		auto const &corners{record.result.vertices};
		if (vertices.size() >= 2 && corners.size() == vertices.size())
		{
			// the corner moves along its other leg to the line from the switch
			auto const corner{corners[k]};
			auto const leg{corners[atstart ? 1 : k - 1] - corner};
			auto const determinant{cross(outward, leg)};
			if (std::abs(determinant) < 1e-9)
				return;
			auto const offset{cross(corner - plan_of(Branch.laid.start), leg) / determinant};
			if (offset <= 0.0)
				return;
			vertices[k].offset = offset;
		}
		// the curve next to the switch takes the shape it was laid with
		auto const &shape{Branch.laid.vertices.front()};
		if (shape.compound || vertices[k].compound)
			vertices[k] = scaled(vertices[k], Branch.factor);
		else
		{
			vertices[k].radius = shape.radius;
			vertices[k].transition_in = atstart ? shape.transition_in : shape.transition_out;
			vertices[k].transition_out = atstart ? shape.transition_out : shape.transition_in;
		}
		auto const result{alignment::compute(design)};
		if (false == result.valid || false == alignment_store::follows(result, Branch.part.tracks))
			return;
		// the km of the joint the paths are laid up to stays as it was
		auto origin{record.origin};
		if (atstart)
		{
			auto const before{alignment_store::chainage_on(record.result, joint)};
			auto const after{alignment_store::chainage_on(result, joint)};
			if (before.has_value() && after.has_value())
				origin += *before - *after;
		}
		auto const layer{record.layer};
		auto const source{record.source};
		auto const fit{record.fit};
		alignment_store::forget(static_cast<int>(i));
		alignment_store::store(design, result, origin, layer, source, fit);
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
	if (false == loose.empty())
		Status = STR_C("Too far from the new ends of the switch, left unjoined: ") + loose;
	if (false == notes.empty())
		Status += (Status.empty() ? "" : "\n") + notes;
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
		if (std::find(Skip.begin(), Skip.end(), Joints[i].other) != Skip.end() || axis->lateral(editor_track::point_position(Track, Joints[i].own)) <= m_straights.tolerance.offset)
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
		std::string reason;
		if (false == plan_switch_branch(Track.m_paths[joint.own.path], joint, *branch) || false == tracks_editable(branch->part.tracks, reason))
		{
			notes += (notes.empty() ? "" : "\n") + format(STR_C("The curve past the end of the switch can't keep to it, %s follows the switch on its own"), track_label(*joint.other).c_str());
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
	struct outer_end
	{
		TTrack *track{nullptr};
		switch_end_move move;
	};
	std::vector<outer_end> ends;
	auto const outer = [&](TTrack &Track, glm::dvec3 const &From, glm::dvec3 const &To, bool const Atstart) {
		if (false == online || glm::distance(From, To) < 0.005 || axis->lateral(To) > m_straights.tolerance.offset)
			return true;
		auto const target{axis->along(To)};
		if (target < low - 0.01 || target > high + 0.01)
		{
			Status = STR_C("The switch would run into another switch or past the end of the straight");
			return false;
		}
		ends.push_back({&Track, {From, To, Atstart ? axis->along(From) - target : target - axis->along(From)}});
		return true;
	};
	if (false == outer(*Set.first, Set.first->m_paths.front().points[segment_data::point::start], Paths[0].points[segment_data::point::start], true) ||
	    false == outer(*Set.second, Set.second->m_paths.front().points[segment_data::point::end], Paths[2].points[segment_data::point::end], false))
		return false;
	std::stable_sort(ends.begin(), ends.end(), [](outer_end const &A, outer_end const &B) { return A.move.shift > B.move.shift; });
	std::array<TTrack *, 2> const switches{Set.first, Set.second};
	std::array<std::vector<editor_track::joint>, 2> joints;
	std::array<std::vector<glm::dvec3>, 2> positions;
	std::array<std::vector<bool>, 2> handled;
	for (std::size_t k = 0; k < 2; ++k)
	{
		joints[k] = editor_track::joints(*switches[k]);
		for (auto const &joint : joints[k])
		{
			positions[k].push_back(editor_track::point_position(*switches[k], joint.own));
			handled[k].push_back(joint.other == Set.link || joint.other == Set.first || joint.other == Set.second);
		}
	}
	auto const joint{Set.first->m_paths.front().points[segment_data::point::end]};
	auto const &link{Set.link->m_paths.front()};
	bool const reversed{glm::distance(link.points[segment_data::point::start], joint) > glm::distance(link.points[segment_data::point::end], joint)};
	std::array<std::vector<switch_branch>, 2> branches;
	for (std::size_t k = 0; k < 2; ++k)
		branches[k] = switch_branches(*switches[k], joints[k], {Set.first, Set.link, Set.second});
	editor_track::edit edit;
	for (auto *track : {Set.first, Set.link, Set.second})
		edit.capture(*track);
	for (auto const &end : ends)
	{
		for (std::size_t k = 0; k < 2; ++k)
			for (std::size_t i = 0; i < joints[k].size(); ++i)
				if (glm::distance(positions[k][i], end.move.from) < geometry::same_point)
					handled[k][i] = true;
		auto const line{editor_track::find_straight(*end.track, m_straights.tolerance)};
		if (false == switch_vacate(*end.track, end.move.from, end.move.to, end.move.shift, line, edit, Status))
			return false;
	}
	Set.first->m_paths.assign(Paths.begin(), Paths.begin() + 2);
	Set.second->m_paths.assign(Paths.begin() + 2, Paths.begin() + 4);
	Set.link->m_paths = {reversed ? geometry::reversed(Paths[4]) : Paths[4]};
	std::string notes;
	for (std::size_t k = 0; k < 2; ++k)
	{
		auto const note{plan_switch_branches(*switches[k], joints[k], branches[k], handled[k])};
		if (false == note.empty())
			notes += (notes.empty() ? "" : "\n") + note;
	}
	std::string loose;
	for (std::size_t k = 0; k < 2; ++k)
	{
		auto const left{switch_rejoin(*switches[k], joints[k], handled[k], edit)};
		if (false == left.empty())
			loose += (loose.empty() ? "" : ", ") + left;
	}
	editor_track::commit(edit.changed);
	for (auto const &set : branches)
		for (auto const &branch : set)
			lay_switch_branch(branch, edit);
	push_track_snapshot(std::move(edit.states), std::move(edit.created), std::move(edit.removed));
	if (false == loose.empty())
		Status = STR_C("Too far from the new ends of the switch, left unjoined: ") + loose;
	if (false == notes.empty())
		Status += (Status.empty() ? "" : "\n") + notes;
	return true;
}

bool editor_mode::slide_switch(TTrack &Track, double const Distance)
{
	if (std::abs(Distance) < 0.005)
		return false;
	// a switch of a crossover along a curve takes the crossover with it
	if (crossover_of(Track).has_value())
		return slide_curved_connection(Track, Distance);
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

bool editor_mode::switch_slides(TTrack const &Track) const
{
	if (Track.eType == tt_Normal)
	{
		auto const set{double_of(Track)};
		return set.has_value() && switch_slides(*set->first) && switch_slides(*set->second);
	}
	return Track.eType == tt_Switch && Track.m_paths.size() == 2 && false == Track.m_editorremoved && false == has_controls(Track.m_paths.front());
}

void editor_mode::switch_slide_start(TTrack &Track)
{
	auto &slide{m_switch.slide};
	slide = {};
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
	std::vector<TTrack *> members{&first};
	if (set.has_value())
		members = {set->first, set->link, set->second};
	for (auto *member : members)
	{
		if (member->eType != tt_Switch)
			continue;
		auto const line{editor_track::find_straight(*member, m_straights.tolerance)};
		for (std::size_t i = 0; i < line.tracks.size(); ++i)
		{
			if (std::find(members.begin(), members.end(), line.tracks[i]) != members.end())
				continue;
			auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
			for (auto const index : {segment_data::point::start, segment_data::point::end})
				slide.joints.push_back(axis->along(path.points[index]));
		}
	}
	std::vector<TTrack const *> const skip(members.begin(), members.end());
	for (auto *member : members)
	{
		if (member->eType != tt_Switch)
			continue;
		std::size_t const offset{set.has_value() && member == set->second ? 2u : 0u};
		auto const joints{editor_track::joints(*member)};
		for (auto &branch : switch_branches(*member, joints, skip))
		{
			auto const &joint{joints[branch.joint]};
			slide.branches.push_back({joint, offset + static_cast<std::size_t>(joint.own.path), std::move(branch)});
		}
	}
}

void editor_mode::switch_slide_update()
{
	auto &slide{m_switch.slide};
	if (false == slide.active)
		return;
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
	auto const &mouse{ImGui::GetIO().MousePos};
	if (glm::length(glm::vec2{mouse.x, mouse.y} - slide.pressed) <= kDragPixels)
		return;
	auto paths{editor_track::slid(*slide.track, slide.distance)};
	if (paths.empty())
		return;
	if (auto const set{double_of(*slide.track)})
	{
		auto const shift{paths.front().points[segment_data::point::start] - slide.track->m_paths.front().points[segment_data::point::start]};
		paths = double_paths(*set);
		for (auto &path : paths)
			path.translate(shift);
	}
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const draw = [&](segment_data const &Path, ImU32 const Color, float const Width) {
		bezier const curve{Path};
		auto previous{curve.point(0.0)};
		for (int i = 1; i <= 24; ++i)
		{
			auto const next{curve.point(i / 24.0)};
			projection.line(drawlist, previous, next, Color, Width);
			previous = next;
		}
	};
	for (auto const &path : paths)
		draw(path, overlay_color::marked, 3.0f);
	// the curves past the switch, laid again up to the straight beyond them
	for (auto const &line : slide.branches)
	{
		if (line.path >= paths.size())
			continue;
		auto branch{line.branch};
		if (plan_switch_branch(paths[line.path], line.joint, branch))
			for (auto const &piece : branch.pieces)
				draw(piece, overlay_color::grip, 2.0f);
	}
	ImVec2 at;
	auto const start{paths.front().points[segment_data::point::start]};
	if (false == projection.project(start, at))
		return;
	auto const km{alignment_store::chainage_at(start)};
	auto const label{km.has_value() ? format("%+.2f m  km %.3f", slide.distance, *km / 1000.0) : format("%+.2f m", slide.distance)};
	auto const size{ImGui::CalcTextSize(label.c_str())};
	drawlist->AddRectFilled(ImVec2(at.x + 10.0f, at.y - 8.0f), ImVec2(at.x + 18.0f + size.x, at.y + 8.0f), IM_COL32(0, 0, 0, 180), 3.0f);
	drawlist->AddText(ImVec2(at.x + 14.0f, at.y - 8.0f), overlay_color::highlight, label.c_str());
}
