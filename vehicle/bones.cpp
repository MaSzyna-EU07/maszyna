/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/bones.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <iterator>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

namespace bones {

namespace {

bool read_number(std::string const &Word, double &Number)
{
	char *end{nullptr};
	Number = std::strtod(Word.c_str(), &end);
	return end != Word.c_str() && *end == '\0';
}

// the three numbers of the words from the First
bool read_vector(std::vector<std::string> const &Words, std::size_t const First, glm::dvec3 &Vector)
{
	if (Words.size() < First + 3)
	{
		return false;
	}
	return read_number(Words[First], Vector.x) && read_number(Words[First + 1], Vector.y) && read_number(Words[First + 2], Vector.z);
}

std::string read_crank(rig const &, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::crank;
	Point.wheel = Words[2];
	return read_vector(Words, 3, Point.rest) ? "" : "crank: the position isn't three numbers";
}

std::string read_slide(rig const &Rig, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::slide;
	Point.driver = Rig.find(Words[2]);
	if (Point.driver < 0)
	{
		return "slide: no point " + Words[2] + " defined before it";
	}
	if (Rig.points[Point.driver].kind == point_kind::node)
	{
		return "slide: a node of a chain can't drive it";
	}
	if (false == read_vector(Words, 3, Point.rest) || false == read_vector(Words, 6, Point.guide))
	{
		return "slide: the position or the guide isn't three numbers";
	}
	if (glm::length(Point.guide) < 1e-6)
	{
		return "slide: the guide has no direction";
	}
	Point.guide = glm::normalize(Point.guide);
	return "";
}

std::string read_coupler(rig const &, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::coupler;
	if (Words[2] == "front")
	{
		Point.end = 0;
	}
	else if (Words[2] == "rear")
	{
		Point.end = 1;
	}
	else
	{
		return "coupler: the end is front or rear, not " + Words[2];
	}
	return read_vector(Words, 3, Point.rest) ? "" : "coupler: the position isn't three numbers";
}

std::string read_middle(rig const &Rig, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::middle;
	Point.driver = Rig.find(Words[2]);
	Point.second = Rig.find(Words[3]);
	if (Point.driver < 0 || Point.second < 0)
	{
		return "mid: the points " + Words[2] + " and " + Words[3] + " are to be defined before it";
	}
	if (Rig.points[Point.driver].kind == point_kind::node || Rig.points[Point.second].kind == point_kind::node)
	{
		return "mid: not between the nodes of a chain";
	}
	Point.rest = 0.5 * (Rig.points[Point.driver].rest + Rig.points[Point.second].rest);
	return "";
}

std::string read_fixed(rig const &, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::fixed;
	return read_vector(Words, 2, Point.rest) ? "" : "fixed: the position isn't three numbers";
}

} // namespace

int rig::find(std::string_view const Name) const
{
	auto const found{std::ranges::find(points, Name, &point::name)};
	return found == points.end() ? -1 : static_cast<int>(found - points.begin());
}

int point_words(std::string_view const Kind)
{
	if (Kind == "fixed")
	{
		return 3;
	}
	if (Kind == "crank" || Kind == "coupler")
	{
		return 4;
	}
	if (Kind == "mid")
	{
		return 2;
	}
	if (Kind == "slide")
	{
		return 7;
	}
	return 0;
}

std::string add_point(rig &Rig, std::vector<std::string> const &Words)
{
	if (Words.size() < 2)
	{
		return "a point needs a name and a kind";
	}
	auto const count{point_words(Words[1])};
	if (count == 0)
	{
		return "unknown kind of point: " + Words[1];
	}
	if (Words.size() < static_cast<std::size_t>(count) + 2)
	{
		return "too few words for a point of the kind " + Words[1];
	}
	if (Rig.find(Words[0]) >= 0)
	{
		return "the point " + Words[0] + " is defined twice";
	}
	point point;
	point.name = Words[0];
	std::string error;
	if (Words[1] == "fixed")
	{
		error = read_fixed(Rig, point, Words);
	}
	else if (Words[1] == "crank")
	{
		error = read_crank(Rig, point, Words);
	}
	else if (Words[1] == "slide")
	{
		error = read_slide(Rig, point, Words);
	}
	else if (Words[1] == "mid")
	{
		error = read_middle(Rig, point, Words);
	}
	else
	{
		error = read_coupler(Rig, point, Words);
	}
	if (error.empty())
	{
		Rig.points.push_back(std::move(point));
	}
	return error;
}

std::string add_bone(rig &Rig, std::vector<std::string> const &Words)
{
	if (Words.size() < 3)
	{
		return "a bone needs its submodel, its anchor and the point it faces (or none)";
	}
	bone bone;
	bone.submodel = Words[0];
	bone.anchor = Rig.find(Words[1]);
	if (bone.anchor < 0)
	{
		return "no point " + Words[1] + " for the anchor of the bone " + Words[0];
	}
	if (Words[2] != "none")
	{
		bone.aim = Rig.find(Words[2]);
		if (bone.aim < 0)
		{
			return "no point " + Words[2] + " for the bone " + Words[0] + " to face";
		}
	}
	Rig.bones.push_back(std::move(bone));
	return "";
}

std::string add_chain(rig &Rig, std::vector<std::string> const &Words)
{
	if (Words.size() < 4)
	{
		return "a chain needs its name, its two points and the count of its nodes";
	}
	chain chain;
	chain.name = Words[0];
	chain.start = Rig.find(Words[1]);
	chain.end = Rig.find(Words[2]);
	if (chain.start < 0 || chain.end < 0)
	{
		return "the points " + Words[1] + " and " + Words[2] + " of the chain " + Words[0] + " are to be defined before it";
	}
	auto count{0.0};
	if (false == read_number(Words[3], count) || count < 1.0 || count > 64.0)
	{
		return "the chain " + Words[0] + " needs 1 to 64 nodes";
	}
	auto const nodes{static_cast<std::size_t>(count)};
	std::vector<glm::dvec3> positions{Rig.points[chain.start].rest};
	for (std::size_t node{0}; node < nodes; ++node)
	{
		glm::dvec3 position;
		if (false == read_vector(Words, 4 + 3 * node, position))
		{
			return "the chain " + Words[0] + " has no three numbers for its node " + std::to_string(node + 1);
		}
		positions.push_back(position);
	}
	positions.push_back(Rig.points[chain.end].rest);
	for (std::size_t piece{0}; piece + 1 < positions.size(); ++piece)
	{
		chain.lengths.push_back(glm::length(positions[piece + 1] - positions[piece]));
	}
	for (std::size_t point{0}; point + 2 < positions.size(); ++point)
	{
		chain.bends.push_back(glm::length(positions[point + 2] - positions[point]));
	}
	for (std::size_t node{0}; node < nodes; ++node)
	{
		auto const name{Words[0] + "." + std::to_string(node + 1)};
		if (Rig.find(name) >= 0)
		{
			return "the point " + name + " is defined twice";
		}
		point point;
		point.name = name;
		point.kind = point_kind::node;
		point.rest = positions[node + 1];
		chain.nodes.push_back(static_cast<int>(Rig.points.size()));
		Rig.points.push_back(std::move(point));
	}
	Rig.chains.push_back(std::move(chain));
	return "";
}

std::string add_bend(rig &Rig, std::vector<std::string> const &Words)
{
	if (Words.size() < 3)
	{
		return "a bent submodel needs its name and two points of its spine at least";
	}
	bend bend;
	bend.submodel = Words[0];
	bend.rooted = std::ranges::find(Words, std::string{"rooted"}) != Words.end();
	auto const head{std::ranges::find(Words, std::string{"head"})};
	auto const rooted{std::ranges::find(Words, std::string{"rooted"})};
	auto const sway{std::ranges::find(Words, std::string{"sway"})};
	auto const spineend{static_cast<std::size_t>(std::min({head, rooted, sway}) - Words.begin())};
	if (sway != Words.end())
	{
		auto const value{sway + 1 != Words.end() ? std::strtod((sway + 1)->c_str(), nullptr) : -1.0};
		if (sway + 1 == Words.end() || value < 0.0 || value > 1.0)
		{
			return "sway of " + Words[0] + " needs a number from 0 to 1";
		}
		bend.sway = value;
	}
	for (std::size_t word{1}; word < spineend; ++word)
	{
		auto const point{Rig.find(Words[word])};
		if (point < 0)
		{
			return "no point " + Words[word] + " for the spine of " + Words[0];
		}
		bend.spine.push_back(point);
	}
	if (bend.spine.size() < 2)
	{
		return "a bent submodel needs two points of its spine at least";
	}
	if (head != Words.end())
	{
		auto const headat{static_cast<std::size_t>(head - Words.begin())};
		if (Words.size() < headat + 3 || Words[headat + 1] == "rooted" || Words[headat + 2] == "rooted")
		{
			return "head of " + Words[0] + " needs the two points of its line";
		}
		bend.head_from = Rig.find(Words[headat + 1]);
		bend.head_to = Rig.find(Words[headat + 2]);
		if (bend.head_from < 0 || bend.head_to < 0)
		{
			return "no points " + Words[headat + 1] + " and " + Words[headat + 2] + " for the head of " + Words[0];
		}
		if (Words.size() > headat + 3 && Words[headat + 3] != "rooted" && Words[headat + 3] != "sway")
		{
			bend.head_up = Rig.find(Words[headat + 3]);
			if (bend.head_up < 0)
			{
				return "no point " + Words[headat + 3] + " over the head of " + Words[0];
			}
		}
	}
	Rig.bends.push_back(std::move(bend));
	return "";
}

std::vector<skin_bind> bind_skin(std::vector<glm::dvec3> const &Spine, std::vector<glm::dvec3> const &Points)
{
	std::vector<skin_bind> binds;
	binds.reserve(Points.size());
	for (auto const &point : Points)
	{
		skin_bind nearest;
		auto distance{std::numeric_limits<double>::max()};
		for (std::size_t piece{0}; piece + 1 < Spine.size(); ++piece)
		{
			auto const along{Spine[piece + 1] - Spine[piece]};
			auto const length2{glm::dot(along, along)};
			auto const t{length2 > 0.0 ? std::clamp(glm::dot(point - Spine[piece], along) / length2, 0.0, 1.0) : 0.0};
			auto const onspine{Spine[piece] + t * along};
			if (auto const off{glm::length(point - onspine)}; off < distance)
			{
				distance = off;
				nearest = {piece, t, point - onspine};
			}
		}
		binds.push_back(nearest);
	}
	return binds;
}

std::vector<glm::dquat> spine_turns(std::vector<glm::dvec3> const &Rest, std::vector<glm::dvec3> const &Now)
{
	// each piece takes the turn of the one before it, and then the least turn that brings it onto its line now: the twist
	// is carried along from the start, so the hose turns about itself smoothly rather than piece by piece
	std::vector<glm::dquat> pieces;
	glm::dquat carried{1.0, 0.0, 0.0, 0.0};
	for (std::size_t piece{0}; piece + 1 < Rest.size() && piece + 1 < Now.size(); ++piece)
	{
		auto const from{Rest[piece + 1] - Rest[piece]};
		auto const to{Now[piece + 1] - Now[piece]};
		if (glm::length(from) >= 1e-9 && glm::length(to) >= 1e-9)
		{
			carried = glm::normalize(glm::rotation(glm::normalize(carried * from), glm::normalize(to)) * carried);
		}
		pieces.push_back(carried);
	}
	if (pieces.empty())
	{
		return {};
	}
	std::vector<glm::dquat> nodes{pieces.front()};
	for (std::size_t node{1}; node < pieces.size(); ++node)
	{
		nodes.push_back(glm::slerp(pieces[node - 1], pieces[node], 0.5));
	}
	nodes.push_back(pieces.back());
	return nodes;
}

glm::dquat line_turn(glm::dvec3 const &FromRest, glm::dvec3 const &ToRest, glm::dvec3 const &FromNow, glm::dvec3 const &ToNow)
{
	auto const rest{ToRest - FromRest};
	auto const now{ToNow - FromNow};
	if (glm::length(rest) < 1e-9 || glm::length(now) < 1e-9)
	{
		return {1.0, 0.0, 0.0, 0.0};
	}
	return glm::rotation(glm::normalize(rest), glm::normalize(now));
}

bool behind_root(bend const &Bend, skin_bind const &Bind)
{
	return Bend.rooted && Bind.piece == 0 && Bind.along <= 0.0;
}

void hold_root(bend const &Bend, std::vector<glm::dquat> &Turns)
{
	if (Bend.rooted && false == Turns.empty())
	{
		Turns.front() = glm::dquat{1.0, 0.0, 0.0, 0.0};
	}
}

void spread_turns(std::vector<glm::dvec3> const &Rest, std::vector<glm::dquat> &Turns)
{
	if (Turns.size() < 3 || Rest.size() != Turns.size())
	{
		return;
	}
	auto total{0.0};
	for (std::size_t node{1}; node < Rest.size(); ++node)
	{
		total += glm::distance(Rest[node], Rest[node - 1]);
	}
	if (total < 1e-9)
	{
		return;
	}
	auto along{0.0};
	for (std::size_t node{1}; node + 1 < Rest.size(); ++node)
	{
		along += glm::distance(Rest[node], Rest[node - 1]);
		Turns[node] = glm::slerp(Turns.front(), Turns.back(), along / total);
	}
}

void keep_lengths(std::vector<glm::dvec3> const &Rest, std::vector<glm::dvec3> &Now)
{
	if (Rest.size() != Now.size())
	{
		return;
	}
	// the pieces as they stand now, before any node is moved
	std::vector<glm::dvec3> const was{Now};
	for (std::size_t node{1}; node < Now.size(); ++node)
	{
		auto const piece{was[node] - was[node - 1]};
		auto const length{glm::length(piece)};
		auto const made{glm::distance(Rest[node], Rest[node - 1])};
		auto const direction{length > 1e-9 ? piece / length : Rest[node] - Rest[node - 1]};
		Now[node] = Now[node - 1] + (length > 1e-9 ? made : 1.0) * direction;
	}
}

void keep_level(std::vector<glm::dvec3> const &Rest, std::vector<glm::dvec3> &Now)
{
	if (Rest.size() != Now.size())
	{
		return;
	}
	std::vector<glm::dvec3> const was{Now};
	for (std::size_t node{1}; node < Now.size(); ++node)
	{
		auto const made{Rest[node] - Rest[node - 1]};
		auto const across{glm::length(glm::dvec2{made.x, made.y})};
		// the direction of the piece now, seen from above, with the rise it has as made
		auto flat{glm::dvec2{was[node].x - was[node - 1].x, was[node].y - was[node - 1].y}};
		auto const length{glm::length(flat)};
		flat = length > 1e-9 ? flat / length : (across > 1e-9 ? glm::dvec2{made.x, made.y} / across : glm::dvec2{0.0, 1.0});
		Now[node] = Now[node - 1] + glm::dvec3{flat * across, made.z};
	}
}

glm::dquat yaw_only(glm::dquat const &Turn)
{
	// the twist of the turn about the vertical: its vector part projected onto the axis, normalised
	glm::dquat twist{Turn.w, 0.0, 0.0, Turn.z};
	auto const length{glm::length(twist)};
	if (length < 1e-9)
	{
		return glm::dquat{1.0, 0.0, 0.0, 0.0};
	}
	return twist / length;
}

skin_pose pose_skin(skin_bind const &Bind, std::vector<glm::dvec3> const &Now, std::vector<glm::dquat> const &Turns)
{
	if (Bind.piece + 1 >= Now.size() || Bind.piece + 1 >= Turns.size())
	{
		return {};
	}
	// the turn between the two nodes, the quaternions mixed and normalised: as good as a slerp for the small angles between
	// neighbouring nodes, and far cheaper for the thousands of vertices of a hose
	auto other{Turns[Bind.piece + 1]};
	if (glm::dot(Turns[Bind.piece], other) < 0.0)
	{
		other = -other;
	}
	auto const turn{glm::normalize(Turns[Bind.piece] * (1.0 - Bind.along) + other * Bind.along)};
	return {glm::mix(Now[Bind.piece], Now[Bind.piece + 1], Bind.along) + turn * Bind.offset, turn};
}

void start_chain(chain_state &State, std::vector<glm::dvec3> const &Nodes, glm::dvec3 const &Start, glm::dvec3 const &End)
{
	State.now = Nodes;
	State.before = Nodes;
	State.start = Start;
	State.end = End;
	State.start_clamp.reset();
	State.end_clamp.reset();
	State.step = 0.0;
	State.pending = 0.0;
	State.lead = glm::dvec3{0.0};
	State.quiet = 0.0;
	State.asleep = false;
	State.started = true;
	State.moving = false;
}

glm::dvec3 drawn(chain_state const &State, std::size_t const Node)
{
	return State.now[Node] + State.lead;
}

namespace {

// the nodes with the ends pinned where they are, each piece of the chain brought back towards its length: its points
// move apart or together by half the difference each, all the pieces from where the nodes were. so a chain comes out
// the same whichever end it starts from: the two vehicles a hose hangs between, each with the chain from its own end,
// get the very same one
// the two points brought towards the given distance by the share of the difference, half of it each
// the moves asked of the points of a chain in a pass, and how many constraints asked them
struct pulls
{
	std::vector<glm::dvec3> moves;
	std::vector<int> counts;
};

// the averaged move of a point taken this many times over: a point of a chain is pulled by about four constraints at
// once, so the plain average would close only a part of the gap a pass
constexpr double relaxation{2.0};

void keep_distance(std::vector<glm::dvec3> const &Points, pulls &Pulls, std::size_t const First, std::size_t const Second, double const Distance, double const Share)
{
	auto const along{Points[Second] - Points[First]};
	auto const length{glm::length(along)};
	if (length < 1e-9)
	{
		return;
	}
	auto const correction{along * (0.5 * Share * (length - Distance) / length)};
	Pulls.moves[First] += correction;
	Pulls.moves[Second] -= correction;
	++Pulls.counts[First];
	++Pulls.counts[Second];
}

// the point brought towards the given distance from a point that stays, by the share of the difference: half of it, as a
// pair of points that both move would close
void keep_distance_from(std::vector<glm::dvec3> const &Points, pulls &Pulls, std::size_t const Point, glm::dvec3 const &Anchor, double const Distance,
                        double const Share)
{
	auto const along{Points[Point] - Anchor};
	auto const length{glm::length(along)};
	if (length < 1e-9)
	{
		return;
	}
	Pulls.moves[Point] -= along * (0.5 * Share * (length - Distance) / length);
	++Pulls.counts[Point];
}

// the nodes with the ends pinned where they are, each piece of the chain brought back towards its length, and every
// other point towards its distance as made, which keeps the chain from folding: a stiff hose keeps its curve. all of it
// from where the nodes were, so a chain comes out the same whichever end it starts from: the two vehicles a hose hangs
// between, each with the chain from its own end, get the very same one. a point takes the average of what its
// constraints ask of it, not the sum: the sum overshoots, and the chain never comes to rest but shivers and stretches
// the points and the pulls of a pass, kept from one pass to the next so as not to be made anew each time
struct solver_scratch
{
	std::vector<glm::dvec3> points;
	struct pulls pulls;
};

void keep_lengths(std::vector<double> const &Lengths, std::vector<double> const &Bends, double const Bending, std::vector<glm::dvec3> &Nodes,
                  chain_ends const &Ends, solver_scratch &Scratch)
{
	auto &points{Scratch.points};
	points.clear();
	points.push_back(Ends.start);
	points.insert(points.end(), Nodes.begin(), Nodes.end());
	points.push_back(Ends.end);
	auto &pulls{Scratch.pulls};
	pulls.moves.assign(points.size(), glm::dvec3{0.0});
	pulls.counts.assign(points.size(), 0);
	for (std::size_t piece{0}; piece < Lengths.size() && piece + 1 < points.size(); ++piece)
	{
		keep_distance(points, pulls, piece, piece + 1, Lengths[piece], 1.0);
	}
	for (std::size_t bend{0}; bend < Bends.size() && bend + 2 < points.size(); ++bend)
	{
		keep_distance(points, pulls, bend, bend + 2, Bends[bend], Bending);
	}
	// a clamped end draws the node next to it to where it is as made
	if (Ends.start_clamp.has_value() && false == Nodes.empty())
	{
		keep_distance_from(points, pulls, 1, *Ends.start_clamp, 0.0, Bending);
	}
	if (Ends.end_clamp.has_value() && false == Nodes.empty())
	{
		keep_distance_from(points, pulls, Nodes.size(), *Ends.end_clamp, 0.0, Bending);
	}
	for (std::size_t node{0}; node < Nodes.size(); ++node)
	{
		auto const count{std::max(pulls.counts[node + 1], 1)};
		Nodes[node] = points[node + 1] + pulls.moves[node + 1] * (relaxation / count);
	}
}

} // namespace

namespace {

// a clamp a substep of the way from where it was to where it is, or where it is if it wasn't
std::optional<glm::dvec3> clamp_between(std::optional<glm::dvec3> const &From, std::optional<glm::dvec3> const &To, double const Along)
{
	if (false == To.has_value())
	{
		return std::nullopt;
	}
	return From.has_value() ? glm::mix(*From, *To, Along) : *To;
}

} // namespace

void simulate(std::vector<double> const &Lengths, std::vector<double> const &Bends, chain_state &State, glm::dvec3 const &Start, glm::dvec3 const &End,
              double const Deltatime, chain_physics const &Physics)
{
	simulate(Lengths, Bends, State, chain_ends{Start, End, std::nullopt, std::nullopt}, Deltatime, Physics);
}

namespace {

// the most substeps a frame; a frame longer than that many substeps takes longer ones
constexpr int max_substeps{64};
// m an end moves from where it stood to wake a chain at rest
constexpr double wake_tolerance{1.0e-4};
// m a substep under which a node or an end stands still
constexpr double still_move{2.0e-6};
// s the chain and its ends stand still before it's left at rest
constexpr double sleep_after{1.0};

// a chain at rest stays so, not simulated, while its ends stay where they were; whether it's (still) at rest
bool stays_asleep(chain_state &State, glm::dvec3 const &Start, glm::dvec3 const &End)
{
	if (false == State.asleep)
	{
		return false;
	}
	if (glm::distance(Start, State.start) < wake_tolerance && glm::distance(End, State.end) < wake_tolerance)
	{
		return true;
	}
	State.asleep = false;
	State.quiet = 0.0;
	return false;
}

// the time the chain and its ends have stood still, the last substep moving no node: after a while it's left at rest
void note_rest(chain_state &State, glm::dvec3 const &Carried, double const Time)
{
	auto moved{glm::length(Carried)};
	for (std::size_t node{0}; node < State.now.size(); ++node)
	{
		moved = std::max(moved, glm::distance(State.now[node], State.before[node]));
	}
	if (moved >= still_move)
	{
		State.quiet = 0.0;
		return;
	}
	State.quiet += Time;
	State.asleep = State.quiet > sleep_after;
}

} // namespace

void simulate(std::vector<double> const &Lengths, std::vector<double> const &Bends, chain_state &State, chain_ends const &Ends, double const Deltatime,
              chain_physics const &Physics)
{
	if (Deltatime <= 0.0 || false == State.started || stays_asleep(State, Ends.start, Ends.end))
	{
		return;
	}
	auto const &Start{Ends.start};
	auto const &End{Ends.end};
	auto const fromstart{State.start};
	auto const fromend{State.end};
	// whole substeps of the time given and what was left over before, the rest left for the next frame
	auto const total{State.pending + Deltatime};
	auto steps{static_cast<int>(std::floor(total / Physics.step))};
	auto step{Physics.step};
	if (steps > max_substeps)
	{
		steps = max_substeps;
		step = total / steps;
	}
	if (steps == 0)
	{
		State.pending = total;
		State.lead = 0.5 * ((Start - fromstart) + (End - fromend));
		return;
	}
	// the ends at the time of the last substep: that far on their way from where they were to where they are
	auto const reach{steps * step / total};
	auto const tostart{glm::mix(fromstart, Start, reach)};
	auto const toend{glm::mix(fromend, End, reach)};
	auto const startclamp{clamp_between(State.start_clamp, Ends.start_clamp, reach)};
	auto const endclamp{clamp_between(State.end_clamp, Ends.end_clamp, reach)};
	auto const keep{std::exp(-Physics.damping * step)};
	// the move of the ends a substep: the hose goes along with the vehicles, only its own swing against them dies down
	auto const carried{0.5 * ((tostart - fromstart) + (toend - fromend)) / static_cast<double>(steps)};
	if (false == State.moving)
	{
		// a chain started where made goes along with its ends from the first step, rather than being left behind by a
		// vehicle already running
		for (std::size_t node{0}; node < State.now.size(); ++node)
		{
			State.before[node] = State.now[node] - carried;
		}
		State.step = step;
		State.moving = true;
	}
	// the move of the last substep, for a substep of this length: only a frame longer than the substeps allowed takes
	// longer ones, and the move taken as it was would speed up or slow down the nodes against their ends with the change
	auto scale{State.step > 0.0 ? step / State.step : 1.0};
	solver_scratch scratch;
	for (auto substep{1}; substep <= steps; ++substep)
	{
		auto const along{static_cast<double>(substep) / steps};
		chain_ends const ends{glm::mix(fromstart, tostart, along), glm::mix(fromend, toend, along), clamp_between(State.start_clamp, startclamp, along),
		                      clamp_between(State.end_clamp, endclamp, along)};
		for (std::size_t node{0}; node < State.now.size(); ++node)
		{
			auto const velocity{carried + ((State.now[node] - State.before[node]) * scale - carried) * keep};
			State.before[node] = State.now[node];
			State.now[node] += velocity + Physics.gravity * (step * step);
		}
		scale = 1.0;
		for (auto iteration{0}; iteration < Physics.iterations; ++iteration)
		{
			keep_lengths(Lengths, Bends, Physics.bending, State.now, ends, scratch);
		}
	}
	State.start = tostart;
	State.end = toend;
	State.start_clamp = startclamp;
	State.end_clamp = endclamp;
	State.step = step;
	State.pending = std::max(0.0, total - steps * step);
	State.lead = 0.5 * ((Start - tostart) + (End - toend));
	note_rest(State, carried, steps * step);
}

void simulate(std::vector<double> const &Lengths, chain_state &State, glm::dvec3 const &Start, glm::dvec3 const &End, double const Deltatime,
              chain_physics const &Physics)
{
	simulate(Lengths, {}, State, Start, End, Deltatime, Physics);
}

glm::dvec3 crank(glm::dvec3 const &Rest, wheel_turn const &Wheel)
{
	auto const turn{glm::angleAxis(glm::radians(Wheel.angle), glm::normalize(Wheel.axis))};
	return Wheel.centre + turn * (Rest - Wheel.centre);
}

glm::dvec3 slide(glm::dvec3 const &Rest, glm::dvec3 const &Guide, glm::dvec3 const &DriverRest, glm::dvec3 const &DriverNow)
{
	// |Rest + s Guide - DriverNow| = |Rest - DriverRest|: s² + 2 b s + c = 0
	auto const length2{glm::dot(Rest - DriverRest, Rest - DriverRest)};
	auto const offset{Rest - DriverNow};
	auto const b{glm::dot(Guide, offset)};
	auto const c{glm::dot(offset, offset) - length2};
	auto const discriminant{b * b - c};
	if (discriminant < 0.0)
	{
		// out of reach of the rod: as close as the guide comes
		return Rest - b * Guide;
	}
	// the root nearer the position as made
	auto const root{std::sqrt(discriminant)};
	auto const shift{std::abs(-b + root) < std::abs(-b - root) ? -b + root : -b - root};
	return Rest + shift * Guide;
}

std::vector<glm::dvec3> pose(rig const &Rig, drive const &Drive)
{
	std::vector<glm::dvec3> positions;
	positions.reserve(Rig.points.size());
	for (std::size_t index{0}; index < Rig.points.size(); ++index)
	{
		auto const &point{Rig.points[index]};
		auto position{point.rest};
		if (point.kind == point_kind::crank && index < Drive.wheels.size())
		{
			position = crank(point.rest, Drive.wheels[index]);
		}
		else if (point.kind == point_kind::slide)
		{
			auto const &driver{Rig.points[point.driver]};
			position = slide(point.rest, point.guide, driver.rest, positions[point.driver]);
		}
		else if (point.kind == point_kind::coupler && index < Drive.couplers.size() && Drive.couplers[index])
		{
			position = point.rest + *Drive.couplers[index];
		}
		else if (point.kind == point_kind::middle)
		{
			position = 0.5 * (positions[point.driver] + positions[point.second]);
		}
		positions.push_back(position);
	}
	return positions;
}

motion bone_motion(glm::dvec3 const &AnchorRest, glm::dvec3 const &AnchorNow, std::optional<glm::dvec3> const &AimRest, std::optional<glm::dvec3> const &AimNow)
{
	motion motion{glm::dmat3{1.0}, AnchorRest, AnchorNow};
	if (false == AimRest.has_value() || false == AimNow.has_value())
	{
		return motion;
	}
	auto const from{*AimRest - AnchorRest};
	auto const to{*AimNow - AnchorNow};
	if (glm::length(from) < 1e-6 || glm::length(to) < 1e-6)
	{
		return motion;
	}
	motion.rotation = glm::mat3_cast(glm::rotation(glm::normalize(from), glm::normalize(to)));
	return motion;
}

animation localise(motion const &Motion, glm::dvec3 const &Origin, glm::dmat3 const &Rotation)
{
	// a point of the submodel x = Origin + Rotation p goes to AnchorNow + R (x - AnchorRest), which is
	// Origin + Rotation (t + Q p) with Q = Rotation^T R Rotation and t = Rotation^T (AnchorNow + R (Origin - AnchorRest) - Origin)
	auto const inverse{glm::transpose(Rotation)};
	auto const shift{Motion.anchor_now + Motion.rotation * (Origin - Motion.anchor_rest) - Origin};
	auto const local{inverse * Motion.rotation * Rotation};
	animation result;
	result.translation = glm::vec3(inverse * shift);
	auto x{0.0};
	auto y{0.0};
	auto z{0.0};
	glm::extractEulerAngleXYZ(glm::dmat4(local), x, y, z);
	result.angles = glm::vec3(glm::degrees(x), glm::degrees(y), glm::degrees(z));
	result.matrix = glm::translate(glm::mat4(1.f), result.translation) * glm::mat4(glm::mat3(local));
	return result;
}

glm::dmat3 model_file_frame()
{
	return glm::dmat3{{-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}};
}

} // namespace bones
