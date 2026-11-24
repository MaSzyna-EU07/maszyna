/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutLine.hpp"
#include "editor/norms/norms.hpp"

#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>
#include <glm/glm.hpp>

// The lines and the nodes of the track geometry (editor-tracks-domain.md §4): the aggregates, their invariants and the checks of them.
// A node is the point the ends of the lines meet at: its place, the heading, the curvature, the cant and the grade there, and the ends
// meeting in it. The continuity of the track is the invariant of the node: the ends of its lines have its place by construction (a line
// has no coordinates of its own at its ends), and its heading, its cant and its grade within what the norms let pass; the curvature
// may change there only by a step of the cant deficiency the profile allows. The legs of a turnout are nodes too: at its start the main and
// the diverging track meet the line it lies on, so a node holds two ends, or three at the start of a turnout.
namespace layout
{

enum class line_end
{
	start,
	end
};

// an end of a line meeting at a node; the sense tells whether the line runs from there along the heading of the node or against it
struct end_ref
{
	line_id line{line_id::none};
	line_end end{line_end::start};
	int sense{1};

	bool operator==(end_ref const &) const = default;
	// the side of the node the line lies on: 1 ahead of it, along its heading, -1 behind it
	int side() const { return end == line_end::start ? sense : -sense; }
};

struct node
{
	node_id id{node_id::none};
	glm::dvec3 position{0.0}; // the scenery's x, the height, z
	glm::dvec2 heading{0.0, 1.0}; // plan, unit
	double curvature{0.0}; // 1/m along the heading, above 0 turning left
	double cant{0.0}; // mm along the heading, above 0 leaning left
	double grade{0.0}; // along the heading, above 0 rising
	std::vector<end_ref> ends;

	bool operator==(node const &) const = default;
};

struct line
{
	line_id id{line_id::none};
	turnout_id turnout{turnout_id::none}; // the turnout whose track it is, none for a line of its own
	node_id start{node_id::none};
	node_id end{node_id::none};
	line_shape shape;
	// the curve next to the end goes on from the curve the node holds, beginning or ending right at it: a line split inside a curve
	bool start_continues{false};
	bool end_continues{false};
	double chainage_start{0.0}; // m, the kilometrage of the start of the line
	norms::line_class category;
	std::optional<norms::profile_id> profile; // of the norms, its own; none: the one of the area it lies in
	// derived: the line laid between its nodes
	solved_line solved;

	bool operator==(line const &) const = default;
	node_id node_at(line_end const End) const { return End == line_end::start ? start : end; }
	bool continues(line_end const End) const { return End == line_end::start ? start_continues : end_continues; }
};

// a turnout (editor-tracks-domain.md §2, §4 "Rozjazd", §7): a type of the catalogue laid into a line, its hand and its direction along the
// line, bent as the line under it is (table 11.8), its speeds computed. Its main track is the part of the line it lies on and its diverging
// track leaves it at its start, each a line the turnout owns; its legs, the start and the ends of the two tracks, are nodes
struct turnout
{
	turnout_id id{turnout_id::none};
	std::string type; // of the catalogue, "Rz 60E1-500-1:12"
	int hand{1}; // the side the diverging track leaves to as the turnout runs from its start: 1 left, -1 right
	int direction{1}; // 1: its main track runs as the line it was laid into does, from the start of the turnout; -1 against it
	node_id start{node_id::none};
	node_id main_end{node_id::none};
	node_id diverging_end{node_id::none};
	line_id main{line_id::none};
	line_id diverging{line_id::none};
	// derived from the line under it
	double chainage{0.0}; // m, the kilometrage of its start
	double main_radius{0.0}; // m, of the main track, signed as table 11.8 takes it: above 0 bent the way the diverging track leaves, 0 straight
	double diverging_radius{0.0}; // m, of the diverging curve bent by table 11.8, signed: above 0 turning to the hand, below 0 the other way
	double cant{0.0}; // mm, the same all over it, above 0 leaning to the side the diverging track leaves to
	double speed_main{0.0}; // km/h
	double speed_diverging{0.0}; // km/h
	double velocity{0.0}; // as the track has it written: the limit of the main track for both, or the diverging one for it alone below 0
	bool movable{false}; // the frog with a movable nose, of the type

	bool operator==(turnout const &) const = default;
	bool bent() const { return main_radius != 0.0; }
};

class transaction;

// the lines and the nodes of a scenery, with the profiles of the norms by place; identifiers given once and never again
class network
{
  public:
	node const *find(node_id const Id) const;
	line const *find(line_id const Id) const;
	turnout const *find(turnout_id const Id) const;
	std::map<node_id, node> const &nodes() const { return m_nodes; }
	std::map<line_id, line> const &lines() const { return m_lines; }
	std::map<turnout_id, turnout> const &turnouts() const { return m_turnouts; }
	// the same lines, nodes and turnouts: the identifiers handed out so far aren't compared
	bool same(network const &Other) const { return m_nodes == Other.m_nodes && m_lines == Other.m_lines && m_turnouts == Other.m_turnouts; }

	norms::area_map areas; // the profiles of the norms by place

	node_id new_node() { return static_cast<node_id>(++m_last_node); }
	line_id new_line() { return static_cast<line_id>(++m_last_line); }
	vertex_id new_vertex() { return static_cast<vertex_id>(++m_last_vertex); }
	turnout_id new_turnout() { return static_cast<turnout_id>(++m_last_turnout); }

	// the end of the line meeting at its node, as the node keeps it
	end_ref const *end_of(line_id const Line, line_end const End) const;
	// how the end of the line meets its node, for laying the line
	end_condition condition(line const &Line, line_end const End) const;

	// the last identifiers handed out, which the project file keeps so that none is handed out again after it's read
	struct handed_out
	{
		std::uint32_t node{0};
		std::uint32_t line{0};
		std::uint32_t vertex{0};
		std::uint32_t turnout{0};

		bool operator==(handed_out const &) const = default;
	};
	handed_out handed() const { return {m_last_node, m_last_line, m_last_vertex, m_last_turnout}; }
	// the identifiers up to these taken as handed out; none goes back below what was handed out already
	void hand_out(handed_out const &Last)
	{
		m_last_node = std::max(m_last_node, Last.node);
		m_last_line = std::max(m_last_line, Last.line);
		m_last_vertex = std::max(m_last_vertex, Last.vertex);
		m_last_turnout = std::max(m_last_turnout, Last.turnout);
	}

  private:
	friend class transaction;
	std::map<node_id, node> m_nodes;
	std::map<line_id, line> m_lines;
	std::map<turnout_id, turnout> m_turnouts;
	std::uint32_t m_last_node{0};
	std::uint32_t m_last_line{0};
	std::uint32_t m_last_vertex{0};
	std::uint32_t m_last_turnout{0};
};

// the lines and nodes as they were before a change, or are after it: none where one didn't exist
struct snapshot
{
	std::map<line_id, std::optional<line>> lines;
	std::map<node_id, std::optional<node>> nodes;
	std::map<turnout_id, std::optional<turnout>> turnouts;
};

// a change of the network in the making: each line and node it touches is kept as it was the first time, so the change can be taken
// back whole, or recorded for undoing and redoing
class transaction
{
  public:
	explicit transaction(network &Network) : m_network{Network} {}
	network const &view() const { return m_network; }
	network &ids() { return m_network; }

	line &change(line_id const Id);
	node &change(node_id const Id);
	turnout &change(turnout_id const Id);
	line &add(line Line);
	node &add(node Node);
	turnout &add(turnout Turnout);
	void erase(line_id const Id);
	void erase(node_id const Id);
	void erase(turnout_id const Id);

	std::set<line_id> touched_lines() const;
	std::set<node_id> touched_nodes() const;
	std::set<turnout_id> touched_turnouts() const;
	snapshot before() const { return m_before; }
	snapshot after() const;
	// everything as it was before the transaction
	void roll_back();

  private:
	network &m_network;
	snapshot m_before;
};

// puts a snapshot in place: the lines and the nodes it has as it has them
void restore(network &Network, snapshot const &State);

// the lines touched laid again between their nodes, and what depends on them: the nodes take the heading, the curvature, the cant and the
// grade of the ends they hold (a node at a free end follows its line), and a line going on from a curve at a node whose curve changed is
// laid again; a turnout whose main track or start changed lays its diverging track again, its end node where the track ends now, and
// computes its bending and its speeds
void settle(transaction &Change);

// the turnout owning the line, none for a line of its own
turnout const *owner_of(network const &Network, line_id const Line);
// the end is the start of the diverging track of a turnout, which runs on into no line through its node and tells it nothing
bool diverging_start(network const &Network, end_ref const &End);

// a line running on through the nodes of two ends: the lines in their order, each forward or backward
struct chain_part
{
	line_id line{line_id::none};
	bool forward{true};
};
std::vector<chain_part> chain_of(network const &Network, line_id const Line);
// the chain described as one line for the norms: the elements running on alike across the nodes as one, the ends open
struct described_chain
{
	norms::line line;
	std::vector<double> starts; // the chainage each part starts at
};
described_chain describe_chain(network const &Network, std::vector<chain_part> const &Chain);

// what the norms found on a line changed, or the rules of a turnout on it
struct notice
{
	line_id line{line_id::none};
	double chainage{0.0}; // along the line
	norms::finding finding; // of a rule of a turnout: no parameter (norms::parameter::count), the tier
	std::string text;
	turnout_id turnout{turnout_id::none}; // a rule of the turnout
	std::vector<glm::dvec2> along; // a BLOCK finding: the element it's on, as points of the plan 2 m apart at most
};

// the BLOCK finding Found is the one Old was: a rule of the same turnout telling the same, or a limit of the same parameter passed on the
// same stretch of the track (their elements within a metre and a half of each other), by no more
bool same_finding(notice const &Old, notice const &Found);

struct verdict
{
	std::vector<std::string> reasons; // what breaks an invariant
	std::vector<notice> blocks; // the BLOCK limits of the norms and the hard rules of the turnouts passed
	std::vector<notice> notices; // WARN and PREFER
	bool fine() const { return reasons.empty() && blocks.empty(); }
	// the reasons and the BLOCK limits, as a refusal tells them
	std::vector<std::string> refusals() const;
};
// the invariants of the touched lines, nodes and turnouts, the norms along the chains through the touched lines, the rules of the turnouts on
// them or at the nodes
verdict check(network const &Network, std::set<line_id> const &Lines, std::set<node_id> const &Nodes, std::set<turnout_id> const &Turnouts = {});
// the same over all of the network
verdict check_all(network const &Network);

// "cant deficiency 162 mm, the limit 150 mm (ST-T1-A6 tab. 9.4)"
std::string finding_text(norms::finding const &Finding);

} // namespace layout
