/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutNetwork.hpp"

#include <optional>
#include <string>
#include <variant>
#include <vector>

// The edits of the track geometry (editor-tracks-domain.md §4, "Edycja = polecenie"; §8): each one is a command which proposes a new
// state of the lines, the nodes and the turnouts it touches; what depends on them is laid again, the topology, the rules of the turnouts
// and the norms are checked (the module of the norms: BLOCK refuses, WARN and PREFER are told), and the change is kept whole or refused
// with the reasons, the state left as it was. Each one kept is one step to undo, what it touched restored as it was.
namespace layout
{

// a line laid between two new nodes, its vertices given with their radii, transitions and cants (the ids are given to them)
struct create_line
{
	glm::dvec3 start{0.0};
	glm::dvec3 end{0.0};
	std::vector<vertex> vertices;
	std::vector<grade_break> grades;
	transition_shape transition{transition_shape::clothoid};
	ramp_shape ramp{ramp_shape::linear};
	norms::line_class category;
	double chainage_start{0.0};
};

// the vertex moved in the plan
struct move_vertex
{
	line_id line{line_id::none};
	vertex_id vertex{vertex_id::none};
	glm::dvec2 point{0.0};
};

// the radius, the transitions or the cant of the vertex, those given
struct set_vertex
{
	line_id line{line_id::none};
	vertex_id vertex{vertex_id::none};
	std::optional<double> radius;
	std::optional<double> transition_in;
	std::optional<double> transition_out;
	std::optional<double> cant;
};

// the line cut at the chainage by a new node: the part before it keeps the line, the part after it is a new line; the track stays as
// it was. Inside a curve the parts go on from the node at its curvature and cant; not inside a vertical curve, nor inside a transition
// curve which isn't a clothoid with a straight cant ramp, whose parts aren't curves of their kind
struct split_line
{
	line_id line{line_id::none};
	double chainage{0.0};
};

// a new line laid between two free ends, solved between the nodes: a straight where they face each other on one, a curve where their
// tangents meet, two curves otherwise. The radius 0 takes the largest the norms prefer which fits, the transition and the cant below 0
// the ones the norms prefer for the radius
struct join_ends
{
	node_id from{node_id::none};
	node_id to{node_id::none};
	double radius{0.0};
	double transition{-1.0};
	double cant{-1.0};
};

// the line taken away; a node left with no end goes with it
struct delete_line
{
	line_id line{line_id::none};
};

// a turnout of the type of the catalogue laid into the line, its start at the chainage: the line cut at its legs, the part between them its
// main track, its diverging track leaving it to the hand (1 left, -1 right) as it runs from its start, along the line (direction 1) or
// against it (-1); bent as the line under it is. Refused with the reasons where a rule of the turnouts or a BLOCK limit of the norms is broken
struct insert_turnout
{
	line_id line{line_id::none};
	double chainage{0.0};
	std::string type;
	int hand{1};
	int direction{1};
};

// the turnout moved along its line by the distance, above 0 toward the end of its main track: the lines on both sides of it one line again
// and cut at its legs anew, the line going on from the end of its diverging track carried with it there; checked as when laid
struct move_turnout
{
	turnout_id turnout{turnout_id::none};
	double distance{0.0};
};

// the turnout of another type of the catalogue, its start where it is; the hand changed where given
struct change_turnout
{
	turnout_id turnout{turnout_id::none};
	std::string type;
	std::optional<int> hand;
};

// the turnout taken out: its diverging track goes, the end of it with it where no line goes on from there; its main track and the lines
// on both sides of it laid as one line again where they can be
struct remove_turnout
{
	turnout_id turnout{turnout_id::none};
};

using command = std::variant<create_line, move_vertex, set_vertex, split_line, join_ends, delete_line, insert_turnout, move_turnout, change_turnout, remove_turnout>;

char const *command_name(command const &Command);

struct outcome
{
	bool done{false};
	std::vector<std::string> reasons; // why it was refused
	std::vector<notice> notices; // what the norms warn of, or would prefer otherwise, on the lines changed
	std::vector<line_id> lines; // made by it
	std::vector<node_id> nodes;
	std::vector<turnout_id> turnouts;
};

// the lines and the nodes with the commands which edited them, to undo and redo one by one
class model
{
  public:
	model() = default;
	explicit model(norms::area_map Areas) { m_network.areas = std::move(Areas); }

	network const &state() const { return m_network; }
	outcome execute(command const &Command);
	bool undo();
	bool redo();
	std::size_t undoable() const { return m_undo.size(); }
	std::size_t redoable() const { return m_redo.size(); }

  private:
	struct step
	{
		std::string name;
		snapshot before;
		snapshot after;
	};
	network m_network;
	std::vector<step> m_undo;
	std::vector<step> m_redo;
};

} // namespace layout
