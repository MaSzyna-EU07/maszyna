/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editortool.h"
#include "editor/editorKilometrage.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

// the kilometrage of the lines: the points of a line where its km is known and where it jumps, the km of the rest from them
class km_tool final : public editor_tool
{
  public:
	km_tool(editor_selection &Selection, editor_view &View, editor_history &History, editor_templates &Templates)
	    : m_selection{Selection}, m_view{View}, m_history{History}, m_templates{Templates}
	{
	}
	void update() override;
	void draw_overlay() const override;
	void render_options() override;
	void forget(scene::basic_node const *Node) override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	void set_open(bool const Open) { m_open = Open; }
	// the point of the track clicked: a new point of the km there, or the one standing by it
	void pick(TTrack &Track, glm::dvec3 const &Point);
	// the km of the lines at the point, a line of text each, empty where none reaches
	std::string readout(glm::dvec3 const &Point) const;

  private:
	editor_selection &m_selection;
	editor_view &m_view;
	editor_history &m_history;
	editor_templates &m_templates;
	bool m_open{false};
	struct picked
	{
		TTrack *track{nullptr};
		glm::dvec3 point{0.0}; // on the axis of the track
		glm::dvec2 along{0.0, 1.0}; // of the track there, in the plan
	};
	std::optional<picked> m_pick;
	int m_edited{-1}; // of the marks, the one standing by the point picked, -1 for a new one
	std::string m_line;
	double m_km{0.0};
	bool m_jump{false};
	double m_beyond{0.0};
	bool m_against{false}; // the km rise against the track picked
	std::string m_status;
	// what the overlay draws, made again when the axes change
	struct tick
	{
		glm::dvec3 at{0.0};
		glm::dvec3 across{0.0}; // in the plan, a unit
		int hectometres{0};
		bool jump{false};
	};
	struct drawn
	{
		std::string line;
		std::vector<glm::dvec3> points;
		std::vector<char> guessed; // of the points, beyond the last point of the km: the km run on from it
		std::vector<tick> ticks;
	};
	std::vector<drawn> m_drawn;
	std::size_t m_revision{0};
	bool m_drawn_once{false};

	void refresh();
	void put();
	void remove();
	// the point of the km picked and the camera over it
	void show(std::size_t const Index);
	// the hectometre posts standing along the line through the track picked read into points of the km: the first and the last
	// post of each run of them numbered by 100 m, a jump where the runs meet; one step of the history
	void read_posts();
	// the way the km rise at the point picked, in the plan
	glm::dvec2 rising() const;
	// warnings of a jump at the point: Id-1 § 19 wants none on a curve nor at a station
	std::string jump_warning(glm::dvec3 const &Point) const;
};
