/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/uilayer.h"

#include "maj0sted/app/editor_document.hpp"
#include "maj0sted/web/editor.hpp"

// track layout tool: straights laid down by hand on the scenery itself and the curves fitted into
// the gaps between them. the panel holds only the controls - the drawing happens in the viewport,
// which the tool puts into a top-down orthographic view for the purpose. none of the geometry is
// computed here: maj0sted solves the whole document and the panel paints what comes back.
class plan_panel : public ui_panel
{

  public:
	plan_panel(std::string const &Name, bool const Isopen);

	void update() override;
	void render_contents() override;

  private:
	// methods
	void render_toolbar();
	void render_gaps();
	void render_storage();
	// paints the solved plan over the rendered scenery and takes the clicks that build it
	void handle_scene();
	void draw_on_scene();
	// re-runs the solver over the whole document; cheap enough to do on every edit
	void solve();
	maj0sted::web::NiweletaSpec *current_niweleta();
	maj0sted::web::NiweletaSpec const *current_niweleta() const;
	// keeps exactly one fit slot per gap, so a gap's parameters survive edits to its neighbours
	void align_fits();
	// true when the solver actually produced the fit requested for the given gap
	bool fit_applied(std::size_t const Gap) const;
	// the panel edits a chained polyline, so a vertex is shared by the two straights meeting at it.
	// the library allows fully independent straights; that freedom just isn't exposed here yet
	std::size_t vertex_count() const;
	void vertex_position(std::size_t const Index, double &X, double &Y) const;
	void append_vertex(double const X, double const Y);
	void move_vertex(std::size_t const Index, double const X, double const Y);
	void drop_last_vertex();
	// takes the camera to the middle of the drawing
	void go_to_plan();
	// members
	maj0sted::app::EditorDocument m_document;
	std::vector<maj0sted::web::NiweletaPolys> m_solved;
	std::size_t m_niweleta{0}; // niweleta being drawn into
	int m_draggedvertex{-1};
	bool m_drawing{true}; // whether a click on the scenery lays track or only picks existing points
	// first corner of a niweleta has nothing to attach to, so it waits for the second click
	bool m_pending{false};
	double m_pendingx{0.0};
	double m_pendingy{0.0};
	std::string m_status;
	char m_path[256]{"editor/plan.m0s"};
};
