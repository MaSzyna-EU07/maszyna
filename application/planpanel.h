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

#include "editor/orthophoto.h"
#include "maj0sted/app/basket_draft.hpp"
#include "maj0sted/app/editor_document.hpp"
#include "maj0sted/app/straight_constraints.hpp"
#include "maj0sted/io/scn_export.hpp"
#include "maj0sted/web/editor.hpp"
#include "maj0sted/web/ribbon.hpp"

#include <vector>

// track layout tool: straights laid down by hand on the scenery itself and the curves fitted into
// the gaps between them. the panel holds only the controls - the drawing happens in the viewport,
// which the tool puts into a top-down orthographic plan view. none of the geometry is computed
// here: maj0sted solves the whole document and the panel paints what comes back.
class plan_panel : public ui_panel
{

  public:
	plan_panel(std::string const &Name, bool const Isopen);

	void update() override;
	void render_contents() override;

  private:
	// methods
	void render_toolbar();
	void render_draft();
	void render_gaps();
	void render_compound(maj0sted::web::GapFit &Fit, std::size_t const Gap, bool &Dirty);
	void fit_compound_to_guides();
	void clear_fit_guides();
	void render_storage();
	void export_scn();
	void render_newmap_dialog();
	void render_location_dialog();
	void start_map(bool const Georeferenced, double const Originx, double const Originy);
	void handle_scene();
	void draw_on_scene();
	void solve();
	void apply_constraints();
	maj0sted::web::NiweletaSpec *current_niweleta();
	maj0sted::web::NiweletaSpec const *current_niweleta() const;
	void align_fits();
	bool fit_applied(std::size_t const Gap) const;
	// independent anchored straights: each endpoint is its own handle
	maj0sted::web::StraightSpec const *handle_straight(std::size_t const Niw, std::size_t const Index) const;
	void append_vertex(double const X, double const Y);
	void drop_last_vertex();
	void delete_selected_straight();
	void delete_current_niweleta();
	void clear_selection();
	// hit tests (screen pixels). niw=-1 means "current niweleta only"
	bool hit_endpoint(ImVec2 const &Mouse, int &OutNiw, int &OutStr, int &OutEnd) const;
	bool hit_straight(ImVec2 const &Mouse, int &OutNiw, int &OutStr, bool const AllNiwelety) const;
	// compound basket: draggable ends of entry/exit KP, free arcs and between-KP
	bool hit_compound_station(ImVec2 const &Mouse, int &OutGap, int &OutKind, int &OutArc) const;
	void drag_compound_station(double const X, double const Y);
	// arc-first draft: continue from a straight end, click appends luk/KP by length
	void begin_draft();
	void discard_draft();
	void start_draft_from_end(std::size_t const Str, int const End);
	void append_draft_at(double const X, double const Y);
	void undo_draft_fragment();
	bool finish_draft();
	bool reopen_draft_from_gap(std::size_t const Gap);
	void rebuild_draft_preview();
	bool draft_tip(double &OutX, double &OutY) const;
	bool draft_end_pose(double &OutX, double &OutY, double &OutAz) const;
	bool hit_draft_station(ImVec2 const &Mouse, int &OutKind, int &OutArc) const;
	void drag_draft_station(double const X, double const Y);
	bool anchor_draft_to_gap(std::size_t const Gap);
	// switches (rozjazdy): a branch leaves a through niweleta without splitting it. clicking the
	// through track drops a switch there and starts a branch niweleta pinned to its frog
	void add_switch_on(std::size_t const Through, double const Wx, double const Wy);
	void render_switches();
	void go_to_plan();

	// members
	maj0sted::app::EditorDocument m_document;
	std::vector<maj0sted::web::NiweletaPolys> m_solved;
	std::vector<maj0sted::web::JunctionGeom> m_junctions;
	std::size_t m_niweleta{0};

	// switch placement: 0 idle, 1 waiting for a click on the through track
	int m_pick_switch{0};
	int m_sw_side{0};          // 0 left, 1 right
	double m_sw_crossing{9.0};  // skos 1:n
	double m_sw_radius{190.0};  // internal arc radius (metres)

	int m_sel_straight{-1};
	int m_sel_end{-1};
	bool m_dragging{false};
	// captured at drag start so Shift keeps the angle while the length changes
	double m_drag_fixed_x{0.0};
	double m_drag_fixed_y{0.0};
	double m_drag_ux{1.0};
	double m_drag_uy{0.0};

	// compound station drag: kind 0=entry KP end, 1=free-arc end, 2=between-KP end, 3=exit KP start
	bool m_dragging_cmp{false};
	int m_cmp_gap{-1};
	int m_cmp_kind{-1};
	int m_cmp_arc{-1};

	bool m_drawing{true};
	bool m_pending{false};
	double m_pendingx{0.0};
	double m_pendingy{0.0};

	// 0 = idle, 1 = picking parallel reference, 2 = picking skew reference
	int m_pick_rel{0};
	// after a reference is chosen, ask for the numeric parameter
	bool m_ask_rel_param{false};
	double m_ask_rel_value{4.5};
	// compound: gap being edited; pick mode accumulates guide points, then Fit runs the solver
	int m_sel_gap{-1};
	bool m_pick_compound_point{false};
	struct FitGuide
	{
		double x{0.0};
		double y{0.0};
	};
	std::vector<FitGuide> m_fit_guides;

	maj0sted::app::BasketDraft m_draft;
	std::vector<maj0sted::web::WebPolyline> m_draft_polys;
	// 0 idle, 1 pick straight end to continue from
	int m_draft_place{0};
	// next click appends: 0 = luk, 1 = KP (wej/między), 2 = KP wyjście (kończy: prosta + kotwica)
	int m_draft_append_kind{0};
	double m_draft_r{300.0};
	double m_draft_finish_straight{100.0};
	// 0 idle, 1 click a straight to choose the gap to anchor into
	int m_anchor_pick{0};
	bool m_dragging_draft{false};
	int m_draft_kind{-1};
	int m_draft_arc{-1};

	char m_namebuf[128]{};

	std::string m_status;
	char m_path[256]{"editor/plan.m0s"};
	char m_scn_path[256]{"scenery/plan_export.scn"};

	bool m_pickingplace{false};
	double m_pickx{0.0};
	double m_picky{0.0};
	double m_mapviewx{0.0};
	double m_mapviewy{0.0};
	double m_mapscale{0.0};

	editor::wms_image m_topomap{maj0sted::web::WmsConfig::geoportal_topo()};
	editor::orthophoto_source m_ortho;
	// 1 km topo cells for views wider than a kilometre (orto's 100 m grid is too dense there)
	editor::orthophoto_source m_topo{maj0sted::web::WmsConfig::geoportal_topo(), 1000, "topo"};
	bool m_showortho{true};
};
