/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// The catenary tool of the editor (sieć trakcyjna): its state and workings. The editor (editormode.h) holds it behind a
// pointer and calls it through a few fixed hooks, so a change here rebuilds only editormodecatenary.cpp, not the files
// which include editormode.h. The tool is a friend of editor_mode and reaches the editor through `e`.

#pragma once

#include "application/editormode.h"
#include "world/catenary_suspension.h"
#include "world/catenary_cantilever.h"

#include <map>
#include <optional>
#include <string>
#include <vector>

struct catenary_editor
{
	explicit catenary_editor(editor_mode &Editor) : e{Editor} {}
	editor_mode &e;
	using track_intent = editor_mode::track_intent;
	using key_hint = editor_mode::key_hint;
	// what a catenary intent (track_intent::kind::catenary) is about, in its detail
	enum class intent_detail { place, select, chain, gantry, mast };
	static bool is_intent(track_intent const &Intent, intent_detail const Detail)
	{
		return Intent.what == track_intent::kind::catenary && Intent.detail == static_cast<int>(Detail);
	}

	// catenary: poles with cantilevers by the track, put where the user sees them (on the orthophoto), the cantilever card
	// chosen from the catalogue by the distance from the face of the pole; spans checked against the catalogue tables
	struct catenary_spot
	{
		TTrack *track{nullptr};
		int path{0};
		double parameter{0.0};
		glm::dvec3 axis{0.0}; // the point of the path, under the rail heads
		glm::dvec2 forward{0.0, 1.0};
		int side{1}; // the pole on the right (+1) or on the left (-1) of the direction of the path
		double radius{0.0}; // of the track there, 0 on a straight
		bool curve{false};
		bool toward{true}; // the wires are offset towards the pole
		double reach{0.0}; // A: from the wires to the face of the pole
		int card{0};
		std::string problem; // why there's no card, empty when there is one
		std::string family; // the family of the card in the described catalogue; empty without it
		bool execution_changed{false}; // the execution chosen in the panel has no card for this A, another one was taken
		bool switch_error{false}; // a pole of a group that doesn't stand by switches (concrete), standing by one
		catenary::suspension data;
	};
	struct catenary_change
	{
		int group{0}; // changes of one action are taken back together
		bool chain{false}; // a change of a chain, otherwise of a suspension
		std::size_t index{0};
		std::optional<catenary::suspension> before; // none: the suspension was added
		std::optional<catenary::chain> chain_before; // none: the chain was added
		bool removed{false}; // the suspension was removed
		bool gantry{false}; // a change of a gantry
		std::optional<catenary::gantry> gantry_before; // none: the gantry was added
		bool anchor{false}; // a change of an anchor
		std::optional<catenary::anchor> anchor_before; // none: the anchor was added
	};
	struct catenary_tool
	{
		std::vector<std::string> poles;
		std::vector<std::string> networks;
		int pole{0};
		int network{0};
		int zone{1};
		int execution{1};
		bool swinging{true};
		bool toward{true};
		bool alternate{true}; // on a straight the next one goes the other way
		int bend{0}; // 0: from the track, 1: straight, 2: curve
		float stagger{0.30f};
		float clearance{2.50f}; // from the axis of the track to the face of the pole
		float height{5.60f}; // of the contact wires over the rails
		float system_height{0.0f}; // hk: of the messenger over the contact wires; 0 = the card's
		bool insulators25{false};
		char file[128]{};
		int selected{-1};
		// chains: the poles placed one after another are joined by spans
		bool chaining{true};
		int chain{-1}; // the chain the next pole is added to
		int last{-1}; // the pole the next span starts at
		bool append{true}; // the next pole goes after the last one of the chain, otherwise before the first
		int chain_selected{-1}; // the chain shown in the panel
		int connect_from{-1}; // hanging the wires on from this suspension: the next poles clicked get the spans
		char power[64]{"none"};
		float voltage{3000.0f};
		int material{0}; // 0 cu, 1 al
		int state{0}; // 0 new, 1 worn, 2 tarnished (state 0, 2, 1 in the entry)
		std::map<std::size_t, catenary_spot> spots; // where the suspensions are by the track, by their numbers
		std::vector<catenary_change> undo;
		int group{0};
		catenary::cantilever_result probe;
		bool probed{false};
		bool probe_good{false};
		std::string status;
		bool scanned{false};
		// gantries: two masts and the beam between them, drops with cantilevers on the beam
		std::vector<std::string> masts;
		int mast_first{0};
		int mast_second{0};
		bool mast_second_own{false}; // the second mast was picked by the user: it no longer follows the first
		bool gantry_high{false};
		int gantry_side{1};
		bool gantry_placing{false}; // the next two clicks put the masts
		bool gantry_single{false}; // what's placed is a single beam element on one through mast: the second click sets its direction
		std::optional<glm::dvec3> gantry_first; // the first mast, waiting for the second
		std::optional<glm::dvec2> gantry_track; // the direction of the track by the first mast, for the beam square to it
		bool gantry_square{true}; // the beam goes square to the track
		bool gantry_snap{true}; // the first mast goes at gantry_face from the axis of the track under the cursor
		int shift_group{0}; // the undo group of the distance of a mast being dragged; 0: the next change opens one
		float gantry_face{2.50f}; // from the axis of the track to the face of the mast (towards the track)
		char gantry_skin[128]{}; // the replaceable texture of the masts, the beam and the drops
		int dragging{0}; // 1: the selected pole along the track, 2: the selected gantry along the track
		glm::vec2 pressed{0.f};
		glm::dvec3 drag_start{0.0}; // the point of the ground the drag started at
		std::vector<std::pair<std::size_t, catenary::gantry>> drag_group; // the spans of the dragged gantry as they were when the drag started
		int drag_undo{0};
		glm::dvec3 drag_anchor{0.0}; // the first mast of the dragged gantry where the drag started
		double drag_axis{0.0}; // its axis from the axis of the track then: the drag keeps it there ("first mast from the track") // the undo group of the drag, opened at its first move
		std::vector<std::pair<std::size_t, catenary::suspension>> drag_drops; // the drops of the gantry, as they were
		int gantry_selected{-1}; // the gantry the drops go to
		std::string drop_choice; // the drop for the next cantilevers on a gantry; empty: the shortest one which fits
		// the described catalogue (data/traction_catalog.txt): the group the user picks, the family and the card in it
		// (empty / 0: chosen by the track, the stagger and the distance A)
		std::string catalog_kind;
		std::string catalog_group;
		std::string catalog_family;
		int catalog_card{0};
		int page{0}; // the page of the catenary field of work: 0 poles, 1 gantries, 2 wires
		int gantry_hang{0}; // the next cantilevers on the selected gantry: 0 on drops of the beam, 1 straight on its masts
	};
	catenary_tool m_catenary;
	void catenary_scan();
	bool catenary_spot_at(TTrack &Track, int const Path, glm::dvec3 const &Point, glm::dvec3 const &Cursor, int const Side, catenary_spot &Spot) const;
	// a pole of this type can't stand there: its group doesn't stand by switches and a switch is by it
	bool catenary_switch_error(std::string const &Pole, glm::dvec3 const &Foot, TTrack const *Track) const;
	bool catenary_switch_error(std::size_t const Index) const; // of a placed pole, remembered until it moves or changes type
	struct switch_check
	{
		glm::dvec3 foot{0.0};
		std::string pole;
		bool error{false};
	};
	mutable std::vector<std::pair<TTrack const *, int>> m_switch_paths; // the paths of switches, gathered again when the number of tracks changes
	mutable std::size_t m_switch_tracks{static_cast<std::size_t>(-1)};
	mutable std::map<std::size_t, switch_check> m_switch_checks;
	void catenary_compose(catenary_spot &Spot, double const Clearance, double const Stagger, bool const Toward, std::string const &Pole = {}) const;
	bool catenary_on_mast(catenary::gantry const &Gantry, double const Stagger, catenary_spot &Spot) const;
	catenary_spot const *catenary_spot_of(std::size_t const Index);
	int catenary_hit() const;
	int catenary_span_hit() const;
	void catenary_press(track_intent const &Intent);
	void catenary_select(int const Index);
	void catenary_apply(catenary_spot const &Spot);
	void catenary_copy(std::vector<std::size_t> const &Targets);
	bool catenary_propose_toward(bool &Toward);
	void catenary_place(catenary_spot const &Spot);
	void catenary_chain_apply();
	void catenary_connect(std::size_t const To);
	int catenary_gantry_hit() const;
	bool catenary_gantry_beam(std::size_t const Index, glm::dvec3 &From, glm::dvec3 &To, double &Bottom) const;
	bool catenary_gantry_second(glm::dvec3 const &Point, glm::dvec3 &Second, std::string &Beam) const;
	glm::dvec3 catenary_mast_point() const;
	// the face of the mast from the axis of the nearest track (within 20 m); false: no track near
	bool catenary_mast_face(std::string const &Mast, glm::dvec3 const &Point, double &Face, glm::dvec2 &Away) const;
	// the selected gantry (all its spans and what hangs on them) moved in the plan
	void catenary_gantry_shift(glm::dvec2 const &Shift);
	// the place of the first mast at Axis from the axis of the track nearest to Point, on the side of Point; false: no track near
	bool catenary_along_track(glm::dvec3 const &Point, double const Axis, glm::dvec3 &Place) const;
	bool catenary_on_gantry(catenary_spot &Spot);
	void catenary_drag_update();
	// the selected gantry anew: another beam, masts, turned around its middle (degrees), or square to the nearest track
	void catenary_gantry_reshape(std::string const &Beam, double const Turn, bool const Square);
	void catenary_release();
	void catenary_gantry_hangers(std::size_t const Gantry, std::vector<std::pair<std::size_t, catenary::suspension>> &Hangers) const;
	bool catenary_delete_selected();
	bool catenary_undo();
	void render_catenary_ui();
	bool render_catenary_catalog_ui();
	void render_catenary_chain_ui();
	// the anchors at both ends of the selected chain: add, settings, remove
	void render_catenary_anchor_ui(std::size_t Chain);
	// the anchor of the chain at its start (Start) or end; catenary_anchor_size() when none
	std::size_t catenary_anchor_of(catenary::chain const &Chain, std::string const &File, bool Start) const;
	void render_catenary_cantilever_ui();
	void render_catenary_wires_ui();
	void catenary_choose_page(int const Page);
	void draw_catenary_overlay() const;
	std::string catenary_intent_label(catenary_spot const &Spot) const;
	// the hooks of editor_mode
	bool catenary_intent(track_intent &Intent);
	std::string catenary_mode_name() const;
	void catenary_key_hints(std::vector<key_hint> &Hints) const;
	bool catenary_on_key(int const Key, int const Action, int const Mods);
};
