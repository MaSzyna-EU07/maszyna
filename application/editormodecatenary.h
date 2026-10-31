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

#include <limits>
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
	enum class intent_detail { place, select, chain, gantry, mast, anchor };
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
		int inward{0}; // the centre of the bend: +1 on the right of the direction of the path, -1 on the left, 0 straight
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
		int continue_pending{-1}; // the chain goes on from this pole, from the next frame of the panel (the button is in the middle of it)
		bool insert_spans{true}; // a pole placed in a span of a chain goes into the chain between the two poles of the span
		int chain{-1}; // the chain the next pole is added to
		int last{-1}; // the pole the next span starts at
		bool append{true}; // the next pole goes after the last one of the chain, otherwise before the first
		int chain_selected{-1}; // the chain shown in the panel
		int connect_from{-1}; // hanging the wires on from this suspension: the next poles clicked get the spans
		int umbrella_from{-1}; // an umbrella from this suspension: the next LMB by a track puts the second cantilever on its pole (or drop)
		int anchor_cantilever{-1}; // a cantilever on the pole of this anchor: the next LMB by a track puts it there
		int anchor_pick{-1}; // anchoring this chain to an existing pole: the next pole clicked carries the tensioning device
		bool anchor_pick_start{false}; // ... at the start of the chain, otherwise at its end
		bool anchor_pick_contact{false}; // ... the contact wires only (anchored apart from the messenger)
		int anchor_selected{-1}; // the anchor picked in the view: its chain is shown, the panel marks it, Del removes it
		bool anchor_scroll{false}; // the panel scrolls to the anchor picked in the view, once
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
		int dragging{0}; // 1: the selected pole along the track, 2: the selected gantry along the track, 3: the selected anchor along the track
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
		// the staggers of the next poles by Iet-107 of PKP PLK (4.6.4, 7.4.2): on a straight the other side of the track axis than
		// the pole before (the full cycle in two spans), on a curve to its outside; the values by the speed of the line
		bool zigzag{true};
		int speed_class{0}; // 0: V <= 160 km/h, 1: V > 160 km/h
		float zigzag_straight{0.30f}; // m, either side of the track axis
		float zigzag_curve{0.30f}; // m, to the outside of the curve
		bool zigzag_flip{false}; // X: the next pole goes the same way as the one before (a swap of the short and the long cantilever)
		// the span of the next pole from the last one: 0 where the cursor is, 1 the recommended one, 2 the length set here
		int span_step{0};
		float span_length{60.0f};
		bool show_staggers{true}; // the staggers written at the poles of the selected chain
		std::optional<int> bend_user; // "cards for" as the user had it before a pole was selected (the pole's own while it is)
		bool file_own{false}; // the catenary file was given by the user: it doesn't follow the active layer
	};
	catenary_tool m_catenary;
	void catenary_scan();
	// the catenary file of the new poles follows the active layer of the scenery
	void catenary_file_follow();
	// the scenery layer of this file name; 0 none
	static std::size_t catenary_layer_of(std::string const &File);
	// Placing: the spot of a new pole, which takes the zigzag proposed after the last pole and the step of the span
	bool catenary_spot_at(TTrack &Track, int const Path, glm::dvec3 const &Point, glm::dvec3 const &Cursor, int const Side, catenary_spot &Spot, bool const Placing = false) const;
	// the zigzag (Iet-107): the largest staggers allowed for the speed of the line, and those the next poles get
	double zigzag_limit(bool const Curve) const;
	double zigzag_stagger(bool const Curve) const;
	// the curve stagger the spans are checked with: Iet-107 or the catalogue (card 0008)
	double span_curve_stagger() const;
	// the network type chosen in the panel
	std::string panel_network() const;
	// the longest span of the network allowed on a curve of this radius (0: straight): the table of the network
	// (traction_layout.txt), above 160 km/h no more than 62 +4 m (Iet-107 7.4.3.1), on a curve the wires tangent to the track
	// axis in the middle of the span (3.4.4.4); 0 when nothing is known
	double span_limit_at(std::string const &Network, double const Radius) const;
	// the span the step "recommended" puts the next pole at: the nominal one (62 m above 160 km/h), no longer than allowed
	// there and than 10 % over the span before (4.6.1.2); 0 when nothing is known
	double span_recommended(double const Radius) const;
	// the digits typed are the span of the next pole
	bool typed_span() const;
	// the wires towards the pole for the next pole at this spot by the zigzag: the other side of the track axis than the last pole
	// (or the same with zigzag_flip); none when there's no last pole near
	std::optional<bool> zigzag_toward(catenary_spot const &Spot) const;
	// the offset of the wires of the suspension from the axis of its track (with the lean of a superelevated track), + towards its pole
	bool stagger_of(std::size_t const Index, double &Offset) const;
	// the same, as a side of the axis: + on the right of Right
	bool lateral_of(std::size_t const Index, glm::dvec2 const &Right, double &Lateral) const;
	// the span the next pole gets from the last one by the step or the digits typed (0: where the cursor is), at a radius there
	double span_step_length(double const Radius) const;
	// the place of the next pole moved along the track (this one or one joined to it) to the span wanted from the last pole;
	// false: no step, or no such place near the cursor
	bool catenary_snap_span(TTrack *&Track, int &Path, double &Parameter) const;
	// the last pole, if it's close enough to go on from (a span of a chain)
	catenary::suspension const *last_pole_near(glm::dvec3 const &Point) const;
	// the span ending at the last pole in its chain (0 when it's the first one)
	double last_span() const;
	// the settings of the panel which make a cantilever, kept while one pole is composed with its own
	struct panel_settings
	{
		int pole{0}, execution{1}, catalog_card{0}, bend{0};
		bool swinging{true}, insulators25{false}, toward{true};
		std::string catalog_kind, catalog_group, catalog_family;
		float stagger{0.30f}, height{5.60f}, system_height{0.0f}, clearance{2.50f};
	};
	panel_settings panel_save() const;
	void panel_restore(panel_settings const &Settings);
	// the panel takes the settings of the suspension (its pole, card group, execution, height of the wires ...)
	void catenary_take_settings(std::size_t const Index);
	// the suspension gets another stagger, its pole and the rest of it stay; Group: the undo group. false: no card for it
	bool catenary_restagger(std::size_t const Index, double const Stagger, bool const Toward, bool const Curve, int const Group);
	// the track bends at the spot enough for the cards of a curve (table 0007)
	static bool track_curve(catenary_spot const &Spot);
	// X: the selected pole the other way (short cantilever <-> long one), or the next pole the other way than the zigzag proposes
	void catenary_flip();
	// the poles of the chain from position From on get the zigzag of Iet-107: the first of them keeps its side (or goes the other
	// way with Flip), the next ones in turn on a straight, outwards on a curve
	void catenary_zigzag_chain(std::size_t const Chain, std::size_t const From, bool const Flip);
	// the check of a chain by Iet-107: staggers, the cycle of the zigzag, the wires in the middle of the spans, the spans and the
	// section; cached until the chain or the settings change
	struct chain_issue
	{
		std::size_t pole{std::numeric_limits<std::size_t>::max()}; // the suspension it's about; max: the whole chain
		std::string text;
		bool error{false};
	};
	struct chain_report
	{
		std::size_t signature{0};
		double length{0.0};
		double shortest{0.0}, longest{0.0};
		std::vector<chain_issue> issues;
		std::vector<std::pair<std::size_t, double>> staggers; // the stagger at each pole, + towards its pole
	};
	mutable std::map<std::size_t, chain_report> m_chain_reports;
	chain_report const &catenary_chain_report(std::size_t const Chain) const;
	void render_catenary_zigzag_ui();
	void render_catenary_chain_report(std::size_t const Chain);
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
	// superelevation of the track at the spot (radians, unsigned) and the horizontal shift of the axis of a leaning
	// pantograph at Height above the rail heads, towards the inside of the bend (table 23-0007, curve with superelevation)
	static double catenary_cant(catenary_spot const &Spot);
	static glm::dvec3 catenary_cant_shift(catenary_spot const &Spot, double Height);
	void catenary_compose(catenary_spot &Spot, double const Clearance, double const Stagger, bool const Toward, std::string const &Pole = {}) const;
	bool catenary_on_mast(catenary::gantry const &Gantry, double const Stagger, catenary_spot &Spot) const;
	catenary_spot const *catenary_spot_of(std::size_t const Index);
	// the same from the const paths (the overlay): the places are a cache filled on demand
	catenary_spot const *catenary_spot_view(std::size_t const Index) const { return const_cast<catenary_editor *>(this)->catenary_spot_of(Index); }
	int catenary_hit() const;
	int catenary_span_hit() const;
	void catenary_press(track_intent const &Intent);
	void catenary_select(int const Index);
	void catenary_apply(catenary_spot const &Spot);
	void catenary_copy(std::vector<std::size_t> const &Targets);
	bool catenary_propose_toward(bool &Toward);
	void catenary_place(catenary_spot const &Spot);
	// the second cantilever of an umbrella on the pole (or drop) of tool.umbrella_from, towards this track; false when it can't
	// be put there (Spot.problem says why when Spot.card is 0)
	bool catenary_umbrella_spot(TTrack &Track, int Path, catenary_spot &Spot);
	void catenary_umbrella_place(catenary_spot const &Spot);
	// a cantilever on the pole of the anchor tool.anchor_cantilever, to this track; false when it can't be put there (Spot.problem
	// says why when Spot.card is 0)
	bool catenary_anchor_cantilever_spot(TTrack &Track, int Path, catenary_spot &Spot) const;
	// the cantilever goes on the anchor pole, the anchor goes onto its pole (on <name>)
	void catenary_anchor_cantilever_place(catenary_spot const &Spot);
	// the two cantilevers of the umbrella of this suspension become separate ones
	void catenary_umbrella_break(std::size_t Index);
	// a cantilever of an umbrella or of spread cantilevers: composed with its pole (and its console mount) where they are
	void catenary_keep_pole(catenary_spot &Spot, catenary::suspension const &Current, double Stagger, bool Toward) const;
	// spread cantilevers on the console of the pole: this one onto the first mount, a second one onto the other, same track
	void catenary_spread_create(std::size_t Index);
	// the pole of an umbrella or of spread cantilevers moved to Foot (rail heads): both cantilevers go with it
	// (Panel: from the distance slider - the selected one takes the stagger of the panel, the slider is one undo group)
	void catenary_pair_move(std::size_t Selected, glm::dvec3 const &Foot, bool Panel);
	// a third cantilever on the pole of spread cantilevers, in the axis of the pole, to the same track (shared_pole)
	void catenary_third_create(std::size_t Index);
	// the span of a chain the spot falls in (in the plan: the wire point near the line of the span, between its ends)
	struct chain_insert
	{
		int chain{-1};
		std::size_t after{0}; // the new pole goes after this position of the chain
		std::size_t from{0}, to{0}; // the suspensions at both ends of the span
	};
	bool catenary_insert_at(catenary_spot const &Spot, chain_insert &Insert) const;
	// the next pole placed starts a new chain
	void catenary_chain_end();
	// the next poles placed go on from this pole: at the end (or the start) of its chain, or a new chain from it;
	// false when the pole is in the middle of a chain
	bool catenary_chain_continue(std::size_t Pole);
	// the chain block of the poles page: what the next click does, end, anchor, go on from the selected pole
	void render_catenary_chain_state();
	void catenary_chain_apply();
	void render_catenary_messenger_from(std::size_t Chain);
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
	// Rail: the height of the rail heads of that track there
	bool catenary_along_track(glm::dvec3 const &Point, double const Axis, glm::dvec3 &Place, double *Rail = nullptr) const;
	// the track a new pole goes by: the one under the cursor, or the nearest one within reach of the place on the ground under
	// the cursor (the pole is put where it stands, not on the axis of the track); found again each frame by catenary_intent()
	struct place_target
	{
		TTrack *track{nullptr};
		int path{0};
		glm::dvec3 point{0.0}; // on the axis of the track, nearest to the cursor
		glm::dvec3 cursor{0.0}; // the place under the cursor at the height of the track
	};
	place_target m_target;
	bool catenary_find_target(place_target &Target) const;
	// the anchor selected in the view moved with the cursor, along the track at the distance its pole has from it
	void catenary_anchor_drag();
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
	std::size_t catenary_anchor_of(catenary::chain const &Chain, std::string const &File, bool Start, bool Contact = false) const;
	// anchors the start or the end of the chain: on a new pole past the last one, or (On >= 0) on that existing pole
	bool catenary_anchor_create(std::size_t Chain, bool Start, int On, bool Contact = false);
	// removes the anchor at the start or the end of the chain (one undo step with the chain)
	void catenary_anchor_delete(std::size_t Chain, bool Start, bool Contact = false);
	// the anchor whose tensioning device is under the cursor; -1 if none
	int catenary_anchor_hit() const;
	// shows the chain in the panel as a click on its span does
	void catenary_chain_show(int Chain);
	void render_catenary_cantilever_ui();
	void render_catenary_wires_ui();
	void catenary_choose_page(int const Page);
	void draw_catenary_overlay() const;
	std::string catenary_intent_label(catenary_spot const &Spot) const;
	// a part picked as a node: what it belongs to selected in the tool
	bool catenary_pick_part(void const *Node);
	// the hooks of editor_mode
	bool catenary_intent(track_intent &Intent);
	std::string catenary_mode_name() const;
	void catenary_key_hints(std::vector<key_hint> &Hints) const;
	bool catenary_on_key(int const Key, int const Action, int const Mods);
};
