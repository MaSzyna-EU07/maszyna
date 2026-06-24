/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "utilities/parser.h"
#include "scene/scene.h"

namespace simulation {

// a deferred visual node recorded during the first (indexing) replay pass, so it can be rebuilt
// on demand when its region section enters camera range -- without re-scanning the whole twin.
// holds where to find the node (which twin + byte offset of its marker) and the context needed to
// place it identically (the transform active at that point, and the include parameters its "(pN)"
// tokens resolve against).
struct visual_ref {
    int twin { -1 };            // index into deserializer_state::twins (file+path to re-open)
    std::size_t offset { 0 };   // byte offset of the node's marker within that twin
    glm::dvec3 t_offset { 0.0 };
    glm::vec3 t_rotation { 0.f };
    glm::vec3 t_scale { 1.f };
    bool has_offset { false };
    bool has_scale { false };
    std::vector<std::string> params; // include parameters in effect (empty for a direct node)
};

struct deserializer_state {
	std::string scenariofile;
	cParser input;
	scene::scratch_data scratchpad;
	using deserializefunctionbind = std::function<void()>;
	std::unordered_map<
	    std::string,
	    deserializefunctionbind> functionmap;
	// progressive (two-pass) load over a binary twin: first pass loads infrastructure,
	// second pass loads visual nodes. false while in the first (infrastructure) pass.
	bool visualphase { false };
	// set once the whole load (both passes / single text pass) has fully finished
	bool done { false };
	// camera-following visual streaming. once infrastructure is up, visual nodes (3d models,
	// terrain shapes) stream in only for the region sections currently within STREAM_RADIUS of
	// the camera; as the camera moves into new sections they are built too. nothing is unloaded.
	// the twin is replayed once per "build cycle" (a set of newly-wanted sections), building
	// only nodes whose section is in that set and O(1)-skipping the rest by their v7 marker.
	glm::dvec3 ringeye { 0.0 };
	// the camera centre is only meaningful once control reaches the driver (the loader hasn't
	// positioned the camera yet). sampled lazily on the first driver pass; if still unusable
	// (e.g. ghostview at the origin) we fall back to building everything in one pass (ringall).
	bool ringeye_valid { false };
	int ringeye_waits { 0 }; // frames spent waiting for the camera to be positioned
	bool ringall { false };  // no usable camera centre -> build all visual nodes in one pass
	bool sectionmode { false }; // usable camera centre -> stream sections within range, follow camera
	bool shapes_built { false }; // first cycle done: explicit shapes + large-range (eager) models built
	bool initial_done { false }; // the first cycle finished -> scenario finalised (map/events/twin)
	bool pass_active { false };  // a section build cycle's replay pass is currently in progress
	std::unordered_set<int> built;   // region-section indices already streamed in
	std::unordered_set<int> tobuild; // section indices targeted by the current/next cycle
	// section index, built during the first replay pass: every deferred visual node is recorded
	// under its region section, so later sections rebuild by seeking straight to their nodes
	// instead of replaying (re-scanning) the whole million-node twin every cycle.
	bool indexed { false };          // first pass finished -> the index below is complete
	std::vector<std::pair<std::string, std::string>> twins; // (file, path) to re-open, interned
	std::unordered_map<std::string, int> twinids;           // "path|file" -> index into twins
	std::unordered_map<int, std::vector<visual_ref>> index; // section -> deferred nodes there
	std::unordered_map<int, std::unique_ptr<cParser>> rebuild_parsers; // one reused parser per twin

	deserializer_state(std::string const &File, cParser::buffertype const Type, const std::string &Path, bool const Loadtraction)
	    : scenariofile(File), input(File, Type, Path, Loadtraction) { }
};

class state_serializer {

public:

// methods
	// starts deserialization from specified file, returns context pointer on success, throws otherwise
	std::shared_ptr<deserializer_state>
	    deserialize_begin(std::string const &Scenariofile);
	// continues deserialization for given context, amount limited by time, returns true if needs to be called again
	bool
	    deserialize_continue(std::shared_ptr<deserializer_state> state);
    // stores class data in specified file, in legacy (text) format
    void
        export_as_text( std::string const &Scenariofile ) const;
	// create new model from node stirng
	TAnimModel * create_model(std::string const &src, std::string const &name, const glm::dvec3 &position);
	// create new eventlauncher from node stirng
	TEventLauncher * create_eventlauncher(std::string const &src, std::string const &name, const glm::dvec3 &position);

private:
// methods
    // restores class data from provided stream
    void deserialize_area( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_isolated( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_assignment( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_atmo( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_camera( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_config( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_description( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_event( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_lua( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_firstinit( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_group( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_endgroup( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_light( cParser &Input, scene::scratch_data &Scratchpad );
	void deserialize_node( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_origin( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_endorigin( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_scale( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_endscale( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_rotate( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_sky( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_test( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_time( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_trainset( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_terrain( cParser &Input, scene::scratch_data &Scratchpad );
    void deserialize_endtrainset( cParser &Input, scene::scratch_data &Scratchpad );
    TTrack * deserialize_path( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata );
    TTraction * deserialize_traction( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata );
    TTractionPowerSource * deserialize_tractionpowersource( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata );
    TMemCell * deserialize_memorycell( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata );
    TEventLauncher * deserialize_eventlauncher( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata );
	TAnimModel * deserialize_model( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata );
    TDynamicObject * deserialize_dynamic( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata );
    sound_source * deserialize_sound( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata );
    void init_time();
    // skips content of stream until specified token
    void skip_until( cParser &Input, std::string const &Token );
    // transforms provided location by specifed rotation and offset
    glm::dvec3 transform( glm::dvec3 Location, scene::scratch_data const &Scratchpad );
    void export_nodes_to_stream( std::ostream &, bool Dirty ) const;
    // region-section index (row-major, clamped to the grid) enclosing a world position --
    // matches basic_region::section()'s indexing, used to bucket visual nodes for streaming.
    static int section_index( glm::dvec3 const &World );
    // record the visual node currently being replayed (twin/offset/transform/params) under its
    // section in the index, so it can be rebuilt later without re-scanning the twin.
    void capture_node( cParser &Input, scene::scratch_data const &Scratchpad, glm::dvec3 const &World );
    // rebuild every indexed node of one section by seeking straight to it (no twin re-scan).
    void rebuild_section( deserializer_state &State, int Section );
    // interns a (file, path) pair into State.twins, returning its index
    static int twin_id( deserializer_state &State, std::string const &File, std::string const &Path );
// members
    // camera-following visual streaming state, mirrored from deserializer_state each
    // deserialize_continue() call so deserialize_model()/deserialize_node() can decide whether
    // a node's section is in the current build set. inactive (builds everything) outside the
    // visual phase, or in ringall (no camera centre) where every node is built in one pass.
    bool m_ringactive { false };
    bool m_ringall { false };   // no camera centre available -> build every node in one pass
    bool m_sectionmode { false }; // stream by camera-range sections
    bool m_shapes_built { false }; // first cycle done: shapes + large-range (eager) models built (skip them)
    glm::dvec3 m_ringeye { 0.0 };
    std::unordered_set<int> const *m_tobuild { nullptr }; // section indices to build this cycle
    // first replay pass records each deferred node into the index (m_indexing); later cycles
    // rebuild sections straight from it (m_rebuilding bypasses the section test so the chosen
    // node always builds). m_state gives deserialize_model()/node() access for capture.
    bool m_indexing { false };
    bool m_rebuilding { false };
    deserializer_state *m_state { nullptr };
};

} // simulation

//---------------------------------------------------------------------------
