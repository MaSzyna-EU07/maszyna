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

// a deferred visual node captured during the visual enumeration pass, to be (re)built
// later in camera-distance order. holds the node's fully resolved tokens as text -- numbers
// come through cParser losslessly (std::to_chars shortest round-trip), so rebuilding from
// this text reproduces the exact same node -- plus a snapshot of the transform/group context
// it was read under, so the out-of-order rebuild places and groups it identically.
struct visual_record {
    std::string text;              // "node <range> <range> <name> <type> ... end<type>"
    glm::dvec3 offset { 0.0 };     // top of the origin stack at capture (identity if none)
    glm::vec3 scale { 1.f };       // top of the scale stack at capture (identity if none)
    glm::vec3 rotation { 0.f };    // active rotation at capture
    bool has_offset { false };
    bool has_scale { false };
    scene::group_handle group {};  // group the node was read under
    glm::dvec3 worldpos { 0.0 };   // transformed position (for models; sort key source)
    bool isshape { false };        // terrain shape/lines -> load before models (ground first)
    double sortkey { 0.0 };        // squared distance to camera; lower = built sooner
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
	// visual phase sub-state: while enumerate is true the visual pass captures deferred
	// nodes into records instead of building them; once the replay is exhausted they are
	// sorted by camera distance and built in that order (enumdone).
	bool enumerate { false };
	bool enumdone { false };
	std::vector<visual_record> records;
	std::size_t buildcursor { 0 };

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
    // visual streaming (camera-ordered deferred load): capture one visual node (already
    // past its "node" token) into a record; build budgeted records in sorted order.
    void enumerate_visual_node( deserializer_state &State );
    bool build_visual_records( std::shared_ptr<deserializer_state> State );
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
};

} // simulation

//---------------------------------------------------------------------------
