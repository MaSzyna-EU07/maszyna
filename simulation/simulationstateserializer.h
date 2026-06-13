/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <unordered_map>

#include "utilities/parser.h"
#include "scene/eu7/eu7_types.h"
#include "scene/scene.h"

class TModel3d;

namespace simulation {

struct eu7_pack_apply_session {
    std::unordered_map<std::string, TModel3d *> *mesh_cache { nullptr };
    std::unordered_map<std::string, scene::node_data> *nodedata_cache { nullptr };
};

struct deserializer_state {
	std::string scenariofile;
	cParser input;
	scene::scratch_data scratchpad;
	using deserializefunctionbind = std::function<void()>;
	std::unordered_map<
	    std::string,
	    deserializefunctionbind> functionmap;

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
	// parsuje wyemitowany tekst modulu EU7B przez istniejacy dispatch tokenow
	void
	    deserialize_module_text(
	        std::string const &Text,
	        std::string const &SourceName,
	        scene::scratch_data &Scratchpad );
	// Fallback SCM/INC gdy brak child .eu7 — normalny parser z include.
	void
	    deserialize_include_file(
	        std::string const &IncludeReference,
	        std::string const &CurrentRelativeFile,
	        std::vector<std::string> const &Parameters,
	        scene::scratch_data &Scratchpad );
	// Bezposrednio z Eu7Scene — bez emit/tokenow.
	void
	    apply_eu7_scene(
	        scene::eu7::Eu7Scene const &Scene,
	        scene::scratch_data &Scratchpad );
	// Tylko MODL z Eu7Scene — bez TRAK/EVNT/…
	void
	    apply_eu7_models(
	        std::vector<scene::eu7::Eu7Model> const &Models,
	        scene::scratch_data &Scratchpad );
	// MODL z chunku PACK (v7) — location/angles juz w world-space.
	void
	    apply_eu7_pack_models(
	        std::vector<scene::eu7::Eu7Model> const &Models,
	        scene::scratch_data &Scratchpad );
	void
	    apply_eu7_pack_models(
	        std::vector<scene::eu7::Eu7Model> const &Models,
	        scene::scratch_data &Scratchpad,
	        std::size_t Offset,
	        std::size_t Count,
	        eu7_pack_apply_session const *Session = nullptr );
	// Wskaznik do bufora zdekodowanego przez worker — bez kopiowania wektora.
	void
	    apply_eu7_pack_models(
	        scene::eu7::Eu7Model const *Models,
	        std::size_t Count,
	        scene::scratch_data &Scratchpad,
	        eu7_pack_apply_session const *Session = nullptr );
	// Chunk insert: Models + Offset w buforze workera (pending_apply_offset).
	void
	    apply_eu7_pack_models(
	        scene::eu7::Eu7Model const *Models,
	        std::size_t Offset,
	        std::size_t Count,
	        scene::scratch_data &Scratchpad,
	        eu7_pack_apply_session const *Session = nullptr );
	// Odlozone sklady AI — ladowane w tle po wejsciu w gre.
	void
	    drain_deferred_eu7_trainsets( double MaxMs = 12.0 );

private:
	struct eu7_transform_state;
	void
	    insert_eu7_models(
	        std::vector<scene::eu7::Eu7Model> const &Models,
	        scene::scratch_data &Scratchpad,
	        eu7_transform_state &State );
	void
	    insert_eu7_pack_models(
	        std::vector<scene::eu7::Eu7Model> const &Models,
	        scene::scratch_data &Scratchpad );
	void
	    insert_eu7_pack_models(
	        std::vector<scene::eu7::Eu7Model> const &Models,
	        scene::scratch_data &Scratchpad,
	        std::size_t Offset,
	        std::size_t Count,
	        eu7_pack_apply_session const *Session = nullptr );
	void
	    insert_eu7_pack_models(
	        scene::eu7::Eu7Model const *Models,
	        std::size_t Count,
	        scene::scratch_data &Scratchpad,
	        eu7_pack_apply_session const *Session = nullptr );
	void
	    deserialize_parser_tokens(
	        cParser &Input,
	        scene::scratch_data &Scratchpad,
	        std::string const &SourceName );
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
    void populate_deserialize_functionmap(
        std::unordered_map<std::string, deserializer_state::deserializefunctionbind> &Functionmap,
        cParser &Input,
        scene::scratch_data &Scratchpad );
};

} // simulation

//---------------------------------------------------------------------------
