/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/TrackSleepers.h"
#include "world/SleeperRules.h"
#include "world/Track.h"
#include "model/Model3d.h"
#include "model/MdlMngr.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <glm/gtc/type_ptr.hpp>

#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <mutex>

namespace track_sleepers {

namespace {

// the models of the beams found so far, by their names and skins and seats
std::map<std::string, beam_model, std::less<>> &known_models() {
    static std::map<std::string, beam_model, std::less<>> models;
    return models;
}

// the models found and loaded by more than one thread
std::mutex &models_mutex() {
    static std::mutex mutex;
    return mutex;
}

// whether this thread loads the models it doesn't find
bool &loads_models() {
    thread_local bool loads { true };
    return loads;
}

// the vertices of the submodel in the space of the model, through its own transform
template <typename Visit>
void vertices_of( TSubModel *Submodel, Visit Visitor ) {

    glm::mat4 transform { 1.f };
    if( Submodel->iFlags & 0xC000 ) {
        transform = Submodel->fMatrix;
    }
    auto const &vertices { Submodel->m_geometry.handle != null_handle ? GfxRenderer->Vertices( Submodel->m_geometry.handle ) : Submodel->Vertices };
    for( auto const &vertex : vertices ) {
        Visitor( glm::vec3 { transform * glm::vec4 { vertex.position, 1.f } } );
    }
}

// whether the model is put together from the parts of a timber, all of them at its top level, with the submodels besides them
void find_parts( beam_model &Model ) {

    // a timber put together from its parts, with a part between the ends stretched and repeated to the length
    std::array<char const *, part_count> const names { "begin", "main", "end" };
    Model.stretchable = true;
    for( int part = 0; part < part_count; ++part ) {
        Model.parts[ part ] = Model.model->GetFromName( names[ part ] );
        Model.stretchable = Model.stretchable && Model.parts[ part ] != nullptr;
    }
    if( false == Model.stretchable ) {
        return;
    }
    // a part is drawn with the other submodels of the top level hidden, so the parts are there too
    auto toplevel { 0 };
    for( auto *submodel { Model.model->Root }; submodel != nullptr; submodel = submodel->Next ) {
        if( std::ranges::find( Model.parts, submodel ) != Model.parts.end() ) {
            ++toplevel;
        }
        else {
            Model.others.push_back( submodel );
        }
    }
    if( toplevel < part_count ) {
        ErrorLog( "Bad sleepers3d: model \"" + Model.name + "\" has its parts begin, main and end below its top level, it's drawn whole" );
        Model.stretchable = false;
        Model.others.clear();
    }
}

// the span of the model along x, of its parts or of the whole of it, and how high it reaches
float measure( beam_model &Model ) {

    auto top { std::numeric_limits<float>::lowest() };
    Model.span = { std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };
    if( Model.stretchable ) {
        for( int part = 0; part < part_count; ++part ) {
            glm::vec2 span { std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };
            vertices_of( Model.parts[ part ], [ &span, &top ]( glm::vec3 const &Position ) {
                span = { std::min( span.x, Position.x ), std::max( span.y, Position.x ) };
                top = std::max( top, Position.y ); } );
            Model.spans[ part ] = span;
            Model.span = { std::min( Model.span.x, span.x ), std::max( Model.span.y, span.y ) };
        }
        Model.stretchable = Model.spans[ part_main ].y > Model.spans[ part_main ].x;
    }
    else {
        // a whole model, measured on its most detailed submodels, the others being the same beam seen from further
        Model.parts = {};
        for( auto *submodel { Model.model->Root }; submodel != nullptr; submodel = submodel->Next ) {
            if( submodel->fSquareMinDist > 0.f ) {
                continue;
            }
            vertices_of( submodel, [ &Model, &top ]( glm::vec3 const &Position ) {
                Model.span = { std::min( Model.span.x, Position.x ), std::max( Model.span.y, Position.x ) };
                top = std::max( top, Position.y ); } );
        }
    }
    return top;
}

// the distances the levels of detail of the model are seen from, each once, the nearest first
void find_bands( beam_model &Model ) {

    // the levels of detail, by the distances they're seen from
    for( auto *submodel { Model.model->Root }; submodel != nullptr; submodel = submodel->Next ) {
        if( std::ranges::find( Model.bands, submodel->fSquareMinDist ) == Model.bands.end() ) {
            Model.bands.push_back( submodel->fSquareMinDist );
        }
    }
    std::ranges::sort( Model.bands );
}

// the model of the name and skin loaded into its entry, measured and logged; null when it can't be loaded or has no beam along x
beam_model const *load_model( beam_model &Model, std::string const &Name, std::string const &Skin, float const Seat ) {

    Model.name = Name;
    Model.skinname = Skin;
    Model.model = TModelsManager::GetModel( Name, false );
    if( Model.model == nullptr ) {
        ErrorLog( "Bad sleepers3d: model \"" + Name + "\" failed to load" );
        return nullptr;
    }
    if( false == Skin.empty() ) {
        Model.skin = GfxRenderer->Fetch_Material( Skin );
    }
    find_parts( Model );
    auto const top { measure( Model ) };
    find_bands( Model );
    // the rails sit on the seat of the sleeper, below the fastenings sticking up over it
    Model.seat = ( Seat >= 0.f ? Seat : top );
    if( Model.length() < 0.5f ) {
        ErrorLog( "Bad sleepers3d: model \"" + Name + "\" has no beam along x" );
        Model.model = nullptr;
        return nullptr;
    }
    WriteLog(
        "sleepers3d: model \"" + Name + "\"" + ( Model.stretchable ? std::string { " of parts" } : std::string { " whole" } )
        + ", " + to_string( Model.span.x, 3 ) + ".." + to_string( Model.span.y, 3 )
        + ( Model.stretchable ?
            ", main " + to_string( Model.spans[ part_main ].x, 3 ) + ".." + to_string( Model.spans[ part_main ].y, 3 ) :
            std::string {} )
        + ", seat " + to_string( Model.seat, 3 ) );
    return &Model;
}

// the model of the beams of the name and skin, loaded and measured on the first use; null when it can't be loaded. the feet of
// the rails go to the given seat in it, or to its top where the seat is below zero
beam_model const *find_model( std::string const &Name, std::string const &Skinname, float const Seat ) {

    if( Name.empty() ) {
        return nullptr;
    }
    auto const Skin { Skinname == "none" ? std::string {} : Skinname };
    auto const key { std::format( "{}|{}|{}", Name, Skin, std::to_string( Seat ) ) };
    std::scoped_lock const lock { models_mutex() };
    auto &models { known_models() };
    auto found { models.find( key ) };
    if( found == models.end() && false == loads_models() ) {
        // loaded for it before a layout is asked for of the thread, for all the tracks, so it's none it could load
        return nullptr;
    }
    if( found == models.end() ) {
        return load_model( models[ key ], Name, Skin, Seat );
    }
    return ( found->second.model != nullptr ? &found->second : nullptr );
}

} // namespace

void models_loaded( bool const Loaded ) {

    loads_models() = Loaded;
}

beam_model const *timber_model() {

    return find_model( Global.Sleepers3D, std::string {}, -1.f );
}

beam_model const *plain_model() {

    return find_model( Global.Sleepers3DPlain, Global.Sleepers3DPlainSkin, Global.Sleepers3DPlainSeat );
}

beam_model const *timber_model( TTrack const &Track ) {

    if( false == Track.m_timber_model_name.empty() ) {
        if( auto const *model { find_model( Track.m_timber_model_name, Track.m_timber_skin_name, -1.f ) }; model != nullptr ) {
            return model;
        }
    }
    return timber_model();
}

beam_model const *plain_model( TTrack const &Track ) {

    if( Track.m_sleeper_enabled && false == Track.m_sleeper_model_name.empty() ) {
        // the sleepers of the track drawn as they are in the model, their tops at the feet of the rails
        if( auto const *model { find_model( Track.m_sleeper_model_name, Track.m_sleeper_skin_name, -1.f ) }; model != nullptr ) {
            return model;
        }
    }
    return plain_model();
}

double plain_length( TTrack const &Track ) {

    if( auto const *model { plain_model( Track ) }; model != nullptr ) {
        return model->length();
    }
    return 2.0 * ( 0.5 * std::abs( Track.fTrackWidth ) + rail_reach + beam_overhang );
}

namespace {

// the model of sleepers or timbers of the file added to those found, once by its name, what goes wrong left in the error
void add_pack_model( std::filesystem::directory_entry const &Entry, std::filesystem::path const &Root, std::error_code &Error, std::vector<pack_model> &Models ) {

    namespace fs = std::filesystem;
    if( false == Entry.is_regular_file( Error ) ) {
        return;
    }
    auto const extension { ToLower( Entry.path().extension().string() ) };
    if( extension != ".t3d" && extension != ".e3d" ) {
        return;
    }
    auto name { ToLower( fs::relative( Entry.path(), Root, Error ).replace_extension().generic_string() ) };
    if( Error || name.empty() ) {
        return;
    }
    if( name.find( "podklad" ) == std::string::npos && name.find( "podrozjazd" ) == std::string::npos
     && name.find( "sleeper" ) == std::string::npos && name.find( "timber" ) == std::string::npos ) {
        return;
    }
    // a timber model has the parts it's put together from, told by their names in the file
    std::ifstream file { Entry.path(), std::ios::binary };
    std::string const content { std::istreambuf_iterator<char>( file ), std::istreambuf_iterator<char>() };
    auto const has = [ &extension, &content ]( std::string const &Part ) {
        return (
            extension == ".t3d" ?
                content.find( "Name: " + Part + "\n" ) != std::string::npos || content.find( "Name: " + Part + " " ) != std::string::npos
             || content.find( "Name: " + Part + "\r" ) != std::string::npos :
                content.find( std::string { '\0' } + Part + '\0' ) != std::string::npos ); };
    auto const timber { has( "begin" ) && has( "main" ) && has( "end" ) };
    if( std::ranges::none_of( Models, [ &name ]( pack_model const &Model ) { return Model.name == name; } ) ) {
        Models.emplace_back( std::move( name ), timber );
    }
}

} // namespace

std::vector<pack_model> const &pack_models() {

    static std::vector<pack_model> models;
    static auto scanned { false };
    if( scanned ) {
        return models;
    }
    scanned = true;
    namespace fs = std::filesystem;
    std::error_code error;
    auto const root { fs::path { paths::models } };
    fs::recursive_directory_iterator entry { root, fs::directory_options::follow_directory_symlink | fs::directory_options::skip_permission_denied, error };
    fs::recursive_directory_iterator const end;
    while( entry != end ) {
        if( error ) {
            break;
        }
        add_pack_model( *entry, root, error, models );
        entry.increment( error );
    }
    std::ranges::sort( models, []( pack_model const &Left, pack_model const &Right ) {
        return ( Left.timber != Right.timber ? Left.timber : Left.name < Right.name ); } );
    return models;
}


} // namespace track_sleepers
