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
#include <optional>

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
    if( ( Submodel->iFlags & 0xC000 ) && ( Submodel->GetMatrix() != nullptr ) ) {
        transform = glm::make_mat4( Submodel->GetMatrix()->readArray() );
    }
    auto const &vertices { Submodel->m_geometry.handle != null_handle ? GfxRenderer->Vertices( Submodel->m_geometry.handle ) : Submodel->Vertices };
    for( auto const &vertex : vertices ) {
        Visitor( glm::vec3 { transform * glm::vec4 { vertex.position, 1.f } } );
    }
}

// the highest the faces of the submodel reach over the point of the plan of the model, in the space of the model; none where
// no face lies over it
std::optional<float> surface_at( TSubModel *Submodel, float const X, float const Z ) {

    glm::mat4 transform { 1.f };
    if( ( Submodel->iFlags & 0xC000 ) && ( Submodel->GetMatrix() != nullptr ) ) {
        transform = glm::make_mat4( Submodel->GetMatrix()->readArray() );
    }
    auto const indexed { Submodel->m_geometry.handle != null_handle };
    auto const &vertices { indexed ? GfxRenderer->Vertices( Submodel->m_geometry.handle ) : Submodel->Vertices };
    auto const &indices { indexed ? GfxRenderer->Indices( Submodel->m_geometry.handle ) : Submodel->Indices };
    auto const count { indices.empty() ? vertices.size() : indices.size() };
    auto const corner = [ &transform, &vertices, &indices ]( std::size_t const Index ) {
        return glm::vec3 { transform * glm::vec4 { vertices[ indices.empty() ? Index : indices[ Index ] ].position, 1.f } }; };
    std::optional<float> result;
    for( std::size_t index = 0; index + 2 < count; index += 3 ) {
        auto const a { corner( index ) };
        auto const b { corner( index + 1 ) };
        auto const c { corner( index + 2 ) };
        auto const area { ( b.x - a.x ) * ( c.z - a.z ) - ( c.x - a.x ) * ( b.z - a.z ) };
        if( std::abs( area ) < 1e-9f ) {
            continue;
        }
        auto const u { ( ( X - a.x ) * ( c.z - a.z ) - ( c.x - a.x ) * ( Z - a.z ) ) / area };
        auto const v { ( ( b.x - a.x ) * ( Z - a.z ) - ( X - a.x ) * ( b.z - a.z ) ) / area };
        if( u < -1e-5f || v < -1e-5f || u + v > 1.f + 1e-5f ) {
            continue;
        }
        auto const height { a.y + u * ( b.y - a.y ) + v * ( c.y - a.y ) };
        result = ( result ? std::max( *result, height ) : height );
    }
    return result;
}

// whether the model is put together from the parts of a timber, all of them at its top level, with the submodels besides them;
// the fastenings of the rails among its parts where it has them
void find_parts( beam_model &Model ) {

    // a timber put together from its parts, with a part between the ends stretched and repeated to the length
    Model.stretchable = true;
    for( int part = 0; part < part_count; ++part ) {
        Model.parts[ part ] = Model.model->GetFromName( part_names[ part ] );
        Model.stretchable = Model.stretchable && ( part >= timber_parts || Model.parts[ part ] != nullptr );
    }
    if( false == Model.stretchable ) {
        return;
    }
    // a part is drawn with the other submodels of the top level hidden, so the parts of the timber are there too
    auto toplevel { 0 };
    for( auto *submodel { Model.model->Root }; submodel != nullptr; submodel = submodel->Next ) {
        auto const found { std::ranges::find( Model.parts, submodel ) };
        if( found == Model.parts.end() ) {
            Model.others.push_back( submodel );
        }
        else if( found - Model.parts.begin() < timber_parts ) {
            ++toplevel;
        }
    }
    if( toplevel < timber_parts ) {
        ErrorLog( "Bad sleepers3d: model \"" + Model.name + "\" has its parts begin, main and end below its top level, it's drawn whole" );
        Model.stretchable = false;
        Model.others.clear();
    }
}

// the span of the model along x, of its parts or of the whole of it, and how high it reaches; the fastenings measured apart, not
// counted in the span and the height of the timber
float measure( beam_model &Model ) {

    auto top { std::numeric_limits<float>::lowest() };
    Model.span = { std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };
    if( Model.stretchable ) {
        for( int part = 0; part < part_count; ++part ) {
            if( Model.parts[ part ] == nullptr ) {
                continue;
            }
            glm::vec2 span { std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };
            glm::vec2 heights { std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };
            auto const timber { part < timber_parts };
            vertices_of( Model.parts[ part ], [ &span, &heights, &top, timber ]( glm::vec3 const &Position ) {
                span = { std::min( span.x, Position.x ), std::max( span.y, Position.x ) };
                heights = { std::min( heights.x, Position.y ), std::max( heights.y, Position.y ) };
                if( timber ) {
                    top = std::max( top, Position.y );
                } } );
            Model.spans[ part ] = span;
            Model.heights[ part ] = heights;
            if( timber ) {
                Model.span = { std::min( Model.span.x, span.x ), std::max( Model.span.y, span.y ) };
            }
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

// where the rails sit in the model: on the seat of the sleeper, below the fastenings sticking up over it, or on the plate of the
// fastening; and how far over it the blades slide on the slide plates of the chairs
void find_seat( beam_model &Model, float const Seat, float const Top ) {

    Model.bed = Top;
    Model.seat = ( Seat >= 0.f ? Seat : Top );
    if( false == Model.fastened() ) {
        return;
    }
    if( Seat < 0.f ) {
        Model.seat = std::max( Top, surface_at( Model.parts[ part_plate ], 0.f, 0.f ).value_or( Top ) );
    }
    for( auto const part : { part_chair, part_slide } ) {
        if( Model.parts[ part ] != nullptr ) {
            Model.slide = std::max( Model.slide, surface_at( Model.parts[ part ], static_cast<float>( -slide_probe ), 0.f ).value_or( Model.seat ) - Model.seat );
        }
    }
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
    find_seat( Model, Seat, top );
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
        + ( Model.fastened() ?
            std::string { ", fastenings plate" } + ( Model.parts[ part_chair ] != nullptr ? " chair" : "" ) + ( Model.parts[ part_check ] != nullptr ? " check" : "" ) + ( Model.parts[ part_slide ] != nullptr ? " slide" : "" ) :
            std::string {} )
        + ", seat " + to_string( Model.seat, 3 )
        + ( Model.slide > 0.f ? ", slide " + to_string( Model.slide, 3 ) : std::string {} ) );
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

beam_model const *timber_model( TTrack const &Track, std::vector<beam> const &Beams ) {

    if( false == Track.m_timber_model_name.empty() ) {
        return timber_model( Track );
    }
    std::vector<std::pair<TTrack const *, std::size_t>> panels;
    for( auto const &beam : Beams ) {
        if( beam.plain || beam.panel == nullptr ) {
            continue;
        }
        auto found { std::find_if( panels.begin(), panels.end(), [&]( auto const &Panel ) { return Panel.first == beam.panel; } ) };
        if( found == panels.end() ) {
            panels.emplace_back( beam.panel, 0 );
            found = std::prev( panels.end() );
        }
        ++found->second;
    }
    auto const most { std::max_element( panels.begin(), panels.end(), []( auto const &Left, auto const &Right ) { return Left.second < Right.second; } ) };
    return ( most != panels.end() ? timber_model( *most->first ) : timber_model( Track ) );
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
