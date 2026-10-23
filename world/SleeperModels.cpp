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
#include "world/TrackFeatures.h"
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
#include <fstream>
#include <iterator>
#include <optional>

namespace track_sleepers {

namespace {

// the models of the beams found so far, by their names and skins and seats
std::map<std::string, beam_model> &known_models() {
    static std::map<std::string, beam_model> models;
    return models;
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
    auto const corner = [&]( std::size_t const Index ) {
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

// the model of the beams of the name and skin, loaded and measured on the first use; null when it can't be loaded. the feet of
// the rails go to the given seat in it, or to its top where the seat is below zero
beam_model const *find_model( std::string const &Name, std::string const &Skinname, float const Seat ) {

    if( Name.empty() ) {
        return nullptr;
    }
    auto const Skin { Skinname == "none" ? std::string {} : Skinname };
    auto const key { Name + '|' + Skin + '|' + std::to_string( Seat ) };
    auto &models { known_models() };
    auto found { models.find( key ) };
    if( found == models.end() ) {
        auto &model { models[ key ] };
        model.name = Name;
        model.skinname = Skin;
        model.model = TModelsManager::GetModel( Name, false );
        if( model.model == nullptr ) {
            ErrorLog( "Bad sleepers3d: model \"" + Name + "\" failed to load" );
            return nullptr;
        }
        if( false == Skin.empty() ) {
            model.skin = GfxRenderer->Fetch_Material( Skin );
        }
        // a timber put together from its parts, with a part between the ends stretched and repeated to the length
        model.stretchable = true;
        for( int part = 0; part < part_count; ++part ) {
            model.parts[ part ] = model.model->GetFromName( part_names[ part ] );
            model.stretchable = model.stretchable && ( part >= timber_parts || model.parts[ part ] != nullptr );
        }
        auto top { std::numeric_limits<float>::lowest() };
        model.span = { std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };
        if( model.stretchable ) {
            for( int part = 0; part < part_count; ++part ) {
                if( model.parts[ part ] == nullptr ) {
                    continue;
                }
                glm::vec2 span { std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };
                auto const timber { part < timber_parts };
                vertices_of( model.parts[ part ], [&]( glm::vec3 const &Position ) {
                    span = { std::min( span.x, Position.x ), std::max( span.y, Position.x ) };
                    if( timber ) {
                        top = std::max( top, Position.y );
                    } } );
                model.spans[ part ] = span;
                if( timber ) {
                    model.span = { std::min( model.span.x, span.x ), std::max( model.span.y, span.y ) };
                }
            }
            model.stretchable = model.spans[ part_main ].y > model.spans[ part_main ].x;
        }
        else {
            // a whole model, measured on its most detailed submodels, the others being the same beam seen from further
            model.parts = {};
            for( auto *submodel { model.model->Root }; submodel != nullptr; submodel = submodel->Next ) {
                if( submodel->fSquareMinDist > 0.f ) {
                    continue;
                }
                vertices_of( submodel, [&]( glm::vec3 const &Position ) {
                    model.span = { std::min( model.span.x, Position.x ), std::max( model.span.y, Position.x ) };
                    top = std::max( top, Position.y ); } );
            }
        }
        // the levels of detail, by the distances they're seen from
        for( auto *submodel { model.model->Root }; submodel != nullptr; submodel = submodel->Next ) {
            if( std::find( model.bands.begin(), model.bands.end(), submodel->fSquareMinDist ) == model.bands.end() ) {
                model.bands.push_back( submodel->fSquareMinDist );
            }
        }
        std::sort( model.bands.begin(), model.bands.end() );
        // the rails sit on the seat of the sleeper, below the fastenings sticking up over it, or on the plate of the fastening
        model.seat = ( Seat >= 0.f ? Seat : top );
        if( Seat < 0.f && model.fastened() ) {
            model.seat = std::max( top, surface_at( model.parts[ part_plate ], 0.f, 0.f ).value_or( top ) );
        }
        if( model.fastened() ) {
            for( auto const part : { part_chair, part_slide } ) {
                if( model.parts[ part ] != nullptr ) {
                    model.slide = std::max( model.slide, surface_at( model.parts[ part ], static_cast<float>( -slide_probe ), 0.f ).value_or( model.seat ) - model.seat );
                }
            }
        }
        if( model.length() < 0.5f ) {
            ErrorLog( "Bad sleepers3d: model \"" + Name + "\" has no beam along x" );
            model.model = nullptr;
            return nullptr;
        }
        WriteLog(
            "sleepers3d: model \"" + Name + "\"" + ( model.stretchable ? std::string { " of parts" } : std::string { " whole" } )
            + ", " + to_string( model.span.x, 3 ) + ".." + to_string( model.span.y, 3 )
            + ( model.stretchable ?
                ", main " + to_string( model.spans[ part_main ].x, 3 ) + ".." + to_string( model.spans[ part_main ].y, 3 ) :
                std::string {} )
            + ( model.fastened() ?
                std::string { ", fastenings plate" } + ( model.parts[ part_chair ] != nullptr ? " chair" : "" ) + ( model.parts[ part_check ] != nullptr ? " check" : "" ) + ( model.parts[ part_slide ] != nullptr ? " slide" : "" ) :
                std::string {} )
            + ", seat " + to_string( model.seat, 3 )
            + ( model.slide > 0.f ? ", slide " + to_string( model.slide, 3 ) : std::string {} ) );
        return &model;
    }
    return ( found->second.model != nullptr ? &found->second : nullptr );
}

} // namespace

beam_model const *timber_model() {

    if( false == track_features::sleepers ) {
        return nullptr;
    }
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
    return 2.0 * ( 0.5 * std::abs( Track.fTrackWidth ) + rail_reach + overhang );
}

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
    for( fs::recursive_directory_iterator entry { root, fs::directory_options::follow_directory_symlink | fs::directory_options::skip_permission_denied, error }, end;
         entry != end; entry.increment( error ) ) {
        if( error ) {
            break;
        }
        if( false == entry->is_regular_file( error ) ) {
            continue;
        }
        auto const extension { ToLower( entry->path().extension().string() ) };
        if( extension != ".t3d" && extension != ".e3d" ) {
            continue;
        }
        auto name { ToLower( fs::relative( entry->path(), root, error ).replace_extension().generic_string() ) };
        if( error || name.empty() ) {
            continue;
        }
        if( name.find( "podklad" ) == std::string::npos && name.find( "podrozjazd" ) == std::string::npos
         && name.find( "sleeper" ) == std::string::npos && name.find( "timber" ) == std::string::npos ) {
            continue;
        }
        // a timber model has the parts it's put together from, told by their names in the file
        std::ifstream file { entry->path(), std::ios::binary };
        std::string const content { std::istreambuf_iterator<char>( file ), std::istreambuf_iterator<char>() };
        auto const has = [&]( std::string const &Part ) {
            return (
                extension == ".t3d" ?
                    content.find( "Name: " + Part + "\n" ) != std::string::npos || content.find( "Name: " + Part + " " ) != std::string::npos
                 || content.find( "Name: " + Part + "\r" ) != std::string::npos :
                    content.find( std::string { '\0' } + Part + '\0' ) != std::string::npos ); };
        auto const timber { has( "begin" ) && has( "main" ) && has( "end" ) };
        if( std::none_of( models.begin(), models.end(), [&]( pack_model const &Model ) { return Model.name == name; } ) ) {
            models.push_back( { name, timber } );
        }
    }
    std::sort( models.begin(), models.end(), []( pack_model const &Left, pack_model const &Right ) {
        return ( Left.timber != Right.timber ? Left.timber : Left.name < Right.name ); } );
    return models;
}


} // namespace track_sleepers
