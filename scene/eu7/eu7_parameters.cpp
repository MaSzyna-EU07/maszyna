/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_parameters.h"
#include "scene/eu7/eu7_apply.h"
#include "scene/eu7/eu7_transform.h"

#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <unordered_map>

namespace scene::eu7 {
namespace {

void
bind_include_parameter_impl( std::string &text, std::vector<std::string> const &parameters, bool const to_lower ) {
    if( parameters.empty() ) {
        return;
    }

    std::size_t pos { 0 };
    while( ( pos = text.find( "(p", pos ) ) != std::string::npos ) {
        auto const close { text.find( ')', pos ) };
        if( close == std::string::npos ) {
            break;
        }

        auto const index_text { text.substr( pos + 2, close - ( pos + 2 ) ) };
        text.erase( pos, ( close - pos ) + 1 );

        auto const index { static_cast<std::size_t>( std::atoi( index_text.c_str() ) ) };
        auto const replacement { (
            index >= 1 && ( index - 1 ) < parameters.size() ) ?
                parameters[ index - 1 ] :
                std::string{ "none" } };

        text.insert( pos, replacement );

        if( to_lower ) {
            for( std::size_t i { pos }; i < pos + replacement.size(); ++i ) {
                text[ i ] = static_cast<char>( tolower( static_cast<unsigned char>( text[ i ] ) ) );
            }
        }

        pos += replacement.size();
    }
}

void
bind_name( std::string &text, std::vector<std::string> const &parameters ) {
    bind_include_parameter_impl( text, parameters, true );
}

void
bind_path( std::string &text, std::vector<std::string> const &parameters ) {
    bind_include_parameter_impl( text, parameters, false );
}

void
bind_optional_name( std::optional<std::string> &text, std::vector<std::string> const &parameters ) {
    if( text ) {
        bind_name( *text, parameters );
    }
}

void
bind_node_strings( Eu7BasicNode &node, std::vector<std::string> const &parameters ) {
    bind_name( node.name, parameters );
    bind_name( node.node_type, parameters );
}

} // namespace

void
bind_include_parameter( std::string &text, std::vector<std::string> const &parameters, bool const to_lower ) {
    bind_include_parameter_impl( text, parameters, to_lower );
}

void
apply_include_parameters_to_models( std::vector<Eu7Model> &models, std::vector<std::string> const &parameters ) {
    if( parameters.empty() ) {
        return;
    }

    for( auto &model : models ) {
        bind_node_strings( model.node, parameters );
        bind_path( model.model_file, parameters );
        bind_path( model.texture_file, parameters );
    }
}

void
apply_include_parameters_to_scene( Eu7Scene &scene, std::vector<std::string> const &parameters ) {
    if( parameters.empty() ) {
        return;
    }

    for( auto &track : scene.tracks ) {
        bind_node_strings( track.node, parameters );
        if( track.visibility ) {
            bind_path( track.visibility->material1, parameters );
            bind_path( track.visibility->material2, parameters );
        }
        for( auto &[key, value] : track.tail_keywords ) {
            bind_name( key, parameters );
            bind_name( value, parameters );
        }
    }

    for( auto &traction : scene.traction ) {
        bind_node_strings( traction.node, parameters );
        bind_name( traction.power_supply_name, parameters );
        bind_name( traction.material_raw, parameters );
        bind_optional_name( traction.parallel_name, parameters );
    }

    for( auto &source : scene.power_sources ) {
        bind_node_strings( source.node, parameters );
    }

    for( auto &shape : scene.shapes ) {
        bind_node_strings( shape.node, parameters );
        bind_path( shape.material_path, parameters );
    }

    for( auto &shape : scene.terrain_shapes ) {
        bind_node_strings( shape.node, parameters );
        bind_path( shape.material_path, parameters );
    }

    for( auto &lines : scene.lines ) {
        bind_node_strings( lines.node, parameters );
    }

    apply_include_parameters_to_models( scene.models, parameters );

    for( auto &cell : scene.memcells ) {
        bind_node_strings( cell.node, parameters );
        bind_name( cell.text, parameters );
        bind_optional_name( cell.track_name, parameters );
    }

    for( auto &launcher : scene.event_launchers ) {
        bind_node_strings( launcher.node, parameters );
        bind_name( launcher.activation_key_raw, parameters );
        bind_name( launcher.event1_name, parameters );
        bind_name( launcher.event2_name, parameters );
        if( launcher.condition ) {
            bind_name( launcher.condition->memcell_name, parameters );
            bind_name( launcher.condition->compare_text, parameters );
        }
    }

    for( auto &vehicle : scene.dynamics ) {
        bind_node_strings( vehicle.node, parameters );
        bind_path( vehicle.data_folder, parameters );
        bind_path( vehicle.skin_file, parameters );
        bind_path( vehicle.mmd_file, parameters );
        bind_name( vehicle.track_name, parameters );
        bind_name( vehicle.driver_type, parameters );
        bind_name( vehicle.coupling_raw, parameters );
        bind_path( vehicle.coupling_params, parameters );
        bind_name( vehicle.load_type, parameters );
        bind_optional_name( vehicle.destination, parameters );
    }

    for( auto &sound : scene.sounds ) {
        bind_node_strings( sound.node, parameters );
        bind_path( sound.wav_file, parameters );
    }

    for( auto &event : scene.events ) {
        bind_name( event.name, parameters );
        for( auto &target : event.targets ) {
            bind_name( target, parameters );
        }
        for( auto &[key, value] : event.payload ) {
            bind_name( key, parameters );
            bind_name( value, parameters );
        }
    }

    for( auto &trainset : scene.trainsets ) {
        bind_name( trainset.name, parameters );
        bind_name( trainset.track, parameters );
        std::unordered_map<std::string, std::string> rebound;
        rebound.reserve( trainset.assignment.size() );
        for( auto const &[key, value] : trainset.assignment ) {
            auto bound_key { key };
            auto bound_value { value };
            bind_name( bound_key, parameters );
            bind_name( bound_value, parameters );
            rebound.emplace( std::move( bound_key ), std::move( bound_value ) );
        }
        trainset.assignment = std::move( rebound );
    }
}

[[nodiscard]] double
resolve_placement_param(
    std::uint8_t const param_index,
    std::vector<std::string> const &parameters ) {
    if( param_index == 0 ) {
        return 0.0;
    }

    auto const index { static_cast<std::size_t>( param_index ) };
    if( index < 1 || index > parameters.size() ) {
        throw std::runtime_error( "EU7: brak parametru p" + std::to_string( param_index ) );
    }

    return std::stod( parameters[ index - 1 ] );
}

Eu7TransformContext
placement_transform_from_include_parameters(
    Eu7IncludePlacement const &binding,
    std::vector<std::string> const &parameters ) {
    Eu7TransformContext placement;
    if( binding.empty() ) {
        return placement;
    }

    try {
        auto const origin_x { resolve_placement_param( binding.origin_x_param, parameters ) };
        auto const origin_y { resolve_placement_param( binding.origin_y_param, parameters ) };
        auto const origin_z { resolve_placement_param( binding.origin_z_param, parameters ) };
        placement.origin_stack.push_back( { origin_x, origin_y, origin_z } );
        placement.rotation.y = resolve_placement_param( binding.rotation_y_param, parameters );
    }
    catch( std::exception const & ) {
        placement = {};
    }

    return placement;
}

void
apply_include_placement_to_scene(
    Eu7Scene &scene,
    Eu7IncludePlacement const &binding,
    std::vector<std::string> const &parameters ) {
    auto const placement {
        placement_transform_from_include_parameters( binding, parameters ) };
    if( false == transform_is_empty( placement ) ) {
        compose_scene_with_include_prefix( scene, placement );
    }
}

void
apply_include_placement_to_models(
    std::vector<Eu7Model> &models,
    Eu7IncludePlacement const &binding,
    std::vector<std::string> const &parameters ) {
    auto const placement {
        placement_transform_from_include_parameters( binding, parameters ) };
    if( false == transform_is_empty( placement ) ) {
        compose_models_with_prefix( models, placement );
    }
}

std::string
module_load_key(
    std::string const &resolved_path,
    std::vector<std::string> const &parameters ) {
    std::string key { resolved_path };
    for( auto const &parameter : parameters ) {
        key.push_back( '\0' );
        key += parameter;
    }
    return key;
}

} // namespace scene::eu7
