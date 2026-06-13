/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_emit.h"
#include "scene/eu7/eu7_transform.h"
#include "utilities/utilities.h"

#include <cmath>
#include <format>
#include <limits>
#include <sstream>

namespace scene::eu7 {
namespace {

[[nodiscard]] std::string
format_double( double const value ) {
    std::string text { std::format( "{:.4f}", value ) };
    if( auto const dot { text.find( '.' ) }; dot != std::string::npos ) {
        while( text.size() > dot + 1 && text.back() == '0' ) {
            text.pop_back();
        }
        if( text.back() == '.' ) {
            text.pop_back();
        }
    }
    return text;
}

class TextEmitter {
public:
    void
    push( std::string value ) {
        if( !first_on_line_ ) {
            out_ << ' ';
        }
        first_on_line_ = false;
        out_ << value;
    }

    void
    push( std::string_view const value ) {
        push( std::string( value ) );
    }

    void
    push( char const *const value ) {
        push( std::string_view( value ) );
    }

    void
    push( double const value ) {
        push( format_double( value ) );
    }

    void
    push( float const value ) {
        push( static_cast<double>( value ) );
    }

    void
    push( int const value ) {
        push( std::to_string( value ) );
    }

    void
    push_vec3( glm::dvec3 const &v ) {
        push( v.x );
        push( v.y );
        push( v.z );
    }

    void
    newline() {
        out_ << '\n';
        first_on_line_ = true;
    }

    [[nodiscard]] std::string
    str() const {
        return out_.str();
    }

private:
    std::ostringstream out_;
    bool first_on_line_ { true };
};

struct TransformEmitState {
    std::vector<glm::dvec3> origin_stack;
    std::vector<glm::dvec3> scale_stack;
    glm::dvec3 rotation{};
    std::size_t group_depth = 0;
};

[[nodiscard]] glm::dvec3
origin_push_delta( std::vector<glm::dvec3> const &stack, std::size_t const index ) {
    auto const &cumulative { stack[ index ] };
    if( index == 0 ) {
        return cumulative;
    }
    auto const &parent { stack[ index - 1 ] };
    return { cumulative.x - parent.x, cumulative.y - parent.y, cumulative.z - parent.z };
}

[[nodiscard]] glm::dvec3
scale_push_factor( std::vector<glm::dvec3> const &stack, std::size_t const index ) {
    auto const &cumulative { stack[ index ] };
    auto const parent { index == 0 ? glm::dvec3{ 1.0, 1.0, 1.0 } : stack[ index - 1 ] };
    return {
        parent.x != 0.0 ? cumulative.x / parent.x : cumulative.x,
        parent.y != 0.0 ? cumulative.y / parent.y : cumulative.y,
        parent.z != 0.0 ? cumulative.z / parent.z : cumulative.z };
}

void
sync_transform_stack( TextEmitter &w, TransformEmitState &state, Eu7TransformContext const &target ) {
    while( state.group_depth > target.group_depth ) {
        w.push( "endgroup" );
        --state.group_depth;
    }
    while( state.scale_stack.size() > target.scale_stack.size() ) {
        w.push( "endscale" );
        state.scale_stack.pop_back();
    }
    while( state.origin_stack.size() > target.origin_stack.size() ) {
        w.push( "endorigin" );
        state.origin_stack.pop_back();
    }

    while( state.origin_stack.size() < target.origin_stack.size() ) {
        auto const index { state.origin_stack.size() };
        auto const delta { origin_push_delta( target.origin_stack, index ) };
        w.push( "origin" );
        w.push_vec3( delta );
        state.origin_stack.push_back( target.origin_stack[ index ] );
    }

    while( state.scale_stack.size() < target.scale_stack.size() ) {
        auto const index { state.scale_stack.size() };
        auto const factor { scale_push_factor( target.scale_stack, index ) };
        w.push( "scale" );
        w.push_vec3( factor );
        state.scale_stack.push_back( target.scale_stack[ index ] );
    }

    while( state.group_depth < target.group_depth ) {
        w.push( "group" );
        ++state.group_depth;
    }

    if( state.rotation.x != target.rotation.x || state.rotation.y != target.rotation.y ||
        state.rotation.z != target.rotation.z ) {
        w.push( "rotate" );
        w.push_vec3( target.rotation );
        state.rotation = target.rotation;
    }
}

void
emit_node_transform( TextEmitter &w, TransformEmitState &state, Eu7BasicNode const &node ) {
    sync_transform_stack( w, state, node.transform );
}

[[nodiscard]] double
range_max_from_node( Eu7BasicNode const &node ) {
    if( node.range_squared_max >= std::numeric_limits<double>::max() * 0.5 ) {
        return -1.0;
    }
    return std::sqrt( node.range_squared_max );
}

[[nodiscard]] double
range_min_from_node( Eu7BasicNode const &node ) {
    return std::sqrt( node.range_squared_min );
}

[[nodiscard]] std::string
node_display_name( Eu7BasicNode const &node ) {
    return node.name.empty() ? "none" : node.name;
}

void
push_node_header( TextEmitter &w, Eu7BasicNode const &node ) {
    w.push( "node" );
    w.push( range_max_from_node( node ) );
    w.push( range_min_from_node( node ) );
    w.push( node_display_name( node ) );
}

[[nodiscard]] std::string_view
track_environment_name( Eu7TrackEnvironment const env ) {
    switch( env ) {
    case Eu7TrackEnvironment::Flat:
        return "flat";
    case Eu7TrackEnvironment::Mountains:
        return "mountains";
    case Eu7TrackEnvironment::Canyon:
        return "canyon";
    case Eu7TrackEnvironment::Tunnel:
        return "tunnel";
    case Eu7TrackEnvironment::Bridge:
        return "bridge";
    case Eu7TrackEnvironment::Bank:
        return "bank";
    default:
        return "flat";
    }
}

[[nodiscard]] std::string_view
track_kind_token( Eu7Track const &track ) {
    if( track.category == Eu7TrackCategory::Road ) {
        return "road";
    }
    if( track.category == Eu7TrackCategory::Water ) {
        return "river";
    }
    switch( track.track_type ) {
    case Eu7TrackType::Switch:
        return "switch";
    case Eu7TrackType::Cross:
        return "cross";
    case Eu7TrackType::Table:
        return "turn";
    case Eu7TrackType::Tributary:
        return "tributary";
    default:
        return "normal";
    }
}

void
push_track_core( TextEmitter &w, Eu7Track const &track ) {
    w.push( track.length );
    w.push( track.track_width );
    w.push( track.friction );
    w.push( track.sound_distance );
    w.push( track.quality_flag );
    w.push( track.damage_flag );
    w.push( track_environment_name( track.environment ) );
    if( track.visibility ) {
        w.push( "vis" );
        w.push( track.visibility->material1 );
        w.push( track.visibility->tex_length );
        w.push( track.visibility->material2 );
        w.push( track.visibility->tex_height1 );
        w.push( track.visibility->tex_width );
        w.push( track.visibility->tex_slope );
    }
    else {
        w.push( "unvis" );
    }
}

void
push_track_segments( TextEmitter &w, Eu7Track const &track, Eu7TransformContext const &transform ) {
    for( auto const &seg : track.paths ) {
        w.push_vec3( subtract_origin_offset( seg.p_start, transform ) );
        w.push( seg.roll_start );
        w.push_vec3( seg.cp_out );
        w.push_vec3( seg.cp_in );
        w.push_vec3( subtract_origin_offset( seg.p_end, transform ) );
        w.push( seg.roll_end );
        w.push( seg.radius );
    }
    for( auto const &[key, value] : track.tail_keywords ) {
        w.push( key );
        if( key == "sleepermodel" ) {
            std::size_t start { 0 };
            while( start < value.size() ) {
                auto const end { value.find( ' ', start ) };
                if( end == std::string::npos ) {
                    w.push( value.substr( start ) );
                    break;
                }
                w.push( value.substr( start, end - start ) );
                start = end + 1;
            }
        }
        else {
            w.push( value );
        }
    }
}

void
emit_track( TextEmitter &w, TransformEmitState &state, Eu7Track const &track ) {
    emit_node_transform( w, state, track.node );
    push_node_header( w, track.node );
    w.push( "track" );
    w.push( track_kind_token( track ) );
    push_track_core( w, track );
    push_track_segments( w, track, track.node.transform );
    w.push( "endtrack" );
    w.newline();
}

[[nodiscard]] std::string
traction_resistivity_token( Eu7Traction const &traction ) {
    if( traction.resistivity_legacy != 0.0 ) {
        return format_double( traction.resistivity_legacy );
    }
    return format_double( traction.resistivity_ohm_per_m / 0.001 );
}

[[nodiscard]] std::string
traction_material_token( Eu7Traction const &traction ) {
    if( !traction.material_raw.empty() ) {
        return traction.material_raw;
    }
    switch( traction.material ) {
    case Eu7TractionWireMaterial::Aluminium:
        return "al";
    case Eu7TractionWireMaterial::None:
        return "none";
    default:
        return "cu";
    }
}

void
emit_traction( TextEmitter &w, TransformEmitState &state, Eu7Traction const &traction ) {
    emit_node_transform( w, state, traction.node );
    auto const &transform { traction.node.transform };
    push_node_header( w, traction.node );
    w.push( "traction" );
    w.push( traction.power_supply_name );
    w.push( traction.nominal_voltage );
    w.push( traction.max_current );
    w.push( traction_resistivity_token( traction ) );
    w.push( traction_material_token( traction ) );
    w.push( traction.wire_thickness );
    w.push( traction.damage_flag );
    w.push_vec3( subtract_origin_offset( traction.wire_p1, transform ) );
    w.push_vec3( subtract_origin_offset( traction.wire_p2, transform ) );
    w.push_vec3( subtract_origin_offset( traction.wire_p3, transform ) );
    w.push_vec3( subtract_origin_offset( traction.wire_p4, transform ) );
    w.push( traction.min_height );
    w.push( traction.segment_length );
    w.push( traction.wire_count );
    w.push( traction.wire_offset );
    w.push( traction.node.visible ? "vis" : "unvis" );
    if( traction.parallel_name ) {
        w.push( "parallel" );
        w.push( *traction.parallel_name );
    }
    w.push( "endtraction" );
    w.newline();
}

void
emit_power_source( TextEmitter &w, TransformEmitState &state, Eu7TractionPowerSource const &source ) {
    emit_node_transform( w, state, source.node );
    push_node_header( w, source.node );
    w.push( "tractionpowersource" );
    w.push_vec3( inverse_transform_point( source.position, source.node.transform ) );
    w.push( source.nominal_voltage );
    w.push( source.voltage_frequency );
    w.push( source.internal_resistance_legacy );
    w.push( source.max_output_current );
    w.push( source.fast_fuse_timeout );
    w.push( source.fast_fuse_repetition );
    w.push( source.slow_fuse_timeout );
    if( source.modifier == Eu7PowerSourceModifier::Recuperation ) {
        w.push( "recuperation" );
    }
    else if( source.modifier == Eu7PowerSourceModifier::Section ) {
        w.push( "section" );
    }
    w.push( "end" );
    w.newline();
}

void
emit_shape( TextEmitter &w, TransformEmitState &state, Eu7Shape const &shape ) {
    emit_node_transform( w, state, shape.node );
    auto local_vertices { shape.vertices };
    inverse_transform_shape_vertices( local_vertices, shape.node.transform );
    push_node_header( w, shape.node );
    auto const shape_type {
        shape.node.node_type.empty() ? std::string_view{ "triangles" } :
                                       std::string_view{ shape.node.node_type } };
    w.push( shape_type );
    w.push( shape.material_path );
    auto const triangle_list { shape_type == "triangles" };
    for( std::size_t i { 0 }; i < local_vertices.size(); ++i ) {
        auto const &vert { local_vertices[ i ] };
        w.push_vec3( vert.position );
        w.push( static_cast<double>( vert.normal.x ) );
        w.push( static_cast<double>( vert.normal.y ) );
        w.push( static_cast<double>( vert.normal.z ) );
        w.push( vert.u );
        w.push( vert.v );
        if( triangle_list && i % 3 != 2 ) {
            w.push( "end" );
        }
    }
    w.push( "endtri" );
    w.newline();
}

void
emit_lines( TextEmitter &w, TransformEmitState &state, Eu7Lines const &lines ) {
    emit_node_transform( w, state, lines.node );
    auto local_vertices { lines.vertices };
    inverse_transform_shape_vertices( local_vertices, lines.node.transform );
    push_node_header( w, lines.node );
    w.push( "lines" );
    w.push( lines.lighting.diffuse.x * 255.0 );
    w.push( lines.lighting.diffuse.y * 255.0 );
    w.push( lines.lighting.diffuse.z * 255.0 );
    w.push( lines.line_width );
    for( std::size_t i { 0 }; i + 1 < local_vertices.size(); i += 2 ) {
        w.push_vec3( local_vertices[ i ].position );
        w.push_vec3( local_vertices[ i + 1 ].position );
    }
    w.push( "endline" );
    w.newline();
}

void
emit_model( TextEmitter &w, TransformEmitState &state, Eu7Model const &model ) {
    emit_node_transform( w, state, model.node );
    auto const local_location { inverse_transform_point( model.location, model.node.transform ) };
    auto const local_rotation_y { model.angles.y - model.node.transform.rotation.y };
    if( model.is_terrain ) {
        w.push( "node" );
        w.push( -1.0 );
        w.push( 0.0 );
        w.push( node_display_name( model.node ) );
    }
    else {
        push_node_header( w, model.node );
    }
    w.push( "model" );
    w.push_vec3( local_location );
    w.push( local_rotation_y );
    w.push( model.model_file );
    w.push( model.texture_file );
    w.push( "endmodel" );
    w.newline();
}

void
emit_memcell( TextEmitter &w, TransformEmitState &state, Eu7MemCell const &cell ) {
    emit_node_transform( w, state, cell.node );
    push_node_header( w, cell.node );
    w.push( "memcell" );
    w.push_vec3( inverse_transform_point( cell.node.area.center, cell.node.transform ) );
    w.push( cell.text );
    w.push( cell.value1 );
    w.push( cell.value2 );
    w.push( cell.track_name.value_or( "none" ) );
    w.push( "endmemcell" );
    w.newline();
}

[[nodiscard]] std::string
launcher_time_token( Eu7EventLauncher const &launcher ) {
    if( launcher.launch_hour != 0 || launcher.launch_minute != 0 ) {
        return std::format( "{}", launcher.launch_hour * 100 + launcher.launch_minute );
    }
    if( launcher.delta_time != 0.0 ) {
        return format_double( -launcher.delta_time );
    }
    return "0";
}

void
emit_launcher( TextEmitter &w, TransformEmitState &state, Eu7EventLauncher const &launcher ) {
    emit_node_transform( w, state, launcher.node );
    push_node_header( w, launcher.node );
    w.push( "eventlauncher" );
    w.push_vec3( inverse_transform_point( launcher.location, launcher.node.transform ) );
    w.push( std::sqrt( launcher.radius_squared ) );
    w.push( launcher.activation_key_raw );
    w.push( launcher_time_token( launcher ) );
    w.push( launcher.event1_name );
    if( !launcher.event2_name.empty() && launcher.event2_name != "none" ) {
        w.push( launcher.event2_name );
    }
    if( launcher.condition ) {
        w.push( "condition" );
        w.push( launcher.condition->memcell_name );
        w.push( launcher.condition->compare_text );
        if( launcher.condition->check_mask & 2 ) {
            w.push( launcher.condition->compare_value1 );
        }
        else {
            w.push( "*" );
        }
        if( launcher.condition->check_mask & 4 ) {
            w.push( launcher.condition->compare_value2 );
        }
        else {
            w.push( "*" );
        }
    }
    if( launcher.train_triggered ) {
        w.push( "traintriggered" );
    }
    w.push( "endeventlauncher" );
    w.newline();
}

void
emit_dynamic( TextEmitter &w, TransformEmitState &state, Eu7Dynamic const &vehicle, bool const in_trainset ) {
    emit_node_transform( w, state, vehicle.node );
    push_node_header( w, vehicle.node );
    w.push( "dynamic" );
    w.push( vehicle.data_folder );
    w.push( vehicle.skin_file );
    w.push( vehicle.mmd_file );
    if( !in_trainset ) {
        w.push( vehicle.track_name );
    }
    w.push( vehicle.offset );
    w.push( vehicle.driver_type );
    if( in_trainset ) {
        w.push( vehicle.coupling_raw.empty() ? std::to_string( vehicle.coupling ) : vehicle.coupling_raw );
    }
    if( !in_trainset ) {
        w.push( vehicle.velocity );
    }
    w.push( vehicle.load_count );
    if( !vehicle.load_type.empty() ) {
        w.push( vehicle.load_type );
    }
    if( vehicle.destination ) {
        w.push( *vehicle.destination );
    }
    w.push( "enddynamic" );
    w.newline();
}

void
emit_sound( TextEmitter &w, TransformEmitState &state, Eu7Sound const &sound ) {
    emit_node_transform( w, state, sound.node );
    push_node_header( w, sound.node );
    w.push( "sound" );
    w.push_vec3( inverse_transform_point( sound.location, sound.node.transform ) );
    w.push( sound.wav_file );
    w.push( "endsound" );
    w.newline();
}

[[nodiscard]] std::string_view
event_type_name( Eu7EventType const type ) {
    switch( type ) {
    case Eu7EventType::AddValues:
        return "addvalues";
    case Eu7EventType::UpdateValues:
        return "updatevalues";
    case Eu7EventType::CopyValues:
        return "copyvalues";
    case Eu7EventType::GetValues:
        return "getvalues";
    case Eu7EventType::PutValues:
        return "putvalues";
    case Eu7EventType::Whois:
        return "whois";
    case Eu7EventType::LogValues:
        return "logvalues";
    case Eu7EventType::Multiple:
        return "multiple";
    case Eu7EventType::Switch:
        return "switch";
    case Eu7EventType::TrackVel:
        return "trackvel";
    case Eu7EventType::Sound:
        return "sound";
    case Eu7EventType::Texture:
        return "texture";
    case Eu7EventType::Animation:
        return "animation";
    case Eu7EventType::Lights:
        return "lights";
    case Eu7EventType::Voltage:
        return "voltage";
    case Eu7EventType::Visible:
        return "visible";
    case Eu7EventType::Friction:
        return "friction";
    case Eu7EventType::Message:
        return "message";
    default:
        return "unknown";
    }
}

[[nodiscard]] std::string
join_targets( std::vector<std::string> const &targets ) {
    std::string joined;
    for( std::size_t i { 0 }; i < targets.size(); ++i ) {
        if( i > 0 ) {
            joined += '|';
        }
        joined += targets[ i ];
    }
    return joined.empty() ? "none" : joined;
}

void
emit_event( TextEmitter &w, Eu7Event const &event ) {
    w.push( "event" );
    if( event.ignored ) {
        w.push( std::string{ "none_" } + event.name );
    }
    else {
        w.push( event.name );
    }
    w.push( event_type_name( event.type ) );
    w.push( event.delay );
    w.push( join_targets( event.targets ) );
    for( auto const &[key, value] : event.payload ) {
        if( !key.empty() ) {
            w.push( key );
        }
        if( !value.empty() ) {
            w.push( value );
        }
    }
    if( event.delay_random != 0.0 ) {
        w.push( "randomdelay" );
        w.push( event.delay_random );
    }
    if( event.delay_departure != 0.0 ) {
        w.push( "departuredelay" );
        w.push( event.delay_departure );
    }
    if( event.passive ) {
        w.push( "passive" );
    }
    w.push( "endevent" );
    w.newline();
}

void
emit_trainset_block(
    TextEmitter &w,
    TransformEmitState &state,
    Eu7Trainset const &trainset,
    std::vector<Eu7Dynamic> const &dynamics ) {
    w.push( "trainset" );
    w.push( trainset.name );
    w.push( trainset.track );
    w.push( trainset.offset );
    w.push( trainset.velocity );
    if( false == trainset.assignment.empty() ) {
        w.push( "assignment" );
        for( auto const &[lang, assignment] : trainset.assignment ) {
            w.push( lang );
            w.push( '"' + assignment + '"' );
        }
        w.push( "endassignment" );
    }
    std::size_t vehicle_slot { 0 };
    for( auto const index : trainset.vehicle_indices ) {
        if( index < dynamics.size() ) {
            auto vehicle { dynamics[ index ] };
            if( vehicle_slot < trainset.couplings.size() ) {
                vehicle.coupling_raw = std::to_string( trainset.couplings[ vehicle_slot ] );
            }
            emit_dynamic( w, state, vehicle, true );
            ++vehicle_slot;
        }
    }
    w.push( "endtrainset" );
    w.newline();
}

} // namespace

std::string
include_text_path( Eu7Include const &include ) {
    if( !include.source_path.empty() ) {
        return include.source_path;
    }
    auto path { include.binary_path };
    replace_slashes( path );
    if( path.ends_with( ".eu7" ) ) {
        path.replace( path.size() - 4, 4, ".inc" );
    }
    return path;
}

std::string
emit_includes_text( std::vector<Eu7Include> const &includes ) {
    if( includes.empty() ) {
        return {};
    }
    TextEmitter w;
    for( auto const &include : includes ) {
        w.push( "include" );
        w.push( include_text_path( include ) );
        for( auto const &param : include.parameters ) {
            w.push( param );
        }
        w.push( "end" );
        w.newline();
    }
    return w.str();
}

std::string
emit_scene_objects_text( Eu7Module const &module ) {
    TextEmitter w;
    TransformEmitState transform_state;
    auto const &scene { module.scene };

    for( auto const &track : scene.tracks ) {
        emit_track( w, transform_state, track );
    }
    for( auto const &traction : scene.traction ) {
        emit_traction( w, transform_state, traction );
    }
    for( auto const &source : scene.power_sources ) {
        emit_power_source( w, transform_state, source );
    }
    for( auto const &model : scene.models ) {
        emit_model( w, transform_state, model );
    }
    for( auto const &cell : scene.memcells ) {
        emit_memcell( w, transform_state, cell );
    }
    for( auto const &launcher : scene.event_launchers ) {
        emit_launcher( w, transform_state, launcher );
    }

    std::vector<bool> emitted_in_trainset( scene.dynamics.size(), false );
    for( auto const &trainset : scene.trainsets ) {
        for( auto const index : trainset.vehicle_indices ) {
            if( index < emitted_in_trainset.size() ) {
                emitted_in_trainset[ index ] = true;
            }
        }
    }
    for( std::size_t i { 0 }; i < scene.dynamics.size(); ++i ) {
        if( !emitted_in_trainset[ i ] ) {
            emit_dynamic( w, transform_state, scene.dynamics[ i ], false );
        }
    }

    for( auto const &sound : scene.sounds ) {
        emit_sound( w, transform_state, sound );
    }

    for( auto const &event : scene.events ) {
        emit_event( w, event );
    }

    for( std::uint32_t i { 0 }; i < scene.first_init_count; ++i ) {
        w.push( "FirstInit" );
        w.newline();
    }

    for( auto const &trainset : scene.trainsets ) {
        emit_trainset_block( w, transform_state, trainset, scene.dynamics );
    }

    return w.str();
}

std::string
emit_module_text( Eu7Module const &module ) {
    TextEmitter w;
    TransformEmitState transform_state;
    auto const &scene { module.scene };

    for( auto const &shape : scene.shapes ) {
        emit_shape( w, transform_state, shape );
    }
    for( auto const &lines : scene.lines ) {
        emit_lines( w, transform_state, lines );
    }

    std::string text { w.str() };
    text += emit_scene_objects_text( module );
    return text;
}

} // namespace scene::eu7
