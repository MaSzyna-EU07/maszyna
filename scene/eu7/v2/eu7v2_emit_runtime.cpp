/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "scene/eu7/v2/eu7v2_emit_runtime.h"

#include "scene/eu7/v2/eu7v2_format.h"
#include "scene/eu7/v2/eu7v2_scene.h"
#include "scene/eu7/v2/eu7v2_records.h"

#include <eu07/scene/bake/pack_model_spool.hpp>
#include <eu07/scene/include_resolve.hpp>
#include <eu07/scene/runtime/scene.hpp>

#include <chrono>
#include <fstream>
#include <limits>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace eu7v2 {

namespace {

namespace rt = eu07::scene::runtime;
namespace bake = eu07::scene::bake;
namespace codec = eu07::scene::binary::codec;

[[nodiscard]] bool is_model_inc_placement( bake::ModuleInclude const &inc ) {
    if( !eu07::scene::detail::isIncFile( inc.sourcePath ) ) {
        return false;
    }
    if( inc.parameters.size() < 5 ) {
        return false;
    }
    try {
        (void)std::stod( inc.parameters[ 1 ] );
        (void)std::stod( inc.parameters[ 2 ] );
        (void)std::stod( inc.parameters[ 3 ] );
        (void)std::stod( inc.parameters[ 4 ] );
        return true;
    } catch( ... ) {
        return false;
    }
}

[[nodiscard]] bool try_parse_inc_placement(
    std::span<std::string const> const params,
    double &x,
    double &y,
    double &z,
    float &rot_y ) {
    if( params.size() < 5 ) {
        return false;
    }
    try {
        x = std::stod( params[ 1 ] );
        y = std::stod( params[ 2 ] );
        z = std::stod( params[ 3 ] );
        rot_y = static_cast<float>( std::stod( params[ 4 ] ) );
        return true;
    } catch( ... ) {
        return false;
    }
}

[[nodiscard]] file_kind file_kind_for_text_path( std::filesystem::path const &text_path ) {
    auto const ext { text_path.extension().string() };
    if( ext == ".inc" ) {
        return file_kind::module;
    }
    return file_kind::sim;
}

// Mirror of scene_baker but sourced from the parser's Runtime* records.
class runtime_baker {
  public:
    runtime_baker(
        bake::RuntimeModule const &module,
        bool const is_root,
        std::vector<codec::ModelSectionBatch> const *pack_batches,
        bake::PackModelSpoolFile const *pack_spool,
        bake::ShapeSpoolFile const *shape_spool,
        std::filesystem::path const &text_path )
        : m_module( module )
        , m_is_root( is_root )
        , m_pack_batches( pack_batches )
        , m_pack_spool( pack_spool )
        , m_shape_spool( shape_spool )
        , m_file_kind( file_kind_for_text_path( text_path ) ) {}

    [[nodiscard]] std::vector<std::uint8_t> run() {
        convert_models();
        convert_shapes();
        convert_lines();
        convert_tracks();
        convert_traction();
        convert_power_sources();
        convert_memcells();
        convert_launchers();
        convert_events();
        convert_sounds();
        convert_dynamics();
        convert_trainsets();
        convert_includes();
        convert_meta();
        return serialize();
    }

    [[nodiscard]] std::size_t model_total() const { return m_instances.size(); }
    [[nodiscard]] std::size_t placement_total() const { return m_placements.size(); }
    [[nodiscard]] std::size_t structural_include_total() const { return m_includes.size(); }

  private:
    [[nodiscard]] std::uint32_t str( std::string const &s ) { return m_strings.intern( s ); }
    [[nodiscard]] std::uint32_t opt_str( std::string const &s ) {
        return s.empty() ? kNoString : m_strings.intern( s );
    }

    [[nodiscard]] node_record node( rt::BasicNode const &n ) {
        node_record out;
        out.name = opt_str( n.name );
        out.type = opt_str( n.nodeType );
        out.area_center = { n.area.center.x, n.area.center.y, n.area.center.z };
        out.area_radius = n.area.radius;
        out.range_sq_min = n.rangeSquaredMin;
        out.range_sq_max = n.rangeSquaredMax;
        out.visible = n.visible;
        return out;
    }

    [[nodiscard]] lighting_block lighting( rt::LightingData const &l ) {
        lighting_block out;
        out.diffuse[ 0 ] = l.diffuse.x; out.diffuse[ 1 ] = l.diffuse.y;
        out.diffuse[ 2 ] = l.diffuse.z; out.diffuse[ 3 ] = l.diffuse.w;
        out.ambient[ 0 ] = l.ambient.x; out.ambient[ 1 ] = l.ambient.y;
        out.ambient[ 2 ] = l.ambient.z; out.ambient[ 3 ] = l.ambient.w;
        out.specular[ 0 ] = l.specular.x; out.specular[ 1 ] = l.specular.y;
        out.specular[ 2 ] = l.specular.z; out.specular[ 3 ] = l.specular.w;
        return out;
    }

    [[nodiscard]] std::uint32_t intern_prototype( rt::RuntimeModelInstance const &model ) {
        std::string key;
        key.reserve( model.modelFile.size() + model.textureFile.size() + 32 );
        key.append( model.modelFile );
        key.push_back( '\x1f' );
        key.append( model.textureFile );
        key.push_back( '\x1f' );
        key.push_back( model.transition ? '1' : '0' );
        key.push_back( model.isTerrain ? '1' : '0' );
        for( auto const s : model.lightStates ) {
            key.append( std::to_string( s ) );
            key.push_back( ',' );
        }
        key.push_back( '\x1f' );
        for( auto const c : model.lightColors ) {
            key.append( std::to_string( c ) );
            key.push_back( ',' );
        }

        auto const it { m_prototype_lookup.find( key ) };
        if( it != m_prototype_lookup.end() ) {
            return it->second;
        }

        model_prototype proto;
        proto.model_file = str( model.modelFile );
        proto.texture_file = opt_str( model.textureFile );
        proto.flags = 0;
        if( model.transition ) {
            proto.flags |= proto_flag::transition;
        }
        if( model.isTerrain ) {
            proto.flags |= proto_flag::is_terrain;
        }
        proto.range_min = -1.f;
        proto.range_max = -1.f;
        proto.light_states = model.lightStates;
        proto.light_colors = model.lightColors;

        auto const id { static_cast<std::uint32_t>( m_prototypes.size() ) };
        m_prototypes.push_back( std::move( proto ) );
        m_prototype_lookup.emplace( std::move( key ), id );
        return id;
    }

    void emit_model( rt::RuntimeModelInstance const &model ) {
        auto const proto { intern_prototype( model ) };
        model_instance inst;
        inst.proto = proto;
        inst.x = model.location.x;
        inst.y = model.location.y;
        inst.z = model.location.z;
        inst.ax = static_cast<float>( model.angles.x );
        inst.ay = static_cast<float>( model.angles.y );
        inst.az = static_cast<float>( model.angles.z );
        inst.sx = static_cast<float>( model.scale.x );
        inst.sy = static_cast<float>( model.scale.y );
        inst.sz = static_cast<float>( model.scale.z );
        inst.cell_id = 0xffu;
        inst.texture_override = kNoString;
        inst.has_node = true;
        inst.node = node( model.node );
        m_instances.push_back( std::move( inst ) );
    }

    void convert_models() {
        if( m_is_root && m_pack_spool != nullptr ) {
            m_pack_spool->for_each_model( [&]( rt::RuntimeModelInstance const &model ) {
                emit_model( model );
            } );
        }
        if( m_is_root && m_pack_batches != nullptr ) {
            // Root: models live in the flattened PACK batches, not scene.models.
            for( auto const &batch : *m_pack_batches ) {
                for( auto const &model : batch.models ) {
                    emit_model( model );
                }
            }
        }
        for( auto const &model : m_module.scene.models ) {
            emit_model( model );
        }
    }

    [[nodiscard]] shape_record make_shape_record( rt::RuntimeShapeNode const &shape ) {
        shape_record r;
        r.node = node( shape.node );
        r.translucent = shape.translucent;
        r.material = opt_str( shape.materialPath );
        r.lighting = lighting( shape.lighting );
        r.ox = shape.origin.x;
        r.oy = shape.origin.y;
        r.oz = shape.origin.z;
        r.vertices.reserve( shape.vertices.size() );
        for( auto const &v : shape.vertices ) {
            mesh_vertex mv;
            mv.px = static_cast<float>( v.position.x - shape.origin.x );
            mv.py = static_cast<float>( v.position.y - shape.origin.y );
            mv.pz = static_cast<float>( v.position.z - shape.origin.z );
            mv.nx = static_cast<float>( v.normal.x );
            mv.ny = static_cast<float>( v.normal.y );
            mv.nz = static_cast<float>( v.normal.z );
            mv.u = static_cast<float>( v.u );
            mv.v = static_cast<float>( v.v );
            r.vertices.push_back( mv );
        }
        return r;
    }

    void convert_shapes() {
        if( m_shape_spool != nullptr ) {
            m_shape_spool->for_each_shape( [&]( rt::RuntimeShapeNode const &shape ) {
                (void)opt_str( shape.materialPath );
                (void)node( shape.node );
            } );
            return;
        }
        for( auto const &shape : m_module.scene.shapes ) {
            m_shapes.push_back( make_shape_record( shape ) );
        }
    }

    void convert_lines() {
        for( auto const &line : m_module.scene.lines ) {
            lines_record r;
            r.node = node( line.node );
            r.lighting = lighting( line.lighting );
            r.line_width = line.lineWidth;
            r.ox = line.origin.x;
            r.oy = line.origin.y;
            r.oz = line.origin.z;
            r.vertices.reserve( line.vertices.size() );
            for( auto const &v : line.vertices ) {
                r.vertices.push_back( { v.position.x, v.position.y, v.position.z } );
            }
            m_lines.push_back( std::move( r ) );
        }
    }

    void convert_tracks() {
        for( auto const &t : m_module.scene.tracks ) {
            track_record r;
            r.node = node( t.node );
            r.track_type = static_cast<std::uint8_t>( t.trackType );
            r.category = static_cast<std::uint8_t>( t.category );
            r.length = t.length;
            r.track_width = t.trackWidth;
            r.friction = t.friction;
            r.sound_distance = t.soundDistance;
            r.quality_flag = t.qualityFlag;
            r.damage_flag = t.damageFlag;
            r.environment = static_cast<std::int8_t>( t.environment );
            if( t.visibility.has_value() ) {
                r.has_visibility = true;
                r.visibility.material1 = str( t.visibility->material1 );
                r.visibility.tex_length = t.visibility->texLength;
                r.visibility.material2 = str( t.visibility->material2 );
                r.visibility.tex_height1 = t.visibility->texHeight1;
                r.visibility.tex_width = t.visibility->texWidth;
                r.visibility.tex_slope = t.visibility->texSlope;
            }
            r.paths.reserve( t.paths.size() );
            for( auto const &p : t.paths ) {
                track_path tp;
                tp.p_start = { p.pStart.x, p.pStart.y, p.pStart.z };
                tp.roll_start = p.rollStart;
                tp.cp_out = { p.cpOut.x, p.cpOut.y, p.cpOut.z };
                tp.cp_in = { p.cpIn.x, p.cpIn.y, p.cpIn.z };
                tp.p_end = { p.pEnd.x, p.pEnd.y, p.pEnd.z };
                tp.roll_end = p.rollEnd;
                tp.radius = p.radius;
                r.paths.push_back( tp );
            }
            r.tail_keywords.reserve( t.tailKeywords.size() );
            for( auto const &kv : t.tailKeywords ) {
                r.tail_keywords.emplace_back( str( kv.first ), str( kv.second ) );
            }
            m_tracks.push_back( std::move( r ) );
        }
    }

    void convert_traction() {
        for( auto const &t : m_module.scene.traction ) {
            traction_record r;
            r.node = node( t.node );
            r.power_supply_name = opt_str( t.powerSupplyName );
            r.nominal_voltage = t.nominalVoltage;
            r.max_current = t.maxCurrent;
            r.resistivity = t.resistivityOhmPerM;
            r.material = static_cast<std::uint8_t>( t.material );
            r.wire_thickness = t.wireThickness;
            r.damage_flag = t.damageFlag;
            r.wire_p1 = { t.wireP1.x, t.wireP1.y, t.wireP1.z };
            r.wire_p2 = { t.wireP2.x, t.wireP2.y, t.wireP2.z };
            r.wire_p3 = { t.wireP3.x, t.wireP3.y, t.wireP3.z };
            r.wire_p4 = { t.wireP4.x, t.wireP4.y, t.wireP4.z };
            r.min_height = t.minHeight;
            r.segment_length = t.segmentLength;
            r.wire_count = t.wireCount;
            r.wire_offset = t.wireOffset;
            if( t.parallelName.has_value() ) {
                r.has_parallel = true;
                r.parallel_name = str( *t.parallelName );
            }
            m_traction.push_back( std::move( r ) );
        }
    }

    void convert_power_sources() {
        for( auto const &p : m_module.scene.powerSources ) {
            power_source_record r;
            r.node = node( p.node );
            r.position = { p.position.x, p.position.y, p.position.z };
            r.nominal_voltage = p.nominalVoltage;
            r.voltage_frequency = p.voltageFrequency;
            r.internal_resistance = p.internalResistance;
            r.max_output_current = p.maxOutputCurrent;
            r.fast_fuse_timeout = p.fastFuseTimeout;
            r.fast_fuse_repetition = p.fastFuseRepetition;
            r.slow_fuse_timeout = p.slowFuseTimeout;
            r.modifier = static_cast<std::uint8_t>( p.modifier );
            m_power.push_back( std::move( r ) );
        }
    }

    void convert_memcells() {
        for( auto const &m : m_module.scene.memcells ) {
            memcell_record r;
            r.node = node( m.node );
            r.text = opt_str( m.text );
            r.value1 = m.value1;
            r.value2 = m.value2;
            if( m.trackName.has_value() ) {
                r.has_track = true;
                r.track_name = str( *m.trackName );
            }
            m_memcells.push_back( std::move( r ) );
        }
    }

    void convert_launchers() {
        for( auto const &l : m_module.scene.eventLaunchers ) {
            launcher_record r;
            r.node = node( l.node );
            r.location = { l.location.x, l.location.y, l.location.z };
            r.radius_squared = l.radiusSquared;
            r.activation_key = l.activationKey;
            r.delta_time = l.deltaTime;
            r.event1_name = opt_str( l.event1Name );
            r.event2_name = opt_str( l.event2Name );
            if( l.condition.has_value() ) {
                r.has_condition = true;
                r.condition.memcell_name = str( l.condition->memcellName );
                r.condition.compare_text = str( l.condition->compareText );
                r.condition.compare_value1 = l.condition->compareValue1;
                r.condition.compare_value2 = l.condition->compareValue2;
                r.condition.check_mask = l.condition->checkMask;
            }
            r.train_triggered = l.trainTriggered;
            r.launch_hour = l.launchHour;
            r.launch_minute = l.launchMinute;
            m_launchers.push_back( std::move( r ) );
        }
    }

    void convert_events() {
        for( auto const &e : m_module.scene.events ) {
            event_record r;
            r.name = opt_str( e.name );
            r.type = static_cast<std::uint8_t>( e.type );
            r.delay = e.delay;
            r.delay_random = e.delayRandom;
            r.delay_departure = e.delayDeparture;
            r.ignored = e.ignored;
            r.passive = e.passive;
            r.targets.reserve( e.targets.size() );
            for( auto const &t : e.targets ) {
                r.targets.push_back( str( t ) );
            }
            r.payload.reserve( e.payload.size() );
            for( auto const &kv : e.payload ) {
                r.payload.emplace_back( str( kv.first ), str( kv.second ) );
            }
            m_events.push_back( std::move( r ) );
        }
    }

    void convert_sounds() {
        for( auto const &s : m_module.scene.sounds ) {
            sound_record r;
            r.node = node( s.node );
            r.location = { s.location.x, s.location.y, s.location.z };
            r.wav_file = opt_str( s.wavFile );
            m_sounds.push_back( std::move( r ) );
        }
    }

    void convert_dynamics() {
        for( auto const &d : m_module.scene.dynamics ) {
            dynamic_record r;
            r.node = node( d.node );
            r.data_folder = opt_str( d.dataFolder );
            r.skin_file = opt_str( d.skinFile );
            r.mmd_file = opt_str( d.mmdFile );
            r.track_name = opt_str( d.trackName );
            r.offset = d.offset;
            r.driver_type = opt_str( d.driverType );
            r.coupling = d.coupling;
            r.coupling_raw = opt_str( d.couplingRaw );
            r.coupling_params = opt_str( d.couplingParams );
            r.velocity = d.velocity;
            r.load_count = d.loadCount;
            r.load_type = opt_str( d.loadType );
            if( d.destination.has_value() ) {
                r.has_destination = true;
                r.destination = str( *d.destination );
            }
            if( d.trainsetIndex.has_value() ) {
                r.has_trainset = true;
                r.trainset_index = static_cast<std::uint32_t>( *d.trainsetIndex );
            }
            m_dynamics.push_back( std::move( r ) );
        }
    }

    void convert_trainsets() {
        for( auto const &t : m_module.scene.trainsets ) {
            trainset_record r;
            r.name = opt_str( t.name );
            r.track = opt_str( t.track );
            r.offset = t.offset;
            r.velocity = t.velocity;
            r.assignment.reserve( t.assignment.size() );
            for( auto const &kv : t.assignment ) {
                r.assignment.emplace_back( str( kv.first ), str( kv.second ) );
            }
            r.vehicle_indices.reserve( t.vehicleIndices.size() );
            for( auto const idx : t.vehicleIndices ) {
                r.vehicle_indices.push_back( static_cast<std::uint32_t>( idx ) );
            }
            r.couplings.reserve( t.couplings.size() );
            for( auto const c : t.couplings ) {
                r.couplings.push_back( c );
            }
            r.driver_index =
                t.driverIndex == static_cast<std::size_t>( -1 )
                    ? 0xffffffffu
                    : static_cast<std::uint32_t>( t.driverIndex );
            m_trainsets.push_back( std::move( r ) );
        }
    }

    [[nodiscard]] transform_record transform( rt::TransformContext const &t ) {
        transform_record out;
        out.origin_stack.reserve( t.originStack.size() );
        for( auto const &v : t.originStack ) {
            out.origin_stack.push_back( { v.x, v.y, v.z } );
        }
        out.scale_stack.reserve( t.scaleStack.size() );
        for( auto const &v : t.scaleStack ) {
            out.scale_stack.push_back( { v.x, v.y, v.z } );
        }
        out.rotation = { t.rotation.x, t.rotation.y, t.rotation.z };
        out.group_depth = static_cast<std::uint32_t>( t.groupStackDepth );
        return out;
    }

    void convert_includes() {
        for( auto const &inc : m_module.includes ) {
            if( is_model_inc_placement( inc ) ) {
                double x { 0.0 }, y { 0.0 }, z { 0.0 };
                float rot_y { 0.f };
                if( !try_parse_inc_placement( inc.parameters, x, y, z, rot_y ) ) {
                    continue;
                }
                module_placement_record p;
                p.module_path = str(
                    binary_path_from_text( std::filesystem::path { inc.sourcePath } )
                        .generic_string() );
                p.texture_override =
                    ( inc.parameters.empty() || inc.parameters[ 0 ] == "none" )
                        ? kNoString
                        : str( inc.parameters[ 0 ] );
                p.x = x;
                p.y = y;
                p.z = z;
                p.rotation_y = rot_y;
                m_placements.push_back( std::move( p ) );
                continue;
            }
            include_record r;
            r.source_line = inc.sourceLine;
            r.source_path = opt_str( inc.sourcePath );
            r.binary_path = inc.sourcePath.empty()
                                ? kNoString
                                : str( binary_path_from_text(
                                           std::filesystem::path { inc.sourcePath } )
                                           .generic_string() );
            r.parameters.reserve( inc.parameters.size() );
            for( auto const &param : inc.parameters ) {
                r.parameters.push_back( str( param ) );
            }
            r.site_transform = transform( inc.siteTransform );
            m_includes.push_back( std::move( r ) );
        }
    }

    void convert_meta() {
        m_meta.first_init_count = m_module.scene.firstInitCount;
        m_meta.has_terrain_chunk = false;
        // Root PACK batches are flattened into plain INST records, so the file
        // carries no PACK streaming chunk; never advertise one.
        m_meta.has_pack_chunk = false;
        m_meta.placement_origin_x = m_module.includePlacement.origin_x_param;
        m_meta.placement_origin_y = m_module.includePlacement.origin_y_param;
        m_meta.placement_origin_z = m_module.includePlacement.origin_z_param;
        m_meta.placement_rotation_y = m_module.includePlacement.rotation_y_param;
    }

    [[nodiscard]] std::vector<std::uint8_t> serialize() {
        container_writer writer( m_file_kind );

        byte_writer strs;
        m_strings.serialize( strs );
        writer.add_chunk( chunk::strs, strs );

        byte_writer meta;
        write_meta( meta, m_meta );
        writer.add_chunk( chunk::meta, meta );

        auto emit { [&]( std::uint32_t id, auto const &write_fn, auto const &items ) {
            if( items.empty() ) {
                return;
            }
            byte_writer payload;
            write_fn( payload, items );
            writer.add_chunk( id, payload );
        } };

        emit( chunk::incl, &write_includes, m_includes );
        emit( chunk::plce, &write_module_placements, m_placements );
        emit( chunk::prot, &write_prototypes, m_prototypes );
        emit( chunk::inst, &write_instances, m_instances );
        if( m_shape_spool != nullptr && m_shape_spool->shape_count() != 0 ) {
            byte_writer payload;
            payload.put_u32( static_cast<std::uint32_t>( m_shape_spool->shape_count() ) );
            m_shape_spool->for_each_shape( [&]( rt::RuntimeShapeNode const &shape ) {
                write_shape_record( payload, make_shape_record( shape ) );
            } );
            writer.add_chunk( chunk::shpe, payload );
        } else {
            emit( chunk::shpe, &write_shapes, m_shapes );
        }
        emit( chunk::line, &write_lines, m_lines );
        emit( chunk::trak, &write_tracks, m_tracks );
        emit( chunk::trac, &write_traction, m_traction );
        emit( chunk::pwrs, &write_power_sources, m_power );
        emit( chunk::memc, &write_memcells, m_memcells );
        emit( chunk::laun, &write_launchers, m_launchers );
        emit( chunk::evnt, &write_events, m_events );
        emit( chunk::sond, &write_sounds, m_sounds );
        emit( chunk::dynm, &write_dynamics, m_dynamics );
        emit( chunk::trst, &write_trainsets, m_trainsets );

        return writer.data();
    }

    bake::RuntimeModule const &m_module;
    bool m_is_root;
    std::vector<codec::ModelSectionBatch> const *m_pack_batches;
    bake::PackModelSpoolFile const *m_pack_spool;
    bake::ShapeSpoolFile const *m_shape_spool;
    file_kind m_file_kind { file_kind::sim };
    std::vector<module_placement_record> m_placements;
    string_table m_strings;

    std::vector<model_prototype> m_prototypes;
    std::unordered_map<std::string, std::uint32_t> m_prototype_lookup;
    std::vector<model_instance> m_instances;
    std::vector<shape_record> m_shapes;
    std::vector<lines_record> m_lines;
    std::vector<track_record> m_tracks;
    std::vector<traction_record> m_traction;
    std::vector<power_source_record> m_power;
    std::vector<memcell_record> m_memcells;
    std::vector<launcher_record> m_launchers;
    std::vector<event_record> m_events;
    std::vector<sound_record> m_sounds;
    std::vector<dynamic_record> m_dynamics;
    std::vector<trainset_record> m_trainsets;
    std::vector<include_record> m_includes;
    module_meta m_meta;
};

// Decoded record counts of an emitted eu7v2 image (chunk-by-chunk).
struct decoded_counts {
    std::size_t includes { 0 };
    std::size_t placements { 0 };
    std::size_t instances { 0 };
    std::size_t shapes { 0 };
    std::size_t lines { 0 };
    std::size_t tracks { 0 };
    std::size_t traction { 0 };
    std::size_t power { 0 };
    std::size_t memcells { 0 };
    std::size_t launchers { 0 };
    std::size_t events { 0 };
    std::size_t sounds { 0 };
    std::size_t dynamics { 0 };
    std::size_t trainsets { 0 };
    bool ok { true };
};

[[nodiscard]] decoded_counts decode_counts( std::vector<std::uint8_t> const &bytes ) {
    decoded_counts c;
    try {
        container_reader reader( bytes.data(), bytes.size() );
        chunk_view chunk;
        while( reader.next( chunk ) ) {
            auto r { chunk.reader() };
            switch( chunk.id ) {
                case chunk::incl: c.includes = read_includes( r ).size(); break;
                case chunk::plce: c.placements = read_module_placements( r ).size(); break;
                case chunk::inst: c.instances = read_instances( r ).size(); break;
                case chunk::shpe: c.shapes = read_shapes( r ).size(); break;
                case chunk::line: c.lines = read_lines( r ).size(); break;
                case chunk::trak: c.tracks = read_tracks( r ).size(); break;
                case chunk::trac: c.traction = read_traction( r ).size(); break;
                case chunk::pwrs: c.power = read_power_sources( r ).size(); break;
                case chunk::memc: c.memcells = read_memcells( r ).size(); break;
                case chunk::laun: c.launchers = read_launchers( r ).size(); break;
                case chunk::evnt: c.events = read_events( r ).size(); break;
                case chunk::sond: c.sounds = read_sounds( r ).size(); break;
                case chunk::dynm: c.dynamics = read_dynamics( r ).size(); break;
                case chunk::trst: c.trainsets = read_trainsets( r ).size(); break;
                default: break;
            }
        }
    }
    catch( parse_error const & ) {
        c.ok = false;
    }
    return c;
}

[[nodiscard]] std::filesystem::path eu7v2_path_for( std::filesystem::path const &text_path ) {
    return binary_path_from_text( text_path );
}

} // namespace

std::vector<std::uint8_t>
emit_runtime_module_bytes(
    bake::RuntimeModule const &module,
    bool const is_root,
    std::vector<codec::ModelSectionBatch> const *pack_batches,
    bake::PackModelSpoolFile const *pack_spool,
    bake::ShapeSpoolFile const *shape_spool,
    std::filesystem::path const &text_path ) {
    runtime_baker baker( module, is_root, pack_batches, pack_spool, shape_spool, text_path );
    return baker.run();
}

emit_outcome
emit_runtime_module(
    bake::RuntimeModule const &module,
    std::filesystem::path const &text_path,
    bool const is_root,
    std::vector<codec::ModelSectionBatch> const *pack_batches,
    bool const verify,
    bake::PackModelSpoolFile const *pack_spool,
    bake::ShapeSpoolFile const *shape_spool ) {
    emit_outcome outcome;

    auto const ms_since { []( std::chrono::steady_clock::time_point const t0 ) {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t0 )
            .count();
    } };

    auto const build_begin { std::chrono::steady_clock::now() };
    runtime_baker baker( module, is_root, pack_batches, pack_spool, shape_spool, text_path );
    std::vector<std::uint8_t> const bytes { baker.run() };
    outcome.model_total = baker.model_total();
    outcome.byte_size = bytes.size();
    outcome.build_ms = ms_since( build_begin );

    auto const out_path { eu7v2_path_for( text_path ) };
    outcome.written_path = out_path.generic_string();

    {
        auto const write_begin { std::chrono::steady_clock::now() };
        std::ofstream output { out_path, std::ios::binary | std::ios::trunc };
        if( !output ) {
            outcome.ok = false;
            outcome.message = "nie mozna zapisac " + outcome.written_path;
            return outcome;
        }
        output.write(
            reinterpret_cast<char const *>( bytes.data() ),
            static_cast<std::streamsize>( bytes.size() ) );
        output.flush();
        outcome.write_ms = ms_since( write_begin );
    }

    if( !verify ) {
        return outcome;
    }

    auto const verify_begin { std::chrono::steady_clock::now() };
    outcome.verified = true;
    decoded_counts const dec { decode_counts( bytes ) };

    std::size_t src_models { module.scene.models.size() };
    if( is_root && pack_spool != nullptr ) {
        src_models += pack_spool->model_count();
    }
    if( is_root && pack_batches != nullptr ) {
        for( auto const &batch : *pack_batches ) {
            src_models += batch.models.size();
        }
    }

    std::size_t src_placements { 0 };
    std::size_t src_structural { 0 };
    for( auto const &inc : module.includes ) {
        if( is_model_inc_placement( inc ) ) {
            ++src_placements;
        } else {
            ++src_structural;
        }
    }

    std::ostringstream report;
    bool pass { dec.ok };
    auto cmp { [&]( char const *label, std::size_t const src, std::size_t const got ) {
        bool const ok { src == got };
        pass = pass && ok;
        report << "    " << ( ok ? "ok  " : "FAIL" ) << ' ' << label << " src=" << src
               << " eu7v2=" << got << '\n';
    } };

    cmp( "includes", src_structural, dec.includes );
    cmp( "placements", src_placements, dec.placements );
    cmp( "models", src_models, dec.instances );
    cmp( "shapes", module.scene.shapes.size() + ( shape_spool != nullptr ? shape_spool->shape_count() : 0 ),
         dec.shapes );
    cmp( "lines", module.scene.lines.size(), dec.lines );
    cmp( "tracks", module.scene.tracks.size(), dec.tracks );
    cmp( "traction", module.scene.traction.size(), dec.traction );
    cmp( "power", module.scene.powerSources.size(), dec.power );
    cmp( "memcells", module.scene.memcells.size(), dec.memcells );
    cmp( "launchers", module.scene.eventLaunchers.size(), dec.launchers );
    cmp( "events", module.scene.events.size(), dec.events );
    cmp( "sounds", module.scene.sounds.size(), dec.sounds );
    cmp( "dynamics", module.scene.dynamics.size(), dec.dynamics );
    cmp( "trainsets", module.scene.trainsets.size(), dec.trainsets );

    outcome.verify_ok = pass;
    outcome.message = report.str();
    outcome.verify_ms = ms_since( verify_begin );
    return outcome;
}

bool
verify_written_module(
    std::filesystem::path const &path,
    module_verify_spec const &spec,
    bool const is_root,
    std::size_t const pack_models,
    std::string *message_out ) {
    std::ifstream input { path, std::ios::binary };
    if( !input ) {
        if( message_out != nullptr ) {
            *message_out = "nie mozna odczytac " + path.generic_string();
        }
        return false;
    }

    input.seekg( 0, std::ios::end );
    const std::streamoff file_size { input.tellg() };
    input.seekg( 0, std::ios::beg );
    if( file_size <= 0 ) {
        if( message_out != nullptr ) {
            *message_out = "pusty plik " + path.generic_string();
        }
        return false;
    }

    std::vector<std::uint8_t> bytes( static_cast<std::size_t>( file_size ) );
    input.read(
        reinterpret_cast<char *>( bytes.data() ),
        static_cast<std::streamsize>( bytes.size() ) );

    decoded_counts const dec { decode_counts( bytes ) };
    if( !dec.ok ) {
        if( message_out != nullptr ) {
            *message_out = "decode blad " + path.generic_string();
        }
        return false;
    }

    std::size_t src_models { spec.models };
    if( is_root ) {
        src_models += pack_models;
    }

    std::ostringstream report;
    bool pass { dec.ok };
    auto cmp { [&]( char const *label, std::size_t const src, std::size_t const got ) {
        bool const ok { src == got };
        pass = pass && ok;
        report << "    " << ( ok ? "ok  " : "FAIL" ) << ' ' << label << " src=" << src
               << " eu7v2=" << got << '\n';
    } };

    cmp( "includes", spec.includes, dec.includes );
    cmp( "placements", spec.placements, dec.placements );
    cmp( "models", src_models, dec.instances );
    cmp( "shapes", spec.shapes, dec.shapes );
    cmp( "lines", spec.lines, dec.lines );
    cmp( "tracks", spec.tracks, dec.tracks );
    cmp( "traction", spec.traction, dec.traction );
    cmp( "power", spec.power, dec.power );
    cmp( "memcells", spec.memcells, dec.memcells );
    cmp( "launchers", spec.launchers, dec.launchers );
    cmp( "events", spec.events, dec.events );
    cmp( "sounds", spec.sounds, dec.sounds );
    cmp( "dynamics", spec.dynamics, dec.dynamics );
    cmp( "trainsets", spec.trainsets, dec.trainsets );

    if( message_out != nullptr ) {
        *message_out = report.str();
    }
    return pass;
}

} // namespace eu7v2
