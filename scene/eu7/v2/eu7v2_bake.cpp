/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "scene/eu7/v2/eu7v2_bake.h"

#include "scene/eu7/v2/eu7v2_format.h"
#include "scene/eu7/v2/eu7v2_scene.h"
#include "scene/eu7/v2/eu7v2_records.h"
#include "scene/eu7/eu7_types.h"

#include <string>
#include <unordered_map>

namespace eu7v2 {

namespace {

// Bakes the parser output into the dependency-free eu7v2 structs, owning the
// string table all chunks share.
class scene_baker {
  public:
    explicit scene_baker( scene::eu7::Eu7Scene const &scene ) : m_scene( scene ) {}
    scene_baker( scene::eu7::Eu7Scene const &scene, scene::eu7::Eu7Module const &module )
        : m_scene( scene ), m_module( &module ) {}

    std::vector<std::uint8_t> run() {
        convert_models();
        convert_terrain();
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
        if( m_module != nullptr ) {
            convert_includes();
            convert_meta();
        }
        return serialize();
    }

  private:
    [[nodiscard]] std::uint32_t str( std::string const &s ) { return m_strings.intern( s ); }

    [[nodiscard]] node_record node( scene::eu7::Eu7BasicNode const &n ) {
        node_record out;
        out.name = str( n.name );
        out.type = str( n.node_type );
        out.area_center = { n.area.center.x, n.area.center.y, n.area.center.z };
        out.area_radius = n.area.radius;
        out.range_sq_min = n.range_squared_min;
        out.range_sq_max = n.range_squared_max;
        out.visible = n.visible;
        return out;
    }

    // -- models: deduplicate identical definitions into prototypes ----------
    void convert_models() {
        for( auto const &model : m_scene.models ) {
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
            inst.cell_id = model.pack_cell_id;
            inst.texture_override = kNoString; // within a single bake the proto already carries the skin
            inst.has_node = true;
            inst.node = node( model.node );
            m_instances.push_back( inst );
        }
    }

    [[nodiscard]] std::uint32_t intern_prototype( scene::eu7::Eu7Model const &model ) {
        std::string key;
        key.reserve( model.model_file.size() + model.texture_file.size() + 32 );
        key.append( model.model_file );
        key.push_back( '\x1f' );
        key.append( model.texture_file );
        key.push_back( '\x1f' );
        key.push_back( model.transition ? '1' : '0' );
        key.push_back( model.is_terrain ? '1' : '0' );
        key.append( std::to_string( model.baked_range_min ) );
        key.push_back( '\x1f' );
        key.append( std::to_string( model.baked_range_max ) );
        key.push_back( '\x1f' );
        for( auto const s : model.light_states ) {
            key.append( std::to_string( s ) );
            key.push_back( ',' );
        }
        key.push_back( '\x1f' );
        for( auto const c : model.light_colors ) {
            key.append( std::to_string( c ) );
            key.push_back( ',' );
        }

        auto const it { m_prototype_lookup.find( key ) };
        if( it != m_prototype_lookup.end() ) {
            return it->second;
        }

        model_prototype proto;
        proto.model_file = str( model.model_file );
        proto.texture_file = model.texture_file.empty() ? kNoString : str( model.texture_file );
        proto.flags = 0;
        if( model.transition ) {
            proto.flags |= proto_flag::transition;
        }
        if( model.is_terrain ) {
            proto.flags |= proto_flag::is_terrain;
        }
        if( model.pack_flags & scene::eu7::kEu7PackFlagInstanceableHint ) {
            proto.flags |= proto_flag::instanceable;
        }
        proto.range_min = model.baked_range_min;
        proto.range_max = model.baked_range_max;
        proto.light_states = model.light_states;
        proto.light_colors = model.light_colors;

        auto const id { static_cast<std::uint32_t>( m_prototypes.size() ) };
        m_prototypes.push_back( std::move( proto ) );
        m_prototype_lookup.emplace( std::move( key ), id );
        return id;
    }

    // -- terrain: store origin in f64, vertices f32 relative to origin ------
    void convert_terrain() {
        for( auto const &shape : m_scene.terrain_shapes ) {
            terrain_mesh mesh;
            mesh.material = shape.material_path.empty() ? kNoString : str( shape.material_path );
            mesh.translucent = shape.translucent;
            mesh.ox = shape.origin.x;
            mesh.oy = shape.origin.y;
            mesh.oz = shape.origin.z;
            mesh.vertices.reserve( shape.vertices.size() );
            for( auto const &v : shape.vertices ) {
                mesh_vertex mv;
                mv.px = static_cast<float>( v.position.x - shape.origin.x );
                mv.py = static_cast<float>( v.position.y - shape.origin.y );
                mv.pz = static_cast<float>( v.position.z - shape.origin.z );
                mv.nx = v.normal.x;
                mv.ny = v.normal.y;
                mv.nz = v.normal.z;
                mv.u = static_cast<float>( v.u );
                mv.v = static_cast<float>( v.v );
                mesh.vertices.push_back( mv );
            }
            m_meshes.push_back( std::move( mesh ) );
        }
    }

    [[nodiscard]] lighting_block lighting( scene::eu7::Eu7LightingData const &l ) {
        lighting_block out;
        out.diffuse[ 0 ] = l.diffuse.x; out.diffuse[ 1 ] = l.diffuse.y;
        out.diffuse[ 2 ] = l.diffuse.z; out.diffuse[ 3 ] = l.diffuse.w;
        out.ambient[ 0 ] = l.ambient.x; out.ambient[ 1 ] = l.ambient.y;
        out.ambient[ 2 ] = l.ambient.z; out.ambient[ 3 ] = l.ambient.w;
        out.specular[ 0 ] = l.specular.x; out.specular[ 1 ] = l.specular.y;
        out.specular[ 2 ] = l.specular.z; out.specular[ 3 ] = l.specular.w;
        return out;
    }

    // -- non-terrain shapes: same encoding as terrain but lossless on node ---
    void convert_shapes() {
        for( auto const &shape : m_scene.shapes ) {
            shape_record r;
            r.node = node( shape.node );
            r.translucent = shape.translucent;
            r.material = shape.material_path.empty() ? kNoString : str( shape.material_path );
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
                mv.nx = v.normal.x;
                mv.ny = v.normal.y;
                mv.nz = v.normal.z;
                mv.u = static_cast<float>( v.u );
                mv.v = static_cast<float>( v.v );
                r.vertices.push_back( mv );
            }
            m_shapes.push_back( std::move( r ) );
        }
    }

    void convert_lines() {
        for( auto const &line : m_scene.lines ) {
            lines_record r;
            r.node = node( line.node );
            r.lighting = lighting( line.lighting );
            r.line_width = line.line_width;
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
        for( auto const &t : m_scene.tracks ) {
            track_record r;
            r.node = node( t.node );
            r.track_type = static_cast<std::uint8_t>( t.track_type );
            r.category = static_cast<std::uint8_t>( t.category );
            r.length = t.length;
            r.track_width = t.track_width;
            r.friction = t.friction;
            r.sound_distance = t.sound_distance;
            r.quality_flag = t.quality_flag;
            r.damage_flag = t.damage_flag;
            r.environment = static_cast<std::int8_t>( t.environment );
            if( t.visibility.has_value() ) {
                r.has_visibility = true;
                r.visibility.material1 = str( t.visibility->material1 );
                r.visibility.tex_length = t.visibility->tex_length;
                r.visibility.material2 = str( t.visibility->material2 );
                r.visibility.tex_height1 = t.visibility->tex_height1;
                r.visibility.tex_width = t.visibility->tex_width;
                r.visibility.tex_slope = t.visibility->tex_slope;
            }
            r.paths.reserve( t.paths.size() );
            for( auto const &p : t.paths ) {
                track_path tp;
                tp.p_start = { p.p_start.x, p.p_start.y, p.p_start.z };
                tp.roll_start = p.roll_start;
                tp.cp_out = { p.cp_out.x, p.cp_out.y, p.cp_out.z };
                tp.cp_in = { p.cp_in.x, p.cp_in.y, p.cp_in.z };
                tp.p_end = { p.p_end.x, p.p_end.y, p.p_end.z };
                tp.roll_end = p.roll_end;
                tp.radius = p.radius;
                r.paths.push_back( tp );
            }
            r.tail_keywords.reserve( t.tail_keywords.size() );
            for( auto const &kv : t.tail_keywords ) {
                r.tail_keywords.emplace_back( str( kv.first ), str( kv.second ) );
            }
            m_tracks.push_back( std::move( r ) );
        }
    }

    void convert_traction() {
        for( auto const &t : m_scene.traction ) {
            traction_record r;
            r.node = node( t.node );
            r.power_supply_name = str( t.power_supply_name );
            r.nominal_voltage = t.nominal_voltage;
            r.max_current = t.max_current;
            r.resistivity = t.resistivity_ohm_per_m;
            r.material = static_cast<std::uint8_t>( t.material );
            r.wire_thickness = t.wire_thickness;
            r.damage_flag = t.damage_flag;
            r.wire_p1 = { t.wire_p1.x, t.wire_p1.y, t.wire_p1.z };
            r.wire_p2 = { t.wire_p2.x, t.wire_p2.y, t.wire_p2.z };
            r.wire_p3 = { t.wire_p3.x, t.wire_p3.y, t.wire_p3.z };
            r.wire_p4 = { t.wire_p4.x, t.wire_p4.y, t.wire_p4.z };
            r.min_height = t.min_height;
            r.segment_length = t.segment_length;
            r.wire_count = t.wire_count;
            r.wire_offset = t.wire_offset;
            if( t.parallel_name.has_value() ) {
                r.has_parallel = true;
                r.parallel_name = str( *t.parallel_name );
            }
            m_traction.push_back( std::move( r ) );
        }
    }

    void convert_power_sources() {
        for( auto const &p : m_scene.power_sources ) {
            power_source_record r;
            r.node = node( p.node );
            r.position = { p.position.x, p.position.y, p.position.z };
            r.nominal_voltage = p.nominal_voltage;
            r.voltage_frequency = p.voltage_frequency;
            r.internal_resistance = p.internal_resistance;
            r.max_output_current = p.max_output_current;
            r.fast_fuse_timeout = p.fast_fuse_timeout;
            r.fast_fuse_repetition = p.fast_fuse_repetition;
            r.slow_fuse_timeout = p.slow_fuse_timeout;
            r.modifier = static_cast<std::uint8_t>( p.modifier );
            m_power.push_back( std::move( r ) );
        }
    }

    void convert_memcells() {
        for( auto const &m : m_scene.memcells ) {
            memcell_record r;
            r.node = node( m.node );
            r.text = str( m.text );
            r.value1 = m.value1;
            r.value2 = m.value2;
            if( m.track_name.has_value() ) {
                r.has_track = true;
                r.track_name = str( *m.track_name );
            }
            m_memcells.push_back( std::move( r ) );
        }
    }

    void convert_launchers() {
        for( auto const &l : m_scene.event_launchers ) {
            launcher_record r;
            r.node = node( l.node );
            r.location = { l.location.x, l.location.y, l.location.z };
            r.radius_squared = l.radius_squared;
            r.activation_key = l.activation_key;
            r.delta_time = l.delta_time;
            r.event1_name = str( l.event1_name );
            r.event2_name = str( l.event2_name );
            if( l.condition.has_value() ) {
                r.has_condition = true;
                r.condition.memcell_name = str( l.condition->memcell_name );
                r.condition.compare_text = str( l.condition->compare_text );
                r.condition.compare_value1 = l.condition->compare_value1;
                r.condition.compare_value2 = l.condition->compare_value2;
                r.condition.check_mask = l.condition->check_mask;
            }
            r.train_triggered = l.train_triggered;
            r.launch_hour = l.launch_hour;
            r.launch_minute = l.launch_minute;
            m_launchers.push_back( std::move( r ) );
        }
    }

    void convert_events() {
        for( auto const &e : m_scene.events ) {
            event_record r;
            r.name = str( e.name );
            r.type = static_cast<std::uint8_t>( e.type );
            r.delay = e.delay;
            r.delay_random = e.delay_random;
            r.delay_departure = e.delay_departure;
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
        for( auto const &s : m_scene.sounds ) {
            sound_record r;
            r.node = node( s.node );
            r.location = { s.location.x, s.location.y, s.location.z };
            r.wav_file = str( s.wav_file );
            m_sounds.push_back( std::move( r ) );
        }
    }

    void convert_dynamics() {
        for( auto const &d : m_scene.dynamics ) {
            dynamic_record r;
            r.node = node( d.node );
            r.data_folder = str( d.data_folder );
            r.skin_file = str( d.skin_file );
            r.mmd_file = str( d.mmd_file );
            r.track_name = str( d.track_name );
            r.offset = d.offset;
            r.driver_type = str( d.driver_type );
            r.coupling = d.coupling;
            r.coupling_raw = str( d.coupling_raw );
            r.coupling_params = str( d.coupling_params );
            r.velocity = d.velocity;
            r.load_count = d.load_count;
            r.load_type = str( d.load_type );
            if( d.destination.has_value() ) {
                r.has_destination = true;
                r.destination = str( *d.destination );
            }
            if( d.trainset_index.has_value() ) {
                r.has_trainset = true;
                r.trainset_index = static_cast<std::uint32_t>( *d.trainset_index );
            }
            m_dynamics.push_back( std::move( r ) );
        }
    }

    void convert_trainsets() {
        for( auto const &t : m_scene.trainsets ) {
            trainset_record r;
            r.name = str( t.name );
            r.track = str( t.track );
            r.offset = t.offset;
            r.velocity = t.velocity;
            r.assignment.reserve( t.assignment.size() );
            for( auto const &kv : t.assignment ) {
                r.assignment.emplace_back( str( kv.first ), str( kv.second ) );
            }
            r.vehicle_indices.reserve( t.vehicle_indices.size() );
            for( auto const idx : t.vehicle_indices ) {
                r.vehicle_indices.push_back( static_cast<std::uint32_t>( idx ) );
            }
            r.couplings.reserve( t.couplings.size() );
            for( auto const c : t.couplings ) {
                r.couplings.push_back( c );
            }
            r.driver_index =
                t.driver_index == static_cast<std::size_t>( -1 )
                    ? 0xffffffffu
                    : static_cast<std::uint32_t>( t.driver_index );
            m_trainsets.push_back( std::move( r ) );
        }
    }

    void convert_includes() {
        for( auto const &inc : m_module->includes ) {
            include_record r;
            r.source_line = inc.source_line;
            r.source_path = inc.source_path.empty() ? kNoString : str( inc.source_path );
            r.binary_path = inc.binary_path.empty() ? kNoString : str( inc.binary_path );
            r.parameters.reserve( inc.parameters.size() );
            for( auto const &p : inc.parameters ) {
                r.parameters.push_back( str( p ) );
            }
            r.site_transform = transform( inc.site_transform );
            m_includes.push_back( std::move( r ) );
        }
    }

    [[nodiscard]] transform_record transform( scene::eu7::Eu7TransformContext const &t ) {
        transform_record out;
        out.origin_stack.reserve( t.origin_stack.size() );
        for( auto const &v : t.origin_stack ) {
            out.origin_stack.push_back( { v.x, v.y, v.z } );
        }
        out.scale_stack.reserve( t.scale_stack.size() );
        for( auto const &v : t.scale_stack ) {
            out.scale_stack.push_back( { v.x, v.y, v.z } );
        }
        out.rotation = { t.rotation.x, t.rotation.y, t.rotation.z };
        out.group_depth = static_cast<std::uint32_t>( t.group_depth );
        return out;
    }

    void convert_meta() {
        m_meta.first_init_count = m_scene.first_init_count;
        m_meta.has_terrain_chunk = m_module->has_terrain_chunk;
        m_meta.has_pack_chunk = m_module->has_pack_chunk;
        m_meta.placement_origin_x = m_module->include_placement.origin_x_param;
        m_meta.placement_origin_y = m_module->include_placement.origin_y_param;
        m_meta.placement_origin_z = m_module->include_placement.origin_z_param;
        m_meta.placement_rotation_y = m_module->include_placement.rotation_y_param;
        m_has_meta = true;
    }

    [[nodiscard]] std::vector<std::uint8_t> serialize() const {
        container_writer writer( file_kind::sim );

        byte_writer strs;
        m_strings.serialize( strs );
        writer.add_chunk( chunk::strs, strs );

        // META must be emitted before the chunks it describes are consumed;
        // a single pass over chunks resolves it (loader reads it whenever seen).
        if( m_has_meta ) {
            byte_writer meta;
            write_meta( meta, m_meta );
            writer.add_chunk( chunk::meta, meta );
        }

        auto emit { [&]( std::uint32_t id, auto const &write_fn, auto const &items ) {
            if( items.empty() ) {
                return;
            }
            byte_writer payload;
            write_fn( payload, items );
            writer.add_chunk( id, payload );
        } };

        emit( chunk::incl, &write_includes, m_includes );
        emit( chunk::prot, &write_prototypes, m_prototypes );
        emit( chunk::inst, &write_instances, m_instances );
        emit( chunk::mesh, &write_terrain_meshes, m_meshes );
        emit( chunk::shpe, &write_shapes, m_shapes );
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

    scene::eu7::Eu7Scene const &m_scene;
    scene::eu7::Eu7Module const *m_module { nullptr };
    string_table m_strings;

    std::vector<model_prototype> m_prototypes;
    std::unordered_map<std::string, std::uint32_t> m_prototype_lookup;
    std::vector<model_instance> m_instances;
    std::vector<terrain_mesh> m_meshes;
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
    bool m_has_meta { false };
};

} // namespace

std::vector<std::uint8_t>
bake_scene( scene::eu7::Eu7Scene const &scene ) {
    scene_baker baker( scene );
    return baker.run();
}

std::vector<std::uint8_t>
bake_module( scene::eu7::Eu7Module const &module ) {
    scene_baker baker( module.scene, module );
    return baker.run();
}

} // namespace eu7v2
