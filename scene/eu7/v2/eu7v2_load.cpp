/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "scene/eu7/v2/eu7v2_load.h"

#include "scene/eu7/v2/eu7v2_format.h"
#include "scene/eu7/v2/eu7v2_scene.h"
#include "scene/eu7/v2/eu7v2_records.h"
#include "scene/eu7/eu7_types.h"

#include <string>

namespace eu7v2 {

namespace {

// Reverse of scene_baker: turns decoded eu7v2 structs back into Eu7Scene.
class scene_loader {
  public:
    scene_loader( string_pool const &pool, scene::eu7::Eu7Scene &out )
        : m_pool( pool ), m_out( out ) {}

    [[nodiscard]] std::string str( std::uint32_t const id ) const {
        return id == kNoString ? std::string() : m_pool.get( id );
    }

    void apply_node( scene::eu7::Eu7BasicNode &n, node_record const &r ) const {
        n.name = str( r.name );
        n.node_type = str( r.type );
        n.area.center = { r.area_center.x, r.area_center.y, r.area_center.z };
        n.area.radius = r.area_radius;
        n.range_squared_min = r.range_sq_min;
        n.range_squared_max = r.range_sq_max;
        n.visible = r.visible;
    }

    void build_models(
        std::vector<model_prototype> const &protos,
        std::vector<model_instance> const &instances ) const {
        m_out.models.reserve( instances.size() );
        for( auto const &inst : instances ) {
            if( inst.proto >= protos.size() ) {
                continue;
            }
            auto const &proto { protos[ inst.proto ] };
            scene::eu7::Eu7Model m;
            m.location = { inst.x, inst.y, inst.z };
            m.angles = { inst.ax, inst.ay, inst.az };
            m.scale = { inst.sx, inst.sy, inst.sz };
            m.model_file = str( proto.model_file );
            m.texture_file =
                inst.texture_override != kNoString ? str( inst.texture_override )
                                                   : str( proto.texture_file );
            m.light_states = proto.light_states;
            m.light_colors = proto.light_colors;
            m.transition = ( proto.flags & proto_flag::transition ) != 0;
            m.is_terrain = ( proto.flags & proto_flag::is_terrain ) != 0;
            m.pack_flags =
                ( proto.flags & proto_flag::instanceable ) ? scene::eu7::kEu7PackFlagInstanceableHint : 0u;
            m.baked_range_min = proto.range_min;
            m.baked_range_max = proto.range_max;
            m.pack_cell_id = inst.cell_id;
            if( inst.has_node ) {
                apply_node( m.node, inst.node );
            }
            else {
                m.node.name = m.model_file;
                m.node.node_type = "model";
                m.node.area.center = m.location;
            }
            m_out.models.push_back( std::move( m ) );
        }
    }

    static void apply_lighting(
        scene::eu7::Eu7LightingData &dst, lighting_block const &src ) {
        dst.diffuse = { src.diffuse[ 0 ], src.diffuse[ 1 ], src.diffuse[ 2 ], src.diffuse[ 3 ] };
        dst.ambient = { src.ambient[ 0 ], src.ambient[ 1 ], src.ambient[ 2 ], src.ambient[ 3 ] };
        dst.specular = { src.specular[ 0 ], src.specular[ 1 ], src.specular[ 2 ], src.specular[ 3 ] };
    }

    void build_shapes( std::vector<shape_record> const &shapes ) const {
        m_out.shapes.reserve( shapes.size() );
        for( auto const &r : shapes ) {
            scene::eu7::Eu7Shape s;
            apply_node( s.node, r.node );
            s.translucent = r.translucent;
            s.material_path = str( r.material );
            apply_lighting( s.lighting, r.lighting );
            s.origin = { r.ox, r.oy, r.oz };
            s.vertices.reserve( r.vertices.size() );
            for( auto const &v : r.vertices ) {
                scene::eu7::Eu7WorldVertex wv;
                wv.position = {
                    r.ox + static_cast<double>( v.px ),
                    r.oy + static_cast<double>( v.py ),
                    r.oz + static_cast<double>( v.pz ) };
                wv.normal = { v.nx, v.ny, v.nz };
                wv.u = v.u;
                wv.v = v.v;
                s.vertices.push_back( wv );
            }
            m_out.shapes.push_back( std::move( s ) );
        }
    }

    void build_lines( std::vector<lines_record> const &items ) const {
        m_out.lines.reserve( items.size() );
        for( auto const &r : items ) {
            scene::eu7::Eu7Lines l;
            apply_node( l.node, r.node );
            apply_lighting( l.lighting, r.lighting );
            l.line_width = r.line_width;
            l.origin = { r.ox, r.oy, r.oz };
            l.vertices.reserve( r.vertices.size() );
            for( auto const &v : r.vertices ) {
                scene::eu7::Eu7WorldVertex wv;
                wv.position = { v.x, v.y, v.z };
                l.vertices.push_back( wv );
            }
            m_out.lines.push_back( std::move( l ) );
        }
    }

    void build_trainsets( std::vector<trainset_record> const &items ) const {
        m_out.trainsets.reserve( items.size() );
        for( auto const &r : items ) {
            scene::eu7::Eu7Trainset t;
            t.name = str( r.name );
            t.track = str( r.track );
            t.offset = r.offset;
            t.velocity = r.velocity;
            for( auto const &kv : r.assignment ) {
                t.assignment.emplace( str( kv.first ), str( kv.second ) );
            }
            t.vehicle_indices.reserve( r.vehicle_indices.size() );
            for( auto const idx : r.vehicle_indices ) {
                t.vehicle_indices.push_back( static_cast<std::size_t>( idx ) );
            }
            t.couplings.reserve( r.couplings.size() );
            for( auto const c : r.couplings ) {
                t.couplings.push_back( c );
            }
            t.driver_index =
                r.driver_index == 0xffffffffu
                    ? static_cast<std::size_t>( -1 )
                    : static_cast<std::size_t>( r.driver_index );
            m_out.trainsets.push_back( std::move( t ) );
        }
    }

    void build_terrain( std::vector<terrain_mesh> const &meshes ) const {
        m_out.terrain_shapes.reserve( meshes.size() );
        for( auto const &mesh : meshes ) {
            scene::eu7::Eu7Shape s;
            s.material_path = str( mesh.material );
            s.translucent = mesh.translucent;
            s.origin = { mesh.ox, mesh.oy, mesh.oz };
            s.vertices.reserve( mesh.vertices.size() );
            for( auto const &v : mesh.vertices ) {
                scene::eu7::Eu7WorldVertex wv;
                wv.position = {
                    mesh.ox + static_cast<double>( v.px ),
                    mesh.oy + static_cast<double>( v.py ),
                    mesh.oz + static_cast<double>( v.pz ) };
                wv.normal = { v.nx, v.ny, v.nz };
                wv.u = v.u;
                wv.v = v.v;
                s.vertices.push_back( wv );
            }
            m_out.terrain_shapes.push_back( std::move( s ) );
        }
    }

    void build_tracks( std::vector<track_record> const &tracks ) const {
        m_out.tracks.reserve( tracks.size() );
        for( auto const &r : tracks ) {
            scene::eu7::Eu7Track t;
            apply_node( t.node, r.node );
            t.track_type = static_cast<scene::eu7::Eu7TrackType>( r.track_type );
            t.category = static_cast<scene::eu7::Eu7TrackCategory>( r.category );
            t.length = r.length;
            t.track_width = r.track_width;
            t.friction = r.friction;
            t.sound_distance = r.sound_distance;
            t.quality_flag = r.quality_flag;
            t.damage_flag = r.damage_flag;
            t.environment = static_cast<scene::eu7::Eu7TrackEnvironment>( r.environment );
            if( r.has_visibility ) {
                scene::eu7::Eu7TrackVisibility vis;
                vis.material1 = str( r.visibility.material1 );
                vis.tex_length = r.visibility.tex_length;
                vis.material2 = str( r.visibility.material2 );
                vis.tex_height1 = r.visibility.tex_height1;
                vis.tex_width = r.visibility.tex_width;
                vis.tex_slope = r.visibility.tex_slope;
                t.visibility = vis;
            }
            t.paths.reserve( r.paths.size() );
            for( auto const &p : r.paths ) {
                scene::eu7::Eu7SegmentPath sp;
                sp.p_start = { p.p_start.x, p.p_start.y, p.p_start.z };
                sp.roll_start = p.roll_start;
                sp.cp_out = { p.cp_out.x, p.cp_out.y, p.cp_out.z };
                sp.cp_in = { p.cp_in.x, p.cp_in.y, p.cp_in.z };
                sp.p_end = { p.p_end.x, p.p_end.y, p.p_end.z };
                sp.roll_end = p.roll_end;
                sp.radius = p.radius;
                t.paths.push_back( sp );
            }
            t.tail_keywords.reserve( r.tail_keywords.size() );
            for( auto const &kv : r.tail_keywords ) {
                t.tail_keywords.emplace_back( str( kv.first ), str( kv.second ) );
            }
            m_out.tracks.push_back( std::move( t ) );
        }
    }

    void build_traction( std::vector<traction_record> const &items ) const {
        m_out.traction.reserve( items.size() );
        for( auto const &r : items ) {
            scene::eu7::Eu7Traction t;
            apply_node( t.node, r.node );
            t.power_supply_name = str( r.power_supply_name );
            t.nominal_voltage = r.nominal_voltage;
            t.max_current = r.max_current;
            t.resistivity_ohm_per_m = r.resistivity;
            t.material = static_cast<scene::eu7::Eu7TractionWireMaterial>( r.material );
            t.wire_thickness = r.wire_thickness;
            t.damage_flag = r.damage_flag;
            t.wire_p1 = { r.wire_p1.x, r.wire_p1.y, r.wire_p1.z };
            t.wire_p2 = { r.wire_p2.x, r.wire_p2.y, r.wire_p2.z };
            t.wire_p3 = { r.wire_p3.x, r.wire_p3.y, r.wire_p3.z };
            t.wire_p4 = { r.wire_p4.x, r.wire_p4.y, r.wire_p4.z };
            t.min_height = r.min_height;
            t.segment_length = r.segment_length;
            t.wire_count = r.wire_count;
            t.wire_offset = r.wire_offset;
            if( r.has_parallel ) {
                t.parallel_name = str( r.parallel_name );
            }
            m_out.traction.push_back( std::move( t ) );
        }
    }

    void build_power_sources( std::vector<power_source_record> const &items ) const {
        m_out.power_sources.reserve( items.size() );
        for( auto const &r : items ) {
            scene::eu7::Eu7TractionPowerSource p;
            apply_node( p.node, r.node );
            p.position = { r.position.x, r.position.y, r.position.z };
            p.nominal_voltage = r.nominal_voltage;
            p.voltage_frequency = r.voltage_frequency;
            p.internal_resistance = r.internal_resistance;
            p.max_output_current = r.max_output_current;
            p.fast_fuse_timeout = r.fast_fuse_timeout;
            p.fast_fuse_repetition = r.fast_fuse_repetition;
            p.slow_fuse_timeout = r.slow_fuse_timeout;
            p.modifier = static_cast<scene::eu7::Eu7PowerSourceModifier>( r.modifier );
            m_out.power_sources.push_back( std::move( p ) );
        }
    }

    void build_memcells( std::vector<memcell_record> const &items ) const {
        m_out.memcells.reserve( items.size() );
        for( auto const &r : items ) {
            scene::eu7::Eu7MemCell m;
            apply_node( m.node, r.node );
            m.text = str( r.text );
            m.value1 = r.value1;
            m.value2 = r.value2;
            if( r.has_track ) {
                m.track_name = str( r.track_name );
            }
            m_out.memcells.push_back( std::move( m ) );
        }
    }

    void build_launchers( std::vector<launcher_record> const &items ) const {
        m_out.event_launchers.reserve( items.size() );
        for( auto const &r : items ) {
            scene::eu7::Eu7EventLauncher l;
            apply_node( l.node, r.node );
            l.location = { r.location.x, r.location.y, r.location.z };
            l.radius_squared = r.radius_squared;
            l.activation_key = r.activation_key;
            l.delta_time = r.delta_time;
            l.event1_name = str( r.event1_name );
            l.event2_name = str( r.event2_name );
            if( r.has_condition ) {
                scene::eu7::Eu7EventLauncherCondition c;
                c.memcell_name = str( r.condition.memcell_name );
                c.compare_text = str( r.condition.compare_text );
                c.compare_value1 = r.condition.compare_value1;
                c.compare_value2 = r.condition.compare_value2;
                c.check_mask = r.condition.check_mask;
                l.condition = c;
            }
            l.train_triggered = r.train_triggered;
            l.launch_hour = r.launch_hour;
            l.launch_minute = r.launch_minute;
            m_out.event_launchers.push_back( std::move( l ) );
        }
    }

    void build_events( std::vector<event_record> const &items ) const {
        m_out.events.reserve( items.size() );
        for( auto const &r : items ) {
            scene::eu7::Eu7Event e;
            e.name = str( r.name );
            e.type = static_cast<scene::eu7::Eu7EventType>( r.type );
            e.delay = r.delay;
            e.delay_random = r.delay_random;
            e.delay_departure = r.delay_departure;
            e.ignored = r.ignored;
            e.passive = r.passive;
            e.targets.reserve( r.targets.size() );
            for( auto const t : r.targets ) {
                e.targets.push_back( str( t ) );
            }
            e.payload.reserve( r.payload.size() );
            for( auto const &kv : r.payload ) {
                e.payload.emplace_back( str( kv.first ), str( kv.second ) );
            }
            m_out.events.push_back( std::move( e ) );
        }
    }

    void build_sounds( std::vector<sound_record> const &items ) const {
        m_out.sounds.reserve( items.size() );
        for( auto const &r : items ) {
            scene::eu7::Eu7Sound s;
            apply_node( s.node, r.node );
            s.location = { r.location.x, r.location.y, r.location.z };
            s.wav_file = str( r.wav_file );
            m_out.sounds.push_back( std::move( s ) );
        }
    }

    void build_dynamics( std::vector<dynamic_record> const &items ) const {
        m_out.dynamics.reserve( items.size() );
        for( auto const &r : items ) {
            scene::eu7::Eu7Dynamic d;
            apply_node( d.node, r.node );
            d.data_folder = str( r.data_folder );
            d.skin_file = str( r.skin_file );
            d.mmd_file = str( r.mmd_file );
            d.track_name = str( r.track_name );
            d.offset = r.offset;
            d.driver_type = str( r.driver_type );
            d.coupling = r.coupling;
            d.coupling_raw = str( r.coupling_raw );
            d.coupling_params = str( r.coupling_params );
            d.velocity = r.velocity;
            d.load_count = r.load_count;
            d.load_type = str( r.load_type );
            if( r.has_destination ) {
                d.destination = str( r.destination );
            }
            if( r.has_trainset ) {
                d.trainset_index = static_cast<std::size_t>( r.trainset_index );
            }
            m_out.dynamics.push_back( std::move( d ) );
        }
    }

  private:
    string_pool const &m_pool;
    scene::eu7::Eu7Scene &m_out;
};

} // namespace

namespace {

// Shared scan: reconstructs the Eu7Scene; when module != nullptr it also
// reconstructs includes / placement / flags / first_init_count.
[[nodiscard]] bool
load_into(
    std::uint8_t const *data,
    std::size_t const size,
    scene::eu7::Eu7Scene &out,
    scene::eu7::Eu7Module *module ) {
    try {
        container_reader reader( data, size );
        if( reader.kind() != file_kind::sim && reader.kind() != file_kind::module ) {
            return false;
        }

        string_pool pool;
        std::vector<model_prototype> protos;
        std::vector<model_instance> instances;
        std::vector<include_record> includes;
        std::vector<module_placement_record> placements;
        bool saw_meta { false };
        module_meta meta;

        // STRS must be resolved before the records that reference it; the baker
        // always writes it first, so a single pass over chunks is enough.
        chunk_view chunk;
        scene_loader loader( pool, out );
        while( reader.next( chunk ) ) {
            auto r { chunk.reader() };
            switch( chunk.id ) {
                case chunk::strs: pool.deserialize( r ); break;
                case chunk::meta: meta = read_meta( r ); saw_meta = true; break;
                case chunk::incl: includes = read_includes( r ); break;
                case chunk::plce: placements = read_module_placements( r ); break;
                case chunk::prot: protos = read_prototypes( r ); break;
                case chunk::inst: instances = read_instances( r ); break;
                case chunk::mesh: loader.build_terrain( read_terrain_meshes( r ) ); break;
                case chunk::shpe: loader.build_shapes( read_shapes( r ) ); break;
                case chunk::line: loader.build_lines( read_lines( r ) ); break;
                case chunk::trak: loader.build_tracks( read_tracks( r ) ); break;
                case chunk::trac: loader.build_traction( read_traction( r ) ); break;
                case chunk::pwrs: loader.build_power_sources( read_power_sources( r ) ); break;
                case chunk::memc: loader.build_memcells( read_memcells( r ) ); break;
                case chunk::laun: loader.build_launchers( read_launchers( r ) ); break;
                case chunk::evnt: loader.build_events( read_events( r ) ); break;
                case chunk::sond: loader.build_sounds( read_sounds( r ) ); break;
                case chunk::dynm: loader.build_dynamics( read_dynamics( r ) ); break;
                case chunk::trst: loader.build_trainsets( read_trainsets( r ) ); break;
                default: break; // unknown chunk: skip
            }
        }

        // models depend on both PROT and INST, build them after the scan
        loader.build_models( protos, instances );

        if( saw_meta ) {
            out.first_init_count = meta.first_init_count;
        }

        if( module != nullptr ) {
            auto resolve { [&]( std::uint32_t const id ) {
                return id == kNoString ? std::string() : pool.get( id );
            } };
            module->includes.reserve( includes.size() + placements.size() );
            for( auto const &inc : includes ) {
                scene::eu7::Eu7Include e;
                e.source_line = inc.source_line;
                e.source_path = resolve( inc.source_path );
                e.binary_path = resolve( inc.binary_path );
                e.parameters.reserve( inc.parameters.size() );
                for( auto const p : inc.parameters ) {
                    e.parameters.push_back( resolve( p ) );
                }
                for( auto const &v : inc.site_transform.origin_stack ) {
                    e.site_transform.origin_stack.push_back( { v.x, v.y, v.z } );
                }
                for( auto const &v : inc.site_transform.scale_stack ) {
                    e.site_transform.scale_stack.push_back( { v.x, v.y, v.z } );
                }
                e.site_transform.rotation = {
                    inc.site_transform.rotation.x,
                    inc.site_transform.rotation.y,
                    inc.site_transform.rotation.z };
                e.site_transform.group_depth =
                    static_cast<std::size_t>( inc.site_transform.group_depth );
                module->includes.push_back( std::move( e ) );
            }
            for( auto const &p : placements ) {
                scene::eu7::Eu7Include e;
                e.binary_path = resolve( p.module_path );
                if( !e.binary_path.empty() ) {
                    std::filesystem::path source { e.binary_path };
                    source.replace_extension( ".inc" );
                    e.source_path = source.generic_string();
                }
                if( p.texture_override != kNoString ) {
                    e.parameters.push_back( resolve( p.texture_override ) );
                } else {
                    e.parameters.push_back( "none" );
                }
                e.parameters.push_back( std::to_string( p.x ) );
                e.parameters.push_back( std::to_string( p.y ) );
                e.parameters.push_back( std::to_string( p.z ) );
                e.parameters.push_back( std::to_string( p.rotation_y ) );
                module->includes.push_back( std::move( e ) );
            }
            if( saw_meta ) {
                module->has_terrain_chunk = meta.has_terrain_chunk;
                module->has_pack_chunk = meta.has_pack_chunk;
                module->include_placement.origin_x_param = meta.placement_origin_x;
                module->include_placement.origin_y_param = meta.placement_origin_y;
                module->include_placement.origin_z_param = meta.placement_origin_z;
                module->include_placement.rotation_y_param = meta.placement_rotation_y;
            }
        }
        return true;
    }
    catch( parse_error const & ) {
        return false;
    }
}

} // namespace

bool
load_scene( std::uint8_t const *data, std::size_t const size, scene::eu7::Eu7Scene &out ) {
    return load_into( data, size, out, nullptr );
}

bool
load_module( std::uint8_t const *data, std::size_t const size, scene::eu7::Eu7Module &out ) {
    return load_into( data, size, out.scene, &out );
}

} // namespace eu7v2
