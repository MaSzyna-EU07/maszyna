/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Standalone round-trip test for the eu7v2 format core. Builds in isolation
// (no engine dependencies) so the foundation can be validated without running
// the simulator:
//     g++ -std=c++20 scene/eu7/v2/eu7v2_test.cpp -o eu7v2_test && ./eu7v2_test

#include "eu7v2_format.h"
#include "eu7v2_scene.h"
#include "eu7v2_records.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace {

using namespace eu7v2;

int g_checks { 0 };
int g_failures { 0 };

void
check( bool const ok, char const *what ) {
    ++g_checks;
    if( !ok ) {
        ++g_failures;
        std::printf( "  FAIL: %s\n", what );
    }
}

// A lean instance record: prototype id + world transform + optional texture override.
struct sample_instance {
    std::uint32_t proto_id;
    double x, y, z;
    float yaw;
    std::uint32_t texture_override; // string id, 0xffffffff = none
};

// A precomputed track-graph edge: track index -> neighbour at an endpoint.
struct sample_edge {
    std::uint32_t track;
    std::uint32_t neighbour;
    std::uint8_t end;        // 0 = prev, 1 = next
    std::uint8_t neighbour_end;
};

std::vector<std::uint8_t>
build_tile() {
    string_table strings;
    auto const tex_a { strings.intern( "textures/rail.dds" ) };
    auto const tex_b { strings.intern( "textures/grass.dds" ) };
    auto const proto_a { strings.intern( "models/sem.e3d" ) };
    (void)tex_b;
    (void)proto_a;

    std::vector<sample_instance> const instances {
        { 0, 10.0, 0.0, -5.0, 1.5707963f, tex_a },
        { 1, 1234.5, 12.25, -6789.0, 0.0f, 0xffffffffu },
        { 2, -42.0, 3.0, 7.0, 3.14159f, tex_b },
    };

    byte_writer strs_payload;
    strings.serialize( strs_payload );

    byte_writer inst_payload;
    inst_payload.put_u32( static_cast<std::uint32_t>( instances.size() ) );
    for( auto const &i : instances ) {
        inst_payload.put_u32( i.proto_id );
        inst_payload.put_f64( i.x );
        inst_payload.put_f64( i.y );
        inst_payload.put_f64( i.z );
        inst_payload.put_f32( i.yaw );
        inst_payload.put_u32( i.texture_override );
    }

    container_writer writer( file_kind::tile );
    writer.add_chunk( chunk::strs, strs_payload );
    writer.add_chunk( chunk::inst, inst_payload );
    return writer.data();
}

void
verify_tile( std::vector<std::uint8_t> const &bytes ) {
    container_reader reader( bytes.data(), bytes.size() );
    check( reader.kind() == file_kind::tile, "tile kind" );
    check( reader.version() == kVersion, "tile version" );

    string_pool pool;
    bool saw_strs { false };
    bool saw_inst { false };

    chunk_view chunk;
    while( reader.next( chunk ) ) {
        if( chunk.id == chunk::strs ) {
            saw_strs = true;
            auto r { chunk.reader() };
            pool.deserialize( r );
            check( pool.size() == 3, "string count" );
            check( pool.get( 0 ) == "textures/rail.dds", "string 0" );
            check( pool.get( 2 ) == "models/sem.e3d", "string 2" );
        }
        else if( chunk.id == chunk::inst ) {
            saw_inst = true;
            auto r { chunk.reader() };
            auto const count { r.get_u32() };
            check( count == 3, "instance count" );

            auto const p0 { r.get_u32() };
            auto const x0 { r.get_f64() };
            auto const y0 { r.get_f64() };
            auto const z0 { r.get_f64() };
            auto const yaw0 { r.get_f32() };
            auto const tex0 { r.get_u32() };
            check( p0 == 0, "inst0 proto" );
            check( x0 == 10.0 && y0 == 0.0 && z0 == -5.0, "inst0 pos (f64 exact)" );
            check( std::fabs( yaw0 - 1.5707963f ) < 1e-6f, "inst0 yaw" );
            check( tex0 == 0, "inst0 tex override id" );

            // second record: verify large f64 coordinate keeps full precision
            auto const p1 { r.get_u32() };
            auto const x1 { r.get_f64() };
            (void)r.get_f64();
            auto const z1 { r.get_f64() };
            (void)r.get_f32();
            auto const tex1 { r.get_u32() };
            check( p1 == 1, "inst1 proto" );
            check( x1 == 1234.5 && z1 == -6789.0, "inst1 large coord precision" );
            check( tex1 == 0xffffffffu, "inst1 no tex override" );
        }
    }

    check( saw_strs, "tile has STRS" );
    check( saw_inst, "tile has INST" );
}

void
verify_trgr_roundtrip() {
    std::vector<sample_edge> const edges {
        { 0, 1, 1, 0 },
        { 1, 2, 1, 0 },
        { 2, 0, 1, 0 },
    };

    byte_writer payload;
    payload.put_u32( static_cast<std::uint32_t>( edges.size() ) );
    for( auto const &e : edges ) {
        payload.put_u32( e.track );
        payload.put_u32( e.neighbour );
        payload.put_u8( e.end );
        payload.put_u8( e.neighbour_end );
    }

    container_writer writer( file_kind::sim );
    writer.add_chunk( chunk::trgr, payload );
    auto const bytes { writer.data() };

    container_reader reader( bytes.data(), bytes.size() );
    check( reader.kind() == file_kind::sim, "sim kind" );

    chunk_view chunk;
    bool ok { false };
    while( reader.next( chunk ) ) {
        if( chunk.id != chunk::trgr ) {
            continue;
        }
        auto r { chunk.reader() };
        auto const count { r.get_u32() };
        check( count == edges.size(), "edge count" );
        for( auto const &expected : edges ) {
            auto const track { r.get_u32() };
            auto const neighbour { r.get_u32() };
            auto const end { r.get_u8() };
            auto const nend { r.get_u8() };
            check(
                track == expected.track && neighbour == expected.neighbour &&
                    end == expected.end && nend == expected.neighbour_end,
                "edge values" );
        }
        ok = true;
    }
    check( ok, "sim has TRGR" );
}

void
verify_truncation_is_rejected() {
    auto bytes { build_tile() };
    bytes.resize( bytes.size() - 4 ); // chop the tail of the last chunk
    bool threw { false };
    try {
        container_reader reader( bytes.data(), bytes.size() );
        chunk_view chunk;
        while( reader.next( chunk ) ) {
            auto r { chunk.reader() };
            // force payload reads
            while( r.remaining() > 0 ) {
                (void)r.get_u8();
            }
        }
    }
    catch( parse_error const & ) {
        threw = true;
    }
    check( threw, "truncated file rejected with parse_error" );
}

void
verify_scene_roundtrip() {
    string_table strings;
    auto const sem_model { strings.intern( "models/sem.e3d" ) };
    auto const sem_tex { strings.intern( "textures/sem.dds" ) };
    auto const grass { strings.intern( "textures/grass.dds" ) };
    auto const tex_override { strings.intern( "textures/sem_winter.dds" ) };

    std::vector<model_prototype> protos( 2 );
    protos[ 0 ].model_file = sem_model;
    protos[ 0 ].texture_file = sem_tex;
    protos[ 0 ].flags = proto_flag::instanceable | proto_flag::transition;
    protos[ 0 ].range_min = 0.f;
    protos[ 0 ].range_max = 4000.f;
    protos[ 0 ].light_states = { 1.f, 0.5f, 0.f };
    protos[ 0 ].light_colors = { 0xff0000ffu, 0x00ff00ffu };
    protos[ 1 ].model_file = strings.intern( "models/tree.e3d" );
    protos[ 1 ].flags = proto_flag::instanceable;

    std::vector<model_instance> instances( 3 );
    instances[ 0 ] = { 0, 100.0, 0.0, -200.0, 0.f, 90.f, 0.f, 1.f, 1.f, 1.f, kNoString, 7 };
    instances[ 1 ] = { 0, 999999.5, 1.25, -888888.0, 0.f, 0.f, 0.f, 2.f, 2.f, 2.f, tex_override, 3 };
    instances[ 2 ] = { 1, -5.0, 0.0, 5.0, 0.f, 45.f, 0.f, 1.f, 1.f, 1.f, kNoString, 0xffu };

    std::vector<terrain_mesh> meshes( 1 );
    meshes[ 0 ].material = grass;
    meshes[ 0 ].translucent = false;
    meshes[ 0 ].ox = 500000.0;
    meshes[ 0 ].oy = 0.0;
    meshes[ 0 ].oz = -500000.0;
    meshes[ 0 ].vertices = {
        { 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f },
        { 1000.f, 0.f, 0.f, 0.f, 1.f, 0.f, 1.f, 0.f },
        { 0.f, 0.f, -1000.f, 0.f, 1.f, 0.f, 0.f, 1.f },
    };

    byte_writer strs_payload;
    strings.serialize( strs_payload );
    byte_writer prot_payload;
    write_prototypes( prot_payload, protos );
    byte_writer inst_payload;
    write_instances( inst_payload, instances );
    byte_writer mesh_payload;
    write_terrain_meshes( mesh_payload, meshes );

    container_writer writer( file_kind::tile );
    writer.add_chunk( chunk::strs, strs_payload );
    writer.add_chunk( chunk::prot, prot_payload );
    writer.add_chunk( chunk::inst, inst_payload );
    writer.add_chunk( chunk::mesh, mesh_payload );
    auto const bytes { writer.data() };

    container_reader reader( bytes.data(), bytes.size() );
    string_pool pool;
    std::vector<model_prototype> rp;
    std::vector<model_instance> ri;
    std::vector<terrain_mesh> rm;
    chunk_view chunk;
    while( reader.next( chunk ) ) {
        auto r { chunk.reader() };
        if( chunk.id == chunk::strs ) {
            pool.deserialize( r );
        }
        else if( chunk.id == chunk::prot ) {
            rp = read_prototypes( r );
        }
        else if( chunk.id == chunk::inst ) {
            ri = read_instances( r );
        }
        else if( chunk.id == chunk::mesh ) {
            rm = read_terrain_meshes( r );
        }
    }

    check( rp.size() == 2, "proto count" );
    check( rp[ 0 ].flags == ( proto_flag::instanceable | proto_flag::transition ), "proto0 flags" );
    check( rp[ 0 ].range_max == 4000.f, "proto0 range" );
    check( rp[ 0 ].light_states.size() == 3 && rp[ 0 ].light_states[ 1 ] == 0.5f, "proto0 light states" );
    check( rp[ 0 ].light_colors.size() == 2 && rp[ 0 ].light_colors[ 0 ] == 0xff0000ffu, "proto0 light colors" );
    check( pool.get( rp[ 0 ].model_file ) == "models/sem.e3d", "proto0 model string" );

    check( ri.size() == 3, "instance count" );
    check( ri[ 0 ].proto == 0 && ri[ 0 ].cell_id == 7, "inst0 proto/cell" );
    check( ri[ 0 ].sx == 1.f && ri[ 0 ].texture_override == kNoString, "inst0 defaults survive" );
    check( ri[ 1 ].x == 999999.5 && ri[ 1 ].z == -888888.0, "inst1 large coord precision" );
    check( ri[ 1 ].sx == 2.f && ri[ 1 ].sy == 2.f && ri[ 1 ].sz == 2.f, "inst1 scale" );
    check( pool.get( ri[ 1 ].texture_override ) == "textures/sem_winter.dds", "inst1 override" );
    check( ri[ 2 ].cell_id == 0xffu, "inst2 no cell" );

    check( rm.size() == 1, "mesh count" );
    check( rm[ 0 ].ox == 500000.0 && rm[ 0 ].oz == -500000.0, "mesh origin f64 precision" );
    check( rm[ 0 ].vertices.size() == 3, "mesh vertex count" );
    check( rm[ 0 ].vertices[ 1 ].px == 1000.f && rm[ 0 ].vertices[ 1 ].u == 1.f, "mesh vertex data" );
    check( pool.get( rm[ 0 ].material ) == "textures/grass.dds", "mesh material string" );
}

void
verify_records_roundtrip() {
    string_table s;

    // --- tracks ---
    std::vector<track_record> tracks( 1 );
    tracks[ 0 ].node.name = s.intern( "track_42" );
    tracks[ 0 ].node.type = s.intern( "track" );
    tracks[ 0 ].node.area_center = { 123456.5, 0.0, -654321.0 };
    tracks[ 0 ].node.range_sq_max = 1.0e12;
    tracks[ 0 ].track_type = 2; // switch
    tracks[ 0 ].category = 1;
    tracks[ 0 ].length = 50.0f;
    tracks[ 0 ].environment = 3;
    tracks[ 0 ].has_visibility = true;
    tracks[ 0 ].visibility.material1 = s.intern( "rail.mat" );
    tracks[ 0 ].visibility.tex_length = 8.f;
    tracks[ 0 ].paths.push_back( { { 1.0, 2.0, 3.0 }, 0.1, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 }, { 10.0, 11.0, 12.0 }, 0.2, 300.0 } );
    tracks[ 0 ].tail_keywords.emplace_back( s.intern( "event0" ), s.intern( "ev_start" ) );

    // --- traction ---
    std::vector<traction_record> traction( 1 );
    traction[ 0 ].node.name = s.intern( "trakcja1" );
    traction[ 0 ].power_supply_name = s.intern( "psupply" );
    traction[ 0 ].nominal_voltage = 3000.f;
    traction[ 0 ].material = 1;
    traction[ 0 ].wire_p1 = { 0.0, 6.0, 0.0 };
    traction[ 0 ].wire_p2 = { 100.0, 6.0, 0.0 };
    traction[ 0 ].wire_count = 2;
    traction[ 0 ].has_parallel = true;
    traction[ 0 ].parallel_name = s.intern( "trakcja0" );

    // --- power sources ---
    std::vector<power_source_record> power( 1 );
    power[ 0 ].node.name = s.intern( "psupply" );
    power[ 0 ].position = { 50.0, 0.0, -50.0 };
    power[ 0 ].nominal_voltage = 3000.f;
    power[ 0 ].max_output_current = 4000.f;
    power[ 0 ].modifier = 2;

    // --- memcells ---
    std::vector<memcell_record> memcells( 1 );
    memcells[ 0 ].node.name = s.intern( "mc_station" );
    memcells[ 0 ].text = s.intern( "PASS depot 1" );
    memcells[ 0 ].value1 = 12.5;
    memcells[ 0 ].value2 = -7.0;
    memcells[ 0 ].has_track = true;
    memcells[ 0 ].track_name = s.intern( "track_42" );

    // --- launchers ---
    std::vector<launcher_record> launchers( 1 );
    launchers[ 0 ].node.name = s.intern( "launch1" );
    launchers[ 0 ].location = { 200.0, 1.0, -200.0 };
    launchers[ 0 ].radius_squared = 400.0;
    launchers[ 0 ].activation_key = 65;
    launchers[ 0 ].event1_name = s.intern( "ev_start" );
    launchers[ 0 ].has_condition = true;
    launchers[ 0 ].condition.memcell_name = s.intern( "mc_station" );
    launchers[ 0 ].condition.compare_value1 = 12.5;
    launchers[ 0 ].condition.check_mask = 3;
    launchers[ 0 ].launch_hour = 14;
    launchers[ 0 ].launch_minute = 30;

    // --- events ---
    std::vector<event_record> events( 1 );
    events[ 0 ].name = s.intern( "ev_start" );
    events[ 0 ].type = 7; // multiple
    events[ 0 ].delay = 2.5;
    events[ 0 ].passive = true;
    events[ 0 ].targets = { s.intern( "ev_a" ), s.intern( "ev_b" ) };
    events[ 0 ].payload.emplace_back( s.intern( "k1" ), s.intern( "v1" ) );

    // --- sounds ---
    std::vector<sound_record> sounds( 1 );
    sounds[ 0 ].node.name = s.intern( "snd1" );
    sounds[ 0 ].location = { -10.0, 2.0, 10.0 };
    sounds[ 0 ].wav_file = s.intern( "ambient.wav" );

    // --- dynamics ---
    std::vector<dynamic_record> dynamics( 1 );
    dynamics[ 0 ].node.name = s.intern( "veh1" );
    dynamics[ 0 ].data_folder = s.intern( "pkp/ep07" );
    dynamics[ 0 ].mmd_file = s.intern( "ep07.mmd" );
    dynamics[ 0 ].track_name = s.intern( "track_42" );
    dynamics[ 0 ].offset = 25.0;
    dynamics[ 0 ].coupling = 3;
    dynamics[ 0 ].velocity = 40.f;
    dynamics[ 0 ].has_destination = true;
    dynamics[ 0 ].destination = s.intern( "Warszawa" );
    dynamics[ 0 ].has_trainset = true;
    dynamics[ 0 ].trainset_index = 5;

    byte_writer strs_payload, trak_p, trac_p, pwrs_p, memc_p, laun_p, evnt_p, sond_p, dynm_p;
    s.serialize( strs_payload );
    write_tracks( trak_p, tracks );
    write_traction( trac_p, traction );
    write_power_sources( pwrs_p, power );
    write_memcells( memc_p, memcells );
    write_launchers( laun_p, launchers );
    write_events( evnt_p, events );
    write_sounds( sond_p, sounds );
    write_dynamics( dynm_p, dynamics );

    container_writer w( file_kind::sim );
    w.add_chunk( chunk::strs, strs_payload );
    w.add_chunk( chunk::trak, trak_p );
    w.add_chunk( chunk::trac, trac_p );
    w.add_chunk( chunk::pwrs, pwrs_p );
    w.add_chunk( chunk::memc, memc_p );
    w.add_chunk( chunk::laun, laun_p );
    w.add_chunk( chunk::evnt, evnt_p );
    w.add_chunk( chunk::sond, sond_p );
    w.add_chunk( chunk::dynm, dynm_p );
    auto const bytes { w.data() };

    container_reader r( bytes.data(), bytes.size() );
    check( r.kind() == file_kind::sim, "records sim kind" );

    string_pool pool;
    std::vector<track_record> rt;
    std::vector<traction_record> rtr;
    std::vector<power_source_record> rp;
    std::vector<memcell_record> rmc;
    std::vector<launcher_record> rl;
    std::vector<event_record> re;
    std::vector<sound_record> rs;
    std::vector<dynamic_record> rd;
    chunk_view c;
    while( r.next( c ) ) {
        auto rr { c.reader() };
        switch( c.id ) {
            case chunk::strs: pool.deserialize( rr ); break;
            case chunk::trak: rt = read_tracks( rr ); break;
            case chunk::trac: rtr = read_traction( rr ); break;
            case chunk::pwrs: rp = read_power_sources( rr ); break;
            case chunk::memc: rmc = read_memcells( rr ); break;
            case chunk::laun: rl = read_launchers( rr ); break;
            case chunk::evnt: re = read_events( rr ); break;
            case chunk::sond: rs = read_sounds( rr ); break;
            case chunk::dynm: rd = read_dynamics( rr ); break;
            default: break;
        }
    }

    check( rt.size() == 1 && pool.get( rt[ 0 ].node.name ) == "track_42", "track name" );
    check( rt[ 0 ].node.area_center.x == 123456.5 && rt[ 0 ].node.area_center.z == -654321.0, "track center f64" );
    check( rt[ 0 ].track_type == 2 && rt[ 0 ].environment == 3, "track type/env" );
    check( rt[ 0 ].has_visibility && pool.get( rt[ 0 ].visibility.material1 ) == "rail.mat", "track visibility" );
    check( rt[ 0 ].paths.size() == 1 && rt[ 0 ].paths[ 0 ].radius == 300.0, "track path" );
    check( rt[ 0 ].paths[ 0 ].p_end.z == 12.0, "track path endpoint" );
    check( rt[ 0 ].tail_keywords.size() == 1 && pool.get( rt[ 0 ].tail_keywords[ 0 ].second ) == "ev_start", "track keyword" );

    check( rtr.size() == 1 && rtr[ 0 ].nominal_voltage == 3000.f, "traction voltage" );
    check( rtr[ 0 ].wire_p2.x == 100.0 && rtr[ 0 ].wire_count == 2, "traction wire" );
    check( rtr[ 0 ].has_parallel && pool.get( rtr[ 0 ].parallel_name ) == "trakcja0", "traction parallel" );

    check( rp.size() == 1 && rp[ 0 ].max_output_current == 4000.f && rp[ 0 ].modifier == 2, "power source" );

    check( rmc.size() == 1 && pool.get( rmc[ 0 ].text ) == "PASS depot 1", "memcell text" );
    check( rmc[ 0 ].value1 == 12.5 && rmc[ 0 ].has_track, "memcell value/track" );

    check( rl.size() == 1 && rl[ 0 ].activation_key == 65, "launcher key" );
    check( rl[ 0 ].has_condition && rl[ 0 ].condition.check_mask == 3, "launcher condition" );
    check( rl[ 0 ].launch_hour == 14 && rl[ 0 ].launch_minute == 30, "launcher time" );

    check( re.size() == 1 && re[ 0 ].type == 7 && re[ 0 ].passive, "event type" );
    check( re[ 0 ].targets.size() == 2 && pool.get( re[ 0 ].targets[ 1 ] ) == "ev_b", "event targets" );
    check( re[ 0 ].payload.size() == 1 && pool.get( re[ 0 ].payload[ 0 ].first ) == "k1", "event payload" );

    check( rs.size() == 1 && pool.get( rs[ 0 ].wav_file ) == "ambient.wav", "sound wav" );

    check( rd.size() == 1 && pool.get( rd[ 0 ].data_folder ) == "pkp/ep07", "dynamic folder" );
    check( rd[ 0 ].offset == 25.0 && rd[ 0 ].velocity == 40.f, "dynamic offset/vel" );
    check( rd[ 0 ].has_destination && pool.get( rd[ 0 ].destination ) == "Warszawa", "dynamic destination" );
    check( rd[ 0 ].has_trainset && rd[ 0 ].trainset_index == 5, "dynamic trainset" );
}

void
verify_extensions_roundtrip() {
    string_table s;

    // --- shapes (SHPE) ---
    std::vector<shape_record> shapes( 1 );
    shapes[ 0 ].node.name = s.intern( "shape0" );
    shapes[ 0 ].node.type = s.intern( "triangles" );
    shapes[ 0 ].translucent = true;
    shapes[ 0 ].material = s.intern( "bricks.mat" );
    shapes[ 0 ].lighting.diffuse[ 0 ] = 0.5f;
    shapes[ 0 ].ox = 12345.5;
    shapes[ 0 ].oy = 1.0;
    shapes[ 0 ].oz = -54321.0;
    shapes[ 0 ].vertices = {
        { 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f },
        { 10.f, 0.f, 0.f, 0.f, 1.f, 0.f, 1.f, 0.f },
        { 0.f, 5.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f },
    };

    // --- lines (LINE) ---
    std::vector<lines_record> lines( 1 );
    lines[ 0 ].node.name = s.intern( "line0" );
    lines[ 0 ].line_width = 2.f;
    lines[ 0 ].ox = 100.0;
    lines[ 0 ].vertices = { { 0.0, 0.0, 0.0 }, { 50.0, 0.0, -3.0 } };

    // --- trainsets (TRST) ---
    std::vector<trainset_record> trainsets( 1 );
    trainsets[ 0 ].name = s.intern( "ts_warszawa" );
    trainsets[ 0 ].track = s.intern( "track_42" );
    trainsets[ 0 ].offset = 25.5f;
    trainsets[ 0 ].velocity = 60.f;
    trainsets[ 0 ].assignment.emplace_back( s.intern( "k" ), s.intern( "v" ) );
    trainsets[ 0 ].vehicle_indices = { 0u, 1u, 2u };
    trainsets[ 0 ].couplings = { 3, -1 };
    trainsets[ 0 ].driver_index = 1u;

    // --- includes (INCL) ---
    std::vector<include_record> includes( 1 );
    includes[ 0 ].source_line = 17;
    includes[ 0 ].source_path = s.intern( "scenery/sub.inc" );
    includes[ 0 ].binary_path = s.intern( "scenery/sub.eu7v2" );
    includes[ 0 ].parameters = { s.intern( "p1" ), s.intern( "p2" ) };
    includes[ 0 ].site_transform.origin_stack.push_back( { 10.0, 0.0, -20.0 } );
    includes[ 0 ].site_transform.rotation = { 0.0, 90.0, 0.0 };
    includes[ 0 ].site_transform.group_depth = 1;

    // --- instance with full node ---
    std::vector<model_prototype> protos( 1 );
    protos[ 0 ].model_file = s.intern( "models/x.e3d" );
    std::vector<model_instance> instances( 1 );
    instances[ 0 ].proto = 0;
    instances[ 0 ].x = 7.0;
    instances[ 0 ].has_node = true;
    instances[ 0 ].node.name = s.intern( "inst_named" );
    instances[ 0 ].node.type = s.intern( "model" );
    instances[ 0 ].node.range_sq_max = 1.0e9;
    instances[ 0 ].node.visible = false;

    // --- meta (META) ---
    module_meta meta;
    meta.first_init_count = 4;
    meta.has_terrain_chunk = true;
    meta.placement_origin_x = 1;
    meta.placement_rotation_y = 4;

    byte_writer strs_p, shpe_p, line_p, trst_p, incl_p, prot_p, inst_p, meta_p;
    s.serialize( strs_p );
    write_shapes( shpe_p, shapes );
    write_lines( line_p, lines );
    write_trainsets( trst_p, trainsets );
    write_includes( incl_p, includes );
    write_prototypes( prot_p, protos );
    write_instances( inst_p, instances );
    write_meta( meta_p, meta );

    container_writer w( file_kind::module );
    w.add_chunk( chunk::strs, strs_p );
    w.add_chunk( chunk::meta, meta_p );
    w.add_chunk( chunk::incl, incl_p );
    w.add_chunk( chunk::shpe, shpe_p );
    w.add_chunk( chunk::line, line_p );
    w.add_chunk( chunk::trst, trst_p );
    w.add_chunk( chunk::prot, prot_p );
    w.add_chunk( chunk::inst, inst_p );
    auto const bytes { w.data() };

    container_reader r( bytes.data(), bytes.size() );
    check( r.kind() == file_kind::module, "ext module kind" );

    string_pool pool;
    std::vector<shape_record> rsh;
    std::vector<lines_record> rln;
    std::vector<trainset_record> rts;
    std::vector<include_record> rin;
    std::vector<model_instance> ri;
    module_meta rmeta;
    chunk_view c;
    while( r.next( c ) ) {
        auto rr { c.reader() };
        switch( c.id ) {
            case chunk::strs: pool.deserialize( rr ); break;
            case chunk::meta: rmeta = read_meta( rr ); break;
            case chunk::incl: rin = read_includes( rr ); break;
            case chunk::shpe: rsh = read_shapes( rr ); break;
            case chunk::line: rln = read_lines( rr ); break;
            case chunk::trst: rts = read_trainsets( rr ); break;
            case chunk::inst: ri = read_instances( rr ); break;
            default: break;
        }
    }

    check( rsh.size() == 1 && rsh[ 0 ].translucent, "shape translucent" );
    check( rsh[ 0 ].ox == 12345.5 && rsh[ 0 ].oz == -54321.0, "shape origin f64" );
    check( rsh[ 0 ].vertices.size() == 3 && rsh[ 0 ].vertices[ 1 ].px == 10.f, "shape verts" );
    check( pool.get( rsh[ 0 ].material ) == "bricks.mat", "shape material" );
    check( rsh[ 0 ].lighting.diffuse[ 0 ] == 0.5f, "shape lighting" );

    check( rln.size() == 1 && rln[ 0 ].line_width == 2.f, "line width" );
    check( rln[ 0 ].vertices.size() == 2 && rln[ 0 ].vertices[ 1 ].x == 50.0, "line verts" );

    check( rts.size() == 1 && pool.get( rts[ 0 ].name ) == "ts_warszawa", "trainset name" );
    check( rts[ 0 ].vehicle_indices.size() == 3 && rts[ 0 ].vehicle_indices[ 2 ] == 2u, "trainset vehicles" );
    check( rts[ 0 ].couplings.size() == 2 && rts[ 0 ].couplings[ 1 ] == -1, "trainset couplings" );
    check( rts[ 0 ].driver_index == 1u && rts[ 0 ].offset == 25.5f, "trainset driver/offset" );

    check( rin.size() == 1 && rin[ 0 ].source_line == 17, "include line" );
    check( pool.get( rin[ 0 ].binary_path ) == "scenery/sub.eu7v2", "include binary path .eu7v2" );
    check( rin[ 0 ].parameters.size() == 2, "include params" );
    check( rin[ 0 ].site_transform.origin_stack.size() == 1 &&
           rin[ 0 ].site_transform.origin_stack[ 0 ].z == -20.0, "include transform origin" );
    check( rin[ 0 ].site_transform.rotation.y == 90.0 &&
           rin[ 0 ].site_transform.group_depth == 1, "include transform rot/group" );

    check( ri.size() == 1 && ri[ 0 ].has_node, "inst has node" );
    check( pool.get( ri[ 0 ].node.name ) == "inst_named" && !ri[ 0 ].node.visible, "inst node fields" );
    check( ri[ 0 ].node.range_sq_max == 1.0e9, "inst node range" );

    check( rmeta.first_init_count == 4 && rmeta.has_terrain_chunk, "meta counts/flags" );
    check( rmeta.placement_origin_x == 1 && rmeta.placement_rotation_y == 4, "meta placement" );
}

void
verify_file_io() {
    auto const bytes { build_tile() };
    char const *path { "eu7v2_roundtrip.bin" };
    {
        std::ofstream os( path, std::ios::binary );
        os.write( reinterpret_cast<char const *>( bytes.data() ),
                  static_cast<std::streamsize>( bytes.size() ) );
    }
    std::ifstream is( path, std::ios::binary | std::ios::ate );
    auto const size { static_cast<std::size_t>( is.tellg() ) };
    is.seekg( 0 );
    std::vector<std::uint8_t> loaded( size );
    is.read( reinterpret_cast<char *>( loaded.data() ),
             static_cast<std::streamsize>( size ) );
    check( loaded == bytes, "file write/read byte-identical" );
    verify_tile( loaded );
    std::remove( path );
}

} // namespace

int
main() {
    std::printf( "eu7v2 format core round-trip test\n" );

    auto const tile { build_tile() };
    verify_tile( tile );
    verify_trgr_roundtrip();
    verify_scene_roundtrip();
    verify_records_roundtrip();
    verify_extensions_roundtrip();
    verify_truncation_is_rejected();
    verify_file_io();

    std::printf( "checks: %d, failures: %d\n", g_checks, g_failures );
    if( g_failures == 0 ) {
        std::printf( "OK\n" );
        return 0;
    }
    std::printf( "FAILED\n" );
    return 1;
}
