/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// ---------------------------------------------------------------------------
// eu7v2 simulation records (iteration 2b): the non-visual scene data that is
// loaded once into the sim core - tracks, traction, power sources, memory
// cells, event launchers, events, sounds and dynamic vehicles.
//
// All strings are referenced by string-table id; optional fields use a leading
// presence flag so absent data costs a single byte. Dependency-free so the
// encode/decode path stays unit testable with a standalone compiler.
// ---------------------------------------------------------------------------

#include "eu7v2_format.h"
#include "eu7v2_scene.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace eu7v2 {

// Shared helpers (put_strid/dvec3/node_record/...) live in eu7v2_scene.h so
// scene payloads and these sim records share a single definition.

// --- TRAK : tracks ---------------------------------------------------------

struct track_path {
    dvec3 p_start;
    double roll_start { 0.0 };
    dvec3 cp_out;
    dvec3 cp_in;
    dvec3 p_end;
    double roll_end { 0.0 };
    double radius { 0.0 };
};

struct track_visibility {
    std::uint32_t material1 { kNoString };
    float tex_length { 4.f };
    std::uint32_t material2 { kNoString };
    float tex_height1 { 0.f };
    float tex_width { 0.f };
    float tex_slope { 0.f };
};

struct track_record {
    node_record node;
    std::uint8_t track_type { 0 };
    std::uint8_t category { 1 };
    float length { 0.f };
    float track_width { 0.f };
    float friction { 0.f };
    float sound_distance { 0.f };
    std::int32_t quality_flag { 0 };
    std::int32_t damage_flag { 0 };
    std::int8_t environment { -1 };
    bool has_visibility { false };
    track_visibility visibility;
    std::vector<track_path> paths;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> tail_keywords; // (key strid, value strid)
};

inline void write_tracks( byte_writer &out, std::vector<track_record> const &tracks ) {
    out.put_u32( static_cast<std::uint32_t>( tracks.size() ) );
    for( auto const &t : tracks ) {
        write_node( out, t.node );
        out.put_u8( t.track_type );
        out.put_u8( t.category );
        out.put_f32( t.length );
        out.put_f32( t.track_width );
        out.put_f32( t.friction );
        out.put_f32( t.sound_distance );
        out.put_i32( t.quality_flag );
        out.put_i32( t.damage_flag );
        out.put_u8( static_cast<std::uint8_t>( t.environment ) );
        out.put_u8( t.has_visibility ? 1u : 0u );
        if( t.has_visibility ) {
            out.put_u32( t.visibility.material1 );
            out.put_f32( t.visibility.tex_length );
            out.put_u32( t.visibility.material2 );
            out.put_f32( t.visibility.tex_height1 );
            out.put_f32( t.visibility.tex_width );
            out.put_f32( t.visibility.tex_slope );
        }
        out.put_u32( static_cast<std::uint32_t>( t.paths.size() ) );
        for( auto const &p : t.paths ) {
            put_dvec3( out, p.p_start.x, p.p_start.y, p.p_start.z );
            out.put_f64( p.roll_start );
            put_dvec3( out, p.cp_out.x, p.cp_out.y, p.cp_out.z );
            put_dvec3( out, p.cp_in.x, p.cp_in.y, p.cp_in.z );
            put_dvec3( out, p.p_end.x, p.p_end.y, p.p_end.z );
            out.put_f64( p.roll_end );
            out.put_f64( p.radius );
        }
        out.put_u32( static_cast<std::uint32_t>( t.tail_keywords.size() ) );
        for( auto const &kv : t.tail_keywords ) {
            out.put_u32( kv.first );
            out.put_u32( kv.second );
        }
    }
}

inline std::vector<track_record> read_tracks( byte_reader &in ) {
    std::vector<track_record> tracks;
    auto const count { in.get_u32() };
    tracks.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        track_record t;
        t.node = read_node( in );
        t.track_type = in.get_u8();
        t.category = in.get_u8();
        t.length = in.get_f32();
        t.track_width = in.get_f32();
        t.friction = in.get_f32();
        t.sound_distance = in.get_f32();
        t.quality_flag = in.get_i32();
        t.damage_flag = in.get_i32();
        t.environment = static_cast<std::int8_t>( in.get_u8() );
        t.has_visibility = in.get_u8() != 0;
        if( t.has_visibility ) {
            t.visibility.material1 = in.get_u32();
            t.visibility.tex_length = in.get_f32();
            t.visibility.material2 = in.get_u32();
            t.visibility.tex_height1 = in.get_f32();
            t.visibility.tex_width = in.get_f32();
            t.visibility.tex_slope = in.get_f32();
        }
        auto const paths { in.get_u32() };
        t.paths.reserve( paths );
        for( std::uint32_t p { 0 }; p < paths; ++p ) {
            track_path tp;
            tp.p_start = get_dvec3( in );
            tp.roll_start = in.get_f64();
            tp.cp_out = get_dvec3( in );
            tp.cp_in = get_dvec3( in );
            tp.p_end = get_dvec3( in );
            tp.roll_end = in.get_f64();
            tp.radius = in.get_f64();
            t.paths.push_back( tp );
        }
        auto const kws { in.get_u32() };
        t.tail_keywords.reserve( kws );
        for( std::uint32_t k { 0 }; k < kws; ++k ) {
            auto const key { in.get_u32() };
            auto const value { in.get_u32() };
            t.tail_keywords.emplace_back( key, value );
        }
        tracks.push_back( std::move( t ) );
    }
    return tracks;
}

// --- TRAC : traction -------------------------------------------------------

struct traction_record {
    node_record node;
    std::uint32_t power_supply_name { kNoString };
    float nominal_voltage { 0.f };
    float max_current { 0.f };
    float resistivity { 0.f };
    std::uint8_t material { 1 };
    float wire_thickness { 0.f };
    std::int32_t damage_flag { 0 };
    dvec3 wire_p1, wire_p2, wire_p3, wire_p4;
    double min_height { 0.0 };
    double segment_length { 0.0 };
    std::int32_t wire_count { 0 };
    float wire_offset { 0.f };
    bool has_parallel { false };
    std::uint32_t parallel_name { kNoString };
};

inline void write_traction( byte_writer &out, std::vector<traction_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &t : items ) {
        write_node( out, t.node );
        out.put_u32( t.power_supply_name );
        out.put_f32( t.nominal_voltage );
        out.put_f32( t.max_current );
        out.put_f32( t.resistivity );
        out.put_u8( t.material );
        out.put_f32( t.wire_thickness );
        out.put_i32( t.damage_flag );
        put_dvec3( out, t.wire_p1.x, t.wire_p1.y, t.wire_p1.z );
        put_dvec3( out, t.wire_p2.x, t.wire_p2.y, t.wire_p2.z );
        put_dvec3( out, t.wire_p3.x, t.wire_p3.y, t.wire_p3.z );
        put_dvec3( out, t.wire_p4.x, t.wire_p4.y, t.wire_p4.z );
        out.put_f64( t.min_height );
        out.put_f64( t.segment_length );
        out.put_i32( t.wire_count );
        out.put_f32( t.wire_offset );
        put_opt_strid( out, t.has_parallel, t.parallel_name );
    }
}

inline std::vector<traction_record> read_traction( byte_reader &in ) {
    std::vector<traction_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        traction_record t;
        t.node = read_node( in );
        t.power_supply_name = in.get_u32();
        t.nominal_voltage = in.get_f32();
        t.max_current = in.get_f32();
        t.resistivity = in.get_f32();
        t.material = in.get_u8();
        t.wire_thickness = in.get_f32();
        t.damage_flag = in.get_i32();
        t.wire_p1 = get_dvec3( in );
        t.wire_p2 = get_dvec3( in );
        t.wire_p3 = get_dvec3( in );
        t.wire_p4 = get_dvec3( in );
        t.min_height = in.get_f64();
        t.segment_length = in.get_f64();
        t.wire_count = in.get_i32();
        t.wire_offset = in.get_f32();
        t.has_parallel = in.get_u8() != 0;
        if( t.has_parallel ) {
            t.parallel_name = in.get_u32();
        }
        items.push_back( t );
    }
    return items;
}

// --- PWRS : traction power sources -----------------------------------------

struct power_source_record {
    node_record node;
    dvec3 position;
    float nominal_voltage { 0.f };
    float voltage_frequency { 0.f };
    float internal_resistance { 0.2f };
    float max_output_current { 0.f };
    float fast_fuse_timeout { 0.f };
    float fast_fuse_repetition { 0.f };
    float slow_fuse_timeout { 0.f };
    std::uint8_t modifier { 0 };
};

inline void write_power_sources( byte_writer &out, std::vector<power_source_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &p : items ) {
        write_node( out, p.node );
        put_dvec3( out, p.position.x, p.position.y, p.position.z );
        out.put_f32( p.nominal_voltage );
        out.put_f32( p.voltage_frequency );
        out.put_f32( p.internal_resistance );
        out.put_f32( p.max_output_current );
        out.put_f32( p.fast_fuse_timeout );
        out.put_f32( p.fast_fuse_repetition );
        out.put_f32( p.slow_fuse_timeout );
        out.put_u8( p.modifier );
    }
}

inline std::vector<power_source_record> read_power_sources( byte_reader &in ) {
    std::vector<power_source_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        power_source_record p;
        p.node = read_node( in );
        p.position = get_dvec3( in );
        p.nominal_voltage = in.get_f32();
        p.voltage_frequency = in.get_f32();
        p.internal_resistance = in.get_f32();
        p.max_output_current = in.get_f32();
        p.fast_fuse_timeout = in.get_f32();
        p.fast_fuse_repetition = in.get_f32();
        p.slow_fuse_timeout = in.get_f32();
        p.modifier = in.get_u8();
        items.push_back( p );
    }
    return items;
}

// --- MEMC : memory cells ---------------------------------------------------

struct memcell_record {
    node_record node;
    std::uint32_t text { kNoString };
    double value1 { 0.0 };
    double value2 { 0.0 };
    bool has_track { false };
    std::uint32_t track_name { kNoString };
};

inline void write_memcells( byte_writer &out, std::vector<memcell_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &m : items ) {
        write_node( out, m.node );
        out.put_u32( m.text );
        out.put_f64( m.value1 );
        out.put_f64( m.value2 );
        put_opt_strid( out, m.has_track, m.track_name );
    }
}

inline std::vector<memcell_record> read_memcells( byte_reader &in ) {
    std::vector<memcell_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        memcell_record m;
        m.node = read_node( in );
        m.text = in.get_u32();
        m.value1 = in.get_f64();
        m.value2 = in.get_f64();
        m.has_track = in.get_u8() != 0;
        if( m.has_track ) {
            m.track_name = in.get_u32();
        }
        items.push_back( m );
    }
    return items;
}

// --- LAUN : event launchers ------------------------------------------------

struct launcher_condition {
    std::uint32_t memcell_name { kNoString };
    std::uint32_t compare_text { kNoString };
    double compare_value1 { 0.0 };
    double compare_value2 { 0.0 };
    std::int32_t check_mask { 0 };
};

struct launcher_record {
    node_record node;
    dvec3 location;
    double radius_squared { 0.0 };
    std::int32_t activation_key { 0 };
    double delta_time { -1.0 };
    std::uint32_t event1_name { kNoString };
    std::uint32_t event2_name { kNoString };
    bool has_condition { false };
    launcher_condition condition;
    bool train_triggered { false };
    std::int32_t launch_hour { -1 };
    std::int32_t launch_minute { -1 };
};

inline void write_launchers( byte_writer &out, std::vector<launcher_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &l : items ) {
        write_node( out, l.node );
        put_dvec3( out, l.location.x, l.location.y, l.location.z );
        out.put_f64( l.radius_squared );
        out.put_i32( l.activation_key );
        out.put_f64( l.delta_time );
        out.put_u32( l.event1_name );
        out.put_u32( l.event2_name );
        out.put_u8( l.has_condition ? 1u : 0u );
        if( l.has_condition ) {
            out.put_u32( l.condition.memcell_name );
            out.put_u32( l.condition.compare_text );
            out.put_f64( l.condition.compare_value1 );
            out.put_f64( l.condition.compare_value2 );
            out.put_i32( l.condition.check_mask );
        }
        out.put_u8( l.train_triggered ? 1u : 0u );
        out.put_i32( l.launch_hour );
        out.put_i32( l.launch_minute );
    }
}

inline std::vector<launcher_record> read_launchers( byte_reader &in ) {
    std::vector<launcher_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        launcher_record l;
        l.node = read_node( in );
        l.location = get_dvec3( in );
        l.radius_squared = in.get_f64();
        l.activation_key = in.get_i32();
        l.delta_time = in.get_f64();
        l.event1_name = in.get_u32();
        l.event2_name = in.get_u32();
        l.has_condition = in.get_u8() != 0;
        if( l.has_condition ) {
            l.condition.memcell_name = in.get_u32();
            l.condition.compare_text = in.get_u32();
            l.condition.compare_value1 = in.get_f64();
            l.condition.compare_value2 = in.get_f64();
            l.condition.check_mask = in.get_i32();
        }
        l.train_triggered = in.get_u8() != 0;
        l.launch_hour = in.get_i32();
        l.launch_minute = in.get_i32();
        items.push_back( l );
    }
    return items;
}

// --- EVNT : events ---------------------------------------------------------

struct event_record {
    std::uint32_t name { kNoString };
    std::uint8_t type { 0 };
    double delay { 0.0 };
    double delay_random { 0.0 };
    double delay_departure { 0.0 };
    bool ignored { false };
    bool passive { false };
    std::vector<std::uint32_t> targets; // string ids
    std::vector<std::pair<std::uint32_t, std::uint32_t>> payload; // (key strid, value strid)
};

inline void write_events( byte_writer &out, std::vector<event_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &e : items ) {
        out.put_u32( e.name );
        out.put_u8( e.type );
        out.put_f64( e.delay );
        out.put_f64( e.delay_random );
        out.put_f64( e.delay_departure );
        out.put_u8( e.ignored ? 1u : 0u );
        out.put_u8( e.passive ? 1u : 0u );
        out.put_u32( static_cast<std::uint32_t>( e.targets.size() ) );
        for( auto const t : e.targets ) {
            out.put_u32( t );
        }
        out.put_u32( static_cast<std::uint32_t>( e.payload.size() ) );
        for( auto const &kv : e.payload ) {
            out.put_u32( kv.first );
            out.put_u32( kv.second );
        }
    }
}

inline std::vector<event_record> read_events( byte_reader &in ) {
    std::vector<event_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        event_record e;
        e.name = in.get_u32();
        e.type = in.get_u8();
        e.delay = in.get_f64();
        e.delay_random = in.get_f64();
        e.delay_departure = in.get_f64();
        e.ignored = in.get_u8() != 0;
        e.passive = in.get_u8() != 0;
        auto const targets { in.get_u32() };
        e.targets.reserve( targets );
        for( std::uint32_t t { 0 }; t < targets; ++t ) {
            e.targets.push_back( in.get_u32() );
        }
        auto const payload { in.get_u32() };
        e.payload.reserve( payload );
        for( std::uint32_t p { 0 }; p < payload; ++p ) {
            auto const key { in.get_u32() };
            auto const value { in.get_u32() };
            e.payload.emplace_back( key, value );
        }
        items.push_back( std::move( e ) );
    }
    return items;
}

// --- SOND : sounds ---------------------------------------------------------

struct sound_record {
    node_record node;
    dvec3 location;
    std::uint32_t wav_file { kNoString };
};

inline void write_sounds( byte_writer &out, std::vector<sound_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &s : items ) {
        write_node( out, s.node );
        put_dvec3( out, s.location.x, s.location.y, s.location.z );
        out.put_u32( s.wav_file );
    }
}

inline std::vector<sound_record> read_sounds( byte_reader &in ) {
    std::vector<sound_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        sound_record s;
        s.node = read_node( in );
        s.location = get_dvec3( in );
        s.wav_file = in.get_u32();
        items.push_back( s );
    }
    return items;
}

// --- DYNM : dynamic vehicles -----------------------------------------------

struct dynamic_record {
    node_record node;
    std::uint32_t data_folder { kNoString };
    std::uint32_t skin_file { kNoString };
    std::uint32_t mmd_file { kNoString };
    std::uint32_t track_name { kNoString };
    double offset { -1.0 };
    std::uint32_t driver_type { kNoString };
    std::int32_t coupling { 3 };
    std::uint32_t coupling_raw { kNoString };
    std::uint32_t coupling_params { kNoString };
    float velocity { 0.f };
    std::int32_t load_count { 0 };
    std::uint32_t load_type { kNoString };
    bool has_destination { false };
    std::uint32_t destination { kNoString };
    bool has_trainset { false };
    std::uint32_t trainset_index { 0 };
};

inline void write_dynamics( byte_writer &out, std::vector<dynamic_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &d : items ) {
        write_node( out, d.node );
        out.put_u32( d.data_folder );
        out.put_u32( d.skin_file );
        out.put_u32( d.mmd_file );
        out.put_u32( d.track_name );
        out.put_f64( d.offset );
        out.put_u32( d.driver_type );
        out.put_i32( d.coupling );
        out.put_u32( d.coupling_raw );
        out.put_u32( d.coupling_params );
        out.put_f32( d.velocity );
        out.put_i32( d.load_count );
        out.put_u32( d.load_type );
        put_opt_strid( out, d.has_destination, d.destination );
        out.put_u8( d.has_trainset ? 1u : 0u );
        if( d.has_trainset ) {
            out.put_u32( d.trainset_index );
        }
    }
}

inline std::vector<dynamic_record> read_dynamics( byte_reader &in ) {
    std::vector<dynamic_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        dynamic_record d;
        d.node = read_node( in );
        d.data_folder = in.get_u32();
        d.skin_file = in.get_u32();
        d.mmd_file = in.get_u32();
        d.track_name = in.get_u32();
        d.offset = in.get_f64();
        d.driver_type = in.get_u32();
        d.coupling = in.get_i32();
        d.coupling_raw = in.get_u32();
        d.coupling_params = in.get_u32();
        d.velocity = in.get_f32();
        d.load_count = in.get_i32();
        d.load_type = in.get_u32();
        d.has_destination = in.get_u8() != 0;
        if( d.has_destination ) {
            d.destination = in.get_u32();
        }
        d.has_trainset = in.get_u8() != 0;
        if( d.has_trainset ) {
            d.trainset_index = in.get_u32();
        }
        items.push_back( d );
    }
    return items;
}

// --- TRST : trainsets ------------------------------------------------------

struct trainset_record {
    std::uint32_t name { kNoString };
    std::uint32_t track { kNoString };
    float offset { 0.f };
    float velocity { 0.f };
    std::vector<std::pair<std::uint32_t, std::uint32_t>> assignment; // (key strid, value strid)
    std::vector<std::uint32_t> vehicle_indices;
    std::vector<std::int32_t> couplings;
    std::uint32_t driver_index { 0xffffffffu }; // (size_t)-1 sentinel
};

inline void write_trainsets( byte_writer &out, std::vector<trainset_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &t : items ) {
        out.put_u32( t.name );
        out.put_u32( t.track );
        out.put_f32( t.offset );
        out.put_f32( t.velocity );
        out.put_u32( static_cast<std::uint32_t>( t.assignment.size() ) );
        for( auto const &kv : t.assignment ) {
            out.put_u32( kv.first );
            out.put_u32( kv.second );
        }
        out.put_u32( static_cast<std::uint32_t>( t.vehicle_indices.size() ) );
        for( auto const idx : t.vehicle_indices ) {
            out.put_u32( idx );
        }
        out.put_u32( static_cast<std::uint32_t>( t.couplings.size() ) );
        for( auto const c : t.couplings ) {
            out.put_i32( c );
        }
        out.put_u32( t.driver_index );
    }
}

inline std::vector<trainset_record> read_trainsets( byte_reader &in ) {
    std::vector<trainset_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        trainset_record t;
        t.name = in.get_u32();
        t.track = in.get_u32();
        t.offset = in.get_f32();
        t.velocity = in.get_f32();
        auto const assign { in.get_u32() };
        t.assignment.reserve( assign );
        for( std::uint32_t a { 0 }; a < assign; ++a ) {
            auto const key { in.get_u32() };
            auto const value { in.get_u32() };
            t.assignment.emplace_back( key, value );
        }
        auto const vehicles { in.get_u32() };
        t.vehicle_indices.reserve( vehicles );
        for( std::uint32_t v { 0 }; v < vehicles; ++v ) {
            t.vehicle_indices.push_back( in.get_u32() );
        }
        auto const couplings { in.get_u32() };
        t.couplings.reserve( couplings );
        for( std::uint32_t c { 0 }; c < couplings; ++c ) {
            t.couplings.push_back( in.get_i32() );
        }
        t.driver_index = in.get_u32();
        items.push_back( std::move( t ) );
    }
    return items;
}

// --- PLCE : lean reusable-module placements (.inc with x/y/z/rot as f64/f32) -

struct module_placement_record {
    std::uint32_t module_path { kNoString }; // e.g. scenery/grass_l61/20.eu7v2
    std::uint32_t texture_override { kNoString };
    double x { 0.0 }, y { 0.0 }, z { 0.0 };
    float rotation_y { 0.f };
    std::uint8_t cell_id { 0xffu };
};

inline void write_module_placements(
    byte_writer &out,
    std::vector<module_placement_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &p : items ) {
        out.put_u32( p.module_path );
        out.put_u32( p.texture_override );
        out.put_f64( p.x );
        out.put_f64( p.y );
        out.put_f64( p.z );
        out.put_f32( p.rotation_y );
        out.put_u8( p.cell_id );
    }
}

inline std::vector<module_placement_record> read_module_placements( byte_reader &in ) {
    std::vector<module_placement_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        module_placement_record p;
        p.module_path = in.get_u32();
        p.texture_override = in.get_u32();
        p.x = in.get_f64();
        p.y = in.get_f64();
        p.z = in.get_f64();
        p.rotation_y = in.get_f32();
        p.cell_id = in.get_u8();
        items.push_back( std::move( p ) );
    }
    return items;
}

// --- INCL : module includes (recursion references) -------------------------

// Snapshot of the origin/scale/rotation stacks at include site (detokenizer).
struct transform_record {
    std::vector<dvec3> origin_stack;
    std::vector<dvec3> scale_stack;
    dvec3 rotation;
    std::uint32_t group_depth { 0 };
};

inline void write_transform( byte_writer &out, transform_record const &t ) {
    out.put_u32( static_cast<std::uint32_t>( t.origin_stack.size() ) );
    for( auto const &v : t.origin_stack ) {
        put_dvec3( out, v.x, v.y, v.z );
    }
    out.put_u32( static_cast<std::uint32_t>( t.scale_stack.size() ) );
    for( auto const &v : t.scale_stack ) {
        put_dvec3( out, v.x, v.y, v.z );
    }
    put_dvec3( out, t.rotation.x, t.rotation.y, t.rotation.z );
    out.put_u32( t.group_depth );
}

inline transform_record read_transform( byte_reader &in ) {
    transform_record t;
    auto const origins { in.get_u32() };
    t.origin_stack.reserve( origins );
    for( std::uint32_t i { 0 }; i < origins; ++i ) {
        t.origin_stack.push_back( get_dvec3( in ) );
    }
    auto const scales { in.get_u32() };
    t.scale_stack.reserve( scales );
    for( std::uint32_t i { 0 }; i < scales; ++i ) {
        t.scale_stack.push_back( get_dvec3( in ) );
    }
    t.rotation = get_dvec3( in );
    t.group_depth = in.get_u32();
    return t;
}

struct include_record {
    std::uint32_t source_line { 0 };
    std::uint32_t source_path { kNoString };
    std::uint32_t binary_path { kNoString }; // points at the .eu7v2 module
    std::vector<std::uint32_t> parameters;   // string ids
    transform_record site_transform;
};

inline void write_includes( byte_writer &out, std::vector<include_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &inc : items ) {
        out.put_u32( inc.source_line );
        out.put_u32( inc.source_path );
        out.put_u32( inc.binary_path );
        out.put_u32( static_cast<std::uint32_t>( inc.parameters.size() ) );
        for( auto const p : inc.parameters ) {
            out.put_u32( p );
        }
        write_transform( out, inc.site_transform );
    }
}

inline std::vector<include_record> read_includes( byte_reader &in ) {
    std::vector<include_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        include_record inc;
        inc.source_line = in.get_u32();
        inc.source_path = in.get_u32();
        inc.binary_path = in.get_u32();
        auto const params { in.get_u32() };
        inc.parameters.reserve( params );
        for( std::uint32_t p { 0 }; p < params; ++p ) {
            inc.parameters.push_back( in.get_u32() );
        }
        inc.site_transform = read_transform( in );
        items.push_back( std::move( inc ) );
    }
    return items;
}

// --- META : module-level metadata (flags, placement, counts) ---------------

struct module_meta {
    std::uint32_t first_init_count { 0 };
    bool has_terrain_chunk { false };
    bool has_pack_chunk { false };
    // include placement (parameter indices, 0 = unused)
    std::uint8_t placement_origin_x { 0 };
    std::uint8_t placement_origin_y { 0 };
    std::uint8_t placement_origin_z { 0 };
    std::uint8_t placement_rotation_y { 0 };
};

inline void write_meta( byte_writer &out, module_meta const &m ) {
    out.put_u32( 1u ); // meta layout version (forward-tolerant)
    out.put_u32( m.first_init_count );
    out.put_u8( m.has_terrain_chunk ? 1u : 0u );
    out.put_u8( m.has_pack_chunk ? 1u : 0u );
    out.put_u8( m.placement_origin_x );
    out.put_u8( m.placement_origin_y );
    out.put_u8( m.placement_origin_z );
    out.put_u8( m.placement_rotation_y );
}

inline module_meta read_meta( byte_reader &in ) {
    module_meta m;
    (void)in.get_u32(); // layout version
    m.first_init_count = in.get_u32();
    m.has_terrain_chunk = in.get_u8() != 0;
    m.has_pack_chunk = in.get_u8() != 0;
    m.placement_origin_x = in.get_u8();
    m.placement_origin_y = in.get_u8();
    m.placement_origin_z = in.get_u8();
    m.placement_rotation_y = in.get_u8();
    return m;
}

} // namespace eu7v2
