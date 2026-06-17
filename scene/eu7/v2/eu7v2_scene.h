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
// eu7v2 scene payloads (iteration 2 slice): the lean model + terrain records
// that fix the "heavy per-object" problem of the legacy streaming path.
//
//   PROT - deduplicated model prototypes (mesh + material + LOD + lights),
//          stored once per unique model, referenced by index.
//   INST - lean instance: prototype index + world transform + optional
//          per-instance texture override. No engine object is created here;
//          the runtime resolves prototype -> shared mesh and renders via
//          instancing.
//   MESH - baked terrain mesh: origin (f64) + vertices relative to origin
//          (f32) so a 1km tile keeps precision without paying f64 per vertex.
//
// Dependency-free on purpose so the whole encode/decode path is unit testable
// with a standalone compiler (see eu7v2_test.cpp), independent of the engine.
// ---------------------------------------------------------------------------

#include "eu7v2_format.h"

#include <cstdint>
#include <string>
#include <vector>

namespace eu7v2 {

constexpr std::uint32_t kNoString { 0xffffffffu };

// --- shared serialization helpers (used by scene payloads and sim records) -

inline void put_strid( byte_writer &out, std::uint32_t const id ) { out.put_u32( id ); }
inline std::uint32_t get_strid( byte_reader &in ) { return in.get_u32(); }

inline void put_dvec3( byte_writer &out, double const x, double const y, double const z ) {
    out.put_f64( x );
    out.put_f64( y );
    out.put_f64( z );
}

struct dvec3 {
    double x { 0.0 }, y { 0.0 }, z { 0.0 };
};

inline dvec3 get_dvec3( byte_reader &in ) {
    dvec3 v;
    v.x = in.get_f64();
    v.y = in.get_f64();
    v.z = in.get_f64();
    return v;
}

inline void put_opt_strid( byte_writer &out, bool const present, std::uint32_t const id ) {
    out.put_u8( present ? 1u : 0u );
    if( present ) {
        out.put_u32( id );
    }
}

// Common node metadata kept after baking (world-space, transform already applied).
struct node_record {
    std::uint32_t name { kNoString };
    std::uint32_t type { kNoString };
    dvec3 area_center;
    float area_radius { -1.f };
    double range_sq_min { 0.0 };
    double range_sq_max { 0.0 };
    bool visible { true };
};

inline void write_node( byte_writer &out, node_record const &n ) {
    out.put_u32( n.name );
    out.put_u32( n.type );
    put_dvec3( out, n.area_center.x, n.area_center.y, n.area_center.z );
    out.put_f32( n.area_radius );
    out.put_f64( n.range_sq_min );
    out.put_f64( n.range_sq_max );
    out.put_u8( n.visible ? 1u : 0u );
}

inline node_record read_node( byte_reader &in ) {
    node_record n;
    n.name = in.get_u32();
    n.type = in.get_u32();
    n.area_center = get_dvec3( in );
    n.area_radius = in.get_f32();
    n.range_sq_min = in.get_f64();
    n.range_sq_max = in.get_f64();
    n.visible = in.get_u8() != 0;
    return n;
}

// RGBA-ish lighting block stored as 12 floats (diffuse/ambient/specular vec4).
struct lighting_block {
    float diffuse[ 4 ] { 0.8f, 0.8f, 0.8f, 1.f };
    float ambient[ 4 ] { 0.2f, 0.2f, 0.2f, 1.f };
    float specular[ 4 ] { 0.f, 0.f, 0.f, 1.f };
};

inline void write_lighting( byte_writer &out, lighting_block const &l ) {
    for( auto const v : l.diffuse ) { out.put_f32( v ); }
    for( auto const v : l.ambient ) { out.put_f32( v ); }
    for( auto const v : l.specular ) { out.put_f32( v ); }
}

inline lighting_block read_lighting( byte_reader &in ) {
    lighting_block l;
    for( auto &v : l.diffuse ) { v = in.get_f32(); }
    for( auto &v : l.ambient ) { v = in.get_f32(); }
    for( auto &v : l.specular ) { v = in.get_f32(); }
    return l;
}

// Prototype flags packed into a single byte.
namespace proto_flag {
constexpr std::uint8_t transition { 1u << 0 };   // model has LOD transition
constexpr std::uint8_t is_terrain { 1u << 1 };   // terrain-style submodel split
constexpr std::uint8_t instanceable { 1u << 2 }; // safe to GPU-instance
} // namespace proto_flag

struct model_prototype {
    std::uint32_t model_file { kNoString };   // string id
    std::uint32_t texture_file { kNoString }; // string id (default skin)
    std::uint8_t flags { 0 };
    float range_min { -1.f };
    float range_max { -1.f };
    std::vector<float> light_states;
    std::vector<std::uint32_t> light_colors;
};

// Lean instance: which prototype, where, and an optional skin override.
struct model_instance {
    std::uint32_t proto { 0 };
    double x { 0.0 }, y { 0.0 }, z { 0.0 };
    float ax { 0.f }, ay { 0.f }, az { 0.f }; // euler angles (deg), engine convention
    float sx { 1.f }, sy { 1.f }, sz { 1.f }; // scale
    std::uint32_t texture_override { kNoString };
    std::uint8_t cell_id { 0xffu };
    bool has_node { false }; // full node metadata present (lossless path)
    node_record node;
};

struct mesh_vertex {
    float px { 0.f }, py { 0.f }, pz { 0.f }; // position relative to mesh origin
    float nx { 0.f }, ny { 0.f }, nz { 0.f }; // normal
    float u { 0.f }, v { 0.f };               // texcoord
};

struct terrain_mesh {
    std::uint32_t material { kNoString }; // string id
    bool translucent { false };
    double ox { 0.0 }, oy { 0.0 }, oz { 0.0 }; // world origin (f64)
    std::vector<mesh_vertex> vertices;
};

// --- PROT ------------------------------------------------------------------

inline void
write_prototypes( byte_writer &out, std::vector<model_prototype> const &protos ) {
    out.put_u32( static_cast<std::uint32_t>( protos.size() ) );
    for( auto const &p : protos ) {
        out.put_u32( p.model_file );
        out.put_u32( p.texture_file );
        out.put_u8( p.flags );
        out.put_f32( p.range_min );
        out.put_f32( p.range_max );
        out.put_u32( static_cast<std::uint32_t>( p.light_states.size() ) );
        for( auto const s : p.light_states ) {
            out.put_f32( s );
        }
        out.put_u32( static_cast<std::uint32_t>( p.light_colors.size() ) );
        for( auto const c : p.light_colors ) {
            out.put_u32( c );
        }
    }
}

inline std::vector<model_prototype>
read_prototypes( byte_reader &in ) {
    std::vector<model_prototype> protos;
    auto const count { in.get_u32() };
    protos.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        model_prototype p;
        p.model_file = in.get_u32();
        p.texture_file = in.get_u32();
        p.flags = in.get_u8();
        p.range_min = in.get_f32();
        p.range_max = in.get_f32();
        auto const states { in.get_u32() };
        p.light_states.reserve( states );
        for( std::uint32_t s { 0 }; s < states; ++s ) {
            p.light_states.push_back( in.get_f32() );
        }
        auto const colors { in.get_u32() };
        p.light_colors.reserve( colors );
        for( std::uint32_t c { 0 }; c < colors; ++c ) {
            p.light_colors.push_back( in.get_u32() );
        }
        protos.push_back( std::move( p ) );
    }
    return protos;
}

// --- INST ------------------------------------------------------------------

// Per-instance presence flags so common cases (unit scale, no override) stay
// compact instead of always paying for scale + override.
namespace inst_flag {
constexpr std::uint8_t has_scale { 1u << 0 };
constexpr std::uint8_t has_texture_override { 1u << 1 };
constexpr std::uint8_t has_node { 1u << 2 };
} // namespace inst_flag

inline void
write_instances( byte_writer &out, std::vector<model_instance> const &instances ) {
    out.put_u32( static_cast<std::uint32_t>( instances.size() ) );
    for( auto const &i : instances ) {
        std::uint8_t flags { 0 };
        bool const unit_scale { i.sx == 1.f && i.sy == 1.f && i.sz == 1.f };
        if( !unit_scale ) {
            flags |= inst_flag::has_scale;
        }
        if( i.texture_override != kNoString ) {
            flags |= inst_flag::has_texture_override;
        }
        if( i.has_node ) {
            flags |= inst_flag::has_node;
        }
        out.put_u8( flags );
        out.put_u32( i.proto );
        out.put_f64( i.x );
        out.put_f64( i.y );
        out.put_f64( i.z );
        out.put_f32( i.ax );
        out.put_f32( i.ay );
        out.put_f32( i.az );
        out.put_u8( i.cell_id );
        if( flags & inst_flag::has_scale ) {
            out.put_f32( i.sx );
            out.put_f32( i.sy );
            out.put_f32( i.sz );
        }
        if( flags & inst_flag::has_texture_override ) {
            out.put_u32( i.texture_override );
        }
        if( flags & inst_flag::has_node ) {
            write_node( out, i.node );
        }
    }
}

inline std::vector<model_instance>
read_instances( byte_reader &in ) {
    std::vector<model_instance> instances;
    auto const count { in.get_u32() };
    instances.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        model_instance m;
        auto const flags { in.get_u8() };
        m.proto = in.get_u32();
        m.x = in.get_f64();
        m.y = in.get_f64();
        m.z = in.get_f64();
        m.ax = in.get_f32();
        m.ay = in.get_f32();
        m.az = in.get_f32();
        m.cell_id = in.get_u8();
        if( flags & inst_flag::has_scale ) {
            m.sx = in.get_f32();
            m.sy = in.get_f32();
            m.sz = in.get_f32();
        }
        if( flags & inst_flag::has_texture_override ) {
            m.texture_override = in.get_u32();
        }
        if( flags & inst_flag::has_node ) {
            m.has_node = true;
            m.node = read_node( in );
        }
        instances.push_back( m );
    }
    return instances;
}

// --- MESH ------------------------------------------------------------------

inline void
write_terrain_meshes( byte_writer &out, std::vector<terrain_mesh> const &meshes ) {
    out.put_u32( static_cast<std::uint32_t>( meshes.size() ) );
    for( auto const &m : meshes ) {
        out.put_u32( m.material );
        out.put_u8( m.translucent ? 1u : 0u );
        out.put_f64( m.ox );
        out.put_f64( m.oy );
        out.put_f64( m.oz );
        out.put_u32( static_cast<std::uint32_t>( m.vertices.size() ) );
        for( auto const &v : m.vertices ) {
            out.put_f32( v.px );
            out.put_f32( v.py );
            out.put_f32( v.pz );
            out.put_f32( v.nx );
            out.put_f32( v.ny );
            out.put_f32( v.nz );
            out.put_f32( v.u );
            out.put_f32( v.v );
        }
    }
}

inline std::vector<terrain_mesh>
read_terrain_meshes( byte_reader &in ) {
    std::vector<terrain_mesh> meshes;
    auto const count { in.get_u32() };
    meshes.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        terrain_mesh m;
        m.material = in.get_u32();
        m.translucent = in.get_u8() != 0;
        m.ox = in.get_f64();
        m.oy = in.get_f64();
        m.oz = in.get_f64();
        auto const verts { in.get_u32() };
        m.vertices.reserve( verts );
        for( std::uint32_t v { 0 }; v < verts; ++v ) {
            mesh_vertex mv;
            mv.px = in.get_f32();
            mv.py = in.get_f32();
            mv.pz = in.get_f32();
            mv.nx = in.get_f32();
            mv.ny = in.get_f32();
            mv.nz = in.get_f32();
            mv.u = in.get_f32();
            mv.v = in.get_f32();
            m.vertices.push_back( mv );
        }
        meshes.push_back( std::move( m ) );
    }
    return meshes;
}

// --- SHPE : non-terrain shape nodes (triangles/strip/fan, baked world-space) -
// Lossless counterpart of MESH: keeps node metadata, lighting and translucency.
// Vertex positions are stored relative to origin (f64) as f32, matching MESH.

struct shape_record {
    node_record node;
    bool translucent { false };
    std::uint32_t material { kNoString };
    lighting_block lighting;
    double ox { 0.0 }, oy { 0.0 }, oz { 0.0 };
    std::vector<mesh_vertex> vertices;
};

inline void write_shape_record( byte_writer &out, shape_record const &s ) {
    write_node( out, s.node );
    out.put_u8( s.translucent ? 1u : 0u );
    out.put_u32( s.material );
    write_lighting( out, s.lighting );
    out.put_f64( s.ox );
    out.put_f64( s.oy );
    out.put_f64( s.oz );
    out.put_u32( static_cast<std::uint32_t>( s.vertices.size() ) );
    for( auto const &v : s.vertices ) {
        out.put_f32( v.px );
        out.put_f32( v.py );
        out.put_f32( v.pz );
        out.put_f32( v.nx );
        out.put_f32( v.ny );
        out.put_f32( v.nz );
        out.put_f32( v.u );
        out.put_f32( v.v );
    }
}

inline void write_shapes( byte_writer &out, std::vector<shape_record> const &shapes ) {
    out.put_u32( static_cast<std::uint32_t>( shapes.size() ) );
    for( auto const &s : shapes ) {
        write_shape_record( out, s );
    }
}

inline std::vector<shape_record> read_shapes( byte_reader &in ) {
    std::vector<shape_record> shapes;
    auto const count { in.get_u32() };
    shapes.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        shape_record s;
        s.node = read_node( in );
        s.translucent = in.get_u8() != 0;
        s.material = in.get_u32();
        s.lighting = read_lighting( in );
        s.ox = in.get_f64();
        s.oy = in.get_f64();
        s.oz = in.get_f64();
        auto const verts { in.get_u32() };
        s.vertices.reserve( verts );
        for( std::uint32_t v { 0 }; v < verts; ++v ) {
            mesh_vertex mv;
            mv.px = in.get_f32();
            mv.py = in.get_f32();
            mv.pz = in.get_f32();
            mv.nx = in.get_f32();
            mv.ny = in.get_f32();
            mv.nz = in.get_f32();
            mv.u = in.get_f32();
            mv.v = in.get_f32();
            s.vertices.push_back( mv );
        }
        shapes.push_back( std::move( s ) );
    }
    return shapes;
}

// --- LINE : line geometry nodes (only vertex positions are meaningful) -------

struct lines_record {
    node_record node;
    lighting_block lighting;
    float line_width { 1.f };
    double ox { 0.0 }, oy { 0.0 }, oz { 0.0 };
    std::vector<dvec3> vertices; // world-space positions
};

inline void write_lines( byte_writer &out, std::vector<lines_record> const &items ) {
    out.put_u32( static_cast<std::uint32_t>( items.size() ) );
    for( auto const &l : items ) {
        write_node( out, l.node );
        write_lighting( out, l.lighting );
        out.put_f32( l.line_width );
        out.put_f64( l.ox );
        out.put_f64( l.oy );
        out.put_f64( l.oz );
        out.put_u32( static_cast<std::uint32_t>( l.vertices.size() ) );
        for( auto const &v : l.vertices ) {
            put_dvec3( out, v.x, v.y, v.z );
        }
    }
}

inline std::vector<lines_record> read_lines( byte_reader &in ) {
    std::vector<lines_record> items;
    auto const count { in.get_u32() };
    items.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        lines_record l;
        l.node = read_node( in );
        l.lighting = read_lighting( in );
        l.line_width = in.get_f32();
        l.ox = in.get_f64();
        l.oy = in.get_f64();
        l.oz = in.get_f64();
        auto const verts { in.get_u32() };
        l.vertices.reserve( verts );
        for( std::uint32_t v { 0 }; v < verts; ++v ) {
            l.vertices.push_back( get_dvec3( in ) );
        }
        items.push_back( std::move( l ) );
    }
    return items;
}

} // namespace eu7v2
