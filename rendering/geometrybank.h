/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "model/ResourceManager.h"

struct world_vertex;

namespace gfx {

struct basic_vertex {

    glm::vec3 position; // 3d space
    glm::vec3 normal; // 3d space
    glm::vec2 texture; // uv space
    glm::vec4 tangent; // xyz - tangent, w - handedness

    basic_vertex() = default;
    basic_vertex( glm::vec3 Position,  glm::vec3 Normal,  glm::vec2 Texture ) :
                  position( Position ),  normal( Normal ), texture( Texture )
    {}
	static basic_vertex convert(world_vertex const &world, glm::dvec3 const &origin);
	world_vertex to_world(glm::dvec3 const &origin = glm::dvec3(0.)) const;
	void serialize( std::ostream&, bool Tangent = false ) const;
    void deserialize( std::istream&, bool Tangent = false );
    void serialize_packed( std::ostream&, bool Tangent = false ) const;
    void deserialize_packed( std::istream&, bool Tangent = false );
};

struct vertex_userdata{
	glm::vec4 data; // user data (for color or additional uv channels, not subject to post-processing)
	void serialize( std::ostream& ) const;
	void deserialize( std::istream& );
	void serialize_packed( std::ostream& ) const;
	void deserialize_packed( std::istream& );
};

// data streams carried in a vertex
enum stream {
    none     = 0x0,
    position = 0x1,
    normal   = 0x2,
    color    = 0x4, // currently normal and colour streams are stored in the same slot, and mutually exclusive
    texture  = 0x8
};

constexpr unsigned int basic_streams { position | normal | texture };
constexpr unsigned int color_streams { position | color | texture };

struct stream_units {

    std::vector<GLint> texture { GL_TEXTURE0 }; // unit associated with main texture data stream. TODO: allow multiple units per stream
};

using basic_index = std::uint32_t;

using vertex_array = std::vector<basic_vertex>;
using userdata_array = std::vector<vertex_userdata>;
using index_array = std::vector<basic_index>;

void calculate_tangents( vertex_array &vertices, index_array const &indices, int type );
void calculate_indices( index_array &Indices, vertex_array &Vertices, userdata_array &Userdata, float tolerancescale = 1.0f );

// generic geometry bank class, allows storage, update and drawing of geometry chunks

struct geometry_handle {
// constructors
    geometry_handle() :
        bank( 0 ), chunk( 0 )
    {}
    geometry_handle(const std::uint32_t Bank, const std::uint32_t Chunk ) :
                             bank( Bank ),      chunk( Chunk )
    {}
// methods

	operator std::uint64_t() const {
/*
        return bank << 14 | chunk; }
*/
        return ( std::uint64_t { bank } << 32 | chunk );
    }

// members
/*
    std::uint32_t
        bank  : 18, // 250k banks
        chunk : 14; // 16k chunks per bank
*/
    std::uint32_t bank;
    std::uint32_t chunk;
};

class geometry_bank {

public:
// types:

// constructors:

// destructor:
    virtual
        ~geometry_bank() {}

// methods:
    // creates a new geometry chunk of specified type from supplied data. returns: handle to the chunk or NULL
    auto create( vertex_array &Vertices, userdata_array& Userdata, unsigned int Type ) -> geometry_handle;
    // creates a new indexed geometry chunk of specified type from supplied data. returns: handle to the chunk or NULL
    auto create( index_array &Indices, vertex_array &Vertices, userdata_array& Userdata, unsigned int Type ) -> geometry_handle;
    // replaces vertex data of specified chunk with the supplied data, starting from specified offset
    auto replace( vertex_array &Vertices, userdata_array& Userdata, geometry_handle const &Geometry, std::size_t Offset = 0 ) -> bool;
    // adds supplied vertex data at the end of specified chunk
    auto append( vertex_array &Vertices, userdata_array& Userdata, geometry_handle const &Geometry ) -> bool;
    // draws geometry stored in specified chunk
    auto draw( geometry_handle const &Geometry, stream_units const &Units, unsigned int Streams = basic_streams ) -> std::size_t;
    // draws geometry stored in specified chunk N times via glDrawElementsInstanced*
    auto draw_instanced( geometry_handle const &Geometry, stream_units const &Units, std::size_t InstanceCount, unsigned int Streams = basic_streams ) -> std::size_t;
    // draws geometry stored in supplied list of chunks
    template <typename Iterator_>
    auto draw( Iterator_ First, Iterator_ Last, stream_units const &Units, unsigned int const Streams = basic_streams ) ->std::size_t {
            std::size_t count { 0 };
            while( First != Last ) {
                count += draw( *First, Units, Streams ); ++First; }
            return count; }
    // frees subclass-specific resources associated with the bank, typically called when the bank wasn't in use for a period of time
    void release();
    // provides direct access to index data of specfied chunk
    auto indices( geometry_handle const &Geometry ) const -> index_array const &;
	// provides direct access to vertex data of specfied chunk
	auto vertices( geometry_handle const &Geometry ) const -> vertex_array const &;
	// provides direct access to vertex user data of specfied chunk
	auto userdata( geometry_handle const &Geometry ) const -> userdata_array const &;

protected:
// types:
    struct geometry_chunk {
        unsigned int type; // kind of geometry used by the chunk
        vertex_array vertices; // geometry data
        index_array indices; // index data
		userdata_array userdata;
        // NOTE: constructor doesn't copy provided geometry data, but moves it
        geometry_chunk( vertex_array &Vertices, userdata_array& Userdata, const unsigned int Type ) :
                                                            type( Type )
        {
            vertices.swap( Vertices );
			userdata.swap( Userdata );
        }
        // NOTE: constructor doesn't copy provided geometry data, but moves it
        geometry_chunk( index_array &Indices, vertex_array &Vertices, userdata_array& Userdata, const unsigned int Type ) :
                                                                                       type( Type )
        {
            vertices.swap( Vertices );
			userdata.swap( Userdata );
            indices.swap( Indices );
        }
    };

    using geometrychunk_sequence = std::vector<geometry_chunk>;

// methods
	auto chunk( geometry_handle const Geometry ) -> geometry_chunk & {
            return m_chunks[ Geometry.chunk - 1 ]; }
	auto chunk( geometry_handle const Geometry ) const -> geometry_chunk const & {
            return m_chunks[ Geometry.chunk - 1 ]; }

// members:
    geometrychunk_sequence m_chunks;

private:
// methods:
    // create() subclass details
    virtual void create_( geometry_handle const &Geometry ) = 0;
    // replace() subclass details
    virtual void replace_( geometry_handle const &Geometry ) = 0;
    // draw() subclass details
    virtual auto draw_( geometry_handle const &Geometry, stream_units const &Units, unsigned int Streams ) -> std::size_t = 0;
    // draw_instanced() subclass details. Default implementation falls back to N regular draws.
    virtual auto draw_instanced_( geometry_handle const &Geometry, stream_units const &Units, std::size_t const InstanceCount, unsigned int const Streams ) -> std::size_t {
        std::size_t count { 0 };
        for( std::size_t i = 0; i < InstanceCount; ++i ) { count += draw_( Geometry, Units, Streams ); }
        return count; }
    // resource release subclass details
    virtual void release_() = 0;
};

// geometry bank manager, holds collection of geometry banks

using geometrybank_handle = geometry_handle;

class geometrybank_manager {

public:
// constructors
    geometrybank_manager() = default;
// methods:
    // performs a resource sweep
    void update();
    // registers a new geometry bank. returns: handle to the bank
    auto register_bank(std::unique_ptr<geometry_bank> bank) -> geometrybank_handle;
    // creates a new geometry chunk of specified type from supplied data, in specified bank. returns: handle to the chunk or NULL
    auto create_chunk( vertex_array &Vertices, userdata_array &Userdata, geometrybank_handle const &Geometry, int Type ) -> geometry_handle;
    // creates a new indexed geometry chunk of specified type from supplied data, in specified bank. returns: handle to the chunk or NULL
    auto create_chunk( index_array &Indices, vertex_array &Vertices, userdata_array &Userdata, geometrybank_handle const &Geometry, unsigned int Type ) -> geometry_handle;
    // replaces data of specified chunk with the supplied vertex data, starting from specified offset
    auto replace( vertex_array &Vertices, userdata_array &Userdata, geometry_handle const &Geometry, std::size_t Offset = 0 ) -> bool;
    // adds supplied vertex data at the end of specified chunk
    auto append( vertex_array &Vertices, userdata_array &Userdata, geometry_handle const &Geometry ) -> bool;
    // draws geometry stored in specified chunk
    void draw( geometry_handle const &Geometry, unsigned int Streams = basic_streams );
    // draws geometry stored in specified chunk InstanceCount times via GPU instancing.
    // The shader reads per-instance modelview matrices from instance_ubo[gl_InstanceID].
    void draw_instanced( geometry_handle const &Geometry, std::size_t InstanceCount, unsigned int Streams = basic_streams );
    template <typename Iterator_>
    void draw( Iterator_ First, Iterator_ Last, unsigned int const Streams = basic_streams ) {
            while( First != Last ) { 
                draw( *First, Streams );
                ++First; } }
    // provides direct access to index data of specfied chunk
    auto indices( geometry_handle const &Geometry ) const -> index_array const &;
	// provides direct access to vertex data of specfied chunk
	auto vertices( geometry_handle const &Geometry ) const -> vertex_array const &;
	// provides direct access to vertex data of specfied chunk
	auto userdata( geometry_handle const &Geometry ) const -> userdata_array const &;
    // sets target texture unit for the texture data stream
    auto units() -> stream_units & { return m_units; }
    // provides access to primitives count
    auto primitives_count() const -> std::size_t const & { return m_primitivecount; }
    auto primitives_count() -> std::size_t & { return m_primitivecount; }

private:
// types:
    using geometrybanktimepoint_pair = std::pair< std::shared_ptr<geometry_bank>, resource_timestamp >;
    using geometrybanktimepointpair_sequence = std::deque< geometrybanktimepoint_pair >;

    // members:
    geometrybanktimepointpair_sequence m_geometrybanks;
    garbage_collector<geometrybanktimepointpair_sequence> m_garbagecollector { m_geometrybanks, 60, 120, "geometry buffer" };
    stream_units m_units;

// methods
	auto valid( geometry_handle const &Geometry ) const -> bool {
            return ( ( Geometry.bank != 0 )
                  && ( Geometry.bank <= m_geometrybanks.size() ) ); }
	auto bank( geometry_handle const Geometry ) -> geometrybanktimepointpair_sequence::value_type & {
            return m_geometrybanks[ Geometry.bank - 1 ]; }
	auto bank( geometry_handle const Geometry ) const -> geometrybanktimepointpair_sequence::value_type const & {
            return m_geometrybanks[ Geometry.bank - 1 ]; }

// members:
    std::size_t m_primitivecount { 0 }; // number of drawn primitives
};

} // namespace gfx
