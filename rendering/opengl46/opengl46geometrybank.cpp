/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46geometrybank.h"

#include "utilities/Logs.h"
#include "utilities/Globals.h"

#include <map>

namespace gfx {

// a vertex and an index buffer shared by banks, with the vao they're drawn with. the banks take parts of them, given back when the
// bank lets its buffers go (a chunk added or resized, the bank released); the free parts are kept by their place, joined with the
// neighbouring ones when given back
struct opengl46_geometry_page {
    // the size of a page: 4M vertices (192 MiB) and 12M indices (48 MiB). a bank larger than that gets a page of its own size
    static constexpr std::size_t VERTICES { 4u << 20 };
    static constexpr std::size_t INDICES { 12u << 20 };

    gl::buffer vertices;
    gl::buffer indices;
    gl::vao vao;
    std::size_t vertexcapacity { 0 };
    std::size_t indexcapacity { 0 };
    std::map<std::size_t, std::size_t> freevertices; // place -> count
    std::map<std::size_t, std::size_t> freeindices;
    std::size_t users { 0 }; // banks with a part of the page
    std::uint32_t id { 0 };
};

namespace {

std::vector<std::unique_ptr<opengl46_geometry_page>> &geometry_pages() {
    static std::vector<std::unique_ptr<opengl46_geometry_page>> pages;
    return pages;
}

// takes Count elements from the free parts, the first one large enough. returns: false if there's none
bool take_part( std::map<std::size_t, std::size_t> &Free, std::size_t const Count, std::size_t &Offset ) {

    if( Count == 0 ) {
        Offset = 0;
        return true;
    }
    for( auto part { Free.begin() }; part != Free.end(); ++part ) {
        if( part->second < Count ) { continue; }
        Offset = part->first;
        auto const rest { part->second - Count };
        Free.erase( part );
        if( rest > 0 ) {
            Free.emplace( Offset + Count, rest );
        }
        return true;
    }
    return false;
}

// gives back Count elements from Offset, joined with the free parts next to them
void give_part( std::map<std::size_t, std::size_t> &Free, std::size_t const Offset, std::size_t const Count ) {

    if( Count == 0 ) { return; }
    auto part { Free.emplace( Offset, Count ).first };
    auto const next { std::next( part ) };
    if( ( next != Free.end() ) && ( part->first + part->second == next->first ) ) {
        part->second += next->second;
        Free.erase( next );
    }
    if( part != Free.begin() ) {
        auto const previous { std::prev( part ) };
        if( previous->first + previous->second == part->first ) {
            previous->second += part->second;
            Free.erase( part );
        }
    }
}

std::uint32_t next_page_id() {
    static std::uint32_t id { 0 };
    return ++id;
}

} // namespace

// opengl vao/vbo-based variant of the geometry bank

// create() subclass details
void
opengl46_vaogeometrybank::create_( gfx::geometry_handle const &Geometry ) {
    // adding a chunk means we'll be (re)building the buffer, which will fill the chunk records, amongst other things.
    // thus we don't need to initialize the values here
    m_chunkrecords.emplace_back( chunk_record() );
    // kiss the existing buffer goodbye, new overall data size means we'll be making a new one
    delete_buffer();
}

// replace() subclass details
void
opengl46_vaogeometrybank::replace_( gfx::geometry_handle const &Geometry ) {

    auto &chunkrecord = m_chunkrecords[ Geometry.chunk - 1 ];
    chunkrecord.is_good = false;
    // if the overall length of the chunk didn't change we can get away with reusing the old buffer...
    if( ( geometry_bank::chunk( Geometry ).vertices.size() != chunkrecord.vertex_count )
     || ( geometry_bank::chunk( Geometry ).userdata.empty() == chunkrecord.has_userdata ) ) {
        // ...but otherwise we'll need to allocate a new one
        // TBD: we could keep and reuse the old buffer also if the new chunk is smaller than the old one,
        // but it'd require some extra tracking and work to keep all chunks up to date; also wasting vram; may be not worth it?
        delete_buffer();
    }
}

void opengl46_vaogeometrybank::setup_buffer()
{
    if( has_buffer() ) { return; }
    // if there's no buffer, we'll have to make one
    // NOTE: this isn't exactly optimal in terms of ensuring the gfx card doesn't stall waiting for the data
    // may be better to initiate upload earlier (during update phase) and trust this effort won't go to waste
    if( true == m_chunks.empty() ) { return; }

    std::size_t
        vertexcount{ 0 },
        indexcount{ 0 };
    auto chunkiterator = m_chunks.cbegin();
	bool has_userdata{ false };
    for( auto &chunkrecord : m_chunkrecords ) {
        // fill records for all chunks, based on the chunk data
        chunkrecord.is_good = false; // if we're re-creating buffer, chunks might've been uploaded in the old one
        chunkrecord.vertex_offset = vertexcount;
        chunkrecord.vertex_count = chunkiterator->vertices.size();
        vertexcount += chunkrecord.vertex_count;
        chunkrecord.index_offset = indexcount;
        chunkrecord.index_count = chunkiterator->indices.size();
		chunkrecord.has_userdata = !chunkiterator->userdata.empty();
        indexcount += chunkrecord.index_count;
		has_userdata |= chunkrecord.has_userdata;
        ++chunkiterator;
    }
    // the odds for all created chunks to get replaced with empty ones are quite low, but the possibility does exist
    if( vertexcount == 0 ) { return; }

    if( Global.gfx_geometry_shared && ( false == has_userdata ) ) {
        // a part of the shared buffers: of a page with room for it, or of a new one
        auto &pages { geometry_pages() };
        opengl46_geometry_page *page { nullptr };
        std::size_t vertexoffset { 0 };
        std::size_t indexoffset { 0 };
        for( auto &candidate : pages ) {
            if( false == take_part( candidate->freevertices, vertexcount, vertexoffset ) ) { continue; }
            if( false == take_part( candidate->freeindices, indexcount, indexoffset ) ) {
                give_part( candidate->freevertices, vertexoffset, vertexcount );
                continue;
            }
            page = candidate.get();
            break;
        }
        if( page == nullptr ) {
            auto newpage { std::make_unique<opengl46_geometry_page>() };
            newpage->id = next_page_id();
            newpage->vertexcapacity = std::max( opengl46_geometry_page::VERTICES, vertexcount );
            newpage->indexcapacity = std::max( opengl46_geometry_page::INDICES, indexcount );
            newpage->vao.bind();
            newpage->indices.allocate( gl::buffer::ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>( newpage->indexcapacity * sizeof( gfx::basic_index ) ), GL_STATIC_DRAW );
            if( ::glGetError() == GL_OUT_OF_MEMORY ) {
                ErrorLog( "openGL error: out of memory; failed to create a shared geometry index buffer" );
                throw std::bad_alloc();
            }
            newpage->vao.setup_ebo( newpage->indices );
            newpage->vertices.allocate( gl::buffer::ARRAY_BUFFER, static_cast<GLsizeiptr>( newpage->vertexcapacity * sizeof( gfx::basic_vertex ) ), GL_STATIC_DRAW );
            if( ::glGetError() == GL_OUT_OF_MEMORY ) {
                ErrorLog( "openGL error: out of memory; failed to create a shared geometry buffer" );
                throw std::bad_alloc();
            }
            auto &vao { newpage->vao };
            auto &buffer { newpage->vertices };
            vao.setup_attrib( buffer, 0, 3, GL_FLOAT, sizeof( basic_vertex ), offsetof( basic_vertex, position ) );
            // NOTE: normal and color streams share the data
            vao.setup_attrib( buffer, 1, 3, GL_FLOAT, sizeof( basic_vertex ), offsetof( basic_vertex, normal ) );
            vao.setup_attrib( buffer, 2, 2, GL_FLOAT, sizeof( basic_vertex ), offsetof( basic_vertex, texture ) );
            vao.setup_attrib( buffer, 3, 4, GL_FLOAT, sizeof( basic_vertex ), offsetof( basic_vertex, tangent ) );
            newpage->freevertices.emplace( 0, newpage->vertexcapacity );
            newpage->freeindices.emplace( 0, newpage->indexcapacity );
            take_part( newpage->freevertices, vertexcount, vertexoffset );
            take_part( newpage->freeindices, indexcount, indexoffset );
            page = newpage.get();
            pages.emplace_back( std::move( newpage ) );
        }
        ++page->users;
        m_page = page;
        m_pagevertexoffset = vertexoffset;
        m_pagevertexcount = vertexcount;
        m_pageindexoffset = indexoffset;
        m_pageindexcount = indexcount;
        // the chunks lie in the shared buffers after one another, from the place of the bank
        for( auto &chunkrecord : m_chunkrecords ) {
            chunkrecord.vertex_offset += vertexoffset;
            chunkrecord.index_offset += indexoffset;
        }
        m_vertex_count = static_cast<int>( vertexcount );
        return;
    }

    if( !m_vao ) {
        m_vao.emplace();
    }
    m_vao->bind();
    // try to set up the buffers we need:
    // optional index buffer...
    if( indexcount > 0 ) {
        m_indexbuffer.emplace();
        m_indexbuffer->allocate( gl::buffer::ELEMENT_ARRAY_BUFFER, indexcount * sizeof( gfx::basic_index ), GL_STATIC_DRAW );
        if( ::glGetError() == GL_OUT_OF_MEMORY ) {
            ErrorLog( "openGL error: out of memory; failed to create a geometry index buffer" );
            throw std::bad_alloc();
        }
        m_vao->setup_ebo( *m_indexbuffer );
    }
    else {
        gl::buffer::unbind( gl::buffer::ELEMENT_ARRAY_BUFFER );
    }
    // ...and geometry buffer
	m_vertexbuffer.emplace();
    // NOTE: we're using static_draw since it's generally true for all we have implemented at the moment
    // TODO: allow to specify usage hint at the object creation, and pass it here
	const int vertex_size = has_userdata ? sizeof( gfx::basic_vertex ) + sizeof(gfx::vertex_userdata) : sizeof(gfx::basic_vertex);
    m_vertexbuffer->allocate( gl::buffer::ARRAY_BUFFER, static_cast<int>(vertexcount * vertex_size), GL_STATIC_DRAW );
    if( ::glGetError() == GL_OUT_OF_MEMORY ) {
        ErrorLog( "openGL error: out of memory; failed to create a geometry buffer" );
        throw std::bad_alloc();
    }
	m_vertex_count = static_cast<int>(vertexcount);

    setup_attrib();
	if(has_userdata)
		setup_userdata();
}

void
opengl46_vaogeometrybank::setup_attrib(size_t offset)
{
    m_vao->setup_attrib( *m_vertexbuffer, 0, 3, GL_FLOAT, sizeof( basic_vertex ), offsetof(basic_vertex, position) + offset * sizeof( basic_vertex ) );
    // NOTE: normal and color streams share the data
    m_vao->setup_attrib( *m_vertexbuffer, 1, 3, GL_FLOAT, sizeof( basic_vertex ), offsetof(basic_vertex, normal) + offset * sizeof( basic_vertex ) );
    m_vao->setup_attrib( *m_vertexbuffer, 2, 2, GL_FLOAT, sizeof( basic_vertex ), offsetof(basic_vertex, texture) + offset * sizeof( basic_vertex ) );
	m_vao->setup_attrib( *m_vertexbuffer, 3, 4, GL_FLOAT, sizeof( basic_vertex ), offsetof(basic_vertex, tangent) + offset * sizeof( basic_vertex ) );
}

// a line for the debug panel: the pages of the shared buffers and how much of them the banks take
std::string
opengl46_vaogeometrybank::shared_info() {

    auto const &pages { geometry_pages() };
    if( pages.empty() ) { return {}; }
    std::size_t users { 0 }, vertices { 0 }, vertexcapacity { 0 }, indices { 0 }, indexcapacity { 0 }, parts { 0 };
    for( auto const &page : pages ) {
        users += page->users;
        vertexcapacity += page->vertexcapacity;
        indexcapacity += page->indexcapacity;
        vertices += page->vertexcapacity;
        indices += page->indexcapacity;
        for( auto const &part : page->freevertices ) { vertices -= part.second; }
        for( auto const &part : page->freeindices ) { indices -= part.second; }
        parts += page->freevertices.size();
    }
    auto const mib = []( std::size_t const Bytes ) { return std::to_string( ( Bytes + ( 1u << 19 ) ) >> 20 ) + " MiB"; };
    return
        " shared geometry buffers: " + std::to_string( pages.size() ) + " pages, " + std::to_string( users ) + " banks, vertices "
        + mib( vertices * sizeof( basic_vertex ) ) + " of " + mib( vertexcapacity * sizeof( basic_vertex ) ) + ", indices "
        + mib( indices * sizeof( basic_index ) ) + " of " + mib( indexcapacity * sizeof( basic_index ) ) + ", " + std::to_string( parts ) + " free parts\n";
}

// binds the vao the bank is drawn with: the one of its page of the shared buffers, or its own
void
opengl46_vaogeometrybank::bind_vao() {

    if( m_page != nullptr ) { m_page->vao.bind(); }
    else                    { m_vao->bind(); }
}

// sends the data of a chunk to the buffers, unless they hold it already
void
opengl46_vaogeometrybank::upload_chunk( chunk_record &Record, geometry_chunk const &Chunk ) {

    if( Record.is_good ) { return; }
    if( m_page != nullptr ) {
        // straight to the shared buffers, without binding them: the element buffer is a part of whichever vao is bound
        if( Record.index_count > 0 ) {
            ::glNamedBufferSubData( m_page->indices, static_cast<GLintptr>( Record.index_offset * sizeof( gfx::basic_index ) ), static_cast<GLsizeiptr>( Record.index_count * sizeof( gfx::basic_index ) ), Chunk.indices.data() );
        }
        ::glNamedBufferSubData( m_page->vertices, static_cast<GLintptr>( Record.vertex_offset * sizeof( gfx::basic_vertex ) ), static_cast<GLsizeiptr>( Record.vertex_count * sizeof( gfx::basic_vertex ) ), Chunk.vertices.data() );
        Record.is_good = true;
        return;
    }
    m_vao->bind();
    // we may potentially need to upload new buffer data before we can draw it
    if( Record.index_count > 0 ) {
        m_indexbuffer->upload( gl::buffer::ELEMENT_ARRAY_BUFFER, Chunk.indices.data(), Record.index_offset * sizeof( gfx::basic_index ), Record.index_count * sizeof( gfx::basic_index ) );
    }
    m_vertexbuffer->upload( gl::buffer::ARRAY_BUFFER, Chunk.vertices.data(), Record.vertex_offset * sizeof( gfx::basic_vertex ), Record.vertex_count * sizeof( gfx::basic_vertex ) );
    if( Record.has_userdata ) {
        m_vertexbuffer->upload( gl::buffer::ARRAY_BUFFER, Chunk.userdata.data(),
            static_cast<int>( m_vertex_count * sizeof( gfx::basic_vertex ) + Record.vertex_offset * sizeof( gfx::vertex_userdata ) ),
            static_cast<int>( Record.vertex_count * sizeof( gfx::vertex_userdata ) ) );
    }
    Record.is_good = true;
}

// draw_group() subclass details
std::uint32_t
opengl46_vaogeometrybank::draw_group_() const {

    return ( m_page != nullptr ? m_page->id : 0 );
}

void opengl46_vaogeometrybank::setup_userdata(size_t offset) {
	const size_t offset_evaluated = m_vertex_count * sizeof(basic_vertex) + offset * sizeof( vertex_userdata );
	m_vao->setup_attrib( *m_vertexbuffer, 4, 4, GL_FLOAT, sizeof( vertex_userdata ), offsetof(vertex_userdata, data) + offset_evaluated );}

// draw() subclass details
// NOTE: units and stream parameters are unused, but they're part of (legacy) interface
// TBD: specialized bank/manager pair without the cruft?
std::size_t
opengl46_vaogeometrybank::draw_( gfx::geometry_handle const &Geometry, gfx::stream_units const &Units, unsigned int const Streams )
{
    setup_buffer();

    auto &chunkrecord = m_chunkrecords.at(Geometry.chunk - 1);
	// sanity check; shouldn't be needed but, eh
	if( chunkrecord.vertex_count == 0 )
		return 0;

    auto const &chunk = gfx::geometry_bank::chunk( Geometry );
    upload_chunk( chunkrecord, chunk );
    // render
    if( chunkrecord.index_count > 0 ) {
        bind_vao();
        ::glDrawRangeElementsBaseVertex(
            chunk.type,
            0, chunkrecord.vertex_count,
            chunkrecord.index_count, GL_UNSIGNED_INT, reinterpret_cast<void const *>( chunkrecord.index_offset * sizeof( gfx::basic_index ) ),
            chunkrecord.vertex_offset );
    }
    else {
        bind_vao();
        ::glDrawArrays( chunk.type, chunkrecord.vertex_offset, chunkrecord.vertex_count );
    }
/*
    m_vao->unbind();
*/
    auto const vertexcount { ( chunkrecord.index_count > 0 ? chunkrecord.index_count : chunkrecord.vertex_count ) };
    switch( chunk.type ) {
        case GL_TRIANGLES:      { return vertexcount / 3; }
        case GL_TRIANGLE_STRIP: { return vertexcount - 2; }
        default:                { return 0; }
    }
}

// draw_instanced() subclass details — single GL instanced draw for InstanceCount instances.
// The vertex shader reads per-instance modelview from instance_ssbo[gl_BaseInstance + gl_InstanceID].
std::size_t
opengl46_vaogeometrybank::draw_instanced_( gfx::geometry_handle const &Geometry, gfx::stream_units const &Units, std::size_t const InstanceCount, std::uint32_t const BaseInstance, unsigned int const Streams )
{
    setup_buffer();

    auto &chunkrecord = m_chunkrecords.at(Geometry.chunk - 1);
    if( chunkrecord.vertex_count == 0 ) { return 0; }

    auto const &chunk = gfx::geometry_bank::chunk( Geometry );
    upload_chunk( chunkrecord, chunk );

    GLsizei const inst_count = static_cast<GLsizei>(InstanceCount);

    if( chunkrecord.index_count > 0 ) {
        bind_vao();
        ::glDrawElementsInstancedBaseVertexBaseInstance(
            chunk.type,
            chunkrecord.index_count, GL_UNSIGNED_INT,
            reinterpret_cast<void const *>( chunkrecord.index_offset * sizeof( gfx::basic_index ) ),
            inst_count,
            chunkrecord.vertex_offset,
            BaseInstance );
    }
    else {
        bind_vao();
        ::glDrawArraysInstancedBaseInstance( chunk.type, chunkrecord.vertex_offset, chunkrecord.vertex_count, inst_count, BaseInstance );
    }

    auto const vertexcount { ( chunkrecord.index_count > 0 ? chunkrecord.index_count : chunkrecord.vertex_count ) };
    std::size_t prims = 0;
    switch( chunk.type ) {
        case GL_TRIANGLES:      prims = vertexcount / 3; break;
        case GL_TRIANGLE_STRIP: prims = ( vertexcount > 2 ? vertexcount - 2 : 0 ); break;
        default:                prims = 0; break;
    }
    return prims * InstanceCount;
}

// indirect_source() subclass details — the chunk is uploaded first if needed, as by a draw, so that an indirect draw
// command made from what this tells finds it in place
bool
opengl46_vaogeometrybank::indirect_source_( gfx::geometry_handle const &Geometry, gfx::indirect_source &Source )
{
    setup_buffer();

    auto &chunkrecord = m_chunkrecords.at(Geometry.chunk - 1);
    if( chunkrecord.vertex_count == 0 ) { return false; }

    auto const &chunk = gfx::geometry_bank::chunk( Geometry );
    upload_chunk( chunkrecord, chunk );

    Source.indexed = ( chunkrecord.index_count > 0 );
    Source.count = static_cast<std::uint32_t>( Source.indexed ? chunkrecord.index_count : chunkrecord.vertex_count );
    Source.first = static_cast<std::uint32_t>( Source.indexed ? chunkrecord.index_offset : chunkrecord.vertex_offset );
    Source.basevertex = static_cast<std::int32_t>( Source.indexed ? chunkrecord.vertex_offset : 0 );
    Source.type = chunk.type;
    return true;
}

// draw_indirect() subclass details — the command comes from the bound draw indirect buffer
void
opengl46_vaogeometrybank::draw_indirect_( gfx::geometry_handle const &Geometry, std::intptr_t const Offset, unsigned int const /*Streams*/ )
{
    setup_buffer();

    auto const &chunkrecord = m_chunkrecords.at(Geometry.chunk - 1);
    if( chunkrecord.vertex_count == 0 ) { return; }

    auto const &chunk = gfx::geometry_bank::chunk( Geometry );
    bind_vao();
    if( chunkrecord.index_count > 0 ) {
        ::glDrawElementsIndirect( chunk.type, GL_UNSIGNED_INT, reinterpret_cast<void const *>( Offset ) );
    }
    else {
        ::glDrawArraysIndirect( chunk.type, reinterpret_cast<void const *>( Offset ) );
    }
}

// draw_indirect_multi() subclass details — the commands come from the bound draw indirect buffer, the chunk tells the type
// and whether they're indexed
bool
opengl46_vaogeometrybank::draw_indirect_multi_( gfx::geometry_handle const &Geometry, std::intptr_t const Offset, std::uint32_t const Count, std::uint32_t const Stride )
{
    setup_buffer();

    // the counts of the chunks are in the commands, which may be for chunks other than this one
    if( ( false == has_buffer() ) || ( Count == 0 ) ) { return true; }

    auto const &chunkrecord = m_chunkrecords.at(Geometry.chunk - 1);
    auto const &chunk = gfx::geometry_bank::chunk( Geometry );
    bind_vao();
    if( chunkrecord.index_count > 0 ) {
        ::glMultiDrawElementsIndirect( chunk.type, GL_UNSIGNED_INT, reinterpret_cast<void const *>( Offset ), static_cast<GLsizei>( Count ), static_cast<GLsizei>( Stride ) );
    }
    else {
        ::glMultiDrawArraysIndirect( chunk.type, reinterpret_cast<void const *>( Offset ), static_cast<GLsizei>( Count ), static_cast<GLsizei>( Stride ) );
    }
    return true;
}

// draw_batch() subclass details — neighbouring non-indexed chunks of the same type are sent with a single GL call.
// The chunks are drawn in the supplied order, same as with a draw_() call for each of them.
std::size_t
opengl46_vaogeometrybank::draw_batch_( gfx::geometry_handle const *First, gfx::geometry_handle const *Last, gfx::stream_units const &Units, unsigned int const Streams )
{
    std::size_t count { 0 };

    setup_buffer();

    std::array<GLint, 64> offsets;
    std::array<GLsizei, 64> sizes;
    GLsizei pending { 0 };
    unsigned int pendingtype { 0 };

    auto const flush = [ & ]() {
        if( pending == 0 ) { return; }
        bind_vao();
        if( pending == 1 ) { ::glDrawArrays( pendingtype, offsets[ 0 ], sizes[ 0 ] ); }
        else               { ::glMultiDrawArrays( pendingtype, offsets.data(), sizes.data(), pending ); }
        pending = 0;
    };

    for( ; First != Last; ++First ) {

        auto const &chunkrecord = m_chunkrecords.at( First->chunk - 1 );
        if( ( false == chunkrecord.is_good )
         || ( chunkrecord.index_count > 0 ) ) {
            // chunks which wait for upload of their data and indexed chunks take the regular route
            flush();
            count += draw_( *First, Units, Streams );
            continue;
        }

        auto const type { gfx::geometry_bank::chunk( *First ).type };
        if( ( pending == static_cast<GLsizei>( offsets.size() ) )
         || ( ( pending > 0 ) && ( type != pendingtype ) ) ) {
            flush();
        }
        pendingtype = type;
        offsets[ pending ] = static_cast<GLint>( chunkrecord.vertex_offset );
        sizes[ pending ] = static_cast<GLsizei>( chunkrecord.vertex_count );
        ++pending;

        switch( type ) {
            case GL_TRIANGLES:      { count += chunkrecord.vertex_count / 3; break; }
            case GL_TRIANGLE_STRIP: { count += ( chunkrecord.vertex_count > 2 ? chunkrecord.vertex_count - 2 : 0 ); break; }
            default:                { break; }
        }
    }
    flush();

    return count;
}

// release () subclass details
void
opengl46_vaogeometrybank::release_() {

    delete_buffer();
}

void
opengl46_vaogeometrybank::delete_buffer() {

    if( m_page != nullptr ) {
        // the part of the shared buffers goes back; a page nobody has a part of goes, as the buffers of a bank did
        give_part( m_page->freevertices, m_pagevertexoffset, m_pagevertexcount );
        give_part( m_page->freeindices, m_pageindexoffset, m_pageindexcount );
        if( --m_page->users == 0 ) {
            auto &pages { geometry_pages() };
            auto *const emptied { m_page };
            pages.erase(
                std::remove_if( std::begin( pages ), std::end( pages ), [ emptied ]( std::unique_ptr<opengl46_geometry_page> const &Page ) { return Page.get() == emptied; } ),
                std::end( pages ) );
        }
        m_page = nullptr;
        m_pagevertexoffset = m_pagevertexcount = m_pageindexoffset = m_pageindexcount = 0;
    }
	m_vao.reset();
	m_vertexbuffer.reset();
    m_indexbuffer.reset();
    // NOTE: since we've deleted the buffer all chunks it held were rendered invalid as well
    // instead of clearing their state here we're delaying it until new buffer is created to avoid looping through chunk records twice
}

} // namespace gfx
