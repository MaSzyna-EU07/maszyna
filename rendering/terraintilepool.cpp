/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "glad/glad.h"

module eu07.rendering.terraintilepool;
import eu07.gl.buffer;

namespace {

// an array of integer samples. nothing may filter them: they carry the cooker's values,
// no-data included
GLuint
create_array( GLenum const Internalformat, GLenum const Type, GLsizei const Side, GLsizei const Layers ) {

    GLuint texture { 0 };
    ::glGenTextures( 1, &texture );
    ::glBindTexture( GL_TEXTURE_2D_ARRAY, texture );
    ::glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    ::glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    ::glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    ::glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    // NOTE: glTexImage3D rather than glTexStorage3D, which is OpenGL 4.2; this renderer holds
    // a 3.3 context, where the entry point is not there at all
    ::glTexImage3D( GL_TEXTURE_2D_ARRAY, 0, Internalformat, Side, Side, Layers, 0, GL_RED_INTEGER, Type, nullptr );
    return texture;
}

// copies the first Layers layers of one integer array into another of the same side, on the
// gpu, through a read framebuffer. the framebuffer bindings are put back as they were found,
// so the renderer's own tracking of them stays true
void
copy_layers( GLuint const Source, GLuint const Target, GLsizei const Side, GLsizei const Layers ) {

    GLint readbinding { 0 };
    ::glGetIntegerv( GL_READ_FRAMEBUFFER_BINDING, &readbinding );
    GLuint framebuffer { 0 };
    ::glGenFramebuffers( 1, &framebuffer );
    ::glBindFramebuffer( GL_READ_FRAMEBUFFER, framebuffer );
    ::glBindTexture( GL_TEXTURE_2D_ARRAY, Target );
    for( GLsizei layer = 0; layer < Layers; ++layer ) {
        ::glFramebufferTextureLayer( GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, Source, 0, layer );
        ::glCopyTexSubImage3D( GL_TEXTURE_2D_ARRAY, 0, 0, 0, layer, 0, 0, Side, Side );
    }
    ::glBindFramebuffer( GL_READ_FRAMEBUFFER, static_cast<GLuint>( readbinding ) );
    ::glDeleteFramebuffers( 1, &framebuffer );
}

} // anonymous namespace

terrain_tile_pool::terrain_tile_pool( std::uint32_t const Side ) :
    m_side( Side ) {

    m_unpack.emplace();
}

terrain_tile_pool::~terrain_tile_pool() {

    if( m_heights != 0 ) { ::glDeleteTextures( 1, &m_heights ); }
    if( m_materials != 0 ) { ::glDeleteTextures( 1, &m_materials ); }
}

std::uint32_t
terrain_tile_pool::acquire() {

    if( true == m_free.empty() ) { grow(); }
    auto const slot { m_free.back() };
    m_free.pop_back();
    return slot;
}

void
terrain_tile_pool::release( std::uint32_t const Slot ) {

    m_free.push_back( Slot );
}

void
terrain_tile_pool::grow() {

    auto const capacity { m_capacity == 0 ? initialcapacity : m_capacity * 2 };
    auto const side { static_cast<GLsizei>( m_side ) };

    auto const heights { create_array( GL_R16UI, GL_UNSIGNED_SHORT, side, static_cast<GLsizei>( capacity ) ) };
    auto const materials { create_array( GL_R8UI, GL_UNSIGNED_BYTE, side, static_cast<GLsizei>( capacity ) ) };
    if( m_capacity > 0 ) {
        copy_layers( m_heights, heights, side, static_cast<GLsizei>( m_capacity ) );
        copy_layers( m_materials, materials, side, static_cast<GLsizei>( m_capacity ) );
        ::glDeleteTextures( 1, &m_heights );
        ::glDeleteTextures( 1, &m_materials );
    }
    m_heights = heights;
    m_materials = materials;

    // new slots are handed out lowest first
    for( auto slot { capacity }; slot > m_capacity; --slot ) {
        m_free.push_back( slot - 1 );
    }
    m_capacity = capacity;
}

void
terrain_tile_pool::upload( std::uint32_t const Slot, std::uint16_t const *Heights, std::uint8_t const *Materials ) {

    auto const samples { static_cast<std::size_t>( m_side ) * m_side };
    auto const heightbytes { samples * sizeof( std::uint16_t ) };

    // orphaned and refilled per tile: the driver can hand out fresh storage instead of waiting
    // for the previous upload to finish
    m_unpack->bind( gl::buffer::PIXEL_UNPACK_BUFFER );
    m_unpack->allocate( gl::buffer::PIXEL_UNPACK_BUFFER, static_cast<GLsizeiptr>( heightbytes + samples ), GL_STREAM_DRAW );
    m_unpack->upload( gl::buffer::PIXEL_UNPACK_BUFFER, Heights, 0, static_cast<GLsizeiptr>( heightbytes ) );
    m_unpack->upload( gl::buffer::PIXEL_UNPACK_BUFFER, Materials, static_cast<int>( heightbytes ), static_cast<GLsizeiptr>( samples ) );

    auto const side { static_cast<GLsizei>( m_side ) };
    ::glBindTexture( GL_TEXTURE_2D_ARRAY, m_heights );
    ::glTexSubImage3D(
        GL_TEXTURE_2D_ARRAY, 0, 0, 0, static_cast<GLint>( Slot ), side, side, 1,
        GL_RED_INTEGER, GL_UNSIGNED_SHORT, nullptr );
    ::glBindTexture( GL_TEXTURE_2D_ARRAY, m_materials );
    ::glTexSubImage3D(
        GL_TEXTURE_2D_ARRAY, 0, 0, 0, static_cast<GLint>( Slot ), side, side, 1,
        GL_RED_INTEGER, GL_UNSIGNED_BYTE, reinterpret_cast<void const *>( heightbytes ) );
    gl::buffer::unbind( gl::buffer::PIXEL_UNPACK_BUFFER );
}
