/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "glad/glad.h"
#include "global_include/interfaces/ITexture_macros.h"

module eu07.rendering.terrainground;
import eu07.gl.shader;
import eu07.gl.vao;
import eu07.glm;
import eu07.utilities.logs;
import eu07.rendering.renderer;
import eu07.model.material;
import eu07.model.texture;
import eu07.global_include.interfaces.itexture;

namespace {

// a placeholder colour per material, spread around the hue circle so neighbouring materials
// without a texture still read as different ground
glm::vec3
placeholder( std::size_t const Index ) {

    auto const hue { static_cast<float>( Index ) * 0.61803399f };
    auto const phase { ( hue - std::floor( hue ) ) * 6.f };
    auto const rising { phase - std::floor( phase ) };
    switch( static_cast<int>( phase ) ) {
        case 0: return { 0.55f, 0.45f + 0.2f * rising, 0.3f };
        case 1: return { 0.55f - 0.2f * rising, 0.6f, 0.3f };
        case 2: return { 0.35f, 0.6f, 0.3f + 0.2f * rising };
        case 3: return { 0.35f, 0.6f - 0.15f * rising, 0.5f };
        case 4: return { 0.35f + 0.2f * rising, 0.45f, 0.5f };
        default: return { 0.55f, 0.45f, 0.5f - 0.15f * rising };
    }
}

// gl state the copy pass changes, captured so it can be put back exactly
struct saved_state {
    GLint drawframebuffer { 0 };
    GLint viewport[ 4 ] {};
    GLboolean depthtest { GL_FALSE };
    GLboolean blend { GL_FALSE };
    GLboolean cullface { GL_FALSE };
    GLboolean scissor { GL_FALSE };
    GLboolean srgb { GL_FALSE };

    saved_state() {
        ::glGetIntegerv( GL_DRAW_FRAMEBUFFER_BINDING, &drawframebuffer );
        ::glGetIntegerv( GL_VIEWPORT, viewport );
        depthtest = ::glIsEnabled( GL_DEPTH_TEST );
        blend = ::glIsEnabled( GL_BLEND );
        cullface = ::glIsEnabled( GL_CULL_FACE );
        scissor = ::glIsEnabled( GL_SCISSOR_TEST );
        srgb = ::glIsEnabled( GL_FRAMEBUFFER_SRGB );
    }
    ~saved_state() {
        auto const set = []( GLenum const Capability, GLboolean const Enabled ) {
            if( Enabled ) { ::glEnable( Capability ); } else { ::glDisable( Capability ); } };
        ::glBindFramebuffer( GL_DRAW_FRAMEBUFFER, static_cast<GLuint>( drawframebuffer ) );
        ::glViewport( viewport[ 0 ], viewport[ 1 ], viewport[ 2 ], viewport[ 3 ] );
        set( GL_DEPTH_TEST, depthtest );
        set( GL_BLEND, blend );
        set( GL_CULL_FACE, cullface );
        set( GL_SCISSOR_TEST, scissor );
        set( GL_FRAMEBUFFER_SRGB, srgb );
    }
};

} // anonymous namespace

terrain_ground::~terrain_ground() {

    destroy();
}

bool
terrain_ground::create( std::vector<std::string> const &Names, std::vector<float> const &Scales ) {

    destroy();

    try {
        gl::shader vertex( "terrainground.vert" );
        gl::shader fragment( "terrainground.frag" );
        m_copy.emplace( std::vector<std::reference_wrapper<gl::shader const>>( { vertex, fragment } ) );
    }
    catch( gl::shader_exception const &error ) {
        ErrorLog( std::string( "Terrain: ground copy shader failed, " ) + error.what() );
        return false;
    }
    m_vao.emplace();

    std::size_t resolved { 0 };
    std::string unresolved;
    m_materials.resize( std::max<std::size_t>( 1, Names.size() ) );
    for( std::size_t index = 0; index < Names.size(); ++index ) {
        auto &entry { m_materials[ index ] };
        entry.fallback = placeholder( index );
        entry.scale = ( index < Scales.size() && Scales[ index ] >= 0.01f ) ? Scales[ index ] : 4.f;
        auto const handle { GfxRenderer->Fetch_Material( Names[ index ] ) };
        if( handle != null_handle ) {
            entry.texture = GfxRenderer->Material( handle )->GetTexture( 0 );
        }
        if( entry.texture != null_handle ) {
            ++resolved;
            ++m_pending;
        }
        else {
            // drawn in its placeholder colour instead, so a missing texture shows up as an
            // obvious flat patch rather than as black ground
            unresolved += ( unresolved.empty() ? "" : ", " ) + Names[ index ];
        }
    }
    WriteLog(
        "Terrain: " + std::to_string( resolved ) + " of " + std::to_string( Names.size() )
        + " materials resolved to textures" );
    if( false == unresolved.empty() ) {
        WriteLog( "Terrain: no texture for " + unresolved );
    }

    // a layer with its mip chain takes four thirds of its base level
    auto const layers { m_materials.size() };
    while( ( m_side > 64 ) && ( m_side * m_side * 4u * 4u / 3u * layers > budgetbytes ) ) { m_side /= 2; }
    auto const side { static_cast<GLsizei>( m_side ) };
    auto const levels { static_cast<GLint>( std::log2( m_side ) ) + 1 };

    ::glGenTextures( 1, &m_array );
    ::glBindTexture( GL_TEXTURE_2D_ARRAY, m_array );
    for( GLint level = 0; level < levels; ++level ) {
        ::glTexImage3D(
            GL_TEXTURE_2D_ARRAY, level, GL_SRGB8_ALPHA8, std::max( 1, side >> level ), std::max( 1, side >> level ),
            static_cast<GLsizei>( layers ), 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr );
    }
    ::glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
    ::glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    ::glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_REPEAT );
    ::glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_REPEAT );
    if( GLAD_GL_ARB_texture_filter_anisotropic ) {
        ::glTexParameterf( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAX_ANISOTROPY, 16.f );
    }

    ::glGenTextures( 1, &m_table );
    ::glBindTexture( GL_TEXTURE_2D, m_table );
    ::glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    ::glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    upload_table();

    ::glGenFramebuffers( 1, &m_framebuffer );
    opengl_texture::reset_unit_cache();
    return true;
}

void
terrain_ground::destroy() {

    if( m_array != 0 ) { ::glDeleteTextures( 1, &m_array ); m_array = 0; }
    if( m_table != 0 ) { ::glDeleteTextures( 1, &m_table ); m_table = 0; }
    if( m_framebuffer != 0 ) { ::glDeleteFramebuffers( 1, &m_framebuffer ); m_framebuffer = 0; }
    m_copy.reset();
    m_vao.reset();
    m_materials.clear();
    m_pending = 0;
    m_side = layerside;
}

// rgb is the placeholder colour, alpha the metres per texture repeat - negative while the
// material has no layer to sample yet
void
terrain_ground::upload_table() {

    std::vector<float> texels;
    texels.reserve( m_materials.size() * 4 );
    for( auto const &entry : m_materials ) {
        texels.insert( texels.end(), { entry.fallback.r, entry.fallback.g, entry.fallback.b, entry.copied ? entry.scale : -entry.scale } );
    }
    ::glBindTexture( GL_TEXTURE_2D, m_table );
    ::glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA32F, static_cast<GLsizei>( m_materials.size() ), 1, 0, GL_RGBA, GL_FLOAT, texels.data() );
}

void
terrain_ground::update() {

    if( ( m_pending == 0 ) || ( m_array == 0 ) ) { return; }

    std::size_t copied { 0 };
    {
        saved_state const saved;
        ::glDisable( GL_DEPTH_TEST );
        ::glDisable( GL_BLEND );
        ::glDisable( GL_CULL_FACE );
        ::glDisable( GL_SCISSOR_TEST );
        // the sources are sRGB and sample as linear; writing through an sRGB target encodes
        // them back, so the layer keeps the precision the source had
        ::glEnable( GL_FRAMEBUFFER_SRGB );
        ::glBindFramebuffer( GL_DRAW_FRAMEBUFFER, m_framebuffer );
        ::glViewport( 0, 0, static_cast<GLsizei>( m_side ), static_cast<GLsizei>( m_side ) );
        m_copy->bind();
        m_vao->bind();

        for( std::size_t index = 0; index < m_materials.size(); ++index ) {
            auto &entry { m_materials[ index ] };
            if( ( true == entry.copied ) || ( entry.texture == null_handle ) ) { continue; }
            // binding asks the engine for the texture; it comes back unready while the data is
            // still loading, and the copy waits for a later frame
            GfxRenderer->Bind_Texture( 0, entry.texture );
            if( false == GfxRenderer->Texture( entry.texture ).get_is_ready() ) { continue; }

            ::glFramebufferTextureLayer( GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, m_array, 0, static_cast<GLint>( index ) );
            ::glDrawArrays( GL_TRIANGLES, 0, 3 );
            entry.copied = true;
            --m_pending;
            ++copied;
            // rare, and it changes gl state for a moment: worth a line, so that anything odd seen on
            // screen can be told apart from what the streamer is doing
            WriteLog( "Terrain: ground texture " + std::to_string( index ) + " copied into the array, "
                + std::to_string( m_pending ) + " still waiting" );
        }

        gl::vao::unbind();
        gl::program::bind( 0 );
    }

    if( copied > 0 ) {
        ::glBindTexture( GL_TEXTURE_2D_ARRAY, m_array );
        ::glGenerateMipmap( GL_TEXTURE_2D_ARRAY );
        ::glBindTexture( GL_TEXTURE_2D_ARRAY, 0 );
        upload_table();
    }
    // last of all, and after the mipmaps rather than before them: the engine remembers what it last
    // put on each texture unit and skips a bind it believes is already in place, so leaving a bind of
    // ours behind its back costs it a frame drawn with the wrong texture
    opengl_texture::reset_unit_cache();
}
