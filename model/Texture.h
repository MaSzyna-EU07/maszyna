/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <istream>
#include <chrono>
#include <limits>
#include "winheaders.h"
#include "utilities/memorystats.h"
#include <string>
#include <functional>
#include "model/ResourceManager.h"
#include "gl/ubo.h"
#include "interfaces/ITexture.h"

struct texture_request; // texture data read by a worker thread, with gfx.textures.streaming

// a texture as its file gives it, whichever api draws it: read and decoded (dds, tga, png, bmp, jpg, ktx, tex), with the mipmaps
// the file has, kept within maxtexturesize; with gfx.textures.streaming the data of a dds file is read by worker threads. the
// data is laid out as opengl takes it (the bottom row first) and its formats are the values of opengl (GL_RGBA,
// GL_COMPRESSED_RGBA_S3TC_DXT1_EXT ...), which the renderers take as they are. a renderer makes its own object of the texture
// from the data (create())
struct texture_source : public ITexture {
    static DDSURFACEDESC2 deserialize_ddsd(std::istream&);
    static DDCOLORKEY deserialize_ddck(std::istream&);
    static DDPIXELFORMAT deserialize_ddpf(std::istream&);
    static DDSCAPS2 deserialize_ddscaps(std::istream&);

// constructors
    texture_source() = default;
// methods
    void
        load();
    void
        make_stub() override;
    virtual void
        set_components_hint( GLint hint ) override;
    virtual int
        get_width() const override {
            return data_width; }
    virtual
    int
        get_height() const override {
            return data_height; }
    virtual
    bool
        is_stub() const override {
            return is_texstub; }
    virtual
    bool
        get_has_alpha() const override {
            return has_alpha; }
    virtual
    bool
        get_is_ready() const override {
            return is_ready; }
    virtual
    std::string_view
        get_traits() const override {
            return traits; }
    virtual
    std::string_view
        get_name() const override {
            return name; }
	virtual
	    std::string_view
	    get_type() const override {
		return type; }

    virtual void make_from_memory(size_t width, size_t height, const uint8_t *data) override;

    // whether the texture has its object made by the renderer (it may be the failed one, with the id 0)
    virtual bool
        resident() const = 0;
    // the object of the renderer goes, with the bank of the textures
    virtual void
        destroy() = 0;
    // another thread of the renderer works on the texture (makes its object from the data, which it reads too): what it holds isn't for
    // the other threads to read until it's done
    virtual bool
        busy() const {
            return false; }

// members
    bool has_alpha{ false }; // indicates the texture has alpha channel
    bool is_ready{ false }; // indicates the texture was processed and is ready for use
    std::string traits; // requested texture attributes: wrapping modes etc
    std::string name; // name of the texture source file
    std::string type; // type of the texture source file
    std::size_t size{ 0 }; // size of the texture data, in kb
    GLint components_hint = 0; // components that material wants

public:
// methods
    void make_request();
    void load_PNG();
    void load_DDS();
    void load_KTX();
    void load_TEX();
    void load_STBI();
    void load_TGA();
    void downsize( GLuint const Format );
    void flip_vertical();

    // takes over the data read by a worker thread, waiting for it if needed
    void complete_data();
    // has a worker thread read the data of the texture, if it's waiting for that (gfx.textures.streaming). Origin: the camera,
    // for data read ahead of it
    void prefetch( glm::dvec3 const *Origin = nullptr );
    // the data is read from the file when it's needed and not kept after the upload (gfx.textures.streaming)
    bool streamed() const;

// members
    std::shared_ptr<texture_request> request; // data being read by a worker thread
    bool reload_on_use = false; // the data was dropped along with the gl texture of a far texture, it's read again when the texture is needed
    bool is_static = false; // is excluded from garbage collection
    bool is_rendertarget = false; // is used as postfx rendertarget, without loaded data
    bool is_texstub = false; // for make_from_memory internal_src: functionality
    std::vector<unsigned char> data; // texture data (stored GL-style, bottom-left origin)
    resource_state data_state{ resource_state::none }; // current state of texture data
    unsigned int data_width{ 0 },
        data_height{ 0 },
        data_mapcount{ 0 };
    GLint data_format{ 0 },
        data_components{ 0 };
    GLint data_type = GL_UNSIGNED_BYTE;
/*
    std::atomic<bool> is_loaded{ false }; // indicates the texture data was loaded and can be processed
    std::atomic<bool> is_good{ false }; // indicates the texture data was retrieved without errors
*/
    // the compressed formats the files give, with the bytes of their blocks of 4x4 texels
    static std::unordered_map<GLint, int> precompressed_formats;

protected:
    // the data of the texture couldn't be read (load(), complete_data())
    virtual void
        on_failed() {}
};

// a texture of the opengl renderer
struct opengl_texture : public texture_source {
// constructors
    opengl_texture() = default;
// methods
    bool
        bind( size_t unit );
    static void
        unbind( size_t unit );
    virtual bool
        create( bool const Static = false ) override;
    // releases resources allocated on the opengl end, storing local copy if requested
    void
        release() override;
    void
        alloc_rendertarget( GLint format, GLint components, int width, int height, int layers = 1, int samples = 1, GLint wrap = GL_CLAMP_TO_EDGE );
    static void
        reset_unit_cache();
    virtual
    size_t
        get_id() const override {
            return id; }
	virtual void update_from_memory(size_t width, size_t height, const uint8_t *data) override;
    bool
        resident() const override {
            return id != invalid_id; }
    void
        destroy() override;

// members
    static constexpr GLuint invalid_id { std::numeric_limits<GLuint>::max() }; // id value of texture without GL resource
    GLuint id{ invalid_id }; // associated GL resource

    GLenum target = GL_TEXTURE_2D;
    static std::array<GLuint, gl::MAX_TEXTURES + gl::HELPER_TEXTURES> units;
    static GLint m_activeunit;

public:
// methods
    void set_filtering() const;

// members
    int samples = 1;
    int layers = 1;
    GLint wrap_mode_s = GL_REPEAT;
    GLint wrap_mode_t = GL_REPEAT;

    static std::unordered_map<GLint, GLint> drivercompressed_formats;
    static std::unordered_map<GLint, std::unordered_map<GLint, GLint>> mapping;

protected:
    void
        on_failed() override;
};

// the textures of a renderer: made from their files the first time they are asked for, given back by their handles, released
// when nothing used them for a while (gfx.resource.sweep) or when only vehicles far away use them (gfx.textures.releasedistance).
// the objects of the textures come from the renderer (the function given to the constructor)
class texture_bank {

public:
    explicit texture_bank( std::function<texture_source *()> Make );
    virtual ~texture_bank();

    // creates texture object out of data stored in specified file
    texture_handle
        create( std::string Filename, bool const Loadnow = true, GLint Formathint = GL_SRGB_ALPHA );
    texture_source &
        mark_as_used( texture_handle const Texture );
    // provides direct access to specified texture object
    texture_source &
        texture( texture_handle const Texture ) const { return *m_textures[Texture].first; }
    // performs a resource sweep
    void
        update();
    // release of gl textures used only far away (gfx.textures.releasedistance). a scan is made of
    // begin_release_scan(), mark_as_needed() for textures of each vehicle and release_unneeded()
    void
        begin_release_scan();
    // marks texture of a vehicle as needed if the vehicle is near, otherwise only as one used by vehicles
    void
        mark_as_needed( texture_handle const Texture, bool const Near );
    // releases gl textures of vehicles not needed nor drawn for a while
    void
        release_unneeded();
    // debug performance string
    std::string
        info() const;
    // what the textures keep in memory, for the debug panel: copies of the data kept in the process (count, bytes), and the
    // textures made on the gpu with the size of their data (count, bytes)
    // with gfx.textures.streaming also the data read for textures not made yet (count, bytes), the textures which wait for a
    // worker to read it, and the ones which wait to be asked for it
    struct memory_data {
        memory_count cpu;
        memory_count gpu;
        memory_count readahead;
        std::uint64_t reading { 0 };
        std::uint64_t waiting { 0 };
    };
    memory_data
        memory() const;
    // has the data of a texture read, if it's waiting for that (gfx.textures.streaming)
    void
        prefetch( texture_handle const Texture, glm::dvec3 const &Origin );
    // gives back the data read for textures which nothing drew within the specified time since it was asked for, or read ahead of
    // the camera which went farther than the specified distance from there since
    std::size_t
        drop_read_ahead( std::chrono::steady_clock::duration const Age, glm::dvec3 const &Camera, double const Distance );

protected:
    // objects of textures were released, the renderer can't trust what it knew of their names
    virtual void
        released() {}

private:
// types:
    typedef std::pair<
        texture_source *,
        resource_timestamp > texturetimepoint_pair;

    typedef std::vector< texturetimepoint_pair > texturetimepointpair_sequence;

    typedef std::unordered_map<std::string, std::size_t> index_map;

// methods:
    // checks whether specified texture is in the texture bank. returns texture id, or npos.
    texture_handle
        find_in_databank( std::string const &Texturename ) const;

  public:
    // checks whether specified file exists. returns name of the located file, or empty string.
    static std::pair<std::string, std::string>
        find_on_disk( std::string const &Texturename );

  private:
    void
        delete_textures();

// members:
    std::function<texture_source *()> m_make;
    texture_handle const npos { 0 }; // should be -1, but the rest of the code uses -1 for something else
    texturetimepointpair_sequence m_textures;
    index_map m_texturemappings;
    std::vector<resource_timestamp> m_neededtimes; // last time a vehicle near enough needed the texture, blank for textures not used by vehicles
    resource_timestamp m_releasescantime; // time of the current release scan
    garbage_collector<texturetimepointpair_sequence> m_garbagecollector { m_textures, 600, 60, "texture" };
};

// the textures of the opengl renderer
class texture_manager : public texture_bank {

public:
    texture_manager();

    // activates specified texture unit
    void
        unit( GLint const Textureunit );
    // binds specified texture to specified texture unit
    void
        bind( std::size_t const Unit, texture_handle const Texture );
    opengl_texture &
        mark_as_used( texture_handle const Texture ) {
            return static_cast<opengl_texture &>( texture_bank::mark_as_used( Texture ) ); }
    // provides direct access to specified texture object
    opengl_texture &
        texture( texture_handle const Texture ) const {
            return static_cast<opengl_texture &>( texture_bank::texture( Texture ) ); }

protected:
    void
        released() override;
};

// reduces provided data image to half of original size, using basic 2x2 average
template <typename Colortype_>
void
downsample( std::size_t const Width, std::size_t const Height, unsigned char *Imagedata ) {

    Colortype_ *destination = reinterpret_cast<Colortype_*>( Imagedata );
    Colortype_ *sampler = reinterpret_cast<Colortype_*>( Imagedata );

    Colortype_ accumulator, color;
/*
    _Colortype color;
    float component;
*/
    for( std::size_t row = 0; row < Height; row += 2, sampler += Width ) { // column movement advances us down another row
        for( std::size_t column = 0; column < Width; column += 2, sampler += 2 ) {
/*
            // straightforward, but won't work with byte data
            auto color = (
                *sampler
                + *( sampler + 1 )
                + *( sampler + Width )
                + *( sampler + Width + 1 ) );
            color /= 4;
*/
            // manual version of the above, but drops colour resolution to 6 bits
            accumulator = *sampler;
            accumulator /= 4;
            color = accumulator;
            accumulator = *(sampler + 1);
            accumulator /= 4;
            color += accumulator;
            accumulator = *(sampler + Width);
            accumulator /= 4;
            color += accumulator;
            accumulator = *(sampler + Width + 1);
            accumulator /= 4;
            color += accumulator;

            *destination++ = color;
/*
            // "full" 8bit resolution
            color = Colortype_(); component = 0;
            for( int idx = 0; idx < sizeof( Colortype_ ); ++idx ) {

                component = (
                    (*sampler)[idx]
                    + ( *( sampler + 1 ) )[idx]
                    + ( *( sampler + Width ) )[idx]
                    + ( *( sampler + Width + 1 ))[idx] );
                color[ idx ] = component /= 4;
            }
            *destination++ = color;
*/
        }
    }
}

//---------------------------------------------------------------------------
