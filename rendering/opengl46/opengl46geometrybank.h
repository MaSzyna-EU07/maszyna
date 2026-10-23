/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "rendering/geometrybank.h"
#include "gl/buffer.h"
#include "gl/vao.h"

namespace gfx {

struct opengl46_geometry_page;

// opengl vao/vbo-based variant of the geometry bank.
// a bank without user data keeps its geometry in a part of a large vertex and index buffer shared with other banks, with one
// vao for all of them (opengl46_geometry_page), so the chunks of many banks can be drawn in one multi draw; the offsets of its
// chunks are places in the shared buffers. a bank with user data (an extra vertex stream) has buffers and a vao of its own

class opengl46_vaogeometrybank : public geometry_bank {

public:
// constructors:
    opengl46_vaogeometrybank() = default;
// destructor
    ~opengl46_vaogeometrybank() {
        delete_buffer(); }
// methods:
    static
    void
        reset() {;}
    // a line for the debug panel: the pages of the shared buffers and how much of them the banks take
    static
    std::string
        shared_info();

private:
// types:
    struct chunk_record {
        std::size_t vertex_offset{ 0 }; // beginning of the chunk vertex data as offset from the beginning of the last established buffer
        std::size_t vertex_count{ 0 }; // size of the chunk in the last established buffer
        std::size_t index_offset{ 0 };
        std::size_t index_count{ 0 };
		bool has_userdata{ false };
        bool is_good{ false }; // true if local content of the chunk matches the data on the opengl end
    };

    typedef std::vector<chunk_record> chunkrecord_sequence;

// methods:
    // create() subclass details
    void
        create_( gfx::geometry_handle const &Geometry ) override;
    // replace() subclass details
    void
        replace_( gfx::geometry_handle const &Geometry ) override;
    // draw() subclass details
    auto
        draw_( gfx::geometry_handle const &Geometry, gfx::stream_units const &Units, unsigned int const Streams ) -> std::size_t override;
    // draw_instanced() subclass details — issues glDrawElementsInstancedBaseVertexBaseInstance
    auto
        draw_instanced_( gfx::geometry_handle const &Geometry, gfx::stream_units const &Units, std::size_t const InstanceCount, std::uint32_t const BaseInstance, unsigned int const Streams ) -> std::size_t override;
    // draw_batch() subclass details — issues glMultiDrawArrays for runs of non-indexed chunks of the same type
    auto
        draw_batch_( gfx::geometry_handle const *First, gfx::geometry_handle const *Last, gfx::stream_units const &Units, unsigned int const Streams ) -> std::size_t override;
    // indirect_source() subclass details — uploads the chunk if needed, tells its place in the buffers
    auto
        indirect_source_( gfx::geometry_handle const &Geometry, gfx::indirect_source &Source ) -> bool override;
    // draw_indirect() subclass details — issues glDrawElementsIndirect or glDrawArraysIndirect
    void
        draw_indirect_( gfx::geometry_handle const &Geometry, std::intptr_t const Offset, unsigned int const Streams ) override;
    // draw_indirect_multi() subclass details — issues glMultiDrawElementsIndirect or glMultiDrawArraysIndirect
    auto
        draw_indirect_multi_( gfx::geometry_handle const &Geometry, std::intptr_t const Offset, std::uint32_t const Count, std::uint32_t const Stride ) -> bool override;
    // release() subclass details
    void
        release_() override;
    // draw_group() subclass details: the shared buffers, if the bank keeps its geometry in them
    auto
        draw_group_() const -> std::uint32_t override;
    void
        setup_buffer();
    // the buffers of the bank were set up
    bool
        has_buffer() const {
            return ( ( m_page != nullptr ) || m_vertexbuffer.has_value() ); }
    // binds the vao the bank is drawn with
    void
        bind_vao();
    // sends the data of a chunk to the buffers, unless they hold it already
    void
        upload_chunk( chunk_record &Record, geometry_chunk const &Chunk );
    void
        setup_attrib(size_t offset = 0);
	void
		setup_userdata(size_t offset = 0);
    void
        delete_buffer();

// members:
	std::optional<gl::buffer> m_vertexbuffer; // vertex buffer data on the opengl end
    std::optional<gl::buffer> m_indexbuffer; // index buffer data on the opengl end
	std::optional<gl::vao> m_vao;
    chunkrecord_sequence m_chunkrecords; // helper data for all stored geometry chunks, in matching order
	int m_vertex_count;
    // the shared buffers holding the geometry of the bank, and its part of them: places of the first vertex and index, counts
    opengl46_geometry_page *m_page { nullptr };
    std::size_t m_pagevertexoffset { 0 };
    std::size_t m_pagevertexcount { 0 };
    std::size_t m_pageindexoffset { 0 };
    std::size_t m_pageindexcount { 0 };
};

} // namespace gfx
