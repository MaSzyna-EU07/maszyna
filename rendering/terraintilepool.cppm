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
#include <optional>
#include <vector>

export module eu07.rendering.terraintilepool;
import eu07.gl.buffer;

export {

// GPU storage for the terrain tiles of one mip level: a texture array of heights and one of
// material indices, a layer per tile. Tiles of a level all have the same side, so they share
// the arrays, and the whole level draws from them in a single call.
//
// Slots are handed out and taken back as tiles come and go. The arrays start small and double
// when full, copying what they hold on the gpu, so a level only ever takes the memory its
// tiles need. Uploads go through a pixel buffer that is orphaned for each tile, which lets the
// driver take the data without stalling on the previous transfer.
class terrain_tile_pool {

public:
    explicit terrain_tile_pool( std::uint32_t const Side );
    ~terrain_tile_pool();
    terrain_tile_pool( terrain_tile_pool const & ) = delete;
    terrain_tile_pool &operator=( terrain_tile_pool const & ) = delete;

    // a free layer, growing the arrays when there is none
    std::uint32_t acquire();
    void release( std::uint32_t const Slot );
    // writes one tile's samples into its layer. both arrays hold Side * Side values
    void upload( std::uint32_t const Slot, std::uint16_t const *Heights, std::uint8_t const *Materials );

    std::uint32_t heights() const { return m_heights; }
    std::uint32_t materials() const { return m_materials; }
    std::uint32_t side() const { return m_side; }
    // gpu memory the arrays occupy
    std::size_t bytes() const { return static_cast<std::size_t>( m_side ) * m_side * 3u * m_capacity; }

private:
    void grow();

    static constexpr std::uint32_t initialcapacity { 16 };

    std::uint32_t m_side { 0 };
    std::uint32_t m_capacity { 0 };
    std::uint32_t m_heights { 0 };
    std::uint32_t m_materials { 0 };
    std::optional<gl::buffer> m_unpack;
    std::vector<std::uint32_t> m_free;
};

}
