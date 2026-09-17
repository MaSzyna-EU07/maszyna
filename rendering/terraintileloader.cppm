/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include "scene/quantizedmeshreader.h"

export module eu07.rendering.terraintileloader;

export {

// Reads and decodes TIN (Quantized Mesh) terrain tiles on a thread of its own.
//
// Reads .qm tile files, dequantizes vertices, and hands back ready-to-upload vertex/index
// data. Does the I/O and decompression off the render thread. The renderer uploads at its own
// pace, and the loader prioritizes nearest tiles first.
//
// Replaces heightfield tile loading entirely - TIN is the only terrain format now.
class terrain_tile_loader {

public:
    struct request {
        std::int32_t x { 0 }, z { 0 };
        std::uint32_t level { 0 };

        bool operator==( request const & ) const = default;
    };

    struct payload {
        request tile;
        bool valid { false };
        
        // Vertex data (dequantized, ready for GPU)
        std::vector<double> positions_x;
        std::vector<double> positions_y;
        std::vector<double> positions_z;
        std::vector<float> uvs_u;
        std::vector<float> uvs_v;
        std::vector<std::uint8_t> materials;
        
        // Index data
        std::vector<std::uint32_t> indices;
        
        // Edge indices for seamless LOD stitching
        std::vector<std::uint16_t> north_edge;
        std::vector<std::uint16_t> south_edge;
        std::vector<std::uint16_t> west_edge;
        std::vector<std::uint16_t> east_edge;
        
        // Tile bounds (for placement)
        double center_x, center_y, center_z;
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;
        double geometric_error;
    };

    terrain_tile_loader() = default;
    ~terrain_tile_loader();
    terrain_tile_loader( terrain_tile_loader const & ) = delete;
    terrain_tile_loader &operator=( terrain_tile_loader const & ) = delete;

    // opens the file on the loader's own handle and starts the thread
    bool open( std::string const &Path );
    // stops the thread; anything not collected is dropped
    void close();

    // replaces the list of wanted tiles, nearest first. tiles being read or already read and
    // not yet collected are left out of it, so asking again every frame costs nothing
    void want( std::vector<request> const &Requests );
    // moves up to Count finished tiles into Out, returning how many
    std::size_t collect( std::vector<payload> &Out, std::size_t const Count );
    // tiles queued or finished and not yet collected
    std::size_t backlog() const;

private:
    void work();
    // whether the tile is being read or waits to be collected. caller holds the lock
    bool underway( request const &Tile ) const;

    quantizedmesh::reader m_reader;
    std::thread m_thread;
    mutable std::mutex m_mutex;
    std::condition_variable m_wake;
    std::deque<request> m_pending;
    std::optional<request> m_running;
    std::deque<payload> m_done;
    bool m_stop { false };
    // finished tiles allowed to wait for collection. past this the thread stops reading:
    // the renderer uploads at its own pace, and a backlog read for where the camera was a
    // second ago is work spent on tiles nobody wants any more
    static constexpr std::size_t donelimit { 32 };
};

}
