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

// Reads and decodes terrain mesh tiles on a thread of its own.
//
// Reading a tile means finding it in the archive, unpacking it and turning the quantized vertices
// back into something a vertex buffer will take. None of that belongs on the render thread. The
// renderer says which tiles it wants, nearest first, and collects whatever has arrived at its own
// pace.
class terrain_tile_loader {

public:
    struct payload {
        quantizedmesh::reader::tile_data data;
        bool valid { false };
    };

    terrain_tile_loader() = default;
    ~terrain_tile_loader();
    terrain_tile_loader( terrain_tile_loader const & ) = delete;
    terrain_tile_loader &operator=( terrain_tile_loader const & ) = delete;

    // opens the source on the loader's own handle and starts the thread. Origin is what the
    // vertices come back measured from
    bool open( std::string const &Path, double Originx, double Originy, double Originz );
    // stops the thread; anything not collected is dropped
    void close();

    // replaces the list of wanted tiles, nearest first. tiles being read or already read and not
    // yet collected are left out of it, so asking again every frame costs nothing
    void want( std::vector<quantizedmesh::tile_address> const &Requests );
    // moves up to Count finished tiles into Out, returning how many
    std::size_t collect( std::vector<payload> &Out, std::size_t const Count );
    // tiles queued or finished and not yet collected
    std::size_t backlog() const;

private:
    void work();
    // whether the tile is being read or waits to be collected. the caller holds the lock
    bool underway( quantizedmesh::tile_address const &Tile ) const;

    quantizedmesh::reader m_reader;
    double m_originx { 0.0 }, m_originy { 0.0 }, m_originz { 0.0 };
    std::thread m_thread;
    mutable std::mutex m_mutex;
    std::condition_variable m_wake;
    std::deque<quantizedmesh::tile_address> m_pending;
    std::optional<quantizedmesh::tile_address> m_running;
    std::deque<payload> m_done;
    bool m_stop { false };
    // finished tiles allowed to wait for collection. past this the thread stops reading: the
    // renderer uploads at its own pace, and a backlog read for where the camera was a second ago
    // is work spent on tiles nobody wants any more
    static constexpr std::size_t donelimit { 32 };
};

}
