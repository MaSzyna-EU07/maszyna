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
#include "scene/heightfieldreader.h"

export module eu07.rendering.terraintileloader;

export {

// Reads and decodes heightfield tiles on a thread of its own.
//
// Reading a tile means a seek, a read and a zstd decode, which is too much to do on the render
// thread for the dozens of tiles a moving camera asks for at once. This does that part and
// nothing else: it knows no gl, keeps its own file handle, and hands back plain sample arrays
// for the renderer to upload at a pace it chooses.
//
// The renderer says each frame what it wants, nearest first, and that list replaces whatever
// was asked for before and has not been started. A tile the camera has already left is never
// read, and the nearest tile is always the next one read.
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
        std::vector<std::uint16_t> heights;
        std::vector<std::uint8_t> materials;
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

    heightfield::reader m_reader;
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
