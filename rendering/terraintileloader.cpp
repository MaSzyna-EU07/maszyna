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
#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "scene/heightfieldreader.h"

module eu07.rendering.terraintileloader;

terrain_tile_loader::~terrain_tile_loader() {

    close();
}

bool
terrain_tile_loader::open( std::string const &Path ) {

    close();
    if( false == m_reader.open( Path ) ) { return false; }
    m_stop = false;
    m_thread = std::thread( &terrain_tile_loader::work, this );
    return true;
}

void
terrain_tile_loader::close() {

    if( true == m_thread.joinable() ) {
        {
            std::lock_guard<std::mutex> lock( m_mutex );
            m_stop = true;
        }
        m_wake.notify_all();
        m_thread.join();
    }
    std::lock_guard<std::mutex> lock( m_mutex );
    m_pending.clear();
    m_done.clear();
    m_running.reset();
    m_reader.close();
}

bool
terrain_tile_loader::underway( request const &Tile ) const {

    if( ( m_running.has_value() ) && ( *m_running == Tile ) ) { return true; }
    return std::any_of(
        m_done.begin(), m_done.end(),
        [ &Tile ]( payload const &Done ) { return Done.tile == Tile; } );
}

void
terrain_tile_loader::want( std::vector<request> const &Requests ) {

    {
        std::lock_guard<std::mutex> lock( m_mutex );
        m_pending.clear();
        for( auto const &tile : Requests ) {
            if( false == underway( tile ) ) {
                m_pending.push_back( tile );
            }
        }
    }
    m_wake.notify_one();
}

std::size_t
terrain_tile_loader::collect( std::vector<payload> &Out, std::size_t const Count ) {

    std::unique_lock<std::mutex> lock( m_mutex );
    std::size_t moved { 0 };
    while( ( moved < Count ) && ( false == m_done.empty() ) ) {
        Out.emplace_back( std::move( m_done.front() ) );
        m_done.pop_front();
        ++moved;
    }
    lock.unlock();
    if( moved > 0 ) {
        // room was made; the thread may be waiting for it
        m_wake.notify_one();
    }
    return moved;
}

std::size_t
terrain_tile_loader::backlog() const {

    std::lock_guard<std::mutex> lock( m_mutex );
    return m_pending.size() + m_done.size() + ( m_running.has_value() ? 1 : 0 );
}

void
terrain_tile_loader::work() {

    while( true ) {
        request tile;
        {
            std::unique_lock<std::mutex> lock( m_mutex );
            m_wake.wait( lock, [ this ]() {
                return m_stop || ( ( false == m_pending.empty() ) && ( m_done.size() < donelimit ) ); } );
            if( true == m_stop ) { return; }
            tile = m_pending.front();
            m_pending.pop_front();
            m_running = tile;
        }

        // the read itself happens outside the lock, so the render thread can replace the
        // wish list while a tile is being decoded
        payload result;
        result.tile = tile;
        result.valid = m_reader.read_level( tile.x, tile.z, tile.level, result.heights, result.materials );

        std::lock_guard<std::mutex> lock( m_mutex );
        m_running.reset();
        m_done.emplace_back( std::move( result ) );
    }
}
