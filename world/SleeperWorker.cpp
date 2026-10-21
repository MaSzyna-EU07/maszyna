/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/SleeperWorker.h"
#include "world/SleeperLayout.h"
#include "world/SleeperRules.h"
#include "world/Track.h"
#include "simulation/simulation.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

namespace track_sleepers {

namespace {

// the version of the layout of the beams, moved on whenever a track changes, so the beams worked out for a cluster hold till then
std::atomic<std::uint64_t> &layout_epoch() {
    static std::atomic<std::uint64_t> epoch { 1 };
    return epoch;
}

std::map<TTrack const *, std::shared_ptr<cluster_beams>> &known_clusters() {
    static std::map<TTrack const *, std::shared_ptr<cluster_beams>> clusters;
    return clusters;
}

std::mutex &clusters_mutex() {
    static std::mutex mutex;
    return mutex;
}

// the beams of the cluster of the track worked out already for the current layout, null where they aren't
std::shared_ptr<cluster_beams> cached_cluster( TTrack const &Track ) {

    std::lock_guard<std::mutex> lock { clusters_mutex() };
    auto const found { known_clusters().find( &Track ) };
    return ( found != known_clusters().end() && found->second->epoch == layout_epoch() ? found->second : nullptr );
}

// the beams of the cluster of the track worked out, and kept for each track of it unless the tracks changed meanwhile
std::shared_ptr<cluster_beams> work_out( TTrack &Track ) {

    auto const start { std::chrono::steady_clock::now() };
    auto const epoch { layout_epoch().load() };
    auto const tracks { cluster_of( Track ) };
    auto const laid { lay_out( tracks, epoch ) };
    faults().slowest = std::max( faults().slowest, std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - start ).count() );
    faults().biggest = std::max( faults().biggest, tracks.size() );
    std::lock_guard<std::mutex> lock { clusters_mutex() };
    if( laid->epoch == layout_epoch() ) {
        for( auto *track : tracks ) {
            known_clusters()[ track ] = laid;
        }
    }
    return laid;
}

std::vector<beam> beams_of( cluster_beams const &Cluster, TTrack const &Track ) {

    auto const beams { Cluster.beams.find( &Track ) };
    return ( beams != Cluster.beams.end() ? beams->second : std::vector<beam> {} );
}

// the clusters of the tracks coming near worked out in a thread of their own, one after another
class background_worker {

public:
    ~background_worker() {
        {
            std::lock_guard<std::mutex> lock { mutex };
            stop = true;
            queue.clear();
        }
        wake.notify_all();
        if( thread.joinable() ) {
            thread.join();
        }
    }

    bool enabled { true };

    void request( TTrack &Track ) {

        if( false == resolved ) {
            // the models the sleepers are made of loaded here, the thread can't load them
            resolved = true;
            for( auto *track : simulation::Paths.sequence() ) {
                if( track != nullptr && bedded( *track ) ) {
                    plain_model( *track );
                    timber_model( *track );
                }
            }
        }
        {
            std::lock_guard<std::mutex> lock { mutex };
            if( queued.count( &Track ) ) {
                return;
            }
            queued.insert( &Track );
            queue.push_back( &Track );
            if( false == thread.joinable() ) {
                thread = std::thread( [ this ]() { run(); } );
            }
        }
        wake.notify_one();
    }

    // nothing left to work out nor being worked out, the waiting ones dropped
    void settle() {

        std::unique_lock<std::mutex> lock { mutex };
        queue.clear();
        queued.clear();
        idle.wait( lock, [ this ]() { return false == busy; } );
    }

private:
    void run() {

        while( true ) {
            TTrack *track { nullptr };
            {
                std::unique_lock<std::mutex> lock { mutex };
                wake.wait( lock, [ this ]() { return stop || false == queue.empty(); } );
                if( stop ) {
                    return;
                }
                track = queue.front();
                queue.pop_front();
                busy = true;
            }
            if( cached_cluster( *track ) == nullptr ) {
                work_out( *track );
            }
            {
                std::lock_guard<std::mutex> lock { mutex };
                queued.erase( track );
                busy = false;
            }
            idle.notify_all();
        }
    }

    std::mutex mutex;
    std::condition_variable wake;
    std::condition_variable idle;
    std::deque<TTrack *> queue;
    std::set<TTrack const *> queued;
    std::thread thread;
    bool stop { false };
    bool busy { false };
    bool resolved { false };
};

background_worker &worker() {
    static background_worker worker;
    return worker;
}

} // namespace

std::vector<beam> place( TTrack &Track ) {

    auto cluster { cached_cluster( Track ) };
    if( cluster == nullptr ) {
        cluster = work_out( Track );
    }
    return beams_of( *cluster, Track );
}

void lay_in_background( bool const Background ) {

    if( false == Background ) {
        worker().settle();
    }
    worker().enabled = Background;
}

void prefetch( TTrack &Track ) {

    if( worker().enabled && cached_cluster( Track ) == nullptr ) {
        worker().request( Track );
    }
}

void outdate_layouts() {

    ++layout_epoch();
}

std::optional<std::vector<beam>> laid_out( TTrack const &Track ) {

    if( auto const cluster { cached_cluster( Track ) }; cluster != nullptr ) {
        return beams_of( *cluster, Track );
    }
    return std::nullopt;
}

bool in_background() {

    return worker().enabled;
}

void ask_layout( TTrack &Track ) {

    worker().request( Track );
}

void settle_background() {

    worker().settle();
}

void forget_layout( TTrack const &Track ) {

    std::lock_guard<std::mutex> lock { clusters_mutex() };
    known_clusters().erase( &Track );
}

} // namespace track_sleepers
