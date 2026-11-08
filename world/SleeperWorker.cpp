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

    std::scoped_lock lock { clusters_mutex() };
    auto const found { known_clusters().find( &Track ) };
    return ( found != known_clusters().end() && found->second->epoch == layout_epoch() ? found->second : nullptr );
}

// the beams of the cluster of the track worked out, and kept for each track of it unless the tracks changed meanwhile
std::shared_ptr<cluster_beams> work_out( TTrack const &Track ) {

    auto const start { std::chrono::steady_clock::now() };
    auto const epoch { layout_epoch().load() };
    auto const tracks { cluster_of( Track ) };
    auto const laid { lay_out( tracks, epoch ) };
    raise_to( faults().slowest, std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - start ).count() );
    raise_to( faults().biggest, tracks.size() );
    std::scoped_lock lock { clusters_mutex() };
    if( laid->epoch == layout_epoch() ) {
        for( auto const *track : tracks ) {
            known_clusters()[ track ] = laid;
        }
    }
    return laid;
}

std::vector<beam> beams_of( cluster_beams const &Cluster, TTrack const &Track ) {

    auto const beams { Cluster.beams.find( &Track ) };
    return ( beams != Cluster.beams.end() ? beams->second : std::vector<beam> {} );
}

// the clusters of the tracks coming near worked out in a thread of their own, one after another.
// the thread reads the scene with no lock against the main thread, which runs meanwhile. what it reads mostly doesn't change after
// the load: the tracks of the cells, the curves of the plain tracks and of both paths of the switches, and the tracks joined to
// the paths of a switch. the editor, which changes them, stops the thread as it's entered. two things can change while driving:
// - a switch thrown sets the curve and the ends it's set to anew (Switch()): those aren't read, a switch is taken by both its paths
//   and the tracks joined to them;
// - a turntable turned parts from the tracks joined to it and joins them again, and makes its curve anew: it's never laid on, so
//   its curve isn't read, and the links of the tracks next to it are, by where a line of tracks ends; the line ends at the
//   turntable whether they're joined at the time or not, so only the link itself is read meanwhile
class background_worker {

public:
    // stopped as the application exits, before the scenery goes; here only where it exits some other way
    ~background_worker() {
        shut_down();
    }

    // the thread finishes the cluster it's on and is joined, the waiting ones dropped; it isn't started again
    void shut_down() {
        {
            std::scoped_lock lock { mutex };
            stop = true;
            queue.clear();
            queued.clear();
        }
        wake.notify_all();
        if( thread.joinable() ) {
            thread.join();
        }
        enabled = false;
    }

    // whether the layouts are worked out in the thread
    bool is_enabled() const {
        return enabled;
    }

    // the layouts worked out in the thread from now on, or where they're asked for
    void set_enabled( bool const Enabled ) {
        enabled = Enabled;
    }

    void request( TTrack &Track ) {

        if( auto const epoch { layout_epoch().load() }; epoch != resolved ) {
            // the models the sleepers are made of loaded here, the thread doesn't load them; again whenever a track changed, as it
            // may have got other models
            resolved = epoch;
            for( auto const *track : simulation::Paths.sequence() ) {
                if( track != nullptr && bedded( *track ) ) {
                    plain_model( *track );
                    timber_model( *track );
                }
            }
        }
        {
            std::scoped_lock lock { mutex };
            if( stop || queued.contains( &Track ) ) {
                return;
            }
            queued.insert( &Track );
            queue.push_back( &Track );
            if( false == thread.joinable() ) {
                thread = std::jthread( [ this ]() { run(); } );
            }
        }
        wake.notify_one();
    }

    // nothing left to work out nor being worked out, the waiting ones dropped
    void settle() {

        std::unique_lock lock { mutex };
        queue.clear();
        queued.clear();
        idle.wait( lock, [ this ]() { return false == busy; } );
    }

private:
    void run() {

        models_loaded( false );
        while( true ) {
            TTrack const *track { nullptr };
            {
                std::unique_lock lock { mutex };
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
                std::scoped_lock lock { mutex };
                queued.erase( track );
                busy = false;
            }
            idle.notify_all();
        }
    }

    bool enabled { true };
    std::mutex mutex;
    std::condition_variable wake;
    std::condition_variable idle;
    std::deque<TTrack *> queue;
    std::set<TTrack const *> queued;
    std::jthread thread;
    bool stop { false };
    bool busy { false };
    std::uint64_t resolved { 0 }; // the version of the layout the models were loaded for
};

background_worker &worker() {
    // what the thread keeps the clusters in is made first, to be destroyed after it where the application didn't stop it
    layout_epoch();
    known_clusters();
    clusters_mutex();
    faults();
    static background_worker worker;
    return worker;
}

} // namespace

std::vector<beam> place( TTrack const &Track ) {

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
    worker().set_enabled( Background );
}

void prefetch( TTrack &Track ) {

    if( worker().is_enabled() && cached_cluster( Track ) == nullptr ) {
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

    return worker().is_enabled();
}

void ask_layout( TTrack &Track ) {

    worker().request( Track );
}

void settle_background() {

    worker().settle();
}

void stop_background() {

    worker().shut_down();
}

} // namespace track_sleepers
