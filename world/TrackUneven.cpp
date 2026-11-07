/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/Track.h"

#include <span>

// The part of TTrack that goes with the unevenness of a track and with what it meets at its ends: joined tracks and switches.
// The rules themselves are in TrackUneven.h, here they get applied to the tracks.

namespace {

double const switch_end_reach { 0.3 }; // how near an end of a path counts as being at a point, m
double const joint_reach { 0.2 }; // ends of joined paths further apart than this stay as they are, m
double const joint_alike { 0.9 }; // ends of joined paths less alike in direction than this stay as they are, cosine

bool is_switch( TTrack const *Track ) {

    return Track != nullptr && Track->eType == tt_Switch && Track->SwitchExtension;
}

glm::dvec3 end_point( TSegment const &Segment, int const End ) {

    return End == 0 ? Segment.FastGetPoint_0() : Segment.FastGetPoint_1();
}

std::array<TTrack *, 4> joined_to_switch( TTrack const &Switch ) {

    auto const &extension { *Switch.SwitchExtension };
    return { extension.pPrevs[ 0 ], extension.pPrevs[ 1 ], extension.pNexts[ 0 ], extension.pNexts[ 1 ] };
}

// the switch and the switches joined directly to it, and to those
std::vector<TTrack *> switch_group( TTrack &Switch ) {

    std::vector group { &Switch };
    std::vector pending { &Switch };
    while( false == pending.empty() ) {
        auto const *track { pending.back() };
        pending.pop_back();
        for( auto *other : joined_to_switch( *track ) ) {
            if( is_switch( other ) && std::ranges::find( group, other ) == group.end() ) {
                group.push_back( other );
                pending.push_back( other );
            }
        }
    }
    return group;
}

// a plain track with none of a component keeps the switch to its design in it, any other comes to the value of the switch
void keep_to_plain_track( std::array<track_uneven::amounts, 2> &Values, TTrack const &Plain ) {

    auto const theirs { Plain.uneven_declared() };
    for( auto &path : Values ) {
        if( theirs.up <= 0.0 ) { path.up = 0.0; }
        if( theirs.side <= 0.0 ) { path.side = 0.0; }
    }
}

// the value along each path: the lowest of that path in the group, 0 in a component that a plain track joined has none of
std::array<track_uneven::amounts, 2> group_path_values( std::span<TTrack * const> const Group ) {

    std::array<track_uneven::amounts, 2> values { Group.front()->uneven_declared( 0 ), Group.front()->uneven_declared( 1 ) };
    for( auto const *track : Group ) {
        values[ 0 ] = track_uneven::least( values[ 0 ], track->uneven_declared( 0 ) );
        values[ 1 ] = track_uneven::least( values[ 1 ], track->uneven_declared( 1 ) );
        for( auto const *other : joined_to_switch( *track ) ) {
            if( other != nullptr && false == is_switch( other ) ) {
                keep_to_plain_track( values, *other );
            }
        }
    }
    return values;
}

// the lowest Value( path, end ) of the ends of the paths at the point; Otherwise when none of them is there
template <typename Value_>
track_uneven::amounts least_at_ends( TTrack const &Switch, glm::dvec3 const &Point, track_uneven::amounts const &Otherwise, Value_ const &Value ) {

    auto found { false };
    track_uneven::amounts result { track_uneven::most, track_uneven::most, track_uneven::most };
    for( int path = 0; path < 2; ++path ) {
        for( int end = 0; end < 2; ++end ) {
            if( glm::length( end_point( *Switch.path_segment( path ), end ) - Point ) < switch_end_reach ) {
                result = track_uneven::least( result, Value( path, end ) );
                found = true;
            }
        }
    }
    return found ? result : Otherwise;
}

// how the end of a path meets the track joined to it
struct path_joint {
    std::optional<std::pair<glm::dvec3, glm::dvec3>> place; // none: too far apart, or going different ways
    float roll;
};

path_joint meet( TTrack const &Track, TSegment const &Segment, int const End, TTrack const &Neighbour ) {

    path_joint result { {}, Segment.design_roll( End ) };
    glm::dvec3 point;
    glm::dvec3 across;
    Segment.plan_frame( End ? Segment.GetLength() : 0.0, point, across );
    auto closest { joint_reach };
    for( int path = 0; path < Neighbour.path_count(); ++path ) {
        auto const *theirs { Neighbour.path_segment( path ) };
        if( theirs == nullptr ) {
            continue;
        }
        for( int end = 0; end < 2; ++end ) {
            // laid towards each other, they look across and lean to opposite sides
            auto const opposite { end == End };
            glm::dvec3 theirpoint;
            glm::dvec3 theiracross;
            theirs->plan_frame( end ? theirs->GetLength() : 0.0, theirpoint, theiracross );
            if( opposite ) {
                theiracross = -theiracross;
            }
            auto const gap { glm::length( theirpoint - point ) };
            if( gap >= closest || glm::dot( across, theiracross ) < joint_alike ) {
                continue;
            }
            closest = gap;
            result.place.emplace( 0.5 * ( point + theirpoint ), glm::normalize( across + theiracross ) );
            if( Track.iCategoryFlag & 1 ) {
                result.roll = track_uneven::joint_roll(
                    Segment.design_roll( End ),
                    opposite ? -theirs->design_roll( end ) : theirs->design_roll( end ),
                    Track.eType == tt_Switch,
                    Neighbour.eType == tt_Switch );
            }
        }
    }
    return result;
}

} // namespace

int TTrack::path_count() const {

    return eType == tt_Switch ? 2 : 1;
}

TSegment *TTrack::path_segment( int const Path ) const {

    return eType == tt_Switch ? SwitchExtension->Segments[ Path ].get() : Segment.get();
}

TTrack *TTrack::path_neighbour( int const Path, int const End ) const {

    if( eType == tt_Switch ) {
        return End == 0 ? SwitchExtension->pPrevs[ Path ] : SwitchExtension->pNexts[ Path ];
    }
    return End == 0 ? trPrev : trNext;
}

track_uneven::amounts TTrack::uneven_declared( int const Path ) const {

    return track_uneven::declared( m_uneven, eType == tt_Normal, Path );
}

bool TTrack::is_uneven() const {

    auto const any = []( track_uneven::amounts const &Amount ) {
        return Amount.up > 0.0 || Amount.side > 0.0 || Amount.cross > 0.0; };
    return any( uneven_declared( 0 ) ) || any( uneven_declared( 1 ) ) || joint_depth() > 0.0;
}

track_uneven::amounts TTrack::uneven_underway() {

    return is_switch( this ) ? switch_uneven().paths[ SwitchExtension->CurrentIndex ] : uneven_declared();
}

double TTrack::uneven_bed_share( double const Depth ) const {

    return track_uneven::bed_share( Depth, std::abs( track_rail_profile( m_profile1.second ).front().position.y ), fTexHeight1 );
}

track_uneven::amounts TTrack::uneven_at( double const Distance, int const Path ) {

    if( eType == tt_Switch ) {
        auto const &stray { switch_uneven() };
        return track_uneven::along( stray.paths[ Path ], stray.ends[ Path ][ 0 ], stray.ends[ Path ][ 1 ], Distance, path_segment( Path )->GetLength() );
    }
    return track_uneven::along( uneven_declared(), uneven_met( trPrev, end_point( *Segment, 0 ) ), uneven_met( trNext, end_point( *Segment, 1 ) ), Distance, Length() );
}

track_uneven::amounts TTrack::uneven_met( TTrack *Other, glm::dvec3 const &Point ) const {

    if( Other == nullptr ) {
        return {};
    }
    return Other->eType == tt_Switch ? Other->switch_uneven_at( Point ) : track_uneven::least( uneven_declared(), Other->uneven_declared() );
}

TTrack::switch_stray const &TTrack::switch_uneven() {

    if( m_switchstray.stamp != s_layoutstamp ) {
        settle_switch_stray();
    }
    return m_switchstray;
}

void TTrack::settle_switch_stray() {

    auto const group { switch_group( *this ) };
    auto const values { group_path_values( group ) };
    for( auto *track : group ) {
        track->m_switchstray.paths = values;
        track->m_switchstray.stamp = s_layoutstamp;
    }
    // with the paths known in all of the group
    for( auto *track : group ) {
        track->settle_switch_ends();
    }
}

void TTrack::settle_switch_ends() {

    for( int path = 0; path < 2; ++path ) {
        for( int end = 0; end < 2; ++end ) {
            auto const point { end_point( *path_segment( path ), end ) };
            auto value { switch_paths_at( point ) };
            if( auto const *other { path_neighbour( path, end ) }; is_switch( other ) ) {
                value = track_uneven::least( value, other->switch_paths_at( point ) );
            }
            m_switchstray.ends[ path ][ end ] = value;
        }
    }
}

track_uneven::amounts TTrack::switch_paths_at( glm::dvec3 const &Point ) const {

    auto const &paths { m_switchstray.paths };
    return least_at_ends( *this, Point, track_uneven::least( paths[ 0 ], paths[ 1 ] ), [ &paths ]( int const Path, int ) { return paths[ Path ]; } );
}

track_uneven::amounts TTrack::switch_uneven_at( glm::dvec3 const &Point ) {

    auto const &stray { switch_uneven() };
    return least_at_ends( *this, Point, track_uneven::least( stray.paths[ 0 ], stray.paths[ 1 ] ), [ &stray ]( int const Path, int const End ) { return stray.ends[ Path ][ End ]; } );
}

double TTrack::joint_depth() const {

    if( eType != tt_Normal ) {
        return 0.0;
    }
    return ( m_uneven.dip >= 0.f ? m_uneven.dip : track_uneven::dip_share * uneven_declared().up );
}

double TTrack::joint_dip( double const Distance ) {

    auto const phase { rail_phase( Distance ) };
    if( phase < 0.0 ) {
        return 0.0;
    }
    auto const fromjoint { std::min( phase, fSoundDistance - phase ) };
    if( fromjoint >= track_uneven::dip_reach ) {
        return 0.0;
    }
    auto const depth { joint_depth() };
    auto const atend = [ this, depth ]( TTrack const *Other ) {
        return ( Other == nullptr || Other->fSoundDistance != fSoundDistance ? 0.0 : std::min( depth, Other->joint_depth() ) ); };
    return 0.001 * track_uneven::dip( fromjoint ) * track_uneven::along( depth, atend( trPrev ), atend( trNext ), Distance, Length() );
}

double TTrack::rail_phase( double const Distance ) {

    if( joint_depth() <= 0.0 || fSoundDistance <= 2.0 * track_uneven::dip_reach ) {
        return -1.0;
    }
    if( m_railrun.stamp != s_layoutstamp ) {
        settle_rail_run();
    }
    return track_uneven::rail_phase( m_railrun.origin + m_railrun.way * Distance, fSoundDistance );
}

bool TTrack::rail_joints_moved() {

    return m_drawnrun.stamp != -1
        && rail_phase( 0.0 ) >= 0.0
        && ( m_railrun.origin != m_drawnrun.origin || m_railrun.way != m_drawnrun.way );
}

std::vector<TTrack *> TTrack::switch_followers( std::vector<TTrack *> const &Tracks ) {

    std::vector<TTrack *> followers;
    auto const follow = [ &followers ]( TTrack *Track ) {
        if( Track != nullptr && Track->is_uneven() && std::ranges::find( followers, Track ) == followers.end() ) {
            followers.push_back( Track );
        } };
    for( auto *track : Tracks ) {
        if( false == is_switch( track ) ) {
            continue;
        }
        for( auto *member : switch_group( *track ) ) {
            follow( member );
            std::ranges::for_each( joined_to_switch( *member ), follow );
        }
    }
    return followers;
}

void TTrack::settle_rail_run() {

    // the next track of the run past an end of a track, and which of its ends is joined there
    auto const onward = []( TTrack const &Track, int const End, int &Joined ) -> TTrack * {
        auto *other { End == 0 ? Track.trPrev : Track.trNext };
        if( other == nullptr || other == &Track || other->joint_depth() <= 0.0 || other->fSoundDistance != Track.fSoundDistance ) {
            return nullptr;
        }
        Joined = ( End == 0 ? Track.iPrevDirection : Track.iNextDirection ) & 1;
        return other; };
    // walks from this track to an end of the run
    auto const walk = [ this, &onward ]( int &End ) {
        auto *track { this };
        int joined { 0 };
        for( int step = 0; step < 1000000; ++step ) {
            auto *next { onward( *track, End, joined ) };
            if( next == nullptr || next == this ) {
                break;
            }
            track = next;
            End = 1 - joined;
        }
        return track; };
    auto const place = []( TTrack const &Track, int const End ) {
        auto const point { end_point( *Track.Segment, End ) };
        return std::make_pair( point.x, point.z ); };

    // start at the end with the lesser coordinates, to get the same joints from any track of the run
    int end { 0 };
    int otherend { 1 };
    auto *track { walk( end ) };
    if( auto *other { walk( otherend ) }; place( *other, otherend ) < place( *track, end ) ) {
        track = other;
        end = otherend;
    }
    auto along { 0.0 };
    while( track != nullptr && track->m_railrun.stamp != s_layoutstamp ) {
        auto const length { track->Length() };
        track->m_railrun = { end == 0 ? along : along + length, end == 0 ? 1.0 : -1.0, s_layoutstamp };
        along += length;
        int joined { 0 };
        track = onward( *track, 1 - end, joined );
        end = joined;
    }
    // tracks joined one way only can leave this one out of the run: it stands on its own then, not to walk all of it again on every call
    if( m_railrun.stamp != s_layoutstamp ) {
        m_railrun = { 0.0, 1.0, s_layoutstamp };
    }
}

void TTrack::share_joints() {

    for( int path = 0; path < path_count(); ++path ) {
        auto *segment { path_segment( path ) };
        if( segment == nullptr ) {
            continue;
        }
        for( int end = 0; end < 2; ++end ) {
            auto const *neighbour { path_neighbour( path, end ) };
            auto const joined { neighbour != nullptr && neighbour != this && ( neighbour->iCategoryFlag & 15 ) == ( iCategoryFlag & 15 ) };
            auto const joint { joined ? meet( *this, *segment, end, *neighbour ) : path_joint { {}, segment->design_roll( end ) } };
            segment->set_joint( end, joint.place, joint.roll );
        }
    }
    if( eType != tt_Switch && Segment && ( iCategoryFlag & 1 ) ) {
        float roll1 { 0.f };
        float roll2 { 0.f };
        Segment->GetRolls( roll1, roll2 );
        if( roll1 != 0.f || roll2 != 0.f ) {
            iTrapezoid |= 1;
        }
    }
}
