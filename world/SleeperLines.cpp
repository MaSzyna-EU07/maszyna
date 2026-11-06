/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/SleeperLines.h"
#include "world/SleeperGeometry.h"
#include "world/SleeperRules.h"

namespace track_sleepers {

// the beams spread evenly between two anchors, the pin at the first one included and the one at the last left to the next stretch
void spread( anchor const &From, anchor const &To, std::vector<double> &Stations ) {

    auto const span { To.at - From.at };
    if( span <= 0.0 ) {
        return;
    }
    auto count { static_cast<int>( std::round( span / pitch + 1.0 - From.offset - To.offset ) ) };
    if( From.offset == 0.5 && To.offset == 0.5 ) {
        // a track between two others gets a beam however short it is
        count = std::max( 1, count );
    }
    // the rails are carried at most so far apart, the beams pulled closer where it takes one more
    while( span / ( count - 1 + From.offset + To.offset ) > pitch_max ) {
        ++count;
    }
    auto const intervals { count - 1 + From.offset + To.offset };
    if( count < 1 || intervals <= 0.0 ) {
        if( From.offset == 0.0 ) {
            Stations.push_back( From.at );
        }
        return;
    }
    auto const step { span / intervals };
    auto const last { To.offset == 0.0 ? count - 1 : count };
    for( int index = 0; index < last; ++index ) {
        Stations.push_back( From.at + ( From.offset + index ) * step );
    }
}

void find_crossings( sampled_path const &Path, glm::dvec3 const &Point, glm::dvec2 const &Along, glm::dvec2 const &Across, double const Reach, std::vector<sampled_path> const &Around, std::vector<crossing> &Crossings ) {

    for( auto const &other : Around ) {
        if( &other == &Path || false == near_bounds( other, Point, Reach ) ) {
            continue;
        }
        for( std::size_t index = 0; index + 1 < other.points.size(); ++index ) {
            auto const &from { other.points[ index ] };
            auto const &to { other.points[ index + 1 ] };
            auto const fromside { glm::dot( plan( from - Point ), Along ) };
            auto const toside { glm::dot( plan( to - Point ), Along ) };
            if( ( fromside < 0.0 ) == ( toside < 0.0 ) ) {
                continue;
            }
            auto const share { fromside / ( fromside - toside ) };
            auto const point { from + ( to - from ) * share };
            auto const across { glm::dot( plan( point - Point ), Across ) };
            if( std::abs( across ) > Reach || std::abs( point.y - Point.y ) > level_tolerance ) {
                continue;
            }
            auto const direction { plan( to - from ) };
            if( glm::dot( direction, direction ) < 1e-12 ) {
                continue;
            }
            auto const unit { glm::normalize( direction ) };
            auto const at { glm::mix( other.distances[ index ], other.distances[ index + 1 ], share ) };
            Crossings.emplace_back( &other, point, unit, at, across, std::abs( glm::dot( unit, Along ) ), false );
        }
    }
}

namespace {

// a path whose rails the line of the beam takes, by where it crosses the line
struct carried_path {
    double across { 0.0 };
    double half { 0.0 };
    double rails { 0.0 };
    double sine { 0.0 };
};

// whether the plain sleepers of the path crossing the line would meet the sleepers of a path the line takes already
bool meets( crossing const &Crossing, std::vector<carried_path> const &Taken, bool const Tight ) {

    return std::ranges::any_of( Taken, [ &Crossing, Tight ]( carried_path const &Carried ) {
        auto const reach { Tight ? Crossing.path->rails + Carried.rails + 2.0 * beam_overhang_min + end_gap : Crossing.path->half + Carried.half + end_gap };
        return std::abs( Crossing.across - Carried.across ) * std::min( Crossing.sine, Carried.sine ) < reach; } );
}

// whether the line takes the rails of the crossing: at a narrow angle, meeting a path it takes, coplanar or too close to be parted
bool takes( sampled_path const &Path, glm::dvec2 const &Forward, crossing const &Crossing, std::vector<carried_path> const &Taken, bool const Tight, bool const Count ) {

    if( Crossing.carried || Crossing.sine < crossing_sine_min || angle_between( Crossing.direction, Forward ) >= grid_angle ) {
        return false;
    }
    if( false == meets( Crossing, Taken, Tight ) ) {
        return false;
    }
    if( off_plane( *Crossing.path, Crossing.point, Path ) > coplanar_tolerance ) {
        if( std::abs( Crossing.across ) * Crossing.sine >= shared_spacing( *Crossing.path, Path ) ) {
            return false;
        }
        // too close to the path to be parted, out of the plane of its rails
        if( Count ) {
            ++faults().offplane;
        }
    }
    return true;
}

// whether the rails of the crossing lie past those of a track the line could take but doesn't, on the same side of the path
bool blocked( crossing const &Crossing, std::vector<crossing> const &Crossings, glm::dvec2 const &Forward ) {

    auto const blocks = [ &Forward ]( crossing const &Other ) {
        return false == Other.carried && Other.sine >= crossing_sine_min && angle_between( Other.direction, Forward ) < grid_angle; };
    return std::ranges::any_of( Crossings, [ &blocks, &Crossing ]( crossing const &Other ) {
        return blocks( Other ) && Other.across * Crossing.across > 0.0 && std::abs( Other.across ) < std::abs( Crossing.across ); } );
}

} // namespace

// the paths whose rails the line of the beam through the point of the path takes, marked among the paths of its cluster crossing
// the line: those whose plain sleepers would meet the sleepers of the path or of any the line takes already, at a narrow enough
// angle to the line, and in one plane with the path unless they run too close to be parted
// the timber takes the rails of a path whose plain sleepers would meet those of a path it takes already; where Tight, only of one so
// close that their sleepers couldn't be laid apart even cut short
void carry( sampled_path const &Path, glm::dvec2 const &Forward, glm::dvec2 const &Along, std::vector<crossing> &Crossings, bool const Tight, bool const Count ) {

    std::vector<carried_path> carried { { 0.0, Path.half, Path.rails, std::max( crossing_sine_min, std::abs( glm::dot( Forward, Along ) ) ) } };
    for( auto added { true }; added; ) {
        added = false;
        for( auto &crossing : Crossings ) {
            if( false == takes( Path, Forward, crossing, carried, Tight, Count ) ) {
                continue;
            }
            crossing.carried = true;
            added = true;
            carried.emplace_back( crossing.across, crossing.path->half, crossing.path->rails, crossing.sine );
        }
    }
    // the line doesn't carry the rails of a track past those of one it doesn't, that one lays its own beams there
    for( auto demoted { true }; demoted; ) {
        demoted = false;
        for( auto &crossing : Crossings ) {
            if( false == crossing.carried ) {
                continue;
            }
            if( blocked( crossing, Crossings, Forward ) ) {
                crossing.carried = false;
                demoted = true;
            }
        }
    }
}

band band_of( sampled_path const &Path, double const Across, double const Sine, glm::dvec2 const &Direction, bool const Carried ) {

    return { Across - Path.rails / Sine, Across + Path.rails / Sine, beam_overhang / Sine, beam_overhang_min / Sine, Carried, &Path, Across, Direction, Sine };
}

namespace {

// the end of the timber kept clear of the rails of the next track the line doesn't carry, and of the sleepers under them
void end_short( piece &Current, band const &Next, double const End, double const Endoverhang, double const Endoverhangmin ) {

    auto const kept { std::max( 0.0, std::clamp( Next.from - Next.overhang - End - end_gap, std::min( Endoverhangmin, Next.from - End - end_gap ), Endoverhang ) ) };
    Current.clipped = Current.clipped || kept < Endoverhang;
    if( kept < Endoverhangmin ) {
        ++faults().clipped;
    }
    Current.to = End + kept;
    Current.coreto = End;
}

// the overhang the timber starts with, clear of the rails of the last track the line doesn't carry and of the sleepers under them
double start_short( piece &Current, band const &Next, double const Fencerails, double const Fencereach ) {

    auto const startoverhang { std::max( 0.0, std::clamp( Next.from - Fencereach - end_gap, std::min( Next.overhangmin, Next.from - Fencerails - end_gap ), Next.overhang ) ) };
    Current.clipped = startoverhang < Next.overhang;
    if( startoverhang < Next.overhangmin ) {
        ++faults().clipped;
    }
    return startoverhang;
}

} // namespace

// the beams on the line through the point: one timber under all the rails it carries, the ends keeping clear of the rails of the
// tracks the line doesn't carry and of the sleepers under them. the timber is parted between the rails of two tracks where each
// keeps at least its least overhang past its rails, unless it's whole: about a crossing of the tracks, as in a scissors crossing
void emit_line( std::vector<band> &Bands, std::vector<piece> &Pieces, bool const Whole ) {

    std::ranges::sort( Bands, []( band const &Left, band const &Right ) { return Left.from < Right.from; } );
    auto open { false };
    piece current;
    auto end { 0.0 };
    auto endoverhang { 0.0 };
    auto endoverhangmin { 0.0 };
    // the far side of the last rails not carried, and of the sleepers under them
    auto fenced { false };
    auto fencerails { 0.0 };
    auto fencereach { 0.0 };
    for( auto const &next : Bands ) {
        if( false == next.carried ) {
            if( open ) {
                end_short( current, next, end, endoverhang, endoverhangmin );
                Pieces.push_back( current );
                open = false;
            }
            if( false == fenced || next.to > fencerails ) {
                fencerails = next.to;
                fencereach = next.to + next.overhang;
            }
            fenced = true;
            continue;
        }
        if( false == open ) {
            current = piece();
            auto startoverhang { next.overhang };
            if( fenced ) {
                startoverhang = start_short( current, next, fencerails, fencereach );
            }
            current.from = next.from - startoverhang;
            current.corefrom = next.from;
            current.single = &next;
            end = next.to;
            endoverhang = next.overhang;
            endoverhangmin = next.overhangmin;
            open = true;
            continue;
        }
        if( false == Whole && next.from - end >= endoverhangmin + next.overhangmin + end_gap ) {
            // the room between them shared by their overhangs
            auto const room { next.from - end - end_gap };
            auto const ours { std::min( endoverhang, room * endoverhang / ( endoverhang + next.overhang ) ) };
            auto const theirs { std::min( next.overhang, room - ours ) };
            current.to = end + ours;
            current.coreto = end;
            current.clipped = current.clipped || ours < endoverhang;
            Pieces.push_back( current );
            current = piece();
            current.from = next.from - theirs;
            current.corefrom = next.from;
            current.clipped = theirs < next.overhang;
            current.single = &next;
            end = next.to;
            endoverhang = next.overhang;
            endoverhangmin = next.overhangmin;
            continue;
        }
        // one timber under the rails of all the tracks it carries
        current.single = nullptr;
        if( next.to > end ) {
            end = next.to;
            endoverhang = next.overhang;
            endoverhangmin = next.overhangmin;
        }
    }
    if( open ) {
        current.to = end + endoverhang;
        current.coreto = end;
        Pieces.push_back( current );
    }
}

// a timber longer than the cap cut in two where the rails of the tracks it carries are furthest apart, the ends sharing the
// room between them, till every piece is short enough or there's nowhere to cut
void cut_long( std::vector<band> const &Bands, std::vector<piece> &Pieces ) {

    for( std::size_t index = 0; index < Pieces.size(); ) {
        auto &whole { Pieces[ index ] };
        if( whole.to - whole.from <= length_cap ) {
            ++index;
            continue;
        }
        // the widest gap between the rails under the timber
        band const *left { nullptr };
        band const *right { nullptr };
        band const *last { nullptr };
        auto widest { 0.0 };
        auto end { std::numeric_limits<double>::lowest() };
        for( auto const &band : Bands ) {
            if( false == band.carried || band.from < whole.from || band.to > whole.to ) {
                continue;
            }
            if( last != nullptr && band.from - end > widest ) {
                widest = band.from - end;
                left = last;
                right = &band;
            }
            if( band.to > end ) {
                end = band.to;
                last = &band;
            }
        }
        if( left == nullptr ) {
            ++index;
            continue;
        }
        auto const room { std::max( 0.0, widest - end_gap ) };
        auto const ours { std::min( left->overhang, room * left->overhang / ( left->overhang + right->overhang ) ) };
        auto const theirs { std::min( right->overhang, room - ours ) };
        if( ours < left->overhangmin || theirs < right->overhangmin ) {
            ++faults().clipped;
        }
        piece second { right->from - theirs, whole.to, true, right->from, whole.coreto, nullptr };
        whole.to = left->to + ours;
        whole.coreto = left->to;
        whole.clipped = true;
        Pieces.insert( Pieces.begin() + index + 1, second );
    }
}


} // namespace track_sleepers
