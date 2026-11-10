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
            Crossings.push_back( { &other, point, unit, at, across, std::abs( glm::dot( unit, Along ) ), false } );
        }
    }
}

// the paths whose rails the line of the beam through the point of the path takes, marked among the paths of its cluster crossing
// the line: those whose plain sleepers would meet the sleepers of the path or of any the line takes already, at a narrow enough
// angle to the line, and in one plane with the path unless they run too close to be parted
// the timber takes the rails of a path whose plain sleepers would meet those of a path it takes already; where Tight, only of one so
// close that their sleepers couldn't be laid apart even cut short
void carry( sampled_path const &Path, glm::dvec2 const &Forward, glm::dvec2 const &Along, std::vector<crossing> &Crossings, bool const Tight, bool const Count ) {

    struct carried_path {
        double across;
        double half;
        double rails;
        double sine;
    };
    std::vector<carried_path> carried { { 0.0, Path.half, Path.rails, std::max( crossing_sine_min, std::abs( glm::dot( Forward, Along ) ) ) } };
    for( auto added { true }; added; ) {
        added = false;
        for( auto &crossing : Crossings ) {
            if( crossing.carried || crossing.sine < crossing_sine_min || angle_between( crossing.direction, Forward ) >= grid_angle ) {
                continue;
            }
            auto const close { std::any_of( carried.begin(), carried.end(), [&]( carried_path const &Carried ) {
                auto const reach { Tight ? crossing.path->rails + Carried.rails + 2.0 * overhang_min + end_gap : crossing.path->half + Carried.half + end_gap };
                return std::abs( crossing.across - Carried.across ) * std::min( crossing.sine, Carried.sine ) < reach; } ) };
            if( false == close ) {
                continue;
            }
            if( off_plane( *crossing.path, crossing.point, Path ) > coplanar_tolerance ) {
                if( std::abs( crossing.across ) * crossing.sine >= shared_spacing( *crossing.path, Path ) ) {
                    continue;
                }
                // too close to the path to be parted, out of the plane of its rails
                if( Count ) {
                    ++faults().offplane;
                }
            }
            crossing.carried = true;
            added = true;
            carried.push_back( { crossing.across, crossing.path->half, crossing.path->rails, crossing.sine } );
        }
    }
    // the line doesn't carry the rails of a track past those of one it doesn't, that one lays its own beams there
    auto const blocks = [&]( crossing const &Other ) {
        return false == Other.carried && Other.sine >= crossing_sine_min && angle_between( Other.direction, Forward ) < grid_angle; };
    for( auto demoted { true }; demoted; ) {
        demoted = false;
        for( auto &crossing : Crossings ) {
            if( false == crossing.carried ) {
                continue;
            }
            if( std::any_of( Crossings.begin(), Crossings.end(), [&]( struct crossing const &Other ) {
                    return blocks( Other ) && Other.across * crossing.across > 0.0 && std::abs( Other.across ) < std::abs( crossing.across ); } ) ) {
                crossing.carried = false;
                demoted = true;
            }
        }
    }
}

band band_of( sampled_path const &Path, double const Across, double const Sine, glm::dvec2 const &Direction, bool const Carried ) {

    return { Across - Path.rails / Sine, Across + Path.rails / Sine, overhang / Sine, overhang_min / Sine, Carried, &Path, Across, Direction, Sine };
}

// the beams on the line through the point: one timber under all the rails it carries, the ends keeping clear of the rails of the
// tracks the line doesn't carry and of the sleepers under them. the timber is parted between the rails of two tracks where each
// keeps at least its least overhang past its rails, unless it's whole: about a crossing of the tracks, as in a scissors crossing
void emit_line( std::vector<band> &Bands, std::vector<piece> &Pieces, bool const Whole ) {

    std::sort( Bands.begin(), Bands.end(), []( band const &Left, band const &Right ) { return Left.from < Right.from; } );
    auto open { false };
    piece current;
    auto end { 0.0 }, endoverhang { 0.0 }, endoverhangmin { 0.0 };
    // the far side of the last rails not carried, and of the sleepers under them
    auto fenced { false };
    auto fencerails { 0.0 }, fencereach { 0.0 };
    for( auto const &next : Bands ) {
        if( false == next.carried ) {
            if( open ) {
                auto const kept { std::max( 0.0, std::clamp( next.from - next.overhang - end - end_gap, std::min( endoverhangmin, next.from - end - end_gap ), endoverhang ) ) };
                current.clipped = current.clipped || kept < endoverhang;
                if( kept < endoverhangmin ) {
                    ++faults().clipped;
                }
                current.to = end + kept;
                current.coreto = end;
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
                startoverhang = std::max( 0.0, std::clamp( next.from - fencereach - end_gap, std::min( next.overhangmin, next.from - fencerails - end_gap ), next.overhang ) );
                current.clipped = startoverhang < next.overhang;
                if( startoverhang < next.overhangmin ) {
                    ++faults().clipped;
                }
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
