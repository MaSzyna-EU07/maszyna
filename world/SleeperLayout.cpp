/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/SleeperLayout.h"
#include "world/SleeperFeatures.h"
#include "world/SleeperGeometry.h"
#include "world/SleeperLines.h"
#include "world/SleeperPaths.h"
#include "world/SleeperRules.h"
#include "world/Track.h"

#include <optional>

namespace track_sleepers {

namespace {

// whether the plain sleepers of the two paths would meet anywhere, so they lie on timbers together there
bool share_anywhere( sampled_path const &Left, sampled_path const &Right ) {

    auto const margin { Left.half + Right.half + half_width + end_gap };
    if( Left.low.x > Right.high.x + margin || Right.low.x > Left.high.x + margin || Left.low.y > Right.high.y + margin || Right.low.y > Left.high.y + margin ) {
        return false;
    }
    for( auto const &pair : { std::make_pair( &Left, &Right ), std::make_pair( &Right, &Left ) } ) {
        auto const &path { *pair.first };
        for( std::size_t index = 0; index < path.points.size(); ++index ) {
            if( shares_with( path, path.points[ index ], heading( path, path.distances[ index ] ), *pair.second ) ) {
                return true;
            }
        }
    }
    return false;
}

// the stretches of the path where another path of the cluster runs so near that their plain sleepers would meet
std::vector<glm::dvec2> zones_of( sampled_path const &Path, std::vector<sampled_path> const &Paths, std::vector<feature> const &Crossings ) {

    return stretches_where( Path, [&]( glm::dvec3 const &Point, glm::dvec2 const &Direction ) {
        return std::any_of( Paths.begin(), Paths.end(), [&]( sampled_path const &Other ) {
            if( &Other == &Path ) {
                return false;
            }
            // tracks crossing share the timbers where their sleepers would meet, others only where they're too close to part them
            auto const crossed { std::any_of( Crossings.begin(), Crossings.end(), [&]( feature const &Feature ) {
                return ( Feature.paths[ 0 ] == &Path && Feature.paths[ 1 ] == &Other ) || ( Feature.paths[ 0 ] == &Other && Feature.paths[ 1 ] == &Path ); } ) };
            if( crossed ) {
                return shares_with( Path, Point, Direction, Other );
            }
            double gap;
            glm::dvec2 otherdirection { Direction };
            return beside( Other, Point, shared_spacing( Path, Other ), gap, otherdirection ) && gap < shared_spacing( Path, Other ); } ); } );
}

// the stretches left of the first ones once the second ones are taken out, those shorter than the least length left out too
std::vector<glm::dvec2> without( std::vector<glm::dvec2> const &Stretches, std::vector<glm::dvec2> const &Taken, double const Least ) {

    std::vector<glm::dvec2> result { Stretches };
    for( auto const &taken : Taken ) {
        std::vector<glm::dvec2> left;
        for( auto const &stretch : result ) {
            if( stretch.y <= taken.x || stretch.x >= taken.y ) {
                left.push_back( stretch );
                continue;
            }
            if( stretch.x < taken.x ) {
                left.emplace_back( stretch.x, taken.x );
            }
            if( stretch.y > taken.y ) {
                left.emplace_back( taken.y, stretch.y );
            }
        }
        result = std::move( left );
    }
    result.erase( std::remove_if( result.begin(), result.end(), [&]( glm::dvec2 const &Stretch ) { return Stretch.y - Stretch.x < Least; } ), result.end() );
    return result;
}

// the beams of a cluster of tracks, laid together: lines of timbers led along the paths near other paths one after another, each
// line across all the rails of the cluster it reaches, and the plain sleepers of every path where no line carries it, in step
// with the last timber carrying it
class cluster_layout {

public:
    cluster_layout( std::vector<TTrack *> const &Tracks, std::uint64_t const Epoch ) :
        tracks( Tracks ), epoch( Epoch ) {}

    std::shared_ptr<cluster_beams> lay() {

        auto result { std::make_shared<cluster_beams>() };
        result->epoch = epoch;
        for( auto *track : tracks ) {
            result->beams[ track ];
            for( int path = 0; path < path_count( *track ); ++path ) {
                paths.emplace_back( sample( *track, path ) );
            }
        }
        paths.erase( std::remove_if( paths.begin(), paths.end(), []( sampled_path const &Path ) { return Path.points.size() < 2; } ), paths.end() );
        for( auto *track : tracks ) {
            for( auto &other : paths_around( *track ) ) {
                if( std::find( tracks.begin(), tracks.end(), other.track ) != tracks.end()
                 || std::any_of( context.begin(), context.end(), [&]( sampled_path const &Known ) { return Known.track == other.track && Known.path == other.path; } ) ) {
                    continue;
                }
                context.emplace_back( std::move( other ) );
            }
        }
        features = find_features( paths );
        for( auto const &feature : features ) {
            if( false == feature.frog && false == feature.grid ) {
                crossings.push_back( feature );
            }
        }
        zones.resize( paths.size() );
        records.resize( paths.size() );
        done.resize( paths.size() );
        for( std::size_t index = 0; index < paths.size(); ++index ) {
            zones[ index ] = ( tracks.size() > 1 || paths.size() > 1 ? zones_of( paths[ index ], paths, crossings ) : std::vector<glm::dvec2> {} );
        }
        // the lines of timbers, led along the path first by priority with a stretch near another path no line carries yet
        for( std::size_t round = 0; round < 1000; ++round ) {
            auto best { paths.size() };
            glm::dvec2 stretch { 0.0 };
            for( std::size_t index = 0; index < paths.size(); ++index ) {
                auto const left { without( zones[ index ], covered( index ), 0.3 ) };
                if( left.empty() ) {
                    continue;
                }
                if( best == paths.size() || before( paths[ index ], paths[ best ] ) ) {
                    best = index;
                    stretch = *std::max_element( left.begin(), left.end(), []( glm::dvec2 const &Left, glm::dvec2 const &Right ) { return Left.y - Left.x < Right.y - Right.x; } );
                }
            }
            if( best == paths.size() ) {
                break;
            }
            lead( best, stretch );
            done[ best ].push_back( stretch );
        }
        // the plain sleepers of each path, between the timbers carrying it and its ends
        for( std::size_t index = 0; index < paths.size(); ++index ) {
            lay_plain( index );
        }
        keep_apart();
        fill_gaps();
        keep_apart();
        for( auto const &laid : beams ) {
            result->beams[ laid.owner ].push_back( laid.value );
        }
        return result;
    }

private:
    struct owned {
        TTrack const *owner;
        beam value;
    };

    std::vector<TTrack *> const &tracks;
    std::uint64_t const epoch; // of the layout the tracks are laid out for
    std::vector<sampled_path> paths; // of the tracks of the cluster
    std::vector<sampled_path> context; // of the tracks about them, whose rails and sleepers the beams keep clear of
    std::vector<feature> features;
    std::vector<feature> crossings; // the features where tracks cross, without the frogs
    std::vector<std::vector<glm::dvec2>> zones; // of each path, where its plain sleepers would meet those of another one
    // a line of timbers carrying a path: where along the path, and which way the line runs in plan
    struct record {
        double at;
        glm::dvec2 along;
    };
    std::vector<std::vector<record>> records; // the lines of timbers carrying each path
    std::vector<std::vector<glm::dvec2>> done; // the stretches of each path lines were led along
    std::vector<owned> beams;

    // the stretches of the path the lines laid so far carry it on, and those lines were led along
    std::vector<glm::dvec2> covered( std::size_t const Index ) const {

        auto result { done[ Index ] };
        auto const at { distances( Index ) };
        if( at.empty() ) {
            return result;
        }
        auto start { at.front() };
        auto last { at.front() };
        for( std::size_t index = 1; index <= at.size(); ++index ) {
            if( index < at.size() && at[ index ] - last <= 1.6 * pitch ) {
                last = at[ index ];
                continue;
            }
            result.emplace_back( start - 0.5 * pitch, last + 0.5 * pitch );
            if( index < at.size() ) {
                start = last = at[ index ];
            }
        }
        return result;
    }

    // the sine of the angle the paths of a grid cross at, no less than of 30 degrees
    static double grid_sine( feature const &Feature ) {

        return std::max( 0.5, std::sin( 2.0 * std::asin( Feature.sine ) ) );
    }

    // the pins of the lines led along the path: the rails of a path crossing this one so wide it's laid on a grid of beams; elsewhere
    // the lines are spread evenly
    std::vector<double> pins( sampled_path const &Path ) const {

        std::vector<double> result;
        for( auto const &feature : features ) {
            if( false == feature.grid ) {
                continue;
            }
            for( int side = 0; side < 2; ++side ) {
                if( feature.paths[ side ] != &Path ) {
                    continue;
                }
                // under the middles of the rails of the other path, which runs across the beams
                auto const *other { feature.paths[ 1 - side ] };
                auto const rail { ( 0.5 * other->gauge + rail_web ) / grid_sine( feature ) };
                result.push_back( feature.at[ side ] - rail );
                result.push_back( feature.at[ side ] + rail );
            }
        }
        std::sort( result.begin(), result.end() );
        return result;
    }

    // whether the beam of the path at the distance is a timber of a crossing at a wide angle: where it crosses another path so wide
    // they're laid as a grid, the other path laying none there, or under the guard rails of the crossing, as far as they reach
    bool crossing_timber( sampled_path const &Path, double const Distance ) const {

        return std::any_of( features.begin(), features.end(), [&]( feature const &Feature ) {
            if( Feature.frog ) {
                return false;
            }
            auto const angle { 2.0 * std::asin( std::min( 1.0, Feature.sine ) ) };
            auto const acute { std::min( angle, glm::pi<double>() - angle ) };
            for( int side = 0; side < 2; ++side ) {
                if( Feature.paths[ side ] != &Path ) {
                    continue;
                }
                auto const *other { Feature.paths[ 1 - side ] };
                auto reach { Feature.grid ? ( other->half + half_width + end_gap ) / grid_sine( Feature ) : 0.0 };
                if( acute > track_rails::wide_angle ) {
                    // past where the rails cross, as far as the guard rails go on along them, a beam reaching under their ends too
                    reach = std::max(
                        reach,
                        ( 0.5 * other->gauge + 0.5 * Path.gauge * std::cos( acute ) + track_rails::flangeway + rail_head + track_rails::guard_reach ) / std::sin( acute ) + half_width );
                }
                if( std::abs( Distance - Feature.at[ side ] ) < reach ) {
                    return true;
                }
            }
            return false; } );
    }

    // the line of the timbers carrying the path nearest to the distance on the side, between the distance and the reach from it
    std::optional<record> record_near( std::size_t const Index, double const Distance, double const Side, double const Reach ) const {

        std::optional<record> result;
        for( auto const &line : records[ Index ] ) {
            auto const along { ( line.at - Distance ) * Side };
            if( along < -0.25 * pitch || along > Reach ) {
                continue;
            }
            if( false == result.has_value() || std::abs( line.at - Distance ) < std::abs( result->at - Distance ) ) {
                result = line;
            }
        }
        return result;
    }

    // where along the path the lines of timbers carry it, in order
    std::vector<double> distances( std::size_t const Index ) const {

        std::vector<double> result;
        for( auto const &line : records[ Index ] ) {
            result.push_back( line.at );
        }
        std::sort( result.begin(), result.end() );
        result.erase( std::unique( result.begin(), result.end(), []( double const Left, double const Right ) { return Right - Left < 0.05; } ), result.end() );
        return result;
    }

    // where the path is at the distance and which way it runs in plan; false where it can't tell
    bool frame_at( sampled_path const &Path, double const Distance, glm::dvec3 &Point, glm::vec3 &Angles, glm::dvec2 &Forward ) const {

        Path.segment->RaPositionGet( Distance, Point, Angles );
        glm::dvec3 behind, ahead;
        glm::vec3 unused;
        Path.segment->RaPositionGet( std::max( 0.0, Distance - 0.1 ), behind, unused );
        Path.segment->RaPositionGet( std::min( Path.length, Distance + 0.1 ), ahead, unused );
        Forward = plan( ahead - behind );
        if( glm::length( Forward ) < 1e-6 ) {
            return false;
        }
        Forward = glm::normalize( Forward );
        return true;
    }

    // the lines at the distances along the path, turned as the frogs and crossings about them ask, but by no more than the rate
    // from one line to the next, nor from the lines carrying the path at either end, so the ends of long timbers keep their spacing
    void lay_lines( std::size_t const Index, std::vector<double> const &Stations, std::optional<record> const &First, std::optional<record> const &Last, double const Rate, bool const Led ) {

        auto const &path { paths[ Index ] };
        std::vector<double> distances;
        std::vector<glm::dvec2> forwards;
        std::vector<double> turns; // of the lines off square to the path, in plan
        for( auto const station : Stations ) {
            glm::dvec3 point;
            glm::vec3 angles;
            glm::dvec2 forward;
            if( false == frame_at( path, station, point, angles, forward ) ) {
                continue;
            }
            distances.push_back( station );
            forwards.push_back( forward );
            // the plain sleepers lie square to their track, turned only to meet the timbers next to them
            auto along { Led ? beam_direction( point, forward, features ) : forward };
            if( Led ) {
                // the timbers about a crossing lie square to the mean way of the tracks they carry, the frogs about it don't turn them:
                // square to the main tracks of a scissors crossing, to the bisector of a diamond
                if( auto const mean { whole_at( path, point, forward, along ) }; mean.has_value() ) {
                    along = *mean;
                }
            }
            turns.push_back( turn_off( forward, along ) );
        }
        if( distances.empty() ) {
            return;
        }
        auto const limit = [&]( std::size_t const From, std::size_t const To, std::optional<record> const &Seed ) {
            auto const step { To > From ? 1 : -1 };
            std::optional<double> previous;
            auto rate { Rate };
            if( Seed.has_value() ) {
                previous = turn_off( forwards[ From ], Seed->along );
                if( false == Led ) {
                    // square after three sleepers
                    rate = std::max( 0.004, std::abs( *previous ) / 3.0 );
                }
            }
            for( auto index { From }; ; index += step ) {
                if( previous.has_value() ) {
                    turns[ index ] = std::clamp( turns[ index ], *previous - rate, *previous + rate );
                }
                previous = turns[ index ];
                if( index == To ) {
                    break;
                }
            } };
        limit( 0, distances.size() - 1, First );
        limit( distances.size() - 1, 0, Last );
        for( std::size_t index = 0; index < distances.size(); ++index ) {
            lay_line( Index, distances[ index ], Led, rotated( forwards[ index ], turns[ index ] ) );
        }
    }

    // the lines of timbers led along the stretch of the path, spread between the lines carrying it at either end, or the ends of the
    // stretch, and the pins on it
    void lead( std::size_t const Index, glm::dvec2 const &Stretch ) {

        auto const &path { paths[ Index ] };
        auto const back { record_near( Index, Stretch.x, -1.0, 2.0 * pitch ) };
        auto const fore { record_near( Index, Stretch.y, 1.0, 2.0 * pitch ) };
        anchor const start { back.has_value() ? anchor { back->at, 1.0 } : anchor { Stretch.x <= 0.01 ? 0.0 : Stretch.x, 0.5 } };
        anchor const end { fore.has_value() ? anchor { fore->at, 1.0 } : anchor { Stretch.y >= path.length - 0.01 ? path.length : Stretch.y, 0.5 } };
        if( end.at - start.at <= 0.0 ) {
            return;
        }
        std::vector<anchor> anchors { start };
        for( auto const pin : pins( path ) ) {
            // two timbers side by side under the rail running along them, its fastenings on both
            if( pin - half_width > start.at + 0.5 * pitch_min && pin + half_width < end.at - 0.5 * pitch_min && pin - half_width - anchors.back().at >= pitch_min ) {
                anchors.push_back( { pin - half_width, 0.0 } );
                anchors.push_back( { pin + half_width, 0.0 } );
            }
        }
        anchors.push_back( end );
        std::vector<double> stations;
        for( std::size_t index = 1; index < anchors.size(); ++index ) {
            spread( anchors[ index - 1 ], anchors[ index ], stations );
        }
        lay_lines( Index, stations, back, fore, timber_turn, true );
    }

    // where the timber led along the path through the point, lying the given way, carries both tracks of a crossing: the way it lies
    // then, square to the middle between the ways of the tracks it carries turned furthest either way
    std::optional<glm::dvec2> whole_at( sampled_path const &Path, glm::dvec3 const &Point, glm::dvec2 const &Forward, glm::dvec2 const &Along ) const {

        if( crossings.empty() ) {
            return std::nullopt;
        }
        std::vector<crossing> members;
        find_crossings( Path, Point, Along, glm::dvec2 { -Along.y, Along.x }, 0.5 * length_cap + Path.reach, paths, members );
        carry( Path, Forward, Along, members, false, false );
        if( false == whole_with( Path, members ) ) {
            return std::nullopt;
        }
        // the outermost tracks it carries, either way along it: the main tracks of a scissors crossing, the crossing ones of a diamond
        std::pair<double, glm::dvec2> low { 0.0, Forward };
        std::pair<double, glm::dvec2> high { 0.0, Forward };
        for( auto const &crossing : members ) {
            if( false == crossing.carried ) {
                continue;
            }
            if( crossing.across < low.first ) {
                low = { crossing.across, crossing.direction };
            }
            if( crossing.across > high.first ) {
                high = { crossing.across, crossing.direction };
            }
        }
        return rotated( Forward, 0.5 * ( turn_off( Forward, low.second ) + turn_off( Forward, high.second ) ) );
    }

    bool whole_with( sampled_path const &Path, std::vector<crossing> const &Members ) const {

        auto const carries = [&]( sampled_path const *Carried ) {
            return Carried == &Path || std::any_of( Members.begin(), Members.end(), [&]( crossing const &Crossing ) { return Crossing.carried && Crossing.path == Carried; } ); };
        return std::any_of( crossings.begin(), crossings.end(), [&]( feature const &Feature ) {
            return carries( Feature.paths[ 0 ] ) && carries( Feature.paths[ 1 ] ); } );
    }

    // the plain sleepers of the path on the stretches no line of timbers carries it, in step with the timbers at their ends
    void lay_plain( std::size_t const Index ) {

        auto const &path { paths[ Index ] };
        auto const at { distances( Index ) };
        std::vector<std::pair<anchor, anchor>> stretches;
        if( at.empty() ) {
            stretches.push_back( { { 0.0, 0.5 }, { path.length, 0.5 } } );
        }
        else {
            stretches.push_back( { { 0.0, 0.5 }, { at.front(), 1.0 } } );
            for( std::size_t index = 1; index < at.size(); ++index ) {
                if( at[ index ] - at[ index - 1 ] > 1.6 * pitch ) {
                    stretches.push_back( { { at[ index - 1 ], 1.0 }, { at[ index ], 1.0 } } );
                }
            }
            stretches.push_back( { { at.back(), 1.0 }, { path.length, 0.5 } } );
        }
        for( auto const &stretch : stretches ) {
            auto const span { stretch.second.at - stretch.first.at };
            if( span <= 0.0 || span < ( stretch.first.offset + stretch.second.offset - 1.0 ) * pitch + 0.5 * pitch_min ) {
                continue;
            }
            std::vector<double> stations;
            spread( stretch.first, stretch.second, stations );
            // turned back square to the path from the timbers at the ends of the stretch
            auto const first { stretch.first.offset == 1.0 ? record_near( Index, stretch.first.at, 1.0, 0.05 ) : std::optional<record> {} };
            auto const last { stretch.second.offset == 1.0 ? record_near( Index, stretch.second.at, -1.0, 0.05 ) : std::optional<record> {} };
            lay_lines( Index, stations, first, last, sleeper_turn, false );
        }
    }

    // the beams on the line through the point of the path at the distance: the timber of the line led along it, taking the rails of
    // the paths of the cluster it reaches, or the plain sleeper of the path alone
    void lay_line( std::size_t const Index, double const Distance, bool const Led, glm::dvec2 const &Along ) {

        auto const &path { paths[ Index ] };
        glm::dvec3 point;
        glm::vec3 angles;
        glm::dvec2 forward;
        if( false == frame_at( path, Distance, point, angles, forward ) ) {
            return;
        }
        auto const along { Along };
        glm::dvec2 const across { -along.y, along.x };
        auto const reach { 0.5 * length_cap + path.reach };
        std::vector<crossing> members, others;
        find_crossings( path, point, along, across, reach, paths, members );
        find_crossings( path, point, along, across, reach, context, others );
        if( Led ) {
            // about a crossing the timbers take all the tracks whose sleepers would meet, elsewhere only those too close to part
            carry( path, forward, along, members, false );
            if( false == whole_with( path, members ) ) {
                for( auto &crossing : members ) {
                    crossing.carried = false;
                }
                carry( path, forward, along, members, true );
            }
        }
        std::vector<band> bands { band_of( path, 0.0, std::max( crossing_sine_min, std::abs( glm::dot( forward, along ) ) ), forward ) };
        for( auto const *crossings : { &members, &others } ) {
            for( auto const &crossing : *crossings ) {
                if( crossing.carried ) {
                    bands.push_back( band_of( *crossing.path, crossing.across, crossing.sine, crossing.direction ) );
                }
                else if( crossing.sine >= crossing_sine_min && angle_between( crossing.direction, forward ) < grid_angle ) {
                    bands.push_back( band_of( *crossing.path, crossing.across, crossing.sine, crossing.direction, false ) );
                }
            }
        }
        // a timber carrying both tracks of a crossing lies whole under all the tracks it carries
        auto const whole { Led && whole_with( path, members ) };
        std::vector<piece> pieces;
        emit_line( bands, pieces, whole );
        cut_long( bands, pieces );
        // the parts of a timber parted between tracks going apart turn square to their own track, the more the further apart the
        // tracks run, till square where their plain sleepers wouldn't touch any more
        std::vector<glm::dvec2> turned( pieces.size(), along );
        for( std::size_t index = 0; index < pieces.size() && pieces.size() > 1; ++index ) {
            auto const *own { pieces[ index ].single };
            if( own == nullptr ) {
                continue;
            }
            band const *nearest { nullptr };
            for( auto const &other : bands ) {
                if( &other != own && other.carried && ( nearest == nullptr || std::abs( other.centre - own->centre ) < std::abs( nearest->centre - own->centre ) ) ) {
                    nearest = &other;
                }
            }
            if( nearest == nullptr ) {
                continue;
            }
            auto const spacing { std::abs( nearest->centre - own->centre ) * std::min( own->sine, nearest->sine ) };
            auto const parted { shared_spacing( *own->path, *nearest->path ) };
            auto const apart { own->path->half + nearest->path->half + end_gap };
            auto const share { apart > parted ? std::clamp( ( spacing - parted ) / ( apart - parted ), 0.0, 1.0 ) : 1.0 };
            turned[ index ] = rotated( along, share * turn_off( along, own->direction ) );
        }
        auto const turned_for = [&]( sampled_path const *Path ) {
            for( std::size_t index = 0; index < pieces.size(); ++index ) {
                if( pieces[ index ].single != nullptr && pieces[ index ].single->path == Path ) {
                    return turned[ index ];
                }
            }
            return along; };
        if( Led ) {
            records[ Index ].push_back( { Distance, turned_for( &path ) } );
            for( auto const &crossing : members ) {
                if( crossing.carried ) {
                    records[ crossing.path - paths.data() ].push_back( { crossing.at, turned_for( crossing.path ) } );
                }
            }
        }
        // the beam lies tilted with the roll of the path, as the ballast is
        glm::dvec3 const flatacross { across.x, 0.0, across.y };
        glm::dvec3 const up { 0.0, 1.0, 0.0 };
        auto const roll { static_cast<double>( angles.x ) };
        auto const rolledacross { flatacross * std::cos( roll ) - up * std::sin( roll ) };
        auto const rolledup { flatacross * std::sin( roll ) + up * std::cos( roll ) };
        auto const top { point - rolledup * static_cast<double>( path.track->rail_height() ) };
        // a sleeper of a plain track running alone and square to it, the rest are the timbers of the switches and crossings
        auto const carried { std::count_if( bands.begin(), bands.end(), []( band const &Band ) { return Band.carried; } ) };
        auto const plain { path.track->eType == tt_Normal && carried == 1 && pieces.size() == 1 && false == pieces.front().clipped && std::abs( glm::dot( forward, along ) ) > 0.9998 && false == crossing_timber( path, Distance ) };
        for( std::size_t index = 0; index < pieces.size(); ++index ) {
            auto const &piece { pieces[ index ] };
            auto const &way { turned[ index ] };
            if( way == along ) {
                beams.push_back( { path.track, { top + rolledacross * piece.from, top + rolledacross * piece.to, glm::dvec3 { along.x, 0.0, along.y }, plain, piece.corefrom - piece.from, piece.coreto - piece.from } } );
                continue;
            }
            // turned about where the middle of its track crosses the line
            auto const centre { top + rolledacross * piece.single->centre };
            glm::dvec3 const turnedacross { glm::dvec3 { -way.y, 0.0, way.x } * std::cos( roll ) - up * std::sin( roll ) };
            beams.push_back( { path.track, { centre + turnedacross * ( piece.from - piece.single->centre ), centre + turnedacross * ( piece.to - piece.single->centre ), glm::dvec3 { way.x, 0.0, way.y }, false, piece.corefrom - piece.from, piece.coreto - piece.from } } );
        }
    }

    // a sleeper of the path put in where the rails of it go further without a beam under them than the pitch lets them, turned
    // as the beams about it are: where beams were left out or lines of timbers led along different paths meet
    void fill_gaps() {

        auto const count { beams.size() };
        // the beams by the cells of a grid they reach into
        plan_grid<std::size_t> grid { 2.0 };
        for( std::size_t laid = 0; laid < count; ++laid ) {
            auto const &beam { beams[ laid ].value };
            grid.insert( plan( 0.5 * ( beam.from + beam.to ) ), 0.5 * glm::length( plan( beam.to - beam.from ) ) + half_width, laid );
        }
        for( std::size_t index = 0; index < paths.size(); ++index ) {
            auto const &path { paths[ index ] };
            // where along the path beams lie under each of its rails, and which way they run
            auto const steps { std::max( 2, static_cast<int>( std::ceil( path.length / 0.05 ) ) ) };
            std::vector<double> gaps;
            std::vector<glm::dvec2> turns;
            for( auto const side : { -1.0, 1.0 } ) {
                std::optional<std::pair<double, glm::dvec2>> last;
                for( int step = 0; step <= steps; ++step ) {
                    auto const distance { path.length * step / steps };
                    glm::dvec3 point, across;
                    path.segment->plan_frame( distance, point, across );
                    auto const rail { plan( point + across * ( side * 0.5 * path.gauge ) ) };
                    std::optional<glm::dvec2> found;
                    auto const *cell { grid.at( rail ) };
                    if( cell == nullptr ) {
                        continue;
                    }
                    for( auto const laid : *cell ) {
                        auto const box { box_of( beams[ laid ].value ) };
                        if( 2.0 * box.half >= 0.1 && holds( box, rail ) ) {
                            found = glm::dvec2 { -box.axis.y, box.axis.x };
                            break;
                        }
                    }
                    if( false == found.has_value() ) {
                        continue;
                    }
                    if( last.has_value() && distance - last->first > 0.06 && distance - last->first + 2.0 * half_width > 1.25 * pitch_max ) {
                        // the middle of the stretch without one, and the way the beams at its ends run
                        gaps.push_back( 0.5 * ( distance + last->first ) );
                        auto along { last->second + ( glm::dot( last->second, *found ) < 0.0 ? -*found : *found ) };
                        turns.push_back( glm::length( along ) > 1e-9 ? glm::normalize( along ) : heading( path, gaps.back() ) );
                    }
                    last = std::make_pair( distance, *found );
                }
            }
            // a sleeper in each, those of both rails about one place put in once
            for( std::size_t gap = 0; gap < gaps.size(); ++gap ) {
                if( std::any_of( gaps.begin(), gaps.begin() + gap, [&]( double const Other ) { return std::abs( Other - gaps[ gap ] ) < 0.3; } ) ) {
                    continue;
                }
                lay_line( index, gaps[ gap ], false, turns[ gap ] );
            }
        }
    }

    // a beam over one laid before it is left out: where tracks cross too wide for one timber to take both, or the lines of two
    // stretches meet at an angle
    void keep_apart() {

        plan_grid<beam_box> grid { 3.0 };
        std::vector<owned> kept;
        auto const clear_of_laid = [&]( beam_box const &Box ) {
            return grid.all_about( Box.centre, Box.half + half_width, [&]( beam_box const &Other ) { return apart( Box, Other ); } ); };
        for( auto laid : beams ) {
            auto const length { glm::length( laid.value.to - laid.value.from ) };
            if( length < 0.1 ) {
                continue;
            }
            if( false == clear_of_laid( box_of( laid.value ) ) ) {
                // shortened from either end or both, down to the least overhang past the rails it carries, till it's clear
                auto const direction { ( laid.value.to - laid.value.from ) / length };
                auto const most { std::max( 0.0, laid.value.corefrom - overhang_min ) }; // off the from end
                auto const mostto { std::max( 0.0, length - laid.value.coreto - overhang_min ) };
                auto found { false };
                for( int step = 1; step <= 20 && false == found; ++step ) {
                    auto const share { step / 20.0 };
                    for( auto const ends : { glm::dvec2 { 0.0, mostto * share }, glm::dvec2 { most * share, 0.0 }, glm::dvec2 { most * share, mostto * share } } ) {
                        auto shortened { laid.value };
                        shortened.from = laid.value.from + direction * ends.x;
                        shortened.to = laid.value.to - direction * ends.y;
                        shortened.corefrom -= ends.x;
                        shortened.coreto -= ends.x;
                        if( clear_of_laid( box_of( shortened ) ) ) {
                            shortened.plain = false;
                            laid.value = shortened;
                            found = true;
                            break;
                        }
                    }
                }
                if( false == found ) {
                    ++faults().dropped;
                    continue;
                }
                ++faults().clipped;
            }
            auto const box { box_of( laid.value ) };
            grid.insert( box.centre, box.half + half_width, box );
            kept.push_back( laid );
        }
        beams = std::move( kept );
    }
};


} // namespace

// the tracks whose timbers are laid together with those of the track: all those reached from it through paths whose plain
// sleepers would meet, in the order of their names
std::vector<TTrack *> cluster_of( TTrack &Track ) {

    std::vector<TTrack *> result { &Track };
    for( std::size_t index = 0; index < result.size() && result.size() < cluster_cap; ++index ) {
        auto const around { paths_around( *result[ index ] ) };
        for( auto const &path : around ) {
            if( path.track != result[ index ] ) {
                continue;
            }
            for( auto const &other : around ) {
                if( other.track == result[ index ] || std::find( result.begin(), result.end(), other.track ) != result.end() ) {
                    continue;
                }
                if( share_anywhere( path, other ) ) {
                    result.push_back( other.track );
                }
            }
        }
    }
    std::sort( result.begin(), result.end(), []( TTrack const *Left, TTrack const *Right ) {
        return ( Left->name() != Right->name() ? Left->name() < Right->name() : Left < Right ); } );
    return result;
}

std::shared_ptr<cluster_beams> lay_out( std::vector<TTrack *> const &Tracks, std::uint64_t const Epoch ) {

    return cluster_layout( Tracks, Epoch ).lay();
}

// how far the paths of the switch stand off one plane where they share the timbers of its panel, and where along the diverging
// path; the switch itself is out of shape for its timbers where it's more than the tolerance
std::pair<double, double> panel_off_plane( TTrack &Track ) {

    std::pair<double, double> result { 0.0, 0.0 };
    if( Track.eType != tt_Switch ) {
        return result;
    }
    auto const main { sample( Track, 0 ) };
    auto const diverging { sample( Track, 1 ) };
    if( main.points.size() < 2 || diverging.points.size() < 2 ) {
        return result;
    }
    auto alongside { false };
    for( std::size_t index = 0; index < diverging.points.size(); ++index ) {
        double gap;
        glm::dvec2 direction;
        if( false == beside( main, diverging.points[ index ], shared_spacing( diverging, main ), gap, direction ) ) {
            if( alongside ) {
                // apart far enough to lay their own
                break;
            }
            // at the start, which both paths share
            continue;
        }
        alongside = true;
        auto const off { off_plane( diverging, diverging.points[ index ], main ) };
        if( off > result.first ) {
            result = { off, diverging.distances[ index ] };
        }
    }
    return result;
}


} // namespace track_sleepers
