/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/TrackSleepers.h"
#include "world/SleeperFastenings.h"
#include "world/SleeperGeometry.h"
#include "world/SleeperLayout.h"
#include "world/SleeperPaths.h"
#include "world/SleeperRules.h"
#include "world/SleeperWorker.h"
#include "world/Track.h"
#include "world/TrackRails.h"
#include "rendering/renderer.h"
#include "simulation/simulation.h"
#include "scene/scene.h"
#include "utilities/utilities.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <format>
#include <map>
#include <optional>
#include <set>

namespace track_sleepers {

bool bedded( TTrack const &Track ) {

    if( ( Track.iCategoryFlag & 15 ) != 1 || Track.m_editorremoved ) {
        return false;
    }
    switch( Track.eType ) {
        case tt_Normal: {
            if( Track.m_material2 == null_handle || Track.Segment == nullptr ) {
                return false;
            }
            break;
        }
        case tt_Switch: {
            // the switch trackbed takes the ballast of a plain track joined to it, as it's made at the load, the switch set straight.
            // the tracks joined to the main path are asked, not the ends the switch is set to now: those change as it's thrown, also
            // while the beams are worked out in the background
            if( auto const ballast = []( TTrack const *Other ) {
                    return Other != nullptr && Other->eType == tt_Normal && Other->m_material2 != null_handle; };
                false == Global.CreateSwitchTrackbeds || Track.SwitchExtension == nullptr
                || ( Track.SwitchExtension->m_material3 == null_handle && false == ballast( Track.SwitchExtension->pPrevs[ 0 ] )
                     && false == ballast( Track.SwitchExtension->pNexts[ 0 ] ) ) ) {
                return false;
            }
            break;
        }
        default: {
            return false;
        }
    }
    return true;
}

bool lays( TTrack const &Track ) {

    // a renderer which doesn't draw them leaves the tracks their ballast, as they have it without the sleepers; the model of the track
    // itself (timbermodel) is enough, without sleepers3d in eu07.ini
    return GfxRenderer != nullptr && GfxRenderer->Draws_Timbers() && bedded( Track ) && timber_model( Track ) != nullptr;
}

double blade_rise( TTrack const &Track ) {

    return ( Track.eType == tt_Switch ? wing_rise( Track ) : 0.0 );
}

double wing_rise( TTrack const &Track ) {

    return ( lays( Track ) ? timber_model( Track )->slide : 0.0 );
}

glm::dvec2 stock_stretch( TTrack const &Track, int const Path, int const Side ) {

    if( Track.eType != tt_Switch || Track.SwitchExtension == nullptr || Side != ( Track.SwitchExtension->RightSwitch == ( Path == 1 ) ? 1 : -1 ) ) {
        return glm::dvec2 { -1.0 };
    }
    auto const &shape { Track.blade( 1 - Path ) };
    return { std::max( 0.0, ( shape.valid ? shape.tip : 0.0 ) - chair_lead ), track_rails::blade_length( Track, 1 - Path ) };
}

double blade_heel( TTrack const &Track, int const Path, int const Side ) {

    if( Track.eType != tt_Switch || Track.SwitchExtension == nullptr || Side != ( Track.SwitchExtension->RightSwitch == ( Path == 0 ) ? 1 : -1 ) ) {
        return 0.0;
    }
    return track_rails::blade_length( Track, Path );
}

std::vector<std::pair<double, double>> shared_zones( TTrack const &Track, int const Path ) {

    std::vector<std::pair<double, double>> result;
    if( false == bedded( Track ) || Path < 0 || Path >= path_count( Track ) ) {
        return result;
    }
    auto const around { paths_around( Track, true ) };
    auto const found { std::ranges::find_if( around, [ &Track, Path ]( sampled_path const &Sampled ) { return Sampled.track == &Track && Sampled.path == Path; } ) };
    if( found == around.end() ) {
        return result;
    }
    auto const &path { *found };
    // where another path runs so near that the timbers carry both
    for( auto const &stretch : stretches_where( path, [ &around, &path ]( glm::dvec3 const &Point, glm::dvec2 const &Direction ) {
            return std::ranges::any_of( around, [ &path, &Point, &Direction ]( sampled_path const &Other ) {
                return &Other != &path && shares_with( path, Point, Direction, Other ); } ); } ) ) {
        result.emplace_back( stretch.x, stretch.y );
    }
    return result;
}

std::pair<double, double> shared_zone( TTrack const &Track, int const Path ) {

    auto const zones { shared_zones( Track, Path ) };
    if( zones.empty() ) {
        return { 0.0, 0.0 };
    }
    return { zones.front().first, zones.back().second };
}

namespace {

// the instances made so far, of all the tracks
std::uint64_t makings { 0 };

// the lengths along x of the parts of the model a timber is put together from
struct part_lengths {
    double begin { 0.0 }; // of the end on the +x side
    double piece { 0.0 }; // of the piece between the ends, at least a centimetre
    double end { 0.0 }; // of the end on the -x side
};

// a run of beams of the track started at the instances laid so far
void open_run( laid const &Laid, chunk &Run ) {

    for( int part = 0; part < part_count; ++part ) {
        Run.first[ part ] = static_cast<std::uint32_t>( Laid.instances[ part ].size() );
    }
    Run.wholefirst = static_cast<std::uint32_t>( Laid.whole.size() );
    Run.plainfirst = static_cast<std::uint32_t>( Laid.plain.size() );
}

// the run of beams of the track ended at the instances laid so far, by the sphere around the middles and half lengths of its beams
void close_run( laid &Laid, chunk &Run, std::vector<std::pair<glm::dvec3, double>> &Runbeams ) {

    if( Runbeams.empty() ) {
        return;
    }
    glm::dvec3 centre { 0.0 };
    for( auto const &[ middle, half ] : Runbeams ) {
        centre += middle;
    }
    centre /= static_cast<double>( Runbeams.size() );
    auto radius { 0.0 };
    for( auto const &[ middle, half ] : Runbeams ) {
        radius = std::max( radius, glm::length( middle - centre ) + half );
    }
    Run.centre = glm::vec3 { centre };
    Run.radius = static_cast<float>( radius + 0.3 );
    for( int part = 0; part < part_count; ++part ) {
        Run.last[ part ] = static_cast<std::uint32_t>( Laid.instances[ part ].size() );
    }
    Run.wholelast = static_cast<std::uint32_t>( Laid.whole.size() );
    Run.plainlast = static_cast<std::uint32_t>( Laid.plain.size() );
    Laid.chunks.push_back( Run );
    Runbeams.clear();
}

// the instances of the beam in its frame: a plain sleeper whole, a timber stretched whole or put together of the parts of the model.
// a plain sleeper of the model of the track itself lifted as given, at the height it's drawn at without the 3D sleepers; the others
// with their seat at the top of the beam
void put_beam( laid &Laid, beam_model const &Parts, beam const &Beam, glm::mat4 const &Frame, double const Length, part_lengths const &Lengths, std::optional<float> const Plainlift ) {

    // a part stretched along x by the scale, and moved so its x lands at the given place along the beam
    auto const put = [ &Laid, &Parts, &Frame ]( int const Part, double const Scale, double const From, double const Place ) {
        auto const shift { Place - Scale * From };
        Laid.instances[ Part ].emplace_back(
            glm::scale(
                glm::translate( Frame, glm::vec3 { static_cast<float>( shift ), -Parts.seat, 0.F } ),
                glm::vec3 { static_cast<float>( Scale ), 1.F, 1.F } ) );
    };
    auto const half { 0.5 * Length };
    auto const between { Length - Lengths.begin - Lengths.end };
    if( Beam.plain && Laid.plains != nullptr ) {
        // the whole sleeper at its own length, its middle under the middle of the track
        auto const &plain { *Laid.plains };
        Laid.plain.emplace_back(
            glm::translate( Frame, glm::vec3 { -0.5F * ( plain.span.x + plain.span.y ), Plainlift.value_or( -plain.seat ), 0.F } ) );
    }
    else if( false == Parts.stretchable ) {
        // a timber of a model which can't be put together, the whole of it stretched to the length
        auto const scale { Length / static_cast<double>( Parts.length() ) };
        Laid.whole.emplace_back(
            glm::scale(
                glm::translate( Frame, glm::vec3 { static_cast<float>( -scale * 0.5 * ( Parts.span.x + Parts.span.y ) ), -Parts.seat, 0.F } ),
                glm::vec3 { static_cast<float>( scale ), 1.F, 1.F } ) );
    }
    else if( between > 0.01 ) {
        // the ends at their own size, and as many pieces between them as fit closest to their own size
        put( part_end, 1.0, Parts.spans[ part_end ].x, -half );
        put( part_begin, 1.0, Parts.spans[ part_begin ].y, half );
        auto const pieces { std::max( 1, static_cast<int>( std::round( between / Lengths.piece ) ) ) };
        auto const piece { between / pieces };
        for( int index = 0; index < pieces; ++index ) {
            put( part_main, piece / Lengths.piece, Parts.spans[ part_main ].x, -half + Lengths.end + index * piece );
        }
    }
    else {
        // a beam shorter than its ends: just the ends, squeezed
        auto const scale { Length / std::max( 0.01, Lengths.begin + Lengths.end ) };
        put( part_end, scale, Parts.spans[ part_end ].x, -half );
        put( part_begin, scale, Parts.spans[ part_begin ].y, half );
    }
}

// the fastenings of the rails lying on the timber, each square to its rail on the top of the timber, the plate under the foot of the
// rail; the rails around the track found at the first timber
void put_fastenings( TTrack &Track, beam_model const &Parts, beam const &Beam, glm::dvec3 const &Up, std::optional<fastened_rails> &Rails ) {

    if( false == Rails.has_value() ) {
        Rails.emplace( Track );
    }
    auto &laid { Track.m_sleepers3d };
    for( auto const &fastening : Rails->on( Beam, Parts ) ) {
        auto across { glm::dvec3 { fastening.across.x, 0.0, fastening.across.y } };
        across = glm::normalize( across - Up * glm::dot( across, Up ) );
        glm::mat4 frame { 1.f };
        frame[ 0 ] = glm::vec4 { glm::vec3 { across }, 0.f };
        frame[ 1 ] = glm::vec4 { glm::vec3 { Up }, 0.f };
        frame[ 2 ] = glm::vec4 { glm::vec3 { glm::cross( across, Up ) }, 0.f };
        frame[ 3 ] = glm::vec4 { glm::vec3 { fastening.point - Track.m_origin }, 1.f };
        auto const pivot { glm::vec3 { fastening.pivot, Parts.heights[ fastening.part ].x, 0.f } };
        laid.instances[ fastening.part ].emplace_back(
            glm::translate( glm::scale( glm::translate( frame, glm::vec3 { fastening.shift, fastening.lift - Parts.seat, 0.f } + pivot ), glm::vec3 { fastening.scale, fastening.height, 1.f } ), -pivot ) );
    }
}

// the beams as instances of the parts of the model of the track: those of the fastenings with them, or left for later and the beams
// kept for them meanwhile
void instance( TTrack &Track, std::vector<beam> &&Beams, bool const Fastened ) {

    auto &laid { Track.m_sleepers3d };
    laid.made = ++makings;
    for( auto &instances : laid.instances ) {
        instances.clear();
    }
    laid.whole.clear();
    laid.plain.clear();
    laid.chunks.clear();
    laid.fastened = true;
    std::vector<beam> {}.swap( laid.beams );
    if( laid.timbers == nullptr ) {
        return;
    }
    // the sleepers of the model of the track itself (sleepermodel) at the height they're drawn at without the 3D sleepers: the
    // origin of the model at the top of the rails, moved up by the sleeper shift, which the model was fitted to
    std::optional<float> plainlift;
    if( laid.plains != nullptr && Track.m_sleeper_enabled && laid.plains != plain_model() ) {
        plainlift = Track.rail_height() + Track.m_sleeper_offset.z;
    }
    auto const &parts { *laid.timbers };

    // the rails around, where they lie on the timbers, for the fastenings of the model; found at the first timber, none for plain sleepers
    std::optional<fastened_rails> rails;

    auto const partlength = [ &parts ]( int const Part ) {
        return static_cast<double>( parts.spans[ Part ].y - parts.spans[ Part ].x ); };
    part_lengths const lengths {
        .begin = partlength( part_begin ),
        .piece = std::max( 0.01, partlength( part_main ) ),
        .end = partlength( part_end ) };

    chunk run;
    std::vector<std::pair<glm::dvec3, double>> runbeams;
    for( auto const &beam : Beams ) {
        if( runbeams.empty() ) {
            open_run( laid, run );
        }
        auto const length { glm::length( beam.to - beam.from ) };
        if( length < 0.1 ) {
            continue;
        }
        // the frame of the beam: x along it, y up, z along the tracks
        glm::dvec3 const x { ( beam.to - beam.from ) / length };
        auto z { glm::normalize( beam.along - x * glm::dot( beam.along, x ) ) };
        auto y { glm::cross( z, x ) };
        if( y.y < 0.0 ) {
            z = -z;
            y = -y;
        }
        auto const middle { 0.5 * ( beam.from + beam.to ) - Track.m_origin };
        glm::mat4 frame { 1.f };
        frame[ 0 ] = glm::vec4 { glm::vec3 { x }, 0.f };
        frame[ 1 ] = glm::vec4 { glm::vec3 { y }, 0.f };
        frame[ 2 ] = glm::vec4 { glm::vec3 { z }, 0.f };
        frame[ 3 ] = glm::vec4 { glm::vec3 { middle }, 1.f };
        put_beam( laid, parts, beam, frame, length, lengths, plainlift );
        if( Fastened && parts.fastened() && false == ( beam.plain && laid.plains != nullptr ) ) {
            put_fastenings( Track, parts, beam, y, rails );
        }
        runbeams.emplace_back( middle, 0.5 * length );
        if( runbeams.size() >= chunk_beams ) {
            close_run( laid, run, runbeams );
        }
    }
    close_run( laid, run, runbeams );
    // kept for as long as the track is near, without the room the vectors grew into
    for( auto &instances : laid.instances ) {
        instances.shrink_to_fit();
    }
    laid.whole.shrink_to_fit();
    laid.plain.shrink_to_fit();
    laid.chunks.shrink_to_fit();
    if( false == Fastened && parts.fastened() ) {
        laid.fastened = false;
        laid.beams = std::move( Beams );
    }
}

} // namespace

void lay( TTrack &Track, bool const Fastened ) {

    auto &laid { Track.m_sleepers3d };
    std::vector<beam> placed;
    if( laid.active && in_background() && timber_model( Track ) != nullptr ) {
        // worked out in the background; till then the track keeps what it has, and tries again
        auto beams { laid_out( Track ) };
        if( false == beams.has_value() ) {
            ask_layout( Track );
            return;
        }
        placed = std::move( *beams );
    }
    laid.dirty = false;
    laid.timbers = timber_model( Track );
    laid.plains = plain_model( Track );
    if( false == laid.active || laid.timbers == nullptr ) {
        instance( Track, {}, true );
        return;
    }
    if( auto const [ off, at ] { panel_off_plane( Track ) }; off > coplanar_tolerance ) {
        WriteLog( std::format(
            "sleepers3d: switch \"{}\" has its paths {} mm off one plane {} m along, where they share the timbers of its panel",
            Track.name(), to_string( off * 1000.0, 1 ), to_string( at, 2 ) ) );
    }
    laid.laying = true;
    auto beams { in_background() ? std::move( placed ) : place( Track ) };
    laid.laying = false;
    laid.timbers = timber_model( Track, beams );
    // the layout of the cluster isn't needed by this track any more, the instances below are what's drawn
    forget_layout( Track );
    instance( Track, std::move( beams ), Fastened );
}

void fasten( TTrack &Track ) {

    auto &laid { Track.m_sleepers3d };
    if( laid.fastened || laid.dirty ) {
        return;
    }
    auto beams { std::move( laid.beams ) };
    instance( Track, std::move( beams ), true );
}

void release_far( glm::dvec3 const &Camera, double const Range ) {

    // the tracks swept in slices, so a big scenery costs little in a frame
    static std::size_t next { 0 };
    auto const &tracks { simulation::Paths.sequence() };
    auto const slice { std::min<std::size_t>( tracks.size(), 256 ) };
    for( std::size_t count = 0; count < slice; ++count ) {
        if( next >= tracks.size() ) {
            next = 0;
        }
        auto *track { tracks[ next++ ] };
        if( track == nullptr || track->m_sleepers3d.laying ) {
            continue;
        }
        if( glm::length( track->location() - Camera ) - track->radius() <= Range ) {
            continue;
        }
        forget_rails( *track );
        if( false == track->m_sleepers3d.active ) {
            continue;
        }
        auto &laid { track->m_sleepers3d };
        forget_layout( *track );
        auto const holds { laid.chunks.capacity() > 0 || laid.whole.capacity() > 0 || laid.plain.capacity() > 0 || laid.beams.capacity() > 0
            || std::ranges::any_of( laid.instances, []( std::vector<glm::mat4> const &Instances ) { return Instances.capacity() > 0; } ) };
        if( false == holds ) {
            continue;
        }
        for( auto &instances : laid.instances ) {
            std::vector<glm::mat4> {}.swap( instances );
        }
        std::vector<glm::mat4> {}.swap( laid.whole );
        std::vector<glm::mat4> {}.swap( laid.plain );
        std::vector<chunk> {}.swap( laid.chunks );
        std::vector<beam> {}.swap( laid.beams );
        laid.dirty = true;
    }
}

namespace {

// the digest of where the beams lie taken on by the beam: its ends, which way it lies and whether it's a plain sleeper
void digest_beam( std::uint64_t &Digest, beam const &Beam ) {

    for( auto const &vector : { Beam.from, Beam.to, Beam.along } ) {
        for( int axis = 0; axis < 3; ++axis ) {
            Digest = ( Digest ^ static_cast<std::uint64_t>( std::llround( vector[ axis ] * 1e4 ) ) ) * 1099511628211ULL;
        }
    }
    Digest = ( Digest ^ ( Beam.plain ? 1U : 2U ) ) * 1099511628211ULL;
}

// the beams of the track placed anew, as boxes in plan with the track they're laid with, counted into what's wrong with the layout
void box_beams( TTrack const &Track, std::vector<beam_box> &Boxes, std::vector<TTrack const *> &Owners, layout_check &Result ) {

    if( auto const [ off, at ] { panel_off_plane( Track ) }; off > coplanar_tolerance ) {
        ++Result.panels;
        if( Result.panels <= 4 ) {
            Result.examples.push_back( std::format( "switch {} {} mm off one plane at {} m", Track.name(), to_string( off * 1000.0, 1 ), to_string( at, 2 ) ) );
        }
    }
    for( auto const &beam : place( Track ) ) {
        digest_beam( Result.digest, beam );
        auto const span { plan( beam.to - beam.from ) };
        auto const length { glm::length( span ) };
        if( length < 0.1 ) {
            continue;
        }
        Boxes.push_back( box_of( beam ) );
        Owners.push_back( &Track );
        Result.longest = std::max( Result.longest, length );
        if( length > length_cap ) {
            ++Result.toolong;
        }
    }
}

// the beams of a cell of the grid which lie over each other, each pair of them counted once
void find_overlaps( std::vector<std::size_t> const &Indices, std::vector<beam_box> const &Boxes, std::vector<TTrack const *> const &Owners, std::set<std::pair<std::size_t, std::size_t>> &Overlaps, layout_check &Result ) {

    for( std::size_t one = 0; one < Indices.size(); ++one ) {
        for( std::size_t other = one + 1; other < Indices.size(); ++other ) {
            auto const pair { std::minmax( Indices[ one ], Indices[ other ] ) };
            if( Overlaps.contains( pair ) || apart( Boxes[ pair.first ], Boxes[ pair.second ] ) ) {
                continue;
            }
            Overlaps.insert( pair );
            if( Overlaps.size() <= 16 ) {
                auto const &box { Boxes[ pair.first ] };
                Result.examples.push_back( std::format(
                    "overlap {}/{} at {} {}", Owners[ pair.first ]->name(), Owners[ pair.second ]->name(), to_string( box.centre.x, 2 ), to_string( box.centre.y, 2 ) ) );
            }
        }
    }
}

// the spacing of the beams under a rail at the distance along the path, kept where it's the widest, counted where it's too wide
void note_spacing( TTrack const &Track, int const Path, double const Spacing, double const Distance, layout_check &Result ) {

    if( Spacing > Result.widest ) {
        Result.widest = Spacing;
        Result.widestat = std::format( "{} {}", Track.name(), to_string( Distance, 2 ) );
    }
    if( Spacing > 1.25 * 1.08 * pitch ) {
        ++Result.gaps;
        if( Result.examples.size() < 24 ) {
            Result.examples.push_back( std::format( "gap {} m under {} path {} at {}", to_string( Spacing, 2 ), Track.name(), Path, to_string( Distance, 2 ) ) );
        }
    }
}

// the rails of the path of the track followed for how far apart the beams under them are
void follow_rails( TTrack const &Track, int const Path, TSegment const &Segment, plan_grid<std::size_t> const &Grid, std::vector<beam_box> const &Boxes, layout_check &Result ) {

    auto const length { Segment.GetLength() };
    auto const steps { static_cast<int>( std::ceil( length / 0.05 ) ) };
    for( auto const side : { -1.0, 1.0 } ) {
        // from the first beam under the rail to the last, how long it runs with none under it
        auto last { -1.0 };
        auto first { -1.0 };
        for( int step = 0; step <= steps; ++step ) {
            auto const distance { length * step / std::max( 1, steps ) };
            glm::dvec3 point;
            glm::dvec3 across;
            Segment.plan_frame( distance, point, across );
            auto const rail { plan( point + across * ( side * 0.5 * std::abs( Track.fTrackWidth ) ) ) };
            if( auto const *cell { Grid.at( rail ) };
                cell == nullptr || std::ranges::none_of( *cell, [ &Boxes, &rail ]( std::size_t const Index ) { return holds( Boxes[ Index ], rail ); } ) ) {
                continue;
            }
            if( last >= 0.0 && distance - last > 0.06 ) {
                // the spacing of the beams, middle to middle, by the stretch without one
                auto const spacing { distance - last + 2.0 * half_width };
                note_spacing( Track, Path, spacing, distance, Result );
            }
            if( first < 0.0 ) {
                first = distance;
            }
            last = distance;
        }
        if( first < 0.0 ) {
            ++Result.bare;
            if( Result.examples.size() < 80 ) {
                Result.examples.push_back( std::format( "no beams under {} path {}", Track.name(), Path ) );
            }
        }
    }
}

} // namespace

layout_check check() {

    layout_check result;
    result.digest = 1469598103934665603ULL;
    settle_background();
    faults().reset();
    // laid anew, for the counts of what's wrong with them
    outdate_layouts();
    // the beams of all the tracks in plan, as boxes, with the grid of where they are
    std::vector<beam_box> boxes;
    std::vector<TTrack const *> owners;
    std::vector<TTrack const *> tracks;
    for( auto const *track : simulation::Paths.sequence() ) {
        if( track == nullptr || track->m_editorremoved || false == track->m_sleepers3d.active ) {
            continue;
        }
        tracks.push_back( track );
        box_beams( *track, boxes, owners, result );
    }
    result.beams = boxes.size();
    result.offplane = faults().offplane;
    result.clipped = faults().clipped;
    result.dropped = faults().dropped;
    result.slowest = faults().slowest;
    result.biggest = faults().biggest;
    // each box in every cell its span touches
    plan_grid<std::size_t> grid { 3.0 };
    for( std::size_t index = 0; index < boxes.size(); ++index ) {
        grid.insert( boxes[ index ].centre, boxes[ index ].half + half_width, index );
    }
    std::set<std::pair<std::size_t, std::size_t>> overlaps;
    for( auto const &[ coordinates, indices ] : grid.cells() ) {
        find_overlaps( indices, boxes, owners, overlaps, result );
    }
    result.overlaps = overlaps.size();
    // the rails of the tracks, followed for how far apart the beams under them are
    for( auto const *track : tracks ) {
        for( int path = 0; path < path_count( *track ); ++path ) {
            auto const *segment { path_segment( *track, path ) };
            if( segment == nullptr ) {
                continue;
            }
            follow_rails( *track, path, *segment, grid, boxes, result );
        }
    }
    return result;
}

void unsettle_around( TTrack const &Track ) {

    if( GfxRenderer == nullptr || false == GfxRenderer->Draws_Timbers() ) {
        // nothing laid, nothing to lay again; without sleepers3d in eu07.ini the tracks with a timbermodel of their own lay still
        return;
    }
    outdate_layouts();
    tracks_near( Track, []( TTrack *Other ) {
        if( Other->m_sleepers3d.active ) {
            Other->m_sleepers3d.dirty = true;
        } } );
}

} // namespace track_sleepers
