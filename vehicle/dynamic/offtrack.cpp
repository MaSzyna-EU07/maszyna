/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others

*/

// a derailed vehicle off the track, on the ground

#include "stdafx.h"
#include "vehicle/DynObj.h"
#include "physics/mover.h"
#include "vehicle/lightsprogrammer.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "rendering/lightarray.h"
#include "vehicle/Camera.h"
#include "vehicle/Train.h"
#include "vehicle/odometer.h"
#include "vehicle/pulleemode.h"
#include "vehicle/Driver.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <random>
#include <iterator>
#include <numbers>
#include <optional>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "utilities/workerpool.h"
#include "utilities/Logs.h"
#include "utilities/glmHelpers.h"
#include "Console.h"
#include "world/Traction.h"
#include "audio/sound.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "vehicle/doorwings.h"
#include "rendering/renderer.h"
#include "editor/editorTerrainStreamer.hpp"
#include "application/uitranscripts.h"
#include "input/messaging.h"
#include "application/application.h"
#include "vehicle/Driver.h"
#include "vehicle/dynamic/internal.h"

namespace
{


using ground_triangles = std::vector<derail_motion::mesh_ground::triangle>;

} // namespace

bool outside( glm::dvec3 const &Centre, double const Radius, glm::dvec2 const &Min, glm::dvec2 const &Max ) {
    return Centre.x + Radius < Min.x || Centre.x - Radius > Max.x || Centre.z + Radius < Min.y || Centre.z - Radius > Max.y;
}
namespace
{

void add_triangle( glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C, glm::dvec2 const &Min, glm::dvec2 const &Max, ground_triangles &Out ) {
    if( std::max( { A.x, B.x, C.x } ) < Min.x || std::min( { A.x, B.x, C.x } ) > Max.x
     || std::max( { A.z, B.z, C.z } ) < Min.y || std::min( { A.z, B.z, C.z } ) > Max.y ) {
        return;
    }
    Out.push_back( { A, B, C } );
}

// the triangles of a piece of the scenery's geometry within the rectangle (x, z), as the editor gathers the ground
void add_shape( scene::shape_node const &Shape, glm::dvec2 const &Min, glm::dvec2 const &Max, ground_triangles &Out ) {
    auto const &data { Shape.data() };
    if( data.area.radius >= 0.f && outside( data.area.center, data.area.radius, Min, Max ) ) {
        return;
    }
    if( false == data.vertices.empty() ) {
        for( std::size_t i = 0; i + 2 < data.vertices.size(); i += 3 ) {
            add_triangle( data.vertices[ i ].position, data.vertices[ i + 1 ].position, data.vertices[ i + 2 ].position, Min, Max, Out );
        }
        return;
    }
    // once the shape is uploaded its world-space source is released; the renderer keeps an origin-relative copy
    if( data.geometry.bank == 0 && data.geometry.chunk == 0 ) {
        return;
    }
    auto const &vertices { GfxRenderer->Vertices( data.geometry ) };
    auto const &indices { GfxRenderer->Indices( data.geometry ) };
    auto const point = [ &data, &vertices ]( std::size_t const Index ) { return data.origin + glm::dvec3 { vertices[ Index ].position }; };
    if( indices.empty() ) {
        for( std::size_t i = 0; i + 2 < vertices.size(); i += 3 ) {
            add_triangle( point( i ), point( i + 1 ), point( i + 2 ), Min, Max, Out );
        }
        return;
    }
    for( std::size_t i = 0; i + 2 < indices.size(); i += 3 ) {
        if( std::max( { indices[ i ], indices[ i + 1 ], indices[ i + 2 ] } ) >= vertices.size() ) {
            continue;
        }
        add_triangle( point( indices[ i ] ), point( indices[ i + 1 ] ), point( indices[ i + 2 ] ), Min, Max, Out );
    }
}

// the ground of the scenery within the rectangle (x, z): its terrain and the other opaque geometry of its sections,
// and the heightmap terrain where it is loaded. the models aren't ground
ground_triangles scenery_ground( glm::dvec2 const &Min, glm::dvec2 const &Max ) {
    ground_triangles triangles;
    if( simulation::Region == nullptr ) {
        return triangles;
    }
    auto const centre { 0.5 * ( Min + Max ) };
    std::vector<scene::basic_section *> sections;
    simulation::Region->sections( glm::dvec3 { centre.x, 0.0, centre.y }, static_cast<float>( 0.5 * glm::length( Max - Min ) ), sections );
    for( auto const *section : sections ) {
        for( auto const &shape : section->m_shapes ) {
            add_shape( shape, Min, Max, triangles );
        }
        for( auto const &cell : section->m_cells ) {
            if( outside( cell.m_area.center, cell.m_area.radius, Min, Max ) ) {
                continue;
            }
            for( auto const &shape : cell.m_shapesopaque ) {
                add_shape( shape, Min, Max, triangles );
            }
        }
    }
    if( EditorTerrain.active() ) {
        EditorTerrain.gather_triangles( Min, Max, triangles );
    }
    return triangles;
}

glm::dvec3 level_left_of( glm::dvec3 const &Front ) {
    return glm::dvec3 { Front.z, 0.0, -Front.x };
}

// the end of the vehicle where it meets the one coupled to it: the middle of its buffer beam, the way it faces. off the
// track, over its base on the ground as it stands: rolled over, its ends go with it as far as the coupling holds them,
// and the buffers of the side it rolled to miss those of an upright one
derail_motion::end_point end_point_of( TDynamicObject const &Vehicle, int const End ) {
    auto const facing { End == end::front ? 1.0 : -1.0 };
    auto const &motion { Vehicle.MoverParameters->OffTrack };
    glm::dvec3 const front { Vehicle.VectorFront().x, 0.0, Vehicle.VectorFront().z };
    auto const length { glm::length( front ) };
    auto const axis { length > 1e-6 ? front / length : glm::dvec3 { 0.0, 0.0, 1.0 } };
    auto const middle { motion.active() ? motion.base() : Vehicle.GetPosition() };
    return derail_motion::end_point {
        middle + glm::dvec3 { 0.0, derail_motion::buffer_height, 0.0 } + axis * ( facing * 0.5 * Vehicle.MoverParameters->Dim.L ),
        axis * facing,
        motion.active() ? motion.tipping() : 0.0 };
}

// the push of the buffers on the end of the vehicle [N], negative the pull of its coupling
double compression_of( TMoverParameters const &Vehicle, int const End ) {
    return ( End == end::front ? -1.0 : 1.0 ) * Vehicle.Couplers[ End ].CForce;
}

} // namespace

double seconds_since( off_track_clock::time_point const Start ) {
    return std::chrono::duration<double>( off_track_clock::now() - Start ).count();
}
namespace
{

std::string_view side_name( int const Side ) {
    if( Side > 0 ) {
        return "left";
    }
    return Side < 0 ? "right" : "none";
}

} // namespace

double TDynamicObject::off_track_gap( int const End ) const {

    auto const *neighbour { MoverParameters->Neighbours[ End ].vehicle };
    if( neighbour == nullptr ) {
        return 0.0;
    }
    return derail_motion::meet( end_point_of( *this, End ), end_point_of( *neighbour, MoverParameters->Neighbours[ End ].vehicle_end ) ).gap;
}

double TDynamicObject::off_track_offset() const {

    if( false == MoverParameters->OffTrack.active() ) {
        return 0.0;
    }
    auto const middle { 0.5 * ( Axle0.pPosition + Axle1.pPosition ) };
    auto const offset { vPosition - middle };
    return glm::dot( glm::dvec3 { offset.x, 0.0, offset.z }, level_left_of( track_front() ) );
}

glm::dvec3 TDynamicObject::track_front() const {

    auto const along { Axle0.pPosition - Axle1.pPosition };
    glm::dvec3 const flat { along.x, 0.0, along.z };
    auto const length { glm::length( flat ) };
    return length > 1e-6 ? flat / length : glm::dvec3 { 0.0, 0.0, 1.0 };
}

derail_motion::formation TDynamicObject::track_bed() const {

    derail_motion::formation bed;
    bed.point = 0.5 * ( Axle0.pPosition + Axle1.pPosition );
    bed.direction = track_front();
    if( auto const *track { RaTrackGet() }; track != nullptr ) {
        // the ballast out of the rails, as wide as the track draws it
        bed.half_width = std::clamp( 0.5 * std::abs( track->fTrackWidth ) + std::abs( track->fTexWidth ), 1.2, 3.5 );
    }
    return bed;
}

int TDynamicObject::outward_side() const {

    // the track as designed turns from the rear axle to the front one: the outside of its curve, or its inside where a
    // buckle bends the line the other way more than the curve does (the criteria of derailment, physics/derailment.h)
    if( auto const design { std::sin( Axle1.vAngles.z - Axle0.vAngles.z ) }; std::abs( design ) >= 1e-4 ) {
        auto const outside { design > 0.0 ? -1 : 1 };
        return MoverParameters->DerailAcceleration < 0.0 ? -outside : outside;
    }
    // a straight track: the path under the vehicle as it is, a buckle on it, turns from the rear axle to the front one
    auto const front { Axle0.PathDirection() };
    auto const rear { Axle1.PathDirection() };
    auto const turn { glm::dot( front, level_left_of( rear ) ) };
    // a turn under a tenth of a milliradian is a straight track
    if( std::abs( turn ) < 1e-4 ) {
        return 0;
    }
    // turning left, the outside of the curve is on the right
    return turn > 0.0 ? -1 : 1;
}

void TDynamicObject::gather_off_track_terrain() {

    auto const &mover { *MoverParameters };
    auto const &position { mover.OffTrack.active() ? mover.OffTrack.current().position : vPosition };
    // as far as it may slide, and some
    auto const slide { derail_motion::stopping_distance( mover.V, derailment::off_rails_friction ).value_or( 0.0 ) };
    auto const reach { std::min( slide + mover.Dim.L + 50.0, 600.0 ) };
    m_offtrackgathered = m_offtracktotal;
    m_offtrackcropped = false;
    // the terrain of another vehicle, where it covers the way this one goes: around it and around where it stops going
    // straight on, by its length and the margin the terrain is gathered again at
    glm::dvec3 heading { vFront.x, 0.0, vFront.z };
    if( mover.OffTrack.active() ) {
        heading = mover.OffTrack.heading();
    }
    auto const length { glm::length( heading ) };
    auto const stop { length > 1e-6 ? position + heading * ( std::copysign( std::max( reach - mover.Dim.L - 50.0, 0.0 ), mover.V ) / length ) : position };
    auto const margin { mover.Dim.L + 30.0 };
    if( auto shared { simulation::Vehicles.off_track_terrain(
            glm::dvec2 { std::min( position.x, stop.x ) - margin, std::min( position.z, stop.z ) - margin },
            glm::dvec2 { std::max( position.x, stop.x ) + margin, std::max( position.z, stop.z ) + margin } ) };
        shared != nullptr ) {
        m_offtrackterrain = std::move( shared );
        m_offtrackmin = m_offtrackterrain->minimum();
        m_offtrackmax = m_offtrackterrain->maximum();
        m_offtracksite.terrain( m_offtrackterrain.get() );
        return;
    }
    m_offtrackmin = glm::dvec2 { position.x - reach, position.z - reach };
    m_offtrackmax = glm::dvec2 { position.x + reach, position.z + reach };
    auto const started { off_track_clock::now() };
    auto triangles { scenery_ground( m_offtrackmin, m_offtrackmax ) };
    auto const trimmed { triangles.size() > off_track_triangles };
    if( trimmed ) {
        // a scenery dense enough to make it slow: the triangles nearest to it are enough
        std::nth_element( triangles.begin(), triangles.begin() + off_track_triangles, triangles.end(),
            [ &position ]( auto const &Left, auto const &Right ) {
                auto const distance = [ &position ]( auto const &Triangle ) {
                    auto const middle { ( Triangle[ 0 ] + Triangle[ 1 ] + Triangle[ 2 ] ) / 3.0 };
                    return std::max( std::abs( middle.x - position.x ), std::abs( middle.z - position.z ) ); };
                return distance( Left ) < distance( Right ); } );
        triangles.resize( off_track_triangles );
        ErrorLog( std::format( "derailment: warning: {} found more than {} triangles of terrain around it, took the nearest", MoverParameters->Name, off_track_triangles ) );
    }
    if( triangles.empty() ) {
        // a scenery without terrain: level with the top of the ballast
        m_offtrackterrain.reset();
    }
    else {
        m_offtrackterrain = std::make_shared<derail_motion::mesh_ground const>( triangles, m_offtrackmin, m_offtrackmax );
        // the vehicles sliding over it take it as it is; the nearest triangles of a dense scenery are its own
        if( false == trimmed ) {
            simulation::Vehicles.share_off_track_terrain( m_offtrackterrain );
        }
    }
    m_offtracksite.terrain( m_offtrackterrain.get() );
    auto const took { seconds_since( started ) };
    if( took > off_track_slow ) {
        ErrorLog( std::format( "derailment: warning: gathering the terrain of {:.0f} m around {} took {:.0f} ms ({} triangles)", reach, MoverParameters->Name, took * 1000.0, triangles.size() ) );
    }
}

void TDynamicObject::crop_off_track_terrain() {

    if( m_offtrackcropped || m_offtrackterrain == nullptr ) {
        return;
    }
    m_offtrackcropped = true;
    // the ground under it, and as far around as it gathers the terrain anew once it moves again
    auto const &position { MoverParameters->OffTrack.current().position };
    auto const reach { 0.5 * MoverParameters->Dim.L + 40.0 };
    m_offtrackmin = glm::dvec2 { position.x - reach, position.z - reach };
    m_offtrackmax = glm::dvec2 { position.x + reach, position.z + reach };
    m_offtrackterrain = std::make_shared<derail_motion::mesh_ground const>( m_offtrackterrain->cropped( m_offtrackmin, m_offtrackmax ) );
    m_offtracksite.terrain( m_offtrackterrain.get() );
}

void TDynamicObject::begin_off_track() {

    auto &mover { *MoverParameters };
    derail_motion::vehicle_shape shape;
    shape.length = mover.Dim.L;
    shape.width = mover.Dim.W > 0.5 ? mover.Dim.W : 3.0;
    // a height up to 2 m in the definition is the height of the floor, the older meaning of the key
    shape.height = mover.Dim.H > 2.0 ? mover.Dim.H : 4.0;
    shape.centre_of_gravity = mover.centre_of_gravity();
    shape.contact_spacing = derailment::contact_spacing( mover.TrackW );
    shape.base = std::max( static_cast<double>( fAxleDist ), 1.0 );
    shape.mass = mover.TotalMass;

    derail_motion::start start;
    start.position = vPosition;
    start.front = vFront;
    start.line = track_front();
    start.speed = mover.V;
    start.reason = mover.DerailCause;
    start.outward = ( mover.DerailSide != 0 ? mover.DerailSide : outward_side() );
    start.acceleration = mover.DerailAcceleration;

    // the terrain as far as it may slide, and some
    m_offtracktotal = 0.0;
    gather_off_track_terrain();
    m_offtracksite = derail_motion::site { m_offtrackterrain.get(), track_bed() };
    // each vehicle its own way of going off, from the seed of the run (the same seed, the same way)
    auto const scatter { derail_motion::draw_scatter( derail_motion::seed_of( Global.random_seed, mover.Name ) ) };
    mover.OffTrack.begin( start, shape, m_offtracksite, scatter );
    m_pulledoff = {};
    m_offtracklogdue = 0.0;
    m_offtrackslowdue = 0.0;
    m_offtrackaxles = 0.0;
    m_offtrackfrom = mover.OffTrack.current().position;

    auto const heading { mover.OffTrack.heading() };
    auto const front { glm::dvec3 { vFront.x, 0.0, vFront.z } };
    auto const turn { std::asin( std::clamp( glm::dot( heading, level_left_of( glm::normalize( front ) ) ), -1.0, 1.0 ) ) };
    std::string terrain { "no terrain, level ground" };
    if( m_offtrackterrain != nullptr ) {
        terrain = std::format( "{} triangles of terrain", m_offtrackterrain->size() );
    }
    WriteLog( std::format(
        "derailment: {} leaves the track at {:.0f} km/h on track {}, heading turned {:.1f} deg to the {}, the outside of the curve to the {}; {} around it",
        mover.Name, std::abs( mover.V ) * 3.6, MyTrack != nullptr ? MyTrack->name() : std::string { "(none)" },
        std::abs( glm::degrees( turn ) ), side_name( turn >= 0.0 ? 1 : -1 ), side_name( start.outward ), terrain ) );
    WriteLog( std::format(
        "derailment: {} scattered by {:+.2f} deg of departure, {:+.2f} deg of yaw at {:+.3f} rad/s, the friction under its ends x{:.2f} / x{:.2f}{}",
        mover.Name, glm::degrees( scatter.departure ), glm::degrees( scatter.yaw ), scatter.yaw_rate, scatter.friction_front, scatter.friction_rear,
        scatter.dig_in >= 0.0 ? std::format( ", its leading end digs in after {:.2f} s, x{:.2f} for {:.2f} s", scatter.dig_in, scatter.dig_in_friction, scatter.dig_in_time ) : std::string {} ) );
}

void TDynamicObject::move_off_track( double const Distance ) {

    auto const started { off_track_clock::now() };
    auto &motion { MoverParameters->OffTrack };
    auto const time { std::exchange( m_offtracktime, 0.0 ) };
    m_offtracktotal += time;
    // near the edge of the terrain gathered, the terrain further on; not more than once a second, whatever happens
    if( m_offtrackterrain != nullptr && Distance != 0.0 && m_offtracktotal - m_offtrackgathered >= 1.0 ) {
        auto const &position { motion.current().position };
        constexpr double margin { 30.0 };
        if( position.x - margin < m_offtrackmin.x || position.x + margin > m_offtrackmax.x
         || position.z - margin < m_offtrackmin.y || position.z + margin > m_offtrackmax.y ) {
            gather_off_track_terrain();
        }
    }
    m_offtracksite.bed( track_bed() );
    // the buffers and the couplings of its ends push and pull it, at an angle to it where the vehicle coupled there is
    std::array<derail_motion::end_force, 2> ends {};
    for( int end = end::front; end <= end::rear; ++end ) {
        auto const *neighbour { MoverParameters->Neighbours[ end ].vehicle };
        auto const compression { compression_of( *MoverParameters, end ) };
        if( neighbour == nullptr || compression == 0.0 ) {
            continue;
        }
        ends[ end ] = derail_motion::force_on(
            derail_motion::meet( end_point_of( *this, end ), end_point_of( *neighbour, MoverParameters->Neighbours[ end ].vehicle_end ) ), compression );
    }
    auto const reversals { motion.reversals() };
    motion.step( time, Distance, MoverParameters->V, m_offtracksite, ends );
    if( ( motion.reversals() - reversals ) % 2 != 0 ) {
        // the body turned about against the way it goes: the vehicle goes rear first, its speed the other way along it
        MoverParameters->V = -MoverParameters->V;
    }
    auto const &pose { motion.current() };
    // the axles go along the track as far as the vehicle went along it, for the track nearest to it: the vehicles on it
    // find it there, the events of the track are passed. by 10 cm at least: a body turning or rocking in place moves
    // the axles back and forth by millimetres, which over the end of a track would fire its events every frame. from
    // where it was the last time, so the moves out of the other bodies between the frames go into the axles too
    m_offtrackaxles += glm::dot( pose.position - std::exchange( m_offtrackfrom, pose.position ), track_front() );
    if( std::abs( m_offtrackaxles ) >= 0.1 ) {
        move_axles( std::exchange( m_offtrackaxles, 0.0 ) );
    }

    place_off_track();
    if( motion.at_rest() ) {
        crop_off_track_terrain();
    }

    pull_neighbours( Distance );

    // what it cost: a frame of the vehicle off the track should take well under a millisecond
    auto const took { seconds_since( started ) };
    m_offtrackslowdue -= time;
    if( took > off_track_slow && m_offtrackslowdue <= 0.0 ) {
        m_offtrackslowdue = 1.0;
        ErrorLog( std::format( "derailment: warning: moving {} off the track took {:.0f} ms in a frame of {:.3f} s", MoverParameters->Name, took * 1000.0, time ) );
    }
}

void TDynamicObject::place_off_track() {

    auto const &pose { MoverParameters->OffTrack.current() };
    vPosition = pose.position;
    vFront = pose.front;
    vLeft = pose.left;
    vUp = pose.up;
    modelRot.z = std::atan2( -vFront.x, vFront.z );
    mMatrix = glm::inverse( BasisChange( vLeft, vUp, vFront ) );
    auto const half { 0.5 * MoverParameters->Dim.L * vFront };
    vCoulpler[ end::front ] = vPosition + half;
    vCoulpler[ end::rear ] = vPosition - half;
    m_offtrackplaced = true;
}

void TDynamicObject::pull_neighbours( double const Distance ) {

    for( int end = end::front; end <= end::rear; ++end ) {
        auto const &coupler { MoverParameters->Couplers[ end ] };
        auto *neighbour { MoverParameters->Neighbours[ end ].vehicle };
        if( coupler.CouplingFlag == Coupling::none || neighbour == nullptr || neighbour->MoverParameters->OffTrack.active()
         || neighbour->MoverParameters->DamageFlag.HasAll(VehicleDamage::derailed) ) {
            m_pulledoff[ end ].reset();
            continue;
        }
        auto &other { *neighbour->MoverParameters };
        auto const otherend { MoverParameters->Neighbours[ end ].vehicle_end };
        glm::dvec3 const otherfront { neighbour->vFront.x, 0.0, neighbour->vFront.z };
        if( glm::length( otherfront ) < 1e-6 ) {
            continue;
        }
        // the coupling pulls the end of the other vehicle towards the beam of this one, the buffers push it along the
        // bisector of the bend between them: at an angle to its axis, the part across it on its leading wheels
        auto const compression { compression_of( other, otherend ) };
        auto const force { derail_motion::force_on( derail_motion::meet( end_point_of( *neighbour, otherend ), end_point_of( *this, end ) ), compression ).force };
        auto const axis { glm::normalize( otherfront ) };
        auto const across { glm::dot( force, level_left_of( axis ) ) };
        auto const angle { std::atan2( std::abs( across ), std::abs( glm::dot( force, axis ) ) ) };
        auto const magnitude { glm::length( glm::dvec3 { force.x, 0.0, force.z } ) };
        auto const mass { other.TotalMass - other.Mred };
        auto const ratio { derail_motion::pull_ratio( magnitude, angle, mass, other.NAxles ) };
        if( false == m_pulledoff[ end ].update( ratio >= derailment::climbing_limit, Distance ) ) {
            continue;
        }
        // the way the force across it goes: towards this one pulled, away from it pushed
        auto const side { across >= 0.0 ? 1 : -1 };
        other.pull_off(
            side,
            mass > 0.0 ? std::abs( across ) / mass : 0.0,
            std::format( "Y/Q {:.2f} from the {} of {} at {:.0f} deg, {:.0f} kN", ratio, compression > 0.0 ? "buffers" : "coupling", MoverParameters->Name, glm::degrees( angle ), magnitude / 1000.0 ) );
        m_pulledoff[ end ].reset();
    }
}

void TDynamicObject::update_off_track( double const Deltatime ) {

    auto const &motion { MoverParameters->OffTrack };
    if( false == motion.active() || m_offtracklogdue < 0.0 ) {
        // on the track, or came to rest and told so
        return;
    }
    m_offtracklogdue -= Deltatime;
    auto const resting { motion.at_rest() };
    if( m_offtracklogdue > 0.0 && false == resting ) {
        return;
    }
    m_offtracklogdue = ( resting ? -1.0 : 1.0 );
    WriteLog( std::format(
        "derailment: {} {} {:.1f} m from where it left the rails, {:.1f} m from the axis of the track, at {:.1f} km/h, sliding sideways at {:.1f} m/s, tipped {:.0f} deg, roll {:.0f} deg, yaw {:+.1f} deg turning at {:+.1f} deg/s{}",
        MoverParameters->Name, resting ? "came to rest" : "off the track", motion.travelled(), off_track_offset(), std::abs( MoverParameters->V ) * 3.6, motion.sideways(),
        glm::degrees( motion.tipping() ), glm::degrees( motion.roll() ), glm::degrees( motion.yaw() ), glm::degrees( motion.yaw_rate() ), motion.lying() ? ", lying on its side" : "" ) );
}
