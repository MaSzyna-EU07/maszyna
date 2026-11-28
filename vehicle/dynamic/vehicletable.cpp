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

// all the vehicles of the scenery and their update

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


// a train on the same track this close to the end of the player's train is taken into the physical coupler model [m]
constexpr double coupler_physics_reach { 200.0 };
// and let go of once none of its vehicles is this close to the player's train [m]; farther than the reach, so that
// the trains don't switch to and fro between the models
constexpr double coupler_physics_release { 500.0 };

// the vehicle and those coupled with it, one after the other
std::vector<TDynamicObject *>
coupled_train( TDynamicObject *Vehicle ) {

    std::vector<TDynamicObject *> train { Vehicle };
    for( int direction = end::front; direction <= end::rear; ++direction ) {
        auto *vehicle { Vehicle };
        auto side { direction };
        // a train of more vehicles than are on the tracks would be one coupled into a ring
        while( train.size() < 10000 ) {
            auto const &mover { *vehicle->MoverParameters };
            auto *next { mover.Neighbours[ side ].vehicle };
            if( next == nullptr
             || false == mover.Couplers[ side ].CouplingFlag.HasAny(Coupling::coupler)
             || std::ranges::find( train, next ) != train.end() ) {
                break;
            }
            train.emplace_back( next );
            // onwards from the end of the next vehicle away from this one
            side = 1 - mover.Neighbours[ side ].vehicle_end;
            vehicle = next;
        }
    }
    return train;
}

// the trains on the same track the player's train is closing in on: those found at its free ends, near enough
std::vector<TDynamicObject *>
trains_closing_in( std::vector<TDynamicObject *> const &Train ) {

    std::vector<TDynamicObject *> trains;
    for( auto *vehicle : Train ) {
        for( int side = end::front; side <= end::rear; ++side ) {
            auto const &mover { *vehicle->MoverParameters };
            auto const &neighbour { mover.Neighbours[ side ] };
            if( neighbour.vehicle == nullptr
             || mover.Couplers[ side ].CouplingFlag.HasAny(Coupling::coupler)
             || neighbour.distance > coupler_physics_reach
             || std::ranges::find( Train, neighbour.vehicle ) != Train.end() ) {
                continue;
            }
            std::ranges::copy( coupled_train( neighbour.vehicle ), std::back_inserter( trains ) );
        }
    }
    return trains;
}

// whether any vehicle of the train is within the distance of any vehicle of the other one
bool
trains_within( std::vector<TDynamicObject *> const &Train, std::vector<TDynamicObject *> const &Other, double const Distance ) {

    return std::ranges::any_of( Train, [&]( TDynamicObject const *Vehicle ) {
        return std::ranges::any_of( Other, [&]( TDynamicObject const *Othervehicle ) {
            return glm::distance( Vehicle->GetPosition(), Othervehicle->GetPosition() ) <= Distance; } ); } );
}

} // namespace

// the physical coupler model covers the player's train and the trains it closes in on, on the same track; the others
// keep the legacy model, as the AI drives them in it
void
vehicle_table::update_coupler_physics() const {

    if( false == Global.coupler_physics
     || true == Global.coupler_physics_all ) {
        return;
    }
    auto *player { simulation::Train != nullptr ? simulation::Train->Dynamic() : nullptr };
    std::vector<TDynamicObject *> train;
    if( player != nullptr ) {
        train = coupled_train( player );
    }
    std::unordered_set<TDynamicObject *> covered { train.begin(), train.end() };
    for( auto *vehicle : trains_closing_in( train ) ) {
        covered.insert( vehicle );
    }

    std::unordered_set<TDynamicObject *> checked;
    for( auto *vehicle : m_items ) {
        if( covered.contains( vehicle ) ) {
            vehicle->MoverParameters->CouplerPhysics = true;
            continue;
        }
        if( false == vehicle->MoverParameters->CouplerPhysics
         || checked.contains( vehicle ) ) {
            continue;
        }
        // a train taken in before stays in the model until it's far from the player's one, all of it alike
        auto const other { coupled_train( vehicle ) };
        checked.insert( other.begin(), other.end() );
        if( false == trains_within( other, train, coupler_physics_release ) ) {
            for( auto *othervehicle : other ) {
                othervehicle->MoverParameters->CouplerPhysics = false;
            }
        }
    }
}

// legacy method, calculates changes in simulation state over specified time
void
vehicle_table::update( double Deltatime, int Iterationcount ) {
    // Ra: w zasadzie to trzeba by utworzyć oddzielną listę taboru do liczenia fizyki
    //    na którą by się zapisywały wszystkie pojazdy będące w ruchu
    //    pojazdy stojące nie potrzebują aktualizacji, chyba że np. ktoś im zmieni nastawę hamulca
    //    oddzielną listę można by zrobić na pojazdy z napędem, najlepiej posortowaną wg typu napędu
    // what the update goes through, for the debug panel. the counts are of the last call in the frame
    auto &counts { Timer::subsystem.vehicles };
    counts.all = static_cast<unsigned int>( m_items.size() );
    counts.enabled = counts.road = counts.ai = counts.active = 0;
    counts.kind = {};
    counts.standing = {};
    counts.awake = {};
    counts.steps += static_cast<unsigned int>( std::max( Iterationcount, 1 ) );
    // the kind of each vehicle: rail or road, near the camera or farther. the time of each vehicle in the loops below goes to its
    // kind, measured from the end of the one before it
    static std::vector<std::uint8_t> kinds;
    kinds.resize( m_items.size() );
    // road vehicles far from the camera with nobody inside go by one step of the physics in the frame, as long as all the steps
    // together (Global.physics_roadlod): their movement goes on, in coarser steps nobody is close enough to tell
    // not in a network session: which vehicles go coarse depends on the camera of each machine, so their movement and the draws of
    // Random() they make would part ways between the machines. nor with physics.fullphysics off, whose steps are long already: one
    // step for all of them would be the time of the whole frame, with the collisions looked at once in it
    static std::vector<std::uint8_t> coarse;
    coarse.assign( m_items.size(), 0 );
    auto const roadlod {
        Global.physics_roadlod
        && Global.FullPhysics
        && Global.network_servers.empty()
        && ( false == Global.network_client.has_value() ) };
    auto const camera { Global.pCamera.Pos };
    for( std::size_t index = 0; index < m_items.size(); ++index ) {
        auto const *vehicle { m_items[ index ] };
        auto const road { ( vehicle->MoverParameters->CategoryFlag & 2 ) != 0 };
        auto const distant { glm::length2( vehicle->GetPosition() - camera ) > Timer::subsystem.VEHICLE_NEAR * Timer::subsystem.VEHICLE_NEAR };
        kinds[ index ] = static_cast<std::uint8_t>( ( road ? 2 : 0 ) + ( distant ? 1 : 0 ) );
        coarse[ index ] = static_cast<std::uint8_t>( roadlod && road && distant && ( false == vehicle->MechInside ) && ( Iterationcount > 1 ) );
        if( false == vehicle->bEnabled ) { continue; }
        ++counts.kind[ kinds[ index ] ];
        auto const *mover { vehicle->MoverParameters };
        if( mover->Vel < 0.01 ) {
            ++counts.standing[ kinds[ index ] ];
            if( ( false == road ) && mover->PhysicActivation ) {
                // the test of TMoverParameters::ComputeTotalForce(), part by part
                auto const moving = []( neighbour_data const &Neighbour ) {
                    return ( Neighbour.vehicle != nullptr )
                        && ( ( Neighbour.vehicle->MoverParameters->Vel > 0.0001 ) || Neighbour.vehicle->MoverParameters->accelerating() ); };
                auto &awake { counts.awake };
                ++awake.vehicles;
                if( mover->CabActive != 0 ) { ++awake.cab; }
                if( mover->Vel > 0.0001 ) { ++awake.speed; }
                if( mover->accelerating() ) { ++awake.acceleration; }
                if( mover->LastSwitchingTime < 5 ) { ++awake.switching; }
                if( mover->is_multiple_unit() ) { ++awake.multipleunit; }
                if( moving( mover->Neighbours[ end::front ] ) || moving( mover->Neighbours[ end::rear ] ) ) { ++awake.neighbour; }
            }
        }
    }
    auto const lap = [ & ]( std::size_t const Index, std::array<Timer::frame_stopwatch, 4> &Times, std::chrono::steady_clock::time_point &Mark ) {
        auto const now { std::chrono::steady_clock::now() };
        if( Index < kinds.size() ) {
            Times[ kinds[ Index ] ].add( std::chrono::duration<float, std::milli>( now - Mark ).count() );
        }
        Mark = now;
    };
    for( auto const *vehicle : m_items ) {
        if( false == vehicle->bEnabled ) { continue; }
        ++counts.enabled;
        if( vehicle->MoverParameters->CategoryFlag & 2 ) { ++counts.road; }
        if( ( vehicle->Mechanik != nullptr ) && vehicle->Mechanik->AIControllFlag ) { ++counts.ai; }
        if( vehicle->MoverParameters->PhysicActivation || vehicle->MechInside ) { ++counts.active; }
    }

    Timer::subsystem.vehicles_prepare.start();
    for( auto *vehicle : m_items ) {
        if( false == vehicle->bEnabled ) { continue; }
        // Ra: zmienić warunek na sprawdzanie pantografów w jednej zmiennej: czy pantografy i czy podniesione
        if( vehicle->MoverParameters->EnginePowerSource.SourceType == TPowerSource::CurrentCollector ) {
            update_traction( vehicle );
        }
        vehicle->MoverParameters->ComputeConstans();
        vehicle->update_neighbours();
    }
    update_coupler_physics();
    Timer::subsystem.vehicles_prepare.stop();
    Timer::subsystem.vehicles_forces.start();
    auto const totaltime { Deltatime * Iterationcount }; // całkowity czas
    // the steps of the physics on many threads (Global.performance_multithreading), as many as the work is worth: a thread for each
    // STEPS_PER_THREAD steps of vehicles with the physics on, up to all the cores. not in a network session (network.server,
    // .client), which keeps the draws of Random() in step across the machines: the threads have engines of their own. the messages
    // to external programs (Global.iMultiplayer) don't mind
    auto constexpr STEPS_PER_THREAD { 128 };
    std::size_t work { 0 };
    for( std::size_t index = 0; index < m_items.size(); ++index ) {
        auto const *vehicle { m_items[ index ] };
        if( vehicle->MoverParameters->PhysicActivation || vehicle->MechInside ) {
            work += ( coarse[ index ] != 0 ? 1 : static_cast<std::size_t>( std::max( Iterationcount, 1 ) ) );
        }
    }
    auto const network { ( false == Global.network_servers.empty() ) || Global.network_client.has_value() };
    auto const threads { (
        network ?
            1 :
            std::min( worker_pool::capacity(), std::max<std::size_t>( 1, ( work + STEPS_PER_THREAD / 2 ) / STEPS_PER_THREAD ) ) ) };
    // what keeps them in one thread, for the debug panel
    counts.capacity = static_cast<unsigned int>( worker_pool::capacity() );
    counts.singlethread = (
        false == Global.performance_multithreading ? std::string{ "performance.multithreading off" } :
        network ? std::string{ "network session" } :
        threads <= 1 ? std::string{ "too little work" } :
        std::string{} );
    counts.threads = 1;
    auto const parallel { ( threads > 1 ) && update_steps_parallel( Deltatime, Iterationcount, coarse, threads ) };
    if( parallel ) {
        counts.threads = static_cast<unsigned int>( threads );
    }
    else {
    if( threads > 1 ) {
        counts.singlethread = "a vehicle coupled with one it doesn't see";
    }
    auto mark { std::chrono::steady_clock::now() };
    if( Iterationcount > 1 ) {
        // ABu: ponizsze wykonujemy tylko jesli wiecej niz jedna iteracja
        for( int iteration = 0; iteration < Iterationcount - 1; ++iteration ) {
            std::size_t index { 0 };
            for( auto *vehicle : m_items ) {
                if( ( index >= coarse.size() ) || ( coarse[ index ] == 0 ) ) {
                    vehicle->UpdateForce( Deltatime );
                }
                lap( index++, Timer::subsystem.vehicles_forces_kind, mark );
            }
            index = 0;
            for( auto *vehicle : m_items ) {
                if( ( index >= coarse.size() ) || ( coarse[ index ] == 0 ) ) {
                    vehicle->FastUpdate( Deltatime );
                }
                lap( index++, Timer::subsystem.vehicles_forces_kind, mark );
            }
            update_brake_pipes( Deltatime );
        }
    }
    {
        std::size_t index { 0 };
        for( auto *vehicle : m_items ) {
            // the vehicles going by one step take it with the time of all of them
            auto const coarsestep { ( index < coarse.size() ) && ( coarse[ index ] != 0 ) };
            vehicle->UpdateForce( coarsestep ? totaltime : Deltatime );
            lap( index++, Timer::subsystem.vehicles_forces_kind, mark );
        }
    }
    }
    Timer::subsystem.vehicles_forces.stop();

    update_off_track_contacts( totaltime );

    Timer::subsystem.vehicles_update.start();
    {
        std::size_t index { 0 };
        auto mark { std::chrono::steady_clock::now() };
        for( auto *vehicle : m_items ) {
            auto const coarsestep { ( index < coarse.size() ) && ( coarse[ index ] != 0 ) };
            // Ra 2015-01: tylko tu przelicza sieć trakcyjną
            vehicle->Update( coarsestep ? totaltime : Deltatime, totaltime );
            lap( index++, Timer::subsystem.vehicles_update_kind, mark );
        }
    }
    update_brake_pipes( Deltatime );
    Timer::subsystem.vehicles_update.stop();

    // jeśli jest coś do usunięcia z listy, to trzeba na końcu
    erase_disabled();

    // the odometers are saved now and then, a crash doesn't lose more than the last minute
    m_odometersavetimer += totaltime;
    if( m_odometersavetimer >= 60.0 ) {
        save_odometers();
    }
}
namespace
{


// the body of the vehicle as it stands, for the contacts: off the track by its motion, on the track upright on it
derail_motion::footprint footprint_of( TDynamicObject const &Vehicle ) {

    auto const &mover { *Vehicle.MoverParameters };
    derail_motion::pose pose;
    if( mover.OffTrack.active() ) {
        pose = mover.OffTrack.current();
    }
    else {
        pose.position = Vehicle.GetPosition();
        pose.front = Vehicle.VectorFront();
        pose.left = Vehicle.VectorLeft();
        pose.up = Vehicle.VectorUp();
    }
    auto const &shape { mover.OffTrack.shape() };
    return derail_motion::footprint_of(
        pose, mover.Dim.L,
        mover.OffTrack.active() ? shape.width : ( mover.Dim.W > 0.5 ? mover.Dim.W : 3.0 ),
        mover.OffTrack.active() ? shape.height : ( mover.Dim.H > 2.0 ? mover.Dim.H : 4.0 ) );
}

// the vehicle as a body in a contact
derail_motion::contact_body contact_body_of( TDynamicObject const &Vehicle ) {

    auto const &mover { *Vehicle.MoverParameters };
    derail_motion::contact_body body;
    body.mass = mover.TotalMass;
    if( mover.OffTrack.active() ) {
        body.base = mover.OffTrack.base();
        body.velocity = mover.OffTrack.velocity();
        body.yaw_rate = mover.OffTrack.yaw_rate();
        body.gyration = mover.OffTrack.gyration();
    }
    else {
        // on the rails it goes along them only
        glm::dvec3 const front { Vehicle.VectorFront().x, 0.0, Vehicle.VectorFront().z };
        auto const length { glm::length( front ) };
        body.guided = true;
        body.guide = length > 1e-6 ? front / length : glm::dvec3 { 0.0, 0.0, 1.0 };
        body.base = Vehicle.GetPosition();
        body.velocity = body.guide * mover.V;
    }
    return body;
}

// how the two came to overlap, for the log
std::string_view overlap_kind( TDynamicObject const &First, TDynamicObject const &Second ) {

    auto const offtrack { First.MoverParameters->OffTrack.active() && Second.MoverParameters->OffTrack.active() };
    for( int end = end::front; end <= end::rear; ++end ) {
        if( First.MoverParameters->Couplers[ end ].Connected == Second.MoverParameters
         && First.MoverParameters->Couplers[ end ].CouplingFlag != Coupling::none ) {
            return offtrack ? "coupled, both off the track" : "coupled, one on the rails";
        }
        if( First.MoverParameters->Neighbours[ end ].vehicle == &Second ) {
            return offtrack ? "neighbours, both off the track" : "neighbours, one on the rails";
        }
    }
    return offtrack ? "not neighbours, both off the track" : "not neighbours, one on the rails";
}

} // namespace

void
vehicle_table::update_off_track_contacts( double const Deltatime ) {

    if( Global.derailment != global_settings::derailment_model::physical || Deltatime <= 0.0 ) {
        return;
    }
    // by their places in the table: the order the pairs are taken in is the same in every run
    std::vector<std::size_t> offtrack;
    for( std::size_t index = 0; index < m_items.size(); ++index ) {
        if( m_items[ index ]->bEnabled && m_items[ index ]->MoverParameters->OffTrack.active() ) {
            offtrack.push_back( index );
        }
    }
    if( offtrack.empty() ) {
        return;
    }
    auto const started { off_track_clock::now() };
    auto deepest { 0.0 };
    std::string deepestpair;
    auto resting { true };
    for( auto const place : offtrack ) {
        auto *vehicle { m_items[ place ] };
        resting = resting && vehicle->MoverParameters->OffTrack.at_rest();
        auto const print { footprint_of( *vehicle ) };
        auto const reach { std::hypot( print.half_length, print.half_width ) };
        for( std::size_t otherplace = 0; otherplace < m_items.size(); ++otherplace ) {
            auto *other { m_items[ otherplace ] };
            if( otherplace == place || false == other->bEnabled ) {
                continue;
            }
            // each pair of two off the track once
            auto const otheroff { other->MoverParameters->OffTrack.active() };
            if( otheroff && otherplace < place ) {
                continue;
            }
            // the bounding circles first
            auto const apart { other->GetPosition() - vehicle->GetPosition() };
            auto const otherreach { 0.5 * other->MoverParameters->Dim.L + std::max( other->MoverParameters->Dim.W, other->MoverParameters->Dim.H ) };
            if( glm::length( glm::dvec3 { apart.x, 0.0, apart.z } ) > reach + otherreach + 1.0 ) {
                continue;
            }
            auto const otherprint { footprint_of( *other ) };
            auto const hit { derail_motion::collide( print, otherprint ) };
            if( false == hit.has_value() ) {
                continue;
            }
            if( hit->depth > deepest ) {
                deepest = hit->depth;
                deepestpair = std::format( "{} and {}, {}", vehicle->MoverParameters->Name, other->MoverParameters->Name, overlap_kind( *vehicle, *other ) );
            }
            auto const body { contact_body_of( *vehicle ) };
            auto const otherbody { contact_body_of( *other ) };
            auto const impulse { derail_motion::contact_impulse( *hit, body, otherbody ) };
            // the one off the track: kicked, and moved out of the other one (half of the way when both are off it)
            auto const [ velocity, yawrate ] { derail_motion::impulse_change( body, -impulse, hit->point ) };
            auto &motion { vehicle->MoverParameters->OffTrack };
            auto const share { otheroff ? 0.5 : 1.0 };
            auto const correction { derail_motion::contact_correction * std::max( 0.0, hit->depth - derail_motion::contact_slop ) };
            if( glm::length( impulse ) > 0.0 ) {
                motion.kick( velocity, yawrate );
                vehicle->MoverParameters->V = motion.speed();
            }
            if( correction > 0.0 ) {
                motion.nudge( -hit->normal * ( share * correction ) );
                vehicle->place_off_track();
            }
            if( otheroff ) {
                auto const [ othervelocity, otheryawrate ] { derail_motion::impulse_change( otherbody, impulse, hit->point ) };
                auto &othermotion { other->MoverParameters->OffTrack };
                if( glm::length( impulse ) > 0.0 ) {
                    othermotion.kick( othervelocity, otheryawrate );
                    other->MoverParameters->V = othermotion.speed();
                }
                if( correction > 0.0 ) {
                    othermotion.nudge( hit->normal * ( share * correction ) );
                    other->place_off_track();
                }
                continue;
            }
            // one on the rails: along them into its speed, across them on its wheels, as a coupling pulls them off
            auto &othermover { *other->MoverParameters };
            othermover.V += glm::dot( impulse, otherbody.guide ) / std::max( othermover.TotalMass, 1.0 );
            if( othermover.DamageFlag.HasAll(VehicleDamage::derailed) ) {
                continue;
            }
            glm::dvec3 const otherleft { otherbody.guide.z, 0.0, -otherbody.guide.x };
            auto const across { glm::dot( impulse, otherleft ) / Deltatime };
            auto const mass { othermover.TotalMass - othermover.Mred };
            auto const ratio { derail_motion::pull_ratio( std::abs( across ), 0.5 * std::numbers::pi, mass, othermover.NAxles ) };
            // held over the window of the climbing as the bodies slide along each other, or a hit three times as hard
            // at once: the wheels jump the rail
            auto const sliding { glm::length( derail_motion::velocity_at( body, hit->point ) - otherbody.velocity ) * Deltatime };
            if( other->m_struck.update( ratio >= derailment::climbing_limit, sliding ) || ratio >= 3.0 * derailment::climbing_limit ) {
                othermover.pull_off(
                    across >= 0.0 ? 1 : -1,
                    mass > 0.0 ? std::abs( across ) / mass : 0.0,
                    std::format( "Y/Q {:.2f} from {} running into it, {:.0f} kN across", ratio, vehicle->MoverParameters->Name, std::abs( across ) / 1000.0 ) );
                other->m_struck.reset();
            }
        }
    }
    // the log: every second the most the bodies overlapped by, once more when all of them are at rest
    if( deepest > m_overlapmost ) {
        m_overlapmost = deepest;
        m_overlappair = deepestpair;
    }
    m_overlapdue -= Deltatime;
    if( ( m_overlapdue <= 0.0 && false == resting ) || ( resting && false == m_overlaplogged ) ) {
        WriteLog( std::format(
            "derailment: the bodies overlap by up to {:.2f} m{} over the last second, {:.2f} m now{}",
            m_overlapmost, m_overlapmost > 0.0 ? " (" + m_overlappair + ")" : std::string {}, deepest, resting ? ", all at rest" : "" ) );
        m_overlapmost = 0.0;
        m_overlappair.clear();
        m_overlapdue = 1.0;
        m_overlaplogged = resting;
    }
    if( false == resting ) {
        m_overlaplogged = false;
    }
    auto const took { seconds_since( started ) };
    if( took > off_track_slow ) {
        ErrorLog( std::format( "derailment: warning: the contacts of {} vehicles off the track took {:.0f} ms", offtrack.size(), took * 1000.0 ) );
    }
}

std::shared_ptr<derail_motion::mesh_ground const>
vehicle_table::off_track_terrain( glm::dvec2 const &Min, glm::dvec2 const &Max ) {

    std::erase_if( m_offtrackterrains, []( auto const &Terrain ) { return Terrain.expired(); } );
    for( auto const &terrain : m_offtrackterrains ) {
        if( auto shared { terrain.lock() }; shared != nullptr && shared->covers( Min, Max ) ) {
            return shared;
        }
    }
    return {};
}

void
vehicle_table::share_off_track_terrain( std::shared_ptr<derail_motion::mesh_ground const> const &Terrain ) {

    if( Terrain != nullptr ) {
        m_offtrackterrains.emplace_back( Terrain );
    }
}

// the air exchange through the brake and main reservoir hoses of the vehicles which stepped their physics, after all of them did
void
vehicle_table::update_brake_pipes( double const Deltatime ) {

    m_brakepipes.clear();
    for( auto *vehicle : m_items ) {
        if( ( true == vehicle->MoverParameters->PipeCouplersPending )
         || ( true == vehicle->MoverParameters->ScndPipeCouplersPending ) ) {
            m_brakepipes.push_back( vehicle->MoverParameters );
        }
    }
    TMoverParameters::UpdatePipeCouplers( m_brakepipes, Deltatime );
}

// records the odometer states of the vehicles and saves them for the next sessions (eu07/maszyna#72)
void
vehicle_table::save_odometers() {

    m_odometersavetimer = 0.0;
    auto &odometers { odometer::states() };
    for( auto const *vehicle : m_items ) {
        if( vehicle->MoverParameters != nullptr ) {
            odometer::record( odometers, vehicle->name(), vehicle->MoverParameters->HasDriverCab(), vehicle->MoverParameters->Odometer );
        }
    }
    odometer::save();
}

// the steps of the physics of vehicle_table::update() (UpdateForce() and FastUpdate() of each vehicle, Iterationcount times) on up
// to Threads threads of worker_pool. a step acts on the vehicle itself, on the ones coupled with it, and on the ones seen ahead or
// behind it (CollisionDetect() changes the speed of the other one); the movement along the tracks, the events and the sounds wait
// for Update(). so the vehicles are put in groups which act on nothing outside them, and each group goes through all its steps in
// one thread, its vehicles in their order, the groups at once: what doing them one by one gives. the groups of vehicles exchanging
// their load, which can load the models of it, are done in the main thread. Coarse: as in update(). returns false, having done
// nothing, when a vehicle is coupled with one it doesn't see next to it, which the groups don't account for
bool
vehicle_table::update_steps_parallel( double const Deltatime, int const Iterationcount, std::vector<std::uint8_t> const &Coarse, std::size_t const Threads ) {

    auto const itemcount { m_items.size() };
    for( std::size_t index = 0; index < itemcount; ++index ) {
        m_items[ index ]->m_tableindex = static_cast<std::uint32_t>( index );
    }
    // the groups, by joining the vehicles with the ones they act on. the coupled ones are those next to them
    m_groupparent.resize( itemcount );
    std::iota( std::begin( m_groupparent ), std::end( m_groupparent ), 0u );
    auto const root = [ this ]( std::uint32_t Index ) {
        while( m_groupparent[ Index ] != Index ) {
            m_groupparent[ Index ] = m_groupparent[ m_groupparent[ Index ] ];
            Index = m_groupparent[ Index ];
        }
        return Index; };
    for( std::size_t index = 0; index < itemcount; ++index ) {
        auto const *mover { m_items[ index ]->MoverParameters };
        for( auto const end : { end::front, end::rear } ) {
            auto const *neighbour { mover->Neighbours[ end ].vehicle };
            if( ( mover->Couplers[ end ].Connected != nullptr )
             && ( ( neighbour == nullptr ) || ( neighbour->MoverParameters != mover->Couplers[ end ].Connected ) ) ) {
                return false;
            }
            if( neighbour == nullptr ) { continue; }
            auto const other { neighbour->m_tableindex };
            if( ( other >= itemcount ) || ( m_items[ other ] != neighbour ) ) {
                // not in the table, shouldn't happen
                return false;
            }
            auto const left { root( static_cast<std::uint32_t>( index ) ) };
            auto const right { root( other ) };
            if( left != right ) {
                m_groupparent[ std::max( left, right ) ] = std::min( left, right );
            }
        }
    }
    auto constexpr none { std::numeric_limits<std::uint32_t>::max() };
    m_groupof.assign( itemcount, none );
    std::size_t groupcount { 0 };
    for( std::size_t index = 0; index < itemcount; ++index ) {
        auto &group { m_groupof[ root( static_cast<std::uint32_t>( index ) ) ] };
        if( group == none ) {
            group = static_cast<std::uint32_t>( groupcount++ );
            if( m_groups.size() < groupcount ) { m_groups.emplace_back(); }
            m_groups[ group ].clear();
        }
        m_groups[ group ].push_back( static_cast<std::uint32_t>( index ) );
    }
    // the groups done in the main thread, and the others largest first, for the threads to end close to one another
    m_threadgroups.clear();
    m_maingroups.clear();
    std::size_t largest { 0 };
    for( std::size_t group = 0; group < groupcount; ++group ) {
        largest = std::max( largest, m_groups[ group ].size() );
        auto const loading { std::any_of(
            std::begin( m_groups[ group ] ), std::end( m_groups[ group ] ),
            [ this ]( std::uint32_t const Index ) {
                auto const *vehicle { m_items[ Index ] };
                return ( vehicle->MoverParameters->LoadStatus != 0 ) || ( vehicle->LoadExchangeTime() > 0.f ); } ) };
        ( loading ? m_maingroups : m_threadgroups ).push_back( static_cast<std::uint32_t>( group ) );
    }
    std::sort(
        std::begin( m_threadgroups ), std::end( m_threadgroups ),
        [ this ]( std::uint32_t const Left, std::uint32_t const Right ) {
            return m_groups[ Left ].size() > m_groups[ Right ].size(); } );
    auto &counts { Timer::subsystem.vehicles };
    counts.groups = static_cast<unsigned int>( groupcount );
    counts.largestgroup = static_cast<unsigned int>( largest );
    counts.maingroups = static_cast<unsigned int>( m_maingroups.size() );

    // all the steps of a group, in the order update() makes them
    auto const totaltime { Deltatime * Iterationcount };
    auto const coarse = [ & ]( std::uint32_t const Index ) {
        return ( Index < Coarse.size() ) && ( Coarse[ Index ] != 0 ); };
    // the draws of Random() a group makes come from an engine of its own, seeded from the run's engine and the group: the same in
    // every run of a seed, whichever thread takes the group (the engines of the threads are seeded at random)
    auto const seed { static_cast<std::uint32_t>( Global.random_engine() ) };
    auto const steps = [ & ]( std::uint32_t const Group ) {
        auto const &members { m_groups[ Group ] };
        std::seed_seq groupseed { seed, members.front() };
        std::mt19937 engine { groupseed }; // NOSONAR: randomness of the simulation, not of anything security-relevant
        auto *const threadengine { thread_random_engine() };
        set_thread_random_engine( &engine );
        // the hoses join only vehicles coupled with each other, which are in one group: the air they carry is exchanged
        // within the group after each step, as update_brake_pipes() does for all the vehicles in one thread
        std::vector<TMoverParameters *> pipes;
        if( Iterationcount > 1 ) {
            // ABu: ponizsze wykonujemy tylko jesli wiecej niz jedna iteracja
            for( int iteration = 0; iteration < Iterationcount - 1; ++iteration ) {
                for( auto const index : members ) {
                    if( false == coarse( index ) ) { m_items[ index ]->UpdateForce( Deltatime ); }
                }
                for( auto const index : members ) {
                    if( false == coarse( index ) ) { m_items[ index ]->FastUpdate( Deltatime ); }
                }
                pipes.clear();
                for( auto const index : members ) {
                    auto *mover { m_items[ index ]->MoverParameters };
                    if( mover->PipeCouplersPending || mover->ScndPipeCouplersPending ) {
                        pipes.push_back( mover );
                    }
                }
                TMoverParameters::UpdatePipeCouplers( pipes, Deltatime );
            }
        }
        for( auto const index : members ) {
            // the vehicles going by one step take it with the time of all of them
            m_items[ index ]->UpdateForce( coarse( index ) ? totaltime : Deltatime );
        }
        set_thread_random_engine( threadengine ); };

    Workers().run(
        m_threadgroups.size(), Threads,
        [ & ]( std::size_t const Group ) {
            steps( m_threadgroups[ Group ] ); } );
    for( auto const group : m_maingroups ) {
        steps( group );
    }
    return true;
}

// legacy method, checks for presence and height of traction wire for specified vehicle
void
vehicle_table::update_traction( TDynamicObject *Vehicle ) {
	auto const vFront = Vehicle->VectorFront(); // wektor normalny dla płaszczyzny ruchu pantografu
	auto const vUp = Vehicle->VectorUp(); // wektor pionu pudła (pochylony od pionu na przechyłce)
	auto const vLeft = Vehicle->VectorLeft(); // wektor odległości w bok (odchylony od poziomu na przechyłce)
    auto const position = Vehicle->GetPosition(); // współrzędne środka pojazdu

    for( int pantographindex = 0; pantographindex < Vehicle->iAnimType[ ANIM_PANTS ]; ++pantographindex ) {
        // pętla po pantografach
        auto *pantograph { Vehicle->pants[ pantographindex ].fParamPants };
        if( true == Vehicle->MoverParameters->Pantographs[ pantographindex ].is_active ) {
            // jeśli pantograf podniesiony
            auto const pant0 { position + vLeft * pantograph->vPos.z + vUp * pantograph->vPos.y + vFront * pantograph->vPos.x };
            if( pantograph->hvPowerWire != nullptr ) {
                // jeżeli znamy drut z poprzedniego przebiegu
                for( int attempts = 0; attempts < 30; ++attempts ) {
                    // sanity check. shouldn't happen in theory, but did happen in practice
                    if( pantograph->hvPowerWire == nullptr ) { break; }

                    // powtarzane aż do znalezienia odpowiedniego odcinka na liście dwukierunkowej
                    if( pantograph->hvPowerWire->iLast & 0x3 ) {
                        // dla ostatniego i przedostatniego przęsła wymuszamy szukanie innego
                        // nie to, że nie ma, ale trzeba sprawdzić inne
                        pantograph->hvPowerWire = nullptr;
                        break;
                    }
                    if( pantograph->hvPowerWire->hvParallel ) {
                        // jeśli przęsło tworzy bieżnię wspólną, to trzeba sprawdzić pozostałe
                        // nie to, że nie ma, ale trzeba sprawdzić inne
                        pantograph->hvPowerWire = nullptr;
                        break;
                    }
                    // obliczamy wyraz wolny równania płaszczyzny (to miejsce nie jest odpowienie)
                    // podstawiamy równanie parametryczne drutu do równania płaszczyzny pantografu
                    // TODO: investigate this routine with reardriver/negative speed, does it picks the right wire?
                    auto const fRaParam =
                        -( glm::dot( pantograph->hvPowerWire->pPoint1, vFront ) - glm::dot( pant0, vFront ) )
                         / glm::dot( pantograph->hvPowerWire->vParametric, vFront );

                    if( fRaParam < -0.001 ) {
                        // histereza rzędu 7cm na 70m typowego przęsła daje 1 promil
                        pantograph->hvPowerWire = pantograph->hvPowerWire->hvNext[ 0 ];
                        continue;
                    }
                    if( fRaParam > 1.001 ) {
                        pantograph->hvPowerWire = pantograph->hvPowerWire->hvNext[ 1 ];
                        continue;
                    }
                    // jeśli t jest w przedziale, wyznaczyć odległość wzdłuż wektorów vUp i vLeft
                    // punkt styku płaszczyzny z drutem (dla generatora łuku el.)
                    auto const vStyk { pantograph->hvPowerWire->pPoint1 + fRaParam * pantograph->hvPowerWire->vParametric };
                    auto const vGdzie { vStyk - pant0 }; // wektor
                    // odległość w pionie musi być w zasięgu ruchu "pionowego" pantografu
                    // musi się mieścić w przedziale ruchu pantografu
                    auto const fVertical { glm::dot( vGdzie, vUp ) };
                    // odległość w bok powinna być mniejsza niż pół szerokości pantografu
                    // to się musi mieścić w przedziale zależnym od szerokości pantografu
                    auto const fHorizontal { std::abs( glm::dot( vGdzie, vLeft ) ) - pantograph->fWidth };
                    // jeśli w pionie albo w bok jest za daleko, to dany drut jest nieużyteczny
                    if( fHorizontal <= 0.0 ) {
                        // koniec pętli, aktualny drut pasuje
                        pantograph->PantTraction = fVertical;
                        break;
                    }
                    else {
                        // the wire is outside contact area and as of now we don't have good detection of parallel sections
                        // as such there's no guaratee there isn't parallel section present.
                        // therefore we don't bother checking if the wire is still within range of guide horns
                        // but simply force area search for potential better option
                        pantograph->hvPowerWire = nullptr;
                        break;
                    }
                }
            }

            if( pantograph->hvPowerWire == nullptr ) {
                // look in the region for a suitable traction piece if we don't already have any
                simulation::Region->update_traction( Vehicle, pantographindex );
            }

            if( pantograph->hvPowerWire == nullptr
             && false == Global.bLiveTraction ) {
                // jeśli drut nie znaleziony ale można oszukiwać to dajemy coś tam dla picu
                Vehicle->pants[ pantographindex ].fParamPants->PantTraction = 1.4;
            }
        }
        else {
            // pantograph is down
            pantograph->hvPowerWire = nullptr;
        }
    }
}

// legacy method, sends list of vehicles over network
void
vehicle_table::DynamicList( bool const Onlycontrolled ) const {
    // odesłanie nazw pojazdów dostępnych na scenerii (nazwy, szczególnie wagonów, mogą się powtarzać!)
    for( auto const *vehicle : m_items ) {
        if( false == Onlycontrolled
         || vehicle->Mechanik != nullptr ) {
            // same nazwy pojazdów
            multiplayer::WyslijString( vehicle->asName, 6 );
        }
    }
    // informacja o końcu listy
    multiplayer::WyslijString( "none", 6 );
}

// maintenance; removes from tracks consists with vehicles marked as disabled
bool
vehicle_table::erase_disabled() {

    if( false == TDynamicObject::bDynamicRemove ) { return false; }

    // go through the list and retrieve vehicles scheduled for removal...
    type_sequence disabledvehicles;
    for( auto *vehicle : m_items ) {
        if( false == vehicle->bEnabled ) {
            disabledvehicles.emplace_back( vehicle );
        }
    }
    // ...now propagate removal flag through affected consists...
    for( auto *vehicle : disabledvehicles ) {
        TDynamicObject *coupledvehicle { vehicle };
        while( ( coupledvehicle = coupledvehicle->Next() ) != nullptr ) {
            coupledvehicle->bEnabled = false;
        }
        // (try to) run propagation in both directions, it's simpler than branching based on direction etc
        coupledvehicle = vehicle;
        while( ( coupledvehicle = coupledvehicle->Prev() ) != nullptr ) {
            coupledvehicle->bEnabled = false;
        }
    }
    // ...then actually remove all disabled vehicles...
    auto vehicleiter = std::begin( m_items );
    while( vehicleiter != std::end( m_items ) ) {

        auto *vehicle { *vehicleiter };

        if( true == vehicle->bEnabled ) {
            ++vehicleiter;
        }
        else {
            // the distance the vehicle ran goes in the odometer store now, it doesn't depend on the vehicle being
            // kept on the list until the next save (eu07/maszyna#72)
            if( vehicle->MoverParameters != nullptr ) {
                odometer::record( odometer::states(), vehicle->name(), vehicle->MoverParameters->HasDriverCab(), vehicle->MoverParameters->Odometer );
            }
            if( vehicle->MyTrack != nullptr
             && true == vehicle->MyTrack->RemoveDynamicObject(vehicle) ) {
                vehicle->MyTrack = nullptr;
            }
            if( simulation::Train != nullptr
             && simulation::Train->Dynamic() == vehicle ) {
                // clear potential train binding
                // TBD, TODO: kill vehicle sounds
				simulation::Train = nullptr;
            }
			simulation::Trains.purge(vehicle->name());
            // remove potential entries in the light array
            simulation::Lights.remove( vehicle );
            simulation::Blackbox.vehicle_removed( vehicle );
/*
            // finally get rid of the vehicle and its record themselves
            // BUG: deleting the vehicle leaves dangling pointers in event->Activator and potentially elsewhere
            // TBD, TODO: either mark 'dead' vehicles with additional flag, or delete the references as well
            SafeDelete( vehicle );
            vehicleiter = m_items.erase( vehicleiter );
*/
            ++vehicleiter; // NOTE: instead of the erase in the disabled section
        }
    }
    // ...and call it a day
    TDynamicObject::bDynamicRemove = false;

    return true;
}
