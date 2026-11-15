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
#include "simulation/simulation.h"
#include "simulation/simulationenvironment.h"
#include "scene/scene.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "vehicle/DynObj.h"

#include <format>

namespace {

// how often the temperature is checked, s of the simulation
double constexpr check_interval { 60.0 };
// the tracks the temperature can buckle, found at the first check
std::vector<TTrack *> heat_sites;
bool heat_sites_found { false };
double heat_clock { 0.0 };
// the tracks to buckle under a train, done with the next update rather than amid the moves of the vehicles
std::vector<TTrack *> heat_due;

double rail_temperature_now() {

    return track_buckle::rail_temperature( Global.AirTemperature, simulation::Environment.sun().getAngle(), Global.Overcast );
}

} // namespace

double TTrack::plan_radius( double const Distance, int &Outward ) const {

    Outward = 0;
    if( Segment == nullptr || eType != tt_Normal ) {
        return 0.0;
    }
    auto const length { Segment->GetLength() };
    auto const reach { std::min( 5.0, 0.25 * length ) };
    auto const at { std::clamp( Distance, reach, length - reach ) };
    glm::dvec3 behind, middle, ahead, across;
    Segment->plan_frame( at - reach, behind, across );
    Segment->plan_frame( at + reach, ahead, across );
    Segment->plan_frame( at, middle, across );
    auto bend { ( behind + ahead - 2.0 * middle ) / ( reach * reach ) };
    bend.y = 0.0;
    auto const curvature { glm::length( bend ) };
    // straighter than the curves the tracks are laid with
    if( curvature < 1.0 / 20000.0 ) {
        return 0.0;
    }
    // away from the centre, which the bend points to
    Outward = ( glm::dot( bend, across ) > 0.0 ? -1 : 1 );
    return 1.0 / curvature;
}

double TTrack::buckle_curvature( double const Distance, double const Base ) const {

    if( false == is_buckled() || false == track_buckle::within( m_buckle, Distance ) ) {
        return 0.0;
    }
    auto const bend { track_buckle::mean_bend( m_buckle, Distance, Base ) };
    auto outward { 0 };
    plan_radius( Distance, outward );
    // the centre of the curve is away from the outer side
    return outward != 0 ? -outward * bend : bend;
}

std::uint64_t TTrack::buckle_key() const {

    if( false == m_name.empty() && m_name != "none" ) {
        return track_buckle::name_hash( m_name );
    }
    auto const point { location() };
    return track_buckle::name_hash( std::format( "{:.1f};{:.1f};{:.1f}", point.x, point.y, point.z ) );
}

track_buckle::shape TTrack::resolve_buckle( track_buckle::declared const &Values, std::uint32_t const Seed ) const {

    if( Segment == nullptr || eType != tt_Normal || iCategoryFlag != 1 ) {
        return {};
    }
    auto const length { Segment->GetLength() };
    auto const key { buckle_key() };
    auto const place { track_buckle::draw( key, Seed, track_buckle::place ) };
    // the side out of the curve where the buckle is
    auto const centre { Values.centre >= 0.f ? static_cast<double>( Values.centre ) : 0.5 * length };
    auto outward { 0 };
    auto const radius { plan_radius( centre, outward ) };
    return track_buckle::resolve( Values, length, outward, radius, place, track_buckle::draw( key, Seed, track_buckle::way ) );
}

bool TTrack::buckle_now( track_buckle::declared const &Values, std::string const &Cause ) {

    if( Global.track_buckling == global_settings::track_buckling_mode::off ) {
        return false;
    }
    if( eType != tt_Normal || iCategoryFlag != 1 || Segment == nullptr ) {
        ErrorLog( "Bad track: \"" + m_name + "\" can't buckle, it isn't a plain rail track" );
        return false;
    }
    auto const shape { resolve_buckle( Values, 0 ) };
    if( track_buckle::given( Values ) && false == track_buckle::active( shape ) ) {
        ErrorLog( std::format( "Bad track: \"{}\" too short for a buckle {:.0f} m long", m_name, 2.0 * track_buckle::reach_share * ( Values.length > 0.f ? Values.length : track_buckle::straight_length ) ) );
        return false;
    }
    m_buckle = shape;
    m_buckle_armed = false;
    // made again with the buckle, which the rails, the sleepers and the axles on the track follow
    init_segments( false );
    auto const rebuild = []( TTrack &Track ) {
        auto &section { simulation::Region->section( Track.location() ) };
        Track.rebuild_geometry( section.m_geometrycreated ? section.m_geometrybank : gfx::geometrybank_handle {} ); };
    rebuild( *this );
    for( auto *crossing : m_crossings ) {
        if( crossing != nullptr && crossing != this ) {
            rebuild( *crossing );
        }
    }
    if( is_buckled() ) {
        WriteLog( std::format(
            "Track \"{}\" buckled ({}): {:.0f} mm to the {} at {:.1f} m, main lobe {:.1f} m, curvature up to 1/{:.0f} m",
            m_name, Cause, std::abs( 1000.0 * m_buckle.amplitude ), m_buckle.amplitude > 0.0 ? "right" : "left", m_buckle.centre, m_buckle.length,
            1.0 / track_buckle::peak_bend( m_buckle.amplitude, m_buckle.length ) ) );
    }
    else {
        WriteLog( "Track \"" + m_name + "\" straightened (" + Cause + ")" );
    }
    return true;
}

bool TTrack::buckles_by_heat() const {

    if( eType != tt_Normal || iCategoryFlag != 1 || Segment == nullptr || is_buckled() || false == m_crossings.empty() ) {
        return false;
    }
    auto outward { 0 };
    auto const radius { plan_radius( 0.5 * Segment->GetLength(), outward ) };
    // continuous welded rail is laid down to a radius of 190 m (PLK ST-T1-A8)
    if( radius > 0.0 && radius < 190.0 ) {
        return false;
    }
    auto const natural { track_buckle::natural( outward != 0 && radius < track_buckle::curve_radius ) };
    if( Segment->GetLength() < 2.0 * ( track_buckle::reach_share * natural.length + track_buckle::end_margin ) ) {
        return false;
    }
    return track_buckle::weak_spot( buckle_key(), Global.random_seed, Segment->GetLength(), m_cwr.worked );
}

track_buckle::limits TTrack::buckle_limits() const {

    auto outward { 0 };
    auto const radius { plan_radius( 0.5 * Segment->GetLength(), outward ) };
    return track_buckle::limits_of( m_cwr, radius, uneven_declared().side );
}

void TTrack::update_buckling( double const Deltatime ) {

    if( Global.track_buckling != global_settings::track_buckling_mode::automatic ) {
        return;
    }
    if( false == heat_sites_found ) {
        heat_sites_found = true;
        for( auto *track : simulation::Paths.sequence() ) {
            if( track != nullptr && track->buckles_by_heat() ) {
                heat_sites.push_back( track );
            }
        }
        WriteLog( std::format( "Track buckling: {} places the heat can buckle", heat_sites.size() ) );
        // the first check right away, a scenario can start in the heat
        heat_clock = check_interval;
    }
    for( auto *track : heat_due ) {
        track->buckle_under_train();
    }
    heat_due.clear();
    heat_clock += Deltatime;
    if( heat_clock < check_interval ) {
        return;
    }
    heat_clock = 0.0;
    auto const rail { rail_temperature_now() };
    for( auto *track : heat_sites ) {
        if( track->is_buckled() ) {
            continue;
        }
        auto const rise { rail - track->m_cwr.neutral };
        if( track_buckle::decide( rise, track->buckle_limits(), false, 1.0 ) == track_buckle::verdict::by_itself ) {
            auto outward { 0 };
            auto const radius { track->plan_radius( 0.5 * track->Segment->GetLength(), outward ) };
            auto values { track_buckle::natural( outward != 0 && radius < track_buckle::curve_radius ) };
            values.centre = static_cast<float>( track_buckle::resolve( values, track->Segment->GetLength(), outward, radius,
                track_buckle::draw( track->buckle_key(), Global.random_seed, track_buckle::place ), 0.0 ).centre );
            track->buckle_now( values, std::format( "rails at {:.0f} C, {:.0f} K above neutral, {:.0f} kN in each", rail, rise,
                0.001 * track_buckle::thermal_force( track_buckle::rail_area( track->m_profile1.first ), rise ) ) );
        }
    }
}

void TTrack::buckle_pass_enter( TDynamicObject const &Dynamic ) {

    if( Global.track_buckling != global_settings::track_buckling_mode::automatic || is_buckled() || false == heat_sites_found ) {
        return;
    }
    if( std::ranges::find( heat_sites, this ) == heat_sites.end() ) {
        return;
    }
    ++m_buckle_passes;
    auto const *mover { Dynamic.MoverParameters };
    auto const train { mover != nullptr && track_buckle::stirs( std::abs( mover->Vel ), mover->BrakePress > 0.5 ) };
    auto const rise { rail_temperature_now() - m_cwr.neutral };
    auto const draw { track_buckle::draw( buckle_key(), Global.random_seed, track_buckle::pass, m_buckle_passes ) };
    m_buckle_armed = ( track_buckle::decide( rise, buckle_limits(), train, draw ) != track_buckle::verdict::none );
}

void TTrack::buckle_pass_leave() {

    if( false == m_buckle_armed ) {
        return;
    }
    m_buckle_armed = false;
    heat_due.push_back( this );
}

void TTrack::buckle_under_train() {

    if( is_buckled() ) {
        return;
    }
    // under the rest of the train, which lifts the track between its bogies
    auto const rail { rail_temperature_now() };
    auto outward { 0 };
    auto const radius { plan_radius( 0.5 * Segment->GetLength(), outward ) };
    auto values { track_buckle::natural( outward != 0 && radius < track_buckle::curve_radius ) };
    values.centre = static_cast<float>( track_buckle::resolve( values, Segment->GetLength(), outward, radius,
        track_buckle::draw( buckle_key(), Global.random_seed, track_buckle::place ), 0.0 ).centre );
    buckle_now( values, std::format( "under a train, rails at {:.0f} C, {:.0f} K above neutral", rail, rail - m_cwr.neutral ) );
}
