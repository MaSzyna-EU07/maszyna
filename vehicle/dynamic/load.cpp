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

// the load of a vehicle, its exchange at the stops, how it shows

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


// initiates load change by specified amounts, with a platform on specified side
void TDynamicObject::LoadExchange( int const Disembark, int const Embark, int const Platforms ) {
/*
    if( ( MoverParameters->Doors.open_control == control_t::passenger )
     || ( MoverParameters->Doors.open_control == control_t::mixed ) ) {
        // jeśli jedzie do tyłu, to drzwi otwiera odwrotnie
        auto const lewe = ( DirectionGet() > 0 ) ? 1 : 2;
        auto const prawe = 3 - lewe;
        if( Platform & lewe ) {
            MoverParameters->OperateDoors( side::left, true, range_t::local );
        }
        if( Platform & prawe ) {
            MoverParameters->OperateDoors( side::right, true, range_t::local );
        }
    }
*/
    if( Platforms == 0 ) { return; } // edge case, if there's no accessible platforms discard the request

    m_exchange.unload_count += Disembark;
    m_exchange.load_count += Embark;
    m_exchange.platforms = Platforms;
    m_exchange.time = 0.0;
}

// calculates time needed to complete current load change
float TDynamicObject::LoadExchangeTime( int const Platforms ) {

    m_exchange.platforms = Platforms;
    return LoadExchangeTime();
}

float TDynamicObject::LoadExchangeTime() const {

    if( m_exchange.unload_count < 0.01 && m_exchange.load_count < 0.01 ) { return 0.f; }

    auto const baseexchangetime { m_exchange.unload_count / MoverParameters->UnLoadSpeed + m_exchange.load_count / MoverParameters->LoadSpeed };
    auto const nominalexchangespeedfactor { ( m_exchange.platforms == 3 ? 2.f : 1.f ) };
    auto const actualexchangespeedfactor { LoadExchangeSpeed() };

    return baseexchangetime / ( actualexchangespeedfactor > 0.f ? actualexchangespeedfactor : nominalexchangespeedfactor );
}

// calculates current load exchange rate
float TDynamicObject::LoadExchangeSpeed() const {
    // platforms (1:left, 2:right, 3:both)
    // with exchange performed on both sides waiting times are halved
    auto exchangespeedfactor { 0.f };
    auto const lewe { DirectionGet() > 0 ? 1 : 2 };
    auto const prawe { 3 - lewe };
    if( m_exchange.platforms & lewe ) {
        exchangespeedfactor += MoverParameters->Doors.instances[side::left].is_open ? 1.f : 0.f;
    }
    if( m_exchange.platforms & prawe ) {
        exchangespeedfactor += MoverParameters->Doors.instances[side::right].is_open ? 1.f : 0.f;
    }

    return exchangespeedfactor;
}

// update state of load exchange operation
void TDynamicObject::update_exchange( double const Deltatime ) {

    // TODO: move offset calculation after initial check, when the loading system is all unified
    update_load_offset();

    if( m_exchange.unload_count < 0.01 && m_exchange.load_count < 0.01 ) { return; }

    if( MoverParameters->Vel < 2.0 ) {

        auto const exchangespeed { LoadExchangeSpeed() };

        if( exchangespeed < ( m_exchange.platforms == 3 ? 2.f : 1.f ) ) {
            // the exchange isn't performed at optimal rate, or at all. try to open viable vehicle doors to speed it up
            auto const lewe { DirectionGet() > 0 ? 1 : 2 };
            auto const prawe { 3 - lewe };
            if( m_exchange.platforms & lewe
             && false == (MoverParameters->Doors.instances[side::left].is_open || MoverParameters->Doors.instances[side::left].is_opening) ) {
                // try to open left door
                MoverParameters->OperateDoors( side::left, true, range_t::local );
            }
            if( m_exchange.platforms & prawe
             && false == (MoverParameters->Doors.instances[side::right].is_open || MoverParameters->Doors.instances[side::right].is_opening) ) {
                // try to open right door
                MoverParameters->OperateDoors( side::right, true, range_t::local );
            }
        }

        if( exchangespeed > 0.f ) {

            m_exchange.time += Deltatime;
            while( m_exchange.unload_count > 0.01
                && m_exchange.time >= 1.0 ) {

                m_exchange.time -= 1.0;
                auto const exchangesize = std::min( m_exchange.unload_count, MoverParameters->UnLoadSpeed * exchangespeed );
                m_exchange.unload_count -= exchangesize;
                MoverParameters->LoadStatus = 1;
                MoverParameters->LoadAmount = std::max( 0.f, MoverParameters->LoadAmount - exchangesize );
                MoverParameters->ComputeMass();
                update_load_visibility();
            }
            if( m_exchange.unload_count < 0.01 ) {
                // finish any potential unloading operation before adding new load
                // don't load more than can fit
                m_exchange.load_count = std::min( m_exchange.load_count, MoverParameters->MaxLoad - MoverParameters->LoadAmount );
                while( m_exchange.load_count > 0.01
                    && m_exchange.time >= 1.0 ) {

                    m_exchange.time -= 1.0;
                    auto const exchangesize = std::min( m_exchange.load_count, MoverParameters->LoadSpeed * exchangespeed );
                    m_exchange.load_count -= exchangesize;
                    MoverParameters->LoadStatus = 2;
                    MoverParameters->LoadAmount = std::min( MoverParameters->MaxLoad, MoverParameters->LoadAmount + exchangesize ); // std::max not strictly needed but, eh
                    MoverParameters->ComputeMass();
                    update_load_visibility();
                }
            }
        }
    }

    if( MoverParameters->Vel > 2.0 ) {
        // if the vehicle moves too fast cancel the exchange
        m_exchange.unload_count = 0;
        m_exchange.load_count = 0;
    }

    if( m_exchange.unload_count < 0.01
     && m_exchange.load_count < 0.01 ) {

        MoverParameters->LoadStatus = 4;
        // if the exchange is completed (or canceled) close the door, if applicable
        if( MoverParameters->Doors.close_control == control_t::passenger
         || MoverParameters->Doors.close_control == control_t::mixed ) {

            if( MoverParameters->Vel > 2.0
             || Random() < (
			                                                 // remotely controlled door are more likely to be left open
			                                                 MoverParameters->Doors.close_control == control_t::passenger ? 0.75 : 0.50) ) {

                MoverParameters->OperateDoors( side::left, false, range_t::local );
                MoverParameters->OperateDoors( side::right, false, range_t::local );
            }
        }
        // if the vehicle was emptied potentially switch load visualization model
        if( MoverParameters->LoadAmount == 0 ) {
            MoverParameters->AssignLoad( "" );
        }
    }
}

void TDynamicObject::LoadUpdate() {

    MoverParameters->LoadStatus &= 1 | 2; // po zakończeniu będzie równe zero
    // przeładowanie modelu ładunku
    if( MoverParameters->LoadTypeChange ) {
        // whether we succeed or not don't try more than once
        MoverParameters->LoadTypeChange = false;
        // bieżąca ścieżka do tekstur to dynamic/...
        Global.asCurrentTexturePath = asBaseDir;
        mdLoad = LoadMMediaFile_mdload( MoverParameters->LoadType.name );
        // TODO: discern from vehicle component which merely uses vehicle directory and has no animations, so it can be initialized outright
        // and actual vehicles which get their initialization after their animations are set up
        if( mdLoad != nullptr ) {
            mdLoad->Init();
        }
        // update bindings between lowpoly sections and potential load chunks placed inside them
        update_load_sections();
        // z powrotem defaultowa sciezka do tekstur
        Global.asCurrentTexturePath = std::string( paths::textures );
    }
}

void
TDynamicObject::update_load_sections() {

    SectionLoadOrder.clear();

    for( auto &section : Sections ) {

        section.load = GetSubmodelFromName( mdLoad,  section.compartment->pName );

        if( section.load != nullptr ) {
            // create entry for each load model chunk assigned to the section
            // TBD, TODO: store also pointer to chunk submodel and control its visibility more directly, instead of per-section visibility flag?
            auto loadchunkcount { section.load->count_children() };
            while( loadchunkcount-- ) {
                SectionLoadOrder.push_back( &section );
            }
            // HACK: disable automatic self-illumination threshold, at least until 3d model update
            if( MoverParameters->CompartmentLights.start_type == start_t::manual ) {
                section.load->SetSelfIllum( 2.0f, true, false );
            }
        }
    }
    shuffle_load_order();
}

void
TDynamicObject::update_load_visibility() {
    // start with clean load chunk visibility slate
    for( auto &section : Sections ) {
        section.load_chunks_visible = 0;
    }
    // each entry in load order sequence matches a single chunk of the section it points to
    // the length of load order sequence matches total number of load chunks
    auto const loadpercentage { (
        MoverParameters->MaxLoad == 0.f ?
            0.0 :
            MoverParameters->LoadAmount / MoverParameters->MaxLoad ) };
    auto visiblechunkcount { (
        SectionLoadOrder.empty() ?
            0 :
            static_cast<int>( std::ceil( loadpercentage * SectionLoadOrder.size() - 0.001f ) ) ) };
    for( auto *section : SectionLoadOrder ) {
        if( visiblechunkcount == 0 ) { break; }
        section->load_chunks_visible++;
        --visiblechunkcount;
    }
}

void
TDynamicObject::update_load_offset() {

    if( MoverParameters->LoadType.offset_min == 0.f ) { return; }

    auto const loadpercentage { (
        MoverParameters->MaxLoad == 0.f ?
            0.0 :
            100.0 * MoverParameters->LoadAmount / MoverParameters->MaxLoad ) };

    LoadOffset = std::lerp( MoverParameters->LoadType.offset_min, 0.f, safe_clamp( 0.0, 1.0, loadpercentage * 0.01 ) );
}

void
TDynamicObject::shuffle_load_order() {

    // order of visible load chunks, nothing security related -- see the engines in Globals.h
    std::shuffle( std::begin( SectionLoadOrder ), std::end( SectionLoadOrder ), Global.random_engine ); // NOSONAR
    // shift chunks assigned to corridors to the end of the list, so they show up last
    std::stable_partition(
        std::begin( SectionLoadOrder ), std::end( SectionLoadOrder ),
        []( vehicle_section const *section ) {
            return section->compartment->pName.find("compartment") == 0 || section->compartment->pName.find("przedzial") == 0; } );
    // NOTE: potentially we're left with a mix of corridor and external section loads
    // but that's not necessarily a wrong outcome, so we leave it this way for the time being
}
