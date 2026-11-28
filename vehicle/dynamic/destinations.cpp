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

// the destination boards and the announcements of a vehicle

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


void TDynamicObject::destination_data::deserialize( cParser &Input ) {

    while( true == deserialize_mapping( Input ) ) {
        ; // all work done by while()
    }
}

bool TDynamicObject::destination_data::deserialize_mapping( cParser &Input ) {
    // token can be a key or block end
    auto const key { Input.getToken<std::string>( true, "\n\r\t  ,;[]" ) };

    if( true == key.empty() || key == "}" ) { return false; }

    if( key == "{" ) {
        script = Input.getToken<std::string>();
    }
    else if( key == "update:" ) {
        auto const value { Input.getToken<std::string>() };
        // TODO: implement
    }
    else if( key == "instance:" ) {
        instancing = Input.getToken<std::string>();
    }
    else if( key == "parameters:" ) {
        parameters = Input.getToken<std::string>();
    }
    else if( key == "background:" ) {
        background = Input.getToken<std::string>();
    }

    return true;
}

/*
Ra:
  Powinny być dwie funkcje wykonujące aktualizację fizyki. Jedna wykonująca
krok obliczeń, powtarzana odpowiednią liczbę razy, a druga wykonująca zbiorczą
aktualzację mniej istotnych elementów.
  Ponadto należało by ustalić odległość składów od kamery i jeśli przekracza
ona np. 10km, to traktować składy jako uproszczone, np. bez wnikania w siły
na sprzęgach, opóźnienie działania hamulca itp. Oczywiście musi mieć to pewną
histerezę czasową, aby te tryby pracy nie przełączały się zbyt szybko.
*/

void TDynamicObject::update_destinations() {

    if( DestinationSign.sign == nullptr ) { return; }

    auto const lowvoltagepower { ( MoverParameters->Power24vIsAvailable || MoverParameters->Power110vIsAvailable ) };

    DestinationSign.sign->fLight = DestinationSign.has_light && lowvoltagepower ? 2.0 : -1.0;

    // jak są 4 tekstury wymienne, to nie zmieniać rozkładem
    if( std::abs( m_materialdata.multi_textures ) >= 4 ) { return; }
    // TODO: dedicated setting to discern electronic signs, instead of fallback on light presence
    m_materialdata.replacable_skins[ 4 ] = DestinationSign.destination != null_handle && (false == DestinationSign.has_light // physical destination signs remain up until manually changed
	                                                                                    || (lowvoltagepower // lcd signs are off without power
	                                                                                        && ctOwner != nullptr)) ? // lcd signs are off for carriages without engine, potentially left on a siding
	                                         DestinationSign.destination :
	                                         DestinationSign.destination_off;

    update_script_destination();
}

void TDynamicObject::update_script_destination() {

    if( DestinationSign.script_sign == nullptr ) { return; }
    // only while the sign shows the destination it draws
    if( m_materialdata.replacable_skins[ 4 ] != DestinationSign.destination ) { return; }

    auto &queue { Application.screens() };
    if( false == queue.due( *DestinationSign.script_sign ) ) { return; }
    // read anew on every refresh, so the sign follows the timetable
    queue.submit( DestinationSign.script_sign, std::make_shared<dictionary_source>( DestinationSign.script_request ), 0 );
}

void TDynamicObject::DestinationClear() {

    if( false == DestinationSign.has_light ) { return; } // physical destination signs remain up until manually changed
    // update_destinations() shows the blank sign in place of the missing destination
    DestinationSign.destination = null_handle;
    DestinationSign.script_sign.reset();
}

void TDynamicObject::DestinationSet(std::string const &to, std::string const &numer) {
    // ustawienie stacji docelowej oraz wymiennej tekstury 4, jeśli istnieje plik
    // w zasadzie, to każdy wagon mógłby mieć inną stację docelową
    // zwłaszcza w towarowych, pod kątem zautomatyzowania maewrów albo pracy górki
    // ale to jeszcze potrwa, zanim będzie możliwe, na razie można wpisać stację z
    // rozkładu

    asDestination = to;

    if( std::abs( m_materialdata.multi_textures ) >= 4 ) { return; } // jak są 4 tekstury wymienne, to nie zmieniać rozkładem
    if( DestinationSign.sign == nullptr )                { return; } // no sign submodel, no problem
    // a sign drawn for the previous destination
    DestinationSign.script_sign.reset();

    // now see if we can find any version of the destination texture
    std::vector<std::string> const destinations = {
        numer, // try dedicated timetable sign first...
        to }; // ...then generic destination sign

    for( auto const &destination : destinations ) {

        DestinationSign.destination = DestinationFind( destination );
        if( DestinationSign.destination != null_handle ) {
            // got what we wanted, we're done here
            return;
        }
    }
    // if we didn't get static texture we might be able to make one
    if( true == destination_set_script( numer ) ) { return; }
    if( DestinationSign.script.empty() ) { return; } // no script so no way to make the texture
    if( numer == "none" )                { return; } // blank or incomplete/malformed timetable, don't bother

    DestinationSign.destination = GfxRenderer->Fetch_Material( destination_request( DestinationSign.script, DestinationSign.instancing, DestinationSign.parameters ) );
}

std::string TDynamicObject::destination_request( std::string const &Script, std::string const &Instancing, std::string const &Parameters ) const {

    auto const trainname { [this]() -> std::string {
        if( ctOwner != nullptr ) { return ctOwner->TrainName(); }
        if( Mechanik != nullptr ) { return Mechanik->TrainName(); }
        return "none"; } }; // shouldn't get here but, eh
    auto const instance { [this, &Instancing]() -> std::string {
        if( Instancing == "name" ) { return MoverParameters->Name; }
        if( Instancing == "type" ) { return MoverParameters->TypeName; }
        return "none"; } };

    std::string signrequest {
          "make:"
        + Script
        + "?"
        // timetable include
        + "$timetable=" + (
            ctOwner == nullptr ?
                MoverParameters->Name : // leading vehicle, can point to it directly
                ctOwner->Vehicle()->MoverParameters->Name ) // owned vehicle, safer to point to owner as carriages can have identical names
        // basic instancing string
        // NOTE: underscore doesn't have any magic meaning for the time being, it's just less likely to conflict with regular dictionary keys
        + "&_id1=" + trainname() };
    // TBD, TODO: replace instancing with support for variables in extra parameters string?
    if( false == Instancing.empty() ) {
        signrequest += "&_id2=" + instance();
    }
    // optionl extra parameters
    if( false == Parameters.empty() ) {
        signrequest += "&" + Parameters;
    }
    return signrequest;
}

script_screens::screen_config const *TDynamicObject::script_destination( script_language &Language ) const {

    auto const &screens { Application.screens() };
    if( false == DestinationSign.lua.script.empty() && screens.runs( script_language::lua ) ) {
        Language = script_language::lua;
        return &DestinationSign.lua;
    }
    if( false == DestinationSign.wasm.script.empty() && screens.runs( script_language::wasm ) ) {
        Language = script_language::wasm;
        return &DestinationSign.wasm;
    }
    return nullptr; // no script sign, maybe a python one
}

bool TDynamicObject::destination_set_script( std::string const &Numer ) {

    if( Numer == "none" ) { return false; } // blank or incomplete/malformed timetable, don't bother
    auto language { script_language::lua };
    auto const *scriptsign { script_destination( language ) };
    if( scriptsign == nullptr ) { return false; }

    auto const signrequest { destination_request( scriptsign->script, scriptsign->instancing, scriptsign->parameters ) };
    auto const signmaterial { GfxRenderer->Fetch_Material( signrequest ) };
    if( signmaterial == null_handle ) { return false; }
    auto &signtexture { GfxRenderer->Texture( GfxRenderer->Material( signmaterial )->GetTexture( 0 ) ) };

    // vehicles showing the same request (one train, one instance) share the screen, as they share the texture
    auto sign { Application.screens().acquire( {
        .kind = script_screen_kind::external,
        .language = language,
        .script = scriptsign->script,
        .key = signrequest,
        .texture = &signtexture,
        .frames = scriptsign->frames } ) };
    if( sign == nullptr ) { return false; }

    DestinationSign.destination = signmaterial;
    DestinationSign.script_sign = std::move( sign );
    DestinationSign.script_request = signrequest.substr( signrequest.find( '?' ) + 1 );
    return true;
}

std::shared_ptr<script_screen> TDynamicObject::draw_script_texture( std::string const &Name, material_handle const Material ) {

    if( false == Name.starts_with( "make:" ) || Material == null_handle ) { return nullptr; }
    auto const query { Name.find( '?' ) };
    auto const script { Name.substr( 5, query == std::string::npos ? std::string::npos : query - 5 ) };
    auto const language { script_language_of( script ) };
    if( false == language.has_value() ) { return nullptr; } // a python texture, made by the texture itself
    if( false == script_screens::screen_script_path( "", script ).has_value() ) {
        ErrorLog( "Bad script texture: \"" + script + "\" leads outside the game's folder" );
        return nullptr;
    }

    auto &queue { Application.screens() };
    auto &texture { GfxRenderer->Texture( GfxRenderer->Material( Material )->GetTexture( 0 ) ) };
    auto screen { queue.acquire( {
        .kind = script_screen_kind::external,
        .language = *language,
        .script = script,
        .key = Name,
        .texture = &texture,
        .frames = 0 } ) };
    if( screen == nullptr ) { return nullptr; }
    queue.submit(
        screen,
        query == std::string::npos ?
            std::make_shared<dictionary_source>() :
            std::make_shared<dictionary_source>( Name.substr( query + 1 ) ),
        0 );
    return screen;
}

material_handle TDynamicObject::DestinationFind( std::string Destination ) {

    if( Destination.empty() ) { return null_handle; }

    // destination textures are kept in the vehicle's directory so we point the current texture path there
    auto const currenttexturepath { Global.asCurrentTexturePath };
    Global.asCurrentTexturePath = asBaseDir;

    auto destinationhandle { null_handle };

    if( Destination.starts_with("make:") ) {
        // autogenerated texture
        destinationhandle = GfxRenderer->Fetch_Material( Destination );
    }
    else {
        // regular texture
        Destination = Bezogonkow( Destination ); // do szukania pliku obcinamy ogonki
        // now see if we can find any version of the texture
        std::vector<std::string> const destinations {
            Destination + '@' + MoverParameters->TypeName,
            Destination };

        for( auto const &destination : destinations ) {
            auto material = TextureTest( ToLower( destination ) );
            if( false == material.empty() ) {
                destinationhandle = GfxRenderer->Fetch_Material( material );
                break;
            }
        }
    }
    // whether we got anything, restore previous texture path
    Global.asCurrentTexturePath = currenttexturepath;

    return destinationhandle;
}

void TDynamicObject::announce( announcement_t const Announcement, bool const Chime ) {

    if( m_doorspeakers.empty() ) { return; }

    auto const *driver { (
        ctOwner != nullptr ?
            ctOwner :
            Mechanik ) };
    if( driver == nullptr ) { return; }

    auto const &timetable { driver->TrainTimetable() };
    auto const &announcements { m_pasystem.announcements };
    auto playchime { Chime };

    if( announcements[ static_cast<int>( Announcement ) ].empty() ) {
        goto followup;
    }
    // if the announcement sound was defined queue playback
    {
        sound_source stopnamesound;
        switch( Announcement ) {
            case announcement_t::approaching:
            case announcement_t::next: {
                stopnamesound = timetable.next_stop_sound();
                break;
            }
            case announcement_t::current: {
                stopnamesound = timetable.current_stop_sound();
                break;
            }
            case announcement_t::destination: {
                stopnamesound = timetable.last_stop_sound();
                break;
            }
            default: {
                break;
            }
        }
        if( stopnamesound.empty() ) {
            goto followup;
        }
        // potentially precede the announcement with a chime...
        if( true == playchime
         && false == announcements[static_cast<int>(announcement_t::chime)].empty() ) {
            m_pasystem.announcement_queue.emplace_back( announcements[ static_cast<int>( announcement_t::chime ) ] );
            playchime = false;
        }
        // ...then play the announcement itself
        m_pasystem.announcement_queue.emplace_back( announcements[ static_cast<int>( Announcement ) ] );
        m_pasystem.announcement_queue.emplace_back( stopnamesound );
    }
followup:
    // potentially follow up with another announcement
    if( Announcement == announcement_t::next ) {
        announce( announcement_t::destination, playchime );
    }
}
