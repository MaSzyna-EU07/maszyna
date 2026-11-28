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

// how a vehicle shows: its model, its sway and its shake

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


void TDynamicObject::ABuSetModelShake(glm::dvec3 mShake)
{
    modelShake = mShake;
};

// ABu 29.01.05 przeklejone z render i renderalpha: *********************
void TDynamicObject::ABuLittleUpdate(double ObjSqrDist)
{ // ABu290105: pozbierane i uporzadkowane powtarzajace
    // sie rzeczy z Render i RenderAlpha
    // dodatkowy warunek, if (ObjSqrDist<...) zeby niepotrzebnie nie zmianiec w
    // obiektach,
    // ktorych i tak nie widac
    // NBMX wrzesien, MC listopad: zuniwersalnione
    btnOn = false; // czy przywrócić stan domyślny po renderowaniu

    // the submodels set below are shared with whatever else is drawn with the same models
    for( auto *model : { mdModel, mdLoad, mdLowPolyInt, mdKabina } ) {
        if( model != nullptr ) {
            model->state_changed();
        }
    }
    for( auto *model : mdAttachments ) {
        if( model != nullptr ) {
            model->state_changed();
        }
    }

    if (mdLoad) // tymczasowo ładunek na poziom podłogi
        if (vFloor.z > 0.0)
            mdLoad->GetSMRoot()->SetTranslate(modelShake + vFloor);

    // model jest współdzielony przez pojazdy, a pora roku może się zmienić w trakcie symulacji
    if (Global.UpdateMaterials)
        on_season_update();

    if (ObjSqrDist < 400 * 400 ) // gdy bliżej niż 400m
    {
        for( auto &animation : pAnimations ) {
            // wykonanie kolejnych animacji
            if( ObjSqrDist < animation.fMaxDist
             && animation.yUpdate ) {
                // jeśli zdefiniowana funkcja aktualizacja animacji (położenia submodeli
                animation.yUpdate( &animation );
            }
        }
        update_slottedlevers();

        if( mdModel != nullptr
         && ObjSqrDist < 50 * 50 ) {
            // gdy bliżej niż 50m
            // ABu290105: rzucanie pudlem
            // te animacje wymagają bananów w modelach!
            mdModel->GetSMRoot()->SetTranslate(modelShake);
            if (mdKabina)
                mdKabina->GetSMRoot()->SetTranslate(modelShake);
            if (mdLoad)
                mdLoad->GetSMRoot()->SetTranslate(modelShake + vFloor);
            if (mdLowPolyInt)
                mdLowPolyInt->GetSMRoot()->SetTranslate(modelShake);
            // ABu: koniec rzucania
            // ABu011104: liczenie obrotow wozkow
            ABuBogies();
            // Mczapkie-100402: rysowanie lub nie - sprzegow
            // ABu-240105: Dodatkowy warunek: if (...).Render, zeby rysowal tylko jeden z polaczonych sprzegow
            // display _on if connected with another vehicle and the coupling owner (render flag)
            // display _xon if connected with another vehicle and not the coupling owner
            // display _xon if not connected, but equipped with coupling adapter
            // display _off if not connected, not equipped with coupling adapter or if _xon model is missing
            if( MoverParameters->Couplers[ end::front ].CouplingFlag.HasAll(Coupling::coupler) ) {
                if( MoverParameters->Couplers[ end::front ].Render ) {
                    btCoupler1.TurnOn();
                }
                else {
                    btCoupler1.TurnxOnWithOffAsFallback();
                }
                btnOn = true;
            }
            else {
                if( true == MoverParameters->Couplers[ end::front ].has_adapter() ) {
                    btCoupler1.TurnxOnWithOffAsFallback();
                    btnOn = true;
                }
            }
            if( MoverParameters->Couplers[ end::rear ].CouplingFlag.HasAll(Coupling::coupler) ) {
                if( MoverParameters->Couplers[ end::rear ].Render ) {
                    btCoupler2.TurnOn();
                }
                else {
                    btCoupler2.TurnxOnWithOffAsFallback();
                }
                btnOn = true;
            }
            else {
                if( true == MoverParameters->Couplers[ end::rear ].has_adapter() ) {
                    btCoupler2.TurnxOnWithOffAsFallback();
                    btnOn = true;
                }
            }
            //********************************************************************************
            // przewody powietrzne j.w., ABu: decyzja czy rysowac tylko na podstawie
            // 'render' - juz
            // nie
            // przewody powietrzne, yB: decyzja na podstawie polaczen w t3d
            if (Global.bnewAirCouplers)
            {
                SetPneumatic(false, false); // wczytywanie z t3d ulozenia wezykow
                SetPneumatic(true, false); // i zapisywanie do zmiennej
                SetPneumatic(true, true); // ktore z nich nalezy
                SetPneumatic(false, true); // wyswietlic w tej klatce

                if (MoverParameters->Couplers[end::front].CouplingFlag.HasAll(Coupling::brakeHose))
                {
                    switch (cp1)
                    {
                    case 1:
                        btCPneumatic1.TurnOn();
                        break;
                    case 2:
                        btCPneumatic1.TurnxOn();
                        break;
                    case 3:
                        btCPneumatic1r.TurnxOn();
                        break;
                    case 4:
                        btCPneumatic1r.TurnOn();
                        break;
                    }
                    btnOn = true;
                }

                if (MoverParameters->Couplers[end::rear].CouplingFlag.HasAll(Coupling::brakeHose))
                {
                    switch (cp2)
                    {
                    case 1:
                        btCPneumatic2.TurnOn();
                        break;
                    case 2:
                        btCPneumatic2.TurnxOn();
                        break;
                    case 3:
                        btCPneumatic2r.TurnxOn();
                        break;
                    case 4:
                        btCPneumatic2r.TurnOn();
                        break;
                    }
                    btnOn = true;
                }

                // przewody zasilajace, j.w. (yB)
                if (MoverParameters->Couplers[end::front].CouplingFlag.HasAll(Coupling::mainHose))
                {
                    switch (sp1)
                    {
                    case 1:
                        btPneumatic1.TurnOn();
                        break;
                    case 2:
                        btPneumatic1.TurnxOn();
                        break;
                    case 3:
                        btPneumatic1r.TurnxOn();
                        break;
                    case 4:
                        btPneumatic1r.TurnOn();
                        break;
                    }
                    btnOn = true;
                }

                if (MoverParameters->Couplers[end::rear].CouplingFlag.HasAll(Coupling::mainHose))
                {
                    switch (sp2)
                    {
                    case 1:
                        btPneumatic2.TurnOn();
                        break;
                    case 2:
                        btPneumatic2.TurnxOn();
                        break;
                    case 3:
                        btPneumatic2r.TurnxOn();
                        break;
                    case 4:
                        btPneumatic2r.TurnOn();
                        break;
                    }
                    btnOn = true;
                }
            }
            //*********************************************************************************/
            else // po staremu ABu'oewmu
            {
                // przewody powietrzne j.w., ABu: decyzja czy rysowac tylko na podstawie
                // 'render'
                if (MoverParameters->Couplers[end::front].CouplingFlag.HasAll(Coupling::brakeHose))
                {
                    if (MoverParameters->Couplers[end::front].Render)
                        btCPneumatic1.TurnOn();
                    else
                        btCPneumatic1r.TurnOn();
                    btnOn = true;
                }

                if (MoverParameters->Couplers[end::rear].CouplingFlag.HasAll(Coupling::brakeHose))
                {
                    if (MoverParameters->Couplers[end::rear].Render)
                        btCPneumatic2.TurnOn();
                    else
                        btCPneumatic2r.TurnOn();
                    btnOn = true;
                }

                // przewody powietrzne j.w., ABu: decyzja czy rysowac tylko na podstawie
                // 'render'
                // //yB - zasilajace
                if (MoverParameters->Couplers[0].CouplingFlag.HasAll(Coupling::mainHose))
                {
                    if (MoverParameters->Couplers[0].Render)
                        btPneumatic1.TurnOn();
                    else
                        btPneumatic1r.TurnOn();
                    btnOn = true;
                }

                if (MoverParameters->Couplers[1].CouplingFlag.HasAll(Coupling::mainHose))
                {
                    if (MoverParameters->Couplers[1].Render)
                        btPneumatic2.TurnOn();
                    else
                        btPneumatic2r.TurnOn();
                    btnOn = true;
                }
            }
            //*************************************************************/// koniec
            // wezykow
            // uginanie zderzakow
            for (int i = 0; i < 2; ++i) {

                if( MoverParameters->Couplers[ i ].has_adapter() ) {
                    // HACK: if there's coupler adapter on this side, we presume there's additional distance put between vehicles
                    // which prevents buffers from clashing against each other (or the other vehicle doesn't have buffers to begin with)
                    continue;
                }

                auto const dist { safe_clamp( MoverParameters->Couplers[ i ].Dist / 2.0, -MoverParameters->Couplers[ i ].DmaxB, 0.0 ) };

                if( dist >= 0.0 ) { continue; }

                if( smBuforLewy[ i ] ) {
                    smBuforLewy[ i ]->SetTranslate( glm::vec3( dist, 0, 0 ) );
                }
                if( smBuforPrawy[ i ] ) {
                    smBuforPrawy[ i ]->SetTranslate( glm::vec3( dist, 0, 0 ) );
                }
            }
        } // vehicle within 50m

        // Winger 160204 - podnoszenie pantografow

        // przewody sterowania ukrotnionego
        if (MoverParameters->Couplers[0].CouplingFlag.HasAll(Coupling::control))
        {
            btCCtrl1.Turn( true );
            btnOn = true;
        }
        // else btCCtrl1.TurnOff();
        if (MoverParameters->Couplers[1].CouplingFlag.HasAll(Coupling::control))
        {
            btCCtrl2.Turn( true );
            btnOn = true;
        }
        // else btCCtrl2.TurnOff();
        // McZapkie-181103: mostki przejsciowe
        if (MoverParameters->Couplers[0].CouplingFlag.HasAll(Coupling::gangway))
        {
            btCPass1.Turn( true );
            btnOn = true;
        }
        // else btCPass1.TurnOff();
        if (MoverParameters->Couplers[1].CouplingFlag.HasAll(Coupling::gangway))
        {
            btCPass2.Turn( true );
            btnOn = true;
        }
        // else btCPass2.TurnOff();
        if (MoverParameters->Power24vIsAvailable || MoverParameters->Power110vIsAvailable)
        { // sygnaly konca pociagu
            if (m_endsignals1.Active()) {
                if (TestFlag(MoverParameters->iLights[end::front], light::redmarker_left | light::redmarker_right ) ) {
                    m_endsignals1.Turn( true );
                    btnOn = true;
                }
            }
            else {
                if (TestFlag(MoverParameters->iLights[end::front], light::redmarker_left) && !HeadlightsAoff) 
                {
                    m_endsignal13.Turn( true );
                    btnOn = true;
                }
				if (TestFlag(MoverParameters->iLights[end::front], light::redmarker_right) && !HeadlightsAoff)
				{
                    m_endsignal12.Turn( true );
                    btnOn = true;
                }
            }
            if (m_endsignals2.Active()) {
                if (TestFlag(MoverParameters->iLights[end::rear], light::redmarker_left | light::redmarker_right ) ) {
                    m_endsignals2.Turn( true );
                    btnOn = true;
                }
            }
            else {
				if (TestFlag(MoverParameters->iLights[end::rear], light::redmarker_left) && !HeadlightsBoff)
				{
                    m_endsignal23.Turn( true );
                    btnOn = true;
                }
				if (TestFlag(MoverParameters->iLights[end::rear], light::redmarker_right) && !HeadlightsBoff)
				{
                    m_endsignal22.Turn( true );
                    btnOn = true;
                }
            }
        }
        // tablice blaszane:
        if (TestFlag(MoverParameters->iLights[end::front], light::rearendsignals)) {
            m_endtab1.Turn( true );
            btnOn = true;
        }
        // else btEndSignalsTab1.TurnOff();
        if (TestFlag(MoverParameters->iLights[end::rear], light::rearendsignals)) {
            m_endtab2.Turn( true );
            btnOn = true;
        }
        // destination signs
        update_destinations();
        // else btEndSignalsTab2.TurnOff();
        // McZapkie-181002: krecenie wahaczem (korzysta z kata obrotu silnika)
        if (iAnimType[ANIM_LEVERS])
            for (int i = 0; i < 4; ++i)
                if (smWahacze[i])
                    smWahacze[i]->SetRotate(float3(1, 0, 0),
                                            fWahaczeAmp * cos(MoverParameters->eAngle));

        // cooling shutters
        // NOTE: shutters display _on state when they're closed, _off otherwise
        if( true == MoverParameters->diesel.heat.water.config.shutters
         && false == MoverParameters->diesel.heat.zaluzje1 ) {
            btShutters1.Turn( true );
            btnOn = true;
        }
        if( true == MoverParameters->diesel.heat.water_aux.config.shutters
         && false == MoverParameters->diesel.heat.zaluzje2 ) {
            btShutters2.Turn( true );
            btnOn = true;
        }

        // only external models ( external_only_on / external_only_off )
		btExteriorOnly.Turn(!bDisplayCab); // display only when cab is not rendered

		if( false == bDisplayCab // edge case, lowpoly may act as a stand-in for the hi-fi cab, so make sure not to show the driver when inside
         && Mechanik != nullptr
         && Mechanik->action() != TAction::actSleep ) {
            // rysowanie figurki mechanika
			btMechanik1.Turn(MoverParameters->CabOccupied > 0);
            btMechanik2.Turn( MoverParameters->CabOccupied < 0 );
            if( MoverParameters->CabOccupied != 0 ) {
                btnOn = true;
            }
        }

    } // vehicle within 400m

    if( MoverParameters->Power24vIsAvailable || MoverParameters->Power110vIsAvailable )
    { // sygnały czoła pociagu //Ra: wyświetlamy bez
        // ograniczeń odległości, by były widoczne z
        // daleka

        if (TestFlag(MoverParameters->iLights[end::front], light::headlight_left) 
            && !HeadlightsAoff && !HighBeamLights)
        {
            if( DimHeadlights ) {
                m_headlamp13.TurnxOnWithOnAsFallback();
            }
            else {
                m_headlamp13.TurnOn();
            }
            btnOn = true;
        }
		if (TestFlag(MoverParameters->iLights[end::front], light::headlight_upper) 
            && !HeadlightsAoff)
        {
            if( DimHeadlights ) {
                m_headlamp11.TurnxOnWithOnAsFallback();
            }
            else {
                m_headlamp11.TurnOn();
            }
            btnOn = true;
        }
		if (TestFlag(MoverParameters->iLights[end::front], light::headlight_right) 
            && !HeadlightsAoff && !HighBeamLights)
        {
            if( DimHeadlights ) {
                m_headlamp12.TurnxOnWithOnAsFallback();
            }
            else {
                m_headlamp12.TurnOn();
            }
            btnOn = true;
        }
        // else btHeadSignals13.TurnOff();
		if (TestFlag(MoverParameters->iLights[end::rear], light::headlight_left) 
            && !HeadlightsBoff && !HighBeamLights)
        {
            if( DimHeadlights ) {
                m_headlamp23.TurnxOnWithOnAsFallback();
            }
            else {
                m_headlamp23.TurnOn();
            }
            btnOn = true;
        }
		if (TestFlag(MoverParameters->iLights[end::rear], light::headlight_upper) 
            && !HeadlightsBoff)
        {
            if( DimHeadlights ) {
                m_headlamp21.TurnxOnWithOnAsFallback();
            }
            else {
                m_headlamp21.TurnOn();
            }
            btnOn = true;
        }
		if (TestFlag(MoverParameters->iLights[end::rear], light::headlight_right) 
            && !HeadlightsBoff && !HighBeamLights)
        {
            if( DimHeadlights ) {
                m_headlamp22.TurnxOnWithOnAsFallback();
            }
            else {
                m_headlamp22.TurnOn();
            }
            btnOn = true;
        }
        // auxiliary lights
		if (TestFlag(MoverParameters->iLights[end::front], light::auxiliary_left) && !HeadlightsAoff)
        {
            if( DimHeadlights ) {
                m_headsignal13.TurnxOnWithOnAsFallback();
            }
            else {
                m_headsignal13.TurnOn();
            }
            btnOn = true;
        }
		if (TestFlag(MoverParameters->iLights[end::front], light::auxiliary_right) && !HeadlightsAoff)
        {
            if( DimHeadlights ) {
                m_headsignal12.TurnxOnWithOnAsFallback();
            }
            else {
                m_headsignal12.TurnOn();
            }
            btnOn = true;
        }
		if (TestFlag(MoverParameters->iLights[end::rear], light::auxiliary_left) && !HeadlightsBoff)
        {
            if( DimHeadlights ) {
                m_headsignal23.TurnxOnWithOnAsFallback();
            }
            else {
                m_headsignal23.TurnOn();
            }
            btnOn = true;
        }
		if (TestFlag(MoverParameters->iLights[end::rear], light::auxiliary_right) && !HeadlightsBoff)
        {
            if( DimHeadlights ) {
                m_headsignal22.TurnxOnWithOnAsFallback();
            }
            else {
                m_headsignal22.TurnOn();
            }
            btnOn = true;
        }

        // logika dlugich
		if (TestFlag(MoverParameters->iLights[end::front], light::headlight_left) 
            && !HeadlightsAoff && HighBeamLights)
			m_highbeam13.TurnxOnWithOnAsFallback();
		else
			m_highbeam13.TurnOff();

        if (TestFlag(MoverParameters->iLights[end::front], light::headlight_right) 
            && !HeadlightsAoff && HighBeamLights)
			m_highbeam12.TurnxOnWithOnAsFallback();
		else
			m_highbeam12.TurnOff();

        // i to samo od dupy strony
		if (TestFlag(MoverParameters->iLights[end::rear], light::headlight_left) 
            && !HeadlightsBoff && HighBeamLights)
			m_highbeam23.TurnxOnWithOnAsFallback();
		else
			m_highbeam23.TurnOff();

        if (TestFlag(MoverParameters->iLights[end::rear], light::headlight_right) 
            && !HeadlightsBoff && HighBeamLights)
			m_highbeam22.TurnxOnWithOnAsFallback();
		else
			m_highbeam22.TurnOff();
    }
    // interior light levels
    auto sectionlightcolor { glm::vec4( 1.f ) };
    bool cabsection{ true };
    for( auto const &section : Sections ) {
        if( cabsection ) {
            // check whether we're still processing cab sections
            auto const &sectionname { section.compartment->pName };
			cabsection &= sectionname.size() >= 4 && sectionname.starts_with("cab");
        }
        // TODO: add cablight devices
        auto const sectionlightlevel { section.light_level * ( cabsection ? 1.0f : MoverParameters->CompartmentLights.intensity ) };
        sectionlightcolor = glm::vec4(
            sectionlightlevel == 0.f || Global.fLuminance > section.compartment->fLight ?
		                                  glm::vec3(240.f / 255.f) : // TBD: save and restore initial submodel diffuse instead of enforcing one?
		                                  InteriorLight, // TODO: per-compartment (type) light color
            sectionlightlevel );
        section.compartment->SetLightLevel( sectionlightcolor, true );
        if( section.load != nullptr ) {
            section.load->SetLightLevel( sectionlightcolor, true );
        }
    }
    // load chunks visibility
    for( auto const &section : Sections ) {
        // section isn't guaranteed to have load model, so check that first
        if( section.load == nullptr ) { continue; }
        section.load->iVisible = section.load_chunks_visible > 0;
        // if the section root isn't visible we can skip meddling with its children
        if( false == section.load->iVisible ) { continue; }
        // if the section root is visible set the state of section chunks
        auto *sectionchunk { section.load->ChildGet() };
        auto visiblechunkcount { section.load_chunks_visible };
        while( sectionchunk != nullptr ) {
            sectionchunk->iVisible = visiblechunkcount > 0;
            --visiblechunkcount;
            sectionchunk = sectionchunk->NextGet();
        }
    }

    // driver cabs visibility
    for( int cabidx = 0; cabidx < LowPolyIntCabs.size(); ++cabidx ) {
        if( LowPolyIntCabs[ cabidx ] == nullptr ) { continue; }
        LowPolyIntCabs[ cabidx ]->iVisible = mdKabina == nullptr ? true : // there's no hi-fi cab
            bDisplayCab == false ? true : // we're in external view
            simulation::Train == nullptr ? true : // not a player-driven vehicle, implies external view
            simulation::Train->Dynamic() != this ? true : // not a player-driven vehicle, implies external view
            JointCabs ? false : // internal view, all cabs share the model so hide them 'all'
            simulation::Train->iCabn != cabidx; // internal view, hide occupied cab and show others
    }
}

void
TDynamicObject::update_shake( double const Timedelta ) {

    ShakeStateBefore = ShakeState;
    // Ra: mechanik powinien być telepany niezależnie od pozycji pojazdu
    // Ra: trzeba zrobić model bujania głową i wczepić go do pojazdu

    // Ra: tu by się przydało uwzględnić rozkład sił:
    // - na postoju horyzont prosto, kabina skosem
    // - przy szybkiej jeździe kabina prosto, horyzont pochylony

    // McZapkie: najpierw policzę pozycję w/m kabiny

    // ABu: rzucamy kabina tylko przy duzym FPS!
    // Mala histereza, zeby bez przerwy nie przelaczalo przy FPS~17
    // Granice mozna ustalic doswiadczalnie. Ja proponuje 14:20
    if( Global.iSlowMotion == 0 ) { // musi być pełna prędkość

        glm::dvec3 shakevector;
        if( MoverParameters->EngineType == TEngineType::DieselElectric
         || MoverParameters->EngineType == TEngineType::DieselEngine ) {
            if( std::abs( MoverParameters->enrot ) > 0.0 ) {
                // engine vibration
                shakevector.x +=
                    std::sin(MoverParameters->eAngle * 4.0) * Timedelta *
				                 EngineShake.scale
                    // fade in with rpm above threshold
                    * std::clamp(
                        ( MoverParameters->enrot - EngineShake.fadein_offset ) * EngineShake.fadein_factor,
                        0.0, 1.0 )
                    // fade out with rpm above threshold
                    * std::lerp(
                        1.0, 0.0,
                        std::clamp(
                            ( MoverParameters->enrot - EngineShake.fadeout_offset ) * EngineShake.fadeout_factor,
                            0.0, 1.0 ) );
            }
        }

        if( HuntingShake.fadein_begin > 0.f
         && true == MoverParameters->TruckHunting ) {
            // hunting oscillation
            HuntingAngle = clamp_circular( HuntingAngle + 4.0 * HuntingShake.frequency * Timedelta * MoverParameters->Vel, 360.0 );
            auto const huntingamount =
                std::lerp(
                    0.0, 1.0,
                    std::clamp(
                        ( MoverParameters->Vel - HuntingShake.fadein_begin ) / ( HuntingShake.fadein_end - HuntingShake.fadein_begin ),
                        0.0, 1.0 ) );
            shakevector.x +=
                std::sin(glm::radians(HuntingAngle)) * Timedelta * HuntingShake.scale
                * huntingamount;
            IsHunting = huntingamount > 0.025;
        }

		if (FreeFlyModeFlag)
            shakevector *= 0;

        auto const iVel { std::min( GetVelocity(), 150.0 ) };
        // an uneven track shakes the cab by itself, in place of the sway drawn from the curve and the jolts drawn at random, as far as it does
        auto const replaced { track_uneven::replaced_shake( MyTrack != nullptr ? MyTrack->uneven_underway() : track_uneven::amounts {} ) };
        if( iVel > 0.5 ) {
            // acceleration-driven base shake
			shakevector += glm::dvec3(
                replaced.sway ? 0.0 : -MoverParameters->AccN * Timedelta * 5.0 * Global.ShakingMultiplierRL, // highlight side sway
                -MoverParameters->AccVert * Timedelta * Global.ShakingMultiplierUD,
                -MoverParameters->AccSVBased * Timedelta * 1.5 * Global.ShakingMultiplierBF); // accent acceleration/deceleration
            // unevenness of the track under the outer axles
            auto const ride { 0.5 * ( Axle0.UnevenBend() + Axle1.UnevenBend() ) * ( MoverParameters->V * MoverParameters->V ) };
            shakevector += glm::dvec3(
                -glm::dot( ride, vLeft ) * Timedelta * 5.0 * Global.ShakingMultiplierRL,
                -ride.y * Timedelta * Global.ShakingMultiplierUD,
                0.0 );
        }

        auto shake { 1.25 * ShakeSpring.ComputateForces( shakevector, ShakeState.offset ) };

        if( false == replaced.jolts && LocalRandom( iVel ) > 25.0 ) {
            // extra shake at increased velocity
            shake += ShakeSpring.ComputateForces(
                glm::dvec3(
                ( LocalRandom( iVel * 2 ) - iVel ) / ( iVel * 2 * 4 ) * BaseShake.jolt_scale.x,
                ( LocalRandom( iVel * 2 ) - iVel ) / ( iVel * 2 * 4 ) * BaseShake.jolt_scale.y,
                ( LocalRandom( iVel * 2 ) - iVel ) / ( iVel * 2 * 4 ) * BaseShake.jolt_scale.z )
//                * (( 200 - DynamicObject->MyTrack->iQualityFlag ) * 0.0075 ) // scale to 75-150% based on track quality
                * 1.25,
                ShakeState.offset );
        }
        shake *= 0.85;

        ShakeState.velocity -= ( shake + ShakeState.velocity * 100.0 ) * (double)( BaseShake.jolt_scale.x + BaseShake.jolt_scale.y + BaseShake.jolt_scale.z ) / 200.0;

        // McZapkie:
        ShakeState.offset += ShakeState.velocity * Timedelta;
        if( std::abs( ShakeState.offset.y ) >  std::abs( BaseShake.jolt_limit ) ) {
            ShakeState.velocity.y = -ShakeState.velocity.y;
        }
    }
    else { // hamowanie rzucania przy spadku FPS
        ShakeState.offset -= ShakeState.offset * std::min( Timedelta, 1.0 ); // po tym chyba potrafią zostać jakieś ułamki, które powodują zjazd
    }
}

bool
TDynamicObject::axle_passed_joint( axle_sounds &Axle, double const Move, int const Direction ) const {

    // from the follower on its side of the vehicle, as the two can be on different tracks. They stand fAxleDist apart, at the bogie pivots, and
    // the offset of the axle is the place of its wheel set, so the two of a bogie come out half of ADist to either side of the pivot
    auto const &follower { Axle.offset >= 0.0 ? Axle0 : Axle1 };
    if( auto const railphase { follower.RailPhase( Axle.offset - std::copysign( 0.5 * fAxleDist, Axle.offset ) ) }; railphase >= 0.0 ) {
        // the rails under the axle, which aren't always those under the track the vehicle is on
        auto const passed { Axle.atjoints && std::abs( railphase - Axle.distance ) > 0.5 * follower.GetTrack()->fSoundDistance };
        Axle.distance = railphase;
        Axle.atjoints = true;
        return passed;
    }
    Axle.atjoints = false;
    Axle.distance += Move * Direction;
    if( Axle.distance >= 0 && Axle.distance <= dRailLength ) {
        return false;
    }
    Axle.distance = clamp_circular( Axle.distance, dRailLength );
    return true;
}

std::pair<double, double>
TDynamicObject::shake_angles() const {

    return shake_angles( ShakeState );
}

std::pair<double, double>
TDynamicObject::shake_angles( shake_state const &State ) const {

    return {
        std::atan( State.velocity.x * BaseShake.angle_scale.x ),
        std::atan( State.velocity.z * BaseShake.angle_scale.z ) };
}

TDynamicObject::shake_state
TDynamicObject::shake_between( double const Fraction ) const {

    if( Fraction >= 1.0 ) {
        return ShakeState;
    }
    auto const between { []( glm::dvec3 const &Before, glm::dvec3 const &After, double const Part ) {
        return glm::dvec3 {
            std::lerp( Before.x, After.x, Part ),
            std::lerp( Before.y, After.y, Part ),
            std::lerp( Before.z, After.z, Part ) }; } };
    auto const part { std::max( Fraction, 0.0 ) };
    return {
        between( ShakeStateBefore.velocity, ShakeState.velocity, part ),
        between( ShakeStateBefore.offset, ShakeState.offset, part ) };
}
