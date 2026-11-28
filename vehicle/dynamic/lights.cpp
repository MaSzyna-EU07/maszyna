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

// the lights of a vehicle and the signals they make

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


void TDynamicObject::SetLights() {
    auto const isfrontcaboccupied { MoverParameters->CabOccupied * DirectionGet() >= 0 };
	int const automaticmarkers { MoverParameters->CabActive == 0 && MoverParameters->InactiveCabFlag & activation::redmarkers
								? light::redmarker_left + light::redmarker_right : 0 };
    auto const lightpos { MoverParameters->LightsPos - 1 };
    std::array<int, 2> const cablights {
        automaticmarkers > 0 ? automaticmarkers : MoverParameters->Lights[ end::front ][ lightpos ],
        automaticmarkers > 0 ? automaticmarkers : MoverParameters->Lights[ end::rear ][ lightpos ] };
    // the custom position keeps the lights lit at coupled ends as well
    auto const ignorecouplings { MoverParameters->LightsPos == 18 };
    auto *vehicle { GetFirstDynamic( MoverParameters->CabOccupied >= 0 ? end::front : end::rear, Coupling::control ) };
    while( vehicle != nullptr ) {
        // set lights on given side if there's no coupling with another vehicle, turn them off otherwise
        auto const iscoupled { [ vehicle, ignorecouplings ]( end const Side ) {
            return false == ignorecouplings
                && vehicle->MoverParameters->Couplers[ Side ].CouplingFlag.HasAny(Coupling::coupler)
                && vehicle->MoverParameters->Neighbours[ Side ].vehicle != nullptr; } };
        auto const lights { lights_programmer::vehicle_lights(
            cablights, isfrontcaboccupied, vehicle->iDirection, { iscoupled( end::front ), iscoupled( end::rear ) } ) };
        vehicle->RaLightsSet( lights.head, lights.rear );
        vehicle = isfrontcaboccupied ? vehicle->Next(Coupling::control) : vehicle->Prev(Coupling::control);
    }
};

void TDynamicObject::RaLightsSet(int head, int rear)
{ // zapalenie świateł z przodu i z
    // tyłu, zależne od kierunku
    // pojazdu
    if (!MoverParameters)
        return; // może tego nie być na początku
    if (rear == ( light::redmarker_left | light::redmarker_right | light::rearendsignals ) )
    { // jeśli koniec pociągu, to trzeba ustalić, czy
        // jest tam czynna lokomotywa
        // EN57 może nie mieć końcówek od środka członu
        if (MoverParameters->Power > 1.0) // jeśli ma moc napędową
            if (!MoverParameters->DirActive) // jeśli nie ma ustawionego kierunku
            { // jeśli ma zarówno światła jak i końcówki, ustalić, czy jest w stanie aktywnym
                // np. lokomotywa na zimno będzie mieć końcówki a nie światła
                rear = light::rearendsignals; // tablice blaszane
                // trzeba to uzależnić od "załączenia baterii" w pojeździe
            }
        if( rear == ( light::redmarker_left | light::redmarker_right | light::rearendsignals ) ) // jeśli nadal obydwie możliwości
            if( iInventory[
                ( iDirection ?
                    end::rear :
                    end::front ) ] & ( light::redmarker_left | light::redmarker_right ) ) {
                // czy ma jakieś światła czerowone od danej strony
                rear = light::redmarker_left | light::redmarker_right; // dwa światła czerwone
            }
            else {
                rear = light::rearendsignals; // tablice blaszane
            }
    }
    // w zależności od kierunku pojazdu w składzie
    if( head >= 0 ) {
        auto const vehicleend { iDirection > 0 ? end::front : end::rear };
        MoverParameters->iLights[ vehicleend ] = head & iInventory[vehicleend];
		bool tLeft = MoverParameters->iLights[vehicleend] & (light::auxiliary_left | light::headlight_left); // roboczo czy jakiekolwiek swiatlo z lewej jest zapalone
		bool tRight = MoverParameters->iLights[vehicleend] & (light::auxiliary_right | light::headlight_right); // a tu z prawej
        if (Controller == Humandriver) {

			auto const dps{MoverParameters->currentDimPosition()};

            // domyślnie
			HighBeamLights = false;
			DimHeadlights = false;

            // obsługa OFF
			if (dps.isOff)
			{
				//MoverParameters->iLights[vehicleend] &= 0 | light::rearendsignals; // zostawiamy tylko tabliczki jesli sa
				HighBeamLights = false;
				DimHeadlights = false;
                switch (MoverParameters->CabActive)
                {
                    case end::front:
					    HeadlightsAoff = true;
                        break;
				    case end::rear:
					    HeadlightsBoff = true;
					    break;
                    default:
					    break;
                }
			}
            else
            {
				switch (MoverParameters->CabActive)
				{
				case end::front:
					HeadlightsAoff = false;
					break;
				case end::rear:
					HeadlightsBoff = false;
					break;
				default:
					break;
				}
            }

			// obsługa przyciemnionych
			DimHeadlights = dps.isDimmed;

			// obsługa długich
			if (dps.isHighBeam)
			{
				HighBeamLights = true;
            }
        }

    }
    if( rear >= 0 ) {
        auto const vehicleend{ iDirection > 0 ? end::rear : end::front };
        MoverParameters->iLights[ vehicleend ] = rear & iInventory[vehicleend];
    }
};

void TDynamicObject::SetLightDimmings()
{
	auto const vehicleend{iDirection > 0 ? end::front : end::rear};
	if (Controller == Humandriver)
	{

		auto const dps{MoverParameters->currentDimPosition()};

		// domyślnie
		HighBeamLights = false;
		DimHeadlights = false;

		// obsługa OFF
		if (dps.isOff)
		{
			// MoverParameters->iLights[vehicleend] &= 0 | light::rearendsignals; // zostawiamy tylko tabliczki jesli sa
			HighBeamLights = false;
			DimHeadlights = false;
			switch (vehicleend)
			{
			case end::front:
				HeadlightsAoff = true;
				break;
			case end::rear:
				HeadlightsBoff = true;
				break;
			default:
				break;
			}
		}
		else
		{
			switch (vehicleend)
			{
			case end::front:
				HeadlightsAoff = false;
				break;
			case end::rear:
				HeadlightsBoff = false;
				break;
			default:
				break;
			}
		}

		// obsługa przyciemnionych
		DimHeadlights = dps.isDimmed;

		// obsługa długich
		if (dps.isHighBeam)
		{
			// obsługa długich
			if (dps.isHighBeam)
			{
				HighBeamLights = true;
			}
		}
	}
}

bool TDynamicObject::has_signal_pc1_on() const {

    auto const vehicleend { iDirection > 0 ? end::front : end::rear };
    auto const equippedlights { iInventory[ vehicleend ] };
    auto const pattern { equippedlights & ( light::headlight_left | light::headlight_right | light::headlight_upper ) };
    auto const patternfallback { equippedlights & ( light::auxiliary_left | light::auxiliary_right | light::headlight_upper ) };
    auto const hasauxiliarylights { ( equippedlights & ( light::auxiliary_left | light::auxiliary_right ) ) != 0 };
    auto const activelights { MoverParameters->iLights[ vehicleend ] };

    return (pattern != 0 && activelights == pattern) || (hasauxiliarylights && activelights == patternfallback) ||
	       (pattern == 0 && patternfallback == 0 && activelights == light::rearendsignals); // pc4
}

bool TDynamicObject::has_signal_pc2_on() const {

    auto const vehicleend { iDirection > 0 ? end::front : end::rear };
    auto const equippedlights { iInventory[ vehicleend ] };
    auto const pattern { equippedlights & ( light::redmarker_left | light::headlight_right | light::headlight_upper ) };
    auto const patternfallback { equippedlights & ( light::redmarker_left | light::auxiliary_right | light::headlight_upper ) };
    auto const hasauxiliarylights { ( equippedlights & ( light::auxiliary_left | light::auxiliary_right ) ) != 0 };
    auto const activelights { MoverParameters->iLights[ vehicleend ] };

    return activelights == pattern || (hasauxiliarylights && activelights == patternfallback);
}

bool TDynamicObject::has_signal_pc5_on() const {

    auto const vehicleend { iDirection > 0 ? end::rear : end::front };
    auto const equippedlights { iInventory[ vehicleend ] };
    auto const pattern { equippedlights & ( light::redmarker_left | light::redmarker_right ) };
    auto const patternfallback { equippedlights & light::rearendsignals };
    auto const activelights { MoverParameters->iLights[ vehicleend ] };

    return (pattern != 0 && activelights == pattern) || (patternfallback != 0 && activelights == patternfallback);
}

bool TDynamicObject::has_signal_on( int const Side, int const Pattern ) const {
/*
    auto const vehicleend { iDirection > 0 ? Side : 1 - Side };
    auto const pattern { iInventory[ vehicleend ] & Pattern };

    return ( MoverParameters->iLights[ vehicleend ] == pattern );
*/
    return MoverParameters->iLights[Side] == (Pattern & iInventory[Side]);
}
