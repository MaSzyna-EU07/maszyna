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

// coupling and uncoupling a vehicle, the adapters of its couplers

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


int TDynamicObject::DettachStatus(int dir)
{ // sprawdzenie odległości sprzęgów
    // rzeczywistych od strony (dir):
    // 0=przód,1=tył
    // Ra: dziwne, że ta funkcja nie jest używana
    if( MoverParameters->Couplers[ dir ].CouplingFlag == Coupling::none ) {
        return 0; // jeśli nic nie podłączone, to jest OK
    }
    return MoverParameters->DettachStatus(dir); // czy jest w odpowiedniej odległości?
}

int TDynamicObject::Dettach(int dir)
{ // rozłączenie sprzęgów rzeczywistych od
    // strony (dir): 0=przód,1=tył
    // zwraca maskę bitową aktualnych sprzegów (0 jeśli rozłączony)
    if (ctOwner)
    { // jeśli pojazd ma przypisany obiekt nadzorujący skład, to póki
        // są wskaźniki
        TDynamicObject *d = this;
        while (d)
        {
            d->ctOwner = nullptr; // usuwanie właściciela
            d = d->Prev();
        }
        d = Next();
        while (d)
        {
            d->ctOwner = nullptr; // usuwanie właściciela
            d = d->Next(); // i w drugą stronę
        }
    }
    if( MoverParameters->Couplers[ dir ].CouplingFlag ) {
        // odczepianie, o ile coś podłączone
        MoverParameters->Dettach( dir );
/*
        if( true == MoverParameters->Dettach( dir ) ) {
            auto *othervehicle { dir ? NextConnected : PrevConnected };
            auto const othercoupler { dir ? NextConnectedNo() : PrevConnectedNo() };
            ( othercoupler ? othervehicle->NextConnected : othervehicle->PrevConnected ) = nullptr;
            ( dir ? NextConnected : PrevConnected ) = nullptr;
        };
*/
    }
    // sprzęg po rozłączaniu (czego się nie da odpiąć
    return MoverParameters->Couplers[dir].CouplingFlag.Bits();
}

void
TDynamicObject::couple( int const Side ) {

    auto const &neighbour { MoverParameters->Neighbours[ Side ] };

    if( neighbour.vehicle == nullptr ) { return; }

    auto const &coupler { MoverParameters->Couplers[ Side ] };
    auto *othervehicle { neighbour.vehicle };
    auto *othervehicleparams{ othervehicle->MoverParameters };
    auto const &othercoupler { othervehicleparams->Couplers[ neighbour.vehicle_end ] };

    if( coupler.CouplingFlag == Coupling::none ) {
        // najpierw hak
        if( ( coupler.AllowedFlag
            & othercoupler.AllowedFlag
            & Coupling::coupler ) == Coupling::coupler ) {
            if( MoverParameters->Attach(
                    Side, neighbour.vehicle_end,
                    othervehicleparams,
                    Coupling::coupler ) ) {
                // one coupling type per key press
                return;
            }
            else {
                WriteLog( "Mechanical coupling failed." );
            }
        }
    }
    if( false == MoverParameters->Couplers[ Side ].CouplingFlag.HasAll(Coupling::brakeHose) ) {
        // pneumatyka
        if( ( coupler.AllowedFlag
            & othercoupler.AllowedFlag
            & Coupling::brakeHose ) == Coupling::brakeHose ) {
            if( MoverParameters->Attach(
                    Side, neighbour.vehicle_end,
                    othervehicleparams,
                    coupler.CouplingFlag | Coupling::brakeHose ) ) {
                SetPneumatic( Side != 0, true );
                othervehicle->SetPneumatic( Side != 0, true );
                // one coupling type per key press
                return;
            }
        }
    }
    if( false == MoverParameters->Couplers[ Side ].CouplingFlag.HasAll(Coupling::mainHose) ) {
        // zasilajacy
        if( ( coupler.AllowedFlag
            & othercoupler.AllowedFlag
            & Coupling::mainHose ) == Coupling::mainHose ) {
            if( MoverParameters->Attach(
                    Side, neighbour.vehicle_end,
                    othervehicleparams,
                    coupler.CouplingFlag | Coupling::mainHose ) ) {
                SetPneumatic( Side != 0, false );
                othervehicle->SetPneumatic( Side != 0, false );
                // one coupling type per key press
                return;
            }
        }
    }
    if( false == MoverParameters->Couplers[ Side ].CouplingFlag.HasAll(Coupling::control) ) {
        // ukrotnionko
        if( (coupler.AllowedFlag & othercoupler.AllowedFlag & Coupling::control) == Coupling::control
         && coupler.control_type == othercoupler.control_type
         && MoverParameters->CompatibleCoupling( Side, *othervehicleparams, neighbour.vehicle_end, coupler.CouplingFlag | Coupling::control ).HasAny( Coupling::control ) ) {
            if( MoverParameters->Attach(
                    Side, neighbour.vehicle_end,
                    othervehicleparams,
                    coupler.CouplingFlag | Coupling::control ) ) {
                // one coupling type per key press
                return;
            }
        }
    }
    if( false == MoverParameters->Couplers[ Side ].CouplingFlag.HasAll(Coupling::gangway) ) {
        // mostek
        if( ( coupler.AllowedFlag
            & othercoupler.AllowedFlag
            & Coupling::gangway ) == Coupling::gangway ) {
            if( MoverParameters->Attach(
                    Side, neighbour.vehicle_end,
                    othervehicleparams,
                    coupler.CouplingFlag | Coupling::gangway ) ) {
                // one coupling type per key press
                return;
            }
        }
    }
    if( false == MoverParameters->Couplers[ Side ].CouplingFlag.HasAll(Coupling::heating) ) {
        // heating
        if( ( coupler.AllowedFlag
            & othercoupler.AllowedFlag
            & Coupling::heating ) == Coupling::heating ) {
            if( MoverParameters->Attach(
                    Side, neighbour.vehicle_end,
                    othervehicleparams,
                    coupler.CouplingFlag | Coupling::heating ) ) {
                // one coupling type per key press
                return;
            }
        }
    }

}

int
TDynamicObject::uncouple( int const Side ) {

    if( DettachStatus(Side) >= 0
     || true == MoverParameters->Couplers[Side].CouplingFlag.HasAll(Coupling::permanent) ) {
        // can't uncouple, return existing coupling state
        return MoverParameters->Couplers[ Side ].CouplingFlag.Bits();
    }
    // jeżeli sprzęg niezablokowany, jest co odczepić i się da
    auto const couplingflag { Dettach( Side ) };
    return couplingflag;
}

bool
TDynamicObject::attach_coupler_adapter( int const Side, bool const Enforce ) {

    auto &coupler { MoverParameters->Couplers[ Side ] };
    // sanity check(s)
    if( coupler.type() == TCouplerType::Automatic ) { return false; }
    auto const *neighbour { MoverParameters->Neighbours[ Side ].vehicle };
    if( neighbour == nullptr
     || MoverParameters->Neighbours[Side].distance > 25.0 ) {
        // can only acquire the adapter from a nearby enough vehicle
        return false;
    }
    // TBD: empty struct instead of fallback defaults, to allow vehicles without adapter?
    auto adapterdata {
        coupleradapter_data {
            { 0.085f, 0.95f },
            "tabor/polsprzeg" } };
    if( false == neighbour->m_coupleradapter.model.empty() ) {
        // explicit coupler adapter definition overrides default parameters
        adapterdata = neighbour->m_coupleradapter;
    }
    if( MoverParameters->Neighbours[Side].distance - adapterdata.position.x < 0.5
     && false == Enforce ) {
        // arbitrary amount of free room required to install the adapter
        // NOTE: this also covers cases with established physical connection
        return false;
    }

    coupler.adapter_type = TCouplerType::Automatic;
    coupler.adapter_length = adapterdata.position.x;
    coupler.adapter_height = adapterdata.position.y;
    // audio flag, visuals update
    coupler.sounds |= sound::attachadapter;
    m_coupleradapters[ Side ] = TModelsManager::GetModel( adapterdata.model );

    return true;
}

bool
TDynamicObject::remove_coupler_adapter( int const Side ) {

    auto &coupler{ MoverParameters->Couplers[ Side ] };

    if( coupler.adapter_type == TCouplerType::NoCoupler ) { return false; }
    // TODO: sanity check(s)
    if( coupler.Connected != nullptr ) {
        // TBD: disallow instead adapter removal if it's coupled with another vehicle?
        uncouple( Side );
    }
    coupler.adapter_type = TCouplerType::NoCoupler;
    coupler.adapter_length = 0.0;
    coupler.adapter_height = 0.0;
    // audio flag, visuals update
    coupler.sounds |= sound::removeadapter;
    m_coupleradapters[ Side ] = nullptr;

    return true;
}

void TDynamicObject::AttachNext(TDynamicObject *Object, int iType)
{ // Ra: doczepia Object na końcu składu (nazwa funkcji może być myląca)
    // Ra: używane tylko przy wczytywaniu scenerii
    auto const vehicleend { iDirection };
    auto const othervehicleend { Object->iDirection ^ 1 };

    MoverParameters->Attach( vehicleend, othervehicleend, Object->MoverParameters, CouplingSet::FromBits( iType ), true, false );
    // update neighbour data for both affected vehicles
    update_neighbours();
    Object->update_neighbours();

    // potentially attach automatic coupler adapter to allow the connection
    // HACK: we're doing it after establishin actual connection, as the method needs valid neighbour data
    auto &coupler { MoverParameters->Couplers[ vehicleend ] };
    auto &othercoupler { Object->MoverParameters->Couplers[ ( othervehicleend != 2 ? othervehicleend : coupler.ConnectedNr ) ] };

    if( coupler.type() != othercoupler.type() ) {
        if( othercoupler.type() == TCouplerType::Automatic ) {
            // try to attach adapter to the vehicle
            attach_coupler_adapter(
                vehicleend,
                true );
        }
        else if( coupler.type() == TCouplerType::Automatic ) {
            // try to attach adapter to the other vehicle
            Object->attach_coupler_adapter(
                othervehicleend != 2 ? othervehicleend : coupler.ConnectedNr,
                true );
        }
        // update distance to neighbours on account of potentially attached adapter
        update_neighbours();
        Object->update_neighbours();
    }
    // potentially adjust vehicle position to avoid collision at the simulation start
    if( MoverParameters->Neighbours[ vehicleend ].distance > -0.001 ) { return; }

    Object->Move( MoverParameters->Neighbours[ vehicleend ].distance * Object->DirectionGet() );
    // HACK: manually update vehicle position as it's used by neighbour distance update we do next
    Object->MoverParameters->Loc = {
        -Object->vPosition.x,
         Object->vPosition.z,
         Object->vPosition.y };
    // update neighbour distance data after moving our vehicle
    update_neighbours();
    Object->update_neighbours();
}
