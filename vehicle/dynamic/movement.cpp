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

// moving a vehicle along its track, its axles and its bogies

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


void TDynamicObject::FastMove(double fDistance)
{
    MoverParameters->dMoveLen = MoverParameters->dMoveLen + fDistance;
}

void TDynamicObject::Move(double fDistance)
{
    if( MoverParameters->OffTrack.active() ) {
        // physics.derailment new: off the track it goes on the ground
        move_off_track( fDistance );
        return;
    }
    move_axles( fDistance );
    if( fDistance != 0.0 || m_offtrackplaced ) // nie liczyć ponownie, jeśli stoi
    { // liczenie pozycji pojazdu tutaj, bo jest używane w wielu miejscach
        // (also once the vehicle off the track is back on the rails)
        m_offtrackplaced = false;
        place_by_axles();
    }
}

void TDynamicObject::move_axles(double fDistance)
{ // przesuwanie pojazdu po
    // trajektorii polega na
    // przesuwaniu poszczególnych osi
    // Ra: wartość prędkości 2km/h ma ograniczyć aktywację eventów w przypadku
    // drgań
    if (Axle0.GetTrack() == Axle1.GetTrack()) // przed przesunięciem
    { // powiązanie pojazdu z osią można zmienić tylko wtedy, gdy skrajne osie są
        // na tym samym torze
        if (MoverParameters->Vel >
            2) //|[km/h]| nie ma sensu zmiana osi, jesli pojazd drga na postoju
            iAxleFirst = MoverParameters->V >= 0.0 ?
                             1 :
                             0; //[m/s] ?1:0 - aktywna druga oś w kierunku jazdy
        // aktualnie eventy aktywuje druga oś, żeby AI nie wyłączało sobie semafora
        // za szybko
    }
    if (fDistance > 0.0)
    { // gdy ruch w stronę sprzęgu 0, doliczyć korektę do osi 1
        bEnabled &= Axle0.Move(fDistance, iAxleFirst == 0); // oś z przodu pojazdu
        bEnabled &= Axle1.Move(fDistance /*-fAdjustment*/, iAxleFirst != 0); // oś z tyłu pojazdu
    }
    else if (fDistance < 0.0)
    { // gdy ruch w stronę sprzęgu 1, doliczyć korektę do osi 0
        bEnabled &= Axle1.Move(fDistance, iAxleFirst != 0); // oś z tyłu pojazdu prusza się pierwsza
        bEnabled &= Axle0.Move(fDistance /*-fAdjustment*/, iAxleFirst == 0); // oś z przodu pojazdu
    }
    else // gf: bez wywolania Move na postoju nie ma event0
    {
        bEnabled &= Axle1.Move(fDistance, iAxleFirst != 0); // oś z tyłu pojazdu prusza się pierwsza
        bEnabled &= Axle0.Move(fDistance, iAxleFirst == 0); // oś z przodu pojazdu
    }
}

void TDynamicObject::place_by_axles()
{ // liczenie pozycji pojazdu tutaj, bo jest używane w wielu miejscach
        vPosition = 0.5 * (Axle1.pPosition + Axle0.pPosition); //środek między skrajnymi osiami
        vFront = Axle0.pPosition - Axle1.pPosition; // wektor pomiędzy skrajnymi osiami
        // Ra 2F1J: to nie jest stabilne (powoduje rzucanie taborem) i wymaga
        // dopracowania
        fAdjustment = glm::length(vFront) - fAxleDist; // na łuku będzie ujemny
        // if (fabs(fAdjustment)>0.02) //jeśli jest zbyt dużo, to rozłożyć na kilka przeliczeń (wygasza drgania?)
        //{//parę centymetrów trzeba by już skorygować; te błędy mogą się też
        // generować na ostrych łukach
        // fAdjustment*=0.5; //w jednym kroku korygowany jest ułamek błędu
        //}
        // else
        // fAdjustment=0.0;
        vFront = glm::normalize(vFront); // kierunek ustawienia pojazdu (wektor jednostkowy)
		vLeft = glm::normalize(glm::cross(glm::dvec3(vWorldUp), vFront)); // wektor poziomy w lewo,
        // normalizacja potrzebna z powodu pochylenia (vFront)
        vUp = glm::cross(vFront, vLeft); // wektor w górę, będzie jednostkowy
        modelRot.z = atan2(-vFront.x, vFront.z); // kąt obrotu pojazdu [rad]; z ABuBogies()
        auto const roll { Roll() }; // suma przechyłek
        if (roll != 0.0)
        { // wyznaczanie przechylenia tylko jeśli jest przechyłka
            // można by pobrać wektory normalne z toru...
			mMatrix = glm::rotate(glm::dmat4(1.0), roll * 0.5, vFront); // ta macierz jest potrzebna głównie do wyświetlania // obrót wzdłuż osi o przechyłkę
			vUp = glm::dvec3(mMatrix * glm::dvec4(vUp, 0.0)); // wektor w górę pojazdu (przekręcenie na przechyłce)
            // vLeft=mMatrix*DynamicObject->vLeft;
            // vUp=CrossProduct(vFront,vLeft); //wektor w górę
            // vLeft=Normalize(CrossProduct(vWorldUp,vFront)); //wektor w lewo
            vLeft = glm::normalize(glm::cross(vUp, vFront)); // wektor w lewo
            // vUp=CrossProduct(vFront,vLeft); //wektor w górę
        }
		mMatrix = BasisChange(vLeft, vUp, vFront); // to też można by od razu policzyć, ale potrzebne jest do wyświetlania // przesuwanie jest jednak rzadziej niż renderowanie
        mMatrix = glm::inverse(mMatrix); // wyliczenie macierzy dla pojazdu (potrzebna tylko do wyświetlania?)
        // if (MoverParameters->CategoryFlag&2)
        { // przesunięcia są używane po wyrzuceniu pociągu z toru
            vPosition.x += MoverParameters->OffsetTrackH * vLeft.x; // dodanie przesunięcia w bok
            vPosition.z += MoverParameters->OffsetTrackH * vLeft.z; // vLeft jest wektorem poprzecznym
            // if () na przechyłce będzie dodatkowo zmiana wysokości samochodu
            vPosition.y += MoverParameters->OffsetTrackV; // te offsety są liczone przez moverparam
        }
        // obliczanie pozycji sprzęgów do liczenia zderzeń
        auto dir = 0.5 * MoverParameters->Dim.L * vFront; // wektor sprzęgu
        vCoulpler[end::front] = vPosition + dir; // współrzędne sprzęgu na początku
        vCoulpler[end::rear] = vPosition - dir; // współrzędne sprzęgu na końcu
        // bCameraNear=
        // if (bCameraNear) //jeśli istotne są szczegóły (blisko kamery)
        { // przeliczenie cienia
            TTrack *t0 = Axle0.GetTrack(); // już po przesunięciu
            TTrack *t1 = Axle1.GetTrack();
            if (t0->eEnvironment == e_flat && t1->eEnvironment == e_flat) // może być e_bridge...
                fShade = 0.0; // standardowe oświetlenie
            else
            { // jeżeli te tory mają niestandardowy stopień zacienienia
                // (e_canyon, e_tunnel)
                if (t0->eEnvironment == t1->eEnvironment)
                {
                    switch (t0->eEnvironment)
                    { // typ zmiany oświetlenia
                    case e_canyon:
                        fShade = 0.65f;
                        break; // zacienienie w kanionie
                    case e_tunnel:
                        fShade = 0.20f;
                        break; // zacienienie w tunelu
                    }
                }
                else // dwa różne
                { // liczymy proporcję
                    double d = Axle0.GetTranslation(); // aktualne położenie na torze
                    if (Axle0.GetDirection() < 0)
                        d = t0->Length() - d; // od drugiej strony liczona długość
                    d /= fAxleDist; // rozsataw osi procentowe znajdowanie się na torze

                    float shadefrom = 1.0f, shadeto = 1.0f;
                    // NOTE, TODO: calculating brightness level is used enough times to warrant encapsulation into a function
                    switch( t0->eEnvironment ) {
                        case e_canyon: { shadeto = 0.65f; break; }
                        case e_tunnel: { shadeto = 0.2f; break; }
                        default: {break; }
                    }
                    switch( t1->eEnvironment ) {
                        case e_canyon: { shadefrom = 0.65f; break; }
                        case e_tunnel: { shadefrom = 0.2f; break; }
                        default: {break; }
                    }
                    fShade = std::lerp( shadefrom, shadeto, static_cast<float>( d ) );
/*
                    switch (t0->eEnvironment)
                    { // typ zmiany oświetlenia - zakładam, że
                    // drugi tor ma e_flat
                    case e_canyon:
                        fShade = (d * 0.65) + (1.0 - d);
                        break; // zacienienie w kanionie
                    case e_tunnel:
                        fShade = (d * 0.20) + (1.0 - d);
                        break; // zacienienie w tunelu
                    }
                    switch (t1->eEnvironment)
                    { // typ zmiany oświetlenia - zakładam, że
                    // pierwszy tor ma e_flat
                    case e_canyon:
                        fShade = d + (1.0 - d) * 0.65;
                        break; // zacienienie w kanionie
                    case e_tunnel:
                        fShade = d + (1.0 - d) * 0.20;
                        break; // zacienienie w tunelu
                    }
*/
                }
            }
        }
}

bool TDynamicObject::UpdateForce(double dt)
{
    if (!bEnabled)
        return false;
    if( dt > 0 ) {
        // wywalenie WS zależy od ustawienia kierunku
        MoverParameters->ComputeTotalForce( dt );
    }
    return true;
}

glm::dvec3 TDynamicObject::get_future_movement() const
{
	return m_future_movement;
}

void TDynamicObject::move_set(double distance)
{
	TDynamicObject *d = this;
	while( d ) {
		d->Move( distance * d->DirectionGet() );
		d = d->Next(); // pozostałe też
	    }
	d = Prev();
	while( d ) {
		d->Move( distance * d->DirectionGet() );
		d = d->Prev(); // w drugą stronę też
	}
}

bool TDynamicObject::FastUpdate(double dt)
{
    if (dt == 0.0)
        return true; // Ra: pauza
    double dDOMoveLen;
    if (!MoverParameters->PhysicActivation)
        return true; // McZapkie: wylaczanie fizyki gdy nie potrzeba

    if (!bEnabled)
        return false;

    // NOTE: coordinate system swap
    // TODO: replace with regular glm vectors
    TLocation const l {
        -vPosition.x,
         vPosition.z,
         vPosition.y };
    TRotation const r {
        0.0,
        0.0,
        modelRot.z };
    // McZapkie: parametry powinny byc pobierane z toru
    // ts.R=MyTrack->fRadius;
    // ts.Len= std::max(MoverParameters->BDist,MoverParameters->ADist);
    // ts.dHtrack=Axle1.pPosition.y-Axle0.pPosition.y;
    // ts.dHrail=((Axle1.GetRoll())+(Axle0.GetRoll()))*0.5f;
    // tp.Width=MyTrack->fTrackWidth;
    // McZapkie-250202
    // tp.friction= MyTrack->fFriction;
    // tp.CategoryFlag= MyTrack->iCategoryFlag&15;
    // tp.DamageFlag=MyTrack->iDamageFlag;
    // tp.QualityFlag=MyTrack->iQualityFlag;
    dDOMoveLen = MoverParameters->FastComputeMovement(dt, ts, tp, l, r); // ,ts,tp,tmpTraction);
    // Move(dDOMoveLen);
    // ResetdMoveLen();
    FastMove(dDOMoveLen);

    if( MoverParameters->LoadStatus ) {
        LoadUpdate(); // zmiana modelu ładunku
    }
    update_exchange( dt );

    return true; // Ra: chyba tak?
}

// calculates distance between event-starting axle and front of the vehicle
double
TDynamicObject::tracing_offset() const {

    auto const axletoend{ ( GetLength() - fAxleDist ) * 0.5 };
    return iAxleFirst ? axletoend : axletoend + iDirection * fAxleDist;
}

// TODO: compute and cache radius during vehicle initialization
double
TDynamicObject::radius() const {

    glm::vec3 diagonal(
        static_cast<float>( MoverParameters->Dim.L ),
        static_cast<float>( MoverParameters->Dim.H ),
        static_cast<float>( MoverParameters->Dim.W ) );
    return glm::length( diagonal ) * 0.5f;
}
