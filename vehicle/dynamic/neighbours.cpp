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

// the vehicles near a vehicle on its track, found for the couplings and the collisions

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

// ABu 29.01.05 koniec przeklejenia *************************************

TDynamicObject * TDynamicObject::ABuFindNearestObject(glm::vec3 pos, TTrack *Track, TDynamicObject *MyPointer, int &CouplNr)
{
	// zwraca wskaznik do obiektu znajdujacego sie na torze (Track), którego sprzęg jest najblizszy punktowi
    // służy np. do łączenia i rozpinania sprzęgów
    // WE: Track      - tor, na ktorym odbywa sie poszukiwanie
    //    MyPointer  - wskaznik do obiektu szukajacego
    // WY: CouplNr    - który sprzęg znalezionego obiektu jest bliższy kamerze

    // Uwaga! Jesli CouplNr==-2 to szukamy njblizszego obiektu, a nie sprzegu!!!
    for( auto dynamic : Track->Dynamics ) {

        if( CouplNr == -2 ) {
            // wektor [kamera-obiekt] - poszukiwanie obiektu
			// length2 is better than length for comparing because it does not require sqrt function
			if (glm::length2(glm::dvec3(pos) - dynamic->vPosition) < sq(10.0))
			{
                // 10 metrów
                return dynamic;
            }
        }
        else {
            // jeśli (CouplNr) inne niz -2, szukamy sprzęgu
			// length2 is better than length for comparing because it does not require sqrt function
			if (glm::length2(glm::dvec3(pos) - dynamic->vCoulpler[0]) < sq(5.0)) {
                // 5 metrów
                CouplNr = 0;
                return dynamic;
            }
			// length2 is better than length for comparing because it does not require sqrt function
			if (glm::length2(glm::dvec3(pos) - dynamic->vCoulpler[1]) < sq(5.0)) {
                // 5 metrów
                CouplNr = 1;
                return dynamic;
            }
        }
    }
    // empty track or nothing found
    return nullptr;
}

TDynamicObject * TDynamicObject::ABuScanNearestObject(glm::vec3 pos, TTrack *Track, double ScanDir, double ScanDist, int &CouplNr)
{ // skanowanie toru w poszukiwaniu obiektu najblizszego punktowi
    if (ABuGetDirection() < 0)
        ScanDir = -ScanDir;
    TDynamicObject *FoundedObj;
    FoundedObj =
	    ABuFindNearestObject(pos, Track, this, CouplNr); // zwraca numer sprzęgu znalezionego pojazdu
    if (FoundedObj == nullptr && Track != nullptr)
    {
        double ActDist; // Przeskanowana odleglosc.
        double CurrDist = 0; // Aktualna dlugosc toru.
        if (ScanDir >= 0)
            ActDist =
                Track->Length() - RaTranslationGet(); //???-przesunięcie wózka względem Point1 toru
        else
            ActDist = RaTranslationGet(); // przesunięcie wózka względem Point1 toru
        while (ActDist < ScanDist)
        {
            ActDist += CurrDist;
            if (ScanDir > 0) // do przodu
            {
                if (Track->iNextDirection) // NOSONAR
                {
                    Track = Track->CurrentNext();
                    ScanDir = -ScanDir;
                }
                else
                    Track = Track->CurrentNext();
            }
            else // do tyłu
            {
                if (Track->iPrevDirection) // NOSONAR
                    Track = Track->CurrentPrev();
                else
                {
                    Track = Track->CurrentPrev();
                    ScanDir = -ScanDir;
                }
            }
            if (Track != nullptr)
            { // jesli jest kolejny odcinek toru
                CurrDist = Track->Length();
				FoundedObj = ABuFindNearestObject(pos, Track, this, CouplNr);
                if (FoundedObj != nullptr)
                    ActDist = ScanDist;
            }
            else // Jesli nie ma, to wychodzimy.
                break;
        }
    } // Koniec szukania najblizszego toru z jakims obiektem.
    return FoundedObj;
}

// ABu 01.11.04 poczatek wyliczania przechylow pudla **********************
void TDynamicObject::ABuModelRoll()
{ // ustawienie przechyłki pojazdu i jego
    // zawartości
    // Ra: przechyłkę załatwiamy na etapie przesuwania modelu
}

// ABu 06.05.04 poczatek wyliczania obrotow wozkow **********************

void TDynamicObject::ABuBogies()
{ // Obracanie wozkow na zakretach. Na razie
    // uwzględnia tylko zakręty,
    // bez zadnych gorek i innych przeszkod.
    if( smBogie[ 0 ] != nullptr && smBogie[ 1 ] != nullptr && MoverParameters->OffTrack.active() ) {
        // off the track the bogies stay straight under the body, they don't follow the track any more
        bogieRot[ 0 ] = glm::vec3 { 0.f };
        bogieRot[ 1 ] = glm::vec3 { 0.f };
        smBogie[ 0 ]->SetRotateXYZ( bogieRot[ 0 ] );
        smBogie[ 1 ]->SetRotateXYZ( bogieRot[ 1 ] );
        return;
    }
    if (smBogie[0] != nullptr && smBogie[1] != nullptr)
    {
        // modelRot.z=ABuAcos(Axle0.pPosition-Axle1.pPosition); //kąt obrotu pojazdu
        // [rad]
        // bogieRot[0].z=ABuAcos(Axle0.pPosition-Axle3.pPosition);
        bogieRot[0].z = Axle0.vAngles.z;
		bogieRot[0] = glm::degrees(modelRot - bogieRot[0]); // mnożenie wektora przez stałą
        smBogie[0]->SetRotateXYZ(bogieRot[0]);
        // bogieRot[1].z=ABuAcos(Axle2.pPosition-Axle1.pPosition);
        bogieRot[1].z = Axle1.vAngles.z;
		bogieRot[1] = glm::degrees(modelRot - bogieRot[1]);
        smBogie[1]->SetRotateXYZ(bogieRot[1]);
    }
};
// ABu 06.05.04 koniec wyliczania obrotow wozkow ************************

// ABu 16.03.03 sledzenie toru przed obiektem: **************************
void TDynamicObject::ABuCheckMyTrack()
{ // Funkcja przypisujaca obiekt
    // prawidlowej tablicy Dynamics,
    // bo gdzies jest jakis blad i wszystkie obiekty z danego
    // pociagu na poczatku stawiane sa na jednym torze i wpisywane
    // do jednej tablicy. Wykonuje sie tylko raz - po to 'ABuChecked'
    TTrack *OldTrack = MyTrack;
    TTrack *NewTrack = Axle0.GetTrack();
    if( NewTrack != OldTrack ) {
        if( OldTrack ) {
            OldTrack->RemoveDynamicObject( this );
        }
        NewTrack->AddDynamicObject(this);
    }
    iAxleFirst = 0; // pojazd powiązany z przednią osią - Axle0
}

// Ra: w poniższej funkcji jest problem ze sprzęgami
TDynamicObject *
TDynamicObject::ABuFindObject( int &Foundcoupler, double &Distance, TTrack const *Track, int const Direction, int const Mycoupler, bool const InLine ) const
{ // Zwraca wskaźnik najbliższego obiektu znajdującego się
    // na torze w określonym kierunku, ale tylko wtedy, kiedy
    // obiekty mogą się zderzyć, tzn. nie mijają się.
    // WE:
    //    Track     - tor, na ktorym odbywa sie poszukiwanie,
    //    Direction - kierunek szukania na torze (+1:w stronę Point2, -1:w stronę Point1)
    //    Mycoupler - nr sprzegu obiektu szukajacego;
    // WY:
    //    wskaznik do znalezionego obiektu.
    //    Foundcoupler - nr sprzegu znalezionego obiektu
    //    Distance - distance to found object

    if( true == Track->Dynamics.empty() ) {
        // sens szukania na tym torze jest tylko, gdy są na nim pojazdy
        Distance += Track->Length(); // doliczenie długości odcinka do przeskanowanej odległości
        return nullptr; // nie ma pojazdów na torze, to jest NULL
    }

    double distance = Track->Length(); // najmniejsza znaleziona odleglość (zaczynamy od długości toru)
    double myposition; // pozycja szukającego na torze
    TDynamicObject *foundobject = nullptr;
    if( MyTrack == Track ) {
        // gdy szukanie na tym samym torze
        myposition = RaTranslationGet(); // położenie wózka względem Point1 toru
    }
    else {
        // gdy szukanie na innym torze
        if( Direction > 0 ) {
            // szukanie w kierunku Point2 (od zera) - jesteśmy w Point1
            myposition = 0;
        }
        else {
            // szukanie w kierunku Point1 (do zera) - jesteśmy w Point2
            myposition = distance;
        }
    }

    double objectposition; // robocza odległość od kolejnych pojazdów na danym odcinku
    for( auto dynamic : Track->Dynamics ) {

        if( dynamic == this ) { continue; } // szukający się nie liczy
        // physics.derailment new: one of the two off the track beside the other one, they don't meet
        if( InLine && ( MoverParameters->OffTrack.active() || dynamic->MoverParameters->OffTrack.active() ) && false == in_line_with( *dynamic ) ) {
            continue;
        }
/*
        if( ( ( dynamic == PrevConnected ) && ( MoverParameters->Couplers[ TMoverParameters::side::front ].CouplingFlag != Coupling::none ) )
         || ( ( dynamic == NextConnected ) && ( MoverParameters->Couplers[ TMoverParameters::side::rear  ].CouplingFlag != Coupling::none ) )  ){
            // stop-gap check to prevent 'detection' of attached vehicles
            // which seems to be a side-effect of inaccurate location calculation in 'simple' mode?
            // TODO: investigate actual cause
            continue;
        }
*/
        if( Direction > 0 ) {
            // jeśli szukanie w kierunku Point2
            objectposition = dynamic->RaTranslationGet() - myposition; // odległogłość tamtego od szukającego
            if( objectposition > 0
             && objectposition < distance ) {
                // gdy jest po właściwej stronie i bliżej niż jakiś wcześniejszy
                Foundcoupler = dynamic->RaDirectionGet() > 0 ? 1 : 0; // to, bo (ScanDir>=0)
            }
            else { continue; }
        }
        else {
            objectposition = myposition - dynamic->RaTranslationGet(); //???-przesunięcie wózka względem Point1 toru
            if( objectposition > 0
             && objectposition < distance ) {
                Foundcoupler = dynamic->RaDirectionGet() > 0 ? 0 : 1; // odwrotnie, bo (ScanDir<0)
            }
            else { continue; }
        }

        if( Track->iCategoryFlag & 254 ) {
            // trajektoria innego typu niż tor kolejowy
            // dla torów nie ma sensu tego sprawdzać, rzadko co jedzie po jednej szynie i się mija
            // Ra: mijanie samochodów wcale nie jest proste
            // Przesuniecie wzgledne pojazdow. Wyznaczane, zeby sprawdzic,
            // czy pojazdy faktycznie sie zderzaja (moga byc przesuniete
            // w/m siebie tak, ze nie zachodza na siebie i wtedy sie mijaja).
            double relativeoffset; // wzajemna odległość poprzeczna
            if( Foundcoupler != Mycoupler ) {
                // facing the same direction
                relativeoffset = std::abs( MoverParameters->OffsetTrackH - dynamic->MoverParameters->OffsetTrackH );
            }
            else {
                relativeoffset = std::abs( MoverParameters->OffsetTrackH + dynamic->MoverParameters->OffsetTrackH );
            }
            if( relativeoffset + relativeoffset > MoverParameters->Dim.W + dynamic->MoverParameters->Dim.W ) {
                // odległość większa od połowy sumy szerokości - kolizji nie będzie
                continue;
            }
            // jeśli zahaczenie jest niewielkie, a jest miejsce na poboczu, to zjechać na pobocze
        }
        foundobject = dynamic; // potencjalna kolizja
        distance = objectposition; // odleglość pomiędzy aktywnymi osiami pojazdów
    }

    Distance += distance; // doliczenie odległości przeszkody albo długości odcinka do przeskanowanej odległości
    return foundobject;
}
