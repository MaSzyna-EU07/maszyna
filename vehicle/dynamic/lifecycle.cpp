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

// a vehicle of the scenery made, set up from its definition and removed

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

const float maxrot = (float)(M_PI / 3.0); // 60°

std::string const TDynamicObject::MED_labels[] = {
    "masa: ", "amax: ", "Fzad: ", "FmPN: ", "FmED: ", "FrED: ", "FzPN: ", "nPrF: "
};

bool TDynamicObject::bDynamicRemove { false };

// helper, locates submodel with specified name in specified 3d model; returns: pointer to the submodel, or null
TSubModel *
GetSubmodelFromName( TModel3d * const Model, std::string const Name ) {

    return Model ? Model->GetFromName(Name) : nullptr;
}

TDynamicObject::TDynamicObject() {
    modelShake = glm::dvec3(0, 0, 0);
//    fTrackBlock = 10000.0; // brak przeszkody na drodze
    btnOn = false;
    vUp = vWorldUp;
    vFront = vWorldFront;
    vLeft = vWorldLeft;
    iNumAxles = 0;
    MoverParameters = nullptr;
    Mechanik = nullptr;
    MechInside = false;
    // McZapkie-270202
    Controller = AIdriver;
    bDisplayCab = false; // 030303
//    NextConnected = PrevConnected = NULL;
//    NextConnectedNo = PrevConnectedNo = 2; // ABu: Numery sprzegow. 2=nie podłączony
    bEnabled = true;
    MyTrack = nullptr;
    // McZapkie-260202
    dWheelAngle[0] = 0.0;
    dWheelAngle[1] = 0.0;
    dWheelAngle[2] = 0.0;
    // Winger 160204 - pantografy
    // PantVolume = 3.5;
    NoVoltTime = 0;
    mdModel = nullptr;
    mdKabina = nullptr;
    // smWiazary[0]=smWiazary[1]=NULL;
    smWahacze[0] = smWahacze[1] = smWahacze[2] = smWahacze[3] = nullptr;
    fWahaczeAmp = 0;
    smBrakeMode = nullptr;
    smLoadMode = nullptr;
    mdLoad = nullptr;
    mdLowPolyInt = nullptr;
    //smMechanik0 = smMechanik1 = NULL;
    smBuforLewy[0] = smBuforLewy[1] = nullptr;
    smBuforPrawy[0] = smBuforPrawy[1] = nullptr;
    smBogie[0] = smBogie[1] = nullptr;
	bogieRot[0] = bogieRot[1] = glm::dvec3(0, 0, 0);
	modelRot = glm::dvec3(0, 0, 0);
    cp1 = cp2 = sp1 = sp2 = 0;
    iDirection = 1; // stoi w kierunku tradycyjnym (0, gdy jest odwrócony)
    iAxleFirst = 0; // numer pierwszej osi w kierunku ruchu (przełączenie
    // następuje, gdy osie sa na
    // tym samym torze)
    RaLightsSet(0, 0); // początkowe zerowanie stanu świateł
    // Ra: domyślne ilości animacji dla zgodności wstecz (gdy brak ilości podanych
    // w MMD)
	// ustawienie liczby modeli animowanych podczas konstruowania obiektu a nie na 0
	// prowadzi prosto do wysypów jeśli źle zdefiniowane mmd
    iAnimations = 0; // na razie nie ma żadnego
    pAnimated = nullptr;
    fShade = 0.0; // standardowe oświetlenie na starcie
    iHornWarning = 1; // numer syreny do użycia po otrzymaniu sygnału do jazdy
    asDestination = "none"; // stojący nigdzie nie jedzie
    pValveGear = nullptr; // Ra: tymczasowo
    iCabs = 0; // maski bitowe modeli kabin
    smBrakeSet = nullptr; // nastawa hamulca (wajcha)
    smLoadSet = nullptr; // nastawa ładunku (wajcha)
    smWiper = nullptr; // wycieraczka (poniekąd też wajcha)
    ctOwner = nullptr; // na początek niczyj
    iOverheadMask = 0; // maska przydzielana przez AI pojazdom posiadającym
    // pantograf, aby wymuszały
    // jazdę bezprądową
    tmpTraction.TractionVoltage = 0; // Ra 2F1H: prowizorka, trzeba przechować
    // napięcie, żeby nie wywalało WS pod
    // izolatorem
    fAdjustment = 0.0; // korekcja odległości pomiędzy wózkami (np. na łukach)
}

TDynamicObject::~TDynamicObject() {
    SafeDelete( Mechanik );
    SafeDelete( MoverParameters );
    SafeDeleteArray( pAnimated ); // lista animowanych submodeli
}

void TDynamicObject::place_on_track(TTrack *Track, double fDist, bool Reversed)
{
	for( auto &axle : m_axlesounds ) {
		// wyszukiwanie osi (0 jest na końcu, dlatego dodajemy długość?)
		axle.distance = (
		    Reversed ?
		         -axle.offset :
		        axle.offset + MoverParameters->Dim.L ) + fDist;
	}
	double fAxleDistHalf = fAxleDist * 0.5;
	// przesuwanie pojazdu tak, aby jego początek był we wskazanym miejcu
	fDist -= 0.5 * MoverParameters->Dim.L; // dodajemy pół długości pojazdu, bo ustawiamy jego środek (zliczanie na minus)
	switch (iNumAxles) {
	    // Ra: pojazdy wstawiane są na tor początkowy, a potem przesuwane
	case 2: // ustawianie osi na torze
		Axle0.Reset();
		Axle0.Init(Track, this, iDirection ? 1 : -1);
		Axle0.Move((iDirection ? fDist : -fDist) + fAxleDistHalf, false);
		Axle1.Reset();
		Axle1.Init(Track, this, iDirection ? 1 : -1);
		Axle1.Move((iDirection ? fDist : -fDist) - fAxleDistHalf, false); // false, żeby nie generować eventów
		break;
	}
	// the stretches the criteria of derailment held over were on the track the vehicle left (consistteleport)
	MoverParameters->derailment_monitor.reset();
	MoverParameters->derailment_distance = 0.0;
	// potrzebne do wyliczenia aktualnej pozycji; nie może być zero, bo nie przeliczy pozycji
	// teraz jeszcze trzeba przypisać pojazdy do nowego toru, bo przesuwanie początkowe osi nie
	// zrobiło tego
	Move( 0.0001 );
	ABuCheckMyTrack(); // zmiana toru na ten, co oś Axle0 (oś z przodu)
}

double
TDynamicObject::Init(std::string Name, // nazwa pojazdu, np. "EU07-424"
                     std::string BaseDir, // z którego katalogu wczytany, np. "PKP/EU07"
                     std::string asReplacableSkin, // nazwa wymiennej tekstury
                     std::string Type_Name, // nazwa CHK/MMD, np. "303E"
                     TTrack *Track, // tor początkowy wstwawienia (początek składu)
                     double fDist, // dystans względem punktu 1
                     std::string DriverType, // typ obsady
                     double fVel, // prędkość początkowa
                     std::string TrainName, // nazwa składu, np. "PE2307" albo Vmax, jeśli pliku nie ma a są cyfry
                     float Load, // ilość ładunku
                     std::string LoadType, // nazwa ładunku
                     bool Reversed, // true, jeśli ma stać odwrotnie w składzie
                     std::string MoreParams // dodatkowe parametry wczytywane w postaci tekstowej
                     )
{ // Ustawienie początkowe pojazdu
    iDirection = Reversed ? 0 : 1; // Ra: 0, jeśli ma być wstawiony jako obrócony tyłem
    asBaseDir = paths::dynamic + BaseDir + "/"; // McZapkie-310302
    asName = Name;
    // derived from the name and the seed of the simulation rather than drawn at random, so that the vehicle
    // keeps the same look for the whole session and looks the same for everyone in a multiplayer game, while
    // another run of the same scenario, with another seed, dresses it differently
    m_variantseed = hash_string( asName, Global.random_seed );
    std::string asAnimName; // zmienna robocza do wyszukiwania osi i wózków
    // a vehicle prepared for being hauled dead (parameter H after its coupling, MaSzyna-EU07/maszyna#64) has nobody
    // driving it, whatever its crew entry says: its cab can't be activated, and a driver in it would either take
    // the train or be one more for the train's driver to put to sleep
    auto const hauled { pullee_mode::requested( MoreParams ) };
    if( hauled && ( DriverType == "headdriver" || DriverType == "reardriver" ) ) {
        WriteLog( "Vehicle " + asName + " is prepared for hauling (parameter H), the driver it was given is left out" );
        DriverType = "nobody";
    }
    // Ra: zmieniamy znaczenie obsady na jednoliterowe, żeby dosadzić kierownika
    if (DriverType == "headdriver")
        DriverType = "1"; // sterujący kabiną +1
    else if (DriverType == "reardriver")
        DriverType = "2"; // sterujący kabiną -1
    else if (DriverType == "passenger")
        DriverType = ""; // legacy type, no longer needed
    else if (DriverType == "nobody")
        DriverType = ""; // nikt nie siedzi

    int Cab = 0; // numer kabiny z obsadą (nie można zająć obu)
    if (DriverType == "1") // od przodu składu
        Cab = 1; // iDirection?1:-1; //iDirection=1 gdy normalnie, =0 odwrotnie
    else if (DriverType == "2") // od tyłu składu
        Cab = -1; // iDirection?-1:1;
/*
    // NOTE: leave passenger in the middle section, this is most likely to be 'passenger' section in MU trains
    else if (DriverType == "p")
    {
        if (Random(6) < 3)
            Cab = 1;
        else
            Cab = -1; // losowy przydział kabiny
    }
*/
    // utworzenie parametrów fizyki
    MoverParameters = new TMoverParameters(iDirection ? fVel : -fVel, Type_Name, asName, Cab);
    // the vehicle carries on with the odometer state of its earlier sessions (eu07/maszyna#72)
    MoverParameters->Odometer = odometer::starting_state( odometer::states(), asName, asBaseDir );
    // McZapkie: TypeName musi byc nazwą CHK/MMD pojazdu
    if (!MoverParameters->LoadFIZ(asBaseDir))
    { // jak wczytanie CHK się nie uda, to błąd
        if (ConversionError == 666)
            ErrorLog( "Bad vehicle: failed to locate definition file \"" + BaseDir + "/" + Type_Name + ".fiz" + "\"" );
        else {
            ErrorLog( "Bad vehicle: failed to load definition from file \"" + BaseDir + "/" + Type_Name + ".fiz\" (error " + std::to_string( ConversionError ) + ")" );
        }
        return 0.0; // zerowa długość to brak pojazdu
    }
    // load the cargo now that we know whether the vehicle will allow it
    MoverParameters->AssignLoad( LoadType, Load );
    MoverParameters->ComputeMass();

    bool driveractive = fVel != 0.0; // jeśli prędkość niezerowa, to aktywujemy ruch
    if (!MoverParameters->CheckLocomotiveParameters(
            driveractive,
            (fVel > 0 ? 1 : -1) * Cab *
                (iDirection ? 1 : -1))) // jak jedzie lub obsadzony to gotowy do drogi
    {
        Error("Parameters mismatch: dynamic object " + asName + " from \"" + BaseDir + "/" + Type_Name + "\"" );
        return 0.0; // zerowa długość to brak pojazdu
    }
    // controller position
    MoverParameters->MainCtrlPos = MoverParameters->MainCtrlNoPowerPos();
    // ustawienie pozycji hamulca
    MoverParameters->LocalBrakePosA = 0.0;
    if (driveractive)
    {
        if (Cab == 0)
            MoverParameters->BrakeCtrlPos =
                static_cast<int>( std::floor(MoverParameters->Handle->GetPos(bh_NP)) );
        else
            MoverParameters->BrakeCtrlPos = static_cast<int>( std::floor(MoverParameters->Handle->GetPos(bh_RP)) );

        // engage independent brake if applicable, if the vehicle is to be on standby
        // NOTE: with more than one driver in the consist this is likely to go awery, since all but one will be sent to sleep
        // TBD, TODO: have the virtual helper release independent brakes during consist check?
        if( DriverType != "" ) {
            if( MoverParameters->LocalBrake != TLocalBrake::ManualBrake ) {
                if( fVel < 1.0 ) {
                    MoverParameters->IncLocalBrakeLevel( LocalBrakePosNo );
					if ( MoverParameters->EIMCtrlEmergency ) {
						MoverParameters->DecLocalBrakeLevel(1);
					}
                }
            }
        }
    }
    else
        MoverParameters->BrakeCtrlPos =
            static_cast<int>( std::floor(MoverParameters->Handle->GetPos(bh_NP)) );

    // poprawienie hamulca po ewentualnym przestawieniu przez Pascal
    // TODO: check if needed, we're not in Pascal anymore, Toto
    MoverParameters->BrakeLevelSet(MoverParameters->BrakeCtrlPos);

    // dodatkowe parametry yB
    MoreParams += "."; // wykonuje o jedną iterację za mało, więc trzeba mu dodać
    // kropkę na koniec
    size_t kropka = MoreParams.find("."); // znajdź kropke
    std::string ActPar; // na parametry
    while (kropka != std::string::npos) // jesli sa kropki jeszcze
    {
        size_t dlugosc = MoreParams.length();
        ActPar = ToUpper(MoreParams.substr(0, kropka)); // pierwszy parametr;
        MoreParams = MoreParams.substr(kropka + 1, dlugosc - kropka); // reszta do dalszej obrobki
        kropka = MoreParams.find(".");

        if (ActPar.substr(0, 1) == "B") // jesli hamulce
        { // sprawdzanie kolejno nastaw
            WriteLog("Wpis hamulca: " + ActPar);
            if ( contains( ActPar, 'G') )
            {
                MoverParameters->BrakeDelaySwitch(bdelay_G);
            }
			if( contains( ActPar, 'P' ) )
            {
                MoverParameters->BrakeDelaySwitch(bdelay_P);
            }
			if( contains( ActPar, 'R' ) )
            {
                MoverParameters->BrakeDelaySwitch(bdelay_R);
            }
			if( contains( ActPar, 'M' ) )
            {
                MoverParameters->BrakeDelaySwitch(bdelay_R);
                MoverParameters->BrakeDelaySwitch(bdelay_R + bdelay_M);
            }
            // wylaczanie hamulca
            if ( contains( ActPar, "<>") ) // wylaczanie na probe hamowania naglego
            {
                MoverParameters->Hamulec->SetBrakeStatus( MoverParameters->Hamulec->GetBrakeStatus() | b_dmg ); // wylacz
            }
            if ( contains( ActPar, '0' ) ) // wylaczanie na sztywno
            {
                MoverParameters->Hamulec->ForceEmptiness();
                MoverParameters->Hamulec->SetBrakeStatus( MoverParameters->Hamulec->GetBrakeStatus() | b_dmg ); // wylacz
            }
            if ( contains( ActPar, 'E' ) ) // oprozniony
            {
                MoverParameters->Hamulec->ForceEmptiness();
                MoverParameters->Pipe->CreatePress(0);
                MoverParameters->Pipe2->CreatePress(0);
            }
            if ( contains( ActPar, 'Q' ) ) // oprozniony
            {
                MoverParameters->Hamulec->ForceEmptiness();
                MoverParameters->Pipe->CreatePress(0.0);
                MoverParameters->PipePress = 0.0;
                MoverParameters->Pipe2->CreatePress(0.0);
                MoverParameters->ScndPipePress = 0.0;
                MoverParameters->PantVolume = 0.1;
                MoverParameters->PantPress = 0.0;
                MoverParameters->CompressedVolume = 0.0;
            }

            if ( contains( ActPar, '1' ) ) // wylaczanie 10%
            {
                if (Random(10) < 1) // losowanie 1/10
                {
                    MoverParameters->Hamulec->ForceEmptiness();
                    MoverParameters->Hamulec->SetBrakeStatus( MoverParameters->Hamulec->GetBrakeStatus() | b_dmg ); // wylacz
                }
            }
            if ( contains( ActPar, 'X') ) // agonalny wylaczanie 20%, usrednienie przekladni
            {
                if (Random(100) < 20) // losowanie 20/100
                {
                    MoverParameters->Hamulec->ForceEmptiness();
                    MoverParameters->Hamulec->SetBrakeStatus( MoverParameters->Hamulec->GetBrakeStatus() | b_dmg ); // wylacz
                }
                if (MoverParameters->BrakeCylMult[2] * MoverParameters->BrakeCylMult[1] >
                    0.01) // jesli jest nastawiacz mechaniczny PL
                {
                    float rnd = Random(100);
                    if (rnd < 20) // losowanie 20/100         usrednienie
                    {
                        MoverParameters->BrakeCylMult[2] = MoverParameters->BrakeCylMult[1] =
                            (MoverParameters->BrakeCylMult[2] + MoverParameters->BrakeCylMult[1]) /
                            2;
                    }
                    else if (rnd < 70) // losowanie 70/100-20/100    oslabienie
                    {
                        MoverParameters->BrakeCylMult[1] = MoverParameters->BrakeCylMult[1] * 0.50;
                        MoverParameters->BrakeCylMult[2] = MoverParameters->BrakeCylMult[2] * 0.75;
                    }
                    else if (rnd < 80) // losowanie 80/100-70/100    tylko prozny
                    {
                        MoverParameters->BrakeCylMult[2] = MoverParameters->BrakeCylMult[1];
                    }
                    else // tylko ladowny
                    {
                        MoverParameters->BrakeCylMult[1] = MoverParameters->BrakeCylMult[2];
                    }
                }
            }
            // nastawianie ladunku
            if ( contains( ActPar, 'T' ) ) // prozny
            {
                MoverParameters->DecBrakeMult();
                MoverParameters->DecBrakeMult();
            } // dwa razy w dol
            if ( contains( ActPar, 'H' ) ) // ladowny I (dla P-Ł dalej prozny)
            {
                MoverParameters->IncBrakeMult();
                MoverParameters->IncBrakeMult();
                MoverParameters->DecBrakeMult();
            } // dwa razy w gore i obniz
            if ( contains( ActPar, 'F' ) ) // ladowny II
            {
                MoverParameters->IncBrakeMult();
                MoverParameters->IncBrakeMult();
            } // dwa razy w gore
            if ( contains( ActPar, 'N' ) ) // parametr neutralny
            {
            }
        } // koniec hamulce
        else if( ActPar.size() >= 3
              && ActPar.front() == 'W' ) {
            // wheel
            ActPar.erase( 0, 1 );

            auto fixedflatsize { 0 };
            auto randomflatsize { 0 };
            auto flatchance { 100 };

            while( false == ActPar.empty() ) {
                // TODO: convert this whole copy and paste mess to something more elegant one day
                switch( ActPar.front() ) {
                    case 'F': {
                        // fixed flat size
                        auto const indexstart { 1 };
                        auto const indexend { ActPar.find_first_not_of( "1234567890", indexstart ) };
                        fixedflatsize = std::atoi( ActPar.substr( indexstart, indexend ).c_str() );
                        ActPar.erase( 0, indexend );
                        break;
                    }
                    case 'R': {
                        // random flat size
                        auto const indexstart { 1 };
                        auto const indexend { ActPar.find_first_not_of( "1234567890", indexstart ) };
                        randomflatsize = std::atoi( ActPar.substr( indexstart, indexend ).c_str() );
                        ActPar.erase( 0, indexend );
                        break;
                    }
                    case 'P': {
                        // random flat probability
                        auto const indexstart { 1 };
                        auto const indexend { ActPar.find_first_not_of( "1234567890", indexstart ) };
                        flatchance = std::atoi( ActPar.substr( indexstart, indexend ).c_str() );
                        ActPar.erase( 0, indexend );
                        break;
                    }
                    case 'H': {
                        // truck hunting
                        auto const indexstart { 1 };
                        auto const indexend { ActPar.find_first_not_of( "1234567890", indexstart ) };
                        auto const huntingchance { std::atoi( ActPar.substr( indexstart, indexend ).c_str() ) };
                        MoverParameters->TruckHunting = Random(0, 100) < huntingchance;
                        ActPar.erase( 0, indexend );
                        break;
                    }
                    default: {
                        // unrecognized key
                        ActPar.erase( 0, 1 );
                        break;
                    }
                }
            }
            if( Random( 0, 100 ) <= flatchance ) {
                MoverParameters->WheelFlat += fixedflatsize + Random( 0, randomflatsize );
            }
        } // wheel
        else if( ActPar.size() >= 2
              && ActPar.front() == 'T' ) {
            // temperature
            ActPar.erase( 0, 1 );

            auto setambient { false };

            while( false == ActPar.empty() ) {
                switch( ActPar.front() ) {
                    case 'A': {
                        // cold start, set all temperatures to ambient level
                        setambient = true;
                        ActPar.erase( 0, 1 );
                        break;
                    }
                    default: {
                        // unrecognized key
                        ActPar.erase( 0, 1 );
                        break;
                    }
                }
            }
            if( true == setambient ) {
                // TODO: pull ambient temperature from environment data
                MoverParameters->dizel_HeatSet( Global.AirTemperature );
            }
        } // temperature
        else if( ActPar.size() >= 2
              && ActPar.front() == 'L' ) {
            // load
            ActPar.erase( 0, 1 );
            // immediately followed by max load override
            // TBD: make it instead an optional sub-parameter?
            {
                auto const indexstart { 0 };
                auto const indexend { ActPar.find_first_not_of( "1234567890", indexstart ) };
                MoverParameters->MaxLoad = std::atoi( ActPar.substr( indexstart, indexend ).c_str() );
                ActPar.erase( 0, indexend );
            }
            while( false == ActPar.empty() ) {
                switch( ActPar.front() ) {
                    default: {
                        // unrecognized key
                        ActPar.erase( 0, 1 );
                        break;
                    }
                }
            }
        } // load
/*        else if (ActPar.substr(0, 1) == "") // tu mozna wpisac inny prefiks i inne rzeczy
        {
            // jakies inne prefiksy
        }
*/
    } // koniec while kropka

    if( hauled ) {
        // once the vehicle has its starting state: the hand brake and the spring brake of a standing one let go, battery off
        MoverParameters->PulleeModeSwitch( true );
    }

    if (MoverParameters->CategoryFlag & 2) // jeśli samochód
    { // ustawianie samochodow na poboczu albo na środku drogi
        if( Track->fTrackWidth < 3.5 // jeśli droga wąska
         || Track->m_road != nullptr ) // a lane of a road node is one-way, the vehicle keeps to its middle
            MoverParameters->OffsetTrackH = 0.0; // to stawiamy na środku, niezależnie od stanu
        // ruchu
        else if( driveractive ) {// od 3.5m do 8.0m jedzie po środku pasa, dla
            // szerszych w odległości
            // 1.5m
            auto const lanefreespace = 0.5 * Track->fTrackWidth - MoverParameters->Dim.W;
            MoverParameters->OffsetTrackH = lanefreespace > 1.5 ?
                -0.5 * MoverParameters->Dim.W - 0.75 : // wide road, keep near centre with safe margin
                lanefreespace > 0.1 ?
                    -0.25 * Track->fTrackWidth : // narrow fit, stay on the middle
                    -0.5 * MoverParameters->Dim.W  - 0.075; // too narrow, just keep some minimal clearance
        }
        else // jak stoi, to kołem na poboczu i pobieramy szerokość razem z
            // poboczem, ale nie z
            // chodnikiem
            MoverParameters->OffsetTrackH =
                -0.5 * (Track->WidthTotal() - MoverParameters->Dim.W) + 0.05;
        iHornWarning = 0; // nie będzie trąbienia po podaniu zezwolenia na jazdę
        if (fDist < 0.0) //-0.5*MoverParameters->Dim.L) //jeśli jest przesunięcie do tyłu
            if (!Track->CurrentPrev()) // a nie ma tam odcinka i trzeba by coś
                // wygenerować
                fDist = -fDist; // to traktujemy, jakby przesunięcie było w drugą stronę
    }

    create_controller( DriverType, !TrainName.empty() );

    ShakeSpring.Init( 125.0 );
    BaseShake = baseshake_config {};
    ShakeState = shake_state {};
    ShakeStateBefore = shake_state {};

    // McZapkie-250202
/*
    iAxles = std::min( MoverParameters->NAxles, MaxAxles ); // ilość osi
*/
    // wczytywanie z pliku nazwatypu.mmd, w tym model
    erase_extension( asReplacableSkin );
    LoadMMediaFile(Type_Name, asReplacableSkin);
    // McZapkie-100402: wyszukiwanie submodeli sprzegów
    btCoupler1.Init( "coupler1", mdModel ); // false - ma być wyłączony
    btCoupler2.Init( "coupler2", mdModel );
    // brake hoses
    btCPneumatic1.Init("cpneumatic1", mdModel);
    btCPneumatic2.Init("cpneumatic2", mdModel);
    btCPneumatic1r.Init("cpneumatic1r", mdModel);
    btCPneumatic2r.Init("cpneumatic2r", mdModel);
    // main hoses
    btPneumatic1.Init("pneumatic1", mdModel);
    btPneumatic2.Init("pneumatic2", mdModel);
    btPneumatic1r.Init("pneumatic1r", mdModel);
    btPneumatic2r.Init("pneumatic2r", mdModel);
    // control cables
    btCCtrl1.Init( "cctrl1", mdModel, false);
    btCCtrl2.Init( "cctrl2", mdModel, false);
    // gangways
    btCPass1.Init( "cpass1", mdModel, false);
    btCPass2.Init( "cpass2", mdModel, false);
    // sygnaly
    m_endsignal12.Init( "endsignal12", mdModel, false );
    m_endsignal13.Init( "endsignal13", mdModel, false );
    m_endsignal22.Init( "endsignal22", mdModel, false );
    m_endsignal23.Init( "endsignal23", mdModel, false );
    m_endsignals1.Init( "endsignals1", mdModel, false );
    m_endsignals2.Init( "endsignals2", mdModel, false );
    m_endtab1.Init( "endtab1", mdModel, false );
    m_endtab2.Init( "endtab2", mdModel, false );
    m_headlamp11.Init( "headlamp11", mdModel ); // górne
    m_headlamp12.Init( "headlamp12", mdModel ); // prawe
    m_headlamp13.Init( "headlamp13", mdModel ); // lewe
	m_highbeam12.Init("highbeam12", mdModel);   // prawe dlugie
	m_highbeam13.Init("highbeam13", mdModel);   // lewe dlugie
    m_headlamp21.Init( "headlamp21", mdModel );
    m_headlamp22.Init( "headlamp22", mdModel );
    m_headlamp23.Init( "headlamp23", mdModel );
	m_highbeam22.Init("highbeam22", mdModel);
	m_highbeam23.Init("highbeam23", mdModel);
    m_headsignal12.Init( "headsignal12", mdModel );
    m_headsignal13.Init( "headsignal13", mdModel );
    m_headsignal22.Init( "headsignal22", mdModel );
    m_headsignal23.Init( "headsignal23", mdModel );
    // informacja, czy ma poszczególne światła
    iInventory[ end::front ] |= m_endsignal12.Active() ? light::redmarker_right : 0;
    iInventory[ end::front ] |= m_endsignal13.Active() ? light::redmarker_left : 0;
    iInventory[ end::rear ]  |= m_endsignal22.Active() ? light::redmarker_right : 0;
    iInventory[ end::rear ]  |= m_endsignal23.Active() ? light::redmarker_left : 0;
    iInventory[ end::front ] |= m_endsignals1.Active() ? light::redmarker_left | light::redmarker_right : 0;
    iInventory[ end::rear ]  |= m_endsignals2.Active() ? light::redmarker_left | light::redmarker_right : 0;
    iInventory[ end::front ] |= m_endtab1.Active() ? light::rearendsignals : 0; // tabliczki blaszane
    iInventory[ end::rear ]  |= m_endtab2.Active() ? light::rearendsignals : 0;
    iInventory[ end::front ] |= m_headlamp11.Active() ? light::headlight_upper : 0;
    iInventory[ end::front ] |= m_headlamp12.Active() ? light::headlight_right : 0;
    iInventory[ end::front ] |= m_headlamp13.Active() ? light::headlight_left : 0;

    iInventory[end::front] |= m_highbeam12.Active() ? light::highbeamlight_right : 0;
	iInventory[end::front] |= m_highbeam13.Active() ? light::highbeamlight_left : 0;

    iInventory[ end::rear ]  |= m_headlamp21.Active() ? light::headlight_upper : 0;
    iInventory[ end::rear ]  |= m_headlamp22.Active() ? light::headlight_right : 0;
    iInventory[ end::rear ]  |= m_headlamp23.Active() ? light::headlight_left : 0;
    iInventory[ end::front ] |= m_headsignal12.Active() ? light::auxiliary_right : 0;
    iInventory[ end::front ] |= m_headsignal13.Active() ? light::auxiliary_left : 0;
    iInventory[ end::rear ]  |= m_headsignal22.Active() ? light::auxiliary_right : 0;
    iInventory[ end::rear ]  |= m_headsignal23.Active() ? light::auxiliary_left : 0;

    iInventory[end::rear] |= m_highbeam22.Active() ? light::highbeamlight_right : 0;
	iInventory[end::rear] |= m_highbeam23.Active() ? light::highbeamlight_left : 0;

    btExteriorOnly.Init("external_only", mdModel, false);

    btMechanik1.Init( "mechanik1", mdLowPolyInt, false);
	btMechanik2.Init( "mechanik2", mdLowPolyInt, false);

    if( MoverParameters->diesel.heat.water.config.shutters ) {
        btShutters1.Init( "shutters1", mdModel, false );
    }
    if( MoverParameters->diesel.heat.water_aux.config.shutters ) {
        btShutters2.Init( "shutters2", mdModel, false );
    }
    TurnOff(); // resetowanie zmiennych submodeli

    if( mdLowPolyInt != nullptr ) {
        // check the low poly interior for potential compartments of interest, ie ones which can be individually lit
        // TODO: definition of relevant compartments in the .mmd file
        TSubModel *submodel { nullptr };
        if( ( submodel = mdLowPolyInt->GetFromName( "cab0" ) ) != nullptr ) {
            Sections.push_back( { submodel, nullptr, 0, 0.0f } );
            LowPolyIntCabs[ 0 ] = submodel;
        }
        if( ( submodel = mdLowPolyInt->GetFromName( "cab1" ) ) != nullptr ) {
            Sections.push_back( { submodel, nullptr, 0, 0.0f } );
            LowPolyIntCabs[ 1 ] = submodel;
        }
        if( ( submodel = mdLowPolyInt->GetFromName( "cab2" ) ) != nullptr ) {
            Sections.push_back( { submodel, nullptr, 0, 0.0f } );
            LowPolyIntCabs[ 2 ] = submodel;
        }
        // passenger car compartments
        std::vector<std::string> nameprefixes = { "corridor", "korytarz", "compartment", "przedzial" };
        for( auto const &nameprefix : nameprefixes ) {
            init_sections( mdLowPolyInt, nameprefix, MoverParameters->CompartmentLights.start_type == start_t::manual );
        }
        // the inside of the door wings, if the interior has them
        init_interior_doors();
    }
    // destination sign
    if( mdModel ) {
        init_destination( mdModel );
    }
    // 'external_load' is an optional special section in the main model, pointing to submodel of external load
    if( mdModel ) {
        init_sections( mdModel, "external_load", false );
    }
    update_load_sections();
    update_load_visibility();
    // wyszukiwanie zderzakow
    if( mdModel ) {
        // jeśli ma w czym szukać
        for( int i = 0; i < 2; i++ ) {
            asAnimName = std::string( "buffer_left0" ) + std::to_string( i + 1 );
            smBuforLewy[ i ] = mdModel->GetFromName( asAnimName );
            if( smBuforLewy[ i ] )
                smBuforLewy[ i ]->WillBeAnimated(); // ustawienie flagi animacji
            asAnimName = std::string( "buffer_right0" ) + std::to_string( i + 1 );
            smBuforPrawy[ i ] = mdModel->GetFromName( asAnimName );
            if( smBuforPrawy[ i ] )
                smBuforPrawy[ i ]->WillBeAnimated();
        }
    }
    if( Track->fSoundDistance > 0.f ) {
        for( auto &axle : m_axlesounds ) {
            // wyszukiwanie osi (0 jest na końcu, dlatego dodajemy długość?)
            axle.distance =
                clamp_circular<double>(
                    ( Reversed ?
                        -axle.offset :
                         axle.offset )
                        - 0.5 * MoverParameters->Dim.L
                        + fDist,
                    Track->fSoundDistance );
        }
    }
    // McZapkie-250202 end.
    Track->AddDynamicObject(this); // wstawiamy do toru na pozycję 0, a potem przesuniemy
    // McZapkie: zmieniono na ilosc osi brane z chk
    // iNumAxles=(MoverParameters->NAxles>3 ? 4 : 2 );
    initial_track = MyTrack;
    iNumAxles = 2;
    // McZapkie-090402: odleglosc miedzy czopami skretu lub osiami
    fAxleDist = safe_clamp(
            std::max( MoverParameters->BDist, MoverParameters->ADist ),
            0.2, //żeby się dało wektory policzyć
            MoverParameters->Dim.L - 0.2 ); // nie mogą być za daleko bo będzie "walenie w mur"
    double fAxleDistHalf = fAxleDist * 0.5;
    // przesuwanie pojazdu tak, aby jego początek był we wskazanym miejcu
    fDist -= 0.5 * MoverParameters->Dim.L; // dodajemy pół długości pojazdu, bo ustawiamy jego środek (zliczanie na minus)
    switch (iNumAxles) {
        // Ra: pojazdy wstawiane są na tor początkowy, a potem przesuwane
    case 2: // ustawianie osi na torze
        Axle0.Init(Track, this, iDirection ? 1 : -1);
        Axle0.Move((iDirection ? fDist : -fDist) + fAxleDistHalf, false);
        Axle1.Init(Track, this, iDirection ? 1 : -1);
        Axle1.Move((iDirection ? fDist : -fDist) - fAxleDistHalf, false); // false, żeby nie generować eventów
        break;
    case 4:
        Axle0.Init(Track, this, iDirection ? 1 : -1);
        Axle0.Move((iDirection ? fDist : -fDist) + (fAxleDistHalf + MoverParameters->ADist * 0.5), false);
        Axle1.Init(Track, this, iDirection ? 1 : -1);
        Axle1.Move((iDirection ? fDist : -fDist) - (fAxleDistHalf + MoverParameters->ADist * 0.5), false);
        // Axle2.Init(Track,this,iDirection?1:-1);
        // Axle2.Move((iDirection?fDist:-fDist)-(fAxleDistHalf-MoverParameters->ADist*0.5),false);
        // Axle3.Init(Track,this,iDirection?1:-1);
        // Axle3.Move((iDirection?fDist:-fDist)+(fAxleDistHalf-MoverParameters->ADist*0.5),false);
        break;
    }
    // potrzebne do wyliczenia aktualnej pozycji; nie może być zero, bo nie przeliczy pozycji
    // teraz jeszcze trzeba przypisać pojazdy do nowego toru, bo przesuwanie początkowe osi nie
    // zrobiło tego
    Move( 0.0001 );
    ABuCheckMyTrack(); // zmiana toru na ten, co oś Axle0 (oś z przodu)
    // Ra: ustawienie pozycji do obliczania sprzęgów
    MoverParameters->Loc = {
        -vPosition.x,
         vPosition.z,
         vPosition.y }; // normalnie przesuwa ComputeMovement() w Update()
    // ABuWozki 060504
    if (mdModel) // jeśli ma w czym szukać
    {
        smBogie[0] = mdModel->GetFromName("bogie1"); // Ra: bo nazwy są małymi
        smBogie[1] = mdModel->GetFromName("bogie2");
        if (!smBogie[0])
            smBogie[0] = mdModel->GetFromName("boogie01"); // Ra: alternatywna nazwa
        if (!smBogie[1])
            smBogie[1] = mdModel->GetFromName("boogie02"); // Ra: alternatywna nazwa
        if (smBogie[0])
            smBogie[0]->WillBeAnimated();
        if (smBogie[1])
            smBogie[1]->WillBeAnimated();
        sm_winter_variant = mdModel->GetFromName("winter_variant");
        sm_spring_variant = mdModel->GetFromName("spring_variant");
        sm_summer_variant = mdModel->GetFromName("summer_variant");
        sm_autumn_variant = mdModel->GetFromName("autumn_variant");
    }
    // ABu: zainicjowanie zmiennej, zeby nic sie nie ruszylo w pierwszej klatce,
    // potem juz liczona prawidlowa wartosc masy
    MoverParameters->ComputeConstans();
    // wektor podłogi dla wagonów, przesuwa ładunek
	vFloor = glm::dvec3(0, 0, MoverParameters->Floor);

    // długość większa od zera oznacza OK; 2mm docisku?
    return MoverParameters->Dim.L;
}

int
TDynamicObject::init_sections( TModel3d const *Model, std::string const &Nameprefix, bool const Overrideselfillum ) {

    auto sectioncount = 0;
    auto sectionindex = 0;
    TSubModel *sectionsubmodel { nullptr };

    do {
        // section names for index < 10 match either prefix0X or prefixX
        // section names above 10 match prefixX
        auto const sectionindexname { std::to_string( sectionindex ) };
        sectionsubmodel = Model->GetFromName( Nameprefix + sectionindexname );
        if( sectionsubmodel == nullptr
         && sectionindex < 10 ) {
            sectionsubmodel = Model->GetFromName( Nameprefix + "0" + sectionindexname );
        }
        if( sectionsubmodel != nullptr ) {
            // HACK: disable automatic self-illumination threshold, at least until 3d model update
            if( Overrideselfillum ) {
                sectionsubmodel->SetSelfIllum( 2.0f, true, false );
            }
            Sections.push_back( {
                sectionsubmodel,
                nullptr, // pointers to load sections are generated afterwards
                0,
                0.0f } );
            ++sectioncount;
        }
        ++sectionindex;
    } while( sectionsubmodel != nullptr
          || sectionindex < 2 ); // chain can start from prefix00 or prefix01

    return sectioncount;
}

// a door wing of the main model is seen from the outside, lit as the exterior. its namesake in the low poly interior,
// if the model has one, follows its moves and is lit with the interior, by its compartment section if it's in one
void
TDynamicObject::init_interior_doors() {

    if( mdLowPolyInt == nullptr ) { return; }

    for( auto &animation : pAnimations ) {
        // the door animation is only set up for a wing found in the main model
        if( false == door_wings::is_wing( animation.iFlags ) ) { continue; }
        if( ( animation.yUpdate == nullptr ) || ( animation.smAnimated == nullptr ) ) { continue; }

        auto *interiorwing { mdLowPolyInt->GetFromName( animation.smAnimated->pName ) };
        if( ( interiorwing == nullptr ) || ( interiorwing == animation.smAnimated ) ) { continue; }

        interiorwing->WillBeAnimated();
        animation.smInterior = interiorwing;
    }
}

bool
TDynamicObject::init_destination( TModel3d *Model ) {

    if( Model->GetSMRoot() == nullptr ) { return false; }

    std::tie( DestinationSign.sign, DestinationSign.has_light ) = Model->GetSMRoot()->find_replacable4();

    return DestinationSign.sign != nullptr;
}

void
TDynamicObject::create_controller( std::string const Type, bool const Trainset ) {

    if( Type == "" ) { return; }

    // McZapkie-151102: rozkład jazdy czytany z pliku *.txt z katalogu w którym jest sceneria
    if( Type == "1"
     || Type == "2" ) {
        // McZapkie-110303: mechanik i rozklad tylko gdy jest obsada
        Mechanik = new TController( Controller, this, Aggressive );

        if( false == Trainset ) {
            // jeśli nie w składzie
            // załączenie rozrządu (wirtualne kabiny) itd.
            Mechanik->DirectionInitial();
            // tryb pociągowy z ustaloną prędkością (względem sprzęgów)
            Mechanik->PutCommand(
                "Timetable:",
                MoverParameters->V * 3.6 * ( iDirection ? -1.0 : 1.0 ),
                0,
                nullptr );
        }
    }
    else if( Type == "p" ) {
        // obserwator w charakterze pasażera
        // Ra: to jest niebezpieczne, bo w razie co będzie pomagał hamulcem bezpieczeństwa
        Mechanik = new TController( Controller, this, Easyman, false );
    }
}
