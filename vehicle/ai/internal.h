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

// the constants and helpers the files of the AI driver share: only they include this header

#pragma once

#include "vehicle/Driver.h"
#include "physics/mover.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "utilities/Logs.h"
#include "utilities/physicslog.h"
#include "vehicle/Train.h"
#include "vehicle/brakereleaser.h"
#include "vehicle/brakesetting.h"
#include "vehicle/brakingcurve.h"
#include "vehicle/fieldshunting.h"
#include "vehicle/handover.h"
#include "vehicle/signalpassing.h"
#include "vehicle/timetableservice.h"
#include "world/mtable.h"
#include "world/stationnames.h"
#include "vehicle/DynObj.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "world/RoadOrder.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "simulation/simulationtime.h"
#include "simulation/simulationtest.h"
#include "world/Track.h"
#include "world/station.h"
#include "input/keyboardinput.h"
#include <algorithm>
#include <format>
#include <ranges>

#define LOGVELOCITY 0
#define LOGORDERS 1
#define LOGSTOPS 1
#define LOGBACKSCAN 0
#define LOGPRESS 0


/*

Moduł obsługujący sterowanie pojazdami (składami pociągów, samochodami).
Ma działać zarówno jako AI oraz przy prowadzeniu przez człowieka. W tym
drugim przypadku jedynie informuje za pomocą napisów o tym, co by zrobił
w tym pierwszym. Obejmuje zarówno maszynistę jak i kierownika pociągu
(dawanie sygnału do odjazdu).

Przeniesiona tutaj została zawartość ai_driver.pas przerobiona na C++.
Również niektóre funkcje dotyczące składów z DynObj.cpp.

Teoria jest wtedy kiedy wszystko wiemy, ale nic nie działa.
Praktyka jest wtedy, kiedy wszystko działa, ale nikt nie wie dlaczego.
Tutaj łączymy teorię z praktyką - tu nic nie działa i nikt nie wie dlaczego…

*/

// zrobione:
// 0. pobieranie komend z dwoma parametrami
// 1. przyspieszanie do zadanej predkosci, ew. hamowanie jesli przekroczona
// 2. hamowanie na zadanym odcinku do zadanej predkosci (ze stabilizacja przyspieszenia)
// 3. wychodzenie z sytuacji awaryjnych: bezpiecznik nadmiarowy, poslizg
// 4. przygotowanie pojazdu do drogi, zmiana kierunku ruchu
// 5. dwa sposoby jazdy - manewrowy i pociagowy
// 6. dwa zestawy psychiki: spokojny i agresywny
// 7. przejscie na zestaw spokojny jesli wystepuje duzo poslizgow lub wybic nadmiarowego.
// 8. lagodne ruszanie (przedluzony czas reakcji na 2 pierwszych nastawnikach)
// 9. unikanie jazdy na oporach rozruchowych
// 10. logowanie fizyki //Ra: nie przeniesione do C++
// 11. kasowanie czuwaka/SHP
// 12. procedury wspomagajace "patrzenie" na odlegle semafory
// 13. ulepszone procedury sterowania
// 14. zglaszanie problemow z dlugim staniem na sygnale S1
// 15. sterowanie EN57
// 16. zmiana kierunku //Ra: z przesiadką po ukrotnieniu
// 17. otwieranie/zamykanie drzwi
// 18. Ra: odczepianie z zahamowaniem i podczepianie
// 19. dla Humandriver: tasma szybkosciomierza - zapis do pliku!
// 20. wybór pozycji zaworu maszynisty w oparciu o zadane opoznienie hamowania


// do zrobienia:
// 1. kierownik pociagu
// 2. madrzejsze unikanie grzania oporow rozruchowych i silnika
// 3. unikanie szarpniec, zerwania pociagu itp
// 4. obsluga innych awarii
// 5. raportowanie problemow, usterek nie do rozwiazania
// 7. samouczacy sie algorytm hamowania

// stałe
const double EasyReactionTime = 0.5; //[s] przebłyski świadomości dla zwykłej jazdy

const double PrepareTime = 2.0; //[s] przebłyski świadomości przy odpalaniu

double brake_response_horizon( TMoverParameters const *Vehicle, TBrakeSystem const Brakesystem, double const Acceleration, double const Brakea0 );

double
uphill_stopping_distance( double const Velocity, double const Gravity );

