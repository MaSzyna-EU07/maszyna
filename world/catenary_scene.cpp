/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/catenary_scene.h"

#include "world/catenary_suspension.h"
#include "world/catenary_cantilever.h"
#include "world/catenary_span.h"
#include "world/catenary_layout.h"
#include "world/catenary_network.h"
#include "world/catenary_tensioner.h"
#include "world/Traction.h"
#include "model/AnimModel.h"
#include <set>
#include "simulation/simulation.h"
#include "utilities/Globals.h"

#include <algorithm>
#include <limits>
#include "utilities/Logs.h"
#include "utilities/parser.h"

#include <cmath>
#include <fstream>
#include <locale>
#include <set>
#include <sstream>

// części wysięgników w scenerii (world/catenary_scene_objects.cpp, w testach atrapa): odłączenie od pliku scenerii i usunięcie
void catenary_part_detach( TAnimModel *Model );
void catenary_part_delete( TAnimModel *Model );
// przęsła łańcuchów (tamże): Body to treść wpisu node traction od nazwy zasilacza do endtraction włącznie.
// Link: od razu połączyć z sąsiednimi przęsłami i zasilaczem (po inicjalizacji scenerii; wcześniej robi to InitTraction)
TTraction *catenary_span_create( std::string const &Name, double RangeMax, double RangeMin, std::string const &Body, bool Link );
void catenary_span_delete( TTraction *Span );

namespace {

// katalog z plikami danych sieci, z ukośnikiem na końcu
std::string Datadirectory { "data/" };

// podwieszenie powiązane z obiektami scenerii
struct scene_suspension {
    catenary::suspension data;
    std::string skin;                   // wymienna tekstura: własna wpisu albo domyślna z pliku
    TAnimModel *support { nullptr };    // konstrukcja wsporcza
    std::vector<TTraction *> messenger; // przęsła, których linę nośną trzyma
    std::vector<TTraction *> contact;   // przęsła, których przewód jezdny trzyma
    std::string file;                   // plik sieci, z którego pochodzi wpis (albo do którego trafi)
    std::vector<TAnimModel *> parts;    // części wysięgnika wstawione do scenerii
    bool removed { false };             // usunięty w edytorze
    // siodło liny nośnej (4160) ostatniego złożenia: obejma liny kotwienia środkowego staje obok niego na teowniku
    bool has_saddle { false };
    std::string saddle_model;
    catenary::cantilever_placement saddle;
    double saddle_heading { 0.0 };
    glm::dvec3 saddle_origin { 0.0 };
};

std::vector<scene_suspension> Suspensions;

// plik sieci znany scenerii
struct scene_file {
    std::string name;                   // tak, jak w dyrektywie (względem katalogu scenerii)
    std::string skin;                   // domyślna tekstura z pliku
    std::string header;                 // komentarze z początku pliku, zachowywane przy zapisie
    bool created { false };             // założony w edytorze, sceneria go jeszcze nie dołącza
    bool changed { false };             // są zmiany do zapisania
};
std::vector<scene_file> Files;

// łańcuch sieci z pliku: przęsła od podwieszenia do podwieszenia
struct scene_chain {
    catenary::chain data;
    std::string file;
    std::vector<TTraction *> spans;     // przęsło k: od podwieszenia k do k+1; nullptr gdy nie dało się go zrobić
    bool removed { false };
    // kotwienie środkowe: części na słupach sąsiednich podwieszeń, bryły (lina, V, zaciski) dla przęsła przed i za środkowym
    std::vector<TAnimModel *> middle_parts;
    std::vector<catenary::solid_vertex> middle_solid[ 2 ];
    std::size_t middle_span { static_cast<std::size_t>( -1 ) };   // przęsło przed podwieszeniem środkowym (za nim: +1)
};
std::vector<scene_chain> Chains;

// bramka z pliku: słupy i dźwigar wstawione jako modele
struct scene_gantry {
    catenary::gantry data;
    std::string file;
    std::vector<TAnimModel *> parts;
    bool removed { false };
};
std::vector<scene_gantry> Gantries;

// kotwienie ciężarowe z pliku: słup kotwowy i urządzenie naprężające; końce drutów dla łańcuchów
struct scene_anchor {
    catenary::anchor data;
    std::string file;
    std::vector<TAnimModel *> parts;
    bool removed { false };
    bool solved { false };
    double messenger[ 3 ] { 0.0, 0.0, 0.0 };   // koniec liny nośnej (uchwyt krańcowy), współrzędne scenerii
    double contact[ 3 ] { 0.0, 0.0, 0.0 };     // koniec przewodów jezdnych (środek między przewodami)
    bool has_messenger2 { false };              // druga lina nośna kończy się na swoim uchwycie (przęsło dociąga do niego dolną linę)
    double messenger2[ 3 ] { 0.0, 0.0, 0.0 };
    double temperature { 0.0 };                // temperatura, przy której złożono
    catenary::tensioner_result result;
    std::vector<catenary::solid_vertex> rope;  // lina jako bryła proceduralna (trójkąty, współrzędne scenerii); rysuje ją przęsło do kotwienia
    // obrys urządzenia dla edytora (współrzędne scenerii): czoło słupa i rolka ruchoma na wysokości rolek, góra i spód kolumny ciężarów
    double shape[ 4 ][ 3 ] { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
};
std::vector<scene_anchor> Anchors;

// bramka wieloprzęsłowa: przęsła (wpisy gantry) stykają się na wspólnych słupach - ten sam punkt (w planie do 0,05 m).
// Model wspólnego słupa stawia przęsło o najniższym numerze; true, gdy słup K przęsła Index stawia inne przęsło
bool
mast_shared_earlier( std::size_t const Index, int const K ) {
    auto const &point { Gantries[ Index ].data.points[ K ] };
    if( Gantries[ Index ].data.masts[ K ].empty() ) { return false; }
    for( std::size_t other = 0; other < Index && other < Gantries.size(); ++other ) {
        if( Gantries[ other ].removed ) { continue; }
        for( int k = 0; k < 2; ++k ) {
            if( Gantries[ other ].data.masts[ k ].empty() ) { continue; }
            auto const dx { Gantries[ other ].data.points[ k ][ 0 ] - point[ 0 ] }, dz { Gantries[ other ].data.points[ k ][ 2 ] - point[ 2 ] };
            if( dx * dx + dz * dz < 0.0025 ) { return true; }
        }
    }
    return false;
}

// przęsła, które mają słup we wskazanym punkcie (poza Except)
std::vector<std::size_t>
gantries_at( double const *Point, std::size_t const Except ) {
    std::vector<std::size_t> result;
    for( std::size_t other = 0; other < Gantries.size(); ++other ) {
        if( other == Except || Gantries[ other ].removed ) { continue; }
        for( int k = 0; k < 2; ++k ) {
            if( Gantries[ other ].data.masts[ k ].empty() ) { continue; }
            auto const dx { Gantries[ other ].data.points[ k ][ 0 ] - Point[ 0 ] }, dz { Gantries[ other ].data.points[ k ][ 2 ] - Point[ 2 ] };
            if( dx * dx + dz * dz < 0.0025 ) { result.push_back( other ); break; }
        }
    }
    return result;
}

// bramka, której słup Mast stoi w punkcie Point (oś słupa na wysokości główki szyny, w planie do 0,10 m); nullptr gdy żadna
catenary::gantry const *
gantry_at_mast( std::string const &Mast, double const *Point ) {
    for( auto const &gantry : Gantries ) {
        if( gantry.removed ) { continue; }
        for( int k = 0; k < 2; ++k ) {
            if( gantry.data.masts[ k ] != Mast ) { continue; }
            auto const dx { gantry.data.points[ k ][ 0 ] - Point[ 0 ] }, dz { gantry.data.points[ k ][ 2 ] - Point[ 2 ] };
            if( dx * dx + dz * dz < 0.01 ) { return &gantry.data; }
        }
    }
    return nullptr;
}

scene_file *
file_named( std::string const &Name ) {
    for( auto &file : Files ) { if( file.name == Name ) { return &file; } }
    return nullptr;
}
// ile podwieszeń z początku listy jest już powiązanych i złożonych
std::size_t Resolved { 0 };
// czy sceneria przeszła już inicjalizację (FirstInit); plik podwieszeń dołączony później jest wiązany od razu
bool Initialized { false };
// uwagi solvera już wypisane do logu; ta sama uwaga dla tej samej karty idzie tylko raz, żeby nie zalać logu
std::set<std::string> Notes;

// katalog wysięgników: części, zestawy ramion i schematy kart (traction_cantilevers.txt) oraz słupy i mocowania
// (traction_supports.txt); czytany przy pierwszym użyciu
catenary::cantilever_catalog const *
cantilevers() {

    static catenary::cantilever_catalog catalog;
    static bool loaded { false };
    static bool good { false };
    if( false == loaded ) {
        loaded = true;
        std::vector<std::string> errors;
        auto const filename { Datadirectory + "traction_cantilevers.txt" };
        good = catalog.load( filename, errors );
        if( false == good ) {
            ErrorLog( "Catenary: file \"" + filename + "\" is missing, cantilevers won't be assembled" );
        }
        // słupy i mocowania do słupów: osobny plik pisany ręcznie (generator go nie rusza), dopisywany do tego samego katalogu
        if( good ) {
            auto const supports { Datadirectory + "traction_supports.txt" };
            if( false == catalog.load( supports, errors ) ) {
                ErrorLog( "Catenary: file \"" + supports + "\" is missing, cantilevers on poles won't be assembled" );
            }
        }
        for( auto const &error : errors ) {
            ErrorLog( error );
        }
    }
    return ( good ? &catalog : nullptr );
}

// zasady rozstawiania konstrukcji (traction_layout.txt); czytane przy pierwszym użyciu, brak pliku = pusta tablica
catenary::layout_tables const &
layout() {

    static catenary::layout_tables tables;
    static bool loaded { false };
    if( false == loaded ) {
        loaded = true;
        std::vector<std::string> errors;
        auto const filename { Datadirectory + "traction_layout.txt" };
        if( false == tables.load( filename, errors ) ) {
            WriteLog( ( "Catenary: file \"" + filename + "\" is missing, spans won't be checked against the catalogue" ).c_str() );
        }
        for( auto const &error : errors ) { ErrorLog( error ); }
    }
    return tables;
}

std::string
text_number( double const Value, int const Precision = 4 ) {

    std::ostringstream stream;
    stream.imbue( std::locale::classic() );
    stream.setf( std::ios::fixed );
    stream.precision( Precision );
    stream << Value;
    return stream.str();
}

// składa wysięgnik podwieszenia z części katalogowych i wstawia części do scenerii jako egzemplarze modeli.
// zwraca liczbę wstawionych części; 0 gdy wysięgnika nie dało się złożyć
// Wire: oś przewodów jezdnych przy podwieszeniu, Rope: oś liny nośnej, Attach: pion przez zaczepy izolatorów,
// a przy wpisie ze słupem oś słupa na wysokości główki szyny. Umbrella: ustawienia strony parasola (tylko przy słupie);
// Probe: tylko złożenie bez wstawiania części, wynik solvera trafia tam (zwraca 1, gdy się złożył)
std::size_t
assemble( scene_suspension &Suspension, catenary::cantilever_catalog const &Catalog, glm::dvec3 const &Wire, glm::dvec3 const *Rope, glm::dvec3 const &Attach,
    catenary::cantilever_umbrella const *Umbrella = nullptr, catenary::cantilever_result *Probe = nullptr ) {

    auto const &data { Suspension.data };
    auto const label { "Bad scenario: suspension \"" + ( data.name.empty() ? std::string( "none" ) : data.name ) + "\" " };
    // przy próbnym złożeniu (edytor) przyczyna trafia do wyniku zamiast do logu
    auto const report = [ & ]( std::string const &Reason ) {
        if( Probe != nullptr ) { Probe->warnings.push_back( Reason ); }
        else { ErrorLog( label + Reason ); }
    };
    auto const *scheme { Catalog.scheme( data.card, data.execution ) };
    if( scheme == nullptr ) {
        report( "uses scheme " + std::to_string( data.card ) + " " + std::to_string( data.execution ) + " which isn't in the cantilever catalogue" );
        return 0;
    }
    // kierunek poziomy od przewodów do konstrukcji wyznacza płaszczyznę wysięgnika
    auto const towardx { Attach.x - Wire.x };
    auto const towardz { Attach.z - Wire.z };
    auto const reach { std::sqrt( towardx * towardx + towardz * towardz ) };
    if( reach < 0.5 ) {
        report( "has its attachment point less than 0.5 m from the contact wires" );
        return 0;
    }
    auto const heading { std::atan2( towardx, towardz ) };
    // lina nośna w układzie wysięgnika; bez podanego punktu jest nad przewodami na wysokości konstrukcyjnej schematu
    catenary::cantilever_input input;
    if( Rope != nullptr ) {
        input.messenger.y = ( ( Rope->x - Wire.x ) * towardx + ( Rope->z - Wire.z ) * towardz ) / reach;
        input.messenger.z = Rope->y - Wire.y;
    }
    else {
        input.messenger.z = scheme->height;
    }
    // zaczepy izolatorów na wysokościach ze schematu; zaczep odciągu idzie za liną nośną, gdy ta wisi inaczej niż w schemacie,
    // a zaczep ukośnika razem z nim tam, gdzie schemat trzyma je w stałym odstępie (karty z trzema zaczepami)
    auto const raise { input.messenger.z - scheme->height };
    input.stay_mount = { reach, scheme->mount_stay + raise };
    input.strut_mount = { reach, scheme->mount_strut + ( scheme->steady_mounted ? raise : 0.0 ) };
    input.steady_mount = { reach, scheme->mount_steady };
    // zestaw izolatorów (np. "25" - 25 kV): każdy izolator schematu NNNN-xx podmieniony na NNNN-<zestaw>, jeśli taka część jest
    if( false == data.insulators.empty() ) {
        for( auto const &role : scheme->roles ) {
            auto const *part { Catalog.part( role.second ) };
            if( part == nullptr || part->kind != "insulator" ) { continue; }
            auto const dash { role.second.find( '-' ) };
            auto const variant { role.second.substr( 0, dash == std::string::npos ? role.second.size() : dash ) + "-" + data.insulators };
            if( Catalog.part( variant ) != nullptr ) { input.overrides[ role.first ] = variant; }
            else if( Probe == nullptr ) { ErrorLog( label + "asks for insulator set \"" + data.insulators + "\", but part \"" + variant + "\" isn't in the cantilever catalogue; \"" + role.second + "\" kept" ); }
        }
    }
    for( auto const &replacement : data.overrides ) {
        input.overrides[ replacement.first ] = replacement.second;
    }

    catenary::cantilever_pole const *pole { nullptr };
    catenary::cantilever_pole onmast;
    if( data.has_pole ) {
        pole = Catalog.pole( data.pole_type );
        if( pole == nullptr ) {
            report( "uses pole \"" + data.pole_type + "\" which isn't in the cantilever catalogue" );
            return 0;
        }
        if( pole->gantry ) {
            // wysięgnik na słupie bramki: mocowania najwyżej pod dźwigarem, na poziomie tej bramki, której słup tu stoi
            onmast = *pole;
            if( auto const *gantry { gantry_at_mast( data.pole_type, data.pole ) }; gantry != nullptr ) {
                onmast.mount_max = ( gantry->high ? pole->mast_high : pole->mast_low ) - pole->below_beam;
            }
            else {
                report( "stands on mast \"" + data.pole_type + "\", but no gantry has that mast there; the mounts go under the lower beam level" );
            }
            pole = &onmast;
        }
    }
    // Drop: obniżenie dolnego zaczepu ukośnika (dobór przy innym zestawie izolatorów, niżej)
    auto const solve = [ & ]( std::map<std::string, std::string> const &Overrides, double const Drop, catenary::cantilever_result &Result ) {
        if( pole != nullptr ) {
            // słup z katalogu: zaczepy, mocowania i słup liczy solver; Attach to oś słupa na wysokości główki szyny
            auto lowered { *scheme };
            lowered.mount_strut -= Drop;
            return catenary::solve_cantilever_on_pole( Catalog, lowered, *pole, reach, Attach.y - Wire.y, input.messenger, Overrides,
                ( Umbrella != nullptr ? *Umbrella : catenary::cantilever_umbrella() ), Result );
        }
        auto attempt { input };
        attempt.overrides = Overrides;
        attempt.strut_mount.z -= Drop;
        return catenary::solve_cantilever( Catalog, *scheme, attempt, Result );
    };
    // inny zestaw izolatorów (25 kV) bez kart katalogowych (Jan 2026-10-17: „lepić na oko”): rury ukośnika, odciągu i wysięgu
    // z izolatorem dobierane z tej samej rodziny części - długość bliska pierwotnej minus przyrost izolatora i po jednym
    // kroku w obie strony; wygrywa złożenie z najmniejszą liczbą uwag solvera, przy równej - najbliższe tej długości.
    // Rury podane w scenerii przez override nie są ruszane. Gdy mimo to zostają uwagi (zwykle uchwyt wysięgu 4050 wchodzi na
    // dłuższy izolator ukośnika - Jan 2026-10-17, 1232), dolny zaczep ukośnika schodzi co 0,1 m (najwyżej 0,5 m), aż uwag
    // nie będzie; zostaje najmniejsze obniżenie z najmniejszą liczbą uwag.
    std::vector<std::pair<std::string, std::vector<std::string>>> fits;
    if( false == data.insulators.empty() ) {
        for( auto const *tube : { "strut", "stay", "steady" } ) {
            auto const role { scheme->roles.find( tube ) }, insulator { scheme->roles.find( std::string( tube ) + "_insulator" ) };
            auto const replaced { input.overrides.find( std::string( tube ) + "_insulator" ) };
            if( role == scheme->roles.end() || insulator == scheme->roles.end() || replaced == input.overrides.end() ) { continue; }
            if( std::any_of( data.overrides.begin(), data.overrides.end(), [ & ]( std::pair<std::string, std::string> const &O ) { return O.first == tube; } ) ) { continue; }
            auto const *original { Catalog.part( role->second ) };
            auto const *oldins { Catalog.part( insulator->second ) }, *newins { Catalog.part( replaced->second ) };
            auto const dash { role->second.find( '-' ) };
            if( original == nullptr || oldins == nullptr || newins == nullptr || dash == std::string::npos ) { continue; }
            auto const family { role->second.substr( 0, dash + 1 ) };
            auto const target { original->length - ( newins->length - oldins->length ) };
            std::vector<std::pair<double, std::string>> sorted;
            for( auto const &part : Catalog.parts() ) {
                if( part.first.compare( 0, family.size(), family ) == 0 && part.second.kind == original->kind ) { sorted.emplace_back( part.second.length, part.first ); }
            }
            std::sort( sorted.begin(), sorted.end() );
            if( sorted.empty() ) { continue; }
            std::size_t nearest { 0 };
            for( std::size_t index = 1; index < sorted.size(); ++index ) {
                if( std::abs( sorted[ index ].first - target ) < std::abs( sorted[ nearest ].first - target ) ) { nearest = index; }
            }
            std::vector<std::string> candidates { sorted[ nearest ].second };   // pierwszy = najbliższy
            if( nearest > 0 ) { candidates.push_back( sorted[ nearest - 1 ].second ); }
            if( nearest + 1 < sorted.size() ) { candidates.push_back( sorted[ nearest + 1 ].second ); }
            fits.emplace_back( tube, candidates );
        }
    }
    catenary::cantilever_result result;
    auto solved { false };
    if( data.insulators.empty() ) {
        solved = solve( input.overrides, 0.0, result );
    }
    else {
        auto best { std::numeric_limits<std::size_t>::max() };
        std::size_t baseline { 0 };     // uwagi niezależne od doboru (np. mocowanie odciągu ścięte do mount_max)
        { catenary::cantilever_result plain; solve( input.overrides, 0.0, plain );
          for( auto const &warning : plain.warnings ) { if( warning.find( "highest mount position" ) != std::string::npos ) { ++baseline; } } }
        for( int step = 0; step <= 5 && best > baseline; ++step ) {
            auto const drop { 0.1 * step };
            std::vector<std::size_t> choice( fits.size(), 0 );
            while( true ) {
                auto overrides { input.overrides };
                for( std::size_t index = 0; index < fits.size(); ++index ) { overrides[ fits[ index ].first ] = fits[ index ].second[ choice[ index ] ]; }
                catenary::cantilever_result attempt;
                if( solve( overrides, drop, attempt ) && attempt.warnings.size() < best ) {
                    best = attempt.warnings.size();
                    result = attempt;
                    result.values[ "strut_mount.drop" ] = drop;
                    solved = true;
                }
                // następna kombinacja (kolejność: najbliższe długości najpierw, więc przy równej liczbie uwag zostaje pierwsza)
                std::size_t index { 0 };
                while( index < fits.size() && ++choice[ index ] == fits[ index ].second.size() ) { choice[ index ] = 0; ++index; }
                if( index == fits.size() ) { break; }
            }
        }
        if( false == solved ) { solved = solve( input.overrides, 0.0, result ); }
    }
    if( Probe != nullptr ) { *Probe = result; return ( solved ? 1 : 0 ); }
    // uwagi solvera to informacja o dopasowaniu części, nie błąd scenerii; błędem jest dopiero wysięgnik, którego nie da się złożyć
    for( auto const &warning : result.warnings ) {
        if( solved ) {
            if( Notes.insert( warning ).second ) { WriteLog( ( "Catenary: " + warning + " (first seen in suspension \"" + ( data.name.empty() ? std::string( "none" ) : data.name ) + "\")" ).c_str() ); }
        }
        else { ErrorLog( label + "can't be assembled: " + warning ); }
    }
    if( false == solved ) { return 0; }

    // każda część to zwykły egzemplarz modelu, taki sam jak z wpisu node model
    std::size_t count { 0 };
    auto const skin { Suspension.skin.empty() ? std::string( "none" ) : Suspension.skin };
    Suspension.has_saddle = false;
    for( auto const &placement : result.parts ) {
        if( false == Suspension.has_saddle && placement.role == "messenger_clamp" ) {
            Suspension.has_saddle = true;
            Suspension.saddle = placement;
            Suspension.saddle_model = ( placement.model.empty() ? Catalog.models() + placement.part : placement.model );
            Suspension.saddle_heading = heading;
            Suspension.saddle_origin = glm::dvec3 { Wire.x, Wire.y, Wire.z };
        }
        auto const part { catenary::cantilever_to_scene( placement, heading ) };
        auto entry {
            "node " + text_number( data.range_max, 1 ) + " " + text_number( data.range_min, 1 ) + " none model 0 0 0 0 "
            + ( placement.model.empty() ? Catalog.models() + placement.part : placement.model ) + ".t3d " + skin
            + " angles " + text_number( part.angles[ 0 ] ) + " " + text_number( part.angles[ 1 ] ) + " " + text_number( part.angles[ 2 ] ) };
        if( part.scale[ 0 ] != 1.0 || part.scale[ 1 ] != 1.0 || part.scale[ 2 ] != 1.0 ) {
            entry += " scale " + text_number( part.scale[ 0 ], 5 ) + " " + text_number( part.scale[ 1 ], 5 ) + " " + text_number( part.scale[ 2 ], 5 );
        }
        entry += " endmodel";
        glm::dvec3 const position { Wire.x + part.offset[ 0 ], Wire.y + part.offset[ 1 ], Wire.z + part.offset[ 2 ] };
        if( auto *model { simulation::State.create_model( entry, "none", position ) }; model != nullptr ) {
            // części należą do pliku sieci, nie do pliku scenerii: zapis scenerii w edytorze ich nie przepisuje
            catenary_part_detach( model );
            Suspension.parts.push_back( model );
            ++count;
        }
    }
    return count;
}


// punkty wpisu z punktami podanymi wprost; Attach to oś słupa (wpis ze słupem) albo pion przez zaczepy
void
entry_points( catenary::suspension const &Data, glm::dvec3 &Wire, glm::dvec3 &Rope, glm::dvec3 &Attach ) {
    Wire = { Data.at[ 0 ], Data.at[ 1 ], Data.at[ 2 ] };
    Rope = { Data.rope[ 0 ], Data.rope[ 1 ], Data.rope[ 2 ] };
    Attach = ( Data.has_pole ? glm::dvec3 { Data.pole[ 0 ], Data.pole[ 1 ], Data.pole[ 2 ] } : glm::dvec3 { Data.attach[ 0 ], Data.attach[ 1 ], Data.attach[ 2 ] } );
}

bool held_rope_at( std::size_t Index, glm::dvec3 &Rope );

// składa wysięgnik wpisu z punktami, bez parasola; zwraca liczbę wstawionych części
std::size_t
assemble_alone( std::size_t const Index, catenary::cantilever_catalog const &Catalog ) {
    auto &suspension { Suspensions[ Index ] };
    glm::dvec3 wire, rope, attach;
    entry_points( suspension.data, wire, rope, attach );
    // linę nośną łańcucha w tym miejscu trzyma inny wysięgnik: linki i zaczepy celują w nią
    auto const held { held_rope_at( Index, rope ) };
    auto const *ropepoint { held || suspension.data.has_rope ? &rope : nullptr };
    // słup stawia inny wpis (trzeci wysięgnik przy rozstawionych)
    catenary::cantilever_umbrella own;
    own.pole = false == suspension.data.shared_pole;
    if( suspension.data.has_spread ) {
        // sam jeden na konsoli: płaszczyzna wysięgnika przez punkt mocowania na konsoli, konsole stawia on
        auto const &point { suspension.data.spread };
        glm::dvec3 const plane { point[ 0 ], attach.y, point[ 2 ] };
        auto const heading { std::atan2( plane.x - wire.x, plane.z - wire.z ) };
        own.spread = true;
        own.console_x = glm::dot( attach - plane, glm::dvec3 { -std::cos( heading ), 0.0, std::sin( heading ) } );
        return assemble( suspension, Catalog, wire, ropepoint, plane, &own );
    }
    return assemble( suspension, Catalog, wire, ropepoint, attach, ( suspension.data.shared_pole ? &own : nullptr ) );
}

// parasol: oba wysięgniki składane najpierw osobno, żeby poznać wysokości mocowań, potem razem na wspólnym słupie
void
assemble_umbrella( std::string const &Name, std::vector<std::size_t> const &List, catenary::cantilever_catalog const &Catalog, std::size_t &Assembled, std::size_t &Parts, bool const Report = true ) {
    auto const alone = [ & ]( std::size_t const Index ) {
        auto const count { assemble_alone( Index, Catalog ) };
        if( count > 0 ) { ++Assembled; Parts += count; }
    };
    auto const &first { Suspensions[ List.front() ].data };
    auto const *pole { Catalog.pole( first.pole_type ) };
    auto paired { List.size() == 2 && pole != nullptr };
    if( List.size() != 2 ) {
        // w edytorze (kolejne zmiany obu wpisów pary) to stan chwilowy: komunikat tylko przy wczytaniu
        if( Report ) { ErrorLog( "Bad scenario: umbrella \"" + Name + "\" has " + std::to_string( List.size() ) + " suspension(s), it needs exactly two; they're assembled separately" ); }
    }
    else {
        auto const &second { Suspensions[ List.back() ].data };
        auto const dx { second.pole[ 0 ] - first.pole[ 0 ] }, dy { second.pole[ 1 ] - first.pole[ 1 ] }, dz { second.pole[ 2 ] - first.pole[ 2 ] };
        if( second.pole_type != first.pole_type || std::sqrt( dx * dx + dy * dy + dz * dz ) > 0.05 ) {
            // w edytorze (kolejne zmiany obu wpisów pary) to stan chwilowy: komunikat tylko przy wczytaniu
            if( Report ) { ErrorLog( "Bad scenario: umbrella \"" + Name + "\" has its suspensions on different poles; they're assembled separately" ); }
            paired = false;
        }
    }
    if( false == paired ) { for( auto const index : List ) { alone( index ); } return; }
    // wysięgniki rozstawione: oba wpisy z punktem na konsoli; jeden bez niego - błąd, składane osobno
    auto const spread { first.has_spread && Suspensions[ List.back() ].data.has_spread };
    if( spread != ( first.has_spread || Suspensions[ List.back() ].data.has_spread ) ) {
        // w edytorze (kolejne zmiany obu wpisów pary) to stan chwilowy: komunikat tylko przy wczytaniu
        if( Report ) { ErrorLog( "Bad scenario: umbrella \"" + Name + "\" has \"spread\" in one suspension only; they're assembled separately" ); }
        for( auto const index : List ) { alone( index ); }
        return;
    }
    glm::dvec3 wires[ 2 ], ropes[ 2 ], poles[ 2 ], attach[ 2 ];
    bool held[ 2 ] { false, false };    // punkt liny podany (wprost albo z wysięgnika, który ją tu trzyma)
    catenary::cantilever_result probes[ 2 ];
    catenary::cantilever_umbrella sides[ 2 ];
    auto probed { true };
    for( int side = 0; side < 2; ++side ) {
        auto &suspension { Suspensions[ List[ side ] ] };
        entry_points( suspension.data, wires[ side ], ropes[ side ], poles[ side ] );
        held[ side ] = held_rope_at( List[ side ], ropes[ side ] ) || suspension.data.has_rope;
        attach[ side ] = poles[ 0 ];
        if( spread ) {
            // płaszczyzna wysięgnika przez punkt mocowania na konsoli; oś słupa przesunięta od niej wzdłuż osi X wysięgnika
            auto const &point { suspension.data.spread };
            attach[ side ] = { point[ 0 ], poles[ 0 ].y, point[ 2 ] };
            auto const heading { std::atan2( attach[ side ].x - wires[ side ].x, attach[ side ].z - wires[ side ].z ) };
            glm::dvec3 const xaxis { -std::cos( heading ), 0.0, std::sin( heading ) };
            sides[ side ].spread = true;
            sides[ side ].console_x = glm::dot( poles[ 0 ] - attach[ side ], xaxis );
        }
        auto probe { sides[ side ] };
        probe.console_parts = false;
        probe.pole = false;
        probed = probed && assemble( suspension, Catalog, wires[ side ], ( held[ side ] ? &ropes[ side ] : nullptr ), attach[ side ], ( spread ? &probe : nullptr ), &probes[ side ] ) > 0;
    }
    if( false == probed ) { for( auto const index : List ) { alone( index ); } return; }
    if( spread ) { catenary::spread_mounts( *pole, probes[ 0 ], probes[ 1 ], sides[ 0 ], sides[ 1 ] ); }
    else { catenary::umbrella_mounts( Catalog, *pole, probes[ 0 ], probes[ 1 ], sides[ 0 ], sides[ 1 ] ); }
    sides[ 0 ].pole = false == first.shared_pole && false == Suspensions[ List.back() ].data.shared_pole;
    sides[ 1 ].pole = false;
    sides[ 1 ].console_parts = false;
    for( int side = 0; side < 2; ++side ) {
        auto &suspension { Suspensions[ List[ side ] ] };
        auto const count { assemble( suspension, Catalog, wires[ side ], ( held[ side ] ? &ropes[ side ] : nullptr ), attach[ side ], &sides[ side ] ) };
        if( count > 0 ) { ++Assembled; Parts += count; }
    }
}

// usuwa ze scenerii części wysięgnika podwieszenia
void
drop_parts( scene_suspension &Suspension ) {
    for( auto *part : Suspension.parts ) { catenary_part_delete( part ); }
    Suspension.parts.clear();
}

} // namespace

// końce spodu dźwigara bramki: każdy przy ścianie swojego słupa (oś słupa przesunięta o jego offset w stronę drugiego słupa),
// na prześwicie tego słupa (dolne albo górne mocowanie)
void
gantry_beam_ends( catenary::gantry const &Data, catenary::gantry_mast const &First, catenary::gantry_mast const *Second, double const Span, double *From, double *To ) {
    glm::dvec3 const a { Data.points[ 0 ][ 0 ], Data.points[ 0 ][ 1 ], Data.points[ 0 ][ 2 ] };
    glm::dvec3 const b { Data.points[ 1 ][ 0 ], Data.points[ 1 ][ 1 ], Data.points[ 1 ][ 2 ] };
    auto const plan { glm::dvec3 { b.x - a.x, 0.0, b.z - a.z } };
    auto const length { std::max( glm::length( plan ), 1e-6 ) };
    auto const inward { plan * ( 1.0 / length ) };
    auto const from { a + inward * First.offset + glm::dvec3 { 0.0, Data.high ? First.mount_high : First.mount_low, 0.0 } };
    // pojedynczy element (bez drugiego słupa): poziomo od ściany słupa na swoją długość
    auto const to { Second != nullptr ? b - inward * Second->offset + glm::dvec3 { 0.0, Data.high ? Second->mount_high : Second->mount_low, 0.0 } : from + inward * Span };
    From[ 0 ] = from.x; From[ 1 ] = from.y; From[ 2 ] = from.z;
    To[ 0 ] = to.x; To[ 1 ] = to.y; To[ 2 ] = to.z;
}

namespace {

// wstawia słupy i dźwigar bramki (albo usuwa, gdy bramka usunięta). zwraca liczbę wstawionych modeli
std::size_t
assemble_gantry( scene_gantry &Gantry ) {
    for( auto *part : Gantry.parts ) { catenary_part_delete( part ); }
    Gantry.parts.clear();
    if( Gantry.removed ) { return 0; }
    auto const *catalog { cantilevers() };
    if( catalog == nullptr ) { return 0; }
    auto const &data { Gantry.data };
    auto const label { "Bad scenario: gantry \"" + ( data.name.empty() ? std::string( "none" ) : data.name ) + "\" " };
    auto const *first { catalog->mast( data.masts[ 0 ] ) };
    auto const single { data.masts[ 1 ].empty() };
    auto const *second { single ? nullptr : catalog->mast( data.masts[ 1 ] ) };
    auto const *beam { catalog->beam( data.beam ) };
    if( first == nullptr || ( second == nullptr && false == single ) || beam == nullptr ) {
        ErrorLog( label + "uses a mast or a beam which isn't in the supports catalogue (traction_supports.txt)" );
        return 0;
    }
    if( single != beam->single ) {
        ErrorLog( label + ( single ? "has one mast, but beam \"" + beam->name + "\" isn't a single element" : "has two masts, but beam \"" + beam->name + "\" is a single element" ) );
        return 0;
    }
    if( single && false == first->through ) {
        ErrorLog( label + "has a single beam element on mast \"" + first->name + "\", which is an end mast; it goes on a through mast" );
    }
    glm::dvec3 const a { data.points[ 0 ][ 0 ], data.points[ 0 ][ 1 ], data.points[ 0 ][ 2 ] };
    glm::dvec3 const b { data.points[ 1 ][ 0 ], data.points[ 1 ][ 1 ], data.points[ 1 ][ 2 ] };
    auto const dx { b.x - a.x }, dz { b.z - a.z };
    auto const distance { std::sqrt( dx * dx + dz * dz ) };
    if( distance < ( single ? 0.1 : 1.0 ) ) { ErrorLog( label + ( single ? "has no direction of its single beam element" : "has its masts less than 1 m apart" ) ); return 0; }
    // między ścianami słupów jest o 0,20 m mniej niż numer dźwigara (E101: 9,9 m - użytkownik), osie słupów o offset każdego dalej
    if( false == single && std::abs( distance - catenary::gantry_spacing( *beam, *first, *second ) ) > 0.05 ) {
        WriteLog( ( "Catenary: gantry \"" + data.name + "\" has its masts " + text_number( distance, 3 ) + " m apart, beam \"" + beam->name + "\" with these masts needs "
            + text_number( catenary::gantry_spacing( *beam, *first, *second ), 3 ) + " m" ).c_str() );
    }
    // układ jak wysięgnika: Y wzdłuż dźwigara (od pierwszego słupa do drugiego), X wzdłuż toru, Z w górę
    auto const heading { std::atan2( dx, dz ) };
    std::vector<catenary::cantilever_placement> placements;
    auto const add = [ & ]( std::string const &Model, double const X, double const Y, double const Z ) {
        catenary::cantilever_placement placement;
        placement.model = Model;
        placement.position[ 0 ] = X;
        placement.position[ 1 ] = Y;
        placement.position[ 2 ] = Z;
        // słup: X modelu (szerokość) wzdłuż dźwigara, jak dźwigar
        double const turned[ 9 ] { 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
        std::copy( std::begin( turned ), std::end( turned ), placement.rotation );
        placements.push_back( placement );
    };
    // bramka wieloprzęsłowa: wspólny słup stawia tylko jedno przęsło; dźwigary z obu stron może mieć tylko słup przelotowy
    auto const index { static_cast<std::size_t>( &Gantry - Gantries.data() ) };
    auto const shared = [ & ]( int const K ) { return index < Gantries.size() && mast_shared_earlier( index, K ); };
    for( int k = 0; k < 2; ++k ) {
        auto const *type { k == 0 ? first : second };
        if( type == nullptr ) { continue; }
        if( index < Gantries.size() && false == type->through && false == gantries_at( data.points[ k ], index ).empty() ) {
            ErrorLog( label + "has beams on both sides of mast \"" + type->name + "\", which is an end mast; a through mast is needed there" );
        }
    }
    if( false == shared( 0 ) ) { add( first->model, 0.0, 0.0, 0.0 ); }
    if( second != nullptr && false == shared( 1 ) ) { add( second->model, 0.0, distance, b.y - a.y ); }
    // dźwigar: każdy koniec na mocowaniu swojego słupa (przesunięcie wzdłuż toru i prześwit), więc przy słupach różnej
    // szerokości albo prześwitu leży lekko skośnie
    double from[ 3 ], to[ 3 ];
    gantry_beam_ends( data, *first, second, beam->span, from, to );
    glm::dvec3 const ends[ 2 ] { { from[ 0 ], from[ 1 ], from[ 2 ] }, { to[ 0 ], to[ 1 ], to[ 2 ] } };
    auto const beamx { ends[ 1 ].x - ends[ 0 ].x }, beamz { ends[ 1 ].z - ends[ 0 ].z };
    auto const beamheading { std::atan2( beamx, beamz ) };
    auto const pitch { std::atan2( ends[ 1 ].y - ends[ 0 ].y, std::sqrt( beamx * beamx + beamz * beamz ) ) };
    catenary::cantilever_placement beamplacement;
    beamplacement.model = beam->model;
    {
        auto const c { std::cos( pitch ) }, s { std::sin( pitch ) };
        double const turned[ 9 ] { 0.0, -1.0, 0.0, c, 0.0, -s, s, 0.0, c };
        std::copy( std::begin( turned ), std::end( turned ), beamplacement.rotation );
    }
    auto const skin { data.skin.empty() ? std::string( "none" ) : data.skin };
    std::size_t count { 0 };
    std::vector<std::pair<catenary::cantilever_placement, double>> all;
    for( auto const &placement : placements ) { all.emplace_back( placement, heading ); }
    all.emplace_back( beamplacement, beamheading );
    for( auto const &item : all ) {
        auto const &placement { item.first };
        auto const part { catenary::cantilever_to_scene( placement, item.second ) };
        auto const entry {
            "node " + text_number( data.range_max, 1 ) + " " + text_number( data.range_min, 1 ) + " none model 0 0 0 0 " + placement.model + ".t3d " + skin
            + " angles " + text_number( part.angles[ 0 ] ) + " " + text_number( part.angles[ 1 ] ) + " " + text_number( part.angles[ 2 ] ) + " endmodel" };
        auto const &origin { &placement == &all.back().first ? ends[ 0 ] : a };
        glm::dvec3 const position { origin.x + part.offset[ 0 ], origin.y + part.offset[ 1 ], origin.z + part.offset[ 2 ] };
        if( auto *model { simulation::State.create_model( entry, "none", position ) }; model != nullptr ) {
            catenary_part_detach( model );
            Gantry.parts.push_back( model );
            ++count;
        }
    }
    return count;
}

std::size_t suspension_named( std::string const &File, std::string const &Name );
// czy łańcuch kończy się na tym kotwieniu (lina z całością albo same przewody: anchor_*_contact); Start - na początku łańcucha
inline bool
anchors_chain( catenary::chain const &Chain, std::string const &Name, bool *Start = nullptr ) {
    if( Name.empty() ) { return false; }
    auto const start { Chain.anchor_start == Name || Chain.anchor_start_contact == Name };
    if( Start != nullptr ) { *Start = start; }
    return start || Chain.anchor_end == Name || Chain.anchor_end_contact == Name;
}
void follow_anchor( scene_anchor const &Anchor );
void add_tube( std::vector<catenary::solid_vertex> &Triangles, std::vector<glm::dvec3> const &Points, double Radius );
void rope_of( catenary::suspension const &Data, double *Rope );

// urządzenia naprężające (traction_tensioners.txt); czytane przy pierwszym użyciu
catenary::tensioner_catalog const *
tensioners() {

    static catenary::tensioner_catalog catalog;
    static bool loaded { false };
    static bool good { false };
    if( false == loaded ) {
        loaded = true;
        std::vector<std::string> errors;
        auto const filename { Datadirectory + "traction_tensioners.txt" };
        good = catalog.load( filename, errors );
        if( false == good ) { ErrorLog( "Catenary: file \"" + filename + "\" is missing, anchors won't be assembled" ); }
        for( auto const &error : errors ) { ErrorLog( error ); }
    }
    return ( good ? &catalog : nullptr );
}

// składa kotwienie: słup i urządzenie naprężające przy temperaturze powietrza; zapamiętuje końce drutów.
// Układ urządzenia (X od słupa w stronę drutów, Y w lewo, Z w górę) to układ wysięgnika obrócony tak, że jego X leży
// wzdłuż kierunku drutów - części eksportowane wzdłuż toru (Jan 2026-10-23) stoją wtedy w nim wprost
std::size_t
assemble_anchor( scene_anchor &Anchor ) {
    for( auto *part : Anchor.parts ) { catenary_part_delete( part ); }
    Anchor.parts.clear();
    Anchor.solved = false;
    if( Anchor.removed ) { return 0; }
    auto const &data { Anchor.data };
    auto const label { "Bad scenario: anchor \"" + ( data.name.empty() ? std::string( "none" ) : data.name ) + "\" " };
    auto const *catalog { cantilevers() };
    auto const *devices { tensioners() };
    if( catalog == nullptr || devices == nullptr ) { return 0; }
    auto const *device { devices->device( data.device ) };
    if( device == nullptr ) { ErrorLog( label + "uses device \"" + data.device + "\" which isn't in traction_tensioners.txt" ); return 0; }
    // na istniejącym słupie: typ i punkt z podwieszenia, które go stawia
    auto poletype { data.pole_type };
    double polepoint[ 3 ] { data.pole[ 0 ], data.pole[ 1 ], data.pole[ 2 ] };
    double cantilever[ 2 ] { 0.0, 0.0 };   // na istniejącym słupie: kierunek jego wysięgnika w planie
    if( false == data.on.empty() ) {
        auto const index { suspension_named( Anchor.file, data.on ) };
        if( index >= Suspensions.size() || false == Suspensions[ index ].data.has_pole ) {
            ErrorLog( label + "is on the pole of suspension \"" + data.on + "\", which isn't in the file or has no pole" );
            return 0;
        }
        poletype = Suspensions[ index ].data.pole_type;
        std::copy( std::begin( Suspensions[ index ].data.pole ), std::end( Suspensions[ index ].data.pole ), polepoint );
        // płaszczyzna wysięgnika tego słupa (od słupa do drutu): oś słupa wzdłuż toru jest do niej prostopadła
        if( Suspensions[ index ].data.has_at ) {
            cantilever[ 0 ] = Suspensions[ index ].data.at[ 0 ] - polepoint[ 0 ];
            cantilever[ 1 ] = Suspensions[ index ].data.at[ 2 ] - polepoint[ 2 ];
        }
    }
    auto const *pole { catalog->pole( poletype ) };
    if( pole == nullptr ) { ErrorLog( label + "uses pole \"" + poletype + "\" which isn't in the supports catalogue" ); return 0; }
    catenary::tensioner_input input;
    // typ sieci: łańcucha, który się na tym kotwieniu kończy (zmiana typu w łańcuchu zmienia kotwienie), inaczej z wpisu
    auto network { data.network };
    for( auto const &chain : Chains ) {
        if( chain.removed || chain.file != Anchor.file || data.name.empty() || chain.data.type.empty() ) { continue; }
        if( anchors_chain( chain.data, data.name ) ) { network = chain.data.type; break; }
    }
    if( false == network.empty() ) {
        if( auto const *type { catenary::types().find( network ) }; type != nullptr ) {
            input.type = devices->code_for( type->type.symbol, type->type.code );
            input.contacts = type->type.contact_count;
            input.messengers = type->type.messenger_count;
            input.messenger_stack = type->type.messenger_stack;
            // sztywności i ciężar sieci do skracania przy brakujących ciężarach (przekrój i moduł z traction_tables.txt)
            auto const &net { type->type };
            input.messenger_stiffness = net.messenger_count * type->messenger_wire.area * type->messenger_wire.modulus * 0.1;
            input.contact_stiffness = net.contact_count * type->contact_wire.area * type->contact_wire.modulus * 0.1;
            input.load = ( net.messenger_count * net.messenger_mass + net.contact_count * net.contact_mass ) * input.gravity;
            // urządzenie bez danych dla tego typu (np. 0461 dla dwóch lin nośnych, 0403 dla sieci 32): najpierw z tej samej karty
            // (nazwa do "-": 0461-2x2, 0403-32), potem dowolne tego samego rodzaju (ciężarowe / stałe)
            if( device->type( input.type ) == nullptr ) {
                auto const card { data.device.substr( 0, data.device.find( '-' ) ) };
                catenary::tensioner_device const *same { nullptr }, *kind { nullptr };
                for( auto const &other : devices->devices() ) {
                    if( other.fixed != device->fixed || other.only != device->only || other.type( input.type ) == nullptr ) { continue; }
                    if( same == nullptr && other.name.substr( 0, other.name.find( '-' ) ) == card ) { same = &other; }
                    if( kind == nullptr ) { kind = &other; }
                }
                if( same != nullptr ) { device = same; }
                else if( kind != nullptr ) { device = kind; }
                // nikt nie ma danych dla tej sieci (brak wpisu "network ... as"): pierwsze dane urządzenia, żeby kotwienie stało
                if( device->type( input.type ) == nullptr && false == device->types.empty() ) {
                    WriteLog( ( label + "network type \"" + network + "\" has no data in the anchor devices; device " + device->name + " uses the data of network type " + std::to_string( device->types.front().code ) ).c_str() );
                    input.type = device->types.front().code;
                }
            }
        }
        else { ErrorLog( label + "uses network type \"" + network + "\" which has no file" ); }
    }
    input.temperature = Global.AirTemperature;
    input.distance = data.distance;
    input.elongation = data.elongation;
    input.missing = data.missing;
    input.ground = data.ground;
    input.height = data.height;
    input.rope_height = data.rope_height;
    // uchwyty do słupa: tył po drugiej stronie słupa; zero urządzenia na czole przy wysokości rolek, słup zbieżny
    input.pole = poletype;
    {
        auto const *devicetable { devices->table( device->table ) };
        auto const rollers { data.height > 0.0 ? data.height : devicetable != nullptr ? devicetable->height : 6.7 };
        // słup z szerokością wzdłuż toru (161x): urządzenie i uchwyty na ścianach wzdłuż drutów, śruby przez szerokość
        input.depth = [ pole, rollers ]( double const Height ) { return pole->width > 0.0 ? pole->width : 0.5 * pole->depth( rollers ) + 0.5 * pole->depth( Height ); };
    }
    input.color = data.color;
    input.top_color = data.top_color;
    // stare złożenie: łańcuch z tym kotwieniem ma druty zaśniedziałe (stan 1); średnia rozpiętość jego przęseł między podwieszeniami
    for( auto const &chain : Chains ) {
        if( chain.removed || chain.file != Anchor.file || data.name.empty() ) { continue; }
        if( anchors_chain( chain.data, data.name ) ) {
            input.old_fittings = ( chain.data.state == 1 );
            // łańcuch z kotwieniem środkowym: odległość od niego liczona wzdłuż łańcucha (podwieszenia aż do słupa kotwienia)
            if( false == chain.data.middle.empty() ) {
                auto const &names { chain.data.suspensions };
                auto const middle { std::find( names.begin(), names.end(), chain.data.middle ) };
                if( middle != names.end() ) {
                    bool start { false };
                    anchors_chain( chain.data, data.name, &start );
                    auto const from { static_cast<std::ptrdiff_t>( middle - names.begin() ) };
                    auto const to { start ? std::ptrdiff_t { 0 } : static_cast<std::ptrdiff_t>( names.size() ) - 1 };
                    double along { 0.0 };
                    double previous[ 2 ];
                    bool has { false }, good { true };
                    for( auto k { from }; good; k += ( start ? -1 : 1 ) ) {
                        auto const index { suspension_named( Anchor.file, names[ k ] ) };
                        if( index >= Suspensions.size() || false == Suspensions[ index ].data.has_at ) { good = false; break; }
                        auto const *at { Suspensions[ index ].data.at };
                        if( has ) { along += std::hypot( at[ 0 ] - previous[ 0 ], at[ 2 ] - previous[ 1 ] ); }
                        previous[ 0 ] = at[ 0 ]; previous[ 1 ] = at[ 2 ];
                        has = true;
                        if( k == to ) { break; }
                    }
                    if( good && has ) {
                        along += std::hypot( polepoint[ 0 ] - previous[ 0 ], polepoint[ 2 ] - previous[ 1 ] );
                        input.distance = along;
                    }
                }
            }
            double total { 0.0 };
            int spans { 0 };
            double previous[ 3 ];
            bool has { false };
            for( auto const &name : chain.data.suspensions ) {
                auto const index { suspension_named( Anchor.file, name ) };
                if( index >= Suspensions.size() || false == Suspensions[ index ].data.has_at ) { has = false; continue; }
                auto const *at { Suspensions[ index ].data.at };
                if( has ) { total += std::hypot( at[ 0 ] - previous[ 0 ], at[ 2 ] - previous[ 2 ] ); ++spans; }
                std::copy( at, at + 3, previous );
                has = true;
            }
            if( spans > 0 ) { input.span = total / spans; }
        }
    }
    // izolatory: rolkowe ESN-2025 z wyboru, a bez wyboru przy drutach zaśniedziałych (jak uchwyty 5110)
    if( data.insulators.empty() ) { input.old_messenger = input.old_contact = input.old_fittings; }
    else {
        input.old_messenger = ( data.insulators == "old" || data.insulators == "messenger" );
        input.old_contact = ( data.insulators == "old" || data.insulators == "contact" );
    }
    // kierunek drutów w planie; zero urządzenia na czole słupa
    glm::dvec3 const axis { polepoint[ 0 ], polepoint[ 1 ], polepoint[ 2 ] };
    auto dx { data.toward[ 0 ] - axis.x }, dz { data.toward[ 2 ] - axis.z };
    auto const along { std::sqrt( dx * dx + dz * dz ) };
    if( along < 1.0 ) { ErrorLog( label + "has its \"toward\" point less than 1 m from the pole" ); return 0; }
    dx /= along;
    dz /= along;
    auto const table { devices->table( device->table ) };
    // czoło: kotwienie ciągnie wzdłuż toru, więc słup kratowy z szerokością wzdłuż toru ma czoło w połowie szerokości
    auto const face { pole->width > 0.0 ? 0.5 * pole->width : 0.5 * pole->depth( data.height > 0.0 ? data.height : table != nullptr ? table->height : 6.7 ) };
    // oś słupa wzdłuż toru (w stronę kotwionego przęsła): własny słup kotwowy stoi w linii drutów; istniejący - prostopadle
    // do swojego wysięgnika. Uchwyty, prowadnica i stos są przy słupie, urządzenie od przegubu ciągnie do drutów
    auto ax { dx }, az { dz };
    if( auto const reach { std::hypot( cantilever[ 0 ], cantilever[ 1 ] ) }; reach > 0.1 ) {
        ax = -cantilever[ 1 ] / reach;
        az = cantilever[ 0 ] / reach;
        if( ax * dx + az * dz < 0.0 ) { ax = -ax; az = -az; }
    }
    glm::dvec3 const origin { axis.x + ax * face, axis.y, axis.z + az * face };
    {
        // urządzenie od czoła w stronę ostatniego podwieszenia
        auto const ex { data.toward[ 0 ] - origin.x }, ez { data.toward[ 2 ] - origin.z };
        auto const reach { std::hypot( ex, ez ) };
        if( reach > 0.5 ) { dx = ex / reach; dz = ez / reach; }
    }
    auto const heading { std::atan2( dz, -dx ) };
    auto const toscene = [ & ]( double const *Point ) {
        catenary::cantilever_placement placement;
        std::copy( Point, Point + 3, placement.position );
        auto const part { catenary::cantilever_to_scene( placement, heading ) };
        return glm::dvec3 { origin.x + part.offset[ 0 ], origin.y + part.offset[ 1 ], origin.z + part.offset[ 2 ] };
    };
    // łańcuchy do drutów wycelowane w ostatnie podwieszenie przed kotwieniem (przewody podchodzą pod kątem)
    {
        // punkt sceny -> układ urządzenia: osie urządzenia w scenie z toscene, układ prostokątny
        double const zero[ 3 ] { 0.0, 0.0, 0.0 };
        auto const base { toscene( zero ) };
        glm::dvec3 axes[ 3 ];
        for( int a = 0; a < 3; ++a ) {
            double unit[ 3 ] { 0.0, 0.0, 0.0 };
            unit[ a ] = 1.0;
            axes[ a ] = toscene( unit ) - base;
        }
        auto const todevice = [ & ]( double const *Point, double *Out ) {
            glm::dvec3 const d { Point[ 0 ] - base.x, Point[ 1 ] - base.y, Point[ 2 ] - base.z };
            for( int a = 0; a < 3; ++a ) { Out[ a ] = glm::dot( d, axes[ a ] ); }
        };
        // oś słupa w układzie urządzenia
        glm::dvec3 const structure { ax, 0.0, az };
        input.structure_angle = std::atan2( glm::dot( structure, axes[ 1 ] ), glm::dot( structure, axes[ 0 ] ) );
        for( auto const &chain : Chains ) {
            if( chain.removed || chain.file != Anchor.file || data.name.empty() || chain.data.suspensions.empty() ) { continue; }
            std::string neighbour;
            bool start { false };
            if( false == anchors_chain( chain.data, data.name, &start ) ) { continue; }
            neighbour = ( start ? chain.data.suspensions.front() : chain.data.suspensions.back() );
            auto const index { suspension_named( Anchor.file, neighbour ) };
            if( index >= Suspensions.size() || false == Suspensions[ index ].data.has_at ) { break; }
            double rope[ 3 ];
            rope_of( Suspensions[ index ].data, rope );
            todevice( Suspensions[ index ].data.at, input.contact_target );
            todevice( rope, input.messenger_target );
            input.aimed = true;
            break;
        }
    }
    auto &result { Anchor.result };
    if( false == catenary::solve_tensioner( *devices, *device, input, result ) ) {
        for( auto const &warning : result.warnings ) { ErrorLog( label + "can't be assembled: " + warning ); }
        return 0;
    }
    for( auto const &warning : result.warnings ) {
        if( Notes.insert( warning ).second ) { WriteLog( ( "Catenary: " + warning + " (first seen in anchor \"" + data.name + "\")" ).c_str() ); }
    }
    // lina: ucha jako części, reszta jedną bryłą proceduralną (bez łączeń na łukach rolek), przeliczoną na scenerię;
    // rysuje ją z bliska przęsło dochodzące do kotwienia (rebuild_span)
    {
        std::vector<catenary::tensioner_vertex> tube;
        catenary::tensioner_rope_solid( result, 0.5 * device->rope_diameter, 8, device->rope_eye, device->rope_eye_length, tube, device->rope_spacer, device->rope_spacer_length );
        // kotwienie rozdzielone: druga lina (urządzenie przewodów), z uchami
        if( false == result.rope2.empty() ) {
            catenary::tensioner_result second;
            second.rope = result.rope2;
            catenary::tensioner_rope_solid( second, 0.5 * device->rope_diameter, 8, device->rope_eye, device->rope_eye_length, tube, device->rope_spacer, device->rope_spacer_length );
            for( auto part : second.parts ) { part.role = "contact_device." + part.role; result.parts.push_back( part ); }
        }
        double const zero[ 3 ] { 0.0, 0.0, 0.0 };
        auto const base { toscene( zero ) };
        glm::dvec3 axes[ 3 ];
        for( int a = 0; a < 3; ++a ) {
            double unit[ 3 ] { 0.0, 0.0, 0.0 };
            unit[ a ] = 1.0;
            axes[ a ] = toscene( unit ) - base;
        }
        // przeliczenie na scenerię może odbijać (zamiana osi): wtedy odwrócić kolejność wierzchołków trójkątów
        auto const &ax { axes[ 0 ] }, &ay { axes[ 1 ] }, &az { axes[ 2 ] };
        auto const mirrored { ax.x * ( ay.y * az.z - ay.z * az.y ) + ax.y * ( ay.z * az.x - ay.x * az.z ) + ax.z * ( ay.x * az.y - ay.y * az.x ) < 0.0 };
        Anchor.rope.clear();
        Anchor.rope.reserve( tube.size() );
        for( std::size_t k = 0; k < tube.size(); ++k ) {
            auto const &item { tube[ mirrored ? ( k - k % 3 ) + ( 2 - k % 3 ) : k ] };
            auto const at { toscene( item.position ) };
            catenary::solid_vertex vertex;
            vertex.position = catenary::vec3 { at.x, at.y, at.z };
            vertex.normal = catenary::vec3 {
                ax.x * item.normal[ 0 ] + ay.x * item.normal[ 1 ] + az.x * item.normal[ 2 ],
                ax.y * item.normal[ 0 ] + ay.y * item.normal[ 1 ] + az.y * item.normal[ 2 ],
                ax.z * item.normal[ 0 ] + ay.z * item.normal[ 1 ] + az.z * item.normal[ 2 ] };
            Anchor.rope.push_back( vertex );
        }
    }
    Anchor.temperature = Global.AirTemperature;

    {
        auto const messenger { toscene( result.messenger_end ) };
        double contact[ 3 ] { 0.0, 0.0, 0.0 };
        for( auto const &end : result.contact_ends ) {
            double point[ 3 ] { end[ 0 ], end[ 1 ], end[ 2 ] };
            auto const at { toscene( point ) };
            contact[ 0 ] += at.x;
            contact[ 1 ] += at.y;
            contact[ 2 ] += at.z;
        }
        auto const ends { std::max<std::size_t>( 1, result.contact_ends.size() ) };
        Anchor.messenger[ 0 ] = messenger.x;
        Anchor.messenger[ 1 ] = messenger.y;
        Anchor.messenger[ 2 ] = messenger.z;
        // druga lina nośna: koniec na jej uchwycie krańcowym, przęsło do kotwienia dociąga tam dolną linę (dopisek "rope2")
        auto const second { result.messenger2_end[ 0 ] != 0.0 || result.messenger2_end[ 1 ] != 0.0 || result.messenger2_end[ 2 ] != 0.0 };
        Anchor.has_messenger2 = second && input.messengers >= 2;
        if( Anchor.has_messenger2 ) {
            auto const at { toscene( result.messenger2_end ) };
            Anchor.messenger2[ 0 ] = at.x;
            Anchor.messenger2[ 1 ] = at.y;
            Anchor.messenger2[ 2 ] = at.z;
        }
        for( int k = 0; k < 3; ++k ) { Anchor.contact[ k ] = contact[ k ] / ends; }
    }
    {
        // obrys dla edytora: czoło słupa i rolka ruchoma na wysokości rolek, kolumna ciężarów od wolnego końca liny do spodu
        auto const column { result.rope.empty() ? std::vector<double> { 0.0, 0.0, result.height } : result.rope.back() };
        double const points[ 4 ][ 3 ] {
            { 0.0, 0.0, result.height },
            { result.b, 0.0, result.height },
            { column[ 0 ], column[ 1 ], column[ 2 ] },
            { column[ 0 ], column[ 1 ], result.s } };
        for( int k = 0; k < 4; ++k ) {
            auto const at { toscene( points[ k ] ) };
            Anchor.shape[ k ][ 0 ] = at.x;
            Anchor.shape[ k ][ 1 ] = at.y;
            Anchor.shape[ k ][ 2 ] = at.z;
        }
    }
    Anchor.solved = true;

    auto const skin { data.skin.empty() ? std::string( "none" ) : data.skin };
    std::size_t count { 0 };
    auto const place = [ & ]( std::string const &Model, catenary::cantilever_placement const &Placement, double const Heading, glm::dvec3 const &Origin ) {
        auto const part { catenary::cantilever_to_scene( Placement, Heading ) };
        auto entry {
            "node " + text_number( data.range_max, 1 ) + " " + text_number( data.range_min, 1 ) + " none model 0 0 0 0 " + Model + ".t3d " + skin
            + " angles " + text_number( part.angles[ 0 ] ) + " " + text_number( part.angles[ 1 ] ) + " " + text_number( part.angles[ 2 ] ) };
        if( part.scale[ 0 ] != 1.0 || part.scale[ 1 ] != 1.0 || part.scale[ 2 ] != 1.0 ) {
            entry += " scale " + text_number( part.scale[ 0 ], 5 ) + " " + text_number( part.scale[ 1 ], 5 ) + " " + text_number( part.scale[ 2 ], 5 );
        }
        entry += " endmodel";
        glm::dvec3 const position { Origin.x + part.offset[ 0 ], Origin.y + part.offset[ 1 ], Origin.z + part.offset[ 2 ] };
        if( auto *model { simulation::State.create_model( entry, "none", position ) }; model != nullptr ) {
            catenary_part_detach( model );
            Anchor.parts.push_back( model );
            ++count;
        }
    };
    // słup: Y wysięgnika od toru do słupa (słup po tej stronie drutów, po której stoi względem punktu "toward");
    // istniejący słup stawia jego podwieszenie
    if( data.on.empty() ) {
        double sidex { dz }, sidez { -dx };
        if( ( axis.x - data.toward[ 0 ] ) * sidex + ( axis.z - data.toward[ 2 ] ) * sidez < 0.0 ) { sidex = -sidex; sidez = -sidez; }
        auto const poleheading { std::atan2( sidex, sidez ) };
        catenary::cantilever_placement placement;
        if( pole->turn != 0.0 ) {
            auto const angle { pole->turn * 3.14159265358979323846 / 180.0 };
            auto const c { std::cos( angle ) }, s { std::sin( angle ) };
            double const turned[ 9 ] { c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0 };
            std::copy( std::begin( turned ), std::end( turned ), placement.rotation );
        }
        place( pole->model, placement, poleheading, axis );
        if( false == pole->foundation.empty() ) { place( pole->foundation, placement, poleheading, axis ); }
    }
    for( auto const &item : result.parts ) {
        catenary::cantilever_placement placement;
        std::copy( std::begin( item.position ), std::end( item.position ), placement.position );
        std::copy( std::begin( item.rotation ), std::end( item.rotation ), placement.rotation );
        std::copy( std::begin( item.scale ), std::end( item.scale ), placement.scale );
        auto const *part { catalog->part( item.part ) };
        auto const model { catalog->models() + ( part != nullptr && false == part->model.empty() ? part->model : item.part ) };
        place( model, placement, heading, origin );
    }
    return count;
}

// kotwienie o podanej nazwie z podanego pliku; nullptr gdy nie ma
scene_anchor *
anchor_named( std::string const &File, std::string const &Name ) {
    if( Name.empty() ) { return nullptr; }
    for( auto &anchor : Anchors ) {
        if( false == anchor.removed && anchor.file == File && anchor.data.name == Name ) { return &anchor; }
    }
    return nullptr;
}

// numer podwieszenia o podanej nazwie z podanego pliku; Suspensions.size() gdy nie ma
std::size_t
suspension_named( std::string const &File, std::string const &Name ) {
    for( std::size_t index = 0; index < Suspensions.size(); ++index ) {
        auto const &suspension { Suspensions[ index ] };
        if( false == suspension.removed && suspension.file == File && suspension.data.name == Name ) { return index; }
    }
    return Suspensions.size();
}

std::string
point_text( double const *Point ) {
    return text_number( Point[ 0 ], 3 ) + " " + text_number( Point[ 1 ], 3 ) + " " + text_number( Point[ 2 ], 3 );
}

// lina nośna podwieszenia: podana wprost albo nad przewodem na wysokości konstrukcyjnej karty
void
rope_of( catenary::suspension const &Data, double *Rope ) {
    if( Data.has_rope ) { for( int i = 0; i < 3; ++i ) { Rope[ i ] = Data.rope[ i ]; } return; }
    auto const *catalog { cantilevers() };
    auto const *scheme { catalog != nullptr ? catalog->scheme( Data.card, Data.execution ) : nullptr };
    Rope[ 0 ] = Data.at[ 0 ];
    Rope[ 1 ] = Data.at[ 1 ] + ( scheme != nullptr ? scheme->height : 1.70 );
    Rope[ 2 ] = Data.at[ 2 ];
}

// punkty końca przęsła: przewód jezdny i lina nośna. Miejsce k w łańcuchu z kotwieniami: najpierw kotwienie początkowe (gdy
// jest), potem podwieszenia, na końcu kotwienie końcowe. false, gdy brakuje wpisu albo kotwienie się nie złożyło
std::size_t
chain_points( scene_chain const &Chain ) {
    return Chain.data.suspensions.size() + ( Chain.data.anchor_start.empty() ? 0 : 1 ) + ( Chain.data.anchor_end.empty() ? 0 : 1 );
}

bool held_rope( scene_chain const &Chain, std::size_t Position, double *Rope );

bool
chain_point( scene_chain const &Chain, std::size_t Point, double *Wire, double *Rope, bool &Anchored ) {
    auto const &data { Chain.data };
    Anchored = false;
    auto const *anchor { static_cast<scene_anchor const *>( nullptr ) };
    if( false == data.anchor_start.empty() ) {
        if( Point == 0 ) { anchor = anchor_named( Chain.file, data.anchor_start ); Anchored = true; }
        else { --Point; }
    }
    if( false == Anchored && Point >= data.suspensions.size() ) {
        if( Point == data.suspensions.size() && false == data.anchor_end.empty() ) { anchor = anchor_named( Chain.file, data.anchor_end ); Anchored = true; }
        else { return false; }
    }
    if( Anchored ) {
        if( anchor == nullptr || false == anchor->solved ) { return false; }
        // przewody zakotwione osobno: ich koniec z drugiego kotwienia
        auto const &contactname { Point == 0 && false == data.anchor_start.empty() ? data.anchor_start_contact : data.anchor_end_contact };
        auto const *contact { contactname.empty() ? anchor : anchor_named( Chain.file, contactname ) };
        if( contact == nullptr || false == contact->solved ) { return false; }
        std::copy( std::begin( contact->contact ), std::end( contact->contact ), Wire );
        std::copy( std::begin( anchor->messenger ), std::end( anchor->messenger ), Rope );
        return true;
    }
    auto const index { suspension_named( Chain.file, data.suspensions[ Point ] ) };
    if( index >= Suspensions.size() || false == Suspensions[ index ].data.has_at ) { return false; }
    std::copy( std::begin( Suspensions[ index ].data.at ), std::end( Suspensions[ index ].data.at ), Wire );
    // lina nośna z innego wysięgnika (rozstawione, sama lina): gdy go nie ma, z tego samego
    if( false == held_rope( Chain, Point, Rope ) ) { rope_of( Suspensions[ index ].data, Rope ); }
    return true;
}

// lina nośna łańcucha w miejscu Position listy, gdy trzyma ją inny wysięgnik (messenger_from): punkt liny trzymającego,
// przesunięty wzdłuż łańcucha do miejsca tego podwieszenia, na wysokości liny trzymającego (rozstawione wysięgi pomocnicze
// 0167 i wysięgnik samej liny w osi słupa między nimi: lina idzie prosto przez siodło, linki obu wysięgów trafiają w nią).
// false, gdy linę trzyma to samo podwieszenie
bool
held_rope( scene_chain const &Chain, std::size_t const Position, double *Rope ) {
    auto const &names { Chain.data.suspensions };
    if( Position >= names.size() ) { return false; }
    auto const &holder { catenary::messenger_holder( Chain.data, names[ Position ] ) };
    if( holder == names[ Position ] ) { return false; }
    auto const own { suspension_named( Chain.file, names[ Position ] ) }, other { suspension_named( Chain.file, holder ) };
    if( own >= Suspensions.size() || other >= Suspensions.size() || false == Suspensions[ other ].data.has_at ) { return false; }
    auto const &listed { Suspensions[ own ].data }, &holding { Suspensions[ other ].data };
    rope_of( holding, Rope );
    // przesunięcie tylko, gdy ten sam wysięgnik trzyma linę także w sąsiednim punkcie listy (para wysięgów pomocniczych z
    // wysięgnikiem samej liny między nimi); sam jeden (rozstawione na konsoli) - lina łamie się w punkcie trzymającego
    auto const sameholder = [ & ]( std::size_t const K ) { return K < names.size() && K != Position && catenary::messenger_holder( Chain.data, names[ K ] ) == holder; };
    if( false == sameholder( Position + 1 ) && ( Position == 0 || false == sameholder( Position - 1 ) ) ) { return true; }
    // kierunek łańcucha w tym miejscu: od poprzedniego do następnego podwieszenia listy
    auto const at = [ & ]( std::size_t const K ) {
        auto const index { suspension_named( Chain.file, names[ K ] ) };
        auto const &data { index < Suspensions.size() && Suspensions[ index ].data.has_at ? Suspensions[ index ].data : listed };
        return glm::dvec2 { data.at[ 0 ], data.at[ 2 ] };
    };
    auto const way { at( Position + 1 < names.size() ? Position + 1 : Position ) - at( Position > 0 ? Position - 1 : Position ) };
    if( glm::length( way ) < 0.1 ) { return true; }
    auto const along { glm::normalize( way ) };
    auto const shift { along * glm::dot( glm::dvec2 { listed.at[ 0 ] - holding.at[ 0 ], listed.at[ 2 ] - holding.at[ 2 ] }, along ) };
    Rope[ 0 ] += shift.x;
    Rope[ 2 ] += shift.y;
    return true;
}

// to samo dla podwieszenia Index (pierwszy łańcuch jego pliku, w którym linę w jego miejscu trzyma inny wysięgnik)
bool
held_rope_at( std::size_t const Index, glm::dvec3 &Rope ) {
    if( Index >= Suspensions.size() || Suspensions[ Index ].data.name.empty() ) { return false; }
    auto const &name { Suspensions[ Index ].data.name };
    for( auto const &chain : Chains ) {
        if( chain.removed || chain.file != Suspensions[ Index ].file ) { continue; }
        auto const &names { chain.data.suspensions };
        auto const found { std::find( names.begin(), names.end(), name ) };
        if( found == names.end() ) { continue; }
        double point[ 3 ];
        if( held_rope( chain, static_cast<std::size_t>( found - names.begin() ), point ) ) {
            Rope = glm::dvec3 { point[ 0 ], point[ 1 ], point[ 2 ] };
            return true;
        }
    }
    return false;
}

// naciąg przęsła k łańcucha od kotwienia z niepełnym stosem ciężarów (daN): przęsła od tego kotwienia do kotwienia
// środkowego albo stałego (distance wzdłuż łańcucha); false, gdy przęsło ma naciąg typu sieci. Bliższe kotwienie wygrywa,
// ciężary na gruncie zostawiają naciąg typu (sieć bez naciągu urządzenia nie da się policzyć)
bool
span_tension( scene_chain const &Chain, std::size_t const Span, double &Messenger, double &Contact ) {
    auto const *type { Chain.data.type.empty() ? nullptr : catenary::types().find( Chain.data.type ) };
    if( type == nullptr ) { return false; }
    auto const points { chain_points( Chain ) };
    if( Span + 1 >= points ) { return false; }
    // odległości wzdłuż łańcucha od początku do środka przęsła i od środka przęsła do końca
    std::vector<double> along( points, 0.0 );
    double previous[ 3 ], wire[ 3 ], rope[ 3 ];
    bool anchored;
    for( std::size_t point = 0; point < points; ++point ) {
        if( false == chain_point( Chain, point, wire, rope, anchored ) ) { return false; }
        if( point > 0 ) { along[ point ] = along[ point - 1 ] + std::hypot( wire[ 0 ] - previous[ 0 ], wire[ 2 ] - previous[ 2 ] ); }
        std::copy( wire, wire + 3, previous );
    }
    auto const middle { 0.5 * ( along[ Span ] + along[ Span + 1 ] ) };
    double best { 1e30 }, ratio { 1.0 };
    for( auto const end : { 0, 1 } ) {
        auto const &name { end == 0 ? Chain.data.anchor_start : Chain.data.anchor_end };
        if( name.empty() ) { continue; }
        auto const *anchor { anchor_named( Chain.file, name ) };
        if( anchor == nullptr || false == anchor->solved || anchor->result.grounded || anchor->result.nominal_tension <= 0.0 ) { continue; }
        auto const reach { end == 0 ? middle : along.back() - middle };
        if( reach > anchor->data.distance || reach >= best ) { continue; }
        best = reach;
        ratio = anchor->result.tension / anchor->result.nominal_tension;
    }
    if( std::abs( ratio - 1.0 ) < 1e-3 || ratio <= 0.0 ) { return false; }
    Messenger = 0.1 * type->type.messenger_tension * ratio;
    Contact = 0.1 * type->type.contact_tension * ratio;
    return true;
}

// treść wpisu node traction dla przęsła k łańcucha; pusta, gdy brakuje podwieszenia. Przęsło do kotwienia bez wieszaków
std::string
span_body( scene_chain const &Chain, std::size_t const Span ) {
    auto const &data { Chain.data };
    if( Span + 1 >= chain_points( Chain ) ) { return {}; }
    double wirea[ 3 ], wireb[ 3 ], ropea[ 3 ], ropeb[ 3 ];
    bool anchoreda, anchoredb;
    if( false == chain_point( Chain, Span, wirea, ropea, anchoreda ) || false == chain_point( Chain, Span + 1, wireb, ropeb, anchoredb ) ) { return {}; }
    std::string body { data.power + " " + text_number( data.voltage, 1 ) + " " + text_number( data.current, 1 ) + " " + text_number( data.resistivity, 4 ) + " "
        + data.material + " " + text_number( data.thickness, 2 ) + " " + std::to_string( data.state ) + "\n" };
    body += point_text( wirea ) + "\n" + point_text( wireb ) + "\n" + point_text( ropea ) + "\n" + point_text( ropeb ) + "\n";
    auto const spacing { anchoreda || anchoredb ? 0.0 : data.hangers[ 1 ] };
    body += text_number( data.hangers[ 0 ], 3 ) + " " + text_number( spacing, 3 ) + " " + std::to_string( static_cast<int>( data.hangers[ 2 ] ) ) + " " + text_number( data.hangers[ 3 ], 3 ) + " vis";
    if( false == data.type.empty() ) {
        body += " type " + data.type;
        // brakujące ciężary w kotwieniu: mniejszy naciąg, większy zwis (starsze wersje symulatora pomijają ten dopisek)
        double messenger, contact;
        if( span_tension( Chain, Span, messenger, contact ) ) { body += " tension " + text_number( messenger, 0 ) + " " + text_number( contact, 0 ); }
    }
    if( false == data.parallel.empty() ) { body += " parallel " + data.parallel; }
    // kotwienie z dwiema liniami: dolna lina przęsła kończy się na swoim uchwycie krańcowym
    for( auto const end : { 0, 1 } ) {
        if( ( end == 0 ? anchoreda : anchoredb ) == false ) { continue; }
        auto const *anchor { anchor_named( Chain.file, end == 0 ? ( Span == 0 ? data.anchor_start : std::string() ) : data.anchor_end ) };
        if( anchor == nullptr || false == anchor->has_messenger2 ) { continue; }
        body += std::string( " rope2 " ) + ( end == 0 ? "start " : "end " ) + point_text( anchor->messenger2 );
    }
    return body + " endtraction";
}

std::string
span_name( scene_chain const &Chain, std::size_t const Span ) {
    return ( Chain.data.name.empty() ? std::string( "none" ) : Chain.data.name + "_" + std::to_string( Span + 1 ) );
}

// prostopadłościan (bryła zastępcza osprzętu bez modelu): środek, oś wzdłuż Along, długość i przekrój
void
add_box( std::vector<catenary::solid_vertex> &Triangles, glm::dvec3 const &Center, glm::dvec3 Along, double const Length, double const Size ) {
    Along = glm::length( Along ) > 1e-9 ? glm::normalize( Along ) : glm::dvec3 { 1.0, 0.0, 0.0 };
    auto side { glm::cross( Along, glm::dvec3 { 0.0, 1.0, 0.0 } ) };
    if( glm::length( side ) < 1e-6 ) { side = glm::dvec3 { 1.0, 0.0, 0.0 }; }
    side = glm::normalize( side );
    auto const up { glm::normalize( glm::cross( side, Along ) ) };
    glm::dvec3 const axes[ 3 ] { Along * ( 0.5 * Length ), side * ( 0.5 * Size ), up * ( 0.5 * Size ) };
    // sześć ścian: dla każdej osi strona + i -; wierzchołki przeciwnie do ruchu wskazówek patrząc z zewnątrz
    for( int a = 0; a < 3; ++a ) {
        for( auto const sign : { 1.0, -1.0 } ) {
            auto const normal { axes[ a ] * sign };
            auto const &u { axes[ ( a + 1 ) % 3 ] }, &v { axes[ ( a + 2 ) % 3 ] };
            auto const c { Center + normal };
            glm::dvec3 const corner[ 4 ] { c - u - v, c + u - v, c + u + v, c - u + v };
            int const order[ 2 ][ 6 ] { { 0, 1, 2, 0, 2, 3 }, { 0, 2, 1, 0, 3, 2 } };
            auto const n { glm::normalize( normal ) };
            for( auto const k : order[ sign > 0.0 ? 0 : 1 ] ) {
                catenary::solid_vertex vertex;
                vertex.position = catenary::vec3 { corner[ k ].x, corner[ k ].y, corner[ k ].z };
                vertex.normal = catenary::vec3 { n.x, n.y, n.z };
                Triangles.push_back( vertex );
            }
        }
    }
}

// rura po łamanej (lina kotwienia środkowego, ramiona V), współrzędne scenerii
void
add_tube( std::vector<catenary::solid_vertex> &Triangles, std::vector<glm::dvec3> const &Points, double const Radius ) {
    catenary::tensioner_result path;
    for( auto const &point : Points ) { path.rope.push_back( { point.x, point.y, point.z } ); }
    std::vector<catenary::tensioner_vertex> tube;
    catenary::tensioner_rope_solid( path, Radius, 6, std::string(), 0.0, tube );
    for( auto const &item : tube ) {
        catenary::solid_vertex vertex;
        vertex.position = catenary::vec3 { item.position[ 0 ], item.position[ 1 ], item.position[ 2 ] };
        vertex.normal = catenary::vec3 { item.normal[ 0 ], item.normal[ 1 ], item.normal[ 2 ] };
        Triangles.push_back( vertex );
    }
}

// czy przęsło k łańcucha ma linkę Y przy końcu End (0: punkt k, 1: punkt k+1) - tak samo jak y_at w catenary_span.cpp:
// oba przęsła przy podwieszeniu mają wieszaki i tę samą długość Y z tablicy typu
bool
span_y_at( scene_chain const &Chain, catenary::network_definition const &Type, std::size_t const Span, int const End ) {
    if( Type.type.y_tension <= 0.0 ) { return false; }
    auto const ylength = [ & ]( std::size_t const K, double &Out ) {
        if( K + 1 >= chain_points( Chain ) ) { return false; }
        double wirea[ 3 ], ropea[ 3 ], wireb[ 3 ], ropeb[ 3 ];
        bool anchoreda, anchoredb;
        if( false == chain_point( Chain, K, wirea, ropea, anchoreda ) || false == chain_point( Chain, K + 1, wireb, ropeb, anchoredb ) ) { return false; }
        if( anchoreda || anchoredb || Chain.data.hangers[ 1 ] <= 0.0 ) { return false; }  // przęsło bez wieszaków
        auto const length { std::hypot( wireb[ 0 ] - wirea[ 0 ], wireb[ 2 ] - wirea[ 2 ] ) };
        auto const y { Type.type.y_length( length ) };
        Out = ( y < length ? y : 0.0 );
        return Out > 0.0;
    };
    double mine, other;
    if( false == ylength( Span, mine ) ) { return false; }
    if( End == 0 && Span == 0 ) { return false; }
    if( false == ylength( End == 0 ? Span - 1 : Span + 1, other ) ) { return false; }
    return std::abs( mine - other ) <= 0.01;
}

// kotwienie środkowe łańcucha (karty 63-0406 / 0408): przy podwieszeniu "middle" lina kotwienia zaciśnięta na linie nośnej 1,00 m
// z każdej strony, dalej do słupów sąsiednich podwieszeń (tam 4430 w uchwytach i łańcuch z izolatorem - urządzenie <karta>
// i <karta>-2 z regulacją), w połowie obu przęseł V 1,50 + 1,50 m do przewodu jezdnego. Lina, V i zaciski jako bryły (bez modeli)
void
assemble_middle( scene_chain &Chain ) {
    for( auto *part : Chain.middle_parts ) { catenary_part_delete( part ); }
    Chain.middle_parts.clear();
    Chain.middle_solid[ 0 ].clear();
    Chain.middle_solid[ 1 ].clear();
    Chain.middle_span = static_cast<std::size_t>( -1 );
    auto const &data { Chain.data };
    if( Chain.removed || data.middle.empty() ) { return; }
    auto const label { "Bad scenario: chain \"" + ( data.name.empty() ? std::string( "none" ) : data.name ) + "\" " };
    auto const found { std::find( data.suspensions.begin(), data.suspensions.end(), data.middle ) };
    if( found == data.suspensions.end() ) { ErrorLog( label + "has its middle anchor at \"" + data.middle + "\", which isn't in the chain" ); return; }
    auto const position { static_cast<std::size_t>( found - data.suspensions.begin() ) };
    if( position == 0 || position + 1 >= data.suspensions.size() ) { ErrorLog( label + "has its middle anchor at an end of the chain; it needs a pole on each side" ); return; }
    auto const offset { data.anchor_start.empty() ? std::size_t { 0 } : std::size_t { 1 } };
    double wires[ 3 ][ 3 ], ropes[ 3 ][ 3 ];
    bool anchored;
    for( int k = 0; k < 3; ++k ) {
        if( false == chain_point( Chain, position + offset + k - 1, wires[ k ], ropes[ k ], anchored ) ) { return; }
    }
    auto const *catalog { cantilevers() };
    auto const *devices { tensioners() };
    auto const *type { data.type.empty() ? nullptr : catenary::types().find( data.type ) };
    if( catalog == nullptr || devices == nullptr ) { return; }
    Chain.middle_span = position + offset - 1;
    auto const vec = []( double const *P ) { return glm::dvec3 { P[ 0 ], P[ 1 ], P[ 2 ] }; };
    auto const middlerope { vec( ropes[ 1 ] ) };
    double const ropeheight { 0.0045 };  // promień liny 9831-5 - ZAŁOŻENIE (przekroju brak)
    // 63-0416 do dźwigara bramki: podwieszenie środkowe na zwisie, liny od dźwigara (4590 na jego spodzie, nad liną nośną) do
    // zacisków na linie nośnej ok. 3,00 m z obu stron; bez urządzeń na słupach sąsiednich
    auto tobeam { data.middle_device == "0416" };
    glm::dvec3 beam { middlerope };
    if( tobeam ) {
        auto const index { suspension_named( Chain.file, data.middle ) };
        auto const *own { index < Suspensions.size() && Suspensions[ index ].data.has_pole ? catalog->pole( Suspensions[ index ].data.pole_type ) : nullptr };
        if( own == nullptr || own->drop.empty() ) {
            ErrorLog( label + "has a middle anchor to a gantry beam (0416) at \"" + data.middle + "\", which doesn't hang on a drop of a gantry; anchored to the poles next to it instead (0406)" );
            tobeam = false;
        }
        else { beam.y = Suspensions[ index ].data.pole[ 1 ]; }  // spód dźwigara (zero zwisu)
    }
    // wysięgnik teowy na słupie środkowym: lina kotwienia idzie przez obejmę na wysięgniku (4160, stare karty 33-0418, kolprojekt1
    // s. 156 poz. 1), bez zacisków 1,00 m od słupa (Jan 2026-10-23: zaciski rozstawione o 1 m tylko przy rurowych). ZAŁOŻENIE:
    // obejmą jest 4160-1 z karty, lina przechodzi przez nią na wysokości liny nośnej
    auto tee { false };
    glm::dvec3 collar { middlerope };   // przez nią przechodzi lina kotwienia przy teowniku
    if( false == tobeam ) {
        auto const index { suspension_named( Chain.file, data.middle ) };
        if( index < Suspensions.size() ) {
            auto const &own { Suspensions[ index ] };
            auto const *scheme { catalog->scheme( own.data.card, own.data.execution ) };
            tee = scheme != nullptr && scheme->assembly.rfind( "tee_", 0 ) == 0;
            // obejma 4160 na teowniku obok siodła liny nośnej, w stronę słupa (Jan 2026-10-26: lina kotwienia ma swoje mocowanie
            // obok, nie w siodle liny); odstęp 0,15 m - ZAŁOŻENIE na oko
            if( tee && own.has_saddle ) {
                double const beside { 0.15 };
                auto placement { own.saddle };
                placement.position[ 1 ] += beside;
                auto const scened { catenary::cantilever_to_scene( placement, own.saddle_heading ) };
                auto const entry {
                    "node " + text_number( data.range_max > 0.0 ? data.range_max : 1000.0, 1 ) + " " + text_number( data.range_min, 1 ) + " none model 0 0 0 0 " + own.saddle_model + ".t3d none"
                    + " angles " + text_number( scened.angles[ 0 ] ) + " " + text_number( scened.angles[ 1 ] ) + " " + text_number( scened.angles[ 2 ] ) + " endmodel" };
                glm::dvec3 const at { own.saddle_origin.x + scened.offset[ 0 ], own.saddle_origin.y + scened.offset[ 1 ], own.saddle_origin.z + scened.offset[ 2 ] };
                if( auto *model3d { simulation::State.create_model( entry, "none", at ) }; model3d != nullptr ) {
                    catenary_part_detach( model3d );
                    Chain.middle_parts.push_back( model3d );
                }
                // przesunięcie o tyle samo w stronę konstrukcji (oś Y wysięgnika w scenerii)
                double const unit[ 3 ] { 0.0, 1.0, 0.0 };
                catenary::cantilever_placement axis;
                std::copy( unit, unit + 3, axis.position );
                auto const toward { catenary::cantilever_to_scene( axis, own.saddle_heading ) };
                collar = middlerope + glm::dvec3 { toward.offset[ 0 ], toward.offset[ 1 ], toward.offset[ 2 ] } * beside;
            }
        }
    }
    for( int side = 0; side < 2; ++side ) {
        auto const neighbour { side == 0 ? 0 : 2 };
        auto &solid { Chain.middle_solid[ side ] };
        // zacisk na linie nośnej 1,00 m od środkowego podwieszenia w stronę sąsiedniego (do dźwigara: ok. 3,00 m; teownik: w obejmie)
        auto const towards { vec( ropes[ neighbour ] ) - middlerope };
        auto const clampdistance { tobeam ? 3.0 : tee ? 0.0 : 1.0 };
        auto const clampchord { middlerope + towards * ( clampdistance / std::max( std::max( clampdistance, 1e-3 ), glm::length( glm::dvec2 { towards.x, towards.z } ) ) ) };
        // połowa przęsła: przewód jezdny (prosto między końcami), lina nośna nad nim na wysokości najkrótszego wieszaka
        auto const wiremiddle { ( vec( wires[ 1 ] ) + vec( wires[ neighbour ] ) ) * 0.5 };
        auto const alongspan { glm::normalize( glm::dvec3 { wires[ neighbour ][ 0 ] - wires[ 1 ][ 0 ], 0.0, wires[ neighbour ][ 2 ] - wires[ 1 ][ 2 ] } ) };
        // lina nośna w przęśle tak, jak ją rysuje przęsło: solver sieci z odstępami wieszaków z tablicy typu (t - ułamek
        // przęsła od podwieszenia środkowego); bez typu - prosto między podwieszeniami
        auto const spanlength { glm::length( glm::dvec2 { wires[ neighbour ][ 0 ] - wires[ 1 ][ 0 ], wires[ neighbour ][ 2 ] - wires[ 1 ][ 2 ] } ) };
        catenary::span_solution solution;
        auto solved { false };
        if( type != nullptr ) {
            // przęsło liczone dokładnie jak rysowane: w jego kierunku (od punktu k do k+1), z tym samym naciągiem (dopisek
            // tension) i z linkami Y tylko tam, gdzie przęsło je dostaje (sąsiednie przęsło z wieszakami i tą samą długością Y)
            auto const span { position + offset - 1 + side };
            auto definition { *type };
            double messengertension, contacttension;
            if( span_tension( Chain, span, messengertension, contacttension ) ) {
                if( messengertension > 0.0 ) { definition.type.messenger_tension = 10.0 * messengertension; }
                if( contacttension > 0.0 && definition.type.contact_tension > 0.0 ) { definition.type.contact_tension = 10.0 * contacttension; }
            }
            auto const first { side == 0 ? neighbour : 1 }, second { side == 0 ? 1 : neighbour };
            catenary::span_points points;
            points.contact_start = catenary::vec3 { wires[ first ][ 0 ], wires[ first ][ 1 ], wires[ first ][ 2 ] };
            points.contact_end = catenary::vec3 { wires[ second ][ 0 ], wires[ second ][ 1 ], wires[ second ][ 2 ] };
            points.messenger_start = catenary::vec3 { ropes[ first ][ 0 ], ropes[ first ][ 1 ], ropes[ first ][ 2 ] };
            points.messenger_end = catenary::vec3 { ropes[ second ][ 0 ], ropes[ second ][ 1 ], ropes[ second ][ 2 ] };
            std::string error;
            auto const gaps { catenary::types().hanger_gaps( type->hanger_table, spanlength ) };
            solved = catenary::solve_span_points( definition, gaps, points, span_y_at( Chain, *type, span, 0 ), span_y_at( Chain, *type, span, 1 ), solution, error )
                  && solution.result.messenger.size() >= 2;
        }
        // punkt liny nośnej jak w przęśle: w planie na cięciwie liny (nie nad osią przewodu jezdnego, który idzie zygzakiem),
        // w pionie nad prostą przewodu jezdnego o wysokość z solvera
        auto const messengerat = [ & ]( double const T ) {
            auto const chord { vec( ropes[ 1 ] ) + ( vec( ropes[ neighbour ] ) - vec( ropes[ 1 ] ) ) * T };
            if( false == solved ) { return chord; }
            auto const wirey { wires[ 1 ][ 1 ] + ( wires[ neighbour ][ 1 ] - wires[ 1 ][ 1 ] ) * T };
            auto const &line { solution.result.messenger };
            auto const x { ( side == 0 ? 1.0 - T : T ) * solution.length };  // przęsło przed środkowym liczone od sąsiada
            auto above { line.back().y };
            for( std::size_t k = 1; k < line.size(); ++k ) {
                if( line[ k ].x >= x ) {
                    auto const f { line[ k ].x > line[ k - 1 ].x ? ( x - line[ k - 1 ].x ) / ( line[ k ].x - line[ k - 1 ].x ) : 0.0 };
                    above = line[ k - 1 ].y + ( line[ k ].y - line[ k - 1 ].y ) * f;
                    break;
                }
            }
            return glm::dvec3 { chord.x, wirey + above, chord.z };
        };
        // zacisk na linie nośnej tam, gdzie ją rysuje przęsło (lina przy podwieszeniu opada, prosta między podwieszeniami
        // mijała ją o kilka cm)
        auto const clamp { tee ? collar : solved && spanlength > clampdistance ? messengerat( clampdistance / spanlength ) : clampchord };
        // koniec liny na słupie sąsiedniego podwieszenia: urządzenie kotwienia stałego tylko z liną
        auto const index { suspension_named( Chain.file, data.suspensions[ position + ( side == 0 ? -1 : 1 ) ] ) };
        glm::dvec3 ropeend { tobeam ? beam : vec( ropes[ neighbour ] ) };
        if( false == tobeam && index < Suspensions.size() && Suspensions[ index ].data.has_pole ) {
            auto const &suspension { Suspensions[ index ].data };
            auto const *pole { catalog->pole( suspension.pole_type ) };
            auto const name { data.middle_device + ( side == 0 ? "" : "-2" ) };
            auto const *device { devices->device( name ) };
            if( device == nullptr ) { device = devices->device( data.middle_device ); }
            if( pole != nullptr && device != nullptr ) {
                glm::dvec3 const axis { suspension.pole[ 0 ], suspension.pole[ 1 ], suspension.pole[ 2 ] };
                // oś słupa wzdłuż toru, prostopadle do jego wysięgnika, w stronę środkowego słupa
                glm::dvec2 cantilever { suspension.has_at ? glm::dvec2 { suspension.at[ 0 ] - axis.x, suspension.at[ 2 ] - axis.z } : glm::dvec2 { 0.0, 0.0 } };
                glm::dvec2 const tomiddle { clamp.x - axis.x, clamp.z - axis.z };
                glm::dvec2 structure { glm::length( cantilever ) > 0.1 ? glm::dvec2 { -cantilever.y, cantilever.x } / glm::length( cantilever ) : glm::normalize( tomiddle ) };
                if( glm::dot( structure, tomiddle ) < 0.0 ) { structure = -structure; }
                auto const face { pole->width > 0.0 ? 0.5 * pole->width : 0.5 * pole->depth( ropes[ neighbour ][ 1 ] - axis.y ) };
                glm::dvec3 const origin { axis.x + structure.x * face, axis.y, axis.z + structure.y * face };
                auto const way { glm::normalize( glm::dvec2 { clamp.x - origin.x, clamp.z - origin.z } ) };
                auto const heading { std::atan2( way.y, -way.x ) };
                auto const toscene = [ & ]( double const *Point ) {
                    catenary::cantilever_placement placement;
                    std::copy( Point, Point + 3, placement.position );
                    auto const part { catenary::cantilever_to_scene( placement, heading ) };
                    return glm::dvec3 { origin.x + part.offset[ 0 ], origin.y + part.offset[ 1 ], origin.z + part.offset[ 2 ] };
                };
                double const zero[ 3 ] { 0.0, 0.0, 0.0 };
                auto const base { toscene( zero ) };
                glm::dvec3 axes[ 3 ];
                for( int a = 0; a < 3; ++a ) {
                    double unit[ 3 ] { 0.0, 0.0, 0.0 };
                    unit[ a ] = 1.0;
                    axes[ a ] = toscene( unit ) - base;
                }
                auto const todevice = [ & ]( glm::dvec3 const &Point, double *Out ) {
                    auto const d { Point - base };
                    for( int a = 0; a < 3; ++a ) { Out[ a ] = glm::dot( d, axes[ a ] ); }
                };
                catenary::tensioner_input input;
                if( type != nullptr ) {
                    input.type = devices->code_for( type->type.symbol, type->type.code );
                    input.contacts = type->type.contact_count;
                    input.messengers = type->type.messenger_count;
                }
                if( device->type( input.type ) == nullptr && false == device->types.empty() ) { input.type = device->types.front().code; }
                input.pole = suspension.pole_type;
                input.depth = [ pole ]( double const Height ) { return pole->width > 0.0 ? pole->width : pole->depth( Height ); };
                input.structure_angle = std::atan2( glm::dot( glm::dvec3 { structure.x, 0.0, structure.y }, axes[ 1 ] ), glm::dot( glm::dvec3 { structure.x, 0.0, structure.y }, axes[ 0 ] ) );
                input.aimed = true;
                todevice( clamp, input.messenger_target );
                todevice( vec( wires[ neighbour ] ), input.contact_target );
                input.rope_height = data.middle_heights[ side ] > 0.0 ? data.middle_heights[ side ] : ropes[ neighbour ][ 1 ] - axis.y;
                input.height = wires[ neighbour ][ 1 ] - axis.y;
                catenary::tensioner_result result;
                if( catenary::solve_tensioner( *devices, *device, input, result ) ) {
                    ropeend = toscene( result.messenger_end );
                    auto const skin { std::string( "none" ) };
                    for( auto const &item : result.parts ) {
                        catenary::cantilever_placement placement;
                        std::copy( std::begin( item.position ), std::end( item.position ), placement.position );
                        std::copy( std::begin( item.rotation ), std::end( item.rotation ), placement.rotation );
                        std::copy( std::begin( item.scale ), std::end( item.scale ), placement.scale );
                        auto const *part { catalog->part( item.part ) };
                        auto const model { catalog->models() + ( part != nullptr && false == part->model.empty() ? part->model : item.part ) };
                        auto const scened { catenary::cantilever_to_scene( placement, heading ) };
                        auto entry {
                            "node " + text_number( data.range_max > 0.0 ? data.range_max : 1000.0, 1 ) + " " + text_number( data.range_min, 1 ) + " none model 0 0 0 0 " + model + ".t3d " + skin
                            + " angles " + text_number( scened.angles[ 0 ] ) + " " + text_number( scened.angles[ 1 ] ) + " " + text_number( scened.angles[ 2 ] ) + " endmodel" };
                        glm::dvec3 const at { origin.x + scened.offset[ 0 ], origin.y + scened.offset[ 1 ], origin.z + scened.offset[ 2 ] };
                        if( auto *model3d { simulation::State.create_model( entry, "none", at ) }; model3d != nullptr ) {
                            catenary_part_detach( model3d );
                            Chain.middle_parts.push_back( model3d );
                        }
                    }
                }
                else {
                    for( auto const &warning : result.warnings ) { ErrorLog( label + "middle anchor can't be assembled: " + warning ); }
                }
            }
        }
        // lina kotwienia: od końca na sąsiednim słupie do zacisku przy słupie środkowym, zwisa jak lina (parabola, 0,5 m
        // na 60 m - ZAŁOŻENIE), nad liną nośną
        auto const hanging = [ & ]( glm::dvec3 const &From, glm::dvec3 const &To, double const Sag, int const Steps ) {
            std::vector<glm::dvec3> points;
            for( int k = 0; k <= Steps; ++k ) {
                auto const t { static_cast<double>( k ) / Steps };
                auto point { From + ( To - From ) * t };
                point.y -= 4.0 * Sag * t * ( 1.0 - t );
                points.push_back( point );
            }
            return points;
        };
        auto const flat = []( glm::dvec3 const &A, glm::dvec3 const &B ) { return glm::length( glm::dvec2 { A.x - B.x, A.z - B.z } ); };
        auto rope { hanging( ropeend, clamp, 0.5 * flat( ropeend, clamp ) / 60.0, 24 ) };
        if( tobeam ) {
            // od dźwigara: uchwyt 4590 i izolator 7150 (bryły zastępcze - wymiary izolatora jak model 7150-01, uchwyt ZAŁOŻENIE)
            auto const down { clamp - beam };
            auto const along { glm::length( down ) > 1e-6 ? glm::normalize( down ) : glm::dvec3 { 0.0, -1.0, 0.0 } };
            add_box( solid, beam, towards, 0.25, 0.08 );
            add_box( solid, beam + along * ( 0.15 + 0.4162 ), along, 0.8324, 0.06 );
        }
        // między zaciskami przy słupie środkowym lina zwisa swobodnie pod uchwytem liny nośnej (Jan 2026-10-25): do połowy
        // odległości (słup środkowy) 0,25 m poniżej liny nośnej
        if( tee ) {
            // obejma na wysięgniku teowym (model 4160 obok siodła; bez siodła w złożeniu - bryła zastępcza nad liną, raz)
            if( side == 0 && glm::length( collar - middlerope ) < 1e-6 ) { add_box( solid, middlerope + glm::dvec3 { 0.0, 0.03, 0.0 }, towards, 0.12, 0.06 ); }
        }
        else if( false == tobeam ) {
            auto const below { hanging( clamp, middlerope, 0.0, 6 ) };
            for( std::size_t k = 1; k < below.size(); ++k ) {
                auto point { below[ k ] };
                auto const t { static_cast<double>( k ) / ( below.size() - 1 ) };
                point.y -= 0.25 * std::sin( 0.5 * 3.14159265358979323846 * t );
                rope.push_back( point );
            }
        }
        add_tube( solid, rope, ropeheight );
        // V (złączka 6410-5 na przewodzie w połowie przęsła, ramiona do liny nośnej 1,50 m w obie strony wzdłuż przęsła).
        // Linka jest sztywna: z zacisku wychodzi chwilę wzdłuż liny nośnej, potem skośnie w dół i dochodzi do złączki wzdłuż
        // przewodu (Jan 2026-10-25, zdjęcie) - krzywa trzeciego stopnia ze styczną do liny i do przewodu, wektory kontrolne
        // po 0,6 odległości w poziomie (ZAŁOŻENIE dobrane na oko do zdjęcia)
        auto const reach { spanlength > 4.0 ? 1.5 / spanlength : 0.0 };
        auto const a { messengerat( 0.5 - reach ) }, b { messengerat( 0.5 + reach ) };
        auto const unit = []( glm::dvec3 const &V ) { return glm::length( V ) > 1e-9 ? glm::normalize( V ) : glm::dvec3 { 1.0, 0.0, 0.0 }; };
        auto const wiredirection { unit( vec( wires[ neighbour ] ) - vec( wires[ 1 ] ) ) };
        auto const step { spanlength > 4.0 ? 0.1 / spanlength : 0.0 };
        auto const stiff = [ & ]( glm::dvec3 const &From, glm::dvec3 const &Fromdirection, glm::dvec3 const &To, glm::dvec3 const &Todirection ) {
            auto const handle { 0.6 * flat( From, To ) };
            auto const p1 { From + Fromdirection * handle }, p2 { To - Todirection * handle };
            std::vector<glm::dvec3> points;
            int const steps { 16 };
            for( int k = 0; k <= steps; ++k ) {
                auto const t { static_cast<double>( k ) / steps }, u { 1.0 - t };
                points.push_back( From * ( u * u * u ) + p1 * ( 3.0 * u * u * t ) + p2 * ( 3.0 * u * t * t ) + To * ( t * t * t ) );
            }
            return points;
        };
        add_tube( solid, stiff( a, unit( messengerat( 0.5 - reach + step ) - a ), wiremiddle, wiredirection ), ropeheight );
        add_tube( solid, stiff( wiremiddle, wiredirection, b, unit( b - messengerat( 0.5 + reach - step ) ) ), ropeheight );
        // bryły zastępcze: zaciski 5560/5570 (na linie nośnej przy słupie środkowym i na końcach V), złączka 6410 na przewodzie
        if( false == tee ) { add_box( solid, clamp, towards, 0.10, 0.045 ); }
        add_box( solid, a, alongspan, 0.08, 0.04 );
        add_box( solid, b, alongspan, 0.08, 0.04 );
        add_box( solid, wiremiddle, alongspan, 0.10, 0.05 );
    }
}

// robi od nowa przęsło k łańcucha (przy podwieszeniu przesuniętym w edytorze tylko dwa przęsła przy nim)
void
rebuild_span( scene_chain &Chain, std::size_t const Span ) {
    if( Span >= Chain.spans.size() ) { return; }
    if( Chain.spans[ Span ] != nullptr ) {
        catenary_span_delete( Chain.spans[ Span ] );
        Chain.spans[ Span ] = nullptr;
    }
    auto const body { span_body( Chain, Span ) };
    if( body.empty() ) { return; }
    Chain.spans[ Span ] = catenary_span_create( span_name( Chain, Span ), Chain.data.range_max, Chain.data.range_min, body, Initialized );
    // przęsło do kotwienia rysuje z bliska jego linę (bryła proceduralna urządzenia naprężającego)
    if( auto *span { Chain.spans[ Span ] }; span != nullptr ) {
        std::vector<catenary::solid_vertex> rope;
        auto const ends { chain_points( Chain ) };
        for( auto const *name : { Span == 0 ? &Chain.data.anchor_start : nullptr, Span + 2 == ends ? &Chain.data.anchor_end : nullptr,
                                  Span == 0 ? &Chain.data.anchor_start_contact : nullptr, Span + 2 == ends ? &Chain.data.anchor_end_contact : nullptr } ) {
            if( name == nullptr || name->empty() ) { continue; }
            if( auto const *anchor { anchor_named( Chain.file, *name ) }; anchor != nullptr && anchor->solved ) {
                rope.insert( rope.end(), anchor->rope.begin(), anchor->rope.end() );
            }
        }
        // kotwienie środkowe: lina, V i zaciski na przęsłach przed i za podwieszeniem środkowym
        if( Span == Chain.middle_span || Span == Chain.middle_span + 1 ) {
            auto const &solid { Chain.middle_solid[ Span == Chain.middle_span ? 0 : 1 ] };
            rope.insert( rope.end(), solid.begin(), solid.end() );
        }
        if( false == rope.empty() ) { catenary_span_attach( *span, rope ); }
    }
}

// wszystkie przęsła łańcucha od nowa (zmiana typu, materiału, listy podwieszeń)
void
rebuild_chain( scene_chain &Chain ) {
    // kotwienia łańcucha od nowa (stan drutów, kierunek do pierwszego/ostatniego podwieszenia), przed przęsłami do nich
    for( auto const *name : { &Chain.data.anchor_start, &Chain.data.anchor_end, &Chain.data.anchor_start_contact, &Chain.data.anchor_end_contact } ) {
        if( auto *anchor { anchor_named( Chain.file, *name ) }; anchor != nullptr ) { assemble_anchor( *anchor ); }
    }
    assemble_middle( Chain );
    for( auto *span : Chain.spans ) { if( span != nullptr ) { catenary_span_delete( span ); } }
    Chain.spans.assign( Chain.removed || Chain.data.suspensions.size() < 2 ? 0 : chain_points( Chain ) - 1, nullptr );
    for( std::size_t span = 0; span < Chain.spans.size(); ++span ) { rebuild_span( Chain, span ); }
}

// numery podwieszeń parasola o podanej nazwie (bez usuniętych)
std::vector<std::size_t>
umbrella_members( std::string const &Name ) {
    std::vector<std::size_t> list;
    for( std::size_t index = 0; index < Suspensions.size(); ++index ) {
        if( false == Suspensions[ index ].removed && Suspensions[ index ].data.has_at && Suspensions[ index ].data.umbrella == Name ) { list.push_back( index ); }
    }
    return list;
}

} // namespace

// obsługa dyrektywy "catenary" ze scenerii: wczytuje wskazany plik podwieszeń
void
catenary_scene_load( cParser &Input ) {

    // nazwa pliku z zachowaniem wielkości liter, potem reszta dyrektywy do endcatenary
    auto const filename { Input.getToken<std::string>( false ) };
    if( filename.empty() || filename == "endcatenary" ) {
        ErrorLog( "Bad scenario: \"catenary\" without a file name in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() ) + ")" );
        return;
    }
    {
        std::string token;
        do {
            token = Input.getToken<std::string>();
        } while( false == token.empty() && token != "endcatenary" );
    }
    if( false == Global.bLoadTraction ) { return; }

    std::vector<std::string> tokens;
    {
        cParser parser( filename, cParser::buffer_FILE, Global.asCurrentSceneryPath, Global.bLoadTraction );
        auto token { parser.getToken<std::string>( false ) };
        while( false == token.empty() ) {
            tokens.emplace_back( token );
            token = parser.getToken<std::string>( false );
        }
    }
    if( tokens.empty() ) {
        ErrorLog( "Bad scenario: catenary file \"" + filename + "\" is missing or empty" );
        return;
    }
    catenary::suspension_file file;
    std::vector<std::string> errors;
    catenary::parse_suspensions( tokens, filename, file, errors );
    for( auto const &error : errors ) {
        ErrorLog( error );
    }
    for( auto const &entry : file.entries ) {
        scene_suspension suspension;
        suspension.data = entry;
        suspension.skin = ( entry.skin.empty() ? file.skin : entry.skin );
        suspension.file = filename;
        Suspensions.emplace_back( suspension );
    }
    for( auto const &line : file.gantries ) {
        scene_gantry gantry;
        gantry.data = line;
        gantry.file = filename;
        Gantries.emplace_back( gantry );
        assemble_gantry( Gantries.back() );
    }
    // kotwienia przed łańcuchami: ostatnie przęsło łańcucha kończy się na ich urządzeniu
    for( auto const &line : file.anchors ) {
        scene_anchor anchor;
        anchor.data = line;
        anchor.file = filename;
        Anchors.emplace_back( anchor );
        assemble_anchor( Anchors.back() );
    }
    // przęsła łańcuchów powstają od razu: przed inicjalizacją scenerii łączy je InitTraction razem z resztą sieci
    for( auto const &line : file.chains ) {
        scene_chain chain;
        chain.data = line;
        chain.file = filename;
        Chains.emplace_back( chain );
        rebuild_chain( Chains.back() );
    }
    if( file_named( filename ) == nullptr ) {
        // komentarze z początku pliku, żeby zapis z edytora ich nie zgubił
        scene_file known;
        known.name = filename;
        known.skin = file.skin;
        std::ifstream source { Global.asCurrentSceneryPath + filename };
        std::string line;
        while( std::getline( source, line ) ) {
            if( false == line.empty() && line.back() == '\r' ) { line.pop_back(); }
            auto const first { line.find_first_not_of( " \t" ) };
            if( first == std::string::npos ) { continue; }
            if( line.compare( first, 2, "//" ) != 0 ) { break; }
            known.header += line + "\n";
        }
        Files.emplace_back( known );
    }
    WriteLog( ( "Catenary: " + std::to_string( file.entries.size() ) + " suspension(s) loaded from file \"" + filename + "\"" ).c_str() );
    if( Initialized ) {
        // dyrektywa po FirstInit: tory, przęsła i modele już są, więc nie ma na co czekać
        catenary_scene_resolve();
    }
}

// po wczytaniu scenerii: wiąże podwieszenia z konstrukcjami wsporczymi i przęsłami po nazwach, zgłasza braki
void
catenary_scene_resolve() {

    Initialized = true;
    if( Resolved >= Suspensions.size() ) { return; }

    auto const first { Resolved };
    std::size_t complete { 0 };
    std::size_t assembled { 0 };
    std::size_t parts { 0 };
    std::map<std::string, std::vector<std::size_t>> umbrellas;     // parasole: nazwa -> wpisy
    for( ; Resolved < Suspensions.size(); ++Resolved ) {
        auto &suspension { Suspensions[ Resolved ] };
        if( suspension.removed ) { continue; }
        auto const &data { suspension.data };
        auto const label { "Bad scenario: suspension \"" + ( data.name.empty() ? std::string( "none" ) : data.name ) + "\" " };
        if( data.has_at ) {
            // wpis z punktami podanymi wprost: nie ma czego wiązać, od razu składanie wysięgnika
            ++complete;
            auto const *catalog { cantilevers() };
            if( catalog == nullptr ) { continue; }
            if( false == data.umbrella.empty() ) { umbrellas[ data.umbrella ].push_back( Resolved ); continue; }   // parasole po pętli, parami
            auto const count { assemble_alone( Resolved, *catalog ) };
            if( count > 0 ) { ++assembled; parts += count; }
            continue;
        }
        bool good { true };
        suspension.support = simulation::Instances.find( data.support );
        if( suspension.support == nullptr ) {
            ErrorLog( label + "refers to nonexistent support model \"" + data.support + "\"" );
            good = false;
        }
        auto const find_spans = [ & ]( std::vector<std::string> const &Names, std::vector<TTraction *> &Spans ) {
            Spans.clear();
            for( auto const &name : Names ) {
                auto *span { simulation::Traction.find( name ) };
                if( span == nullptr ) {
                    ErrorLog( label + "refers to nonexistent traction piece \"" + name + "\"" );
                    good = false;
                    continue;
                }
                if( catenary_span_symbol( *span ).empty() ) {
                    ErrorLog( label + "refers to traction piece \"" + name + "\" which has no network type" );
                    good = false;
                }
                Spans.emplace_back( span );
            }
        };
        find_spans( data.messenger, suspension.messenger );
        find_spans( data.contact, suspension.contact );
        if( good ) { ++complete; }
    }
    auto const *catalog { umbrellas.empty() ? nullptr : cantilevers() };
    if( catalog != nullptr ) {
        for( auto const &umbrella : umbrellas ) { assemble_umbrella( umbrella.first, umbrella.second, *catalog, assembled, parts ); }
    }
    // kotwienia środkowe przy teownikach od nowa: obejma liny kotwienia staje obok siodła liny nośnej, a wysięgniki złożyły
    // się dopiero teraz (łańcuchy powstały przy wczytaniu)
    auto const anymiddle { std::any_of( Chains.begin(), Chains.end(), []( scene_chain const &Chain ) { return false == Chain.removed && false == Chain.data.middle.empty(); } ) };
    if( auto const *cantilevercatalog { anymiddle ? cantilevers() : nullptr }; cantilevercatalog != nullptr ) {
        for( auto &chain : Chains ) {
            if( chain.removed || chain.data.middle.empty() ) { continue; }
            auto const index { suspension_named( chain.file, chain.data.middle ) };
            if( index >= Suspensions.size() || false == Suspensions[ index ].has_saddle ) { continue; }
            auto const *scheme { cantilevercatalog->scheme( Suspensions[ index ].data.card, Suspensions[ index ].data.execution ) };
            if( scheme == nullptr || scheme->assembly.rfind( "tee_", 0 ) != 0 ) { continue; }
            assemble_middle( chain );
            if( chain.middle_span == static_cast<std::size_t>( -1 ) ) { continue; }
            for( auto const span : { chain.middle_span, chain.middle_span + 1 } ) { rebuild_span( chain, span ); }
        }
    }
    WriteLog( ( "Catenary: " + std::to_string( complete ) + " of " + std::to_string( Suspensions.size() - first ) + " suspension(s) linked to supports and traction pieces" ).c_str() );
    if( assembled > 0 ) {
        WriteLog( ( "Catenary: " + std::to_string( assembled ) + " cantilever(s) assembled from " + std::to_string( parts ) + " part(s)" ).c_str() );
    }
}

// zmienia katalog, z którego czytany jest katalog wysięgników; domyślnie "data/". do testów poza symulatorem
void
catenary_scene_data( std::string const &Directory ) {

    Datadirectory = Directory;
    if( false == Datadirectory.empty() && Datadirectory.back() != '/' && Datadirectory.back() != '\\' ) { Datadirectory += '/'; }
}

// przed wczytaniem nowej scenerii: usuwa podwieszenia poprzedniej
void
catenary_scene_clear() {

    Suspensions.clear();
    Chains.clear();
    Gantries.clear();
    Anchors.clear();
    Files.clear();
    Notes.clear();
    Resolved = 0;
    Initialized = false;
}

// ---- edytor scenerii -------------------------------------------------------------------------------------------------

catenary::cantilever_catalog const *
catenary_scene_catalog() {
    return cantilevers();
}

catenary::layout_tables const &
catenary_scene_layout() {
    return layout();
}

catenary::structure_catalog const &
catenary_scene_structures() {
    static catenary::structure_catalog catalog;
    static bool loaded { false };
    if( false == loaded ) {
        loaded = true;
        std::vector<std::string> errors;
        auto const filename { Datadirectory + "traction_catalog.txt" };
        if( false == catalog.load( filename, errors ) ) {
            WriteLog( ( "Catenary: file \"" + filename + "\" is missing, the editor picks the cards without the described catalogue" ).c_str() );
        }
        for( auto const &error : errors ) { ErrorLog( error ); }
    }
    return catalog;
}

std::size_t
catenary_scene_size() {
    return Suspensions.size();
}

catenary::suspension const *
catenary_scene_get( std::size_t const Index ) {
    if( Index >= Suspensions.size() || Suspensions[ Index ].removed ) { return nullptr; }
    return &Suspensions[ Index ].data;
}

std::string
catenary_scene_file_of( std::size_t const Index ) {
    return ( Index < Suspensions.size() ? Suspensions[ Index ].file : std::string() );
}

std::size_t
catenary_scene_parts( std::size_t const Index ) {
    return ( Index < Suspensions.size() ? Suspensions[ Index ].parts.size() : 0 );
}

std::vector<std::string>
catenary_scene_files() {
    std::vector<std::string> names;
    for( auto const &file : Files ) { names.push_back( file.name ); }
    return names;
}

namespace {

// składa wysięgnik podwieszenia od nowa, w parasolu razem z drugim wysięgnikiem
void
reassemble( std::size_t const Index ) {
    auto const *catalog { cantilevers() };
    auto &suspension { Suspensions[ Index ] };
    if( catalog == nullptr || suspension.removed || false == suspension.data.has_at ) { return; }
    std::size_t assembled { 0 }, parts { 0 };
    if( false == suspension.data.umbrella.empty() ) {
        auto const members { umbrella_members( suspension.data.umbrella ) };
        // w edytorze parasol na chwilę ma jeden wysięgnik (drugi dopiero dochodzi albo właśnie odszedł): bez komunikatu
        if( members.size() == 1 ) {
            drop_parts( suspension );
            assemble_alone( Index, *catalog );
            return;
        }
        for( auto const index : members ) { drop_parts( Suspensions[ index ] ); }
        assemble_umbrella( suspension.data.umbrella, members, *catalog, assembled, parts, false );
        return;
    }
    drop_parts( suspension );
    assemble_alone( Index, *catalog );
}

// przęsła łańcuchów przy podwieszeniu idą za nim, kotwienia na jego słupie też
void
follow_chains( std::size_t const Index ) {
    auto const &suspension { Suspensions[ Index ] };
    if( suspension.data.name.empty() ) { return; }
    for( auto &anchor : Anchors ) {
        if( anchor.removed || anchor.file != suspension.file || anchor.data.on != suspension.data.name ) { continue; }
        assemble_anchor( anchor );
        follow_anchor( anchor );
    }
    // trzyma linę w miejscu innych podwieszeń: ich linki celują w nią
    for( auto const &chain : Chains ) {
        if( chain.removed || chain.file != suspension.file ) { continue; }
        for( auto const &pair : chain.data.messenger_from ) {
            if( pair.second != suspension.data.name || pair.first == pair.second ) { continue; }
            if( auto const index { suspension_named( chain.file, pair.first ) }; index < Suspensions.size() && index != Index ) { reassemble( index ); }
        }
    }
    for( auto &chain : Chains ) {
        if( chain.removed || chain.file != suspension.file ) { continue; }
        auto const &names { chain.data.suspensions };
        for( std::size_t position = 0; position < names.size(); ++position ) {
            if( names[ position ] != suspension.data.name && catenary::messenger_holder( chain.data, names[ position ] ) != suspension.data.name ) { continue; }
            // skrajne podwieszenie: kotwienie celuje w nie łańcuchami do drutów
            std::string const *end { position == 0 ? &chain.data.anchor_start : position + 1 == names.size() ? &chain.data.anchor_end : nullptr };
            if( end != nullptr && false == end->empty() ) {
                if( auto *anchor { anchor_named( chain.file, *end ) }; anchor != nullptr ) { assemble_anchor( *anchor ); }
            }
            // przewody zakotwione osobno
            std::string const *endcontact { position == 0 ? &chain.data.anchor_start_contact : position + 1 == names.size() ? &chain.data.anchor_end_contact : nullptr };
            if( endcontact != nullptr && false == endcontact->empty() ) {
                if( auto *anchor { anchor_named( chain.file, *endcontact ) }; anchor != nullptr ) { assemble_anchor( *anchor ); }
            }
            auto const point { position + ( chain.data.anchor_start.empty() ? 0 : 1 ) };
            // kotwienie środkowe przy tym podwieszeniu albo przy sąsiednim: od nowa, z oboma przęsłami przy nim
            if( false == chain.data.middle.empty() ) {
                auto const middle { std::find( names.begin(), names.end(), chain.data.middle ) };
                auto const at { static_cast<std::size_t>( middle - names.begin() ) };
                if( middle != names.end() && position + 1 >= at && position <= at + 1 ) {
                    assemble_middle( chain );
                    for( auto const span : { chain.middle_span, chain.middle_span + 1 } ) {
                        if( span != point && span + 1 != point ) { rebuild_span( chain, span ); }
                    }
                }
            }
            if( point > 0 ) { rebuild_span( chain, point - 1 ); }
            rebuild_span( chain, point );
        }
    }
}

void
mark_changed( std::string const &File ) {
    if( auto *file { file_named( File ) }; file != nullptr ) { file->changed = true; }
}

} // namespace

std::size_t
catenary_scene_add( std::string const &File, catenary::suspension const &Data ) {

    if( file_named( File ) == nullptr ) {
        scene_file created;
        created.name = File;
        created.created = true;
        created.header = "// Plik sieci trakcyjnej (podwieszenia), zalozony w edytorze scenerii.\n";
        Files.emplace_back( created );
    }
    scene_suspension suspension;
    suspension.data = Data;
    suspension.file = File;
    auto const *file { file_named( File ) };
    suspension.skin = ( Data.skin.empty() ? file->skin : Data.skin );
    Suspensions.emplace_back( suspension );
    auto const index { Suspensions.size() - 1 };
    // wpis dodany po wczytaniu scenerii nie przejdzie już przez catenary_scene_resolve()
    if( Resolved == index ) { ++Resolved; }
    mark_changed( File );
    reassemble( index );
    return index;
}

bool
catenary_scene_update( std::size_t const Index, catenary::suspension const &Data ) {

    if( Index >= Suspensions.size() || Suspensions[ Index ].removed ) { return false; }
    auto &suspension { Suspensions[ Index ] };
    // zmiana nazwy parasola: stary partner zostaje sam i też musi się złożyć od nowa
    auto const before { suspension.data.umbrella };
    suspension.data = Data;
    auto const *file { file_named( suspension.file ) };
    suspension.skin = ( Data.skin.empty() ? ( file != nullptr ? file->skin : std::string() ) : Data.skin );
    mark_changed( suspension.file );
    if( false == before.empty() && before != Data.umbrella ) {
        drop_parts( suspension );
        for( auto const index : umbrella_members( before ) ) { reassemble( index ); break; }
    }
    reassemble( Index );
    follow_chains( Index );
    return true;
}

void
catenary_scene_remove( std::size_t const Index ) {

    if( Index >= Suspensions.size() || Suspensions[ Index ].removed ) { return; }
    auto &suspension { Suspensions[ Index ] };
    auto const umbrella { suspension.data.umbrella };
    drop_parts( suspension );
    suspension.removed = true;
    mark_changed( suspension.file );
    if( false == umbrella.empty() ) {
        for( auto const index : umbrella_members( umbrella ) ) { reassemble( index ); break; }
    }
    // z łańcuchów wypada; łańcuch z jednym podwieszeniem przestaje istnieć
    if( false == suspension.data.name.empty() ) {
        for( auto &chain : Chains ) {
            if( chain.removed || chain.file != suspension.file ) { continue; }
            auto &names { chain.data.suspensions };
            auto const before { names.size() };
            names.erase( std::remove( names.begin(), names.end(), suspension.data.name ), names.end() );
            // lina nośna trzymana przez nie albo w jego punkcie: wraca do wysięgnika z listy
            auto &pairs { chain.data.messenger_from };
            auto const pairsbefore { pairs.size() };
            pairs.erase( std::remove_if( pairs.begin(), pairs.end(), [ & ]( auto const &Pair ) { return Pair.first == suspension.data.name || Pair.second == suspension.data.name; } ), pairs.end() );
            if( names.size() == before && pairs.size() == pairsbefore ) { continue; }
            if( names.size() < 2 ) { chain.removed = true; }
            rebuild_chain( chain );
        }
    }
}

bool
catenary_scene_restore( std::size_t const Index, catenary::suspension const &Data ) {

    if( Index >= Suspensions.size() ) { return false; }
    auto &suspension { Suspensions[ Index ] };
    suspension.removed = false;
    suspension.data = Data;
    mark_changed( suspension.file );
    reassemble( Index );
    follow_chains( Index );
    return true;
}

bool
catenary_scene_probe( catenary::suspension const &Data, catenary::cantilever_result &Result ) {

    Result = catenary::cantilever_result();
    auto const *catalog { cantilevers() };
    if( catalog == nullptr ) { Result.warnings.push_back( "the cantilever catalogue isn't loaded" ); return false; }
    if( false == Data.has_at || false == ( Data.has_pole || Data.has_attach ) ) { Result.warnings.push_back( "the suspension has no explicit points" ); return false; }
    scene_suspension probe;
    probe.data = Data;
    glm::dvec3 wire, rope, attach;
    entry_points( Data, wire, rope, attach );
    return assemble( probe, *catalog, wire, ( Data.has_rope ? &rope : nullptr ), attach, nullptr, &Result ) > 0;
}

bool
catenary_scene_changed() {
    return std::any_of( Files.begin(), Files.end(), []( scene_file const &File ) { return File.changed; } );
}

std::vector<std::string>
catenary_scene_new_files() {
    std::vector<std::string> names;
    for( auto const &file : Files ) { if( file.created ) { names.push_back( file.name ); } }
    return names;
}

void
catenary_scene_file_included( std::string const &File ) {
    if( auto *file { file_named( File ) }; file != nullptr ) { file->created = false; }
}

bool
catenary_scene_save( std::string &Message ) {

    std::vector<std::string> written;
    for( auto &file : Files ) {
        if( false == file.changed ) { continue; }
        // wpisy w kolejności listy; usunięte pomijane. CRLF jak w pozostałych plikach scenerii
        std::string text;
        std::istringstream header { file.header };
        for( std::string line; std::getline( header, line ); ) { text += line + "\r\n"; }
        if( false == file.skin.empty() ) { text += "skin " + file.skin + "\r\n"; }
        std::size_t count { 0 };
        for( auto const &suspension : Suspensions ) {
            if( suspension.removed || suspension.file != file.name ) { continue; }
            text += catenary::write_suspension( suspension.data ) + "\r\n";
            ++count;
        }
        for( auto const &chain : Chains ) {
            if( chain.removed || chain.file != file.name ) { continue; }
            text += catenary::write_chain( chain.data ) + "\r\n";
        }
        for( auto const &gantry : Gantries ) {
            if( gantry.removed || gantry.file != file.name ) { continue; }
            text += catenary::write_gantry( gantry.data ) + "\r\n";
        }
        for( auto const &anchor : Anchors ) {
            if( anchor.removed || anchor.file != file.name ) { continue; }
            text += catenary::write_anchor( anchor.data ) + "\r\n";
        }
        auto const path { Global.asCurrentSceneryPath + file.name };
        std::ofstream output { path, std::ios::binary | std::ios::trunc };
        if( false == output.is_open() ) {
            Message = "catenary file \"" + file.name + "\" can't be written";
            return false;
        }
        output << text;
        output.close();
        if( output.fail() ) {
            Message = "catenary file \"" + file.name + "\" wasn't written completely";
            return false;
        }
        file.changed = false;
        written.push_back( file.name + " (" + std::to_string( count ) + ")" );
    }
    Message.clear();
    for( auto const &name : written ) { Message += ( Message.empty() ? "" : ", " ) + name; }
    return true;
}

// ---- łańcuchy ----------------------------------------------------------------------------------------------------------

std::string
catenary_scene_new_name( std::string const &File ) {
    // s0001, s0002, ... pierwsza wolna w pliku
    for( int number = 1; ; ++number ) {
        auto name { std::to_string( number ) };
        name = "s" + std::string( name.size() < 4 ? 4 - name.size() : 0, '0' ) + name;
        auto const taken { std::any_of( Suspensions.begin(), Suspensions.end(), [ & ]( scene_suspension const &Suspension ) { return false == Suspension.removed && Suspension.file == File && Suspension.data.name == name; } ) };
        if( false == taken ) { return name; }
    }
}

std::string
catenary_scene_new_chain_name( std::string const &File ) {
    auto stem { File };
    if( auto const slash { stem.find_last_of( "/\\" ) }; slash != std::string::npos ) { stem.erase( 0, slash + 1 ); }
    if( auto const dot { stem.rfind( '.' ) }; dot != std::string::npos ) { stem.erase( dot ); }
    for( int number = 1; ; ++number ) {
        auto const name { stem + "_l" + std::to_string( number ) };
        auto const taken { std::any_of( Chains.begin(), Chains.end(), [ & ]( scene_chain const &Chain ) { return Chain.data.name == name; } ) };
        if( false == taken ) { return name; }
    }
}

std::size_t
catenary_chain_size() {
    return Chains.size();
}

catenary::chain const *
catenary_chain_get( std::size_t const Index ) {
    if( Index >= Chains.size() || Chains[ Index ].removed ) { return nullptr; }
    return &Chains[ Index ].data;
}

std::string
catenary_chain_file( std::size_t const Index ) {
    return ( Index < Chains.size() ? Chains[ Index ].file : std::string() );
}

std::vector<std::size_t>
catenary_chain_members( std::size_t const Index ) {
    std::vector<std::size_t> members;
    if( Index >= Chains.size() || Chains[ Index ].removed ) { return members; }
    for( auto const &name : Chains[ Index ].data.suspensions ) { members.push_back( suspension_named( Chains[ Index ].file, name ) ); }
    return members;
}

std::size_t
catenary_chain_add( std::string const &File, catenary::chain const &Data ) {
    scene_chain chain;
    chain.data = Data;
    chain.file = File;
    Chains.emplace_back( chain );
    rebuild_chain( Chains.back() );
    mark_changed( File );
    return Chains.size() - 1;
}

bool
catenary_chain_update( std::size_t const Index, catenary::chain const &Data ) {
    if( Index >= Chains.size() ) { return false; }
    auto &chain { Chains[ Index ] };
    // podwieszenia, których linę trzyma (trzymał) inny wysięgnik: ich linki celują w linę, więc składane od nowa
    std::set<std::string> held;
    for( auto const *pairs : { &static_cast<catenary::chain const &>( chain.data ).messenger_from, &Data.messenger_from } ) {
        for( auto const &pair : *pairs ) { if( pair.first != pair.second ) { held.insert( pair.first ); } }
    }
    chain.data = Data;
    chain.removed = Data.suspensions.size() < 2;
    for( auto const &name : held ) {
        if( auto const index { suspension_named( chain.file, name ) }; index < Suspensions.size() ) { reassemble( index ); }
    }
    rebuild_chain( chain );
    mark_changed( chain.file );
    return true;
}

void
catenary_chain_remove( std::size_t const Index ) {
    if( Index >= Chains.size() || Chains[ Index ].removed ) { return; }
    Chains[ Index ].removed = true;
    rebuild_chain( Chains[ Index ] );
    mark_changed( Chains[ Index ].file );
}

bool
catenary_chain_middle( std::size_t const Index, std::size_t &Parts, std::size_t &Triangles ) {
    Parts = Triangles = 0;
    if( Index >= Chains.size() || Chains[ Index ].removed || Chains[ Index ].data.middle.empty() ) { return false; }
    auto const &chain { Chains[ Index ] };
    Parts = chain.middle_parts.size();
    Triangles = ( chain.middle_solid[ 0 ].size() + chain.middle_solid[ 1 ].size() ) / 3;
    return chain.middle_span != static_cast<std::size_t>( -1 );
}

int
catenary_chain_of( std::size_t const Suspension, std::size_t &Position ) {
    if( Suspension >= Suspensions.size() || Suspensions[ Suspension ].data.name.empty() ) { return -1; }
    auto const &suspension { Suspensions[ Suspension ] };
    for( std::size_t index = 0; index < Chains.size(); ++index ) {
        auto const &chain { Chains[ index ] };
        if( chain.removed || chain.file != suspension.file ) { continue; }
        auto const found { std::find( chain.data.suspensions.begin(), chain.data.suspensions.end(), suspension.data.name ) };
        if( found != chain.data.suspensions.end() ) {
            Position = static_cast<std::size_t>( found - chain.data.suspensions.begin() );
            return static_cast<int>( index );
        }
    }
    return -1;
}

// ---- bramki -------------------------------------------------------------------------------------------------------------

std::size_t
catenary_gantry_size() {
    return Gantries.size();
}

catenary::gantry const *
catenary_gantry_get( std::size_t const Index ) {
    if( Index >= Gantries.size() || Gantries[ Index ].removed ) { return nullptr; }
    return &Gantries[ Index ].data;
}

std::string
catenary_gantry_file( std::size_t const Index ) {
    return ( Index < Gantries.size() ? Gantries[ Index ].file : std::string() );
}

std::size_t
catenary_gantry_parts( std::size_t const Index ) {
    return ( Index < Gantries.size() ? Gantries[ Index ].parts.size() : 0 );
}

namespace {
// przęsła bramki wieloprzęsłowej stykające się z przęsłem Index w jego starych (Before) albo obecnych słupach - składane od
// nowa, bo wspólny słup stawia przęsło o najniższym numerze
void
reassemble_gantry_neighbours( std::size_t const Index, catenary::gantry const &Before ) {
    std::set<std::size_t> touched;
    for( int k = 0; k < 2; ++k ) {
        for( auto const other : gantries_at( Before.points[ k ], Index ) ) { touched.insert( other ); }
        if( false == Gantries[ Index ].removed ) {
            for( auto const other : gantries_at( Gantries[ Index ].data.points[ k ], Index ) ) { touched.insert( other ); }
        }
    }
    for( auto const other : touched ) { assemble_gantry( Gantries[ other ] ); }
}
} // namespace

std::size_t
catenary_gantry_add( std::string const &File, catenary::gantry const &Data ) {
    if( file_named( File ) == nullptr ) {
        scene_file created;
        created.name = File;
        created.created = true;
        created.header = "// Plik sieci trakcyjnej (podwieszenia), zalozony w edytorze scenerii.\n";
        Files.emplace_back( created );
    }
    scene_gantry gantry;
    gantry.data = Data;
    gantry.file = File;
    Gantries.emplace_back( gantry );
    assemble_gantry( Gantries.back() );
    mark_changed( File );
    return Gantries.size() - 1;
}

bool
catenary_gantry_update( std::size_t const Index, catenary::gantry const &Data ) {
    if( Index >= Gantries.size() ) { return false; }
    auto &gantry { Gantries[ Index ] };
    auto const before { gantry.data };
    gantry.data = Data;
    gantry.removed = false;
    assemble_gantry( gantry );
    mark_changed( gantry.file );
    reassemble_gantry_neighbours( Index, before );
    // wysięgniki na słupach tej bramki: mocowania pod dźwigarem na jej (może zmienionym) poziomie
    if( auto const *catalog { cantilevers() }; catalog != nullptr ) {
        for( std::size_t index = 0; index < Suspensions.size(); ++index ) {
            auto const &data { Suspensions[ index ].data };
            if( Suspensions[ index ].removed || false == data.has_pole ) { continue; }
            auto const *pole { catalog->pole( data.pole_type ) };
            if( pole != nullptr && pole->gantry && gantry_at_mast( data.pole_type, data.pole ) == &gantry.data ) { reassemble( index ); }
        }
    }
    return true;
}

void
catenary_gantry_remove( std::size_t const Index ) {
    if( Index >= Gantries.size() || Gantries[ Index ].removed ) { return; }
    Gantries[ Index ].removed = true;
    assemble_gantry( Gantries[ Index ] );
    mark_changed( Gantries[ Index ].file );
    // wspólny słup, który stawiało to przęsło, stawia teraz sąsiednie
    reassemble_gantry_neighbours( Index, Gantries[ Index ].data );
}

std::string
catenary_scene_new_gantry_name( std::string const &File ) {
    auto stem { File };
    if( auto const slash { stem.find_last_of( "/\\" ) }; slash != std::string::npos ) { stem.erase( 0, slash + 1 ); }
    if( auto const dot { stem.rfind( '.' ) }; dot != std::string::npos ) { stem.erase( dot ); }
    for( int number = 1; ; ++number ) {
        auto const name { stem + "_b" + std::to_string( number ) };
        if( std::none_of( Gantries.begin(), Gantries.end(), [ & ]( scene_gantry const &Gantry ) { return Gantry.data.name == name; } ) ) { return name; }
    }
}

// ---- kotwienia ---------------------------------------------------------------------------------------------------------

namespace {

// przęsła łańcucha z kotwieniem idą za nim: przęsło do kotwienia (położenie końców), a pozostałe przez naciąg
// (brakujące ciężary zmieniają zwis wszystkich przęseł do kotwienia środkowego)
void
follow_anchor( scene_anchor const &Anchor ) {
    for( auto &chain : Chains ) {
        if( chain.removed || chain.file != Anchor.file || Anchor.data.name.empty() ) { continue; }
        if( false == anchors_chain( chain.data, Anchor.data.name ) ) { continue; }
        for( std::size_t span = 0; span < chain.spans.size(); ++span ) { rebuild_span( chain, span ); }
    }
}

} // namespace

std::size_t
catenary_anchor_size() {
    return Anchors.size();
}

catenary::anchor const *
catenary_anchor_get( std::size_t const Index ) {
    if( Index >= Anchors.size() || Anchors[ Index ].removed ) { return nullptr; }
    return &Anchors[ Index ].data;
}

std::string
catenary_anchor_file( std::size_t const Index ) {
    return ( Index < Anchors.size() ? Anchors[ Index ].file : std::string() );
}

std::size_t
catenary_anchor_parts( std::size_t const Index ) {
    return ( Index < Anchors.size() ? Anchors[ Index ].parts.size() : 0 );
}

catenary::tensioner_result const *
catenary_anchor_result( std::size_t const Index ) {
    if( Index >= Anchors.size() || Anchors[ Index ].removed ) { return nullptr; }
    return &Anchors[ Index ].result;
}

std::vector<catenary_anchor_card>
catenary_anchor_devices( std::string const &Only ) {
    std::vector<catenary_anchor_card> result;
    // urządzenia kotwienia środkowego (lina kotwienia środkowego) nie są kotwieniem końca łańcucha
    static std::set<std::string> const middle { "0406", "0406-2", "0408", "0408-2", "0418", "0418-2" };
    if( auto const *devices { tensioners() }; devices != nullptr ) {
        for( auto const &device : devices->devices() ) {
            if( device.only != Only || middle.count( device.name ) ) { continue; }
            // odmiany dla typów sieci (-10, -32, ...) dobiera się samo; połówki rozdzielonego (-m, -c) i nazwane - na liście
            auto const dash { device.name.find( '-' ) };
            auto const half { dash != std::string::npos && ( device.name.substr( dash ) == "-m" || device.name.substr( dash ) == "-c" ) };
            if( dash == std::string::npos || false == device.title.empty() || ( false == Only.empty() && half ) ) {
                result.push_back( { device.name, device.fixed, false == device.small_pulley.empty() || device.split, device.split, device.title } );
            }
        }
    }
    return result;
}

catenary::tensioner_catalog const *
catenary_scene_tensioners() {
    return tensioners();
}

bool
catenary_anchor_shape( std::size_t const Index, double ( &Shape )[ 4 ][ 3 ] ) {
    if( Index >= Anchors.size() || Anchors[ Index ].removed || false == Anchors[ Index ].solved ) { return false; }
    for( int k = 0; k < 4; ++k ) { std::copy( std::begin( Anchors[ Index ].shape[ k ] ), std::end( Anchors[ Index ].shape[ k ] ), Shape[ k ] ); }
    return true;
}

int
catenary_anchor_chain( std::size_t const Index, bool &Start ) {
    if( Index >= Anchors.size() || Anchors[ Index ].removed ) { return -1; }
    auto const &anchor { Anchors[ Index ] };
    for( std::size_t k = 0; k < Chains.size(); ++k ) {
        auto const &chain { Chains[ k ] };
        if( chain.removed || chain.file != anchor.file || anchor.data.name.empty() ) { continue; }
        if( anchors_chain( chain.data, anchor.data.name, &Start ) ) { return static_cast<int>( k ); }
    }
    return -1;
}

std::size_t
catenary_anchor_add( std::string const &File, catenary::anchor const &Data ) {
    if( file_named( File ) == nullptr ) {
        scene_file created;
        created.name = File;
        created.created = true;
        Files.emplace_back( created );
    }
    scene_anchor anchor;
    anchor.data = Data;
    anchor.file = File;
    Anchors.emplace_back( anchor );
    assemble_anchor( Anchors.back() );
    follow_anchor( Anchors.back() );
    mark_changed( File );
    return Anchors.size() - 1;
}

bool
catenary_anchor_update( std::size_t const Index, catenary::anchor const &Data ) {
    if( Index >= Anchors.size() ) { return false; }
    auto &anchor { Anchors[ Index ] };
    anchor.data = Data;
    anchor.removed = false;
    assemble_anchor( anchor );
    follow_anchor( anchor );
    mark_changed( anchor.file );
    return true;
}

void
catenary_anchor_remove( std::size_t const Index ) {
    if( Index >= Anchors.size() || Anchors[ Index ].removed ) { return; }
    auto &anchor { Anchors[ Index ] };
    anchor.removed = true;
    assemble_anchor( anchor );
    anchor.solved = false;
    follow_anchor( anchor );    // przęsło do usuniętego kotwienia znika (łańcuch kończy się na ostatnim podwieszeniu)
    mark_changed( anchor.file );
}

std::string
catenary_scene_new_anchor_name( std::string const &File ) {
    for( int number = 1; ; ++number ) {
        auto const name { "k" + std::to_string( number ) };
        auto used { false };
        for( auto const &anchor : Anchors ) { if( false == anchor.removed && anchor.file == File && anchor.data.name == name ) { used = true; break; } }
        if( false == used ) { return name; }
    }
}

void
catenary_scene_temperature() {
    for( auto &anchor : Anchors ) {
        if( anchor.removed || std::abs( anchor.temperature - Global.AirTemperature ) < 0.5 ) { continue; }
        assemble_anchor( anchor );
        follow_anchor( anchor );
    }
}
