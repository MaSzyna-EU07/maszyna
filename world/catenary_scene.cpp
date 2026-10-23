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
    double temperature { 0.0 };                // temperatura, przy której złożono
    catenary::tensioner_result result;
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
    for( auto const &placement : result.parts ) {
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

// składa wysięgnik wpisu z punktami, bez parasola; zwraca liczbę wstawionych części
std::size_t
assemble_alone( std::size_t const Index, catenary::cantilever_catalog const &Catalog ) {
    auto &suspension { Suspensions[ Index ] };
    glm::dvec3 wire, rope, attach;
    entry_points( suspension.data, wire, rope, attach );
    return assemble( suspension, Catalog, wire, ( suspension.data.has_rope ? &rope : nullptr ), attach );
}

// parasol: oba wysięgniki składane najpierw osobno, żeby poznać wysokości mocowań, potem razem na wspólnym słupie
void
assemble_umbrella( std::string const &Name, std::vector<std::size_t> const &List, catenary::cantilever_catalog const &Catalog, std::size_t &Assembled, std::size_t &Parts ) {
    auto const alone = [ & ]( std::size_t const Index ) {
        auto const count { assemble_alone( Index, Catalog ) };
        if( count > 0 ) { ++Assembled; Parts += count; }
    };
    auto const &first { Suspensions[ List.front() ].data };
    auto const *pole { Catalog.pole( first.pole_type ) };
    auto paired { List.size() == 2 && pole != nullptr };
    if( List.size() != 2 ) {
        ErrorLog( "Bad scenario: umbrella \"" + Name + "\" has " + std::to_string( List.size() ) + " suspension(s), it needs exactly two; they're assembled separately" );
    }
    else {
        auto const &second { Suspensions[ List.back() ].data };
        auto const dx { second.pole[ 0 ] - first.pole[ 0 ] }, dy { second.pole[ 1 ] - first.pole[ 1 ] }, dz { second.pole[ 2 ] - first.pole[ 2 ] };
        if( second.pole_type != first.pole_type || std::sqrt( dx * dx + dy * dy + dz * dz ) > 0.05 ) {
            ErrorLog( "Bad scenario: umbrella \"" + Name + "\" has its suspensions on different poles; they're assembled separately" );
            paired = false;
        }
    }
    if( false == paired ) { for( auto const index : List ) { alone( index ); } return; }
    glm::dvec3 wires[ 2 ], ropes[ 2 ], poles[ 2 ];
    catenary::cantilever_result probes[ 2 ];
    auto probed { true };
    for( int side = 0; side < 2; ++side ) {
        auto &suspension { Suspensions[ List[ side ] ] };
        entry_points( suspension.data, wires[ side ], ropes[ side ], poles[ side ] );
        probed = probed && assemble( suspension, Catalog, wires[ side ], ( suspension.data.has_rope ? &ropes[ side ] : nullptr ), poles[ 0 ], nullptr, &probes[ side ] ) > 0;
    }
    if( false == probed ) { for( auto const index : List ) { alone( index ); } return; }
    catenary::cantilever_umbrella sides[ 2 ];
    catenary::umbrella_mounts( Catalog, *pole, probes[ 0 ], probes[ 1 ], sides[ 0 ], sides[ 1 ] );
    sides[ 1 ].pole = false;
    for( int side = 0; side < 2; ++side ) {
        auto &suspension { Suspensions[ List[ side ] ] };
        auto const count { assemble( suspension, Catalog, wires[ side ], ( suspension.data.has_rope ? &ropes[ side ] : nullptr ), poles[ 0 ], &sides[ side ] ) };
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
    auto const *pole { catalog->pole( data.pole_type ) };
    if( pole == nullptr ) { ErrorLog( label + "uses pole \"" + data.pole_type + "\" which isn't in the supports catalogue" ); return 0; }
    catenary::tensioner_input input;
    if( false == data.network.empty() ) {
        if( auto const *type { catenary::types().find( data.network ) }; type != nullptr ) {
            input.type = type->type.code;
            input.contacts = type->type.contact_count;
        }
        else { ErrorLog( label + "uses network type \"" + data.network + "\" which has no file" ); }
    }
    input.temperature = Global.AirTemperature;
    input.distance = data.distance;
    input.elongation = data.elongation;
    input.missing = data.missing;
    input.ground = data.ground;
    input.color = data.color;
    input.top_color = data.top_color;
    auto &result { Anchor.result };
    if( false == catenary::solve_tensioner( *devices, *device, input, result ) ) {
        for( auto const &warning : result.warnings ) { ErrorLog( label + "can't be assembled: " + warning ); }
        return 0;
    }
    for( auto const &warning : result.warnings ) {
        if( Notes.insert( warning ).second ) { WriteLog( ( "Catenary: " + warning + " (first seen in anchor \"" + data.name + "\")" ).c_str() ); }
    }
    if( auto const *cord { catalog->part( device->rope ) }; cord != nullptr ) {
        catenary::tensioner_rope_parts( result, device->rope, cord->length, cord->girth );
    }
    else { ErrorLog( label + "rope part \"" + device->rope + "\" isn't in the cantilever catalogue, the rope isn't drawn" ); }
    Anchor.temperature = Global.AirTemperature;

    // kierunek drutów w planie; zero urządzenia na czole słupa
    glm::dvec3 const axis { data.pole[ 0 ], data.pole[ 1 ], data.pole[ 2 ] };
    auto dx { data.toward[ 0 ] - axis.x }, dz { data.toward[ 2 ] - axis.z };
    auto const along { std::sqrt( dx * dx + dz * dz ) };
    if( along < 1.0 ) { ErrorLog( label + "has its \"toward\" point less than 1 m from the pole" ); return 0; }
    dx /= along;
    dz /= along;
    auto const table { devices->table( device->table ) };
    auto const face { 0.5 * pole->depth( table != nullptr ? table->height : 6.7 ) };
    glm::dvec3 const origin { axis.x + dx * face, axis.y, axis.z + dz * face };
    auto const heading { std::atan2( dz, -dx ) };
    auto const toscene = [ & ]( double const *Point ) {
        catenary::cantilever_placement placement;
        std::copy( Point, Point + 3, placement.position );
        auto const part { catenary::cantilever_to_scene( placement, heading ) };
        return glm::dvec3 { origin.x + part.offset[ 0 ], origin.y + part.offset[ 1 ], origin.z + part.offset[ 2 ] };
    };
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
        for( int k = 0; k < 3; ++k ) { Anchor.contact[ k ] = contact[ k ] / ends; }
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
    // słup: Y wysięgnika od toru do słupa (słup po tej stronie drutów, po której stoi względem punktu "toward")
    {
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
        std::copy( std::begin( anchor->contact ), std::end( anchor->contact ), Wire );
        std::copy( std::begin( anchor->messenger ), std::end( anchor->messenger ), Rope );
        return true;
    }
    auto const index { suspension_named( Chain.file, data.suspensions[ Point ] ) };
    if( index >= Suspensions.size() || false == Suspensions[ index ].data.has_at ) { return false; }
    std::copy( std::begin( Suspensions[ index ].data.at ), std::end( Suspensions[ index ].data.at ), Wire );
    rope_of( Suspensions[ index ].data, Rope );
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
    if( false == data.type.empty() ) { body += " type " + data.type; }
    if( false == data.parallel.empty() ) { body += " parallel " + data.parallel; }
    return body + " endtraction";
}

std::string
span_name( scene_chain const &Chain, std::size_t const Span ) {
    return ( Chain.data.name.empty() ? std::string( "none" ) : Chain.data.name + "_" + std::to_string( Span + 1 ) );
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
}

// wszystkie przęsła łańcucha od nowa (zmiana typu, materiału, listy podwieszeń)
void
rebuild_chain( scene_chain &Chain ) {
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
        for( auto const index : members ) { drop_parts( Suspensions[ index ] ); }
        assemble_umbrella( suspension.data.umbrella, members, *catalog, assembled, parts );
        return;
    }
    drop_parts( suspension );
    assemble_alone( Index, *catalog );
}

// przęsła łańcuchów przy podwieszeniu idą za nim
void
follow_chains( std::size_t const Index ) {
    auto const &suspension { Suspensions[ Index ] };
    if( suspension.data.name.empty() ) { return; }
    for( auto &chain : Chains ) {
        if( chain.removed || chain.file != suspension.file ) { continue; }
        auto const &names { chain.data.suspensions };
        for( std::size_t position = 0; position < names.size(); ++position ) {
            if( names[ position ] != suspension.data.name ) { continue; }
            auto const point { position + ( chain.data.anchor_start.empty() ? 0 : 1 ) };
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
            if( names.size() == before ) { continue; }
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
    chain.data = Data;
    chain.removed = Data.suspensions.size() < 2;
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

// przęsła do kotwienia (pierwsze albo ostatnie łańcucha) idą za nim
void
follow_anchor( scene_anchor const &Anchor ) {
    for( auto &chain : Chains ) {
        if( chain.removed || chain.file != Anchor.file || Anchor.data.name.empty() ) { continue; }
        if( chain.data.anchor_start == Anchor.data.name && false == chain.spans.empty() ) { rebuild_span( chain, 0 ); }
        if( chain.data.anchor_end == Anchor.data.name && false == chain.spans.empty() ) { rebuild_span( chain, chain.spans.size() - 1 ); }
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
