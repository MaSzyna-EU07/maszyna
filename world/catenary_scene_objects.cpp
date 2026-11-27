/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Części wysięgników sieci trakcyjnej jako obiekty scenerii: odłączenie od pliku scenerii i usunięcie.
// Osobno od catenary_scene.cpp, żeby testy poza symulatorem mogły podstawić atrapę

#include "stdafx.h"

#include "model/AnimModel.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "world/Traction.h"
#include "world/TractionPower.h"
#include "world/catenary_span.h"

// część należy do pliku sieci, nie do pliku scenerii, w którym akurat jest dyrektywa: bez warstwy zapis scenerii
// w edytorze jej nie przepisuje (inaczej każda część wysięgnika trafiłaby do scenerii jako zwykły model)
void
catenary_part_detach( TAnimModel *Model ) {

    if( Model == nullptr ) { return; }
    scene::Layers.count( Model->layer(), scene::layer_item::model, -1 );
    Model->layer( null_handle );
    // edytor nie rusza jej jak zwykłego modelu (przesunięta albo usunięta osobno odpadłaby od sieci, która ją trzyma)
    Model->m_catenary = true;
}

void
catenary_part_delete( TAnimModel *Model ) {

    if( Model == nullptr ) { return; }
    simulation::State.delete_model( Model );
}

// przęsło łańcucha jako zwykły TTraction, wczytany z treści wpisu node traction. Bez warstwy scenerii: zapis scenerii
// w edytorze go nie przepisuje, przęsła łańcucha zapisują się w pliku sieci jako jeden wpis
TTraction *
catenary_span_create( std::string const &Name, double const RangeMax, double const RangeMin, std::string const &Body, bool const Link ) {

    scene::node_data nodedata;
    nodedata.name = Name;
    nodedata.type = "traction";
    nodedata.range_max = RangeMax;
    nodedata.range_min = RangeMin;
    cParser parser( Body );
    auto *span { new TTraction( nodedata ) };
    span->Load( &parser, glm::dvec3() );
    span->m_catenary = true;
    if( false == simulation::Traction.insert( span ) ) {
        ErrorLog( "Bad scenario: duplicate traction piece name \"" + span->name() + "\" made by a catenary chain" );
    }
    simulation::Region->insert_and_register( span );
    if( false == Link ) { return span; }
    // po inicjalizacji scenerii: zasilacz i sąsiednie przęsła, tak jak robi to InitTraction przy wczytaniu
    if( auto *powersource { simulation::Powergrid.find( span->asPowerSupplyName ) }; powersource != nullptr ) {
        span->PowerSet( powersource );
    }
    std::vector<TTraction *> neighbours;
    for( int end = 0; end < 2; ++end ) {
        auto const [ matching, connection ] = simulation::Region->find_traction( end == 0 ? span->pPoint1 : span->pPoint2, span );
        if( matching != nullptr && ( connection == 0 || connection == 1 ) && matching->hvNext[ connection ] == nullptr ) {
            span->Connect( end, matching, connection );
            if( span->psSection == nullptr && matching->psSection != nullptr ) { span->PowerSet( matching->psSection ); }
            neighbours.push_back( matching );
        }
    }
    auto &section { simulation::Region->section( span->location() ) };
    if( section.m_geometrycreated ) {
        span->create_geometry( section.m_geometrybank );
    }
    // sąsiedzi mają teraz z tej strony przęsło (np. wspólną linkę Y): ich linie i bryły od nowa
    for( auto *neighbour : neighbours ) { catenary_span_refresh( *neighbour ); }
    return span;
}

void
catenary_span_delete( TTraction *Span ) {

    if( Span == nullptr ) { return; }
    // sąsiednie przęsła tracą połączenie z tym, a z nim np. wspólną linkę Y: ich linie i bryły od nowa
    std::vector<TTraction *> neighbours;
    for( int end = 0; end < 2; ++end ) {
        auto *neighbour { Span->hvNext[ end ] };
        if( neighbour == nullptr ) { continue; }
        for( int other = 0; other < 2; ++other ) {
            if( neighbour->hvNext[ other ] == Span ) {
                neighbour->hvNext[ other ] = nullptr;
                neighbour->iLast = 1;
            }
        }
        neighbours.push_back( neighbour );
    }
    for( auto *neighbour : neighbours ) { catenary_span_refresh( *neighbour ); }
    catenary_span_release( *Span );
    simulation::Region->erase_and_unregister( Span );
    auto const name { Span->name() };
    simulation::Traction.purge( Span );
    // nazwa prowadziła do usuniętego przęsła: zwolniona, żeby nowe przęsło o tej nazwie dało się znaleźć
    if( false == name.empty() && simulation::Traction.find( name ) == nullptr ) { simulation::Traction.detach( name ); }
}
