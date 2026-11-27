/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Dopiski wpisu "node ... traction" po "vis"/"unvis", do "endtraction": czytane przy wczytaniu przęsła i zapisywane
// z powrotem przy zapisie z edytora, żeby zapis nie gubił niczego, co wczytanie przeczytało

#pragma once

#include <array>
#include <ostream>
#include <string>

#include "utilities/parser.h"
#include "utilities/utilities.h"

struct traction_tail {
    std::string parallel; // "parallel <przęsło>": bieżnia wspólna
    std::string type; // "type <symbol>": typ sieci, wielkość liter zachowana
    std::array<double, 2> tension { 0.0, 0.0 }; // "tension <lina nośna> <przewody jezdne>" w daN; 0 = jak w typie
    std::array<glm::dvec3, 2> rope2 {}; // "rope2 start|end x y z": końce drugiej liny nośnej, w układzie scenerii
    std::array<bool, 2> rope2set { false, false };
};

// czyta dopiski do "endtraction" włącznie; Origin: przesunięcie punktów (origin scenerii)
inline void
read_traction_tail( cParser &Parser, glm::dvec3 const &Origin, traction_tail &Tail ) {
    auto token { Parser.getToken<std::string>() };
    while( false == token.empty() && token != "endtraction" ) {
        if( token == "parallel" ) {
            Parser.getTokens();
            Parser >> Tail.parallel;
        }
        else if( token == "type" ) {
            Tail.type = Parser.getToken<std::string>( false );
        }
        else if( token == "rope2" ) {
            auto const which { Parser.getToken<std::string>() };
            auto const point { LoadPoint( Parser ) + Origin };
            auto const end { which == "end" ? 1 : 0 };
            Tail.rope2[ end ] = point;
            Tail.rope2set[ end ] = true;
        }
        else if( token == "tension" ) {
            Parser.getTokens( 2 );
            Parser >> Tail.tension[ 0 ] >> Tail.tension[ 1 ];
        }
        token = Parser.getToken<std::string>();
    }
}

// zapisuje dopiski w postaci, którą czyta read_traction_tail(), bez "endtraction"; każdy zakończony spacją
inline void
write_traction_tail( std::ostream &Output, traction_tail const &Tail ) {
    if( false == Tail.parallel.empty() ) {
        Output << "parallel " << Tail.parallel << ' ';
    }
    if( false == Tail.type.empty() ) {
        Output << "type " << Tail.type << ' ';
    }
    if( Tail.tension[ 0 ] != 0.0 || Tail.tension[ 1 ] != 0.0 ) {
        Output << "tension " << Tail.tension[ 0 ] << ' ' << Tail.tension[ 1 ] << ' ';
    }
    for( int end = 0; end < 2; ++end ) {
        if( false == Tail.rope2set[ end ] ) { continue; }
        auto const &point { Tail.rope2[ end ] };
        Output << "rope2 " << ( end == 0 ? "start" : "end" ) << ' ' << point.x << ' ' << point.y << ' ' << point.z << ' ';
    }
}
