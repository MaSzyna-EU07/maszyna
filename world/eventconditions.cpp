/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the text of the event conditions: read from the scenery and written back by the editor. kept apart from
// Event.cpp so the unit tests can check that what is written reads back the same

#include "stdafx.h"
#include "world/Event.h"
#include "utilities/Logs.h"

#include <format>

bool
basic_event::is_keyword( std::string const &Token ) {
    // TODO: convert to array lookup if keyword list gets longer
    return Token == "endevent"
        || Token == "randomdelay"
        || Token == "departuredelay";
}

void
basic_event::event_conditions::deserialize( cParser &Input, std::string_view const Eventname ) {
    // przetwarzanie warunków, wspólne dla Multiple i UpdateValues

    std::string token;
    while( true == Input.getTokens()
        && false == (token = Input.peek()).empty()
        && false == basic_event::is_keyword(token) ) {

        if( token == "trackoccupied" ) {
            flags |= flags::track_busy;
        }
        else if( token == "trackfree" ) {
            flags |= flags::track_free;
        }
        else if( token == "propability" || token == "probability") { //remove propability in few years after changing old scenery scripts 01.2021
            flags |= flags::probability;
            Input.getTokens();
            Input >> probability;
        }
        else if( token == "memcompare" ) {
            Input.getTokens( 1, false ); // case sensitive
            if( Input.peek() != "*" ) //"*" - nie brac command pod uwage
            { // zapamiętanie łańcucha do porównania
                Input >> memcompare_text;
                flags |= flags::text;
            }
            Input.getTokens();
            if( Input.peek() != "*" ) //"*" - nie brac val1 pod uwage
            {
                Input >> memcompare_value1;
                flags |= flags::value1;
            }
            Input.getTokens();
            if( Input.peek() != "*" ) //"*" - nie brac val2 pod uwage
            {
                Input >> memcompare_value2;
                flags |= flags::value2;
            }
        }
        else if( token == "memcompareex" ) {
            memcompare_pass = comparison_pass_from_string( Input.getToken<std::string>() );
            Input.getTokens();
            if( Input.peek() != "*" ) //"*" - nie brac pod uwage
            { // two tokens, operator followed by comparison value
                std::string operatorstring;
                Input >> operatorstring;
                memcompare_text_operator = comparison_operator_from_string( operatorstring );
                Input.getTokens( 1, false ); // case sensitive
                Input >> memcompare_text;
                flags |= flags::text;
            }
            Input.getTokens();
            if( Input.peek() != "*" ) //"*" - nie brac pod uwage
            { // two tokens, operator followed by comparison value
                std::string operatorstring;
                Input >> operatorstring;
                memcompare_value1_operator = comparison_operator_from_string( operatorstring );
                Input.getTokens();
                Input >> memcompare_value1;
                flags |= flags::value1;
            }
            Input.getTokens();
            if( Input.peek() != "*" ) //"*" - nie brac pod uwage
            { // two tokens, operator followed by comparison value
                std::string operatorstring;
                Input >> operatorstring;
                memcompare_value2_operator = comparison_operator_from_string( operatorstring );
                Input.getTokens();
                Input >> memcompare_value2;
                flags |= flags::value2;
            }
        }
        else {
            // e.g. a name written after trackfree: the track test takes the targets of the event, never a name of its own
            ErrorLog( std::format( "Bad event: \"{}\" has an unknown word \"{}\" in its condition, it's skipped", Eventname, token ) );
        }
    }
}

// sends basic content of the class in legacy (text) format to provided stream
void
basic_event::event_conditions::export_as_text( std::ostream &Output ) const {

    if( flags != 0 ) {
        Output << "condition ";
        if( ( flags & flags::track_busy ) != 0 ) {
            Output << "trackoccupied ";
        }
        if( ( flags & flags::track_free ) != 0 ) {
            Output << "trackfree ";
        }
        if( ( flags & flags::probability ) != 0 ) {
            Output
                << "probability "
                << probability << ' ';
        }
        if( ( flags & ( flags::text | flags::value1 | flags::value2 ) ) != 0 ) {
            // NOTE: export doesn't preserve original memcompare condition, these are all upgraded to memcompareex format for simplicity
            // memcompareex <pass> then for the text and both values either * or the operator followed by the compared value,
            // the order deserialize() reads them in. a legacy memcompare is the same as memcompareex all with all operators ==
            Output
                << "memcompareex "
                << to_string( memcompare_pass ) << ' '
                << ( ( flags & flags::text )   == 0 ? "*" : to_string( memcompare_text_operator )   + ' ' + memcompare_text ) << ' '
                << ( ( flags & flags::value1 ) == 0 ? "*" : to_string( memcompare_value1_operator ) + ' ' + std::to_string( memcompare_value1 ) ) << ' '
                << ( ( flags & flags::value2 ) == 0 ? "*" : to_string( memcompare_value2_operator ) + ' ' + std::to_string( memcompare_value2 ) ) << ' ';
        }
    }
}
