/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "world/catenary_catalog.h"
#include "world/catenary_cantilever.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace catenary {

bool catalog_family::fits( bool const Curve, bool const Toward, int const Card ) const {
    if( curve >= 0 && ( curve == 1 ) != Curve ) { return false; }
    if( toward == 0 || toward == 1 ) { return ( toward == 1 ) == Toward; }
    if( toward == 2 ) { return ( Card % 2 == 1 ) == Toward; }
    return true;
}

bool structure_catalog::load( std::string const &Filename, std::vector<std::string> &Errors ) {
    std::ifstream file { Filename };
    if( false == file.is_open() ) { return false; }
    std::vector<std::string> lines;
    std::string line;
    while( std::getline( file, line ) ) {
        if( false == line.empty() && line.back() == '\r' ) { line.pop_back(); }
        lines.push_back( line );
    }
    parse( lines, Filename, Errors );
    return true;
}

namespace {

std::string trimmed( std::string const &Text ) {
    auto const first { Text.find_first_not_of( " \t" ) };
    if( first == std::string::npos ) { return {}; }
    auto const last { Text.find_last_not_of( " \t" ) };
    return Text.substr( first, last - first + 1 );
}

// karty: "1110-1119" (zakres) albo "0111"
bool cards_of( std::string const &Word, std::vector<int> &Cards ) {
    auto const dash { Word.find( '-' ) };
    try {
        if( dash == std::string::npos ) { Cards.push_back( std::stoi( Word ) ); return true; }
        auto const from { std::stoi( Word.substr( 0, dash ) ) }, to { std::stoi( Word.substr( dash + 1 ) ) };
        if( to < from || to - from > 100 ) { return false; }
        for( int card = from; card <= to; ++card ) { Cards.push_back( card ); }
        return true;
    }
    catch( ... ) { return false; }
}

} // namespace

void structure_catalog::parse( std::vector<std::string> const &Lines, std::string const &Source, std::vector<std::string> &Errors ) {
    // do którego wpisu dopisuje się "text"
    std::string *text { nullptr };
    catalog_group *lastgroup { nullptr };
    for( std::size_t number = 0; number < Lines.size(); ++number ) {
        auto line { Lines[ number ] };
        if( auto const comment { line.find( "//" ) }; comment != std::string::npos ) { line.erase( comment ); }
        line = trimmed( line );
        if( line.empty() ) { continue; }
        auto const report = [ & ]( std::string const &Reason ) {
            Errors.push_back( "Catenary: file \"" + Source + "\" line " + std::to_string( number + 1 ) + ": " + Reason );
        };
        // słowa przed " : " i tytuł za nim
        std::string head { line }, title;
        if( auto const colon { line.find( " : " ) }; colon != std::string::npos ) {
            head = line.substr( 0, colon );
            title = trimmed( line.substr( colon + 3 ) );
        }
        std::istringstream stream { head };
        std::vector<std::string> words;
        for( std::string word; stream >> word; ) { words.push_back( word ); }
        auto const &keyword { words.front() };
        if( keyword == "text" ) {
            auto const body { trimmed( line.substr( 4 ) ) };
            if( text == nullptr ) { report( "\"text\" before any entry" ); continue; }
            *text += ( text->empty() ? "" : " " ) + body;
            continue;
        }
        if( keyword == "exec" ) {
            if( lastgroup == nullptr || words.size() < 2 ) { report( "\"exec\" needs a number and a group before it" ); continue; }
            try { lastgroup->executions.emplace_back( std::stoi( words[ 1 ] ), title ); }
            catch( ... ) { report( "bad execution number" ); }
            continue;
        }
        if( words.size() < 2 ) { report( "\"" + keyword + "\" without an id" ); continue; }
        auto const &id { words[ 1 ] };
        if( keyword == "category" ) {
            m_categories.push_back( { id, title, {} } );
            text = &m_categories.back().text;
            lastgroup = nullptr;
            continue;
        }
        if( keyword == "group" ) {
            if( words.size() < 3 ) { report( "group \"" + id + "\" needs its category" ); continue; }
            catalog_group group;
            group.id = id;
            group.category = words[ 2 ];
            group.title = title;
            for( std::size_t k = 3; k + 1 < words.size(); k += 2 ) {
                if( words[ k ] == "execution" ) { group.execution_user = ( words[ k + 1 ] != "reach" ); }
                else { report( "group \"" + id + "\": unknown keyword \"" + words[ k ] + "\"" ); }
            }
            if( std::none_of( m_categories.begin(), m_categories.end(), [ & ]( catalog_category const &C ) { return C.id == group.category; } ) ) {
                report( "group \"" + id + "\" is in category \"" + group.category + "\" which isn't defined before it" );
            }
            m_groups.push_back( group );
            lastgroup = &m_groups.back();
            text = &lastgroup->text;
            continue;
        }
        if( keyword == "family" ) {
            if( words.size() < 3 ) { report( "family \"" + id + "\" needs its group" ); continue; }
            catalog_family family;
            family.id = id;
            family.group = words[ 2 ];
            family.title = title;
            auto good { true };
            for( std::size_t k = 3; k < words.size() && good; ) {
                auto const &key { words[ k++ ] };
                if( key == "cards" ) {
                    while( k < words.size() && false == words[ k ].empty() && std::isdigit( static_cast<unsigned char>( words[ k ][ 0 ] ) ) ) {
                        if( false == cards_of( words[ k++ ], family.cards ) ) { report( "family \"" + id + "\": bad card \"" + words[ k - 1 ] + "\"" ); good = false; }
                    }
                }
                else if( key == "curve" && k < words.size() ) { family.curve = ( words[ k++ ] == "yes" ? 1 : 0 ); }
                else if( key == "toward" && k < words.size() ) { auto const &value { words[ k++ ] }; family.toward = ( value == "yes" ? 1 : value == "no" ? 0 : 2 ); }
                else if( key == "image" && k < words.size() ) { family.image = words[ k++ ]; }
                else { report( "family \"" + id + "\": unknown keyword \"" + key + "\"" ); good = false; }
            }
            if( false == good ) { text = nullptr; continue; }
            if( family.cards.empty() ) { report( "family \"" + id + "\" has no cards" ); text = nullptr; continue; }
            if( group( family.group ) == nullptr ) { report( "family \"" + id + "\" is in group \"" + family.group + "\" which isn't defined before it" ); }
            m_families.push_back( family );
            text = &m_families.back().text;
            continue;
        }
        if( keyword == "polegroup" ) {
            catalog_polegroup polegroup;
            polegroup.id = id;
            polegroup.title = title;
            for( std::size_t k = 2; k < words.size(); ) {
                auto const &key { words[ k++ ] };
                if( key == "foundation" && k < words.size() ) { polegroup.foundation = words[ k++ ]; }
                else if( key == "site" && k < words.size() ) { polegroup.site = words[ k++ ]; }
                else if( key == "no_switch" ) { polegroup.no_switch = true; }
                else if( key == "poles" ) { while( k < words.size() ) { polegroup.poles.push_back( words[ k++ ] ); } }
                else { report( "polegroup \"" + id + "\": unknown keyword \"" + key + "\"" ); break; }
            }
            m_polegroups.push_back( polegroup );
            text = &m_polegroups.back().text;
            lastgroup = nullptr;
            continue;
        }
        report( "unknown keyword \"" + keyword + "\"" );
        text = nullptr;
    }
}

catalog_group const *structure_catalog::group( std::string const &Id ) const {
    for( auto const &group : m_groups ) { if( group.id == Id ) { return &group; } }
    return nullptr;
}

catalog_family const *structure_catalog::family( std::string const &Id ) const {
    for( auto const &family : m_families ) { if( family.id == Id ) { return &family; } }
    return nullptr;
}

catalog_family const *structure_catalog::family_of( int const Card ) const {
    for( auto const &family : m_families ) {
        if( std::find( family.cards.begin(), family.cards.end(), Card ) != family.cards.end() ) { return &family; }
    }
    return nullptr;
}

catalog_polegroup const *structure_catalog::polegroup_of( std::string const &Pole ) const {
    for( auto const &polegroup : m_polegroups ) {
        if( std::find( polegroup.poles.begin(), polegroup.poles.end(), Pole ) != polegroup.poles.end() ) { return &polegroup; }
    }
    return nullptr;
}

catalog_pick pick_card( structure_catalog const &Catalog, cantilever_catalog const &Cantilevers, std::string const &Group, std::string const &Family,
    int const Card, bool const Curve, bool const Toward, int const Execution, double const Reach ) {

    catalog_pick result;
    auto const *group { Catalog.group( Group ) };
    if( group == nullptr ) { result.problem = "no group \"" + Group + "\" in the catalogue"; return result; }
    // pasmo A: od dolnej granicy włącznie do górnej wyłącznie (jak w kartach: 1,45-1,75 i 1,75-2,05 nie zachodzą na siebie)
    auto const in_band = [ & ]( cantilever_scheme const &Scheme ) {
        // karta z jednym A (peronowe: 4,70 / 5,00 / 5,30) - A w odległości do 0,15 m
        if( Scheme.reach_max - Scheme.reach_min < 1e-6 ) { return std::abs( Reach - Scheme.reach_min ) <= 0.15; }
        return Reach >= Scheme.reach_min && Reach < Scheme.reach_max;
    };
    auto const within = [ & ]( catalog_family const &Candidate, catalog_pick &Pick ) {
        for( auto const card : Candidate.cards ) {
            if( Card > 0 && card != Card ) { continue; }
            if( Family.empty() && Card <= 0 && false == Candidate.fits( Curve, Toward, card ) ) { continue; }
            cantilever_scheme const *other { nullptr };
            for( auto const &entry : Cantilevers.schemes() ) {
                if( entry.first.first != card || false == in_band( entry.second ) ) { continue; }
                if( false == group->execution_user || entry.second.execution == Execution ) {
                    Pick.family = &Candidate;
                    Pick.scheme = &entry.second;
                    return true;
                }
                if( other == nullptr ) { other = &entry.second; }
            }
            if( other != nullptr ) {
                Pick.family = &Candidate;
                Pick.scheme = other;
                Pick.execution_changed = true;
                return true;
            }
        }
        return false;
    };
    // rodziny grupy: najpierw te, które dają wybrane wykonanie
    catalog_pick fallback;
    auto anyfamily { false };
    for( auto const &family : Catalog.families() ) {
        if( family.group != Group ) { continue; }
        if( false == Family.empty() && family.id != Family ) { continue; }
        anyfamily = true;
        catalog_pick pick;
        if( false == within( family, pick ) ) { continue; }
        if( false == pick.execution_changed ) { return pick; }
        if( fallback.scheme == nullptr ) { fallback = pick; }
    }
    if( fallback.scheme != nullptr ) { return fallback; }
    if( false == anyfamily ) { result.problem = "no family of the group fits"; }
    else {
        char text[ 32 ];
        std::snprintf( text, sizeof( text ), "%.2f", Reach );
        result.problem = std::string( "no card of the group covers A = " ) + text + " m" + ( Family.empty() ? std::string( Curve ? " on a curve" : " on a straight" ) + ( Toward ? ", wires towards the pole" : ", wires away from the pole" ) : std::string() );
    }
    return result;
}

} // namespace catenary
