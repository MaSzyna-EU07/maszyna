/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "world/catenary_tensioner.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace catenary {

namespace {

double const kPi { 3.14159265358979323846 };

std::vector<std::string> words_of( std::string Line ) {
    if( auto const comment { Line.find( "//" ) }; comment != std::string::npos ) { Line.erase( comment ); }
    std::istringstream stream { Line };
    std::vector<std::string> words;
    std::string word;
    while( stream >> word ) { words.push_back( word ); }
    return words;
}

bool number_of( std::string const &Word, double &Value ) {
    try {
        std::size_t used { 0 };
        Value = std::stod( Word, &used );
        return used == Word.size();
    }
    catch( ... ) { return false; }
}

// położenie z interpolacją liniową w tablicy rosnącej; Index i Fraction między Index a Index + 1. false poza zakresem
bool bracket( std::vector<double> const &Values, double const Value, std::size_t &Index, double &Fraction ) {
    if( Values.size() < 2 ) { Index = 0; Fraction = 0.0; return Values.size() == 1 && Value == Values.front(); }
    auto inside { true };
    auto value { Value };
    if( value < Values.front() ) { value = Values.front(); inside = false; }
    if( value > Values.back() ) { value = Values.back(); inside = false; }
    Index = 0;
    while( Index + 2 < Values.size() && value > Values[ Index + 1 ] ) { ++Index; }
    auto const span { Values[ Index + 1 ] - Values[ Index ] };
    Fraction = span > 0.0 ? ( value - Values[ Index ] ) / span : 0.0;
    return inside;
}

struct vec { double x { 0.0 }, y { 0.0 }, z { 0.0 }; };
vec operator+( vec const &A, vec const &B ) { return { A.x + B.x, A.y + B.y, A.z + B.z }; }
vec operator-( vec const &A, vec const &B ) { return { A.x - B.x, A.y - B.y, A.z - B.z }; }
vec operator*( vec const &A, double const B ) { return { A.x * B, A.y * B, A.z * B }; }
double dot( vec const &A, vec const &B ) { return A.x * B.x + A.y * B.y + A.z * B.z; }
vec cross( vec const &A, vec const &B ) { return { A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x }; }
double length( vec const &A ) { return std::sqrt( dot( A, A ) ); }
vec unit( vec const &A ) { auto const l { length( A ) }; return l > 1e-12 ? A * ( 1.0 / l ) : vec { 1.0, 0.0, 0.0 }; }

// obrót: kolumny to osie części w układzie urządzenia
void set_rotation( tensioner_placement &Placement, vec const &X, vec const &Y, vec const &Z ) {
    vec const axes[ 3 ] { X, Y, Z };
    for( int row = 0; row < 3; ++row ) {
        for( int column = 0; column < 3; ++column ) {
            auto const &axis { axes[ column ] };
            Placement.rotation[ row * 3 + column ] = ( row == 0 ? axis.x : row == 1 ? axis.y : axis.z );
        }
    }
}

// oś Y części możliwie wzdłuż Y urządzenia, prostopadła do Direction
vec side_of( vec const &Direction ) {
    vec const y { 0.0, 1.0, 0.0 };
    auto const side { y - Direction * dot( y, Direction ) };
    return length( side ) > 1e-9 ? unit( side ) : vec { 0.0, 0.0, 1.0 };
}

tensioner_placement placed( std::string const &Role, std::string const &Part, vec const &At ) {
    tensioner_placement placement;
    placement.role = Role;
    placement.part = Part;
    placement.position[ 0 ] = At.x;
    placement.position[ 1 ] = At.y;
    placement.position[ 2 ] = At.z;
    return placement;
}

// część liniowa: biegnie w -X części (modele Jana: 4440 z hole_1 na -X), więc -X części w kierunku Direction
tensioner_placement placed_along( std::string const &Role, std::string const &Part, vec const &At, vec const &Direction ) {
    auto placement { placed( Role, Part, At ) };
    auto const x { unit( Direction ) * -1.0 };
    auto const y { side_of( x ) * -1.0 };
    set_rotation( placement, x, y, cross( x, y ) );
    return placement;
}

// łańcuch części liniowych od From w kierunku Direction; zwraca koniec
vec place_chain( std::vector<tensioner_link> const &Links, std::string const &Role, vec From, vec const &Direction, tensioner_result &Result ) {
    auto const step { unit( Direction ) };
    int index { 0 };
    for( auto const &link : Links ) {
        // "~część": część odwrócona - jej zero (pin_a) na końcu tego odcinka, biegnie z powrotem (uchwyt 4440 przy rolce ruchomej)
        auto const reversed { false == link.name.empty() && link.name[ 0 ] == '~' };
        auto const name { reversed ? link.name.substr( 1 ) : link.name };
        auto const end { From + step * link.length };
        auto placement { placed_along( Role + "." + std::to_string( ++index ), name, reversed ? end : From, reversed ? step * -1.0 : step ) };
        if( link.roll != 0.0 ) {
            // obrót wokół osi części (kolumna X): Y i Z obracają się w swojej płaszczyźnie
            auto const angle { link.roll * kPi / 180.0 };
            auto const c { std::cos( angle ) }, s { std::sin( angle ) };
            for( int r = 0; r < 3; ++r ) {
                auto const y { placement.rotation[ r * 3 + 1 ] }, z { placement.rotation[ r * 3 + 2 ] };
                placement.rotation[ r * 3 + 1 ] = c * y + s * z;
                placement.rotation[ r * 3 + 2 ] = -s * y + c * z;
            }
        }
        Result.parts.push_back( placement );
        From = end;
    }
    return From;
}

} // namespace

void tensioner_table::formula_values( double const Temperature, double const Distance, double &B, double &S ) const {
    auto const shift { Distance * alpha * ( Temperature - tn ) };
    B = bn - shift;
    S = sn - 4.0 * shift;
}

int tensioner_color( std::string const &Name ) {
    if( Name == "zolty" || Name == "1" ) { return 1; }
    if( Name == "szary" || Name == "2" ) { return 2; }
    if( Name == "zolty_pedzel" || Name == "3" ) { return 3; }
    if( Name == "szary_brzydki" || Name == "4" ) { return 4; }
    return 0;
}

bool tensioner_table::lookup( double const Temperature, double const Distance, double &B, double &S ) const {
    if( formula ) {
        formula_values( Temperature, Distance, B, S );
        return true;
    }
    B = 0.0;
    S = 0.0;
    if( temperatures.empty() || distances.empty() ) { return false; }
    std::size_t row { 0 }, column { 0 };
    double down { 0.0 }, across { 0.0 };
    auto const inside_d { bracket( distances, Distance, row, down ) };
    auto const inside_t { bracket( temperatures, Temperature, column, across ) };
    auto const value = [ & ]( std::vector<std::vector<double>> const &Table ) {
        auto const at = [ & ]( std::size_t const R, std::size_t const C ) {
            return Table[ std::min( R, Table.size() - 1 ) ][ std::min( C, Table[ 0 ].size() - 1 ) ];
        };
        auto const top { at( row, column ) * ( 1.0 - across ) + at( row, column + 1 ) * across };
        auto const bottom { at( row + 1, column ) * ( 1.0 - across ) + at( row + 1, column + 1 ) * across };
        return top * ( 1.0 - down ) + bottom * down;
    };
    B = value( b );
    S = value( s );
    return inside_d && inside_t;
}

tensioner_type const *tensioner_device::type( int const Code ) const {
    for( auto const &entry : types ) { if( entry.code == Code ) { return &entry; } }
    return nullptr;
}

bool tensioner_catalog::load( std::string const &Filename, std::vector<std::string> &Errors ) {
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

void tensioner_catalog::parse( std::vector<std::string> const &Lines, std::string const &Source, std::vector<std::string> &Errors ) {
    tensioner_table *table { nullptr };
    tensioner_device *device { nullptr };
    int number { 0 };
    auto const report = [ & ]( std::string const &Text ) { Errors.push_back( Source + ":" + std::to_string( number ) + ": " + Text ); };
    // pary "część długość" od pozycji From
    auto const links_of = [ & ]( std::vector<std::string> const &Words, std::size_t const From, std::vector<tensioner_link> &Links ) {
        for( auto k { From }; k + 1 < Words.size(); k += 2 ) {
            tensioner_link link;
            link.name = Words[ k ];
            if( auto const at { link.name.find( '@' ) }; at != std::string::npos ) {
                if( false == number_of( link.name.substr( at + 1 ), link.roll ) ) { report( "bad roll in \"" + link.name + "\"" ); }
                link.name.erase( at );
            }
            if( false == number_of( Words[ k + 1 ], link.length ) ) { report( "bad length \"" + Words[ k + 1 ] + "\"" ); return; }
            Links.push_back( link );
        }
        if( ( Words.size() - From ) % 2 != 0 ) { report( "\"" + Words[ 0 ] + "\": a part without a length" ); }
    };
    for( auto const &text : Lines ) {
        ++number;
        auto const words { words_of( text ) };
        if( words.empty() ) { continue; }
        auto const &keyword { words[ 0 ] };
        // tablica montażowa
        if( table != nullptr ) {
            if( keyword == "endtable" ) {
                for( auto const &row : table->b ) {
                    if( row.size() != table->temperatures.size() ) { report( "table " + table->name + ": a row of b has a wrong number of values" ); }
                }
                for( auto const &row : table->s ) {
                    if( row.size() != table->temperatures.size() ) { report( "table " + table->name + ": a row of s has a wrong number of values" ); }
                }
                table = nullptr;
                continue;
            }
            if( keyword == "temperatures" ) {
                table->temperatures.clear();
                for( std::size_t k = 1; k < words.size(); ++k ) {
                    double value;
                    if( number_of( words[ k ], value ) ) { table->temperatures.push_back( value ); }
                    else { report( "bad temperature \"" + words[ k ] + "\"" ); }
                }
                continue;
            }
            if( keyword == "row" && words.size() > 2 ) {
                double distance;
                if( false == number_of( words[ 1 ], distance ) ) { report( "bad distance \"" + words[ 1 ] + "\"" ); continue; }
                std::vector<double> *target { nullptr };
                table->distances.push_back( distance );
                table->b.emplace_back();
                table->s.emplace_back();
                for( std::size_t k = 2; k < words.size(); ++k ) {
                    if( words[ k ] == "b" ) { target = &table->b.back(); continue; }
                    if( words[ k ] == "s" ) { target = &table->s.back(); continue; }
                    double value;
                    if( target == nullptr || false == number_of( words[ k ], value ) ) { report( "bad value \"" + words[ k ] + "\"" ); continue; }
                    target->push_back( value );
                }
                continue;
            }
            report( "unknown keyword \"" + keyword + "\" in a table" );
            continue;
        }
        // urządzenie
        if( device != nullptr ) {
            if( keyword == "enddevice" ) { device = nullptr; continue; }
            if( keyword == "mount" ) { links_of( words, 1, device->mount ); continue; }
            if( keyword == "holder" ) { links_of( words, 1, device->holder ); continue; }
            if( keyword == "hanger" ) { links_of( words, 1, device->hanger ); continue; }
            if( keyword == "messenger" ) { links_of( words, 1, device->messenger ); continue; }
            if( keyword == "contact" ) { links_of( words, 1, device->contact ); continue; }
            if( keyword == "cascade" ) { links_of( words, 1, device->cascade ); continue; }
            if( keyword == "series" && words.size() >= 4 ) {
                // series <mała rolka> <odstęp osi w bloku> <zaczep liny od osi dużej stałej> [holder <uchwyt stały>]
                device->small_pulley = words[ 1 ];
                if( false == number_of( words[ 2 ], device->small_spacing ) || false == number_of( words[ 3 ], device->dead_end ) ) { report( "series: bad numbers" ); }
                if( words.size() >= 6 && words[ 4 ] == "holder" ) { device->fixed_holder = words[ 5 ]; }
                continue;
            }
            if( keyword == "messenger_each" ) { links_of( words, 1, device->messenger_each ); continue; }
            if( keyword == "contact_each" ) { links_of( words, 1, device->contact_each ); continue; }
            if( keyword == "messenger_equalizer" && words.size() >= 3 ) {
                // messenger_equalizer <część> <długość> [spread <rozstaw lin>] [z]
                device->messenger_equalizer.name = words[ 1 ];
                if( false == number_of( words[ 2 ], device->messenger_equalizer.length ) ) { report( "bad length \"" + words[ 2 ] + "\"" ); }
                for( std::size_t k = 3; k < words.size(); ++k ) {
                    if( words[ k ] == "spread" && k + 1 < words.size() ) { number_of( words[ ++k ], device->messenger_spread ); }
                    else if( words[ k ] == "z" ) { device->messenger_equalizer_z = true; }
                }
                continue;
            }
            if( keyword == "only" && words.size() >= 2 ) {
                if( words[ 1 ] == "messenger" || words[ 1 ] == "contact" ) { device->only = words[ 1 ]; }
                else { report( "only: messenger or contact" ); }
                continue;
            }
            if( keyword == "split" && words.size() >= 3 ) { device->split_messenger = words[ 1 ]; device->split_contact = words[ 2 ]; continue; }
            if( keyword == "frame" && words.size() >= 5 ) {
                device->frame = words[ 1 ];
                double side { -1.0 };
                if( false == number_of( words[ 2 ], device->frame_length ) || false == number_of( words[ 3 ], device->frame_y ) || false == number_of( words[ 4 ], side ) ) { report( "frame: bad numbers" ); }
                device->frame_side = side < 0.0 ? -1 : 1;
                device->frame_owner = words.size() >= 6 && words[ 5 ] == "owner";
                continue;
            }
            if( keyword == "messenger2" ) { links_of( words, 1, device->messenger2 ); continue; }
            if( keyword == "guide" && words.size() >= 3 ) {
                device->guide = words[ 1 ];
                if( false == number_of( words[ 2 ], device->guide_length ) ) { report( "bad guide length \"" + words[ 2 ] + "\"" ); }
                for( std::size_t k = 3; k + 1 < words.size(); k += 2 ) {
                    if( words[ k ] == "mount" ) { number_of( words[ k + 1 ], device->guide_mount ); }
                    else if( words[ k ] == "bottom" ) { number_of( words[ k + 1 ], device->guide_bottom ); }
                    else { report( "unknown guide keyword \"" + words[ k ] + "\"" ); }
                }
                continue;
            }
            if( keyword == "brackets" && words.size() >= 2 ) {
                tensioner_device::bracket_set set;
                std::size_t k { 1 };
                for( ; k < words.size() && words[ k ] != "top" && words[ k ] != "bottom" && words[ k ] != "hole" && words[ k ] != "bolt"; ++k ) {
                    set.poles.push_back( words[ k ] );
                }
                for( ; k < words.size(); ) {
                    auto const &key { words[ k ] };
                    if( ( key == "top" || key == "bottom" ) && k + 2 < words.size() ) {
                        ( key == "top" ? set.top_front : set.bottom_front ) = words[ k + 1 ];
                        ( key == "top" ? set.top_back : set.bottom_back ) = words[ k + 2 ];
                        k += 3;
                    }
                    else if( key == "hole" && k + 2 < words.size() ) { number_of( words[ k + 1 ], set.hole_out ); number_of( words[ k + 2 ], set.hole_drop ); k += 3; }
                    else if( key == "bolt" && k + 1 < words.size() ) { number_of( words[ k + 1 ], set.bolt ); k += 2; }
                    else { report( "unknown brackets keyword \"" + key + "\"" ); break; }
                }
                device->brackets.push_back( set );
                continue;
            }
            if( keyword == "pivot" && words.size() >= 3 ) {
                number_of( words[ 1 ], device->pivot_top );
                number_of( words[ 2 ], device->pivot_bottom );
                if( words.size() >= 5 && words[ 3 ] == "rise" ) { number_of( words[ 4 ], device->mount_rise ); }
                continue;
            }
            if( keyword == "ring" && words.size() >= 3 ) {
                device->ring = words[ 1 ];
                if( false == number_of( words[ 2 ], device->ring_arm ) ) { report( "bad ring arm \"" + words[ 2 ] + "\"" ); }
                for( std::size_t k = 3; k + 1 < words.size(); k += 2 ) {
                    if( words[ k ] == "turn" ) { number_of( words[ k + 1 ], device->ring_turn ); }
                    else if( words[ k ] == "at" ) { number_of( words[ k + 1 ], device->ring_at ); }
                    else { report( "unknown ring keyword \"" + words[ k ] + "\"" ); }
                }
                continue;
            }
            if( keyword == "messenger_old" ) { links_of( words, 1, device->messenger_old ); continue; }
            if( keyword == "contact_old" ) { links_of( words, 1, device->contact_old ); continue; }
            if( ( keyword == "messenger_clamp" || keyword == "contact_clamp" || keyword == "contact_clamp_old" || keyword == "equalizer" ) && words.size() >= 3 ) {
                auto &link { keyword == "messenger_clamp" ? device->messenger_clamp : keyword == "contact_clamp" ? device->contact_clamp : keyword == "contact_clamp_old" ? device->contact_clamp_old : device->equalizer };
                link.name = words[ 1 ];
                if( false == number_of( words[ 2 ], link.length ) ) { report( "bad length \"" + words[ 2 ] + "\"" ); }
                if( keyword == "equalizer" && words.size() >= 5 && words[ 3 ] == "spread" ) { number_of( words[ 4 ], device->equalizer_spread ); }
                continue;
            }
            if( keyword == "safety" && words.size() >= 5 ) {
                // safety <ramię przy rolce stałej> <ramię przy ruchomej> <rozstaw otworów> <odsunięcie w bok>
                device->safety_fixed = words[ 1 ];
                device->safety_moving = words[ 2 ];
                number_of( words[ 3 ], device->safety_arm );
                number_of( words[ 4 ], device->safety_offset );
                continue;
            }
            if( keyword == "safety_series" && words.size() >= 7 ) {
                device->series_arm_fixed = words[ 1 ];
                device->series_arm_moving = words[ 2 ];
                device->series_bar = words[ 3 ];
                number_of( words[ 4 ], device->series_arm );
                number_of( words[ 5 ], device->series_bar_length );
                number_of( words[ 6 ], device->safety_offset );
                continue;
            }
            if( keyword == "title" && words.size() >= 2 ) {
                device->title.clear();
                for( std::size_t k = 1; k < words.size(); ++k ) { device->title += ( k > 1 ? " " : "" ) + words[ k ]; }
                continue;
            }
            if( keyword == "type" && words.size() >= 2 ) {
                tensioner_type type;
                double code;
                if( false == number_of( words[ 1 ], code ) ) { report( "bad type \"" + words[ 1 ] + "\"" ); continue; }
                type.code = static_cast<int>( code );
                for( std::size_t k = 2; k < words.size(); ) {
                    auto const &key { words[ k++ ] };
                    if( key == "stack" ) {
                        while( k + 1 < words.size() && m_weights.end() != std::find_if( m_weights.begin(), m_weights.end(), [ & ]( tensioner_weight const &W ) { return W.name == words[ k ]; } ) ) {
                            double count;
                            if( false == number_of( words[ k + 1 ], count ) ) { report( "bad count \"" + words[ k + 1 ] + "\"" ); break; }
                            type.stack.emplace_back( words[ k ], static_cast<int>( count ) );
                            k += 2;
                        }
                    }
                    else if( key == "lever" && k + 2 < words.size() ) {
                        type.messenger_hole = words[ k++ ];
                        type.contact_hole = words[ k++ ];
                        type.axis_hole = words[ k++ ];
                    }
                    else if( key == "second" && k + 2 < words.size() ) {
                        type.second_messenger_hole = words[ k++ ];
                        type.second_contact_hole = words[ k++ ];
                        type.second_axis_hole = words[ k++ ];
                    }
                    else if( key == "rod" && k < words.size() ) { type.rod = words[ k++ ]; }
                    else if( key == "clamp" && k < words.size() ) { type.messenger_clamp = words[ k++ ]; }
                    else { report( "type " + words[ 1 ] + ": unknown keyword \"" + key + "\" (a weight has to be defined before the device)" ); break; }
                }
                device->types.push_back( type );
                continue;
            }
            report( "unknown keyword \"" + keyword + "\" in a device" );
            continue;
        }
        if( keyword == "network" && words.size() >= 4 && words[ 2 ] == "as" ) {
            double code;
            if( false == number_of( words[ 3 ], code ) ) { report( "bad network code \"" + words[ 3 ] + "\"" ); continue; }
            m_networks[ words[ 1 ] ] = static_cast<int>( code );
            continue;
        }
        if( keyword == "table" && words.size() >= 2 ) {
            // "table <nazwa> base <inna> shift_b <m> shift_s <m> ..." - jednowierszowa, wartości innej tablicy przesunięte
            // o stałą (tablice 0011-6/-7/-8 różnią się od 0011-1 tylko położeniem nominalnym)
            tensioner_table entry;
            entry.name = words[ 1 ];
            double shiftb { 0.0 }, shifts { 0.0 };
            auto derived { false };
            for( std::size_t k = 2; k + 1 < words.size(); k += 2 ) {
                if( words[ k ] == "height" ) { number_of( words[ k + 1 ], entry.height ); }
                else if( words[ k ] == "rod" ) { number_of( words[ k + 1 ], entry.rod ); }
                else if( words[ k ] == "shift_b" ) { number_of( words[ k + 1 ], shiftb ); }
                else if( words[ k ] == "shift_s" ) { number_of( words[ k + 1 ], shifts ); }
                else if( words[ k ] == "formula" ) {
                    // "formula <bn> <sn> <tn> <alfa>": cztery liczby
                    if( k + 4 < words.size() && number_of( words[ k + 1 ], entry.bn ) && number_of( words[ k + 2 ], entry.sn )
                     && number_of( words[ k + 3 ], entry.tn ) && number_of( words[ k + 4 ], entry.alpha ) ) { entry.formula = true; }
                    else { report( "table " + entry.name + ": formula needs bn sn tn alpha" ); }
                    k += 3; // pętla doda jeszcze 2
                }
                else if( words[ k ] == "base" ) {
                    derived = true;
                    auto const *base { this->table( words[ k + 1 ] ) };
                    if( base == nullptr ) { report( "table " + entry.name + ": no base table " + words[ k + 1 ] + " before it" ); continue; }
                    entry.temperatures = base->temperatures;
                    entry.distances = base->distances;
                    entry.b = base->b;
                    entry.s = base->s;
                    entry.formula = base->formula;
                    entry.bn = base->bn;
                    entry.sn = base->sn;
                    entry.tn = base->tn;
                    entry.alpha = base->alpha;
                }
                else { report( "table " + entry.name + ": unknown keyword \"" + words[ k ] + "\"" ); }
            }
            if( derived ) {
                for( auto &row : entry.b ) { for( auto &value : row ) { value += shiftb; } }
                for( auto &row : entry.s ) { for( auto &value : row ) { value += shifts; } }
                entry.bn += shiftb;
                entry.sn += shifts;
            }
            m_tables.push_back( entry );
            if( false == derived ) { table = &m_tables.back(); }
            continue;
        }
        if( keyword == "weight" && words.size() >= 2 ) {
            tensioner_weight weight;
            weight.name = words[ 1 ];
            for( std::size_t k = 2; k + 1 < words.size(); k += 2 ) {
                if( words[ k ] == "height" ) { number_of( words[ k + 1 ], weight.height ); }
                else if( words[ k ] == "mass" ) { number_of( words[ k + 1 ], weight.mass ); }
                else if( words[ k ] == "turn" ) { number_of( words[ k + 1 ], weight.turn ); }
                else if( words[ k ] == "model" ) { weight.model = words[ k + 1 ]; }
                else if( words[ k ] == "offset" ) { double value { 0.0 }; number_of( words[ k + 1 ], value ); weight.offset = static_cast<int>( value ); }
                else { report( "weight " + weight.name + ": unknown keyword \"" + words[ k ] + "\"" ); }
            }
            m_weights.push_back( weight );
            continue;
        }
        if( keyword == "pulley" && words.size() >= 2 ) {
            tensioner_pulley pulley;
            pulley.name = words[ 1 ];
            for( std::size_t k = 2; k + 1 < words.size(); k += 2 ) {
                double value { 0.0 };
                number_of( words[ k + 1 ], value );
                if( words[ k ] == "diameter" ) { pulley.diameter = value; }
                else if( words[ k ] == "width" ) { pulley.width = value; }
                else if( words[ k ] == "grooves" ) { pulley.grooves = static_cast<int>( value ); }
                else if( words[ k ] == "sheave" ) { pulley.sheave = words[ k + 1 ]; }
                else { report( "pulley " + pulley.name + ": unknown keyword \"" + words[ k ] + "\"" ); }
            }
            m_pulleys.push_back( pulley );
            continue;
        }
        if( keyword == "lever" && words.size() >= 2 ) {
            tensioner_lever lever;
            lever.name = words[ 1 ];
            for( std::size_t k = 2; k < words.size(); ) {
                if( words[ k ] == "hole" && k + 3 < words.size() ) {
                    double along, aside;
                    if( number_of( words[ k + 2 ], along ) && number_of( words[ k + 3 ], aside ) ) { lever.holes[ words[ k + 1 ] ] = { along, aside }; }
                    else { report( "lever " + lever.name + ": bad hole \"" + words[ k + 1 ] + "\"" ); }
                    k += 4;
                }
                else { report( "lever " + lever.name + ": unknown keyword \"" + words[ k ] + "\"" ); break; }
            }
            m_levers.push_back( lever );
            continue;
        }
        if( keyword == "part" && words.size() >= 4 && words[ 2 ] == "length" ) {
            double value;
            if( number_of( words[ 3 ], value ) ) { m_lengths[ words[ 1 ] ] = value; }
            else { report( "part " + words[ 1 ] + ": bad length" ); }
            continue;
        }
        if( keyword == "device" && words.size() >= 2 ) {
            m_devices.emplace_back();
            device = &m_devices.back();
            device->name = words[ 1 ];
            for( std::size_t k = 2; k + 1 < words.size(); k += 2 ) {
                auto const &key { words[ k ] };
                auto const &value { words[ k + 1 ] };
                if( key == "table" ) { device->table = value; }
                else if( key == "pulley" ) { device->pulley = value; }
                else if( key == "lever" ) { device->lever = value; }
                else if( key == "lever2" ) { device->lever2 = value; }
                else if( key == "rod" ) { device->rod = value; }
                else if( key == "kind" ) {
                    if( value == "fixed" ) { device->fixed = true; }
                    else if( value == "split" ) { device->split = true; }
                    else if( value != "weights" ) { report( "device " + device->name + ": unknown kind \"" + value + "\"" ); }
                }
                else if( key == "rope" ) { device->rope = value; }
                else if( key == "rope_diameter" ) { number_of( value, device->rope_diameter ); }
                else if( key == "rope_eye" && k + 2 < words.size() ) { device->rope_eye = value; number_of( words[ k + 2 ], device->rope_eye_length ); ++k; }
                else if( key == "rope_spacer" && k + 2 < words.size() ) { device->rope_spacer = value; number_of( words[ k + 2 ], device->rope_spacer_length ); ++k; }
                else { report( "device " + device->name + ": unknown keyword \"" + key + "\"" ); }
            }
            continue;
        }
        report( "unknown keyword \"" + keyword + "\"" );
    }
    if( table != nullptr ) { report( "table " + table->name + " without endtable" ); }
    if( device != nullptr ) { report( "device " + device->name + " without enddevice" ); }
}

tensioner_table const *tensioner_catalog::table( std::string const &Name ) const {
    for( auto const &entry : m_tables ) { if( entry.name == Name ) { return &entry; } }
    return nullptr;
}
tensioner_weight const *tensioner_catalog::weight( std::string const &Name ) const {
    for( auto const &entry : m_weights ) { if( entry.name == Name ) { return &entry; } }
    return nullptr;
}
tensioner_pulley const *tensioner_catalog::pulley( std::string const &Name ) const {
    for( auto const &entry : m_pulleys ) { if( entry.name == Name ) { return &entry; } }
    return nullptr;
}
tensioner_device::bracket_set const *tensioner_device::brackets_for( std::string const &Pole ) const {
    bracket_set const *best { nullptr };
    std::size_t length { 0 };
    for( auto const &set : brackets ) {
        for( auto const &pole : set.poles ) {
            if( Pole.rfind( pole, 0 ) == 0 && ( best == nullptr || pole.size() > length ) ) { best = &set; length = pole.size(); }
        }
    }
    return best;
}

tensioner_lever const *tensioner_catalog::lever( std::string const &Name ) const {
    for( auto const &entry : m_levers ) { if( entry.name == Name ) { return &entry; } }
    return nullptr;
}
int tensioner_catalog::code_for( std::string const &Symbol, int const Code ) const {
    auto const found { m_networks.find( Symbol ) };
    return ( found != m_networks.end() ? found->second : Code );
}

tensioner_device const *tensioner_catalog::device( std::string const &Name ) const {
    for( auto const &entry : m_devices ) { if( entry.name == Name ) { return &entry; } }
    return nullptr;
}

namespace {

// kotwienie stałe: przegubowy uchwyt (4430) w parze uchwytów konstrukcji na wysokości liny i drugi na wysokości przewodów,
// łańcuchy do drutów wycelowane w ostatnie podwieszenie. Uchwyty w układzie konstrukcji (structure_angle) jak przy ciężarowym
bool solve_fixed( tensioner_device const &Device, tensioner_type const &Type, tensioner_input const &Input, tensioner_result &Result ) {
    Result.fixed = true;
    vec const out { 1.0, 0.0, 0.0 };
    auto const ca { std::cos( Input.structure_angle ) }, sa { std::sin( Input.structure_angle ) };
    auto const onstructure = [ & ]( vec const &P ) { return vec { ca * P.x - sa * P.y, sa * P.x + ca * P.y, P.z }; };
    auto const fix = [ & ]( std::size_t const From ) {
        if( Input.structure_angle == 0.0 ) { return; }
        for( auto k { From }; k < Result.parts.size(); ++k ) {
            auto &part { Result.parts[ k ] };
            auto const p { onstructure( vec { part.position[ 0 ], part.position[ 1 ], part.position[ 2 ] } ) };
            part.position[ 0 ] = p.x; part.position[ 1 ] = p.y; part.position[ 2 ] = p.z;
            for( int c = 0; c < 3; ++c ) {
                auto const q { onstructure( vec { part.rotation[ c ], part.rotation[ 3 + c ], part.rotation[ 6 + c ] } ) };
                part.rotation[ c ] = q.x; part.rotation[ 3 + c ] = q.y; part.rotation[ 6 + c ] = q.z;
            }
        }
    };
    auto const *brackets { Input.pole.empty() ? nullptr : Device.brackets_for( Input.pole ) };
    auto const pivotx { brackets != nullptr ? brackets->hole_out : 0.0 };
    // wysokości: przewody - podana albo ostatniego podwieszenia; lina - ostatniego podwieszenia albo 1,70 wyżej
    auto const contactheight { Input.height > 0.0 ? Input.height : Input.aimed ? Input.contact_target[ 2 ] : 5.60 };
    auto const messengerheight { Input.rope_height > 0.0 ? Input.rope_height : Input.aimed ? Input.messenger_target[ 2 ] + ( Input.height > 0.0 ? Input.height - Input.contact_target[ 2 ] : 0.0 ) : contactheight + 1.70 };
    Result.rope_height = messengerheight;
    Result.height = contactheight;
    bool warned { false };
    auto const aimed = [ & ]( vec const &From, double const *Target ) {
        if( false == Input.aimed ) { return out; }
        vec const to { Target[ 0 ] - From.x, Target[ 1 ] - From.y, Target[ 2 ] - From.z };
        if( to.x < 1.0 ) {
            if( false == warned ) { Result.warnings.push_back( "the last suspension is behind or too close to the anchor, the wires are pulled level" ); warned = true; }
            return out;
        }
        return unit( to );
    };
    // para uchwytów wokół przegubu na wysokości Height (górny wprost, dolny odwrócony), z tyłem po drugiej stronie konstrukcji
    auto const pair = [ & ]( std::string const &Role, double const Height ) {
        if( brackets == nullptr ) { return; }
        auto const from { Result.parts.size() };
        for( auto const upside : { false, true } ) {
            vec const at { 0.0, 0.0, upside ? Height - Device.pivot_bottom - brackets->hole_drop : Height + Device.pivot_top + brackets->hole_drop };
            auto const name { Role + ( upside ? "_bottom" : "_top" ) };
            if( false == brackets->top_front.empty() ) {
                auto placement { placed( name, brackets->top_front, at ) };
                if( upside ) { set_rotation( placement, vec { -1.0, 0.0, 0.0 }, vec { 0.0, 1.0, 0.0 }, vec { 0.0, 0.0, -1.0 } ); }
                else { set_rotation( placement, vec { -1.0, 0.0, 0.0 }, vec { 0.0, -1.0, 0.0 }, vec { 0.0, 0.0, 1.0 } ); }
                Result.parts.push_back( placement );
            }
            auto const depth { Input.depth ? Input.depth( at.z ) : 0.0 };
            if( false == brackets->top_back.empty() && depth > 0.0 ) {
                auto placement { placed( name + ".back", brackets->top_back, vec { -depth, 0.0, at.z } ) };
                if( upside ) { set_rotation( placement, vec { 1.0, 0.0, 0.0 }, vec { 0.0, -1.0, 0.0 }, vec { 0.0, 0.0, -1.0 } ); }
                Result.parts.push_back( placement );
                std::string const text { "the bracket bolts are shorter than the structure is deep" };
                if( depth > brackets->bolt + 1e-6 && std::find( Result.warnings.begin(), Result.warnings.end(), text ) == Result.warnings.end() ) { Result.warnings.push_back( text ); }
            }
        }
        fix( from );
    };
    // łańcuch od przegubu do drutów: przegub obraca się za drutami, za wyrównywaczem każdy drut osobno
    auto const wires = [ & ]( std::string const &Role, double const Height, double const *Target, std::vector<tensioner_link> const &Links,
                              tensioner_link const &Equalizer, int const Count, double const Spread, std::vector<tensioner_link> const &Each,
                              tensioner_link const &Clamp, bool const Vertical, std::vector<vec> &Ends ) {
        pair( "bracket_" + Role, Height );
        auto const start { onstructure( vec { pivotx, 0.0, Height } ) };
        auto const way { aimed( start, Target ) };
        std::vector<tensioner_link> chain { Device.mount.empty() ? std::vector<tensioner_link> {} : std::vector<tensioner_link> { Device.mount.front() } };
        chain.insert( chain.end(), Links.begin(), Links.end() );
        auto end { place_chain( chain, Role, start, way, Result ) };
        std::vector<vec> starts;
        if( Count >= 2 && false == Equalizer.name.empty() ) {
            // przewody obok siebie, liny jedna nad drugą (wyrównywacz obrócony o 90 stopni wokół łańcucha)
            vec const up { -way.x * way.z, -way.y * way.z, way.x * way.x + way.y * way.y };
            auto const across { Vertical ? unit( up ) : unit( vec { -way.y, way.x, 0.0 } ) };
            auto placement { placed_along( Role + "_equalizer", Equalizer.name, end, way ) };
            if( Vertical ) { set_rotation( placement, way * -1.0, across, cross( way * -1.0, across ) ); }
            Result.parts.push_back( placement );
            end = end + way * Equalizer.length;
            for( int k = 0; k < Count; ++k ) { starts.push_back( end + across * ( ( k - 0.5 * ( Count - 1 ) ) * Spread ) ); }
        }
        else { starts.push_back( end ); }
        for( std::size_t k = 0; k < starts.size(); ++k ) {
            auto const tag { Role + "_each." + std::to_string( k + 1 ) };
            // dwie liny nośne: górna celuje w linę przęsła, dolna o odstęp lin niżej
            auto own { way };
            if( Vertical && starts.size() == 2 && Input.aimed ) {
                double const lower[ 3 ] { Target[ 0 ], Target[ 1 ], Target[ 2 ] - ( k == 0 ? Input.messenger_stack : 0.0 ) };
                own = aimed( starts[ k ], lower );
            }
            auto at { place_chain( Each, tag, starts[ k ], own, Result ) };
            if( false == Clamp.name.empty() ) {
                Result.parts.push_back( placed_along( Role + "_clamp." + std::to_string( k + 1 ), Clamp.name, at, own ) );
                at = at + own * Clamp.length;
            }
            Ends.push_back( at );
        }
    };
    std::vector<vec> ropes, contacts;
    auto messengerclamp { Device.messenger_clamp };
    if( false == Type.messenger_clamp.empty() ) { messengerclamp.name = Type.messenger_clamp; }
    // kotwienie samych przewodów (only contact, 0404): bez łańcucha liny
    if( Device.only != "contact" ) {
        wires( "messenger", messengerheight, Input.messenger_target, Device.messenger, Device.messenger_equalizer, Input.messengers, Device.equalizer_spread,
            Device.messenger_each, messengerclamp, true, ropes );
    }
    auto const &clamp { Input.old_fittings && false == Device.contact_clamp_old.name.empty() ? Device.contact_clamp_old : Device.contact_clamp };
    // kotwienie środkowe: tylko lina kotwienia (only messenger)
    if( Device.only != "messenger" ) {
        wires( "contact", contactheight, Input.contact_target, Device.contact, Device.equalizer, Input.contacts, Device.equalizer_spread, Device.contact_each, clamp, false, contacts );
    }
    auto const copy = []( vec const &P, double *Out ) { Out[ 0 ] = P.x; Out[ 1 ] = P.y; Out[ 2 ] = P.z; };
    // lina nośna przęsła to górna (dolna o odstęp lin niżej, jak w typie sieci)
    if( false == ropes.empty() ) { copy( ropes.back(), Result.messenger_end ); }
    if( ropes.size() >= 2 ) { copy( ropes.front(), Result.messenger2_end ); }
    for( auto const &end : contacts ) { Result.contact_ends.push_back( { end.x, end.y, end.z } ); }
    // obrys dla edytora: od czoła do końca przegubu, na wysokości przewodów
    Result.b = pivotx + ( Device.mount.empty() ? 0.0 : Device.mount.front().length );
    Result.s = contactheight;
    return true;
}

} // namespace

bool solve_tensioner( tensioner_catalog const &Catalog, tensioner_device const &Device, tensioner_input const &Input, tensioner_result &Result ) {
    Result = tensioner_result();
    auto const fail = [ & ]( std::string const &Text ) { Result.warnings.push_back( Text ); return false; };
    auto const *type { Device.type( Input.type ) };
    if( type == nullptr ) { return fail( "device " + Device.name + " has no data for network type " + std::to_string( Input.type ) ); }
    if( Device.fixed ) { return solve_fixed( Device, *type, Input, Result ); }
    if( Device.split ) {
        // kotwienie rozdzielone: urządzenie liny i urządzenie przewodów (albo ich odmiana dla typu sieci: <nazwa>-<cokolwiek>)
        auto const pick = [ & ]( std::string const &Name ) -> tensioner_device const * {
            if( auto const *exact { Catalog.device( Name ) }; exact != nullptr && exact->type( Input.type ) != nullptr ) { return exact; }
            for( auto const &other : Catalog.devices() ) {
                if( other.name.rfind( Name + "-", 0 ) == 0 && other.type( Input.type ) != nullptr ) { return &other; }
            }
            return nullptr;
        };
        auto const *messengerdevice { pick( Device.split_messenger ) }, *contactdevice { pick( Device.split_contact ) };
        if( messengerdevice == nullptr || contactdevice == nullptr ) { return fail( "device " + Device.name + " has no messenger / contact device for network type " + std::to_string( Input.type ) ); }
        tensioner_result rope, wires;
        auto contactinput { Input };
        contactinput.missing = 0;    // brakujące ciężary - ze stosu liny
        if( Input.height > 0.0 ) {
            // wysokość podana dla rolek liny; przewody o tyle niżej, co w tablicach
            auto const *upper { Catalog.table( messengerdevice->table ) }, *lower { Catalog.table( contactdevice->table ) };
            if( upper != nullptr && lower != nullptr ) { contactinput.height = Input.height - ( upper->height - lower->height ); }
        }
        if( false == solve_tensioner( Catalog, *messengerdevice, Input, rope ) ) { Result = rope; return false; }
        if( false == solve_tensioner( Catalog, *contactdevice, contactinput, wires ) ) { Result = wires; return false; }
        Result = rope;
        for( auto part : wires.parts ) { part.role = "contact_device." + part.role; Result.parts.push_back( part ); }
        Result.rope2 = wires.rope;
        Result.contact_ends = wires.contact_ends;
        Result.contact_tension = wires.contact_tension;
        Result.tension = rope.tension + wires.tension;
        Result.nominal_tension = rope.nominal_tension + wires.nominal_tension;
        Result.weights = rope.weights + wires.weights;
        Result.grounded = rope.grounded || wires.grounded;
        for( auto const &warning : wires.warnings ) {
            if( std::find( Result.warnings.begin(), Result.warnings.end(), warning ) == Result.warnings.end() ) { Result.warnings.push_back( warning ); }
        }
        return true;
    }
    auto const *table { Catalog.table( Device.table ) };
    if( table == nullptr ) { return fail( "no table " + Device.table ); }
    auto const *pulley { Catalog.pulley( Device.pulley ) };
    if( pulley == nullptr ) { return fail( "no pulley " + Device.pulley ); }
    // urządzenie tylko liny albo tylko przewodów (kotwienie rozdzielone): bez dźwigni, wszystkie "otwory" w osi na końcu uchwytu
    auto const only { false == Device.only.empty() };
    auto const onlymessenger { Device.only == "messenger" };
    static std::pair<double, double> const nowhere { 0.0, 0.0 };
    auto const *lever { only ? nullptr : Catalog.lever( Device.lever ) };
    if( false == only && lever == nullptr ) { return fail( "no lever " + Device.lever ); }
    auto const hole = [ & ]( std::string const &Name ) -> std::pair<double, double> const * {
        if( only ) { return &nowhere; }
        auto const found { lever->holes.find( Name ) };
        return found == lever->holes.end() ? nullptr : &found->second;
    };
    auto const *messengerhole { hole( type->messenger_hole ) };
    auto const *contacthole { hole( type->contact_hole ) };
    auto const *axishole { hole( type->axis_hole ) };
    if( messengerhole == nullptr || contacthole == nullptr || axishole == nullptr ) {
        return fail( "lever " + lever->name + " has no hole " + type->messenger_hole + "/" + type->contact_hole + "/" + type->axis_hole );
    }
    // druga dźwignia (dźwignie szeregowo): lever2 urządzenia albo ta sama część co pierwsza
    auto const cascade { false == only && false == type->second_axis_hole.empty() };
    auto const *secondlever { cascade ? Catalog.lever( Device.lever2.empty() ? Device.lever : Device.lever2 ) : nullptr };
    if( cascade && secondlever == nullptr ) { return fail( "no lever " + Device.lever2 ); }
    auto const secondholeof = [ & ]( std::string const &Name ) -> std::pair<double, double> const * {
        auto const found { secondlever->holes.find( Name ) };
        return found == secondlever->holes.end() ? nullptr : &found->second;
    };
    auto const *secondmessenger { cascade ? secondholeof( type->second_messenger_hole ) : nullptr };
    auto const *secondcontact { cascade ? secondholeof( type->second_contact_hole ) : nullptr };
    auto const *secondaxis { cascade ? secondholeof( type->second_axis_hole ) : nullptr };
    if( cascade && ( secondmessenger == nullptr || secondcontact == nullptr || secondaxis == nullptr ) ) {
        return fail( "lever " + secondlever->name + " has no hole " + type->second_messenger_hole + "/" + type->second_contact_hole + "/" + type->second_axis_hole );
    }
    if( cascade && ( Device.cascade.empty() || Device.messenger2.empty() ) ) { return fail( "device " + Device.name + " has no cascade/messenger2 chain for the second lever" ); }
    Result.cascade = cascade;
    auto const series { false == Device.small_pulley.empty() };
    auto const rodname { type->rod.empty() ? Device.rod : type->rod };
    auto const rodlength { [ & ] { auto const found { Catalog.part_lengths().find( rodname ) }; return found != Catalog.part_lengths().end() ? found->second : table->rod; }() };

    // położenie z tablicy, trwałe wydłużenie sieci przesuwa rolkę do konstrukcji (ciężary 4 razy więcej)
    double b, s;
    if( false == table->lookup( Input.temperature, Input.distance, b, s ) ) {
        Result.warnings.push_back( "temperature or distance outside table " + table->name + ", taken from its edge" );
    }
    b -= Input.elongation;
    s -= 4.0 * Input.elongation;
    // pręt innej długości niż w tablicy: spód ciężarów niżej o różnicę (uwaga 3 na karcie 0011-1), góra pręta na tej samej linie
    s -= rodlength - table->rod;
    // urządzenie wyżej albo niżej na konstrukcji niż w tablicy: lina ta sama, ciężary przesunięte o tyle samo
    auto const height { Input.height > 0.0 ? Input.height : table->height };
    s += height - table->height;

    // stos ciężarów od dołu, brakujące z góry
    std::vector<tensioner_weight const *> stack;
    for( auto const &[ name, count ] : type->stack ) {
        auto const *weight { Catalog.weight( name ) };
        if( weight == nullptr ) { return fail( "no weight " + name ); }
        for( int k = 0; k < count; ++k ) { stack.push_back( weight ); }
    }
    double fullmass { 0.0 };
    for( auto const *weight : stack ) { fullmass += weight->mass; }
    for( int k = 0; k < Input.missing && false == stack.empty(); ++k ) { stack.pop_back(); }
    double mass { 0.0 }, stackheight { 0.0 };
    for( auto const *weight : stack ) { mass += weight->mass; stackheight += weight->height; }
    Result.weights = static_cast<int>( stack.size() );
    Result.stack_height = stackheight;
    Result.tension = 4.0 * mass * Input.gravity;
    Result.nominal_tension = 4.0 * fullmass * Input.gravity;

    // podział naciągu dźwignią: momenty względem osi; część naciągu na otworze liny
    auto const share = [ & ]( std::pair<double, double> const &Messenger, std::pair<double, double> const &Contact, std::pair<double, double> const &Axis ) {
        auto const arm_m { std::abs( Messenger.first - Axis.first ) }, arm_c { std::abs( Contact.first - Axis.first ) };
        return arm_m + arm_c > 1e-9 ? arm_c / ( arm_m + arm_c ) : 0.5;
    };
    auto const messengershare { only ? ( onlymessenger ? 1.0 : 0.0 ) : share( *messengerhole, *contacthole, *axishole ) };
    auto const secondshare { cascade ? share( *secondmessenger, *secondcontact, *secondaxis ) : 0.0 };
    // części naciągu całej sieci: pierwsza lina, druga lina (szeregowo), przewody. Przesunięcia końców drutów przenoszą się na oś
    // pierwszej dźwigni z tymi samymi wagami (dźwignia sztywna: oś = część·przesunięcie liny + reszta·przesunięcie drugiej strony)
    auto const first { messengershare };
    auto const second { cascade ? ( 1.0 - messengershare ) * secondshare : 0.0 };
    auto const wires { 1.0 - first - second };
    auto const ropeshare { first + second };

    // mniejszy naciąg niż w tablicy: sieć krótsza o wydłużenie sprężyste od różnicy naciągów i o nadmiar długości zwisu.
    // Wieszaki trzymają przewody pod liną, więc dodatkowy zwis jest wspólny i niesie go naciąg liny i przewodów razem:
    // lina zwisa f0·(Hm0 + Hc)/(Hm + Hc), przewody (prosto przy pełnym naciągu) f0·(Hm0 − Hm)/(Hm + Hc), gdzie
    // f0 = w·a²/(8·Hm0); nadmiar długości zwisu w przęśle 8·f²/(3·a), przęseł distance/a. Dwie liny szeregowo: każda ma połowę
    // sztywności i swoją część naciągu
    if( mass > 0.0 && Result.tension < Result.nominal_tension && ( Input.messenger_stiffness > 0.0 || Input.contact_stiffness > 0.0 ) ) {
        auto const length { std::max( 0.0, Input.distance ) };
        auto const drop { Result.nominal_tension - Result.tension };
        auto const messenger0 { Result.nominal_tension * ropeshare }, messenger { Result.tension * ropeshare };
        auto const contact0 { Result.nominal_tension * wires }, contact { Result.tension * wires };
        double sagmessengershift { 0.0 }, sagcontactshift { 0.0 };
        if( Input.load > 0.0 && Input.span > 0.0 && messenger0 > 0.0 && messenger + contact > 0.0 ) {
            auto const sag0 { Input.load * Input.span * Input.span / ( 8.0 * messenger0 ) };
            auto const sagmessenger { sag0 * ( messenger0 + contact ) / ( messenger + contact ) };
            auto const sagcontact { sag0 * ( messenger0 - messenger ) / ( messenger + contact ) };
            auto const spans { length / Input.span };
            sagmessengershift = spans * 8.0 / ( 3.0 * Input.span ) * ( sagmessenger * sagmessenger - sag0 * sag0 );
            sagcontactshift = spans * 8.0 / ( 3.0 * Input.span ) * sagcontact * sagcontact;
        }
        // przesunięcie końca jednej grupy drutów o części Part naciągu i sztywności Stiffness
        auto const shift = [ & ]( double const Part, double const Stiffness, double const Sag ) {
            return Part * ( ( Stiffness > 0.0 ? Part * drop * length / Stiffness : 0.0 ) + Sag );
        };
        if( cascade ) {
            Result.shortening = shift( first, 0.5 * Input.messenger_stiffness, sagmessengershift ) + shift( second, 0.5 * Input.messenger_stiffness, sagmessengershift );
        }
        else {
            Result.shortening = shift( first, Input.messenger_stiffness, sagmessengershift );
        }
        Result.shortening += shift( wires, Input.contact_stiffness, sagcontactshift );
        b += Result.shortening;
        s += 4.0 * Result.shortening;
    }

    // ciężary nie zejdą niżej niż grunt: lina nie puści rolki bliżej konstrukcji, sieć traci naciąg
    if( s < Input.ground ) {
        b += ( Input.ground - s ) / 4.0;
        s = Input.ground;
        Result.grounded = true;
        Result.tension = 0.0;
        Result.warnings.push_back( "the weights rest on the ground: the network has no tension from the device" );
    }
    Result.b = b;
    Result.s = s;

    Result.height = height;
    vec const out { 1.0, 0.0, 0.0 }, down { 0.0, 0.0, -1.0 };

    // rolka stała na uchwytach od czoła konstrukcji, rolka ruchoma w b
    // uchwyty do konstrukcji: 4450 zaczyna się w otworach uchwytów, hole_out przed czołem
    auto const *brackets { Input.pole.empty() ? nullptr : Device.brackets_for( Input.pole ) };
    auto const pivotx { brackets != nullptr ? brackets->hole_out : 0.0 };
    // układ konstrukcji: obrócony o structure_angle wokół pionu przez zero (czoło); części konstrukcji liczone w nim, potem tu
    auto const ca { std::cos( Input.structure_angle ) }, sa { std::sin( Input.structure_angle ) };
    auto const onstructure = [ & ]( vec const &P ) { return vec { ca * P.x - sa * P.y, sa * P.x + ca * P.y, P.z }; };
    auto const fix = [ & ]( std::size_t const From ) {
        if( Input.structure_angle == 0.0 ) { return; }
        for( auto k { From }; k < Result.parts.size(); ++k ) {
            auto &part { Result.parts[ k ] };
            auto const at { onstructure( vec { part.position[ 0 ], part.position[ 1 ], part.position[ 2 ] } ) };
            part.position[ 0 ] = at.x; part.position[ 1 ] = at.y; part.position[ 2 ] = at.z;
            for( int column = 0; column < 3; ++column ) {
                auto const axis { onstructure( vec { part.rotation[ column ], part.rotation[ 3 + column ], part.rotation[ 6 + column ] } ) };
                part.rotation[ column ] = axis.x; part.rotation[ 3 + column ] = axis.y; part.rotation[ 6 + column ] = axis.z;
            }
        }
    };
    // przód: -X modelu przed czoło (obrót o 180 stopni wokół pionu); Upside: do góry nogami (otwór nad zerem)
    auto const front = [ & ]( std::string const &Role, std::string const &Part, vec const &At, bool const Upside ) {
        if( Part.empty() ) { return; }
        auto placement { placed( Role, Part, At ) };
        if( Upside ) { set_rotation( placement, vec { -1.0, 0.0, 0.0 }, vec { 0.0, 1.0, 0.0 }, vec { 0.0, 0.0, -1.0 } ); }
        else { set_rotation( placement, vec { -1.0, 0.0, 0.0 }, vec { 0.0, -1.0, 0.0 }, vec { 0.0, 0.0, 1.0 } ); }
        Result.parts.push_back( placement );
    };
    // tył: na tylnej ścianie konstrukcji, -X modelu na zewnątrz (w -X urządzenia)
    auto const back = [ & ]( std::string const &Role, std::string const &Part, vec const &At, bool const Upside ) {
        if( Part.empty() || brackets == nullptr ) { return; }
        auto const depth { Input.depth ? Input.depth( At.z ) : 0.0 };
        if( depth <= 0.0 ) { return; }
        auto placement { placed( Role, Part, vec { -depth, At.y, At.z } ) };
        if( Upside ) { set_rotation( placement, vec { 1.0, 0.0, 0.0 }, vec { 0.0, -1.0, 0.0 }, vec { 0.0, 0.0, -1.0 } ); }
        Result.parts.push_back( placement );
        if( depth > brackets->bolt + 1e-6 ) {
            std::string const text { "the bracket bolts are shorter than the structure is deep" };
            if( std::find( Result.warnings.begin(), Result.warnings.end(), text ) == Result.warnings.end() ) { Result.warnings.push_back( text ); }
        }
    };
    auto const bracketsfrom { Result.parts.size() };
    if( brackets != nullptr ) {
        // dwa uchwyty wokół 4450: górny za mount_top, dolny odwrócony za mount_bottom
        vec const top { 0.0, 0.0, height - Device.mount_rise + Device.pivot_top + brackets->hole_drop };
        vec const bottom { 0.0, 0.0, height - Device.mount_rise - Device.pivot_bottom - brackets->hole_drop };
        front( "bracket_top", brackets->top_front, top, false );
        back( "bracket_top.back", brackets->top_back, top, false );
        front( "bracket_bottom", brackets->top_front, bottom, true );
        back( "bracket_bottom.back", brackets->top_back, bottom, true );
    }
    fix( bracketsfrom );
    // przegub 4450 w otworach uchwytów (przy konstrukcji), dalej urządzenie w stronę drutów
    // uchwyt z otworem rolki nad zerem (5040) zaczyna się niżej, rolka zostaje na wysokości z tablicy
    auto const fixedaxle { place_chain( Device.mount, "mount", onstructure( vec { pivotx, 0.0, height - Device.mount_rise } ), out, Result ) + vec { 0.0, 0.0, Device.mount_rise } };
    // rolki szeregowe: b to oś dużej ruchomej (skrajnej), uchwyt dźwigni zaczyna się na małej ruchomej
    auto const *smallpulley { series ? Catalog.pulley( Device.small_pulley ) : nullptr };
    if( series && smallpulley == nullptr ) { return fail( "no pulley " + Device.small_pulley ); }
    vec const movingaxle { series ? b - Device.small_spacing : b, 0.0, height };
    vec const movingbig { b, 0.0, height };
    auto const fixedsmall { fixedaxle + out * Device.small_spacing };
    auto const radius { 0.5 * pulley->diameter + 0.5 * Device.rope_diameter };
    auto const smallradius { series ? 0.5 * smallpulley->diameter + 0.5 * Device.rope_diameter : radius };
    if( series ? movingaxle.x - fixedsmall.x < 2.0 * smallradius : movingaxle.x - fixedaxle.x < 2.0 * radius ) {
        Result.warnings.push_back( "the pulleys overlap: the network pulled the moving one to the structure" );
    }
    if( series ) {
        double b0 { b }, s0 { 0.0 };
        table->lookup( 10.0, Input.distance, b0, s0 );
        auto const travel { b - b0 };
        // lina przechodzi: mała ruchoma d, mała stała 2d, duża ruchoma 3d, duża stała 4d
        Result.turn_moving_a = travel / smallradius;
        Result.turn_fixed_a = 2.0 * travel / smallradius;
        Result.turn_moving_b = 3.0 * travel / radius;
        Result.turn_fixed_b = 4.0 * travel / radius;
        auto const spun = [ & ]( std::string const &Role, std::string const &Part, vec const &Axle, double const Angle ) {
            auto placement { placed( Role, Part, Axle ) };
            auto const c { std::cos( Angle ) }, s { std::sin( Angle ) };
            set_rotation( placement, vec { c, 0.0, -s }, vec { 0.0, 1.0, 0.0 }, vec { s, 0.0, c } );
            Result.parts.push_back( placement );
        };
        spun( "pulley_fixed", pulley->name, fixedaxle, Result.turn_fixed_b );
        spun( "pulley_fixed_small", smallpulley->name, fixedsmall, Result.turn_fixed_a );
        spun( "pulley_moving", smallpulley->name, movingaxle, Result.turn_moving_a );
        spun( "pulley_moving_big", pulley->name, movingbig, Result.turn_moving_b );
        if( false == Device.fixed_holder.empty() ) { Result.parts.push_back( placed_along( "holder_fixed", Device.fixed_holder, fixedaxle, out ) ); }
    }
    else {
        // krążki obracają się o linę przewiniętą od położenia przy +10 °C (bez wydłużenia). Przy przesunięciu rolki ruchomej
        // o d lina przechodzi kolejno: ruchoma A d, stała A 2d, ruchoma B 3d, stała B 4d (tyle jadą ciężary); wszystkie
        // krążki obracają się w tę samą stronę: przy rolce oddalającej się od konstrukcji lina cofa się w stronę zaczepu,
        // co w płaszczyźnie X-Z jest obrotem wokół +Y (Z w stronę X)
        double b0 { b }, s0 { 0.0 };
        table->lookup( 10.0, Input.distance, b0, s0 );
        auto const travel { b - b0 };
        auto const turn = [ & ]( double const Times ) { return Times * travel / radius; };
        auto const twin { pulley->grooves >= 2 && false == pulley->sheave.empty() };
        Result.turn_moving_a = turn( 1.0 );
        Result.turn_fixed_a = turn( 2.0 );
        Result.turn_moving_b = turn( pulley->grooves >= 2 ? 3.0 : 1.0 );
        Result.turn_fixed_b = turn( pulley->grooves >= 2 ? 4.0 : 2.0 );
        auto const spun = [ & ]( std::string const &Role, std::string const &Part, vec const &Axle, double const Angle ) {
            auto placement { placed( Role, Part, Axle ) };
            auto const c { std::cos( Angle ) }, s { std::sin( Angle ) };
            set_rotation( placement, vec { c, 0.0, -s }, vec { 0.0, 1.0, 0.0 }, vec { s, 0.0, c } );
            Result.parts.push_back( placement );
        };
        spun( "pulley_fixed", pulley->name, fixedaxle, Result.turn_fixed_a );
        spun( "pulley_moving", pulley->name, movingaxle, Result.turn_moving_a );
        if( twin ) {
            spun( "sheave_fixed", pulley->sheave, fixedaxle, Result.turn_fixed_b );
            spun( "sheave_moving", pulley->sheave, movingaxle, Result.turn_moving_b );
        }
    }
    // dźwignia: otwór osi na końcu uchwytu od rolki ruchomej; przy otworze liny niżej niż otwór przewodów odwrócona.
    // Dźwignia na sworzniu staje prostopadle do wypadkowej sił z obu łańcuchów, a uchwyt 4440 idzie wzdłuż tej
    // wypadkowej (pochylenie tylko w płaszczyźnie X-Z urządzenia; łańcuchy celują dalej w 3D). Otwory zależą od
    // pochylenia, a pochylenie od kierunków z otworów, więc kilka przybliżeń (cel daleko - zbiega się od razu)
    auto const flipped { messengerhole->first < contacthole->first };
    auto const sign { flipped ? -1.0 : 1.0 };
    Result.messenger_tensions[ 0 ] = Result.tension * first;
    Result.messenger_tensions[ 1 ] = Result.tension * second;
    Result.messenger_tension = Result.tension * ropeshare;
    Result.contact_tension = Result.tension - Result.messenger_tension;
    double holderlength { 0.0 };
    for( auto const &link : Device.holder ) { holderlength += link.length; }
    // kierunek łańcucha do ostatniego podwieszenia albo poziomo; Short: cel za blisko albo za konstrukcją
    auto const aim = [ & ]( vec const &From, double const *Target, bool &Short ) {
        Short = false;
        if( false == Input.aimed ) { return out; }
        vec const to { Target[ 0 ] - From.x, Target[ 1 ] - From.y, Target[ 2 ] - From.z };
        if( to.x < 1.0 ) { Short = true; return out; }
        return unit( to );
    };
    vec pull { out };                       // kierunek uchwytu i wypadkowej, w płaszczyźnie X-Z
    vec axispoint, across, upward;          // oś dźwigni i jej osie: across = wzdłuż pull, upward = wzdłuż dźwigni
    auto const lever_frame = [ & ]() {
        axispoint = movingaxle + pull * holderlength;
        across = pull;
        upward = vec { -pull.z, 0.0, pull.x };
    };
    auto const holepoint = [ & ]( std::pair<double, double> const &Hole ) {
        return axispoint + across * ( sign * ( Hole.second - axishole->second ) ) + upward * ( sign * ( Hole.first - axishole->first ) );
    };
    lever_frame();
    // druga dźwignia (szeregowo): oś na końcu łańcucha "cascade" z otworu przewodów pierwszej, wzdłuż wypadkowej drugiej
    auto const secondsign { cascade && secondmessenger->first < secondcontact->first ? -1.0 : 1.0 };
    double cascadelength { 0.0 };
    for( auto const &link : Device.cascade ) { cascadelength += link.length; }
    vec secondpull { out }, secondaxispoint;
    auto const secondhole = [ & ]( std::pair<double, double> const &Hole ) {
        vec const up { -secondpull.z, 0.0, secondpull.x };
        return secondaxispoint + secondpull * ( secondsign * ( Hole.second - secondaxis->second ) ) + up * ( secondsign * ( Hole.first - secondaxis->first ) );
    };
    auto const second_frame = [ & ]() { if( cascade ) { secondaxispoint = holepoint( *contacthole ) + secondpull * cascadelength; } };
    second_frame();
    // wypadkowa w płaszczyźnie X-Z; dźwignia nie przechyla się dalej niż do 45 stopni (dziwny cel - zostaje bliżej pionu)
    auto const flat_pull = []( vec const &Resultant, vec &Pull ) {
        vec const flat { Resultant.x, 0.0, Resultant.z };
        if( length( flat ) < 1e-9 || flat.x < std::abs( flat.z ) ) { return false; }
        Pull = unit( flat );
        return true;
    };
    if( Input.aimed ) {
        for( int pass = 0; pass < 6; ++pass ) {
            bool shortm, shortc;
            auto const previous { pull };
            auto const wm { aim( holepoint( *messengerhole ), Input.messenger_target, shortm ) };
            vec wc;
            if( cascade ) {
                // druga dźwignia ciągnie pierwszą wzdłuż swojej wypadkowej
                auto const wm2 { aim( secondhole( *secondmessenger ), Input.messenger_target, shortm ) };
                auto const wc2 { aim( secondhole( *secondcontact ), Input.contact_target, shortc ) };
                flat_pull( wm2 * secondshare + wc2 * ( 1.0 - secondshare ), secondpull );
                wc = secondpull;
            }
            else { wc = aim( holepoint( *contacthole ), Input.contact_target, shortc ); }
            if( false == flat_pull( wm * messengershare + wc * ( 1.0 - messengershare ), pull ) ) { break; }
            lever_frame();
            second_frame();
            if( length( pull - previous ) < 1e-6 && pass > 0 ) { break; }
        }
    }
    Result.lever_tilt = std::atan2( -pull.z, pull.x );
    {
        auto const from { Result.parts.size() };
        place_chain( Device.holder, "holder", movingaxle, pull, Result );
        // rolki szeregowe: uchwyt 4440-5 odwrócony tyłem na przód względem uchwytu rolki stałej (Jan 2026-10-25) - obrót o 180
        // stopni wokół jego długiej osi
        if( series ) {
            for( auto k { from }; k < Result.parts.size(); ++k ) {
                auto &r { Result.parts[ k ].rotation };
                for( auto const column : { 1, 2 } ) { for( int row = 0; row < 3; ++row ) { r[ row * 3 + column ] = -r[ row * 3 + column ]; } }
            }
        }
    }

    // zabezpieczenie: ramiona z osi obu rolek do wspólnej dolnej śruby, para po obu stronach rolek
    // zabezpieczenie rolek szeregowych: ramiona z osi małych rolek pochylone tak, żeby ich końce dzielił drążek
    if( series && false == Device.series_arm_fixed.empty() ) {
        auto const offset { 0.5 * ( ( movingaxle.x - fixedsmall.x ) - Device.series_bar_length ) };
        if( std::abs( offset ) < Device.series_arm ) {
            auto const drop { std::sqrt( Device.series_arm * Device.series_arm - offset * offset ) };
            for( auto const side : { -1.0, 1.0 } ) {
                vec const aside { 0.0, side * Device.safety_offset, 0.0 };
                auto const a { fixedsmall + aside }, b { movingaxle + aside };
                vec const lowa { a.x + offset, a.y, height - drop }, lowb { b.x - offset, b.y, height - drop };
                auto const role { std::string( side < 0.0 ? "safety_a" : "safety_b" ) };
                Result.parts.push_back( placed_along( role + ".fixed", Device.series_arm_fixed, a, lowa - a ) );
                Result.parts.push_back( placed_along( role + ".moving", Device.series_arm_moving, b, lowb - b ) );
                if( false == Device.series_bar.empty() ) { Result.parts.push_back( placed_along( role + ".bar", Device.series_bar, lowa, lowb - lowa ) ); }
            }
        }
        else { Result.warnings.push_back( "the pulleys are further apart than the safety arms reach" ); }
    }
    if( false == series && false == Device.safety_fixed.empty() ) {
        auto const half { 0.5 * ( movingaxle.x - fixedaxle.x ) };
        if( half < Device.safety_arm ) {
            auto const drop { std::sqrt( Device.safety_arm * Device.safety_arm - half * half ) };
            for( auto const side : { -1.0, 1.0 } ) {
                vec const offset { 0.0, side * Device.safety_offset, 0.0 };
                vec const bolt { fixedaxle.x + half, offset.y, height - drop };
                auto const role { std::string( side < 0.0 ? "safety_a" : "safety_b" ) };
                Result.parts.push_back( placed_along( role + ".fixed", Device.safety_fixed, fixedaxle + offset, bolt - ( fixedaxle + offset ) ) );
                Result.parts.push_back( placed_along( role + ".moving", Device.safety_moving, movingaxle + offset, bolt - ( movingaxle + offset ) ) );
            }
        }
        else { Result.warnings.push_back( "the pulleys are further apart than the safety arms reach" ); }
    }

    // model dźwigni (Jan): zero w otworze osi II, otwory na drutach odsunięte w -X modelu, czyli w stronę drutów;
    // -X modelu idzie więc w stronę drutów (obrót o 180 stopni wokół pionu), odwrócona dodatkowo o 180 wokół Y;
    // przechylona razem z układem across/upward
    if( false == only ) {
        auto placement { placed( "lever", lever->name, holepoint( { 0.0, 0.0 } ) ) };
        set_rotation( placement, across * -sign, vec { 0.0, -1.0, 0.0 }, upward * sign );
        Result.parts.push_back( placement );
    }
    if( cascade ) {
        place_chain( Device.cascade, "cascade", holepoint( *contacthole ), secondpull, Result );
        auto placement { placed( "lever2", secondlever->name, secondhole( { 0.0, 0.0 } ) ) };
        set_rotation( placement, secondpull * -secondsign, vec { 0.0, -1.0, 0.0 }, vec { -secondpull.z, 0.0, secondpull.x } * secondsign );
        Result.parts.push_back( placement );
    }

    // łańcuchy do drutów: w stronę ostatniego podwieszenia (gdy podane), inaczej poziomo
    bool warned { false };
    auto const aimed = [ & ]( vec const &From, double const *Target ) {
        bool tooshort;
        auto const way { aim( From, Target, tooshort ) };
        if( tooshort && false == warned ) {
            Result.warnings.push_back( "the last suspension is behind or too close to the anchor, the wires are pulled level" );
            warned = true;
        }
        return way;
    };
    // lina nośna z otworu dźwigni do uchwytu krańcowego; Role: messenger albo messenger2
    // druga (dolna) lina nośna celuje o odstęp lin niżej niż pierwsza
    double const lowertarget[ 3 ] { Input.messenger_target[ 0 ], Input.messenger_target[ 1 ], Input.messenger_target[ 2 ] - Input.messenger_stack };
    auto const rope_chain = [ & ]( vec const &Start, std::vector<tensioner_link> const &Links, std::string const &Role, double *End, double const *Target ) {
        auto const way { aimed( Start, Target ) };
        auto end { place_chain( Links, Role, Start, way, Result ) };
        auto const clamp { type->messenger_clamp.empty() ? Device.messenger_clamp.name : type->messenger_clamp };
        if( false == clamp.empty() ) {
            Result.parts.push_back( placed_along( Role + "_clamp", clamp, end, way ) );
            end = end + way * Device.messenger_clamp.length;
        }
        End[ 0 ] = end.x;
        End[ 1 ] = end.y;
        End[ 2 ] = end.z;
    };
    // izolatory rolkowe tylko w zwykłym złożeniu (szeregowo izolator jest w łańcuchu do drugiej dźwigni)
    if( only && onlymessenger && Input.messengers >= 2 && false == Device.messenger_equalizer.name.empty() ) {
        // dwie liny z jednego urządzenia (sieć 32: dźwignia równoramienna 6403, 35: wyrównywacz 4410): jedna nad drugą
        auto const start { holepoint( *messengerhole ) };
        auto const way { aimed( start, Input.messenger_target ) };
        auto end { place_chain( Device.messenger, "messenger", start, way, Result ) };
        vec const up { -way.x * way.z, -way.y * way.z, way.x * way.x + way.y * way.y };
        auto const across { unit( up ) };
        auto placement { placed_along( "messenger_equalizer", Device.messenger_equalizer.name, end, way ) };
        if( false == Device.messenger_equalizer_z ) { set_rotation( placement, way * -1.0, across, cross( way * -1.0, across ) ); }
        Result.parts.push_back( placement );
        end = end + way * Device.messenger_equalizer.length;
        auto const clamp { type->messenger_clamp.empty() ? Device.messenger_clamp.name : type->messenger_clamp };
        for( int k = 0; k < 2; ++k ) {
            auto const tag { "messenger_each." + std::to_string( k + 1 ) };
            // każda lina celuje w ostatnie podwieszenie osobno (rozstaw dźwigni 6403 zbiega się w przęśle)
            auto const from { end + across * ( ( k - 0.5 ) * Device.messenger_spread ) };
            auto const own { aimed( from, k == 1 ? Input.messenger_target : lowertarget ) };
            auto at { place_chain( Device.messenger_each, tag, from, own, Result ) };
            if( false == clamp.empty() ) {
                Result.parts.push_back( placed_along( "messenger_clamp." + std::to_string( k + 1 ), clamp, at, own ) );
                at = at + own * Device.messenger_clamp.length;
            }
            auto *out { k == 1 ? Result.messenger_end : Result.messenger2_end };
            out[ 0 ] = at.x; out[ 1 ] = at.y; out[ 2 ] = at.z;
        }
    }
    else if( false == only || onlymessenger ) {
        rope_chain( holepoint( *messengerhole ), Input.old_messenger && false == cascade && false == Device.messenger_old.empty() ? Device.messenger_old : Device.messenger, "messenger", Result.messenger_end, Input.messenger_target );
    }
    if( cascade ) { rope_chain( secondhole( *secondmessenger ), Device.messenger2, "messenger2", Result.messenger2_end, lowertarget ); }
    if( false == only || false == onlymessenger ) {
        auto const start { cascade ? secondhole( *secondcontact ) : holepoint( *contacthole ) };
        auto const way { aimed( start, Input.contact_target ) };
        auto const &links { Input.old_contact && false == cascade && false == Device.contact_old.empty() ? Device.contact_old : Device.contact };
        auto end { place_chain( links, "contact", start, way, Result ) };
        std::vector<vec> ends;
        if( Input.contacts >= 2 && false == Device.equalizer.name.empty() ) {
            Result.parts.push_back( placed_along( "equalizer", Device.equalizer.name, end, way ) );
            end = end + way * Device.equalizer.length;
            // rozstaw przewodów poziomo, w poprzek kierunku łańcucha
            auto const across { unit( vec { -way.y, way.x, 0.0 } ) };
            for( int k = 0; k < Input.contacts; ++k ) {
                auto const offset { ( k - 0.5 * ( Input.contacts - 1 ) ) * Device.equalizer_spread };
                ends.push_back( end + across * offset );
            }
        }
        else {
            ends.push_back( end );
        }
        for( std::size_t k = 0; k < ends.size(); ++k ) {
            auto at { ends[ k ] };
            auto const &clamp { Input.old_fittings && false == Device.contact_clamp_old.name.empty() ? Device.contact_clamp_old : Device.contact_clamp };
            if( false == clamp.name.empty() ) {
                Result.parts.push_back( placed_along( "contact_clamp." + std::to_string( k + 1 ), clamp.name, at, way ) );
                at = at + way * clamp.length;
            }
            Result.contact_ends.push_back( { at.x, at.y, at.z } );
        }
    }

    // lina: koniec przy rolce stałej (dołem), żyły między rowkami A i B, wolny koniec z rolki stałej w dół
    auto const groove { pulley->grooves >= 2 ? 0.25 * pulley->width : 0.0 };
    double const ya { -groove }, yb { groove };
    auto &rope { Result.rope };
    auto const point = [ & ]( vec const &P ) { rope.push_back( { P.x, P.y, P.z } ); };
    auto const arc = [ & ]( vec const &Center, double const Y, double const From, double const To ) {
        // kąt w płaszczyźnie X-Z od +X w stronę +Z; punkty co najwyżej co 5 stopni (gładka rura liny), bez pierwszego
        auto const steps { std::max( 1, static_cast<int>( std::ceil( std::abs( To - From ) / ( kPi / 36.0 ) - 1e-9 ) ) ) };
        for( int k = 1; k <= steps; ++k ) {
            auto const angle { From + ( To - From ) * k / steps };
            point( vec { Center.x + radius * std::cos( angle ), Y, Center.z + radius * std::sin( angle ) } );
        }
    };
    if( series ) {
        // zaczep na uchwycie stałym, ukośnie pod małą ruchomą, wokół niej w górę, górą do małej stałej, wokół niej w dół,
        // dołem do dużej ruchomej, wokół niej w górę, górą do dużej stałej i z jej lewej strony w dół
        auto const around = [ & ]( vec const &Center, double const Radius, double const From, double const To ) {
            auto const steps { std::max( 1, static_cast<int>( std::ceil( std::abs( To - From ) / ( kPi / 36.0 ) - 1e-9 ) ) ) };
            for( int k = 1; k <= steps; ++k ) {
                auto const angle { From + ( To - From ) * k / steps };
                point( vec { Center.x + Radius * std::cos( angle ), 0.0, Center.z + Radius * std::sin( angle ) } );
            }
        };
        point( vec { fixedaxle.x + Device.dead_end, 0.0, height } );
        point( vec { movingaxle.x, 0.0, height - smallradius } );
        around( movingaxle, smallradius, -0.5 * kPi, 0.5 * kPi );
        point( vec { fixedsmall.x, 0.0, height + smallradius } );
        around( fixedsmall, smallradius, 0.5 * kPi, 1.5 * kPi );
        point( vec { movingbig.x, 0.0, height - radius } );
        around( movingbig, radius, -0.5 * kPi, 0.5 * kPi );
        point( vec { fixedaxle.x, 0.0, height + radius } );
        around( fixedaxle, radius, 0.5 * kPi, kPi );
    }
    else {
        point( vec { fixedaxle.x, 0.0, height - radius } );
        point( vec { movingaxle.x, ya, height - radius } );
        arc( movingaxle, ya, -0.5 * kPi, 0.5 * kPi );
        point( vec { fixedaxle.x, ya, height + radius } );
        arc( fixedaxle, ya, 0.5 * kPi, 1.5 * kPi );
        point( vec { movingaxle.x, yb, height - radius } );
        arc( movingaxle, yb, -0.5 * kPi, 0.5 * kPi );
        point( vec { fixedaxle.x, yb, height + radius } );
        arc( fixedaxle, yb, 0.5 * kPi, kPi );
    }

    // pręt z ciężarami pod liną, łączniki między liną a prętem
    double hangerlength { 0.0 };
    for( auto const &link : Device.hanger ) { hangerlength += link.length; }
    // oś stosu: pod rolką stałą; z prowadnicą na uchwytach ustawia ją pierścień jeżdżący po osi prowadnicy (mount 5070 w hole_1
    // uchwytu, Jan 2026-10-24) - lina schodzi z rolki lekko ukośnie
    auto const turn { Device.ring_turn * kPi / 180.0 };
    vec toward { -std::cos( turn ), std::sin( turn ), 0.0 };   // od osi stosu do prowadnicy
    auto const onbrackets { brackets != nullptr && false == Device.guide.empty() && false == Device.ring.empty() };
    auto const onframe { onbrackets && false == Device.frame.empty() };
    // na ramie 5090: prowadnica w otworze ramy, stos pod liną (Y 0) o ramię pierścienia od prowadnicy, przy konstrukcji albo dalej;
    // pierścień obrócony w stronę prowadnicy (Jan 2026-10-24)
    if( onframe ) {
        auto const offset { std::max( 0.0, Device.ring_arm * Device.ring_arm - Device.frame_y * Device.frame_y ) };
        vec const guide { brackets->hole_out + Device.frame_length, Device.frame_y, 0.0 };
        vec const stack { guide.x + Device.frame_side * std::sqrt( offset ), 0.0, 0.0 };
        toward = unit( vec { guide.x - stack.x, guide.y - stack.y, 0.0 } );
    }
    vec const guideaxis { onframe ? vec { brackets->hole_out + Device.frame_length, Device.frame_y, 0.0 } : onbrackets ? vec { brackets->hole_out, 0.0, 0.0 } - toward * Device.guide_mount : vec {} };
    // z prowadnicą stos liczony w układzie konstrukcji (fix na końcu), bez niej - pod rolką, w układzie urządzenia
    auto const columnx { onbrackets ? guideaxis.x - toward.x * Device.ring_arm : fixedaxle.x - radius };
    auto const columny { onbrackets ? guideaxis.y - toward.y * Device.ring_arm : yb };
    vec const ropeend { columnx, columny, s + rodlength + hangerlength };
    if( ropeend.z > height - 0.05 ) { Result.warnings.push_back( "the weights would hang above the pulley: rod or hangers too long for the table" ); }
    point( onbrackets ? onstructure( ropeend ) : ropeend );
    auto const stackfrom { Result.parts.size() };
    place_chain( Device.hanger, "hanger", ropeend, down, Result );
    {
        auto placement { placed( "rod", rodname, vec { columnx, columny, s } ) };
        Result.parts.push_back( placement );
    }
    auto z { s };
    int index { 0 };
    int turns { 0 }; // ciężary już leżące na pręcie (każdy kolejny obrócony o "turn" swojego rodzaju)
    for( auto const *weight : stack ) {
        ++index;
        auto const top { index == static_cast<int>( stack.size() ) };
        auto const &color { top && false == Input.top_color.empty() ? Input.top_color : Input.color };
        auto model { weight->name };
        if( false == weight->model.empty() ) {
            auto number { tensioner_color( color ) };
            if( number == 0 ) { number = 2; }
            number += weight->offset;
            model = weight->model + "-" + ( number < 10 ? "0" : "" ) + std::to_string( number );
        }
        auto placement { placed( "weight." + std::to_string( index ), model, vec { columnx, columny, z } ) };
        auto const angle { weight->turn * ( turns++ ) * kPi / 180.0 };
        set_rotation( placement, vec { std::cos( angle ), std::sin( angle ), 0.0 }, vec { -std::sin( angle ), std::cos( angle ), 0.0 }, vec { 0.0, 0.0, 1.0 } );
        Result.parts.push_back( placement );
        z += weight->height;
    }
    if( z > s + rodlength ) { Result.warnings.push_back( "the stack is higher than the rod" ); }
    // pierścień prowadzący na stosie, prowadnica w osi jego przesuwu, w stronę konstrukcji
    // obrót wokół osi stosu: -X modelu (w stronę prowadnicy) idzie w kierunku toward
    auto const turned = [ & ]( std::string const &Role, std::string const &Part, vec const &At ) {
        auto placement { placed( Role, Part, At ) };
        set_rotation( placement, toward * -1.0, vec { -toward.y, toward.x, 0.0 } * -1.0, vec { 0.0, 0.0, 1.0 } );
        return placement;
    };
    auto const ringz { Device.ring_at < 0.0 ? z : s + std::min( Device.ring_at, z - s ) };
    // prowadnica tylko na uchwytach konstrukcji (bez nich nie ma jej gdzie przykręcić) - wtedy też bez pierścienia
    if( onbrackets ) {
        Result.parts.push_back( turned( "ring", Device.ring, vec { columnx, columny, ringz } ) );
    }
    if( onbrackets ) {
        auto const guide { vec { columnx, columny, Device.guide_bottom } + toward * ( Device.ring.empty() ? 0.0 : Device.ring_arm ) };
        Result.parts.push_back( turned( "guide", Device.guide, guide ) );
        Result.has_guide = true;
        auto const at { guide + toward * Device.guide_mount };
        double const mount[ 3 ] { at.x, at.y, at.z };
        std::copy( mount, mount + 3, Result.guide_mount );
        std::copy( mount, mount + 3, Result.guide_end );
        Result.guide_end[ 2 ] += Device.guide_length;
        if( mount[ 0 ] < 0.0 ) { Result.warnings.push_back( "the guide mounts are behind the face of the structure" ); }
        // uchwyty prowadnicy (karta 33-0455): dolny poz. 2 przy spodzie, górny poz. 1 przy końcu; otwory w mount / end
        // na ramach 5090: ramy w otworach uchwytów (środek słupa), stawia je i ich uchwyty jedno z dwóch urządzeń (owner)
        if( onframe && Device.frame_owner ) {
            for( auto const z : { Result.guide_mount[ 2 ], Result.guide_end[ 2 ] } ) {
                auto placement { placed_along( z == Result.guide_mount[ 2 ] ? "frame_low" : "frame_high", Device.frame, vec { brackets->hole_out, 0.0, z }, out ) };
                // dolna rama odwrócona o 180 stopni wokół swojej długiej osi (Jan 2026-10-25), jak dolny uchwyt
                if( z == Result.guide_mount[ 2 ] ) {
                    auto &r { placement.rotation };
                    for( auto const column : { 1, 2 } ) { for( int row = 0; row < 3; ++row ) { r[ row * 3 + column ] = -r[ row * 3 + column ]; } }
                }
                Result.parts.push_back( placement );
            }
        }
        if( onbrackets && ( false == onframe || Device.frame_owner ) ) {
            // dolny odwrócony o 180 stopni (karta 33-0455): otwór nad zerem
            auto const y { onframe ? 0.0 : at.y };
            vec const low { 0.0, y, Result.guide_mount[ 2 ] - brackets->hole_drop };
            vec const high { 0.0, y, Result.guide_end[ 2 ] + brackets->hole_drop };
            front( "bracket_guide_low", brackets->bottom_front, low, true );
            back( "bracket_guide_low.back", brackets->bottom_back, low, true );
            front( "bracket_guide_high", brackets->top_front, high, false );
            back( "bracket_guide_high.back", brackets->top_back, high, false );
        }
        if( false == Device.ring.empty() && ( ringz < Device.guide_bottom || ringz > Device.guide_bottom + Device.guide_length ) ) {
            Result.warnings.push_back( "the guide ring is off the guide" );
        }
        // mocowania prowadnicy w układzie urządzenia
        for( auto *point : { Result.guide_mount, Result.guide_end } ) {
            auto const p { onstructure( vec { point[ 0 ], point[ 1 ], point[ 2 ] } ) };
            point[ 0 ] = p.x; point[ 1 ] = p.y; point[ 2 ] = p.z;
        }
    }
    if( onbrackets ) { fix( stackfrom ); }
    return true;
}

void tensioner_rope_solid( tensioner_result &Result, double const Radius, int const Sides, std::string const &Eye, double const EyeLength, std::vector<tensioner_vertex> &Triangles,
    std::string const &Spacer, double const SpacerLength ) {
    std::vector<vec> points;
    for( auto const &point : Result.rope ) {
        vec const p { point[ 0 ], point[ 1 ], point[ 2 ] };
        if( points.empty() || length( p - points.back() ) > 1e-6 ) { points.push_back( p ); }
    }
    if( points.size() < 2 || Radius <= 0.0 ) { return; }
    // ucha: zaczep ("eye") w skrajnym punkcie liny, zero ucha w stronę liny; rura zaczyna się i kończy w zerze ucha
    if( false == Eye.empty() && EyeLength > 1e-6 ) {
        auto const eye = [ & ]( std::string const &Role, vec const &Joint, vec const &Toward ) {
            auto const z { unit( Toward ) };
            auto const y { side_of( z ) };
            auto placement { placed( Role, Eye, Joint ) };
            // ucho obrócone o 90 stopni wokół liny względem pierwotnego (Jan 2026-10-25): płaszczyzna ucha pionowo
            set_rotation( placement, y, cross( y, z ) * -1.0, z );
            Result.parts.push_back( placement );
        };
        auto const n { points.size() };
        auto const startdir { unit( points[ 1 ] - points[ 0 ] ) };
        auto const enddir { unit( points[ n - 1 ] - points[ n - 2 ] ) };
        auto const start { points[ 0 ] + startdir * std::min( EyeLength, length( points[ 1 ] - points[ 0 ] ) ) };
        auto const end { points[ n - 1 ] - enddir * std::min( EyeLength, length( points[ n - 1 ] - points[ n - 2 ] ) ) };
        eye( "rope_eye.1", start, startdir * -1.0 );
        eye( "rope_eye.2", end, enddir );
        // przekładka ślizgowa na sworzniu ucha przy pręcie: sworzeń wzdłuż X ucha (ucho płaskie w Y-Z modelu), w zaczepie
        if( false == Spacer.empty() ) {
            auto const z { unit( enddir ) };
            auto const pin { side_of( z ) };   // X ucha (obróconego o 90 stopni)
            auto placement { placed( "rope_spacer", Spacer, points[ n - 1 ] - pin * ( 0.5 * SpacerLength ) ) };
            set_rotation( placement, z, cross( pin, z ), pin );
            Result.parts.push_back( placement );
        }
        points.front() = start;
        points.back() = end;
    }
    // pierścienie przekroju w każdym punkcie ścieżki, w płaszczyźnie dwusiecznej sąsiednich odcinków; oś odniesienia
    // przekroju przenoszona z punktu do punktu bez skręcania (rura się nie skręca na łukach)
    auto const count { points.size() };
    auto const sides { std::max( 3, Sides ) };
    std::vector<vec> along( count ), across( count );
    for( std::size_t k = 0; k < count; ++k ) {
        auto const before { k > 0 ? unit( points[ k ] - points[ k - 1 ] ) : vec {} };
        auto const after { k + 1 < count ? unit( points[ k + 1 ] - points[ k ] ) : vec {} };
        along[ k ] = k == 0 ? after : k + 1 == count ? before : unit( before + after );
        auto const reference { k == 0 ? side_of( along[ 0 ] ) : across[ k - 1 ] };
        across[ k ] = unit( reference - along[ k ] * dot( reference, along[ k ] ) );
    }
    auto const ring = [ & ]( std::size_t const Point, int const Side, vec &Position, vec &Normal ) {
        auto const angle { 2.0 * kPi * ( Side % sides ) / sides };
        auto const u { across[ Point ] };
        auto const v { cross( along[ Point ], u ) };
        Normal = u * std::cos( angle ) + v * std::sin( angle );
        Position = points[ Point ] + Normal * Radius;
    };
    auto const emit = [ & ]( vec const &Position, vec const &Normal ) {
        Triangles.push_back( { { Position.x, Position.y, Position.z }, { Normal.x, Normal.y, Normal.z } } );
    };
    Triangles.reserve( Triangles.size() + ( count - 1 ) * sides * 6 );
    for( std::size_t k = 0; k + 1 < count; ++k ) {
        for( int side = 0; side < sides; ++side ) {
            vec p00, n00, p01, n01, p10, n10, p11, n11;
            ring( k, side, p00, n00 );
            ring( k, side + 1, p01, n01 );
            ring( k + 1, side, p10, n10 );
            ring( k + 1, side + 1, p11, n11 );
            emit( p00, n00 ); emit( p11, n11 ); emit( p10, n10 );
            emit( p00, n00 ); emit( p01, n01 ); emit( p11, n11 );
        }
    }
}

} // namespace catenary
