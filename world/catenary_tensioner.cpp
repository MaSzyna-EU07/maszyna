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

// część liniowa: +X części w kierunku Direction
tensioner_placement placed_along( std::string const &Role, std::string const &Part, vec const &At, vec const &Direction ) {
    auto placement { placed( Role, Part, At ) };
    auto const x { unit( Direction ) };
    auto const y { side_of( x ) };
    set_rotation( placement, x, y, cross( x, y ) );
    return placement;
}

// łańcuch części liniowych od From w kierunku Direction; zwraca koniec
vec place_chain( std::vector<tensioner_link> const &Links, std::string const &Role, vec From, vec const &Direction, tensioner_result &Result ) {
    auto const step { unit( Direction ) };
    int index { 0 };
    for( auto const &link : Links ) {
        Result.parts.push_back( placed_along( Role + "." + std::to_string( ++index ), link.name, From, step ) );
        From = From + step * link.length;
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
            if( ( keyword == "messenger_clamp" || keyword == "contact_clamp" || keyword == "equalizer" ) && words.size() >= 3 ) {
                auto &link { keyword == "messenger_clamp" ? device->messenger_clamp : keyword == "contact_clamp" ? device->contact_clamp : device->equalizer };
                link.name = words[ 1 ];
                if( false == number_of( words[ 2 ], link.length ) ) { report( "bad length \"" + words[ 2 ] + "\"" ); }
                if( keyword == "equalizer" && words.size() >= 5 && words[ 3 ] == "spread" ) { number_of( words[ 4 ], device->equalizer_spread ); }
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
                else if( key == "rod" ) { device->rod = value; }
                else if( key == "rope" ) { device->rope = value; }
                else if( key == "rope_diameter" ) { number_of( value, device->rope_diameter ); }
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
tensioner_lever const *tensioner_catalog::lever( std::string const &Name ) const {
    for( auto const &entry : m_levers ) { if( entry.name == Name ) { return &entry; } }
    return nullptr;
}
tensioner_device const *tensioner_catalog::device( std::string const &Name ) const {
    for( auto const &entry : m_devices ) { if( entry.name == Name ) { return &entry; } }
    return nullptr;
}

bool solve_tensioner( tensioner_catalog const &Catalog, tensioner_device const &Device, tensioner_input const &Input, tensioner_result &Result ) {
    Result = tensioner_result();
    auto const fail = [ & ]( std::string const &Text ) { Result.warnings.push_back( Text ); return false; };
    auto const *type { Device.type( Input.type ) };
    if( type == nullptr ) { return fail( "device " + Device.name + " has no data for network type " + std::to_string( Input.type ) ); }
    auto const *table { Catalog.table( Device.table ) };
    if( table == nullptr ) { return fail( "no table " + Device.table ); }
    auto const *pulley { Catalog.pulley( Device.pulley ) };
    if( pulley == nullptr ) { return fail( "no pulley " + Device.pulley ); }
    auto const *lever { Catalog.lever( Device.lever ) };
    if( lever == nullptr ) { return fail( "no lever " + Device.lever ); }
    auto const hole = [ & ]( std::string const &Name ) -> std::pair<double, double> const * {
        auto const found { lever->holes.find( Name ) };
        return found == lever->holes.end() ? nullptr : &found->second;
    };
    auto const *messengerhole { hole( type->messenger_hole ) };
    auto const *contacthole { hole( type->contact_hole ) };
    auto const *axishole { hole( type->axis_hole ) };
    if( messengerhole == nullptr || contacthole == nullptr || axishole == nullptr ) {
        return fail( "lever " + lever->name + " has no hole " + type->messenger_hole + "/" + type->contact_hole + "/" + type->axis_hole );
    }
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

    // stos ciężarów od dołu, brakujące z góry
    std::vector<tensioner_weight const *> stack;
    for( auto const &[ name, count ] : type->stack ) {
        auto const *weight { Catalog.weight( name ) };
        if( weight == nullptr ) { return fail( "no weight " + name ); }
        for( int k = 0; k < count; ++k ) { stack.push_back( weight ); }
    }
    for( int k = 0; k < Input.missing && false == stack.empty(); ++k ) { stack.pop_back(); }
    double mass { 0.0 }, stackheight { 0.0 };
    for( auto const *weight : stack ) { mass += weight->mass; stackheight += weight->height; }
    Result.weights = static_cast<int>( stack.size() );
    Result.stack_height = stackheight;
    Result.tension = 4.0 * mass * Input.gravity;

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

    auto const height { table->height };
    vec const out { 1.0, 0.0, 0.0 }, down { 0.0, 0.0, -1.0 };

    // rolka stała na uchwytach od czoła konstrukcji, rolka ruchoma w b
    auto const fixedaxle { place_chain( Device.mount, "mount", vec { 0.0, 0.0, height }, out, Result ) };
    vec const movingaxle { b, 0.0, height };
    auto const radius { 0.5 * pulley->diameter + 0.5 * Device.rope_diameter };
    if( movingaxle.x - fixedaxle.x < 2.0 * radius ) { Result.warnings.push_back( "the pulleys overlap: the network pulled the moving one to the structure" ); }
    {
        auto fixed { placed( "pulley_fixed", pulley->name, fixedaxle ) };
        Result.parts.push_back( fixed );
        Result.parts.push_back( placed( "pulley_moving", pulley->name, movingaxle ) );
    }
    auto const axispoint { place_chain( Device.holder, "holder", movingaxle, out, Result ) };

    // dźwignia: otwór osi na axispoint; przy otworze liny niżej niż otwór przewodów odwrócona
    auto const flipped { messengerhole->first < contacthole->first };
    auto const sign { flipped ? -1.0 : 1.0 };
    auto const holepoint = [ & ]( std::pair<double, double> const &Hole ) {
        return axispoint + vec { sign * ( Hole.second - axishole->second ), 0.0, sign * ( Hole.first - axishole->first ) };
    };
    {
        auto placement { placed( "lever", lever->name, holepoint( { 0.0, 0.0 } ) ) };
        set_rotation( placement, vec { sign, 0.0, 0.0 }, vec { 0.0, 1.0, 0.0 }, vec { 0.0, 0.0, sign } );
        Result.parts.push_back( placement );
    }
    // podział naciągu dźwignią: momenty względem osi
    auto const armmessenger { std::abs( messengerhole->first - axishole->first ) };
    auto const armcontact { std::abs( contacthole->first - axishole->first ) };
    if( armmessenger + armcontact > 1e-9 ) {
        Result.messenger_tension = Result.tension * armcontact / ( armmessenger + armcontact );
        Result.contact_tension = Result.tension - Result.messenger_tension;
    }

    // łańcuchy do drutów
    {
        auto end { place_chain( Device.messenger, "messenger", holepoint( *messengerhole ), out, Result ) };
        auto const clamp { type->messenger_clamp.empty() ? Device.messenger_clamp.name : type->messenger_clamp };
        if( false == clamp.empty() ) {
            Result.parts.push_back( placed_along( "messenger_clamp", clamp, end, out ) );
            end = end + out * Device.messenger_clamp.length;
        }
        Result.messenger_end[ 0 ] = end.x;
        Result.messenger_end[ 1 ] = end.y;
        Result.messenger_end[ 2 ] = end.z;
    }
    {
        auto end { place_chain( Device.contact, "contact", holepoint( *contacthole ), out, Result ) };
        std::vector<vec> ends;
        if( Input.contacts >= 2 && false == Device.equalizer.name.empty() ) {
            Result.parts.push_back( placed_along( "equalizer", Device.equalizer.name, end, out ) );
            end = end + out * Device.equalizer.length;
            for( int k = 0; k < Input.contacts; ++k ) {
                auto const offset { ( k - 0.5 * ( Input.contacts - 1 ) ) * Device.equalizer_spread };
                ends.push_back( end + vec { 0.0, offset, 0.0 } );
            }
        }
        else {
            ends.push_back( end );
        }
        for( std::size_t k = 0; k < ends.size(); ++k ) {
            auto at { ends[ k ] };
            if( false == Device.contact_clamp.name.empty() ) {
                Result.parts.push_back( placed_along( "contact_clamp." + std::to_string( k + 1 ), Device.contact_clamp.name, at, out ) );
                at = at + out * Device.contact_clamp.length;
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
        // kąt w płaszczyźnie X-Z od +X w stronę +Z; punkty co najwyżej co 15 stopni, bez pierwszego
        auto const steps { std::max( 1, static_cast<int>( std::ceil( std::abs( To - From ) / ( kPi / 12.0 ) ) ) ) };
        for( int k = 1; k <= steps; ++k ) {
            auto const angle { From + ( To - From ) * k / steps };
            point( vec { Center.x + radius * std::cos( angle ), Y, Center.z + radius * std::sin( angle ) } );
        }
    };
    point( vec { fixedaxle.x, 0.0, height - radius } );
    point( vec { movingaxle.x, ya, height - radius } );
    arc( movingaxle, ya, -0.5 * kPi, 0.5 * kPi );
    point( vec { fixedaxle.x, ya, height + radius } );
    arc( fixedaxle, ya, 0.5 * kPi, 1.5 * kPi );
    point( vec { movingaxle.x, yb, height - radius } );
    arc( movingaxle, yb, -0.5 * kPi, 0.5 * kPi );
    point( vec { fixedaxle.x, yb, height + radius } );
    arc( fixedaxle, yb, 0.5 * kPi, kPi );

    // pręt z ciężarami pod liną, łączniki między liną a prętem
    double hangerlength { 0.0 };
    for( auto const &link : Device.hanger ) { hangerlength += link.length; }
    auto const columnx { fixedaxle.x - radius };
    vec const ropeend { columnx, yb, s + rodlength + hangerlength };
    if( ropeend.z > height - 0.05 ) { Result.warnings.push_back( "the weights would hang above the pulley: rod or hangers too long for the table" ); }
    point( ropeend );
    place_chain( Device.hanger, "hanger", ropeend, down, Result );
    {
        auto placement { placed( "rod", rodname, vec { columnx, yb, s } ) };
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
        auto placement { placed( "weight." + std::to_string( index ), model, vec { columnx, yb, z } ) };
        auto const angle { weight->turn * ( turns++ ) * kPi / 180.0 };
        set_rotation( placement, vec { std::cos( angle ), std::sin( angle ), 0.0 }, vec { -std::sin( angle ), std::cos( angle ), 0.0 }, vec { 0.0, 0.0, 1.0 } );
        Result.parts.push_back( placement );
        z += weight->height;
    }
    if( z > s + rodlength ) { Result.warnings.push_back( "the stack is higher than the rod" ); }
    return true;
}

void tensioner_rope_parts( tensioner_result &Result, std::string const &Cord, double const Length, double const Girth ) {
    for( std::size_t k = 1; k < Result.rope.size(); ++k ) {
        vec const from { Result.rope[ k - 1 ][ 0 ], Result.rope[ k - 1 ][ 1 ], Result.rope[ k - 1 ][ 2 ] };
        vec const to { Result.rope[ k ][ 0 ], Result.rope[ k ][ 1 ], Result.rope[ k ][ 2 ] };
        auto const span { length( to - from ) };
        if( span < 1e-6 ) { continue; }
        auto const z { unit( to - from ) };
        auto const y { side_of( z ) };
        auto placement { placed( "rope." + std::to_string( k ), Cord, from ) };
        set_rotation( placement, cross( y, z ), y, z );
        placement.scale[ 0 ] = Girth;
        placement.scale[ 1 ] = Girth;
        placement.scale[ 2 ] = Length > 1e-9 ? span / Length : 1.0;
        Result.parts.push_back( placement );
    }
}

} // namespace catenary
