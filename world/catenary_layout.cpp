#include "world/catenary_layout.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <locale>
#include <sstream>

namespace catenary {

namespace {

bool to_number( std::string const &Text, double &Value ) {
    std::istringstream stream { Text };
    stream.imbue( std::locale::classic() );
    stream >> Value;
    return false == stream.fail() && stream.eof();
}

} // namespace

bool layout_tables::load( std::string const &Filename, std::vector<std::string> &Errors ) {

    std::ifstream file { Filename };
    if( false == file.is_open() ) { return false; }
    std::vector<std::string> tokens;
    std::string line;
    while( std::getline( file, line ) ) {
        auto const comment { line.find( "//" ) };
        if( comment != std::string::npos ) { line.erase( comment ); }
        std::istringstream words { line };
        std::string word;
        while( words >> word ) { tokens.push_back( word ); }
    }
    parse( tokens, Filename, Errors );
    return true;
}

void layout_tables::parse( std::vector<std::string> const &Tokens, std::string const &Source, std::vector<std::string> &Errors ) {

    std::size_t index { 0 };
    auto const count { Tokens.size() };
    while( index < count ) {
        auto const &key { Tokens[ index++ ] };
        if( key == "span" ) {
            // span <symbol> <strefa I> <strefa II>
            double first { 0.0 }, second { 0.0 };
            if( index + 3 <= count && to_number( Tokens[ index + 1 ], first ) && to_number( Tokens[ index + 2 ], second ) ) {
                m_spans[ Tokens[ index ] ] = { first, second };
                index += 3;
            }
            else {
                Errors.push_back( "Bad layout file \"" + Source + "\": \"span\" needs a network symbol and two lengths" );
                index = std::min( index + 3, count );
            }
        }
        else if( key == "curve_stagger" ) {
            double value { 0.0 };
            if( index < count && to_number( Tokens[ index ], value ) && value > 0.0 ) { m_curvestagger = value; }
            else { Errors.push_back( "Bad layout file \"" + Source + "\": \"curve_stagger\" needs a positive number" ); }
            ++index;
        }
        else {
            Errors.push_back( "Bad layout file \"" + Source + "\": unexpected token \"" + key + "\"" );
        }
    }
}

double layout_tables::nominal_span( std::string const &Symbol, int const Zone ) const {

    auto const lookup { m_spans.find( Symbol ) };
    if( lookup == m_spans.end() ) { return 0.0; }
    return ( Zone == 2 ? lookup->second.second : lookup->second.first );
}

std::vector<std::string> layout_tables::symbols() const {

    std::vector<std::string> result;
    for( auto const &span : m_spans ) { result.push_back( span.first ); }
    return result;
}

double curve_span( double const Radius, double const Stagger ) {

    if( Radius <= 0.0 || Stagger <= 0.0 ) { return 0.0; }
    return 2.0 * std::sqrt( 2.0 * Radius * Stagger );
}

double span_limit( layout_tables const &Tables, std::string const &Symbol, int const Zone, double const Radius, double const Stagger ) {

    auto const nominal { Tables.nominal_span( Symbol, Zone ) };
    if( nominal <= 0.0 ) { return 0.0; }
    if( Radius <= 0.0 ) { return nominal; }
    return std::min( nominal, curve_span( Radius, Stagger ) );
}

double plan_radius( double const A[ 2 ], double const B[ 2 ], double const C[ 2 ] ) {

    auto const ab { std::hypot( B[ 0 ] - A[ 0 ], B[ 1 ] - A[ 1 ] ) };
    auto const bc { std::hypot( C[ 0 ] - B[ 0 ], C[ 1 ] - B[ 1 ] ) };
    auto const ca { std::hypot( A[ 0 ] - C[ 0 ], A[ 1 ] - C[ 1 ] ) };
    // podwójne pole trójkąta
    auto const area2 { std::abs( ( B[ 0 ] - A[ 0 ] ) * ( C[ 1 ] - A[ 1 ] ) - ( B[ 1 ] - A[ 1 ] ) * ( C[ 0 ] - A[ 0 ] ) ) };
    // prawie prosta: promień ponad 20 km traktowany jak prosta
    if( area2 < 1e-9 ) { return 0.0; }
    auto const radius { ab * bc * ca / ( 2.0 * area2 ) };
    return ( radius > 20000.0 ? 0.0 : radius );
}

int reach_digit( double const Reach ) {

    if( Reach < 1.45 || Reach >= 4.45 ) { return -1; }
    return std::clamp( static_cast<int>( std::floor( ( Reach - 1.45 ) / 0.30 ) ), 0, 9 );
}

int family_decade( suspension_family const &Family ) {

    return ( Family.swinging ? 1100 : 1200 ) + ( Family.curve ? 30 : 10 ) + ( Family.toward_pole ? 0 : 10 );
}

} // namespace catenary
