#include "world/catenary_suspension.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <locale>
#include <set>
#include <sstream>

namespace catenary {

namespace {

bool to_number( std::string const &Text, double &Value ) {
    std::istringstream stream { Text };
    stream.imbue( std::locale::classic() );
    stream >> Value;
    return false == stream.fail() && stream.eof();
}

std::string trim( std::string const &Text ) {
    auto const first { Text.find_first_not_of( " \t\r\n" ) };
    if( first == std::string::npos ) { return {}; }
    auto const last { Text.find_last_not_of( " \t\r\n" ) };
    return Text.substr( first, last - first + 1 );
}

// ścieżka modelu w postaci do porównań: małe litery, ukośniki w jedną stronę
std::string normalize_path( std::string Path ) {
    std::transform( Path.begin(), Path.end(), Path.begin(), []( unsigned char const Character ) { return static_cast<char>( Character == '\\' ? '/' : std::tolower( Character ) ); } );
    return Path;
}

// słowa kluczowe wewnątrz wpisu podwieszenia
bool is_keyword( std::string const &Token ) {
    static std::set<std::string> const keywords { "support", "scheme", "messenger", "contact", "insulators", "override", "skin", "at", "attach", "rope", "pole", "umbrella", "endsuspension" };
    return keywords.find( Token ) != keywords.end();
}

} // namespace

void parse_suspensions( std::vector<std::string> const &Tokens, std::string const &Source, suspension_file &Output, std::vector<std::string> &Errors ) {

    auto const count { Tokens.size() };
    std::size_t index { 0 };
    std::set<std::string> names;
    auto const report = [ & ]( std::string const &Entry, std::string const &Reason ) {
        Errors.push_back( "Bad catenary file \"" + Source + "\"" + ( Entry.empty() ? "" : ", suspension \"" + Entry + "\"" ) + ": " + Reason );
    };
    // wartości słowa kluczowego: kolejne słowa do następnego słowa kluczowego
    auto const values = [ & ]() {
        std::vector<std::string> list;
        while( index < count && false == is_keyword( Tokens[ index ] ) && Tokens[ index ] != "node" ) { list.push_back( Tokens[ index++ ] ); }
        return list;
    };

    while( index < count ) {
        auto const &token { Tokens[ index++ ] };
        if( token == "skin" ) {
            // domyślna tekstura dla całego pliku
            if( index < count ) { Output.skin = Tokens[ index++ ]; }
            else { report( "", "\"skin\" without a texture name" ); }
            continue;
        }
        if( token != "node" ) {
            report( "", "unexpected token \"" + token + "\"" );
            continue;
        }
        if( index + 4 > count ) { report( "", "incomplete \"node\" entry at the end of the file" ); break; }
        suspension entry;
        auto good { to_number( Tokens[ index ], entry.range_max ) && to_number( Tokens[ index + 1 ], entry.range_min ) };
        entry.name = Tokens[ index + 2 ];
        auto const type { Tokens[ index + 3 ] };
        index += 4;
        if( false == good ) { report( entry.name, "visibility range isn't a number" ); }
        if( type == "chain" ) {
            // łańcuch: słowa kluczowe z wartościami do endchain
            chain line;
            line.name = ( entry.name == "none" ? std::string() : entry.name );
            line.range_max = entry.range_max;
            line.range_min = entry.range_min;
            auto closed { false };
            while( index < count ) {
                auto const key { Tokens[ index++ ] };
                if( key == "endchain" ) { closed = true; break; }
                if( key == "node" ) { --index; break; }
                auto const one = [ & ]( std::string &Value ) {
                    if( index < count && Tokens[ index ] != "endchain" ) { Value = Tokens[ index++ ]; return true; }
                    report( line.name, "\"" + key + "\" needs a value" ); good = false; return false; };
                auto const number = [ & ]( double &Value ) {
                    std::string text;
                    if( one( text ) && false == to_number( text, Value ) ) { report( line.name, "\"" + key + "\" needs a number" ); good = false; } };
                if( key == "power" ) { one( line.power ); }
                else if( key == "voltage" ) { number( line.voltage ); }
                else if( key == "current" ) { number( line.current ); }
                else if( key == "resistivity" ) { number( line.resistivity ); }
                else if( key == "material" ) { one( line.material ); }
                else if( key == "thickness" ) { number( line.thickness ); }
                else if( key == "state" ) { double value { 0.0 }; number( value ); line.state = static_cast<int>( value ); }
                else if( key == "type" ) { one( line.type ); }
                else if( key == "parallel" ) { one( line.parallel ); }
                else if( key == "anchor_start" ) { one( line.anchor_start ); }
                else if( key == "anchor_end" ) { one( line.anchor_end ); }
                else if( key == "hangers" ) { for( auto &value : line.hangers ) { number( value ); } }
                else if( key == "suspensions" ) {
                    while( index < count && Tokens[ index ] != "endchain" && Tokens[ index ] != "node" ) { line.suspensions.push_back( Tokens[ index++ ] ); }
                }
                else { report( line.name, "unexpected token \"" + key + "\" in a chain" ); good = false; }
            }
            if( false == closed ) { report( line.name, "missing \"endchain\"" ); good = false; }
            if( line.power.empty() ) { report( line.name, "the chain has no \"power\"" ); good = false; }
            if( line.suspensions.size() < 2 ) { report( line.name, "a chain needs at least two suspensions" ); good = false; }
            if( good ) { Output.chains.push_back( line ); }
            continue;
        }
        if( type == "gantry" ) {
            gantry line;
            line.name = ( entry.name == "none" ? std::string() : entry.name );
            line.range_max = entry.range_max;
            line.range_min = entry.range_min;
            auto closed { false };
            int masts { 0 };
            while( index < count ) {
                auto const key { Tokens[ index++ ] };
                if( key == "endgantry" ) { closed = true; break; }
                if( key == "node" ) { --index; break; }
                auto const one = [ & ]() -> std::string {
                    if( index < count && Tokens[ index ] != "endgantry" ) { return Tokens[ index++ ]; }
                    report( line.name, "\"" + key + "\" needs a value" ); good = false; return {}; };
                if( key == "beam" ) { line.beam = one(); }
                else if( key == "level" ) { line.high = ( one() == "high" ); }
                else if( key == "side" ) { line.side = ( one() == "-1" ? -1 : 1 ); }
                else if( key == "skin" ) { line.skin = one(); }
                else if( key == "mast" ) {
                    if( masts >= 2 ) { report( line.name, "a gantry has two masts" ); good = false; break; }
                    line.masts[ masts ] = one();
                    for( auto &coordinate : line.points[ masts ] ) {
                        if( false == to_number( one(), coordinate ) ) { report( line.name, "\"mast\" needs a type and three numbers" ); good = false; break; }
                    }
                    ++masts;
                }
                else if( key == "tip" ) {
                    // pojedynczy element dźwigara na jednym słupie: koniec wspornika zamiast drugiego słupa
                    if( masts != 1 ) { report( line.name, "\"tip\" goes after the one mast of a single beam element" ); good = false; break; }
                    line.masts[ 1 ].clear();
                    for( auto &coordinate : line.points[ 1 ] ) {
                        if( false == to_number( one(), coordinate ) ) { report( line.name, "\"tip\" needs three numbers" ); good = false; break; }
                    }
                    ++masts;
                }
                else { report( line.name, "unexpected token \"" + key + "\" in a gantry" ); good = false; }
            }
            if( false == closed ) { report( line.name, "missing \"endgantry\"" ); good = false; }
            if( masts != 2 || line.beam.empty() ) { report( line.name, "a gantry needs a beam and two masts (or one mast and the tip of a single beam element)" ); good = false; }
            if( good ) { Output.gantries.push_back( line ); }
            continue;
        }
        if( type == "anchor" ) {
            anchor line;
            line.name = ( entry.name == "none" ? std::string() : entry.name );
            line.range_max = entry.range_max;
            line.range_min = entry.range_min;
            auto closed { false };
            auto haspole { false }, hastoward { false };
            while( index < count ) {
                auto const key { Tokens[ index++ ] };
                if( key == "endanchor" ) { closed = true; break; }
                if( key == "node" ) { --index; break; }
                auto const one = [ & ]() -> std::string {
                    if( index < count && Tokens[ index ] != "endanchor" ) { return Tokens[ index++ ]; }
                    report( line.name, "\"" + key + "\" needs a value" ); good = false; return {}; };
                auto const number = [ & ]( double &Value ) {
                    if( false == to_number( one(), Value ) ) { report( line.name, "\"" + key + "\" needs a number" ); good = false; } };
                auto const point = [ & ]( double *Point ) {
                    for( int k = 0; k < 3; ++k ) { number( Point[ k ] ); } };
                if( key == "device" ) { line.device = one(); }
                else if( key == "network" ) { line.network = one(); }
                else if( key == "pole" ) { line.pole_type = one(); point( line.pole ); haspole = true; }
                else if( key == "toward" ) { point( line.toward ); hastoward = true; }
                else if( key == "distance" ) { number( line.distance ); }
                else if( key == "color" ) { line.color = one(); }
                else if( key == "top_color" ) { line.top_color = one(); }
                else if( key == "elongation" ) { number( line.elongation ); }
                else if( key == "missing" ) { double value { 0.0 }; number( value ); line.missing = static_cast<int>( value ); }
                else if( key == "ground" ) { number( line.ground ); }
                else if( key == "skin" ) { line.skin = one(); }
                else { report( line.name, "unexpected token \"" + key + "\" in an anchor" ); good = false; }
            }
            if( false == closed ) { report( line.name, "missing \"endanchor\"" ); good = false; }
            if( false == haspole || false == hastoward ) { report( line.name, "an anchor needs \"pole\" and \"toward\"" ); good = false; }
            if( good ) { Output.anchors.push_back( line ); }
            continue;
        }
        if( type != "suspension" ) {
            // inny rodzaj wpisu: jeden komunikat i pominięcie całej jego treści, do następnego wpisu
            report( entry.name, "node type \"" + type + "\" isn't supported in a catenary file" );
            while( index < count && Tokens[ index ] != "node" ) { ++index; }
            continue;
        }
        // treść wpisu do endsuspension
        bool closed { false };
        while( index < count ) {
            auto const key { Tokens[ index++ ] };
            if( key == "endsuspension" ) { closed = true; break; }
            if( key == "node" ) {
                // brak endsuspension: ten wpis przepada, a następny czytany jest normalnie
                --index;
                break;
            }
            if( false == is_keyword( key ) ) {
                report( entry.name, "unexpected token \"" + key + "\"" );
                good = false;
                continue;
            }
            auto const list { values() };
            if( key == "support" ) {
                if( list.size() == 1 ) { entry.support = list.front(); }
                else { report( entry.name, "\"support\" needs exactly one node name" ); good = false; }
            }
            else if( key == "scheme" ) {
                double card { 0.0 }, execution { 0.0 };
                if( list.size() == 2 && to_number( list[ 0 ], card ) && to_number( list[ 1 ], execution ) ) {
                    entry.card = static_cast<int>( card );
                    entry.execution = static_cast<int>( execution );
                }
                else { report( entry.name, "\"scheme\" needs two numbers: catalogue card and variant" ); good = false; }
            }
            else if( key == "messenger" ) {
                entry.messenger.insert( entry.messenger.end(), list.begin(), list.end() );
            }
            else if( key == "contact" ) {
                entry.contact.insert( entry.contact.end(), list.begin(), list.end() );
            }
            else if( key == "insulators" ) {
                if( list.size() == 1 ) { entry.insulators = list.front(); }
                else { report( entry.name, "\"insulators\" needs exactly one set name" ); good = false; }
            }
            else if( key == "override" ) {
                if( list.size() == 2 ) { entry.overrides.emplace_back( list[ 0 ], list[ 1 ] ); }
                else { report( entry.name, "\"override\" needs a role and a part" ); good = false; }
            }
            else if( key == "skin" ) {
                if( list.size() == 1 ) { entry.skin = list.front(); }
                else { report( entry.name, "\"skin\" needs exactly one texture name" ); good = false; }
            }
            else if( key == "at" || key == "attach" || key == "rope" ) {
                auto *point { key == "at" ? entry.at : key == "attach" ? entry.attach : entry.rope };
                if( list.size() == 3 && to_number( list[ 0 ], point[ 0 ] ) && to_number( list[ 1 ], point[ 1 ] ) && to_number( list[ 2 ], point[ 2 ] ) ) {
                    ( key == "at" ? entry.has_at : key == "attach" ? entry.has_attach : entry.has_rope ) = true;
                }
                else { report( entry.name, "\"" + key + "\" needs three numbers: a point in scenery coordinates" ); good = false; }
            }
            else if( key == "umbrella" ) {
                if( list.size() == 1 ) { entry.umbrella = list.front(); }
                else { report( entry.name, "\"umbrella\" needs exactly one name" ); good = false; }
            }
            else if( key == "pole" ) {
                if( list.size() == 4 && to_number( list[ 1 ], entry.pole[ 0 ] ) && to_number( list[ 2 ], entry.pole[ 1 ] ) && to_number( list[ 3 ], entry.pole[ 2 ] ) ) {
                    entry.pole_type = list[ 0 ];
                    entry.has_pole = true;
                }
                else { report( entry.name, "\"pole\" needs a pole type and three numbers: the pole axis at the rail head in scenery coordinates" ); good = false; }
            }
        }
        if( false == closed ) { report( entry.name, "missing \"endsuspension\"" ); good = false; }
        if( good ) {
            if( entry.has_at || entry.has_attach || entry.has_rope || entry.has_pole ) {
                // wpis z punktami podanymi wprost: konstrukcja i przęsła nie są potrzebne, ale punkty muszą być w komplecie
                if( false == entry.has_at || false == ( entry.has_attach || entry.has_pole ) ) { report( entry.name, "explicit points need \"at\" and either \"attach\" or \"pole\"" ); good = false; }
                if( entry.has_attach && entry.has_pole ) { report( entry.name, "\"attach\" and \"pole\" can't be used together" ); good = false; }
                if( entry.card == 0 ) { report( entry.name, "explicit points need a \"scheme\"" ); good = false; }
                if( false == entry.umbrella.empty() && false == entry.has_pole ) { report( entry.name, "\"umbrella\" needs a \"pole\"" ); good = false; }
            }
            else {
                if( entry.support.empty() ) { report( entry.name, "missing \"support\"" ); good = false; }
                if( entry.messenger.empty() && entry.contact.empty() ) { report( entry.name, "no spans listed under \"messenger\" or \"contact\"" ); good = false; }
            }
            if( entry.messenger.size() > 2 ) { report( entry.name, "more than two spans listed under \"messenger\"" ); good = false; }
            if( entry.name.empty() || entry.name == "none" ) { entry.name.clear(); }
            else if( false == names.insert( entry.name ).second ) { report( entry.name, "duplicate suspension name" ); good = false; }
        }
        if( good ) { Output.entries.push_back( entry ); }
    }
    // łańcuchy wskazują podwieszenia z tego samego pliku: brakujące nazwy to błąd całego łańcucha
    for( auto chain { Output.chains.begin() }; chain != Output.chains.end(); ) {
        std::string missing;
        for( auto const &name : chain->suspensions ) {
            if( std::none_of( Output.entries.begin(), Output.entries.end(), [ & ]( suspension const &Entry ) { return Entry.name == name; } ) ) { missing = name; break; }
        }
        if( missing.empty() ) { ++chain; continue; }
        report( chain->name, "chain refers to suspension \"" + missing + "\" which isn't in the file; the chain is left out" );
        chain = Output.chains.erase( chain );
    }
}

std::string write_suspension( suspension const &Entry ) {

    auto const number = []( double const Value, int const Precision ) {
        std::ostringstream stream;
        stream.imbue( std::locale::classic() );
        stream.setf( std::ios::fixed );
        stream.precision( Precision );
        stream << Value;
        return stream.str();
    };
    auto const point = [ & ]( double const *Point ) { return number( Point[ 0 ], 3 ) + " " + number( Point[ 1 ], 3 ) + " " + number( Point[ 2 ], 3 ); };
    auto const join = []( std::vector<std::string> const &Names ) { std::string list; for( auto const &name : Names ) { list += " " + name; } return list; };
    std::string text { "node " + number( Entry.range_max, 0 ) + " " + number( Entry.range_min, 0 ) + " " + ( Entry.name.empty() ? std::string( "none" ) : Entry.name ) + " suspension" };
    if( false == Entry.support.empty() ) { text += " support " + Entry.support; }
    if( Entry.card != 0 ) { text += " scheme " + std::to_string( Entry.card ) + " " + std::to_string( Entry.execution ); }
    if( false == Entry.messenger.empty() ) { text += " messenger" + join( Entry.messenger ); }
    if( false == Entry.contact.empty() ) { text += " contact" + join( Entry.contact ); }
    if( false == Entry.insulators.empty() ) { text += " insulators " + Entry.insulators; }
    for( auto const &replacement : Entry.overrides ) { text += " override " + replacement.first + " " + replacement.second; }
    if( false == Entry.skin.empty() ) { text += " skin " + Entry.skin; }
    if( Entry.has_at ) { text += " at " + point( Entry.at ); }
    if( Entry.has_rope ) { text += " rope " + point( Entry.rope ); }
    if( Entry.has_attach ) { text += " attach " + point( Entry.attach ); }
    if( Entry.has_pole ) { text += " pole " + Entry.pole_type + " " + point( Entry.pole ); }
    if( false == Entry.umbrella.empty() ) { text += " umbrella " + Entry.umbrella; }
    return text + " endsuspension";
}

std::string write_chain( chain const &Entry ) {

    auto const number = []( double const Value ) {
        std::ostringstream stream;
        stream.imbue( std::locale::classic() );
        stream << Value;
        return stream.str();
    };
    std::string text { "node " + number( Entry.range_max ) + " " + number( Entry.range_min ) + " " + ( Entry.name.empty() ? std::string( "none" ) : Entry.name ) + " chain" };
    text += " power " + Entry.power + " voltage " + number( Entry.voltage ) + " current " + number( Entry.current ) + " resistivity " + number( Entry.resistivity );
    text += " material " + Entry.material + " thickness " + number( Entry.thickness ) + " state " + std::to_string( Entry.state );
    if( false == Entry.type.empty() ) { text += " type " + Entry.type; }
    text += " hangers " + number( Entry.hangers[ 0 ] ) + " " + number( Entry.hangers[ 1 ] ) + " " + number( Entry.hangers[ 2 ] ) + " " + number( Entry.hangers[ 3 ] );
    if( false == Entry.parallel.empty() ) { text += " parallel " + Entry.parallel; }
    if( false == Entry.anchor_start.empty() ) { text += " anchor_start " + Entry.anchor_start; }
    if( false == Entry.anchor_end.empty() ) { text += " anchor_end " + Entry.anchor_end; }
    text += " suspensions";
    for( auto const &name : Entry.suspensions ) { text += " " + name; }
    return text + " endchain";
}

std::string write_gantry( gantry const &Entry ) {

    auto const number = []( double const Value, int const Precision ) {
        std::ostringstream stream;
        stream.imbue( std::locale::classic() );
        stream.setf( std::ios::fixed );
        stream.precision( Precision );
        stream << Value;
        return stream.str();
    };
    std::string text { "node " + number( Entry.range_max, 0 ) + " " + number( Entry.range_min, 0 ) + " " + ( Entry.name.empty() ? std::string( "none" ) : Entry.name ) + " gantry" };
    text += " beam " + Entry.beam + " level " + ( Entry.high ? "high" : "low" ) + " side " + std::to_string( Entry.side );
    if( false == Entry.skin.empty() ) { text += " skin " + Entry.skin; }
    for( int mast = 0; mast < 2; ++mast ) {
        // pojedynczy element dźwigara: zamiast drugiego słupa koniec wspornika
        text += ( Entry.masts[ mast ].empty() ? std::string( " tip" ) : " mast " + Entry.masts[ mast ] ) + " " + number( Entry.points[ mast ][ 0 ], 3 ) + " " + number( Entry.points[ mast ][ 1 ], 3 ) + " " + number( Entry.points[ mast ][ 2 ], 3 );
    }
    return text + " endgantry";
}

std::string write_anchor( anchor const &Entry ) {

    auto const number = []( double const Value, int const Precision ) {
        std::ostringstream stream;
        stream.imbue( std::locale::classic() );
        stream.setf( std::ios::fixed );
        stream.precision( Precision );
        stream << Value;
        return stream.str();
    };
    auto const point = [ & ]( double const *Point ) {
        return number( Point[ 0 ], 3 ) + " " + number( Point[ 1 ], 3 ) + " " + number( Point[ 2 ], 3 );
    };
    std::string text { "node " + number( Entry.range_max, 0 ) + " " + number( Entry.range_min, 0 ) + " " + ( Entry.name.empty() ? std::string( "none" ) : Entry.name ) + " anchor" };
    text += " device " + Entry.device;
    if( false == Entry.network.empty() ) { text += " network " + Entry.network; }
    text += " pole " + Entry.pole_type + " " + point( Entry.pole ) + " toward " + point( Entry.toward ) + " distance " + number( Entry.distance, 1 );
    if( false == Entry.color.empty() ) { text += " color " + Entry.color; }
    if( false == Entry.top_color.empty() ) { text += " top_color " + Entry.top_color; }
    if( Entry.elongation != 0.0 ) { text += " elongation " + number( Entry.elongation, 3 ); }
    if( Entry.missing != 0 ) { text += " missing " + std::to_string( Entry.missing ); }
    if( Entry.ground != -0.50 ) { text += " ground " + number( Entry.ground, 2 ); }
    if( false == Entry.skin.empty() ) { text += " skin " + Entry.skin; }
    return text + " endanchor";
}

double support_data::half_width( double const Height ) const {
    if( height <= 0.0 ) { return 0.5 * width_base; }
    auto const t { std::clamp( Height / height, 0.0, 1.0 ) };
    return 0.5 * ( width_base + ( width_top - width_base ) * t );
}

bool support_registry::load( std::string const &Filename, std::vector<std::string> &Errors ) {

    std::ifstream file { Filename };
    if( !file ) { return false; }
    std::string line, model;
    support_data data;
    bool inside { false };
    int linenumber { 0 };
    auto const report = [ & ]( std::string const &Reason ) {
        Errors.push_back( "Bad support registry \"" + Filename + "\" (line " + std::to_string( linenumber ) + "): " + Reason );
    };
    while( std::getline( file, line ) ) {
        ++linenumber;
        auto const comment { line.find( "//" ) };
        if( comment != std::string::npos ) { line.erase( comment ); }
        line = trim( line );
        if( line.empty() ) { continue; }
        if( false == inside ) {
            // support <ścieżka modelu> {
            std::istringstream stream { line };
            std::string keyword, brace;
            stream >> keyword >> model >> brace;
            if( keyword != "support" || model.empty() || brace != "{" ) { report( "expected: support <model path> {" ); continue; }
            data = support_data();
            inside = true;
            continue;
        }
        if( line == "}" ) {
            if( data.type.empty() ) { report( "support \"" + model + "\" has no \"type\"" ); }
            else if( data.height <= 0.0 || data.width_base <= 0.0 || data.width_top <= 0.0 ) { report( "support \"" + model + "\" needs positive height, width_base and width_top" ); }
            else { m_supports[ normalize_path( model ) ] = data; }
            inside = false;
            continue;
        }
        auto const colon { line.find( ':' ) };
        if( colon == std::string::npos ) { report( "expected: <key>: <value>" ); continue; }
        auto const key { trim( line.substr( 0, colon ) ) }, value { trim( line.substr( colon + 1 ) ) };
        if( key == "type" ) { data.type = value; }
        else if( key == "height" || key == "width_base" || key == "width_top" ) {
            double number { 0.0 };
            if( false == to_number( value, number ) ) { report( "\"" + key + "\" isn't a number" ); continue; }
            if( key == "height" ) { data.height = number; }
            else if( key == "width_base" ) { data.width_base = number; }
            else { data.width_top = number; }
        }
        else { report( "unknown key \"" + key + "\"" ); }
    }
    if( inside ) { report( "support \"" + model + "\" isn't closed with \"}\"" ); }
    return true;
}

support_data const *support_registry::find( std::string const &Model ) const {
    auto const lookup { m_supports.find( normalize_path( Model ) ) };
    return ( lookup != m_supports.end() ? &lookup->second : nullptr );
}

} // namespace catenary
