/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#ifdef __linux__
#include <unistd.h>
#endif

module eu07.simulation.loadprofile;
import eu07.utilities.logs;

namespace loadprofile {

namespace {

bool profiling { false };

std::array<double, static_cast<std::size_t>( stage::count )> stagetime {};
std::array<std::size_t, static_cast<std::size_t>( kind::count )> nodecount {};
std::array<double, static_cast<std::size_t>( kind::count )> kindtime {};

std::size_t vertexcount { 0 };
std::size_t splitcount { 0 };   // shapes the trimmer had to divide
std::size_t splitpieces { 0 };  // extra pieces the trimmer produced
std::size_t memoryatstart { 0 };
double totaltime { 0.0 };

// resident set size in kilobytes, 0 when it cannot be read. linux only; on other
// platforms the memory lines of the report simply stay at zero
std::size_t
resident_memory() {
#ifdef __linux__
    std::ifstream statm( "/proc/self/statm" );
    if( false == statm.good() ) { return 0; }
    std::size_t total { 0 }, resident { 0 };
    statm >> total >> resident;
    return resident * ( static_cast<std::size_t>( ::sysconf( _SC_PAGESIZE ) ) / 1024 );
#else
    return 0;
#endif
}

std::string const stagenames[] {
    "token loop (rest)",
    "triangle import",
    "triangle insert",
    "  of which divider",
    "other node types" };

std::string const kindnames[] {
    "triangles", "lines", "track", "traction", "model",
    "dynamic", "memcell", "eventlauncher", "sound", "other" };

std::string
megabytes( std::size_t const Kilobytes ) {
    std::ostringstream converted;
    converted << std::fixed << std::setprecision( 1 ) << ( Kilobytes / 1024.0 ) << " MB";
    return converted.str();
}

std::string
seconds( double const Seconds ) {
    std::ostringstream converted;
    converted << std::fixed << std::setprecision( 2 ) << Seconds << " s";
    return converted.str();
}

} // namespace

bool
enabled() {
    return profiling;
}

void
enable( bool const Enable ) {
    profiling = Enable;
}

void
reset() {
    stagetime.fill( 0.0 );
    nodecount.fill( 0 );
    kindtime.fill( 0.0 );
    totaltime = 0.0;
    vertexcount = 0;
    splitcount = 0;
    splitpieces = 0;
    memoryatstart = resident_memory();
}

void
add_time( stage const Stage, double const Seconds ) {
    if( false == profiling ) { return; }
    stagetime[ static_cast<std::size_t>( Stage ) ] += Seconds;
}

void
add_total( double const Seconds ) {
    if( false == profiling ) { return; }
    totaltime += Seconds;
}

void
add_time( kind const Kind, double const Seconds ) {
    if( false == profiling ) { return; }
    kindtime[ static_cast<std::size_t>( Kind ) ] += Seconds;
}

void
add_node( kind const Kind ) {
    if( false == profiling ) { return; }
    ++nodecount[ static_cast<std::size_t>( Kind ) ];
}

void
add_vertices( std::size_t const Vertices ) {
    if( false == profiling ) { return; }
    vertexcount += Vertices;
}

void
add_split( std::size_t const Pieces ) {
    if( false == profiling ) { return; }
    ++splitcount;
    splitpieces += Pieces;
}

kind
kind_of( std::string_view const Type ) {

    if( ( Type == "triangles" ) || ( Type == "triangle_strip" ) || ( Type == "triangle_fan" ) ) { return kind::triangles; }
    if( ( Type == "lines" ) || ( Type == "line_strip" ) || ( Type == "line_loop" ) ) { return kind::lines; }
    if( Type == "track" ) { return kind::track; }
    if( Type == "traction" ) { return kind::traction; }
    if( Type == "model" ) { return kind::model; }
    if( Type == "dynamic" ) { return kind::dynamic; }
    if( Type == "memcell" ) { return kind::memcell; }
    if( Type == "eventlauncher" ) { return kind::eventlauncher; }
    if( Type == "sound" ) { return kind::sound; }

    return kind::other;
}

void
report( std::string const &Scenariofile ) {

    if( false == profiling ) { return; }

    auto const memoryatend { resident_memory() };

    std::ostringstream output;
    output << "-- scenery load profile: " << Scenariofile << "\n";

    // the node stages are measured inside the token loop, so what is left of the loop
    // time once they are taken out is the parsing itself. the divider is a subset of
    // the insert stage and stays out of both sums
    auto accounted { 0.0 };
    for( auto stageindex { 0u }; stageindex < static_cast<std::size_t>( stage::count ); ++stageindex ) {
        if( ( stageindex != static_cast<std::size_t>( stage::shape_divide ) )
         && ( stageindex != static_cast<std::size_t>( stage::parse ) ) ) {
            accounted += stagetime[ stageindex ];
        }
    }
    stagetime[ static_cast<std::size_t>( stage::parse ) ] = ( totaltime > accounted ? totaltime - accounted : 0.0 );
    auto const total { totaltime > accounted ? totaltime : accounted };

    for( auto stageindex { 0u }; stageindex < static_cast<std::size_t>( stage::count ); ++stageindex ) {
        auto const share {
            total > 0.0 ?
                100.0 * stagetime[ stageindex ] / total :
                0.0 };
        output
            << "   " << std::left << std::setw( 20 ) << stagenames[ stageindex ]
            << std::right << std::setw( 10 ) << seconds( stagetime[ stageindex ] )
            << std::setw( 8 ) << std::fixed << std::setprecision( 1 ) << share << " %\n";
    }
    output << "   " << std::left << std::setw( 20 ) << "total" << std::right << std::setw( 10 ) << seconds( total ) << "\n";

    output << "-- nodes\n";
    for( auto kindindex { 0u }; kindindex < static_cast<std::size_t>( kind::count ); ++kindindex ) {
        if( nodecount[ kindindex ] == 0 ) { continue; }
        output
            << "   " << std::left << std::setw( 20 ) << kindnames[ kindindex ]
            << std::right << std::setw( 12 ) << nodecount[ kindindex ] << "\n";
    }
    output << "-- time by node kind\n";
    for( auto kindindex { 0u }; kindindex < static_cast<std::size_t>( kind::count ); ++kindindex ) {
        if( kindtime[ kindindex ] < 0.005 ) { continue; }
        output
            << "   " << std::left << std::setw( 20 ) << kindnames[ kindindex ]
            << std::right << std::setw( 10 ) << seconds( kindtime[ kindindex ] )
            << std::setw( 12 ) << nodecount[ kindindex ] << " nodes\n";
    }
    output << "-- nodes\n";
    output << "   " << std::left << std::setw( 20 ) << "shape vertices" << std::right << std::setw( 12 ) << vertexcount << "\n";
    output << "   " << std::left << std::setw( 20 ) << "shapes divided" << std::right << std::setw( 12 ) << splitcount
           << "  (extra pieces: " << splitpieces << ")\n";

    output << "-- memory (resident)\n";
    output << "   " << std::left << std::setw( 20 ) << "before load" << std::right << std::setw( 12 ) << megabytes( memoryatstart ) << "\n";
    output << "   " << std::left << std::setw( 20 ) << "after load" << std::right << std::setw( 12 ) << megabytes( memoryatend ) << "\n";
    output << "   " << std::left << std::setw( 20 ) << "delta" << std::right << std::setw( 12 )
           << megabytes( memoryatend > memoryatstart ? memoryatend - memoryatstart : 0 ) << "\n";

    WriteLog( output.str() );

    std::ofstream file( "loadprofile.txt", std::ios::app );
    if( true == file.good() ) {
        file << output.str() << std::endl;
    }
}

} // loadprofile
