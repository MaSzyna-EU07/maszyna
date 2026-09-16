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
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

export module eu07.simulation.loadprofile;

export {

// Scenery load profiler. Answers the questions phase 0 of the binary format plan asks:
// where the load time goes, how many nodes of each kind the scenery holds, and how much
// memory the result occupies. Measurement only - nothing here changes what gets loaded.
namespace loadprofile {

// stages the load time is split into. shape_import covers parsing the vertex data of a
// triangle node, shape_insert the region side of it: trimming to cell boundaries and
// handing the geometry to its bank
enum class stage : std::size_t {
    parse,          // token loop remainder: total time less every stage below
    shape_import,
    shape_insert,
    shape_divide,   // RaTriangleDivider, the subset of shape_insert spent splitting
    other_nodes,    // every node type other than the triangle family
    count
};

// node kinds counted separately; anything else lands in 'other'
enum class kind : std::size_t {
    triangles,
    lines,
    track,
    traction,
    model,
    dynamic,
    memcell,
    eventlauncher,
    sound,
    other,
    count
};

// discards everything gathered so far and takes the starting memory reading
void reset();
// accumulates elapsed time in the given stage
void add_time( stage Stage, double const Seconds );
// accumulates elapsed time against a node kind, so that "everything else" can be told
// apart into what it actually is
void add_time( kind const Kind, double const Seconds );
// accumulates elapsed time of the whole token loop. the parse stage is reported as
// what is left of it once the node stages are taken out
void add_total( double const Seconds );
// registers a node of the given kind
void add_node( kind const Kind );
// adds to the tally of vertices held by shape nodes
void add_vertices( std::size_t const Vertices );
// registers a shape split into extra pieces by the cell-boundary trimmer
void add_split( std::size_t const Pieces );
// true when profiling is switched on; the instrumentation is skipped otherwise
bool enabled();
// switches profiling on or off. off by default, turned on by the -loadprofile switch
void enable( bool const Enable );
// resolves the node type name used by the scenario format to a counter
kind kind_of( std::string_view const Type );
// writes the gathered numbers to the log and to loadprofile.txt in the working directory
void report( std::string const &Scenariofile );

// RAII helper accumulating the lifetime of the scope against a stage or a node kind. the
// clock is not read at all while profiling is off, since this sits on per-node paths
template <typename Counter_>
class scoped_timer {

public:
    explicit scoped_timer( Counter_ const Counter ) :
        m_counter( Counter ) {
        if( true == enabled() ) {
            m_start = std::chrono::steady_clock::now();
            m_running = true;
        } }
    ~scoped_timer() {
        if( false == m_running ) { return; }
        add_time(
            m_counter,
            std::chrono::duration<double>( std::chrono::steady_clock::now() - m_start ).count() ); }

    scoped_timer( scoped_timer const & ) = delete;
    scoped_timer &operator=( scoped_timer const & ) = delete;

private:
    Counter_ m_counter;
    std::chrono::steady_clock::time_point m_start;
    bool m_running { false };
};

} // loadprofile

} // export
