/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <cstdint>
#include <string>
#include <algorithm>
#include <set>
#include <utility>
#include <vector>
#include "utilities/Globals_macros.h"
#include "global_include/interfaces/ITexture_macros.h"
#include "scene/logicformat.h"

module eu07.simulation.logiccook;
import eu07.scene.logiccontainer;
import eu07.simcore;
import eu07.simulation.simulation;
import eu07.simulation.simulationsounds;
import eu07.utilities.logs;
import eu07.utilities.globals;
import eu07.utilities.utilities;
import eu07.simulation.cookdeps;

namespace simulation {

namespace {



// the flat record an event becomes. everything here is read off the event itself, so the
// container cannot drift from what the text path built
scene::logic::event_record
record_of( basic_event const &Event, scene::logic::event_kind const Kind, std::uint32_t const Sibling, std::uint32_t const Name ) {

    scene::logic::event_record record {};
    record.kind = static_cast<std::uint16_t>( Kind );
    record.name = Name;
    record.sibling = ( Sibling < simulation::Events.event_count() ? Sibling : scene::logic::no_index );
    record.group = ( Event.group() == null_handle ? scene::logic::no_index : static_cast<std::uint32_t>( Event.group() ) );
    record.delay = Event.m_delay;
    record.delayrandom = Event.m_delayrandom;
    record.delaydeparture = Event.m_delaydeparture;
    record.ignored = ( Event.m_ignored ? 1 : 0 );
    record.passive = ( Event.m_passive ? 1 : 0 );
    return record;
}


// where an event of this kind looks for its targets, and where it looks next. this mirrors
// each subclass's init() rather than guessing: an event that resolves against the memory
// cells will never find its target among the tracks, and saying otherwise here would bind
// it to whatever happened to share the name
std::pair<scene::logic::node_table, scene::logic::node_table>
tables_for( scene::logic::event_kind const Kind ) {

    using kind = scene::logic::event_kind;
    using table = scene::logic::node_table;

    switch( Kind ) {
        case kind::updatevalues:
        case kind::addvalues:
        case kind::copyvalues:
        case kind::getvalues:
        case kind::putvalues:
        case kind::whois:
        case kind::logvalues:
        case kind::multiple:      return { table::memory, table::none };
        case kind::texture:
        case kind::animation:
        case kind::lights:        return { table::instance, table::none };
        case kind::switchevent:
        case kind::trackvel:      return { table::path, table::none };
        case kind::voltage:       return { table::powergrid, table::none };
        case kind::sound:         return { table::sound, table::none };
        // a visible event takes model instances and tracks alike, and looks in that order
        case kind::visible:       return { table::instance, table::path };
        default:                  return { table::none, table::none };
    }
}

std::uint32_t
index_in( scene::logic::node_table const Table, std::string const &Name ) {

    using table = scene::logic::node_table;
    switch( Table ) {
        case table::memory:    return simulation::Memory.find_id( Name );
        case table::instance:  return simulation::Instances.find_id( Name );
        case table::path:      return simulation::Paths.find_id( Name );
        case table::sound:     return simulation::Sounds.find_id( Name );
        case table::powergrid: return simulation::Powergrid.find_id( Name );
        default:               return static_cast<std::uint32_t>( -1 );
    }
}

// the entries an event's targets become, each one carrying the registry it was found in
std::vector<scene::logic::target_entry>
targets_of( basic_event const &Event, scene::logic::event_kind const Kind, scene::logic::container_writer &Writer ) {

    auto const kind { Kind };
    auto const [ first, second ] { tables_for( kind ) };

    std::vector<scene::logic::target_entry> entries;
    for( auto const &name : Event.target_names() ) {

        scene::logic::target_entry entry {};
        entry.name = Writer.intern( name );
        entry.table = static_cast<std::uint16_t>( scene::logic::node_table::none );
        entry.index = scene::logic::no_index;

        for( auto const table : { first, second } ) {
            if( table == scene::logic::node_table::none ) { continue; }
            auto const index { index_in( table, name ) };
            if( index != static_cast<std::uint32_t>( -1 ) ) {
                entry.table = static_cast<std::uint16_t>( table );
                entry.index = index;
                break;
            }
        }
        entries.emplace_back( entry );
    }
    return entries;
}

} // anonymous namespace

bool
cook_logic( std::string const &Sceneryfile, std::vector<std::string> const &Sources ) {

    scene::logic::container_writer writer;
    auto const count { simulation::Events.event_count() };
    std::uint32_t unknown { 0 };
    std::set<std::string> unknownkinds;

    for( std::uint32_t index = 0; index < count; ++index ) {
        auto const *cooked { simulation::Events.event_at( index ) };
        if( cooked == nullptr ) { continue; }
        auto const name { writer.intern( cooked->name() ) };
        auto const kind { scene::logic::kind_of( cooked->type_name() ) };
        auto const record { record_of( *cooked, kind, simulation::Events.sibling_of( index ), name ) };
        if( record.kind == static_cast<std::uint16_t>( scene::logic::event_kind::unknown ) ) {
            ++unknown;
            if( unknownkinds.insert( cooked->type_name() ).second ) {
                ErrorLog( "Logic container: no cooked kind for event type \"" + cooked->type_name() + "\" (first seen on \"" + cooked->name() + "\")" );
            }
        }
        writer.add_event( record, targets_of( *cooked, kind, writer ) );
    }

    if( unknown > 0 ) {
        // a kind the container has no value for would come back as something else entirely,
        // so say so loudly rather than write a file that reads back wrong
        ErrorLog( "Logic container: " + std::to_string( unknown ) + " events of a kind this build cannot cook" );
    }

    auto const path { scene::logic::container_path( Sceneryfile ) };
    return ( true == writer.write( path ) )
        && ( true == cookdeps::write( path + ".deps", Sources ) );
}

bool
logic_stale( std::string const &Sceneryfile, std::vector<std::string> const &Sources ) {

    auto const path { scene::logic::container_path( Sceneryfile ) };

    if( false == FileExists( path ) ) {
        return true;
    }
    if( ( false == cookdeps::current( path + ".deps" ) )
     || ( false == cookdeps::lists( path + ".deps", Sources ) ) ) {
        WriteLog( "Logic container: \"" + path + "\" was made from files that have changed since, recooking" );
        return true;
    }
    // the header says whether this build can read it at all, which covers a container
    // cooked before a schema change
    if( false == scene::logic::container_reader::readable( path ) ) {
        WriteLog( "Logic container: \"" + path + "\" cannot be read by this build, recooking" );
        return true;
    }
    return false;
}

bool
verify_logic( std::string const &Sceneryfile ) {

    scene::logic::container_reader reader;
    auto const path { scene::logic::container_path( Sceneryfile ) };
    if( false == reader.open( path ) ) {
        ErrorLog( "Logic container: cannot verify \"" + path + "\"" );
        return false;
    }

    std::uint32_t mismatches { 0 };
    auto const count { simulation::Events.event_count() };
    if( reader.events() != count ) {
        ErrorLog(
            "Logic container: holds " + std::to_string( reader.events() )
            + " events, the scenery has " + std::to_string( count ) );
        ++mismatches;
    }

    auto const shared { std::min<std::size_t>( reader.events(), count ) };
    for( std::size_t index = 0; index < shared; ++index ) {
        auto const *loaded { simulation::Events.event_at( static_cast<std::uint32_t>( index ) ) };
        if( loaded == nullptr ) { continue; }
        auto const &record { reader.event( index ) };

        auto const complain {
            [ &mismatches, loaded ]( std::string const &What ) {
                ErrorLog( "Logic container: event \"" + loaded->name() + "\" " + What );
                ++mismatches;
            } };

        if( reader.string( record.name ) != loaded->name() ) {
            complain( "is called \"" + std::string( reader.string( record.name ) ) + "\" in the container" );
            continue;
        }
        if( record.kind != static_cast<std::uint16_t>( scene::logic::kind_of( loaded->type_name() ) ) ) {
            complain( "was cooked as a different kind" );
        }
        // delays are written as they are held, so they compare exactly. a difference here
        // means the record lost precision, which is the whole thing worth catching
        if( record.delay != loaded->m_delay ) {
            complain( "has a different delay in the container" );
        }
        if( record.delayrandom != loaded->m_delayrandom ) {
            complain( "has a different random delay in the container" );
        }

        auto const targets { loaded->target_names() };
        if( record.targetcount != targets.size() ) {
            complain(
                "has " + std::to_string( record.targetcount ) + " targets in the container, "
                + std::to_string( targets.size() ) + " in the scenery" );
            continue;
        }
        for( std::uint32_t target = 0; target < record.targetcount; ++target ) {
            auto const &entry { reader.target( record.targetfirst + target ) };
            if( reader.string( entry.name ) != targets[ target ] ) {
                complain( "target " + std::to_string( target ) + " is \"" + std::string( reader.string( entry.name ) ) + "\", not \"" + targets[ target ] + "\"" );
                continue;
            }
            // the index is only meaningful for the parse the container was cooked from, so
            // the check that matters is whether it still lands on that same name
            if( entry.index != scene::logic::no_index ) {
                auto const table { static_cast<scene::logic::node_table>( entry.table ) };
                if( index_in( table, targets[ target ] ) != entry.index ) {
                    complain( "target " + std::to_string( target ) + " \"" + targets[ target ] + "\" is at a different index now" );
                }
            }
        }
    }

    if( mismatches == 0 ) {
        WriteLog( "Logic container: \"" + path + "\" matches the loaded scenery, " + std::to_string( count ) + " events" );
        return true;
    }
    ErrorLog( "Logic container: \"" + path + "\" disagrees with the loaded scenery in " + std::to_string( mismatches ) + " places" );
    return false;
}

} // namespace simulation
