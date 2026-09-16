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
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>
#include "scene/logicformat.h"

module eu07.scene.logiccontainer;
import eu07.utilities.logs;
import eu07.utilities.utilities;

namespace scene::logic {

namespace {

// pads the stream out to the next section boundary with zeroes, so a cooked file has no
// uninitialised bytes in it and two cooks of the same scenery compare equal
void
pad( std::ofstream &Output ) {

    std::array<char, alignment> zeroes {};
    auto const position { static_cast<std::size_t>( Output.tellp() ) };
    auto const padding { aligned( position ) - position };
    if( padding > 0 ) {
        Output.write( zeroes.data(), static_cast<std::streamsize>( padding ) );
    }
}

template <typename Type_>
void
write_array( std::ofstream &Output, std::vector<Type_> const &Data ) {

    if( false == Data.empty() ) {
        Output.write( reinterpret_cast<char const *>( Data.data() ), static_cast<std::streamsize>( Data.size() * sizeof( Type_ ) ) );
    }
}

template <typename Type_>
bool
read_array( std::ifstream &Input, section_entry const &Section, std::vector<Type_> &Data ) {

    if( Section.elementsize != sizeof( Type_ ) ) { return false; }
    if( Section.bytes != Section.count * sizeof( Type_ ) ) { return false; }
    Data.resize( Section.count );
    Input.seekg( static_cast<std::streamoff>( Section.offset ) );
    if( Section.count > 0 ) {
        Input.read( reinterpret_cast<char *>( Data.data() ), static_cast<std::streamsize>( Section.bytes ) );
    }
    return static_cast<bool>( Input );
}

} // anonymous namespace

std::uint32_t
container_writer::intern( std::string const &Text ) {

    auto const lookup { m_interned.find( Text ) };
    if( lookup != m_interned.end() ) {
        return lookup->second;
    }
    auto const index { static_cast<std::uint32_t>( m_strings.size() ) };
    m_strings.emplace_back( Text );
    m_interned.emplace( Text, index );
    return index;
}

void
container_writer::add_event( event_record Record, std::vector<target_entry> const &Targets ) {

    Record.targetfirst = static_cast<std::uint32_t>( m_targets.size() );
    Record.targetcount = static_cast<std::uint32_t>( Targets.size() );
    m_targets.insert( m_targets.end(), Targets.begin(), Targets.end() );
    m_events.emplace_back( Record );
}

bool
container_writer::write( std::string const &Path ) const {

    std::ofstream output( Path, std::ios::binary | std::ios::trunc );
    if( false == output.good() ) {
        ErrorLog( "Logic container: cannot write \"" + Path + "\"" );
        return false;
    }

    // the strings blob and its table are built first, since the header has to know how big
    // every section is before any of them is written
    std::vector<char> blob;
    std::vector<string_entry> table;
    table.reserve( m_strings.size() );
    for( auto const &text : m_strings ) {
        table.emplace_back( string_entry{ static_cast<std::uint32_t>( blob.size() ), static_cast<std::uint32_t>( text.size() ) } );
        blob.insert( blob.end(), text.begin(), text.end() );
    }

    std::array<section_entry, static_cast<std::size_t>( section::count )> sections {};
    auto offset { aligned( sizeof( file_header ) + sizeof( sections ) ) };

    auto const place {
        [ &offset, &sections ]( section const Kind, std::size_t const Elementsize, std::size_t const Count ) {
            auto &entry { sections[ static_cast<std::size_t>( Kind ) ] };
            entry.kind = static_cast<std::uint32_t>( Kind );
            entry.elementsize = static_cast<std::uint32_t>( Elementsize );
            entry.count = Count;
            entry.bytes = Elementsize * Count;
            entry.offset = offset;
            offset = aligned( offset + entry.bytes );
        } };

    place( section::strings, 1, blob.size() );
    place( section::stringtable, sizeof( string_entry ), table.size() );
    place( section::events, sizeof( event_record ), m_events.size() );
    place( section::targets, sizeof( target_entry ), m_targets.size() );

    file_header header {};
    header.magic = magic;
    header.version = version;
    header.schema_hash = schema_hash;
    header.endian = endian_marker;
    header.sectioncount = static_cast<std::uint32_t>( section::count );
    header.filesize = offset;

    output.write( reinterpret_cast<char const *>( &header ), sizeof( header ) );
    output.write( reinterpret_cast<char const *>( sections.data() ), sizeof( sections ) );
    pad( output );
    write_array( output, blob );
    pad( output );
    write_array( output, table );
    pad( output );
    write_array( output, m_events );
    pad( output );
    write_array( output, m_targets );
    pad( output );

    if( false == output.good() ) {
        ErrorLog( "Logic container: failed while writing \"" + Path + "\"" );
        return false;
    }
    WriteLog(
        "Logic container: wrote \"" + Path + "\", "
        + std::to_string( m_events.size() ) + " events, "
        + std::to_string( m_targets.size() ) + " targets, "
        + std::to_string( m_strings.size() ) + " strings, "
        + std::to_string( offset ) + " bytes" );
    return true;
}

// reads the header and checks it, and the file's size, against what this build writes. a
// cooked container is a cache, so anything unexpected means fall back to the text, not guess
// at what an older writer meant
static bool
accepted( std::ifstream &Input, std::string const &Path ) {

    if( false == Input.good() ) { return false; }

    file_header header {};
    Input.read( reinterpret_cast<char *>( &header ), sizeof( header ) );
    if( false == static_cast<bool>( Input ) ) { return false; }

    if( header.magic != magic ) {
        ErrorLog( "Logic container \"" + Path + "\" is not one" );
        return false;
    }
    if( ( header.version != version ) || ( header.schema_hash != schema_hash ) ) {
        WriteLog( "Logic container \"" + Path + "\" was cooked by a different build, ignoring it" );
        return false;
    }
    if( header.endian != endian_marker ) {
        ErrorLog( "Logic container \"" + Path + "\" was cooked on the other endianness" );
        return false;
    }
    if( header.sectioncount != static_cast<std::uint32_t>( section::count ) ) {
        WriteLog( "Logic container \"" + Path + "\" holds a different set of sections, ignoring it" );
        return false;
    }
    Input.seekg( 0, std::ios::end );
    if( static_cast<std::uint64_t>( Input.tellg() ) != header.filesize ) {
        ErrorLog( "Logic container \"" + Path + "\" is truncated" );
        return false;
    }
    return true;
}

bool
container_reader::readable( std::string const &Path ) {

    std::ifstream input( Path, std::ios::binary );
    return accepted( input, Path );
}

bool
container_reader::open( std::string const &Path ) {

    close();

    std::ifstream input( Path, std::ios::binary );
    if( false == accepted( input, Path ) ) { return false; }
    input.seekg( sizeof( file_header ) );

    std::array<section_entry, static_cast<std::size_t>( section::count )> sections {};
    input.read( reinterpret_cast<char *>( sections.data() ), sizeof( sections ) );
    if( false == static_cast<bool>( input ) ) { return false; }

    auto const &strings { sections[ static_cast<std::size_t>( section::strings ) ] };
    m_strings.resize( strings.count );
    input.seekg( static_cast<std::streamoff>( strings.offset ) );
    if( strings.count > 0 ) {
        input.read( m_strings.data(), static_cast<std::streamsize>( strings.bytes ) );
    }

    if( ( false == read_array( input, sections[ static_cast<std::size_t>( section::stringtable ) ], m_stringtable ) )
     || ( false == read_array( input, sections[ static_cast<std::size_t>( section::events ) ], m_events ) )
     || ( false == read_array( input, sections[ static_cast<std::size_t>( section::targets ) ], m_targets ) ) ) {
        ErrorLog( "Logic container \"" + Path + "\" has a section this build cannot read" );
        close();
        return false;
    }

    m_ready = true;
    return true;
}

void
container_reader::close() {

    m_ready = false;
    m_strings.clear();
    m_stringtable.clear();
    m_events.clear();
    m_targets.clear();
}

std::string_view
container_reader::string( std::uint32_t const Index ) const {

    if( Index >= m_stringtable.size() ) { return {}; }
    auto const &entry { m_stringtable[ Index ] };
    return { m_strings.data() + entry.offset, entry.length };
}

std::string
container_path( std::string const &Sceneryfile ) {

    return scenery_sidecar( Sceneryfile, ".logic" );
}

event_kind
kind_of( std::string const &Type ) {

         if( Type == "updatevalues" ) { return event_kind::updatevalues; }
    else if( Type == "addvalues" )    { return event_kind::addvalues; }
    else if( Type == "copyvalues" )   { return event_kind::copyvalues; }
    else if( Type == "getvalues" )    { return event_kind::getvalues; }
    else if( Type == "putvalues" )    { return event_kind::putvalues; }
    else if( Type == "whois" )        { return event_kind::whois; }
    else if( Type == "logvalues" )    { return event_kind::logvalues; }
    else if( Type == "multiple" )     { return event_kind::multiple; }
    else if( Type == "switch" )       { return event_kind::switchevent; }
    else if( Type == "trackvel" )     { return event_kind::trackvel; }
    else if( Type == "sound" )        { return event_kind::sound; }
    else if( Type == "texture" )      { return event_kind::texture; }
    else if( Type == "animation" )    { return event_kind::animation; }
    else if( Type == "lights" )       { return event_kind::lights; }
    else if( Type == "voltage" )      { return event_kind::voltage; }
    else if( Type == "visible" )      { return event_kind::visible; }
    else if( Type == "friction" )     { return event_kind::friction; }
    else if( Type == "message" )      { return event_kind::message; }
    else if( Type == "lua" )          { return event_kind::lua; }
    return event_kind::unknown;
}

} // namespace scene::logic
