/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <algorithm>
#include <array>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <sstream>
#include <string>
#include <vector>

module eu07.simulation.cookdeps;
import eu07.utilities.logs;
import eu07.utilities.utilities;

namespace simulation::cookdeps {

namespace {

constexpr char const *magic { "eu07-cookdeps 1" };

struct entry {
    std::uintmax_t size { 0 };
    std::int64_t time { 0 };
    std::uint64_t hash { 0 };
    std::string path;
};

// fnv-1a over the file's bytes. stable across runs and builds, which is what a manifest needs;
// returns false when the file cannot be read
bool
hash_of( std::string const &Path, std::uintmax_t &Size, std::uint64_t &Hash ) {

    std::ifstream input( Path, std::ios::binary );
    if( false == input.good() ) { return false; }
    Hash = 0xcbf29ce484222325ull;
    Size = 0;
    std::array<char, 1 << 16> buffer;
    while( input ) {
        input.read( buffer.data(), static_cast<std::streamsize>( buffer.size() ) );
        auto const count { static_cast<std::size_t>( input.gcount() ) };
        for( std::size_t index = 0; index < count; ++index ) {
            Hash ^= static_cast<unsigned char>( buffer[ index ] );
            Hash *= 0x100000001b3ull;
        }
        Size += count;
    }
    return true;
}

std::vector<std::string>
unique( std::vector<std::string> Files ) {

    std::sort( Files.begin(), Files.end() );
    Files.erase( std::unique( Files.begin(), Files.end() ), Files.end() );
    return Files;
}

bool
read( std::string const &Path, std::vector<entry> &Entries ) {

    std::ifstream input( Path );
    std::string line;
    if( ( false == static_cast<bool>( std::getline( input, line ) ) ) || ( line != magic ) ) { return false; }
    while( std::getline( input, line ) ) {
        std::istringstream fields( line );
        entry item;
        if( false == static_cast<bool>( fields >> item.size >> item.time >> std::hex >> item.hash >> std::dec ) ) { return false; }
        fields.get(); // the space before the path, which may itself hold spaces
        std::getline( fields, item.path );
        Entries.push_back( item );
    }
    return true;
}

} // anonymous namespace

bool
write( std::string const &Path, std::vector<std::string> const &Files ) {

    std::ostringstream text;
    text << magic << '\n';
    for( auto const &file : unique( Files ) ) {
        entry item;
        item.path = file;
        if( false == hash_of( file, item.size, item.hash ) ) {
            // a file that was read a moment ago and cannot be now; the artifact is recorded as
            // depending on its absence, which any later read will contradict
            item.size = 0;
            item.hash = 0;
        }
        item.time = static_cast<std::int64_t>( last_modified( file ) );
        text << item.size << ' ' << item.time << ' ' << std::hex << item.hash << std::dec << ' ' << item.path << '\n';
    }

    std::ofstream output( Path, std::ios::trunc );
    output << text.str();
    if( false == output.good() ) {
        ErrorLog( "Cooked data: cannot write \"" + Path + "\"" );
        return false;
    }
    return true;
}

bool
current( std::string const &Path ) {

    std::vector<entry> entries;
    if( false == read( Path, entries ) ) { return false; }
    for( auto const &item : entries ) {
        auto const time { static_cast<std::int64_t>( last_modified( item.path ) ) };
        std::error_code error;
        auto const onsize { std::filesystem::file_size( item.path, error ) };
        if( ( false == static_cast<bool>( error ) ) && ( time != 0 ) && ( time == item.time ) && ( onsize == item.size ) ) {
            // same size and modification time: taken as unchanged without reading it
            continue;
        }
        std::uintmax_t size { 0 };
        std::uint64_t hash { 0 };
        if( ( false == hash_of( item.path, size, hash ) ) || ( size != item.size ) || ( hash != item.hash ) ) {
            WriteLog( "Cooked data: \"" + item.path + "\" has changed since \"" + Path + "\" was written" );
            return false;
        }
    }
    return true;
}

bool
lists( std::string const &Path, std::vector<std::string> const &Files ) {

    std::vector<entry> entries;
    if( false == read( Path, entries ) ) { return false; }
    std::vector<std::string> listed;
    listed.reserve( entries.size() );
    for( auto const &item : entries ) { listed.push_back( item.path ); }
    return unique( listed ) == unique( Files );
}

} // namespace simulation::cookdeps
