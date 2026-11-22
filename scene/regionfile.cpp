/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/regionfile.h"
#include "scene/sn_utils.h"

#include <filesystem>
#include <format>
#include <fstream>
#include <vector>

namespace scene::region_file {

std::string
check_layout( std::istream &Input, std::uint32_t const Header, std::uint32_t const Version, std::uint32_t const Sectionlimit ) {

    Input.clear();
    Input.seekg( 0, std::ios::end );
    auto const end { Input.tellg() };
    Input.seekg( 0, std::ios::beg );
    if( ( end < 0 ) || ( false == Input.good() ) ) {
        return "can't be read";
    }
    auto const filesize { static_cast<std::uint64_t>( end ) };

    auto const headermain { sn_utils::ld_uint32( Input ) };
    if( auto const headertype { sn_utils::ld_uint32( Input ) }; ( false == Input.good() ) || ( headermain != Header ) || ( headertype != Version ) ) {
        return "is of either unrecognized type or version";
    }
    auto sectioncount { sn_utils::ld_uint32( Input ) };
    if( false == Input.good() ) {
        return "ends before the section count";
    }
    if( sectioncount > Sectionlimit ) {
        return std::format( "holds {} sections, more than the {} a region has", sectioncount, Sectionlimit );
    }
    std::vector<bool> present( Sectionlimit, false );
    std::uint64_t position { 3 * sizeof( std::uint32_t ) };
    std::uint32_t sectionnumber { 0 };
    while( sectionnumber < sectioncount ) {
        if( filesize - position < section_header_size ) {
            return std::format( "ends after {} of its {} sections", sectionnumber, sectioncount );
        }
        std::uint32_t index;
        std::uint32_t size;
        if( auto const error { read_section_header( Input, Sectionlimit, index, size ) }; false == error.empty() ) {
            return error;
        }
        if( true == present[ index ] ) {
            return std::format( "holds section {} more than once", index );
        }
        present[ index ] = true;
        if( filesize - position < size ) {
            return std::format(
                "ends inside section {} ({} of {}), {} of its {} bytes are there", index, sectionnumber + 1, sectioncount, filesize - position, size );
        }
        position += size;
        Input.seekg( static_cast<std::streamoff>( position ), std::ios::beg );
        if( false == Input.good() ) {
            return std::format( "can't be read past section {}", index );
        }
        ++sectionnumber;
    }
    if( position != filesize ) {
        return std::format( "has {} bytes past its last section", filesize - position );
    }
    return {};
}

std::string
read_section_header( std::istream &Input, std::uint32_t const Sectionlimit, std::uint32_t &Index, std::uint32_t &Size ) {

    Index = sn_utils::ld_uint32( Input );
    Size = sn_utils::ld_uint32( Input );
    if( false == Input.good() ) {
        return "ends inside a section header";
    }
    if( Index >= Sectionlimit ) {
        return std::format( "holds section {}, out of the {} a region has", Index, Sectionlimit );
    }
    if( Size < section_header_size ) {
        return std::format( "gives section {} the size of {} bytes, less than its header takes", Index, Size );
    }
    return {};
}

std::string
open_temporary( std::string const &Filename, std::ofstream &Output ) {

    Output.open( Filename + ".tmp", std::ios::binary | std::ios::trunc );
    if( Output.fail() ) {
        return "can't write to file \"" + Filename + ".tmp\"";
    }
    return {};
}

std::string
finish_temporary( std::string const &Filename, std::ofstream &Output, std::string Error ) {

    if( ( true == Error.empty() ) && ( Output.fail() ) ) {
        Error = "can't write to file \"" + Filename + ".tmp\"";
    }
    Output.close();
    if( ( true == Error.empty() ) && ( Output.fail() ) ) {
        Error = "can't write to file \"" + Filename + ".tmp\"";
    }
    std::filesystem::path const temporaryfile { Filename + ".tmp" };
    std::error_code result;
    if( false == Error.empty() ) {
        std::filesystem::remove( temporaryfile, result );
        return Error;
    }
    // the finished file replaces the previous version in one go, so an interrupted write doesn't leave a broken one behind
    std::filesystem::rename( temporaryfile, Filename, result );
    if( result ) {
        Error = "can't replace file \"" + Filename + "\": " + result.message();
        std::filesystem::remove( temporaryfile, result );
        return Error;
    }
    return {};
}

} // scene::region_file
