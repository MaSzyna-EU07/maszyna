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
    auto const headertype { sn_utils::ld_uint32( Input ) };
    if( ( false == Input.good() ) || ( headermain != Header ) || ( headertype != Version ) ) {
        return "is of either unrecognized type or version";
    }
    auto sectioncount { sn_utils::ld_uint32( Input ) };
    if( false == Input.good() ) {
        return "ends before the section count";
    }
    if( sectioncount > Sectionlimit ) {
        return "holds " + std::to_string( sectioncount ) + " sections, more than the " + std::to_string( Sectionlimit ) + " a region has";
    }
    std::vector<bool> present( Sectionlimit, false );
    std::uint64_t position { 3 * sizeof( std::uint32_t ) };
    std::uint32_t sectionnumber { 0 };
    while( sectionnumber < sectioncount ) {
        if( filesize - position < section_header_size ) {
            return "ends after " + std::to_string( sectionnumber ) + " of its " + std::to_string( sectioncount ) + " sections";
        }
        std::uint32_t index, size;
        auto const error { read_section_header( Input, Sectionlimit, index, size ) };
        if( false == error.empty() ) {
            return error;
        }
        if( true == present[ index ] ) {
            return "holds section " + std::to_string( index ) + " more than once";
        }
        present[ index ] = true;
        if( filesize - position < size ) {
            return "ends inside section " + std::to_string( index ) + " (" + std::to_string( sectionnumber + 1 ) + " of " + std::to_string( sectioncount ) + "), "
                + std::to_string( filesize - position ) + " of its " + std::to_string( size ) + " bytes are there";
        }
        position += size;
        Input.seekg( static_cast<std::streamoff>( position ), std::ios::beg );
        if( false == Input.good() ) {
            return "can't be read past section " + std::to_string( index );
        }
        ++sectionnumber;
    }
    if( position != filesize ) {
        return "has " + std::to_string( filesize - position ) + " bytes past its last section";
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
        return "holds section " + std::to_string( Index ) + ", out of the " + std::to_string( Sectionlimit ) + " a region has";
    }
    if( Size < section_header_size ) {
        return "gives section " + std::to_string( Index ) + " the size of " + std::to_string( Size ) + " bytes, less than its header takes";
    }
    return {};
}

std::string
write_through_temporary( std::string const &Filename, std::function<void( std::ostream & )> const &Write ) {

    auto const temporaryfile { Filename + ".tmp" };
    std::string error;
    {
        std::ofstream output { temporaryfile, std::ios::binary | std::ios::trunc };
        if( output.fail() ) {
            return "can't write to file \"" + temporaryfile + "\"";
        }
        try {
            Write( output );
            output.flush();
        }
        catch( std::exception const &Exception ) {
            error = "can't write to file \"" + temporaryfile + "\": " + Exception.what();
        }
        if( ( true == error.empty() ) && ( output.fail() ) ) {
            error = "can't write to file \"" + temporaryfile + "\"";
        }
        output.close();
        if( ( true == error.empty() ) && ( output.fail() ) ) {
            error = "can't write to file \"" + temporaryfile + "\"";
        }
    }
    std::error_code result;
    if( false == error.empty() ) {
        std::filesystem::remove( temporaryfile, result );
        return error;
    }
    // the finished file replaces the previous version in one go, so an interrupted write doesn't leave a broken one behind
    std::filesystem::rename( temporaryfile, Filename, result );
    if( result ) {
        error = "can't replace file \"" + Filename + "\": " + result.message();
        std::filesystem::remove( temporaryfile, result );
        return error;
    }
    return {};
}

} // scene::region_file
