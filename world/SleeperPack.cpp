/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/TrackSleepers.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>

// the models of sleepers and timbers found among the files of a folder, by their names; apart from the models drawn, so the
// unit tests can look through a folder of their own

namespace track_sleepers {

namespace {

// the model of sleepers or timbers of the file added to those found, once by its name, what goes wrong left in the error
void add_pack_model( std::filesystem::directory_entry const &Entry, std::filesystem::path const &Root, std::error_code &Error, std::vector<pack_model> &Models ) {

    namespace fs = std::filesystem;
    if( false == Entry.is_regular_file( Error ) ) {
        return;
    }
    // a name with a character the code page lacks throws as it's turned into the narrow names the models are loaded by, which
    // couldn't open the file anyway: the file is left out, not the rest of the folder with it
    std::string extension;
    std::string name;
    try {
        extension = ToLower( Entry.path().extension().string() );
        if( extension != ".t3d" && extension != ".e3d" ) {
            return;
        }
        name = ToLower( fs::relative( Entry.path(), Root, Error ).replace_extension().generic_string() );
    }
    catch( std::exception const & ) {
        return;
    }
    if( Error || name.empty() ) {
        return;
    }
    if( name.find( "podklad" ) == std::string::npos && name.find( "podrozjazd" ) == std::string::npos
     && name.find( "sleeper" ) == std::string::npos && name.find( "timber" ) == std::string::npos ) {
        return;
    }
    // a timber model has the parts it's put together from, told by their names in the file
    std::ifstream file { Entry.path(), std::ios::binary };
    std::string const content { std::istreambuf_iterator<char>( file ), std::istreambuf_iterator<char>() };
    auto const has = [ &extension, &content ]( std::string const &Part ) {
        return (
            extension == ".t3d" ?
                content.find( "Name: " + Part + "\n" ) != std::string::npos || content.find( "Name: " + Part + " " ) != std::string::npos
             || content.find( "Name: " + Part + "\r" ) != std::string::npos :
                content.find( std::string { '\0' } + Part + '\0' ) != std::string::npos ); };
    auto const timber { has( "begin" ) && has( "main" ) && has( "end" ) };
    if( std::ranges::none_of( Models, [ &name ]( pack_model const &Model ) { return Model.name == name; } ) ) {
        Models.emplace_back( std::move( name ), timber );
    }
}

} // namespace

std::vector<pack_model> pack_models( std::filesystem::path const &Root ) {

    namespace fs = std::filesystem;
    std::vector<pack_model> models;
    std::error_code error;
    fs::recursive_directory_iterator entry { Root, fs::directory_options::follow_directory_symlink | fs::directory_options::skip_permission_denied, error };
    fs::recursive_directory_iterator const end;
    while( entry != end ) {
        if( error ) {
            break;
        }
        add_pack_model( *entry, Root, error, models );
        entry.increment( error );
    }
    std::ranges::sort( models, []( pack_model const &Left, pack_model const &Right ) {
        return ( Left.timber != Right.timber ? Left.timber : Left.name < Right.name ); } );
    return models;
}

} // namespace track_sleepers
