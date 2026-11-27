/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "model/materialdata.h"

#include "model/Texture.h"
#include "rendering/renderer.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"
#include "utilities/Globals.h"

// Ra 2015-01: sprawdzenie dostępności tekstury o podanej nazwie
std::string
TextureTest( std::string const &Name ) {

    auto const lookup {
        FileExists(
            { Global.asCurrentTexturePath + Name, Name, paths::textures + Name },
            { ".mat", ".dds", ".tga", ".ktx", ".png", ".bmp", ".jpg", ".tex" } ) };

    return lookup.first + lookup.second;
}

// assigns specified texture or a group of textures to replacable texture slots
void
material_data::assign( std::string const &Replacableskin ) {

    // a template puts its parameter after a folder ("tabl/(p6)"): a texture made by a script given there keeps its scheme
    if( auto const scheme { Replacableskin.find( "/make:" ) };
        scheme != std::string::npos
     && false == contains( Replacableskin, '|' ) ) {
        assign( Replacableskin.substr( scheme + 1 ) );
        return;
    }
    // check for the pipe method first
    if( contains( Replacableskin,  '|' ) ) {
        cParser nameparser( Replacableskin );
        nameparser.getTokens( 4, false, "|" );
        int skinindex = 0;
        std::string texturename; nameparser >> texturename;
        while( texturename != "" && skinindex < 4 ) {
            replacable_skins[ skinindex + 1 ] = GfxRenderer->Fetch_Material( texturename );
            ++skinindex;
            texturename = ""; nameparser >> texturename;
        }
        multi_textures = skinindex;
    }
    else {
        // otherwise try the basic approach
        int skinindex = 0;
        do {
            // test quietly for file existence so we don't generate tons of false errors in the log
            // NOTE: this means actual missing files won't get reported which is hardly ideal, but still somewhat better
            auto const material { TextureTest( ToLower( Replacableskin + "," + std::to_string( skinindex + 1 ) ) ) };
            if( true == material.empty() ) { break; }

            replacable_skins[ skinindex + 1 ] = GfxRenderer->Fetch_Material( material );
            ++skinindex;
        } while( skinindex < 4 );
        multi_textures = skinindex;
        if( multi_textures == 0 ) {
            // zestaw nie zadziałał, próbujemy normanie
            replacable_skins[ 1 ] = GfxRenderer->Fetch_Material( Replacableskin );
        }
    }
    if( replacable_skins[ 1 ] == null_handle ) {
        // last ditch attempt, check for single replacable skin texture
        replacable_skins[ 1 ] = GfxRenderer->Fetch_Material( Replacableskin );
    }

    // BUGS! it's not entierly designed whether opacity is property of material or submodel,
    // and code does confusing things with this in various places
    textures_alpha = GfxRenderer->Material(replacable_skins[1])->is_translucent() ? 0x31310031 : // tekstura -1 z kanałem alfa - nie renderować w cyklu nieprzezroczystych
	                                                                                0x30300030; // wszystkie tekstury nieprzezroczyste - nie renderować w cyklu przezroczystych
    if( GfxRenderer->Material( replacable_skins[ 2 ] )->is_translucent() ) {
        // tekstura -2 z kanałem alfa - nie renderować w cyklu nieprzezroczystych
        textures_alpha |= 0x02020002;
    }
    if( GfxRenderer->Material( replacable_skins[ 3 ] )->is_translucent() ) {
        // tekstura -3 z kanałem alfa - nie renderować w cyklu nieprzezroczystych
        textures_alpha |= 0x04040004;
    }
    if( GfxRenderer->Material( replacable_skins[ 4 ] )->is_translucent() ) {
        // tekstura -4 z kanałem alfa - nie renderować w cyklu nieprzezroczystych
        textures_alpha |= 0x08080008;
    }
}
