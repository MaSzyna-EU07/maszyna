/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the replaceable skins of a model: the textures a vehicle or a scenery model puts in place of the replaceable ones

#pragma once

#include <string>

#include "interfaces/IMaterial.h"
#include "utilities/Classes.h"

struct material_data {

    int textures_alpha{ 0x30300030 }; // maska przezroczystości tekstur. default: tekstury wymienne nie mają przezroczystości
    material_handle replacable_skins[ 5 ] = { null_handle, null_handle, null_handle, null_handle, null_handle }; // McZapkie:zmienialne nadwozie
    int multi_textures{ 0 }; //<0 tekstury wskazane wpisem, >0 tekstury z przecinkami, =0 jedna

    // assigns specified texture or a group of textures to replacable texture slots
    void assign( std::string const &Replacableskin );
};

// the file of the texture or the material of the given name, looked up as the models do; empty when there is none
std::string TextureTest( std::string const &Name );
