/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "audio/startsample.h"

#include <algorithm>
#include <cstdint>

namespace audio {

int start_sample( float const Start, int const Buffersize ) {

    auto const samples { Buffersize / static_cast<int>( sizeof( std::int16_t ) ) };
    if( samples <= 0
     || false == ( Start > 0.f ) ) { // also a NaN
        return 0;
    }
    return std::min( static_cast<int>( std::min( Start, 1.f ) * samples ), samples - 1 );
}

} // audio
