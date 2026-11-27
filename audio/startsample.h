/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

namespace audio {

// the sample a sound set to start at Start (a fraction of its length, 0-1) begins to play from, in a buffer of
// Buffersize bytes of 16 bit mono samples (the sound buffers are all made so, see upload_sound()). always a sample
// the buffer has: OpenAL rejects an offset at or past the end with AL_INVALID_VALUE. 0 for an empty buffer
int start_sample( float const Start, int const Buffersize );

} // audio
