/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the horn controls of the cab, on the WarningSignal bits of the vehicle (1: low tone, 2: high tone)
namespace horn {

int const low{1};
int const high{2};

// the signal after the key or button of the Tone is pressed. a single lever for both tones, without
// separate buttons for them, can't sound both tones at once
int pressed(int Signal, int Tone, bool Singlelever);
// the signal after the key or button of the Tone is released
int released(int Signal, int Tone);

} // namespace horn
