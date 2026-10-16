/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

class TMoverParameters;

// what changes about a vehicle when the AI hands it over to the human driver. the AI operates some
// controls directly, bypassing the cab controls a human uses, and only the AI ever releases them;
// the human gets them in the state matching the cab controls instead.
namespace handover {

// releases what the AI latches, but which is held only while pressed for the human driver:
// the horn, the sandbox, the departure signal and the brake releaser
void release_ai_controls(TMoverParameters &Occupied, TMoverParameters &Controlling);

// position of the light preset selector [1..LightsPosNo] for the human driver: the current one when it
// gives the head lights the AI has set, else the first one which does, else the current one.
// Lightstable is the end of the selector's table in use, Inventory the lights the head end has
int light_selector_position(TMoverParameters const &Occupied, int Lightstable, int Headlights, int Inventory);

// position of the headlight dimmer for the human driver: the current one, unless it's the off position
// which would hide the head lights the AI has lit. then the default one, else the first regular one,
// else any other than off
int dimmer_position(TMoverParameters const &Occupied, bool Headlightson);

} // namespace handover
