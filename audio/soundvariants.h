/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <string_view>

class cParser;

// the rules of the optional variant of a sound definition, heard instead of the definition when the listener is
// enclosed in the vehicle carrying the sound (MaSzyna-EU07/maszyna#61). a sound with a variant has it as a block inside its
// own block:
//
//   engine: {
//       soundmain: engine_outside.wav
//       inside: {
//           soundmain: engine_cab.wav
//           soundproofing: [ 1 1 1 0.8 0.8 0.8 ]
//       }
//   }
//
// the variant takes what it doesn't define itself (placement, range, offset, soundproofing, start offset, pitch variation)
// from the definition it belongs to, and the volume and pitch the vehicle gives the sound go to both. soundproofing
// applies to the variant as to any sound. a definition without a variant plays as before.
// kept apart from the audio side, so the unit tests can check them without an audio device
namespace sound_variants {

// where the listener is, numbered as the listener locations of the soundproofing tables (the column is this + 1):
// rear cab, engine compartment, front cab, cab window, a camera attached to a vehicle, a free camera
inline constexpr int rear_cab{-1};
inline constexpr int engine_compartment{0};
inline constexpr int front_cab{1};
inline constexpr int cab_window{2};
inline constexpr int attached_camera{3};
inline constexpr int free_camera{4};

// the location of the listener: the cab window when looking out of it, a camera (attached to a vehicle or free) in the
// external view, otherwise the occupied compartment of the vehicle the camera is in (-1, 0 or 1). without such a vehicle
// the listener is a free camera
int listener_location(bool Windowopen, bool Freefly, bool Invehicle, int Occupiedcab);

// true if the listener at the location is enclosed in a vehicle: in a cab or the engine compartment with the windows shut
bool listener_enclosed(int Location);

// the definition a sound plays for a listener at the location: the inside variant, if the sound has one and the listener
// is enclosed in the vehicle which carries the sound, otherwise the sound's own definition. Ownersvehicle tells whether the
// listener's vehicle is the sound's owner; the other vehicles of the consist count as outside, as in the soundproofing,
// and so does any vehicle for a sound without an owner
enum class choice {
	main,
	inside,
};
choice select(bool Hasinside, int Location, bool Ownersvehicle);

// the volume factor of a definition during the crossfade which follows the change of the chosen one: the one chosen
// rises from 0 to 1 and the other falls from 1 to 0 over Duration seconds, with equal power (the two recordings aren't
// correlated, so their powers add up). reaches exactly 1 and 0 at the end
float crossfade_gain(bool Chosen, double Elapsed, double Duration);

// the start time of the crossfade which follows a change of the chosen definition at Now, when the previous change was at
// Previousstart. a crossfade still going on is reversed from where it is, so both definitions keep their volume at the
// change; otherwise the new crossfade starts at Now
double crossfade_start(double Now, double Previousstart, double Duration);

// the key of the inside variant inside of a sound block
inline constexpr std::string_view inside_key{"inside:"};

// takes the block which comes next in the input, from its '{' to the matching '}', and returns it as text which gives the
// same tokens when parsed again. returns an empty text if the next token (which it takes) isn't '{'
std::string capture_block(cParser &Input);

} // namespace sound_variants
