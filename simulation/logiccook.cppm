/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <string>
#include <vector>

export module eu07.simulation.logiccook;

export {

// Cooking the LOGIC container out of a scenery that has just been loaded.
//
// The plan called for a standalone tools/scn-cook reading .scn directly. It is cooked from
// inside the engine instead, and on purpose: an event's targets, siblings and indices are the
// product of the whole scenery deserializer, not of the parser, so a standalone cooker would
// have to grow a second copy of it. Cooking from the loaded state guarantees the thing that
// actually matters - that what is written is exactly what the text path produces - and the
// determinism the plan asks for comes from the writer, not from where it is called.
namespace simulation {

// writes the container next to the scenery, with the manifest of the Sources it was made
// from. returns false and logs on failure
bool cook_logic( std::string const &Sceneryfile, std::vector<std::string> const &Sources );
// reads a cooked container back and checks it against the events currently loaded. logs
// every disagreement and returns true only when there are none
bool verify_logic( std::string const &Sceneryfile );
// whether the container next to this scenery needs cooking: there is none, this build cannot
// read the one that is there, a file it was made from has changed, or the scenery now reads
// a different set of files than Sources says it was made from
bool logic_stale( std::string const &Sceneryfile, std::vector<std::string> const &Sources );

} // namespace simulation

}
