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

export module eu07.simulation.cookdeps;

export {

// What a cooked artifact of a scenery was made from, and whether it still is.
//
// A manifest lists every file that went into the artifact with its size, modification time
// and a hash of its content. The artifact is current when every one of those files still has
// that content. Size and time are the quick check: when both match, the file is taken as
// unchanged; when either differs, the content is hashed, so a file that was only touched or
// copied does not force a rebuild, and one that was edited always does.
//
// A manifest cannot know about a file that was added to the scenery since it was written - an
// include line that was not there before. That shows when the scenery is read, so a caller
// compares the files it just read with the manifest afterwards, and treats the artifact as
// stale when the two differ.
namespace simulation::cookdeps {

// writes the manifest of Files, hashing each. returns false and logs on failure
bool write( std::string const &Path, std::vector<std::string> const &Files );
// whether a manifest exists at Path and every file it lists still has the content it had
bool current( std::string const &Path );
// whether the manifest at Path lists exactly Files, in any order and ignoring repeats
bool lists( std::string const &Path, std::vector<std::string> const &Files );

} // namespace simulation::cookdeps

}
