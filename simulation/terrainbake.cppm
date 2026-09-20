/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

export module eu07.simulation.terrainbake;
import eu07.glm;
import eu07.model.vertex;

export {

// Cooking a scenery's terrain into mesh tiles on its first run, and leaving out the triangles
// the tiles stand for on every run after that.
//
// The first load lays every triangle of ground into the cook and writes the tiles beside the
// scenery, along with a note of which nodes went into them and of what the scenery looked like
// at the time. Later loads read that note: a node the tiles stand for is not built at all, one
// the tiles stand for in part is built without those triangles, and if anything the scenery is
// made of has changed since, the lot is cooked again.
namespace simulation::terrainbake {

enum class verdict {
    draw,     // the tiles do not stand for this node: build it as written
    skip,     // the tiles stand for all of it
    filter    // the tiles stand for part of it; build the rest
};

struct node_plan {
    verdict what { verdict::draw };
    // per triangle of the node, whether the scenery still draws it. only for verdict::filter
    std::vector<bool> drawn;
};

struct node_decision {
    bool skip { false };
    bool bake { false };
    std::uint64_t key { 0 };
    node_plan plan;
};

// picks up tiles cooked earlier, or arms the cook when there are none
void begin( std::string const &Sceneryfile );
// whether triangles are being collected for a cook
bool collecting();

// what to do with one triangle node of the scenery
node_decision examine(
    std::string const &File, std::size_t const Line, std::vector<std::string> const &Parameters,
    std::string_view const Type, glm::dvec3 const &Offset, glm::vec3 const &Rotation, bool const Worldspace );

// hands a node's triangles to the cook, where they are placed in the world. a material with an
// alpha channel is not ground - it is a tree, a bush, a fence or a catenary mast - and is left to
// the scenery to draw
void add(
    std::uint64_t const Key, std::vector<world_vertex> const &Vertices, std::string_view const Material,
    bool const Translucent );

// writes the tiles, or reports on what the tiles cooked earlier stood for. Included is every
// file the scenery read, which is what tells a later load whether it is still the same scenery
void finish( std::string const &Sceneryfile, std::vector<std::string> const &Included );

} // namespace simulation::terrainbake

}
