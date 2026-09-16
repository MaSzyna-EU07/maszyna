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

// Baking a scenery's heightfield on its first run, and deciding on every later run which
// triangles it replaces.
//
// A cooked heightfield is what makes a large scenery affordable - l204 loads in 25 seconds
// against 219, in 1.9 GB against 9.1 - but until now it had to be produced by hand with the
// terraincook tool and named in the scenery text. This bakes it the first time the scenery is
// opened and picks it up by itself afterwards.
//
// Nothing here guesses what terrain is. Every triangle the scenery draws is laid into the
// heightfield where it is drawn, and afterwards each one is asked whether the heightfield still
// shows it. A tree, a wall or a gantry stands on its edge and never reaches a sample; a bridge
// deck loses its samples to the ground beneath; those are drawn as they always were. Only
// triangles the heightfield really shows stop being drawn on their own, so nothing can vanish
// from the scenery - the worst a bad bake can do is draw something twice.
//
// A triangle node is recognised by the file and line it is written at - with that file's size
// and modification time - the parameters its include was given, and the origin and rotation in
// force, not by its position in the load. An edited file changes the keys of everything in it,
// and those nodes are then simply drawn again instead of the wrong ones being dropped. Hashing
// the node's text instead would cost a second parse of every triangle node in the scenery.
//
// The rasterising is scene/terraincooker.h, the same code the tool uses.
namespace simulation::terrainbake {

// what to do with one triangle node
enum class verdict {
    draw,   // the heightfield does not show it: import and draw it as written
    skip,   // the heightfield shows all of it: do not even import it
    filter  // the heightfield shows some of it: import it and draw only the rest
};

struct node_plan {
    verdict what { verdict::draw };
    // for filter: one entry per triangle in import order, true where it is still drawn
    std::vector<bool> drawn;
};


// called before the scenery is parsed. picks up a heightfield baked earlier together with its
// node table, or arms the bake when there is none
void begin( std::string const &Sceneryfile );
// what the load is to do with one triangle node, decided in one place: whether it is read at
// all, whether it goes into a bake, and which of its triangles are still drawn
struct node_decision {
    bool skip { false };      // step over it without importing
    bool bake { false };      // hand it to add() once imported
    std::uint64_t key { 0 };
    node_plan plan;
};

// Worldspace says whether the node stands in world coordinates, which is all that decides it
// when the scenery names a hand-cooked heightfield of its own
node_decision examine(
    std::string const &File, std::size_t const Line, std::vector<std::string> const &Parameters,
    std::string_view const Type, glm::dvec3 const &Offset, glm::vec3 const &Rotation, bool const Worldspace );
// one triangle node, in world space, as it is about to be drawn
void add( std::uint64_t const Key, std::vector<world_vertex> const &Vertices, std::string_view const Material );
// called once the scenery has been parsed. writes the heightfield and its node table
void finish( std::string const &Sceneryfile );

} // namespace simulation::terrainbake

}
