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
#include <unordered_map>
#include <vector>

export module eu07.scene.prefab;

export {

// Shared archetypes. A prefab is a model plus the events, memcell, animation channels and
// sounds that give it behaviour, described once and instanced many times. It exists because
// the .inc templates it replaces are not templates at all: cParser pastes text and
// substitutes (pN), so a signal that differs only by which aspects it can display needs a
// whole new file. The scenery directory holds over twelve hundred of them.
//
// What makes one prefab enough for the whole signal family is that its parameter is a list,
// not a number. Every signal template is the same shape - a state event, a light pattern and
// a speed the driver reads, for each aspect it can display, wrapped in a fixed core of
// _sem_info, _uszk and the dark state. Give the prefab the aspects and the rest follows. A
// shape signal adds animation channels to each aspect and a sound to the transition, which
// is more per aspect but not a different shape.
//
// Instances expand to scenery text and go through the ordinary node and event parsing. The
// binary format will compile them directly; until then this keeps one description of the
// signal family instead of one per variant.
namespace scene {

// one submodel driven by an aspect. kind is the word the animation event takes, translate or
// rotate; target is where the submodel goes, in metres or degrees; speed is how fast
struct prefab_animation {
    std::string kind;
    std::string submodel;
    std::string x{ "0" }, y{ "0" }, z{ "0" };
    std::string speed{ "0" };
};

// one thing the signal can show. lights is the pattern the model's light channels take, one
// entry per channel, -1 leaving a channel alone; velocity is what the driver reads from the
// memcell, -1 meaning no limit
struct prefab_aspect {
    std::string name;
    std::vector<int> lights;
    std::string lightdelay{ "0.0" };
    std::string velocity{ "-1" };
    std::string velocitynext{ "-1" };
    std::vector<prefab_animation> animations;
};

// a second model hung on the instance: the lattice mast a shape signal stands on, its
// distant lod, the plate carrying the signal's symbol
struct prefab_attachment {
    std::string rangemax{ "50" };
    std::string rangemin{ "0" };
    std::string model;
    // the instance's own texture when "instance", otherwise a literal, "none" included
    std::string texture{ "none" };
};

struct prefab_definition {
    std::string name;
    std::string model;
    std::string modelangle{ "0" };
    std::vector<prefab_attachment> attachments;
    // how many light channels the model has, which is how many values an aspect's pattern holds
    std::size_t chambers{ 0 };
    std::vector<prefab_aspect> aspects;
    // aspect the signal shows before anything sets it, by name; empty lights every channel off
    std::string initial;
    // the node's own starting pattern, when it is not simply the initial aspect: a shape
    // signal starts with channel zero on automatic, which is not a thing any aspect says
    std::vector<int> initiallights;
    // played when the signal changes aspect. shape signals have one, light signals do not
    std::string sound;
    // emit the _dzienny / _nocny / _auto events which switch channel zero between day,
    // night and automatic lighting. only models with that channel want them
    bool lightmodes{ false };
};

// arguments one instance supplies
struct prefab_placement {
    std::string name;
    double x{ 0.0 }, y{ 0.0 }, z{ 0.0 };
    double yaw{ 0.0 };
    std::string texture;
};

class prefab_registry {

public:
    // looks the definition up, loading <Name>.pfb from the scenery directory on first use.
    // returns nullptr and logs if there is no such prefab
    prefab_definition const * definition( std::string const &Name );
    // scenery text realising the instance: the model node, its attachments, memcell, sound
    // and events. empty if the definition is missing
    std::string expand( std::string const &Name, prefab_placement const &Placement );

private:
    bool load( std::string const &Name );

    std::unordered_map<std::string, prefab_definition> m_definitions;
    // names which failed to load, so a scenery full of them complains once each
    std::unordered_map<std::string, bool> m_missing;
};

} // namespace scene

inline scene::prefab_registry Prefabs;

}
