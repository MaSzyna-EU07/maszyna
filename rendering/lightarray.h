#pragma once

#include "utilities/Classes.h"

// collection of virtual light sources present in the scene
// used by the renderer to determine most suitable placement for actual light sources during render
struct light_array {

public:
// types
    struct light_record {

        light_record( TDynamicObject const *Owner, int const Lightindex) :
                                      owner(Owner),    index(Lightindex)
        {};

        TDynamicObject const *owner; // the object in world which 'carries' the light
        int index{ -1 }; // 0: front lights, 1: rear lights
        glm::dvec3 position; // position of the light in 3d scene
        glm::vec3 direction; // direction of the light in 3d scene
        glm::vec3 color{ 255.0f / 255.0f, 241.0f / 255.0f, 224.0f / 255.0f }; // color of the light, default is halogen light
        float intensity{ 0.0f }; // (combined) intensity of the light(s)
        int count{ 0 }; // number (or pattern) of active light(s)
        glm::vec3 state{ 0.f }; // state of individual lights
    };

// methods
    // adds records for lights of specified owner to the collection
    void
        insert( TDynamicObject const *Owner );
    // removes records for lights of specified owner from the collection
    void
        remove( TDynamicObject const *Owner );
    // updates records in the collection
    void
        update();

// types
    typedef std::vector<light_record> lightrecord_array;

    struct free_light_record {
        glm::dvec3 position;
        glm::vec3  direction { 0.f, -1.f, 0.f };
        glm::vec3  color     { 1.f,  1.f,  1.f };
        float intensity  { 1.0f };
        float range      { 25.0f };
        float inner_cutoff { 0.9659f };  // cos(15°)
        float outer_cutoff { 0.9063f };  // cos(25°)
    };

// members
    lightrecord_array data;
    std::vector<free_light_record> free_lights;
};
