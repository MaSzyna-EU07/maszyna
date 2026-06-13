#pragma once

// Kontrakt danych po deserializacji w MaSzynie (maszyna-fresh).
// Odpowiedniki: scene/scenenode.h, model/vertex.h, scene/scene.h (scratch_data).
// Parsed* w tym repo = AST z tekstu; Runtime* = docelowe pola symulatora.

#include <eu07/scene/node/types.hpp>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace eu07::scene::runtime {

using Vec3 = scene::Vec3;

struct Vec4 {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float w = 1.f;
};

// model/vertex.h — world_vertex
struct WorldVertex {
    Vec3 position;
    Vec3 normal;
    double u = 0.0;
    double v = 0.0;
};

// scenenode.h — lighting_data (w pliku 0–255, w runtime float 0–1)
struct LightingData {
    Vec4 diffuse{0.8f, 0.8f, 0.8f, 1.f};
    Vec4 ambient{0.2f, 0.2f, 0.2f, 1.f};
    Vec4 specular{0.f, 0.f, 0.f, 1.f};
};

// scenenode.h — bounding_area
struct BoundingArea {
    Vec3 center;
    float radius = -1.f;
};

// scenenode.h — node_data (naglowek kazdego `node`)
struct NodeData {
    double rangeMin = 0.0;
    double rangeMax = std::numeric_limits<double>::max();
    std::string name;
    std::string type;
};

// Segment.h — segment_data (sciezka Bezier toru/trakcji)
struct SegmentPath {
    Vec3 pStart;
    double rollStart = 0.0;
    Vec3 cpOut;
    Vec3 cpIn;
    Vec3 pEnd;
    double rollEnd = 0.0;
    double radius = 0.0;
};

} // namespace eu07::scene::runtime
