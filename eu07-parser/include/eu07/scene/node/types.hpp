#pragma once

#include <optional>
#include <string>
#include <vector>

namespace eu07::scene {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct MeshVertex {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    double nz = 0.0;
    double u = 0.0;
    double v = 0.0;
};

struct MaterialRgb {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
};

struct NodeMaterial {
    MaterialRgb ambient;
    MaterialRgb diffuse;
    MaterialRgb specular;
};

} // namespace eu07::scene
