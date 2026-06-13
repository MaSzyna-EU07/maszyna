#pragma once

// simulation/simulationstateserializer.cpp — transform() + basic_region::insert

#include <eu07/scene/node/types.hpp>
#include <eu07/scene/runtime/basic_node.hpp>
#include <eu07/scene/runtime/scratch.hpp>
#include <eu07/scene/runtime/types.hpp>

#include <cmath>
#include <vector>

namespace eu07::scene::runtime {

// Punkt modelu / memcell / sound / eventlauncher / tractionpowersource:
// rotateY → scale (iloczyn osiowy) → offset.
[[nodiscard]] inline Vec3 transformPoint(Vec3 location, const RuntimeScratchpad& scratch) {
    if (scratch.location.rotation.y != 0.0) {
        const double rad = scratch.location.rotation.y * (3.14159265358979323846 / 180.0);
        const double c = std::cos(rad);
        const double s = std::sin(rad);
        const double x = location.x * c + location.z * s;
        const double z = -location.x * s + location.z * c;
        location.x = x;
        location.z = z;
    }

    if (!scratch.location.scaleStack.empty()) {
        const Vec3& sc = scratch.location.scaleStack.back();
        location.x *= sc.x;
        location.y *= sc.y;
        location.z *= sc.z;
    }

    if (!scratch.location.offsetStack.empty()) {
        const Vec3& off = scratch.location.offsetStack.back();
        location.x += off.x;
        location.y += off.y;
        location.z += off.z;
    }

    return location;
}

// Shape/lines: rotacja Z → X → Y na wierzcholkach, potem offset. Bez scale na geometrii.
inline void transformShapeVertices(std::vector<WorldVertex>& vertices, const RuntimeScratchpad& scratch) {
    if (scratch.location.rotation.x != 0.0 || scratch.location.rotation.y != 0.0 ||
        scratch.location.rotation.z != 0.0) {
        const double rx = scratch.location.rotation.x * (3.14159265358979323846 / 180.0);
        const double ry = scratch.location.rotation.y * (3.14159265358979323846 / 180.0);
        const double rz = scratch.location.rotation.z * (3.14159265358979323846 / 180.0);

        auto rotateZ = [&](Vec3& v) {
            const double c = std::cos(rz);
            const double s = std::sin(rz);
            const double x = v.x * c - v.y * s;
            const double y = v.x * s + v.y * c;
            v.x = x;
            v.y = y;
        };
        auto rotateX = [&](Vec3& v) {
            const double c = std::cos(rx);
            const double s = std::sin(rx);
            const double y = v.y * c - v.z * s;
            const double z = v.y * s + v.z * c;
            v.y = y;
            v.z = z;
        };
        auto rotateY = [&](Vec3& v) {
            const double c = std::cos(ry);
            const double s = std::sin(ry);
            const double x = v.x * c + v.z * s;
            const double z = -v.x * s + v.z * c;
            v.x = x;
            v.z = z;
        };

        for (WorldVertex& vtx : vertices) {
            rotateZ(vtx.position);
            rotateX(vtx.position);
            rotateY(vtx.position);
            Vec3 n{vtx.normal.x, vtx.normal.y, vtx.normal.z};
            rotateZ(n);
            rotateX(n);
            rotateY(n);
            vtx.normal = n;
        }
    }

    if (!scratch.location.offsetStack.empty()) {
        const Vec3& off = scratch.location.offsetStack.back();
        for (WorldVertex& vtx : vertices) {
            vtx.position.x += off.x;
            vtx.position.y += off.y;
            vtx.position.z += off.z;
        }
    }
}

[[nodiscard]] inline RuntimeScratchpad scratchFromTransform(const TransformContext& transform) {
    RuntimeScratchpad scratch;
    scratch.location.offsetStack = transform.originStack;
    scratch.location.scaleStack = transform.scaleStack;
    scratch.location.rotation = transform.rotation;
    return scratch;
}

[[nodiscard]] inline Vec3 inverseTransformPoint(Vec3 location, const TransformContext& transform) {
    const RuntimeScratchpad scratch = scratchFromTransform(transform);

    if (!scratch.location.offsetStack.empty()) {
        const Vec3& off = scratch.location.offsetStack.back();
        location.x -= off.x;
        location.y -= off.y;
        location.z -= off.z;
    }

    if (!scratch.location.scaleStack.empty()) {
        const Vec3& sc = scratch.location.scaleStack.back();
        if (sc.x != 0.0) {
            location.x /= sc.x;
        }
        if (sc.y != 0.0) {
            location.y /= sc.y;
        }
        if (sc.z != 0.0) {
            location.z /= sc.z;
        }
    }

    if (scratch.location.rotation.y != 0.0) {
        const double rad = -scratch.location.rotation.y * (3.14159265358979323846 / 180.0);
        const double c = std::cos(rad);
        const double s = std::sin(rad);
        const double x = location.x * c + location.z * s;
        const double z = -location.x * s + location.z * c;
        location.x = x;
        location.z = z;
    }

    return location;
}

inline void inverseTransformShapeVertices(
    std::vector<WorldVertex>& vertices,
    const TransformContext& transform) {
    const RuntimeScratchpad scratch = scratchFromTransform(transform);

    if (!scratch.location.offsetStack.empty()) {
        const Vec3& off = scratch.location.offsetStack.back();
        for (WorldVertex& vtx : vertices) {
            vtx.position.x -= off.x;
            vtx.position.y -= off.y;
            vtx.position.z -= off.z;
        }
    }

    if (scratch.location.rotation.x != 0.0 || scratch.location.rotation.y != 0.0 ||
        scratch.location.rotation.z != 0.0) {
        const double rx = -scratch.location.rotation.x * (3.14159265358979323846 / 180.0);
        const double ry = -scratch.location.rotation.y * (3.14159265358979323846 / 180.0);
        const double rz = -scratch.location.rotation.z * (3.14159265358979323846 / 180.0);

        auto rotateY = [&](Vec3& v) {
            const double c = std::cos(ry);
            const double s = std::sin(ry);
            const double x = v.x * c + v.z * s;
            const double z = -v.x * s + v.z * c;
            v.x = x;
            v.z = z;
        };
        auto rotateX = [&](Vec3& v) {
            const double c = std::cos(rx);
            const double s = std::sin(rx);
            const double y = v.y * c - v.z * s;
            const double z = v.y * s + v.z * c;
            v.y = y;
            v.z = z;
        };
        auto rotateZ = [&](Vec3& v) {
            const double c = std::cos(rz);
            const double s = std::sin(rz);
            const double x = v.x * c - v.y * s;
            const double y = v.x * s + v.y * c;
            v.x = x;
            v.y = y;
        };

        for (WorldVertex& vtx : vertices) {
            rotateY(vtx.position);
            rotateX(vtx.position);
            rotateZ(vtx.position);
            Vec3 n{vtx.normal.x, vtx.normal.y, vtx.normal.z};
            rotateY(n);
            rotateX(n);
            rotateZ(n);
            vtx.normal = n;
        }
    }
}

[[nodiscard]] inline Vec3 subtractOriginOffset(const Vec3& world, const TransformContext& transform) {
    Vec3 local = world;
    if (!transform.originStack.empty()) {
        const Vec3& off = transform.originStack.back();
        local.x -= off.x;
        local.y -= off.y;
        local.z -= off.z;
    }
    return local;
}

} // namespace eu07::scene::runtime
