#pragma once

#include <eu07/scene/node/types.hpp>

#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace eu07::geo {

template <int CellSizeM>
class SpatialGrid {
public:
    struct ItemBounds {
        double minX = 0.0;
        double maxX = 0.0;
        double minZ = 0.0;
        double maxZ = 0.0;
    };

private:
    struct CellKey {
        int x = 0;
        int z = 0;

        [[nodiscard]] bool operator==(const CellKey& other) const noexcept {
            return x == other.x && z == other.z;
        }
    };

    struct CellKeyHash {
        [[nodiscard]] std::size_t operator()(const CellKey& key) const noexcept {
            return static_cast<std::size_t>(key.x * 73856093) ^
                   static_cast<std::size_t>(key.z * 19349663);
        }
    };

    std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> grid_;

    [[nodiscard]] static int cellIndex(const double value) noexcept {
        return static_cast<int>(std::floor(value / static_cast<double>(CellSizeM)));
    }

public:
    void clear() { grid_.clear(); }

    void insert(const std::size_t itemIndex, const ItemBounds& bounds) {
        const int minIx = cellIndex(bounds.minX);
        const int maxIx = cellIndex(bounds.maxX);
        const int minIz = cellIndex(bounds.minZ);
        const int maxIz = cellIndex(bounds.maxZ);
        for (int x = minIx; x <= maxIx; ++x) {
            for (int z = minIz; z <= maxIz; ++z) {
                grid_[{x, z}].push_back(itemIndex);
            }
        }
    }

    void query(
        const scene::Vec3& point,
        const double maxDist,
        std::vector<std::size_t>& out) const {
        out.clear();
        const int cx = cellIndex(point.x);
        const int cz = cellIndex(point.z);
        int range = static_cast<int>(std::ceil(maxDist / static_cast<double>(CellSizeM)));
        if (range < 1) {
            range = 1;
        }

        for (int dz = -range; dz <= range; ++dz) {
            for (int dx = -range; dx <= range; ++dx) {
                const auto it = grid_.find({cx + dx, cz + dz});
                if (it == grid_.end()) {
                    continue;
                }
                for (const std::size_t idx : it->second) {
                    out.push_back(idx);
                }
            }
        }
    }
};

} // namespace eu07::geo
