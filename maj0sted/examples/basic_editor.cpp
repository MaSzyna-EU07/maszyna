// Minimal walkthrough: create a map project and inspect its CRS.

#include <cstdio>

#include "maj0sted/maj0sted.hpp"

using namespace maj0sted::domain;

int main() {
    std::printf("maj0sted %s\n\n", MAJ0STED_VERSION_STRING);

    const MapProject project;  // defaults to EPSG:2180
    std::printf("Map project CRS: EPSG:%d\n", project.crs().epsg());

    const CartesianPosition position{500000.0, 300000.0};
    std::printf("Sample position: x=%.1f m, y=%.1f m\n", position.x(),
                position.y());

    return 0;
}
