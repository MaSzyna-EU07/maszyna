#include "check.hpp"
#include "maj0sted/maj0sted.hpp"

using namespace maj0sted::domain;

int main() {
    // A map project defaults to the Cartesian CRS EPSG:2180.
    const MapProject project;
    CHECK(project.crs() == Crs::default_crs());
    CHECK(project.crs().epsg() == 2180);

    // An explicit CRS is honoured.
    const MapProject wgs84{Crs{4326}};
    CHECK(wgs84.crs().epsg() == 4326);

    // Cartesian position value object.
    const CartesianPosition p{100.5, 200.25};
    CHECK(p.x() == 100.5);
    CHECK(p.y() == 200.25);
    CHECK(CartesianPosition{1.0, 2.0} == CartesianPosition{1.0, 2.0});
    CHECK(CartesianPosition{1.0, 2.0} != CartesianPosition{2.0, 1.0});
    CHECK(CartesianPosition{}.x() == 0.0);

    return REPORT();
}
