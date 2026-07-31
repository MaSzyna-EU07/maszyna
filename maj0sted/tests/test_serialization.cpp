#include <variant>

#include "check.hpp"
#include "maj0sted/maj0sted.hpp"

using namespace maj0sted::domain;

namespace {

MapProject make_project() {
    MapProject project{Crs{2180}};

    Niweleta a{NiweletaId{5}, "Odcinek A"};
    const StraightId s1 =
        a.add_straight(CartesianPosition{0.0, 0.0}, CartesianPosition{100.0, 0.0});
    (void)s1;
    a.add_plan_element(TransitionCurve{Length::from_metres(30.0), std::nullopt,
                                       Radius::from_metres(300.0), TurnDirection::Right});
    a.add_plan_element(CircularArc{Radius::from_metres(300.0), TurnDirection::Right,
                                   Length::from_metres(40.0)});
    a.add_profile_element(Grade{Length::from_metres(80.0), 3.0});
    a.add_profile_element(VerticalCurve{Length::from_metres(40.0),
                                        Radius::from_metres(5000.0),
                                        VerticalCurvature::Sag});
    project.add_niweleta(std::move(a));

    // A second niweleta with an azimuth break, to exercise joints.
    Niweleta b{NiweletaId{9}, "Odcinek B z załamaniem"};
    b.add_plan_element(
        Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    b.add_plan_element(Straight{CartesianPosition{50.0, 0.0}, CartesianPosition{50.0, 50.0}},
                       JointContinuity::AzimuthBreak);
    project.add_niweleta(std::move(b));

    return project;
}

void round_trip_is_stable() {
    const MapProject project = make_project();
    const std::string text = maj0sted::io::serialize(project);
    const MapProject loaded = maj0sted::io::deserialize(text);

    // Re-serializing the loaded project yields byte-identical text.
    CHECK(maj0sted::io::serialize(loaded) == text);

    CHECK(loaded.crs().epsg() == 2180);
    CHECK(loaded.niwelety().size() == 2);
}

void niweleta_and_geometry_survive() {
    const MapProject loaded = maj0sted::io::deserialize(maj0sted::io::serialize(make_project()));

    const Niweleta& a = loaded.niwelety()[0];
    CHECK(a.id() == NiweletaId{5});
    CHECK(a.name() == "Odcinek A");
    CHECK(a.plan().size() == 3);
    CHECK(a.profile().size() == 2);

    // Straight kept its id and geometry.
    const auto& straight = std::get<Straight>(a.plan().elements()[0]);
    CHECK(straight.id() == StraightId{1});
    CHECK(straight.length().metres() == 100.0);

    // Arc kept radius, direction and length.
    const auto& arc = std::get<CircularArc>(a.plan().elements()[2]);
    CHECK(arc.radius().metres() == 300.0);
    CHECK(arc.direction() == TurnDirection::Right);
    CHECK(arc.length().value().metres() == 40.0);

    // Profile survived.
    const auto& grade = std::get<Grade>(a.profile().elements()[0]);
    CHECK(grade.slope_permille() == 3.0);

    // Name with spaces and the azimuth-break joint survived on the 2nd niweleta.
    const Niweleta& b = loaded.niwelety()[1];
    CHECK(b.name() == "Odcinek B z załamaniem");
    CHECK(b.plan().joint(0) == JointContinuity::AzimuthBreak);
}

void malformed_input_throws() {
    CHECK_THROWS(maj0sted::io::deserialize("not a maj0sted file"));
    CHECK_THROWS(maj0sted::io::deserialize("maj0sted 1\ncrs 2180\nniwelety 3\n"));
}

}  // namespace

int main() {
    RUN(round_trip_is_stable);
    RUN(niweleta_and_geometry_survive);
    RUN(malformed_input_throws);
    return REPORT();
}
