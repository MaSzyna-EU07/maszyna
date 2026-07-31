#include <variant>

#include "check.hpp"
#include "maj0sted/maj0sted.hpp"

using namespace maj0sted::domain;

namespace {

void plan_and_profile_element_lengths() {
    const PlanElement straight =
        Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{100.0, 0.0}};
    const PlanElement arc = CircularArc{Radius::from_metres(300.0),
                                        TurnDirection::Left,
                                        Length::from_metres(50.0)};
    const PlanElement transition = TransitionCurve{
        Length::from_metres(30.0), std::nullopt, Radius::from_metres(300.0),
        TurnDirection::Left};
    CHECK(length_of(straight).value().metres() == 100.0);
    CHECK(length_of(arc).value().metres() == 50.0);
    CHECK(length_of(transition).value().metres() == 30.0);

    const ProfileElement grade = Grade{Length::from_metres(120.0), 5.0};
    const ProfileElement vcurve = VerticalCurve{
        Length::from_metres(60.0), Radius::from_metres(4000.0),
        VerticalCurvature::Crest};
    CHECK(length_of(grade).metres() == 120.0);
    CHECK(length_of(vcurve).metres() == 60.0);
}

void value_objects_guard_invariants() {
    CHECK_THROWS(Length::from_metres(-1.0));
    CHECK_THROWS(Radius::from_metres(0.0));
    CHECK_THROWS(Radius::from_metres(-5.0));
}

void niweleta_aggregates_plan_and_profile() {
    Niweleta n{NiweletaId{1}, "Odcinek 1"};
    CHECK(n.name() == "Odcinek 1");
    CHECK(n.plan().empty());
    CHECK(n.profile().empty());

    // Plan: prosta -> klotoida -> łuk -> klotoida -> prosta  (razem 180 m)
    n.add_plan_element(
        Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    n.add_plan_element(TransitionCurve{Length::from_metres(30.0), std::nullopt,
                                       Radius::from_metres(300.0),
                                       TurnDirection::Right});
    n.add_plan_element(CircularArc{Radius::from_metres(300.0),
                                   TurnDirection::Right,
                                   Length::from_metres(20.0)});
    n.add_plan_element(TransitionCurve{Length::from_metres(30.0),
                                       Radius::from_metres(300.0), std::nullopt,
                                       TurnDirection::Right});
    n.add_plan_element(
        Straight{CartesianPosition{100.0, 0.0}, CartesianPosition{150.0, 0.0}});

    // Profil: pochylenie -> łuk pionowy -> pochylenie  (razem 180 m)
    n.add_profile_element(Grade{Length::from_metres(80.0), 3.0});
    n.add_profile_element(VerticalCurve{Length::from_metres(40.0),
                                        Radius::from_metres(5000.0),
                                        VerticalCurvature::Sag});
    n.add_profile_element(Grade{Length::from_metres(60.0), -2.0});

    CHECK(n.plan().size() == 5);
    CHECK(n.profile().size() == 3);
    CHECK(n.plan_length().metres() == 180.0);
    CHECK(n.profile_length().metres() == 180.0);
    CHECK(n.lengths_consistent());
}

void inconsistent_lengths_are_detected() {
    Niweleta n{NiweletaId{2}};
    n.add_plan_element(
        Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{100.0, 0.0}});
    n.add_profile_element(Grade{Length::from_metres(90.0), 0.0});
    CHECK(!n.lengths_consistent());
    CHECK(n.plan_length().metres() == 100.0);
    CHECK(n.profile_length().metres() == 90.0);
}

void niweleta_assigns_straight_ids() {
    Niweleta n{NiweletaId{3}};
    // add_straight mints and returns fresh ids.
    const StraightId a =
        n.add_straight(CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0});
    const StraightId b =
        n.add_straight(CartesianPosition{50.0, 0.0}, CartesianPosition{100.0, 0.0});
    CHECK(!a.is_null());
    CHECK(!b.is_null());
    CHECK(a != b);

    // An anonymous straight added via add_plan_element is identified too.
    Niweleta m{NiweletaId{4}};
    m.add_plan_element(
        Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    const auto& element = m.plan().elements().front();
    CHECK(!std::get<Straight>(element).id().is_null());
}

}  // namespace

int main() {
    RUN(plan_and_profile_element_lengths);
    RUN(value_objects_guard_invariants);
    RUN(niweleta_aggregates_plan_and_profile);
    RUN(inconsistent_lengths_are_detected);
    RUN(niweleta_assigns_straight_ids);
    return REPORT();
}
