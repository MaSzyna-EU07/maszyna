// TYMCZASOWA ZASLEPKA - patrz komentarz w join.hpp.

#include "maj0sted/editor/join.hpp"

namespace maj0sted::editor {

namespace {
constexpr const char* kMissing = "scalanie koncow niedostepne: brak modulu join";
}  // namespace

const char* join_kind_name(JoinKind kind) {
    switch (kind) {
        case JoinKind::Straight:
            return "prosta";
        case JoinKind::Arc:
            return "luk";
        case JoinKind::ArcLineArc:
            return "luk-prosta-luk";
        case JoinKind::ArcArc:
            return "luk-luk";
        case JoinKind::None:
            break;
    }
    return "nic";
}

JoinPlan plan_join(const domain::geometry::Pose& leaving, const domain::geometry::Pose& arriving,
                   const JoinSettings& settings) {
    (void)leaving;
    (void)arriving;
    (void)settings;
    JoinPlan plan;
    plan.why = kMissing;
    return plan;
}

JoinReport join_ends(Document& document, const Solution& solution, TrackId a, int a_end, TrackId b,
                     int b_end, const JoinSettings& settings) {
    (void)document;
    (void)solution;
    (void)a;
    (void)a_end;
    (void)b;
    (void)b_end;
    (void)settings;
    JoinReport report;
    report.why = kMissing;
    return report;
}

}  // namespace maj0sted::editor
