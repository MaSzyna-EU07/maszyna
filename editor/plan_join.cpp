/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <string>
#include <vector>

module eu07.editor.plan_join;

// TYMCZASOWA ZASLEPKA - patrz komentarz w join.hpp.


namespace editor::plan {

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

JoinPlan plan_join(const geometry::Pose& leaving, const geometry::Pose& arriving,
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

}  // namespace editor::plan
