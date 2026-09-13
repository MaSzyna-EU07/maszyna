#pragma once

// TYMCZASOWA ZASLEPKA. Modul scalania koncow (plan_join / join_ends) nie zostal nigdy
// zacommitowany: commit 4ddfeb02 dopisal `src/editor/join.cpp` do CMakeLists i `#include` w
// planpanel.cpp, ale samych plikow nie ma ani w drzewie, ani w historii, ani w stashach. Bez
// tego naglowka i join.cpp branch sie nie kompiluje wcale. Ta zaslepka odtwarza dokladnie to
// API, ktorego uzywa panel planu, i mowi "niedostepne" - zeby reszta edytora dala sie uruchomic.
// Do zastapienia prawdziwym rozwiazaniem geometrii (luk-prosta-luk) i chirurgia dokumentu.

#include <string>
#include <vector>

#include "maj0sted/domain/geometry/segment_layout.hpp"  // Pose
#include "maj0sted/editor/model.hpp"
#include "maj0sted/editor/solution.hpp"

namespace maj0sted::editor {

/// Co stanelo miedzy dwoma koncami.
enum class JoinKind {
    None,
    Straight,
    Arc,
    ArcLineArc,
    ArcArc,
};

[[nodiscard]] const char* join_kind_name(JoinKind kind);

/// Czym wolno gietc to, co lezy miedzy koncami.
struct JoinSettings {
    double radius{300.0};
};

/// Propozycja, przed tknieciem dokumentu.
struct JoinPlan {
    bool ok{false};
    std::string why{};
    JoinKind kind{JoinKind::None};
    std::vector<Element> elements{};
    double tightest{0.0};
};

[[nodiscard]] JoinPlan plan_join(const domain::geometry::Pose& leaving,
                                 const domain::geometry::Pose& arriving,
                                 const JoinSettings& settings);

/// Co scalenie kosztowalo.
struct JoinReport {
    bool ok{false};
    std::string why{};
    JoinPlan join{};
    int fused{0};
    int turnouts{0};
    bool reversed{false};
};

[[nodiscard]] JoinReport join_ends(Document& document, const Solution& solution, TrackId a,
                                   int a_end, TrackId b, int b_end, const JoinSettings& settings);

}  // namespace maj0sted::editor
