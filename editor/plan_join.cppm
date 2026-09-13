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

export module eu07.editor.plan_join;
export import eu07.editor.plan_segment;
export import eu07.editor.plan_model;
export import eu07.editor.plan_solution;

export {

// TYMCZASOWA ZASLEPKA. Modul scalania koncow (plan_join / join_ends) nie zostal nigdy
// zacommitowany: commit 4ddfeb02 dopisal `src/editor/join.cpp` do CMakeLists i `#include` w
// planpanel.cpp, ale samych plikow nie ma ani w drzewie, ani w historii, ani w stashach. Bez
// tego naglowka i join.cpp branch sie nie kompiluje wcale. Ta zaslepka odtwarza dokladnie to
// API, ktorego uzywa panel planu, i mowi "niedostepne" - zeby reszta edytora dala sie uruchomic.
// Do zastapienia prawdziwym rozwiazaniem geometrii (luk-prosta-luk) i chirurgia dokumentu.



namespace editor::plan {

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

[[nodiscard]] JoinPlan plan_join(const geometry::Pose& leaving,
                                 const geometry::Pose& arriving,
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

}  // namespace editor::plan

}  // export
