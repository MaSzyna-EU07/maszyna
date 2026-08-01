#pragma once

#include <string>
#include <vector>

#include "maj0sted/editor/editor.hpp"
#include "maj0sted/editor/ribbon.hpp"
#include "maj0sted/editor/solve.hpp"

namespace maj0sted::app {

struct BasketDraft {
    bool active{false};
    double x0{0.0};
    double y0{0.0};
    double az0{0.0};
    int dir{1};
    double entry_t{0.0};
    double exit_t{0.0};
    std::vector<maj0sted::editor::CompoundArcSpec> arcs;
    double preview_next_r{300.0};
    int attach_str{-1};
    int attach_end{-1};
    int reedit_gap{-1};
    maj0sted::editor::GapFit reedit_backup{};
};

[[nodiscard]] maj0sted::editor::RibbonRequest draft_to_ribbon(const BasketDraft& draft);

[[nodiscard]] std::vector<maj0sted::editor::PlanPolyline> preview_basket_draft(
    BasketDraft& draft, double fallback_next_r = 300.0);

void start_basket_draft_from_end(BasketDraft& draft,
                                 const maj0sted::editor::StraightSpec& straight,
                                 int straight_index, int end, double default_r = 300.0);

[[nodiscard]] bool reopen_basket_draft(BasketDraft& draft,
                                       maj0sted::editor::NiweletaSpec& niweleta,
                                       std::size_t gap,
                                       const maj0sted::editor::NiweletaPolys* solved);

void discard_basket_draft(BasketDraft& draft, maj0sted::editor::NiweletaSpec* niweleta);

[[nodiscard]] bool anchor_basket_draft(BasketDraft& draft,
                                       maj0sted::editor::NiweletaSpec& niweleta,
                                       std::size_t gap);

struct FinishBasketDraftResult {
    bool ok{false};
    std::string status;
};

[[nodiscard]] FinishBasketDraftResult finish_basket_draft(
    BasketDraft& draft, maj0sted::editor::NiweletaSpec& niweleta,
    std::vector<maj0sted::editor::NiweletaSpec>& all_niwelety, int niweleta_index,
    double finish_straight_m);

}  // namespace maj0sted::app
