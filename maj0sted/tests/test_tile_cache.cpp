#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "check.hpp"
#include "maj0sted/editor/tile_cache.hpp"

using namespace maj0sted::editor;

namespace {

// --- tile grid geometry --------------------------------------------------

void grid_is_fixed_100m_cells() {
    // One fixed grid: every cell is 100 m × 100 m regardless of the zoom arg.
    CHECK(std::abs(TileGrid::kCellMetres - 100.0) < 1e-9);
    CHECK(std::abs(TileGrid::tile_span() - 100.0) < 1e-6);
    CHECK(std::abs(TileGrid::tile_span(14) - 100.0) < 1e-6);  // zoom ignored
    // Resolution is constant: 100 m over 1024 px.
    CHECK(std::abs(TileGrid::resolution() - 100.0 / 1024.0) < 1e-9);
    CHECK(std::abs(TileGrid::resolution(7) - 100.0 / 1024.0) < 1e-9);
}

void tile_at_and_bbox_are_consistent() {
    const double x = 500123.0;  // somewhere in EPSG:2180 (Poland)
    const double y = 300456.0;
    const TileKey key = TileGrid::tile_at(x, y);
    const TileBBox box = TileGrid::bbox(key);

    // Every tile key sits on the single fixed level.
    CHECK(key.zoom == TileGrid::kLevel);

    // The point must fall inside its own cell's ground box.
    CHECK(box.min_x <= x && x < box.max_x);
    CHECK(box.min_y <= y && y < box.max_y);

    // The box edge length must equal the fixed 100 m cell span.
    CHECK(std::abs((box.max_x - box.min_x) - 100.0) < 1e-6);
    CHECK(std::abs((box.max_y - box.min_y) - 100.0) < 1e-6);

    // Neighbouring cells are exactly one span (100 m) apart.
    const TileBBox right = TileGrid::bbox(TileKey{TileGrid::kLevel, key.x + 1, key.y});
    CHECK(std::abs(right.min_x - box.max_x) < 1e-6);
}

void zoom_for_resolution_is_single_level() {
    // There is no pyramid: any target resolution maps to the one fixed level.
    for (double mpp : {50.0, 8.0, 1.0, 0.25}) {
        CHECK(TileGrid::zoom_for_resolution(mpp) == TileGrid::kLevel);
    }
}

void tiles_for_view_covers_and_caps() {
    const double span = 100.0;  // fixed cell
    // A view roughly 3x2 cells.
    TileBBox view{500000.0, 300000.0, 500000.0 + 2.5 * span, 300000.0 + 1.5 * span};
    const auto tiles = TileGrid::tiles_for_view(view);
    CHECK(!tiles.empty());
    // Every corner cell of the view must be present.
    const TileKey lo = TileGrid::tile_at(view.min_x, view.min_y);
    const TileKey hi = TileGrid::tile_at(view.max_x, view.max_y);
    bool has_lo = false, has_hi = false;
    for (const auto& t : tiles) {
        if (t == lo) has_lo = true;
        if (t == hi) has_hi = true;
    }
    CHECK(has_lo && has_hi);

    // The cap is honoured.
    TileBBox huge{0.0, 0.0, 1e9, 1e9};
    const auto capped = TileGrid::tiles_for_view(huge, TileGrid::kLevel, 32);
    CHECK(static_cast<int>(capped.size()) <= 32);
    const TileKey centre = TileGrid::tile_at(5e8, 5e8);
    bool has_centre = false;
    for (const auto& tile : capped) {
        if (tile == centre) has_centre = true;
    }
    CHECK(has_centre);  // cap is centred, not taken from a remote corner

    const double infinity = std::numeric_limits<double>::infinity();
    CHECK(TileGrid::tiles_for_view({0.0, 0.0, infinity, 1.0}).empty());
}

// --- WMS URL -------------------------------------------------------------

void getmap_url_is_wms_1_1_1_epsg2180() {
    WmsConfig cfg = WmsConfig::geoportal_ortho();
    const TileBBox box{100.0, 200.0, 300.0, 400.0};
    const std::string url = wms_getmap_url(cfg, box);
    CHECK(url.find("SERVICE=WMS") != std::string::npos);
    CHECK(url.find("VERSION=1.1.1") != std::string::npos);
    CHECK(url.find("REQUEST=GetMap") != std::string::npos);
    CHECK(url.find("SRS=EPSG:2180") != std::string::npos);
    // BBOX in minx,miny,maxx,maxy order.
    CHECK(url.find("BBOX=100,200,300,400") != std::string::npos);
    CHECK(url.find("WIDTH=1024") != std::string::npos);
}

// --- cache + injected fetch ---------------------------------------------

void miss_fetches_then_hit_is_served_from_cache() {
    TileService service;
    int fetch_calls = 0;
    // Synchronous injected fetcher that returns three canned bytes.
    service.set_fetcher([&](const std::string& url,
                            TileService::DoneFn done) {
        ++fetch_calls;
        CHECK(url.find("BBOX=") != std::string::npos);
        done(true, std::vector<std::uint8_t>{1, 2, 3}, "image/jpeg");
    });

    int ready_events = 0;
    TileKey last_ready{};
    service.set_on_ready([&](TileKey k) {
        ++ready_events;
        last_ready = k;
    });

    const TileKey key{12, 3, 4};
    CHECK(!service.has(key));

    // First request: a miss, so exactly one fetch and the tile becomes Ready.
    const TileStatus first = service.request(key);
    CHECK(first == TileStatus::Ready);  // fetcher was synchronous
    CHECK(fetch_calls == 1);
    CHECK(ready_events == 1);
    CHECK(last_ready == key);
    CHECK(service.has(key));

    const TileImage* image = service.peek(key);
    CHECK(image != nullptr);
    CHECK(image->bytes.size() == 3);
    CHECK(image->content_type == "image/jpeg");

    // Second request for the same tile: a hit, no extra fetch.
    const TileStatus second = service.request(key);
    CHECK(second == TileStatus::Ready);
    CHECK(fetch_calls == 1);  // unchanged: served from cache
    CHECK(service.size() == 1);
}

void failed_fetch_is_retried_until_success() {
    // A failure is not terminal: the service retries until a tile arrives. With
    // a synchronous injected fetcher the retries unwind inline, so a fetcher that
    // fails twice then succeeds must end Ready after exactly three attempts.
    TileService service;
    int calls = 0;
    service.set_fetcher([&](const std::string&, TileService::DoneFn done) {
        ++calls;
        if (calls < 3) {
            done(false, {}, {});  // fail
        } else {
            done(true, std::vector<std::uint8_t>{7}, "image/jpeg");  // then succeed
        }
    });

    const TileKey key{10, 1, 1};
    const TileStatus status = service.request(key);
    CHECK(status == TileStatus::Ready);
    CHECK(calls == 3);
    CHECK(service.has(key));

    const TileImage* image = service.peek(key);
    CHECK(image != nullptr);
    CHECK(image->bytes.size() == 1);
}

void retry_gives_up_after_ten_attempts() {
    // A tile that never loads is retried at most 10 times, then marked Failed.
    // With a synchronous always-failing fetcher that is 1 initial + 10 retries.
    TileService service;
    int calls = 0;
    service.set_fetcher([&](const std::string&, TileService::DoneFn done) {
        ++calls;
        done(false, {}, {});
    });

    const TileKey key{2, 2, 2};
    CHECK(service.request(key) == TileStatus::Failed);
    CHECK(calls == 11);  // initial + kMaxRetries (10)
    CHECK(!service.has(key));
    CHECK(service.peek(key) == nullptr);
}

void store_seeds_cache_directly() {
    TileService service;
    const TileKey key{9, 2, 2};
    service.store(key, std::vector<std::uint8_t>{9, 9}, "image/png");
    CHECK(service.has(key));
    // A request now is a pure hit even with no fetcher installed.
    CHECK(service.request(key) == TileStatus::Ready);
}

}  // namespace

int main() {
    RUN(grid_is_fixed_100m_cells);
    RUN(tile_at_and_bbox_are_consistent);
    RUN(zoom_for_resolution_is_single_level);
    RUN(tiles_for_view_covers_and_caps);
    RUN(getmap_url_is_wms_1_1_1_epsg2180);
    RUN(miss_fetches_then_hit_is_served_from_cache);
    RUN(failed_fetch_is_retried_until_success);
    RUN(retry_gives_up_after_ten_attempts);
    RUN(store_seeds_cache_directly);
    return REPORT();
}
