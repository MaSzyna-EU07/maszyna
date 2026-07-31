#include "maj0sted/web/tile_cache.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

#ifdef __EMSCRIPTEN__
#include <cstring>

#include <emscripten/emscripten.h>
#include <emscripten/fetch.h>
#endif

namespace maj0sted::web {

namespace {

// Shortest round-trippable, locale-independent representation of a double —
// same convention as maj0sted::io so URLs are stable and precise.
std::string num(double value) {
    char buffer[64];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
    return std::string(buffer, result.ptr);
}

// Maximum number of retry attempts per tile after the initial fetch fails.
constexpr int kMaxRetries = 10;

}  // namespace

// --- TileGrid ------------------------------------------------------------

double TileGrid::resolution(int /*zoom*/) noexcept {
    return kCellMetres / static_cast<double>(kTilePixels);  // constant, single grid
}

double TileGrid::tile_span(int /*zoom*/) noexcept {
    return kCellMetres;  // every cell is 100 m × 100 m
}

TileKey TileGrid::tile_at(double x, double y, int /*zoom*/) noexcept {
    const auto safe_index = [](double coordinate, double origin) {
        const double value = std::floor((coordinate - origin) / kCellMetres);
        if (!std::isfinite(value)) return 0;
        if (value <= static_cast<double>(std::numeric_limits<int>::min())) {
            return std::numeric_limits<int>::min();
        }
        if (value >= static_cast<double>(std::numeric_limits<int>::max())) {
            return std::numeric_limits<int>::max();
        }
        return static_cast<int>(value);
    };
    return TileKey{kLevel, safe_index(x, kOriginX), safe_index(y, kOriginY)};
}

TileBBox TileGrid::bbox(const TileKey& key) noexcept {
    const double min_x = kOriginX + static_cast<double>(key.x) * kCellMetres;
    const double min_y = kOriginY + static_cast<double>(key.y) * kCellMetres;
    return TileBBox{min_x, min_y, min_x + kCellMetres, min_y + kCellMetres};
}

int TileGrid::zoom_for_resolution(double /*metres_per_pixel*/) noexcept {
    return kLevel;  // one fixed 100 m grid, no pyramid
}

std::vector<TileKey> TileGrid::tiles_for_view(const TileBBox& view, int /*zoom*/,
                                              int max_tiles) {
    std::vector<TileKey> tiles;
    if (max_tiles <= 0 || !std::isfinite(view.min_x) ||
        !std::isfinite(view.min_y) || !std::isfinite(view.max_x) ||
        !std::isfinite(view.max_y)) {
        return tiles;
    }
    const double min_x = std::min(view.min_x, view.max_x);
    const double max_x = std::max(view.min_x, view.max_x);
    const double min_y = std::min(view.min_y, view.max_y);
    const double max_y = std::max(view.min_y, view.max_y);

    const TileKey lo = tile_at(min_x, min_y);
    const TileKey hi = tile_at(max_x, max_y);

    using Wide = std::int64_t;
    const Wide width = static_cast<Wide>(hi.x) - lo.x + 1;
    const Wide height = static_cast<Wide>(hi.y) - lo.y + 1;
    if (width <= 0 || height <= 0) return tiles;

    Wide columns = width;
    Wide rows = height;
    const Wide cap = max_tiles;
    if (height > cap || width > cap / height) {
        // A fixed 100 m grid can contain millions of cells when zoomed far out.
        // Return a small, viewport-centred window instead of walking from a
        // remote corner or allocating/requesting the whole range.
        const long double aspect =
            static_cast<long double>(width) / static_cast<long double>(height);
        columns = std::clamp<Wide>(
            static_cast<Wide>(std::sqrt(static_cast<long double>(cap) * aspect)),
            1, std::min(width, cap));
        rows = std::clamp<Wide>(cap / columns, 1, height);
    }

    const Wide centre_x =
        std::midpoint(static_cast<Wide>(lo.x), static_cast<Wide>(hi.x));
    const Wide centre_y =
        std::midpoint(static_cast<Wide>(lo.y), static_cast<Wide>(hi.y));
    const Wide first_col = std::clamp(
        centre_x - columns / 2, static_cast<Wide>(lo.x),
        static_cast<Wide>(hi.x) - columns + 1);
    const Wide first_row = std::clamp(
        centre_y - rows / 2, static_cast<Wide>(lo.y),
        static_cast<Wide>(hi.y) - rows + 1);

    tiles.reserve(static_cast<std::size_t>(columns * rows));
    for (Wide row = first_row; row < first_row + rows; ++row) {
        for (Wide col = first_col; col < first_col + columns; ++col) {
            tiles.push_back(
                TileKey{kLevel, static_cast<int>(col), static_cast<int>(row)});
        }
    }
    return tiles;
}

// --- WmsConfig -----------------------------------------------------------

WmsConfig WmsConfig::geoportal_ortho() {
    WmsConfig cfg;
    // Public Geoportal ortophoto WMS (EPSG:2180). If this endpoint blocks CORS
    // for emscripten_fetch, swap base_url/layers via set_config() from the host.
    cfg.base_url =
        "https://mapy.geoportal.gov.pl/wss/service/PZGIK/ORTO/WMS/StandardResolution";
    cfg.layers = "Raster";
    cfg.format = "image/jpeg";
    return cfg;
}

WmsConfig WmsConfig::geoportal_topo() {
    WmsConfig cfg;
    cfg.base_url =
        "https://mapy.geoportal.gov.pl/wss/service/img/guest/TOPO/MapServer/WMSServer";
    cfg.layers = "Raster";
    cfg.format = "image/png";
    return cfg;
}

std::string wms_getmap_url(const WmsConfig& cfg, const TileBBox& box) {
    std::string url = cfg.base_url;
    url += (url.find('?') == std::string::npos) ? '?' : '&';
    url += "SERVICE=WMS";
    url += "&VERSION=" + cfg.version;
    url += "&REQUEST=GetMap";
    url += "&LAYERS=" + cfg.layers;
    url += "&STYLES=";
    url += "&SRS=" + cfg.srs;
    url += "&FORMAT=" + cfg.format;
    url += "&WIDTH=" + std::to_string(cfg.tile_pixels);
    url += "&HEIGHT=" + std::to_string(cfg.tile_pixels);
    // WMS 1.1.1: BBOX = minx,miny,maxx,maxy in the SRS axis order (easting,
    // northing for EPSG:2180).
    url += "&BBOX=" + num(box.min_x) + "," + num(box.min_y) + "," + num(box.max_x) +
           "," + num(box.max_y);
    return url;
}

// --- TileService ---------------------------------------------------------

TileService::TileService(WmsConfig cfg) : cfg_{std::move(cfg)} {}

std::string TileService::url_for(const TileKey& key) const {
    return wms_getmap_url(cfg_, TileGrid::bbox(key));
}

bool TileService::has(const TileKey& key) const {
    const auto it = cache_.find(key);
    return it != cache_.end() && it->second.status == TileStatus::Ready;
}

const TileImage* TileService::peek(const TileKey& key) const {
    const auto it = cache_.find(key);
    if (it != cache_.end() && it->second.status == TileStatus::Ready) {
        return &it->second;
    }
    return nullptr;
}

void TileService::store(const TileKey& key, std::vector<std::uint8_t> bytes,
                        std::string content_type) {
    TileImage& image = cache_[key];
    image.status = TileStatus::Ready;
    image.bytes = std::move(bytes);
    image.content_type = std::move(content_type);
}

TileStatus TileService::request(const TileKey& key) {
    const auto it = cache_.find(key);
    if (it != cache_.end()) {
        // A tile that gave up (10 failed attempts) is retried the next time it is
        // asked for (a pan/zoom/toggle), so the underlay self-heals once the
        // endpoint is reachable again — no page reload needed.
        if (it->second.status == TileStatus::Failed) {
            it->second.attempts = 0;
            it->second.status = TileStatus::Loading;
            begin_fetch(key);
        }
        return it->second.status;  // otherwise: hit, no network
    }

    cache_[key].status = TileStatus::Loading;
    begin_fetch(key);
    return cache_[key].status;
}

void TileService::complete_fetch(const TileKey& key, bool ok,
                                 std::vector<std::uint8_t> bytes,
                                 std::string content_type) {
    if (ok) {
        store(key, std::move(bytes), std::move(content_type));
        if (on_ready_) on_ready_(key);
        return;
    }
    // Retry up to kMaxRetries times per tile; after that give up (Failed).
    TileImage& image = cache_[key];
    if (++image.attempts <= kMaxRetries) {
        image.status = TileStatus::Loading;
        schedule_retry(key);
    } else {
        // Give up quietly: do NOT notify the host, so a Failed tile does not by
        // itself trigger a redraw → re-request → refetch loop in the background.
        // It is retried only on the next user-driven request (see request()).
        image.status = TileStatus::Failed;
    }
}

#ifdef __EMSCRIPTEN__
namespace {
// Delay between retry attempts, so a persistently failing endpoint is retried
// steadily rather than in a tight loop.
constexpr int kRetryDelayMs = 1500;

struct FetchCtx {
    TileService* self;
    TileKey key;
};

struct RetryCtx {
    TileService* self;
    TileKey key;
};

void on_retry(void* user_data) {
    auto* ctx = static_cast<RetryCtx*>(user_data);
    TileService* self = ctx->self;
    const TileKey key = ctx->key;
    delete ctx;
    self->retry_now(key);
}

void on_fetch_success(emscripten_fetch_t* fetch) {
    auto* ctx = static_cast<FetchCtx*>(fetch->userData);
    std::vector<std::uint8_t> bytes(
        reinterpret_cast<const std::uint8_t*>(fetch->data),
        reinterpret_cast<const std::uint8_t*>(fetch->data) +
            static_cast<std::size_t>(fetch->numBytes));
    TileService* self = ctx->self;
    const TileKey key = ctx->key;
    delete ctx;
    emscripten_fetch_close(fetch);
    self->complete_fetch(key, true, std::move(bytes), {});
}

void on_fetch_error(emscripten_fetch_t* fetch) {
    auto* ctx = static_cast<FetchCtx*>(fetch->userData);
    TileService* self = ctx->self;
    const TileKey key = ctx->key;
    delete ctx;
    emscripten_fetch_close(fetch);
    self->complete_fetch(key, false, {}, {});
}
}  // namespace
#endif

void TileService::begin_fetch(const TileKey& key) {
    const std::string url = url_for(key);

    // An injected fetcher wins on every platform (used by native tests and any
    // host that wants to route the request itself).
    if (fetcher_) {
        const TileKey captured = key;
        fetcher_(url, [this, captured](bool ok, std::vector<std::uint8_t> bytes,
                                       std::string content_type) {
            complete_fetch(captured, ok, std::move(bytes), std::move(content_type));
        });
        return;
    }

#ifdef __EMSCRIPTEN__
    emscripten_fetch_attr_t attr;
    emscripten_fetch_attr_init(&attr);
    std::strcpy(attr.requestMethod, "GET");
    attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY;
    attr.userData = new FetchCtx{this, key};
    attr.onsuccess = on_fetch_success;
    attr.onerror = on_fetch_error;
    emscripten_fetch(&attr, url.c_str());
#else
    // No network backend available (native, no injected fetcher): mark failed.
    complete_fetch(key, false, {}, {});
#endif
}

void TileService::retry_now(const TileKey& key) { begin_fetch(key); }

void TileService::schedule_retry(const TileKey& key) {
#ifdef __EMSCRIPTEN__
    // Defer asynchronously: infinite retry without blocking or recursion.
    emscripten_async_call(&on_retry, new RetryCtx{this, key}, kRetryDelayMs);
#else
    // Injected-fetcher path (native/tests): re-issue immediately. The fetcher
    // controls timing, so a fetcher that eventually succeeds ends the loop.
    begin_fetch(key);
#endif
}

}  // namespace maj0sted::web
