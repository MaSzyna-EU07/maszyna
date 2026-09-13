#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace maj0sted::editor {

/// A stable tile in the EPSG:2180 tiling pyramid. Tiles are addressed by their
/// zoom level and integer column/row, so the same ground square always maps to
/// the same key regardless of the current viewport — that is what makes the
/// cache reusable across pans and zooms.
struct TileKey {
    int zoom{0};
    int x{0};  ///< column (increases with easting)
    int y{0};  ///< row (increases with northing)

    friend bool operator==(const TileKey&, const TileKey&) = default;
};

struct TileKeyHash {
    std::size_t operator()(const TileKey& k) const noexcept {
        // Mix the three small integers into one hash.
        std::uint64_t h = static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.x));
        h = h * 1099511628211ULL ^
            static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.y));
        h = h * 1099511628211ULL ^ static_cast<std::uint64_t>(k.zoom);
        return static_cast<std::size_t>(h);
    }
};

/// A bounding box in EPSG:2180 metres (easting = x, northing = y).
struct TileBBox {
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
};

/// The tiling scheme. A pure, static value type defining a square grid over the
/// EPSG:2180 plane. The default grid is fixed 100 m × 100 m cells (key.zoom =
/// kLevel). Coarser overview grids encode the cell size in metres in
/// @c TileKey::zoom when that value is >= 100 (e.g. zoom 1000 → 1 km cells for
/// topo underlays). Nothing here touches the network.
class TileGrid {
public:
    static constexpr int kTilePixels = 1024;  ///< pixels per cell edge (GetMap WIDTH/HEIGHT)
    static constexpr double kOriginX = 0.0;   ///< easting of cell (0,0)'s left edge
    static constexpr double kOriginY = 0.0;   ///< northing of cell (0,0)'s bottom edge
    /// Default cache cell size: 100 m × 100 m ground square.
    static constexpr double kCellMetres = 100.0;
    /// Default grid level stored in TileKey::zoom for the 100 m grid.
    static constexpr int kLevel = 0;
    /// Zoom values at or above this are treated as the cell edge in metres.
    static constexpr int kCellZoomMin = 100;

    /// Cell edge in metres for @p key (100 m default, or key.zoom when coarse).
    [[nodiscard]] static double cell_metres(const TileKey& key) noexcept;

    /// Ground resolution (metres per pixel) of a cell.
    [[nodiscard]] static double resolution(int zoom = kLevel) noexcept;

    /// Edge length of a cell in metres for the given zoom/cell encoding.
    [[nodiscard]] static double tile_span(int zoom = kLevel) noexcept;

    /// The cell whose ground square contains (@p x, @p y).
    [[nodiscard]] static TileKey tile_at(double x, double y, int zoom = kLevel) noexcept;

    /// The ground bounding box (EPSG:2180) of @p key.
    [[nodiscard]] static TileBBox bbox(const TileKey& key) noexcept;

    /// The grid level for a target resolution. Always kLevel for the fine grid.
    [[nodiscard]] static int zoom_for_resolution(double metres_per_pixel) noexcept;

    /// Every cell whose ground square intersects @p view, capped at @p max_tiles.
    /// Pass a coarse cell size (e.g. 1000) as @p zoom to get a 1 km overview grid.
    [[nodiscard]] static std::vector<TileKey> tiles_for_view(const TileBBox& view,
                                                             int zoom = kLevel,
                                                             int max_tiles = 64);
};

/// Configuration for a WMS GetMap source. Defaults target a public EPSG:2180
/// endpoint (Polish Geoportal). Every field is data, so a different backend is a
/// pure configuration change with no code edits.
struct WmsConfig {
    std::string base_url;
    std::string layers;
    std::string format{"image/jpeg"};
    std::string version{"1.1.1"};  ///< 1.1.1 keeps BBOX as minx,miny,maxx,maxy
    std::string srs{"EPSG:2180"};
    int tile_pixels{TileGrid::kTilePixels};

    /// Geoportal ortophoto (ortofotomapa), EPSG:2180, JPEG.
    [[nodiscard]] static WmsConfig geoportal_ortho();

    /// Geoportal topographic base map (BDOT/raster), EPSG:2180, PNG.
    [[nodiscard]] static WmsConfig geoportal_topo();
};

/// Builds the full WMS 1.1.1 GetMap URL for @p box using @p cfg. Numbers are
/// formatted locale-independently (std::to_chars), matching the rest of the
/// library's serialization.
[[nodiscard]] std::string wms_getmap_url(const WmsConfig& cfg, const TileBBox& box);

enum class TileStatus { Missing, Loading, Ready, Failed };

/// A cached tile image: raw encoded bytes (JPEG/PNG) plus their MIME type.
struct TileImage {
    TileStatus status{TileStatus::Missing};
    std::vector<std::uint8_t> bytes;
    std::string content_type;
    int attempts{0};  ///< failed fetch attempts so far (see TileService retry cap)
};

/// Owns the tile grid, the tile cache and the WMS fetch. This is the single
/// place that decides which tile is needed and whether it is already cached. The
/// download itself is the host's: it injects a Fetcher, so the tiling and caching
/// logic here stays transport-free and is exercised in tests without a network.
class TileService {
public:
    /// Called when a fetch finishes: (success, bytes, content_type).
    using DoneFn =
        std::function<void(bool, std::vector<std::uint8_t>, std::string)>;
    /// Injectable network backend: given a URL, eventually invoke DoneFn.
    using Fetcher = std::function<void(const std::string& url, DoneFn)>;

    explicit TileService(WmsConfig cfg = WmsConfig::geoportal_ortho());

    [[nodiscard]] const WmsConfig& config() const noexcept { return cfg_; }
    void set_config(WmsConfig cfg) { cfg_ = std::move(cfg); }

    /// Sets the transport. When unset, a requested tile fails at once (there is
    /// no built-in downloader — the host owns that).
    void set_fetcher(Fetcher fetcher) { fetcher_ = std::move(fetcher); }

    /// Notified (with the tile's key) whenever a tile transitions to Ready or
    /// Failed. Lets the renderer schedule a redraw when an async tile arrives.
    void set_on_ready(std::function<void(TileKey)> cb) { on_ready_ = std::move(cb); }

    /// Ensures @p key is cached or being fetched, and returns its current status.
    /// A cache hit returns Ready immediately and issues no network request.
    TileStatus request(const TileKey& key);

    /// The cached image for @p key, or nullptr if it is not Ready.
    [[nodiscard]] const TileImage* peek(const TileKey& key) const;

    /// Stores bytes for @p key directly, marking it Ready. Exposed so a caller
    /// (or a test) can seed the cache without going through the fetcher.
    void store(const TileKey& key, std::vector<std::uint8_t> bytes,
               std::string content_type = {});

    [[nodiscard]] bool has(const TileKey& key) const;
    [[nodiscard]] std::size_t size() const noexcept { return cache_.size(); }
    void clear() { cache_.clear(); }

    /// The GetMap URL this service would use for @p key.
    [[nodiscard]] std::string url_for(const TileKey& key) const;

    /// Finalises an in-flight fetch: on success stores the bytes (Ready), on
    /// failure marks the tile Failed, then fires the on_ready callback. Public so
    /// the host's fetcher callback can hand the result back here.
    void complete_fetch(const TileKey& key, bool ok, std::vector<std::uint8_t> bytes,
                        std::string content_type);

    /// Re-issues the fetch for @p key.
    void retry_now(const TileKey& key);

private:
    void begin_fetch(const TileKey& key);
    /// Re-issues a fetch for @p key after a failure — the service retries up to
    /// kMaxRetries times, then marks the tile Failed. The fetcher owns the timing.
    void schedule_retry(const TileKey& key);

    WmsConfig cfg_;
    std::unordered_map<TileKey, TileImage, TileKeyHash> cache_;
    Fetcher fetcher_;
    std::function<void(TileKey)> on_ready_;
};

}  // namespace maj0sted::editor
