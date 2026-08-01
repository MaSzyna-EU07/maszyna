#include <filesystem>
#include <string>

#include "check.hpp"
#include "maj0sted/app/editor_document.hpp"

using namespace maj0sted::app;
using maj0sted::editor::GapFit;
using maj0sted::editor::NiweletaSpec;
using maj0sted::editor::StraightSpec;

namespace {

EditorDocument sample_document() {
    EditorDocument doc;

    NiweletaSpec a;
    a.name = "Niweleta 1";
    a.straights = {StraightSpec{0.0, 0.0, 120.0, 0.0},
                   StraightSpec{120.0, 0.0, 120.0, 120.0, true}};
    GapFit fit;
    fit.gap = 0;
    fit.mode = 2;  // arc + transitions
    fit.radius = 40.0;
    fit.transition = 12.5;
    a.fits = {fit};

    NiweletaSpec b;
    b.name = "Bocznica z spacją";
    b.straights = {StraightSpec{500000.25, 300000.75, 500200.0, 300000.0}};
    b.straights[0].rel_kind = 1;
    b.straights[0].rel_niw = 0;
    b.straights[0].rel_str = 0;
    b.straights[0].rel_offset = -4.5;

    doc.niwelety = {a, b};
    doc.view_x = 500100.5;
    doc.view_y = 300050.25;
    doc.view_extent = 420.0;
    doc.origin_set = true;
    doc.georeferenced = true;
    doc.origin_x = 500000.0;
    doc.origin_y = 300000.0;
    return doc;
}

bool same(const StraightSpec& x, const StraightSpec& y) {
    return x.x1 == y.x1 && x.y1 == y.y1 && x.x2 == y.x2 && x.y2 == y.y2 &&
           x.hidden == y.hidden && x.rel_kind == y.rel_kind &&
           x.rel_niw == y.rel_niw && x.rel_str == y.rel_str &&
           x.rel_offset == y.rel_offset && x.rel_cot == y.rel_cot &&
           x.rel_side == y.rel_side && x.rel_length == y.rel_length;
}

bool same(const GapFit& x, const GapFit& y) {
    return x.gap == y.gap && x.mode == y.mode && x.radius == y.radius &&
           x.transition == y.transition && x.r1 == y.r1 && x.arc1_len == y.arc1_len &&
           x.between == y.between && x.r2 == y.r2 && x.entry_t == y.entry_t &&
           x.exit_t == y.exit_t;
}

void check_equal(const EditorDocument& x, const EditorDocument& y) {
    CHECK(x.view_x == y.view_x);
    CHECK(x.view_y == y.view_y);
    CHECK(x.view_extent == y.view_extent);
    CHECK(x.origin_set == y.origin_set);
    CHECK(x.georeferenced == y.georeferenced);
    CHECK(x.origin_x == y.origin_x);
    CHECK(x.origin_y == y.origin_y);
    CHECK(x.niwelety.size() == y.niwelety.size());
    if (x.niwelety.size() != y.niwelety.size()) return;
    for (std::size_t n = 0; n < x.niwelety.size(); ++n) {
        const auto& a = x.niwelety[n];
        const auto& b = y.niwelety[n];
        CHECK(a.name == b.name);
        CHECK(a.straights.size() == b.straights.size());
        for (std::size_t i = 0; i < a.straights.size() && i < b.straights.size(); ++i) {
            CHECK(same(a.straights[i], b.straights[i]));
        }
        CHECK(a.fits.size() == b.fits.size());
        for (std::size_t i = 0; i < a.fits.size() && i < b.fits.size(); ++i) {
            CHECK(same(a.fits[i], b.fits[i]));
        }
    }
}

void serialize_round_trips_exactly() {
    const EditorDocument doc = sample_document();
    const std::string text = serialize_document(doc);
    const EditorDocument back = deserialize_document(text);
    check_equal(doc, back);

    // The file really embeds a maj0sted project (reuses io::serialize).
    CHECK(text.rfind("maj0sted", 0) == 0);
    CHECK(text.find("\neditor 2\n") != std::string::npos);

    // Serialization is stable (byte-for-byte) across a round trip.
    CHECK(serialize_document(back) == text);
}

void embedded_project_is_valid_and_readable() {
    const EditorDocument doc = sample_document();
    // to_map_project keeps one niweleta per editor niweleta and the straights.
    const auto project = to_map_project(doc);
    CHECK(project.niwelety().size() == 2);
    const EditorDocument imported = from_map_project(project);
    CHECK(imported.niwelety.size() == 2);
    CHECK(imported.niwelety[0].straights.size() == 2);
    CHECK(imported.niwelety[0].name == "Niweleta 1");
}

void save_then_load_from_disk() {
    const EditorDocument doc = sample_document();
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "maj0sted_test";
    std::filesystem::remove_all(dir);
    const std::string path = (dir / "sub" / "proj.m0s").string();

    CHECK(save_project(doc, path));  // creates parent dirs
    CHECK(std::filesystem::exists(path));

    const auto loaded = load_project(path);
    CHECK(loaded.has_value());
    if (loaded) check_equal(doc, *loaded);

    std::filesystem::remove_all(dir);
}

void load_missing_file_is_nullopt() {
    const std::string path =
        (std::filesystem::temp_directory_path() / "maj0sted_nope_12345.m0s").string();
    std::filesystem::remove(path);
    CHECK(!load_project(path).has_value());
}

}  // namespace

int main() {
    RUN(serialize_round_trips_exactly);
    RUN(embedded_project_is_valid_and_readable);
    RUN(save_then_load_from_disk);
    RUN(load_missing_file_is_nullopt);
    return REPORT();
}
