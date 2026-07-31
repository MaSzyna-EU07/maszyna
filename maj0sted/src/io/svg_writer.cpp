#include "maj0sted/io/svg_writer.hpp"

#include <charconv>
#include <string>

namespace maj0sted::io {

namespace {

std::string num(double value) {
    char buffer[64];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
    return std::string(buffer, result.ptr);
}

const char* colour(render::ElementKind kind) {
    switch (kind) {
        case render::ElementKind::Straight:
            return "#222222";
        case render::ElementKind::Arc:
            return "#1f77b4";
        case render::ElementKind::Transition:
            return "#ff7f0e";
    }
    return "#000000";
}

}  // namespace

std::string to_svg(const render::Scene& scene, const SvgOptions& options) {
    double world_w = scene.max_x - scene.min_x;
    double world_h = scene.max_y - scene.min_y;
    if (world_w <= 0.0) world_w = 1.0;
    if (world_h <= 0.0) world_h = 1.0;

    const double scale = (options.width_px - 2.0 * options.margin_px) / world_w;
    const double height_px = 2.0 * options.margin_px + world_h * scale;

    // World -> SVG. North is up, so the y axis is flipped.
    const auto sx = [&](double x) { return options.margin_px + (x - scene.min_x) * scale; };
    const auto sy = [&](double y) { return options.margin_px + (scene.max_y - y) * scale; };

    std::string out;
    out += "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" + num(options.width_px) +
           "\" height=\"" + num(height_px) + "\" viewBox=\"0 0 " + num(options.width_px) +
           " " + num(height_px) + "\">\n";
    out += "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";

    for (const auto& polyline : scene.polylines) {
        out += std::string("<polyline fill=\"none\" stroke=\"") + colour(polyline.kind) +
               "\" stroke-width=\"" + num(options.stroke_px) +
               "\" stroke-linejoin=\"round\" points=\"";
        for (const auto& p : polyline.points) {
            out += num(sx(p.x)) + "," + num(sy(p.y)) + " ";
        }
        out += "\"/>\n";
    }

    out += "</svg>\n";
    return out;
}

}  // namespace maj0sted::io
