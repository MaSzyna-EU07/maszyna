// A simple visual check tool: build a niweleta, fit a compound curve with
// transition curves between the two straights, apply it, and write the plan to
// an SVG file you can open in a browser.
//
// Usage:  plan_to_svg [output.svg]   (default: plan.svg)
//
// The reusable pieces are render::sample() (geometry -> polylines) and
// io::to_svg(). A different GUI would call render::sample() and paint itself.

#include <cstdio>
#include <fstream>
#include <string>

#include "maj0sted/io/svg_writer.hpp"
#include "maj0sted/maj0sted.hpp"
#include "maj0sted/render/plan_sampler.hpp"

using namespace maj0sted::domain;

int main(int argc, char** argv) {
    MapProject project;

    // Two straights forming a 90-degree corner.
    Niweleta niweleta{NiweletaId{1}, "Demo"};
    const StraightId entry =
        niweleta.add_straight(CartesianPosition{0.0, 0.0}, CartesianPosition{300.0, 0.0});
    const StraightId exit = niweleta.add_straight(
        CartesianPosition{300.0, 0.0}, CartesianPosition{300.0, 300.0},
        JointContinuity::AzimuthBreak);

    // A compound curve R120 -> R80 with clothoids at the ends and between arcs.
    const CompoundFitParameters params{
        .entry = entry,
        .exit = exit,
        .entry_transition = Length::from_metres(30.0),
        .exit_transition = Length::from_metres(30.0),
        .arcs = {CompoundArc{Radius::from_metres(120.0), Length::from_metres(60.0),
                             Length::from_metres(25.0)},
                 CompoundArc{Radius::from_metres(80.0)}}};

    const FitResult fit = FittingService::fit_compound(niweleta, params);
    niweleta.apply_fit(fit);
    project.add_niweleta(std::move(niweleta));

    const maj0sted::render::Scene scene = maj0sted::render::sample(project);
    const std::string svg = maj0sted::io::to_svg(scene);

    const std::string path = (argc > 1) ? argv[1] : "plan.svg";
    std::ofstream out(path);
    if (!out) {
        std::fprintf(stderr, "cannot write %s\n", path.c_str());
        return 1;
    }
    out << svg;
    std::printf("wrote %s (%zu polylines) — open it in a browser\n", path.c_str(),
                scene.polylines.size());
    return 0;
}
