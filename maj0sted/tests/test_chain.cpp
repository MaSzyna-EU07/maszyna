// The element chain: a walk, one element at a time, with every length said
// outright. Nothing here is solved for — what the document says is what is laid.

#include <cmath>
#include <numbers>

#include "check.hpp"
#include "maj0sted/editor/chain.hpp"

using namespace maj0sted::editor;
using maj0sted::domain::geometry::Pose;

namespace {

constexpr double kTol = 1e-6;

bool near(double a, double b, double tolerance = 1e-6) {
    return std::abs(a - b) <= tolerance;
}

Pose pose_at(double x, double y, double azimuth) {
    return Pose{x, y, std::sin(azimuth), std::cos(azimuth)};
}

Element line(Document& document, double metres) {
    return Element{mint_element(document), Kind::Line, 0.0, 0, metres, Free{}};
}
Element arc(Document& document, double radius, int hand, double metres) {
    return Element{mint_element(document), Kind::Arc, radius, hand, metres, Free{}};
}
Element clothoid(Document& document, double radius, int hand, double metres) {
    return Element{mint_element(document), Kind::Clothoid, radius, hand, metres, Free{}};
}

bool has(const TrackOutcome& outcome, Code code) {
    for (const auto& diagnostic : outcome.diagnostics) {
        if (diagnostic.code == code) {
            return true;
        }
    }
    return false;
}

// -------------------------------------------------------------------------

/// A straight, an arc and a straight laid head to tail: no construction lines,
/// no fitting, nothing to close onto.
void open_chain_lays_head_to_tail() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.anchor = AtPose{0.0, 0.0, 0.0};
    track.elements = {line(document, 100.0), arc(document, 300.0, 1, 150.0),
                      line(document, 50.0)};
    document.tracks.push_back(track);

    const TrackOutcome out = lay_track(document, track, pose_at(0.0, 0.0, 0.0));

    CHECK(out.diagnostics.empty());
    CHECK(out.track.complete);
    CHECK(out.track.elements.size() == 3);
    CHECK(near(out.track.length, 300.0));

    // Heading turned by L/R = 150/300 = 0.5 rad, to the left (hand = +1).
    const double turned = 0.5;
    CHECK(near(out.track.end.hx, std::sin(-turned) * -1.0 + 0.0, 1e-9) ||
          near(std::atan2(out.track.end.hx, out.track.end.hy), -turned, 1e-9));
    // Start pose points north; after a left turn the azimuth decreases.
    CHECK(near(std::atan2(out.track.end.hx, out.track.end.hy), -turned, 1e-9));

    // The first element starts exactly at the anchor.
    CHECK(near(out.track.elements[0].start.x, 0.0));
    CHECK(near(out.track.elements[0].start.y, 0.0));
    // The straight really is 100 m due north.
    CHECK(near(out.track.elements[0].end.x, 0.0, 1e-9));
    CHECK(near(out.track.elements[0].end.y, 100.0, 1e-9));
}

/// An arc as the very first element — the "łuk wprost z krzyżownicy" case. The
/// old model could not say this: a curve only existed between two straights.
void a_chain_may_start_with_an_arc() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.elements = {arc(document, 190.0, -1, 60.0)};
    document.tracks.push_back(track);

    const TrackOutcome out = lay_track(document, track, pose_at(10.0, 20.0, 0.0));

    CHECK(out.diagnostics.empty());
    CHECK(out.track.elements.size() == 1);
    CHECK(near(out.track.elements[0].start.x, 10.0));
    CHECK(near(out.track.elements[0].start.y, 20.0));
    CHECK(near(out.track.elements[0].k0, -1.0 / 190.0));
    CHECK(near(out.track.length, 60.0));
}

/// Two arcs back to back with no straight between them — also unsayable before.
void arcs_may_follow_each_other() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.elements = {arc(document, 400.0, 1, 100.0), arc(document, 250.0, -1, 80.0)};
    document.tracks.push_back(track);

    const TrackOutcome out = lay_track(document, track, pose_at(0.0, 0.0, 0.0));

    CHECK(out.diagnostics.empty());
    CHECK(out.track.elements.size() == 2);
    // The second starts exactly where the first ends, tangentially.
    CHECK(near(out.track.elements[1].start.x, out.track.elements[0].end.x, 1e-12));
    CHECK(near(out.track.elements[1].start.y, out.track.elements[0].end.y, 1e-12));
    CHECK(near(out.track.elements[1].start.hx, out.track.elements[0].end.hx, 1e-12));
    // Net turn: +100/400 - 80/250 = 0.25 - 0.32 = -0.07 rad.
    CHECK(near(std::atan2(out.track.end.hx, out.track.end.hy), 0.07, 1e-9));
}

/// A chain that simply stops. Nothing is synthesised to close it onto.
void an_open_chain_need_not_close() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.elements = {arc(document, 300.0, 1, 500.0)};
    document.tracks.push_back(track);

    const TrackOutcome out = lay_track(document, track, pose_at(0.0, 0.0, 0.0));
    CHECK(out.diagnostics.empty());
    CHECK(out.track.complete);
    CHECK(out.track.elements.size() == 1);
}

/// What cannot be laid is said out loud, and the chain still gives up whatever
/// stood before it.
void a_bad_element_is_reported_and_stops_the_chain() {
    {   // A length of nothing.
        Document document;
        Track track;
        track.id = mint_track(document);
        track.elements = {line(document, 100.0), arc(document, 300.0, 1, 0.0),
                          line(document, 50.0)};
        const TrackOutcome out = lay_track(document, track, pose_at(0.0, 0.0, 0.0));
        CHECK(has(out, Code::DegenerateElement));
        CHECK(!out.track.complete);
        CHECK(out.track.elements.size() == 1);  // the straight before it still stands
        CHECK(near(out.track.length, 100.0));
    }
    {   // An arc without a radius.
        Document document;
        Track track;
        track.id = mint_track(document);
        track.elements = {arc(document, 0.0, 1, 50.0)};
        const TrackOutcome out = lay_track(document, track, pose_at(0.0, 0.0, 0.0));
        CHECK(has(out, Code::DegenerateElement));
        CHECK(out.track.elements.empty());
    }
    {   // An arc that does not say which way it turns.
        Document document;
        Track track;
        track.id = mint_track(document);
        track.elements = {arc(document, 300.0, 0, 50.0)};
        const TrackOutcome out = lay_track(document, track, pose_at(0.0, 0.0, 0.0));
        CHECK(has(out, Code::DegenerateElement));
    }
    {   // A transition curve that eases nothing: flat to flat.
        Document document;
        Track track;
        track.id = mint_track(document);
        track.elements = {line(document, 20.0), clothoid(document, 0.0, 1, 40.0)};
        const TrackOutcome out = lay_track(document, track, pose_at(0.0, 0.0, 0.0));
        CHECK(has(out, Code::DegenerateElement));
        CHECK(out.track.elements.size() == 1);
    }
}

/// A transition curve takes its near-end curvature from wherever the chain has
/// got to, and hands its far-end curvature on. That is the only way one element
/// depends on another.
void a_transition_eases_between_what_it_finds() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.elements = {arc(document, 300.0, 1, 100.0), clothoid(document, 600.0, 1, 50.0),
                      arc(document, 600.0, 1, 80.0), clothoid(document, 0.0, 1, 40.0),
                      line(document, 30.0)};
    document.tracks.push_back(track);

    const TrackOutcome out = lay_track(document, track, pose_at(0.0, 0.0, 0.0));

    CHECK(out.diagnostics.empty());
    CHECK(out.track.elements.size() == 5);
    CHECK(near(out.track.elements[1].k0, 1.0 / 300.0));
    CHECK(near(out.track.elements[1].k1, 1.0 / 600.0));
    CHECK(near(out.track.elements[3].k0, 1.0 / 600.0));
    CHECK(near(out.track.elements[3].k1, 0.0));
    // Every element kept the length it was given.
    CHECK(near(out.track.length, 100.0 + 50.0 + 80.0 + 40.0 + 30.0));
}

/// Solving twice gives the same answer: nothing is written back anywhere.
void solving_is_idempotent() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.elements = {line(document, 120.0), clothoid(document, 500.0, -1, 40.0),
                      arc(document, 500.0, -1, 180.0), clothoid(document, 0.0, -1, 40.0),
                      line(document, 60.0)};

    const TrackOutcome first = lay_track(document, track, pose_at(0.0, 0.0, 0.0));
    const TrackOutcome second = lay_track(document, track, pose_at(0.0, 0.0, 0.0));

    CHECK(first.diagnostics.empty());
    CHECK(first.track.elements.size() == second.track.elements.size());
    for (std::size_t i = 0; i < first.track.elements.size(); ++i) {
        CHECK(near(first.track.elements[i].length, second.track.elements[i].length, kTol));
        CHECK(near(first.track.elements[i].start.x, second.track.elements[i].start.x, kTol));
    }
}

}  // namespace

int main() {
    RUN(open_chain_lays_head_to_tail);
    RUN(a_chain_may_start_with_an_arc);
    RUN(arcs_may_follow_each_other);
    RUN(an_open_chain_need_not_close);
    RUN(a_bad_element_is_reported_and_stops_the_chain);
    RUN(a_transition_eases_between_what_it_finds);
    RUN(solving_is_idempotent);
    return REPORT();
}
