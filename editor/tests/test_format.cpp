/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
#include <variant>

#include <cmath>
#include <string>
#include "check.hpp"
import eu07.editor.plan_layout;
import eu07.editor.plan_document_io;

// The .m0s format, version 5: everything by id, an explicit version, and a
// round trip that is byte-identical. Older files still open: a version-3 one has
// its arcs and blade fields converted to the piece list on the way in, and a
// version-4 one simply had no nose on its blade yet.



using namespace editor::plan;

namespace {

Document sample() {
    Document document;

    TurnoutType type;
    type.name = "Rz 1:9 R190 z iglicą nagiętą";  // spaces and non-ASCII on purpose
    type.crossing_n = 9.0;
    type.length = 27.1;
    type.pieces = {
        TurnoutPieceSpec{0, 2.42, 0.0, 0.0, 0.0},
        TurnoutPieceSpec{1, 8.3, 190.0, 190.0, 0.0058},
        TurnoutPieceSpec{2, 12.75, 190.0, 250.0, 0.0},
        TurnoutPieceSpec{3, 4.125, 0.0, 0.0, 0.0},
    };
    type.blade.tip_thickness = 0.005;
    type.blade.nose = 0.125;
    type.blade.railtop_width = 0.07;
    document.turnout_types.push_back(type);

    Track through;
    through.id = mint_track(document);
    through.name = "tor zasadniczy 1";
    through.anchor = AtPose{100.5, -2.25, 0.7853981633974483};
    through.elements = {
        Element{mint_element(document), Kind::Line, 0.0, 0, 412.5, Free{}},
        Element{mint_element(document), Kind::Clothoid, 600.0, -1, 40.0, Free{}},
        Element{mint_element(document), Kind::Arc, 600.0, -1, 235.125, Free{}},
        Element{mint_element(document), Kind::Line, 0.0, 0, 180.0, Free{}},
    };
    document.tracks.push_back(through);

    TurnoutPlacement placement;
    placement.id = mint_turnout(document);
    placement.type = type.name;
    placement.on = through.id;
    placement.station = 312.75;
    placement.hand = -1;
    placement.facing = false;
    placement.bend_from_track = false;
    placement.bend = -1.0 / 380.292;  // wygięty dwustronnie, na własny łuk
    document.turnouts.push_back(placement);

    Track branch;
    branch.id = mint_track(document);
    branch.name = "odnoga";
    branch.anchor = AtPort{placement.id, Port::Frog};
    branch.elements = {
        Element{mint_element(document), Kind::Arc, 190.0, -1, 45.5, Free{}},
        Element{mint_element(document), Kind::Line, 0.0, 0, 123.5, Free{}},
    };
    // Held parallel to the through track's last straight, at międzytorze.
    branch.elements[1].hold = Parallel{through.elements[3].id, 4.75};
    document.tracks.push_back(branch);

    document.view_x = 500.0;
    document.view_y = 250.0;
    document.view_extent = 2000.0;
    document.origin_set = true;
    document.georeferenced = true;
    document.origin_x = 6540000.25;
    document.origin_y = 5760000.5;
    // an absolute path into another MaSzyna, spaces and all
    document.scn_path = "/home/kto to/MaSzyna/scenery/plan export.scn";
    return document;
}

bool same(const Document& a, const Document& b) {
    if (a.tracks.size() != b.tracks.size() ||
        a.turnouts.size() != b.turnouts.size() ||
        a.turnout_types.size() != b.turnout_types.size() || a.next_id != b.next_id) {
        return false;
    }
    for (std::size_t i = 0; i < a.tracks.size(); ++i) {
        if (a.tracks[i].id != b.tracks[i].id || a.tracks[i].name != b.tracks[i].name ||
            a.tracks[i].elements.size() != b.tracks[i].elements.size()) {
            return false;
        }
        if (a.tracks[i].anchor.index() != b.tracks[i].anchor.index()) {
            return false;
        }
        for (std::size_t k = 0; k < a.tracks[i].elements.size(); ++k) {
            const Element& x = a.tracks[i].elements[k];
            const Element& y = b.tracks[i].elements[k];
            if (x.id != y.id || x.kind != y.kind || x.radius != y.radius ||
                x.hand != y.hand || x.length != y.length ||
                x.hold.index() != y.hold.index()) {
                return false;
            }
        }
    }
    return true;
}

void round_trip_is_byte_identical() {
    const Document document = sample();
    const std::string text = editor::plan::io::serialize(document);

    const auto reloaded = editor::plan::io::deserialize(text);
    CHECK(reloaded.has_value());
    if (!reloaded) {
        return;
    }
    CHECK(same(document, *reloaded));
    CHECK(editor::plan::io::serialize(*reloaded) == text);
}

void every_field_survives() {
    const Document document = sample();
    const auto reloaded = editor::plan::io::deserialize(editor::plan::io::serialize(document));
    CHECK(reloaded.has_value());
    if (!reloaded) {
        return;
    }

    // A name with spaces and non-ASCII.
    CHECK(reloaded->turnout_types[0].name == "Rz 1:9 R190 z iglicą nagiętą");
    // The geometry list, every piece of it, and the blade's own dimensions.
    CHECK(reloaded->turnout_types[0].pieces.size() == 4);
    CHECK(reloaded->turnout_types[0].pieces[0].part == 0);
    CHECK(reloaded->turnout_types[0].pieces[0].length == 2.42);
    CHECK(reloaded->turnout_types[0].pieces[1].part == 1);
    CHECK(reloaded->turnout_types[0].pieces[1].radius_start == 190.0);
    CHECK(reloaded->turnout_types[0].pieces[1].turn_in == 0.0058);
    CHECK(reloaded->turnout_types[0].pieces[2].radius_end == 250.0);
    CHECK(reloaded->turnout_types[0].pieces[3].part == 3);
    CHECK(reloaded->turnout_types[0].blade.tip_thickness == 0.005);
    CHECK(reloaded->turnout_types[0].blade.nose == 0.125);
    CHECK(reloaded->turnout_types[0].blade.railtop_width == 0.07);
    // Placement, including a backward-facing turnout.
    CHECK(reloaded->turnouts[0].station == 312.75);
    CHECK(reloaded->turnouts[0].hand == -1);
    CHECK(reloaded->turnouts[0].facing == false);
    CHECK(reloaded->turnouts[0].bend_from_track == false);
    CHECK(std::abs(reloaded->turnouts[0].bend + 1.0 / 380.292) < 1e-12);
    CHECK(reloaded->turnouts[0].type == "Rz 1:9 R190 z iglicą nagiętą");
    // Every element says its own shape and its own length.
    CHECK(reloaded->tracks[0].elements[0].kind == Kind::Line);
    CHECK(reloaded->tracks[0].elements[0].length == 412.5);
    CHECK(reloaded->tracks[0].elements[1].kind == Kind::Clothoid);
    CHECK(reloaded->tracks[0].elements[1].radius == 600.0);
    CHECK(reloaded->tracks[0].elements[2].length == 235.125);
    CHECK(reloaded->tracks[0].elements[2].hand == -1);
    // A hold, and the one element that carries it.
    CHECK(std::holds_alternative<Free>(reloaded->tracks[1].elements[0].hold));
    CHECK(std::holds_alternative<Parallel>(reloaded->tracks[1].elements[1].hold));
    CHECK(std::get<Parallel>(reloaded->tracks[1].elements[1].hold).offset == 4.75);
    CHECK(std::get<Parallel>(reloaded->tracks[1].elements[1].hold).ref ==
          reloaded->tracks[0].elements[3].id);
    // Anchors.
    CHECK(std::holds_alternative<AtPose>(reloaded->tracks[0].anchor));
    CHECK(std::holds_alternative<AtPort>(reloaded->tracks[1].anchor));
    CHECK(std::get<AtPort>(reloaded->tracks[1].anchor).port == Port::Frog);
    // Georeference and view.
    CHECK(reloaded->origin_x == 6540000.25);
    CHECK(reloaded->view_extent == 2000.0);
    CHECK(reloaded->georeferenced);
    // and where the plan is exported to, which says which MaSzyna it is drawn for
    CHECK(reloaded->scn_path == "/home/kto to/MaSzyna/scenery/plan export.scn");
    // Ids keep going where they left off, so nothing minted later collides.
    CHECK(reloaded->next_id == document.next_id);
}

void an_older_file_is_refused() {
    // Version 1 and version 2 described different models; both are refused
    // outright rather than half-understood. Version 3 is not refused — it is
    // converted, which an_old_file_upgrades_to_the_piece_list checks.
    const std::string version_1 =
        "the plan library 1\ncrs 2180\nniwelety 0\neditor 1\neniw 2 1\nename niweleta 1\n";
    CHECK(!editor::plan::io::deserialize(version_1).has_value());
    const std::string version_2 =
        "m0s 2\nnext 9\nview 0 0 0\norigin 0 0 0 0\nlines 0\ntypes 0\n"
        "tracks 0\n";
    CHECK(!editor::plan::io::deserialize(version_2).has_value());
    CHECK(!editor::plan::io::deserialize("").has_value());
    CHECK(!editor::plan::io::deserialize("m0s 8\n").has_value());
}

void a_file_from_before_lukowanie_still_opens() {
    // Version 5 placements had nothing to say about a bend, because a turnout stood
    // on the track as it found it. That is what taking the bend from the track means,
    // so they open as exactly that and nothing about them moves.
    const std::string version_5 =
        "m0s 5\nnext 5\nview 0 0 0\norigin 0 0 0 0\ntypes 0\n"
        "turnouts 1\nturnout 2 1 100 1 1\ntutype Rz 1:9 R190\ntracks 0\n";
    const auto document = editor::plan::io::deserialize(version_5);
    CHECK(document.has_value());
    if (!document) {
        return;
    }
    CHECK(document->turnouts.size() == 1);
    CHECK(document->turnouts[0].station == 100.0);
    CHECK(document->turnouts[0].bend_from_track);
    CHECK(document->turnouts[0].bend == 0.0);
    CHECK(editor::plan::io::serialize(*document).find("m0s 7") != std::string::npos);
}

void a_file_from_before_the_export_path_still_opens() {
    // Version 6 said nothing about where a plan exported to, so it opens with
    // nothing said and the host falls back on its own answer.
    const std::string version_6 =
        "m0s 6\nnext 3\nview 0 0 0\norigin 0 0 0 0\ntypes 0\nturnouts 0\n"
        "tracks 1\ntrack 1\ntkname tor\nanchor pose 0 0 0\nelements 1\n"
        "element 2 line 0 0 250\n";
    const auto document = editor::plan::io::deserialize(version_6);
    CHECK(document.has_value());
    if (!document) {
        return;
    }
    CHECK(document->scn_path.empty());
    CHECK(document->tracks.size() == 1);
    CHECK(editor::plan::io::serialize(*document).find("m0s 7") != std::string::npos);
}

void a_file_from_before_the_nose_still_opens() {
    // Version 4 had the piece list but no nose on the blade: the type line went
    // straight from the tip thickness to the railhead width. It opens with the nose
    // at nothing, which is what the type then reports against the curve it lays.
    const std::string version_4 =
        "m0s 4\nnext 1\nview 0 0 0\norigin 0 0 0 0\n"
        "types 1\ntype 9 27.138 0.0049 0.07 1\ntseg 2 21.02 190 190 0\n"
        "tname Rz 1:9 R190\nturnouts 0\ntracks 0\n";
    const auto document = editor::plan::io::deserialize(version_4);
    CHECK(document.has_value());
    if (!document) {
        return;
    }
    CHECK(document->turnout_types.size() == 1);
    CHECK(document->turnout_types[0].blade.tip_thickness == 0.0049);
    CHECK(document->turnout_types[0].blade.nose == 0.0);
    CHECK(document->turnout_types[0].blade.railtop_width == 0.07);
    CHECK(document->turnout_types[0].pieces.size() == 1);

    // and it is written back in the new form, with a place for the nose
    CHECK(editor::plan::io::serialize(*document).find("m0s 7") != std::string::npos);
}

void an_old_file_upgrades_to_the_piece_list() {
    // A version-3 catalogue: one arc, no blade, and a closing arc that was worked
    // out on the fly. It has to come back as an explicit list that lays the very
    // same shape — on the crossing angle and on the catalogue length.
    const std::string version_3 =
        "m0s 3\nnext 1\nview 0 0 0\norigin 0 0 0 0\n"
        "types 1\ntype 9 0 0 27.138 0 0 0 1\ntarc 190 0 0\ntname Rz 1:9 R190\n"
        "turnouts 0\ntracks 0\n";
    const auto document = editor::plan::io::deserialize(version_3);
    CHECK(document.has_value());
    if (!document) {
        return;
    }
    CHECK(document->turnout_types.size() == 1);
    const TurnoutType& type = document->turnout_types[0];
    CHECK(type.name == "Rz 1:9 R190");
    CHECK(type.pieces.size() == 2);  // the closing arc and the frog rail
    CHECK(type.pieces[0].part == 2);
    CHECK(std::abs(type.pieces[0].length - 190.0 * std::atan(1.0 / 9.0)) < 1e-9);
    CHECK(type.pieces[1].part == 3);

    const auto laid = editor::plan::lay_turnout(
        editor::plan::geometry::Pose{0.0, 0.0, 1.0, 0.0}, to_domain(type, 1));
    CHECK(laid.valid);
    CHECK(std::abs(laid.angle_residual) < 1e-9);
    CHECK(std::abs(laid.length_residual) < 1e-9);

    // and it is written back in the new form
    CHECK(editor::plan::io::serialize(*document).find("tseg ") != std::string::npos);
}

void a_truncated_file_is_refused() {
    const std::string text = editor::plan::io::serialize(sample());
    // Cut off half way: the counts no longer match what follows.
    CHECK(!editor::plan::io::deserialize(text.substr(0, text.size() / 2)).has_value());
}

}  // namespace

int main() {
    RUN(round_trip_is_byte_identical);
    RUN(every_field_survives);
    RUN(an_older_file_is_refused);
    RUN(an_old_file_upgrades_to_the_piece_list);
    RUN(a_file_from_before_the_nose_still_opens);
    RUN(a_file_from_before_lukowanie_still_opens);
    RUN(a_truncated_file_is_refused);
    return REPORT();
}
