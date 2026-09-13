/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <optional>
#include <variant>
#include <string>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string_view>
#include <system_error>
#include <vector>

module eu07.editor.plan_document_io;

namespace editor::plan::io {
namespace {

using namespace editor::plan;

constexpr int kVersion = 7;
constexpr int kNoExportPath = 6;          // before a plan remembered where it exports to
constexpr int kStraightOnly = 5;          // before a placement could be bent onto a łuk
constexpr int kNoseless = 4;              // the piece list, but before the blade carried its nose
constexpr int kLegacyTurnoutVersion = 3;  // arcs + blade fields, before the piece list

std::string num(double value) {
    char buffer[64];
    const auto [end, error] = std::to_chars(buffer, buffer + sizeof(buffer), value);
    if (error != std::errc{}) {
        return "0";
    }
    return std::string{buffer, end};
}

std::string num(std::uint32_t value) { return std::to_string(value); }

/// One line of the file, split into tokens, with the tail kept whole so a name
/// may contain spaces.
struct Line {
    std::vector<std::string_view> tokens;
    std::string_view tail;  ///< everything after the first token, verbatim

    [[nodiscard]] std::string_view tag() const {
        return tokens.empty() ? std::string_view{} : tokens.front();
    }
    [[nodiscard]] std::size_t size() const { return tokens.size(); }
};

Line split(std::string_view text) {
    Line line;
    std::size_t i = 0;
    bool first = true;
    while (i < text.size()) {
        while (i < text.size() && (text[i] == ' ' || text[i] == '\t')) {
            ++i;
        }
        if (i >= text.size()) {
            break;
        }
        if (!first && line.tail.empty()) {
            line.tail = text.substr(i);
        }
        const std::size_t start = i;
        while (i < text.size() && text[i] != ' ' && text[i] != '\t') {
            ++i;
        }
        line.tokens.push_back(text.substr(start, i - start));
        first = false;
    }
    return line;
}

bool to_double(std::string_view token, double& out) {
    const auto [end, error] = std::from_chars(token.data(), token.data() + token.size(), out);
    return error == std::errc{} && end == token.data() + token.size();
}

bool to_uint(std::string_view token, std::uint32_t& out) {
    const auto [end, error] = std::from_chars(token.data(), token.data() + token.size(), out);
    return error == std::errc{} && end == token.data() + token.size();
}

bool to_int(std::string_view token, int& out) {
    const auto [end, error] = std::from_chars(token.data(), token.data() + token.size(), out);
    return error == std::errc{} && end == token.data() + token.size();
}

/// A cursor over the file's lines, skipping blanks.
class Reader {
public:
    explicit Reader(const std::string& text) {
        std::istringstream stream{text};
        std::string raw;
        while (std::getline(stream, raw)) {
            if (!raw.empty() && raw.back() == '\r') {
                raw.pop_back();
            }
            storage_.push_back(std::move(raw));
        }
    }

    /// The next non-blank line, or nothing at the end of the file.
    std::optional<Line> next() {
        while (at_ < storage_.size()) {
            Line line = split(storage_[at_++]);
            if (!line.tokens.empty()) {
                return line;
            }
        }
        return std::nullopt;
    }

    /// The tag of the next non-blank line, without consuming it. Optional
    /// records — an element's hold — are recognised this way.
    std::string_view peek_tag() {
        while (at_ < storage_.size()) {
            const Line line = split(storage_[at_]);
            if (!line.tokens.empty()) {
                return line.tokens.front();
            }
            ++at_;
        }
        return {};
    }

    /// The next line, required to carry @p tag and at least @p tokens tokens.
    std::optional<Line> next(std::string_view tag, std::size_t tokens) {
        const std::optional<Line> line = next();
        if (!line || line->tag() != tag || line->size() < tokens) {
            return std::nullopt;
        }
        return line;
    }

private:
    std::vector<std::string> storage_;
    std::size_t at_{0};
};

/// Reads a version-3 turnout type — arcs plus the old fixed blade fields — and
/// converts it to a piece list. The old geometry closed itself: the last arc spent
/// whatever deflection was left to the crossing angle, and the frog rail ran on to
/// the catalogue length. Both are worked out here once, so an old plan opens as the
/// same shape and is saved in the new form.
bool read_legacy_type(Reader& reader, TurnoutType& type) {
    const std::optional<Line> line = reader.next("type", 9);
    double entry_transition = 0.0;
    double exit_transition = 0.0;
    double pre_blade = 0.0;
    double blade_angle = 0.0;
    double blade_length = 0.0;
    std::uint32_t arc_count = 0;
    if (!line || !to_double(line->tokens[1], type.crossing_n) ||
        !to_double(line->tokens[2], entry_transition) ||
        !to_double(line->tokens[3], exit_transition) ||
        !to_double(line->tokens[4], type.length) ||
        !to_double(line->tokens[5], pre_blade) ||
        !to_double(line->tokens[6], blade_angle) ||
        !to_double(line->tokens[7], blade_length) || !to_uint(line->tokens[8], arc_count)) {
        return false;
    }

    struct LegacyArc {
        double radius{0.0};
        double length{0.0};
        double transition_to_next{0.0};
    };
    std::vector<LegacyArc> arcs;
    for (std::uint32_t k = 0; k < arc_count; ++k) {
        const std::optional<Line> arc_line = reader.next("tarc", 4);
        LegacyArc arc;
        if (!arc_line || !to_double(arc_line->tokens[1], arc.radius) ||
            !to_double(arc_line->tokens[2], arc.length) ||
            !to_double(arc_line->tokens[3], arc.transition_to_next)) {
            return false;
        }
        arcs.push_back(arc);
    }
    if (arcs.empty() || arcs.front().radius <= 0.0 || arcs.back().radius <= 0.0) {
        return false;
    }

    const double alfa = type.crossing_n > 0.0 ? std::atan(1.0 / type.crossing_n) : 0.0;
    // heading and through-projection are accumulated as the pieces are appended, so
    // the closing arc and the frog rail can be given their old lengths
    double heading = 0.0;
    double projection = 0.0;  // along the through track
    double offset = 0.0;      // away from it
    const auto append = [&](int part, double length, double r0, double r1, double turn_in) {
        if (length <= 0.0) {
            return;
        }
        type.pieces.push_back(TurnoutPieceSpec{part, length, r0, r1, turn_in});
        heading += turn_in;
        const double k0 = r0 > 0.0 ? 1.0 / r0 : 0.0;
        const double k1 = r1 > 0.0 ? 1.0 / r1 : 0.0;
        const double turned = 0.5 * (k0 + k1) * length;
        // a straight projects whole; a turning piece projects its chord, which is
        // shorter than its length by sin(theta/2)/(theta/2) and points at half the turn
        const double half = 0.5 * turned;
        const double chord = std::abs(half) < 1e-12 ? length : length * std::sin(half) / half;
        projection += chord * std::cos(heading + half);
        offset += chord * std::sin(heading + half);
        heading += turned;
    };

    append(0, pre_blade, 0.0, 0.0, 0.0);
    append(1, blade_length, 0.0, 0.0, blade_angle);
    double pending_turn = blade_length > 0.0 ? 0.0 : blade_angle;
    if (entry_transition > 0.0) {
        append(2, entry_transition, 0.0, arcs.front().radius, pending_turn);
        pending_turn = 0.0;
    }
    for (std::size_t k = 0; k + 1 < arcs.size(); ++k) {
        append(2, arcs[k].length, arcs[k].radius, arcs[k].radius, pending_turn);
        pending_turn = 0.0;
        append(2, arcs[k].transition_to_next, arcs[k].radius, arcs[k + 1].radius, 0.0);
    }
    const double closing = alfa - heading - 0.5 * exit_transition / arcs.back().radius;
    append(2, closing * arcs.back().radius, arcs.back().radius, arcs.back().radius,
           pending_turn);
    append(2, exit_transition, arcs.back().radius, 0.0, 0.0);

    const double rail = type.length > 0.0 && std::cos(alfa) > 1e-6
                            ? (type.length - projection) / std::cos(alfa)
                            : arcs.back().radius * std::tan(alfa * 0.5);
    append(3, rail, 0.0, 0.0, 0.0);

    // version 3 called the length the projection on the through track; version 4 uses
    // the catalogue's own a + b, so it is restated here from where the frog ended up
    if (std::abs(std::sin(alfa)) > 1e-9) {
        const double back = offset / std::sin(alfa);
        type.length = projection - back * std::cos(alfa) + back;
    }
    return !type.pieces.empty();
}

}  // namespace

// ---------------------------------------------------------------------------

std::string serialize(const Document& document) {
    std::string out;
    out += "m0s " + std::to_string(kVersion) + "\n";
    out += "next " + num(document.next_id) + "\n";
    out += "view " + num(document.view_x) + " " + num(document.view_y) + " " +
           num(document.view_extent) + "\n";
    out += "origin " + std::string(document.origin_set ? "1" : "0") + " " +
           std::string(document.georeferenced ? "1" : "0") + " " + num(document.origin_x) +
           " " + num(document.origin_y) + "\n";
    // the tail is kept whole, so a path may have spaces in it like any other name
    out += "scn " + document.scn_path + "\n";

    out += "types " + std::to_string(document.turnout_types.size()) + "\n";
    for (const auto& type : document.turnout_types) {
        out += "type " + num(type.crossing_n) + " " + num(type.length) + " " +
               num(type.blade.tip_thickness) + " " + num(type.blade.nose) + " " +
               num(type.blade.railtop_width) + " " + std::to_string(type.pieces.size()) +
               "\n";
        for (const auto& piece : type.pieces) {
            out += "tseg " + std::to_string(piece.part) + " " + num(piece.length) + " " +
                   num(piece.radius_start) + " " + num(piece.radius_end) + " " +
                   num(piece.turn_in) + "\n";
        }
        out += "tname " + type.name + "\n";
    }

    out += "turnouts " + std::to_string(document.turnouts.size()) + "\n";
    for (const auto& turnout : document.turnouts) {
        out += "turnout " + num(static_cast<std::uint32_t>(turnout.id)) + " " +
               num(static_cast<std::uint32_t>(turnout.on)) + " " + num(turnout.station) +
               " " + std::to_string(turnout.hand) + " " +
               std::string(turnout.facing ? "1" : "0") + " " +
               std::string(turnout.bend_from_track ? "1" : "0") + " " + num(turnout.bend) +
               "\n";
        out += "tutype " + turnout.type + "\n";
    }

    out += "tracks " + std::to_string(document.tracks.size()) + "\n";
    for (const auto& track : document.tracks) {
        out += "track " + num(static_cast<std::uint32_t>(track.id)) + "\n";
        out += "tkname " + track.name + "\n";
        if (const auto* pose = std::get_if<AtPose>(&track.anchor)) {
            out += "anchor pose " + num(pose->x) + " " + num(pose->y) + " " + num(pose->az) +
                   "\n";
        } else {
            const auto& port = std::get<AtPort>(track.anchor);
            out += "anchor port " + num(static_cast<std::uint32_t>(port.turnout)) + " " +
                   std::string(port.port == Port::Frog ? "frog" : "start") + "\n";
        }
        out += "elems " + std::to_string(track.elements.size()) + "\n";
        for (const auto& element : track.elements) {
            const char* kind = element.kind == Kind::Line     ? "line"
                               : element.kind == Kind::Arc    ? "arc"
                                                              : "clot";
            out += "elem " + num(static_cast<std::uint32_t>(element.id)) + " " + kind + " " +
                   num(element.radius) + " " + std::to_string(element.hand) + " " +
                   num(element.length) + "\n";
            if (const auto* parallel = std::get_if<Parallel>(&element.hold)) {
                out += "epar " + num(static_cast<std::uint32_t>(parallel->ref)) + " " +
                       num(parallel->offset) + "\n";
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------------------

std::optional<Document> deserialize(const std::string& text) {
    Reader reader{text};
    Document document;

    int version = 0;
    {
        const std::optional<Line> header = reader.next("m0s", 2);
        if (!header || !to_int(header->tokens[1], version) ||
            (version != kVersion && version != kNoExportPath && version != kStraightOnly &&
             version != kNoseless && version != kLegacyTurnoutVersion)) {
            return std::nullopt;
        }
    }
    {
        const std::optional<Line> line = reader.next("next", 2);
        if (!line || !to_uint(line->tokens[1], document.next_id)) {
            return std::nullopt;
        }
    }
    {
        const std::optional<Line> line = reader.next("view", 4);
        if (!line || !to_double(line->tokens[1], document.view_x) ||
            !to_double(line->tokens[2], document.view_y) ||
            !to_double(line->tokens[3], document.view_extent)) {
            return std::nullopt;
        }
    }
    {
        const std::optional<Line> line = reader.next("origin", 5);
        if (!line || !to_double(line->tokens[3], document.origin_x) ||
            !to_double(line->tokens[4], document.origin_y)) {
            return std::nullopt;
        }
        document.origin_set = line->tokens[1] == "1";
        document.georeferenced = line->tokens[2] == "1";
    }
    if (version > kNoExportPath) {
        // an older plan never said where it exported to, and the host has its own
        // answer for that
        const std::optional<Line> line = reader.next("scn", 1);
        if (!line) {
            return std::nullopt;
        }
        document.scn_path = std::string{line->tail};
    }

    const auto count_of = [&](std::string_view tag, std::size_t& out) {
        const std::optional<Line> line = reader.next(tag, 2);
        std::uint32_t value = 0;
        if (!line || !to_uint(line->tokens[1], value)) {
            return false;
        }
        out = value;
        return true;
    };

    std::size_t count = 0;

    // ---- turnout types ----------------------------------------------------
    if (!count_of("types", count)) {
        return std::nullopt;
    }
    for (std::size_t i = 0; i < count; ++i) {
        TurnoutType type;
        if (version == kLegacyTurnoutVersion) {
            if (!read_legacy_type(reader, type)) {
                return std::nullopt;
            }
        } else {
            // the nose (d) came late: before it, a type line went straight from the tip
            // thickness to the railhead width
            const bool has_nose = version > kNoseless;
            const std::optional<Line> line = reader.next("type", has_nose ? 7 : 6);
            std::uint32_t piece_count = 0;
            if (!line || !to_double(line->tokens[1], type.crossing_n) ||
                !to_double(line->tokens[2], type.length) ||
                !to_double(line->tokens[3], type.blade.tip_thickness) ||
                (has_nose && !to_double(line->tokens[4], type.blade.nose)) ||
                !to_double(line->tokens[has_nose ? 5 : 4], type.blade.railtop_width) ||
                !to_uint(line->tokens[has_nose ? 6 : 5], piece_count)) {
                return std::nullopt;
            }
            for (std::uint32_t k = 0; k < piece_count; ++k) {
                const std::optional<Line> piece_line = reader.next("tseg", 6);
                TurnoutPieceSpec piece;
                if (!piece_line || !to_int(piece_line->tokens[1], piece.part) ||
                    !to_double(piece_line->tokens[2], piece.length) ||
                    !to_double(piece_line->tokens[3], piece.radius_start) ||
                    !to_double(piece_line->tokens[4], piece.radius_end) ||
                    !to_double(piece_line->tokens[5], piece.turn_in)) {
                    return std::nullopt;
                }
                type.pieces.push_back(piece);
            }
        }
        const std::optional<Line> name = reader.next("tname", 1);
        if (!name) {
            return std::nullopt;
        }
        type.name = std::string{name->tail};
        document.turnout_types.push_back(std::move(type));
    }

    // ---- turnouts ---------------------------------------------------------
    if (!count_of("turnouts", count)) {
        return std::nullopt;
    }
    for (std::size_t i = 0; i < count; ++i) {
        // łukowanie came late: before it every placement stood on the track as it
        // found it, which is what taking the bend from the track means anyway
        const bool has_bend = version > kStraightOnly;
        const std::optional<Line> line = reader.next("turnout", has_bend ? 8 : 6);
        TurnoutPlacement placement;
        std::uint32_t id = 0;
        std::uint32_t on = 0;
        if (!line || !to_uint(line->tokens[1], id) || !to_uint(line->tokens[2], on) ||
            !to_double(line->tokens[3], placement.station) ||
            !to_int(line->tokens[4], placement.hand)) {
            return std::nullopt;
        }
        placement.id = static_cast<TurnoutId>(id);
        placement.on = static_cast<TrackId>(on);
        placement.facing = line->tokens[5] == "1";
        if (has_bend) {
            placement.bend_from_track = line->tokens[6] == "1";
            if (!to_double(line->tokens[7], placement.bend)) {
                return std::nullopt;
            }
        }
        const std::optional<Line> type = reader.next("tutype", 1);
        if (!type) {
            return std::nullopt;
        }
        placement.type = std::string{type->tail};
        document.turnouts.push_back(std::move(placement));
    }

    // ---- tracks -----------------------------------------------------------
    if (!count_of("tracks", count)) {
        return std::nullopt;
    }
    for (std::size_t i = 0; i < count; ++i) {
        const std::optional<Line> head = reader.next("track", 2);
        Track track;
        std::uint32_t id = 0;
        if (!head || !to_uint(head->tokens[1], id)) {
            return std::nullopt;
        }
        track.id = static_cast<TrackId>(id);

        const std::optional<Line> name = reader.next("tkname", 1);
        if (!name) {
            return std::nullopt;
        }
        track.name = std::string{name->tail};

        const std::optional<Line> anchor = reader.next("anchor", 2);
        if (!anchor) {
            return std::nullopt;
        }
        if (anchor->tokens[1] == "pose" && anchor->size() >= 5) {
            AtPose pose;
            if (!to_double(anchor->tokens[2], pose.x) ||
                !to_double(anchor->tokens[3], pose.y) ||
                !to_double(anchor->tokens[4], pose.az)) {
                return std::nullopt;
            }
            track.anchor = pose;
        } else if (anchor->tokens[1] == "port" && anchor->size() >= 4) {
            std::uint32_t turnout_id = 0;
            if (!to_uint(anchor->tokens[2], turnout_id)) {
                return std::nullopt;
            }
            track.anchor = AtPort{static_cast<TurnoutId>(turnout_id),
                                  anchor->tokens[3] == "start" ? Port::Start : Port::Frog};
        } else {
            return std::nullopt;
        }

        std::size_t element_count = 0;
        if (!count_of("elems", element_count)) {
            return std::nullopt;
        }
        for (std::size_t k = 0; k < element_count; ++k) {
            const std::optional<Line> line = reader.next("elem", 6);
            Element element;
            std::uint32_t element_id = 0;
            if (!line || !to_uint(line->tokens[1], element_id) ||
                !to_double(line->tokens[3], element.radius) ||
                !to_int(line->tokens[4], element.hand) ||
                !to_double(line->tokens[5], element.length)) {
                return std::nullopt;
            }
            element.id = static_cast<ElementId>(element_id);
            if (line->tokens[2] == "line") {
                element.kind = Kind::Line;
            } else if (line->tokens[2] == "arc") {
                element.kind = Kind::Arc;
            } else if (line->tokens[2] == "clot") {
                element.kind = Kind::Clothoid;
            } else {
                return std::nullopt;
            }

            // A hold, when there is one, follows its element immediately.
            if (reader.peek_tag() == "epar") {
                const std::optional<Line> hold = reader.next("epar", 3);
                std::uint32_t ref = 0;
                Parallel parallel;
                if (!hold || !to_uint(hold->tokens[1], ref) ||
                    !to_double(hold->tokens[2], parallel.offset)) {
                    return std::nullopt;
                }
                parallel.ref = static_cast<ElementId>(ref);
                element.hold = parallel;
            }

            track.elements.push_back(element);
        }
        document.tracks.push_back(std::move(track));
    }

    return document;
}

// ---------------------------------------------------------------------------

std::string default_project_path() { return "editor/plan.m0s"; }

bool save(const Document& document, const std::string& path) {
    std::ofstream file{path, std::ios::binary | std::ios::trunc};
    if (!file) {
        return false;
    }
    const std::string text = serialize(document);
    file.write(text.data(), static_cast<std::streamsize>(text.size()));
    return static_cast<bool>(file);
}

std::optional<Document> load(const std::string& path) {
    std::ifstream file{path, std::ios::binary};
    if (!file) {
        return std::nullopt;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return deserialize(buffer.str());
}

}  // namespace editor::plan::io
