#include "maj0sted/app/editor_document.hpp"

#include <cctype>
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "maj0sted/io/project_serializer.hpp"
#include "maj0sted/maj0sted.hpp"

namespace maj0sted::app {

using namespace maj0sted::domain;
using maj0sted::editor::GapFit;
using maj0sted::editor::NiweletaSpec;
using maj0sted::editor::StraightSpec;

namespace {

// Shortest round-trippable, locale-independent double — matches maj0sted::io.
std::string num(double value) {
    char buffer[64];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
    return std::string(buffer, result.ptr);
}

std::vector<std::string> tokenize(const std::string& line) {
    std::vector<std::string> tokens;
    std::size_t i = 0;
    const std::size_t n = line.size();
    while (i < n) {
        while (i < n && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
        std::size_t j = i;
        while (j < n && !std::isspace(static_cast<unsigned char>(line[j]))) ++j;
        if (j > i) tokens.push_back(line.substr(i, j - i));
        i = j;
    }
    return tokens;
}

double to_double(const std::string& text) {
    double value{};
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{}) {
        throw std::runtime_error{"maj0sted: bad number '" + text + "'"};
    }
    return value;
}

long to_long(const std::string& text) {
    long value{};
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{}) {
        throw std::runtime_error{"maj0sted: bad integer '" + text + "'"};
    }
    return value;
}

// Placeholder turn direction for the resolved-domain view: derived from the two
// straights' headings (sign of the cross product).
TurnDirection gap_direction(const StraightSpec& a, const StraightSpec& b) noexcept {
    const double ax = a.x2 - a.x1, ay = a.y2 - a.y1;
    const double bx = b.x2 - b.x1, by = b.y2 - b.y1;
    return (ax * by - ay * bx) >= 0.0 ? TurnDirection::Left : TurnDirection::Right;
}

double gap_radius(const GapFit* fit) noexcept {
    if (fit != nullptr) {
        if (fit->radius > 0.0) return fit->radius;
        if (!fit->arcs.empty() && fit->arcs.front().radius > 0.0) {
            return fit->arcs.front().radius;
        }
        if (fit->r1 > 0.0) return fit->r1;
    }
    return 300.0;  // benign placeholder; arcs carry no length in this view
}

}  // namespace

MapProject to_map_project(const EditorDocument& document) {
    MapProject project{Crs::default_crs()};

    for (std::size_t index = 0; index < document.niwelety.size(); ++index) {
        const NiweletaSpec& spec = document.niwelety[index];
        try {
            Niweleta niweleta{NiweletaId{static_cast<std::uint64_t>(index + 1)},
                              spec.name};
            for (std::size_t i = 0; i < spec.straights.size(); ++i) {
                const StraightSpec& s = spec.straights[i];
                if (i > 0) {
                    // Keep two anchored straights from being adjacent (which the
                    // alignment forbids) by threading a floating placeholder arc.
                    const GapFit* fit = nullptr;
                    for (const auto& f : spec.fits) {
                        if (static_cast<std::size_t>(f.gap) == i - 1 && f.mode > 0) {
                            fit = &f;
                            break;
                        }
                    }
                    niweleta.add_plan_element(
                        CircularArc{Radius::from_metres(gap_radius(fit)),
                                    gap_direction(spec.straights[i - 1], s)});
                }
                niweleta.add_plan_element(
                    Straight{CartesianPosition{s.x1, s.y1}, CartesianPosition{s.x2, s.y2}});
            }
            project.add_niweleta(std::move(niweleta));
        } catch (...) {
            // Degenerate/inconsistent niweleta: keep the (named) shell so the
            // project still lists it; the editor section restores the details.
            project.add_niweleta(Niweleta{
                NiweletaId{static_cast<std::uint64_t>(index + 1)}, spec.name});
        }
    }
    return project;
}

EditorDocument from_map_project(const MapProject& project) {
    EditorDocument document;
    for (const auto& niweleta : project.niwelety()) {
        NiweletaSpec spec;
        spec.name = niweleta.name();
        for (const auto& element : niweleta.plan().elements()) {
            if (const auto* straight = std::get_if<Straight>(&element)) {
                spec.straights.push_back(StraightSpec{
                    straight->start().x(), straight->start().y(),
                    straight->end().x(), straight->end().y()});
            }
        }
        document.niwelety.push_back(std::move(spec));
    }
    return document;
}

std::string serialize_document(const EditorDocument& document) {
    // Reuse the domain serializer for a real, inspectable maj0sted project...
    std::string out = maj0sted::io::serialize(to_map_project(document));

    // ...then the lossless editor section (straights + fit parameters).
    out += "editor " + std::to_string(document.niwelety.size()) + "\n";
    if (document.view_extent > 0.0) {
        out += "eview " + num(document.view_x) + " " + num(document.view_y) + " " +
               num(document.view_extent) + "\n";
    }
    if (document.origin_set) {
        out += "eorigin " + (document.georeferenced ? std::string{"1"} : "0") + " " +
               num(document.origin_x) + " " + num(document.origin_y) + "\n";
    }
    for (const auto& spec : document.niwelety) {
        out += "eniw " + std::to_string(spec.straights.size()) + " " +
               std::to_string(spec.fits.size()) + "\n";
        out += "ename " + spec.name + "\n";
        for (const auto& s : spec.straights) {
            out += "estr " + num(s.x1) + " " + num(s.y1) + " " + num(s.x2) + " " +
                   num(s.y2) + " " + (s.hidden ? "1" : "0") + "\n";
        }
        for (std::size_t i = 0; i < spec.straights.size(); ++i) {
            const auto& s = spec.straights[i];
            if (s.rel_kind == 1) {
                out += "epar " + std::to_string(i) + " " +
                       std::to_string(s.rel_niw) + " " + std::to_string(s.rel_str) +
                       " " + num(s.rel_offset) + "\n";
            } else if (s.rel_kind == 2) {
                out += "eskew " + std::to_string(i) + " " +
                       std::to_string(s.rel_niw) + " " + std::to_string(s.rel_str) +
                       " " + num(s.rel_cot) + " " + std::to_string(s.rel_side) + " " +
                       num(s.rel_length) + "\n";
            }
        }
        for (const auto& f : spec.fits) {
            // efit2: gap mode radius transition entry_t exit_t <n> [r len trans]*
            // The variable arc list supersedes the legacy fixed two-arc 'efit'.
            out += "efit2 " + std::to_string(f.gap) + " " +
                   std::to_string(f.mode) + " " + num(f.radius) + " " +
                   num(f.transition) + " " + num(f.entry_t) + " " + num(f.exit_t) +
                   " " + std::to_string(f.arcs.size());
            for (const auto& a : f.arcs) {
                out += " " + num(a.radius) + " " + num(a.length) + " " +
                       num(a.transition_to_next);
            }
            out += "\n";
        }
    }
    // switches: through/station/side/facing/skew/branch, then the internal curve
    // inline in the same shape as an efit2 tail (mode radius trans entry exit <n> …)
    out += "ejuncs " + std::to_string(document.junctions.size()) + "\n";
    for (const auto& j : document.junctions) {
        out += "ejunc " + std::to_string(j.through) + " " + num(j.station) + " " +
               std::to_string(j.side) + " " + (j.facing ? std::string{"1"} : "0") +
               " " + num(j.crossing_n) + " " + std::to_string(j.branch) + " " +
               std::to_string(j.curve.mode) + " " + num(j.curve.radius) + " " +
               num(j.curve.transition) + " " + num(j.curve.entry_t) + " " +
               num(j.curve.exit_t) + " " + std::to_string(j.curve.arcs.size());
        for (const auto& a : j.curve.arcs) {
            out += " " + num(a.radius) + " " + num(a.length) + " " +
                   num(a.transition_to_next);
        }
        // catalogue length trails the arcs, so files written before it still parse (length stays 0)
        out += " " + num(j.length);
        out += "\n";
    }
    return out;
}

EditorDocument deserialize_document(const std::string& text) {
    std::vector<std::string> lines;
    {
        std::istringstream in(text);
        std::string line;
        while (std::getline(in, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            lines.push_back(line);
        }
    }

    // Locate the editor section (the domain block before it is interop-only).
    std::size_t cursor = lines.size();
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const auto tokens = tokenize(lines[i]);
        if (!tokens.empty() && tokens[0] == "editor") {
            cursor = i;
            break;
        }
    }
    if (cursor >= lines.size()) {
        throw std::runtime_error{"maj0sted: missing 'editor' section"};
    }

    const auto next = [&]() -> const std::string& {
        while (cursor < lines.size()) {
            const std::string& l = lines[cursor++];
            bool only_ws = true;
            for (const char c : l) {
                if (!std::isspace(static_cast<unsigned char>(c))) {
                    only_ws = false;
                    break;
                }
            }
            if (!only_ws) return l;
        }
        throw std::runtime_error{"maj0sted: unexpected end of editor section"};
    };

    EditorDocument document;
    const auto header = tokenize(next());
    if (header.size() < 2 || header[0] != "editor") {
        throw std::runtime_error{"maj0sted: expected 'editor <count>'"};
    }
    const long niweleta_count = to_long(header[1]);
    // Optional camera/zoom line (absent in older files).
    {
        std::size_t peek = cursor;
        while (peek < lines.size()) {
            bool only_ws = true;
            for (const char c : lines[peek]) {
                if (!std::isspace(static_cast<unsigned char>(c))) {
                    only_ws = false;
                    break;
                }
            }
            if (!only_ws) {
                break;
            }
            ++peek;
        }
        if (peek < lines.size()) {
            const auto view_tok = tokenize(lines[peek]);
            if (view_tok.size() >= 4 && view_tok[0] == "eview") {
                next();
                document.view_x = to_double(view_tok[1]);
                document.view_y = to_double(view_tok[2]);
                document.view_extent = to_double(view_tok[3]);
            }
        }
        // Optional local-zero line (absent in older files). May follow eview.
        peek = cursor;
        while (peek < lines.size()) {
            bool only_ws = true;
            for (const char c : lines[peek]) {
                if (!std::isspace(static_cast<unsigned char>(c))) {
                    only_ws = false;
                    break;
                }
            }
            if (!only_ws) {
                break;
            }
            ++peek;
        }
        if (peek < lines.size()) {
            const auto origin_tok = tokenize(lines[peek]);
            if (origin_tok.size() >= 4 && origin_tok[0] == "eorigin") {
                next();
                document.origin_set = true;
                document.georeferenced = to_long(origin_tok[1]) != 0;
                document.origin_x = to_double(origin_tok[2]);
                document.origin_y = to_double(origin_tok[3]);
            }
        }
    }
    for (long n = 0; n < niweleta_count; ++n) {
        const auto eniw = tokenize(next());
        if (eniw.size() < 3 || eniw[0] != "eniw") {
            throw std::runtime_error{"maj0sted: expected 'eniw <straights> <fits>'"};
        }
        const long straight_count = to_long(eniw[1]);
        const long fit_count = to_long(eniw[2]);

        NiweletaSpec spec;
        const std::string& name_line = next();
        if (name_line.rfind("ename", 0) != 0) {
            throw std::runtime_error{"maj0sted: expected 'ename'"};
        }
        spec.name = name_line.size() > 6 ? name_line.substr(6) : std::string{};

        for (long i = 0; i < straight_count; ++i) {
            const auto e = tokenize(next());
            if (e.size() < 5 || e[0] != "estr") {
                throw std::runtime_error{"maj0sted: bad 'estr' line"};
            }
            spec.straights.push_back(
                StraightSpec{to_double(e[1]), to_double(e[2]), to_double(e[3]),
                             to_double(e[4]), e.size() >= 6 && to_long(e[5]) != 0});
        }
        // Optional parallel/skew lines sit between straights and fits; older files omit them.
        auto apply_relation = [&](const std::vector<std::string>& e) -> bool {
            if (e.empty()) return false;
            if (e[0] == "epar" && e.size() >= 5) {
                const long idx = to_long(e[1]);
                if (idx < 0 || static_cast<std::size_t>(idx) >= spec.straights.size()) {
                    throw std::runtime_error{"maj0sted: bad 'epar' index"};
                }
                auto& s = spec.straights[static_cast<std::size_t>(idx)];
                s.rel_kind = 1;
                s.rel_niw = static_cast<int>(to_long(e[2]));
                s.rel_str = static_cast<int>(to_long(e[3]));
                s.rel_offset = to_double(e[4]);
                return true;
            }
            if (e[0] == "eskew" && e.size() >= 7) {
                const long idx = to_long(e[1]);
                if (idx < 0 || static_cast<std::size_t>(idx) >= spec.straights.size()) {
                    throw std::runtime_error{"maj0sted: bad 'eskew' index"};
                }
                auto& s = spec.straights[static_cast<std::size_t>(idx)];
                s.rel_kind = 2;
                s.rel_niw = static_cast<int>(to_long(e[2]));
                s.rel_str = static_cast<int>(to_long(e[3]));
                s.rel_cot = to_double(e[4]);
                s.rel_side = static_cast<int>(to_long(e[5]));
                s.rel_length = to_double(e[6]);
                return true;
            }
            return false;
        };
        auto peek_tokens = [&]() -> std::vector<std::string> {
            std::size_t i = cursor;
            while (i < lines.size()) {
                bool only_ws = true;
                for (const char c : lines[i]) {
                    if (!std::isspace(static_cast<unsigned char>(c))) {
                        only_ws = false;
                        break;
                    }
                }
                if (!only_ws) {
                    return tokenize(lines[i]);
                }
                ++i;
            }
            return {};
        };
        while (apply_relation(peek_tokens())) {
            next(); // consume the relation line
        }
        for (long i = 0; i < fit_count; ++i) {
            const auto e = tokenize(next());
            GapFit f;
            if (!e.empty() && e[0] == "efit2") {
                // New variable-arc format.
                if (e.size() < 8) {
                    throw std::runtime_error{"maj0sted: bad 'efit2' line"};
                }
                f.gap = static_cast<int>(to_long(e[1]));
                f.mode = static_cast<int>(to_long(e[2]));
                f.radius = to_double(e[3]);
                f.transition = to_double(e[4]);
                f.entry_t = to_double(e[5]);
                f.exit_t = to_double(e[6]);
                const long arc_count = to_long(e[7]);
                for (long a = 0; a < arc_count; ++a) {
                    const std::size_t base = 8 + static_cast<std::size_t>(a) * 3;
                    if (base + 2 >= e.size()) {
                        throw std::runtime_error{"maj0sted: truncated 'efit2' arcs"};
                    }
                    f.arcs.push_back(maj0sted::editor::CompoundArcSpec{
                        to_double(e[base]), to_double(e[base + 1]),
                        to_double(e[base + 2])});
                }
            } else if (!e.empty() && e[0] == "efit" && e.size() >= 11) {
                // Legacy fixed two-arc format; arcs stay empty and to_request
                // falls back to the r1/r2 scalar fields.
                f.gap = static_cast<int>(to_long(e[1]));
                f.mode = static_cast<int>(to_long(e[2]));
                f.radius = to_double(e[3]);
                f.transition = to_double(e[4]);
                f.r1 = to_double(e[5]);
                f.arc1_len = to_double(e[6]);
                f.between = to_double(e[7]);
                f.r2 = to_double(e[8]);
                f.entry_t = to_double(e[9]);
                f.exit_t = to_double(e[10]);
            } else {
                throw std::runtime_error{"maj0sted: bad 'efit'/'efit2' line"};
            }
            spec.fits.push_back(f);
        }
        document.niwelety.push_back(std::move(spec));
    }
    // Optional switches section (absent in older files); mirrors the eview/eorigin peek.
    {
        std::size_t peek = cursor;
        while (peek < lines.size()) {
            bool only_ws = true;
            for (const char c : lines[peek]) {
                if (!std::isspace(static_cast<unsigned char>(c))) {
                    only_ws = false;
                    break;
                }
            }
            if (!only_ws) break;
            ++peek;
        }
        if (peek < lines.size()) {
            const auto head = tokenize(lines[peek]);
            if (head.size() >= 2 && head[0] == "ejuncs") {
                next();  // consume the 'ejuncs <count>' header
                const long junction_count = to_long(head[1]);
                for (long j = 0; j < junction_count; ++j) {
                    const auto e = tokenize(next());
                    if (e.size() < 12 || e[0] != "ejunc") {
                        throw std::runtime_error{"maj0sted: bad 'ejunc' line"};
                    }
                    maj0sted::editor::Junction junction;
                    junction.through = static_cast<int>(to_long(e[1]));
                    junction.station = to_double(e[2]);
                    junction.side = static_cast<int>(to_long(e[3]));
                    junction.facing = to_long(e[4]) != 0;
                    junction.crossing_n = to_double(e[5]);
                    junction.branch = static_cast<int>(to_long(e[6]));
                    junction.curve.mode = static_cast<int>(to_long(e[7]));
                    junction.curve.radius = to_double(e[8]);
                    junction.curve.transition = to_double(e[9]);
                    junction.curve.entry_t = to_double(e[10]);
                    junction.curve.exit_t = to_double(e[11]);
                    const long arc_count = e.size() >= 13 ? to_long(e[12]) : 0;
                    for (long a = 0; a < arc_count; ++a) {
                        const std::size_t base = 13 + static_cast<std::size_t>(a) * 3;
                        if (base + 2 >= e.size()) {
                            throw std::runtime_error{"maj0sted: truncated 'ejunc' arcs"};
                        }
                        junction.curve.arcs.push_back(maj0sted::editor::CompoundArcSpec{
                            to_double(e[base]), to_double(e[base + 1]),
                            to_double(e[base + 2])});
                    }
                    // optional catalogue length after the arcs (absent in the earliest files)
                    const std::size_t after_arcs = 13 + static_cast<std::size_t>(arc_count) * 3;
                    if (e.size() > after_arcs) {
                        junction.length = to_double(e[after_arcs]);
                    }
                    document.junctions.push_back(std::move(junction));
                }
            }
        }
    }
    return document;
}

std::string default_project_path() {
    // Fallback only; the editor passes its own path (see plan_panel). Relative, so
    // it lands under the working directory rather than at a filesystem root.
    return "editor/project.m0s";
}

bool save_project(const EditorDocument& document, const std::string& path) {
    try {
        const std::filesystem::path file{path};
        if (file.has_parent_path()) {
            std::filesystem::create_directories(file.parent_path());
        }
        std::ofstream out(file, std::ios::binary | std::ios::trunc);
        if (!out) return false;
        out << serialize_document(document);
        return static_cast<bool>(out);
    } catch (...) {
        return false;
    }
}

std::optional<EditorDocument> load_project(const std::string& path) {
    try {
        const std::filesystem::path file{path};
        if (!std::filesystem::exists(file)) return std::nullopt;
        std::ifstream in(file, std::ios::binary);
        if (!in) return std::nullopt;
        std::ostringstream buffer;
        buffer << in.rdbuf();
        return deserialize_document(buffer.str());
    } catch (...) {
        return std::nullopt;
    }
}

}  // namespace maj0sted::app
