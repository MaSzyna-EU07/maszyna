#include "maj0sted/io/project_serializer.hpp"

#include <cctype>
#include <charconv>
#include <cstdint>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <variant>
#include <vector>

#include "maj0sted/maj0sted.hpp"

namespace maj0sted::io {

using namespace maj0sted::domain;

namespace {

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// --- writing -------------------------------------------------------------

// Shortest round-trippable, locale-independent representation of a double.
std::string num(double value) {
    char buffer[64];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
    return std::string(buffer, result.ptr);
}

char dir_char(TurnDirection direction) {
    return direction == TurnDirection::Left ? 'L' : 'R';
}
char joint_char(JointContinuity joint) {
    return joint == JointContinuity::AzimuthBreak ? 'B' : 'T';
}
char curvature_char(VerticalCurvature curvature) {
    return curvature == VerticalCurvature::Crest ? 'C' : 'S';
}

std::string optional_metres(const std::optional<Length>& length) {
    return length ? num(length->metres()) : std::string{"-"};
}
std::string optional_metres(const std::optional<Radius>& radius) {
    return radius ? num(radius->metres()) : std::string{"-"};
}

// --- reading -------------------------------------------------------------

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

std::uint64_t to_u64(const std::string& text) {
    std::uint64_t value{};
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{}) {
        throw std::runtime_error{"maj0sted: bad integer '" + text + "'"};
    }
    return value;
}

TurnDirection to_direction(const std::string& text) {
    if (text == "L") return TurnDirection::Left;
    if (text == "R") return TurnDirection::Right;
    throw std::runtime_error{"maj0sted: bad direction '" + text + "'"};
}
JointContinuity to_joint(const std::string& text) {
    if (text == "T") return JointContinuity::Tangent;
    if (text == "B") return JointContinuity::AzimuthBreak;
    throw std::runtime_error{"maj0sted: bad joint '" + text + "'"};
}
VerticalCurvature to_curvature(const std::string& text) {
    if (text == "C") return VerticalCurvature::Crest;
    if (text == "S") return VerticalCurvature::Sag;
    throw std::runtime_error{"maj0sted: bad curvature '" + text + "'"};
}

std::optional<Length> to_optional_length(const std::string& text) {
    if (text == "-") return std::nullopt;
    return Length::from_metres(to_double(text));
}
std::optional<Radius> to_optional_radius(const std::string& text) {
    if (text == "-") return std::nullopt;
    return Radius::from_metres(to_double(text));
}

}  // namespace

std::string serialize(const MapProject& project) {
    std::string out;
    out += "maj0sted 1\n";
    out += "crs " + std::to_string(project.crs().epsg()) + "\n";
    out += "niwelety " + std::to_string(project.niwelety().size()) + "\n";

    for (const auto& niweleta : project.niwelety()) {
        out += "niweleta " + std::to_string(niweleta.id().value()) + "\n";
        out += "name " + niweleta.name() + "\n";

        const auto& plan = niweleta.plan();
        out += "plan " + std::to_string(plan.size()) + "\n";
        for (std::size_t k = 0; k < plan.elements().size(); ++k) {
            const char joint = (k == 0) ? 'T' : joint_char(plan.joint(k - 1));
            std::visit(
                overloaded{
                    [&](const Straight& s) {
                        out += "straight " + std::to_string(s.id().value()) + " " +
                               num(s.start().x()) + " " + num(s.start().y()) + " " +
                               num(s.end().x()) + " " + num(s.end().y()) + " " + joint +
                               "\n";
                    },
                    [&](const CircularArc& a) {
                        out += "arc " + num(a.radius().metres()) + " " +
                               dir_char(a.direction()) + " " + optional_metres(a.length()) +
                               " " + joint + "\n";
                    },
                    [&](const TransitionCurve& t) {
                        out += "transition " + num(t.length().metres()) + " " +
                               optional_metres(t.start_radius()) + " " +
                               optional_metres(t.end_radius()) + " " +
                               dir_char(t.direction()) + " " + joint + "\n";
                    },
                },
                plan.elements()[k]);
        }

        const auto& profile = niweleta.profile();
        out += "profile " + std::to_string(profile.size()) + "\n";
        for (const auto& element : profile.elements()) {
            std::visit(
                overloaded{
                    [&](const Grade& g) {
                        out += "grade " + num(g.length().metres()) + " " +
                               num(g.slope_permille()) + "\n";
                    },
                    [&](const VerticalCurve& v) {
                        out += "vcurve " + num(v.length().metres()) + " " +
                               num(v.radius().metres()) + " " +
                               curvature_char(v.curvature()) + "\n";
                    },
                },
                element);
        }
    }
    return out;
}

MapProject deserialize(const std::string& text) {
    // Split into non-blank lines, stripping any trailing '\r'.
    std::vector<std::string> lines;
    {
        std::istringstream in(text);
        std::string line;
        while (std::getline(in, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            bool only_ws = true;
            for (const char c : line) {
                if (!std::isspace(static_cast<unsigned char>(c))) {
                    only_ws = false;
                    break;
                }
            }
            if (!only_ws) lines.push_back(line);
        }
    }

    std::size_t cursor = 0;
    const auto next_line = [&]() -> const std::string& {
        if (cursor >= lines.size()) {
            throw std::runtime_error{"maj0sted: unexpected end of data"};
        }
        return lines[cursor++];
    };
    const auto expect = [&](const std::vector<std::string>& tokens, const char* tag,
                            std::size_t min_size) {
        if (tokens.size() < min_size || tokens[0] != tag) {
            throw std::runtime_error{std::string{"maj0sted: expected '"} + tag + "'"};
        }
    };

    {
        const auto header = tokenize(next_line());
        expect(header, "maj0sted", 2);
    }
    const auto crs_line = tokenize(next_line());
    expect(crs_line, "crs", 2);
    const Crs crs{static_cast<int>(to_u64(crs_line[1]))};

    const auto count_line = tokenize(next_line());
    expect(count_line, "niwelety", 2);
    const std::size_t niweleta_count = static_cast<std::size_t>(to_u64(count_line[1]));

    MapProject project{crs};
    for (std::size_t index = 0; index < niweleta_count; ++index) {
        const auto header = tokenize(next_line());
        expect(header, "niweleta", 2);
        const NiweletaId id{to_u64(header[1])};

        const std::string& name_line = next_line();
        if (name_line.rfind("name", 0) != 0) {
            throw std::runtime_error{"maj0sted: expected 'name'"};
        }
        std::string name = name_line.size() > 5 ? name_line.substr(5) : std::string{};
        Niweleta niweleta{id, std::move(name)};

        const auto plan_line = tokenize(next_line());
        expect(plan_line, "plan", 2);
        const std::size_t plan_count = static_cast<std::size_t>(to_u64(plan_line[1]));
        for (std::size_t k = 0; k < plan_count; ++k) {
            const auto e = tokenize(next_line());
            if (e.empty()) throw std::runtime_error{"maj0sted: empty plan element"};
            if (e[0] == "straight") {
                expect(e, "straight", 7);
                niweleta.add_plan_element(
                    Straight{StraightId{to_u64(e[1])},
                             CartesianPosition{to_double(e[2]), to_double(e[3])},
                             CartesianPosition{to_double(e[4]), to_double(e[5])}},
                    to_joint(e[6]));
            } else if (e[0] == "arc") {
                expect(e, "arc", 5);
                niweleta.add_plan_element(
                    CircularArc{Radius::from_metres(to_double(e[1])), to_direction(e[2]),
                                to_optional_length(e[3])},
                    to_joint(e[4]));
            } else if (e[0] == "transition") {
                expect(e, "transition", 6);
                niweleta.add_plan_element(
                    TransitionCurve{Length::from_metres(to_double(e[1])),
                                    to_optional_radius(e[2]), to_optional_radius(e[3]),
                                    to_direction(e[4])},
                    to_joint(e[5]));
            } else {
                throw std::runtime_error{"maj0sted: unknown plan element '" + e[0] + "'"};
            }
        }

        const auto profile_line = tokenize(next_line());
        expect(profile_line, "profile", 2);
        const std::size_t profile_count =
            static_cast<std::size_t>(to_u64(profile_line[1]));
        for (std::size_t k = 0; k < profile_count; ++k) {
            const auto e = tokenize(next_line());
            if (e.empty()) throw std::runtime_error{"maj0sted: empty profile element"};
            if (e[0] == "grade") {
                expect(e, "grade", 3);
                niweleta.add_profile_element(
                    Grade{Length::from_metres(to_double(e[1])), to_double(e[2])});
            } else if (e[0] == "vcurve") {
                expect(e, "vcurve", 4);
                niweleta.add_profile_element(
                    VerticalCurve{Length::from_metres(to_double(e[1])),
                                  Radius::from_metres(to_double(e[2])),
                                  to_curvature(e[3])});
            } else {
                throw std::runtime_error{"maj0sted: unknown profile element '" + e[0] +
                                         "'"};
            }
        }

        project.add_niweleta(std::move(niweleta));
    }
    return project;
}

}  // namespace maj0sted::io
