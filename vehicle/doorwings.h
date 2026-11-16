/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <glm/vec3.hpp>

// the poses of the animated door wings of a vehicle, for the door position of the mover. a wing is posed alike
// in the main model and, when the low poly interior has a submodel of the same name, in the interior as well
namespace door_wings {

// the kinds of wing animation, numbered as DoorOpenMethod of the .fiz (TMoverParameters::door_data::type)
inline constexpr int shift_type{1};
inline constexpr int rotate_type{2};
inline constexpr int fold_type{3};
inline constexpr int plug_type{4};

// whether the animation with the given flags (TAnim::iFlags, the kind of animation in bits 4-7) moves a door wing
inline bool is_wing(int const Flags) {
	return (Flags & 0xF0) == 0x10;
}

// odd numbered wings (door1, door3...) are on the left side of the vehicle, even numbered on the right
inline bool on_right_side(int const Number) {
	return (Number & 1) == 0;
}

// sliding door (type 1): shifted along the vehicle
inline glm::vec3 slide(float const Position) {
	return {0.f, 0.f, Position};
}

// plug door (type 4): pushed out first, up to Rangeout, then slid along the vehicle
inline glm::vec3 plug(float const Position, float const Rangeout) {
	return {std::min(Position * 2.f, Rangeout), 0.f, std::max(0.f, Position - Rangeout * 0.5f)};
}

// folding door (type 3): the angles of the smaller wing, of the larger wing (its child) and of the step (the
// child of the larger wing)
struct fold_angles {
	float smaller;
	float larger;
	float step;
};

inline fold_angles fold(float const Position) {
	return {Position, -Position - Position, Position};
}

// the animation of one wing given in the .mmd, in place of the one the .fiz sets for all door wings of the vehicle
// (eu07/maszyna#70). the wing still follows the door of its side, over its own range:
//   animdoor: <wing number> { type: <Shift|Rotate|Fold|Plug> range: <shift [m] or angle [deg]> rangeout: <plug [m]> }
// every key is optional; a key left out keeps the value of the .fiz
struct wing_spec {
	int number{0}; // of the wing, as in animdoorprefix: door1 is 1
	int type{rotate_type};
	float range{0.f}; // shift or angle of the wing when the door is fully open
	float range_out{0.f}; // how far a plug wing is pushed out
};

// the kind of wing animation with the given name of DoorOpenMethod, in any letter case
inline std::optional<int> type_from_name(std::string_view const Name) {
	std::string name{Name};
	std::transform(name.begin(), name.end(), name.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	if (name == "shift") {
		return shift_type;
	}
	if (name == "rotate") {
		return rotate_type;
	}
	if (name == "fold") {
		return fold_type;
	}
	if (name == "plug") {
		return plug_type;
	}
	return std::nullopt;
}

// the number written in the whole of the text, or nothing
inline std::optional<float> number_from_text(std::string const &Text) {
	if (Text.empty()) {
		return std::nullopt;
	}
	char *end{nullptr};
	auto const value{std::strtof(Text.c_str(), &end)};
	if (end != Text.c_str() + Text.size() || false == std::isfinite(value)) {
		return std::nullopt;
	}
	return value;
}

// applies the keys and values of the block of an animdoor entry (the tokens between the braces) to the spec, which
// comes filled with the values of the .fiz. returns what is wrong with the block, or an empty text when nothing is;
// the keys before the fault are applied
inline std::string parse_spec(std::vector<std::string> const &Tokens, wing_spec &Spec) {
	for (std::size_t index = 0; index < Tokens.size(); index += 2) {
		std::string key{Tokens[index]};
		std::transform(key.begin(), key.end(), key.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
		if (index + 1 >= Tokens.size()) {
			return "no value for \"" + key + "\"";
		}
		auto const &value{Tokens[index + 1]};
		if (key == "type:") {
			auto const type{type_from_name(value)};
			if (false == type.has_value()) {
				return "unknown door type \"" + value + "\"";
			}
			Spec.type = *type;
		}
		else if (key == "range:" || key == "rangeout:") {
			auto const number{number_from_text(value)};
			if (false == number.has_value()) {
				return "bad number \"" + value + "\" for \"" + key + "\"";
			}
			(key == "range:" ? Spec.range : Spec.range_out) = *number;
		}
		else {
			return "unknown key \"" + key + "\"";
		}
	}
	return {};
}

// reads an animdoor entry after its key: the wing number and the block in braces, into the spec filled with the values
// of the .fiz. returns what is wrong with the entry, or an empty text when nothing is. the parser is a cParser
template <typename Parser_>
std::string read_spec(Parser_ &Parser, wing_spec &Spec) {
	Parser.getTokens(1, false);
	Parser >> Spec.number;
	std::string token;
	Parser.getTokens(1, false);
	Parser >> token;
	if (token != "{") {
		return "no { after the wing number";
	}
	std::vector<std::string> block;
	while (true) {
		token.clear();
		Parser.getTokens(1, false);
		Parser >> token;
		if (token.empty()) {
			return "no } after the keys";
		}
		if (token == "}") {
			break;
		}
		block.emplace_back(token);
	}
	return parse_spec(block, Spec);
}

// the spec given for the wing with the given number, or null when the wing animates as the .fiz says
inline wing_spec const *find(std::vector<wing_spec> const &Specs, int const Number) {
	auto const lookup{std::find_if(Specs.begin(), Specs.end(), [Number](wing_spec const &Spec) { return Spec.number == Number; })};
	return lookup != Specs.end() ? &(*lookup) : nullptr;
}

// keeps the spec of a wing; a later entry for the same wing replaces the earlier one
inline void store(std::vector<wing_spec> &Specs, wing_spec const &Spec) {
	auto lookup{std::find_if(Specs.begin(), Specs.end(), [&Spec](wing_spec const &Stored) { return Stored.number == Spec.number; })};
	if (lookup != Specs.end()) {
		*lookup = Spec;
	}
	else {
		Specs.emplace_back(Spec);
	}
}

// the shift or angle of a wing which opens over Wingrange, while the door of its side opens over Doorrange: the wing
// is as far along its range as the door is along the door range
inline float wing_position(float const Doorposition, float const Doorrange, float const Wingrange) {
	return Doorrange != 0.f ? Doorposition / Doorrange * Wingrange : 0.f;
}

} // namespace door_wings
