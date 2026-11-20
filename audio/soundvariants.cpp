/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "audio/soundvariants.h"
#include "utilities/parser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numbers>

namespace sound_variants {

namespace {

// the separators of the tokens of a sound block, as the sound definition reads them
char const *const block_break{"\n\r\t ,;"};

// a token as text which parses back into the same token: in quotes if it holds what the parser would split it at, take
// for a comment or change, as it can when it came from a quoted text
std::string as_text(std::string const &Token) {
	auto const needsquotes{
		std::ranges::any_of(Token, [](char const Character) {
			auto const value{static_cast<unsigned char>(Character)};
			return std::isspace(value) != 0 || Character == ',' || Character == ';' || std::isupper(value) != 0;
		})
		|| Token.find("//") != std::string::npos
		|| Token.find("/*") != std::string::npos};
	return needsquotes ? '"' + Token + '"' : Token;
}

} // namespace

int listener_location(bool const Windowopen, bool const Freefly, bool const Invehicle, int const Occupiedcab) {
	if (Windowopen) {
		return cab_window;
	}
	if (Freefly) {
		return Invehicle ? attached_camera : free_camera;
	}
	return Invehicle ? Occupiedcab : free_camera;
}

bool listener_enclosed(int const Location) {
	return Location >= rear_cab && Location <= front_cab;
}

choice select(bool const Hasinside, int const Location) {
	return Hasinside && listener_enclosed(Location) ? choice::inside : choice::main;
}

float crossfade_gain(bool const Chosen, double const Elapsed, double const Duration) {
	auto const progress{Duration > 0.0 ? std::clamp(Elapsed / Duration, 0.0, 1.0) : 1.0};
	if (progress >= 1.0) {
		return Chosen ? 1.f : 0.f;
	}
	auto const angle{progress * std::numbers::pi * 0.5};
	return static_cast<float>(Chosen ? std::sin(angle) : std::cos(angle));
}

std::string capture_block(cParser &Input) {
	auto token{Input.getToken<std::string>(true, block_break)};
	if (token != "{") {
		return {};
	}
	std::string block{token};
	auto depth{1};
	while (depth > 0) {
		token = Input.getToken<std::string>(true, block_break);
		if (token.empty()) {
			// the input ended inside of the block; close it, so the definition reads what there is
			block += " }";
			break;
		}
		if (token == "{") {
			++depth;
		}
		else if (token == "}") {
			--depth;
		}
		block += ' ';
		block += as_text(token);
	}
	return block;
}

} // namespace sound_variants
