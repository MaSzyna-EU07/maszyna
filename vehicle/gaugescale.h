/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "utilities/parser.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

// a non-uniform scale of a cab gauge (MaSzyna-EU07/maszyna#74), such as the marker of the controller position of
// the ST43, whose marks are not evenly spaced: the gauge shows what a table gives for its value, interpolated
// linearly between the points of the table, instead of value * scale + offset
namespace gauge_scale {

struct point {
	float value; // the value of the gauge
	float scaled; // what the gauge shows for it, in the units of value * scale + offset (turns for rot, metres for mov)
};

// puts the points in the order of their values; points of the same value keep their order, so two of them make a step
inline void sort(std::vector<point> &Points) {
	std::ranges::stable_sort(Points, {}, &point::value);
}

// what the gauge shows for the Value, from the Points sorted by value, which can't be empty;
// below the first point and above the last one the gauge stays at their ends of the scale
inline float interpolate(std::vector<point> const &Points, float const Value) {
	auto const upper{std::ranges::upper_bound(Points, Value, {}, &point::value)};
	if (upper == Points.begin()) {
		return Points.front().scaled;
	}
	if (upper == Points.end()) {
		return Points.back().scaled;
	}
	auto const &from{*std::prev(upper)};
	auto const &to{*upper};
	// from.value <= Value < to.value
	return std::lerp(from.scaled, to.scaled, (Value - from.value) / (to.value - from.value));
}

// a table as read from the config
struct table {
	std::vector<point> points; // sorted by value
	bool closed{false}; // it ended with its "]"
	bool block_end{false}; // the "}" of the gauge's block came in place of the "]", and was read
	bool malformed{false}; // something that isn't a number, or a value without what the gauge shows for it
};

// the number Text holds, nothing when it holds anything else
inline std::optional<float> number(std::string_view const Text) {
	float value{};
	auto const *last{Text.data() + Text.size()};
	if (auto const [end, error]{std::from_chars(Text.data(), last, value)}; error != std::errc{} || end != last) {
		return std::nullopt;
	}
	return value;
}

// what came with a token of a table
enum class token_end { more, closed, block_end, file_end };

// reads one token of a table into Numbers
inline token_end read_token(cParser &Input, std::vector<float> &Numbers, bool &Malformed) {
	auto token{Input.getToken<std::string>(false, "\n\r\t ,;")};
	if (token.empty()) {
		return token_end::file_end;
	}
	if (token == "}") {
		return token_end::block_end;
	}
	auto const closes{token.ends_with(']')};
	std::erase(token, '[');
	std::erase(token, ']');
	if (auto const value{number(token)}; value) {
		Numbers.push_back(*value);
	}
	else if (false == token.empty()) {
		Malformed = true;
	}
	return closes ? token_end::closed : token_end::more;
}

// reads a table written as [ value scaled, value scaled, ... ], pairs of a value of the gauge and what it shows for
// it; the brackets may touch the numbers, the commas are optional
inline table read(cParser &Input) {
	table result;
	std::vector<float> numbers;
	auto end{token_end::more};
	while (end == token_end::more) {
		end = read_token(Input, numbers, result.malformed);
	}
	result.closed = end == token_end::closed;
	result.block_end = end == token_end::block_end;
	if (numbers.size() % 2 != 0) {
		result.malformed = true;
	}
	for (std::size_t index{0}; index + 1 < numbers.size(); index += 2) {
		result.points.emplace_back(numbers[index], numbers[index + 1]);
	}
	sort(result.points);
	return result;
}

} // namespace gauge_scale
