#include "stdafx.h"
#include "doctest.h"
#include "editor/editorProfile.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace {

struct ground_of_line {
	std::vector<double> chainages, heights, tolerances, reaches;
};

ground_of_line read_ground(std::string const &Name) {
	ground_of_line result;
	std::ifstream file{std::string{EU07_UNIT_TESTS_DATA} + "/" + Name};
	for (std::string row; std::getline(file, row);) {
		if (row.empty() || row.front() == '#')
			continue;
		std::istringstream values{row};
		std::string chainage, height, tolerance, reach;
		values >> chainage >> height >> tolerance >> reach;
		result.chainages.push_back(std::strtod(chainage.c_str(), nullptr));
		result.heights.push_back(std::strtod(height.c_str(), nullptr));
		result.tolerances.push_back(std::strtod(tolerance.c_str(), nullptr));
		result.reaches.push_back(std::strtod(reach.c_str(), nullptr));
	}
	return result;
}

// the real ground of a line, which the fit took under a field past a viaduct
ground_of_line past_viaduct() {
	return read_ground("fit_past_viaduct.txt");
}

profile::line fit(ground_of_line const &Ground) {
	profile::context context;
	context.length = Ground.chainages.back();
	profile::ground_fit options;
	options.tolerance = 0.15;
	options.grade_max = std::numeric_limits<double>::max();
	options.follow_length = 60.0;
	options.follow_height = 4.0;
	options.follow_steepest = 100.0;
	options.follow_tolerance = 0.05;
	return profile::fit_ground(Ground.chainages, Ground.heights, context, profile::line{}, options, Ground.tolerances, Ground.reaches);
}

} // namespace

TEST_CASE("ground fit: a line held down on a viaduct comes back to the field before its loose end") {
	auto const ground{past_viaduct()};
	REQUIRE(ground.chainages.size() > 300);
	auto const line{fit(ground)};
	REQUIRE(line.points.size() >= 2);
	auto const end{ground.chainages.back()};
	// the field lies at 79.9 to 80.2: no going on under it at the grade the line took on the viaduct
	CHECK(profile::elevation(line, end) > 79.5);
	CHECK(profile::elevation(line, end - 100.0) > 79.5);
}

TEST_CASE("ground fit: the same off a loose start") {
	auto ground{past_viaduct()};
	REQUIRE(ground.chainages.size() > 300);
	auto const end{ground.chainages.back()};
	for (auto &chainage : ground.chainages)
		chainage = end - chainage;
	std::reverse(ground.chainages.begin(), ground.chainages.end());
	std::reverse(ground.heights.begin(), ground.heights.end());
	std::reverse(ground.tolerances.begin(), ground.tolerances.end());
	std::reverse(ground.reaches.begin(), ground.reaches.end());
	auto const line{fit(ground)};
	REQUIRE(line.points.size() >= 2);
	CHECK(profile::elevation(line, 0.0) > 79.5);
	CHECK(profile::elevation(line, 100.0) > 79.5);
}
