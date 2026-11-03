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

// a route of the editor as the selftest profiledump writes it: the tracks sampled, the ends joined, the switches, the norms
struct route_dump {
	ground_of_line ground;
	profile::context context;
	profile::line settings;
};

route_dump read_route(std::string const &Name) {
	route_dump result;
	std::ifstream file{std::string{EU07_UNIT_TESTS_DATA} + "/" + Name};
	for (std::string row; std::getline(file, row);) {
		std::istringstream values{row};
		std::string kind;
		values >> kind;
		auto &context{result.context};
		if (kind == "S") {
			double chainage{0.0}, height{0.0};
			values >> chainage >> height;
			result.ground.chainages.push_back(chainage);
			result.ground.heights.push_back(height);
		}
		else if (kind == "C")
			values >> context.length >> context.start_joined >> context.start_elevation >> context.start_grade >> context.end_joined >> context.end_elevation >> context.end_grade;
		else if (kind == "Z") {
			profile::context::zone zone;
			values >> zone.from >> zone.to;
			context.switches.push_back(zone);
		}
		else if (kind == "H") {
			profile::context::zone zone;
			values >> zone.from >> zone.to;
			context.humps.push_back(zone);
		}
		else if (kind == "F") {
			std::pair<double, double> fixed;
			values >> fixed.first >> fixed.second;
			context.fixed.push_back(fixed);
		}
		else if (kind == "L") {
			auto &line{result.settings};
			values >> line.speed >> line.norms.grade_max >> line.norms.grade_switch_max >> line.norms.radius_factor >> line.norms.radius_min >> line.norms.curve_threshold >> line.norms.element_min >> line.norms.fixed_tolerance >> line.join_start >> line.join_end;
		}
	}
	return result;
}

// the tracks of a line moved by the grade lines shared: a design laid anew onto them, as profile_refit_stored lays it
profile::line refit(ground_of_line const &Ground, profile::context const &Context = {}, profile::line const &Settings = {}) {
	auto context{Context};
	if (context.length <= 0.0)
		context.length = Ground.chainages.back();
	std::vector<double> const tolerances(Ground.chainages.size(), 0.005);
	std::vector<double> const reaches(Ground.chainages.size(), 100.0);
	profile::ground_fit options;
	options.tolerance = 0.01;
	options.grade_max = 50.0;
	options.humps = profile::humps(Ground.chainages, Ground.heights, context, options.tolerance);
	options.hump_radius = 300.0;
	return profile::fit_ground(Ground.chainages, Ground.heights, context, Settings, options, tolerances, reaches);
}

double worst_departure(profile::line const &Line, ground_of_line const &Ground) {
	double worst{0.0};
	for (std::size_t i = 0; i < Ground.chainages.size(); ++i)
		worst = std::max(worst, std::abs(profile::elevation(Line, Ground.chainages[i]) - Ground.heights[i]));
	return worst;
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

TEST_CASE("ground fit: a design laid anew onto tracks bending gently into a sag keeps to them") {
	// the tracks go from -6.6 to +5.8 per mille over some 100 m, as a curve of about 8 km: curves of the 4 km the norms ask for
	// follow it only two or more of them, a straight between
	auto const route{read_route("refit_sag.txt")};
	auto const &ground{route.ground};
	REQUIRE(ground.chainages.size() > 300);
	REQUIRE(false == route.context.switches.empty());
	auto const line{refit(ground, route.context, route.settings)};
	REQUIRE(line.points.size() >= 2);
	CHECK(worst_departure(line, ground) < 0.015);
	// with nothing told of the route as well
	CHECK(worst_departure(refit(ground), ground) < 0.015);
}
