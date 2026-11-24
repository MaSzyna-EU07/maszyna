// the assumptions of the project (editor/editorProjectSetup): the line they're kept in, read back the same, the defaults where the line
// says nothing or what it can't read, the class of the line the norms judge by, the gauge of the norms; kept in the project file as written

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorProjectSetup.hpp"
#include "editor/project/projectFile.hpp"

#include <string>
#include <vector>

TEST_CASE("project setup: the defaults are PLK, 100 km/h, mixed traffic, a main track, 1435 mm, 60E1") {
	project_setup::setup const setup;
	CHECK(setup.profile == norms::profile_id::plk);
	CHECK(setup.speed == doctest::Approx(100.0));
	CHECK(setup.traffic == norms::traffic::mixed);
	CHECK(setup.track == norms::track_kind::main);
	CHECK(setup.gauge == 1435);
	CHECK(setup.rail == "60E1");
	CHECK(project_setup::rail_axes(setup) == doctest::Approx(1500.0));
	CHECK(project_setup::record(setup) == "setup norms plk speed 100 traffic mixed track main gauge 1435 rail 60E1");
}

TEST_CASE("project setup: the line is read back the same, the textures with blanks too") {
	project_setup::setup setup;
	setup.profile = norms::profile_id::tsi;
	setup.speed = 80.0;
	setup.traffic = norms::traffic::freight;
	setup.track = norms::track_kind::side;
	setup.gauge = 1000;
	setup.rail = "49E1";
	setup.rails = "rail_screw new";
	setup.ballast = "1000mm/ballast 100%";
	auto const line{project_setup::record(setup)};
	CAPTURE(line);
	auto const read{project_setup::read(std::string_view{line})};
	REQUIRE(read.has_value());
	CHECK(*read == setup);
	// among the //$u lines, the first which is one
	auto const found{project_setup::read(std::vector<std::string>{"# the assumptions of the project", line})};
	REQUIRE(found.has_value());
	CHECK(*found == setup);
}

TEST_CASE("project setup: what the line doesn't say or can't be read keeps the default, another line isn't one") {
	auto const read{project_setup::read(std::string_view{"setup speed fast norms xyz gauge 1520 colour red"})};
	REQUIRE(read.has_value());
	CHECK(read->speed == doctest::Approx(100.0));
	CHECK(read->profile == norms::profile_id::plk);
	CHECK(read->gauge == 1520);
	CHECK(project_setup::rail_axes(*read) == doctest::Approx(1585.0));
	CHECK_FALSE(project_setup::read(std::string_view{"point 1 2"}).has_value());
	CHECK_FALSE(project_setup::read(std::vector<std::string>{}).has_value());
}

TEST_CASE("project setup: the class of the line the norms judge by, its speed the one of the line laid") {
	project_setup::setup setup;
	setup.speed = 120.0;
	setup.traffic = norms::traffic::passenger;
	setup.track = norms::track_kind::side;
	auto const own{project_setup::line_class(setup)};
	CHECK(own.vmax == doctest::Approx(120.0));
	CHECK(own.kind == norms::traffic::passenger);
	CHECK(own.track == norms::track_kind::side);
	CHECK(project_setup::line_class(setup, 60.0).vmax == doctest::Approx(60.0));
	CHECK(project_setup::diverging_speed(setup) == doctest::Approx(40.0));
	setup.track = norms::track_kind::main;
	CHECK(project_setup::diverging_speed(setup) == doctest::Approx(60.0));
}

TEST_CASE("project setup: kept in the project file as it was written, read back the same") {
	project::document document;
	project_setup::setup setup;
	setup.speed = 160.0;
	document.setup = project_setup::record(setup);
	auto const text{project::write(document)};
	CHECK(text.find(document.setup) != std::string::npos);
	auto const read{project::read(text)};
	REQUIRE(read.project.has_value());
	CHECK(read.project->setup == document.setup);
	CHECK(project::same(*read.project, document));
	auto const back{project_setup::read(std::string_view{read.project->setup})};
	REQUIRE(back.has_value());
	CHECK(*back == setup);
	// a project with none writes none
	CHECK(project::write(project::document{}).find("setup") == std::string::npos);
}
