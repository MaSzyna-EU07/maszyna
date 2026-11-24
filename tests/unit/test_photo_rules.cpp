#include "stdafx.h"
#include "doctest.h"
#include "editor/editorGeometry.hpp"
#include "editor/editorPhotoRules.hpp"
#include "editor/editorRecords.hpp"
#include "editor/editorSettings.hpp"

#include <set>
#include <sstream>
#include <utility>
#include <vector>

namespace {

using photo_rules::origin_source;

std::set<std::pair<int, int>> squares(std::vector<glm::dvec2> const &Polygon, double const Size) {
	auto const found{geometry::squares_in(Polygon, Size)};
	return {found.begin(), found.end()};
}

// a square of tiles of 256 m, the tiles from First to Last along both axes
std::vector<glm::dvec2> block(int const First, int const Last) {
	double const low{First * 256.0 + 10.0};
	double const high{(Last + 1) * 256.0 - 10.0};
	return {{low, low}, {high, low}, {high, high}, {low, high}};
}

} // namespace

TEST_CASE("photo: the photo turned on or off is the scenery's, kept with the editor") {
	editorSettings settings;
	CHECK_FALSE(settings.orthophoto_on("a").has_value());
	settings.orthophoto_on("a", false);
	settings.orthophoto_on("b c", true);
	CHECK(settings.orthophoto_on("a") == std::optional<bool>{false});
	CHECK(settings.orthophoto_on("b c") == std::optional<bool>{true});
	CHECK_FALSE(settings.orthophoto_on("d").has_value());

	// written and read back, the scenery with a blank in its name too
	std::stringstream file;
	settings.write(file);
	editorSettings again;
	again.read(file);
	CHECK(again.orthophoto_on("a") == std::optional<bool>{false});
	CHECK(again.orthophoto_on("b c") == std::optional<bool>{true});
	CHECK_FALSE(again.orthophoto_on("d").has_value());

	again.orthophoto_forget("a");
	CHECK_FALSE(again.orthophoto_on("a").has_value());
}

TEST_CASE("photo: the global off of an older editor is left behind") {
	std::istringstream file{"ortho_on 0\northo_opacity 0.4\n"};
	editorSettings settings;
	settings.read(file);
	CHECK_FALSE(settings.orthophoto_on("any").has_value());
	CHECK(settings.orthophoto().opacity == doctest::Approx(0.4f));
}

TEST_CASE("photo: on as the user left it in the scenery, else as the scenery keeps it, else off") {
	// the wizard's //$o on 1 with nothing chosen by the user in the scenery: on
	auto const wizard{photo_rules::photo_on(std::nullopt, true)};
	CHECK(wizard.on);
	CHECK(wizard.chosen);
	// turned off by the user in it: off, whatever the scenery says
	CHECK_FALSE(photo_rules::photo_on(false, true).on);
	CHECK(photo_rules::photo_on(true, false).on);
	// nothing said: off, and the terrain model may turn it on
	auto const none{photo_rules::photo_on(std::nullopt, std::nullopt)};
	CHECK_FALSE(none.on);
	CHECK_FALSE(none.chosen);
}

TEST_CASE("photo: //$g wins over the origin remembered and the one of //$o, the difference told") {
	glm::dvec2 const g{500000.0, 600000.0};
	auto const choice{photo_rules::choose_origin(g, std::nullopt, glm::dvec2{510000.0, 600000.0})};
	CHECK(choice.from == origin_source::georeference);
	CHECK(choice.origin == g);
	CHECK(choice.differs == doctest::Approx(10000.0));

	auto const same{photo_rules::choose_origin(g, g, g)};
	CHECK(same.differs == doctest::Approx(0.0));
	CHECK(same.differs <= photo_rules::origin_tolerance);

	auto const mark{photo_rules::choose_origin(g, glm::dvec2{500000.0, 600030.0}, std::nullopt)};
	CHECK(mark.from == origin_source::georeference);
	CHECK(mark.differs == doctest::Approx(30.0));

	// without //$g: the editor's //$o, then the one remembered
	CHECK(photo_rules::choose_origin(std::nullopt, glm::dvec2{1.0, 2.0}, glm::dvec2{3.0, 4.0}).from == origin_source::mark);
	auto const remembered{photo_rules::choose_origin(glm::dvec2{0.0}, std::nullopt, glm::dvec2{3.0, 4.0})};
	CHECK(remembered.from == origin_source::remembered);
	CHECK(remembered.origin == glm::dvec2{3.0, 4.0});
	CHECK(photo_rules::choose_origin(std::nullopt, std::nullopt, std::nullopt).from == origin_source::none);

	// a //$o aligned by hand goes before //$g: the user moved the photo off it on purpose
	auto const aligned{photo_rules::choose_origin(g, glm::dvec2{500000.0, 600030.0}, glm::dvec2{510000.0, 600000.0}, true)};
	CHECK(aligned.from == origin_source::mark);
	CHECK(aligned.origin == glm::dvec2{500000.0, 600030.0});
	CHECK(aligned.differs == doctest::Approx(0.0));
}

TEST_CASE("photo: //$o says the origin was aligned by hand, which a release before it passes over") {
	auto const line{editor_records::orthophoto_line(true, true, 500000.0, 600030.0, true)};
	CHECK(line == "on 1 origin 500000.00 600030.00 aligned");
	bool on{false};
	double north{0.0};
	double east{0.0};
	bool aligned{false};
	REQUIRE(editor_records::read_orthophoto(line, on, north, east, &aligned));
	CHECK(on);
	CHECK(north == doctest::Approx(500000.0));
	CHECK(east == doctest::Approx(600030.0));
	CHECK(aligned);
	// read as before without the word, and the word passed over where it isn't asked for
	CHECK(editor_records::orthophoto_line(true, true, 500000.0, 600030.0) == "on 1 origin 500000.00 600030.00");
	bool again{false};
	REQUIRE(editor_records::read_orthophoto("on 1 origin 1.00 2.00", on, north, east, &again));
	CHECK_FALSE(again);
	REQUIRE(editor_records::read_orthophoto(line, on, north, east));
	CHECK(east == doctest::Approx(600030.0));
}

TEST_CASE("photo: a tile which failed is tried again later and later, up to 5 min, never given up") {
	CHECK(photo_rules::retry_delay(0) == doctest::Approx(0.0));
	CHECK(photo_rules::retry_delay(1) == doctest::Approx(5.0));
	CHECK(photo_rules::retry_delay(2) == doctest::Approx(10.0));
	CHECK(photo_rules::retry_delay(5) == doctest::Approx(80.0));
	CHECK(photo_rules::retry_delay(7) == doctest::Approx(300.0));
	CHECK(photo_rules::retry_delay(1000) == doctest::Approx(300.0));
	for (int failures = 1; failures < 40; ++failures)
		CHECK(photo_rules::retry_delay(failures) <= photo_rules::retry_most);
}

TEST_CASE("photo: the 4K photos come when the view is zoomed in to 15 cm a pixel, and go past 25 cm") {
	CHECK(photo_rules::hires_wanted(0.10, false));
	CHECK(photo_rules::hires_wanted(0.15, false));
	CHECK_FALSE(photo_rules::hires_wanted(0.16, false));
	// on already: they stay a little further out
	CHECK(photo_rules::hires_wanted(0.20, true));
	CHECK_FALSE(photo_rules::hires_wanted(0.26, true));
	// a view of no known size
	CHECK_FALSE(photo_rules::hires_wanted(0.0, true));
}

TEST_CASE("photo: what a download that failed is told as") {
	CHECK(photo_rules::failure_reason("WinHTTP error 12007") == "no connection to the internet: the name of the server isn't found");
	CHECK(photo_rules::failure_reason("WinHTTP error 12002") == "the server didn't answer in time");
	CHECK(photo_rules::failure_reason("WinHTTP error 12029") == "the server can't be reached");
	CHECK(photo_rules::failure_reason("HTTP 503") == "the server is busy or out of order");
	CHECK(photo_rules::failure_reason("HTTP 404") == "the server has no photo of the place");
	CHECK(photo_rules::failure_reason("unexpected response: <ServiceException") == "the server sent an error in place of the photo");
	CHECK(photo_rules::failure_reason("something else") == "the download failed");
}

TEST_CASE("photo: the failures of the connection told alike for the terrain model, its own messages left as they are") {
	CHECK(photo_rules::connection_reason("WinHTTP error 12002") == "the server didn't answer in time");
	CHECK(photo_rules::connection_reason("WinHTTP error 12007") == "no connection to the internet: the name of the server isn't found");
	CHECK(photo_rules::connection_reason("WinHTTP error 12030") == "the server can't be reached");
	CHECK(photo_rules::connection_reason("WinHTTP error 12175") == "the secure connection to the server failed");
	CHECK(photo_rules::connection_reason("HTTP 503") == "the server is busy or out of order");
	CHECK(photo_rules::connection_reason("HTTP 404") == "the server refused the request");
	CHECK(photo_rules::connection_reason("WinHTTP error 12345") == "the download failed");
	CHECK(photo_rules::connection_reason("The service gave no terrain model").empty());
}

TEST_CASE("photo: the download area grows by the square around the camera at its edge") {
	// 4 x 4 tiles, the camera in the tile at the east edge
	auto const area{block(0, 3)};
	auto const grown{photo_rules::grow_area(area, {3.5 * 256.0, 1.5 * 256.0}, 256.0, 1, 2, 1000)};
	REQUIRE(grown.polygon.size() >= 4);
	auto const tiles{squares(grown.polygon, 256.0)};
	// the old tiles all in, the new ones 2 tiles around the camera's, nothing else
	for (int x = 0; x < 4; ++x)
		for (int y = 0; y < 4; ++y)
			CHECK(tiles.count({x, y}) == 1);
	for (int x = 1; x <= 5; ++x)
		for (int y = -1; y <= 3; ++y)
			CHECK(tiles.count({x, y}) == 1);
	CHECK(tiles.size() == grown.total);
	CHECK(grown.total == 16 + 10 + 3); // the columns 4 and 5 of y -1..3, the row -1 of x 1..3
	CHECK(grown.added == grown.total - 16);
	CHECK_FALSE(geometry::polygon_crosses_itself(grown.polygon));
}

TEST_CASE("photo: the download area stays where the camera is deep inside it or far from it") {
	auto const area{block(0, 9)};
	// in the middle, the square around it inside
	auto const inside{photo_rules::grow_area(area, {5.5 * 256.0, 5.5 * 256.0}, 256.0, 1, 2, 1000)};
	CHECK(inside.polygon.empty());
	CHECK(inside.total == 100);
	// 3 tiles off its edge: elsewhere, not at its edge
	auto const away{photo_rules::grow_area(area, {13.5 * 256.0, 5.5 * 256.0}, 256.0, 1, 2, 1000)};
	CHECK(away.polygon.empty());
	CHECK_FALSE(away.limited);
	// no area: nothing to grow
	CHECK(photo_rules::grow_area({}, {0.0, 0.0}, 256.0, 1, 2, 1000).polygon.empty());
}

TEST_CASE("photo: the download area doesn't grow past its limit") {
	auto const area{block(0, 9)};
	auto const grown{photo_rules::grow_area(area, {9.5 * 256.0, 5.5 * 256.0}, 256.0, 1, 2, 105)};
	CHECK(grown.polygon.empty());
	CHECK(grown.limited);
	CHECK(grown.total == 100);
}

TEST_CASE("photo: an area grown across the mouth of a U takes the inside of the U in, no hole left") {
	// a U of tiles: the columns 0 and 4 of y 0..4 and the row 0 between them; the camera over the mouth
	std::vector<std::pair<int, int>> u;
	for (int y = 0; y < 5; ++y)
		u.insert(u.end(), {{0, y}, {4, y}});
	for (int x = 1; x < 4; ++x)
		u.emplace_back(x, 0);
	auto const outline{photo_rules::squares_outline(u, 256.0, 1.0)};
	REQUIRE(outline.size() == 8);
	CHECK(squares(outline, 256.0).size() == 13);
	auto const grown{photo_rules::grow_area(outline, {2.5 * 256.0, 5.5 * 256.0}, 256.0, 2, 2, 1000)};
	REQUIRE(grown.polygon.size() >= 4);
	auto const tiles{squares(grown.polygon, 256.0)};
	// the square around the camera, x 0..4 and y 3..7, closes the U: its inside is taken in
	for (int x = 0; x < 5; ++x)
		for (int y = 0; y < 8; ++y)
			CHECK(tiles.count({x, y}) == 1);
	CHECK(tiles.size() == 40);
	CHECK(grown.total == 40);
	CHECK(grown.added == 27);
}

TEST_CASE("photo: the outline of squares not in one piece is none") {
	CHECK(photo_rules::squares_outline({{0, 0}, {2, 0}}, 256.0, 1.0).empty());
	auto const one{photo_rules::squares_outline({{3, 4}}, 256.0, 1.0)};
	REQUIRE(one.size() == 4);
	CHECK(squares(one, 256.0) == std::set<std::pair<int, int>>{{3, 4}});
}
