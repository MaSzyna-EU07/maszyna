#include "stdafx.h"
#include "doctest.h"

#include "utilities/parser.h"
#include "world/TractionTail.h"

#include <sstream>
#include <string>

// the optional entries of a traction node after "vis" / "unvis" (world/TractionTail.h): what the load reads, the save
// of the editor writes back, so an edited span keeps its own tension and the ends of its second messenger

namespace {

traction_tail read(std::string const &Text, glm::dvec3 const &Origin = glm::dvec3{0.0}) {
	cParser parser(Text, cParser::buffer_TEXT);
	traction_tail tail;
	read_traction_tail(parser, Origin, tail);
	return tail;
}

std::string write(traction_tail const &Tail) {
	std::ostringstream output;
	write_traction_tail(output, Tail);
	return output.str();
}

} // namespace

TEST_CASE("traction tail: tension, rope2, parallel and type survive a load and a save") {
	auto const loaded{read("parallel Other_Span type YwsC120-2C tension 1000 900 rope2 start 1 2 3 rope2 end 4.5 5 6 endtraction", glm::dvec3{10.0, 0.0, -20.0})};
	CHECK(loaded.parallel == "other_span");
	CHECK(loaded.type == "YwsC120-2C");
	CHECK(loaded.tension[0] == doctest::Approx(1000.0));
	CHECK(loaded.tension[1] == doctest::Approx(900.0));
	REQUIRE(loaded.rope2set[0]);
	REQUIRE(loaded.rope2set[1]);
	// the points get the origin of the scenery, as the points of the span do
	CHECK(loaded.rope2[0].x == doctest::Approx(11.0));
	CHECK(loaded.rope2[0].z == doctest::Approx(-17.0));
	CHECK(loaded.rope2[1].x == doctest::Approx(14.5));

	// written in the scenery's coordinates, read back without an origin: the same
	auto const written{write(loaded)};
	auto const again{read(written + "endtraction")};
	CHECK(again.parallel == loaded.parallel);
	CHECK(again.type == loaded.type);
	CHECK(again.tension == loaded.tension);
	CHECK(again.rope2set == loaded.rope2set);
	for (int end = 0; end < 2; ++end) {
		CAPTURE(end);
		CHECK(again.rope2[end].x == doctest::Approx(loaded.rope2[end].x));
		CHECK(again.rope2[end].y == doctest::Approx(loaded.rope2[end].y));
		CHECK(again.rope2[end].z == doctest::Approx(loaded.rope2[end].z));
	}
	CHECK(write(again) == written);
}

TEST_CASE("traction tail: a span without the entries is written as before") {
	CHECK(write(read("endtraction")).empty());
	// parallel and type alone, in the order and the form the save always wrote them
	CHECK(write(read("parallel p2 type C120-2C endtraction")) == "parallel p2 type C120-2C ");
	// one end of the second messenger only
	auto const one{read("rope2 end 1 2 3 endtraction")};
	CHECK_FALSE(one.rope2set[0]);
	CHECK(write(one) == "rope2 end 1 2 3 ");
	// the tension of the messenger alone keeps the 0 of the contact wires (as in the type)
	CHECK(write(read("tension 1200 0 endtraction")) == "tension 1200 0 ");
}
