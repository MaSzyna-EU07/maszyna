#include "stdafx.h"
#include "doctest.h"

#include "editor/editorRoadLimits.hpp"
#include "utilities/parser.h"

#include <glm/glm.hpp>
#include <sstream>

// the type of a road's surface (eu07/maszyna GH#347): a dirt road, a track between fields, can be laid out tighter than
// a paved one; the type is saved with the road, and the roads made before the types load and save as they did

namespace {

// what the scenery entry of a road says about its surface, read the way road_node::import_from() reads it:
// a road which doesn't say what it's made of is paved
struct surface_read
{
	std::string material{"asphaltdark1"};
	road_surface_type type{road_surface_type::paved};
	float texturelength{4.f};
	bool bad{false}; // an unknown type was named
	std::string next; // the token which follows the read properties
};

surface_read read(std::string const &Entry) {
	surface_read result;
	cParser parser(Entry, cParser::buffer_TEXT);
	auto token{parser.getToken<std::string>()};
	while (false == token.empty() && token != "endroad") {
		if (token == "surface") {
			result.material = parser.getToken<std::string>();
		}
		else if (token == "surfacetype") {
			if (false == road_surface::from_name(parser.getToken<std::string>(), result.type))
				result.bad = true;
		}
		else if (token == "texlength") {
			parser.getTokens();
			parser >> result.texturelength;
		}
		else {
			result.next = token;
			break;
		}
		token = parser.getToken<std::string>();
	}
	return result;
}

// the appearance part of the scenery entry, written the way road_node::export_as_text_() writes it
std::string write(std::string const &Material, road_surface_type const Type, float const Texturelength) {
	std::ostringstream output;
	output << "surface " << Material << ' ' << "texlength " << Texturelength << ' ';
	output << road_surface::entry(Type);
	output << "side left shoulder 1.5 grass ";
	return output.str();
}

} // namespace

TEST_CASE("road surface type: a paved road keeps the limits every road had before the types") {
	// the values editorRoad.cpp had as kTightestRadius, kShortestRoad, kSharpestBranch and in editor_road::plan()
	auto const paved{road_limits_for(road_surface_type::paved)};
	CHECK(paved.tightest_radius == doctest::Approx(6.0));
	CHECK(paved.shortest_road == doctest::Approx(2.0));
	CHECK(paved.sharpest_branch == doctest::Approx(glm::radians(30.0)));
	CHECK(paved.tied_lead == doctest::Approx(0.1));
	CHECK(paved.widest_turn == doctest::Approx(glm::radians(170.0)));
}

TEST_CASE("road surface type: a dirt road is laid out more freely in every respect") {
	auto const paved{road_limits_for(road_surface_type::paved)};
	auto const dirt{road_limits_for(road_surface_type::dirt)};
	// a bend a tractor makes on a single lane track, still wider than the half of such a road
	CHECK(dirt.tightest_radius < paved.tightest_radius);
	CHECK(dirt.tightest_radius >= 1.5);
	CHECK(dirt.shortest_road < paved.shortest_road);
	CHECK(dirt.shortest_road > 0.0);
	CHECK(dirt.sharpest_branch < paved.sharpest_branch);
	CHECK(dirt.sharpest_branch > 0.0);
	// an end may lead off square to the way to the other end, but not away from it
	CHECK(dirt.tied_lead < paved.tied_lead);
	CHECK(dirt.tied_lead >= 0.0);
	// a hairpin: the arc may turn past a half circle
	CHECK(dirt.widest_turn > glm::radians(180.0));
	CHECK(dirt.widest_turn < glm::radians(360.0));
}

TEST_CASE("road surface type: the types go by their scenery names") {
	for (auto const type : {road_surface_type::paved, road_surface_type::dirt}) {
		CAPTURE(road_surface::name(type));
		road_surface_type read{type == road_surface_type::paved ? road_surface_type::dirt : road_surface_type::paved};
		CHECK(road_surface::from_name(road_surface::name(type), read));
		CHECK(read == type);
	}
	CHECK(std::string{road_surface::name(road_surface_type::paved)} == "paved");
	CHECK(std::string{road_surface::name(road_surface_type::dirt)} == "dirt");
	// an unknown name changes nothing
	road_surface_type type{road_surface_type::dirt};
	CHECK_FALSE(road_surface::from_name("gravel", type));
	CHECK(type == road_surface_type::dirt);
	CHECK_FALSE(road_surface::from_name("", type));
	CHECK(type == road_surface_type::dirt);
}

TEST_CASE("road surface type: a paved road is saved as the roads were before the types") {
	CHECK(road_surface::entry(road_surface_type::paved).empty());
	CHECK(write("asphaltdark1", road_surface_type::paved, 4.f) == "surface asphaltdark1 texlength 4 side left shoulder 1.5 grass ");
	// and an entry of a road made before the types reads paved, with what follows read as before
	auto const old{read("surface asphalt2 texlength 6 side left shoulder 1.5 grass endroad")};
	CHECK(old.type == road_surface_type::paved);
	CHECK(old.material == "asphalt2");
	CHECK(old.texturelength == doctest::Approx(6.f));
	CHECK(old.next == "side");
	CHECK_FALSE(old.bad);
}

TEST_CASE("road surface type: a dirt road is saved with its type and read back") {
	auto const text{write(road_surface::default_material(road_surface_type::dirt), road_surface_type::dirt, 5.f)};
	CHECK(text == "surface droga_polna texlength 5 surfacetype dirt side left shoulder 1.5 grass ");
	auto const back{read(text)};
	CHECK(back.type == road_surface_type::dirt);
	CHECK(back.material == "droga_polna");
	CHECK(back.texturelength == doctest::Approx(5.f));
	CHECK(back.next == "side");
	CHECK_FALSE(back.bad);
	// and written again it's the same
	CHECK(write(back.material, back.type, back.texturelength) == text);
	// the type is read wherever it stands among the properties, and an explicit paved reads as the default
	CHECK(read("surfacetype dirt surface droga_polna endroad").type == road_surface_type::dirt);
	CHECK(read("surface asphaltdark1 surfacetype paved endroad").type == road_surface_type::paved);
}

TEST_CASE("road surface type: an unknown type leaves the road paved") {
	auto const back{read("surface asphaltdark1 surfacetype cobbles texlength 4 side")};
	CHECK(back.bad);
	CHECK(back.type == road_surface_type::paved);
	// the unknown name is taken as the value of the type, not as the next property
	CHECK(back.texturelength == doctest::Approx(4.f));
	CHECK(back.next == "side");
}

TEST_CASE("road surface type: a road made dirt or paved gets the surface of the type") {
	CHECK(std::string{road_surface::default_material(road_surface_type::paved)} == "asphaltdark1");
	CHECK(std::string{road_surface::default_material(road_surface_type::dirt)} == "droga_polna");
}
