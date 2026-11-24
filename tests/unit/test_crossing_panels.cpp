// the crossing panels CBP made of their drawings (world/CrossingPanels.cpp, Id-111 of PKP PLK), for the sweeps to lay along the tracks:
// the names, the sizes of the pieces, the cut corners of PWS, the edges, the joints, and the triangles closing each panel facing out

#include "stdafx.h"
#include "doctest.h"
#include "world/CrossingPanels.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

struct extent {
	glm::dvec3 low{1e9};
	glm::dvec3 high{-1e9};
};

extent extent_of(std::vector<world_vertex> const &Vertices) {
	extent result;
	for (auto const &vertex : Vertices) {
		result.low = glm::min(result.low, vertex.position);
		result.high = glm::max(result.high, vertex.position);
	}
	return result;
}

// the widest the top of the piece is across, at a distance along it: the corners of its top faces there
double top_width(std::vector<world_vertex> const &Vertices, double const Along) {
	double low{1e9}, high{-1e9};
	for (std::size_t i = 0; i + 2 < Vertices.size(); i += 3)
		for (int k = 0; k < 3; ++k) {
			auto const &p{Vertices[i + k].position};
			if (std::abs(p.y) < 1e-9 && std::abs(p.z - Along) < 1e-6) {
				low = std::min(low, p.x);
				high = std::max(high, p.x);
			}
		}
	return high - low;
}

// the volume the triangles close, by the divergence theorem: above 0 with all of them facing out
double volume(std::vector<world_vertex> const &Vertices) {
	double result{0.0};
	for (std::size_t i = 0; i + 2 < Vertices.size(); i += 3)
		result += glm::dot(Vertices[i].position, glm::cross(Vertices[i + 1].position, Vertices[i + 2].position)) / 6.0;
	return result;
}

} // namespace

TEST_CASE("crossing panels: the names of the pieces, read and written") {
	crossing_panels::spec spec;
	REQUIRE(crossing_panels::parse("@cbp/in180-1435x3", spec));
	CHECK(spec.kind == crossing_panels::piece::inner_row);
	CHECK(spec.height == 180);
	CHECK(spec.gauge == 1435);
	CHECK(spec.count == 3);
	CHECK(crossing_panels::name(spec) == "@cbp/in180-1435x3");
	REQUIRE(crossing_panels::parse("@cbp/pws140-1520", spec));
	CHECK(spec.kind == crossing_panels::piece::pws);
	CHECK(spec.height == 140);
	CHECK(spec.gauge == 1520);
	CHECK(crossing_panels::name(spec) == "@cbp/pws140-1520");
	REQUIRE(crossing_panels::parse("@cbp/out180-1435x4", spec));
	CHECK(spec.kind == crossing_panels::piece::outer_row);
	CHECK(spec.count == 4);
	// an inner row is two panels at least
	REQUIRE(crossing_panels::parse("@cbp/in180-1435x1", spec));
	CHECK(spec.count == 2);
	// not panels: the models, the heights and gauges there are no panels for
	CHECK_FALSE(crossing_panels::parse("przejazdy/cbp-pw", spec));
	CHECK_FALSE(crossing_panels::parse("@cbp/pw200-1435", spec));
	CHECK_FALSE(crossing_panels::parse("@cbp/pw180-1000", spec));
	CHECK_FALSE(crossing_panels::parse("@cbp/xx180-1435", spec));
	// the height for the rail: 180 mm for 60E1 (172 mm high), 140 for 49E1 (149 mm)
	CHECK(crossing_panels::height_for(0.172) == 180);
	CHECK(crossing_panels::height_for(0.149) == 140);
}

TEST_CASE("crossing panels: PW, PWS and PZ as Id-111 draws them, 3000 mm long, 1300 (1380), 640 mm wide, 180 or 140 mm thick") {
	for (auto const height : {180, 140})
		for (auto const gauge : {1435, 1520}) {
			CAPTURE(height);
			CAPTURE(gauge);
			auto const width{gauge == 1520 ? 1.380 : 1.300};
			auto const pw{crossing_panels::geometry({crossing_panels::piece::pw, height, gauge, 1})};
			auto const box{extent_of(pw)};
			CHECK(box.high.z - box.low.z == doctest::Approx(3.0));
			CHECK(box.high.x - box.low.x == doctest::Approx(width));
			CHECK(box.high.y == doctest::Approx(0.0));
			CHECK(-box.low.y == doctest::Approx(height / 1000.0));
			// the top as wide as the panel, the joint at either end: the top set back 5 mm
			CHECK(top_width(pw, crossing_panels::joint) == doctest::Approx(width));
			// the flangeway it leaves in the track: (1435 - 1300) / 2 = 67.5 mm, (1520 - 1380) / 2 = 70 mm
			CHECK((gauge / 1000.0 - width) / 2.0 >= 0.067);
			CHECK(volume(pw) > 0.0);
			// the volume of the cross-section along the length: the top, the sides down, the slopes in by 150 mm to the bottom
			auto const drop{crossing_panels::edge_drop(height)};
			auto const h{height / 1000.0};
			auto const section{width * drop + (h - drop) * (width + (width - 2.0 * crossing_panels::edge_slope)) / 2.0};
			CHECK(volume(pw) == doctest::Approx(section * 3.0).epsilon(0.01));
			// PWS: the corners cut 150 x 260 at its end, the top 1000 (1080) mm wide there
			auto const pws{crossing_panels::geometry({crossing_panels::piece::pws, height, gauge, 1})};
			CHECK(top_width(pws, 3.0 - crossing_panels::corner_along) == doctest::Approx(width));
			CHECK(top_width(pws, 3.0 - crossing_panels::joint) == doctest::Approx(width - 2.0 * crossing_panels::corner_across));
			CHECK(volume(pws) > 0.0);
			CHECK(volume(pws) < volume(pw));
			// PZ: 640 mm wide, from its edge by the rail at 0 outwards
			auto const pz{crossing_panels::geometry({crossing_panels::piece::pz, height, gauge, 1})};
			auto const outer{extent_of(pz)};
			CHECK(outer.low.x == doctest::Approx(0.0));
			CHECK(outer.high.x == doctest::Approx(0.640));
			CHECK(-outer.low.y == doctest::Approx(height / 1000.0));
			CHECK(volume(pz) > 0.0);
		}
}

TEST_CASE("crossing panels: the rows, a PWS at either end with the cut outwards, the panels 3 m apart with a joint of 10 mm") {
	auto const row{crossing_panels::geometry({crossing_panels::piece::inner_row, 180, 1435, 3})};
	auto const box{extent_of(row)};
	CHECK(box.high.z - box.low.z == doctest::Approx(9.0));
	CHECK(crossing_panels::piece_length({crossing_panels::piece::inner_row, 180, 1435, 3}) == doctest::Approx(9.0));
	// cut at both ends of the row, whole in between
	CHECK(top_width(row, crossing_panels::joint) < 1.3 - 0.25);
	CHECK(top_width(row, 9.0 - crossing_panels::joint) < 1.3 - 0.25);
	CHECK(top_width(row, 3.0 + crossing_panels::joint) == doctest::Approx(1.3));
	// the joints: the tops of two panels 10 mm apart
	CHECK(top_width(row, 3.0) < 0.0);
	auto const outer{crossing_panels::geometry({crossing_panels::piece::outer_row, 180, 1435, 3})};
	CHECK(extent_of(outer).high.z == doctest::Approx(9.0));
	CHECK(volume(outer) == doctest::Approx(3.0 * volume(crossing_panels::geometry({crossing_panels::piece::pz, 180, 1435, 1}))).epsilon(1e-6));
	// the texture: the steel of the angles along the edges, the concrete of the top between them
	int steel{0}, concrete{0};
	for (auto const &vertex : row) {
		if (vertex.texture.y >= 0.93f - 1e-4f && vertex.texture.y <= 0.98f + 1e-4f)
			++steel;
		if (std::abs(vertex.position.y) < 1e-9 && std::abs(vertex.position.x) < 0.6 && vertex.texture.y < 0.87f)
			++concrete;
	}
	CHECK(steel > 0);
	CHECK(concrete > 0);
	// for the limits: R 600 m and more, the sleepers 0.60 m apart, the panels 5 of them long
	CHECK(crossing_panels::minimum_radius == doctest::Approx(600.0));
	CHECK(crossing_panels::length / crossing_panels::sleeper_spacing == doctest::Approx(5.0));
}
