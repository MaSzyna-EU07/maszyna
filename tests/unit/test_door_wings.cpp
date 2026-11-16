#include "stdafx.h"
#include "doctest.h"

#include "vehicle/doorwings.h"
#include "utilities/parser.h"

// the poses of the animated door wings (eu07/maszyna#512): a wing of the main model and its namesake in the low
// poly interior get the same pose, worked out once from the door position of the mover

TEST_CASE("door wings: only the door animations move wings") {
	CHECK(door_wings::is_wing(0x010)); // TAnim::TypeSet for the doors
	CHECK_FALSE(door_wings::is_wing(0x000)); // axles
	CHECK_FALSE(door_wings::is_wing(0x055)); // pantographs, five submodels
	CHECK_FALSE(door_wings::is_wing(0x070)); // doorsteps
	CHECK_FALSE(door_wings::is_wing(0x080)); // mirrors
	CHECK(door_wings::is_wing(0x210)); // the parameter count in the upper bits doesn't matter
}

TEST_CASE("door wings: odd wings are on the left, even ones on the right") {
	CHECK_FALSE(door_wings::on_right_side(1));
	CHECK(door_wings::on_right_side(2));
	CHECK_FALSE(door_wings::on_right_side(3));
	CHECK(door_wings::on_right_side(4));
}

TEST_CASE("door wings: a sliding wing shifts along the vehicle by the door position") {
	CHECK(door_wings::slide(0.f) == glm::vec3(0.f, 0.f, 0.f));
	CHECK(door_wings::slide(0.65f) == glm::vec3(0.f, 0.f, 0.65f));
}

TEST_CASE("door wings: a plug wing goes out first, then slides") {
	auto const rangeout{0.1f};
	// closed
	CHECK(door_wings::plug(0.f, rangeout) == glm::vec3(0.f, 0.f, 0.f));
	// still going out, and not sliding yet
	auto const out{door_wings::plug(0.02f, rangeout)};
	CHECK(out.x == doctest::Approx(0.04f));
	CHECK(out.z == 0.f);
	// out as far as it goes, sliding by what the position has beyond half of the way out
	auto const open{door_wings::plug(0.7f, rangeout)};
	CHECK(open.x == rangeout);
	CHECK(open.y == 0.f);
	CHECK(open.z == doctest::Approx(0.65f));
}

TEST_CASE("door wings: a folding wing turns one way and its larger wing twice as far the other way") {
	auto const angles{door_wings::fold(40.f)};
	CHECK(angles.smaller == 40.f);
	CHECK(angles.larger == -80.f);
	CHECK(angles.step == 40.f);
	auto const closed{door_wings::fold(0.f)};
	CHECK(closed.smaller == 0.f);
	CHECK(closed.larger == 0.f);
	CHECK(closed.step == 0.f);
}

// a wing with its own kind of animation (eu07/maszyna#70): animdoor: in the .mmd, for vehicles whose doors on one side
// aren't all alike, e.g. 111Ainw with folding doors at one end and wider ones at the other

namespace {

// the spec of the .fiz the entries start from: folding doors which turn by 80 degrees
door_wings::wing_spec const fizdoors{0, door_wings::fold_type, 80.f, 0.f};

// reads the entry after animdoor: from the text, starting from the spec of the .fiz
std::string read(std::string const &Entry, door_wings::wing_spec &Spec) {
	Spec = fizdoors;
	cParser parser(Entry, cParser::buffer_TEXT);
	return door_wings::read_spec(parser, Spec);
}

} // namespace

TEST_CASE("door wings: the kinds of animation are named as DoorOpenMethod of the .fiz") {
	CHECK(door_wings::type_from_name("Shift") == door_wings::shift_type);
	CHECK(door_wings::type_from_name("Rotate") == door_wings::rotate_type);
	CHECK(door_wings::type_from_name("Fold") == door_wings::fold_type);
	CHECK(door_wings::type_from_name("Plug") == door_wings::plug_type);
	// in any letter case
	CHECK(door_wings::type_from_name("plug") == door_wings::plug_type);
	CHECK(door_wings::type_from_name("SHIFT") == door_wings::shift_type);
	// the same numbers as TMoverParameters::door_data::type
	CHECK(door_wings::shift_type == 1);
	CHECK(door_wings::rotate_type == 2);
	CHECK(door_wings::fold_type == 3);
	CHECK(door_wings::plug_type == 4);
	CHECK_FALSE(door_wings::type_from_name("Roll").has_value());
	CHECK_FALSE(door_wings::type_from_name("").has_value());
}

TEST_CASE("door wings: an animdoor entry gives the wing its own kind, range and plug shift") {
	door_wings::wing_spec spec;
	CHECK(read("3 { type: Plug range: 0.9 rangeout: 0.06 } endmodels", spec).empty());
	CHECK(spec.number == 3);
	CHECK(spec.type == door_wings::plug_type);
	CHECK(spec.range == doctest::Approx(0.9f));
	CHECK(spec.range_out == doctest::Approx(0.06f));
}

TEST_CASE("door wings: the keys an animdoor entry leaves out keep the values of the .fiz") {
	door_wings::wing_spec spec;
	SUBCASE("only the kind") {
		CHECK(read("2 { type: Rotate }", spec).empty());
		CHECK(spec.number == 2);
		CHECK(spec.type == door_wings::rotate_type);
		CHECK(spec.range == 80.f);
		CHECK(spec.range_out == 0.f);
	}
	SUBCASE("only the range, keys in any letter case and order") {
		CHECK(read("1 { Range: -95 }", spec).empty());
		CHECK(spec.type == door_wings::fold_type);
		CHECK(spec.range == -95.f);
	}
	SUBCASE("nothing") {
		CHECK(read("4 { }", spec).empty());
		CHECK(spec.number == 4);
		CHECK(spec.type == fizdoors.type);
		CHECK(spec.range == fizdoors.range);
	}
}

TEST_CASE("door wings: a faulty animdoor entry says what is wrong with it") {
	door_wings::wing_spec spec;
	CHECK(read("3 type: Shift range: 0.65", spec) == "no { after the wing number");
	CHECK(read("3 { type: Roll range: 0.65 }", spec) == "unknown door type \"Roll\"");
	CHECK(read("3 { type: Shift range: 0,65 }", spec) == "bad number \"0,65\" for \"range:\"");
	CHECK(read("3 { type: Shift range: wide }", spec) == "bad number \"wide\" for \"range:\"");
	CHECK(read("3 { kind: Shift }", spec) == "unknown key \"kind:\"");
	CHECK(read("3 { type: Shift range: }", spec) == "no value for \"range:\"");
	CHECK(read("3 { type: Shift range: 0.65", spec) == "no } after the keys");
}

TEST_CASE("door wings: the parsed keys go into the spec, pairs of a key and its value") {
	door_wings::wing_spec spec{fizdoors};
	CHECK(door_wings::parse_spec({"type:", "Shift", "range:", "0.65"}, spec).empty());
	CHECK(spec.type == door_wings::shift_type);
	CHECK(spec.range == doctest::Approx(0.65f));
	CHECK(door_wings::parse_spec({"rangeout:"}, spec) == "no value for \"rangeout:\"");
	CHECK(door_wings::parse_spec({}, spec).empty());
}

TEST_CASE("door wings: only the wings with an entry animate in their own way, the latest entry counts") {
	std::vector<door_wings::wing_spec> specs;
	CHECK(door_wings::find(specs, 1) == nullptr);
	door_wings::store(specs, {3, door_wings::shift_type, 0.65f, 0.f});
	door_wings::store(specs, {4, door_wings::shift_type, 0.65f, 0.f});
	door_wings::store(specs, {3, door_wings::plug_type, 0.9f, 0.06f});
	CHECK(specs.size() == 2);
	CHECK(door_wings::find(specs, 1) == nullptr);
	CHECK(door_wings::find(specs, 2) == nullptr);
	REQUIRE(door_wings::find(specs, 3) != nullptr);
	CHECK(door_wings::find(specs, 3)->type == door_wings::plug_type);
	CHECK(door_wings::find(specs, 3)->range == 0.9f);
	REQUIRE(door_wings::find(specs, 4) != nullptr);
	CHECK(door_wings::find(specs, 4)->type == door_wings::shift_type);
}

TEST_CASE("door wings: a wing with its own range is as far along it as the door of its side") {
	// folding doors of the .fiz turn by 80 degrees; the wide wing slides by 0.9 m
	CHECK(door_wings::wing_position(0.f, 80.f, 0.9f) == 0.f);
	CHECK(door_wings::wing_position(40.f, 80.f, 0.9f) == doctest::Approx(0.45f));
	CHECK(door_wings::wing_position(80.f, 80.f, 0.9f) == doctest::Approx(0.9f));
	// a wing turning the other way
	CHECK(door_wings::wing_position(20.f, 80.f, -100.f) == doctest::Approx(-25.f));
	// no doors in the .fiz: the wing stays closed
	CHECK(door_wings::wing_position(0.f, 0.f, 0.9f) == 0.f);
	// the pose of the plug wing at the end of its range
	auto const open{door_wings::plug(door_wings::wing_position(80.f, 80.f, 0.9f), 0.06f)};
	CHECK(open.x == doctest::Approx(0.06f));
	CHECK(open.z == doctest::Approx(0.87f));
}
