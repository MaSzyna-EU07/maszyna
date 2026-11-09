#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

// the cab interior light set by the positions of the light preset selector: an optional third column of LightsList,
// 1 the cab light on, 2 dimmed, 0 off; a row without it leaves the cab light to its own switch (eu07/maszyna#529)

using test::definition;
using test::vehicle;

namespace {

auto const shunting{light::headlight_left | light::headlight_right};

} // namespace

TEST_CASE("lights preset cab light: a table without the column sets no position's cab light") {
	vehicle wagon{definition::load("wagon")
	                  .set("LightsList", "Size", 2)
	                  .set("LightsList", "Default", 1)
	                  .set("LightsList", "rows", {{0, 0}, {shunting, shunting}})};

	REQUIRE(wagon->LightsPosNo == 2);
	CHECK(wagon->Lights[end::front][1] == shunting);
	CHECK_FALSE(wagon->LightsPresetCabLight(1).has_value());
	CHECK_FALSE(wagon->LightsPresetCabLight(2).has_value());
}

TEST_CASE("lights preset cab light: the third column turns the cab light off, on, or on dimmed") {
	// off - shunting - shunting with the cab lit - shunting with the cab lit dimmed - shunting leaving the cab light
	vehicle wagon{definition::load("wagon")
	                  .set("LightsList", "Size", 5)
	                  .set("LightsList", "Default", 1)
	                  .set("LightsList", "rows", {{0, 0, 0}, {shunting, shunting}, {shunting, shunting, 1}, {shunting, shunting, 3}, {shunting, 0, -1}})};

	REQUIRE(wagon->LightsPosNo == 5);
	// the light columns read as before
	CHECK(wagon->Lights[end::front][2] == shunting);
	CHECK(wagon->Lights[end::rear][2] == shunting);
	CHECK(wagon->Lights[end::rear][4] == 0);

	auto const off{wagon->LightsPresetCabLight(1)};
	REQUIRE(off.has_value());
	CHECK_FALSE(off->light);
	CHECK_FALSE(off->dim);

	CHECK_FALSE(wagon->LightsPresetCabLight(2).has_value());

	auto const lit{wagon->LightsPresetCabLight(3)};
	REQUIRE(lit.has_value());
	CHECK(lit->light);
	CHECK_FALSE(lit->dim);

	auto const dimmed{wagon->LightsPresetCabLight(4)};
	REQUIRE(dimmed.has_value());
	CHECK(dimmed->light);
	CHECK(dimmed->dim);

	// a negative value is the same as no value
	CHECK_FALSE(wagon->LightsPresetCabLight(5).has_value());
}

TEST_CASE("lights preset cab light: positions outside the table, and the lightsset custom one, set nothing") {
	vehicle wagon{definition::load("wagon")
	                  .set("LightsList", "Size", 1)
	                  .set("LightsList", "Default", 1)
	                  .set("LightsList", "rows", {{shunting, shunting, 1}})};

	CHECK(wagon->LightsPresetCabLight(1).has_value());
	CHECK_FALSE(wagon->LightsPresetCabLight(0).has_value());
	CHECK_FALSE(wagon->LightsPresetCabLight(2).has_value());
	CHECK_FALSE(wagon->LightsPresetCabLight(18).has_value());
	CHECK_FALSE(wagon->LightsPresetCabLight(19).has_value());
}

TEST_CASE("lights preset cab light: a stray note after the two columns is not read as the cab light") {
	// some vehicles of the game carry rows like "38 38 / A"
	auto text{definition::load("wagon")
	              .set("LightsList", "Size", 2)
	              .set("LightsList", "Default", 1)
	              .set("LightsList", "rows", {{0, 0}, {38, 38}})
	              .text()};
	auto const list{text.find("LightsList")};
	REQUIRE(list != std::string::npos);
	auto const row{text.find("38", list)};
	REQUIRE(row != std::string::npos);
	auto const rowend{text.find('\n', row)};
	REQUIRE(rowend != std::string::npos);
	text.insert(rowend, " / A");
	TMoverParameters wagon(0.0, "wagon", "test", 0);
	REQUIRE(wagon.LoadFIZ_Text(text));
	REQUIRE(wagon.LightsPosNo == 2);
	CHECK(wagon.Lights[end::front][1] == 38);
	CHECK(wagon.Lights[end::rear][1] == 38);
	CHECK_FALSE(wagon.LightsPresetCabLight(2).has_value());
}
