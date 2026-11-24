#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

#include <fstream>

#include <nlohmann/json.hpp>

using test::definition;
using test::vehicle;

// the field shunting step a position of the master controller selects (RList, its sixth column ScndAct) indexes the
// motor parameter table, MotorParam, which has MotorParametersArraySize + 1 entries. The loader takes ScndAct from the
// .fiz as it is, so the readers of the table (TMoverParameters::Momentum(), MomentumF(), TController::ESMVelocity())
// go through motor_param(), which keeps the index inside the table

namespace {

// the RList rows of the EP09 with the ScndAct of the first position replaced
nlohmann::ordered_json ep09_rlist_with_scndact(std::string const &Scndact) {
	std::ifstream file{std::string{EU07_UNIT_TESTS_DATA} + "/fiz/ep09_v2/104e.json"};
	REQUIRE(file.good());
	// '=' rather than braces: GCC takes a json in braces for an initializer list, an array holding it
	auto const ep09 = nlohmann::ordered_json::parse(file);
	auto rows = ep09.at("RList").at("rows");
	REQUIRE(rows.size() > 1);
	rows[1][5] = Scndact;
	return rows;
}

} // namespace

TEST_CASE("motor parameters: the ScndAct of an RList row is read from the .fiz as it is") {
	vehicle ep09{definition::fiz("ep09_v2/104e")};
	// the first position of the EP09 runs on field shunting step 2
	CHECK(ep09->RList[1].ScndAct == 2);
	CHECK(&ep09->motor_param(ep09->RList[1].ScndAct) == &ep09->MotorParam[2]);

	// a .fiz giving a step beyond the table: nothing checks it on loading
	vehicle broken{definition::fiz("ep09_v2/104e").set("RList", "rows", ep09_rlist_with_scndact("20"))};
	CHECK(broken->RList[1].ScndAct == 20);
	CHECK(broken->RList[1].ScndAct > MotorParametersArraySize);
}

TEST_CASE("motor parameters: motor_param() keeps a step from the .fiz inside the table") {
	vehicle broken{definition::fiz("ep09_v2/104e").set("RList", "rows", ep09_rlist_with_scndact("20"))};
	auto const &table{broken->MotorParam};
	// past the end: the last entry, rather than memory beyond the table
	CHECK(&broken->motor_param(broken->RList[1].ScndAct) == &table[MotorParametersArraySize]);
	CHECK(&broken->motor_param(MotorParametersArraySize + 1) == &table[MotorParametersArraySize]);
	// below the start: the first entry
	CHECK(&broken->motor_param(-1) == &table[0]);
	// inside the table: the entry itself
	for (int step = 0; step <= MotorParametersArraySize; ++step) {
		CHECK(&broken->motor_param(step) == &table[step]);
	}
}
