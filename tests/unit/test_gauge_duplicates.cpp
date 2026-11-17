#include "stdafx.h"
#include "doctest.h"

#include "vehicle/gaugeduplicates.h"

// cab controls with more than one submodel (eu07/maszyna#527): a definition of a control in the .mmd marked
// "duplicate: yes" adds a copy of the control already defined in the cab, instead of replacing it. TGauge itself
// (3d models, sounds) is not part of the core library, the rule which decides it is

namespace {

// stand-ins for the 3d models of the cab
struct model {};

} // namespace

TEST_CASE("gauge duplicates: the values of the duplicate key which mark a duplicate") {
	CHECK(gauge_duplicates::is_duplicate("yes"));
	CHECK(gauge_duplicates::is_duplicate("true"));
	CHECK(gauge_duplicates::is_duplicate("1"));
	CHECK_FALSE(gauge_duplicates::is_duplicate("no"));
	CHECK_FALSE(gauge_duplicates::is_duplicate("false"));
	CHECK_FALSE(gauge_duplicates::is_duplicate("0"));
	CHECK_FALSE(gauge_duplicates::is_duplicate(""));
}

TEST_CASE("gauge duplicates: a duplicate adds a copy to the control loaded from either model of the cab") {
	model cab;
	model lowpoly;
	CHECK(gauge_duplicates::adds_copy<model>(true, &cab, &cab, &lowpoly));
	CHECK(gauge_duplicates::adds_copy<model>(true, &lowpoly, &cab, &lowpoly));
	// a cab without a low poly interior
	CHECK(gauge_duplicates::adds_copy<model>(true, &cab, &cab, nullptr));
}

TEST_CASE("gauge duplicates: a definition not marked as a duplicate replaces the control, as before") {
	model cab;
	model lowpoly;
	CHECK_FALSE(gauge_duplicates::adds_copy<model>(false, &cab, &cab, &lowpoly));
	CHECK_FALSE(gauge_duplicates::adds_copy<model>(false, nullptr, &cab, &lowpoly));
}

TEST_CASE("gauge duplicates: with nothing loaded in this cab a duplicate is the control itself") {
	model cab;
	model lowpoly;
	model previouscab;
	// the control isn't defined before it
	CHECK_FALSE(gauge_duplicates::adds_copy<model>(true, nullptr, &cab, &lowpoly));
	CHECK_FALSE(gauge_duplicates::adds_copy<model>(true, nullptr, &cab, nullptr));
	// the control is left over from the cab loaded before
	CHECK_FALSE(gauge_duplicates::adds_copy<model>(true, &previouscab, &cab, &lowpoly));
	CHECK_FALSE(gauge_duplicates::adds_copy<model>(true, &previouscab, &cab, nullptr));
}
