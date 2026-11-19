#include "stdafx.h"
#include "doctest.h"

#include "simulation/simulationsnapshot.h"
#include "utilities/parser.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

// the text of a simulation snapshot (simulation/simulationsnapshot.cpp): entries written and read back, what the
// reader makes of a damaged file, the names of the things of the scenery, the couplings to change

namespace {

namespace snapshot = simulation::snapshot;

snapshot::document sample()
{
	snapshot::document document;
	document.add("snapshot", "1").set("scenery", "td.scn").set("seed", 1234567890U).set("tracks", 812);
	document.add("time", "now").set("year", 2026).set("month", 11).set("day", 18).set("hour", 10).set("minute", 32).set("second", 15.125);
	document.add("memcell", "sem_a").set("text", "SetVelocity").set("value1", 40.0).set("value2", -1.0);
	document.add("memcell", "status").set("text", "").set("value1", 0.1).set("value2", 1e-300);
	document.add("switch", "#17").set("state", 1);
	document.add("event", "sem_a_s1").set("sibling", 0).set("delay", 12.5).set("activator", "EU07-424");
	return document;
}

} // namespace

TEST_CASE("snapshot: a document written and read back is the same, and so is its text")
{
	auto const document{sample()};
	auto const text{snapshot::to_text(document)};
	auto const read{snapshot::parse(text)};
	CHECK(read.problems.empty());
	CHECK(read.records == document.records);
	CHECK(snapshot::to_text(read) == text);
	CHECK(read.version() == snapshot::format_version);
	REQUIRE(read.header() != nullptr);
	CHECK(read.header()->text("scenery") == "td.scn");
	CHECK(read.header()->value("seed", 0U) == 1234567890U);
}

TEST_CASE("snapshot: the values keep their case, their spaces and whatever else they hold")
{
	std::vector<std::string> const values{
	    "plain",
	    "MiXeD_Case",
	    "",
	    "<>",
	    "<",
	    "<<double",
	    "two words",
	    " leading and trailing ",
	    "semi;colon",
	    "quote\"inside",
	    "back\\slash",
	    "ends with backslash\\",
	    "http://not.a/comment",
	    "a /* b */ c",
	    "tab\there",
	    "line\nbreak",
	    "include",
	    "endmemcell",
	    "none",
	    "Nowa Wie\xc5\x9b",
	    "(p1)",
	    "#12",
	    "$name",
	};
	snapshot::document document;
	document.add("snapshot", "1");
	auto &entry{document.add("memcell", "cell with a space")};
	auto index{0};
	for (auto const &value : values)
		entry.set("key" + std::to_string(index++), value);

	auto const text{snapshot::to_text(document)};
	auto const read{snapshot::parse(text)};
	CAPTURE(text);
	CHECK(read.problems.empty());
	REQUIRE(read.records.size() == 2);
	CHECK(read.records[1].name == "cell with a space");
	REQUIRE(read.records[1].fields.size() == values.size());
	index = 0;
	for (auto const &value : values)
	{
		CAPTURE(index);
		CHECK(read.records[1].fields[index].second == value);
		++index;
	}
	CHECK(snapshot::to_text(read) == text);
}

TEST_CASE("snapshot: numbers read back as the very same values")
{
	std::vector<double> const values{
	    0.0,
	    -0.0,
	    1.0,
	    -1.0,
	    0.1,
	    1.0 / 3.0,
	    5.000000000000001,
	    123456789.123456789,
	    1e-300,
	    1e300,
	    std::numeric_limits<double>::min(),
	    std::numeric_limits<double>::max(),
	    std::numeric_limits<double>::denorm_min(),
	    std::numeric_limits<double>::epsilon(),
	    std::numeric_limits<double>::infinity(),
	    -std::numeric_limits<double>::infinity(),
	};
	for (auto const value : values)
	{
		auto const text{snapshot::number_text(value)};
		CAPTURE(text);
		auto const read{snapshot::text_number(text)};
		REQUIRE(read.has_value());
		CHECK(*read == value);
		CHECK(std::signbit(*read) == std::signbit(value));
	}
	auto const nan{snapshot::text_number(snapshot::number_text(std::nan("")))};
	REQUIRE(nan.has_value());
	CHECK(std::isnan(*nan));

	// single precision values are written as short as they can be, and read back the same
	for (auto const value : {0.1f, 1.435f, 3.4028235e38f, 1.17549435e-38f, -2.5f})
	{
		snapshot::record entry{"vehicle", "test"};
		entry.set("value", value);
		CAPTURE(entry.text("value"));
		CHECK(entry.value("value", 0.f) == value);
	}
	// whole numbers stay whole
	CHECK(snapshot::number_text(5.0) == "5");
	CHECK(snapshot::number_text(-120.0) == "-120");

	CHECK_FALSE(snapshot::text_number("").has_value());
	CHECK_FALSE(snapshot::text_number("abc").has_value());
	CHECK_FALSE(snapshot::text_number("12abc").has_value());
	CHECK_FALSE(snapshot::text_number("1 2").has_value());
	CHECK(snapshot::text_number("+2.5") == 2.5);
}

TEST_CASE("snapshot: an entry gives its values in the type asked for, and leaves alone what it doesn't have")
{
	snapshot::record entry{"vehicle", "eu07-424"};
	entry.set("battery", true).set("mains", false).set("mainctrl", 12).set("pipe", 4.85).set("load", "coal").set("direction", -1);
	CHECK(entry.value("battery", false));
	CHECK_FALSE(entry.value("mains", true));
	CHECK(entry.value("mainctrl", 0) == 12);
	CHECK(entry.value("pipe", 0.0) == 4.85);
	CHECK(entry.text("load") == "coal");
	CHECK(entry.value("direction", 0) == -1);

	// a key the entry doesn't have
	auto untouched{7};
	CHECK_FALSE(entry.get("scndctrl", untouched));
	CHECK(untouched == 7);
	CHECK(entry.value("scndctrl", 3) == 3);
	CHECK(entry.text("destination", "none") == "none");
	// a value which isn't a number, asked for as one
	CHECK_FALSE(entry.get("load", untouched));
	CHECK(untouched == 7);

	// a key set again keeps its place
	entry.set("mains", true);
	CHECK(entry.fields[1].first == "mains");
	CHECK(entry.fields[1].second == "1");
	CHECK(entry.fields.size() == 6);

	enum class mode { off, on, automatic };
	entry.set("mode", mode::automatic);
	CHECK(entry.value("mode", mode::off) == mode::automatic);
}

TEST_CASE("snapshot: the reader says what's wrong with a damaged file and keeps what it could read")
{
	SUBCASE("no header")
	{
		auto const read{snapshot::parse("memcell a text x value1 1 value2 2 endmemcell")};
		CHECK(read.version() == 0);
		REQUIRE(read.problems.size() == 1);
		CHECK(read.problems[0].find("snapshot") != std::string::npos);
		CHECK(read.records.size() == 1);
	}
	SUBCASE("empty file")
	{
		auto const read{snapshot::parse("// nothing here\n")};
		CHECK(read.records.empty());
		CHECK(read.problems.size() == 1);
	}
	SUBCASE("cut in the middle of an entry")
	{
		auto const read{snapshot::parse("snapshot 1 scenery td.scn endsnapshot memcell a text x value1")};
		REQUIRE(read.records.size() == 2);
		CHECK(read.records[1].text("text") == "x");
		REQUIRE(read.problems.size() == 1);
		CHECK(read.problems[0].find("value1") != std::string::npos);
	}
	SUBCASE("entry not closed")
	{
		auto const read{snapshot::parse("snapshot 1 endsnapshot memcell a text x")};
		REQUIRE(read.problems.size() == 1);
		CHECK(read.problems[0].find("endmemcell") != std::string::npos);
	}
	SUBCASE("key without a value")
	{
		auto const read{snapshot::parse("snapshot 1 endsnapshot memcell a text endmemcell switch s state 1 endswitch")};
		REQUIRE(read.problems.size() == 1);
		// the entries after it are read all the same
		REQUIRE(read.records.size() == 3);
		CHECK(read.records[2].value("state", 0) == 1);
	}
	SUBCASE("newer version")
	{
		auto const read{snapshot::parse("snapshot 99 endsnapshot")};
		CHECK(read.version() == 99);
		REQUIRE(read.problems.size() == 1);
		CHECK(read.problems[0].find("99") != std::string::npos);
	}
	SUBCASE("missing file")
	{
		auto const read{snapshot::load("no/such/file.snapshot")};
		CHECK(read.records.empty());
		CHECK(read.problems.size() == 1);
	}
}

TEST_CASE("snapshot: entries and keys of a newer version pass through a reader which doesn't know them")
{
	auto const text{std::string{"snapshot 1 scenery td.scn endsnapshot\n"
	                            "hologram deck3 brightness 0.5 endhologram\n"
	                            "memcell a text x value1 1 value2 2 colour red endmemcell\n"}};
	auto const read{snapshot::parse(text)};
	CHECK(read.problems.empty());
	REQUIRE(read.records.size() == 3);
	CHECK(read.of("hologram").size() == 1);
	REQUIRE(read.find("memcell", "a") != nullptr);
	CHECK(read.find("memcell", "a")->text("colour") == "red");
	CHECK(read.find("memcell", "b") == nullptr);
	// comments, line breaks and the semicolons of the scenery files are all the same to the reader
	auto const loose{snapshot::parse("snapshot 1 scenery td.scn endsnapshot // header\n\n/* the cells */ memcell a\n\ttext x;\n value1 1 value2 2 colour red\nendmemcell")};
	CHECK(loose.problems.empty());
	REQUIRE(loose.records.size() == 2);
	CHECK(loose.records[1] == *read.find("memcell", "a"));
}

TEST_CASE("snapshot: things of the scenery are named by their own names, or by their place in the table")
{
	CHECK(snapshot::reference("tor_12", 5, true) == "tor_12");
	// a name which doesn't lead to the thing, as a shared one or none at all
	CHECK(snapshot::reference("tor_12", 5, false) == "#5");
	CHECK(snapshot::reference("", 0, true) == "#0");
	// a name which reads as a place
	CHECK(snapshot::reference("#3", 9, true) == "#9");

	CHECK(snapshot::reference_index("#5") == 5U);
	CHECK(snapshot::reference_index("#0") == 0U);
	CHECK_FALSE(snapshot::reference_index("tor_12").has_value());
	CHECK_FALSE(snapshot::reference_index("#").has_value());
	CHECK_FALSE(snapshot::reference_index("#1a").has_value());
	CHECK_FALSE(snapshot::reference_index("").has_value());
}

TEST_CASE("snapshot: the hash of the names tells sceneries apart")
{
	auto const hash{[](std::vector<std::string> const &Names) {
		snapshot::name_hash result;
		for (auto const &name : Names)
			result.add(name);
		return result.text();
	}};
	CHECK(hash({"a", "b"}) == hash({"a", "b"}));
	CHECK(hash({"a", "b"}) != hash({"b", "a"}));
	CHECK(hash({"ab", "c"}) != hash({"a", "bc"}));
	CHECK(hash({"a"}) != hash({"a", ""}));
	CHECK(hash({}).size() == 16);
}

TEST_CASE("snapshot: the couplings of the scenery are changed into the ones of the snapshot")
{
	using link = snapshot::coupling_link;
	constexpr int full{3};
	std::vector<std::string> const present{"loco", "w1", "w2", "w3", "other"};

	SUBCASE("nothing to do when they agree, whichever end a link is given from")
	{
		std::vector<link> const current{{"loco", 1, "w1", 0, full}, {"w1", 1, "w2", 0, full}};
		std::vector<link> const wanted{{"loco", 0, "", 0, 0}, {"loco", 1, "w1", 0, full}, {"w1", 0, "loco", 1, full}, {"w1", 1, "w2", 0, full},
		                               {"w2", 0, "w1", 1, full}, {"w2", 1, "", 0, 0}};
		auto const plan{snapshot::plan_couplings(current, wanted, present)};
		CHECK(plan.part.empty());
		CHECK(plan.join.empty());
		CHECK(plan.change.empty());
		CHECK(plan.missing.empty());
	}
	SUBCASE("a wagon left behind is parted")
	{
		std::vector<link> const current{{"loco", 1, "w1", 0, full}, {"w1", 1, "w2", 0, full}};
		std::vector<link> const wanted{{"loco", 0, "", 0, 0}, {"loco", 1, "w1", 0, full}, {"w1", 0, "loco", 1, full}, {"w1", 1, "", 0, 0},
		                               {"w2", 0, "", 0, 0}, {"w2", 1, "", 0, 0}};
		auto const plan{snapshot::plan_couplings(current, wanted, present)};
		REQUIRE(plan.part.size() == 1);
		CHECK(plan.part[0] == link{"w1", 1, "w2", 0, full});
		CHECK(plan.join.empty());
	}
	SUBCASE("a wagon picked up is joined, once")
	{
		std::vector<link> const current{{"loco", 1, "w1", 0, full}};
		std::vector<link> const wanted{{"loco", 1, "w1", 0, full}, {"w1", 0, "loco", 1, full}, {"w1", 1, "w3", 1, 1}, {"w3", 1, "w1", 1, 1}, {"w3", 0, "", 0, 0}};
		auto const plan{snapshot::plan_couplings(current, wanted, present)};
		CHECK(plan.part.empty());
		REQUIRE(plan.join.size() == 1);
		CHECK(plan.join[0] == link{"w1", 1, "w3", 1, 1});
	}
	SUBCASE("a locomotive run round its train parts at one end and joins at the other")
	{
		std::vector<link> const current{{"loco", 1, "w1", 0, full}, {"w1", 1, "w2", 0, full}};
		std::vector<link> const wanted{{"loco", 0, "w2", 1, full}, {"loco", 1, "", 0, 0}, {"w1", 0, "", 0, 0}, {"w1", 1, "w2", 0, full},
		                               {"w2", 0, "w1", 1, full}, {"w2", 1, "loco", 0, full}};
		auto const plan{snapshot::plan_couplings(current, wanted, present)};
		REQUIRE(plan.part.size() == 1);
		CHECK(plan.part[0] == link{"loco", 1, "w1", 0, full});
		REQUIRE(plan.join.size() == 1);
		CHECK(plan.join[0] == link{"loco", 0, "w2", 1, full});
	}
	SUBCASE("hoses taken off change the coupling without parting it")
	{
		std::vector<link> const current{{"loco", 1, "w1", 0, full}};
		std::vector<link> const wanted{{"loco", 1, "w1", 0, 1}, {"w1", 0, "loco", 1, 1}};
		auto const plan{snapshot::plan_couplings(current, wanted, present)};
		CHECK(plan.part.empty());
		CHECK(plan.join.empty());
		REQUIRE(plan.change.size() == 1);
		CHECK(plan.change[0] == link{"loco", 1, "w1", 0, 1});
	}
	SUBCASE("a coupler taken by a vehicle the snapshot doesn't know is freed for the one it wants there")
	{
		std::vector<link> const current{{"other", 1, "w1", 0, full}};
		std::vector<link> const wanted{{"loco", 1, "w1", 0, full}, {"w1", 0, "loco", 1, full}};
		auto const plan{snapshot::plan_couplings(current, wanted, present)};
		REQUIRE(plan.part.size() == 1);
		CHECK(plan.part[0] == link{"other", 1, "w1", 0, full});
		REQUIRE(plan.join.size() == 1);
	}
	SUBCASE("couplings between vehicles the snapshot doesn't know stay")
	{
		std::vector<link> const current{{"other", 1, "w3", 0, full}, {"loco", 1, "w1", 0, full}};
		std::vector<link> const wanted{{"loco", 1, "w1", 0, full}, {"w1", 0, "loco", 1, full}};
		auto const plan{snapshot::plan_couplings(current, wanted, present)};
		CHECK(plan.part.empty());
		CHECK(plan.join.empty());
	}
	SUBCASE("a vehicle the scenery no longer has can't be coupled to")
	{
		std::vector<link> const current{{"loco", 1, "w1", 0, full}};
		std::vector<link> const wanted{{"loco", 1, "gone", 0, full}, {"gone", 0, "loco", 1, full}, {"w1", 0, "", 0, 0}};
		auto const plan{snapshot::plan_couplings(current, wanted, present)};
		REQUIRE(plan.part.size() == 1);
		CHECK(plan.join.empty());
		REQUIRE(plan.missing.size() == 1);
		CHECK(plan.missing[0].other == "gone");
	}
}

TEST_CASE("snapshot: the command line says which snapshot to load and when to take others")
{
	auto const parse{[](std::vector<char const *> Arguments) {
		Arguments.insert(Arguments.begin(), "eu07");
		return snapshot::parse_command_line(Arguments);
	}};
	SUBCASE("none asked for")
	{
		auto const line{parse({"-s", "td.scn", "--test"})};
		CHECK(line.load.empty());
		CHECK(line.saves.empty());
		CHECK(line.error.empty());
	}
	SUBCASE("one to load, two to take")
	{
		auto const line{parse({"--test", "-s", "td.scn", "--load-snapshot", "saves/a.snapshot", "--snapshot-at", "120", "b.snapshot", "--snapshot-at", "60.5", "c.snapshot"})};
		CHECK(line.error.empty());
		CHECK(line.load == "saves/a.snapshot");
		REQUIRE(line.saves.size() == 2);
		CHECK(line.saves[0].time == 120.0);
		CHECK(line.saves[0].file == "b.snapshot");
		CHECK(line.saves[1].time == 60.5);
		CHECK(line.saves[1].file == "c.snapshot");
	}
	SUBCASE("the values of other options aren't taken for options")
	{
		auto const line{parse({"-s", "--load-snapshot", "-v", "--snapshot-at", "-ini", "--load-snapshot", "--test-timeout", "--snapshot-at"})};
		CHECK(line.load.empty());
		CHECK(line.saves.empty());
		CHECK(line.error.empty());
	}
	SUBCASE("a file name is missing")
	{
		CHECK_FALSE(parse({"--load-snapshot"}).error.empty());
		CHECK_FALSE(parse({"--snapshot-at", "60"}).error.empty());
		CHECK_FALSE(parse({"--snapshot-at"}).error.empty());
	}
	SUBCASE("the time isn't a number of seconds")
	{
		CHECK_FALSE(parse({"--snapshot-at", "soon", "a.snapshot"}).error.empty());
		CHECK_FALSE(parse({"--snapshot-at", "-5", "a.snapshot"}).error.empty());
		CHECK_FALSE(parse({"--snapshot-at", "nan", "a.snapshot"}).error.empty());
		CHECK(parse({"--snapshot-at", "0", "a.snapshot"}).error.empty());
	}
	SUBCASE("how many values the options of the simulator take")
	{
		CHECK(snapshot::option_value_count("-s") == 1);
		CHECK(snapshot::option_value_count("--load-snapshot") == 1);
		CHECK(snapshot::option_value_count("--snapshot-at") == 2);
		CHECK(snapshot::option_value_count("--test") == 0);
		CHECK(snapshot::option_value_count("td.scn") == 0);
	}
}
