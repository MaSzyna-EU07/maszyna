#include "stdafx.h"
#include "doctest.h"

#include "simulation/simulationsnapshot.h"
#include "utilities/parser.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <sstream>
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
	    "not.a//comment",
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
		entry.set(std::format("key{}", index++), value);

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
	std::vector const values{
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
	snapshot::take_refusals();
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
	CHECK(snapshot::take_refusals().size() == 1);

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
		// not a damage: a reader takes what it knows of it
		auto const read{snapshot::parse("snapshot 99 endsnapshot")};
		CHECK(read.version() == 99);
		CHECK(read.problems.empty());
		REQUIRE(read.warnings.size() == 1);
		CHECK(read.warnings[0].find("99") != std::string::npos);
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
	SUBCASE("a vehicle which stays where the scenery put it is parted from the ones put where the snapshot has them")
	{
		// w1 couldn't be put where the snapshot has it: it's left out of Present, and its entry out of Wanted
		std::vector<std::string> const placed{"loco", "w2", "w3", "other"};
		std::vector<link> const current{{"loco", 1, "w1", 0, full}, {"w1", 1, "w2", 0, full}};
		std::vector<link> const wanted{{"loco", 0, "", 0, 0}, {"loco", 1, "w1", 0, full}, {"w2", 0, "w1", 1, full}, {"w2", 1, "", 0, 0}};
		auto const plan{snapshot::plan_couplings(current, wanted, placed)};
		REQUIRE(plan.part.size() == 2);
		CHECK(plan.part[0] == link{"loco", 1, "w1", 0, full});
		CHECK(plan.part[1] == link{"w1", 1, "w2", 0, full});
		CHECK(plan.join.empty());
		CHECK(plan.change.empty());
		CHECK(plan.missing.size() == 2);
	}
	SUBCASE("a vehicle which stays where the scenery put it keeps the vehicles the snapshot doesn't know")
	{
		std::vector<std::string> const placed{"loco", "w2", "w3", "other"};
		std::vector<link> const current{{"other", 1, "w1", 0, full}, {"loco", 1, "w2", 0, full}};
		std::vector<link> const wanted{{"loco", 1, "w2", 0, full}, {"w2", 0, "loco", 1, full}};
		auto const plan{snapshot::plan_couplings(current, wanted, placed)};
		CHECK(plan.part.empty());
		CHECK(plan.join.empty());
		CHECK(plan.missing.empty());
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

namespace {

// whether one of the refusals told of names the key
bool refused(std::vector<std::string> const &Refusals, std::string_view const Key)
{
	return std::ranges::any_of(Refusals, [&](std::string_view const Line) { return Line.find(Key) != std::string_view::npos; });
}

// a vehicle entry with the couplings given as text, as a damaged file can have them
snapshot::record coupled(std::string const &Name, std::string const &End, std::string const &Flags)
{
	snapshot::record entry{"vehicle", Name};
	entry.fields.emplace_back("coupler0", "other");
	entry.fields.emplace_back("coupler0end", End);
	entry.fields.emplace_back("coupler0flags", Flags);
	return entry;
}

std::string file_text(std::filesystem::path const &File)
{
	std::ifstream input{File, std::ios::binary};
	std::ostringstream text;
	text << input.rdbuf();
	return text.str();
}

} // namespace

TEST_CASE("snapshot: a value out of the range of what it's read into is refused and told of")
{
	snapshot::take_refusals();
	snapshot::record entry{"vehicle", "eu07-424"};
	entry.fields = {{"huge", "1e18"}, {"nan", "nan"}, {"fraction", "2.5"}, {"negative", "-1"}, {"inf", "inf"}, {"big", "1e300"}, {"three", "3"}};

	SUBCASE("a whole number has to be whole and fit its type")
	{
		auto number{7};
		CHECK_FALSE(entry.get("huge", number));
		CHECK_FALSE(entry.get("nan", number));
		CHECK_FALSE(entry.get("fraction", number));
		CHECK_FALSE(entry.get("inf", number));
		CHECK(number == 7);
		std::size_t count{7};
		CHECK_FALSE(entry.get("negative", count));
		CHECK(count == 7);
		CHECK(entry.get("huge", count));
		CHECK(count == 1000000000000000000ULL);
		auto flag{false};
		CHECK_FALSE(entry.get("nan", flag));
		enum class mode { off, on };
		auto setting{mode::off};
		CHECK_FALSE(entry.get("huge", setting));
		CHECK(setting == mode::off);
		auto const refusals{snapshot::take_refusals()};
		CHECK(refusals.size() == 7);
		CHECK(refused(refusals, "huge \"1e18\""));
		CHECK(refused(refusals, "eu07-424"));
	}
	SUBCASE("a measure takes what isn't a number and the infinities, but not more than its type holds")
	{
		auto value{0.0};
		CHECK(entry.get("nan", value));
		CHECK(std::isnan(value));
		CHECK(entry.get("inf", value));
		CHECK(std::isinf(value));
		auto single{1.f};
		CHECK_FALSE(entry.get("big", single));
		CHECK(single == 1.f);
		CHECK(snapshot::take_refusals().size() == 1);
	}
	SUBCASE("a value which has to be in a range")
	{
		auto end{0};
		CHECK_FALSE(entry.get("three", end, 0, 1));
		CHECK(end == 0);
		auto measure{0.5};
		CHECK_FALSE(entry.get("nan", measure, 0.0, 1.0));
		CHECK(measure == 0.5);
		CHECK(entry.value("three", -1, 0, 1) == -1);
		CHECK(entry.value("three", -1, 0, 5) == 3);
		auto const refusals{snapshot::take_refusals()};
		REQUIRE(refusals.size() == 3);
		CHECK(refused(refusals, "0 .. 1"));
	}
	SUBCASE("a value brought into its range")
	{
		auto position{0};
		CHECK(entry.get_clamped("three", position, 0, 2));
		CHECK(position == 2);
		CHECK(entry.get_clamped("negative", position, 0, 2));
		CHECK(position == 0);
		CHECK_FALSE(entry.get_clamped("huge", position, 0, 2));
		CHECK(position == 0);
		auto const refusals{snapshot::take_refusals()};
		REQUIRE(refusals.size() == 3);
		CHECK(refused(refusals, "2 is taken"));
	}
	SUBCASE("a number of a list as a whole number")
	{
		CHECK(snapshot::whole_number(3.0, 0, 5) == 3);
		CHECK_FALSE(snapshot::whole_number(3.5, 0, 5).has_value());
		CHECK_FALSE(snapshot::whole_number(6.0, 0, 5).has_value());
		CHECK_FALSE(snapshot::whole_number(std::nan(""), 0, 5).has_value());
		CHECK_FALSE(snapshot::whole_number(1e18, std::numeric_limits<int>::min(), std::numeric_limits<int>::max()).has_value());
	}
	SUBCASE("a great many refusals are counted, not listed")
	{
		for (auto index{0}; index < 150; ++index)
			entry.refuse("huge", "a test");
		auto const refusals{snapshot::take_refusals()};
		REQUIRE(refusals.size() == 101);
		CHECK(refusals.back() == "and 50 more values refused");
		CHECK(snapshot::take_refusals().empty());
	}
}

TEST_CASE("snapshot: the couplings of a damaged file are refused where they'd point past a vehicle's couplers")
{
	snapshot::take_refusals();
	std::vector const entries{
	    coupled("end3", "3", "3"),        // past the two couplers: an index past the array of them
	    coupled("end2", "2", "3"),        // the same
	    coupled("endminus", "-1", "3"),   //
	    coupled("endnan", "nan", "3"),    //
	    coupled("endhuge", "1e18", "3"),  //
	    coupled("endhalf", "0.5", "3"),   //
	    coupled("flagsbig", "1", "4096"), // not a flag of the couplings
	    coupled("flagsminus", "1", "-1"), //
	    coupled("good", "1", "3"),        // the one which stays
	};
	std::vector<snapshot::record const *> pointers;
	for (auto const &entry : entries)
		pointers.emplace_back(&entry);
	auto const links{snapshot::read_couplings(pointers)};
	REQUIRE(links.size() == 1);
	CHECK(links[0] == snapshot::coupling_link{"good", 0, "other", 1, 3});
	auto const refusals{snapshot::take_refusals()};
	CHECK(refusals.size() == 8);
	CHECK(refused(refusals, "end3"));
	CHECK(refused(refusals, "flagsbig"));
}

TEST_CASE("snapshot: couplings which would close a ring of vehicles are refused")
{
	using link = snapshot::coupling_link;
	std::vector<std::string> const present{"a", "b", "c"};
	SUBCASE("two vehicles coupled at both ends")
	{
		std::vector<link> const wanted{{"a", 1, "b", 0, 3}, {"b", 1, "a", 0, 3}};
		auto const plan{snapshot::plan_couplings({}, wanted, present)};
		CHECK(plan.join.size() == 1);
		REQUIRE(plan.refused.size() == 1);
		CHECK(plan.refused[0] == link{"b", 1, "a", 0, 3});
	}
	SUBCASE("a vehicle coupled to itself")
	{
		auto const plan{snapshot::plan_couplings({}, {{"a", 0, "a", 1, 3}}, present)};
		CHECK(plan.join.empty());
		CHECK(plan.refused.size() == 1);
	}
	SUBCASE("the last of a consist the scenery made to its first")
	{
		std::vector<link> const current{{"a", 1, "b", 0, 3}, {"b", 1, "c", 0, 3}};
		auto const plan{snapshot::plan_couplings(current, {{"c", 1, "a", 0, 3}}, present)};
		CHECK(plan.join.empty());
		CHECK(plan.refused.size() == 1);
	}
	SUBCASE("a consist turned round is no ring: the old link is parted first")
	{
		std::vector<link> const current{{"a", 1, "b", 0, 3}};
		std::vector<link> const wanted{{"a", 0, "b", 1, 3}, {"a", 1, "", 0, 0}, {"b", 0, "", 0, 0}};
		auto const plan{snapshot::plan_couplings(current, wanted, present)};
		CHECK(plan.part.size() == 1);
		CHECK(plan.join.size() == 1);
		CHECK(plan.refused.empty());
	}
}

TEST_CASE("snapshot: counts and indices of a damaged file stay within what the entry holds")
{
	snapshot::take_refusals();
	snapshot::record entry{"driver", "eu07-424"};
	entry.fields = {{"table", "3"}, {"p0", "x"}, {"p1", "x"}, {"p2", "x"}, {"semnext", "2"}, {"semnextstop", "3"}};
	SUBCASE("a count of what the entry lists")
	{
		CHECK(snapshot::read_count(entry, "table") == 3u);
		for (auto const *value : {"1e18", "-1", "nan", "inf", "2.5", "7", "huge"})
		{
			CAPTURE(value);
			entry.fields[0].second = value;
			CHECK_FALSE(snapshot::read_count(entry, "table").has_value());
		}
		// the count can't be past the keys of the entry
		entry.fields[0].second = "6";
		CHECK(snapshot::read_count(entry, "table") == 6u);
		CHECK_FALSE(snapshot::read_count(entry, "limits").has_value());
		CHECK(snapshot::take_refusals().size() == 7);
	}
	SUBCASE("an index into a table")
	{
		CHECK(snapshot::read_index(entry, "semnext", 3) == 2u);
		CHECK_FALSE(snapshot::read_index(entry, "semnextstop", 3).has_value());
		CHECK_FALSE(snapshot::read_index(entry, "semnext", 0).has_value());
		CHECK_FALSE(snapshot::read_index(entry, "absent", 3).has_value());
		auto const refusals{snapshot::take_refusals()};
		REQUIRE(refusals.size() == 2);
		CHECK(refused(refusals, "past the end"));
	}
}

TEST_CASE("snapshot: a damaged file isn't put over the scenery unless that's forced")
{
	auto document{sample()};
	document.records.front().set("entries", document.records.size());
	auto const text{snapshot::to_text(document)};
	SUBCASE("a whole file is used")
	{
		auto const read{snapshot::parse(text)};
		CHECK(read.problems.empty());
		CHECK(snapshot::accepted(read, false));
	}
	SUBCASE("a file cut in the middle of an entry")
	{
		auto const read{snapshot::parse(text.substr(0, text.find("endmemcell")))};
		CHECK_FALSE(read.problems.empty());
		CHECK_FALSE(snapshot::accepted(read, false));
		CHECK(snapshot::accepted(read, true));
		// what could be read is there for --load-snapshot-force
		CHECK(read.find("memcell", "sem_a") != nullptr);
	}
	SUBCASE("a file cut between two entries, which reads without a fault but for the count of its header")
	{
		auto const read{snapshot::parse(text.substr(0, text.find("switch")))};
		REQUIRE(read.problems.size() == 1);
		CHECK(read.problems[0].find("cut short") != std::string::npos);
		CHECK_FALSE(snapshot::accepted(read, false));
		CHECK(snapshot::accepted(read, true));
	}
	SUBCASE("a file without a header isn't a snapshot, forced or not")
	{
		auto const read{snapshot::parse("memcell a text x endmemcell")};
		CHECK_FALSE(snapshot::accepted(read, false));
		CHECK_FALSE(snapshot::accepted(read, true));
		CHECK_FALSE(snapshot::accepted(snapshot::load("no/such/file.snapshot"), true));
	}
	SUBCASE("a file of a newer version is used for what it holds that's known")
	{
		auto const read{snapshot::parse("snapshot 2 endsnapshot memcell a text x endmemcell")};
		CHECK(snapshot::accepted(read, false));
	}
	SUBCASE("the switch which forces it, beside the file to load")
	{
		std::vector<char const *> const arguments{"eu07", "--load-snapshot", "a.snapshot", "--load-snapshot-force"};
		auto const line{snapshot::parse_command_line(arguments)};
		CHECK(line.error.empty());
		CHECK(line.load == "a.snapshot");
		CHECK(line.force);
		CHECK(snapshot::option_value_count("--load-snapshot-force") == 0);
		std::vector<char const *> const plain{"eu07", "--load-snapshot", "a.snapshot"};
		CHECK_FALSE(snapshot::parse_command_line(plain).force);
	}
}

TEST_CASE("snapshot: a file is written whole or not at all")
{
	auto const folder{std::filesystem::path{EU07_UNIT_TESTS_SCRATCH} / "snapshot_write"};
	std::filesystem::remove_all(folder);
	std::filesystem::create_directories(folder);
	auto const file{folder / "state.snapshot"};
	auto const temporary{folder / "state.snapshot.tmp"};
	auto const document{sample()};

	// a new file
	CHECK(snapshot::write_file(file.string(), document).empty());
	CHECK(file_text(file) == snapshot::to_text(document));
	CHECK_FALSE(std::filesystem::exists(temporary));
	// one written over another
	auto other{sample()};
	other.add("memcell", "more").set("text", "x");
	CHECK(snapshot::write_file(file.string(), other).empty());
	CHECK(file_text(file) == snapshot::to_text(other));
	CHECK_FALSE(std::filesystem::exists(temporary));

	SUBCASE("the stream fails half way: the file stays as it was, and nothing's left beside it")
	{
		auto const error{snapshot::write_file(file.string(), [](std::ostream &Output) {
			Output << "snapshot 1 scenery td.scn endsnapshot\nmemcell a te";
			Output.setstate(std::ios::badbit);
		})};
		CHECK_FALSE(error.empty());
		CHECK(file_text(file) == snapshot::to_text(other));
		CHECK_FALSE(std::filesystem::exists(temporary));
	}
	SUBCASE("the file can't be made at all: the old one stays")
	{
		std::filesystem::create_directory(temporary);
		CHECK_FALSE(snapshot::write_file(file.string(), document).empty());
		CHECK(file_text(file) == snapshot::to_text(other));
		std::filesystem::remove(temporary);
	}
	SUBCASE("a folder which isn't there")
	{
		CHECK_FALSE(snapshot::write_file((folder / "no" / "such.snapshot").string(), document).empty());
		CHECK_FALSE(std::filesystem::exists(folder / "no"));
	}
	std::filesystem::remove_all(folder);
}

TEST_CASE("snapshot: a vehicle put further than a metre from the place in the world its entry gives isn't where it was")
{
	snapshot::record entry{"vehicle", "eu07-424"};
	entry.set("x", 1200.5).set("y", 3.25).set("z", -840.0);
	CHECK(snapshot::placed_where_saved(entry, 1200.5, 3.25, -840.0));
	CHECK(snapshot::placed_where_saved(entry, 1201.0, 3.25, -840.5));
	// a track named "#12" which is another one after a track was added before it in the scenery
	CHECK_FALSE(snapshot::placed_where_saved(entry, 1202.0, 3.25, -840.0));
	CHECK_FALSE(snapshot::placed_where_saved(entry, 1200.5, 3.25, -2840.0));
	CHECK_FALSE(snapshot::placed_where_saved(entry, std::nan(""), 3.25, -840.0));
	// an entry which doesn't say where it is can't tell
	snapshot::record unsaid{"vehicle", "eu07-424"};
	CHECK(snapshot::placed_where_saved(unsaid, 0.0, 0.0, 0.0));
	unsaid.set("x", 1.0).set("y", 2.0);
	CHECK(snapshot::placed_where_saved(unsaid, 500.0, 0.0, 0.0));
	unsaid.set("z", "nan");
	CHECK(snapshot::placed_where_saved(unsaid, 500.0, 0.0, 0.0));
}

TEST_CASE("snapshot: the fingerprint of a changed scenery says what changed")
{
	snapshot::record saved{"snapshot", "1"};
	saved.set("scenery", "td.scn").set("tracks", 812).set("events", 300).set("names", "0123456789abcdef");
	snapshot::record now;
	now.set("tracks", 812).set("events", 300).set("names", "0123456789abcdef");
	CHECK(snapshot::fingerprint_changes(saved, now).empty());
	// a track added to the scenery
	now.set("tracks", 813).set("names", "fedcba9876543210");
	CHECK(snapshot::fingerprint_changes(saved, now) == "tracks 813 (snapshot 812), names fedcba9876543210 (snapshot 0123456789abcdef)");
	// a snapshot which doesn't say goes by what it says
	snapshot::record older{"snapshot", "1"};
	CHECK(snapshot::fingerprint_changes(older, now).empty());
}

TEST_CASE("snapshot: a snapshot saved where one of the same name is takes a name of its own")
{
	auto const folder{std::filesystem::path{EU07_UNIT_TESTS_SCRATCH} / "snapshot_names"};
	std::filesystem::remove_all(folder);
	std::filesystem::create_directories(folder);
	auto const file{(folder / "td_2026-11-24_10-32-15.snapshot").generic_string()};
	CHECK(snapshot::free_file(file) == file);
	std::ofstream{file} << "snapshot 1 endsnapshot";
	auto const second{snapshot::free_file(file)};
	CHECK(second == (folder / "td_2026-11-24_10-32-15_2.snapshot").generic_string());
	std::ofstream{second} << "snapshot 1 endsnapshot";
	CHECK(snapshot::free_file(file) == (folder / "td_2026-11-24_10-32-15_3.snapshot").generic_string());
	// a name without an extension
	auto const bare{(folder / "state").generic_string()};
	std::ofstream{bare} << "snapshot 1 endsnapshot";
	CHECK(snapshot::free_file(bare) == bare + "_2");
	std::filesystem::remove_all(folder);
}
