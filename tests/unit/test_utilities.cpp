#include "stdafx.h"
#include "doctest.h"
#include "scratch.h"

#include "utilities/Globals.h"
#include "utilities/utilities.h"

// the helpers of utilities/utilities.cpp and utilities.h as they behave now: the numbers and texts they give, their
// oddities included. the ones which look like mistakes are listed in COVERAGE.md. the helpers which read from a
// parser (LoadPoint, the sets, the map) are in test_parser.cpp

namespace {

// puts the debug mode back as it was
struct debug_mode {
	bool const before{DebugModeFlag};

	explicit debug_mode(bool const Enabled) { DebugModeFlag = Enabled; }
	debug_mode(debug_mode const &) = delete;
	debug_mode &operator=(debug_mode const &) = delete;
	~debug_mode() { DebugModeFlag = before; }
};

// the polish letters in windows-1250, as in test_win1250.cpp
std::string const kWin1250{"\xA5\xC6\xCA\xA3\xD1\xD3\x8C\x8F\xAF\xB9\xE6\xEA\xB3\xF1\xF3\x9C\x9F\xBF"};

} // namespace

// time ----------------------------------------------------------------------------------------------------------------

TEST_CASE("utilities: the time between two times of a day is in minutes, the shorter way round midnight") {
	CHECK(CompareTime(10, 0, 10, 30) == 30.0);
	CHECK(CompareTime(10, 30, 10, 0) == -30.0);
	// 23:50 to 0:05 is a quarter later, not a day earlier
	CHECK(CompareTime(23, 50, 0, 5) == 15.0);
	CHECK(CompareTime(0, 5, 23, 50) == -15.0);
	// half a day stays as it is counted
	CHECK(CompareTime(0, 0, 12, 0) == 720.0);
	CHECK(CompareTime(12, 0, 0, 0) == -720.0);
	// a negative hour stands for no time given
	CHECK(CompareTime(10, 0, -1, 30) == 0.0);
}

TEST_CASE("utilities: the current time is given as a text") {
	std::string now;
	try {
		now = Now();
	}
	catch (std::runtime_error const &error) {
		// the local time needs the time zone database of the system, which a bare one may be without
		MESSAGE("Now() has no time zone to tell the local time in: " << error.what());
		return;
	}
	CHECK_FALSE(now.empty());
}

TEST_CASE("utilities: a time is taken apart the same way in every thread") {
	// 2001-09-09 01:46:40 UTC, a Sunday
	std::time_t const time{1000000000};
	auto const utc{utc_tm(time)};
	CHECK(utc.tm_year == 101);
	CHECK(utc.tm_mon == 8);
	CHECK(utc.tm_mday == 9);
	CHECK(utc.tm_hour == 1);
	CHECK(utc.tm_min == 46);
	CHECK(utc.tm_sec == 40);
	CHECK(utc.tm_wday == 0);
	auto const epoch{utc_tm(0)};
	CHECK(epoch.tm_year == 70);
	CHECK(epoch.tm_yday == 0);
	CHECK(epoch.tm_hour == 0);
	CHECK(epoch.tm_wday == 4);
	// the local time is the one the system puts back together into the same moment
	auto local{local_tm(time)};
	CHECK(std::mktime(&local) == time);
}

// flags ---------------------------------------------------------------------------------------------------------------

TEST_CASE("utilities: setting a flag tells whether it changed anything, a negative value clears") {
	int flags{0};
	CHECK(SetFlag(flags, 4));
	CHECK(flags == 4);
	CHECK_FALSE(SetFlag(flags, 4));
	// some of the bits set already still count as a change
	CHECK(SetFlag(flags, 6));
	CHECK(flags == 6);
	CHECK(TestFlag(flags, 6));
	CHECK_FALSE(TestFlag(flags, 3));
	CHECK(TestFlagAny(flags, 3));
	CHECK_FALSE(TestFlagAny(flags, 1));
	CHECK(SetFlag(flags, -4));
	CHECK(flags == 2);
	CHECK_FALSE(SetFlag(flags, -4));
	CHECK_FALSE(SetFlag(flags, 0));
	CHECK(flags == 2);
}

TEST_CASE("utilities: clearing a flag needs all of its bits set") {
	int flags{1};
	CHECK_FALSE(ClearFlag(flags, 3));
	CHECK(flags == 1);
	CHECK(ClearFlag(flags, 1));
	CHECK(flags == 0);
	// no bits at all are always there to clear
	CHECK(ClearFlag(flags, 0));
	CHECK(flags == 0);
}

// random numbers ------------------------------------------------------------------------------------------------------

TEST_CASE("utilities: random whole numbers are within the bounds, both included, given in any order") {
	std::set<int> seen;
	for (auto draw{0}; draw < 200; ++draw) {
		auto const value{Random(3, 7)};
		CHECK(value >= 3);
		CHECK(value <= 7);
		seen.insert(value);
		auto const swapped{Random(7, 3)};
		CHECK(swapped >= 3);
		CHECK(swapped <= 7);
	}
	CHECK(seen.size() == 5);
	CHECK(Random(5, 5) == 5);
}

TEST_CASE("utilities: random numbers are within the bounds given in any order") {
	for (auto draw{0}; draw < 200; ++draw) {
		auto const value{Random(2.0, 4.0)};
		CHECK(value >= 2.0);
		CHECK(value <= 4.0);
		auto const swapped{Random(4.0, 2.0)};
		CHECK(swapped >= 2.0);
		CHECK(swapped <= 4.0);
		auto const unit{Random()};
		CHECK(unit >= 0.0);
		CHECK(unit <= 1.0);
		auto const upto{Random(10.0)};
		CHECK(upto >= 0.0);
		CHECK(upto <= 10.0);
		auto const local{LocalRandom(-1.0, 1.0)};
		CHECK(local >= -1.0);
		CHECK(local <= 1.0);
		auto const localswapped{LocalRandom(1.0, -1.0)};
		CHECK(localswapped >= -1.0);
		CHECK(localswapped <= 1.0);
		auto const localunit{LocalRandom()};
		CHECK(localunit >= 0.0);
		CHECK(localunit <= 1.0);
		auto const localupto{LocalRandom(10.0)};
		CHECK(localupto >= 0.0);
		CHECK(localupto <= 10.0);
	}
}

TEST_CASE("utilities: the same seed gives the same random numbers") {
	std::vector<int> first;
	Global.random_engine.seed(42);
	for (auto draw{0}; draw < 5; ++draw) {
		first.push_back(Random(0, 1000000));
	}
	std::vector<int> second;
	Global.random_engine.seed(42);
	for (auto draw{0}; draw < 5; ++draw) {
		second.push_back(Random(0, 1000000));
	}
	CHECK(first == second);
}

TEST_CASE("utilities: the local random numbers have an engine of their own") {
	Global.local_random_engine.seed(5);
	auto const first{LocalRandom()};
	Global.local_random_engine.seed(5);
	// the numbers of the simulation drawn in between don't move it
	Random();
	Random(1, 6);
	CHECK(LocalRandom() == first);
}

TEST_CASE("utilities: a seed of the system's entropy is there to be drawn") {
	auto const first{true_random_seed()};
	auto const second{true_random_seed()};
	CHECK((first != 0 || second != 0));
}

TEST_CASE("utilities: the seed a text stands for is its number, or its hash the way Java counts it") {
	CHECK(seed_of("12345") == 12345u);
	CHECK(seed_of("0") == 0u);
	CHECK(seed_of("4294967295") == 4294967295u);
	CHECK(seed_of("hello") == 99162322u);
	CHECK(seed_of("test") == 3556498u);
	CHECK(seed_of("") == 0u);
	// a number which isn't a seed is a word: negative, or too big
	CHECK(seed_of("-5") == 1448u);
	CHECK(seed_of("4294967296") == 3632702254u);
	// the bytes over ascii count as positive
	CHECK(seed_of("\xB9") == 185u);
	// a word starting with digits is taken for the number in front of it
	CHECK(seed_of("12abc") == 12u);
}

TEST_CASE("utilities: a version 4 uuid is drawn from the random numbers of the simulation") {
	auto const id{generate_uuid_v4()};
	REQUIRE(id.size() == 36);
	auto wrong{0};
	for (std::size_t index{0}; index < id.size(); ++index) {
		auto const character{id[index]};
		if (index == 8 || index == 13 || index == 18 || index == 23) {
			wrong += (character == '-' ? 0 : 1);
		}
		else {
			wrong += ((character >= '0' && character <= '9') || (character >= 'a' && character <= 'f') ? 0 : 1);
		}
	}
	CAPTURE(id);
	CHECK(wrong == 0);
	// the version and the variant
	CHECK(id[14] == '4');
	CHECK((id[19] == '8' || id[19] == '9' || id[19] == 'a' || id[19] == 'b'));
	CHECK(generate_uuid_v4() != id);
	// the same seed gives the same one
	Global.random_engine.seed(11);
	auto const first{generate_uuid_v4()};
	Global.random_engine.seed(11);
	CHECK(generate_uuid_v4() == first);
}

TEST_CASE("utilities: a failure is drawn only over the threshold, the likelier the less the value is over it") {
	debug_mode const mode{false};
	for (auto draw{0}; draw < 50; ++draw) {
		// at the threshold and under it: never
		CHECK_FALSE(FuzzyLogic(1.0, 2.0, 1000.0));
		CHECK_FALSE(FuzzyLogic(2.0, 2.0, 1000.0));
		CHECK_FALSE(FuzzyLogicAI(1.0, 2.0, 1000.0));
		CHECK_FALSE(FuzzyLogicAI(2.0, 2.0, 1000.0));
		// a chance over 1 is a certainty, a chance of 0 never comes
		CHECK(FuzzyLogic(2.0, 1.0, 10.0));
		CHECK(FuzzyLogicAI(2.0, 1.0, 10.0));
		CHECK_FALSE(FuzzyLogic(2.0, 1.0, 0.0));
		CHECK_FALSE(FuzzyLogicAI(2.0, 1.0, 0.0));
	}
}

TEST_CASE("utilities: the debug mode keeps the failures away, but not the ones drawn regardless of it") {
	debug_mode const mode{true};
	CHECK_FALSE(FuzzyLogic(2.0, 1.0, 10.0));
	CHECK(FuzzyLogicAI(2.0, 1.0, 10.0));
}

// texts ---------------------------------------------------------------------------------------------------------------

TEST_CASE("utilities: a text is split at every separator, the empty parts kept") {
	CHECK(Split("a,b,,c", ',') == std::vector<std::string>{"a", "b", "", "c"});
	CHECK(Split("abc", ',') == std::vector<std::string>{"abc"});
	CHECK(Split("a,", ',') == std::vector<std::string>{"a", ""});
	CHECK(Split(",a", ',') == std::vector<std::string>{"", "a"});
	CHECK(Split("10|20", '|') == std::vector<std::string>{"10", "20"});
	// an empty text has no parts at all
	CHECK(Split("", ',').empty());
}

TEST_CASE("utilities: a name is split from the number it ends with") {
	CHECK(split_string_and_number("abc12") == std::pair<std::string, int>{"abc", 12});
	CHECK(split_string_and_number("t1") == std::pair<std::string, int>{"t", 1});
	CHECK(split_string_and_number("abc") == std::pair<std::string, int>{"abc", 0});
	CHECK(split_string_and_number("") == std::pair<std::string, int>{"", 0});
	CHECK(split_string_and_number("12") == std::pair<std::string, int>{"", 12});
	// a minus is a part of the number
	CHECK(split_string_and_number("x-5") == std::pair<std::string, int>{"x", -5});
	// the number is the first one, what follows it is dropped
	CHECK(split_string_and_number("a1b2") == std::pair<std::string, int>{"a", 1});
}

TEST_CASE("utilities: a name with a hyphen, or with a number too big, is an exception") {
	// the hyphen is taken for the start of a number
	CHECK_THROWS_AS(split_string_and_number("a-b"), std::invalid_argument);
	CHECK_THROWS_AS(split_string_and_number("door-"), std::invalid_argument);
	CHECK_THROWS_AS(split_string_and_number("a99999999999"), std::out_of_range);
}

TEST_CASE("utilities: whole numbers are written to a width, padded with spaces on the left") {
	CHECK(to_string(5, 3) == "  5");
	CHECK(to_string(-5, 4) == "  -5");
	CHECK(to_string(12345, 3) == "12345");
	CHECK(to_string(7, 0) == "7");
}

TEST_CASE("utilities: numbers are written with the decimals asked for, and to a width") {
	CHECK(to_string(3.14159, 2) == "3.14");
	CHECK(to_string(1.0, 3) == "1.000");
	CHECK(to_string(-1.26, 1) == "-1.3");
	CHECK(to_string(2.675, 2) == "2.67");
	// a half rounds to the even digit
	CHECK(to_string(2.5, 0) == "2");
	CHECK(to_string(3.14159, 2, 8) == "    3.14");
	CHECK(to_string(-1.5, 1, 6) == "  -1.5");
	// a number wider than the width isn't cut
	CHECK(to_string(123.456, 1, 3) == "123.5");
	CHECK(to_string(true) == "true");
	CHECK(to_string(false) == "false");
}

TEST_CASE("utilities: numbers are written in hex with capital digits, the negative ones as unsigned") {
	CHECK(to_hex_str(255) == "0x00FF");
	CHECK(to_hex_str(255, 2) == "0xFF");
	CHECK(to_hex_str(0, 8) == "0x00000000");
	CHECK(to_hex_str(0x12345) == "0x12345");
	CHECK(to_hex_str(-1) == "0xFFFFFFFF");
}

TEST_CASE("utilities: the minutes of a timetable are written with their tenths as a raised digit") {
	// two characters for the minutes, then the digit, a space for no tenths; the digits are utf-8
	CHECK(to_minutes_str(5.5f, true, 3) == "05\xE2\x81\xB5");
	CHECK(to_minutes_str(7.9f, true, 3) == "07\xE2\x81\xB9");
	CHECK(to_minutes_str(0.0f, true, 3) == "00 ");
	CHECK(to_minutes_str(5.0f, false, 3) == " 5 ");
	CHECK(to_minutes_str(12.3f, false, 3) == "12\xC2\xB3");
	CHECK(to_minutes_str(5.0f, false, 5) == "   5 ");
	// only the last digits fit
	CHECK(to_minutes_str(123.0f, false, 3) == "23 ");
}

TEST_CASE("utilities: minutes can't be written a single character wide") {
	// the padding is counted as one character less than nothing
	CHECK_THROWS_AS(to_minutes_str(5.0f, false, 1), std::length_error);
}

TEST_CASE("utilities: a text is read as a whole number, with a default for an empty one") {
	CHECK(stol_def("42", 7) == 42);
	CHECK(stol_def("-3", 7) == -3);
	CHECK(stol_def("  15 ", 7) == 15);
	CHECK(stol_def("12abc", 7) == 12);
	CHECK(stol_def("", 7) == 7);
	CHECK(stol_def("   ", 7) == 7);
	// a text which isn't a number gives zero, not the default
	CHECK(stol_def("abc", 7) == 0);
}

TEST_CASE("utilities: the case of the letters of ascii is changed") {
	CHECK(ToLower("AbC-12 Zz") == "abc-12 zz");
	CHECK(ToUpper("AbC-12 Zz") == "ABC-12 ZZ");
	CHECK(ToLower("").empty());
	// the other bytes stay, in the "C" locale the tests run in
	CHECK(ToUpper("\xB3\xC5\x82") == "\xB3\xC5\x82");
	CHECK(ToLower("\xA3\xC5\x81") == "\xA3\xC5\x81");
}

TEST_CASE("utilities: the polish letters of windows-1250 are replaced with the letters of ascii") {
	auto text{kWin1250};
	win1250_to_ascii(text);
	CHECK(text == "ACELNOSZZacelnoszz");
	// the rest of the text stays, other letters with marks too
	std::string other{"tor 1, caf\xE9"};
	win1250_to_ascii(other);
	CHECK(other == "tor 1, caf\xE9");
}

TEST_CASE("utilities: a text for the timetable loses its marks, control characters and, when asked, underscores") {
	// the letters up to 0xB0, which the table has in the same place whatever the compiler
	CHECK(Bezogonkow("\xA5\xA3\x8C\x8F\xAF\x9C\x9F") == "ALSZZsz");
	CHECK(Bezogonkow("Warszawa Zachodnia") == "Warszawa Zachodnia");
	CHECK(Bezogonkow("a\tb\nc") == "a b c");
	CHECK(Bezogonkow("Bia\xA3" "a_Podlaska") == "BiaLa_Podlaska");
	CHECK(Bezogonkow("Bia\xA3" "a_Podlaska", true) == "BiaLa Podlaska");
}

TEST_CASE("utilities: the polish letters from 0xB2 on lose their marks only in a build which makes one byte of the table's mark") {
	// the table of Bezogonkow() has a plus-minus sign for 0xB1, written as the character in the source, a utf-8 file.
	// MSVC puts it into the table as the one byte of the system's code page, and the table is as it was meant.
	// GCC and Clang put the two bytes of utf-8 there, which moves the entries after it by one (see COVERAGE.md)
	if (Bezogonkow("\xB1") == "\xB1") {
		CHECK(Bezogonkow(kWin1250) == "ACELNOSZZacelnoszz");
	}
	else {
		// the first byte of the sign for 0xB1, the second for 0xB2, and each letter after them as the entry before its own
		CHECK(Bezogonkow("\xB1\xB2") == "\xC2\xB1");
		CHECK(Bezogonkow(kWin1250) == "ALELDNSZZ,le,dnszl");
	}
}

TEST_CASE("utilities: utf-8 is decoded into code points of one to four bytes") {
	std::u32string decoded{U"old"};
	REQUIRE(utf8_to_utf32("a\xC5\x82\xE2\x82\xAC\xF0\x9F\x98\x80", decoded));
	REQUIRE(decoded.size() == 4);
	CHECK(decoded[0] == U'a');
	CHECK(decoded[1] == char32_t{0x142});
	CHECK(decoded[2] == char32_t{0x20AC});
	CHECK(decoded[3] == char32_t{0x1F600});
	REQUIRE(utf8_to_utf32("", decoded));
	CHECK(decoded.empty());
}

TEST_CASE("utilities: what isn't utf-8 is refused") {
	std::u32string decoded;
	// a byte which can't start a sequence
	CHECK_FALSE(utf8_to_utf32("\x80", decoded));
	CHECK_FALSE(utf8_to_utf32("\xF8\x88\x80\x80\x80", decoded));
	// a sequence cut short, or continued with a byte which doesn't continue one
	CHECK_FALSE(utf8_to_utf32("\xE2\x82", decoded));
	CHECK_FALSE(utf8_to_utf32("\xC5\x41", decoded));
	// a code point written longer than it needs
	CHECK_FALSE(utf8_to_utf32("\xC0\x80", decoded));
	CHECK_FALSE(utf8_to_utf32("\xE0\x80\xAF", decoded));
	// a surrogate, and a code point past the last one
	CHECK_FALSE(utf8_to_utf32("\xED\xA0\x80", decoded));
	CHECK_FALSE(utf8_to_utf32("\xF4\x90\x80\x80", decoded));
	// what was decoded before the bad byte is left in the output
	CHECK_FALSE(utf8_to_utf32("ab\x80", decoded));
	CHECK(decoded == std::u32string{U"ab"});
}

TEST_CASE("utilities: windows-1250 is decoded into code points, the unassigned bytes into zeros") {
	std::u32string decoded{U"old"};
	win1250_to_utf32("A\xB3\xA5\x80\xFF\x81", decoded);
	REQUIRE(decoded.size() == 6);
	CHECK(decoded[0] == U'A');
	CHECK(decoded[1] == char32_t{0x142});
	CHECK(decoded[2] == char32_t{0x104});
	CHECK(decoded[3] == char32_t{0x20AC});
	CHECK(decoded[4] == char32_t{0x2D9});
	CHECK(decoded[5] == char32_t{0});
	win1250_to_utf32("", decoded);
	CHECK(decoded.empty());
}

TEST_CASE("utilities: the value of a key is the text after its equals sign, up to the next space") {
	CHECK(extract_value("Key", "a=1 Key=val b=2") == "val");
	CHECK(extract_value("Key", "Key=val") == "val");
	CHECK(extract_value("Key", "a=1 Key=   val b=2") == "val");
	CHECK(extract_value("Key", "a=1 b=2").empty());
	// the key is a whole word, in its case
	CHECK(extract_value("Key", "MyKey=val key=other").empty());
}

TEST_CASE("utilities: a key without a value takes the next key for it, and is an exception at the end of the text") {
	CHECK(extract_value("Key", "Key= Other=5") == "Other=5");
	CHECK_THROWS_AS(extract_value("Key", "a=1 Key="), std::out_of_range);
	CHECK_THROWS_AS(extract_value("Key", "a=1 Key=  "), std::out_of_range);
}

TEST_CASE("utilities: the value of a key is read into a variable, or the default given when the key isn't there") {
	int number{-1};
	CHECK(extract_value(number, "Key", "a=1 Key=12", ""));
	CHECK(number == 12);
	CHECK_FALSE(extract_value(number, "Other", "a=1 Key=12", "7"));
	CHECK(number == 7);
	// no default leaves the variable as it is
	CHECK_FALSE(extract_value(number, "Other", "a=1 Key=12", ""));
	CHECK(number == 7);
	double real{0.0};
	CHECK(extract_value(real, "Key", "Key=2.5", ""));
	CHECK(real == doctest::Approx(2.5));

	// a flag is on only for a yes
	bool flag{false};
	CHECK(extract_value(flag, "Key", "Key=Yes", ""));
	CHECK(flag);
	CHECK(extract_value(flag, "Key", "Key=1", ""));
	CHECK_FALSE(flag);
	CHECK(extract_value(flag, "Key", "Key=true", ""));
	CHECK_FALSE(flag);
	CHECK_FALSE(extract_value(flag, "Other", "Key=no", "yes"));
	CHECK(flag);
	CHECK_FALSE(extract_value(flag, "Other", "Key=no", ""));
	CHECK(flag);
}

TEST_CASE("utilities: a text has a hash which is the same on every system") {
	// FNV-1a, 32 bits
	CHECK(hash_string("") == 2166136261u);
	CHECK(hash_string("a") == 0xE40C292Cu);
	CHECK(hash_string("foobar") == 0xBF9CF968u);
	CHECK(hash_string("EU07-005") == 1926712284u);
	// the seed gives another one
	CHECK(hash_string("", 5) == 2166136256u);
	CHECK(hash_string("EU07-005", 42) == 1563400698u);
}

TEST_CASE("utilities: the common start of two texts is counted, and a text is looked for in another") {
	CHECK(len_common_prefix("abcdef", "abcxyz") == 3);
	CHECK(len_common_prefix("abc", "abc") == 3);
	CHECK(len_common_prefix("abc", "ab") == 2);
	CHECK(len_common_prefix("", "abc") == 0);
	CHECK(len_common_prefix("xbc", "abc") == 0);
	CHECK(contains("dynamic/pkp/eu07", "pkp"));
	CHECK_FALSE(contains("dynamic/pkp/eu07", "PKP"));
	CHECK(contains("abc", ""));
	CHECK(contains("abc", 'c'));
	CHECK_FALSE(contains("abc", 'd'));
}

// file names ----------------------------------------------------------------------------------------------------------

TEST_CASE("utilities: the extension of a file name is cut off at its last dot") {
	auto const erased{[](std::string Name) {
		auto const removed{erase_extension(Name)};
		return std::pair<bool, std::string>{removed, Name};
	}};
	CHECK(erased("file.mat") == std::pair<bool, std::string>{true, "file"});
	CHECK(erased("a.b.c") == std::pair<bool, std::string>{true, "a.b"});
	CHECK(erased("file.") == std::pair<bool, std::string>{true, "file"});
	CHECK(erased("../file.mat") == std::pair<bool, std::string>{true, "../file"});
	CHECK(erased("noext") == std::pair<bool, std::string>{false, "noext"});
	CHECK(erased("") == std::pair<bool, std::string>{false, ""});
	// the dots of the folder above aren't one
	CHECK(erased("../file") == std::pair<bool, std::string>{false, "../file"});
	CHECK(erased("textures/../x") == std::pair<bool, std::string>{false, "textures/../x"});
	// nor is a dot the name starts with
	CHECK(erased(".hidden") == std::pair<bool, std::string>{false, ".hidden"});
	// a dot in the name of a folder is taken for one
	CHECK(erased("dir.d/file") == std::pair<bool, std::string>{true, "dir"});
}

TEST_CASE("utilities: the slashes of a file name are turned and the leading ones removed") {
	std::string name{"dynamic\\pkp\\eu07.mmd"};
	replace_slashes(name);
	CHECK(name == "dynamic/pkp/eu07.mmd");
	std::string leading{"//textures/a.dds"};
	erase_leading_slashes(leading);
	CHECK(leading == "textures/a.dds");
	std::string plain{"textures/a.dds"};
	erase_leading_slashes(plain);
	CHECK(plain == "textures/a.dds");
	std::string only{"///"};
	erase_leading_slashes(only);
	CHECK(only.empty());
	// only the forward ones
	std::string backward{"\\textures"};
	erase_leading_slashes(backward);
	CHECK(backward == "\\textures");
}

TEST_CASE("utilities: the folder of a file name is the part up to its last slash, the slash included") {
	CHECK(substr_path("dynamic/pkp/eu07.mmd") == "dynamic/pkp/");
	CHECK(substr_path("/file") == "/");
	CHECK(substr_path("file.t3d").empty());
	CHECK(substr_path("").empty());
	// a folder written the Windows way isn't found
	CHECK(substr_path("dynamic\\eu07.mmd").empty());
}

TEST_CASE("utilities: a file is told to be there, and looked for among names and extensions") {
	test::scratch_folder const folder{"utilities_files"};
	folder.write("tex.png", "x");
	folder.write("other.dds", "x");
	auto const base{folder.path()};

	CHECK(FileExists(base + "tex.png"));
	CHECK_FALSE(FileExists(base + "tex.dds"));
	// a folder is there as well
	CHECK(FileExists(base));

	// the first name which has a file with one of the extensions, the extensions tried in their order
	auto const found{FileExists(std::vector<std::string>{base + "missing", base + "tex", base + "other"}, std::vector<std::string>{".dds", ".png"})};
	CHECK(found.first == base + "tex");
	CHECK(found.second == ".png");
	auto const other{FileExists(std::vector<std::string>{base + "other", base + "tex"}, std::vector<std::string>{".png", ".dds"})};
	CHECK(other.first == base + "other");
	CHECK(other.second == ".dds");
	auto const none{FileExists(std::vector<std::string>{base + "missing"}, std::vector<std::string>{".dds", ".png"})};
	CHECK(none.first.empty());
	CHECK(none.second.empty());
}

TEST_CASE("utilities: a file is found by its name in another case") {
	test::scratch_folder const folder{"utilities_cased"};
	auto const name{folder.write("Cased.TXT", "x")};

	// named as it is, it's given back as it was named
	CHECK(FileCased(name) == name);
	// named in another case: the name it has on the disk where the case matters, the name given where it doesn't
	auto const cased{FileCased(folder.path("cased.txt"))};
	REQUIRE_FALSE(cased.empty());
	CHECK(FileExists(cased));
	CHECK(ToLower(std::filesystem::path{cased}.filename().string()) == "cased.txt");
	// no such file in any case
	CHECK(FileCased(folder.path("nothing.txt")).empty());
	CHECK(FileCased("eu07_unit_test_no_such_file.xyz").empty());
}

TEST_CASE("utilities: the time a file was changed is given, zero for a file which isn't there") {
	test::scratch_folder const folder{"utilities_modified"};
	auto const name{folder.write("fresh.txt", "x")};
	auto const modified{last_modified(name)};
	CHECK(modified != 0);
	// written just now
	CHECK(std::abs(std::difftime(std::time(nullptr), modified)) < 24.0 * 3600.0);
	CHECK(last_modified(folder.path("nothing.txt")) == 0);
}

TEST_CASE("utilities: the folders of the user's settings and of the cache are named after the game") {
	// where they are depends on the system and its environment; no folder at all without the variables
	if (auto const config{user_config_path("eu07.ini")}; false == config.empty()) {
		CHECK(config.filename().string() == "eu07.ini");
		CHECK(config.parent_path().filename().string() == "MaSzyna");
	}
	if (auto const cache{user_cache_path("geoportal")}; false == cache.empty()) {
		CHECK(cache.filename().string() == "geoportal");
#if defined(_WIN32)
		CHECK(cache.parent_path().filename().string() == "cache");
		CHECK(cache.parent_path().parent_path().filename().string() == "MaSzyna");
#else
		CHECK(cache.parent_path().filename().string() == "MaSzyna");
#endif
	}
#if defined(_WIN32)
	CHECK(user_config_path("eu07.ini").empty() == (std::getenv("APPDATA") == nullptr));
	CHECK(user_cache_path("x").empty() == (std::getenv("LOCALAPPDATA") == nullptr));
#else
	CHECK(user_config_path("eu07.ini").empty() == (std::getenv("HOME") == nullptr));
#endif
}

// numbers -------------------------------------------------------------------------------------------------------------

TEST_CASE("utilities: the sign of a number, and a number rounded to a whole one") {
	CHECK(sign(-2.5) == -1.0);
	CHECK(sign(0.0) == 0.0);
	CHECK(sign(3) == 1);
	// the other one has no zero
	CHECK(Sign(0.0) == 1.0);
	CHECK(Sign(-0.1) == -1.0);
	CHECK(sq(-3) == 9);
	CHECK(Round(2.4) == 2);
	CHECK(Round(2.5) == 3);
	CHECK(Round(0.0) == 0);
	// the negative ones are rounded up: half is added, then the fraction cut
	CHECK(Round(-0.6) == 0);
	CHECK(Round(-1.5) == -1);
	CHECK(Round(-1.6) == -1);
	CHECK(Round(-2.5) == -2);
}

TEST_CASE("utilities: a value is kept within bounds, given in any order") {
	CHECK(safe_clamp(5, 0, 3) == 3);
	CHECK(safe_clamp(5, 3, 0) == 3);
	CHECK(safe_clamp(-1, 3, 0) == 0);
	CHECK(safe_clamp(2.0, 3.0, 1.0) == 2.0);
	CHECK(safe_clamp(2.0, 2.0, 2.0) == 2.0);
}

TEST_CASE("utilities: a value is wrapped into a range, as an angle into the full circle") {
	CHECK(clamp_circular(370.0) == doctest::Approx(10.0));
	CHECK(clamp_circular(-10.0) == doctest::Approx(350.0));
	CHECK(clamp_circular(360.0) == doctest::Approx(0.0));
	CHECK(clamp_circular(720.0) == doctest::Approx(0.0));
	CHECK(clamp_circular(7, 3) == 1);
	CHECK(clamp_circular(-1, 3) == 2);
	CHECK(clamp_circular(3, 3) == 0);
}

TEST_CASE("utilities: a size is rounded down to a power of two within its limits") {
	CHECK(clamp_power_of_two(100u) == 64u);
	CHECK(clamp_power_of_two(1024u) == 1024u);
	CHECK(clamp_power_of_two(0u) == 1u);
	CHECK(clamp_power_of_two(20000u) == 16384u);
	CHECK(clamp_power_of_two(40000u) == 16384u);
	CHECK(clamp_power_of_two(100u, 128u, 8192u) == 128u);
	CHECK(clamp_power_of_two(10000u, 128u, 8192u) == 8192u);
}

TEST_CASE("utilities: a value is rounded to a step, a whole number down to it") {
	CHECK(quantize(0.12f, 0.05f) == doctest::Approx(0.10));
	CHECK(quantize(1.3, 0.5) == doctest::Approx(1.5));
	CHECK(quantize(-1.3, 0.5) == doctest::Approx(-1.5));
	// the division of whole numbers drops the fraction before the rounding
	CHECK(quantize(7u, 4u) == 4u);
	CHECK(quantize(8u, 4u) == 8u);
}

TEST_CASE("utilities: the lower of two speed limits, -1 standing for none") {
	CHECK(min_speed(30.0, 50.0) == 30.0);
	CHECK(min_speed(-1.0, 50.0) == 50.0);
	CHECK(min_speed(50.0, -1.0) == 50.0);
	CHECK(min_speed(-1.0, -1.0) == -1.0);
	CHECK(min_speed(0.0, 50.0) == 0.0);
}

TEST_CASE("utilities: a value between two others is eased in and out") {
	CHECK(smoothInterpolate(0.0, 10.0, 0.0) == doctest::Approx(0.0));
	CHECK(smoothInterpolate(0.0, 10.0, 0.25) == doctest::Approx(1.5625));
	CHECK(smoothInterpolate(0.0, 10.0, 0.5) == doctest::Approx(5.0));
	CHECK(smoothInterpolate(0.0, 10.0, 1.0) == doctest::Approx(10.0));
	CHECK(smoothInterpolate(10.0, 0.0, 0.25) == doctest::Approx(8.4375));
	// the factor isn't kept between 0 and 1
	CHECK(smoothInterpolate(0.0, 10.0, 2.0) == doctest::Approx(-40.0));
}

TEST_CASE("utilities: two numbers are equal within a margin, exactly with none") {
	CHECK(is_equal(1.0, 1.000001));
	CHECK_FALSE(is_equal(1.0, 1.1));
	CHECK(is_equal(1.0, 1.05, 0.1));
	CHECK(is_equal(1.0, 1.0, 0.0));
	CHECK_FALSE(is_equal(1.0, 1.000001, 0.0));
}

TEST_CASE("utilities: the point of a segment nearest to another point is given as a part of the segment") {
	glm::dvec3 const start{0.0, 0.0, 0.0};
	glm::dvec3 const end{10.0, 0.0, 0.0};
	CHECK(nearest_segment_point(start, end, glm::dvec3{5.0, 3.0, 0.0}) == doctest::Approx(0.5));
	CHECK(nearest_segment_point(start, end, glm::dvec3{2.5, -7.0, 1.0}) == doctest::Approx(0.25));
	// past the ends it's the end
	CHECK(nearest_segment_point(start, end, glm::dvec3{-4.0, 1.0, 0.0}) == 0.0);
	CHECK(nearest_segment_point(start, end, glm::dvec3{14.0, 1.0, 0.0}) == 1.0);
}

TEST_CASE("utilities: points in a line make no triangle, and a set of points has a box around it") {
	glm::dvec3 const a{0.0, 0.0, 0.0};
	glm::dvec3 const b{1.0, 1.0, 1.0};
	CHECK(degenerate(a, b, glm::dvec3{2.0, 2.0, 2.0}));
	CHECK(degenerate(a, b, b));
	CHECK_FALSE(degenerate(a, b, glm::dvec3{1.0, 0.0, 0.0}));

	std::vector<glm::dvec3> points{{1.0, -2.0, 3.0}, {-4.0, 5.0, 0.5}, {0.0, 0.0, 9.0}};
	glm::dvec3 low;
	glm::dvec3 high;
	bounding_box(low, high, std::begin(points), std::end(points));
	CHECK(low.x == -4.0);
	CHECK(low.y == -2.0);
	CHECK(low.z == 0.5);
	CHECK(high.x == 1.0);
	CHECK(high.y == 5.0);
	CHECK(high.z == 9.0);
}
