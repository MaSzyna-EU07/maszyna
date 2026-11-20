#include "stdafx.h"
#include "doctest.h"
#include "utilities/utilities.h"

// the polish letters, in windows-1250 and in utf-8; written as bytes, the build reads the sources in any code page
namespace
{
std::string const kWin1250{"\xA5\xC6\xCA\xA3\xD1\xD3\x8C\x8F\xAF\xB9\xE6\xEA\xB3\xF1\xF3\x9C\x9F\xBF"};
std::string const kUtf8{"\xC4\x84\xC4\x86\xC4\x98\xC5\x81\xC5\x83\xC3\x93\xC5\x9A\xC5\xB9\xC5\xBB"
                        "\xC4\x85\xC4\x87\xC4\x99\xC5\x82\xC5\x84\xC3\xB3\xC5\x9B\xC5\xBA\xC5\xBC"};
// the name of a station the way a timetable has it: Biala_Podlaska with its l-stroke, in windows-1250
std::string const kBiala{"Bia\xB3" "a_Podlaska"};
} // namespace

TEST_CASE("win1250: the polish letters of a scenery file read as utf-8") {
	CHECK(win1250_to_utf8("Bia\xB3" "a Podlaska") == "Bia\xC5\x82" "a Podlaska");
	CHECK(win1250_to_utf8(kWin1250) == kUtf8);
	// the rest of the text stays as it is
	CHECK(win1250_to_utf8("tor 1, km 0.178") == "tor 1, km 0.178");
}

TEST_CASE("win1250: what win1250_to_utf8 gives is valid utf-8") {
	std::u32string decoded;
	CHECK_FALSE(utf8_to_utf32(kWin1250, decoded));
	REQUIRE(utf8_to_utf32(win1250_to_utf8(kWin1250), decoded));
	CHECK(decoded.size() == kWin1250.size());
	CHECK(decoded[12] == U'\x0142');
}

TEST_CASE("win1250: utf-8 written back the way a scenery file has it") {
	CHECK(utf8_to_win1250("Bia\xC5\x82" "a Podlaska") == "Bia\xB3" "a Podlaska");
	CHECK(utf8_to_win1250(kUtf8) == kWin1250);
	CHECK(win1250_to_utf8(utf8_to_win1250(kUtf8)) == kUtf8);
	// the letters other than the polish ones turn into question marks
	CHECK(utf8_to_win1250("\xC3\xA9t\xC3\xA9") == "?t?");
}

TEST_CASE("win1250: a text of a data file is turned into utf-8 for the user interface") {
	// plain ascii stays
	CHECK(data_text_to_utf8("tor 1, km 0.178") == "tor 1, km 0.178");
	CHECK(data_text_to_utf8("").empty());
	// windows-1250: the polish letters
	CHECK(data_text_to_utf8(kWin1250) == kUtf8);
	CHECK(data_text_to_utf8(kBiala) == "Bia\xC5\x82" "a_Podlaska");
	// and the letters of the other languages of the code page: a-acute, z-caron, i-acute, u-diaeresis, o-double-acute
	CHECK(data_text_to_utf8("n\xE1" "dra\x9E\xED") == "n\xC3\xA1" "dra\xC5\xBE\xC3\xAD");
	CHECK(data_text_to_utf8("\xFC\xF5") == "\xC3\xBC\xC5\x91");
	// a character outside of the two-byte range (the euro sign), and a byte the code page has nothing for
	CHECK(data_text_to_utf8("5 \x80") == "5 \xE2\x82\xAC");
	CHECK(data_text_to_utf8("a\x81" "b") == "a?b");
	// a text already in utf-8 stays as it is
	CHECK(data_text_to_utf8(kUtf8) == kUtf8);
	CHECK(data_text_to_utf8("Bia\xC5\x82" "a Podlaska") == "Bia\xC5\x82" "a Podlaska");
	CHECK(data_text_to_utf8("\xE2\x94\x82 60 \xE2\x94\x82") == "\xE2\x94\x82 60 \xE2\x94\x82");
	// capital letters with marks in a row make valid utf-8 by chance (O-acute and L-stroke: D3 A3), but of a
	// cyrillic letter; they are read as the windows-1250 they are
	CHECK(data_text_to_utf8("W\xD3\xA3KA") == "W\xC3\x93\xC5\x81KA");
	CHECK(data_text_to_utf8("R\xD3\xAF" "A") == "R\xC3\x93\xC5\xBB" "A");
	// line ends and tabs of a longer text stay
	CHECK(data_text_to_utf8("jedna\n\tdruga \xB3") == "jedna\n\tdruga \xC5\x82");
	// what comes out is valid utf-8
	std::u32string decoded;
	std::string all;
	for (auto value{0x20}; value < 0x100; ++value) {
		all += static_cast<char>(value);
	}
	REQUIRE(utf8_to_utf32(data_text_to_utf8(all), decoded));
	CHECK(decoded.size() == all.size());
}

TEST_CASE("win1250: a name of a data file is shown in one line, with its letters") {
	// what Bezogonkow() did for the user interface, but for the marks: control characters and, when asked,
	// underscores turn into spaces
	CHECK(data_name_to_utf8("a\tb\nc") == "a b c");
	CHECK(data_name_to_utf8(kBiala) == "Bia\xC5\x82" "a_Podlaska");
	CHECK(data_name_to_utf8(kBiala, true) == "Bia\xC5\x82" "a Podlaska");
	CHECK(data_name_to_utf8("Warszawa_Zachodnia", true) == "Warszawa Zachodnia");
	CHECK(data_name_to_utf8("Warszawa_Zachodnia") == "Warszawa_Zachodnia");
	CHECK(data_name_to_utf8("Bia\xC5\x82" "a_Podlaska\r", true) == "Bia\xC5\x82" "a Podlaska ");
	// for a name without letters outside of ascii it gives what Bezogonkow() gave
	for (auto const *name : {"Warszawa_Zachodnia", "Gorna", "tor 1\t2", "", "Os1234"}) {
		CAPTURE(name);
		CHECK(data_name_to_utf8(name) == Bezogonkow(name));
		CHECK(data_name_to_utf8(name, true) == Bezogonkow(name, true));
	}
}

TEST_CASE("win1250: a column of the timetable panel is as wide in letters as it was in bytes") {
	CHECK(utf8_length("") == 0);
	CHECK(utf8_length("Gorna") == 5);
	CHECK(utf8_length("Bia\xC5\x82" "a Podlaska") == 14);
	CHECK(utf8_length(kUtf8) == 18);
	CHECK(utf8_length("\xE2\x94\x82 60 \xE2\x94\x82") == 6);
	// a byte which is no part of a valid sequence counts as a character
	CHECK(utf8_length("a\xB3" "b") == 3);
	CHECK(utf8_length("a\xC5") == 2);

	// filled with spaces
	CHECK(utf8_fit("Gorna", 8) == "Gorna   ");
	CHECK(utf8_fit("", 3) == "   ");
	auto const caption{data_name_to_utf8(kBiala, true)};
	auto const fitted{utf8_fit(caption, 34)};
	CHECK(utf8_length(fitted) == 34);
	CHECK(fitted.size() == 35); // one letter of two bytes
	CHECK(fitted.starts_with(caption));
	// cut between the letters, never inside of one
	CHECK(utf8_fit("Bia\xC5\x82" "a Podlaska", 4) == "Bia\xC5\x82");
	CHECK(utf8_fit("Bia\xC5\x82" "a Podlaska", 3) == "Bia");
	CHECK(utf8_fit(kUtf8, 2) == "\xC4\x84\xC4\x86");
	CHECK(utf8_fit("Gorna", 0).empty());
	// a name of ascii fits the way the panel cut it before
	std::string const longname{"Bardzo_Dluga_Nazwa_Stacji_Wezlowej_Osobowej"};
	CHECK(utf8_fit(longname, 34) == (longname + "                                  ").substr(0, 34));
	CHECK(utf8_fit("Gorna", 34) == (std::string{"Gorna"} + "                                  ").substr(0, 34));
}
