#include "stdafx.h"

#include "doctest.h"

#include "utilities/asciiname.h"

TEST_CASE("ascii name: plain names stay as they are")
{
	CHECK(ascii_name::plain("tor_01-a"));
	CHECK(ascii_name::fold("tor_01-a") == "tor_01-a");
	CHECK(ascii_name::fold("ToR") == "tor");
}

TEST_CASE("ascii name: the Polish letters in UTF-8 lose their marks")
{
	auto const name{std::string{"\xC5\x81\xC4\x85\x6B\x61_\xC5\xBC\xC3\xB3\xC5\x82\x74\xC4\x99_\xC5\x9B\xC4\x87\xC5\x84\xC5\xBA"}}; // Łąka_żółtę_śćńź
	CHECK_FALSE(ascii_name::plain(name));
	CHECK(ascii_name::fold(name) == "laka_zolte_scnz");
	CHECK(ascii_name::readable(name) == name);
}

TEST_CASE("ascii name: the Polish letters in Windows-1250 lose their marks")
{
	// ąćęłńóśźż ĄĆĘŁŃÓŚŹŻ
	auto const name{std::string{"\xB9\xE6\xEA\xB3\xF1\xF3\x9C\x9F\xBF_\xA5\xC6\xCA\xA3\xD1\xD3\x8C\x8F\xAF"}};
	CHECK(ascii_name::fold(name) == "acelnoszz_acelnoszz");
	CHECK(ascii_name::readable(name) == "\xC4\x85\xC4\x87\xC4\x99\xC5\x82\xC5\x84\xC3\xB3\xC5\x9B\xC5\xBA\xC5\xBC_\xC4\x84\xC4\x86\xC4\x98\xC5\x81\xC5\x83\xC3\x93\xC5\x9A\xC5\xB9\xC5\xBB");
}

TEST_CASE("ascii name: other letters with marks, and whatever else isn't ASCII")
{
	CHECK(ascii_name::fold("Stra\xC3\x9F" "e") == "strasse");            // Straße
	CHECK(ascii_name::fold("\xC4\x8C" "esk\xC3\xA1") == "ceska");           // Česká
	CHECK(ascii_name::fold("\xC3\x86" "r\xC3\xB8") == "aero");              // Ærø
	CHECK(ascii_name::fold("tor\xE2\x82\xAC" "1") == "tor_1");              // the euro sign
	CHECK(ascii_name::fold("\xD0\xBF\xD1\x83\xD1\x82\xD1\x8C") == "____"); // путь
	CHECK(ascii_name::fold("x\x81y") == "x_y");                            // a byte the code page doesn't have
	for (int byte = 0x80; byte < 0x100; ++byte)
		CHECK(ascii_name::plain(ascii_name::fold(std::string(1, static_cast<char>(byte)))));
}
