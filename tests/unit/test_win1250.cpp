#include "stdafx.h"
#include "doctest.h"
#include "utilities/utilities.h"

// the polish letters, in windows-1250 and in utf-8; written as bytes, the build reads the sources in any code page
namespace
{
std::string const kWin1250{"\xA5\xC6\xCA\xA3\xD1\xD3\x8C\x8F\xAF\xB9\xE6\xEA\xB3\xF1\xF3\x9C\x9F\xBF"};
std::string const kUtf8{"\xC4\x84\xC4\x86\xC4\x98\xC5\x81\xC5\x83\xC3\x93\xC5\x9A\xC5\xB9\xC5\xBB"
                        "\xC4\x85\xC4\x87\xC4\x99\xC5\x82\xC5\x84\xC3\xB3\xC5\x9B\xC5\xBA\xC5\xBC"};
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
