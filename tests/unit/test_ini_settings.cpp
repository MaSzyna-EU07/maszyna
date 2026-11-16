#include "stdafx.h"
#include "doctest.h"
#include "application/inisettings.h"

#include <string>

// the settings of the menu of the pause written back into eu07.ini: the values of the keys change, nothing else of the file does
namespace
{
using ini_settings::is_utf8;
using ini_settings::value;
using ini_settings::with_values;
} // namespace

TEST_CASE("a value is put in place of the one of its key, the comment kept")
{
	std::string const text{"fieldofview 45\t// kat\r\nsound.volume 1.0\t\t// (1.0) glosnosc\r\nlang pl\r\n"};
	CHECK(with_values(text, {value{"sound.volume", "0.75"}}) == "fieldofview 45\t// kat\r\nsound.volume 0.75\t\t// (1.0) glosnosc\r\nlang pl\r\n");
}

TEST_CASE("a value without a comment, and a key without a value")
{
	CHECK(with_values("gfx.ssr medium\n", {value{"gfx.ssr", "high"}}) == "gfx.ssr high\n");
	CHECK(with_values("gfx.ssr\n", {value{"gfx.ssr", "low"}}) == "gfx.ssr low\n");
	CHECK(with_values("gfx.ssr // odbicia\n", {value{"gfx.ssr", "low"}}) == "gfx.ssr low // odbicia\n");
}

TEST_CASE("a key the text has twice gets the value in both places")
{
	CHECK(with_values("sound.volume 1\nsound.volume 2\n", {value{"sound.volume", "0.5"}}) == "sound.volume 0.5\nsound.volume 0.5\n");
}

TEST_CASE("a key the text hasn't is added before endconfig, or at the end")
{
	CHECK(with_values("lang pl\r\nendconfig\r\nsound.volume 1\r\n", {value{"sound.volume", "0.5"}}) == "lang pl\r\nsound.volume 0.5\r\nendconfig\r\nsound.volume 1\r\n");
	CHECK(with_values("lang pl", {value{"sound.volume", "0.5"}}) == "lang pl\nsound.volume 0.5\n");
	CHECK(with_values("", {value{"sound.volume", "0.5"}}) == "sound.volume 0.5\n");
}

TEST_CASE("keys only as the first word of a line count, commented ones don't")
{
	std::string const text{"// sound.volume 2\nmaxtexturesize sound.volume\n  sound.volume 1\n"};
	CHECK(with_values(text, {value{"sound.volume", "0.5"}}) == "// sound.volume 2\nmaxtexturesize sound.volume\n  sound.volume 0.5\n");
}

TEST_CASE("the bytes of other encodings stay as they are")
{
	std::string const text{"ui.bg.opacity 0.65 // przezroczysto\xB9\xE6\r\nsound.volume 1\r\n"};
	CHECK(with_values(text, {value{"sound.volume", "0.5"}}) == "ui.bg.opacity 0.65 // przezroczysto\xB9\xE6\r\nsound.volume 0.5\r\n");
}

TEST_CASE("UTF-8 told from Windows-1250")
{
	CHECK(is_utf8("zwi\xC4\x99kszenie nastawnika"));
	CHECK(is_utf8("plain ascii"));
	CHECK_FALSE(is_utf8("zwi\xEAkszenie nastawnika"));
	CHECK_FALSE(is_utf8("\xC4"));
}
