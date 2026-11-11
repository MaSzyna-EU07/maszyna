#include "stdafx.h"
#include "doctest.h"
#include "scratch.h"

#include "scene/scenelayers.h"
#include "utilities/Globals.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"

// the parser of the game's text files (utilities/parser.cpp), as it behaves now: what a token is, what the comments,
// the quotes and the include directives do to the tokens, how the texts turn into numbers, and what the parser tells
// of its place in the file. these record the behaviour for later changes of the parser, its oddities included; the
// ones which look like mistakes are listed in COVERAGE.md

namespace {

char const *const default_break{"\n\r\t ;"};

// the tokens left in the parser, joined with '|'
std::string rest(cParser &Parser, bool const ToLower = true, char const *Break = default_break) {
	std::string joined;
	for (auto token{Parser.getToken<std::string>(ToLower, Break)}; false == token.empty(); token = Parser.getToken<std::string>(ToLower, Break)) {
		if (false == joined.empty()) {
			joined += '|';
		}
		joined += token;
	}
	return joined;
}

// the tokens of a text
std::string tokens(std::string const &Text, bool const ToLower = true, char const *Break = default_break) {
	cParser parser(Text, cParser::buffer_TEXT);
	return rest(parser, ToLower, Break);
}

// puts the settings of the binary terrain back as they were
struct binary_terrain_settings {
	bool state{Global.file_binary_terrain_state};
	int skipped{Global.file_binary_terrain_skipped};

	binary_terrain_settings() = default;
	binary_terrain_settings(binary_terrain_settings const &) = delete;
	binary_terrain_settings &operator=(binary_terrain_settings const &) = delete;
	~binary_terrain_settings() {
		Global.file_binary_terrain_state = state;
		Global.file_binary_terrain_skipped = skipped;
	}
};

} // namespace

// tokens --------------------------------------------------------------------------------------------------------------

TEST_CASE("parser: tokens end at spaces, tabs, line ends and semicolons, and come in lower case unless asked otherwise") {
	std::string const text{"Alpha\tBETA;gamma\r\nDelta  ;; epsilon"};
	CHECK(tokens(text) == "alpha|beta|gamma|delta|epsilon");
	CHECK(tokens(text, false) == "Alpha|BETA|gamma|Delta|epsilon");
	// commas, colons and brackets are a part of the token for the default separators
	CHECK(tokens("pos: [1,2] end") == "pos:|[1,2]|end");
	// only the letters of ascii are changed, the bytes of utf-8 and windows-1250 stay
	CHECK(tokens("\xC5\x81\xC3\x93\x44\xC5\xB9 \xA3\xD3\x44\x8F") == "\xC5\x81\xC3\x93\x64\xC5\xB9|\xA3\xD3\x64\x8F");
}

TEST_CASE("parser: every token can be read with separators of its own") {
	cParser parser("a,b c;d", cParser::buffer_TEXT);
	CHECK(parser.getToken<std::string>(true, ",") == "a");
	CHECK(parser.getToken<std::string>(true, " ") == "b");
	CHECK(parser.getToken<std::string>() == "c");
	CHECK(parser.getToken<std::string>() == "d");
	CHECK(parser.getToken<std::string>().empty());
}

TEST_CASE("parser: with no separators the rest of the text is one token") {
	// no separators given at all is taken as none
	cParser parser("one two\nthree", cParser::buffer_TEXT);
	CHECK(parser.getToken<std::string>(false, nullptr) == "one two\nthree");
	CHECK(parser.Line() == 2);
	CHECK(tokens("one two\nthree", true, "") == "one two\nthree");
}

TEST_CASE("parser: the end of the text shows as eof, and a read past it as a failure, the way a stream does it") {
	SUBCASE("the text ends with a token") {
		cParser parser("a b", cParser::buffer_TEXT);
		CHECK(parser.ok());
		CHECK_FALSE(parser.eof());
		CHECK(parser.getToken<std::string>() == "a");
		CHECK_FALSE(parser.eof());
		// the last token is ended by the end of the text
		CHECK(parser.getToken<std::string>() == "b");
		CHECK(parser.eof());
		CHECK(parser.ok());
		CHECK(parser.getToken<std::string>().empty());
		CHECK_FALSE(parser.ok());
	}
	SUBCASE("the text ends with a separator") {
		cParser parser("a\n", cParser::buffer_TEXT);
		CHECK(parser.getToken<std::string>() == "a");
		CHECK_FALSE(parser.eof());
		CHECK(parser.getToken<std::string>().empty());
		CHECK(parser.eof());
		CHECK(parser.ok());
		CHECK(parser.getToken<std::string>().empty());
		CHECK_FALSE(parser.ok());
	}
	SUBCASE("an empty text fails with its first read") {
		// the check for a byte order mark looks for more text after the empty first token
		cParser parser("", cParser::buffer_TEXT);
		CHECK(parser.ok());
		CHECK(parser.getToken<std::string>().empty());
		CHECK(parser.eof());
		CHECK_FALSE(parser.ok());
	}
}

TEST_CASE("parser: a byte order mark at the start of the text isn't a part of the first token") {
	cParser parser("\xEF\xBB\xBF" "first second", cParser::buffer_TEXT);
	CHECK(parser.getToken<std::string>() == "first");
	// the place of the token in the text is the one after the mark
	CHECK(parser.TokenBegin() == 3);
	CHECK(parser.TokenEnd() == 8);
	CHECK(parser.getToken<std::string>() == "second");
	// a mark alone on its line gives no token
	CHECK(tokens("\xEF\xBB\xBF\nfirst") == "first");
	// it's a mark only at the start
	CHECK(tokens("a \xEF\xBB\xBF" "b") == "a|\xEF\xBB\xBF" "b");
}

// comments ------------------------------------------------------------------------------------------------------------

TEST_CASE("parser: comments are left out, to the end of the line or of the block") {
	cParser parser("one // rest of the line\ntwo /* a\nblock */ three", cParser::buffer_TEXT);
	CHECK(rest(parser) == "one|two|three");
	CHECK(parser.Line() == 3);
	// a comment ends the token it touches, and the text after it is a token of its own
	CHECK(tokens("abc//x\ndef") == "abc|def");
	CHECK(tokens("ab/*x*/cd") == "ab|cd");
	// a slash or a star alone is a character like any other
	CHECK(tokens("1/2 a*b / *") == "1/2|a*b|/|*");
}

TEST_CASE("parser: the comments stay in the tokens when the parser is told not to skip them") {
	cParser parser("a // b /* c */", cParser::buffer_TEXT);
	parser.skipComments = false;
	CHECK(rest(parser) == "a|//|b|/*|c|*/");
}

TEST_CASE("parser: a comment the text ends in leaves the parser failed, not just at the end") {
	for (auto const *text : {"a /* never closed", "a // the last line"}) {
		CAPTURE(text);
		cParser parser(text, cParser::buffer_TEXT);
		CHECK(parser.getToken<std::string>() == "a");
		CHECK(parser.ok());
		CHECK(parser.getToken<std::string>().empty());
		CHECK(parser.eof());
		CHECK_FALSE(parser.ok());
	}
	// with a line end after it, the parser is at the end like after a token
	cParser parser("a // the last line\n", cParser::buffer_TEXT);
	CHECK(parser.getToken<std::string>() == "a");
	CHECK(parser.getToken<std::string>().empty());
	CHECK(parser.eof());
	CHECK(parser.ok());
}

TEST_CASE("parser: other comment marks can be added") {
	CHECK(tokens("a # note\nb -- c\nd") == "a|#|note|b|--|c|d");
	cParser parser("a # note\nb -- c\nd", cParser::buffer_TEXT);
	// also after the parser read something
	CHECK(parser.getToken<std::string>() == "a");
	parser.addCommentStyle("#", "\n");
	parser.addCommentStyle("--", "\n");
	CHECK(rest(parser) == "b|d");

	SUBCASE("a comment without an end mark takes the rest of the text") {
		cParser ending("keep these stop and nothing else", cParser::buffer_TEXT);
		ending.addCommentStyle("stop", "");
		CHECK(rest(ending) == "keep|these");
	}
	SUBCASE("a mark of letters is compared with the token as it's read, in lower case or not") {
		cParser lower("a REM x\nb", cParser::buffer_TEXT);
		lower.addCommentStyle("rem", "\n");
		CHECK(rest(lower) == "a|b");
		cParser cased("a REM x\nb", cParser::buffer_TEXT);
		cased.addCommentStyle("rem", "\n");
		CHECK(rest(cased, false) == "a|REM|x|b");
	}
}

// quotes --------------------------------------------------------------------------------------------------------------

TEST_CASE("parser: quotes make one token of the text between them, with its case and separators kept") {
	CHECK(tokens("Name \"Two Words\" Tail") == "name|Two Words|tail");
	CHECK(tokens("\"a;b\tc\" x") == "a;b\tc|x");
	// the marks of a comment too
	CHECK(tokens("\"http://host/path\" x") == "http://host/path|x");
	// the quoted text is glued to what it touches
	CHECK(tokens("Pre\"In Side\"Post next") == "preIn Sidepost|next");
	// a backslash takes the next character as it is
	CHECK(tokens("\"say \\\"hi\\\" \\\\ done\" x") == "say \"hi\" \\ done|x");
}

TEST_CASE("parser: empty quotes give no token") {
	CHECK(tokens("\"\" after") == "after");
	CHECK(tokens("x \"\" y") == "x|y");
}

TEST_CASE("parser: quotes left open take the rest of the text, and the parser fails") {
	cParser parser("\"open ended", cParser::buffer_TEXT);
	CHECK(parser.getToken<std::string>() == "open ended");
	CHECK(parser.eof());
	CHECK_FALSE(parser.ok());
}

// place in the text ---------------------------------------------------------------------------------------------------

TEST_CASE("parser: the line is the one the reading got to, counted from 1") {
	cParser parser("one\ntwo\n\n\nthree // note\nfour \"a\nb\" /* x\ny */ five", cParser::buffer_TEXT);
	CHECK(parser.Line() == 1);
	// the line end which ended the token is read already
	CHECK(parser.getToken<std::string>() == "one");
	CHECK(parser.Line() == 2);
	CHECK(parser.getToken<std::string>() == "two");
	CHECK(parser.Line() == 3);
	CHECK(parser.getToken<std::string>() == "three");
	CHECK(parser.Line() == 5);
	CHECK(parser.getToken<std::string>() == "four");
	CHECK(parser.Line() == 6);
	// the lines inside of quotes and comments count too
	CHECK(parser.getToken<std::string>() == "a\nb");
	CHECK(parser.Line() == 7);
	CHECK(parser.getToken<std::string>() == "five");
	CHECK(parser.Line() == 8);
	CHECK(parser.LineMain() == 8);
}

TEST_CASE("parser: a line ended the Windows way is counted at its line feed") {
	cParser parser("one\r\ntwo", cParser::buffer_TEXT);
	CHECK(parser.getToken<std::string>() == "one");
	// only the carriage return is read yet
	CHECK(parser.Line() == 1);
	CHECK(parser.getToken<std::string>() == "two");
	CHECK(parser.Line() == 2);
}

TEST_CASE("parser: the place of the token last read is given as offsets into the text") {
	cParser parser("ab  cd", cParser::buffer_TEXT);
	CHECK(parser.getToken<std::string>() == "ab");
	CHECK(parser.TokenBegin() == 0);
	CHECK(parser.TokenEnd() == 2);
	CHECK(parser.getToken<std::string>() == "cd");
	CHECK(parser.TokenBegin() == 4);
	CHECK(parser.TokenEnd() == 6);

	SUBCASE("the quotes are a part of it") {
		cParser quoted("\"x y\" z", cParser::buffer_TEXT);
		CHECK(quoted.getToken<std::string>() == "x y");
		CHECK(quoted.TokenBegin() == 0);
		CHECK(quoted.TokenEnd() == 5);
	}
	SUBCASE("and so is a comment touching the token") {
		cParser commented("ab//c\nd", cParser::buffer_TEXT);
		CHECK(commented.getToken<std::string>() == "ab");
		CHECK(commented.TokenBegin() == 0);
		CHECK(commented.TokenEnd() == 6);
	}
}

TEST_CASE("parser: the progress is the part of the text read, in percent") {
	cParser parser("aaaa bbbb ", cParser::buffer_TEXT);
	CHECK(parser.getProgress() == 0);
	CHECK(parser.getToken<std::string>() == "aaaa");
	CHECK(parser.getProgress() == 50);
	CHECK(parser.getFullProgress() == 50);
	// a text put in front of the rest takes its share of what is left
	parser.injectString("cc dd eeee");
	CHECK(parser.getFullProgress() == 50);
	CHECK(parser.getToken<std::string>() == "cc");
	CHECK(parser.getProgress() == 50);
	CHECK(parser.getFullProgress() == 65);
	// nothing to read is all read
	CHECK(cParser("", cParser::buffer_TEXT).getProgress() == 100);
	CHECK(cParser("", cParser::buffer_TEXT).getFullProgress() == 100);
}

TEST_CASE("parser: a text made by the program is named by the folder given with it, or not at all") {
	CHECK(cParser("a", cParser::buffer_TEXT).Name().empty());
	CHECK(cParser("a", cParser::buffer_TEXT, "scenery/").Name() == "scenery/");
	CHECK_FALSE(cParser("a", cParser::buffer_TEXT).InLayerFile());
}

// values --------------------------------------------------------------------------------------------------------------

TEST_CASE("parser: several tokens are read at once and taken one by one into variables of their types") {
	cParser parser("1 2.5 -3 Word yes rest", cParser::buffer_TEXT);
	REQUIRE(parser.getTokens(5));
	CHECK(parser.peek() == "1");
	int whole{0};
	double real{0.0};
	float single{0.f};
	std::string word;
	bool flag{false};
	parser >> whole >> real >> single >> word >> flag;
	CHECK(whole == 1);
	CHECK(real == doctest::Approx(2.5));
	CHECK(single == doctest::Approx(-3.0));
	CHECK(word == "word");
	CHECK(flag);
	CHECK(parser.peek().empty());
	// the tokens not asked for are still to be read
	CHECK(parser.getToken<std::string>() == "rest");
}

TEST_CASE("parser: too few tokens are told, and the variables left without one keep their values") {
	cParser parser("a b", cParser::buffer_TEXT);
	CHECK_FALSE(parser.getTokens(3));
	std::string first;
	std::string second;
	std::string third{"untouched"};
	parser >> first >> second >> third;
	CHECK(first == "a");
	CHECK(second == "b");
	CHECK(third == "untouched");
	// the same for the other types
	int whole{7};
	bool flag{true};
	parser >> whole >> flag;
	CHECK(whole == 7);
	CHECK(flag);
}

TEST_CASE("parser: the tokens are read in lower case by default, and as written when asked") {
	cParser parser("Alpha Beta", cParser::buffer_TEXT);
	REQUIRE(parser.getTokens(2, false));
	std::string first;
	std::string second;
	parser >> first >> second;
	CHECK(first == "Alpha");
	CHECK(second == "Beta");
}

TEST_CASE("parser: whole numbers are read up to what isn't a part of one") {
	cParser parser("42 -7 3.9 12abc", cParser::buffer_TEXT);
	CHECK(parser.getToken<int>() == 42);
	CHECK(parser.getToken<int>() == -7);
	CHECK(parser.getToken<int>() == 3);
	CHECK(parser.getToken<int>() == 12);
	// nothing left to read is a zero
	CHECK(parser.getToken<int>() == 0);
}

TEST_CASE("parser: numbers with a fraction take a point, an exponent and a sign, whatever the system's language") {
	cParser parser("0.25 -1.5e3 .5 +2.5 7. 1,5", cParser::buffer_TEXT);
	CHECK(parser.getToken<double>() == doctest::Approx(0.25));
	CHECK(parser.getToken<double>() == doctest::Approx(-1500.0));
	CHECK(parser.getToken<double>() == doctest::Approx(0.5));
	CHECK(parser.getToken<double>() == doctest::Approx(2.5));
	CHECK(parser.getToken<double>() == doctest::Approx(7.0));
	// a comma isn't a decimal point: the number ends at it
	CHECK(parser.getToken<double>() == doctest::Approx(1.0));
	CHECK(cParser("1e-3", cParser::buffer_TEXT).getToken<float>() == doctest::Approx(0.001));
	// a word is a zero
	CHECK(cParser("word", cParser::buffer_TEXT).getToken<double>() == 0.0);
}

TEST_CASE("parser: true, yes and 1 are true, anything else is false") {
	cParser parser("Yes TRUE 1 no 0 maybe", cParser::buffer_TEXT);
	// read as a flag the token is put in lower case, whatever the caller asks for
	CHECK(parser.getToken<bool>(false));
	CHECK(parser.getToken<bool>(false));
	CHECK(parser.getToken<bool>());
	CHECK_FALSE(parser.getToken<bool>());
	CHECK_FALSE(parser.getToken<bool>());
	CHECK_FALSE(parser.getToken<bool>());
	// nothing left to read is false
	CHECK_FALSE(parser.getToken<bool>());

	// a token read before with its case kept is compared as it is
	cParser cased("Yes yes", cParser::buffer_TEXT);
	REQUIRE(cased.getTokens(2, false));
	bool first{true};
	bool second{false};
	cased >> first >> second;
	CHECK_FALSE(first);
	CHECK(second);
}

TEST_CASE("parser: three numbers are read as a vector, with commas and brackets as separators") {
	cParser parser("[ 1, 2.5 ; -3 ] next", cParser::buffer_TEXT);
	auto const vector{parser.getToken<glm::vec3>()};
	CHECK(vector.x == doctest::Approx(1.0));
	CHECK(vector.y == doctest::Approx(2.5));
	CHECK(vector.z == doctest::Approx(-3.0));
	// the closing bracket is left for the next read
	CHECK(rest(parser) == "]|next");
}

TEST_CASE("parser: a token can be skipped, or compared with the one expected") {
	cParser parser("alpha Beta gamma", cParser::buffer_TEXT);
	CHECK(parser.expectToken("alpha"));
	parser.ignoreToken();
	CHECK(parser.expectToken("gamma"));
	CHECK_FALSE(parser.expectToken("gamma"));
	// the token is compared in lower case
	CHECK_FALSE(cParser("Beta", cParser::buffer_TEXT).expectToken("Beta"));
	CHECK(cParser("Beta", cParser::buffer_TEXT).expectToken("beta"));
}

// tokens waiting ------------------------------------------------------------------------------------------------------

TEST_CASE("parser: the tokens read and not taken are dropped by the next read") {
	cParser parser("a b c d e", cParser::buffer_TEXT);
	CHECK(parser.autoclear());
	REQUIRE(parser.getTokens(2));
	CHECK(parser.peek() == "a");
	CHECK(parser.getToken<std::string>() == "c");
	CHECK(parser.peek().empty());
}

TEST_CASE("parser: told to keep them, the next reads take the waiting tokens first") {
	cParser parser("a b c d e", cParser::buffer_TEXT);
	CHECK_FALSE(parser.autoclear(false).autoclear());
	REQUIRE(parser.getTokens(2));
	CHECK(parser.getToken<std::string>() == "a");
	CHECK(parser.peek() == "b");
	// one waits, so one more is read
	REQUIRE(parser.getTokens(2));
	std::string first;
	std::string second;
	parser >> first >> second;
	CHECK(first == "b");
	CHECK(second == "c");
	CHECK(parser.getToken<std::string>() == "d");
}

TEST_CASE("parser: many waiting tokens are taken in their order") {
	// enough of them for the queue to drop the ones taken on the way
	std::string text;
	for (auto number{0}; number < 200; ++number) {
		text += std::to_string(number) + ' ';
	}
	cParser parser(text, cParser::buffer_TEXT);
	parser.autoclear(false);
	REQUIRE(parser.getTokens(200));
	auto wrong{0};
	for (auto number{0}; number < 200; ++number) {
		auto value{-1};
		parser >> value;
		if (value != number) {
			++wrong;
		}
	}
	CHECK(wrong == 0);
	CHECK(parser.peek().empty());
}

// parameters ----------------------------------------------------------------------------------------------------------

TEST_CASE("parser: (pN) in a token is replaced with the N-th parameter, or with none when there's no such one") {
	std::string const text{"(p1) x(p2)y (p3) (P1) (p0) (p1)-(p2) (p1"};
	std::vector<std::string> const parameters{"First", "Second"};
	// the parameter is put in lower case with the token
	cParser lower(text, cParser::buffer_TEXT, "", true, parameters);
	CHECK(rest(lower) == "first|xsecondy|none|first|none|first-second|(p1");
	// with the case kept, a capital P isn't a parameter
	cParser cased(text, cParser::buffer_TEXT, "", true, parameters);
	CHECK(rest(cased, false) == "First|xSecondy|none|(P1)|none|First-Second|(p1");
	// without parameters nothing is replaced
	CHECK(tokens("(p1) (p3)") == "(p1)|(p3)");
}

// text put in front ---------------------------------------------------------------------------------------------------

TEST_CASE("parser: a text given to the parser is read before the rest, the one given last first") {
	cParser parser("m1 m2", cParser::buffer_TEXT);
	parser.injectString("a b");
	parser.injectString("x y");
	// what to do with the tokens read and not taken is told to the texts given, too
	parser.autoclear(false);
	CHECK(rest(parser) == "x|y|a|b|m1|m2");
}

TEST_CASE("parser: the text after the token last read can be taken as it is and skipped") {
	cParser parser("one two three four", cParser::buffer_TEXT);
	// the first token is left to the parser
	CHECK(parser.remainingText().empty());
	CHECK(parser.getToken<std::string>() == "one");
	CHECK(std::string{parser.remainingText()} == "two three four");
	parser.skipText(4, 0, 3);
	CHECK(std::string{parser.remainingText()} == "three four");
	CHECK(parser.TokenBegin() == 5);
	CHECK(parser.TokenEnd() == 8);
	CHECK(parser.Line() == 1);
	CHECK(parser.getToken<std::string>() == "three");
	// the lines skipped are counted as the caller says, and the end of the text isn't passed
	parser.skipText(1000, 2, 4);
	CHECK(parser.Line() == 3);
	CHECK(parser.TokenBegin() == 14);
	CHECK(parser.TokenEnd() == 18);
	CHECK(parser.remainingText().empty());
	CHECK(parser.getToken<std::string>().empty());
}

TEST_CASE("parser: the text isn't given when the parser would change its tokens") {
	SUBCASE("tokens wait to be taken") {
		cParser parser("one two three", cParser::buffer_TEXT);
		REQUIRE(parser.getTokens(2));
		CHECK(parser.remainingText().empty());
	}
	SUBCASE("there are parameters to put in") {
		cParser parser("one two three", cParser::buffer_TEXT, "", true, {"x"});
		CHECK(parser.getToken<std::string>() == "one");
		CHECK(parser.remainingText().empty());
	}
	SUBCASE("the comments aren't skipped") {
		cParser parser("one two three", cParser::buffer_TEXT);
		parser.skipComments = false;
		CHECK(parser.getToken<std::string>() == "one");
		CHECK(parser.remainingText().empty());
	}
	SUBCASE("there are other comment marks") {
		cParser parser("one two three", cParser::buffer_TEXT);
		parser.addCommentStyle("#", "\n");
		CHECK(parser.getToken<std::string>() == "one");
		CHECK(parser.remainingText().empty());
	}
}

TEST_CASE("parser: the text after the token is the one of the text being read, a text put in front included") {
	cParser parser("m1 m2", cParser::buffer_TEXT);
	CHECK(parser.getToken<std::string>() == "m1");
	CHECK(std::string{parser.remainingText()} == "m2");
	parser.injectString("x y z");
	// its first token is left to the parser, too
	CHECK(parser.remainingText().empty());
	CHECK(parser.getToken<std::string>() == "x");
	CHECK(std::string{parser.remainingText()} == "y z");
	parser.skipText(2, 0, 1);
	CHECK(rest(parser) == "z|m2");
}

// files ---------------------------------------------------------------------------------------------------------------

TEST_CASE("parser: a file is read from the folder given, whatever its line ends") {
	test::scratch_folder const folder{"parser_file"};
	folder.write("plain.txt", "Alpha beta\ngamma");
	folder.write("windows.txt", "a 1\r\nb 2\r\n");
	folder.write("marked.txt", "\xEF\xBB\xBF" "first second");

	cParser parser("plain.txt", cParser::buffer_FILE, folder.path());
	CHECK(parser.ok());
	CHECK(parser.Line() == 1);
	CHECK(parser.Name() == folder.path() + "plain.txt");
	CHECK(parser.InLayerFile());
	CHECK(rest(parser) == "alpha|beta|gamma");

	cParser windows("windows.txt", cParser::buffer_FILE, folder.path());
	CHECK(rest(windows) == "a|1|b|2");
	cParser marked("marked.txt", cParser::buffer_FILE, folder.path());
	CHECK(rest(marked) == "first|second");
	// the folder can be a part of the name as well
	cParser whole(folder.path("plain.txt"), cParser::buffer_FILE);
	CHECK(whole.Name() == folder.path("plain.txt"));
	CHECK(rest(whole) == "alpha|beta|gamma");
}

TEST_CASE("parser: a file named in another case than on the disk is found") {
	test::scratch_folder const folder{"parser_case"};
	folder.write("Mixed.TXT", "found");
	cParser parser("mixed.txt", cParser::buffer_FILE, folder.path());
	CHECK(parser.ok());
	CHECK(rest(parser) == "found");
}

TEST_CASE("parser: a file which isn't there gives a failed parser with nothing to read") {
	test::scratch_folder const folder{"parser_missing"};
	cParser parser("nothing.txt", cParser::buffer_FILE, folder.path());
	CHECK_FALSE(parser.ok());
	// no line to be at
	CHECK(parser.Line() == 0);
	CHECK(parser.getProgress() == 100);
	CHECK(parser.Name() == folder.path() + "nothing.txt");
	CHECK(parser.getToken<std::string>().empty());
}

TEST_CASE("parser: an empty file is read, with nothing in it") {
	test::scratch_folder const folder{"parser_empty"};
	folder.write("empty.txt", "");
	cParser parser("empty.txt", cParser::buffer_FILE, folder.path());
	CHECK(parser.ok());
	CHECK(parser.Line() == 1);
	CHECK(parser.getProgress() == 100);
	CHECK(parser.getToken<std::string>().empty());
}

TEST_CASE("parser: a folder given for a file has nothing to read") {
	test::scratch_folder const folder{"parser_folder"};
	std::filesystem::create_directories(folder.root() / "inner.txt");
	// whether it opens at all depends on the system, so the state of the parser does too
	cParser parser("inner.txt", cParser::buffer_FILE, folder.path());
	CHECK(parser.getToken<std::string>().empty());
}

TEST_CASE("parser: the tokens of a file are counted, the ones of the files it includes too") {
	test::scratch_folder const folder{"parser_count"};
	folder.write("count.txt", "a b\nc // d e\n\"f g\" include extra.txt end\n");
	folder.write("extra.txt", "x y");
	folder.write("empty.txt", "");
	CHECK(cParser::countTokens("count.txt", folder.path()) == 6);
	CHECK(cParser::countTokens("empty.txt", folder.path()) == 0);
	CHECK(cParser::countTokens("nothing.txt", folder.path()) == 0);
}

// includes ------------------------------------------------------------------------------------------------------------

TEST_CASE("parser: include puts the tokens of another file in place of the directive, with its parameters put in") {
	test::scratch_folder const folder{"parser_include"};
	folder.write("part.inc", "Body (p1) (p2) (p3)\n");
	std::string const text{"start include PART.inc One Two end finish"};

	cParser parser(text, cParser::buffer_TEXT, folder.path());
	CHECK(parser.getToken<std::string>() == "start");
	CHECK(parser.LineMain() == 1);
	CHECK(parser.getToken<std::string>() == "body");
	// the parser tells of the file it reads now
	CHECK(parser.Name() == folder.path() + "part.inc");
	CHECK(parser.Line() == 1);
	CHECK(parser.LineMain() == -1);
	CHECK_FALSE(parser.InLayerFile());
	// the parameters are read as written, and put in lower case with the tokens they go into
	CHECK(rest(parser) == "one|two|none|finish");
	CHECK(parser.Name() == folder.path());
	CHECK(parser.LineMain() == 1);

	// with the case kept, the name of the file is as written, and found in its case on the disk
	cParser cased(text, cParser::buffer_TEXT, folder.path());
	CHECK(rest(cased, false) == "start|Body|One|Two|none|finish");
	// a file included without parameters keeps its placeholders
	cParser bare("include part.inc end", cParser::buffer_TEXT, folder.path());
	CHECK(rest(bare) == "body|(p1)|(p2)|(p3)");
}

TEST_CASE("parser: an included file includes others, named from the folder of the first file, not from its own") {
	test::scratch_folder const folder{"parser_nested"};
	folder.write("outer.inc", "o1 include sub/inner.inc (p1) end o2");
	folder.write("sub/inner.inc", "i1 (p1)");
	folder.write("sub/a.inc", "a include b.inc end");
	folder.write("sub/b.inc", "beside");
	folder.write("b.inc", "root");

	// a parameter can be passed on
	cParser parser("m1 include outer.inc X end m2", cParser::buffer_TEXT, folder.path());
	CHECK(rest(parser) == "m1|o1|i1|x|o2|m2");
	cParser relative("include sub/a.inc end", cParser::buffer_TEXT, folder.path());
	CHECK(rest(relative) == "a|root");
	// the folders can be written the Windows way
	cParser backslash("include sub\\inner.inc Y end", cParser::buffer_TEXT, folder.path());
	CHECK(rest(backslash) == "i1|y");
}

TEST_CASE("parser: an include of a file which isn't there, or is empty, is passed over with its parameters") {
	test::scratch_folder const folder{"parser_bad_include"};
	folder.write("empty.inc", "");
	cParser missing("a include nothing.inc 1 2 end b", cParser::buffer_TEXT, folder.path());
	CHECK(rest(missing) == "a|b");
	cParser empty("a include empty.inc 1 2 end b", cParser::buffer_TEXT, folder.path());
	CHECK(rest(empty) == "a|b");
}

TEST_CASE("parser: include is a directive only in lower case and when the parser expands them") {
	test::scratch_folder const folder{"parser_no_include"};
	folder.write("part.inc", "body");
	// read with its case kept, another spelling is a token like any other
	cParser cased("Include part.inc end", cParser::buffer_TEXT, folder.path());
	CHECK(rest(cased, false) == "Include|part.inc|end");
	cParser lower("INCLUDE part.inc end", cParser::buffer_TEXT, folder.path());
	CHECK(rest(lower) == "body");
	cParser plain("include part.inc end", cParser::buffer_TEXT, folder.path());
	plain.expandIncludes = false;
	CHECK(rest(plain) == "include|part.inc|end");
}

TEST_CASE("parser: read line by line, a line starting with include is the directive") {
	test::scratch_folder const folder{"parser_line_include"};
	folder.write("macro.txt", "first (p1)\nsecond (p2)\n");
	folder.write("main.txt", "head\ninclude macro.txt 5 6 end\ntail\n");

	cParser parser("main.txt", cParser::buffer_FILE, folder.path());
	CHECK(rest(parser, false, "\n\r") == "head|first 5|second 6|tail");
	// also when the parser is told not to expand the directives
	cParser plain("main.txt", cParser::buffer_FILE, folder.path());
	plain.expandIncludes = false;
	CHECK(rest(plain, false, "\n\r") == "head|first 5|second 6|tail");
	// and any line starting with these letters is taken for one: this one for an include of the file "d"
	cParser word("one\nincluded in price\ntwo", cParser::buffer_TEXT, folder.path());
	CHECK(rest(word, false, "\n\r") == "one|two");
}

TEST_CASE("parser: an include can name several files to pick one from at random") {
	test::scratch_folder const folder{"parser_random_include"};
	folder.write("only.inc", "chosen");
	folder.write("one.inc", "first");
	folder.write("two.inc", "second");
	std::vector<std::string> const none;

	cParser single("include [ only.inc ] end tail", cParser::buffer_TEXT, folder.path(), true, none, true);
	CHECK(rest(single) == "chosen|tail");
	// a single name is still a single file
	cParser plain("include only.inc end tail", cParser::buffer_TEXT, folder.path(), true, none, true);
	CHECK(rest(plain) == "chosen|tail");
	cParser either("include [ one.inc two.inc ] end tail", cParser::buffer_TEXT, folder.path(), true, none, true);
	auto const picked{either.getToken<std::string>()};
	CHECK((picked == "first" || picked == "second"));
	CHECK(rest(either) == "tail");
	// a parser not told to takes the bracket for the name of the file
	cParser literal("include [ only.inc ] end tail", cParser::buffer_TEXT, folder.path());
	CHECK(rest(literal) == "tail");
}

TEST_CASE("parser: with the traction left out, the includes of the files in a tr or tra folder are passed over") {
	// no files needed: the directives are skipped up to their end
	cParser parser("before include tr/wires.scm 1 2 end after include other/tra/x.scm end last", cParser::buffer_TEXT, "", false);
	CHECK(rest(parser) == "before|after|last");
	// the folder is told by the letters before the slash, so this one is taken for traction too
	cParser centre("include centr/dworzec.scm end x", cParser::buffer_TEXT, "", false);
	CHECK(rest(centre) == "x");

	// the other files are read
	test::scratch_folder const folder{"parser_traction"};
	folder.write("tory.scm", "track");
	folder.write("tr/siec.scm", "wires");
	cParser without("include tory.scm end include tr/siec.scm end x", cParser::buffer_TEXT, folder.path(), false);
	CHECK(rest(without) == "track|x");
	cParser with("include tory.scm end include tr/siec.scm end x", cParser::buffer_TEXT, folder.path(), true);
	CHECK(rest(with) == "track|wires|x");
}

TEST_CASE("parser: read line by line, a traction include passed over takes the lines up to the next end with it") {
	// the directive is skipped in the text of the parser, though its end was on the line of the directive
	cParser parser("include tr/wires.scm end\nfirst line\nsecond end\nthird line", cParser::buffer_TEXT, "", false);
	CHECK(rest(parser, false, "\n\r") == "third line");
}

TEST_CASE("parser: the include of a terrain file is passed over while the binary terrain is in use") {
	test::scratch_folder const folder{"parser_terrain"};
	folder.write("map_ter.scm", "terrain");
	std::string const text{"a include map_ter.scm 1 2 end b"};
	binary_terrain_settings restored;

	Global.file_binary_terrain_state = true;
	Global.file_binary_terrain_skipped = 0;
	cParser skipping(text, cParser::buffer_TEXT, folder.path());
	CHECK(rest(skipping) == "a|b");
	CHECK(Global.file_binary_terrain_skipped == 1);

	Global.file_binary_terrain_state = false;
	cParser reading(text, cParser::buffer_TEXT, folder.path());
	CHECK(rest(reading) == "a|terrain|b");
	CHECK(Global.file_binary_terrain_skipped == 1);
}

TEST_CASE("parser: for a scenery opened for editing the included files are told to the scenery layers") {
	// the layers themselves are stubs in the tests (engine_stubs.cpp): the templates used are all they keep
	test::scratch_folder const folder{"parser_layers"};
	folder.write("main.scm", "include szablon_testowy.inc end include other.scm 7 end tail");
	folder.write("szablon_testowy.inc", "p");
	folder.write("other.scm", "o (p1)");

	cParser parser("main.scm", cParser::buffer_FILE, folder.path());
	parser.sceneryLayers = true;
	CHECK(parser.InLayerFile());
	// a template isn't a layer file, a scenery file is
	CHECK(parser.getToken<std::string>() == "p");
	CHECK_FALSE(parser.InLayerFile());
	CHECK(scene::Layers.templates().count("szablon_testowy.inc") == 1);
	CHECK(parser.getToken<std::string>() == "o");
	CHECK(parser.InLayerFile());
	CHECK(rest(parser) == "7|tail");
	CHECK(parser.InLayerFile());
}

// what the helpers of utilities.h read with the parser -----------------------------------------------------------------

TEST_CASE("parser: a point is three numbers") {
	cParser parser("1 2.5 -3 4", cParser::buffer_TEXT);
	auto const point{LoadPoint(parser)};
	CHECK(point.x == doctest::Approx(1.0));
	CHECK(point.y == doctest::Approx(2.5));
	CHECK(point.z == doctest::Approx(-3.0));
	CHECK(parser.getToken<int>() == 4);
	// the ones there's no number for are zero
	cParser few("1 2.5", cParser::buffer_TEXT);
	auto const flat{LoadPoint(few)};
	CHECK(flat.x == doctest::Approx(1.0));
	CHECK(flat.y == doctest::Approx(2.5));
	CHECK(flat.z == 0.0);
}

TEST_CASE("parser: a set of names is a single one, or several in brackets") {
	cParser single("Dir\\File.T3D next", cParser::buffer_TEXT);
	// in lower case, with the slashes turned
	CHECK(deserialize_set(single) == std::vector<std::string>{"dir/file.t3d"});
	CHECK(single.getToken<std::string>() == "next");

	cParser several("[ a B\\c d ] next", cParser::buffer_TEXT);
	CHECK(deserialize_set(several) == std::vector<std::string>{"a", "b/c", "d"});
	CHECK(several.getToken<std::string>() == "next");

	// the brackets have to stand apart from the names
	cParser touching("[a b] next", cParser::buffer_TEXT);
	CHECK(deserialize_set(touching) == std::vector<std::string>{"[a"});
	// a set left open ends with the text, nothing gives an empty one
	cParser open("[ a b", cParser::buffer_TEXT);
	CHECK(deserialize_set(open) == std::vector<std::string>{"a", "b"});
	cParser nothing("", cParser::buffer_TEXT);
	CHECK(deserialize_set(nothing).empty());
	// the separators are the caller's
	cParser commas("[,a,b,]", cParser::buffer_TEXT);
	CHECK(deserialize_set(commas, ",") == std::vector<std::string>{"a", "b"});
}

TEST_CASE("parser: one name of a set is picked at random") {
	cParser single("Dir\\File.T3D next", cParser::buffer_TEXT);
	CHECK(deserialize_random_set(single) == "dir/file.t3d");
	CHECK(single.getToken<std::string>() == "next");

	cParser one("[ only ] next", cParser::buffer_TEXT);
	CHECK(deserialize_random_set(one) == "only");
	CHECK(one.getToken<std::string>() == "next");

	// the whole set is read whichever is picked
	cParser several("[ a b c ] next", cParser::buffer_TEXT);
	auto const picked{deserialize_random_set(several)};
	CHECK((picked == "a" || picked == "b" || picked == "c"));
	CHECK(several.getToken<std::string>() == "next");

	// a set inside of a set gives one of its names to the outer one
	cParser nested("[ [ x ] ] next", cParser::buffer_TEXT);
	CHECK(deserialize_random_set(nested) == "x");
	CHECK(nested.getToken<std::string>() == "next");
	// an empty set gives nothing
	cParser empty("[ ] next", cParser::buffer_TEXT);
	CHECK(deserialize_random_set(empty).empty());
	CHECK(empty.getToken<std::string>() == "next");
}

TEST_CASE("parser: the same seed picks the same name of a set") {
	std::string const text{"[ a b c d e f g h ]"};
	Global.random_engine.seed(7);
	cParser first(text, cParser::buffer_TEXT);
	auto const picked{deserialize_random_set(first)};
	Global.random_engine.seed(7);
	cParser second(text, cParser::buffer_TEXT);
	CHECK(deserialize_random_set(second) == picked);
}

TEST_CASE("parser: a map is read a line for a key and its value") {
	cParser parser("alpha One\nbeta Two Words", cParser::buffer_TEXT);
	std::map<std::string, std::string> map;
	deserialize_map(map, parser);
	CHECK(map.size() == 2);
	// with the case kept, the value being the rest of the line
	CHECK(map["alpha"] == "One");
	CHECK(map["beta"] == "Two Words");

	SUBCASE("the values are read as their type") {
		cParser numbers("a 1.5\nb 2", cParser::buffer_TEXT);
		std::map<std::string, double> values;
		deserialize_map(values, numbers);
		CHECK(values.size() == 2);
		CHECK(values["a"] == doctest::Approx(1.5));
		CHECK(values["b"] == doctest::Approx(2.0));
	}
	SUBCASE("a line end after the last line adds an empty key") {
		cParser ended("alpha One\nbeta Two\n", cParser::buffer_TEXT);
		std::map<std::string, std::string> entries;
		deserialize_map(entries, ended);
		CHECK(entries.size() == 3);
		CHECK(entries.count("") == 1);
	}
	SUBCASE("and so does an empty text") {
		cParser empty("", cParser::buffer_TEXT);
		std::map<std::string, std::string> entries;
		deserialize_map(entries, empty);
		CHECK(entries.size() == 1);
		CHECK(entries.count("") == 1);
	}
	SUBCASE("the carriage return of a line ended the Windows way stays in the value") {
		cParser windows("alpha One\r\nbeta Two", cParser::buffer_TEXT);
		std::map<std::string, std::string> entries;
		deserialize_map(entries, windows);
		CHECK(entries["alpha"] == "One\r");
		CHECK(entries["beta"] == "Two");
	}
}
