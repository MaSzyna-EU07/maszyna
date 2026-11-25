#include "stdafx.h"
#include "doctest.h"
#include "scratch.h"

#include "utilities/wholefile.h"

#include <string>
#include <string_view>

// the read of a whole file in one go (utilities/wholefile.cpp), which the loader of the binary models (.e3d) parses
// from memory: the bytes come as they are in the file, whatever they are

using namespace std::string_view_literals;

namespace {

std::string_view content(whole_file const &File) {
	return {File.data.get(), File.size};
}

} // namespace

TEST_CASE("whole file: the bytes come as they are in the file, the line ends and the text end marks too") {
	test::scratch_folder const folder{"whole_file"};
	// all the byte values, then a size which isn't a multiple of any buffer, so the end of the file is read as well
	std::string bytes;
	for (int i{0}; i < 256; ++i) {
		bytes += static_cast<char>(i);
	}
	bytes += "\r\n\n\r\x1a\x00\xff"sv;
	std::string text;
	while (text.size() < 70000) {
		text += bytes;
	}
	text += "end";
	auto const path{folder.write("model.e3d", text)};

	auto const file{read_whole_file(path)};
	REQUIRE(file.size == text.size());
	CHECK(content(file) == text);
}

TEST_CASE("whole file: a small file is read whole") {
	test::scratch_folder const folder{"whole_file_small"};
	auto const path{folder.write("e3d0.e3d", "E3D0")};
	auto const file{read_whole_file(path)};
	CHECK(content(file) == "E3D0");
}

TEST_CASE("whole file: an empty file and a file which isn't there give no data") {
	test::scratch_folder const folder{"whole_file_none"};
	auto const empty{read_whole_file(folder.write("empty.e3d", ""))};
	CHECK(empty.size == 0);
	CHECK(empty.data == nullptr);

	auto const missing{read_whole_file(folder.path("missing.e3d"))};
	CHECK(missing.size == 0);
	CHECK(missing.data == nullptr);
}
