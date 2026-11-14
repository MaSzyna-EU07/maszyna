#include "stdafx.h"
#include "doctest.h"
#include "scratch.h"
#include "scene/regionfile.h"
#include "scene/sn_utils.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

// the region file (<scenery>.sbt), the static geometry of a scenery the game caches beside it: a file cut short or
// damaged is turned down before any of it is used, and the game writes it so that it is never left cut short

namespace {

std::uint32_t constexpr header{0x37305545}; // EU07
std::uint32_t constexpr version{0x32544253}; // SBT2
std::uint32_t constexpr sectionlimit{500 * 500};

struct section {
	std::uint32_t index;
	std::string data;
};

// the file as basic_region::serialize() lays it out: the section size covers the index, the size and the data
std::string region(std::vector<section> const &Sections, std::uint32_t const Header = header, std::uint32_t const Version = version) {
	std::ostringstream output;
	sn_utils::ls_uint32(output, Header);
	sn_utils::ls_uint32(output, Version);
	sn_utils::ls_uint32(output, static_cast<std::uint32_t>(Sections.size()));
	for (auto const &section : Sections) {
		sn_utils::ls_uint32(output, section.index);
		sn_utils::ls_uint32(output, static_cast<std::uint32_t>(scene::region_file::section_header_size + section.data.size()));
		output << section.data;
	}
	return output.str();
}

std::string check(std::string const &File) {
	std::istringstream input{File};
	return scene::region_file::check_layout(input, header, version, sectionlimit);
}

std::string read(std::filesystem::path const &Path) {
	std::ifstream input{Path, std::ios::binary};
	return {std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
}

} // namespace

TEST_CASE("region file: a file laid out whole is accepted, with no sections or with many") {
	CHECK(check(region({})) == "");
	CHECK(check(region({{0, ""}, {7, std::string(100, 'x')}, {sectionlimit - 1, std::string(3, '\0')}})) == "");
}

TEST_CASE("region file: a file cut short anywhere is turned down") {
	auto const file{region({{12, std::string(40, 'a')}, {13, std::string(7, 'b')}, {250, std::string(64, 'c')}})};
	REQUIRE(check(file) == "");
	for (std::size_t length{0}; length < file.size(); ++length) {
		CAPTURE(length);
		CHECK(check(file.substr(0, length)) != "");
	}
	// the count of the sections is checked against those present, and the length against where the last one ends
	CHECK(check(file.substr(0, file.size() - 64 - 8)).find("ends after 2 of its 3 sections") != std::string::npos);
	CHECK(check(file.substr(0, file.size() / 2)).find("ends inside section") != std::string::npos);
}

TEST_CASE("region file: a file of another type or version is turned down") {
	CHECK(check(region({{1, "x"}}, header, version + 1)).find("unrecognized type or version") != std::string::npos);
	CHECK(check(region({{1, "x"}}, header + 1, version)).find("unrecognized type or version") != std::string::npos);
	CHECK(check("EU0").find("unrecognized type or version") != std::string::npos);
}

TEST_CASE("region file: sections the region can't hold, held twice, or of impossible sizes are turned down") {
	CHECK(check(region({{sectionlimit, "x"}})).find("out of the 250000") != std::string::npos);
	CHECK(check(region({{0xffffffffu, "x"}})).find("out of the 250000") != std::string::npos);
	CHECK(check(region({{5, "x"}, {5, "y"}})).find("more than once") != std::string::npos);

	// a size smaller than the header of the section
	auto file{region({{5, "abcd"}})};
	file[16] = 7;
	file[17] = file[18] = file[19] = 0;
	CHECK(check(file).find("less than its header takes") != std::string::npos);
	// a size reaching past the end of the file
	file[16] = 13;
	CHECK(check(file).find("ends inside section 5") != std::string::npos);
	// a size falling short of it
	file[16] = 11;
	CHECK(check(file).find("bytes past its last section") != std::string::npos);
	// more sections than a region has
	auto many{region({})};
	many[8] = many[9] = many[10] = many[11] = static_cast<char>(0xff);
	CHECK(check(many).find("more than the 250000") != std::string::npos);
}

TEST_CASE("region file: the header of a section is read and checked") {
	std::uint32_t index{0}, size{0};
	{
		std::istringstream input{region({{42, "abc"}}).substr(12)};
		CHECK(scene::region_file::read_section_header(input, sectionlimit, index, size) == "");
		CHECK(index == 42);
		CHECK(size == 11);
	}
	{
		std::istringstream input{region({{42, "abc"}}).substr(12, 6)};
		CHECK(scene::region_file::read_section_header(input, sectionlimit, index, size).find("ends inside a section header") != std::string::npos);
	}
	{
		std::istringstream input{region({{42, "abc"}}).substr(12)};
		CHECK(scene::region_file::read_section_header(input, 42, index, size).find("out of the 42") != std::string::npos);
	}
}

TEST_CASE("region file: a string cut short by the end of the data ends there, rather than being read on forever") {
	std::istringstream input{std::string{"abc"}};
	CHECK(sn_utils::d_str(input) == "abc");
	CHECK(input.fail());
	CHECK(sn_utils::d_str(input) == "");

	std::istringstream whole{std::string{"abc\0def", 7}};
	CHECK(sn_utils::d_str(whole) == "abc");
	CHECK(whole.good());
}

TEST_CASE("region file: the bytes left in the data are known, so a count it can't hold is turned down before anything is made for it") {
	std::istringstream input{std::string(100, 'x')};
	CHECK(sn_utils::remaining(input) == 100);
	sn_utils::ld_uint32(input);
	CHECK(sn_utils::remaining(input) == 96);
	// asking doesn't move the reading on
	CHECK(sn_utils::ld_uint32(input) == 0x78787878);
	input.seekg(0, std::ios::end);
	CHECK(sn_utils::remaining(input) == 0);
	sn_utils::ld_uint32(input);
	CHECK(input.fail());
	CHECK(sn_utils::remaining(input) == 0);
}

TEST_CASE("region file: the file is written through a temporary one, which replaces the previous file when complete") {
	test::scratch_folder folder{"region_file"};
	auto const file{folder.root() / "scenery.sbt"};
	auto const temporary{folder.root() / "scenery.sbt.tmp"};
	auto const write = [&](std::string const &Content) {
		return scene::region_file::write_through_temporary(file.string(), [&](std::ostream &Output) { Output << Content; });
	};

	SUBCASE("a new file") {
		CHECK(write("first") == "");
		CHECK(read(file) == "first");
		CHECK_FALSE(std::filesystem::exists(temporary));
	}
	SUBCASE("a previous file is replaced, a longer one entirely") {
		REQUIRE(write("a much longer first version") == "");
		CHECK(write("second") == "");
		CHECK(read(file) == "second");
		CHECK_FALSE(std::filesystem::exists(temporary));
	}
	SUBCASE("a temporary file left by a write cut short earlier is written over") {
		folder.write("scenery.sbt.tmp", "left over from a crash, longer than what follows");
		CHECK(write("third") == "");
		CHECK(read(file) == "third");
		CHECK_FALSE(std::filesystem::exists(temporary));
	}
}

TEST_CASE("region file: a write which fails leaves the previous file, or none, and no temporary file") {
	test::scratch_folder folder{"region_file_failed"};
	auto const file{folder.root() / "scenery.sbt"};
	auto const temporary{folder.root() / "scenery.sbt.tmp"};
	folder.write("scenery.sbt", "previous");

	SUBCASE("the writer gives up half way") {
		auto const error{scene::region_file::write_through_temporary(file.string(), [](std::ostream &Output) {
			Output << "half of the";
			throw std::runtime_error("out of memory");
		})};
		CHECK(error.find("out of memory") != std::string::npos);
	}
	SUBCASE("the stream fails") {
		auto const error{scene::region_file::write_through_temporary(file.string(), [](std::ostream &Output) {
			Output << "half of the";
			Output.setstate(std::ios::badbit);
		})};
		CHECK(error.find("can't write to file") != std::string::npos);
	}
	CHECK(read(file) == "previous");
	CHECK_FALSE(std::filesystem::exists(temporary));

	SUBCASE("the temporary file can't be made") {
		auto const missing{folder.root() / "no such folder" / "scenery.sbt"};
		auto const error{scene::region_file::write_through_temporary(missing.string(), [](std::ostream &Output) { Output << "x"; })};
		CHECK(error.find("can't write to file") != std::string::npos);
		CHECK_FALSE(std::filesystem::exists(missing));
	}
}
