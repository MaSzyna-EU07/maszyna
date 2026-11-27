#include "stdafx.h"
#include "doctest.h"
#include "scratch.h"

#include "world/TrackSleepers.h"

#include <filesystem>
#include <fstream>
#include <string>

// the models of sleepers and timbers the track editor lists (world/TrackSleepers.h), found by the names of the files in a
// folder. a file named with a character the code page lacks is left out, rather than throwing out of the scan (GE#560)

namespace {

void write_model(std::filesystem::path const &Path, std::string const &Content) {
	std::filesystem::create_directories(Path.parent_path());
	std::ofstream file{Path, std::ios::binary};
	file << Content;
}

// whether the name can be turned into the narrow names of the code page: on Windows a Chinese character can't be with any
// of the single byte pages, where the narrow names are UTF-8 it can
bool narrow_name(std::filesystem::path const &Path) {
	try {
		static_cast<void>(Path.generic_string());
		return true;
	} catch (std::exception const &) {
		return false;
	}
}

} // namespace

TEST_CASE("sleeper models: a file named outside of the code page is left out, the rest is listed") {
	test::scratch_folder const folder{"sleeper_models"};
	auto const root{folder.root()};
	write_model(root / "pkp" / "podklad_a.t3d", "Name: podklad\n");
	write_model(root / "pkp" / "podrozjazd_b.t3d", "Name: begin\nName: main\nName: end\n");
	auto const odd{root / "pkp" / std::filesystem::path{u8"podklad_\u4e2d.t3d"}};
	write_model(odd, "Name: podklad\n");
	// nor any other model of the folder named so, looked at before its name tells it's no sleeper
	write_model(root / "drzewa" / std::filesystem::path{u8"drzewo_\u4e2d.e3d"}, "");
	write_model(root / "drzewa" / "drzewo.e3d", "");
	REQUIRE(std::filesystem::exists(odd));

	std::vector<track_sleepers::pack_model> models;
	REQUIRE_NOTHROW(models = track_sleepers::pack_models(root));

	auto const listed{narrow_name(odd)};
	REQUIRE(models.size() == (listed ? 3u : 2u));
	// the timbers first, then by the name
	CHECK(models[0].name == "pkp/podrozjazd_b");
	CHECK(models[0].timber);
	CHECK(models[1].name == "pkp/podklad_a");
	CHECK_FALSE(models[1].timber);
	if (listed) {
		CHECK(models[2].name.find("pkp/podklad_") == 0);
	}
}

TEST_CASE("sleeper models: a folder missing lists none") {
	test::scratch_folder const folder{"sleeper_models_missing"};
	std::vector<track_sleepers::pack_model> models;
	REQUIRE_NOTHROW(models = track_sleepers::pack_models(folder.root() / "missing"));
	CHECK(models.empty());
}
