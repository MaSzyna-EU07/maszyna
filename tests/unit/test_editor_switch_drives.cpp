#include "stdafx.h"
#include "doctest.h"
#include "editor/editorSwitchDrives.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace {

// a track of the scenery: its name as written, and whether the editor removed it
struct track
{
	std::string name;
	bool removed{false};
};

// the scenery as the game finds its tracks: Paths finds a name as written or lower case, the later of two tracks of a name; a track
// deleted takes its name away (editor_track::retire()), Retired lists those names
struct scenery
{
	std::vector<track> tracks;
	std::vector<std::string> retired;
	int lookups{0}; // of the whole sequence

	std::optional<bool> find(std::string const &Name) const
	{
		auto const name{switch_drives::game_name(Name)};
		if (std::ranges::find(retired, name) != retired.end())
			return std::nullopt;
		std::optional<bool> found;
		for (auto const &entry : tracks)
			if (switch_drives::game_name(entry.name) == name)
				found = entry.removed; // the later one
		return found;
	}

	std::vector<std::string> standing()
	{
		++lookups;
		std::vector<std::string> names;
		for (auto const &entry : tracks)
			if (false == entry.removed)
				names.push_back(entry.name);
		return names;
	}

	switch_drives::standing_switches lookup()
	{
		return switch_drives::standing_switches{[this](std::string const &Name) { return find(Name); }, [this] { return standing(); }};
	}

	// deletes the later track of the name, as the editor does
	void erase_later(std::string const &Name)
	{
		auto const name{switch_drives::game_name(Name)};
		for (auto entry{tracks.rbegin()}; entry != tracks.rend(); ++entry)
			if (switch_drives::game_name(entry->name) == name && false == entry->removed)
			{
				entry->removed = true;
				retired.push_back(name);
				return;
			}
	}
};

} // namespace

TEST_CASE("switch drives: a name is found as the game finds it")
{
	CHECK(switch_drives::game_name("Zwr\\12A") == "zwr/12a");
	CHECK(switch_drives::game_name("zwr_1") == "zwr_1");

	scenery line{{{"Zwr1"}, {"zwr2", true}}, {}};
	auto switches{line.lookup()};
	CHECK(switches.stands("Zwr1"));
	CHECK(switches.stands("ZWR1"));
	// one the name finds, removed, doesn't stand
	CHECK_FALSE(switches.stands("zwr2"));
	CHECK_FALSE(switches.stands("zwr3"));
}

TEST_CASE("switch drives: two switches of one name, the later deleted, leave the drive throwing the earlier")
{
	// an older scenery has two tracks of the name; the drive set to it throws the one the game finds
	scenery line{{{"zwr7"}, {"other"}, {"Zwr7"}}, {}};
	line.erase_later("zwr7");
	REQUIRE_FALSE(line.find("zwr7").has_value()); // the name went with the track deleted
	{
		auto switches{line.lookup()};
		// the earlier one still stands: the drive isn't stray, and a hint to delete it would delete a working drive
		CHECK(switches.stands("zwr7"));
		CHECK(switches.stands("ZWR7"));
		CHECK(switches.any_stands({"zwr7"}));
		CHECK_FALSE(switches.stands("zwr8"));
		// the tracks of the scenery are read once, however many names find none
		CHECK(line.lookups == 1);
	}
	// both deleted: the drive throws nothing now
	line.erase_later("zwr7");
	auto switches{line.lookup()};
	CHECK_FALSE(switches.stands("zwr7"));
	CHECK_FALSE(switches.any_stands({"zwr7"}));
}

TEST_CASE("switch drives: the drive of a double switch stands while one of its four switches does")
{
	scenery line{{{"dkw1a"}, {"dkw1b"}, {"dkw1c"}, {"dkw1d"}}, {}};
	std::vector<std::string> const names{"dkw1a", "dkw1b", "dkw1c", "dkw1d"};
	for (auto const *name : {"dkw1a", "dkw1b", "dkw1c"})
	{
		line.erase_later(name);
		CHECK(line.lookup().any_stands(names));
	}
	line.erase_later("dkw1d");
	CHECK_FALSE(line.lookup().any_stands(names));
}

TEST_CASE("switch drives: the switches of a template are read once while the templates stay the same")
{
	switch_drives::template_switches templates;
	std::map<std::string, int> reads;
	std::map<std::string, std::vector<std::string>> files{{"napedy/naped_l.inc", {""}}, {"napedy/dkw.inc", {"a", "b", "c", "d"}}, {"podsypka.inc", {}}};
	auto const read = [&reads, &files](std::string const &File, std::vector<std::string> &Suffixes) {
		++reads[File];
		auto const found{files.find(File)};
		if (found == files.end())
			return false;
		Suffixes = found->second;
		return true;
	};

	SUBCASE("read once in a revision")
	{
		CHECK(templates.of("napedy/naped_l.inc", 1, 10, read) == std::vector<std::string>{""});
		CHECK(templates.of("napedy/naped_l.inc", 1, 11, read) == std::vector<std::string>{""});
		CHECK(templates.of("napedy/dkw.inc", 1, 12, read).size() == 4);
		CHECK(templates.of("podsypka.inc", 1, 12, read).empty());
		CHECK(templates.of("podsypka.inc", 1, 13, read).empty());
		CHECK(reads["napedy/naped_l.inc"] == 1);
		CHECK(reads["napedy/dkw.inc"] == 1);
		CHECK(reads["podsypka.inc"] == 1);
	}

	SUBCASE("a template changed is read again in the next revision")
	{
		CHECK(templates.of("napedy/naped_l.inc", 1, 10, read) == std::vector<std::string>{""});
		// saved in the editor, or changed on the disk and rescanned: no drive any more
		files["napedy/naped_l.inc"] = {};
		CHECK(templates.of("napedy/naped_l.inc", 1, 11, read) == std::vector<std::string>{""});
		CHECK(templates.of("napedy/naped_l.inc", 2, 11, read).empty());
		CHECK(reads["napedy/naped_l.inc"] == 2);
	}

	SUBCASE("a file that can't be read is tried again in a later frame")
	{
		CHECK(templates.of("napedy/nowy.inc", 1, 10, read).empty());
		// each include of it asks in the same frame: read once
		CHECK(templates.of("napedy/nowy.inc", 1, 10, read).empty());
		CHECK(reads["napedy/nowy.inc"] == 1);
		CHECK(templates.of("napedy/nowy.inc", 1, 11, read).empty());
		CHECK(reads["napedy/nowy.inc"] == 2);
		// there now, with the same revision: no rescan needed
		files["napedy/nowy.inc"] = {""};
		CHECK(templates.of("napedy/nowy.inc", 1, 12, read) == std::vector<std::string>{""});
		CHECK(templates.of("napedy/nowy.inc", 1, 13, read) == std::vector<std::string>{""});
		CHECK(reads["napedy/nowy.inc"] == 3);
	}
}
