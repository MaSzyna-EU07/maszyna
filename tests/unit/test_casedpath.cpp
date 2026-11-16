#include "stdafx.h"
#include "doctest.h"
#include "scratch.h"

#include "utilities/casedpath.h"
#include "utilities/utilities.h"

#include <atomic>
#include <map>
#include <mutex>
#include <set>
#include <thread>

// files named in another case than on the disk (utilities/casedpath.h). the index is tested twice: on a file system made
// up here, which tells the case apart on every system, and on the real one in a scratch folder, which tells it apart on
// Linux (the CI) and not on Windows, where only what holds on both is checked

namespace {

// a file system which tells the case apart: files and the folders they are in, by their paths as normal_path() gives them
class fake_disk {
  public:
	fake_disk() { m_folders.insert("."); }

	// a file, with the folders it's in
	void add(std::string const &Path) {
		std::scoped_lock const lock{m_lock};
		auto path{cased::normal_path(Path)};
		m_files.insert(path);
		while (path != "." && path != "/") {
			auto const parent{parent_of(path)};
			m_children[parent].insert(name_of(path));
			m_folders.insert(parent);
			path = parent;
		}
	}
	void remove(std::string const &Path) {
		std::scoped_lock const lock{m_lock};
		auto const path{cased::normal_path(Path)};
		m_files.erase(path);
		m_children[parent_of(path)].erase(name_of(path));
	}

	bool exists(std::string const &Path) const {
		std::scoped_lock const lock{m_lock};
		auto const path{cased::normal_path(Path)};
		return m_files.count(path) > 0 || m_folders.count(path) > 0;
	}
	bool list(std::string const &Directory, std::vector<std::string> &Names) {
		std::scoped_lock const lock{m_lock};
		++m_listed[Directory];
		if (m_folders.count(Directory) == 0) {
			return false;
		}
		auto const children{m_children.find(Directory)};
		if (children != m_children.end()) {
			Names.assign(children->second.begin(), children->second.end());
		}
		return true;
	}
	// how many times the folder was listed
	int listed(std::string const &Directory) const {
		std::scoped_lock const lock{m_lock};
		auto const found{m_listed.find(Directory)};
		return found != m_listed.end() ? found->second : 0;
	}

  private:
	static std::string parent_of(std::string const &Path) {
		auto const slash{Path.rfind('/')};
		return slash == std::string::npos ? std::string{"."} : slash == 0 ? std::string{"/"} : Path.substr(0, slash);
	}
	static std::string name_of(std::string const &Path) {
		auto const slash{Path.rfind('/')};
		return slash == std::string::npos ? Path : Path.substr(slash + 1);
	}

	mutable std::mutex m_lock;
	std::set<std::string> m_files;
	std::set<std::string> m_folders;
	std::map<std::string, std::set<std::string>> m_children;
	std::map<std::string, int> m_listed;
};

// an index of the made up file system
struct fake_index {
	fake_disk disk;
	cased::index index{[this](std::string const &Directory, std::vector<std::string> &Names) { return disk.list(Directory, Names); },
	                   [this](std::string const &Path) { return disk.exists(Path); }};

	fake_index() = default;
	fake_index(fake_index const &) = delete;
	fake_index &operator=(fake_index const &) = delete;
};

// whether the scratch folder tells the case apart
bool case_matters(test::scratch_folder const &Folder) {
	Folder.write("probe.txt", "x");
	return false == cased::path_exists(Folder.path("PROBE.TXT"));
}

// whether the file or the folder is on the disk named exactly so, as if the disk told the case apart (on Windows too): each
// name below the folder given has to be among the names listed in its folder
bool exactly_on_disk(std::string const &Path, std::string const &Below) {
	if (false == cased::path_exists(Path)) {
		return false;
	}
	auto const path{cased::normal_path(Path)};
	auto const below{cased::normal_path(Below)};
	if (path.size() <= below.size() || path.compare(0, below.size(), below) != 0 || path[below.size()] != '/') {
		// the folder itself, or outside of it
		return true;
	}
	auto folder{below};
	auto position{below.size() + 1};
	while (position < path.size()) {
		auto end{path.find('/', position)};
		if (end == std::string::npos) {
			end = path.size();
		}
		auto const name{path.substr(position, end - position)};
		std::vector<std::string> names;
		if (false == cased::list_directory(folder, names) || std::find(names.begin(), names.end(), name) == names.end()) {
			return false;
		}
		folder += '/' + name;
		position = end + 1;
	}
	return true;
}

// the timetables as the data has them (scenery/galicja2000/rozklady, scenery/Braniewo), asked for as the game asks for them:
// "scenery/" + the name from the scenery, lowercased by the parser, + ".txt" (world/mtable.cpp, TTrainParameters::LoadTTfile)
char const *const galicja_timetable{"scenery/galicja2000/rozklady/2003/TMZ77184.txt"};
char const *const galicja_asked{"scenery/galicja2000/rozklady/2003/tmz77184.txt"};
char const *const braniewo_timetable{"scenery/Braniewo/rozklady/TGS3657.txt"};
char const *const braniewo_asked{"scenery/braniewo/rozklady/tgs3657.txt"};

} // namespace

// the helpers -----------------------------------------------------------------------------------------------------------

TEST_CASE("cased path: names are lowered in ASCII only, the other bytes kept") {
	CHECK(cased::lower("Scenery/Braniewo/TGS3657.TXT") == "scenery/braniewo/tgs3657.txt");
	// UTF-8 Łódź and cp1250 Łódź: the bytes of the letters outside of ASCII as they are, as ToLower() keeps them
	CHECK(cased::lower("\xC5\x81\xC3\xB3" "D\xC5\xBA") == "\xC5\x81\xC3\xB3" "d\xC5\xBA");
	CHECK(cased::lower("\xA3\xF3" "D\x9F") == "\xA3\xF3" "d\x9F");
	CHECK(cased::lower("") == "");
}

TEST_CASE("cased path: a path is made normal the way the folders are kept") {
	CHECK(cased::normal_path("scenery/braniewo") == "scenery/braniewo");
	CHECK(cased::normal_path("scenery\\braniewo\\") == "scenery/braniewo");
	CHECK(cased::normal_path("./scenery//braniewo/.") == "scenery/braniewo");
	CHECK(cased::normal_path("scenery/braniewo/../galicja2000") == "scenery/galicja2000");
	CHECK(cased::normal_path("scenery/..") == ".");
	CHECK(cased::normal_path("") == ".");
	CHECK(cased::normal_path(".") == ".");
	// above the current folder the '..' stay, above the root is the root
	CHECK(cased::normal_path("../../data") == "../../data");
	CHECK(cased::normal_path("a/../../data") == "../data");
	CHECK(cased::normal_path("/data/../..") == "/");
	CHECK(cased::normal_path("//data") == "/data");
	CHECK(cased::normal_path("/") == "/");
#ifdef _WIN32
	CHECK(cased::normal_path("C:\\Games\\MaSzyna\\..\\x") == "C:/Games/x");
	CHECK(cased::normal_path("c:/") == "c:/");
#endif
}

// a file system which tells the case apart -----------------------------------------------------------------------------

TEST_CASE("cased path: a path named as it is on the disk is given back as it is, with nothing listed") {
	fake_index fake;
	fake.disk.add("scenery/plain.scn");
	CHECK(fake.index.resolve("scenery/plain.scn") == "scenery/plain.scn");
	CHECK(fake.index.resolve("scenery") == "scenery");
	CHECK(fake.index.resolve(".") == ".");
	CHECK(fake.index.listings() == 0);
	// nothing to look for
	CHECK(fake.index.resolve("").empty());
	CHECK(fake.index.resolve_in_any_case("").empty());
}

TEST_CASE("cased path: a file named in another case is found under its name on the disk") {
	fake_index fake;
	fake.disk.add("scenery/Cased.TXT");
	CHECK(fake.index.resolve("scenery/cased.txt") == "scenery/Cased.TXT");
	CHECK(fake.index.resolve("scenery/CASED.txt") == "scenery/Cased.TXT");
	// only the folder of the file is listed, the folder named as it is needs no listing
	CHECK(fake.disk.listed("scenery") == 1);
	CHECK(fake.disk.listed(".") == 0);
	CHECK(fake.index.listings() == 1);
}

TEST_CASE("cased path: a folder named in another case is found, the file in it as well") {
	fake_index fake;
	fake.disk.add("scenery/Braniewo/Base.ctr");
	fake.disk.add("scenery/Braniewo/szeroki.ctr");
	CHECK(fake.index.resolve("scenery/braniewo/base.ctr") == "scenery/Braniewo/Base.ctr");
	CHECK(fake.index.resolve("scenery/braniewo/szeroki.ctr") == "scenery/Braniewo/szeroki.ctr");
	CHECK(fake.index.resolve("scenery/braniewo") == "scenery/Braniewo");
	CHECK(fake.index.resolve("scenery/braniewo/") == "scenery/Braniewo/");
	// each folder listed once
	CHECK(fake.disk.listed("scenery") == 1);
	CHECK(fake.disk.listed("scenery/Braniewo") == 1);
}

TEST_CASE("cased path: a deep path with names in another case on many levels is found") {
	fake_index fake;
	fake.disk.add("dynamic/pkp/vectron_v1/display/D/Screen.PNG");
	fake.disk.add("A/b/C/d/E/f.TXT");
	CHECK(fake.index.resolve("dynamic/pkp/vectron_v1/display/d/screen.png") == "dynamic/pkp/vectron_v1/display/D/Screen.PNG");
	CHECK(fake.index.resolve("a/B/c/D/e/F.txt") == "A/b/C/d/E/f.TXT");
	CHECK(fake.index.resolve("a/b/c/d/e/f.txt") == "A/b/C/d/E/f.TXT");
}

TEST_CASE("cased path: a file which isn't there in any case isn't looked for again") {
	fake_index fake;
	fake.disk.add("textures/Wall.dds");
	CHECK(fake.index.resolve("textures/nothing.dds").empty());
	CHECK(fake.disk.listed("textures") == 1);
	CHECK(fake.index.resolve("textures/nothing.dds").empty());
	CHECK(fake.index.resolve("textures/NOTHING.DDS").empty());
	// another file in the same folder: from the names listed already
	CHECK(fake.index.resolve("textures/wall.dds") == "textures/Wall.dds");
	CHECK(fake.disk.listed("textures") == 1);
	// nor a folder which isn't there, nor what's in it
	CHECK(fake.index.resolve("textures/none/wall.dds").empty());
	CHECK(fake.index.resolve("textures/none/other.dds").empty());
	CHECK(fake.disk.listed("textures") == 1);
	CHECK(fake.disk.listed("textures/none") == 0);
	// a file in place of a folder
	CHECK(fake.index.resolve("textures/wall.dds/x").empty());
	CHECK(fake.index.listings() == 2);
	CHECK(fake.disk.listed("textures/Wall.dds") == 1);
}

TEST_CASE("cased path: a file added to a folder listed already is found once the folder is forgotten") {
	fake_index fake;
	fake.disk.add("scenery/Old.scm");
	CHECK(fake.index.resolve("scenery/old.scm") == "scenery/Old.scm");
	fake.disk.add("scenery/New.scm");
	// as the folder was
	CHECK(fake.index.resolve("scenery/new.scm").empty());
	// named as it is it's there anyway
	CHECK(fake.index.resolve("scenery/New.scm") == "scenery/New.scm");
	// the folder named in any case, in any form
	fake.index.forget("SCENERY\\");
	CHECK(fake.index.resolve("scenery/new.scm") == "scenery/New.scm");
	CHECK(fake.disk.listed("scenery") == 2);

	fake.disk.add("scenery/Third.scm");
	CHECK(fake.index.resolve("scenery/third.scm").empty());
	fake.index.forget_all();
	CHECK(fake.index.resolve("scenery/third.scm") == "scenery/Third.scm");
	// forgetting another folder leaves this one
	fake.index.forget("textures");
	fake.index.forget("scenery/third.scm");
	CHECK(fake.index.resolve("scenery/old.scm") == "scenery/Old.scm");
	CHECK(fake.disk.listed("scenery") == 3);
}

TEST_CASE("cased path: a file taken away after its folder was listed isn't given") {
	fake_index fake;
	fake.disk.add("scenery/Gone.scm");
	CHECK(fake.index.resolve("scenery/gone.scm") == "scenery/Gone.scm");
	fake.disk.remove("scenery/Gone.scm");
	// the folder is listed again, once, and the name isn't there
	CHECK(fake.index.resolve("scenery/gone.scm").empty());
	CHECK(fake.disk.listed("scenery") == 2);
	CHECK(fake.index.resolve("scenery/gone.scm").empty());
	CHECK(fake.disk.listed("scenery") == 2);
}

TEST_CASE("cased path: of names which differ in case only, the one named exactly is chosen, otherwise always the same one") {
	fake_index fake;
	fake.disk.add("collide/a.txt");
	fake.disk.add("collide/A.txt");
	fake.disk.add("collide/b.TXT");
	CHECK(fake.index.resolve("collide/a.txt") == "collide/a.txt");
	CHECK(fake.index.resolve("collide/A.txt") == "collide/A.txt");
	// in the order of the bytes, upper case first
	CHECK(fake.index.resolve("collide/a.TXT") == "collide/A.txt");
	CHECK(fake.index.resolve("COLLIDE/a.TXT") == "collide/A.txt");
	CHECK(fake.index.resolve("collide/B.txt") == "collide/b.TXT");
	// from the names listed: the one named exactly
	CHECK(fake.index.resolve_in_any_case("collide/a.txt") == "collide/a.txt");
	CHECK(fake.index.resolve_in_any_case("collide/A.txt") == "collide/A.txt");
	CHECK(fake.disk.listed("collide") == 1);

	// folders alike: the one named so, otherwise the first; the file is looked for in that one only
	fake.disk.add("Collide/Other.txt");
	fake.index.forget_all();
	CHECK(fake.index.resolve("collide/other.txt").empty());
	CHECK(fake.index.resolve("Collide/other.txt") == "Collide/Other.txt");
	CHECK(fake.index.resolve("COLLIDE/other.txt") == "Collide/Other.txt");
	CHECK(fake.index.resolve("COLLIDE/b.txt").empty());
}

TEST_CASE("cased path: the folders can be separated either way, and named with '.' and '..'") {
	fake_index fake;
	fake.disk.add("scenery/Braniewo/Base.ctr");
	fake.disk.add("scenery/Galicja/Main.scm");
	// the separators turned to '/', the rest of the path kept as given
	CHECK(fake.index.resolve("scenery\\braniewo\\base.ctr") == "scenery/Braniewo/Base.ctr");
	CHECK(fake.index.resolve("./scenery//braniewo/./base.ctr") == "./scenery//Braniewo/./Base.ctr");
	CHECK(fake.index.resolve("scenery/galicja/../braniewo/base.ctr") == "scenery/Galicja/../Braniewo/Base.ctr");
	CHECK(fake.index.resolve("scenery/braniewo/../galicja/main.scm") == "scenery/Braniewo/../Galicja/Main.scm");
	// a folder which isn't there can't be stepped out of
	CHECK(fake.index.resolve("scenery/none/../braniewo/base.ctr").empty());
	// the folder above the scenery listed once, whichever way it was named
	CHECK(fake.disk.listed("scenery") == 1);

	// a path from the root
	fake.disk.add("/games/MaSzyna/Scenery/X.scn");
	CHECK(fake.index.resolve("/games/maszyna/scenery/x.scn") == "/games/MaSzyna/Scenery/X.scn");
	CHECK(fake.index.resolve("\\games\\maszyna\\scenery\\..\\scenery\\x.scn") == "/games/MaSzyna/Scenery/../Scenery/X.scn");
	CHECK(fake.disk.listed("/") == 0);
	CHECK(fake.disk.listed("/games") == 1);
}

TEST_CASE("cased path: names outside of ASCII are compared byte by byte") {
	fake_index fake;
	// UTF-8 Łódź, and cp1250 Łódź
	std::string const utf8{"\xC5\x81\xC3\xB3" "d\xC5\xBA"};
	std::string const cp1250{"\xA3\xF3" "d\x9F"};
	fake.disk.add("scenery/" + utf8 + "/Plik.txt");
	fake.disk.add("scenery/" + cp1250 + "/Plik.txt");
	CHECK(fake.index.resolve("scenery/" + utf8 + "/plik.txt") == "scenery/" + utf8 + "/Plik.txt");
	CHECK(fake.index.resolve("scenery/" + cp1250 + "/plik.txt") == "scenery/" + cp1250 + "/Plik.txt");
	// the ASCII letters of the name in any case
	CHECK(fake.index.resolve("scenery/\xC5\x81\xC3\xB3" "D\xC5\xBA/plik.txt") == "scenery/" + utf8 + "/Plik.txt");
	// the letters outside of ASCII aren't folded, as ToLower() doesn't fold them: ł isn't Ł
	CHECK(fake.index.resolve("scenery/\xC5\x82\xC3\xB3" "d\xC5\xBA/plik.txt").empty());
	// half of a letter is no name
	CHECK(fake.index.resolve("scenery/\xC5/plik.txt").empty());
}

TEST_CASE("cased path: the timetables of galicja2000 and Braniewo are found as the game asks for them (GE#542)") {
	fake_index fake;
	fake.disk.add(galicja_timetable);
	fake.disk.add("scenery/galicja2000/rozklady/2003/RJ_44101.txt");
	fake.disk.add(braniewo_timetable);
	fake.disk.add("scenery/Braniewo/Base.ctr");
	CHECK(fake.index.resolve(galicja_asked) == galicja_timetable);
	CHECK(fake.index.resolve("scenery/galicja2000/rozklady/2003/rj_44101.txt") == "scenery/galicja2000/rozklady/2003/RJ_44101.txt");
	CHECK(fake.index.resolve(braniewo_asked) == braniewo_timetable);
	// a timetable which isn't there: a number for the speed, see LoadTTfile
	CHECK(fake.index.resolve("scenery/galicja2000/rozklady/2003/80.txt").empty());
	CHECK(fake.disk.listed("scenery/galicja2000/rozklady/2003") == 1);
}

TEST_CASE("cased path: many threads look for files at once, while the folders are forgotten") {
	fake_index fake;
	std::vector<std::pair<std::string, std::string>> asked;
	for (auto folder = 0; folder < 8; ++folder) {
		for (auto file = 0; file < 8; ++file) {
			auto const name{"Folder" + std::to_string(folder) + "/Sub/File" + std::to_string(file) + ".TXT"};
			fake.disk.add(name);
			asked.emplace_back(cased::lower(name), name);
		}
	}
	asked.emplace_back("folder0/sub/nothing.txt", "");
	asked.emplace_back("Folder1/Sub/File1.TXT", "Folder1/Sub/File1.TXT");

	std::atomic<int> wrong{0};
	std::atomic<bool> done{false};
	std::vector<std::thread> threads;
	for (auto thread = 0; thread < 8; ++thread) {
		threads.emplace_back([&, thread]() {
			for (auto round = 0; round < 200; ++round) {
				for (std::size_t i = 0; i < asked.size(); ++i) {
					auto const &entry{asked[(i + static_cast<std::size_t>(thread) * 7) % asked.size()]};
					if (fake.index.resolve(entry.first) != entry.second) {
						++wrong;
					}
				}
			}
		});
	}
	std::thread forgetting{[&]() {
		auto round{0};
		while (false == done) {
			if (++round % 2 == 0) {
				fake.index.forget("folder" + std::to_string(round % 8) + "/sub");
			}
			else {
				fake.index.forget_all();
			}
			std::this_thread::yield();
		}
	}};
	for (auto &thread : threads) {
		thread.join();
	}
	done = true;
	forgetting.join();
	CHECK(wrong.load() == 0);
}

// the real file system ------------------------------------------------------------------------------------------------

TEST_CASE("cased path: files and folders in another case on the disk itself") {
	test::scratch_folder const folder{"cased_disk"};
	auto const sensitive{case_matters(folder)};
	CAPTURE(sensitive);
	auto const root{folder.path()};
	folder.write(galicja_timetable, "galicja");
	folder.write(braniewo_timetable, "braniewo");
	folder.write("Dir/Sub/File.TXT", "file");

	cased::index index;
	// named as they are
	CHECK(index.resolve(root + galicja_timetable) == root + galicja_timetable);
	CHECK(index.resolve(root + "Dir/Sub/File.TXT") == root + "Dir/Sub/File.TXT");
	CHECK(index.listings() == 0);
	// named in lower case: the names on the disk where the case matters, the names given where it doesn't
	CHECK(index.resolve(root + galicja_asked) == root + (sensitive ? galicja_timetable : galicja_asked));
	CHECK(index.resolve(root + braniewo_asked) == root + (sensitive ? braniewo_timetable : braniewo_asked));
	CHECK(index.resolve(root + "dir/sub/file.txt") == root + (sensitive ? "Dir/Sub/File.TXT" : "dir/sub/file.txt"));
	CHECK(index.resolve(root + "dir/sub/../sub/file.txt") == root + (sensitive ? "Dir/Sub/../Sub/File.TXT" : "dir/sub/../sub/file.txt"));
	// the folder of the timetable of galicja2000; scenery and scenery/Braniewo/rozklady; the scratch folder, Dir and Dir/Sub
	CHECK(index.listings() == (sensitive ? 6u : 0u));
	// in any case, what's found is there
	CHECK(cased::path_exists(index.resolve(root + "DIR/SUB/FILE.TXT")));
	CHECK(cased::path_exists(index.resolve_in_any_case(root + galicja_asked)));
	// where the case doesn't matter the names are taken as given
	CHECK(index.resolve_in_any_case(root + galicja_asked) == root + (sensitive ? galicja_timetable : galicja_asked));

	// not there
	CHECK(index.resolve(root + "dir/sub/nothing.txt").empty());
	CHECK(index.resolve(root + "nothing/file.txt").empty());
	auto const listed{index.listings()};
	CHECK(index.resolve(root + "dir/sub/nothing.txt").empty());
	CHECK(index.resolve(root + "nothing/file.txt").empty());
	// not listed again. where the case doesn't matter, a folder taken as named and later found listed in its own case is
	// kept twice, so there it may be listed once more
	if (sensitive) {
		CHECK(index.listings() == listed);
	}
	else {
		CHECK(index.listings() <= listed + 1);
	}

	// added to a folder listed already: found once the folder is forgotten, where the case matters
	folder.write("Dir/Sub/Added.TXT", "added");
	CHECK(index.resolve(root + "dir/sub/added.txt") == (sensitive ? "" : root + "dir/sub/added.txt"));
	index.forget(root + "dir/sub");
	CHECK(index.resolve(root + "dir/sub/added.txt") == root + (sensitive ? "Dir/Sub/Added.TXT" : "dir/sub/added.txt"));
}

TEST_CASE("cased path: the folders on the disk listed, the disk taken as if it told the case apart") {
	// the lister of the game on real folders, on every system: only the check of a name as written is made up
	test::scratch_folder const folder{"cased_disk_exact"};
	auto const root{folder.path()};
	folder.write(galicja_timetable, "galicja");
	folder.write(braniewo_timetable, "braniewo");
	folder.write("Dir/Sub/File.TXT", "file");
	cased::index index{cased::list_directory, [&root](std::string const &Path) { return exactly_on_disk(Path, root); }};

	CHECK(index.resolve(root + galicja_timetable) == root + galicja_timetable);
	CHECK(index.listings() == 0);
	CHECK(index.resolve(root + galicja_asked) == root + galicja_timetable);
	CHECK(index.resolve(root + braniewo_asked) == root + braniewo_timetable);
	CHECK(index.resolve(root + "dir\\sub\\file.txt") == root + "Dir/Sub/File.TXT");
	CHECK(index.resolve(root + "DIR/sub/../SUB/./file.txt") == root + "Dir/Sub/../Sub/./File.TXT");
	CHECK(index.listings() == 6);
	CHECK(index.resolve(root + "dir/sub/nothing.txt").empty());
	CHECK(index.resolve(root + "dir/nothing/file.txt").empty());
	CHECK(index.listings() == 6);

	folder.write("Dir/Sub/Added.TXT", "added");
	CHECK(index.resolve(root + "dir/sub/added.txt").empty());
	index.forget(root + "DIR/SUB/");
	CHECK(index.resolve(root + "dir/sub/added.txt") == root + "Dir/Sub/Added.TXT");
	CHECK(index.listings() == 7);

	// taken away: the folders on the way (the scratch folder, Dir, Dir/Sub) listed again, and nothing found
	std::filesystem::remove(folder.root() / "Dir" / "Sub" / "Added.TXT");
	CHECK(index.resolve(root + "dir/sub/added.txt").empty());
	CHECK(index.listings() == 10);
	CHECK(index.resolve(root + "dir/sub/added.txt").empty());
	CHECK(index.listings() == 10);
}

TEST_CASE("cased path: of files on the disk which differ in case only, the one named exactly is chosen") {
	test::scratch_folder const folder{"cased_disk_collision"};
	if (false == case_matters(folder)) {
		// the system can't hold two such files
		return;
	}
	auto const root{folder.path()};
	folder.write("a.txt", "lower");
	folder.write("A.txt", "upper");
	cased::index index;
	CHECK(index.resolve_in_any_case(root + "a.txt") == root + "a.txt");
	CHECK(index.resolve_in_any_case(root + "A.txt") == root + "A.txt");
	CHECK(index.resolve(root + "a.TXT") == root + "A.txt");
	CHECK(index.resolve(root + "a.TXT") == root + "A.txt");
}

#ifndef _WIN32
// on Windows the names of the files are not the bytes the game gives
TEST_CASE("cased path: a folder named outside of ASCII on the disk itself") {
	test::scratch_folder const folder{"cased_disk_utf8"};
	auto const sensitive{case_matters(folder)};
	auto const root{folder.path()};
	std::string const utf8{"\xC5\x81\xC3\xB3" "d\xC5\xBA"};
	folder.write("scenery/" + utf8 + "/Plik.txt", "x");
	cased::index index;
	CHECK(index.resolve(root + "scenery/" + utf8 + "/plik.txt") == root + "scenery/" + utf8 + (sensitive ? "/Plik.txt" : "/plik.txt"));
	CHECK(index.resolve(root + "SCENERY/" + utf8 + "/PLIK.TXT") == root + (sensitive ? "scenery/" : "SCENERY/") + utf8 + (sensitive ? "/Plik.txt" : "/PLIK.TXT"));
}
#endif

TEST_CASE("cased path: many threads look for files on the disk at once") {
	test::scratch_folder const folder{"cased_disk_threads"};
	auto const sensitive{case_matters(folder)};
	auto const root{folder.path()};
	folder.write(galicja_timetable, "galicja");
	folder.write(braniewo_timetable, "braniewo");
	auto const galicja{root + (sensitive ? galicja_timetable : galicja_asked)};
	auto const braniewo{root + (sensitive ? braniewo_timetable : braniewo_asked)};

	cased::index index;
	std::atomic<int> wrong{0};
	std::vector<std::thread> threads;
	for (auto thread = 0; thread < 8; ++thread) {
		threads.emplace_back([&]() {
			for (auto round = 0; round < 50; ++round) {
				if (index.resolve(root + galicja_asked) != galicja || index.resolve(root + braniewo_asked) != braniewo || false == index.resolve(root + "nothing.txt").empty()) {
					++wrong;
				}
			}
		});
	}
	for (auto &thread : threads) {
		thread.join();
	}
	CHECK(wrong.load() == 0);
}

// the game's own lookups --------------------------------------------------------------------------------------------------

TEST_CASE("cased path: the timetable is found the way the game opens it, through FileCased()") {
	test::scratch_folder const folder{"cased_timetable"};
	auto const sensitive{case_matters(folder)};
	folder.write(galicja_timetable, "Rodzaj i numer pociagu");
	folder.write(braniewo_timetable, "TGS3657");

	struct timetable_file {
		char const *timetable; // on the disk
		char const *asked; // by the game
	};
	for (auto const &entry : {timetable_file{galicja_timetable, galicja_asked}, timetable_file{braniewo_timetable, braniewo_asked}}) {
		std::string const timetable{entry.timetable};
		std::string const asked{entry.asked};
		CAPTURE(asked);
		// as world/mtable.cpp makes the name: the folder of the sceneries, the name from the scenery, the extension
		std::string trainname{asked};
		trainname = trainname.substr(std::string{"scenery/"}.size());
		trainname.erase(trainname.size() - 4);
		auto name{folder.path() + "scenery/" + trainname + ".txt"};
		replace_slashes(name);

		std::ifstream file{name};
		// opened as named where the case doesn't matter, through FileCased() where it does
		CHECK(file.is_open() != sensitive);
		auto const cased{FileCased(name)};
		REQUIRE_FALSE(cased.empty());
#ifdef _WIN32
		CHECK(cased == name);
#else
		CHECK(cased == folder.path() + (sensitive ? timetable : asked));
#endif
		std::ifstream found{cased};
		CHECK(found.is_open());

		// the same through cased_path(), which FileCased() uses
		CHECK(cased_path(name) == cased);
	}
	// nothing more to find in either way
	CHECK(FileCased(folder.path("scenery/galicja2000/rozklady/2003/nothing.txt")).empty());
	CHECK(cased_path(folder.path("scenery/galicja2000/rozklady/2003/nothing.txt")).empty());
	cased_path_forget(folder.path("scenery"));
	cased_path_forget_all();
}

// the files of the managers (textures, materials, models, sounds) -------------------------------------------------------

namespace {

// the names a texture is looked for under, as texture_manager::find_on_disk() makes them (model/Texture.cpp)
std::vector<std::string> texture_names(std::string const &Texturepath, std::string const &Name) {
	return {Texturepath + Name, Name, "textures/" + Name};
}
std::vector<std::string> const texture_extensions{".dds", ".tga", ".ktx", ".png", ".bmp", ".jpg", ".tex"};

} // namespace

TEST_CASE("cased path: of the names and extensions, the first one there is given as it was asked for") {
	fake_index fake;
	fake.disk.add("dynamic/PKP/EU07_v1/Skin1.DDS");
	fake.disk.add("dynamic/PKP/EU07_v1/Tor/Szyna.PNG");
	fake.disk.add("textures/Tor/Szyna.TGA");
	fake.disk.add("Models/Lok/EU07.e3d");
	auto const exists{[&fake](std::string const &Path) { return false == fake.index.resolve(Path).empty(); }};

	// the skin of a vehicle, in the folder of the vehicle: the name and the extension asked for, the key of the texture
	auto const skin{cased::first_existing(texture_names("dynamic/pkp/eu07_v1/", "skin1"), texture_extensions, exists)};
	CHECK(skin.first == "dynamic/pkp/eu07_v1/skin1");
	CHECK(skin.second == ".dds");
	// the file opened is the one on the disk
	CHECK(fake.index.path_to_open(skin.first + skin.second) == "dynamic/PKP/EU07_v1/Skin1.DDS");

	// each name with all the extensions before the next name, as on Windows: the .png in the folder of the vehicle comes
	// before the .tga in textures/, though .tga is earlier on the list of extensions
	auto const rail{cased::first_existing(texture_names("dynamic/pkp/eu07_v1/", "tor/szyna"), texture_extensions, exists)};
	CHECK(rail.first == "dynamic/pkp/eu07_v1/tor/szyna");
	CHECK(rail.second == ".png");
	CHECK(fake.index.path_to_open(rail.first + rail.second) == "dynamic/PKP/EU07_v1/Tor/Szyna.PNG");
	auto const common{cased::first_existing(texture_names("", "tor/szyna"), texture_extensions, exists)};
	CHECK(common.first == "textures/tor/szyna");
	CHECK(common.second == ".tga");
	CHECK(fake.index.path_to_open(common.first + common.second) == "textures/Tor/Szyna.TGA");

	// a model, as TModelsManager::find_on_disk() looks for it: the name, then in models/
	auto const model{cased::first_existing({"lok/eu07", "models/lok/eu07"}, {".e3d", ".t3d"}, exists)};
	CHECK(model.first == "models/lok/eu07");
	CHECK(model.second == ".e3d");
	CHECK(fake.index.path_to_open(model.first + model.second) == "Models/Lok/EU07.e3d");

	// nothing there in any case
	auto const none{cased::first_existing(texture_names("dynamic/pkp/eu07_v1/", "nothing"), texture_extensions, exists)};
	CHECK(none.first.empty());
	CHECK(none.second.empty());
	CHECK(cased::first_existing({}, texture_extensions, exists).first.empty());
	CHECK(cased::first_existing({"textures/tor/szyna"}, {}, exists).first.empty());
	// a file which isn't there is opened as named, for the opening to fail on it as it did
	CHECK(fake.index.path_to_open("textures/nothing.dds") == "textures/nothing.dds");
	CHECK(fake.index.path_to_open("").empty());

	// each folder listed once, however many names and extensions were looked for in it
	CHECK(fake.disk.listed("dynamic/PKP/EU07_v1") == 1);
	CHECK(fake.disk.listed("textures") == 1);
	CHECK(fake.disk.listed("textures/Tor") == 1);
}

TEST_CASE("cased path: a file named as it is on the disk is opened as named, its folders not listed") {
	fake_index fake;
	fake.disk.add("textures/tor/szyna.dds");
	auto const exists{[&fake](std::string const &Path) { return false == fake.index.resolve(Path).empty(); }};
	auto const rail{cased::first_existing(texture_names("", "tor/szyna"), texture_extensions, exists)};
	CHECK(rail.first == "textures/tor/szyna");
	CHECK(rail.second == ".dds");
	CHECK(fake.index.path_to_open(rail.first + rail.second) == "textures/tor/szyna.dds");
	// the folder "tor" of the names before isn't in the current folder, which is listed for it, once; the folders of the
	// file there are not
	CHECK(fake.disk.listed(".") == 1);
	CHECK(fake.disk.listed("textures") == 0);
	CHECK(fake.disk.listed("textures/tor") == 0);
	CHECK(fake.index.listings() == 1);
}

TEST_CASE("cased path: FileExists() finds the file in another case under the name asked for, disk_path() opens it") {
	test::scratch_folder const folder{"cased_assets"};
	auto const sensitive{case_matters(folder)};
	CAPTURE(sensitive);
	auto const root{folder.path()};
	folder.write("Dynamic/PKP/EU07_v1/Skin1.DDS", "skin");
	folder.write("Sounds/Horn1.OGG", "horn");
	folder.write("textures/szyna.dds", "rail");
	auto const listings{cased::path_index().listings()};

	// the texture, the way texture_manager::find_on_disk() asks: the name asked for, in lower case, is the key of the texture
	auto const skin{FileExists(texture_names(root + "dynamic/pkp/eu07_v1/", "skin1"), texture_extensions)};
	CHECK(skin.first == root + "dynamic/pkp/eu07_v1/skin1");
	CHECK(skin.second == ".dds");
	// opened at its path on the disk where the case matters, as named where it doesn't
	auto const skinfile{disk_path(skin.first + skin.second)};
	CHECK(skinfile == root + (sensitive ? "Dynamic/PKP/EU07_v1/Skin1.DDS" : "dynamic/pkp/eu07_v1/skin1.dds"));
	std::ifstream skinstream{skinfile, std::ios::binary};
	std::string content;
	std::getline(skinstream, content);
	CHECK(content == "skin");

	// the sound, the way buffer_manager::find_file() asks
	auto const horn{FileExists(std::vector<std::string>{root + "sounds/horn1"}, std::vector<std::string>{".ogg", ".flac", ".wav"})};
	CHECK(horn.first == root + "sounds/horn1");
	CHECK(horn.second == ".ogg");
	CHECK(FileExists(root + "sounds/horn1.ogg"));
	CHECK(cased::path_exists(disk_path(root + "sounds/horn1.ogg")));

	// named as it is: the same name to open
	CHECK(FileExists(root + "textures/szyna.dds"));
	CHECK(disk_path(root + "textures/szyna.dds") == root + "textures/szyna.dds");
	// not there in any case: not found, and opened as named, to fail as it did
	CHECK_FALSE(FileExists(root + "sounds/nothing.ogg"));
	CHECK(FileExists(std::vector<std::string>{root + "sounds/nothing"}, std::vector<std::string>{".ogg", ".wav"}).first.empty());
	CHECK(disk_path(root + "sounds/nothing.ogg") == root + "sounds/nothing.ogg");
	CHECK(disk_path("").empty());

#ifdef _WIN32
	// on Windows the system finds the names in any case: nothing listed, the names to open are the ones given
	CHECK(cased::path_index().listings() == listings);
	CHECK(disk_path(root + "SOUNDS/HORN1.OGG") == root + "SOUNDS/HORN1.OGG");
#else
	static_cast<void>(listings);
#endif
	cased_path_forget_all();
}
