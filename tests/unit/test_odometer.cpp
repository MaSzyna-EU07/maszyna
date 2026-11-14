#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "utilities/Globals.h"
#include "vehicle/odometer.h"

#include <filesystem>
#include <fstream>
#include <sstream>

// the odometer of a vehicle, kept between the sessions (eu07/maszyna#72): one state the cab's counters and the
// python screens show, saved in one store, taking over the states the python screens kept in their own files

namespace {

odometer::reading started(double const Km, odometer::origin const Source = odometer::origin::saved) {
	odometer::reading reading;
	reading.begin(Km, Source);
	return reading;
}

odometer::reading run(double const Km, double const Metres) {
	auto reading{started(Km)};
	reading.run(Metres);
	return reading;
}

// a directory of its own for a test, removed afterwards. it's made in the working directory of the tests (the build
// directory), not in the shared temporary one
struct scratch_directory {
	std::filesystem::path path;
	explicit scratch_directory(std::string const &Name) : path{std::filesystem::current_path() / ("eu07_unit_" + Name)} {
		std::filesystem::remove_all(path);
		std::filesystem::create_directories(path);
	}
	scratch_directory(scratch_directory const &) = delete;
	scratch_directory &operator=(scratch_directory const &) = delete;
	~scratch_directory() {
		std::error_code error;
		std::filesystem::remove_all(path, error);
	}
};

void write_file(std::filesystem::path const &Path, std::string const &Content) {
	std::filesystem::create_directories(Path.parent_path());
	std::ofstream{Path, std::ios::binary} << Content;
}

} // namespace

TEST_CASE("odometer: a vehicle counts the distance it runs either way, as the session's counter does") {
	for (auto const velocity : {40.0, -40.0}) {
		CAPTURE(velocity);
		test::vehicle vehicle{test::definition::load("wagon"), velocity};
		vehicle->Odometer.begin(123456.0, odometer::origin::saved);
		vehicle.run(10.0);
		CHECK(vehicle->DistCounter > 0.05);
		CHECK(vehicle->Odometer.km - 123456.0 == doctest::Approx(vehicle->DistCounter));
		CHECK(vehicle->Odometer.start == 123456.0);
		CHECK(vehicle->Odometer.moved());
	}
}

TEST_CASE("odometer: the store reads back what it writes") {
	odometer::store store;
	store.record("EN76-012a", run(1361539.4283, 1000.0));
	store.record("loco with spaces", run(42.0, 500.0));
	CHECK(store.modified());

	std::stringstream file;
	store.write(file);
	odometer::store read;
	CHECK(read.read(file) == 2);
	CHECK(read.find("EN76-012a").value() == doctest::Approx(1361540.4283));
	CHECK(read.find("loco with spaces").value() == doctest::Approx(42.5));
	CHECK_FALSE(read.find("EN76-012").has_value());
	CHECK_FALSE(read.modified());
}

TEST_CASE("odometer: the store skips the comments and the malformed lines") {
	std::stringstream file{"# comment\r\n"
	                       "100.5 good\r\n"
	                       "\n"
	                       "nonsense\n"
	                       "abc name\n"
	                       "-5 negative\n"
	                       "nan notanumber\n"
	                       "7 \n"
	                       "200\ttabbed\n"};
	odometer::store store;
	CHECK(store.read(file) == 2);
	CHECK(store.find("good").value() == 100.5);
	CHECK(store.find("tabbed").value() == 200.0);
	CHECK(store.size() == 2);
}

TEST_CASE("odometer: a vehicle gets in the store once it has run, then it's kept up to date") {
	odometer::store store;
	SUBCASE("a vehicle which only stood doesn't get in") {
		store.record("standing", started(5000.0, odometer::origin::guessed));
		CHECK_FALSE(store.find("standing").has_value());
		CHECK_FALSE(store.modified());
	}
	SUBCASE("one which ran does") {
		store.record("running", run(5000.0, 250.0));
		CHECK(store.find("running").value() == doctest::Approx(5000.25));
		CHECK(store.modified());
	}
	SUBCASE("one already in the store follows its state, and an unchanged one doesn't make the store modified") {
		std::stringstream file{"300 known\n"};
		store.read(file);
		store.record("known", started(300.0));
		CHECK_FALSE(store.modified());
		store.record("known", run(300.0, 1000.0));
		CHECK(store.find("known").value() == doctest::Approx(301.0));
		CHECK(store.modified());
		store.saved();
		CHECK_FALSE(store.modified());
	}
	SUBCASE("a nameless vehicle doesn't") {
		store.record("", run(1.0, 1000.0));
		CHECK(store.size() == 0);
	}
}

TEST_CASE("odometer: the state the python screens kept is read") {
	auto const legacy = [](std::string const &Content) {
		std::stringstream file{Content};
		return odometer::read_legacy(file);
	};
	// written as python's str() of a float
	CHECK(legacy("1361539.4283109533").value() == doctest::Approx(1361539.4283109533));
	CHECK(legacy("170650.0\n").value() == 170650.0);
	CHECK(legacy("  5000 ").value() == 5000.0);
	CHECK_FALSE(legacy("").has_value());
	CHECK_FALSE(legacy("12 km").has_value());
	CHECK_FALSE(legacy("inf").has_value());
}

TEST_CASE("odometer: the state starts from the store, else the python screens' file, else a guess") {
	scratch_directory directory{"odometer_legacy"};
	// the python screens dropped the last letter of the vehicle name: the units of the EN76 shared the file
	write_file(directory.path / "kilometry" / "EN76-012_przebieg.txt", "1361539.5");
	std::stringstream file{"2000 EN76-012b\n"};
	odometer::store store;
	store.read(file);

	auto const saved{odometer::starting_state(store, "EN76-012b", directory.path)};
	CHECK(saved.source == odometer::origin::saved);
	CHECK(saved.km == 2000.0);

	auto const legacy{odometer::starting_state(store, "EN76-012a", directory.path)};
	CHECK(legacy.source == odometer::origin::legacy);
	CHECK(legacy.km == 1361539.5);
	CHECK(legacy.start == legacy.km);

	auto const guessed{odometer::starting_state(store, "SM42-001", directory.path)};
	CHECK(guessed.source == odometer::origin::guessed);
	CHECK(guessed.km == odometer::guessed_state("SM42-001"));
	CHECK_FALSE(guessed.moved());

	// an unreadable legacy file is no state
	write_file(directory.path / "kilometry" / "ET22-100_przebieg.txt", "");
	CHECK(odometer::starting_state(store, "ET22-100a", directory.path).source == odometer::origin::guessed);
}

TEST_CASE("odometer: a guessed state is the same for the same vehicle, in range") {
	for (auto const *name : {"", "a", "EN57-1001ra", "ET22-100", "SM42-001"}) {
		CAPTURE(name);
		auto const state{odometer::guessed_state(name)};
		CHECK(state == odometer::guessed_state(name));
		CHECK(state >= 10000.0);
		CHECK(state < 500000.0);
	}
	CHECK(odometer::guessed_state("ET22-100") != odometer::guessed_state("ET22-101"));
}

TEST_CASE("odometer: the cab's counter offset replaces only a guessed state, the distance run kept") {
	CHECK(odometer::counter_seed(101532.f, 1.f).value() == 101532.0);
	CHECK(odometer::counter_seed(1015320.f, 10.f).value() == doctest::Approx(101532.0));
	CHECK_FALSE(odometer::counter_seed(0.f, 1.f).has_value());
	CHECK_FALSE(odometer::counter_seed(100.f, 0.f).has_value());
	CHECK_FALSE(odometer::counter_seed(-100.f, 1.f).has_value());

	SUBCASE("guessed") {
		auto reading{started(33333.0, odometer::origin::guessed)};
		reading.run(2000.0);
		reading.seed(101532.0);
		CHECK(reading.source == odometer::origin::counter);
		CHECK(reading.start == 101532.0);
		CHECK(reading.km == doctest::Approx(101534.0));
		// the next cab load doesn't seed it again
		reading.seed(5.0);
		CHECK(reading.start == 101532.0);
	}
	for (auto const source : {odometer::origin::saved, odometer::origin::legacy}) {
		CAPTURE(static_cast<int>(source));
		auto reading{started(777.0, source)};
		reading.seed(101532.0);
		CHECK(reading.km == 777.0);
		CHECK(reading.source == source);
	}
}

TEST_CASE("odometer: a state counted from a made up one is saved as such, the cab's counter still replaces it") {
	scratch_directory directory{"odometer_guessed"};
	auto const guess{odometer::guessed_state("EU07-424")};
	// the vehicle runs 30 km with no cab loaded (the AI's, loose stock)
	odometer::store first;
	auto ai{odometer::starting_state(first, "EU07-424", directory.path)};
	REQUIRE(ai.source == odometer::origin::guessed);
	ai.run(30000.0);
	first.record("EU07-424", ai);
	std::stringstream file;
	first.write(file);
	CHECK(file.str().find("\n~") != std::string::npos);

	odometer::store second;
	REQUIRE(second.read(file) == 1);
	CHECK(second.guessed("EU07-424"));
	CHECK(second.find("EU07-424").value() == doctest::Approx(guess + 30.0));
	auto player{odometer::starting_state(second, "EU07-424", directory.path)};
	CHECK(player.source == odometer::origin::guessed);
	CHECK(player.km == doctest::Approx(guess + 30.0));

	SUBCASE("a cab whose counter has an offset loads it: the offset replaces the made up state, the 30 km kept") {
		player.run(2000.0);
		player.seed(101532.0);
		CHECK(player.source == odometer::origin::counter);
		CHECK(player.start == doctest::Approx(101562.0));
		CHECK(player.km == doctest::Approx(101564.0));
		// recorded as known, also when the counter shows less than the made up state did
		second.record("EU07-424", player);
		CHECK_FALSE(second.guessed("EU07-424"));
		CHECK(second.find("EU07-424").value() == doctest::Approx(101564.0));
		odometer::reading low;
		low.begin(10.0, odometer::origin::counter);
		odometer::store replaced;
		std::stringstream guessedfile{"~400000 SM42-001\n"};
		replaced.read(guessedfile);
		replaced.record("SM42-001", low);
		CHECK(replaced.find("SM42-001").value() == 10.0);
		CHECK_FALSE(replaced.guessed("SM42-001"));
		CHECK(replaced.modified());
	}
	SUBCASE("it runs with no cab again: it stays guessed") {
		player.run(1000.0);
		second.record("EU07-424", player);
		CHECK(second.guessed("EU07-424"));
		CHECK(second.find("EU07-424").value() == doctest::Approx(guess + 31.0));
	}
}

TEST_CASE("odometer: the store reads the marker of a made up state, and skips it malformed") {
	std::stringstream file{"~300 guessed one\n"
	                       "~ nothing\n"
	                       "~abc name\n"
	                       "~-5 negative\n"
	                       "~~5 twice\n"
	                       "400 known\n"};
	odometer::store store;
	CHECK(store.read(file) == 2);
	CHECK(store.find("guessed one").value() == 300.0);
	CHECK(store.guessed("guessed one"));
	CHECK_FALSE(store.guessed("known"));
	CHECK_FALSE(store.guessed("missing"));
}

TEST_CASE("odometer: a recorded state is replaced only by a newer one") {
	std::stringstream file{"500 known\n~600 guessed\n"};
	odometer::store store;
	store.read(file);
	// a vehicle of this simulation counting from less than another simulation saved meanwhile
	store.record("known", run(400.0, 50000.0));
	CHECK(store.find("known").value() == 500.0);
	CHECK_FALSE(store.modified());
	// a larger one does, also when the vehicle hasn't run in this session
	store.record("known", run(900.0, 0.0));
	CHECK(store.find("known").value() == 900.0);
	CHECK(store.modified());
	// a guessed state doesn't replace a known one, even a larger one
	odometer::reading guessed;
	guessed.begin(5000.0, odometer::origin::guessed);
	guessed.run(1000.0);
	store.record("known", guessed);
	CHECK(store.find("known").value() == 900.0);
	CHECK_FALSE(store.guessed("known"));
	// a larger guessed one replaces a guessed one
	store.record("guessed", guessed);
	CHECK(store.find("guessed").value() == doctest::Approx(5001.0));
	CHECK(store.guessed("guessed"));
}

TEST_CASE("odometer: two simulations saving one file keep each other's states") {
	scratch_directory directory{"odometer_merge"};
	auto const path{directory.path / "eu07_odometers.txt"};
	write_file(path, "1000 shared\n~20000 both guessed\n50 untouched\n");
	auto const load = [&path]() {
		odometer::store store;
		std::ifstream input{path, std::ios::binary};
		store.read(input);
		return store;
	};
	auto first{load()};
	auto second{load()};
	auto const guessed = [](double const Km, double const Metres) {
		odometer::reading reading;
		reading.begin(Km, odometer::origin::guessed);
		reading.run(Metres);
		return reading;
	};

	// both run the shared vehicle, as two multiplayer simulations of the same train do, the first one further
	first.record("shared", run(1000.0, 5000.0));
	first.record("first only", run(70.0, 1000.0));
	first.record("both guessed", guessed(20000.0, 3000.0));
	second.record("shared", run(1000.0, 2000.0));
	second.record("second only", run(80.0, 1000.0));
	second.record("both guessed", guessed(20000.0, 1000.0));

	REQUIRE(odometer::save(first, path));
	CHECK_FALSE(first.modified());
	REQUIRE(odometer::save(second, path));

	auto const saved{load()};
	CHECK(saved.size() == 5);
	// the larger state of the shared vehicle, not the sum of the two: they counted the same run
	CHECK(saved.find("shared").value() == doctest::Approx(1005.0));
	CHECK(saved.find("first only").value() == doctest::Approx(71.0));
	CHECK(saved.find("second only").value() == doctest::Approx(81.0));
	CHECK(saved.find("both guessed").value() == doctest::Approx(20003.0));
	CHECK(saved.guessed("both guessed"));
	CHECK(saved.find("untouched").value() == 50.0);
	// the second one now knows the states the first one saved, for the vehicles it loads later
	CHECK(second.find("first only").value() == doctest::Approx(71.0));

	SUBCASE("a known state saved by one replaces a guessed one of the other") {
		odometer::reading seeded;
		seeded.begin(100.0, odometer::origin::counter);
		first.record("both guessed", seeded);
		REQUIRE(odometer::save(first, path));
		second.record("both guessed", guessed(20003.0, 5000.0));
		REQUIRE(odometer::save(second, path));
		auto const after{load()};
		CHECK(after.find("both guessed").value() == 100.0);
		CHECK_FALSE(after.guessed("both guessed"));
		CHECK(after.find("second only").value() == doctest::Approx(81.0));
	}
	SUBCASE("nothing but the file is left in the directory") {
		std::size_t files{0};
		for (auto const &entry : std::filesystem::directory_iterator{directory.path}) {
			CAPTURE(entry.path().string());
			CHECK(entry.path().filename() == path.filename());
			++files;
		}
		CHECK(files == 1);
	}
	SUBCASE("a store with nothing recorded since doesn't write, once the file showed what it saved") {
		REQUIRE(odometer::save(second, path));
		write_file(path, "1 replaced by hand\n");
		REQUIRE(odometer::save(second, path));
		CHECK(load().size() == 1);
	}
}

namespace {

odometer::store load_store(std::filesystem::path const &Path) {
	odometer::store store;
	std::ifstream input{Path, std::ios::binary};
	store.read(input);
	return store;
}

// a comment line added to the file; a save that writes the file drops it
void mark(std::filesystem::path const &Path) {
	std::ofstream{Path, std::ios::binary | std::ios::app} << "# not written since\n";
}

bool marked(std::filesystem::path const &Path) {
	std::ifstream input{Path, std::ios::binary};
	std::stringstream content;
	content << input.rdbuf();
	return content.str().find("# not written since") != std::string::npos;
}

// the save of Late reads the file before the save of Early writes it, and puts its own file in place after: Late
// writes the file as it was before Early's save, Early's states are gone from it
void interleave(odometer::store &Early, odometer::store &Late, std::filesystem::path const &Path) {
	auto const view{Path.parent_path() / "late_view" / Path.filename()};
	std::filesystem::create_directories(view.parent_path());
	std::filesystem::copy_file(Path, view, std::filesystem::copy_options::overwrite_existing);
	REQUIRE(odometer::save(Early, Path));
	REQUIRE(odometer::save(Late, view));
	std::filesystem::rename(view, Path);
}

} // namespace

TEST_CASE("odometer: a save another simulation's save put its file over is written again") {
	scratch_directory directory{"odometer_lost_save"};
	auto const path{directory.path / "eu07_odometers.txt"};
	write_file(path, "1000 shared\n");
	auto first{load_store(path)};
	auto second{load_store(path)};
	first.record("first only", run(70.0, 1000.0));
	second.record("second only", run(80.0, 1000.0));

	interleave(second, first, path);
	// the race of the two saves: the second one's state is gone from the file
	REQUIRE_FALSE(load_store(path).find("second only").has_value());
	REQUIRE(load_store(path).find("first only").has_value());

	// the second one's next save, nothing recorded since, finds the file without its state and writes it again
	CHECK_FALSE(second.modified());
	REQUIRE(odometer::save(second, path));
	auto const saved{load_store(path)};
	CHECK(saved.size() == 3);
	CHECK(saved.find("second only").value_or(0.0) == doctest::Approx(81.0));
	CHECK(saved.find("first only").value_or(0.0) == doctest::Approx(71.0));
	CHECK(saved.find("shared").value_or(0.0) == 1000.0);

	// both find their states in the file now: their next saves don't write it
	mark(path);
	for (auto round{0}; round < 3; ++round) {
		REQUIRE(odometer::save(first, path));
		REQUIRE(odometer::save(second, path));
	}
	CHECK(marked(path));
	CHECK(load_store(path).size() == 3);
}

TEST_CASE("odometer: two simulations with different states of a vehicle settle on the newer one") {
	scratch_directory directory{"odometer_lost_save_conflict"};
	auto const path{directory.path / "eu07_odometers.txt"};
	write_file(path, "1000 shared\n~20000 guessed by both\n");
	auto first{load_store(path)};
	auto second{load_store(path)};
	// the first one ran the shared vehicle further, and the made up one further from its made up state; the second
	// one has a known state of the made up one, from a cab's counter
	first.record("shared", run(1000.0, 10000.0));
	second.record("shared", run(1000.0, 5000.0));
	odometer::reading guessed;
	guessed.begin(20000.0, odometer::origin::guessed);
	guessed.run(9000.0);
	first.record("guessed by both", guessed);
	odometer::reading seeded;
	seeded.begin(300.0, odometer::origin::counter);
	second.record("guessed by both", seeded);

	// the second one's save is lost to the first one's, which holds the made up state
	interleave(second, first, path);
	REQUIRE(load_store(path).guessed("guessed by both"));

	// the second one writes its known state again. the first one takes it, and the second one the larger state of
	// the shared vehicle the first one saved
	REQUIRE(odometer::save(second, path));
	REQUIRE(odometer::save(first, path));
	auto const settled{load_store(path)};
	CHECK(settled.find("shared").value_or(0.0) == doctest::Approx(1010.0));
	CHECK(settled.find("guessed by both").value_or(0.0) == 300.0);
	CHECK_FALSE(settled.guessed("guessed by both"));
	CHECK(first.find("guessed by both").value_or(0.0) == 300.0);
	CHECK(second.find("shared").value_or(0.0) == doctest::Approx(1010.0));

	// neither one keeps writing the file over to have its own state there
	mark(path);
	for (auto round{0}; round < 3; ++round) {
		REQUIRE(odometer::save(first, path));
		REQUIRE(odometer::save(second, path));
	}
	CHECK(marked(path));
}

TEST_CASE("odometer: a save with nothing recorded since doesn't write the file again") {
	scratch_directory directory{"odometer_saved_once"};
	auto const path{directory.path / "eu07_odometers.txt"};
	odometer::store store;
	// states the file holds rounded to its digits
	store.record("known", run(1000.0, 1234.56789));
	odometer::reading guessed;
	guessed.begin(12345.678912, odometer::origin::guessed);
	guessed.run(98.76543);
	store.record("guessed", guessed);
	REQUIRE(odometer::save(store, path));
	CHECK_FALSE(store.modified());

	// the next save finds the file holding the states, rounded as they are: it doesn't write it
	mark(path);
	REQUIRE(odometer::save(store, path));
	CHECK(marked(path));
	// and the saves after it don't look at the file any more
	std::filesystem::remove(path);
	REQUIRE(odometer::save(store, path));
	CHECK_FALSE(std::filesystem::exists(path));
}

TEST_CASE("odometer: a test run neither reads nor saves the player's store") {
	if (false == Global.test.enabled)
		CHECK(odometer::persistent());
	// the autotests (--test) run the vehicles of the player's game, their runs aren't the player's
	struct test_mode {
		bool const was{Global.test.enabled};
		test_mode() { Global.test.enabled = true; }
		test_mode(test_mode const &) = delete;
		test_mode &operator=(test_mode const &) = delete;
		~test_mode() { Global.test.enabled = was; }
	} const testmode;
	// stops before the player's file is touched, if the gate is gone
	REQUIRE_FALSE(odometer::persistent());

	auto &states{odometer::states()};
	states.record("tested", run(1000.0, 5000.0));
	CHECK(states.find("tested").value() == doctest::Approx(1005.0));
	CHECK(odometer::save());
	// a save to the file would have cleared it
	CHECK(states.modified());
}

TEST_CASE("odometer: the vehicles with a driver's cab are the ones with the driver's brake valve") {
	// the definitions of the game: a locomotive, cab cars of units, a motor car of a unit without a cab, the wagons
	struct expected {
		char const *path;
		bool cab;
	};
	for (auto const &[path, cab] : {expected{"4e_v1/4e", true}, expected{"sm42_v1/6d1", true}, expected{"en57_v1/5bgb", true},
	                                expected{"21xm_v1/214ma", true}, expected{"impuls_v1/31we-a_pr", true},
	                                expected{"ed72_v1/6bs-sa", false}, expected{"impuls_v1/31we-b_pr", false},
	                                expected{"11xa_v2/93a", false}, expected{"bdhpumn_v3/bdhpumn", false},
	                                expected{"430w_v2/430w", false}}) {
		CAPTURE(path);
		test::vehicle vehicle{test::definition::fiz(path)};
		CHECK(vehicle->HasDriverCab() == cab);
	}
	test::vehicle wagon{test::definition::load("wagon")};
	CHECK_FALSE(wagon->HasDriverCab());
	test::vehicle locomotive{test::definition::load("wagon").set("Cntrl", "BrakeHandle", "FV4a")};
	CHECK(locomotive->HasDriverCab());
}

TEST_CASE("odometer: only the vehicles with a driver's cab are recorded") {
	odometer::store store;
	SUBCASE("a wagon which ran doesn't get in") {
		odometer::record(store, "wagon", false, run(5000.0, 1000.0));
		CHECK_FALSE(store.find("wagon").has_value());
		CHECK_FALSE(store.modified());
	}
	SUBCASE("a locomotive the AI ran before any cab loaded it gets in with its made up state, still replaceable") {
		odometer::reading ai;
		ai.begin(odometer::guessed_state("ET22-100"), odometer::origin::guessed);
		ai.run(2000.0);
		odometer::record(store, "ET22-100", true, ai);
		CHECK(store.find("ET22-100").value() == doctest::Approx(ai.km));
		CHECK(store.guessed("ET22-100"));
		CHECK(store.modified());
	}
	SUBCASE("the state of a wagon already in the file stays as it was, and is written back") {
		std::stringstream file{"300 old wagon\n"};
		store.read(file);
		odometer::record(store, "old wagon", false, run(300.0, 1000.0));
		CHECK(store.find("old wagon").value() == 300.0);
		CHECK_FALSE(store.modified());
		std::stringstream written;
		store.write(written);
		odometer::store read;
		CHECK(read.read(written) == 1);
		CHECK(read.find("old wagon").value() == 300.0);
	}
}

TEST_CASE("odometer: a vehicle driven without a driver's brake valve is recorded once a cab shows its odometer") {
	// the draisines (DL2, DR2) have a cab and no BrakeHandle, so their type doesn't tell of the cab; the cab of the
	// DL2 has a counter with an offset of 85
	auto const definition{test::definition::fiz("dr2_v1/dr2")};
	test::vehicle draisine{definition};
	REQUIRE_FALSE(draisine->HasDriverCab());
	scratch_directory directory{"odometer_cab_without_brake_valve"};
	auto const file{directory.path / "eu07_odometers.txt"};
	std::string const name{"DL2-001"};
	// the state a vehicle of the name starts a session with, nothing kept by the python screens
	auto const start = [&](odometer::store const &States) { return odometer::starting_state(States, name, directory.path, directory.path); };
	odometer::store store;
	draisine->Odometer = start(store);
	REQUIRE(draisine->Odometer.source == odometer::origin::guessed);

	SUBCASE("run by the AI, no cab loaded and no counter bound, it isn't recorded") {
		draisine->Odometer.run(1500.0);
		CHECK_FALSE(odometer::kept(draisine->HasDriverCab(), draisine->Odometer));
		odometer::record(store, name, draisine->HasDriverCab(), draisine->Odometer);
		CHECK_FALSE(store.find(name).has_value());
		CHECK_FALSE(store.modified());
	}
	SUBCASE("its state seeded by the cab's counter is saved, read back, and counted on in the next session") {
		// the cab loads: the counter seeds the state (TTrain, distcounter:)
		draisine->Odometer.seed(odometer::counter_seed(85.f, 1.f).value());
		REQUIRE(draisine->Odometer.source == odometer::origin::counter);
		draisine->Odometer.run(2500.0);
		CHECK(odometer::kept(draisine->HasDriverCab(), draisine->Odometer));
		odometer::record(store, name, draisine->HasDriverCab(), draisine->Odometer);
		CHECK(store.find(name).value() == doctest::Approx(87.5));
		CHECK_FALSE(store.guessed(name));
		REQUIRE(odometer::save(store, file));

		// the next session: the vehicle starts from the saved state, not from the offset of the counter
		odometer::store next;
		{
			std::ifstream input{file, std::ios::binary};
			CHECK(next.read(input) == 1);
		}
		test::vehicle again{definition};
		again->Odometer = start(next);
		CHECK(again->Odometer.km == doctest::Approx(87.5));
		CHECK(again->Odometer.source == odometer::origin::saved);
		// the cab loads again: the counter has nothing to seed, a saved state isn't the counter's any more. the
		// counter bound to the vehicle is what tells of its cab now
		again->Odometer.seed(odometer::counter_seed(85.f, 1.f).value());
		CHECK(again->Odometer.km == doctest::Approx(87.5));
		CHECK(again->Odometer.source == odometer::origin::saved);
		again->Odometer.shown = true;
		again->Odometer.run(1000.0);
		CHECK(odometer::kept(again->HasDriverCab(), again->Odometer));
		odometer::record(next, name, again->HasDriverCab(), again->Odometer);
		CHECK(next.find(name).value() == doctest::Approx(88.5));
		CHECK(next.modified());
		REQUIRE(odometer::save(next, file));

		// a session the AI runs it in, with no cab loaded: the saved state stays as it is
		test::vehicle ai{definition};
		ai->Odometer = start(next);
		CHECK(ai->Odometer.km == doctest::Approx(88.5));
		ai->Odometer.run(4000.0);
		odometer::record(next, name, ai->HasDriverCab(), ai->Odometer);
		CHECK(next.find(name).value() == doctest::Approx(88.5));
		CHECK_FALSE(next.modified());
	}
	SUBCASE("its cab loaded for a player, with no counter in it, it's recorded with its made up state") {
		draisine->Odometer.shown = true;
		draisine->Odometer.run(1500.0);
		odometer::record(store, name, draisine->HasDriverCab(), draisine->Odometer);
		CHECK(store.find(name).value() == doctest::Approx(draisine->Odometer.km));
		CHECK(store.guessed(name));
	}
	SUBCASE("a state the session starts with doesn't carry the cab of an earlier one") {
		draisine->Odometer.shown = true;
		draisine->Odometer = start(store);
		CHECK_FALSE(draisine->Odometer.shown);
	}
}

TEST_CASE("odometer: the files of both kinds of python screens are read, by the name each used") {
	scratch_directory directory{"odometer_legacy_screens"};
	auto const vehicles{directory.path / "dynamic" / "elf_v1"};
	auto const scripts{directory.path / "scripts"};
	// the renderers of the units, next to the .mmd: state['name'][:-1], the cab cars of a unit share the file
	write_file(vehicles / "kilometry" / "EN76-022_przebieg.txt", "250000.5");
	CHECK(odometer::legacy_state(vehicles, "EN76-022a", scripts).value() == 250000.5);
	CHECK(odometer::legacy_state(vehicles, "EN76-022d", scripts).value() == 250000.5);
	// the shared screens of scripts/ (koliber_*): the whole name
	write_file(scripts / "kilometry" / "EU07-424_przebieg.txt", "123456.0");
	CHECK(odometer::legacy_state(vehicles, "EU07-424", scripts).value() == 123456.0);
	auto const started{odometer::starting_state(odometer::store{}, "EU07-424", vehicles, scripts)};
	CHECK(started.source == odometer::origin::legacy);
	CHECK(started.km == 123456.0);
	// neither
	CHECK_FALSE(odometer::legacy_state(vehicles, "EU07-425", scripts).has_value());
	CHECK_FALSE(odometer::legacy_state(vehicles, "", scripts).has_value());
}
