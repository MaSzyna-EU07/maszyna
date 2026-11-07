#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
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

// a directory of its own for a test, removed afterwards
struct scratch_directory {
	std::filesystem::path path;
	explicit scratch_directory(std::string const &Name) : path{std::filesystem::temp_directory_path() / ("eu07_unit_" + Name)} {
		std::filesystem::remove_all(path);
		std::filesystem::create_directories(path);
	}
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
