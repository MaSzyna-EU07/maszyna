#include "stdafx.h"
#include "doctest.h"

#include "utilities/physicslog.h"

#include <chrono>
#include <filesystem>
#include <format>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>

// the physics logs are kept and written by a thread of their own every so often, each file opened only to be written to

namespace {

// a folder of its own for the logs of a test, in the build folder of the tests, removed with the object. the writer
// makes it itself
class log_folder {
public:
	explicit log_folder(std::string const &Name) : m_path{std::filesystem::path{EU07_UNIT_TESTS_SCRATCH} / std::format("eu07_physicslog_test_{}", Name)} {
		std::filesystem::remove_all(m_path);
	}
	~log_folder() {
		std::error_code error;
		std::filesystem::remove_all(m_path, error);
	}
	log_folder(log_folder const &) = delete;
	log_folder &operator=(log_folder const &) = delete;

	std::filesystem::path const &path() const { return m_path; }
	// the log as it stands, read in text mode as it is written: what was given to it
	std::string read(std::string const &Name) const { return read_file(m_path / (Name + ".dat")); }
	static std::string read_file(std::filesystem::path const &File) {
		std::ifstream file{File};
		if (false == file.is_open())
			return {};
		std::ostringstream text;
		text << file.rdbuf();
		return text.str();
	}
	// waits up to five seconds for the log to read Text
	bool reads(std::string const &Name, std::string const &Text) const {
		auto const until{std::chrono::steady_clock::now() + std::chrono::seconds{5}};
		while (read(Name) != Text && std::chrono::steady_clock::now() < until)
			std::this_thread::sleep_for(std::chrono::milliseconds{10});
		return read(Name) == Text;
	}

private:
	std::filesystem::path m_path;
};

constexpr auto header{"Time[s] Velocity[m/s]\r\n"};

} // namespace

TEST_CASE("physicslog: a log begun is emptied and given its header, and its lines follow it at each writing") {
	log_folder folder{"begin"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	logs.begin("EU07-424", header);
	logs.append("EU07-424", "0.0500 0.0000\r\n");
	CHECK(folder.read("EU07-424").empty());
	logs.flush();
	CHECK(folder.read("EU07-424") == std::string{header} + "0.0500 0.0000\r\n");

	logs.append("EU07-424", "0.1000 1.5000\r\n");
	logs.flush();
	CHECK(folder.read("EU07-424") == std::string{header} + "0.0500 0.0000\r\n0.1000 1.5000\r\n");

	// begun again: the lines not written yet are dropped along with those written
	logs.append("EU07-424", "0.1500 3.0000\r\n");
	logs.begin("EU07-424", header);
	logs.flush();
	CHECK(folder.read("EU07-424") == header);
}

TEST_CASE("physicslog: no file is kept open between the writings, and a log not begun is added to as it stands") {
	log_folder folder{"closed"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	logs.begin("SM42-001", header);
	logs.flush();
	// Windows doesn't let a file open in a program be removed. Linux does, so there it checks nothing; the log is
	// written again below anyway
	std::error_code error;
	CHECK(std::filesystem::remove(folder.path() / "SM42-001.dat", error));
	CHECK_FALSE(error);

	logs.append("SM42-001", "0.0500 0.0000\r\n");
	logs.flush();
	CHECK(folder.read("SM42-001") == "0.0500 0.0000\r\n");
}

TEST_CASE("physicslog: the thread writes the logs by itself") {
	log_folder folder{"thread"};
	physicslog_writer logs{folder.path(), std::chrono::milliseconds{20}};

	logs.begin("ET22-001", header);
	logs.append("ET22-001", "0.0500 0.0000\r\n");
	CHECK(folder.reads("ET22-001", std::string{header} + "0.0500 0.0000\r\n"));
	logs.append("ET22-001", "0.1000 0.0000\r\n");
	CHECK(folder.reads("ET22-001", std::string{header} + "0.0500 0.0000\r\n0.1000 0.0000\r\n"));
}

TEST_CASE("physicslog: stopping writes what is kept at once, and what comes after waits for a writing") {
	log_folder folder{"stop"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	logs.begin("EN57-1001", header);
	logs.append("EN57-1001", "0.0500 0.0000\r\n");
	logs.stop();
	CHECK(folder.read("EN57-1001") == std::string{header} + "0.0500 0.0000\r\n");

	logs.append("EN57-1001", "0.1000 0.0000\r\n");
	std::this_thread::sleep_for(std::chrono::milliseconds{50});
	CHECK(folder.read("EN57-1001") == std::string{header} + "0.0500 0.0000\r\n");
	logs.flush();
	CHECK(folder.read("EN57-1001") == std::string{header} + "0.0500 0.0000\r\n0.1000 0.0000\r\n");
}

TEST_CASE("physicslog: the lines of a log whose file can't be written are kept, with its header, for the next try") {
	log_folder folder{"failed"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	// a folder in the way of the file: it can't be opened, as one held by another program
	std::filesystem::create_directories(folder.path() / "EU07-005.dat");
	logs.begin("EU07-005", header);
	logs.append("EU07-005", "0.0500 0.0000\r\n");
	logs.flush();
	logs.append("EU07-005", "0.1000 0.5000\r\n");
	logs.flush();
	CHECK(std::filesystem::is_directory(folder.path() / "EU07-005.dat"));

	std::filesystem::remove(folder.path() / "EU07-005.dat");
	logs.append("EU07-005", "0.1500 1.0000\r\n");
	logs.flush();
	CHECK(folder.read("EU07-005") == std::string{header} + "0.0500 0.0000\r\n0.1000 0.5000\r\n0.1500 1.0000\r\n");
}

TEST_CASE("physicslog: a log begun anew while its file can't be written drops what was kept for it") {
	log_folder folder{"failedbegun"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	std::filesystem::create_directories(folder.path() / "SU45-001.dat");
	logs.begin("SU45-001", header);
	logs.append("SU45-001", "0.0500 0.0000\r\n");
	logs.flush();
	std::filesystem::remove(folder.path() / "SU45-001.dat");
	logs.begin("SU45-001", header);
	logs.append("SU45-001", "0.0500 2.0000\r\n");
	logs.flush();
	CHECK(folder.read("SU45-001") == std::string{header} + "0.0500 2.0000\r\n");
}

TEST_CASE("physicslog: a log whose file can't be written doesn't keep the others from theirs") {
	log_folder folder{"failedother"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	std::filesystem::create_directories(folder.path() / "EU07-006.dat");
	logs.begin("EU07-006", header);
	logs.append("EU07-006", "0.0500 0.0000\r\n");
	logs.begin("EU07-007", header);
	logs.append("EU07-007", "0.0500 1.0000\r\n");
	logs.flush();
	CHECK(std::filesystem::is_directory(folder.path() / "EU07-006.dat"));
	CHECK(folder.read("EU07-007") == std::string{header} + "0.0500 1.0000\r\n");
}

TEST_CASE("physicslog: a vehicle named in the code page of its scenery has its log, whatever the code page of the system") {
	log_folder folder{"codepage"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	// "SM42-Łódź" in 1250, as the sceneries have it: not UTF-8, which a Windows set to it would have thrown on
	std::string const name{"SM42-\xA3\xF3" "d\x9F"}; // apart, or the d would be read as a digit of the \xF3
	logs.begin(name, header);
	logs.append(name, "0.0500 0.0000\r\n");
	logs.flush();
	CHECK(logs.file(name).parent_path() == folder.path());
	CHECK(log_folder::read_file(logs.file(name)) == std::string{header} + "0.0500 0.0000\r\n");
}

TEST_CASE("physicslog: what no file name may have on Windows is replaced in the name of the file, on every system") {
	log_folder folder{"filename"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	// a spawned vehicle's name has a ':', which on NTFS would write into a stream of the file "spawn_ezt"
	CHECK(logs.file("spawn_ezt:12").filename() == "spawn_ezt_12.dat");
	CHECK(logs.file("a<b>c\"d/e\\f|g?h*i\tj").filename() == "a_b_c_d_e_f_g_h_i_j.dat");
	CHECK(logs.file("EN57-1001 'a'.2").filename() == "EN57-1001 'a'.2.dat");
	// the devices of Windows, with an extension too, aren't files
	CHECK(logs.file("nul").filename() == "_nul.dat");
	CHECK(logs.file("Com1.2").filename() == "_Com1.2.dat");
	CHECK(logs.file("con ").filename() == "_con .dat");
	CHECK(logs.file("console").filename() == "console.dat");
	CHECK(logs.file("com0").filename() == "com0.dat");
	CHECK(logs.shown("spawn_ezt:12") == folder.path().generic_string() + "/spawn_ezt_12.dat");

	logs.begin("spawn_ezt:12", header);
	logs.append("spawn_ezt:12", "0.0500 0.0000\r\n");
	logs.flush();
	CHECK(folder.read("spawn_ezt_12") == std::string{header} + "0.0500 0.0000\r\n");
	CHECK(false == std::filesystem::exists(folder.path() / "spawn_ezt"));
}

TEST_CASE("physicslog: stopping twice, from two threads at once, writes once and ends") {
	log_folder folder{"stoptwice"};
	physicslog_writer logs{folder.path(), std::chrono::milliseconds{20}};

	logs.begin("EP09-001", header);
	logs.append("EP09-001", "0.0500 0.0000\r\n");
	{
		std::jthread other{[&logs] { logs.stop(); }};
		logs.stop();
	}
	CHECK(folder.read("EP09-001") == std::string{header} + "0.0500 0.0000\r\n");
}

TEST_CASE("physicslog: a flush with a time limit writes, as flush does") {
	log_folder folder{"flushfor"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	logs.begin("ET41-001", header);
	logs.append("ET41-001", "0.0500 0.0000\r\n");
	CHECK(logs.flush_for(std::chrono::seconds{1}));
	CHECK(folder.read("ET41-001") == std::string{header} + "0.0500 0.0000\r\n");
}

TEST_CASE("physicslog: what is kept is written when the writer goes") {
	log_folder folder{"end"};
	{
		physicslog_writer logs{folder.path(), std::chrono::hours{1}};
		logs.begin("ST44-001", header);
		logs.append("ST44-001", "0.0500 0.0000\r\n");
	}
	CHECK(folder.read("ST44-001") == std::string{header} + "0.0500 0.0000\r\n");
}

TEST_CASE("physicslog: a log written by someone else before is added to after what it holds") {
	log_folder folder{"after"};
	std::filesystem::create_directories(folder.path());
	{
		std::ofstream before{folder.path() / "SU45-001.dat"};
		before << "earlier run\n";
	}
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	logs.append("SU45-001", "0.0500 0.0000\r\n");
	logs.flush();
	logs.append("SU45-001", "0.1000 0.0000\r\n");
	logs.flush();
	CHECK(folder.read("SU45-001") == "earlier run\n0.0500 0.0000\r\n0.1000 0.0000\r\n");
}

TEST_CASE("physicslog: the writings go on every interval while they're quick, and less often once they take long") {
	using namespace std::chrono_literals;
	using duration = std::chrono::steady_clock::duration;
	// quick: the interval
	CHECK(physicslog_writer::wait_after(2000ms, duration{0}) == duration{2000ms});
	CHECK(physicslog_writer::wait_after(2000ms, duration{500ms}) == duration{2000ms});
	// a third of the interval: still the interval
	CHECK(physicslog_writer::wait_after(2100ms, duration{700ms}) == duration{2100ms});
	// longer: three times the writing, the disk busy with the logs a quarter of the time
	CHECK(physicslog_writer::wait_after(2000ms, duration{1s}) == duration{3s});
	CHECK(physicslog_writer::wait_after(2000ms, duration{10s}) == duration{30s});
}

TEST_CASE("physicslog: a line given as its values is written as the stream of them always wrote it") {
	log_folder folder{"values"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	physicslog_values line;
	line.time = 0.05;
	line.wheelspeed = 1.23456;
	line.acceleration = -0.5;
	line.mainctrl = 3;
	line.direction = 1;
	line.command = "ShuntVelocity";
	line.value1 = 40.0;
	// a float of the vehicle, widened as the stream widened it
	line.enginetemperature = 50.1f;
	line.vehiclespeed = 36.0;
	logs.begin("SM31-001", header);
	logs.append("SM31-001", line);
	logs.flush();
	CHECK(folder.read("SM31-001") == std::string{header} +
	                                       "0.0500 1.2346 -0.5000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 3 0 0.0000 0.0000 1 "
	                                       "ShuntVelocity 40.0000 0.0000 0 0 50.1000 0.0000 0.0000 0.0000 36.0000\r\n");
}

TEST_CASE("physicslog: lines given as values and as text are written in the order given") {
	log_folder folder{"order"};
	physicslog_writer logs{folder.path(), std::chrono::hours{1}};

	physicslog_values first;
	first.time = 0.05;
	physicslog_values last;
	last.time = 0.15;
	logs.append("EP09-001", first);
	logs.append("EP09-001", "text\r\n");
	logs.append("EP09-001", last);
	logs.flush();
	auto const text{folder.read("EP09-001")};
	CHECK(text.starts_with("0.0500 "));
	CHECK(text.find("\r\ntext\r\n0.1500 ") != std::string::npos);
}
