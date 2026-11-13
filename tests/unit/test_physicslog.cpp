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
	std::string read(std::string const &Name) const {
		std::ifstream file{m_path / (Name + ".dat")};
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
	// Windows doesn't let a file open in a program be removed
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

TEST_CASE("physicslog: what is kept is written when the writer goes") {
	log_folder folder{"end"};
	{
		physicslog_writer logs{folder.path(), std::chrono::hours{1}};
		logs.begin("ST44-001", header);
		logs.append("ST44-001", "0.0500 0.0000\r\n");
	}
	CHECK(folder.read("ST44-001") == std::string{header} + "0.0500 0.0000\r\n");
}
