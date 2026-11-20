#include "stdafx.h"
#include "doctest.h"
#include "scratch.h"

#include "utilities/crashsignals.h"

// the stack of a crash on Linux (utilities/crashsignals.h): written to the log and to the standard error, after which
// the signal ends the process as it always did. the crash happens in a child process, which the test watches
#if defined(__linux__)

#include <array>
#include <csignal>
#include <fstream>
#include <iterator>
#include <string>

#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

std::string read_all(int const File) {
	std::string text;
	std::array<char, 4096> buffer{};
	for (auto count{::read(File, buffer.data(), buffer.size())}; count > 0; count = ::read(File, buffer.data(), buffer.size()))
		text.append(buffer.data(), static_cast<std::size_t>(count));
	return text;
}

std::string read_file(std::string const &Path) {
	std::ifstream file{Path, std::ios::binary};
	return {std::istreambuf_iterator<char>{file}, std::istreambuf_iterator<char>{}};
}

// the child: the report goes to the pipe in place of the standard error, no core dump is left behind
[[noreturn]] void crash_in_child(std::array<int, 2> const &Pipe, std::string const &Log) {
	// a child stuck before the crash (a lock some other thread held at the fork) ends anyway, and the test with it
	alarm(10);
	dup2(Pipe[1], STDERR_FILENO);
	close(Pipe[0]);
	close(Pipe[1]);
	rlimit const nocore{0, 0};
	setrlimit(RLIMIT_CORE, &nocore);

	crash_signals::install("test version 1.2");
	crash_signals::set_log_path(Log);
	raise(SIGSEGV);
	_exit(0); // not reached: the signal ends the child
}

} // namespace

TEST_CASE("crash signals: a crash writes its stack to the log and to the standard error, and still ends by its signal") {
	test::scratch_folder const folder{"crash_signals"};
	auto const log{folder.write("log.txt", "the last line of the log\n")};

	std::array<int, 2> pipe_ends{};
	REQUIRE(pipe(pipe_ends.data()) == 0);
	auto const child{fork()};
	REQUIRE(child >= 0);
	if (child == 0)
		crash_in_child(pipe_ends, log);

	close(pipe_ends[1]);
	auto const errors{read_all(pipe_ends[0])};
	close(pipe_ends[0]);
	int status{0};
	REQUIRE(waitpid(child, &status, 0) == child);

	CAPTURE(errors);
	// the process ends as it would without the handler
	CHECK(WIFSIGNALED(status));
	CHECK(WTERMSIG(status) == SIGSEGV);
	// the report on the standard error, which the starter shows
	CHECK(errors.find("Crash: signal 11 (SIGSEGV) at address 0x") != std::string::npos);
	CHECK(errors.find("Version: test version 1.2") != std::string::npos);
	CHECK(errors.find("Stack (addr2line") != std::string::npos);
	// with the frames of the program that crashed
	CHECK(errors.find("eu07_unit_tests") != std::string::npos);
	// and the same at the end of the log, after what it held
	auto const logtext{read_file(log)};
	CAPTURE(logtext);
	CHECK(logtext.starts_with("the last line of the log\n"));
	CHECK(logtext.find("Crash: signal 11 (SIGSEGV)") != std::string::npos);
	CHECK(logtext.find("eu07_unit_tests") != std::string::npos);
}

#endif
