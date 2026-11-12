#include "stdafx.h"
#include "doctest.h"

#include "utilities/logfiles.h"

#include <algorithm>

// the files the log is written to, and which of them a run deletes when it starts

TEST_CASE("log files: without multiplelogs the run writes the shared log.txt and errors.txt")
{
	CHECK(log_file_name(log_file::log, false, "td", "20261112_1200123") == "log.txt");
	CHECK(log_file_name(log_file::errors, false, "td", "20261112_1200123") == "errors.txt");
}

TEST_CASE("log files: with multiplelogs the run writes its own in logs/, named after the scenery and the time")
{
	CHECK(log_file_name(log_file::log, true, "autotest_case", "20261112_1200123") == "logs/log (autotest_case) 20261112_1200123.txt");
	CHECK(log_file_name(log_file::errors, true, "autotest_case", "20261112_1200123") == "logs/errors (autotest_case) 20261112_1200123.txt");
}

TEST_CASE("log files: without multiplelogs the start deletes the shared files it is going to write")
{
	// errors.txt is made at the first error only: one of the run before must not stay behind a run without errors
	auto const stale{stale_log_files(false)};
	CHECK(stale.size() == 2);
	for (auto const kind : {log_file::log, log_file::errors})
		CHECK(std::ranges::count(stale, log_file_name(kind, false, "td", "20261112_1200123")) == 1);
}

TEST_CASE("log files: with multiplelogs the start keeps the shared log.txt and errors.txt of the last ordinary run")
{
	// the scenario tests run with multiplelogs; the logs of the player's last game in the game folder stay
	CHECK(stale_log_files(true).empty());
}
