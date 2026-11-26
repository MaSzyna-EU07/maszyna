#pragma once

#include <string>
#include <vector>

namespace test {

// keeps the lines written to the log while it's alive, so a test can tell what was logged (engine_stubs.cpp). one at a time
class log_capture {
  public:
	log_capture();
	~log_capture();
	log_capture(log_capture const &) = delete;
	log_capture &operator=(log_capture const &) = delete;

	// the lines written so far
	std::vector<std::string> lines() const;
};

} // namespace test
