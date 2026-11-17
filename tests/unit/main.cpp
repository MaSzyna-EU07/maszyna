#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "stdafx.h"
#include "doctest.h"

#include "vehicle.h"

namespace {

// the simulation draws random numbers (brake delays, pressures, wheel damage) from engines shared by
// everything. reseeding them before every run of a test case, subcases included, gives each test the
// same numbers whether it runs alone from the IDE or among all the others
class deterministic_random : public doctest::IReporter {
public:
	explicit deterministic_random(doctest::ContextOptions const &) {}

	void test_case_start(doctest::TestCaseData const &) override { test::reseed_random(); }
	void test_case_reenter(doctest::TestCaseData const &) override { test::reseed_random(); }

	// the rest of doctest's reporter interface: a listener reports nothing, the console reporter does that
	void report_query(doctest::QueryData const &) override { /* nothing to report */ }
	void test_run_start() override { /* nothing to report */ }
	void test_run_end(doctest::TestRunStats const &) override { /* nothing to report */ }
	void test_case_end(doctest::CurrentTestCaseStats const &) override { /* nothing to report */ }
	void test_case_exception(doctest::TestCaseException const &) override { /* nothing to report */ }
	void subcase_start(doctest::SubcaseSignature const &) override { /* nothing to report */ }
	void subcase_end() override { /* nothing to report */ }
	void log_assert(doctest::AssertData const &) override { /* nothing to report */ }
	void log_message(doctest::MessageData const &) override { /* nothing to report */ }
	void test_case_skipped(doctest::TestCaseData const &) override { /* nothing to report */ }
};

} // namespace

DOCTEST_REGISTER_LISTENER("deterministic_random", 1, deterministic_random);
