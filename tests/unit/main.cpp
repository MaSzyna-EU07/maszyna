#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "stdafx.h"
#include "doctest.h"

#include "utilities/Globals.h"

namespace {

// the simulation draws random numbers (brake delays, pressures, wheel damage) from engines shared by
// everything. reseeding them before every run of a test case, subcases included, gives each test the
// same numbers whether it runs alone from the IDE or among all the others
class deterministic_random : public doctest::IReporter {
public:
	explicit deterministic_random(doctest::ContextOptions const &) {}

	void test_case_start(doctest::TestCaseData const &) override { reseed(); }
	void test_case_reenter(doctest::TestCaseData const &) override { reseed(); }

	void report_query(doctest::QueryData const &) override {}
	void test_run_start() override {}
	void test_run_end(doctest::TestRunStats const &) override {}
	void test_case_end(doctest::CurrentTestCaseStats const &) override {}
	void test_case_exception(doctest::TestCaseException const &) override {}
	void subcase_start(doctest::SubcaseSignature const &) override {}
	void subcase_end() override {}
	void log_assert(doctest::AssertData const &) override {}
	void log_message(doctest::MessageData const &) override {}
	void test_case_skipped(doctest::TestCaseData const &) override {}

private:
	static void reseed() {
		Global.random_engine.seed(1);
		Global.local_random_engine.seed(1);
	}
};

} // namespace

DOCTEST_REGISTER_LISTENER("deterministic_random", 1, deterministic_random);
