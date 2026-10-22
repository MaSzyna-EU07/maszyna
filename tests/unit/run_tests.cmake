# runs the unit tests at the end of building the game: cmake -DTESTS=<test binary> -DGAME=<game binary> -P run_tests.cmake
# prints a one line summary when they pass, the full report when they don't. a failure fails the build and
# removes the game binary, so the next build links it again and reruns the tests

execute_process(
	COMMAND "${TESTS}" --no-intro --no-version
	RESULT_VARIABLE result
	OUTPUT_VARIABLE report
	ERROR_VARIABLE report)

string(REGEX MATCH "test cases: *([0-9]+) *\\| *([0-9]+) passed *\\| *([0-9]+) failed" _ "${report}")
set(cases "${CMAKE_MATCH_1}")
set(passed "${CMAKE_MATCH_2}")
set(failed "${CMAKE_MATCH_3}")
string(REGEX MATCH "assertions: *([0-9]+)" _ "${report}")
set(checks "${CMAKE_MATCH_1}")

if(result EQUAL 0)
	message("Unit tests: ${passed}/${cases} passed (${checks} checks)")
else()
	message("${report}")
	file(REMOVE "${GAME}")
	if(cases STREQUAL "")
		message(FATAL_ERROR "Unit tests: didn't run (${result}), see above. -DWITH_TESTS=OFF builds the game without them")
	endif()
	message(FATAL_ERROR "Unit tests: ${failed} of ${cases} FAILED, see above. -DWITH_TESTS=OFF builds the game without them")
endif()
