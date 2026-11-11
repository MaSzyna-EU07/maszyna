# Unit tests of the simulation, built and run without the game data: the test vehicles are JSON
# definitions in data/ (see vehicle.h), loaded from memory by the game's own vehicle loader. The code
# under test is the game's core library (eu07_core), the very objects the game links; the rest of the
# engine it reaches for is replaced with stubs (engine_stubs.cpp). They run as part of the build,
# from the IDE, or with ctest.

set(UNIT_TESTS_NAME eu07_unit_tests)

# a new test_*.cpp file or file in data/ (definitions, real vehicles, golden traces) is picked up on its own
file(GLOB UNIT_TEST_CASES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")
file(GLOB_RECURSE UNIT_TEST_DATA CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/data/*")

add_executable(${UNIT_TESTS_NAME}
	main.cpp
	vehicle.cpp
	fiz.cpp
	drive.cpp
	consist.cpp
	scenario.cpp
	golden.cpp
	invariants.cpp
	figures.cpp
	engine_stubs.cpp
	${UNIT_TEST_CASES})

# the core library brings the game's build settings along
target_link_libraries(${UNIT_TESTS_NAME} PRIVATE eu07_core)
# the script screen host, tested with the real runtimes (test_script_screen_host.cpp)
target_sources(${UNIT_TESTS_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/scripting/scriptscreens.cpp")
if (WITH_LUA)
	target_sources(${UNIT_TESTS_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/scripting/luaprogram.cpp" "${CMAKE_SOURCE_DIR}/stb/stb_image.c")
endif()
if (WITH_WASM)
	target_sources(${UNIT_TESTS_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/scripting/wasmprogram.cpp")
	target_link_libraries(${UNIT_TESTS_NAME} PRIVATE eu07_wamr)
endif()
target_include_directories(${UNIT_TESTS_NAME} PRIVATE "${DEPS_DIR}/doctest")
# the names the tests CAPTURE are often C strings, shown as text rather than pointers
target_compile_definitions(${UNIT_TESTS_NAME} PRIVATE EU07_UNIT_TESTS_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data" DOCTEST_CONFIG_TREAT_CHAR_STAR_AS_STRING)
# the folders of the tests that write files (the scripts of test_script_screen_host.cpp) are made in the build folder,
# and removed by the test that made them
target_compile_definitions(${UNIT_TESTS_NAME} PRIVATE EU07_UNIT_TESTS_SCRATCH="${CMAKE_CURRENT_BINARY_DIR}")
if (MSVC)
	target_compile_options(${UNIT_TESTS_NAME} PRIVATE /bigobj)
endif()
set_target_properties(${UNIT_TESTS_NAME} PROPERTIES FOLDER "tests")

add_test(NAME ${UNIT_TESTS_NAME} COMMAND ${UNIT_TESTS_NAME})
if (WIN32)
	# the libraries of the game (lua51.dll) are next to the game, see run_tests.cmake
	set_tests_properties(${UNIT_TESTS_NAME} PROPERTIES ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:eu07>")
endif()

# for running the tests at the end of building the game, see the main CMakeLists.txt
set(UNIT_TEST_DATA ${UNIT_TEST_DATA} PARENT_SCOPE)
