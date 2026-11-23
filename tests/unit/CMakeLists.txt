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
	editor_stubs.cpp
	# the geometry of the lines the track editor designs, standing on its own
	"${CMAKE_SOURCE_DIR}/editor/editorAlignment.cpp"
	# and the heights of the tracks laid
	"${CMAKE_SOURCE_DIR}/editor/editorGroundFollow.cpp"
	"${CMAKE_SOURCE_DIR}/editor/editorProfile.cpp"
	# and the switches laid into the curves, and the catalogue of the switches the turnouts of the domain are laid from
	"${CMAKE_SOURCE_DIR}/editor/editorSwitchCurve.cpp"
	"${CMAKE_SOURCE_DIR}/editor/editorSwitchDesign.cpp"
	# and the fouling points of the switches, widened in the curves as the gauge is
	"${CMAKE_SOURCE_DIR}/editor/editorGauge.cpp"
	"${CMAKE_SOURCE_DIR}/editor/editorFouling.cpp"
	# and the http client the editor fetches its aids with
	"${CMAKE_SOURCE_DIR}/editor/editorHttp.cpp"
	# and the //$ lines it keeps in the scenery files, and the descriptions of the templates (//$e)
	"${CMAKE_SOURCE_DIR}/editor/editorRecords.cpp"
	"${CMAKE_SOURCE_DIR}/scene/scenelayercontext.cpp"
	"${CMAKE_SOURCE_DIR}/editor/editorIncludeInfo.cpp"
	# the tracks read from the text of the scenery files and written back, with no scene (test_track_text.cpp): the track and
	# what reading and writing it takes, the rest of the engine it reaches for stood in by track_stubs.cpp
	"${CMAKE_SOURCE_DIR}/world/Track.cpp"
	"${CMAKE_SOURCE_DIR}/world/Segment.cpp"
	"${CMAKE_SOURCE_DIR}/world/TrackUneven.cpp"
	"${CMAKE_SOURCE_DIR}/world/MemCell.cpp"
	"${CMAKE_SOURCE_DIR}/scene/scenenode.cpp"
	"${CMAKE_SOURCE_DIR}/utilities/Timer.cpp"
	"${CMAKE_SOURCE_DIR}/rendering/geometrybank.cpp"
	track_stubs.cpp
	# and the cant the track editor matches at the joints of the tracks
	"${CMAKE_SOURCE_DIR}/editor/editorTrackShape.cpp"
	# the norms of the track geometry, a module of its own, and the report of the norms on the tracks laid (test_norms_*.cpp)
	"${CMAKE_SOURCE_DIR}/editor/norms/normsTables.cpp"
	"${CMAKE_SOURCE_DIR}/editor/norms/normsLimits.cpp"
	"${CMAKE_SOURCE_DIR}/editor/norms/normsEngine.cpp"
	"${CMAKE_SOURCE_DIR}/editor/norms/normsLine.cpp"
	"${CMAKE_SOURCE_DIR}/editor/editorNormsReport.cpp"
	# the domain of the track geometry: the elements, the lines and the nodes, the commands editing them, the tracks made of them
	"${CMAKE_SOURCE_DIR}/editor/layout/layoutElements.cpp"
	"${CMAKE_SOURCE_DIR}/editor/layout/layoutLine.cpp"
	"${CMAKE_SOURCE_DIR}/editor/layout/layoutNetwork.cpp"
	"${CMAKE_SOURCE_DIR}/editor/layout/layoutCommands.cpp"
	"${CMAKE_SOURCE_DIR}/editor/layout/layoutTurnout.cpp"
	"${CMAKE_SOURCE_DIR}/editor/layout/layoutGenerator.cpp"
	"${CMAKE_SOURCE_DIR}/editor/layout/layoutConnection.cpp"
	# the project file of the domain, the tracks written kept by their checksums, the //$ lines migrated into it, the scenarios using a map (test_project_*.cpp, test_scenario_index.cpp)
	"${CMAKE_SOURCE_DIR}/editor/project/projectFile.cpp"
	"${CMAKE_SOURCE_DIR}/editor/project/projectGenerated.cpp"
	"${CMAKE_SOURCE_DIR}/editor/project/projectMigration.cpp"
	"${CMAKE_SOURCE_DIR}/editor/project/sceneryText.cpp"
	"${CMAKE_SOURCE_DIR}/editor/project/scenarioIndex.cpp"
	# the project of the map open in the editor, its commands kept as steps of the editor's history (test_project_session.cpp)
	"${CMAKE_SOURCE_DIR}/editor/project/projectSession.cpp"
	${UNIT_TEST_CASES})

# the core library brings the game's build settings along
target_link_libraries(${UNIT_TESTS_NAME} PRIVATE eu07_core)
# the script screen host, tested with the real runtimes (test_script_screen_host.cpp)
target_sources(${UNIT_TESTS_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/scripting/scriptscreens.cpp")
if (WITH_LUA)
	target_sources(${UNIT_TESTS_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/scripting/luaprogram.cpp" "${CMAKE_SOURCE_DIR}/scripting/luasandbox.cpp" "${CMAKE_SOURCE_DIR}/stb/stb_image.c")
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
