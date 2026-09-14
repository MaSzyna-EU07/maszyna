# Unit tests for the plan library (the geometry and the solver behind the track
# layout tool). They are off by default: switched on, the interface units are
# compiled a second time into a small library of their own, because a test
# executable can only import modules from a target it links, and linking the game
# would drag the whole simulator in. The code under test needs nothing but a
# C++20 compiler - no glm, no engine, no imgui.
option(EU07_EDITOR_PLAN_TESTS "Build the editor plan library unit tests" OFF)
if (NOT EU07_EDITOR_PLAN_TESTS)
	return()
endif()

enable_testing()

set(PLAN_INTERFACES
	../plan_segment.cppm
	../plan_turnout_geometry.cppm
	../plan_model.cppm
	../plan_solution.cppm
	../plan_chain.cppm
	../plan_join.cppm
	../plan_layout.cppm
	../plan_render.cppm
	../plan_sketch.cppm
	../plan_tilecache.cppm
	../plan_document_io.cppm
	../plan_scn_export.cppm
	../plan_scene.cppm
	../plan_track_renderer.cppm
	../plan_rail_renderer.cppm)

file(GLOB PLAN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../plan_*.cpp")

add_library(editor_plan STATIC ${PLAN_SOURCES})
target_sources(editor_plan
	PUBLIC FILE_SET CXX_MODULES
	BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/..
	FILES ${PLAN_INTERFACES})
set_target_properties(editor_plan PROPERTIES
	CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF
	POSITION_INDEPENDENT_CODE ON FOLDER "editor")

foreach(name chain hold lines graph format turnout tile_cache split)
	add_executable(editor_plan_test_${name} test_${name}.cpp)
	target_link_libraries(editor_plan_test_${name} PRIVATE editor_plan)
	target_include_directories(editor_plan_test_${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
	set_target_properties(editor_plan_test_${name} PROPERTIES
		CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF FOLDER "editor")
	# the sample plan is a fixture: the format test opens the very file the editor ships
	target_compile_definitions(editor_plan_test_${name}
		PRIVATE EU07_PLAN_SAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../samples")
	add_test(NAME plan_${name} COMMAND editor_plan_test_${name})
endforeach()

# test_scn.cpp and test_join.cpp are missing: they were never committed, along
# with the join module those two exercised (see the stub in plan_join.cpp).
