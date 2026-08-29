cmake_minimum_required(VERSION 3.20)

project(maj0sted
    VERSION 0.1.0
    DESCRIPTION "Model and solver for a railway track-layout editor"
    LANGUAGES CXX)

# ----------------------------------------------------------------------------
# Options
# ----------------------------------------------------------------------------
option(MAJ0STED_BUILD_TESTS        "Build unit tests"          ON)
option(MAJ0STED_WARNINGS_AS_ERRORS "Treat warnings as errors"  OFF)

# Only drive tests/examples/install defaults when built on its own, not when
# pulled in via add_subdirectory() from a larger project.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(MAJ0STED_STANDALONE ON)
else()
    set(MAJ0STED_STANDALONE OFF)
endif()

# ----------------------------------------------------------------------------
# The static library
# ----------------------------------------------------------------------------
add_library(maj0sted STATIC
    src/domain/turnout.cpp
    src/editor/model.cpp
    src/editor/solution.cpp
    src/editor/chain.cpp
    src/editor/sketch.cpp
    src/editor/layout.cpp
    src/editor/render.cpp
    src/editor/tile_cache.cpp
    src/render/rail_renderer.cpp
    src/io/document_io.cpp
    src/io/scn_export.cpp
)
add_library(maj0sted::maj0sted ALIAS maj0sted)

target_compile_features(maj0sted PUBLIC cxx_std_20)

target_include_directories(maj0sted PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(MSVC)
    target_compile_options(maj0sted PRIVATE /W4)
else()
    target_compile_options(maj0sted PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

if(MAJ0STED_WARNINGS_AS_ERRORS)
    if(MSVC)
        target_compile_options(maj0sted PRIVATE /WX)
    else()
        target_compile_options(maj0sted PRIVATE -Werror)
    endif()
endif()

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
if(MAJ0STED_BUILD_TESTS AND MAJ0STED_STANDALONE)
    enable_testing()
    add_subdirectory(tests)
endif()

# ----------------------------------------------------------------------------
# Install / package
# ----------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS maj0sted
    EXPORT maj0stedTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT maj0stedTargets
    FILE maj0stedTargets.cmake
    NAMESPACE maj0sted::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/maj0sted)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/maj0stedConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/maj0stedConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/maj0sted)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/maj0stedConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/maj0stedConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/maj0stedConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/maj0sted)
