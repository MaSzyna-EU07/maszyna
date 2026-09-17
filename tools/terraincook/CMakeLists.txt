# Standalone analysis tool for phase 1 of the binary scene format plan. Deliberately
# free of engine dependencies so it configures and builds in seconds, on its own.
cmake_minimum_required(VERSION 3.20)
project(terraincook CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(zstd CONFIG QUIET)
if(NOT zstd_FOUND)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)
	add_library(zstd::libzstd ALIAS PkgConfig::ZSTD)
endif()

find_package(CDT CONFIG REQUIRED)

add_executable(terraincook 
	main.cpp
	../scene/terraincooker_tin.cpp
)

# the on-disk format lives with the engine, so the tool and the reader cannot drift apart
target_include_directories(terraincook PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../..")
target_link_libraries(terraincook PRIVATE 
	zstd::libzstd
	CDT::CDT
)

if(NOT MSVC)
	target_compile_options(terraincook PRIVATE -O2 -Wall)
endif()
