# fetches the parts of the NVIDIA DLSS SDK (github.com/NVIDIA/DLSS) the Vulkan renderer builds with (WITH_DLSS): the headers, the
# static library of NGX and the DLSS library for the platform. the SDK keeps the binaries of every platform in git (hundreds of MB a
# release), so it isn't a submodule: this takes only the files needed, with a sparse checkout of one tag.
#
#   cmake -P tools/fetch_dlss_sdk.cmake [-DDLSS_SDK_ROOT=<dir>] [-DDLSS_SDK_TAG=<tag>] [-DDLSS_SDK_PLATFORMS=windows;linux] [-DDLSS_SDK_ARCH=x86_64|aarch64]
#
# DLSS_SDK_ROOT: where it goes, ref/DLSS by default (ignored by git), which CMakeLists.txt looks at too. DLSS_SDK_PLATFORMS: the
# platforms whose libraries are taken, the one running this by default. run again to change the tag or the platforms.
# the SDK has its own license (LICENSE.txt in it): the DLSS libraries may go out with the program, the SDK itself may not.

cmake_minimum_required(VERSION 3.16)

if(NOT DLSS_SDK_ROOT)
	set(DLSS_SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../ref/DLSS")
endif()
get_filename_component(DLSS_SDK_ROOT "${DLSS_SDK_ROOT}" ABSOLUTE)
if(NOT DLSS_SDK_TAG)
	set(DLSS_SDK_TAG "v310.9.16")
endif()
if(NOT DLSS_SDK_PLATFORMS)
	if(CMAKE_HOST_WIN32)
		set(DLSS_SDK_PLATFORMS "windows")
	else()
		set(DLSS_SDK_PLATFORMS "linux")
	endif()
endif()

if(NOT DLSS_SDK_ARCH)
	if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
		set(DLSS_SDK_ARCH "aarch64")
	else()
		set(DLSS_SDK_ARCH "x86_64")
	endif()
endif()

find_package(Git REQUIRED)

# the headers, the license, and for each platform the static library of ngx (the variants of the dynamic c runtime the program
# builds with, /MD and /MDd) with the release and the development library of dlss (the latter shows the indicator and takes the
# debug keys, gfx.dlss.developer)
set(patterns "/include/" "/LICENSE.txt")
foreach(platform IN LISTS DLSS_SDK_PLATFORMS)
	if(platform STREQUAL "windows")
		# the static libraries of x86_64 are in x64/, those of arm64 at the top of their directory
		set(static "/lib/Windows_${DLSS_SDK_ARCH}")
		if(DLSS_SDK_ARCH STREQUAL "x86_64")
			set(static "${static}/x64")
		endif()
		list(APPEND patterns
			"${static}/nvsdk_ngx_d.lib"
			"${static}/nvsdk_ngx_d_dbg.lib"
			"/lib/Windows_${DLSS_SDK_ARCH}/rel/nvngx_dlss.dll"
			"/lib/Windows_${DLSS_SDK_ARCH}/dev/nvngx_dlss.dll")
	elseif(platform STREQUAL "linux")
		list(APPEND patterns
			"/lib/Linux_${DLSS_SDK_ARCH}/libnvsdk_ngx.a"
			"/lib/Linux_${DLSS_SDK_ARCH}/rel/libnvidia-ngx-dlss.so.*"
			"/lib/Linux_${DLSS_SDK_ARCH}/dev/libnvidia-ngx-dlss.so.*")
	else()
		message(FATAL_ERROR "unknown platform \"${platform}\" in DLSS_SDK_PLATFORMS, expected windows or linux")
	endif()
endforeach()

function(run_git)
	execute_process(COMMAND "${GIT_EXECUTABLE}" ${ARGN} RESULT_VARIABLE result)
	if(NOT result EQUAL 0)
		message(FATAL_ERROR "git ${ARGN} failed (${result})")
	endif()
endfunction()

if(EXISTS "${DLSS_SDK_ROOT}/.git")
	message(STATUS "updating the DLSS SDK in ${DLSS_SDK_ROOT} to ${DLSS_SDK_TAG}")
	run_git(-C "${DLSS_SDK_ROOT}" fetch --depth 1 --filter=blob:none origin "refs/tags/${DLSS_SDK_TAG}:refs/tags/${DLSS_SDK_TAG}")
	run_git(-C "${DLSS_SDK_ROOT}" sparse-checkout set --no-cone ${patterns})
	run_git(-C "${DLSS_SDK_ROOT}" checkout --detach "${DLSS_SDK_TAG}")
else()
	message(STATUS "fetching the DLSS SDK ${DLSS_SDK_TAG} to ${DLSS_SDK_ROOT}")
	run_git(clone --filter=blob:none --no-checkout --depth 1 --branch "${DLSS_SDK_TAG}" https://github.com/NVIDIA/DLSS.git "${DLSS_SDK_ROOT}")
	run_git(-C "${DLSS_SDK_ROOT}" sparse-checkout set --no-cone ${patterns})
	run_git(-C "${DLSS_SDK_ROOT}" checkout)
endif()
message(STATUS "the DLSS SDK is in ${DLSS_SDK_ROOT}; configure with -DWITH_DLSS=ON")
