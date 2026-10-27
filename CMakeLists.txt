cmake_minimum_required(VERSION 3.26)
if(POLICY CMP0058)
	cmake_policy(SET CMP0058 NEW)
endif()
if(POLICY CMP0074)
	cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0069)
	cmake_policy(SET CMP0069 NEW)
endif()

include(CheckCXXSymbolExists)

set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake_modules/")

set(DEPS_DIR ${DEPS_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/ref")
project("eu07")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=undefined")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined -Wformat")
#set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} -fsanitize=address -fsanitize=undefined")

string(TIMESTAMP VERSION_YEAR "%y")
string(TIMESTAMP VERSION_MONTH "%m")
string(TIMESTAMP VERSION_DAY "%d")

# Use these as version numbers
set(VERSION_MAJOR ${VERSION_YEAR})
set(VERSION_MINOR ${VERSION_MONTH})
set(VERSION_PATCH ${VERSION_DAY})

set(VERSION_STRING "${VERSION_YEAR}.${VERSION_MONTH}.${VERSION_DAY}")


set(CMAKE_CXX_STANDARD 20)

file(GLOB HEADERS "*.h"
"Console/*.h"
"McZapkie/*.h"
"gl/*.h"
"network/*.h"
"vr/*.h"
"network/backend/*.h"
"widgets/*.h"
"launcher/*.h"
"extras/*.h"
"global_include/*.h"
"simulation/*.h"
"scene/*.h"
"rendering/*.h"
"rendering/common/*.h"
"rendering/opengl46/*.h"
"rendering/vulkan14/*.h"
"audio/*.h"
"environment/*.h"
"application/*.h"
"input/*.h"
"scripting/*.h"
"model/*.h"
"utilities/*.h"
"world/*.h"
"vehicle/*.h")

set(GLOBAL_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(GLOBAL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/global_include")
set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui")

if (APPLE)
	set (CMAKE_FIND_FRAMEWORK LAST)
endif()

option(WITH_OPENGL_MODERN "Compile with OpenGL modern renderer" ON)
option(WITH_VULKAN "Compile with the Vulkan 1.4 renderer (gfxrenderer vulkan); needs the submodules ref/volk and ref/Vulkan-Headers" ON)
option(WITH_DLSS "Compile the Vulkan renderer with NVIDIA DLSS (gfx.antialiasing dlss); needs the DLSS SDK in DLSS_SDK_ROOT, cmake -P tools/fetch_dlss_sdk.cmake" OFF)
option(WITH_PYTHON "Python 3.14 embedded scripting" ON)
option(WITH_UART "Compile with libserialport" ON)
option(WITH_LUA "Compile with lua scripting support" ON)
option(WITH_OPENVR "Compile with OpenVR" OFF)
option(WITH_ZMQ "Compile with cppzmq" OFF)
option(WITH_CRASHPAD "Compile with crashpad" OFF)
option(WITH_DISCORD_RPC "Compile with DISCORD RICH PRESENCE" ON)
option(USE_LTO "Use link-time optimization" OFF)
option(WITH_TESTS "Build and run the unit tests with the game, a failing test fails the build" ON)
option(GENERATE_PDB "Generate executable with program debugging symbols" ON)
option(ENABLE_MCC "Enable multicore compilation" ON)
option(WITHDUMPGEN "Enable generating DMP files on crash" ON)
option(ENABLE_AVX2 "Enable AVX2 instruction set (requires AVX2-capable CPU, Haswell/2013 or newer)" ON)
option(ADD_VERSION_SUFFIX "Add date and commit hash to the executable name" ON)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# vcpkg init
set(VCPKG_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/ref/vcpkg")
set(VCPKG_INSTALLED_PATH "${VCPKG_ROOT}/installed/x64-windows")
list(APPEND CMAKE_PREFIX_PATH "${VCPKG_INSTALLED_PATH}")
list(APPEND CMAKE_MODULE_PATH "${VCPKG_INSTALLED_PATH}/share")

set(SOURCES
"model/Texture.cpp"
"utilities/Timer.cpp"
"utilities/workerpool.cpp"
"utilities/memorystats.cpp"
"world/Track.cpp"
"world/TrackSleepers.cpp"
"world/TrackBlend.cpp"
"world/SleeperPaths.cpp"
"world/SleeperFeatures.cpp"
"world/SleeperLines.cpp"
"world/SleeperLayout.cpp"
"world/SleeperModels.cpp"
"world/SleeperWorker.cpp"
"world/GaugeLine.cpp"
"world/RailCrossings.cpp"
"world/RailMaterials.cpp"
"world/RailProfile.cpp"
"world/RailPlan.cpp"
"world/RailCarve.cpp"
"world/RailLayer.cpp"
"world/Road.cpp"
"world/Sweep.cpp"
"world/RoadPoint.cpp"
"world/Traction.cpp"
"world/catenary_solver.cpp"
"world/catenary_network.cpp"
"world/catenary_span.cpp"
"world/catenary_tensioner.cpp"
"world/catenary_suspension.cpp"
"world/catenary_scene.cpp"
"world/catenary_cantilever.cpp"
"world/catenary_layout.cpp"
"world/catenary_catalog.cpp"
"world/catenary_scene_objects.cpp"
"world/TractionPower.cpp"
"vehicle/Train.cpp"
"world/TrkFoll.cpp"
"vehicle/AirCoupler.cpp"
"model/AnimModel.cpp"
"vehicle/Button.cpp"
"vehicle/Camera.cpp"
"vehicle/Driver.cpp"
"application/driverhints.cpp"
"vehicle/DynObj.cpp"
"EU07.cpp"
"export_e3d_standalone.cpp"
"export_btf_standalone.cpp"
"nodebank_previews_standalone.cpp"
"world/Event.cpp"
"world/EvLaunch.cpp"
"utilities/Float3d.cpp"
"vehicle/Gauge.cpp"
"utilities/Globals.cpp"
"utilities/Logs.cpp"
"model/MdlMngr.cpp"
"world/MemCell.cpp"
"model/Model3d.cpp"
"world/mtable.cpp"
"rendering/nullrenderer.cpp"
"rendering/renderer.cpp"
"model/ResourceManager.cpp"
"scene/sn_utils.cpp"
"world/Segment.cpp"
"environment/sky.cpp"
"environment/sun.cpp"
"environment/stars.cpp"
"environment/starcatalog.cpp"
"environment/weather.cpp"
"rendering/lightarray.cpp"
"environment/skydome.cpp"
"audio/sound.cpp"
"world/Spring.cpp"
"rendering/frustum.cpp"
"application/uilayer.cpp"
"rendering/openglmatrixstack.cpp"
"environment/moon.cpp"
"input/command.cpp"
"input/keyboardinput.cpp"
"input/gamepadinput.cpp"
"input/drivermouseinput.cpp"
"utilities/translation.cpp"
"model/material.cpp"
"stdafx.cpp"
"input/messaging.cpp"
"scene/scene.cpp"
"scene/scenenode.cpp"
"simulation/simulation.cpp"
"model/vertex.cpp"
"audio/audio.cpp"
"audio/audiorenderer.cpp"
"audio/radiostream.cpp"
"audio/fmradio.cpp"
"utilities/motiontelemetry.cpp"
"application/uitranscripts.cpp"
"world/station.cpp"
"application/application.cpp"
"simulation/simulationtime.cpp"
"scene/sceneeditor.cpp"
"rendering/screenshot.cpp"
"input/driverkeyboardinput.cpp"
"application/drivermode.cpp"
"application/driveruilayer.cpp"
"application/driveruipanels.cpp"
"input/editorkeyboardinput.cpp"
"editor/editorSettings.cpp"
"editor/editorModelSets.cpp"
"editor/editorTrackPresets.cpp"
"editor/editorIncludeInfo.cpp"
"editor/editorTerrain.cpp"
"editor/editorTrack.cpp"
"editor/editorRoad.cpp"
"editor/editorAlignment.cpp"
"editor/editorTrackShape.cpp"
"editor/editorGroundFollow.cpp"
"editor/editorAlignmentStore.cpp"
"editor/editorSwitchStore.cpp"
"editor/editorSwitchDesign.cpp"
"editor/editorConnections.cpp"
"application/editormodeconnection.cpp"
"application/editormodeconnectioncurve.cpp"
"application/editormodedigitize.cpp"
"editor/editorSpeed.cpp"
"editor/editorProfile.cpp"
"editor/editorGauge.cpp"
"editor/editorInfra.cpp"
"editor/editorTerrainStreamer.cpp"
"editor/editorOrthophoto.cpp"
"editor/editorHttp.cpp"
"editor/editorNmt.cpp"
"editor/editorPreviews.cpp"
"editor/editorIcons.cpp"
"editor/editorClips.cpp"
"application/editormode.cpp"
"application/editormodetrack.cpp"
"application/editormoderoute.cpp"
"application/editormodestraights.cpp"
"application/editormodelay.cpp"
"application/editormodeswitchplace.cpp"
"application/editormodetrackgizmo.cpp"
"application/editormoderecords.cpp"
"application/editormodeswitchdrive.cpp"
"application/editormodeswitchtool.cpp"
"application/editormodenmt.cpp"
"application/editormodeswitchslide.cpp"
"application/editormodeswitchdesign.cpp"
"application/editormodetrackbed.cpp"
"application/editormodetrackux.cpp"
"application/editormodetrackprops.cpp"
"application/editormoderoad.cpp"
"application/editormodeprofile.cpp"
"application/editormodetextures.cpp"
"application/editormodegauge.cpp"
"application/editormodeinfra.cpp"
"application/editormodespeed.cpp"
"application/editormodejoints.cpp"
"application/editormodeparallel.cpp"
"application/editormodelineside.cpp"
"application/editormodevehicle.cpp"
"application/editormodeselection.cpp"
"application/editormodeviewport.cpp"
"application/editormodeterrainchunks.cpp"
"application/editormodesignal.cpp"
"application/editormodecatenary.cpp"
"application/editormodesweep.cpp"
"application/editormodeturntable.cpp"
"application/editormodeselftest.cpp"
"application/editormodeselftestscene.cpp"
"application/editormodeselftestsleepers.cpp"
"application/editormodeselftestsweep.cpp"
"application/editormodeselftesttrack.cpp"
"application/editormodeselftestswitch.cpp"
"application/editormodeselftestjoin.cpp"
"application/editormodeselftestsignal.cpp"
"application/editormodeselftestterrain.cpp"
"application/editormodeselftestrails.cpp"
"application/editormodeselftestmonkey.cpp"
"application/scriptedinput.cpp"
"input/editormouseinput.cpp"
"application/editoruilayer.cpp"
"application/editoruipanels.cpp"
"application/scenarioloadermode.cpp"
"application/scenarioloaderuilayer.cpp"
"scene/scenenodegroups.cpp"
"scene/scenelayers.cpp"
"scene/scenelayerssave.cpp"
"scene/sceneterrain.cpp"
"scene/heightmapterrain.cpp"
"scene/scenemodelentries.cpp"
"simulation/simulationenvironment.cpp"
"simulation/simulationstateserializer.cpp"
"rendering/precipitation.cpp"
"scripting/pythonscreenviewer.cpp"
"utilities/dictionary.cpp"
"rendering/particles.cpp"
"utilities/headtrack.cpp"
"rendering/geometrybank.cpp"
"simulation/simulationsounds.cpp"
"rendering/openglcamera.cpp"

"network/network.cpp"
"network/message.cpp"
"network/manager.cpp"
"network/backend/asio.cpp"

"widgets/vehiclelist.cpp"
"widgets/map.cpp"
"widgets/map_objects.cpp"
"widgets/popup.cpp"
"widgets/time.cpp"
"widgets/weatherpanel.cpp"
"widgets/vehicleparams.cpp"
"widgets/trainingcard.cpp"
"widgets/perfgraphs.cpp"

"${DEPS_DIR}/glad/src/glad.c"
"${DEPS_DIR}/dds-ktx/src/dds-ktx.c"
"${DEPS_DIR}/zstd/zstd.c"

"gl/shader.cpp"
"gl/vao.cpp"
"gl/ubo.cpp"
"gl/framebuffer.cpp"
"gl/renderbuffer.cpp"
"gl/postfx.cpp"
"gl/cubemap.cpp"
"gl/glsl_common.cpp"
"rendering/common/shadersource.cpp"
"rendering/common/scenewalk.cpp"
"rendering/common/scenewalkindirect.cpp"
"rendering/common/instancecull.cpp"
"rendering/common/shadowfit.cpp"
"rendering/common/localshadows.cpp"
"rendering/common/cabprobe.cpp"
"rendering/common/skyocclusion.cpp"
"rendering/common/terrainmaterials.cpp"
"rendering/common/grassfield.cpp"
"rendering/common/gpuparticles.cpp"
"rendering/common/windscreenwater.cpp"
"rendering/common/locallights.cpp"
"rendering/common/skylight.cpp"
"rendering/common/weatherstate.cpp"
"rendering/common/exposure.cpp"
"rendering/common/temporalaa.cpp"
"rendering/common/screenreflections.cpp"
"gl/buffer.cpp"
"gl/fence.cpp"
"gl/pbo.cpp"
"gl/query.cpp"
"gl/ssbo.cpp"
"gl/streambuffer.cpp"
"gl/cluster_grid.cpp"
"gl/readback.cpp"

"imgui/imgui.cpp"
"imgui/imgui_demo.cpp"
"imgui/imgui_draw.cpp"
"imgui/imgui_widgets.cpp"
"imgui/imgui_tables.cpp"
"imgui/imgui_impl_glfw.cpp"
"imgui/ImGuizmo.cpp"

"stb/stb_image.c"

"launcher/launchermode.cpp"
"launcher/scenery_list.cpp"
"launcher/launcheruilayer.cpp"
"launcher/vehicle_picker.cpp"
"launcher/keymapper.cpp"
"launcher/textures_scanner.cpp"
"launcher/scenery_scanner.cpp"
"launcher/scenery_wizard.cpp"

"vr/vr_interface.cpp"
)

set(DEFINITIONS)
set(SOURCES ${SOURCES} "imgui/imgui_impl_opengl3.cpp")
set_source_files_properties("imgui/imgui_impl_opengl3.cpp" PROPERTIES SKIP_PRECOMPILE_HEADERS TRUE)

if (WITH_CRASHPAD)
	set(DEFINITIONS ${DEFINITIONS} "WITH_CRASHPAD")
	set(SOURCES ${SOURCES} "utilities/crashreporter.cpp")
	if (WIN32)
		include_directories("${DEPS_DIR}/crashpad/include" "${DEPS_DIR}/crashpad/include/mini_chromium")
	else()
		include_directories("${DEPS_DIR}/crashpad/crashpad" "${DEPS_DIR}/crashpad/crashpad/third_party/mini_chromium/mini_chromium")
	endif()
endif()

if(WITH_PYTHON)
	set(DEFINITIONS ${DEFINITIONS} "WITH_PYTHON")
	set(SOURCES ${SOURCES} "scripting/PyInt.cpp")
else()
	set(SOURCES ${SOURCES} "scripting/PyIntStub.cpp")
endif()

if (WITH_UART)
	set(DEFINITIONS ${DEFINITIONS} "WITH_UART")
	set(SOURCES ${SOURCES} "utilities/uart.cpp")
endif()

if (WITH_ZMQ)
	set(DEFINITIONS ${DEFINITIONS} "WITH_ZMQ")
	set(SOURCES ${SOURCES} "input/zmq_input.cpp")
endif()

if (WITH_OPENVR)
	set(DEFINITIONS ${DEFINITIONS} "WITH_OPENVR")
	set(SOURCES ${SOURCES} "vr/openvr_imp.cpp")
endif()

if (WITH_LUA)
	set(DEFINITIONS ${DEFINITIONS} "WITH_LUA")
	set(SOURCES ${SOURCES} "scripting/lua.cpp")
endif()

if (WITHDUMPGEN)
	set(DEFINITIONS ${DEFINITIONS} "WITHDUMPGEN")
endif()

if (WITH_OPENGL_MODERN)
        set(SOURCES ${SOURCES}
            "rendering/opengl46/opengl46geometrybank.cpp"
            "rendering/opengl46/opengl46light.cpp"
            "rendering/opengl46/opengl46skydome.cpp"
            "rendering/opengl46/opengl46sky.cpp"
            "rendering/opengl46/opengl46weather.cpp"
            "rendering/opengl46/opengl46exposure.cpp"
            "rendering/opengl46/opengl46ao.cpp"
            "rendering/opengl46/opengl46localshadows.cpp"
            "rendering/opengl46/opengl46cabprobe.cpp"
            "rendering/opengl46/opengl46skyocclusion.cpp"
            "rendering/opengl46/opengl46instancecull.cpp"
            "rendering/opengl46/opengl46hiz.cpp"
            "rendering/opengl46/opengl46shadowfit.cpp"
            "rendering/opengl46/opengl46precipitation.cpp"
            "rendering/opengl46/opengl46particles.cpp"
            "rendering/opengl46/opengl46gpuparticles.cpp"
            "rendering/opengl46/opengl46water.cpp"
            "rendering/opengl46/opengl46terrain.cpp"
            "rendering/opengl46/opengl46grass.cpp"
            "rendering/opengl46/opengl46backend.cpp"
            "rendering/opengl46/opengl46renderer.cpp")
		set(DEFINITIONS ${DEFINITIONS} "WITH_OPENGL_MODERN")
endif()

if (WITH_VULKAN)
	# the vulkan loader is loaded by volk when the program runs, so the program starts without it (and runs with opengl)
	set(VULKAN_HEADERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ref/Vulkan-Headers/include")
	set(VOLK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ref/volk")
	# the memory allocator (header only) and the shader compiler of the renderer
	set(VMA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ref/VulkanMemoryAllocator/include")
	set(GLSLANG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ref/glslang")
	if (NOT EXISTS "${VOLK_DIR}/volk.h" OR NOT EXISTS "${VULKAN_HEADERS_DIR}/vulkan/vulkan_core.h" OR NOT EXISTS "${VMA_DIR}/vk_mem_alloc.h" OR NOT EXISTS "${GLSLANG_DIR}/CMakeLists.txt")
		message(FATAL_ERROR "WITH_VULKAN=ON needs the submodules ref/volk, ref/Vulkan-Headers, ref/VulkanMemoryAllocator and ref/glslang: git submodule update --init ref/volk ref/Vulkan-Headers ref/VulkanMemoryAllocator ref/glslang")
	endif()
	set(SOURCES ${SOURCES}
		"rendering/vulkan14/vulkan14common.cpp"
		"rendering/vulkan14/vulkan14capabilities.cpp"
		"rendering/vulkan14/vulkan14instance.cpp"
		"rendering/vulkan14/vulkan14device.cpp"
		"rendering/vulkan14/vulkan14swapchain.cpp"
		"rendering/vulkan14/vulkan14memory.cpp"
		"rendering/vulkan14/vulkan14vma.cpp"
		"rendering/vulkan14/vulkan14shaders.cpp"
		"rendering/vulkan14/vulkan14descriptors.cpp"
		"rendering/vulkan14/vulkan14compute.cpp"
		"rendering/vulkan14/vulkan14passes.cpp"
		"rendering/vulkan14/vulkan14texture.cpp"
		"rendering/vulkan14/vulkan14imgui.cpp"
		"rendering/vulkan14/vulkan14renderer.cpp"
		"rendering/vulkan14/vulkan14geometry.cpp"
		"rendering/vulkan14/vulkan14scene.cpp"
		"rendering/vulkan14/vulkan14frame.cpp"
		"rendering/vulkan14/vulkan14sky.cpp"
		"rendering/vulkan14/vulkan14weather.cpp"
		"rendering/vulkan14/vulkan14ao.cpp"
		"rendering/vulkan14/vulkan14cull.cpp"
		"rendering/vulkan14/vulkan14indirect.cpp"
		"rendering/vulkan14/vulkan14shadows.cpp"
		"rendering/vulkan14/vulkan14lamps.cpp"
		"rendering/vulkan14/vulkan14cabprobe.cpp"
		"rendering/vulkan14/vulkan14skyocclusion.cpp"
		"rendering/vulkan14/vulkan14terrain.cpp"
		"rendering/vulkan14/vulkan14grass.cpp"
		"rendering/vulkan14/vulkan14particles.cpp"
		"rendering/vulkan14/vulkan14water.cpp"
		"rendering/vulkan14/vulkan14debug.cpp"
		"rendering/vulkan14/vulkan14commands.cpp"
		"rendering/vulkan14/vulkan14raytracing.cpp"
		"rendering/vulkan14/vulkan14dlss.cpp"
		"ref/volk/volk.c")
	set(DEFINITIONS ${DEFINITIONS} "WITH_VULKAN")

	# nvidia dlss of the vulkan renderer, from the dlss sdk (github.com/NVIDIA/DLSS) outside the repository: the headers and the
	# static library of ngx to build with, the dlss library next to the program to run with (rendering/vulkan14/vulkan14dlss.h), and the
	# one of its frame generation (gfx.framegeneration) where the sdk has it
	if (WITH_DLSS)
		set(DLSS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/ref/DLSS" CACHE PATH "The NVIDIA DLSS SDK, with include/ and lib/ (cmake -P tools/fetch_dlss_sdk.cmake)")
		if (NOT EXISTS "${DLSS_SDK_ROOT}/include/nvsdk_ngx_helpers_vk.h")
			message(FATAL_ERROR "WITH_DLSS=ON needs the DLSS SDK in DLSS_SDK_ROOT (${DLSS_SDK_ROOT}): cmake -P tools/fetch_dlss_sdk.cmake")
		endif()
		if (CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
			set(DLSS_ARCH "aarch64")
		else()
			set(DLSS_ARCH "x86_64")
		endif()
		if (WIN32)
			# the variants of the dynamic c runtime (/MD, /MDd in debug)
			# the static libraries of x86_64 are in x64/ (the others of older visual studios beside it), those of arm64 at the top
			if (DLSS_ARCH STREQUAL "x86_64")
				set(DLSS_NGX_DIR "${DLSS_SDK_ROOT}/lib/Windows_x86_64/x64")
			else()
				set(DLSS_NGX_DIR "${DLSS_SDK_ROOT}/lib/Windows_${DLSS_ARCH}")
			endif()
			set(DLSS_NGX_LIBRARY "$<IF:$<CONFIG:Debug>,${DLSS_NGX_DIR}/nvsdk_ngx_d_dbg.lib,${DLSS_NGX_DIR}/nvsdk_ngx_d.lib>")
			set(DLSS_NGX_CHECK "${DLSS_NGX_DIR}/nvsdk_ngx_d.lib")
			set(DLSS_RUNTIME_PATTERN "nvngx_dlss.dll")
			set(DLSSG_RUNTIME_PATTERN "nvngx_dlssg.dll")
			set(DLSS_RUNTIME_DIR "${DLSS_SDK_ROOT}/lib/Windows_${DLSS_ARCH}")
		elseif (UNIX AND NOT APPLE)
			set(DLSS_NGX_LIBRARY "${DLSS_SDK_ROOT}/lib/Linux_${DLSS_ARCH}/libnvsdk_ngx.a")
			set(DLSS_NGX_CHECK "${DLSS_NGX_LIBRARY}")
			set(DLSS_RUNTIME_PATTERN "libnvidia-ngx-dlss.so.*")
			set(DLSSG_RUNTIME_PATTERN "libnvidia-ngx-dlssg.so.*")
			set(DLSS_RUNTIME_DIR "${DLSS_SDK_ROOT}/lib/Linux_${DLSS_ARCH}")
		else()
			message(FATAL_ERROR "WITH_DLSS=ON: the DLSS SDK has no libraries for this platform")
		endif()
		if (NOT EXISTS "${DLSS_NGX_CHECK}")
			message(FATAL_ERROR "WITH_DLSS=ON: ${DLSS_NGX_CHECK} isn't in the DLSS SDK, fetch the libraries of this platform: cmake -P tools/fetch_dlss_sdk.cmake")
		endif()
		file(GLOB DLSS_RUNTIME_RELEASE "${DLSS_RUNTIME_DIR}/rel/${DLSS_RUNTIME_PATTERN}")
		file(GLOB DLSS_RUNTIME_DEVELOPMENT "${DLSS_RUNTIME_DIR}/dev/${DLSS_RUNTIME_PATTERN}")
		if (NOT DLSS_RUNTIME_RELEASE)
			message(WARNING "WITH_DLSS=ON: no ${DLSS_RUNTIME_PATTERN} in ${DLSS_RUNTIME_DIR}/rel, it has to be put next to the program by hand")
		endif()
		file(GLOB DLSSG_RUNTIME_RELEASE "${DLSS_RUNTIME_DIR}/rel/${DLSSG_RUNTIME_PATTERN}")
		file(GLOB DLSSG_RUNTIME_DEVELOPMENT "${DLSS_RUNTIME_DIR}/dev/${DLSSG_RUNTIME_PATTERN}")
		if (DLSSG_RUNTIME_RELEASE)
			list(APPEND DLSS_RUNTIME_RELEASE ${DLSSG_RUNTIME_RELEASE})
		else()
			message(STATUS "WITH_DLSS=ON: no ${DLSSG_RUNTIME_PATTERN} in ${DLSS_RUNTIME_DIR}/rel, the frame generation (gfx.framegeneration) needs it next to the program")
		endif()
		list(APPEND DLSS_RUNTIME_DEVELOPMENT ${DLSSG_RUNTIME_DEVELOPMENT})
		# the sdk is seen by vulkan14dlss.cpp only; the rest of the renderer asks vk14::dlss::compiled()
		set_source_files_properties("rendering/vulkan14/vulkan14dlss.cpp" PROPERTIES COMPILE_DEFINITIONS "WITH_DLSS" SKIP_PRECOMPILE_HEADERS ON)
	endif()

	# glslang compiles the shaders of shaders/vk14 when the program runs (no spirv-tools, no hlsl, no programs of its own)
	set(GLSLANG_TESTS OFF CACHE BOOL "" FORCE)
	set(GLSLANG_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
	set(ENABLE_GLSLANG_BINARIES OFF CACHE BOOL "" FORCE)
	set(ENABLE_OPT OFF CACHE BOOL "" FORCE)
	set(ENABLE_HLSL OFF CACHE BOOL "" FORCE)
	set(ENABLE_SPVREMAPPER OFF CACHE BOOL "" FORCE)
	set(BUILD_EXTERNAL OFF CACHE BOOL "" FORCE)
	add_subdirectory("${GLSLANG_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/glslang" EXCLUDE_FROM_ALL)
endif()

add_subdirectory("${DEPS_DIR}/glfw/" EXCLUDE_FROM_ALL)

set(SOURCES ${SOURCES} "extras/piped_proc.cpp" "widgets/cameraview_extcam.cpp")

find_package(Git)

if (GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
	execute_process(
	  COMMAND ${GIT_EXECUTABLE} describe --match=DUMMY --always --dirty
	  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	  OUTPUT_VARIABLE GIT_HASH
	  OUTPUT_STRIP_TRAILING_WHITESPACE
	)

    execute_process(
	  COMMAND ${GIT_EXECUTABLE} show -s --format=%ci
	  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	  OUTPUT_VARIABLE SRC_DATE
	  OUTPUT_STRIP_TRAILING_WHITESPACE
	)

    execute_process(
	  COMMAND ${GIT_EXECUTABLE} show -s --format=%cs
	  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	  OUTPUT_VARIABLE SRC_SHORT_DATE
	  OUTPUT_STRIP_TRAILING_WHITESPACE
	)
else()
	string(TIMESTAMP SRC_SHORT_DATE %Y-%m-%d UTC)
	string(TIMESTAMP SRC_DATE UTC)
	set(GIT_HASH "(unknown)")
endif()

configure_file("${CMAKE_SOURCE_DIR}/version_info.h.in" "${CMAKE_BINARY_DIR}/geninc/version_info.h")

# the icons of the editor are built into the executable: the png is written out as the bytes of an array, see editor/editorIcons.cpp
set(EDITOR_ICONS "${CMAKE_SOURCE_DIR}/editor/icons/editor_icons.png")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${EDITOR_ICONS}")
file(READ "${EDITOR_ICONS}" EDITOR_ICONS_HEX HEX)
string(REGEX REPLACE "(................................)" "\\1\n" EDITOR_ICONS_HEX "${EDITOR_ICONS_HEX}")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," EDITOR_ICONS_HEX "${EDITOR_ICONS_HEX}")
file(CONFIGURE OUTPUT "${CMAKE_BINARY_DIR}/geninc/editor_icons_png.inc" CONTENT "${EDITOR_ICONS_HEX}\n")

set (PREFIX "")
set (SUFFIX "")

if (ADD_VERSION_SUFFIX)
	set (SUFFIX "_${SRC_SHORT_DATE}_${GIT_HASH}")
endif()

set(OPENVR_LIBRARY "openvr_api")
set(OPENVR_INCLUDE_DIR "")

if (WIN32)
	set(DEFINITIONS ${DEFINITIONS} "HAVE_ROUND") # to make pymath to not redefine round
	# set(DEFINITIONS ${DEFINITIONS} "GLFW_DLL")

	set(SOURCES ${SOURCES} "windows.cpp" "Console.cpp" "Console/LPT.cpp" "Console/PoKeys55.cpp")
	#set(GLFW3_INCLUDE_DIR ${GLFW3_INCLUDE_DIR} "${DEPS_DIR}/glfw/include/")
	set(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIRS} "${DEPS_DIR}/libpng/include/")
	#set(ZLIB_INCLUDE_DIR ${ZLIB_INCLUDE_DIR} "${DEPS_DIR}/zlib/")
	set(OPENAL_INCLUDE_DIR ${OPENAL_INCLUDE_DIR} "${DEPS_DIR}/openal/include")
	set(LIBSNDFILE_INCLUDE_DIR ${LIBSNDFILE_INCLUDE_DIR} "${DEPS_DIR}/libsndfile/include")
	set(LUAJIT_INCLUDE_DIR ${LUAJIT_INCLUDE_DIR} "${DEPS_DIR}/luajit/include")
	set(libserialport_INCLUDE_DIR ${LIBSERIALPORT_INCLUDE_DIR} "${DEPS_DIR}/libserialport/include")
	set(OPENVR_INCLUDE_DIR "${DEPS_DIR}/openvr/include")

	#if(${CMAKE_EXE_LINKER_FLAGS} STREQUAL "/machine:x64")
	#	set (ARCH "x64")
	#else()
	#	set (ARCH "x86")
	#endif()

	set(ARCH "x64")
	set (PREFIX "-${ARCH}")

	#set(GLFW3_LIBRARIES ${GLFW3_LIBRARIES} "${DEPS_DIR}/glfw/lib/${ARCH}/glfw3dll.lib")
	set(PNG_LIBRARIES ${PNG_LIBRARIES} "${DEPS_DIR}/libpng/lib/${ARCH}/libpng16.lib")
	set(OPENAL_LIBRARY ${OPENAL_LIBRARY} "${DEPS_DIR}/openal/lib/${ARCH}/OpenAL32.lib")
	set(LIBSNDFILE_LIBRARY ${LIBSNDFILE_LIBRARY} "${DEPS_DIR}/libsndfile/lib/${ARCH}/libsndfile-1.lib")
	set(LUAJIT_LIBRARIES ${LUAJIT_LIBRARIES} "${DEPS_DIR}/luajit/lib/${ARCH}/lua51.lib")
	set(libserialport_LIBRARY ${LIBSERIALPORT_LIBRARY} "${DEPS_DIR}/libserialport/lib/${ARCH}/libserialport-0.lib")
	set(cppzmq_LIBRARY ${cppzmq_LIBRARY} "${DEPS_DIR}/cppzmq/lib/${ARCH}/libzmq.lib")
	set(OPENVR_LIBRARY "${DEPS_DIR}/openvr/lib/${ARCH}/openvr_api.lib")
endif()

set(cppzmq_INCLUDE_DIR ${cppzmq_INCLUDE_DIR} "${DEPS_DIR}/cppzmq/include")
set(GLM_INCLUDE_DIR ${GLM_INCLUDE_DIR} "${DEPS_DIR}/glm/")

# Process version.rc.in into version.rc
configure_file(${CMAKE_SOURCE_DIR}/eu07.rc.in
               ${CMAKE_BINARY_DIR}/eu07.rc @ONLY)

if (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
    set(SOURCES ${SOURCES} ${CMAKE_BINARY_DIR}/eu07.rc)
    set(SOURCES ${SOURCES} eu07.ico)
    set(SOURCES ${SOURCES} eu07.ico
			editor/editorSettings.hpp)
    add_compile_options("/utf-8")
endif()

# the build settings of the game: include dirs, definitions, compile options and libraries. the game,
# its core library and the unit tests all build with the same ones
add_library(eu07_settings INTERFACE)

# the parts of the simulation the game shares with the unit tests: the vehicle physics, the vehicle
# definition parser and the helpers they use. built once, and tested as the game uses them
set(CORE_SOURCES
"McZapkie/friction.cpp"
"McZapkie/hamulce.cpp"
"McZapkie/Mover.cpp"
"McZapkie/Oerlikon_ESt.cpp"
"utilities/parser.cpp"
"utilities/utilities.cpp"
"scripting/ladderlogic.cpp"
"vehicle/handover.cpp"
)
add_library(eu07_core STATIC ${CORE_SOURCES})
target_link_libraries(eu07_core PUBLIC eu07_settings)
set_target_properties(eu07_core PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	FOLDER "libraries")

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} eu07_core)
if (WITH_VULKAN)
	# the vulkan headers are seen by the files of rendering/vulkan14 only, they include them
	target_include_directories(${PROJECT_NAME} PRIVATE "${VULKAN_HEADERS_DIR}" "${VOLK_DIR}" "${VMA_DIR}")
	target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS} glslang glslang-default-resource-limits)
	if (WITH_DLSS)
		target_include_directories(${PROJECT_NAME} PRIVATE "${DLSS_SDK_ROOT}/include")
		target_link_libraries(${PROJECT_NAME} "${DLSS_NGX_LIBRARY}" ${CMAKE_DL_LIBS})
		# the dlss library next to the program, the development one (gfx.dlss.developer) in ngx_dev/
		if (DLSS_RUNTIME_RELEASE)
			add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLSS_RUNTIME_RELEASE} "$<TARGET_FILE_DIR:${PROJECT_NAME}>")
		endif()
		if (DLSS_RUNTIME_DEVELOPMENT)
			add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>/ngx_dev"
				COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLSS_RUNTIME_DEVELOPMENT} "$<TARGET_FILE_DIR:${PROJECT_NAME}>/ngx_dev")
		endif()
	endif()
endif()

target_compile_definitions(eu07_settings INTERFACE ${DEFINITIONS})

target_include_directories(eu07_settings INTERFACE
	"."
	"Console"
	"McZapkie"
	"gl"
	"${DEPS_DIR}/glad/include"
	"${DEPS_DIR}/dds-ktx/include"
	"${DEPS_DIR}/zstd"
	"${CMAKE_BINARY_DIR}/geninc"
	"${GLOBAL_INCLUDE_DIR}")

if (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
	# override cmake defaults
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO " /MD /Zi /O2 /DNDEBUG")
	set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO " /DEBUG /INCREMENTAL:NO /OPT:REF /OPT:ICF")

	target_link_options(${PROJECT_NAME} PRIVATE "/MAP")

	# /wd4996: disable "deprecation" warnings
	# /wd4244: disable warnings for conversion with possible loss of data
	# /wd5033: disable because it is all over Python headers
	target_compile_options(eu07_settings INTERFACE /wd4996 /wd4244 /wd5033)
	set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/LARGEADDRESSAWARE")
endif()

set_target_properties( ${PROJECT_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/pdb"
	OUTPUT_NAME "${PROJECT_NAME}${PREFIX}${SUFFIX}"
	DEBUG_POSTFIX "_d"
)

check_cxx_symbol_exists(__GNUC__ "" COMPILER_HAVE_GNUC)
if (COMPILER_HAVE_GNUC)
	target_compile_options(eu07_settings INTERFACE -fvisibility=hidden)
endif()

if(NOT ${CMAKE_VERSION} VERSION_LESS "3.16.0")
	option(USE_PCH "Use precompiled header" ON)

	if (USE_PCH)
		target_precompile_headers(${PROJECT_NAME} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/stdafx.h>")
		target_precompile_headers(eu07_core PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/stdafx.h>")
		set_source_files_properties("/glad/src/glad.c" "ref/dds-ktx/src/dds-ktx.c" "ref/zstd/zstd.c" "stb/stb_image.c"
			"imgui/imgui.cpp" "imgui/imgui_demo.cpp" "imgui/imgui_draw.cpp" "imgui/imgui_widgets.cpp" "imgui/imgui_tables.cpp" "imgui/imgui_impl_glfw.cpp" "imgui/ImGuizmo.cpp"
			PROPERTIES SKIP_PRECOMPILE_HEADERS TRUE)
	endif()
endif()

if (USE_LTO)
	set_property(TARGET ${PROJECT_NAME} eu07_core PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

if (WITH_OPENVR)
	target_include_directories(eu07_settings INTERFACE ${OPENVR_INCLUDE_DIR})
	target_link_libraries(eu07_settings INTERFACE ${OPENVR_LIBRARY})
endif()

if (WITH_CRASHPAD)
	if (WIN32)
		target_link_libraries(${PROJECT_NAME} "${DEPS_DIR}/crashpad/lib_md/base.lib")
		target_link_libraries(${PROJECT_NAME} "${DEPS_DIR}/crashpad/lib_md/client.lib")
		target_link_libraries(${PROJECT_NAME} "${DEPS_DIR}/crashpad/lib_md/util.lib")
	else()
		target_link_libraries(${PROJECT_NAME} "${DEPS_DIR}/crashpad/crashpad/out/Default/obj/client/libclient.a")
		target_link_libraries(${PROJECT_NAME} "${DEPS_DIR}/crashpad/crashpad/out/Default/obj/util/libutil.a")
		target_link_libraries(${PROJECT_NAME} "${DEPS_DIR}/crashpad/crashpad/out/Default/obj/third_party/mini_chromium/mini_chromium/base/libbase.a")
	endif()
endif()

target_link_libraries(eu07_settings INTERFACE glfw)

if(WITH_PYTHON)
	find_package(Python3 3.14 REQUIRED COMPONENTS Development)
	target_include_directories(eu07_settings INTERFACE ${Python3_INCLUDE_DIRS})
	target_link_libraries(eu07_settings INTERFACE ${Python3_LIBRARIES})

	if(WIN32)
		# Python3_ROOT_DIR is only an input hint and is not populated by
		# find_package(... COMPONENTS Development). Derive the install root from
		# the include dir (parent of <root>/include) when it was not provided.
		if(NOT Python3_ROOT_DIR)
			get_filename_component(Python3_ROOT_DIR "${Python3_INCLUDE_DIRS}" DIRECTORY)
		endif()
		set(EU07_PY_DLL "${Python3_ROOT_DIR}/python314.dll")
		set(EU07_PY_ZLIB "${Python3_ROOT_DIR}/DLLs/zlib1.dll")
		set(EU07_PY_LIBDIR "${Python3_ROOT_DIR}/Lib")
		set(EU07_PY_DLLSDIR "${Python3_ROOT_DIR}/DLLs")
	endif()
endif()

if (NOT WIN32)
	find_package(PNG 1.6 REQUIRED)
	target_link_libraries(eu07_settings INTERFACE PNG::PNG)
else()
	include_directories(${PNG_INCLUDE_DIRS})
	target_link_libraries(eu07_settings INTERFACE ${PNG_LIBRARIES})
endif()

find_package(Threads REQUIRED)
target_link_libraries(eu07_settings INTERFACE Threads::Threads)

find_package(GLM REQUIRED)
target_include_directories(eu07_settings INTERFACE ${GLM_INCLUDE_DIR})

# JSON lib
add_subdirectory(ref/json)
target_link_libraries(eu07_settings INTERFACE nlohmann_json::nlohmann_json)
target_include_directories(eu07_settings INTERFACE ${NLOHMANN_JSON_INCLUDE_INSTALL_DIR})



if (WITH_DISCORD_RPC)
	add_subdirectory(ref/discord-rpc)
	set_target_properties(discord-rpc PROPERTIES FOLDER "libraries")
	target_link_libraries(eu07_settings INTERFACE discord-rpc)

	# Add the include directory so the compiler can find discord_rpc.h
	target_include_directories(eu07_settings INTERFACE
		"${CMAKE_CURRENT_SOURCE_DIR}/ref/discord-rpc/include"
	)

	target_compile_definitions(eu07_settings INTERFACE WITH_DISCORD_RPC=1)
else()
	target_compile_definitions(eu07_settings INTERFACE WITH_DISCORD_RPC=0)
endif()

find_package(OpenAL REQUIRED)
if (TARGET OpenAL::OpenAL)
	target_link_libraries(eu07_settings INTERFACE OpenAL::OpenAL)
else()
	target_include_directories(eu07_settings INTERFACE ${OPENAL_INCLUDE_DIR})
	target_link_libraries(eu07_settings INTERFACE ${OPENAL_LIBRARY})
endif()

find_package(SndFile REQUIRED)
target_link_libraries(eu07_settings INTERFACE SndFile::sndfile)

if (WITH_LUA)
	find_package(LuaJIT REQUIRED)
	target_include_directories(eu07_settings INTERFACE ${LUAJIT_INCLUDE_DIR})
	target_link_libraries(eu07_settings INTERFACE ${LUAJIT_LIBRARIES})
endif()

if (WITH_UART)
	find_package(libserialport REQUIRED)
	target_include_directories(eu07_settings INTERFACE ${libserialport_INCLUDE_DIR})
	target_link_libraries(eu07_settings INTERFACE ${libserialport_LIBRARY})
endif()

find_package(ASIO REQUIRED)
target_link_libraries(eu07_settings INTERFACE asio::asio)

if (WIN32)
	target_link_libraries(eu07_settings INTERFACE ws2_32)
endif()

# http client of the editor's geoportal orthophoto layer: WinHTTP on Windows, libcurl elsewhere when available
if (WIN32)
	target_link_libraries(eu07_settings INTERFACE winhttp)
else()
	find_package(CURL QUIET)
	if (CURL_FOUND)
		target_link_libraries(eu07_settings INTERFACE CURL::libcurl)
		target_compile_definitions(eu07_settings INTERFACE EU07_WITH_CURL)
	else()
		message(STATUS "libcurl not found: the editor's orthophoto layer will only show cached tiles")
	endif()
endif()

if (WITH_ZMQ)
	find_package(cppzmq REQUIRED)
	target_include_directories(eu07_settings INTERFACE ${cppzmq_INCLUDE_DIR})
	target_link_libraries(eu07_settings INTERFACE ${cppzmq_LIBRARY})
endif()

if (WIN32)
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:${PROJECT_NAME}> $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}>
			COMMAND_EXPAND_LISTS
	)
endif ()

if(WIN32 AND WITH_PYTHON)
	add_custom_command(
		TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${EU07_PY_DLL}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${EU07_PY_ZLIB}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
		# Lib with its site-packages is huge, the script copies it only when the Python install has changed
		COMMAND ${CMAKE_COMMAND} "-DPY_LIBDIR=${EU07_PY_LIBDIR}" "-DPY_DLLSDIR=${EU07_PY_DLLSDIR}" "-DDEST_DIR=$<TARGET_FILE_DIR:${PROJECT_NAME}>"
			-P "${CMAKE_CURRENT_SOURCE_DIR}/CMake_modules/CopyPythonRuntime.cmake"
	)
endif()

# yaml-cpp, for the descriptions of scenery templates edited in the scenery editor and for the heightmap terrain
add_subdirectory("ref/yaml-cpp" "${CMAKE_CURRENT_BINARY_DIR}/yaml-cpp" EXCLUDE_FROM_ALL)
target_link_libraries(eu07_settings INTERFACE yaml-cpp)


# Enable compiling with debug symbols
if(GENERATE_PDB)
    if (MSVC)
        # Ensure Debug and Release configurations use appropriate runtime libraries
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /MDd")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi /MD")

        # Add compile options for debugging
        add_compile_options(/Zi)   # Always enable debug symbols
        add_link_options(/DEBUG /MAP)

        # Ensure that the PCH is built with the correct runtime library for each configuration
        set_target_properties(${PROJECT_NAME} eu07_core PROPERTIES
            COMPILE_OPTIONS "$<$<CONFIG:Debug>:/MDd>$<$<CONFIG:Release>:/MD>")
    else()
        # GCC/Clang (ensure -g is set for all configurations)
        add_compile_options(-g)
    endif()
endif()

if(ENABLE_MCC AND MSVC)
    message(STATUS "Enabling multi-processor compilation for MSVC.")
	add_compile_options(/MP)
endif()

if(ENABLE_AVX2)
    message(STATUS "Enabling AVX2 instruction set.")
    if (MSVC)
        target_compile_options(eu07_settings INTERFACE /arch:AVX2)
    else()
        target_compile_options(eu07_settings INTERFACE -mavx2)
    endif()
endif()

if(WITH_TESTS)
	enable_testing()
	add_subdirectory(tests/unit)
	# the unit tests run at the end of building the game, once it's linked: a failing one fails the build
	# (and removes the game binary, so the next build links it again and reruns them)
	add_dependencies(${PROJECT_NAME} eu07_unit_tests)
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND}
			-DTESTS=$<TARGET_FILE:eu07_unit_tests>
			-DGAME=$<TARGET_FILE:${PROJECT_NAME}>
			-P "${CMAKE_CURRENT_SOURCE_DIR}/tests/unit/run_tests.cmake"
		COMMENT "Running the unit tests"
		VERBATIM)
	# a change of the tests alone links the game again, so they run (Ninja and Makefile generators;
	# Visual Studio builds rerun them with the next change of the game)
	set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS
		$<TARGET_FILE:eu07_unit_tests> ${UNIT_TEST_DATA} "${CMAKE_CURRENT_SOURCE_DIR}/tests/unit/run_tests.cmake")
endif()
