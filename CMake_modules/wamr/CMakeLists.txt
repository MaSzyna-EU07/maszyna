# The WebAssembly Micro Runtime (ref/wamr) for the WebAssembly script screens (scripting/wasmprogram.cpp).
#
# Built here, in a directory of its own, because the WAMR build scripts set their configuration with
# add_definitions() and include_directories(): kept in this directory, none of it reaches the game's sources.
# Only the interpreter is built (see WAMR_BUILD_FAST_INTERP), with instruction metering so that a screen stuck in a loop
# can't hang the worker thread, and without the signal-based bounds checks, which would compete with the
# game's own crash handling.

if (WIN32)
	set(WAMR_BUILD_PLATFORM "windows")
elseif (APPLE)
	set(WAMR_BUILD_PLATFORM "darwin")
else ()
	set(WAMR_BUILD_PLATFORM "linux")
endif ()

set(WAMR_ROOT_DIR "${CMAKE_SOURCE_DIR}/ref/wamr")

set(WAMR_BUILD_INTERP 1)
# the fast interpreter counts the instructions only where it dispatches with computed gotos (GCC, Clang): built with
# MSVC it would let a screen stuck in a loop hang the worker. the classic interpreter counts them everywhere
if (MSVC)
	set(WAMR_BUILD_FAST_INTERP 0)
else ()
	set(WAMR_BUILD_FAST_INTERP 1)
endif ()
set(WAMR_BUILD_AOT 0)
set(WAMR_BUILD_JIT 0)
set(WAMR_BUILD_FAST_JIT 0)
set(WAMR_BUILD_LIBC_BUILTIN 0)
set(WAMR_BUILD_LIBC_WASI 0)
set(WAMR_BUILD_MULTI_MODULE 0)
set(WAMR_BUILD_LIB_PTHREAD 0)
set(WAMR_BUILD_LIB_WASI_THREADS 0)
set(WAMR_BUILD_THREAD_MGR 0)
set(WAMR_BUILD_SIMD 0)
set(WAMR_BUILD_BULK_MEMORY 1)
set(WAMR_BUILD_REF_TYPES 1)
set(WAMR_BUILD_MINI_LOADER 0)
set(WAMR_BUILD_INSTRUCTION_METERING 1)
# NOTE: the calls into the natives use WAMR's assembler invokeNative (MASM with MSVC). its C version
# (WAMR_BUILD_INVOKE_NATIVE_GENERAL) crashes on Windows x64 as soon as a module runs
set(WAMR_DISABLE_HW_BOUND_CHECK 1)
set(WAMR_DISABLE_STACK_HW_BOUND_CHECK 1)

find_package(Threads REQUIRED)
include("${WAMR_ROOT_DIR}/build-scripts/runtime_lib.cmake")

add_library(eu07_wamr STATIC ${WAMR_RUNTIME_LIB_SOURCE})
target_include_directories(eu07_wamr INTERFACE "${WAMR_ROOT_DIR}/core/iwasm/include")
# with MSVC the WAMR headers declare the API dllimport unless told otherwise; the library is linked statically
target_compile_definitions(eu07_wamr INTERFACE "WASM_RUNTIME_API_EXTERN=")
target_link_libraries(eu07_wamr PRIVATE Threads::Threads)
if (WIN32)
	target_link_libraries(eu07_wamr PRIVATE ws2_32 ntdll)
else ()
	target_link_libraries(eu07_wamr PRIVATE m ${CMAKE_DL_LIBS})
endif ()
if (MSVC)
	# third-party code: its warnings are not ours to fix
	target_compile_options(eu07_wamr PRIVATE /W0)
else ()
	target_compile_options(eu07_wamr PRIVATE -w)
endif ()
set_target_properties(eu07_wamr PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	FOLDER "libraries")
