# Copies the Python runtime (Lib and DLLs) used by the embedded interpreter next to the executable.
# Run as a post-build step: cmake -DPY_LIBDIR=... -DPY_DLLSDIR=... -DDEST_DIR=... -P CopyPythonRuntime.cmake
#
# Lib also carries the site-packages of whichever Python install the build found, which can be tens of thousands
# of files. Copying them after every link took minutes, so the copy is skipped while the source install looks
# unchanged since the last copy. Delete <exe dir>/python64/.eu07_python_runtime to force a refresh.

foreach(_var PY_LIBDIR PY_DLLSDIR DEST_DIR)
	if(NOT DEFINED ${_var})
		message(FATAL_ERROR "CopyPythonRuntime.cmake: ${_var} is not set")
	endif()
endforeach()

# directory timestamps change when pip adds or removes a package, or the Python install itself is replaced
set(_stamp "")
foreach(_path "${PY_LIBDIR}" "${PY_LIBDIR}/site-packages" "${PY_DLLSDIR}")
	file(TIMESTAMP "${_path}" _time "%Y-%m-%dT%H:%M:%S" UTC)
	string(APPEND _stamp "${_path} ${_time}\n")
endforeach()

set(_stamp_file "${DEST_DIR}/python64/.eu07_python_runtime")
set(_old_stamp "")
if(EXISTS "${_stamp_file}")
	file(READ "${_stamp_file}" _old_stamp)
endif()

if(NOT _old_stamp STREQUAL _stamp)
	message(STATUS "Copying the Python runtime to ${DEST_DIR}/python64")
	foreach(_pair "${PY_LIBDIR}|Lib" "${PY_DLLSDIR}|DLLs")
		string(REPLACE "|" ";" _pair "${_pair}")
		list(GET _pair 0 _source)
		list(GET _pair 1 _target)
		execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_directory_if_different "${_source}" "${DEST_DIR}/python64/${_target}" RESULT_VARIABLE _result)
		if(NOT _result EQUAL 0)
			message(FATAL_ERROR "CopyPythonRuntime.cmake: copying ${_source} failed")
		endif()
	endforeach()
	file(WRITE "${_stamp_file}" "${_stamp}")
endif()
