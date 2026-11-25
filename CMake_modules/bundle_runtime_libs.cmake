# copies the shared libraries the game needs next to it, where its RPATH ($ORIGIN) finds them first. Linux only:
# cmake -DGAME=<game binary> -DSEARCH_DIRS=<dir|dir|...> [-DCMAKE_OBJDUMP=<objdump>] -P bundle_runtime_libs.cmake
#
# the libraries of the system itself stay out: the C library and the loader, the GPU driver stack (libGL and its
# kin come with the driver), X11 / Wayland, the sound servers, libstdc++ (the GPU drivers bring their own need of a
# newer one) and a few more every distribution has. what a library left out depends on isn't bundled either
#
# libcurl and everything it pulls in (TLS, Kerberos, LDAP, HTTP/2, compression, name handling) stay out too: the
# build machine's copy knows the CA bundle path of the build machine's distribution (Debian), so shipped next to
# the game it breaks the certificate checks on Fedora or openSUSE, and the security updates of the TLS stack come
# from the user's system, not from our package. the game isn't linked with libcurl: the editor loads the system's one
# at run time (editor/editorHttp.cpp), and without it only the editor's downloads are off

get_filename_component(game_folder "${GAME}" DIRECTORY)
set(manifest "${game_folder}/bundled_libs.txt")

# the libraries copied by the previous build go first: the game's RPATH would find them in its folder, and then
# a library updated in the system would never be copied again
if(EXISTS "${manifest}")
	file(STRINGS "${manifest}" previous)
	foreach(name IN LISTS previous)
		if(NOT name STREQUAL "")
			file(REMOVE "${game_folder}/${name}")
		endif()
	endforeach()
	file(REMOVE "${manifest}")
endif()

string(REPLACE "|" ";" search_dirs "${SEARCH_DIRS}")
if(DEFINED ENV{LD_LIBRARY_PATH})
	string(REPLACE ":" ";" ld_library_path "$ENV{LD_LIBRARY_PATH}")
	list(APPEND search_dirs ${ld_library_path})
endif()
list(REMOVE_ITEM search_dirs "")
list(REMOVE_DUPLICATES search_dirs)

set(system_libraries
	"^ld-linux"
	"^linux-vdso"
	"^lib(c|m|dl|rt|pthread|resolv|util|mvec|anl|nsl|crypt|BrokenLocale|thread_db)\\.so"
	"^libstdc\\+\\+\\.so"
	"^libgcc_s\\.so"
	"^lib(GL|GLX|EGL|OpenGL|GLdispatch|GLESv1_CM|GLESv2|GLU|gbm|drm|vulkan)\\.so"
	"^libX[A-Za-z0-9_-]*\\.so"
	"^libxcb"
	"^libxkbcommon"
	"^libwayland-"
	"^libdecor"
	"^lib(asound|pulse|pulse-simple|jack|pipewire-[0-9.]+)\\.so"
	"^lib(udev|systemd|dbus-1)\\.so"
	"^libz\\.so"
	"^lib(fontconfig|freetype|harfbuzz|expat|uuid)\\.so"
	"^libcurl"
	"^lib(ssl|crypto)\\.so"
	"^lib(gssapi_krb5|krb5|k5crypto|krb5support|com_err|keyutils)\\.so"
	"^lib(ldap|lber|sasl2)"
	"^lib(nghttp2|nghttp3|ngtcp2|rtmp|ssh2?|psl|idn2|unistring)\\.so"
	"^libbrotli(dec|enc|common)\\.so"
	"^libzstd\\.so"
	"^lib(gnutls|hogweed|nettle|gmp|p11-kit|tasn1|ffi)\\.so")

file(GET_RUNTIME_DEPENDENCIES
	EXECUTABLES "${GAME}"
	RESOLVED_DEPENDENCIES_VAR resolved
	UNRESOLVED_DEPENDENCIES_VAR unresolved
	DIRECTORIES ${search_dirs}
	PRE_EXCLUDE_REGEXES ${system_libraries})

if(unresolved)
	list(JOIN unresolved ", " unresolved_text)
	message(WARNING "Bundled libraries: not found, so not copied next to the game: ${unresolved_text}")
endif()

# each library is copied under the name the game asks for (libpng16.so.16), as a file rather than the chain of
# symlinks it is in the system: the build artifacts don't keep symlinks
set(names "")
foreach(library IN LISTS resolved)
	get_filename_component(name "${library}" NAME)
	file(REAL_PATH "${library}" source)
	file(COPY_FILE "${source}" "${game_folder}/${name}")
	list(APPEND names "${name}")
endforeach()

list(SORT names)
list(JOIN names "\n" manifest_text)
file(WRITE "${manifest}" "${manifest_text}\n")
list(LENGTH names count)
list(JOIN names " " names_text)
message("Bundled libraries (${count}): ${names_text}")
