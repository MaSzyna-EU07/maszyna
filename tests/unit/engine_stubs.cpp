// stand-ins for the parts of the engine which the core library (eu07_core) links against, but which the
// unit tests don't exercise. when a change makes the library use something new from the engine and the
// test build fails to link, either move the real source into the library (CORE_SOURCES in the main
// CMakeLists.txt, if it stands on its own) or add a stub here.

#include "stdafx.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "scene/scenenodegroups.h"
#include "scene/scenelayers.h"

#include <cstdlib>
#include <iostream>

// settings ---------------------------------------------------------------------------------------

global_settings &GetGlobalSettings() {
	// defaults, as before reading eu07.ini
	static global_settings settings;
	return settings;
}

// logs: quiet, unless EU07_UNIT_TESTS_LOG is set --------------------------------------------------

namespace {
bool log_enabled() {
	static bool const enabled{std::getenv("EU07_UNIT_TESTS_LOG") != nullptr};
	return enabled;
}
} // namespace

void WriteLog(const char *str, logtype const, bool const isError) {
	if (log_enabled() && str != nullptr) {
		(isError ? std::cerr : std::cout) << str << "\n";
	}
}

void WriteLog(const std::string &str, logtype const Type) {
	WriteLog(str.c_str(), Type, false);
}

void ErrorLog(const std::string &str, logtype const Type) {
	WriteLog(str.c_str(), Type, true);
}

// scenery structure, which the parser reports include directives to --------------------------------

namespace scene {

node_groups Groups;
node_layers Layers;

group_handle node_groups::create(bool const) {
	return null_handle;
}

group_handle node_groups::close() {
	return null_handle;
}

layer_handle node_layers::open(std::string const &, include_site) {
	return null_handle;
}

layer_handle node_layers::close() {
	return null_handle;
}

void node_layers::instance_begin(std::string const &, source_span const &) {}

// basic_node has no key function for the Itanium C++ ABI (tooltip() is declared plain but defined inline), so
// GCC and Clang emit its vtable wherever scenenode.h is used, here too, and the vtable wants radius_().
// the same as the game's, in scene/scenenode.cpp
float basic_node::radius_() {
	return 0.f;
}

} // namespace scene
