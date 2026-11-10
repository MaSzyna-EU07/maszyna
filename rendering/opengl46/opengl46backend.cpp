/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// how the application makes the windows of the opengl 4.6 renderer: a core profile context of the main window, which the
// windows made later share, and the gl functions loaded through it

#include "stdafx.h"
#include "rendering/opengl46/opengl46renderer.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

namespace
{

// the context of the required version couldn't be created. a context with default settings tells which version the
// driver does provide, so the user can be told what is missing
std::string describe_gl_driver()
{
	glfwDefaultWindowHints();
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	auto *probe = glfwCreateWindow(64, 64, "eu07 opengl probe", nullptr, nullptr);
	if (probe == nullptr)
		return "no OpenGL context";

	glfwMakeContextCurrent(probe);
	std::string description;
	auto const getstring = reinterpret_cast<PFNGLGETSTRINGPROC>(glfwGetProcAddress("glGetString"));
	if (getstring != nullptr)
	{
		auto const text = [&](GLenum const Name) {
			auto const *value = reinterpret_cast<char const *>(getstring(Name));
			return std::string(value != nullptr ? value : "?");
		};
		description = "OpenGL " + text(GL_VERSION) + ", " + text(GL_RENDERER) + " (" + text(GL_VENDOR) + ")";
	}
	glfwMakeContextCurrent(nullptr);
	glfwDestroyWindow(probe);

	return description.empty() ? "unknown OpenGL version" : description;
}

void window_hints()
{
	if (Global.gfx_skippipeline && Global.iMultisampling > 0)
	{
		glfwWindowHint(GLFW_SAMPLES, 1 << Global.iMultisampling);
	}

	// activate core profile for opengl 4.6 renderer
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);

	if (Global.gfx_gldebug)
		glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);

	glfwWindowHint(GLFW_SRGB_CAPABLE, !Global.gfx_shadergamma);
}

bool after_window(GLFWwindow *Window)
{
	glfwMakeContextCurrent(Window);
	glfwSwapInterval(Global.VSync ? 1 : 0); // vsync

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		ErrorLog("Bad init: failed to initialize glad");
		return false;
	}

	return true;
}

void window_failed(std::string const &Reason)
{
	auto const driver = describe_gl_driver();
	ErrorLog("Bad init: failed to create OpenGL 4.6 core profile context (" + Reason + "); the graphics card/driver provides: " + driver);
#ifdef _WIN32
	auto const message = "MaSzyna requires OpenGL 4.6 (core profile), but the OpenGL 4.6 context could not be created.\n\n"
	                     "Your graphics card/driver provides:\n" + driver + "\n\n"
	                     "Details: " + Reason + "\n\n"
	                     "Update the graphics driver or use a graphics card with OpenGL 4.6 support.";
	MessageBoxA(nullptr, message.c_str(), "MaSzyna", MB_ICONERROR | MB_OK);
#endif
}

} // namespace

gfx_backend_desc opengl46_renderer::backend_desc()
{
	gfx_backend_desc desc;
	desc.name = "opengl 4.6";
	desc.create = &opengl46_renderer::create_func;
	desc.window_hints = &window_hints;
	desc.after_window = &after_window;
	desc.window_failed = &window_failed;
	desc.gl_context = true;
	return desc;
}
