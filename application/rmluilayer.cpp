/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/rmluilayer.h"

#ifdef WITH_RMLUI

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string_view>

#include <RmlUi/Core.h>
#include <RmlUi/Debugger.h>

#include "RmlUi_Platform_GLFW.h"
#include "application/rmluirender.h"
#include "imgui/imgui.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"

namespace
{

// the system of RmlUi on glfw (the time, the clipboard), with the log and the translations of the simulator
class system_interface final : public SystemInterface_GLFW
{
  public:
	using SystemInterface_GLFW::SystemInterface_GLFW;

	bool LogMessage(Rml::Log::Type const Type, Rml::String const &Message) override
	{
		if (Type == Rml::Log::LT_ERROR || Type == Rml::Log::LT_ASSERT)
			ErrorLog("rmlui: " + Message);
		else if (Type == Rml::Log::LT_WARNING || Type == Rml::Log::LT_ALWAYS)
			WriteLog("rmlui: " + Message);
		// true: an assertion doesn't stop the program in the debugger
		return true;
	}

	// the texts of the documents are looked up in the translations of the simulator (lang/*.po), as those of imgui are
	int TranslateString(Rml::String &Translated, Rml::String const &Input) override
	{
		Translated = Translations.lookup_s(Input);
		return Translated != Input ? 1 : 0;
	}

	// the cursor stays as the simulator sets it, as with imgui (ImGuiConfigFlags_NoMouseCursorChange)
	void SetMouseCursor(Rml::String const &) override
	{
		// nothing to do
	}
};

struct state
{
	std::unique_ptr<system_interface> system;
	rmlui::imgui_render_interface render;
	Rml::Context *context{nullptr};
	int renderedframe{-2}; // the frame of imgui the documents were last drawn in
	bool cursorvisible{false};
};

std::unique_ptr<state> &instance()
{
	static std::unique_ptr<state> running;
	return running;
}

struct font_file
{
	std::string_view path;
	bool fallback; // its glyphs stand in for those missing in the fonts the documents name
};

// the fonts of the simulator, those imgui uses. true: an ordinary one, for the texts, is there
bool load_fonts()
{
	constexpr std::array fonts{
	    font_file{"fonts/dejavusans.ttf", true},
	    font_file{"fonts/dejavusans-bold.ttf", false},
	    font_file{"fonts/DejaVuSans-Bold.ttf", false},
	    font_file{"fonts/dejavusansmono.ttf", false},
	    font_file{"fonts/NotoSansSC-Regular.ttf", true},
	};
	auto texts{false};
	for (auto const &font : fonts)
	{
		Rml::String const path{font.path};
		if (false == FileExists(path) || false == Rml::LoadFontFace(path, font.fallback))
			continue;
		texts = texts || font.fallback;
	}
	return texts;
}

// the context the input goes to: the documents are on screen, drawn in this frame of imgui or the last one
Rml::Context *input_context()
{
	auto const *running{instance().get()};
	if (running == nullptr || ImGui::GetCurrentContext() == nullptr || ImGui::GetFrameCount() - running->renderedframe > 1)
		return nullptr;
	return running->context;
}

// a control of a document has the focus (a field of text, say), not just the document
bool control_focused(Rml::Context &Context)
{
	auto *focus{Context.GetFocusElement()};
	return focus != nullptr && dynamic_cast<Rml::ElementDocument *>(focus) == nullptr;
}

bool start(GLFWwindow *Window)
{
	auto running{std::make_unique<state>()};
	running->system = std::make_unique<system_interface>(Window);
	Rml::SetSystemInterface(running->system.get());
	Rml::SetRenderInterface(&running->render);
	if (false == Rml::Initialise())
	{
		ErrorLog("rmlui: can't start, the interface is the one of imgui alone");
		return false;
	}
	if (false == load_fonts())
	{
		ErrorLog("rmlui: no font for the texts in fonts/ (dejavusans.ttf), the interface is the one of imgui alone");
		Rml::Shutdown();
		return false;
	}
	int width{0};
	int height{0};
	glfwGetWindowSize(Window, &width, &height);
	running->context = Rml::CreateContext("main", Rml::Vector2i{std::max(width, 1), std::max(height, 1)});
	if (running->context == nullptr)
	{
		ErrorLog("rmlui: can't make the context of the documents, the interface is the one of imgui alone");
		Rml::Shutdown();
		return false;
	}
	running->context->SetDensityIndependentPixelRatio(Global.ui_scale);
	Rml::Debugger::Initialise(running->context);
	instance() = std::move(running);
	WriteLog("rmlui: " + Rml::GetVersion() + " started");
	return true;
}

} // namespace

bool rmlui::init(GLFWwindow *Window)
{
	if (false == Global.ui_rmlui || instance() != nullptr)
		return instance() != nullptr;
	// the documents are drawn by the backend of imgui of the renderer
	if (Window == nullptr || ImGui::GetCurrentContext() == nullptr || GfxRenderer == nullptr || GfxRenderer->GetImguiRenderer() == nullptr)
		return false;
	return start(Window);
}

void rmlui::shutdown()
{
	auto &running{instance()};
	if (running == nullptr)
		return;
	// the documents, the fonts and the textures go, the images of the textures with them
	Rml::Shutdown();
	running->render.release_all();
	running.reset();
}

bool rmlui::active()
{
	return instance() != nullptr;
}

Rml::Context *rmlui::context()
{
	auto const *running{instance().get()};
	return running != nullptr ? running->context : nullptr;
}

void rmlui::render()
{
	auto *running{instance().get()};
	if (running == nullptr || ImGui::GetCurrentContext() == nullptr)
		return;
	auto const &display{ImGui::GetIO().DisplaySize};
	Rml::Vector2i const size{static_cast<int>(display.x), static_cast<int>(display.y)};
	if (size.x <= 0 || size.y <= 0)
		return;
	if (running->context->GetDimensions() != size)
		running->context->SetDimensions(size);
	running->context->Update();
	auto *viewport{ImGui::GetMainViewport()};
	running->render.begin_frame(ImGui::GetBackgroundDrawList(viewport), viewport->Pos);
	running->context->Render();
	running->render.end_frame();
	running->renderedframe = ImGui::GetFrameCount();
}

void rmlui::show_debugger(bool const Show)
{
	if (active())
		Rml::Debugger::SetVisible(Show);
}

bool rmlui::debugger_shown()
{
	return active() && Rml::Debugger::IsVisible();
}

bool rmlui::on_key(int const Key, int const Action, int const Mods)
{
	auto *context{input_context()};
	if (context == nullptr || false == control_focused(*context))
		return false;
	return false == RmlGLFW::ProcessKeyCallback(context, Key, Action, Mods);
}

bool rmlui::on_char(unsigned int const Codepoint)
{
	auto *context{input_context()};
	if (context == nullptr || false == control_focused(*context))
		return false;
	return false == RmlGLFW::ProcessCharCallback(context, Codepoint);
}

bool rmlui::on_mouse_button(int const Button, int const Action, int const Mods)
{
	auto *context{input_context()};
	if (context == nullptr || false == instance()->cursorvisible)
		return false;
	return false == RmlGLFW::ProcessMouseButtonCallback(context, Button, Action, Mods);
}

bool rmlui::on_scroll(double const Yoffset)
{
	auto *context{input_context()};
	if (context == nullptr || false == instance()->cursorvisible)
		return false;
	return false == RmlGLFW::ProcessScrollCallback(context, Yoffset, 0);
}

void rmlui::on_cursor_pos(double const X, double const Y, bool const Visible)
{
	auto *running{instance().get()};
	if (running == nullptr)
		return;
	// with the cursor hidden the simulator turns the view by it, the documents don't follow it then
	if (false == Visible)
	{
		if (running->cursorvisible)
			running->context->ProcessMouseLeave();
		running->cursorvisible = false;
		return;
	}
	running->cursorvisible = true;
	running->context->ProcessMouseMove(static_cast<int>(std::lround(X)), static_cast<int>(std::lround(Y)), 0);
}

void rmlui::on_cursor_leave()
{
	if (auto *running{instance().get()}; running != nullptr)
	{
		running->context->ProcessMouseLeave();
		running->cursorvisible = false;
	}
}

#else

bool rmlui::init(GLFWwindow *)
{
	return false;
}

void rmlui::shutdown()
{
	// built without RmlUi
}

bool rmlui::active()
{
	return false;
}

Rml::Context *rmlui::context()
{
	return nullptr;
}

void rmlui::render()
{
	// built without RmlUi
}

void rmlui::show_debugger(bool)
{
	// built without RmlUi
}

bool rmlui::debugger_shown()
{
	return false;
}

bool rmlui::on_key(int, int, int)
{
	return false;
}

bool rmlui::on_char(unsigned int)
{
	return false;
}

bool rmlui::on_mouse_button(int, int, int)
{
	return false;
}

bool rmlui::on_scroll(double)
{
	return false;
}

void rmlui::on_cursor_pos(double, double, bool)
{
	// built without RmlUi
}

void rmlui::on_cursor_leave()
{
	// built without RmlUi
}

#endif
