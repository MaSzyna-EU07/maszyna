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
#include <chrono>
#include <cmath>
#include <memory>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

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
		if (Type != Rml::Log::LT_ERROR && Type != Rml::Log::LT_ASSERT && Type != Rml::Log::LT_WARNING && Type != Rml::Log::LT_ALWAYS)
			return true;
		// a message which comes again each frame is written once, the log doesn't grow with it
		if (m_recent.contains(Message))
		{
			++m_repeated;
			return true;
		}
		if (m_recent.size() >= 64)
			m_recent.clear();
		m_recent.insert(Message);
		if (m_repeated > 0)
		{
			WriteLog("rmlui: " + std::to_string(m_repeated) + " messages written before not written again");
			m_repeated = 0;
		}
		if (Type == Rml::Log::LT_ERROR || Type == Rml::Log::LT_ASSERT)
			ErrorLog("rmlui: " + Message);
		else
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

  private:
	std::unordered_set<Rml::String> m_recent; // the messages written lately
	int m_repeated{0}; // of them, not written again since
};

struct state
{
	std::unique_ptr<system_interface> system;
	rmlui::imgui_render_interface render;
	Rml::Context *context{nullptr};
	int renderedframe{-2}; // the frame of imgui the documents were last drawn in
	bool cursorvisible{false};
	bool debugger{false}; // the debugger was started, the first time it was asked for
	std::vector<std::pair<Rml::ElementDocument *, int>> framebound; // the documents shown in a frame only, and the frame
	rmlui::frame_cost cost;

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
	    font_file{"fonts/bahnschrift.ttf", false},
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
	// the screens of the modes not drawn in this frame are gone with their modes
	std::erase_if(running->framebound, [frame = ImGui::GetFrameCount()](std::pair<Rml::ElementDocument *, int> const &Entry) {
		if (Entry.second == frame)
			return false;
		Entry.first->Hide();
		return true;
	});
	using clock = std::chrono::steady_clock;
	auto const start{clock::now()};
	running->context->Update();
	auto const updated{clock::now()};
	auto *viewport{ImGui::GetMainViewport()};
	running->render.begin_frame(ImGui::GetBackgroundDrawList(viewport), viewport->Pos);
	running->context->Render();
	running->render.end_frame();
	auto const drawn{clock::now()};
	running->renderedframe = ImGui::GetFrameCount();
	// averaged over about the last 30 frames
	auto const milliseconds = [](clock::duration const Duration) { return std::chrono::duration<float, std::milli>(Duration).count(); };
	auto &cost{running->cost};
	cost.update_ms += (milliseconds(updated - start) - cost.update_ms) * 0.033f;
	cost.draw_ms += (milliseconds(drawn - updated) - cost.draw_ms) * 0.033f;
	cost.vertices = running->render.vertices();
}

rmlui::frame_cost rmlui::cost()
{
	auto const *running{instance().get()};
	return running != nullptr ? running->cost : frame_cost{};
}

void rmlui::show_in_frame(Rml::ElementDocument *Document)
{
	auto *running{instance().get()};
	if (running == nullptr || Document == nullptr || ImGui::GetCurrentContext() == nullptr)
		return;
	auto const frame{ImGui::GetFrameCount()};
	auto const entry{std::ranges::find(running->framebound, Document, &std::pair<Rml::ElementDocument *, int>::first)};
	if (entry != running->framebound.end())
		entry->second = frame;
	else
		running->framebound.emplace_back(Document, frame);
	if (false == Document->IsVisible())
		Document->Show(Rml::ModalFlag::None, Rml::FocusFlag::None);
}

void rmlui::forget(Rml::ElementDocument *Document)
{
	if (auto *running{instance().get()}; running != nullptr)
		std::erase_if(running->framebound, [Document](std::pair<Rml::ElementDocument *, int> const &Entry) { return Entry.first == Document; });
}

void rmlui::show_debugger(bool const Show)
{
	auto *running{instance().get()};
	if (running == nullptr)
		return;
	// started when it's first asked for: until then it doesn't gather the messages and the events of the documents
	if (false == running->debugger)
	{
		if (false == Show)
			return;
		running->debugger = Rml::Debugger::Initialise(running->context);
	}
	Rml::Debugger::SetVisible(Show);
}

bool rmlui::debugger_shown()
{
	auto const *running{instance().get()};
	return running != nullptr && running->debugger && Rml::Debugger::IsVisible();
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

void rmlui::show_in_frame(Rml::ElementDocument *)
{
	// built without RmlUi
}

rmlui::frame_cost rmlui::cost()
{
	return {};
}

void rmlui::forget(Rml::ElementDocument *)
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
