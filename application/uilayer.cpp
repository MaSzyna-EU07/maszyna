/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/uilayer.h"

#include <utility>

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"
#include "simulation/simulation.h"
#include "utilities/translation.h"
#include "application/application.h"
#include "application/editormode.h"

#include "imgui/imgui_impl_glfw.h"

GLFWwindow *ui_layer::m_window{nullptr};
GLFWwindow *ui_layer::m_keywindow{nullptr};
ImGuiIO *ui_layer::m_imguiio{nullptr};
GLint ui_layer::m_textureunit{GL_TEXTURE0};
bool ui_layer::m_cursorvisible;
ImFont *ui_layer::font_default{nullptr};
ImFont *ui_layer::font_mono{nullptr};
ImFont *ui_layer::font_loading{nullptr};

ui_panel::ui_panel(std::string Identifier, bool const Isopen) : is_open(Isopen), m_name(std::move(Identifier)) {}

void ui_panel::render()
{
	if (false == is_open)
		return;

	int flags = window_flags;
	if (flags == -1)
		flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoCollapse;
	if (size.x > 0)
		flags |= ImGuiWindowFlags_NoResize;
	if (no_title_bar)
		flags |= ImGuiWindowFlags_NoTitleBar;

	if (pos.x != -1 && pos.y != -1)
		ImGui::SetNextWindowPos(ImVec2(pos.x, pos.y), ImGuiCond_Always);
	if (size.x > 0)
		ImGui::SetNextWindowSize(ImVec2S(size.x, size.y), ImGuiCond_Always);
	else if (size_min.x == -1)
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiCond_FirstUseEver);

	if (size_min.x > 0)
		ImGui::SetNextWindowSizeConstraints(ImVec2S(size_min.x, size_min.y), ImVec2S(size_max.x, size_max.y));

	auto const panelname{(title.empty() ? m_name : title) + "###" + m_name};
	if (ImGui::Begin(panelname.c_str(), &is_open, flags))
	{
		render_contents();

		popups.remove_if([](const std::unique_ptr<ui::popup> &popup) { return popup->render(); });
	}

	ImGui::End();
}

void ui_panel::render_contents()
{
	for (auto const &line : text_lines)
	{
		ImGui::TextColored(ImVec4(line.color.r, line.color.g, line.color.b, line.color.a), line.data.c_str());
	}
}

void ui_panel::register_popup(std::unique_ptr<ui::popup> &&popup)
{
	popups.push_back(std::move(popup));
}

void ui_expandable_panel::render_contents()
{
	ImGui::Checkbox(STR_C("expand"), &is_expanded);
	ui_panel::render_contents();
}

void ui_log_panel::render_contents()
{
	ImGui::PushFont(ui_layer::font_mono);

	for (const std::string &s : log_scrollback)
		ImGui::TextUnformatted(s.c_str());
	if (ImGui::GetScrollY() == ImGui::GetScrollMaxY())
		ImGui::SetScrollHereY(1.0f);

	ImGui::PopFont();
}

ui_layer::ui_layer()
{
	if (Global.loading_log)
		add_external_panel(&m_logpanel);
	m_logpanel.size = {700, 400};
}

ui_layer::~ui_layer() {}

bool ui_layer::key_callback(int key, int scancode, int action, int mods)
{
	ImGui_ImplGlfw_KeyCallback(m_keywindow != nullptr ? m_keywindow : m_window, key, scancode, action, mods);
	return m_imguiio->WantCaptureKeyboard;
}

bool ui_layer::char_callback(unsigned int c)
{
	ImGui_ImplGlfw_CharCallback(m_window, c);
	return m_imguiio->WantCaptureKeyboard;
}

bool ui_layer::scroll_callback(double xoffset, double yoffset)
{
	ImGui_ImplGlfw_ScrollCallback(m_window, xoffset, yoffset);
	return m_imguiio->WantCaptureMouse;
}

bool ui_layer::mouse_button_callback(int button, int action, int mods)
{
	ImGui_ImplGlfw_MouseButtonCallback(m_window, button, action, mods);
	return m_imguiio->WantCaptureMouse;
}

// the glfw backend of imgui works with events since 1.87: position, hover and focus of the window are passed to it as they come
void ui_layer::cursor_pos_callback(double x, double y)
{
	if (ImGui::GetCurrentContext() == nullptr)
		return;
	ImGui_ImplGlfw_CursorPosCallback(m_window, x, y);
}

void ui_layer::cursor_enter_callback(int entered)
{
	if (ImGui::GetCurrentContext() == nullptr)
		return;
	ImGui_ImplGlfw_CursorEnterCallback(m_window, entered);
}

void ui_layer::focus_callback(int focused)
{
	if (ImGui::GetCurrentContext() == nullptr)
		return;
	ImGui_ImplGlfw_WindowFocusCallback(m_window, focused);
}

void ui_layer::viewport_key_callback(GLFWwindow *Window, int key, int scancode, int action, int mods)
{
	// imgui gets the key with the window it came from (the state of modifiers is read from it), the simulator gets it unless imgui takes it
	m_keywindow = Window;
	Application.on_key(key, scancode, action, mods);
	m_keywindow = nullptr;
}

namespace
{
// window creation of the glfw backend, wrapped to give the new windows the key callback of the simulator
void (*imgui_create_window)(ImGuiViewport *Viewport){nullptr};

void create_viewport_window(ImGuiViewport *Viewport)
{
	imgui_create_window(Viewport);
	if (Viewport->PlatformHandle != nullptr)
		glfwSetKeyCallback(static_cast<GLFWwindow *>(Viewport->PlatformHandle), ui_layer::viewport_key_callback);
}
} // namespace

void ui_layer::imgui_style()
{
	// palette shared with the MaSzyna starter: flat dark panels, thin borders, green accents
	auto const rgb = [](int const Color, float const Alpha = 1.0f) {
		return ImVec4(((Color >> 16) & 0xff) / 255.0f, ((Color >> 8) & 0xff) / 255.0f, (Color & 0xff) / 255.0f, Alpha);
	};
	int constexpr panel{0x1e2327}; // window background
	int constexpr base{0x15171b}; // darker background, between panels
	int constexpr header{0x2a3036}; // section headers, hovered items
	int constexpr border{0x3a424a};
	int constexpr field{0x121517}; // input fields, combos
	int constexpr accent{0x177f00}; // selection, active elements
	int constexpr accentbright{0x41c400}; // marks: checkmarks, active tab underline
	int constexpr accentdim{0x164b0e};

	ImVec4 *colors = ImGui::GetStyle().Colors;

	colors[ImGuiCol_Text] = rgb(0xffffff);
	colors[ImGuiCol_TextDisabled] = rgb(0xa0a0a2);
	colors[ImGuiCol_WindowBg] = rgb(panel, Global.UIBgOpacity); // ui.bg.opacity from config file
	colors[ImGuiCol_ChildBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	colors[ImGuiCol_PopupBg] = rgb(base, 0.98f);
	colors[ImGuiCol_Border] = rgb(border);
	colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	colors[ImGuiCol_FrameBg] = rgb(field);
	colors[ImGuiCol_FrameBgHovered] = rgb(header);
	colors[ImGuiCol_FrameBgActive] = rgb(border);
	colors[ImGuiCol_TitleBg] = rgb(base);
	colors[ImGuiCol_TitleBgActive] = rgb(header);
	colors[ImGuiCol_TitleBgCollapsed] = rgb(base, 0.75f);
	colors[ImGuiCol_MenuBarBg] = rgb(panel);
	colors[ImGuiCol_ScrollbarBg] = rgb(base, 0.60f);
	colors[ImGuiCol_ScrollbarGrab] = rgb(border);
	colors[ImGuiCol_ScrollbarGrabHovered] = rgb(0x5a636c);
	colors[ImGuiCol_ScrollbarGrabActive] = rgb(0x858585);
	colors[ImGuiCol_CheckMark] = rgb(accentbright);
	colors[ImGuiCol_SliderGrab] = rgb(accent);
	colors[ImGuiCol_SliderGrabActive] = rgb(accentbright);
	colors[ImGuiCol_Button] = rgb(header);
	colors[ImGuiCol_ButtonHovered] = rgb(border);
	colors[ImGuiCol_ButtonActive] = rgb(accent);
	// shared by section headers and selected list items
	colors[ImGuiCol_Header] = rgb(header);
	colors[ImGuiCol_HeaderHovered] = rgb(accentdim);
	colors[ImGuiCol_HeaderActive] = rgb(accent);
	colors[ImGuiCol_Separator] = rgb(border);
	colors[ImGuiCol_SeparatorHovered] = rgb(accent);
	colors[ImGuiCol_SeparatorActive] = rgb(accentbright);
	colors[ImGuiCol_ResizeGrip] = rgb(border, 0.50f);
	colors[ImGuiCol_ResizeGripHovered] = rgb(accent);
	colors[ImGuiCol_ResizeGripActive] = rgb(accentbright);
	colors[ImGuiCol_Tab] = rgb(header);
	colors[ImGuiCol_TabHovered] = rgb(border);
	colors[ImGuiCol_TabActive] = rgb(accent); // also the line under the tab bar
	colors[ImGuiCol_TabUnfocused] = rgb(header);
	colors[ImGuiCol_TabUnfocusedActive] = rgb(accentdim);
	// imgui 1.90+ marks the selected tab with a line over it as well, in the same colours
	colors[ImGuiCol_TabSelectedOverline] = rgb(accent);
	colors[ImGuiCol_TabDimmedSelectedOverline] = rgb(accentdim);
	colors[ImGuiCol_PlotLines] = ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
	colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
	colors[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
	colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
	colors[ImGuiCol_TextSelectedBg] = rgb(accent, 0.60f);
	colors[ImGuiCol_DragDropTarget] = rgb(accentbright, 0.90f);
	colors[ImGuiCol_NavHighlight] = rgb(accentbright);
	colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
	colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
	colors[ImGuiCol_ModalWindowDimBg] = rgb(base, 0.60f);

	// flat, square elements with thin borders
	auto &style = ImGui::GetStyle();
	style.WindowRounding = 0.0f;
	style.ChildRounding = 0.0f;
	style.FrameRounding = 0.0f;
	style.PopupRounding = 0.0f;
	style.ScrollbarRounding = 0.0f;
	style.GrabRounding = 0.0f;
	style.TabRounding = 0.0f;
	style.WindowBorderSize = 1.0f;
	style.ChildBorderSize = 1.0f;
	style.PopupBorderSize = 1.0f;
	style.FrameBorderSize = 1.0f;
	style.TabBorderSize = 0.0f;

	style.ScaleAllSizes(Global.ui_scale);
}

bool ui_layer::init(GLFWwindow *Window)
{
    m_window = Window;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    m_imguiio = &ImGui::GetIO();

	m_imguiio->ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
    // m_imguiio->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // m_imguiio->ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    static const ImWchar ranges[] =
    {
        0x0020, 0x00FF, // Basic Latin + Latin Supplement
        0x0100, 0x017F, // Latin Extended-A
        0x2070, 0x2079, // superscript
        0x2500, 0x256C, // box drawings
        0,
    };

    ImFontConfig config;
	// config.FontDataOwnedByAtlas = false;  // Avoid duplicate release of font data

	if (FileExists("fonts/dejavusans.ttf")) {
		// Load basic font first
		font_default = m_imguiio->Fonts->AddFontFromFileTTF("fonts/dejavusans.ttf", Global.ui_fontsize, &config, &ranges[0]);
		
		// Add support for Chinese character ranges
		if (font_default) {
			ImFontConfig chinese_config;
			chinese_config.MergeMode = true;
			chinese_config.PixelSnapH = true;
			chinese_config.OversampleH = 1;
			chinese_config.OversampleV = 1;
			// Use NotoSansSC-Regular.ttf font file
			if (FileExists("fonts/NotoSansSC-Regular.ttf")) {
				const ImWchar* chinese_ranges = m_imguiio->Fonts->GetGlyphRangesChineseFull();
				m_imguiio->Fonts->AddFontFromFileTTF("fonts/NotoSansSC-Regular.ttf", Global.ui_fontsize, &chinese_config, chinese_ranges);
			}
		}
	}
	
	if (FileExists("fonts/dejavusansmono.ttf")) {
		ImFontConfig mono_config;
		mono_config.GlyphRanges = &ranges[0]; // Basic Latin characters
		font_mono = m_imguiio->Fonts->AddFontFromFileTTF("fonts/dejavusansmono.ttf", Global.ui_fontsize, &mono_config, &ranges[0]);
		
		// Add Chinese character support for monospace font as well
		if (font_mono) {
			ImFontConfig chinese_mono_config;
			chinese_mono_config.OversampleH = 1;
			chinese_mono_config.OversampleV = 1;
			chinese_mono_config.MergeMode = true;
			chinese_mono_config.PixelSnapH = true;
			
			// Use NotoSansSC-Regular.ttf for monospace font as well
			if (FileExists("fonts/NotoSansSC-Regular.ttf")) {
				const ImWchar* chinese_ranges = m_imguiio->Fonts->GetGlyphRangesChineseFull();
				m_imguiio->Fonts->AddFontFromFileTTF("fonts/NotoSansSC-Regular.ttf", Global.ui_fontsize, &chinese_mono_config, chinese_ranges);
			}
		}
	}
	if (FileExists("fonts/bahnschrift.ttf")) {
		ImFontConfig loading_config;
		font_loading = m_imguiio->Fonts->AddFontFromFileTTF("fonts/bahnschrift.ttf", 48, &loading_config, &ranges[0]);
		
		// Add Chinese character support for loading font as well
		if (font_loading) {
			ImFontConfig chinese_loading_config;
			chinese_loading_config.OversampleH = 1;
			chinese_loading_config.OversampleV = 1;
			chinese_loading_config.MergeMode = true;
			chinese_loading_config.PixelSnapH = true;
			
			// Use NotoSansMonoCJKsc-Regular.ttf for loading font
			if (FileExists("fonts/NotoSansMonoCJKsc-Regular.ttf")) {
				const ImWchar* chinese_ranges = m_imguiio->Fonts->GetGlyphRangesChineseFull();
				m_imguiio->Fonts->AddFontFromFileTTF("fonts/NotoSansMonoCJKsc-Regular.ttf", 48, &chinese_loading_config, chinese_ranges);
			}
		}
	}

	if (!font_default && !font_mono)
		font_default = font_mono = m_imguiio->Fonts->AddFontDefault();
	else if (!font_default)
		font_default = font_mono;
	else if (!font_mono)
		font_mono = font_default;
	if (!font_loading)
		font_loading = font_default;

	imgui_style();

    ImGui_ImplGlfw_InitForOpenGL(m_window, false);

    if (!GfxRenderer->GetImguiRenderer() || !GfxRenderer->GetImguiRenderer()->Init())
	  {
		  return false;
	  }

	// panels can be docked together, and dragged out of the simulator window into windows of their own (e.g. on another monitor)
	// when both backends can do it. glfw can't place windows under wayland. both flags have to be set before the first frame
	m_imguiio->ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	auto const viewportsupport{(m_imguiio->BackendFlags & ImGuiBackendFlags_PlatformHasViewports) && (m_imguiio->BackendFlags & ImGuiBackendFlags_RendererHasViewports) &&
	                           glfwGetPlatform() != GLFW_PLATFORM_WAYLAND};
	if (Global.ui_viewports && viewportsupport)
	{
		m_imguiio->ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
		auto &platformio{ImGui::GetPlatformIO()};
		imgui_create_window = platformio.Platform_CreateWindow;
		platformio.Platform_CreateWindow = create_viewport_window;
	}
	WriteLog(std::string("ui: panels in separate windows ") + ((m_imguiio->ConfigFlags & ImGuiConfigFlags_ViewportsEnable) ? "enabled" : (viewportsupport ? "disabled in settings" : "not supported here")));

    return true;
}

void ui_layer::shutdown()
{
	ImGui::EndFrame();

	GfxRenderer->GetImguiRenderer()->Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

bool ui_layer::on_key(int const Key, int const Action)
{
	if (Action == GLFW_PRESS)
	{
		if (Key == GLFW_KEY_PRINT_SCREEN)
		{
			Application.queue_screenshot();
			return true;
		}

		if (Key == GLFW_KEY_F9)
		{
			m_logpanel.is_open = !m_logpanel.is_open;
			return true;
		}

		if (Key == GLFW_KEY_F10)
		{
			m_quit_active = !m_quit_active;
			return true;
		}

		if (m_quit_active)
		{
			if (Key == GLFW_KEY_Y)
			{
				Application.queue_quit(false);
				return true;
			}
			else if (Key == GLFW_KEY_N)
			{
				m_quit_active = false;
				return true;
			}
		}
	}

	return false;
}

bool ui_layer::on_cursor_pos(double const Horizontal, double const Vertical)
{
	return false;
}

bool ui_layer::on_mouse_button(int const Button, int const Action)
{
	return false;
}

void ui_layer::on_window_resize(int w, int h)
{
	for (auto *panel : m_panels)
		panel->on_window_resize(w, h);
}

void ui_layer::update()
{
	for (auto *panel : m_panels)
		panel->update();

	for (auto it = m_ownedpanels.rbegin(); it != m_ownedpanels.rend(); it++)
	{
		(*it)->update();
		if (!(*it)->is_open)
			m_ownedpanels.erase(std::next(it).base());
	}
}

void ui_layer::render()
{
	render_background();
	render_panels();
	render_tooltip();
	render_menu();
	render_quit_widget();
	render_hierarchy();
	// template method implementation
	render_();

	render_internal();
}

void ui_layer::render_internal()
{
	ImGui::Render();
	GfxRenderer->GetImguiRenderer()->Render();

	if (m_imguiio->ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
	{
		// panels outside of the main window are drawn in their own windows, whose gl contexts are shared with the main one
		auto *context{glfwGetCurrentContext()};
		ImGui::UpdatePlatformWindows();
		ImGui::RenderPlatformWindowsDefault();
		glfwMakeContextCurrent(context);
	}
}

void ui_layer::begin_ui_frame()
{
	begin_ui_frame_internal();
}

void ui_layer::begin_ui_frame_internal()
{
	GfxRenderer->GetImguiRenderer()->BeginFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void ui_layer::render_quit_widget()
{
	if (!m_quit_active)
		return;

	ImGui::SetNextWindowSize(ImVec2(0, 0));
	ImGui::Begin(STR_C("Quit"), &m_quit_active, ImGuiWindowFlags_NoResize);
	ImGui::TextUnformatted(STR_C("Quit simulation?"));
	if (ImGui::Button(STR_C("Yes")))
		Application.queue_quit(false);

	ImGui::SameLine();
	if (ImGui::Button(STR_C("No")))
		m_quit_active = false;
	ImGui::End();
}

void ui_layer::render_hierarchy(){
	if(!m_editor_hierarchy)
		return;

	ImGui::SetNextWindowSize(ImVec2(0, 0));
	ImGui::Begin(STR_C("Scene Hierarchy"), &m_editor_hierarchy, ImGuiWindowFlags_AlwaysAutoResize);
	ImGui::Text("Registered nodes: %zu", scene::Hierarchy.size());
    ImGui::BeginChild("hierarchy_list", ImVec2(500, 300), true);

	for (auto &entry : scene::Hierarchy)
        {
        	const std::string &uuid = entry.first;
            scene::basic_node *node = entry.second;

            if (node)
            {
                char buf[512];
                std::snprintf(buf, sizeof(buf), "%s | %s (%.1f, %.1f, %.1f)",
                            node->name().c_str(),
                            uuid.c_str(),
                            node->location().x,
                            node->location().y,
                            node->location().z);

                if (ImGui::Selectable(buf, false))
                {
                    // Focus camera on selected node
                    auto const node_pos = node->location();
                    auto const camera_offset = glm::dvec3(0.0, 10.0, -20.0);
					editor_mode::set_focus_active(false);
					TCamera &camera = editor_mode::get_camera(); 
					camera.Pos = node_pos + camera_offset;
				
                }

                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("UUID: %s\nNode type: %s", uuid.c_str(), typeid(*node).name());
                }
            }
        }

    ImGui::EndChild();
    ImGui::End();

}
 
void ui_layer::set_cursor(int const Mode)
{
	glfwSetInputMode(m_window, GLFW_CURSOR, Mode);
	m_cursorvisible = Mode != GLFW_CURSOR_DISABLED;
}

void ui_layer::set_progress(std::string const &Text)
{
	m_progresstext = Text;
}

void ui_layer::set_progress(float const progress, float const subtaskprogress)
{
	m_progress = progress * 0.01f;
	m_subtaskprogress = subtaskprogress * 0.01f;
}

void ui_layer::set_background(std::string const &Filename)
{
	m_background = Filename.empty() ? null_handle : GfxRenderer->Fetch_Texture(Filename);
}

void ui_layer::clear_panels()
{
	m_panels.clear();
	m_ownedpanels.clear();
}

void ui_layer::add_owned_panel(ui_panel *Panel)
{
	for (auto &panel : m_ownedpanels)
		if (panel->name() == Panel->name())
		{
			delete Panel;
			return;
		}

	Panel->is_open = true;
	m_ownedpanels.emplace_back(Panel);
}

void ui_layer::render_panels()
{
	for (auto *panel : m_panels)
		panel->render();
	for (auto &panel : m_ownedpanels)
		panel->render();

	if (m_imgui_demo)
		ImGui::ShowDemoWindow(&m_imgui_demo);
}

void ui_layer::render_tooltip()
{
	if (!m_cursorvisible || m_imguiio->WantCaptureMouse || m_tooltip.empty())
		return;

	ImGui::BeginTooltip();
	ImGui::TextUnformatted(m_tooltip.c_str());
	ImGui::EndTooltip();
}

void ui_layer::render_menu_contents()
{
	if (ImGui::BeginMenu(STR_C("General")))
	{
		bool flag = DebugModeFlag;
		if (ImGui::MenuItem(STR_C("Debug mode"), nullptr, &flag))
		{
			command_relay relay;
			relay.post(user_command::debugtoggle, 0.0, 0.0, GLFW_RELEASE, 0);
		}
		ImGui::MenuItem(STR_C("Quit"), "F10", &m_quit_active);
		ImGui::EndMenu();
	}
	if (ImGui::BeginMenu(STR_C("Tools")))
	{
		static bool log = Global.iWriteLogEnabled & 1;

		ImGui::MenuItem(STR_C("Logging to log.txt"), nullptr, &log);
		if (log)
			Global.iWriteLogEnabled |= 1;
		else
			Global.iWriteLogEnabled &= ~1;

		if (ImGui::MenuItem(STR_C("Screenshot"), "PrtScr"))
			Application.queue_screenshot();
		ImGui::EndMenu();
	}
	if (ImGui::BeginMenu(STR_C("Windows")))
	{
		ImGui::MenuItem(STR_C("Log"), "F9", &m_logpanel.is_open);
		if (DebugModeFlag)
		{
			ImGui::MenuItem(STR_C("ImGui Demo"), nullptr, &m_imgui_demo);
			bool ret = ImGui::MenuItem(STR_C("Headlight config"), nullptr, GfxRenderer->Debug_Ui_State(std::nullopt));

			GfxRenderer->Debug_Ui_State(ret);
		}
		if(EditorModeFlag){
			ImGui::MenuItem("Hierarchy", nullptr, &m_editor_hierarchy);
			bool change_history_enabled = editor_mode::change_history();
			if (ImGui::MenuItem("Change History", nullptr, &change_history_enabled))
			{
				editor_mode::set_change_history(change_history_enabled);
			}
			bool settings_open = editor_mode::settings_open();
			if (ImGui::MenuItem("Editor Settings", nullptr, &settings_open))
			{
				editor_mode::set_settings_open(settings_open);
			}
		}
		ImGui::EndMenu();
	}
}

void ui_layer::render_menu()
{
	glm::dvec2 mousepos = Global.cursor_pos;

	if (!((Global.ControlPicking && mousepos.y < 50.0f) || m_imguiio->WantCaptureMouse) || m_suppress_menu)
		return;

	if (ImGui::BeginMainMenuBar())
	{
		render_menu_contents();
		ImGui::EndMainMenuBar();
	}
}

void ui_layer::render_background()
{
	if (m_background == 0)
		return;

	ImVec2 display_size = ImGui::GetIO().DisplaySize;
	ITexture &tex = GfxRenderer->Texture(m_background);
	tex.create();

	float tex_w = (float)tex.get_width();
	float tex_h = (float)tex.get_height();

	// skalowanie "cover" – wypełnia cały ekran, zachowując proporcje
	float scale_factor = display_size.x / display_size.y > tex_w / tex_h ? display_size.x / tex_w : display_size.y / tex_h;

	ImVec2 image_size(tex_w * scale_factor, tex_h * scale_factor);

	// wyśrodkowanie obrazka
	ImVec2 start_position((display_size.x - image_size.x) * 0.5f, (display_size.y - image_size.y) * 0.5f);
	ImVec2 end_position(start_position.x + image_size.x, start_position.y + image_size.y);

	// obrazek jest odwrócony w pionie – odwracamy UV
	ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())->AddImage((ImTextureID)(intptr_t)(tex.get_id()), start_position, end_position, ImVec2(0, 1), ImVec2(1, 0));
}
