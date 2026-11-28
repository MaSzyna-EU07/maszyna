/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/applicationmode.h"
#include "scripting/PyInt.h"
#include "scripting/scriptscreens.h"
#include "network/manager.h"
#include "utilities/headtrack.h"

#include <atomic>
#include <thread>
#ifdef WITH_UART
#include "utilities/uart.h"
#endif

struct gfx_backend_desc;

class eu07_application {
    const int MAX_NETWORK_PER_FRAME = 1000;

public:
// types
    enum mode {
		launcher = 0,
        scenarioloader,
        driver,
        editor,
        count_
    };
// constructors
    eu07_application() = default;
// methods
    // 0 when the simulator started, below 0 when its start failed, above 0 when the player closed it before it started
    int
        init( int Argc, char *Argv[] );
    int
        run();
    // issues request for a worker thread to perform specified task. returns: true if task was scheduled

    #ifdef WITH_DISCORD_RPC
    void DiscordRPCService(); // discord rich presence service function (runs as separate thread)
    #endif
    bool
        request( python_taskqueue::task_request const &Task );
    // ensures the main thread holds the python gil and can safely execute python calls
    void
        acquire_python_lock();
    // frees the python gil and swaps out the main thread
    void
        release_python_lock();
    // the screens drawn by scripts (Lua, WebAssembly)
    script_screen_queue &
        screens() {
            return m_screenqueue; }
    void
        exit();
    void
        render_ui();
	void
	    begin_ui_frame();
    // switches application to specified mode
    bool
        pop_mode();
    bool
        push_mode( eu07_application::mode const Mode );
    void
        set_title( std::string const &Title );
    // any thread: what the thread loading the scenery in the background reports is shown from the next frame of the main thread
    void
        set_progress( float const Progress = 0.f, float const Subtaskprogress = 0.f );
    void
        set_tooltip( std::string const &Tooltip );
    void
        set_cursor( int const Mode );
    void
        set_cursor_pos( double const Horizontal, double const Vertical );
    void queue_screenshot();
    // input handlers
    void on_key( int const Key, int const Scancode, int const Action, int const Mods );
    void on_char( unsigned int const Char );
    void on_cursor_pos( double const Horizontal, double const Vertical );
    void on_mouse_button( int const Button, int const Action, int const Mods );
    void on_scroll( double const Xoffset, double const Yoffset );
	void on_focus_change(bool focus);
	void on_window_resize(int w, int h);
    // gives access to specified window, creates a new window if index == -1
    GLFWwindow *
        window( int const Windowindex = 0, bool visible = false, int width = 1, int height = 1, GLFWmonitor *monitor = nullptr, bool keep_ownership = true, bool share_ctx = true );
    GLFWmonitor * find_monitor( const std::string &str ) const;
    std::string describe_monitor( GLFWmonitor *monitor ) const;
	// generate network sync verification number
	double
	    generate_sync();
	void
        queue_quit(bool direct);
    bool
        is_server() const;
    bool
        is_client() const;

private:
// types
    using modeptr_array = std::array<std::shared_ptr<application_mode>, static_cast<std::size_t>( mode::count_ )>;
    using mode_stack = std::stack<mode>;
// methods
    void init_debug();
    void init_console();
    void init_files();
    int  init_settings( int Argc, char *Argv[] );
    int  init_locale();
    int  init_glfw();
    // the backend of the renderer picked by gfxrenderer, opengl if it can't run here
    void select_gfx_backend();
    int  create_main_window();
    // the backend picked can't run after all: the main window is made anew for opengl
    int  fallback_to_opengl( std::string const &Reason );
    void init_callbacks();
    int  init_ui();
    int  init_gfx();
    int  init_audio();
    int  init_data();
    int  init_modes();
	bool init_network();
    int run_crashgui();
    // the progress reported by another thread (set_progress()) given to the mode
    void apply_progress();
// members

    // the progress of the scenery loaded by a thread of its own, for the main thread to show (set_progress())
    std::thread::id m_mainthread { std::this_thread::get_id() };
    std::atomic<float> m_pendingprogress { 0.f };
    std::atomic<float> m_pendingsubtaskprogress { 0.f };
    std::atomic<bool> m_progresspending { false };

    bool m_screenshot_queued = false;
    // the ui of the crash gui, drawn in the frames of the renderer instead of the ui of the modes while it's there
    std::function<void()> m_crashui;

    modeptr_array m_modes { nullptr }; // collection of available application behaviour modes
    mode_stack m_modestack; // current behaviour mode
    python_taskqueue m_taskqueue;
    script_screen_queue m_screenqueue;
    std::vector<GLFWwindow *> m_windows;
    gfx_backend_desc const *m_gfxbackend { nullptr }; // owned by gfx_renderer_factory

	std::optional<network::manager> m_network;
    std::optional<headtrack> m_headtrack;
};

extern eu07_application Application;
