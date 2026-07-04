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
#include "network/manager.h"
#include "utilities/headtrack.h"
#ifdef WITH_UART
#include "utilities/uart.h"
#endif

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
        push_mode( mode Mode );
    void
        set_title( std::string const &Title );
    void
        set_progress( float Progress = 0.f, float Subtaskprogress = 0.f );
    void
        set_tooltip( std::string const &Tooltip );
	static void
        set_cursor( int Mode );
    void
        set_cursor_pos( double Horizontal, double Vertical );
    void queue_screenshot();
    // input handlers
    void on_key( int Key, int Scancode, int Action, int Mods );
	static void on_char( unsigned int Char );
    void on_cursor_pos( double Horizontal, double Vertical );
    void on_mouse_button( int Button, int Action, int Mods );
    void on_scroll( double Xoffset, double Yoffset );
	void on_focus_change(bool focus);
	void on_window_resize(int w, int h);
    // gives access to specified window, creates a new window if index == -1
    GLFWwindow *
        window( int Windowindex = 0, bool visible = false, int width = 1, int height = 1, GLFWmonitor *monitor = nullptr, bool keep_ownership = true, bool share_ctx = true );
    GLFWmonitor * find_monitor( const std::string &str ) const;
	static std::string describe_monitor( GLFWmonitor *monitor );
	// generate network sync verification number
	static double
	    generate_sync();
	void
        queue_quit(bool direct);
    bool
        is_server() const;
    bool
        is_client() const;

private:
// types
    using modeptr_array = std::array<std::shared_ptr<application_mode>, static_cast<std::size_t>( count_ )>;
    using mode_stack = std::stack<mode>;
// methods
	static bool needs_ogl();
    void init_debug();
	static void init_console();
    void init_files();
	static int  init_settings( int Argc, char *Argv[] );
	static int  init_locale();
    int  init_glfw();
    void init_callbacks();
	static int  init_ogl();
    int  init_ui();
    int  init_gfx();
	static int  init_audio();
	static int  init_data();
    int  init_modes();
	bool init_network();
    int run_crashgui();
// members

    bool m_screenshot_queued = false;

    modeptr_array m_modes { nullptr }; // collection of available application behaviour modes
    mode_stack m_modestack; // current behaviour mode
    python_taskqueue m_taskqueue;
    std::vector<GLFWwindow *> m_windows;
    int m_glfwversion;

	std::optional<network::manager> m_network;
    std::optional<headtrack> m_headtrack;
};

extern eu07_application Application;
