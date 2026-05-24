/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <deque>

#include "application/uilayer.h"

struct ImGuiInputTextCallbackData;

namespace dev {

// Parsed command arguments — name of the command is args[0]
using args_t = std::vector<std::string>;

// Handler signature: receives all tokens including the command name at [0]
using command_handler_t = std::function<void(const args_t &)>;

struct console_command {
    std::string name;
    std::string description;
    command_handler_t handler;
};

// -----------------------------------------------------------------------
// console_panel
//   Inherits from ui_panel. Embed as a member in any ui_layer subclass,
//   then call add_external_panel() to register it.
//
//   Toggle visibility with the backtick / grave-accent key (`) handled
//   from ui_layer::on_key().
//
//   Register commands from anywhere via the static global instance:
//       dev::Console.register_command("name", "desc", handler);
// -----------------------------------------------------------------------
class console_panel : public ui_panel {
public:
    console_panel();

    void render() override;
    void render_contents() override;

    // Print a line to the console output
    void print(const std::string &text, glm::vec4 color = {0.9f, 0.9f, 0.9f, 1.0f});
    void print_info(const std::string &text);
    void print_ok(const std::string &text);
    void print_error(const std::string &text);
    void print_warn(const std::string &text);

    // Register a new command. Duplicate names overwrite previous.
    void register_command(std::string name, std::string description, command_handler_t handler);

    // Execute a raw input string (tokenises, looks up, calls handler)
    void execute(const std::string &input);

    // Focus the input field next frame
    void focus_input() { m_want_focus = true; }

private:
    struct log_entry {
        std::string text;
        glm::vec4   color;
    };

    void execute_command(const std::string &name, const args_t &args);
    std::vector<std::string> completions_for(const std::string &prefix) const;

    // ImGui InputText callback — used for history and tab-completion
    static int input_callback(ImGuiInputTextCallbackData *data);

    std::unordered_map<std::string, console_command> m_commands;
    std::deque<log_entry>      m_log;
    std::deque<std::string>    m_history;   // previously executed lines
    int                        m_history_pos { -1 }; // -1 = new input
    char                       m_input_buf[512] {};
    bool                       m_scroll_to_bottom { false };
    bool                       m_want_focus { false };

    static constexpr std::size_t MAX_LOG   = 1024;
    static constexpr std::size_t MAX_HIST  = 128;

    // Built-in commands registered in the constructor
    void register_builtins();
};

// Global singleton — accessible from anywhere after the header is included
extern console_panel Console;

} // namespace dev
