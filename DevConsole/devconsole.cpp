/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "DevConsole/devconsole.h"

#include <sstream>
#include <algorithm>
#include <cctype>

#include "utilities/Globals.h"
#include "utilities/Logs.h"

namespace dev {

// -----------------------------------------------------------------------
// Global singleton
// -----------------------------------------------------------------------
console_panel Console;

// -----------------------------------------------------------------------
// helpers
// -----------------------------------------------------------------------
static std::string str_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

// Tokenise a command line. Handles "quoted tokens" as a single arg.
static args_t tokenise(const std::string &line)
{
    args_t tokens;
    std::istringstream ss(line);
    std::string tok;
    while (ss >> std::quoted(tok))
        tokens.push_back(tok);
    return tokens;
}

// -----------------------------------------------------------------------
// console_panel
// -----------------------------------------------------------------------
console_panel::console_panel()
    : ui_panel("DevConsole", false)
{
    title = "Developer Console";

    register_builtins();
}

void console_panel::register_builtins()
{
    register_command("help", "List all commands, or describe one: help [command]",
        [this](const args_t &args) {
            if (args.size() >= 2) {
                auto it = m_commands.find(str_tolower(args[1]));
                if (it == m_commands.end()) {
                    print_error("Unknown command: " + args[1]);
                } else {
                    print_ok(it->second.name + " — " + it->second.description);
                }
                return;
            }
            print_info("Available commands:");
            std::vector<std::string> names;
            names.reserve(m_commands.size());
            for (auto &kv : m_commands)
                names.push_back(kv.first);
            std::sort(names.begin(), names.end());
            for (auto &n : names) {
                std::string line = "  " + n;
                auto desc = m_commands[n].description;
                if (!desc.empty())
                    line += " — " + desc;
                print(line);
            }
        });

    register_command("clear", "Clear the console output",
        [this](const args_t &) {
            m_log.clear();
        });

    register_command("ping", "Responds with pong",
        [this](const args_t &) {
            print_ok("pong");
        });

    register_command("echo", "Print arguments back: echo [text...]",
        [this](const args_t &args) {
            std::string msg;
            for (std::size_t i = 1; i < args.size(); ++i) {
                if (i > 1) msg += ' ';
                msg += args[i];
            }
            print(msg);
        });

    register_command("history", "Show command history",
        [this](const args_t &) {
            if (m_history.empty()) {
                print_info("(history is empty)");
                return;
            }
            int idx = 0;
            for (auto &h : m_history)
                print("  " + std::to_string(idx++) + ": " + h);
        });
}

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------
void console_panel::register_command(std::string name, std::string description,
                                     command_handler_t handler)
{
    std::string key = str_tolower(name);
    m_commands[key] = console_command{ std::move(name), std::move(description),
                                        std::move(handler) };
}

void console_panel::print(const std::string &text, glm::vec4 color)
{
    // Split multi-line strings into separate entries
    std::istringstream ss(text);
    std::string line;
    while (std::getline(ss, line))
        m_log.push_back({ line, color });

    while (m_log.size() > MAX_LOG)
        m_log.pop_front();

    m_scroll_to_bottom = true;
}

void console_panel::print_info(const std::string &text)
{
    print("[info] " + text, { 0.6f, 0.8f, 1.0f, 1.0f });
}

void console_panel::print_ok(const std::string &text)
{
    print("[ok] " + text, { 0.42f, 0.80f, 0.23f, 1.0f });
}

void console_panel::print_error(const std::string &text)
{
    print("[error] " + text, { 1.0f, 0.35f, 0.35f, 1.0f });
}

void console_panel::print_warn(const std::string &text)
{
    print("[warn] " + text, { 1.0f, 0.80f, 0.20f, 1.0f });
}

void console_panel::execute(const std::string &input)
{
    std::string trimmed = input;
    // ltrim / rtrim
    trimmed.erase(0, trimmed.find_first_not_of(" \t\r\n"));
    trimmed.erase(trimmed.find_last_not_of(" \t\r\n") + 1);

    if (trimmed.empty())
        return;

    // Echo the input
    print("> " + trimmed, { 0.42f, 0.64f, 0.23f, 1.0f });

    // Save to history (no duplicates at the end)
    if (m_history.empty() || m_history.back() != trimmed)
        m_history.push_back(trimmed);
    if (m_history.size() > MAX_HIST)
        m_history.pop_front();
    m_history_pos = -1;

    auto tokens = tokenise(trimmed);
    if (tokens.empty())
        return;

    std::string cmd_name = str_tolower(tokens[0]);
    execute_command(cmd_name, tokens);
}

void console_panel::execute_command(const std::string &name, const args_t &args)
{
    auto it = m_commands.find(name);
    if (it == m_commands.end()) {
        print_error("Unknown command '" + name + "'. Type 'help' for a list.");
        return;
    }
    try {
        it->second.handler(args);
    }
    catch (const std::exception &e) {
        print_error(std::string("Exception: ") + e.what());
    }
}

std::vector<std::string> console_panel::completions_for(const std::string &prefix) const
{
    std::string lp = str_tolower(prefix);
    std::vector<std::string> result;
    for (auto &kv : m_commands)
        if (kv.first.substr(0, lp.size()) == lp)
            result.push_back(kv.first);
    std::sort(result.begin(), result.end());
    return result;
}

// -----------------------------------------------------------------------
// ImGui InputText callback
// -----------------------------------------------------------------------
struct CallbackUserData {
    console_panel *self;
    // current text in the buffer at callback time
    std::string    completion_prefix;
};

int console_panel::input_callback(ImGuiInputTextCallbackData *data)
{
    auto *ud = static_cast<CallbackUserData *>(data->UserData);
    console_panel *self = ud->self;

    if (data->EventFlag == ImGuiInputTextFlags_CallbackCompletion) {
        // Collect the word being typed (last space-delimited token)
        const char *end   = data->Buf + data->CursorPos;
        const char *begin = end;
        while (begin > data->Buf && *(begin - 1) != ' ')
            --begin;

        std::string prefix(begin, end);
        auto candidates = self->completions_for(prefix);

        if (candidates.empty()) {
            // nothing
        } else if (candidates.size() == 1) {
            // Replace the partial token with the full match + space
            data->DeleteChars(static_cast<int>(begin - data->Buf),
                               static_cast<int>(end - begin));
            data->InsertChars(data->CursorPos, candidates[0].c_str());
            data->InsertChars(data->CursorPos, " ");
        } else {
            // Find common prefix among candidates
            std::string common = candidates[0];
            for (std::size_t i = 1; i < candidates.size(); ++i) {
                std::size_t j = 0;
                while (j < common.size() && j < candidates[i].size() &&
                       common[j] == candidates[i][j])
                    ++j;
                common.resize(j);
            }
            if (common.size() > prefix.size()) {
                data->DeleteChars(static_cast<int>(begin - data->Buf),
                                   static_cast<int>(end - begin));
                data->InsertChars(data->CursorPos, common.c_str());
            }
            // Show all candidates
            self->print_info("Candidates:");
            for (auto &c : candidates)
                self->print("  " + c);
        }
    }
    else if (data->EventFlag == ImGuiInputTextFlags_CallbackHistory) {
        const int prev_pos = self->m_history_pos;

        if (data->EventKey == ImGuiKey_UpArrow) {
            if (self->m_history_pos == -1)
                self->m_history_pos = static_cast<int>(self->m_history.size()) - 1;
            else if (self->m_history_pos > 0)
                --self->m_history_pos;
        } else if (data->EventKey == ImGuiKey_DownArrow) {
            if (self->m_history_pos != -1) {
                ++self->m_history_pos;
                if (self->m_history_pos >= static_cast<int>(self->m_history.size()))
                    self->m_history_pos = -1;
            }
        }

        if (prev_pos != self->m_history_pos) {
            const char *entry = (self->m_history_pos >= 0)
                ? self->m_history[self->m_history_pos].c_str()
                : "";
            data->DeleteChars(0, data->BufTextLen);
            data->InsertChars(0, entry);
        }
    }

    return 0;
}

// -----------------------------------------------------------------------
// Rendering
// -----------------------------------------------------------------------
void console_panel::render()
{
    if (!is_open)
        return;

    ImGui::SetNextWindowSize(ImVec2(700, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(400, 200), ImVec2(FLT_MAX, FLT_MAX));

    auto const panelname = (title.empty() ? m_name : title) + "###" + m_name;
    if (ImGui::Begin(panelname.c_str(), &is_open, ImGuiWindowFlags_NoCollapse))
        render_contents();
    ImGui::End();
}

void console_panel::render_contents()
{
    // --- Log area ---
    const float footer_height = ImGui::GetStyle().ItemSpacing.y
                              + ImGui::GetFrameHeightWithSpacing();
    ImGui::BeginChild("console_scrolling",
                      ImVec2(0, -footer_height),
                      false,
                      ImGuiWindowFlags_HorizontalScrollbar);

    ImGui::PushFont(ui_layer::font_mono);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 1));

    for (auto &entry : m_log) {
        ImGui::PushStyleColor(ImGuiCol_Text,
                              ImVec4(entry.color.r, entry.color.g,
                                     entry.color.b, entry.color.a));
        ImGui::TextUnformatted(entry.text.c_str());
        ImGui::PopStyleColor();
    }

    if (m_scroll_to_bottom ||
        ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.0f);
    m_scroll_to_bottom = false;

    ImGui::PopStyleVar();
    ImGui::PopFont();
    ImGui::EndChild();

    ImGui::Separator();

    // --- Input line ---
    ImGui::PushFont(ui_layer::font_mono);

    bool reclaim_focus = false;

    ImGuiInputTextFlags input_flags =
        ImGuiInputTextFlags_EnterReturnsTrue   |
        ImGuiInputTextFlags_CallbackCompletion |
        ImGuiInputTextFlags_CallbackHistory;

    CallbackUserData cbud{ this, "" };

    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::InputText("##console_input", m_input_buf, sizeof(m_input_buf),
                         input_flags, &console_panel::input_callback, &cbud))
    {
        execute(m_input_buf);
        m_input_buf[0] = '\0';
        reclaim_focus = true;
    }

    ImGui::PopFont();

    // Auto-focus the input field when the window is shown
    ImGui::SetItemDefaultFocus();
    if (reclaim_focus || m_want_focus) {
        ImGui::SetKeyboardFocusHere(-1);
        m_want_focus = false;
    }
}

} // namespace dev
