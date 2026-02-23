/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editormode.h"
#include "editoruilayer.h"

#include "application.h"
#include "Globals.h"
#include "simulation.h"
#include "simulationtime.h"
#include "simulationenvironment.h"
#include "Timer.h"
#include "Console.h"
#include "renderer.h"
#include "AnimModel.h"
#include "scene.h"


#include "imgui/imgui.h"
#include "Logs.h"
#include <cmath>
#include <functional>
#include <vector>

// Static member initialization
TCamera editor_mode::Camera;
bool editor_mode::m_focus_active = false;
bool editor_mode::m_change_history = true;

namespace
{
    using vec3 = glm::vec3;
    using dvec2 = glm::dvec2;

    inline bool is_release(int state)
    {
        return state == GLFW_RELEASE;
    }

    inline bool is_press(int state)
    {
        return state == GLFW_PRESS;
    }

} 

bool editor_mode::editormode_input::init()
{
    return (mouse.init() && keyboard.init());
}

void editor_mode::editormode_input::poll()
{
    keyboard.poll();
}

editor_mode::editor_mode() {
	m_userinterface = std::make_shared<editor_ui>();
 }

editor_ui *editor_mode::ui() const
{
    return static_cast<editor_ui *>(m_userinterface.get());
}

bool editor_mode::init()
{
    Camera.Init({0, 15, 0}, {glm::radians(-30.0), glm::radians(180.0), 0}, nullptr);
    return m_input.init();
}

void editor_mode::apply_rotation_for_new_node(scene::basic_node *node, int rotation_mode, float fixed_rotation_value)
{
    if (!node)
        return;

    if (rotation_mode == functions_panel::RANDOM)
    {
        const vec3 rotation{0.0f, LocalRandom(0.0, 360.0), 0.0f};
        m_editor.rotate(node, rotation, 1);
    }
    else if (rotation_mode == functions_panel::FIXED)
    {
        const vec3 rotation{0.0f, fixed_rotation_value, 0.0f};
        m_editor.rotate(node, rotation, 0);
    }
}

void editor_mode::start_focus(scene::basic_node *node, double duration)
{
    if (!node)
        return;

    m_focus_active = true;
    m_focus_time = 0.0;
    m_focus_duration = duration;

    m_focus_start_pos = Camera.Pos;
    m_focus_start_lookat = Camera.LookAt;

    m_focus_target_lookat = node->location();

    glm::dvec3 dir = m_focus_start_pos - m_focus_start_lookat;
    double dist = glm::length(dir);
    m_focus_target_pos = m_focus_target_lookat + glm::dvec3(10.0, 3.0, 10.0);
}

void editor_mode::handle_brush_mouse_hold(int Action, int Button)
{
    auto const mode = ui()->mode();
    auto const rotation_mode = ui()->rot_mode();
    auto const fixed_rotation_value = ui()->rot_val();

    if(mode != nodebank_panel::BRUSH)
        return;
    
    GfxRenderer->Pick_Node_Callback(
        [this, mode, rotation_mode, fixed_rotation_value, Action, Button](scene::basic_node * /*node*/) {
            const std::string *src = ui()->get_active_node_template();
            if (!src)
                return;

            std::string name = "editor_";

            glm::dvec3 newPos = clamp_mouse_offset_to_max(GfxRenderer->Mouse_Position());
            double distance = glm::distance(newPos, oldPos);
            if (distance < ui()->getSpacing())
                return;

            TAnimModel *cloned = simulation::State.create_model(*src, name, Camera.Pos + newPos);
            oldPos = newPos;
            if (!cloned)
                return;

            std::string new_name = "editor_" + cloned->uuid.to_string();

            cloned->m_name = new_name;
            
            std::string as_text;
            cloned->export_as_text(as_text);
            push_snapshot(cloned, EditorSnapshot::Action::Add, as_text);

            m_node = cloned;
            apply_rotation_for_new_node(m_node, rotation_mode, fixed_rotation_value);
            ui()->set_node(m_node);
        });
}

void editor_mode::add_to_hierarchy(scene::basic_node *node)
{
    if (!node) return;
    scene::Hierarchy[node->uuid.to_string()] = node;
}

void editor_mode::remove_from_hierarchy(scene::basic_node *node)
{
    if (!node) return;
    auto it = scene::Hierarchy.find(node->uuid.to_string());
    if (it != scene::Hierarchy.end())
        scene::Hierarchy.erase(it);
}

scene::basic_node* editor_mode::find_in_hierarchy(const std::string &uuid_str)
{
    if (uuid_str.empty()) return nullptr;
    auto it = scene::Hierarchy.find(uuid_str);
    return (it != scene::Hierarchy.end()) ? it->second : nullptr;
}

scene::basic_node* editor_mode::find_node_by_any(scene::basic_node *node_ptr, const std::string &uuid_str, const std::string &name)
{
    if (node_ptr) return node_ptr;
    
    if (!uuid_str.empty()) {
        auto *node = find_in_hierarchy(uuid_str);
        if (node) return node;
    }
    
    if (!name.empty()) {
        return simulation::Instances.find(name);
    }
    
    return nullptr;
}

void editor_mode::push_snapshot(scene::basic_node *node, EditorSnapshot::Action Action, std::string const &Serialized)
{
    if (!node)
        return;

    if(m_max_history_size >= 0 && (int)m_history.size() >= m_max_history_size)
    {
        m_history.erase(m_history.begin(), m_history.begin() + ((int)m_history.size() - m_max_history_size + 1));
    }

    EditorSnapshot snap;
    snap.action = Action;
    snap.node_name = node->name();
    snap.position = node->location();
    snap.uuid = node->uuid;
    
    if (auto *model = dynamic_cast<TAnimModel *>(node))
    {
        snap.rotation = model->Angles();
    }
    else
    {
        snap.rotation = glm::vec3(0.0f);
    }

    if (Action == EditorSnapshot::Action::Delete || Action == EditorSnapshot::Action::Add)
    {
        if (!Serialized.empty())
            snap.serialized = Serialized;
        else
            node->export_as_text(snap.serialized);
    }


    snap.node_ptr = node;

    m_history.push_back(std::move(snap));
    g_redo.clear();
}

glm::dvec3 editor_mode::clamp_mouse_offset_to_max(const glm::dvec3 &offset)
{
    double len = glm::length(offset);
    if (len <= static_cast<double>(kMaxPlacementDistance) || len <= 1e-6)
        return offset;
    return glm::normalize(offset) * static_cast<double>(kMaxPlacementDistance);
}

void editor_mode::nullify_history_pointers(scene::basic_node *node)
{
    if (!node)
        return;

    for (auto &s : m_history)
    {
        if (s.node_ptr == node)
            s.node_ptr = nullptr;
    }

    for (auto &s : g_redo)
    {
        if (s.node_ptr == node)
            s.node_ptr = nullptr;
    }
}

void editor_mode::undo_last()
{
    if (m_history.empty())
        return;

    EditorSnapshot snap = m_history.back();
    m_history.pop_back();

    if (snap.action == EditorSnapshot::Action::Delete)
    {
        // undo delete -> recreate model
        EditorSnapshot redoSnap;
        redoSnap.action = EditorSnapshot::Action::Delete;
        redoSnap.node_name = snap.node_name;
        redoSnap.serialized = snap.serialized;
        redoSnap.position = snap.position;
        redoSnap.node_ptr = nullptr;
        g_redo.push_back(std::move(redoSnap));

        TAnimModel *created = simulation::State.create_model(snap.serialized, snap.node_name, snap.position);
        if (created)
        {
            created->location(snap.position);
            created->Angles(snap.rotation);
            m_node = created;
            m_node->uuid = snap.uuid; // restore original UUID for better tracking (not strictly necessary) 
            add_to_hierarchy(created);
            ui()->set_node(m_node);
        }
        return;
    }

    scene::basic_node *target = find_node_by_any(snap.node_ptr, snap.uuid.to_string(), snap.node_name);
    if (!target)
        return;

    EditorSnapshot current;
    current.action = snap.action;
    current.node_name = snap.node_name;
    current.node_ptr = target;
    current.position = target->location();
    if (auto *model = dynamic_cast<TAnimModel *>(target))
        current.rotation = model->Angles();
    else
        current.rotation = glm::vec3(0.0f);
    g_redo.push_back(std::move(current));

    if (snap.action == EditorSnapshot::Action::Add)
    {
        // undo add -> delete the instance
        if (auto *model = dynamic_cast<TAnimModel *>(target))
        {
          
            nullify_history_pointers(model);
            remove_from_hierarchy(model);
            simulation::State.delete_model(model);
            m_node = nullptr;
            ui()->set_node(nullptr);
        }
        return;
    }

    target->location(snap.position);

    if (auto *model = dynamic_cast<TAnimModel *>(target))
    {
        glm::vec3 cur = model->Angles();
        glm::vec3 delta = snap.rotation - cur;
        m_editor.rotate(target, delta, 0);
    }

    m_node = target;
    ui()->set_node(m_node);
}

void editor_mode::redo_last()
{
    if (g_redo.empty())
        return;

    EditorSnapshot snap = g_redo.back();
    g_redo.pop_back();

    // handle delete redo (re-delete) separately
    if (snap.action == EditorSnapshot::Action::Delete)
    {
        EditorSnapshot hist;
        hist.action = snap.action;
        hist.node_name = snap.node_name;
        hist.serialized = snap.serialized;
        hist.position = snap.position;
        hist.uuid = snap.uuid;
        m_history.push_back(std::move(hist));

        scene::basic_node *target = simulation::Instances.find(snap.node_name);
        if (target)
        {
            if (auto *model = dynamic_cast<TAnimModel *>(target))
            {
                nullify_history_pointers(model);
                remove_from_hierarchy(model);
                simulation::State.delete_model(model);
                m_node = nullptr;
                ui()->set_node(nullptr);
            }
        }
        return;
    }

    scene::basic_node *target = find_node_by_any(snap.node_ptr, snap.uuid.to_string(), snap.node_name);

    EditorSnapshot hist;
    hist.action = snap.action;
    hist.node_name = snap.node_name;
    hist.node_ptr = target;

    if (target)
    {
        hist.position = target->location();
        if (auto *model = dynamic_cast<TAnimModel *>(target))
            hist.rotation = model->Angles();
        hist.uuid = snap.uuid;
    }
    m_history.push_back(std::move(hist));

    if (snap.action == EditorSnapshot::Action::Add)
    {
        TAnimModel *created = simulation::State.create_model(snap.serialized, snap.node_name, snap.position);
        if (created)
        {
            created->location(snap.position);
            created->Angles(snap.rotation);
            m_node = created;
            m_node->uuid = snap.uuid;
            ui()->set_node(m_node);
            if (!m_history.empty())
                m_history.back().node_ptr = created;
        }
        return;
    }

    if (!target)
        return;

    // apply redo position
    target->location(snap.position);
    if (auto *model = dynamic_cast<TAnimModel *>(target))
    {
        glm::vec3 cur = model->Angles();
        glm::vec3 delta = snap.rotation - cur;
        m_editor.rotate(target, delta, 0);
    }

    m_node = target;
    ui()->set_node(m_node);
}

bool editor_mode::update()
{
    Timer::UpdateTimers(true);

    simulation::State.update_clocks();
    simulation::Environment.update();

    auto const deltarealtime = Timer::GetDeltaRenderTime();

    // fixed step render time routines (50 Hz)
    fTime50Hz += deltarealtime; // accumulate even when paused to keep frame reads stable
    while (fTime50Hz >= 1.0 / 50.0)
    {
#ifdef _WIN32
        Console::Update();
#endif
        m_userinterface->update();

        // update brush settings visibility depending on panel mode
        ui()->toggleBrushSettings(ui()->mode() == nodebank_panel::BRUSH);

        if (mouseHold)
        {
            // process continuous brush placement
            if(ui()->mode() == nodebank_panel::BRUSH)
                handle_brush_mouse_hold(GLFW_REPEAT, GLFW_MOUSE_BUTTON_LEFT);
        }

        // decelerate camera velocity with thresholding
        Camera.Velocity *= 0.65f;
        if (std::abs(Camera.Velocity.x) < 0.01)
            Camera.Velocity.x = 0.0;
        if (std::abs(Camera.Velocity.y) < 0.01)
            Camera.Velocity.y = 0.0;
        if (std::abs(Camera.Velocity.z) < 0.01)
            Camera.Velocity.z = 0.0;

        fTime50Hz -= 1.0 / 50.0;
 
    }

    // variable step routines
    update_camera(deltarealtime);

    simulation::Region->update_sounds();
    audio::renderer.update(Global.iPause ? 0.0 : deltarealtime);

    GfxRenderer->Update(deltarealtime);

    simulation::is_ready = true;

    // --- ImGui: Editor History Window & Settings ---
    if(!m_change_history) return true;
 
    render_change_history();

    return true;
}

void editor_mode::update_camera(double const Deltatime)
{
    // account for keyboard-driven motion
    // if focus animation active, interpolate camera toward target
    if (m_focus_active)
    {
        m_focus_time += Deltatime;
        double t = m_focus_duration > 0.0 ? (m_focus_time / m_focus_duration) : 1.0;
        if (t >= 1.0)
            t = 1.0;
        // smoothstep easing
        double s = t * t * (3.0 - 2.0 * t);
        Camera.Pos = glm::mix(m_focus_start_pos, m_focus_target_pos, s);
        Camera.LookAt = glm::mix(m_focus_start_lookat, m_focus_target_lookat, s);
        if (t >= 1.0)
            m_focus_active = false;
    }

    Camera.Update();

    // reset window state (will be set again if UI requires it)
    Global.CabWindowOpen = false;

    // publish camera back to global copy
    Global.pCamera = Camera;
}

void editor_mode::enter()
{
    m_statebackup = {Global.pCamera, FreeFlyModeFlag, Global.ControlPicking};

    Camera = Global.pCamera;

    if (!FreeFlyModeFlag)
    {
        auto const *vehicle = Camera.m_owner;
        if (vehicle)
        {
            auto const cab = (vehicle->MoverParameters->CabOccupied == 0 ? 1 : vehicle->MoverParameters->CabOccupied);
            auto const left = vehicle->VectorLeft() * cab;
            Camera.Pos = Math3D::vector3(Camera.Pos.x, vehicle->GetPosition().y, Camera.Pos.z) + left * vehicle->GetWidth() + Math3D::vector3(1.25f * left.x, 1.6f, 1.25f * left.z);
            Camera.m_owner = nullptr;
            Camera.LookAt = vehicle->GetPosition();
            Camera.RaLook(); // single camera reposition
            FreeFlyModeFlag = true;
        }
    }

    Global.ControlPicking = true;
    EditorModeFlag = true;

    Application.set_cursor(GLFW_CURSOR_NORMAL);
}

void editor_mode::exit()
{
    EditorModeFlag = false;
    Global.ControlPicking = m_statebackup.picking;
    FreeFlyModeFlag = m_statebackup.freefly;
    Global.pCamera = m_statebackup.camera;

    g_redo.clear();
    m_history.clear();

    Application.set_cursor((Global.ControlPicking ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_DISABLED));

    if (!Global.ControlPicking)
    {
        Application.set_cursor_pos(0, 0);
    }
}

void editor_mode::on_key(int const Key, int const Scancode, int const Action, int const Mods)
{
#ifndef __unix__
    Global.shiftState = (Mods & GLFW_MOD_SHIFT) ? true : false;
    Global.ctrlState = (Mods & GLFW_MOD_CONTROL) ? true : false;
    Global.altState = (Mods & GLFW_MOD_ALT) ? true : false;
#endif
    bool anyModifier = Mods & (GLFW_MOD_SHIFT | GLFW_MOD_CONTROL | GLFW_MOD_ALT);

    // first give UI a chance to handle the key
    if (!anyModifier && m_userinterface->on_key(Key, Action))
        return;

    // then internal input handling
    if (m_input.keyboard.key(Key, Action))
        return;

    if (Action == GLFW_RELEASE)
        return;

    // shortcuts: undo/redo
    if (Global.ctrlState && Key == GLFW_KEY_Z && is_press(Action))
    {
        undo_last();
        return;
    }
    if (Global.ctrlState && Key == GLFW_KEY_Y && is_press(Action))
    {
        redo_last();
        return;
    }

    // legacy hardcoded keyboard commands
    switch (Key)
    {
    case GLFW_KEY_F11:
        if (Action != GLFW_PRESS)
            break;

        if (!Global.ctrlState && !Global.shiftState)
        {
            Application.pop_mode();
        }
        else if (Global.ctrlState && Global.shiftState)
        {
            simulation::State.export_as_text(Global.SceneryFile);
        }
        break;

    case GLFW_KEY_F12:
        if (Global.ctrlState && Global.shiftState && is_press(Action))
        {
            DebugModeFlag = !DebugModeFlag;
        }
        break;

    case GLFW_KEY_DELETE:
        if (is_press(Action))
        {
            TAnimModel *model = dynamic_cast<TAnimModel *>(m_node);
            if (model)
            {
                // record deletion for undo (serialize full node)
                std::string as_text;
                
                model->export_as_text(as_text);
                std::string debug = "Deleting node: " + as_text + "\nSerialized data:\n";
                push_snapshot(model, EditorSnapshot::Action::Delete, as_text);
                WriteLog(debug, logtype::generic);

                // clear history pointers referencing this model before actually deleting it
                nullify_history_pointers(model);
                remove_from_hierarchy(model);

                m_node = nullptr;
                m_dragging = false;
                ui()->set_node(nullptr);
                simulation::State.delete_model(model);
            }
        }
        break;

    case GLFW_KEY_F:
        if (is_press(Action))
        {
            if(!m_node)
                break;

            // start smooth focus camera on selected node
            start_focus(m_node, 0.6);
        }
        break;

    default:
        break;
    }
}

void editor_mode::on_cursor_pos(double const Horizontal, double const Vertical)
{
    dvec2 const mousemove = dvec2{Horizontal, Vertical} - m_input.mouse.position();
    m_input.mouse.position(Horizontal, Vertical);

    if (m_input.mouse.button(GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
        return;
    if (!m_node)
        return;

    if (m_takesnapshot)
    {
        // record appropriate action type depending on current input mode
        if (mode_rotationX() || mode_rotationY() || mode_rotationZ())
            push_snapshot(m_node, EditorSnapshot::Action::Rotate);
        else
            push_snapshot(m_node, EditorSnapshot::Action::Move);

        m_takesnapshot = false;
    }

    if (mode_translation())
    {
        if (mode_translation_vertical())
        {
            float const translation = static_cast<float>(mousemove.y * -0.01);
            m_editor.translate(m_node, translation);
        }
        else
        {
            auto mouseOffset = clamp_mouse_offset_to_max(GfxRenderer->Mouse_Position());
            auto const mouseworldposition = Camera.Pos + mouseOffset;
            m_editor.translate(m_node, mouseworldposition, mode_snap());
        }
    }
    else if (mode_rotationY())
    {
        vec3 const rotation{0.0f, static_cast<float>(mousemove.x) * 0.25f, 0.0f};
        float const quantization = (mode_snap() ? 5.0f : 0.0f);
        m_editor.rotate(m_node, rotation, quantization);
    }
    else if (mode_rotationZ())
    {
        vec3 const rotation{0.0f, 0.0f, static_cast<float>(mousemove.x) * 0.25f};
        float const quantization = (mode_snap() ? 5.0f : 0.0f);
        m_editor.rotate(m_node, rotation, quantization);
    }
    else if (mode_rotationX())
    {
        vec3 const rotation{static_cast<float>(mousemove.y) * 0.25f, 0.0f, 0.0f};
        float const quantization = (mode_snap() ? 5.0f : 0.0f);
        m_editor.rotate(m_node, rotation, quantization);
    }
}

void editor_mode::on_mouse_button(int const Button, int const Action, int const Mods)
{
    // UI first
    if (m_userinterface->on_mouse_button(Button, Action))
    {
        m_input.mouse.button(Button, Action);
        return;
    }

    if (Button == GLFW_MOUSE_BUTTON_LEFT)
    {
		
        auto const mode = ui()->mode();
        auto const rotation_mode = ui()->rot_mode();
        auto const fixed_rotation_value = ui()->rot_val();

        if (is_press(Action))
        {
            mouseHold = true;
            m_node = nullptr;

            // delegate node picking behaviour depending on current panel mode
            GfxRenderer->Pick_Node_Callback(
                [this, mode, rotation_mode, fixed_rotation_value](scene::basic_node *node) {
                    // ignore picks that are beyond allowed placement distance
                    if (node) {
                        double const dist = glm::distance(node->location(), glm::dvec3{Global.pCamera.Pos});
                        if (dist > static_cast<double>(kMaxPlacementDistance))
                            return;
                    }
                    if (mode == nodebank_panel::MODIFY)
                    {
                        if (!m_dragging)
                            return;

                        m_node = node;
                        ui()->set_node(m_node);
                    }
                    else if (mode == nodebank_panel::COPY)
                    {
                        if (node && typeid(*node) == typeid(TAnimModel))
                        {
                            std::string as_text;
                            node->export_as_text(as_text);
                            ui()->add_node_template(as_text);
                        }

                        m_dragging = false;
                    }
                    else if (mode == nodebank_panel::ADD)
                    {
                        const std::string *src = ui()->get_active_node_template();
                        if (!src)
                            return;

                        std::string name = "editor_";
                        glm::dvec3 mouseOffset = clamp_mouse_offset_to_max(GfxRenderer->Mouse_Position());
                        TAnimModel *cloned = simulation::State.create_model(*src, name, Camera.Pos + mouseOffset);
                        if (!cloned)
                            return;

                        // record addition for undo
                        std::string as_text;
                        std::string new_name = "editor_" + cloned->uuid.to_string();

                        cloned->m_name = new_name;
                        cloned->export_as_text(as_text);
                        push_snapshot(cloned, EditorSnapshot::Action::Add, as_text);

                        if (!m_dragging)
                            return;

                        m_node = cloned;
                        apply_rotation_for_new_node(m_node, rotation_mode, fixed_rotation_value);
                        ui()->set_node(m_node);
                    }
                });

            m_dragging = true;
            m_takesnapshot = true;
        }
        else
        {
            if (is_release(Action))
                mouseHold = false;

            m_dragging = false;
        }
    }

    m_input.mouse.button(Button, Action);
}

void editor_mode::render_change_history(){


    ImGui::Begin("Editor History", &m_change_history, ImGuiWindowFlags_AlwaysAutoResize);
    int maxsize = m_max_history_size;
    if (ImGui::InputInt("Max history size", &maxsize))
    {
        m_max_history_size = std::max(0, maxsize);
        if ((int)m_history.size() > m_max_history_size && m_max_history_size >= 0)
        {
            auto remove_count = (int)m_history.size() - m_max_history_size;
            m_history.erase(m_history.begin(), m_history.begin() + remove_count);
            // adjust selected index
            if (m_selected_history_idx >= (int)m_history.size())
                m_selected_history_idx = (int)m_history.size() - 1;
        }
        
    }  

    float dist = kMaxPlacementDistance;
    if (ImGui::InputFloat("Max placement distance", &dist))
    {
        kMaxPlacementDistance = std::max(0.0f, dist);
    }

    ImGui::Separator();

    ImGui::Text("History (newest at end): %zu entries", m_history.size());
    ImGui::BeginChild("history_list", ImVec2(400, 200), true);
    for (int i = 0; i < (int)m_history.size(); ++i)
    {
        auto &s = m_history[i];
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%3d: %s %s pos=(%.1f,%.1f,%.1f)", i,
                        (s.action == EditorSnapshot::Action::Add) ? "ADD" :
                        (s.action == EditorSnapshot::Action::Delete) ? "DEL" :
                        (s.action == EditorSnapshot::Action::Move) ? "MOV" :
                        (s.action == EditorSnapshot::Action::Rotate) ? "ROT" : "OTH",
                        s.node_name.empty() ? "(noname)" : s.node_name.c_str(),
                        s.position.x, s.position.y, s.position.z);

        if (ImGui::Selectable(buf, m_selected_history_idx == i))
            m_selected_history_idx = i;
    }
    ImGui::EndChild();

    ImGui::Separator();
    if (ImGui::Button("Clear History"))
    {
        m_history.clear();
        g_redo.clear();
        m_selected_history_idx = -1;
    }
    ImGui::SameLine();
   
    ImGui::SameLine();
    if (ImGui::Button("Undo Selected"))
    {
        if (m_selected_history_idx >= 0 && m_selected_history_idx < (int)m_history.size())
        {
            int target = m_selected_history_idx;
            int undoCount = (int)m_history.size() - 1 - target;
            for (int k = 0; k < undoCount; ++k)
                undo_last();
            m_selected_history_idx = -1;
        }
    }      

    ImGui::End();
}


void editor_mode::on_event_poll()
{
    m_input.poll();
}

bool editor_mode::is_command_processor() const
{
    return false;
}

bool editor_mode::mode_translation() const
{
    return (false == Global.altState);
}

bool editor_mode::mode_translation_vertical() const
{
    return (true == Global.shiftState);
}

bool editor_mode::mode_rotationY() const
{
    return ((true == Global.altState) && (false == Global.ctrlState) && (false == Global.shiftState));
}
 
bool editor_mode::mode_rotationX() const
{
    return ((true == Global.altState) && (true == Global.ctrlState) && (false == Global.shiftState));
}

bool editor_mode::mode_rotationZ() const
{
    return ((true == Global.altState) && (true == Global.ctrlState) && (true == Global.shiftState));
}

bool editor_mode::mode_snap() const
{
    return ((false == Global.altState) && (true == Global.ctrlState) && (false == Global.shiftState));
}

bool editor_mode::focus_active()
{
    return m_focus_active;
}

void editor_mode::set_focus_active(bool isActive)
{
    m_focus_active = isActive;
}
