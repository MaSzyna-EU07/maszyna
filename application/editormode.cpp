/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"

#include "application/application.h"
#include "editor/editorSettings.hpp"
#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Timer.h"
#include "Console.h"
#include "rendering/renderer.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "utilities/Float3d.h"
#include "scene/scene.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"


#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include "utilities/Logs.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

// Static member initialization
TCamera editor_mode::Camera;
bool editor_mode::m_focus_active = false;
bool editor_mode::m_change_history = false;
bool editor_mode::m_settings_open = false;

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

    // true when the most recent left button press landed in the viewport rather than on an ImGui window.
    // valid once ImGui has processed the press (i.e. from mode update onwards), so it's also used to
    // discard deferred pick callbacks of a press that leaked past the application's UI filter
    inline bool viewport_click()
    {
        return false == ImGui::GetIO().MouseDownOwned[0];
    }

    // tests whether the vertical line through (Px,Pz) passes over triangle abc; if so returns the
    // surface height at that point through OutY. used by the "snap to ground" (END) feature.
    inline bool triangle_height_at(glm::dvec3 const &a, glm::dvec3 const &b, glm::dvec3 const &c,
                                   double const Px, double const Pz, double &OutY)
    {
        double const ux = b.x - a.x, uz = b.z - a.z;
        double const vx = c.x - a.x, vz = c.z - a.z;
        double const wx = Px - a.x, wz = Pz - a.z;
        double const den = ux * vz - vx * uz;
        if (std::abs(den) < 1e-9)
            return false; // degenerate or vertical triangle, no defined height
        double const s = (wx * vz - vx * wz) / den;
        double const t = (ux * wz - wx * uz) / den;
        if (s < 0.0 || t < 0.0 || s + t > 1.0)
            return false;
        OutY = a.y + s * (b.y - a.y) + t * (c.y - a.y);
        return true;
    }

    using world_triangle = std::array<glm::dvec3, 3>;

    // walks a model's submodel tree (mirroring the renderer's transform chain) and appends every
    // mesh triangle, in world space, to Out. siblings are iterated to avoid deep recursion.
    void gather_submodel_triangles(TSubModel *Submodel, glm::dmat4 const &M, std::vector<world_triangle> &Out)
    {
        for (TSubModel *sub = Submodel; sub != nullptr; sub = sub->Next)
        {
            glm::dmat4 mlocal = M;
            if (sub->iFlags & 0xC000 && sub->GetMatrix() != nullptr)
                mlocal = M * glm::dmat4(glm::make_mat4(sub->GetMatrix()->readArray()));

            if (sub->eType < TP_ROTATOR) // a drawable mesh, not a rotator/light/etc.
            {
                auto const handle = sub->m_geometry.handle;
                if (handle.bank != 0 || handle.chunk != 0)
                {
                    auto const &verts = GfxRenderer->Vertices(handle);
                    auto const &indices = GfxRenderer->Indices(handle);
                    auto const to_world = [&](gfx::basic_vertex const &v) {
                        return glm::dvec3(mlocal * glm::dvec4(glm::dvec3(v.position), 1.0));
                    };
                    if (false == indices.empty())
                    {
                        for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
                            Out.push_back({to_world(verts[indices[i]]), to_world(verts[indices[i + 1]]), to_world(verts[indices[i + 2]])});
                    }
                    else
                    {
                        for (std::size_t i = 0; i + 2 < verts.size(); i += 3)
                            Out.push_back({to_world(verts[i]), to_world(verts[i + 1]), to_world(verts[i + 2])});
                    }
                }
            }

            if (sub->Child != nullptr)
                gather_submodel_triangles(sub->Child, mlocal, Out); // children inherit this matrix
        }
    }

    // world transform of a model instance, matching the renderer: translate * rotateY * rotateX * rotateZ * scale
    glm::dmat4 instance_matrix(TAnimModel const &Model)
    {
        glm::dmat4 rootm(1.0);
        rootm = glm::translate(rootm, Model.location());
        glm::vec3 const angles = Model.Angles();
        if (angles.y != 0.0f) rootm = glm::rotate(rootm, glm::radians(static_cast<double>(angles.y)), glm::dvec3(0.0, 1.0, 0.0));
        if (angles.x != 0.0f) rootm = glm::rotate(rootm, glm::radians(static_cast<double>(angles.x)), glm::dvec3(1.0, 0.0, 0.0));
        if (angles.z != 0.0f) rootm = glm::rotate(rootm, glm::radians(static_cast<double>(angles.z)), glm::dvec3(0.0, 0.0, 1.0));
        return glm::scale(rootm, glm::dvec3(Model.Scale()));
    }

    // even-odd test of a point against a polygon outline projected onto the XZ plane
    bool point_in_polygon_xz(std::vector<glm::dvec3> const &Polygon, double const X, double const Z)
    {
        bool inside = false;
        for (std::size_t i = 0, j = Polygon.size() - 1; i < Polygon.size(); j = i++)
        {
            auto const &a = Polygon[i];
            auto const &b = Polygon[j];
            if ((a.z > Z) != (b.z > Z) && X < (b.x - a.x) * (Z - a.z) / (b.z - a.z) + a.x)
                inside = !inside;
        }
        return inside;
    }

    // area of a polygon outline projected onto the XZ plane (shoelace formula)
    double polygon_area_xz(std::vector<glm::dvec3> const &Polygon)
    {
        if (Polygon.size() < 3)
            return 0.0;
        double sum = 0.0;
        for (std::size_t i = 0, j = Polygon.size() - 1; i < Polygon.size(); j = i++)
            sum += Polygon[j].x * Polygon[i].z - Polygon[i].x * Polygon[j].z;
        return std::abs(sum) * 0.5;
    }

    // short readable name of a node template (model file and texture), like the node bank shows it
    std::string template_label(std::string const &Template)
    {
        cParser tokenizer(Template);
        tokenizer.getTokens(9, false); // node, ranges, name, type, position, rotation
        auto model{tokenizer.getToken<std::string>(false)};
        auto texture{tokenizer.getToken<std::string>(false)};
        replace_slashes(model);
        erase_extension(model);
        replace_slashes(texture);
        return texture == "none" || texture.empty() ? model : model + " (" + texture + ")";
    }

    // world triangles bucketed on a uniform XZ grid, for many ground height queries over one area
    class triangle_grid
    {
      public:
        triangle_grid(std::vector<world_triangle> Triangles, glm::dvec2 const &Min, glm::dvec2 const &Max) : m_triangles(std::move(Triangles)), m_min(Min)
        {
            glm::dvec2 const extent = glm::max(Max - Min, glm::dvec2(1.0));
            // ~8 m cells, coarser for big areas so large terrain triangles don't flood the grid
            m_cellsize = std::max(8.0, std::max(extent.x, extent.y) / 128.0);
            m_columns = static_cast<int>(std::ceil(extent.x / m_cellsize)) + 1;
            m_rows = static_cast<int>(std::ceil(extent.y / m_cellsize)) + 1;
            m_cells.resize(static_cast<std::size_t>(m_columns) * m_rows);

            for (std::uint32_t idx = 0; idx < m_triangles.size(); ++idx)
            {
                auto const &t = m_triangles[idx];
                int const c0 = column(std::min({t[0].x, t[1].x, t[2].x}));
                int const c1 = column(std::max({t[0].x, t[1].x, t[2].x}));
                int const r0 = row(std::min({t[0].z, t[1].z, t[2].z}));
                int const r1 = row(std::max({t[0].z, t[1].z, t[2].z}));
                for (int r = r0; r <= r1; ++r)
                    for (int c = c0; c <= c1; ++c)
                        m_cells[static_cast<std::size_t>(r) * m_columns + c].push_back(idx);
            }
        }

        // highest surface at (X,Z) that isn't above Ceiling. returns: true if there is one
        bool height_at(double const X, double const Z, double const Ceiling, double &OutY) const
        {
            bool found = false;
            for (auto const idx : m_cells[static_cast<std::size_t>(row(Z)) * m_columns + column(X)])
            {
                auto const &t = m_triangles[idx];
                double y;
                if (triangle_height_at(t[0], t[1], t[2], X, Z, y) && y <= Ceiling && (!found || y > OutY))
                {
                    OutY = y;
                    found = true;
                }
            }
            return found;
        }

      private:
        int column(double const X) const
        {
            return std::clamp(static_cast<int>(std::floor((X - m_min.x) / m_cellsize)), 0, m_columns - 1);
        }
        int row(double const Z) const
        {
            return std::clamp(static_cast<int>(std::floor((Z - m_min.y) / m_cellsize)), 0, m_rows - 1);
        }

        std::vector<world_triangle> m_triangles;
        std::vector<std::vector<std::uint32_t>> m_cells;
        glm::dvec2 m_min;
        double m_cellsize{8.0};
        int m_columns{1};
        int m_rows{1};
    };

    // appends triangles of a shape node which overlap specified XZ rectangle
    void gather_shape_triangles(scene::shape_node const &Shape, glm::dvec2 const &Min, glm::dvec2 const &Max, std::vector<world_triangle> &Out)
    {
        auto const &data = Shape.data();
        if (data.area.radius >= 0.0f)
        {
            double const r = data.area.radius;
            if (data.area.center.x + r < Min.x || data.area.center.x - r > Max.x || data.area.center.z + r < Min.y || data.area.center.z - r > Max.y)
                return;
        }

        auto const add = [&](glm::dvec3 const &a, glm::dvec3 const &b, glm::dvec3 const &c) {
            if (std::max({a.x, b.x, c.x}) < Min.x || std::min({a.x, b.x, c.x}) > Max.x || std::max({a.z, b.z, c.z}) < Min.y || std::min({a.z, b.z, c.z}) > Max.y)
                return;
            Out.push_back({a, b, c});
        };

        if (false == data.vertices.empty())
        {
            for (std::size_t i = 0; i + 2 < data.vertices.size(); i += 3)
                add(data.vertices[i].position, data.vertices[i + 1].position, data.vertices[i + 2].position);
            return;
        }
        // once the shape is uploaded its world-space source is released; the renderer keeps an origin-relative copy
        if (data.geometry.bank == 0 && data.geometry.chunk == 0)
            return;
        auto const &verts = GfxRenderer->Vertices(data.geometry);
        auto const &indices = GfxRenderer->Indices(data.geometry);
        auto const to_world = [&](gfx::basic_vertex const &v) { return data.origin + glm::dvec3(v.position); };
        if (false == indices.empty())
        {
            for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
                add(to_world(verts[indices[i]]), to_world(verts[indices[i + 1]]), to_world(verts[indices[i + 2]]));
        }
        else
        {
            for (std::size_t i = 0; i + 2 < verts.size(); i += 3)
                add(to_world(verts[i]), to_world(verts[i + 1]), to_world(verts[i + 2]));
        }
    }

} 

bool editor_mode::editormode_input::init()
{
    return mouse.init() && keyboard.init();
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
    EditorSettings.load();
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

    glm::dvec3 const center = node->location();

    // distance that frames the object's bounding sphere within the vertical FOV, with some margin
    double const radius = std::max(1.0, static_cast<double>(node->radius()));
    double const fovy = glm::radians(static_cast<double>(Global.FieldOfView) / std::max(0.01, static_cast<double>(Global.ZoomFactor)));
    double distance = radius / std::tan(fovy * 0.5) * 1.6;
    distance = std::clamp(distance, radius * 1.5, static_cast<double>(kMaxPlacementDistance));

    // keep the camera on the side it currently views from, so the move turns toward the object
    // rather than flying around it; fall back to a pleasant 3/4 direction when sitting on top of it
    glm::dvec3 dir = Camera.Pos - center;
    double const len = glm::length(dir);
    dir = len > 1e-3 ? dir / len : glm::normalize(glm::dvec3(1.0, 0.5, 1.0));

    m_focus_start_pos = Camera.Pos;
    m_focus_start_angle = Camera.Angle;
    m_focus_target_pos = center + dir * distance;

    // target orientation looks from the target position straight at the object
    glm::dvec3 look = center - m_focus_target_pos;
    double const looklen = glm::length(look);
    if (looklen > 1e-6)
        look /= looklen;
    m_focus_target_angle = glm::vec3(
        static_cast<float>(std::asin(glm::clamp(look.y, -1.0, 1.0))),   // pitch
        static_cast<float>(std::atan2(-look.x, -look.z)),               // yaw
        0.0f);                                                          // roll

    m_focus_active = true;
    m_focus_time = 0.0;
    m_focus_duration = duration;
}

void editor_mode::snap_to_ground(scene::basic_node *node)
{
    if (!node || !simulation::Region)
        return;

    glm::dvec3 const origin = node->location();
    if (!simulation::Region->point_inside(origin))
        return;

    // small tolerance so a node already resting on a surface still snaps cleanly to it
    double const epsilon = 0.05;
    double bestY = -std::numeric_limits<double>::max();
    bool found = false;

    // record the highest surface that is at or below the node's current position at its (x,z)
    auto consider_triangle = [&](glm::dvec3 const &a, glm::dvec3 const &b, glm::dvec3 const &c) {
        double y;
        if (triangle_height_at(a, b, c, origin.x, origin.z, y) && y <= origin.y + epsilon && y > bestY)
        {
            bestY = y;
            found = true;
        }
    };

    auto consider_shapes = [&](std::vector<scene::shape_node> const &shapes) {
        for (auto const &shape : shapes)
        {
            // quick reject: skip shapes whose bounding circle doesn't cover our (x,z) column
            auto const &sdata = shape.data();
            double const sdx = origin.x - sdata.area.center.x;
            double const sdz = origin.z - sdata.area.center.z;
            if (sdx * sdx + sdz * sdz > static_cast<double>(sdata.area.radius) * sdata.area.radius)
                continue;

            auto const &verts = sdata.vertices;
            for (std::size_t i = 0; i + 2 < verts.size(); i += 3)
                consider_triangle(verts[i].position, verts[i + 1].position, verts[i + 2].position);
        }
    };

    scene::basic_section &sec = simulation::Region->section(origin);
    // section level holds the large opaque geometry, including legacy terrain
    consider_shapes(sec.m_shapes);

    // scan a 3x3 neighbourhood of cells for smaller geometry and other model instances below us
    for (int dz = -1; dz <= 1; ++dz)
        for (int dx = -1; dx <= 1; ++dx)
        {
            scene::basic_cell &cell = sec.cell(origin, glm::ivec2(dx, dz));
            consider_shapes(cell.m_shapesopaque);
            consider_shapes(cell.m_shapestranslucent);

            // other instances are approximated by their bounding sphere, so a node can rest on top of them
            for (auto *inst : cell.m_instancesopaque)
            {
                if (!inst || inst == node)
                    continue;
                glm::dvec3 const ic = inst->location();
                double const r = static_cast<double>(inst->radius());
                double const idx = origin.x - ic.x, idz = origin.z - ic.z;
                double const horiz2 = idx * idx + idz * idz;
                if (horiz2 < r * r)
                {
                    double const ytop = ic.y + std::sqrt(r * r - horiz2);
                    if (ytop <= origin.y + epsilon && ytop > bestY)
                    {
                        bestY = ytop;
                        found = true;
                    }
                }
            }
        }

    // editable terrain patches keep their heightmap on the CPU, so query them directly
    for (editor_terrain *terrain : active_terrains())
    {
        if (!terrain->contains(origin.x, origin.z))
            continue;
        double const y = terrain->height_at(origin.x, origin.z);
        if (y <= origin.y + epsilon && y > bestY)
        {
            bestY = y;
            found = true;
        }
    }

    if (!found)
        return;

    push_snapshot(node, EditorSnapshot::Action::Move);
    glm::dvec3 target = origin;
    target.y = bestY;
    m_editor.translate(node, target, true); // true == apply the computed Y (free vertical move)
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
            // picks are resolved a few frames later; drop the ones queued before the button went up,
            // or for a press that turned out to belong to the UI
            if (!mouseHold || !viewport_click())
                return;

            const std::string *src = ui()->get_active_node_template();
            if (!src)
                return;

            std::string name = "editor_";

            // spacing is measured in world space: the mouse offset is camera-relative and the camera may move mid-stroke
            glm::dvec3 const newPos = Camera.Pos + clamp_mouse_offset_to_max(GfxRenderer->Mouse_Position());
            if (m_brush_has_last && glm::distance(newPos, oldPos) < ui()->getSpacing())
                return;

            TAnimModel *cloned = simulation::State.create_model(*src, name, newPos);
            oldPos = newPos;
            m_brush_has_last = true;
            if (!cloned)
                return;

            std::string new_name = "editor_" + cloned->uuid.to_string();

            cloned->m_name = new_name;
            apply_rotation_for_new_node(cloned, rotation_mode, fixed_rotation_value);

            std::string as_text;
            cloned->export_as_text(as_text);
            push_snapshot(cloned, EditorSnapshot::Action::Add, as_text);

            m_node = cloned;
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
    return it != scene::Hierarchy.end() ? it->second : nullptr;
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
        snap.scale = model->Scale();
    }
    else
    {
        snap.rotation = glm::vec3(0.0f);
        snap.scale = glm::vec3(1.0f);
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

    // deleted nodes also drop out of the "undo last fill" set
    m_fill_last.erase(std::remove(m_fill_last.begin(), m_fill_last.end(), node), m_fill_last.end());
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
    {
        current.rotation = model->Angles();
        current.scale = model->Scale();
    }
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
        model->Scale(snap.scale);
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
        {
            hist.rotation = model->Angles();
            hist.scale = model->Scale();
        }
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
            created->Scale(snap.scale);
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
        model->Scale(snap.scale);
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

    // reconcile camera fly-mode with the real right-button state. ImGui is always fed the button
    // events (even when it captures the mouse), so io.MouseDown[1] is authoritative; if a release
    // was swallowed by an ImGui window while flying, force the editor out of fly-mode here so the
    // camera doesn't get stuck spinning with a hidden cursor.
    if (!ImGui::GetIO().MouseDown[1] && m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
    {
        m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT, GLFW_RELEASE);
        Application.set_cursor(GLFW_CURSOR_NORMAL);
    }

    // same for the left button (brush / sculpt / click placement). the UI filter in the application
    // uses io.WantCaptureMouse from the previous frame, so a press on a panel the cursor has just
    // moved onto can leak into the viewport, and its release is then swallowed by the UI; the brush
    // would keep painting with the button up. by now ImGui has processed this frame's input, so
    // MouseDown is the real button state and MouseDownOwned tells whether the press hit the UI.
    // (pick callbacks queued by such a press are discarded separately, see viewport_click())
    if (mouseHold && (!ImGui::GetIO().MouseDown[0] || !viewport_click()))
    {
        mouseHold = false;
        m_dragging = false;
        m_input.mouse.button(GLFW_MOUSE_BUTTON_LEFT, GLFW_RELEASE);
    }

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

    // note: the streamer is advanced centrally in the application main loop (so it runs in every
    // mode), using the published Global.pCamera; nothing to do here

    // continuous terrain sculpting while the left mouse button is held in sculpt mode
    if (m_terrain_sculpt && mouseHold)
        handle_terrain_sculpt(deltarealtime);

    // debounced auto mesh simplification: once sculpting has settled for a short while, simplify
    // any chunk that was edited. holding the brush keeps the timer reset so we don't churn mid-stroke.
    if (m_terrain_auto_optimize)
    {
        auto const terrains = active_terrains();
        bool any_dirty = false;
        for (editor_terrain *terrain : terrains)
            if (terrain->dirty())
            {
                any_dirty = true;
                break;
            }

        if (!any_dirty || (m_terrain_sculpt && mouseHold))
        {
            m_terrain_idle = 0.0; // actively editing (or nothing pending): hold off
        }
        else
        {
            m_terrain_idle += deltarealtime;
            if (m_terrain_idle >= 0.5) // settle time
            {
                for (editor_terrain *terrain : terrains)
                    if (terrain->dirty())
                        terrain->optimize(m_terrain_simplify_error);
                m_terrain_idle = 0.0;
            }
        }
    }

    // --- ImGuizmo: in-viewport transform gizmo for the selected node ---
    render_gizmo();

    // --- area fill: outline overlay and settings, while the mode is active ---
    if (ui()->mode() == nodebank_panel::FILL)
    {
        draw_area_fill_outline();
        render_area_fill();
    }

    // --- ImGui: Editor Settings & History windows ---
    if(m_settings_open)
        render_settings();

    if(!m_change_history) return true;

    render_change_history();

    return true;
}

void editor_mode::render_settings()
{
    ImGui::Begin("Editor Settings", &m_settings_open, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::TextUnformatted("Camera movement");

    const char *schemes[] = {"WSAD (new)", "Arrows (legacy)"};
    int current = EditorSettings.movement() == editorSettings::movement_scheme::legacy ? 1 : 0;
    if (ImGui::Combo("##movement_scheme", &current, schemes, IM_ARRAYSIZE(schemes)))
    {
        EditorSettings.movement(current == 1 ? editorSettings::movement_scheme::legacy
                                             : editorSettings::movement_scheme::wsad);
        m_input.keyboard.apply_scheme();
        EditorSettings.save();
    }

    ImGui::Separator();
    ImGui::Checkbox("Transform gizmo (ImGuizmo)", &m_gizmo_enabled);

    render_terrain_ui();

    ImGui::End();
}

void editor_mode::render_terrain_ui()
{
    ImGui::Separator();
    ImGui::TextUnformatted("Terrain");

    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("Grid cells", &m_terrain_cells);
    m_terrain_cells = std::clamp(m_terrain_cells, 1, 512);
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("Cell size (m)", &m_terrain_cellsize);
    if (m_terrain_cellsize < 0.1f)
        m_terrain_cellsize = 0.1f;
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("Base height (m)", &m_terrain_baseheight);
    ImGui::SetNextItemWidth(200.0f);
    ImGui::InputText("Texture (optional)", m_terrain_texture, IM_ARRAYSIZE(m_terrain_texture));

    if (ImGui::Button("Create flat terrain"))
    {
        // centre the new patch horizontally on the camera, flat at the requested base height
        glm::dvec3 const center(Camera.Pos.x, static_cast<double>(m_terrain_baseheight), Camera.Pos.z);
        auto terrain = std::make_unique<editor_terrain>();
        if (terrain->create(center, m_terrain_cells, m_terrain_cellsize, std::string(m_terrain_texture)))
        {
            if (m_terrain_auto_optimize)
                terrain->optimize(m_terrain_simplify_error);
            m_terrains.push_back(std::move(terrain));
        }
        else
            WriteLog("Editor: failed to create terrain", logtype::generic);
    }

    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("Chunks / side", &m_terrain_chunks);
    m_terrain_chunks = std::clamp(m_terrain_chunks, 1, 32);
    ImGui::SameLine();
    if (ImGui::Button("Create chunked terrain"))
        create_chunked_terrain();
    ImGui::TextDisabled("total %d x %d m, %d chunks",
                        static_cast<int>(m_terrain_chunks * m_terrain_cells * m_terrain_cellsize),
                        static_cast<int>(m_terrain_chunks * m_terrain_cells * m_terrain_cellsize),
                        m_terrain_chunks * m_terrain_chunks);

    if (ImGui::Checkbox("Chunk edit mode (LMB add neighbour / Shift = delete)", &m_chunk_edit))
        if (m_chunk_edit)
            m_terrain_sculpt = false; // mutually exclusive with sculpting
    ImGui::Text("Grid chunks: %zu", m_grid_chunks.size());

    ImGui::Separator();
    ImGui::TextUnformatted("Streaming (open world, follows camera)");
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("Stream radius", &m_stream_radius);
    m_stream_radius = std::clamp(m_stream_radius, 0, 16);
    ImGui::Checkbox("Persist edits to disk (16-bit)", &m_stream_persist);
    bool streaming = m_streamer.active();
    if (ImGui::Checkbox("Stream terrain", &streaming))
    {
        if (streaming)
        {
            // per-scenery chunk folder so chunks from different sceneries don't collide
            std::string scenery = Global.SceneryFile;
            auto const slash = scenery.find_last_of("/\\");
            if (slash != std::string::npos)
                scenery = scenery.substr(slash + 1);
            auto const dot = scenery.find_last_of('.');
            if (dot != std::string::npos)
                scenery = scenery.substr(0, dot);
            if (scenery.empty())
                scenery = "default";
            m_streamer.directory("editor_terrain/" + scenery);

            m_streamer.configure(m_terrain_cells, m_terrain_cellsize, m_stream_radius,
                                 m_terrain_baseheight, std::string(m_terrain_texture));
            m_streamer.simplify(m_terrain_auto_optimize, m_terrain_simplify_error);
            m_streamer.persist(m_stream_persist);

            // hand the authored grid chunks over to streaming: persist them to disk, then drop the
            // in-memory meshes so the streamer owns residency (it loads them back within the radius)
            for (auto &entry : m_grid_chunks)
                if (entry.second)
                    m_streamer.save_chunk(entry.first.first, entry.first.second, *entry.second);
            for (auto &entry : m_grid_chunks)
                if (entry.second)
                    entry.second->destroy();
            m_grid_chunks.clear();
        }
        else
        {
            m_streamer.clear(); // saves modified chunks before dropping them
        }
        m_streamer.active(streaming);
    }
    if (m_streamer.active())
    {
        // radius / simplify / persist are safe to tweak live; chunk size/base are fixed at toggle
        m_streamer.radius(m_stream_radius);
        m_streamer.simplify(m_terrain_auto_optimize, m_terrain_simplify_error);
        m_streamer.persist(m_stream_persist);
        ImGui::Text("Resident chunks: %zu  (dir: %s)", m_streamer.resident(), m_streamer.directory().c_str());
    }

    ImGui::Text("Patches: %zu", m_terrains.size());

    // capture: sample the selected model's geometry into an editable patch and remove the original
    if (dynamic_cast<TAnimModel *>(m_node) != nullptr)
    {
        if (ImGui::Button("Capture selected model as terrain"))
            capture_terrain();
    }
    else
    {
        ImGui::TextDisabled("Capture: select a model instance first");
    }

    std::vector<editor_terrain *> const terrains = active_terrains();
    if (!terrains.empty())
    {
        ImGui::Separator();
        if (ImGui::Checkbox("Sculpt mode (LMB raise / Shift = lower)", &m_terrain_sculpt))
            if (m_terrain_sculpt)
                m_chunk_edit = false; // mutually exclusive with chunk editing
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("Brush radius", &m_terrain_brush_radius);
        if (m_terrain_brush_radius < 0.5f)
            m_terrain_brush_radius = 0.5f;
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("Brush strength", &m_terrain_brush_strength);

        // one-shot nudge of the most recent manual patch at its centre, handy for a quick test
        if (!m_terrains.empty())
        {
            auto &terrain = m_terrains.back();
            glm::dvec3 const c = terrain->centre();
            if (ImGui::Button("Raise centre"))
                terrain->sculpt(c.x, c.z, m_terrain_brush_radius, m_terrain_brush_strength);
            ImGui::SameLine();
            if (ImGui::Button("Lower centre"))
                terrain->sculpt(c.x, c.z, m_terrain_brush_radius, -m_terrain_brush_strength);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Optimize (mesh simplification, all patches)");
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("Flatness tol (m)", &m_terrain_simplify_error);
        if (m_terrain_simplify_error < 0.01f)
            m_terrain_simplify_error = 0.01f;
        ImGui::Checkbox("Auto-optimize after sculpt", &m_terrain_auto_optimize);
        if (ImGui::Button("Optimize all"))
            for (editor_terrain *t : terrains)
                t->optimize(m_terrain_simplify_error);
        ImGui::SameLine();
        if (ImGui::Button("Full-res all"))
            for (editor_terrain *t : terrains)
                t->unoptimize();

        std::size_t tris = 0, full = 0;
        for (editor_terrain *t : terrains)
        {
            tris += t->triangles();
            full += t->full_triangles();
        }
        ImGui::Text("Triangles: %zu / %zu", tris, full);
    }
}

editor_terrain *editor_mode::terrain_at(double X, double Z)
{
    for (auto &terrain : m_terrains)
        if (terrain && terrain->contains(X, Z))
            return terrain.get();
    double const size = chunk_grid_size();
    auto const it = m_grid_chunks.find({static_cast<int>(std::floor(X / size)), static_cast<int>(std::floor(Z / size))});
    if (it != m_grid_chunks.end() && it->second && it->second->contains(X, Z))
        return it->second.get();
    return m_streamer.terrain_at(X, Z);
}

std::vector<editor_terrain *> editor_mode::active_terrains()
{
    std::vector<editor_terrain *> out;
    out.reserve(m_terrains.size() + m_grid_chunks.size() + m_streamer.resident());
    for (auto &terrain : m_terrains)
        if (terrain)
            out.push_back(terrain.get());
    for (auto &entry : m_grid_chunks)
        if (entry.second)
            out.push_back(entry.second.get());
    m_streamer.collect(out);
    return out;
}

void editor_mode::add_grid_chunk(int Cx, int Cz)
{
    std::pair<int, int> const key{Cx, Cz};
    if (m_grid_chunks.count(key))
        return; // already occupied

    double const size = chunk_grid_size();
    int const cells = std::clamp(m_terrain_cells, 1, 256);
    glm::dvec3 const center((Cx + 0.5) * size, static_cast<double>(m_terrain_baseheight), (Cz + 0.5) * size);

    auto terrain = std::make_unique<editor_terrain>();
    if (!terrain->create(center, cells, m_terrain_cellsize, std::string(m_terrain_texture)))
        return;
    if (m_terrain_auto_optimize)
        terrain->optimize(m_terrain_simplify_error);
    m_grid_chunks[key] = std::move(terrain);
}

void editor_mode::remove_grid_chunk(int Cx, int Cz)
{
    auto const it = m_grid_chunks.find({Cx, Cz});
    if (it == m_grid_chunks.end())
        return;
    if (it->second)
        it->second->destroy();
    m_grid_chunks.erase(it);
}

void editor_mode::handle_chunk_edit_click(bool DeleteMode)
{
    // world point under the cursor; must land on existing geometry to give a valid depth
    glm::dvec3 const world = Camera.Pos + GfxRenderer->Mouse_Position();
    double const size = chunk_grid_size();
    int const cx = static_cast<int>(std::floor(world.x / size));
    int const cz = static_cast<int>(std::floor(world.z / size));
    bool const streaming = m_streamer.active();

    if (DeleteMode)
    {
        if (streaming)
            m_streamer.remove_chunk(cx, cz);
        else
            remove_grid_chunk(cx, cz);
        return;
    }

    // if the clicked cell holds a chunk, target the neighbour nearest the clicked edge (the empty
    // side); otherwise fill the clicked cell
    bool const occupied = streaming
                              ? m_streamer.terrain_at(world.x, world.z) != nullptr
                              : m_grid_chunks.count({cx, cz}) > 0;
    int tcx = cx, tcz = cz;
    if (occupied)
    {
        double const lx = world.x - cx * size, lz = world.z - cz * size;
        double const dw = lx, de = size - lx, dn = lz, ds = size - lz;
        double const nearest = std::min({dw, de, dn, ds});
        if (nearest == dw)
            tcx = cx - 1;
        else if (nearest == de)
            tcx = cx + 1;
        else if (nearest == dn)
            tcz = cz - 1;
        else
            tcz = cz + 1;
    }

    if (streaming)
        m_streamer.add_chunk(tcx, tcz);
    else
        add_grid_chunk(tcx, tcz);
}

void editor_mode::create_chunked_terrain()
{
    int const chunks = std::clamp(m_terrain_chunks, 1, 32);
    double const size = chunk_grid_size();

    // snap the field to the global chunk grid (so it aligns with manual/streamed chunks), centred
    // on the camera's chunk
    int const ccx = static_cast<int>(std::floor(Camera.Pos.x / size));
    int const ccz = static_cast<int>(std::floor(Camera.Pos.z / size));
    int const half = chunks / 2;

    int created = 0;
    for (int dz = 0; dz < chunks; ++dz)
        for (int dx = 0; dx < chunks; ++dx)
        {
            int const cx = ccx - half + dx, cz = ccz - half + dz;
            if (!m_grid_chunks.count({cx, cz}))
            {
                add_grid_chunk(cx, cz);
                ++created;
            }
        }

    WriteLog("Editor: created chunked terrain with " + std::to_string(created) + " chunks", logtype::generic);
}

void editor_mode::save_scene_with_terrain()
{
    // commit authored terrain so the scenery streams it on load. if not already streaming, hand the
    // manual grid chunks over to the streamer (same as toggling Stream terrain on)
    if (!m_streamer.active())
    {
        std::string scenery = Global.SceneryFile;
        auto const slash = scenery.find_last_of("/\\");
        if (slash != std::string::npos)
            scenery = scenery.substr(slash + 1);
        auto const dot = scenery.find_last_of('.');
        if (dot != std::string::npos)
            scenery = scenery.substr(0, dot);
        if (scenery.empty())
            scenery = "default";

        m_streamer.directory("editor_terrain/" + scenery);
        m_streamer.configure(m_terrain_cells, m_terrain_cellsize, m_stream_radius, m_terrain_baseheight,
                             std::string(m_terrain_texture));
        m_streamer.simplify(m_terrain_auto_optimize, m_terrain_simplify_error);
        m_streamer.persist(true);

        for (auto &entry : m_grid_chunks)
            if (entry.second)
                m_streamer.save_chunk(entry.first.first, entry.first.second, *entry.second);
        for (auto &entry : m_grid_chunks)
            if (entry.second)
                entry.second->destroy();
        m_grid_chunks.clear();
        m_streamer.active(true);
    }

    m_streamer.flush(); // save resident edited chunks to disk

    // export scenery; the exported .scm now carries an `editorterrain` directive (streamer is active)
    simulation::State.export_as_text(Global.SceneryFile);
    WriteLog("Editor: saved scene + terrain", logtype::generic);
}

void editor_mode::handle_terrain_sculpt(double Deltatime)
{
    // world point under the cursor (Mouse_Position is camera-relative, like the brush placement uses)
    glm::dvec3 const world = Camera.Pos + GfxRenderer->Mouse_Position();
    // only sculpt when the cursor is actually over terrain (avoids editing on a stale depth read)
    if (terrain_at(world.x, world.z) == nullptr)
        return;

    double const rate = m_terrain_brush_strength * Deltatime; // metres applied this frame
    double const signedrate = Global.shiftState ? -rate : rate;
    // apply to every chunk the brush touches; each patch clips to its own bounds, so a stroke
    // crossing a chunk boundary edits both and shared-edge vertices stay in sync
    for (editor_terrain *terrain : active_terrains())
        terrain->sculpt(world.x, world.z, m_terrain_brush_radius, signedrate);
}

void editor_mode::capture_terrain()
{
    TAnimModel *model = dynamic_cast<TAnimModel *>(m_node);
    if (model == nullptr || model->pModel == nullptr)
    {
        WriteLog("Editor: select a model instance to capture as terrain", logtype::generic);
        return;
    }

    std::vector<world_triangle> tris;
    gather_submodel_triangles(model->pModel->Root, instance_matrix(*model), tris);
    if (tris.empty())
    {
        WriteLog("Editor: selected model has no readable geometry to capture", logtype::generic);
        return;
    }

    // horizontal bounds of the captured geometry
    glm::dvec3 lo(std::numeric_limits<double>::max());
    glm::dvec3 hi(-std::numeric_limits<double>::max());
    for (auto const &t : tris)
        for (auto const &p : t)
        {
            lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
            hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
        }

    glm::dvec3 const center((lo.x + hi.x) * 0.5, lo.y, (lo.z + hi.z) * 0.5);
    double const extent = std::max(hi.x - lo.x, hi.z - lo.z);
    int const cells = std::max(1, m_terrain_cells);
    float const cellsize = static_cast<float>(std::max(0.1, extent / cells));

    // sampler: highest captured triangle at (x,z)
    auto const sampler = [&tris](double X, double Z, double &OutY) -> bool {
        double best = -std::numeric_limits<double>::max();
        bool found = false;
        for (auto const &t : tris)
        {
            double const minx = std::min({t[0].x, t[1].x, t[2].x});
            double const maxx = std::max({t[0].x, t[1].x, t[2].x});
            double const minz = std::min({t[0].z, t[1].z, t[2].z});
            double const maxz = std::max({t[0].z, t[1].z, t[2].z});
            if (X < minx || X > maxx || Z < minz || Z > maxz)
                continue;
            double y;
            if (triangle_height_at(t[0], t[1], t[2], X, Z, y) && (!found || y > best))
            {
                best = y;
                found = true;
            }
        }
        if (found)
            OutY = best;
        return found;
    };

    auto terrain = std::make_unique<editor_terrain>();
    if (!terrain->create(center, cells, cellsize, std::string(m_terrain_texture), sampler))
    {
        WriteLog("Editor: terrain capture failed", logtype::generic);
        return;
    }
    m_terrains.push_back(std::move(terrain));

    // remove the original instance (recorded as a deletion so it can be undone)
    std::string as_text;
    model->export_as_text(as_text);
    push_snapshot(model, EditorSnapshot::Action::Delete, as_text);
    nullify_history_pointers(model);
    remove_from_hierarchy(model);
    m_node = nullptr;
    m_dragging = false;
    ui()->set_node(nullptr);
    simulation::State.delete_model(model);
}

void editor_mode::add_area_fill_point()
{
    // like the other placement tools, the point comes from the depth buffer under the cursor
    // (ground geometry only), relative to the camera
    glm::dvec3 const offset = GfxRenderer->Mouse_Position();
    if (glm::length(offset) < 1e-3)
        return; // no ground under the cursor (yet)

    m_fill_points.push_back(Camera.Pos + offset);
}

void editor_mode::undo_last_area_fill()
{
    // nullify_history_pointers() removes deleted nodes from m_fill_last, so work on a copy
    auto const created = std::move(m_fill_last);
    m_fill_last.clear();

    for (TAnimModel *model : created)
    {
        if (model == nullptr)
            continue;
        if (m_node == model)
        {
            m_node = nullptr;
            ui()->set_node(nullptr);
        }
        nullify_history_pointers(model);
        remove_from_hierarchy(model);
        simulation::State.delete_model(model);
    }
    m_fill_status = "Removed " + std::to_string(created.size()) + " objects of the last fill";
}

void editor_mode::run_area_fill()
{
    // limits keeping a single click from freezing the editor for too long
    std::size_t constexpr max_objects = 20000;
    // large model instances (e.g. terrain tiles) are treated as ground, smaller ones (houses, trees) are not
    float constexpr terrain_model_radius = 50.0f;
    // ground is searched up to this far above the height interpolated from the outline points,
    // so hills inside the outline are found but bridges or roofs high above it are not
    double constexpr height_tolerance = 50.0;

    if (m_fill_points.size() < 3)
    {
        m_fill_status = "Outline needs at least 3 points";
        return;
    }

    // model package
    std::vector<std::string> package;
    if (m_fill_package == 0)
        package = m_fill_custom;
    else
        package = ui()->nodebank_group_templates(static_cast<std::size_t>(m_fill_package - 1));
    package.erase(std::remove_if(package.begin(), package.end(), [](std::string const &Template) { return Template.empty(); }), package.end());
    if (package.empty())
    {
        m_fill_status = "Selected model package is empty";
        return;
    }

    // area bounds
    glm::dvec2 bmin(std::numeric_limits<double>::max());
    glm::dvec2 bmax(-std::numeric_limits<double>::max());
    for (auto const &p : m_fill_points)
    {
        bmin = glm::min(bmin, glm::dvec2(p.x, p.z));
        bmax = glm::max(bmax, glm::dvec2(p.x, p.z));
    }
    double const area = polygon_area_xz(m_fill_points);
    auto const target = static_cast<std::size_t>(std::min<double>(std::round(area / 10000.0 * std::max(0.0f, m_fill_density)), max_objects));
    if (target == 0)
    {
        m_fill_status = "Density too low for this area, nothing to place";
        return;
    }

    // scatter points: random samples inside the outline, rejecting the ones closer than the minimal spacing
    // to an already accepted point (checked on a hashed grid with cell size == spacing)
    double const spacing = std::max(0.0f, m_fill_min_spacing);
    std::unordered_map<std::int64_t, std::vector<std::size_t>> occupancy;
    auto const cell_key = [](std::int64_t const X, std::int64_t const Z) { return (X << 32) ^ (Z & 0xffffffff); };
    std::vector<glm::dvec2> points;
    points.reserve(target);
    for (std::size_t attempt = 0; attempt < target * 30 && points.size() < target; ++attempt)
    {
        glm::dvec2 const p(LocalRandom(bmin.x, bmax.x), LocalRandom(bmin.y, bmax.y));
        if (false == point_in_polygon_xz(m_fill_points, p.x, p.y))
            continue;
        if (spacing > 0.0)
        {
            auto const cx = static_cast<std::int64_t>(std::floor(p.x / spacing));
            auto const cz = static_cast<std::int64_t>(std::floor(p.y / spacing));
            bool free = true;
            for (std::int64_t dz = -1; dz <= 1 && free; ++dz)
                for (std::int64_t dx = -1; dx <= 1 && free; ++dx)
                {
                    auto const lookup = occupancy.find(cell_key(cx + dx, cz + dz));
                    if (lookup == occupancy.end())
                        continue;
                    for (auto const idx : lookup->second)
                        if (glm::distance(points[idx], p) < spacing)
                        {
                            free = false;
                            break;
                        }
                }
            if (!free)
                continue;
            occupancy[cell_key(cx, cz)].push_back(points.size());
        }
        points.push_back(p);
    }

    // ground geometry within the area. shapes hold the (legacy) terrain, large instances the terrain tiles
    std::vector<world_triangle> triangles;
    glm::dvec2 const center = (bmin + bmax) * 0.5;
    float const radius = static_cast<float>(glm::length(bmax - bmin) * 0.5);
    auto const sections = simulation::Region->sections(glm::dvec3(center.x, 0.0, center.y), radius); // copy, the result is a scratchpad
    for (auto *section : sections)
    {
        for (auto const &shape : section->m_shapes)
            gather_shape_triangles(shape, bmin, bmax, triangles);
        for (auto &cell : section->m_cells)
        {
            double const r = cell.m_area.radius;
            if (cell.m_area.center.x + r < bmin.x || cell.m_area.center.x - r > bmax.x || cell.m_area.center.z + r < bmin.y || cell.m_area.center.z - r > bmax.y)
                continue;
            for (auto const &shape : cell.m_shapesopaque)
                gather_shape_triangles(shape, bmin, bmax, triangles);
            if (false == m_fill_models_as_ground)
                continue;
            for (auto *instance : cell.m_instancesopaque)
            {
                if (instance == nullptr || instance->Model() == nullptr || instance->radius() < terrain_model_radius)
                    continue;
                std::vector<world_triangle> modeltriangles;
                gather_submodel_triangles(instance->Model()->Root, instance_matrix(*instance), modeltriangles);
                for (auto const &t : modeltriangles)
                    if (std::max({t[0].x, t[1].x, t[2].x}) >= bmin.x && std::min({t[0].x, t[1].x, t[2].x}) <= bmax.x && std::max({t[0].z, t[1].z, t[2].z}) >= bmin.y &&
                        std::min({t[0].z, t[1].z, t[2].z}) <= bmax.y)
                        triangles.push_back(t);
            }
        }
    }
    triangle_grid const ground(std::move(triangles), bmin, bmax);
    auto const terrains = active_terrains();

    auto const rotation_mode = m_fill_random_rotation ? functions_panel::RANDOM : ui()->rot_mode();
    auto const fixed_rotation_value = ui()->rot_val();
    float const scalemin = std::max(0.01f, std::min(m_fill_scale_min, m_fill_scale_max));
    float const scalemax = std::max(0.01f, std::max(m_fill_scale_min, m_fill_scale_max));

    std::vector<TAnimModel *> created;
    created.reserve(points.size());
    std::size_t unsupported = 0;
    for (auto const &p : points)
    {
        // reference height: the outline points, weighted by inverse squared distance
        double weights = 0.0, reference = 0.0;
        for (auto const &v : m_fill_points)
        {
            double const w = 1.0 / std::max(1e-6, glm::length2(glm::dvec2(v.x, v.z) - p));
            weights += w;
            reference += w * v.y;
        }
        reference /= weights;
        double const ceiling = reference + height_tolerance;

        double y = -std::numeric_limits<double>::max();
        bool found = ground.height_at(p.x, p.y, ceiling, y);
        for (editor_terrain *terrain : terrains)
        {
            if (!terrain->contains(p.x, p.y))
                continue;
            double const h = terrain->height_at(p.x, p.y);
            if (h <= ceiling && (!found || h > y))
            {
                y = h;
                found = true;
            }
        }
        if (!found)
        {
            y = reference;
            ++unsupported;
        }

        glm::dvec3 const location(p.x, y, p.y);
        if (!simulation::Region->point_inside(location))
            continue;

        auto const &src = package[package.size() > 1 ? std::min(package.size() - 1, static_cast<std::size_t>(LocalRandom(0.0, static_cast<double>(package.size())))) : 0];
        TAnimModel *cloned = simulation::State.create_model(src, "editor_", location);
        if (!cloned)
            continue;
        cloned->m_name = "editor_" + cloned->uuid.to_string();
        apply_rotation_for_new_node(cloned, rotation_mode, fixed_rotation_value);
        if (scalemin != 1.0f || scalemax != 1.0f)
            cloned->Scale(static_cast<float>(LocalRandom(scalemin, scalemax)));
        created.push_back(cloned);
    }

    // a new fill replaces the "last fill" undo set; the previous objects stay in the scene
    m_fill_last = std::move(created);

    m_fill_status = "Placed " + std::to_string(m_fill_last.size()) + " of " + std::to_string(target) + " objects";
    if (points.size() < target)
        m_fill_status += " (limited by min. spacing)";
    if (unsupported > 0)
        m_fill_status += ", " + std::to_string(unsupported) + " without ground found (height interpolated)";
    WriteLog("Editor: area fill - " + m_fill_status, logtype::generic);
}

void editor_mode::draw_area_fill_outline() const
{
    if (m_fill_points.empty())
        return;

    // same camera-relative view and clean perspective the transform gizmo uses
    ImGuiIO const &io = ImGui::GetIO();
    glm::mat4 const view = GfxRenderer->Camera_View_Matrix();
    glm::dvec3 const camerapos = GfxRenderer->Camera_Position();
    float const fovy = glm::radians(Global.FieldOfView / Global.ZoomFactor);
    float const aspect = io.DisplaySize.y > 0.0f ? io.DisplaySize.x / io.DisplaySize.y : 1.0f;
    glm::mat4 const viewprojection = glm::perspective(fovy, aspect, 0.1f, 10000.0f) * view;

    auto const to_clip = [&](glm::dvec3 const &Point) { return viewprojection * glm::vec4(glm::vec3(Point - camerapos), 1.0f); };
    auto const to_screen = [&](glm::vec4 const &Clip) {
        return ImVec2((Clip.x / Clip.w * 0.5f + 0.5f) * io.DisplaySize.x, (0.5f - Clip.y / Clip.w * 0.5f) * io.DisplaySize.y);
    };
    float constexpr nearw = 0.1f;

    ImDrawList *drawlist = ImGui::GetBackgroundDrawList();
    ImU32 const edgecolor = IM_COL32(255, 200, 40, 230);
    ImU32 const closingcolor = IM_COL32(255, 200, 40, 110);

    std::size_t const count = m_fill_points.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        bool const closing = (i + 1 == count);
        if (closing && count < 3)
            break;
        glm::vec4 a = to_clip(m_fill_points[i]);
        glm::vec4 b = to_clip(m_fill_points[(i + 1) % count]);
        // clip the edge against the near plane, so points behind the camera don't flip across the screen
        if (a.w < nearw && b.w < nearw)
            continue;
        if (a.w < nearw)
            a = glm::mix(a, b, (nearw - a.w) / (b.w - a.w));
        else if (b.w < nearw)
            b = glm::mix(b, a, (nearw - b.w) / (a.w - b.w));
        drawlist->AddLine(to_screen(a), to_screen(b), closing ? closingcolor : edgecolor, 2.0f);
    }
    for (std::size_t i = 0; i < count; ++i)
    {
        glm::vec4 const c = to_clip(m_fill_points[i]);
        if (c.w < nearw)
            continue;
        drawlist->AddCircleFilled(to_screen(c), i == 0 ? 6.0f : 4.0f, i == 0 ? IM_COL32(255, 90, 40, 255) : edgecolor);
    }
}

void editor_mode::render_area_fill()
{
    ImGui::Begin("Area fill", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing);

    ImGui::TextUnformatted("LMB: add outline point, Backspace: remove last point");
    double const area = polygon_area_xz(m_fill_points);
    ImGui::Text("Points: %zu   Area: %.0f m2 (%.2f ha)", m_fill_points.size(), area, area / 10000.0);
    if (ImGui::Button("Remove last point") && !m_fill_points.empty())
        m_fill_points.pop_back();
    ImGui::SameLine();
    if (ImGui::Button("Clear outline"))
        m_fill_points.clear();

    ImGui::Separator();

    // model package: a node bank group, or a custom set assembled from node bank templates
    auto const groups = ui()->nodebank_groups();
    m_fill_package = std::clamp(m_fill_package, 0, static_cast<int>(groups.size()));
    std::string const current = m_fill_package == 0 ? std::string("Custom set") : groups[m_fill_package - 1];
    ImGui::SetNextItemWidth(250.0f);
    if (ImGui::BeginCombo("Model package", current.c_str()))
    {
        if (ImGui::Selectable("Custom set", m_fill_package == 0))
            m_fill_package = 0;
        for (int i = 0; i < static_cast<int>(groups.size()); ++i)
        {
            auto const label = groups[i] + "##fillgroup" + std::to_string(i);
            if (ImGui::Selectable(label.c_str(), m_fill_package == i + 1))
                m_fill_package = i + 1;
        }
        ImGui::EndCombo();
    }

    if (m_fill_package == 0)
    {
        ImGui::BeginChild("fill_custom_set", ImVec2(350.0f, 110.0f), true);
        for (int i = 0; i < static_cast<int>(m_fill_custom.size()); ++i)
        {
            auto const label = template_label(m_fill_custom[i]) + "##fillcustom" + std::to_string(i);
            if (ImGui::Selectable(label.c_str(), m_fill_custom_idx == i))
                m_fill_custom_idx = i;
        }
        ImGui::EndChild();
        if (ImGui::Button("Add selected template"))
        {
            // the node bank selection, not a random pick from the brush set
            auto const *src = ui()->get_active_node_template(true);
            if (src && !src->empty())
                m_fill_custom.push_back(*src);
        }
        ImGui::SameLine();
        if (ImGui::Button("Remove") && m_fill_custom_idx >= 0 && m_fill_custom_idx < static_cast<int>(m_fill_custom.size()))
        {
            m_fill_custom.erase(m_fill_custom.begin() + m_fill_custom_idx);
            m_fill_custom_idx = -1;
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear set"))
        {
            m_fill_custom.clear();
            m_fill_custom_idx = -1;
        }
    }
    else
    {
        ImGui::Text("Templates in group: %zu", ui()->nodebank_group_templates(static_cast<std::size_t>(m_fill_package - 1)).size());
    }

    ImGui::Separator();

    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("Density (objects/ha)", &m_fill_density, 10.0f, 100.0f, "%.1f");
    m_fill_density = std::max(0.0f, m_fill_density);
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("Min. spacing (m)", &m_fill_min_spacing, 0.5f, 2.0f, "%.1f");
    m_fill_min_spacing = std::max(0.0f, m_fill_min_spacing);
    ImGui::SetNextItemWidth(200.0f);
    ImGui::DragFloatRange2("Scale", &m_fill_scale_min, &m_fill_scale_max, 0.01f, 0.1f, 5.0f, "min %.2f", "max %.2f");
    ImGui::Checkbox("Random rotation (off: Functions panel setting)", &m_fill_random_rotation);
    ImGui::Checkbox("Large models count as ground (terrain tiles)", &m_fill_models_as_ground);

    ImGui::Text("Estimated objects: %.0f", std::round(area / 10000.0 * m_fill_density));

    if (ImGui::Button("Fill area"))
        run_area_fill();
    ImGui::SameLine();
    std::string const undolabel = "Undo last fill (" + std::to_string(m_fill_last.size()) + ")";
    if (ImGui::Button(undolabel.c_str()) && !m_fill_last.empty())
        undo_last_area_fill();

    if (!m_fill_status.empty())
        ImGui::TextUnformatted(m_fill_status.c_str());

    ImGui::End();
}

void editor_mode::render_gizmo()
{
    // the transform gizmo is suppressed while editing terrain, so the brush/chunk tool owns the mouse
    if (!m_gizmo_enabled || m_terrain_sculpt || m_chunk_edit)
    {
        m_gizmo_using = false;
        return;
    }

    // compact control window: lets the user pick the transform mode without keyboard shortcuts
    ImGui::Begin("Gizmo", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing);
    int op = static_cast<int>(m_gizmo_op);
    ImGui::RadioButton("Translate (Q)", &op, static_cast<int>(gizmo_operation::translate));
    ImGui::SameLine();
    ImGui::RadioButton("Rotate (W)", &op, static_cast<int>(gizmo_operation::rotate));
    ImGui::SameLine();
    ImGui::RadioButton("Scale (E)", &op, static_cast<int>(gizmo_operation::scale));
    m_gizmo_op = static_cast<gizmo_operation>(op);

    if (m_gizmo_op != gizmo_operation::scale) // ImGuizmo always scales in local space
        ImGui::Checkbox("Local space (R)", &m_gizmo_local);
    if (m_gizmo_op == gizmo_operation::translate)
    {
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("Snap (hold Ctrl)", &m_gizmo_snap);
        if (m_gizmo_snap < 0.0f)
            m_gizmo_snap = 0.0f;
    }
    if (!m_node)
        ImGui::TextDisabled("No node selected");
    ImGui::End();

    if (!m_node)
    {
        m_gizmo_using = false;
        return;
    }

    ImGuizmo::BeginFrame();
    ImGuizmo::SetOrthographic(false);

    ImGuiIO const &io = ImGui::GetIO();
    ImGuizmo::SetRect(0.0f, 0.0f, io.DisplaySize.x, io.DisplaySize.y);

    // the view matrix comes from the most recent color pass and is camera-relative
    // (rotation only), so the gizmo is positioned relative to the camera as well.
    glm::mat4 const view = GfxRenderer->Camera_View_Matrix();
    glm::dvec3 const camerapos = GfxRenderer->Camera_Position();

    // the engine's own projection bakes in reverse-Z (and screen orientation), which ImGuizmo
    // doesn't expect; rebuild a clean, standard perspective that matches the rendered view.
    // for the main viewport the engine uses a symmetric frustum with this exact fov/aspect.
    float const fovy = glm::radians(Global.FieldOfView / Global.ZoomFactor);
    float const aspect = io.DisplaySize.y > 0.0f ? io.DisplaySize.x / io.DisplaySize.y : 1.0f;
    glm::mat4 const projection = glm::perspective(fovy, aspect, 0.1f, 10000.0f);

    // rotation/scale are only meaningful for instanced models; other node types translate only
    TAnimModel *model = dynamic_cast<TAnimModel *>(m_node);

    glm::vec3 const relativepos = glm::vec3(m_node->location() - camerapos);
    glm::vec3 const angles = model ? model->Angles() : glm::vec3(0.0f);
    glm::vec3 const scalevec = model ? model->Scale() : glm::vec3(1.0f);

    // build the gizmo model matrix from the node's current translation + rotation + scale
    float const translation[3] = {relativepos.x, relativepos.y, relativepos.z};
    float const rotation[3] = {angles.x, angles.y, angles.z};
    float const scale[3] = {scalevec.x, scalevec.y, scalevec.z};
    glm::mat4 matrix(1.0f);
    ImGuizmo::RecomposeMatrixFromComponents(translation, rotation, scale, glm::value_ptr(matrix));

    // map the editor's transform mode onto ImGuizmo; fall back to translate for non-models
    ImGuizmo::OPERATION operation = ImGuizmo::TRANSLATE;
    EditorSnapshot::Action action = EditorSnapshot::Action::Move;
    if (model && m_gizmo_op == gizmo_operation::rotate)
    {
        operation = ImGuizmo::ROTATE;
        action = EditorSnapshot::Action::Rotate;
    }
    else if (model && m_gizmo_op == gizmo_operation::scale)
    {
        operation = ImGuizmo::SCALE;
        action = EditorSnapshot::Action::Scale;
    }
    ImGuizmo::MODE const mode = m_gizmo_local ? ImGuizmo::LOCAL : ImGuizmo::WORLD;

    // optional snapping while Ctrl is held (metres / degrees / scale factor depending on mode)
    glm::vec3 snapvalue(0.0f);
    if (operation == ImGuizmo::TRANSLATE)
        snapvalue = glm::vec3(m_gizmo_snap);
    else if (operation == ImGuizmo::ROTATE)
        snapvalue = glm::vec3(5.0f);
    else
        snapvalue = glm::vec3(0.1f);
    float const *snap = Global.ctrlState && snapvalue.x > 0.0f ? glm::value_ptr(snapvalue) : nullptr;

    ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(projection),
                         operation, mode, glm::value_ptr(matrix), nullptr, snap);

    if (ImGuizmo::IsUsing())
    {
        // record a single undo snapshot at the start of the drag
        if (!m_gizmo_using)
        {
            push_snapshot(m_node, action);
            m_gizmo_using = true;
        }

        float newtranslation[3], newrotation[3], newscale[3];
        ImGuizmo::DecomposeMatrixToComponents(glm::value_ptr(matrix), newtranslation, newrotation, newscale);

        if (operation == ImGuizmo::ROTATE && model)
        {
            // apply the rotation delta relative to the model's current orientation
            glm::vec3 const newangles(newrotation[0], newrotation[1], newrotation[2]);
            m_editor.rotate(model, newangles - model->Angles(), 0.0f);
        }
        else if (operation == ImGuizmo::SCALE && model)
        {
            model->Scale(glm::vec3(newscale[0], newscale[1], newscale[2]));
        }
        else
        {
            glm::dvec3 const newworldpos = camerapos + glm::dvec3(newtranslation[0], newtranslation[1], newtranslation[2]);
            // pass Snaptoground == true so the gizmo's Y component is applied (free 3D move)
            m_editor.translate(m_node, newworldpos, true);
        }
    }
    else
    {
        m_gizmo_using = false;
    }
}

void editor_mode::update_camera(double const Deltatime)
{
    Camera.Update();

    // focus animation runs after Camera.Update() so it overrides any residual velocity/rotation;
    // it smoothly drives both position and orientation toward the framed object
    if (m_focus_active)
    {
        m_focus_time += Deltatime;
        double t = m_focus_duration > 0.0 ? m_focus_time / m_focus_duration : 1.0;
        if (t >= 1.0)
            t = 1.0;
        // smoothstep easing
        float const s = static_cast<float>(t * t * (3.0 - 2.0 * t));

        Camera.Pos = glm::mix(m_focus_start_pos, m_focus_target_pos, static_cast<double>(s));

        // interpolate angles, taking the shortest path around the yaw wrap-around
        constexpr float TWO_PI = 6.283185307179586f;
        float const dyaw = std::remainder(m_focus_target_angle.y - m_focus_start_angle.y, TWO_PI);
        Camera.Angle.x = m_focus_start_angle.x + (m_focus_target_angle.x - m_focus_start_angle.x) * s;
        Camera.Angle.y = m_focus_start_angle.y + dyaw * s;
        Camera.Angle.z = m_focus_start_angle.z + (m_focus_target_angle.z - m_focus_start_angle.z) * s;

        // suppress any residual fly velocity so it doesn't fight the animation
        Camera.Velocity = glm::dvec3(0.0);

        if (t >= 1.0)
            m_focus_active = false;
    }

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
            const int cab = vehicle->MoverParameters->CabOccupied == 0 ? 1 : vehicle->MoverParameters->CabOccupied;
            const glm::dvec3 left = vehicle->VectorLeft() * (double)cab;
            Camera.Pos = glm::dvec3(Camera.Pos.x, vehicle->GetPosition().y, Camera.Pos.z) + left * vehicle->GetWidth() + glm::dvec3(1.25f * left.x, 1.6f, 1.25f * left.z);
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
    m_fill_last.clear();

    // drop selection so a stale/dangling node pointer isn't used on the next editor session
    m_node = nullptr;
    m_gizmo_using = false;
    ui()->set_node(nullptr);

    Application.set_cursor(Global.ControlPicking ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_DISABLED);

    if (!Global.ControlPicking)
    {
        Application.set_cursor_pos(0, 0);
    }
}

void editor_mode::on_key(int const Key, int const Scancode, int const Action, int const Mods)
{
#ifndef __unix__
    Global.shiftState = Mods & GLFW_MOD_SHIFT ? true : false;
    Global.ctrlState = Mods & GLFW_MOD_CONTROL ? true : false;
    Global.altState = Mods & GLFW_MOD_ALT ? true : false;
#endif
    bool anyModifier = Mods & (GLFW_MOD_SHIFT | GLFW_MOD_CONTROL | GLFW_MOD_ALT);

    // first give UI a chance to handle the key
    if (!anyModifier && m_userinterface->on_key(Key, Action))
        return;

    // gizmo transform shortcuts (Q/W/E/R) — only when the camera isn't being flown (RMB up).
    // handled before the camera keyboard step because Q/W/E are also the fly-mode movement keys,
    // which would otherwise consume them.
    if (!anyModifier && is_press(Action)
        && m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) != GLFW_PRESS)
    {
        bool handled = true;
        switch (Key)
        {
        case GLFW_KEY_Q: m_gizmo_op = gizmo_operation::translate; break;
        case GLFW_KEY_W: m_gizmo_op = gizmo_operation::rotate; break;
        case GLFW_KEY_E: m_gizmo_op = gizmo_operation::scale; break;
        case GLFW_KEY_R: m_gizmo_local = !m_gizmo_local; break;
        default: handled = false; break;
        }
        if (handled)
            return;
    }

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

    case GLFW_KEY_BACKSPACE:
        // area fill: remove the last outline point
        if (is_press(Action) && ui()->mode() == nodebank_panel::FILL && !m_fill_points.empty())
            m_fill_points.pop_back();
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

    case GLFW_KEY_END:
        if (is_press(Action) && m_node)
        {
            // Unreal-style "snap to floor": drop the selected node onto the surface below it.
            // works against triangle geometry (shape_node terrain / opaque shapes); once a proper
            // editable terrain mesh exists, dropping onto it works without further changes here.
            snap_to_ground(m_node);
        }
        break;

    default:
        break;
    }
}

void editor_mode::on_cursor_pos(double const Horizontal, double const Vertical)
{
    // object transforms are handled by the gizmo now; here we only forward the cursor to the
    // mouse input, which rotates the camera while the right mouse button is held (panning mode)
    m_input.mouse.position(Horizontal, Vertical);
}

void editor_mode::on_mouse_button(int const Button, int const Action, int const Mods)
{
    // UI first
    if (m_userinterface->on_mouse_button(Button, Action))
    {
        m_input.mouse.button(Button, Action);
        return;
    }

    // in chunk-edit mode the left button adds a neighbouring chunk (Shift = delete the clicked one)
    if (m_chunk_edit && Button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (is_press(Action))
            handle_chunk_edit_click(Global.shiftState);
        m_input.mouse.button(Button, Action);
        return;
    }

    // in terrain sculpt mode the left button paints the terrain instead of picking nodes
    if (m_terrain_sculpt && Button == GLFW_MOUSE_BUTTON_LEFT)
    {
        mouseHold = is_press(Action);
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
            m_brush_has_last = false; // every brush stroke starts with a placement

            // in area fill mode the left button only adds outline points, it doesn't touch the selection
            if (mode == nodebank_panel::FILL)
            {
                GfxRenderer->Pick_Node_Callback([this](scene::basic_node * /*node*/) {
                    if (viewport_click())
                        add_area_fill_point();
                });
                m_input.mouse.button(Button, Action);
                return;
            }

            // delegate node picking behaviour depending on current panel mode.
            // NOTE: the pick is resolved a few frames later, often after the button was already released,
            // so the callback mustn't depend on the button still being held
            GfxRenderer->Pick_Node_Callback(
                [this, mode, rotation_mode, fixed_rotation_value](scene::basic_node *node) {
                    // the press turned out to be meant for the UI
                    if (!viewport_click())
                        return;

                    m_node = nullptr;

                    // ignore picks that are beyond allowed placement distance
                    if (node) {
                        double const dist = glm::distance(node->location(), glm::dvec3{Global.pCamera.Pos});
                        if (dist > static_cast<double>(kMaxPlacementDistance))
                            return;
                    }
                    if (mode == nodebank_panel::MODIFY)
                    {
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

                        std::string new_name = "editor_" + cloned->uuid.to_string();
                        cloned->m_name = new_name;
                        apply_rotation_for_new_node(cloned, rotation_mode, fixed_rotation_value);

                        // record addition for undo (after rotation, so a redo recreates the same thing)
                        std::string as_text;
                        cloned->export_as_text(as_text);
                        push_snapshot(cloned, EditorSnapshot::Action::Add, as_text);

                        m_node = cloned;
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
    else if (Button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        // game-engine style look: hide & grab the cursor while flying, restore it on release
        Application.set_cursor(is_press(Action) ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
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
                        s.action == EditorSnapshot::Action::Add ? "ADD" :
                        s.action == EditorSnapshot::Action::Delete ? "DEL" :
                        s.action == EditorSnapshot::Action::Move ? "MOV" :
                        s.action == EditorSnapshot::Action::Rotate ? "ROT" :
                        s.action == EditorSnapshot::Action::Scale ? "SCA" : "OTH",
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
    // game-engine style camera: WSAD/EQ only fly the camera while the right mouse button is held.
    // when it's released the keyboard is free for gizmo shortcuts, and we flush a zero-movement
    // command once so the camera doesn't keep coasting on the last velocity it was given.
    bool const flying = m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    if (flying)
    {
        m_input.poll();
    }
    else if (m_camera_flying)
    {
        m_camera_relay.post(user_command::movehorizontal, 0.0, 0.0, GLFW_PRESS, 0);
        m_camera_relay.post(user_command::movevertical, 0.0, 0.0, GLFW_PRESS, 0);
    }
    m_camera_flying = flying;
}

bool editor_mode::is_command_processor() const
{
    return false;
}

bool editor_mode::focus_active()
{
    return m_focus_active;
}

void editor_mode::set_focus_active(bool isActive)
{
    m_focus_active = isActive;
}
