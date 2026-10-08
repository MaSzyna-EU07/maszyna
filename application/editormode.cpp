/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include <filesystem>
#include <iterator>
#include <fstream>
#include <regex>
#include "application/editormode.h"
#include "application/editoruilayer.h"
#ifndef _WIN32
#include <spawn.h>
#include <unistd.h>
extern char **environ;
#endif
#include "application/editorprojection.h"

#include "application/application.h"
#include "editor/editorSettings.hpp"
#include "editor/editorModelSets.hpp"
#include "editor/editorIncludeInfo.hpp"
#include "editor/editorGroundMesh.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "utilities/translation.h"
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
#include "scene/scenelayers.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"


#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "imgui/ImGuizmo.h"
#include "utilities/Logs.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <array>
#include <chrono>
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

    // meshes of the shapes and the models asked about as ground. the ground is asked about a tile at a time, by the orthophoto
    // and by the road tools, and without these every question meant going through every triangle of every terrain model in reach
    struct ground_record
    {
        ground_mesh mesh;
        // what the mesh was made of, to tell when it has to be made again
        std::size_t vertices{0};
        std::size_t indices{0};
        glm::dvec3 origin{0.0};
        TModel3d const *model{nullptr};
        glm::dmat4 matrix{0.0};
        std::uint64_t used{0}; // number of the last question it served
    };
    std::unordered_map<std::uint64_t, ground_record> ground_shapes; // by the geometry of the shape
    std::unordered_map<TAnimModel const *, ground_record> ground_models;
    std::uint64_t ground_question{0};
    std::size_t ground_held{0}; // triangles in the meshes
    std::size_t constexpr ground_limit = 8000000; // triangles kept; some 400 MB with the grids
    std::size_t constexpr ground_small = 256; // triangles; anything smaller is quicker to go through than to look up

    // lets go of the meshes left alone the longest, once there's too much of them
    void trim_ground_meshes()
    {
        if (ground_held <= ground_limit)
            return;
        std::vector<std::uint64_t> ages;
        for (auto const &record : ground_shapes)
            ages.emplace_back(record.second.used);
        for (auto const &record : ground_models)
            ages.emplace_back(record.second.used);
        std::sort(ages.begin(), ages.end());
        // the older half goes, except for what the current question needed
        auto const threshold = std::min(ages[ages.size() / 2], ground_question - 1);
        for (auto record = ground_shapes.begin(); record != ground_shapes.end();)
        {
            if (record->second.used <= threshold)
            {
                ground_held -= record->second.mesh.size();
                record = ground_shapes.erase(record);
            }
            else
                ++record;
        }
        for (auto record = ground_models.begin(); record != ground_models.end();)
        {
            if (record->second.used <= threshold)
            {
                ground_held -= record->second.mesh.size();
                record = ground_models.erase(record);
            }
            else
                ++record;
        }
    }

    // forgets the meshes of the shapes, whose geometry can be changed in place (terrain being sculpted)
    void forget_ground_shapes()
    {
        for (auto const &record : ground_shapes)
            ground_held -= record.second.mesh.size();
        ground_shapes.clear();
    }

    // appends triangles of a shape node which overlap specified XZ rectangle
    void gather_shape_triangles(scene::shape_node const &Shape, glm::dvec2 const &Min, glm::dvec2 const &Max, std::vector<world_triangle> &Out, bool const Skiproads = false)
    {
        auto const &data = Shape.data();
        if (editor_orthophoto::owns(data.geometry))
            return; // the orthophoto laid over the ground isn't ground itself
        if (Skiproads && owned_shapes::owns(data.geometry))
            return; // neither are the roads, for the ones which ask where to put a road
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
        auto const count = (indices.empty() ? verts.size() : indices.size()) / 3;
        if (count >= ground_small)
        {
            // a large shape is sorted once, and asked for what's in the rectangle from then on
            auto &record = ground_shapes[(static_cast<std::uint64_t>(data.geometry.bank) << 32) | data.geometry.chunk];
            if (record.mesh.size() == 0 || record.vertices != verts.size() || record.indices != indices.size() || record.origin != data.origin)
            {
                std::vector<ground_mesh::triangle> triangles;
                triangles.reserve(count);
                if (false == indices.empty())
                {
                    for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
                        triangles.push_back({verts[indices[i]].position, verts[indices[i + 1]].position, verts[indices[i + 2]].position});
                }
                else
                {
                    for (std::size_t i = 0; i + 2 < verts.size(); i += 3)
                        triangles.push_back({verts[i].position, verts[i + 1].position, verts[i + 2].position});
                }
                ground_held -= record.mesh.size();
                record.mesh.assign(std::move(triangles), data.origin);
                ground_held += record.mesh.size();
                record.vertices = verts.size();
                record.indices = indices.size();
                record.origin = data.origin;
            }
            record.used = ground_question;
            record.mesh.collect(Min, Max, Out);
            return;
        }
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

    // ground triangles overlapping an XZ rectangle (Min/Max are x,z). shapes hold the (legacy) terrain,
    // model instances at least ModelRadius large (if ModelsAsGround) the terrain tiles
    void gather_ground_triangles(glm::dvec2 const &Min, glm::dvec2 const &Max, bool const ModelsAsGround, float const ModelRadius, std::vector<world_triangle> &Out, bool const Skiproads = false)
    {
        auto const started = std::chrono::steady_clock::now();
        ++ground_question;
        glm::dvec2 const center = (Min + Max) * 0.5;
        float const radius = static_cast<float>(glm::length(Max - Min) * 0.5);
        auto const sections = simulation::Region->sections(glm::dvec3(center.x, 0.0, center.y), radius); // copy, the result is a scratchpad
        for (auto *section : sections)
        {
            for (auto const &shape : section->m_shapes)
                gather_shape_triangles(shape, Min, Max, Out, Skiproads);
            for (auto &cell : section->m_cells)
            {
                double const r = cell.m_area.radius;
                if (cell.m_area.center.x + r < Min.x || cell.m_area.center.x - r > Max.x || cell.m_area.center.z + r < Min.y || cell.m_area.center.z - r > Max.y)
                    continue;
                for (auto const &shape : cell.m_shapesopaque)
                    gather_shape_triangles(shape, Min, Max, Out);
                if (false == ModelsAsGround)
                    continue;
                for (auto *instance : cell.m_instancesopaque)
                {
                    if (instance == nullptr || instance->Model() == nullptr || instance->radius() < ModelRadius)
                        continue;
                    // going through the triangles of a model which doesn't reach the rectangle is all cost and no gain
                    double const reach = instance->radius();
                    if (instance->location().x + reach < Min.x || instance->location().x - reach > Max.x || instance->location().z + reach < Min.y || instance->location().z - reach > Max.y)
                        continue;
                    // the model is gone through once, and asked for what's in the rectangle from then on, for as long as it stays put
                    auto const matrix = instance_matrix(*instance);
                    auto &record = ground_models[instance];
                    if (record.model != instance->Model() || record.matrix != matrix)
                    {
                        std::vector<world_triangle> modeltriangles;
                        gather_submodel_triangles(instance->Model()->Root, matrix, modeltriangles);
                        glm::dvec3 const origin = instance->location();
                        std::vector<ground_mesh::triangle> triangles;
                        triangles.reserve(modeltriangles.size());
                        for (auto const &t : modeltriangles)
                            triangles.push_back({glm::vec3(t[0] - origin), glm::vec3(t[1] - origin), glm::vec3(t[2] - origin)});
                        ground_held -= record.mesh.size();
                        record.mesh.assign(std::move(triangles), origin);
                        ground_held += record.mesh.size();
                        // a model which isn't loaded yet has nothing to show for itself, and is looked at again the next time
                        record.model = (record.mesh.size() > 0 ? instance->Model() : nullptr);
                        record.matrix = matrix;
                    }
                    record.used = ground_question;
                    record.mesh.collect(Min, Max, Out);
                }
            }
        }
        trim_ground_meshes();
        // going through the scenery is the one thing here which can take long enough to be felt; when it does it's worth knowing
        auto const elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        if (elapsed > 250.0)
            WriteLog("editor: gathering the ground took " + std::to_string(static_cast<int>(elapsed)) + " ms, for " + std::to_string(Out.size()) + " triangles; " + std::to_string(ground_held) + " are kept for the next time");
    }

    // ground kept for the road tools, which ask about the same places frame after frame. it's gathered a tile at a time:
    // gathering is what takes time, so a place asked about for the first time costs a single tile a frame instead of a stall
    struct ground_tile
    {
        std::unique_ptr<triangle_grid> grid;
        double gathered{0.0}; // when, on the clock of the user interface
        int used{0}; // frame it was last asked for
    };
    double constexpr ground_tile_size = 256.0;
    double constexpr ground_tile_lifetime = 30.0; // older tiles are gathered again when asked for, to catch up with the scenery
    std::size_t constexpr ground_tile_limit = 96;
    std::map<std::pair<int, int>, ground_tile> ground_tiles;
    int ground_tile_frame{-1}; // frame the last tile was gathered in without being told to

    // scenery file name without path and extension
    std::string scenery_stem()
    {
        std::string scenery = Global.SceneryFile;
        auto const slash = scenery.find_last_of("/\\");
        if (slash != std::string::npos)
            scenery = scenery.substr(slash + 1);
        auto const dot = scenery.find_last_of('.');
        if (dot != std::string::npos)
            scenery = scenery.substr(0, dot);
        return scenery.empty() ? "default" : scenery;
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
	// the area fill settings live in the node bank window, in the tab of the fill mode
	ui()->set_fill_options([this]() { render_area_fill(); });
	ui()->set_gizmo_options([this]() { render_gizmo_options(); });
	ui()->set_array_options([this]() { render_array(); });
	ui()->set_bend_options([this]() { render_bend(); });
	ui()->set_file_actions([this]() { save(); }, [this]() { export_scenery(); });
	ui()->set_new_scenery([this]() { m_newscenery_asked = true; });
	ui()->set_open_scenery([this]() { m_openscenery_asked = true; });
	ui()->set_menu_options([this]() {
		render_object_menu();
		render_track_menu();
		render_road_menu();
		render_map_menu();
	});
	m_roadtool.settings.normalize();
	road_select(nullptr);
	// the orthophoto is fitted onto the same ground as the area fill uses, terrain tile models included
	// the roads built in the editor aren't ground: the imagery goes under them
	m_orthophoto.ground_source([](glm::dvec2 const &Min, glm::dvec2 const &Max, std::vector<world_triangle> &Out) {
		gather_ground_triangles(Min, Max, true, 50.0f, Out, true);
	});
	editor_track::observe(this);
 }

editor_mode::~editor_mode()
{
	editor_track::observe(nullptr);
}

std::vector<double> editor_mode::ground_heights(std::vector<glm::dvec3> const &Points, bool const Fresh, std::vector<char> *Found)
{
    std::vector<double> heights;
    heights.reserve(Points.size());
    if (Found != nullptr)
        Found->assign(Points.size(), 0);
    for (auto const &point : Points)
        heights.push_back(point.y);
    if (Points.empty() || simulation::Region == nullptr)
        return heights;

    double const now = ImGui::GetTime();
    int const frame = ImGui::GetFrameCount();
    // surfaces much higher than the point are something it's under, rather than the ground
    double constexpr height_tolerance = 50.0;
    auto const terrains = active_terrains();
    for (std::size_t i = 0; i < Points.size(); ++i)
    {
        auto const &p = Points[i];
        std::pair<int, int> const key{static_cast<int>(std::floor(p.x / ground_tile_size)), static_cast<int>(std::floor(p.z / ground_tile_size))};
        auto &tile = ground_tiles[key];
        bool const missing = (tile.grid == nullptr);
        bool const stale = missing || now < tile.gathered || now - tile.gathered > ground_tile_lifetime;
        // told to wait the caller gets every tile it needs, otherwise what's there, with one tile gathered each frame
        if (stale && (Fresh ? missing : ground_tile_frame != frame))
        {
            glm::dvec2 const tilemin{key.first * ground_tile_size, key.second * ground_tile_size};
            glm::dvec2 const tilemax{tilemin + glm::dvec2{ground_tile_size}};
            std::vector<world_triangle> triangles;
            // the same ground as the area fill uses, terrain tile models included
            gather_ground_triangles(tilemin, tilemax, true, 50.0f, triangles, true);
            tile.grid = std::make_unique<triangle_grid>(std::move(triangles), tilemin, tilemax);
            tile.gathered = now;
            if (false == Fresh)
                ground_tile_frame = frame;
        }
        tile.used = frame;

        double const ceiling = p.y + height_tolerance;
        double y = 0.0;
        bool found = (tile.grid != nullptr) && tile.grid->height_at(p.x, p.z, ceiling, y);
        for (editor_terrain *terrain : terrains)
        {
            if (!terrain->contains(p.x, p.z))
                continue;
            double const h = terrain->height_at(p.x, p.z);
            if (h <= ceiling && (!found || h > y))
            {
                y = h;
                found = true;
            }
        }
        if (found)
            heights[i] = y;
        if (Found != nullptr)
            (*Found)[i] = found ? 1 : 0;
    }
    // the tiles left alone the longest go once there are too many
    while (ground_tiles.size() > ground_tile_limit)
    {
        auto oldest = ground_tiles.begin();
        for (auto tile = ground_tiles.begin(); tile != ground_tiles.end(); ++tile)
            if (tile->second.used < oldest->second.used)
                oldest = tile;
        if (oldest->second.used == frame)
            break;
        ground_tiles.erase(oldest);
    }
    return heights;
}

editor_ui *editor_mode::ui() const
{
    return static_cast<editor_ui *>(m_userinterface.get());
}

bool editor_mode::init()
{
    EditorSettings.load();
    EditorModelSets.load();
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

            // spacing is measured in world space: the mouse offset is camera-relative and the camera may move mid-stroke
            glm::dvec3 const newPos = Camera.Pos + clamp_mouse_offset_to_max(GfxRenderer->Mouse_Position());
            if (m_brush_has_last && glm::distance(newPos, oldPos) < ui()->getSpacing())
                return;

            // picked only when a placement is due, so a random pick from a set isn't drawn every tick
            const std::string *src = ui()->get_active_node_template();
            if (!src || src->starts_with(editor_includes::directive_mark)) // scenery templates are placed one at a time
                return;

            // NOTE: no name for what the brush puts in, as with thousands of nodes the names take their share of the time
            // the scenery needs to load. a node which needs one, to be referred to by an event, gets it in the node properties
            TAnimModel *cloned = simulation::State.create_model(*src, std::string{}, placement_on_ground(newPos));
            oldPos = newPos;
            m_brush_has_last = true;
            if (!cloned)
                return;

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
    snap.layer = node->layer();
    
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
        for (auto &copy : s.copies)
        {
            if (copy.model == node)
                copy.model = nullptr;
        }
    }

    for (auto &s : g_redo)
    {
        if (s.node_ptr == node)
            s.node_ptr = nullptr;
    }

    // deleted nodes also drop out of the "undo last fill" set
    m_fill_last.erase(std::remove(m_fill_last.begin(), m_fill_last.end(), node), m_fill_last.end());

    // an array no longer follows its settings once the model it was made of is gone
    if (m_array.base == node)
        m_array.base = nullptr;
}

void editor_mode::undo_last()
{
    if (m_history.empty())
        return;

    EditorSnapshot snap = m_history.back();
    m_history.pop_back();

    if (snap.action == EditorSnapshot::Action::Bend)
    {
        restore_bend(snap, true);
        g_redo.push_back(std::move(snap));
        return;
    }

    if (false == snap.sweeps.empty() || false == snap.sweeps_toggled.empty())
    {
        restore_sweeps(snap);
        g_redo.push_back(std::move(snap));
        return;
    }

    if (false == snap.instances.empty() || false == snap.directives.empty())
    {
        restore_includes(snap);
        if (snap.action != EditorSnapshot::Action::TrackEdit)
        {
            g_redo.push_back(std::move(snap));
            return;
        }
    }

    if (snap.instance != 0)
    {
        restore_include(snap);
        g_redo.push_back(std::move(snap));
        return;
    }

    if (snap.action == EditorSnapshot::Action::TrackEdit)
    {
        restore_track_snapshot(snap, g_redo, true);
        return;
    }

    if (snap.action == EditorSnapshot::Action::RoadEdit)
    {
        restore_road_snapshot(snap, g_redo, true);
        return;
    }

    if (snap.action == EditorSnapshot::Action::Array)
    {
        restore_array_snapshot(std::move(snap), g_redo, true);
        return;
    }

    if (snap.action == EditorSnapshot::Action::Delete)
    {
        // undo delete -> recreate model
        EditorSnapshot redoSnap;
        redoSnap.action = EditorSnapshot::Action::Delete;
        redoSnap.node_name = snap.node_name;
        redoSnap.serialized = snap.serialized;
        redoSnap.position = snap.position;
        redoSnap.node_ptr = nullptr;
        redoSnap.uuid = snap.uuid;
        redoSnap.layer = snap.layer;
        g_redo.push_back(std::move(redoSnap));

        TAnimModel *created = simulation::State.create_model(snap.serialized, snap.node_name, snap.position);
        if (created)
        {
            scene::Layers.move(created, snap.layer); // back to the layer it was deleted from, rather than the active one
            created->location(snap.position);
            created->Angles(snap.rotation);
            m_node = created;
            m_node->uuid = snap.uuid; // restore original UUID for better tracking (not strictly necessary) 
            add_to_hierarchy(created);
            // a node without a name can be found for the redo only this way
            g_redo.back().node_ptr = created;
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
    current.layer = target->layer();
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

    if (snap.action == EditorSnapshot::Action::Bend)
    {
        restore_bend(snap, false);
        m_history.push_back(std::move(snap));
        return;
    }

    if (false == snap.sweeps.empty() || false == snap.sweeps_toggled.empty())
    {
        restore_sweeps(snap);
        m_history.push_back(std::move(snap));
        return;
    }

    if (false == snap.instances.empty() || false == snap.directives.empty())
    {
        restore_includes(snap);
        if (snap.action != EditorSnapshot::Action::TrackEdit)
        {
            m_history.push_back(std::move(snap));
            return;
        }
    }

    if (snap.instance != 0)
    {
        restore_include(snap);
        m_history.push_back(std::move(snap));
        return;
    }

    if (snap.action == EditorSnapshot::Action::TrackEdit)
    {
        restore_track_snapshot(snap, m_history, false);
        return;
    }

    if (snap.action == EditorSnapshot::Action::RoadEdit)
    {
        restore_road_snapshot(snap, m_history, false);
        return;
    }

    if (snap.action == EditorSnapshot::Action::Array)
    {
        restore_array_snapshot(std::move(snap), m_history, false);
        return;
    }

    // handle delete redo (re-delete) separately
    if (snap.action == EditorSnapshot::Action::Delete)
    {
        EditorSnapshot hist;
        hist.action = snap.action;
        hist.node_name = snap.node_name;
        hist.serialized = snap.serialized;
        hist.position = snap.position;
        hist.uuid = snap.uuid;
        hist.layer = snap.layer;
        m_history.push_back(std::move(hist));

        // NOTE: located the way other steps do it, as the name alone doesn't lead to a node which has none
        scene::basic_node *target = find_node_by_any(snap.node_ptr, snap.uuid.to_string(), snap.node_name);
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
            scene::Layers.move(created, snap.layer);
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
    selftest_step();
    if (m_vehicle.leave)
    {
        m_vehicle.leave = false;
        Application.pop_mode();
        return true;
    }

    Timer::UpdateTimers(true);

    simulation::State.update_clocks();
    simulation::Environment.update();
    // bindings are matched with the objects by their locations, before anything gets moved
    if (false == m_bindings_loaded && simulation::is_ready)
        infra_load();

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

    // drop the selection if its layer was hidden or locked in the meantime
    if (m_node != nullptr && false == scene::Layers.editable(m_node))
    {
        m_node = nullptr;
        m_dragging = false;
        ui()->set_node(nullptr);
    }
    if (m_instance != 0 && false == scene::Layers.removable(m_instance))
    {
        select_include(0);
    }
    if (m_include.changed)
    {
        // a parameter of the selected include was changed in the node properties
        m_include.changed = false;
        apply_include();
    }
    if (false == m_include.active && false == m_gizmo_using)
    {
        m_include_gesture = false;
    }
    // models replaced after changes of includes are destroyed once the renderer can't have them among its pick candidates
    for (auto &retired : m_retired)
    {
        if (--retired.second <= 0)
        {
            simulation::Instances.purge(retired.first);
        }
    }
    m_retired.erase(std::remove_if(m_retired.begin(), m_retired.end(), [](std::pair<TAnimModel *, int> const &Retired) { return Retired.second <= 0; }), m_retired.end());

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

    // --- geoportal orthophoto: streamed around the camera, drawn beneath the other overlays ---
    m_orthophoto.update(Camera.Pos);
    draw_orthophoto();

    // --- ImGuizmo: in-viewport transform gizmo for the selected node ---
    render_gizmo();

    render_track_inspector();
    if (selected_track())
        draw_track_overlay();
    if (route_active())
        draw_route_overlay();
    if (ui()->mode() == nodebank_panel::TRACK && m_track_tab == track_tab::straights)
        draw_straights_overlay();
    if (ui()->mode() == nodebank_panel::TRACK)
    {
        if (m_track_box.active && false == ImGui::GetIO().MouseDown[0])
            finish_track_box();
        update_track_hover();
        point_drag_update();
        if (m_track_tab == track_tab::turntable)
            draw_turntable_overlay();
        if (m_track_tab == track_tab::signals)
        {
            signal_refresh();
            signal_move_update();
            draw_signal_overlay();
        }
        draw_parallel_preview();
        if (m_track_tab == track_tab::lineside)
        {
            draw_hekto_overlay();
            draw_fouling_overlay();
            draw_vehicle_marker();
        }
        render_track_context();
        update_build_tools();
        update_track_intent();
        draw_track_hover();
        draw_track_set();
        draw_track_spread();
        draw_build_overlay();
        draw_track_intent();
        speed_step();
        joints_step();
        draw_track_hints();
        draw_profile_overlay();
        draw_infra_overlay();
        draw_speed_overlay();
        draw_joints_overlay();
    }
    else
    {
        draw_sweep_overlay();
        m_sweep.open = false;
        bend_drag_update();
        draw_bend_overlay();
    }
    update_gauge();
    render_gauge_window();

    // roads: trajectories of the lanes, and the tools of the road window
    update_road_tool();
    draw_road_overlay();
    render_road_window();

    // --- array: the one made last is kept in line with its settings ---
    update_array();

    // --- area fill: outline overlay while the mode is active (its settings are drawn in the node bank window) ---
    if (ui()->mode() == nodebank_panel::FILL)
        draw_area_fill_outline();

    render_orthophoto_window();
    render_new_scenery_popup();
    render_open_scenery_popup();

    // --- ImGui: Editor Settings & History windows ---
    if(m_settings_open)
        render_settings();

    if(!m_change_history) return true;

    render_change_history();

    return true;
}

void editor_mode::render_settings()
{
    ImGui::Begin(STR_C("Editor Settings"), &m_settings_open, ImGuiWindowFlags_AlwaysAutoResize);

    if (ImGui::BeginTabBar("##editorsettings"))
    {
        if (ImGui::BeginTabItem(STR_C("General")))
        {
            ImGui::TextUnformatted(STR_C("Camera movement"));

            const char *schemes[] = {STR_C("WSAD (new)"), STR_C("Arrows (legacy)")};
            int current = EditorSettings.movement() == editorSettings::movement_scheme::legacy ? 1 : 0;
            if (ImGui::Combo("##movement_scheme", &current, schemes, IM_ARRAYSIZE(schemes)))
            {
                EditorSettings.movement(current == 1 ? editorSettings::movement_scheme::legacy
                                                     : editorSettings::movement_scheme::wsad);
                m_input.keyboard.apply_scheme();
                EditorSettings.save();
            }

            ImGui::Separator();
            ImGui::Checkbox(STR_C("Transform gizmo (ImGuizmo)"), &m_gizmo_enabled);
            ImGui::EndTabItem();
        }
        bool const terrainwanted = m_terrain_tab_wanted;
        m_terrain_tab_wanted = false;
        if (ImGui::BeginTabItem(STR_C("Terrain"), nullptr, terrainwanted ? ImGuiTabItemFlags_SetSelected : 0))
        {
            render_terrain_ui();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
}

void editor_mode::render_terrain_ui()
{
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt(STR_C("Grid cells"), &m_terrain_cells);
    m_terrain_cells = std::clamp(m_terrain_cells, 1, 512);
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat(STR_C("Cell size (m)"), &m_terrain_cellsize);
    if (m_terrain_cellsize < 0.1f)
        m_terrain_cellsize = 0.1f;
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat(STR_C("Base height (m)"), &m_terrain_baseheight);
    ImGui::SetNextItemWidth(200.0f);
    ImGui::InputText(STR_C("Texture (optional)"), m_terrain_texture, IM_ARRAYSIZE(m_terrain_texture));

    if (ImGui::Button(STR_C("Create flat terrain")))
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
    ImGui::InputInt(STR_C("Chunks / side"), &m_terrain_chunks);
    m_terrain_chunks = std::clamp(m_terrain_chunks, 1, 32);
    ImGui::SameLine();
    if (ImGui::Button(STR_C("Create chunked terrain")))
        create_chunked_terrain();
    ImGui::TextDisabled(STR_C("total %d x %d m, %d chunks"),
                        static_cast<int>(m_terrain_chunks * m_terrain_cells * m_terrain_cellsize),
                        static_cast<int>(m_terrain_chunks * m_terrain_cells * m_terrain_cellsize),
                        m_terrain_chunks * m_terrain_chunks);

    if (ImGui::Checkbox(STR_C("Chunk edit mode (LMB add neighbour / Shift = delete)"), &m_chunk_edit))
        if (m_chunk_edit)
            m_terrain_sculpt = false; // mutually exclusive with sculpting
    ImGui::Text(STR_C("Grid chunks: %zu"), m_grid_chunks.size());

    ImGui::Separator();
    ImGui::TextUnformatted(STR_C("Streaming (open world, follows camera)"));
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt(STR_C("Stream radius"), &m_stream_radius);
    m_stream_radius = std::clamp(m_stream_radius, 0, 16);
    ImGui::Checkbox(STR_C("Persist edits to disk (16-bit)"), &m_stream_persist);
    bool streaming = m_streamer.active();
    if (ImGui::Checkbox(STR_C("Stream terrain"), &streaming))
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
        ImGui::Text(STR_C("Resident chunks: %zu  (dir: %s)"), m_streamer.resident(), m_streamer.directory().c_str());
    }

    ImGui::Text(STR_C("Patches: %zu"), m_terrains.size());

    // capture: sample the selected model's geometry into an editable patch and remove the original
    if (dynamic_cast<TAnimModel *>(m_node) != nullptr)
    {
        if (ImGui::Button(STR_C("Capture selected model as terrain")))
            capture_terrain();
    }
    else
    {
        ImGui::TextDisabled(STR_C("Capture: select a model instance first"));
    }

    std::vector<editor_terrain *> const terrains = active_terrains();
    if (!terrains.empty())
    {
        ImGui::Separator();
        if (ImGui::Checkbox((std::string(m_terrain_brush_smooth ? STR_C("Sculpt mode (LMB smooth)") : STR_C("Sculpt mode (LMB raise / Shift = lower)")) + "###sculptmode").c_str(), &m_terrain_sculpt))
            if (m_terrain_sculpt)
                m_chunk_edit = false; // mutually exclusive with chunk editing
        int brush = m_terrain_brush_smooth ? 1 : 0;
        ImGui::RadioButton(STR_C("Raise / lower"), &brush, 0);
        ImGui::SameLine();
        ImGui::RadioButton(STR_C("Smooth"), &brush, 1);
        m_terrain_brush_smooth = (brush == 1);
        if (m_terrain_brush_smooth && ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", STR_C("Evens out the ground under the brush: bumps and sharp edges go, the shape stays.\nThe strength sets how fast"));
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat(STR_C("Brush radius"), &m_terrain_brush_radius);
        if (m_terrain_brush_radius < 0.5f)
            m_terrain_brush_radius = 0.5f;
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat(STR_C("Brush strength"), &m_terrain_brush_strength);

        // one-shot nudge of the most recent manual patch at its centre, handy for a quick test
        if (!m_terrains.empty())
        {
            auto &terrain = m_terrains.back();
            glm::dvec3 const c = terrain->centre();
            if (ImGui::Button(STR_C("Raise centre")))
                terrain->sculpt(c.x, c.z, m_terrain_brush_radius, m_terrain_brush_strength);
            ImGui::SameLine();
            if (ImGui::Button(STR_C("Lower centre")))
                terrain->sculpt(c.x, c.z, m_terrain_brush_radius, -m_terrain_brush_strength);
        }

        ImGui::Separator();
        ImGui::TextUnformatted(STR_C("Optimize (mesh simplification, all patches)"));
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat(STR_C("Flatness tol (m)"), &m_terrain_simplify_error);
        if (m_terrain_simplify_error < 0.01f)
            m_terrain_simplify_error = 0.01f;
        ImGui::Checkbox(STR_C("Auto-optimize after sculpt"), &m_terrain_auto_optimize);
        if (ImGui::Button(STR_C("Optimize all")))
            for (editor_terrain *t : terrains)
                t->optimize(m_terrain_simplify_error);
        ImGui::SameLine();
        if (ImGui::Button(STR_C("Full-res all")))
            for (editor_terrain *t : terrains)
                t->unoptimize();

        std::size_t tris = 0, full = 0;
        for (editor_terrain *t : terrains)
        {
            tris += t->triangles();
            full += t->full_triangles();
        }
        ImGui::Text(STR_C("Triangles: %zu / %zu"), tris, full);
    }
}

void editor_mode::load_orthophoto_settings()
{
    std::string const scenery = scenery_stem();
    if (scenery == m_orthophoto_scenery)
        return;
    m_orthophoto_scenery = scenery;

    auto const &stored = EditorSettings.orthophoto();
    editor_orthophoto::config config = m_orthophoto.settings();
    config.radius = stored.radius;
    config.height = stored.height;
    config.opacity = stored.opacity;
    config.year = stored.year;
    config.hires = stored.hires;
    config.drape = stored.drape;
    config.lift = stored.lift;
    config.in_scene = stored.in_scene;
    config.north = 0.0;
    config.east = 0.0;
    m_georeference_read = false;
    read_georeference();
    if (false == std::as_const(EditorSettings).orthophoto_origin(scenery, config.north, config.east) && m_georeference)
    {
        config.north = m_georeference_origin.x;
        config.east = m_georeference_origin.y;
    }
    m_orthophoto.settings(config);
    m_orthophoto_origin_edit = {config.north, config.east};
}

void editor_mode::save_orthophoto_settings()
{
    editor_orthophoto::config const &config = m_orthophoto.settings();
    auto &stored = EditorSettings.orthophoto();
    stored.radius = config.radius;
    stored.height = config.height;
    stored.opacity = config.opacity;
    stored.year = config.year;
    stored.hires = config.hires;
    stored.drape = config.drape;
    stored.lift = config.lift;
    stored.in_scene = config.in_scene;
    if (config.north != 0.0 || config.east != 0.0)
        EditorSettings.orthophoto_origin(m_orthophoto_scenery, config.north, config.east);
    EditorSettings.save();
}

void editor_mode::render_orthophoto_ui()
{
    load_orthophoto_settings();

    editor_orthophoto::config config = m_orthophoto.settings();
    bool changed = false; // apply to the layer
    bool persist = false; // and store in the editor settings

    bool enabled = m_orthophoto.enabled();
    if (ImGui::Checkbox(STR_C("Show orthophoto (geoportal.gov.pl)"), &enabled))
        m_orthophoto.enabled(enabled);

    ImGui::Separator();
    ImGui::TextUnformatted(STR_C("Scenery origin (0,0,0) in PUWG 1992 / EPSG:2180"));
    // geodetic convention, as displayed by geoportal.gov.pl: X grows north, Y grows east.
    // applied once editing ends, so half-typed numbers don't trigger downloads
    ImGui::SetNextItemWidth(160.0f);
    ImGui::InputDouble(STR_C("X (northing, m)"), &m_orthophoto_origin_edit.x, 0.0, 0.0, "%.2f");
    bool const northedited = ImGui::IsItemDeactivatedAfterEdit();
    ImGui::SetNextItemWidth(160.0f);
    ImGui::InputDouble(STR_C("Y (easting, m)"), &m_orthophoto_origin_edit.y, 0.0, 0.0, "%.2f");
    bool const eastedited = ImGui::IsItemDeactivatedAfterEdit();
    if (northedited || eastedited)
    {
        config.north = m_orthophoto_origin_edit.x;
        config.east = m_orthophoto_origin_edit.y;
        changed = persist = true;
    }
    if (m_georeference)
    {
        ImGui::TextDisabled("%s", m_georeference_line.c_str());
        ImGui::TextDisabled(STR_C("in the scenery file: Y (easting) %.0f m, X (northing) %.0f m"), m_georeference_origin.y, m_georeference_origin.x);
        if (config.north != m_georeference_origin.x || config.east != m_georeference_origin.y)
        {
            ImGui::SameLine();
            if (ImGui::SmallButton(STR_C("Use it")))
            {
                config.north = m_orthophoto_origin_edit.x = m_georeference_origin.x;
                config.east = m_orthophoto_origin_edit.y = m_georeference_origin.y;
                changed = persist = true;
            }
        }
    }
    if (config.north == 0.0 && config.east == 0.0)
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), STR_C("Enter the origin of this scenery first"));
    else
        ImGui::TextDisabled(STR_C("Camera at X %.1f  Y %.1f"), config.north + Camera.Pos.z, config.east - Camera.Pos.x);

    ImGui::Separator();
    ImGui::SetNextItemWidth(160.0f);
    changed |= ImGui::SliderInt(STR_C("Distance (tiles)"), &config.radius, 0, editor_orthophoto::max_radius);
    persist |= ImGui::IsItemDeactivatedAfterEdit();
    ImGui::SameLine();
    int const span = static_cast<int>((2 * config.radius + 1) * editor_orthophoto::tile_size);
    ImGui::TextDisabled("%d x %d m", span, span);

    // ImGui 1.73 has no public disabled state yet, the internal item flag does the job
    auto const begin_disabled = [](bool Disabled) {
        if (!Disabled)
            return;
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    };
    auto const end_disabled = [](bool Disabled) {
        if (!Disabled)
            return;
        ImGui::PopStyleVar();
        ImGui::PopItemFlag();
    };

    if (ImGui::Checkbox(STR_C("Fit to terrain"), &config.drape))
        changed = persist = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(STR_C("Lays the imagery onto the ground geometry (terrain shapes, terrain tile models, editor terrain).\n"
                          "Places without any ground use the height below."));
    if (config.drape)
    {
        ImGui::SameLine();
        if (ImGui::Button(STR_C("Refit")))
        {
            // the ground was edited: what's kept of it has to go as well
            ground_tiles.clear();
            forget_ground_shapes();
            m_orthophoto.refit();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(STR_C("Samples the ground again, after it was edited"));
    }

    begin_disabled(!config.drape);
    ImGui::SetNextItemWidth(160.0f);
    changed |= ImGui::DragFloat(STR_C("Lift above ground (m)"), &config.lift, 0.01f, 0.0f, 50.0f, "%.2f");
    persist |= ImGui::IsItemDeactivatedAfterEdit();
    end_disabled(!config.drape);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(STR_C("Raises the fitted imagery above the ground. Keep it below the objects you want to see:\n"
                          "in the scene everything lower than this ends up under the imagery."));

    begin_disabled(config.drape);
    ImGui::SetNextItemWidth(160.0f);
    changed |= ImGui::DragFloat(STR_C("Height (m)"), &config.height, 0.1f, -1000.0f, 3000.0f, "%.2f");
    persist |= ImGui::IsItemDeactivatedAfterEdit();
    end_disabled(config.drape);

    if (ImGui::Checkbox(STR_C("Covered by objects and terrain"), &config.in_scene))
        changed = persist = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(STR_C("Draws the imagery as part of the scene, so tracks, models and terrain in front of it hide it.\n"
                          "It is lit like the scene. Fully opaque, mouse placement lands on it as on any surface;\n"
                          "translucent, it doesn't catch the mouse.\n"
                          "Not available with the Better Renderer, which doesn't pick up geometry added in the editor."));

    ImGui::SetNextItemWidth(160.0f);
    changed |= ImGui::SliderFloat(STR_C("Opacity"), &config.opacity, 0.0f, 1.0f, "%.2f");
    persist |= ImGui::IsItemDeactivatedAfterEdit();
    if (config.in_scene && ImGui::IsItemHovered())
        ImGui::SetTooltip(STR_C("In the scene the opacity is part of the textures: the tiles are read again from the cache\n"
                          "shortly after the slider stops, which takes a moment."));

    // newest imagery, or the newest imagery taken up to the end of the selected year
    int const thisyear = static_cast<int>(std::chrono::year_month_day{std::chrono::floor<std::chrono::days>(std::chrono::system_clock::now())}.year());
    std::string const yearlabel = config.year ? std::to_string(config.year) : std::string(STR_C("Newest"));
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("Photo year", yearlabel.c_str()))
    {
        if (ImGui::Selectable(STR_C("Newest"), config.year == 0))
        {
            config.year = 0;
            changed = persist = true;
        }
        for (int year = thisyear; year >= editor_orthophoto::oldest_year; --year)
        {
            if (ImGui::Selectable(std::to_string(year).c_str(), config.year == year))
            {
                config.year = year;
                changed = persist = true;
            }
        }
        ImGui::EndCombo();
    }

    if (ImGui::Checkbox(STR_C("4K tiles where available"), &config.hires))
        changed = persist = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(STR_C("4096 px tiles (~6 cm/px) from the high-resolution orthophoto, for the tiles around the camera.\n"
                          "Areas it doesn't cover use the standard imagery. Each 4K tile takes ~85 MB of video memory."));

    if (changed)
        m_orthophoto.settings(config);
    if (persist)
        save_orthophoto_settings();

    ImGui::Separator();
    auto const stats = m_orthophoto.stats();
    ImGui::Text(STR_C("Tiles: %d shown, %d loading, %d failed"), stats.resident, stats.loading, stats.failed);
    if (stats.failed > 0)
    {
        ImGui::SameLine();
        if (ImGui::Button(STR_C("Retry")))
            m_orthophoto.retry_failed();
    }
    if (!editor_orthophoto::can_download())
        ImGui::TextDisabled(STR_C("This build has no HTTP client, only cached tiles are shown"));
    ImGui::TextDisabled(STR_C("Cache: %s"), editor_orthophoto::cache_directory().c_str());
    if (ImGui::Button(STR_C("Clear cache")))
        m_orthophoto.clear_cache();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(STR_C("Deletes downloaded tiles from disk. They are fetched again around the camera."));
    ImGui::TextDisabled(STR_C("Imagery: GUGiK, geoportal.gov.pl"));
}

void editor_mode::draw_orthophoto()
{
    if (!m_orthophoto.enabled())
        return;
    // the camera was just published to the renderer (update_camera), so build the view from it rather than
    // from the renderer's previous frame; camera-relative rotation, same projection as the other overlays
    glm::dmat4 viewmatrix{1.0};
    Camera.SetMatrix(viewmatrix);
    glm::mat4 const view = glm::mat4(glm::mat3(viewmatrix));
    ImGuiIO const &io = ImGui::GetIO();
    float const aspect = io.DisplaySize.y > 0.0f ? io.DisplaySize.x / io.DisplaySize.y : 1.0f;
    m_orthophoto.draw(editor_mode::projection_matrix(aspect) * view, Camera.Pos, io.DisplaySize.x, io.DisplaySize.y);
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

namespace
{
    std::uint64_t geometry_key(gfx::geometry_handle const &Geometry)
    {
        return (static_cast<std::uint64_t>(Geometry.bank) << 32) | Geometry.chunk;
    }

    // the shapes of the scenery which can be ground: the ones the editor makes itself (its terrain, roads, imagery) left out
    template <typename Visitor> void visit_scenery_shapes(std::set<std::uint64_t> const &Own, Visitor const &Visit)
    {
        auto const scenery = [&](scene::shape_node const &Shape) {
            auto const &data = Shape.data();
            return Own.count(geometry_key(data.geometry)) == 0 && false == editor_orthophoto::owns(data.geometry) && false == owned_shapes::owns(data.geometry);
        };
        for (std::size_t i = 0; i < sq(scene::EU07_REGIONSIDESECTIONCOUNT); ++i)
        {
            auto *section = simulation::Region->get_section(i);
            if (section == nullptr)
                continue;
            for (auto &shape : section->m_shapes)
                if (scenery(shape))
                    Visit(shape);
            for (auto &cell : section->m_cells)
                for (auto &shape : cell.m_shapesopaque)
                    if (scenery(shape))
                        Visit(shape);
        }
    }

    // takes the shapes of specified materials out of the scene
    std::size_t drop_scenery_shapes(std::set<std::uint64_t> const &Own, std::set<material_handle> const &Materials)
    {
        std::size_t dropped = 0;
        auto const dropping = [&](scene::shape_node const &Shape) {
            auto const &data = Shape.data();
            bool const drop = Materials.count(data.material) != 0 && Own.count(geometry_key(data.geometry)) == 0 && false == editor_orthophoto::owns(data.geometry) &&
                              false == owned_shapes::owns(data.geometry);
            dropped += drop ? 1 : 0;
            return drop;
        };
        for (std::size_t i = 0; i < sq(scene::EU07_REGIONSIDESECTIONCOUNT); ++i)
        {
            auto *section = simulation::Region->get_section(i);
            if (section == nullptr)
                continue;
            section->m_shapes.erase(std::remove_if(section->m_shapes.begin(), section->m_shapes.end(), dropping), section->m_shapes.end());
            for (auto &cell : section->m_cells)
                cell.m_shapesopaque.erase(std::remove_if(cell.m_shapesopaque.begin(), cell.m_shapesopaque.end(), dropping), cell.m_shapesopaque.end());
        }
        return dropped;
    }
} // namespace

std::map<material_handle, std::size_t> editor_mode::ground_materials()
{
    std::map<material_handle, std::size_t> result;
    if (simulation::Region == nullptr)
        return result;
    std::set<std::uint64_t> own;
    for (auto *terrain : active_terrains())
        own.insert(geometry_key(terrain->geometry()));
    glm::dvec2 const everywhere{std::numeric_limits<double>::max() * 0.5};
    visit_scenery_shapes(own, [&](scene::shape_node const &Shape) {
        std::vector<world_triangle> triangles;
        gather_shape_triangles(Shape, -everywhere, everywhere, triangles, true);
        if (false == triangles.empty())
            result[Shape.data().material] += triangles.size();
    });
    return result;
}

std::string editor_mode::convert_ground(std::set<material_handle> const &Materials)
{
    if (simulation::Region == nullptr || Materials.empty())
        return {};
    bool const streaming = m_streamer.active();
    double const size = streaming ? m_streamer.cells() * m_streamer.cellsize() : chunk_grid_size();
    int const cells = streaming ? m_streamer.cells() : std::clamp(m_terrain_cells, 1, 256);
    double const cellsize = size / cells;
    if (size <= 0.0)
        return {};

    std::set<std::uint64_t> own;
    for (auto *terrain : active_terrains())
        own.insert(geometry_key(terrain->geometry()));
    std::vector<world_triangle> triangles;
    std::vector<material_handle> materials;
    glm::dvec2 const everywhere{std::numeric_limits<double>::max() * 0.5};
    visit_scenery_shapes(own, [&](scene::shape_node const &Shape) {
        if (Materials.count(Shape.data().material) == 0)
            return;
        gather_shape_triangles(Shape, -everywhere, everywhere, triangles, true);
        materials.resize(triangles.size(), Shape.data().material);
    });
    if (triangles.empty())
        return STR_C("The chosen materials have no triangles");

    // the triangles reaching each chunk
    std::map<std::pair<int, int>, std::vector<std::uint32_t>> buckets;
    for (std::uint32_t i = 0; i < triangles.size(); ++i)
    {
        auto const &t = triangles[i];
        int const x0 = static_cast<int>(std::floor(std::min({t[0].x, t[1].x, t[2].x}) / size));
        int const x1 = static_cast<int>(std::floor(std::max({t[0].x, t[1].x, t[2].x}) / size));
        int const z0 = static_cast<int>(std::floor(std::min({t[0].z, t[1].z, t[2].z}) / size));
        int const z1 = static_cast<int>(std::floor(std::max({t[0].z, t[1].z, t[2].z}) / size));
        for (int x = x0; x <= x1; ++x)
            for (int z = z0; z <= z1; ++z)
                buckets[{x, z}].push_back(i);
    }

    // the terrain of the editor has a single texture, saved with it: the material which covers the most
    std::map<material_handle, double> coverage;
    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        auto const &t = triangles[i];
        coverage[materials[i]] += 0.5 * std::abs((t[1].x - t[0].x) * (t[2].z - t[0].z) - (t[2].x - t[0].x) * (t[1].z - t[0].z));
    }
    auto const dominant = std::max_element(coverage.begin(), coverage.end(), [](auto const &A, auto const &B) { return A.second < B.second; })->first;
    if (auto const *material = GfxRenderer->Material(dominant))
    {
        auto const name = material->GetName();
        std::snprintf(m_terrain_texture, sizeof(m_terrain_texture), "%s", name.c_str());
    }
    std::string const texture(m_terrain_texture);
    // a whole scenery of the full grid would be a lot to draw
    m_terrain_auto_optimize = true;
    if (streaming)
    {
        m_streamer.simplify(true, m_terrain_simplify_error);
        m_streamer.configure(m_streamer.cells(), m_streamer.cellsize(), m_streamer.radius(), m_terrain_baseheight, texture);
    }

    // the old ground goes first, so what's made in its place isn't taken for it
    auto const dropped = drop_scenery_shapes(own, Materials);
    std::size_t made = 0, skipped = 0;
    std::size_t const side = static_cast<std::size_t>(cells) + 1;
    for (auto const &bucket : buckets)
    {
        auto const &key = bucket.first;
        glm::dvec2 const low{key.first * size, key.second * size};
        glm::dvec2 const middle{low + glm::dvec2{size * 0.5}};
        if (streaming ? m_streamer.terrain_at(middle.x, middle.y) != nullptr : m_grid_chunks.count(key) > 0)
        {
            ++skipped;
            continue;
        }
        std::vector<world_triangle> local;
        for (auto const i : bucket.second)
            local.push_back(triangles[i]);
        triangle_grid const ground(std::move(local), low - glm::dvec2{1.0}, low + glm::dvec2{size + 1.0});
        std::vector<double> heights(side * side, 0.0);
        std::vector<char> found(side * side, 0);
        std::vector<std::size_t> known;
        for (std::size_t iz = 0; iz < side; ++iz)
            for (std::size_t ix = 0; ix < side; ++ix)
            {
                auto const index = iz * side + ix;
                if (ground.height_at(low.x + ix * cellsize, low.y + iz * cellsize, std::numeric_limits<double>::max(), heights[index]))
                {
                    found[index] = 1;
                    known.push_back(index);
                }
            }
        if (known.empty())
            continue;
        // at the edge of the ground the chunk is carried on level with the nearest point of it
        if (known.size() < heights.size())
            for (std::size_t index = 0; index < heights.size(); ++index)
            {
                if (found[index] != 0)
                    continue;
                double best = std::numeric_limits<double>::max();
                for (auto const other : known)
                {
                    double const dx = static_cast<double>(index % side) - static_cast<double>(other % side);
                    double const dz = static_cast<double>(index / side) - static_cast<double>(other / side);
                    if (dx * dx + dz * dz < best)
                    {
                        best = dx * dx + dz * dz;
                        heights[index] = heights[other];
                    }
                }
            }
        auto const sampler = [&](double X, double Z, double &OutY) {
            auto const ix = static_cast<std::size_t>(std::clamp<long>(std::lround((X - low.x) / cellsize), 0, cells));
            auto const iz = static_cast<std::size_t>(std::clamp<long>(std::lround((Z - low.y) / cellsize), 0, cells));
            OutY = heights[iz * side + ix];
            return true;
        };
        if (streaming)
        {
            m_streamer.add_chunk(key.first, key.second);
            if (auto *terrain = m_streamer.terrain_at(middle.x, middle.y))
            {
                terrain->reshape([&](double X, double Z, float &Height) {
                    double y;
                    sampler(X, Z, y);
                    Height = static_cast<float>(y);
                    return true;
                });
                ++made;
            }
        }
        else
        {
            auto chunk = std::make_unique<editor_terrain>();
            if (chunk->create(glm::dvec3{middle.x, m_terrain_baseheight, middle.y}, cells, static_cast<float>(cellsize), texture, sampler))
            {
                // the full grid everywhere would be a lot to draw; the simplified one keeps it exact where it matters
                chunk->optimize(m_terrain_simplify_error);
                m_grid_chunks[key] = std::move(chunk);
                ++made;
            }
        }
    }
    ground_tiles.clear();
    forget_ground_shapes();

    auto const erased = scene::Layers.erase_shapes(Materials);
    auto summary = format(STR_C("Ground converted: %zu chunks of editor terrain made, %zu shapes taken out of the scene, %zu triangle nodes go from the scenery files on save"), made, dropped, erased.first);
    if (skipped > 0)
        summary += format(STR_C(", %zu chunks kept as there was editor terrain already"), skipped);
    if (erased.second > 0)
        summary += format(STR_C(", %zu nodes stay as their files can't be rewritten"), erased.second);
    WriteLog("Editor: " + summary, logtype::generic);
    return summary;
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
    commit_terrain();

    // export scenery; the exported .scm now carries an `editorterrain` directive (streamer is active)
    export_scenery();
    WriteLog("Editor: saved scene + terrain", logtype::generic);
}

void editor_mode::commit_terrain()
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
}

bool editor_mode::save()
{
    if (scene::Layers.empty())
    {
        // without the sources tracked during the load there's no telling where the changes belong
        ui()->set_status(STR("The scenery wasn't opened for editing. Start the simulator with: -edit <scenery file>"), true);
        return false;
    }

    // terrain made in the editor is kept in files of its own, the scenery only needs the directive which loads it
    std::vector<std::string> rootstatements;
    if (m_streamer.active() || false == m_grid_chunks.empty())
    {
        // the chunks go over to the streamer, what was kept to take back the shaping refers to the ones which are gone
        m_profile.earthworks.undo.clear();
        commit_terrain();
        if (false == scene::Layers.terrain_directive())
        {
            rootstatements.emplace_back(
                "editorterrain " + m_streamer.directory() + ' ' + std::to_string(m_streamer.cells()) + ' ' + to_string(m_streamer.cellsize(), 3) + ' '
                + std::to_string(m_streamer.radius()) + (m_streamer.texture().empty() ? "" : ' ' + m_streamer.texture()) + " endeditorterrain");
        }
    }

    if (m_profile.changed)
        profile_store();
    infra_store();
    if (false == m_saved_camera.has_value() || glm::distance(m_saved_camera->first, glm::dvec3{Camera.Pos}) > 0.5 || glm::distance(m_saved_camera->second, glm::vec3{Camera.Angle}) > 0.01f)
    {
        m_saved_camera = std::make_pair(glm::dvec3{Camera.Pos}, glm::vec3{Camera.Angle});
        scene::Layers.mark(scene::layer_handle{1}, "//$c", {format("%.3f %.3f %.3f %.5f %.5f", Camera.Pos.x, Camera.Pos.y, Camera.Pos.z, Camera.Angle.x, Camera.Angle.y)});
    }
    auto const terrainadded{false == rootstatements.empty()};
    auto const controls{turntable_controls(rootstatements)};
    std::vector<std::string> trailing;
    starter_trainset(trailing);
    auto const result = scene::Layers.save(rootstatements, trailing);
    if (result.success && controls)
        m_turntable_included = true;
    if (result.success && trailing.size() == 3)
    {
        std::istringstream header{trailing.front()};
        std::string keyword, timetable, track;
        double offset{0.0};
        header >> keyword >> timetable >> track >> offset;
        if (auto *path{simulation::Paths.find(track)}; path != nullptr && simulation::Vehicles.find("wmb10-6457") == nullptr)
            simulation::State.insert_trainset("wmb10-6457", path, offset, trailing[1] + "\n");
    }
    if (result.success && terrainadded)
    {
        scene::Layers.terrain_directive(true);
    }
    ui()->set_status(result.message, false == result.success);
    WriteLog("Editor: " + result.message, logtype::generic);
    return result.success;
}

void editor_mode::toggle_include(scene::instance_handle const Instance)
{
    if (false == scene::Layers.tracked(Instance))
        return;
    auto const &included = scene::Layers.instance(Instance);
    if (included.dead)
    {
        // its directive is gone from the scenery file, and the place it was at with it
        ui()->set_status("Include of \"" + *included.file + "\" was removed from the saved scenery, this can't be taken back.", true);
        return;
    }
    scene::Layers.removed(Instance, false == included.removed);
    ui()->set_status("Include of \"" + *included.file + "\" " + (included.removed ? "removed." : "restored."));
}

void editor_mode::restore_include(EditorSnapshot &Snapshot)
{
    if (Snapshot.serialized.empty())
    {
        // the include was placed or removed; either is taken back by flipping its removal
        toggle_include(Snapshot.instance);
        select_include(0);
        return;
    }
    // parameters of the include were changed. the snapshot holds the directive to go back to, and leaves with the current one
    auto current = scene::Layers.directive(Snapshot.instance);
    if (current.empty() || scene::Layers.instance(Snapshot.instance).dead)
    {
        ui()->set_status("Include of \"" + Snapshot.node_name + "\" is gone from the scenery, the change can't be taken back.", true);
        return;
    }
    set_include_directive(Snapshot.instance, Snapshot.serialized);
    Snapshot.serialized = std::move(current);
    select_include(scene::Layers.removable(Snapshot.instance) ? Snapshot.instance : 0);
}

void editor_mode::restore_includes(EditorSnapshot &Snapshot)
{
    std::size_t gone{0};
    for (auto const instance : Snapshot.instances)
    {
        if (false == scene::Layers.tracked(instance) || scene::Layers.instance(instance).dead)
        {
            ++gone;
            continue;
        }
        scene::Layers.removed(instance, false == scene::Layers.instance(instance).removed);
    }
    for (auto &entry : Snapshot.directives)
    {
        auto current = scene::Layers.directive(entry.first);
        if (current.empty() || scene::Layers.instance(entry.first).dead)
        {
            ++gone;
            continue;
        }
        set_include_directive(entry.first, entry.second);
        entry.second = std::move(current);
    }
    select_include(0);
    ui()->set_status(gone > 0 ? std::to_string(gone) + " of the includes are gone from the saved scenery, they can't be taken back." : std::to_string(Snapshot.instances.size() + Snapshot.directives.size()) + " includes restored.", gone > 0);
}

void editor_mode::select_include(scene::instance_handle const Instance)
{
    m_instance = (scene::Layers.tracked(Instance) ? Instance : 0);
    m_include = include_selection{};
    m_include_gesture = false;
    if (m_instance == 0)
    {
        ui()->set_include(nullptr);
        return;
    }
    auto const &included = scene::Layers.instance(m_instance);
    m_include.instance = m_instance;
    m_include.target = *included.file;
    // NOTE: a template without a usable description is still presented, with its parameters unnamed
    std::string error;
    editor_includes::load(*included.file, m_include.info, error);
    if (false == editor_includes::parse_directive(scene::Layers.directive(m_instance), m_include.target, m_include.values))
    {
        m_include.values.clear();
        m_include.issue = "The directive of the include can't be read from the scenery file, its parameters can't be changed.";
    }
    // with a rotation or a scale in effect the parameters don't translate to a location in the scene in a single way,
    // see place_include()
    m_include.placement = (included.context.rotation == glm::vec3{0.f} && included.context.scale == glm::vec3{1.f});
    ui()->set_include(&m_include);
}

void editor_mode::set_include_directive(scene::instance_handle const Instance, std::string const &Directive)
{
    if (false == scene::Layers.modify(Instance, Directive))
        return;
    std::vector<TAnimModel *> retired;
    simulation::State.rebuild_include(Instance, retired);
    // the pick of the renderer is resolved a few frames after it's requested, and can still deliver one of these
    for (auto *model : retired)
        m_retired.emplace_back(model, 120);
}

void editor_mode::apply_include()
{
    if (m_instance == 0 || false == m_include.issue.empty())
        return;
    for (auto const &value : m_include.values)
    {
        if (value == "end" || value.find_first_of("\"\r\n") != std::string::npos)
        {
            // either would cut the directive short
            ui()->set_status("A parameter of an include can't be \"end\", or have a quotation mark in it.", true);
            select_include(m_instance); // back to the values of the directive
            return;
        }
    }
    auto before = scene::Layers.directive(m_instance);
    auto const directive = editor_includes::compose_directive(m_include.target, m_include.values);
    if (directive == before)
        return;
    std::string read_before, read_after;
    if (false == m_include.info.signal_read.empty())
    {
        std::string file;
        std::vector<std::string> previous;
        if (editor_includes::parse_directive(before, file, previous))
        {
            read_before = editor_includes::substitute(m_include.info.signal_read, previous);
            read_after = editor_includes::substitute(m_include.info.signal_read, m_include.values);
        }
    }
    if (false == m_include_gesture)
    {
        // a single undo step for the whole drag
        EditorSnapshot snap;
        snap.instance = m_instance;
        snap.node_name = *scene::Layers.instance(m_instance).file;
        snap.serialized = std::move(before);
        if (read_before != read_after)
            signal_renamed(read_before, read_after, snap);
        m_history.push_back(std::move(snap));
        g_redo.clear();
        m_include_gesture = true;
    }
    else if (read_before != read_after && false == m_history.empty())
        signal_renamed(read_before, read_after, m_history.back());
    set_include_directive(m_instance, directive);
}

void editor_mode::place_include(std::string const &File, int const RotationMode, float const FixedRotation)
{
    if (scene::Layers.empty())
    {
        // the directive has to go to a scenery file, and without the sources tracked there's no telling which
        ui()->set_status("Scenery templates can be placed only in a scenery opened for editing (-edit).", true);
        WriteLog("Editor: scenery templates can be placed only in a scenery opened for editing (-edit)", logtype::generic);
        return;
    }
    include_info info;
    std::string issue;
    auto const parameters = editor_includes::parameter_count(File);
    if (false == editor_includes::load(File, info, issue) || false == editor_includes::complete(info, parameters, &issue))
    {
        ui()->set_status("Template \"" + File + "\" can't be placed: " + issue, true);
        return;
    }

    // the directive is added to the file of the active layer, and has to make sense with the placement in effect where it goes
    auto const layer = scene::Layers.resolve(scene::Layers.active());
    if (std::string reason; false == scene::Layers.accepts(layer, &reason))
    {
        ui()->set_status("Template \"" + File + "\" can't be placed: " + (reason.empty() ? "there's no active layer to put it in" : "the active layer can't take it, " + reason), true);
        return;
    }
    auto const context = scene::Layers.layer(layer).context_insert();
    if (context.rotation != glm::vec3{0.f} || context.scale != glm::vec3{1.f})
    {
        // a template can pass its parameters to an origin directive, which isn't affected by these, or straight to
        // the nodes, which are. there's no single set of parameter values which puts both at the cursor
        ui()->set_status("Template \"" + File + "\" can't be placed: the file of the active layer would receive it with a rotation or a scale in effect. Pick another layer.", true);
        return;
    }
    glm::dvec3 const location = placement_on_ground(Camera.Pos + clamp_mouse_offset_to_max(GfxRenderer->Mouse_Position()));
    std::optional<float> yaw;
    if (RotationMode == functions_panel::RANDOM)
        yaw = static_cast<float>(LocalRandom(0.0, 360.0));
    else if (RotationMode == functions_panel::FIXED)
        yaw = FixedRotation;

    auto const directive = editor_includes::directive(File, info, parameters, context.to_local(location), yaw);
    auto const instance = scene::Layers.place(layer, File, directive);
    if (instance == 0)
    {
        ui()->set_status("Template \"" + File + "\" can't be placed in the active layer.", true);
        return;
    }
    // show the models of the template right away. the rest of it comes into play when the saved scenery is loaded
    auto const preview = simulation::State.preview_include(directive, context, layer, instance);

    EditorSnapshot snap;
    snap.instance = instance;
    snap.node_name = File;
    m_history.push_back(std::move(snap));
    g_redo.clear();
    select_include(instance);

    ui()->set_status(
        "\"" + info.name + "\" placed in layer \"" + scene::Layers.layer(layer).name + "\": " + std::to_string(preview.first) + " model(s) shown"
        + (preview.second > 0 ? ", " + std::to_string(preview.second) + " other statement(s) of the template take effect once the scenery is saved and loaded again." : "."));
}

void editor_mode::export_scenery()
{
    // hiding a layer clears the visibility flag of its nodes, and the flag is a part of exported node data.
    // show the hidden layers for the duration of the export, so the files get the visibility defined in the scenery
    auto const hiddenlayers = scene::Layers.hidden();
    for (auto const layer : hiddenlayers)
        scene::Layers.visible(layer, true);

    simulation::State.export_as_text(Global.SceneryFile);

    for (auto const layer : hiddenlayers)
        scene::Layers.visible(layer, false);
}

void editor_mode::handle_terrain_sculpt(double Deltatime)
{
    // world point under the cursor (Mouse_Position is camera-relative, like the brush placement uses)
    glm::dvec3 const world = Camera.Pos + GfxRenderer->Mouse_Position();
    // only sculpt when the cursor is actually over terrain (avoids editing on a stale depth read)
    if (terrain_at(world.x, world.z) == nullptr)
        return;

    double const rate = m_terrain_brush_strength * Deltatime; // metres applied this frame
    if (m_terrain_brush_smooth)
    {
        // every patch the brush touches is smoothed from the same ground, so the edges they share stay together
        auto const terrains = active_terrains();
        double const reach = m_terrain_brush_radius + 2.0 * m_terrain_cellsize + 1.0;
        std::vector<std::pair<editor_terrain *, std::vector<float>>> before;
        for (editor_terrain *terrain : terrains)
        {
            auto const centre = terrain->centre();
            double const half = terrain->extent() * 0.5;
            if (std::abs(centre.x - world.x) <= half + reach && std::abs(centre.z - world.z) <= half + reach)
                before.emplace_back(terrain, terrain->heights());
        }
        auto const ground = [&](double X, double Z, double Fallback) {
            for (auto const &entry : before)
                if (entry.first->contains(X, Z))
                    return entry.first->height_in(entry.second, X, Z);
            return Fallback;
        };
        double const amount = std::min(1.0, rate * 0.5);
        for (auto const &entry : before)
            entry.first->smooth(world.x, world.z, m_terrain_brush_radius, amount, ground);
    }
    else
    {
        double const signedrate = Global.shiftState ? -rate : rate;
        // apply to every chunk the brush touches; each patch clips to its own bounds, so a stroke
        // crossing a chunk boundary edits both and shared-edge vertices stay in sync
        for (editor_terrain *terrain : active_terrains())
            terrain->sculpt(world.x, world.z, m_terrain_brush_radius, signedrate);
    }
    // what the road tools know about the ground is no longer true
    ground_tiles.clear();
    forget_ground_shapes();
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

// the ground under a point the cursor landed on. the orthophoto laid over the ground catches the cursor like any
// other surface, so the point can hang over the ground by as much as the orthophoto is lifted, or lie on the flat sheet
// the orthophoto makes where it's not fitted to the ground. what's put in the scenery is to stand on the ground itself
glm::dvec3 editor_mode::placement_on_ground(glm::dvec3 Location)
{
    double constexpr reach = 2.0; // the ground is asked about this far around the point
    double constexpr slack = 0.5; // a surface this far over the point still is what the cursor landed on
    double constexpr height_tolerance = 50.0; // with nothing there or lower, how far up the ground is looked for
    float constexpr terrain_model_radius = 50.0f; // the same models count as ground as for the area fill and the orthophoto

    glm::dvec2 const bmin{Location.x - reach, Location.z - reach};
    glm::dvec2 const bmax{Location.x + reach, Location.z + reach};
    std::vector<world_triangle> triangles;
    gather_ground_triangles(bmin, bmax, true, terrain_model_radius, triangles);
    triangle_grid const ground(std::move(triangles), bmin, bmax);
    auto const terrains = active_terrains();
    for (double const ceiling : {Location.y + slack, Location.y + height_tolerance})
    {
        double y = -std::numeric_limits<double>::max();
        bool found = ground.height_at(Location.x, Location.z, ceiling, y);
        for (editor_terrain *terrain : terrains)
        {
            if (!terrain->contains(Location.x, Location.z))
                continue;
            double const h = terrain->height_at(Location.x, Location.z);
            if (h <= ceiling && (!found || h > y))
            {
                y = h;
                found = true;
            }
        }
        if (found)
        {
            Location.y = y;
            break;
        }
    }
    // with no ground found the point stays where the cursor landed
    return Location;
}

// puts every instance of the model the selected node shows on the ground under it
void editor_mode::drop_model_instances()
{
    auto *selected = dynamic_cast<TAnimModel *>(m_node);
    if (selected == nullptr || selected->Model() == nullptr)
    {
        ui()->set_status(STR("Select a model in the scene first"), true);
        return;
    }
    if (selected->radius() >= 50.0f)
    {
        // it'd be put on top of itself
        ui()->set_status("\"" + selected->Model()->NameGet() + "\" is large enough to count as ground itself, so there's nothing to put it on", true);
        return;
    }
    std::vector<TAnimModel *> instances;
    std::vector<glm::dvec3> points;
    int locked = 0;
    for (auto *instance : simulation::Instances.sequence())
    {
        if (instance == nullptr || instance->Model() != selected->Model())
            continue;
        if (instance->from_template() || false == scene::Layers.editable(instance))
        {
            // placed by a template, or on a layer which is hidden or locked
            ++locked;
            continue;
        }
        instances.push_back(instance);
        points.push_back(instance->location());
    }
    // the ground the road tools use: terrain shapes, large models and the terrain made in the editor, without the roads and the orthophoto.
    // a point with nothing under it comes back with the height it has
    auto const heights = ground_heights(points, true);
    int moved = 0;
    for (std::size_t idx = 0; idx < instances.size(); ++idx)
    {
        if (std::abs(heights[idx] - points[idx].y) < 0.001)
            continue;
        auto *instance = instances[idx];
        // one step of the history for each instance
        push_snapshot(instance, EditorSnapshot::Action::Move);
        auto location = points[idx];
        location.y = heights[idx];
        simulation::Region->erase(instance);
        instance->location(location);
        simulation::Region->insert(instance);
        instance->mark_dirty();
        ++moved;
    }
    std::string status = std::to_string(moved) + " of " + std::to_string(instances.size()) + " instances of \"" + selected->Model()->NameGet() + "\" put on the ground";
    if (moved > 0)
        status += "; Ctrl+Z takes them back one at a time";
    if (locked > 0)
        status += ". " + std::to_string(locked) + " left alone: placed by a template, or on a layer which can't be edited";
    ui()->set_status(status);
}

void editor_mode::read_georeference()
{
    if (m_georeference_read)
        return;
    m_georeference_read = true;
    m_georeference = false;
    m_georeference_line.clear();
    // the name of the scenery is kept in lower case, the file may have capitals in its name
    std::filesystem::path file{Global.asCurrentSceneryPath + Global.SceneryFile};
    std::error_code error;
    if (false == std::filesystem::exists(file, error))
    {
        for (auto const &entry : std::filesystem::directory_iterator(Global.asCurrentSceneryPath, error))
        {
            auto name{entry.path().filename().string()};
            std::transform(name.begin(), name.end(), name.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
            if (name == Global.SceneryFile)
            {
                file = entry.path();
                break;
            }
        }
    }
    std::ifstream stream(file);
    std::string line;
    for (int count = 0; count < 200 && std::getline(stream, line); ++count)
    {
        auto const mark = line.find("//$g");
        if (mark == std::string::npos)
            continue;
        // Rainsted writes the origin of the scenery as: //$g <system> <easting> <northing>, in kilometres
        std::istringstream words(line.substr(mark + 4));
        std::string system;
        double east{0.0}, north{0.0};
        if (false == static_cast<bool>(words >> system >> east >> north) || system.find("1992") == std::string::npos)
            continue;
        auto const scale = (east < 10000.0 && north < 10000.0) ? 1000.0 : 1.0;
        m_georeference_origin = {north * scale, east * scale};
        m_georeference_line = line.substr(mark);
        if (false == m_georeference_line.empty() && m_georeference_line.back() == '\r')
            m_georeference_line.pop_back();
        m_georeference = true;
        break;
    }
}

void editor_mode::render_map_menu()
{
    if (false == ImGui::BeginMenu(STR_C("Map")))
        return;
    bool enabled = m_orthophoto.enabled();
    if (ImGui::MenuItem(STR_C("Orthophoto"), nullptr, enabled))
        m_orthophoto.enabled(false == enabled);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", STR_C("Aerial imagery of geoportal.gov.pl under the scenery, laid out by the origin of the scenery"));
    ImGui::MenuItem(STR_C("Orthophoto settings..."), nullptr, &m_orthophoto_window);
    ImGui::EndMenu();
}

void editor_mode::render_new_scenery_popup()
{
    if (m_newscenery_asked)
    {
        ImGui::OpenPopup(STR_C("New scenery##restart"));
        m_newscenery_asked = false;
    }
    if (false == ImGui::BeginPopupModal(STR_C("New scenery##restart"), nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::TextUnformatted(STR_C("The editor starts again, with the wizard of a new scenery."));
    auto const editsession{false == scene::Layers.empty()};
    if (false == m_history.empty())
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "%s", STR_C("What isn't saved of this scenery is lost."));
    if (editsession && ImGui::Button(STR_C("Save and start the wizard")))
    {
        if (save() && false == restart_for_new_scenery())
            ui()->set_status(STR_C("The editor couldn't be started again"), true);
        ImGui::CloseCurrentPopup();
    }
    if (editsession)
        ImGui::SameLine();
    if (ImGui::Button(STR_C("Start the wizard")))
    {
        if (false == restart_for_new_scenery())
            ui()->set_status(STR_C("The editor couldn't be started again"), true);
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button(STR_C("Cancel")))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

bool editor_mode::restart_for_new_scenery()
{
    return restart_editor(std::string{});
}

bool editor_mode::restart_editor(std::string const &Scenery)
{
#ifdef _WIN32
    _putenv("EU07_EDITOR_SELFTEST=");
    wchar_t path[MAX_PATH];
    if (GetModuleFileNameW(nullptr, path, MAX_PATH) == 0)
        return false;
    std::wstring command{L"\"" + std::wstring{path} + L"\" -edit"};
    if (false == Scenery.empty())
        command += L" \"" + std::filesystem::path(Scenery).wstring() + L"\"";
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION process{};
    if (false == CreateProcessW(nullptr, command.data(), nullptr, nullptr, FALSE, 0, nullptr, nullptr, &startup, &process))
        return false;
    CloseHandle(process.hThread);
    CloseHandle(process.hProcess);
#else
    unsetenv("EU07_EDITOR_SELFTEST");
    char path[4096];
    auto const size{readlink("/proc/self/exe", path, sizeof(path) - 1)};
    if (size <= 0)
        return false;
    path[size] = '\0';
    char edit[]{"-edit"};
    std::string scenery{Scenery};
    char *arguments[]{path, edit, scenery.empty() ? nullptr : scenery.data(), nullptr};
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETSID);
    pid_t child;
    auto const result{posix_spawn(&child, path, nullptr, &attributes, arguments, environ)};
    posix_spawnattr_destroy(&attributes);
    if (result != 0)
        return false;
#endif
    WriteLog(Scenery.empty() ? std::string{"Editor: started again for a new scenery"} : "Editor: started again with scenery " + Scenery);
    Application.queue_quit(true);
    return true;
}

void editor_mode::render_open_scenery_popup()
{
    auto const lower = [](std::string Text) {
        std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
        return Text;
    };
    if (m_openscenery_asked)
    {
        m_openscenery_asked = false;
        m_openscenery_list.clear();
        m_openscenery_choice.clear();
        std::error_code error;
        for (auto const &entry : std::filesystem::directory_iterator(Global.asCurrentSceneryPath, error))
        {
            auto const name{entry.path().filename().string()};
            if (entry.is_regular_file(error) && name.size() > 4 && lower(name.substr(name.size() - 4)) == ".scn" && name.front() != '$')
                m_openscenery_list.emplace_back(name);
        }
        std::sort(m_openscenery_list.begin(), m_openscenery_list.end(), [&](std::string const &A, std::string const &B) { return lower(A) < lower(B); });
        ImGui::OpenPopup(STR_C("Open scenery##restart"));
    }
    if (false == ImGui::BeginPopupModal(STR_C("Open scenery##restart"), nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::TextUnformatted(STR_C("The editor starts again with the scenery chosen below."));
    ImGui::SetNextItemWidth(360.0f);
    if (ImGui::IsWindowAppearing())
        ImGui::SetKeyboardFocusHere();
    ImGui::InputTextWithHint("##opensearch", STR_C("search"), m_openscenery_search, sizeof(m_openscenery_search));
    std::string const text{lower(m_openscenery_search)};
    ImGui::BeginChild("##openlist", ImVec2(360.0f, ImGui::GetTextLineHeightWithSpacing() * 14.0f), true);
    for (auto const &name : m_openscenery_list)
    {
        if (false == text.empty() && lower(name).find(text) == std::string::npos)
            continue;
        if (ImGui::Selectable(name.c_str(), name == m_openscenery_choice, ImGuiSelectableFlags_AllowDoubleClick))
        {
            m_openscenery_choice = name;
            if (ImGui::IsMouseDoubleClicked(0) && false == restart_editor(name))
                ui()->set_status(STR_C("The editor couldn't be started again"), true);
        }
    }
    ImGui::EndChild();
    auto const editsession{false == scene::Layers.empty()};
    if (false == m_history.empty())
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "%s", STR_C("What isn't saved of this scenery is lost."));
    auto const chosen{false == m_openscenery_choice.empty()};
    if (chosen && editsession && ImGui::Button(STR_C("Save and open")))
    {
        if (save() && false == restart_editor(m_openscenery_choice))
            ui()->set_status(STR_C("The editor couldn't be started again"), true);
        ImGui::CloseCurrentPopup();
    }
    if (chosen && editsession)
        ImGui::SameLine();
    if (chosen && ImGui::Button(STR_C("Open")))
    {
        if (false == restart_editor(m_openscenery_choice))
            ui()->set_status(STR_C("The editor couldn't be started again"), true);
        ImGui::CloseCurrentPopup();
    }
    if (chosen)
        ImGui::SameLine();
    if (ImGui::Button(STR_C("Cancel")))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void editor_mode::render_orthophoto_window()
{
    if (false == m_orthophoto_window)
        return;
    ImGui::SetNextWindowSize(ImVec2(430.0f, 0.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(STR_C("Orthophoto##window"), &m_orthophoto_window, ImGuiWindowFlags_AlwaysAutoResize))
        render_orthophoto_ui();
    ImGui::End();
}

void editor_mode::render_object_menu()
{
    if (ImGui::BeginMenu(STR_C("Objects")))
    {
        auto const *selected = dynamic_cast<TAnimModel const *>(m_node);
        if (ImGui::MenuItem(STR_C("Put all instances of the selected model on the ground"), nullptr, false, selected != nullptr && selected->Model() != nullptr))
            drop_model_instances();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("%s", STR_C("Select a model in the scene. Every instance of the same model file is moved straight up or down\n"
                                    "to the ground under it: terrain, large models and terrain made in the editor."));
        ImGui::EndMenu();
    }
}

void editor_mode::add_area_fill_point()
{
    // like the other placement tools, the point comes from the depth buffer under the cursor
    // (ground geometry only), relative to the camera
    glm::dvec3 const offset = GfxRenderer->Mouse_Position();
    if (glm::length(offset) < 1e-3)
        return; // no ground under the cursor (yet)

    // the outline gives the fill the height to look for the ground at, and the height of what it finds no ground for
    m_fill_points.push_back(placement_on_ground(Camera.Pos + offset));
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
    m_fill_status = format(STR_C("Removed %zu objects of the last fill"), created.size());
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
        m_fill_status = STR("Outline needs at least 3 points");
        return;
    }

    // model package
    std::vector<std::string> package;
    if (m_fill_source.kind == model_set_ref::source::manual)
        package = m_fill_custom;
    else
        for (auto const *entry : ui()->nodebank().set_entries(m_fill_source))
            package.push_back(*entry);
    package.erase(std::remove_if(package.begin(), package.end(), [](std::string const &Template) { return Template.empty(); }), package.end());
    if (package.empty())
    {
        m_fill_status = STR("Selected model package is empty");
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
        m_fill_status = STR("Density too low for this area, nothing to place");
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

    // ground geometry within the area
    std::vector<world_triangle> triangles;
    gather_ground_triangles(bmin, bmax, m_fill_models_as_ground, terrain_model_radius, triangles);
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
        // NOTE: no names here either, same as with the brush
        TAnimModel *cloned = simulation::State.create_model(src, std::string{}, location);
        if (!cloned)
            continue;
        apply_rotation_for_new_node(cloned, rotation_mode, fixed_rotation_value);
        if (scalemin != 1.0f || scalemax != 1.0f)
            cloned->Scale(static_cast<float>(LocalRandom(scalemin, scalemax)));
        created.push_back(cloned);
    }

    // a new fill replaces the "last fill" undo set; the previous objects stay in the scene
    m_fill_last = std::move(created);

    m_fill_status = format(STR_C("Placed %zu of %zu objects"), m_fill_last.size(), static_cast<std::size_t>(target));
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
    float const aspect = io.DisplaySize.y > 0.0f ? io.DisplaySize.x / io.DisplaySize.y : 1.0f;
    glm::mat4 const viewprojection = editor_mode::projection_matrix(aspect) * view;

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
    ImGui::TextDisabled(STR_C("LMB: add outline point   Backspace: remove last point"));
    double const area = polygon_area_xz(m_fill_points);
    ImGui::Text(STR_C("Points: %zu   Area: %.0f m2 (%.2f ha)"), m_fill_points.size(), area, area / 10000.0);
    if (ImGui::Button(STR_C("Remove last point")) && !m_fill_points.empty())
        m_fill_points.pop_back();
    ImGui::SameLine();
    if (ImGui::Button(STR_C("Clear outline")))
        m_fill_points.clear();

    ImGui::Separator();

    // model package: a hand-assembled list, a user-defined set or a node bank group
    auto &bank = ui()->nodebank();
    bank.set_combo(STR_C("Model set"), m_fill_source, STR_C("Custom list"));
    if (m_fill_source.kind == model_set_ref::source::manual)
        bank.manual_list("fillset", m_fill_custom, m_fill_custom_idx);
    else
        ImGui::TextDisabled(STR_C("%zu templates in set"), bank.set_entries(m_fill_source).size());

    ImGui::Separator();

    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat(STR_C("Density (objects/ha)"), &m_fill_density, 10.0f, 100.0f, "%.1f");
    m_fill_density = std::max(0.0f, m_fill_density);
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat(STR_C("Min. spacing (m)"), &m_fill_min_spacing, 0.5f, 2.0f, "%.1f");
    m_fill_min_spacing = std::max(0.0f, m_fill_min_spacing);
    ImGui::SetNextItemWidth(200.0f);
    ImGui::DragFloatRange2("Scale", &m_fill_scale_min, &m_fill_scale_max, 0.01f, 0.1f, 5.0f, "min %.2f", "max %.2f");
    ImGui::Checkbox(STR_C("Random rotation"), &m_fill_random_rotation);
    if (!m_fill_random_rotation)
        ui()->render_rotation_controls();
    ImGui::Checkbox(STR_C("Large models count as ground (terrain tiles)"), &m_fill_models_as_ground);

    ImGui::Text(STR_C("Estimated objects: %.0f"), std::round(area / 10000.0 * m_fill_density));

    if (ImGui::Button(STR_C("Fill area")))
        run_area_fill();
    ImGui::SameLine();
    std::string const undolabel = format(STR_C("Undo last fill (%zu)"), m_fill_last.size());
    if (ImGui::Button(undolabel.c_str()) && !m_fill_last.empty())
        undo_last_area_fill();

    if (!m_fill_status.empty())
        ImGui::TextUnformatted(m_fill_status.c_str());
}

glm::vec3 editor_mode::array_size(TAnimModel const &Base)
{
    auto *model = Base.Model();
    if (model == nullptr)
        return glm::vec3{0.0f};
    if (m_array.sized == model)
        return m_array.size;
    // the triangles the other tools take for the ground, left where the model has them
    std::vector<world_triangle> triangles;
    gather_submodel_triangles(model->Root, glm::dmat4{1.0}, triangles);
    if (triangles.empty())
        return glm::vec3{0.0f}; // not loaded yet, or nothing to it; it's asked about again the next time
    glm::dvec3 low{std::numeric_limits<double>::max()};
    glm::dvec3 high{-std::numeric_limits<double>::max()};
    for (auto const &triangle : triangles)
    {
        for (auto const &vertex : triangle)
        {
            low = glm::min(low, vertex);
            high = glm::max(high, vertex);
        }
    }
    m_array.sized = model;
    m_array.size = glm::vec3{high - low};
    return m_array.size;
}

glm::dvec3 editor_mode::array_offset(TAnimModel const &Base)
{
    auto const &settings = m_array.settings;
    glm::dvec3 offset{0.0};
    if (settings.relative)
        offset += glm::dvec3{settings.relative_offset * array_size(Base) * Base.Scale()};
    if (settings.constant)
        offset += glm::dvec3{settings.constant_offset};
    return offset;
}

int editor_mode::array_count(double const Step) const
{
    auto const &settings = m_array.settings;
    if (settings.fit == 0)
        return std::clamp(settings.count, 1, array_tool::limit);
    if (Step < array_tool::minimal_step)
        return 1;
    // the model, and a copy for each whole offset the length takes
    return static_cast<int>(std::min<double>(array_tool::limit, std::floor(std::max(0.0f, settings.length) / Step + 1e-6) + 1.0));
}

std::vector<std::pair<glm::dvec3, glm::vec3>> editor_mode::array_placements(TAnimModel &Base)
{
    // a model this large counts as ground itself, and would be put on top of its own copies
    float constexpr terrain_model_radius = 50.0f;

    std::vector<std::pair<glm::dvec3, glm::vec3>> places;
    auto const &settings = m_array.settings;
    auto const offset = array_offset(Base);
    auto const count = array_count(glm::length(offset));
    if (count < 2 || glm::length(offset) < array_tool::minimal_step)
        return places;

    // the offset goes along the axes of the model, which is turned the way the renderer does it: around y, then x, then z
    auto angles = Base.Angles();
    glm::dmat4 turned{1.0};
    turned = glm::rotate(turned, glm::radians(static_cast<double>(angles.y)), glm::dvec3{0.0, 1.0, 0.0});
    turned = glm::rotate(turned, glm::radians(static_cast<double>(angles.x)), glm::dvec3{1.0, 0.0, 0.0});
    turned = glm::rotate(turned, glm::radians(static_cast<double>(angles.z)), glm::dvec3{0.0, 0.0, 1.0});
    auto step = glm::dvec3{turned * glm::dvec4{offset, 0.0}};
    auto location = Base.location();
    places.reserve(count - 1);
    for (int idx = 1; idx < count; ++idx)
    {
        // each copy is turned against the one before it, and leads to the next one the way it faces
        location += step;
        if (false == simulation::Region->point_inside(location))
            break; // the row ends at the edge of the region, nodes past it are left out of the scene
        if (settings.turn != 0.0f)
        {
            step = glm::rotateY(step, glm::radians(static_cast<double>(settings.turn)));
            angles.y = clamp_circular(angles.y + settings.turn, 360.0f);
        }
        places.emplace_back(location, angles);
    }

    if (settings.ground && Base.radius() < terrain_model_radius)
    {
        // the ground the road tools use. a copy with nothing under it stays at the height the offset gave it
        std::vector<glm::dvec3> points;
        points.reserve(places.size() + 1);
        points.push_back(Base.location());
        for (auto const &place : places)
            points.push_back(place.first);
        std::vector<char> found;
        auto const heights = ground_heights(points, true, &found);
        if (found[0])
        {
            auto const clearance = points[0].y - heights[0];
            for (std::size_t idx = 0; idx < places.size(); ++idx)
            {
                if (found[idx + 1])
                    places[idx].first.y = heights[idx + 1] + clearance;
            }
        }
    }
    return places;
}

bool editor_mode::array_live() const
{
    // the model the array was made of is still selected, and the array is the last thing done
    return m_array.base != nullptr && m_array.base == m_node && false == m_history.empty() && m_history.size() == m_array.step &&
           m_history.back().action == EditorSnapshot::Action::Array;
}

void editor_mode::make_array()
{
    auto *base = dynamic_cast<TAnimModel *>(m_node);
    if (base == nullptr || false == scene::Layers.editable(base))
        return;
    auto const step = glm::length(array_offset(*base));
    if (step < array_tool::minimal_step)
    {
        m_array.status = STR("The offsets come to nothing, the copies would sit in the model");
        return;
    }
    if (array_count(step) < 2)
    {
        m_array.status = (m_array.settings.fit == 0 ? STR("An array of one is the model itself, nothing to make") : STR("The length takes no copy, nothing to make"));
        return;
    }

    // a single undo step for the whole array, whatever is done to it while it follows the settings
    trim_history();
    EditorSnapshot snap;
    snap.action = EditorSnapshot::Action::Array;
    snap.node_name = base->name();
    snap.position = base->location();
    snap.layer = base->layer();
    m_history.push_back(std::move(snap));
    g_redo.clear();
    m_array.base = base;
    m_array.step = m_history.size();
    shape_array();
}

void editor_mode::shape_array()
{
    auto *base = m_array.base;
    auto &copies = m_history.back().copies;
    auto const places = array_placements(*base);

    // the copies there's no place for anymore go
    while (copies.size() > places.size())
    {
        auto *model = copies.back().model;
        copies.pop_back();
        if (model == nullptr)
            continue;
        nullify_history_pointers(model);
        remove_from_hierarchy(model);
        simulation::State.delete_model(model);
    }

    std::string definition;
    for (std::size_t idx = 0; idx < places.size(); ++idx)
    {
        TAnimModel *model = nullptr;
        if (idx < copies.size())
        {
            model = copies[idx].model;
            m_editor.translate(model, places[idx].first, true); // true: the height is set as well
        }
        else
        {
            if (definition.empty())
                base->export_as_text(definition);
            // NOTE: no names for the copies, same as with the brush
            model = simulation::State.create_model(definition, std::string{}, places[idx].first);
            if (model == nullptr)
                break;
            // the copies go where the model is, rather than to the active layer
            scene::Layers.move(model, base->layer());
            EditorSnapshot::array_copy copy;
            copy.model = model;
            copy.uuid = model->uuid;
            copies.push_back(std::move(copy));
        }
        model->Angles(places[idx].second);
        model->Scale(base->Scale());
    }

    m_array.applied = m_array.settings;
    m_array.location = base->location();
    m_array.angles = base->Angles();
    m_array.scale = base->Scale();
    if (copies.size() < places.size())
        m_array.status = format(STR_C("Only %zu of %zu copies could be made"), copies.size(), places.size());
    else if (copies.empty())
        m_array.status = STR("No copies with these settings");
    else
        m_array.status = format(STR_C("The model and %zu copies; Ctrl+Z takes the array back"), copies.size());
}

void editor_mode::update_array()
{
    if (m_array.base == nullptr)
        return;
    if (false == array_live())
    {
        // once left, the copies are models like any other
        m_array.base = nullptr;
        return;
    }
    auto const *base = m_array.base;
    if (m_array.settings == m_array.applied && base->location() == m_array.location && base->Angles() == m_array.angles && base->Scale() == m_array.scale)
        return;
    shape_array();
}

void editor_mode::restore_array_snapshot(EditorSnapshot Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo)
{
    // either way the array is done with following the settings
    m_array.base = nullptr;
    for (auto &copy : Snapshot.copies)
    {
        if (Undo)
        {
            // NOTE: a copy which was deleted and brought back is another model by now, found by its uuid
            auto *model = dynamic_cast<TAnimModel *>(find_node_by_any(copy.model, copy.uuid.to_string(), std::string{}));
            copy.model = nullptr;
            copy.serialized.clear();
            if (model == nullptr)
                continue; // deleted since, and stays that way
            // the copy is brought back the way it is now, with whatever was done to it since the array was made
            model->export_as_text(copy.serialized);
            copy.name = model->name();
            copy.position = model->location();
            copy.rotation = model->Angles();
            copy.scale = model->Scale();
            if (m_node == model)
            {
                m_node = nullptr;
                m_dragging = false;
                ui()->set_node(nullptr);
            }
            nullify_history_pointers(model);
            remove_from_hierarchy(model);
            simulation::State.delete_model(model);
        }
        else
        {
            if (copy.serialized.empty())
                continue;
            auto *model = simulation::State.create_model(copy.serialized, copy.name, copy.position);
            if (model == nullptr)
                continue;
            scene::Layers.move(model, Snapshot.layer);
            model->Angles(copy.rotation);
            model->Scale(copy.scale);
            model->uuid = copy.uuid;
            add_to_hierarchy(model);
            copy.model = model;
        }
    }
    Opposite.push_back(std::move(Snapshot));
}

void editor_mode::render_array()
{
    auto &settings = m_array.settings;
    auto *selected = dynamic_cast<TAnimModel *>(m_node);
    if (selected != nullptr && false == scene::Layers.editable(selected))
        selected = nullptr;

    ImGui::PushID("array");
    ImGui::TextDisabled(STR_C("Copies of the selected model, set out in a row"));
    ImGui::RadioButton(STR_C("Fixed count"), &settings.fit, 0);
    ImGui::SameLine();
    ImGui::RadioButton(STR_C("Fit length"), &settings.fit, 1);
    ImGui::SetNextItemWidth(120.0f);
    if (settings.fit == 0)
    {
        ImGui::InputInt(STR_C("Count"), &settings.count);
        settings.count = std::clamp(settings.count, 1, array_tool::limit);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", STR_C("Number of the models in the array, the selected one included"));
    }
    else
    {
        ImGui::InputFloat(STR_C("Length (m)"), &settings.length, 1.0f, 10.0f, "%.2f");
        settings.length = std::max(0.0f, settings.length);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", STR_C("As many copies are made as fit in this length, measured along the row"));
    }

    ImGui::Checkbox(STR_C("Relative offset"), &settings.relative);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", STR_C("Distance between the copies as a part of the size of the model, along its axes: x, y (up), z.\n"
                                "1 along an axis puts the copies end to end"));
    if (settings.relative)
    {
        ImGui::SetNextItemWidth(200.0f);
        ImGui::DragFloat3(STR_C("x size"), &settings.relative_offset.x, 0.01f, 0.0f, 0.0f, "%.3f");
    }
    ImGui::Checkbox(STR_C("Constant offset"), &settings.constant);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", STR_C("Distance between the copies in metres, along the axes of the model: x, y (up), z.\n"
                                "With both offsets on the copies are set apart by their sum"));
    if (settings.constant)
    {
        ImGui::SetNextItemWidth(200.0f);
        ImGui::DragFloat3("m", &settings.constant_offset.x, 0.05f, 0.0f, 0.0f, "%.3f");
    }
    ImGui::SetNextItemWidth(120.0f);
    ImGui::DragFloat(STR_C("Turn per copy (deg)"), &settings.turn, 0.1f, -180.0f, 180.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", STR_C("Each copy is turned by this much around the vertical axis against the one before it,\n"
                                "and the row bends with them into an arc"));
    ImGui::Checkbox(STR_C("Follow the ground"), &settings.ground);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", STR_C("The copies are put as high over the ground as the model is, instead of level with it.\n"
                                "Not for the models large enough to count as ground themselves"));

    if (selected != nullptr)
    {
        auto const size = array_size(*selected) * selected->Scale();
        auto const step = glm::length(array_offset(*selected));
        ImGui::TextDisabled(STR_C("Model %.2f x %.2f x %.2f m, %d in the array, %.2f m apart"), size.x, size.y, size.z, array_count(step), step);
    }

    if (array_live())
    {
        // NOTE: kept away from the place of the other button, a click too many on which would make the array twice
        ImGui::TextDisabled(STR_C("The array follows the settings"));
        ImGui::SameLine();
        if (ImGui::Button(STR_C("Done")))
            m_array.base = nullptr;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", STR_C("The copies stay the way they are. Selecting or changing anything else does the same"));
    }
    else if (selected != nullptr)
    {
        if (ImGui::Button(STR_C("Make array")))
            make_array();
    }
    else
    {
        ImGui::TextDisabled(STR_C("Select a model to make an array of"));
    }
    if (false == m_array.status.empty())
        ImGui::TextUnformatted(m_array.status.c_str());
    ImGui::PopID();
}

void editor_mode::render_gizmo_options()
{
    if (ImGui::Button(Global.EditorOrtho ? "3D view (O)" : "Top view, orthographic (O)"))
        toggle_ortho();
    if (Global.EditorOrtho)
    {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::SliderFloat(STR_C("Extent (m), wheel"), &Global.EditorOrthoExtent, 5.0f, 5000.0f, "%.0f", 3.0f);
    }
    ImGui::Checkbox(STR_C("Enabled"), &m_gizmo_enabled);
    if (!m_gizmo_enabled)
        return;
    if (m_terrain_sculpt || m_chunk_edit)
    {
        ImGui::TextDisabled(STR_C("Suspended while editing terrain"));
        return;
    }

    // lets the user pick the transform mode without keyboard shortcuts
    int op = static_cast<int>(m_gizmo_op);
    ImGui::RadioButton(STR_C("Translate (Q)"), &op, static_cast<int>(gizmo_operation::translate));
    ImGui::SameLine();
    ImGui::RadioButton(STR_C("Rotate (W)"), &op, static_cast<int>(gizmo_operation::rotate));
    ImGui::SameLine();
    ImGui::RadioButton(STR_C("Scale (E)"), &op, static_cast<int>(gizmo_operation::scale));
    m_gizmo_op = static_cast<gizmo_operation>(op);

    if (m_gizmo_op != gizmo_operation::scale) // ImGuizmo always scales in local space
        ImGui::Checkbox(STR_C("Local space (R)"), &m_gizmo_local);
    if (m_gizmo_op == gizmo_operation::translate)
    {
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat(STR_C("Snap (hold Ctrl)"), &m_gizmo_snap);
        if (m_gizmo_snap < 0.0f)
            m_gizmo_snap = 0.0f;
    }
    if (m_instance != 0)
    {
        auto const located = editor_includes::parameter_with_role(m_include.info, "pos.x") != 0 || editor_includes::parameter_with_role(m_include.info, "pos.y") != 0 ||
                             editor_includes::parameter_with_role(m_include.info, "pos.z") != 0;
        ImGui::TextDisabled("%s", false == located             ? "Include selected, no position parameters described" :
                                  false == m_include.placement ? STR_C("Include selected, no gizmo under rotate or scale") :
                                                                 STR_C("Include selected, axes as its template allows"));
    }
    else if (!m_node)
        ImGui::TextDisabled(STR_C("No node selected"));
}

void editor_mode::render_gizmo()
{
    // the transform gizmo is suppressed while editing terrain, so the brush/chunk tool owns the mouse
    if (!m_gizmo_enabled || m_terrain_sculpt || m_chunk_edit)
    {
        m_gizmo_using = false;
        return;
    }

    if (ui()->mode() == nodebank_panel::TRACK && m_track_tab == track_tab::path && m_track_set.size() >= 2)
    {
        render_track_set_gizmo();
        return;
    }

    if (straights_active())
    {
        render_straight_gizmo();
        return;
    }

    if (route_active())
    {
        render_route_gizmo();
        return;
    }

    if (!m_node && m_instance == 0)
    {
        m_gizmo_using = false;
        return;
    }

    if (selected_track())
    {
        // in the track modes the whole path is moved only while selecting, or the switch in its mode
        if (ui()->mode() != nodebank_panel::TRACK || (m_track_tab == track_tab::path && false == m_point_drag.active) || (m_track_tab == track_tab::turnout && selected_track()->eType == tt_Switch))
            render_track_gizmo();
        else
            m_track_gizmo_using = false;
        return;
    }

    ImGuizmo::BeginFrame();
    ImGuizmo::SetOrthographic(Global.EditorOrtho);

    ImGuiIO const &io = ImGui::GetIO();
    ImGuizmo::SetRect(0.0f, 0.0f, io.DisplaySize.x, io.DisplaySize.y);

    // the view matrix comes from the most recent color pass and is camera-relative
    // (rotation only), so the gizmo is positioned relative to the camera as well.
    glm::mat4 const view = GfxRenderer->Camera_View_Matrix();
    glm::dvec3 const camerapos = GfxRenderer->Camera_Position();

    // the engine's own projection bakes in reverse-Z (and screen orientation), which ImGuizmo
    // doesn't expect; rebuild a clean, standard perspective that matches the rendered view.
    // for the main viewport the engine uses a symmetric frustum with this exact fov/aspect.
    float const aspect = io.DisplaySize.y > 0.0f ? io.DisplaySize.x / io.DisplaySize.y : 1.0f;
    glm::mat4 const projection = editor_mode::projection_matrix(aspect);

    if (!m_node)
    {
        render_include_gizmo(view, projection, camerapos);
        return;
    }

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

void editor_mode::render_include_gizmo(glm::mat4 const &View, glm::mat4 const &Projection, glm::dvec3 const &Camerapos)
{
    m_gizmo_using = false;
    if (false == m_include.placement || false == m_include.issue.empty())
        return;

    // parameters of the template the gizmo can drive, as indices of their values; -1 where the template has none
    static char const *const positionroles[] = {"pos.x", "pos.y", "pos.z"};
    static char const *const rotationroles[] = {"rot.x", "rot.y", "rot.z"};
    auto const numeric_parameter = [this](char const *Role, double &Value) {
        auto const index = editor_includes::parameter_with_role(m_include.info, Role) - 1;
        if (index < 0 || index >= static_cast<int>(m_include.values.size()))
            return -1;
        auto const &text = m_include.values[index];
        char *end = nullptr;
        auto const number = std::strtod(text.c_str(), &end);
        if (text.empty() || *end != '\0')
            return -1;
        Value = number;
        return index;
    };
    int position[3], rotation[3];
    double location[3] = {0.0, 0.0, 0.0}, angles[3] = {0.0, 0.0, 0.0};
    auto positions = 0, rotations = 0;
    for (auto axis = 0; axis < 3; ++axis)
    {
        position[axis] = numeric_parameter(positionroles[axis], location[axis]);
        rotation[axis] = numeric_parameter(rotationroles[axis], angles[axis]);
        positions += (position[axis] >= 0 ? 1 : 0);
        rotations += (rotation[axis] >= 0 ? 1 : 0);
    }
    if (positions == 0)
        return; // nothing tells where the include is

    // only the axes the template has parameters for can be manipulated
    auto operation = 0;
    for (auto axis = 0; axis < 3; ++axis)
    {
        if (m_gizmo_op == gizmo_operation::translate && position[axis] >= 0)
            operation |= (ImGuizmo::TRANSLATE_X << axis);
        else if (m_gizmo_op == gizmo_operation::rotate && rotation[axis] >= 0)
            operation |= (ImGuizmo::ROTATE_X << axis);
    }
    if (operation == 0)
        return;
    auto const translating = (m_gizmo_op == gizmo_operation::translate);

    // the parameters are relative to the origin in effect at the directive
    auto const &offset = scene::Layers.instance(m_instance).context.offset;
    glm::vec3 const relativepos = glm::vec3(offset + glm::dvec3{location[0], location[1], location[2]} - Camerapos);
    float const translation[3] = {relativepos.x, relativepos.y, relativepos.z};
    float const rotationangles[3] = {static_cast<float>(angles[0]), static_cast<float>(angles[1]), static_cast<float>(angles[2])};
    float const scale[3] = {1.0f, 1.0f, 1.0f};
    glm::mat4 matrix(1.0f);
    ImGuizmo::RecomposeMatrixFromComponents(translation, rotationangles, scale, glm::value_ptr(matrix));

    // moving along the axes of the include would need all three of them
    ImGuizmo::MODE const mode = (m_gizmo_local && (false == translating || positions == 3)) ? ImGuizmo::LOCAL : ImGuizmo::WORLD;
    glm::vec3 const snapvalue(translating ? m_gizmo_snap : 5.0f);
    float const *snap = Global.ctrlState && snapvalue.x > 0.0f ? glm::value_ptr(snapvalue) : nullptr;

    ImGuizmo::Manipulate(glm::value_ptr(View), glm::value_ptr(Projection), static_cast<ImGuizmo::OPERATION>(operation), mode, glm::value_ptr(matrix), nullptr, snap);

    if (false == ImGuizmo::IsUsing())
        return;
    m_gizmo_using = true;

    float newtranslation[3], newrotation[3], newscale[3];
    ImGuizmo::DecomposeMatrixToComponents(glm::value_ptr(matrix), newtranslation, newrotation, newscale);
    if (translating)
    {
        glm::dvec3 const newlocation = Camerapos + glm::dvec3{newtranslation[0], newtranslation[1], newtranslation[2]} - offset;
        for (auto axis = 0; axis < 3; ++axis)
        {
            if (position[axis] >= 0)
                m_include.values[position[axis]] = editor_includes::number(newlocation[axis]);
        }
    }
    else
    {
        if (rotations == 1)
        {
            // the decomposition spreads a turn of more than a quarter over all three angles;
            // with a single axis to turn around the angle is taken straight from the matrix
            newrotation[0] = glm::degrees(std::atan2(matrix[1][2], matrix[1][1]));
            newrotation[1] = glm::degrees(std::atan2(-matrix[0][2], matrix[0][0]));
            newrotation[2] = glm::degrees(std::atan2(matrix[0][1], matrix[0][0]));
        }
        for (auto axis = 0; axis < 3; ++axis)
        {
            if (rotation[axis] >= 0)
                m_include.values[rotation[axis]] = editor_includes::number(clamp_circular(newrotation[axis]));
        }
    }
    apply_include();
}

glm::mat4 editor_mode::projection_matrix(float const Aspect)
{
    if (Global.EditorOrtho)
    {
        auto const height{Global.EditorOrthoExtent};
        return glm::ortho(-height * Aspect, height * Aspect, -height, height, -10000.0f, 10000.0f);
    }
    return glm::perspective(glm::radians(Global.FieldOfView / Global.ZoomFactor), Aspect, 0.1f, 10000.0f);
}

void editor_mode::toggle_ortho()
{
    Global.EditorOrtho = !Global.EditorOrtho;
    if (Global.EditorOrtho)
    {
        m_ortho_pitch = Camera.Angle.x;
        Camera.Angle.x = -M_PI_2;
        Camera.Angle.z = 0.0;
    }
    else
    {
        Camera.Angle.x = m_ortho_pitch;
    }
}

void editor_mode::on_scroll(double const Xoffset, double const Yoffset)
{
    if (false == Global.EditorOrtho || ImGui::GetIO().WantCaptureMouse)
        return;
    auto const before{Global.EditorOrthoExtent};
    auto const after{std::clamp(before * static_cast<float>(std::pow(0.85, Yoffset)), 5.0f, 5000.0f)};
    screen_projection const projection;
    glm::dvec3 origin, direction;
    projection.ray(ImGui::GetIO().MousePos, origin, direction);
    auto const scale{static_cast<double>(after) / static_cast<double>(before)};
    Camera.Pos.x = origin.x + (Camera.Pos.x - origin.x) * scale;
    Camera.Pos.z = origin.z + (Camera.Pos.z - origin.z) * scale;
    Global.EditorOrthoExtent = after;
    Global.pCamera = Camera;
}

void editor_mode::update_camera(double const Deltatime)
{
    Camera.Update();
    if (Global.EditorOrtho)
    {
        Camera.Angle.x = -M_PI_2;
        Camera.Angle.z = 0.0;
    }

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

    if (simulation::Region != nullptr)
    {
        // the editor works with geometry of the whole scenery, including the parts of binary terrain which weren't needed so far
        simulation::Region->load_terrain();
    }

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

    if (Global.editor_session && false == m_opened_at_track)
    {
        m_opened_at_track = true;
        auto const saved{scene::Layers.empty() ? std::vector<std::string>{} : scene::Layers.marked(scene::layer_handle{1}, "//$c")};
        if (false == saved.empty())
        {
            std::istringstream values(saved.front());
            glm::dvec3 position{0.0};
            glm::vec3 angle{0.0f};
            if (values >> position.x >> position.y >> position.z >> angle.x >> angle.y)
            {
                Camera.Pos = position;
                Camera.Angle = angle;
                Camera.m_owner = nullptr;
                FreeFlyModeFlag = true;
                Global.pCamera = Camera;
                m_saved_camera = {position, angle};
                WriteLog("Editor: camera where it was at the last save");
            }
        }
        for (auto *track : simulation::Paths.sequence())
        {
            if (m_saved_camera.has_value() || track == nullptr || track->m_editorremoved || track->m_paths.empty())
                continue;
            auto const &path{track->m_paths.front()};
            auto const start{path.points[segment_data::point::start]};
            auto const end{path.points[segment_data::point::end]};
            auto const centre{0.5 * (start + end)};
            glm::dvec3 along{end.x - start.x, 0.0, end.z - start.z};
            along = glm::length(along) > 1e-6 ? glm::normalize(along) : glm::dvec3{0.0, 0.0, 1.0};
            glm::dvec3 const side{-along.z, 0.0, along.x};
            Camera.Pos = centre - along * 30.0 + side * 15.0 + glm::dvec3{0.0, 20.0, 0.0};
            auto const look{glm::normalize(centre - Camera.Pos)};
            Camera.Angle = glm::vec3(static_cast<float>(std::asin(std::clamp(look.y, -1.0, 1.0))), static_cast<float>(std::atan2(-look.x, -look.z)), 0.0f);
            Camera.m_owner = nullptr;
            FreeFlyModeFlag = true;
            Global.pCamera = Camera;
            WriteLog("Editor: camera at the first track, " + track->name());
            break;
        }
    }

    Global.ControlPicking = true;
    EditorModeFlag = true;

    load_orthophoto_settings();

    Application.set_cursor(GLFW_CURSOR_NORMAL);
}

void editor_mode::exit()
{
    EditorModeFlag = false;
    Global.ControlPicking = m_statebackup.picking;
    FreeFlyModeFlag = m_statebackup.freefly;
    Global.pCamera = m_statebackup.camera;

    commit_track_drag(true);
    m_track_gizmo_using = false;
    m_track_drag.clear();
    m_track_drag_points.clear();
    m_track_snap = {};
    m_track_point = {};
    m_route = {};
    m_route_gizmo_using = false;
    if (Global.EditorOrtho)
        toggle_ortho();

    g_redo.clear();
    m_history.clear();
    m_fill_last.clear();
    m_orthophoto.cancel_pending();
    m_orthophoto.detach_scene(); // the layer is an editor aid, keep it out of the other modes
    scene::Layers.show_all(); // likewise for hidden scenery layers, the other modes get the scenery as it was defined

    // drop selection so a stale/dangling node pointer isn't used on the next editor session
    m_node = nullptr;
    select_include(0);
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

    if (Action == GLFW_RELEASE && Key >= 0 && Key <= GLFW_KEY_LAST)
    {
        input::keys[Key] = GLFW_RELEASE;
        if (Key == GLFW_KEY_LEFT_SHIFT || Key == GLFW_KEY_RIGHT_SHIFT)
            input::key_shift = false;
        if (Key == GLFW_KEY_LEFT_CONTROL || Key == GLFW_KEY_RIGHT_CONTROL)
            input::key_ctrl = false;
        if (Key == GLFW_KEY_LEFT_ALT || Key == GLFW_KEY_RIGHT_ALT)
            input::key_alt = false;
    }

    // first give UI a chance to handle the key
    if (!anyModifier && m_userinterface->on_key(Key, Action))
        return;

    if (ui()->mode() == nodebank_panel::TRACK && is_press(Action) && (Mods & (GLFW_MOD_CONTROL | GLFW_MOD_ALT)) == 0 && typed_key(Key))
        return;

    // gizmo transform shortcuts (Q/W/E/R) — only when the camera isn't being flown (RMB up).
    // handled before the camera keyboard step because Q/W/E are also the fly-mode movement keys,
    // which would otherwise consume them.
    if (!anyModifier && is_press(Action)
        && m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) != GLFW_PRESS)
    {
        if (ui()->mode() == nodebank_panel::TRACK && false == m_track_gizmo_using && false == m_straights.dragging && false == m_switch.placing && track_shortcut(Key))
            return;
        bool handled = true;
        switch (Key)
        {
        case GLFW_KEY_O: toggle_ortho(); break;
        case GLFW_KEY_Q: m_gizmo_op = gizmo_operation::translate; break;
        case GLFW_KEY_W: m_gizmo_op = gizmo_operation::rotate; break;
        case GLFW_KEY_E: m_gizmo_op = gizmo_operation::scale; break;
        case GLFW_KEY_R: m_gizmo_local = !m_gizmo_local; break;
        case GLFW_KEY_B: handled = ui()->mode() != nodebank_panel::TRACK && bend_shortcut(); break;
        default: handled = false; break;
        }
        if (handled)
            return;
    }

    // save; checked ahead of the camera keys, which have S bound to moving back
    if (Global.ctrlState && false == Global.shiftState && Key == GLFW_KEY_S)
    {
        if (is_press(Action))
            save();
        return;
    }

    if (Global.ctrlState && false == Global.shiftState && Key == GLFW_KEY_F && ui()->mode() == nodebank_panel::TRACK)
    {
        if (is_press(Action))
        {
            show_track_tab(m_track_tab);
            m_track_search.focus = true;
        }
        return;
    }

    // then internal input handling
    if (m_input.keyboard.key(Key, Action))
        return;

    if (Action == GLFW_RELEASE)
        return;

    // shortcuts: undo/redo. not in the middle of a drag, it would go on from the state which was undone
    if (Global.ctrlState && (Key == GLFW_KEY_Z || Key == GLFW_KEY_Y) && is_press(Action) && (m_track_gizmo_using || m_track_set_using || m_straights.dragging || m_extend.active || m_switch.placing || m_point_drag.active || m_handle_drag.active))
        return;
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
            export_scenery();
        }
        break;

    case GLFW_KEY_F12:
        if (Global.ctrlState && Global.shiftState && is_press(Action))
        {
            DebugModeFlag = !DebugModeFlag;
        }
        break;

    case GLFW_KEY_DELETE:
        if (is_press(Action) && road_delete())
        {
            break;
        }
        if (is_press(Action) && m_instance != 0 && signal_delete(m_instance))
        {
            break;
        }
        if (is_press(Action) && m_instance != 0)
        {
            // include of a scenery template. what it shows goes out of sight, the directive is erased on save
            scene::Layers.removed(m_instance, true);
            EditorSnapshot snap;
            snap.instance = m_instance;
            snap.node_name = *scene::Layers.instance(m_instance).file;
            m_history.push_back(std::move(snap));
            g_redo.clear();
            ui()->set_status("Include of \"" + *scene::Layers.instance(m_instance).file + "\" removed, Ctrl+Z brings it back. Its directive is erased when the scenery is saved.");
            select_include(0);
            break;
        }
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
            else if (ui()->mode() == nodebank_panel::TRACK && m_track_tab == track_tab::path && m_track_set.size() >= 2)
            {
                track_set_delete();
            }
            else if (selected_track() != nullptr)
            {
                delete_selected_track();
            }
        }
        break;

    case GLFW_KEY_ESCAPE:
        if (is_press(Action))
        {
            road_cancel();
        }
        // the first Esc drops what is going on in the track mode, the next one goes back to the selection
        if (is_press(Action) && ui()->mode() == nodebank_panel::TRACK)
        {
            if (m_lay.active && false == m_lay.points.empty())
                lay_cancel();
            else if (track_busy())
            {
                cancel_track_tools();
                m_straights.tool = 0;
            }
            else if (m_track_tab == track_tab::path && false == m_track_set.empty())
                m_track_set.clear();
            else if (m_track_tab != track_tab::path)
                show_track_tab(track_tab::path);
            else if (selected_track() != nullptr)
            {
                m_node = nullptr;
                m_track_point = {};
                ui()->set_node(nullptr);
            }
        }
        break;

    case GLFW_KEY_BACKSPACE:
        // area fill: remove the last outline point
        if (is_press(Action) && ui()->mode() == nodebank_panel::FILL && !m_fill_points.empty())
            m_fill_points.pop_back();
        if (is_press(Action) && ui()->mode() == nodebank_panel::TRACK && m_lay.active && !m_lay.points.empty())
        {
            m_lay.points.pop_back();
            if (m_lay.points.empty())
                m_lay.start = {};
        }
        break;

    case GLFW_KEY_ENTER:
    case GLFW_KEY_KP_ENTER:
        if (is_press(Action) && ui()->mode() == nodebank_panel::TRACK && m_lay.active)
            lay_enter();
        break;

    case GLFW_KEY_K:
        if (is_press(Action) && road_split())
        {
            break;
        }
        if (is_press(Action) && ui()->mode() == nodebank_panel::TRACK && selected_track() != nullptr)
            split_track_at(*selected_track(), Global.pCamera.Pos + GfxRenderer->Mouse_Position());
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

    // with the road window open the left button works its tools, whichever edit mode is on but the track one
    if (m_roadtool.window && Button == GLFW_MOUSE_BUTTON_LEFT && ui()->mode() != nodebank_panel::TRACK)
    {
        if (is_press(Action))
        {
            // NOTE: the pick is resolved a few frames later, by then it's known whether the press was meant for the UI
            GfxRenderer->Pick_Node_Callback([this](scene::basic_node * /*node*/) {
                if (viewport_click())
                    road_click();
            });
        }
        m_input.mouse.button(Button, Action);
        return;
    }

    if (Button == GLFW_MOUSE_BUTTON_LEFT && is_press(Action) && false == m_bend.picking && ui()->mode() != nodebank_panel::TRACK && (bend_drag_start(Mods) || bend_click()))
    {
        m_input.mouse.button(Button, Action);
        return;
    }

    if (m_bend.picking && Button == GLFW_MOUSE_BUTTON_LEFT && ui()->mode() != nodebank_panel::TRACK)
    {
        if (is_press(Action))
            bend_pick(Global.pCamera.Pos + GfxRenderer->Mouse_Position());
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

            if (mode == nodebank_panel::TRACK)
            {
                track_press(track_intent_at(Mods));
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
                    select_include(0);

                    // in a scenery opened for editing selection has its limits. nodes of locked layers are off limits.
                    // nodes defined by a scenery template can't be rewritten one by one on save, so a click on one
                    // selects the include of the template as a whole
                    if (node && mode == nodebank_panel::MODIFY)
                    {
                        std::string reason;
                        if (node->from_template() && scene::Layers.removable(node->m_instance, &reason))
                        {
                            select_include(node->m_instance);
                            ui()->set_status("Include of \"" + *scene::Layers.instance(m_instance).file + "\" selected. Its parameters are in the node properties, Del removes it from the scenery.");
                            node = nullptr;
                        }
                        else if (node->from_template() || false == scene::Layers.editable(node, &reason))
                        {
                            ui()->set_status("\"" + (node->name().empty() ? std::string{"(unnamed node)"} : node->name()) + "\" can't be edited: " + reason, true);
                            node = nullptr;
                        }
                    }

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

                        if (src->starts_with(editor_includes::directive_mark))
                        {
                            // a scenery template rather than a single node
                            place_include(src->substr(editor_includes::directive_mark.size()), rotation_mode, fixed_rotation_value);
                            return;
                        }

                        std::string name = "editor_";
                        glm::dvec3 mouseOffset = clamp_mouse_offset_to_max(GfxRenderer->Mouse_Position());
                        TAnimModel *cloned = simulation::State.create_model(*src, name, placement_on_ground(Camera.Pos + mouseOffset));
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
            {
                mouseHold = false;
                finish_track_box();
                point_drag_finish();
                m_handle_drag = {};
                sweep_release();
                if (m_straights.tool_mouse)
                    finish_straight_gesture();
                if (m_extend.active)
                    finish_extend();
                if (m_switch.placing)
                    finish_switch_placement();
                if (m_turntable.placing)
                    turntable_finish_placement();
                if (m_signal.placing || m_signal.moving)
                    signal_release();
            }

            m_dragging = false;
        }
    }
    else if (Button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        if (ui()->mode() == nodebank_panel::TRACK)
        {
            if (is_press(Action))
                track_context_press();
            else if (is_release(Action))
                track_context_release();
        }
        // game-engine style look: hide & grab the cursor while flying, restore it on release
        Application.set_cursor(is_press(Action) ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
    }

    m_input.mouse.button(Button, Action);
}

void editor_mode::render_change_history(){


    ImGui::Begin(STR_C("Editor History"), &m_change_history, ImGuiWindowFlags_AlwaysAutoResize);
    int maxsize = m_max_history_size;
    if (ImGui::InputInt(STR_C("Max history size"), &maxsize))
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
    if (ImGui::InputFloat(STR_C("Max placement distance"), &dist))
    {
        kMaxPlacementDistance = std::max(0.0f, dist);
    }

    ImGui::Separator();

    ImGui::Text(STR_C("History (newest at end): %zu entries"), m_history.size());
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
                        s.action == EditorSnapshot::Action::Scale ? "SCA" :
                        s.action == EditorSnapshot::Action::TrackEdit ? "TRK" :
                        s.action == EditorSnapshot::Action::RoadEdit ? "ROAD" :
                        s.action == EditorSnapshot::Action::Array ? "ARR" : "OTH",
                        s.node_name.empty() ? "(noname)" : s.node_name.c_str(),
                        s.position.x, s.position.y, s.position.z);

        if (ImGui::Selectable(buf, m_selected_history_idx == i))
            m_selected_history_idx = i;
    }
    ImGui::EndChild();

    ImGui::Separator();
    if (ImGui::Button(STR_C("Clear History")))
    {
        m_history.clear();
        g_redo.clear();
        m_selected_history_idx = -1;
    }
    ImGui::SameLine();
   
    ImGui::SameLine();
    if (ImGui::Button(STR_C("Undo Selected")))
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

void editor_mode::starter_trainset(std::vector<std::string> &Trailing)
{
    static std::regex const trainset{R"((^|\n)[ \t]*trainset[ \t])", std::regex::icase};
    for (std::size_t handle = 1; handle <= scene::Layers.size(); ++handle)
    {
        auto const &layer{scene::Layers.layer(static_cast<scene::layer_handle>(handle))};
        if (layer.created)
            continue;
        std::ifstream input{Global.asCurrentSceneryPath + layer.name, std::ios_base::binary};
        std::string const content{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
        if (std::regex_search(content, trainset))
            return;
    }
    std::error_code error;
    if (false == std::filesystem::exists("dynamic/pkp/wmb10_v1/wmb10.mmd", error))
        return;
    for (auto const *track : simulation::Paths.sequence())
    {
        if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || (track->iCategoryFlag & 1) == 0 || track->name().empty() || track->m_paths.empty())
            continue;
        auto const length{geometry::bezier{track->m_paths.front()}.plan_length()};
        if (length < 15.0)
            continue;
        Trailing.push_back("trainset none " + track->name() + " " + to_string(std::min(length, 0.5 * length + 5.0), 2) + " 0");
        Trailing.push_back("node -1 0 wmb10-6457 dynamic pkp/wmb10_v1 wmb10-6457 wmb10 0 headdriver 0 0 enddynamic");
        Trailing.push_back("endtrainset");
        WriteLog("Editor: the scenery has no trainset, a WMB10 is put on " + track->name() + " to drive");
        return;
    }
}
