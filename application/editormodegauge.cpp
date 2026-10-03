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
#include "editor/editorGauge.hpp"
#include "editor/editorGeometry.hpp"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/editoroverlay.h"
#include "rendering/renderer.h"
#include "simulation/simulation.h"
#include "world/Track.h"
#include "imgui/imgui.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <optional>

using geometry::plan_of;

namespace
{

char const *const gauge_file{"editor_gauges.txt"};
std::size_t const gauge_triangle_limit{200000};

// samples of the route, found by the position in the plan
class corridor
{
public:
	struct local
	{
		std::size_t sample;
		double lateral; // positive to the left
		double height; // above the rail top
	};

	corridor(std::vector<editor_track::route_sample> const &Samples, double const Margin) : m_samples{Samples}, m_margin{Margin}
	{
		m_min = m_max = plan_of(Samples.front().position);
		for (auto const &sample : Samples)
		{
			m_min = glm::min(m_min, plan_of(sample.position));
			m_max = glm::max(m_max, plan_of(sample.position));
		}
		m_min -= glm::dvec2(Margin + cell);
		m_max += glm::dvec2(Margin + cell);
		m_columns = static_cast<int>((m_max.x - m_min.x) / cell) + 1;
		m_rows = static_cast<int>((m_max.y - m_min.y) / cell) + 1;
		m_cells.resize(static_cast<std::size_t>(m_columns) * m_rows);
		for (std::size_t i = 0; i < Samples.size(); ++i)
			m_cells[index(cell_of(plan_of(Samples[i].position)))].push_back(i);
	}

	static glm::dvec2 normal(editor_track::route_sample const &Sample)
	{
		return {-Sample.direction.y, Sample.direction.x};
	}

	// whether the sphere may reach the gauge
	bool near(glm::dvec3 const &Center, double const Radius) const
	{
		auto const center{plan_of(Center)};
		if (center.x + Radius < m_min.x || center.x - Radius > m_max.x || center.y + Radius < m_min.y || center.y - Radius > m_max.y)
			return false;
		auto const reach{(Radius + m_margin) * (Radius + m_margin)};
		return std::any_of(m_samples.begin(), m_samples.end(), [&](editor_track::route_sample const &Sample) { return glm::length2(plan_of(Sample.position) - center) <= reach; });
	}

	std::optional<local> locate(glm::dvec3 const &Point) const
	{
		auto const point{plan_of(Point)};
		auto const cell{cell_of(point)};
		std::size_t nearest{m_samples.size()};
		double best{m_margin * m_margin};
		for (int row = std::max(0, cell.y - 1); row <= std::min(m_rows - 1, cell.y + 1); ++row)
			for (int column = std::max(0, cell.x - 1); column <= std::min(m_columns - 1, cell.x + 1); ++column)
				for (auto const i : m_cells[index({column, row})])
				{
					auto const distance{glm::length2(plan_of(m_samples[i].position) - point)};
					if (distance < best)
					{
						best = distance;
						nearest = i;
					}
				}
		if (nearest == m_samples.size())
			return std::nullopt;
		auto const &sample{m_samples[nearest]};
		auto const offset{point - plan_of(sample.position)};
		auto const along{glm::dot(offset, sample.direction)};
		if ((nearest == 0 && along < -0.6) || (nearest + 1 == m_samples.size() && along > 0.6))
			return std::nullopt;
		return local{nearest, glm::dot(offset, normal(sample)), Point.y - (sample.position.y + sample.grade * along)};
	}

private:
	static constexpr double cell{8.0};

	glm::ivec2 cell_of(glm::dvec2 const &Point) const
	{
		return {std::clamp(static_cast<int>((Point.x - m_min.x) / cell), 0, m_columns - 1), std::clamp(static_cast<int>((Point.y - m_min.y) / cell), 0, m_rows - 1)};
	}
	std::size_t index(glm::ivec2 const &Cell) const
	{
		return static_cast<std::size_t>(Cell.y) * m_columns + Cell.x;
	}

	std::vector<editor_track::route_sample> const &m_samples;
	double m_margin;
	glm::dvec2 m_min, m_max;
	int m_columns{1}, m_rows{1};
	std::vector<std::vector<std::size_t>> m_cells;
};

gauge::section section_of(editor_track::route_sample const &Sample)
{
	gauge::section section;
	if (std::abs(Sample.curvature) > 1e-5)
	{
		section.radius = 1.0 / std::abs(Sample.curvature);
		section.inner = Sample.curvature > 0.0 ? 1 : -1;
	}
	section.cant = Sample.cant;
	return section;
}

glm::dmat4 placement(TAnimModel const &Instance)
{
	auto const angles{Instance.Angles()};
	auto matrix{glm::translate(glm::dmat4(1.0), Instance.location())};
	matrix = glm::rotate(matrix, glm::radians(static_cast<double>(angles.y)), glm::dvec3(0.0, 1.0, 0.0));
	matrix = glm::rotate(matrix, glm::radians(static_cast<double>(angles.x)), glm::dvec3(1.0, 0.0, 0.0));
	matrix = glm::rotate(matrix, glm::radians(static_cast<double>(angles.z)), glm::dvec3(0.0, 0.0, 1.0));
	return glm::scale(matrix, glm::dvec3(Instance.Scale()));
}

// calls the visitor with the world corners of the visible triangles of the submodel, its siblings and children
template <typename Visitor> void visit_triangles(TSubModel *Submodel, glm::dmat4 const &Matrix, Visitor const &Visit)
{
	for (auto *submodel = Submodel; submodel != nullptr; submodel = submodel->NextGet())
	{
		if (submodel->iVisible == 0)
			continue;
		auto matrix{Matrix};
		if ((submodel->iFlags & 0xC000) && submodel->GetMatrix() != nullptr)
			matrix *= glm::dmat4(glm::make_mat4(submodel->GetMatrix()->readArray()));
		if (submodel->eType == GL_TRIANGLES && submodel->m_geometry.handle != null_handle)
		{
			auto const &vertices{GfxRenderer->Vertices(submodel->m_geometry.handle)};
			auto const &indices{GfxRenderer->Indices(submodel->m_geometry.handle)};
			auto const corner{[&](std::size_t const Index) { return glm::dvec3(matrix * glm::dvec4(glm::dvec3(vertices[Index].position), 1.0)); }};
			auto const count{indices.empty() ? vertices.size() : indices.size()};
			for (std::size_t i = 0; i + 2 < count; i += 3)
			{
				if (indices.empty())
					Visit(corner(i), corner(i + 1), corner(i + 2));
				else
					Visit(corner(indices[i]), corner(indices[i + 1]), corner(indices[i + 2]));
			}
		}
		visit_triangles(submodel->ChildGet(), matrix, Visit);
	}
}

} // namespace

void editor_mode::gauge_load()
{
	if (m_gauge.profiles.empty())
		m_gauge.profiles = gauge::load_profiles(gauge_file);
	m_gauge.profile = std::clamp(m_gauge.profile, 0, static_cast<int>(m_gauge.profiles.size()) - 1);
}

void editor_mode::update_gauge()
{
	auto *track{m_gauge.enabled && ui()->mode() == nodebank_panel::TRACK ? selected_track() : nullptr};
	if (track == nullptr)
	{
		if (false == EditorOverlay.batches.empty())
			EditorOverlay.clear();
		m_gauge.track = nullptr;
		return;
	}
	// the geometry is rescanned once the edit ends
	if (ImGui::IsMouseDown(0) || m_track_gizmo_using || m_dragging)
	{
		m_gauge.pending = true;
		return;
	}
	if (track == m_gauge.track && m_history.size() == m_gauge.history && false == m_gauge.pending)
		return;
	m_gauge.track = track;
	m_gauge.history = m_history.size();
	m_gauge.pending = false;
	scan_gauge(*track);
}

void editor_mode::scan_gauge(TTrack &Track)
{
	gauge_load();
	EditorOverlay.clear();
	m_gauge.intrusions = 0;
	m_gauge.models = 0;
	auto route{editor_track::run_route(Track, m_gauge.reach)};
	auto const samples{editor_track::sample_route(route, 1.0)};
	auto const &profile{m_gauge.profiles[m_gauge.profile]};
	if (samples.size() < 2 || profile.outline.size() < 2)
		return;

	double widest{0.0}, top{0.0};
	for (auto const &point : profile.outline)
	{
		widest = std::max(widest, point.half_width);
		top = std::max(top, point.height);
	}
	corridor const space{samples, widest + gauge::curve_widening(150.0) + gauge::cant_widening(0.2, top) + 0.5};
	auto const origin{samples.front().position};
	EditorOverlay.origin = origin;
	auto const relative{[&](glm::dvec3 const &Point) { return glm::vec3(Point - origin); }};
	auto const lift{[](editor_track::route_sample const &Sample, double const Lateral, double const Height) {
		auto const normal{corridor::normal(Sample)};
		return Sample.position + glm::dvec3(normal.x * Lateral, Height, normal.y * Lateral);
	}};

	// outline across the track, from the bottom right up and down to the bottom left
	auto const ring{[&](editor_track::route_sample const &Sample) {
		auto const section{section_of(Sample)};
		std::vector<glm::vec3> points;
		for (int side : {-1, 1})
			for (std::size_t j = 0; j < profile.outline.size(); ++j)
			{
				auto const &point{profile.outline[side < 0 ? j : profile.outline.size() - 1 - j]};
				auto width{point.half_width + gauge::curve_widening(section.radius)};
				if (side == section.inner)
					width += gauge::cant_widening(section.cant, point.height);
				points.push_back(relative(lift(Sample, side * width, point.height)));
			}
		return points;
	}};

	gfx::editor_overlay::batch surface{{0.2f, 0.9f, 0.3f, 0.12f}, GL_TRIANGLES};
	gfx::editor_overlay::batch lines{{0.2f, 1.0f, 0.3f, 0.5f}, GL_LINES};
	gfx::editor_overlay::batch intruding{{1.0f, 0.1f, 0.05f, 0.6f}, GL_TRIANGLES, true};
	std::vector<glm::vec3> previous;
	double lastring{-1e9};
	for (std::size_t i = 0; i < samples.size(); i += 2)
	{
		auto const current{ring(samples[i])};
		if (false == previous.empty())
			for (std::size_t j = 1; j < current.size(); ++j)
			{
				surface.points.insert(surface.points.end(), {previous[j - 1], previous[j], current[j], previous[j - 1], current[j], current[j - 1]});
				lines.points.insert(lines.points.end(), {previous[j], current[j]});
			}
		if (samples[i].chainage - lastring >= 10.0 || i + 2 >= samples.size())
		{
			lastring = samples[i].chainage;
			for (std::size_t j = 1; j < current.size(); ++j)
				lines.points.insert(lines.points.end(), {current[j - 1], current[j]});
		}
		previous = current;
	}

	// triangle enters the gauge when any point of a grid spread over it does
	auto const inside{[&](glm::dvec3 const &Point) {
		auto const local{space.locate(Point)};
		return local && gauge::intrusion(profile, section_of(samples[local->sample]), local->lateral, local->height) > 0.0;
	}};
	for (auto *instance : simulation::Instances.sequence())
	{
		if (intruding.points.size() >= 3 * gauge_triangle_limit)
			break;
		if (instance == nullptr || false == instance->visible() || instance->Model() == nullptr || instance->Model()->GetSMRoot() == nullptr)
			continue;
		if (false == space.near(instance->location(), instance->radius()))
			continue;
		auto const before{intruding.points.size()};
		visit_triangles(instance->Model()->GetSMRoot(), placement(*instance), [&](glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C) {
			auto const edge{std::max({glm::distance(A, B), glm::distance(B, C), glm::distance(C, A)})};
			auto const steps{std::clamp(static_cast<int>(std::ceil(edge / 0.5)), 1, 24)};
			for (int u = 0; u <= steps; ++u)
				for (int v = 0; u + v <= steps; ++v)
				{
					auto const a{static_cast<double>(u) / steps};
					auto const b{static_cast<double>(v) / steps};
					if (inside(A + (B - A) * a + (C - A) * b))
					{
						intruding.points.insert(intruding.points.end(), {relative(A), relative(B), relative(C)});
						return;
					}
				}
		});
		if (intruding.points.size() > before)
			++m_gauge.models;
	}
	m_gauge.intrusions = intruding.points.size() / 3;
	EditorOverlay.batches = {std::move(surface), std::move(lines), std::move(intruding)};
}

void editor_mode::render_gauge_ui()
{
	if (false == ImGui::CollapsingHeader("Structure gauge"))
		return;
	if (ImGui::Checkbox("Show in the 3D view", &m_gauge.enabled))
		m_gauge.pending = true;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Translucent tunnel of the gauge along the line of the selected path,\nwith the triangles of the models which enter it in red. Widened in curves and on the inner side of a cant.");
	gauge_load();
	auto const previous{m_gauge.profile};
	for (int i = 0; i < static_cast<int>(m_gauge.profiles.size()); ++i)
	{
		if (i > 0)
			ImGui::SameLine();
		ImGui::RadioButton(m_gauge.profiles[i].name.c_str(), &m_gauge.profile, i);
	}
	bool changed{previous != m_gauge.profile};
	float reach{static_cast<float>(m_gauge.reach)};
	if (ImGui::SliderFloat("Reach each way (m)", &reach, 50.0f, 2000.0f, "%.0f"))
		m_gauge.reach = reach;
	changed |= ImGui::IsItemDeactivatedAfterEdit();
	if (m_gauge.enabled)
	{
		if (m_gauge.models > 0)
			ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%zu models enter the gauge (%zu triangles)", m_gauge.models, m_gauge.intrusions);
		else
			ImGui::TextDisabled("The gauge is clear");
	}

	if (ImGui::TreeNode("Outline (right half)"))
	{
		ImGui::TextDisabled("Approximate starting points, check against PN-69/K-02057.\nMetres from the track axis and above the rail top, bottom up.");
		auto &profile{m_gauge.profiles[m_gauge.profile]};
		bool edited{false};
		bool inserted{false};
		int remove{-1};
		ImGui::Columns(3, "gaugeoutline", false);
		ImGui::SetColumnWidth(0, 110.0f);
		ImGui::SetColumnWidth(1, 110.0f);
		ImGui::TextDisabled("Half-width");
		ImGui::NextColumn();
		ImGui::TextDisabled("Height");
		ImGui::NextColumn();
		ImGui::NextColumn();
		for (int i = 0; i < static_cast<int>(profile.outline.size()); ++i)
		{
			ImGui::PushID(i);
			auto &point{profile.outline[i]};
			ImGui::SetNextItemWidth(-1.0f);
			ImGui::InputDouble("##width", &point.half_width, 0.0, 0.0, "%.3f");
			edited |= ImGui::IsItemDeactivatedAfterEdit();
			ImGui::NextColumn();
			ImGui::SetNextItemWidth(-1.0f);
			ImGui::InputDouble("##height", &point.height, 0.0, 0.0, "%.3f");
			edited |= ImGui::IsItemDeactivatedAfterEdit();
			ImGui::NextColumn();
			if (ImGui::SmallButton("+"))
			{
				auto const &next{profile.outline[std::min<std::size_t>(i + 1, profile.outline.size() - 1)]};
				profile.outline.insert(profile.outline.begin() + i + 1, {0.5 * (point.half_width + next.half_width), 0.5 * (point.height + next.height)});
				inserted = edited = true;
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Insert a point after this one");
			ImGui::SameLine();
			if (profile.outline.size() > 2 && ImGui::SmallButton("x"))
				remove = i;
			ImGui::NextColumn();
			ImGui::PopID();
			if (inserted)
				break;
		}
		ImGui::Columns(1);
		if (remove >= 0)
		{
			profile.outline.erase(profile.outline.begin() + remove);
			edited = true;
		}
		if (ImGui::SmallButton("Restore the starting outlines"))
		{
			m_gauge.profiles = gauge::default_profiles();
			edited = true;
		}
		if (edited)
		{
			gauge::save_profiles(gauge_file, m_gauge.profiles);
			changed = true;
		}
		ImGui::TreePop();
	}
	if (changed)
		m_gauge.pending = true;
}
