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
#include "application/editorprojection.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"
#include "model/AnimModel.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <regex>
#include <sstream>

namespace
{

using geometry::arc_pieces;
using geometry::plan_distance;
using geometry::plan_of;
using geometry::signed_angle;

double constexpr kRimTolerance{0.05};
double constexpr kMinimumRadius{80.0};
float constexpr kDragPixels{5.0f};

glm::dvec2 heading_vector(double const Heading)
{
	return {std::sin(glm::radians(Heading)), std::cos(glm::radians(Heading))};
}

double heading_of(glm::dvec2 const &Direction)
{
	return glm::degrees(std::atan2(Direction.x, Direction.y));
}

double circular(double Angle)
{
	Angle = std::fmod(Angle, 360.0);
	if (Angle <= -180.0)
		Angle += 360.0;
	if (Angle > 180.0)
		Angle -= 360.0;
	return Angle;
}

double line_angle(double const Angle)
{
	auto result{circular(Angle)};
	if (result >= 90.0)
		result -= 180.0;
	if (result < -90.0)
		result += 180.0;
	return result;
}

bool numeric(std::string const &Text, double &Value)
{
	char *end{nullptr};
	Value = std::strtod(Text.c_str(), &end);
	return false == Text.empty() && *end == '\0';
}

std::string normalized(std::string File)
{
	std::replace(File.begin(), File.end(), '\\', '/');
	return ToLower(File);
}

std::vector<double> fixed_angles(std::string const &File)
{
	std::vector<double> result;
	std::ifstream input{Global.asCurrentSceneryPath + File, std::ios_base::binary};
	if (false == input.is_open())
		return result;
	std::string const content{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
	static std::regex const pattern{R"(event\s+\S*tor(\d+)rot\s+animation\s+\S+\s+\S+\s+rotate\s+\S+\s+\S+\s+\S+\s+(\S+))", std::regex::icase};
	for (auto match{std::sregex_iterator(content.begin(), content.end(), pattern)}; match != std::sregex_iterator(); ++match)
	{
		double angle;
		if (numeric((*match)[2].str(), angle) && std::none_of(result.begin(), result.end(), [&](double const Angle) { return std::abs(line_angle(Angle - angle)) < 0.01; }))
			result.push_back(angle);
	}
	return result;
}

glm::dvec3 centre_of(TTrack const &Table)
{
	auto const &path{Table.m_paths.front()};
	return glm::mix(path.points[segment_data::point::start], path.points[segment_data::point::end], 0.5);
}

double length_of(TTrack const &Table)
{
	auto const &path{Table.m_paths.front()};
	return plan_distance(path.points[segment_data::point::start], path.points[segment_data::point::end]);
}

struct rim_end
{
	double heading{0.0};
	TTrack *track{nullptr};
};

std::vector<rim_end> rim_ends(TTrack const *Table, glm::dvec3 const &Centre, double const Rim)
{
	std::vector<rim_end> result;
	for (auto *section : simulation::Region->sections(Centre, static_cast<float>(Rim + 10.0)))
		for (auto const &cell : section->m_cells)
			for (auto *other : cell.m_directories.paths)
			{
				if (other == Table || other->m_editorremoved || (other->iCategoryFlag & 15) != 1 || other->eType == tt_Table)
					continue;
				for (auto const &path : other->m_paths)
					for (auto const &point : {path.points[segment_data::point::start], path.points[segment_data::point::end]})
					{
						if (std::abs(plan_distance(point, Centre) - Rim) > kRimTolerance || std::abs(point.y - Centre.y) > 0.5)
							continue;
						auto const heading{heading_of(plan_of(point - Centre))};
						if (std::none_of(result.begin(), result.end(), [&](rim_end const &End) { return End.track == other && std::abs(circular(End.heading - heading)) < 0.01; }))
							result.push_back({heading, other});
					}
			}
	return result;
}

void draw_circle(screen_projection const &Projection, ImDrawList *Drawlist, glm::dvec3 const &Centre, double const Radius, ImU32 const Colour, float const Thickness)
{
	int const count{48};
	glm::dvec3 previous{Centre.x, Centre.y, Centre.z + Radius};
	for (int i = 1; i <= count; ++i)
	{
		auto const angle{glm::two_pi<double>() * i / count};
		glm::dvec3 const next{Centre.x + Radius * std::sin(angle), Centre.y, Centre.z + Radius * std::cos(angle)};
		Projection.line(Drawlist, previous, next, Colour, Thickness);
		previous = next;
	}
}

void draw_pieces(screen_projection const &Projection, ImDrawList *Drawlist, std::vector<segment_data> const &Pieces, ImU32 const Colour, float const Thickness)
{
	for (auto const &piece : Pieces)
	{
		geometry::bezier const curve{piece};
		auto const count{std::clamp(static_cast<int>(curve.plan_length() / 2.0), 2, 48)};
		auto previous{curve.point(0.0)};
		for (int k = 1; k <= count; ++k)
		{
			auto const next{curve.point(static_cast<double>(k) / count)};
			Projection.line(Drawlist, previous, next, Colour, Thickness);
			previous = next;
		}
	}
}

void label(screen_projection const &Projection, glm::dvec3 const &Point, std::string const &Text)
{
	ImVec2 at;
	if (false == Projection.project(Point, at))
		return;
	auto *drawlist{ImGui::GetForegroundDrawList(ImGui::GetMainViewport())};
	auto const size{ImGui::CalcTextSize(Text.c_str())};
	drawlist->AddRectFilled(ImVec2(at.x + 8.f, at.y - 4.f), ImVec2(at.x + 16.f + size.x, at.y + size.y + 4.f), IM_COL32(0, 0, 0, 170), 4.f);
	drawlist->AddText(ImVec2(at.x + 12.f, at.y), IM_COL32(255, 255, 255, 235), Text.c_str());
}

} // namespace

void editor_mode::scan_turntables()
{
	auto &tool{m_turntable};
	if (tool.scanned)
		return;
	tool.scanned = true;
	tool.templates.clear();
	if (false == EditorIncludes.scanned())
		EditorIncludes.scan();
	for (auto const &entry : EditorIncludes.entries())
	{
		if (entry.category != "turntable")
			continue;
		include_info info;
		std::string error;
		if (false == editor_includes::load(entry.file, info, error) || info.turntable_length <= 0.0)
			continue;
		turntable_template item;
		item.file = entry.file;
		item.name = info.name.empty() ? entry.file : info.name;
		item.length = info.turntable_length;
		item.track = editor_includes::parameter_with_role(info, "track");
		item.yaw = editor_includes::parameter_with_role(info, "rot.y");
		if (info.turntable_angles > 0 && info.turntable_positions > 0)
		{
			item.angles = info.turntable_angles;
			item.positions = info.turntable_positions;
		}
		else
			item.fixed = fixed_angles(entry.file);
		if (item.track == 0 || item.yaw == 0 || (item.angles == 0 && item.fixed.empty()))
			continue;
		tool.templates.push_back(std::move(item));
	}
	std::stable_sort(tool.templates.begin(), tool.templates.end(), [](turntable_template const &A, turntable_template const &B) { return (A.angles > 0) != (B.angles > 0) ? A.angles > 0 : A.name < B.name; });
	tool.chosen = tool.templates.empty() ? -1 : 0;
}

std::string editor_mode::turntable_name_for_new() const
{
	if (m_turntable.name[0] != '\0')
		return ToLower(m_turntable.name);
	for (int i = 1;; ++i)
	{
		auto const name{"obr" + std::to_string(i)};
		if (simulation::Paths.find(name) == nullptr && simulation::Instances.find(name) == nullptr)
			return name;
	}
}

bool editor_mode::turntable_read(TTrack *Table)
{
	auto &tool{m_turntable};
	if (Table == nullptr || Table->eType != tt_Table || Table->m_editorremoved || Table->m_paths.empty())
	{
		tool.table = nullptr;
		tool.include = 0;
		tool.kind = -1;
		tool.exits.clear();
		tool.fits.clear();
		return false;
	}
	if (tool.table != Table)
	{
		tool.fits.clear();
		tool.hovered = -1;
		tool.status.clear();
		tool.error.clear();
	}
	scan_turntables();
	tool.table = Table;
	tool.length = length_of(*Table);
	auto const &path{Table->m_paths.front()};
	tool.table_yaw = heading_of(plan_of(path.points[segment_data::point::start] - centre_of(*Table)));
	tool.include = 0;
	tool.kind = -1;
	tool.exits.clear();
	for (scene::instance_handle handle = 1; handle <= scene::Layers.instance_count() && tool.include == 0; ++handle)
	{
		if (false == scene::Layers.tracked(handle))
			continue;
		auto const &included{scene::Layers.instance(handle)};
		if (included.removed || included.dead || included.file == nullptr)
			continue;
		auto const file{normalized(*included.file)};
		auto const kind{std::find_if(tool.templates.begin(), tool.templates.end(), [&](turntable_template const &Template) { return normalized(Template.file) == file; })};
		if (kind == tool.templates.end())
			continue;
		std::string target;
		std::vector<std::string> values;
		if (false == editor_includes::parse_directive(scene::Layers.directive(handle), target, values) || kind->track > static_cast<int>(values.size()) || ToLower(values[kind->track - 1]) != Table->name())
			continue;
		tool.include = handle;
		tool.kind = static_cast<int>(kind - tool.templates.begin());
		if (double yaw; kind->yaw <= static_cast<int>(values.size()) && numeric(values[kind->yaw - 1], yaw))
			tool.table_yaw = yaw;
		if (kind->angles > 0)
		{
			for (int i = 0; i < kind->positions && kind->angles - 1 + i < static_cast<int>(values.size()); ++i)
				if (double angle; numeric(values[kind->angles - 1 + i], angle) && std::none_of(tool.exits.begin(), tool.exits.end(), [&](double const Exit) { return std::abs(line_angle(Exit - angle)) < 0.01; }))
					tool.exits.push_back(angle);
		}
		else
			tool.exits = kind->fixed;
	}
	return true;
}

glm::dvec3 editor_mode::turntable_centre() const
{
	return m_turntable.table != nullptr ? centre_of(*m_turntable.table) : glm::dvec3{0.0};
}

std::vector<double> editor_mode::turntable_lines() const
{
	std::vector<double> result;
	for (auto const angle : m_turntable.exits)
		result.push_back(m_turntable.table_yaw + angle);
	return result;
}

double editor_mode::turntable_snap(double const Heading) const
{
	auto const &tool{m_turntable};
	auto const fixed{tool.kind >= 0 && tool.templates[tool.kind].angles == 0};
	std::optional<double> nearest;
	for (auto const line : turntable_lines())
		for (auto const heading : {line, line + 180.0})
			if (false == nearest.has_value() || std::abs(circular(heading - Heading)) < std::abs(circular(*nearest - Heading)))
				nearest = heading;
	if (nearest.has_value() && (fixed || std::abs(circular(*nearest - Heading)) < 1.0))
		return circular(*nearest);
	auto const step{tool.step > 0.f ? static_cast<double>(tool.step) : 0.1};
	return circular(tool.table_yaw + std::round(circular(Heading - tool.table_yaw) / step) * step);
}

bool editor_mode::turntable_exit_at(glm::dvec3 const &Ground, double &Heading, double &Length) const
{
	auto const &tool{m_turntable};
	if (tool.table == nullptr)
		return false;
	auto const centre{turntable_centre()};
	auto const rim{tool.length * 0.5};
	auto const distance{plan_distance(Ground, centre)};
	if (distance < rim + 1.0 || distance > rim + tool.reach)
		return false;
	Heading = turntable_snap(heading_of(plan_of(Ground - centre)));
	Length = std::max(5.0, std::round((distance - rim) * 2.0) * 0.5);
	for (auto const &end : rim_ends(tool.table, centre, rim))
		if (std::abs(circular(end.heading - Heading)) < 0.05)
			return false;
	return true;
}

bool editor_mode::turntable_fit_end(editor_track::snap_target const &End, turntable_fit &Fit) const
{
	auto const &tool{m_turntable};
	Fit = {};
	Fit.end = End;
	auto const centre{turntable_centre()};
	auto const rim{tool.length * 0.5};
	glm::dvec2 const c{plan_of(centre)};
	auto const into{plan_of(End.direction)};
	if (tool.table == nullptr || glm::length(into) < 1e-6)
		return false;
	glm::dvec2 const t{glm::normalize(into)};
	auto target{End.position};
	glm::dvec2 e{plan_of(target)};
	struct shape
	{
		glm::dvec2 radial{0.0};
		double radius{0.0};
		double first{0.0};
		double turn{0.0};
		double second{0.0};
	};
	auto const connect = [&](glm::dvec2 const &Radial, double const Radius) -> std::optional<shape> {
		auto const rimpoint{c + Radial * rim};
		auto const delta{signed_angle(Radial, t)};
		auto const offset{e - rimpoint};
		if (std::abs(delta) < 1e-5)
		{
			auto const along{glm::dot(offset, Radial)};
			if (along < 0.5 || std::abs(geometry::cross(Radial, offset)) > 0.02)
				return std::nullopt;
			return shape{Radial, 0.0, along, 0.0, 0.0};
		}
		if (std::abs(delta) > glm::radians(90.0))
			return std::nullopt;
		auto const a{geometry::cross(t, offset) / geometry::cross(t, Radial)};
		auto const s{geometry::cross(Radial, offset) / geometry::cross(Radial, t)};
		auto const tangent{Radius * std::tan(std::abs(delta) * 0.5)};
		if (a - tangent < -1e-3 || s - tangent < -1e-3)
			return std::nullopt;
		return shape{Radial, Radius, std::max(0.0, a - tangent), delta, std::max(0.0, s - tangent)};
	};
	std::optional<shape> best;
	std::string issue;
	auto const better = [&](shape const &Candidate) {
		if (false == best.has_value())
			return true;
		auto const straight = [](shape const &Shape) { return Shape.radius == 0.0; };
		if (straight(Candidate) != straight(*best))
			return straight(Candidate);
		return Candidate.radius > best->radius + 1e-6;
	};
	auto const solve = [&]() {
		auto const fixed{tool.kind >= 0 && tool.templates[tool.kind].angles == 0};
		if (fixed)
		{
			for (auto const line : turntable_lines())
				for (auto const heading : {line, line + 180.0})
				{
					auto const radial{heading_vector(heading)};
					for (int i = 0; tool.radius * std::pow(0.9, i) >= kMinimumRadius; ++i)
						if (auto const candidate{connect(radial, tool.radius * std::pow(0.9, i))})
						{
							if (better(*candidate))
								best = candidate;
							break;
						}
				}
			if (false == best.has_value())
				issue = STR("It doesn't line up with any position of the turntable");
		}
		else if (std::abs(geometry::cross(t, c - e)) < 0.02 && glm::dot(t, e - c) > rim + 0.5)
		{
			best = connect(t, 0.0);
		}
		else
		{
			for (int r = 0; tool.radius * std::pow(0.9, r) >= kMinimumRadius && false == best.has_value(); ++r)
			{
				auto const radius{tool.radius * std::pow(0.9, r)};
				auto const gap = [&](double const S) {
					auto const vertex{e - t * S};
					auto const radial{vertex - c};
					if (glm::length(radial) < 1e-6)
						return -1.0;
					auto const delta{signed_angle(glm::normalize(radial), t)};
					return S - radius * std::tan(std::abs(delta) * 0.5);
				};
				auto const limit{plan_distance(End.position, centre) + 400.0};
				double low{0.0};
				std::optional<double> high;
				for (int i = 1; i * 0.25 <= limit; ++i)
				{
					auto const s{i * 0.25};
					if (gap(s) >= 0.0)
					{
						high = s;
						break;
					}
					low = s;
				}
				if (false == high.has_value())
					continue;
				auto upper{*high};
				for (int i = 0; i < 50; ++i)
				{
					auto const middle{(low + upper) * 0.5};
					(gap(middle) >= 0.0 ? upper : low) = middle;
				}
				auto const vertex{e - t * upper};
				if (glm::length(vertex - c) <= rim)
					continue;
				best = connect(glm::normalize(vertex - c), radius);
			}
			if (false == best.has_value())
				issue = format(STR_C("Too close to the turntable for a curve of %.0f m or more"), kMinimumRadius);
		}
	};
	solve();
	auto *own{End.track};
	if (false == best.has_value() && own != nullptr && own->eType == tt_Normal && own->m_paths.size() == 1 && own->Dynamics.empty())
	{
		auto const &path{own->m_paths.front()};
		auto const opposite{End.point.kind == editor_track::point_kind::start ? path.points[segment_data::point::end] : path.points[segment_data::point::start]};
		auto const straight{std::abs(geometry::cross(t, plan_of(opposite - End.position))) < 0.01 && glm::dot(t, plan_of(opposite - End.position)) > 1.0};
		std::string reason;
		if (straight && editor_track::can_edit_geometry(*own, reason))
		{
			auto const first{issue};
			target = opposite;
			e = plan_of(target);
			solve();
			if (best.has_value())
				Fit.replaces = own;
			else
			{
				target = End.position;
				e = plan_of(target);
				issue = first;
			}
		}
	}
	if (false == best.has_value())
	{
		Fit.issue = issue.empty() ? STR("It points away from the turntable") : issue;
		return false;
	}
	auto const &shape{*best};
	Fit.heading = heading_of(shape.radial);
	Fit.radius = shape.radius;
	Fit.length = shape.first + shape.radius * std::abs(shape.turn) + shape.second;
	auto const grade{Fit.length > 1e-6 ? (target.y - centre.y) / Fit.length : 0.0};
	glm::dvec3 point{c.x + shape.radial.x * rim, centre.y, c.y + shape.radial.y * rim};
	auto const straight = [&](glm::dvec2 const &Direction, double const Length) {
		segment_data piece;
		piece.points[segment_data::point::start] = point;
		point = {point.x + Direction.x * Length, point.y + grade * Length, point.z + Direction.y * Length};
		piece.points[segment_data::point::end] = point;
		Fit.pieces.push_back(piece);
	};
	if (shape.first > 0.05)
		straight(shape.radial, shape.first);
	if (shape.radius > 0.0)
	{
		auto const arc{arc_pieces(point, shape.radial, grade, shape.radius, std::abs(shape.turn), shape.turn > 0.0 ? 1 : -1)};
		Fit.pieces.insert(Fit.pieces.end(), arc.begin(), arc.end());
		point = arc.back().points[segment_data::point::end];
	}
	if (shape.second > 0.05)
		straight(t, shape.second);
	if (false == Fit.pieces.empty())
		Fit.pieces.back().points[segment_data::point::end] = target;
	return false == Fit.pieces.empty();
}

void editor_mode::turntable_find_fits()
{
	auto &tool{m_turntable};
	tool.fits.clear();
	tool.hovered = -1;
	tool.error.clear();
	if (tool.table == nullptr)
		return;
	auto const centre{turntable_centre()};
	auto const rim{tool.length * 0.5};
	for (auto const &end : editor_track::free_ends(nullptr, 1, centre, rim + tool.reach, {tool.table}))
	{
		if (std::abs(plan_distance(end.position, centre) - rim) <= kRimTolerance)
			continue;
		turntable_fit fit;
		turntable_fit_end(end, fit);
		fit.chosen = fit.issue.empty();
		tool.fits.push_back(std::move(fit));
	}
	std::sort(tool.fits.begin(), tool.fits.end(), [](turntable_fit const &A, turntable_fit const &B) { return A.end.distance < B.end.distance; });
	tool.status = tool.fits.empty() ? format(STR_C("No free ends within %.0f m of the rim"), tool.reach) : std::string{};
}

std::vector<scene::instance_handle> editor_mode::turntable_includes(TTrack const &Table)
{
	scan_turntables();
	std::vector<scene::instance_handle> result;
	for (scene::instance_handle handle = 1; handle <= scene::Layers.instance_count(); ++handle)
	{
		if (false == scene::Layers.tracked(handle))
			continue;
		auto const &included{scene::Layers.instance(handle)};
		if (included.removed || included.dead || included.file == nullptr)
			continue;
		auto const kind{std::find_if(m_turntable.templates.begin(), m_turntable.templates.end(), [&](turntable_template const &Template) { return normalized(Template.file) == normalized(*included.file); })};
		std::string target;
		std::vector<std::string> values;
		if (kind != m_turntable.templates.end() && editor_includes::parse_directive(scene::Layers.directive(handle), target, values) && kind->track <= static_cast<int>(values.size()) && ToLower(values[kind->track - 1]) == Table.name())
			result.push_back(handle);
	}
	return result;
}

void editor_mode::turntable_build(std::vector<turntable_fit const *> const &Fits)
{
	auto &tool{m_turntable};
	if (tool.table == nullptr || Fits.empty())
		return;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	for (auto const *fit : Fits)
	{
		if (false == fit->issue.empty() || fit->end.track == nullptr || fit->end.track->m_editorremoved)
			continue;
		for (auto const &piece : fit->pieces)
			created.push_back(editor_track::create_path(*fit->end.track, piece));
		if (fit->replaces != nullptr && false == fit->replaces->m_editorremoved)
		{
			editor_track::retire(*fit->replaces);
			removed.push_back(fit->replaces);
		}
	}
	if (created.empty())
		return;
	editor_track::commit(created);
	push_track_snapshot({}, created, removed);
	turntable_store_exits();
	tool.status = format(STR_C("%d track(s) led to the turntable"), static_cast<int>(Fits.size()));
}

void editor_mode::turntable_store_exits()
{
	auto &tool{m_turntable};
	if (tool.table == nullptr || tool.include == 0 || tool.kind < 0)
	{
		turntable_read(tool.table);
		return;
	}
	auto const &kind{tool.templates[tool.kind]};
	if (kind.angles == 0)
	{
		turntable_read(tool.table);
		return;
	}
	std::vector<double> lines;
	for (auto const &end : rim_ends(tool.table, turntable_centre(), tool.length * 0.5))
	{
		auto const angle{line_angle(end.heading - tool.table_yaw)};
		if (std::none_of(lines.begin(), lines.end(), [&](double const Line) { return std::abs(line_angle(Line - angle)) < 0.01; }))
			lines.push_back(angle);
	}
	std::sort(lines.begin(), lines.end());
	if (static_cast<int>(lines.size()) > kind.positions)
		tool.error = format(STR_C("The turntable has %d positions, %d track lines lead to it: the rest won't be reached"), kind.positions, static_cast<int>(lines.size()));
	std::string target;
	std::vector<std::string> values;
	if (false == editor_includes::parse_directive(scene::Layers.directive(tool.include), target, values))
		return;
	for (int i = 0; i < kind.positions && kind.angles - 1 + i < static_cast<int>(values.size()); ++i)
		values[kind.angles - 1 + i] = editor_includes::number(i < static_cast<int>(lines.size()) ? lines[i] : lines.empty() ? 0.0 : lines.front());
	auto const directive{editor_includes::compose_directive(target, values)};
	auto before{scene::Layers.directive(tool.include)};
	if (directive != before)
	{
		EditorSnapshot snap;
		snap.node_name = target;
		snap.directives.emplace_back(tool.include, std::move(before));
		set_include_directive(tool.include, directive);
		trim_history();
		m_history.push_back(std::move(snap));
		g_redo.clear();
	}
	turntable_read(tool.table);
}

void editor_mode::turntable_place(glm::dvec3 const &Centre, double const Yaw)
{
	auto &tool{m_turntable};
	scan_turntables();
	tool.error.clear();
	if (tool.chosen < 0 || tool.chosen >= static_cast<int>(tool.templates.size()))
		return;
	auto const &kind{tool.templates[tool.chosen]};
	auto const name{turntable_name_for_new()};
	if (simulation::Paths.find(name) != nullptr)
	{
		tool.error = format(STR_C("The name %s is taken by another path"), name.c_str());
		return;
	}
	auto const rim{kind.length * 0.5};
	auto const direction{heading_vector(Yaw)};
	segment_data path;
	path.points[segment_data::point::start] = {Centre.x + direction.x * rim, Centre.y, Centre.z + direction.y * rim};
	path.points[segment_data::point::end] = {Centre.x - direction.x * rim, Centre.y, Centre.z - direction.y * rim};
	auto *table{editor_track::create_turntable(path, tool.snap.track, name)};
	editor_track::commit({table});
	push_track_snapshot({}, {table});
	template_item item;
	item.file = kind.file;
	item.location = Centre;
	item.yaw = Yaw;
	item.described = true;
	item.track = table->name();
	std::string error;
	if (place_templates({item}, error) == 0)
		tool.error = error;
	tool.name[0] = '\0';
	m_node = table;
	m_track_point = {};
	ui()->set_node(m_node);
	turntable_read(table);
	tool.status = format(STR_C("Turntable %s: click around it for the tracks out of it, or fit the tracks around. It turns after the scenery is saved and loaded again"), table->name().c_str());
}

void editor_mode::turntable_press(track_intent const &Intent)
{
	using kind = track_intent::kind;
	auto &tool{m_turntable};
	auto const &io{ImGui::GetIO()};
	switch (Intent.what)
	{
	case kind::turntable_place:
		tool.placing = true;
		tool.pressed = {io.MousePos.x, io.MousePos.y};
		tool.snap = Intent.snap;
		tool.centre = Intent.position;
		return;
	case kind::turntable_exit:
	{
		if (tool.table == nullptr)
			return;
		auto const centre{turntable_centre()};
		auto const rim{tool.length * 0.5};
		auto const direction{heading_vector(tool.heading)};
		segment_data piece;
		piece.points[segment_data::point::start] = {centre.x + direction.x * rim, centre.y, centre.z + direction.y * rim};
		piece.points[segment_data::point::end] = {centre.x + direction.x * (rim + tool.exit_length), centre.y, centre.z + direction.y * (rim + tool.exit_length)};
		auto const ends{rim_ends(tool.table, centre, rim)};
		auto *track{ends.empty() ? editor_track::create_path(m_lay.style, piece) : editor_track::create_path(*ends.front().track, piece)};
		editor_track::commit({track});
		push_track_snapshot({}, {track});
		turntable_store_exits();
		tool.status = format(STR_C("Track out of the turntable at %.1f deg, %.1f m"), circular(tool.heading - tool.table_yaw), tool.exit_length);
		return;
	}
	case kind::turntable_fit:
	{
		turntable_fit fit;
		if (turntable_fit_end(Intent.snap, fit))
			turntable_build({&fit});
		else
			tool.error = fit.issue;
		return;
	}
	default: return;
	}
}

void editor_mode::turntable_finish_placement()
{
	auto &tool{m_turntable};
	if (false == tool.placing)
		return;
	tool.placing = false;
	scan_turntables();
	if (tool.chosen < 0)
		return;
	auto const rim{tool.templates[tool.chosen].length * 0.5};
	auto const &io{ImGui::GetIO()};
	auto const dragged{glm::length(glm::vec2{io.MousePos.x, io.MousePos.y} - tool.pressed) >= kDragPixels};
	if (tool.snap.track != nullptr && false == dragged)
	{
		auto const into{glm::normalize(plan_of(tool.snap.direction))};
		glm::dvec3 const centre{tool.snap.position.x - into.x * rim, tool.snap.position.y, tool.snap.position.z - into.y * rim};
		tool.yaw = heading_of(into);
		turntable_place(centre, tool.yaw);
		return;
	}
	tool.snap = {};
	if (dragged)
	{
		auto const cursor{cursor_level(tool.centre.y)};
		if (plan_distance(cursor, tool.centre) > 0.5)
			tool.yaw = std::round(heading_of(plan_of(cursor - tool.centre)) * 10.0) / 10.0;
	}
	turntable_place(tool.centre, tool.yaw);
}

editor_mode::track_intent editor_mode::turntable_intent(glm::dvec3 const &Ground, bool const Shift)
{
	using kind = track_intent::kind;
	auto &tool{m_turntable};
	track_intent intent;
	intent.position = Ground;
	auto const make = [&](kind const What, std::string const &Action) {
		intent.what = What;
		intent.action = Action;
		return intent;
	};
	auto const &io{ImGui::GetIO()};
	glm::vec2 const mouse{io.MousePos.x, io.MousePos.y};
	if (tool.table != selected_track())
		turntable_read(selected_track());
	if (tool.table != nullptr)
	{
		auto const centre{turntable_centre()};
		auto const rim{tool.length * 0.5};
		auto const end{snap_free_end(Ground, mouse, nullptr, 1, {tool.table})};
		if (end.track != nullptr && std::abs(plan_distance(end.position, centre) - rim) > kRimTolerance && plan_distance(end.position, centre) <= rim + tool.reach)
		{
			intent.snap = end;
			intent.track = end.track;
			intent.position = end.position;
			return make(kind::turntable_fit, STR("Click: lead this track to the turntable"));
		}
		double heading, length;
		if (turntable_exit_at(cursor_level(centre.y), heading, length))
		{
			tool.heading = heading;
			tool.exit_length = length;
			auto const direction{heading_vector(heading)};
			intent.position = {centre.x + direction.x * rim, centre.y, centre.z + direction.y * rim};
			return make(kind::turntable_exit, format(STR_C("Click: a track out of the turntable, %.1f deg, %.1f m"), circular(heading - tool.table_yaw), length));
		}
	}
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->eType != tt_Table || track->m_editorremoved || track->m_paths.empty())
			continue;
		if (plan_distance(centre_of(*track), Ground) <= length_of(*track) * 0.5)
		{
			intent.track = track;
			return make(kind::select, track == tool.table ? std::string{} : STR("Click: this turntable"));
		}
	}
	scan_turntables();
	if (tool.chosen < 0)
		return intent;
	if (false == Shift)
	{
		auto const end{snap_free_end(Ground, mouse, nullptr, 1, {})};
		if (end.track != nullptr)
		{
			intent.snap = end;
			intent.position = end.position;
			return make(kind::turntable_place, STR("Click: a turntable at the end of this track"));
		}
	}
	return make(kind::turntable_place, STR("Press: the centre of a new turntable, drag: the direction of its bridge"));
}

void editor_mode::render_turntable_ui()
{
	auto &tool{m_turntable};
	scan_turntables();
	auto *selected{selected_track()};
	if (tool.table != selected)
		turntable_read(selected);
	ImVec4 const warncolour{1.f, 0.75f, 0.3f, 1.f};

	if (tool.table != nullptr)
	{
		auto &table{*tool.table};
		ImGui::Text(STR_C("Turntable %s, bridge %.1f m"), table.name().c_str(), tool.length);
		if (tool.kind >= 0)
		{
			auto const &kind{tool.templates[tool.kind]};
			ImGui::TextDisabled("%s", kind.name.c_str());
			ImGui::SameLine();
			ImGui::TextDisabled(kind.angles > 0 ? STR_C("(positions set by the tracks around, up to %d)") : STR_C("(%d positions fixed by the template)"), kind.angles > 0 ? kind.positions : static_cast<int>(kind.fixed.size()));
		}
		else
			ImGui::TextColored(warncolour, "%s", STR_C("No turntable template takes this path as its track: the positions aren't known"));
		render_track_name(table);

		auto const centre{turntable_centre()};
		auto const ends{rim_ends(&table, centre, tool.length * 0.5)};
		if (false == tool.exits.empty() && ImGui::TreeNodeEx(format(STR_C("Positions (%d)"), static_cast<int>(tool.exits.size())).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
		{
			for (std::size_t i = 0; i < tool.exits.size(); ++i)
			{
				auto const heading{tool.table_yaw + tool.exits[i]};
				auto const occupied = [&](double const Heading) { return std::any_of(ends.begin(), ends.end(), [&](rim_end const &End) { return std::abs(circular(End.heading - Heading)) < 0.05; }); };
				auto const front{occupied(heading)};
				auto const back{occupied(heading + 180.0)};
				ImGui::Text("%2d  %7.2f deg", static_cast<int>(i + 1), tool.exits[i]);
				ImGui::SameLine();
				if (front || back)
					ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.5f, 1.f), "%s%s%s", front ? STR_C("track") : "", front && back ? " + " : "", back ? STR_C("track on the other end") : "");
				else
					ImGui::TextDisabled("%s", STR_C("no track"));
			}
			ImGui::TreePop();
		}

		ImGui::Separator();
		ImGui::TextUnformatted(STR_C("Tracks around"));
		ImGui::PushItemWidth(110.f);
		ImGui::InputFloat(STR_C("Reach from the rim, m"), &tool.reach, 10.f, 50.f, "%.0f");
		tool.reach = std::clamp(tool.reach, 5.f, 500.f);
		ImGui::InputFloat(STR_C("Radius of the curves, m"), &tool.radius, 10.f, 50.f, "%.0f");
		tool.radius = std::clamp(tool.radius, static_cast<float>(kMinimumRadius), 2000.f);
		if (tool.kind < 0 || tool.templates[tool.kind].angles > 0)
		{
			ImGui::InputFloat(STR_C("Angle step of a new track, deg (0: free)"), &tool.step, 0.5f, 2.5f, "%.2f");
			tool.step = std::clamp(tool.step, 0.f, 45.f);
		}
		ImGui::PopItemWidth();
		ImGui::TextDisabled("%s", STR_C("Click around the turntable: a track out of it, as long as the cursor is far.\nClick a free end near it: that track is led to the rim, straight or with a curve"));
		if (ImGui::Button(STR_C("Find the free ends around")))
			turntable_find_fits();
		if (false == tool.fits.empty())
		{
			int chosen{0};
			for (auto const &fit : tool.fits)
				chosen += fit.chosen && fit.issue.empty() ? 1 : 0;
			ImGui::SameLine();
			if (ImGui::Button(format(STR_C("Lead the chosen to the turntable (%d)"), chosen).c_str()))
			{
				std::vector<turntable_fit const *> fits;
				for (auto const &fit : tool.fits)
					if (fit.chosen && fit.issue.empty())
						fits.push_back(&fit);
				turntable_build(fits);
				turntable_find_fits();
			}
			tool.hovered = -1;
			ImGui::BeginChild("##turntablefits", ImVec2(0.f, 180.f), true);
			for (int i = 0; i < static_cast<int>(tool.fits.size()); ++i)
			{
				auto &fit{tool.fits[i]};
				ImGui::PushID(i);
				if (fit.issue.empty())
					ImGui::Checkbox("##chosen", &fit.chosen);
				else
					ImGui::TextDisabled("  -  ");
				ImGui::SameLine();
				auto const name{fit.end.track->name().empty() ? std::string{"(noname)"} : fit.end.track->name()};
				if (ImGui::Selectable(name.c_str(), false, ImGuiSelectableFlags_None, ImVec2(ImGui::CalcTextSize(name.c_str()).x, 0.f)))
					focus_track(*fit.end.track, fit.end.point.path, fit.end.position);
				if (ImGui::IsItemHovered())
					tool.hovered = i;
				ImGui::SameLine();
				if (false == fit.issue.empty())
					ImGui::TextColored(warncolour, "%s", fit.issue.c_str());
				else if (fit.radius > 0.0)
					ImGui::TextDisabled(STR_C("%.1f deg, curve R %.0f m, %.1f m"), circular(fit.heading - tool.table_yaw), fit.radius, fit.length);
				else
					ImGui::TextDisabled(STR_C("%.1f deg, straight %.1f m"), circular(fit.heading - tool.table_yaw), fit.length);
				if (fit.replaces != nullptr)
				{
					ImGui::SameLine();
					ImGui::TextDisabled(STR_C("in place of its last straight %s"), fit.replaces->name().c_str());
				}
				ImGui::PopID();
			}
			ImGui::EndChild();
		}
	}
	else
	{
		if (selected != nullptr && selected->eType == tt_Table)
			ImGui::TextColored(warncolour, "%s", STR_C("This turntable can't be read"));
		ImGui::TextUnformatted(STR_C("New turntable"));
		if (tool.templates.empty())
			ImGui::TextColored(warncolour, "%s", STR_C("No turntables: no template is described with category: turntable and its turntable: {length}"));
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		{
			auto const &kind{tool.templates[i]};
			auto const text{kind.angles > 0 ? format(STR_C("%s   %.0f m, up to %d positions set by the tracks"), kind.name.c_str(), kind.length, kind.positions)
			                                : format(STR_C("%s   %.0f m, %d fixed positions"), kind.name.c_str(), kind.length, static_cast<int>(kind.fixed.size()))};
			if (ImGui::Selectable((text + "##turntable" + std::to_string(i)).c_str(), tool.chosen == i))
				tool.chosen = i;
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", kind.file.c_str());
		}
		ImGui::SetNextItemWidth(160.f);
		ImGui::InputTextWithHint(STR_C("Name##turntablename"), turntable_name_for_new().c_str(), tool.name, sizeof(tool.name), ImGuiInputTextFlags_CharsNoBlank);
		ImGui::TextDisabled("%s", STR_C("Press where the centre goes and drag to turn the bridge.\nClick a free end of a track: the turntable lines up with it, at its end. Shift: no lining up"));
	}
	if (false == tool.error.empty())
		ImGui::TextColored(ImVec4(1.f, 0.45f, 0.35f, 1.f), "%s", tool.error.c_str());
	else if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void editor_mode::draw_turntable_overlay() const
{
	using kind = track_intent::kind;
	auto const &tool{m_turntable};
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const camera{glm::dvec3{Global.pCamera.Pos}};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->eType != tt_Table || track->m_editorremoved || track->m_paths.empty() || track == tool.table)
			continue;
		auto const centre{centre_of(*track)};
		if (glm::distance(centre, camera) < 1500.0)
			draw_circle(projection, drawlist, centre, length_of(*track) * 0.5, IM_COL32(255, 255, 255, 90), 1.5f);
	}
	if (tool.table != nullptr)
	{
		auto const centre{turntable_centre()};
		auto const rim{tool.length * 0.5};
		draw_circle(projection, drawlist, centre, rim, overlay_color::selected, 2.5f);
		auto const ends{rim_ends(tool.table, centre, rim)};
		for (auto const line : turntable_lines())
			for (auto const heading : {line, line + 180.0})
			{
				auto const direction{heading_vector(heading)};
				auto const occupied{std::any_of(ends.begin(), ends.end(), [&](rim_end const &End) { return std::abs(circular(End.heading - heading)) < 0.05; })};
				glm::dvec3 const from{centre.x + direction.x * rim, centre.y, centre.z + direction.y * rim};
				glm::dvec3 const to{centre.x + direction.x * (rim + 6.0), centre.y, centre.z + direction.y * (rim + 6.0)};
				projection.line(drawlist, from, to, occupied ? IM_COL32(90, 230, 110, 230) : IM_COL32(255, 255, 255, 150), occupied ? 2.5f : 1.5f);
			}
		for (int i = 0; i < static_cast<int>(tool.fits.size()); ++i)
		{
			auto const &fit{tool.fits[i]};
			auto const colour{false == fit.issue.empty() ? IM_COL32(255, 150, 60, 200) : fit.chosen ? IM_COL32(90, 230, 110, 230) : IM_COL32(200, 200, 200, 140)};
			draw_pieces(projection, drawlist, fit.pieces, colour, i == tool.hovered ? 4.f : 2.f);
			ImVec2 at;
			if (projection.project(fit.end.position, at))
				drawlist->AddCircle(at, i == tool.hovered ? 10.f : 6.f, colour, 16, 2.f);
		}
	}
	auto const &intent{m_intent};
	switch (intent.what)
	{
	case kind::turntable_exit:
	{
		auto const centre{turntable_centre()};
		auto const direction{heading_vector(tool.heading)};
		auto const rim{tool.length * 0.5};
		glm::dvec3 const to{centre.x + direction.x * (rim + tool.exit_length), centre.y, centre.z + direction.y * (rim + tool.exit_length)};
		projection.line(drawlist, intent.position, to, overlay_color::grip, 3.f);
		break;
	}
	case kind::turntable_fit:
	{
		turntable_fit fit;
		auto const fits{turntable_fit_end(intent.snap, fit)};
		draw_pieces(projection, drawlist, fit.pieces, fits ? overlay_color::grip : overlay_color::invalid, 3.f);
		if (false == fits)
			label(projection, intent.position, fit.issue);
		break;
	}
	case kind::turntable_place:
	{
		if (tool.chosen < 0 || tool.chosen >= static_cast<int>(tool.templates.size()))
			break;
		auto const rim{tool.templates[tool.chosen].length * 0.5};
		auto centre{intent.position};
		auto yaw{tool.yaw};
		if (intent.snap.track != nullptr)
		{
			auto const into{glm::normalize(plan_of(intent.snap.direction))};
			centre = {intent.snap.position.x - into.x * rim, intent.snap.position.y, intent.snap.position.z - into.y * rim};
			yaw = heading_of(into);
		}
		auto const direction{heading_vector(yaw)};
		draw_circle(projection, drawlist, centre, rim, IM_COL32(255, 255, 255, 170), 1.5f);
		projection.line(drawlist, {centre.x + direction.x * rim, centre.y, centre.z + direction.y * rim}, {centre.x - direction.x * rim, centre.y, centre.z - direction.y * rim}, IM_COL32(255, 255, 255, 170), 2.f);
		break;
	}
	default: break;
	}
	if (tool.placing && tool.chosen >= 0 && tool.chosen < static_cast<int>(tool.templates.size()))
	{
		auto const rim{tool.templates[tool.chosen].length * 0.5};
		auto const cursor{cursor_level(tool.centre.y)};
		auto const &io{ImGui::GetIO()};
		auto const dragged{glm::length(glm::vec2{io.MousePos.x, io.MousePos.y} - tool.pressed) >= kDragPixels};
		auto centre{tool.centre};
		auto yaw{tool.yaw};
		if (tool.snap.track != nullptr && false == dragged)
		{
			auto const into{glm::normalize(plan_of(tool.snap.direction))};
			centre = {tool.snap.position.x - into.x * rim, tool.snap.position.y, tool.snap.position.z - into.y * rim};
			yaw = heading_of(into);
		}
		else if (dragged && plan_distance(cursor, centre) > 0.5)
			yaw = std::round(heading_of(plan_of(cursor - centre)) * 10.0) / 10.0;
		auto const direction{heading_vector(yaw)};
		draw_circle(projection, drawlist, centre, rim, overlay_color::marked, 2.5f);
		projection.line(drawlist, {centre.x + direction.x * rim, centre.y, centre.z + direction.y * rim}, {centre.x - direction.x * rim, centre.y, centre.z - direction.y * rim}, overlay_color::marked, 3.f);
		if (dragged)
			label(projection, cursor, format("%.1f deg", yaw));
	}
}

bool editor_mode::turntable_controls(std::vector<std::string> &Rootstatements)
{
	if (scene::Layers.empty())
		return false;
	std::string root{scene::Layers.layer(scene::layer_handle{1}).name};
	if (auto const slash{root.find_last_of("/\\")}; slash != std::string::npos)
		root.erase(0, slash + 1);
	if (auto const dot{root.rfind('.')}; dot != std::string::npos)
		root.erase(dot);
	if (root.empty())
		return false;
	auto const file{root + "_obrotnice.scm"};
	std::ostringstream text;
	auto tables{0};
	for (auto const *table : simulation::Paths.sequence())
	{
		if (table == nullptr || table->eType != tt_Table || table->m_editorremoved || table->m_paths.empty() || table->name().empty())
			continue;
		auto const &name{table->name()};
		std::vector<int> slots;
		std::string events;
		scan_turntables();
		for (scene::instance_handle handle = 1; handle <= scene::Layers.instance_count() && slots.empty(); ++handle)
		{
			if (false == scene::Layers.tracked(handle))
				continue;
			auto const &included{scene::Layers.instance(handle)};
			if (included.removed || included.dead || included.file == nullptr)
				continue;
			auto const kind{std::find_if(m_turntable.templates.begin(), m_turntable.templates.end(), [&](turntable_template const &Template) { return normalized(Template.file) == normalized(*included.file); })};
			if (kind == m_turntable.templates.end() || kind->angles == 0)
				continue;
			std::string target;
			std::vector<std::string> values;
			if (false == editor_includes::parse_directive(scene::Layers.directive(handle), target, values) || kind->track > static_cast<int>(values.size()) || ToLower(values[kind->track - 1]) != name)
				continue;
			{
				std::ifstream input{Global.asCurrentSceneryPath + kind->file, std::ios_base::binary};
				events.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
				events = ToLower(events);
				std::replace_if(events.begin(), events.end(), [](char const Character) { return Character == '\t' || Character == '\r' || Character == '\n'; }, ' ');
				auto const parameter{"(p" + std::to_string(kind->track) + ")"};
				for (auto at{events.find(parameter)}; at != std::string::npos; at = events.find(parameter, at + name.size()))
					events.replace(at, parameter.size(), name);
			}
			std::vector<double> seen;
			for (int i = 0; i < kind->positions && kind->angles - 1 + i < static_cast<int>(values.size()); ++i)
				if (double angle; numeric(values[kind->angles - 1 + i], angle) && std::none_of(seen.begin(), seen.end(), [&](double const Other) { return std::abs(circular(Other - angle)) < 0.01; }))
				{
					seen.push_back(angle);
					slots.push_back(i + 1);
				}
		}
		if (slots.empty())
			for (int position = 1; position <= 64; ++position)
				slots.push_back(position);
		std::vector<int> positions;
		for (auto const position : slots)
		{
			// the events of a turntable placed in this session get loaded only with the scenery loaded again
			auto const event{name + "_tor" + std::to_string(position)};
			if (events.find(" " + event + " ") != std::string::npos || simulation::Events.FindEvent(event) != nullptr)
				positions.push_back(position);
		}
		if (positions.size() < 2)
			continue;
		++tables;
		auto const count{positions.size()};
		auto const centre{centre_of(*table)};
		auto const at{format("%.3f %.3f %.3f", centre.x, centre.y, centre.z)};
		text << "node -1 0 " << name << "_keylock memcell " << at << " none 0 0 none endmemcell\r\n";
		text << "event " << name << "_lock updatevalues 0 " << name << "_keylock * 1 * endevent\r\n";
		text << "event " << name << "_unlock updatevalues 1.5 " << name << "_keylock * 0 * endevent\r\n";
		for (auto const forward : {true, false})
		{
			auto const step{std::string{forward ? "_next" : "_prev"}};
			text << "event " << name << step << " multiple 0 " << name << "_keylock " << name << "_lock " << name << "_unlock " << name << step << '0';
			for (auto const position : positions)
				text << ' ' << name << step << position;
			text << " condition memcompare * 0 * endevent\r\n";
			// the turntable starts with no position set, the first step takes it to the first one
			text << "event " << name << step << "0 multiple 0 " << name << ' ' << name << "_tor" << positions.front() << " condition memcompare * 0 * endevent\r\n";
			for (std::size_t i = 0; i < count; ++i)
			{
				auto const target{positions[forward ? (i + 1) % count : (i + count - 1) % count]};
				text << "event " << name << step << positions[i] << " multiple 0 " << name << ' ' << name << "_tor" << target << " condition memcompare * " << positions[i] << " * endevent\r\n";
			}
		}
		text << "node -1 0 " << name << "_key eventlauncher " << at << " 150 t 0 " << name << "_next " << name << "_prev end\r\n";
	}
	auto const path{Global.asCurrentSceneryPath + file};
	std::error_code error;
	auto const exists{std::filesystem::exists(path, error)};
	if (tables == 0 && false == exists)
		return false;
	auto const content{"// made by the scenery editor on each save: T turns each turntable to its next position, Shift+T to the previous one\r\n" + text.str()};
	std::string current;
	{
		std::ifstream input{path, std::ios_base::binary};
		current.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
	}
	if (current != content)
	{
		std::ofstream output{path, std::ios_base::binary | std::ios_base::trunc};
		output << content;
	}
	auto included{m_turntable_included};
	for (std::size_t handle = 1; handle <= scene::Layers.size() && false == included; ++handle)
	{
		auto name{scene::Layers.layer(static_cast<scene::layer_handle>(handle)).name};
		std::replace(name.begin(), name.end(), '\\', '/');
		included = ToLower(name) == ToLower(file) || ToLower(name).ends_with("/" + ToLower(file));
	}
	if (included)
		return false;
	Rootstatements.push_back("include " + file + " end");
	return true;
}
