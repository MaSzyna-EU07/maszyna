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
#include "editor/editorFormat.hpp"
#include "editor/editorGauge.hpp"
#include "editor/editorGeometry.hpp"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/editoroverlay.h"
#include "rendering/renderer.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "world/Track.h"
#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>

using geometry::plan_of;

namespace
{

char const *const gauge_file{"editor_structure_gauges.txt"};
// past this distance of the camera from the origin of the overlay it's rebuilt, to keep its float precision
double const gauge_origin_range{2000.0};
// length of the line shown each way from the selected path, and around a hit
double const gauge_reach{500.0};
double const gauge_spot_reach{60.0};
// time spent on the scan of the whole map in a frame
auto const gauge_slice{std::chrono::milliseconds(8)};
// distance of the camera from the place of a hit, along the track
double const gauge_view_distance{4.0};

// sampled line of the tracks with the widening of the gauge along it
struct gauge_line
{
	gauge::profile const *profile{nullptr};
	editor_track::route route;
	std::vector<editor_track::route_sample> samples;
	std::vector<gauge::section> sections;
};

// half of the carriageway of a road at the sample, it narrows linearly to the width at the end
double carriageway(editor_track::route_span const &Span, double const Chainage)
{
	auto const &track{*Span.track};
	auto const end{track.fTrackWidth2 > 0.f ? track.fTrackWidth2 : track.fTrackWidth};
	auto t{Span.to > Span.from ? std::clamp((Chainage - Span.from) / (Span.to - Span.from), 0.0, 1.0) : 0.0};
	if (false == Span.forward)
		t = 1.0 - t;
	return 0.5 * std::abs(track.fTrackWidth + (end - track.fTrackWidth) * t);
}

gauge_line make_line(editor_track::route &Route, gauge::profile const &Profile)
{
	gauge_line line;
	line.profile = &Profile;
	line.samples = editor_track::sample_route(Route, 1.0);
	line.route = Route;
	bool const road{Profile.kind == gauge::kind::road};
	std::vector<double> chainage, curvature, cant;
	for (auto &sample : line.samples)
	{
		if (false == road)
			sample.position.y += gauge::rail_height;
		chainage.push_back(sample.chainage);
		curvature.push_back(sample.curvature);
		cant.push_back(sample.cant);
	}
	line.sections = gauge::sections(Profile.kind, chainage, curvature, cant);
	if (road)
		for (std::size_t i = 0; i < line.samples.size(); ++i)
			line.sections[i].base = carriageway(line.route.spans[line.samples[i].span], line.samples[i].chainage);
	return line;
}

// outline checked along the path, none on the pavements, which are the roads with the pavement texture
gauge::profile const *outline_of(gauge::choice const &Choice, TTrack const &Track)
{
	auto const *profile{Choice.of(Track.iCategoryFlag)};
	if (profile == nullptr || profile->outline.size() < 2)
		return nullptr;
	if (profile == Choice.road)
	{
		auto name{editor_track::material_name(Track.m_material1)};
		std::transform(name.begin(), name.end(), name.begin(), [](unsigned char const C) { return static_cast<char>(std::tolower(C)); });
		if (name.find("chodnik") != std::string::npos)
			return nullptr;
	}
	return profile;
}

// the line along the route from the path with the outline of its kind, none without the outline or the length
std::optional<gauge_line> track_line(gauge::choice const &Choice, TTrack const &Track, editor_track::route &Route)
{
	auto const *profile{outline_of(Choice, Track)};
	if (profile == nullptr)
		return std::nullopt;
	auto line{make_line(Route, *profile)};
	if (line.samples.size() < 2)
		return std::nullopt;
	return line;
}

// lines through all paths of the scenery, each path in one of them at least
std::vector<gauge_line> scenery_lines(gauge::choice const &Choice)
{
	std::vector<gauge_line> lines;
	std::unordered_set<TTrack const *> visited;
	auto const seen{[&](editor_track::route_span const &Span) { return visited.count(Span.track) > 0; }};
	auto const checked{[&](TTrack const *Track) { return Track != nullptr && editor_track::is_supported(*Track) && outline_of(Choice, *Track) != nullptr; }};
	for (auto *track : simulation::Paths.sequence())
	{
		if (false == checked(track) || visited.count(track) > 0)
			continue;
		auto route{editor_track::run_route(*track, 1e7)};
		// the line may run on over the paths already covered, one of them is enough for the widening ahead of a curve
		auto &spans{route.spans};
		while (spans.size() > 2 && seen(spans[0]) && seen(spans[1]))
			spans.erase(spans.begin());
		while (spans.size() > 2 && seen(spans[spans.size() - 1]) && seen(spans[spans.size() - 2]))
			spans.pop_back();
		for (auto const &span : spans)
			visited.insert(span.track);
		lines.push_back(make_line(route, *outline_of(Choice, *track)));
	}
	// diverging paths of the switches
	for (auto *track : simulation::Paths.sequence())
		if (checked(track) && track->eType == tt_Switch)
		{
			editor_track::route route;
			route.spans.push_back({track, 1, true});
			lines.push_back(make_line(route, *outline_of(Choice, *track)));
		}
	return lines;
}

// samples of the lines, found by the position in the plan
class corridor
{
public:
	explicit corridor(std::vector<gauge_line> const &Lines) : m_lines{Lines}
	{
		for (std::uint32_t line = 0; line < Lines.size(); ++line)
			for (std::uint32_t i = 0; i < Lines[line].samples.size(); ++i)
			{
				// with the room for the widening in the sharpest curves on the largest cant
				m_margin = std::max(m_margin, gauge::widest(*Lines[line].profile) + Lines[line].sections[i].base + 1.2);
				auto const point{plan_of(Lines[line].samples[i].position)};
				m_cells[key(point, cell)].push_back({line, i});
				m_regions.insert(key(point, region));
			}
	}

	static glm::dvec2 normal(editor_track::route_sample const &Sample)
	{
		return {-Sample.direction.y, Sample.direction.x};
	}

	// whether the sphere may reach the gauge of a line sampled in the cells around it
	bool close(glm::dvec3 const &Center, double const Radius) const
	{
		auto const center{plan_of(Center)};
		auto const reach{Radius + m_margin};
		auto const from{index(center - reach, cell)};
		auto const to{index(center + reach, cell)};
		for (auto x = from.x; x <= to.x; ++x)
			for (auto y = from.y; y <= to.y; ++y)
				if (m_cells.count(key(x, y)) > 0)
					return true;
		return false;
	}

	// whether the sphere may reach the gauge, roughly
	bool nearby(glm::dvec3 const &Center, double const Radius) const
	{
		auto const center{plan_of(Center)};
		auto const reach{Radius + m_margin + region};
		auto const from{index(center - reach, region)};
		auto const to{index(center + reach, region)};
		for (auto x = from.x; x <= to.x; ++x)
			for (auto y = from.y; y <= to.y; ++y)
				if (m_regions.count(key(x, y)) > 0)
					return true;
		return false;
	}

	struct hit
	{
		double depth{0.0}; // not positive outside of the gauge
		editor_track::route_span const *span{nullptr}; // of the line the point enters
		glm::dvec2 direction{0.0}; // of the line there, in the plan
	};
	// deepest intrusion of the point into the gauge of the lines nearby
	hit intrusion(glm::dvec3 const &Point) const
	{
		auto const point{plan_of(Point)};
		auto const center{index(point, cell)};
		// nearest sample of every line around
		std::vector<std::pair<entry, double>> nearest;
		for (auto x = center.x - 1; x <= center.x + 1; ++x)
			for (auto y = center.y - 1; y <= center.y + 1; ++y)
			{
				auto const found{m_cells.find(key(x, y))};
				if (found == m_cells.end())
					continue;
				for (auto const &candidate : found->second)
				{
					auto const distance{glm::length2(plan_of(sample(candidate).position) - point)};
					if (distance > m_margin * m_margin)
						continue;
					auto const same{std::find_if(nearest.begin(), nearest.end(), [&](auto const &Other) { return Other.first.line == candidate.line; })};
					if (same == nearest.end())
						nearest.push_back({candidate, distance});
					else if (distance < same->second)
						*same = {candidate, distance};
				}
			}
		hit deepest;
		for (auto const &[found, distance] : nearest)
		{
			auto const &line{m_lines[found.line]};
			auto const &sample{line.samples[found.index]};
			auto const offset{point - plan_of(sample.position)};
			auto const along{glm::dot(offset, sample.direction)};
			if ((found.index == 0 && along < -0.6) || (found.index + 1 == line.samples.size() && along > 0.6))
				continue;
			gauge::ring(*line.profile, line.sections[found.index], m_ring);
			auto const depth{gauge::intrusion(m_ring, {glm::dot(offset, normal(sample)), Point.y - (sample.position.y + sample.grade * along)})};
			if (depth > deepest.depth)
				deepest = {depth, &line.route.spans[sample.span], sample.direction};
		}
		return deepest;
	}

private:
	static constexpr double cell{8.0};
	static constexpr double region{64.0};

	struct entry
	{
		std::uint32_t line;
		std::uint32_t index;
	};

	static glm::i64vec2 index(glm::dvec2 const &Point, double const Size)
	{
		return {static_cast<std::int64_t>(std::floor(Point.x / Size)), static_cast<std::int64_t>(std::floor(Point.y / Size))};
	}
	static std::uint64_t key(std::int64_t const X, std::int64_t const Y)
	{
		return (static_cast<std::uint64_t>(X) << 32) ^ static_cast<std::uint32_t>(Y);
	}
	static std::uint64_t key(glm::dvec2 const &Point, double const Size)
	{
		auto const cell{index(Point, Size)};
		return key(cell.x, cell.y);
	}
	editor_track::route_sample const &sample(entry const &Entry) const
	{
		return m_lines[Entry.line].samples[Entry.index];
	}

	std::vector<gauge_line> const &m_lines;
	double m_margin{0.0};
	mutable std::vector<gauge::place> m_ring; // reused by the checks
	std::unordered_map<std::uint64_t, std::vector<entry>> m_cells;
	std::unordered_set<std::uint64_t> m_regions;
};

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

// the deepest place of the model in the gauge, added to the hits when it enters it
void scan_model(corridor const &Space, TAnimModel &Instance, std::vector<editor_mode::gauge_hit> &Hits)
{
	if (false == Instance.visible() || Instance.Model() == nullptr || Instance.Model()->GetSMRoot() == nullptr)
		return;
	if (false == Space.nearby(Instance.location(), Instance.radius()))
		return;
	editor_mode::gauge_hit hit{Instance.name(), Instance.location()};
	// the triangle enters the gauge when any point of a grid spread over it does
	visit_triangles(Instance.Model()->GetSMRoot(), placement(Instance), [&](glm::dvec3 const &A, glm::dvec3 const &B, glm::dvec3 const &C) {
		auto const middle{(A + B + C) / 3.0};
		if (false == Space.close(middle, std::max({glm::distance(middle, A), glm::distance(middle, B), glm::distance(middle, C)})))
			return;
		auto const edge{std::max({glm::distance(A, B), glm::distance(B, C), glm::distance(C, A)})};
		auto const steps{std::clamp(static_cast<int>(std::ceil(edge / 0.5)), 1, 24)};
		for (int u = 0; u <= steps; ++u)
			for (int v = 0; u + v <= steps; ++v)
			{
				auto const point{A + (B - A) * (static_cast<double>(u) / steps) + (C - A) * (static_cast<double>(v) / steps)};
				auto const found{Space.intrusion(point)};
				if (found.depth <= 0.0)
					continue;
				if (found.depth > hit.depth)
				{
					hit.depth = found.depth;
					hit.point = point;
					hit.track = found.span->track;
					hit.path = found.span->path;
					hit.direction = found.direction;
				}
				return;
			}
	});
	if (hit.depth > 0.0)
		Hits.push_back(std::move(hit));
}

void sort_hits(std::vector<editor_mode::gauge_hit> &Hits)
{
	std::sort(Hits.begin(), Hits.end(), [](auto const &Left, auto const &Right) { return Left.depth > Right.depth; });
}

// the models entering the gauge, the deepest first
void scan_models(corridor const &Space, std::vector<editor_mode::gauge_hit> &Hits)
{
	for (auto *instance : simulation::Instances.sequence())
		if (instance != nullptr)
			scan_model(Space, *instance, Hits);
	sort_hits(Hits);
}

// translucent tunnel of the gauge along the line, with the rings every 10 m and the lines along its corners
void build_tunnel(gauge_line const &Line, std::vector<glm::dvec3> &Surface, std::vector<glm::dvec3> &Edges)
{
	std::vector<gauge::place> places;
	auto const ring{[&](std::size_t const Index) {
		gauge::ring(*Line.profile, Line.sections[Index], places);
		auto const &sample{Line.samples[Index]};
		auto const normal{corridor::normal(sample)};
		std::vector<glm::dvec3> points;
		for (auto const &place : places)
			points.push_back(sample.position + glm::dvec3(normal.x * place.lateral, place.height, normal.y * place.lateral));
		return points;
	}};
	std::vector<glm::dvec3> previous;
	double lastring{-1e9};
	for (std::size_t i = 0; i < Line.samples.size(); i += 2)
	{
		auto const current{ring(i)};
		if (false == previous.empty())
			for (std::size_t j = 1; j < current.size(); ++j)
			{
				Surface.insert(Surface.end(), {previous[j - 1], previous[j], current[j], previous[j - 1], current[j], current[j - 1]});
				Edges.insert(Edges.end(), {previous[j], current[j]});
			}
		if (Line.samples[i].chainage - lastring >= 10.0 || i + 2 >= Line.samples.size())
		{
			lastring = Line.samples[i].chainage;
			for (std::size_t j = 1; j < current.size(); ++j)
				Edges.insert(Edges.end(), {current[j - 1], current[j]});
		}
		previous = current;
	}
}

// table of the points of the outline with the inserting and removing of the points, whether it changed
bool edit_points(std::vector<gauge::profile::point> &Points)
{
	bool edited{false};
	bool inserted{false};
	int remove{-1};
	ImGui::Columns(3, STR_C("gaugeoutline"), false);
	ImGui::SetColumnWidth(0, 110.0f);
	ImGui::SetColumnWidth(1, 110.0f);
	ImGui::TextDisabled(STR_C("Half-width"));
	ImGui::NextColumn();
	ImGui::TextDisabled(STR_C("Height"));
	ImGui::NextColumn();
	ImGui::NextColumn();
	for (int i = 0; i < static_cast<int>(Points.size()); ++i)
	{
		ImGui::PushID(i);
		auto &point{Points[i]};
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
			auto const &next{Points[std::min<std::size_t>(i + 1, Points.size() - 1)]};
			Points.insert(Points.begin() + i + 1, {0.5 * (point.half_width + next.half_width), 0.5 * (point.height + next.height)});
			inserted = edited = true;
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip(STR_C("Insert a point after this one"));
		ImGui::SameLine();
		if (Points.size() > 2 && ImGui::SmallButton("x"))
			remove = i;
		ImGui::NextColumn();
		ImGui::PopID();
		if (inserted)
			break;
	}
	ImGui::Columns(1);
	if (remove >= 0)
	{
		Points.erase(Points.begin() + remove);
		edited = true;
	}
	return edited;
}

} // namespace

struct gauge_scan
{
	std::optional<gauge::profile> railway;
	std::optional<gauge::profile> road;
	std::vector<gauge_line> lines;
	std::optional<corridor> space; // of the lines
	std::size_t next{0}; // instance to check
	float progress{0.f};
	std::chrono::steady_clock::time_point started;
};

void editor_mode::gauge_load()
{
	if (m_gauge.profiles.empty())
		m_gauge.profiles = gauge::load_profiles(gauge_file);
	// the chosen railway and road outlines, the first ones of the kind when there's none
	auto const fits{[&](int const Index, bool const Road) {
		return Index >= 0 && Index < static_cast<int>(m_gauge.profiles.size()) && (m_gauge.profiles[Index].kind == gauge::kind::road) == Road;
	}};
	for (auto [index, road] : {std::pair<int *, bool>{&m_gauge.profile, false}, std::pair<int *, bool>{&m_gauge.road, true}})
		if (false == fits(*index, road))
		{
			*index = -1;
			for (int i = 0; i < static_cast<int>(m_gauge.profiles.size()) && *index < 0; ++i)
				if (fits(i, road))
					*index = i;
		}
}

gauge::choice editor_mode::gauge_choice() const
{
	auto const pick{[&](int const Index) { return Index >= 0 ? &m_gauge.profiles[Index] : nullptr; }};
	return {pick(m_gauge.profile), pick(m_gauge.road)};
}

std::vector<editor_mode::gauge_hit> const &editor_mode::gauge_hits() const
{
	return m_gauge.map.scanned ? m_gauge.map.hits : m_gauge.line.hits;
}

void editor_mode::update_gauge()
{
	if (m_gauge.scan)
		step_gauge_map();
	if (false == m_gauge.open)
	{
		if (false == EditorOverlay.batches.empty())
			EditorOverlay.clear();
		m_gauge.published = false;
		return;
	}
	auto *track{m_gauge.enabled ? selected_track() : nullptr};
	// the geometry is rescanned once the edit ends
	if (ImGui::IsMouseDown(0) || m_track_gizmo_using || m_dragging)
		m_gauge.pending = true;
	else if (track == nullptr)
	{
		if (m_gauge.track != nullptr)
		{
			m_gauge.line = {};
			m_gauge.track = nullptr;
			m_gauge.published = false;
		}
	}
	else if (track != m_gauge.track || m_history.size() != m_gauge.history || m_gauge.pending)
	{
		m_gauge.track = track;
		m_gauge.history = m_history.size();
		m_gauge.pending = false;
		scan_gauge(*track);
		m_gauge.published = false;
	}
	if (false == m_gauge.published || glm::distance(Camera.Pos, EditorOverlay.origin) > gauge_origin_range)
		gauge_publish();
}

void editor_mode::scan_gauge(TTrack &Track)
{
	gauge_load();
	m_gauge.line = {};
	auto route{editor_track::run_route(Track, gauge_reach)};
	auto line{track_line(gauge_choice(), Track, route)};
	if (false == line.has_value())
		return;
	std::vector<gauge_line> const lines{std::move(*line)};
	corridor const space{lines};
	build_tunnel(lines.front(), m_gauge.line.surface, m_gauge.line.edges);
	scan_models(space, m_gauge.line.hits);
	if (false == m_gauge.map.scanned)
		m_gauge.current = -1;
}

void editor_mode::scan_gauge_map()
{
	gauge_load();
	auto scan{std::make_shared<gauge_scan>()};
	scan->started = std::chrono::steady_clock::now();
	auto const chosen{gauge_choice()};
	// copies, the outlines may be edited while it runs
	scan->railway = chosen.railway != nullptr ? std::optional<gauge::profile>{*chosen.railway} : std::nullopt;
	scan->road = chosen.road != nullptr ? std::optional<gauge::profile>{*chosen.road} : std::nullopt;
	scan->lines = scenery_lines({scan->railway ? &*scan->railway : nullptr, scan->road ? &*scan->road : nullptr});
	scan->space.emplace(scan->lines);
	m_gauge.map = {};
	m_gauge.map.scanned = true;
	m_gauge.map.history = m_history.size();
	auto const name{[](gauge::profile const *Profile) { return Profile != nullptr ? Profile->name : std::string{"-"}; }};
	m_gauge.map.profile = name(chosen.railway) + ", " + name(chosen.road);
	m_gauge.current = -1;
	m_gauge.published = false;
	m_gauge.scan = std::move(scan);
}

void editor_mode::step_gauge_map()
{
	auto &scan{*m_gauge.scan};
	auto const &instances{simulation::Instances.sequence()};
	auto const until{std::chrono::steady_clock::now() + gauge_slice};
	while (scan.next < instances.size() && std::chrono::steady_clock::now() < until)
	{
		if (auto *instance{instances[scan.next++]})
			scan_model(*scan.space, *instance, m_gauge.map.hits);
	}
	scan.progress = instances.empty() ? 1.f : static_cast<float>(scan.next) / instances.size();
	if (scan.next >= instances.size())
		finish_gauge_map();
}

void editor_mode::finish_gauge_map()
{
	auto const scan{std::move(m_gauge.scan)};
	auto &hits{m_gauge.map.hits};
	// the hit the camera went to stays chosen
	auto const chosen{m_gauge.current >= 0 && m_gauge.current < static_cast<int>(hits.size()) ? std::optional<glm::dvec3>{hits[m_gauge.current].point} : std::nullopt};
	sort_hits(hits);
	if (chosen)
		m_gauge.current = static_cast<int>(std::find_if(hits.begin(), hits.end(), [&](auto const &Hit) { return Hit.point == *chosen; }) - hits.begin());
	WriteLog("Structure gauge (" + m_gauge.map.profile + "): " + std::to_string(hits.size()) + " models enter it along " + std::to_string(scan->lines.size()) + " lines, scanned in " +
	         std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - scan->started).count()) + " ms");
	if (false == chosen.has_value() && false == hits.empty())
		gauge_focus(0);
}

void editor_mode::gauge_publish()
{
	m_gauge.published = true;
	EditorOverlay.clear();
	EditorOverlay.origin = Camera.Pos;
	auto const batch{[](glm::vec4 const &Color, unsigned int const Type, std::vector<glm::dvec3> const &Points) {
		gfx::editor_overlay::batch result{Color, Type};
		result.points.reserve(Points.size());
		for (auto const &point : Points)
			result.points.emplace_back(point - EditorOverlay.origin);
		return result;
	}};
	std::vector<glm::dvec3> marker;
	auto const &hits{gauge_hits()};
	if (m_gauge.current >= 0 && m_gauge.current < static_cast<int>(hits.size()))
	{
		auto const point{hits[m_gauge.current].point};
		marker = {point - glm::dvec3(0.0, 3.0, 0.0), point + glm::dvec3(0.0, 3.0, 0.0), point - glm::dvec3(1.0, 0.0, 0.0), point + glm::dvec3(1.0, 0.0, 0.0),
		          point - glm::dvec3(0.0, 0.0, 1.0), point + glm::dvec3(0.0, 0.0, 1.0)};
	}
	EditorOverlay.batches = {batch({0.2f, 0.9f, 0.3f, 0.12f}, GL_TRIANGLES, m_gauge.line.surface),
	                         batch({0.2f, 1.0f, 0.3f, 0.5f}, GL_LINES, m_gauge.line.edges),
	                         batch({1.0f, 0.9f, 0.1f, 1.0f}, GL_LINES, marker),
	                         batch({0.2f, 0.9f, 0.3f, 0.12f}, GL_TRIANGLES, m_gauge.spot.surface),
	                         batch({0.2f, 1.0f, 0.3f, 0.5f}, GL_LINES, m_gauge.spot.edges)};
}

void editor_mode::gauge_focus(int const Index)
{
	auto const &hits{gauge_hits()};
	if (hits.empty())
		return;
	m_gauge.current = (Index % static_cast<int>(hits.size()) + static_cast<int>(hits.size())) % static_cast<int>(hits.size());
	m_gauge.published = false;
	gauge_spot(hits[m_gauge.current]);
	auto const &hit{hits[m_gauge.current]};
	// along the track, tangent to the gauge at the height of the place, from the end nearer to the camera
	glm::dvec3 look{hit.direction.x, 0.0, hit.direction.y};
	look = glm::length(look) > 1e-6 ? glm::normalize(look) : glm::dvec3(0.0, 0.0, 1.0);
	if (glm::dot(Camera.Pos - hit.point, look) > 0.0)
		look = -look;
	m_focus_start_pos = Camera.Pos;
	m_focus_start_angle = Camera.Angle;
	m_focus_target_pos = hit.point - look * gauge_view_distance;
	m_focus_target_angle = glm::vec3(static_cast<float>(std::asin(std::clamp(look.y, -1.0, 1.0))), static_cast<float>(std::atan2(-look.x, -look.z)), 0.0f);
	m_focus_active = true;
	m_focus_time = 0.0;
	m_focus_duration = 0.6;
}

void editor_mode::gauge_spot(gauge_hit const &Hit)
{
	m_gauge.spot = {};
	auto const &paths{simulation::Paths.sequence()};
	// the hit outlives the scan, the path may be gone since
	if (Hit.track == nullptr || std::find(paths.begin(), paths.end(), Hit.track) == paths.end())
		return;
	auto *track{const_cast<TTrack *>(Hit.track)};
	editor_track::route route;
	if (Hit.path == 0)
		route = editor_track::run_route(*track, gauge_spot_reach);
	else
		route.spans.push_back({track, Hit.path, true});
	if (auto const line{track_line(gauge_choice(), *track, route)})
		build_tunnel(*line, m_gauge.spot.surface, m_gauge.spot.edges);
}

void editor_mode::render_gauge_window()
{
	if (false == m_gauge.open)
		return;
	ImGui::SetNextWindowSize(ImVec2(380.0f, 420.0f), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin(STR_C("Structure gauge"), &m_gauge.open))
	{
		ImGui::End();
		return;
	}
	if (ImGui::Checkbox(STR_C("Show along the selected path"), &m_gauge.enabled))
		m_gauge.pending = true;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip(STR_C("Translucent tunnel of the gauge 500 m each way along the line of the selected path.\nWidened in the curves under 250 m and tilted on the cant, from 20 m (inner side)\nand 26 m (outer side) ahead of the curve, as in the PKP PLK standard, volume II."));
	gauge_load();
	bool changed{render_gauge_choice("Railway", m_gauge.profile, false)};
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip(STR_C("GPL: unified structure gauge, widened only in the curves under 250 m.\nG1, G2, GA, GB, GC: limit (GSZ) and nominal (NSZ) installation gauges, widened by 3750/R in all curves."));
	changed |= render_gauge_choice("Road", m_gauge.road, true);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip(STR_C("Clearance over the roads, from 0.5 m beyond the edges of the carriageway"));

	if (m_gauge.scan)
	{
		auto const label{format("Scanning the map %.0f%%, %zu found", m_gauge.scan->progress * 100.f, m_gauge.map.hits.size())};
		ImGui::ProgressBar(m_gauge.scan->progress, ImVec2(-70.0f, 0.0f), label.c_str());
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Cancel")))
			finish_gauge_map();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip(STR_C("Stops the scan and keeps the models found so far"));
	}
	else if (ImGui::Button(STR_C("Find the violations on the whole map")))
		scan_gauge_map();
	else if (ImGui::IsItemHovered())
		ImGui::SetTooltip(STR_C("Checks the models along all tracks of the scenery against the chosen gauge\nand lists them, the deepest first.\nRuns in the background, the list fills up as it goes"));
	if (m_gauge.map.scanned && false == static_cast<bool>(m_gauge.scan))
	{
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Clear")))
		{
			m_gauge.map = {};
			m_gauge.current = -1;
			m_gauge.published = false;
		}
		else if (m_gauge.map.history != m_history.size())
			ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), STR_C("The scenery changed since the scan (%s)"), m_gauge.map.profile.c_str());
	}
	render_gauge_hits();
	changed |= render_gauge_outline(m_gauge.profile);
	changed |= render_gauge_outline(m_gauge.road);
	if (changed)
		m_gauge.pending = true;
	ImGui::End();
}

void editor_mode::render_gauge_hits()
{
	auto const &hits{gauge_hits()};
	if (false == m_gauge.map.scanned && (false == m_gauge.enabled || m_gauge.track == nullptr))
		return;
	if (hits.empty())
	{
		ImGui::TextDisabled(m_gauge.map.scanned ? STR_C("No model enters the gauge on the map") : STR_C("The gauge is clear along this line"));
		return;
	}
	ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), STR_C("%zu models enter the gauge%s"), hits.size(), m_gauge.map.scanned ? STR_C(" on the map") : "");
	if (ImGui::ArrowButton("##gaugeprevious", ImGuiDir_Left))
		gauge_focus(m_gauge.current - 1);
	ImGui::SameLine();
	if (ImGui::ArrowButton("##gaugenext", ImGuiDir_Right))
		gauge_focus(m_gauge.current + 1);
	ImGui::SameLine();
	ImGui::Text("%d / %zu", m_gauge.current + 1, hits.size());
	ImGui::BeginChild("gaugehits", ImVec2(0.0f, std::min(160.0f, ImGui::GetTextLineHeightWithSpacing() * hits.size() + 8.0f)), true);
	ImGuiListClipper clipper(static_cast<int>(hits.size()));
	while (clipper.Step())
		for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i)
		{
			auto const &hit{hits[i]};
			auto const label{(hit.model.empty() ? std::string{"(unnamed)"} : hit.model) + "  " + std::to_string(static_cast<int>(std::lround(hit.depth * 100.0))) + " cm##" + std::to_string(i)};
			if (ImGui::Selectable(label.c_str(), m_gauge.current == i))
				gauge_focus(i);
		}
	ImGui::EndChild();
}

bool editor_mode::render_gauge_choice(char const *Label, int &Index, bool const Road)
{
	bool changed{false};
	ImGui::SetNextItemWidth(-60.0f);
	if (ImGui::BeginCombo(Label, Index >= 0 ? m_gauge.profiles[Index].name.c_str() : STR_C("none")))
	{
		for (int i = 0; i < static_cast<int>(m_gauge.profiles.size()); ++i)
			if ((m_gauge.profiles[i].kind == gauge::kind::road) == Road && ImGui::Selectable(m_gauge.profiles[i].name.c_str(), i == Index))
			{
				changed = i != Index;
				Index = i;
			}
		ImGui::EndCombo();
	}
	return changed;
}

bool editor_mode::render_gauge_outline(int const Index)
{
	if (Index < 0)
		return false;
	auto &profile{m_gauge.profiles[Index]};
	if (false == ImGui::TreeNode(&profile, STR_C("Outline of %s (right half)"), profile.name.c_str()))
		return false;
	ImGui::TextDisabled(profile.kind == gauge::kind::road ? STR_C("Metres from the edge of the carriageway and above the road, bottom up.")
	                                                      : "PKP PLK standard, volume II; below 1170 mm the limit installation gauge.\nMetres from the track axis and above the rail top, bottom up.");
	bool edited{edit_points(profile.outline)};
	if (ImGui::SmallButton(STR_C("Restore the starting outlines")))
	{
		m_gauge.profiles = gauge::default_profiles();
		gauge_load();
		edited = true;
	}
	if (edited)
		gauge::save_profiles(gauge_file, m_gauge.profiles);
	ImGui::TreePop();
	return edited;
}
