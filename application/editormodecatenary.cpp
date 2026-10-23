/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Catenary in the editor: poles with cantilevers put by the track where the user sees them (on the orthophoto). The card of
// the cantilever is chosen from the catalogue by the distance from the wires to the face of the pole (band A, last digit of
// the card), the cantilever is assembled by the same solver the simulation uses and shown at once. Nothing is laid out
// automatically: the spans between the poles the user put are only checked against the tables of the catalogue.

#include "stdafx.h"
#include "application/editormodecatenary.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "world/Track.h"
#include "world/catenary_scene.h"
#include "world/catenary_layout.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>

namespace
{

using geometry::bezier;
using geometry::plan_of;

double const kRailTop{0.18}; // m from the points of a track to the heads of its rails
double const kCatenaryRange{1500.0}; // m from the camera, farther poles aren't drawn nor picked
double const kSpanSearch{150.0}; // m, the farthest a neighbouring pole of the same line is looked for
double const kSameLine{1.5}; // m across the span, farther the two poles belong to different tracks
double const kCurveRadius{4000.0}; // m, a track bent less is a straight for the cards (table 0007)
float const kPolePick{12.0f};
double const kSwitchRange{8.0}; // m from a pole to a switch path: the pole stands by the switch (also on the neighbouring track)

double nearest_parameter(TTrack const &Track, int const Path, glm::dvec3 const &Point)
{
	bezier const curve{Track.m_paths[Path]};
	auto parameter{0.0};
	auto best{std::numeric_limits<double>::max()};
	for (int k = 0; k <= 256; ++k)
	{
		auto const distance{geometry::plan_distance(curve.point(k / 256.0), Point)};
		if (distance < best)
		{
			best = distance;
			parameter = k / 256.0;
		}
	}
	return parameter;
}

glm::dvec2 forward_at(TTrack const &Track, int const Path, double const Parameter)
{
	auto const tangent{bezier{Track.m_paths[Path]}.first(Parameter)};
	glm::dvec2 const planar{tangent.x, tangent.z};
	return glm::length(planar) > 1e-9 ? glm::normalize(planar) : glm::dvec2{0.0, 1.0};
}

glm::dvec2 right_of(glm::dvec2 const &Direction)
{
	return {-Direction.y, Direction.x};
}

// the pole the suspension stands on, or the point it's attached at
glm::dvec3 pole_of(catenary::suspension const &Data)
{
	return Data.has_pole ? glm::dvec3{Data.pole[0], Data.pole[1], Data.pole[2]} : glm::dvec3{Data.attach[0], Data.attach[1], Data.attach[2]};
}

glm::dvec3 wire_of(catenary::suspension const &Data)
{
	return {Data.at[0], Data.at[1], Data.at[2]};
}

// from the pole towards the wires, in the plan; across the track
glm::dvec2 across_of(catenary::suspension const &Data)
{
	auto const delta{plan_of(wire_of(Data)) - plan_of(pole_of(Data))};
	return glm::length(delta) > 1e-6 ? glm::normalize(delta) : glm::dvec2{1.0, 0.0};
}

// the top of the pole over its point; a drop hangs down from its point, so its "top" is negative
double pole_top(catenary::suspension const &Data)
{
	auto const *catalog{catenary_scene_catalog()};
	auto const *pole{catalog != nullptr && Data.has_pole ? catalog->pole(Data.pole_type) : nullptr};
	if (pole != nullptr && false == pole->drop.empty())
		return -pole->length;
	return pole != nullptr ? pole->top : 7.0;
}

// spans to check against the catalogue: the spans of the chains, and between the poles which aren't in any chain the pairs of
// the same line next to each other. Two suspensions are of the same line when the chord between their wires runs along the
// track: across the bisector of their directions it's off no more than the staggers of the wires make (the wires of the
// neighbouring track are 4 m and more away)
struct span_check
{
	std::size_t from{0}, to{0};
	double length{0.0};
	double radius{0.0};
	double limit{0.0};
	int chain{-1};
};

// radius of the track over the span from the change of the direction of the cantilevers: R = length / angle
double span_radius(catenary::suspension const &A, catenary::suspension const &B, double const Length)
{
	auto const across{across_of(A)};
	auto otheracross{across_of(B)};
	if (glm::dot(otheracross, across) < 0.0)
		otheracross = -otheracross;
	auto const angle{std::acos(std::clamp(glm::dot(across, otheracross), -1.0, 1.0))};
	return angle > 1e-4 ? Length / angle : 0.0;
}

std::vector<span_check> spans_around(glm::dvec3 const &Camera, std::string const &Network, int const Zone)
{
	auto const &tables{catenary_scene_layout()};
	std::vector<span_check> result;
	auto const add = [&](std::size_t const From, std::size_t const To, std::string const &Symbol, int const Chain) {
		auto const &a{*catenary_scene_get(From)};
		auto const &b{*catenary_scene_get(To)};
		span_check span;
		span.from = From;
		span.to = To;
		span.chain = Chain;
		span.length = glm::length(plan_of(wire_of(b)) - plan_of(wire_of(a)));
		span.radius = span_radius(a, b, span.length);
		span.limit = catenary::span_limit(tables, Symbol.empty() ? Network : Symbol, Zone, span.radius > kCurveRadius ? 0.0 : span.radius, tables.curve_stagger());
		result.push_back(span);
	};
	std::vector<bool> chained(catenary_scene_size(), false);
	for (std::size_t chain = 0; chain < catenary_chain_size(); ++chain)
	{
		auto const *data{catenary_chain_get(chain)};
		if (data == nullptr)
			continue;
		auto const members{catenary_chain_members(chain)};
		for (std::size_t k = 0; k < members.size(); ++k)
		{
			if (members[k] < chained.size())
				chained[members[k]] = true;
			if (k + 1 >= members.size() || catenary_scene_get(members[k]) == nullptr || catenary_scene_get(members[k + 1]) == nullptr)
				continue;
			if (glm::distance(wire_of(*catenary_scene_get(members[k])), Camera) > kCatenaryRange)
				continue;
			add(members[k], members[k + 1], data->type, static_cast<int>(chain));
		}
	}
	std::vector<std::size_t> nearby;
	for (std::size_t index = 0; index < catenary_scene_size(); ++index)
	{
		auto const *data{catenary_scene_get(index)};
		if (data != nullptr && data->has_at && false == chained[index] && glm::distance(wire_of(*data), Camera) < kCatenaryRange)
			nearby.push_back(index);
	}
	std::vector<std::pair<std::size_t, std::size_t>> pairs;
	for (auto const i : nearby)
	{
		auto const &a{*catenary_scene_get(i)};
		auto const across{across_of(a)};
		glm::dvec2 const along{-across.y, across.x};
		for (int direction : {1, -1})
		{
			auto best{kSpanSearch};
			std::size_t found{i};
			for (auto const j : nearby)
			{
				if (j == i)
					continue;
				auto const &b{*catenary_scene_get(j)};
				auto const chord{plan_of(wire_of(b)) - plan_of(wire_of(a))};
				auto const length{glm::length(chord)};
				if (length < 1.0 || length >= best || glm::dot(chord, along) * direction <= 0.0)
					continue;
				// the directions of both suspensions turned to one sense; their bisector is the direction of the chord on a curve
				auto otheracross{across_of(b)};
				if (glm::dot(otheracross, across) < 0.0)
					otheracross = -otheracross;
				auto const bisector{glm::normalize(across + otheracross)};
				if (std::abs(glm::dot(chord, bisector)) > kSameLine)
					continue;
				best = length;
				found = j;
			}
			if (found != i)
				pairs.emplace_back(std::min(i, found), std::max(i, found));
		}
	}
	// a span found from both ends counts once
	std::sort(pairs.begin(), pairs.end());
	pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
	for (auto const &pair : pairs)
		add(pair.first, pair.second, {}, -1);
	return result;
}

// distance on the screen from the mouse to the segment between two points, the largest float if it's behind the camera
float mouse_to_segment(screen_projection const &Projection, glm::dvec3 const &A, glm::dvec3 const &B)
{
	ImVec2 a, b;
	if (false == Projection.project(A, a) || false == Projection.project(B, b))
		return std::numeric_limits<float>::max();
	auto const &mouse{ImGui::GetIO().MousePos};
	ImVec2 const span{b.x - a.x, b.y - a.y};
	auto const length2{span.x * span.x + span.y * span.y};
	auto const t{length2 > 1e-6f ? std::clamp(((mouse.x - a.x) * span.x + (mouse.y - a.y) * span.y) / length2, 0.0f, 1.0f) : 0.0f};
	auto const dx{a.x + span.x * t - mouse.x};
	auto const dy{a.y + span.y * t - mouse.y};
	return dx * dx + dy * dy;
}

char const *const kExecutions[] = {"1 - two contact wires, light arms, hk 1.70", "2 - two contact wires, heavy arms, hk 1.70", "3 - one contact wire, heavy arms, hk 1.70",
                                   "4 - one contact wire, heavy arms, hk 1.30", "5 - one contact wire, light arms, hk 1.70"};
char const *const kMaterials[] = {"cu", "al"};
// replaceable textures of the gantries (the masts, beams and drops have none of their own)
char const *const kGantrySkins[] = {"siec_trakcyjna/bramki_new1", "siec_trakcyjna/bramki_new2", "siec_trakcyjna/bramki_old1", "siec_trakcyjna/bramki_old2", "siec_trakcyjna/bramki_old3"};

// a pole worked on by itself: for that while no gantry counts as selected (otherwise the spot would hang on a drop)
struct without_gantry
{
	int &selected;
	int saved;
	explicit without_gantry(int &Selected) : selected{Selected}, saved{Selected} { selected = -1; }
	~without_gantry() { selected = saved; }
};
// two mast points of gantries are one mast (a gantry of many spans) when they're this close in the plan
bool same_mast(double const *A, double const *B)
{
	auto const dx{A[0] - B[0]}, dz{A[2] - B[2]};
	return dx * dx + dz * dz < 0.0025;
}

// the spans of the gantry: Gantry and all joined with it on shared masts, Gantry first
std::vector<std::size_t> gantry_group(std::size_t const Gantry, std::size_t const Without = std::numeric_limits<std::size_t>::max())
{
	std::vector<std::size_t> group;
	if (catenary_gantry_get(Gantry) == nullptr)
		return group;
	group.push_back(Gantry);
	for (std::size_t next = 0; next < group.size(); ++next)
	{
		auto const *span{catenary_gantry_get(group[next])};
		for (std::size_t other = 0; span != nullptr && other < catenary_gantry_size(); ++other)
		{
			auto const *candidate{catenary_gantry_get(other)};
			if (candidate == nullptr || other == Without || std::find(group.begin(), group.end(), other) != group.end())
				continue;
			for (int a = 0; a < 2; ++a)
				for (int b = 0; b < 2; ++b)
					if (false == span->masts[a].empty() && false == candidate->masts[b].empty() && same_mast(span->points[a], candidate->points[b]) && std::find(group.begin(), group.end(), other) == group.end())
						group.push_back(other);
		}
	}
	return group;
}

// how many other spans stand on this mast point
int spans_at(double const *Point, std::size_t const Except)
{
	int count{0};
	for (std::size_t other = 0; other < catenary_gantry_size(); ++other)
	{
		auto const *candidate{catenary_gantry_get(other)};
		if (candidate != nullptr && other != Except && ((false == candidate->masts[0].empty() && same_mast(candidate->points[0], Point)) || (false == candidate->masts[1].empty() && same_mast(candidate->points[1], Point))))
			++count;
	}
	return count;
}

// the items of a combo of poles or masts, by the groups of the described catalogue (foundation, where they stand), the
// rest at the end; Tag gives a word added to a name (e.g. through / end mast). Returns true when another one was chosen
bool grouped_items(std::vector<std::string> const &Names, int &Chosen, std::function<std::string(std::string const &)> const &Tag = {})
{
	auto const &structures{catenary_scene_structures()};
	auto changed{false};
	auto const item = [&](int const Index) {
		auto label{"  " + Names[Index]};
		if (Tag)
			label += Tag(Names[Index]);
		if (ImGui::Selectable(label.c_str(), Index == Chosen))
		{
			changed = Index != Chosen;
			Chosen = Index;
		}
	};
	std::vector<bool> shown(Names.size(), false);
	for (auto const &group : structures.polegroups())
	{
		std::vector<int> members;
		for (auto const &name : group.poles)
			if (auto const found{std::find(Names.begin(), Names.end(), name)}; found != Names.end())
				members.push_back(static_cast<int>(found - Names.begin()));
		if (members.empty())
			continue;
		std::string tags;
		if (group.foundation == "piles")
			tags += STR(", on piles");
		else if (group.foundation == "cast")
			tags += STR(", on a cast foundation");
		if (group.site == "line")
			tags += STR(", on the line");
		else if (group.site == "station")
			tags += STR(", in a station");
		else if (group.site == "platform")
			tags += STR(", on a platform");
		ImGui::TextDisabled("%s%s", group.title.c_str(), tags.c_str());
		if (ImGui::IsItemHovered() && false == group.text.empty())
			ImGui::SetTooltip("%s", group.text.c_str());
		for (auto const index : members)
		{
			shown[index] = true;
			item(index);
		}
	}
	if (std::find(shown.begin(), shown.end(), false) != shown.end())
	{
		if (false == structures.polegroups().empty())
			ImGui::TextDisabled("%s", STR_C("Other (not described in traction_catalog.txt)"));
		for (int i = 0; i < static_cast<int>(Names.size()); ++i)
			if (false == shown[i])
				item(i);
	}
	return changed;
}

// the states of the wires in the panel, and as they're written in the entries
char const *const kStates[] = {"new", "worn", "tarnished"};
int const kStateValues[] = {0, 2, 1};

} // namespace

void catenary_editor::catenary_scan()
{
	auto &tool{m_catenary};
	if (tool.scanned)
		return;
	tool.scanned = true;
	tool.poles.clear();
	if (auto const *catalog{catenary_scene_catalog()}; catalog != nullptr)
	{
		for (auto const &pole : catalog->poles())
			if (pole.second.drop.empty() && false == pole.second.gantry)
				tool.poles.push_back(pole.first);
		tool.masts.clear();
		for (auto const &mast : catalog->masts())
			tool.masts.push_back(mast.first);
	}
	// the first group of the described catalogue to start with: the tubular swinging ones, if they're there
	if (auto const &structures{catenary_scene_structures()}; false == structures.empty() && structures.group(tool.catalog_group) == nullptr)
	{
		auto const *start{structures.group("t_tube_swing")};
		if (start == nullptr)
			start = &structures.groups().front();
		tool.catalog_group = start->id;
		tool.catalog_kind = start->category;
	}
	tool.networks = catenary_scene_layout().symbols();
	auto const preferred{std::find(tool.networks.begin(), tool.networks.end(), std::string{"YwsC120-2C"})};
	tool.network = preferred != tool.networks.end() ? static_cast<int>(preferred - tool.networks.begin()) : 0;
	auto const pole{std::find(tool.poles.begin(), tool.poles.end(), std::string{"1611-11"})};
	tool.pole = pole != tool.poles.end() ? static_cast<int>(pole - tool.poles.begin()) : 0;
	if (tool.file[0] == '\0')
	{
		auto const files{catenary_scene_files()};
		auto name{files.empty() ? std::filesystem::path{Global.SceneryFile}.stem().string() + "_siec.scm" : files.front()};
		std::snprintf(tool.file, sizeof(tool.file), "%s", name.c_str());
	}
}

bool catenary_editor::catenary_spot_at(TTrack &Track, int const Path, glm::dvec3 const &Point, glm::dvec3 const &Cursor, int const Side, catenary_spot &Spot) const
{
	if (Track.m_paths.empty() || Track.m_editorremoved)
		return false;
	auto const path{std::clamp(Path, 0, static_cast<int>(Track.m_paths.size()) - 1)};
	bezier const curve{Track.m_paths[path]};
	auto const parameter{nearest_parameter(Track, path, Point)};
	Spot.track = &Track;
	Spot.path = path;
	Spot.parameter = parameter;
	Spot.axis = curve.point(parameter);
	Spot.forward = forward_at(Track, path, parameter);
	auto const right{right_of(Spot.forward)};
	Spot.side = Side != 0 ? Side : (glm::dot(plan_of(Cursor) - plan_of(Spot.axis), right) >= 0.0 ? 1 : -1);
	// the radius over about 10 m each way; on a curve the wires go to the outside of it (table 0007)
	auto const length{std::max(curve.plan_length(), 1.0)};
	auto const step{std::min(10.0 / length, 0.5)};
	auto const before{curve.point(std::max(0.0, parameter - step))};
	auto const after{curve.point(std::min(1.0, parameter + step))};
	double a[2]{before.x, before.z}, b[2]{Spot.axis.x, Spot.axis.z}, c[2]{after.x, after.z};
	Spot.radius = catenary::plan_radius(a, b, c);
	switch (m_catenary.bend)
	{
	case 1: Spot.curve = false; break;
	case 2: Spot.curve = true; break;
	default: Spot.curve = Spot.radius > 0.0 && Spot.radius < kCurveRadius; break;
	}
	Spot.toward = m_catenary.toward;
	if (Spot.curve && Spot.radius > 0.0)
	{
		// the middle of the arc is outwards of the chord: is the pole on the outside of the curve
		Spot.toward = glm::dot(plan_of(Spot.axis) - 0.5 * (plan_of(before) + plan_of(after)), right) * Spot.side > 0.0;
	}
	auto const stagger{Spot.curve ? std::max(static_cast<double>(m_catenary.stagger), catenary_scene_layout().curve_stagger()) : static_cast<double>(m_catenary.stagger)};
	catenary_compose(Spot, m_catenary.clearance, stagger, Spot.toward);
	// a gantry is selected: the cantilever hangs on a drop on its beam
	glm::dvec3 from, to;
	double bottom{0.0};
	auto const *catalog{catenary_scene_catalog()};
	if (m_catenary.gantry_placing || m_catenary.gantry_selected < 0 || catalog == nullptr || false == catenary_gantry_beam(static_cast<std::size_t>(m_catenary.gantry_selected), from, to, bottom))
		return true;
	auto const *gantry{catenary_gantry_get(static_cast<std::size_t>(m_catenary.gantry_selected))};
	if (gantry != nullptr && m_catenary.gantry_hang == 1)
		return catenary_on_mast(*gantry, stagger, Spot);
	auto const *beam{gantry != nullptr ? catalog->beam(gantry->beam) : nullptr};
	std::vector<std::pair<double, std::string>> drops;
	for (auto const &pole : catalog->poles())
		if (beam != nullptr && pole.second.drop == beam->drop)
			drops.emplace_back(pole.second.length, pole.first);
	std::sort(drops.begin(), drops.end());
	if (drops.empty())
	{
		Spot.card = 0;
		Spot.problem = STR("No drops in the catalogue for this beam");
		return true;
	}
	// along the track onto the line of the beam
	catenary_compose(Spot, m_catenary.clearance, stagger, Spot.toward, drops.front().second);
	auto const p{plan_of(glm::dvec3{Spot.data.pole[0], 0.0, Spot.data.pole[2]})};
	auto const f{Spot.forward};
	auto const origin{plan_of(from)};
	auto const u{plan_of(to) - origin};
	auto const denominator{f.x * u.y - f.y * u.x};
	if (std::abs(denominator) < 1e-6)
	{
		Spot.card = 0;
		Spot.problem = STR("The track runs along the beam of the gantry");
		return true;
	}
	auto const s{((origin.x - p.x) * u.y - (origin.y - p.y) * u.x) / denominator};
	auto const t{((origin.x - p.x) * f.y - (origin.y - p.y) * f.x) / denominator};
	if (t < 0.0 || t > 1.0)
	{
		Spot.card = 0;
		Spot.problem = STR("The drop would be off the beam of the gantry");
		return true;
	}
	Spot.axis.x += f.x * s;
	Spot.axis.z += f.y * s;
	bottom = from.y + (to.y - from.y) * t; // the beam may lie aslant between masts of different clearance
	// the shortest drop which reaches down to the lowest mount of the cantilever
	auto const take = [&](std::string const &Name) {
		catenary_compose(Spot, m_catenary.clearance, stagger, Spot.toward, Name);
		Spot.data.pole[1] = bottom;
	};
	// the drop chosen in the panel, if it's one for this beam
	if (false == m_catenary.drop_choice.empty() && std::any_of(drops.begin(), drops.end(), [&](auto const &Drop) { return Drop.second == m_catenary.drop_choice; }))
	{
		take(m_catenary.drop_choice);
		return true;
	}
	for (auto const &drop : drops)
	{
		take(drop.second);
		if (Spot.card == 0)
			return true;
		catenary::cantilever_result probe;
		catenary_scene_probe(Spot.data, probe);
		if (std::none_of(probe.warnings.begin(), probe.warnings.end(), [](std::string const &Warning) { return Warning.find("below the lowest mount") != std::string::npos; }))
			return true;
	}
	take(drops.back().second);
	return true;
}

// the line of the beam of the gantry at its bottom (where the drops hang from)
bool catenary_editor::catenary_gantry_beam(std::size_t const Index, glm::dvec3 &From, glm::dvec3 &To, double &Bottom) const
{
	auto const *gantry{catenary_gantry_get(Index)};
	auto const *catalog{catenary_scene_catalog()};
	auto const *first{gantry != nullptr && catalog != nullptr ? catalog->mast(gantry->masts[0]) : nullptr};
	auto const single{gantry != nullptr && gantry->masts[1].empty()};
	auto const *second{gantry != nullptr && catalog != nullptr && false == single ? catalog->mast(gantry->masts[1]) : nullptr};
	auto const *beam{gantry != nullptr && catalog != nullptr ? catalog->beam(gantry->beam) : nullptr};
	if (first == nullptr || (second == nullptr && false == single) || beam == nullptr)
		return false;
	// each end on the mount of its own mast; a single element from the face of its mast for its length
	double from[3], to[3];
	gantry_beam_ends(*gantry, *first, second, beam->span, from, to);
	From = {from[0], from[1], from[2]};
	To = {to[0], to[1], to[2]};
	Bottom = From.y;
	return true;
}

glm::dvec3 catenary_editor::catenary_mast_point() const
{
	// on the ground, at the height of the rail heads of the track under the cursor if there's one
	auto point{e.cursor_ground()};
	if (e.m_hover.track != nullptr)
	{
		point.y = e.m_hover.point.y + kRailTop;
		// the first mast: its face at the set distance from the axis of that track, on the side of the cursor
		auto const &tool{m_catenary};
		auto const *catalog{catenary_scene_catalog()};
		if (tool.gantry_snap && false == tool.gantry_first.has_value() && false == e.m_hover.track->m_paths.empty() && catalog != nullptr && false == tool.masts.empty())
		{
			auto const path{std::clamp(e.m_hover.path, 0, static_cast<int>(e.m_hover.track->m_paths.size()) - 1)};
			bezier const curve{e.m_hover.track->m_paths[path]};
			auto const parameter{nearest_parameter(*e.m_hover.track, path, point)};
			auto const axis{curve.point(parameter)};
			auto const right{right_of(forward_at(*e.m_hover.track, path, parameter))};
			auto const side{glm::dot(plan_of(point) - plan_of(axis), right) >= 0.0 ? 1.0 : -1.0};
			auto const *mast{catalog->mast(tool.masts[std::clamp(tool.mast_first, 0, static_cast<int>(tool.masts.size()) - 1)])};
			auto const half{mast != nullptr ? mast->offset : 0.0};
			auto const at{plan_of(axis) + right * side * (static_cast<double>(tool.gantry_face) + half)};
			point = {at.x, axis.y + kRailTop, at.y};
		}
	}
	return point;
}

bool catenary_editor::catenary_mast_face(std::string const &Mast, glm::dvec3 const &Point, double &Face, glm::dvec2 &Away) const
{
	auto const *catalog{catenary_scene_catalog()};
	auto const *mast{catalog != nullptr ? catalog->mast(Mast) : nullptr};
	auto best{20.0};
	auto found{false};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		for (int path = 0; path < static_cast<int>(track->m_paths.size()); ++path)
		{
			bezier const curve{track->m_paths[path]};
			if (geometry::plan_distance(curve.point(0.5), Point) > curve.plan_length() * 0.5 + best)
				continue;
			auto const parameter{nearest_parameter(*track, path, Point)};
			auto const axis{curve.point(parameter)};
			auto const distance{geometry::plan_distance(axis, Point)};
			if (distance < best && distance > 1e-3)
			{
				best = distance;
				Away = glm::normalize(plan_of(Point) - plan_of(axis));
				found = true;
			}
		}
	}
	// the face towards the track: the half of the mast across it as the drops and the beam take it (its offset)
	Face = best - (mast != nullptr ? mast->offset : 0.0);
	return found;
}

bool catenary_editor::catenary_along_track(glm::dvec3 const &Point, double const Axis, glm::dvec3 &Place) const
{
	// of the tracks near, the one the point is nearest to the wanted distance from: the same track as before, also over its joints
	auto best{std::numeric_limits<double>::max()};
	auto found{false};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		for (int path = 0; path < static_cast<int>(track->m_paths.size()); ++path)
		{
			bezier const curve{track->m_paths[path]};
			if (geometry::plan_distance(curve.point(0.5), Point) > curve.plan_length() * 0.5 + Axis + 10.0)
				continue;
			auto const parameter{nearest_parameter(*track, path, Point)};
			auto const axis{curve.point(parameter)};
			auto const distance{geometry::plan_distance(axis, Point)};
			// off the end of the path: the point isn't square to it
			auto const forward{forward_at(*track, path, parameter)};
			if ((parameter <= 1e-6 || parameter >= 1.0 - 1e-6) && std::abs(glm::dot(plan_of(Point) - plan_of(axis), forward)) > 0.05)
				continue;
			if (std::abs(distance - Axis) >= best)
				continue;
			best = std::abs(distance - Axis);
			auto const right{right_of(forward)};
			auto const side{glm::dot(plan_of(Point) - plan_of(axis), right) >= 0.0 ? 1.0 : -1.0};
			auto const at{plan_of(axis) + right * side * Axis};
			Place = {at.x, Point.y, at.y};
			found = true;
		}
	}
	return found && best < 10.0;
}

void catenary_editor::catenary_gantry_shift(glm::dvec2 const &Shift)
{
	auto &tool{m_catenary};
	if (tool.gantry_selected < 0 || glm::length(Shift) < 1e-6)
		return;
	std::vector<std::pair<std::size_t, catenary::gantry>> spans;
	std::vector<std::pair<std::size_t, catenary::suspension>> hangers;
	for (auto const span : gantry_group(static_cast<std::size_t>(tool.gantry_selected)))
	{
		spans.emplace_back(span, *catenary_gantry_get(span));
		catenary_gantry_hangers(span, hangers);
	}
	// dragging the distance: the state before the drag is taken back at once
	auto const record{tool.shift_group == 0 || tool.undo.empty() || tool.undo.back().group != tool.shift_group};
	if (record)
		tool.shift_group = ++tool.group;
	for (auto const &[span, before] : spans)
	{
		if (false == record)
			break;
		catenary_change change;
		change.group = tool.group;
		change.gantry = true;
		change.index = span;
		change.gantry_before = before;
		tool.undo.push_back(change);
	}
	if (record)
		for (auto const &[index, before] : hangers)
			tool.undo.push_back({tool.group, false, index, before, std::nullopt, false});
	for (auto const &[span, before] : spans)
	{
		auto moved{before};
		for (auto &point : moved.points)
		{
			point[0] += Shift.x;
			point[2] += Shift.y;
		}
		catenary_gantry_update(span, moved);
	}
	for (auto const &[index, before] : hangers)
	{
		auto shifted{before};
		for (auto *point : {shifted.at, shifted.rope, shifted.pole, shifted.attach})
		{
			point[0] += Shift.x;
			point[2] += Shift.y;
		}
		catenary_scene_update(index, shifted);
		tool.spots.erase(index);
	}
}

// the second mast: towards the cursor, as far as the nearest beam of the catalogue spans
bool catenary_editor::catenary_gantry_second(glm::dvec3 const &Point, glm::dvec3 &Second, std::string &Beam) const
{
	auto const &tool{m_catenary};
	auto const *catalog{catenary_scene_catalog()};
	if (catalog == nullptr || false == tool.gantry_first.has_value())
		return false;
	auto const first{*tool.gantry_first};
	auto const delta{plan_of(Point) - plan_of(first)};
	auto distance{glm::length(delta)};
	if (tool.gantry_square && tool.gantry_track.has_value())
		distance = std::abs(glm::dot(right_of(*tool.gantry_track), delta)); // square to the track: only the distance across it counts
	if (distance < 1.0)
		return false;
	// the beam runs between the faces of the masts: their axes are farther apart by the offsets of both
	auto const *a{catalog->mast(tool.masts.empty() ? std::string{} : tool.masts[std::clamp(tool.mast_first, 0, static_cast<int>(tool.masts.size()) - 1)])};
	auto const *b{catalog->mast(tool.masts.empty() ? std::string{} : tool.masts[std::clamp(tool.mast_second, 0, static_cast<int>(tool.masts.size()) - 1)])};
	// a single element: from the face of its one mast for its length; otherwise between the faces of the masts 0.20 m less than the beam
	auto const offsets{tool.gantry_single ? (a != nullptr ? a->offset : 0.0) : (a != nullptr ? a->offset : 0.0) + (b != nullptr ? b->offset : 0.0) - 0.20};
	auto best{std::numeric_limits<double>::max()};
	double span{0.0};
	for (auto const &beam : catalog->beams())
		if (beam.second.single == tool.gantry_single && std::abs(beam.second.span + offsets - distance) < best)
		{
			best = std::abs(beam.second.span + offsets - distance);
			span = beam.second.span;
			Beam = beam.first;
		}
	if (span <= 0.0)
		return false;
	auto direction{delta / distance};
	if (tool.gantry_square && tool.gantry_track.has_value())
	{
		// square to the track by the first mast, on the side of the cursor
		auto const across{right_of(*tool.gantry_track)};
		direction = glm::dot(across, delta) >= 0.0 ? across : -across;
	}
	// both masts at the height of the first one (the rail heads of the track by it); their heights can be changed by 0.20 m later
	Second = {first.x + direction.x * (span + offsets), first.y, first.z + direction.y * (span + offsets)};
	return true;
}

int catenary_editor::catenary_gantry_hit() const
{
	screen_projection const projection;
	auto best{kPolePick * kPolePick};
	int result{-1};
	for (std::size_t index = 0; index < catenary_gantry_size(); ++index)
	{
		glm::dvec3 from, to;
		double bottom{0.0};
		if (false == catenary_gantry_beam(index, from, to, bottom) || glm::distance(from, glm::dvec3{Global.pCamera.Pos}) > kCatenaryRange)
			continue;
		auto const distance{mouse_to_segment(projection, from, to)};
		if (distance < best)
		{
			best = distance;
			result = static_cast<int>(index);
		}
	}
	return result;
}

// the cantilever straight on a mast of the gantry: the mast on the side of the cursor, the cantilever across the track at the
// mast (the spot moved along the track to it), the card from the distance of the face of the mast
bool catenary_editor::catenary_on_mast(catenary::gantry const &Gantry, double const Stagger, catenary_spot &Spot) const
{
	auto const *catalog{catenary_scene_catalog()};
	auto const right{right_of(Spot.forward) * static_cast<double>(Spot.side)};
	int best{-1};
	auto nearest{std::numeric_limits<double>::max()};
	for (int k = 0; k < 2; ++k)
	{
		if (Gantry.masts[k].empty())
			continue;
		glm::dvec3 const mast{Gantry.points[k][0], Gantry.points[k][1], Gantry.points[k][2]};
		auto const across{glm::dot(plan_of(mast) - plan_of(Spot.axis), right)};
		if (across > 0.3 && across < nearest)
		{
			nearest = across;
			best = k;
		}
	}
	Spot.card = 0;
	if (best < 0)
	{
		Spot.problem = STR("No mast of this gantry on that side of the track");
		return true;
	}
	auto const &name{Gantry.masts[best]};
	auto const *pole{catalog != nullptr ? catalog->pole(name) : nullptr};
	if (pole == nullptr || false == pole->gantry)
	{
		Spot.problem = format(STR_C("Mast %s has no mounts for a cantilever (strut/stay in its entry in traction_supports.txt)"), name.c_str());
		return true;
	}
	glm::dvec3 const mast{Gantry.points[best][0], Gantry.points[best][1], Gantry.points[best][2]};
	// along the track onto the mast
	Spot.parameter = nearest_parameter(*Spot.track, Spot.path, mast);
	Spot.axis = bezier{Spot.track->m_paths[Spot.path]}.point(Spot.parameter);
	Spot.forward = forward_at(*Spot.track, Spot.path, Spot.parameter);
	auto const out{right_of(Spot.forward) * static_cast<double>(Spot.side)};
	auto const clearance{glm::dot(plan_of(mast) - plan_of(Spot.axis), out) - 0.5 * pole->depth(0.0)};
	catenary_compose(Spot, clearance, Stagger, Spot.toward, name);
	Spot.data.pole[0] = mast.x;
	Spot.data.pole[1] = mast.y;
	Spot.data.pole[2] = mast.z;
	return true;
}

// whether a pole of this type can't stand there: its group is no_switch (concrete 1701, 1711) and the track it stands by is a switch,
// or a switch path passes within kSwitchRange of its foot (also a switch on the neighbouring track)
bool catenary_editor::catenary_switch_error(std::string const &Pole, glm::dvec3 const &Foot, TTrack const *Track) const
{
	auto const *group{catenary_scene_structures().polegroup_of(Pole)};
	if (group == nullptr || false == group->no_switch)
		return false;
	if (Track != nullptr && Track->eType == tt_Switch)
		return true;
	auto const &tracks{simulation::Paths.sequence()};
	if (tracks.size() != m_switch_tracks)
	{
		m_switch_tracks = tracks.size();
		m_switch_paths.clear();
		m_switch_checks.clear();
		for (auto const *track : tracks)
			if (track != nullptr && track->eType == tt_Switch)
				for (int path = 0; path < static_cast<int>(track->m_paths.size()); ++path)
					m_switch_paths.emplace_back(track, path);
	}
	for (auto const &[track, path] : m_switch_paths)
	{
		if (track->m_editorremoved)
			continue;
		bezier const curve{track->m_paths[path]};
		if (geometry::plan_distance(curve.point(0.5), Foot) > curve.plan_length() * 0.5 + kSwitchRange)
			continue;
		if (geometry::plan_distance(curve.point(nearest_parameter(*track, path, Foot)), Foot) < kSwitchRange)
			return true;
	}
	return false;
}

bool catenary_editor::catenary_switch_error(std::size_t const Index) const
{
	auto const *data{catenary_scene_get(Index)};
	if (data == nullptr || false == data->has_pole)
		return false;
	auto const foot{pole_of(*data)};
	auto &check{m_switch_checks[Index]};
	if (check.pole != data->pole_type || check.foot != foot)
	{
		check.pole = data->pole_type;
		check.foot = foot;
		check.error = catenary_switch_error(data->pole_type, foot, nullptr);
	}
	return check.error;
}

void catenary_editor::catenary_compose(catenary_spot &Spot, double const Clearance, double const Stagger, bool const Toward, std::string const &Pole) const
{
	auto const &tool{m_catenary};
	Spot.card = 0;
	Spot.problem.clear();
	Spot.toward = Toward;
	auto const *catalog{catenary_scene_catalog()};
	auto const *pole{catalog == nullptr ? nullptr : false == Pole.empty() ? catalog->pole(Pole) : tool.pole >= 0 && tool.pole < static_cast<int>(tool.poles.size()) ? catalog->pole(tool.poles[tool.pole]) : nullptr};
	if (pole == nullptr)
	{
		Spot.problem = STR("No pole: the catalogue (data/traction_cantilevers.txt, data/traction_supports.txt) isn't loaded or has no poles");
		return;
	}
	auto const height{static_cast<double>(tool.height)};
	auto const railhead{Spot.axis.y + kRailTop};
	auto const right{right_of(Spot.forward) * static_cast<double>(Spot.side)}; // from the track to the pole
	glm::dvec3 const out{right.x, 0.0, right.y};
	auto const axisdistance{Clearance + 0.5 * pole->depth(0.0)};
	auto const offset{Toward ? Stagger : -Stagger};
	auto const polepoint{glm::dvec3{Spot.axis.x, railhead, Spot.axis.z} + out * axisdistance};
	auto const wire{glm::dvec3{Spot.axis.x, railhead + height, Spot.axis.z} + out * offset};
	Spot.reach = axisdistance - offset - 0.5 * pole->depth(height);
	// the card: from the group of the described catalogue, or (without it) from the tubular families 11xx/12xx
	catenary::cantilever_scheme const *scheme{nullptr};
	auto const &structures{catenary_scene_structures()};
	auto const described{false == structures.empty() && structures.group(tool.catalog_group) != nullptr};
	int decade{0};
	std::string pickproblem;
	Spot.family.clear();
	Spot.execution_changed = false;
	if (described)
	{
		auto const pick{catenary::pick_card(structures, *catalog, tool.catalog_group, tool.catalog_family, tool.catalog_card, Spot.curve, Toward, tool.execution, Spot.reach)};
		scheme = pick.scheme;
		if (pick.family != nullptr)
			Spot.family = pick.family->id;
		Spot.execution_changed = pick.execution_changed;
		pickproblem = pick.problem;
	}
	else
	{
		catenary::suspension_family family;
		family.swinging = tool.swinging;
		family.curve = Spot.curve;
		family.toward_pole = Toward;
		decade = catenary::family_decade(family);
		scheme = catalog->scheme_for_reach(decade, tool.execution, Spot.reach);
	}
	auto &data{Spot.data};
	data = catenary::suspension();
	data.range_max = 1000.0;
	data.execution = tool.execution;
	data.has_at = true;
	data.at[0] = wire.x;
	data.at[1] = wire.y;
	data.at[2] = wire.z;
	data.has_pole = true;
	data.pole_type = pole->name;
	data.pole[0] = polepoint.x;
	data.pole[1] = polepoint.y;
	data.pole[2] = polepoint.z;
	Spot.switch_error = catenary_switch_error(pole->name, polepoint, Spot.track);
	if (tool.insulators25)
		data.insulators = "25";
	if (scheme == nullptr)
	{
		if (described)
			Spot.problem = pickproblem.find("no group") == 0 ? pickproblem : format(STR_C("No card of the group for A = %.2f m"), Spot.reach);
		else
			Spot.problem = format(STR_C("No card %d0-%d9 in execution %d for A = %.2f m (cards cover A 1.45-4.45 m, not all of them are in every family)"), decade / 10, decade / 10, tool.execution, Spot.reach);
		return;
	}
	Spot.card = scheme->card;
	data.card = scheme->card;
	data.execution = scheme->execution;
	data.has_rope = true;
	data.rope[0] = wire.x;
	// the system height: the card's, or the one set in the panel (the solver raises the stay mount with the messenger)
	data.rope[1] = wire.y + (tool.system_height > 0.0f ? static_cast<double>(tool.system_height) : scheme->height);
	data.rope[2] = wire.z;
}

// where the suspension stands by the track: the place it was put at in this session, or the nearest path to its wires
catenary_editor::catenary_spot const *catenary_editor::catenary_spot_of(std::size_t const Index)
{
	auto &tool{m_catenary};
	if (auto const found{tool.spots.find(Index)}; found != tool.spots.end())
		return &found->second;
	auto const *data{catenary_scene_get(Index)};
	if (data == nullptr || false == data->has_at)
		return nullptr;
	auto const pole{pole_of(*data)};
	auto const wire{wire_of(*data)};
	TTrack *nearest{nullptr};
	int nearestpath{0};
	auto best{12.0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		for (int path = 0; path < static_cast<int>(track->m_paths.size()); ++path)
		{
			bezier const curve{track->m_paths[path]};
			if (geometry::plan_distance(curve.point(0.5), wire) > curve.plan_length() * 0.5 + best)
				continue;
			auto const distance{geometry::plan_distance(curve.point(nearest_parameter(*track, path, wire)), wire)};
			if (distance < best)
			{
				best = distance;
				nearest = track;
				nearestpath = path;
			}
		}
	}
	if (nearest == nullptr)
		return nullptr;
	catenary_spot spot;
	{
		without_gantry const plain{tool.gantry_selected};
		catenary_spot_at(*nearest, nearestpath, wire, pole, 0, spot);
	}
	// what the suspension has, not what the panel would make
	auto const right{right_of(spot.forward) * static_cast<double>(spot.side)};
	spot.toward = glm::dot(plan_of(wire) - plan_of(spot.axis), right) >= 0.0;
	spot.data = *data;
	spot.card = data->card;
	auto const decade{(data->card / 10) % 10};
	if (data->card / 100 == 11 || data->card / 100 == 12)
		spot.curve = decade == 3 || decade == 4;
	return &(tool.spots[Index] = spot);
}

std::string catenary_editor::catenary_intent_label(catenary_spot const &Spot) const
{
	auto const where{Spot.curve ? format(STR_C("curve R %.0f m"), Spot.radius) : std::string{STR("straight")}};
	auto const offset{Spot.toward ? STR("towards the pole") : STR("away from the pole")};
	if (Spot.card == 0)
		return format(STR_C("Pole here, %s, wires %s: %s"), where.c_str(), offset.c_str(), Spot.problem.c_str());
	auto label{format(STR_C("Click: pole %s, card %04d execution %d, A = %.2f m (%s, wires %s)"), Spot.data.pole_type.c_str(), Spot.card, Spot.data.execution, Spot.reach, where.c_str(), offset.c_str())};
	if (auto const *family{catenary_scene_structures().family(Spot.family)}; family != nullptr)
		label += "\n" + family->title;
	if (Spot.execution_changed)
		label += "\n" + format(STR_C("execution %d has no card for this A, execution %d taken"), m_catenary.execution, Spot.data.execution);
	if (Spot.switch_error)
		label += "\n" + format(STR_C("ERROR: pole %s doesn't stand by a switch, take a steel one"), Spot.data.pole_type.c_str());
	if (m_catenary.chaining && m_catenary.last >= 0 && catenary_scene_get(static_cast<std::size_t>(m_catenary.last)) != nullptr)
		label += "\n" + format(STR_C("with the span from the last pole, %.1f m"), glm::length(plan_of(wire_of(Spot.data)) - plan_of(wire_of(*catenary_scene_get(static_cast<std::size_t>(m_catenary.last))))));
	return label;
}

int catenary_editor::catenary_hit() const
{
	screen_projection const projection;
	auto best{kPolePick * kPolePick};
	int result{-1};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (std::size_t index = 0; index < catenary_scene_size(); ++index)
	{
		auto const *data{catenary_scene_get(index)};
		if (data == nullptr || false == data->has_at)
			continue;
		auto const foot{pole_of(*data)};
		if (glm::distance(foot, camera) > kCatenaryRange)
			continue;
		auto const distance{mouse_to_segment(projection, foot, foot + glm::dvec3{0.0, pole_top(*data), 0.0})};
		if (distance < best)
		{
			best = distance;
			result = static_cast<int>(index);
		}
	}
	return result;
}

// the chain whose span is under the cursor; -1 if none
int catenary_editor::catenary_span_hit() const
{
	screen_projection const projection;
	auto best{kPolePick * kPolePick * 0.5f};
	int result{-1};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (std::size_t chain = 0; chain < catenary_chain_size(); ++chain)
	{
		if (catenary_chain_get(chain) == nullptr)
			continue;
		auto const members{catenary_chain_members(chain)};
		for (std::size_t k = 0; k + 1 < members.size(); ++k)
		{
			auto const *a{catenary_scene_get(members[k])};
			auto const *b{catenary_scene_get(members[k + 1])};
			if (a == nullptr || b == nullptr || glm::distance(wire_of(*a), camera) > kCatenaryRange)
				continue;
			auto const distance{mouse_to_segment(projection, wire_of(*a), wire_of(*b))};
			if (distance < best)
			{
				best = distance;
				result = static_cast<int>(chain);
			}
		}
	}
	return result;
}

void catenary_editor::catenary_select(int const Index)
{
	auto &tool{m_catenary};
	tool.selected = Index;
	tool.probed = false;
	auto const *data{Index >= 0 ? catenary_scene_get(static_cast<std::size_t>(Index)) : nullptr};
	if (data == nullptr)
	{
		tool.selected = -1;
		return;
	}
	// the chain it's in is shown with it
	std::size_t position{0};
	tool.chain_selected = catenary_chain_of(static_cast<std::size_t>(Index), position);
	// the settings of the panel take what the suspension has, so that changing them changes it
	if (auto const found{std::find(tool.poles.begin(), tool.poles.end(), data->pole_type)}; found != tool.poles.end())
		tool.pole = static_cast<int>(found - tool.poles.begin());
	tool.execution = std::clamp(data->execution, 1, 5);
	tool.swinging = data->card / 100 != 12;
	// the group of its card in the described catalogue: changing the settings keeps it in that group
	if (auto const *family{catenary_scene_structures().family_of(data->card)}; family != nullptr)
	{
		tool.catalog_group = family->group;
		if (auto const *group{catenary_scene_structures().group(family->group)}; group != nullptr)
			tool.catalog_kind = group->category;
		tool.catalog_family.clear();
		tool.catalog_card = 0;
	}
	tool.insulators25 = data->insulators == "25";
	auto const decade{(data->card / 10) % 10};
	if (data->card / 100 == 11 || data->card / 100 == 12)
		tool.bend = (decade == 3 || decade == 4) ? 2 : 1;
	auto const *spot{catenary_spot_of(static_cast<std::size_t>(Index))};
	if (spot == nullptr)
	{
		tool.status = STR("The suspension isn't by any track: it can't be changed here");
		return;
	}
	auto const right{right_of(spot->forward) * static_cast<double>(spot->side)};
	auto const *catalog{catenary_scene_catalog()};
	auto const *pole{catalog != nullptr ? catalog->pole(data->pole_type) : nullptr};
	auto const railhead{spot->axis.y + kRailTop};
	auto const offset{glm::dot(plan_of(wire_of(*data)) - plan_of(spot->axis), right)};
	tool.stagger = static_cast<float>(std::abs(offset));
	tool.toward = offset >= 0.0;
	tool.height = static_cast<float>(data->at[1] - railhead);
	// its system height: own when it differs from the card's
	if (auto const *catalog{catenary_scene_catalog()}; catalog != nullptr && data->has_rope)
	{
		auto const *scheme{catalog->scheme(data->card, data->execution)};
		auto const hk{data->rope[1] - data->at[1]};
		tool.system_height = scheme != nullptr && std::abs(hk - scheme->height) > 0.005 ? static_cast<float>(hk) : 0.0f;
	}
	if (pole != nullptr)
		tool.clearance = static_cast<float>(glm::dot(plan_of(pole_of(*data)) - plan_of(spot->axis), right) - 0.5 * pole->depth(0.0));
}

void catenary_editor::catenary_apply(catenary_spot const &Spot)
{
	auto &tool{m_catenary};
	if (tool.selected < 0)
		return;
	auto const index{static_cast<std::size_t>(tool.selected)};
	auto const *current{catenary_scene_get(index)};
	if (current == nullptr || Spot.card == 0)
		return;
	auto data{Spot.data};
	// what the panel doesn't set stays as it was
	data.name = current->name;
	data.range_max = current->range_max;
	data.range_min = current->range_min;
	data.skin = current->skin;
	data.overrides = current->overrides;
	data.umbrella = current->umbrella;
	// dragging a slider changes it many times: all of that is taken back at once
	auto const &last{tool.undo.empty() ? catenary_change{} : tool.undo.back()};
	if (tool.undo.empty() || last.chain || last.index != index || last.removed || false == last.before.has_value())
		tool.undo.push_back({++tool.group, false, index, *current, std::nullopt, false});
	catenary_scene_update(index, data);
	tool.spots[index] = Spot;
	tool.spots[index].data = data;
	tool.probed = false;
}

// the poles of the chain get the settings of the selected one: pole, its face from the track, execution, height of the wires,
// insulators. Each keeps its own stagger, so the zigzag stays as it is
void catenary_editor::catenary_copy(std::vector<std::size_t> const &Targets)
{
	auto &tool{m_catenary};
	++tool.group;
	std::size_t copied{0}, failed{0};
	for (auto const index : Targets)
	{
		if (static_cast<int>(index) == tool.selected)
			continue;
		auto const *current{catenary_scene_get(index)};
		auto const *found{catenary_spot_of(index)};
		if (current == nullptr || found == nullptr)
		{
			++failed;
			continue;
		}
		auto spot{*found};
		auto const right{right_of(spot.forward) * static_cast<double>(spot.side)};
		auto const offset{glm::dot(plan_of(wire_of(*current)) - plan_of(spot.axis), right)};
		auto const *catalog{catenary_scene_catalog()};
		auto const *own{catalog != nullptr ? catalog->pole(current->pole_type) : nullptr};
		auto const drop{own != nullptr && false == own->drop.empty()};
		if (own != nullptr && own->gantry)
		{
			catenary_compose(spot, glm::dot(plan_of(pole_of(*current)) - plan_of(spot.axis), right) - 0.5 * own->depth(0.0), std::abs(offset), offset >= 0.0, current->pole_type);
			std::copy(std::begin(current->pole), std::end(current->pole), spot.data.pole);
		}
		else
			catenary_compose(spot, tool.clearance, std::abs(offset), offset >= 0.0, drop ? current->pole_type : std::string{});
		if (drop)
			spot.data.pole[1] = current->pole[1];
		if (spot.card == 0)
		{
			++failed;
			continue;
		}
		auto data{spot.data};
		data.name = current->name;
		data.range_max = current->range_max;
		data.range_min = current->range_min;
		data.skin = current->skin;
		data.overrides = current->overrides;
		data.umbrella = current->umbrella;
		tool.undo.push_back({tool.group, false, index, *current, std::nullopt, false});
		catenary_scene_update(index, data);
		tool.spots[index] = spot;
		tool.spots[index].data = data;
		++copied;
	}
	tool.status = failed == 0 ? format(STR_C("Settings copied to %zu poles"), copied) : format(STR_C("Settings copied to %zu poles, %zu couldn't take them (no card for their A or not by a track)"), copied, failed);
}

// the stagger the next pole of the chain gets: the other way than the last one on a straight; after the user put three the same
// way in a row they're kept that way, until one goes the other way again
bool catenary_editor::catenary_propose_toward(bool &Toward)
{
	auto &tool{m_catenary};
	if (false == tool.alternate || tool.last < 0)
		return false;
	std::vector<std::size_t> order;
	if (tool.chain >= 0 && catenary_chain_get(static_cast<std::size_t>(tool.chain)) != nullptr)
	{
		order = catenary_chain_members(static_cast<std::size_t>(tool.chain));
		if (false == tool.append)
			std::reverse(order.begin(), order.end());
	}
	else
		order = {static_cast<std::size_t>(tool.last)};
	int streak{0};
	bool last{true};
	for (auto item{order.rbegin()}; item != order.rend(); ++item)
	{
		auto const *spot{catenary_spot_of(*item)};
		if (spot == nullptr)
			break;
		if (streak == 0)
			last = spot->toward;
		else if (spot->toward != last || spot->curve)
			break;
		if (spot->curve)
		{
			// a curve decides the stagger itself: the next one simply goes the other way
			streak = 1;
			break;
		}
		++streak;
	}
	if (streak == 0)
		return false;
	Toward = streak >= 3 ? last : false == last;
	return true;
}

void catenary_editor::catenary_place(catenary_spot const &Spot)
{
	auto &tool{m_catenary};
	++tool.group;
	auto data{Spot.data};
	std::string const file{tool.file};
	data.name = catenary_scene_new_name(file);
	// a drop on a gantry takes the texture of the gantry
	if (auto const *gantry{tool.gantry_selected >= 0 ? catenary_gantry_get(static_cast<std::size_t>(tool.gantry_selected)) : nullptr}; gantry != nullptr && data.pole_type != tool.poles[std::clamp(tool.pole, 0, static_cast<int>(tool.poles.size()) - 1)])
		data.skin = gantry->skin;
	auto const index{catenary_scene_add(file, data)};
	tool.spots[index] = Spot;
	tool.spots[index].data = data;
	tool.undo.push_back({tool.group, false, index, std::nullopt, std::nullopt, false});
	tool.selected = -1;
	tool.status = format(STR_C("Pole %s with card %d execution %d placed, A = %.2f m"), data.pole_type.c_str(), Spot.card, data.execution, Spot.reach);
	if (catenary_scene_parts(index) == 0)
		tool.status += STR(" - the cantilever didn't assemble, see the log");
	// the span from the last pole: a new chain of the two, or one more pole of the chain
	auto const *previous{tool.chaining && tool.last >= 0 ? catenary_scene_get(static_cast<std::size_t>(tool.last)) : nullptr};
	if (previous != nullptr && catenary_scene_file_of(static_cast<std::size_t>(tool.last)) == file)
	{
		auto previousdata{*previous};
		if (previousdata.name.empty())
		{
			// a pole from before the editor had no name, a chain needs it
			tool.undo.push_back({tool.group, false, static_cast<std::size_t>(tool.last), previousdata, std::nullopt, false});
			previousdata.name = catenary_scene_new_name(file);
			catenary_scene_update(static_cast<std::size_t>(tool.last), previousdata);
		}
		auto const *chain{tool.chain >= 0 ? catenary_chain_get(static_cast<std::size_t>(tool.chain)) : nullptr};
		if (chain != nullptr && catenary_chain_file(static_cast<std::size_t>(tool.chain)) == file)
		{
			auto changed{*chain};
			tool.undo.push_back({tool.group, true, static_cast<std::size_t>(tool.chain), std::nullopt, *chain, false});
			if (tool.append)
				changed.suspensions.push_back(data.name);
			else
				changed.suspensions.insert(changed.suspensions.begin(), data.name);
			catenary_chain_update(static_cast<std::size_t>(tool.chain), changed);
		}
		else
		{
			catenary::chain created;
			created.name = catenary_scene_new_chain_name(file);
			created.power = tool.power;
			created.voltage = tool.voltage;
			created.material = kMaterials[std::clamp(tool.material, 0, 1)];
			created.state = kStateValues[std::clamp(tool.state, 0, 2)];
			created.type = tool.networks.empty() ? std::string{} : tool.networks[std::clamp(tool.network, 0, static_cast<int>(tool.networks.size()) - 1)];
			created.suspensions = {previousdata.name, data.name};
			tool.chain = static_cast<int>(catenary_chain_add(file, created));
			tool.append = true;
			tool.undo.push_back({tool.group, true, static_cast<std::size_t>(tool.chain), std::nullopt, std::nullopt, false});
		}
		tool.chain_selected = tool.chain;
	}
	else
		tool.chain = -1; // nothing to go on from: the next span starts a new chain
	tool.last = static_cast<int>(index);
	// the next one: the other way, unless the user keeps putting them the same way
	if (bool toward{tool.toward}; catenary_propose_toward(toward))
		tool.toward = toward;
}

// the span from the suspension the wires are being hung from (tool.connect_from) to To: To goes on the chain at its end,
// From goes on the chain of To at its end, the two chains become one, or the two make a new chain
void catenary_editor::catenary_connect(std::size_t const To)
{
	auto &tool{m_catenary};
	auto const from{static_cast<std::size_t>(tool.connect_from)};
	auto const *a{catenary_scene_get(from)};
	auto const *b{catenary_scene_get(To)};
	if (a == nullptr)
	{
		tool.connect_from = -1;
		return;
	}
	if (b == nullptr || To == from)
		return;
	auto const file{catenary_scene_file_of(from)};
	if (catenary_scene_file_of(To) != file)
	{
		tool.status = STR("The poles are in different files: a chain can't join them");
		return;
	}
	std::size_t pa{0}, pb{0};
	auto const ca{catenary_chain_of(from, pa)};
	auto const cb{catenary_chain_of(To, pb)};
	auto const *chaina{ca >= 0 ? catenary_chain_get(static_cast<std::size_t>(ca)) : nullptr};
	auto const *chainb{cb >= 0 ? catenary_chain_get(static_cast<std::size_t>(cb)) : nullptr};
	if (chaina != nullptr && ca == cb)
	{
		tool.status = STR("Both poles are in this chain already");
		return;
	}
	auto const end = [](catenary::chain const *Chain, std::size_t const Position) {
		return Chain == nullptr || Position == 0 || Position + 1 == Chain->suspensions.size();
	};
	if (false == end(chaina, pa) || false == end(chainb, pb))
	{
		tool.status = STR("The pole is in the middle of a chain: the spans go on from the ends of a chain only");
		return;
	}
	++tool.group;
	// a pole from before the editor had no name, a chain needs it
	auto const named = [&](std::size_t const Index) {
		auto data{*catenary_scene_get(Index)};
		if (false == data.name.empty())
			return data.name;
		tool.undo.push_back({tool.group, false, Index, data, std::nullopt, false});
		data.name = catenary_scene_new_name(file);
		catenary_scene_update(Index, data);
		return data.name;
	};
	auto const namea{named(from)};
	auto const nameb{named(To)};
	int result{-1};
	if (chaina != nullptr)
	{
		// From's chain with From last, then To (and To's chain from To on)
		auto changed{*chaina};
		if (pa == 0 && changed.suspensions.size() > 1)
			std::reverse(changed.suspensions.begin(), changed.suspensions.end());
		if (chainb != nullptr)
		{
			auto other{chainb->suspensions};
			if (pb != 0)
				std::reverse(other.begin(), other.end());
			changed.suspensions.insert(changed.suspensions.end(), other.begin(), other.end());
			tool.undo.push_back({tool.group, true, static_cast<std::size_t>(cb), std::nullopt, *chainb, false});
			catenary_chain_remove(static_cast<std::size_t>(cb));
		}
		else
			changed.suspensions.push_back(nameb);
		tool.undo.push_back({tool.group, true, static_cast<std::size_t>(ca), std::nullopt, *chaina, false});
		catenary_chain_update(static_cast<std::size_t>(ca), changed);
		result = ca;
	}
	else if (chainb != nullptr)
	{
		// From before To on To's chain
		auto changed{*chainb};
		if (pb == 0)
			changed.suspensions.insert(changed.suspensions.begin(), namea);
		else
			changed.suspensions.push_back(namea);
		tool.undo.push_back({tool.group, true, static_cast<std::size_t>(cb), std::nullopt, *chainb, false});
		catenary_chain_update(static_cast<std::size_t>(cb), changed);
		result = cb;
	}
	else
	{
		catenary::chain created;
		created.name = catenary_scene_new_chain_name(file);
		created.power = tool.power;
		created.voltage = tool.voltage;
		created.material = kMaterials[std::clamp(tool.material, 0, 1)];
		created.state = kStateValues[std::clamp(tool.state, 0, 2)];
		created.type = tool.networks.empty() ? std::string{} : tool.networks[std::clamp(tool.network, 0, static_cast<int>(tool.networks.size()) - 1)];
		created.suspensions = {namea, nameb};
		result = static_cast<int>(catenary_chain_add(file, created));
		tool.undo.push_back({tool.group, true, static_cast<std::size_t>(result), std::nullopt, std::nullopt, false});
	}
	tool.chain_selected = result;
	// the next click goes on from this pole
	tool.connect_from = static_cast<int>(To);
	tool.selected = static_cast<int>(To);
	tool.probed = false;
	tool.status = format(STR_C("Span %s - %s hung; LMB the next pole, or stop with the button"), namea.c_str(), nameb.c_str());
}

// what hangs on a span of a gantry: the drops on its beam and the cantilevers straight on its masts
void catenary_editor::catenary_gantry_hangers(std::size_t const Gantry, std::vector<std::pair<std::size_t, catenary::suspension>> &Hangers) const
{
	glm::dvec3 from, to;
	double bottom{0.0};
	auto const *catalog{catenary_scene_catalog()};
	auto const *gantry{catenary_gantry_get(Gantry)};
	if (catalog == nullptr || gantry == nullptr || false == catenary_gantry_beam(Gantry, from, to, bottom))
		return;
	for (std::size_t index = 0; index < catenary_scene_size(); ++index)
	{
		auto const *data{catenary_scene_get(index)};
		auto const *pole{data != nullptr && data->has_pole ? catalog->pole(data->pole_type) : nullptr};
		if (pole == nullptr || std::any_of(Hangers.begin(), Hangers.end(), [index](auto const &Hanger) { return Hanger.first == index; }))
			continue;
		glm::dvec3 const point{data->pole[0], data->pole[1], data->pole[2]};
		if (pole->gantry)
		{
			for (int k = 0; k < 2; ++k)
				if (gantry->masts[k] == data->pole_type && glm::length(plan_of(point) - plan_of(glm::dvec3{gantry->points[k][0], 0.0, gantry->points[k][2]})) < 0.1)
				{
					Hangers.emplace_back(index, *data);
					break;
				}
			continue;
		}
		if (pole->drop.empty())
			continue;
		auto const u{to - from};
		auto const along{std::clamp(glm::dot(point - from, u) / std::max(glm::dot(u, u), 1e-9), 0.0, 1.0)};
		if (glm::distance(point, from + u * along) < 0.3)
			Hangers.emplace_back(index, *data);
	}
}

void catenary_editor::catenary_press(track_intent const &Intent)
{
	auto &tool{m_catenary};
	tool.status.clear();
	auto const &mouse{ImGui::GetIO().MousePos};
	tool.pressed = {mouse.x, mouse.y};
	if (is_intent(Intent, intent_detail::select))
	{
		if (tool.connect_from >= 0)
		{
			// hanging the wires on: the span from the last pole to this one
			catenary_connect(static_cast<std::size_t>(Intent.index));
			return;
		}
		catenary_select(Intent.index);
		// a pole selected on the gantries page: the cantilever on the gantry, its settings are shown there; on the wires page it's
		// the pole the wires go from
		if (tool.page == 1 && tool.gantry_selected < 0)
			tool.page = 0;
		// held and dragged: the pole goes along the track
		tool.dragging = tool.selected >= 0 ? 1 : 0;
		return;
	}
	if (is_intent(Intent, intent_detail::gantry))
	{
		tool.gantry_selected = Intent.index;
		tool.selected = -1;
		tool.page = 1;
		tool.status = STR("The cantilevers now go on drops on this gantry: LMB over a track; drag the beam to move the gantry");
		// held and dragged: the whole gantry goes with the cursor
		tool.dragging = 2;
		tool.drag_start = e.cursor_ground();
		// held at its distance from the track: the first mast goes along the track, at the axis distance it has now
		if (auto const *gantry{catenary_gantry_get(static_cast<std::size_t>(Intent.index))}; gantry != nullptr)
		{
			tool.drag_anchor = {gantry->points[0][0], gantry->points[0][1], gantry->points[0][2]};
			double face{0.0};
			glm::dvec2 away{0.0};
			auto const *mast{catenary_scene_catalog() != nullptr ? catenary_scene_catalog()->mast(gantry->masts[0]) : nullptr};
			tool.drag_axis = catenary_mast_face(gantry->masts[0], tool.drag_anchor, face, away) ? face + (mast != nullptr ? mast->offset : 0.0) : 0.0;
		}
		// a gantry of many spans goes whole: all the spans joined on their masts, with what hangs on them
		tool.drag_group.clear();
		tool.drag_undo = 0;
		tool.drag_drops.clear();
		for (auto const span : gantry_group(static_cast<std::size_t>(tool.gantry_selected)))
		{
			tool.drag_group.emplace_back(span, *catenary_gantry_get(span));
			catenary_gantry_hangers(span, tool.drag_drops);
		}
		return;
	}
	if (is_intent(Intent, intent_detail::mast))
	{
		auto const point{catenary_mast_point()};
		if (false == tool.gantry_first.has_value())
		{
			tool.gantry_first = point;
			tool.gantry_track.reset();
			// the track nearest to the first mast (within 20 m): the beam can go square to it
			auto best{20.0};
			for (auto *track : simulation::Paths.sequence())
			{
				if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
					continue;
				for (int path = 0; path < static_cast<int>(track->m_paths.size()); ++path)
				{
					bezier const curve{track->m_paths[path]};
					if (geometry::plan_distance(curve.point(0.5), point) > curve.plan_length() * 0.5 + best)
						continue;
					auto const parameter{nearest_parameter(*track, path, point)};
					auto const distance{geometry::plan_distance(curve.point(parameter), point)};
					if (distance < best)
					{
						best = distance;
						tool.gantry_track = forward_at(*track, path, parameter);
						// the gantry stands at the height of the rail heads of that track
						tool.gantry_first->y = curve.point(parameter).y + kRailTop;
					}
				}
			}
			return;
		}
		glm::dvec3 second;
		std::string beam;
		if (false == catenary_gantry_second(point, second, beam) || tool.masts.empty())
			return;
		catenary::gantry gantry;
		std::string const file{tool.file};
		gantry.name = catenary_scene_new_gantry_name(file);
		gantry.beam = beam;
		gantry.high = tool.gantry_high;
		gantry.side = 1;
		gantry.skin = tool.gantry_skin;
		gantry.masts[0] = tool.masts[std::clamp(tool.mast_first, 0, static_cast<int>(tool.masts.size()) - 1)];
		gantry.masts[1] = tool.gantry_single ? std::string{} : tool.masts[std::clamp(tool.mast_second, 0, static_cast<int>(tool.masts.size()) - 1)];
		auto const first{*tool.gantry_first};
		gantry.points[0][0] = first.x;
		gantry.points[0][1] = first.y;
		gantry.points[0][2] = first.z;
		gantry.points[1][0] = second.x;
		gantry.points[1][1] = second.y;
		gantry.points[1][2] = second.z;
		auto const index{catenary_gantry_add(file, gantry)};
		catenary_change change;
		change.group = ++tool.group;
		change.gantry = true;
		change.index = index;
		tool.undo.push_back(change);
		tool.gantry_selected = static_cast<int>(index);
		tool.gantry_placing = false;
		tool.gantry_single = false;
		tool.gantry_first.reset();
		tool.status = format(STR_C("Gantry %s with beam %s placed; LMB over a track puts a drop with a cantilever on it"), gantry.name.c_str(), beam.c_str());
		double face{0.0};
		glm::dvec2 away{0.0};
		if (false == gantry.masts[1].empty() && catenary_mast_face(gantry.masts[1], second, face, away))
			tool.status += format(STR_C(" (second mast %.2f m from the track axis)"), face);
		return;
	}
	if (is_intent(Intent, intent_detail::chain))
	{
		tool.selected = -1;
		tool.page = 2;
		tool.chain_selected = Intent.index;
		if (auto const *chain{catenary_chain_get(static_cast<std::size_t>(Intent.index))}; chain != nullptr)
		{
			// the panel shows what the chain has
			std::snprintf(tool.power, sizeof(tool.power), "%s", chain->power.c_str());
			tool.voltage = static_cast<float>(chain->voltage);
			tool.material = chain->material == "al" ? 1 : 0;
			tool.state = chain->state == 2 ? 1 : chain->state == 1 ? 2 : 0;
			if (auto const found{std::find(tool.networks.begin(), tool.networks.end(), chain->type)}; found != tool.networks.end())
				tool.network = static_cast<int>(found - tool.networks.begin());
		}
		return;
	}
	if (Intent.track == nullptr)
		return;
	catenary_spot spot;
	if (false == catenary_spot_at(*Intent.track, Intent.path, e.m_hover.point, e.cursor_level(e.m_hover.point.y), 0, spot))
		return;
	if (spot.card == 0)
	{
		tool.status = spot.problem;
		return;
	}
	catenary_place(spot);
}

// the panel settings of the chain go to the selected chain
void catenary_editor::catenary_chain_apply()
{
	auto &tool{m_catenary};
	if (tool.chain_selected < 0)
		return;
	auto const index{static_cast<std::size_t>(tool.chain_selected)};
	auto const *chain{catenary_chain_get(index)};
	if (chain == nullptr)
		return;
	auto changed{*chain};
	changed.power = tool.power;
	changed.voltage = tool.voltage;
	changed.material = kMaterials[std::clamp(tool.material, 0, 1)];
	changed.state = kStateValues[std::clamp(tool.state, 0, 2)];
	if (false == tool.networks.empty())
		changed.type = tool.networks[std::clamp(tool.network, 0, static_cast<int>(tool.networks.size()) - 1)];
	auto const &last{tool.undo.empty() ? catenary_change{} : tool.undo.back()};
	if (tool.undo.empty() || false == last.chain || last.index != index || false == last.chain_before.has_value())
		tool.undo.push_back({++tool.group, true, index, std::nullopt, *chain, false});
	catenary_chain_update(index, changed);
}

bool catenary_editor::catenary_delete_selected()
{
	auto &tool{m_catenary};
	tool.connect_from = -1;
	if (tool.selected < 0)
		return false;
	auto const index{static_cast<std::size_t>(tool.selected)};
	auto const *current{catenary_scene_get(index)};
	if (current == nullptr)
		return false;
	++tool.group;
	// the chains it's in lose it: their lists are kept to be taken back
	for (std::size_t chain = 0; chain < catenary_chain_size(); ++chain)
	{
		auto const *data{catenary_chain_get(chain)};
		if (data != nullptr && false == current->name.empty() && catenary_chain_file(chain) == catenary_scene_file_of(index)
		    && std::find(data->suspensions.begin(), data->suspensions.end(), current->name) != data->suspensions.end())
			tool.undo.push_back({tool.group, true, chain, std::nullopt, *data, false});
	}
	tool.undo.push_back({tool.group, false, index, *current, std::nullopt, true});
	catenary_scene_remove(index);
	tool.selected = -1;
	if (tool.last == static_cast<int>(index))
		tool.last = -1;
	tool.status = STR("Pole removed");
	return true;
}

bool catenary_editor::catenary_undo()
{
	auto &tool{m_catenary};
	tool.connect_from = -1;
	// in the middle of a drag, or with nothing of its own: the editor takes back its own changes
	if (tool.dragging != 0 || tool.undo.empty())
		return false;
	auto const group{tool.undo.back().group};
	while (false == tool.undo.empty() && tool.undo.back().group == group)
	{
		auto const change{tool.undo.back()};
		tool.undo.pop_back();
		if (change.anchor)
		{
			if (change.anchor_before.has_value())
				catenary_anchor_update(change.index, *change.anchor_before);
			else
				catenary_anchor_remove(change.index);
			continue;
		}
		if (change.gantry)
		{
			if (change.gantry_before.has_value())
				catenary_gantry_update(change.index, *change.gantry_before);
			else
			{
				catenary_gantry_remove(change.index);
				if (tool.gantry_selected == static_cast<int>(change.index))
					tool.gantry_selected = -1;
			}
			continue;
		}
		if (change.chain)
		{
			if (change.chain_before.has_value())
				catenary_chain_update(change.index, *change.chain_before);
			else
			{
				catenary_chain_remove(change.index);
				if (tool.chain == static_cast<int>(change.index))
					tool.chain = -1;
			}
			continue;
		}
		if (false == change.before.has_value())
		{
			catenary_scene_remove(change.index);
			if (tool.last == static_cast<int>(change.index))
				tool.last = -1;
		}
		else if (change.removed)
			catenary_scene_restore(change.index, *change.before);
		else
			catenary_scene_update(change.index, *change.before);
		tool.spots.erase(change.index);
	}
	// the next span goes on from the end of the chain as it is now
	if (tool.chain >= 0 && catenary_chain_get(static_cast<std::size_t>(tool.chain)) != nullptr)
	{
		auto const members{catenary_chain_members(static_cast<std::size_t>(tool.chain))};
		tool.last = members.empty() ? -1 : static_cast<int>(tool.append ? members.back() : members.front());
	}
	else if (tool.last >= 0 && catenary_scene_get(static_cast<std::size_t>(tool.last)) == nullptr)
		tool.last = -1;
	tool.selected = -1;
	tool.probed = false;
	tool.status = STR("Taken back");
	return true;
}

void catenary_editor::render_catenary_chain_ui()
{
	auto &tool{m_catenary};
	auto const *chain{tool.chain_selected >= 0 ? catenary_chain_get(static_cast<std::size_t>(tool.chain_selected)) : nullptr};
	ImGui::Separator();
	if (chain == nullptr)
	{
		ImGui::TextDisabled("%s", STR_C("New chains take these settings; LMB on a span selects its chain"));
	}
	else
	{
		ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "%s", format(STR_C("Chain %s: %zu poles, %zu spans; changes apply to all of them"), chain->name.c_str(), chain->suspensions.size(), chain->suspensions.size() - 1).c_str());
	}
	auto changed{false};
	ImGui::PushItemWidth(180.0f);
	if (false == tool.networks.empty() && ImGui::BeginCombo(STR_C("Network type"), tool.networks[std::clamp(tool.network, 0, static_cast<int>(tool.networks.size()) - 1)].c_str()))
	{
		for (int i = 0; i < static_cast<int>(tool.networks.size()); ++i)
			if (ImGui::Selectable(tool.networks[i].c_str(), i == tool.network))
			{
				tool.network = i;
				changed = true;
			}
		ImGui::EndCombo();
	}
	if (ImGui::Combo(STR_C("Material"), &tool.material, kMaterials, 2))
		changed = true;
	std::string const statenames[] = {STR(kStates[0]), STR(kStates[1]), STR(kStates[2])};
	char const *const states[] = {statenames[0].c_str(), statenames[1].c_str(), statenames[2].c_str()};
	if (ImGui::Combo(STR_C("Wires"), &tool.state, states, 3))
		changed = true;
	ImGui::InputText(STR_C("Power source"), tool.power, sizeof(tool.power));
	if (ImGui::IsItemDeactivatedAfterEdit())
		changed = true;
	if (ImGui::InputFloat(STR_C("Voltage (V)"), &tool.voltage, 0.0f, 0.0f, "%.0f"))
		changed = true;
	ImGui::PopItemWidth();
	if (changed && chain != nullptr)
		catenary_chain_apply();
	if (chain != nullptr)
	{
		if (ImGui::Button(STR_C("Continue the chain from its end")))
		{
			tool.chain = tool.chain_selected;
			tool.append = true;
			auto const members{catenary_chain_members(static_cast<std::size_t>(tool.chain))};
			tool.last = members.empty() ? -1 : static_cast<int>(members.back());
			tool.selected = -1;
			if (bool toward{tool.toward}; catenary_propose_toward(toward))
				tool.toward = toward;
		}
		ImGui::SameLine();
		if (ImGui::Button(STR_C("from its start")))
		{
			tool.chain = tool.chain_selected;
			tool.append = false;
			auto const members{catenary_chain_members(static_cast<std::size_t>(tool.chain))};
			tool.last = members.empty() ? -1 : static_cast<int>(members.front());
			tool.selected = -1;
			if (bool toward{tool.toward}; catenary_propose_toward(toward))
				tool.toward = toward;
		}
		render_catenary_anchor_ui(static_cast<std::size_t>(tool.chain_selected));
	}
}

std::size_t catenary_editor::catenary_anchor_of(catenary::chain const &Chain, std::string const &File, bool const Start) const
{
	auto const &name{Start ? Chain.anchor_start : Chain.anchor_end};
	if (name.empty())
		return catenary_anchor_size();
	for (std::size_t index = 0; index < catenary_anchor_size(); ++index)
		if (auto const *anchor{catenary_anchor_get(index)}; anchor != nullptr && anchor->name == name && catenary_anchor_file(index) == File)
			return index;
	return catenary_anchor_size();
}

namespace
{

// the colours of the weights as the models have them (executions 1-4)
char const *const kWeightColors[] = {"zolty", "szary", "zolty_pedzel", "szary_brzydki"};
char const *const kWeightColorNames[] = {"yellow", "grey", "yellow, brushed", "grey, shabby"};

int weight_color_index(std::string const &Name)
{
	for (int i = 0; i < 4; ++i)
		if (Name == kWeightColors[i])
			return i;
	return 1;
}

} // namespace

void catenary_editor::render_catenary_anchor_ui(std::size_t const Chain)
{
	auto &tool{m_catenary};
	auto const *chain{catenary_chain_get(Chain)};
	if (chain == nullptr)
		return;
	auto const file{catenary_chain_file(Chain)};
	auto const members{catenary_chain_members(Chain)};
	if (members.size() < 2)
		return;
	ImGui::Separator();
	ImGui::TextUnformatted(STR_C("Anchors (weight tensioning, card 23-0461)"));
	for (int end = 0; end < 2; ++end)
	{
		auto const start{end == 0};
		ImGui::PushID(end);
		// the last two poles at this end: the wires come along them to the anchor
		auto const *last{catenary_scene_get(start ? members.front() : members.back())};
		auto const *previous{catenary_scene_get(start ? members[1] : members[members.size() - 2])};
		auto const index{catenary_anchor_of(*chain, file, start)};
		auto const *anchor{index < catenary_anchor_size() ? catenary_anchor_get(index) : nullptr};
		if (last == nullptr || previous == nullptr || false == last->has_at || false == previous->has_at)
		{
			ImGui::PopID();
			continue;
		}
		// along the wires, away from the chain
		glm::dvec2 along{last->at[0] - previous->at[0], last->at[2] - previous->at[2]};
		if (glm::length(along) < 0.1)
		{
			ImGui::PopID();
			continue;
		}
		along = glm::normalize(along);
		auto const base{last->has_pole ? glm::dvec2{last->pole[0], last->pole[2]} : glm::dvec2{last->at[0], last->at[2]}};
		auto const baseheight{last->has_pole ? last->pole[1] : last->at[1] - static_cast<double>(tool.height)};
		if (anchor == nullptr)
		{
			if (ImGui::Button(start ? STR_C("Anchor the start of the chain") : STR_C("Anchor the end of the chain")))
			{
				// the anchor pole 24 m past the last pole, in line with it (catalogue 22 m; Glowacki 24 m for big tensions)
				catenary::anchor data;
				data.name = catenary_scene_new_anchor_name(file);
				data.network = chain->type;
				data.pole_type = tool.poles.empty() ? std::string{} : tool.poles[std::clamp(tool.pole, 0, static_cast<int>(tool.poles.size()) - 1)];
				auto const at{base + along * 24.0};
				data.pole[0] = at.x;
				data.pole[1] = baseheight;
				data.pole[2] = at.y;
				std::copy(std::begin(last->at), std::end(last->at), data.toward);
				// from the middle anchor: half of the chain, by default
				double length{0.0};
				for (std::size_t k = 1; k < members.size(); ++k)
				{
					auto const *a{catenary_scene_get(members[k - 1])};
					auto const *b{catenary_scene_get(members[k])};
					if (a != nullptr && b != nullptr)
						length += std::hypot(b->at[0] - a->at[0], b->at[2] - a->at[2]);
				}
				data.distance = std::clamp(std::round(0.5 * length / 10.0) * 10.0, 100.0, 750.0);
				data.color = "szary";
				data.top_color = "zolty";
				auto changed{*chain};
				(start ? changed.anchor_start : changed.anchor_end) = data.name;
				++tool.group;
				catenary_change added;
				added.group = tool.group;
				added.anchor = true;
				added.index = catenary_anchor_add(file, data);
				tool.undo.push_back(added);
				catenary_change linked;
				linked.group = tool.group;
				linked.chain = true;
				linked.index = Chain;
				linked.chain_before = *chain;
				tool.undo.push_back(linked);
				catenary_chain_update(Chain, changed);
				tool.status = STR("Anchored: the last span goes without droppers to the tensioning device");
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("An anchor pole 24 m past the last pole, in line with it, with the weight tensioning device; the last span hangs without droppers"));
			ImGui::PopID();
			continue;
		}
		ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "%s", format(start ? STR_C("Start: anchor %s") : STR_C("End: anchor %s"), anchor->name.c_str()).c_str());
		auto data{*anchor};
		auto edited{false};
		ImGui::PushItemWidth(180.0f);
		{
			// the anchor pole along the line of the last two poles
			glm::dvec2 const pole{data.pole[0], data.pole[2]};
			auto distance{static_cast<float>(glm::dot(pole - base, along))};
			if (ImGui::SliderFloat(STR_C("Anchor pole from the last pole (m)"), &distance, 10.0f, 40.0f, "%.1f"))
			{
				auto const at{base + along * static_cast<double>(distance)};
				data.pole[0] = at.x;
				data.pole[2] = at.y;
				edited = true;
			}
		}
		if (ImGui::BeginCombo(STR_C("Anchor pole"), data.pole_type.c_str()))
		{
			auto chosen{static_cast<int>(std::find(tool.poles.begin(), tool.poles.end(), data.pole_type) - tool.poles.begin())};
			if (grouped_items(tool.poles, chosen) && chosen >= 0 && chosen < static_cast<int>(tool.poles.size()))
			{
				data.pole_type = tool.poles[chosen];
				edited = true;
			}
			ImGui::EndCombo();
		}
		{
			auto distance{static_cast<float>(data.distance)};
			if (ImGui::SliderFloat(STR_C("From the middle anchor (m)"), &distance, 50.0f, 900.0f, "%.0f"))
			{
				data.distance = distance;
				edited = true;
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Half-section length (from the middle or fixed anchor): the weights and the pulley move with the air temperature this much more"));
		}
		std::string const colornames[] = {STR(kWeightColorNames[0]), STR(kWeightColorNames[1]), STR(kWeightColorNames[2]), STR(kWeightColorNames[3])};
		char const *const colors[] = {colornames[0].c_str(), colornames[1].c_str(), colornames[2].c_str(), colornames[3].c_str()};
		{
			auto color{weight_color_index(data.color)};
			if (ImGui::Combo(STR_C("Weights"), &color, colors, 4))
			{
				data.color = kWeightColors[color];
				edited = true;
			}
			auto top{data.top_color.empty() ? color : weight_color_index(data.top_color)};
			if (ImGui::Combo(STR_C("Top weight"), &top, colors, 4))
			{
				data.top_color = kWeightColors[top];
				edited = true;
			}
		}
		{
			auto missing{data.missing};
			if (ImGui::SliderInt(STR_C("Weights missing"), &missing, 0, 10))
			{
				data.missing = missing;
				edited = true;
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Stolen from the top of the stack: about 100 daN less tension each"));
			auto elongation{static_cast<float>(data.elongation * 100.0)};
			if (ImGui::SliderFloat(STR_C("Wires stretched since regulated (cm)"), &elongation, 0.0f, 60.0f, "%.0f"))
			{
				data.elongation = elongation / 100.0;
				edited = true;
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Creep and wear of an old network: the weights hang 4 times this much lower; on the ground the network loses its tension"));
		}
		ImGui::PopItemWidth();
		if (edited)
		{
			// one undo step for one drag of a slider
			auto const &lastchange{tool.undo.empty() ? catenary_change{} : tool.undo.back()};
			if (tool.undo.empty() || false == lastchange.anchor || lastchange.index != index || false == ImGui::IsMouseDragging(0))
			{
				catenary_change change;
				change.group = ++tool.group;
				change.anchor = true;
				change.index = index;
				change.anchor_before = *anchor;
				tool.undo.push_back(change);
			}
			catenary_anchor_update(index, data);
		}
		if (auto const *result{catenary_anchor_result(index)}; result != nullptr)
		{
			ImGui::TextDisabled("%s", format(STR_C("%.1f deg C: pulley %.2f m from the pole, weights %.2f m over the rails; %d weights, messenger %.0f daN, contact wires %.0f daN"),
			                                   Global.AirTemperature, result->b, result->s, result->weights, result->messenger_tension, result->contact_tension).c_str());
			for (auto const &warning : result->warnings)
				ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%s", warning.c_str());
		}
		if (ImGui::Button(STR_C("Remove the anchor")))
		{
			auto changed{*chain};
			(start ? changed.anchor_start : changed.anchor_end).clear();
			++tool.group;
			catenary_change removed;
			removed.group = tool.group;
			removed.anchor = true;
			removed.index = index;
			removed.anchor_before = *anchor;
			tool.undo.push_back(removed);
			catenary_change unlinked;
			unlinked.group = tool.group;
			unlinked.chain = true;
			unlinked.index = Chain;
			unlinked.chain_before = *chain;
			tool.undo.push_back(unlinked);
			catenary_anchor_remove(index);
			catenary_chain_update(Chain, changed);
		}
		ImGui::PopID();
	}
}

// the cantilever from the described catalogue: category, group, family (automatic by the track), card (automatic by A),
// execution. Returns true when anything changed
bool catenary_editor::render_catenary_catalog_ui()
{
	auto &tool{m_catenary};
	auto const &structures{catenary_scene_structures()};
	auto const *catalog{catenary_scene_catalog()};
	auto changed{false};
	auto const *group{structures.group(tool.catalog_group)};
	std::string categorytitle;
	for (auto const &category : structures.categories())
		if (category.id == tool.catalog_kind)
			categorytitle = category.title;
	if (ImGui::BeginCombo(STR_C("Kind"), categorytitle.c_str()))
	{
		for (auto const &category : structures.categories())
		{
			if (ImGui::Selectable(category.title.c_str(), category.id == tool.catalog_kind) && category.id != tool.catalog_kind)
			{
				tool.catalog_kind = category.id;
				// the first group of the kind
				for (auto const &candidate : structures.groups())
					if (candidate.category == category.id)
					{
						tool.catalog_group = candidate.id;
						break;
					}
				tool.catalog_family.clear();
				tool.catalog_card = 0;
				changed = true;
			}
			if (ImGui::IsItemHovered() && false == category.text.empty())
				ImGui::SetTooltip("%s", category.text.c_str());
		}
		ImGui::EndCombo();
	}
	group = structures.group(tool.catalog_group);
	ImGui::PushItemWidth(320.0f);
	if (ImGui::BeginCombo(STR_C("Group"), group != nullptr ? group->title.c_str() : ""))
	{
		for (auto const &candidate : structures.groups())
		{
			if (candidate.category != tool.catalog_kind)
				continue;
			if (ImGui::Selectable(candidate.title.c_str(), candidate.id == tool.catalog_group) && candidate.id != tool.catalog_group)
			{
				tool.catalog_group = candidate.id;
				tool.catalog_family.clear();
				tool.catalog_card = 0;
				changed = true;
			}
			if (ImGui::IsItemHovered() && false == candidate.text.empty())
				ImGui::SetTooltip("%s", candidate.text.c_str());
		}
		ImGui::EndCombo();
	}
	group = structures.group(tool.catalog_group);
	if (group != nullptr && false == group->text.empty())
	{
		ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 480.0f);
		ImGui::TextDisabled("%s", group->text.c_str());
		ImGui::PopTextWrapPos();
	}
	// the family: by the track and the stagger, or the one chosen
	auto const *family{structures.family(tool.catalog_family)};
	auto const automatic{STR("by the track and the stagger")};
	if (ImGui::BeginCombo(STR_C("Family"), family != nullptr ? family->title.c_str() : automatic.c_str()))
	{
		if (ImGui::Selectable(automatic.c_str(), family == nullptr))
		{
			tool.catalog_family.clear();
			tool.catalog_card = 0;
			changed = true;
		}
		for (auto const &candidate : structures.families())
		{
			if (candidate.group != tool.catalog_group)
				continue;
			if (ImGui::Selectable(candidate.title.c_str(), candidate.id == tool.catalog_family))
			{
				tool.catalog_family = candidate.id;
				tool.catalog_card = 0;
				changed = true;
			}
			if (ImGui::IsItemHovered() && false == candidate.text.empty())
				ImGui::SetTooltip("%s", candidate.text.c_str());
		}
		ImGui::EndCombo();
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Automatic: the family for a straight or a curve and for the wires towards or away from the pole, as the place by the track has it"));
	// the card: by the distance A, or the one chosen (of the chosen family)
	family = structures.family(tool.catalog_family);
	if (family != nullptr && family->cards.size() > 1)
	{
		auto const byreach{STR("by the distance A")};
		if (ImGui::BeginCombo(STR_C("Card"), tool.catalog_card > 0 ? format("%04d", tool.catalog_card).c_str() : byreach.c_str()))
		{
			if (ImGui::Selectable(byreach.c_str(), tool.catalog_card <= 0))
			{
				tool.catalog_card = 0;
				changed = true;
			}
			for (auto const card : family->cards)
			{
				if (catalog == nullptr || std::none_of(catalog->schemes().begin(), catalog->schemes().end(), [card](auto const &Scheme) { return Scheme.first.first == card; }))
					continue;
				if (ImGui::Selectable(format("%04d", card).c_str(), card == tool.catalog_card))
				{
					tool.catalog_card = card;
					changed = true;
				}
			}
			ImGui::EndCombo();
		}
	}
	// the execution: chosen, or from the band A
	if (group != nullptr && group->execution_user)
	{
		std::string current{format(STR_C("execution %d"), tool.execution)};
		for (auto const &execution : group->executions)
			if (execution.first == tool.execution)
				current = execution.second;
		if (ImGui::BeginCombo(STR_C("Execution"), current.c_str()))
		{
			for (auto const &execution : group->executions)
				if (ImGui::Selectable(execution.second.c_str(), execution.first == tool.execution))
				{
					tool.execution = execution.first;
					changed = true;
				}
			ImGui::EndCombo();
		}
	}
	else if (group != nullptr)
		ImGui::TextDisabled("%s", STR_C("Execution: from the distance A (the band of the card)"));
	ImGui::PopItemWidth();
	return changed;
}

// the wires page: the selected chain and its settings, the spans hung from the selected pole to the next ones
void catenary_editor::render_catenary_wires_ui()
{
	auto &tool{m_catenary};
	auto const *data{tool.selected >= 0 ? catenary_scene_get(static_cast<std::size_t>(tool.selected)) : nullptr};
	if (data == nullptr)
		ImGui::TextDisabled("%s", STR_C("LMB on a pole selects it: the wires can then be hung from it to the next poles; LMB on a span selects its chain"));
	else
	{
		ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "%s", format(STR_C("Pole %s"), data->name.empty() ? "-" : data->name.c_str()).c_str());
		if (ImGui::Button(tool.connect_from >= 0 ? STR_C("Stop hanging the wires") : STR_C("Hang the wires to another pole")))
		{
			tool.connect_from = tool.connect_from >= 0 ? -1 : tool.selected;
			tool.status = tool.connect_from >= 0 ? STR("LMB the next pole: the span from this one is hung to it; then the next one, and so on") : std::string{};
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Joins this pole with the poles clicked after it: it goes on the chain at its end, a new chain of the two, or two chains become one"));
	}
	render_catenary_chain_ui();
}

void catenary_editor::catenary_choose_page(int const Page)
{
	auto &tool{m_catenary};
	if (Page == tool.page)
		return;
	tool.page = Page;
	// what was going on in the page left stays behind
	tool.gantry_placing = false;
	tool.gantry_single = false;
	tool.gantry_first.reset();
	tool.connect_from = -1;
	if (Page != 1)
		tool.gantry_selected = -1; // the poles go by the track again, not on the drops of a gantry
	tool.status.clear();
}

// the settings of the cantilevers (the next ones placed, or the selected one) and the selected pole
void catenary_editor::render_catenary_cantilever_ui()
{
	auto &tool{m_catenary};
	auto const *catalog{catenary_scene_catalog()};
	auto changed{false};
	ImGui::PushItemWidth(180.0f);
	if (tool.selected < 0)
	{
		ImGui::InputText(STR_C("Catenary file"), tool.file, sizeof(tool.file));
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("File in the scenery directory the new poles go to; a new one is added to the scenery with the directive catenary <file> endcatenary when it's saved"));
		ImGui::Checkbox(STR_C("Chain: a span from the last pole"), &tool.chaining);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Each pole placed is joined with a span to the one placed before it, one chain from anchor to anchor"));
		if (tool.last >= 0 && catenary_scene_get(static_cast<std::size_t>(tool.last)) != nullptr)
		{
			ImGui::SameLine();
			if (ImGui::Button(STR_C("End the chain")))
			{
				tool.chain = -1;
				tool.last = -1;
			}
		}
	}
	else
		ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "%s", format(STR_C("Selected pole (%s): changes apply to it at once"), catenary_scene_file_of(static_cast<std::size_t>(tool.selected)).c_str()).c_str());

	// on the gantries the drop or the mast is the pole, its distance is the one of the gantry
	if (tool.page == 0)
	{
		if (ImGui::BeginCombo(STR_C("Pole"), tool.poles[std::clamp(tool.pole, 0, static_cast<int>(tool.poles.size()) - 1)].c_str()))
		{
			if (grouped_items(tool.poles, tool.pole))
				changed = true;
			ImGui::EndCombo();
		}
		changed |= ImGui::SliderFloat(STR_C("Face of the pole from the track axis (m)"), &tool.clearance, 1.60f, 4.50f, "%.2f");
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Structure gauge 2.20 or 2.50 m by the catalogue (card 0007); drag it and the cantilever follows, the card changes with band A. Ctrl+click types a value"));
	}
	changed |= ImGui::SliderFloat(STR_C("Stagger of the wires (m)"), &tool.stagger, 0.0f, 0.45f, "%.2f");
	if (ImGui::Checkbox(STR_C("Wires towards the pole"), &tool.toward))
		changed = true;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("On a straight the wires zigzag: towards the pole and away from it in turn. On a curve they go to its outside, which decides it"));
	ImGui::SameLine();
	ImGui::Checkbox(STR_C("In turn"), &tool.alternate);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("The next pole of the chain gets the wires the other way than the last one; after three the same way in a row it keeps that way, until one goes the other way"));
	std::string const bendnames[] = {STR("by the track"), STR("straight"), STR("curve")};
	char const *const bends[] = {bendnames[0].c_str(), bendnames[1].c_str(), bendnames[2].c_str()};
	if (ImGui::Combo(STR_C("Cards for"), &tool.bend, bends, 3))
		changed = true;
	if (false == catenary_scene_structures().empty())
		changed |= render_catenary_catalog_ui();
	else
	{
		if (ImGui::Checkbox(STR_C("Swinging cantilever (11xx)"), &tool.swinging))
			changed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Off: the fixed ones (12xx)"));
		if (ImGui::BeginCombo(STR_C("Execution"), STR(kExecutions[std::clamp(tool.execution, 1, 5) - 1]).c_str()))
		{
			for (int i = 1; i <= 5; ++i)
				if (ImGui::Selectable(STR(kExecutions[i - 1]).c_str(), i == tool.execution))
				{
					tool.execution = i;
					changed = true;
				}
			ImGui::EndCombo();
		}
	}
	changed |= ImGui::SliderFloat(STR_C("Contact wires over the rails (m)"), &tool.height, 4.90f, 6.50f, "%.2f");
	{
		// the system height (hk): from the card, or set by hand
		auto own{tool.system_height > 0.0f};
		if (ImGui::Checkbox(STR_C("Own system height"), &own))
		{
			tool.system_height = own ? 1.70f : 0.0f;
			changed = true;
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Off: the system height of the card (1.70 / 1.30 ...). On: the messenger at the height set here over the contact wires; the stay mount goes with it, the solver warns when the cantilever can't take it"));
		if (own)
		{
			ImGui::SameLine();
			ImGui::PushItemWidth(100.0f);
			changed |= ImGui::SliderFloat(STR_C("hk (m)"), &tool.system_height, 0.25f, 2.50f, "%.2f");
			ImGui::PopItemWidth();
		}
	}
	changed |= ImGui::Checkbox(STR_C("25 kV insulators"), &tool.insulators25);
	ImGui::RadioButton(STR_C("Wind zone I"), &tool.zone, 1);
	ImGui::SameLine();
	ImGui::RadioButton(STR_C("Wind zone II"), &tool.zone, 2);
	if (tool.networks.empty())
		ImGui::TextDisabled("%s", STR_C("No data/traction_layout.txt: the spans aren't checked"));
	ImGui::PopItemWidth();

	// the selected pole follows the settings at once
	if (tool.selected >= 0 && changed)
	{
		if (auto const *found{catenary_spot_of(static_cast<std::size_t>(tool.selected))}; found != nullptr)
		{
			auto spot{*found};
			spot.curve = tool.bend == 2 || (tool.bend == 0 && spot.radius > 0.0 && spot.radius < kCurveRadius);
			// a drop on a gantry stays the drop it is, hanging from the beam
			auto const *current{catenary_scene_get(static_cast<std::size_t>(tool.selected))};
			auto const *own{current != nullptr ? catalog->pole(current->pole_type) : nullptr};
			auto const drop{own != nullptr && false == own->drop.empty()};
			if (own != nullptr && own->gantry)
			{
				// on a mast of a gantry: the mast stays, the distance is its own
				auto const right{right_of(spot.forward) * static_cast<double>(spot.side)};
				catenary_compose(spot, glm::dot(plan_of(pole_of(*current)) - plan_of(spot.axis), right) - 0.5 * own->depth(0.0), tool.stagger, tool.toward, current->pole_type);
				std::copy(std::begin(current->pole), std::end(current->pole), spot.data.pole);
			}
			else
				catenary_compose(spot, tool.clearance, tool.stagger, tool.toward, drop ? current->pole_type : std::string{});
			if (drop)
				spot.data.pole[1] = current->pole[1];
			if (spot.card != 0)
				catenary_apply(spot);
			else
				tool.status = spot.problem;
		}
	}
	if (tool.selected >= 0)
	{
		auto const *data{catenary_scene_get(static_cast<std::size_t>(tool.selected))};
		if (data == nullptr)
			tool.selected = -1;
		else
		{
			if (false == tool.probed)
			{
				tool.probe_good = catenary_scene_probe(*data, tool.probe);
				tool.probed = true;
			}
			auto const *spot{catenary_spot_of(static_cast<std::size_t>(tool.selected))};
			ImGui::Text("%s", format(STR_C("Card %d execution %d, A = %.2f m, %zu parts"), data->card, data->execution, spot != nullptr ? spot->reach : 0.0, catenary_scene_parts(static_cast<std::size_t>(tool.selected))).c_str());
			if (false == tool.probe_good)
				ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", STR_C("The cantilever doesn't assemble:"));
			for (auto const &warning : tool.probe.warnings)
				ImGui::TextWrapped("- %s", warning.c_str());
			// a drop on a gantry: another length of it
			if (auto const *own{catalog->pole(data->pole_type)}; own != nullptr && false == own->drop.empty())
			{
				ImGui::PushItemWidth(160.0f);
				if (ImGui::BeginCombo(STR_C("Drop of the pole"), data->pole_type.c_str()))
				{
					for (auto const &pole : catalog->poles())
						if (pole.second.drop == own->drop && ImGui::Selectable(format("%s  %.1f m", pole.first.c_str(), pole.second.length).c_str(), pole.first == data->pole_type))
						{
							auto changed{*data};
							changed.pole_type = pole.first;
							tool.undo.push_back({++tool.group, false, static_cast<std::size_t>(tool.selected), *data, std::nullopt, false});
							catenary_scene_update(static_cast<std::size_t>(tool.selected), changed);
							if (auto found{tool.spots.find(static_cast<std::size_t>(tool.selected))}; found != tool.spots.end())
								found->second.data = changed;
							tool.probed = false;
						}
					ImGui::EndCombo();
				}
				ImGui::PopItemWidth();
				data = catenary_scene_get(static_cast<std::size_t>(tool.selected));
				if (data == nullptr)
					return;
			}
			// the wires from this pole to others already standing
			if (ImGui::Button(tool.connect_from >= 0 ? STR_C("Stop hanging the wires") : STR_C("Hang the wires to another pole")))
			{
				tool.connect_from = tool.connect_from >= 0 ? -1 : tool.selected;
				tool.status = tool.connect_from >= 0 ? STR("LMB the next pole: the span from this one is hung to it; then the next one, and so on") : std::string{};
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Joins this pole with the poles clicked after it: it goes on the chain at its end, a new chain of the two, or two chains become one"));
			// copying to the neighbours in the chain
			std::size_t position{0};
			auto const chain{catenary_chain_of(static_cast<std::size_t>(tool.selected), position)};
			if (chain >= 0)
			{
				auto const members{catenary_chain_members(static_cast<std::size_t>(chain))};
				ImGui::TextUnformatted(STR_C("Copy to:"));
				ImGui::SameLine();
				if (position > 0 && ImGui::SmallButton(STR_C("previous")))
					catenary_copy({members[position - 1]});
				ImGui::SameLine();
				if (position + 1 < members.size() && ImGui::SmallButton(STR_C("next")))
					catenary_copy({members[position + 1]});
				ImGui::SameLine();
				if (ImGui::SmallButton(STR_C("both")))
				{
					std::vector<std::size_t> targets;
					if (position > 0)
						targets.push_back(members[position - 1]);
					if (position + 1 < members.size())
						targets.push_back(members[position + 1]);
					catenary_copy(targets);
				}
				ImGui::SameLine();
				if (ImGui::SmallButton(STR_C("the whole chain")))
					catenary_copy(members);
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("The pole, its face from the track, execution, height of the wires and insulators; each pole keeps its stagger"));
			}
			if (ImGui::Button(STR_C("Remove the pole")))
				catenary_delete_selected();
			ImGui::SameLine();
			if (ImGui::Button(STR_C("Done")))
				tool.selected = -1;
		}
	}
}

void catenary_editor::render_catenary_ui()
{
	auto &tool{m_catenary};
	catenary_scan();
	auto const *catalog{catenary_scene_catalog()};
	if (catalog == nullptr || tool.poles.empty())
	{
		ImGui::TextWrapped("%s", STR_C("The cantilever catalogue isn't loaded: data/traction_cantilevers.txt and data/traction_supports.txt (with the poles) are needed"));
		return;
	}
	// the page chosen in the toolbar: poles, gantries, wires
	if (tool.page == 1)
	{
		// gantries
		ImGui::Separator();
		if (false == tool.masts.empty())
		{
			ImGui::PushItemWidth(160.0f);
			auto const combo = [&](char const *Label, int &Chosen) {
				if (ImGui::BeginCombo(Label, tool.masts[std::clamp(Chosen, 0, static_cast<int>(tool.masts.size()) - 1)].c_str()))
				{
					grouped_items(tool.masts, Chosen, [&](std::string const &Name) {
						auto const *mast{catalog->mast(Name)};
						return mast == nullptr ? std::string{} : mast->through ? STR(" (through)") : STR(" (end)");
					});
					ImGui::EndCombo();
				}
			};
			{
				// the second mast follows the first until the user picks it himself
				auto const before{tool.mast_first};
				combo(STR_C("First mast"), tool.mast_first);
				if (tool.mast_first != before && false == tool.mast_second_own)
					tool.mast_second = tool.mast_first;
			}
			ImGui::SameLine();
			{
				auto const before{tool.mast_second};
				combo(STR_C("Second mast"), tool.mast_second);
				if (tool.mast_second != before)
					tool.mast_second_own = true;
			}
			ImGui::PopItemWidth();
			ImGui::TextUnformatted(STR_C("Beam:"));
			ImGui::SameLine();
			if (ImGui::RadioButton(STR_C("lower mount"), false == tool.gantry_high))
				tool.gantry_high = false;
			ImGui::SameLine();
			if (ImGui::RadioButton(STR_C("upper mount"), tool.gantry_high))
				tool.gantry_high = true;
			ImGui::Checkbox(STR_C("Square to the track"), &tool.gantry_square);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("The beam goes square to the track the first mast is put by"));
			ImGui::Checkbox(STR_C("First mast from the track"), &tool.gantry_snap);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("The first mast goes with its face at this distance from the axis of the track under the cursor, on the side of the cursor; the second one as far as the beam spans"));
			if (tool.gantry_snap)
			{
				ImGui::SameLine();
				ImGui::PushItemWidth(120.0f);
				// a gantry selected: the distance of its first mast, changed here it moves the whole gantry square to that track
				auto const *selected{tool.gantry_selected >= 0 && false == tool.gantry_placing ? catenary_gantry_get(static_cast<std::size_t>(tool.gantry_selected)) : nullptr};
				double face{0.0};
				glm::dvec2 away{0.0};
				auto const measured{selected != nullptr && catenary_mast_face(selected->masts[0], glm::dvec3{selected->points[0][0], selected->points[0][1], selected->points[0][2]}, face, away)};
				if (measured)
					tool.gantry_face = static_cast<float>(face);
				if (ImGui::DragFloat(STR_C("Face from the track axis (m)##placing"), &tool.gantry_face, 0.01f, 0.5f, 20.0f, "%.2f") && measured && std::abs(tool.gantry_face - face) > 1e-4)
				{
					catenary_gantry_shift(away * (static_cast<double>(tool.gantry_face) - face));
					tool.status = format(STR_C("Gantry %s: mast %d %.2f m from the track axis"), selected->name.c_str(), 1, tool.gantry_face);
				}
				if (ImGui::IsItemActivated())
					tool.shift_group = 0;
				ImGui::PopItemWidth();
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("From the axis of the track to the face of the first mast. Placing: the first mast goes there. A gantry selected: drag or Ctrl+click and type, the gantry moves square to the track; dragged with the mouse it goes only along the track, at this distance"));
			}
			ImGui::SameLine();
			if (tool.gantry_skin[0] == '\0')
				std::snprintf(tool.gantry_skin, sizeof(tool.gantry_skin), "%s", kGantrySkins[0]);
			ImGui::PushItemWidth(200.0f);
			if (ImGui::BeginCombo(STR_C("Texture"), tool.gantry_skin))
			{
				for (auto const *skin : kGantrySkins)
					if (ImGui::Selectable(skin, std::string{skin} == tool.gantry_skin))
						std::snprintf(tool.gantry_skin, sizeof(tool.gantry_skin), "%s", skin);
				ImGui::EndCombo();
			}
			ImGui::PopItemWidth();
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Replaceable texture of the masts, the beam and the drops of the new gantry (the models have none of their own)"));
			if (ImGui::Button(tool.gantry_placing ? STR_C("Cancel the gantry") : STR_C("Place a gantry")))
			{
				tool.gantry_placing = false == tool.gantry_placing;
				tool.gantry_single = false;
				tool.gantry_first.reset();
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("LMB the first mast, LMB the second: the beam is the one of the catalogue nearest to that distance"));
			if (false == tool.gantry_placing)
			{
				ImGui::SameLine();
				if (ImGui::Button(STR_C("Place a single beam element")))
				{
					tool.gantry_placing = true;
					tool.gantry_single = true;
					tool.gantry_first.reset();
					tool.status = STR("LMB the through mast (the first mast above), LMB where the element points: the element of the catalogue nearest to that length");
				}
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("A single beam element (3017-3025, 1910) on one through mast: on one side fixed to the mast, on the other end to a console"));
			}
			if (auto const *gantry{tool.gantry_selected >= 0 ? catenary_gantry_get(static_cast<std::size_t>(tool.gantry_selected)) : nullptr}; gantry != nullptr)
			{
				ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "%s", format(STR_C("Gantry %s, beam %s: the cantilevers go on drops on it"), gantry->name.c_str(), gantry->beam.c_str()).c_str());
				{
					// the mount of the beam of this span, changed in place (the drops and the cantilevers on them follow)
					auto high{gantry->high};
					ImGui::TextUnformatted(STR_C("Beam of this gantry:"));
					ImGui::SameLine();
					if (ImGui::RadioButton((std::string{STR_C("lower mount")} + "##selected").c_str(), false == high))
						high = false;
					ImGui::SameLine();
					if (ImGui::RadioButton((std::string{STR_C("upper mount")} + "##selected").c_str(), high))
						high = true;
					if (high != gantry->high)
					{
						auto const selected{static_cast<std::size_t>(tool.gantry_selected)};
						// the drops on the beam: where along it they hang, to hang them again on it at its new level
						std::vector<std::pair<std::size_t, catenary::suspension>> hangers;
						catenary_gantry_hangers(selected, hangers);
						glm::dvec3 from, to;
						double bottom{0.0};
						catenary_gantry_beam(selected, from, to, bottom);
						std::vector<std::pair<std::size_t, double>> along;
						for (auto const &[index, data] : hangers)
						{
							auto const *pole{catalog->pole(data.pole_type)};
							if (pole == nullptr || pole->gantry)
								continue; // the cantilevers on the masts follow the beam by themselves
							auto const u{plan_of(to) - plan_of(from)};
							along.emplace_back(index, std::clamp(glm::dot(plan_of(glm::dvec3{data.pole[0], 0.0, data.pole[2]}) - plan_of(from), u) / std::max(glm::dot(u, u), 1e-9), 0.0, 1.0));
						}
						auto changed{*gantry};
						changed.high = high;
						++tool.group;
						catenary_change change;
						change.group = tool.group;
						change.gantry = true;
						change.index = selected;
						change.gantry_before = *gantry;
						tool.undo.push_back(change);
						catenary_gantry_update(selected, changed);
						auto const *beam{catalog->beam(changed.beam)};
						std::vector<std::pair<double, std::string>> drops;
						for (auto const &pole : catalog->poles())
							if (beam != nullptr && pole.second.drop == beam->drop)
								drops.emplace_back(pole.second.length, pole.first);
						std::sort(drops.begin(), drops.end());
						if (catenary_gantry_beam(selected, from, to, bottom) && false == drops.empty())
							for (auto const &[index, t] : along)
							{
								auto const *before{catenary_scene_get(index)};
								if (before == nullptr)
									continue;
								tool.undo.push_back({tool.group, false, index, *before, std::nullopt, false});
								auto data{*before};
								data.pole[1] = from.y + (to.y - from.y) * t;
								// the shortest drop which still reaches down to the lowest mount of the cantilever
								for (auto const &drop : drops)
								{
									data.pole_type = drop.second;
									catenary::cantilever_result probe;
									catenary_scene_probe(data, probe);
									if (std::none_of(probe.warnings.begin(), probe.warnings.end(), [](std::string const &Warning) { return Warning.find("below the lowest mount") != std::string::npos; }))
										break;
								}
								catenary_scene_update(index, data);
								tool.spots.erase(index);
							}
						tool.status = format(high ? STR_C("Gantry %s: the beam on the upper mount") : STR_C("Gantry %s: the beam on the lower mount"), changed.name.c_str());
					}
				}
				// another beam: the first mast stays, the second goes as far as the beam spans
				ImGui::PushItemWidth(120.0f);
				if (ImGui::BeginCombo(STR_C("Beam"), gantry->beam.c_str()))
				{
					for (auto const &beam : catalog->beams())
						if (beam.second.single == gantry->masts[1].empty() && ImGui::Selectable(format("%s  %.1f m", beam.first.c_str(), beam.second.span).c_str(), beam.first == gantry->beam))
							catenary_gantry_reshape(beam.first, 0.0, false);
					ImGui::EndCombo();
				}
				ImGui::PopItemWidth();
				ImGui::SameLine();
				auto const step = [&](int const Direction) {
					// the beam one step (0.1 m) longer or shorter
					auto const *current{catalog->beam(gantry->beam)};
					if (current == nullptr)
						return;
					std::string next;
					auto best{std::numeric_limits<double>::max()};
					for (auto const &beam : catalog->beams())
					{
						if (beam.second.single != current->single)
							continue;
						auto const difference{(beam.second.span - current->span) * Direction};
						if (difference > 0.01 && difference < best)
						{
							best = difference;
							next = beam.first;
						}
					}
					if (false == next.empty())
						catenary_gantry_reshape(next, 0.0, false);
				};
				if (ImGui::SmallButton("-"))
					step(-1);
				ImGui::SameLine();
				if (ImGui::SmallButton("+"))
					step(1);
				ImGui::SameLine();
				if (ImGui::Button((STR("Square to the track") + "##squareselected").c_str()))
					catenary_gantry_reshape({}, 0.0, true);
				ImGui::SameLine();
				if (ImGui::Button(STR_C("Take the masts")))
					catenary_gantry_reshape({}, 0.0, false);
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("The masts chosen above go to this gantry; the first mast stays, the second goes as far as the beam needs"));
				// a gantry of many spans: the next span from a through mast at an end of this one, in line with it
				for (int k = 0; k < 2; ++k)
				{
					auto const *type{catalog->mast(gantry->masts[k])};
					auto const free{spans_at(gantry->points[k], static_cast<std::size_t>(tool.gantry_selected)) == 0};
					if (k == 1)
						ImGui::SameLine();
					ImGui::BeginDisabled(type == nullptr || false == type->through || false == free);
					if (ImGui::Button(format(STR_C("Next span from mast %d"), k + 1).c_str()))
					{
						tool.gantry_placing = true;
						glm::dvec3 const from{gantry->points[k][0], gantry->points[k][1], gantry->points[k][2]};
						glm::dvec3 const other{gantry->points[1 - k][0], gantry->points[1 - k][1], gantry->points[1 - k][2]};
						tool.gantry_first = from;
						// in line: "square to the track" with a track along which right_of() is the way out of this span
						auto const out{glm::normalize(plan_of(from) - plan_of(other))};
						tool.gantry_track = glm::dvec2{out.y, -out.x};
						tool.gantry_square = true;
						std::snprintf(tool.gantry_skin, sizeof(tool.gantry_skin), "%s", gantry->skin.c_str());
						if (auto const found{std::find(tool.masts.begin(), tool.masts.end(), gantry->masts[k])}; found != tool.masts.end())
							tool.mast_first = static_cast<int>(found - tool.masts.begin());
						tool.status = STR("LMB: the next mast of the gantry, as far as the nearest beam spans (the mast type: the second one above)");
					}
					ImGui::EndDisabled();
					if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
						ImGui::SetTooltip("%s", type != nullptr && false == type->through ? STR_C("An end mast: a beam can go on one side of it only; a through mast is needed") : false == free ? STR_C("This mast has beams on both sides already") : STR_C("The beam of the next span on the other side of this through mast, in line with this span"));
					ImGui::SameLine();
					ImGui::BeginDisabled(type == nullptr || false == type->through || false == free || gantry->masts[k].empty());
					if (ImGui::Button(format(STR_C("Single element from mast %d"), k + 1).c_str()))
					{
						tool.gantry_placing = true;
						tool.gantry_single = true;
						glm::dvec3 const from{gantry->points[k][0], gantry->points[k][1], gantry->points[k][2]};
						glm::dvec3 const other{gantry->points[1 - k][0], gantry->points[1 - k][1], gantry->points[1 - k][2]};
						tool.gantry_first = from;
						auto const out{glm::normalize(plan_of(from) - plan_of(other))};
						tool.gantry_track = glm::dvec2{out.y, -out.x};
						tool.gantry_square = true;
						std::snprintf(tool.gantry_skin, sizeof(tool.gantry_skin), "%s", gantry->skin.c_str());
						if (auto const found{std::find(tool.masts.begin(), tool.masts.end(), gantry->masts[k])}; found != tool.masts.end())
							tool.mast_first = static_cast<int>(found - tool.masts.begin());
						tool.status = STR("LMB: as far as the single element should reach, in line with this span");
					}
					ImGui::EndDisabled();
					if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
						ImGui::SetTooltip("%s", STR_C("A single beam element (3017-3025, 1910) on the other side of this through mast, in line with this span"));
				}
				{
					float turn{0.0f};
					ImGui::PushItemWidth(120.0f);
					if (ImGui::DragFloat(STR_C("Turn (degrees)"), &turn, 0.1f, -10.0f, 10.0f, "%.1f") && turn != 0.0f)
						catenary_gantry_reshape({}, turn, false);
					ImGui::PopItemWidth();
					if (ImGui::IsItemHovered())
						ImGui::SetTooltip("%s", STR_C("Drag sideways: turns the gantry around its first mast"));
				}
				gantry = catenary_gantry_get(static_cast<std::size_t>(tool.gantry_selected));
				if (gantry == nullptr)
					return;
				// the heights of the masts: their zero is at the rail heads; the foundation head (variants -1...-4) takes the ground.
				// both together, and each by itself no more than 0.20 m from the other
				auto const heighten = [&](catenary::gantry const &Changed) {
					auto const &last{tool.undo.empty() ? catenary_change{} : tool.undo.back()};
					if (tool.undo.empty() || false == last.gantry || last.index != static_cast<std::size_t>(tool.gantry_selected) || false == last.gantry_before.has_value())
					{
						catenary_change change;
						change.group = ++tool.group;
						change.gantry = true;
						change.index = static_cast<std::size_t>(tool.gantry_selected);
						change.gantry_before = *gantry;
						tool.undo.push_back(change);
					}
					catenary_gantry_update(static_cast<std::size_t>(tool.gantry_selected), Changed);
				};
				{
					auto both{static_cast<float>(gantry->points[0][1])};
					ImGui::PushItemWidth(120.0f);
					if (ImGui::DragFloat(STR_C("Gantry at the height (m)"), &both, 0.01f, -1000.0f, 5000.0f, "%.2f"))
					{
						auto changed{*gantry};
						auto const shift{static_cast<double>(both) - gantry->points[0][1]};
						changed.points[0][1] += shift;
						changed.points[1][1] += shift;
						heighten(changed);
					}
					ImGui::PopItemWidth();
					gantry = catenary_gantry_get(static_cast<std::size_t>(tool.gantry_selected));
					if (gantry == nullptr)
						return;
				}
				for (int mast = 0; mast < 2; ++mast)
				{
					if (gantry->masts[mast].empty())
						continue; // a single element has no second mast
					auto height{static_cast<float>(gantry->points[mast][1])};
					ImGui::PushItemWidth(120.0f);
					ImGui::PushID(mast);
					auto const label{format(STR_C("Mast %d at the height (m)"), mast + 1)};
					if (ImGui::DragFloat(label.c_str(), &height, 0.01f, -1000.0f, 5000.0f, "%.2f"))
					{
						auto changed{*gantry};
						auto const other{gantry->points[1 - mast][1]};
						changed.points[mast][1] = std::clamp(static_cast<double>(height), other - 0.20, other + 0.20);
						heighten(changed);
					}
					if (ImGui::IsItemHovered())
						ImGui::SetTooltip("%s", STR_C("The zero of the mast is at the rail heads (the clearance of the beam is counted from it); the ground is taken by the foundation head of the variant -1...-4. Drag or Ctrl+click to type"));
					ImGui::PopID();
					ImGui::PopItemWidth();
					if (mast == 0)
						ImGui::SameLine();
				}
				gantry = catenary_gantry_get(static_cast<std::size_t>(tool.gantry_selected));
				if (gantry == nullptr)
					return;
				if (ImGui::Button(STR_C("Give it the texture")))
				{
					// the gantry and the drops hanging from its beam
					auto changed{*gantry};
					changed.skin = tool.gantry_skin;
					catenary_change change;
					change.group = ++tool.group;
					change.gantry = true;
					change.index = static_cast<std::size_t>(tool.gantry_selected);
					change.gantry_before = *gantry;
					tool.undo.push_back(change);
					glm::dvec3 from, to;
					double bottom{0.0};
					catenary_gantry_beam(static_cast<std::size_t>(tool.gantry_selected), from, to, bottom);
					for (std::size_t index = 0; index < catenary_scene_size(); ++index)
					{
						auto const *data{catenary_scene_get(index)};
						if (data == nullptr || false == data->has_pole)
							continue;
						auto const *pole{catalog->pole(data->pole_type)};
						auto const point{glm::dvec3{data->pole[0], data->pole[1], data->pole[2]}};
						auto const u{to - from};
						auto const t{std::clamp(glm::dot(point - from, u) / std::max(glm::dot(u, u), 1e-9), 0.0, 1.0)};
						if (pole == nullptr || pole->drop.empty() || glm::distance(point, from + u * t) > 0.3)
							continue;
						tool.undo.push_back({tool.group, false, index, *data, std::nullopt, false});
						auto skinned{*data};
						skinned.skin = tool.gantry_skin;
						catenary_scene_update(index, skinned);
					}
					catenary_gantry_update(static_cast<std::size_t>(tool.gantry_selected), changed);
				}
				ImGui::SameLine();
				if (ImGui::Button(STR_C("Remove the gantry")))
				{
					catenary_change change;
					change.group = ++tool.group;
					change.gantry = true;
					change.index = static_cast<std::size_t>(tool.gantry_selected);
					change.gantry_before = *gantry;
					tool.undo.push_back(change);
					catenary_gantry_remove(static_cast<std::size_t>(tool.gantry_selected));
					tool.gantry_selected = -1;
				}
				ImGui::SameLine();
				if (ImGui::Button(STR_C("Back to the poles")))
					tool.gantry_selected = -1;
				// the next cantilevers: on drops of the beam, or straight on the masts
				ImGui::RadioButton(STR_C("On drops"), &tool.gantry_hang, 0);
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("LMB over a track hangs the cantilever on a drop moved along the beam to the track"));
				ImGui::SameLine();
				ImGui::RadioButton(STR_C("On the masts"), &tool.gantry_hang, 1);
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("LMB by a track puts the cantilever straight on the mast of the gantry on that side; the card from the distance of the mast"));
				// which drop the next cantilevers hang on
				if (auto const *beam{tool.gantry_hang == 0 ? catalog->beam(gantry->beam) : nullptr}; beam != nullptr)
				{
					ImGui::PushItemWidth(160.0f);
					auto const automatic{STR("the shortest which fits")};
					if (ImGui::BeginCombo(STR_C("Drop"), tool.drop_choice.empty() ? automatic.c_str() : tool.drop_choice.c_str()))
					{
						if (ImGui::Selectable(automatic.c_str(), tool.drop_choice.empty()))
							tool.drop_choice.clear();
						for (auto const &pole : catalog->poles())
							if (pole.second.drop == beam->drop && ImGui::Selectable(format("%s  %.1f m", pole.first.c_str(), pole.second.length).c_str(), pole.first == tool.drop_choice))
								tool.drop_choice = pole.first;
						ImGui::EndCombo();
					}
					ImGui::PopItemWidth();
				}
			}
		}
		// the cantilevers going on the drops and on the masts of the selected gantry, and the selected one of them
		if ((tool.gantry_selected >= 0 || tool.selected >= 0) && ImGui::CollapsingHeader(STR_C("Cantilevers"), ImGuiTreeNodeFlags_DefaultOpen))
		{
			render_catenary_cantilever_ui();
		}
	}
	else if (tool.page == 2)
	{
		render_catenary_wires_ui();
	}
	else
	{
		render_catenary_cantilever_ui();
	}
	if (false == tool.undo.empty())
	{
		if (ImGui::Button(STR_C("Take back")))
			catenary_undo();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Takes back the last change of the catenary (also Ctrl+Z while this tool is open)"));
	}
	if (catenary_scene_changed())
		ImGui::TextDisabled("%s", STR_C("The catenary files are written when the scenery is saved"));
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void catenary_editor::draw_catenary_overlay() const
{
	auto const &tool{m_catenary};
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto const draw_pole = [&](catenary::suspension const &Data, ImU32 const Colour, float const Width) {
		auto const foot{pole_of(Data)};
		auto const wire{wire_of(Data)};
		projection.line(drawlist, foot, foot + glm::dvec3{0.0, pole_top(Data), 0.0}, Colour, Width);
		projection.line(drawlist, glm::dvec3{foot.x, wire.y, foot.z}, wire, Colour, Width * 0.6f);
		ImVec2 point;
		if (projection.project(wire, point))
			drawlist->AddCircleFilled(point, Width + 1.5f, Colour);
	};
	// gantries: the masts and the line of the bottom of the beam, where the drops hang from
	auto const *catalog{catenary_scene_catalog()};
	for (std::size_t index = 0; index < catenary_gantry_size(); ++index)
	{
		glm::dvec3 from, to;
		double bottom{0.0};
		auto const *gantry{catenary_gantry_get(index)};
		if (gantry == nullptr || false == catenary_gantry_beam(index, from, to, bottom) || glm::distance(from, camera) > kCatenaryRange)
			continue;
		auto const selected{static_cast<int>(index) == tool.gantry_selected};
		auto const hovered{is_intent(e.m_intent, intent_detail::gantry) && e.m_intent.index == static_cast<int>(index)};
		auto const broken{catenary_gantry_parts(index) == 0};
		auto const colour{selected ? overlay_color::selected : hovered ? overlay_color::highlight : broken ? overlay_color::invalid : IM_COL32(255, 160, 60, 170)};
		projection.line(drawlist, from, to, colour, selected || hovered ? 3.0f : 1.5f);
		for (int mast = 0; mast < 2; ++mast)
		{
			if (gantry->masts[mast].empty())
				continue; // the tip of a single element
			auto const *type{catalog != nullptr ? catalog->mast(gantry->masts[mast]) : nullptr};
			glm::dvec3 const foot{gantry->points[mast][0], gantry->points[mast][1], gantry->points[mast][2]};
			projection.line(drawlist, foot, foot + glm::dvec3{0.0, type != nullptr ? type->height : 8.0, 0.0}, colour, selected || hovered ? 3.0f : 1.5f);
		}
		ImVec2 middle;
		if ((selected || hovered || broken) && projection.project(0.5 * (from + to) + glm::dvec3{0.0, 1.2, 0.0}, middle))
			drawlist->AddText(middle, colour, (gantry->name + "  " + gantry->beam).c_str());
	}
	// the gantry being placed: the first mast, and the second one where the beam would reach
	if (tool.gantry_placing && tool.gantry_first.has_value())
	{
		auto const first{*tool.gantry_first};
		projection.line(drawlist, first, first + glm::dvec3{0.0, 8.0, 0.0}, overlay_color::grip, 3.0f);
		glm::dvec3 second;
		std::string beam;
		if (catenary_gantry_second(catenary_mast_point(), second, beam))
		{
			if (false == tool.gantry_single)
				projection.line(drawlist, second, second + glm::dvec3{0.0, 8.0, 0.0}, overlay_color::marked, 3.0f);
			projection.line(drawlist, first + glm::dvec3{0.0, 7.5, 0.0}, second + glm::dvec3{0.0, 7.5, 0.0}, overlay_color::marked, 2.0f);
			ImVec2 middle;
			if (projection.project(0.5 * (first + second) + glm::dvec3{0.0, 8.2, 0.0}, middle))
				drawlist->AddText(middle, overlay_color::marked, format("%s  %.1f m", beam.c_str(), glm::length(plan_of(second) - plan_of(first))).c_str());
		}
	}
	for (std::size_t index = 0; index < catenary_scene_size(); ++index)
	{
		auto const *data{catenary_scene_get(index)};
		if (data == nullptr || false == data->has_at || glm::distance(pole_of(*data), camera) > kCatenaryRange)
			continue;
		auto const selected{static_cast<int>(index) == tool.selected};
		auto const hovered{is_intent(e.m_intent, intent_detail::select) && e.m_intent.index == static_cast<int>(index)};
		auto const broken{catenary_scene_parts(index) == 0};
		auto const byswitch{catenary_switch_error(index)};
		auto const last{static_cast<int>(index) == tool.last && tool.chaining};
		auto const colour{selected ? overlay_color::selected : hovered ? overlay_color::highlight : broken || byswitch ? overlay_color::invalid : last ? overlay_color::grip : IM_COL32(255, 210, 60, 150)};
		draw_pole(*data, byswitch && false == selected ? overlay_color::invalid : colour, selected || hovered || last || byswitch ? 3.0f : 1.5f);
		ImVec2 head;
		if ((selected || hovered || broken || byswitch) && projection.project(pole_of(*data) + glm::dvec3{0.0, pole_top(*data) + 0.3, 0.0}, head))
		{
			auto label{format("%d-%d", data->card, data->execution) + (broken ? std::string{"  "} + STR("doesn't assemble") : std::string{})};
			if (byswitch)
				label += "  " + format(STR_C("%s by a switch"), data->pole_type.c_str());
			drawlist->AddText(ImVec2(head.x + 6.0f, head.y - 6.0f), colour, label.c_str());
		}
	}
	// spans against the catalogue: green within the limit, red over it; the selected chain brighter
	auto const &network{tool.networks.empty() ? std::string{} : tool.networks[std::clamp(tool.network, 0, static_cast<int>(tool.networks.size()) - 1)]};
	for (auto const &span : spans_around(camera, network, tool.zone))
	{
		auto const a{wire_of(*catenary_scene_get(span.from))};
		auto const b{wire_of(*catenary_scene_get(span.to))};
		auto const over{span.limit > 0.0 && span.length > span.limit + 0.05};
		auto const chosen{span.chain >= 0 && (span.chain == tool.chain_selected || (is_intent(e.m_intent, intent_detail::chain) && e.m_intent.index == span.chain))};
		auto const colour{over ? overlay_color::invalid : chosen ? overlay_color::selected : span.chain >= 0 ? IM_COL32(90, 230, 120, 200) : IM_COL32(90, 230, 120, 110)};
		projection.line(drawlist, a, b, colour, over || chosen ? 2.5f : 1.2f);
		ImVec2 middle;
		if (projection.project(0.5 * (a + b), middle))
		{
			auto const label{span.limit > 0.0 ? format("%.1f / %.0f m", span.length, span.limit) : format("%.1f m", span.length)};
			auto const text{span.radius > 0.0 && span.radius < kCurveRadius ? label + format("  R %.0f", span.radius) : label};
			drawlist->AddText(ImVec2(middle.x + 4.0f, middle.y + 2.0f), colour, text.c_str());
		}
	}
	// what a click puts there, with the span from the last pole
	if (is_intent(e.m_intent, intent_detail::place) && e.m_hover.track != nullptr)
	{
		catenary_spot spot;
		if (catenary_spot_at(*e.m_hover.track, e.m_hover.path, e.m_hover.point, e.cursor_level(e.m_hover.point.y), 0, spot))
		{
			auto const colour{spot.card != 0 && false == spot.switch_error ? overlay_color::marked : overlay_color::invalid};
			draw_pole(spot.data, colour, 3.0f);
			projection.line(drawlist, spot.axis + glm::dvec3{0.0, kRailTop, 0.0}, pole_of(spot.data), colour, 1.5f);
			if (tool.chaining && tool.last >= 0)
				if (auto const *previous{catenary_scene_get(static_cast<std::size_t>(tool.last))}; previous != nullptr)
					projection.line(drawlist, wire_of(*previous), wire_of(spot.data), colour, 1.5f);
		}
	}
}

float const kDragStart{6.0f};

void catenary_editor::catenary_drag_update()
{
	auto &tool{m_catenary};
	if (tool.dragging == 0)
		return;
	if (false == ImGui::GetIO().MouseDown[0])
	{
		catenary_release();
		return;
	}
	auto const &mouse{ImGui::GetIO().MousePos};
	if (glm::length(glm::vec2{mouse.x, mouse.y} - tool.pressed) <= kDragStart)
		return;
	if (tool.dragging == 1)
	{
		// the pole along the track: the place under the cursor, the side and the stagger it has
		auto const *found{tool.selected >= 0 ? catenary_spot_of(static_cast<std::size_t>(tool.selected)) : nullptr};
		auto const *current{tool.selected >= 0 ? catenary_scene_get(static_cast<std::size_t>(tool.selected)) : nullptr};
		if (found == nullptr || current == nullptr || found->track == nullptr || found->track->m_editorremoved)
			return;
		auto *track{e.m_hover.track != nullptr ? e.m_hover.track : found->track};
		auto const path{e.m_hover.track != nullptr ? e.m_hover.path : found->path};
		auto const side{found->side};
		catenary_spot spot;
		{
			without_gantry const plain{tool.gantry_selected};
			if (false == catenary_spot_at(*track, path, e.cursor_level(found->axis.y), e.cursor_level(found->axis.y), side, spot))
				return;
		}
		auto const *catalog{catenary_scene_catalog()};
		auto const *own{catalog != nullptr ? catalog->pole(current->pole_type) : nullptr};
		auto const drop{own != nullptr && false == own->drop.empty()};
		if (drop || (own != nullptr && own->gantry))
			return; // a drop, or a cantilever on a mast, goes with its gantry
		spot.curve = tool.bend == 2 || (tool.bend == 0 && spot.radius > 0.0 && spot.radius < kCurveRadius);
		catenary_compose(spot, tool.clearance, tool.stagger, tool.toward);
		if (spot.card != 0)
			catenary_apply(spot);
		return;
	}
	if (tool.dragging == 2 && false == tool.drag_group.empty() && tool.gantry_selected >= 0)
	{
		auto const now{e.cursor_ground()};
		auto dx{now.x - tool.drag_start.x}, dz{now.z - tool.drag_start.z};
		// held at its distance from the track: the first mast only along the track
		if (glm::dvec3 place; tool.gantry_snap && tool.drag_axis > 0.0 && catenary_along_track(tool.drag_anchor + glm::dvec3{dx, 0.0, dz}, tool.drag_axis, place))
		{
			dx = place.x - tool.drag_anchor.x;
			dz = place.z - tool.drag_anchor.z;
		}
		// the first move of the drag opens one undo group for all the spans and what hangs on them
		if (tool.drag_undo == 0)
		{
			tool.drag_undo = ++tool.group;
			for (auto const &[span, before] : tool.drag_group)
			{
				catenary_change change;
				change.group = tool.drag_undo;
				change.gantry = true;
				change.index = span;
				change.gantry_before = before;
				tool.undo.push_back(change);
			}
			for (auto const &[index, before] : tool.drag_drops)
				tool.undo.push_back({tool.drag_undo, false, index, before, std::nullopt, false});
		}
		for (auto const &[span, before] : tool.drag_group)
		{
			auto moved{before};
			for (auto &point : moved.points)
			{
				point[0] += dx;
				point[2] += dz;
			}
			catenary_gantry_update(span, moved);
		}
		for (auto const &[index, before] : tool.drag_drops)
		{
			if (catenary_scene_get(index) == nullptr)
				continue;
			auto shifted{before};
			for (auto *point : {shifted.at, shifted.rope, shifted.pole, shifted.attach})
			{
				point[0] += dx;
				point[2] += dz;
			}
			catenary_scene_update(index, shifted);
			tool.spots.erase(index);
		}
	}
}

void catenary_editor::catenary_release()
{
	auto &tool{m_catenary};
	tool.dragging = 0;
	tool.drag_group.clear();
	tool.drag_undo = 0;
	tool.drag_drops.clear();
	tool.probed = false;
}

void catenary_editor::catenary_gantry_reshape(std::string const &Beam, double const Turn, bool const Square)
{
	auto &tool{m_catenary};
	auto const *catalog{catenary_scene_catalog()};
	auto const *gantry{tool.gantry_selected >= 0 ? catenary_gantry_get(static_cast<std::size_t>(tool.gantry_selected)) : nullptr};
	if (catalog == nullptr || gantry == nullptr)
		return;
	auto changed{*gantry};
	if (false == Beam.empty())
		changed.beam = Beam;
	if (false == tool.masts.empty() && Beam.empty() && Turn == 0.0 && false == Square)
	{
		// the masts chosen in the panel
		changed.masts[0] = tool.masts[std::clamp(tool.mast_first, 0, static_cast<int>(tool.masts.size()) - 1)];
		if (false == gantry->masts[1].empty())
			changed.masts[1] = tool.masts[std::clamp(tool.mast_second, 0, static_cast<int>(tool.masts.size()) - 1)];
	}
	auto const single{changed.masts[1].empty()};
	auto const *beam{catalog->beam(changed.beam)};
	auto const *a{catalog->mast(changed.masts[0])};
	auto const *b{single ? nullptr : catalog->mast(changed.masts[1])};
	if (beam == nullptr || a == nullptr || (b == nullptr && false == single))
		return;
	glm::dvec3 const first{gantry->points[0][0], gantry->points[0][1], gantry->points[0][2]};
	glm::dvec3 const second{gantry->points[1][0], gantry->points[1][1], gantry->points[1][2]};
	auto direction{plan_of(second) - plan_of(first)};
	direction = glm::length(direction) > 1e-6 ? glm::normalize(direction) : glm::dvec2{1.0, 0.0};
	auto const middle{0.5 * (plan_of(first) + plan_of(second))};
	if (Turn != 0.0)
	{
		auto const angle{glm::radians(Turn)};
		direction = {direction.x * std::cos(angle) - direction.y * std::sin(angle), direction.x * std::sin(angle) + direction.y * std::cos(angle)};
	}
	if (Square)
	{
		// square to the track nearest to the middle of the gantry
		auto best{std::numeric_limits<double>::max()};
		glm::dvec2 along{0.0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
				continue;
			for (int path = 0; path < static_cast<int>(track->m_paths.size()); ++path)
			{
				bezier const curve{track->m_paths[path]};
				glm::dvec3 const probe{middle.x, 0.0, middle.y};
				if (geometry::plan_distance(curve.point(0.5), probe) > curve.plan_length() * 0.5 + std::min(best, 60.0))
					continue;
				auto const parameter{nearest_parameter(*track, path, probe)};
				auto const distance{geometry::plan_distance(curve.point(parameter), probe)};
				if (distance < best)
				{
					best = distance;
					along = forward_at(*track, path, parameter);
				}
			}
		}
		if (glm::length(along) < 0.5)
		{
			tool.status = STR("No track near the gantry");
			return;
		}
		auto const across{right_of(along)};
		direction = glm::dot(across, direction) >= 0.0 ? across : -across;
	}
	// a single element: the tip point at its length from the face of the mast (only its direction matters)
	auto const spacing{single ? beam->span + a->offset : catenary::gantry_spacing(*beam, *a, *b)};
	// turned or squared: around its first mast (the gantry is set out from it on the ground); a new beam or masts: the first mast stays
	glm::dvec2 const start{plan_of(first)};
	auto const end{start + direction * spacing};
	changed.points[0][0] = start.x;
	changed.points[0][2] = start.y;
	changed.points[1][0] = end.x;
	changed.points[1][2] = end.y;
	auto const selected{static_cast<std::size_t>(tool.gantry_selected)};
	// the new places of the spans: this one, and in a gantry of many spans the others joined with it
	std::vector<std::pair<std::size_t, catenary::gantry>> moves;
	auto const group{gantry_group(selected)};
	if (group.size() > 1 && (Turn != 0.0 || Square))
	{
		// turned or squared: the whole gantry around its middle, by the angle this span turns
		auto const before{plan_of(second) - plan_of(first)};
		auto const after{direction};
		auto const angle{std::atan2(before.x * after.y - before.y * after.x, glm::dot(before, after))};
		// around the first mast of the gantry: the first mast of its first span (they're all in one line, at one angle)
		auto const firstspan{*std::min_element(group.begin(), group.end())};
		glm::dvec2 const middle{catenary_gantry_get(firstspan)->points[0][0], catenary_gantry_get(firstspan)->points[0][2]};
		auto const c{std::cos(angle)}, si{std::sin(angle)};
		for (auto const span : group)
		{
			auto turned{*catenary_gantry_get(span)};
			for (auto &point : turned.points)
			{
				glm::dvec2 const p{point[0] - middle.x, point[2] - middle.y};
				point[0] = middle.x + p.x * c - p.y * si;
				point[2] = middle.y + p.x * si + p.y * c;
			}
			moves.emplace_back(span, turned);
		}
	}
	else
	{
		moves.emplace_back(selected, changed);
		// another beam or masts: the spans beyond a mast which moved go with it
		for (int k = 0; k < 2; ++k)
		{
			glm::dvec2 const shift{changed.points[k][0] - gantry->points[k][0], changed.points[k][2] - gantry->points[k][2]};
			if (glm::length(shift) < 1e-4)
				continue;
			// the spans joined at this mast, and everything joined with them except through this span
			for (std::size_t other = 0; other < catenary_gantry_size(); ++other)
			{
				auto const *candidate{catenary_gantry_get(other)};
				if (candidate == nullptr || other == selected || false == (same_mast(candidate->points[0], gantry->points[k]) || same_mast(candidate->points[1], gantry->points[k])))
					continue;
				for (auto const span : gantry_group(other, selected))
				{
					if (std::any_of(moves.begin(), moves.end(), [span](auto const &Move) { return Move.first == span; }))
						continue;
					auto shifted{*catenary_gantry_get(span)};
					for (auto &point : shifted.points)
					{
						point[0] += shift.x;
						point[2] += shift.y;
					}
					moves.emplace_back(span, shifted);
				}
			}
		}
	}
	auto const &last{tool.undo.empty() ? catenary_change{} : tool.undo.back()};
	// turning in small steps is taken back at once
	if (Turn == 0.0 || tool.undo.empty() || false == last.gantry || last.index != selected || false == last.gantry_before.has_value())
	{
		++tool.group;
		// the selected span last: the next small turn finds it at the top of the undo and joins this group
		for (auto move{moves.rbegin()}; move != moves.rend(); ++move)
		{
			catenary_change change;
			change.group = tool.group;
			change.gantry = true;
			change.index = move->first;
			change.gantry_before = *catenary_gantry_get(move->first);
			tool.undo.push_back(change);
		}
	}
	for (auto const &move : moves)
		catenary_gantry_update(move.first, move.second);
	tool.status = format(STR_C("Gantry %s: beam %s, masts %.3f m apart"), changed.name.c_str(), changed.beam.c_str(), spacing);
}

// the hooks of editor_mode (editormode.h), and what they did in the editor before the tool got a file of its own

bool catenary_editor::catenary_intent(track_intent &Intent)
{
	using kind = track_intent::kind;
	if (m_catenary.gantry_placing)
	{
		Intent.position = catenary_mast_point();
		Intent.action = m_catenary.gantry_first.has_value() ? STR("Click: the second mast, as far as the nearest beam spans") : STR("Click: the first mast of the gantry");
		Intent.what = kind::catenary;
		Intent.detail = static_cast<int>(intent_detail::mast);
		return true;
	}
	if (auto const hit{catenary_hit()}; hit >= 0)
	{
		Intent.index = hit;
		Intent.action = m_catenary.connect_from >= 0 ? STR("Click: hang the span to this pole") : STR("Click: select the pole");
		Intent.what = kind::catenary;
		Intent.detail = static_cast<int>(intent_detail::select);
		return true;
	}
	if (auto const hit{catenary_gantry_hit()}; hit >= 0)
	{
		Intent.index = hit;
		Intent.action = hit == m_catenary.gantry_selected ? STR("Drag: move the gantry") : STR("Click: put the cantilevers on drops on this gantry, drag: move it");
		Intent.what = kind::catenary;
		Intent.detail = static_cast<int>(intent_detail::gantry);
		return true;
	}
	if (auto const hit{catenary_span_hit()}; hit >= 0)
	{
		Intent.index = hit;
		Intent.action = STR("Click: select the chain of this span");
		Intent.what = kind::catenary;
		Intent.detail = static_cast<int>(intent_detail::chain);
		return true;
	}
	// the wires page places nothing, the gantries page places only on the selected gantry
	if (e.m_hover.track != nullptr && m_catenary.page != 2 && (m_catenary.page != 1 || m_catenary.gantry_selected >= 0))
	{
		catenary_spot spot;
		if (catenary_spot_at(*e.m_hover.track, e.m_hover.path, e.m_hover.point, e.cursor_level(e.m_hover.point.y), 0, spot))
		{
			Intent.track = e.m_hover.track;
			Intent.path = e.m_hover.path;
			Intent.position = spot.axis;
			Intent.action = catenary_intent_label(spot);
			Intent.what = kind::catenary;
			Intent.detail = static_cast<int>(intent_detail::place);
			return true;
		}
	}
	return false;
}

std::string catenary_editor::catenary_mode_name() const
{
	if (m_catenary.page == 2)
		return m_catenary.connect_from >= 0 ? STR_C("Wires: LMB on the next pole hangs the span to it") : STR_C("Wires: LMB on a span selects its chain, LMB on a pole selects it");
	if (m_catenary.page == 1)
	{
		if (m_catenary.gantry_placing)
			return STR_C("Gantries: LMB the masts");
		return m_catenary.gantry_selected >= 0 ? STR_C("Gantries: LMB by a track puts a cantilever on the selected gantry, drag the beam to move it") : STR_C("Gantries: LMB on a gantry selects it, or place a new one below");
	}
	if (m_catenary.selected >= 0)
		return STR_C("Catenary: the selected pole follows the settings below");
	return STR_C("Catenary: LMB by the track places a pole on that side, LMB on a pole selects it");
}

void catenary_editor::catenary_key_hints(std::vector<key_hint> &Hints) const
{
	Hints = {{"LMB by a track", "places a pole on that side, with the card for its distance"}, {"LMB on a pole", "selects it: the settings then change it at once"}};
	if (m_catenary.selected >= 0)
		Hints.push_back({"Del", "removes the selected pole"});
}

bool catenary_editor::catenary_on_key(int const Key, int const Action, int const Mods)
{
	return false;
}

catenary_editor &editor_mode::catenary_tool()
{
	if (false == static_cast<bool>(m_catenary_editor))
		m_catenary_editor = std::make_shared<catenary_editor>(*this);
	return *m_catenary_editor;
}

void editor_mode::catenary_scan() { catenary_tool().catenary_scan(); }
bool editor_mode::catenary_intent(track_intent &Intent) { return catenary_tool().catenary_intent(Intent); }
void editor_mode::catenary_press(track_intent const &Intent) { catenary_tool().catenary_press(Intent); }
void editor_mode::catenary_release() { catenary_tool().catenary_release(); }
bool editor_mode::catenary_dragging() const { return m_catenary_editor && m_catenary_editor->m_catenary.dragging != 0; }
void editor_mode::catenary_drag_update() { catenary_tool().catenary_drag_update(); }
void editor_mode::draw_catenary_overlay() const
{
	if (m_catenary_editor)
		m_catenary_editor->draw_catenary_overlay();
}
void editor_mode::render_catenary_ui() { catenary_tool().render_catenary_ui(); }
bool editor_mode::catenary_delete_selected() { return catenary_tool().catenary_delete_selected(); }
bool editor_mode::catenary_undo() { return catenary_tool().catenary_undo(); }
int editor_mode::catenary_page() { return catenary_tool().m_catenary.page; }
void editor_mode::catenary_choose_page(int const Page) { catenary_tool().catenary_choose_page(Page); }
bool editor_mode::catenary_on_key(int const Key, int const Action, int const Mods) { return catenary_tool().catenary_on_key(Key, Action, Mods); }
std::string editor_mode::catenary_mode_name() const { return m_catenary_editor ? m_catenary_editor->catenary_mode_name() : STR_C("Catenary: LMB by the track places a pole on that side, LMB on a pole selects it"); }
void editor_mode::catenary_key_hints(std::vector<key_hint> &Hints) const
{
	if (m_catenary_editor)
		m_catenary_editor->catenary_key_hints(Hints);
}
