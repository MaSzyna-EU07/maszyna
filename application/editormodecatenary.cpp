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
double const kInsertWidth{1.5}; // m in the plan from the line of a span: a pole placed there goes into the span
double const kInsertEnds{2.0}; // m from the ends of the span where it doesn't
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
	// on a long path a step of the search is metres: refined around the best one, so that a point square to the track (a pole
	// given by another cantilever on it) gets its own foot on the track and not one a metre off
	auto low{std::max(0.0, parameter - 1.0 / 256.0)}, high{std::min(1.0, parameter + 1.0 / 256.0)};
	for (int k = 0; k < 40; ++k)
	{
		auto const a{low + (high - low) / 3.0}, b{high - (high - low) / 3.0};
		if (geometry::plan_distance(curve.point(a), Point) < geometry::plan_distance(curve.point(b), Point))
			high = b;
		else
			low = a;
	}
	return 0.5 * (low + high);
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

// the suspensions on the pole of this one (the same type at the same point, as the scenery pairs an umbrella): this one too
std::vector<std::size_t> pole_sharers(std::size_t const Index)
{
	std::vector<std::size_t> list;
	auto const *own{catenary_scene_get(Index)};
	if (own == nullptr || false == own->has_pole)
		return list;
	for (std::size_t index = 0; index < catenary_scene_size(); ++index)
	{
		auto const *data{catenary_scene_get(index)};
		if (data != nullptr && data->has_pole && data->pole_type == own->pole_type
		    && glm::length(glm::dvec3{data->pole[0] - own->pole[0], data->pole[1] - own->pole[1], data->pole[2] - own->pole[2]}) < 0.05)
			list.push_back(index);
	}
	return list;
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
	{
		// the centre of the bend is on the side of the chord's middle
		auto const bend{glm::dot(0.5 * (plan_of(before) + plan_of(after)) - plan_of(Spot.axis), right)};
		Spot.inward = std::abs(bend) < 1e-6 ? 0 : bend > 0.0 ? 1 : -1;
	}
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

double catenary_editor::catenary_cant(catenary_spot const &Spot)
{
	if (Spot.track == nullptr || Spot.path < 0 || Spot.path >= static_cast<int>(Spot.track->m_paths.size()))
		return 0.0;
	auto const &rolls{Spot.track->m_paths[Spot.path].rolls};
	return std::abs(glm::radians(glm::mix(static_cast<double>(rolls[0]), static_cast<double>(rolls[1]), std::clamp(Spot.parameter, 0.0, 1.0))));
}

glm::dvec3 catenary_editor::catenary_cant_shift(catenary_spot const &Spot, double const Height)
{
	auto const cant{catenary_cant(Spot)};
	if (cant < 1e-4)
		return glm::dvec3{0.0};
	// towards the centre of the bend; on a cant without a bend (shouldn't be) by the sign of the roll: a positive one lifts the left rail
	auto side{Spot.inward};
	if (side == 0)
	{
		auto const &rolls{Spot.track->m_paths[Spot.path].rolls};
		side = glm::mix(rolls[0], rolls[1], static_cast<float>(Spot.parameter)) > 0.0f ? 1 : -1;
	}
	auto const right{right_of(Spot.forward) * static_cast<double>(side)};
	auto const shift{Height * std::sin(cant)};
	// the rail heads: the lower rail stays at the points of the track, the middle goes up by half the cant (TSegment::Init)
	return glm::dvec3{right.x * shift, 0.75 * std::sin(cant), right.y * shift};
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
	// on a superelevated track the stagger is measured from the leaning axis of the track (table 23-0007)
	auto const wire{glm::dvec3{Spot.axis.x, railhead + height, Spot.axis.z} + catenary_cant_shift(Spot, height) + out * offset};
	Spot.reach = glm::dot(plan_of(polepoint) - plan_of(wire), plan_of(out)) - 0.5 * pole->depth(height);
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
	auto const lean{catenary_cant_shift(spot, wire.y - (spot.axis.y + kRailTop))};
	spot.toward = glm::dot(plan_of(wire) - plan_of(spot.axis + lean), right) >= 0.0;
	spot.data = *data;
	spot.card = data->card;
	auto const decade{(data->card / 10) % 10};
	if (data->card / 100 == 11 || data->card / 100 == 12)
		spot.curve = decade == 3 || decade == 4;
	return &(tool.spots[Index] = spot);
}

std::string catenary_editor::catenary_intent_label(catenary_spot const &Spot) const
{
	auto where{Spot.curve ? format(STR_C("curve R %.0f m"), Spot.radius) : std::string{STR("straight")}};
	if (auto const cant{catenary_cant(Spot)}; cant >= 1e-4)
		where += format(STR_C(", superelevation %.0f mm"), 1500.0 * std::sin(cant));
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
		// the pole, or the cantilever from it to the wires (two cantilevers of an umbrella or spread ones share one pole: the
		// cantilever tells them apart, and wins a tie with the pole)
		auto const wire{wire_of(*data)};
		auto const plane{data->has_spread ? glm::dvec3{data->spread[0], foot.y, data->spread[2]} : foot};
		glm::dvec3 const root{plane.x, wire.y, plane.z};
		auto const arm{mouse_to_segment(projection, root, root + (wire - root) * 0.85)}; // not up to the wires: the span is picked there
		auto const distance{std::min(mouse_to_segment(projection, foot, foot + glm::dvec3{0.0, pole_top(*data), 0.0}), arm * 0.999f)};
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

// the anchor whose device is under the cursor: the line from the face of the pole to the moving pulley, or the column of weights
int catenary_editor::catenary_anchor_hit() const
{
	screen_projection const projection;
	auto best{kPolePick * kPolePick * 0.5f};
	int result{-1};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (std::size_t index = 0; index < catenary_anchor_size(); ++index)
	{
		double shape[4][3];
		if (false == catenary_anchor_shape(index, shape))
			continue;
		glm::dvec3 const face{shape[0][0], shape[0][1], shape[0][2]}, lever{shape[1][0], shape[1][1], shape[1][2]};
		glm::dvec3 const top{shape[2][0], shape[2][1], shape[2][2]}, bottom{shape[3][0], shape[3][1], shape[3][2]};
		if (glm::distance(face, camera) > kCatenaryRange)
			continue;
		auto const distance{std::min(mouse_to_segment(projection, face, lever), mouse_to_segment(projection, top, bottom))};
		if (distance < best)
		{
			best = distance;
			result = static_cast<int>(index);
		}
	}
	return result;
}

void catenary_editor::catenary_select(int const Index)
{
	auto &tool{m_catenary};
	tool.anchor_selected = -1;
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
	data.has_spread = current->has_spread;
	data.shared_pole = current->shared_pole;
	std::copy(std::begin(current->spread), std::end(current->spread), data.spread);
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
		else if (false == drop && pole_sharers(index).size() > 1)
			catenary_keep_pole(spot, *current, std::abs(offset), offset >= 0.0);
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
		data.has_spread = current->has_spread;
		data.shared_pole = current->shared_pole;
		std::copy(std::begin(current->spread), std::end(current->spread), data.spread);
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

bool catenary_editor::catenary_insert_at(catenary_spot const &Spot, chain_insert &Insert) const
{
	auto const &tool{m_catenary};
	if (false == tool.insert_spans || false == Spot.data.has_at || (tool.page == 1 && tool.gantry_selected >= 0))
		return false;
	auto const wire{plan_of(wire_of(Spot.data))};
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto best{kInsertWidth};
	auto found{false};
	for (std::size_t chain = 0; chain < catenary_chain_size(); ++chain)
	{
		if (catenary_chain_get(chain) == nullptr)
			continue;
		auto const members{catenary_chain_members(chain)};
		for (std::size_t k = 0; k + 1 < members.size(); ++k)
		{
			auto const *a{catenary_scene_get(members[k])};
			auto const *b{catenary_scene_get(members[k + 1])};
			if (a == nullptr || b == nullptr || false == a->has_at || false == b->has_at || glm::distance(wire_of(*a), camera) > kCatenaryRange)
				continue;
			auto const pa{plan_of(wire_of(*a))}, pb{plan_of(wire_of(*b))};
			auto const span{pb - pa};
			auto const length{glm::length(span)};
			if (length < 2.0 * kInsertEnds)
				continue;
			auto const along{glm::dot(wire - pa, span / length)};
			// not at the ends: a pole there is the pole of the end, or the next one past it
			if (along < kInsertEnds || along > length - kInsertEnds)
				continue;
			auto const aside{std::abs(glm::dot(wire - pa, glm::dvec2{-span.y, span.x} / length))};
			if (aside < best)
			{
				best = aside;
				Insert.chain = static_cast<int>(chain);
				Insert.after = k;
				Insert.from = members[k];
				Insert.to = members[k + 1];
				found = true;
			}
		}
	}
	return found;
}

void catenary_editor::catenary_chain_end()
{
	auto &tool{m_catenary};
	tool.chain = -1;
	tool.last = -1;
	tool.connect_from = -1;
	tool.anchor_pick = -1;
	tool.status = STR("The chain is ended: the next pole starts a new one");
}

bool catenary_editor::catenary_chain_continue(std::size_t const Pole)
{
	auto &tool{m_catenary};
	auto const *data{catenary_scene_get(Pole)};
	if (data == nullptr || false == data->has_at)
		return false;
	std::size_t position{0};
	auto const chain{catenary_chain_of(Pole, position)};
	if (chain >= 0)
	{
		auto const members{catenary_chain_members(static_cast<std::size_t>(chain))};
		auto const atend{position + 1 == members.size()};
		if (position != 0 && false == atend)
		{
			tool.status = STR("The pole is in the middle of a chain: the chain goes on from its ends only");
			return false;
		}
		tool.chain = chain;
		tool.append = atend;
		tool.chain_selected = chain;
	}
	else
		tool.chain = -1; // a new chain from this pole
	// the next poles go to the file of this one: a chain is in one file
	std::snprintf(tool.file, sizeof(tool.file), "%s", catenary_scene_file_of(Pole).c_str());
	tool.last = static_cast<int>(Pole);
	tool.chaining = true;
	tool.connect_from = -1;
	tool.anchor_pick = -1;
	tool.anchor_selected = -1;
	tool.selected = -1;
	tool.probed = false;
	tool.page = 0;
	if (bool toward{tool.toward}; catenary_propose_toward(toward))
		tool.toward = toward;
	tool.status = format(STR_C("The next poles go on from %s: LMB by the track; Esc ends the chain"), data->name.empty() ? "-" : data->name.c_str());
	return true;
}

void catenary_editor::catenary_place(catenary_spot const &Spot)
{
	auto &tool{m_catenary};
	++tool.group;
	auto data{Spot.data};
	// in a span of a chain: the pole goes into the chain between the two, in the file of the chain
	chain_insert insert;
	auto const inserting{catenary_insert_at(Spot, insert)};
	std::string const file{inserting ? catenary_chain_file(static_cast<std::size_t>(insert.chain)) : std::string{tool.file}};
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
	if (inserting)
	{
		// the chain going on (tool.last) stays as it was
		if (auto const *chain{catenary_chain_get(static_cast<std::size_t>(insert.chain))}; chain != nullptr && insert.after + 1 < chain->suspensions.size())
		{
			auto changed{*chain};
			tool.undo.push_back({tool.group, true, static_cast<std::size_t>(insert.chain), std::nullopt, *chain, false});
			changed.suspensions.insert(changed.suspensions.begin() + static_cast<std::ptrdiff_t>(insert.after + 1), data.name);
			auto const name{changed.name};
			catenary_chain_update(static_cast<std::size_t>(insert.chain), changed);
			tool.chain_selected = insert.chain;
			auto const *a{catenary_scene_get(insert.from)};
			auto const *b{catenary_scene_get(insert.to)};
			tool.status += format(STR_C("; put into chain %s between %s and %s"), name.c_str(), a != nullptr ? a->name.c_str() : "-", b != nullptr ? b->name.c_str() : "-");
		}
		return;
	}
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

bool catenary_editor::catenary_umbrella_spot(TTrack &Track, int const Path, catenary_spot &Spot)
{
	auto &tool{m_catenary};
	auto const *source{tool.umbrella_from >= 0 ? catenary_scene_get(static_cast<std::size_t>(tool.umbrella_from)) : nullptr};
	auto const *catalog{catenary_scene_catalog()};
	auto const *pole{source != nullptr && catalog != nullptr ? catalog->pole(source->pole_type) : nullptr};
	if (pole == nullptr || false == source->has_pole || false == source->has_at)
		return false;
	auto const foot{pole_of(*source)};
	{
		// square to the pole on this track, on the side of the pole; not onto a drop of the selected gantry
		without_gantry const plain{tool.gantry_selected};
		if (false == catenary_spot_at(Track, Path, foot, foot, 0, Spot))
			return false;
	}
	// the cantilever on the same pole (or drop): its distance is the one of the pole, the panel gives the rest
	auto const right{right_of(Spot.forward) * static_cast<double>(Spot.side)};
	auto const clearance{glm::dot(plan_of(foot) - plan_of(Spot.axis), right) - 0.5 * pole->depth(0.0)};
	auto const stagger{Spot.curve ? std::max(static_cast<double>(tool.stagger), catenary_scene_layout().curve_stagger()) : static_cast<double>(tool.stagger)};
	catenary_compose(Spot, clearance, stagger, Spot.toward, source->pole_type);
	std::copy(std::begin(source->pole), std::end(source->pole), Spot.data.pole);
	if (Spot.card == 0)
		return true;
	// the second cantilever goes the other way from the pole than the first one
	auto const mine{plan_of(wire_of(Spot.data)) - plan_of(foot)};
	auto const other{plan_of(wire_of(*source)) - plan_of(foot)};
	if (glm::dot(mine, other) > 0.0)
	{
		Spot.card = 0;
		Spot.problem = STR("This track is on the same side of the pole as its cantilever: the second cantilever of an umbrella goes to a track on the other side");
	}
	return true;
}

void catenary_editor::catenary_umbrella_place(catenary_spot const &Spot)
{
	auto &tool{m_catenary};
	auto const from{static_cast<std::size_t>(tool.umbrella_from)};
	auto const *source{tool.umbrella_from >= 0 ? catenary_scene_get(from) : nullptr};
	tool.umbrella_from = -1;
	if (source == nullptr)
		return;
	++tool.group;
	auto const file{catenary_scene_file_of(from)};
	auto sourcedata{*source};
	tool.undo.push_back({tool.group, false, from, sourcedata, std::nullopt, false});
	if (sourcedata.name.empty())
		sourcedata.name = catenary_scene_new_name(file); // a pole from before the editor had no name
	// a name of the umbrella no other suspension has
	std::string name;
	for (int k = 1; name.empty(); ++k)
	{
		auto const candidate{"parasol_" + sourcedata.name + (k > 1 ? "_" + std::to_string(k) : std::string{})};
		auto used{false};
		for (std::size_t index = 0; index < catenary_scene_size() && false == used; ++index)
			if (auto const *data{catenary_scene_get(index)}; data != nullptr && data->umbrella == candidate)
				used = true;
		if (false == used)
			name = candidate;
	}
	// the first one named first, so that the second gets a name of its own
	sourcedata.umbrella = name;
	catenary_scene_update(from, sourcedata);
	if (auto found{tool.spots.find(from)}; found != tool.spots.end())
		found->second.data = sourcedata;
	auto data{Spot.data};
	data.name = catenary_scene_new_name(file);
	data.skin = sourcedata.skin;
	data.umbrella = name;
	auto const index{catenary_scene_add(file, data)};
	tool.spots[index] = Spot;
	tool.spots[index].data = data;
	tool.undo.push_back({tool.group, false, index, std::nullopt, std::nullopt, false});
	catenary_select(static_cast<int>(index));
	tool.status = format(STR_C("Umbrella %s: the second cantilever (card %d execution %d, A = %.2f m) on pole %s of %s; hang its wires with the button below or C"), name.c_str(), Spot.card, data.execution, Spot.reach, data.pole_type.c_str(), sourcedata.name.c_str());
	if (catenary_scene_parts(index) == 0)
		tool.status += STR(" - the cantilever didn't assemble, see the log");
}

void catenary_editor::catenary_keep_pole(catenary_spot &Spot, catenary::suspension const &Current, double const Stagger, bool const Toward) const
{
	auto const *catalog{catenary_scene_catalog()};
	auto const *own{catalog != nullptr ? catalog->pole(Current.pole_type) : nullptr};
	if (own == nullptr)
	{
		Spot.card = 0;
		Spot.problem = STR("No pole: the catalogue (data/traction_cantilevers.txt, data/traction_supports.txt) isn't loaded or has no poles");
		return;
	}
	// the plane of the cantilever: through the pole, or through its mount on the console of spread cantilevers
	auto const plane{Current.has_spread ? glm::dvec3{Current.spread[0], Current.pole[1], Current.spread[2]} : pole_of(Current)};
	auto const right{right_of(Spot.forward) * static_cast<double>(Spot.side)};
	catenary_compose(Spot, glm::dot(plan_of(plane) - plan_of(Spot.axis), right) - 0.5 * own->depth(0.0), Stagger, Toward, Current.pole_type);
	std::copy(std::begin(Current.pole), std::end(Current.pole), Spot.data.pole);
	Spot.data.has_spread = Current.has_spread;
	std::copy(std::begin(Current.spread), std::end(Current.spread), Spot.data.spread);
}

void catenary_editor::catenary_spread_create(std::size_t const Index)
{
	auto &tool{m_catenary};
	auto const *current{catenary_scene_get(Index)};
	auto const *found{catenary_spot_of(Index)};
	auto const *catalog{catenary_scene_catalog()};
	auto const *pole{current != nullptr && catalog != nullptr ? catalog->pole(current->pole_type) : nullptr};
	// a cantilever without strut and stay (spread steady arms 0167) goes on the beam 4690, the others on the console 4890
	auto const *scheme{current != nullptr && catalog != nullptr ? catalog->scheme(current->card, current->execution) : nullptr};
	auto const steadyonly{scheme != nullptr && scheme->roles.count("strut") == 0 && scheme->roles.count("stay") == 0};
	auto const consolename{pole == nullptr ? std::string{} : steadyonly && false == pole->steady_console.empty() ? pole->steady_console : pole->console};
	auto const *console{false == consolename.empty() ? catalog->mount(consolename) : nullptr};
	if (current == nullptr || found == nullptr || false == current->has_pole || false == current->umbrella.empty())
		return;
	if (console == nullptr || false == console->console)
	{
		tool.status = format(STR_C("Pole %s carries no console for spread cantilevers (\"console\" in its entry in traction_supports.txt)"), current->pole_type.c_str());
		return;
	}
	auto const sourcedata{*current};
	auto const spot{*found};
	auto const foot{pole_of(sourcedata)};
	// the axis of the console mounts (the X axis of the cantilevers): along the track, square to the way from the track to the pole
	auto const out{right_of(spot.forward) * static_cast<double>(spot.side)};
	glm::dvec3 const xaxis{-out.y, 0.0, out.x};
	auto const right{right_of(spot.forward) * static_cast<double>(spot.side)};
	auto const offset{glm::dot(plan_of(wire_of(sourcedata)) - plan_of(spot.axis), right)};
	catenary_spot spots[2];
	for (int k = 0; k < 2; ++k)
	{
		auto const point{foot + xaxis * console->console_x[k]};
		{
			without_gantry const plain{tool.gantry_selected};
			if (false == catenary_spot_at(*spot.track, spot.path, point, foot, spot.side, spots[k]))
				return;
		}
		spots[k].curve = spot.curve;
		catenary::suspension planned{sourcedata};
		planned.has_spread = true;
		planned.spread[0] = point.x;
		planned.spread[1] = foot.y;
		planned.spread[2] = point.z;
		catenary_keep_pole(spots[k], planned, std::abs(offset), offset >= 0.0);
		if (spots[k].card == 0)
		{
			tool.status = spots[k].problem;
			return;
		}
	}
	++tool.group;
	auto const file{catenary_scene_file_of(Index)};
	tool.undo.push_back({tool.group, false, Index, sourcedata, std::nullopt, false});
	auto first{spots[0].data};
	first.name = sourcedata.name.empty() ? catenary_scene_new_name(file) : sourcedata.name;
	first.range_max = sourcedata.range_max;
	first.range_min = sourcedata.range_min;
	first.skin = sourcedata.skin;
	first.overrides = sourcedata.overrides;
	std::string name;
	for (int k = 1; name.empty(); ++k)
	{
		auto const candidate{"rozstaw_" + first.name + (k > 1 ? "_" + std::to_string(k) : std::string{})};
		auto used{false};
		for (std::size_t index = 0; index < catenary_scene_size() && false == used; ++index)
			if (auto const *data{catenary_scene_get(index)}; data != nullptr && data->umbrella == candidate)
				used = true;
		if (false == used)
			name = candidate;
	}
	first.umbrella = name;
	catenary_scene_update(Index, first);
	tool.spots[Index] = spots[0];
	tool.spots[Index].data = first;
	auto second{spots[1].data};
	second.name = catenary_scene_new_name(file);
	second.skin = sourcedata.skin;
	second.umbrella = name;
	auto const added{catenary_scene_add(file, second)};
	tool.spots[added] = spots[1];
	tool.spots[added].data = second;
	tool.undo.push_back({tool.group, false, added, std::nullopt, std::nullopt, false});
	catenary_select(static_cast<int>(added));
	tool.status = format(STR_C("Spread cantilevers %s on console %s: %s and %s, %.2f m apart; give each its chain, \"Messenger held by\" splits the messenger and the wires between them"),
	                     name.c_str(), console->name.c_str(), first.name.c_str(), second.name.c_str(), console->console_x[0] - console->console_x[1]);
	if (catenary_scene_parts(added) == 0 || catenary_scene_parts(Index) == 0)
		tool.status += STR(" - the cantilever didn't assemble, see the log");
}

void catenary_editor::catenary_third_create(std::size_t const Index)
{
	auto &tool{m_catenary};
	auto const *current{catenary_scene_get(Index)};
	auto const *found{catenary_spot_of(Index)};
	if (current == nullptr || found == nullptr || found->track == nullptr || false == current->has_pole)
		return;
	auto const sourcedata{*current};
	auto const source{*found};
	auto const foot{pole_of(sourcedata)};
	// in the axis of the pole, on its own mounts, to the same track; the pole is stood by the spread pair
	catenary_spot spot;
	{
		without_gantry const plain{tool.gantry_selected};
		if (false == catenary_spot_at(*source.track, source.path, foot, foot, source.side, spot))
			return;
	}
	spot.curve = source.curve;
	catenary::suspension planned{sourcedata};
	planned.has_spread = false;
	planned.umbrella.clear();
	planned.shared_pole = true;
	catenary_keep_pole(spot, planned, tool.stagger, tool.toward);
	if (spot.card == 0)
	{
		tool.status = spot.problem;
		return;
	}
	auto data{spot.data};
	auto const file{catenary_scene_file_of(Index)};
	data.name = catenary_scene_new_name(file);
	data.skin = sourcedata.skin;
	data.shared_pole = true;
	++tool.group;
	auto const added{catenary_scene_add(file, data)};
	tool.spots[added] = spot;
	tool.spots[added].data = data;
	tool.undo.push_back({tool.group, false, added, std::nullopt, std::nullopt, false});
	catenary_select(static_cast<int>(added));
	tool.status = format(STR_C("Cantilever %s in the axis of pole %s (card %d execution %d): set its card (a messenger only one: 1002-1015, 1290-1297), hang it in a chain, an umbrella to the other track can go on it too"),
	                     data.name.c_str(), data.pole_type.c_str(), data.card, data.execution);
	if (catenary_scene_parts(added) == 0)
		tool.status += STR(" - the cantilever didn't assemble, see the log");
}

void catenary_editor::catenary_pair_move(std::size_t const Selected, glm::dvec3 const &Foot, bool const Panel)
{
	auto &tool{m_catenary};
	auto const *current{catenary_scene_get(Selected)};
	if (current == nullptr || false == current->has_pole)
		return;
	auto const oldfoot{pole_of(*current)};
	std::vector<std::pair<std::size_t, catenary_spot>> moved;
	for (auto const index : pole_sharers(Selected))
	{
		auto const *data{catenary_scene_get(index)};
		if (data == nullptr)
			continue;
		auto const *found{catenary_spot_of(index)};
		if (found == nullptr || found->track == nullptr)
			return;
		auto const found_copy{*found};
		// each keeps its place against the pole (its console mount) and its stagger, on its own track
		auto const plane{data->has_spread ? glm::dvec3{data->spread[0], data->pole[1], data->spread[2]} : pole_of(*data)};
		auto planenew{Foot + (plane - oldfoot)};
		planenew.y = Foot.y;
		auto const right{right_of(found_copy.forward) * static_cast<double>(found_copy.side)};
		auto offset{glm::dot(plan_of(wire_of(*data)) - plan_of(found_copy.axis), right)};
		// from the panel: the selected one takes its stagger from there
		if (Panel && index == Selected)
			offset = tool.toward ? static_cast<double>(tool.stagger) : -static_cast<double>(tool.stagger);
		catenary_spot spot;
		{
			without_gantry const plain{tool.gantry_selected};
			if (false == catenary_spot_at(*found_copy.track, found_copy.path, planenew, Foot, found_copy.side, spot))
				return;
		}
		spot.curve = found_copy.curve;
		catenary::suspension planned{*data};
		planned.pole[0] = Foot.x;
		planned.pole[1] = Foot.y;
		planned.pole[2] = Foot.z;
		planned.spread[0] = planenew.x;
		planned.spread[1] = Foot.y;
		planned.spread[2] = planenew.z;
		catenary_keep_pole(spot, planned, std::abs(offset), offset >= 0.0);
		if (spot.card == 0)
		{
			tool.status = spot.problem;
			return;
		}
		auto &result{spot.data};
		result.name = data->name;
		result.range_max = data->range_max;
		result.range_min = data->range_min;
		result.skin = data->skin;
		result.overrides = data->overrides;
		result.umbrella = data->umbrella;
		result.shared_pole = data->shared_pole;
		moved.emplace_back(index, spot);
	}
	// one undo group for the whole drag (of the pole, or of the slider of its distance)
	auto record{false};
	int group{0};
	if (Panel)
	{
		record = tool.shift_group == 0 || tool.undo.empty() || tool.undo.back().group != tool.shift_group;
		if (record)
			tool.shift_group = ++tool.group;
		group = tool.shift_group;
	}
	else
	{
		record = tool.drag_undo == 0;
		if (record)
			tool.drag_undo = ++tool.group;
		group = tool.drag_undo;
	}
	for (auto const &[index, spot] : moved)
	{
		if (record)
			tool.undo.push_back({group, false, index, *catenary_scene_get(index), std::nullopt, false});
		catenary_scene_update(index, spot.data);
		tool.spots[index] = spot;
	}
	tool.probed = false;
}

void catenary_editor::catenary_umbrella_break(std::size_t const Index)
{
	auto &tool{m_catenary};
	auto const *current{catenary_scene_get(Index)};
	if (current == nullptr || current->umbrella.empty())
		return;
	auto const name{current->umbrella};
	++tool.group;
	// another one on the pole stands it already (the third one by spread cantilevers stands none)
	auto placed{false};
	for (auto const other : pole_sharers(Index))
		if (auto const *data{catenary_scene_get(other)}; data != nullptr && data->umbrella != name && false == data->shared_pole)
			placed = true;
	for (std::size_t index = 0; index < catenary_scene_size(); ++index)
	{
		auto const *data{catenary_scene_get(index)};
		if (data == nullptr || data->umbrella != name)
			continue;
		auto changed{*data};
		tool.undo.push_back({tool.group, false, index, changed, std::nullopt, false});
		changed.umbrella.clear();
		changed.has_spread = false; // a cantilever off its console pair stands by the pole as one of its own
		changed.shared_pole = changed.shared_pole || placed; // the pole stands once
		placed = placed || false == changed.shared_pole;
		catenary_scene_update(index, changed);
		if (auto found{tool.spots.find(index)}; found != tool.spots.end())
			found->second.data = changed;
	}
	tool.probed = false;
	tool.status = format(STR_C("Umbrella %s separated: two cantilevers on their own mounts"), name.c_str());
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
		if (tool.anchor_pick >= 0)
		{
			// the tensioning device on this pole
			if (catenary_anchor_create(static_cast<std::size_t>(tool.anchor_pick), tool.anchor_pick_start, Intent.index, tool.anchor_pick_contact))
				tool.anchor_pick = -1;
			return;
		}
		tool.umbrella_from = -1;
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
		tool.anchor_selected = -1;
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
		tool.anchor_selected = -1;
		catenary_chain_show(Intent.index);
		return;
	}
	if (is_intent(Intent, intent_detail::anchor))
	{
		// the anchor: its chain on the wires page, the anchor marked in the panel
		bool start{false};
		auto const chain{catenary_anchor_chain(static_cast<std::size_t>(Intent.index), start)};
		catenary_chain_show(chain);
		tool.anchor_selected = Intent.index;
		tool.anchor_scroll = true;
		if (auto const *anchor{catenary_anchor_get(static_cast<std::size_t>(Intent.index))}; anchor != nullptr)
			tool.status = chain >= 0 ? format(STR_C("Anchor %s selected: its settings are below, Del removes it"), anchor->name.c_str())
			                         : format(STR_C("Anchor %s isn't at the end of any chain"), anchor->name.c_str());
		return;
	}
	if (Intent.track == nullptr)
		return;
	catenary_spot spot;
	if (tool.umbrella_from >= 0)
	{
		// the second cantilever of an umbrella on the pole of the first one
		if (false == catenary_umbrella_spot(*Intent.track, Intent.path, spot))
			return;
		if (spot.card == 0)
			tool.status = spot.problem;
		else
			catenary_umbrella_place(spot);
		return;
	}
	if (false == catenary_spot_at(*Intent.track, Intent.path, e.m_hover.point, e.cursor_level(e.m_hover.point.y), 0, spot))
		return;
	if (spot.card == 0)
	{
		tool.status = spot.problem;
		return;
	}
	catenary_place(spot);
}

void catenary_editor::catenary_chain_show(int const Chain)
{
	auto &tool{m_catenary};
	tool.selected = -1;
	tool.page = 2;
	tool.chain_selected = Chain;
	if (auto const *chain{Chain >= 0 ? catenary_chain_get(static_cast<std::size_t>(Chain)) : nullptr}; chain != nullptr)
	{
		// the panel shows what the chain has
		std::snprintf(tool.power, sizeof(tool.power), "%s", chain->power.c_str());
		tool.voltage = static_cast<float>(chain->voltage);
		tool.material = chain->material == "al" ? 1 : 0;
		tool.state = chain->state == 2 ? 1 : chain->state == 1 ? 2 : 0;
		if (auto const found{std::find(tool.networks.begin(), tool.networks.end(), chain->type)}; found != tool.networks.end())
			tool.network = static_cast<int>(found - tool.networks.begin());
	}
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
	tool.anchor_pick = -1;
	if (tool.selected < 0 && tool.anchor_selected >= 0)
	{
		// the anchor picked in the view
		auto const picked{static_cast<std::size_t>(tool.anchor_selected)};
		tool.anchor_selected = -1;
		auto const *anchor{catenary_anchor_get(picked)};
		if (anchor == nullptr)
			return false;
		bool start{false};
		if (auto const chain{catenary_anchor_chain(picked, start)}; chain >= 0)
		{
			auto const *owner{catenary_chain_get(static_cast<std::size_t>(chain))};
			auto const contact{owner != nullptr && (start ? owner->anchor_start_contact : owner->anchor_end_contact) == anchor->name};
			catenary_anchor_delete(static_cast<std::size_t>(chain), start, contact);
		}
		else
		{
			catenary_change removed;
			removed.group = ++tool.group;
			removed.anchor = true;
			removed.index = picked;
			removed.anchor_before = *anchor;
			tool.undo.push_back(removed);
			catenary_anchor_remove(picked);
		}
		tool.status = STR("Anchor removed");
		return true;
	}
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
		    && (std::find(data->suspensions.begin(), data->suspensions.end(), current->name) != data->suspensions.end()
		        || std::any_of(data->messenger_from.begin(), data->messenger_from.end(), [&](auto const &Pair) { return Pair.first == current->name || Pair.second == current->name; })))
			tool.undo.push_back({tool.group, true, chain, std::nullopt, *data, false});
	}
	// the other cantilever of its umbrella stays alone
	if (false == current->umbrella.empty())
		for (std::size_t other = 0; other < catenary_scene_size(); ++other)
		{
			auto const *data{catenary_scene_get(other)};
			if (other == index || data == nullptr || data->umbrella != current->umbrella)
				continue;
			auto changed{*data};
			tool.undo.push_back({tool.group, false, other, changed, std::nullopt, false});
			changed.umbrella.clear(); // a spread one stays alone on its console
			catenary_scene_update(other, changed);
			if (auto found{tool.spots.find(other)}; found != tool.spots.end())
				found->second.data = changed;
		}
	current = catenary_scene_get(index);
	// it stood the pole and others stay on it: one of them stands it now (not one of a pair, which the pair stands itself)
	if (false == current->shared_pole)
	{
		std::vector<std::size_t> rest;
		for (auto const other : pole_sharers(index))
			if (other != index)
				rest.push_back(other);
		auto const standing{std::any_of(rest.begin(), rest.end(), [](std::size_t const Other) { auto const *data{catenary_scene_get(Other)}; return data != nullptr && false == data->shared_pole; })};
		if (false == rest.empty() && false == standing)
		{
			auto changed{*catenary_scene_get(rest.front())};
			tool.undo.push_back({tool.group, false, rest.front(), changed, std::nullopt, false});
			changed.shared_pole = false;
			catenary_scene_update(rest.front(), changed);
			if (auto found{tool.spots.find(rest.front())}; found != tool.spots.end())
				found->second.data = changed;
		}
		current = catenary_scene_get(index);
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
	tool.anchor_pick = -1;
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

std::size_t catenary_editor::catenary_anchor_of(catenary::chain const &Chain, std::string const &File, bool const Start, bool const Contact) const
{
	auto const &name{Contact ? (Start ? Chain.anchor_start_contact : Chain.anchor_end_contact) : (Start ? Chain.anchor_start : Chain.anchor_end)};
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

bool catenary_editor::catenary_anchor_create(std::size_t const Chain, bool const Start, int const On, bool const Contact)
{
	auto &tool{m_catenary};
	auto const *chain{catenary_chain_get(Chain)};
	if (chain == nullptr)
		return false;
	auto const file{catenary_chain_file(Chain)};
	auto const members{catenary_chain_members(Chain)};
	if (members.size() < 2)
		return false;
	auto const *last{catenary_scene_get(Start ? members.front() : members.back())};
	auto const *previous{catenary_scene_get(Start ? members[1] : members[members.size() - 2])};
	if (last == nullptr || previous == nullptr || false == last->has_at || false == previous->has_at)
		return false;
	catenary::anchor data;
	data.name = catenary_scene_new_anchor_name(file);
	data.network = chain->type;
	if (On >= 0)
	{
		// on an existing pole: it has to stand (a pole entry) and be in the same file as the chain
		auto const *carrier{catenary_scene_get(static_cast<std::size_t>(On))};
		if (carrier == nullptr || false == carrier->has_pole || carrier->name.empty() || catenary_scene_file_of(static_cast<std::size_t>(On)) != file)
		{
			tool.status = STR("This pole can't carry the anchor: it has to be a pole (not a drop or a point) from the file of the chain");
			return false;
		}
		if (static_cast<std::size_t>(On) == (Start ? members.front() : members.back()))
		{
			tool.status = STR("The anchor goes on another pole than the last one of the chain");
			return false;
		}
		data.on = carrier->name;
		data.pole_type = carrier->pole_type;
		std::copy(std::begin(carrier->pole), std::end(carrier->pole), data.pole);
	}
	else
	{
		// the anchor pole 24 m past the last pole, in line with it (catalogue 22 m; Glowacki 24 m for big tensions)
		glm::dvec2 along{last->at[0] - previous->at[0], last->at[2] - previous->at[2]};
		if (glm::length(along) < 0.1)
			return false;
		along = glm::normalize(along);
		auto const base{last->has_pole ? glm::dvec2{last->pole[0], last->pole[2]} : glm::dvec2{last->at[0], last->at[2]}};
		auto const at{base + along * 24.0};
		data.pole_type = tool.poles.empty() ? std::string{} : tool.poles[std::clamp(tool.pole, 0, static_cast<int>(tool.poles.size()) - 1)];
		data.pole[0] = at.x;
		data.pole[1] = last->has_pole ? last->pole[1] : last->at[1] - static_cast<double>(tool.height);
		data.pole[2] = at.y;
	}
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
	++tool.group;
	if (Contact)
	{
		// the contact wires by themselves (0404, or the wires of a split one); the anchor of the messenger keeps the messenger only
		auto const main{catenary_anchor_of(*chain, file, Start)};
		auto const *messenger{main < catenary_anchor_size() ? catenary_anchor_get(main) : nullptr};
		if (messenger == nullptr)
			return false;
		auto const *devices{catenary_scene_tensioners()};
		auto const *device{devices != nullptr ? devices->device(messenger->device) : nullptr};
		auto const fixed{device == nullptr || device->fixed};
		data.device = fixed ? "0404" : "0482-c";
		if (device == nullptr || device->only != "messenger")
		{
			auto changedmessenger{*messenger};
			changedmessenger.device = fixed ? "0405" : "0482-m";
			catenary_change before;
			before.group = tool.group;
			before.anchor = true;
			before.index = main;
			before.anchor_before = *messenger;
			tool.undo.push_back(before);
			catenary_anchor_update(main, changedmessenger);
		}
		(Start ? changed.anchor_start_contact : changed.anchor_end_contact) = data.name;
	}
	else
		(Start ? changed.anchor_start : changed.anchor_end) = data.name;
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
	tool.anchor_pick_contact = false;
	tool.status = Contact ? STR("The contact wires are anchored on their own; the other anchor holds the messenger only") : STR("Anchored: the last span goes without droppers to the tensioning device");
	return true;
}

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
	// middle anchor (cards 63-0406 / 0408): at one of the inner poles of the chain, the anchor rope goes to the poles on both sides
	{
		auto changed{*chain};
		std::vector<std::string> labels{STR("none")};
		int chosen{0};
		for (std::size_t k = 1; k + 1 < chain->suspensions.size(); ++k)
		{
			labels.push_back(format(STR_C("pole %d (%s)"), static_cast<int>(k + 1), chain->suspensions[k].c_str()));
			if (chain->suspensions[k] == chain->middle)
				chosen = static_cast<int>(labels.size()) - 1;
		}
		ImGui::PushItemWidth(180.0f);
		auto edited{false};
		if (ImGui::BeginCombo(STR_C("Middle anchor"), labels[chosen].c_str()))
		{
			for (std::size_t k = 0; k < labels.size(); ++k)
				if (ImGui::Selectable(labels[k].c_str(), static_cast<int>(k) == chosen))
				{
					changed.middle = k == 0 ? std::string{} : chain->suspensions[k];
					changed.middle_heights[0] = changed.middle_heights[1] = 0.0;
					edited = true;
				}
			ImGui::EndCombo();
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The anchor rope is clamped on the messenger 1 m on each side of this pole and goes to the poles next to it; in the middle of both spans a V ties it to the contact wire. The weight anchors measure their distance from it"));
		if (false == chain->middle.empty())
		{
			char const *const cards[] = {"0406", "0408", "0418", "0416"};
			int insulated{0};
			for (int k = 0; k < 4; ++k)
				if (chain->middle_device == cards[k])
					insulated = k;
			std::string const kinds[] = {STR("with insulation (card 0406)"), STR("without insulation (card 0408)"), STR("insulated on one side (card 0418)"), STR("to the beam of a gantry (card 0416)")};
			char const *const items[] = {kinds[0].c_str(), kinds[1].c_str(), kinds[2].c_str(), kinds[3].c_str()};
			if (ImGui::Combo(STR_C("Middle anchor kind"), &insulated, items, 4))
			{
				changed.middle_device = cards[std::clamp(insulated, 0, 3)];
				edited = true;
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("0416: the middle pole is a drop of a gantry; the anchor ropes go from the beam above the messenger to clamps on it about 3 m on both sides, the poles next to it carry nothing"));
			// heights of the anchor rope on the poles before and after (when the cantilever is in the way)
			auto const where{std::find(chain->suspensions.begin(), chain->suspensions.end(), chain->middle) - chain->suspensions.begin()};
			auto const members{catenary_chain_members(Chain)};
			for (int side = 0; side < 2 && chain->middle_device != "0416"; ++side)
			{
				auto const k{static_cast<std::size_t>(where) + (side == 0 ? -1 : 1)};
				auto const *neighbour{k < members.size() ? catenary_scene_get(members[k]) : nullptr};
				if (neighbour == nullptr || false == neighbour->has_pole)
					continue;
				auto const automatic{(neighbour->has_rope ? neighbour->rope[1] : neighbour->at[1] + 1.70) - neighbour->pole[1]};
				auto height{static_cast<float>(chain->middle_heights[side] > 0.0 ? chain->middle_heights[side] : automatic)};
				ImGui::PushID(side);
				if (ImGui::SliderFloat(side == 0 ? STR_C("Anchor rope on the pole before (m)") : STR_C("Anchor rope on the pole after (m)"), &height, 4.0f, 10.0f, "%.2f"))
				{
					changed.middle_heights[side] = height;
					edited = true;
				}
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("Height of the anchor of the middle anchor rope on that pole above the rail heads; by default at the messenger. Move it when it hits the cantilever"));
				ImGui::PopID();
			}
			if ((chain->middle_heights[0] > 0.0 || chain->middle_heights[1] > 0.0) && ImGui::Button(STR_C("Anchor rope at the messenger")))
			{
				changed.middle_heights[0] = changed.middle_heights[1] = 0.0;
				edited = true;
			}
		}
		ImGui::PopItemWidth();
		if (edited)
		{
			// one undo step for one drag of a slider
			auto const &last{tool.undo.empty() ? catenary_change{} : tool.undo.back()};
			if (tool.undo.empty() || false == last.chain || last.index != Chain || false == ImGui::IsMouseDragging(0))
				tool.undo.push_back({++tool.group, true, Chain, std::nullopt, *chain, false});
			catenary_chain_update(Chain, changed);
			chain = catenary_chain_get(Chain);
			if (chain == nullptr)
				return;
		}
	}
	ImGui::TextUnformatted(STR_C("Anchors"));
	// each end: the anchor of the network (or of the messenger only), then the one of the contact wires when they're anchored apart
	for (int slot = 0; slot < 4; ++slot)
	{
		auto const end{slot / 2};
		auto const contact{slot % 2 == 1};
		auto const start{end == 0};
		auto const separate{false == (start ? chain->anchor_start_contact : chain->anchor_end_contact).empty()};
		ImGui::PushID(slot);
		// the last two poles at this end: the wires come along them to the anchor
		auto const *last{catenary_scene_get(start ? members.front() : members.back())};
		auto const *previous{catenary_scene_get(start ? members[1] : members[members.size() - 2])};
		auto const index{catenary_anchor_of(*chain, file, start, contact)};
		auto const *anchor{index < catenary_anchor_size() ? catenary_anchor_get(index) : nullptr};
		if (contact && anchor == nullptr)
		{
			// the contact wires apart from the messenger (0405 + 0404, or the two halves of a split one): on another pole
			if (catenary_anchor_of(*chain, file, start) < catenary_anchor_size())
			{
				auto const picking{tool.anchor_pick == static_cast<int>(Chain) && tool.anchor_pick_start == start && tool.anchor_pick_contact};
				if (ImGui::SmallButton(picking ? STR_C("Stop picking the pole") : STR_C("Contact wires anchored apart: on another pole")))
				{
					tool.anchor_pick = picking ? -1 : static_cast<int>(Chain);
					tool.anchor_pick_start = start;
					tool.anchor_pick_contact = false == picking;
					tool.connect_from = -1;
					tool.status = picking ? std::string{} : STR("LMB on the pole the anchor of the contact wires goes on");
				}
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("The messenger stays on the anchor above (it becomes one of the messenger only: 0405 or the messenger of a split one), the contact wires go to an anchor of their own on the pole picked (0404 or the wires of a split one)"));
			}
			ImGui::PopID();
			continue;
		}
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
				catenary_anchor_create(Chain, start, -1);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("An anchor pole 24 m past the last pole, in line with it, with the weight tensioning device; the last span hangs without droppers"));
			ImGui::SameLine();
			auto const picking{tool.anchor_pick == static_cast<int>(Chain) && tool.anchor_pick_start == start};
			if (ImGui::Button(picking ? STR_C("Stop picking the pole") : STR_C("to an existing pole")))
			{
				tool.anchor_pick = picking ? -1 : static_cast<int>(Chain);
				tool.anchor_pick_start = start;
				tool.anchor_pick_contact = false;
				tool.connect_from = -1;
				tool.status = picking ? std::string{} : STR("LMB on the pole the tensioning device goes on");
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("The tensioning device goes on a pole already standing (with its cantilever); the pole stays as it is"));
			ImGui::PopID();
			continue;
		}
		auto const picked{tool.anchor_selected == static_cast<int>(index)};
		ImGui::TextColored(picked ? ImVec4(1.0f, 0.85f, 0.2f, 1.0f) : ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "%s",
		                   (format(start ? STR_C("Start: anchor %s") : STR_C("End: anchor %s"), anchor->name.c_str()) + (contact ? " - " + STR("contact wires") : separate ? " - " + STR("messenger") : std::string{})
		                    + (picked ? "  " + STR("(selected in the view)") : std::string{})).c_str());
		if (picked && tool.anchor_scroll)
		{
			// picked in the view just now: the panel goes to its settings
			ImGui::SetScrollHereY(0.2f);
			tool.anchor_scroll = false;
		}
		auto data{*anchor};
		auto edited{false};
		ImGui::PushItemWidth(180.0f);
		// kind of the anchor: the card of the device (weight tensioning 0461, fixed 0403); variants for network types go by themselves
		auto fixed{false};
		{
			auto const devices{catenary_anchor_devices(contact ? "contact" : separate ? "messenger" : "")};
			auto card{data.device.substr(0, data.device.find('-'))};
			for (auto const &item : devices)
				if (item.card == data.device)
					card = data.device; // a variant listed by itself (0461-2x2)
			std::vector<std::string> labels;
			int chosen{-1};
			for (std::size_t k = 0; k < devices.size(); ++k)
			{
				if (false == devices[k].title.empty())
					labels.push_back(devices[k].title + " (" + devices[k].card + ")");
				else
				labels.push_back(format(devices[k].fixed ? STR_C("fixed (card %s)") : devices[k].split ? STR_C("weights, messenger and contact wires separately (card %s)") : devices[k].series ? STR_C("weights, pulleys in series (card %s)") : STR_C("weight tensioning (card %s)"), devices[k].card.c_str()));
				if (devices[k].card == card)
				{
					chosen = static_cast<int>(k);
					fixed = devices[k].fixed;
				}
			}
			if (ImGui::BeginCombo(STR_C("Kind"), chosen >= 0 ? labels[chosen].c_str() : data.device.c_str()))
			{
				for (std::size_t k = 0; k < devices.size(); ++k)
					if (ImGui::Selectable(labels[k].c_str(), static_cast<int>(k) == chosen))
					{
						data.device = devices[k].card;
						data.height = 0.0;
						data.rope_height = 0.0;
						fixed = devices[k].fixed;
						edited = true;
					}
				ImGui::EndCombo();
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Weight tensioning keeps the tension with the air temperature; a fixed anchor holds the wires straight on the pole (the other end of the half-section has the weights)"));
		}
		if (false == data.on.empty())
			ImGui::TextDisabled("%s", format(STR_C("On the pole of %s"), data.on.c_str()).c_str());
		else
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
		if (data.on.empty() && ImGui::BeginCombo(STR_C("Anchor pole"), data.pole_type.c_str()))
		{
			auto chosen{static_cast<int>(std::find(tool.poles.begin(), tool.poles.end(), data.pole_type) - tool.poles.begin())};
			if (grouped_items(tool.poles, chosen) && chosen >= 0 && chosen < static_cast<int>(tool.poles.size()))
			{
				data.pole_type = tool.poles[chosen];
				edited = true;
			}
			ImGui::EndCombo();
		}
		if (false == fixed && false == chain->middle.empty())
			ImGui::TextDisabled("%s", STR_C("Distance from the middle anchor: along the chain"));
		else if (false == fixed)
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
		if (false == fixed)
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
		if (false == fixed)
		{
			// the insulators of the chains to the wires: resin 7150 or the old roller ESN-2025
			static char const *const values[] = {"", "new", "old", "messenger", "contact"};
			std::string const names[] = {STR("by the wires (roller ones when tarnished)"), STR("resin 7150"), STR("roller ESN-2025 on both"), STR("roller on the messenger only"), STR("roller on the contact wires only")};
			char const *const items[] = {names[0].c_str(), names[1].c_str(), names[2].c_str(), names[3].c_str(), names[4].c_str()};
			int chosen{0};
			for (int i = 0; i < 5; ++i)
				if (data.insulators == values[i])
					chosen = i;
			if (ImGui::Combo(STR_C("Insulators"), &chosen, items, 5))
			{
				data.insulators = values[chosen];
				edited = true;
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Old assemblies have the roller insulator ESN-2025 straight in the lever hole, then the turnbuckle and the clamp; by default the roller ones come with tarnished wires, as the 5110 clamps do"));
		}
		{
			if (false == fixed)
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
			auto const *solved{catenary_anchor_result(index)};
			auto height{static_cast<float>(data.height > 0.0 ? data.height : (solved != nullptr && solved->height > 0.0 ? solved->height : 6.70))};
			if (ImGui::SliderFloat(fixed ? STR_C("Contact wire anchor above the rail heads (m)") : STR_C("Pulleys above the rail heads (m)"), &height, 4.5f, 9.0f, "%.2f"))
			{
				data.height = height;
				edited = true;
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", fixed ? STR_C("Height of the anchor of the contact wires on the mast (the messenger one goes as much higher as at the last pole); by default at the height of the wires at the last pole. Drag or Ctrl+click to type")
				                              : STR_C("Height of the pulley axles on the mast; the weights move up or down with them. The mounting table gives 6.70 m. Drag or Ctrl+click to type"));
			if (fixed)
			{
				auto rope{static_cast<float>(data.rope_height > 0.0 ? data.rope_height : (solved != nullptr && solved->rope_height > 0.0 ? solved->rope_height : 7.40))};
				if (ImGui::SliderFloat(STR_C("Messenger anchor above the rail heads (m)"), &rope, 5.0f, 10.0f, "%.2f"))
				{
					data.rope_height = rope;
					edited = true;
				}
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("Height of the anchor of the messenger on the mast; by default as at the last pole. Drag or Ctrl+click to type"));
				if ((data.height > 0.0 || data.rope_height > 0.0) && ImGui::Button(STR_C("Heights as at the last pole")))
				{
					data.height = 0.0;
					data.rope_height = 0.0;
					edited = true;
				}
			}
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
			if (result->fixed)
				ImGui::TextDisabled("%s", format(STR_C("Fixed anchor: messenger %.2f m, contact wires %.2f m over the rails"), result->rope_height, result->height).c_str());
			else
				ImGui::TextDisabled("%s", format(STR_C("%.1f deg C: pulley %.2f m from the pole, weights %.2f m over the rails; %d weights, messenger %.0f daN, contact wires %.0f daN"),
				                                   Global.AirTemperature, result->b, result->s, result->weights, result->messenger_tension, result->contact_tension).c_str());
			for (auto const &warning : result->warnings)
				ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%s", warning.c_str());
		}
		if (ImGui::Button(STR_C("Remove the anchor")))
		{
			catenary_anchor_delete(Chain, start, contact);
			if (tool.anchor_selected == static_cast<int>(index))
				tool.anchor_selected = -1;
		}
		ImGui::PopID();
	}
}

void catenary_editor::catenary_anchor_delete(std::size_t const Chain, bool const Start, bool const Contact)
{
	auto &tool{m_catenary};
	auto const *chain{catenary_chain_get(Chain)};
	if (chain == nullptr)
		return;
	auto const file{catenary_chain_file(Chain)};
	auto const index{catenary_anchor_of(*chain, file, Start, Contact)};
	auto const *anchor{index < catenary_anchor_size() ? catenary_anchor_get(index) : nullptr};
	if (anchor == nullptr)
		return;
	auto changed{*chain};
	++tool.group;
	if (Contact)
	{
		changed.anchor_start_contact = Start ? std::string{} : changed.anchor_start_contact;
		changed.anchor_end_contact = Start ? changed.anchor_end_contact : std::string{};
		// the anchor of the messenger takes the contact wires again: the whole network on it
		auto const main{catenary_anchor_of(*chain, file, Start)};
		if (auto const *messenger{main < catenary_anchor_size() ? catenary_anchor_get(main) : nullptr}; messenger != nullptr)
		{
			auto const *devices{catenary_scene_tensioners()};
			auto const *device{devices != nullptr ? devices->device(messenger->device) : nullptr};
			if (device != nullptr && device->only == "messenger")
			{
				auto whole{*messenger};
				whole.device = device->fixed ? "0403" : "0461";
				catenary_change before;
				before.group = tool.group;
				before.anchor = true;
				before.index = main;
				before.anchor_before = *messenger;
				tool.undo.push_back(before);
				catenary_anchor_update(main, whole);
			}
		}
	}
	else
	{
		(Start ? changed.anchor_start : changed.anchor_end).clear();
		// the contact wires anchored apart go with it
		auto const contact{catenary_anchor_of(*chain, file, Start, true)};
		if (auto const *wires{contact < catenary_anchor_size() ? catenary_anchor_get(contact) : nullptr}; wires != nullptr)
		{
			catenary_change gone;
			gone.group = tool.group;
			gone.anchor = true;
			gone.index = contact;
			gone.anchor_before = *wires;
			tool.undo.push_back(gone);
			catenary_anchor_remove(contact);
			(Start ? changed.anchor_start_contact : changed.anchor_end_contact).clear();
		}
	}
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

void catenary_editor::render_catenary_chain_state()
{
	auto &tool{m_catenary};
	auto const colour{ImVec4(0.3f, 0.85f, 1.0f, 1.0f)};
	ImGui::Separator();
	// the selected pole: the chain can go on from it
	if (tool.selected >= 0)
	{
		auto const pole{static_cast<std::size_t>(tool.selected)};
		std::size_t position{0};
		auto const chain{catenary_chain_of(pole, position)};
		auto const *data{catenary_chain_get(chain >= 0 ? static_cast<std::size_t>(chain) : catenary_chain_size())};
		auto const middle{data != nullptr && position != 0 && position + 1 != data->suspensions.size()};
		if (middle)
		{
			ImGui::TextDisabled("%s", format(STR_C("In the middle of chain %s (pole %zu of %zu)"), data->name.c_str(), position + 1, data->suspensions.size()).c_str());
			return;
		}
		if (ImGui::Button(data != nullptr ? STR_C("Go on with the chain from this pole (C)") : STR_C("Start a chain from this pole (C)")))
			tool.continue_pending = tool.selected;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The poles placed next by the track are joined to this one with spans, one after another; Esc ends the chain"));
		if (data != nullptr)
		{
			ImGui::SameLine();
			ImGui::TextDisabled("%s", format(position == 0 ? STR_C("start of chain %s") : STR_C("end of chain %s"), data->name.c_str()).c_str());
		}
		return;
	}
	ImGui::Checkbox(STR_C("Chain: a span from the last pole"), &tool.chaining);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Each pole placed is joined with a span to the one placed before it, one chain from anchor to anchor"));
	ImGui::SameLine();
	ImGui::Checkbox(STR_C("Into spans"), &tool.insert_spans);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("A pole placed under the span of a chain (its wire point within 1.5 m of the span, not at its ends) goes into that chain between the two poles"));
	if (false == tool.chaining)
		return;
	auto const *last{tool.last >= 0 ? catenary_scene_get(static_cast<std::size_t>(tool.last)) : nullptr};
	if (last == nullptr)
	{
		ImGui::TextDisabled("%s", STR_C("The next pole starts a new chain; LMB on a pole and C goes on from it"));
		return;
	}
	auto const *chain{tool.chain >= 0 ? catenary_chain_get(static_cast<std::size_t>(tool.chain)) : nullptr};
	auto const lastname{last->name.empty() ? std::string{"-"} : last->name};
	if (chain != nullptr)
		ImGui::TextColored(colour, "%s", format(tool.append ? STR_C("Next pole: a span from %s, at the end of chain %s (%zu poles)") : STR_C("Next pole: a span from %s, at the start of chain %s (%zu poles)"),
		                                        lastname.c_str(), chain->name.c_str(), chain->suspensions.size()).c_str());
	else
		ImGui::TextColored(colour, "%s", format(STR_C("Next pole: a new chain from %s"), lastname.c_str()).c_str());
	if (ImGui::Button(STR_C("End the chain (Esc)")))
		catenary_chain_end();
	if (chain != nullptr && chain->suspensions.size() >= 2)
	{
		auto const index{static_cast<std::size_t>(tool.chain)};
		auto const start{false == tool.append};
		if (catenary_anchor_of(*chain, catenary_chain_file(index), start) >= catenary_anchor_size())
		{
			ImGui::SameLine();
			if (ImGui::Button(STR_C("Anchor here and end")))
			{
				if (catenary_anchor_create(index, start, -1))
				{
					catenary_chain_end();
					tool.chain_selected = static_cast<int>(index);
					tool.status = STR("Anchored and ended: the anchor pole is 24 m past the last pole; its settings are on the Wires page");
				}
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("An anchor pole 24 m past the last pole, in line with it, with the weight tensioning device; the chain is ended"));
		}
	}
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
		render_catenary_chain_state();
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
	tool.anchor_pick = -1;
	tool.anchor_selected = -1;
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
		render_catenary_chain_state();
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
			else if (false == drop && own != nullptr && pole_sharers(static_cast<std::size_t>(tool.selected)).size() > 1)
			{
				// an umbrella or spread cantilevers: the pole stays, unless its face from the track was changed - then it goes
				// across the track and the other cantilever goes with it
				auto const plane{current->has_spread ? glm::dvec3{current->spread[0], current->pole[1], current->spread[2]} : pole_of(*current)};
				auto const right{right_of(spot.forward) * static_cast<double>(spot.side)};
				auto const clearance{glm::dot(plan_of(plane) - plan_of(spot.axis), right) - 0.5 * own->depth(0.0)};
				if (std::abs(clearance - static_cast<double>(tool.clearance)) > 1e-4)
				{
					catenary_compose(spot, tool.clearance, tool.stagger, tool.toward);
					if (spot.card != 0)
					{
						auto foot{glm::dvec3{spot.data.pole[0], spot.data.pole[1], spot.data.pole[2]} - (plane - pole_of(*current))};
						foot.y = current->pole[1];
						catenary_pair_move(static_cast<std::size_t>(tool.selected), foot, true);
						spot.card = -1; // done
					}
				}
				else
					catenary_keep_pole(spot, *current, tool.stagger, tool.toward);
			}
			else
				catenary_compose(spot, tool.clearance, tool.stagger, tool.toward, drop ? current->pole_type : std::string{});
			if (drop)
				spot.data.pole[1] = current->pole[1];
			if (spot.card > 0)
				catenary_apply(spot);
			else if (spot.card == 0)
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
			// an umbrella: the second cantilever on this pole (or drop), to the track on its other side
			if (data->umbrella.empty())
			{
				if (ImGui::Button(tool.umbrella_from >= 0 ? STR_C("Stop the umbrella") : STR_C("Umbrella: a cantilever to the other track")))
				{
					tool.umbrella_from = tool.umbrella_from >= 0 ? -1 : tool.selected;
					tool.connect_from = -1;
					tool.anchor_pick = -1;
					tool.status = tool.umbrella_from >= 0 ? STR("LMB by the track on the other side of the pole: the second cantilever goes on this pole, with the settings of the panel") : std::string{};
				}
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("Two cantilevers on one pole or drop, to the tracks on both sides of it; on adjustable mounts (band round the pole) they share the double mounts 4640-4670 at one height"));
				auto const *ownscheme{catalog->scheme(data->card, data->execution)};
				auto const steadyonly{ownscheme != nullptr && ownscheme->roles.count("strut") == 0 && ownscheme->roles.count("stay") == 0};
				if (auto const *own{catalog->pole(data->pole_type)}; own != nullptr && (steadyonly ? false == own->steady_console.empty() : false == own->console.empty()))
				{
					ImGui::SameLine();
					if (ImGui::Button(STR_C("Spread cantilevers")))
					{
						catenary_spread_create(static_cast<std::size_t>(tool.selected));
						data = tool.selected >= 0 ? catenary_scene_get(static_cast<std::size_t>(tool.selected)) : nullptr;
						if (data == nullptr)
							return;
					}
					if (ImGui::IsItemHovered())
						ImGui::SetTooltip("%s", STR_C("Two cantilevers to this track on consoles 4890 (cards 0517/0518), 1.19 m apart along it: this one goes onto the first mount, the second onto the other; each can then hold any chain or only its messenger or its wires"));
				}
			}
			else
			{
				std::string partner;
				for (std::size_t index = 0; index < catenary_scene_size(); ++index)
					if (auto const *other{catenary_scene_get(index)}; other != nullptr && static_cast<int>(index) != tool.selected && other->umbrella == data->umbrella)
						partner = other->name;
				ImGui::TextUnformatted(format(data->has_spread ? STR_C("Spread cantilevers %s with %s") : STR_C("Umbrella %s with %s"), data->umbrella.c_str(), partner.empty() ? "-" : partner.c_str()).c_str());
				if (data->has_spread)
				{
					// a third one on the same pole, in its axis on its own mounts (messenger only by the spread ones holding the wires)
					if (ImGui::SmallButton(STR_C("A cantilever in the axis of the pole")))
					{
						catenary_third_create(static_cast<std::size_t>(tool.selected));
						data = tool.selected >= 0 ? catenary_scene_get(static_cast<std::size_t>(tool.selected)) : nullptr;
						if (data == nullptr)
							return;
					}
					if (ImGui::IsItemHovered())
						ImGui::SetTooltip("%s", STR_C("A third cantilever to this track on the same pole, in its axis on the usual mounts (move them apart from the consoles by hand if they meet); the pole stays one"));
					ImGui::SameLine();
				}
				ImGui::SameLine();
				if (ImGui::SmallButton(STR_C("Separate")))
				{
					catenary_umbrella_break(static_cast<std::size_t>(tool.selected));
					data = catenary_scene_get(static_cast<std::size_t>(tool.selected));
					if (data == nullptr)
						return;
				}
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("The two cantilevers stay where they are, each on its own single mounts"));
			}
			render_catenary_chain_state();
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
				render_catenary_messenger_from(static_cast<std::size_t>(chain));
			}
			if (ImGui::Button(STR_C("Remove the pole")))
				catenary_delete_selected();
			ImGui::SameLine();
			if (ImGui::Button(STR_C("Done")))
				tool.selected = -1;
		}
	}
}

// the messenger of the chain at the selected suspension held by another cantilever on the same pole or next to it (spread
// cantilevers on the console 4890, a cantilever of the messenger only by a pull-off of the wires): chain entry messenger_from
void catenary_editor::render_catenary_messenger_from(std::size_t const Chain)
{
	auto &tool{m_catenary};
	auto const *chain{catenary_chain_get(Chain)};
	auto const *data{tool.selected >= 0 ? catenary_scene_get(static_cast<std::size_t>(tool.selected)) : nullptr};
	if (chain == nullptr || data == nullptr || data->name.empty())
		return;
	auto const file{catenary_chain_file(Chain)};
	auto const holder{catenary::messenger_holder(*chain, data->name)};
	// a cantilever of the messenger only in the chain between two others of its pole (spread steady arms 0167 with a messenger
	// cantilever in the axis of the pole): it leaves the list, the wires go straight between the two, the messenger through it
	{
		auto const &list{chain->suspensions};
		auto const found{std::find(list.begin(), list.end(), data->name)};
		auto const position{static_cast<std::size_t>(found - list.begin())};
		if (found != list.end() && position > 0 && position + 1 < list.size() && chain->middle != data->name)
		{
			auto const sharers{pole_sharers(static_cast<std::size_t>(tool.selected))};
			auto const onpole = [&](std::string const &Name) {
				return std::any_of(sharers.begin(), sharers.end(), [&](std::size_t const Index) {
					auto const *other{catenary_scene_get(Index)};
					return other != nullptr && other->name == Name;
				});
			};
			if (onpole(list[position - 1]) && onpole(list[position + 1]))
			{
				if (ImGui::Button(STR_C("Messenger only: the wires straight past it")))
				{
					auto changed{*chain};
					tool.undo.push_back({++tool.group, true, Chain, std::nullopt, *chain, false});
					auto const before{list[position - 1]}, after{list[position + 1]};
					auto &pairs{changed.messenger_from};
					pairs.erase(std::remove_if(pairs.begin(), pairs.end(), [&](auto const &Pair) { return Pair.first == before || Pair.first == after || Pair.first == data->name; }), pairs.end());
					pairs.emplace_back(before, data->name);
					pairs.emplace_back(after, data->name);
					changed.suspensions.erase(changed.suspensions.begin() + static_cast<std::ptrdiff_t>(position));
					tool.status = format(STR_C("The wires of the chain go straight from %s to %s, the messenger through %s"), before.c_str(), after.c_str(), data->name.c_str());
					catenary_chain_update(Chain, changed);
					return;
				}
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("This cantilever holds only the messenger: it leaves the chain, the contact wires go straight between the two cantilevers of the pole next to it in the chain, the messenger goes through this one and their cords hang from it"));
			}
		}
	}
	// the cantilevers up to 3 m from this pole (in the plan), in the file of the chain
	constexpr double kHolderRange{3.0};
	std::vector<std::string> names{data->name};
	for (std::size_t index = 0; index < catenary_scene_size(); ++index)
	{
		auto const *other{catenary_scene_get(index)};
		if (other == nullptr || static_cast<int>(index) == tool.selected || other->name.empty() || false == other->has_rope || catenary_scene_file_of(index) != file)
			continue;
		if (glm::length(plan_of(pole_of(*other)) - plan_of(pole_of(*data))) <= kHolderRange)
			names.push_back(other->name);
	}
	if (names.size() < 2 && holder == data->name)
		return;
	ImGui::PushItemWidth(160.0f);
	auto const label{holder == data->name ? format(STR_C("%s (this one)"), holder.c_str()) : holder};
	if (ImGui::BeginCombo(STR_C("Messenger held by"), label.c_str()))
	{
		for (auto const &name : names)
		{
			auto const text{name == data->name ? format(STR_C("%s (this one)"), name.c_str()) : name};
			if (ImGui::Selectable(text.c_str(), name == holder) && name != holder)
			{
				auto changed{*chain};
				tool.undo.push_back({++tool.group, true, Chain, std::nullopt, *chain, false});
				auto &pairs{changed.messenger_from};
				pairs.erase(std::remove_if(pairs.begin(), pairs.end(), [&](auto const &Pair) { return Pair.first == data->name; }), pairs.end());
				if (name != data->name)
					pairs.emplace_back(data->name, name);
				tool.status = name == data->name ? STR("The messenger goes through this cantilever again")
				                                 : format(STR_C("The wires of the chain go through %s, its messenger through %s"), data->name.c_str(), name.c_str());
				catenary_chain_update(Chain, changed);
				break;
			}
		}
		ImGui::EndCombo();
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Another cantilever up to 3 m from this pole holds the messenger of this chain here, this one only its wires: spread cantilevers on a console, a cantilever of the messenger only, chains crossing over in a tension span"));
	ImGui::PopItemWidth();
}

void catenary_editor::render_catenary_ui()
{
	auto &tool{m_catenary};
	catenary_scan();
	if (tool.continue_pending >= 0)
	{
		catenary_chain_continue(static_cast<std::size_t>(tool.continue_pending));
		tool.continue_pending = -1;
	}
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
	// anchors: the device from the face of the pole to the moving pulley and the column of weights; not assembled - red at the pole
	for (std::size_t index = 0; index < catenary_anchor_size(); ++index)
	{
		auto const *anchor{catenary_anchor_get(index)};
		if (anchor == nullptr)
			continue;
		auto const selected{static_cast<int>(index) == tool.anchor_selected};
		auto const hovered{is_intent(e.m_intent, intent_detail::anchor) && e.m_intent.index == static_cast<int>(index)};
		double shape[4][3];
		ImVec2 head;
		if (false == catenary_anchor_shape(index, shape))
		{
			if (false == anchor->on.empty())
				continue; // on the pole of a suspension: that one shows what's wrong with it
			glm::dvec3 const foot{anchor->pole[0], anchor->pole[1], anchor->pole[2]};
			if (glm::distance(foot, camera) > kCatenaryRange)
				continue;
			projection.line(drawlist, foot, foot + glm::dvec3{0.0, 8.0, 0.0}, overlay_color::invalid, 2.0f);
			if (projection.project(foot + glm::dvec3{0.0, 8.3, 0.0}, head))
				drawlist->AddText(ImVec2(head.x + 6.0f, head.y - 6.0f), overlay_color::invalid, (anchor->name + "  " + STR("doesn't assemble")).c_str());
			continue;
		}
		glm::dvec3 const face{shape[0][0], shape[0][1], shape[0][2]}, lever{shape[1][0], shape[1][1], shape[1][2]};
		glm::dvec3 const top{shape[2][0], shape[2][1], shape[2][2]}, bottom{shape[3][0], shape[3][1], shape[3][2]};
		if (glm::distance(face, camera) > kCatenaryRange)
			continue;
		auto const *result{catenary_anchor_result(index)};
		auto const warned{result != nullptr && false == result->warnings.empty()};
		auto const colour{selected ? overlay_color::selected : hovered ? overlay_color::highlight : warned ? overlay_color::invalid : IM_COL32(120, 200, 255, 170)};
		auto const width{selected || hovered ? 3.0f : 1.5f};
		projection.line(drawlist, face, lever, colour, width);
		projection.line(drawlist, top, bottom, colour, width);
		ImVec2 point;
		if (projection.project(lever, point))
			drawlist->AddCircleFilled(point, width + 1.5f, colour);
		if ((selected || hovered || warned) && projection.project(top + glm::dvec3{0.0, 0.4, 0.0}, head))
		{
			auto label{anchor->name};
			if (result != nullptr)
				label += format("  %d x, %.0f daN", result->weights, result->tension);
			if (warned)
				label += "  (" + result->warnings.front() + ")";
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
			chain_insert insert;
			if (catenary_insert_at(spot, insert))
			{
				// into the span: the two new spans to its ends instead of the old one
				auto const *a{catenary_scene_get(insert.from)};
				auto const *b{catenary_scene_get(insert.to)};
				if (a != nullptr && b != nullptr)
				{
					projection.line(drawlist, wire_of(*a), wire_of(spot.data), overlay_color::grip, 2.5f);
					projection.line(drawlist, wire_of(spot.data), wire_of(*b), overlay_color::grip, 2.5f);
					ImVec2 point;
					if (projection.project(wire_of(spot.data) + glm::dvec3{0.0, 0.6, 0.0}, point))
						drawlist->AddText(ImVec2(point.x + 6.0f, point.y - 14.0f), overlay_color::grip, format(STR_C("into the span %s - %s"), a->name.c_str(), b->name.c_str()).c_str());
				}
			}
			else if (tool.chaining && tool.last >= 0)
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
		if (pole_sharers(static_cast<std::size_t>(tool.selected)).size() > 1)
		{
			// an umbrella or spread cantilevers: the pole goes where this one puts it, the other cantilever goes with it
			if (spot.card == 0)
				return;
			auto const plane{current->has_spread ? glm::dvec3{current->spread[0], current->pole[1], current->spread[2]} : pole_of(*current)};
			auto foot{glm::dvec3{spot.data.pole[0], spot.data.pole[1], spot.data.pole[2]} - (plane - pole_of(*current))};
			foot.y = spot.data.pole[1];
			catenary_pair_move(static_cast<std::size_t>(tool.selected), foot, false);
			return;
		}
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
	// the tensioning device of an anchor (not while a pole is being picked for an anchor or a span)
	if (m_catenary.anchor_pick < 0 && m_catenary.connect_from < 0)
		if (auto const hit{catenary_anchor_hit()}; hit >= 0)
		{
			Intent.index = hit;
			Intent.action = STR("Click: select the anchor");
			Intent.what = kind::catenary;
			Intent.detail = static_cast<int>(intent_detail::anchor);
			return true;
		}
	if (auto const hit{catenary_hit()}; hit >= 0)
	{
		Intent.index = hit;
		Intent.action = m_catenary.anchor_pick >= 0 ? STR("Click: the tensioning device on this pole") : m_catenary.connect_from >= 0 ? STR("Click: hang the span to this pole") : STR("Click: select the pole");
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
	// the second cantilever of an umbrella: by any track, onto the pole of the first one
	if (m_catenary.umbrella_from >= 0 && e.m_hover.track != nullptr && m_catenary.page != 2)
	{
		catenary_spot spot;
		if (catenary_umbrella_spot(*e.m_hover.track, e.m_hover.path, spot))
		{
			Intent.track = e.m_hover.track;
			Intent.path = e.m_hover.path;
			Intent.position = spot.axis;
			Intent.action = spot.card == 0 ? format(STR_C("Umbrella: %s"), spot.problem.c_str())
			                               : format(STR_C("Click: the second cantilever of the umbrella, card %04d execution %d, A = %.2f m"), spot.card, spot.data.execution, spot.reach);
			Intent.what = kind::catenary;
			Intent.detail = static_cast<int>(intent_detail::place);
			return true;
		}
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
		return m_catenary.anchor_pick >= 0 ? STR_C("Wires: LMB on the pole the tensioning device goes on") : m_catenary.connect_from >= 0 ? STR_C("Wires: LMB on the next pole hangs the span to it") : STR_C("Wires: LMB on a span selects its chain, LMB on a pole selects it");
	if (m_catenary.page == 1)
	{
		if (m_catenary.gantry_placing)
			return STR_C("Gantries: LMB the masts");
		return m_catenary.gantry_selected >= 0 ? STR_C("Gantries: LMB by a track puts a cantilever on the selected gantry, drag the beam to move it") : STR_C("Gantries: LMB on a gantry selects it, or place a new one below");
	}
	if (m_catenary.umbrella_from >= 0)
		return STR_C("Umbrella: LMB by the track on the other side of the pole puts the second cantilever on it; Esc stops");
	if (m_catenary.selected >= 0)
		return STR_C("Catenary: the selected pole follows the settings below; C goes on with the chain from it");
	if (m_catenary.chaining && m_catenary.last >= 0)
		return STR_C("Catenary: LMB by the track places the next pole of the chain; Esc ends the chain");
	return STR_C("Catenary: LMB by the track places a pole on that side, LMB on a pole selects it");
}

void catenary_editor::catenary_key_hints(std::vector<key_hint> &Hints) const
{
	Hints = {{"LMB by a track", "places a pole on that side, with the card for its distance"}, {"LMB on a pole", "selects it: the settings then change it at once"}};
	Hints.push_back({"LMB on a tensioning device", "selects the anchor and its chain"});
	Hints.push_back({"LMB under a span", "puts the pole into the chain between the two poles"});
	if (m_catenary.chaining && m_catenary.last >= 0 && m_catenary.selected < 0)
		Hints.push_back({"Esc", "ends the chain: the next pole starts a new one"});
	if (m_catenary.selected >= 0)
	{
		if (m_catenary.umbrella_from >= 0)
			Hints.push_back({"LMB by the other track", "the second cantilever of the umbrella on this pole"});
		Hints.push_back({"C", "the chain goes on from the selected pole"});
		Hints.push_back({"Del", "removes the selected pole"});
	}
	else if (m_catenary.anchor_selected >= 0)
		Hints.push_back({"Del", "removes the selected anchor"});
}

bool catenary_editor::catenary_on_key(int const Key, int const Action, int const Mods)
{
	auto &tool{m_catenary};
	if (Action != GLFW_PRESS || (Mods & (GLFW_MOD_CONTROL | GLFW_MOD_ALT | GLFW_MOD_SHIFT)) != 0 || ImGui::GetIO().WantTextInput)
		return false;
	if (Key == GLFW_KEY_ESCAPE)
	{
		// first what's being picked, then the chain going on; with nothing of that the editor has the key
		if (tool.anchor_pick >= 0 || tool.connect_from >= 0 || tool.umbrella_from >= 0)
		{
			tool.anchor_pick = -1;
			tool.connect_from = -1;
			tool.umbrella_from = -1;
			tool.status.clear();
			return true;
		}
		if (tool.chaining && tool.last >= 0 && tool.selected < 0)
		{
			catenary_chain_end();
			return true;
		}
		return false;
	}
	if (Key == GLFW_KEY_C && tool.selected >= 0)
	{
		catenary_chain_continue(static_cast<std::size_t>(tool.selected));
		return true;
	}
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
