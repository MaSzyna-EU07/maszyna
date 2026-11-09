/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the vertical profile of a road in a window of its own: its nodes picked, dragged up or down and made level, of one grade,
// a vertical curve, over the ground or evened out by the class of the road

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorRoad.hpp"
#include "utilities/translation.h"
#include "world/Road.h"
#include "simulation/simulation.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
// a tooltip of the item above, when hovered
void item_tooltip(char const *Text)
{
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C(Text));
}
// a button which changes the road
bool change_button(char const *Label)
{
	return ImGui::Button(Label);
}
} // namespace

namespace
{

glm::dvec3 bezier(std::array<glm::dvec3, 4> const &P, double const T)
{
	auto const u{1.0 - T};
	return P[0] * (u * u * u) + P[1] * (3.0 * u * u * T) + P[2] * (3.0 * u * T * T) + P[3] * (T * T * T);
}

// the bezier parts a path is made of: one, the path itself
std::vector<std::array<glm::dvec3, 4> *> parts(segment_data &Path)
{
	return {&Path.points};
}

double constexpr kJoin{0.5}; // m: ends this close are the same point

struct oriented_piece
{
	road_node *road;
	bool reversed;
};

// the pieces of a road in order along it, each with the way it goes: from an end no other piece of the road meets; at a fork
// the run stops
std::vector<oriented_piece> ordered_chain(std::vector<road_node *> const &Pieces)
{
	auto const &pieces{Pieces};
	using oriented = oriented_piece;
	// the pieces in order along the road, each with the way it goes: from an end no other piece of the road meets
	auto const end_of = [](road_node const *Road, int const End) { return Road->definition().point(static_cast<double>(End)); };
	auto const meets = [&](glm::dvec3 const &Point, road_node const *Except) {
		for (auto const *other : pieces)
			if (other != Except && (glm::distance(end_of(other, 0), Point) < kJoin || glm::distance(end_of(other, 1), Point) < kJoin))
				return true;
		return false;
	};
	road_node *start{pieces.front()};
	bool start_reversed{false};
	for (auto *road : pieces)
	{
		if (false == meets(end_of(road, 0), road))
		{
			start = road;
			start_reversed = false;
			break;
		}
		if (false == meets(end_of(road, 1), road))
		{
			start = road;
			start_reversed = true;
			break;
		}
	}
	std::vector<oriented> order{{start, start_reversed}};
	std::vector<road_node *> left{pieces};
	left.erase(std::find(left.begin(), left.end(), start));
	while (false == left.empty())
	{
		auto const tail{end_of(order.back().road, order.back().reversed ? 0 : 1)};
		auto next{left.end()};
		bool reversed{false};
		for (auto it{left.begin()}; it != left.end(); ++it)
		{
			if (glm::distance(end_of(*it, 0), tail) < kJoin)
			{
				next = it;
				reversed = false;
				break;
			}
			if (glm::distance(end_of(*it, 1), tail) < kJoin)
			{
				next = it;
				reversed = true;
				break;
			}
		}
		if (next == left.end())
			break; // a fork: the rest isn't in line with this run
		order.push_back({*next, reversed});
		left.erase(next);
	}

	return order;
}

} // namespace

namespace
{

// the ends of a bezier part and its handles as points: the scenery keeps the handles as vectors from the ends,
// both zero for a straight
std::array<glm::dvec3, 4> absolute(std::array<glm::dvec3, 4> const &P)
{
	if (P[1] == glm::dvec3{0.0} && P[2] == glm::dvec3{0.0})
		return {P[0], P[0] + (P[3] - P[0]) / 3.0, P[3] - (P[3] - P[0]) / 3.0, P[3]};
	return {P[0], P[0] + P[1], P[3] + P[2], P[3]};
}

double flat(glm::dvec3 const &V)
{
	return glm::length(glm::dvec2{V.x, V.z});
}

// slope of a vector: metres up for a metre seen from above
double slope(glm::dvec3 const &V)
{
	auto const run{flat(V)};
	return run > 1e-6 ? V.y / run : 0.0;
}

// the road of a piece as a profile: the points where its bezier parts meet or end (the nodes), in order along it, with
// the run to each, the height and the grade there (along the run); the parts know which nodes are at their two ends
struct chain_profile
{
	struct part_ref
	{
		std::size_t entry; // in order
		std::size_t part; // of the path of that piece
		std::size_t node0; // node at the start of the part (its point 0)
		std::size_t node3; // node at its end
		std::size_t first; // of the two, the one earlier along the run
	};
	std::vector<oriented_piece> order;
	std::vector<road_node::state> states;
	std::vector<part_ref> refs;
	std::vector<double> runs;
	std::vector<double> heights;
	std::vector<double> grades;
	std::vector<glm::dvec3> positions;

	bool empty() const { return runs.size() < 2; }
};

chain_profile build_profile(road_node *Road)
{
	chain_profile profile;
	if (Road == nullptr)
		return profile;
	profile.order = ordered_chain(editor_road::chain(*Road));
	for (auto const &entry : profile.order)
		profile.states.push_back(entry.road->definition());
	std::vector<double> sums, counts;
	auto const node = [&](glm::dvec3 const &Position, double const Run) {
		profile.positions.push_back(Position);
		profile.runs.push_back(Run);
		sums.push_back(0.0);
		counts.push_back(0.0);
		return profile.positions.size() - 1;
	};
	for (std::size_t n = 0; n < profile.order.size(); ++n)
	{
		auto const reversed{profile.order[n].reversed};
		auto list{parts(profile.states[n].axis)};
		for (std::size_t index = 0; index < list.size(); ++index)
		{
			auto const k{reversed ? list.size() - 1 - index : index};
			auto const a{absolute(*list[k])};
			auto const start{reversed ? a[3] : a[0]};
			auto const end{reversed ? a[0] : a[3]};
			if (profile.runs.empty())
				node(start, 0.0);
			double length{0.0};
			glm::dvec3 previous{a[0]};
			for (int i = 1; i <= 32; ++i)
			{
				auto const point{bezier(a, i / 32.0)};
				length += flat(point - previous);
				previous = point;
			}
			auto const from{profile.positions.size() - 1};
			auto const to{node(end, profile.runs.back() + length)};
			chain_profile::part_ref ref{n, k, reversed ? to : from, reversed ? from : to, from};
			profile.refs.push_back(ref);
			// the grades at the two ends, along the run
			auto const sign{reversed ? -1.0 : 1.0};
			auto const lead{flat(a[1] - a[0]) > 1e-6 ? a[1] - a[0] : a[3] - a[0]};
			auto const tail{flat(a[3] - a[2]) > 1e-6 ? a[3] - a[2] : a[3] - a[0]};
			sums[ref.node0] += sign * slope(lead);
			counts[ref.node0] += 1.0;
			sums[ref.node3] += sign * slope(tail);
			counts[ref.node3] += 1.0;
		}
	}
	for (std::size_t i = 0; i < profile.positions.size(); ++i)
	{
		profile.heights.push_back(profile.positions[i].y);
		profile.grades.push_back(counts[i] > 0.0 ? sums[i] / counts[i] : 0.0);
	}
	return profile;
}

// the grade at a node from its neighbours, for nodes whose heights were set without one
double neighbour_grade(chain_profile const &Profile, std::vector<double> const &Heights, std::size_t const I)
{
	auto const last{Heights.size() - 1};
	auto const a{I > 0 ? I - 1 : I};
	auto const b{I < last ? I + 1 : I};
	auto const run{Profile.runs[b] - Profile.runs[a]};
	return run > 1e-6 ? (Heights[b] - Heights[a]) / run : 0.0;
}

// the road along the run, sampled every couple of metres between two nodes, as it is now
void sample_between(chain_profile const &Profile, std::size_t const First, std::size_t const Last, std::vector<double> &Runs, std::vector<double> &Heights,
                    std::vector<glm::dvec3> *Points = nullptr)
{
	for (auto const &ref : Profile.refs)
	{
		auto const later{ref.first == ref.node0 ? ref.node3 : ref.node0};
		if (ref.first < First || later > Last)
			continue;
		auto list{parts(const_cast<road_node::state &>(Profile.states[ref.entry]).axis)};
		auto const a{absolute(*list[ref.part])};
		auto const forward{ref.first == ref.node0};
		auto const s0{Profile.runs[ref.first]}, s1{Profile.runs[later]};
		int constexpr steps{32};
		std::vector<double> along{0.0};
		std::vector<glm::dvec3> points{bezier(a, forward ? 0.0 : 1.0)};
		glm::dvec3 previous{points.front()};
		for (int i = 1; i <= steps; ++i)
		{
			auto const t{forward ? i / static_cast<double>(steps) : 1.0 - i / static_cast<double>(steps)};
			auto const point{bezier(a, t)};
			along.push_back(along.back() + flat(point - previous));
			points.push_back(point);
			previous = point;
		}
		auto const scale{along.back() > 1e-9 ? (s1 - s0) / along.back() : 0.0};
		for (std::size_t i = (Runs.empty() ? 0 : 1); i < along.size(); ++i)
		{
			Runs.push_back(s0 + along[i] * scale);
			Heights.push_back(points[i].y);
			if (Points != nullptr)
				Points->push_back(points[i]);
		}
	}
}

// evened out between two nodes, which stay where they are: the grade no steeper than set, the bend of the profile no sharper
// than the radius of a crest (bending down) or a dip (bending up), and as near the heights it had as that lets it.
// The heights and the grades of the nodes between come out of it
void smooth_between(chain_profile const &Profile, std::size_t const First, std::size_t const Last, double const Grade, double const Crest, double const Sag,
                    std::vector<double> &Heights, std::vector<double> &Grades)
{
	std::vector<double> runs, heights;
	sample_between(Profile, First, Last, runs, heights);
	if (runs.size() < 3)
		return;
	auto const from{Profile.runs[First]}, to{Profile.runs[Last]};
	auto const length{to - from};
	if (length < 1.0)
		return;
	auto const count{static_cast<std::size_t>(std::ceil(length / 2.0)) + 1};
	auto const ds{length / static_cast<double>(count - 1)};
	std::vector<double> design(count);
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const s{from + ds * static_cast<double>(i)};
		auto const it{std::lower_bound(runs.begin(), runs.end(), s)};
		auto const k{static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(it - runs.begin(), 1, static_cast<std::ptrdiff_t>(runs.size()) - 1))};
		auto const t{(s - runs[k - 1]) / std::max(1e-9, runs[k] - runs[k - 1])};
		design[i] = heights[k - 1] + (heights[k] - heights[k - 1]) * std::clamp(t, 0.0, 1.0);
	}
	design.front() = Heights[First];
	design.back() = Heights[Last];
	auto const original{design};
	for (int pass = 0; pass < 4000; ++pass)
	{
		auto const pull{pass < 2000 ? 0.02 : 0.0};
		double moved{0.0};
		for (std::size_t i = 1; i + 1 < count; ++i)
		{
			design[i] += (original[i] - design[i]) * pull;
			auto const bend{design[i - 1] - 2.0 * design[i] + design[i + 1]}; // > 0: a dip
			auto const limit{ds * ds / (bend > 0.0 ? Sag : Crest)};
			if (std::abs(bend) > limit)
			{
				auto const excess{(std::abs(bend) - limit) * (bend > 0.0 ? 1.0 : -1.0)};
				design[i] += excess * 0.5;
				moved = std::max(moved, std::abs(excess));
			}
		}
		for (std::size_t i = 1; i + 1 < count; ++i)
			design[i] = std::clamp(design[i], design[i - 1] - Grade * ds, design[i - 1] + Grade * ds);
		for (std::size_t i = count - 2; i > 0; --i)
			design[i] = std::clamp(design[i], design[i + 1] - Grade * ds, design[i + 1] + Grade * ds);
		if (pass > 2200 && moved < 1e-5)
			break;
	}
	auto const height_at = [&](double const S) {
		auto const x{std::clamp((S - from) / ds, 0.0, static_cast<double>(count - 1))};
		auto const i{std::min(static_cast<std::size_t>(x), count - 2)};
		return design[i] + (design[i + 1] - design[i]) * (x - static_cast<double>(i));
	};
	auto const grade_at = [&](double const S) {
		auto const a{std::max(from, S - ds)}, b{std::min(to, S + ds)};
		return b - a > 1e-9 ? (height_at(b) - height_at(a)) / (b - a) : 0.0;
	};
	for (std::size_t k = First; k <= Last; ++k)
	{
		if (k != First && k != Last)
			Heights[k] = height_at(Profile.runs[k]);
		Grades[k] = grade_at(Profile.runs[k]);
	}
}

// classes of roads with about the grades and the radii of vertical curves the rules give them
struct road_class
{
	char const *name;
	float grade; // %
	float crest; // m
	float sag; // m
};
road_class const kRoadClasses[]{
    {"Field or forest track", 12.0f, 300.0f, 200.0f},
    {"Access road (D)", 10.0f, 600.0f, 300.0f},
    {"Local road (L)", 8.0f, 1000.0f, 600.0f},
    {"Collector road (Z)", 7.0f, 1500.0f, 1000.0f},
    {"Main road (G)", 6.0f, 2500.0f, 1500.0f},
    {"Main road of fast traffic (GP)", 5.0f, 4500.0f, 2500.0f},
    {"Expressway (S)", 4.5f, 8000.0f, 3500.0f},
    {"Motorway (A)", 4.0f, 12000.0f, 4500.0f},
};

// the heights and the grades given to the nodes written into the pieces: the ends of every part at the heights, its handles
// rising with the grades; a straight gets its handles made so it can bend up and down. The arms of the junctions at the two
// ends of the run go with them
void write_profile(chain_profile &Profile, std::vector<double> const &Heights, std::vector<double> const &Grades,
                   std::vector<std::pair<road_node *, road_node::state>> &Changes, std::vector<std::pair<junction_node *, junction_node::state>> &Junctions)
{
	for (auto const &ref : Profile.refs)
	{
		auto list{parts(Profile.states[ref.entry].axis)};
		auto &p{*list[ref.part]};
		auto const a{absolute(p)};
		if (p[1] == glm::dvec3{0.0} && p[2] == glm::dvec3{0.0})
		{
			p[1] = a[1] - a[0];
			p[2] = a[2] - a[3];
		}
		auto const sign{Profile.order[ref.entry].reversed ? -1.0 : 1.0};
		p[0].y = Heights[ref.node0];
		p[3].y = Heights[ref.node3];
		p[1].y = sign * Grades[ref.node0] * flat(p[1]);
		p[2].y = -sign * Grades[ref.node3] * flat(p[2]);
	}
	for (std::size_t n = 0; n < Profile.order.size(); ++n)
		Changes.emplace_back(Profile.order[n].road, Profile.states[n]);
	for (auto const end : {std::size_t{0}, Profile.positions.size() - 1})
	{
		for (auto const &arm : editor_road::arms_at(Profile.positions[end]))
		{
			auto entry{std::find_if(Junctions.begin(), Junctions.end(), [&arm](auto const &Listed) { return Listed.first == arm.first; })};
			if (entry == Junctions.end())
			{
				Junctions.emplace_back(arm.first, arm.first->definition());
				entry = std::prev(Junctions.end());
			}
			entry->second.arms[arm.second].position.y = Heights[end];
		}
	}
}

} // namespace

void editor_mode::road_profile_smooth()
{
	auto &tool{m_roadtool};
	auto *road{m_road_profile.road != nullptr ? m_road_profile.road : tool.selected};
	auto profile{build_profile(road)};
	if (profile.empty() || profile.runs.back() < 5.0)
	{
		tool.status = STR_C("The road is too short to even out");
		return;
	}
	auto heights{profile.heights};
	auto grades{profile.grades};
	auto const grade{std::clamp(static_cast<double>(tool.maxgrade), 0.1, 30.0) / 100.0};
	smooth_between(profile, 0, heights.size() - 1, grade, std::max(50.0, static_cast<double>(tool.crest)), std::max(50.0, static_cast<double>(tool.sag)), heights, grades);
	double moved{0.0}, steepest{0.0};
	for (std::size_t i = 0; i < heights.size(); ++i)
	{
		moved = std::max(moved, std::abs(heights[i] - profile.heights[i]));
		steepest = std::max(steepest, std::abs(grades[i]));
	}
	if (road_profile_commit(road, heights, grades))
		tool.status = format(STR_C("Profile evened out over %.0f m, nodes: %zu; steepest %.1f %%, moved by up to %.2f m. Ctrl+Z takes it back"), profile.runs.back(),
		                     heights.size(), steepest * 100.0, moved);
	if (profile.order.size() < editor_road::chain(*road).size())
		tool.status += std::string{". "} + STR_C("The road forks: only the run from its end to the fork was evened out");
}

bool editor_mode::road_profile_commit(road_node *Road, std::vector<double> const &Heights, std::vector<double> const &Grades)
{
	auto profile{build_profile(Road)};
	if (profile.empty() || Heights.size() != profile.runs.size() || Grades.size() != profile.runs.size())
		return false;
	std::vector<std::pair<road_node *, road_node::state>> changes;
	std::vector<std::pair<junction_node *, junction_node::state>> junctions;
	write_profile(profile, Heights, Grades, changes, junctions);
	if (false == road_points_apply(changes, junctions))
		return false;
	road_profile_sample();
	// the points selected in the view go up or down with the nodes
	if (std::any_of(m_road_profile.selected.begin(), m_road_profile.selected.end(), [](char const Selected) { return Selected != 0; }))
		road_profile_select_points();
	return true;
}

void editor_mode::road_profile_operation()
{
	auto &view{m_road_profile};
	auto &tool{m_roadtool};
	auto profile{build_profile(view.road)};
	if (profile.empty() || view.selected.size() != profile.runs.size())
		return;
	std::size_t first{profile.runs.size()}, last{0}, count{0};
	for (std::size_t i = 0; i < view.selected.size(); ++i)
		if (view.selected[i] != 0)
		{
			first = std::min(first, i);
			last = std::max(last, i);
			++count;
		}
	if (count == 0)
	{
		tool.status = STR_C("Select the nodes in the graph first");
		return;
	}
	auto heights{profile.heights};
	auto grades{profile.grades};
	auto const &runs{profile.runs};
	std::string done;
	switch (view.op)
	{
	case 0: // level
		for (std::size_t i = first; i <= last; ++i)
		{
			heights[i] = view.level;
			grades[i] = 0.0;
		}
		done = format(STR_C("Level at %.2f m over %.0f m"), view.level, runs[last] - runs[first]);
		break;
	case 1: // one grade
	{
		if (view.grade_ends && last == first)
		{
			tool.status = STR_C("Select two nodes at least: the grade runs from the first to the last");
			return;
		}
		auto const grade{view.grade_ends ? (heights[last] - heights[first]) / std::max(1e-6, runs[last] - runs[first]) : view.grade / 100.0};
		for (std::size_t i = first; i <= last; ++i)
		{
			heights[i] = profile.heights[first] + grade * (runs[i] - runs[first]);
			grades[i] = grade;
		}
		done = format(STR_C("One grade of %.2f %% over %.0f m"), grade * 100.0, runs[last] - runs[first]);
		break;
	}
	case 2: // a vertical curve between the grade coming in and the one going out
	{
		if (last == first)
		{
			tool.status = STR_C("Select two nodes at least: the curve goes between the first and the last");
			return;
		}
		auto const g1{first > 0 ? (heights[first] - heights[first - 1]) / std::max(1e-6, runs[first] - runs[first - 1]) : grades[first]};
		auto const g2{last + 1 < heights.size() ? (heights[last + 1] - heights[last]) / std::max(1e-6, runs[last + 1] - runs[last]) : grades[last]};
		if (std::abs(g2 - g1) < 1e-5)
		{
			for (std::size_t i = first; i <= last; ++i)
			{
				heights[i] = heights[first] + g1 * (runs[i] - runs[first]);
				grades[i] = g1;
			}
			done = STR_C("The grades on both sides are the same: a straight grade between them");
			break;
		}
		// where the two grades meet
		auto const s_first{runs[first]}, s_last{runs[last]};
		auto const meet{(heights[last] - heights[first] + g1 * s_first - g2 * s_last) / (g1 - g2)};
		if (meet <= s_first || meet >= s_last)
		{
			tool.status = STR_C("The grades before and after the selected nodes don't meet between them: select more of the road, or set a grade first");
			return;
		}
		auto const change{std::abs(g2 - g1)};
		auto length{static_cast<double>(view.radius) * change};
		length = std::min(length, 2.0 * std::min(meet - s_first, s_last - meet));
		auto const start{meet - length * 0.5};
		auto const height_on_first = [&](double const S) { return heights[first] + g1 * (S - s_first); };
		auto const height_on_last = [&](double const S) { return heights[last] + g2 * (S - s_last); };
		for (std::size_t i = first; i <= last; ++i)
		{
			auto const s{runs[i]};
			if (s <= start)
			{
				heights[i] = height_on_first(s);
				grades[i] = g1;
			}
			else if (s >= start + length)
			{
				heights[i] = height_on_last(s);
				grades[i] = g2;
			}
			else
			{
				auto const x{s - start};
				heights[i] = height_on_first(s) + (g2 - g1) / (2.0 * length) * x * x;
				grades[i] = g1 + (g2 - g1) * x / length;
			}
		}
		done = format(g2 < g1 ? STR_C("A crest curve of R %.0f m, %.0f m long, between %.2f %% and %.2f %%") : STR_C("A dip curve of R %.0f m, %.0f m long, between %.2f %% and %.2f %%"),
		              length / change, length, g1 * 100.0, g2 * 100.0);
		if (length / change + 1.0 < view.radius)
			done += std::string{". "} + STR_C("No room for the radius set: the curve is as long as the selection lets it be");
		break;
	}
	case 3: // over the ground
	{
		std::size_t found{0};
		for (std::size_t i = first; i <= last; ++i)
		{
			double ground{0.0};
			if (m_streamer.height_at(profile.positions[i].x, profile.positions[i].z, ground))
			{
				heights[i] = ground + view.offset;
				++found;
			}
		}
		for (std::size_t i = first; i <= last; ++i)
			grades[i] = neighbour_grade(profile, heights, i);
		done = format(STR_C("Nodes put %.2f m over the ground: %zu"), view.offset, found);
		break;
	}
	case 4: // moved up or down
		for (std::size_t i = 0; i < heights.size(); ++i)
			if (view.selected[i] != 0)
				heights[i] += view.shift;
		done = format(STR_C("Selected nodes moved by %+.2f m"), view.shift);
		break;
	case 5: // evened out between the first and the last
	{
		if (last == first)
		{
			tool.status = STR_C("Select two nodes at least: what's between them is evened out");
			return;
		}
		auto const grade{std::clamp(static_cast<double>(tool.maxgrade), 0.1, 30.0) / 100.0};
		smooth_between(profile, first, last, grade, std::max(50.0, static_cast<double>(tool.crest)), std::max(50.0, static_cast<double>(tool.sag)), heights, grades);
		done = format(STR_C("Evened out over %.0f m: grade up to %.1f %%, crests R %.0f m, dips R %.0f m"), runs[last] - runs[first], tool.maxgrade, tool.crest, tool.sag);
		break;
	}
	default:
		return;
	}
	if (road_profile_commit(view.road, heights, grades))
		tool.status = done + ". " + STR_C("Ctrl+Z takes it back");
}

void editor_mode::road_profile_window_open()
{
	m_road_profile.open = true;
	m_road_profile.road = m_roadtool.selected;
	road_profile_sample();
}

void editor_mode::road_profile_window_toggle()
{
	if (m_road_profile.open)
		m_road_profile.open = false;
	else
		road_profile_window_open();
}

void editor_mode::road_profile_sample()
{
	auto &view{m_road_profile};
	view.runs.clear();
	view.heights.clear();
	view.ground.clear();
	view.history = m_history.size();
	auto const profile{build_profile(view.road)};
	if (profile.empty())
	{
		view.node_runs.clear();
		view.node_heights.clear();
		view.node_positions.clear();
		view.selected.clear();
		return;
	}
	// the selection kept where the nodes are the same ones
	if (profile.positions.size() != view.node_positions.size())
		view.selected.assign(profile.positions.size(), 0);
	view.node_runs.assign(profile.runs.begin(), profile.runs.end());
	view.node_heights.assign(profile.heights.begin(), profile.heights.end());
	view.node_positions = profile.positions;
	std::vector<double> runs, heights;
	std::vector<glm::dvec3> points;
	sample_between(profile, 0, profile.runs.size() - 1, runs, heights, &points);
	for (std::size_t i = 0; i < runs.size(); ++i)
	{
		view.runs.push_back(static_cast<float>(runs[i]));
		view.heights.push_back(static_cast<float>(heights[i]));
	}
	// the ground under the road, every few metres of it
	view.ground_runs.clear();
	double last{-1e9};
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		if (runs[i] - last < 3.0 && i + 1 < points.size())
			continue;
		last = runs[i];
		double ground{0.0};
		view.ground_runs.push_back(static_cast<float>(runs[i]));
		view.ground.push_back(m_streamer.height_at(points[i].x, points[i].z, ground) ? static_cast<float>(ground) : std::numeric_limits<float>::quiet_NaN());
	}
}

// the selection of the nodes in the graph given to the road tool as its selected points, so the view shows them too
void editor_mode::road_profile_select_points()
{
	auto &view{m_road_profile};
	auto &tool{m_roadtool};
	tool.points.clear();
	for (std::size_t i = 0; i < view.selected.size() && i < view.node_positions.size(); ++i)
		if (view.selected[i] != 0)
			tool.points.push_back(view.node_positions[i]);
	if (false == tool.points.empty())
	{
		tool.selected = nullptr;
		tool.junction = nullptr;
		tool.marker = nullptr;
	}
	view.synced = tool.points;
}

void editor_mode::render_road_profile_window()
{
	auto &view{m_road_profile};
	if (false == view.open)
		return;
	auto &tool{m_roadtool};
	// the road gone (deleted, or another scenery): the window waits for another one
	if (view.road != nullptr)
	{
		bool alive{false};
		for (auto const *road : simulation::Roads.sequence())
			if (road == view.road && false == road->m_editorremoved)
				alive = true;
		if (false == alive)
		{
			view.road = nullptr;
			road_profile_sample();
		}
	}
	// the window follows the road selected: another piece clicked, the profile of its road read
	if (tool.selected != nullptr && tool.selected != view.road)
	{
		view.road = tool.selected;
		view.selected.clear();
		view.node_positions.clear();
		road_profile_sample();
	}
	// undone or done again: read anew
	if (view.road != nullptr && view.history != m_history.size())
		road_profile_sample();
	// points selected in the view: the nodes of the graph with them
	if (tool.points != view.synced)
	{
		view.synced = tool.points;
		for (std::size_t i = 0; i < view.selected.size(); ++i)
			view.selected[i] = std::any_of(tool.points.begin(), tool.points.end(), [&](glm::dvec3 const &Point) { return glm::distance(Point, view.node_positions[i]) < 0.3; }) ? 1 : 0;
		// a point of another road: the window goes to it
		if (false == tool.points.empty() && std::none_of(view.selected.begin(), view.selected.end(), [](char const Selected) { return Selected != 0; }))
		{
			auto const ends{editor_road::ends_at(tool.points.front())};
			if (false == ends.empty() && ends.front().road != view.road)
			{
				view.road = ends.front().road;
				view.node_positions.clear();
				road_profile_sample();
				for (std::size_t i = 0; i < view.selected.size(); ++i)
					view.selected[i] = std::any_of(tool.points.begin(), tool.points.end(), [&](glm::dvec3 const &Point) { return glm::distance(Point, view.node_positions[i]) < 0.3; }) ? 1 : 0;
			}
		}
	}
	ImGui::SetNextWindowSize(ImVec2(820.0f, 560.0f), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin(STR_C("Vertical profile of the road###roadprofile"), &view.open))
	{
		ImGui::End();
		return;
	}
	if (view.road == nullptr || view.node_runs.size() < 2)
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.82f, 0.24f, 1.0f));
		ImGui::TextWrapped("%s", STR_C("Select a road (Roads, Select, LMB on a piece) and its profile shows here"));
		ImGui::PopStyleColor();
		ImGui::End();
		return;
	}
	ImGui::TextUnformatted(view.road->name().c_str());
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Read again##roadprofile")))
		road_profile_sample();
	ImGui::SameLine();
	char const *const exaggerations[]{STR_C("fit to the window"), "1 : 1", "1 : 2", "1 : 5", "1 : 10", "1 : 20", "1 : 50"};
	int const factors[]{0, 1, 2, 5, 10, 20, 50};
	ImGui::SetNextItemWidth(150.0f);
	ImGui::Combo(STR_C("Heights stretched##roadprofile"), &view.exaggeration, exaggerations, IM_ARRAYSIZE(exaggerations));
	item_tooltip("How much more the heights are drawn than the lengths: fit to the window shows every little change as a hill;\n1:10 is what the profiles of roads are usually drawn at");

	// the graph
	auto const size{ImVec2(ImGui::GetContentRegionAvail().x, 230.0f)};
	auto const origin{ImGui::GetCursorScreenPos()};
	ImGui::InvisibleButton("##roadprofilegraph", size);
	auto const graph_hovered{ImGui::IsItemHovered()};
	auto *draw{ImGui::GetWindowDrawList()};
	draw->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(25, 28, 32, 255));
	auto const length{std::max(1.0f, view.node_runs.back())};
	float low{std::numeric_limits<float>::max()}, high{-low};
	for (auto const h : view.heights)
	{
		low = std::min(low, h);
		high = std::max(high, h);
	}
	for (auto const h : view.ground)
		if (std::isfinite(h))
		{
			low = std::min(low, h);
			high = std::max(high, h);
		}
	auto const factor{factors[std::clamp(view.exaggeration, 0, 6)]};
	if (factor == 0)
	{
		high = std::max(high, low + 2.0f);
		auto const margin{(high - low) * 0.08f};
		low -= margin;
		high += margin;
	}
	else
	{
		// metres for a pixel across, and that over the factor up
		auto const span{length / size.x * size.y / static_cast<float>(factor)};
		auto const middle{(low + high) * 0.5f};
		low = middle - span * 0.5f;
		high = middle + span * 0.5f;
	}
	auto const at = [&](float const Run, float const Height) {
		return ImVec2(origin.x + Run / length * size.x, origin.y + size.y - (Height - low) / (high - low) * size.y);
	};
	auto const metres_per_pixel{(high - low) / size.y};
	draw->PushClipRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), true);
	// the grid: heights every round number of metres, the run every round number of metres
	auto const round_step = [](float const Range, float const Lines) {
		float const steps[]{0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 50.0f, 100.0f, 200.0f, 500.0f, 1000.0f, 2000.0f, 5000.0f};
		for (auto const step : steps)
			if (Range / step <= Lines)
				return step;
		return 10000.0f;
	};
	auto const ystep{round_step(high - low, 6.0f)};
	for (auto y{std::ceil(low / ystep) * ystep}; y <= high; y += ystep)
	{
		auto const p{at(0.0f, y)};
		draw->AddLine(ImVec2(origin.x, p.y), ImVec2(origin.x + size.x, p.y), IM_COL32(255, 255, 255, 22));
		auto const label{format("%.*f m", ystep < 1.0f ? 1 : 0, y)};
		draw->AddText(ImVec2(origin.x + 4.0f, p.y - ImGui::GetTextLineHeight()), IM_COL32(200, 200, 200, 140), label.c_str());
	}
	auto const xstep{round_step(length, 8.0f)};
	for (auto x{xstep}; x < length; x += xstep)
	{
		auto const p{at(x, low)};
		draw->AddLine(ImVec2(p.x, origin.y), ImVec2(p.x, origin.y + size.y), IM_COL32(255, 255, 255, 18));
		auto const label{format("%.0f", x)};
		draw->AddText(ImVec2(p.x + 3.0f, origin.y + size.y - ImGui::GetTextLineHeight() - 2.0f), IM_COL32(200, 200, 200, 120), label.c_str());
	}
	// the ground brown, the road grey
	for (std::size_t i = 1; i < view.ground.size(); ++i)
		if (std::isfinite(view.ground[i - 1]) && std::isfinite(view.ground[i]))
			draw->AddLine(at(view.ground_runs[i - 1], view.ground[i - 1]), at(view.ground_runs[i], view.ground[i]), IM_COL32(170, 120, 70, 255), 2.0f);
	for (std::size_t i = 1; i < view.runs.size(); ++i)
		draw->AddLine(at(view.runs[i - 1], view.heights[i - 1]), at(view.runs[i], view.heights[i]), IM_COL32(220, 220, 225, 255), 2.0f);

	// the nodes: picked, dragged up or down, boxed
	auto const &io{ImGui::GetIO()};
	auto const mouse{io.MousePos};
	int hovered_node{-1};
	{
		float best{8.0f * 8.0f};
		for (std::size_t i = 0; i < view.node_runs.size(); ++i)
		{
			auto const p{at(view.node_runs[i], view.node_heights[i])};
			auto const d{(p.x - mouse.x) * (p.x - mouse.x) + (p.y - mouse.y) * (p.y - mouse.y)};
			if (d < best)
			{
				best = d;
				hovered_node = static_cast<int>(i);
			}
		}
	}
	if (view.selected.size() != view.node_runs.size())
		view.selected.assign(view.node_runs.size(), 0);
	auto const drag_offset = [&]() {
		auto offset{(view.drag_mouse - mouse.y) * view.drag_scale};
		if (io.KeyCtrl)
			offset *= 0.1f;
		return std::round(offset * 100.0f) / 100.0f;
	};
	if (graph_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
	{
		if (hovered_node >= 0)
		{
			auto &chosen{view.selected[hovered_node]};
			if (io.KeyShift)
				chosen = chosen != 0 ? 0 : 1;
			else if (chosen == 0)
			{
				std::fill(view.selected.begin(), view.selected.end(), 0);
				chosen = 1;
			}
			if (chosen != 0)
			{
				view.dragging = true;
				view.drag_mouse = mouse.y;
				view.drag_scale = metres_per_pixel;
			}
			road_profile_select_points();
		}
		else
		{
			view.boxing = true;
			view.box_from = mouse;
		}
	}
	if (view.dragging && false == ImGui::IsMouseDown(ImGuiMouseButton_Left))
	{
		view.dragging = false;
		auto const offset{drag_offset()};
		auto profile{build_profile(view.road)};
		if (std::abs(offset) >= 0.005f && profile.runs.size() == view.selected.size())
		{
			auto heights{profile.heights};
			auto grades{profile.grades};
			for (std::size_t i = 0; i < heights.size(); ++i)
				if (view.selected[i] != 0)
					heights[i] += offset;
			for (std::size_t i = 0; i < heights.size(); ++i)
				if (view.selected[i] != 0)
					grades[i] = neighbour_grade(profile, heights, i);
			if (road_profile_commit(view.road, heights, grades))
			{
				tool.status = format(STR_C("Nodes moved by %+.2f m. Ctrl+Z takes it back"), offset);
				road_profile_select_points();
			}
		}
	}
	if (view.boxing)
	{
		auto const a{ImVec2(std::min(view.box_from.x, mouse.x), std::min(view.box_from.y, mouse.y))};
		auto const b{ImVec2(std::max(view.box_from.x, mouse.x), std::max(view.box_from.y, mouse.y))};
		if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
		{
			draw->AddRectFilled(a, b, IM_COL32(40, 220, 255, 30));
			draw->AddRect(a, b, IM_COL32(40, 220, 255, 160));
		}
		else
		{
			view.boxing = false;
			auto const tiny{b.x - a.x < 3.0f && b.y - a.y < 3.0f};
			if (false == io.KeyShift)
				std::fill(view.selected.begin(), view.selected.end(), 0);
			if (false == tiny)
				for (std::size_t i = 0; i < view.node_runs.size(); ++i)
				{
					auto const p{at(view.node_runs[i], view.node_heights[i])};
					// across the run is enough: a box over a stretch takes all its nodes, whatever their heights
					if (p.x >= a.x && p.x <= b.x)
						view.selected[i] = 1;
				}
			road_profile_select_points();
		}
	}
	auto const preview{view.dragging ? drag_offset() : 0.0f};
	for (std::size_t i = 0; i < view.node_runs.size(); ++i)
	{
		auto const chosen{view.selected[i] != 0};
		auto const p{at(view.node_runs[i], view.node_heights[i] + (chosen ? preview : 0.0f))};
		if (chosen && view.dragging)
			draw->AddLine(at(view.node_runs[i], view.node_heights[i]), p, IM_COL32(255, 210, 60, 200), 1.5f);
		draw->AddCircleFilled(p, chosen ? 5.5f : 4.0f, chosen ? IM_COL32(40, 220, 255, 255) : IM_COL32(255, 255, 255, 230));
		if (static_cast<int>(i) == hovered_node)
			draw->AddCircle(p, 8.0f, IM_COL32(255, 255, 255, 200), 0, 1.5f);
	}
	draw->PopClipRect();
	if (view.dragging)
		ImGui::SetTooltip(STR_C("%+.2f m"), preview);
	else if (graph_hovered && view.runs.size() >= 2)
	{
		auto const run{(mouse.x - origin.x) / size.x * length};
		auto const it{std::lower_bound(view.runs.begin(), view.runs.end(), run)};
		auto const i{static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(it - view.runs.begin(), 1, static_cast<std::ptrdiff_t>(view.runs.size()) - 1))};
		auto const grade{(view.heights[i] - view.heights[i - 1]) / std::max(0.01f, view.runs[i] - view.runs[i - 1]) * 100.0f};
		auto const g{std::lower_bound(view.ground_runs.begin(), view.ground_runs.end(), run)};
		auto const gi{static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(g - view.ground_runs.begin(), 0, static_cast<std::ptrdiff_t>(view.ground.size()) - 1))};
		draw->AddLine(ImVec2(mouse.x, origin.y), ImVec2(mouse.x, origin.y + size.y), IM_COL32(255, 255, 255, 60));
		if (hovered_node >= 0)
			ImGui::SetTooltip(STR_C("Node %d: %.0f m, height %.2f m"), hovered_node + 1, view.node_runs[hovered_node], view.node_heights[hovered_node]);
		else
			ImGui::SetTooltip(STR_C("%.0f m: road %.2f m, ground %.2f m, grade %.1f %%"), view.runs[i], view.heights[i], view.ground.empty() ? 0.0f : view.ground[gi], grade);
	}
	// the hints, yellow as everywhere
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.82f, 0.24f, 1.0f));
	ImGui::TextWrapped("%s", STR_C("LMB on a node: select it   Shift+LMB: add or take out   drag a node up or down: its height (Ctrl: finer)   drag over the graph: select a stretch   the selected nodes show in the view too; Alt+LMB on a point there drags its height"));
	ImGui::PopStyleColor();
	ImGui::TextDisabled(STR_C("%.0f m long, nodes: %zu, heights %.1f - %.1f m; the road grey, the ground brown"), length, view.node_runs.size(), low, high);

	// what's done to the selected nodes
	std::size_t count{0};
	std::size_t first{view.selected.size()}, last{0};
	float sum{0.0f};
	for (std::size_t i = 0; i < view.selected.size(); ++i)
		if (view.selected[i] != 0)
		{
			++count;
			first = std::min(first, i);
			last = std::max(last, i);
			sum += view.node_heights[i];
		}
	ImGui::SeparatorText(STR_C("Selected nodes##roadprofile"));
	if (ImGui::SmallButton(STR_C("Select all##roadprofile")))
	{
		std::fill(view.selected.begin(), view.selected.end(), 1);
		road_profile_select_points();
	}
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Select none##roadprofile")))
	{
		std::fill(view.selected.begin(), view.selected.end(), 0);
		road_profile_select_points();
	}
	ImGui::SameLine();
	if (count == 0)
		ImGui::TextDisabled("%s", STR_C("none: click the nodes in the graph or the points in the view"));
	else
		ImGui::Text(STR_C("%zu, from %.0f m to %.0f m"), count, view.node_runs[first], view.node_runs[last]);
	char const *const operations[]{STR_C("Level##roadop"), STR_C("One grade"), STR_C("Vertical curve"), STR_C("Over the ground"), STR_C("Up or down by"), STR_C("Even out")};
	ImGui::SetNextItemWidth(200.0f);
	ImGui::Combo(STR_C("What to make of them##roadprofile"), &view.op, operations, IM_ARRAYSIZE(operations));
	switch (view.op)
	{
	case 0:
		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputFloat(STR_C("Height (m)##roadlevel"), &view.level, 0.1f, 1.0f, "%.2f");
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("their average")) && count > 0)
			view.level = std::round(sum / static_cast<float>(count) * 100.0f) / 100.0f;
		item_tooltip("All the selected nodes at this height, with no grade at them");
		break;
	case 1:
		ImGui::Checkbox(STR_C("From the first selected node to the last##roadgrade"), &view.grade_ends);
		if (false == view.grade_ends)
		{
			ImGui::SetNextItemWidth(120.0f);
			ImGui::InputFloat(STR_C("Grade (%)##roadgrade"), &view.grade, 0.1f, 1.0f, "%.2f");
			item_tooltip("From the first selected node on, up along the road where it's more than 0");
		}
		break;
	case 2:
		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputFloat(STR_C("Radius (m)##roadcurve"), &view.radius, 100.0f, 1000.0f, "%.0f");
		view.radius = std::clamp(view.radius, 20.0f, 100000.0f);
		item_tooltip("A curve rounding off where the grade before the selected nodes meets the one after them:\n"
		             "the grades are read from the nodes just outside the selection, so select a stretch with some of the road left on both sides");
		break;
	case 3:
		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputFloat(STR_C("Over the ground by (m)##roadground"), &view.offset, 0.05f, 0.5f, "%.2f");
		item_tooltip("Each selected node put this much over the ground under it; less than 0 puts it into a cutting");
		break;
	case 4:
		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputFloat(STR_C("By (m)##roadshift"), &view.shift, 0.1f, 1.0f, "%+.2f");
		item_tooltip("Less than 0 lowers them: a road into a cutting, then lead the terrain to it");
		break;
	case 5:
		ImGui::TextDisabled("%s", STR_C("By the grade and the radii of the class of the road below; the first and the last selected node stay"));
		break;
	default:
		break;
	}
	ImGui::BeginDisabled(count == 0);
	if (change_button(STR_C("Apply to the selected nodes##roadprofile")))
		road_profile_operation();
	ImGui::EndDisabled();

	ImGui::SeparatorText(STR_C("The whole road##roadprofile"));
	{
		std::vector<char const *> classes;
		for (auto const &entry : kRoadClasses)
			classes.push_back(STR_C(entry.name));
		ImGui::SetNextItemWidth(260.0f);
		if (ImGui::Combo(STR_C("Class of the road##roadprofile"), &view.road_class, classes.data(), static_cast<int>(classes.size())))
		{
			auto const &entry{kRoadClasses[std::clamp(view.road_class, 0, static_cast<int>(classes.size()) - 1)]};
			tool.maxgrade = entry.grade;
			tool.crest = entry.crest;
			tool.sag = entry.sag;
			view.radius = entry.crest;
		}
		item_tooltip("Sets the steepest grade and the smallest radii of the vertical curves about as the rules have them for that class");
	}
	if (ImGui::TreeNode(STR_C("Grade and radii##roadprofile")))
	{
		ImGui::SetNextItemWidth(120.0f);
		ImGui::DragFloat(STR_C("Steepest grade (%)##road"), &tool.maxgrade, 0.1f, 0.5f, 20.0f, "%.1f");
		ImGui::SetNextItemWidth(120.0f);
		ImGui::DragFloat(STR_C("Crest radius (m)##road"), &tool.crest, 50.0f, 50.0f, 50000.0f, "%.0f");
		item_tooltip("The smallest radius of the vertical curve over a hilltop: the larger, the farther a driver sees over it");
		ImGui::SetNextItemWidth(120.0f);
		ImGui::DragFloat(STR_C("Dip radius (m)##road"), &tool.sag, 50.0f, 50.0f, 50000.0f, "%.0f");
		ImGui::TreePop();
	}
	if (change_button(STR_C("Even out the whole road")))
		road_profile_smooth();
	item_tooltip("The heights of the whole road: grades no steeper and curves no sharper than set, as near the heights it had as that lets;\n"
	             "its two ends stay where they are, so it keeps meeting the junctions. Ctrl+Z takes it back");

	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
	ImGui::End();
}


