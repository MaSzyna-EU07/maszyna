/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutLegacy.hpp"
#include "editor/layout/layoutGenerator.hpp"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <map>
#include <set>
#include <sstream>
#include <string_view>
#include <tuple>

namespace layout
{

namespace
{

// an end of a path of an old track: the track, the path, which end
struct path_end
{
	std::size_t track{0};
	int path{0};
	bool at_end{false};

	bool operator==(path_end const &) const = default;
};

glm::dvec3 point_of(std::vector<legacy_track> const &Tracks, path_end const &End)
{
	auto const &path{Tracks[End.track].paths[static_cast<std::size_t>(End.path)]};
	return path.points[End.at_end ? segment_data::point::end : segment_data::point::start];
}

// the heading of the path at the end, as the path runs from its start
glm::dvec2 heading_at(segment_data const &Path, bool const AtEnd)
{
	auto const curve{drawn(Path)};
	auto const along{AtEnd ? curve.p3 - curve.p2 : curve.p1 - curve.p0};
	auto plan{geometry::plan_of(along)};
	if (glm::length(plan) < 1e-12)
		plan = geometry::plan_of(curve.p3 - curve.p0);
	return glm::length(plan) < 1e-12 ? glm::dvec2{0.0, 1.0} : glm::normalize(plan);
}

// the ends of the paths by where they are, in squares of a metre
class end_index
{
  public:
	explicit end_index(std::vector<legacy_track> const &Tracks) : m_tracks{Tracks}
	{
		for (std::size_t t = 0; t < Tracks.size(); ++t)
			for (std::size_t p = 0; p < Tracks[t].paths.size(); ++p)
				for (auto const at_end : {false, true})
				{
					path_end const end{t, static_cast<int>(p), at_end};
					m_squares[square_of(point_of(Tracks, end))].push_back(end);
				}
	}

	// the ends at the point, but the one given
	std::vector<path_end> at(glm::dvec3 const &Point, path_end const &But) const
	{
		std::vector<path_end> result;
		auto const [x, z]{square_of(Point)};
		for (auto dx = -1; dx <= 1; ++dx)
			for (auto dz = -1; dz <= 1; ++dz)
			{
				auto const found{m_squares.find({x + dx, z + dz})};
				if (found == m_squares.end())
					continue;
				for (auto const &end : found->second)
					if (false == (end == But) && glm::distance(point_of(m_tracks, end), Point) < legacy_joint)
						result.push_back(end);
			}
		std::ranges::sort(result, [](path_end const &A, path_end const &B) { return std::tie(A.track, A.path, A.at_end) < std::tie(B.track, B.path, B.at_end); });
		return result;
	}

  private:
	std::vector<legacy_track> const &m_tracks;
	std::map<std::pair<long long, long long>, std::vector<path_end>> m_squares;

	static std::pair<long long, long long> square_of(glm::dvec3 const &Point) { return {static_cast<long long>(std::floor(Point.x)), static_cast<long long>(std::floor(Point.z))}; }
};

// how the chain ends at the end of a link: the ends of the other paths there; Out: the end of the chain (else its start)
legacy_end end_of_chain(std::vector<legacy_track> const &Tracks, std::vector<path_end> const &Others, bool const Out)
{
	legacy_end result;
	if (Others.empty())
		return result;
	// the ends of one switch (its start holds both its paths): its main path holds the chain
	auto const one_track{std::ranges::all_of(Others, [&](path_end const &End) { return End.track == Others.front().track; })};
	if (false == one_track)
	{
		result.kind = legacy_end_kind::fork;
		return result;
	}
	auto const &holder{Others.front()};
	auto const &path{Tracks[holder.track].paths[static_cast<std::size_t>(holder.path)]};
	// the heading the holding path runs with through the point, turned to run as the chain does: into it at its start, out of it at its end
	auto heading{heading_at(path, holder.at_end)};
	// a path ending at the point runs on into the chain at its start; one starting there runs away from the end of the chain
	if (holder.at_end == Out)
		heading = -heading;
	result.kind = legacy_end_kind::held;
	result.heading = heading;
	result.by = holder.track;
	return result;
}

// the plain track of Take the chain goes on into from the end of its link, if exactly one is joined there and nothing else is
std::optional<legacy_link> going_on(std::vector<legacy_track> const &Tracks, std::vector<path_end> const &Others, std::vector<bool> const &Taken)
{
	if (Others.size() != 1)
		return std::nullopt;
	auto const &other{Others.front()};
	if (Tracks[other.track].is_switch || false == Taken[other.track])
		return std::nullopt;
	return legacy_link{other.track, other.path, false == other.at_end};
}

path_end exit_of(legacy_link const &Link)
{
	return {Link.track, Link.path, Link.forward};
}

path_end entry_of(legacy_link const &Link)
{
	return {Link.track, Link.path, false == Link.forward};
}

// the letters A to Z made small, as the parser makes the names
std::string lowered(std::string_view const Text)
{
	std::string result{Text};
	for (auto &c : result)
		if (c >= 'A' && c <= 'Z')
			c = static_cast<char>(c - 'A' + 'a');
	return result;
}

bool lexically_before(glm::dvec3 const &A, glm::dvec3 const &B)
{
	return A.x < B.x || (A.x == B.x && A.z < B.z);
}

} // namespace

double legacy_track::velocity() const
{
	std::istringstream words{attributes};
	std::string word;
	while (words >> word)
		if (word == "velocity")
		{
			double value{0.0};
			if (words >> value)
				return value;
		}
	return 0.0;
}

std::vector<legacy_chain> walk_chains(std::vector<legacy_track> const &Tracks, std::vector<std::size_t> const &Take)
{
	std::vector<legacy_chain> result;
	std::vector<bool> taken(Tracks.size(), false);
	for (auto const index : Take)
		if (index < Tracks.size() && false == Tracks[index].is_switch && Tracks[index].paths.size() == 1)
			taken[index] = true;
	end_index const ends{Tracks};
	std::vector<bool> visited(Tracks.size(), false);
	std::vector<std::size_t> order{Take};
	std::ranges::sort(order);
	for (auto const seed : order)
	{
		if (seed >= Tracks.size() || false == taken[seed] || visited[seed])
			continue;
		visited[seed] = true;
		std::vector<legacy_link> forward{{seed, 0, true}};
		std::vector<legacy_link> backward;
		// on from the end of the seed, then back from its start, as far as the plain tracks of Take go on one into the other
		auto const walk = [&ends, &Tracks, &taken, &visited](legacy_link Last, std::vector<legacy_link> &Into) {
			for (;;)
			{
				auto const out{exit_of(Last)};
				auto const others{ends.at(point_of(Tracks, out), out)};
				auto const next{going_on(Tracks, others, taken)};
				if (false == next.has_value() || visited[next->track])
					return others;
				visited[next->track] = true;
				Into.push_back(*next);
				Last = *next;
			}
		};
		auto const after{walk(forward.front(), forward)};
		legacy_link const back_seed{seed, 0, false};
		auto const before{walk(back_seed, backward)};
		legacy_chain chain;
		for (auto link{backward.rbegin()}; link != backward.rend(); ++link)
			chain.links.push_back({link->track, link->path, false == link->forward});
		chain.links.insert(chain.links.end(), forward.begin(), forward.end());
		// the ends: what's at the start of the first link and at the end of the last one, the paths of the chain itself left out
		auto const without_chain = [&chain](std::vector<path_end> Others) {
			std::erase_if(Others, [&](path_end const &End) { return std::ranges::any_of(chain.links, [&](legacy_link const &Link) { return Link.track == End.track; }); });
			return Others;
		};
		chain.start = end_of_chain(Tracks, without_chain(before), false);
		chain.end = end_of_chain(Tracks, without_chain(after), true);
		// each chain one way, whichever track it was found from
		auto const first{point_of(Tracks, entry_of(chain.links.front()))};
		auto const last{point_of(Tracks, exit_of(chain.links.back()))};
		if (lexically_before(last, first))
		{
			std::ranges::reverse(chain.links);
			for (auto &link : chain.links)
				link.forward = false == link.forward;
			std::swap(chain.start, chain.end);
			chain.start.heading = -chain.start.heading;
			chain.end.heading = -chain.end.heading;
		}
		result.push_back(std::move(chain));
	}
	return result;
}

std::vector<chain_path> paths_of(std::vector<legacy_track> const &Tracks, legacy_chain const &Chain)
{
	std::vector<chain_path> result;
	for (auto const &link : Chain.links)
		result.push_back({Tracks[link.track].paths[static_cast<std::size_t>(link.path)], link.forward});
	return result;
}

bool takeover_plan::silent() const
{
	return false == lines.empty() && std::ranges::all_of(lines, [](taken_line const &Line) { return Line.silent; });
}

take_over takeover_plan::command() const
{
	take_over result;
	for (auto const &line : lines)
	{
		take_over::taken each;
		each.start = line.fit.start;
		each.end = line.fit.end;
		each.shape = line.fit.shape;
		each.category = line.category;
		each.hard_splits = line.hard_splits;
		result.lines.push_back(std::move(each));
	}
	return result;
}

takeover_plan plan_takeover(std::vector<legacy_track> const &Tracks, std::vector<std::size_t> const &Take, takeover_options const &Options)
{
	takeover_plan result;
	result.tolerance = Options.recognition.tolerance;
	for (auto const index : Take)
	{
		if (index >= Tracks.size())
			continue;
		auto const &track{Tracks[index]};
		if (track.is_switch)
			result.notes.push_back(std::format("{}: a switch stays as it is, the lines run on from its legs", track.name.empty() ? std::string{"(no name)"} : track.name));
		else if (track.paths.size() != 1)
			result.notes.push_back(std::format("{}: a track of {} paths isn't a plain track", track.name, track.paths.size()));
	}
	for (auto &chain : walk_chains(Tracks, Take))
	{
		auto options{Options.recognition};
		options.start = {chain.start.kind == legacy_end_kind::held, chain.start.heading};
		options.end = {chain.end.kind == legacy_end_kind::held, chain.end.heading};
		taken_line line;
		line.fit = recognize(paths_of(Tracks, chain), options);
		auto const names = [&chain, &Tracks]() {
			std::string text;
			for (auto const &link : chain.links)
				text += (text.empty() ? "" : ", ") + (Tracks[link.track].name.empty() ? std::string{"(no name)"} : Tracks[link.track].name);
			return text;
		};
		if (false == line.fit.valid())
		{
			result.notes.push_back(std::format("{}: {}", names(), line.fit.error));
			continue;
		}
		line.category = Options.category;
		double fastest{0.0};
		for (std::size_t k = 0; k < chain.links.size(); ++k)
		{
			auto const &track{Tracks[chain.links[k].track]};
			fastest = std::max(fastest, std::abs(track.velocity()));
			old_track old;
			old.name = track.name == "none" ? std::string{} : track.name;
			old.surface = track.surface;
			old.attributes = track.attributes;
			old.reversed = false == chain.links[k].forward;
			line.hard_splits.push_back({line.fit.joints[k], old});
		}
		if (fastest > 0.0)
			line.category.vmax = fastest;
		line.silent = line.fit.silent(result.tolerance);
		line.chain = std::move(chain);
		result.worst.add(line.fit.measured);
		result.lines.push_back(std::move(line));
	}
	return result;
}

std::vector<std::size_t> takeover_result::doubtful() const
{
	std::vector<std::size_t> result;
	for (std::size_t i = 0; i < anchors.size(); ++i)
		if (anchors[i].doubtful())
			result.push_back(i);
	return result;
}

takeover_result take_over_tracks(model &Model, takeover_plan const &Plan, std::vector<lineside_object> const &Objects, std::uint32_t const First)
{
	takeover_result result;
	result.made = Model.execute(Plan.command());
	if (false == result.made.done)
		return result;
	// the lines laid, in the order of the plan, and the old tracks along them by their names made small, as the scenery names them
	std::map<std::string, line_id> tracks;
	for (std::size_t i = 0; i < Plan.lines.size() && i < result.made.lines.size(); ++i)
	{
		auto const &line{Plan.lines[i]};
		result.deviations.push_back(line.fit.measured);
		result.worst.add(line.fit.measured);
		for (auto const &split : line.hard_splits)
			if (false == split.track.name.empty())
				tracks[lowered(split.track.name)] = result.made.lines[i];
	}
	result.silent = Plan.silent();
	auto anchors{infer_anchors(Model.state(), Objects, tracks, First)};
	std::erase_if(anchors, [&result](anchor const &Each) { return std::ranges::find(result.made.lines, Each.line) == result.made.lines.end(); });
	result.anchors = std::move(anchors);
	return result;
}

} // namespace layout
