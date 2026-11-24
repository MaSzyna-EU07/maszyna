/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/project/legacyScenery.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <format>
#include <map>
#include <optional>
#include <set>

namespace project
{

namespace
{

using scenery_text::statement;

std::size_t constexpr kPathWords{15}; // a path: the start and its roll, the control vectors, the end and its roll, the radius
int constexpr kEventDepth{4}; // events calling events, followed this deep from a track
std::array<std::string_view, 6> constexpr kTrackEvents{"event0", "event1", "event2", "eventall0", "eventall1", "eventall2"};

std::optional<double> real(std::string_view const Text)
{
	double value{0.0};
	auto const [end, error]{std::from_chars(Text.data(), Text.data() + Text.size(), value)};
	if (error != std::errc{} || end != Text.data() + Text.size())
		return std::nullopt;
	return value;
}

// the numbers of the words from the index on, as many as asked for; none where one isn't a number
std::optional<std::vector<double>> numbers(statement const &Statement, std::size_t const From, std::size_t const Count)
{
	if (From + Count > Statement.words.size())
		return std::nullopt;
	std::vector<double> result;
	for (std::size_t i = From; i < From + Count; ++i)
	{
		auto const value{real(Statement.words[i].text)};
		if (false == value.has_value())
			return std::nullopt;
		result.push_back(*value);
	}
	return result;
}

std::string words_of(statement const &Statement, std::size_t const From, std::size_t const To)
{
	std::string result;
	for (auto i = From; i < To && i < Statement.words.size(); ++i)
		result += (result.empty() ? "" : " ") + Statement.words[i].text;
	return result;
}

std::string where(scenery_text::scenery const &Scenery, statement const &Statement)
{
	return std::format("{} line {}", Scenery.files[Statement.file].name, Statement.line);
}

// a track of the railway: its paths moved by the origin in effect, as the simulator moves them (no rotation, no scale)
std::optional<layout::legacy_track> track_of(statement const &Statement, std::string &Why)
{
	auto const kind{Statement.word(4)};
	if (kind != "normal" && kind != "switch")
		return std::nullopt;
	layout::legacy_track result;
	result.name = Statement.words.size() > 2 ? Statement.words[2].text : std::string{};
	result.is_switch = kind == "switch";
	auto const path_at{Statement.word(12) == "vis" ? std::size_t{19} : std::size_t{13}};
	auto const count{result.is_switch ? std::size_t{2} : std::size_t{1}};
	auto const values{numbers(Statement, path_at, kPathWords * count)};
	if (false == values.has_value() || Statement.words.size() < path_at + kPathWords * count + 1)
	{
		Why = std::format("the track {} can't be read", result.name);
		return std::nullopt;
	}
	result.surface = words_of(Statement, 6, path_at);
	auto const &offset{Statement.context.offset};
	for (std::size_t p = 0; p < count; ++p)
	{
		auto const *v{values->data() + p * kPathWords};
		segment_data path;
		path.points[segment_data::point::start] = glm::dvec3{v[0], v[1], v[2]} + offset;
		path.rolls[0] = static_cast<float>(v[3]);
		path.points[segment_data::point::control1] = {v[4], v[5], v[6]};
		path.points[segment_data::point::control2] = {v[7], v[8], v[9]};
		path.points[segment_data::point::end] = glm::dvec3{v[10], v[11], v[12]} + offset;
		path.rolls[1] = static_cast<float>(v[13]);
		path.radius = static_cast<float>(v[14]);
		result.paths.push_back(path);
	}
	// the attributes up to endtrack
	result.attributes = words_of(Statement, path_at + kPathWords * count, Statement.words.size() - 1);
	return result;
}

// the events, what they act on and the events they call, by their names made small
struct event_graph
{
	std::map<std::string, std::vector<std::string>> targets;
	std::map<std::string, std::vector<std::string>> children;

	void add(statement const &Event)
	{
		auto const name{Event.word(0)};
		auto const type{Event.word(1)};
		auto const list{Event.word(3)};
		std::size_t at{0};
		while (at <= list.size())
		{
			auto end{list.find('|', at)};
			end = end == std::string::npos ? list.size() : end;
			if (auto const target{list.substr(at, end - at)}; false == target.empty() && target != "none")
				targets[name].push_back(target);
			at = end + 1;
		}
		if (type != "multiple")
			return;
		for (std::size_t i = 4; i < Event.words.size(); ++i)
		{
			auto const word{Event.word(i)};
			if (word == "condition" || word == "endevent" || word == "randomdelay")
				break;
			if (word != "else" && word != "none")
				children[name].push_back(word);
		}
	}

	// what the events of the track act on, the events they call followed
	void acted_on(std::vector<std::string> const &Events, std::set<std::string> &Into) const
	{
		std::vector<std::string> open{Events};
		std::set<std::string> seen;
		for (int depth = 0; depth < kEventDepth && false == open.empty(); ++depth)
		{
			std::vector<std::string> next;
			for (auto const &event : open)
			{
				if (false == seen.insert(event).second)
					continue;
				if (auto const found{targets.find(event)}; found != targets.end())
					Into.insert(found->second.begin(), found->second.end());
				if (auto const found{children.find(event)}; found != children.end())
					next.insert(next.end(), found->second.begin(), found->second.end());
			}
			open = std::move(next);
		}
	}
};

// the events a track calls: those of its attributes, and those named after it (<track>:event1...)
std::vector<std::string> events_of(layout::legacy_track const &Track, event_graph const &Events)
{
	std::vector<std::string> result;
	auto const words{scenery_text::split(Track.attributes)};
	for (std::size_t i = 0; i + 1 < words.size(); ++i)
		if (std::ranges::find(kTrackEvents, scenery_text::lowered(words[i].text)) != kTrackEvents.end())
			result.push_back(scenery_text::lowered(words[i + 1].text));
	auto const prefix{scenery_text::lowered(Track.name) + ":"};
	for (auto const &[name, targets] : Events.targets)
		if (name.starts_with(prefix))
			result.push_back(name);
	return result;
}

std::optional<layout::lineside_object> object_of(statement const &Statement, std::string &Why)
{
	auto const type{Statement.word(3)};
	layout::lineside_object result;
	result.kind = type;
	result.name = Statement.words.size() > 2 ? Statement.words[2].text : std::string{};
	if (scenery_text::lowered(result.name) == "none")
		result.name.clear();
	auto const &context{Statement.context};
	if (type == "traction")
	{
		auto const values{numbers(Statement, 11, 12)};
		if (false == values.has_value())
		{
			Why = std::format("the wire {} can't be read", result.name);
			return std::nullopt;
		}
		result.key = Statement.words[4].text;
		for (std::size_t p = 0; p < 4; ++p)
			result.points.push_back(glm::dvec3{(*values)[p * 3], (*values)[p * 3 + 1], (*values)[p * 3 + 2]} + context.offset);
		result.location = (result.points[0] + result.points[1]) * 0.5;
		result.group = infra::category::catenary;
		return result;
	}
	auto const place{numbers(Statement, 4, 3)};
	if (false == place.has_value())
	{
		Why = std::format("the {} {} can't be read", type, result.name);
		return std::nullopt;
	}
	result.location = context.to_world({(*place)[0], (*place)[1], (*place)[2]});
	if (type == "model")
	{
		result.yaw = real(Statement.words.size() > 7 ? Statement.words[7].text : std::string{}).value_or(0.0) + context.rotation.y;
		result.key = Statement.words.size() > 8 ? Statement.words[8].text : std::string{"-"};
		result.group = infra::classify(result.key + " " + result.name, infra::default_rules());
	}
	else
	{
		result.group = infra::category::logic;
		if (type == "memcell" && Statement.words.size() > 10 && Statement.word(10) != "none")
		{
			result.key = Statement.words[10].text;
			result.tracks.push_back(Statement.word(10));
		}
	}
	return result;
}

} // namespace

legacy_scenery read_legacy(scenery_text::scenery const &Scenery, bool const Map)
{
	legacy_scenery result;
	event_graph events;
	for (auto const &statement : Scenery.statements)
		if (statement.keyword == "event" && (false == Map || statement.map))
			events.add(statement);
	for (auto const &statement : Scenery.statements)
	{
		if (statement.keyword != "node" || (Map && false == statement.map))
			continue;
		auto const type{statement.word(3)};
		std::string why;
		if (type == "track")
		{
			if (auto track{track_of(statement, why)})
			{
				track->file = Scenery.files[statement.file].name;
				track->line = statement.line;
				result.tracks.push_back(std::move(*track));
			}
		}
		else if (type == "model" || type == "traction" || type == "memcell" || type == "eventlauncher")
		{
			if (auto object{object_of(statement, why)})
			{
				object->file = Scenery.files[statement.file].name;
				object->line = statement.line;
				result.objects.push_back(std::move(*object));
			}
		}
		if (false == why.empty())
			result.problems.push_back(std::format("{}: {}", where(Scenery, statement), why));
	}
	// the tracks naming each object: those whose events act on it
	std::map<std::string, std::set<std::string>> named;
	for (auto const &track : result.tracks)
	{
		std::set<std::string> acted;
		events.acted_on(events_of(track, events), acted);
		for (auto const &target : acted)
			named[target].insert(scenery_text::lowered(track.name));
	}
	for (auto &object : result.objects)
		if (auto const found{named.find(scenery_text::lowered(object.name))}; false == object.name.empty() && found != named.end())
			for (auto const &track : found->second)
				if (std::ranges::find(object.tracks, track) == object.tracks.end())
					object.tracks.push_back(track);
	return result;
}

legacy_scenery read_legacy(std::filesystem::path const &Folder, std::string const &Root)
{
	auto const scenery{scenery_text::read(Folder, Root)};
	auto result{read_legacy(scenery, false)};
	result.problems.insert(result.problems.begin(), scenery.problems.begin(), scenery.problems.end());
	return result;
}

} // namespace project
