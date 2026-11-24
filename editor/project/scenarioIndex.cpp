/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/project/scenarioIndex.hpp"
#include "editor/project/sceneryText.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

namespace project
{

namespace
{

using scenery_text::statement;

// the name of the file as the strings of the game hold it, none where the code page can't (Windows): generic_string() throws there,
// and the game couldn't open such a file by its name either
std::optional<std::string> name_of(std::filesystem::path const &Path)
{
	try
	{
		return Path.filename().generic_string();
	}
	catch (std::system_error const &)
	{
		return std::nullopt;
	}
}

// the file has the extension, in any case, without its name being made a string
bool has_extension(std::filesystem::path const &Path, std::u8string_view const Extension)
{
	auto const extension{Path.extension().u8string()};
	return extension.size() == Extension.size() && std::ranges::equal(extension, Extension, [](char8_t const A, char8_t const B) { return std::tolower(static_cast<unsigned char>(A)) == std::tolower(static_cast<unsigned char>(B)); });
}

// the files the files include, by a quick look at their words, each file looked at once for all the scenarios: a scenario whose includes
// don't reach the file of the map doesn't use it, and isn't read as a whole
class include_graph
{
  public:
	explicit include_graph(std::filesystem::path Folder) : m_folder{std::move(Folder)} {}

	// the includes of the root reach the map, or a file named by the parameters of a template, which only reading it all tells
	bool may_reach(std::string const &Root, std::string const &Map, bool const SkipTerrain)
	{
		std::vector<std::string> pending{Root};
		std::set<std::string> seen;
		while (false == pending.empty())
		{
			auto const name{pending.back()};
			pending.pop_back();
			auto const key{scenery_text::lowered(name)};
			if (false == seen.insert(key).second)
				continue;
			if (scenery_text::same_file(name, Map) || name.find("(p") != std::string::npos)
				return true;
			if (SkipTerrain && name != Root && key.find("_ter.scm") != std::string::npos)
				continue;
			for (auto const &include : includes(key, name))
				pending.push_back(include);
		}
		return false;
	}

  private:
	std::vector<std::string> const &includes(std::string const &Key, std::string const &Name)
	{
		auto found{m_includes.find(Key)};
		if (found != m_includes.end())
			return found->second;
		auto const text{scenery_text::file_text(m_folder / Name)};
		return m_includes.emplace(Key, scenery_text::includes_in(text.value_or(std::string{}))).first->second;
	}

	std::filesystem::path m_folder;
	std::map<std::string, std::vector<std::string>> m_includes;
};

// the events a track calls by its name and theirs, <track>:<sequence> (TTrack::AssignEvents(), Global.iHiddenEvents)
constexpr std::array<std::string_view, 6> named_sequences{"event0", "eventall0", "event1", "eventall1", "event2", "eventall2"};

// the words which end the parameters of an event (basic_event::is_keyword())
bool event_keyword(std::string_view const Word)
{
	return Word == "endevent" || Word == "randomdelay" || Word == "departuredelay";
}

// the targets of an event, | or , between them, none for none (basic_event::deserialize_targets())
std::vector<std::string> targets_of(std::string const &Word)
{
	std::vector<std::string> result;
	std::size_t at{0};
	while (at <= Word.size())
	{
		auto end{Word.find_first_of("|,", at)};
		end = (end == std::string::npos ? Word.size() : end);
		auto target{Word.substr(at, end - at)};
		if (false == target.empty() && target != "none")
			result.push_back(std::move(target));
		at = end + 1;
	}
	return result;
}

// the statements which name or define what the index is about
bool kept(std::string_view const Keyword, std::string_view const Type)
{
	if (Keyword == "node")
		return Type == "track" || Type == "memcell" || Type == "eventlauncher" || Type == "dynamic";
	return Keyword == "event" || Keyword == "trainset" || Keyword == "endtrainset" || Keyword == "isolated";
}

class collector
{
  public:
	collector(scenery_text::scenery const &Scenery, std::vector<reference> &Output) : m_scenery{Scenery}, m_output{Output} {}

	void name(reference_kind const Kind, std::string const &Name, reference_use const Use, std::string const &From, statement const &Statement)
	{
		if (Name.empty() || Name == "none")
			return;
		m_output.push_back({Kind, Name, Use, From, m_scenery.files[Statement.file].name, Statement.line});
	}

	void event(statement const &Event)
	{
		auto const name{Event.word(0)};
		auto const type{Event.word(1)};
		auto const targets{targets_of(Event.word(3))};
		// the parameters, up to the end of the event
		std::vector<std::string> parameters;
		for (std::size_t i = 4; i < Event.words.size(); ++i)
			parameters.push_back(Event.word(i));
		if (type == "multiple")
		{
			bool tracks{false};
			bool cells{false};
			bool condition{false};
			for (auto const &word : parameters)
			{
				if (condition)
				{
					tracks = tracks || word == "trackoccupied" || word == "trackfree";
					cells = cells || word == "memcompare" || word == "memcompareex";
					continue;
				}
				if (event_keyword(word))
					break;
				if (word == "condition")
					condition = true;
				else if (word != "else" && false == word.starts_with("none_"))
					this->name(reference_kind::event, word, reference_use::event_child, name, Event);
			}
			for (auto const &target : targets)
			{
				if (tracks)
					this->name(reference_kind::track, target, reference_use::condition, name, Event);
				if (cells)
					this->name(reference_kind::memcell, target, reference_use::condition, name, Event);
			}
		}
		else if (type == "addvalues" || type == "updatevalues" || type == "getvalues" || type == "whois" || type == "logvalues" || type == "copyvalues")
		{
			for (auto const &target : targets)
				this->name(reference_kind::memcell, target, reference_use::event_target, name, Event);
			if (type == "copyvalues" && false == parameters.empty() && false == event_keyword(parameters.front()))
				this->name(reference_kind::memcell, parameters.front(), reference_use::copy_source, name, Event);
		}
		else if (type == "switch" || type == "trackvel" || type == "trackbuckle")
		{
			for (auto const &target : targets)
				this->name(reference_kind::track, target, reference_use::event_target, name, Event);
		}
		// the event a track calls by its name
		if (auto const colon{name.rfind(':')}; colon != std::string::npos && colon > 0 && std::ranges::find(named_sequences, std::string_view{name}.substr(colon + 1)) != named_sequences.end())
			this->name(reference_kind::track, name.substr(0, colon), reference_use::named_event, name, Event);
	}

	void track(statement const &Track)
	{
		auto const name{Track.word(2)};
		for (std::size_t i = 4; i + 1 < Track.words.size(); ++i)
		{
			auto const word{Track.word(i)};
			if (std::ranges::find(named_sequences, word) != named_sequences.end())
				this->name(reference_kind::event, Track.word(++i), reference_use::track_event, name, Track);
		}
	}

	void launcher(statement const &Launcher)
	{
		// <range> <range> <name> eventlauncher <x y z> <radius> <key> <delay> <event1> [<event2>] [condition <cell> <text> <value1> <value2>] ... end
		auto const name{Launcher.word(2)};
		this->name(reference_kind::event, Launcher.word(10), reference_use::launcher, name, Launcher);
		std::size_t next{11};
		auto const second{Launcher.word(next)};
		if (second != "end" && second != "condition" && second != "traintriggered")
		{
			this->name(reference_kind::event, second, reference_use::launcher, name, Launcher);
			++next;
		}
		if (Launcher.word(next) == "condition")
			this->name(reference_kind::memcell, Launcher.word(next + 1), reference_use::launcher_condition, name, Launcher);
	}

  private:
	scenery_text::scenery const &m_scenery;
	std::vector<reference> &m_output;
};

} // namespace

char const *kind_name(reference_kind const Kind)
{
	switch (Kind)
	{
	case reference_kind::event: return "event";
	case reference_kind::memcell: return "memcell";
	default: return "track";
	}
}

char const *use_name(reference_use const Use)
{
	switch (Use)
	{
	case reference_use::trainset: return "trainset";
	case reference_use::vehicle: return "vehicle";
	case reference_use::event_target: return "event target";
	case reference_use::condition: return "condition";
	case reference_use::copy_source: return "copy source";
	case reference_use::event_child: return "multiple event";
	case reference_use::launcher: return "event launcher";
	case reference_use::launcher_condition: return "event launcher condition";
	case reference_use::track_event: return "track event";
	case reference_use::memcell_track: return "memory cell track";
	case reference_use::isolated: return "isolated";
	default: return "named event";
	}
}

scenario_index index_scenarios(std::filesystem::path const &Folder, std::string const &Map)
{
	scenario_index result;
	result.map = Map;
	std::vector<std::string> scenarios;
	std::error_code error;
	for (std::filesystem::directory_iterator file{Folder, std::filesystem::directory_options::skip_permission_denied, error}, end; file != end; file.increment(error))
	{
		if (false == has_extension(file->path(), u8".scn") || false == file->is_regular_file(error))
			continue;
		if (auto name{name_of(file->path())})
			scenarios.push_back(std::move(*name));
	}
	std::ranges::sort(scenarios, [](std::string const &A, std::string const &B) { return scenery_text::lowered(A) < scenery_text::lowered(B); });

	include_graph includes{Folder};
	for (auto const &scenario : scenarios)
	{
		scenery_text::read_options options;
		options.map = Map;
		options.keep = kept;
		// the terrain of a scenario with its binary terrain file comes from that one, its text files aren't read (cParser)
		auto binary{Folder / scenario};
		binary.replace_extension(".sbt");
		options.skip_terrain = std::filesystem::exists(binary, error);
		if (false == includes.may_reach(scenario, Map, options.skip_terrain))
			continue;
		auto const scenery{scenery_text::read(Folder, scenario, options)};
		if (false == scenery.uses_map())
			continue;
		for (auto const &problem : scenery.problems)
			result.problems.push_back(scenario + ": " + problem);

		scenario_use use;
		use.scenario = scenario;
		collector names{scenery, use.references};
		bool trainset{false};
		for (auto const &statement : scenery.statements)
		{
			auto const &keyword{statement.keyword};
			auto const type{keyword == "node" ? statement.word(3) : std::string{}};
			// what the map defines
			if (statement.map)
			{
				if (type == "track")
					result.tracks.insert(statement.word(2));
				else if (type == "memcell")
					result.memcells.insert(statement.word(2));
				else if (keyword == "event")
					result.events.insert(statement.word(0));
			}
			// the trainsets wherever they are, a scenario which is its own map too
			if (keyword == "trainset")
			{
				trainset = true;
				names.name(reference_kind::track, statement.word(1), reference_use::trainset, statement.word(0), statement);
			}
			else if (keyword == "endtrainset")
				trainset = false;
			else if (type == "dynamic" && false == trainset)
				names.name(reference_kind::track, statement.word(7), reference_use::vehicle, statement.word(2), statement);
			if (statement.map)
				continue;
			// what the rest of the scenario names
			if (keyword == "event")
				names.event(statement);
			else if (keyword == "isolated")
			{
				for (std::size_t i = 1; i + 1 < statement.words.size(); ++i)
					names.name(reference_kind::track, statement.word(i), reference_use::isolated, statement.word(0), statement);
			}
			else if (type == "track")
				names.track(statement);
			else if (type == "memcell")
				names.name(reference_kind::track, statement.word(10), reference_use::memcell_track, statement.word(2), statement);
			else if (type == "eventlauncher")
				names.launcher(statement);
		}
		result.scenarios.push_back(std::move(use));
	}

	// of what the scenarios name, what the map defines
	for (auto &use : result.scenarios)
	{
		std::erase_if(use.references, [&](reference const &Reference) {
			auto const &defined{Reference.kind == reference_kind::track ? result.tracks : Reference.kind == reference_kind::event ? result.events : result.memcells};
			return false == defined.contains(Reference.name);
		});
		std::ranges::sort(use.references);
		auto const repeated{std::ranges::unique(use.references)};
		use.references.erase(repeated.begin(), repeated.end());
	}
	return result;
}

} // namespace project
