/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/project/scenarioIndexCache.hpp"
#include "editor/project/projectFile.hpp"
#include "editor/project/sceneryText.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iterator>
#include <sstream>
#include <thread>

namespace project
{

namespace
{

constexpr int kIndexVersion{1};

template <typename Enum, std::size_t Count> std::optional<Enum> named(std::string const &Name, char const *(*Namer)(Enum), Enum const (&All)[Count])
{
	for (auto const each : All)
		if (Name == Namer(each))
			return each;
	return std::nullopt;
}

reference_kind const all_kinds[]{reference_kind::track, reference_kind::event, reference_kind::memcell};
reference_use const all_uses[]{reference_use::trainset,       reference_use::vehicle,     reference_use::event_target, reference_use::condition,
                               reference_use::copy_source,    reference_use::event_child, reference_use::launcher,     reference_use::launcher_condition,
                               reference_use::track_event,    reference_use::memcell_track, reference_use::isolated,   reference_use::named_event};

} // namespace

index_fingerprint fingerprint_of(std::filesystem::path const &Folder, std::string const &Map)
{
	index_fingerprint result;
	result.map = Map;
	std::error_code error;
	for (std::filesystem::directory_iterator file{Folder, std::filesystem::directory_options::skip_permission_denied, error}, end; file != end; file.increment(error))
	{
		// the extension looked at before the name is made a string, which throws on Windows outside of the code page
		if (false == scenery_text::has_extension(file->path(), u8".scn") || false == file->is_regular_file(error))
			continue;
		auto name{scenery_text::name_of(file->path())};
		if (false == name.has_value())
			continue;
		index_fingerprint::file entry;
		entry.name = std::move(*name);
		entry.size = file->file_size(error);
		entry.time = std::chrono::duration_cast<std::chrono::seconds>(file->last_write_time(error).time_since_epoch()).count();
		result.files.push_back(std::move(entry));
	}
	std::ranges::sort(result.files, [](index_fingerprint::file const &A, index_fingerprint::file const &B) { return A.name < B.name; });
	return result;
}

std::string write_index(scenario_index const &Index, index_fingerprint const &Fingerprint)
{
	std::ostringstream text;
	text << "eu07index " << kIndexVersion << '\n';
	text << "map " << word(Index.map) << '\n';
	for (auto const &file : Fingerprint.files)
		text << "file " << word(file.name) << ' ' << file.size << ' ' << file.time << '\n';
	for (auto const &name : Index.tracks)
		text << "track " << word(name) << '\n';
	for (auto const &name : Index.events)
		text << "event " << word(name) << '\n';
	for (auto const &name : Index.memcells)
		text << "memcell " << word(name) << '\n';
	for (auto const &scenario : Index.scenarios)
	{
		text << "scenario " << word(scenario.scenario) << '\n';
		for (auto const &reference : scenario.references)
			text << "ref " << kind_name(reference.kind) << ' ' << word(reference.name) << ' ' << word(use_name(reference.use)) << ' ' << word(reference.from) << ' ' << word(reference.file) << ' '
			     << reference.line << '\n';
	}
	for (auto const &problem : Index.problems)
		text << "problem " << word(problem) << '\n';
	text << "end\n";
	return text.str();
}

std::optional<stored_index> read_index(std::string_view const Text)
{
	stored_index result;
	std::istringstream lines{std::string{Text}};
	std::string line;
	if (false == static_cast<bool>(std::getline(lines, line)))
		return std::nullopt;
	{
		std::istringstream header{line};
		std::string magic;
		int version{0};
		if (false == static_cast<bool>(header >> magic >> version) || magic != "eu07index" || version != kIndexVersion)
			return std::nullopt;
	}
	bool ended{false};
	while (std::getline(lines, line))
	{
		if (false == line.empty() && line.back() == '\r')
			line.pop_back();
		std::istringstream words{line};
		std::string keyword;
		if (false == static_cast<bool>(words >> keyword))
			continue;
		std::string name;
		if (keyword == "end")
		{
			ended = true;
			break;
		}
		if (keyword == "map" && words >> name)
		{
			result.index.map = unword(name);
			result.fingerprint.map = result.index.map;
		}
		else if (keyword == "file")
		{
			index_fingerprint::file entry;
			if (false == static_cast<bool>(words >> name >> entry.size >> entry.time))
				return std::nullopt;
			entry.name = unword(name);
			result.fingerprint.files.push_back(std::move(entry));
		}
		else if ((keyword == "track" || keyword == "event" || keyword == "memcell") && words >> name)
			(keyword == "track" ? result.index.tracks : keyword == "event" ? result.index.events : result.index.memcells).insert(unword(name));
		else if (keyword == "scenario" && words >> name)
			result.index.scenarios.push_back({unword(name), {}});
		else if (keyword == "ref")
		{
			std::string kind, use, from, file;
			reference entry;
			if (result.index.scenarios.empty() || false == static_cast<bool>(words >> kind >> name >> use >> from >> file >> entry.line))
				return std::nullopt;
			auto const kindof{named(kind, kind_name, all_kinds)};
			auto const useof{named(unword(use), use_name, all_uses)};
			if (false == kindof.has_value() || false == useof.has_value())
				return std::nullopt;
			entry.kind = *kindof;
			entry.use = *useof;
			entry.name = unword(name);
			entry.from = unword(from);
			entry.file = unword(file);
			result.index.scenarios.back().references.push_back(std::move(entry));
		}
		else if (keyword == "problem" && words >> name)
			result.index.problems.push_back(unword(name));
		else
			return std::nullopt;
	}
	if (false == ended)
		return std::nullopt;
	return result;
}

std::vector<scenario_use> users_of(scenario_index const &Index, std::vector<std::string> const &Tracks)
{
	std::vector<std::string> names;
	for (auto const &track : Tracks)
		names.push_back(scenery_text::lowered(track));
	std::vector<scenario_use> result;
	for (auto const &scenario : Index.scenarios)
	{
		scenario_use use{scenario.scenario, {}};
		for (auto const &entry : scenario.references)
			if (entry.kind == reference_kind::track && std::ranges::find(names, entry.name) != names.end())
				use.references.push_back(entry);
		if (false == use.references.empty())
			result.push_back(std::move(use));
	}
	return result;
}

index_cache::index_cache(builder Build, fingerprinter Fingerprint) : m_build{std::move(Build)}, m_fingerprint{std::move(Fingerprint)} {}

void index_cache::request(std::filesystem::path const &Folder, std::string const &Map, std::filesystem::path const &Store)
{
	if (m_building != nullptr)
	{
		if (Folder == m_folder && Map == m_map)
			return;
		// of another map: the one being built is let finish and thrown away
		m_building.reset();
	}
	auto const fingerprint{m_fingerprint(Folder, Map)};
	if (m_index != nullptr && Folder == m_folder && Map == m_map && fingerprint == m_index_fingerprint)
		return;
	m_folder = Folder;
	m_map = Map;
	m_store = Store;
	m_index.reset();
	if (false == Store.empty())
	{
		std::ifstream file{Store, std::ios::binary};
		if (file.is_open())
		{
			std::string const text{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
			if (auto stored{read_index(text)}; stored.has_value() && stored->fingerprint == fingerprint && stored->index.map == Map)
			{
				m_index = std::make_shared<scenario_index const>(std::move(stored->index));
				m_index_fingerprint = fingerprint;
				return;
			}
		}
	}
	m_building = std::make_shared<slot>();
	std::thread{[into = m_building, build = m_build, Folder, Map, fingerprint]() {
		built made{build(Folder, Map), fingerprint};
		std::scoped_lock const hold{into->lock};
		into->made = std::move(made);
	}}.detach();
}

std::shared_ptr<scenario_index const> index_cache::ready()
{
	std::optional<built> finished;
	if (m_building != nullptr)
	{
		std::scoped_lock const hold{m_building->lock};
		finished = std::move(m_building->made);
	}
	if (finished.has_value())
	{
		m_building.reset();
		auto &made{*finished};
		m_index_fingerprint = made.fingerprint;
		m_index = std::make_shared<scenario_index const>(std::move(made.index));
		if (false == m_store.empty())
		{
			std::error_code error;
			std::filesystem::create_directories(m_store.parent_path(), error);
			auto temporary{m_store};
			temporary += ".new";
			{
				std::ofstream file{temporary, std::ios::binary | std::ios::trunc};
				auto const text{write_index(*m_index, m_index_fingerprint)};
				file.write(text.data(), static_cast<std::streamsize>(text.size()));
			}
			std::filesystem::rename(temporary, m_store, error);
		}
	}
	if (m_building != nullptr)
		return nullptr;
	return m_index;
}

void index_cache::forget()
{
	m_building.reset();
	m_index.reset();
	m_index_fingerprint = {};
}

} // namespace project
