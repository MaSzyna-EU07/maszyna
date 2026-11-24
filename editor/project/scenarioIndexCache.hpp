/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/project/scenarioIndex.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <mutex>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// The scenario index of the map open in the editor, built in the background and kept (editor-tracks-domain.md §0 pkt 7, §8 "Usuń linię":
// the editor asks it before a track a scenario names is taken away or renamed). On a big map it takes minutes (E3: 80 to 210 s), so it's
// built once, as the map is opened, and kept: in memory, and in a file between the sessions, for as long as the scenarios are the files
// they were (the *.scn files of the folder, by their names, sizes and times of change; a change of a file one of them includes alone
// isn't seen, the map's own files neither: the map defines what the editor changes).
namespace project
{

// the scenarios of the folder: each *.scn file, its size and the time of its last change
struct index_fingerprint
{
	struct file
	{
		std::string name;
		std::uintmax_t size{0};
		std::int64_t time{0};

		bool operator==(file const &) const = default;
	};
	std::string map;
	std::vector<file> files; // in the order of their names

	bool operator==(index_fingerprint const &) const = default;
};
index_fingerprint fingerprint_of(std::filesystem::path const &Folder, std::string const &Map);

// the index written as text with its fingerprint, and read back; none from a text which isn't one, or of another version
std::string write_index(scenario_index const &Index, index_fingerprint const &Fingerprint);
struct stored_index
{
	scenario_index index;
	index_fingerprint fingerprint;
};
std::optional<stored_index> read_index(std::string_view const Text);

// the scenarios naming any of the tracks, with only the references to them, in the order of the index; the names as the simulator
// compares them, A to Z small
std::vector<scenario_use> users_of(scenario_index const &Index, std::vector<std::string> const &Tracks);

class index_cache
{
  public:
	using builder = std::function<scenario_index(std::filesystem::path const &, std::string const &)>;
	using fingerprinter = std::function<index_fingerprint(std::filesystem::path const &, std::string const &)>;
	explicit index_cache(builder Build = index_scenarios, fingerprinter Fingerprint = fingerprint_of);
	// an index being built is let finish on its own and thrown away: the editor doesn't wait minutes for it to close
	~index_cache() = default;
	index_cache(index_cache const &) = delete;
	index_cache &operator=(index_cache const &) = delete;

	// the index of the map is to be had: kept already, for the scenarios as they are now, read from the file kept between the sessions
	// (Store, none for none), or built in the background from now on
	void request(std::filesystem::path const &Folder, std::string const &Map, std::filesystem::path const &Store = {});
	// the index of the map asked for last, once it's built; the one built is written into the file kept between the sessions
	std::shared_ptr<scenario_index const> ready();
	bool building() const { return m_building != nullptr; }
	// the index taken for one of other scenarios: built again at the next request
	void forget();

  private:
	struct built
	{
		scenario_index index;
		index_fingerprint fingerprint;
	};
	// what the thread building the index hands over, kept by both while either needs it
	struct slot
	{
		std::mutex lock;
		std::optional<built> made;
	};
	builder m_build;
	fingerprinter m_fingerprint;
	std::filesystem::path m_folder;
	std::string m_map;
	std::filesystem::path m_store;
	std::shared_ptr<scenario_index const> m_index;
	index_fingerprint m_index_fingerprint;
	std::shared_ptr<slot> m_building;
};

} // namespace project
