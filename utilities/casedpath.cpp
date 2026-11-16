/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/casedpath.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <filesystem>
#include <system_error>

namespace
{

bool is_separator(char const Character)
{
	return Character == '/' || Character == '\\';
}

// the length of the root of the path: the separators it starts with, after the drive letter on Windows. the rest is relative
// to the root, and the root itself isn't looked for
std::size_t root_length(std::string_view const Path)
{
	std::size_t length{0};
#ifdef _WIN32
	if (Path.size() >= 2 && Path[1] == ':' && ((Path[0] >= 'A' && Path[0] <= 'Z') || (Path[0] >= 'a' && Path[0] <= 'z')))
		length = 2;
#endif
	while (length < Path.size() && is_separator(Path[length]))
		++length;
	return length;
}

// the root the way the folders are kept: one '/' for the separators, after the drive letter if there's one
std::string root_key(std::string_view const Root)
{
	std::string key;
	auto separators{Root};
	if (Root.size() >= 2 && Root[1] == ':')
	{
		key = Root.substr(0, 2);
		separators = Root.substr(2);
	}
	if (false == separators.empty())
		key += '/';
	return key;
}

// the name in the folder, the folder kept as normal_path() gives it
std::string join(std::string const &Directory, std::string const &Name)
{
	if (Directory.empty() || Directory == ".")
		return Name;
	if (Directory.back() == '/' || Directory.back() == ':')
		return Directory + Name;
	return Directory + '/' + Name;
}

// the folder made of the root and the names in it, "." for the current folder
std::string directory_of(std::string const &Root, std::vector<std::string> const &Names)
{
	auto directory{Root};
	for (auto const &name : Names)
		directory = join(directory, name);
	return directory.empty() ? std::string{"."} : directory;
}

// '..' takes the name before it out; at the start of a relative path it stays, above the root is the root
void step_up(std::vector<std::string> &Names, std::string const &Root)
{
	if (false == Names.empty() && Names.back() != "..")
		Names.pop_back();
	else if (Root.empty())
		Names.emplace_back("..");
}

} // namespace

namespace cased
{

bool list_directory(std::string const &Directory, std::vector<std::string> &Names)
{
	std::error_code error;
	std::filesystem::directory_iterator entry{std::filesystem::path{Directory}, error};
	if (error)
		return false;
	for (std::filesystem::directory_iterator const end; entry != end;)
	{
		try
		{
			// the bytes the system gives where the case matters; on Windows a name outside of the code page throws, and it can't
			// be asked for in the code page either
			Names.emplace_back(entry->path().filename().string());
		}
		catch (std::exception const &)
		{
			// skipped, see above
		}
		entry.increment(error);
		if (error)
			break;
	}
	return true;
}

bool path_exists(std::string const &Path)
{
	std::error_code error;
	return std::filesystem::exists(std::filesystem::path{Path}, error);
}

std::string lower(std::string_view const Name)
{
	std::string lowered{Name};
	for (auto &character : lowered)
		if (character >= 'A' && character <= 'Z')
			character = static_cast<char>(character - 'A' + 'a');
	return lowered;
}

std::string normal_path(std::string_view const Path)
{
	std::string path{Path};
	std::replace(path.begin(), path.end(), '\\', '/');
	auto const rootlength{root_length(path)};
	auto const root{root_key(std::string_view{path}.substr(0, rootlength))};
	std::vector<std::string> names;
	auto position{rootlength};
	while (position < path.size())
	{
		auto end{path.find('/', position)};
		if (end == std::string::npos)
			end = path.size();
		auto name{path.substr(position, end - position)};
		position = end + 1;
		if (name.empty() || name == ".")
			continue;
		if (name == "..")
			step_up(names, root);
		else
			names.emplace_back(std::move(name));
	}
	return directory_of(root, names);
}

index::index(directory_lister Lister, path_probe Probe) : m_lister{std::move(Lister)}, m_probe{std::move(Probe)} {}

std::string index::resolve(std::string_view const Path)
{
	if (Path.empty())
		return {};
	std::string path{Path};
	if (m_probe(path))
		return path;
	return resolve_in_any_case(Path);
}

std::string index::resolve_in_any_case(std::string_view const Path)
{
	if (Path.empty())
		return {};
	std::vector<std::string> listed;
	if (auto const found{walk(Path, listed)}; false == found.empty())
	{
		if (m_probe(found))
			return found;
		// the folders were listed before the file was taken away or renamed: listed again, once
		for (auto const &directory : listed)
			forget(directory);
		listed.clear();
		if (auto const again{walk(Path, listed)}; false == again.empty() && m_probe(again))
			return again;
	}
	return {};
}

std::string index::walk(std::string_view const Path, std::vector<std::string> &Listed)
{
	// the path is given back with the names changed in place: a name in lower case has the same bytes as the name on the
	// disk, but for the case of the ASCII letters
	std::string result{Path};
	std::replace(result.begin(), result.end(), '\\', '/');
	auto const rootlength{root_length(result)};
	auto const root{root_key(std::string_view{result}.substr(0, rootlength))};
	std::vector<std::string> names; // the folders on the way, as on the disk
	auto position{rootlength};
	while (position < result.size())
	{
		auto end{result.find('/', position)};
		if (end == std::string::npos)
			end = result.size();
		auto const name{result.substr(position, end - position)};
		auto const start{position};
		position = end + 1;
		if (name.empty() || name == ".")
			continue;
		if (name == "..")
		{
			step_up(names, root);
			continue;
		}
		auto const directory{directory_of(root, names)};
		auto listing{cached(directory)};
		if (listing == nullptr)
		{
			// a name as it's on the disk needs no listing of its folder, which may be a large one, e.g. textures/
			if (m_probe(join(directory, name)))
			{
				names.emplace_back(name);
				continue;
			}
			listing = list(directory);
		}
		Listed.emplace_back(directory);
		if (false == listing->listed)
			return {};
		auto const found{listing->names.find(lower(name))};
		if (found == listing->names.end())
			return {};
		auto const &candidates{found->second};
		auto const &chosen{std::find(candidates.begin(), candidates.end(), name) != candidates.end() ? name : candidates.front()};
		if (candidates.size() > 1)
			report_collision(directory, candidates, chosen);
		result.replace(start, name.size(), chosen);
		names.emplace_back(chosen);
	}
	return result;
}

std::shared_ptr<index::folder const> index::cached(std::string const &Directory) const
{
	std::shared_lock const lock{m_lock};
	auto const found{m_folders.find(Directory)};
	return found != m_folders.end() ? found->second : nullptr;
}

std::shared_ptr<index::folder const> index::list(std::string const &Directory)
{
	// listed without the lock, the other threads go on meanwhile; when two list the same folder at once, the first one kept
	auto listing{std::make_shared<index::folder>()};
	std::vector<std::string> names;
	listing->listed = m_lister(Directory, names);
	++m_listings;
	for (auto &name : names)
		listing->names[lower(name)].emplace_back(std::move(name));
	for (auto &alike : listing->names)
		if (alike.second.size() > 1)
			std::sort(alike.second.begin(), alike.second.end());
	std::unique_lock const lock{m_lock};
	return m_folders.try_emplace(Directory, std::move(listing)).first->second;
}

void index::forget(std::string_view const Directory)
{
	auto const key{lower(normal_path(Directory))};
	std::unique_lock const lock{m_lock};
	for (auto entry{m_folders.begin()}; entry != m_folders.end();)
	{
		if (lower(entry->first) == key)
			entry = m_folders.erase(entry);
		else
			++entry;
	}
}

void index::forget_all()
{
	std::unique_lock const lock{m_lock};
	m_folders.clear();
}

std::size_t index::listings() const
{
	return m_listings.load();
}

void index::report_collision(std::string const &Directory, std::vector<std::string> const &Names, std::string const &Chosen)
{
	{
		std::scoped_lock const lock{m_reportedlock};
		if (false == m_reported.insert(join(Directory, lower(Chosen))).second)
			return;
	}
	std::string names;
	for (auto const &name : Names)
		names += (names.empty() ? "\"" : ", \"") + name + "\"";
	WriteLog("Files named alike but for the case in \"" + Directory + "\": " + names + "; \"" + Chosen + "\" is used", logtype::file);
}

index &path_index()
{
	static index instance;
	return instance;
}

} // namespace cased

std::string cased_path(std::string_view const Path)
{
#ifdef _WIN32
	std::string path{Path};
	return (false == path.empty() && cased::path_exists(path)) ? path : std::string{};
#else
	return cased::path_index().resolve(Path);
#endif
}

void cased_path_forget(std::string_view const Directory)
{
#ifdef _WIN32
	static_cast<void>(Directory);
#else
	cased::path_index().forget(Directory);
#endif
}

void cased_path_forget_all()
{
#ifndef _WIN32
	cased::path_index().forget_all();
#endif
}
