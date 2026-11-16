/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <atomic>
#include <concepts>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Files named in another case than they have on the disk.
//
// The game asks for nearly every file in lower case: the parser lowercases the names it reads, and so do the managers of
// models, textures and sounds. The data is made on Windows, where a name is found in any case, and some of its files and
// folders have upper case letters (the timetables of galicja2000, the folder scenery/Braniewo). Where the file system
// tells the case apart (Linux), such a path is found here part by part, each folder on the way listed once and kept:
// its names in lower case, to the names on the disk. A path named as it is on the disk costs one check, as it did.
//
// The names are folded in ASCII only, as ToLower() does: the bytes of UTF-8 (or cp1250) letters are compared as they are.
// Two names in one folder which differ in case only (which Windows can't hold, so the data hardly has them) give the
// one named exactly as asked, otherwise the first of them in the order of their bytes; the log tells of it once.

namespace cased
{

// puts the names in the folder (names, not paths) to Names; false when the folder can't be listed (not there, not a folder)
using directory_lister = std::function<bool(std::string const &Directory, std::vector<std::string> &Names)>;
// whether there's a file or a folder named exactly so
using path_probe = std::function<bool(std::string const &Path)>;

// the lister and the probe of the file system itself
bool list_directory(std::string const &Directory, std::vector<std::string> &Names);
bool path_exists(std::string const &Path);

// ASCII letters lowered, the other bytes as they are
std::string lower(std::string_view Name);

// the path the way the folders are kept: '/' for '\', no '.' nor empty parts, a name followed by '..' taken out, no '/' at
// the end; "." for the current folder. the root ("/", "C:/") stays as given. names aren't checked on the disk, so
// 'link/..' is the folder the link is in, not the one above where it points to
std::string normal_path(std::string_view Path);

// the names of the folders, listed when they are needed, on a file system given by the lister and the probe.
// used from many threads at once
class index
{
  public:
	explicit index(directory_lister Lister = list_directory, path_probe Probe = path_exists);

	// the path as it is on the disk: the path given, if it's there; otherwise the path given with its names in another case
	// changed to the names on the disk (so of the same length, with '/' for '\'); empty when it isn't there in any case.
	// a name not found in a folder listed already isn't looked for again: a file added later is found after forget()
	std::string resolve(std::string_view Path);
	// as resolve(), when the path as given is known not to be there
	std::string resolve_in_any_case(std::string_view Path);
	// the path to open the file with: as resolve(), or the path given when it isn't there in any case, for the opening to
	// fail on it as it did
	std::string path_to_open(std::string_view Path);

	// the content of the folder changed (a file written, removed, renamed): it's listed again when needed. the folder is
	// named in any case, any form normal_path() makes the same
	void forget(std::string_view Directory);
	void forget_all();

	// how many times a folder was listed, for the tests
	std::size_t listings() const;

  private:
	struct folder
	{
		bool listed{false};
		// the names in lower case, to the names on the disk, in the order of their bytes
		std::unordered_map<std::string, std::vector<std::string>> names;
	};

	std::shared_ptr<folder const> cached(std::string const &Directory) const;
	std::shared_ptr<folder const> list(std::string const &Directory);
	std::string walk(std::string_view Path, std::vector<std::string> &Listed);
	void report_collision(std::string const &Directory, std::vector<std::string> const &Names, std::string const &Chosen);

	directory_lister m_lister;
	path_probe m_probe;
	mutable std::shared_mutex m_lock;
	std::unordered_map<std::string, std::shared_ptr<folder const>> m_folders;
	std::mutex m_reportedlock;
	std::unordered_set<std::string> m_reported;
	std::atomic<std::size_t> m_listings{0};
};

// the index of the files of the game, which cased_path() looks in where the case matters
index &path_index();

// the first name with the first extension which is there, as Exists tells: each name with all the extensions, before the
// next name. the name and the extension are given back as they were asked for, not as they are on the disk, being the
// keys the managers of textures, materials, models and sounds keep the files under; a pair of empty texts if none is there
std::pair<std::string, std::string> first_existing(std::vector<std::string> const &Names, std::vector<std::string> const &Extensions,
                                                   path_probe const &Exists);

} // namespace cased

// the file or the folder as it is on the disk: the path given if it's there, the path with its names on the disk if it's
// there in another case, empty if it isn't there at all. on Windows the system finds a name in any case itself, so there
// it's only the path given if it's there, or empty
std::string cased_path(std::string_view Path);
// the path to open a file of the game with, which FileExists() found under the name the game asked for: the path as it is
// on the disk where it's there in another case, otherwise the path given. on Windows the path given, with no look at the disk
//
// a file of the game data (scenery, vehicles, models, textures, sounds, terrain, scripts, lang, shaders...) is opened through
// disk_path() at the moment it's opened: std::ifstream, fopen, stbi_load, sf_open, luaL_dofile and the like get disk_path(name),
// not the name itself. the name the game asked for stays what it keeps the file under and what it writes to the log. files the
// game writes itself and reads back under its own name (the settings, the odometers, the logs) need it not; the cParser
// reading a file finds it by itself (FileCased()), and so does FileExists()
std::string disk_path(std::string_view Path);
// as disk_path(), for a path held as std::filesystem::path; on Windows the path given as it is, not converted to text and back
template <std::same_as<std::filesystem::path> Path> Path disk_path(Path const &File)
{
#ifdef _WIN32
	return File;
#else
	return Path{disk_path(std::string_view{File.native()})};
#endif
}
// the content of the folder changed, see cased::index::forget(); nothing to do on Windows
void cased_path_forget(std::string_view Directory);
void cased_path_forget_all();
