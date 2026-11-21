/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "environment/georeference.h"

#include "editor/editorGeoMap.hpp"
#include "editor/editorSettings.hpp"
#include "utilities/Globals.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>
#include <vector>

namespace
{

std::string lowered(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return Text;
}

// a file of the scenery folder by its name, whatever the case of its letters
std::filesystem::path scenery_file(std::string const &Name)
{
	std::filesystem::path path{std::filesystem::path{Global.asCurrentSceneryPath} / Name};
	std::error_code error;
	if (std::filesystem::exists(path, error))
		return path;
	auto const folder{path.parent_path()};
	auto const wanted{lowered(path.filename().string())};
	for (std::filesystem::directory_iterator entry{folder, error}, end; entry != end; entry.increment(error))
	{
		try
		{
			if (lowered(entry->path().filename().string()) == wanted)
				return entry->path();
		}
		catch (std::exception const &)
		{
		}
	}
	return path;
}

// the //$g line of the scenery file, or of a file it includes near its top
std::optional<glm::dvec2> origin_of_files()
{
	std::vector<std::filesystem::path> files{scenery_file(Global.SceneryFile)};
	std::string line;
	for (std::size_t index = 0; index < files.size(); ++index)
	{
		std::ifstream stream(files[index]);
		for (int count = 0; (count < 200) && std::getline(stream, line); ++count)
		{
			if (auto const origin{georeference::parse_line(line)})
				return origin;
			// include <file> ... end, from the scenery file only
			auto const start{line.find_first_not_of(" \t")};
			if ((index == 0) && (files.size() < 8) && (start != std::string::npos) && (lowered(line.substr(start, 8)) == "include "))
			{
				std::istringstream words(line.substr(start + 8));
				std::string name;
				if ((words >> name) && (name.find("//") != 0))
				{
					std::replace(name.begin(), name.end(), '\\', '/');
					files.emplace_back(scenery_file(name));
				}
			}
		}
	}
	return std::nullopt;
}

} // namespace

std::optional<glm::dvec2> georeference::parse_line(std::string const &Line)
{
	auto const mark{Line.find("//$g")};
	if (mark == std::string::npos)
		return std::nullopt;
	std::istringstream words(Line.substr(mark + 4));
	std::string system;
	double east{0.0};
	double north{0.0};
	if ((false == static_cast<bool>(words >> system >> east >> north)) || (system.find("1992") == std::string::npos))
		return std::nullopt;
	auto const scale{((east < 10000.0) && (north < 10000.0)) ? 1000.0 : 1.0};
	return glm::dvec2{north * scale, east * scale};
}

std::optional<glm::dvec2> georeference::scenery_origin()
{
	static std::string scenery;
	static std::optional<glm::dvec2> origin;
	if (scenery == Global.SceneryFile)
		return origin;
	scenery = Global.SceneryFile;
	origin.reset();
	// the place the editor keeps for the scenery wins, as it does in the editor
	auto name{scenery};
	if (auto const slash = name.find_last_of("/\\"); slash != std::string::npos)
		name = name.substr(slash + 1);
	if (auto const dot = name.find_last_of('.'); dot != std::string::npos)
		name = name.substr(0, dot);
	double north{0.0};
	double east{0.0};
	if ((false == name.empty()) && std::as_const(EditorSettings).orthophoto_origin(name, north, east))
		origin = glm::dvec2{north, east};
	else
		origin = origin_of_files();
	// only a place in Poland: the PUWG 1992 doesn't reach further
	if (origin && (false == editor_geo::in_poland(glm::dvec2{origin->y, origin->x})))
		origin.reset();
	return origin;
}

glm::dvec2 georeference::to_degrees(glm::dvec2 const &Origin, glm::dvec3 const &World)
{
	auto const east{Origin.y - World.x};
	auto const north{Origin.x + World.z};
	return editor_geo::to_degrees(glm::dvec2{east, north});
}
