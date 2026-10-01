/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorSettings.hpp"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>

editorSettings EditorSettings;

namespace
{
namespace fs = std::filesystem;

fs::path settings_path()
{
	fs::path p = user_config_path("eu07_editor.ini");
	return p.empty() ? fs::path("eu07_editor.ini") : p;
}

const char *scheme_to_string(editorSettings::movement_scheme scheme)
{
	return scheme == editorSettings::movement_scheme::legacy ? "legacy" : "wsad";
}

// numbers are stored locale-independent
template <typename Type_> void parse(std::string const &Text, Type_ &Out)
{
	std::istringstream stream(Text);
	stream.imbue(std::locale::classic());
	Type_ value;
	if (stream >> value)
		Out = value;
}

const std::string origin_prefix{"ortho_origin@"};

// settings are whitespace separated key/value pairs, so the scenery name used in the key can't contain spaces
std::string origin_key(std::string const &Scenery)
{
	std::string key = origin_prefix + Scenery;
	for (char &c : key)
		if (std::isspace(static_cast<unsigned char>(c)))
			c = '_';
	return key;
}
}

bool editorSettings::load()
{
	const fs::path path = settings_path();
	std::error_code ec;
	if (!fs::exists(path, ec))
		return false;

	std::ifstream stream(path);
	if (!stream.is_open())
		return false;

	std::string key, value;
	while (stream >> key >> value)
	{
		if (key == "movement_scheme")
			m_movement = (value == "legacy") ? movement_scheme::legacy : movement_scheme::wsad;
		else if (key == "ortho_radius")
			parse(value, m_orthophoto.radius);
		else if (key == "ortho_height")
			parse(value, m_orthophoto.height);
		else if (key == "ortho_opacity")
			parse(value, m_orthophoto.opacity);
		else if (key == "ortho_year")
			parse(value, m_orthophoto.year);
		else if (key == "ortho_4k")
			m_orthophoto.hires = (value == "1");
		else if (key == "ortho_drape")
			m_orthophoto.drape = (value == "1");
		else if (key == "ortho_lift")
			parse(value, m_orthophoto.lift);
		else if (key == "ortho_in_scene")
			m_orthophoto.in_scene = (value == "1");
		else if (key.rfind(origin_prefix, 0) == 0)
		{
			// northing;easting
			auto const separator = value.find(';');
			if (separator == std::string::npos)
				continue;
			double north{0.0}, east{0.0};
			parse(value.substr(0, separator), north);
			parse(value.substr(separator + 1), east);
			m_orthophoto_origins[key] = {north, east};
		}
	}
	return true;
}

bool editorSettings::save()
{
	const fs::path path = settings_path();
	std::error_code ec;
	fs::create_directories(path.parent_path(), ec);

	std::ofstream stream(path, std::ios::trunc);
	if (!stream.is_open())
	{
		ErrorLog("failed to save editor settings");
		return false;
	}

	stream.imbue(std::locale::classic());
	stream << "movement_scheme " << scheme_to_string(m_movement) << "\n";
	stream << "ortho_radius " << m_orthophoto.radius << "\n";
	stream << "ortho_height " << m_orthophoto.height << "\n";
	stream << "ortho_opacity " << m_orthophoto.opacity << "\n";
	stream << "ortho_year " << m_orthophoto.year << "\n";
	stream << "ortho_4k " << (m_orthophoto.hires ? 1 : 0) << "\n";
	stream << "ortho_drape " << (m_orthophoto.drape ? 1 : 0) << "\n";
	stream << "ortho_lift " << m_orthophoto.lift << "\n";
	stream << "ortho_in_scene " << (m_orthophoto.in_scene ? 1 : 0) << "\n";
	stream << std::fixed << std::setprecision(3);
	for (auto const &origin : m_orthophoto_origins)
		stream << origin.first << " " << origin.second.first << ";" << origin.second.second << "\n";
	return true;
}

bool editorSettings::orthophoto_origin(std::string const &Scenery, double &North, double &East) const
{
	auto const lookup = m_orthophoto_origins.find(origin_key(Scenery));
	if (lookup == m_orthophoto_origins.end())
		return false;
	North = lookup->second.first;
	East = lookup->second.second;
	return true;
}

void editorSettings::orthophoto_origin(std::string const &Scenery, double North, double East)
{
	m_orthophoto_origins[origin_key(Scenery)] = {North, East};
}
