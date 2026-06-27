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

#include <cstdlib>
#include <filesystem>
#include <fstream>

editorSettings EditorSettings;

namespace
{
namespace fs = std::filesystem;

fs::path settings_path()
{
#ifdef _WIN32
	if (const char *appdata = std::getenv("APPDATA"))
		return fs::path(appdata) / "MaSzyna" / "eu07_editor.ini";
#else
	if (const char *home = std::getenv("HOME"))
		return fs::path(home) / ".config" / "MaSzyna" / "eu07_editor.ini";
#endif
	return fs::path("eu07_editor.ini");
}

const char *scheme_to_string(editorSettings::movement_scheme scheme)
{
	return scheme == editorSettings::movement_scheme::legacy ? "legacy" : "wsad";
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

	stream << "movement_scheme " << scheme_to_string(m_movement) << "\n";
	return true;
}
