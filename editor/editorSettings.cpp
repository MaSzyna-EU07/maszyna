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
#include <cstdio>
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
const std::string offset_prefix{"nmt_offset@"};

// settings are whitespace separated key/value pairs, so the scenery name used in the key can't contain spaces
std::string scenery_key(std::string const &Prefix, std::string const &Scenery)
{
	std::string key = Prefix + Scenery;
	for (char &c : key)
		if (std::isspace(static_cast<unsigned char>(c)))
			c = '_';
	return key;
}

// a field of a WMS layer in the settings, which keep values with no blank in them: the blanks, the separator of the fields and the
// percent sign go as %XX
std::string encoded(std::string const &Text)
{
	std::string out;
	for (unsigned char const c : Text)
	{
		if (std::isspace(c) || c == ';' || c == '%')
		{
			char hex[4];
			std::snprintf(hex, sizeof(hex), "%%%02X", c);
			out += hex;
		}
		else
			out += static_cast<char>(c);
	}
	return out;
}

std::string decoded(std::string const &Text)
{
	std::string out;
	for (std::size_t i = 0; i < Text.size(); ++i)
	{
		if (Text[i] == '%' && i + 2 < Text.size() && std::isxdigit(static_cast<unsigned char>(Text[i + 1])) && std::isxdigit(static_cast<unsigned char>(Text[i + 2])))
		{
			out += static_cast<char>(std::strtoul(Text.substr(i + 1, 2).c_str(), nullptr, 16));
			i += 2;
		}
		else
			out += Text[i];
	}
	return out;
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
		else if (key == "ortho_on")
			m_orthophoto.on = (value == "1");
		else if (key == "platform_edge")
			parse(value, m_platform_edge);
		else if (key == "signal_offset")
			parse(value, m_signal_offset);
		else if (key == "nodebank_previews")
			m_nodebank_previews = (value == "1");
		else if (key == "nodebank_card")
			parse(value, m_nodebank_card);
		else if (key == "colours")
			parse(value, m_palette);
		else if (key == "first_steps")
			m_first_steps = (value == "1");
		else if (key == "terrain_offer")
			m_terrain_offer = (value == "1");
		else if (key == "track_tools_all")
			m_track_tools_more = (value == "1");
		else if (key == "terrain_while_laying")
			m_terrain_while_laying = (value == "1");
		else if (key == "nmt_band")
			parse(value, m_terrain_model.band);
		else if (key == "nmt_around")
			parse(value, m_terrain_model.around);
		else if (key == "nmt_shape")
			m_terrain_model.shape = (value == "1");
		else if (key == "nmt_follow")
			m_terrain_model.follow = (value == "1");
		else if (key == "top_ballast")
			m_top_tracks = value == "1" ? 0 : 1;
		else if (key == "top_tracks")
			m_top_tracks = std::clamp(std::atoi(value.c_str()), 0, 2);
		else if (key == "top_rails")
			m_top_rails = static_cast<std::uint32_t>(std::strtoul(value.c_str(), nullptr, 16)) | 0xff000000u;
		else if (key == "corner_radii")
		{
			// low;high
			double low{0.0}, high{0.0};
			if (auto const separator{value.find(';')}; separator != std::string::npos)
			{
				parse(value.substr(0, separator), low);
				parse(value.substr(separator + 1), high);
				if (low >= 10.0 && high >= low)
					corners(low, high);
			}
		}
		// under a key of its own since the laying goes from point to point unless told otherwise: the fitted heights chosen
		// before, when they were the default, are left behind
		else if (key == "laying_heights")
			m_heights = value == "points" ? lay_heights::points : value == "level" ? lay_heights::level : lay_heights::fitted;
		// visible;opacity;address;layers;title;terms
		else if (key == "wms_layer")
		{
			std::vector<std::string> fields;
			for (std::size_t from{0};;)
			{
				auto const separator{value.find(';', from)};
				fields.push_back(decoded(value.substr(from, separator == std::string::npos ? std::string::npos : separator - from)));
				if (separator == std::string::npos)
					break;
				from = separator + 1;
			}
			if (fields.size() < 4 || fields[2].empty() || fields[3].empty())
				continue;
			wms_layer layer;
			layer.visible = fields[0] == "1";
			parse(fields[1], layer.opacity);
			layer.url = fields[2];
			layer.layers = fields[3];
			if (fields.size() > 4)
				layer.title = fields[4];
			if (fields.size() > 5)
				layer.terms = fields[5];
			m_wms_layers.push_back(layer);
		}
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
		else if (key.rfind(offset_prefix, 0) == 0)
		{
			// offset;fitted
			auto const separator = value.find(';');
			if (separator == std::string::npos)
				continue;
			double offset{0.0};
			parse(value.substr(0, separator), offset);
			m_terrain_model_offsets[key] = {offset, value.substr(separator + 1) == "1"};
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
	if (m_orthophoto.on.has_value())
		stream << "ortho_on " << (*m_orthophoto.on ? 1 : 0) << "\n";
	stream << "platform_edge " << m_platform_edge << "\n";
	stream << "signal_offset " << m_signal_offset << "\n";
	stream << "nodebank_previews " << (m_nodebank_previews ? 1 : 0) << "\n";
	stream << "nodebank_card " << m_nodebank_card << "\n";
	stream << "colours " << m_palette << "\n";
	stream << "first_steps " << (m_first_steps ? 1 : 0) << "\n";
	stream << "terrain_offer " << (m_terrain_offer ? 1 : 0) << "\n";
	stream << "track_tools_all " << (m_track_tools_more ? 1 : 0) << "\n";
	stream << "terrain_while_laying " << (m_terrain_while_laying ? 1 : 0) << "\n";
	stream << "nmt_band " << m_terrain_model.band << "\n";
	stream << "nmt_around " << m_terrain_model.around << "\n";
	stream << "nmt_shape " << (m_terrain_model.shape ? 1 : 0) << "\n";
	stream << "nmt_follow " << (m_terrain_model.follow ? 1 : 0) << "\n";
	stream << "top_tracks " << m_top_tracks << "\n";
	stream << "top_rails " << std::hex << (m_top_rails & 0xffffffu) << std::dec << "\n";
	stream << "corner_radii " << m_corner_low << ";" << m_corner_high << "\n";
	stream << "laying_heights " << (m_heights == lay_heights::points ? "points" : m_heights == lay_heights::level ? "level" : "fitted") << "\n";
	for (auto const &layer : m_wms_layers)
		stream << "wms_layer " << (layer.visible ? 1 : 0) << ";" << layer.opacity << ";" << encoded(layer.url) << ";" << encoded(layer.layers) << ";" << encoded(layer.title) << ";" << encoded(layer.terms) << "\n";
	stream << std::fixed << std::setprecision(3);
	for (auto const &origin : m_orthophoto_origins)
		stream << origin.first << " " << origin.second.first << ";" << origin.second.second << "\n";
	for (auto const &offset : m_terrain_model_offsets)
		stream << offset.first << " " << offset.second.first << ";" << (offset.second.second ? 1 : 0) << "\n";
	return true;
}

bool editorSettings::orthophoto_origin(std::string const &Scenery, double &North, double &East) const
{
	auto const lookup = m_orthophoto_origins.find(scenery_key(origin_prefix, Scenery));
	if (lookup == m_orthophoto_origins.end())
		return false;
	North = lookup->second.first;
	East = lookup->second.second;
	return true;
}

void editorSettings::orthophoto_origin(std::string const &Scenery, double North, double East)
{
	m_orthophoto_origins[scenery_key(origin_prefix, Scenery)] = {North, East};
}

bool editorSettings::terrain_model_offset(std::string const &Scenery, double &Offset, bool &Fitted) const
{
	auto const lookup = m_terrain_model_offsets.find(scenery_key(offset_prefix, Scenery));
	if (lookup == m_terrain_model_offsets.end())
		return false;
	Offset = lookup->second.first;
	Fitted = lookup->second.second;
	return true;
}

void editorSettings::terrain_model_offset(std::string const &Scenery, double Offset, bool Fitted)
{
	// the model as it is, never fitted, is what a scenery without the line has
	if (Offset == 0.0 && false == Fitted)
		m_terrain_model_offsets.erase(scenery_key(offset_prefix, Scenery));
	else
		m_terrain_model_offsets[scenery_key(offset_prefix, Scenery)] = {Offset, Fitted};
}
