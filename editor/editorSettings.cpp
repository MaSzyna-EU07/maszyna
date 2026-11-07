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
#include <format>
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
			out += std::format("%{:02X}", c);
		else
			out += static_cast<char>(c);
	}
	return out;
}

std::string decoded(std::string const &Text)
{
	std::string out;
	std::size_t i{0};
	while (i < Text.size())
	{
		if (Text[i] == '%' && i + 2 < Text.size() && std::isxdigit(static_cast<unsigned char>(Text[i + 1])) && std::isxdigit(static_cast<unsigned char>(Text[i + 2])))
		{
			out += static_cast<char>(std::strtoul(Text.substr(i + 1, 2).c_str(), nullptr, 16));
			i += 3;
		}
		else
		{
			out += Text[i];
			++i;
		}
	}
	return out;
}

// the fields of a WMS layer in the settings, apart at the separators and decoded
std::vector<std::string> wms_fields(std::string const &Value)
{
	std::vector<std::string> fields;
	for (std::size_t from{0};;)
	{
		auto const separator{Value.find(';', from)};
		fields.push_back(decoded(Value.substr(from, separator == std::string::npos ? std::string::npos : separator - from)));
		if (separator == std::string::npos)
			break;
		from = separator + 1;
	}
	return fields;
}

// visible;opacity;address;layers;title;terms; none when the address or the layers are missing
std::optional<editorSettings::wms_layer> wms_layer_read(std::string const &Value)
{
	auto const fields{wms_fields(Value)};
	if (fields.size() < 4 || fields[2].empty() || fields[3].empty())
		return std::nullopt;
	editorSettings::wms_layer layer;
	layer.visible = fields[0] == "1";
	parse(fields[1], layer.opacity);
	layer.url = fields[2];
	layer.layers = fields[3];
	if (fields.size() > 4)
		layer.title = fields[4];
	if (fields.size() > 5)
		layer.terms = fields[5];
	return layer;
}

editorSettings::lay_heights heights_read(std::string const &Value)
{
	if (Value == "points")
		return editorSettings::lay_heights::points;
	if (Value == "level")
		return editorSettings::lay_heights::level;
	return editorSettings::lay_heights::fitted;
}

char const *heights_name(editorSettings::lay_heights const Heights)
{
	if (Heights == editorSettings::lay_heights::points)
		return "points";
	if (Heights == editorSettings::lay_heights::level)
		return "level";
	return "fitted";
}

// a line of the settings for the orthophoto read into it: false when the key is none of its
bool load_orthophoto(std::string const &Key, std::string const &Value, editorSettings::orthophoto_settings &Out)
{
	if (Key == "ortho_radius")
		parse(Value, Out.radius);
	else if (Key == "ortho_height")
		parse(Value, Out.height);
	else if (Key == "ortho_opacity")
		parse(Value, Out.opacity);
	else if (Key == "ortho_year")
		parse(Value, Out.year);
	else if (Key == "ortho_4k")
		Out.hires = (Value == "1");
	else if (Key == "ortho_drape")
		Out.drape = (Value == "1");
	else if (Key == "ortho_lift")
		parse(Value, Out.lift);
	else if (Key == "ortho_in_scene")
		Out.in_scene = (Value == "1");
	else if (Key == "ortho_on")
		Out.on = (Value == "1");
	else
		return false;
	return true;
}

// a line of the settings for the terrain model read into them: false when the key is none of theirs
bool load_terrain_model(std::string const &Key, std::string const &Value, editorSettings::terrain_model_settings &Out)
{
	if (Key == "nmt_band")
		parse(Value, Out.band);
	else if (Key == "nmt_around")
		parse(Value, Out.around);
	else if (Key == "nmt_shape")
		Out.shape = (Value == "1");
	else if (Key == "nmt_follow")
		Out.follow = (Value == "1");
	else
		return false;
	return true;
}
}

bool editorSettings::load_editor(std::string const &Key, std::string const &Value)
{
	if (Key == "movement_scheme")
		m_movement = (Value == "legacy") ? movement_scheme::legacy : movement_scheme::wsad;
	else if (Key == "platform_edge")
		parse(Value, m_platform_edge);
	else if (Key == "signal_offset")
		parse(Value, m_signal_offset);
	else if (Key == "nodebank_previews")
		m_nodebank_previews = (Value == "1");
	else if (Key == "nodebank_card")
		parse(Value, m_nodebank_card);
	else if (Key == "colours")
		parse(Value, m_palette);
	else if (Key == "first_steps")
		m_first_steps = (Value == "1");
	else if (Key == "terrain_offer")
		m_terrain_offer = (Value == "1");
	else if (Key == "terrain_buried_ask")
		m_terrain_buried_ask = (Value == "1");
	else if (Key == "track_tools_all")
		m_track_tools_more = (Value == "1");
	else if (Key == "terrain_while_laying")
		m_terrain_while_laying = (Value == "1");
	else if (Key == "top_ballast")
		m_top_tracks = Value == "1" ? 0 : 1;
	else if (Key == "top_tracks")
		m_top_tracks = std::clamp(std::atoi(Value.c_str()), 0, 2);
	else if (Key == "top_rails")
		m_top_rails = static_cast<std::uint32_t>(std::strtoul(Value.c_str(), nullptr, 16)) | 0xff000000u;
	// under a key of its own since the laying goes from point to point unless told otherwise: the fitted heights chosen
	// before, when they were the default, are left behind
	else if (Key == "laying_heights")
		m_heights = heights_read(Value);
	else
		return false;
	return true;
}

void editorSettings::load_scenery(std::string const &Key, std::string const &Value)
{
	if (Key == "wms_layer")
	{
		if (auto const layer{wms_layer_read(Value)})
			m_wms_layers.push_back(*layer);
	}
	else if (Key.rfind(origin_prefix, 0) == 0)
	{
		// northing;easting
		auto const separator = Value.find(';');
		if (separator == std::string::npos)
			return;
		double north{0.0};
		double east{0.0};
		parse(Value.substr(0, separator), north);
		parse(Value.substr(separator + 1), east);
		m_orthophoto_origins[Key] = {north, east};
	}
	else if (Key.rfind(offset_prefix, 0) == 0)
	{
		// offset;fitted, the flag the one character after the separator
		auto const separator = Value.find(';');
		if (separator == std::string::npos)
			return;
		double offset{0.0};
		parse(Value.substr(0, separator), offset);
		m_terrain_model_offsets[Key] = {offset, Value.size() == separator + 2 && Value.ends_with('1')};
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

	std::string key;
	std::string value;
	while (stream >> key >> value)
	{
		// the keys of the scenery, some of them begun with a prefix, are looked at after all the others
		if (false == load_orthophoto(key, value, m_orthophoto) && false == load_terrain_model(key, value, m_terrain_model) && false == load_editor(key, value))
			load_scenery(key, value);
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
	stream << "terrain_buried_ask " << (m_terrain_buried_ask ? 1 : 0) << "\n";
	stream << "track_tools_all " << (m_track_tools_more ? 1 : 0) << "\n";
	stream << "terrain_while_laying " << (m_terrain_while_laying ? 1 : 0) << "\n";
	stream << "nmt_band " << m_terrain_model.band << "\n";
	stream << "nmt_around " << m_terrain_model.around << "\n";
	stream << "nmt_shape " << (m_terrain_model.shape ? 1 : 0) << "\n";
	stream << "nmt_follow " << (m_terrain_model.follow ? 1 : 0) << "\n";
	stream << "top_tracks " << m_top_tracks << "\n";
	stream << std::format("top_rails {:x}\n", m_top_rails & 0xffffffu);
	stream << "laying_heights " << heights_name(m_heights) << "\n";
	for (auto const &layer : m_wms_layers)
		stream << "wms_layer " << (layer.visible ? 1 : 0) << ";" << layer.opacity << ";" << encoded(layer.url) << ";" << encoded(layer.layers) << ";" << encoded(layer.title) << ";" << encoded(layer.terms) << "\n";
	stream << std::fixed << std::setprecision(3);
	for (auto const &origin : m_orthophoto_origins)
		stream << origin.first << " " << origin.second.first << ";" << origin.second.second << "\n";
	for (auto const &[scenery, stored] : m_terrain_model_offsets)
		stream << scenery << " " << stored.first << ";" << (stored.second ? 1 : 0) << "\n";
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
