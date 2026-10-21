/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// properties of the selected tracks in the inspector: one sheet for one track and for many, a value shared by all of
// them shown as it is and values which differ as "different", a value typed or chosen going to all of them in one
// step of the undo; the properties copied from one track and pasted on others; the tracks like the selected ones
// selected in the whole scenery

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorTrackPresets.hpp"

#include "rendering/renderer.h"
#include "world/Track.h"
#include "world/TrackFeatures.h"
#include "world/TrackSleepers.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"

#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <sstream>

namespace
{

using geometry::bezier;

int const kRailCategory{1};
std::size_t const kListedValues{8}; // in the tooltip of a property whose values differ
std::size_t const kLargeSelection{300}; // above it the values are gathered again only every few frames

bool rail(TTrack const &Track)
{
	return (Track.iCategoryFlag & 15) == kRailCategory && false == Track.m_editorremoved && Track.visible();
}

std::string name_of(TTrack const &Track)
{
	return Track.name().empty() ? std::string{STR_C("(no name)")} : Track.name();
}

char const *type_of(TTrack const &Track)
{
	switch (Track.eType)
	{
	case tt_Normal: return "normal";
	case tt_Switch: return "switch";
	case tt_Cross: return "cross";
	case tt_Table: return "turntable";
	case tt_Tributary: return "tributary";
	default: return "unknown type";
	}
}

// a number as short as it can be written: no trailing zeros
std::string decimal(double const Value, int const Digits)
{
	auto text{format("%.*f", Digits, Value)};
	if (text.find('.') != std::string::npos)
	{
		while (text.back() == '0')
			text.pop_back();
		if (text.back() == '.')
			text.pop_back();
	}
	return text == "-0" ? std::string{"0"} : text;
}

bool parse_number(std::string Text, double &Value)
{
	std::replace(Text.begin(), Text.end(), ',', '.');
	auto const *start{Text.c_str()};
	char *end{nullptr};
	Value = std::strtod(start, &end);
	if (end == start)
		return false;
	while (*end == ' ')
		++end;
	return *end == '\0' && std::isfinite(Value);
}

std::vector<std::string> words_of(std::string const &Text)
{
	std::istringstream stream{Text};
	std::vector<std::string> words;
	for (std::string word; stream >> word;)
		words.push_back(word);
	return words;
}

material_handle bed_of(TTrack const &Track)
{
	if (Track.eType == tt_Switch)
		return Track.SwitchExtension ? Track.SwitchExtension->m_material3 : null_handle;
	return Track.m_material2;
}

// image of the first texture of a material, 0 while there's none; reading a texture takes a moment, a few of them go in a frame
std::uint64_t material_image(material_handle const Material)
{
	static int frame{-1};
	static int loads{0};
	if (Material == null_handle)
		return 0;
	auto const *data{GfxRenderer->Material(Material)};
	if (data == nullptr)
		return 0;
	auto const texture{data->GetTexture(0)};
	if (texture == null_handle)
		return 0;
	auto &image{GfxRenderer->Texture(texture)};
	if (false == image.get_is_ready())
	{
		if (frame != ImGui::GetFrameCount())
		{
			frame = ImGui::GetFrameCount();
			loads = 0;
		}
		if (loads >= 3)
			return 0;
		++loads;
		image.create();
	}
	return image.get_is_ready() ? static_cast<std::uint64_t>(image.get_id()) : 0;
}

std::string lowered(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return Text;
}

bool texture_extension(std::string const &Extension)
{
	return Extension == ".dds" || Extension == ".tga" || Extension == ".png" || Extension == ".bmp" || Extension == ".jpg" || Extension == ".ktx" || Extension == ".tex";
}

// a name in two lines under a tile: as much as fits, then the rest; the end, where the names differ, is kept when the rest is too long
std::pair<std::string, std::string> two_lines(std::string const &Text, float const Width)
{
	if (ImGui::CalcTextSize(Text.c_str()).x <= Width)
		return {Text, {}};
	std::size_t split{0};
	while (split < Text.size() && ImGui::CalcTextSize(Text.c_str(), Text.c_str() + split + 1).x <= Width)
		++split;
	while (split > 0 && (static_cast<unsigned char>(Text[split]) & 0xC0) == 0x80)
		--split;
	auto rest{Text.substr(split)};
	if (ImGui::CalcTextSize(rest.c_str()).x > Width)
	{
		while (rest.size() > 1 && ImGui::CalcTextSize(("..." + rest).c_str()).x > Width)
		{
			rest.erase(0, 1);
			while (false == rest.empty() && (static_cast<unsigned char>(rest.front()) & 0xC0) == 0x80)
				rest.erase(0, 1);
		}
		rest = "..." + rest;
	}
	return {Text.substr(0, split), rest};
}

// files of the folder of the textures shown in the picker at most, the rest wait for more of the name typed
std::size_t const kPickerFiles{120};

// the width of each of Count buttons in a row across the panel
float button_width(int const Count)
{
	auto const spacing{ImGui::GetStyle().ItemSpacing.x};
	return std::max(1.0f, (ImGui::GetContentRegionAvail().x - spacing * static_cast<float>(Count - 1)) / static_cast<float>(Count));
}

enum class field
{
	number,
	text,
	choice,
	check,
	texture
};

// parts of the sheet, in their order
enum section : int
{
	traffic,
	look,
	look_bed,
	look_sleepers,
	ride
};

// what can be pasted, each by itself
enum group : int
{
	speed,
	surroundings,
	overhead_line,
	circuits,
	rails,
	trackbed,
	trackbed_shape,
	sleepers,
	ride_sound,
	frogs,
	sleeper_placement, // spacing and shift of the sleepers and of the trackbed under them, apart from their models
	group_count
};

char const *const kGroupLabels[group_count] = {"Speed", "Environment", "Overhead line", "Track circuits", "Rails", "Trackbed", "Shape of the trackbed", "Sleepers", "Ride and sound", "Frog and check rails", "Sleeper placement"};
// the groups as the presets keep them in their file, by names which don't change with the order of the groups
char const *const kGroupKeys[group_count] = {"speed", "environment", "overhead", "circuits", "rails", "trackbed", "bedshape", "sleepers", "ride", "frogs", "sleeperplacement"};

struct track_property
{
	char const *key{""}; // names it in the self-tests and in the copied properties
	field kind{field::number};
	int part{section::traffic};
	int group{group::speed};
	bool rebuild{false}; // the geometry of the track is made again after a change
	bool late{false};    // pasted after the others, it depends on them
	bool more{false};
	int texture{-1};     // which list of the textures on the tracks: 0 rails, 1 trackbed
	char const *label{""};
	char const *tooltip{""};
	char const *empty{nullptr}; // hint of an empty field
	std::vector<char const *> choices;
	std::function<bool(TTrack const &)> applies;
	std::function<std::string(TTrack const &)> get;
	// false: the text doesn't fit the track or changes nothing, the track stays as it was
	std::function<bool(TTrack &, std::string const &)> set;
};

track_property &add(std::vector<track_property> &List, char const *Key, field const Kind, int const Part, int const Group, char const *Label, char const *Tooltip)
{
	auto &property{List.emplace_back()};
	property.key = Key;
	property.kind = Kind;
	property.part = Part;
	property.group = Group;
	property.label = Label;
	property.tooltip = Tooltip;
	property.applies = [](TTrack const &Track) { return Track.m_visible; };
	return property;
}

// a number kept in a member of the track, Least the lowest value it takes
void add_float(std::vector<track_property> &List, char const *Key, int const Part, int const Group, char const *Label, char const *Tooltip, float TTrack::*Member, int const Digits, float const Least, bool const Rebuild)
{
	auto &property{add(List, Key, field::number, Part, Group, Label, Tooltip)};
	property.rebuild = Rebuild;
	property.get = [Member, Digits](TTrack const &Track) { return decimal(Track.*Member, Digits); };
	property.set = [Member, Digits, Least](TTrack &Track, std::string const &Text) {
		double value;
		if (false == parse_number(Text, value))
			return false;
		auto const clamped{std::max(Least, static_cast<float>(value))};
		if (decimal(clamped, Digits) == decimal(Track.*Member, Digits))
			return false;
		Track.*Member = clamped;
		return true;
	};
}

std::vector<track_property> make_properties()
{
	std::vector<track_property> list;
	auto const any = [](TTrack const &) { return true; };
	float const lowest{std::numeric_limits<float>::lowest()};

	// traffic
	{
		auto &property{add(list, "speed", field::number, section::traffic, group::speed, "Speed (km/h)", "Speed limit of the track for the AI drivers and the timetable.\nEmpty or -1: no limit of its own")};
		property.applies = any;
		property.empty = "no limit";
		property.get = [](TTrack const &Track) {
			auto const velocity{editor_track::velocity(Track)};
			return velocity < 0.0 ? std::string{} : decimal(velocity, 1);
		};
		property.set = [](TTrack &Track, std::string const &Text) {
			double velocity{-1.0};
			if (false == words_of(Text).empty() && false == parse_number(Text, velocity))
				return false;
			if (velocity < 0.0)
				velocity = -1.0;
			if (std::abs(editor_track::velocity(Track) - velocity) < 1e-6)
				return false;
			editor_track::velocity(Track, velocity);
			return true;
		};
	}
	{
		auto &property{add(list, "environment", field::choice, section::traffic, group::surroundings, "Environment", "Surroundings of the track, they affect the sound of the passing vehicles")};
		property.applies = any;
		property.choices = {"flat", "mountains", "canyon", "tunnel", "bridge", "bank"};
		property.get = [](TTrack const &Track) { return std::to_string(std::clamp(static_cast<int>(Track.eEnvironment), 0, 5)); };
		property.set = [](TTrack &Track, std::string const &Text) {
			double index;
			if (false == parse_number(Text, index) || index < 0.0 || index > 5.0 || static_cast<int>(index) == static_cast<int>(Track.eEnvironment))
				return false;
			Track.eEnvironment = static_cast<TEnvironmentType>(static_cast<int>(index));
			return true;
		};
	}
	{
		auto &property{add(list, "overhead", field::choice, section::traffic, group::overhead_line, "Overhead line", "State of the overhead line over the track: the trains coast through a section with no current,\nor lower the pantographs and keep to the speed limit given")};
		property.applies = any;
		property.more = true;
		property.choices = {"Normal", "No current, coasting", "Pantographs lowered, speed limit"};
		property.get = [](TTrack const &Track) { return std::string{Track.fOverhead < 0.f ? "0" : Track.fOverhead == 0.f ? "1" : "2"}; };
		property.set = [](TTrack &Track, std::string const &Text) {
			double index;
			if (false == parse_number(Text, index) || index < 0.0 || index > 2.0)
				return false;
			auto const mode{static_cast<int>(index)};
			auto const overhead{mode == 0 ? -1.f : mode == 1 ? 0.f : Track.fOverhead > 0.f ? Track.fOverhead : 40.f};
			if (overhead == Track.fOverhead)
				return false;
			editor_track::overhead(Track, overhead);
			return true;
		};
	}
	{
		auto &property{add(list, "overheadspeed", field::number, section::traffic, group::overhead_line, "Speed with the pantographs lowered (km/h)", "Speed limit of the trains with the pantographs lowered")};
		property.more = true;
		property.applies = [](TTrack const &Track) { return Track.fOverhead > 0.f; };
		property.get = [](TTrack const &Track) { return decimal(Track.fOverhead, 0); };
		property.set = [](TTrack &Track, std::string const &Text) {
			double limit;
			if (false == parse_number(Text, limit))
				return false;
			auto const overhead{std::max(1.f, static_cast<float>(limit))};
			if (overhead == Track.fOverhead)
				return false;
			editor_track::overhead(Track, overhead);
			return true;
		};
	}
	{
		auto &property{add(list, "circuits", field::text, section::traffic, group::circuits, "Track circuits", "Names of the track circuits (isolated) the track belongs to, separated by spaces.\nA name which doesn't exist yet makes a new circuit. Tracks with vehicles on them keep theirs")};
		property.applies = any;
		property.more = true;
		property.empty = "no circuits";
		property.get = [](TTrack const &Track) {
			std::string text;
			for (auto const *isolated : Track.Isolated)
				text += (text.empty() ? "" : " ") + isolated->asName;
			return text;
		};
		property.set = [get = property.get](TTrack &Track, std::string const &Text) {
			auto const names{words_of(Text)};
			std::string joined;
			for (auto const &name : names)
				joined += (joined.empty() ? "" : " ") + name;
			if (false == Track.Dynamics.empty() || joined == get(Track))
				return false;
			editor_track::isolated(Track, names);
			return true;
		};
	}

	// look
	{
		auto &property{add(list, "rails", field::texture, section::look, group::rails, "Rails", "Texture of the rails; of both paths of a switch, those of its path 2 can then be set apart.\nA material dragged from the texture browser can be dropped here")};
		property.rebuild = true;
		property.texture = 0;
		property.empty = "No texture";
		property.get = [](TTrack const &Track) { return editor_track::material_name(Track.m_material1); };
		property.set = [](TTrack &Track, std::string const &Text) {
			auto const material{editor_track::fetch_material(Text)};
			if (material == Track.m_material1 && (Track.eType != tt_Switch || material == Track.m_material2))
				return false;
			Track.m_material1 = material;
			if (Track.eType == tt_Switch)
				Track.m_material2 = material;
			return true;
		};
	}
	{
		// a switch keeps the rails of its path 2 in its second texture, its trackbed is m_material3
		auto &property{add(list, "rails2", field::texture, section::look, group::rails, "Rails of path 2",
		                   "Texture of the rails of path 2 of the switch, from p3 to p4, usually the diverging track.\nPath 1, from p1 to p2, keeps the texture of the rails above.\nA material dragged from the texture browser can be dropped here")};
		property.rebuild = true;
		property.late = true; // after the rails, which set both paths
		property.texture = 0;
		property.empty = "No texture";
		property.applies = [](TTrack const &Track) { return Track.m_visible && Track.eType == tt_Switch; };
		property.get = [](TTrack const &Track) { return editor_track::material_name(Track.m_material2); };
		property.set = [](TTrack &Track, std::string const &Text) {
			auto const material{editor_track::fetch_material(Text)};
			if (Track.eType != tt_Switch || material == Track.m_material2)
				return false;
			Track.m_material2 = material;
			return true;
		};
	}
	{
		// the profiles of the rails the game has (models/tory/railprofile_*), the first: none, the default shape
		static char const *const profiles[] = {"", "s60", "uic60", "s49", "s42", "s39", "s30pn", "s30din", "s24", "s20", "s18", "s14", "s10", "r-65", "ri60n", "lk1"};
		auto &property{add(list, "railprofile", field::choice, section::look, group::rails, "Rail profile",
		                   "Shape of the rails, the railprofile of the track. It also picks the switch templates of the rail: S60 60E1, S49 49E1.\nNo profile: the default shape, and the templates of all the rails are offered")};
		property.rebuild = true;
		property.more = true;
		property.applies = [](TTrack const &Track) { return Track.m_visible && (Track.iCategoryFlag & 15) == 1; };
		property.choices = {"No profile", "S60 (60E1)", "UIC60", "S49 (49E1)", "S42", "S39", "S30 PN", "S30 DIN", "S24", "S20", "S18", "S14", "S10", "R65", "Ri60N", "LK1"};
		property.get = [](TTrack const &Track) {
			auto const at{std::find_if(std::begin(profiles), std::end(profiles), [&](char const *Name) { return Track.m_profile1.first == Name; })};
			return at != std::end(profiles) ? std::to_string(at - std::begin(profiles)) : Track.m_profile1.first;
		};
		property.set = [](TTrack &Track, std::string const &Text) {
			double index;
			if (false == parse_number(Text, index) || index < 0.0 || index >= static_cast<double>(std::size(profiles)))
				return false;
			std::string const name{profiles[static_cast<int>(index)]};
			if (name == Track.m_profile1.first)
				return false;
			Track.rail_profile(name);
			return true;
		};
	}
	{
		auto &property{add(list, "trackbed", field::texture, section::look, group::trackbed, "Trackbed", "Texture of the trackbed (ballast) under the rails.\nA material dragged from the texture browser can be dropped here")};
		property.rebuild = true;
		property.texture = 1;
		property.empty = "No texture";
		property.applies = [](TTrack const &Track) { return Track.m_visible && (Track.eType != tt_Switch || Track.SwitchExtension != nullptr); };
		property.get = [](TTrack const &Track) { return editor_track::material_name(bed_of(Track)); };
		property.set = [](TTrack &Track, std::string const &Text) {
			auto const material{editor_track::fetch_material(Text)};
			if (material == bed_of(Track))
				return false;
			if (Track.eType == tt_Switch)
				Track.SwitchExtension->m_material3 = material;
			else
				Track.m_material2 = material;
			return true;
		};
	}
	{
		auto &property{add(list, "frogs", field::check, section::look, group::frogs, "Frog and check rails made from the rails", "Cuts the rails of the switch at its frog and lays wing rails and check rails.\nOff for a switch whose include has its own model of them")};
		property.rebuild = true;
		property.applies = [](TTrack const &Track) { return track_features::crossings && Track.eType == tt_Switch && Track.SwitchExtension != nullptr; };
		property.get = [](TTrack const &Track) { return std::string{Track.SwitchExtension->m_frogs ? "1" : "0"}; };
		property.set = [](TTrack &Track, std::string const &Text) {
			auto const frogs{Text == "1"};
			if (frogs == Track.SwitchExtension->m_frogs)
				return false;
			Track.SwitchExtension->m_frogs = frogs;
			return true;
		};
	}
	{
		auto &property{add(list, "movablenose", field::check, section::look, group::frogs, "Movable nose", "The nose of the frog moves with the blades over to the wing rail of the track set, leaving no gap; the frog has no check rails")};
		property.rebuild = true;
		property.applies = [](TTrack const &Track) { return track_features::crossings && Track.eType == tt_Switch && Track.SwitchExtension != nullptr; };
		property.get = [](TTrack const &Track) { return std::string{Track.SwitchExtension->m_movablenose ? "1" : "0"}; };
		property.set = [](TTrack &Track, std::string const &Text) {
			auto const movable{Text == "1"};
			if (movable == Track.SwitchExtension->m_movablenose)
				return false;
			Track.SwitchExtension->m_movablenose = movable;
			return true;
		};
	}
	{
		auto const blades = [&](char const *Key, char const *Label, char const *Tooltip, double TSwitchExtension::*Member) {
			auto &property{add(list, Key, field::number, section::look, group::frogs, Label, Tooltip)};
			property.rebuild = true;
			property.applies = [](TTrack const &Track) { return Track.eType == tt_Switch && Track.SwitchExtension != nullptr; };
			property.get = [Member](TTrack const &Track) { return decimal(( *Track.SwitchExtension ).*Member, 3); };
			property.set = [Member](TTrack &Track, std::string const &Text) {
				double value;
				if (false == parse_number(Text, value))
					return false;
				value = std::max(0.0, value);
				if (decimal(value, 3) == decimal(( *Track.SwitchExtension ).*Member, 3))
					return false;
				( *Track.SwitchExtension ).*Member = value;
				return true;
			};
		};
		blades("bladeheel", "Blade heel (m)", "Where the moving blades end, from the start of the switch, as the template of the switch gives it.\n0: as long as the flangeway of 58 mm behind them needs", &TSwitchExtension::m_bladeheel);
		blades("bladethrow", "Blade throw (m)", "How far the tips of the blades move over between the two paths.\n0: the usual 0.160 m", &TSwitchExtension::m_bladethrow);
	}
	add_float(list, "texlength", section::look_bed, group::trackbed_shape, "Texture length (m)", "Length after which the texture of the rails and the trackbed repeats", &TTrack::fTexLength, 2, 0.01f, true);
	{
		auto &property{add(list, "bedheight", field::number, section::look_bed, group::trackbed_shape, "Trackbed height (m)", "Height of the trackbed edge below the rail top")};
		property.rebuild = true;
		property.get = [](TTrack const &Track) { return decimal(editor_track::texture_height(Track), 3); };
		property.set = [](TTrack &Track, std::string const &Text) {
			double height;
			if (false == parse_number(Text, height) || decimal(height, 3) == decimal(editor_track::texture_height(Track), 3))
				return false;
			editor_track::texture_height(Track, static_cast<float>(height));
			return true;
		};
	}
	add_float(list, "bedwidth", section::look_bed, group::trackbed_shape, "Trackbed width (m)", "Width of the trackbed outside of the rail", &TTrack::fTexWidth, 3, lowest, true);
	add_float(list, "bedslope", section::look_bed, group::trackbed_shape, "Trackbed slope (m)", "Horizontal reach of the slope at the outer edge of the trackbed", &TTrack::fTexSlope, 3, lowest, true);
	{
		auto &property{add(list, "sleepermodel", field::text, section::look_sleepers, group::sleepers, "Sleeper model", "Path of the .e3d model, as in the scenery file")};
		property.rebuild = true;
		property.empty = "no model";
		property.get = [](TTrack const &Track) { return Track.m_sleeper_model_name; };
		property.set = [](TTrack &Track, std::string const &Text) {
			auto const words{words_of(Text)};
			auto const model{words.empty() ? std::string{} : words.front()};
			if (model == Track.m_sleeper_model_name)
				return false;
			Track.m_sleeper_model_name = model;
			if (model.empty())
				Track.m_sleeper_enabled = false;
			return true;
		};
	}
	{
		auto &property{add(list, "sleepers", field::check, section::look_sleepers, group::sleepers, "Sleeper models", "Repeats the model along the track, the sleepers in place of the drawn trackbed.\nOnly on the tracks with a sleeper model")};
		property.rebuild = true;
		property.late = true;
		property.get = [](TTrack const &Track) { return std::string{Track.m_sleeper_enabled ? "1" : "0"}; };
		property.set = [](TTrack &Track, std::string const &Text) {
			auto const enabled{Text == "1"};
			if (enabled == Track.m_sleeper_enabled || (enabled && Track.m_sleeper_model_name.empty()))
				return false;
			Track.m_sleeper_enabled = enabled;
			return true;
		};
	}
	{
		auto &property{add(list, "sleeperskin", field::text, section::look_sleepers, group::sleepers, "Sleeper skin", "Replaceable skin of the model, empty: the one of the model")};
		property.rebuild = true;
		property.empty = "of the model";
		property.get = [](TTrack const &Track) { return Track.m_sleeper_skin_name; };
		property.set = [](TTrack &Track, std::string const &Text) {
			auto const words{words_of(Text)};
			auto skin{words.empty() ? std::string{} : words.front()};
			if (skin == "none")
				skin.clear();
			if (skin == Track.m_sleeper_skin_name)
				return false;
			Track.m_sleeper_skin_name = skin;
			return true;
		};
	}
	{
		// the models of sleepers and timbers of the scenery data, chosen by the name; the first choice leaves the one of eu07.ini
		static std::vector<std::string> sleepermodels;
		static std::vector<std::string> timbermodels;
		if (sleepermodels.empty())
		{
			for (auto const &model : track_sleepers::pack_models())
				(model.timber ? timbermodels : sleepermodels).push_back(model.name);
		}
		auto const choices = [](std::vector<std::string> const &Models) {
			std::vector<char const *> result{"(as in eu07.ini)"};
			for (auto const &model : Models)
				result.push_back(model.c_str());
			return result;
		};
		auto const index_of = [](std::vector<std::string> const &Models, std::string const &Name) {
			auto const found{std::find(Models.begin(), Models.end(), Name)};
			return found == Models.end() ? 0 : static_cast<int>(found - Models.begin()) + 1;
		};
		{
			auto &property{add(list, "sleeperpack", field::choice, section::look_sleepers, group::sleepers, "Sleeper model from the pack", "Model of the sleepers of the track running alone, of the models of sleepers in the scenery data, laid as 3D sleepers\nin place of the one set with sleepers3dplain in eu07.ini. (as in eu07.ini): the one set there")};
			property.rebuild = true;
			property.applies = [](TTrack const &Track) { return track_features::sleepers && Track.m_visible; };
			property.choices = choices(sleepermodels);
			property.get = [index_of](TTrack const &Track) { return std::to_string(Track.m_sleeper_enabled ? index_of(sleepermodels, Track.m_sleeper_model_name) : 0); };
			property.set = [](TTrack &Track, std::string const &Text) {
				double index;
				if (false == parse_number(Text, index) || index < 0.0 || index > static_cast<double>(sleepermodels.size()))
					return false;
				auto const model{index < 1.0 ? std::string{} : sleepermodels[static_cast<std::size_t>(index) - 1]};
				if (model == (Track.m_sleeper_enabled ? Track.m_sleeper_model_name : std::string{}))
					return false;
				Track.m_sleeper_model_name = model;
				Track.m_sleeper_enabled = (false == model.empty());
				return true;
			};
		}
		{
			auto &property{add(list, "timbermodel", field::choice, section::look_sleepers, group::sleepers, "Timber model", "Model of the timbers under the switch or the crossing the track is part of, of the models of timbers in the scenery data,\nin place of the one set with sleepers3d in eu07.ini. It's put together from its submodels begin, main and end to the length of each timber")};
			property.rebuild = true;
			property.applies = [](TTrack const &Track) { return track_features::sleepers && Track.m_visible; };
			property.choices = choices(timbermodels);
			property.get = [index_of](TTrack const &Track) { return std::to_string(index_of(timbermodels, Track.m_timber_model_name)); };
			property.set = [](TTrack &Track, std::string const &Text) {
				double index;
				if (false == parse_number(Text, index) || index < 0.0 || index > static_cast<double>(timbermodels.size()))
					return false;
				auto const model{index < 1.0 ? std::string{} : timbermodels[static_cast<std::size_t>(index) - 1]};
				if (model == Track.m_timber_model_name)
					return false;
				Track.m_timber_model_name = model;
				return true;
			};
		}
		{
			auto &property{add(list, "timberskin", field::text, section::look_sleepers, group::sleepers, "Timber skin", "Replaceable skin of the timber model, empty: the one of the model")};
			property.rebuild = true;
			property.applies = [](TTrack const &Track) { return track_features::sleepers && Track.m_visible; };
			property.empty = "of the model";
			property.get = [](TTrack const &Track) { return Track.m_timber_skin_name; };
			property.set = [](TTrack &Track, std::string const &Text) {
				auto const words{words_of(Text)};
				auto skin{words.empty() ? std::string{} : words.front()};
				if (skin == "none")
					skin.clear();
				if (skin == Track.m_timber_skin_name)
					return false;
				Track.m_timber_skin_name = skin;
				return true;
			};
		}
	}
	add_float(list, "sleeperspacing", section::look_sleepers, group::sleeper_placement, "Sleeper spacing (m)", "Distance between the sleepers along the track", &TTrack::m_sleeper_frequency, 3, 0.1f, true);
	char const *const offsets[][3] = {{"sleeperside", "Sleeper shift sideways (m)", "Shift of each model sideways, to the right of the track"},
	                                  {"sleeperalong", "Sleeper shift along (m)", "Shift of each model along the track"},
	                                  {"sleeperup", "Sleeper shift up (m)", "Shift of each model up"}};
	for (int axis = 0; axis < 3; ++axis)
	{
		auto &property{add(list, offsets[axis][0], field::number, section::look_sleepers, group::sleeper_placement, offsets[axis][1], offsets[axis][2])};
		property.rebuild = true;
		property.get = [axis](TTrack const &Track) { return decimal(Track.m_sleeper_offset[axis], 3); };
		property.set = [axis](TTrack &Track, std::string const &Text) {
			double offset;
			if (false == parse_number(Text, offset) || decimal(offset, 3) == decimal(Track.m_sleeper_offset[axis], 3))
				return false;
			Track.m_sleeper_offset[axis] = static_cast<float>(offset);
			return true;
		};
	}
	add_float(list, "bedshift", section::look_sleepers, group::sleeper_placement, "Trackbed shift (m)", "Vertical shift of the drawn trackbed under the sleepers, negative lowers it", &TTrack::m_sleeper_ballast_z, 3, lowest, true);

	// ride and sound
	add_float(list, "friction", section::ride, group::ride_sound, "Friction", "Friction coefficient of the rails, 0.15 for the dry rails; lower values make the wheels slip", &TTrack::fFriction, 3, 0.f, false);
	list.back().applies = any;
	add_float(list, "sounddistance", section::ride, group::ride_sound, "Sound distance (m)", "Distance between the rail joints, sets the rhythm of the wheel clatter; -1: the default", &TTrack::fSoundDistance, 1, -1.f, false);
	list.back().applies = any;
	{
		auto &property{add(list, "quality", field::number, section::ride, group::ride_sound, "Quality", "Quality of the track, 20 is the nominal one; lower values give a rougher ride")};
		property.applies = any;
		property.get = [](TTrack const &Track) { return std::to_string(Track.iQualityFlag); };
		property.set = [](TTrack &Track, std::string const &Text) {
			double quality;
			if (false == parse_number(Text, quality) || static_cast<int>(quality) == Track.iQualityFlag)
				return false;
			Track.iQualityFlag = static_cast<int>(quality);
			return true;
		};
	}
	{
		auto &property{add(list, "damage", field::number, section::ride, group::ride_sound, "Damage flags", "0: track in order; any value adds jolts, 128: the vehicles derail on the track")};
		property.applies = any;
		property.get = [](TTrack const &Track) { return std::to_string(Track.iDamageFlag); };
		property.set = [](TTrack &Track, std::string const &Text) {
			double damage;
			if (false == parse_number(Text, damage) || static_cast<int>(damage) == Track.iDamageFlag)
				return false;
			editor_track::damage(Track, static_cast<int>(damage));
			return true;
		};
	}
	add_float(list, "gauge", section::ride, group::ride_sound, "Gauge (m)", "Gauge of the track, 1.435 m for the standard gauge", &TTrack::fTrackWidth, 3, 0.01f, true);
	list.back().applies = any;
	return list;
}

std::vector<track_property> const &properties()
{
	static std::vector<track_property> const list{make_properties()};
	return list;
}

using value_counts = std::vector<std::pair<std::string, int>>;

// values of the property over the tracks it applies to, each with the number of the tracks having it, the most common first
value_counts values_of(track_property const &Property, std::vector<TTrack *> const &Tracks)
{
	std::map<std::string, int> counted;
	for (auto const *track : Tracks)
		if (track != nullptr && false == track->m_editorremoved && Property.applies(*track))
			++counted[Property.get(*track)];
	value_counts result(counted.begin(), counted.end());
	std::stable_sort(result.begin(), result.end(), [](auto const &Left, auto const &Right) { return Left.second > Right.second; });
	return result;
}

// a value as the user reads it
std::string shown_value(track_property const &Property, std::string const &Value)
{
	switch (Property.kind)
	{
	case field::choice:
	{
		auto const index{std::atoi(Value.c_str())};
		return index >= 0 && index < static_cast<int>(Property.choices.size()) ? std::string{STR_C(Property.choices[index])} : Value;
	}
	case field::check: return Value == "1" ? STR_C("yes") : STR_C("no");
	default: return Value.empty() || Value == "none" ? std::string{STR_C(Property.empty != nullptr ? Property.empty : "(none)")} : Value;
	}
}

std::string tracks_count(std::size_t const Count)
{
	return format(STR_P("%d track", "%d tracks", static_cast<long long>(Count)), static_cast<int>(Count));
}

// the tooltip of a row: what the property is, and the values of the tracks when they differ
void row_tooltip(track_property const &Property, value_counts const &Values, std::size_t const Applies, std::size_t const Selected)
{
	// not over a field being typed into, it would cover the rows under it
	if (false == ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort) || ImGui::IsItemActive())
		return;
	ImGui::BeginTooltip();
	ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
	ImGui::TextUnformatted(STR_C(Property.tooltip));
	if (Applies < Selected)
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), STR_C("Applies to %d of the %d selected tracks, the others don't have it"), static_cast<int>(Applies), static_cast<int>(Selected));
	if (Values.size() > 1)
	{
		ImGui::Separator();
		ImGui::TextUnformatted(STR_C("The tracks differ:"));
		for (std::size_t i = 0; i < Values.size() && i < kListedValues; ++i)
			ImGui::BulletText("%s  —  %s", shown_value(Property, Values[i].first).c_str(), tracks_count(static_cast<std::size_t>(Values[i].second)).c_str());
		if (Values.size() > kListedValues)
			ImGui::BulletText(STR_C("and %d more values"), static_cast<int>(Values.size() - kListedValues));
		ImGui::TextDisabled("%s", STR_C("A value given here goes to all of them"));
	}
	ImGui::PopTextWrapPos();
	ImGui::EndTooltip();
}

// the values of the properties of the track, as they're copied and kept by the presets
std::vector<std::pair<std::string, std::string>> values_of_track(TTrack const &Track)
{
	std::vector<std::pair<std::string, std::string>> values;
	for (auto const &property : properties())
		if (property.applies(Track))
			values.emplace_back(property.key, property.get(Track));
	return values;
}

// the groups some of the values are of
std::array<bool, group_count> groups_in(std::vector<std::pair<std::string, std::string>> const &Values)
{
	std::array<bool, group_count> groups{};
	for (auto const &[key, value] : Values)
		for (auto const &property : properties())
			if (key == property.key)
				groups[property.group] = true;
	return groups;
}

// the groups the preset overwrites, from the names it keeps them by
std::array<bool, group_count> groups_of(editor_track_preset const &Preset)
{
	std::array<bool, group_count> groups{};
	for (auto const &key : Preset.groups)
		for (int group = 0; group < group_count; ++group)
			if (key == kGroupKeys[group])
				groups[group] = true;
	return groups;
}

std::vector<std::string> group_keys(std::array<bool, group_count> const &Groups)
{
	std::vector<std::string> keys;
	for (int group = 0; group < group_count; ++group)
		if (Groups[group])
			keys.emplace_back(kGroupKeys[group]);
	return keys;
}

// what the preset puts on the tracks, for its tooltip
std::string preset_summary(editor_track_preset const &Preset)
{
	auto const groups{groups_of(Preset)};
	auto const available{groups_in(Preset.values)};
	std::string overwrites;
	for (int group = 0; group < group_count; ++group)
		if (groups[group] && available[group])
			overwrites += (overwrites.empty() ? "" : ", ") + std::string{STR_C(kGroupLabels[group])};
	auto text{overwrites.empty() ? std::string{STR_C("Overwrites nothing: choose what it overwrites in the inspector")} : format(STR_C("Overwrites: %s"), overwrites.c_str())};
	if (false == Preset.from.empty())
		text += "\n" + format(STR_C("Taken from %s"), Preset.from.c_str());
	return text;
}

} // namespace

std::vector<TTrack *> editor_mode::selected_tracks()
{
	track_set_prune();
	if (m_track_set.size() >= 2)
		return m_track_set;
	if (auto *track{selected_track()}; track != nullptr && false == track->m_editorremoved)
		return {track};
	return {};
}

void editor_mode::select_tracks(std::vector<TTrack *> const &Tracks)
{
	// the tracks may be the set itself
	std::vector<TTrack *> tracks;
	for (auto *track : Tracks)
		if (track != nullptr && false == track->m_editorremoved && std::find(tracks.begin(), tracks.end(), track) == tracks.end())
			tracks.push_back(track);
	m_track_set.clear();
	if (tracks.size() >= 2)
		m_track_set = tracks;
	m_node = tracks.empty() ? nullptr : tracks.back();
	m_track_point = {};
	ui()->set_node(m_node);
}

std::size_t editor_mode::tracks_apply(std::vector<TTrack *> const &Tracks, std::function<bool(TTrack &)> const &Change, bool const Rebuild, char const *What)
{
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> changed;
	std::size_t locked{0};
	for (auto *track : Tracks)
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		if (false == scene::Layers.editable(track))
		{
			++locked;
			continue;
		}
		auto const before{editor_track::capture(*track)};
		if (false == Change(*track))
			continue;
		states.emplace_back(track, before);
		changed.push_back(track);
	}
	if (false == changed.empty())
	{
		push_track_snapshot(std::move(states));
		for (auto *track : changed)
		{
			if (Rebuild)
				editor_track::commit_parameters(*track);
			else
				track->mark_dirty();
		}
		++m_track_values.generation;
	}
	if (Tracks.size() > 1 || locked > 0)
	{
		auto status{changed.empty() ? format(STR_C("%s: nothing to change"), STR_C(What)) : format(STR_P("%s: changed on %d track, Ctrl+Z takes it back", "%s: changed on %d tracks, Ctrl+Z takes it back", static_cast<long long>(changed.size())), STR_C(What), static_cast<int>(changed.size()))};
		if (locked > 0)
			status += format(STR_C(", %zu in the files which can't be changed are left as they are"), locked);
		ui()->set_status(status, locked > 0);
	}
	return changed.size();
}

int editor_mode::track_property_index(std::string const &Key) const
{
	auto const &list{properties()};
	for (std::size_t i = 0; i < list.size(); ++i)
		if (Key == list[i].key)
			return static_cast<int>(i);
	return -1;
}

std::size_t editor_mode::track_property_apply(int const Property, std::vector<TTrack *> const &Tracks, std::string const &Text)
{
	auto const &property{properties()[Property]};
	std::vector<TTrack *> targets;
	for (auto *track : Tracks)
		if (track != nullptr && false == track->m_editorremoved && property.applies(*track))
			targets.push_back(track);
	auto const changed{tracks_apply(targets, [&](TTrack &Track) { return property.set(Track, Text); }, property.rebuild, property.label)};
	if (changed == 0 && targets.size() == 1)
	{
		double value;
		if (property.kind == field::number && false == words_of(Text).empty() && false == parse_number(Text, value))
			ui()->set_status(format(STR_C("%s: \"%s\" isn't a number"), STR_C(property.label), Text.c_str()), true);
		else if (std::string{property.key} == "sleepers" && Text == "1" && targets.front()->m_sleeper_model_name.empty())
			ui()->set_status(STR_C("Give the sleeper model first"), true);
		else if (std::string{property.key} == "circuits" && false == targets.front()->Dynamics.empty())
			ui()->set_status(STR_C("The track circuits can't change while vehicles stand on the path"), true);
	}
	return changed;
}

void editor_mode::track_field_settle()
{
	// a field left without being drawn again, the selection gone or folded away: what was typed is taken as it is
	auto &edit{m_track_field};
	if (edit.property < 0 || edit.frame + 1 >= ImGui::GetFrameCount())
		return;
	auto const property{edit.property};
	auto const tracks{edit.tracks};
	std::string const typed{edit.text.data()};
	auto const initial{edit.initial};
	edit = {};
	if (typed != initial)
		track_property_apply(property, tracks, typed);
}

void editor_mode::track_textures_scan()
{
	for (int kind = 0; kind < 2; ++kind)
	{
		std::map<material_handle, int> counted;
		for (auto const *track : simulation::Paths.sequence())
			if (track != nullptr && rail(*track) && track->m_visible && (kind == 0 || track->eType != tt_Switch || track->SwitchExtension))
				++counted[kind == 0 ? track->m_material1 : bed_of(*track)];
		auto &list{m_track_textures[kind]};
		list.clear();
		for (auto const &[material, count] : counted)
			if (material != null_handle)
				list.push_back({material, editor_track::material_name(material), count});
		std::stable_sort(list.begin(), list.end(), [](auto const &Left, auto const &Right) { return Left.count > Right.count; });
	}
}

void editor_mode::track_textures_index()
{
	if (m_track_texture_indexed)
		return;
	m_track_texture_indexed = true;
	std::map<std::string, bool> found; // the name, whether there's a .mat of it
	std::filesystem::path const root{paths::textures};
	auto const options{std::filesystem::directory_options::follow_directory_symlink | std::filesystem::directory_options::skip_permission_denied};
	std::error_code error;
	for (std::filesystem::recursive_directory_iterator entry{root, options, error}, end; false == static_cast<bool>(error) && entry != end; entry.increment(error))
	{
		std::error_code ignored;
		if (entry->is_directory(ignored))
		{
			// the normal maps go with their textures, not on a track by themselves
			if (lowered(entry->path().filename().string()) == "normals")
				entry.disable_recursion_pending();
			continue;
		}
		auto const extension{lowered(entry->path().extension().string())};
		auto const material{extension == ".mat"};
		if (false == material && false == texture_extension(extension))
			continue;
		auto path{entry->path().lexically_relative(root)};
		path.replace_extension();
		auto &known{found[path.generic_string()]};
		known = known || material;
	}
	m_track_texture_index.clear();
	m_track_texture_index.reserve(found.size());
	for (auto const &[name, material] : found)
		m_track_texture_index.push_back({name, lowered(name), material});
}

std::uint64_t editor_mode::track_texture_image(int const Kind, std::string const &Name)
{
	for (auto const &use : m_track_textures[Kind])
		if (use.name == Name)
			return material_image(use.material);
	auto const &index{m_track_texture_index};
	auto const found{std::lower_bound(index.begin(), index.end(), Name, [](texture_index_entry const &Entry, std::string const &Wanted) { return Entry.name < Wanted; })};
	texture_browser::item item;
	item.name = Name;
	item.material = found == index.end() || found->name != Name || found->material;
	return texture_thumbnail(item);
}

bool editor_mode::track_texture_picker(int const Kind, std::string const &Current, bool const Mixed, std::string &Chosen)
{
	auto &filter{m_track_texture_filter};
	if (ImGui::IsWindowAppearing())
	{
		track_textures_scan();
		track_textures_index();
		filter[0] = '\0';
		m_track_texture_hovered.clear();
		ImGui::SetKeyboardFocusHere();
	}
	// the thumbnails are read a few in a frame
	m_textures.loads = 0;
	ImGui::SetNextItemWidth(-FLT_MIN);
	auto const entered{ImGui::InputTextWithHint("##filter", STR_C("find, or a texture path and Enter"), filter.data(), filter.size(), ImGuiInputTextFlags_EnterReturnsTrue)};
	std::string const typed{filter.data()};
	auto const words{words_of(lowered(typed))};
	auto const matches = [&](std::string const &Lowered) {
		return std::all_of(words.begin(), words.end(), [&](std::string const &Word) { return Lowered.find(Word) != std::string::npos; });
	};
	// those on the tracks of the scenery first, then the files; with nothing typed the files offered are the ones named like the kind
	std::vector<track_texture_use const *> used;
	std::set<std::string> usednames;
	for (auto const &use : m_track_textures[Kind])
	{
		usednames.insert(use.name);
		if (matches(lowered(use.name)))
			used.push_back(&use);
	}
	std::string const suggested{Kind == 0 ? "rail" : "tpb"};
	std::vector<texture_index_entry const *> files;
	std::size_t more{0};
	for (auto const &entry : m_track_texture_index)
	{
		if (usednames.count(entry.name) > 0 || (words.empty() ? entry.lowered.find(suggested) == std::string::npos : false == matches(entry.lowered)))
			continue;
		if (files.size() < kPickerFiles)
			files.push_back(&entry);
		else
			++more;
	}
	std::string const first{false == used.empty() ? used.front()->name : false == files.empty() ? files.front()->name : std::string{}};
	if (entered && false == words.empty())
	{
		// the first match when there's one, the path as typed otherwise
		Chosen = first.empty() ? typed.substr(typed.find_first_not_of(' ')) : first;
		return true;
	}

	// large: the tile under the mouse, or what Enter would choose, or the texture of the tracks
	auto const shown{false == m_track_texture_hovered.empty() ? m_track_texture_hovered : false == words.empty() ? first : Mixed ? std::string{} : Current};
	m_track_texture_hovered.clear();
	auto const big{std::round(ImGui::GetFontSize() * 10.0f)};
	auto const corner{ImGui::GetCursorScreenPos()};
	if (auto const image{shown.empty() || shown == "none" ? 0 : track_texture_image(Kind, shown)}; image != 0)
		ImGui::Image(static_cast<ImTextureID>(image), ImVec2(big, big), ImVec2(0, 1), ImVec2(1, 0));
	else
	{
		ImGui::Dummy(ImVec2(big, big));
		ImGui::GetWindowDrawList()->AddRect(corner, ImVec2(corner.x + big, corner.y + big), ImGui::GetColorU32(ImGuiCol_Border));
	}
	ImGui::SameLine();
	ImGui::BeginGroup();
	ImGui::PushTextWrapPos(0.0f);
	if (shown.empty())
		ImGui::TextDisabled("%s", words.empty() ? STR_C("The selected tracks have different ones") : STR_C("None of them: Enter sets the path as typed"));
	else
	{
		ImGui::TextUnformatted(shown == "none" ? STR_C("No texture") : shown.c_str());
		auto const use{std::find_if(m_track_textures[Kind].begin(), m_track_textures[Kind].end(), [&](track_texture_use const &Use) { return Use.name == shown; })};
		if (use != m_track_textures[Kind].end())
			ImGui::TextDisabled(STR_C("In this scenery: %s"), tracks_count(static_cast<std::size_t>(use->count)).c_str());
		else if (shown != "none")
			ImGui::TextDisabled("%s", STR_C("From the textures folder, on no track yet"));
		if (false == Mixed && shown == Current)
			ImGui::TextDisabled("%s", STR_C("On the selected tracks now"));
		if (false == words.empty() && shown == first)
			ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), "%s", STR_C("Enter chooses this one"));
	}
	ImGui::PopTextWrapPos();
	ImGui::EndGroup();

	// the tiles, a click chooses
	auto const tile{std::round(ImGui::GetFontSize() * 6.0f)};
	auto const label{ImGui::GetTextLineHeight() * 2.0f};
	auto const spacing{ImGui::GetStyle().ItemSpacing};
	bool picked{false};
	ImGui::BeginChild("##tiles", ImVec2(0.0f, std::round((tile + label + spacing.y) * 2.5f)), ImGuiChildFlags_Borders);
	auto const columns{std::max(1, static_cast<int>((ImGui::GetContentRegionAvail().x + spacing.x) / (tile + spacing.x)))};
	int column{0};
	auto const draw_tile = [&](std::string const &Name, int const Count) {
		if (column++ % columns != 0)
			ImGui::SameLine();
		ImGui::PushID(Name.c_str());
		auto const low{ImGui::GetCursorScreenPos()};
		if (ImGui::Selectable("##tile", false == Mixed && Name == Current, ImGuiSelectableFlags_None, ImVec2(tile, tile + label)))
		{
			Chosen = Name;
			picked = true;
		}
		if (ImGui::IsItemHovered())
			m_track_texture_hovered = Name;
		if (ImGui::IsItemVisible())
		{
			auto *drawlist{ImGui::GetWindowDrawList()};
			if (auto const image{track_texture_image(Kind, Name)}; image != 0)
				drawlist->AddImage(static_cast<ImTextureID>(image), low, ImVec2(low.x + tile, low.y + tile), ImVec2(0, 1), ImVec2(1, 0));
			else
				drawlist->AddRect(low, ImVec2(low.x + tile, low.y + tile), ImGui::GetColorU32(ImGuiCol_Border));
			auto const slash{Name.find_last_of('/')};
			auto const [line1, line2]{two_lines(slash == std::string::npos ? Name : Name.substr(slash + 1), tile)};
			drawlist->AddText(ImVec2(low.x, low.y + tile), ImGui::GetColorU32(ImGuiCol_Text), line1.c_str());
			drawlist->AddText(ImVec2(low.x, low.y + tile + ImGui::GetTextLineHeight()), ImGui::GetColorU32(ImGuiCol_Text), line2.c_str());
			if (Count > 0)
			{
				// how many tracks of the scenery have it
				auto const badge{format("%d", Count)};
				auto const size{ImGui::CalcTextSize(badge.c_str())};
				drawlist->AddRectFilled(low, ImVec2(low.x + size.x + 6.0f, low.y + size.y + 2.0f), IM_COL32(0, 0, 0, 190), 3.0f);
				drawlist->AddText(ImVec2(low.x + 3.0f, low.y + 1.0f), IM_COL32(255, 220, 120, 255), badge.c_str());
			}
		}
		ImGui::PopID();
	};
	if (false == used.empty())
	{
		ImGui::TextDisabled("%s", STR_C("On the tracks of this scenery:"));
		for (auto const *use : used)
			draw_tile(use->name, use->count);
	}
	if (false == files.empty())
	{
		ImGui::TextDisabled("%s", words.empty() ? STR_C("Suggested from the textures folder:") : STR_C("In the textures folder:"));
		column = 0;
		for (auto const *entry : files)
			draw_tile(entry->name, 0);
	}
	if (more > 0)
		ImGui::TextDisabled(STR_C("and %d more: type more of the name"), static_cast<int>(more));
	if (used.empty() && files.empty())
		ImGui::TextDisabled("%s", STR_C("None of them: Enter sets the path as typed"));
	ImGui::EndChild();
	if (ImGui::Selectable(STR_C("No texture"), false == Mixed && Current == "none"))
	{
		Chosen = "none";
		picked = true;
	}
	return picked;
}

void editor_mode::render_track_properties(std::vector<TTrack *> const &Tracks)
{
	auto const &list{properties()};
	auto &cache{m_track_values};
	// the values are gathered again when the selection or the tracks change, for a large selection not on every frame
	std::size_t signature{Tracks.size()};
	for (auto const *track : Tracks)
		signature = signature * 31 + reinterpret_cast<std::uintptr_t>(track);
	auto const stale{cache.signature != signature || cache.generation_seen != cache.generation || cache.history != m_history.size() || Tracks.size() <= kLargeSelection || ImGui::GetFrameCount() - cache.frame > 15};
	if (stale)
	{
		cache.signature = signature;
		cache.generation_seen = cache.generation;
		cache.history = m_history.size();
		cache.frame = ImGui::GetFrameCount();
		cache.values.assign(list.size(), {});
		cache.applies.assign(list.size(), 0);
		for (std::size_t i = 0; i < list.size(); ++i)
		{
			auto values{values_of(list[i], Tracks)};
			std::size_t applies{0};
			for (auto const &value : values)
				applies += static_cast<std::size_t>(value.second);
			cache.applies[i] = applies;
			cache.values[i] = std::move(values);
		}
	}

	auto const row_label = [&](std::size_t const Index) {
		auto const &property{list[Index]};
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		auto const partial{cache.applies[Index] < Tracks.size()};
		auto const mixed{cache.values[Index].size() > 1};
		ImGui::PushTextWrapPos(0.0f);
		if (mixed)
			ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.45f, 1.0f), "%s", STR_C(property.label));
		else
			ImGui::TextUnformatted(STR_C(property.label));
		ImGui::PopTextWrapPos();
		if (partial)
		{
			ImGui::SameLine(0.0f, 4.0f);
			ImGui::TextDisabled("(%d)", static_cast<int>(cache.applies[Index]));
		}
		row_tooltip(property, cache.values[Index], cache.applies[Index], Tracks.size());
		ImGui::TableSetColumnIndex(1);
		ImGui::SetNextItemWidth(-FLT_MIN);
	};

	auto const text_row = [&](std::size_t const Index) {
		auto const &property{list[Index]};
		auto const &values{cache.values[Index]};
		auto &edit{m_track_field};
		auto const editing{edit.property == static_cast<int>(Index)};
		std::array<char, 256> shown{};
		if (values.size() == 1)
			std::snprintf(shown.data(), shown.size(), "%s", values.front().first == "none" ? "" : values.front().first.c_str());
		auto *buffer{editing ? edit.text.data() : shown.data()};
		auto const hint{values.size() > 1 ? format(STR_C("different (%d)"), static_cast<int>(values.size())) : property.empty != nullptr ? std::string{STR_C(property.empty)} : std::string{}};
		ImGui::InputTextWithHint("##value", hint.c_str(), buffer, shown.size(), property.kind == field::number ? ImGuiInputTextFlags_CharsDecimal : 0);
		if (ImGui::IsItemActivated())
		{
			edit.property = static_cast<int>(Index);
			edit.tracks = Tracks;
			edit.initial = buffer;
			std::snprintf(edit.text.data(), edit.text.size(), "%s", buffer);
		}
		if (edit.property == static_cast<int>(Index))
			edit.frame = ImGui::GetFrameCount();
		row_tooltip(property, values, cache.applies[Index], Tracks.size());
		if (ImGui::IsItemDeactivated() && edit.property == static_cast<int>(Index))
		{
			auto const tracks{edit.tracks};
			std::string const typed{edit.text.data()};
			auto const initial{edit.initial};
			edit = {};
			if (typed != initial)
				track_property_apply(static_cast<int>(Index), tracks, typed);
		}
	};

	auto const choice_row = [&](std::size_t const Index) {
		auto const &property{list[Index]};
		auto const &values{cache.values[Index]};
		auto const current{values.size() == 1 ? std::atoi(values.front().first.c_str()) : -1};
		auto const preview{current >= 0 && current < static_cast<int>(property.choices.size()) ? std::string{STR_C(property.choices[current])} : format(STR_C("different (%d)"), static_cast<int>(values.size()))};
		if (ImGui::BeginCombo("##value", preview.c_str()))
		{
			for (int i = 0; i < static_cast<int>(property.choices.size()); ++i)
				if (ImGui::Selectable(STR_C(property.choices[i]), i == current))
					track_property_apply(static_cast<int>(Index), Tracks, std::to_string(i));
			ImGui::EndCombo();
		}
		row_tooltip(property, values, cache.applies[Index], Tracks.size());
	};

	auto const check_row = [&](std::size_t const Index) {
		auto const &property{list[Index]};
		auto const &values{cache.values[Index]};
		auto const mixed{values.size() > 1};
		bool value{false == mixed && values.front().first == "1"};
		if (mixed)
			ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
		if (ImGui::Checkbox("##value", &value))
			track_property_apply(static_cast<int>(Index), Tracks, value ? "1" : "0");
		if (mixed)
			ImGui::PopItemFlag();
		row_tooltip(property, values, cache.applies[Index], Tracks.size());
	};

	auto const texture_row = [&](std::size_t const Index) {
		auto const &property{list[Index]};
		auto const &values{cache.values[Index]};
		auto const mixed{values.size() > 1};
		auto const common{mixed ? std::string{} : values.front().first};
		auto const preview{mixed ? format(STR_C("different (%d)"), static_cast<int>(values.size())) : common == "none" ? std::string{STR_C(property.empty)} : common};
		auto const side{ImGui::GetFrameHeight()};
		if (false == mixed)
		{
			if (auto const image{material_image(editor_track::fetch_material(common))}; image != 0)
			{
				ImGui::Image(static_cast<ImTextureID>(image), ImVec2(side, side), ImVec2(0, 1), ImVec2(1, 0));
				if (ImGui::IsItemHovered())
				{
					auto const big{std::round(ImGui::GetFontSize() * 14.0f)};
					ImGui::BeginTooltip();
					ImGui::Image(static_cast<ImTextureID>(image), ImVec2(big, big), ImVec2(0, 1), ImVec2(1, 0));
					ImGui::TextUnformatted(common.c_str());
					ImGui::EndTooltip();
				}
				ImGui::SameLine(0.0f, 4.0f);
			}
		}
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::SetNextWindowSizeConstraints(ImVec2(ImGui::GetFontSize() * 36.0f, 0.0f), ImVec2(FLT_MAX, FLT_MAX));
		if (ImGui::BeginCombo("##value", preview.c_str(), ImGuiComboFlags_HeightLargest))
		{
			if (std::string chosen; track_texture_picker(property.texture, common, mixed, chosen))
			{
				track_property_apply(static_cast<int>(Index), Tracks, chosen);
				ImGui::CloseCurrentPopup();
			}
			ImGui::EndCombo();
		}
		row_tooltip(property, values, cache.applies[Index], Tracks.size());
		if (std::string dropped; material_drop(dropped))
			track_property_apply(static_cast<int>(Index), Tracks, dropped);
	};

	auto const any_in = [&](int const Part, bool const More) {
		for (std::size_t i = 0; i < list.size(); ++i)
			if (list[i].part == Part && list[i].more == More && cache.applies[i] > 0)
				return true;
		return false;
	};
	auto const rows = [&](int const Part, bool const More) {
		if (false == any_in(Part, More))
			return;
		if (false == ImGui::BeginTable(format("##trackprops%d%d", Part, More ? 1 : 0).c_str(), 2, ImGuiTableFlags_SizingStretchProp))
			return;
		ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthStretch, 0.45f);
		ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 0.55f);
		for (std::size_t i = 0; i < list.size(); ++i)
		{
			if (list[i].part != Part || list[i].more != More || cache.applies[i] == 0)
				continue;
			ImGui::PushID(list[i].key);
			row_label(i);
			switch (list[i].kind)
			{
			case field::choice: choice_row(i); break;
			case field::check: check_row(i); break;
			case field::texture: texture_row(i); break;
			default: text_row(i); break;
			}
			ImGui::PopID();
		}
		ImGui::EndTable();
	};
	auto const more = [&](char const *Id, int const Part) {
		if (false == any_in(Part, true))
			return;
		ImGui::PushID(Id);
		if (ImGui::TreeNode(STR_C("More settings")))
		{
			rows(Part, true);
			ImGui::TreePop();
		}
		ImGui::PopID();
	};

	if (ImGui::CollapsingHeader(STR_C("Traffic"), ImGuiTreeNodeFlags_DefaultOpen))
	{
		rows(section::traffic, false);
		more("traffic", section::traffic);
	}
	if ((any_in(section::look, false) || any_in(section::look, true) || any_in(section::look_bed, false) || any_in(section::look_sleepers, false)) && ImGui::CollapsingHeader(STR_C("Look"), ImGuiTreeNodeFlags_DefaultOpen))
	{
		rows(section::look, false);
		more("look", section::look);
		if (any_in(section::look_bed, false) && ImGui::TreeNode(STR_C("Shape of the trackbed")))
		{
			rows(section::look_bed, false);
			ImGui::TreePop();
		}
		if (any_in(section::look_sleepers, false) && ImGui::TreeNode(STR_C("Sleeper models")))
		{
			rows(section::look_sleepers, false);
			render_track_presets(Tracks);
			ImGui::TreePop();
		}
	}
	if (ImGui::CollapsingHeader(STR_C("Ride and sound")))
		rows(section::ride, false);
}

void editor_mode::copy_track_style(TTrack const &Track)
{
	auto &style{m_track_style};
	style.from = name_of(Track);
	style.values = values_of_track(Track);
	ui()->set_status(format(STR_C("Properties of %s copied: Ctrl+Shift+V pastes them on the selected tracks"), style.from.c_str()), false);
}

std::size_t editor_mode::paste_track_style(std::vector<TTrack *> const &Tracks)
{
	auto const &style{m_track_style};
	if (style.values.empty() || Tracks.empty())
		return 0;
	auto const changed{apply_track_values(style.values, style.groups, Tracks, "Pasted properties")};
	if (false == changed.has_value())
	{
		ui()->set_status(STR_C("Nothing chosen to paste"), true);
		return 0;
	}
	if (Tracks.size() == 1)
		ui()->set_status(*changed > 0 ? format(STR_C("Properties of %s pasted, Ctrl+Z takes them back"), style.from.c_str()) : std::string{STR_C("The track has these properties already")}, false);
	return *changed;
}

std::optional<std::size_t> editor_mode::apply_track_values(std::vector<std::pair<std::string, std::string>> const &Values, track_groups const &Groups, std::vector<TTrack *> const &Tracks, char const *What)
{
	static_assert(group_count == std::tuple_size_v<track_groups>, "a choice for each group of the pasted properties");
	auto const &list{properties()};
	std::vector<std::pair<track_property const *, std::string>> pasted;
	bool rebuild{false};
	for (auto const late : {false, true})
		for (auto const &[key, value] : Values)
		{
			auto const index{track_property_index(key)};
			if (index < 0)
				continue;
			auto const &property{list[index]};
			if (property.late != late || false == Groups[property.group])
				continue;
			pasted.emplace_back(&property, value);
			rebuild |= property.rebuild;
		}
	if (pasted.empty())
		return std::nullopt;
	return tracks_apply(Tracks, [&](TTrack &Track) {
		bool any{false};
		for (auto const &[property, value] : pasted)
			if (property->applies(Track))
				any |= property->set(Track, value);
		return any;
	}, rebuild, What);
}

std::size_t editor_mode::apply_track_preset(int const Id, std::vector<TTrack *> const &Tracks)
{
	auto const *preset{EditorTrackPresets.find(Id)};
	if (preset == nullptr || Tracks.empty())
		return 0;
	auto const name{preset->name};
	auto const changed{apply_track_values(preset->values, groups_of(*preset), Tracks, "Applied a track preset")};
	if (false == changed.has_value())
	{
		ui()->set_status(format(STR_C("The preset \"%s\" overwrites nothing: choose what it overwrites"), name.c_str()), true);
		return 0;
	}
	if (*changed > 0)
		ui()->set_status(format(STR_P("Preset \"%s\" applied on %d track, Ctrl+Z takes it back", "Preset \"%s\" applied on %d tracks, Ctrl+Z takes it back", static_cast<long long>(*changed)), name.c_str(), static_cast<int>(*changed)), false);
	else
		ui()->set_status(STR_C("The tracks have the properties of the preset already"), false);
	return *changed;
}

void editor_mode::render_track_presets(std::vector<TTrack *> const &Tracks)
{
	if (Tracks.empty() || false == ImGui::TreeNode(STR_C("Presets")))
		return;
	auto &panel{m_track_presets};
	auto const &presets{EditorTrackPresets.presets()};
	if (EditorTrackPresets.find(panel.selected) == nullptr)
		panel.selected = presets.empty() ? 0 : presets.front().id;
	auto const single{Tracks.size() == 1};
	auto const *selected{EditorTrackPresets.find(panel.selected)};

	ImGui::SetNextItemWidth(-FLT_MIN);
	if (ImGui::BeginCombo("##trackpreset", selected != nullptr ? selected->name.c_str() : STR_C("No presets yet")))
	{
		for (auto const &preset : presets)
		{
			if (ImGui::Selectable(preset.name.c_str(), preset.id == panel.selected))
				panel.selected = preset.id;
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", preset_summary(preset).c_str());
		}
		ImGui::EndCombo();
	}
	if (selected != nullptr && ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", preset_summary(*selected).c_str());

	auto const width{button_width(2)};
	ImGui::BeginDisabled(selected == nullptr);
	if (ImGui::Button(format(STR_P("Apply on %d track", "Apply on %d tracks", static_cast<long long>(Tracks.size())), static_cast<int>(Tracks.size())).c_str(), ImVec2(width, 0.0f)))
		apply_track_preset(panel.selected, Tracks);
	ImGui::EndDisabled();
	item_tooltip("Puts the values of the preset on the selected tracks, of what it overwrites; Ctrl+Z takes them back.\nTools in the menu applies the presets too");
	ImGui::SameLine();
	ImGui::BeginDisabled(false == single);
	if (ImGui::Button(STR_C("New from the track"), ImVec2(width, 0.0f)))
	{
		// rails, trackbed, its shape, sleepers, frogs and sleeper placement at first, the look of the track
		std::array<bool, group_count> groups{};
		for (auto const group : {group::rails, group::trackbed, group::trackbed_shape, group::sleepers, group::frogs, group::sleeper_placement})
			groups[group] = true;
		panel.selected = EditorTrackPresets.create(STR_C("New preset"), name_of(*Tracks.front()), values_of_track(*Tracks.front()), group_keys(groups));
		panel.nameid = 0;
		ui()->set_status(format(STR_C("Properties of %s saved as a preset: give it a name and choose what it overwrites"), name_of(*Tracks.front()).c_str()), false);
	}
	ImGui::EndDisabled();
	if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
		ImGui::SetTooltip("%s", STR_C(single ? "Saves the properties of this track as a new preset" : "Select one track to save its properties as a preset"));

	// pointers to the presets don't outlive a change of their list
	selected = EditorTrackPresets.find(panel.selected);
	if (selected == nullptr)
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("A preset keeps the properties of a track: its rails, trackbed, sleepers... to put them on other tracks,\nwith the choice of what it overwrites"));
		ImGui::PopTextWrapPos();
		ImGui::TreePop();
		return;
	}
	auto const id{selected->id};

	// the name
	if (panel.nameid != id)
	{
		std::snprintf(panel.name, sizeof(panel.name), "%s", selected->name.c_str());
		panel.nameid = id;
	}
	ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(STR_C("Rename")).x - ImGui::GetStyle().FramePadding.x * 2 - ImGui::GetStyle().ItemSpacing.x);
	auto const entered{ImGui::InputText("##presetname", panel.name, IM_ARRAYSIZE(panel.name), ImGuiInputTextFlags_EnterReturnsTrue)};
	ImGui::PopItemWidth();
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Rename")) || entered)
	{
		EditorTrackPresets.rename(id, panel.name);
		panel.nameid = 0; // reload, the name could have been adjusted to stay unique
		selected = EditorTrackPresets.find(id);
	}

	// what it overwrites, of the groups it has values of
	ImGui::TextDisabled("%s", STR_C("Overwrites:"));
	// its own id scope: the group checkboxes share their labels with the sections of the properties
	ImGui::PushID("presetgroups");
	auto const available{groups_in(selected->values)};
	auto groups{groups_of(*selected)};
	auto changed{false};
	int column{0};
	auto const half{ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x * 0.5f};
	for (int group = 0; group < group_count; ++group)
	{
		if (false == available[group])
			continue;
		if (column % 2 == 1)
			ImGui::SameLine(half);
		changed |= ImGui::Checkbox(STR_C(kGroupLabels[group]), &groups[group]);
		++column;
	}
	ImGui::PopID();
	if (changed)
	{
		// the groups it has no values of are kept as they were, for a track they're taken again from
		auto kept{groups_of(*selected)};
		for (int group = 0; group < group_count; ++group)
			if (available[group])
				kept[group] = groups[group];
		EditorTrackPresets.set_groups(id, group_keys(kept));
		selected = EditorTrackPresets.find(id);
	}

	ImGui::BeginDisabled(false == single);
	if (ImGui::Button(STR_C("Update from the track"), ImVec2(width, 0.0f)))
	{
		EditorTrackPresets.update(id, name_of(*Tracks.front()), values_of_track(*Tracks.front()));
		ui()->set_status(format(STR_C("Preset \"%s\" takes the properties of %s now"), selected->name.c_str(), name_of(*Tracks.front()).c_str()), false);
	}
	ImGui::EndDisabled();
	if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
		ImGui::SetTooltip("%s", STR_C(single ? "The preset takes the properties of this track in place of the ones it had; what it overwrites stays" : "Select one track to take its properties into the preset"));
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Delete"), ImVec2(width, 0.0f)))
		ImGui::OpenPopup("##deletepreset");
	if (ImGui::BeginPopup("##deletepreset"))
	{
		ImGui::Text(STR_C("Delete preset \"%s\"?"), selected->name.c_str());
		if (ImGui::Button(STR_C("Delete##confirm")))
		{
			EditorTrackPresets.remove(id);
			panel.selected = 0;
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Cancel")))
			ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
	}
	ImGui::TreePop();
}

void editor_mode::render_track_presets_menu()
{
	if (false == ImGui::BeginMenu(STR_C("Track presets")))
		return;
	auto const &presets{EditorTrackPresets.presets()};
	if (presets.empty())
		ImGui::TextDisabled("%s", STR_C("No presets: save one in the inspector of a track, Look, Sleeper models, Presets"));
	auto const tracks{selected_tracks()};
	int chosen{0};
	for (auto const &preset : presets)
	{
		if (ImGui::MenuItem(preset.name.c_str(), nullptr, false, false == tracks.empty()))
			chosen = preset.id;
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", tracks.empty() ? STR_C("Select the tracks to apply the preset on first") : (format(STR_P("Applies the preset on %d selected track", "Applies the preset on %d selected tracks", static_cast<long long>(tracks.size())), static_cast<int>(tracks.size())) + "\n" + preset_summary(preset)).c_str());
	}
	ImGui::EndMenu();
	if (chosen != 0)
		apply_track_preset(chosen, tracks);
}

void editor_mode::render_track_style(std::vector<TTrack *> const &Tracks)
{
	auto &style{m_track_style};
	auto const single{Tracks.size() == 1};
	if (false == ImGui::CollapsingHeader(format("%s###trackstyle%d", STR_C("Copying properties"), single ? 1 : 2).c_str(), single ? 0 : ImGuiTreeNodeFlags_DefaultOpen))
		return;
	auto const width{button_width(3)};
	ImGui::BeginDisabled(false == single);
	if (ImGui::Button(STR_C("Copy"), ImVec2(width, 0.0f)))
		copy_track_style(*Tracks.front());
	ImGui::EndDisabled();
	if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
		ImGui::SetTooltip("%s", STR_C(single ? "Copies the properties of this track, to paste them on others (Ctrl+Shift+C)" : "Select one track to copy its properties, or take them from a track with the pipette"));
	ImGui::SameLine();
	ImGui::BeginDisabled(style.values.empty());
	// pressed in while its choice is open under the row
	if (style.open)
		ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
	if (ImGui::Button(STR_C("Paste..."), ImVec2(width, 0.0f)))
		style.open = false == style.open;
	if (style.open)
		ImGui::PopStyleColor();
	ImGui::EndDisabled();
	if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
	{
		if (style.values.empty())
			ImGui::SetTooltip("%s", STR_C("Copy the properties of a track first"));
		else
			ImGui::SetTooltip(STR_C("Properties of %s: choose what goes on the selected tracks (Ctrl+Shift+V pastes the same choice at once)"), style.from.c_str());
	}
	ImGui::SameLine();
	if (style.picking)
	{
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.55f, 0.1f, 1.0f));
		if (ImGui::Button(STR_C("Click a track"), ImVec2(width, 0.0f)))
			style.picking = false;
		ImGui::PopStyleColor();
		item_tooltip("Click a track in the view to take its properties; Esc or this button gives up");
	}
	else
	{
		if (ImGui::Button(STR_C("Pipette"), ImVec2(width, 0.0f)))
			style.picking = true;
		item_tooltip("Pipette: the next click on a track in the view takes its properties, to paste them on the selection");
	}

	if (false == style.open || style.values.empty())
		return;
	// its own id scope: the group checkboxes share their labels with the sections of the properties under them
	ImGui::PushID("paste");
	ImGui::Indent();
	ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), STR_C("Paste from %s:"), style.from.c_str());
	std::array<bool, group_count> available{};
	for (auto const &[key, value] : style.values)
		if (auto const index{track_property_index(key)}; index >= 0)
			available[properties()[index].group] = true;
	int column{0};
	auto const half{ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x * 0.5f};
	for (int group = 0; group < group_count; ++group)
	{
		if (false == available[group])
			continue;
		if (column % 2 == 1)
			ImGui::SameLine(half);
		ImGui::Checkbox(STR_C(kGroupLabels[group]), &style.groups[group]);
		++column;
	}
	if (ImGui::Button(format(STR_P("Paste on %d track", "Paste on %d tracks", static_cast<long long>(Tracks.size())), static_cast<int>(Tracks.size())).c_str()))
	{
		paste_track_style(Tracks);
		style.open = false;
	}
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Cancel")))
		style.open = false;
	ImGui::Unindent();
	ImGui::PopID();
	ImGui::Separator();
}

std::vector<TTrack *> editor_mode::similar_tracks(int const Property, std::vector<TTrack *> const &Tracks) const
{
	auto const &property{properties()[Property]};
	std::set<std::string> wanted;
	for (auto const *track : Tracks)
		if (track != nullptr && property.applies(*track))
			wanted.insert(property.get(*track));
	std::vector<TTrack *> found;
	if (wanted.empty())
		return found;
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && rail(*track) && property.applies(*track) && wanted.count(property.get(*track)) > 0)
			found.push_back(track);
	return found;
}

void editor_mode::render_track_set_header(std::vector<TTrack *> const &Tracks)
{
	double length{0.0};
	int switches{0};
	for (auto const *track : Tracks)
	{
		length += track->Length();
		switches += track->eType == tt_Switch ? 1 : 0;
	}
	ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.2f, 1.0f), STR_P("%d track selected, %.1f m", "%d tracks selected, %.1f m", static_cast<long long>(Tracks.size())), static_cast<int>(Tracks.size()), length);
	item_tooltip("What is set below goes to all of them, in one step of the undo");
	ImGui::SameLine();
	if (auto const room{ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(STR_C("Deselect")).x - ImGui::GetStyle().FramePadding.x * 2.0f}; room > 0.0f)
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + room);
	if (ImGui::SmallButton(STR_C("Deselect")))
	{
		select_tracks({});
		return;
	}
	item_tooltip("Clears the selection (Esc)");
	if (switches > 0)
		ImGui::TextDisabled(STR_P("among them %d switch", "among them %d switches", static_cast<long long>(switches)), switches);
	ImGui::TextDisabled("%s", STR_C("What is set below goes to all of them"));

	// the list: hovering a row shows the track in the view, a click flies to it, a double click keeps it alone
	if (ImGui::TreeNode(STR_C("List of the selected tracks")))
	{
		auto const rows{std::min<std::size_t>(Tracks.size(), 10)};
		ImGui::BeginChild("##selectedtracks", ImVec2(0.0f, ImGui::GetFrameHeightWithSpacing() * static_cast<float>(rows) + 8.0f), ImGuiChildFlags_Borders);
		auto const column{ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x * 0.62f};
		TTrack *removed{nullptr};
		TTrack *alone{nullptr};
		ImGuiListClipper clipper;
		clipper.Begin(static_cast<int>(Tracks.size()));
		while (clipper.Step())
			for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i)
			{
				auto *track{Tracks[i]};
				ImGui::PushID(i);
				if (ImGui::SmallButton("x"))
					removed = track;
				item_tooltip("Takes the track out of the selection");
				ImGui::SameLine();
				if (ImGui::Selectable(name_of(*track).c_str(), false, ImGuiSelectableFlags_AllowDoubleClick))
				{
					if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
						alone = track;
					else
						focus_track(*track, 0, bezier{track->m_paths.front()}.point(0.5));
				}
				if (ImGui::IsItemHovered())
				{
					m_track_list_hover = track;
					m_track_list_hover_frame = ImGui::GetFrameCount();
					ImGui::SetTooltip("%s", STR_C("Click: look at it, double click: select only this one"));
				}
				ImGui::SameLine(column);
				ImGui::TextDisabled("%s, %.1f m", STR_C(type_of(*track)), track->Length());
				ImGui::PopID();
			}
		ImGui::EndChild();
		ImGui::TreePop();
		if (removed != nullptr)
			track_set_toggle(*removed);
		else if (alone != nullptr)
			select_tracks({alone});
	}
}

void editor_mode::render_track_select_tools(std::vector<TTrack *> const &Tracks)
{
	auto const single{Tracks.size() == 1};
	auto const open{ImGui::CollapsingHeader(format("%s  (?)###trackselect%d", STR_C("Selection"), single ? 1 : 2).c_str(), single ? 0 : ImGuiTreeNodeFlags_DefaultOpen)};
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Shift+click: add or take out a track\n"
		                              "Ctrl+click: the same\n"
		                              "Drag on the ground: select the tracks in a box, with Shift: add them\n"
		                              "Ctrl+Shift+click: the whole run of joined tracks\n"
		                              "Esc: clear the selection, Del: delete the selected tracks\n"
		                              "Ctrl+Shift+C / Ctrl+Shift+V: copy and paste the properties"));
	if (false == open)
		return;
	auto const width{button_width(2)};
	if (ImGui::Button(STR_C("+ Joined"), ImVec2(width, 0.0f)))
	{
		if (m_track_set.empty() && false == Tracks.empty())
			m_track_set = Tracks;
		track_set_grow();
		select_tracks(m_track_set);
	}
	item_tooltip("Adds the tracks joined to the selection, by the rules of the spread in the tool options");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Alike..."), ImVec2(width, 0.0f)))
		ImGui::OpenPopup("##similar");
	item_tooltip("Selects the tracks of the whole scenery which have the same rails, trackbed, speed... as the selected ones");
	if (ImGui::BeginPopup("##similar"))
	{
		char const *const keys[] = {"rails", "trackbed", "speed", "environment", "circuits", "overhead"};
		char const *const labels[] = {"The same rails", "The same trackbed", "The same speed", "The same environment", "The same track circuits", "The same overhead line"};
		if (ImGui::IsWindowAppearing())
		{
			m_track_similar.clear();
			for (auto const *key : keys)
				m_track_similar.push_back(similar_tracks(track_property_index(key), Tracks));
		}
		for (std::size_t i = 0; i < std::size(keys) && i < m_track_similar.size(); ++i)
		{
			auto const &found{m_track_similar[i]};
			if (ImGui::MenuItem(STR_C(labels[i]), tracks_count(found.size()).c_str(), false, false == found.empty()))
			{
				select_tracks(found);
				ui()->set_status(format(STR_C("%s: %s selected in the scenery"), STR_C(labels[i]), tracks_count(found.size()).c_str()), false);
			}
		}
		ImGui::EndPopup();
	}
}

editor_mode::selftest_result editor_mode::selftest_props(std::string const &Command, std::istringstream &Words)
{
	if (Command == "tracksel")
	{
		std::vector<TTrack *> tracks;
		for (std::string name; Words >> name;)
			if (auto *track{simulation::Paths.find(name)}; track != nullptr)
				tracks.push_back(track);
		select_tracks(tracks);
		WriteLog(format("SELFTEST tracksel: %zu tracks selected", selected_tracks().size()));
		return selftest_result::handled;
	}
	if (Command == "trackprop")
	{
		std::string key;
		Words >> key;
		std::string value;
		std::getline(Words, value);
		value = words_of(value).empty() ? std::string{} : value.substr(value.find_first_not_of(' '));
		auto const index{track_property_index(key)};
		if (index < 0)
		{
			WriteLog("SELFTEST trackprop: no property " + key);
			return selftest_result::handled;
		}
		auto const changed{track_property_apply(index, selected_tracks(), value)};
		WriteLog(format("SELFTEST trackprop %s=\"%s\": %zu changed", key.c_str(), value.c_str(), changed));
		return selftest_result::handled;
	}
	if (Command == "trackprops")
	{
		auto const tracks{selected_tracks()};
		for (auto const &property : properties())
		{
			auto const values{values_of(property, tracks)};
			std::string text;
			for (auto const &[value, count] : values)
				text += format(" [%s]x%d", value.c_str(), count);
			WriteLog(format("SELFTEST trackprops %s:%s", property.key, text.c_str()));
		}
		return selftest_result::handled;
	}
	if (Command == "trackstyle")
	{
		std::string action;
		Words >> action;
		if (action == "copy")
		{
			std::string name;
			Words >> name;
			if (auto *track{name.empty() ? selected_track() : simulation::Paths.find(name)}; track != nullptr)
				copy_track_style(*track);
			WriteLog(format("SELFTEST trackstyle copy: %zu values from %s", m_track_style.values.size(), m_track_style.from.c_str()));
		}
		else if (action == "groups")
		{
			// the groups pasted, as their numbers: 0 speed ... 9 frogs, 10 sleeper placement
			m_track_style.groups.fill(false);
			for (int group; Words >> group;)
				if (group >= 0 && group < group_count)
					m_track_style.groups[group] = true;
		}
		else if (action == "paste")
			WriteLog(format("SELFTEST trackstyle paste: %zu changed", paste_track_style(selected_tracks())));
		else if (action == "panel")
			m_track_style.open = true;
		return selftest_result::handled;
	}
	if (Command == "tracksimilar")
	{
		std::string key;
		Words >> key;
		auto const index{track_property_index(key)};
		if (index >= 0)
			select_tracks(similar_tracks(index, selected_tracks()));
		WriteLog(format("SELFTEST tracksimilar %s: %zu selected", key.c_str(), selected_tracks().size()));
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
