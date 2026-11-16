/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/pausemenu.h"

#ifdef WITH_RMLUI

#include <algorithm>
#include <array>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <iterator>
#include <string_view>
#include <vector>

#include <RmlUi/Core.h>

#include "application/application.h"
#include "application/inisettings.h"
#include "application/rmluidocument.h"
#include "application/rmluilayer.h"
#include "audio/audiorenderer.h"
#include "input/command.h"
#include "input/keyboardinput.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"
#include "vehicle/Train.h"
#include "world/catenary_span.h"

namespace
{

// staticFiles/ui/pausemenu.rml of the sources, written out by CMake as the bytes of pausemenu_rml
#include "pausemenu_rml.inc"

// the levels of the screen space reflections, as the menu names them and as eu07.ini does
constexpr std::array ssr_names{STRN("off"), STRN("low"), STRN("medium"), STRN("high")};
constexpr std::array ssr_keys{"no", "low", "medium", "high"};

// a key of the controls, as the page of the controls lists it
struct binding_row
{
	Rml::String action;
	Rml::String key;
};

// the values of the data model of the document, see staticFiles/ui/pausemenu.rml; the settings are bound to the globals
struct menu_values
{
	bool shown{false};
	Rml::String page{"main"};
	Rml::String save_status;
	bool radio_shown{false};
	std::vector<binding_row> bindings;
};

std::size_t ssr_level()
{
	return static_cast<std::size_t>(std::clamp(Global.gfx_ssr, 0, 3));
}

float radio_volume()
{
	return simulation::Train != nullptr ? simulation::Train->get_radiovolume() : Global.DefaultRadioVolume;
}

// the radio of the vehicle driven, and of the vehicles entered later
void set_radio_volume(float const Volume)
{
	Global.DefaultRadioVolume = std::clamp(Volume, 0.f, 1.f);
	if (simulation::Train == nullptr)
		return;
	command_relay relay;
	relay.post(user_command::radiovolumeset, Global.DefaultRadioVolume, 0.0, GLFW_PRESS, 0);
}

// a volume of the sounds, which the sources take up again
void set_volume(float &Volume, float const Value)
{
	Volume = std::clamp(Value, 0.f, 1.f);
	audio::event_volume_change = true;
}

void bind_float(Rml::DataModelConstructor &Constructor, Rml::String const &Name, std::function<float()> const &Get, std::function<void(float)> const &Set)
{
	Constructor.BindFunc(
	    Name, [Get](Rml::Variant &Value) { Value = Get(); }, [Set](Rml::Variant const &Value) { Set(Value.Get<float>()); });
}

void bind_int(Rml::DataModelConstructor &Constructor, Rml::String const &Name, std::function<int()> const &Get, std::function<void(int)> const &Set)
{
	Constructor.BindFunc(
	    Name, [Get](Rml::Variant &Value) { Value = Get(); }, [Set](Rml::Variant const &Value) { Set(Value.Get<int>()); });
}

// the settings of the debug panel (Settings), which take effect at once
void bind_settings(Rml::DataModelConstructor &Constructor)
{
	bind_int(Constructor, "ssr", [] { return Global.gfx_ssr; }, [](int const Level) { Global.gfx_ssr = std::clamp(Level, 0, 3); });
	Constructor.BindFunc("ssr_name", [](Rml::Variant &Value) { Value = Translations.lookup_s(ssr_names[ssr_level()]); });
	bind_int(
	    Constructor, "shadow_ranks", [] { return Global.gfx_shadow_rank_cutoff; }, [](int const Ranks) { Global.gfx_shadow_rank_cutoff = std::clamp(Ranks, 1, 3); });
	// shown as a positive angle, as eu07.ini takes it
	bind_float(
	    Constructor, "shadow_angle", [] { return -Global.gfx_shadow_angle_min; },
	    [](float const Angle) { Global.gfx_shadow_angle_min = -std::clamp(quantize(Angle, 0.05f), 0.2f, 1.f); });
	bind_float(
	    Constructor, "catenary_range", [] { return CatenarySettings.solid_range; },
	    [](float const Range) { CatenarySettings.solid_range = std::clamp(quantize(Range, 10.f), 0.f, 2000.f); });
	bind_float(Constructor, "volume", [] { return Global.AudioVolume; }, [](float const Volume) { Global.AudioVolume = std::clamp(Volume, 0.f, 2.f); });
	bind_float(Constructor, "volume_vehicle", [] { return Global.VehicleVolume; }, [](float const Volume) { set_volume(Global.VehicleVolume, Volume); });
	bind_float(
	    Constructor, "volume_positional", [] { return Global.EnvironmentPositionalVolume; },
	    [](float const Volume) { set_volume(Global.EnvironmentPositionalVolume, Volume); });
	bind_float(
	    Constructor, "volume_ambient", [] { return Global.EnvironmentAmbientVolume; },
	    [](float const Volume) { set_volume(Global.EnvironmentAmbientVolume, Volume); });
	bind_float(Constructor, "radio", &radio_volume, &set_radio_volume);
}

// the settings of the menu as eu07.ini holds them
std::vector<ini_settings::value> settings_values()
{
	std::vector<ini_settings::value> values;
	values.emplace_back("gfx.ssr", ssr_keys[ssr_level()]);
	values.emplace_back("gfx.shadow.rank.cutoff", std::to_string(Global.gfx_shadow_rank_cutoff));
	values.emplace_back("gfx.shadow.angle.min", std::format("{:.2f}", -Global.gfx_shadow_angle_min));
	values.emplace_back("catenary.solidrange", std::format("{:.0f}", CatenarySettings.solid_range));
	values.emplace_back("sound.volume", std::format("{:.2f}", Global.AudioVolume));
	values.emplace_back("sound.volume.radio", std::format("{:.2f}", radio_volume()));
	values.emplace_back("sound.volume.vehicle", std::format("{:.2f}", Global.VehicleVolume));
	values.emplace_back("sound.volume.positional", std::format("{:.2f}", Global.EnvironmentPositionalVolume));
	values.emplace_back("sound.volume.ambient", std::format("{:.2f}", Global.EnvironmentAmbientVolume));
	return values;
}

// the eu07.ini the game reads: the one of the profile of the user, or the one of the game folder when the profile has none
std::filesystem::path settings_file()
{
	auto const profile{user_config_path("eu07.ini")};
	if (false == profile.empty() && std::filesystem::exists(profile))
		return profile;
	return std::filesystem::path{"eu07.ini"};
}

std::string path_text(std::filesystem::path const &Path)
{
	auto const text{Path.u8string()};
	return std::string{reinterpret_cast<char const *>(text.data()), text.size()};
}

// the settings written into eu07.ini, the rest of the file left as it was. returns: what the menu tells of it
Rml::String save_settings()
{
	auto const path{settings_file()};
	std::string text;
	{
		std::ifstream input{path, std::ios::binary};
		if (false == input.is_open())
		{
			ErrorLog("settings: can't read " + path_text(path));
			return STR("Can't save the settings to") + " " + path_text(path);
		}
		text.assign(std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{});
	}
	// written aside and put in place of the file, which a failed write leaves whole
	auto temporary{path};
	temporary += ".tmp";
	{
		std::ofstream output{temporary, std::ios::binary | std::ios::trunc};
		output << ini_settings::with_values(text, settings_values());
		// a failed write may show only when the file is closed
		output.close();
		if (output.fail())
		{
			ErrorLog("settings: can't write " + path_text(temporary));
			return STR("Can't save the settings to") + " " + path_text(path);
		}
	}
	std::error_code error;
	std::filesystem::rename(temporary, path, error);
	if (error)
	{
		ErrorLog("settings: can't replace " + path_text(path) + ": " + error.message());
		return STR("Can't save the settings to") + " " + path_text(path);
	}
	WriteLog("settings: saved to " + path_text(path));
	return STR("Settings saved to") + " " + path_text(path);
}

// the keys of the controls of the driver which have one, with their descriptions of eu07_input-keyboard.ini (in Windows-1250 or
// UTF-8), or the names of the commands without one
std::vector<binding_row> binding_rows(keyboard_input const &Keyboard)
{
	std::vector<binding_row> rows;
	for (auto const &[command, setup] : Keyboard.bindings())
	{
		auto key{Keyboard.binding_hint(command)};
		if (key.empty())
			continue;
		auto description{std::get<std::string>(setup)};
		if (description.empty())
			description = simulation::Commands_descriptions[static_cast<std::size_t>(command)].name;
		else if (false == ini_settings::is_utf8(description))
			description = win1250_to_utf8(description);
		rows.emplace_back(std::move(description), std::move(key));
	}
	return rows;
}

void resume()
{
	command_relay relay;
	relay.post(user_command::pausetoggle, 0.0, 0.0, GLFW_RELEASE, 0);
}

// a page of the menu shown: main, settings, controls or quit
void show_page(menu_values &Values, keyboard_input const &Keyboard, Rml::String const &Page)
{
	Values.page = Page;
	Values.save_status.clear();
	if (Values.page == "controls")
		Values.bindings = binding_rows(Keyboard);
}

void bind_menu(Rml::DataModelConstructor &Constructor, menu_values &Values, keyboard_input const &Keyboard)
{
	if (auto row{Constructor.RegisterStruct<binding_row>()})
	{
		row.RegisterMember("action", &binding_row::action);
		row.RegisterMember("key", &binding_row::key);
	}
	Constructor.RegisterArray<std::vector<binding_row>>();
	Constructor.Bind("shown", &Values.shown);
	Constructor.Bind("page", &Values.page);
	Constructor.Bind("save_status", &Values.save_status);
	Constructor.Bind("radio_shown", &Values.radio_shown);
	Constructor.Bind("bindings", &Values.bindings);
	bind_settings(Constructor);
	Constructor.BindEventCallback("resume", [](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &) { resume(); });
	Constructor.BindEventCallback("quit", [](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &) { Application.queue_quit(false); });
	Constructor.BindEventCallback("save", [values = &Values](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &) { values->save_status = save_settings(); });
	Constructor.BindEventCallback("show", [values = &Values, keyboard = &Keyboard](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &Arguments) {
		if (false == Arguments.empty())
			show_page(*values, *keyboard, Arguments.front().Get<Rml::String>());
	});
}

} // namespace

struct pause_menu::state
{
	menu_values values;
	rmlui::document document;
};

pause_menu::pause_menu(keyboard_input const &Keyboard) : m_keyboard(&Keyboard) {}

pause_menu::~pause_menu() = default;

bool pause_menu::open()
{
	auto opened{std::make_unique<state>()};
	auto const bound = [values = &opened->values, keyboard = m_keyboard](Rml::DataModelConstructor &Constructor) { bind_menu(Constructor, *values, *keyboard); };
	if (false == opened->document.open("pause_menu", "ui/pausemenu.rml", pausemenu_rml, bound))
		return false;
	m_state = std::move(opened);
	return true;
}

bool pause_menu::update(bool const Paused)
{
	if (m_failed || false == rmlui::active())
		return false;
	if (m_state == nullptr)
	{
		// loaded when it's needed
		if (false == Paused)
			return true;
		if (false == open())
		{
			m_failed = true;
			return false;
		}
	}
	auto &values{m_state->values};
	if (Paused != values.shown)
	{
		// each pause starts at the first page
		values.shown = Paused;
		show_page(values, *m_keyboard, "main");
		if (Paused)
			m_state->document.show(true);
		else
			m_state->document.hide();
	}
	values.radio_shown = simulation::Train != nullptr;
	if (Paused)
		m_state->document.dirty();
	return true;
}

bool pause_menu::on_key(int const Key, int const Action)
{
	if (Key != GLFW_KEY_ESCAPE || m_state == nullptr)
		return false;
	if (Action == GLFW_RELEASE)
	{
		auto const taken{m_escape};
		m_escape = false;
		return taken;
	}
	auto &values{m_state->values};
	if (false == values.shown || values.page == "main")
		return false;
	show_page(values, *m_keyboard, "main");
	m_escape = true;
	return true;
}

#else

struct pause_menu::state
{
};

pause_menu::pause_menu(keyboard_input const &Keyboard) : m_keyboard(&Keyboard) {}

pause_menu::~pause_menu() = default;

bool pause_menu::open()
{
	return false;
}

bool pause_menu::update(bool)
{
	return false;
}

bool pause_menu::on_key(int, int)
{
	return false;
}

#endif
