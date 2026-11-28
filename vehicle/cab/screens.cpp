/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others

*/

// the screens of the cab

#include "stdafx.h"
#include "vehicle/Train.h"
#include "vehicle/horn.h"
#include "vehicle/linebreaker.h"
#include "vehicle/holdcoillever.h"
#include "vehicle/compartmentlightsswitch.h"
#include "vehicle/odometer.h"
#include "vehicle/mainctrlact.h"
#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "world/Event.h"
#include "simulation/simulationtime.h"
#include "vehicle/Camera.h"
#include "utilities/Logs.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "utilities/Timer.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "world/mtable.h"
#include "Console.h"
#include "application/application.h"
#include "rendering/renderer.h"
#include "physics/mover.h"
#include <future>
#include <thread>
#include <cmath>
#include <algorithm>
#include <format>
#include "vehicle/cab/internal.h"


void TTrain::update_screens(double dt)
{
	for (auto &screen : m_screens)
	{
		if (screen.updatetimecounter >= 0)
			screen.updatetimecounter += dt;

		if (screen.updatetimecounter <= screen.updatetime)
			continue;

		screen.updatetimecounter = screen.updatetime > 0 ? 0 : -1;

		auto state_dict = GetTrainState(screen.parameters);

		state_dict->insert("touches", *screen.touch_list);
		screen.touch_list->clear();

		Application.request({screen.script, state_dict, screen.rt});
	}
}

void TTrain::update_script_screens()
{
	auto &queue{Application.screens()};
	for (auto &entry : m_scriptscreens)
	{
		if (false == queue.due(*entry.screen))
		{
			continue;
		}
		auto input{GetTrainState(entry.parameters)};
		input->insert("touches", *entry.touch_list);
		entry.touch_list->clear();
		queue.submit(entry.screen, std::move(input), id());
	}
}

bool TTrain::find_screen_surface(std::string const &Target, std::string_view const Kind, TSubModel *&Submodel, ITexture *&Texture) const
{
	Submodel = DynamicObject->mdKabina     ? DynamicObject->mdKabina->GetFromName(Target) :
	           DynamicObject->mdLowPolyInt ? DynamicObject->mdLowPolyInt->GetFromName(Target) :
	                                         nullptr;
	if (Submodel == nullptr)
	{
		WriteLog(std::format("{}: submodel {} not found - Ignoring screen", Kind, Target));
		return false;
	}
	auto const material{Submodel->GetMaterial()};
	if (material <= 0)
	{
		// sub model nie posiada tekstury lub tekstura wymienna - nie obslugiwana
		WriteLog(std::format("{}: invalid texture id {} - Ignoring screen", Kind, material));
		return false;
	}

	Texture = &GfxRenderer->Texture(GfxRenderer->Material(material)->GetTexture(0));
	return true;
}

void TTrain::load_script_screen(cParser &Parser, script_language const Language)
{
	script_screen_entry entry;
	// the block is read whole even when the script screens are off
	entry.config.deserialize(Parser);
	if (entry.config.script.empty())
	{
		return;
	}
	if (auto path{script_screens::screen_script_path(DynamicObject->asBaseDir, entry.config.script)}; path.has_value())
	{
		entry.config.script = std::move(*path);
	}
	else
	{
		ErrorLog(std::format("Bad script screen: \"{}\" leads outside the game's folder", entry.config.script));
		return;
	}

	TSubModel *submodel{nullptr};
	ITexture *texture{nullptr};
	auto &queue{Application.screens()};
	if (false == queue.runs(Language))
	{
		// the screens of the language are off
		return;
	}
	if (entry.config.target != "none" &&
	    false == find_screen_surface(entry.config.target, Language == script_language::lua ? "Lua Screen" : "WASM Screen", submodel, texture))
	{
		return;
	}

	entry.screen = queue.acquire({.kind = script_screen_kind::cab,
	                              .language = Language,
	                              .script = entry.config.script,
	                              .key = std::format("cab:{}:{}:{}", DynamicObject->name(), entry.config.target, entry.config.script),
	                              .texture = texture,
	                              .frames = entry.config.frames});
	if (entry.screen == nullptr)
	{
		return;
	}
	if (texture != nullptr)
	{
		// make the surface static so it doesn't get destroyed by garbage collector if the user spends long time outside cab
		texture->create(true);
	}
	if (false == entry.config.parameters.empty())
	{
		entry.parameters = dictionary_source(entry.config.parameters);
	}
	entry.touch_list = std::make_shared<std::vector<glm::vec2>>();
	if (submodel != nullptr)
	{
		submodel->screen_touch_list = entry.touch_list;
	}
	m_scriptscreens.emplace_back(std::move(entry));
}

const TTrain::screenentry_sequence &TTrain::get_screens()
{
	update_screens(Timer::GetDeltaTime());
	return m_screens;
}
