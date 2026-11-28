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

// the cab: its gauges, its buttons, the mapping of the controls, the screens it has

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

void control_mapper::clear()
{

	*this = control_mapper();
}

void control_mapper::insert(TGauge const &Gauge, std::string const &Label)
{

	if (Gauge.SubModel != nullptr)
	{
		m_controlnames.try_emplace(Gauge.SubModel, Label);
	}
	if (Gauge.SubModelOn != nullptr)
	{
		m_controlnames.try_emplace(Gauge.SubModelOn, Label);
	}
	// the duplicate submodels of a control (duplicate: yes) work the control as well (eu07/maszyna#527)
	for (auto const &mirror : Gauge.mirrors())
	{
		insert(mirror, Label);
	}

	m_names.emplace(Label);
}

std::string control_mapper::find(TSubModel const *Control) const
{

	auto const lookup = m_controlnames.find(Control);
	if (lookup != m_controlnames.end())
	{
		return lookup->second;
	}
	else
	{
		return "";
	}
}

bool control_mapper::contains(std::string const &Control) const
{

	return m_names.contains(Control);
}

void TTrain::screen_entry::deserialize(cParser &Input)
{

	while (true == deserialize_mapping(Input))
	{
		// all work done by while()
	}
}

bool TTrain::screen_entry::deserialize_mapping(cParser &Input)
{
	// token can be a key or block end
	auto const key{Input.getToken<std::string>(true, "\n\r\t  ,;[]")};

	if (true == key.empty() || key == "}")
	{
		return false;
	}

	if (key == "{")
	{
		script = Input.getToken<std::string>();
	}
	else if (key == "target:")
	{
		target = Input.getToken<std::string>();
	}
	else if (key == "updatetime:")
	{
		updatetime = Input.getToken<int>();
	}
	else if (key == "parameters:")
	{
		parameters = dictionary_source(Input.getToken<std::string>());
	}
	else
	{
		// HACK: we expect this to be true only if the screen entry doesn't start with a { which means legacy configuration format
		target = key;
		script = Input.getToken<std::string>();
		return false;
	}

	return true;
}

void TCab::Load(cParser &Parser)
{
	// NOTE: clearing control tables here is bit of a crutch, imposed by current scheme of loading compartments anew on each cab change
	ggList.clear();
	btList.clear();

	std::string token;
	Parser.getTokens();
	Parser >> token;
	if (token == "cablight:")
	{
		Parser.getTokens(9, false);
		/*
		        Parser
		            >> dimm.r
		            >> dimm.g
		            >> dimm.b
		            >> intlit.r
		            >> intlit.g
		            >> intlit.b
		            >> intlitlow.r
		            >> intlitlow.g
		            >> intlitlow.b;
		*/
		Parser.getTokens();
		Parser >> token;
	}
	CabPos1.x = static_cast<float>(std::stod(token));
	Parser.getTokens(5, false);
	Parser >> CabPos1.y >> CabPos1.z >> CabPos2.x >> CabPos2.y >> CabPos2.z;

	bEnabled = true;
	bOccupied = true;
}

TGauge &TCab::Gauge(int n)
{ // pobranie adresu obiektu aniomowanego ruchem
	/*
	    if (n < 0)
	    { // rezerwacja wolnego
	        ggList[iGauges].Clear();
	        return ggList + iGauges++;
	    }
	    else if (n < iGauges)
	        return ggList + n;
	    return NULL;
	*/
	if (n < 0)
	{
		ggList.emplace_back();
		return ggList.back();
	}
	else
	{
		return ggList[n];
	}
};

TButton &TCab::Button(int n)
{ // pobranie adresu obiektu animowanego wyborem 1 z 2
	/*
	    if (n < 0)
	    { // rezerwacja wolnego
	        return btList + iButtons++;
	    }
	    else if (n < iButtons)
	        return btList + n;
	    return NULL;
	*/
	if (n < 0)
	{
		btList.emplace_back();
		return btList.back();
	}
	else
	{
		return btList[n];
	}
};

void TCab::Update(bool const Power)
{ // odczyt parametrów i ustawienie animacji submodelom
	for (auto &gauge : ggList)
	{
		// animacje izometryczne
		gauge.UpdateValue(); // odczyt parametru i przeliczenie na kąt
		gauge.Update(); // ustawienie animacji
	}

	for (auto &button : btList)
	{
		// animacje dwustanowe
		button.Update(Power); // odczyt parametru i wybór submodelu
	}
};
