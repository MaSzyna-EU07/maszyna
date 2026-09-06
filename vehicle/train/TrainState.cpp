/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/train/Train.h"

#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "world/Event.h"
#include "simulation/simulationtime.h"
#include "model/Model3d.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "world/mtable.h"
#include "application/application.h"
#include "rendering/renderer.h"
#include <future>
#include <cmath>
#include <algorithm>

void TTrain::screen_entry::deserialize(cParser &Input)
{

	while (true == deserialize_mapping(Input))
	{
		; // all work done by while()
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
	CabPos1.x = std::stod(token);
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

// NOTE: we're currently using universal handlers and static handler map but it may be beneficial to have these implemented on individual class instance basis
// TBD, TODO: consider this approach if we ever want to have customized consist behaviour to received commands, based on the consist/vehicle type or whatever
TTrain::commandhandler_map const TTrain::m_commandhandlers = {

    {user_command::aidriverenable, &TTrain::OnCommand_aidriverenable},
    {user_command::aidriverdisable, &TTrain::OnCommand_aidriverdisable},
    {user_command::jointcontrollerset, &TTrain::OnCommand_jointcontrollerset},
    {user_command::mastercontrollerincrease, &TTrain::OnCommand_mastercontrollerincrease},
    {user_command::mastercontrollerincreasefast, &TTrain::OnCommand_mastercontrollerincreasefast},
    {user_command::mastercontrollerdecrease, &TTrain::OnCommand_mastercontrollerdecrease},
    {user_command::mastercontrollerdecreasefast, &TTrain::OnCommand_mastercontrollerdecreasefast},
    {user_command::mastercontrollerset, &TTrain::OnCommand_mastercontrollerset},
    {user_command::secondcontrollerincrease, &TTrain::OnCommand_secondcontrollerincrease},
    {user_command::secondcontrollerincreasefast, &TTrain::OnCommand_secondcontrollerincreasefast},
    {user_command::secondcontrollerdecrease, &TTrain::OnCommand_secondcontrollerdecrease},
    {user_command::secondcontrollerdecreasefast, &TTrain::OnCommand_secondcontrollerdecreasefast},
    {user_command::secondcontrollerset, &TTrain::OnCommand_secondcontrollerset},
    {user_command::dynamicbrakecontrollerincrease, &TTrain::OnCommand_DynamicBrakeControllerIncrease},
    {user_command::dynamicbrakecontrollerincreasefast, &TTrain::OnCommand_DynamicBrakeControllerIncreaseFast},
    {user_command::dynamicbrakecontrollerdecrease, &TTrain::OnCommand_DynamicBrakeControllerDecrease},
    {user_command::dynamicbrakecontrollerdecreasefast, &TTrain::OnCommand_DynamicBrakeControllerDecreaseFast},
    {user_command::dynamicbrakecontrollerset, &TTrain::OnCommand_DynamicBrakeControllerSet},
    {user_command::notchingrelaytoggle, &TTrain::OnCommand_notchingrelaytoggle},
    {user_command::tempomattoggle, &TTrain::OnCommand_tempomattoggle},
    {user_command::mucurrentindicatorothersourceactivate, &TTrain::OnCommand_mucurrentindicatorothersourceactivate},
    {user_command::independentbrakeincrease, &TTrain::OnCommand_independentbrakeincrease},
    {user_command::independentbrakeincreasefast, &TTrain::OnCommand_independentbrakeincreasefast},
    {user_command::independentbrakedecrease, &TTrain::OnCommand_independentbrakedecrease},
    {user_command::independentbrakedecreasefast, &TTrain::OnCommand_independentbrakedecreasefast},
    {user_command::independentbrakeset, &TTrain::OnCommand_independentbrakeset},
    {user_command::independentbrakebailoff, &TTrain::OnCommand_independentbrakebailoff},
    {user_command::universalbrakebutton1, &TTrain::OnCommand_universalbrakebutton1},
    {user_command::universalbrakebutton2, &TTrain::OnCommand_universalbrakebutton2},
    {user_command::universalbrakebutton3, &TTrain::OnCommand_universalbrakebutton3},
    {user_command::trainbrakeincrease, &TTrain::OnCommand_trainbrakeincrease},
    {user_command::trainbrakedecrease, &TTrain::OnCommand_trainbrakedecrease},
    {user_command::trainbrakeset, &TTrain::OnCommand_trainbrakeset},
    {user_command::trainbrakecharging, &TTrain::OnCommand_trainbrakecharging},
    {user_command::trainbrakerelease, &TTrain::OnCommand_trainbrakerelease},
    {user_command::trainbrakefirstservice, &TTrain::OnCommand_trainbrakefirstservice},
    {user_command::trainbrakeservice, &TTrain::OnCommand_trainbrakeservice},
    {user_command::trainbrakefullservice, &TTrain::OnCommand_trainbrakefullservice},
    {user_command::trainbrakehandleoff, &TTrain::OnCommand_trainbrakehandleoff},
    {user_command::trainbrakeemergency, &TTrain::OnCommand_trainbrakeemergency},
    {user_command::trainbrakebasepressureincrease, &TTrain::OnCommand_trainbrakebasepressureincrease},
    {user_command::trainbrakebasepressuredecrease, &TTrain::OnCommand_trainbrakebasepressuredecrease},
    {user_command::trainbrakebasepressurereset, &TTrain::OnCommand_trainbrakebasepressurereset},
    {user_command::trainbrakeoperationtoggle, &TTrain::OnCommand_trainbrakeoperationtoggle},
    {user_command::manualbrakeincrease, &TTrain::OnCommand_manualbrakeincrease},
    {user_command::manualbrakedecrease, &TTrain::OnCommand_manualbrakedecrease},
    {user_command::alarmchaintoggle, &TTrain::OnCommand_alarmchaintoggle},
    {user_command::alarmchainenable, &TTrain::OnCommand_alarmchainenable},
    {user_command::alarmchaindisable, &TTrain::OnCommand_alarmchaindisable},
    {user_command::wheelspinbrakeactivate, &TTrain::OnCommand_wheelspinbrakeactivate},
    {user_command::sandboxactivate, &TTrain::OnCommand_sandboxactivate},
    {user_command::autosandboxtoggle, &TTrain::OnCommand_autosandboxtoggle},
    {user_command::autosandboxactivate, &TTrain::OnCommand_autosandboxactivate},
    {user_command::autosandboxdeactivate, &TTrain::OnCommand_autosandboxdeactivate},
    {user_command::epbrakecontroltoggle, &TTrain::OnCommand_epbrakecontroltoggle},
    {user_command::epbrakecontrolenable, &TTrain::OnCommand_epbrakecontrolenable},
    {user_command::epbrakecontroldisable, &TTrain::OnCommand_epbrakecontroldisable},
    {user_command::trainbrakeoperationmodeincrease, &TTrain::OnCommand_trainbrakeoperationmodeincrease},
    {user_command::trainbrakeoperationmodedecrease, &TTrain::OnCommand_trainbrakeoperationmodedecrease},
    {user_command::brakeactingspeedincrease, &TTrain::OnCommand_brakeactingspeedincrease},
    {user_command::brakeactingspeeddecrease, &TTrain::OnCommand_brakeactingspeeddecrease},
    {user_command::brakeactingspeedsetcargo, &TTrain::OnCommand_brakeactingspeedsetcargo},
    {user_command::brakeactingspeedsetpassenger, &TTrain::OnCommand_brakeactingspeedsetpassenger},
    {user_command::brakeactingspeedsetrapid, &TTrain::OnCommand_brakeactingspeedsetrapid},
    {user_command::brakeloadcompensationincrease, &TTrain::OnCommand_brakeloadcompensationincrease},
    {user_command::brakeloadcompensationdecrease, &TTrain::OnCommand_brakeloadcompensationdecrease},
    {user_command::mubrakingindicatortoggle, &TTrain::OnCommand_mubrakingindicatortoggle},
    {user_command::reverserincrease, &TTrain::OnCommand_reverserincrease},
    {user_command::reverserdecrease, &TTrain::OnCommand_reverserdecrease},
    {user_command::reverserforwardhigh, &TTrain::OnCommand_reverserforwardhigh},
    {user_command::reverserforward, &TTrain::OnCommand_reverserforward},
    {user_command::reverserneutral, &TTrain::OnCommand_reverserneutral},
    {user_command::reverserbackward, &TTrain::OnCommand_reverserbackward},
    {user_command::alerteracknowledge, &TTrain::OnCommand_alerteracknowledge},
    {user_command::cabsignalacknowledge, &TTrain::OnCommand_cabsignalacknowledge},
    {user_command::batterytoggle, &TTrain::OnCommand_batterytoggle},
    {user_command::batteryenable, &TTrain::OnCommand_batteryenable},
    {user_command::batterydisable, &TTrain::OnCommand_batterydisable},
    {user_command::cabactivationtoggle, &TTrain::OnCommand_cabactivationtoggle},
    {user_command::cabactivationenable, &TTrain::OnCommand_cabactivationenable},
    {user_command::cabactivationdisable, &TTrain::OnCommand_cabactivationdisable},
    {user_command::pantographcompressorvalvetoggle, &TTrain::OnCommand_pantographcompressorvalvetoggle},
    {user_command::pantographcompressorvalveenable, &TTrain::OnCommand_pantographcompressorvalveenable},
    {user_command::pantographcompressorvalvedisable, &TTrain::OnCommand_pantographcompressorvalvedisable},
    {user_command::pantographcompressoractivate, &TTrain::OnCommand_pantographcompressoractivate},
    {user_command::pantographtogglefront, &TTrain::OnCommand_pantographtogglefront},
    {user_command::pantographtogglerear, &TTrain::OnCommand_pantographtogglerear},
    {user_command::pantographraisefront, &TTrain::OnCommand_pantographraisefront},
    {user_command::pantographraiserear, &TTrain::OnCommand_pantographraiserear},
    {user_command::pantographlowerfront, &TTrain::OnCommand_pantographlowerfront},
    {user_command::pantographlowerrear, &TTrain::OnCommand_pantographlowerrear},

    {user_command::wiperswitchincrease, &TTrain::OnCommand_wiperswitchincrease},
    {user_command::wiperswitchdecrease, &TTrain::OnCommand_wiperswitchdecrease},

    {user_command::lightsset, &TTrain::OnCommand_lightsset},

    {user_command::pantographlowerall, &TTrain::OnCommand_pantographlowerall},
    {user_command::pantographselectnext, &TTrain::OnCommand_pantographselectnext},
    {user_command::pantographselectprevious, &TTrain::OnCommand_pantographselectprevious},
    {user_command::pantographtoggleselected, &TTrain::OnCommand_pantographtoggleselected},
    {user_command::pantographraiseselected, &TTrain::OnCommand_pantographraiseselected},
    {user_command::pantographlowerselected, &TTrain::OnCommand_pantographlowerselected},
    {user_command::pantographvalvesupdate, &TTrain::OnCommand_pantographvalvesupdate},
    {user_command::pantographvalvesoff, &TTrain::OnCommand_pantographvalvesoff},
    {user_command::linebreakertoggle, &TTrain::OnCommand_linebreakertoggle},
    {user_command::linebreakeropen, &TTrain::OnCommand_linebreakeropen},
    {user_command::linebreakerclose, &TTrain::OnCommand_linebreakerclose},
    {user_command::fuelpumptoggle, &TTrain::OnCommand_fuelpumptoggle},
    {user_command::fuelpumpenable, &TTrain::OnCommand_fuelpumpenable},
    {user_command::fuelpumpdisable, &TTrain::OnCommand_fuelpumpdisable},
    {user_command::oilpumptoggle, &TTrain::OnCommand_oilpumptoggle},
    {user_command::oilpumpenable, &TTrain::OnCommand_oilpumpenable},
    {user_command::oilpumpdisable, &TTrain::OnCommand_oilpumpdisable},
    {user_command::waterheaterbreakertoggle, &TTrain::OnCommand_waterheaterbreakertoggle},
    {user_command::waterheaterbreakerclose, &TTrain::OnCommand_waterheaterbreakerclose},
    {user_command::waterheaterbreakeropen, &TTrain::OnCommand_waterheaterbreakeropen},
    {user_command::waterheatertoggle, &TTrain::OnCommand_waterheatertoggle},
    {user_command::waterheaterenable, &TTrain::OnCommand_waterheaterenable},
    {user_command::waterheaterdisable, &TTrain::OnCommand_waterheaterdisable},
    {user_command::waterpumpbreakertoggle, &TTrain::OnCommand_waterpumpbreakertoggle},
    {user_command::waterpumpbreakerclose, &TTrain::OnCommand_waterpumpbreakerclose},
    {user_command::waterpumpbreakeropen, &TTrain::OnCommand_waterpumpbreakeropen},
    {user_command::waterpumptoggle, &TTrain::OnCommand_waterpumptoggle},
    {user_command::waterpumpenable, &TTrain::OnCommand_waterpumpenable},
    {user_command::waterpumpdisable, &TTrain::OnCommand_waterpumpdisable},
    {user_command::watercircuitslinktoggle, &TTrain::OnCommand_watercircuitslinktoggle},
    {user_command::watercircuitslinkenable, &TTrain::OnCommand_watercircuitslinkenable},
    {user_command::watercircuitslinkdisable, &TTrain::OnCommand_watercircuitslinkdisable},
    {user_command::convertertoggle, &TTrain::OnCommand_convertertoggle},
    {user_command::converterenable, &TTrain::OnCommand_converterenable},
    {user_command::converterdisable, &TTrain::OnCommand_converterdisable},
    {user_command::convertertogglelocal, &TTrain::OnCommand_convertertogglelocal},
    {user_command::converteroverloadrelayreset, &TTrain::OnCommand_converteroverloadrelayreset},
    {user_command::compressortoggle, &TTrain::OnCommand_compressortoggle},
    {user_command::compressorenable, &TTrain::OnCommand_compressorenable},
    {user_command::compressordisable, &TTrain::OnCommand_compressordisable},
    {user_command::compressortogglelocal, &TTrain::OnCommand_compressortogglelocal},
    {user_command::compressorpresetactivatenext, &TTrain::OnCommand_compressorpresetactivatenext},
    {user_command::compressorpresetactivateprevious, &TTrain::OnCommand_compressorpresetactivateprevious},
    {user_command::compressorpresetactivatedefault, &TTrain::OnCommand_compressorpresetactivatedefault},
    {user_command::motorblowerstogglefront, &TTrain::OnCommand_motorblowerstogglefront},
    {user_command::motorblowerstogglerear, &TTrain::OnCommand_motorblowerstogglerear},
    {user_command::motorblowersdisableall, &TTrain::OnCommand_motorblowersdisableall},
    {user_command::coolingfanstoggle, &TTrain::OnCommand_coolingfanstoggle},
    {user_command::motorconnectorsopen, &TTrain::OnCommand_motorconnectorsopen},
    {user_command::motorconnectorsclose, &TTrain::OnCommand_motorconnectorsclose},
    {user_command::motordisconnect, &TTrain::OnCommand_motordisconnect},
    {user_command::motoroverloadrelaythresholdtoggle, &TTrain::OnCommand_motoroverloadrelaythresholdtoggle},
    {user_command::motoroverloadrelaythresholdsetlow, &TTrain::OnCommand_motoroverloadrelaythresholdsetlow},
    {user_command::motoroverloadrelaythresholdsethigh, &TTrain::OnCommand_motoroverloadrelaythresholdsethigh},
    {user_command::motoroverloadrelayreset, &TTrain::OnCommand_motoroverloadrelayreset},
    {user_command::universalrelayreset1, &TTrain::OnCommand_universalrelayreset},
    {user_command::universalrelayreset2, &TTrain::OnCommand_universalrelayreset},
    {user_command::universalrelayreset3, &TTrain::OnCommand_universalrelayreset},
    {user_command::heatingtoggle, &TTrain::OnCommand_heatingtoggle},
    {user_command::heatingenable, &TTrain::OnCommand_heatingenable},
    {user_command::heatingdisable, &TTrain::OnCommand_heatingdisable},
    {user_command::lightspresetactivatenext, &TTrain::OnCommand_lightspresetactivatenext},
    {user_command::lightspresetactivateprevious, &TTrain::OnCommand_lightspresetactivateprevious},
    {user_command::headlighttoggleleft, &TTrain::OnCommand_headlighttoggleleft},
    {user_command::headlightenableleft, &TTrain::OnCommand_headlightenableleft},
    {user_command::headlightdisableleft, &TTrain::OnCommand_headlightdisableleft},
    {user_command::headlighttoggleright, &TTrain::OnCommand_headlighttoggleright},
    {user_command::headlightenableright, &TTrain::OnCommand_headlightenableright},
    {user_command::headlightdisableright, &TTrain::OnCommand_headlightdisableright},
    {user_command::headlighttoggleupper, &TTrain::OnCommand_headlighttoggleupper},
    {user_command::headlightenableupper, &TTrain::OnCommand_headlightenableupper},
    {user_command::headlightdisableupper, &TTrain::OnCommand_headlightdisableupper},
    {user_command::redmarkertoggleleft, &TTrain::OnCommand_redmarkertoggleleft},
    {user_command::redmarkerenableleft, &TTrain::OnCommand_redmarkerenableleft},
    {user_command::redmarkerdisableleft, &TTrain::OnCommand_redmarkerdisableleft},
    {user_command::redmarkertoggleright, &TTrain::OnCommand_redmarkertoggleright},
    {user_command::redmarkerenableright, &TTrain::OnCommand_redmarkerenableright},
    {user_command::redmarkerdisableright, &TTrain::OnCommand_redmarkerdisableright},
    {user_command::headlighttogglerearleft, &TTrain::OnCommand_headlighttogglerearleft},
    {user_command::headlightenablerearleft, &TTrain::OnCommand_headlightenablerearleft},
    {user_command::headlightdisablerearleft, &TTrain::OnCommand_headlightdisablerearleft},
    {user_command::headlighttogglerearright, &TTrain::OnCommand_headlighttogglerearright},
    {user_command::headlightenablerearright, &TTrain::OnCommand_headlightenablerearright},
    {user_command::headlightdisablerearright, &TTrain::OnCommand_headlightdisablerearright},
    {user_command::headlighttogglerearupper, &TTrain::OnCommand_headlighttogglerearupper},
    {user_command::headlightenablerearupper, &TTrain::OnCommand_headlightenablerearupper},
    {user_command::headlightdisablerearupper, &TTrain::OnCommand_headlightdisablerearupper},
    {user_command::modernlightdimmerdecrease, &TTrain::OnCommand_modernlightdimmerdecrease},
    {user_command::modernlightdimmerincrease, &TTrain::OnCommand_modernlightdimmerincrease},
    {user_command::redmarkertogglerearleft, &TTrain::OnCommand_redmarkertogglerearleft},
    {user_command::redmarkerenablerearleft, &TTrain::OnCommand_redmarkerenablerearleft},
    {user_command::redmarkerdisablerearleft, &TTrain::OnCommand_redmarkerdisablerearleft},
    {user_command::redmarkertogglerearright, &TTrain::OnCommand_redmarkertogglerearright},
    {user_command::redmarkerenablerearright, &TTrain::OnCommand_redmarkerenablerearright},
    {user_command::redmarkerdisablerearright, &TTrain::OnCommand_redmarkerdisablerearright},
    {user_command::redmarkerstoggle, &TTrain::OnCommand_redmarkerstoggle},
    {user_command::endsignalstoggle, &TTrain::OnCommand_endsignalstoggle},
    {user_command::headlightsdimtoggle, &TTrain::OnCommand_headlightsdimtoggle},
    {user_command::headlightsdimenable, &TTrain::OnCommand_headlightsdimenable},
    {user_command::headlightsdimdisable, &TTrain::OnCommand_headlightsdimdisable},
    {user_command::interiorlighttoggle, &TTrain::OnCommand_interiorlighttoggle},
    {user_command::interiorlightenable, &TTrain::OnCommand_interiorlightenable},
    {user_command::interiorlightdisable, &TTrain::OnCommand_interiorlightdisable},
    {user_command::interiorlightdimtoggle, &TTrain::OnCommand_interiorlightdimtoggle},
    {user_command::interiorlightdimenable, &TTrain::OnCommand_interiorlightdimenable},
    {user_command::interiorlightdimdisable, &TTrain::OnCommand_interiorlightdimdisable},
    {user_command::compartmentlightstoggle, &TTrain::OnCommand_compartmentlightstoggle},
    {user_command::compartmentlightsenable, &TTrain::OnCommand_compartmentlightsenable},
    {user_command::compartmentlightsdisable, &TTrain::OnCommand_compartmentlightsdisable},
    {user_command::instrumentlighttoggle, &TTrain::OnCommand_instrumentlighttoggle},
    {user_command::instrumentlightenable, &TTrain::OnCommand_instrumentlightenable},
    {user_command::instrumentlightdisable, &TTrain::OnCommand_instrumentlightdisable},
    {user_command::dashboardlighttoggle, &TTrain::OnCommand_dashboardlighttoggle},
    {user_command::dashboardlightenable, &TTrain::OnCommand_dashboardlightenable},
    {user_command::dashboardlightdisable, &TTrain::OnCommand_dashboardlightdisable},
    {user_command::timetablelighttoggle, &TTrain::OnCommand_timetablelighttoggle},
    {user_command::timetablelightenable, &TTrain::OnCommand_timetablelightenable},
    {user_command::timetablelightdisable, &TTrain::OnCommand_timetablelightdisable},
    {user_command::doorlocktoggle, &TTrain::OnCommand_doorlocktoggle},
    {user_command::doortoggleleft, &TTrain::OnCommand_doortoggleleft},
    {user_command::doortoggleright, &TTrain::OnCommand_doortoggleright},
    {user_command::doorpermitleft, &TTrain::OnCommand_doorpermitleft},
    {user_command::doorpermitright, &TTrain::OnCommand_doorpermitright},
    {user_command::doorpermitpresetactivatenext, &TTrain::OnCommand_doorpermitpresetactivatenext},
    {user_command::doorpermitpresetactivateprevious, &TTrain::OnCommand_doorpermitpresetactivateprevious},
    {user_command::dooropenleft, &TTrain::OnCommand_dooropenleft},
    {user_command::dooropenright, &TTrain::OnCommand_dooropenright},
    {user_command::doorcloseleft, &TTrain::OnCommand_doorcloseleft},
    {user_command::doorcloseright, &TTrain::OnCommand_doorcloseright},
    {user_command::dooropenall, &TTrain::OnCommand_dooropenall},
    {user_command::doorcloseall, &TTrain::OnCommand_doorcloseall},
    {user_command::doorsteptoggle, &TTrain::OnCommand_doorsteptoggle},
    {user_command::doormodetoggle, &TTrain::OnCommand_doormodetoggle},
    {user_command::mirrorstoggle, &TTrain::OnCommand_mirrorstoggle},
    {user_command::nearestcarcouplingincrease, &TTrain::OnCommand_nearestcarcouplingincrease},
    {user_command::nearestcarcouplingdisconnect, &TTrain::OnCommand_nearestcarcouplingdisconnect},
    {user_command::nearestcarcoupleradapterattach, &TTrain::OnCommand_nearestcarcoupleradapterattach},
    {user_command::nearestcarcoupleradapterremove, &TTrain::OnCommand_nearestcarcoupleradapterremove},
    {user_command::occupiedcarcouplingdisconnect, &TTrain::OnCommand_occupiedcarcouplingdisconnect},
    {user_command::departureannounce, &TTrain::OnCommand_departureannounce},
    {user_command::hornlowactivate, &TTrain::OnCommand_hornlowactivate},
    {user_command::hornhighactivate, &TTrain::OnCommand_hornhighactivate},
    {user_command::whistleactivate, &TTrain::OnCommand_whistleactivate},
    {user_command::radiotoggle, &TTrain::OnCommand_radiotoggle},
    {user_command::radioenable, &TTrain::OnCommand_radioenable},
    {user_command::radiodisable, &TTrain::OnCommand_radiodisable},
    {user_command::radiochannelincrease, &TTrain::OnCommand_radiochannelincrease},
    {user_command::radiochanneldecrease, &TTrain::OnCommand_radiochanneldecrease},
    {user_command::radiochannelset, &TTrain::OnCommand_radiochannelset},
    {user_command::radiostopsend, &TTrain::OnCommand_radiostopsend},
    {user_command::radiostopenable, &TTrain::OnCommand_radiostopenable},
    {user_command::radiostopdisable, &TTrain::OnCommand_radiostopdisable},
    {user_command::radiostoptest, &TTrain::OnCommand_radiostoptest},
    {user_command::radiocall1send, &TTrain::OnCommand_radiocall1send},
    {user_command::radiocall3send, &TTrain::OnCommand_radiocall3send},
    {user_command::radiovolumeincrease, &TTrain::OnCommand_radiovolumeincrease},
    {user_command::radiovolumedecrease, &TTrain::OnCommand_radiovolumedecrease},
    {user_command::radiovolumeset, &TTrain::OnCommand_radiovolumeset},
    {user_command::cabchangeforward, &TTrain::OnCommand_cabchangeforward},
    {user_command::cabchangebackward, &TTrain::OnCommand_cabchangebackward},
    {user_command::generictoggle0, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle1, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle2, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle3, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle4, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle5, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle6, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle7, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle8, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle9, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle10, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle11, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle12, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle13, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle14, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle15, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle16, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle17, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle18, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle19, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle20, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle21, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle22, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle23, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle24, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle25, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle26, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle27, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle28, &TTrain::OnCommand_generictoggle},
    {user_command::generictoggle29, &TTrain::OnCommand_generictoggle},
    {user_command::vehiclemoveforwards, &TTrain::OnCommand_vehiclemoveforwards},
    {user_command::vehiclemovebackwards, &TTrain::OnCommand_vehiclemovebackwards},
    {user_command::vehicleboost, &TTrain::OnCommand_vehicleboost},
    {user_command::springbraketoggle, &TTrain::OnCommand_springbraketoggle},
    {user_command::springbrakeenable, &TTrain::OnCommand_springbrakeenable},
    {user_command::springbrakedisable, &TTrain::OnCommand_springbrakedisable},
    {user_command::springbrakeshutofftoggle, &TTrain::OnCommand_springbrakeshutofftoggle},
    {user_command::springbrakeshutoffenable, &TTrain::OnCommand_springbrakeshutoffenable},
    {user_command::springbrakeshutoffdisable, &TTrain::OnCommand_springbrakeshutoffdisable},
    {user_command::springbrakerelease, &TTrain::OnCommand_springbrakerelease},
    {user_command::distancecounteractivate, &TTrain::OnCommand_distancecounteractivate},
    {user_command::speedcontrolincrease, &TTrain::OnCommand_speedcontrolincrease},
    {user_command::speedcontroldecrease, &TTrain::OnCommand_speedcontroldecrease},
    {user_command::speedcontrolpowerincrease, &TTrain::OnCommand_speedcontrolpowerincrease},
    {user_command::speedcontrolpowerdecrease, &TTrain::OnCommand_speedcontrolpowerdecrease},
    {user_command::speedcontrolbutton0, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::speedcontrolbutton1, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::speedcontrolbutton2, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::speedcontrolbutton3, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::speedcontrolbutton4, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::speedcontrolbutton5, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::speedcontrolbutton6, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::speedcontrolbutton7, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::speedcontrolbutton8, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::speedcontrolbutton9, &TTrain::OnCommand_speedcontrolbutton},
    {user_command::inverterenable1, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable2, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable3, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable4, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable5, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable6, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable7, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable8, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable9, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable10, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable11, &TTrain::OnCommand_inverterenable},
    {user_command::inverterenable12, &TTrain::OnCommand_inverterenable},
    {user_command::inverterdisable1, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable2, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable3, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable4, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable5, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable6, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable7, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable8, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable9, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable10, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable11, &TTrain::OnCommand_inverterdisable},
    {user_command::inverterdisable12, &TTrain::OnCommand_inverterdisable},
    {user_command::invertertoggle1, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle2, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle3, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle4, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle5, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle6, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle7, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle8, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle9, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle10, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle11, &TTrain::OnCommand_invertertoggle},
    {user_command::invertertoggle12, &TTrain::OnCommand_invertertoggle},
};

std::vector<std::string> const TTrain::fPress_labels = {

    "ch1:  ", "ch2:  ", "ch3:  ", "ch4:  ", "ch5:  ", "ch6:  ", "ch7:  ", "ch8:  ", "ch9:  ", "ch0:  "};

TTrain::TTrain()
{

	ShowNextCurrent = false;
	// McZapkie-240302 - przyda sie do tachometru
	fTachoVelocity = 0;
	fTachoCount = 0;
	fPPress = fNPress = 0;

	// asMessage="";
	pMechOffset = glm::dvec3(0, 0, 0);
	fBlinkTimer = 0;
	fHaslerTimer = 0;
	DynamicSet(nullptr); // ustawia wszystkie mv*
	//-----
	pMechSittingPosition = glm::dvec3(0, 0, 0); // ABu: 180404
	fTachoTimer = 0.0; // włączenie skoków wskazań prędkościomierza

	//
	for (int i = 0; i < 8; i++)
	{
		bMains[i] = false;
		fCntVol[i] = 0.0f;
		bPants[i][0] = false;
		bPants[i][1] = false;
		bFuse[i] = false;
		bBatt[i] = false;
		bConv[i] = false;
		bComp[i][0] = false;
		bComp[i][1] = false;
		// bComp[ i ][ 2 ] = false;
		// bComp[ i ][ 3 ] = false;
		bHeat[i] = false;
	}
	bCompressors.clear();
	for (int i = 0; i < 9; ++i)
		for (int j = 0; j < 10; ++j)
		{
			fEIMParams[i][j] = 0.0;
			fDieselParams[i][j] = 0.0;
		}

	for (int i = 0; i < 20; ++i)
	{
		for (int j = 0; j < 7; ++j)
			fPress[i][j] = 0.0;
		bBrakes[i][0] = bBrakes[i][1] = false;
	}
}

TTrain::~TTrain() {}

bool TTrain::Init(TDynamicObject *NewDynamicObject, bool e3d)
{ // powiązanie ręcznego sterowania kabiną z pojazdem
	if (NewDynamicObject->Mechanik == nullptr)
	{
		/*
		ErrorLog( "Bad config: can't take control of inactive vehicle \"" + NewDynamicObject->asName + "\"" );
		return false;
		*/
		auto const activecab{(NewDynamicObject->MoverParameters->CabOccupied > 0 ? "1" : NewDynamicObject->MoverParameters->CabOccupied < 0 ? "2" : "p")};
		NewDynamicObject->create_controller(activecab, NewDynamicObject->ctOwner != nullptr);
	}

	DynamicSet(NewDynamicObject);
	if (!e3d)
		if (DynamicObject->Mechanik == nullptr)
			return false;

	DynamicObject->MechInside = true;

	fMainRelayTimer = 0; // Hunter, do k...y nędzy, ustawiaj wartości początkowe zmiennych!

	iCabn = mvOccupied->CabOccupied > 0 ? 1 : mvOccupied->CabOccupied < 0 ? 2 : 0;

	{
		Global.CurrentMaxTextureSize = Global.iMaxCabTextureSize;

		auto const filename{mvOccupied->TypeName + ".mmd"};
		LoadMMediaFile(filename);
		InitializeCab(mvOccupied->CabOccupied, filename);

		Global.CurrentMaxTextureSize = Global.iMaxTextureSize;

		if (DynamicObject->Controller == Humandriver)
		{
			// McZapkie-030303: mozliwosc wyswietlania kabiny, w przyszlosci dac opcje w mmd
			DynamicObject->bDisplayCab = true;
		}
	}

	// Ra: taka proteza - przesłanie kierunku do członów connected
	/*
	if (mvControlled->DirActive > 0)
	{ // było do przodu
	    mvControlled->DirectionBackward();
	    mvControlled->DirectionForward();
	}
	else if (mvControlled->DirActive < 0)
	{
	    mvControlled->DirectionForward();
	    mvControlled->DirectionBackward();
	}
	*/
	if (false == DynamicObject->Mechanik->AIControllFlag)
	{
		DynamicObject->Mechanik->sync_consist_reversers();
	}

	return true;
}

std::shared_ptr<dictionary_source> TTrain::GetTrainState(dictionary_source const &Extraparameters)
{

	if (mvOccupied == nullptr || mvControlled == nullptr)
	{
		return nullptr;
	}

	auto dict = std::make_shared<dictionary_source>(Extraparameters);
	if (dict == nullptr)
	{
		return nullptr;
	}

	dict->insert("name", DynamicObject->asName);
	dict->insert("cab", mvOccupied->CabOccupied);
	dict->insert("cabactive", mvOccupied->CabActive);
	dict->insert("master", mvOccupied->CabMaster);
	// basic systems state data
	dict->insert("battery", mvOccupied->Power24vIsAvailable);
	dict->insert("linebreaker", mvControlled->Mains);
	dict->insert("main_init", mvControlled->MainsInitTimeCountdown < mvControlled->MainsInitTime && mvControlled->MainsInitTimeCountdown > 0.0);
	dict->insert("main_ready", false == mvControlled->Mains && fHVoltage > 0.0 && mvControlled->MainsInitTimeCountdown <= 0.0);
	dict->insert("converter", mvOccupied->Power110vIsAvailable);
	dict->insert("converter_overload", mvControlled->ConvOvldFlag);
	dict->insert("compress", mvControlled->CompressorFlag);
	dict->insert("pant_compressor", mvPantographUnit->PantCompFlag);
	dict->insert("lights_front", mvOccupied->iLights[end::front]);
	dict->insert("lights_rear", mvOccupied->iLights[end::rear]);
	dict->insert("off_from_dimmer", mvOccupied->dimPositions[mvOccupied->modernDimmerPosition].isOff);
	dict->insert("lights_compartments", mvOccupied->CompartmentLights.is_active || mvOccupied->CompartmentLights.is_disabled);
	if (Dynamic()->Mechanik)
	{
		auto const *controller{Dynamic()->Mechanik};
		auto const cabmodifier{cab_to_end() == end::front ? 1 : -1};
		auto const traindirection{controller->Direction() * cabmodifier};
		auto const *frontvehicle{controller->Vehicle(traindirection >= 0 ? end::front : end::rear)};
		auto const *rearvehicle{controller->Vehicle(traindirection >= 0 ? end::rear : end::front)};
		auto const frontvehicledirection{(frontvehicle->DirectionGet() == controller->Vehicle()->DirectionGet() ? 1 : -1)};
		auto const rearvehicledirection{(rearvehicle->DirectionGet() == controller->Vehicle()->DirectionGet() ? 1 : -1)};
		auto const fronttrainlights{frontvehicle->MoverParameters->iLights[frontvehicledirection * cabmodifier >= 0 ? end::front : end::rear]};
		auto const reartrainlights{rearvehicle->MoverParameters->iLights[rearvehicledirection * cabmodifier >= 0 ? end::rear : end::front]};
		dict->insert("lights_train_front", fronttrainlights);
		dict->insert("lights_train_rear", reartrainlights);
	}
	else
	{
		// fallback, in the unlikely case we lose the controller
		dict->insert("lights_train_front", mvOccupied->iLights[end::front]);
		dict->insert("lights_train_rear", mvOccupied->iLights[end::rear]);
	}
	// reverser
	dict->insert("direction", mvOccupied->DirActive);
	// throttle
	dict->insert("mainctrl_pos", mvControlled->MainCtrlPos);
	dict->insert("mainctrl_pos_count", mvControlled->MainCtrlPosNo);
	dict->insert("main_ctrl_actual_pos", mvControlled->MainCtrlActualPos);
	dict->insert("scndctrl_pos", mvControlled->ScndCtrlPos);
	dict->insert("dynamicbrake_pos", mvControlled->DynamicBrakeCtrlPos);
	dict->insert("scnd_ctrl_actual_pos", mvControlled->ScndCtrlActualPos);
	dict->insert("brakectrl_pos", mvControlled->fBrakeCtrlPos);
	dict->insert("localbrake_pos", mvControlled->LocalBrakePosA);
	dict->insert("new_speed", mvOccupied->NewSpeed);
	dict->insert("speedctrl", mvOccupied->SpeedCtrlValue);
	dict->insert("speedctrlpower", mvOccupied->SpeedCtrlUnit.DesiredPower);
	dict->insert("speedctrlactive", mvOccupied->SpeedCtrlUnit.IsActive);
	dict->insert("speedctrlstandby", mvOccupied->SpeedCtrlUnit.Standby);
	// brakes
	dict->insert("manual_brake", mvOccupied->ManualBrakePos > 0);
	bool const bEP = mvControlled->LocHandle->GetCP() > 0.2 || fEIMParams[0][5] > 0.01;
	dict->insert("dir_brake", bEP);
	bool bPN{false};
	if (typeid(*mvOccupied->Hamulec) == typeid(TLSt) || typeid(*mvOccupied->Hamulec) == typeid(TEStED))
	{

		TBrake *temp_ham = mvOccupied->Hamulec.get();
		bPN = static_cast<TLSt *>(temp_ham)->GetEDBCP() > 0.2;
	}
	dict->insert("indir_brake", bPN);
	dict->insert("emergency_brake", mvOccupied->AlarmChainFlag);
	dict->insert("brake_delay_flag", mvOccupied->BrakeDelayFlag);
	dict->insert("brake_op_mode_flag", mvOccupied->BrakeOpModeFlag);
	dict->insert("pipelock", mvOccupied->LockPipe);
	// other controls
	dict->insert("ca", mvOccupied->SecuritySystem.is_vigilance_blinking());
	dict->insert("shp", mvOccupied->SecuritySystem.is_cabsignal_blinking());
	dict->insert("distance_counter", m_distancecounter);
	dict->insert("pantpress", std::abs(mvPantographUnit->PantPress));
	dict->insert("universal3", InstrumentLightActive);
	for (auto idx = 0; idx < ggUniversals.size(); idx++)
	{
		if (idx != 3)
		{
			dict->insert("universal" + std::to_string(idx), ggUniversals[idx].GetValue() > 0.5);
		}
	}
	dict->insert("radio", mvOccupied->Radio);
	dict->insert("radio_channel", RadioChannel());
	dict->insert("radio_volume", m_radiovolume);
	dict->insert("door_lock", mvOccupied->Doors.lock_enabled);
	dict->insert("door_step", mvOccupied->Doors.step_enabled);
	dict->insert("door_permit_left", mvOccupied->Doors.instances[side::left].open_permit);
	dict->insert("door_permit_right", mvOccupied->Doors.instances[side::right].open_permit);
	// movement data
	dict->insert("velocity", std::abs(mvOccupied->Vel));
	dict->insert("tractionforce", std::abs(mvOccupied->Ft));
	dict->insert("slipping_wheels", mvOccupied->SlippingWheels);
	dict->insert("sanding", mvOccupied->SandDose);
	dict->insert("odometer", mvOccupied->DistCounter);
	// electric current data
	dict->insert("traction_voltage", std::abs(mvPantographUnit->PantographVoltage));
	dict->insert("voltage", std::abs(mvControlled->EngineVoltage));
	dict->insert("im", std::abs(mvControlled->Im));
	dict->insert("fuse", mvControlled->FuseFlag);
	dict->insert("epfuse", mvOccupied->EpFuse);
	dict->insert("power_drawn", mvOccupied->EnergyMeter.first);
	dict->insert("power_returned", mvOccupied->EnergyMeter.second);

	// induction motor state data
	char const *TXTT[10] = {"fd", "fdt", "fdb", "pd", "pdt", "pdb", "itothv", "1", "2", "3"};
	char const *TXTC[10] = {"fr", "frt", "frb", "pr", "prt", "prb", "im", "vm", "ihv", "uhv"};
	char const *TXTD[10] = {"enrot", "nrot", "fill_des", "fill_real", "clutch_des", "clutch_real", "water_temp", "oil_press", "engine_temp", "retarder_fill"};
	char const *TXTP[7] = {"bc", "bp", "sp", "cp", "rp", "mass", "spring"};
	char const *TXTB[2] = {"spring_active", "spring_shutoff"};
	for (int j = 0; j < 10; ++j)
		dict->insert("eimp_t_" + std::string(TXTT[j]), fEIMParams[0][j]);
	for (int i = 0; i < 8; ++i)
	{
		auto const idx{std::to_string(i + 1)};
		for (int j = 0; j < 10; ++j)
			dict->insert("eimp_c" + idx + "_" + std::string(TXTC[j]), fEIMParams[i + 1][j]);

		for (int j = 0; j < 10; ++j)
			dict->insert("diesel_param_" + idx + "_" + std::string(TXTD[j]), fDieselParams[i + 1][j]);

		dict->insert("eimp_c" + idx + "_ms", bMains[i]);
		dict->insert("eimp_c" + idx + "_cv", fCntVol[i]);
		dict->insert("eimp_c" + idx + "_fuse", bFuse[i]);
		dict->insert("eimp_c" + idx + "_batt", bBatt[i]);
		dict->insert("eimp_c" + idx + "_conv", bConv[i]);
		dict->insert("eimp_c" + idx + "_heat", bHeat[i]);

		dict->insert("eimp_u" + idx + "_pf", bPants[i][0]);
		dict->insert("eimp_u" + idx + "_pr", bPants[i][1]);
		dict->insert("eimp_u" + idx + "_comp_a", bComp[i][0]);
		dict->insert("eimp_u" + idx + "_comp_w", bComp[i][1]);
	}

	dict->insert("compressors_no", (int)bCompressors.size());
	for (int i = 0; i < bCompressors.size(); i++)
	{
		auto const idx{std::to_string(i + 1)};
		dict->insert("compressors_" + idx + "_allow", std::get<0>(bCompressors[i]));
		dict->insert("compressors_" + idx + "_work", std::get<1>(bCompressors[i]));
		dict->insert("compressors_" + idx + "_car_no", std::get<2>(bCompressors[i]));
	}

	bool kier = DynamicObject->DirectionGet() * mvOccupied->CabOccupied > 0;
	TDynamicObject *p = DynamicObject->GetFirstDynamic(mvOccupied->CabOccupied < 0 ? end::rear : end::front, 4);
	int in = 0;
	while (p && in < 8)
	{
		if (p->MoverParameters->eimc[eimc_p_Pmax] > 1)
		{
			in++;
			dict->insert("eimp_c" + std::to_string(in) + "_invno", p->MoverParameters->InvertersNo);
			for (int j = 0; j < p->MoverParameters->InvertersNo; j++)
			{
				dict->insert("eimp_c" + std::to_string(in) + "_inv" + std::to_string(j + 1) + "_act", p->MoverParameters->Inverters[j].IsActive);
				dict->insert("eimp_c" + std::to_string(in) + "_inv" + std::to_string(j + 1) + "_error", p->MoverParameters->Inverters[j].Error);
				dict->insert("eimp_c" + std::to_string(in) + "_inv" + std::to_string(j + 1) + "_allow", p->MoverParameters->Inverters[j].Activate);
			}
		}
		p = kier ? p->Next(4) : p->Prev(4);
	}
	for (int i = 0; i < 20; ++i)
	{
		for (int j = 0; j < 7; ++j)
		{
			dict->insert("eimp_pn" + std::to_string(i + 1) + "_" + TXTP[j], fPress[i][j]);
		}
		for (int j = 0; j < 2; ++j)
		{
			dict->insert("brakes_" + std::to_string(i + 1) + "_" + TXTB[j], bBrakes[i][j]);
		}
	}
	// multi-unit state data
	dict->insert("car_no", iCarNo);
	dict->insert("power_no", iPowerNo);
	dict->insert("unit_no", iUnitNo);

	for (int i = 0; i < 20; i++)
	{
		auto const caridx{std::to_string(i + 1)};
		dict->insert("doors_" + caridx, bDoors[i][0]);
		dict->insert("doors_l_" + caridx, bDoors[i][1]);
		dict->insert("doors_r_" + caridx, bDoors[i][2]);
		dict->insert("doorstep_l_" + caridx, bDoors[i][3]);
		dict->insert("doorstep_r_" + caridx, bDoors[i][4]);
		dict->insert("doors_no_" + caridx, iDoorNo[i]);
		dict->insert("code_" + caridx, std::to_string(iUnits[i]) + cCode[i]);
		dict->insert("car_name" + caridx, asCarName[i]);
		dict->insert("slip_" + caridx, bSlip[i]);
	}
	// ai state data
	auto const *driver{(DynamicObject->ctOwner != nullptr ? DynamicObject->ctOwner : DynamicObject->Mechanik)};

	dict->insert("velocity_desired", driver->VelDesired);
	dict->insert("velroad", driver->VelRoad);
	dict->insert("vellimitlast", driver->VelLimitLast);
	dict->insert("velsignallast", driver->VelSignalLast);
	dict->insert("velsignalnext", driver->VelSignalNext);
	dict->insert("velnext", driver->VelNext);
	dict->insert("actualproximitydist", driver->ActualProximityDist);
	// train data
	driver->TrainTimetable().serialize(dict.get());
	dict->insert("train_atpassengerstop", driver->IsAtPassengerStop);
	dict->insert("train_length", driver->fLength);
	// world state data
	dict->insert("scenario", Global.SceneryFile);
	dict->insert("hours", static_cast<int>(simulation::Time.data().wHour));
	dict->insert("minutes", static_cast<int>(simulation::Time.data().wMinute));
	dict->insert("seconds", static_cast<int>(simulation::Time.second()));
	dict->insert("air_temperature", Global.AirTemperature);
	dict->insert("light_level", Global.fLuminance - std::max(0.f, Global.Overcast - 1.f));

	return dict;
}

TTrain::state_t TTrain::get_state() const
{

	return {
	    btLampkaSHP.GetValue(),
	    btLampkaCzuwaka.GetValue(),
	    btLampkaRadioStop.GetValue(),
	    btLampkaOpory.GetValue(),
	    btLampkaWylSzybki.GetValue(),
	    btLampkaPrzekRozn.GetValue(),
	    btLampkaNadmSil.GetValue(),
	    btLampkaStyczn.GetValue(),
	    btLampkaPoslizg.GetValue(),
	    btLampkaNadmPrzetw.GetValue(),
	    btLampkaPrzetwOff.GetValue(),
	    btLampkaNadmSpr.GetValue(),
	    btLampkaNadmWent.GetValue(),
	    btLampkaWysRozr.GetValue(),
	    btLampkaOgrzewanieSkladu.GetValue(),
	    static_cast<std::uint8_t>(iCabn),
	    btHaslerBrakes.GetValue(),
	    btHaslerCurrent.GetValue(),
	    mvOccupied->SecuritySystem.is_beeping(),
	    btLampkaHVoltageB.GetValue(),
	    fTachoVelocity,
	    static_cast<float>(mvOccupied->Compressor),
	    static_cast<float>(mvOccupied->PipePress),
	    static_cast<float>(mvOccupied->BrakePress),
	    static_cast<float>(mvPantographUnit->PantPress),
	    fHVoltage,
	    {fHCurrent[mvControlled->TrainType & dt_EZT ? 0 : 1], fHCurrent[2], fHCurrent[3]},
	    ggLVoltage.GetValue(),
	    mvOccupied->DistCounter,
	    static_cast<std::uint8_t>(RadioChannel()),
	    btLampkaSpringBrakeActive.GetValue(),
	    btLampkaNapNastHam.GetValue(),
	    mvOccupied->DirActive > 0,
	    mvOccupied->DirActive < 0,
	    mvOccupied->Doors.instances[mvOccupied->CabOccupied < 0 ? side::right : side::left].open_permit,
	    mvOccupied->Doors.instances[mvOccupied->CabOccupied < 0 ? side::right : side::left].is_open,
	    mvOccupied->Doors.instances[mvOccupied->CabOccupied < 0 ? side::left : side::right].open_permit,
	    mvOccupied->Doors.instances[mvOccupied->CabOccupied < 0 ? side::left : side::right].is_open,
	    mvOccupied->Doors.step_enabled,
	    mvOccupied->Power24vIsAvailable,
	    0,
	    mvOccupied->LockPipe,
	    btLampkaRadioMessage.GetValue(),
	};
}
