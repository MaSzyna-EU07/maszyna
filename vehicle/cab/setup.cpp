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

// setting the cab up from its definition: its gauges and its buttons

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


// McZapkie-310302
// wczytywanie pliku z danymi multimedialnymi (dzwieki, kontrolki, kabiny)
bool TTrain::LoadMMediaFile(std::string const &asFileName)
{
	using enum sound_placement;
	using enum sound_type;
	// initialize sounds so potential entries from previous vehicle don't stick around
	std::unordered_map<std::string, std::tuple<std::optional<sound_source> &, sound_placement, float, sound_type, int, double>> internalsounds = {
	    {"ctrl:", {dsbNastawnikJazdy, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"ctrlscnd:", {dsbNastawnikBocz, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"reverserkey:", {dsbReverserKey, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"buzzer:", {dsbBuzzer, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"buzzershp:", {dsbBuzzerShp, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"radiostop:", {m_radiostop, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"slipalarm:", {dsbSlipAlarm, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"distancecounter:", {m_distancecounterclear, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"tachoclock:", {dsbHasler, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"switch:", {dsbSwitch, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"pneumaticswitch:", {dsbPneumaticSwitch, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"airsound:", {rsHiss, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, sound_parameters::amplitude, 100.0}},
	    {"airsound2:", {rsHissU, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, sound_parameters::amplitude, 100.0}},
	    {"airsound3:", {rsHissE, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, sound_parameters::amplitude, 100.0}},
	    {"airsound4:", {rsHissX, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, sound_parameters::amplitude, 100.0}},
	    {"airsound5:", {rsHissT, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, sound_parameters::amplitude, 100.0}},
	    {"localbrakesound:", {rsSBHiss, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, sound_parameters::amplitude, 100.0}},
	    {"localbrakesound2:", {rsSBHissU, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, sound_parameters::amplitude, 100.0}},
	    {"brakesound:", {rsBrake, internal, -1, single, sound_parameters::amplitude | sound_parameters::frequency, 100.0}},
	    {"fadesound:", {rsFadeSound, internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, single, 0, 100.0}},
	    {"runningnoise:", {rsRunningNoise, internal, EU07_SOUND_GLOBALRANGE, single, sound_parameters::amplitude | sound_parameters::frequency, mvOccupied->Vmax}},
	    {"resonancenoise:", {rsResonanceNoise, internal, EU07_SOUND_GLOBALRANGE, single, sound_parameters::amplitude | sound_parameters::frequency, mvOccupied->Vmax}},
	    {"windsound:", {rsWindSound, internal, EU07_SOUND_GLOBALRANGE, single, sound_parameters::amplitude | sound_parameters::frequency, mvOccupied->Vmax}},
	    {"huntingnoise:", {rsHuntingNoise, internal, EU07_SOUND_GLOBALRANGE, single, sound_parameters::amplitude | sound_parameters::frequency, mvOccupied->Vmax}},
	    {"rainsound:", {m_rainsound, internal, -1, single, 0, 100.0}},
	};
	for (auto &[soundname, soundsource] : internalsounds)
	{
		std::get<std::optional<sound_source> &>(soundsource).reset();
	}
	// NOTE: since radiosound is an incomplete template not using std::optional it gets a special treatment
	m_radiosound.owner(DynamicObject);
	m_fmradiosound.owner(DynamicObject);
	CabSoundLocations.clear();

	cParser parser(asFileName, cParser::buffer_FILE, DynamicObject->asBaseDir, true, std::vector<std::string>(), true);
	// NOTE: yaml-style comments are disabled until conflict in use of # is resolved
	// parser.addCommentStyle( "#", "\n" );
	std::string token;
	do
	{
		token = "";
		parser.getTokens();
		parser >> token;
	} while (token != "" && token != "internaldata:");

	if (token == "internaldata:")
	{

		do
		{
			token = "";
			parser.getTokens();
			parser >> token;

			auto lookup{internalsounds.find(token)};
			if (lookup == internalsounds.end())
			{
				continue;
			}

			auto const soundconfig{lookup->second};
			sound_source sound{std::get<sound_placement>(soundconfig), std::get<float>(soundconfig)};
			sound.deserialize(parser, std::get<sound_type>(soundconfig), std::get<int>(soundconfig), static_cast<int>(std::get<double>(soundconfig)));
			sound.owner(DynamicObject);
			std::get<std::optional<sound_source> &>(soundconfig) = sound;
		} while (token != "");

		// assign default samples to sound emitters which weren't included in the config file
		if (!m_rainsound)
		{
			sound_source rainsound;
			rainsound.deserialize("rainsound_default", single);
			rainsound.owner(DynamicObject);
			m_rainsound = rainsound;
		}
		if (!rsSBHiss)
		{
			// fallback for vehicles without defined local brake hiss sound
			rsSBHiss = rsHiss;
		}
		if (!rsSBHissU)
		{
			// fallback for vehicles without defined local brake hiss sound
			rsSBHissU = rsHissU;
		}
		if (rsBrake)
		{
			rsBrake->m_frequencyfactor /= static_cast<float>(1 + mvOccupied->Vmax);
		}
		if (rsResonanceNoise)
		{
			rsResonanceNoise->m_frequencyfactor /= static_cast<float>(1 + mvOccupied->Vmax);
		}
		if (rsWindSound)
		{
			rsWindSound->m_frequencyfactor /= static_cast<float>(1 + mvOccupied->Vmax);
		}
		if (rsRunningNoise)
		{
			rsRunningNoise->m_frequencyfactor /= static_cast<float>(1 + mvOccupied->Vmax);
		}
		if (rsHuntingNoise)
		{
			rsHuntingNoise->m_frequencyfactor /= static_cast<float>(1 + mvOccupied->Vmax);
		}
	}
	std::vector<std::reference_wrapper<std::optional<sound_source>>> sounds = {
	    dsbReverserKey, dsbNastawnikJazdy, dsbNastawnikBocz, dsbSwitch,        dsbPneumaticSwitch, rsHiss,         rsHissU,   rsHissE,   rsHissX,      rsHissT,      rsSBHiss,
	    rsSBHissU,      rsFadeSound,       rsRunningNoise,   rsResonanceNoise, rsWindSound,        rsHuntingNoise, dsbHasler, dsbBuzzer, dsbBuzzerShp, dsbSlipAlarm, m_distancecounterclear,
	    m_rainsound,    m_radiostop};
	for (auto &sound : sounds)
	{
		if (sound.get())
		{
			CabSoundLocations.emplace_back(sound, sound.get()->offset());
		}
	}

	return true;
}

bool TTrain::InitializeCab(int NewCabNo, std::string const &asFileName)
{
	m_controlmapper.clear();
	// clear python screens
	m_screens.clear();
	m_scriptscreens.clear();
	// reset sound positions
	auto const nullvector{glm::vec3()};
	std::vector<std::reference_wrapper<std::optional<sound_source>>> sounds = {
	    dsbReverserKey, dsbNastawnikJazdy, dsbNastawnikBocz, dsbSwitch,        dsbPneumaticSwitch, rsHiss,         rsHissU,   rsHissE,   rsHissX,      rsHissT,      rsSBHiss,
	    rsSBHissU,      rsFadeSound,       rsRunningNoise,   rsResonanceNoise, rsWindSound,        rsHuntingNoise, dsbHasler, dsbBuzzer, dsbBuzzerShp, dsbSlipAlarm, m_distancecounterclear,
	    m_rainsound,    m_radiostop};
	for (auto const &sound : sounds)
	{
		if (sound.get())
		{
			sound.get()->offset(nullvector);
		}
	}
	m_radiosound.offset(nullvector);
	m_fmradiosound.offset(nullvector);
	m_fmradiopresent = false;
	for (auto const &[soundsource, soundoffset] : CabSoundLocations)
	{
		if (soundsource.get() && soundsource.get()->offset() == nullvector)
		{
			soundsource.get()->offset(soundoffset);
		}
	}
	// reset view angles
	pMechViewAngle = {0.0, 0.0};

	is_cab_initialized = true; // the attempt may fail, but it's the attempt that counts

	bool parse = false;
	int cabindex = 0;
	DynamicObject->mdKabina = nullptr; // likwidacja wskaźnika na dotychczasową kabinę
	switch (NewCabNo)
	{ // ustalenie numeru kabiny do wczytania
	case -1:
		cabindex = 2;
		break;
	case 1:
		cabindex = 1;
		break;
	case 0:
		cabindex = 0;
		break;
	default:
		break;
	}
	iCabn = cabindex;

	std::string cabstr("cab" + std::to_string(cabindex) + "definition:");

	cParser parser(asFileName, cParser::buffer_FILE, DynamicObject->asBaseDir, true, std::vector<std::string>(), true);
	// NOTE: yaml-style comments are disabled until conflict in use of # is resolved
	// parser.addCommentStyle( "#", "\n" );
	std::string token;
	do
	{
		// szukanie kabiny
		token = "";
		parser.getTokens();
		parser >> token;
	} while (token != "" && token != cabstr);

	if (token == cabstr)
	{
		// jeśli znaleziony wpis kabiny
		Cabine[cabindex].Load(parser);
		// NOTE: the position and angle definitions depend on strict entry order
		// TODO: refactor into more flexible arrangement
		parser.getTokens();
		parser >> token;
		if (token == std::string("driver" + std::to_string(cabindex) + "angle:"))
		{
			// camera view angle
			parser.getTokens(2, false);
			// angle is specified in degrees but internally stored in radians
			glm::vec2 viewangle;
			parser >> viewangle.y // yaw first, then pitch
			    >> viewangle.x;
			pMechViewAngle = glm::radians(viewangle);

			Global.pCamera.Angle.x = pMechViewAngle.x;
			Global.pCamera.Angle.y = pMechViewAngle.y;

			parser.getTokens();
			parser >> token;
		}
		if (token == std::string("driver" + std::to_string(cabindex) + "pos:"))
		{
			// pozycja poczatkowa maszynisty
			parser.getTokens(3, false);
			parser >> pMechOffset.x >> pMechOffset.y >> pMechOffset.z;
			pMechSittingPosition = pMechOffset;

			parser.getTokens();
			parser >> token;
		}
		// ABu: pozycja siedzaca mechanika
		if (token == std::string("driver" + std::to_string(cabindex) + "sitpos:"))
		{
			// ABu 180404 pozycja siedzaca maszynisty
			parser.getTokens(3, false);
			parser >> pMechSittingPosition.x >> pMechSittingPosition.y >> pMechSittingPosition.z;

			parser.getTokens();
			parser >> token;
		}
		// else parse=false;
		do
		{
			if (parse == true)
			{
				token = "";
				parser.getTokens();
				parser >> token;
			}
			else
			{
				parse = true;
			}
			// inicjacja kabiny
			// Ra 2014-08: zmieniamy zasady - zamiast przypisywać submodel do
			// istniejących obiektów animujących
			// będziemy teraz uaktywniać obiekty animujące z tablicy i podawać im
			// submodel oraz wskaźnik na parametr
			if (token == std::string("cab" + std::to_string(cabindex) + "model:"))
			{
				// model kabiny
				parser.getTokens();
				parser >> token;
				std::ranges::replace(token, '\\', '/');
				if (token != "none")
				{
					// bieżąca sciezka do tekstur to dynamic/...
					Global.asCurrentTexturePath = DynamicObject->asBaseDir;
					// szukaj kabinę jako oddzielny model
					// name can contain leading slash, erase it to avoid creation of double slashes when the name is combined with current directory
					replace_slashes(token);
					erase_leading_slashes(token);
					if (token[0] == '/')
					{
						token.erase(0, 1);
					}
					TModel3d *kabina = TModelsManager::GetModel(DynamicObject->asBaseDir + token, true, true, Global.network_servers.empty() && !Global.network_client ? 0 : id());
					// z powrotem defaultowa sciezka do tekstur
					Global.asCurrentTexturePath = paths::textures;
					// if (DynamicObject->mdKabina!=k)
					if (kabina != nullptr)
					{
						DynamicObject->mdKabina = kabina; // nowa kabina
					}
					//(mdKabina) może zostać to samo po przejściu do innego członu bez
					// zmiany kabiny, przy powrocie musi być wiązanie ponowne
					// else
					// break; //wyjście z pętli, bo model zostaje bez zmian
				}
				else if (cabindex == 1)
				{
					// model tylko, gdy nie ma kabiny 1
					// McZapkie-170103: szukaj elementy kabiny w glownym modelu
					DynamicObject->mdKabina = DynamicObject->mdModel;
				}
				clear_cab_controls();
			}
			/*
			            if (nullptr == DynamicObject->mdKabina)
			            {
			                // don't bother with other parts until the cab is initialised
			                continue;
			            }
			*/
			else if (true == initialize_gauge(parser, token, cabindex))
			{
				// matched the token, grab the next one
				continue;
			}
			else if (true == initialize_button(parser, token, cabindex))
			{
				// matched the token, grab the next one
				continue;
			}
			// TODO: add "pydestination:"
			else if (token == "pyscreen:")
			{
				screen_entry screen;
				screen.deserialize(parser);
				if (false == screen.script.empty() && substr_path(screen.script).empty())
				{
					screen.script = DynamicObject->asBaseDir + screen.script;
				}

				ITexture *tex = nullptr;
				TSubModel *submodel = nullptr;
				if (screen.target != "none")
				{
					if (false == find_screen_surface(screen.target, "Python Screen", submodel, tex))
					{
						continue;
					}
				}
				else
				{
					// TODO: fix leak
					tex = ITexture::null_texture();
					tex->make_stub();
				}

				tex->create(true); // make the surface static so it doesn't get destroyed by garbage
				                   // collector if the user spends long time outside cab
				// TBD, TODO: keep texture handles around, so we can undo the static switch when the
				// user changes cabs?
				auto rt = std::make_shared<python_rt>();
				rt->shared_tex = tex;

				// record renderer and material binding for future update requests
				m_screens.emplace_back(screen);
				m_screens.back().rt = rt;

				m_screens.back().touch_list = std::make_shared<std::vector<glm::vec2>>();
				if (submodel)
					submodel->screen_touch_list = m_screens.back().touch_list;

				if (Global.python_displaywindows)
					m_screens.back().viewer = std::make_unique<python_screen_viewer>(rt, m_screens.back().touch_list, m_screens.back().script);
			}
			else if (token == "luascreen:")
			{
				load_script_screen(parser, script_language::lua);
			}
			else if (token == "wasmscreen:")
			{
				load_script_screen(parser, script_language::wasm);
			}
			else if (token == "pyscreenupdatetime:")
			{
				parser.getTokens();
				parser >> ScreenUpdateRate;
			}
			else if (token == "fmradio:")
			{
				// broadcast radio receiver, the parameter selects character of its sound
				std::string fmradiotype;
				parser.getTokens();
				parser >> fmradiotype;
				m_fmradiotype = (fmradiotype == "wooden" ? audio::fmradio_type::wooden : audio::fmradio_type::cheap);
				m_fmradiopresent = true;
			}
			else if (token == "fmradiospeaker:")
			{
				// submodel marking location of the receiver speaker, the sound is emitted from there
				std::string speakername;
				parser.getTokens();
				parser >> speakername;
				TSubModel *speaker{nullptr};
				for (auto const *source : {DynamicObject->mdKabina, DynamicObject->mdLowPolyInt})
				{
					if (source != nullptr && (speaker = source->GetFromName(speakername)) != nullptr)
					{
						break;
					}
				}
				if (speaker != nullptr)
				{
					m_fmradiosound.offset(speaker->offset(1.f));
				}
				else
				{
					ErrorLog("Bad config: failed to locate fm radio speaker submodel \"" + speakername + "\"", logtype::model);
				}
				m_fmradiopresent = true;
			}
			// btLampkaUnknown`"unknown",mdKabina,false);
		} while (token != ""
		         // TODO: enable full per-cab deserialization when/if .mmd files get proper per-cab switch configuration
		         //               && ( token != "cab1definition:" )
		         //               && ( token != "cab2definition:" )
		         && token != "cab0definition:");
		for (auto &screen : m_screens)
		{
			if (screen.updatetime > 0)
			{
				screen.updatetime = std::max((int)screen.updatetime, Global.PythonScreenUpdateRate) * 0.001;
			}
			if (screen.updatetime == 0)
			{
				screen.updatetime = std::max(Global.PythonScreenUpdateRate, ScreenUpdateRate) * 0.001;
			}
			if (screen.updatetime < -1)
			{
				screen.updatetime = -screen.updatetime * 0.001;
			}
		}
	}
	else
	{
		return false;
	}
	/*
	    if (DynamicObject->mdKabina)
	    {
	*/
	// configure placement of sound emitters which aren't bound with any device model, and weren't placed manually
	auto const caboffset{glm::dvec3{(Cabine[cabindex].CabPos1 + Cabine[cabindex].CabPos2) * 0.5f} + glm::dvec3{0, 1, 0}};
	// NOTE: since radiosound is an incomplete template not using std::optional it gets a special treatment
	if (m_radiosound.offset() == nullvector)
	{
		m_radiosound.offset(btLampkaRadio.model_offset());
	}
	if (m_radiosound.offset() == nullvector)
	{
		m_radiosound.offset(caboffset);
	}
	if (m_fmradiosound.offset() == nullvector)
	{
		m_fmradiosound.offset(caboffset);
	}
	if (m_fmradiopresent && false == m_fmradiopower && m_controlmapper.contains("fmradiovolume_sw:") && false == m_controlmapper.contains("fmradio_sw:"))
	{
		// receivers without dedicated switch are switched with the volume knob, so it has to start at the off position
		m_fmradiovolume = 0.f;
	}
	std::vector<std::pair<std::reference_wrapper<std::optional<sound_source>>, glm::vec3>> soundlocations = {
	    {dsbReverserKey, ggDirKey.model_offset()},
	    {dsbNastawnikJazdy, ggJointCtrl.model_offset()},
	    {dsbNastawnikJazdy, ggMainCtrl.model_offset()}, // NOTE: fallback for vehicles without universal controller
	    {dsbNastawnikBocz, ggScndCtrl.model_offset()},
	    {dsbSwitch, caboffset},
	    {dsbPneumaticSwitch, caboffset},
	    {rsHiss, ggBrakeCtrl.model_offset()},
	    {rsHissU, ggBrakeCtrl.model_offset()},
	    {rsHissE, ggBrakeCtrl.model_offset()},
	    {rsHissX, ggBrakeCtrl.model_offset()},
	    {rsHissT, ggBrakeCtrl.model_offset()},
	    {rsSBHiss, ggLocalBrake.model_offset()},
	    {rsSBHiss, ggBrakeCtrl.model_offset()}, // NOTE: fallback if the local brake model can't be located
	    {rsSBHissU, ggLocalBrake.model_offset()},
	    {rsSBHissU, ggBrakeCtrl.model_offset()}, // NOTE: fallback if the local brake model can't be located
	    {rsFadeSound, caboffset},
	    {rsRunningNoise, caboffset},
	    {rsResonanceNoise, caboffset},
	    {rsWindSound, caboffset},
	    {rsHuntingNoise, caboffset},
	    {dsbHasler, caboffset},
	    {dsbBuzzer, btLampkaCzuwaka.model_offset()},
	    {dsbBuzzerShp, btLampkaCzuwaka.model_offset()},
	    {dsbSlipAlarm, caboffset},
	    {m_distancecounterclear, btLampkaCzuwaka.model_offset()},
	    {m_rainsound, caboffset},
	    {m_radiostop, m_radiosound.offset()},
	};
	for (auto const &[soundsource, soundoffset] : soundlocations)
	{
		if (soundsource.get() && soundsource.get()->offset() == nullvector)
		{
			soundsource.get()->offset(soundoffset);
		}
	}
	// second pass, in case some items received no positioning due to missing submodels etc
	for (auto const &[soundsource, soundoffset] : soundlocations)
	{
		if (soundsource.get() && soundsource.get()->offset() == nullvector)
		{
			soundsource.get()->offset(caboffset);
		}
	}

	if (DynamicObject->mdKabina)
	{
		DynamicObject->mdKabina->Init(); // obrócenie modelu oraz optymalizacja, również zapisanie binarnego
		// the vehicle has a cab, a player is in it: the odometer store keeps the vehicle also when its type has no
		// driver's brake valve (eu07/maszyna#72)
		DynamicObject->MoverParameters->Odometer.shown = true;
	}

	set_cab_controls(NewCabNo < 0 ? 2 : NewCabNo);

	// set pantograpths (hujhujhuj)
	change_pantograph_selection(0);

	/*
	return true;
}
return (token == "none");
*/
	return true;
}

glm::dvec3 TTrain::MirrorPosition(bool lewe)
{ // zwraca współrzędne widoku kamery z lusterka
	auto const shiftdirection{(lewe ? -1 : 1) * (iCabn == 2 ? 1 : -1)};

	return DynamicObject->mMatrix *
	       glm::dvec4(mvOccupied->Dim.W * (0.5 * shiftdirection) + 0.2 * shiftdirection, 1.5 + Cabine[iCabn].CabPos1.y, std::lerp(Cabine[iCabn].CabPos1.z, Cabine[iCabn].CabPos2.z, 0.5), 1.0);
};

// clears state of all cabin controls
void TTrain::clear_cab_controls()
{
	// indicators exposed to custom control devices
	btLampkaSHP.Clear(0);
	btLampkaCzuwaka.Clear(1);
	btLampkaOpory.Clear(2);
	btLampkaWylSzybki.Clear(3);
	btLampkaNadmSil.Clear(4);
	btLampkaStyczn.Clear(5);
	btLampkaPoslizg.Clear(6);
	btLampkaNadmPrzetw.Clear(mvControlled->TrainType == train_type::emu ? -1 : 7); // EN57 nie ma tej lampki
	btLampkaPrzetwOff.Clear(mvControlled->TrainType == train_type::emu ? 7 : -1); // za to ma tę
	btLampkaNadmSpr.Clear(8);
	btLampkaNadmWent.Clear(9);
	btLampkaWysRozr.Clear(mvControlled->TrainType == train_type::et22 ? -1 : 10); // ET22 nie ma tej lampki
	btLampkaOgrzewanieSkladu.Clear(11);
	// overheat indicator lamps
	btLampkaOilOverheat.Clear(-1);
	btLampkaWaterOverheat.Clear(-1);
	btLampkaWaterAuxOverheat.Clear(-1);
	btLampkaEngineOverheat.Clear(-1);
	btHaslerBrakes.Clear(12); // ciśnienie w cylindrach do odbijania na haslerze
	btHaslerCurrent.Clear(13); // prąd na silnikach do odbijania na haslerze
	// Numer 14 jest używany dla buczka SHP w update_sounds()
	// Jeśli ustawiamy nową wartość dla PoKeys wolna jest 15

	// other cab controls
	// TODO: arrange in more readable manner, and eventually refactor
	ggJointCtrl.Clear();
	ggMainCtrl.Clear();
	ggMainCtrlAct.Clear();
	ggScndCtrl.Clear();
	ggScndCtrlButton.Clear();
	ggScndCtrlOffButton.Clear();
	ggDistanceCounterButton.Clear();
	ggDirKey.Clear();
	ggDirForwardButton.Clear();
	ggDirNeutralButton.Clear();
	ggDirBackwardButton.Clear();
	ggBrakeCtrl.Clear();
	ggLocalBrake.Clear();
	ggAlarmChain.Clear();
	ggBrakeProfileCtrl.Clear();
	ggBrakeProfileG.Clear();
	ggBrakeProfileR.Clear();
	ggBrakeOperationModeCtrl.Clear();
	ggWiperSw.Clear();
	ggMaxCurrentCtrl.Clear();
	ggAutoRelayButton.Clear();
	ggMainOffButton.Clear();
	ggMainOnButton.Clear();
	for (auto &remotelinebreaker : m_remotelinebreakers)
	{
		remotelinebreaker.main_switch.Clear();
		remotelinebreaker.on_button.Clear();
		remotelinebreaker.off_button.Clear();
		remotelinebreaker.held_without_switch = false;
	}
	ggSecurityResetButton.Clear();
	ggSHPResetButton.Clear();
	ggReleaserButton.Clear();
	ggSpringBrakeOnButton.Clear();
	ggSpringBrakeOffButton.Clear();
	ggUniveralBrakeButton1.Clear();
	ggUniveralBrakeButton2.Clear();
	ggUniveralBrakeButton3.Clear();
	ggEPFuseButton.Clear();
	ggSandButton.Clear();
	ggAutoSandButton.Clear();
	ggAntiSlipButton.Clear();
	ggHornButton.Clear();
	ggHornLowButton.Clear();
	ggHornHighButton.Clear();
	ggWhistleButton.Clear();
	ggHelperButton.Clear();
	ggNextCurrentButton.Clear();
	ggSpeedControlIncreaseButton.Clear();
	ggSpeedControlDecreaseButton.Clear();
	ggSpeedControlPowerIncreaseButton.Clear();
	ggSpeedControlPowerDecreaseButton.Clear();
	for (auto &speedctrlbutton : ggSpeedCtrlButtons)
	{
		speedctrlbutton.Clear();
	}
	for (auto &universal : ggUniversals)
	{
		universal.Clear();
	}
	for (auto &item : ggInverterEnableButtons)
	{
		item.Clear();
	}
	for (auto &item : ggInverterDisableButtons)
	{
		item.Clear();
	}
	for (auto &item : ggInverterToggleButtons)
	{
		item.Clear();
	}
	for (auto &relayresetbutton : ggRelayResetButtons)
	{
		relayresetbutton.Clear();
	}
	ggInstrumentLightButton.Clear();
	ggDashboardLightButton.Clear();
	ggTimetableLightButton.Clear();
	// hunter-091012
	ggCabLightDimButton.Clear();
	ggCompartmentLightsButton.Clear();
	ggCompartmentLightsOnButton.Clear();
	ggCompartmentLightsOffButton.Clear();
	ggBatteryButton.Clear();
	ggBatteryOnButton.Clear();
	ggBatteryOffButton.Clear();
	ggCabActivationButton.Clear();
	//-------
	ggFuseButton.Clear();
	ggConverterFuseButton.Clear();
	ggStLinOffButton.Clear();
	ggRadioChannelSelector.Clear();
	ggRadioChannelPrevious.Clear();
	ggRadioChannelNext.Clear();
	ggRadioStop.Clear();
	ggRadioTest.Clear();
	ggRadioCall1.Clear();
	ggRadioCall3.Clear();
	ggRadioVolumeSelector.Clear();
	ggRadioVolumePrevious.Clear();
	ggRadioVolumeNext.Clear();
	ggRadioOnButton.Clear();
	ggRadioOffButton.Clear();
	ggRadioFuseOnButton.Clear();
	ggRadioFuseOffButton.Clear();
	ggDoorLeftPermitButton.Clear();
	ggDoorRightPermitButton.Clear();
	ggDoorPermitPresetButton.Clear();
	ggDoorLeftButton.Clear();
	ggDoorRightButton.Clear();
	ggDoorLeftOnButton.Clear();
	ggDoorRightOnButton.Clear();
	ggDoorLeftOffButton.Clear();
	ggDoorRightOffButton.Clear();
	ggDoorAllOnButton.Clear();
	ggDoorAllOffButton.Clear();
	ggTrainHeatingButton.Clear();
	ggSignallingButton.Clear();
	ggDoorSignallingButton.Clear();
	ggDoorStepButton.Clear();
	ggDepartureSignalButton.Clear();
	ggCompressorButton.Clear();
	ggCompressorLocalButton.Clear();
	ggConverterButton.Clear();
	ggConverterOffButton.Clear();
	ggConverterLocalButton.Clear();
	ggMainButton.Clear();
	/*
	    ggPantFrontButton.Clear();
	    ggPantRearButton.Clear();
	    ggPantFrontButtonOff.Clear();
	    ggPantRearButtonOff.Clear();
	*/
	ggPantAllDownButton.Clear();
	ggPantSelectedButton.Clear();
	ggPantSelectedDownButton.Clear();
	ggPantValvesButton.Clear();
	ggPantCompressorButton.Clear();
	ggPantCompressorValve.Clear();

	ggPantValvesOff.Clear();
	ggPantValvesUpdate.Clear();

	ggI1B.Clear();
	ggI2B.Clear();
	ggI3B.Clear();
	ggItotalB.Clear();
	ggOilPressB.Clear();
	ggWater1TempB.Clear();

	ggClockSInd.Clear();
	ggClockMInd.Clear();
	ggClockHInd.Clear();
	ggEngineVoltage.Clear();
	ggLVoltage.Clear();
	ggMainGearStatus.Clear();
	ggIgnitionKey.Clear();

	ggWaterPumpBreakerButton.Clear();
	ggWaterPumpButton.Clear();
	ggWaterHeaterBreakerButton.Clear();
	ggWaterHeaterButton.Clear();
	ggWaterCircuitsLinkButton.Clear();
	ggFuelPumpButton.Clear();
	ggOilPumpButton.Clear();
	ggOilFuelPumpButton.Clear();
	ggTractionModeSwitch.Clear();
	ggGensetButton.Clear();
	ggMotorBlowersFrontButton.Clear();
	ggMotorBlowersRearButton.Clear();
	ggMotorBlowersAllOffButton.Clear();

	btLampkaPrzetw.Clear();
	btLampkaPrzetwB.Clear();
	btLampkaPrzetwBOff.Clear();
	btLampkaPrzetwAOff.Clear();
	btLampkaPrzekRozn.Clear();
	btLampkaPrzekRoznPom.Clear();
	btLampkaUkrotnienie.Clear();
	btLampkaHamPosp.Clear();
	btLampkaWylSzybkiOff.Clear();
	btLampkaWylSzybkiB.Clear();
	btLampkaWylSzybkiBOff.Clear();
	btLampkaMainBreakerReady.Clear();
	btLampkaMainBreakerBlinkingIfReady.Clear();
	btLampkaBezoporowa.Clear();
	btLampkaBezoporowaB.Clear();
	btLampkaMaxSila.Clear();
	btLampkaPrzekrMaxSila.Clear();
	btLampkaRadio.Clear();
	btLampkaRadioMessage.Clear();
	btLampkaRadioStop.Clear();
	btLampkaHamulecReczny.Clear();
	btLampkaBlokadaDrzwi.Clear();
	btLampkaDoorLockOff.Clear();
	for (auto &universal : btUniversals)
	{
		universal.Clear();
	}
	btInstrumentLight.Clear();
	btDashboardLight.Clear();
	btTimetableLight.Clear();
	btLampkaWentZaluzje.Clear();
	btLampkaDoorLeft.Clear();
	btLampkaDoorRight.Clear();
	btLampkaDepartureSignal.Clear();
	btLampkaRezerwa.Clear();
	btLampkaBoczniki.Clear();
	btLampkaBocznik1.Clear();
	btLampkaBocznik2.Clear();
	btLampkaBocznik3.Clear();
	btLampkaBocznik4.Clear();
	btLampkaGear1.Clear();
	btLampkaGear2.Clear();
	btLampkaHydroLockup.Clear();
	btLampkaRadiotelefon.Clear();
	btLampkaHamienie.Clear();
	btLampkaBrakingOff.Clear();
	btLampkaED.Clear();
	btLampkaBrakeProfileG.Clear();
	btLampkaBrakeProfileP.Clear();
	btLampkaBrakeProfileR.Clear();
	btLampkaSpringBrakeActive.Clear();
	btLampkaSpringBrakeInactive.Clear();
	btLampkaSprezarka.Clear();
	btLampkaSprezarkaB.Clear();
	btLampkaSprezarkaOff.Clear();
	btLampkaSprezarkaBOff.Clear();
	btLampkaFuelPumpOff.Clear();
	btLampkaOilPump.Clear();
	btLampkaNapNastHam.Clear();
	btLampkaOporyB.Clear();
	btLampkaStycznB.Clear();
	btLampkaHamowanie1zes.Clear();
	btLampkaHamowanie2zes.Clear();
	btLampkaNadmPrzetwB.Clear();
	btLampkaHVoltageB.Clear();
	btLampkaForward.Clear();
	btLampkaBackward.Clear();
	btLampkaNeutral.Clear();
	// light indicators
	btLampkaUpperLight.Clear();
	btLampkaLeftLight.Clear();
	btLampkaRightLight.Clear();
	btLampkaLeftEndLight.Clear();
	btLampkaRightEndLight.Clear();
	btLampkaRearUpperLight.Clear();
	btLampkaRearLeftLight.Clear();
	btLampkaRearRightLight.Clear();
	btLampkaRearLeftEndLight.Clear();
	btLampkaRearRightEndLight.Clear();
	// others
	btLampkaMalfunction.Clear();
	btLampkaMalfunctionB.Clear();
	btLampkaMotorBlowers.Clear();
	btLampkaCoolingFans.Clear();
	btLampkaLouvresClosed.Clear();
	btLampkaTempomat.Clear();
	btLampkaDistanceCounter.Clear();

	ggLeftLightButton.Clear();
	ggRightLightButton.Clear();
	ggUpperLightButton.Clear();
	ggDimHeadlightsButton.Clear();
	ggModernLightDimSw.Clear();
	ggLeftEndLightButton.Clear();
	ggRightEndLightButton.Clear();
	ggLightsButton.Clear();
	ggCompressorListButton.Clear();
	// hunter-230112
	ggRearLeftLightButton.Clear();
	ggRearRightLightButton.Clear();
	ggRearUpperLightButton.Clear();
	ggRearLeftEndLightButton.Clear();
	ggRearRightEndLightButton.Clear();
}

// NOTE: we can get rid of this function once we have per-cab persistent state
void TTrain::set_cab_controls(int const Cab)
{
	// switches
	// battery
	ggBatteryButton.PutValue(ggBatteryButton.type() == TGaugeType::push ? 0.5f : mvOccupied->Power24vIsAvailable ? 1.f : 0.f);
	// activation
	ggCabActivationButton.PutValue(ggCabActivationButton.type() == TGaugeType::push ? 0.5f : mvOccupied->IsCabMaster() ? 1.f : 0.f);
	// line breaker
	if (ggMainButton.SubModel != nullptr)
	{ // instead of single main button there can be on/off pair
		ggMainButton.PutValue(ggMainButton.type() == TGaugeType::push ? 0.5f : m_linebreakerstate != linebreaker::state::open ? 1.f : 0.f);
	}
	// line breakers of the other vehicles in multiple
	for (std::size_t index = 0; index < m_remotelinebreakers.size(); ++index)
	{
		auto &mainswitch{m_remotelinebreakers[index].main_switch};
		if (mainswitch.SubModel == nullptr)
		{
			continue;
		}
		if (mainswitch.type() == TGaugeType::push)
		{
			mainswitch.PutValue(0.5f);
			continue;
		}
		auto const *engine{remote_engine(index)};
		mainswitch.PutValue(engine != nullptr && engine_start::remote_switch::is_running(*engine) ? 1.f : 0.f);
	}

	if (ggModernLightDimSw.SubModel != nullptr)
	{
		mvOccupied->modernDimmerPosition = mvOccupied->modernDimmerDefaultPosition;
		ggModernLightDimSw.PutValue(static_cast<float>(mvOccupied->modernDimmerDefaultPosition));
	}

	// Init separate buttons
	if (ggPantValvesUpdate.SubModel != nullptr)
	{
		ggPantValvesUpdate.PutValue(0.f);
	}
	if (ggPantValvesOff.SubModel != nullptr)
	{
		ggPantValvesOff.PutValue(0.f);
	}

	// motor connectors
	ggStLinOffButton.PutValue(mvControlled->StLinSwitchOff ? 1.f : 0.f);
	// radio
	ggRadioChannelSelector.PutValue(static_cast<float>((Dynamic()->Mechanik ? Dynamic()->Mechanik->iRadioChannel : 1) - 1));
	// pantographs
	/*
	    if( mvOccupied->PantSwitchType != "impulse" ) {
	        if( ggPantFrontButton.SubModel ) {
	            ggPantFrontButton.PutValue(
	                ( mvControlled->Pantographs[end::front].valve.is_enabled ?
	                    1.f :
	                    0.f ) );
	        }
	        if( ggPantFrontButtonOff.SubModel ) {
	            ggPantFrontButtonOff.PutValue(
	                ( mvControlled->Pantographs[end::front].valve.is_disabled ?
	                    1.f :
	                    0.f ) );
	        }
	    }
	    if( mvOccupied->PantSwitchType != "impulse" ) {
	        if( ggPantRearButton.SubModel ) {
	            ggPantRearButton.PutValue(
	                ( mvControlled->Pantographs[end::rear].valve.is_enabled ?
	                    1.f :
	                    0.f ) );
	        }
	        if( ggPantRearButtonOff.SubModel ) {
	            ggPantRearButtonOff.PutValue(
	                ( mvControlled->Pantographs[end::rear].valve.is_disabled ?
	                    1.f :
	                    0.f ) );
	        }
	    }
	*/
	// front/end pantograph selection is relative to occupied cab
	if (ggPantSelectedButton.type() == TGaugeType::toggle)
	{
		ggPantSelectedButton.PutValue(mvPantographUnit->PantsValve.is_enabled ? 1.f : 0.f);
	}
	else
	{
		if (false == m_controlmapper.contains("pantselectedoff_sw:"))
		{
			// single impulse switch arrangement, with neutral position mid-way
			ggPantSelectedButton.PutValue(0.5f);
		}
	}
	if (ggPantSelectedDownButton.type() == TGaugeType::toggle)
	{
		ggPantSelectedDownButton.PutValue(mvPantographUnit->PantsValve.is_disabled ? 1.f : 0.f);
	}

	ggPantValvesButton.PutValue(0.5f);

	// auxiliary compressor
	ggPantCompressorValve.PutValue(mvControlled->bPantKurek3 ? 0.f : // default setting is pantographs connected with primary tank
	                                                           1.f);
	ggPantCompressorButton.PutValue(mvPantographUnit->PantCompFlag ? 1.f : 0.f);
	// converter
	if (mvOccupied->ConvSwitchType != "impulse")
	{
		ggConverterButton.PutValue(mvOccupied->ConverterAllow ? 1.f : 0.f);
	}
	ggConverterLocalButton.PutValue(mvOccupied->ConverterAllowLocal ? 1.f : 0.f);
	// compressor
	ggCompressorButton.PutValue(mvControlled->CompressorAllow ? 1.f : 0.f);
	ggCompressorLocalButton.PutValue(mvControlled->CompressorAllowLocal ? 1.f : 0.f);
	ggCompressorListButton.PutValue(static_cast<float>(mvOccupied->CompressorListPos - 1));
	// motor overload relay threshold / shunt mode
	ggMaxCurrentCtrl.PutValue(true == mvControlled->ShuntModeAllow ? (true == mvControlled->ShuntMode ? 1.f : 0.f) : mvControlled->MotorOverloadRelayHighThreshold ? 1.f : 0.f);
	// automatic relays (AutoRelay)
	ggAutoRelayButton.PutValue(mvOccupied->AutoRelayFlag ? 1.f : 0.f);
	// lights
	ggLightsButton.PutValue(static_cast<float>(mvOccupied->LightsPos - 1));

	auto const vehicleend{cab_to_end(Cab)};

	if ((mvOccupied->iLights[vehicleend] & light::headlight_left) != 0)
	{
		ggLeftLightButton.PutValue(1.f);
	}
	if ((mvOccupied->iLights[vehicleend] & light::headlight_right) != 0)
	{
		ggRightLightButton.PutValue(1.f);
	}
	if ((mvOccupied->iLights[vehicleend] & light::headlight_upper) != 0)
	{
		ggUpperLightButton.PutValue(1.f);
	}
	if ((mvOccupied->iLights[vehicleend] & light::redmarker_left) != 0)
	{
		if (ggLeftEndLightButton.SubModel != nullptr)
		{
			ggLeftEndLightButton.PutValue(1.f);
		}
		else
		{
			ggLeftLightButton.PutValue(-1.f);
		}
	}
	if ((mvOccupied->iLights[vehicleend] & light::redmarker_right) != 0)
	{
		if (ggRightEndLightButton.SubModel != nullptr)
		{
			ggRightEndLightButton.PutValue(1.f);
		}
		else
		{
			ggRightLightButton.PutValue(-1.f);
		}
	}
	if (1 == DynamicObject->MoverParameters->modernDimmerPosition)
	{
		ggDimHeadlightsButton.PutValue(static_cast<float>(DynamicObject->MoverParameters->modernDimmerPosition));
	}
	// cab lights
	if (true == Cabine[Cab].bLightDim)
	{
		ggCabLightDimButton.PutValue(1.f);
	}
	// compartment lights
	ggCompartmentLightsButton.PutValue(ggCompartmentLightsButton.type() == TGaugeType::push ? 0.5f : mvOccupied->CompartmentLights.is_enabled ? 1.f : 0.f);
	// instrument lights
	ggInstrumentLightButton.PutValue(InstrumentLightActive ? 1.f : 0.f);
	ggDashboardLightButton.PutValue(DashboardLightActive ? 1.f : 0.f);
	ggTimetableLightButton.PutValue(TimetableLightActive ? 1.f : 0.f);
	// doors permits
	if (false == ggDoorLeftPermitButton.is_push())
	{
		ggDoorLeftPermitButton.PutValue(mvOccupied->Doors.instances[cab_to_end() == end::front ? side::left : side::right].open_permit ? 1.f : 0.f);
	}
	if (false == ggDoorRightPermitButton.is_push())
	{
		ggDoorRightPermitButton.PutValue(mvOccupied->Doors.instances[cab_to_end() == end::front ? side::right : side::left].open_permit ? 1.f : 0.f);
	}
	ggDoorPermitPresetButton.PutValue(static_cast<float>(mvOccupied->Doors.permit_preset));
	// door controls
	ggDoorLeftButton.PutValue(mvOccupied->Doors.instances[cab_to_end() == end::front ? side::left : side::right].is_closed ? 0.f : 1.f);
	ggDoorRightButton.PutValue(mvOccupied->Doors.instances[cab_to_end() == end::front ? side::right : side::left].is_closed ? 0.f : 1.f);
	// door lock
	ggDoorSignallingButton.PutValue(mvOccupied->Doors.lock_enabled ? 1.f : 0.f);
	// door step
	if (false == ggDoorStepButton.is_push())
	{
		ggDoorStepButton.PutValue(mvOccupied->Doors.step_enabled ? 1.f : 0.f);
	}
	// heating
	if (false == ggTrainHeatingButton.is_push() && false == mvOccupied->HeatingSwitchImpulse)
	{
		ggTrainHeatingButton.PutValue(mvControlled->Heating ? 1.f : 0.f);
	}
	// brake acting time
	if (ggBrakeProfileCtrl.SubModel != nullptr)
	{
		ggBrakeProfileCtrl.PutValue((mvOccupied->BrakeDelayFlag & bdelay_R) != 0 ? 2.f : static_cast<float>(mvOccupied->BrakeDelayFlag - 1));
	}
	if (ggBrakeProfileG.SubModel != nullptr)
	{
		ggBrakeProfileG.PutValue(mvOccupied->BrakeDelayFlag == bdelay_G ? 1.f : 0.f);
	}
	if (ggBrakeProfileR.SubModel != nullptr)
	{
		ggBrakeProfileR.PutValue((mvOccupied->BrakeDelayFlag & bdelay_R) != 0 ? 1.f : 0.f);
	}

	if (ggWiperSw.SubModel != nullptr)
	{
		ggWiperSw.PutValue(static_cast<float>(mvOccupied->wiperSwitchPos));
	}

	if (ggBrakeOperationModeCtrl.SubModel != nullptr)
	{
		ggBrakeOperationModeCtrl.PutValue(mvOccupied->BrakeOpModeFlag > 0 ? static_cast<float>(std::log2(mvOccupied->BrakeOpModeFlag)) : 0);
	}
	// alarm chain
	ggAlarmChain.PutValue(mvControlled->AlarmChainFlag ? 1.f : 0.f);
	// brake signalling
	ggSignallingButton.PutValue(mvControlled->Signalling ? 1.f : 0.f);
	// multiple-unit current indicator source
	ggNextCurrentButton.PutValue(ShowNextCurrent ? 1.f : 0.f);
	// water pump
	ggWaterPumpBreakerButton.PutValue(mvControlled->WaterPump.breaker ? 1.f : 0.f);
	if (ggWaterPumpButton.type() != TGaugeType::push)
	{
		ggWaterPumpButton.PutValue(mvControlled->WaterPump.is_enabled ? 1.f : 0.f);
	}
	// water heater
	ggWaterHeaterBreakerButton.PutValue(mvControlled->WaterHeater.breaker ? 1.f : 0.f);
	ggWaterHeaterButton.PutValue(mvControlled->WaterHeater.is_enabled ? 1.f : 0.f);
	ggWaterCircuitsLinkButton.PutValue(mvControlled->WaterCircuitsLink ? 1.f : 0.f);
	// fuel pump
	if (ggFuelPumpButton.type() != TGaugeType::push)
	{
		ggFuelPumpButton.PutValue(mvControlled->FuelPump.is_enabled ? 1.f : 0.f);
	}
	// oil pump
	if (ggOilPumpButton.type() != TGaugeType::push)
	{
		ggOilPumpButton.PutValue(mvControlled->OilPump.is_enabled ? 1.f : 0.f);
	}
	// joint oil and fuel pump
	if (ggOilFuelPumpButton.type() != TGaugeType::push)
	{
		ggOilFuelPumpButton.PutValue(mvControlled->OilFuelPumpsEnabled() ? 1.f : 0.f);
	}
	// traction selector and genset of a dual-mode vehicle
	ggTractionModeSwitch.PutValue(mvControlled->GensetSelected() ? 1.f : 0.f);
	if (ggGensetButton.type() != TGaugeType::push)
	{
		ggGensetButton.PutValue(mvControlled->Genset.is_enabled ? 1.f : 0.f);
	}
	// traction motor fans
	if (ggMotorBlowersFrontButton.type() != TGaugeType::push)
	{
		ggMotorBlowersFrontButton.PutValue(mvControlled->MotorBlowers[end::front].is_enabled ? 1.f : 0.f);
	}
	if (ggMotorBlowersRearButton.type() != TGaugeType::push)
	{
		ggMotorBlowersRearButton.PutValue(mvControlled->MotorBlowers[end::rear].is_enabled ? 1.f : 0.f);
	}
	if (ggMotorBlowersAllOffButton.type() != TGaugeType::push)
	{
		ggMotorBlowersAllOffButton.PutValue(mvControlled->MotorBlowers[end::front].is_disabled || mvControlled->MotorBlowers[end::rear].is_disabled ? 1.f : 0.f);
	}
	// second controller
	if (ggScndCtrl.is_push())
	{
		ggScndCtrl.PutValue(ggScndCtrl.is_toggle() ? 0.5f : // pushtoggle is two-way control with neutral position in the middle
		                                             0.f); // push is on/off control, active while held down, due to legacy use
	}
	// tempomat
	if (false == ggScndCtrlButton.is_push())
	{
		ggScndCtrlButton.PutValue(mvControlled->IsSpeedCtrlEngaged() ? 1.f : 0.f);
	}
	// sandbox
	if (ggAutoSandButton.type() != TGaugeType::push)
	{
		ggAutoSandButton.PutValue(mvControlled->SandDoseAutoAllow ? 1.f : 0.f);
	}
	// radio
	ggRadioVolumeSelector.PutValue(m_radiovolume);

	// finding each inverter - not so optimal, but action ins performed only during changing cabin
	bool kier = DynamicObject->DirectionGet() * mvOccupied->CabOccupied > 0;
	auto const flag{CouplingSet::FromBits(DynamicObject->MoverParameters->InverterControlCouplerFlag)};
	int itemstart = 0;
	for ([[maybe_unused]] auto const &item : ggInverterToggleButtons) // for each button
	{
		int itemindex = itemstart;
		itemstart++;
		TDynamicObject *p = DynamicObject->GetFirstDynamic(mvOccupied->CabOccupied < 0 ? end::rear : end::front, flag);
		while (p)
		{
			if (p->MoverParameters->eimc[eimc_p_Pmax] > 1)
			{
				if (itemindex < p->MoverParameters->InvertersNo)
				{
					// visual feedback
					ggInverterToggleButtons[itemstart - 1].PutValue(p->MoverParameters->Inverters[itemindex].Activate ? 1.0 : 0.0);
					break;
				}
				else
				{
					itemindex -= p->MoverParameters->InvertersNo;
				}
			}
			p = kier ? p->Next(flag) : p->Prev(flag);
		}
	}

	// we reset all indicators, as they're set during the update pass
	// TODO: when cleaning up break setting indicator state into a separate function, so we can reuse it
}

// initializes a button matching provided label. returns: true if the label was found, false otherwise
// TODO: refactor the cabin controls into some sensible structure
bool TTrain::initialize_button(cParser &Parser, std::string const &Label, int const Cabindex)
{

	std::unordered_map<std::string, TButton &> const lights = {
	    {"i-maxft:", btLampkaMaxSila},
	    {"i-maxftt:", btLampkaPrzekrMaxSila},
	    {"i-radio:", btLampkaRadio},
	    {"i-radiomessage:", btLampkaRadioMessage},
	    {"i-radiostop:", btLampkaRadioStop},
	    {"i-manual_brake:", btLampkaHamulecReczny},
	    {"i-door_blocked:", btLampkaBlokadaDrzwi},
	    {"i-door_blockedoff:", btLampkaDoorLockOff},
	    {"i-slippery:", btLampkaPoslizg},
	    {"i-contactors:", btLampkaStyczn},
	    {"i-conv_ovld:", btLampkaNadmPrzetw},
	    {"i-converter:", btLampkaPrzetw},
	    {"i-converteroff:", btLampkaPrzetwOff},
	    {"i-converteraoff:", btLampkaPrzetwAOff},
	    {"i-converterb:", btLampkaPrzetwB},
	    {"i-converterboff:", btLampkaPrzetwBOff},
	    {"i-diff_relay:", btLampkaPrzekRozn},
	    {"i-diff_relay2:", btLampkaPrzekRoznPom},
	    {"i-motor_ovld:", btLampkaNadmSil},
	    {"i-train_controll:", btLampkaUkrotnienie},
	    {"i-brake_delay_r:", btLampkaHamPosp},
	    {"i-mainbreaker:", btLampkaWylSzybki},
	    {"i-mainbreakerb:", btLampkaWylSzybkiB},
	    {"i-mainbreakeroff:", btLampkaWylSzybkiOff},
	    {"i-mainbreakerboff:", btLampkaWylSzybkiBOff},
	    {"i-mainbreakerready:", btLampkaMainBreakerReady},
	    {"i-mainbreakerblinking:", btLampkaMainBreakerBlinkingIfReady},
	    {"i-vent_ovld:", btLampkaNadmWent},
	    {"i-comp_ovld:", btLampkaNadmSpr},
	    // overheat indicator lamps
	    {"i-oil_overheat:", btLampkaOilOverheat},
	    {"i-water_overheat:", btLampkaWaterOverheat},
	    {"i-wateraux_overheat:", btLampkaWaterAuxOverheat},
	    {"i-engine_overheat:", btLampkaEngineOverheat},
	    {"i-resistors:", btLampkaOpory},
	    {"i-no_resistors:", btLampkaBezoporowa},
	    {"i-no_resistors_b:", btLampkaBezoporowaB},
	    {"i-highcurrent:", btLampkaWysRozr},
	    {"i-vent_trim:", btLampkaWentZaluzje},
	    {"i-motorblowers:", btLampkaMotorBlowers},
	    {"i-coolingfans:", btLampkaCoolingFans},
	    {"i-louvresclosed:", btLampkaLouvresClosed},
	    {"i-tempomat:", btLampkaTempomat},
	    {"i-distancecounter:", btLampkaDistanceCounter},
	    {"i-trainheating:", btLampkaOgrzewanieSkladu},
	    {"i-security_aware:", btLampkaCzuwaka},
	    {"i-security_cabsignal:", btLampkaSHP},
	    {"i-security_aware_cabsignal:", btLampkaCzuwakaSHP},
	    {"i-door_left:", btLampkaDoorLeft},
	    {"i-door_right:", btLampkaDoorRight},
	    {"i-departure_signal:", btLampkaDepartureSignal},
	    {"i-reserve:", btLampkaRezerwa},
	    {"i-scnd:", btLampkaBoczniki},
	    {"i-scnd1:", btLampkaBocznik1},
	    {"i-scnd2:", btLampkaBocznik2},
	    {"i-scnd3:", btLampkaBocznik3},
	    {"i-scnd4:", btLampkaBocznik4},
	    {"i-gear1:", btLampkaGear1},
	    {"i-gear2:", btLampkaGear2},
	    {"i-hydrolockup:", btLampkaHydroLockup},
	    {"i-braking:", btLampkaHamienie},
	    {"i-brakingoff:", btLampkaBrakingOff},
	    {"i-dynamicbrake:", btLampkaED},
	    {"i-brakeprofileg:", btLampkaBrakeProfileG},
	    {"i-brakeprofilep:", btLampkaBrakeProfileP},
	    {"i-brakeprofiler:", btLampkaBrakeProfileR},
	    {"i-springbrakeactive:", btLampkaSpringBrakeActive},
	    {"i-springbrakeinactive:", btLampkaSpringBrakeInactive},
	    {"i-braking-ezt:", btLampkaHamowanie1zes},
	    {"i-braking-ezt2:", btLampkaHamowanie2zes},
	    {"i-compressor:", btLampkaSprezarka},
	    {"i-compressorb:", btLampkaSprezarkaB},
	    {"i-compressoroff:", btLampkaSprezarkaOff},
	    {"i-compressorboff:", btLampkaSprezarkaBOff},
	    {"i-fuelpumpoff:", btLampkaFuelPumpOff},
	    {"i-oilpump:", btLampkaOilPump},
	    {"i-voltbrake:", btLampkaNapNastHam},
	    {"i-resistorsb:", btLampkaOporyB},
	    {"i-contactorsb:", btLampkaStycznB},
	    {"i-conv_ovldb:", btLampkaNadmPrzetwB},
	    {"i-hvoltageb:", btLampkaHVoltageB},
	    {"i-malfunction:", btLampkaMalfunction},
	    {"i-malfunctionb:", btLampkaMalfunctionB},
	    {"i-forward:", btLampkaForward},
	    {"i-backward:", btLampkaBackward},
	    {"i-neutral:", btLampkaNeutral},
	    {"i-upperlight:", btLampkaUpperLight},
	    {"i-leftlight:", btLampkaLeftLight},
	    {"i-rightlight:", btLampkaRightLight},
	    {"i-leftend:", btLampkaLeftEndLight},
	    {"i-rightend:", btLampkaRightEndLight},
	    {"i-rearupperlight:", btLampkaRearUpperLight},
	    {"i-rearleftlight:", btLampkaRearLeftLight},
	    {"i-rearrightlight:", btLampkaRearRightLight},
	    {"i-rearleftend:", btLampkaRearLeftEndLight},
	    {"i-rearrightend:", btLampkaRearRightEndLight},
	    {"i-dashboardlight:", btDashboardLight},
	    {"i-timetablelight:", btTimetableLight},
	    {"i-universal0:", btUniversals[0]},
	    {"i-universal1:", btUniversals[1]},
	    {"i-universal2:", btUniversals[2]},
	    {"i-universal3:", btUniversals[3]},
	    {"i-universal4:", btUniversals[4]},
	    {"i-universal5:", btUniversals[5]},
	    {"i-universal6:", btUniversals[6]},
	    {"i-universal7:", btUniversals[7]},
	    {"i-universal8:", btUniversals[8]},
	    {"i-universal9:", btUniversals[9]},
	    {"i-cabactived:", btCabActived},
	    {"i-aklvents:", btAKLVents},
	    {"i-compressorany:", btCompressors},
	    {"i-edenabled", btEDenabled},
	};
	{
		auto lookup = lights.find(Label);
		if (lookup != lights.end())
		{
			lookup->second.Load(Parser, DynamicObject);
			return true;
		}
	}
	// TODO: move viable dedicated lights to the automatic light array
	std::unordered_map<std::string, bool const *> const autolights = {
	    {"i-doors:", &m_doors},
	    {"i-doorpermit_left:", &m_doorspermitleft},
	    {"i-doorpermit_right:", &m_doorspermitright},
	    {"i-doorpermit_any:", &m_doorpermits},
	    {"i-doorstep:", &mvOccupied->Doors.step_enabled},
	    {"i-mainpipelock:", &mvOccupied->LockPipe},
	    {"i-battery:", &mvOccupied->Power24vIsAvailable},
	    {"i-cablight:", &Cabine[iCabn].bLight},
	    {"i-compartmentlights:", &mvOccupied->CompartmentLights.is_active},
	};
	{
		auto lookup = autolights.find(Label);
		if (lookup != autolights.end())
		{
			auto &button = Cabine[Cabindex].Button(-1); // pierwsza wolna lampka
			button.Load(Parser, DynamicObject);
			button.AssignBool(lookup->second);
			return true;
		}
	}
	// custom lights
	if (Label == "i-instrumentlight:")
	{
		btInstrumentLight.Load(Parser, DynamicObject);
		InstrumentLightType = 0;
	}
	else if (Label == "i-instrumentlight_m:")
	{
		btInstrumentLight.Load(Parser, DynamicObject);
		InstrumentLightType = 1;
	}
	else if (Label == "i-instrumentlight_c:")
	{
		btInstrumentLight.Load(Parser, DynamicObject);
		InstrumentLightType = 2;
	}
	else if (Label == "i-instrumentlight_a:")
	{
		btInstrumentLight.Load(Parser, DynamicObject);
		InstrumentLightType = 3;
	}
	else if (Label == "i-instrumentlight_l:")
	{
		btInstrumentLight.Load(Parser, DynamicObject);
		InstrumentLightType = 4;
	}
	else if (Label == "i-doors:")
	{
		int i = Parser.getToken<int>() - 1;
		auto &button = Cabine[Cabindex].Button(-1); // pierwsza wolna lampka
		button.Load(Parser, DynamicObject);
		button.AssignBool(bDoors[0] + 3 * i);
	}
	else
	{
		// failed to match the label
		return false;
	}

	return true;
}

// initializes a gauge matching provided label. returns: true if the label was found, false otherwise
// TODO: refactor the cabin controls into some sensible structure
bool TTrain::initialize_gauge(cParser &Parser, std::string const &Label, int const Cabindex)
{

	std::unordered_map<std::string, TGauge &> const gauges = {{"jointctrl:", ggJointCtrl},
	                                                          {"mainctrl:", ggMainCtrl},
	                                                          {"scndctrl:", ggScndCtrl},
	                                                          {"dirkey:", ggDirKey},
	                                                          {"brakectrl:", ggBrakeCtrl},
	                                                          {"localbrake:", ggLocalBrake},
	                                                          {"alarmchain:", ggAlarmChain},
	                                                          {"brakeprofile_sw:", ggBrakeProfileCtrl},
	                                                          {"brakeprofileg_sw:", ggBrakeProfileG},
	                                                          {"brakeprofiler_sw:", ggBrakeProfileR},
	                                                          {"brakeopmode_sw:", ggBrakeOperationModeCtrl},
	                                                          {"maxcurrent_sw:", ggMaxCurrentCtrl},
	                                                          {"autorelay_sw:", ggAutoRelayButton},
	                                                          {"main_off_bt:", ggMainOffButton},
	                                                          {"main_on_bt:", ggMainOnButton},
	                                                          {"mainb_sw:", m_remotelinebreakers[0].main_switch},
	                                                          {"mainb_off_bt:", m_remotelinebreakers[0].off_button},
	                                                          {"mainb_on_bt:", m_remotelinebreakers[0].on_button},
	                                                          {"mainc_sw:", m_remotelinebreakers[1].main_switch},
	                                                          {"mainc_off_bt:", m_remotelinebreakers[1].off_button},
	                                                          {"mainc_on_bt:", m_remotelinebreakers[1].on_button},
	                                                          {"security_reset_bt:", ggSecurityResetButton},
	                                                          {"shp_reset_bt:", ggSHPResetButton},
	                                                          {"releaser_bt:", ggReleaserButton},
	                                                          {"springbrakeon_bt:", ggSpringBrakeOnButton},
	                                                          {"springbrakeoff_bt:", ggSpringBrakeOffButton},
	                                                          {"universalbrake1_bt:", ggUniveralBrakeButton1},
	                                                          {"universalbrake2_bt:", ggUniveralBrakeButton2},
	                                                          {"universalbrake3_bt:", ggUniveralBrakeButton3},
	                                                          {"epbrake_bt:", ggEPFuseButton},
	                                                          {"sand_bt:", ggSandButton},
	                                                          {"autosandallow_sw:", ggAutoSandButton},
	                                                          {"antislip_bt:", ggAntiSlipButton},
	                                                          {"horn_bt:", ggHornButton},
	                                                          {"hornlow_bt:", ggHornLowButton},
	                                                          {"hornhigh_bt:", ggHornHighButton},
	                                                          {"whistle_bt:", ggWhistleButton},
	                                                          {"helper_bt:", ggHelperButton},
	                                                          {"fuse_bt:", ggFuseButton},
	                                                          {"converterfuse_bt:", ggConverterFuseButton},
	                                                          {"stlinoff_bt:", ggStLinOffButton},
	                                                          {"doorpermitpreset_sw:", ggDoorPermitPresetButton},
	                                                          {"door_left_sw:", ggDoorLeftButton},
	                                                          {"door_right_sw:", ggDoorRightButton},
	                                                          {"doorlefton_sw:", ggDoorLeftOnButton},
	                                                          {"doorrighton_sw:", ggDoorRightOnButton},
	                                                          {"doorleftoff_sw:", ggDoorLeftOffButton},
	                                                          {"doorrightoff_sw:", ggDoorRightOffButton},
	                                                          {"doorallon_sw:", ggDoorAllOnButton},
	                                                          {"departure_signal_bt:", ggDepartureSignalButton},
	                                                          {"upperlight_sw:", ggUpperLightButton},
	                                                          {"leftlight_sw:", ggLeftLightButton},
	                                                          {"rightlight_sw:", ggRightLightButton},
	                                                          {"dimheadlights_sw:", ggDimHeadlightsButton},
	                                                          {"leftend_sw:", ggLeftEndLightButton},
	                                                          {"rightend_sw:", ggRightEndLightButton},
	                                                          {"lights_sw:", ggLightsButton},
	                                                          {"moderndimmer_sw:", ggModernLightDimSw},
	                                                          {"rearupperlight_sw:", ggRearUpperLightButton},
	                                                          {"rearleftlight_sw:", ggRearLeftLightButton},
	                                                          {"rearrightlight_sw:", ggRearRightLightButton},
	                                                          {"rearleftend_sw:", ggRearLeftEndLightButton},
	                                                          {"rearrightend_sw:", ggRearRightEndLightButton},
	                                                          {"compressor_sw:", ggCompressorButton},
	                                                          {"compressorlocal_sw:", ggCompressorLocalButton},
	                                                          {"compressorlist_sw:", ggCompressorListButton},
	                                                          {"converter_sw:", ggConverterButton},
	                                                          {"converterlocal_sw:", ggConverterLocalButton},
	                                                          {"converteroff_sw:", ggConverterOffButton},
	                                                          {"main_sw:", ggMainButton},
	                                                          {"waterpumpbreaker_sw:", ggWaterPumpBreakerButton},
	                                                          {"waterpump_sw:", ggWaterPumpButton},
	                                                          {"waterheaterbreaker_sw:", ggWaterHeaterBreakerButton},
	                                                          {"waterheater_sw:", ggWaterHeaterButton},
	                                                          {"water1tempb:", ggWater1TempB},
	                                                          {"watercircuitslink_sw:", ggWaterCircuitsLinkButton},
	                                                          {"fuelpump_sw:", ggFuelPumpButton},
	                                                          {"oilpump_sw:", ggOilPumpButton},
	                                                          {"oilfuelpump_sw:", ggOilFuelPumpButton},
	                                                          {"tractionmode_sw:", ggTractionModeSwitch},
	                                                          {"genset_sw:", ggGensetButton},
	                                                          {"oilpressb:", ggOilPressB},
	                                                          {"motorblowersfront_sw:", ggMotorBlowersFrontButton},
	                                                          {"motorblowersrear_sw:", ggMotorBlowersRearButton},
	                                                          {"motorblowersalloff_sw:", ggMotorBlowersAllOffButton},
	                                                          {"radiochannel_sw:", ggRadioChannelSelector},
	                                                          {"radiochannelprev_sw:", ggRadioChannelPrevious},
	                                                          {"radiochannelnext_sw:", ggRadioChannelNext},
	                                                          {"radiostop_sw:", ggRadioStop},
	                                                          {"radiotest_sw:", ggRadioTest},
	                                                          {"radiocall1_sw:", ggRadioCall1},
	                                                          {"radiocall3_sw:", ggRadioCall3},
	                                                          {"radiovolume_sw:", ggRadioVolumeSelector},
	                                                          {"radiovolumeprev_sw:", ggRadioVolumePrevious},
	                                                          {"radiovolumenext_sw:", ggRadioVolumeNext},
	                                                          {"radioon_sw:", ggRadioOnButton},
	                                                          {"radiooff_sw:", ggRadioOffButton},
	                                                          {"radiofuseon_sw:", ggRadioFuseOnButton},
	                                                          {"radiofuseoff_sw:", ggRadioFuseOffButton},
	                                                          /*
	                                                                  { "pantfront_sw:", ggPantFrontButton },
	                                                                  { "pantrear_sw:", ggPantRearButton },
	                                                                  { "pantfrontoff_sw:", ggPantFrontButtonOff },
	                                                                  { "pantrearoff_sw:", ggPantRearButtonOff },
	                                                          */
	                                                          {"pantalloff_sw:", ggPantAllDownButton},
	                                                          {"pantselected_sw:", ggPantSelectedButton},
	                                                          {"pantselectedoff_sw:", ggPantSelectedDownButton},
	                                                          {"pantvalves_sw:", ggPantValvesButton},
	                                                          {"pantcompressor_sw:", ggPantCompressorButton},
	                                                          {"pantcompressorvalve_sw:", ggPantCompressorValve},
	                                                          {"trainheating_sw:", ggTrainHeatingButton},
	                                                          {"signalling_sw:", ggSignallingButton},
	                                                          {"door_signalling_sw:", ggDoorSignallingButton},
	                                                          {"nextcurrent_sw:", ggNextCurrentButton},
	                                                          {"instrumentlight_sw:", ggInstrumentLightButton},
	                                                          {"dashboardlight_sw:", ggDashboardLightButton},
	                                                          {"timetablelight_sw:", ggTimetableLightButton},
	                                                          {"cablightdim_sw:", ggCabLightDimButton},
	                                                          {"compartmentlights_sw:", ggCompartmentLightsButton},
	                                                          {"compartmentlightson_sw:", ggCompartmentLightsOnButton},
	                                                          {"compartmentlightsoff_sw:", ggCompartmentLightsOffButton},
	                                                          {"battery_sw:", ggBatteryButton},
	                                                          {"batteryon_sw:", ggBatteryOnButton},
	                                                          {"batteryoff_sw:", ggBatteryOffButton},
	                                                          {"cabactivation_sw:", ggCabActivationButton},
	                                                          {"distancecounter_sw:", ggDistanceCounterButton},
	                                                          {"relayreset1_bt:", ggRelayResetButtons[0]},
	                                                          {"relayreset2_bt:", ggRelayResetButtons[1]},
	                                                          {"relayreset3_bt:", ggRelayResetButtons[2]},
	                                                          {"universal0:", ggUniversals[0]},
	                                                          {"universal1:", ggUniversals[1]},
	                                                          {"universal2:", ggUniversals[2]},
	                                                          {"universal3:", ggUniversals[3]},
	                                                          {"universal4:", ggUniversals[4]},
	                                                          {"universal5:", ggUniversals[5]},
	                                                          {"universal6:", ggUniversals[6]},
	                                                          {"universal7:", ggUniversals[7]},
	                                                          {"universal8:", ggUniversals[8]},
	                                                          {"universal9:", ggUniversals[9]},
	                                                          {"universal10:", ggUniversals[10]},
	                                                          {"universal11:", ggUniversals[11]},
	                                                          {"universal12:", ggUniversals[12]},
	                                                          {"universal13:", ggUniversals[13]},
	                                                          {"universal14:", ggUniversals[14]},
	                                                          {"universal15:", ggUniversals[15]},
	                                                          {"universal16:", ggUniversals[16]},
	                                                          {"universal17:", ggUniversals[17]},
	                                                          {"universal18:", ggUniversals[18]},
	                                                          {"universal19:", ggUniversals[19]},
	                                                          {"universal20:", ggUniversals[20]},
	                                                          {"universal21:", ggUniversals[21]},
	                                                          {"universal22:", ggUniversals[22]},
	                                                          {"universal23:", ggUniversals[23]},
	                                                          {"universal24:", ggUniversals[24]},
	                                                          {"universal25:", ggUniversals[25]},
	                                                          {"universal26:", ggUniversals[26]},
	                                                          {"universal27:", ggUniversals[27]},
	                                                          {"universal28:", ggUniversals[28]},
	                                                          {"universal29:", ggUniversals[29]},
	                                                          {"inverterenable1_bt:", ggInverterEnableButtons[0]},
	                                                          {"inverterenable2_bt:", ggInverterEnableButtons[1]},
	                                                          {"inverterenable3_bt:", ggInverterEnableButtons[2]},
	                                                          {"inverterenable4_bt:", ggInverterEnableButtons[3]},
	                                                          {"inverterenable5_bt:", ggInverterEnableButtons[4]},
	                                                          {"inverterenable6_bt:", ggInverterEnableButtons[5]},
	                                                          {"inverterenable7_bt:", ggInverterEnableButtons[6]},
	                                                          {"inverterenable8_bt:", ggInverterEnableButtons[7]},
	                                                          {"inverterenable9_bt:", ggInverterEnableButtons[8]},
	                                                          {"inverterenable10_bt:", ggInverterEnableButtons[9]},
	                                                          {"inverterenable11_bt:", ggInverterEnableButtons[10]},
	                                                          {"inverterenable12_bt:", ggInverterEnableButtons[11]},
	                                                          {"inverterdisable1_bt:", ggInverterDisableButtons[0]},
	                                                          {"inverterdisable2_bt:", ggInverterDisableButtons[1]},
	                                                          {"inverterdisable3_bt:", ggInverterDisableButtons[2]},
	                                                          {"inverterdisable4_bt:", ggInverterDisableButtons[3]},
	                                                          {"inverterdisable5_bt:", ggInverterDisableButtons[4]},
	                                                          {"inverterdisable6_bt:", ggInverterDisableButtons[5]},
	                                                          {"inverterdisable7_bt:", ggInverterDisableButtons[6]},
	                                                          {"inverterdisable8_bt:", ggInverterDisableButtons[7]},
	                                                          {"inverterdisable9_bt:", ggInverterDisableButtons[8]},
	                                                          {"inverterdisable10_bt:", ggInverterDisableButtons[9]},
	                                                          {"inverterdisable11_bt:", ggInverterDisableButtons[10]},
	                                                          {"inverterdisable12_bt:", ggInverterDisableButtons[11]},
	                                                          {"invertertoggle1_bt:", ggInverterToggleButtons[0]},
	                                                          {"invertertoggle2_bt:", ggInverterToggleButtons[1]},
	                                                          {"invertertoggle3_bt:", ggInverterToggleButtons[2]},
	                                                          {"invertertoggle4_bt:", ggInverterToggleButtons[3]},
	                                                          {"invertertoggle5_bt:", ggInverterToggleButtons[4]},
	                                                          {"invertertoggle6_bt:", ggInverterToggleButtons[5]},
	                                                          {"invertertoggle7_bt:", ggInverterToggleButtons[6]},
	                                                          {"invertertoggle8_bt:", ggInverterToggleButtons[7]},
	                                                          {"invertertoggle9_bt:", ggInverterToggleButtons[8]},
	                                                          {"invertertoggle10_bt:", ggInverterToggleButtons[9]},
	                                                          {"invertertoggle11_bt:", ggInverterToggleButtons[10]},
	                                                          {"invertertoggle12_bt:", ggInverterToggleButtons[11]},
	                                                          {"pantvalvesupdate_bt:", ggPantValvesUpdate},
	                                                          {"pantvalvesoff_bt:", ggPantValvesOff},
	                                                          {"wipers_sw:", ggWiperSw}};
	{
		auto const lookup{gauges.find(Label)};
		if (lookup != gauges.end())
		{
			lookup->second.Load(Parser, DynamicObject);
			m_controlmapper.insert(lookup->second, lookup->first);
			return true;
		}
	}
	// dedicated gauges with state-driven optional submodel
	// TODO: move viable gauges here
	// TODO: convert dedicated gauges to auto-allocated ones, replace dedicated references in command handlers to mapper lookups
	std::unordered_map<std::string, std::tuple<TGauge &, bool const *>> const stategauges = {
	    {"tempomat_sw:", {ggScndCtrlButton, &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"tempomatoff_sw:", {ggScndCtrlOffButton, &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedinc_bt:", {ggSpeedControlIncreaseButton, &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speeddec_bt:", {ggSpeedControlDecreaseButton, &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedctrlpowerinc_bt:", {ggSpeedControlPowerIncreaseButton, &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedctrlpowerdec_bt:", {ggSpeedControlPowerDecreaseButton, &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton0:", {ggSpeedCtrlButtons[0], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton1:", {ggSpeedCtrlButtons[1], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton2:", {ggSpeedCtrlButtons[2], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton3:", {ggSpeedCtrlButtons[3], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton4:", {ggSpeedCtrlButtons[4], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton5:", {ggSpeedCtrlButtons[5], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton6:", {ggSpeedCtrlButtons[6], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton7:", {ggSpeedCtrlButtons[7], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton8:", {ggSpeedCtrlButtons[8], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"speedbutton9:", {ggSpeedCtrlButtons[9], &mvOccupied->SpeedCtrlUnit.IsActive}},
	    {"doorleftpermit_sw:", {ggDoorLeftPermitButton, &m_doorspermitleft}},
	    {"doorrightpermit_sw:", {ggDoorRightPermitButton, &m_doorspermitright}},
	    {"dooralloff_sw:", {ggDoorAllOffButton, &m_doors}},
	    {"doorstep_sw:", {ggDoorStepButton, &mvOccupied->Doors.step_enabled}},
	    {"dirforward_bt:", {ggDirForwardButton, &m_dirforward}},
	    {"dirneutral_bt:", {ggDirNeutralButton, &m_dirneutral}},
	    {"dirbackward_bt:", {ggDirBackwardButton, &m_dirbackward}},
	};
	{
		auto const lookup{stategauges.find(Label)};
		if (lookup != stategauges.end())
		{
			auto &gauge{std::get<TGauge &>(lookup->second)};
			gauge.Load(Parser, DynamicObject);
			gauge.AssignState(std::get<bool const *>(lookup->second));
			m_controlmapper.insert(gauge, lookup->first);
			return true;
		}
	}
	// TODO: move viable dedicated gauges to the automatic array
	std::unordered_map<std::string, bool *> const autoboolgauges = {
	    {"doormode_sw:", &mvOccupied->Doors.remote_only},
	    {"coolingfans_sw:", &mvControlled->RVentForceOn},
	    {"pantfront_sw:", &mvPantographUnit->Pantographs[end::front].valve.is_enabled},
	    {"pantrear_sw:", &mvPantographUnit->Pantographs[end::rear].valve.is_enabled},
	    {"pantfrontoff_sw:", &mvPantographUnit->Pantographs[end::front].valve.is_disabled},
	    {"pantrearoff_sw:", &mvPantographUnit->Pantographs[end::rear].valve.is_disabled},
	    {"radio_sw:", &mvOccupied->Radio},
	    {"radiofuse_sw:", &mvOccupied->RadioFuse},
	    {"cablight_sw:", &Cabine[iCabn].bLight},
	    {"springbraketoggle_bt:", &mvOccupied->SpringBrake.Activate},
	    {"couplingdisconnect_sw:", &m_couplingdisconnect},
	    {"couplingdisconnectback_sw:", &m_couplingdisconnectback},
	    {"mirrors_sw:", &mvOccupied->MirrorForbidden},
	};
	{
		auto lookup = autoboolgauges.find(Label);
		if (lookup != autoboolgauges.end())
		{
			auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna lampka
			gauge.Load(Parser, DynamicObject);
			gauge.AssignBool(lookup->second);
			m_controlmapper.insert(gauge, lookup->first);
			return true;
		}
	}
	// TODO: move viable dedicated gauges to the automatic array
	std::unordered_map<std::string, int *> const autointgauges = {
	    {"manualbrake:", &mvOccupied->ManualBrakePos},
	    {"pantselect_sw:", &mvOccupied->PantsPreset.second[cab_to_end()]},
	};
	{
		auto lookup = autointgauges.find(Label);
		if (lookup != autointgauges.end())
		{
			auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna lampka
			gauge.Load(Parser, DynamicObject);
			gauge.AssignInt(lookup->second);
			m_controlmapper.insert(gauge, lookup->first);
			return true;
		}
	}

	// broadcast radio receiver
	if (Label == "fmradio_sw:" || Label == "fmradiovolume_sw:" || Label == "fmradiotune_sw:" || Label == "fmradioscale:")
	{
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		if (Label == "fmradio_sw:")
		{
			gauge.AssignBool(&m_fmradiopower);
		}
		else
		{
			gauge.AssignFloat(Label == "fmradiovolume_sw:" ? &m_fmradiovolume : &m_fmradiodial);
		}
		if (Label != "fmradioscale:")
		{
			// the scale pointer is an indicator, the rest are controls
			m_controlmapper.insert(gauge, Label);
		}
		m_fmradiopresent = true;
		// unless the speaker location is defined explicitly, the sound comes from the first defined element of the receiver
		if (m_fmradiosound.offset() == glm::vec3())
		{
			m_fmradiosound.offset(gauge.model_offset());
		}
		return true;
	}

	// ABu 090305: uniwersalne przyciski lub inne rzeczy
	if (Label == "mainctrlact:")
	{
		ggMainCtrlAct.Load(Parser, DynamicObject);
	}
	// SEKCJA WSKAZNIKOW
	else if (Label == "tachometer:" || Label == "tachometerb:")
	{
		// predkosciomierz wskaźnikowy z szarpaniem
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&fTachoVelocityJump);
		// bind tachometer sound location to the meter
		if (dsbHasler && dsbHasler->offset() == glm::vec3())
		{
			dsbHasler->offset(gauge.model_offset());
		}
	}
	else if (Label == "tachometern:")
	{
		// predkosciomierz wskaźnikowy bez szarpania
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&fTachoVelocity);
		// bind tachometer sound location to the meter
		if (dsbHasler && dsbHasler->offset() == glm::vec3())
		{
			dsbHasler->offset(gauge.model_offset());
		}
	}
	else if (Label == "tachometerd:")
	{
		// predkosciomierz cyfrowy
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&fTachoVelocity);
		// bind tachometer sound location to the meter
		if (dsbHasler && dsbHasler->offset() == glm::vec3())
		{
			dsbHasler->offset(gauge.model_offset());
		}
	}
	else if (Label == "hvcurrent1:" || Label == "hvcurrent1b:")
	{
		// 1szy amperomierz
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(fHCurrent + 1);
	}
	else if (Label == "hvcurrent2:" || Label == "hvcurrent2b:")
	{
		// 2gi amperomierz
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(fHCurrent + 2);
	}
	else if (Label == "hvcurrent3:" || Label == "hvcurrent3b:")
	{
		// 3ci amperomierz
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałska
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(fHCurrent + 3);
	}
	else if (Label == "hvcurrent:" || Label == "hvcurrentb:")
	{
		// amperomierz calkowitego pradu
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(fHCurrent);
	}
	else if (Label == "eimscreen:")
	{
		// amperomierz calkowitego pradu
		int i;
		int j;
		Parser.getTokens(2, false);
		Parser >> i >> j;
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&fEIMParams[i][j]);
	}
	else if (Label == "brakes:")
	{
		// specified pipe pressure of specified consist vehicle
		int i;
		int j;
		Parser.getTokens(2, false);
		Parser >> i >> j;
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignFloat(&fPress[std::clamp(i, 1, 20) - 1][std::clamp(j, 0, 3)]);
	}
	else if (Label == "brakepress:" || Label == "brakepressb:")
	{
		// manometr cylindrow hamulcowych
		// Ra 2014-08: przeniesione do TCab
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignDouble(&mvOccupied->BrakePress);
	}
	else if (Label == "pipepress:" || Label == "pipepressb:")
	{
		// manometr przewodu hamulcowego
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignDouble(&mvOccupied->PipePress);
	}
	else if (Label == "scndpress:")
	{
		// manometr przewodu hamulcowego
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignDouble(&mvOccupied->ScndPipePress);
	}
	else if (Label == "limpipepress:")
	{
		// manometr zbiornika sterujacego zaworu maszynisty
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignDouble(&m_brakehandlecp);
	}
	else if (Label == "cntrlpress:")
	{
		// manometr zbiornika kontrolnego/rorzďż˝du
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignDouble(&mvPantographUnit->PantPress);
	}
	else if (Label == "springbrakepress:")
	{
		// manometr cylindra hamulca sprężynowego
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignDouble(&mvOccupied->SpringBrake.SBP);
	}
	else if (Label == "epctrlvalue:")
	{
		// wskazowka sterowania sila hamulca ep
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignDouble(&mvOccupied->EpForce);
	}
	else if (Label == "compressor:" || Label == "compressorb:")
	{
		// manometr sprezarki/zbiornika glownego
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignDouble(&mvOccupied->Compressor);
	}
	else if (Label == "oilpress:")
	{
		// oil pressure
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&mvControlled->OilPump.pressure);
	}
	else if (Label == "oiltemp:")
	{
		// oil temperature
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&mvControlled->dizel_heat.To);
	}
	else if (Label == "water1temp:")
	{
		// main circuit water temperature
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&mvControlled->dizel_heat.temperatura1);
	}
	else if (Label == "water2temp:")
	{
		// auxiliary circuit water temperature
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&mvControlled->dizel_heat.temperatura2);
	}
	else if (Label == "pantpress:")
	{
		// pantograph tank pressure
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject, 0.1);
		gauge.AssignDouble(&mvPantographUnit->PantPress);
	}
	// yB - dla drugiej sekcji
	else if (Label == "hvbcurrent1:")
	{
		// 1szy amperomierz
		ggI1B.Load(Parser, DynamicObject);
	}
	else if (Label == "hvbcurrent2:")
	{
		// 2gi amperomierz
		ggI2B.Load(Parser, DynamicObject);
	}
	else if (Label == "hvbcurrent3:")
	{
		// 3ci amperomierz
		ggI3B.Load(Parser, DynamicObject);
	}
	else if (Label == "hvbcurrent:")
	{
		// amperomierz calkowitego pradu
		ggItotalB.Load(Parser, DynamicObject);
	}
	//*************************************************************
	else if (Label == "clock:")
	{
		// zegar analogowy
		if (Parser.getToken<std::string>() == "analog" && DynamicObject->mdKabina)
		{
			// McZapkie-300302: zegarek
			ggClockSInd.Init(DynamicObject->mdKabina->GetFromName("ClockShand"), nullptr, TGaugeAnimation::gt_Rotate, static_cast<float>(1.0 / 60.0));
			ggClockMInd.Init(DynamicObject->mdKabina->GetFromName("ClockMhand"), nullptr, TGaugeAnimation::gt_Rotate, static_cast<float>(1.0 / 60.0));
			ggClockHInd.Init(DynamicObject->mdKabina->GetFromName("ClockHhand"), nullptr, TGaugeAnimation::gt_Rotate, static_cast<float>(1.0 / 12.0));
		}
	}
	else if (Label == "clock_seconds:")
	{
		ggClockSInd.Load(Parser, DynamicObject);
	}
	else if (Label == "evoltage:")
	{
		// woltomierz napiecia silnikow
		ggEngineVoltage.Load(Parser, DynamicObject);
	}
	else if (Label == "hvoltage:")
	{
		// woltomierz wysokiego napiecia
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&fHVoltage);
	}
	else if (Label == "lvoltage:")
	{
		// woltomierz niskiego napiecia
		ggLVoltage.Load(Parser, DynamicObject);
	}
	else if (Label == "enrot1m:")
	{
		// obrotomierz
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(fEngine + 1);
	} // ggEnrot1m.Load(Parser,DynamicObject->mdKabina);
	else if (Label == "enrot2m:")
	{
		// obrotomierz
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(fEngine + 2);
	} // ggEnrot2m.Load(Parser,DynamicObject->mdKabina);
	else if (Label == "enrot3m:")
	{ // obrotomierz
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(fEngine + 3);
	} // ggEnrot3m.Load(Parser,DynamicObject->mdKabina);
	else if (Label == "engageratio:")
	{
		// np. ciśnienie sterownika sprzęgła
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignDouble(&mvControlled->dizel_engage);
	} // ggEngageRatio.Load(Parser,DynamicObject->mdKabina);
	else if (Label == "maingearstatus:")
	{
		// np. ciśnienie sterownika skrzyni biegów
		ggMainGearStatus.Load(Parser, DynamicObject);
	}
	else if (Label == "ignitionkey:")
	{
		ggIgnitionKey.Load(Parser, DynamicObject);
	}
	else if (Label == "distcounter:")
	{
		// Ra 2014-07: licznik kilometrów
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		// the counter shows the odometer of the vehicle, kept between the sessions (eu07/maszyna#72). its offset
		// was the state it started each session with, now it is that only for a vehicle with no known state
		if (auto const seed{odometer::counter_seed(gauge.offset(), gauge.scale())}; seed.has_value())
			mvControlled->Odometer.seed(*seed);
		gauge.offset(0.f);
		gauge.AssignDouble(&mvControlled->Odometer.km);
		// the vehicle the counter counts for is one the odometer store keeps, also in the sessions the counter seeds
		// nothing (its state is a saved one by then) and when it's the motor car of the unit the cab is in
		mvControlled->Odometer.shown = true;
	}
	else if (Label == "shuntmodepower:")
	{
		// shunt mode power slider
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignDouble(&mvControlled->AnPos);
		m_controlmapper.insert(gauge, "shuntmodepower:");
	}
	else if (Label == "heatingvoltage:")
	{
		if (mvControlled->HeatingPowerSource.SourceType == TPowerSource::Generator)
		{
			auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
			gauge.Load(Parser, DynamicObject);
			gauge.AssignDouble(&mvControlled->HeatingPowerSource.EngineGenerator.voltage);
		}
	}
	else if (Label == "heatingcurrent:")
	{
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignDouble(&mvControlled->TotalCurrent);
	}
	else
	{
		// failed to match the label
		return false;
	}

	return true;
}
