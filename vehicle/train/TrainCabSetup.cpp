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
#include "vehicle/Camera.h"
#include "utilities/Logs.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "utilities/Timer.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "application/application.h"
#include "rendering/renderer.h"
#include <future>
#include <cmath>
#include <algorithm>



// cab movement update, fixed step part
void TTrain::UpdateCab()
{

	// Ra: przesiadka, jeśli AI zmieniło kabinę (a człon?)...
	if (DynamicObject->Mechanik // może nie być?
	    && DynamicObject->Mechanik->AIControllFlag)
	{

		if (iCabn != ( // numer kabiny (-1: kabina B)
		                 mvOccupied->CabOccupied == -1 ? 2 : mvOccupied->CabOccupied))
		{

			InitializeCab(mvOccupied->CabOccupied, mvOccupied->TypeName + ".mmd");
		}
	}
	iCabn = mvOccupied->CabOccupied == -1 ? 2 : mvOccupied->CabOccupied;
}

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

bool TTrain::CabChange(int iDirection)
{ // McZapkie-090902: zmiana kabiny 1->0->2 i z powrotem
	if (DynamicObject->Mechanik == nullptr || true == DynamicObject->Mechanik->AIControllFlag)
	{
		// jeśli prowadzi AI albo jest w innym członie
		// jak AI prowadzi, to nie można mu mieszać
		if (std::abs(mvOccupied->CabOccupied + iDirection) > 1)
			return false; // ewentualna zmiana pojazdu
		mvOccupied->CabOccupied += iDirection;
	}
	else
	{ // jeśli pojazd prowadzony ręcznie albo wcale (wagon)
		mvOccupied->CabDeactivisationAuto();
		if (mvOccupied->ChangeCab(iDirection))
		{
			if (InitializeCab(mvOccupied->CabOccupied, mvOccupied->TypeName + ".mmd"))
			{
				// zmiana kabiny w ramach tego samego pojazdu
				mvOccupied->CabActivisationAuto(); // załączenie rozrządu (wirtualne kabiny)
				DynamicObject->Mechanik->DirectionChange();
				return true; // udało się zmienić kabinę
			}
		}
		// aktywizacja poprzedniej, bo jeszcze nie wiadomo, czy jakiś pojazd jest
		mvOccupied->CabActivisationAuto();
	}
	return false; // ewentualna zmiana pojazdu
}

// McZapkie-310302
// wczytywanie pliku z danymi multimedialnymi (dzwieki, kontrolki, kabiny)
bool TTrain::LoadMMediaFile(std::string const &asFileName)
{
	// initialize sounds so potential entries from previous vehicle don't stick around
	std::unordered_map<std::string, std::tuple<std::optional<sound_source> &, sound_placement, float, sound_type, int, double>> internalsounds = {
	    {"ctrl:", {dsbNastawnikJazdy, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"ctrlscnd:", {dsbNastawnikBocz, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"reverserkey:", {dsbReverserKey, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"buzzer:", {dsbBuzzer, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"buzzershp:", {dsbBuzzerShp, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"radiostop:", {m_radiostop, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"slipalarm:", {dsbSlipAlarm, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"distancecounter:", {m_distancecounterclear, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"tachoclock:", {dsbHasler, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"switch:", {dsbSwitch, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"pneumaticswitch:", {dsbPneumaticSwitch, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"airsound:", {rsHiss, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, sound_parameters::amplitude, 100.0}},
	    {"airsound2:", {rsHissU, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, sound_parameters::amplitude, 100.0}},
	    {"airsound3:", {rsHissE, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, sound_parameters::amplitude, 100.0}},
	    {"airsound4:", {rsHissX, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, sound_parameters::amplitude, 100.0}},
	    {"airsound5:", {rsHissT, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, sound_parameters::amplitude, 100.0}},
	    {"localbrakesound:", {rsSBHiss, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, sound_parameters::amplitude, 100.0}},
	    {"localbrakesound2:", {rsSBHissU, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, sound_parameters::amplitude, 100.0}},
	    {"brakesound:", {rsBrake, sound_placement::internal, -1, sound_type::single, sound_parameters::amplitude | sound_parameters::frequency, 100.0}},
	    {"fadesound:", {rsFadeSound, sound_placement::internal, EU07_SOUND_CABCONTROLSCUTOFFRANGE, sound_type::single, 0, 100.0}},
	    {"runningnoise:", {rsRunningNoise, sound_placement::internal, EU07_SOUND_GLOBALRANGE, sound_type::single, sound_parameters::amplitude | sound_parameters::frequency, mvOccupied->Vmax}},
	    {"resonancenoise:", {rsResonanceNoise, sound_placement::internal, EU07_SOUND_GLOBALRANGE, sound_type::single, sound_parameters::amplitude | sound_parameters::frequency, mvOccupied->Vmax}},
	    {"windsound:", {rsWindSound, sound_placement::internal, EU07_SOUND_GLOBALRANGE, sound_type::single, sound_parameters::amplitude | sound_parameters::frequency, mvOccupied->Vmax}},
	    {"huntingnoise:", {rsHuntingNoise, sound_placement::internal, EU07_SOUND_GLOBALRANGE, sound_type::single, sound_parameters::amplitude | sound_parameters::frequency, mvOccupied->Vmax}},
	    {"rainsound:", {m_rainsound, sound_placement::internal, -1, sound_type::single, 0, 100.0}},
	};
	for (auto &soundconfig : internalsounds)
	{
		std::get<std::optional<sound_source> &>(soundconfig.second).reset();
	}
	// NOTE: since radiosound is an incomplete template not using std::optional it gets a special treatment
	m_radiosound.owner(DynamicObject);
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
			sound.deserialize(parser, std::get<sound_type>(soundconfig), std::get<int>(soundconfig), std::get<double>(soundconfig));
			sound.owner(DynamicObject);
			std::get<std::optional<sound_source> &>(soundconfig) = sound;
		} while (token != "");

		// assign default samples to sound emitters which weren't included in the config file
		if (!m_rainsound)
		{
			sound_source rainsound;
			rainsound.deserialize("rainsound_default", sound_type::single);
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
			rsBrake->m_frequencyfactor /= 1 + mvOccupied->Vmax;
		}
		if (rsResonanceNoise)
		{
			rsResonanceNoise->m_frequencyfactor /= 1 + mvOccupied->Vmax;
		}
		if (rsWindSound)
		{
			rsWindSound->m_frequencyfactor /= 1 + mvOccupied->Vmax;
		}
		if (rsRunningNoise)
		{
			rsRunningNoise->m_frequencyfactor /= 1 + mvOccupied->Vmax;
		}
		if (rsHuntingNoise)
		{
			rsHuntingNoise->m_frequencyfactor /= 1 + mvOccupied->Vmax;
		}
	}
	auto const nullvector{glm::vec3()};
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
	// reset sound positions
	auto const nullvector{glm::vec3()};
	std::vector<std::reference_wrapper<std::optional<sound_source>>> sounds = {
	    dsbReverserKey, dsbNastawnikJazdy, dsbNastawnikBocz, dsbSwitch,        dsbPneumaticSwitch, rsHiss,         rsHissU,   rsHissE,   rsHissX,      rsHissT,      rsSBHiss,
	    rsSBHissU,      rsFadeSound,       rsRunningNoise,   rsResonanceNoise, rsWindSound,        rsHuntingNoise, dsbHasler, dsbBuzzer, dsbBuzzerShp, dsbSlipAlarm, m_distancecounterclear,
	    m_rainsound,    m_radiostop};
	for (auto &sound : sounds)
	{
		if (sound.get())
		{
			sound.get()->offset(nullvector);
		}
	}
	m_radiosound.offset(nullvector);
	for (auto &sound : CabSoundLocations)
	{
		if (sound.first.get() && sound.first.get()->offset() == nullvector)
		{
			sound.first.get()->offset(sound.second);
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
				std::replace(token.begin(), token.end(), '\\', '/');
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
					submodel = DynamicObject->mdKabina     ? DynamicObject->mdKabina->GetFromName(screen.target) :
					           DynamicObject->mdLowPolyInt ? DynamicObject->mdLowPolyInt->GetFromName(screen.target) :
					                                         nullptr;
					if (submodel == nullptr)
					{
						WriteLog("Python Screen: submodel " + screen.target + " not found - Ignoring screen");
						continue;
					}
					auto const material{submodel->GetMaterial()};
					if (material <= 0)
					{
						// sub model nie posiada tekstury lub tekstura wymienna - nie obslugiwana
						WriteLog("Python Screen: invalid texture id " + std::to_string(material) + " - Ignoring screen");
						continue;
					}

					tex = &GfxRenderer->Texture(GfxRenderer->Material(material)->GetTexture(0));
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
			else if (token == "pyscreenupdatetime:")
			{
				parser.getTokens();
				parser >> ScreenUpdateRate;
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
	for (auto &sound : soundlocations)
	{
		if (sound.first.get() && sound.first.get()->offset() == nullvector)
		{
			sound.first.get()->offset(sound.second);
		}
	}
	// second pass, in case some items received no positioning due to missing submodels etc
	for (auto &sound : soundlocations)
	{
		if (sound.first.get() && sound.first.get()->offset() == nullvector)
		{
			sound.first.get()->offset(caboffset);
		}
	}

	if (DynamicObject->mdKabina)
		DynamicObject->mdKabina->Init(); // obrócenie modelu oraz optymalizacja, również zapisanie binarnego

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

void TTrain::DynamicSet(TDynamicObject *d)
{ // taka proteza: chcę podłączyć
	// kabinę EN57 bezpośrednio z
	// silnikowym, aby nie robić tego
	// przez ukrotnienie
	// drugi silnikowy i tak musi być ukrotniony, podobnie jak kolejna jednostka
	// problem się robi ze światłami, które będą zapalane w silnikowym, ale muszą
	// świecić się w rozrządczych
	// dla EZT światła czołowe będą "zapalane w silnikowym", ale widziane z
	// rozrządczych
	// również wczytywanie MMD powinno dotyczyć aktualnego członu
	// problematyczna może być kwestia wybranej kabiny (w silnikowym...)
	// jeśli silnikowy będzie zapięty odwrotnie (tzn. -1), to i tak powinno
	// jeździć dobrze
	// również hamowanie wykonuje się zaworem w członie, a nie w silnikowym...
	DynamicObject = d; // jedyne miejsce zmiany
	mvOccupied = mvControlled = d ? DynamicObject->MoverParameters : nullptr; // albo silnikowy w EZT

	if (DynamicObject == nullptr)
	{
		return;
	}

	mvControlled = DynamicObject->FindPowered()->MoverParameters;
	mvSecond = nullptr; // gdyby się nic nie znalazło
	if (mvOccupied->Power > 1.0) // dwuczłonowe lub ukrotnienia, żeby nie szukać każdorazowo
		if (mvOccupied->Couplers[1].Connected ? mvOccupied->Couplers[1].AllowedFlag & coupling::control : false)
		{ // gdy jest człon od sprzęgu 1, a sprzęg łączony
			// warsztatowo (powiedzmy)
			if (mvOccupied->Couplers[1].Connected->Power > 1.0) // ten drugi ma moc
				mvSecond = (TMoverParameters *)mvOccupied->Couplers[1].Connected; // wskaźnik na drugiego
		}
		else if (mvOccupied->Couplers[0].Connected ? mvOccupied->Couplers[0].AllowedFlag & coupling::control : false)
		{ // gdy jest człon od sprzęgu 0, a sprzęg łączony
			// warsztatowo (powiedzmy)
			if (mvOccupied->Couplers[0].Connected->Power > 1.0) // ale ten drugi ma moc
				mvSecond = (TMoverParameters *)mvOccupied->Couplers[0].Connected; // wskaźnik na drugiego
		}
	// cache nearest unit equipped with pantographs
	{
		auto *lookup{DynamicObject->FindPantographCarrier()};
		// HACK: set pointer to existing vehicle to avoid error checking all over the place
		mvPantographUnit = lookup != nullptr ? lookup->MoverParameters : mvControlled;
	}
};

void TTrain::MoveToVehicle(TDynamicObject *target)
{
	// > Ra: to nie może być tak robione, to zbytnia proteza jest
	// indeed, too much hacks...
	// TODO: cleanup
	TTrain *target_train = simulation::Trains.find(target->name());
	if (target_train)
	{
		// let's try to destroy this TTrain and move to already existing one

		if (!Dynamic()->Mechanik || !Dynamic()->Mechanik->AIControllFlag)
		{
			// tylko jeśli ręcznie prowadzony
			// jeśli prowadzi AI, to mu nie robimy dywersji!
			Occupied()->CabDeactivisation();
			Occupied()->CabOccupied = 0;
			Occupied()->BrakeLevelSet(Occupied()->Handle->GetPos(bh_NP)); // rozwala sterowanie hamulcem GF 04-2016
			Occupied()->MainCtrlPos = Occupied()->MainCtrlNoPowerPos();
			Occupied()->ScndCtrlPos = 0;
			Dynamic()->MechInside = false;
			Dynamic()->Controller = AIdriver;

			Dynamic()->bDisplayCab = false;
			Dynamic()->ABuSetModelShake({});

			if (Dynamic()->Mechanik)
				Dynamic()->Mechanik->MoveTo(target);

			target_train->Occupied()->LimPipePress = target_train->Occupied()->PipePress;
			target_train->Occupied()->CabActivisationAuto(true); // załączenie rozrządu (wirtualne kabiny)
			target_train->Dynamic()->MechInside = true;
			if (target_train->Dynamic()->Mechanik)
			{
				target_train->Dynamic()->Controller = target_train->Dynamic()->Mechanik->AIControllFlag;
				target_train->Dynamic()->Mechanik->DirectionChange();
			}
			else
			{
				target_train->Dynamic()->Controller = Humandriver;
			}
		}
		else
		{
			target_train->Dynamic()->bDisplayCab = false;
			target_train->Dynamic()->ABuSetModelShake({});
		}

		target_train->Dynamic()->ABuSetModelShake({}); // zerowanie przesunięcia przed powrotem?

		// potentially move player
		if (simulation::Train == this)
		{
			simulation::Train = target_train;
			// our local driver may potentially be in external view mode, in which case we shouldn't activate cab visualization
			target_train->Dynamic()->bDisplayCab |= !FreeFlyModeFlag;
		}

		// delete this TTrain
		pending_delete = true;
	}
	else
	{
		// move this TTrain to other dynamic

		// remove TTrain from global list, we're going to change dynamic anyway
		simulation::Trains.detach(Dynamic()->name());

		if (!Dynamic()->Mechanik || !Dynamic()->Mechanik->AIControllFlag)
		{
			// tylko jeśli ręcznie prowadzony
			// jeśli prowadzi AI, to mu nie robimy dywersji!

			Occupied()->CabDeactivisation();
			Occupied()->CabOccupied = 0;
			Occupied()->BrakeLevelSet(Occupied()->Handle->GetPos(bh_NP)); // rozwala sterowanie hamulcem GF 04-2016
			Occupied()->MainCtrlPos = Occupied()->MainCtrlNoPowerPos();
			Occupied()->ScndCtrlPos = 0;
			Dynamic()->MechInside = false;
			Dynamic()->Controller = AIdriver;

			Dynamic()->bDisplayCab = false;
			Dynamic()->ABuSetModelShake({});

			if (Dynamic()->Mechanik)
				Dynamic()->Mechanik->MoveTo(target);

			DynamicSet(target);

			Dynamic()->MechInside = true;
			if (Dynamic()->Mechanik)
			{
				Dynamic()->Controller = Dynamic()->Mechanik->AIControllFlag;
				Dynamic()->Mechanik->DirectionChange();
			}
			else
			{
				Dynamic()->Controller = Humandriver;
			}

			Occupied()->LimPipePress = Occupied()->PipePress;
			Occupied()->CabActivisationAuto(true); // załączenie rozrządu (wirtualne kabiny)
		}
		else
		{
			Dynamic()->bDisplayCab = false;
			Dynamic()->ABuSetModelShake({});

			DynamicSet(target);
		}

		{
			auto const filename{Occupied()->TypeName + ".mmd"};
			LoadMMediaFile(filename);
			InitializeCab(Occupied()->CabActive, filename);
		}

		Dynamic()->ABuSetModelShake({}); // zerowanie przesunięcia przed powrotem?

		if (simulation::Train == this)
		{
			// our local driver may potentially be in external view mode, in which case we shouldn't activate cab visualization
			Dynamic()->bDisplayCab |= !FreeFlyModeFlag;
		}

		// add it back with updated dynamic name
		simulation::Trains.insert(this);
	}
}

// checks whether specified point is within boundaries of the active cab
bool TTrain::point_inside(glm::dvec3 const Point) const
{

	return Point.x >= Cabine[iCabn].CabPos1.x && Point.x <= Cabine[iCabn].CabPos2.x && Point.y >= Cabine[iCabn].CabPos1.y + 0.5 && Point.y <= Cabine[iCabn].CabPos2.y + 1.8 &&
	       Point.z >= Cabine[iCabn].CabPos1.z && Point.z <= Cabine[iCabn].CabPos2.z;
}

glm::dvec3 TTrain::clamp_inside(glm::dvec3 const &Point) const
{

	if (DebugModeFlag)
	{
		return Point;
	}

	return {std::clamp(Point.x, (double)Cabine[iCabn].CabPos1.x, (double)Cabine[iCabn].CabPos2.x), std::clamp(Point.y, (double)Cabine[iCabn].CabPos1.y + 0.5, (double)Cabine[iCabn].CabPos2.y + 1.8),
	        std::clamp(Point.z, (double)Cabine[iCabn].CabPos1.z, (double)Cabine[iCabn].CabPos2.z)};
}

const TTrain::screenentry_sequence &TTrain::get_screens()
{
	update_screens(Timer::GetDeltaTime());
	return m_screens;
}

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
	btLampkaNadmPrzetw.Clear((mvControlled->TrainType & dt_EZT) != 0 ? -1 : 7); // EN57 nie ma tej lampki
	btLampkaPrzetwOff.Clear((mvControlled->TrainType & dt_EZT) != 0 ? 7 : -1); // za to ma tę
	btLampkaNadmSpr.Clear(8);
	btLampkaNadmWent.Clear(9);
	btLampkaWysRozr.Clear((mvControlled->TrainType & dt_ET22) != 0 ? -1 : 10); // ET22 nie ma tej lampki
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
	ggMainOffButton.Clear();
	ggMainOnButton.Clear();
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
	ggMotorBlowersFrontButton.Clear();
	ggMotorBlowersRearButton.Clear();
	ggMotorBlowersAllOffButton.Clear();

	btLampkaPrzetw.Clear();
	btLampkaPrzetwB.Clear();
	btLampkaPrzetwBOff.Clear();
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
		ggMainButton.PutValue(ggMainButton.type() == TGaugeType::push ? 0.5f : m_linebreakerstate > 0 ? 1.f : 0.f);
	}

	if (ggModernLightDimSw.SubModel != nullptr)
	{
		mvOccupied->modernDimmerPosition = mvOccupied->modernDimmerDefaultPosition;
		ggModernLightDimSw.PutValue(mvOccupied->modernDimmerDefaultPosition);
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
	ggRadioChannelSelector.PutValue((Dynamic()->Mechanik ? Dynamic()->Mechanik->iRadioChannel : 1) - 1);
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
		ggConverterButton.PutValue(mvControlled->ConverterAllow ? 1.f : 0.f);
	}
	ggConverterLocalButton.PutValue(mvControlled->ConverterAllowLocal ? 1.f : 0.f);
	// compressor
	ggCompressorButton.PutValue(mvControlled->CompressorAllow ? 1.f : 0.f);
	ggCompressorLocalButton.PutValue(mvControlled->CompressorAllowLocal ? 1.f : 0.f);
	ggCompressorListButton.PutValue(mvOccupied->CompressorListPos - 1);
	// motor overload relay threshold / shunt mode
	ggMaxCurrentCtrl.PutValue(true == mvControlled->ShuntModeAllow ? (true == mvControlled->ShuntMode ? 1.f : 0.f) : mvControlled->MotorOverloadRelayHighThreshold ? 1.f : 0.f);
	// lights
	ggLightsButton.PutValue(mvOccupied->LightsPos - 1);

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
		ggDimHeadlightsButton.PutValue(DynamicObject->MoverParameters->modernDimmerPosition);
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
		ggDoorLeftPermitButton.PutValue(mvOccupied->Doors.instances[(cab_to_end() == end::front ? side::left : side::right)].open_permit ? 1.f : 0.f);
	}
	if (false == ggDoorRightPermitButton.is_push())
	{
		ggDoorRightPermitButton.PutValue(mvOccupied->Doors.instances[(cab_to_end() == end::front ? side::right : side::left)].open_permit ? 1.f : 0.f);
	}
	ggDoorPermitPresetButton.PutValue(mvOccupied->Doors.permit_preset);
	// door controls
	ggDoorLeftButton.PutValue(mvOccupied->Doors.instances[(cab_to_end() == end::front ? side::left : side::right)].is_closed ? 0.f : 1.f);
	ggDoorRightButton.PutValue(mvOccupied->Doors.instances[(cab_to_end() == end::front ? side::right : side::left)].is_closed ? 0.f : 1.f);
	// door lock
	ggDoorSignallingButton.PutValue(mvOccupied->Doors.lock_enabled ? 1.f : 0.f);
	// door step
	if (false == ggDoorStepButton.is_push())
	{
		ggDoorStepButton.PutValue(mvOccupied->Doors.step_enabled ? 1.f : 0.f);
	}
	// heating
	if (false == ggTrainHeatingButton.is_push())
	{
		ggTrainHeatingButton.PutValue(mvControlled->Heating ? 1.f : 0.f);
	}
	// brake acting time
	if (ggBrakeProfileCtrl.SubModel != nullptr)
	{
		ggBrakeProfileCtrl.PutValue((mvOccupied->BrakeDelayFlag & bdelay_R) != 0 ? 2.f : mvOccupied->BrakeDelayFlag - 1);
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
		ggWiperSw.PutValue(mvOccupied->wiperSwitchPos);
	}

	if (ggBrakeOperationModeCtrl.SubModel != nullptr)
	{
		ggBrakeOperationModeCtrl.PutValue(mvOccupied->BrakeOpModeFlag > 0 ? std::log2(mvOccupied->BrakeOpModeFlag) : 0);
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
		ggScndCtrlButton.PutValue(mvControlled->ScndCtrlPos > 0 ? 1.f : 0.f);
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
	int flag = DynamicObject->MoverParameters->InverterControlCouplerFlag;
	int itemstart = 0;
	for (auto &item : ggInverterToggleButtons) // for each button
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
	                                                          {"main_off_bt:", ggMainOffButton},
	                                                          {"main_on_bt:", ggMainOnButton},
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
		int i, j;
		Parser.getTokens(2, false);
		Parser >> i >> j;
		auto &gauge = Cabine[Cabindex].Gauge(-1); // pierwsza wolna gałka
		gauge.Load(Parser, DynamicObject);
		gauge.AssignFloat(&fEIMParams[i][j]);
	}
	else if (Label == "brakes:")
	{
		// specified pipe pressure of specified consist vehicle
		int i, j;
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
		if (Parser.getToken<std::string>() == "analog")
		{
			if (DynamicObject->mdKabina)
			{
				// McZapkie-300302: zegarek
				ggClockSInd.Init(DynamicObject->mdKabina->GetFromName("ClockShand"), nullptr, TGaugeAnimation::gt_Rotate, 1.0 / 60.0);
				ggClockMInd.Init(DynamicObject->mdKabina->GetFromName("ClockMhand"), nullptr, TGaugeAnimation::gt_Rotate, 1.0 / 60.0);
				ggClockHInd.Init(DynamicObject->mdKabina->GetFromName("ClockHhand"), nullptr, TGaugeAnimation::gt_Rotate, 1.0 / 12.0);
			}
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
		gauge.AssignDouble(&mvControlled->DistCounter);
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
