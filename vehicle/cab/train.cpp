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

// the train the user drives: construction, its state, taking a vehicle, the trains of the scenery

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


std::vector<std::string> const TTrain::fPress_labels = {

    "ch1:  ", "ch2:  ", "ch3:  ", "ch4:  ", "ch5:  ", "ch6:  ", "ch7:  ", "ch8:  ", "ch9:  ", "ch0:  "};

TTrain::TTrain()
{

	// McZapkie-240302 - przyda sie do tachometru

	// asMessage="";
	DynamicSet(nullptr); // ustawia wszystkie mv*
	//-----

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
		for (auto &press : fPress[i])
			press = 0.0;
		bBrakes[i][0] = false;
		bBrakes[i][1] = false;
	}
}

TTrain::~TTrain() = default;

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
	if (!e3d && DynamicObject->Mechanik == nullptr)
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
	dict->insert("off_from_dimmer", mvOccupied->currentDimPosition().isOff);
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
	auto const &brakevalve{*mvOccupied->Hamulec};
	if (auto const &brakevalvetype{typeid(brakevalve)}; brakevalvetype == typeid(TLSt) || brakevalvetype == typeid(TEStED))
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
	dict->insert("radio_fuse", mvOccupied->RadioFuse);
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
	dict->insert("odometer", mvOccupied->DistCounter); // distance run in the session, km
	dict->insert("odometer_total", mvOccupied->Odometer.km); // the vehicle's odometer, kept between the sessions
	// electric current data
	dict->insert("traction_voltage", std::abs(mvPantographUnit->PantographVoltage));
	dict->insert("voltage", std::abs(mvControlled->EngineVoltage));
	dict->insert("im", std::abs(mvControlled->Im));
	dict->insert("fuse", mvControlled->FuseFlag);
	dict->insert("epfuse", mvOccupied->EpFuse);
	dict->insert("power_drawn", mvOccupied->EnergyMeter.first);
	dict->insert("power_returned", mvOccupied->EnergyMeter.second);

	// induction motor state data
	std::array<char const *, 10> const TXTT{"fd", "fdt", "fdb", "pd", "pdt", "pdb", "itothv", "1", "2", "3"};
	std::array<char const *, 10> const TXTC{"fr", "frt", "frb", "pr", "prt", "prb", "im", "vm", "ihv", "uhv"};
	std::array<char const *, 10> const TXTD{"enrot", "nrot", "fill_des", "fill_real", "clutch_des", "clutch_real", "water_temp", "oil_press", "engine_temp", "retarder_fill"};
	std::array<char const *, 7> const TXTP{"bc", "bp", "sp", "cp", "rp", "mass", "spring"};
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
	TDynamicObject *p = DynamicObject->GetFirstDynamic(mvOccupied->CabOccupied < 0 ? end::rear : end::front, Coupling::control);
	int in = 0;
	while (p && in < 8)
	{
		if (p->MoverParameters->inductionMotor.config[eimc_p_Pmax] > 1)
		{
			in++;
			dict->insert("eimp_c" + std::to_string(in) + "_invno", p->MoverParameters->inductionMotor.inverterCount);
			for (int j = 0; j < p->MoverParameters->inductionMotor.inverterCount; j++)
			{
				dict->insert("eimp_c" + std::to_string(in) + "_inv" + std::to_string(j + 1) + "_act", p->MoverParameters->inductionMotor.inverters[j].IsActive);
				dict->insert("eimp_c" + std::to_string(in) + "_inv" + std::to_string(j + 1) + "_error", p->MoverParameters->inductionMotor.inverters[j].Error);
				dict->insert("eimp_c" + std::to_string(in) + "_inv" + std::to_string(j + 1) + "_allow", p->MoverParameters->inductionMotor.inverters[j].Activate);
			}
		}
		p = kier ? p->Next(Coupling::control) : p->Prev(Coupling::control);
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
	auto const *vehicledriver{(DynamicObject->ctOwner != nullptr ? DynamicObject->ctOwner : DynamicObject->Mechanik)};

	dict->insert("velocity_desired", vehicledriver->VelDesired);
	dict->insert("velroad", vehicledriver->VelRoad);
	dict->insert("vellimitlast", vehicledriver->VelLimitLast);
	dict->insert("velsignallast", vehicledriver->VelSignalLast);
	dict->insert("velsignalnext", vehicledriver->VelSignalNext);
	dict->insert("velnext", vehicledriver->VelNext);
	dict->insert("actualproximitydist", vehicledriver->ActualProximityDist);
	// train data
	vehicledriver->TrainTimetable().serialize(dict.get());
	dict->insert("train_atpassengerstop", vehicledriver->IsAtPassengerStop);
	dict->insert("train_length", vehicledriver->fLength);
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
	    {fHCurrent[mvControlled->TrainType == train_type::emu ? 0 : 1], fHCurrent[2], fHCurrent[3]},
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

bool TTrain::is_eztoer() const
{

	return mvControlled->TrainType == train_type::emu && mvOccupied->BrakeSubsystem == TBrakeSubSystem::ss_ESt && mvControlled->Power24vIsAvailable == true && mvControlled->EpFuse == true &&
	       mvControlled->DirActive != 0; // od yB
}

// mover master controller to specified position
void TTrain::set_master_controller(double const Position)
{

	auto positionchange{std::min<int>(static_cast<int>(Position), mvControlled->CoupledCtrl ? mvControlled->MainCtrlPosNo + mvControlled->ScndCtrlPosNo : mvControlled->MainCtrlPosNo) -
	                    (mvControlled->CoupledCtrl ? mvControlled->MainCtrlPos + mvControlled->ScndCtrlPos : mvControlled->MainCtrlPos)};
	while (positionchange < 0 && true == mvControlled->DecMainCtrl(1))
	{
		++positionchange;
	}
	while (positionchange > 0 && true == mvControlled->IncMainCtrl(1))
	{
		--positionchange;
	}
}

// moves train brake lever to specified position, potentially emits switch sound if conditions are met
void TTrain::set_train_brake(double const Position)
{

	auto const originalbrakeposition{static_cast<int>(100.0 * mvOccupied->fBrakeCtrlPos)};
	mvOccupied->BrakeLevelSet(Position);

	if (static_cast<int>(100.0 * mvOccupied->fBrakeCtrlPos) == originalbrakeposition)
	{
		return;
	}

	// sound feedback if the lever movement activates one of the switches
	if (true == is_eztoer() && false == ((originalbrakeposition / 100 == 0 || originalbrakeposition / 100 >= 5) && (mvOccupied->BrakeCtrlPos == 0 || mvOccupied->BrakeCtrlPos >= 5)) && dsbPneumaticSwitch)
	{
		dsbPneumaticSwitch->play();
	}
}

void TTrain::zero_charging_train_brake()
{
	using enum TBrakeHandle;

	if (mvOccupied->BrakeCtrlPos == -1 && DynamicObject->Controller != AIdriver && Global.iFeedbackMode < 3 &&
	    (mvOccupied->BrakeHandle == FVel6 || mvOccupied->BrakeHandle == MHZ_EN57 || mvOccupied->BrakeHandle == MHZ_K8P))
	{
		// Odskakiwanie hamulce EP
		set_train_brake(0);
	}
}

void TTrain::set_train_brake_speed(TDynamicObject *Vehicle, int const Speed)
{

	// visual feedback
	// TODO: add setting indicator to vehicle class, for external lever/indicator
	if (true == Vehicle->MoverParameters->BrakeDelaySwitch(Speed) && Vehicle == DynamicObject)
	{
		if (ggBrakeProfileCtrl.SubModel != nullptr)
		{
			ggBrakeProfileCtrl.UpdateValue((mvOccupied->BrakeDelayFlag & bdelay_R) != 0 ? 2.0 : static_cast<float>(mvOccupied->BrakeDelayFlag - 1), dsbSwitch);
		}
		if (ggBrakeProfileG.SubModel != nullptr)
		{
			ggBrakeProfileG.UpdateValue(mvOccupied->BrakeDelayFlag == bdelay_G ? 1.0 : 0.0, dsbSwitch);
		}
		if (ggBrakeProfileR.SubModel != nullptr)
		{
			ggBrakeProfileR.UpdateValue((mvOccupied->BrakeDelayFlag & bdelay_R) != 0 ? 1.0 : 0.0, dsbSwitch);
		}
	}
}

void TTrain::set_paired_open_motor_connectors_button(bool const State)
{

	if (mvControlled->TrainType == train_type::et41 || mvControlled->TrainType == train_type::et42)
	{
		// crude implementation of the button affecting entire unit for multi-unit engines
		// TODO: rework it into part of standard command propagation system
		if (mvControlled->Couplers[end::front].Connected != nullptr && true == mvControlled->Couplers[end::front].CouplingFlag.HasAll(Coupling::permanent))
		{
			mvControlled->Couplers[end::front].Connected->StLinSwitchOff = State;
		}
		if (mvControlled->Couplers[end::rear].Connected != nullptr && true == mvControlled->Couplers[end::rear].CouplingFlag.HasAll(Coupling::permanent))
		{
			mvControlled->Couplers[end::rear].Connected->StLinSwitchOff = State;
		}
	}
}

// locates nearest vehicle belonging to the consist
TDynamicObject *TTrain::find_nearest_consist_vehicle(bool freefly, glm::vec3 pos) const
{
	if (!freefly)
		return DynamicObject;

	auto couplerindex{-2}; // scan for vehicle, not any specific coupler
	auto *vehicle{DynamicObject->ABuScanNearestObject(pos, DynamicObject->GetTrack(), 1, 1500, couplerindex)};
	if (vehicle == nullptr)
		vehicle = DynamicObject->ABuScanNearestObject(pos, DynamicObject->GetTrack(), -1, 1500, couplerindex);
	// TBD, TODO: perform owner test for the located vehicle
	return vehicle;
}

// cab movement update, fixed step part
void TTrain::UpdateCab()
{

	// Ra: przesiadka, jeśli AI zmieniło kabinę (a człon?)...
	if (DynamicObject->Mechanik // może nie być?
	    && DynamicObject->Mechanik->AIControllFlag && iCabn != ( // numer kabiny (-1: kabina B)
		                 mvOccupied->CabOccupied == -1 ? 2 : mvOccupied->CabOccupied))
	{

		InitializeCab(mvOccupied->CabOccupied, mvOccupied->TypeName + ".mmd");
	}
	iCabn = mvOccupied->CabOccupied == -1 ? 2 : mvOccupied->CabOccupied;
}

void TTrain::add_distance(double const Distance)
{

	auto const meterenabled{m_distancecounter >= 0 && (mvOccupied->Power24vIsAvailable || mvOccupied->Power110vIsAvailable)};

	if (true == meterenabled)
	{
		m_distancecounter += static_cast<float>(Distance * Occupied()->CabOccupied);
	}
	else
	{
		m_distancecounter = -1.f;
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
		if (mvOccupied->ChangeCab(iDirection) && InitializeCab(mvOccupied->CabOccupied, mvOccupied->TypeName + ".mmd"))
		{
			// zmiana kabiny w ramach tego samego pojazdu
			mvOccupied->CabActivisationAuto(); // załączenie rozrządu (wirtualne kabiny)
			DynamicObject->Mechanik->DirectionChange();
			return true; // udało się zmienić kabinę
		}
		// aktywizacja poprzedniej, bo jeszcze nie wiadomo, czy jakiś pojazd jest
		mvOccupied->CabActivisationAuto();
	}
	return false; // ewentualna zmiana pojazdu
}

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
	mvControlled = d ? DynamicObject->MoverParameters : nullptr; // albo silnikowy w EZT
	mvOccupied = mvControlled;

	if (DynamicObject == nullptr)
	{
		return;
	}

	mvControlled = DynamicObject->FindPowered()->MoverParameters;
	mvSecond = nullptr; // gdyby się nic nie znalazło
	if (mvOccupied->Power > 1.0) // dwuczłonowe lub ukrotnienia, żeby nie szukać każdorazowo
	{
		if (mvOccupied->Couplers[1].Connected != nullptr && mvOccupied->Couplers[1].AllowedFlag.HasAny(Coupling::control))
		{ // gdy jest człon od sprzęgu 1, a sprzęg łączony
			// warsztatowo (powiedzmy)
			if (mvOccupied->Couplers[1].Connected->Power > 1.0) // ten drugi ma moc
				mvSecond = mvOccupied->Couplers[1].Connected; // wskaźnik na drugiego
		}
		else if (mvOccupied->Couplers[0].Connected != nullptr && mvOccupied->Couplers[0].AllowedFlag.HasAny(Coupling::control))
		{ // gdy jest człon od sprzęgu 0, a sprzęg łączony
			// warsztatowo (powiedzmy)
			if (mvOccupied->Couplers[0].Connected->Power > 1.0) // ale ten drugi ma moc
				mvSecond = mvOccupied->Couplers[0].Connected; // wskaźnik na drugiego
		}
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

	// the bounds come from the .mmd as written, and some cabs have them reversed (the B cars of SA134 and 21xM,
	// ET40, 754...); std::clamp is undefined for those and on MSVC makes the camera jump between the two bounds
	// every frame, as its result is fed back here. safe_clamp keeps the camera on the upper bound, as 26.05 did
	return {safe_clamp(Point.x, (double)Cabine[iCabn].CabPos1.x, (double)Cabine[iCabn].CabPos2.x), safe_clamp(Point.y, (double)Cabine[iCabn].CabPos1.y + 0.5, (double)Cabine[iCabn].CabPos2.y + 1.8),
	        safe_clamp(Point.z, (double)Cabine[iCabn].CabPos1.z, (double)Cabine[iCabn].CabPos2.z)};
}

void TTrain::radio_message(sound_source *Message, int const Channel)
{

	if (auto const soundrange{Message->range()}; soundrange > 0 && glm::length2(Message->location() - glm::dvec3{DynamicObject->GetPosition()}) > sq(soundrange))
	{
		// skip message playback if the receiver is outside of the emitter's range
		return;
	}
	// NOTE: we initiate playback of all sounds in range, in case the user switches on the radio or tunes to the right channel mid-play
	m_radiomessages.emplace_back(Channel, std::make_shared<sound_source>(m_radiosound));
	// assign sound to the template and play it
	auto &message = *m_radiomessages.back().second.get();
	auto const radioenabled{mvOccupied->RadioPowered()};
	auto const volume{true == radioenabled && Dynamic()->Mechanik != nullptr && Channel == RadioChannel() ? 1.0 : 0.0};
	message.copy_sounds(*Message).gain(static_cast<float>(volume)).play();
}

uint16_t TTrain::id()
{
	if (vid == 0)
	{
		vid = ++simulation::prev_train_id;
		WriteLog("net: assigning id " + std::to_string(vid) + " to vehicle " + Dynamic()->name(), logtype::net);
	}
	return vid;
}
namespace
{

// updates trains in range [Start, End)
void update_trains(std::deque<TTrain *> const &Trains, std::size_t const Start, std::size_t const End, double const Dt)
{
	for (std::size_t j = Start; j < End; ++j)
	{
		if (Trains[j])
			Trains[j]->Update(Dt);
	}
}

} // namespace

void train_table::updateAsync(double dt)
{
	const int threads = std::max(1, Global.trainThreads);
	const size_t total = m_items.size();
	const size_t chunkSize = (total + threads - 1) / threads;
	auto const instance{TSubModel::iInstance};

	std::vector<std::jthread> workers;
	std::vector<std::future<void>> results; // przekazują wyjątki z wątków do głównego wątku
	workers.reserve(threads);
	results.reserve(threads);

	for (int i = 0; i < threads; ++i)
	{
		const size_t start = i * chunkSize;
		const size_t end = std::min(start + chunkSize, total);

		if (start >= end)
			break; // brak więcej danych

		// the instance whose animations the gauges set (TSubModel::iInstance, per thread since the walks of the vulkan renderer went on
		// several threads) is the one the main thread set for the driven vehicle
		std::packaged_task<void()> task([this, start, end, dt, instance]() {
			TSubModel::iInstance = instance;
			update_trains(m_items, start, end, dt);
		});
		results.emplace_back(task.get_future());
		workers.emplace_back(std::move(task));
	}

	// Poczekaj aż wszystkie wątki skończą
	for (auto &worker : workers)
		worker.join();

	for (auto &result : results)
		result.get();

	// Teraz kasowanie (tylko w głównym wątku)
	for (TTrain *train : m_items)
	{
		if (!train)
			continue;

		if (train->pending_delete)
		{
			purge(train->Dynamic()->name());
			if (simulation::Train == train)
				simulation::Train = nullptr;
		}
		else if (simulation::Train != train && Global.network_servers.empty() && !Global.network_client)
		{
			purge(train->Dynamic()->name());
		}
	}
}

void train_table::update(double dt)
{
	for (TTrain *train : m_items)
	{
		if (!train)
			continue;

		train->Update(dt);

		if (train->pending_delete)
		{
			purge(train->Dynamic()->name());
			if (simulation::Train == train)
				simulation::Train = nullptr;
		}
		// for single-player destroy non-player trains
		else if (simulation::Train != train && Global.network_servers.empty() && !Global.network_client)
		{
			purge(train->Dynamic()->name());
		}
	}
}

TTrain *train_table::find_id(std::uint16_t const Id) const
{

	if (Id == 0)
	{
		return nullptr;
	}

	for (TTrain *train : m_items)
	{
		if (!train)
		{
			continue;
		}
		if (train->id() == Id)
		{
			return train;
		}
	}
	return nullptr;
}
