/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// reading the vehicle definition (.fiz): the parser and its tables

#include "stdafx.h"
#include "physics/mover.h"
#include "vehicle/DynObj.h"
#include "physics/oerlikonest.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/parser.h"
#include "simulation/simulation.h"
#include "vehicle/fieldshunting.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <utility>
#include <vector>
#include "physics/components/internal.h"


int ConversionError = 0;

// *************************************************************************************************
// FUNKCJE PARSERA WCZYTYWANIA PLIKU FIZYKI POJAZDU
// *************************************************************************************************
bool startBPT;
bool startMPT;
bool startMPT0;
bool startRLIST;
bool startUCLIST;
bool startDIZELMOMENTUMLIST;
bool startDIZELV2NMAXLIST;
bool startHYDROTCLIST;
bool startPMAXLIST;
bool startDLIST;
bool startFFLIST;
bool startWWLIST;
bool startWiperList;
bool startDimmerList;
bool startFFEDLIST;
bool startLIGHTSLIST;
bool startCOMPRESSORLIST;
int LISTLINE;

bool issection(std::string_view Name, std::string_view Input)
{

	return Input.compare(0, Name.size(), Name) == 0;
}

// *************************************************************************************************
// Q: 20160717
// *************************************************************************************************
// parsowanie Motor Param Table
// checks a motor parameter table position read from the .fiz file against the table size
static bool is_valid_mpt_index(int const Index)
{
	if (Index >= 0 && Index <= MotorParametersArraySize)
	{
		return true;
	}
	WriteLog("Read MPT: position " + std::to_string(Index) + " out of range 0-" + std::to_string(MotorParametersArraySize) + " in line " + std::to_string(LISTLINE));
	return false;
}

bool TMoverParameters::readMPT0(std::string const &line)
{

	// TBD, TODO: split into separate functions similar to readMPT if more varied schemes appear?
	cParser parser(line);
	if (false == parser.getTokens(7, false))
	{
		WriteLog("Read MPT0: arguments missing in line " + std::to_string(LISTLINE));
		return false;
	}
	int idx = 0; // numer pozycji
	parser >> idx;
	if (false == is_valid_mpt_index(idx))
	{
		return false;
	}
	if (EngineType == TEngineType::DieselEngine)
	{
		parser >> MotorParam[idx].mIsat >> MotorParam[idx].fi0 >> MotorParam[idx].fi >> MotorParam[idx].mfi0 >> MotorParam[idx].mfi >> MotorParam[idx].Isat;
	}
	else
	{
		parser >> MotorParam[idx].mfi >> MotorParam[idx].mIsat >> MotorParam[idx].mfi0 >> MotorParam[idx].fi >> MotorParam[idx].Isat >> MotorParam[idx].fi0;
	}
	if (true == parser.getTokens(1, false))
	{
		int autoswitch{0};
		parser >> autoswitch;
		MotorParam[idx].AutoSwitch = autoswitch == 1;
	}
	else
	{
		MotorParam[idx].AutoSwitch = false;
	}
	return true;
}

bool TMoverParameters::readMPT(std::string const &line)
{

	++LISTLINE;

	switch (EngineType)
	{

	case TEngineType::ElectricSeriesMotor:
	{
		return readMPTElectricSeries(line);
	}
	case TEngineType::DieselElectric:
	{
		return readMPTDieselElectric(line);
	}
	case TEngineType::DieselEngine:
	{
		return readMPTDieselEngine(line);
	}
	default:
	{
		return false;
	}
	}
}

bool TMoverParameters::readMPTElectricSeries(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(5, false))
	{
		WriteLog("Read MPT: arguments missing in line " + std::to_string(LISTLINE));
		return false;
	}
	int idx = 0; // numer pozycji
	parser >> idx;
	if (false == is_valid_mpt_index(idx))
	{
		return false;
	}
	parser >> MotorParam[idx].mfi >> MotorParam[idx].mIsat >> MotorParam[idx].fi >> MotorParam[idx].Isat;
	if (true == parser.getTokens(1, false))
	{
		int autoswitch{0};
		parser >> autoswitch;
		MotorParam[idx].AutoSwitch = autoswitch == 1;
	}
	else
	{
		MotorParam[idx].AutoSwitch = false;
	}
	return true;
}

bool TMoverParameters::readMPTDieselElectric(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(7, false))
	{
		WriteLog("Read MPT: arguments missing in line " + std::to_string(LISTLINE));
		return false;
	}
	int idx = 0; // numer pozycji
	parser >> idx;
	if (false == is_valid_mpt_index(idx))
	{
		return false;
	}
	parser >> MotorParam[idx].mfi >> MotorParam[idx].mIsat >> MotorParam[idx].fi >> MotorParam[idx].Isat >> MPTRelay[idx].Iup >> MPTRelay[idx].Idown;

	return true;
}

bool TMoverParameters::readMPTDieselEngine(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(4, false))
	{
		WriteLog("Read MPT: arguments missing in line " + std::to_string(LISTLINE));
		return false;
	}
	int idx = 0; // numer pozycji
	parser >> idx;
	if (false == is_valid_mpt_index(idx))
	{
		return false;
	}
	parser >> MotorParam[idx].mIsat >> MotorParam[idx].fi >> MotorParam[idx].mfi;
	if (true == parser.getTokens(1, false))
	{
		int autoswitch{0};
		parser >> autoswitch;
		MotorParam[idx].AutoSwitch = autoswitch == 1;
	}
	else
	{
		MotorParam[idx].AutoSwitch = false;
	}
	return true;
}

bool TMoverParameters::readBPT(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(5, false))
	{

		WriteLog("Read BPT: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	++LISTLINE;
	std::string braketype;
	int idx = 0;
	parser >> idx;
	parser >> BrakePressureTable[idx].PipePressureVal >> BrakePressureTable[idx].BrakePressureVal >> BrakePressureTable[idx].FlowSpeedVal >> braketype;
	if (braketype == "Pneumatic")
	{
		BrakePressureTable[idx].BrakeType = TBrakeSystem::Pneumatic;
	}
	else if (braketype == "ElectroPneumatic")
	{
		BrakePressureTable[idx].BrakeType = TBrakeSystem::ElectroPneumatic;
	}
	else
	{
		BrakePressureTable[idx].BrakeType = TBrakeSystem::Individual;
	}

	return true;
}

bool TMoverParameters::readRList(std::string const &Input)
{

	cParser parser(Input);
	if (false == parser.getTokens(5, false))
	{

		WriteLog("Read RList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	auto idx = LISTLINE++;
	if (idx >= std::size(RList))
	{
		WriteLog("Read RList: number of entries exceeded capacity of the data table");
		return false;
	}
	parser >> RList[idx].Relay >> RList[idx].R >> RList[idx].Bn >> RList[idx].Mn >> RList[idx].AutoSwitch;

	if (true == parser.getTokens(1, false))
	{
		parser >> RList[idx].ScndAct;
	}
	else
	{
		RList[idx].ScndAct = 0;
	}

	return true;
}

bool TMoverParameters::readUCList(std::string const &line)
{

	cParser parser(line);
	parser.getTokens(10, false);
	auto idx = LISTLINE++;
	if (idx >= std::size(UniCtrlList))
	{
		WriteLog("Read UCList: number of entries exceeded capacity of the data table");
		return false;
	}
	int i = 0;
	parser >> i >> UniCtrlList[idx].mode >> UniCtrlList[idx].MinCtrlVal >> UniCtrlList[idx].MaxCtrlVal >> UniCtrlList[idx].SetCtrlVal >> UniCtrlList[idx].SpeedUp >> UniCtrlList[idx].SpeedDown >>
	    UniCtrlList[idx].ReturnPosition >> UniCtrlList[idx].NextPosFastInc >> UniCtrlList[idx].PrevPosFastDec;

	return true;
}

bool TMoverParameters::readDList(std::string const &line)
{

	cParser parser(line);
	parser.getTokens(3, false);
	auto idx = LISTLINE++;
	if (idx >= std::size(RList))
	{
		WriteLog("Read DList: number of entries exceeded capacity of the data table");
		return false;
	}
	parser >> RList[idx].Relay >> RList[idx].R >> RList[idx].Mn;

	return true;
}

bool TMoverParameters::readDMList(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(2, false))
	{

		WriteLog("Read DMList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	++LISTLINE;
	double x = 0.0;
	double y = 0.0;
	parser >> x >> y;

	diesel.momentTable.emplace(x / 60.0, y);

	return true;
}

bool TMoverParameters::readV2NMAXList(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(2, false))
	{

		WriteLog("Read V2nmaxList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	++LISTLINE;
	double x = 0.0;
	double y = 0.0;
	parser >> x >> y;

	diesel.speedMaxRevolutionsTable.emplace(x, y / 60.0);

	return true;
}

bool TMoverParameters::readHTCList(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(2, false))
	{

		WriteLog("Read HTCList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	++LISTLINE;
	double x = 0.0;
	double y = 0.0;
	parser >> x >> y;

	torqueConverter.table.emplace(x, y);

	return true;
}

bool TMoverParameters::readPmaxList(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(2, false))
	{

		WriteLog("Read PmaxList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	++LISTLINE;
	double x = 0.0;
	double y = 0.0;
	parser >> x >> y;

	inductionMotor.maxPowerTable.emplace(x, y);

	return true;
}

bool TMoverParameters::readFFList(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(2, false))
	{
		WriteLog("Read FList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	int idx = LISTLINE++;
	if (idx >= std::size(FFlist))
	{
		WriteLog("Read FList: number of entries exceeded capacity of the data table");
		return false;
	}
	parser >> FFlist[idx].v >> FFlist[idx].freq;

	return true;
}

bool TMoverParameters::readFFEDList(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(2, false))
	{
		WriteLog("Read FList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	int idx = LISTLINE++;
	if (idx >= std::size(FFEDlist))
	{
		WriteLog("Read FList: number of entries exceeded capacity of the data table");
		return false;
	}
	parser >> FFEDlist[idx].v >> FFEDlist[idx].freq;

	return true;
}

// parsowanie wiperList
bool TMoverParameters::readWiperList(std::string const &line)
{
	cParser parser(line);
	if (false == parser.getTokens(4, false))
	{
		WriteLog("Read WiperList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	int idx = LISTLINE++;
	if (idx >= std::size(WiperList))
	{
		WriteLog("Read WiperList: number of entries exceeded capacity of the data table");
		return false;
	}
	parser >> WiperList[idx].byteSum >> WiperList[idx].WiperSpeed >> WiperList[idx].interval >> WiperList[idx].outBackDelay;
	return true;
}

bool TMoverParameters::readDimmerList(std::string const &line)
{
	cParser parser(line);
	if (false == parser.getTokens(3, false))
	{
		WriteLog("Read DimmerList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	++LISTLINE;

	dimPosition dps;
	parser >> dps.isHighBeam >> dps.isDimmed >> dps.isOff;
	dimPositions.push_back(dps);
	return true;
}

// parsowanie WWList
bool TMoverParameters::readWWList(std::string const &line)
{

	cParser parser(line);
	if (false == parser.getTokens(4, false))
	{
		WriteLog("Read WWList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	int idx = LISTLINE++;
	if (idx >= std::size(DElist))
	{
		WriteLog("Read WWList: number of entries exceeded capacity of the data table");
		return false;
	}
	parser >> DElist[idx].RPM >> DElist[idx].GenPower >> DElist[idx].Umax >> DElist[idx].Imax;

	if (true == parser.getTokens(3, false))
	{
		// optional parameters for shunt mode
		parser >> SST[idx].Umin >> SST[idx].Umax >> SST[idx].Pmax;

		SST[idx].Pmin = std::sqrt(std::pow(SST[idx].Umin, 2) / 47.6);
		SST[idx].Pmax = std::min(SST[idx].Pmax, std::pow(SST[idx].Umax, 2) / 47.6);
	}
	return true;
}

bool TMoverParameters::readLightsList(std::string const &Input)
{

	cParser parser(Input);
	if (false == parser.getTokens(2, false))
	{
		WriteLog("Read LightsList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	int idx = LISTLINE++;
	if (idx > 16)
	{
		WriteLog("Read LightsList: number of entries exceeded capacity of the data table");
		return false;
	}
	parser >> Lights[0][idx] >> Lights[1][idx];
	// optional third column: the cab interior light the preset sets (eu07/maszyna#529). anything but a whole number
	// (some rows carry stray notes after the two columns) leaves the cab light to its own switch
	LightsCabLight[idx] = -1;
	if (true == parser.getTokens(1, false))
	{
		std::string cablight;
		parser >> cablight;
		int flags{-1};
		auto const [end, error]{std::from_chars(cablight.data(), cablight.data() + cablight.size(), flags)};
		if (error == std::errc{} && end == cablight.data() + cablight.size() && flags >= 0)
		{
			LightsCabLight[idx] = flags;
		}
	}

	return true;
}

bool TMoverParameters::readCompressorList(std::string const &Input)
{

	cParser parser(Input);
	if (false == parser.getTokens(4, false))
	{
		WriteLog("Read CompressorList: arguments missing in line " + std::to_string(LISTLINE + 1));
		return false;
	}
	int idx = LISTLINE++;
	if (idx > 8 - 1)
	{
		WriteLog("Read CompressorList: number of entries exceeded capacity of the data table");
		return false;
	}
	// NOTE: content of slot [x][0] is hardcoded elsewhere
	parser >> CompressorList[0][idx + 1] >> CompressorList[1][idx + 1] >> CompressorList[2][idx + 1] >> CompressorList[3][idx + 1];

	return true;
}

// *************************************************************************************************
// Q: 20160719
// *************************************************************************************************
void TMoverParameters::BrakeValveDecode(std::string const &Valve)
{
	using enum TBrakeValve;

	std::map<std::string, TBrakeValve, std::less<>> valvetypes{{"W", W},
	                                              {"W_Lu_L", W_Lu_L},
	                                              {"W_Lu_XR", W_Lu_XR},
	                                              {"W_Lu_VI", W_Lu_VI},
	                                              {"K", K},
	                                              {"Kg", Kg},
	                                              {"Kp", Kp},
	                                              {"Kss", Kss},
	                                              {"Kkg", Kkg},
	                                              {"Kkp", Kkp},
	                                              {"Kks", Kks},
	                                              {"Hikp1", Hikp1},
	                                              {"Hikss", Hikss},
	                                              {"Hikg1", Hikg1},
	                                              {"KE", KE},
	                                              {"SW", SW},
	                                              {"EStED", EStED},
	                                              {"NESt3", NESt3},
	                                              {"ESt3", ESt3},
	                                              {"LSt", LSt},
	                                              {"ESt4", ESt4},
	                                              {"ESt3AL2", ESt3AL2},
	                                              {"EP1", EP1},
	                                              {"EP2", EP2},
	                                              {"M483", M483},
	                                              {"CV1_L_TR", CV1_L_TR},
	                                              {"CV1", CV1},
	                                              {"CV1_R", CV1_R}};
	auto lookup = valvetypes.find(Valve);
	BrakeValve = lookup != valvetypes.end() ? lookup->second : Other;

	if (BrakeValve == Other && contains(Valve, "ESt"))
	{

		BrakeValve = ESt3;
	}
}

// *************************************************************************************************
// Q: 20160719
// *************************************************************************************************
void TMoverParameters::BrakeSubsystemDecode()
{
	using enum TBrakeSubSystem;
	BrakeSubsystem = ss_None;
	switch (BrakeValve)
	{
	using enum TBrakeValve;
	case W:
	case W_Lu_L:
	case W_Lu_VI:
	case W_Lu_XR:
		BrakeSubsystem = ss_W;
		break;
	case ESt3:
	case ESt3AL2:
	case ESt4:
	case EP2:
	case EP1:
		BrakeSubsystem = ss_ESt;
		break;
	case KE:
		BrakeSubsystem = ss_KE;
		break;
	case CV1:
	case CV1_L_TR:
		BrakeSubsystem = ss_Dako;
		break;
	case LSt:
	case EStED:
		BrakeSubsystem = ss_LSt;
		break;
	default:
		break;
	}
}

// *************************************************************************************************
// Q: 20160717
// Funkcja pelniaca role pierwotnej LoadChkFile wywolywana w dynobj.cpp w double
// TDynamicObject::Init()
// Po niej wykonywana jest CreateBrakeSys(), ktora jest odpowiednikiem CheckLocomotiveParameters()
// *************************************************************************************************
bool TMoverParameters::LoadFIZ(std::string const &chkpath)
{
	chkPath = chkpath; // assign class path for reloading

	ConversionError = 666;
	std::string file = TypeName + ".fiz";

	WriteLog("LOAD FIZ FROM " + file);
	/*
	    std::ifstream in(file);
	    if (!in.is_open())
	    {
	        WriteLog("E8 - FIZ FILE NOT EXIST.");
	        return false;
	    }
	*/
	cParser fizparser(file, cParser::buffer_FILE, chkpath);
	if (false == fizparser.ok())
	{
		WriteLog("E8 - FIZ FILE NOT EXIST.");
		return false;
	}

	return LoadFIZ_Parse(fizparser);
}

// reads the vehicle definition from text in the .fiz format, e.g. one made up by a unit test
bool TMoverParameters::LoadFIZ_Text(std::string const &Text)
{
	ConversionError = 666;
	cParser fizparser(Text, cParser::buffer_TEXT);
	return LoadFIZ_Parse(fizparser);
}

bool TMoverParameters::LoadFIZ_Parse(cParser &fizparser)
{
	LISTLINE = 0;
	startBPT = false;
	startMPT = false;
	startMPT0 = false;
	startRLIST = false;
	startUCLIST = false;
	startDLIST = false;
	startDIZELMOMENTUMLIST = false;
	startDIZELV2NMAXLIST = false;
	startHYDROTCLIST = false;
	startPMAXLIST = false;
	startFFLIST = false;
	startFFEDLIST = false;
	startWWLIST = false;
	startWiperList = false;
	startDimmerList = false;
	startLIGHTSLIST = false;
	startCOMPRESSORLIST = false;

	ConversionError = 0;

	// Zbieranie danych zawartych w pliku FIZ
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	std::map<std::string, std::string, std::less<>> fizlines;
	std::string inputline;
	/*
	    while (std::getline(in, inputline))
	*/
	while (fizparser.ok())
	{

		inputline = fizparser.getToken<std::string>(false, "\n\r");

		if (inputline.starts_with("//") || contains(inputline, '#'))
		{
			// skip commented lines
			continue;
		}

		if (!inputline.empty() && inputline.front() == ' ')
		{
			// guard against malformed config files with leading spaces
			inputline.erase(0, inputline.find_first_not_of(' '));
		}

		// trim CR at end (mainly for linux)
		if (!inputline.empty() && inputline.back() == '\r')
			inputline.pop_back();

		if (inputline.length() == 0)
		{
			startBPT = false;
			continue;
		}

		if (LoadFIZ_TableEnd(inputline) || LoadFIZ_Section(inputline, fizlines) || LoadFIZ_TableSection(inputline, fizlines))
		{
			continue;
		}
		LoadFIZ_TableRow(inputline);
	} // while line
	/*
	    in.close();
	*/
	// Operacje na zebranych parametrach - przypisywanie do wlasciwych zmiennych i ustawianie
	// zaleznosci

	bool result;
	if (ConversionError == 0)
		result = true;
	else
		result = false;

	// ustawiamy domyslna pozycje dimmera
	// the DefaultPos of a short or an empty DimmerList would put the dimmer outside of it
	modernDimmerDefaultPosition = clampDimmerPosition(modernDimmerDefaultPosition);
	modernDimmerPosition = clampDimmerPosition(modernDimmerPosition);
	if (!enableModernDimmer)
	{
		modernDimmerPosition = modernDimmerDefaultPosition;
	}

	WriteLog("CERROR: " + std::to_string(ConversionError) + ", SUCCES: " + std::to_string(result));
	return result;
}

// the end of the table being read; returns whether the line is one
bool TMoverParameters::LoadFIZ_TableEnd(std::string const &inputline) const
{
	// checking if table parsing should be switched off goes first...
	if (issection("END-MPT", inputline))
	{
		startBPT = false;
		startMPT = false;
		startMPT0 = false;
		return true;
	}
	if (issection("END-RL", inputline))
	{
		startBPT = false;
		startRLIST = false;
		return true;
	}
	if (issection("END-UCL", inputline))
	{
		startBPT = false;
		startUCLIST = false;
		return true;
	}
	if (issection("END-DL", inputline))
	{
		startBPT = false;
		startDLIST = false;
		return true;
	}
	if (issection("END-DML", inputline))
	{
		startBPT = false;
		startDIZELMOMENTUMLIST = false;
		return true;
	}
	if (issection("END-V2NL", inputline))
	{
		startBPT = false;
		startDIZELV2NMAXLIST = false;
		return true;
	}
	if (issection("END-HTCL", inputline))
	{
		startBPT = false;
		startHYDROTCLIST = false;
		return true;
	}
	if (issection("END-PML", inputline))
	{
		startBPT = false;
		startPMAXLIST = false;
		return true;
	}
	if (issection("endff", inputline))
	{
		startBPT = false;
		startFFLIST = false;
		startFFEDLIST = false;
		return true;
	}
	if (issection("endwl", inputline))
	{
		// skonczylismy czytac liste konfiguracji wycieraczek
		startBPT = false;
		startWiperList = false;
		return true;
	}
	if (issection("endDimmerList", inputline))
	{
		// skonczylismy czytac liste konfiguracji pstryka od przyciemnienia
		startBPT = false;
		startDimmerList = false;
		return true;
	}
	if (issection("END-WWL", inputline))
	{
		startBPT = false;
		startWWLIST = false;
		return true;
	}
	if (issection("endL", inputline))
	{
		startBPT = false;
		startLIGHTSLIST = false;
		return true;
	}
	if (issection("endCL", inputline))
	{
		startBPT = false;
		startCOMPRESSORLIST = false;
		return true;
	}
	return false;
}

// a section of the definition giving values, kept in fizlines by its name; returns whether the line is one
bool TMoverParameters::LoadFIZ_Section(std::string const &inputline, std::map<std::string, std::string, std::less<>> &fizlines)
{
	// ...then all recognized sections...
	if (issection("Param.", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Param", inputline);
		LoadFIZ_Param(inputline);
		return true;
	}

	if (issection("Load:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Load", inputline);
		LoadFIZ_Load(inputline);
		return true;
	}

	if (issection("Dimensions:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Dimensions", inputline);
		LoadFIZ_Dimensions(inputline);
		return true;
	}

	if (issection("Wheels:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Wheels", inputline);
		LoadFIZ_Wheels(inputline);
		return true;
	}

	if (issection("Brake:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Brake", inputline);
		LoadFIZ_Brake(inputline);
		return true;
	}

	if (issection("Doors:", inputline))
	{

		startBPT = false;
		fizlines.try_emplace("Doors", inputline);
		LoadFIZ_Doors(inputline);
		return true;
	}

	if (issection("BuffCoupl.", inputline))
	{

		startBPT = false;
		fizlines.try_emplace("BuffCoupl", inputline);
		LoadFIZ_BuffCoupl(inputline, 0);
		return true;
	}

	else if (issection("BuffCoupl1.", inputline))
	{

		startBPT = false;
		fizlines.try_emplace("BuffCoupl1", inputline);
		LoadFIZ_BuffCoupl(inputline, 1);
		return true;
	}

	else if (issection("BuffCoupl2.", inputline))
	{

		startBPT = false;
		fizlines.try_emplace("BuffCoupl2", inputline);
		LoadFIZ_BuffCoupl(inputline, 2);
		return true;
	}

	if (issection("TurboPos:", inputline))
	{

		startBPT = false;
		fizlines.try_emplace("TurboPos", inputline);
		LoadFIZ_TurboPos(inputline);
		return true;
	}

	if (issection("Cntrl.", inputline))
	{

		startBPT = true;
		LISTLINE = 0;
		fizlines.try_emplace("Cntrl", inputline);
		LoadFIZ_Cntrl(inputline);
		return true;
	}

	if (issection("Headlights:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Headlights", inputline);
		LoadFIZ_Headlights(inputline);
		return true;
	}

	if (issection("Blending:", inputline))
	{

		startBPT = false;
		LISTLINE = 0;
		fizlines.try_emplace("Blending", inputline);
		LoadFIZ_Blending(inputline);
		return true;
	}

	if (issection("DCEMUED:", inputline))
	{

		startBPT = false;
		LISTLINE = 0;
		fizlines.try_emplace("DCEMUED", inputline);
		LoadFIZ_DCEMUED(inputline);
		return true;
	}

	if (issection("SpringBrake:", inputline))
	{

		startBPT = false;
		LISTLINE = 0;
		fizlines.try_emplace("SpringBrake", inputline);
		LoadFIZ_SpringBrake(inputline);
		return true;
	}

	if (issection("Light:", inputline))
	{

		startBPT = false;
		fizlines.try_emplace("Light", inputline);
		LoadFIZ_Light(inputline);
		return true;
	}

	if (issection("Security:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Security", inputline);
		SecuritySystem.load(inputline, Vmax);
		extract_value(EmergencyBrakeWarningSignal, "EmergencyBrakeWarningSignal", inputline, "");
		return true;
	}

	if (issection("Clima:", inputline))
	{

		startBPT = false;
		fizlines.try_emplace("Clima", inputline);
		LoadFIZ_Clima(inputline);
		return true;
	}

	if (issection("Power:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Power", inputline);
		LoadFIZ_Power(inputline);
		return true;
	}

	if (issection("SpeedControl:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("SpeedControl", inputline);
		LoadFIZ_SpeedControl(inputline);
		return true;
	}

	if (issection("Engine:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Engine", inputline);
		LoadFIZ_Engine(inputline);
		return true;
	}

	if (issection("Switches:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Switches", inputline);
		LoadFIZ_Switches(inputline);
		return true;
	}

	if (issection("Circuit:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("Circuit", inputline);
		LoadFIZ_Circuit(inputline);
		return true;
	}

	if (issection("AI:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("AI", inputline);
		LoadFIZ_AI(inputline);
		return true;
	}

	return false;
}

// a section of the definition opening a table, kept in fizlines by its name; returns whether the line is one
bool TMoverParameters::LoadFIZ_TableSection(std::string const &inputline, std::map<std::string, std::string, std::less<>> &fizlines)
{
	if (issection("MotorParamTable:", inputline))
	{
		startBPT = false;
		startMPT = true;
		LISTLINE = 0;
		fizlines.try_emplace("MotorParamTable", inputline);
		LoadFIZ_MotorParamTable(inputline);
		return true;
	}

	if (issection("MotorParamTable0:", inputline))
	{
		startBPT = false;
		startMPT0 = true;
		LISTLINE = 0;
		return true;
	}

	if (issection("RList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("RList", inputline);
		startRLIST = true;
		LISTLINE = 0;
		LoadFIZ_RList(inputline);
		return true;
	}

	if (issection("UCList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("UCList", inputline);
		startUCLIST = true;
		LISTLINE = 0;
		LoadFIZ_UCList(inputline);
		return true;
	}

	if (issection("DList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("DList", inputline);
		startDLIST = true;
		LISTLINE = 0;
		LoadFIZ_DList(inputline);
		return true;
	}

	if (issection("DMList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("DMList", inputline);
		startDIZELMOMENTUMLIST = true;
		LISTLINE = 0;
		return true;
	}

	if (issection("HTCList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("HTCList", inputline);
		startHYDROTCLIST = true;
		LISTLINE = 0;
		return true;
	}

	if (issection("PmaxList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("PmaxList", inputline);
		startPMAXLIST = true;
		LISTLINE = 0;
		return true;
	}

	if (issection("V2NList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("V2NList", inputline);
		startDIZELV2NMAXLIST = true;
		LISTLINE = 0;
		return true;
	}

	if (issection("ffList:", inputline))
	{
		startBPT = false;
		startFFLIST = true;
		LISTLINE = 0;
		LoadFIZ_FFList(inputline);
		return true;
	}

	if (issection("ffBrakeList:", inputline))
	{
		startBPT = false;
		startFFEDLIST = true;
		LISTLINE = 0;
		LoadFIZ_FFEDList(inputline);
		return true;
	}

	if (issection("WWList:", inputline))
	{
		startBPT = false;
		startWWLIST = true;
		LISTLINE = 0;

		return true;
	}

	if (issection("WiperList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("WiperList", inputline);
		startWiperList = true;
		LISTLINE = 0;
		LoadFIZ_WiperList(inputline);

		return true;
	}

	if (issection("DimmerList:", inputline))
	{
		dimPositions.clear(); // uzywamy customowej listy
		startBPT = false;
		startDimmerList = true;
		fizlines.try_emplace("DimmerList", inputline);
		LoadFIZ_DimmerList(inputline);
		return true;
	}

	if (issection("LightsList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("LightsList", inputline);
		startLIGHTSLIST = true;
		LISTLINE = 0;
		LoadFIZ_LightsList(inputline);
		return true;
	}

	if (issection("CompressorList:", inputline))
	{
		startBPT = false;
		fizlines.try_emplace("CompressorList", inputline);
		startCOMPRESSORLIST = true;
		LISTLINE = 0;
		LoadFIZ_CompressorList(inputline);
		return true;
	}
	return false;
}

// a row of the table being read
void TMoverParameters::LoadFIZ_TableRow(std::string const &inputline)
{
	// ...and finally, table parsers.
	// NOTE: once table parsing is enabled it lasts until switched off, when another section is recognized
	if (true == startBPT)
	{
		readBPT(inputline);
		return;
	}
	if (true == startMPT)
	{
		readMPT(inputline);
		return;
	}
	if (true == startMPT0)
	{
		readMPT0(inputline);
		return;
	}
	if (true == startRLIST)
	{
		readRList(inputline);
		return;
	}
	if (true == startUCLIST)
	{
		readUCList(inputline);
		return;
	}
	if (true == startDLIST)
	{
		readDList(inputline);
		return;
	}
	if (true == startDIZELMOMENTUMLIST)
	{
		readDMList(inputline);
		return;
	}
	if (true == startDIZELV2NMAXLIST)
	{
		readV2NMAXList(inputline);
		return;
	}
	if (true == startHYDROTCLIST)
	{
		readHTCList(inputline);
		return;
	}
	if (true == startPMAXLIST)
	{
		readPmaxList(inputline);
		return;
	}
	if (true == startFFLIST)
	{
		readFFList(inputline);
		return;
	}
	if (true == startFFEDLIST)
	{
		readFFEDList(inputline);
		return;
	}
	if (true == startWWLIST)
	{
		readWWList(inputline);
		return;
	}
	if (true == startWiperList)
	{
		readWiperList(inputline);
		return;
	}
	if (true == startDimmerList)
	{
		readDimmerList(inputline);
	}
	if (true == startLIGHTSLIST)
	{
		readLightsList(inputline);
		return;
	}
	if (true == startCOMPRESSORLIST)
	{
		readCompressorList(inputline);
		return;
	}
}

bool TMoverParameters::reload_FIZ()
{
	WriteLog("[DEV] Reloading FIZ for " + Name);
	// pause simulation
	Global.iPause |= 0b1000;
	if (bool result = LoadFIZ(chkPath); result == true)
	{
		// jesli sie udalo przeladowac FIZ
		Global.iPause &= 0b0111;
		WriteLog("[DEV] FIZ reloaded for " + Name);
	}
	else
	{
		// failed to reload - exit simulator
		ErrorLog("[DEV] Failed to reload fiz for vehicle " + Name);
	}
	return true;
}
