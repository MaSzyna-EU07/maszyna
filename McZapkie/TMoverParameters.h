#pragma once

#include "core.h"
#include "lights_preset_cablight.h"
#include "TLocation.h"
#include "TRotation.h"
#include "TDimension.h"
#include "TCommand.h"
#include "TTrackShape.h"
#include "TTrackParam.h"
#include "TTractionParam.h"
#include "TBrakePressure.h"
#include "TGrateType.h"
#include "TBoilerType.h"
#include "TCurrentCollector.h"
#include "engine_generator.h"
#include "TAccumulator.h"
#include "TPowerCable.h"
#include "THeater.h"
#include "TTransducer.h"
#include "TPowerParameters.h"
#include "TScheme.h"
#include "TDEScheme.h"
#include "TFFScheme.h"
#include "TWiperScheme.h"
#include "TShuntScheme.h"
#include "TMPTRelay.h"
#include "TMotorParameters.h"
#include "TUniversalCtrl.h"
#include "TSecuritySystem.h"
#include "TTransmision.h"
#include "power_coupling.h"
#include "TCoupling.h"
#include "neighbour_data.h"
#include "speed_control.h"
#include "inverter.h"
#include "basic_device.h"
#include "basic_light.h"
#include "cooling_fan.h"
#include "fuel_pump.h"
#include "oil_pump.h"
#include "water_pump.h"
#include "basic_valve.h"
#include "basic_pantograph.h"
#include "basic_door.h"
#include "door_data.h"
#include "water_heater.h"
#include "heater_config_t.h"
#include "heat_data.h"
#include "fluid_circuit_t.h"
#include "circuit_config_t.h"
#include "spring_brake.h"
#include "load_attributes.h"
#include "dimPosition.h"
#include "derailment.h"
#include "collision.h"

namespace mover
{
class TMoverParameters
{ // Ra: wrapper na kod pascalowy, przejmujący jego funkcje  Q: 20160824 - juz nie wrapper a klasa bazowa :)
  private:
	// types
	/* TODO: implement
	    // communication cable, exchanging control signals with adjacent vehicle
	    struct jumper_cable {
	    // types
	        using flag_pair = std::pair<bool, bool>;
	    // members
	        // booleans
	        // std::array<bool, 1> flags {};
	        // boolean pairs, exchanged data is swapped when connected to a matching end (front to front or back to back)
	        // TBD, TODO: convert to regular bool array for efficiency once it's working?
	        std::array<flag_pair, 1> flag_pairs {};
	        // integers
	        // std::array<int, 1> values {};
	    };
	*/
	// basic approximation of a generic device
	// TBD: inheritance or composition?
	

	

	

	// basic approximation of a fuel pump
	

	// basic approximation of an oil pump
	

	// basic approximation of a water pump
	

	// basic approximation of a solenoid valve
	

	// basic approximation of a pantograph
	

	// basic approximation of doors
	

	

	

	

	

  public:
	std::string chkPath;
	bool reload_FIZ();
	double dMoveLen = 0.0;
	/*---opis lokomotywy, wagonu itp*/
	/*--opis serii--*/
	int CategoryFlag = 1; /*1 - pociag, 2 - samochod, 4 - statek, 8 - samolot*/
	/*--sekcja stalych typowych parametrow*/
	std::string TypeName; /*nazwa serii/typu*/
	int TrainType = 0; /*typ: EZT/elektrowoz - Winger 040304 Ra: powinno być szybciej niż string*/
	TEngineType EngineType = TEngineType::None; /*typ napedu*/
	TPowerParameters EnginePowerSource; /*zrodlo mocy dla silnikow*/
	TPowerParameters SystemPowerSource; /*zrodlo mocy dla systemow sterowania/przetwornic/sprezarek*/
	TPowerParameters HeatingPowerSource; /*zrodlo mocy dla ogrzewania*/
	TPowerParameters AlterHeatPowerSource; /*alternatywne zrodlo mocy dla ogrzewania*/
	TPowerParameters LightPowerSource; /*zrodlo mocy dla oswietlenia*/
	TPowerParameters AlterLightPowerSource; /*alternatywne mocy dla oswietlenia*/
	double Vmax = -1.0;
	double Mass = 0.0;
	double Power = 0.0; /*max. predkosc kontrukcyjna, masa wlasna, moc*/
	double Mred = 0.0; /*Ra: zredukowane masy wirujące; potrzebne do obliczeń hamowania*/
	double TotalMass = 0.0; /*wyliczane przez ComputeMass*/
	double HeatingPower = 0.0;
	double EngineHeatingRPM{0.0}; // guaranteed engine revolutions with heating enabled
	double LightPower = 0.0; /*moc pobierana na ogrzewanie/oswietlenie*/
	double BatteryVoltage = 0.0; /*Winger - baterie w elektrykach*/
	bool Battery = false; /*Czy sa zalaczone baterie*/
	start_t BatteryStart = start_t::manual;
	bool EpFuse = true; /*Czy jest wlaczony hamulec EP*/
	double EpForce = 0.0; /*Poziom zadanej sily EP*/
	bool Signalling = false; /*Czy jest zalaczona sygnalizacja hamowania ostatniego wagonu*/
	bool Radio = false; /*Czy jest zalaczony radiotelefon*/
	// the cut-off in the radio's power supply: a fuse in the cabinet, a lever on the desk or the switch on the radio's power
	// supply (eu07/maszyna#438). Closed unless the driver opens it, so vehicles without such a switch work as before
	bool RadioFuse = true;
	float NominalBatteryVoltage = 0.f; /*Winger - baterie w elektrykach*/
	TDimension Dim; /*wymiary*/
	double Cx = 0.0; /*wsp. op. aerodyn.*/
	float Floor = 0.96f; // poziom podłogi dla ładunków
	// height of the centre of gravity of the empty vehicle above the rail [m] (CoGH in the Dimensions of the .fiz),
	// negative when the definition gives none: derailment::default_centre_of_gravity() then
	double CentreOfGravity = -1.0;
	float WheelDiameter = 1.f; /*srednica kol napednych*/
	float WheelDiameterL = 0.9f; // Ra: srednica kol tocznych przednich
	float WheelDiameterT = 0.9f; // Ra: srednica kol tocznych tylnych
	float TrackW = 1.435f; /*nominalna szerokosc toru [m]*/
	double AxleInertialMoment = 0.0; /*moment bezwladnosci zestawu kolowego*/
	std::string AxleArangement; /*uklad osi np. Bo'Bo' albo 1'C*/
	int NPoweredAxles = 0; /*ilosc osi napednych liczona z powyzszego*/
	int NAxles = 0; /*ilosc wszystkich osi j.w.*/
	int BearingType = 1; /*lozyska: 0 - slizgowe, 1 - toczne*/
	double ADist = 0.0;
	double BDist = 0.0; /*odlegosc osi oraz czopow skretu*/
	/*hamulce:*/
	int NBpA = 0; /*ilosc el. ciernych na os: 0 1 2 lub 4*/
	int SandCapacity = 0; /*zasobnik piasku [kg]*/
	TBrakeSystem BrakeSystem = TBrakeSystem::Individual; /*rodzaj hamulca zespolonego*/
	TBrakeSubSystem BrakeSubsystem = TBrakeSubSystem::ss_None;
	TBrakeValve BrakeValve = TBrakeValve::NoValve;
	TBrakeHandle BrakeHandle = TBrakeHandle::NoHandle;
	TBrakeHandle BrakeLocHandle = TBrakeHandle::NoHandle;
	bool LocHandleTimeTraxx = false; /*hamulec dodatkowy typu traxx*/
	double MBPM = 1.0; /*masa najwiekszego cisnienia*/

	int wiperSwitchPos = 0; // pozycja przelacznika wycieraczek
	double WiperAngle = {45.0}; // kat pracy wycieraczek

	std::shared_ptr<TBrake> Hamulec;
	std::shared_ptr<TDriverHandle> Handle;
	std::shared_ptr<TDriverHandle> LocHandle;
	std::shared_ptr<TReservoir> Pipe;
	std::shared_ptr<TReservoir> Pipe2;
	spring_brake SpringBrake;

	TLocalBrake LocalBrake = TLocalBrake::NoBrake; /*rodzaj hamulca indywidualnego*/
	TBrakePressureTable BrakePressureTable; /*wyszczegolnienie cisnien w rurze*/
	TBrakePressure BrakePressureActual; // wartości ważone dla aktualnej pozycji kranu
	int ASBType = 0; /*0: brak hamulca przeciwposlizgowego, 1: reczny, 2: automat*/
	int UniversalBrakeButtonFlag[3] = {0, 0, 0}; /* mozliwe działania przycisków hamulcowych */
	int UniversalResetButtonFlag[3] = {0, 0, 0}; // customizable reset buttons assignments
	int TurboTest = 0;
	bool batterySwAlreadyFired = false; // czy przycisk baterii juz zostal wcisniety
	bool isBatteryButtonImpulse = false; // czy przelacznik baterii traktowac jako pojedynczy przycisk
	bool shouldHoldBatteryButton = false; // czy nalezy przytrzymac przycisk baterii aby wlaczyc/wylaczyc baterie
	float BatteryButtonHoldTime = 1.f; // minimalny czas przytrzymania przycisku baterii
	double MaxBrakeForce = 0.0; /*maksymalna sila nacisku hamulca*/
	double MaxBrakePress[5]; // pomocniczy, proz, sred, lad, pp
	double P2FTrans = 0.0;
	double TrackBrakeForce = 0.0; /*sila nacisku hamulca szynowego*/
	int BrakeMethod = 0; /*flaga rodzaju hamulca*/
	bool Handle_AutomaticOverload = false; // automatyczna asymilacja na pozycji napelniania
	bool Handle_ManualOverload = false; // reczna asymilacja na guzik
	double Handle_GenericDoubleParameter1 = 0.0;
	double Handle_GenericDoubleParameter2 = 0.0;
	double Handle_OverloadMaxPressure = 1.0; // maksymalne zwiekszenie cisnienia przy asymilacji
	double Handle_OverloadPressureDecrease = 0.002; // predkosc spadku cisnienia przy asymilacji
	/*max. cisnienie w cyl. ham., stala proporcjonalnosci p-K*/
	double HighPipePress = 0.0;
	double LowPipePress = 0.0;
	double DeltaPipePress = 0.0;
	/*max. i min. robocze cisnienie w przewodzie glownym oraz roznica miedzy nimi*/
	double CntrlPipePress = 0.0; // ciśnienie z zbiorniku sterującym
	double BrakeVolume = 0.0;
	double BrakeVVolume = 0.0;
	double VeselVolume = 0.0;
	/*pojemnosc powietrza w ukladzie hamulcowym, w ukladzie glownej sprezarki [m^3] */
	int BrakeCylNo = 0; /*ilosc cylindrow ham.*/
	double BrakeCylRadius = 0.0;
	double BrakeCylDist = 0.0;
	double BrakeCylMult[3];
	int LoadFlag = 0;
	/*promien cylindra, skok cylindra, przekladnia hamulcowa*/
	double BrakeCylSpring = 0.0; /*suma nacisku sprezyn powrotnych, kN*/
	double BrakeSlckAdj = 0.0; /*opor nastawiacza skoku tloka, kN*/
	double BrakeRigEff = 0.0; /*sprawnosc przekladni dzwigniowej*/
	double RapidMult = 1.0; /*przelozenie rapidu*/
	double RapidVel = 55.0; /*szybkosc przelaczania rapidu*/
	int BrakeValveSize = 0;
	std::string BrakeValveParams;
	double Spg = 0.0;
	double MinCompressor = 0.0;
	double MaxCompressor = 0.0;
	double MinCompressor_cabA = 0.0;
	double MaxCompressor_cabA = 0.0;
	double MinCompressor_cabB = 0.0;
	double MaxCompressor_cabB = 0.0;
	bool CabDependentCompressor = false;
	double CompressorSpeed = 0.0;
	int CompressorList[4][9]; // pozycje świateł, przód - tył, 1 .. 16
	double EmergencyValveOn = 0.0;
	double EmergencyValveOff = 0.0;
	bool EmergencyValveOpen = false;
	double EmergencyValveArea = 0.0;
	double LockPipeOn = -1.0;
	double LockPipeOff = -1.0;
	double HandleUnlock = -3.0;
	bool EmergencyCutsOffHandle = false;
	int CompressorListPosNo = 0;
	int CompressorListDefPos = 1;
	bool CompressorListWrap = false;
	/*cisnienie wlaczania, zalaczania sprezarki, wydajnosc sprezarki*/
	TBrakeDelayTable BrakeDelay; /*opoznienie hamowania/odhamowania t/o*/
	double AirLeakRate{0.01}; // base rate of air leak from brake system components ( 0.001 = 1 l/sec )
	int BrakeCtrlPosNo = 0; /*ilosc pozycji hamulca*/
	/*nastawniki:*/
	int MainCtrlPosNo = 0; /*ilosc pozycji nastawnika*/
	int ScndCtrlPosNo = 0;
	int LightsPosNo = 0;
	int LightsDefPos = 1;
	bool LightsWrap = false;
	int Lights[2][18]; // pozycje świateł, przód - tył, 1 .. 16 (17 do lightsset)
	// the cab interior light each light preset sets, from the optional third column of LightsList (bits of
	// lights_preset_cablight::flag); -1: the preset leaves the cab light as it is (eu07/maszyna#529)
	std::array<int, 18> LightsCabLight{[] {
		std::array<int, 18> positions;
		positions.fill(-1);
		return positions;
	}()};
	int ScndInMain{0}; /*zaleznosc bocznika od nastawnika*/
	bool MBrake = false; /*Czy jest hamulec reczny*/
	double maxTachoSpeed = {0.0}; // maksymalna predkosc na tarczce predkosciomierza analogowego
	double StopBrakeDecc = 0.0;
	// the holding brake (StopBrakeDecc) of a unit with the MED lets go only once power has been demanded for this long (SBDReleaseDelay, s);
	// 0 lets it go at the first demand
	double StopBrakeReleaseDelay = 0.0;
	double StopBrakeReleaseTimer = 0.0; // how long power has been demanded without a break, s
	bool ReleaseParkingBySpringBrake{false};
	bool ReleaseParkingBySpringBrakeWhenDoorIsOpen{false};
	bool SpringBrakeCutsOffDrive{true};
	double SpringBrakeDriveEmergencyVel{-1};
	bool HideDirStatusWhenMoving{false}; // Czy gasic lampki kierunku powyzej predkosci zdefiniowanej przez HideDirStatusSpeed
	int HideDirStatusSpeed{1}; // Predkosc od ktorej lampki kierunku sa wylaczane
	bool isDoubleClickForMeasureNeeded = {false}; // czy rozpoczecie pomiaru odleglosci odbywa sie po podwojnym wcisnienciu przycisku?
	float DistanceCounterDoublePressPeriod = {1.f}; // czas w jakim nalezy podwojnie wcisnac przycisk, aby rozpoczac pomiar odleglosci
	TSecuritySystem SecuritySystem;
	int EmergencyBrakeWarningSignal{0}; // combined with basic WarningSignal when manual emergency brake is active
	horn_supply HornSupply{horn_supply::air}; // of the low and high tone horns (WarningSignal 1 and 2)
	horn_supply Horn3Supply{horn_supply::air}; // of the third horn (WarningSignal 4)
	// main reservoir pressure [bar, as MinCP and MaxCP] below which a pneumatic horn stays silent
	static constexpr double HornMinPressure{2.0};
	TUniversalCtrlTable UniCtrlList; /*lista pozycji uniwersalnego nastawnika*/
	int UniCtrlListSize = 0; /*wielkosc listy pozycji uniwersalnego nastawnika*/
	bool UniCtrlIntegratedBrakePNCtrl = false; /*zintegrowany nastawnik JH obsluguje hamulec PN*/
	bool UniCtrlIntegratedBrakeCtrl = false; /*zintegrowany nastawnik JH obsluguje hamowanie*/
	bool UniCtrlIntegratedLocalBrakeCtrl = false; /*zintegrowany nastawnik JH obsluguje hamowanie hamulcem pomocniczym*/
	int UniCtrlNoPowerPos{0}; // cached highesr position not generating traction force
	// UCList CtrlValStep: the universal controller (EIMCtrlType 3) changes its setting in steps of this size (e.g. 0.05 for the
	// 5% steps of the 6Dg), one at once when moved to a raising or lowering position, then one each time its SpeedUp / SpeedDown
	// covers a step; 0 (the default) changes it continuously
	double UniCtrlValStep{0.0};
	std::pair<std::string, std::array<int, 2>> PantsPreset{"0132", {0, 0}}; // pantograph preset switches; .first holds possible setups as chars, .second holds currently selected preset in each cab

	/*-sekcja parametrow dla lokomotywy elektrycznej*/
	TSchemeTable RList; /*lista rezystorow rozruchowych i polaczen silnikow, dla dizla: napelnienia*/
	int RlistSize = 0;
	TMotorParameters MotorParam[MotorParametersArraySize + 1];
	/*rozne parametry silnika przy bocznikowaniach*/
	/*dla lokomotywy spalinowej z przekladnia mechaniczna: przelozenia biegow*/
	// bounds-safe read access to MotorParam
	TMotorParameters const &motor_param(int const Position) const
	{
		if (Position < 0)
			return MotorParam[0];
		if (Position > MotorParametersArraySize)
			return MotorParam[MotorParametersArraySize];
		return MotorParam[Position];
	}
	TTransmision Transmision;
	// record   {liczba zebow przekladni}
	//  NToothM, NToothW : byte;
	//  Ratio: real;  {NToothW/NToothM}
	// end;
	double NominalVoltage = 0.0; /*nominalne napiecie silnika*/
	double WindingRes = 0.0;
	double u = 0.0; // wspolczynnik tarcia yB wywalic!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	double CircuitRes = 0.0; /*rezystancje silnika i obwodu*/
	int IminLo = 0;
	int IminHi = 0; /*prady przelacznika automatycznego rozruchu, uzywane tez przez ai_driver*/
	int ImaxLo = 0; // maksymalny prad niskiego rozruchu
	int ImaxHi = 0; // maksymalny prad wysokiego rozruchu
	bool MotorOverloadRelayHighThreshold{false};
	double nmax = 0.0; /*maksymalna dop. ilosc obrotow /s*/
	double InitialCtrlDelay = 0.0;
	double CtrlDelay = 0.0; /* -//-  -//- miedzy kolejnymi poz.*/
	double CtrlDownDelay = 0.0; /* -//-  -//- przy schodzeniu z poz.*/ /*hunter-101012*/
	int FastSerialCircuit = 0; /*0 - po kolei zamyka styczniki az do osiagniecia szeregowej, 1 - natychmiastowe wejscie na szeregowa*/ /*hunter-111012*/
	int BackwardsBranchesAllowed = 1;
	int AutoRelayType = 0; /*0 -brak, 1 - jest, 2 - opcja*/
	bool CoupledCtrl = false; /*czy mainctrl i scndctrl sa sprzezone*/
	bool HasCamshaft{false};
	int DynamicBrakeType = 0; /*patrz dbrake_**/
	int DynamicBrakeAmpmeters = 2; /*liczba amperomierzy przy hamowaniu ED*/
	bool SplitEDPneumaticBrake = false; /*czy hamulec ED i pneumatyczny pomocniczy są rozdzielone na osobne nastawniki*/
	int DynamicBrakeCtrlPosNo = 10; /*ilosc pozycji nastawnika hamulca ED (DBPN, domyslnie 10)*/
	// the dynamic brake of a diesel-electric vehicle (Engine: EDBrake=...): the traction motors brake into resistors while
	// the engine runs, with the force of the definition rather than from a circuit, as the drive of these vehicles is
	// computed from the generator's power. no definition, no dynamic brake (MaSzyna-EU07/maszyna#82)
	struct diesel_electric_dynamic_brake
	{
		enum class control
		{
			none,
			controller, // the negative range of the integrated controller (EIMCtrlType > 0), as the TRAXX one of the BR285
			lever // a lever of its own (the SplitEDPneumaticBrake one, DBPN steps), with the master controller at zero
		};
		control source{control::none};
		double max_force{0.0}; // [N] at the wheel rims of the whole vehicle, EDBrakeFmax [kN]
		double max_power{0.0}; // [W] the resistors take, EDBrakePmax [kW]; 0: no limit
		double fade_low{0.0}; // [km/h] EDBrakeV0: no force below
		double fade_high{0.0}; // [km/h] EDBrakeV1: the whole force above, rising in between
		double rate{0.5}; // [1/s] EDBrakeRate: how fast the effort follows the demand
		bool blending{false}; // EDBrakeBlending: the locomotive's train brake lets off while the dynamic brake works (Hamulec->SetED)
		// state
		double level{0.0}; // the effort, 0..1
		double force{0.0}; // [N] the braking force, against the motion
	};
	diesel_electric_dynamic_brake DEDynamicBrake;
	double DynamicBrakeRes = 5.8; /*rezystancja oporników przy hamowaniu ED*/
	double DynamicBrakeRes1 = 5.8; /*rezystancja oporników przy hamowaniu ED - 1 tryb*/
	double DynamicBrakeRes2 = 5.8; /*rezystancja oporników przy hamowaniu ED - 2 tryb*/
	double TUHEX_Sum = 750; /*nastawa sterownika hamowania ED - aktualna*/
	double TUHEX_Diff = 10; /*histereza działania sterownika hamowania ED*/
	double TUHEX_MinIw = 60; /*minimalny prąd wzbudzenia przy hamowaniu ED - wynika z chk silnika*/
	double TUHEX_MaxIw = 400; /*maksymalny prąd wzbudzenia przy hamowaniu ED - ograniczenie max siły*/
	double TUHEX_Sum1 = 750; /*nastawa1 sterownika hamowania ED*/
	double TUHEX_Sum2 = 750; /*nastawa2 sterownika hamowania ED*/
	double TUHEX_Sum3 = 750; /*nastawa3 sterownika hamowania ED*/
	int TUHEX_Stages = 0; /*liczba stopni hamowania ED*/
	// TODO: wrap resistor fans variables and state into a device
	int RVentType = 0; /*0 - brak, 1 - jest, 2 - automatycznie wlaczany*/
	double RVentnmax = 1.0; /*maks. obroty wentylatorow oporow rozruchowych*/
	double RVentCutOff = 0.0; /*rezystancja wylaczania wentylatorow dla RVentType=2*/
	double RVentSpeed{0.5}; // rozpedzanie sie wentylatora obr/s^2}
	double RVentMinI{50.0}; // przy jakim pradzie sie wylaczaja}
	bool RVentForceOn{false}; // forced activation switch
	int CompressorPower = 1; // 0: main circuit, 1: z przetwornicy, reczne, 2: w przetwornicy, stale, 3: diesel engine, 4: converter of unit in front, 5: converter of unit behind
	int SmallCompressorPower = 0; /*Winger ZROBIC*/
	bool Trafo = false; /*pojazd wyposażony w transformator*/

	/*-sekcja parametrow dla lokomotywy spalinowej z przekladnia mechaniczna*/
	double dizel_Mmax = 1.0;
	double dizel_nMmax = 1.0;
	double dizel_Mnmax = 2.0;
	double dizel_nmax = 2.0;
	double dizel_nominalfill = 0.0;
	std::map<double, double> dizel_Momentum_Table;
	std::map<double, double> dizel_vel2nmax_Table;
	/*parametry aproksymacji silnika spalinowego*/
	double dizel_Mstand = 0.0; /*moment oporow ruchu silnika dla enrot=0*/
	/*                dizel_auto_min, dizel_auto_max: real; {predkosc obrotowa przelaczania automatycznej skrzyni biegow*/
	double dizel_nmax_cutoff = 0.0; /*predkosc obrotowa zadzialania ogranicznika predkosci*/
	double dizel_nmin = 0.0; /*najmniejsza dopuszczalna predkosc obrotowa*/
	double dizel_nmin_hdrive = 0.0; /*najmniejsza dopuszczalna predkosc obrotowa w czasie jazdy na hydro */
	double dizel_nmin_hdrive_factor = 0.0; /*wspolczynnik wzrostu obrotow minimalnych hydro zaleznosci od zadanego procentu*/
	double dizel_nmin_retarder = 0.0; /*obroty pracy podczas hamowania retarderem*/
	double dizel_nreg_acc = 999.0; /*tempo zwiększania prędkości przez regulator dizla */
	double dizel_minVelfullengage = 0.0; /*najmniejsza predkosc przy jezdzie ze sprzeglem bez poslizgu*/
	double dizel_maxVelANS = 3.0; /*predkosc progowa rozlaczenia przetwornika momentu*/
	double dizel_AIM = 1.0; /*moment bezwladnosci walu itp*/
	double dizel_RevolutionsDecreaseRate{2.0};
	double dizel_NominalFuelConsumptionRate = 250.0; /*jednostkowe zużycie paliwa przy mocy nominalnej, wczytywane z fiz, g/kWh*/
	double dizel_FuelConsumption = 0.0; /*współczynnik zużycia paliwa przeliczony do jednostek maszynowych, l/obrót*/
	double dizel_FuelConsumptionActual = 0.0; /*chwilowe spalanie paliwa w l/h*/
	double dizel_FuelConsumptedTotal = 0.0; /*ilość paliwa zużyta od początku symulacji, l*/
	double dizel_engageDia = 0.5;
	double dizel_engageMaxForce = 6000.0;
	double dizel_engagefriction = 0.5; /*parametry sprzegla*/
	double engagedownspeed = 0.9;
	double engageupspeed = 0.5;
	/*parametry przetwornika momentu*/
	bool hydro_TC = false; /*obecnosc hydraulicznego przetwornika momentu*/
	double hydro_TC_TMMax = 2.0; /*maksymalne wzmocnienie momentu*/
	double hydro_TC_CouplingPoint = 0.85; /*wzgledna predkosc zasprzeglenia*/
	double hydro_TC_LockupTorque = 3000.0; /*moment graniczny sprzegla blokujacego*/
	double hydro_TC_LockupRate = 1.0; /*szybkosc zalaczania sprzegla blokujacego*/
	double hydro_TC_UnlockRate = 1.0; /*szybkosc rozlaczania sprzegla blokujacego*/
	double hydro_TC_FillRateInc = 1.0; /*szybkosc napelniania sprzegla*/
	double hydro_TC_FillRateDec = 1.0; /*szybkosc oprozniania sprzegla*/
	double hydro_TC_TorqueInIn = 4.5; /*stala momentu proporcjonalnego do kwadratu obrotow wejsciowych*/
	double hydro_TC_TorqueInOut = 0.0; /*stala momentu proporcjonalnego do roznica obrotow wejsciowych i wyjsciowych*/
	double hydro_TC_TorqueOutOut = 0.0; /*stala momentu proporcjonalnego do kwadratu obrotow wyjsciowych*/
	double hydro_TC_LockupSpeed = 1.0; /*prog predkosci zalaczania sprzegla blokujacego*/
	double hydro_TC_UnlockSpeed = 1.0; /*prog predkosci rozlaczania sprzegla blokujacego*/
	std::map<double, double> hydro_TC_Table; /*tablica przetwornika momentu*/
	/*parametry retardera*/
	bool hydro_R = false; /*obecnosc retardera*/
	int hydro_R_Placement = 0; /*umiejscowienie retardera: 0 - za skrzynia biegow, 1 - miedzy przetwornikiem a biegami, 2 - przed skrzynia biegow */
	double hydro_R_TorqueInIn = 1.0; /*stala momentu proporcjonalnego do kwadratu obrotow wejsciowych*/
	double hydro_R_MaxTorque = 1.0; /*maksymalny moment retardera*/
	double hydro_R_MaxPower = 1.0; /*maksymalna moc retardera - ogranicza moment*/
	double hydro_R_FillRateInc = 1.0; /*szybkosc napelniania sprzegla*/
	double hydro_R_FillRateDec = 1.0; /*szybkosc oprozniania sprzegla*/
	double hydro_R_MinVel = 1.0; /*minimalna predkosc, przy ktorej retarder dziala*/
	double hydro_R_EngageVel = 1.0; /*minimalna predkosc hamowania, przy ktorej sprzeglo jest wciaz wlaczone*/
	bool hydro_R_Clutch = false; /*czy retarder ma rozłączalne sprzęgło*/
	double hydro_R_ClutchSpeed = 10.0; /*szybkość narastania obrotów po włączeniu sprzęgła retardera*/
	bool hydro_R_WithIndividual = false; /*czy dla autobusów jest to łączone*/
	/*- dla lokomotyw spalinowo-elektrycznych -*/
	double AnPos = 0.0; // pozycja sterowania dokladnego (analogowego)
	bool AnalogCtrl = false; //
	bool AnMainCtrl = false; //
	bool ShuntModeAllow = false;
	bool ShuntMode = false;

	bool Flat = false;
	double Vhyp = 1.0;
	TDESchemeTable DElist;
	TFFSchemeTable FFlist;
	int FFListSize = 0;
	TFFSchemeTable FFEDlist;
	int FFEDListSize = 0;
	TWiperSchemeTable WiperList;
	int WiperListSize;
	int WiperDefaultPos{0};
	int modernWpierListSize;

	double Vadd = 1.0;
	TMPTRelayTable MPTRelay;
	int RelayType = 0;
	TShuntSchemeTable SST;
	double PowerCorRatio = 1.0; // Wspolczynnik korekcyjny
	/*- dla uproszczonego modelu silnika (dumb) oraz dla drezyny*/
	double Ftmax = 0.0;
	/*- dla lokomotyw z silnikami indukcyjnymi -*/
	double eimc[26];
	bool EIMCLogForce = false; //
	static std::vector<std::string> const eimc_labels;
	double InverterFrequency{0.0}; // current frequency of power inverters
	int InvertersNo = 0; // number of inverters
	double InvertersRatio = 0.0;
	std::vector<inverter> Inverters; // all inverters
	int InverterControlCouplerFlag = 4; // which coupling flag is necessary to controll inverters
	int Imaxrpc = 0; // Maksymalny prad rezystora hamowania chlodzonego pasywnie
	int BRVto = 0; // Czas jaki wentylatory jeszcze dodatkowo schladzaja rezystor
	double BRVtimer = 0; // Timer dla podtrzymania wentylatora
	std::map<double, double> EIM_Pmax_Table; /*tablica mocy maksymalnej od predkosci*/
	/* -dla pojazdów z blendingiem EP/ED (MED) */
	double MED_Vmax = 0; // predkosc maksymalna dla obliczen chwilowej sily hamowania EP w MED
	double MED_Vmin = 0; // predkosc minimalna dla obliczen chwilowej sily hamowania EP w MED
	double MED_Vref = 0; // predkosc referencyjna dla obliczen dostepnej sily hamowania EP w MED
	double MED_amax{9.81}; // maksymalne opoznienie hamowania sluzbowego MED
	bool MED_EPVC = false; // czy korekcja sily hamowania EP, gdy nie ma dostepnego ED
	double MED_EPVC_Time = 7; // czas korekcji sily hamowania EP, gdy nie ma dostepnego ED
	bool MED_Ncor = false; // czy korekcja sily hamowania z uwzglednieniem nacisku
	double MED_MinBrakeReqED = 0; // minimalne zadanie sily hamowania uruchamiajace ED - ponizej tylko EP
	double MED_FrED_factor = 1; // mnoznik sily hamowania ED do korekty blendingu
	double MED_ED_Delay1 = 0; // opoznienie wdrazania hamowania ED (pierwszy raz)
	double MED_ED_Delay2 = 0; // opoznienie zwiekszania sily hamowania ED (kolejne razy)

	int DCEMUED_CC{0}; // na którym sprzęgu sprawdzać działanie ED
	double DCEMUED_EP_max_Vel{0.0}; // maksymalna prędkość, przy której działa EP przy włączonym ED w jednostce (dla tocznych)
	double DCEMUED_EP_min_Im{0.0}; // minimalny prąd, przy którym EP nie działa przy włączonym ED w członie (dla silnikowych)
	double DCEMUED_EP_delay{0.0}; // opóźnienie włączenia hamulca EP przy hamowaniu ED - zwłoka wstępna

	/*-dla wagonow*/
	
	std::vector<load_attributes> LoadAttributes;
	float MaxLoad = 0.f; /*masa w T lub ilosc w sztukach - ladownosc*/
	double OverLoadFactor = 0.0; /*ile razy moze byc przekroczona ladownosc*/
	float LoadSpeed = 0.f;
	float UnLoadSpeed = 0.f; /*szybkosc na- i rozladunku jednostki/s*/
#ifdef EU07_USEOLDDOORCODE
	int DoorOpenCtrl = 0;
	int DoorCloseCtrl = 0; /*0: przez pasazera, 1: przez maszyniste, 2: samoczynne (zamykanie)*/
	double DoorStayOpen = 0.0; /*jak dlugo otwarte w przypadku DoorCloseCtrl=2*/
	bool DoorClosureWarning = false; /*czy jest ostrzeganie przed zamknieciem*/
	bool DoorClosureWarningAuto = false; // departure signal plays automatically while door closing button is held down
	double DoorOpenSpeed = 1.0;
	double DoorCloseSpeed = 1.0; /*predkosc otwierania i zamykania w j.u. */
	double DoorMaxShiftL = 0.5;
	double DoorMaxShiftR = 0.5;
	double DoorMaxPlugShift = 0.1; /*szerokosc otwarcia lub kat*/
	int DoorOpenMethod = 2; /*sposob otwarcia - 1: przesuwne, 2: obrotowe, 3: trójelementowe*/
	float DoorCloseDelay{0.f}; // delay (in seconds) before the door begin closing, once conditions to close are met
	double PlatformSpeed = 0.5; /*szybkosc stopnia*/
	double PlatformMaxShift{0.0}; /*wysuniecie stopnia*/
	int PlatformOpenMethod{2}; /*sposob animacji stopnia*/
#endif
	double MirrorMaxShift{90.0};
	double MirrorVelClose{5.0};
	bool MirrorForbidden{false}; /*czy jest pozwolenie na otworzenie lusterek (przycisk)*/
	bool ScndS = false; /*Czy jest bocznikowanie na szeregowej*/
	bool ScndCtrlResistiveLock{false}; // the shunt controller can't be moved, up or down, while the master controller is at zero or on a resistive position, and drops to 0 when the master controller gets there (ScndCtrlResistiveLock=Yes in Cntrl.)
	bool SpeedCtrl = false; /*czy jest tempomat*/
	speed_control SpeedCtrlUnit; /*parametry tempomatu*/
	double SpeedCtrlButtons[10]{30, 40, 50, 60, 70, 80, 90, 100, 110, 120}; /*przyciski prędkości*/
	double SpeedCtrlDelay = 2; /*opoznienie dzialania tempomatu z wybieralna predkoscia*/
	double SpeedCtrlValue = 0; /*wybrana predkosc jazdy na tempomacie*/
	/*--sekcja zmiennych*/
	/*--opis konkretnego egzemplarza taboru*/
	TLocation Loc{0.0, 0.0, 0.0}; // pozycja pojazdów do wyznaczenia odległości pomiędzy sprzęgami
	TRotation Rot{0.0, 0.0, 0.0};
	glm::vec3 Front{};
	std::string Name; /*nazwa wlasna*/
	TCoupling Couplers[2]; // urzadzenia zderzno-sprzegowe, polaczenia miedzy wagonami
	// the physical coupler model covers the vehicle: it's in the player's train or in one the player's train closes in on
	// (vehicle_table::update_coupler_physics()); with couplerphysics all in eu07.ini every vehicle is covered
	bool CouplerPhysics{false};
	std::array<neighbour_data, 2> Neighbours; // potential collision sources
	bool Power110vIsAvailable = false; // cached availability of 110v power
	bool Power24vIsAvailable = false; // cached availability of 110v power
	double Power24vVoltage{0.0}; // cached battery voltage
	bool EventFlag = false; /*!o true jesli cos nietypowego sie wydarzy*/
	int SoundFlag = 0; /*!o patrz stale sound_ */
	int AIFlag{0}; // HACK: events of interest for consist owner
	double DistCounter = 0.0; /*! licznik kilometrow */
	// the criteria of derailment (physics.derailment new), and the distance travelled since they were checked [m]
	derailment::monitor derailment_monitor;
	double derailment_distance = 0.0;
	// physics.derailment monitor: the name of the track the criteria were last reported on (one report on a track), and
	// how many reports there were
	std::optional<std::string const *> derailment_reported;
	int derailment_reports = 0;
	odometer::reading Odometer; // total distance run, kept between the sessions (eu07/maszyna#72)
	std::pair<double, double> EnergyMeter; // energy <drawn, returned> from grid [kWh]
	double V = 0.0; // predkosc w [m/s] względem sprzęgów (dodania gdy jedzie w stronę 0)
	double Vel = 0.0; // moduł prędkości w [km/h], używany przez AI
	double AccS = 0.0; // efektywne przyspieszenie styczne w [m/s^2] (wszystkie siły)
	double AccSVBased{}; // tangential acceleration calculated from velocity change
	double AccN = 0.0; // przyspieszenie normalne w [m/s^2]
	double AccVert = 0.0; // vertical acceleration
	double nrot = 0.0; // predkosc obrotowa kol (obrotow na sekunde)
	double nrot_eps = 0.0; // przyspieszenie kątowe kół (bez kierunku)
	double WheelFlat = 0.0;
	bool TruckHunting{true}; // enable/disable truck hunting calculation
	/*! rotacja kol [obr/s]*/
	double EnginePower = 0.0; /*! chwilowa moc silnikow*/
	double dL = 0.0;
	double Fb = 0.0;
	double Ff = 0.0; /*przesuniecie, sila hamowania i tarcia*/
	double FTrain = 0.0;
	double FStand = 0.0; /*! sila pociagowa i oporow ruchu*/
	double FTotal = 0.0; /*! calkowita sila dzialajaca na pojazd*/
	double UnitBrakeForce = 0.0; /*!s siła hamowania przypadająca na jeden element*/
	double Ntotal = 0.0; /*!s siła nacisku klockow*/
	bool SlippingWheels = false;
	bool SandDose = false; /*! poslizg kol, sypanie piasku*/
	bool SandDoseManual = false; /*piaskowanie reczne*/
	bool SandDoseAuto = false; /*piaskowanie automatyczne*/
	bool SandDoseAutoAllow = true; /*zezwolenie na automatyczne piaskowanie*/
	double Sand = 0.0; /*ilosc piasku*/
	double BrakeSlippingTimer = 0.0; /*pomocnicza zmienna do wylaczania przeciwposlizgu*/
	double dpBrake = 0.0;
	double dpPipe = 0.0;
	double dpMainValve = 0.0;
	double dpLocalValve = 0.0;
	double EmergencyValveFlow = 0.0; // air flow through alerter valve during last simulation step
	/*! przyrosty cisnienia w kroku czasowym*/
	double ScndPipePress = 0.0; /*cisnienie w przewodzie zasilajacym*/
	bool ScndPipeCouplersPending{false}; // the main reservoir pipe stepped, its air exchange through the hoses is yet to come (UpdatePipeCouplers())
	double BrakePress = 0.0; /*!o cisnienie w cylindrach hamulcowych*/
	double LocBrakePress = 0.0; /*!o cisnienie w cylindrach hamulcowych z pomocniczego*/
	double PipeBrakePress = 0.0; /*!o cisnienie w cylindrach hamulcowych z przewodu*/
	double PipePress = 0.0; /*!o cisnienie w przewodzie glownym*/
	bool PipeCouplersPending{false}; // the brake pipe stepped, its air exchange through the hoses is yet to come (UpdatePipeCouplers())
	double EqvtPipePress = 0.0; /*!o cisnienie w przewodzie glownym skladu*/
	double Volume = 0.0; /*objetosc spr. powietrza w zbiorniku hamulca*/
	double CompressedVolume = 0.0; /*objetosc spr. powietrza w ukl. zasilania*/
	double PantVolume = 0.48; /*objetosc spr. powietrza w ukl. pantografu*/ // aby podniesione pantografy opadły w krótkim czasie przy wyłączonej sprężarce
	double Compressor = 0.0; /*! cisnienie w ukladzie zasilajacym*/
	bool CompressorFlag = false; /*!o czy wlaczona sprezarka*/
	bool PantCompFlag = false; /*!o czy wlaczona sprezarka pantografow*/
	bool CompressorAllow = false; /*! zezwolenie na uruchomienie sprezarki  NBMX*/
	bool CompressorAllowLocal{true}; // local device state override (most units don't have this fitted so it's set to true not to intefere)
	bool CompressorGovernorLock{false}; // indicates whether compressor pressure switch was activated due to reaching cut-out pressure
	double CompressorStartDelay{0.0}; // time [s] the compressor waits after getting its power before it can start, after the engine starts say
	double CompressorPowerTime{0.0}; // time [s] since the compressor got its power
	bool CompressorTankValve{false}; // indicates excessive pressure is vented from compressor tank directly and instantly
	start_t CompressorStart{start_t::manual}; // whether the compressor is started manually, or another way
	start_t PantographCompressorStart{start_t::manual};
	basic_valve PantsValve;
	std::array<basic_pantograph, 2> Pantographs;
	bool PantAllDown{false};
	double PantFrontVolt = 0.0; // pantograf pod napieciem? 'Winger 160404
	double PantRearVolt = 0.0;
	// TODO converter parameters, for when we start cleaning up mover parameters
	start_t ConverterStart{start_t::manual}; // whether converter is started manually, or by other means
	float ConverterStartDelay{0.0f}; // delay (in seconds) before the converter is started, once its activation conditions are met
	double ConverterStartDelayTimer{0.0}; // helper, for tracking whether converter start delay passed
	bool ConverterAllow = false; /*zezwolenie na prace przetwornicy NBMX*/
	bool ConverterAllowLocal{true}; // local device state override (most units don't have this fitted so it's set to true not to intefere)
	bool ConverterFlag = false; /*!  czy wlaczona przetwornica NBMX*/
	bool BRVentilators = false; /* Czy rezystor hamowania pracuje */
	start_t ConverterOverloadRelayStart{start_t::manual}; // whether overload relay reset responds to dedicated button
	bool ConverterOverloadRelayOffWhenMainIsOff{false};
	fuel_pump FuelPump;
	oil_pump OilPump;
	water_pump WaterPump;
	water_heater WaterHeater;
	bool WaterCircuitsLink{false}; // optional connection between water circuits
	heat_data dizel_heat;
	std::array<cooling_fan, 2> MotorBlowers;
	door_data Doors;
	float DoorsOpenWithPermitAfter{-1.f}; // remote open if permit button is held for specified time. NOTE: separate from door data as its cab control thing
	int DoorsPermitLightBlinking{0}; // when the doors permit signal light is blinking

	int BrakeCtrlPos = -2; /*nastawa hamulca zespolonego*/
	double BrakeCtrlPosR = 0.0; /*nastawa hamulca zespolonego - plynna dla FV4a*/
	double BrakeCtrlPos2 = 0.0; /*nastawa hamulca zespolonego - kapturek dla FV4a*/
	int ManualBrakePos = 0; /*nastawa hamulca recznego*/
	double LocalBrakePosA = 0.0; /*nastawa hamulca pomocniczego*/
	double LocalBrakePosAEIM = 0.0; /*pozycja hamulca pomocniczego ep dla asynchronicznych ezt*/
	double DynamicBrakeCtrlPos = 0.0; /*nastawa nastawnika hamulca elektrodynamicznego (0..1) - aktywne tylko gdy SplitEDPneumaticBrake*/
	bool UniversalBrakeButtonActive[3] = {false, false, false}; /* brake button pressed */
	/*
	    int BrakeStatus = b_off; //0 - odham, 1 - ham., 2 - uszk., 4 - odluzniacz, 8 - antyposlizg, 16 - uzyte EP, 32 - pozycja R, 64 - powrot z R
	*/
	bool AlarmChainFlag = false; // manual emergency brake
	bool RadioStopFlag = false; /*hamowanie nagle*/
	bool LockPipe = false; /*locking brake pipe in emergency state*/
	bool UnlockPipe = false; /*unlockig brake pipe button pressed*/
	int BrakeDelayFlag = 0; /*nastawa opoznienia ham. osob/towar/posp/exp 0/1/2/4*/
	int BrakeDelays = 0; /*nastawy mozliwe do uzyskania*/
	// the AI wagon inspector cut the brake out (b_dmg) and puts it back when the train no longer needs it
	// cut out (brake_setting::apply). Anyone else cutting it out or putting it back takes it over
	bool BrakeCutOutByInspector = false;
	int BrakeOpModeFlag = 0; /*nastawa trybu pracy PS/PN/EP/MED 1/2/4/8*/
	int BrakeOpModes = 0; /*nastawy mozliwe do uzyskania*/
	bool DynamicBrakeFlag = false; /*czy wlaczony hamulec elektrodymiczny*/
	bool DynamicBrakeEMUStatus = true; /*czy hamulec ED dziala w ezt*/
	int TUHEX_StageActual = 0; /*aktualny stopien tuhexa*/
	bool TUHEX_ResChange = false; /*czy zmiana rezystancji hamowania ED - odwzbudzanie*/
	bool TUHEX_Active = false; /*czy hamowanie ED wywołane z zewnątrz*/
	//                NapUdWsp: integer;
	double LimPipePress = 0.0; /*stabilizator cisnienia*/
	double ActFlowSpeed = 0.0; /*szybkosc stabilizatora*/

	int DamageFlag = 0; // kombinacja bitowa stalych dtrain_* }
	int EngDmgFlag = 0; // kombinacja bitowa stalych usterek}

	// EndSignalsFlag: byte;  {ABu 060205: zmiany - koncowki: 1/16 - swiatla prz/tyl, 2/31 - blachy prz/tyl}
	// HeadSignalsFlag: byte; {ABu 060205: zmiany - swiatla: 1/2/4 - przod, 16/32/63 - tyl}
	TCommand CommandIn;
	/*komenda przekazywana przez PutCommand*/
	/*i wykonywana przez RunInternalCommand*/
	std::string CommandOut; /*komenda przekazywana przez ExternalCommand*/
	std::string CommandLast; // Ra: ostatnio wykonana komenda do podglądu
	double ValueOut = 0.0; /*argument komendy która ma byc przekazana na zewnatrz*/

	TTrackShape RunningShape; /*geometria toru po ktorym jedzie pojazd*/
	TTrackParam RunningTrack; /*parametry toru po ktorym jedzie pojazd*/
	double OffsetTrackH = 0.0;
	double OffsetTrackV = 0.0; /*przesuniecie poz. i pion. w/m osi toru*/

	/*-zmienne dla lokomotyw*/
	bool Mains = false; /*polozenie glownego wylacznika*/
	double MainsInitTime{0.0}; // config, initialization time (in seconds) of the main circuit after it receives power, before it can be closed
	double MainsInitTimeCountdown{0.0}; // current state of main circuit initialization, remaining time (in seconds) until it's ready
	start_t MainsStart{start_t::manual};
	// config, Engine:JointEngineStart. whether the cab's line breaker / engine start switch works the vehicles in multiple
	// too; when it doesn't, the cab has separate switches for them (mainb_*, mainc_*)
	bool JointEngineStart{true};
	// config, .fiz Cntrl.PantRequiresMain: the line breaker closes without traction voltage and holds without it, and the
	// pantographs rise and stay up only while it's closed (EU05, EP05, ČD 181 family); off, the pantographs come first and the line
	// breaker needs their voltage. what the main circuit feeds still needs the voltage (MainCircuitPowered()), and without
	// it the undervoltage relay opens the line contactors (NoVoltRelay)
	bool PantographsRequireMains{false};
	bool LineBreakerClosesOnlyAtNoPowerPos{false};
	bool ControlPressureSwitch{false}; // activates if the main pipe and/or brake cylinder pressure aren't within operational levels
	bool HasControlPressureSwitch{true};
	// config, .fiz Engine:PressureSwitchBrakeCylinder. whether the pressure switch watches the brake cylinders (it cuts the
	// start off above 2 bar there) as well as the brake pipe (below 3.6 bar). off, it's left with the brake pipe alone,
	// and the vehicle takes current whatever the pressure in its brake cylinders (EP05)
	bool PressureSwitchBrakeCylinder{true};
	// config, .fiz Cntrl.EmergencyBrakeMainOff. the brake pipe pressure [bar] taken for an emergency brake application:
	// below it, with the master controller on a power position, the line breaker opens and doesn't close again. where
	// the pantographs stay up only through the closed line breaker (PantographsRequireMains) they drop with it (EP05).
	// 0: an emergency application leaves the line breaker alone
	double EmergencyBrakeMainOffPressure{0.0};
	bool ReleaserEnabledOnlyAtNoPowerPos{false};
	int MainCtrlPos = 0; /*polozenie glownego nastawnika*/
	int ScndCtrlPos = 0; /*polozenie dodatkowego nastawnika*/
	int LightsPos = 0; /*polozenie przelacznika wielopozycyjnego swiatel*/
	int CompressorListPos = 0; /*polozenie przelacznika wielopozycyjnego sprezarek*/
	int DirActive = 0; // czy lok. jest wlaczona i w ktorym kierunku: względem wybranej kabiny: -1 - do tylu, +1 - do przodu, 0 - wylaczona
	int DirAbsolute = 0; // zadany kierunek jazdy względem sprzęgów (1=w strone 0,-1=w stronę 1)
	int MainCtrlMaxDirChangePos{0}; // can't change reverser state with master controller set above this position
	int CabActive = 0; // numer kabiny, z której jest sterowanie: 1 lub -1; w przeciwnym razie brak sterowania - rozrzad
	int CabOccupied = 0; // numer kabiny, w ktorej jest obsada (zwykle jedna na skład) // TODO: move to TController
	bool CabMaster = false; // czy pojazd jest nadrzędny w składzie
	// pullee mode ("jazda ciągniona", MaSzyna-EU07/maszyna#64): the vehicle is prepared for being hauled dead in a
	// train, see PulleeModeSwitch()
	bool PulleeMode{false};
	inline bool IsCabMaster() const
	{
		return CabActive == CabOccupied && CabMaster;
	} // czy aktualna kabina jest na pewno tą, z której można sterować
	bool AutomaticCabActivation = true; // czy zmostkowany rozrzad przelacza sie sam przy zmianie kabiny
	int InactiveCabFlag = 0; // co sie dzieje przy dezaktywacji kabiny
	bool InactiveCabPantsCheck = false; // niech DynamicObject sprawdzi pantografy
	double LastSwitchingTime = 0.0; /*czas ostatniego przelaczania czegos*/
	int WarningSignal = 0; // 0: nie trabi, 1,2,4: trabi
	bool DepartureSignal = false; /*sygnal odjazdu*/
	bool InsideConsist = false;
	/*-zmienne dla lokomotywy elektrycznej*/
	TTractionParam RunningTraction; /*parametry sieci trakcyjnej najblizej lokomotywy*/
	double enrot = 0.0; // ilosc obrotow silnika
	double Im = 0.0; // prad silnika
	/*
	    // currently not used
	    double IHeating = 0.0;
	    double ITraction = 0.0;
	*/
	double Itot = 0.0; // prad calkowity
	double TotalCurrent = 0.0;
	// momenty
	double Mm = 0.0;
	double Mw = 0.0;
	// sily napedne
	double Fw = 0.0;
	double Ft = 0.0;
	// Ra: Im jest ujemny, jeśli lok jedzie w stronę sprzęgu 1
	// a ujemne powinien być przy odwróconej polaryzacji sieci...
	// w wielu miejscach jest używane abs(Im)
	int Imin = 0;
	int Imax = 0; /*prad przelaczania automatycznego rozruchu, prad bezpiecznika*/
	double EngineVoltage = 0.0; // voltage supplied to engine
	int MainCtrlActualPos = 0; /*wskaznik RList*/
	int ScndCtrlActualPos = 0; /*wskaznik MotorParam*/
	bool DelayCtrlFlag = false; // czy czekanie na 1. pozycji na załączenie?
	double LastRelayTime = 0.0; /*czas ostatniego przelaczania stycznikow*/
	bool AutoRelayFlag = false; /*mozna zmieniac jesli AutoRelayType=2*/
	bool FuseFlag = false; /*!o bezpiecznik nadmiarowy*/
	bool ConvOvldFlag = false; /*!  nadmiarowy przetwornicy i ogrzewania*/
	bool GroundRelay{true}; // switches off to protect against damage from earths
	start_t GroundRelayStart{start_t::manual}; // relay activation method
	bool StLinFlag = false; /*!o styczniki liniowe*/
	bool StLinSwitchOff{false}; // state of the button forcing motor connectors open
	line_contactors_interlock LineContactorsInterlock{line_contactors_interlock::none}; // what else the line contactors need to close
	double LouvresOpenPressure{5.0}; // main reservoir pressure [bar] at which the pneumatic louvres of the motor fans open (ET22)
	bool ResistorsFlag = false; /*!o jazda rezystorowa*/
	double RventRot = 0.0; /*!s obroty wentylatorow rozruchowych*/
	double PantographVoltage{0.0}; // voltage supplied to pantographs
	double PantPress = 0.0; /*Cisnienie w zbiornikach pantografow*/
	bool PantPressSwitchActive{false}; // state of the pantograph pressure switch. gets primed at defined pressure level in pantograph air system
	bool PantPressLockActive{false}; // pwr system state flag. fires when pressure switch activates by pantograph pressure dropping below defined level
	// switches off if the power level drops below threshold, which opens the line breaker; with PantographsRequireMains
	// it opens the line contactors instead, and switches back on only with the master controller at zero or the first position
	bool NoVoltRelay{true};
	bool OvervoltageRelay{true}; // switches off if the power level goes above threshold
	bool s_CAtestebrake = false; // hunter-091012: zmienna dla testu ca
	std::array<std::pair<double, double>, 4> PowerCircuits; // 24v, 110v, 3x400v and 3000v power circuits, voltage from local sources and current draw pairs

	/*-zmienne dla lokomotywy spalinowej z przekladnia mechaniczna*/
	double dizel_fill = 0.0; /*napelnienie*/
	double dizel_engagestate = 0.0; /*sprzeglo skrzyni biegow: 0 - luz, 1 - wlaczone, 0.5 - wlaczone 50% (z poslizgiem)*/
	double dizel_engage = 0.0; /*sprzeglo skrzyni biegow: aktualny docisk*/
	double dizel_automaticgearstatus = 0.0; /*0 - bez zmiany, -1 zmiana na nizszy +1 zmiana na wyzszy*/
	bool dizel_startup{false}; // engine startup procedure request indicator
	bool dizel_ignition{false}; // engine ignition request indicator
	bool dizel_spinup{false}; // engine spin up to idle speed flag
	// the starter turns the engine a while before it fires: smoke comes out only after SmokeDelay (the Engine section of the vehicle's
	// .fiz, seconds from the start of the spin up) has passed. 0, the default, lets it out as soon as the starter turns, as before eu07/maszyna#523
	double dizel_SmokeDelay{0.0};
	double dizel_FiringCountdown{0.0}; // seconds left until the engine being started fires, 0 once it runs on its own
	double dizel_engagedeltaomega = 0.0; /*roznica predkosci katowych tarcz sprzegla*/
	double dizel_n_old = 0.0; /*poredkosc na potrzeby obliczen sprzegiel*/
	double dizel_Torque = 0.0; /*aktualny moment obrotowy silnika spalinowego*/
	double dizel_Power = 0.0; /*aktualna moc silnika spalinowego*/
	double dizel_nreg_min = 0.0; /*predkosc regulatora minimalna, zmienna w hydro*/
	double dizel_nreg_max = 0.0; /*predkosc regulatora maksymalna, zmienna w hydro*/

	/* - zmienne dla przetowrnika momentu */
	double hydro_TC_Fill = 0.0; /*napelnienie*/
	bool hydro_TC_Lockup = false; /*zapiecie sprzegla*/
	double hydro_TC_TorqueIn = 0.0; /*moment*/
	double hydro_TC_TorqueOut = 0.0; /*moment*/
	double hydro_TC_TMRatio = 1.0; /*aktualne przelozenie momentu*/
	double hydro_TC_Request = 0.0; /*zadanie sily*/
	double hydro_TC_nIn = 0.0; /*predkosc obrotowa walu wejsciowego*/
	double hydro_TC_nOut = 0.0; /*predkosc obrotowa walu wyjsciowego*/

	/* - zmienne dla przetowrnika momentu */
	double hydro_R_Fill = 0.0; /*napelnienie*/
	double hydro_R_Torque = 0.0; /*moment*/
	double hydro_R_Request = 0.0; /*zadanie sily hamowania*/
	double hydro_R_n = 0.0; /*predkosc obrotowa retardera*/
	bool hydro_R_ClutchActive = false; /*czy retarder jest napędzany*/

	/*- zmienne dla lokomotyw z silnikami indukcyjnymi -*/
	double eimic = 0; /*aktualna pozycja zintegrowanego sterowania jazda i hamowaniem*/
	double eimic_analog = 0; /*pozycja zadajnika analogowa*/
	double eimic_real = 0; /*faktycznie uzywana pozycja zintegrowanego sterowania jazda i hamowaniem*/
	double eimic_step_progress{0.0}; // the part of the next UniCtrlValStep the controller has covered so far
	double eim_localbrake = 0; /*nastawa hamowania dodatkowego pneumatycznego lokomotywy*/
	int EIMCtrlType = 0; /*rodzaj wariantu zadajnika jazdy*/
	bool SpeedCtrlTypeTime = false; /*czy tempomat sterowany czasowo*/
	int SpeedCtrlAutoTurnOffFlag = 0; /*czy tempomat sam się wyłącza*/
	bool EIMCtrlAdditionalZeros = false; /*czy ma dodatkowe zero jazdy i zero hamowania */
	bool EIMCtrlEmergency = false; /*czy ma dodatkowe zero jazdy i zero hamowania */
	bool EIMCtrlMainEmergency = false; /*czy zadajnik typu 2 (Elf) ma pozycje hamowania naglego ponizej swojej pozycji 0*/
	double eimv_pr = 0; /*realizowany procent dostepnej sily rozruchu/hamowania*/
	double eimv[21];
	static std::vector<std::string> const eimv_labels;
	double SpeedCtrlTimer = 0; /*zegar dzialania tempomatu z wybieralna predkoscia*/
	double eimicSpeedCtrl = 1; /*pozycja sugerowana przez tempomat*/
	double eimicSpeedCtrlIntegral = 0; /*calkowany blad ustawienia predkosci*/
	double NewSpeed = 0; /*nowa predkosc do zadania*/
	double MED_EPVC_CurrentTime = 0; /*aktualny czas licznika czasu korekcji siły EP*/
	double MED_ED_DelayTimer = 0; /*aktualny czas licznika opoznienia hamowania ED*/

	/*-zmienne dla drezyny*/
	double PulseForce = 0.0; /*przylozona sila*/
	double PulseForceTimer = 0.0;
	int PulseForceCount = 0;

	/*dla drezyny, silnika spalinowego i parowego*/
	double eAngle = M_PI * 0.5;

	/*-dla wagonow*/
	float LoadAmount = 0.f; /*masa w T lub ilosc w sztukach - zaladowane*/
	load_attributes LoadType;
	std::string LoadQuantity; // jednostki miary
	int LoadStatus = 0; //+1=trwa rozladunek,+2=trwa zaladunek,+4=zakończono,0=zaktualizowany model
	bool LoadTypeChange{false}; // indicates load type was changed
	double LastLoadChangeTime = 0.0; // raz (roz)ładowania
#ifdef EU07_USEOLDDOORCODE
	bool DoorBlocked = false; // Czy jest blokada drzwi
	bool DoorLockEnabled{true};
	bool DoorLeftOpened = false; // stan drzwi
	double DoorLeftOpenTimer{-1.0}; // left door closing timer for automatic door type
	bool DoorRightOpened = false;
	double DoorRightOpenTimer{-1.0}; // right door closing timer for automatic door type
#endif
	// TODO: move these switch types where they belong, cabin definition
	std::string PantSwitchType;
	std::string ConvSwitchType;
	bool HeatingSwitchImpulse{false}; // the train heating switch springs back to its middle position
	std::string StLinSwitchType;

	bool Heating = false; // ogrzewanie 'Winger 020304
	bool HeatingAllow{false}; // heating switch // TODO: wrap heating in a basic device
	int DoubleTr = 1; // trakcja ukrotniona - przedni pojazd 'Winger 160304
	basic_light CompartmentLights;

	bool PhysicActivation = true;

	/*ABu: stale dla wyznaczania sil (i nie tylko) po optymalizacji*/
	double FrictConst1 = 0.0;
	double FrictConst2s = 0.0;
	double FrictConst2d = 0.0;
	double TotalMassxg = 0.0; /*TotalMass*g*/

	double fBrakeCtrlPos = -2.0; // płynna nastawa hamulca zespolonego
	bool bPantKurek3 = true; // kurek trójdrogowy (pantografu): true=połączenie z ZG, false=połączenie z małą sprężarką // domyślnie zbiornik pantografu połączony jest ze zbiornikiem głównym
	bool PantAutoValve{false}; // type of installed pantograph compressor valve
	int iProblem = 0; // flagi problemów z taborem, aby AI nie musiało porównywać; 0=może jechać
	int iLights[2]; // bity zapalonych świateł tutaj, żeby dało się liczyć pobór prądu

	// Status nowszego hebelka od przyciemniania swiatel/swiatel dlugich
	// 0 - swiatla wylaczone (opcja dziala tylko gdy w fiz zdefiniowano OffState w sekcji Switches; w przeciwnym wypadku pstryk startuje z wartoscia == 1
	// 1 - swiatla normalne przyciemnione
	// 2 - swiatla normalne
	// 3 - swiatla dlugie przyciemnione
	// 4 - swiatla dlugie normalne

	

	// default positions
	std::vector<dimPosition> dimPositions = {{false, false, false}, // 0 - Not dimmed
	                                         {false, true, false}}; // 1 - Dimmed

	int modernDimmerPosition{0};
	int modernDimmerDefaultPosition{0};
	// bool modernContainOffPos{true};
	bool enableModernDimmer{false};
	bool modernDimmerCanCycle{false};
	// the given dimmer position brought into the DimmerList, to its first or last entry; 0 for an empty list
	int clampDimmerPosition(int Position) const;
	// the entry of the current dimmer position, or a plain one (not off, not dimmed, no high beam) when the
	// position is outside the DimmerList: the .fiz can give a short or an empty list, or a DefaultPos past its end
	dimPosition currentDimPosition() const;
	// moves the dimmer by Step positions within the DimmerList, around to its other end when it can cycle;
	// a position outside the list is brought into it first. false when the dimmer doesn't move
	bool stepDimmerPosition(int Step);

	// Barwa reflektora
	int refR{255}; // Czerwony
	int refG{255}; // Zielony
	int refB{255}; // Niebieski

	double dimMultiplier{0.6f}; // mnoznik swiatel przyciemnionych
	double normMultiplier{1.0f}; // mnoznik swiatel zwyklych
	double highDimMultiplier{2.5f}; // mnoznik dlugich przyciemnionych
	double highMultiplier{2.8f}; // mnoznik dlugich

	plc::basic_controller m_plc;

	int AIHintPantstate{0}; // suggested pantograph setup
	// suggested pantograph setup in shunting (AI: ShuntPantstate=, eu07/maszyna#461), the bits as in Pantstate: 1 front,
	// 2 rear, 3 both; 0 (the default) shunts with the pantographs handled as in train mode
	int AIHintShuntPantstate{0};
	bool AIHintPantUpIfIdle{true}; // whether raise both pantographs if idling for a while
	double AIHintLocalBrakeAccFactor{1.05}; // suggested acceleration weight for local brake operation

	TMoverParameters(double VelInitial, std::string const &TypeNameInit, std::string const &NameInit, int Cab);
	// obsługa sprzęgów
	static double CouplerDist(TMoverParameters const *Left, TMoverParameters const *Right);
	static double Distance(const TLocation &Loc1, const TLocation &Loc2, const TDimension &Dim1, const TDimension &Dim2);
	bool Attach(int ConnectNo, int ConnectToNr, TMoverParameters *ConnectTo, int CouplingType, bool Enforce = false, bool Audible = true);
	int DettachStatus(int ConnectNo);
	bool Dettach(int ConnectNo);
	void damage_coupler(int const End);
	// repairs the broken coupler of the end, and the one of the vehicle it was coupled with, so they can be coupled
	// again; the vehicles clear their coupler damage once neither of their couplers is broken. returns: whether there
	// was a broken coupler to repair
	bool repair_coupler(int const End);
	void Derail(DerailReason Reason);
	// the velocity of the vehicles coupled to this one changes by the Change along the axis of this one [m/s], as the
	// legacy derailment changes the velocity of this one (physics.collisions new)
	void change_train_velocity(double Change);
	// physics.derailment new: the criteria of derailment.h over the step, which travelled the Distance [m]; monitor: what
	// they would do, to the log
	void check_derailment(TTrackShape const &Shape, TTrackParam const &Track, double Distance);
	// the vehicle leaves the rails, as the criteria of derailment.h found
	void derail(derailment::verdict const &Verdict, derailment::vehicle_state const &Vehicle, derailment::track_state const &Track, std::string const *TrackName = nullptr);
	// height of the centre of gravity above the rail with the load [m]
	double centre_of_gravity() const;
	// the vehicle is back on the rails (resetconsist, SetDamage taking dtrain_out off): not derailed any more, and the
	// criteria of derailment start from scratch, so a later derailment goes the way its model has it
	void rerail();
	// what resetconsist does to a vehicle: no damage of any kind, the alarm chain released, back on the rails
	void repair_damage();
	// how the vehicle left the rails by the criteria of derailment.h: upright, or on its side. none for any other
	// derailment, which stops it as the legacy model does (with physics.collisions new by a friction of its weight, the
	// forces of the vehicles coupled to it acting on it)
	derailment::outcome Derailed{derailment::outcome::none};
	// physics.collisions new: what the vehicle is for the defaults of its crush and its buffers, the crush curve and the
	// height of the buffers of an end, and how long the push of its ends held its wheels over the climbing limit
	collision::vehicle_kind collision_kind() const;
	collision::crush_curve crush_curve_at(int End) const;
	double buffer_height_at(int End) const;
	collision::pushed_monitor pushed_off_monitor;
	// how the vehicle left the rails in a collision (physics.collisions new): overriding or pushed off, NONE otherwise
	DerailReason CollisionDerail{NONE};
	// physics.collisions monitor: how many crushes the log told of, of the hits of this vehicle on the others
	int crush_reports{0};
	// physics.collisions new: the hit of the end on the stop of its buffers, harder than the steel takes, crushes the
	// weaker end and may let one vehicle climb onto the other one, see collision.h. returns: whether it did, and the
	// velocities took the hit; the legacy collision takes the rest. physics.collisions monitor: the log tells what the
	// crush of the whole hit would be, once for a hit, nothing changes, and it returns false
	bool crush_hit(int End, double dt);
	// the force pressing the ends pushes the vehicle off the track (physics.collisions new), the step travelled the
	// Distance [m]: the flange climbs while the wheels roll on
	void check_pushed_off(double dt, double Distance);
	// the vehicle leaves the rails in a collision, the reason told in the log, and slides on as the criteria of
	// derailment.h have it
	void derail_in_collision(DerailReason Reason, std::string const &Why);
	bool DirectionForward();
	bool DirectionBackward(void); /*! kierunek ruchu*/
	bool EIMDirectionChangeAllow(void) const;
	inline double IsVehicleEIMBrakingFactor() const
	{
		return DynamicBrakeFlag && ResistorsFlag ? 0.0 : eimv[eimv_Ipoj] < 0 ? -1.0 : 1.0;
	}
	void BrakeLevelSet(double b);
	bool BrakeLevelAdd(double b);
	bool IncBrakeLevel(); // wersja na użytek AI
	bool DecBrakeLevel();
	bool ChangeCab(int direction);
	bool CurrentSwitch(bool const State);
	bool IsMotorOverloadRelayHighThresholdOn() const;
	void UpdateBatteryVoltage(double dt);
	double BatteryDrain(double dt, bool converteractive) const;
	double BatteryDrainCharged(double dt, bool converteractive) const;
	double BatteryDrainLow(double dt, bool converteractive) const;
	double BatteryDrainSwitchedOff(double dt) const;
	double ComputeMovement(double dt, double dt1, const TTrackShape &Shape, TTrackParam const &Track, TTractionParam const &ElectricTraction, TLocation const &NewLoc,
	                       TRotation const &NewRot); // oblicza przesuniecie pojazdu
	double FastComputeMovement(double dt, const TTrackShape &Shape, TTrackParam &Track, TLocation const &NewLoc, TRotation const &NewRot); // oblicza przesuniecie pojazdu - wersja zoptymalizowana
	void compute_movement_(double const Deltatime);
	double ShowEngineRotation(int VehN) const;

	// Q *******************************************************************************************
	double GetTrainsetVoltage(int const Coupling = coupling::heating | coupling::highvoltage) const;
	double GetTrainsetHighVoltage() const;
	bool switch_physics(bool const State);
	double LocalBrakeRatio(void) const;
	double ManualBrakeRatio(void) const;
	double PipeRatio(void) const; /*ile napelniac*/
	double RealPipeRatio(void) const; /*jak szybko*/
	double BrakeVP(void) const;
	double EngineRPMRatio() const; // returns current engine revolutions as percentage of max engine revolutions, in range 0-1
	double EngineIdleRPM() const;
	double EngineMaxRPM() const;

	/*! przesylanie komend sterujacych*/
	bool SendCtrlToNext(std::string const &CtrlCommand, double const ctrlvalue, double const dir, int const Couplertype = coupling::control);
	bool SetInternalCommand(std::string const &NewCommand, double NewValue1, double NewValue2, int const Couplertype = coupling::control);
	double GetExternalCommand(std::string &Command) const;
	bool RunCommand(std::string const &Command, double CValue1, double CValue2, int const Couplertype = coupling::control);
	// the parts of RunCommand(), by the area of the commands
	bool RunControllerCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunRelaysCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunCabCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunBrakeCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunBrakeEquipmentCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunAuxiliariesCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunPumpsCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunBlowersCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunPowerCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunDoorsCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	void DoorPermitCommand(double CValue1, double CValue2);
	void DoorRemoteCommand(bool Open, double CValue1, double CValue2);
	bool RunLoadCommand(std::string const &Command, double CValue1, double CValue2, int Couplertype, bool &OK);
	bool RunInternalCommand();
	void PutCommand(std::string const &NewCommand, double NewValue1, double NewValue2, const TLocation &NewLocation);
	bool CabActivisation(bool const Enforce = false);
	bool CabDeactivisation(bool const Enforce = false);
	bool CabActivisationAuto(bool const Enforce = false);
	bool CabDeactivisationAuto(bool const Enforce = false);

	/*! funkcje zwiekszajace/zmniejszajace nastawniki*/
	/*! glowny nastawnik:*/
	bool IncMainCtrl(int CtrlSpeed);
	bool DecMainCtrl(int CtrlSpeed);
	// the parts of IncMainCtrl() and DecMainCtrl() which depend on the kind of the engine
	bool IncMainCtrlByEngine(int CtrlSpeed);
	bool IncMainCtrlSeriesMotor(int CtrlSpeed);
	bool DecMainCtrlByEngine(int CtrlSpeed);
	bool DecMainCtrlSeriesMotor(int CtrlSpeed);
	bool IsMainCtrlActualNoPowerPos() const; // whether the master controller is actually set to position which won't generate any extra power
	bool IsMainCtrlNoPowerPos() const; // whether the master controller is set to position which won't generate any extra power
	bool IsMainCtrlMaxPowerPos() const;
	int MainCtrlNoPowerPos() const; // highest setting of master controller which won't cause engine to generate extra power
	int MainCtrlActualPowerPos() const; // current actual setting of master controller, relative to the highest setting not generating extra power
	int MainCtrlPowerPos() const; // current setting of master controller, relative to the highest setting not generating extra power
	// positions the master controller of the EIMCtrlType 2 (Elf) has below its own position 0, the emergency brake (0 or 1)
	int EIMCtrlEmergencyPositions() const;
	// whether the master controller is in its emergency brake position (EIMCtrlType 2 with EIMCtrlMainEmergency)
	bool IsMainCtrlEmergencyPos() const;
	// the master controller position sent to the coupled vehicles ("MainCtrl"), the same with and without
	// EIMCtrlMainEmergency: the Elf's positions without the emergency brake one, which goes as the full braking
	int MainCtrlPosForCoupled() const;
	// the master controller position of this vehicle for one received from a coupled vehicle ("MainCtrl")
	int MainCtrlPosFromCoupled(int Position) const;
	/*! pomocniczy nastawnik:*/
	bool IncScndCtrl(int CtrlSpeed);
	bool DecScndCtrl(int CtrlSpeed);
	int GetVirtualScndPos() const;
	// whether ScndCtrlResistiveLock keeps the shunt controller from being moved at the current master controller position
	bool IsScndCtrlLockedByMainCtrl() const;
	// ScndCtrlResistiveLock: the shunt controller drops to 0 once the master controller is at zero or on a resistive position, so it
	// can't stay engaged there, locked in place (eu07/maszyna#402)
	void ScndCtrlResistiveLockCheck();
	// NOTE: with the cruise control, either end of the secondary controller, as before the split of the two (eu07/maszyna#69)
	bool IsScndCtrlNoPowerPos() const;
	bool IsScndCtrlMaxPowerPos() const;

	bool AddPulseForce(int Multipler); /*dla drezyny*/

	bool SandboxManual(bool const State, range_t const Notify = range_t::consist); /*wlacza/wylacza reczne sypanie piasku*/
	bool SandboxAuto(bool const State, range_t const Notify = range_t::consist); /*wlacza/wylacza automatyczne sypanie piasku*/
	bool Sandbox(bool const State, range_t const Notify = range_t::consist); /*wlacza/wylacza sypanie piasku*/
	bool SandboxAutoAllow(bool const State); /*wlacza/wylacza zezwolenie na automatyczne sypanie piasku*/

	/*! zbijanie czuwaka/SHP*/
	void SecuritySystemReset(void);
	void SecuritySystemCheck(double dt);

	bool BatterySwitch(bool State, range_t const Notify = range_t::consist);
	bool EpFuseSwitch(bool State);
	bool SpringBrakeActivate(bool State);
	bool SpringBrakeShutOff(bool State);
	bool SpringBrakeRelease();
	bool PulleeModeSwitch(bool const State);

	/*! stopnie hamowania - hamulec zasadniczy*/
	bool IncBrakeLevelOld(void);
	bool DecBrakeLevelOld(void);
	bool IncLocalBrakeLevel(float const CtrlSpeed);
	bool DecLocalBrakeLevel(float const CtrlSpeed);
	/*! ABu 010205: - skrajne polozenia ham. pomocniczego*/
	bool IncManualBrakeLevel(int CtrlSpeed);
	bool DecManualBrakeLevel(int CtrlSpeed);
	/*! oddzielny nastawnik hamulca elektrodynamicznego (gdy SplitEDPneumaticBrake)*/
	bool IncDynamicBrakeLevel(float const CtrlSpeed);
	bool DecDynamicBrakeLevel(float const CtrlSpeed);
	bool DynamicBrakeLevelSet(double Position);
	double DynamicBrakeRatio(void) const;
	bool DynamicBrakeAvailable(void) const; /*czy ED dostepny w aktualnym oknie predkosci Vh0..Vh1*/
	bool DynamicBrakeSwitch(bool Switch);
	bool RadiostopSwitch(bool Switch);
	bool AlarmChainSwitch(bool const State);
	bool AntiSlippingBrake(void);
	bool BrakeReleaser(int state);
	bool UniversalBrakeButton(int button, int state); /*uniwersalny przycisk hamulca*/
	bool SwitchEPBrake(int state);
	// whether the EP brake control of the cab has power: battery, the reverser out of neutral and the EP fuse on
	bool EPBrakeControlPowered() const;
	bool AntiSlippingButton(void); /*! reczny wlacznik urzadzen antyposlizgowych*/

	/*funkcje dla ukladow pneumatycznych*/
	bool IncBrakePress(double &brake, double PressLimit, double dp);
	bool DecBrakePress(double &brake, double PressLimit, double dp);
	bool BrakeDelaySwitch(int BDS); /*! przelaczanie nastawy opoznienia*/
	bool IncBrakeMult(void); /*przelaczanie prozny/ladowny*/
	bool DecBrakeMult(void);
	/*pomocnicze funkcje dla ukladow pneumatycznych*/
	void UpdateBrakePressure(double dt);
	void UpdatePipePressure(double dt);
	// the parts of UpdatePipePressure()
	void UpdateLocalBrakeValve(double dt);
	void UpdateMainValve(double dt);
	double MainValveSupply();
	double MainValveHandlePosition() const;
	void UpdateEmergencyValve(double dt);
	void UpdateDistributorSettings();
	void UpdateEPBrake();
	void CompressorCheck(double dt); /*wlacza, wylacza kompresor, laduje zbiornik*/
	void UpdatePantVolume(double dt); // Ra
	void PantPressureSwitchCheck();
	void PantPressureSwitchTrip();
	void UpdateScndPipePressure(double dt);
	void UpdateSpringBrake(double dt);
	double GetDVc(double dt);
	int PipeCouplersSubsteps(double dt) const;
	double GetScndDVc(double dt);
	int ScndPipeCouplersSubsteps(double dt) const;
	// the air exchange through the brake and main reservoir hoses, after the vehicles stepped
	static void UpdatePipeCouplers(std::vector<TMoverParameters *> const &Vehicles, double dt);

	/*funkcje obliczajace sily*/
	void ComputeConstans(void); // ABu: wczesniejsze wyznaczenie stalych dla liczenia sil
	void ComputeMass(void);
	void ComputeTotalForce(double dt);
	double Adhesive(double staticfriction) const;
	double TractionForce(double dt);
	// the parts of TractionForce(), in the order it makes them
	double UpdateEngineRevolutions(double dt, double &dtrans, double &dmoment);
	double DieselElectricTargetRevolutions();
	void UpdateVentilators(double dt);
	void UpdateResistorBlowers(double dt);
	double UpdateEnginePower();
	void VoltageRelaysCheck();
	void DumbTractionForce(double PosRatio);
	void WheelsDrivenTractionForce(double dt);
	void SeriesMotorTractionForce(double dt);
	void SeriesMotorDynamicBrake(double dt);
	void DieselEngineTractionForce(double dt, double dmoment, double dtrans);
	void DieselElectricTractionForce(double tmp);
	void DieselElectricShuntForce(double tmpV, double &tmp);
	double DieselElectricDigitalForce(double tmpV, double &tempUmax, double &tempImax);
	double DieselElectricForce(double tmpV, double LinePower, double Ratio, double HyperbolaPower) const;
	void DieselElectricShuntPower(double tmp);
	void DieselElectricGenerator(double tmp, double tempUmax, double tempImax);
	void ShuntFieldRelaysCheck(double tmpV, double PosRatio, double tmp);
	void ShuntFieldRelaysType2(double tmp);
	void ShuntFieldRelaysType41(double tmpV);
	void ShuntFieldRelaysType45();
	void ShuntFieldRelaysType46();
	void ShuntFieldRelaysType46Up();
	void ShuntFieldRelaysType46Down();
	void InductionMotorTractionForce(double dt);
	void InductionMotorSpeedControl(double dt);
	void SpeedControlByTime(double dt);
	void SpeedControlByPosition(double dt);
	void InductionMotorBrakeBlending(double ActiveInverters, double &PosRatio, double &tmp);
	void InductionMotorDynamicBrakeFlag();
	double InductionMotorBrakeRatio(double ActiveInverters, double edBCP);
	void InductionMotorDrive(double dt);
	void InductionMotorForceLimits();
	void UpdateInverterFrequency();
	void InductionMotorOff();
	// the dynamic brake of a diesel-electric vehicle: the effort following the demand, the force it gives at the speed
	void update_dieselelectric_dynamic_brake(double dt);
	double dieselelectric_dynamic_brake_demand() const; // 0..1, the effort the driver asks for
	double dieselelectric_dynamic_brake_limit() const; // [N] the force at full effort at the speed
	void apply_dieselelectric_dynamic_brake(double dt); // the force in place of the drive's one, over the step dt
	double dieselelectric_motor_current(double Moment) const; // the current of the motors giving the moment
	double FrictionForce() const;
	double BrakeForceR(double ratio, double velocity) const;
	double BrakeForceP(double press, double velocity) const;
	double BrakeForce(const TTrackParam &Track);
	double CouplerForce(int const End, double dt);
	void CouplerStretchCheck(int End, double CF, double dt);
	// whether the coupling at the end is computed by the physical model: couplerphysics set in eu07.ini, and this
	// vehicle or the one at the end covered by it (CouplerPhysics), or every vehicle with couplerphysics all
	bool coupler_physics_at(int const End) const;
	// the force of the coupling at the end by the physical model (couplerphysics in eu07.ini), see couplerphysics.h
	double coupler_force_physical(int const End, double const dt);
	// with the physical model: whether the vehicles at the end hit the stop of their springs at the last update and
	// still close in on it, so that they collide; a hit hard enough breaks the coupling or crushes the buffers
	bool coupler_stop_hit(int const End, TMoverParameters &Other, int const OtherEnd);
	// couples the automatic couplers of the end and of the other vehicle pressed against each other, unless the hit
	// is too fast for them to catch
	void automatic_coupling(int const End, TMoverParameters &Other, int const OtherEnd);
	void CollisionDetect(int const End, double const dt);
	// closing speed of a collision [km/h] above which the vehicles involved may derail, adjusted for their mass ratio
	static constexpr double SafeCollisionVelocity{15.0};
	// closing speed [km/h] of the End of this vehicle and the OtherEnd of the other vehicle it touches, along the coupler
	// axis: the sum of the speeds in a head-on collision, the difference in a rear-end one, negative when they part
	double CollisionVelocity(int const End, TMoverParameters const &Other, int const OtherEnd) const;
	// velocity [m/s] of this vehicle relative to the other vehicle it touches with its End and the OtherEnd of the
	// other one, along this vehicle's axis
	double CouplerVelocityDifference(int const End, TMoverParameters const &Other, int const OtherEnd) const;
	// whether the automatic couplers at the End of this vehicle and the OtherEnd of the other one can catch each other on
	// contact, rather than the vehicles colliding (with crash damage enabled, only at a closing speed which doesn't derail them)
	bool AutomaticCouplingSafe(int const End, TMoverParameters const &Other, int const OtherEnd) const;
	// whether the control coupling (multiple-unit control) can join the End of this vehicle and the OtherEnd of the other
	// one: not between an electric and a diesel traction vehicle, unless both couplers name the same control system
	// (ControlType). Vehicles without own traction (cars, control trailers) take any. Global.control_coupling_engine_check
	// off allows it always (eu07/maszyna#520)
	bool ControlCouplingAllowed(int const End, TMoverParameters const &Other, int const OtherEnd) const;
	// the CouplingType to join the End of this vehicle and the OtherEnd of the other one with, without the control coupling
	// when ControlCouplingAllowed() rules it out. a permanent coupling (of the vehicles of one unit) is kept whole
	int CompatibleCoupling(int const End, TMoverParameters const &Other, int const OtherEnd, int const CouplingType) const;
	// the horns of the Signal (WarningSignal bits) which have the air or the voltage they need to sound
	int SoundingHorns(int const Signal) const;
	// whether the signal lamps of the main circuit (line contactors, ground relay, motor overload relays) can light:
	// the pressure switch, when fitted, cuts the start off and puts them out, otherwise they go out once the brakes are applied
	bool MainCircuitLampsEnabled() const
	{
		return false == ControlPressureSwitch && (HasControlPressureSwitch || BrakePress < 1.0);
	}
	// whether an emergency brake application holds the line breaker open: the brake pipe below the pressure taken for one
	// (EmergencyBrakeMainOffPressure) while the master controller is on a power position
	bool EmergencyBrakeHoldsMainOff() const
	{
		return EmergencyBrakeMainOffPressure > 0.0 && PipePress < EmergencyBrakeMainOffPressure && false == IsMainCtrlNoPowerPos();
	}
	// whether the lamp of the open line contactors is lit. on the electric locomotives it tells the start can be made, so
	// it goes out with the start cut off; on the diesel-electric ones (the SU45 and SU46 family) it just tells the
	// contactors are open, whatever opened them: the master controller at 0, or the pressure switch (eu07/maszyna#263)
	bool LineContactorsLampLit() const
	{
		if (EngineType == TEngineType::DieselElectric)
		{
			return false == StLinFlag;
		}
		return false == StLinFlag && MainCircuitLampsEnabled();
	}
	// the vehicles the line breaker / engine start switch of this vehicle works, and its engine stopping on its own
	// stops: the whole consist, or with JointEngineStart=No this vehicle alone (eu07/maszyna#80)
	range_t MainSwitchRange() const
	{
		return JointEngineStart ? range_t::consist : range_t::local;
	}
	// whether the pneumatic louvres of the motor fans are still closed, the main reservoir short of the pressure that opens them
	bool LouvresClosed() const
	{
		return Compressor < LouvresOpenPressure;
	}
	// whether the devices the line contactors are interlocked with (LineContactorsInterlock) let them close
	bool LineContactorsInterlockReleased() const;
	// whether the line contactors wait for the converter of the vehicle, switched off with its local switch (converterlocal_sw).
	// the AI driver doesn't leave it so: the converters of the other vehicles running, it would wait for the contactors forever
	bool LineContactorsHeldByLocalConverterSwitch() const
	{
		return LineContactorsInterlock == line_contactors_interlock::converter && false == ConverterAllowLocal;
	}
	// whether the vehicle has a driver's cab: it has the driver's brake valve (BrakeHandle in the Cntrl. section of the
	// .fiz), which the wagons and most cars of the units without a cab don't (a few copy the Cntrl. section of the cab
	// car, as the EN57 s car does). it's the vehicle type's, unlike the cab the scenario puts a driver in
	bool HasDriverCab() const
	{
		return BrakeHandle != TBrakeHandle::NoHandle;
	}
	// whether the cab command opening the main circuit opens it where the only cab switch of the circuit is the button
	// closing it: on a diesel that's the starter button, and the command is the one way to stop an engine whose fuel pump
	// starts on its own (eu07/maszyna#427). the other vehicles with such a button keep ignoring the command
	bool MainCircuitOpensWithStarterOnly() const
	{
		return EngineType == TEngineType::DieselEngine || EngineType == TEngineType::DieselElectric;
	}
	// whether the pumps a joint oil and fuel pump switch works, those not started automatically, are all switched on
	bool OilFuelPumpsEnabled() const
	{
		auto const fuelmanual{FuelPump.start_type != start_t::automatic};
		auto const oilmanual{OilPump.start_type != start_t::automatic};
		return (fuelmanual || oilmanual) && (false == fuelmanual || FuelPump.is_enabled) && (false == oilmanual || OilPump.is_enabled);
	}
	// whether the radio is switched on, its power supply isn't cut off and it has the low voltage to work
	bool RadioPowered() const
	{
		return Radio && RadioFuse && (Power24vIsAvailable || Power110vIsAvailable);
	}
	/*obrot kol uwzgledniajacy poslizg*/
	double ComputeRotatingWheel(double WForce, double dt, double n) const;

	/*--funkcje dla lokomotyw*/
	bool WaterPumpBreakerSwitch(bool State, range_t const Notify = range_t::consist); // water pump breaker state toggle
	bool WaterPumpSwitch(bool State, range_t const Notify = range_t::consist); // water pump state toggle
	bool WaterPumpSwitchOff(bool State, range_t const Notify = range_t::consist); // water pump state toggle
	bool WaterHeaterBreakerSwitch(bool State, range_t const Notify = range_t::consist); // water heater breaker state toggle
	bool WaterHeaterSwitch(bool State, range_t const Notify = range_t::consist); // water heater state toggle
	bool WaterCircuitsLinkSwitch(bool State, range_t const Notify = range_t::consist); // water circuits link state toggle
	bool FuelPumpSwitch(bool State, range_t const Notify = range_t::consist); // fuel pump state toggle
	bool FuelPumpSwitchOff(bool State, range_t const Notify = range_t::consist); // fuel pump state toggle
	bool OilPumpSwitch(bool State, range_t const Notify = range_t::consist); // oil pump state toggle
	bool OilPumpSwitchOff(bool State, range_t const Notify = range_t::consist); // oil pump state toggle
	bool MotorBlowersSwitch(bool State, end const Side, range_t const Notify = range_t::consist); // traction motor fan state toggle
	bool MotorBlowersSwitchOff(bool State, end const Side, range_t const Notify = range_t::consist); // traction motor fan state toggle
	bool CompartmentLightsSwitch(bool State, range_t const Notify = range_t::consist); // compartment lights state toggle
	bool CompartmentLightsSwitchOff(bool State, range_t const Notify = range_t::consist); // compartment lights state toggle
	bool MainSwitch(bool const State, range_t const Notify = range_t::consist); /*! wylacznik glowny*/
	void MainSwitch_(bool const State);
	bool MainSwitchCheck() const; // checks conditions for closing the line breaker
	bool ConverterSwitch(bool State, range_t const Notify = range_t::consist); /*! wl/wyl przetwornicy*/
	bool CompressorSwitch(bool State, range_t const Notify = range_t::consist); /*! wl/wyl sprezarki*/
	bool ChangeCompressorPreset(int const Change, range_t const Notify = range_t::consist);
	bool HeatingSwitch(bool const State, range_t const Notify = range_t::consist);
	void HeatingSwitch_(bool const State);
	double EnginePowerSourceVoltage() const; // returns voltage of defined main engine power source
	// whether the main circuit feeds what runs on it (the converter, a compressor, the heating, the motor and resistor
	// blowers, the traction motors): the line breaker closed, and where it closes ahead of the pantographs and holds
	// without traction voltage (PantographsRequireMains) also the voltage there, at least the one the undervoltage
	// trip takes. everywhere else it's the same as Mains
	bool MainCircuitPowered() const;

	/*-funkcje typowe dla lokomotywy elektrycznej*/
	void LowVoltagePowerCheck(double const Deltatime);
	void MainsCheck(double const Deltatime);
	void PowerCouplersCheck(double const Deltatime, coupling const Coupling);
	// the parts of PowerCouplersCheck()
	double LocalPowerVoltage(coupling Coupling);
	void PowerCouplersVoltage(coupling Coupling, double abslocalvoltage, bool localpowersource);
	void PowerCouplersCurrent(coupling Coupling, bool localpowersource);
	double *PowerCircuitCurrent(coupling Coupling);
	void ConverterCheck(double const Timestep); // przetwornica
	void HeatingCheck(double const Timestep);
	void WaterPumpCheck(double const Timestep);
	void WaterHeaterCheck(double const Timestep);
	void FuelPumpCheck(double const Timestep);
	void OilPumpCheck(double const Timestep);
	void MotorBlowersCheck(double const Timestep);
	// whether the circuit a traction motor blower runs off has power: the battery, the converter or the main circuit, by its start type
	bool MotorBlowerPowerIsAvailable(cooling_fan const &Blower) const;
	void PantographsCheck(double const Timestep);
	void LightsCheck(double const Timestep);
	bool FuseOn(range_t const Notify = range_t::consist); // bezpiecznik nadamiary
	bool FuseFlagCheck(void) const; // sprawdzanie flagi nadmiarowego
	void FuseOff(void); // wylaczenie nadmiarowego
	bool UniversalResetButton(int const Button, range_t const Notify = range_t::consist);
	bool RelayReset(int const Relays, range_t const Notify = range_t::consist); // resets specified relays
	double ShowCurrent(int AmpN) const; // pokazuje bezwgl. wartosc pradu na wybranym amperomierzu
	double ShowCurrentP(int AmpN) const; // pokazuje bezwgl. wartosc pradu w wybranym pojezdzie                                                             //Q 20160722

	/*!o pokazuje bezwgl. wartosc obrotow na obrotomierzu jednego z 3 pojazdow*/
	/*function ShowEngineRotation(VehN:int): integer; //Ra 2014-06: przeniesione do C++*/
	/*funkcje uzalezniajace sile pociagowa od predkosci: v2n, n2r, current, momentum*/
	double v2n(void);
	double Current(double n, double U);
	// the parts of Current()
	void UpdateSeriesMotorDynamicBrakeFlag();
	double SeriesMotorCurrent(double n, double U, double R, double Mn) const;
	double SeriesMotorDrivingCurrent(double n, double U, double Rz, double Mn, int SP) const;
	void MotorFailuresCheck(double n);
	double Momentum(double I) const;
	double MomentumF(double I, double Iw, int SCP) const;

	bool CutOffEngine(void); // odlaczenie udszkodzonych silnikow
	/*funkcje automatycznego rozruchu np EN57*/
	bool MaxCurrentSwitch(bool State, range_t const Notify = range_t::consist); // przelacznik pradu wysokiego rozruchu
	bool MinCurrentSwitch(bool State); // przelacznik pradu automatycznego rozruchu
	bool AutoRelaySwitch(bool State); // przelacznik automatycznego rozruchu
	bool AutoRelayCheck(); // symulacja automatycznego rozruchu
	// the parts of AutoRelayCheck()
	bool AutoRelayShuntStep(bool ARFASI2);
	bool AutoRelayMainStep(bool ARFASI);
	bool AutoRelayMainUp(bool ARFASI);
	bool AutoRelayMainNotchUp();
	bool AutoRelayMainDown();
	bool AutoRelayLineContactorsStep(bool motorconnectorsoff);
	bool CamshaftRunDown();
	bool CamshaftRunDownEMU();
	bool MotorConnectorsCheck();
	bool ResistorsFlagCheck(void) const; // sprawdzenie kontrolki oporow rozruchowych NBMX

	bool OperatePantographsValve(operation_t const State, range_t const Notify = range_t::consist);
	bool OperatePantographValve(end const End, operation_t const State, range_t const Notify = range_t::consist);
	bool DropAllPantographs(bool const State, range_t const Notify = range_t::consist);

	void CheckEIMIC(double dt); // sprawdzenie i zmiana nastawy zintegrowanego nastawnika jazdy/hamowania
	// the parts of CheckEIMIC(), by the kind of the controller
	void EIMICFromController(double brakeDemand, double offset, double multiplier);
	void EIMICFromSteps(double dt);
	void EIMICFromFourPositions(double dt);
	void EIMICFromUniversalController(double dt, double brakeDemand);
	void UniversalControllerTrainBrake();
	void EIMICTraxxLocalBrake(double dt);
	void EIMICLimit();
	// whether the holding brake (StopBrakeDecc) lets go at the power demand (eimic, above 0 when power is demanded) of this step of
	// Dt seconds: once the demand has stayed above 0 for StopBrakeReleaseDelay. a demand of 0 or less starts the count again
	bool UpdateStopBrakeRelease(double Demand, double Dt);
	void CheckSpeedCtrl(double dt);
	void SpeedCtrlRegulator(double dt, double accfactor);
	// whether the cruise control has a switch of its own: the secondary controller doubles as its switch unless it
	// shunts the field of the traction motors already, or there's none (the 6Dg, the ST45; eu07/maszyna#69)
	bool IsSpeedCtrlSeparate() const;
	// whether the cruise control is switched on: by its own switch, or by the secondary controller off its zero position
	bool IsSpeedCtrlEngaged() const;
	// the driver's switch of the cruise control: on, to hold the speed of the moment, or off. its own switch, or the
	// secondary controller doubling as one (first position on, back to zero off); returns whether it did anything
	bool SpeedCtrlSwitch(bool State);
	void SpeedCtrlButton(int button);
	void SpeedCtrlInc();
	void SpeedCtrlDec();
	bool SpeedCtrlPowerInc();
	bool SpeedCtrlPowerDec();

	/*-funkcje typowe dla lokomotywy spalinowej z przekladnia mechaniczna*/
	bool dizel_EngageSwitch(double state);
	bool dizel_EngageChange(double dt);
	bool dizel_AutoGearCheck(void);
	bool dizel_GearShiftCheck(double VelUp, double VelDown);
	bool dizel_GearShiftRequest(double VelUp, double VelDown);
	void dizel_ClutchCheck();
	void dizel_ClutchByController();
	void dizel_ClutchByIntegratedController();
	double dizel_fillcheck(int mcp, double dt);
	double dizel_DigitalFill(double dt);
	double dizel_GovernorFill(int mcp, double realfill) const;
	double dizel_GovernorRevolutions(int mcp, double &realfill) const;
	double dizel_Momentum(double dizel_fill, double n, double dt);
	void hydro_TCFillCheck(double dt);
	void hydro_TCTorque();
	double dizel_MomentumRetarder(double n, double dt); // moment hamowania retardera
	void dizel_HeatSet(float const Value);
	void dizel_Heat(double const dt);
	bool dizel_StartupCheck();
	bool dizel_Update(double dt);

	/* funckje dla wagonow*/
	bool AssignLoad(std::string const &Name, float const Amount = 0.f);
	bool LoadingDone(double LSpeed, std::string_view Loadname);
	bool PermitDoors(side const Door, bool const State = true, range_t const Notify = range_t::consist);
	void PermitDoors_(side const Door, bool const State = true);
	bool ChangeDoorPermitPreset(int const Change, range_t const Notify = range_t::consist);
	bool PermitDoorStep(bool const State, range_t const Notify = range_t::consist);
	bool ChangeDoorControlMode(bool const State, range_t const Notify = range_t::consist);
	bool OperateDoors(side const Door, bool const State, range_t const Notify = range_t::consist);
	bool LockDoors(bool const State, range_t const Notify = range_t::consist);
	bool signal_departure(bool const State, range_t const Notify = range_t::consist); // toggles departure warning
	void update_doors(double const Deltatime); // door controller update
	door_closure_t door_closure() const; // state of the door closure signalling of this vehicle
	// whether the door closure lamp in the cab of this vehicle is lit: with DoorClosureSignal, for the given state of the
	// consist; without it, for the lock of the vehicle's own doors, as the lamp always worked
	bool door_closure_lamp_lit(door_closure_t const Consist, bool const BlinkLit) const;
	// the parts of update_doors()
	// not const: the door they change is one of Doors.instances
	void update_door_requests(basic_door &door, bool localopencontrol, bool remoteopencontrol, bool localclosecontrol, bool remoteclosecontrol);
	void move_door(basic_door &door, double Deltatime);
	bool door_open_request(basic_door const &door, bool localopencontrol, bool remoteopencontrol) const;
	bool door_close_request(basic_door const &door, bool localclosecontrol, bool remoteclosecontrol) const;
	bool doors_powered() const;
	bool door_step_folding(basic_door const &door) const;
	float door_open_duration(basic_door const &door, bool localopencontrol, bool remoteopencontrol) const;
	void update_door_timers(double Deltatime);

	/* funkcje dla samochodow*/
	bool ChangeOffsetH(double DeltaOffset);

	/*funkcje ladujace pliki opisujace pojazd*/
	bool LoadFIZ(std::string const &chkpath); // Q 20160717    bool LoadChkFile(std::string chkpath);
	bool LoadFIZ_Text(std::string const &Text); // the same, from text in the .fiz format
	bool CheckLocomotiveParameters(bool ReadyFlag, int Dir);
	// the parts of CheckLocomotiveParameters()
	void CreateBrakeValve();
	void CreateBrakeHandles();
	void CreateBrakePipes();
	double InitialScndPipePress(double HosePressure) const;
	void SetReadyToDepart();
	void SetBraked();
	std::string EngineDescription(int what) const;
	// the cab interior light the light preset at given position (1-based, as LightsPos) sets, if it sets one
	std::optional<lights_preset_cablight> LightsPresetCabLight(int Position) const;

  private:
	bool LoadFIZ_Parse(cParser &Parser); // reads the definition from either of the above
	// the parts of LoadFIZ_Parse(), for a line of the definition
	bool LoadFIZ_TableEnd(std::string const &inputline) const;
	bool LoadFIZ_Section(std::string const &inputline, std::map<std::string, std::string, std::less<>> &fizlines);
	bool LoadFIZ_TableSection(std::string const &inputline, std::map<std::string, std::string, std::less<>> &fizlines);
	void LoadFIZ_TableRow(std::string const &inputline);
	void LoadFIZ_Param(std::string const &line);
	void LoadFIZ_Load(std::string const &line);
	void LoadFIZ_Dimensions(std::string const &line);
	void LoadFIZ_Wheels(std::string const &line);
	void LoadFIZ_Brake(std::string const &line);
	void LoadFIZ_Doors(std::string const &line);
	void LoadFIZ_BuffCoupl(std::string const &line, int const Index);
	void LoadFIZ_TurboPos(std::string const &line);
	void LoadFIZ_Cntrl(std::string const &line);
	void LoadFIZ_CntrlBrakes(std::string const &line);
	void LoadFIZ_CntrlTrainBrake(std::string const &line);
	void LoadFIZ_CntrlAntiSlipBrake(std::string_view asb);
	void LoadFIZ_CntrlControllers(std::string const &line);
	void LoadFIZ_CntrlStarts(std::string const &line);
	void LoadFIZ_Blending(std::string const &line);
	void LoadFIZ_DCEMUED(std::string const &line);
	void LoadFIZ_SpringBrake(std::string const &line);
	void LoadFIZ_Light(std::string const &line);
	void LoadFIZ_Headlights(std::string const &Line);
	void LoadFIZ_Clima(std::string const &line);
	void LoadFIZ_Power(std::string const &Line);
	void LoadFIZ_SpeedControl(std::string const &Line);
	void LoadFIZ_Engine(std::string const &Input);
	void LoadFIZ_EngineDiesel(std::string const &Input);
	void LoadFIZ_EngineDieselElectric(std::string const &Input);
	void LoadFIZ_EngineInductionMotor(std::string const &Input);
	void LoadFIZ_EngineDieselShared(std::string const &Input);
	void LoadFIZ_Switches(std::string const &Input);
	void LoadFIZ_MotorParamTable(std::string const &Input);
	void LoadFIZ_Circuit(std::string const &Input);
	void LoadFIZ_AI(std::string const &Input);
	void LoadFIZ_RList(std::string const &Input);
	void LoadFIZ_UCList(std::string const &Input);
	void LoadFIZ_DieselElectricDynamicBrake(std::string const &Input);
	void LoadFIZ_DList(std::string const &Input);
	void LoadFIZ_FFList(std::string const &Input);
	void LoadFIZ_FFEDList(std::string const &Input);
	void LoadFIZ_WiperList(std::string const &Input);
	void LoadFIZ_LightsList(std::string const &Input);
	void LoadFIZ_DimmerList(std::string const &Input);
	void LoadFIZ_CompressorList(std::string const &Input);
	void LoadFIZ_PowerParamsDecode(TPowerParameters &Powerparameters, std::string const &Prefix, std::string const &Input);
	TPowerType LoadFIZ_PowerDecode(std::string const &Power) const;
	TPowerSource LoadFIZ_SourceDecode(std::string const &Source) const;
	TEngineType LoadFIZ_EngineDecode(std::string const &Engine) const;
	bool readMPT0(std::string const &line);
	bool readMPT(std::string const &line); // Q 20160717
	bool readMPTElectricSeries(std::string const &line);
	bool readMPTDieselElectric(std::string const &line);
	bool readMPTDieselEngine(std::string const &line);
	bool readBPT(/*int const ln,*/ std::string const &line); // Q 20160721
	bool readRList(std::string const &Input);
	bool readUCList(std::string const &Input);
	bool readDList(std::string const &line);
	bool readDMList(std::string const &line);
	bool readV2NMAXList(std::string const &line);
	bool readHTCList(std::string const &line);
	bool readPmaxList(std::string const &line);
	bool readFFList(std::string const &line);
	bool readFFEDList(std::string const &line);
	bool readWWList(std::string const &line);
	bool readWiperList(std::string const &line);
	bool readDimmerList(std::string const &line);
	bool readLightsList(std::string const &Input);
	bool readCompressorList(std::string const &Input);
	void BrakeValveDecode(std::string const &s); // Q 20160719
	void BrakeSubsystemDecode(); // Q 20160719
};

} // namespace mover
