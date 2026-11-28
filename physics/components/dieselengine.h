/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the diesel engine of a vehicle and its hydraulic transmission: their definition from the .fiz and their state.
// a vehicle has them whatever its drive; the drives of a diesel engine (DieselEngine, DieselElectric) use them

#pragma once

#include <map>

// the heat of a diesel engine: its cooling circuits, their fans and shutters, the temperatures
struct EngineHeat
{
	// input, state of relevant devices
	bool cooling{false}; // TODO: user controlled device, implement
	//    bool okienko { true }; // window in the engine compartment
	// system configuration
	bool auxiliary_water_circuit{false}; // cooling system has an extra water circuit
	double fan_speed{0.075}; // cooling fan rpm; either fraction of engine rpm, or absolute value if negative
	// heat exchange factors
	double kw{0.35};
	double kv{0.6};
	double kfe{1.0};
	double kfs{80.0};
	double kfo{25.0};
	double kfo2{25.0};
	// system parts
	struct fluid_circuit_t
	{

		struct circuit_config_t
		{
			float temp_min{-1}; // lowest accepted temperature
			float temp_max{-1}; // highest accepted temperature
			float temp_cooling{-1}; // active cooling activation point
			float temp_flow{-1}; // fluid flow activation point
			bool shutters{false}; // the radiator has shutters to assist the cooling
		};
		circuit_config_t config;
		bool is_cold{false}; // fluid is too cold
		bool is_warm{false}; // fluid is too hot
		bool is_hot{false}; // fluid temperature crossed cooling threshold
		bool is_flowing{false}; // fluid is being pushed through the circuit
	};
	fluid_circuit_t water;
	fluid_circuit_t water_aux;
	fluid_circuit_t oil;
	fluid_circuit_t engine;
	// output, state of affected devices
	bool PA{false}; // malfunction flag
	float rpmw{0.0}; // current main circuit fan revolutions
	float rpmwz{0.0}; // desired main circuit fan revolutions
	bool zaluzje1{false};
	float rpmw2{0.0}; // current auxiliary circuit fan revolutions
	float rpmwz2{0.0}; // desired auxiliary circuit fan revolutions
	bool zaluzje2{false};
	// output, temperatures
	float Te{15.0}; // ambient temperature TODO: get it from environment data
	// NOTE: by default the engine is initialized in warm, startup-ready state
	float Ts{50.0}; // engine temperature
	float To{45.0}; // oil temperature
	float Tsr{50.0}; // main circuit radiator temperature (?)
	float Twy{50.0}; // main circuit water temperature
	float Tsr2{40.0}; // secondary circuit radiator temperature (?)
	float Twy2{40.0}; // secondary circuit water temperature
	float temperatura1{50.0};
	float temperatura2{40.0};
	float powerfactor{1.0}; // coefficient of heat generation for engines other than su45
	// engine overheat threshold
	float engine_max_temp{-1}; // maximum acceptable engine temperature, triggers overheat lamp when exceeded
	bool engine_is_hot{false}; // engine temperature crossed cooling threshold
};

// the diesel engine, its governor, its clutch and its fuel (the dizel_ values before)
struct DieselEngine
{
	// the definition (the Engine section of the .fiz)
	double maxMoment = 1.0; // Mmax
	double maxMomentRevolutions = 1.0; // nMmax
	double maxRevolutionsMoment = 2.0; // Mnmax
	double maxRevolutions = 2.0; // nmax
	double nominalFill = 0.0;
	std::map<double, double> momentTable;
	std::map<double, double> speedMaxRevolutionsTable; // vel2nmax
	/*parametry aproksymacji silnika spalinowego*/
	double standingMoment = 0.0; /*moment oporow ruchu silnika dla enrot=0*/
	double cutoffRevolutions = 0.0; /*predkosc obrotowa zadzialania ogranicznika predkosci*/
	double minRevolutions = 0.0; /*najmniejsza dopuszczalna predkosc obrotowa*/
	double hydraulicMinRevolutions = 0.0; /*najmniejsza dopuszczalna predkosc obrotowa w czasie jazdy na hydro */
	double hydraulicMinRevolutionsFactor = 0.0; /*wspolczynnik wzrostu obrotow minimalnych hydro zaleznosci od zadanego procentu*/
	double retarderRevolutions = 0.0; /*obroty pracy podczas hamowania retarderem*/
	double governorAcceleration = 999.0; /*tempo zwiększania prędkości przez regulator dizla */
	double fullEngageMinVelocity = 0.0; /*najmniejsza predkosc przy jezdzie ze sprzeglem bez poslizgu*/
	double converterReleaseVelocity = 3.0; /*predkosc progowa rozlaczenia przetwornika momentu*/
	double shaftInertia = 1.0; /*moment bezwladnosci walu itp*/
	double revolutionsDecreaseRate{2.0};
	double nominalFuelConsumptionRate = 250.0; /*jednostkowe zużycie paliwa przy mocy nominalnej, wczytywane z fiz, g/kWh*/
	double fuelConsumption = 0.0; /*współczynnik zużycia paliwa przeliczony do jednostek maszynowych, l/obrót*/
	double clutchDiameter = 0.5;
	double clutchMaxForce = 6000.0;
	double clutchFriction = 0.5; /*parametry sprzegla*/
	// the starter turns the engine a while before it fires: smoke comes out only after SmokeDelay (the Engine section of the vehicle's
	// .fiz, seconds from the start of the spin up) has passed. 0, the default, lets it out as soon as the starter turns, as before eu07/maszyna#523
	double smokeDelay{0.0};

	// the state
	double fill = 0.0; /*napelnienie*/
	double clutchState = 0.0; /*sprzeglo skrzyni biegow: 0 - luz, 1 - wlaczone, 0.5 - wlaczone 50% (z poslizgiem)*/
	double clutchPressure = 0.0; /*sprzeglo skrzyni biegow: aktualny docisk*/
	double automaticGearShift = 0.0; /*0 - bez zmiany, -1 zmiana na nizszy +1 zmiana na wyzszy*/
	bool startupRequested{false}; // engine startup procedure request indicator
	bool ignitionRequested{false}; // engine ignition request indicator
	bool spinningUp{false}; // engine spin up to idle speed flag
	double firingCountdown{0.0}; // seconds left until the engine being started fires, 0 once it runs on its own
	double clutchSlip = 0.0; /*roznica predkosci katowych tarcz sprzegla*/
	double previousRevolutions = 0.0; /*poredkosc na potrzeby obliczen sprzegiel*/
	double torque = 0.0; /*aktualny moment obrotowy silnika spalinowego*/
	double power = 0.0; /*aktualna moc silnika spalinowego*/
	double governorMinRevolutions = 0.0; /*predkosc regulatora minimalna, zmienna w hydro*/
	double governorMaxRevolutions = 0.0; /*predkosc regulatora maksymalna, zmienna w hydro*/
	double fuelConsumptionActual = 0.0; /*chwilowe spalanie paliwa w l/h*/
	double fuelConsumedTotal = 0.0; /*ilość paliwa zużyta od początku symulacji, l*/
	EngineHeat heat;
};

// the hydraulic torque converter (the hydro_TC values before)
struct TorqueConverter
{
	// the definition
	bool present = false; /*obecnosc hydraulicznego przetwornika momentu*/
	double maxTorqueRatio = 2.0; /*maksymalne wzmocnienie momentu*/
	double couplingPoint = 0.85; /*wzgledna predkosc zasprzeglenia*/
	double lockupTorque = 3000.0; /*moment graniczny sprzegla blokujacego*/
	double lockupRate = 1.0; /*szybkosc zalaczania sprzegla blokujacego*/
	double unlockRate = 1.0; /*szybkosc rozlaczania sprzegla blokujacego*/
	double fillRateIncrease = 1.0; /*szybkosc napelniania sprzegla*/
	double fillRateDecrease = 1.0; /*szybkosc oprozniania sprzegla*/
	double torqueInIn = 4.5; /*stala momentu proporcjonalnego do kwadratu obrotow wejsciowych*/
	double torqueInOut = 0.0; /*stala momentu proporcjonalnego do roznica obrotow wejsciowych i wyjsciowych*/
	double torqueOutOut = 0.0; /*stala momentu proporcjonalnego do kwadratu obrotow wyjsciowych*/
	double lockupSpeed = 1.0; /*prog predkosci zalaczania sprzegla blokujacego*/
	double unlockSpeed = 1.0; /*prog predkosci rozlaczania sprzegla blokujacego*/
	std::map<double, double> table; /*tablica przetwornika momentu*/

	// the state
	double fill = 0.0; /*napelnienie*/
	bool lockedUp = false; /*zapiecie sprzegla*/
	double torqueIn = 0.0; /*moment*/
	double torqueOut = 0.0; /*moment*/
	double torqueRatio = 1.0; /*aktualne przelozenie momentu*/
	double request = 0.0; /*zadanie sily*/
	double revolutionsIn = 0.0; /*predkosc obrotowa walu wejsciowego*/
	double revolutionsOut = 0.0; /*predkosc obrotowa walu wyjsciowego*/
};

// the hydraulic retarder (the hydro_R values before)
struct Retarder
{
	// the definition
	bool present = false; /*obecnosc retardera*/
	int placement = 0; /*umiejscowienie retardera: 0 - za skrzynia biegow, 1 - miedzy przetwornikiem a biegami, 2 - przed skrzynia biegow */
	double torqueInIn = 1.0; /*stala momentu proporcjonalnego do kwadratu obrotow wejsciowych*/
	double maxTorque = 1.0; /*maksymalny moment retardera*/
	double maxPower = 1.0; /*maksymalna moc retardera - ogranicza moment*/
	double fillRateIncrease = 1.0; /*szybkosc napelniania sprzegla*/
	double fillRateDecrease = 1.0; /*szybkosc oprozniania sprzegla*/
	double minVelocity = 1.0; /*minimalna predkosc, przy ktorej retarder dziala*/
	double engageVelocity = 1.0; /*minimalna predkosc hamowania, przy ktorej sprzeglo jest wciaz wlaczone*/
	bool hasClutch = false; /*czy retarder ma rozłączalne sprzęgło*/
	double clutchSpeed = 10.0; /*szybkość narastania obrotów po włączeniu sprzęgła retardera*/
	bool withIndividual = false; /*czy dla autobusów jest to łączone*/

	// the state
	double fill = 0.0; /*napelnienie*/
	double torque = 0.0; /*moment*/
	double request = 0.0; /*zadanie sily hamowania*/
	double revolutions = 0.0; /*predkosc obrotowa retardera*/
	bool clutchActive = false; /*czy retarder jest napędzany*/
};
