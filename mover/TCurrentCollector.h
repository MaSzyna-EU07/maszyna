#pragma once

#include "mover/core.h"

namespace mover
{
struct TCurrentCollector
{
	/// <summary>Number of pantographs of this type.</summary>
	long CollectorsNo; // musi być tu, bo inaczej się kopie
	/// <summary>Minimum pantograph height [m] (currently unused).</summary>
	double MinH;
	/// <summary>Maximum pantograph height [m] (currently unused).</summary>
	double MaxH;
	/// <summary>Working width of the contact strip [m].</summary>
	double CSW; // szerokość części roboczej (styku) ślizgacza
	/// <summary>Minimum acceptable contact-line voltage [V].</summary>
	double MinV;
	/// <summary>Maximum acceptable contact-line voltage [V].</summary>
	double MaxV;
	/// <summary>True if an over-voltage protection relay is fitted.</summary>
	bool OVP; // czy jest przekaznik nadnapieciowy
	/// <summary>Minimum voltage required to engage the main switch [V].</summary>
	double InsetV; // minimalne napięcie wymagane do załączenia
	/// <summary>Minimum air pressure required to operate [bar].</summary>
	double MinPress; // minimalne ciśnienie do załączenia WS
	/// <summary>Maximum air pressure after the reductor [bar].</summary>
	double MaxPress; // maksymalne ciśnienie za reduktorem
	/// <summary>True if power output is faked (e.g. for AI vehicles without a model).</summary>
	bool FakePower;
	/// <summary>Physical layout id (orientation / side).</summary>
	int PhysicalLayout;
	/// <summary>Pantograph type (model).</summary>
	TPantType PantographType;
	// inline TCurrentCollector() {
	//     CollectorsNo = 0;
	//     MinH, MaxH, CSW, MinV, MaxV = 0.0;
	//     OVP, InsetV, MinPress, MaxPress = 0.0;
	// }
};

} // namespace mover
