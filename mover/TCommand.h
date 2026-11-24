#pragma once

#include "mover/core.h"
#include "mover/TLocation.h"

namespace mover
{
struct TCommand
{
	/// <summary>Command name.</summary>
	std::string Command; /*komenda*/
	/// <summary>First numeric argument.</summary>
	double Value1 = 0.0; /*argumenty komendy*/
	/// <summary>Second numeric argument.</summary>
	double Value2 = 0.0;
	/// <summary>Coupling flag controlling how the command propagates between vehicles.</summary>
	int Coupling{coupling::control}; // coupler flag used to determine command propagation
	/// <summary>World-space location associated with the command.</summary>
	TLocation Location;
};

} // namespace mover
