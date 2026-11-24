#pragma once

#include "core.h"
#include "TCurrentCollector.h"
#include "engine_generator.h"
#include "TAccumulator.h"
#include "TPowerCable.h"
#include "THeater.h"
#include "TTransducer.h"

namespace mover
{
struct TPowerParameters
{
	/// <summary>Maximum source voltage [V].</summary>
	double MaxVoltage{0.0};
	/// <summary>Maximum source current [A].</summary>
	double MaxCurrent{0.0};
	/// <summary>Internal resistance [Ω].</summary>
	double IntR{0.001};
	/// <summary>Type of source (selects which union member is meaningful).</summary>
	TPowerSource SourceType;
	union
	{
		struct
		{
			THeater RHeater;
		};
		struct
		{
			TPowerCable RPowerCable;
		};
		struct
		{
			TCurrentCollector CollectorParameters;
		};
		struct
		{
			TAccumulator RAccumulator;
		};
		struct
		{
			engine_generator EngineGenerator;
		};
		struct
		{
			TTransducer Transducer;
		};
		struct
		{
			TPowerType PowerType;
		};
	};
	inline TPowerParameters()
	{
		SourceType = TPowerSource::NotDefined;
		PowerType = TPowerType::NoPower;
		RPowerCable.PowerTrans = TPowerType::NoPower;
	}
};

} // namespace mover
