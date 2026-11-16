/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/drivingaid.h"

#include <limits>

#include "utilities/utilities.h"

namespace driving_aid
{
namespace
{

void take_proximity_limit(speed_limit_inputs const &Inputs, int const Limit, speed_limit_ahead &Ahead)
{
	Ahead.limit = Limit;
	Ahead.distance = Inputs.proximity;
}

// the speed change ahead weighed against the speed after the end of the current limit: the lower of these two takes priority,
// otherwise the limit lasts at least until the current one ends
void weigh_proximity_limit(speed_limit_inputs const &Inputs, speed_limit_ahead &Ahead)
{
	auto const noactivespeedlimit{Inputs.last_distance < 0};
	auto const atproximity{min_speed(Inputs.schedule, static_cast<int>(Inputs.next_speed))};
	if (atproximity == Ahead.limit)
	{
		if (noactivespeedlimit)
			take_proximity_limit(Inputs, atproximity, Ahead);
		return;
	}
	if (atproximity < Ahead.limit)
	{
		// a limit ahead stricter than the current one is important enough to tell of; otherwise only if it lies after the end
		// of the current (lower) limit
		if (atproximity < Inputs.desired || Inputs.proximity > Ahead.distance)
			take_proximity_limit(Inputs, atproximity, Ahead);
		return;
	}
	// the proximity distance lies after the end of the current limit, told of only if there's no such limit
	if (noactivespeedlimit)
		take_proximity_limit(Inputs, atproximity, Ahead);
}

} // namespace

speed_limit_ahead next_speed_limit(speed_limit_inputs const &Inputs)
{
	auto const speedlimit{static_cast<int>(Inputs.desired)};
	speed_limit_ahead ahead{speedlimit, std::numeric_limits<double>::max()};
	// when the driver may not move, any next limit is irrelevant
	if (speedlimit == 0)
		return ahead;
	// first the speed change after the end of the current limit
	if (Inputs.last_distance > 0)
	{
		ahead.limit = min_speed(Inputs.schedule, static_cast<int>(Inputs.last_speed));
		ahead.distance = Inputs.last_distance;
	}
	weigh_proximity_limit(Inputs, ahead);
	// a current limit reaching beyond the line scanned: how long it is isn't known, nothing is told rather than something misleading
	if (ahead.distance >= Inputs.scan_range)
		ahead.limit = speedlimit;
	// nor is a 'limit' told of which is the vehicle ahead: the driver sets the proximity distance to its very distance then
	else if (Inputs.proximity == Inputs.obstacle) // NOSONAR: the same value copied, not the result of a calculation
		ahead.limit = speedlimit;
	return ahead;
}

} // namespace driving_aid
