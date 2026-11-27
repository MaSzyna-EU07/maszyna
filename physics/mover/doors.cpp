/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the doors and the exchange of the load

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
#include "physics/mover/internal.h"


bool TMoverParameters::AssignLoad(std::string const &Loadname, float const Amount)
{

	if (Loadname == "pantstate")
	{
		if (EnginePowerSource.SourceType == TPowerSource::CurrentCollector)
		{
			// wartość niby "pantstate" - nazwa dla formalności, ważna jest ilość
			auto const pantographsetup{static_cast<int>(Amount)};
			if (pantographsetup & 1 << 2)
			{
				DoubleTr = -1;
			}
			if (pantographsetup & 1 << 0)
			{
				if (DoubleTr == 1)
				{
					OperatePantographValve(end::front, operation_t::enable, range_t::local);
				}
				else
				{
					OperatePantographValve(end::rear, operation_t::enable, range_t::local);
				}
			}
			if (pantographsetup & 1 << 1)
			{
				if (DoubleTr == 1)
				{
					OperatePantographValve(end::rear, operation_t::enable, range_t::local);
				}
				else
				{
					OperatePantographValve(end::front, operation_t::enable, range_t::local);
				}
			}
			return true;
		}
		else
		{
			return false;
		}
	}

	if (Loadname.empty())
	{
		// empty the vehicle if requested
		LoadTypeChange = LoadType.name != Loadname;
		LoadType = load_attributes();
		LoadAmount = 0.f;
		return true;
	}
	// can't mix load types, at least for the time being
	if (LoadAmount > 0 && LoadType.name != Loadname)
	{
		return false;
	}

	auto const loadattributes = std::ranges::find_if(LoadAttributes, [&](auto const &attributes) { return attributes.name == Loadname; });
	if (loadattributes == std::end(LoadAttributes))
	{
		// didn't find matching load configuration, this type is unsupported
		return false;
	}
	LoadTypeChange = LoadType.name != Loadname;
	LoadType = *loadattributes;
	LoadAmount = safe_clamp(Amount, 0.f, MaxLoad);
	ComputeMass();
	return true;
}

// *************************************************************************************************
// Q: 20160713
// Test zakończenia załadunku / rozładunku
// *************************************************************************************************
bool TMoverParameters::LoadingDone(double const LSpeed, std::string_view Loadname)
{

	if (LSpeed == 0.0)
	{
		// zerowa prędkość zmiany, to koniec
		LoadStatus = 4;
		return true;
	}

	if (Loadname.empty())
	{
		return LoadStatus >= 4;
	}
	if (Loadname != LoadType.name)
	{
		return LoadStatus >= 4;
	}

	// test zakończenia załadunku/rozładunku
	// load exchange speed is reduced if the wagon is overloaded
	auto const loadchange{static_cast<float>(std::abs(LSpeed * LastLoadChangeTime) * (LoadAmount > MaxLoad ? 0.5 : 1.0))};

	if (LSpeed < 0)
	{
		// gdy rozładunek
		LoadStatus = 2; // trwa rozładunek (włączenie naliczania czasu)
		if (loadchange > 0) // jeśli coś przeładowano
		{
			LastLoadChangeTime = 0; // naliczony czas został zużyty
			LoadAmount -= loadchange; // zmniejszenie ilości ładunku
			CommandIn.Value1 -= loadchange; // zmniejszenie ilości do rozładowania
			if (LoadAmount <= 0 || CommandIn.Value1 <= 0)
			{
				// pusto lub rozładowano żądaną ilość
				LoadStatus = 4; // skończony rozładunek
				LoadAmount = safe_clamp(LoadAmount, 0.f, MaxLoad); // ładunek nie może być ujemny
			}
			if (LoadAmount == 0.f)
			{
				AssignLoad(""); // jak nic nie ma, to nie ma też nazwy
			}
			ComputeMass();
		}
	}
	else if (LSpeed > 0)
	{
		// gdy załadunek
		LoadStatus = 1; // trwa załadunek (włączenie naliczania czasu)
		if (loadchange > 0) // jeśli coś przeładowano
		{
			LastLoadChangeTime = 0; // naliczony czas został zużyty
			LoadAmount += loadchange; // zwiększenie ładunku
			CommandIn.Value1 -= loadchange;
			if (LoadAmount >= MaxLoad * (1.0 + OverLoadFactor) || CommandIn.Value1 <= 0)
			{
				LoadStatus = 4; // skończony załadunek
				LoadAmount = std::min<float>(static_cast<float>(MaxLoad * (1.0 + OverLoadFactor)), LoadAmount);
			}
			ComputeMass();
		}
	}

	return LoadStatus >= 4;
}

bool TMoverParameters::ChangeDoorPermitPreset(int const Change, range_t const Notify)
{

	auto const initialstate{Doors.permit_preset};

	if (false == Doors.permit_presets.empty())
	{

		Doors.permit_preset = safe_clamp(Doors.permit_preset + Change, 0, static_cast<int>(Doors.permit_presets.size() - 1));
		auto const doors{Doors.permit_presets[Doors.permit_preset]};
		auto const permitleft{((doors & 1) != 0)};
		auto const permitright{((doors & 2) != 0)};

		PermitDoors(CabActive > 0 ? side::left : side::right, permitleft, Notify);
		PermitDoors(CabActive > 0 ? side::right : side::left, permitright, Notify);
	}

	return Doors.permit_preset != initialstate;
}

bool TMoverParameters::PermitDoorStep(bool const State, range_t const Notify)
{

	auto const initialstate{Doors.step_enabled};

	Doors.step_enabled = State;
	if (Notify != range_t::local)
	{
		// wysłanie wyłączenia do pozostałych?
		SendCtrlToNext(VehicleCommand::doorStep, State == true ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return Doors.step_enabled != initialstate;
}

bool TMoverParameters::PermitDoors(side const Door, bool const State, range_t const Notify)
{

	bool const initialstate{Doors.instances[Door].open_permit};

	PermitDoors_(Door, State);

	if (Notify != range_t::local)
	{

		SendCtrlToNext(VehicleCommand::doorPermit,
		               (State ? 1 : -1) // positive: grant, negative: revoke
		                   * (Door == (CabActive > 0 ? side::left : side::right) ? // 1=lewe, 2=prawe (swap if reversed)
		                          1 :
		                          2),
		               CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return Doors.instances[Door].open_permit != initialstate;
}

void TMoverParameters::PermitDoors_(side const Door, bool const State)
{

	if (State && State != Doors.instances[Door].open_permit)
	{
		SetFlag(SoundFlag, sound::doorpermit);
	}
	Doors.instances[Door].open_permit = State;
}

bool TMoverParameters::ChangeDoorControlMode(bool const State, range_t const Notify)
{

	auto const initialstate{Doors.remote_only};

	Doors.remote_only = State;
	if (Notify != range_t::local)
	{
		// wysłanie wyłączenia do pozostałych?
		SendCtrlToNext(VehicleCommand::doorMode, State == true ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	if (true == State)
	{
		// when door are put in remote control mode they're automatically open
		// TBD, TODO: make it dependant on config switch?
		OperateDoors(side::left, true);
		OperateDoors(side::right, true);
	}

	return Doors.remote_only != initialstate;
}

bool TMoverParameters::OperateDoors(side const Door, bool const State, range_t const Notify)
{

	auto &door{Doors.instances[Door]};
	/*
	    if( ( State == true ? door.is_open : door.is_closed ) ) {
	        // TBD: should the command be passed to other vehicles regardless of whether it affected the primary target?
	        // (for the time being no, methods are often invoked blindly which would lead to commands spam)
	        return false;
	    }
	*/
	bool result{false};

	if (Notify == range_t::local)
	{
		door.local_open = State;
		door.local_close = false == State;
		result = true;
	}
	else
	{
		// remote door operation signals require power to propagate
		if (Power24vIsAvailable || Power110vIsAvailable)
		{
			door.remote_open = State;
			door.remote_close = false == State;
			result = true;
		}
	}

	if (Notify != range_t::local)
	{

		SendCtrlToNext(State == true ? VehicleCommand::doorOpen : VehicleCommand::doorClose,
		               Door == (CabActive > 0 ? side::left : side::right) ? // 1=lewe, 2=prawe (swap if reversed)
		                   1 :
		                   2,
		               CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return result;
}

// toggle door lock
bool TMoverParameters::LockDoors(bool const State, range_t const Notify)
{

	auto const initialstate{Doors.lock_enabled};

	Doors.lock_enabled = State;
	if (Notify != range_t::local)
	{
		// wysłanie wyłączenia do pozostałych?
		SendCtrlToNext(VehicleCommand::doorLock, State == true ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return Doors.lock_enabled != initialstate;
}

// toggles departure warning
bool TMoverParameters::signal_departure(bool const State, range_t const Notify)
{

	if (DepartureSignal == State)
	{
		// TBD: should the command be passed to other vehicles regardless of whether it affected the primary target?
		return false;
	}

	DepartureSignal = State;
	if (Notify != range_t::local)
	{
		// wysłanie wyłączenia do pozostałych?
		SendCtrlToNext(VehicleCommand::departureSignal, State == true ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return true;
}

// state of the door closure signalling (SOD) of this vehicle: open while a door isn't fully closed, released while
// the closed doors can be opened (a permit given, or, for doors that need none, no lock engaged), locked otherwise.
// doors the crew doesn't close (autonomous) aren't signalled, the same as for the door lights of the consist
door_closure_t TMoverParameters::door_closure() const
{
	if (Doors.range == 0.f || Doors.close_control == control_t::autonomous)
	{
		return door_closure_t::none;
	}
	if (std::ranges::any_of(Doors.instances, [](basic_door const &Door) { return false == Door.is_closed; }))
	{
		return door_closure_t::open;
	}
	if (Doors.is_locked)
	{
		return door_closure_t::locked;
	}
	if (Doors.permit_needed && std::ranges::none_of(Doors.instances, [](basic_door const &Door) { return Door.open_permit; }))
	{
		return door_closure_t::locked;
	}
	return door_closure_t::released;
}

bool TMoverParameters::door_closure_lamp_lit(door_closure_t const Consist, bool const BlinkLit) const
{
	return Doors.closure_signal ? door_closure_lamp(Consist, BlinkLit) : Doors.is_locked;
}

bool door_closure_lamp(door_closure_t const State, bool const BlinkLit)
{
	switch (State)
	{
	case door_closure_t::locked:
		return true;
	case door_closure_t::released:
		return BlinkLit;
	default:
		return false;
	}
}

// automatic door controller update
void TMoverParameters::update_doors(double const Deltatime)
{

	if (Doors.range == 0.f)
	{
		return;
	} // HACK: crude way to distinguish vehicles with actual doors

	// NBMX Obsluga drzwi, MC: zuniwersalnione
	auto const localopencontrol{false == Doors.remote_only && (Doors.open_control == control_t::passenger || Doors.open_control == control_t::mixed)};
	auto const remoteopencontrol{Doors.open_control == control_t::driver || Doors.open_control == control_t::conductor || Doors.open_control == control_t::mixed};
	auto const localclosecontrol{false == Doors.remote_only && (Doors.close_control == control_t::passenger || Doors.close_control == control_t::mixed)};
	auto const remoteclosecontrol{Doors.close_control == control_t::driver || Doors.close_control == control_t::conductor || Doors.close_control == control_t::mixed};

	Doors.is_locked = true == Doors.has_lock && true == Doors.lock_enabled && Vel >= Doors.doorLockSpeed;

	for (auto &door : Doors.instances)
	{
		update_door_requests(door, localopencontrol, remoteopencontrol, localclosecontrol, remoteclosecontrol);
		move_door(door, Deltatime);
	}

	if (false == Doors.instances[side::right].is_open && false == Doors.instances[side::left].is_open)
	{
		return;
	}

	if (Doors.auto_duration > 0.f)
	{
		update_door_timers(Deltatime);
	}
	/*
	    // the door are closed if their timer goes below 0, or if the vehicle is moving faster than defined threshold
	    std::array<side, 2> const doorids { side::right, side::left };
	    for( auto const doorid : doorids ) {
	        auto const &door { Doors.instances[ doorid ] };
	        if( true == door.is_open ) {
	            if( ( ( Doors.auto_velocity != -1.f ) && ( Vel > Doors.auto_velocity ) )
	             || ( ( door.auto_timer != -1.f ) && ( door.auto_timer <= 0.f ) ) ) {
	                // close the door and set the timer to expired state (closing may happen sooner if vehicle starts moving)
	                OperateDoors( doorid, false, range_t::local );
	            }
	        }
	    }
	*/
}

// the permit, the state and the requests of a door, and whether it and its step open or close
void TMoverParameters::update_door_requests(basic_door &door, bool const localopencontrol, bool const remoteopencontrol, bool const localclosecontrol, bool const remoteclosecontrol)
{
	// revoke permit if...
	door.open_permit = true == door.open_permit // ...we already have one...
	                   && (false == Doors.permit_presets.empty() // ...there's no preset switch controlling permit state...
	                       || (false == Doors.is_locked // ...and the door lock is engaged...
	                           && false == door.remote_close)); // ...or the door is about to be closed

	door.is_open = door.position >= Doors.range && (false == Doors.step_enabled || door.step_position >= (Doors.step_range != 0.f ? 1.f : 0.f));
	door.is_closed = door.position <= 0.f && door.step_position <= 0.f;
	door.is_door_closed = door.position <= 0.f;

	door.local_open = door.local_open && false == door.is_open && (false == Doors.permit_needed || door.open_permit);
	door.remote_open = (door.remote_open || Doors.remote_only) && false == door.is_open && (false == Doors.permit_needed || door.open_permit);
	door.local_close = door.local_close && false == door.is_closed && (false == remoteopencontrol || false == door.remote_open);
	door.remote_close = door.remote_close && false == door.is_closed && (false == localopencontrol || false == door.local_open);

	auto const openrequest{door_open_request(door, localopencontrol, remoteopencontrol)};
	auto const closerequest{door_close_request(door, localclosecontrol, remoteclosecontrol)};
	auto const ispowered{doors_powered()};

	door.is_opening = false == door.is_open && true == ispowered && false == closerequest && (true == door.is_opening || (true == openrequest && false == Doors.is_locked));
	door.is_closing = false == door.is_closed && true == ispowered && false == openrequest && (door.is_closing || closerequest);
	door.step_unfolding = Doors.step_range != 0.f && Doors.step_enabled && false == Doors.is_locked && door.step_position < 1.f && door.is_opening;
	door.step_folding = door_step_folding(door);

	if (true == door.is_opening)
	{
		door.auto_timer = door_open_duration(door, localopencontrol, remoteopencontrol);
	}
}

// whether the door is asked to open, by the passengers, from afar or by itself
bool TMoverParameters::door_open_request(basic_door const &door, bool const localopencontrol, bool const remoteopencontrol) const
{
	auto const autoopenrequest{Doors.open_control == control_t::autonomous && (false == Doors.permit_needed || door.open_permit)};
	return (localopencontrol && door.local_open) || (remoteopencontrol && door.remote_open) || (autoopenrequest && false == door.is_open);
}

// whether the door is asked to close, by the passengers, from afar, or by itself: over the speed, after its time or
// with its permit taken back
bool TMoverParameters::door_close_request(basic_door const &door, bool const localclosecontrol, bool const remoteclosecontrol) const
{
	auto const autocloserequest{(Doors.auto_velocity != -1.f && Vel > Doors.auto_velocity) || (door.auto_timer != -1.f && door.auto_timer <= 0.f) ||
	                            (Doors.permit_needed && false == door.open_permit)};
	return (door.remote_close && remoteclosecontrol) || (door.local_close && localclosecontrol) || (autocloserequest && door.is_open);
}

// whether the doors have the voltage they work on
bool TMoverParameters::doors_powered() const
{
	if (Doors.voltage == 0)
	{
		return true;
	}
	if (Doors.voltage == 24)
	{
		return Power24vIsAvailable || Power110vIsAvailable;
	}
	return Doors.voltage == 110 && Power110vIsAvailable;
}

// whether the unfolded step folds: without the permission to stay out, or with the door closing, once it's about to
// close or closed
bool TMoverParameters::door_step_folding(basic_door const &door) const
{
	auto const unfolded{door.step_position > 0.f};
	auto const mayfold{false == Doors.step_enabled || (Doors.permit_needed ? false == door.open_permit : door.is_closing)};
	return unfolded && mayfold && (door.close_delay > Doors.close_delay || door.position <= 0.f);
}

// how long the opening door stays open: the time of the doors, for the passengers' opening and, if it counts, the
// opening from afar; none otherwise
float TMoverParameters::door_open_duration(basic_door const &door, bool const localopencontrol, bool const remoteopencontrol) const
{
	if (localopencontrol && door.local_open)
	{
		return Doors.auto_duration;
	}
	if (remoteopencontrol && door.remote_open && Doors.auto_include_remote)
	{
		return Doors.auto_duration;
	}
	return -1.f;
}

// a door and its step moved as they open or close
void TMoverParameters::move_door(basic_door &door, double const Deltatime)
{
	// doors
	if (true == door.is_opening)
	{
		// open door
		if (false == door.step_unfolding // no wait if no doorstep
		    || Doors.step_type == 2)
		{ // no wait for rotating doorstep
			door.open_delay += static_cast<float>(Deltatime);
			if (door.open_delay > Doors.open_delay)
			{
				door.position = std::min<float>(Doors.range, static_cast<float>(door.position + Doors.open_rate * Deltatime));
			}
		}
		door.close_delay = 0.f;
	}
	if (true == door.is_closing)
	{
		// close door
		door.close_delay += static_cast<float>(Deltatime);
		if (door.close_delay > Doors.close_delay)
		{
			door.position = std::max<float>(0.f, static_cast<float>(door.position - Doors.close_rate * Deltatime));
		}
		door.open_delay = 0.f;
	}
	// doorsteps
	if (door.step_unfolding)
	{
		// unfold left doorstep
		door.step_position = std::min<float>(1.f, static_cast<float>(door.step_position + Doors.step_rate * Deltatime));
	}
	if (door.step_folding)
	{
		// fold left doorstep
		if (is_multiple_unit())
		{
			// multi-unit vehicles typically fold the doorstep only after closing the door
			if (door.position <= 0.f)
			{
				door.step_position = std::max<float>(0.f, static_cast<float>(door.step_position - Doors.step_rate * Deltatime));
			}
		}
		else
		{
			door.step_position = std::max<float>(0.f, static_cast<float>(door.step_position - Doors.step_rate * Deltatime));
		}
	}
}

// the timers of the open doors which close after a while, kept running while the load is exchanged
void TMoverParameters::update_door_timers(double const Deltatime)
{
	// update door timers if the door close after defined time
	for (auto &door : Doors.instances)
	{

		if (false == door.is_open)
		{
			continue;
		}

		if (door.auto_timer > 0.f)
		{
			door.auto_timer -= static_cast<float>(Deltatime);
		}
		// if there's load exchange in progress, reset the timer(s) for already open doors
		if (door.auto_timer != -1.f && (LoadStatus & (2 | 1)) != 0)
		{
			door.auto_timer = Doors.auto_duration;
		}
	}
}
