/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the security system (vigilance, cab signalling, radio-stop)

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


void TSecuritySystem::set_enabled(bool e)
{
	if (vigilance_enabled || cabsignal_enabled || radiostop_enabled)
		enabled = e;
	if (CabDependent)
		cabactive = 0;
}

void TSecuritySystem::acknowledge_press()
{
	pressed = true;

	if (vigilance_timer > AwareDelay)
	{
		alert_timer = 0.0;
		vigilance_timer = 0.0;
		return;
	}
	vigilance_timer = 0.0;

	if (!separate_acknowledge && cabsignal_active && !cabsignal_lock)
	{
		cabsignal_active = false;
		alert_timer = 0.0;
	}
}

void TSecuritySystem::acknowledge_release()
{
	pressed = false;

	if (press_timer > MaxHoldTime)
		alert_timer = 0.0;
	press_timer = 0.0;
}

void TSecuritySystem::cabsignal_reset()
{
	if (cabsignal_active && !cabsignal_lock)
	{
		cabsignal_active = false;
		alert_timer = 0.0;
	}
}

void TSecuritySystem::update(double dt, double vel, bool pwr, int cab)
{
	if (!enabled || !pwr || DebugModeFlag)
	{
		power = pwr;
		cabsignal_active = false;
		vigilance_timer = 0.0;
		alert_timer = 0.0;
		press_timer = 0.0;
		return;
	}

	bool just_powered_on = !power && pwr;
	bool just_activated = CabDependent && cabactive != cab;

	/* enabling battery */
	if (cabsignal_enabled && (just_powered_on || just_activated))
	{
		cabsignal_active = true;
		alert_timer = SoundSignalDelay;
	}

	power = pwr;
	velocity = vel;
	cabactive = cab;

	if (vigilance_suspended)
	{
		// the vigilance device is switched off: no time counts towards a new alarm, of the alerter or of the button held
		// too long. an alarm already on, with the emergency braking it leads to, stays until it's acknowledged as usual
		if (vigilance_timer <= AwareDelay)
			vigilance_timer = 0.0;
		if (press_timer <= MaxHoldTime)
			press_timer = 0.0;
	}
	else
	{
		if (vigilance_enabled && velocity > AwareMinSpeed)
			vigilance_timer += dt;

		if (pressed && (!is_sifa || velocity > AwareMinSpeed))
			press_timer += dt;
	}

	if (vigilance_timer > AwareDelay || press_timer > MaxHoldTime || cabsignal_active)
		alert_timer += dt;
}

void TSecuritySystem::set_cabsignal()
{
	if (cabsignal_enabled && power)
		cabsignal_active = true;
}

bool TSecuritySystem::has_separate_acknowledge() const
{
	return separate_acknowledge;
}

bool TSecuritySystem::is_blinking() const
{
	if (!power)
		return false;

	return alert_timer > 0.0;
}

bool TSecuritySystem::is_vigilance_blinking() const
{
	if (!power)
		return false;

	return press_timer > MaxHoldTime || vigilance_timer > AwareDelay;
}

bool TSecuritySystem::is_cabsignal_blinking() const
{
	if (!power)
		return false;

	return cabsignal_active;
}

bool TSecuritySystem::is_beeping() const
{
	if (!power)
		return false;

	return alert_timer > SoundSignalDelay && (!separate_acknowledge || is_vigilance_blinking());
}

bool TSecuritySystem::is_cabsignal_beeping() const
{
	if (!power)
		return false;

	return alert_timer > SoundSignalDelay && is_cabsignal_blinking();
}

bool TSecuritySystem::is_braking() const
{
	if (!power && enabled)
		return true;

	return alert_timer > SoundSignalDelay + EmergencyBrakeDelay;
}

bool TSecuritySystem::radiostop_available() const
{
	return radiostop_enabled;
}

void TSecuritySystem::set_cabsignal_lock(bool v)
{
	cabsignal_lock = v;
}

void TSecuritySystem::set_shunt_mode(bool const State)
{
	vigilance_suspended = State && AwareOffInShuntMode;
}

bool TSecuritySystem::is_engine_blocked() const
{
	if (!is_sifa)
		return false;

	return velocity < AwareMinSpeed && pressed;
}

void TSecuritySystem::load(std::string const &line, double Vmax)
{
	std::string awaresystem = extract_value("AwareSystem", line);
	if (awaresystem.find("Active") != std::string::npos)
		vigilance_enabled = true;
	if (awaresystem.find("CabSignal") != std::string::npos)
		cabsignal_enabled = true;
	if (awaresystem.find("Sifa") != std::string::npos)
		is_sifa = true;
	if (awaresystem.find("SeparateAcknowledge") != std::string::npos)
		separate_acknowledge = true;

	extract_value(AwareDelay, "AwareDelay", line, "");
	AwareMinSpeed = 0.1 * Vmax; // domyślnie 10% Vmax
	extract_value(AwareMinSpeed, "AwareMinSpeed", line, "");
	extract_value(SoundSignalDelay, "SoundSignalDelay", line, "");
	extract_value(EmergencyBrakeDelay, "EmergencyBrakeDelay", line, "");
	extract_value(MaxHoldTime, "MaxHoldTime", line, "");
	extract_value(radiostop_enabled, "RadioStop", line, "");
	extract_value(MagnetLocation, "MagnetLocation", line, "");
	extract_value(CabDependent, "CabDependent", line, "");
	extract_value(AwareOffInShuntMode, "AwareOffInShuntMode", line, "");
}

// *****************************************************************************
// Q: 20160710
// zbicie czuwaka / SHP
// *****************************************************************************
// hunter-091012: rozbicie alarmow, dodanie testu czuwaka
void TMoverParameters::SecuritySystemReset(void) // zbijanie czuwaka/SHP
{
	// reset all, used by AI
	SecuritySystem.acknowledge_press();
	SecuritySystem.acknowledge_release();
	SecuritySystem.cabsignal_reset();
}

// *************************************************************************************************
// Q: 20160711
// sprawdzanie stanu CA/SHP
// *************************************************************************************************
void TMoverParameters::SecuritySystemCheck(double dt)
{
	bool isPower = Power24vIsAvailable || Power110vIsAvailable;
	// the shunt mode of the SM42 switches its vigilance device (CA) off, on the vehicles with AwareOffInShuntMode=Yes (eu07/maszyna#457)
	SecuritySystem.set_shunt_mode(ShuntMode);
	SecuritySystem.update(dt, Vel, isPower, CabActive);

	if (!Battery || !Radio || !RadioFuse)
	{ // wyłączenie baterii deaktywuje sprzęt
		RadiostopSwitch(false);
	}
}

// *************************************************************************************************
// Q: 20160711
// włączenie / wyłączenie hamowania awaryjnego
// *************************************************************************************************
bool TMoverParameters::RadiostopSwitch(bool Switch)
{
	bool EBS;
	if (BrakeSystem != TBrakeSystem::Individual && BrakeCtrlPosNo > 0)
	{

		if (true == Switch && false == RadioStopFlag)
		{
			RadioStopFlag = Switch;
			EBS = true;
		}
		else
		{
			if (Switch == false && std::abs(V) < 0.1)
			{
				// odblokowanie hamulca bezpieczenistwa tylko po zatrzymaniu
				RadioStopFlag = Switch;
				EBS = true;
			}
			else
			{
				EBS = false;
			}
		}
	}
	else
	{
		// nie ma hamulca bezpieczenstwa gdy nie ma hamulca zesp.
		EBS = false;
	}

	return EBS;
}
