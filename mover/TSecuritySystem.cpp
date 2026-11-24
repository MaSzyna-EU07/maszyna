#include "stdafx.h"
#include "mover/TSecuritySystem.h"

#include "utilities/Globals.h"
#include "utilities/parser.h"

namespace mover
{

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


} // namespace mover
