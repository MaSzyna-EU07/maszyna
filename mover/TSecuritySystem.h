#pragma once

#include "mover/core.h"

namespace mover
{
class TSecuritySystem
{
	bool vigilance_enabled = false;
	bool cabsignal_enabled = false;
	bool radiostop_enabled = false;

	bool cabsignal_active = false;
	bool pressed = false;
	bool enabled = false;
	bool is_sifa = false; // Sifa-like pedal device, with inverted input for convenient keyboard usage
	bool separate_acknowledge = false; // cabsignal reset button is separate from vigilance
	bool cabsignal_lock = false;
	bool vigilance_suspended = false; // the vigilance device switched off by the shunt mode of the vehicle, see AwareOffInShuntMode

	double vigilance_timer = 0.0;
	double alert_timer = 0.0;
	double press_timer = 0.0;

	double velocity = 0.0;
	bool power = false;
	int cabactive = 0;

	double AwareDelay = 30.0;
	double AwareMinSpeed = 0.0;
	double SoundSignalDelay = 5.0;
	double EmergencyBrakeDelay = 5.0;
	double MaxHoldTime = 1.5;
	bool CabDependent = false;
	bool AwareOffInShuntMode = false; // the shunt mode of the vehicle switches the vigilance device off, as on the SM42

  public:
	void set_enabled(bool e);
	void acknowledge_press();
	void acknowledge_release();
	void cabsignal_reset();
	void update(double dt, double Vel, bool pwr, int cab);
	void set_cabsignal();
	void set_cabsignal_lock(bool);
	void set_shunt_mode(bool State);
	bool is_blinking() const;
	bool is_vigilance_blinking() const;
	bool is_cabsignal_blinking() const;
	bool is_beeping() const;
	bool is_cabsignal_beeping() const;
	bool is_braking() const;
	bool is_engine_blocked() const;
	bool radiostop_available() const;
	bool has_separate_acknowledge() const;
	void load(std::string const &line, double Vmax);

	double MagnetLocation = 0.0;
};

} // namespace mover
