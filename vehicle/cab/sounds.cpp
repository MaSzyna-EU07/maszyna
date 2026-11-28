/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others

*/

// the sounds of the cab

#include "stdafx.h"
#include "vehicle/Train.h"
#include "vehicle/horn.h"
#include "vehicle/linebreaker.h"
#include "vehicle/holdcoillever.h"
#include "vehicle/compartmentlightsswitch.h"
#include "vehicle/odometer.h"
#include "vehicle/mainctrlact.h"
#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "world/Event.h"
#include "simulation/simulationtime.h"
#include "vehicle/Camera.h"
#include "utilities/Logs.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "utilities/Timer.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "world/mtable.h"
#include "Console.h"
#include "application/application.h"
#include "rendering/renderer.h"
#include "physics/mover.h"
#include <future>
#include <thread>
#include <cmath>
#include <algorithm>
#include <format>
#include "vehicle/cab/internal.h"


void TTrain::update_sounds(double const Deltatime)
{

	double volume{0.0};
	double const brakevolumescale{0.5};

	// Winger-160404 - syczenie pomocniczego (luzowanie)
	if (m_lastlocalbrakepressure != -1.f)
	{
		// calculate rate of pressure drop in local brake cylinder, once it's been initialized
		auto const brakepressuredifference{mvOccupied->LocBrakePress - m_lastlocalbrakepressure};
		m_localbrakepressurechange = static_cast<float>(std::lerp(m_localbrakepressurechange, 10 * (brakepressuredifference / Deltatime), 0.1f));
	}
	m_lastlocalbrakepressure = static_cast<float>(mvOccupied->LocBrakePress);
	// local brake, release
	if (rsSBHiss)
	{
		if (m_localbrakepressurechange < -0.05f && mvOccupied->LocBrakePress > mvOccupied->BrakePress - 0.05)
		{
			rsSBHiss->gain(static_cast<float>(std::clamp(rsSBHiss->m_amplitudeoffset + rsSBHiss->m_amplitudefactor * -m_localbrakepressurechange * 0.05, 0.0, 1.5)));
			rsSBHiss->play(sound_flags::exclusive | sound_flags::looping);
		}
		else
		{
			// don't stop the sound too abruptly
			volume = std::max(0.0, rsSBHiss->gain() - 0.1 * Deltatime);
			rsSBHiss->gain(static_cast<float>(volume));
			if (volume < 0.05)
			{
				rsSBHiss->stop();
			}
		}
	}
	// local brake, engage
	if (rsSBHissU)
	{
		if (m_localbrakepressurechange > 0.05f)
		{
			rsSBHissU->gain(static_cast<float>(std::clamp(rsSBHissU->m_amplitudeoffset + rsSBHissU->m_amplitudefactor * m_localbrakepressurechange * 0.05, 0.0, 1.5)));
			rsSBHissU->play(sound_flags::exclusive | sound_flags::looping);
		}
		else
		{
			// don't stop the sound too abruptly
			volume = std::max(0.0, rsSBHissU->gain() - 0.1 * Deltatime);
			rsSBHissU->gain(static_cast<float>(volume));
			if (volume < 0.05)
			{
				rsSBHissU->stop();
			}
		}
	}

	// McZapkie-280302 - syczenie
	// TODO: softer volume reduction than plain abrupt stop, perhaps as reusable wrapper?
	if (mvOccupied->BrakeHandle == TBrakeHandle::FV4a || mvOccupied->BrakeHandle == TBrakeHandle::FVel6)
	{
		// upuszczanie z PG
		if (rsHiss)
		{
			fPPress = std::lerp(fPPress, static_cast<float>(mvOccupied->Handle->GetSound(s_fv4a_b)), 0.05f);
			volume = fPPress > 0 ? rsHiss->m_amplitudefactor * fPPress * 0.25 + rsHiss->m_amplitudeoffset : 0;
			if (volume * brakevolumescale > 0.05)
			{
				rsHiss->gain(static_cast<float>(volume * brakevolumescale));
				rsHiss->play(sound_flags::exclusive | sound_flags::looping);
			}
			else
			{
				rsHiss->stop();
			}
		}
		// napelnianie PG
		if (rsHissU)
		{
			fNPress = std::lerp(fNPress, static_cast<float>(mvOccupied->Handle->GetSound(s_fv4a_u)), 0.25f);
			volume = fNPress > 0 ? rsHissU->m_amplitudefactor * fNPress + rsHissU->m_amplitudeoffset : 0;
			if (volume * brakevolumescale > 0.05)
			{
				rsHissU->gain(static_cast<float>(volume * brakevolumescale));
				rsHissU->play(sound_flags::exclusive | sound_flags::looping);
			}
			else
			{
				rsHissU->stop();
			}
		}
		// upuszczanie przy naglym
		if (rsHissE)
		{
			volume = mvOccupied->Handle->GetSound(s_fv4a_e) * rsHissE->m_amplitudefactor + rsHissE->m_amplitudeoffset;
			if (volume * brakevolumescale > 0.05)
			{
				rsHissE->gain(static_cast<float>(volume * brakevolumescale));
				rsHissE->play(sound_flags::exclusive | sound_flags::looping);
			}
			else
			{
				rsHissE->stop();
			}
		}
		// upuszczanie sterujacego fala
		if (rsHissX)
		{
			volume = mvOccupied->Handle->GetSound(s_fv4a_x) * rsHissX->m_amplitudefactor + rsHissX->m_amplitudeoffset;
			if (volume * brakevolumescale > 0.05)
			{
				rsHissX->gain(static_cast<float>(volume * brakevolumescale));
				rsHissX->play(sound_flags::exclusive | sound_flags::looping);
			}
			else
			{
				rsHissX->stop();
			}
		}
		// upuszczanie z czasowego
		if (rsHissT)
		{
			volume = mvOccupied->Handle->GetSound(s_fv4a_t) * rsHissT->m_amplitudefactor + +rsHissT->m_amplitudeoffset;
			if (volume * brakevolumescale > 0.05)
			{
				rsHissT->gain(static_cast<float>(volume * brakevolumescale));
				rsHissT->play(sound_flags::exclusive | sound_flags::looping);
			}
			else
			{
				rsHissT->stop();
			}
		}
	}
	else
	{
		// jesli nie FV4a
		// upuszczanie z PG
		if (rsHiss)
		{
			fPPress = static_cast<float>((4.0f * fPPress + std::max(0.0, mvOccupied->dpMainValve)) / (4.0f + 1.0f));
			volume = fPPress > 0.0f ? 2.0 * rsHiss->m_amplitudefactor * fPPress + rsHiss->m_amplitudeoffset : 0.0;
			if (volume > 0.05)
			{
				rsHiss->gain(static_cast<float>(volume));
				rsHiss->play(sound_flags::exclusive | sound_flags::looping);
			}
			else
			{
				rsHiss->stop();
			}
		}
		// napelnianie PG
		if (rsHissU)
		{
			fNPress = static_cast<float>((4.0f * fNPress + std::min(0.0, mvOccupied->dpMainValve)) / (4.0f + 1.0f));
			volume = fNPress < 0.0f ? -1.0 * rsHissU->m_amplitudefactor * fNPress + rsHissU->m_amplitudeoffset : 0.0;
			if (volume > 0.01)
			{
				rsHissU->gain(static_cast<float>(volume));
				rsHissU->play(sound_flags::exclusive | sound_flags::looping);
			}
			else
			{
				rsHissU->stop();
			}
		}
	} // koniec nie FV4a

	// brakes
	if (rsBrake)
	{
		if (mvOccupied->UnitBrakeForce > 10.0 && mvOccupied->Vel > 0.05)
		{

			auto const brakeforceratio{
			    std::clamp(mvOccupied->UnitBrakeForce / std::max(1.0, mvOccupied->BrakeForceR(1.0, mvOccupied->Vel) / (mvOccupied->NAxles * std::max(1, mvOccupied->NBpA))), 0.0, 1.0)};
			// HACK: in external view mute the sound rather than stop it, in case there's an opening bookend it'd (re)play on sound restart after returning inside
			volume = FreeFlyModeFlag ? 0.0 : rsBrake->m_amplitudeoffset + std::sqrt(brakeforceratio * std::lerp(0.4, 1.0, mvOccupied->Vel / (1 + mvOccupied->Vmax))) * rsBrake->m_amplitudefactor;
			rsBrake->pitch(static_cast<float>(rsBrake->m_frequencyoffset + mvOccupied->Vel * rsBrake->m_frequencyfactor));
			rsBrake->gain(static_cast<float>(volume));
			rsBrake->play(sound_flags::exclusive | sound_flags::looping);
		}
		else
		{
			rsBrake->stop();
		}
	}

	// ambient sound
	// since it's typically ticking of the clock we can center it on tachometer or on middle of compartment bounding area
	if (rsFadeSound)
	{
		rsFadeSound->play(sound_flags::exclusive | sound_flags::looping);
	}

	if (dsbSlipAlarm)
	{
		// alarm przy poslizgu dla 181/182 - BOMBARDIER
		if (mvControlled->SlippingWheels && DynamicObject->GetVelocity() > 1.0)
		{
			dsbSlipAlarm->play(sound_flags::exclusive | sound_flags::looping);
		}
		else
		{
			dsbSlipAlarm->stop();
		}
	}

	// dzwiek wiatru rozbijajacego sie o szyby w kabinie
	if (rsWindSound)
	{
		if (!FreeFlyModeFlag && !Global.CabWindowOpen && DynamicObject->GetVelocity() > 0.5)
			update_sounds_resonancenoise(*rsWindSound);
		else
			rsWindSound->stop(FreeFlyModeFlag);
	}

	// dzwiek rezonansu (taki drugi runningnoise w sumie)
	if (rsResonanceNoise)
	{
		if (!FreeFlyModeFlag && !Global.CabWindowOpen && DynamicObject->GetVelocity() > 0.5)
		{

			update_sounds_resonancenoise(*rsResonanceNoise);
		}
		else
			rsResonanceNoise->stop(FreeFlyModeFlag);
	}

	// szum w czasie jazdy
	if (rsRunningNoise)
	{
		if (false == FreeFlyModeFlag && false == Global.CabWindowOpen && DynamicObject->GetVelocity() > 0.5)
		{

			update_sounds_runningnoise(*rsRunningNoise);
		}
		else
		{
			// don't play the optional ending sound if the listener switches views
			rsRunningNoise->stop(true == FreeFlyModeFlag);
		}
	}
	// hunting oscillation noise
	if (rsHuntingNoise)
	{
		if (false == FreeFlyModeFlag && false == Global.CabWindowOpen && DynamicObject->GetVelocity() > 0.5 && DynamicObject->IsHunting)
		{

			update_sounds_runningnoise(*rsHuntingNoise);
			// modify calculated sound volume by hunting amount
			auto const huntingamount = std::lerp(
			    0.0, 1.0, std::clamp((mvOccupied->Vel - DynamicObject->HuntingShake.fadein_begin) / (DynamicObject->HuntingShake.fadein_end - DynamicObject->HuntingShake.fadein_begin), 0.0, 1.0));

			rsHuntingNoise->gain(static_cast<float>(rsHuntingNoise->gain() * huntingamount));
		}
		else
		{
			// don't play the optional ending sound if the listener switches views
			rsHuntingNoise->stop(true == FreeFlyModeFlag);
		}
	}
	// rain sound
	if (m_rainsound)
	{
		if (false == FreeFlyModeFlag && false == Global.CabWindowOpen && Global.Weather == "rain:")
		{
			if (m_rainsound->is_combined())
			{
				m_rainsound->pitch(static_cast<float>(Global.Overcast - 1.0));
			}
			m_rainsound->gain(m_rainsound->m_amplitudeoffset + m_rainsound->m_amplitudefactor * 1.f);
			m_rainsound->play(sound_flags::exclusive | sound_flags::looping);
		}
		else
		{
			m_rainsound->stop();
		}
	}

	if (dsbHasler)
	{
		if (fTachoCount >= 3.f)
		{
			auto const soundfrequency{(true == dsbHasler->is_combined() ? fTachoVelocity * 0.01 : dsbHasler->m_frequencyoffset + dsbHasler->m_frequencyfactor)};
			dsbHasler->pitch(static_cast<float>(soundfrequency));
			dsbHasler->gain(dsbHasler->m_amplitudeoffset + dsbHasler->m_amplitudefactor);
			dsbHasler->play(sound_flags::exclusive | sound_flags::looping);
		}
		else if (fTachoCount < 1.f)
		{
			dsbHasler->stop();
		}
	}

	// power-reliant sounds
	if (mvOccupied->Power24vIsAvailable || mvOccupied->Power110vIsAvailable)
	{

		// buzzer shp
		if (mvOccupied->SecuritySystem.is_cabsignal_beeping())
		{
			if (dsbBuzzerShp && false == dsbBuzzerShp->is_playing())
			{
				dsbBuzzerShp->pitch(dsbBuzzerShp->m_frequencyoffset + dsbBuzzerShp->m_frequencyfactor);
				dsbBuzzerShp->gain(dsbBuzzerShp->m_amplitudeoffset + dsbBuzzerShp->m_amplitudefactor);
				dsbBuzzerShp->play(sound_flags::looping);
			}
		}
		else
		{
			if (dsbBuzzerShp && true == dsbBuzzerShp->is_playing())
			{
				dsbBuzzerShp->stop();
			}
		}

		// McZapkie-141102: SHP i czuwak, TODO: sygnalizacja kabinowa
		// hunter-091012: rozdzielenie alarmow
		if (mvOccupied->SecuritySystem.is_beeping())
		{
			if (dsbBuzzer && false == dsbBuzzer->is_playing())
			{
				dsbBuzzer->pitch(dsbBuzzer->m_frequencyoffset + dsbBuzzer->m_frequencyfactor);
				dsbBuzzer->gain(dsbBuzzer->m_amplitudeoffset + dsbBuzzer->m_amplitudefactor);
				dsbBuzzer->play(sound_flags::looping);
#ifdef _WIN32
				Console::BitsSet(1 << 14); // ustawienie bitu 16 na PoKeys
#endif
			}
		}
		else
		{
			if (dsbBuzzer && true == dsbBuzzer->is_playing())
			{
				dsbBuzzer->stop();
#ifdef _WIN32
				Console::BitsClear(1 << 14); // ustawienie bitu 16 na PoKeys
#endif
			}
		}
		// distance meter alert
		if (m_distancecounterclear)
		{
			auto const *owner{(DynamicObject->ctOwner != nullptr ? DynamicObject->ctOwner : DynamicObject->Mechanik)};
			if (m_distancecounter > owner->fLength)
			{
				// play assigned sound if the train travelled its full length since meter activation
				// TBD: check all combinations of directions and active cab
				m_distancecounter = -1.f; // turn off the meter after its task is done
				m_distancecounterclear->pitch(m_distancecounterclear->m_frequencyoffset + m_distancecounterclear->m_frequencyfactor);
				m_distancecounterclear->gain(m_distancecounterclear->m_amplitudeoffset + m_distancecounterclear->m_amplitudefactor);
				m_distancecounterclear->play(sound_flags::exclusive);
			}
		}
	}
	else
	{
		// stop power-reliant sounds if power is cut
		if (dsbBuzzer && true == dsbBuzzer->is_playing())
		{
			dsbBuzzer->stop();
#ifdef _WIN32
			Console::BitsClear(1 << 14); // ustawienie bitu 16 na PoKeys
#endif
		}

		if (dsbBuzzerShp && dsbBuzzerShp->is_playing())
		{
			dsbBuzzerShp->stop();
		}

		if (m_distancecounterclear)
		{
			m_distancecounterclear->stop();
		}
	}

	update_sounds_radio();
	update_sounds_fmradio(Deltatime);
}

void TTrain::update_sounds_resonancenoise(sound_source &Sound) const
{
	// frequency calculation
	auto const normalizer{mvOccupied->Vmax * 0.01f};
	auto const soundfrequency{Sound.m_frequencyoffset + Sound.m_frequencyfactor * mvOccupied->Vel * normalizer};

	// volume calculation
	auto volume = Sound.m_amplitudeoffset + Sound.m_amplitudefactor * std::lerp(mvOccupied->Vel / (1 + mvOccupied->Vmax), 1.0, 0.5); // scale base volume between 0.5-1.0

	if (volume > 0.05)
	{
		Sound.pitch(static_cast<float>(soundfrequency)).gain(static_cast<float>(volume)).play(sound_flags::exclusive | sound_flags::looping);
	}
	else
	{
		Sound.stop();
	}
}

void TTrain::update_sounds_runningnoise(sound_source &Sound)
{
	// frequency calculation
	auto const normalizer{(true == Sound.is_combined() ? mvOccupied->Vmax * 0.01f : 1.f)};
	auto const soundfrequency{Sound.m_frequencyoffset + Sound.m_frequencyfactor * mvOccupied->Vel * normalizer};

	// volume calculation
	auto volume = Sound.m_amplitudeoffset + Sound.m_amplitudefactor * std::lerp(mvOccupied->Vel / (1 + mvOccupied->Vmax), 1.0,
	                                                                              0.5); // scale base volume between 0.5-1.0
	if (std::abs(mvOccupied->nrot) > 0.01)
	{
		// hamulce wzmagaja halas
		auto const brakeforceratio{(std::clamp(mvOccupied->UnitBrakeForce / std::max(1.0, mvOccupied->BrakeForceR(1.0, mvOccupied->Vel) / (mvOccupied->NAxles * std::max(1, mvOccupied->NBpA))), 0.0, 1.0))};

		volume *= 1 + 0.125 * brakeforceratio;
	}
	// scale volume by track quality
	// TODO: track quality and/or environment factors as separate subroutine
	volume *= std::lerp(0.8, 1.2, std::clamp(DynamicObject->MyTrack->iQualityFlag / 20.0, 0.0, 1.0));
	// for single sample sounds muffle the playback at low speeds
	if (false == Sound.is_combined())
	{
		volume *= std::lerp(0.0, 1.0, std::clamp(mvOccupied->Vel / 25.0, 0.0, 1.0));
	}

	if (volume > 0.05)
	{
		Sound.pitch(static_cast<float>(soundfrequency)).gain(static_cast<float>(volume)).play(sound_flags::exclusive | sound_flags::looping);
	}
	else
	{
		Sound.stop();
	}
}

void TTrain::update_sounds_radio()
{

	radio_message_played = false;
	if (false == m_radiomessages.empty())
	{
		// erase completed radio messages from the list
		std::erase_if(m_radiomessages, [](auto const &source) { return false == source.second->is_playing(); });
	}
	// adjust audibility of remaining messages based on current radio conditions
	auto const radioenabled{mvOccupied->RadioPowered()};
	for (auto const &[messagechannel, messagesound] : m_radiomessages)
	{
		auto const volume{true == radioenabled && Dynamic()->Mechanik != nullptr && messagechannel == RadioChannel() ? m_radiovolume : 0.0};
		messagesound->gain(static_cast<float>(volume));
		radio_message_played |= true == radioenabled && Dynamic()->Mechanik != nullptr && messagechannel == RadioChannel();
	}
	// radiostop
	if (m_radiostop)
	{
		if (true == radioenabled && true == mvOccupied->RadioStopFlag)
		{
			m_radiostop->play(sound_flags::exclusive | sound_flags::looping);
			radio_message_played |= true;
		}
		else
		{
			m_radiostop->stop();
		}
	}
	if (radio_message_played)
	{
		btLampkaRadioMessage.gain(m_radiovolume);
	}
}

// operates the broadcast radio receiver
void TTrain::update_sounds_fmradio(double const Deltatime)
{
	// NOTE: there's a single receiver in the simulation, it plays in the cab occupied by the user
	if (simulation::Train != this || false == m_fmradiopresent || false == Global.fmradio_enabled)
	{
		return;
	}
	auto &fmradio{audio::renderer.fmradio()};
	auto const bandwidth{audio::EU07_FMRADIO_BANDEND - audio::EU07_FMRADIO_BANDSTART};
	if (false == m_fmradiotuned)
	{
		// initially the receiver is tuned to the first station on the list
		m_fmradiodial = 100.f * (fmradio.default_frequency() - audio::EU07_FMRADIO_BANDSTART) / bandwidth;
		m_fmradiotuned = true;
	}
	// knobs
	if (m_fmradiovolumechange != 0)
	{
		m_fmradiovolume = std::clamp(m_fmradiovolume + m_fmradiovolumechange * 0.5f * static_cast<float>(Deltatime), 0.f, 1.f);
		if (false == m_controlmapper.contains("fmradio_sw:"))
		{
			// receivers without dedicated switch are switched with the volume knob
			m_fmradiopower = m_fmradiovolume > 0.f;
		}
	}
	if (m_fmradiotunechange != 0)
	{
		// the knob moves slowly at first to allow fine tuning, then speeds up
		m_fmradiotunetime += static_cast<float>(Deltatime);
		auto const tunerate{std::lerp(0.15f, 2.5f, std::clamp(m_fmradiotunetime * 0.5f, 0.f, 1.f))}; // MHz per second
		m_fmradiodial = std::clamp(m_fmradiodial + 100.f * m_fmradiotunechange * tunerate * static_cast<float>(Deltatime) / bandwidth, 0.f, 100.f);
	}
	else
	{
		m_fmradiotunetime = 0.f;
	}

	fmradio.type(m_fmradiotype);
	fmradio.tune(audio::EU07_FMRADIO_BANDSTART + m_fmradiodial * 0.01f * bandwidth);
	fmradio.volume(m_fmradiovolume);
	fmradio.placement(m_fmradiosound.location(), m_fmradiosound.soundproofing_factor(), m_fmradiosound.range());
	// NOTE: the receiver runs on batteries of its own, so it doesn't care about the state of the vehicle
	fmradio.power(m_fmradiopower);
}
