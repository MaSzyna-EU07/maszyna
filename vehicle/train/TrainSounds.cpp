/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/train/Train.h"

#include "utilities/Globals.h"
#include "world/Event.h"
#include "simulation/simulationtime.h"
#include "model/Model3d.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "Console.h"
#include "rendering/renderer.h"
#include <future>
#include <cmath>
#include <algorithm>


void TTrain::update_sounds(double const Deltatime)
{

	double volume{0.0};
	double const brakevolumescale{0.5};

	// Winger-160404 - syczenie pomocniczego (luzowanie)
	if (m_lastlocalbrakepressure != -1.f)
	{
		// calculate rate of pressure drop in local brake cylinder, once it's been initialized
		auto const brakepressuredifference{mvOccupied->LocBrakePress - m_lastlocalbrakepressure};
		m_localbrakepressurechange = std::lerp(m_localbrakepressurechange, 10 * (brakepressuredifference / Deltatime), 0.1f);
	}
	m_lastlocalbrakepressure = mvOccupied->LocBrakePress;
	// local brake, release
	if (rsSBHiss)
	{
		if (m_localbrakepressurechange < -0.05f && mvOccupied->LocBrakePress > mvOccupied->BrakePress - 0.05)
		{
			rsSBHiss->gain(std::clamp(rsSBHiss->m_amplitudeoffset + rsSBHiss->m_amplitudefactor * -m_localbrakepressurechange * 0.05, 0.0, 1.5));
			rsSBHiss->play(sound_flags::exclusive | sound_flags::looping);
		}
		else
		{
			// don't stop the sound too abruptly
			volume = std::max(0.0, rsSBHiss->gain() - 0.1 * Deltatime);
			rsSBHiss->gain(volume);
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
			rsSBHissU->gain(std::clamp(rsSBHissU->m_amplitudeoffset + rsSBHissU->m_amplitudefactor * m_localbrakepressurechange * 0.05, 0.0, 1.5));
			rsSBHissU->play(sound_flags::exclusive | sound_flags::looping);
		}
		else
		{
			// don't stop the sound too abruptly
			volume = std::max(0.0, rsSBHissU->gain() - 0.1 * Deltatime);
			rsSBHissU->gain(volume);
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
				rsHiss->gain(volume * brakevolumescale);
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
				rsHissU->gain(volume * brakevolumescale);
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
				rsHissE->gain(volume * brakevolumescale);
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
				rsHissX->gain(volume * brakevolumescale);
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
				rsHissT->gain(volume * brakevolumescale);
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
			fPPress = (4.0f * fPPress + std::max(0.0, mvOccupied->dpMainValve)) / (4.0f + 1.0f);
			volume = fPPress > 0.0f ? 2.0 * rsHiss->m_amplitudefactor * fPPress + rsHiss->m_amplitudeoffset : 0.0;
			if (volume > 0.05)
			{
				rsHiss->gain(volume);
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
			fNPress = (4.0f * fNPress + std::min(0.0, mvOccupied->dpMainValve)) / (4.0f + 1.0f);
			volume = fNPress < 0.0f ? -1.0 * rsHissU->m_amplitudefactor * fNPress + rsHissU->m_amplitudeoffset : 0.0;
			if (volume > 0.01)
			{
				rsHissU->gain(volume);
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
			rsBrake->pitch(rsBrake->m_frequencyoffset + mvOccupied->Vel * rsBrake->m_frequencyfactor);
			rsBrake->gain(volume);
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

			rsHuntingNoise->gain(rsHuntingNoise->gain() * huntingamount);
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
				m_rainsound->pitch(Global.Overcast - 1.0);
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
			auto const frequency{(true == dsbHasler->is_combined() ? fTachoVelocity * 0.01 : dsbHasler->m_frequencyoffset + dsbHasler->m_frequencyfactor)};
			dsbHasler->pitch(frequency);
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
		if (dsbBuzzer)
		{
			if (true == dsbBuzzer->is_playing())
			{
				dsbBuzzer->stop();
#ifdef _WIN32
				Console::BitsClear(1 << 14); // ustawienie bitu 16 na PoKeys
#endif
			}
		}

		if (dsbBuzzerShp && dsbBuzzerShp->is_playing())
		{
			dsbBuzzerShp->stop();
		}
		{
		}

		if (m_distancecounterclear)
		{
			m_distancecounterclear->stop();
		}
	}

	update_sounds_radio();
}

void TTrain::update_sounds_resonancenoise(sound_source &Sound)
{
	// frequency calculation
	auto const normalizer{mvOccupied->Vmax * 0.01f};
	auto const frequency{Sound.m_frequencyoffset + Sound.m_frequencyfactor * mvOccupied->Vel * normalizer};

	// volume calculation
	auto volume = Sound.m_amplitudeoffset + Sound.m_amplitudefactor * std::lerp(mvOccupied->Vel / (1 + mvOccupied->Vmax), 1.0, 0.5); // scale base volume between 0.5-1.0

	if (volume > 0.05)
	{
		Sound.pitch(frequency).gain(volume).play(sound_flags::exclusive | sound_flags::looping);
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
	auto const frequency{Sound.m_frequencyoffset + Sound.m_frequencyfactor * mvOccupied->Vel * normalizer};

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
		Sound.pitch(frequency).gain(volume).play(sound_flags::exclusive | sound_flags::looping);
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
		m_radiomessages.erase(std::remove_if(std::begin(m_radiomessages), std::end(m_radiomessages), [](auto const &source) { return false == source.second->is_playing(); }),
		                      std::end(m_radiomessages));
	}
	// adjust audibility of remaining messages based on current radio conditions
	auto const radioenabled{true == mvOccupied->Radio && (mvOccupied->Power24vIsAvailable || mvOccupied->Power110vIsAvailable)};
	for (auto &message : m_radiomessages)
	{
		auto const volume{true == radioenabled && Dynamic()->Mechanik != nullptr && message.first == RadioChannel() ? m_radiovolume : 0.0};
		message.second->gain(volume);
		radio_message_played |= true == radioenabled && Dynamic()->Mechanik != nullptr && message.first == RadioChannel();
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

void TTrain::radio_message(sound_source *Message, int const Channel)
{

	auto const soundrange{Message->range()};
	if (soundrange > 0 && glm::length2(Message->location() - glm::dvec3{DynamicObject->GetPosition()}) > sq(soundrange))
	{
		// skip message playback if the receiver is outside of the emitter's range
		return;
	}
	// NOTE: we initiate playback of all sounds in range, in case the user switches on the radio or tunes to the right channel mid-play
	m_radiomessages.emplace_back(Channel, std::make_shared<sound_source>(m_radiosound));
	// assign sound to the template and play it
	auto &message = *m_radiomessages.back().second.get();
	auto const radioenabled{true == mvOccupied->Radio && (mvOccupied->Power24vIsAvailable || mvOccupied->Power110vIsAvailable)};
	auto const volume{true == radioenabled && Dynamic()->Mechanik != nullptr && Channel == RadioChannel() ? 1.0 : 0.0};
	message.copy_sounds(*Message).gain(volume).play();
}
