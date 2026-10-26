/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "audio/fmradio.h"

#include "utilities/Globals.h"
#include "utilities/Logs.h"

namespace audio
{

namespace
{

auto const EU07_FMRADIO_BUFFERCOUNT{8}; // number of sound buffers owned by the receiver
auto const EU07_FMRADIO_QUEUELENGTH{6}; // max number of sound buffers waiting for playback. kept short, so the sound follows the knobs without noticeable delay
auto const EU07_FMRADIO_BUFFERSPERSECOND{40}; // inverse of the length of a single sound buffer
// sample rate of generated audio. it has to stay the same for all buffers queued in the emitter, so station audio is converted to it if needed
unsigned int const EU07_FMRADIO_SAMPLERATE{44100};
auto const EU07_FMRADIO_PREBUFFERLENGTH{1.0}; // seconds of station audio required to let it through
auto const EU07_FMRADIO_UNATTENDEDLIMIT{1.0}; // seconds without contact from the owner after which the receiver turns itself off
auto const EU07_FMRADIO_CAPTURERANGE{0.15f}; // MHz, max distance from the station frequency at which the station can be heard
auto const EU07_FMRADIO_CLEANRANGE{0.03f}; // MHz, max distance from the station frequency at which the reception is undisturbed
auto const EU07_FMRADIO_SETTLETIME{0.1}; // seconds the tuning knob has to spend at a station before the receiver connects with its stream
// seconds the connection with a station is kept after the tuning knob moves away from it.
// allows to pass a station, hear where it was and come back to it without waiting for another connection
auto const EU07_FMRADIO_LINGERTIME{8.0};
auto const EU07_FMRADIO_CARRIERNOISE{0.02f}; // fraction of the noise left when the receiver is locked on a station
// the receiver makes a short electric crackle when it locks on a station or lets go of it
auto const EU07_FMRADIO_CRACKLELEVEL{0.25f};
auto const EU07_FMRADIO_CRACKLEDENSITY{0.12f}; // fraction of samples which carry a spike
auto const EU07_FMRADIO_CRACKLEDECAY{0.008f}; // seconds
auto const EU07_FMRADIO_CRACKLELENGTH{0.04f}; // seconds
auto const EU07_FMRADIO_NOISELEVEL{0.3f}; // amplitude of the noise heard between the stations
// level of the receiver signal at the input of the simulated amplifier. modern stations broadcast at full scale most of the time,
// so without some headroom the amplifier and the filters boosting parts of the spectrum would be driven into heavy distortion
auto const EU07_FMRADIO_HEADROOM{0.5f};
auto const EU07_FMRADIO_LIMITERKNEE{0.9f}; // output level above which the peaks are rounded off
// gain of the emitter with the volume knob turned all the way up. regular cab sounds are short and loud,
// while a broadcast played at nominal gain gets lost in the noise of a running vehicle
auto const EU07_FMRADIO_LOUDNESS{3.f};
auto const EU07_FMRADIO_PI{3.14159265358979f};

std::string trim(std::string const &Text)
{
	auto const first{Text.find_first_not_of(" \t\r\n")};
	if (first == std::string::npos)
	{
		return {};
	}
	auto const last{Text.find_last_not_of(" \t\r\n")};
	return Text.substr(first, last - first + 1);
}

} // namespace

// filter designs follow the audio eq cookbook by Robert Bristow-Johnson
fm_radio::biquad fm_radio::biquad::highpass(float const Rate, float const Frequency, float const Q)
{
	auto const omega{2.f * EU07_FMRADIO_PI * std::min(Frequency, Rate * 0.45f) / Rate};
	auto const cosine{std::cos(omega)};
	auto const alpha{std::sin(omega) / (2.f * Q)};
	auto const a0{1.f + alpha};
	biquad filter;
	filter.b0 = (1.f + cosine) * 0.5f / a0;
	filter.b1 = -(1.f + cosine) / a0;
	filter.b2 = filter.b0;
	filter.a1 = -2.f * cosine / a0;
	filter.a2 = (1.f - alpha) / a0;
	return filter;
}

fm_radio::biquad fm_radio::biquad::lowpass(float const Rate, float const Frequency, float const Q)
{
	auto const omega{2.f * EU07_FMRADIO_PI * std::min(Frequency, Rate * 0.45f) / Rate};
	auto const cosine{std::cos(omega)};
	auto const alpha{std::sin(omega) / (2.f * Q)};
	auto const a0{1.f + alpha};
	biquad filter;
	filter.b0 = (1.f - cosine) * 0.5f / a0;
	filter.b1 = (1.f - cosine) / a0;
	filter.b2 = filter.b0;
	filter.a1 = -2.f * cosine / a0;
	filter.a2 = (1.f - alpha) / a0;
	return filter;
}

// Gain is in decibels
fm_radio::biquad fm_radio::biquad::peak(float const Rate, float const Frequency, float const Q, float const Gain)
{
	auto const amplitude{std::pow(10.f, Gain / 40.f)};
	auto const omega{2.f * EU07_FMRADIO_PI * std::min(Frequency, Rate * 0.45f) / Rate};
	auto const cosine{std::cos(omega)};
	auto const alpha{std::sin(omega) / (2.f * Q)};
	auto const a0{1.f + alpha / amplitude};
	biquad filter;
	filter.b0 = (1.f + alpha * amplitude) / a0;
	filter.b1 = -2.f * cosine / a0;
	filter.b2 = (1.f - alpha * amplitude) / a0;
	filter.a1 = filter.b1;
	filter.a2 = (1.f - alpha / amplitude) / a0;
	return filter;
}

void fm_radio::power(bool const State)
{
	std::scoped_lock lock{m_controlmutex};
	m_control.power = State;
	m_control.unattended = 0.0;
}

// sets frequency the receiver is tuned to, in MHz
void fm_radio::tune(float const Frequency)
{
	std::scoped_lock lock{m_controlmutex};
	m_control.frequency = std::clamp(Frequency, EU07_FMRADIO_BANDSTART, EU07_FMRADIO_BANDEND);
}

// sets position of the volume knob, in range 0-1
void fm_radio::volume(float const Volume)
{
	std::scoped_lock lock{m_controlmutex};
	m_control.volume = std::clamp(Volume, 0.f, 1.f);
}

void fm_radio::type(fmradio_type const Type)
{
	std::scoped_lock lock{m_controlmutex};
	m_control.type = Type;
}

// sets location of the receiver in the simulation region space, and audibility parameters at the current position of the listener
void fm_radio::placement(glm::dvec3 const &Location, float const Soundproofing, float const Range)
{
	std::scoped_lock lock{m_controlmutex};
	m_control.location = Location;
	m_control.soundproofing = Soundproofing;
	m_control.range = std::max(1.f, Range);
}

// returns frequency of the first station on the list, or middle of the band if there's none
float fm_radio::default_frequency()
{
	load_stations();
	return m_stations.empty() ? (EU07_FMRADIO_BANDSTART + EU07_FMRADIO_BANDEND) * 0.5f : m_stations.front().frequency;
}

// imports the list of stations. each line of the file holds name of a station, address of its stream and its frequency, separated with semicolons
void fm_radio::load_stations()
{
	std::scoped_lock lock{m_stationsmutex};
	if (m_stationsloaded)
	{
		return;
	}
	m_stationsloaded = true;

	std::ifstream input{Global.fmradio_stations};
	if (false == input.is_open())
	{
		// NOTE: the list isn't a part of the simulator and it's up to the user to provide one, so its absence isn't an error
		WriteLog("fm radio: no station list found at \"" + Global.fmradio_stations + "\", the receiver will pick up only noise", logtype::sound);
		return;
	}
	std::string line;
	auto linenumber{0};
	while (std::getline(input, line))
	{
		++linenumber;
		if (linenumber == 1 && line.rfind("\xEF\xBB\xBF", 0) == 0)
		{
			line.erase(0, 3); // byte order mark
		}
		line = trim(line);
		if (line.empty() || line.front() == '#' || line.rfind("//", 0) == 0)
		{
			continue;
		}
		// NOTE: the address can potentially contain semicolons, so we take the outermost ones as separators
		auto const nameend{line.find(';')};
		auto const frequencystart{line.rfind(';')};
		if (nameend == std::string::npos || frequencystart == nameend)
		{
			ErrorLog("fm radio: malformed entry in line " + std::to_string(linenumber) + " of the station list, expected \"name; address; frequency\"", logtype::sound);
			continue;
		}
		station entry;
		entry.name = trim(line.substr(0, nameend));
		entry.url = trim(line.substr(nameend + 1, frequencystart - nameend - 1));
		auto frequency{trim(line.substr(frequencystart + 1))};
		std::replace(std::begin(frequency), std::end(frequency), ',', '.');
		entry.frequency = static_cast<float>(std::atof(frequency.c_str()));
		if (entry.url.empty() || entry.frequency < EU07_FMRADIO_BANDSTART || entry.frequency > EU07_FMRADIO_BANDEND)
		{
			ErrorLog("fm radio: station \"" + entry.name + "\" in line " + std::to_string(linenumber) + " of the station list has no address or its frequency is outside of 87.5-108 MHz range", logtype::sound);
			continue;
		}
		m_stations.emplace_back(entry);
	}
	WriteLog(m_stations.empty() ? "fm radio: the station list is empty, the receiver will pick up only noise" : "fm radio: " + std::to_string(m_stations.size()) + " station(s) on the list", logtype::sound);
}

// returns index of the station which can be received at specified frequency, or -1
int fm_radio::find_station(float const Frequency) const
{
	auto result{-1};
	auto distance{EU07_FMRADIO_CAPTURERANGE};
	for (std::size_t index{0}; index < m_stations.size(); ++index)
	{
		auto const stationdistance{std::abs(m_stations[index].frequency - Frequency)};
		if (stationdistance < distance)
		{
			distance = stationdistance;
			result = static_cast<int>(index);
		}
	}
	return result;
}

// selects the station to receive, taking into account movement of the tuning knob
void fm_radio::update_tuning(control_block const &Control, double const Deltatime)
{
	auto const candidate{find_station(Control.frequency)};
	if (candidate != m_candidate)
	{
		m_candidate = candidate;
		m_candidatetime = 0.0;
	}
	else
	{
		m_candidatetime += Deltatime;
	}
	// NOTE: connection is made after the knob stays at the station for a moment, to skip stations passed at full speed
	if (candidate != -1 && candidate != m_station && m_candidatetime >= EU07_FMRADIO_SETTLETIME)
	{
		m_station = candidate;
		WriteLog("fm radio: tuned to \"" + m_stations[m_station].name + "\", " + std::to_string(m_stations[m_station].frequency) + " MHz", logtype::sound);
		m_stream.open(m_stations[m_station].url);
		m_buffering = true;
		m_inputphase = 0.0;
		m_inputsamples = {0, 0};
	}
	if (m_station != -1)
	{
		if (m_station == candidate)
		{
			m_lingertime = 0.0;
		}
		else
		{
			// the knob has left the station we were connected with. the connection is kept for a while in case the knob comes back
			m_lingertime += Deltatime;
			if (m_lingertime > EU07_FMRADIO_LINGERTIME)
			{
				m_stream.close();
				m_station = -1;
			}
		}
	}
}

// prepares speaker simulation for specified receiver
void fm_radio::setup_speaker(fmradio_type const Type)
{
	m_speakertype = Type;
	m_speaker.clear();
	auto const rate{static_cast<float>(EU07_FMRADIO_SAMPLERATE)};
	switch (Type)
	{
	case fmradio_type::cheap:
	{
		// tiny speaker in a thin plastic case: no bass to speak of, harsh midrange, overdriven amplifier
		m_speaker.emplace_back(biquad::highpass(rate, 420.f, 0.8f));
		m_speaker.emplace_back(biquad::highpass(rate, 300.f, 0.6f));
		m_speaker.emplace_back(biquad::peak(rate, 1900.f, 1.4f, 7.f));
		m_speaker.emplace_back(biquad::lowpass(rate, 3400.f, 0.9f));
		m_speakerdrive = 2.2f;
		m_speakergain = 0.75f;
		break;
	}
	case fmradio_type::wooden:
	{
		// large speaker in a wooden cabinet: full, slightly boomy lows and soft, rolled off highs
		m_speaker.emplace_back(biquad::highpass(rate, 65.f, 0.7f));
		m_speaker.emplace_back(biquad::peak(rate, 130.f, 0.9f, 5.f));
		m_speaker.emplace_back(biquad::peak(rate, 2400.f, 0.8f, -2.5f));
		m_speaker.emplace_back(biquad::lowpass(rate, 4200.f, 0.6f));
		m_speakerdrive = 1.f;
		m_speakergain = 1.1f;
		break;
	}
	}
}

// fills the staging area with next piece of the receiver output
void fm_radio::generate_chunk(control_block const &Control, std::size_t const Samplecount)
{
	m_chunk.assign(Samplecount, 0);
	m_mix.resize(Samplecount);

	// strength of the station signal at the current position of the tuning knob. it drops with the distance from the station frequency
	auto carrier{0.f};
	if (m_candidate != -1)
	{
		auto const distance{std::abs(m_stations[m_candidate].frequency - Control.frequency)};
		auto const detune{std::clamp((distance - EU07_FMRADIO_CLEANRANGE) / (EU07_FMRADIO_CAPTURERANGE - EU07_FMRADIO_CLEANRANGE), 0.f, 1.f)};
		carrier = 1.f - detune * detune * (3.f - 2.f * detune);
	}
	// the moment the receiver locks on a station or loses it is marked with a crackle.
	// together with the drop of the noise it tells there's a station, even if it's passed too quickly to make out anything else
	// NOTE: the thresholds are set apart, so the receiver doesn't keep thumping when the knob is left right at the edge
	auto const locked{m_locked ? carrier > 0.4f : carrier >= 0.6f};
	if (locked != m_locked)
	{
		m_locked = locked;
		m_thumpposition = 0;
	}

	// station audio
	// NOTE: it's available only if we're connected with the station at the knob, the connection may be still in progress or kept with another station
	auto reception{0.f};
	if (m_station != -1 && m_station == m_candidate)
	{
		auto const streamrate{m_stream.rate()};
		if (m_buffering && streamrate > 0 && m_stream.available() >= static_cast<std::size_t>(streamrate * EU07_FMRADIO_PREBUFFERLENGTH))
		{
			m_buffering = false;
		}
		if (false == m_buffering)
		{
			// station audio is converted to our sample rate with linear interpolation, which is plenty for a radio speaker
			auto const step{static_cast<double>(streamrate) / EU07_FMRADIO_SAMPLERATE};
			m_input.assign(static_cast<std::size_t>(m_inputphase + step * Samplecount), 0);
			if (m_stream.read(m_input.data(), m_input.size()) < m_input.size())
			{
				// we ran out of data. the missing part of the chunk is already silent, and the station is put on hold until the reserve is rebuilt
				m_buffering = true;
			}
			std::size_t inputindex{0};
			for (std::size_t index{0}; index < Samplecount; ++index)
			{
				m_inputphase += step;
				while (m_inputphase >= 1.0)
				{
					m_inputsamples[0] = m_inputsamples[1];
					m_inputsamples[1] = (inputindex < m_input.size() ? m_input[inputindex++] : 0);
					m_inputphase -= 1.0;
				}
				m_chunk[index] = static_cast<std::int16_t>(m_inputsamples[0] + (m_inputsamples[1] - m_inputsamples[0]) * m_inputphase);
			}
			reception = carrier;
		}
	}

	// mix of the station, the noise and the crackle
	auto const receptionstep{(reception - m_reception) / static_cast<float>(Samplecount)};
	auto const carrierstep{(carrier - m_carrier) / static_cast<float>(Samplecount)};
	auto const cracklelength{static_cast<int>(EU07_FMRADIO_CRACKLELENGTH * EU07_FMRADIO_SAMPLERATE)};
	for (std::size_t index{0}; index < Samplecount; ++index)
	{
		// NOTE: the clamps matter, accumulated rounding errors can push the values slightly below zero and the square root further down wouldn't like it
		m_reception = std::clamp(m_reception + receptionstep, 0.f, 1.f);
		m_carrier = std::clamp(m_carrier + carrierstep, 0.f, 1.f);
		// white noise from a xorshift generator
		m_noisestate ^= m_noisestate << 13;
		m_noisestate ^= m_noisestate >> 17;
		m_noisestate ^= m_noisestate << 5;
		auto const noise{static_cast<float>(m_noisestate) * (2.f / 4294967296.f) - 1.f};
		auto signal{m_chunk[index] * (1.f / 32768.f)};
		if (m_reception < 0.999f)
		{
			// an fm receiver which is off the station frequency cuts off peaks of the signal, the further the stronger
			auto const ceiling{0.05f + 0.95f * m_reception};
			signal = std::clamp(signal * (1.f + 2.f * (1.f - m_reception)), -ceiling, ceiling);
		}
		// the noise is pushed down by the station signal itself, whether there's anything to hear on it or not
		auto const noisegain{std::max((1.f - m_carrier) * (1.f - m_carrier), EU07_FMRADIO_CARRIERNOISE)};
		auto sample{signal * std::sqrt(m_reception) + noise * noisegain * EU07_FMRADIO_NOISELEVEL};
		if (m_thumpposition >= 0)
		{
			// sparse spikes of random polarity, fading out quickly
			auto const time{static_cast<float>(m_thumpposition) / EU07_FMRADIO_SAMPLERATE};
			if (std::abs(noise) > 1.f - EU07_FMRADIO_CRACKLEDENSITY)
			{
				sample += (noise < 0.f ? -EU07_FMRADIO_CRACKLELEVEL : EU07_FMRADIO_CRACKLELEVEL) * std::exp(-time / EU07_FMRADIO_CRACKLEDECAY);
			}
			if (++m_thumpposition >= cracklelength)
			{
				m_thumpposition = -1;
			}
		}
		m_mix[index] = sample;
	}
	m_reception = reception;
	m_carrier = carrier;

	// speaker
	if (false == Global.fmradio_cleansound)
	{
		if (m_speaker.empty() || m_speakertype != Control.type)
		{
			setup_speaker(Control.type);
		}
		for (auto &sample : m_mix)
		{
			// amplifier, with soft saturation
			sample = std::tanh(sample * EU07_FMRADIO_HEADROOM * m_speakerdrive);
			for (auto &filter : m_speaker)
			{
				sample = filter.process(sample);
			}
			sample *= m_speakergain;
		}
		// safety net, an invalid value stuck in a filter would silence the receiver for good
		for (auto &filter : m_speaker)
		{
			if (false == std::isfinite(filter.z1) || false == std::isfinite(filter.z2))
			{
				filter.z1 = 0.f;
				filter.z2 = 0.f;
				std::fill(std::begin(m_mix), std::end(m_mix), 0.f);
			}
		}
	}

	for (std::size_t index{0}; index < Samplecount; ++index)
	{
		// peaks which still go over the top are rounded off, as hard clipping sounds much worse
		auto sample{m_mix[index]};
		auto const level{std::abs(sample)};
		if (level > EU07_FMRADIO_LIMITERKNEE)
		{
			auto const limited{EU07_FMRADIO_LIMITERKNEE + (1.f - EU07_FMRADIO_LIMITERKNEE) * std::tanh((level - EU07_FMRADIO_LIMITERKNEE) / (1.f - EU07_FMRADIO_LIMITERKNEE))};
			sample = (sample < 0.f ? -limited : limited);
		}
		m_chunk[index] = static_cast<std::int16_t>(sample * 32767.f);
	}
}

// updates state of the receiver. expects active audio context
void fm_radio::update(double const Deltatime, glm::dvec3 const &Listener)
{
	control_block control;
	{
		std::scoped_lock lock{m_controlmutex};
		if (m_control.unattended > EU07_FMRADIO_UNATTENDEDLIMIT)
		{
			// the owner is gone, or has more important things to do
			m_control.power = false;
		}
		// NOTE: the time is added after the check, so a single long frame doesn't count as a lack of contact
		m_control.unattended += Deltatime;
		control = m_control;
	}

	// pass messages from the stream worker to the log
	radio_stream::message message;
	while (m_stream.fetch_message(message))
	{
		if (message.second)
		{
			ErrorLog(message.first, logtype::sound);
		}
		else
		{
			WriteLog(message.first, logtype::sound);
		}
	}

	if (false == control.power)
	{
		if (m_active)
		{
			m_stream.close();
			reset_emitter();
			m_station = -1;
			m_candidate = -1;
			m_active = false;
		}
		return;
	}
	if (false == m_active)
	{
		// switched on
		load_stations();
		m_reception = 0.f;
		m_carrier = 0.f;
		m_locked = false;
		m_thumpposition = -1;
		m_active = true;
	}
	update_tuning(control, Deltatime);

	if (m_source == audio::null_resource && false == create_emitter())
	{
		return;
	}

	// feed the emitter with generated audio
	auto queuedcount{reclaim_buffers()};
	auto const chunksize{static_cast<std::size_t>(EU07_FMRADIO_SAMPLERATE / EU07_FMRADIO_BUFFERSPERSECOND)};
	while (queuedcount < EU07_FMRADIO_QUEUELENGTH && false == m_sparebuffers.empty())
	{
		generate_chunk(control, chunksize);
		auto const buffer{m_sparebuffers.back()};
		m_sparebuffers.pop_back();
		::alBufferData(buffer, AL_FORMAT_MONO16, m_chunk.data(), static_cast<ALsizei>(m_chunk.size() * sizeof(std::int16_t)), static_cast<ALsizei>(EU07_FMRADIO_SAMPLERATE));
		::alSourceQueueBuffers(m_source, 1, &buffer);
		++queuedcount;
	}

	// location
	auto const offset{glm::vec3{control.location - Listener}};
	::alSourcefv(m_source, AL_POSITION, glm::value_ptr(offset));
	// gain
	// NOTE: the knob position is squared to approximate the response of an audio taper potentiometer
	auto gain{EU07_FMRADIO_LOUDNESS * control.volume * control.volume * control.soundproofing * Global.VehicleVolume};
	auto const rangesquared{control.range * control.range};
	auto const distancesquared{glm::length2(offset)};
	if (distancesquared > rangesquared)
	{
		// outside of nominal hearing range the volume fades out, same way as it does for regular sounds
		auto const fadedistance{control.range * 0.75f};
		gain *= std::lerp(1.f, 0.f, std::clamp((distancesquared - rangesquared) / (fadedistance * fadedistance), 0.f, 1.f));
	}
	::alSourcef(m_source, AL_GAIN, gain);
	::alSourcef(m_source, AL_REFERENCE_DISTANCE, control.range * (1.f / 16.f) * control.soundproofing);

	// playback. the emitter stops on its own when it runs out of queued data, so it may need a restart
	ALint state{AL_INITIAL};
	::alGetSourcei(m_source, AL_SOURCE_STATE, &state);
	if (state != AL_PLAYING && queuedcount > 0)
	{
		::alSourcePlay(m_source);
	}
}

// stops the reception and frees implementation-side resources. expects active audio context
void fm_radio::release()
{
	m_stream.close();
	m_station = -1;
	m_candidate = -1;
	m_active = false;
	if (m_source != audio::null_resource)
	{
		reset_emitter();
		::alDeleteSources(1, &m_source);
		m_source = audio::null_resource;
	}
	if (false == m_buffers.empty())
	{
		::alDeleteBuffers(static_cast<ALsizei>(m_buffers.size()), m_buffers.data());
		m_buffers.clear();
		m_sparebuffers.clear();
	}
}

bool fm_radio::create_emitter()
{
	::alGetError(); // reset the error state, so we don't get blamed for past sins of others

	::alGenSources(1, &m_source);
	if (::alGetError() != AL_NO_ERROR)
	{
		m_source = audio::null_resource;
		if (false == m_emitterfailed)
		{
			ErrorLog("fm radio: failed to create sound emitter", logtype::sound);
			m_emitterfailed = true;
		}
		return false;
	}
	if (m_buffers.empty())
	{
		m_buffers.resize(EU07_FMRADIO_BUFFERCOUNT);
		::alGenBuffers(static_cast<ALsizei>(m_buffers.size()), m_buffers.data());
		if (::alGetError() != AL_NO_ERROR)
		{
			m_buffers.clear();
			::alDeleteSources(1, &m_source);
			m_source = audio::null_resource;
			if (false == m_emitterfailed)
			{
				ErrorLog("fm radio: failed to create sound buffers", logtype::sound);
				m_emitterfailed = true;
			}
			return false;
		}
	}
	m_sparebuffers = m_buffers;
	m_emitterfailed = false;

	::alSourcef(m_source, AL_PITCH, 1.f);
	::alSourcef(m_source, AL_MAX_GAIN, EU07_FMRADIO_LOUDNESS); // by default the gain is capped at 1
	::alSourcef(m_source, AL_ROLLOFF_FACTOR, 1.75f); // matches the setup of regular sounds
	::alSourcei(m_source, AL_LOOPING, AL_FALSE);
	glm::vec3 const zero{0.f};
	::alSourcefv(m_source, AL_VELOCITY, glm::value_ptr(zero));

	return true;
}

// stops the playback and takes back all sound buffers
void fm_radio::reset_emitter()
{
	if (m_source == audio::null_resource)
	{
		return;
	}
	::alSourceStop(m_source);
	::alSourcei(m_source, AL_BUFFER, 0); // detaches all queued buffers
	m_sparebuffers = m_buffers;
}

// collects buffers which were played back. returns: number of buffers still waiting in the queue
int fm_radio::reclaim_buffers()
{
	ALint processedcount{0};
	::alGetSourcei(m_source, AL_BUFFERS_PROCESSED, &processedcount);
	while (processedcount-- > 0)
	{
		ALuint buffer{0};
		::alSourceUnqueueBuffers(m_source, 1, &buffer);
		if (buffer != 0 && m_sparebuffers.size() < m_buffers.size())
		{
			m_sparebuffers.emplace_back(buffer);
		}
	}
	return static_cast<int>(m_buffers.size() - m_sparebuffers.size());
}

} // namespace audio
