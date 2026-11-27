/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "audio/audio.h"
#include "audio/radiostream.h"

namespace audio
{

float const EU07_FMRADIO_BANDSTART{87.5f}; // MHz
float const EU07_FMRADIO_BANDEND{108.f}; // MHz

// kind of the receiver, determines character of its sound
enum class fmradio_type
{
	cheap, // pocket-sized plastic receiver with a tiny speaker
	wooden // old table receiver with a large speaker in a wooden cabinet
};

// broadcast radio receiver installed in a vehicle cab.
// the stations are internet radio streams assigned to frequencies of the fm band in a text file.
// the sound is generated piece by piece from the stream and synthesized noise, then played through a positional emitter
// owned by the receiver, as this doesn't fit the regular buffer-based sound pipeline
class fm_radio
{

  public:
	// constructors
	fm_radio() = default;
	// methods
	// owner-side interface, to be called by the vehicle the receiver is installed in.
	// NOTE: the owner is expected to call power() on regular basis, a receiver left without supervision turns itself off
	void power(bool const State);
	// sets frequency the receiver is tuned to, in MHz
	void tune(float const Frequency);
	// sets position of the volume knob, in range 0-1
	void volume(float const Volume);
	void type(fmradio_type const Type);
	// sets location of the receiver in the simulation region space, and audibility parameters at the current position of the listener
	void placement(glm::dvec3 const &Location, float const Soundproofing, float const Range);
	// returns frequency of the first station on the list, or middle of the band if there's none
	float default_frequency();
	// renderer-side interface
	// updates state of the receiver. expects active audio context
	void update(double const Deltatime, glm::dvec3 const &Listener);
	// stops the reception and frees implementation-side resources. expects active audio context
	void release();

  private:
	// types
	struct control_block
	{
		bool power{false};
		float frequency{EU07_FMRADIO_BANDSTART};
		float volume{0.f};
		fmradio_type type{fmradio_type::cheap};
		glm::dvec3 location{0.0};
		float soundproofing{1.f};
		float range{15.f};
		double unattended{0.0}; // time since the last call from the owner
	};
	struct station
	{
		std::string name;
		std::string url;
		float frequency{0.f}; // MHz
	};
	// second order filter section
	struct biquad
	{
		float b0{1.f}, b1{0.f}, b2{0.f}, a1{0.f}, a2{0.f};
		float z1{0.f}, z2{0.f};

		float process(float const Input)
		{
			auto const output{b0 * Input + z1};
			z1 = b1 * Input - a1 * output + z2;
			z2 = b2 * Input - a2 * output;
			return output;
		}
		static biquad highpass(float const Rate, float const Frequency, float const Q);
		static biquad lowpass(float const Rate, float const Frequency, float const Q);
		static biquad peak(float const Rate, float const Frequency, float const Q, float const Gain);
	};
	// methods
	void load_stations();
	// returns index of the station which can be received at specified frequency, or -1
	int find_station(float const Frequency) const;
	// selects the station to receive, taking into account movement of the tuning knob
	void update_tuning(control_block const &Control, double const Deltatime);
	// prepares speaker simulation for specified receiver
	void setup_speaker(fmradio_type const Type);
	// fills the staging area with next piece of the receiver output
	void generate_chunk(control_block const &Control, std::size_t const Samplecount);
	bool create_emitter();
	// stops the playback and takes back all sound buffers
	void reset_emitter();
	// collects buffers which were played back. returns: number of buffers still waiting in the queue
	int reclaim_buffers();
	// members
	std::mutex m_controlmutex;
	control_block m_control; // settings requested by the owner
	std::mutex m_stationsmutex;
	std::vector<station> m_stations;
	bool m_stationsloaded{false};
	radio_stream m_stream;
	bool m_active{false}; // the receiver is switched on
	int m_station{-1}; // station the stream is connected with
	int m_candidate{-1}; // station at the current position of the tuning knob
	double m_candidatetime{0.0}; // time the knob has spent at the candidate station
	double m_lingertime{0.0}; // time since the knob has left the station the stream is connected with
	bool m_buffering{true}; // station audio is held until there's enough data to keep it going
	float m_reception{0.f}; // current quality of the station audio, 0-1
	float m_carrier{0.f}; // current strength of the station signal, 0-1
	bool m_locked{false}; // the receiver is locked on a station signal
	int m_thumpposition{-1}; // progress of the sound made when the receiver locks on a station or loses it, in samples. negative if the sound isn't playing
	double m_inputphase{0.0}; // sample rate conversion, position between two most recent samples of station audio
	std::array<std::int16_t, 2> m_inputsamples{0, 0}; // sample rate conversion, two most recent samples of station audio
	fmradio_type m_speakertype{fmradio_type::cheap};
	std::vector<biquad> m_speaker; // filters simulating the speaker and the case
	float m_speakerdrive{1.f}; // signal level at the input of the simulated amplifier
	float m_speakergain{1.f}; // make-up gain
	std::uint32_t m_noisestate{0x2545f491}; // state of the noise generator
	bool m_emitterfailed{false}; // helper, prevents flooding the log with the same error
	ALuint m_source{audio::null_resource};
	std::vector<ALuint> m_buffers; // all sound buffers owned by the receiver
	std::vector<ALuint> m_sparebuffers; // sound buffers which aren't queued for playback
	std::vector<std::int16_t> m_chunk; // helper, staging area for audio data
	std::vector<std::int16_t> m_input; // helper, staging area for audio data
	std::vector<float> m_mix; // helper, staging area for audio data
};

} // namespace audio
