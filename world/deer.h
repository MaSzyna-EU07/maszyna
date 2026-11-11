/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <vector>
#include <glm/glm.hpp>

class TAnimModel;
class TDynamicObject;
namespace scene
{
class basic_region;
}

// deer at the tracks (Global.deer, off unless eu07.ini turns it on): every Global.deer_interval metres the train of the player
// covers, a deer is put (with the chance Global.deer_chance) on the path the train goes along, Global.deer_distance metres ahead
// of it, grazing (the model animals/jelen_je, animated by textures). the horn of the train within Global.deer_hornrange of it, or
// the train coming closer than Global.deer_fleedistance, frightens it: it raises its head (animals/jelen_zryw), looks at the train
// and runs away from the tracks in bounds (animals/jelen_skok, moved by the game along with the hooves), and grazes again where it
// stopped. the game picks the clip and the moment of each of the models (shader vat, vat_random -1), so one hands over to the next
// in the very same pose. the deer go once the train is past them. they aren't parts of the scenery (not saved, not edited)
namespace simulation
{

class deer_herd
{
  public:
	// after the trains moved, Deltatime: the time of the simulation the frame took, seconds
	void update(double const Deltatime);
	// deer about at the moment
	std::size_t count() const
	{
		return m_deer.size();
	}

  private:
	// the ground under the way the deer runs, at equal steps from its start to its end
	static constexpr std::size_t Groundsamples{9};
	enum class deer_state
	{
		grazing, // till it hears the horn or sees the train close
		alarmed, // grazing still, till the moment of the grazing its startle starts from (time)
		startled, // raises its head (since time)
		fleeing, // looks at the train and runs away (since time)
		resting // grazes where it stopped, for good
	};
	struct deer
	{
		TAnimModel *model{nullptr};
		glm::dvec3 start{0.0}; // where it stood when it started to run, or stands
		glm::dvec3 heading{0.0, 0.0, 1.0}; // where its head points, level, of length 1
		deer_state state{deer_state::grazing};
		double frame{0.0}; // of the grazing at the time time
		double time{0.0}; // of the simulation (Timer::GetTime()), seconds; see deer_state
		int clip{0}; // of the startle: the moment of the grazing it starts from
		int look{0}; // the clip of the flight which looks at the train
		double closest{1e9}; // the train came this close, metres
		std::array<double, Groundsamples> ground{}; // heights of the ground along the way it runs
	};
	// the train at Train, its horn sounding, at the time Now: the deer takes fright, runs, or goes (true) once the train is past it
	bool tend(deer &Animal, glm::dvec3 const &Train, bool const Horn, double const Now) const;
	// puts a deer on the path ahead of the train, where it's plain track (no switch) with flat ground beside it and no platform near;
	// false where there's none
	bool spawn(TDynamicObject const &Vehicle);
	// the deer took fright: when it starts, where it looks
	void alarm(deer &Animal, glm::dvec3 const &Train, double const Now) const;
	// the deer starts to run
	void flee(deer &Animal) const;
	// moves a running deer, true when it stopped
	bool run(deer &Animal, double const Now) const;

	std::vector<deer> m_deer;
	double m_travelled{0.0}; // by the train since the last deer, metres
	scene::basic_region const *m_region{nullptr}; // of the scenery the deer were put in
};

extern deer_herd Deer;

} // namespace simulation
