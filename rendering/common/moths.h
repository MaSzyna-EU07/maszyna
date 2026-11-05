/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <map>
#include <utility>
#include <vector>
#include "rendering/common/locallights.h"

class TAnimModel;

// moths flying about the lit lamps of the scenery. a part of the lamps of the scenery models which shine downwards (Global.moths_share,
// picked by the places of the lamps, so always the same ones) gets a swarm: an instance of the model animals/cmy (animated by textures,
// shader vat: the swarms differ in the moment and the clip of the animation by their places) put in the scenery at the light point.
// the swarm is shown while its lamp shines (the light points the local lights gathered in the frame), in the warm part of the year
// (not in the winter, the air not colder than Global.moths_temperature), to Global.moths_range from the camera. the swarms are made
// when their lamps first come within the range, aren't parts of the scenery (not saved, not edited) and go with it
namespace gfx
{

class moths
{
  public:
	// after the lights of the frame were gathered: makes the swarms of the lamps which came within the range, shows the swarms of the
	// lit lamps and hides the others
	void update(std::vector<local_lights::light> const &Lights, glm::dvec3 const &Camera, unsigned int const Framestamp);
	// forgets the swarms: the scenery they were put in is about to be replaced
	void reset();
	// swarms made, and shown in the last frame
	std::size_t count() const
	{
		return m_count;
	}
	std::size_t shown() const
	{
		return m_shown;
	}

  private:
	struct swarm
	{
		glm::dvec3 position{0.0}; // of the light point the swarm was decided for
		TAnimModel *model{nullptr}; // nullptr: the lamp has no swarm
		unsigned int seen{0}; // the frame the lamp shone last in
	};
	// the lamps met: the instance and the submodel of the light point
	std::map<std::pair<void const *, void const *>, swarm> m_lamps;
	std::vector<glm::dvec3> m_places; // of the swarms made, so two light points of one lamp don't both get one
	std::size_t m_count{0};
	std::size_t m_shown{0};
};

// the swarms of the scenery, kept by the local lights (local_lights::gather(), reset())
extern moths Moths;

} // namespace gfx
