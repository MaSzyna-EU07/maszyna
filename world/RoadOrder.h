/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>

#include <glm/glm.hpp>

#include "world/Event.h"

// an order for the drivers of road vehicles heading for a place they may have to wait at: a closed level crossing,
// a junction they have to give way at. to them it's what a signal is to the driver of a train. it doesn't go to
// the event manager and never runs: the drivers find it on the lane they scan, and ask it what speed it allows
class road_order : public basic_event
{

  public:
	road_order()
	{
		m_passive = true;
	}
	void init() override {}
	std::string input_text() const override
	{
		return "SetVelocity";
	}
	TCommandType input_command() const override
	{
		return TCommandType::cm_SetVelocity;
	}
	double input_value(int const Index) const override
	{
		return m_velocity;
	}
	glm::dvec3 input_location() const override
	{
		return m_location;
	}
	// members
	double m_velocity{-1.0}; // 0: stop, -1: no limit
	glm::dvec3 m_location{0.0};

  private:
	std::string type() const override
	{
		return "roadorder";
	}
	void deserialize_(cParser &Input, scene::scratch_data &Scratchpad) override {}
	void run_() override {}
	void export_as_text_(std::ostream &Output) const override {}
};

//---------------------------------------------------------------------------
