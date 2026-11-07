/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodebuildtools.h"

void editor_mode::make_build_tools()
{
	editor_build_services &build{*this};
	m_build_tools = std::make_unique<build_tools>(build);
	// nothing selected in the road window at the start
	m_build_tools->road.road_select(nullptr);
}

road_tool &editor_mode::road_state()
{
	return m_roadtool;
}

std::vector<double> editor_mode::ground_heights_at(std::vector<glm::dvec3> const &Points, bool const Fresh, std::vector<char> *Found) const
{
	return ground_heights(Points, Fresh, Found);
}
