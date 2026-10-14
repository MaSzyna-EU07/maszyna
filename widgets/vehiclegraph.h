/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/uilayer.h"

namespace ui
{

// inspector of the graph of the occupied vehicle, when it's defined by a .vhg file: values of the ports and the internal
// states of the blocks, live, with forcing of the signal values
class vehiclegraph_panel : public ui_panel
{
public:
	vehiclegraph_panel();

	void render_contents() override;

private:
	std::string m_filter;
	char m_filterbuffer[64]{};
	double m_forcevalue{0.0};
};

} // namespace ui
