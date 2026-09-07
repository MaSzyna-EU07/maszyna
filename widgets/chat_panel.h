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

// what the participants of a session are saying to one another. opened and closed with the
// key left of 1, and while it is open the text box holds the keyboard, so typing "d" is
// typing "d" rather than opening the doors of whatever you happen to be driving
class chat_panel : public ui_panel
{
  public:
	chat_panel();

	void render_contents() override;

	// called when the panel is opened, so that the cursor lands in the text box
	void focus_input();

  private:
	void submit();

	std::array<char, 256> m_input{};
	bool m_focusrequested{false};
	uint64_t m_lastserial{0};
};

} // namespace ui
