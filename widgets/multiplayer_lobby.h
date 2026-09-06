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
#include "network/entities.h"

namespace ui
{

// lobby of a multiplayer session: shows the vehicle roster published by the server and
// lets the player pick the one to work on. it lives inside the driver mode rather than in
// a mode of its own on purpose - the world has to keep running while somebody is choosing,
// and the very same screen is what a player uses to change vehicles later on
class multiplayer_lobby_panel : public ui_panel
{
public:
	multiplayer_lobby_panel();

	void render_contents() override;

private:
	void claim(network::NetworkEntityId const Entity, network::NetworkEntityId const Current);
	std::string describe_crew(network::NetworkEntityId const Entity) const;
};

} // namespace ui
