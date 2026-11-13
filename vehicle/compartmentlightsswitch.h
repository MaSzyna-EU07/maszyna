/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the compartment lights get two requests: on (CompartmentLights.is_enabled) and off (is_disabled), and the off one
// wins. A two-state compartmentlights_sw: holds one of them, so turning it to one position withdraws the other request.
// A push type compartmentlights_sw: and the separate compartmentlightson_sw: / compartmentlightsoff_sw: push buttons
// withdraw their own request when they're released, so the other command leaves it alone. A cab with none of these
// controls (the keyboard toggle, ctrl+;, is all there is) acts as if it had the two-state switch, otherwise nothing
// would ever withdraw the off request and the lights couldn't be turned on again.
namespace compartment_lights_switch {

// whether the command for one of the requests also withdraws the other one, from the cab's compartmentlights_sw:
// (whether it's there and whether it's a toggle type switch) and whether the cab has the push button of the other
// request (compartmentlightsoff_sw: for the on command, compartmentlightson_sw: for the off command)
inline bool withdraws_other_request(bool const Hasswitch, bool const Switchistoggle, bool const Hasotherbutton) {
	return Hasswitch ? Switchistoggle : false == Hasotherbutton;
}

} // namespace compartment_lights_switch
