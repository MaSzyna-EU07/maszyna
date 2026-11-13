/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the field shunting (ScndCtrlPos) of the diesel-electric locomotives
namespace fieldshunting {

// who sets the field shunting
enum class control : int {
	manual, // the driver, with the second controller
	automatic, // the shunting relays (RelayType)
	off // nobody: the shunting is switched off, the field stays full
};

// who sets the field shunting of a diesel-electric locomotive, by its AutoRelay (Autorelaytype: 0 No, 1 Yes,
// 2 Optional) and its AutoRelayFlag (Autorelayflag). with the flag set, the relays. AutoRelay=Optional is a switch
// that lets the relays shunt the field, as the SU45's and SP45's "załączenie bocznikowania" (eu07/maszyna#167):
// switched off, nobody. otherwise the driver
control diesel_electric(int Autorelaytype, bool Autorelayflag);

} // namespace fieldshunting
