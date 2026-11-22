/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "vehicle/crew.h"

class TDynamicObject;

namespace crew {

// carries out the crew command of an event (see crew.h) on the vehicle it names, or on Activator, the vehicle
// which set the event off, when it names none: puts an AI driver in, the way the scenery does for a vehicle of
// its own with a driver, or takes the AI driver off and leaves the train secured. whatever it does or refuses
// goes to the log. called from the event queue, between the updates of the vehicles
void change(request const &Request, TDynamicObject const *Activator);

} // namespace crew
