/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/signalpassing.h"

namespace signal_passing {

bool seen_ahead_when_found( double const Distance, bool const Standing ) {

    return Distance >= ( Standing ? 0.0 : -moving_tolerance );
}

bool seen_ahead_on_update( double const Distance, bool const Seenahead ) {

    return Seenahead || Distance >= 0.0;
}

bool passed_now( bool const Elapsedbefore, bool const Elapsed, bool const Seenahead ) {

    return Elapsed && !Elapsedbefore && Seenahead;
}

} // namespace signal_passing
