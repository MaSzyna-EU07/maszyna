module;

export module eu07.world.station;
import eu07.simcore;
import eu07.world.mtable;
import eu07.utilities.classes;

export {
/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/



// a simple station, performs freight and passenger exchanges with visiting consists
class basic_station {

public:
// methods
    // exchanges load with consist attached to specified vehicle, operating on specified schedule; returns: time needed for exchange, in seconds
    double
        update_load( TDynamicObject *First, Mtable::TTrainParameters &Schedule, int const Platform );
};

namespace simulation {

extern basic_station Station; // temporary object, for station functionality tests

} // simulation


}  // export
