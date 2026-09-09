/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

//---------------------------------------------------------------------------
// Ra: zestaw klas do robienia wskaźników, aby uporządkować nagłówki
//---------------------------------------------------------------------------
module;
#include <cstddef>
#include <stdexcept>

export module eu07.utilities.classes;

export {

namespace plc {
using element_handle = short;
}

namespace scene {
using group_handle = std::size_t;
}

namespace Mtable
{
class TMtableTime; // czas dla danego posterunku
};


enum class TCommandType
{ // binarne odpowiedniki komend w komórce pamięci
    cm_Unknown, // ciąg nierozpoznany (nie jest komendą)
    cm_Ready, // W4 zezwala na odjazd, ale semafor może zatrzymać
    cm_SetVelocity, // prędkość pociągowa zadawana na semaforze
    cm_RoadVelocity, // prędkość drogowa
    cm_SectionVelocity, //ograniczenie prędkości na odcinku
    cm_ShuntVelocity, // prędkość manewrowa na semaforze
    cm_SetProximityVelocity, // informacja wstępna o ograniczeniu
    cm_ChangeDirection,
    cm_PassengerStopPoint,
    cm_OutsideStation,
//    cm_Shunt, // unused?
    cm_EmergencyBrake,
    cm_SecuritySystemMagnet,
    cm_Command // komenda pobierana z komórki
};

using material_handle = int;
using texture_handle = int;

struct invalid_scenery_exception : std::runtime_error {
	invalid_scenery_exception() : std::runtime_error("cannot load scenery") {}
};


}  // export
