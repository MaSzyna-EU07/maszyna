/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others

*/

// the log of the physics of the train the driver runs

#include "stdafx.h"
#include "vehicle/Driver.h"
#include "physics/mover.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "utilities/Logs.h"
#include "utilities/physicslog.h"
#include "vehicle/Train.h"
#include "vehicle/brakereleaser.h"
#include "vehicle/brakesetting.h"
#include "vehicle/brakingcurve.h"
#include "vehicle/fieldshunting.h"
#include "vehicle/handover.h"
#include "vehicle/signalpassing.h"
#include "vehicle/timetableservice.h"
#include "world/mtable.h"
#include "world/stationnames.h"
#include "vehicle/DynObj.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "world/RoadOrder.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "simulation/simulationtime.h"
#include "simulation/simulationtest.h"
#include "world/Track.h"
#include "world/station.h"
#include "input/keyboardinput.h"
#include <algorithm>
#include <format>
#include <ranges>
#include "vehicle/ai/internal.h"


std::string_view TController::physicslog_header() {
    // the columns of the readers of these logs, the Hasler tape of the starter among them, stay where they are:
    // a new one goes at the end
    return "Time[s] Velocity[m/s] Acceleration[m/ss] "
           "Coupler.Dist[m] Coupler.Force[N] TractionForce[kN] FrictionForce[kN] BrakeForce[kN] "
           "BrakePress[MPa] PipePress[MPa] MotorCurrent[A] "
           "MCP SCP BCP LBP Direction Command CVal1 CVal2 "
           "Security Wheelslip "
           "EngineTemp[Deg] OilTemp[Deg] WaterTemp[Deg] WaterAuxTemp[Deg] "
           "VehicleVelocity[km/h]"
           "\r\n";
}

std::string TController::physicslog_line() const {
    std::ostringstream line;
    line << std::fixed << std::setprecision( 4 )
        << ElapsedTime << " "
        << fabs(11.31 * mvOccupied->WheelDiameter * mvOccupied->nrot) << " "
        << mvControlling->AccS << " "
        << mvOccupied->Couplers[1].Dist << " "
        << mvOccupied->Couplers[1].CForce << " "
        << mvOccupied->Ft << " "
        << mvOccupied->Ff << " "
        << mvOccupied->Fb << " "
        << mvOccupied->BrakePress << " "
        << mvOccupied->PipePress << " "
        << mvControlling->Im << " "
        << int(mvControlling->MainCtrlPos) << " "
        << int(mvControlling->ScndCtrlPos) << " "
        << mvOccupied->fBrakeCtrlPos << " "
        << mvOccupied->LocalBrakePosA << " "
        << int(mvControlling->DirActive) << " "
        << ( mvOccupied->CommandIn.Text.empty() ? "none" : mvOccupied->CommandIn.Text.c_str() ) << " "
        << mvOccupied->CommandIn.Value1 << " "
        << mvOccupied->CommandIn.Value2 << " "
        << int(mvControlling->SecuritySystem.is_blinking()) << " "
        << int(mvControlling->SlippingWheels) << " "
        << mvControlling->diesel.heat.Ts << " "
        << mvControlling->diesel.heat.To << " "
        << mvControlling->diesel.heat.temperatura1 << " "
        << mvControlling->diesel.heat.temperatura2 << " "
        // vehicle speed [km/h], the second column is the wheel speed, which stays at 0 while the wheels are locked in a skid
        << mvOccupied->V * 3.6
        << "\r\n";
    return line.str();
}

void TController::begin_physicslog()
{
    // physicslog/<vehicle>.dat, written every two seconds along with the logs of the other drivers
#if LOGPRESS == 0
    PhysicsLogs.begin( VehicleName, physicslog_header() );
#endif
#if LOGPRESS == 1
    PhysicsLogs.begin( VehicleName, "t\tVel\tAcc\tPP\tVVP\tBP\tBVP\tCVP\n" );
#endif
    PhysicsLogBegun = true;
}

void TController::PhysicsLog()
{ // zapis logu - na razie tylko wypisanie parametrów
#if LOGPRESS == 0
    PhysicsLogs.append( VehicleName, physicslog_line() );
#endif
#if LOGPRESS == 1
    std::ostringstream line;
    line << ElapsedTime << "\t" << fabs(11.31 * mvOccupied->WheelDiameter * mvOccupied->nrot)
         << "\t";
    line << mvControlling->AccS << "\t";
    line << mvOccupied->PipePress << "\t" << mvOccupied->CntrlPipePress << "\t"
         << mvOccupied->BrakePress << "\t";
    line << mvOccupied->Volume << "\t" << mvOccupied->Hamulec->GetCRP() << "\n";
    PhysicsLogs.append( VehicleName, line.str() );
#endif
};
