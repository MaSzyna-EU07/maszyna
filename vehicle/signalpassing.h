/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// when a point of the AI's speed table (a signal above all) counts as passed by the train: the test mode's
// "signal passed at danger" and the blackbox record of passed signals (TController::CheckSignalPassed)
namespace signal_passing {

// how far behind the head a point found by the scan of a moving train may lie and still count as one the head
// had ahead [m]. GetDistanceToEvent() finds the distance in steps of a metre from the position of the memory
// cell, so a signal just ahead of the head may come out a little behind it
inline constexpr double moving_tolerance { 5.0 };

// whether a point the scan finds Distance [m] ahead of the head (negative: behind it) counts as one the head had
// ahead. a moving train may be just passing it, so a point up to moving_tolerance behind counts. a standing train
// isn't passing anything: a point found behind its head was passed before (a train set or stopped past a signal),
// and counts only once an update finds it ahead
bool seen_ahead_when_found( double Distance, bool Standing );

// whether the head had the point ahead, after an update finds it Distance [m] ahead of the head
bool seen_ahead_on_update( double Distance, bool Seenahead );

// whether an update of a point reports it as passed: it became elapsed in this update, and the head had it ahead
bool passed_now( bool Elapsedbefore, bool Elapsed, bool Seenahead );

} // namespace signal_passing
