/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <string_view>
#include <vector>

// the files the log is written to, and which of them a run clears when it starts. kept apart from the log service so
// the unit tests can check them

// the log of everything, and the list of the errors (made at the first error)
enum class log_file { log, errors };

// the name of a log file: the shared one in the game folder, or with multiplelogs the run's own in logs/, named after
// the scenery and the time
std::string log_file_name(log_file Kind, bool MultipleLogs, std::string_view Scenery, std::string_view Date);

// the shared log files a run deletes when it starts: the ones it writes itself, that is without multiplelogs, so an
// errors.txt of an earlier run doesn't stay behind when this one logs no error. with multiplelogs the run writes files
// of its own in logs/, and the log.txt / errors.txt of the last ordinary run are kept
std::vector<std::string> stale_log_files(bool MultipleLogs);
