/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/logfiles.h"

#include <format>

std::string log_file_name(log_file const Kind, bool const MultipleLogs, std::string_view const Scenery, std::string_view const Date)
{
	std::string_view const name{Kind == log_file::errors ? "errors" : "log"};
	return MultipleLogs ? std::format("logs/{} ({}) {}.txt", name, Scenery, Date) : std::format("{}.txt", name);
}

std::vector<std::string> stale_log_files(bool const MultipleLogs)
{
	if (MultipleLogs)
		return {};
	return {log_file_name(log_file::log, false, {}, {}), log_file_name(log_file::errors, false, {}, {})};
}
