/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/editorProjectSetup.hpp"

#include <string>

// the fields of the assumptions of the project ("Co rysujesz?"): the step of the wizard of a new scenery, the card of the inspector of the
// tracks of a scenery with none, the window they're changed in later. One screen of them, the defaults chosen, the look under More
namespace setup_ui
{

// the fields; true when one was changed
bool fields(char const *Id, project_setup::setup &Setup);
// "PLK · 100 km/h · mixed traffic · main track · 1435 mm · 60E1", as a line of the summary of them
std::string summary(project_setup::setup const &Setup);
// the name of the profile as the fields show it: PLK, TSI, EN 13803, CH
char const *profile_label(norms::profile_id const Profile);

} // namespace setup_ui
