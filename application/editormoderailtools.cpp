/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormoderailtools.h"

void editor_mode::make_rail_tools()
{
	m_rail_tools = std::make_unique<rail_tools>();
}
