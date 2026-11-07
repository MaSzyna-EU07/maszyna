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

rail_tools::rail_tools(editor_services &Services, editor_rail_services &Rail)
    : speed{Services, Services, Services, Services}, srk{Services, Services, Services, Services, Services}, km{Services, Services, Services, Services}
{
}

void editor_mode::make_rail_tools()
{
	editor_services &services{*this};
	editor_rail_services &rail{*this};
	m_rail_tools = std::make_unique<rail_tools>(services, rail);
	m_tools.insert(m_tools.end(), {&m_rail_tools->speed, &m_rail_tools->srk, &m_rail_tools->km});
}
