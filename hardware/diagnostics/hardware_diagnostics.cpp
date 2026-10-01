/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/diagnostics/hardware_diagnostics.h"

namespace hardware
{

void rate_counter::update( double const Deltatime )
{
	m_window += Deltatime;
	if( m_window < 1.0 )
	{
		return;
	}
	m_rate = static_cast<float>( m_accumulator / m_window );
	m_accumulator = 0;
	m_window = 0.0;
}

} // namespace hardware
