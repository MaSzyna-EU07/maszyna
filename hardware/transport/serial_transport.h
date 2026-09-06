/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <libserialport.h>

#include <string>
#include <vector>

#include "hardware/transport/hardware_transport.h"

// serial port transport, one connection per port, as described by the specification
// the port is opened and serviced exclusively from the hardware worker thread

namespace hardware
{

class serial_transport : public hardware_transport
{

public:
// methods
	serial_transport( std::string Port, int const Baud );
	~serial_transport() override;

	bool connected() const override { return m_port != nullptr; }
	bool connect() override;
	void disconnect() override;
	int read( std::uint8_t *Buffer, std::size_t const Size ) override;
	int write( std::uint8_t const *Data, std::size_t const Size ) override;

	std::string kind() const override { return "Serial"; }
	std::string endpoint() const override;
	std::string last_error() const override { return m_lasterror; }

private:
// members
	std::string m_portname;
	int m_baud = 115200;
	sp_port *m_port = nullptr;
	std::string m_lasterror;
};

// list of serial ports currently present in the system
std::vector<std::string> list_serial_ports();

} // namespace hardware
