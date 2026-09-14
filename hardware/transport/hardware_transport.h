/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <string>

// transport abstraction of the hardware protocol
// the upper layers only ever see a byte stream, which is what lets the same protocol run over
// serial, tcp or rs-485 without changing message semantics
// every method is called from the hardware worker thread only

namespace hardware
{

class hardware_transport
{

public:
// methods
	virtual ~hardware_transport() = default;

	virtual bool connected() const = 0;
	// tries to establish the link, returns true on success
	virtual bool connect() = 0;
	virtual void disconnect() = 0;
	// non-blocking read, returns the number of bytes placed in the buffer or -1 on error
	virtual int read( std::uint8_t *Buffer, std::size_t const Size ) = 0;
	// non-blocking write, returns the number of bytes accepted or -1 on error
	virtual int write( std::uint8_t const *Data, std::size_t const Size ) = 0;

	// human readable transport type, i.e. "Serial"
	virtual std::string kind() const = 0;
	// human readable endpoint, i.e. "COM4 @ 115200"
	virtual std::string endpoint() const = 0;
	// last error reported by the underlying device
	virtual std::string last_error() const = 0;
};

} // namespace hardware
