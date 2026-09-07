/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <deque>
#include <string>

#include "network/entities.h"

namespace network
{

// the longest thing anybody may say at once. a line, not an essay - and a cap the server
// applies to what it receives rather than trusting the sender to have applied it
constexpr size_t CHAT_LENGTH_LIMIT = 200;
// how much of the conversation is kept
constexpr size_t CHAT_HISTORY = 100;

struct chat_line
{
	PeerId author{PEER_NONE};
	// resolved when the line arrives: a peer may be gone by the time it is read back
	std::string name;
	std::string text;
	// simulation clock, for the timestamp in front of the line
	double time{0.0};
};

// puts a line in the log, whoever it came from. also writes it to the ordinary log, which
// is what a headless server has instead of a chat window
void note_chat(PeerId Author, std::string const &Text);

std::deque<chat_line> const &chat_log();

// how many lines have arrived since the session started. the panel watches this to know
// when to scroll, and the rest of the ui to know there is something new
uint64_t chat_serial();

void clear_chat();

// trims and caps a line, and says whether anything worth sending is left
std::string tidy_chat(std::string const &Raw);

} // namespace network
