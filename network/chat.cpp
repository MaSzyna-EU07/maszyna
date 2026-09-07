/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "network/chat.h"

#include "utilities/Globals.h"
#include "utilities/Logs.h"

namespace
{

std::deque<network::chat_line> g_log;
uint64_t g_serial{0};

} // namespace

std::string network::tidy_chat(std::string const &Raw)
{
	std::string text;
	text.reserve(Raw.size());

	bool space{false};
	for (char const character : Raw)
	{
		unsigned char const value = (unsigned char)character;
		// a newline would let one line pretend to be several, including one that looks
		// like it came from somebody else
		if (value < 0x20 || value == 0x7f)
			continue;

		if (character == ' ')
		{
			space = !text.empty();
			continue;
		}

		if (space)
		{
			text += ' ';
			space = false;
		}

		text += character;
		if (text.size() >= CHAT_LENGTH_LIMIT)
			break;
	}

	return text;
}

void network::note_chat(PeerId const Author, std::string const &Text)
{
	auto const text = tidy_chat(Text);
	if (text.empty())
		return;

	chat_line line;
	line.author = Author;
	line.name = Peers.name_of(Author);
	line.text = text;
	line.time = Global.fTimeAngleDeg;

	g_log.emplace_back(line);
	while (g_log.size() > CHAT_HISTORY)
		g_log.pop_front();

	++g_serial;

	// on a machine running without a window this is the only place the conversation shows
	WriteLog("chat: <" + line.name + "> " + line.text, logtype::net);
}

std::deque<network::chat_line> const &network::chat_log()
{
	return g_log;
}

uint64_t network::chat_serial()
{
	return g_serial;
}

void network::clear_chat()
{
	g_log.clear();
	g_serial = 0;
}
