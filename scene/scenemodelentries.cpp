/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/scenemodelentries.h"

#include "utilities/parser.h"

#include <algorithm>
#include <array>
#include <future>
#include <system_error>
#include <thread>
#include <unordered_map>

namespace scene
{

namespace
{

enum char_kind : unsigned char
{
	char_regular,
	char_separator, // one of the separators the scenery parser splits the text on
	char_special // can make the parser treat the token in a way of its own: a quote, or a part of a comment mark
};

constexpr std::array<unsigned char, 256> make_char_kinds()
{
	std::array<unsigned char, 256> kinds{};
	kinds['\n'] = char_separator;
	kinds['\r'] = char_separator;
	kinds['\t'] = char_separator;
	kinds[' '] = char_separator;
	kinds[';'] = char_separator;
	kinds['"'] = char_special;
	kinds['/'] = char_special;
	return kinds;
}

constexpr std::array<unsigned char, 256> char_kinds{make_char_kinds()};

char to_lower(char const Character)
{
	// only the ascii letters, like the parser does it
	return (Character >= 'A' && Character <= 'Z') ? static_cast<char>(Character - 'A' + 'a') : Character;
}

// compares a token with a lower case keyword the way a token converted by the parser would compare
bool equals(std::string_view const Token, std::string_view const Keyword)
{
	if (Token.size() != Keyword.size())
	{
		return false;
	}
	for (std::size_t idx = 0; idx < Token.size(); ++idx)
	{
		if (to_lower(Token[idx]) != Keyword[idx])
		{
			return false;
		}
	}
	return true;
}

// reads the definitions held by one piece of a text
class entry_reader
{
  public:
	explicit entry_reader(std::string_view const Text) : m_text(Text) {}
	// reads definitions from specified place in the text, until the end of one reaches Limit or something else turns up.
	// NOTE: line breaks are counted from the place the reading starts at
	void read(std::size_t Begin, std::size_t Limit, bool Keywordread);
	// members
	model_entry_block block;
	std::size_t cursor{0}; // end of the last definition read, or the place the reading started at
	std::size_t linebreaks{0}; // line breaks ahead of the cursor
	bool complete{false}; // the reading got as far as the limit

  private:
	// moves past the separators and the line comments. returns: false at a block comment
	bool skip(std::size_t &Position, std::size_t &Linebreaks) const;
	// takes the token at specified place, which has to be past the separators. returns: false if the parser wouldn't take
	// the token the way it's written: there's a quote in it, or a comment starts in its middle
	bool token(std::size_t &Position, std::string_view &Token) const;
	// takes the next token. returns: false if there isn't one which can be used as it's written
	bool next(std::size_t &Position, std::size_t &Linebreaks, std::string_view &Token) const
	{
		return skip(Position, Linebreaks) && token(Position, Token) && false == Token.empty();
	}
	// takes the next token as a number. returns: false unless the parser would convert the token without a stream
	template <typename Type_> bool number(std::size_t &Position, std::size_t &Linebreaks, Type_ &Value) const
	{
		std::string_view text;
		return next(Position, Linebreaks, text) && cParser::parseNumber(text, Value);
	}
	// takes a complete definition. returns: false if what's there isn't one, or isn't one the reader covers
	bool entry(std::size_t &Position, std::size_t &Linebreaks, bool Keywordread, model_entry &Entry);
	std::uint32_t appearance(std::string_view Model, std::string_view Texture);
	// members
	std::string_view m_text;
	std::unordered_map<std::string_view, std::uint32_t> m_appearances; // by the part of the text which names the model and the texture
	std::string_view m_recentkey;
	std::uint32_t m_recentappearance{0};
};

bool entry_reader::skip(std::size_t &Position, std::size_t &Linebreaks) const
{
	auto const *const data{m_text.data()};
	auto const size{m_text.size()};
	while (Position < size)
	{
		auto const character{data[Position]};
		if (char_kinds[static_cast<unsigned char>(character)] == char_separator)
		{
			if (character == '\n')
			{
				++Linebreaks;
			}
			++Position;
			continue;
		}
		if (character == '/' && Position + 1 < size)
		{
			if (data[Position + 1] == '/')
			{
				// the comment takes the rest of the line, along with the line break
				auto const end{m_text.find('\n', Position + 2)};
				if (end == std::string_view::npos)
				{
					Position = size;
				}
				else
				{
					Position = end + 1;
					++Linebreaks;
				}
				continue;
			}
			if (data[Position + 1] == '*')
			{
				return false;
			}
		}
		break;
	}
	return true;
}

bool entry_reader::token(std::size_t &Position, std::string_view &Token) const
{
	auto const *const data{m_text.data()};
	auto const size{m_text.size()};
	auto const begin{Position};
	while (Position < size)
	{
		auto const kind{char_kinds[static_cast<unsigned char>(data[Position])]};
		if (kind == char_separator)
		{
			break;
		}
		if (kind == char_special)
		{
			if (data[Position] == '"')
			{
				return false;
			}
			if (Position + 1 < size && (data[Position + 1] == '/' || data[Position + 1] == '*'))
			{
				return false;
			}
		}
		++Position;
	}
	Token = m_text.substr(begin, Position - begin);
	return true;
}

bool entry_reader::entry(std::size_t &Position, std::size_t &Linebreaks, bool const Keywordread, model_entry &Entry)
{
	std::string_view word;
	if (false == Keywordread)
	{
		if (false == next(Position, Linebreaks, word) || false == equals(word, "node"))
		{
			return false;
		}
		Entry.begin = Position - word.size();
		// the parser takes a token along with the separator which ends it, and the loader notes the line right after
		Entry.line = Linebreaks + ((Position < m_text.size() && m_text[Position] == '\n') ? 1 : 0);
	}
	// NOTE: negative minimal range marks the model as terrain, which is processed in a way of its own
	if (false == number(Position, Linebreaks, Entry.range_max) || false == number(Position, Linebreaks, Entry.range_min) || Entry.range_min < 0.0)
	{
		return false;
	}
	// NOTE: the parser takes include for a directive wherever it shows up
	if (false == next(Position, Linebreaks, Entry.name) || equals(Entry.name, "include"))
	{
		return false;
	}
	if (false == next(Position, Linebreaks, word) || false == equals(word, "model"))
	{
		return false;
	}
	if (false == number(Position, Linebreaks, Entry.location.x) || false == number(Position, Linebreaks, Entry.location.y) || false == number(Position, Linebreaks, Entry.location.z) ||
	    false == number(Position, Linebreaks, Entry.angle))
	{
		return false;
	}
	std::string_view model;
	std::string_view texture;
	if (false == next(Position, Linebreaks, model) || equals(model, "include") || false == next(Position, Linebreaks, texture) || equals(texture, "include"))
	{
		return false;
	}
	while (true)
	{
		if (false == next(Position, Linebreaks, word))
		{
			return false;
		}
		if (equals(word, "endmodel"))
		{
			break;
		}
		if (equals(word, "angles"))
		{
			// NOTE: a block which shows up again is left to the parser, along with the question of what it means
			if (Entry.has_angles || false == number(Position, Linebreaks, Entry.angles.x) || false == number(Position, Linebreaks, Entry.angles.y) || false == number(Position, Linebreaks, Entry.angles.z))
			{
				return false;
			}
			Entry.has_angles = true;
		}
		else if (equals(word, "scale"))
		{
			if (Entry.has_scale || false == number(Position, Linebreaks, Entry.scale.x) || false == number(Position, Linebreaks, Entry.scale.y) || false == number(Position, Linebreaks, Entry.scale.z))
			{
				return false;
			}
			Entry.has_scale = true;
		}
		else if (equals(word, "notransition"))
		{
			Entry.notransition = true;
		}
		else
		{
			return false;
		}
	}
	Entry.end = Position;
	Entry.appearance = appearance(model, texture);
	return true;
}

std::uint32_t entry_reader::appearance(std::string_view const Model, std::string_view const Texture)
{
	// the two tokens along with whatever is between them. the same pair written in another way gets an entry of its own
	std::string_view const key{Model.data(), static_cast<std::size_t>((Texture.data() + Texture.size()) - Model.data())};
	if (key == m_recentkey)
	{
		return m_recentappearance;
	}
	auto lookup{m_appearances.find(key)};
	if (lookup == m_appearances.end())
	{
		model_appearance appearance;
		appearance.model.assign(Model);
		std::transform(std::begin(appearance.model), std::end(appearance.model), std::begin(appearance.model), [](char const Character) { return Character == '\\' ? '/' : to_lower(Character); });
		appearance.texture.assign(Texture);
		std::replace(std::begin(appearance.texture), std::end(appearance.texture), '\\', '/');
		block.appearances.emplace_back(std::move(appearance));
		lookup = m_appearances.emplace(key, static_cast<std::uint32_t>(block.appearances.size() - 1)).first;
	}
	m_recentkey = key;
	m_recentappearance = lookup->second;
	return lookup->second;
}

void entry_reader::read(std::size_t const Begin, std::size_t const Limit, bool Keywordread)
{
	cursor = Begin;
	linebreaks = 0;
	while (cursor < Limit)
	{
		auto position{cursor};
		auto breaks{linebreaks};
		model_entry definition;
		if (false == entry(position, breaks, Keywordread, definition))
		{
			break;
		}
		Keywordread = false;
		block.entries.emplace_back(definition);
		cursor = position;
		linebreaks = breaks;
	}
	complete = (cursor >= Limit);
}

} // namespace

model_entry_run read_model_entries(std::string_view const Text, bool const Keywordread, model_reader_setup const &Setup)
{
	model_entry_run run;

	entry_reader lead{Text};
	lead.read(0, std::min(Text.size(), Setup.lead), Keywordread);
	if (lead.block.entries.empty())
	{
		return run;
	}
	auto cursor{lead.cursor};
	auto linebreaks{lead.linebreaks};
	run.blocks.emplace_back(std::move(lead.block));

	if (lead.complete && cursor < Text.size())
	{
		// the run goes on: the next part of the text is divided between the threads. a thread needs a place to start at,
		// which is found by looking for the keyword ending a definition. whether the place really is an end of a definition
		// is known only when the thread reading the part ahead of it gets there, so the pieces are put together as long as
		// each one ended right where the next one began. if that's not the case, e.g. the place found was inside a comment,
		// the pieces past it are dropped and their text is read again by the next call, starting from a place which is certain
		auto const limit{cursor + std::min(Text.size() - cursor, std::max<std::size_t>(Setup.window, 1))};
		auto const threads{std::clamp<std::size_t>(Setup.threads != 0 ? Setup.threads : std::thread::hardware_concurrency(), 1, 16)};
		auto const count{std::clamp<std::size_t>((limit - cursor) / std::max<std::size_t>(Setup.share, 1), 1, threads)};
		std::string_view const endmark{"endmodel"};
		std::vector<std::size_t> bounds{cursor};
		for (std::size_t idx = 1; idx < count; ++idx)
		{
			auto const mark{Text.find(endmark, cursor + (limit - cursor) / count * idx)};
			if (mark == std::string_view::npos || mark + endmark.size() >= limit)
			{
				break;
			}
			if (mark + endmark.size() > bounds.back())
			{
				bounds.emplace_back(mark + endmark.size());
			}
		}
		bounds.emplace_back(limit);

		std::vector<entry_reader> readers(bounds.size() - 1, entry_reader{Text});
		{
			std::vector<std::future<void>> tasks;
			for (std::size_t idx = 1; idx < readers.size(); ++idx)
			{
				auto const work{[&readers, &bounds, idx]() { readers[idx].read(bounds[idx], bounds[idx + 1], false); }};
				try
				{
					tasks.emplace_back(std::async(std::launch::async, work));
				}
				catch (std::system_error const &)
				{
					// no thread to be had, the piece is read right here
					work();
				}
			}
			readers.front().read(bounds[0], bounds[1], false);
			for (auto &task : tasks)
			{
				task.get();
			}
		}
		for (std::size_t idx = 0; idx < readers.size(); ++idx)
		{
			auto &reader{readers[idx]};
			for (auto &definition : reader.block.entries)
			{
				definition.line += linebreaks;
			}
			cursor = reader.cursor;
			linebreaks += reader.linebreaks;
			if (false == reader.block.entries.empty())
			{
				run.blocks.emplace_back(std::move(reader.block));
			}
			if (reader.cursor != bounds[idx + 1])
			{
				break;
			}
		}
	}
	run.length = cursor;
	run.linebreaks = linebreaks;

	return run;
}

} // namespace scene

//---------------------------------------------------------------------------
