/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/inisettings.h"

#include <algorithm>
#include <format>

namespace ini_settings
{
namespace
{

constexpr std::string_view blanks{" \t"};

// a line of the text: what's on it, and how it ends ("\r\n", "\n", or nothing at the end of the text)
struct line
{
	std::string_view content;
	std::string_view end;
};

std::vector<line> split_lines(std::string_view const Text)
{
	std::vector<line> lines;
	std::size_t start{0};
	while (start < Text.size())
	{
		auto const newline{Text.find('\n', start)};
		auto const next{newline == std::string_view::npos ? Text.size() : newline + 1};
		auto contentend{newline == std::string_view::npos ? Text.size() : newline};
		if (contentend > start && Text[contentend - 1] == '\r')
			--contentend;
		lines.emplace_back(Text.substr(start, contentend - start), Text.substr(contentend, next - contentend));
		start = next;
	}
	return lines;
}

// the first word of a line, a key if the line has one
std::string_view first_token(std::string_view const Content)
{
	auto const begin{Content.find_first_not_of(blanks)};
	if (begin == std::string_view::npos)
		return {};
	auto const end{Content.find_first_of(blanks, begin)};
	return Content.substr(begin, end == std::string_view::npos ? std::string_view::npos : end - begin);
}

// the line of a key, which ends at KeyEnd, with Text as its value; the blanks and the comment after the value stay
std::string with_value(std::string_view const Content, std::size_t const KeyEnd, std::string_view const Text)
{
	auto const valuestart{Content.find_first_not_of(blanks, KeyEnd)};
	// nothing after the key
	if (valuestart == std::string_view::npos)
		return std::format("{} {}", Content.substr(0, KeyEnd), Text);
	auto const comment{Content.find("//", valuestart)};
	// no value, a comment right after the key
	if (comment == valuestart)
		return std::format("{}{} {}", Content.substr(0, valuestart), Text, Content.substr(valuestart));
	auto const valuelast{Content.find_last_not_of(blanks, comment == std::string_view::npos ? std::string_view::npos : comment - 1)};
	return std::format("{}{}{}", Content.substr(0, valuestart), Text, Content.substr(valuelast + 1));
}

// the text put together line by line, the values in place
class writer
{
  public:
	writer(std::vector<value> const &Values, std::string_view const Newline) : m_values(Values), m_found(Values.size(), false), m_newline(Newline) {}

	void add(line const &Line)
	{
		auto const token{first_token(Line.content)};
		if (false == m_ended && token == "endconfig")
		{
			add_missing();
			m_ended = true;
		}
		auto const match{m_ended ? m_values.end() : std::ranges::find(m_values, token, &value::key)};
		if (match == m_values.end())
		{
			m_text += Line.content;
		}
		else
		{
			m_found[static_cast<std::size_t>(match - m_values.begin())] = true;
			auto const keyend{static_cast<std::size_t>(token.data() - Line.content.data()) + token.size()};
			m_text += with_value(Line.content, keyend, match->text);
		}
		m_text += Line.end;
	}

	std::string finish()
	{
		if (false == m_ended)
		{
			if (false == m_text.empty() && false == m_text.ends_with('\n'))
				m_text += m_newline;
			add_missing();
		}
		return std::move(m_text);
	}

  private:
	// the keys the text hasn't got, each in a line of its own
	void add_missing()
	{
		for (std::size_t index{0}; index < m_values.size(); ++index)
		{
			if (m_found[index])
				continue;
			m_text += std::format("{} {}{}", m_values[index].key, m_values[index].text, m_newline);
			m_found[index] = true;
		}
	}

	std::vector<value> const &m_values;
	std::vector<bool> m_found;
	std::string_view m_newline;
	std::string m_text;
	bool m_ended{false}; // past endconfig, after which nothing is read
};

// the length of the UTF-8 sequence a byte starts, 0 if it can't start one
std::size_t sequence_length(unsigned char const Lead)
{
	if (Lead < 0x80U)
		return 1;
	if (Lead >= 0xC2U && Lead <= 0xDFU)
		return 2;
	if (Lead >= 0xE0U && Lead <= 0xEFU)
		return 3;
	if (Lead >= 0xF0U && Lead <= 0xF4U)
		return 4;
	return 0;
}

} // namespace

std::string with_values(std::string_view const Text, std::vector<value> const &Values)
{
	writer text{Values, Text.find("\r\n") != std::string_view::npos ? "\r\n" : "\n"};
	for (auto const &line : split_lines(Text))
		text.add(line);
	return text.finish();
}

bool is_utf8(std::string_view const Text)
{
	std::size_t position{0};
	while (position < Text.size())
	{
		auto const length{sequence_length(static_cast<unsigned char>(Text[position]))};
		if (length == 0 || position + length > Text.size())
			return false;
		for (std::size_t next{1}; next < length; ++next)
		{
			if ((static_cast<unsigned char>(Text[position + next]) & 0xC0U) != 0x80U)
				return false;
		}
		position += length;
	}
	return true;
}

} // namespace ini_settings
