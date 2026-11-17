/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scripting/scriptscreenrules.h"

#include "utilities/parser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <vector>

namespace script_screens
{

void screen_config::deserialize(cParser &Input)
{
	while (deserialize_mapping(Input))
	{
		// all work done by while()
	}
}

bool screen_config::deserialize_mapping(cParser &Input)
{
	// token can be a key or block end
	auto const key{Input.getToken<std::string>(true, "\n\r\t  ,;[]")};

	if (key.empty() || key == "}")
	{
		return false;
	}

	if (key == "{")
	{
		script = Input.getToken<std::string>();
	}
	else if (key == "target:")
	{
		target = Input.getToken<std::string>();
	}
	else if (key == "frames:")
	{
		frames = Input.getToken<int>();
	}
	else if (key == "parameters:")
	{
		parameters = Input.getToken<std::string>();
	}
	else if (key == "instance:")
	{
		instancing = Input.getToken<std::string>();
	}
	else if (key == "background:")
	{
		background = Input.getToken<std::string>();
	}
	else
	{
		// a key of a later version: skip its value
		Input.getToken<std::string>();
	}

	return true;
}

frame_schedule::frame_schedule(std::uint64_t const Start, int const Interval, int const Phase)
    : m_interval{std::max(Interval, 1)}, m_next{Start + static_cast<std::uint64_t>(std::max(Phase, 0))}
{
}

int frame_schedule::effective_interval(int const Configured, int const Requested)
{
	return std::max({Configured, Requested, 1});
}

int frame_schedule::phase(std::string_view const Key, int const Interval)
{
	if (Interval <= 1)
	{
		return 0;
	}
	// FNV-1a: the same offset for the same name on every compiler
	std::uint32_t hash{2166136261u};
	for (auto const character : Key)
	{
		hash ^= static_cast<std::uint8_t>(character);
		hash *= 16777619u;
	}
	return static_cast<int>(hash % static_cast<std::uint32_t>(Interval));
}

bool frame_schedule::poll(std::uint64_t const Frame)
{
	if (Frame < m_next)
	{
		return false;
	}
	m_next = Frame + static_cast<std::uint64_t>(m_interval);
	return true;
}

bool is_safe_relative_path(std::string_view const Path)
{
	if (Path.empty() || Path.front() == '/' || Path.front() == '\\' || Path.find(':') != std::string_view::npos)
	{
		return false;
	}
	std::size_t start{0};
	while (start <= Path.size())
	{
		auto const end{std::min(Path.find_first_of("/\\", start), Path.size())};
		if (Path.substr(start, end - start) == "..")
		{
			return false;
		}
		start = end + 1;
	}
	return true;
}

std::optional<std::string> screen_script_path(std::string const &Folder, std::string const &Script)
{
	// as substr_path() tells a folder
	auto path{Script.find('/') == std::string::npos ? Folder + Script : Script};
	if (false == is_safe_relative_path(path))
	{
		return std::nullopt;
	}
	return path;
}

namespace
{

// the end of the set that starts at Start ('['), past its ']'; Pattern.size() for a set that doesn't end. as Lua's
// classEnd: a ']' right after '[' or '[^' is in the set, and '%' escapes the character after it
std::size_t set_end(std::string_view const Pattern, std::size_t Start)
{
	auto idx{Start + 1};
	if (idx < Pattern.size() && Pattern[idx] == '^')
	{
		++idx;
	}
	do
	{
		if (idx >= Pattern.size())
		{
			return Pattern.size();
		}
		if (Pattern[idx++] == '%')
		{
			++idx;
		}
	} while (idx >= Pattern.size() || Pattern[idx] != ']');
	return idx + 1;
}

// one item of a pattern, as Lua's matcher takes them
struct pattern_item
{
	enum class kind
	{
		single, // a character, '.', a %class or a [set], with its quantifier if any
		paren, // '(' or ')': a capture costs nothing
		scan, // %bxy or a back reference %1-%9: goes along the subject
		frontier, // %f[set]: a test of one position
		end_anchor // '$' that ends the pattern
	};
	kind type{kind::single};
	std::size_t begin{0}; // of a single, without its quantifier
	std::size_t end{0};
	char quantifier{0}; // '*', '+', '-', '?' or none
};

std::vector<pattern_item> pattern_items(std::string_view const Pattern, std::size_t idx)
{
	std::vector<pattern_item> items;
	while (idx < Pattern.size())
	{
		auto const character{Pattern[idx]};
		if (character == '(' || character == ')')
		{
			items.push_back({pattern_item::kind::paren, idx, idx + 1});
			++idx;
			continue;
		}
		if (character == '$' && idx + 1 == Pattern.size())
		{
			items.push_back({pattern_item::kind::end_anchor, idx, idx + 1});
			break;
		}
		std::size_t end{idx + 1};
		if (character == '%' && idx + 1 < Pattern.size())
		{
			auto const next{Pattern[idx + 1]};
			if (next == 'b')
			{
				items.push_back({pattern_item::kind::scan, idx, idx + 4});
				idx += 4;
				continue;
			}
			if (next == 'f')
			{
				auto const after{idx + 2 < Pattern.size() && Pattern[idx + 2] == '[' ? set_end(Pattern, idx + 2) : idx + 2};
				items.push_back({pattern_item::kind::frontier, idx, after});
				idx = after;
				continue;
			}
			if (next >= '0' && next <= '9')
			{
				items.push_back({pattern_item::kind::scan, idx, idx + 2});
				idx += 2;
				continue;
			}
			end = idx + 2;
		}
		else if (character == '[')
		{
			end = set_end(Pattern, idx);
		}
		pattern_item item{pattern_item::kind::single, idx, end};
		if (end < Pattern.size() && (Pattern[end] == '*' || Pattern[end] == '+' || Pattern[end] == '-' || Pattern[end] == '?'))
		{
			item.quantifier = Pattern[end];
			++end;
		}
		items.push_back(item);
		idx = end;
	}
	return items;
}

// when the rest of a pattern matches, once the matcher gets to it
enum class rest_match
{
	always, // from any position: whatever comes before it is tried once
	at_end, // at least at the end of the subject
	maybe
};

rest_match match_with(std::string_view const Pattern, pattern_item const &Item, rest_match const Rest)
{
	switch (Item.type)
	{
	case pattern_item::kind::paren:
		return Rest;
	case pattern_item::kind::end_anchor:
		return rest_match::at_end;
	case pattern_item::kind::single:
		if (Item.quantifier == '*' || Item.quantifier == '-' || Item.quantifier == '?')
		{
			// matches nothing as well; '.' repeated goes on to the end of the subject
			auto const dot{Item.quantifier != '?' && Item.end - Item.begin == 1 && Pattern[Item.begin] == '.'};
			return dot && Rest == rest_match::at_end ? rest_match::always : Rest;
		}
		return rest_match::maybe;
	default:
		return rest_match::maybe;
	}
}

// whether Character is in the %class Class, as Lua's match_class
bool in_class(unsigned char const Character, unsigned char const Class)
{
	bool in{false};
	switch (std::tolower(Class))
	{
	case 'a': in = std::isalpha(Character) != 0; break;
	case 'c': in = std::iscntrl(Character) != 0; break;
	case 'd': in = std::isdigit(Character) != 0; break;
	case 'l': in = std::islower(Character) != 0; break;
	case 'p': in = std::ispunct(Character) != 0; break;
	case 's': in = std::isspace(Character) != 0; break;
	case 'u': in = std::isupper(Character) != 0; break;
	case 'w': in = std::isalnum(Character) != 0; break;
	case 'x': in = std::isxdigit(Character) != 0; break;
	case 'z': in = Character == 0; break;
	default: return Class == Character;
	}
	return std::isupper(Class) != 0 ? !in : in;
}

// the longest run of the subject that the single item (a character, '.' or a %class) matches; the whole subject for a
// set, which isn't gone through here
std::size_t longest_run(std::string_view const Pattern, pattern_item const &Item, std::string_view const Subject)
{
	auto const single{Pattern.substr(Item.begin, Item.end - Item.begin)};
	if (single == "." || single.front() == '[')
	{
		return Subject.size();
	}
	std::size_t longest{0};
	std::size_t run{0};
	for (auto const character : Subject)
	{
		auto const c{static_cast<unsigned char>(character)};
		auto const in{single.size() == 2 ? in_class(c, static_cast<unsigned char>(single[1])) : c == static_cast<unsigned char>(single[0])};
		run = in ? run + 1 : 0;
		longest = std::max(longest, run);
	}
	return longest;
}

double pattern_cost(std::string_view const Pattern, std::size_t const Size, std::string_view const *Subject, bool const Plain, bool const Anchorable)
{
	auto const length{static_cast<double>(std::max<std::size_t>(Pattern.size(), 1))};
	auto const positions{static_cast<double>(Size) + 1.0};
	// LuaJIT searches plainly for a pattern without the special characters as well
	if (Plain || Pattern.find_first_of("^$*+?.([%-") == std::string_view::npos)
	{
		return positions * length;
	}
	auto const anchored{Anchorable && Pattern.front() == '^'};
	auto const items{pattern_items(Pattern, anchored ? 1U : 0U)};
	auto cost{anchored ? 1.0 : positions};
	// from the end: what follows each item, and whether that's only the end of the subject
	auto rest{rest_match::always};
	auto only_end{true};
	for (auto item{items.rbegin()}; item != items.rend(); ++item)
	{
		if (item->type == pattern_item::kind::single && item->quantifier == '?')
		{
			cost *= 2.0;
		}
		auto const repeated{item->type == pattern_item::kind::scan
		                    || (item->type == pattern_item::kind::single && item->quantifier != 0 && item->quantifier != '?')};
		// a repetition followed by what always matches is tried once (as one that ends the pattern): past it the match
		// is over. otherwise the rest is tried at each length it takes, as many as the subject has positions; before
		// '$' alone, as many as its longest run in the subject, past which it stops
		if (repeated && rest != rest_match::always)
		{
			auto const before_end{only_end && rest == rest_match::at_end && item->type == pattern_item::kind::single};
			cost *= before_end && Subject != nullptr ? static_cast<double>(longest_run(Pattern, *item, *Subject)) + 1.0 : positions;
		}
		only_end = only_end && (item->type == pattern_item::kind::paren || item->type == pattern_item::kind::end_anchor);
		rest = match_with(Pattern, *item, rest);
	}
	return cost * length;
}

} // namespace

double lua_pattern_cost(std::string_view const Pattern, std::size_t const Subject, bool const Plain, bool const Anchorable)
{
	return pattern_cost(Pattern, Subject, nullptr, Plain, Anchorable);
}

double lua_pattern_cost(std::string_view const Pattern, std::string_view const Subject, bool const Plain, bool const Anchorable)
{
	return pattern_cost(Pattern, Subject.size(), &Subject, Plain, Anchorable);
}
} // namespace script_screens
