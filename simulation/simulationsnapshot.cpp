/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "simulation/simulationsnapshot.h"

#include "utilities/parser.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <system_error>

namespace simulation::snapshot {

namespace {

// a value which is no text at all is written as this, there's no way to write an empty token
constexpr std::string_view empty_mark{"<>"};
// lines of the file are kept shorter than this, where the tokens allow
constexpr std::size_t line_width{110};

// characters a token can hold without quotes around it: nothing which ends a token, starts a comment or a quoted text
bool is_plain(std::string_view const Text)
{
	if (Text.find("//") != std::string_view::npos || Text.find("/*") != std::string_view::npos)
		return false;
	return std::ranges::none_of(Text, [](char const Character) {
		auto const code{static_cast<unsigned char>(Character)};
		return code <= ' ' || Character == ';' || Character == '"' || Character == '\\' || code == 127;
	});
}

// the value as a token of the file; Terminator is the word which ends the entry the value is written in
std::string encode(std::string_view const Value, std::string_view const Terminator)
{
	if (Value.empty())
		return std::string(empty_mark);
	std::string text;
	if (Value.front() == '<' || Value == Terminator)
	{
		// one more '<' in front keeps the value apart from the mark of an empty one, and from the end of its entry
		text += '<';
	}
	text += Value;
	if (is_plain(text))
		return text;
	std::string quoted{"\""};
	for (auto const character : text)
	{
		if (character == '"' || character == '\\')
			quoted += '\\';
		quoted += character;
	}
	quoted += '"';
	return quoted;
}

// the value a token of the file stands for; the parser has taken the quotes off already
std::string decode(std::string const &Token)
{
	if (Token == empty_mark)
		return {};
	if (Token.starts_with('<'))
		return Token.substr(1);
	return Token;
}

std::string next_token(cParser &Input)
{
	return Input.getToken<std::string>(false);
}

// the headings put before the entries of a type, for a reader of the file
std::string_view heading(std::string_view const Type)
{
	using entry = std::pair<std::string_view, std::string_view>;
	static constexpr std::array headings{
	    entry{"time", "date and time of the simulation"},
	    entry{"weather", "weather"},
	    entry{"random", "random number engine"},
	    entry{"memcell", "memory cells: text, value 1, value 2"},
	    entry{"switch", "switches: the path set"},
	    entry{"track", "tracks changed by events: speed limit, damage"},
	    entry{"crossing", "road crossings: the way through which was taken last"},
	    entry{"madetrack", "tracks made on the run where a track of the scenery ends: turning loops, derailing tracks"},
	    entry{"powersource", "traction power sources: voltage, current, admittance now and before, output, fuses, their timer and count"},
	    entry{"isolated", "isolated sections: axles counted in"},
	    entry{"event", "queued events: seconds left, the vehicle which set them off"},
	    entry{"launcher", "event launchers: their timers"},
	    entry{"model", "lights of the models which events switch: the signals"},
	    entry{"vehicle", "vehicles: place, speed, couplings, load, machinery"},
	    entry{"driver", "drivers: orders, timetable"},
	    entry{"player", "the vehicle of the player"},
	};
	auto const lookup{std::ranges::find(headings, Type, &entry::first)};
	return lookup != headings.end() ? lookup->second : std::string_view{};
}

void write_record(std::ostream &Output, record const &Record)
{
	auto const terminator{"end" + Record.type};
	std::string line{Record.type + ' ' + encode(Record.name, terminator)};
	for (auto const &[key, value] : Record.fields)
	{
		auto const token{key + ' ' + encode(value, terminator)};
		if (line.size() + 1 + token.size() > line_width)
		{
			Output << line << '\n';
			line = "   ";
		}
		line += ' ';
		line += token;
	}
	if (line.size() + 1 + terminator.size() > line_width)
	{
		Output << line << '\n';
		line = "   ";
	}
	Output << line << ' ' << terminator << '\n';
}

// reads the keys and values of an entry, up to its end. returns: what went wrong, if anything
std::string read_fields(cParser &Input, record &Record)
{
	auto const terminator{"end" + Record.type};
	while (true)
	{
		auto key{next_token(Input)};
		if (key == terminator)
			return {};
		if (key.empty())
			return std::format("entry \"{} {}\" isn't closed with {}", Record.type, Record.name, terminator);
		auto const value{next_token(Input)};
		if (value.empty() || value == terminator)
			return std::format("entry \"{} {}\" has key {} without a value", Record.type, Record.name, key);
		Record.fields.emplace_back(std::move(key), decode(value));
	}
}

using coupler_key = std::pair<std::string, int>;

struct coupler_link {
	std::string other;
	int otherend{0};
	int flags{0};
};

using coupler_map = std::map<coupler_key, coupler_link, std::less<>>;

// the links by each of their ends; a link to nothing is the vehicle's end standing free
coupler_map by_coupler(std::vector<coupling_link> const &Links)
{
	coupler_map map;
	for (auto const &link : Links)
	{
		auto const linked{false == link.other.empty() && link.flags != 0};
		map[coupler_key{link.vehicle, link.end}] = linked ? coupler_link{link.other, link.otherend, link.flags} : coupler_link{};
		if (linked)
			map[coupler_key{link.other, link.otherend}] = coupler_link{link.vehicle, link.end, link.flags};
	}
	return map;
}

bool same_partner(coupler_link const &Left, coupler_link const &Right)
{
	return Left.other == Right.other && Left.otherend == Right.otherend;
}

// each link once: the end it's listed by first is the one which comes first in the list
std::vector<coupling_link> unique_links(std::vector<coupling_link> const &Links)
{
	std::vector<coupling_link> unique;
	std::set<coupler_key, std::less<>> seen;
	for (auto const &link : Links)
	{
		if (link.other.empty() || link.flags == 0)
			continue;
		if (seen.contains(coupler_key{link.vehicle, link.end}) || seen.contains(coupler_key{link.other, link.otherend}))
			continue;
		seen.emplace(link.vehicle, link.end);
		seen.emplace(link.other, link.otherend);
		unique.emplace_back(link);
	}
	return unique;
}

// what the snapshot wants of a coupler, if it says anything of it
coupler_link const *wanted_link(coupler_map const &Wanted, std::string const &Vehicle, int const End)
{
	auto const lookup{Wanted.find(coupler_key{Vehicle, End})};
	return lookup != Wanted.end() ? &lookup->second : nullptr;
}

} // namespace

std::string number_text(double const Value)
{
	if (std::isnan(Value))
		return "nan";
	if (std::isinf(Value))
		return Value > 0.0 ? "inf" : "-inf";
	// the shortest text which reads back as the same value
	return std::format("{}", Value);
}

std::string number_text(float const Value)
{
	if (std::isnan(Value) || std::isinf(Value))
		return number_text(static_cast<double>(Value));
	return std::format("{}", Value);
}

std::optional<double> text_number(std::string_view Text)
{
	if (Text.empty())
		return std::nullopt;
	if (Text == "nan")
		return std::nan("");
	if (Text == "inf")
		return HUGE_VAL;
	if (Text == "-inf")
		return -HUGE_VAL;
	if (Text.front() == '+')
		Text.remove_prefix(1);
	double value{0.0};
	auto const *const end{Text.data() + Text.size()};
	if (auto const [parsed, error]{std::from_chars(Text.data(), end, value)}; error == std::errc{} && parsed == end)
		return value;
	return std::nullopt;
}

std::string numbers_text(std::span<double const> const Numbers)
{
	std::string text;
	for (auto const number : Numbers)
	{
		if (false == text.empty())
			text += ',';
		text += number_text(number);
	}
	return text;
}

std::vector<double> text_numbers(std::string_view Text)
{
	std::vector<double> numbers;
	while (false == Text.empty())
	{
		auto const comma{Text.find(',')};
		if (auto const number{text_number(Text.substr(0, comma))}; number.has_value())
			numbers.emplace_back(*number);
		Text.remove_prefix(comma == std::string_view::npos ? Text.size() : comma + 1);
	}
	return numbers;
}

std::string const *record::find(std::string_view const Key) const
{
	auto const lookup{std::ranges::find(fields, Key, &std::pair<std::string, std::string>::first)};
	return lookup != fields.end() ? &lookup->second : nullptr;
}

std::string record::text(std::string_view const Key, std::string_view const Fallback) const
{
	auto const *value{find(Key)};
	return value != nullptr ? *value : std::string(Fallback);
}

void record::set_text(std::string_view const Key, std::string Value)
{
	if (auto const lookup{std::ranges::find(fields, Key, &std::pair<std::string, std::string>::first)}; lookup != fields.end())
	{
		lookup->second = std::move(Value);
		return;
	}
	fields.emplace_back(std::string(Key), std::move(Value));
}

record &document::add(std::string_view const Type, std::string_view const Name)
{
	return records.emplace_back(Type, Name);
}

std::vector<record const *> document::of(std::string_view const Type) const
{
	std::vector<record const *> found;
	for (auto const &entry : records)
	{
		if (entry.type == Type)
			found.emplace_back(&entry);
	}
	return found;
}

record const *document::find(std::string_view const Type, std::string_view const Name) const
{
	auto const lookup{std::ranges::find_if(records, [&](record const &Entry) { return Entry.type == Type && Entry.name == Name; })};
	return lookup != records.end() ? &*lookup : nullptr;
}

record *document::find(std::string_view const Type, std::string_view const Name)
{
	auto const lookup{std::ranges::find_if(records, [&](record const &Entry) { return Entry.type == Type && Entry.name == Name; })};
	return lookup != records.end() ? &*lookup : nullptr;
}

record const *document::header() const
{
	return false == records.empty() && records.front().type == "snapshot" ? &records.front() : nullptr;
}

int document::version() const
{
	auto const *first{header()};
	if (first == nullptr)
		return 0;
	auto const number{text_number(first->name)};
	return number.has_value() ? static_cast<int>(*number) : 0;
}

void write(std::ostream &Output, document const &Document)
{
	Output << "// MaSzyna simulation snapshot: the state of a running scenery, loaded over it with --load-snapshot\n";
	std::string lasttype;
	for (auto const &entry : Document.records)
	{
		if (entry.type != lasttype)
		{
			lasttype = entry.type;
			if (auto const text{heading(entry.type)}; false == text.empty())
				Output << "\n// " << text << '\n';
		}
		write_record(Output, entry);
	}
}

std::string to_text(document const &Document)
{
	std::ostringstream output;
	write(output, Document);
	return output.str();
}

document read(cParser &Input)
{
	// a value can be any word, "include" among them
	Input.expandIncludes = false;
	document result;
	auto type{next_token(Input)};
	while (false == type.empty())
	{
		auto name{next_token(Input)};
		if (name.empty())
		{
			result.problems.emplace_back(std::format("the file ends after \"{}\"", type));
			break;
		}
		auto &entry{result.records.emplace_back(type, decode(name))};
		if (auto problem{read_fields(Input, entry)}; false == problem.empty())
			result.problems.emplace_back(std::move(problem));
		type = next_token(Input);
	}
	if (result.header() == nullptr)
		result.problems.emplace_back("the file doesn't start with a \"snapshot\" entry");
	else if (result.version() > format_version)
		result.problems.emplace_back(std::format("the file is of version {}, newer than {} which this build writes", result.version(), format_version));
	return result;
}

document parse(std::string const &Text)
{
	cParser parser(Text, cParser::buffer_TEXT);
	return read(parser);
}

document load(std::string const &File)
{
	std::ifstream input(File, std::ios::binary);
	if (false == input.is_open())
	{
		document result;
		result.problems.emplace_back(std::format("file \"{}\" can't be opened", File));
		return result;
	}
	std::ostringstream text;
	text << input.rdbuf();
	return parse(text.str());
}

std::string reference(std::string_view const Name, std::size_t const Index, bool const Foundbyname)
{
	if (Foundbyname && false == Name.empty() && Name.front() != '#')
		return std::string(Name);
	return std::format("#{}", Index);
}

std::optional<std::size_t> reference_index(std::string_view const Reference)
{
	if (Reference.size() < 2 || Reference.front() != '#')
		return std::nullopt;
	std::size_t index{0};
	auto const *const begin{Reference.data() + 1};
	auto const *const end{Reference.data() + Reference.size()};
	if (auto const [parsed, error]{std::from_chars(begin, end, index)}; error == std::errc{} && parsed == end)
		return index;
	return std::nullopt;
}

void name_hash::add(std::string_view const Name)
{
	constexpr std::uint64_t prime{1099511628211ULL};
	for (auto const character : Name)
	{
		m_hash ^= static_cast<unsigned char>(character);
		m_hash *= prime;
	}
	// the end of the name counts too: "ab", "c" isn't "a", "bc"
	m_hash ^= 0xffU;
	m_hash *= prime;
}

std::string name_hash::text() const
{
	return std::format("{:016x}", m_hash);
}

int option_value_count(std::string_view const Option)
{
	if (Option == "--snapshot-at")
		return 2;
	if (Option == "-s" || Option == "-v" || Option == "-seed" || Option == "-ini" || Option == "--test-timeout" || Option == "--test-wall-timeout" ||
	    Option == "--load-snapshot")
		return 1;
	return 0;
}

command_line parse_command_line(std::span<char const *const> const Arguments)
{
	command_line result;
	std::size_t index{1};
	while (index < Arguments.size())
	{
		std::string_view const option{Arguments[index]};
		auto const values{static_cast<std::size_t>(option_value_count(option))};
		auto const complete{index + values < Arguments.size()};
		if (option == "--load-snapshot")
		{
			if (complete)
				result.load = Arguments[index + 1];
			else if (result.error.empty())
				result.error = "--load-snapshot needs the name of a snapshot file";
		}
		else if (option == "--snapshot-at")
		{
			auto const seconds{complete ? text_number(Arguments[index + 1]) : std::nullopt};
			if (seconds.has_value() && std::isfinite(*seconds) && *seconds >= 0.0)
				result.saves.emplace_back(*seconds, Arguments[index + 2]);
			else if (result.error.empty())
				result.error = "--snapshot-at needs a number of seconds and the name of a file";
		}
		index += 1 + values;
	}
	return result;
}

coupling_plan plan_couplings(std::vector<coupling_link> const &Current, std::vector<coupling_link> const &Wanted, std::vector<std::string> const &Present)
{
	coupling_plan plan;
	auto const current{by_coupler(Current)};
	auto const wanted{by_coupler(Wanted)};
	std::set<std::string, std::less<>> const present(Present.begin(), Present.end());

	// what's coupled and shouldn't be, or should be another way
	for (auto const &link : unique_links(Current))
	{
		auto const *here{wanted_link(wanted, link.vehicle, link.end)};
		auto const *there{wanted_link(wanted, link.other, link.otherend)};
		if (here == nullptr && there == nullptr)
			continue;
		auto const kept{here != nullptr && same_partner(*here, coupler_link{link.other, link.otherend, link.flags})};
		if (false == kept)
		{
			plan.part.emplace_back(link);
			continue;
		}
		if (here->flags != link.flags)
			plan.change.emplace_back(link.vehicle, link.end, link.other, link.otherend, here->flags);
	}
	// what should be coupled and isn't
	for (auto const &link : unique_links(Wanted))
	{
		if (false == present.contains(link.vehicle) || false == present.contains(link.other))
		{
			plan.missing.emplace_back(link);
			continue;
		}
		auto const lookup{current.find(coupler_key{link.vehicle, link.end})};
		if (lookup != current.end() && same_partner(lookup->second, coupler_link{link.other, link.otherend, link.flags}))
			continue;
		plan.join.emplace_back(link);
	}
	return plan;
}

} // namespace simulation::snapshot
