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
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
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

// the consists the links make, as sets of vehicles: two vehicles are of one consist if a chain of links joins them
class consists {

public:
	// joins the consists of the two vehicles. returns: false if they're of one consist already, which the link would
	// make into a ring
	bool join(std::string const &Left, std::string const &Right)
	{
		auto const left{root(Left)};
		auto const right{root(Right)};
		if (left == right)
			return false;
		m_parents[left] = right;
		return true;
	}

private:
	std::string root(std::string const &Vehicle) const
	{
		auto current{Vehicle};
		for (auto lookup{m_parents.find(current)}; lookup != m_parents.end(); lookup = m_parents.find(current))
			current = lookup->second;
		return current;
	}

	std::map<std::string, std::string, std::less<>> m_parents;
};

// what's refused is kept in the order it came, up to this many lines
constexpr std::size_t refusals_kept{100};

struct refusal_list {
	std::vector<std::string> lines;
	std::size_t more{0};
};

refusal_list &refusals()
{
	static refusal_list list;
	return list;
}

// a value of the file in what's told of it: a damaged file can have anything in a value, a whole other file too
std::string shortened(std::string_view const Text)
{
	constexpr std::size_t length{40};
	return Text.size() <= length ? std::string(Text) : std::string(Text.substr(0, length)) + "...";
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

std::optional<int> whole_number(double const Value, int const Min, int const Max)
{
	if (false == detail::fits_integer<int>(Value) || Value < Min || Value > Max)
		return std::nullopt;
	return static_cast<int>(Value);
}

void refuse(std::string Text)
{
	auto &list{refusals()};
	if (list.lines.size() < refusals_kept)
		list.lines.emplace_back(std::move(Text));
	else
		++list.more;
}

std::vector<std::string> take_refusals()
{
	auto list{std::move(refusals())};
	refusals() = {};
	if (list.more != 0)
		list.lines.emplace_back(std::format("and {} more values refused", list.more));
	return std::move(list.lines);
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

void record::refuse(std::string_view const Key, std::string_view const Why) const
{
	snapshot::refuse(std::format("{} {}: {} \"{}\" refused, {}", type, shortened(name), Key, shortened(text(Key)), Why));
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

std::optional<std::size_t> read_count(record const &Entry, std::string_view const Key)
{
	if (std::size_t count{0}; Entry.get(Key, count, std::size_t{0}, Entry.fields.size()))
		return count;
	return std::nullopt;
}

std::optional<std::size_t> read_index(record const &Entry, std::string_view const Key, std::size_t const Size)
{
	std::size_t index{0};
	if (false == Entry.get(Key, index))
		return std::nullopt;
	if (index >= Size)
	{
		Entry.refuse(Key, std::format("it's past the end of a table of {}", Size));
		return std::nullopt;
	}
	return index;
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
	return lookup != records.end() ? std::to_address(lookup) : nullptr;
}

record *document::find(std::string_view const Type, std::string_view const Name)
{
	auto const lookup{std::ranges::find_if(records, [&](record const &Entry) { return Entry.type == Type && Entry.name == Name; })};
	return lookup != records.end() ? std::to_address(lookup) : nullptr;
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
	// a version which isn't a whole number is no version this build knows
	return whole_number(text_number(first->name).value_or(0.0), 0, std::numeric_limits<int>::max()).value_or(0);
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
	auto const *header{result.header()};
	if (header == nullptr)
	{
		result.problems.emplace_back("the file doesn't start with a \"snapshot\" entry");
		return result;
	}
	if (result.version() > format_version)
		result.warnings.emplace_back(std::format("the file is of version {}, newer than {} which this build writes", result.version(), format_version));
	// the header counts the entries of the file: a file cut short between two entries reads without a fault otherwise
	if (auto const *entries{header->find("entries")}; entries != nullptr)
	{
		if (auto const count{text_number(*entries)}; false == count.has_value() || *count != static_cast<double>(result.records.size()))
		{
			result.problems.emplace_back(std::format("the header says the file has {} entries, it has {}: it's cut short, or it was put together from pieces",
			                                         shortened(*entries), result.records.size()));
		}
	}
	return result;
}

document parse(std::string const &Text)
{
	cParser parser(Text, cParser::buffer_TEXT);
	return read(parser);
}

bool accepted(document const &Document, bool const Force)
{
	return Document.header() != nullptr && (Document.problems.empty() || Force);
}

namespace {

std::filesystem::path temporary_file(std::string const &File)
{
	std::filesystem::path temporary{File};
	temporary += ".tmp";
	return temporary;
}

} // namespace

std::string open_temporary(std::string const &File, std::ofstream &Output)
{
	Output.open(temporary_file(File), std::ios::binary | std::ios::trunc);
	if (false == Output.is_open())
		return std::format("file \"{}.tmp\" can't be written", File);
	return {};
}

std::string put_in_place(std::string const &File, std::ofstream &Output)
{
	auto const temporary{temporary_file(File)};
	std::error_code error;
	Output.flush();
	Output.close();
	if (Output.fail())
	{
		std::filesystem::remove(temporary, error);
		return std::format("writing file \"{}.tmp\" failed", File);
	}
	// the whole text is there: it takes the place of the file at once
	std::filesystem::rename(temporary, File, error);
	if (error)
	{
		std::filesystem::remove(temporary, error);
		return std::format(R"(file "{0}.tmp" can't be put in place of "{0}")", File);
	}
	return {};
}

std::string write_file(std::string const &File, document const &Document)
{
	return write_file(File, [&Document](std::ostream &Output) { write(Output, Document); });
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
		if (option == "--load-snapshot-force")
		{
			result.force = true;
		}
		else if (option == "--load-snapshot")
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
		if (auto const *there{wanted_link(wanted, link.other, link.otherend)}; here == nullptr && there == nullptr)
			continue;
		// a vehicle which stays where the scenery put it isn't coupled to one put where the snapshot has it
		if (auto const kept{here != nullptr && present.contains(link.vehicle) && present.contains(link.other) &&
		                    same_partner(*here, coupler_link{link.other, link.otherend, link.flags})};
		    false == kept)
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
		if (auto const lookup{current.find(coupler_key{link.vehicle, link.end})}; lookup != current.end() && same_partner(lookup->second, coupler_link{link.other, link.otherend, link.flags}))
			continue;
		plan.join.emplace_back(link);
	}
	// the links which stay and the ones to make mustn't close a ring of vehicles
	consists joined;
	for (auto const &link : unique_links(Current))
	{
		if (std::ranges::find(plan.part, link) == plan.part.end())
			joined.join(link.vehicle, link.other);
	}
	std::erase_if(plan.join, [&joined, &plan](coupling_link const &Link) {
		if (Link.vehicle != Link.other && joined.join(Link.vehicle, Link.other))
			return false;
		plan.refused.emplace_back(Link);
		return true;
	});
	return plan;
}

std::vector<coupling_link> read_couplings(std::vector<record const *> const &Entries)
{
	std::vector<coupling_link> links;
	for (auto const *entry : Entries)
	{
		for (auto const side : {0, 1})
		{
			auto const key{std::format("coupler{}", side)};
			if (false == entry->has(key + "flags"))
				continue;
			// the end of the coupler on the other side is the index of one of the two couplers of a vehicle
			auto otherend{0};
			auto flags{0};
			if (false == entry->get(key + "end", otherend, 0, 1) || false == entry->get(key + "flags", flags, 0, coupling_flags_mask))
				continue;
			links.emplace_back(entry->name, side, entry->text(key), otherend, flags);
		}
	}
	return links;
}

bool placed_where_saved(record const &Entry, double const X, double const Y, double const Z)
{
	auto const x{text_number(Entry.text("x"))};
	auto const y{text_number(Entry.text("y"))};
	auto const z{text_number(Entry.text("z"))};
	if (false == x.has_value() || false == y.has_value() || false == z.has_value() || false == std::isfinite(*x + *y + *z))
		return true;
	// written so that a vehicle put at a place which isn't a number is off
	return std::hypot(X - *x, Y - *y, Z - *z) <= misplaced_distance;
}

std::string fingerprint_changes(record const &Saved, record const &Now)
{
	std::string changed;
	for (auto const &[key, value] : Now.fields)
	{
		if (auto const saved{Saved.text(key, value)}; saved != value)
			changed += std::format("{}{} {} (snapshot {})", changed.empty() ? "" : ", ", key, value, shortened(saved));
	}
	return changed;
}

std::string free_file(std::string const &File)
{
	std::error_code error;
	if (false == std::filesystem::exists(File, error))
		return File;
	// the number goes before the extension of the name, the folder is written as it was given
	auto const extension{std::filesystem::path{File}.extension().string()};
	auto const stem{File.substr(0, File.size() - extension.size())};
	auto number{2};
	auto file{std::format("{}_{}{}", stem, number, extension)};
	while (std::filesystem::exists(file, error))
	{
		++number;
		file = std::format("{}_{}{}", stem, number, extension);
	}
	return file;
}

} // namespace simulation::snapshot
