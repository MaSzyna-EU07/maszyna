/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <iosfwd>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

class cParser;

// snapshot of a running simulation (stage A of the saved game): a text file in the syntax of the scenery files, which
// names the scenery it was taken in and holds what changed since that scenery was loaded. loading one means loading the
// scenery the usual way and then putting the saved state over it.
// this file is the text itself: the entries, how they're written and read. what goes into them is in
// simulationsnapshotmover.cpp (the state of a vehicle's machinery) and simulationsnapshotworld.cpp (the rest)
namespace simulation::snapshot {

// version of the format, the name of the "snapshot" entry which starts the file. a reader takes the files of its own
// version and older ones, and of a newer file what it knows: unknown entries and keys are skipped
inline constexpr int format_version{1};

// writes a number so that reading it back gives the very same value
std::string number_text(double Value);
// a single precision value takes fewer digits to come back the same
std::string number_text(float Value);
// reads a number written by number_text(), or any other the standard reads. returns: empty if the text isn't a number
std::optional<double> text_number(std::string_view Text);
// a few numbers as one value, with commas between them, and back; what isn't a number is left out
std::string numbers_text(std::span<double const> Numbers);
std::vector<double> text_numbers(std::string_view Text);
// a number of the file as a whole number between Min and Max. returns: empty if it isn't one (not finite, a fraction,
// out of the range)
std::optional<int> whole_number(double Value, int Min, int Max);

// the values of the file the reader doesn't take: a number out of the range of what it's put into, a count past what
// the entry can hold, a text where a number is needed. a snapshot from a damaged or edited file mustn't put an index
// past the end of a table nor a count which takes forever, so each such value is left out (or brought into its range,
// where that's said) and told of here; the simulation collects them while a snapshot is put back, for the log
void refuse(std::string Text);
// the refusals told of since the last call, in their order. a damaged file can give a great many: past the first
// hundred only their number is kept, given as the last line
std::vector<std::string> take_refusals();

// one entry of a snapshot: "<type> <name> <key> <value> <key> <value> ... end<type>". the values are single tokens,
// kept as text in the order of the file, so an entry read and written back comes out the same, whatever it holds
class record {

public:
	record() = default;
	record(std::string_view Type, std::string_view Name) : type(Type), name(Name) {}

	// adds the value, or replaces the one the key has
	template <typename Type_> record &set(std::string_view Key, Type_ const &Value);
	// the value of the key, or nullptr
	std::string const *find(std::string_view Key) const;
	bool has(std::string_view Key) const { return find(Key) != nullptr; }
	// the value of the key in the type of Target, which stays as it is when the entry has no such key or the value
	// can't be read as that type (a whole number for a whole one, within the range of the type). a value which can't
	// be read is refused (refuse()). returns: true if Target was set
	template <typename Type_> bool get(std::string_view Key, Type_ &Target) const;
	// the same, for a value which has to be between Min and Max: one past them is refused
	template <typename Type_> bool get(std::string_view Key, Type_ &Target, Type_ Min, Type_ Max) const;
	// the same, but a value past Min or Max is brought to it, and that's told of (refuse())
	template <typename Type_> bool get_clamped(std::string_view Key, Type_ &Target, Type_ Min, Type_ Max) const;
	// the value of the key, or Fallback
	template <typename Type_> Type_ value(std::string_view Key, Type_ Fallback) const;
	// the value of the key between Min and Max, or Fallback
	template <typename Type_> Type_ value(std::string_view Key, Type_ Fallback, Type_ Min, Type_ Max) const;
	std::string text(std::string_view Key, std::string_view Fallback = {}) const;
	// tells of a value of the entry the reader didn't take, and why (refuse())
	void refuse(std::string_view Key, std::string_view Why) const;

	bool operator==(record const &) const = default;

	std::string type;
	std::string name;
	std::vector<std::pair<std::string, std::string>> fields;

private:
	void set_text(std::string_view Key, std::string Value);
};

// a count the entry gives of things it lists, each under a key of its own (the points of a table of a driver: "table"
// and "p0", "p1"...): a count past the number of keys of the entry can't be the count of what it lists, and is refused,
// as is one which isn't a whole number. returns: empty if the entry hasn't the key or its value is refused
std::optional<std::size_t> read_count(record const &Entry, std::string_view Key);
// an index the entry gives into a table of Size things: one past the end of the table is refused.
// returns: empty if the entry hasn't the key or its value is refused
std::optional<std::size_t> read_index(record const &Entry, std::string_view Key, std::size_t Size);

// the whole file: the entries in their order. the first one is the header, "snapshot <version> ... endsnapshot"
struct document {
	std::vector<record> records;
	// what the reader couldn't make sense of: the file is damaged or cut short. a document with problems isn't put
	// over a scenery unless that's forced (accepted())
	std::vector<std::string> problems;
	// what a reader should know, but which doesn't keep the document from being used: a file of a newer version
	std::vector<std::string> warnings;

	record &add(std::string_view Type, std::string_view Name);
	// the entries of the type, in the order of the file
	std::vector<record const *> of(std::string_view Type) const;
	// the first entry of the type with the name, or nullptr
	record const *find(std::string_view Type, std::string_view Name) const;
	record *find(std::string_view Type, std::string_view Name);
	// the header, or nullptr if the document doesn't start with one
	record const *header() const;
	// the version the header gives, 0 without a header
	int version() const;
};

// the document as text
void write(std::ostream &Output, document const &Document);
std::string to_text(document const &Document);
// reads the entries until the input ends
document read(cParser &Input);
document parse(std::string const &Text);
// reads the file; a file which can't be opened gives a document with that as its problem
document load(std::string const &File);
// whether the document is to be put over the scenery: it has to start with a header, and to have no problems unless
// Force says to use what could be read of it all the same (--load-snapshot-force)
bool accepted(document const &Document, bool Force);
// writes the text Writer gives to the file, whole or not at all: to "<file>.tmp" first, which takes the place of the
// file only once it's all written. a write which fails leaves the file as it was, and no .tmp behind.
// returns: what went wrong, empty if nothing did
std::string write_file(std::string const &File, std::function<void(std::ostream &)> const &Writer);
std::string write_file(std::string const &File, document const &Document);

// a thing of the scenery is named in a snapshot by its name where it has one of its own, which is the one the tables
// of the simulation find it by, and by its place in the table otherwise ("#12"): the tables are filled in the order
// of the scenery files, so the same scenery gives the same places
std::string reference(std::string_view Name, std::size_t Index, bool Foundbyname);
// the place in the table a reference gives, if it's of the "#12" kind
std::optional<std::size_t> reference_index(std::string_view Reference);

// a hash of names (FNV-1a, 64 bits), for the fingerprint of the scenery in the header: the snapshot remembers how many
// tracks, memory cells, events and vehicles the scenery had and a hash of their names, and a scenery which gives
// other numbers when the snapshot is loaded isn't the one it was taken in
class name_hash {

public:
	void add(std::string_view Name);
	std::uint64_t value() const { return m_hash; }
	// as text, 16 hexadecimal digits
	std::string text() const;

private:
	std::uint64_t m_hash{14695981039346656037ULL};
};

// the flags of a coupling a snapshot can give: every bit of the coupling enum of the physics (MOVER.h), from coupler
// (0x1) to power3x400v (0x400)
inline constexpr int coupling_flags_mask{0x7ff};

// what the vehicles of a snapshot are coupled to: for a vehicle and its end (0 front, 1 rear) the vehicle and the end
// on the other side, and the coupling::flags of what's joined
struct coupling_link {
	std::string vehicle;
	int end{0};
	std::string other;
	int otherend{0};
	int flags{0};

	bool operator==(coupling_link const &) const = default;
};

// what has to be done to the couplings of the scenery to get the ones of the snapshot: the links to part first, then
// the links to make. a link is listed once, by the vehicle and end which come first in the given lists
struct coupling_plan {
	std::vector<coupling_link> part;
	std::vector<coupling_link> join;
	// the links to set other flags on: joined to the same vehicle and end already, but by other means
	std::vector<coupling_link> change;
	// the links of the snapshot to vehicles which aren't in Present
	std::vector<coupling_link> missing;
	// the links of the snapshot which would close a ring of vehicles (a vehicle coupled to itself, or the last vehicle
	// of a consist to its first): a consist is a line, and the code which walks along one would never get to its end
	std::vector<coupling_link> refused;
};
// Current: the links as they are, each given from both of its ends or from one. Wanted: the same for the snapshot.
// Present: the names of the vehicles of the scenery. links of vehicles the snapshot says nothing of stay as they are,
// unless a vehicle of the snapshot needs the coupler at their other end
coupling_plan plan_couplings(std::vector<coupling_link> const &Current, std::vector<coupling_link> const &Wanted, std::vector<std::string> const &Present);
// the couplings of the vehicle entries ("coupler0", "coupler0end", "coupler0flags" and the same for 1), for
// plan_couplings(). an end other than 0 or 1, or flags which aren't those of a coupling, are refused (refuse()): that
// coupler is left out, as if the entry said nothing of it
std::vector<coupling_link> read_couplings(std::vector<record const *> const &Entries);

// the snapshots asked for on the command line of the simulator
struct command_line {
	struct save_request {
		double time{0.0}; // of the simulation, seconds since the scenery started (Timer::GetTime())
		std::string file;
	};
	std::string load; // --load-snapshot <file>: put over the scenery once it's loaded
	bool force{false}; // --load-snapshot-force: a damaged file is put over the scenery as far as it could be read
	std::vector<save_request> saves; // --snapshot-at <seconds> <file>, any number of them: written when the time comes
	std::string error; // what's wrong with the options; empty if nothing
};
// picks the snapshot options out of a command line (the first argument is the program), skipping the values of the
// other options the simulator takes
command_line parse_command_line(std::span<char const *const> Arguments);
// how many values follow an option of the simulator on its command line, for the parsers which pick out their own
int option_value_count(std::string_view Option);

// implementation of the templates

namespace detail {

template <typename Type_> std::string value_text(Type_ const &Value)
{
	using type = std::remove_cvref_t<Type_>;
	if constexpr (std::is_same_v<type, bool>)
		return Value ? "1" : "0";
	else if constexpr (std::is_enum_v<type>)
		return std::to_string(static_cast<long long>(Value));
	else if constexpr (std::is_integral_v<type>)
		return std::to_string(Value);
	else if constexpr (std::is_same_v<type, float>)
		return number_text(Value);
	else if constexpr (std::is_floating_point_v<type>)
		return number_text(static_cast<double>(Value));
	else
		return std::string(std::string_view(Value));
}

// whether the number fits a variable of the integer type: whole, and within its range
template <typename Integer_> bool fits_integer(double const Number)
{
	using limits = std::numeric_limits<Integer_>;
	// the bounds as exact doubles: -2^63 .. 2^63 for a 64 bit signed type, 0 .. 2^64 for an unsigned one; the upper one
	// is past the range
	auto const lowest{limits::is_signed ? -std::ldexp(1.0, limits::digits) : 0.0};
	auto const past{std::ldexp(1.0, limits::digits)};
	return std::isfinite(Number) && std::trunc(Number) == Number && Number >= lowest && Number < past;
}

template <typename Type_> bool text_value(std::string const &Text, Type_ &Target)
{
	using type = std::remove_cvref_t<Type_>;
	if constexpr (std::is_same_v<type, std::string>)
	{
		Target = Text;
		return true;
	}
	else
	{
		auto const number{text_number(Text)};
		if (false == number.has_value())
			return false;
		if constexpr (std::is_same_v<type, bool>)
		{
			if (std::isnan(*number))
				return false;
			Target = *number != 0.0;
		}
		else if constexpr (std::is_enum_v<type> || std::is_integral_v<type>)
		{
			// an integer out of the range of its type is undefined, a fraction isn't what was written
			using integer = typename std::conditional_t<std::is_enum_v<type>, std::underlying_type<type>, std::type_identity<type>>::type;
			if (false == fits_integer<integer>(*number))
				return false;
			Target = static_cast<type>(static_cast<integer>(*number));
		}
		else
		{
			// not a number and the infinities are values like any other, a finite number too big for the type isn't: one
			// which rounds past its largest value
			auto const top{static_cast<double>(std::numeric_limits<type>::max())};
			auto const step{top - static_cast<double>(std::nextafter(std::numeric_limits<type>::max(), type{0}))};
			if (std::isfinite(*number) && std::abs(*number) >= top + step / 2)
				return false;
			Target = static_cast<type>(*number);
		}
		return true;
	}
}

// the text of a bound, for what's told of a value out of its range
template <typename Type_> std::string bound_text(Type_ const Value)
{
	if constexpr (std::is_enum_v<Type_>)
		return std::to_string(static_cast<long long>(Value));
	else
		return value_text(Value);
}

} // namespace detail

template <typename Type_> record &record::set(std::string_view const Key, Type_ const &Value)
{
	set_text(Key, detail::value_text(Value));
	return *this;
}

template <typename Type_> bool record::get(std::string_view const Key, Type_ &Target) const
{
	auto const *text{find(Key)};
	if (text == nullptr)
		return false;
	if (detail::text_value(*text, Target))
		return true;
	refuse(Key, "it can't be read as a value of its kind");
	return false;
}

template <typename Type_> bool record::get(std::string_view const Key, Type_ &Target, Type_ const Min, Type_ const Max) const
{
	auto value{Target};
	if (false == get(Key, value))
		return false;
	// written so that not a number fails too
	if (false == (value >= Min && value <= Max))
	{
		refuse(Key, std::format("it's out of the range {} .. {}", detail::bound_text(Min), detail::bound_text(Max)));
		return false;
	}
	Target = value;
	return true;
}

template <typename Type_> bool record::get_clamped(std::string_view const Key, Type_ &Target, Type_ const Min, Type_ const Max) const
{
	auto value{Target};
	if (false == get(Key, value))
		return false;
	if constexpr (std::is_floating_point_v<Type_>)
	{
		if (std::isnan(value))
		{
			refuse(Key, "it's not a number");
			return false;
		}
	}
	if (value < Min || value > Max)
	{
		auto const clamped{value < Min ? Min : Max};
		refuse(Key, std::format("it's out of the range {} .. {}, {} is taken", detail::bound_text(Min), detail::bound_text(Max), detail::bound_text(clamped)));
		value = clamped;
	}
	Target = value;
	return true;
}

template <typename Type_> Type_ record::value(std::string_view const Key, Type_ Fallback) const
{
	get(Key, Fallback);
	return Fallback;
}

template <typename Type_> Type_ record::value(std::string_view const Key, Type_ Fallback, Type_ const Min, Type_ const Max) const
{
	get(Key, Fallback, Min, Max);
	return Fallback;
}

} // namespace simulation::snapshot
