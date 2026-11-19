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
#include <cstddef>
#include <cstdint>
#include <format>
#include <iosfwd>
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
	// can't be read as that type. returns: true if Target was set
	template <typename Type_> bool get(std::string_view Key, Type_ &Target) const;
	// the value of the key, or Fallback
	template <typename Type_> Type_ value(std::string_view Key, Type_ Fallback) const;
	std::string text(std::string_view Key, std::string_view Fallback = {}) const;

	bool operator==(record const &) const = default;

	std::string type;
	std::string name;
	std::vector<std::pair<std::string, std::string>> fields;

private:
	void set_text(std::string_view Key, std::string Value);
};

// the whole file: the entries in their order. the first one is the header, "snapshot <version> ... endsnapshot"
struct document {
	std::vector<record> records;
	// what the reader couldn't make sense of; a document with problems may still be of use
	std::vector<std::string> problems;

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
};
// Current: the links as they are, each given from both of its ends or from one. Wanted: the same for the snapshot.
// Present: the names of the vehicles of the scenery. links of vehicles the snapshot says nothing of stay as they are,
// unless a vehicle of the snapshot needs the coupler at their other end
coupling_plan plan_couplings(std::vector<coupling_link> const &Current, std::vector<coupling_link> const &Wanted, std::vector<std::string> const &Present);

// the snapshots asked for on the command line of the simulator
struct command_line {
	struct save_request {
		double time{0.0}; // of the simulation, seconds since the scenery started (Timer::GetTime())
		std::string file;
	};
	std::string load; // --load-snapshot <file>: put over the scenery once it's loaded
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
			Target = *number != 0.0;
		else
			Target = static_cast<type>(*number);
		return true;
	}
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
	return text != nullptr && detail::text_value(*text, Target);
}

template <typename Type_> Type_ record::value(std::string_view const Key, Type_ Fallback) const
{
	get(Key, Fallback);
	return Fallback;
}

} // namespace simulation::snapshot
