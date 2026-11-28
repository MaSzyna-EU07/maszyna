/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// a set of the values of an enum class whose values are bits: what a coupling joins, the damage of a vehicle. the
// numbers of the files (their sums of the bits) come in only through FromBits() and go out through Bits()

#pragma once

#include <type_traits>

// an enum is a kind of the sets by a specialisation of this, which also gives its operator |
template <typename Enum> struct IsEnumSetKind : std::false_type
{
};

template <typename Enum> class EnumSet
{
	static_assert(std::is_enum_v<Enum>, "a set of the values of an enum");

public:
	constexpr EnumSet() = default;
	constexpr EnumSet(Enum const value) : bits{static_cast<unsigned>(value)} {} // NOSONAR: a value is a set of one
	// the set the number of a file gives (the sum of the numbers of the values)
	static constexpr EnumSet FromBits(int const number)
	{
		EnumSet set;
		set.bits = static_cast<unsigned>(number);
		return set;
	}
	// every value: a set of any of them has one of these
	static constexpr EnumSet All()
	{
		return FromBits(-1);
	}
	// the number of the set, for the files and the log
	constexpr int Bits() const
	{
		return static_cast<int>(bits);
	}
	constexpr bool Empty() const
	{
		return bits == 0;
	}
	// any value at all: as the old number was true when it wasn't zero
	constexpr explicit operator bool() const
	{
		return bits != 0;
	}
	// all the values of the other set are in this one
	constexpr bool HasAll(EnumSet const other) const
	{
		return (bits & other.bits) == other.bits;
	}
	// any of the values of the other set is in this one
	constexpr bool HasAny(EnumSet const other) const
	{
		return (bits & other.bits) != 0;
	}
	// adds the values; returns whether any of them was missing (as SetFlag())
	constexpr bool Set(EnumSet const other)
	{
		if (HasAll(other))
		{
			return false;
		}
		bits |= other.bits;
		return true;
	}
	// takes the values away, where all of them are there; returns whether it did (as ClearFlag())
	constexpr bool Clear(EnumSet const other)
	{
		if (false == HasAll(other))
		{
			return false;
		}
		bits &= ~other.bits;
		return true;
	}
	// this set without the values of the other
	constexpr EnumSet Without(EnumSet const other) const
	{
		return FromBits(static_cast<int>(bits & ~other.bits));
	}
	constexpr EnumSet &operator|=(EnumSet const other)
	{
		bits |= other.bits;
		return *this;
	}
	constexpr EnumSet &operator&=(EnumSet const other)
	{
		bits &= other.bits;
		return *this;
	}
	// the values in one of the sets and not in the other: what changed between them
	constexpr EnumSet &operator^=(EnumSet const other)
	{
		bits ^= other.bits;
		return *this;
	}
	friend constexpr EnumSet operator|(EnumSet left, EnumSet const right)
	{
		return left |= right;
	}
	friend constexpr EnumSet operator&(EnumSet left, EnumSet const right)
	{
		return left &= right;
	}
	friend constexpr EnumSet operator^(EnumSet left, EnumSet const right)
	{
		return left ^= right;
	}
	friend constexpr bool operator==(EnumSet const &left, EnumSet const &right) = default;

private:
	unsigned bits{0};
};

// two values make a set of them, for the enums which are kinds of the sets
template <typename Enum>
    requires IsEnumSetKind<Enum>::value
constexpr EnumSet<Enum> operator|(Enum const left, Enum const right)
{
	return EnumSet<Enum>{left} | EnumSet<Enum>{right};
}
