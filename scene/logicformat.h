/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <cstdint>

// On-disk layout of the scenery's LOGIC container. Single source of truth for both the
// cooker and the engine, so it deliberately knows nothing about either: no engine types, no
// includes beyond the fixed-width integers.
//
// The container holds what the simulation needs before it can run - events, and the tables
// that bind them to each other - as flat arrays indexed by position. Nothing here is a
// pointer and nothing is looked up by name at load: an event's targets are a range in a
// shared array, and a sibling is an index. That is the whole point of the section. The text
// path resolves those names in init_targets() every time a scenery loads, and it is the part
// of the load that cannot be made cheaper while the names are still there.
//
// Refusing to load is the only migration story. A container whose magic, version or schema
// hash disagrees with this header is rejected and the scenery falls back to the text path,
// because a cooked file is a cache and can always be rebuilt.
namespace scene::logic {

// "EU07LOGC", little-endian, so a file opened in a hex editor names itself
inline constexpr std::uint64_t magic { 0x43474f4c37305545ull };
inline constexpr std::uint32_t version { 1 };
// stamped into the header and compared on load. bump it whenever a record below changes
// shape, so a stale container is refused rather than misread
inline constexpr std::uint32_t schema_hash { 0x2f74be09u };
// written as 1; a reader seeing 0x01000000 knows it is looking at the wrong endianness
inline constexpr std::uint32_t endian_marker { 1 };
// every section starts on this boundary, so records can be read in place
inline constexpr std::size_t alignment { 16 };
// index meaning "no such element", used where a record refers to another and need not
inline constexpr std::uint32_t no_index { 0xffffffffu };

enum class section : std::uint32_t {
    strings = 0,     // the string blob every name indexes into
    stringtable = 1, // one string_entry per interned string
    events = 2,      // one event_record per event, in the order the manager holds them
    targets = 3,     // target_entry array; an event owns a contiguous run of it
    count
};

struct file_header {
    std::uint64_t magic;
    std::uint32_t version;
    std::uint32_t schema_hash;
    std::uint32_t endian;
    std::uint32_t sectioncount;
    // total size of the file as written, so a truncated container is caught before it is read
    std::uint64_t filesize;
    std::uint64_t reserved[ 3 ];
};

struct section_entry {
    std::uint32_t kind;
    std::uint32_t elementsize;
    std::uint64_t offset;
    std::uint64_t bytes;
    std::uint64_t count;
};

// one interned string: where it starts in the strings blob and how long it is. strings are
// not terminated, so a reader hands out views straight into the mapped buffer
struct string_entry {
    std::uint32_t offset;
    std::uint32_t length;
};

// what an event does. the values are written to disk, so they are assigned explicitly and
// never reordered - a new kind goes on the end
enum class event_kind : std::uint16_t {
    unknown      = 0,
    updatevalues = 1,
    copyvalues   = 2,
    getvalues    = 3,
    putvalues    = 4,
    whois        = 5,
    logvalues    = 6,
    multiple     = 7,
    switchevent  = 8,  // written "switch" in the scenery, which is a keyword here
    trackvel     = 9,
    sound        = 10,
    texture      = 11,
    animation    = 12,
    lights       = 13,
    voltage      = 14,
    visible      = 15,
    friction     = 16,
    message      = 17,
    lua          = 18,
    // the same class as updatevalues, told apart by a flag the base does not hold. it gets
    // its own value rather than being cooked as updatevalues, because until the flag word is
    // cooked too that is the only thing keeping the two apart
    addvalues    = 19,
};

// the part of an event that every kind has. what the kind itself needs beyond this lives in
// the payload range, which the cooker writes and the matching subclass reads back
struct event_record {
    std::uint16_t kind;
    // the per-kind flag word lives in the subclasses, not in the base, so it is cooked with
    // the rest of the payload once the sections for it exist. zero until then
    std::uint16_t reserved0;
    std::uint32_t name;       // index into the string table
    std::uint32_t targetfirst; // first entry in the targets section
    std::uint32_t targetcount;
    std::uint32_t sibling;    // index of the next event sharing this name, or no_index
    std::uint32_t group;      // scene group handle as cooked, or no_index
    double delay;
    double delayrandom;
    // NaN when the event is not tied to a departure. written as the exact bit pattern the
    // text path produces, so a round trip compares equal bit for bit
    double delaydeparture;
    std::uint8_t ignored;
    std::uint8_t passive;
    std::uint8_t padding[ 6 ];
};

// one target of one event: the name it was written with, and the index of whatever it
// resolved to. the name is kept because the editor exports text again, and because an
// unresolved target has to be reported by the name the author wrote
// which of the engine's node registries a target was found in. an event resolves its
// targets against one or two of them, decided by what kind of event it is - and for a
// visible event, decided per target, since it looks in the instances first and the tracks
// after. the values are written to disk, so they are assigned explicitly
enum class node_table : std::uint16_t {
    none = 0,
    memory = 1,
    instance = 2,
    path = 3,
    sound = 4,
    powergrid = 5,
};

// one target of one event: which registry held it and where in that registry, plus the
// name it was written with.
//
// The index is the target's position in that registry, which is the order the scenery
// built it in - so it is only meaningful for the parse this container was cooked from.
// The name is kept as the check on that: a loader compares it against the node the index
// lands on and falls back to a lookup when they disagree, which is what stops a container
// that went stale without being noticed from binding events to the wrong nodes.
struct target_entry {
    std::uint32_t name;
    std::uint16_t table;
    std::uint16_t reserved;
    std::uint32_t index;
};
static_assert( sizeof( file_header ) == 56, "logic file_header changed shape" );
static_assert( sizeof( section_entry ) == 32, "logic section_entry changed shape" );
static_assert( sizeof( string_entry ) == 8, "logic string_entry changed shape" );
static_assert( sizeof( event_record ) == 56, "logic event_record changed shape" );
static_assert( sizeof( target_entry ) == 12, "logic target_entry changed shape" );
static_assert( alignof( event_record ) <= alignment, "logic records must fit the section alignment" );

// where a section's data starts, given what has been written so far
inline constexpr std::size_t
aligned( std::size_t const Offset ) {

    return ( Offset + alignment - 1 ) & ~( alignment - 1 );
}

} // namespace scene::logic
