/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <iosfwd>
#include <memory>

#include "utilities/utilities.h" // MAKE_ID4

// Binary, modular scenery container ("eu7" format, version 1).
//
// Replaces the legacy text formats (.scn / .scm / .inc) and the old terrain-only
// .sbt blob. The mapping is one-to-one and modular: every text file compiles to
// exactly one binary twin and includes are preserved as references, so the binary
// twin of one file points to the binary twins of the files it includes.
//     route.scn -> route.scnb
//     piece.inc -> piece.incb
//     mesh.scm  -> mesh.scmb
//
// A binary twin stores the source file's own, fully resolved content (comments
// stripped, parameters and random sets resolved) as an ordered list of entries.
// Numeric tokens are stored as 8-byte IEEE doubles (so coordinates, ranges etc. are
// genuinely binary, not ASCII). All string tokens (keywords like "node"/"endmodel",
// names, paths) are interned into a per-file string table and referenced from the
// entries by a small varint index, so a keyword that occurs thousands of times is
// stored once. An "include" entry marks the spot where the source file pulled in
// another file, carrying the (interned) include filename expression and parameters.
//
// The binary is consumed transparently at the cParser file layer: opening a file
// whose binary twin exists serves tokens from the twin instead of tokenizing text,
// and an include entry triggers the same include machinery as the text path (which
// recursively prefers the included file's own binary twin). The scenery
// deserializer is unaware of the format and needs no changes; the file's identity
// (cParser::Name()), node grouping per .inc and parameter substitution are all
// preserved exactly as in the text path.

namespace scene {

// "eu7" + format version, little-endian via MAKE_ID4 ('e','u','7',<ver>).
// v2 introduced typed numeric entries (f64); v3 interns string tokens into a per-file
// table referenced by varint indices. bumping the version invalidates older twins so
// they are recompiled rather than misread.
constexpr std::uint32_t SCENERYBINARY_MAGIC { MAKE_ID4( 'e', 'u', '7', 3 ) };

// file extension of the binary twin, derived from source kind
std::string const SCENERYBINARY_EXT_SCN { ".scnb" };
std::string const SCENERYBINARY_EXT_INC { ".incb" };
std::string const SCENERYBINARY_EXT_SCM { ".scmb" };
std::string const SCENERYBINARY_EXT_CTR { ".ctrb" };

// which text format the binary file was compiled from
enum class scenery_file_kind : std::uint8_t {
    scn = 0,
    inc = 1,
    scm = 2,
};

// kind of a stored entry
enum class scenery_entry_type : std::uint8_t {
    token = 0,   // a non-numeric resolved token (name/path/keyword), stored as a string
    include = 1, // an include directive: pulls in another file with parameters
    number = 2,  // a numeric token, stored as an 8-byte IEEE double
};

// a single ordered element of a file's resolved content
struct scenery_entry {
    scenery_entry_type type { scenery_entry_type::token };
    // token value (token entries only)
    std::string text;
    // numeric value (number entries only)
    double number { 0.0 };
    // include only: the raw filename expression as written in the source, i.e. either
    // a single filename token or a random set "[ a b c ]" (including the brackets).
    // kept verbatim so the random choice is re-evaluated on every replay rather than
    // frozen at compile time.
    std::vector<std::string> fileexpr;
    // include only: directive parameters
    std::vector<std::string> params;
};

// accumulates a file's entries and writes them, with a header, to a stream
class scenery_binary_writer {

public:
    void add_token( std::string Token );
    // stores a numeric token as a typed double
    void add_number( double Value );
    // Fileexpr is the verbatim filename expression (single token or random set)
    void add_include( std::vector<std::string> Fileexpr, std::vector<std::string> Params );
    std::size_t entry_count() const { return m_entries.size(); }
    // serializes header + entries to the stream. returns false on stream failure.
    bool write( std::ostream &Output, scenery_file_kind Kind ) const;

private:
    std::vector<scenery_entry> m_entries;
};

// reads a binary scenery file fully into memory and exposes its entries in order
class scenery_binary_reader {

public:
    // validates magic/version and loads every entry. returns false if the stream is
    // not a valid current-version binary scenery file.
    bool open( std::istream &Input );

    scenery_file_kind kind() const { return m_kind; }
    std::vector<scenery_entry> const &entries() const { return m_entries; }

private:
    scenery_file_kind m_kind { scenery_file_kind::scn };
    std::vector<scenery_entry> m_entries;
};

// maps a source scenery filename to the extension of its binary twin
std::string scenerybinary_extension_for( std::string const &Sourcefile );

// Asynchronous baking: serializing and writing a twin is pure, self-contained work
// (it only needs the writer's collected entries), so it is offloaded to a small thread
// pool and overlaps with the rest of scene construction instead of blocking the load.
// Takes ownership of the writer.
void scenerybinary_write_async( std::unique_ptr<scenery_binary_writer> Writer, std::string Path, scenery_file_kind Kind );
// Blocks until every queued async write has finished, then emits their log lines on the
// calling (main) thread. Call once the scenario has finished loading.
void scenerybinary_wait_all();

} // namespace scene
