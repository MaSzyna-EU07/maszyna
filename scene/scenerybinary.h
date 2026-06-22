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
#include <string_view>
#include <vector>
#include <iosfwd>
#include <memory>
#include <sstream>
#include <unordered_map>

#include "utilities/utilities.h" // MAKE_ID4

// Binary, modular scenery container ("eu7" format).
//
// Replaces the legacy text formats (.scn / .scm / .inc / .ctr) and the old terrain-only
// .sbt blob. The mapping is one-to-one and modular: every text file compiles to exactly
// one binary twin and includes are preserved as references, so the binary twin of one
// file points to the binary twins of the files it includes.
//     route.scn -> route.scnb     piece.inc -> piece.incb
//     mesh.scm  -> mesh.scmb       events.ctr -> events.ctrb
//
// A twin stores the source file's own, fully resolved content (comments stripped,
// parameters/random sets resolved) as an ordered list of entries:
//   * string tokens (keywords like "node"/"endmodel", names, paths) are interned into a
//     per-file table and referenced by a varint index, so a keyword that occurs
//     thousands of times is stored once;
//   * numeric tokens are stored typed -- integral values as a zig-zag varint (1-2 bytes
//     for the many small ints), fractional values as an 8-byte IEEE double;
//   * an "include" entry carries the (interned) filename expression and parameters.
// Each entry's type is packed into the low bits of a single varint together with the
// token index, so the common case (a token) costs just that one varint.
//
// Reading is buffer-based and streaming: the whole twin is read into memory once and
// parsed by pointer (no per-byte stream calls), and entries are decoded on demand
// (no intermediate materialisation), with string tokens served as views into the
// buffer. The binary is consumed transparently at the cParser file layer; the scenery
// deserializer is unaware of the format. cParser::Name(), per-.inc node grouping and
// parameter substitution are preserved exactly as in the text path.

namespace scene {

// "eu7" + format version, little-endian via MAKE_ID4 ('e','u','7',<ver>).
// v4: buffer-streamed reader, packed entry tags, zig-zag varint integers.
// v5: tokens stored in original case (lower-cased per consumer at replay) with a quoted
//     flag, so baking is grammar-independent (enables the headless/standalone baker).
// v6: top-level nodes wrapped in a marker carrying their class (infrastructure/visual)
//     and byte span, so the reader can serve or skip a whole node per load pass
//     (enables progressive loading: infrastructure eager, visuals deferred).
// bumping the version invalidates older twins so they are recompiled rather than misread.
constexpr std::uint32_t SCENERYBINARY_MAGIC { MAKE_ID4( 'e', 'u', '7', 6 ) };

// which entries a reader serves in a given load pass; nodes outside the requested class
// are skipped (directives/includes are always served, to keep transform/group state)
enum class scenery_load_pass : std::uint8_t {
    all = 0,            // everything (single-pass load, == pre-v6 behaviour)
    infrastructure = 1, // directives + infrastructure nodes; visual nodes skipped
    visual = 2,         // directives + visual nodes; infrastructure nodes skipped
};

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

// logical kind of an entry as seen by the consumer (the on-disk integer/float split is
// an encoding detail hidden inside the reader/writer)
enum class scenery_entry_type : std::uint8_t {
    token = 0,   // a non-numeric token (name/path/keyword); lower-cased per consumer at replay
    include = 1, // an include directive: pulls in another file with parameters
    number = 2,  // a numeric token
    qtoken = 3,  // a quoted token; case preserved verbatim at replay
};

// a decoded entry handed to the consumer during streaming reads; string fields are
// views into the reader's buffer and are valid until the next read / reader destruction
struct scenery_entry_view {
    scenery_entry_type type { scenery_entry_type::token };
    std::string_view text;                       // token
    double number { 0.0 };                       // number
    std::vector<std::string_view> fileexpr;      // include
    std::vector<std::string_view> params;        // include
};

// accumulates a file's content and serializes it. entries are encoded incrementally into
// a compact byte buffer as they are added (strings interned on the fly), so even a huge
// source file costs only its interned table plus a few bytes per token -- not a heavy
// struct per token -- keeping parallel baking within memory.
class scenery_binary_writer {

public:
    // Quoted marks the token as a quoted span (its case is preserved verbatim at replay)
    void add_token( std::string const &Token, bool Quoted = false );
    void add_number( double Value );
    // Fileexpr is the verbatim filename expression (single token or random set)
    void add_include( std::vector<std::string> const &Fileexpr, std::vector<std::string> const &Params );
    // wrap a top-level node so the reader can serve/skip it per pass. between begin_node()
    // and end_node() the add_*() entries are buffered; end_node() emits a marker (class +
    // byte span) followed by the buffered entries.
    void begin_node();
    void end_node( bool Visual );
    std::size_t entry_count() const { return m_count; }
    // serializes header + string table + encoded entries. returns false on stream failure.
    bool write( std::ostream &Output, scenery_file_kind Kind ) const;

private:
    std::uint32_t intern( std::string const &Text );
    std::ostream &sink(); // current entry sink: node buffer while in a node, else m_entries

    std::unordered_map<std::string, std::uint32_t> m_lookup; // string -> table index
    std::vector<std::string> m_table;                        // interned strings, in order
    std::ostringstream m_entries;                            // encoded entry bytes
    std::ostringstream m_nodebuf;                            // current node's entries (buffered)
    bool m_innode { false };                                 // currently between begin/end_node
    std::size_t m_count { 0 };                               // number of entries
};

// parses a binary twin held in a memory buffer and streams its entries on demand.
// the buffer must outlive the reader (the cParser owns both).
class scenery_binary_reader {

public:
    // validates magic/version and indexes the string table; positions at the first
    // entry. returns false if Buffer is not a valid current-version twin.
    bool open( std::string_view Buffer );

    scenery_file_kind kind() const { return m_kind; }
    // selects which nodes are served vs skipped (default: all). may be changed between
    // reads (e.g. to drive separate eager/visual passes over a re-opened twin).
    void set_pass( scenery_load_pass Pass ) { m_pass = Pass; }
    // decodes the next served entry into Out, skipping node markers / out-of-pass nodes;
    // returns false once all entries are consumed
    bool next( scenery_entry_view &Out );
    bool exhausted() const { return m_cursor >= m_end; }
    // fraction of bytes consumed so far, 0..100, for the loading bar
    int progress() const { return ( m_size == 0 ? 100 : static_cast<int>( ( m_cursor - m_begin ) * 100 / m_size ) ); }

private:
    std::vector<std::string_view> m_table;
    char const *m_begin { nullptr }; // start of the entry section
    char const *m_cursor { nullptr };
    char const *m_end { nullptr };
    std::ptrdiff_t m_size { 0 };      // entry section byte length
    scenery_load_pass m_pass { scenery_load_pass::all };
    scenery_file_kind m_kind { scenery_file_kind::scn };
};

// maps a source scenery filename to the extension of its binary twin
std::string scenerybinary_extension_for( std::string const &Sourcefile );

// Asynchronous baking: serializing and writing a twin is self-contained work, so it is
// offloaded to a small thread pool and overlaps with scene construction. Takes ownership
// of the writer.
void scenerybinary_write_async( std::unique_ptr<scenery_binary_writer> Writer, std::string Path, scenery_file_kind Kind );
// Blocks until every queued async write has finished, logging results on the calling
// (main) thread. Call once the scenario has finished loading.
void scenerybinary_wait_all();

// Headless bake: tokenizes the scenario file and, recursively, every file it includes,
// writing each one's binary twin -- with no scene construction, renderer or window. Used
// by the "-bake" command line mode to precompile a scenery offline. Fresh twins are left
// untouched; only missing/stale ones are (re)compiled. Returns false if the file can't be
// opened.
bool scenerybinary_bake_headless( std::string const &Scenariofile, std::string const &Path, bool Loadtraction );

} // namespace scene
