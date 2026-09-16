/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>
#include "scene/logicformat.h"

export module eu07.scene.logiccontainer;

export {

// Writing and reading the LOGIC container. The layout it produces is described entirely by
// logicformat.h; this is the part that knows how to get bytes in and out of it.
//
// The writer is deterministic: the same scenery cooked twice produces the same file, byte for
// byte. Strings are interned in first-seen order rather than by a hash, records are written in
// the order the manager holds them, and the padding between sections is zeroed. That is worth
// the small cost, because it is what makes a cooked file comparable - a diff that is empty
// proves a refactor changed nothing.
namespace scene::logic {

class container_writer {

public:
    // interns a string, returning its index. the same text always gets the same index
    std::uint32_t intern( std::string const &Text );
    // adds one event and its targets, already resolved to the registry that holds them
    void add_event( event_record Record, std::vector<target_entry> const &Targets );
    // writes the container. returns false and logs on any failure
    bool write( std::string const &Path ) const;

    std::size_t events() const { return m_events.size(); }

private:
    std::vector<std::string> m_strings;
    std::map<std::string, std::uint32_t> m_interned;
    std::vector<event_record> m_events;
    std::vector<target_entry> m_targets;
};

class container_reader {

public:
    // reads the whole container into memory. refuses anything whose magic, version, schema
    // hash, endianness or size disagrees with what this build expects
    bool open( std::string const &Path );
    // whether this build can read the container, judged from its header and size alone
    static bool readable( std::string const &Path );
    void close();
    bool ready() const { return m_ready; }

    std::size_t events() const { return m_events.size(); }
    event_record const & event( std::size_t const Index ) const { return m_events[ Index ]; }
    std::size_t targets() const { return m_targets.size(); }
    target_entry const & target( std::size_t const Index ) const { return m_targets[ Index ]; }
    // a view into the string blob, valid while the reader is open
    std::string_view string( std::uint32_t const Index ) const;

private:
    bool m_ready { false };
    std::vector<char> m_strings;
    std::vector<string_entry> m_stringtable;
    std::vector<event_record> m_events;
    std::vector<target_entry> m_targets;
};

// the container's name for a scenery file: alongside it, with the extension replaced
std::string container_path( std::string const &Sceneryfile );
// maps the word an event writes as its type onto the cooked kind
event_kind kind_of( std::string const &Type );

} // namespace scene::logic

}
