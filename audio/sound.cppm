module;
#include <chrono>
#include <cstddef>
#include <deque>
#include <sstream>
#include <unordered_map>
#include <array>
#include <ostream>
#include <utility>
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>
#include <string>
#include <list>
#include <stack>
#include <cstdint>
#include <atomic>
#include "audio/audiorenderer_macros.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

export module eu07.audio.sound;
import eu07.audio.audio;
import eu07.glm;
import eu07.utilities.parser;
import eu07.application.uitranscripts;
import eu07.model.resourcemanager;
import eu07.utilities.classes;
import eu07.utilities.names;

export {
/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/



float const EU07_SOUND_GLOBALRANGE { -1.f };
float const EU07_SOUND_CABCONTROLSCUTOFFRANGE { 7.5f };
float const EU07_SOUND_CABANNOUNCEMENTCUTOFFRANGE{ -10.f };
float const EU07_SOUND_BRAKINGCUTOFFRANGE { 100.f };
float const EU07_SOUND_RUNNINGNOISECUTOFFRANGE { 200.f };
float const EU07_SOUND_HANDHELDRADIORANGE { 3500.f };

enum class sound_type {
    single,
    multipart
};

enum sound_parameters {
    range     = 0x1,
    amplitude = 0x2,
    frequency = 0x4,
};

enum sound_flags {
    looping = 0x1, // the main sample will be looping
    exclusive = 0x2, // the source won't dispatch more than one active instance of the sound
    event = 0x80 // sound was activated by an event; we should keep note of the activation state for the update() calls it may receive
};

enum class sound_placement {
    general, // source is equally audible in potential carrier and outside of it
    internal, // source is located inside of the carrier, and less audible when the listener is outside
    engine, // source is located in the engine compartment, less audible when the listener is outside and even less in the cabs
    external, // source is located on the outside of the carrier, and less audible when the listener is inside
    external_ambient, // source is located on the outside of the carrier, with fixed volume
    custom, // source doesn't fit in any standard location or requires custom soundproofing
};

auto const EU07_SOUNDPROOFING_NONE{ 1.f };
auto const EU07_SOUNDPROOFING_SOME{ std::sqrt( 0.65f ) };
auto const EU07_SOUNDPROOFING_STRONG{ std::sqrt( 0.20f ) };
auto const EU07_SOUNDPROOFING_VERYSTRONG{ std::sqrt( 0.01f ) };

// mini controller and audio dispatcher; issues play commands for the audio renderer,
// updates parameters of created audio emitters for the playback duration
// TODO: move to simulation namespace after clean up of owner classes
// openal_source and sound_source refer to each other, so they must share a
// module: neither can forward-declare the other across a module boundary.
// The renderer only ever handles a sound_source through a pointer.
class sound_source;

/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/



// Dodaj brakujące includy

// Definicje dla OpenAL 1.25 jeśli brakuje
#ifndef ALC_CONNECTED
#define ALC_CONNECTED 0x313
#endif

#ifndef ALC_DEFAULT_ALL_DEVICES_SPECIFIER
#endif

#define EU07_SOUND_PROOFINGUSESRANGE


using uint32_sequence = std::vector<std::uint32_t>;

enum class sound_category : unsigned int {
    unknown = 0,
    vehicle,
    local,
    ambient,
};

struct sound_properties {
    glm::dvec3 location;
    float pitch { 1.f };
    sound_category category { sound_category::unknown };
    float gain { 1.f };
    float soundproofing { 1.f };
    std::uintptr_t soundproofing_stamp { ~std::uintptr_t{0} };
};

enum class sync_state {
    good,
    bad_distance,
    bad_resource
};

namespace audio {

struct openal_source {

    friend class openal_renderer;

    using buffer_sequence = std::vector<audio::buffer_handle>;

    ALuint id { audio::null_resource };
    sound_source *controller { nullptr };
    uint32_sequence sounds;
    int sound_index { 0 };
    bool sound_change { false };
    bool is_playing { false };
    bool is_looping { false };
    sound_properties properties;
    sync_state sync { sync_state::good };

    openal_source() = default;

    template <class Iterator_>
    openal_source &bind( sound_source *Controller, uint32_sequence Sounds, Iterator_ First, Iterator_ Last );

    void play();
    void update( double const Deltatime, glm::vec3 const &Listenervelocity );
    void sync_with( sound_properties const &State );
    void stop();
    void loop( bool const State );
    void range( float const Range );
    void pitch( float const Pitch );
    void clear();

private:
    double update_deltatime { 0.0 };
    float pitch_variation { 1.f };
    float sound_range { 50.f };
    glm::vec3 sound_distance { 0.f };
    glm::vec3 sound_velocity { 0.f };
    bool is_in_range { false };
    bool is_multipart { false };
};

class openal_renderer {
public:
    openal_renderer() = default;
    ~openal_renderer();

    audio::buffer_handle fetch_buffer( std::string const &Filename );
    audio::openal_buffer const &buffer( audio::buffer_handle const Buffer ) const;

    bool init();

    template <class Iterator_>
    void insert( Iterator_ First, Iterator_ Last, sound_source *Controller, uint32_sequence Sounds ) {
        m_sources.emplace_back( fetch_source().bind( Controller, Sounds, First, Last ) );
    }

    void erase( sound_source const *Controller );
    void update( double const Deltatime );

    glm::dvec3 cached_camerapos;

private:
    using source_list = std::list<audio::openal_source>;
    using source_sequence = std::stack<ALuint>;

    bool init_caps();
    audio::openal_source fetch_source();

    ALCdevice *m_device { nullptr };
    ALCcontext *m_context { nullptr };
    bool m_ready { false };
    glm::vec3 m_listenervelocity;
    bool m_freeflymode { true };
    bool m_windowopen { true };
    int m_activecab { 0 };

    buffer_manager m_buffers;
    source_list m_sources;
    source_sequence m_sourcespares;

    // OpenAL Soft extension entry points, resolved at runtime via al(c)GetProcAddress
    LPALDEFERUPDATESSOFT m_alDeferUpdatesSOFT { nullptr };
    LPALPROCESSUPDATESSOFT m_alProcessUpdatesSOFT { nullptr };
    LPALCDEVICEPAUSESOFT m_alcDevicePauseSOFT { nullptr };
    LPALCDEVICERESUMESOFT m_alcDeviceResumeSOFT { nullptr };
    LPALCREOPENDEVICESOFT m_alcReopenDeviceSOFT { nullptr };

    bool m_candetectdisconnect { false };
    ALCint m_contextattributes[3] { 0, 0, 0 };
    double m_devicechecktime { 0.0 };

    // ALC_SOFT_system_events: event-driven following of output-device / default-output changes.
    // Preferred over polling because it reliably catches both device removal and default changes
    // (e.g. re-plugging headphones), which alcGetString(DEFAULT_ALL_DEVICES) does not report live.
    LPALCEVENTCONTROLSOFT m_alcEventControlSOFT { nullptr };
    LPALCEVENTCALLBACKSOFT m_alcEventCallbackSOFT { nullptr };
    bool m_usedeviceevents { false };
    std::atomic<bool> m_outputchanged { false }; // set from the (possibly off-thread) event callback
    static void ALC_APIENTRY device_event_callback( ALCenum eventtype, ALCenum devicetype, ALCdevice *device, ALCsizei length, ALCchar const *message, void *userparam ) noexcept;
};

extern openal_renderer renderer;
extern bool event_volume_change;

} // namespace audio

class sound_source {

public:
// constructors
    sound_source( sound_placement const Placement = sound_placement::general, float const Range = 50.f );

// destructor
    ~sound_source();

// methods
    // restores state of the class from provided data stream
    sound_source &
        deserialize( cParser &Input, sound_type const Legacytype, int const Legacyparameters = 0, int const Chunkrange = 100 );
    sound_source &
        deserialize( std::string const &Input, sound_type const Legacytype, int const Legacyparameters = 0 );
    // sends content of the class in legacy (text) format to provided stream
    void
        export_as_text( std::ostream &Output ) const;
    // copies list of sounds from provided source
    sound_source &
        copy_sounds( sound_source const &Source );
    // issues contextual play commands for the audio renderer
    void
        play( int const Flags = 0 );
    // maintains playback of sounds started by event
    void
        play_event();
    // stops currently active play commands controlled by this emitter
    void
        stop( bool const Skipend = false );
    // adjusts parameters of provided implementation-side sound source
    void
        update( audio::openal_source &Source );
    // sets base volume of the emiter to specified value
    sound_source &
        gain( float const Gain );
    // returns current base volume of the emitter
    float
        gain() const;
    // sets base pitch of the emitter to specified value
    sound_source &
        pitch( float const Pitch );
    // Owner setter. The owning vehicle is stored as an opaque pointer so that this
    // interface does not name TDynamicObject: the vehicle module imports this one
    // (vehicles hold sound sources by value), so naming it here would close a cycle
    // in the module graph, and a forward declaration cannot cross a module boundary.
    // sound.cpp casts it back before use.
    void
        owner( void const *Owner );
    // sound source offset setter/getter
    void
        offset( glm::vec3 const Offset );
    glm::vec3 const &
        offset() const;
    // sound source name setter/getter
    void
        name( std::string Name );
    std::string const &
        name() const;
    // playback starting point shift setter/getter
    void
        start( float const Offset );
    float const &
        start() const;
    // custom soundproofing setter/getter
    auto &
        soundproofing();
    auto const &
        soundproofing() const;
    // returns true if there isn't any sound buffer associated with the object, false otherwise
    bool
        empty() const;
    // returns true if the source is emitting any sound; by default doesn't take into account optional ending soudnds
    bool
        is_playing( bool const Includesoundends = false ) const;
    // returns true if the source uses sample table
    bool
        is_combined() const;
    // returns true if specified buffer is one of the optional bookends
    bool
        is_bookend( audio::buffer_handle const Buffer ) const;
    // returns true if the source has optional bookends
    bool
        has_bookends() const;
    // returns location of the sound source in simulation region space
    glm::dvec3 const
        location() const;
    // returns defined range of the sound
    void
        range( float const Range );
    float const
        range() const;

// members
    float m_amplitudefactor { 1.f }; // helper, value potentially used by gain calculation
    float m_amplitudeoffset { 0.f }; // helper, value potentially used by gain calculation
    float m_frequencyfactor { 1.f }; // helper, value potentially used by pitch calculation
    float m_frequencyoffset { 0.f }; // helper, value potentially used by pitch calculation

private:
// types
    struct sound_data {
        audio::buffer_handle buffer;
        int playing; // number of currently active sample instances
    };

    struct chunk_data {
        int threshold; // nominal point of activation for the given chunk
        float fadein; // actual activation point for the given chunk
        float fadeout; // actual end point of activation range for the given chunk
        float pitch; // base pitch of the chunk
    };

    using soundchunk_pair = std::pair<sound_data, chunk_data>;

    using sound_handle = std::uint32_t;
    enum sound_id : std::uint32_t {
        begin,
        main,
        end,
        // 31 bits for index into relevant array, msb selects between the sample table and the basic array
        chunk = 1u << 31
    };

// methods
    // imports member data pair from the provided data stream
    bool
        deserialize_mapping( cParser &Input );
    // imports values for initial, main and ending sounds from provided data stream
    void
        deserialize_soundset( cParser &Input );
    // issues contextual play commands for the audio renderer
    void
        play_basic();
    void
        play_combined();
    // calculates requested sound point, used to select specific sample from the sample table
    float
        compute_combined_point() const;
    void
        update_basic( audio::openal_source &Source );
    void
        update_combined( audio::openal_source &Source );
    void
        update_crossfade( sound_handle const Chunk );
    void
        update_counter( sound_handle const Sound, int const Value );
    void
        update_location();
    // potentially updates area-based gain factor of the source. returns: true if location has changed
    bool
        update_soundproofing();
    void
        insert( sound_handle const Sound );
    template <class Iterator_>
    void
        insert( Iterator_ First, Iterator_ Last ) {
            update_counter( *First, 1 );
            std::vector<audio::buffer_handle> buffers;
            uint32_sequence sounds;
            std::for_each(
                First, Last,
                [&]( sound_handle const &soundhandle ) {
                    buffers.emplace_back( sound( soundhandle ).buffer );
                    sounds.emplace_back( soundhandle ); } );
            audio::renderer.insert( std::begin( buffers ), std::end( buffers ), this, sounds ); }
    sound_data &
        sound( sound_handle const Sound );
    sound_data const &
        sound( sound_handle const Sound ) const;

// members
    void const * m_owner { nullptr }; // optional, the vehicle carrying this sound source (a TDynamicObject)
    glm::vec3 m_offset; // relative position of the source, either from the owner or the region centre
    sound_placement m_placement;
    float m_range { 50.f }; // audible range of the emitted sounds
    std::string m_name;
    int m_flags {}; // requested playback parameters
    sound_properties m_properties; // current properties of the emitted sounds
    float m_pitchvariation {}; // emitter-specific shift for base pitch
    float m_startoffset {}; // emitter-specific shift for playback starting point
    bool m_stop { false }; // indicates active sample instances should be terminated
/*
    bool m_stopend { false }; // indicates active instances of optional ending sound should be terminated
*/
    bool m_playbeginning { true }; // indicates started sounds should be preceeded by opening bookend if there's one
    std::array<sound_data, 3> m_sounds { {} }; // basic sounds emitted by the source, main and optional bookends
    std::vector<soundchunk_pair> m_soundchunks; // table of samples activated when associated variable is within certain range
    bool m_soundchunksempty { true }; // helper, cached check whether sample table is linked with any actual samples
    int m_crossfaderange {}; // range of transition from one chunk to another
    std::optional< std::array<float, 6> > m_soundproofing; // custom soundproofing parameters
};

}  // export -- out-of-line member definitions and explicit
// specialisations are not namespace-scope declarations, so they cannot
// carry `export`; the class itself is exported and that is what counts.
// owner setter
inline void sound_source::owner( void const *Owner ) { m_owner = Owner; }
// sound source offset setter/getter
inline void sound_source::offset( glm::vec3 const Offset ) { m_offset = Offset; }
inline glm::vec3 const & sound_source::offset() const { return m_offset; }
// sound source name setter/getter
inline void sound_source::name( std::string Name ) { m_name = Name; }
inline std::string const & sound_source::name() const { return m_name; }
// playback starting point shift setter/getter
inline void sound_source::start( float const Offset ) { m_startoffset = Offset; }
inline float const & sound_source::start() const { return m_startoffset; }
// custom soundproofing setter/getter
inline auto & sound_source::soundproofing() { return m_soundproofing; }
inline auto const & sound_source::soundproofing() const { return m_soundproofing; }
export {


// collection of sound sources present in the scene
class sound_table : public basic_table<sound_source> {

};

}  // export
