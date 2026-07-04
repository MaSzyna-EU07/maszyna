/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "audio/audio.h"
#include "model/ResourceManager.h"
#include "application/uitranscripts.h"

// Dodaj brakujące includy
#include <list>
#include <stack>
#include <cstdint>
#include <atomic>

// Definicje dla OpenAL 1.25 jeśli brakuje
#ifndef ALC_CONNECTED
#define ALC_CONNECTED 0x313
#endif

#ifndef ALC_DEFAULT_ALL_DEVICES_SPECIFIER
#define ALC_DEFAULT_ALL_DEVICES_SPECIFIER 0x1012
#endif

#define EU07_SOUND_PROOFINGUSESRANGE

class opengl_renderer;
class sound_source;

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

    friend opengl_renderer;

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