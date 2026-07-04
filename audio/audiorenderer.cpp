/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "audio/audiorenderer.h"

#include "audio/sound.h"
#include "utilities/Globals.h"
#include "vehicle/Camera.h"
#include "utilities/Logs.h"
#include "simulation/simulation.h"
#include "vehicle/Train.h"

namespace audio {

openal_renderer renderer;
bool event_volume_change { false };

float const EU07_SOUND_CUTOFFRANGE { 3000.f }; // 2750 m = max expected emitter spawn range, plus safety margin
float const EU07_SOUND_VELOCITYLIMIT { 250 / 3.6f }; // 343 m/sec ~= speed of sound; arbitrary limit of 250 km/h

// potentially clamps length of provided vector to 343 meters
// TBD: make a generic method for utilities out of this
glm::vec3
limit_velocity( glm::vec3 const &Velocity ) {

    auto const ratio { glm::length( Velocity ) / EU07_SOUND_VELOCITYLIMIT };

    return ratio > 1.f ? Velocity / ratio : Velocity;
}

// starts playback of queued buffers
void
openal_source::play() {

    if( id == audio::null_resource ) { return; } // no implementation-side source to match, no point

    ::alSourcePlay( id );

    ALint state;
    ::alGetSourcei( id, AL_SOURCE_STATE, &state );
    is_playing = state == AL_PLAYING;
}

// stops the playback
void
openal_source::stop() {

    if( id == audio::null_resource ) { return; } // no implementation-side source to match, no point

    loop( false );
    // NOTE: workaround for potential edge cases where ::alSourceStop() doesn't set source which wasn't yet started to AL_STOPPED
    int state;
    ::alGetSourcei( id, AL_SOURCE_STATE, &state );
    if( state == AL_INITIAL ) {
        play();
    }
    ::alSourceStop( id );
    is_playing = false;
}

// updates state of the source
void
openal_source::update( double const Deltatime, glm::vec3 const &Listenervelocity ) {

    update_deltatime = Deltatime; // cached for time-based processing of data from the controller
    if( sound_range < 0.0 ) {
        sound_velocity = Listenervelocity; // cached for doppler shift calculation
    }
/*
    // HACK: if the application gets stuck for long time loading assets the audio can gone awry.
    // terminate all sources when it happens to stay on the safe side
    if( Deltatime > 1.0 ) {
        stop();
    }
*/
    if( id != audio::null_resource ) {

        sound_change = false;
        ::alGetSourcei( id, AL_BUFFERS_PROCESSED, &sound_index );
        // for multipart sounds trim away processed buffers until only one remains, the last one may be set to looping by the controller
        // TBD, TODO: instead of change flag move processed buffer ids to separate queue, for accurate tracking of longer buffer sequences
        ALuint discard;
        while( sound_index > 0
            && sounds.size() > 1 ) {
            ::alSourceUnqueueBuffers( id, 1, &discard );
            sounds.erase( std::begin( sounds ) );
            --sound_index;
            sound_change = true;
            // potentially adjust starting point of the last buffer (to reduce chance of reverb effect with multiple, looping copies playing)
            if( controller->start() > 0.f && sounds.size() == 1 ) {
                ALint bufferid;
                ::alGetSourcei(
                    id,
                    AL_BUFFER,
                    &bufferid );
                ALint buffersize;
                ::alGetBufferi( bufferid, AL_SIZE, &buffersize );
                ::alSourcei(
                    id,
                    AL_SAMPLE_OFFSET,
                    static_cast<ALint>( controller->start() * ( buffersize / sizeof( std::int16_t ) ) ) );
            }
        }

        int state;
        ::alGetSourcei( id, AL_SOURCE_STATE, &state );
        is_playing = state == AL_PLAYING;
    }

    // request instructions from the controller
    controller->update( *this );
}

// configures state of the source to match the provided set of properties
void
openal_source::sync_with( sound_properties const &State ) {

    if( id == audio::null_resource ) {
        // no implementation-side source to match, return sync error so the controller can clean up on its end
        sync = sync_state::bad_resource;
        return;
    }
    // velocity
    if( update_deltatime > 0.0
     && sound_range >= 0
     && properties.location != glm::dvec3() ) {
        // after sound position was initialized we can start velocity calculations
        sound_velocity = limit_velocity( ( State.location - properties.location ) / update_deltatime );
    }
    // NOTE: velocity at this point can be either listener velocity for global sounds, actual sound velocity, or 0 if sound position is yet unknown
    ::alSourcefv( id, AL_VELOCITY, glm::value_ptr( sound_velocity ) );

    // location
    sound_distance = State.location - renderer.cached_camerapos;
    if( sound_range != -1 ) {
        // range cutoff check for songs other than 'unlimited'
        // NOTE: since we're comparing squared distances we can ignore that sound range can be negative
        auto const cutoffrange = is_multipart ? EU07_SOUND_CUTOFFRANGE : // we keep multi-part sounds around longer, to minimize restarts as the sounds get out and back in range
		                                        sound_range * 7.5f;
        if( glm::length2( sound_distance ) > std::min( sq(cutoffrange), sq(EU07_SOUND_CUTOFFRANGE) ) ) {
            stop();
            sync = sync_state::bad_distance; // flag sync failure for the controller
            return;
        }
    }
    if( sound_range >= 0 ) {
        // Convert dvec3 to vec3 for OpenAL
        glm::vec3 sound_distance_float = glm::vec3(sound_distance);
        ::alSourcefv( id, AL_POSITION, glm::value_ptr( sound_distance_float ) );
    }
    else {
        // sounds with 'unlimited' or negative range are positioned on top of the listener
        glm::vec3 zero_pos{ 0.f, 0.f, 0.f };
        ::alSourcefv( id, AL_POSITION, glm::value_ptr( zero_pos ) );
    }
    // gain
    auto const gain {
        State.gain
        * State.soundproofing
        * ( State.category == sound_category::vehicle ? Global.VehicleVolume :
            State.category == sound_category::local ? Global.EnvironmentPositionalVolume :
            State.category == sound_category::ambient ? Global.EnvironmentAmbientVolume :
            1.f ) };
    if( State.gain != properties.gain
     || State.soundproofing_stamp != properties.soundproofing_stamp
     || audio::event_volume_change ) {
        // gain value has changed
        ::alSourcef( id, AL_GAIN, gain );
        auto const range { (
            sound_range >= 0 ?
                sound_range :
                5 ) }; // range of -1 means sound of unlimited range, positioned at the listener
        ::alSourcef( id, AL_REFERENCE_DISTANCE, range * ( 1.f / 16.f ) * State.soundproofing );
    }
    if( sound_range != -1 ) {
        auto const rangesquared { sound_range * sound_range };
        auto const distancesquared { static_cast<float>( glm::length2( sound_distance ) ) };
        if( distancesquared > rangesquared
         || false == is_in_range ) {
            // if the emitter is outside of its nominal hearing range or was outside of it during last check
            // adjust the volume to a suitable fraction of nominal value
            auto const fadedistance { sound_range * 0.75f };
            auto const rangefactor {
                std::lerp(
                    1.f, 0.f, std::clamp(
                        ( distancesquared - rangesquared ) / ( fadedistance * fadedistance ),
                        0.f, 1.f ) ) };
            ::alSourcef( id, AL_GAIN, gain * rangefactor );
        }
        is_in_range = distancesquared <= rangesquared;
    }
    // pitch
    if( State.pitch != properties.pitch ) {
        // pitch value has changed
        ::alSourcef( id, AL_PITCH, std::clamp( State.pitch * pitch_variation, 0.1f, 10.f ) );
    }
    // all synced up
    properties = State;
    sync = sync_state::good;
}

// sets max audible distance for sounds emitted by the source
void
openal_source::range( float const Range ) {

    // NOTE: we cache actual specified range, as we'll be giving 'unlimited' range special treatment
    sound_range = Range;

    if( id == audio::null_resource ) { return; } // no implementation-side source to match, no point

    auto const range { (
        Range >= 0 ?
            Range :
            5 ) }; // range of -1 means sound of unlimited range, positioned at the listener
    ::alSourcef( id, AL_REFERENCE_DISTANCE, range * ( 1.f / 16.f ) );
    ::alSourcef( id, AL_ROLLOFF_FACTOR, 1.75f );
}

// sets modifier applied to the pitch of sounds emitted by the source
void
openal_source::pitch( float const Pitch ) {

    pitch_variation = Pitch;
    // invalidate current pitch value to enforce change of next syns
    properties.pitch = -1.f;
}

// toggles looping of the sound emitted by the source
void
openal_source::loop( bool const State ) {

    if( id == audio::null_resource ) { return; } // no implementation-side source to match, no point
    if( is_looping == State ) { return; }

    is_looping = State;
    ::alSourcei(
        id,
        AL_LOOPING,
        State ? AL_TRUE : AL_FALSE);
}

// releases bound buffers and resets state of the class variables
// NOTE: doesn't release allocated implementation-side source
void
openal_source::clear() {

    if( id != audio::null_resource ) {
        // unqueue bound buffers:
        // ensure no buffer is in use...
        stop();
        // ...prepare space for returned ids of unqueued buffers (not that we need that info)...
        std::vector<ALuint> bufferids;
        bufferids.resize( sounds.size() );
        // ...release the buffers...
        ::alSourceUnqueueBuffers( id, bufferids.size(), bufferids.data() );
    }
    // ...and reset reset the properties, except for the id of the allocated source
    // NOTE: not strictly necessary since except for the id the source data typically get discarded in next step
    auto const sourceid { id };
    *this = openal_source();
    id = sourceid;
}



openal_renderer::~openal_renderer() {

    if( m_alcEventCallbackSOFT != nullptr ) { m_alcEventCallbackSOFT( nullptr, nullptr ); } // stop callbacks before teardown

    ::alcMakeContextCurrent( nullptr );

    if( m_context != nullptr ) { ::alcDestroyContext( m_context ); }
    if( m_device != nullptr )  { ::alcCloseDevice( m_device ); }
}

// invoked by OpenAL (possibly on an internal thread) on device events; only flags the change,
// the actual reopen is done on the main thread in update()
void ALC_APIENTRY
openal_renderer::device_event_callback( ALCenum eventtype, ALCenum devicetype, ALCdevice */*device*/, ALCsizei /*length*/, ALCchar const */*message*/, void *userparam ) noexcept {

    if( userparam == nullptr ) { return; }
    if( devicetype == ALC_CAPTURE_DEVICE_SOFT ) { return; } // only care about playback output
    if( eventtype != ALC_EVENT_TYPE_DEFAULT_DEVICE_CHANGED_SOFT
     && eventtype != ALC_EVENT_TYPE_DEVICE_REMOVED_SOFT ) { return; }
    static_cast<openal_renderer*>( userparam )->m_outputchanged.store( true );
}

audio::buffer_handle
openal_renderer::fetch_buffer( std::string const &Filename ) {

    return m_buffers.create( Filename );
}

// provides direct access to a specified buffer
audio::openal_buffer const &
openal_renderer::buffer( audio::buffer_handle const Buffer ) const {

    return m_buffers.buffer( Buffer );
}

// initializes the service
bool
openal_renderer::init() {

    if( true == m_ready ) {
        // already initialized and enabled
        return true;
    }
    if( false == init_caps() ) {
        // basic initialization failed
        return false;
    }
    ::alDistanceModel( AL_INVERSE_DISTANCE_CLAMPED );
    ::alDopplerFactor( 0.25f );
    // all done
    m_ready = true;
    return true;
}

// removes from the queue all sounds controlled by the specified sound emitter
void
openal_renderer::erase( sound_source const *Controller ) {

    auto source { std::begin( m_sources ) };
    while( source != std::end( m_sources ) ) {
        if( source->controller == Controller ) {
            // if the controller is the one specified, kill it
            source->clear();
            if( source->id != audio::null_resource ) {
                // keep around functional sources, but no point in doing it with the above-the-limit ones
                m_sourcespares.push( source->id );
            }
            source = m_sources.erase( source );
        }
        else {
            // otherwise proceed through the list normally
            ++source;
        }
    }
}

// updates state of all active emitters
void
openal_renderer::update( double const Deltatime ) {

    ALenum err = alGetError();
    if (err != AL_NO_ERROR)
    {
        std::string errname;
        if (err == AL_INVALID_NAME)
            errname = "AL_INVALID_NAME";
        else if (err == AL_INVALID_ENUM)
            errname = "AL_INVALID_ENUM";
        else if (err == AL_INVALID_VALUE)
            errname = "AL_INVALID_VALUE";
        else if (err == AL_INVALID_OPERATION)
            errname = "AL_INVALID_OPERATION";
        else if (err == AL_OUT_OF_MEMORY)
            errname = "AL_OUT_OF_MEMORY";
        else
            errname = "unknown";

        ErrorLog("sound: al error: " + errname);
    }

    if (Deltatime == 0.0)
    {
        if (m_alcDevicePauseSOFT)
            m_alcDevicePauseSOFT(m_device);
        return;
    }

    if (m_alcDeviceResumeSOFT)
        m_alcDeviceResumeSOFT(m_device);

    // keep audio on the correct output (OpenAL won't re-route on its own). Reopen playback on the
    // current default output when the active device is lost (headphones unplugged) or the system
    // default output changes (headphones plugged back in, or default switched in Windows). NULL
    // device name selects the current default; context and sources are preserved.
    if( m_alcReopenDeviceSOFT != nullptr && Global.AudioRenderer.empty() ) {
        bool needsreopen{ false };
        if( m_usedeviceevents ) {
            // event-driven: the callback (any thread) flags default-output / device-removal changes
            needsreopen = m_outputchanged.exchange( false );
        }
        else if( m_candetectdisconnect ) {
            // fallback without ALC_SOFT_system_events: poll for a hard disconnect at ~1 Hz
            m_devicechecktime += Deltatime;
            if( m_devicechecktime >= 1.0 ) {
                m_devicechecktime = 0.0;
                ALCint connected{ ALC_TRUE };
                ::alcGetIntegerv( m_device, ALC_CONNECTED, 1, &connected );
                needsreopen = ( connected == ALC_FALSE );
            }
        }
        if( needsreopen ) {
            if( m_alcReopenDeviceSOFT( m_device, nullptr, m_contextattributes ) == ALC_TRUE ) {
                auto const *nowon { (char const *)::alcGetString( nullptr, ALC_DEFAULT_ALL_DEVICES_SPECIFIER ) };
                WriteLog( "sound: audio output changed, reopened playback on \"" + std::string{ nowon ? nowon : "?" } + "\"" );
            }
            else {
                ErrorLog( "sound: audio output changed but reopening on the default device failed (will retry)" );
            }
        }
    }

    // update listener
    // gain
    ::alListenerf( AL_GAIN, Global.AudioVolume );
    // orientation
    glm::dmat4 cameramatrix;
    Global.pCamera.SetMatrix( cameramatrix );
    auto cameraposition = Global.pCamera.Pos + glm::dvec3(Global.viewport_move * glm::mat3(cameramatrix));
    cameramatrix = glm::dmat4(glm::inverse(Global.viewport_rotate)) * cameramatrix;
    auto rotationmatrix { glm::mat3{ cameramatrix } };
    // AL_ORIENTATION expects 6 tightly-packed floats (at, then up). Do NOT reinterpret a
    // glm::vec3[2] here: with GLM_FORCE_DEFAULT_ALIGNED_GENTYPES a glm::vec3 is 16 bytes
    // (padded), so the array is not 6 contiguous floats and the 'up' vector gets read from
    // padding as garbage, corrupting the listener basis (left/right swapped).
    auto const at { glm::vec3{ 0, 0,-1 } * rotationmatrix };
    auto const up { glm::vec3{ 0, 1, 0 } * rotationmatrix };
    ALfloat const orientation[ 6 ] = { at.x, at.y, at.z, up.x, up.y, up.z };
    ::alListenerfv( AL_ORIENTATION, orientation );
    // velocity
    if( Deltatime > 0 ) {
        auto cameramove { cameraposition - cached_camerapos };
        cached_camerapos = cameraposition;
        // intercept sudden user-induced camera jumps...
        // ...from free fly mode change
        if( m_freeflymode != FreeFlyModeFlag ) {
            m_freeflymode = FreeFlyModeFlag;
            cameramove = glm::dvec3{ 0.0 };
        }
        // ...from jump between cab and window/mirror view
        if( m_windowopen != Global.CabWindowOpen ) {
            m_windowopen = Global.CabWindowOpen;
            cameramove = glm::dvec3{ 0.0 };
        }
        // ... from cab change
        if( simulation::Train != nullptr && simulation::Train->iCabn != m_activecab ) {
            m_activecab = simulation::Train->iCabn;
            cameramove = glm::dvec3{ 0.0 };
        }
        // ... from camera jump to another location
        if( glm::length2( cameramove ) > sq(100.0)) { // length2 is better than length for comparing because it does not require sqrt function
            cameramove = glm::dvec3{ 0.0 };
        }
        m_listenervelocity = limit_velocity( cameramove / Deltatime );

        ::alListenerfv( AL_VELOCITY, reinterpret_cast<ALfloat const *>( glm::value_ptr( m_listenervelocity ) ) );
    }

    // update active emitters
    auto source { std::begin( m_sources ) };
    while( source != std::end( m_sources ) ) {
        // update each source
        source->update( Deltatime, m_listenervelocity );
        // if after the update the source isn't playing, put it away on the spare stack, it's done
        if( false == source->is_playing ) {
            source->clear();
            if( source->id != audio::null_resource ) {
                // keep around functional sources, but no point in doing it with the above-the-limit ones
                m_sourcespares.push( source->id );
            }
            source = m_sources.erase( source );
        }
        else {
            // otherwise proceed through the list normally
            ++source;
        }
    }

    // reset potentially used volume change flag
    audio::event_volume_change = false;

    if (m_alProcessUpdatesSOFT && m_alDeferUpdatesSOFT)
    {
        m_alProcessUpdatesSOFT();
        m_alDeferUpdatesSOFT();
    }
}

// returns an instance of implementation-side part of the sound emitter
audio::openal_source
openal_renderer::fetch_source() {

    audio::openal_source newsource;
    if( false == m_sourcespares.empty() ) {
        // reuse (a copy of) already allocated source
        newsource.id = m_sourcespares.top();
        m_sourcespares.pop();
    }
    if( newsource.id == audio::null_resource ) {
        // if there's no source to reuse, try to generate a new one
        ::alGenSources( 1, &newsource.id );
        // Check for errors
        ALenum err = alGetError();
        if (err != AL_NO_ERROR) {
            ErrorLog("sound: failed to generate source, error: " + std::to_string(err));
            newsource.id = audio::null_resource;
        }
    }
    if( newsource.id == audio::null_resource ) {
        // if we still don't have a working source, see if we can sacrifice an already active one
        // under presumption it's more important to play new sounds than keep the old ones going
        // TBD, TODO: for better results we could use range and/or position for the new sound
        // to better weight whether the new sound is really more important
        auto leastimportantsource { std::end( m_sources ) };
        auto leastimportantweight { std::numeric_limits<float>::max() };

        for( auto source { std::begin( m_sources ) }; source != std::cend( m_sources ); ++source ) {

            if( source->id == audio::null_resource
             || true == source->is_multipart
             || false == source->is_playing ) {

                continue;
            }
            auto const sourceweight { (
                source->sound_range != -1 ?
                    source->sound_range * source->sound_range / ( static_cast<float>( glm::length2( source->sound_distance ) ) + 1 ) :
                    std::numeric_limits<float>::max() ) };
            if( sourceweight < leastimportantweight ) {
                leastimportantsource = source;
                leastimportantweight = sourceweight;
            }
        }
        if( leastimportantsource != std::end(m_sources)
         && leastimportantweight < 1.f ) {
            // only accept the candidate if it's outside of its nominal hearing range
            leastimportantsource->stop();
            // HACK: dt of 0 is a roundabout way to notify the controller its emitter has stopped
            leastimportantsource->update( 0, m_listenervelocity );
            leastimportantsource->clear();
            // we should be now free to grab the id and get rid of the remains
            newsource.id = leastimportantsource->id;
            m_sources.erase( leastimportantsource );
        }
    }

    if( newsource.id != audio::null_resource ) {
        // for sources with functional emitter reset emitter parameters from potential last use
        ::alSourcef( newsource.id, AL_PITCH, 1.f );
        ::alSourcef( newsource.id, AL_GAIN, 1.f );
        glm::vec3 zero_pos{ 0.f, 0.f, 0.f };
        ::alSourcefv( newsource.id, AL_POSITION, glm::value_ptr( zero_pos ) );
        ::alSourcefv( newsource.id, AL_VELOCITY, glm::value_ptr( zero_pos ) );
    }

    return newsource;
}

bool
openal_renderer::init_caps() {

    if( ::alcIsExtensionPresent( nullptr, "ALC_ENUMERATION_EXT" ) == AL_TRUE ) {
        // enumeration supported
        WriteLog( "available audio devices:" );
        auto const *devices { ::alcGetString( nullptr, ALC_DEVICE_SPECIFIER ) };
        auto const
            *device { devices },
            *next { devices + 1 };
        while( device && *device != '\0' && next && *next != '\0' ) {
            WriteLog( { device } );
            auto const len { std::strlen( device ) };
            device += len + 1;
            next += len + 2;
        }
    }

    // NOTE: default value of audio renderer variable is empty string, meaning argument of NULL i.e. 'preferred' device
    m_device = ::alcOpenDevice( Global.AudioRenderer.empty() ? nullptr : Global.AudioRenderer.c_str() );
    if( m_device == nullptr ) {
        ErrorLog( "Failed to obtain audio device" );
        return false;
    }

    ALCint versionmajor, versionminor;
    ::alcGetIntegerv( m_device, ALC_MAJOR_VERSION, 1, &versionmajor );
    ::alcGetIntegerv( m_device, ALC_MINOR_VERSION, 1, &versionminor );
    auto const oalversion { std::to_string( versionmajor ) + "." + std::to_string( versionminor ) };

    std::string al_renderer((char *)::alcGetString( m_device, ALC_DEVICE_SPECIFIER ));
    crashreport_add_info("openal_renderer", al_renderer);
    crashreport_add_info("openal_version", oalversion);

    WriteLog(
        "Audio Renderer: " + al_renderer
        + " OpenAL API spec: " + oalversion ); // ALC spec level, always 1.1; not the library version

    WriteLog( "Supported extensions: " + std::string{ (char *)::alcGetString( m_device, ALC_EXTENSIONS ) } );

    ALCint attr[3] = { ALC_MONO_SOURCES, Global.audio_max_sources, 0 }; // request more sounds
    std::copy( std::begin( attr ), std::end( attr ), std::begin( m_contextattributes ) ); // cached for device reopen

    m_context = ::alcCreateContext( m_device, attr );
    if( m_context == nullptr ) {
        ErrorLog( "Failed to create audio context" );
        return false;
    }

    if (!alcMakeContextCurrent(m_context))
    {
        ErrorLog("sound: cannot select context");
        return false;
    }

    // the version reported above is the OpenAL API spec level (always 1.1); the real implementation
    // version string (e.g. "1.1 ALSOFT 1.24.2") is only queryable once a context is current
    if( auto const *libversion { (char const *)::alGetString( AL_VERSION ) } ) {
        crashreport_add_info( "openal_lib_version", libversion );
        WriteLog( "sound: library version: " + std::string{ libversion } );
    }

    // Initialize all extension function pointers
    if (alIsExtensionPresent("AL_SOFT_deferred_updates"))
    {
        m_alDeferUpdatesSOFT = (LPALDEFERUPDATESSOFT)alGetProcAddress("alDeferUpdatesSOFT");
        m_alProcessUpdatesSOFT = (LPALPROCESSUPDATESSOFT)alGetProcAddress("alProcessUpdatesSOFT");
    }
    if (!m_alDeferUpdatesSOFT || !m_alProcessUpdatesSOFT)
        WriteLog("sound: warning: extension AL_SOFT_deferred_updates not found");

    if (alcIsExtensionPresent(m_device, "ALC_SOFT_pause_device"))
    {
        m_alcDevicePauseSOFT = (LPALCDEVICEPAUSESOFT)alcGetProcAddress(m_device, "alcDevicePauseSOFT");
        m_alcDeviceResumeSOFT = (LPALCDEVICERESUMESOFT)alcGetProcAddress(m_device, "alcDeviceResumeSOFT");
    }
    if (!m_alcDevicePauseSOFT || !m_alcDeviceResumeSOFT)
        WriteLog("sound: warning: extension ALC_SOFT_pause_device not found");

    m_candetectdisconnect = ( alcIsExtensionPresent( m_device, "ALC_EXT_disconnect" ) == ALC_TRUE );
    if( alcIsExtensionPresent( m_device, "ALC_SOFT_reopen_device" ) == ALC_TRUE )
        m_alcReopenDeviceSOFT = (LPALCREOPENDEVICESOFT)alcGetProcAddress( m_device, "alcReopenDeviceSOFT" );
    if( !m_alcReopenDeviceSOFT )
        WriteLog( "sound: warning: extension ALC_SOFT_reopen_device not found; audio output device changes won't be followed" );

    // prefer event-driven output following (ALC_SOFT_system_events) over polling: it reliably
    // catches both device removal and default-output changes, incl. re-plugging headphones
    if( m_alcReopenDeviceSOFT != nullptr
     && alcIsExtensionPresent( m_device, "ALC_SOFT_system_events" ) == ALC_TRUE ) {
        m_alcEventControlSOFT = (LPALCEVENTCONTROLSOFT)alcGetProcAddress( m_device, "alcEventControlSOFT" );
        m_alcEventCallbackSOFT = (LPALCEVENTCALLBACKSOFT)alcGetProcAddress( m_device, "alcEventCallbackSOFT" );
        if( m_alcEventControlSOFT != nullptr && m_alcEventCallbackSOFT != nullptr ) {
            m_alcEventCallbackSOFT( &openal_renderer::device_event_callback, this );
            ALCenum const events[]{ ALC_EVENT_TYPE_DEFAULT_DEVICE_CHANGED_SOFT, ALC_EVENT_TYPE_DEVICE_REMOVED_SOFT };
            m_alcEventControlSOFT( 2, events, ALC_TRUE );
            m_usedeviceevents = true;
            WriteLog( "sound: following audio output device changes via ALC_SOFT_system_events" );
        }
    }

    return true;
}

} // audio