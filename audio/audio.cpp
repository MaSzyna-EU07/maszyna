/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include <sndfile.h>

#include "audio/audio.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "model/ResourceManager.h"
#include "utilities/utilities.h"

namespace audio {

namespace {

// content of a sound file, mixed down to 16 bit mono
struct decoded_sound {
	std::vector<std::int16_t> samples;
	unsigned int rate { 0 };
	bool mixed { false }; // the file had more than one channel
};

// reads and decodes whole sound file. throws std::runtime_error on failure
// NOTE: uses no simulator state, it's also run by the background decoder threads
decoded_sound decode_sound( std::string const &File ) {

	SF_INFO si;
	si.format = 0;

	SNDFILE *sf = sf_open(File.c_str(), SFM_READ, &si);

	if (sf == nullptr)
		throw std::runtime_error("sound: sf_open failed");

	sf_command(sf, SFC_SET_NORM_FLOAT, nullptr, SF_TRUE);

	std::unique_ptr<float[]> fbuf { new float[si.frames * si.channels] };
	if (sf_readf_float(sf, fbuf.get(), si.frames) != si.frames) {
		sf_close(sf);
		throw std::runtime_error("sound: incomplete file");
	}

	sf_close(sf);

	decoded_sound sound;
	sound.rate = si.samplerate;
	sound.mixed = ( si.channels != 1 );
	sound.samples.resize( si.frames );

	for (size_t i = 0; i < si.frames; i++)
	{
		float accum = 0;
		for (size_t j = 0; j < si.channels; j++)
			accum += fbuf[i * si.channels + j];

		long val = lrintf(accum / si.channels * 32767.0f);
		if (val > 32767)
			val = 32767;
		if (val < -32767)
			val = -32767;
		sound.samples[i] = val;
	}

	return sound;
}

// creates AL buffer holding provided sound. returns: id of the buffer, or null_resource on failure
ALuint upload_sound( decoded_sound const &Sound ) {

	ALuint id { null_resource };
	alGenBuffers(1, &id);
	if (id != null_resource && alIsBuffer(id)) {
		alGetError();
		alBufferData(id, AL_FORMAT_MONO16, Sound.samples.data(), static_cast<ALsizei>(Sound.samples.size() * 2), Sound.rate);
	}
	else {
		id = null_resource;
		const char *str = alGetString(alGetError());
		ErrorLog("sound: failed to create AL buffer: " + (str != nullptr ? std::string(str) : ""));
	}
	return id;
}

} // namespace

openal_buffer::openal_buffer( std::string const &Filename ) :
    name( Filename ) {

	WriteLog("sound: loading file: " + Filename);

	auto const sound { decode_sound( Filename ) };

	rate = sound.rate;

	if (sound.mixed)
		WriteLog("sound: warning: mixing multichannel file to mono");

	id = upload_sound( sound );

	fetch_caption();
}

// sound files decoded by worker threads. AL buffers for them are made on the main thread
struct buffer_manager::decoder {

	struct job {
		audio::buffer_handle handle;
		std::string file;
		decoded_sound sound; // written by the thread decoding the file
		std::string error;
		bool done { false }; // guarded by the mutex
	};

	std::mutex mutex;
	std::condition_variable queued; // wakes workers when files are added
	std::condition_variable decoded; // wakes the main thread waiting for a file
	std::deque<std::shared_ptr<job>> queue; // files no worker has taken yet
	std::vector<std::shared_ptr<job>> pending; // files without AL buffer yet, in order of request. main thread only
	bool stop { false };
	std::vector<std::thread> workers;

	decoder() {
		auto const count { std::clamp<unsigned int>( std::thread::hardware_concurrency(), 2, 9 ) - 1 };
		for( unsigned int idx = 0; idx < count; ++idx ) {
			workers.emplace_back( [ this ]() { work(); } );
		}
	}
	~decoder() {
		{
			std::lock_guard<std::mutex> lock( mutex );
			stop = true;
		}
		queued.notify_all();
		for( auto &worker : workers ) {
			worker.join();
		}
	}

	static void decode( job &Item ) {
		try {
			Item.sound = decode_sound( Item.file );
		}
		catch( std::exception const &Error ) {
			Item.error = Error.what();
		}
	}

	void work() {
		std::unique_lock<std::mutex> lock( mutex );
		while( true ) {
			queued.wait( lock, [ this ]() { return stop || ( false == queue.empty() ); } );
			if( stop ) {
				return;
			}
			auto item { queue.front() };
			queue.pop_front();
			lock.unlock();
			decode( *item );
			lock.lock();
			item->done = true;
			decoded.notify_all();
		}
	}

	// waits until the specified file is decoded. a file no worker has taken yet is decoded right away
	void wait( std::shared_ptr<job> const &Item ) {
		std::unique_lock<std::mutex> lock( mutex );
		if( Item->done ) {
			return;
		}
		auto const lookup { std::find( std::begin( queue ), std::end( queue ), Item ) };
		if( lookup != std::end( queue ) ) {
			queue.erase( lookup );
			lock.unlock();
			decode( *Item );
			lock.lock();
			Item->done = true;
			return;
		}
		decoded.wait( lock, [ &Item ]() { return Item->done; } );
	}

	bool is_done( job const &Item ) {
		std::lock_guard<std::mutex> lock( mutex );
		return Item.done;
	}

	// makes the AL buffer out of the decoded file
	static void finish( job &Item, openal_buffer &Buffer ) {
		if( false == Item.error.empty() ) {
			ErrorLog( "Bad file: failed to load audio file \"" + Item.file + "\" (" + Item.error + ")", logtype::file );
			return;
		}
		Buffer.rate = Item.sound.rate;
		if( Item.sound.mixed ) {
			WriteLog( "sound: warning: mixing multichannel file \"" + Item.file + "\" to mono" );
		}
		Buffer.id = upload_sound( Item.sound );
		Item.sound = {};
		Buffer.fetch_caption();
	}
};

// retrieves sound caption in currently set language
void
openal_buffer::fetch_caption() {

    std::string captionfilename { name };
    captionfilename.erase( captionfilename.rfind( '.' ) ); // obcięcie rozszerzenia
    captionfilename += "-" + Global.asLang + ".txt"; // już może być w różnych językach
    if( true == FileExists( captionfilename ) ) {
        // wczytanie
        std::ifstream inputfile( captionfilename );
        caption.assign( std::istreambuf_iterator<char>( inputfile ), std::istreambuf_iterator<char>() );
    }
}

buffer_manager::buffer_manager() {

    m_buffers.emplace_back( openal_buffer() ); // empty bindings for null buffer
}

buffer_manager::~buffer_manager() {

    // workers are stopped before the buffers go
    m_decoder.reset();

    for( auto &buffer : m_buffers ) {
        if( buffer.id != null_resource ) {
            ::alDeleteBuffers( 1, &buffer.id );
        }
    }
}

// creates buffer object out of data stored in specified file. returns: handle to the buffer or null_handle if creation failed
audio::buffer_handle
buffer_manager::create( std::string const &Filename ) {

    auto filename { ToLower( Filename ) };

    erase_extension( filename );

    audio::buffer_handle lookup { null_handle };
    std::string filelookup;
    if( false == Global.asCurrentDynamicPath.empty() ) {
        // try dynamic-specific sounds first
        lookup = find_buffer( Global.asCurrentDynamicPath + filename );
        if( lookup != null_handle ) {
            return lookup;
        }
        filelookup = find_file( Global.asCurrentDynamicPath + filename );
        if( false == filelookup.empty() ) {
            return emplace( filelookup );
        }
    }
    if( filename.find( '/' ) != std::string::npos ) {
        // if the filename includes path, try to use it directly
        lookup = find_buffer( filename );
        if( lookup != null_handle ) {
            return lookup;
        }
        filelookup = find_file( filename );
        if( false == filelookup.empty() ) {
            return emplace( filelookup );
        }
    }
    // if dynamic-specific and/or direct lookups find nothing, try the default sound folder
    lookup = find_buffer( paths::sounds + filename );
    if( lookup != null_handle ) {
        return lookup;
    }
    filelookup = find_file( paths::sounds + filename );
    if( false == filelookup.empty() ) {
        return emplace( filelookup );
    }
    // if we still didn't find anything, give up
	ErrorLog( "Bad file: failed to locate audio file \"" + Filename + "\"", logtype::file );
    return null_handle;
}

// provides direct access to a specified buffer
audio::openal_buffer const &
buffer_manager::buffer( audio::buffer_handle const Buffer ) const {

    if( ( m_decoder != nullptr )
     && ( false == m_decoder->pending.empty() )
     && ( m_buffers[ Buffer ].id == null_resource ) ) {
        complete( Buffer );
    }
    return m_buffers[ Buffer ];
}

// creates AL buffers for files decoded in the background so far, or for all of them if requested
void
buffer_manager::update( bool const Wait ) {

    if( ( m_decoder == nullptr )
     || ( m_decoder->pending.empty() ) ) {
        return;
    }
    std::vector<std::shared_ptr<decoder::job>> undone;
    for( auto &item : m_decoder->pending ) {
        if( Wait ) {
            m_decoder->wait( item );
        }
        else if( false == m_decoder->is_done( *item ) ) {
            undone.emplace_back( item );
            continue;
        }
        decoder::finish( *item, m_buffers[ item->handle ] );
    }
    m_decoder->pending.swap( undone );
}

// creates AL buffer for a file decoded in the background, waiting for the decoder if needed
void
buffer_manager::complete( audio::buffer_handle const Buffer ) const {

    auto &pending { m_decoder->pending };
    auto const lookup {
        std::find_if(
            std::begin( pending ), std::end( pending ),
            [ Buffer ]( std::shared_ptr<decoder::job> const &Item ) { return Item->handle == Buffer; } ) };
    if( lookup == std::end( pending ) ) {
        return;
    }
    auto const item { *lookup };
    pending.erase( lookup );
    m_decoder->wait( item );
    decoder::finish( *item, m_buffers[ Buffer ] );
}

// places in the bank a buffer containing data stored in specified file. returns: handle to the buffer
audio::buffer_handle
buffer_manager::emplace( std::string Filename ) {

    buffer_handle const handle { m_buffers.size() };
    if( true == Global.AudioAsyncLoad ) {
        // the file is decoded in the background, the AL buffer is made once the data is ready or when the buffer is needed
        WriteLog( "sound: loading file: " + Filename );
        openal_buffer buffer;
        buffer.name = Filename;
        m_buffers.emplace_back( std::move( buffer ) );
        if( m_decoder == nullptr ) {
            m_decoder = std::make_unique<decoder>();
        }
        auto item { std::make_shared<decoder::job>() };
        item->handle = handle;
        item->file = Filename;
        m_decoder->pending.emplace_back( item );
        {
            std::lock_guard<std::mutex> lock( m_decoder->mutex );
            m_decoder->queue.emplace_back( item );
        }
        m_decoder->queued.notify_one();
    }
    else {
        m_buffers.emplace_back( Filename );
    }

    // NOTE: we store mapping without file type extension, to simplify lookups
    erase_extension( Filename );
    m_buffermappings.emplace(
        Filename,
        handle );

    return handle;
}

audio::buffer_handle
buffer_manager::find_buffer( std::string const &Buffername ) const {

    auto const lookup = m_buffermappings.find( Buffername );
    if( lookup != std::end( m_buffermappings ) )
        return lookup->second;
    else
        return null_handle;
}

std::string
buffer_manager::find_file( std::string const &Filename ) const {

    auto const lookup {
        FileExists(
            { Filename },
            { ".ogg", ".flac", ".wav" } ) };

    return lookup.first + lookup.second;
}

} // audio

//---------------------------------------------------------------------------
