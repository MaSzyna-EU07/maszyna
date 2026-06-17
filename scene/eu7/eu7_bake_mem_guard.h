/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

#if defined( _WIN32 )
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#else
#include <cstdio>
#include <unistd.h>
#endif

namespace scene::eu7::bake_parser {

inline constexpr unsigned kDefaultBakeMemLimitGb { 50u };

[[nodiscard]] inline std::uint64_t
process_private_bytes() noexcept {
#if defined( _WIN32 )
    PROCESS_MEMORY_COUNTERS_EX counters {};
    counters.cb = sizeof( counters );
    if( GetProcessMemoryInfo(
            GetCurrentProcess(),
            reinterpret_cast<PROCESS_MEMORY_COUNTERS *>( &counters ),
            sizeof( counters ) ) ) {
        return counters.PrivateUsage;
    }
    return 0;
#else
    std::FILE *status { std::fopen( "/proc/self/statm", "r" ) };
    if( status == nullptr ) {
        return 0;
    }
    long total_pages { 0 };
    long resident_pages { 0 };
    if( std::fscanf( status, "%ld %ld", &total_pages, &resident_pages ) != 2 ) {
        std::fclose( status );
        return 0;
    }
    std::fclose( status );
    long const page_size { sysconf( _SC_PAGESIZE ) };
    if( page_size <= 0 ) {
        return 0;
    }
    return static_cast<std::uint64_t>( resident_pages ) * static_cast<std::uint64_t>( page_size );
#endif
}

// Background poll; abort() when private commit exceeds limit_bytes (0 = disabled).
class bake_mem_guard {
  public:
    explicit bake_mem_guard( std::uint64_t const limit_bytes ) : m_limit_bytes( limit_bytes ) {}

    bake_mem_guard( bake_mem_guard const & ) = delete;
    bake_mem_guard &operator=( bake_mem_guard const & ) = delete;

    ~bake_mem_guard() { stop(); }

    void start() {
        if( m_limit_bytes == 0 || m_thread.joinable() ) {
            return;
        }
        m_stop.store( false, std::memory_order_release );
        m_thread = std::thread( [this]() { run(); } );
    }

    void stop() {
        m_stop.store( true, std::memory_order_release );
        if( m_thread.joinable() ) {
            m_thread.join();
        }
    }

  private:
    void run() const {
        while( !m_stop.load( std::memory_order_acquire ) ) {
            std::uint64_t const used { process_private_bytes() };
            if( used > m_limit_bytes ) {
                double const used_gb { static_cast<double>( used ) / ( 1024.0 * 1024.0 * 1024.0 ) };
                double const limit_gb {
                    static_cast<double>( m_limit_bytes ) / ( 1024.0 * 1024.0 * 1024.0 ) };
                std::fprintf(
                    stdout,
                    "[EU7v2] FATAL: limit pamieci %.1f GB przekroczony (uzyte ~%.1f GB) — abort\n",
                    limit_gb,
                    used_gb );
                std::fflush( stdout );
                std::fprintf(
                    stderr,
                    "[EU7v2] FATAL: limit pamieci %.1f GB przekroczony (uzyte ~%.1f GB) — abort\n",
                    limit_gb,
                    used_gb );
                std::fflush( stderr );
                std::abort();
            }
            std::this_thread::sleep_for( std::chrono::milliseconds { 500 } );
        }
    }

    std::uint64_t const m_limit_bytes { 0 };
    std::atomic<bool> m_stop { false };
    std::thread m_thread;
};

} // namespace scene::eu7::bake_parser
