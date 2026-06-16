#pragma once
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <atomic>

// Thread pool that reads files from disk into memory so the main thread can
// skip disk I/O and parse directly from RAM.  Only covers the binary path
// (E3D); text models fall back to synchronous load.
class AsyncFilePreloader {
public:
    using Buffer = std::shared_ptr<std::vector<char>>;

    AsyncFilePreloader() = default;
    ~AsyncFilePreloader() { stop(); }

    AsyncFilePreloader(const AsyncFilePreloader&) = delete;
    AsyncFilePreloader& operator=(const AsyncFilePreloader&) = delete;

    // Start N worker threads (clamped to [1, hardware_concurrency]).
    void start(int thread_count = 0);

    // Signal workers to stop and join them.  Clears the queue but keeps
    // already-loaded buffers so late callers still get cache hits.
    void stop();

    // Queue a file for background loading.  Silently skips duplicates.
    void queue(std::string path);

    // Return the preloaded buffer for `path`, or nullptr if not ready yet.
    // Does NOT block — callers must fall back to synchronous I/O if nullptr.
    Buffer get(const std::string& path) const;

    // Drop all cached buffers (call after loading finishes to free memory).
    void clear();

    bool is_running() const { return m_running.load(std::memory_order_relaxed); }

private:
    void worker();

    mutable std::mutex          m_mutex;
    std::condition_variable     m_cv;
    std::queue<std::string>     m_pending;
    std::unordered_set<std::string> m_queued;  // prevents duplicate queuing
    std::unordered_map<std::string, Buffer> m_cache;
    std::vector<std::thread>    m_workers;
    std::atomic<bool>           m_running { false };
};

extern AsyncFilePreloader GModelPreloader;
