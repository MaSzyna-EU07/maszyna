#include "stdafx.h"
#include "AsyncFilePreloader.h"
#include <fstream>
#include <algorithm>
#include <thread>

AsyncFilePreloader GModelPreloader;

void AsyncFilePreloader::start(int thread_count) {
    if (m_running.load()) return;

    int n = thread_count > 0 ? thread_count
                              : (int)std::thread::hardware_concurrency();
    n = std::clamp(n, 1, 8);

    m_running = true;
    m_workers.reserve(n);
    for (int i = 0; i < n; ++i)
        m_workers.emplace_back([this] { worker(); });
}

void AsyncFilePreloader::stop() {
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_running = false;
        // drain the queue so workers don't pick up more work
        while (!m_pending.empty()) m_pending.pop();
    }
    m_cv.notify_all();
    for (auto& t : m_workers)
        if (t.joinable()) t.join();
    m_workers.clear();
}

void AsyncFilePreloader::queue(std::string path) {
    std::lock_guard<std::mutex> lk(m_mutex);
    if (!m_running) return;
    if (!m_queued.insert(path).second) return;  // already queued / cached
    m_pending.push(std::move(path));
    m_cv.notify_one();
}

AsyncFilePreloader::Buffer AsyncFilePreloader::get(const std::string& path) const {
    std::lock_guard<std::mutex> lk(m_mutex);
    auto it = m_cache.find(path);
    return (it != m_cache.end()) ? it->second : nullptr;
}

void AsyncFilePreloader::clear() {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_cache.clear();
    m_queued.clear();
}

void AsyncFilePreloader::worker() {
    while (true) {
        std::string path;
        {
            std::unique_lock<std::mutex> lk(m_mutex);
            m_cv.wait(lk, [this] {
                return !m_pending.empty() || !m_running.load(std::memory_order_relaxed);
            });
            if (!m_running && m_pending.empty()) return;
            if (m_pending.empty()) continue;
            path = std::move(m_pending.front());
            m_pending.pop();
        }

        auto buf = std::make_shared<std::vector<char>>();
        {
            std::ifstream f(path, std::ios::binary | std::ios::ate);
            if (f) {
                auto sz = static_cast<std::streamsize>(f.tellg());
                f.seekg(0);
                buf->resize(static_cast<size_t>(sz));
                f.read(buf->data(), sz);
                if (!f) buf->clear();  // read error — fall back to sync load
            }
        }

        {
            std::lock_guard<std::mutex> lk(m_mutex);
            m_cache[path] = std::move(buf);
        }
    }
}
