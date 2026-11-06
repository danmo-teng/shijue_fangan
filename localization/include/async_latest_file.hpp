#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace omni {
// A snapshot file already replaces its predecessor. Keep one pending latest
// snapshot, rather than putting disk I/O or an unbounded history on control.
class AsyncLatestFile {
public:
    using Write = std::function<bool(const std::string &, const std::string &)>;
    explicit AsyncLatestFile(std::string path, Write write = atomic_write)
        : path_(std::move(path)), write_(std::move(write)) {
        if (!path_.empty()) thread_ = std::thread([this] { run(); });
    }
    ~AsyncLatestFile() { close(); }
    void submit(std::string snapshot) {
        if (path_.empty()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) return;
        if (pending_) ++coalesced_;
        latest_ = std::move(snapshot); pending_ = true; ready_.notify_one();
    }
    void close() {
        { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
        ready_.notify_one();
        if (thread_.joinable()) thread_.join();
    }
    std::uint64_t errors() const { return errors_.load(); }
    std::uint64_t coalesced() const { return coalesced_.load(); }
    static bool atomic_write(const std::string &path, const std::string &snapshot) {
        const auto temporary = path + ".tmp";
        std::ofstream file(temporary, std::ios::trunc);
        file << snapshot; file.close();
        return file && std::rename(temporary.c_str(), path.c_str()) == 0;
    }
private:
    void run() {
        while (true) {
            std::string snapshot;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this] { return pending_ || stopping_; });
                if (!pending_) break;
                snapshot.swap(latest_); pending_ = false;
            }
            bool ok = false;
            try { ok = write_(path_, snapshot); } catch (...) {}
            if (!ok && ++errors_ == 1)
                std::cerr << "[SNAPSHOT] write failed: " << path_ << "; control continues\n";
        }
    }
    std::string path_, latest_;
    Write write_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::thread thread_;
    bool pending_ = false, stopping_ = false;
    std::atomic<std::uint64_t> errors_{0}, coalesced_{0};
};
}  // namespace omni
