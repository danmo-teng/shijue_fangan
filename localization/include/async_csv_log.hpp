#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace omni {
// Diagnostic I/O never runs on the Pose/SDK/UART processing thread.
// Overflow or disk failure is reported as incomplete logging, not a control gate.
class AsyncCsvLog {
public:
    explicit AsyncCsvLog(std::string path) : path_(std::move(path)) {
        if (enabled()) thread_ = std::thread([this] { run(); });
    }
    ~AsyncCsvLog() { close(); }
    bool enabled() const { return !path_.empty(); }
    void submit(std::string row) {
        if (!enabled()) return;
        std::lock_guard<std::mutex> guard(mutex_);
        if (stopping_ || failed_ || rows_.size() >= 4096) { ++dropped_; return; }
        rows_.push_back(std::move(row));
        ready_.notify_one();
    }
    void close() {
        { std::lock_guard<std::mutex> guard(mutex_); stopping_ = true; }
        ready_.notify_one();
        if (thread_.joinable()) thread_.join();
    }
    std::uint64_t dropped() const { return dropped_.load(); }
    bool failed() const { return failed_.load(); }
private:
    void run() {
        std::ofstream file(path_, std::ios::trunc);
        if (!file) failed_ = true;
        auto flushed = std::chrono::steady_clock::now();
        while (true) {
            std::deque<std::string> batch;
            bool ending;
            {
                std::unique_lock<std::mutex> guard(mutex_);
                ready_.wait_for(guard, std::chrono::milliseconds(100),
                    [this] { return stopping_ || !rows_.empty(); });
                batch.swap(rows_);
                ending = stopping_;
            }
            for (const auto &row : batch) {
                if (failed_) ++dropped_;
                else { file << row; if (!file) { failed_ = true; ++dropped_; } }
            }
            const auto now = std::chrono::steady_clock::now();
            if (ending || now - flushed >= std::chrono::milliseconds(100)) {
                if (!failed_) { file.flush(); if (!file) failed_ = true; }
                flushed = now;
            }
            if (ending) break;
        }
    }
    std::string path_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::string> rows_;
    std::thread thread_;
    bool stopping_ = false;
    std::atomic<bool> failed_{false};
    std::atomic<std::uint64_t> dropped_{0};
};
}  // namespace omni
