#pragma once
#include <librealsense2/rs.hpp>
#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace omni {
// Same SDK device/pipeline. At most two 2s bursts, 4 image pairs/s. No I/O in callback.
class DebugFisheyeEvidence {
public:
    DebugFisheyeEvidence(std::string output,std::string request,std::string session);
    ~DebugFisheyeEvidence();
    void offer(const rs2::frameset &,std::uint64_t receive_ns);
private:
    struct Image {std::string name;int burst,sensor,width,height,domain;std::uint64_t frame,receive;double capture;std::vector<unsigned char> pixels;};
    void run();
    std::string output_,request_,session_;
    std::atomic<bool> stop_{false};
    std::atomic<std::uint64_t> until_{0};
    std::atomic<int> burst_{0};
    std::atomic<std::uint64_t> dropped_{0};
    std::uint64_t last_pair_ns_=0;
    std::mutex mutex_;std::deque<Image> images_;std::thread writer_;
};
}
