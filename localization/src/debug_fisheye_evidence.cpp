#include "debug_fisheye_evidence.hpp"
#include <json.hpp>
#include <fstream>
#include <chrono>
#include <cstring>
#include <iostream>

namespace omni {
DebugFisheyeEvidence::DebugFisheyeEvidence(std::string output,std::string request,std::string session)
    :output_(std::move(output)),request_(std::move(request)),session_(std::move(session)),writer_([this]{run();}) {}
DebugFisheyeEvidence::~DebugFisheyeEvidence() {stop_=true;if(writer_.joinable())writer_.join();}
void DebugFisheyeEvidence::offer(const rs2::frameset &frames,std::uint64_t now)
{
    if(now>=until_ || now-last_pair_ns_<250000000)return;
    std::lock_guard<std::mutex> lock(mutex_);
    if(images_.size()>2) {++dropped_;return;}
    last_pair_ns_=now;
    for(int sensor=1;sensor<=2;++sensor) {
        auto frame=frames.get_fisheye_frame(sensor);
        if(!frame || frame.get_bytes_per_pixel()!=1)continue;
        Image image;image.burst=burst_;image.sensor=sensor;image.width=frame.get_width();image.height=frame.get_height();
        image.frame=frame.get_frame_number();image.receive=now;image.capture=frame.get_timestamp();image.domain=frame.get_frame_timestamp_domain();
        image.name="fisheye_b"+std::to_string(image.burst)+"_s"+std::to_string(sensor)+"_f"+std::to_string(image.frame)+".pgm";
        image.pixels.resize(image.width*image.height);
        auto data=static_cast<const unsigned char *>(frame.get_data());
        for(int y=0;y<image.height;++y)std::memcpy(image.pixels.data()+y*image.width,data+y*frame.get_stride_in_bytes(),image.width);
        images_.push_back(std::move(image));
    }
}
void DebugFisheyeEvidence::run()
{
    std::ofstream manifest(output_+"/fisheye_frames.csv");
    manifest<<"session_id,filename,burst,sensor,frame_number,capture_timestamp_ms,timestamp_domain,receive_monotonic_ns,dropped_images\n";
    int seen=0;
    while(true) {
        if(!stop_ && burst_<2) {
            try {
                std::ifstream file(request_);nlohmann::json request;
                if(file) {
                    file>>request;int id=request.at("request_id").get<int>();
                    if(id>seen && id<=2 && id>=1) {
                        const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                        seen=id;burst_=id;until_=now+2000000000ULL;
                    }
                }
            } catch(const std::exception &) {} // Partial/missing control is not a camera or vehicle fault.
        }
        std::deque<Image> batch;
        {std::lock_guard<std::mutex> lock(mutex_);batch.swap(images_);}
        for(const auto &image:batch) {
            std::ofstream file(output_+"/"+image.name,std::ios::binary);
            file<<"P5\n"<<image.width<<' '<<image.height<<"\n255\n";
            file.write(reinterpret_cast<const char *>(image.pixels.data()),image.pixels.size());file.close();
            if(!file) {++dropped_;continue;}
            manifest<<session_<<','<<image.name<<','<<image.burst<<','<<image.sensor<<','<<image.frame<<','
                    <<std::fixed<<image.capture<<','<<image.domain<<','<<image.receive<<','<<dropped_<<'\n';
        }
        manifest.flush();
        if(!manifest) {std::cerr<<"[FISHEYE] evidence log failed; images are not validation evidence\n";break;}
        if(stop_ && batch.empty())break;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}
}
