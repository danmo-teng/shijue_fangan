#include "motion_runtime.hpp"
#include "motion_source_native.hpp"
#include "async_csv_log.hpp"
#include "priority_inheritance_mutex.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <mutex>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <system_error>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {
using Json = nlohmann::json;
using Source = omni::NativeMotionSource;
using History = Source::History;
using Yaws = Source::YawHistory;
constexpr double PI = 3.141592653589793238462643383279502884;
constexpr std::uint32_t UNKNOWN = 0xffffffffu;
thread_local std::string last_error;

std::int64_t now_ns() {
    timespec value{}; clock_gettime(CLOCK_MONOTONIC, &value);
    return std::int64_t(value.tv_sec) * 1000000000LL + value.tv_nsec;
}
bool has(const Json &j, const char *key) { return j.is_object() && j.find(key) != j.end(); }
template<class T> T get(const Json &j, const char *key, T fallback) {
    if (!has(j,key) || j.at(key).is_null()) return fallback;
    return j.at(key).get<T>();
}
bool is_true(const Json &j, const char *key, bool fallback=false) {
    return !has(j,key) ? fallback : (j.at(key).is_boolean() && j.at(key).get<bool>());
}
bool truth(const Json &j) {
    if (j.is_null()) return false;
    if (j.is_boolean()) return j.get<bool>();
    if (j.is_number()) return j.get<double>() != 0.;
    return !j.empty();
}
template<class T> void bounded_push(std::deque<T> &values, T value, std::size_t capacity=4096) {
    if (values.size() >= capacity) values.pop_front();
    values.push_back(std::move(value));
}
Json rows(const History &values) {
    Json result=Json::array(); for (const auto &v:values) result.push_back(v); return result;
}
Json rows(const Yaws &values) {
    Json result=Json::array(); for (const auto &v:values) result.push_back(v); return result;
}
template<class Container> Container window_values(const Container &values, double first, double last) {
    Container result; if(values.empty())return result;
    // The same measured support plus one bracketing row on either side.
    auto begin=std::lower_bound(values.begin(),values.end(),first,
        [](const typename Container::value_type &row,double t){return row[0]<t;});
    auto end=std::upper_bound(values.begin(),values.end(),last,
        [](double t,const typename Container::value_type &row){return t<row[0];});
    const auto n=values.size();
    std::size_t lo=std::size_t(begin-values.begin());if(lo)--lo;
    std::size_t hi=std::min(n,std::size_t(end-values.begin())+1);
    lo=std::min(lo,n>2?n-2:std::size_t(0));
    hi=std::max(hi,std::min(n,lo+2));
    for(std::size_t i=lo;i<hi;++i)result.push_back(values[i]);
    return result;
}
std::uint64_t be(const unsigned char *p,int count) {
    std::uint64_t result=0;while(count--)result=(result<<8)|*p++;return result;
}
std::int32_t signed32(const unsigned char *p) {
    const std::uint32_t bits=std::uint32_t(be(p,4)); std::int32_t result;
    std::memcpy(&result,&bits,sizeof result);return result;
}
std::int64_t signed64(const unsigned char *p) {
    const std::uint64_t bits=be(p,8); std::int64_t result;
    std::memcpy(&result,&bits,sizeof result);return result;
}
std::string hex(const unsigned char *p,int count) {
    const char digits[]="0123456789abcdef";std::string result;result.reserve(count*2);
    while(count--){result.push_back(digits[*p>>4]);result.push_back(digits[*p++&15]);}return result;
}
std::uint16_t crc16(const unsigned char *p,std::size_t n) {
    std::uint16_t result=0xffff;while(n--){result^=*p++;for(int i=0;i<8;++i)result=(result&1)?(result>>1)^0xa001:result>>1;}return result;
}
Json decode_feedback(const unsigned char *p,std::size_t n) {
    if(n<19)throw std::invalid_argument("short native motion feedback");
    const auto received=be(p,8);p+=8;n-=8;
    if(n<11||p[0]!=0xd4||p[1]!=0x4d||p[n-1]!=0xe4)throw std::invalid_argument("bad extended envelope");
    const auto length=be(p+4,2);
    if(p[2]!=1||length>160||length+11!=n)throw std::invalid_argument("bad version or length");
    if(crc16(p+2,n-5)!=(p[n-3]|(std::uint16_t(p[n-2])<<8)))throw std::invalid_argument("bad CRC");
    Json j={{"version",p[2]},{"type",p[3]},{"sequence",be(p+6,2)}};
    if(p[3]!=7&&p[3]!=8)return j;
    if(length!=84)throw std::invalid_argument("bad feedback payload length");
    const auto *d=p+8;
    j["boot_epoch"]=hex(d,16);j["reference_epoch"]=be(d+16,4);
    if(p[3]==7){
        j["measurement_id"]=be(d+20,4);j["t0_us"]=be(d+24,8);j["t1_us"]=be(d+32,8);
        j["forward_um_s"]=signed32(d+40);j["left_um_s"]=signed32(d+44);
        j["raw1_u32"]=be(d+48,4);j["raw2_u32"]=be(d+52,4);j["raw3_u32"]=be(d+56,4);
        j["delta1"]=signed32(d+60);j["delta2"]=signed32(d+64);j["delta3"]=signed32(d+68);
        j["flags"]=be(d+72,4);j["source_uncertainty_us"]=be(d+76,4);j["capture_window_us"]=be(d+80,4);
    }else{
        j["publication_id"]=be(d+20,4);j["yaw_mdeg"]=signed64(d+24);j["yaw_receive_us"]=be(d+32,8);
        j["yaw_source_id"]=be(d+40,4);j["yaw_flags"]=be(d+44,4);j["yaw_uncertainty_us"]=be(d+48,4);
        j["rate_mdps"]=signed32(d+52);j["rate_receive_us"]=be(d+56,8);j["rate_source_id"]=be(d+64,4);
        j["rate_flags"]=be(d+68,4);j["rate_uncertainty_us"]=be(d+72,4);j["publish_us"]=be(d+76,8);
    }
    j["receive_monotonic_ns"]=received;j["capture_time_qualified"]=false;j["sdk_time_qualified"]=false;
    return j;
}

// Direct port of TimedMeasurements. Clock uncertainty remains diagnostic; no
// sample is rejected for age or unknown drift and no synthetic sync is added.
struct ClockMap {
    Json identity=nullptr;std::deque<Json> syncs;
    bool have_wheel=false;Json wheel_epoch=nullptr;std::uint32_t wheel_id=0;
    std::uint64_t wheel_samples=0,duplicate_wheel=0;
    void set_identity(const Json &sample) {
        if(!identity.is_null()&&identity.at("boot_epoch")!=sample.at("boot_epoch")){
            syncs.clear();have_wheel=false;
        }
        identity=sample;
    }
    bool v13()const {return get<int>(identity,"protocol_major",-1)==1&&get<int>(identity,"protocol_minor",-1)==3;}
    bool valid_yaw(const Json &sample)const {
        auto flags=sample.at("yaw_flags").get<std::uint32_t>();
        return bool(flags&1)&&!(flags&(v13()?32u:(4u|8u|32u|1024u)));
    }
    bool sync(const Json &sample) {
        if(identity.is_null()||sample.at("boot_epoch")!=identity.at("boot_epoch")||get<int>(sample,"epoch_valid",0)!=1)return false;
        try{
            auto before=sample.at("host_write_before_ns").get<std::int64_t>();
            auto after=sample.at("receive_monotonic_ns").get<std::int64_t>();
            auto prepared=sample.at("response_prepare_us").get<std::int64_t>();
            auto receive=sample.at("receive_event_us").get<std::int64_t>();
            if(after<before||prepared<receive)return false;
            auto q=sample.at("tick_resolution_us").get<std::int64_t>()*1000;
            auto lower=prepared*1000-after-q,upper=receive*1000-before+q;
            if(lower>upper)return false;
            bounded_push(syncs,Json{{"lower",lower},{"upper",upper},{"host_ns",after},
                {"nonce",sample.at("nonce")},{"boot_epoch",sample.at("boot_epoch")}},16);
            return true;
        }catch(const std::exception &){return false;}
    }
    Json status()const {
        Json missing=Json::array();
        if(identity.is_null()||!truth(identity.value("epoch_valid",Json(false))))missing.push_back("boot_epoch_invalid");
        if(!identity.is_null()&&!is_true(identity,"interface_compatible",true))missing.push_back("device_interface_not_reviewed_or_mismatched");
        if(syncs.empty())missing.push_back("no_matched_clock_exchange");
        if(!identity.is_null()&&get<std::uint32_t>(identity,"drift_bound_ppm",UNKNOWN)==UNKNOWN)missing.push_back("mcu_clock_drift_bound_unknown");
        return Json{{"qualified",missing.empty()},{"missing",missing},{"exchanges",syncs.size()},
            {"last_exchange_host_ns",syncs.empty()?Json(nullptr):syncs.back().at("host_ns")}};
    }
    struct Mapped {Json time=nullptr,error=nullptr,reasons=Json::array();};
    Mapped map(std::int64_t tick)const {
        auto state=status();Mapped result;result.reasons=state.at("missing");if(syncs.empty())return result;
        const auto &s=syncs.back();auto lo=s.at("lower").get<std::int64_t>(),hi=s.at("upper").get<std::int64_t>();
        // Python // rounds down, including negative MCU-host offset sums.
        auto sum=lo+hi;auto midpoint=sum/2;if(sum<0&&sum%2)--midpoint;
        auto estimate=tick*1000-midpoint;result.time=estimate;
        if(!state.at("qualified").get<bool>())return result;
        double error=double(hi-lo)/2.+std::abs(double(estimate-s.at("host_ns").get<std::int64_t>()))*
            identity.at("drift_bound_ppm").get<double>()/1e6;
        result.error=std::int64_t(std::ceil(error));result.reasons=Json::array();return result;
    }
    Json wheel(const Json &s) {
        bool same=!identity.is_null()&&s.at("boot_epoch")==identity.at("boot_epoch")&&
            truth(identity.value("epoch_valid",Json(false)))&&is_true(identity,"interface_compatible",true);
        Json result={{"mcu_epoch",same?s.at("boot_epoch"):Json(nullptr)},
            {"reference_epoch",s.at("reference_epoch")},
            {"feedback_protocol",identity.is_null()?Json(nullptr):Json::array({identity.value("protocol_major",Json(nullptr)),identity.value("protocol_minor",Json(nullptr))})},
            {"boot_epoch_valid",!identity.is_null()&&truth(identity.value("epoch_valid",Json(false)))},
            {"host_action_id_at_receive",s.value("host_action_id_at_receive",Json(nullptr))},
            {"host_chassis_session_id",s.value("host_chassis_session_id",Json(nullptr))},
            {"receive_monotonic_ns",s.at("receive_monotonic_ns")},{"capture_time_qualified",false},
            {"time_uncertainty_ns",nullptr},{"raw_mcu_sample",s},
            {"source_time_semantics","MCU event; not automatically physical acquisition"},{"clock_status",status()}};
        auto id=s.at("measurement_id").get<std::uint32_t>();const auto &epoch=s.at("boot_epoch");
        if(have_wheel&&epoch==wheel_epoch){
            if(id==wheel_id){++duplicate_wheel;return nullptr;}
            std::uint32_t step=id-wheel_id;
            if(!step||step>=0x80000000u){result["measurement_valid"]=false;result["qualification_reasons"]=Json::array({"out_of_order_wheel"});return result;}
        }
        have_wheel=true;wheel_epoch=epoch;wheel_id=id;++wheel_samples;
        auto start=map(s.at("t0_us").get<std::int64_t>()),end=map(s.at("t1_us").get<std::int64_t>());
        Json reasons=start.reasons;
        for(const auto &r:end.reasons)if(std::find(reasons.begin(),reasons.end(),r)==reasons.end())reasons.push_back(r);
        auto flags=s.at("flags").get<std::uint32_t>(),source_error=s.at("source_uncertainty_us").get<std::uint32_t>();
        if(source_error==UNKNOWN||!(flags&2))reasons.push_back("wheel_capture_time_bound_unknown_including_rear_EXTI");
        if(!same)reasons.push_back("boot_epoch_invalid");
        double dt=(s.at("t1_us").get<double>()-s.at("t0_us").get<double>())/1e6;
        bool valid=(flags&1)&&!(flags&(v13()?(8|16|32):(4|8|32|256|512|1024)))&&dt>0;
        Json calculated=nullptr;double forward=s.at("forward_um_s").get<double>()/1e6,left=s.at("left_um_s").get<double>()/1e6;
        if(dt>0){
            double k=PI*.070/1061.268,d1=s.at("delta1").get<double>(),d2=s.at("delta2").get<double>(),d3=s.at("delta3").get<double>();
            double x=k*(d1-d2)/(std::sqrt(3.)*dt),y=k*(d1+d2-2*d3)/(3*dt);
            calculated=Json::array({x,y,0.});
            if(std::max(std::abs(forward-x),std::abs(left-y))>.00001){valid=false;reasons.push_back("lower_body_velocity_disagrees_with_F405_raw_deltas");}
        }
        Json uncertainty=nullptr;
        if(reasons.empty())uncertainty=std::max(start.error.get<std::int64_t>(),end.error.get<std::int64_t>())+std::int64_t(source_error)*1000;
        result["source"]="measured_wheel_feedback";result["sample_id"]=id;result["v_body_mps"]=Json::array({forward,left,0.});
        result["host_recomputed_v_body_mps"]=calculated;result["alignment_clock"]="mcu_event";
        result["alignment_start_ns"]=s.at("t0_us").get<std::int64_t>()*1000;
        result["alignment_end_ns"]=s.at("t1_us").get<std::int64_t>()*1000;
        result["interval_start_ns"]=start.time;result["interval_end_ns"]=end.time;result["measurement_valid"]=valid;
        result["capture_time_qualified"]=valid&&reasons.empty()&&!uncertainty.is_null()&&uncertainty.get<std::int64_t>()<=5000000;
        result["time_uncertainty_ns"]=uncertainty;result["qualification_reasons"]=reasons;
        result["slip_state"]=(flags&128)?"unknown":"not_certified_by_this_interface";return result;
    }
};

struct Datagram {
    int fd=-1;sockaddr_un address{};socklen_t length=0;
    Datagram()=default;Datagram(const Datagram&)=delete;Datagram &operator=(const Datagram&)=delete;
    ~Datagram(){if(fd>=0)::close(fd);}
    void open(const std::string &name) {
        if(name.empty())return;
        std::string path=name;if(path[0]=='@')path[0]='\0';
        if(path.size()>sizeof address.sun_path||path.size()<2||path[0]!='\0')throw std::invalid_argument("nonempty abstract local address required");
        fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(fd<0)throw std::runtime_error(std::strerror(errno));
        address.sun_family=AF_UNIX;std::memcpy(address.sun_path,path.data(),path.size());
        length=static_cast<socklen_t>(offsetof(sockaddr_un,sun_path)+path.size());
    }
    bool send(const std::string &data) {
        if(fd<0)return false;
        ssize_t sent;do{sent=sendto(fd,data.data(),data.size(),MSG_DONTWAIT|MSG_NOSIGNAL,reinterpret_cast<sockaddr*>(&address),length);}while(sent<0&&errno==EINTR);
        return sent==static_cast<ssize_t>(data.size());
    }
};

using RuntimeMutex = omni::PriorityInheritanceMutex;

class Runtime {
public:
    explicit Runtime(const Json &config):session_(get<std::string>(config,"session_id","")),
        pose_session_(get<std::string>(config,"pose_session_id",session_)),
        policy_session_(get<std::string>(config,"sdk_policy_session_id",session_)) {
        try {
            feedback_fd_=duplicate(get<int>(config,"feedback_fd",-1));
            pose_fd_=duplicate(get<int>(config,"pose_fd",-1));
            wheel_policy_fd_=duplicate(get<int>(config,"wheel_policy_fd",-1));
            wake_=eventfd(0,EFD_NONBLOCK|EFD_CLOEXEC);if(wake_<0)throw std::runtime_error(std::strerror(errno));
            control_.open(get<std::string>(config,"control_address",""));policy_.open(get<std::string>(config,"sdk_policy_address",""));
            auto root=get<std::string>(config,"log_root",get<std::string>(config,"root",""));
            if(!root.empty()&&root.back()!='/')root+='/';
            feedback_log_.reset(new omni::AsyncCsvLog(root.empty()?"":root+"motion_feedback.jsonl",true));
            pose_log_.reset(new omni::AsyncCsvLog(root.empty()?"":root+"pose_motion.jsonl",true));
            control_log_.reset(new omni::AsyncCsvLog(root.empty()?"":root+"control_pose.jsonl",true));
            events_log_.reset(new omni::AsyncCsvLog(root.empty()?"":root+"native_motion_events.jsonl",true));
            thread_=std::thread([this]{run();});
        } catch(...) { cleanup_fds();throw; }
    }
    ~Runtime(){close();}
    void close() {
        bool expected=false;if(!stopping_.compare_exchange_strong(expected,true))return;
        if(wake_>=0){std::uint64_t one=1;const auto ignored=::write(wake_,&one,sizeof one);(void)ignored;}
        if(thread_.joinable())thread_.join();cleanup_fds();
        if(feedback_log_)feedback_log_->close();if(pose_log_)pose_log_->close();
        if(control_log_)control_log_->close();if(events_log_)events_log_->close();
    }
    Json command(const Json &request) {
        std::lock_guard<RuntimeMutex> guard(mutex_);
        const auto kind=request.at("kind").get<std::string>();
        const auto &p=has(request,"payload")?request.at("payload"):request;
        Json result={{"ok",true}};
        if(kind=="context")context(p);
        else if(kind=="configure") {reference_=p;motion_.configure(p);}
        else if(kind=="pose")ingest_pose(p);
        else if(kind=="yaw")ingest_yaw(p.at("time_ns").get<std::int64_t>(),p.at("yaw_deg").get<double>(),p.at("source"));
        else if(kind=="wheel"){motion_.wheel(p,yaws_);++wheels_ingested_;}
        else if(kind=="wheel_suppression"){
            motion_.set_wheel_suppressed(p.at("suppressed").get<bool>(),get<std::string>(p,"reason","safe_zone_low_speed"));
        }
        else if(kind=="competition_wheel_policy")apply_competition_wheel_policy(p);
        else if(kind=="walls"){
            History raw=poses_;if(has(p,"raw_poses")){raw.clear();for(const auto &row:p.at("raw_poses"))raw.push_back(
                Source::Row{{row.at(0).get<double>(),row.at(1).get<double>(),row.at(2).get<double>(),row.at(3).get<double>()}});}
            motion_.observe_walls(get<std::string>(p,"scan_id",""),axes(p,"axes"),get<std::int64_t>(p,"reference_ns",-1),
                p.value("center_xy",Json(nullptr)),raw,get<bool>(p,"reacquired",false));
        }
        else if(kind=="field_reference"){
            reference_=p.at("reference");revision_=get<std::int64_t>(p,"revision",revision_+1);
            offset_={{0.,0.}};has_anchor_=get<bool>(p,"has_anchor",false);last_fit_=nullptr;
            motion_.configure(reference_);
        }
        else if(kind=="clear_reference"){
            revision_=get<std::int64_t>(p,"revision",revision_+1);reference_=nullptr;offset_={{0.,0.}};has_anchor_=false;
            motion_.reference=nullptr;motion_.history.clear();last_fit_=nullptr;
        }
        else if(kind=="invalidate")invalidate(get<std::string>(p,"reason","reference_invalidated"),get<std::int64_t>(p,"revision",revision_+1));
        else if(kind=="lidar_state"){
            const bool changed=has(p,"enabled")&&get<bool>(p,"enabled",lidar_enabled_)!=lidar_enabled_;
            if(has(p,"enabled"))lidar_enabled_=p.at("enabled").get<bool>();
            if(has(p,"connected"))lidar_connected_=p.at("connected").get<bool>();
            if(has(p,"error"))lidar_error_=p.at("error");
            if(has(p,"revision"))revision_=p.at("revision").get<std::int64_t>();else if(changed)++revision_;
            if(get<bool>(p,"reset_wall_fixes",changed)){motion_.wall_fixes=0;last_fit_=nullptr;}
            if(has(p,"scan_count"))scan_count_=p.at("scan_count").get<std::uint64_t>();
        }
        else if(kind=="apply_wall")result=apply_wall(p);
        else throw std::invalid_argument("unknown native motion command: "+kind);
        publish();return result;
    }
    Json snapshot(const Json &query) {
        const auto kind=get<std::string>(query,"kind","state");
        if(kind!="state"&&kind!="window"&&kind!="histories")throw std::invalid_argument("unknown native motion snapshot kind");
        double first=get<double>(query,"first_ns",-std::numeric_limits<double>::infinity());
        double last=get<double>(query,"last_ns",std::numeric_limits<double>::infinity());
        const bool latest=get<bool>(query,"latest",false),consume=get<bool>(query,"consume_events",false);
        Json result;History raw_copy,selected_copy,history_copy,wheel_copy,pending_copy;
        Yaws yaw_copy,native_yaw_copy;std::deque<Json> event_copy;
        {
            std::lock_guard<RuntimeMutex> guard(mutex_);
            // Only bounded native values and the small endpoint metadata are
            // copied under the hot lock. Thousands of row JSON allocations,
            // string encoding and Python decoding occur after releasing it.
            result=state();event_copy=motion_.events;
            if(kind=="window"){
                const auto &selected=motion_.history.empty()?poses_:motion_.history;
                if(latest&&!selected.empty())first=last=selected.back()[0];
                raw_copy=window_values(poses_,first,last);selected_copy=window_values(selected,first,last);
                yaw_copy=window_values(yaws_,first,last);history_copy=window_values(motion_.history,first,last);
            }else if(kind=="histories"){
                raw_copy=poses_;yaw_copy=yaws_;history_copy=motion_.history;wheel_copy=motion_.wheels;
                native_yaw_copy=motion_.native_yaws;pending_copy=motion_.pending;
            }
            if(consume)motion_.events.clear();
        }
        result["events"]=Json(event_copy);
        if(kind=="window"){
            result["raw_poses"]=rows(raw_copy);result["poses"]=rows(selected_copy);result["yaws"]=rows(yaw_copy);
            result["history"]=rows(history_copy);result["motion_history"]=result["history"];
        }else if(kind=="histories"){
            result["poses"]=rows(raw_copy);result["yaws"]=rows(yaw_copy);result["history"]=rows(history_copy);
            result["motion_history"]=result["history"];result["wheels"]=rows(wheel_copy);
            result["native_yaws"]=rows(native_yaw_copy);result["pending"]=rows(pending_copy);
        }
        return result;
    }
private:
    RuntimeMutex mutex_;std::thread thread_;std::atomic<bool> stopping_{false};
    int feedback_fd_=-1,pose_fd_=-1,wheel_policy_fd_=-1,feedback_connection_=-1,pose_connection_=-1,wake_=-1;
    Datagram control_,policy_;std::string session_,pose_session_,policy_session_;
    std::unique_ptr<omni::AsyncCsvLog> feedback_log_,pose_log_,control_log_,events_log_;
    Source motion_;History poses_;Yaws yaws_;ClockMap clock_;
    Json reference_=nullptr,last_sample_=nullptr,last_raw_pose_=Json::object(),last_yaw_=nullptr,last_fit_=nullptr;
    Source::Vec2 offset_{{0.,0.}};std::int64_t revision_=0,last_scan_ns_=-1;
    bool has_anchor_=false,lidar_enabled_=true,lidar_connected_=false,timed_yaw_seen_=false;
    Json lidar_error_=nullptr,last_yaw_epoch_=nullptr;std::uint32_t last_yaw_id_=0;bool have_yaw_id_=false;
    Json native_error_=nullptr,pose_error_=nullptr,thread_error_=nullptr;
    std::uint64_t frames_=0,wheel_frames_=0,gyro_frames_=0,decode_errors_=0,poses_ingested_=0,wheels_ingested_=0,yaws_ingested_=0;
    std::uint64_t scan_count_=0,fit_count_=0,reacquisitions_=0,control_updates_=0,packets_sent_=0,output_drops_=0;
    Json last_packet_=nullptr,last_ready_packet_=nullptr,last_key_=nullptr;
    bool last_policy_pause_=false,policy_error_reported_=false;std::string last_policy_reason_="measured_wheel_input_enabled";
    Json scheduling_=Json::object(),competition_wheel_policy_=nullptr;
    static int duplicate(int fd) {if(fd<0)return -1;int result=fcntl(fd,F_DUPFD_CLOEXEC,0);if(result<0)throw std::runtime_error(std::strerror(errno));return result;}
    void cleanup_fds() {
        for(auto fd:{feedback_connection_,pose_connection_,feedback_fd_,pose_fd_,wheel_policy_fd_,wake_})if(fd>=0)::close(fd);
        feedback_connection_=pose_connection_=feedback_fd_=pose_fd_=wheel_policy_fd_=wake_=-1;
    }
    static std::vector<std::string> axes(const Json &p,const char *key) {
        return has(p,key)?p.at(key).get<std::vector<std::string>>():std::vector<std::string>{};
    }
    void event(const std::string &name,Json fields=Json::object()) {
        fields["event"]=name;fields["timestamp_monotonic_ns"]=now_ns();events_log_->submit(fields.dump()+"\n");
    }
    void restore_competition_wheel_policy() {
        if(!competition_wheel_policy_.is_null())motion_.set_wheel_suppressed(
            competition_wheel_policy_.at("suppressed").get<bool>(),competition_wheel_policy_.at("reason").get<std::string>());
    }
    void apply_competition_wheel_policy(const Json &packet) {
        // A local business-phase request, never an MCU handshake/contact claim
        // or a sample-age qualification. One runner publishes only transitions.
        if(get<int>(packet,"schema",0)!=1||get<std::string>(packet,"kind","")!="competition_wheel_policy"||
                get<std::string>(packet,"session_id","")!=session_||
                !has(packet,"suppressed")||!packet.at("suppressed").is_boolean())return;
        const auto reason=get<std::string>(packet,"reason","safe_zone_low_speed");
        competition_wheel_policy_=packet;competition_wheel_policy_["reason"]=reason;
        restore_competition_wheel_policy();
        event("competition_wheel_policy_applied",competition_wheel_policy_);
    }
    bool receive_wheel_policy() {
        if(wheel_policy_fd_<0)return false;
        char data[4096];ssize_t count;
        do{count=recv(wheel_policy_fd_,data,sizeof data,MSG_DONTWAIT|MSG_TRUNC);}while(count<0&&errno==EINTR);
        if(count<0){if(errno==EAGAIN||errno==EWOULDBLOCK)return false;throw std::runtime_error(std::strerror(errno));}
        if(count>static_cast<ssize_t>(sizeof data))throw std::invalid_argument("oversize local wheel policy");
        apply_competition_wheel_policy(Json::parse(data,data+count));publish();return true;
    }
    void invalidate(const std::string &reason,std::int64_t revision) {
        reference_=nullptr;revision_=revision;yaws_.clear();last_fit_=nullptr;motion_=Source();has_anchor_=false;
        restore_competition_wheel_policy();
        event("reference_invalidated",Json{{"reason",reason}});
    }
    void ingest_pose(const Json &sample) {
        last_raw_pose_=sample;++poses_ingested_;pose_log_->submit(sample.dump()+"\n");
        for(const char *key:{"x_m","y_m","yaw_rad"})if(!has(sample,key)||!sample.at(key).is_number()||!std::isfinite(sample.at(key).get<double>()))return;
        if(!last_sample_.is_null()){
            if(sample.at("session_id")!=last_sample_.at("session_id")){
                reference_=nullptr;++revision_;poses_.clear();motion_=Source();
                restore_competition_wheel_policy();
            }else if(sample.at("frame_number").get<std::uint64_t>()<=last_sample_.at("frame_number").get<std::uint64_t>())return;
        }
        auto t=sample.at("timestamp_monotonic_ns").get<std::int64_t>();
        if(!poses_.empty()&&t<=poses_.back()[0])t=sample.at("receive_monotonic_ns").get<std::int64_t>();
        if(!poses_.empty()&&t<=poses_.back()[0])return;
        Json adjusted=sample;adjusted["timestamp_monotonic_ns"]=t;motion_.pose(adjusted);
        if(get<int>(sample,"tracker_confidence",0)<=0)return;
        bounded_push(poses_,Source::Row{{double(t),sample.at("x_m").get<double>(),sample.at("y_m").get<double>(),sample.at("yaw_rad").get<double>()}});
        last_sample_=sample;
    }
    void ingest_yaw(std::int64_t t,double degrees,const Json &source) {
        if(!std::isfinite(degrees))return;
        if(!yaws_.empty()&&t<yaws_.back()[0])return;
        if(!yaws_.empty()&&t==yaws_.back()[0])yaws_.pop_back();
        bounded_push(yaws_,std::array<double,2>{{double(t),degrees*PI/180.}});last_yaw_=source;
        motion_.yaw(t,degrees*PI/180.,source);++yaws_ingested_;
    }
    void context(const Json &event_data) {
        // Accept one already matched original client event; raw ID/SYNC bytes
        // are never promoted to clock/firmware context by this receiver.
        const auto kind=event_data.at("kind").get<std::string>();const auto &s=event_data.at("sample");
        if(kind=="clock_context"){
            if(has(s,"identity")&&!s.at("identity").is_null()){
                clock_.set_identity(s.at("identity"));clock_.syncs.clear();
                if(has(s,"syncs"))for(const auto &sync:s.at("syncs"))bounded_push(clock_.syncs,sync,16);
            }
        }else if(kind=="timed_identity")clock_.set_identity(s);
        else if(kind=="timed_clock_sync")clock_.sync(s);
        else if(kind=="device_identity_changed"){
            invalidate("device_identity_changed",revision_+1);timed_yaw_seen_=false;have_yaw_id_=false;
        }else if(kind=="timed_wheel_diagnostic"){
            auto measured=clock_.wheel(s);if(!measured.is_null()){motion_.wheel(measured,yaws_);++wheels_ingested_;}
        }else if(kind=="timed_gyro_diagnostic"){
            if(clock_.identity.is_null()||s.at("boot_epoch")!=clock_.identity.at("boot_epoch"))return;
            auto id=s.at("yaw_source_id").get<std::uint32_t>();
            if(have_yaw_id_&&last_yaw_epoch_==s.at("boot_epoch")&&last_yaw_id_==id)return;
            if(!clock_.valid_yaw(s))return;
            auto mapped=clock_.map(s.at("yaw_receive_us").get<std::int64_t>());
            auto t=!mapped.time.is_null()&&mapped.time.get<std::int64_t>()>=0?mapped.time.get<std::int64_t>():s.at("receive_monotonic_ns").get<std::int64_t>();
            ingest_yaw(t,s.at("yaw_mdeg").get<double>()/1000.,Json{{"source","versioned_primary_yaw"},{"raw",s},
                {"time_qualified",false},{"time_uncertainty_ns",mapped.error},{"time_diagnostics",mapped.reasons}});
            last_yaw_epoch_=s.at("boot_epoch");last_yaw_id_=id;have_yaw_id_=true;timed_yaw_seen_=true;
        }else if(kind=="imu"&&!timed_yaw_seen_&&get<bool>(s,"wire_valid",false)){
            ingest_yaw(s.at("timestamp_monotonic_ns").get<std::int64_t>(),s.at("yaw_deg").get<double>(),
                Json{{"source","legacy_primary_yaw_receive"},{"raw",s},{"time_qualified",false}});
        }
    }
    Source::Vec2 field_xy(const Source::Vec3 &point)const {
        double a=reference_.at("vio_rotation").get<double>(),c=std::cos(a),s=std::sin(a);
        double x=point[0]-reference_.at("pose_origin").at(0).get<double>(),y=point[1]-reference_.at("pose_origin").at(1).get<double>();
        return {{c*x-s*y+reference_.at("start_center_xy_m").at(0).get<double>(),s*x+c*y+reference_.at("start_center_xy_m").at(1).get<double>()}};
    }
    Json current(bool evidence=false)const {
        if(reference_.is_null()||poses_.empty()||yaws_.empty())return Json{{"ready",false},{"reason","尚未从围栏建立场地参考"},
            {"scans",scan_count_},{"fixes",fit_count_},{"initialization",nullptr},{"lidar_corrections_enabled",lidar_enabled_}};
        const auto &selected=(motion_.history.empty()?poses_:motion_.history).back();auto t=std::int64_t(selected[0]);
        Source::Vec3 xyz{{selected[1],selected[2],selected[3]}};
        auto center=field_xy(xyz),raw=field_xy(Source::sample(poses_,double(t)));
        double yaw=Source::sample_yaw(yaws_,double(t))+reference_.at("yaw_offset").get<double>();
        double degrees=std::fmod(yaw*180./PI,360.);if(degrees<0)degrees+=360.;
        std::string source=!motion_.history.empty()?motion_.mode:(has_anchor_?"VIO":"VIO_FROM_INITIAL_REFERENCE");
        if(has_anchor_)source="LIDAR_ANCHORED_"+source;
        auto now=now_ns();return Json{{"source",source},{"motion_selection",motion_.snapshot(evidence)},
            {"position_reacquisitions",reacquisitions_},{"current_wall_axes",last_fit_.is_null()?Json::array():last_fit_.value("position_observed_axes",Json::array())},
            {"lidar_corrections_enabled",lidar_enabled_},{"last_scan_age_ms",last_scan_ns_<0?Json(nullptr):Json((now-last_scan_ns_)/1e6)},
            {"scans",scan_count_},{"fixes",fit_count_},{"ready",true},{"center_xy_m",Json::array({center[0]+offset_[0],center[1]+offset_[1]})},
            {"t265_center_xy_m",raw},{"yaw_deg",degrees},{"sample_monotonic_ns",t},{"motion_age_ms",(now-t)/1e6},
            {"motion_boundary_policy","hold_outside_measured_support"},{"raw_t265_sample_ns",std::int64_t(poses_.back()[0])},
            {"reference",reference_},{"qualified_pose",false},{"accuracy_verified",false},{"control_pose_published",false}};
    }
    Json apply_wall(const Json &p) {
        auto expected=get<std::int64_t>(p,"revision",revision_);
        if(expected!=revision_)return Json{{"ok",false},{"applied",false},{"reason","queued scan belongs to a previous reference"},{"revision",revision_}};
        if(reference_.is_null()||poses_.empty())return Json{{"ok",false},{"applied",false},{"reason","reference unavailable"},{"revision",revision_}};
        auto ns=p.at("reference_ns").get<std::int64_t>();auto candidate=p.value("candidate_center_xy_m",Json(nullptr));
        auto observed=axes(p,"position_observed_axes");bool reacquired=get<bool>(p,"position_reacquired",false);
        motion_.observe_walls(get<std::string>(p,"scan_id",""),observed,ns,candidate,poses_,reacquired);
        auto selected=Source::sample(motion_.history.empty()?poses_:motion_.history,double(ns));auto xy=field_xy(selected);
        if(!candidate.is_null()){
            for(const auto &axis:observed){auto i=axis=="x"?0:axis=="y"?1:-1;if(i<0)throw std::invalid_argument("unknown wall axis");offset_[i]=candidate.at(i).get<double>()-xy[i];}
            ++fit_count_;has_anchor_=true;if(reacquired)++reacquisitions_;
        }
        last_scan_ns_=ns;last_fit_=p;
        return Json{{"ok",true},{"applied",true},{"current_xy",xy},{"offset",offset_},{"motion_selection",motion_.snapshot()},
            {"source",motion_.mode},{"fixes",fit_count_},{"reacquisitions",reacquisitions_},{"revision",revision_}};
    }
    Json format_control(const Json &value)const {
        if(!value.at("ready").get<bool>()){
            if(last_ready_packet_.is_null())return nullptr;
            Json packet=last_ready_packet_;packet["packet_id"]=packets_sent_;packet["quality"]="LOST";
            packet["reference_invalid_reason"]=get<std::string>(value,"reason","reference unavailable");return packet;
        }
        const auto &xy=value.at("center_xy_m");return Json{{"schema_version",1},{"session_id",session_},{"packet_id",packets_sent_},
            {"frame","field"},{"timestamp_monotonic_ns",value.at("sample_monotonic_ns")},{"quality","DEGRADED"},
            {"pose",Json{{"x_m",xy.at(0)},{"y_m",xy.at(1)},{"yaw_deg",value.at("yaw_deg")}}},
            {"pose_source",value.at("source")},{"motion_source",value.at("motion_selection").at("mode")},
            {"t265",Json{{"tracker_confidence",last_raw_pose_.value("tracker_confidence",Json(0))},
                {"mapper_confidence",last_raw_pose_.value("mapper_confidence",Json(0))},
                {"travel_from_start_m",last_raw_pose_.value("travel_from_start_m",Json(0.))},
                {"translation_scale",1.},{"translation_scale_enabled",false},{"raw_center_xy_m",value.at("t265_center_xy_m")},
                {"sample_monotonic_ns",value.at("raw_t265_sample_ns")}}},
            {"wheel",Json{{"fusion_weight",0.},{"official_sdk_input",true},{"independent_reference",false}}},
            {"navigation",Json{{"distance_compensation_enabled",false},{"wheel_progress_m",nullptr}}},
            {"qualified_pose",false},{"accuracy_verified",false},{"current_wall_axes",value.at("current_wall_axes")},
            {"lidar_corrections_enabled",lidar_enabled_},{"lidar_connected",lidar_connected_},{"lidar_error",lidar_error_},
            {"position_reacquisitions",reacquisitions_},{"reference",reference_},{"motion_selection",value.at("motion_selection")}};
    }
    void publish() {
        publish_policy();auto value=current();
        static const Json empty_raw=Json::object();
        const Json &raw=last_sample_.is_null()?empty_raw:last_sample_;
        Json key=Json::array({revision_,value.at("ready"),value.value("sample_monotonic_ns",Json(nullptr)),
            value.value("center_xy_m",Json::array()),value.value("yaw_deg",Json(nullptr)),value.value("source",Json(nullptr)),
            value.value("current_wall_axes",Json::array()),value.value("fixes",Json(nullptr)),
            raw.value("tracker_confidence",Json(nullptr)),raw.value("mapper_confidence",Json(nullptr)),
            !poses_.empty(),!yaws_.empty(),lidar_connected_,lidar_error_});
        if(key==last_key_)return;
        auto packet=format_control(value);
        if(value.at("ready").get<bool>())last_ready_packet_=packet;
        if(control_.fd>=0&&!packet.is_null()){
            auto encoded=packet.dump();
            if(control_.send(encoded)){++packets_sent_;last_packet_=packet;control_log_->submit(encoded+"\n");}
            else ++output_drops_;
        }
        ++control_updates_;last_key_=std::move(key);
    }
    void publish_policy() {
        bool pause=motion_.wheel_input_paused();
        const auto reason=motion_.wheel_input_reason();
        if(policy_.fd<0||(pause==last_policy_pause_&&reason==last_policy_reason_))return;
        Json packet={{"schema",3},{"kind","wheel_source_policy"},{"session_id",policy_session_},{"pause",pause},
            {"reason",reason},
            {"wall_motion_evidence",motion_.wall_evidence},{"dependency_persists_after_pause",true}};
        if(policy_.send(packet.dump())){
            last_policy_pause_=pause;last_policy_reason_=reason;policy_error_reported_=false;packet["sdk_application_confirmed"]=false;event("wheel_source_policy_requested",packet);
        }else if(!policy_error_reported_){
            event("wheel_source_policy_transport_error",Json{{"reason",std::strerror(errno)},{"pause_requested",pause}});policy_error_reported_=true;
        }
    }
    Json state()const {
        auto feedback=Json{{"connected",feedback_connection_>=0},{"error",native_error_},{"decode_errors",decode_errors_},
            {"frames",frames_},{"wheel_frames",wheel_frames_},{"gyro_frames",gyro_frames_},{"timed_yaw_seen",timed_yaw_seen_},
            {"conversion","original_forward_motion_event"},{"identity_present",!clock_.identity.is_null()}};
        Json logs={{"motion_feedback",Json{{"dropped",feedback_log_->dropped()},{"failed",feedback_log_->failed()}}},
            {"pose_motion",Json{{"dropped",pose_log_->dropped()},{"failed",pose_log_->failed()}}},
            {"control_pose",Json{{"dropped",control_log_->dropped()},{"failed",control_log_->failed()}}},
            {"events",Json{{"dropped",events_log_->dropped()},{"failed",events_log_->failed()}}}};
        auto value=current(true);
        return Json{{"last",value},{"first_pose",poses_.empty()?Json(nullptr):Json(poses_.front())},
            {"last_pose",poses_.empty()?Json(nullptr):Json(poses_.back())},{"first_yaw",yaws_.empty()?Json(nullptr):Json(yaws_.front())},
            {"last_yaw",yaws_.empty()?Json(nullptr):Json(yaws_.back())},
            {"last_yaw_row",yaws_.empty()?Json(nullptr):Json(yaws_.back())},{"last_sample",last_sample_},{"raw_pose",last_raw_pose_},
            {"last_yaw_source",last_yaw_},{"have_tracked_pose",!poses_.empty()},{"have_primary_yaw",!yaws_.empty()},
            {"pose_ingested",poses_ingested_},{"measured_wheel_ingested",wheels_ingested_},{"primary_yaw_ingested",yaws_ingested_},
            {"control_updates",control_updates_},{"native_feedback",feedback},{"motion_selection",motion_.snapshot()},
            {"reference",reference_},{"offset",offset_},{"revision",revision_},{"has_anchor",has_anchor_},
            {"competition_wheel_policy",competition_wheel_policy_},
            {"fixes",fit_count_},{"reacquisitions",reacquisitions_},{"scans",scan_count_},{"last_scan_ns",last_scan_ns_<0?Json(nullptr):Json(last_scan_ns_)},
            {"direct_output",Json{{"packets_sent",packets_sent_},{"output_drops",output_drops_},{"last_packet",last_packet_}}},
            {"logging",logs},{"pose_input_error",pose_error_},{"motion_scheduling",scheduling_},{"error",thread_error_},
            {"last_wheel",motion_.last_wheel?Json::array({motion_.last_wheel->first,motion_.last_wheel->second}):Json(nullptr)}};
    }
    bool receive_one(bool feedback) {
        int &connection=feedback?feedback_connection_:pose_connection_;const int listener=feedback?feedback_fd_:pose_fd_;
        if(listener<0)return false;
        if(connection<0){
            pollfd probe{listener,POLLIN,0};if(poll(&probe,1,0)<=0||!(probe.revents&POLLIN))return false;
            connection=accept4(listener,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
            if(connection<0){if(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)return false;throw std::runtime_error(std::strerror(errno));}
            int capacity=262144;setsockopt(connection,SOL_SOCKET,SO_RCVBUF,&capacity,sizeof capacity);
        }
        unsigned char data[16384];ssize_t count;
        do{count=recv(connection,data,feedback?179:sizeof data,MSG_DONTWAIT);}while(count<0&&errno==EINTR);
        if(count==0){::close(connection);connection=-1;return false;}
        if(count<0){
            if(errno==EAGAIN||errno==EWOULDBLOCK)return false;
            auto reason=std::string(std::strerror(errno));::close(connection);connection=-1;throw std::runtime_error(reason);
        }
        if(feedback){
            auto sample=decode_feedback(data,std::size_t(count));++frames_;auto type=sample.at("type").get<int>();
            if(type==7||type==8){
                if(type==7)++wheel_frames_;else ++gyro_frames_;
                feedback_log_->submit(sample.dump()+"\n");
                context(Json{{"kind",type==7?"timed_wheel_diagnostic":"timed_gyro_diagnostic"},{"sample",sample}});
            }
        }else{
            auto sample=Json::parse(reinterpret_cast<const char*>(data),reinterpret_cast<const char*>(data)+count);
            if(pose_session_.empty()||get<std::string>(sample,"session_id","")==pose_session_)ingest_pose(sample);
        }
        publish();return true;
    }
    void run() {
        const char *rt=std::getenv("RDK_LOCALIZATION_RT");
        const bool requested=!rt||(std::strcmp(rt,"0")&&std::strcmp(rt,"off")&&std::strcmp(rt,"false"));
        sched_param priority{};priority.sched_priority=10;
        const int error=requested?pthread_setschedparam(pthread_self(),SCHED_FIFO,&priority):0;
        int policy=0;const int query_error=pthread_getschedparam(pthread_self(),&policy,&priority);
        {std::lock_guard<RuntimeMutex> guard(mutex_);scheduling_=Json{{"requested",requested},{"requested_policy","SCHED_FIFO"},{"requested_priority",10},
            {"actual_policy",query_error?Json(nullptr):Json(policy)},
            {"actual_priority",query_error?Json(nullptr):Json(priority.sched_priority)},
            {"applied",requested&&!error&&!query_error&&policy==SCHED_FIFO&&priority.sched_priority==10},
            {"error",error?Json(std::strerror(error)):Json(nullptr)},
            {"query_error",query_error?Json(std::strerror(query_error)):Json(nullptr)},
            {"mutex_protocol_requested","PTHREAD_PRIO_INHERIT"},
            {"mutex_protocol_actual",mutex_.priority_inheritance()?"PTHREAD_PRIO_INHERIT":"PTHREAD_PRIO_NONE"},
            {"mutex_priority_inheritance",mutex_.priority_inheritance()},
            {"mutex_protocol_error",mutex_.protocol_error()?Json(std::strerror(mutex_.protocol_error())):Json(nullptr)}};}
        while(!stopping_.load()){
            bool changed=false;
            // Phase changes wake this same owner directly; GUI/disk/geometry
            // are not on the request path. Drain only a bounded transition set.
            for(int i=0;i<16;++i){
                try{std::lock_guard<RuntimeMutex> guard(mutex_);if(!receive_wheel_policy())break;changed=true;}
                catch(const std::exception &e){std::lock_guard<RuntimeMutex> guard(mutex_);
                    event("competition_wheel_policy_error",Json{{"reason",e.what()}});break;}
            }
            for(int i=0;i<16&&!stopping_.load();++i){
                bool active=false;
                for(bool feedback:{true,false}){
                    if(stopping_.load())break;
                    try{std::lock_guard<RuntimeMutex> guard(mutex_);if(receive_one(feedback)){active=true;changed=true;}}
                    catch(const std::exception &e){
                        std::lock_guard<RuntimeMutex> guard(mutex_);
                        if(feedback){++decode_errors_;native_error_=e.what();event("native_motion_feedback_error",Json{{"reason",e.what()}});}
                        else{pose_error_=e.what();event("pose_input_error",Json{{"reason",e.what()}});}
                    }
                }
                if(!active)break;
            }
            if(changed)continue;
            pollfd waits[4];int count=0;
            {std::lock_guard<RuntimeMutex> guard(mutex_);
                waits[count++]={wake_,POLLIN,0};
                if(feedback_fd_>=0)waits[count++]={feedback_connection_>=0?feedback_connection_:feedback_fd_,POLLIN,0};
                if(pose_fd_>=0)waits[count++]={pose_connection_>=0?pose_connection_:pose_fd_,POLLIN,0};
                if(wheel_policy_fd_>=0)waits[count++]={wheel_policy_fd_,POLLIN,0};
            }
            int ready;do{ready=poll(waits,count,-1);}while(ready<0&&errno==EINTR&&!stopping_.load());
            if(ready<0){std::lock_guard<RuntimeMutex> guard(mutex_);thread_error_=std::strerror(errno);break;}
        }
    }
};

char *result_string(const Json &value) {
    auto encoded=value.dump();char *result=static_cast<char*>(std::malloc(encoded.size()+1));
    if(!result)throw std::bad_alloc();std::memcpy(result,encoded.c_str(),encoded.size()+1);return result;
}
Json parse(const char *text) {return text&&*text?Json::parse(text):Json::object();}
char *error_result(const std::exception &error) {
    last_error=error.what();try{return result_string(Json{{"ok",false},{"error",last_error}});}catch(...){return nullptr;}
}
}

extern "C" void *omni_motion_runtime_create(const char *config_json) {
    try{last_error.clear();return new Runtime(parse(config_json));}catch(const std::exception &e){last_error=e.what();return nullptr;}catch(...){last_error="unknown native motion create error";return nullptr;}
}
extern "C" char *omni_motion_runtime_command(void *handle,const char *command_json) {
    try{if(!handle)throw std::invalid_argument("null native motion handle");return result_string(static_cast<Runtime*>(handle)->command(parse(command_json)));}
    catch(const std::exception &e){return error_result(e);}catch(...){return nullptr;}
}
extern "C" char *omni_motion_runtime_snapshot(void *handle,const char *query_json) {
    try{if(!handle)throw std::invalid_argument("null native motion handle");return result_string(static_cast<Runtime*>(handle)->snapshot(parse(query_json)));}
    catch(const std::exception &e){return error_result(e);}catch(...){return nullptr;}
}
extern "C" void omni_motion_runtime_close(void *handle) {delete static_cast<Runtime*>(handle);}
extern "C" char *omni_motion_runtime_finish(void *handle) {
    if(!handle)return nullptr;
    // Ownership is consumed even if the final snapshot cannot be allocated.
    std::unique_ptr<Runtime> owner(static_cast<Runtime*>(handle));
    try{owner->close();return result_string(owner->snapshot(Json{{"kind","state"}}));}
    catch(const std::exception &e){return error_result(e);}catch(...){return nullptr;}
}
extern "C" void omni_motion_runtime_free_string(char *value) {std::free(value);}
extern "C" const char *omni_motion_runtime_last_error() {return last_error.c_str();}
