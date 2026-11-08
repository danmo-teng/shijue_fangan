#include "f405_wheel_measurements.hpp"
#include "f407_protocol.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace omni {
namespace {
using Json=F405WheelMeasurements::Json;
std::uint64_t be(const std::uint8_t *p,int n) {
    std::uint64_t v=0;for(int i=0;i<n;++i)v=(v<<8)|p[i];return v;
}
std::int64_t signed_be(const std::uint8_t *p,int n) {
    const auto bits=be(p,n);
    if(n==4) {const std::uint32_t u=bits;std::int32_t v;std::memcpy(&v,&u,4);return v;}
    std::int64_t v;std::memcpy(&v,&bits,8);return v;
}
std::string hex(const std::uint8_t *p,int n) {
    static const char digits[]="0123456789abcdef";
    std::string s;for(int i=0;i<n;++i){s+=digits[p[i]>>4];s+=digits[p[i]&15];}return s;
}
bool envelope(const std::uint8_t *p,std::size_t n) {
    return n>=11 && p[0]==0xd4 && p[1]==0x4d && p[2]==1 &&
        be(p+4,2)<=160 && be(p+4,2)+11==n && p[n-1]==0xe4 &&
        modbus_crc16(p+2,n-5)==std::uint16_t(p[n-3]|(p[n-2]<<8));
}
void merge(Json &target,const Json &extra) {for(auto it=extra.begin();it!=extra.end();++it)target[it.key()]=it.value();}
bool after_id(std::uint32_t a,std::uint32_t b) {const std::uint32_t d=a-b;return d>0&&d<0x80000000u;}
}

F405WheelMeasurements::F405WheelMeasurements(const LocalizationConfig &c,std::string s,Record r)
    :config_(c),session_(std::move(s)),record_(std::move(r)) {}

void F405WheelMeasurements::request(const std::uint8_t *p,std::size_t n,std::uint64_t host_ns) {
    if(n==15 && p[0]==0xa3 && p[1]==0xb3 && p[2]==0x19 && p[14]==0xc3 &&
        modbus_crc16(p+2,10)==std::uint16_t(p[12]|(p[13]<<8))) {
        motion_request_={{"wire_sequence",p[3]},{"opcode",p[4]},{"uart_write_before_ns",host_ns},
            {"acceptance","request_only; correlate chassis_actions/0x1A; not DONE"}};
        record_({{"event","wheel_native_motion_request"},{"request",motion_request_}});return;
    }
    // A formal startup file contains two concatenated requests. No new query,
    // timeout, periodic handshake or freshness qualification is introduced.
    for(std::size_t offset=0;offset+11<=n;) {
        const auto size=be(p+offset+4,2)+11;
        if(size>n-offset || !envelope(p+offset,size))return;
        const auto *q=p+offset;
        if((q[3]==1||q[3]==3||q[3]==5||q[3]==9)&&size>=19) {
            if(requests_.size()>=32)requests_.erase(requests_.begin());
            requests_[be(q+8,8)]={{"type",q[3]},{"sequence",be(q+6,2)},{"host_write_before_ns",host_ns}};
        }
        offset+=size;
    }
}

F405WheelMeasurements::Json F405WheelMeasurements::decode(const std::uint8_t *p,std::size_t n,std::uint64_t rx) {
    if(!envelope(p,n))throw std::runtime_error("invalid_extended_envelope");
    const auto *d=p+8;const auto len=n-11;const int kind=p[3];
    Json s{{"type",kind},{"version",1},{"sequence",be(p+6,2)},{"receive_monotonic_ns",rx}};
    if(kind==2 && len==137) {
        merge(s,{{"nonce",be(d,8)},{"epoch_fault",be(d+8,2)},{"boot_epoch",hex(d+10,16)},
            {"reference_epoch",be(d+26,4)},{"capabilities",be(d+30,8)},{"board",be(d+38,2)},
            {"mode",d[40]},{"service",d[41]},{"protocol_major",d[42]},{"protocol_minor",d[43]},
            {"tick_hz",be(d+44,4)},{"uart_baud",be(d+48,4)},{"clock_flags",be(d+52,4)},
            {"drift_bound_ppm",be(d+56,4)},{"firmware_id",hex(d+82,32)},
            {"git_commit",hex(d+114,20)},{"source_dirty",d[134]},{"max_payload",be(d+135,2)}});
    } else if(kind==4 && len==46) {
        merge(s,{{"nonce",be(d,8)},{"boot_epoch",hex(d+8,16)},{"receive_event_us",be(d+24,8)},
            {"response_prepare_us",be(d+32,8)},{"tick_resolution_us",be(d+40,4)},{"epoch_valid",be(d+44,2)}});
    } else if(kind==7 && len==84) {
        merge(s,{{"boot_epoch",hex(d,16)},{"reference_epoch",be(d+16,4)},{"measurement_id",be(d+20,4)},
            {"t0_us",be(d+24,8)},{"t1_us",be(d+32,8)},{"forward_um_s",signed_be(d+40,4)},
            {"left_um_s",signed_be(d+44,4)},{"raw1_u32",be(d+48,4)},{"raw2_u32",be(d+52,4)},
            {"raw3_u32",be(d+56,4)},{"delta1",signed_be(d+60,4)},{"delta2",signed_be(d+64,4)},
            {"delta3",signed_be(d+68,4)},{"flags",be(d+72,4)},
            {"source_uncertainty_us",be(d+76,4)},{"capture_window_us",be(d+80,4)}});
    } else if(kind==8 && len==84) {
        merge(s,{{"boot_epoch",hex(d,16)},{"reference_epoch",be(d+16,4)},{"publication_id",be(d+20,4)},
            {"yaw_mdeg",signed_be(d+24,8)},{"yaw_receive_us",be(d+32,8)},{"yaw_source_id",be(d+40,4)},
            {"yaw_flags",be(d+44,4)},{"yaw_uncertainty_us",be(d+48,4)},{"rate_mdps",signed_be(d+52,4)},
            {"rate_receive_us",be(d+56,8)},{"rate_source_id",be(d+64,4)},{"rate_flags",be(d+68,4)},
            {"rate_uncertainty_us",be(d+72,4)},{"publish_us",be(d+76,8)}});
    } else throw std::runtime_error("unsupported_extended_type_or_length");
    return s;
}

void F405WheelMeasurements::feedback(const std::uint8_t *p,std::size_t n,std::uint64_t rx) {
    if(n<4 || (p[3]!=2&&p[3]!=4&&p[3]!=7&&p[3]!=8))return;
    Json s;
    try{s=decode(p,n,rx);}catch(const std::exception &e) {
        record_({{"event","wheel_native_rejected"},{"reason",e.what()},{"receive_monotonic_ns",rx}});return;
    }
    const int kind=s["type"];
    if(kind==2||kind==4) {
        const auto q=requests_.find(s["nonce"].get<std::uint64_t>());
        if(q==requests_.end() || q->second["type"].get<int>()+1!=kind || q->second["sequence"]!=s["sequence"])return;
        s["host_write_before_ns"]=q->second["host_write_before_ns"];requests_.erase(q);
        if(kind==2) {
            const int major=s["protocol_major"],minor=s["protocol_minor"];
            const unsigned flags=s["clock_flags"];
            s["epoch_valid"]=s["epoch_fault"].get<unsigned>()==0 && (flags&2) && (minor!=1 || (flags&1)) && s["boot_epoch"]!=std::string(32,'0');
            s["interface_compatible"]=s["board"]==405 && s["mode"]==5 && (s["service"]==1||s["service"]==2) &&
                major==1 && (minor==0||minor==1||minor==3) && s["max_payload"]==160;
            if(!identity_.is_null() && identity_["boot_epoch"]!=s["boot_epoch"]) {
                rates_.clear();pending_.clear();have_wheel_=have_rate_=have_end_=false;sync_=nullptr;
            }
            identity_=s;
        } else if(!identity_.is_null() && s["boot_epoch"]==identity_["boot_epoch"] && s["epoch_valid"]==1)sync_=s;
        record_({{"event",kind==2?"wheel_native_identity":"wheel_native_clock_exchange"},{"sample",s}});return;
    }
    const auto epoch=s["boot_epoch"].get<std::string>();
    const bool source_identity=!identity_.is_null() && identity_.value("epoch_valid",false) &&
        identity_.value("interface_compatible",false) && identity_["boot_epoch"]==epoch;
    s["mcu_epoch"]=source_identity?Json(epoch):Json();
    // Match the current, explicitly unqualified timing semantics. This is not
    // an adoption gate; send_wheel_odometry has no sample timestamp argument.
    s["capture_time_qualified"]=false;s["time_uncertainty_ns"]=nullptr;
    s["qualification_reasons"]={"source_and_clock_bounds_not_calibrated"};
    s["alignment_clock"]="mcu_event";
    const bool v13=!identity_.is_null() && identity_["protocol_major"]==1 && identity_["protocol_minor"]==3;
    s["motion_wire_request_at_receive"]=motion_request_;
    if(kind==8) {
        const auto id=s["rate_source_id"].get<std::uint32_t>();
        if(have_rate_&&rate_epoch_==epoch) {
            if(id==rate_id_)return;
            if(!after_id(id,rate_id_)){rates_.clear();record_({{"event","wheel_native_rejected"},{"reason","out_of_order_rate"},{"sample",s}});return;}
        }
        rate_epoch_=epoch;rate_id_=id;have_rate_=true;
        const unsigned flags=s["rate_flags"];
        s["wire_valid"]=v13 ? bool((flags&1)&&!(flags&32)) : ((flags&3)==3 && !(flags&(4|8|32|128|1024)));
        s["alignment_capture_ns"]=s["rate_receive_us"].get<std::uint64_t>()*1000;
        s["yaw_rate_deg_s"]=s["rate_mdps"].get<double>()/1000.;
        record_({{"event","wheel_native_measurement"},{"sample",s}});
        if(source_identity&&s["wire_valid"]==true) {
            if(rates_.size()>=128)rates_.pop_front();rates_.push_back(std::move(s));
        } else rates_.clear();
        return;
    }
    const auto id=s["measurement_id"].get<std::uint32_t>();
    if(have_wheel_&&wheel_epoch_==epoch) {
        if(id==wheel_id_)return;
        if(!after_id(id,wheel_id_)){record_({{"event","wheel_native_rejected"},{"reason","out_of_order_wheel"},{"sample",s}});return;}
    }
    wheel_epoch_=epoch;wheel_id_=id;have_wheel_=true;
    const auto t0=s["t0_us"].get<std::uint64_t>(),t1=s["t1_us"].get<std::uint64_t>();
    const double dt=t1>t0?(t1-t0)/1e6:0.;
    const unsigned flags=s["flags"];
    bool valid=(flags&1)&&!(flags&(v13?(8|16|32):(4|8|32|256|512|1024)))&&dt>0;
    const double forward=s["forward_um_s"].get<double>()/1e6,left=s["left_um_s"].get<double>()/1e6;
    s["v_body_mps"]={forward,left,0.};
    if(dt>0) {
        const double k=std::acos(-1.)*.070/1061.268;
        const double a=s["delta1"].get<double>(),b=s["delta2"].get<double>(),c=s["delta3"].get<double>();
        const double hf=k*(a-b)/(std::sqrt(3.)*dt),hl=k*(a+b-2*c)/(3*dt);
        s["host_recomputed_v_body_mps"]={hf,hl,0.};
        if(std::max(std::abs(hf-forward),std::abs(hl-left))>.00001)valid=false;
    }
    s["measurement_valid"]=valid;s["source"]="measured_wheel_feedback";
    s["sample_id"]=id;s["alignment_start_ns"]=t0*1000;s["alignment_end_ns"]=t1*1000;
    s["interval_start_ns"]=nullptr;s["interval_end_ns"]=nullptr;
    if(!sync_.is_null()) {
        const auto lo=static_cast<std::int64_t>(sync_["response_prepare_us"].get<std::uint64_t>()*1000)-sync_["receive_monotonic_ns"].get<std::int64_t>()-sync_["tick_resolution_us"].get<std::int64_t>()*1000;
        const auto hi=static_cast<std::int64_t>(sync_["receive_event_us"].get<std::uint64_t>()*1000)-sync_["host_write_before_ns"].get<std::int64_t>()+sync_["tick_resolution_us"].get<std::int64_t>()*1000;
        if(lo<=hi) {s["interval_start_ns"]=static_cast<std::int64_t>(t0*1000)-(lo+hi)/2;
                    s["interval_end_ns"]=static_cast<std::int64_t>(t1*1000)-(lo+hi)/2;}
    }
    record_({{"event","wheel_native_measurement"},{"sample",s}});
    if(pending_.size()>=64) {
        record_({{"event","wheel_native_rejected"},{"reason","wheel_queue_overflow"},{"sample",pending_.front()}});pending_.pop_front();
    }
    pending_.push_back(std::move(s));
}

F405WheelMeasurements::Json F405WheelMeasurements::prepare(const Json &s,std::string &reason) {
    if(s["mcu_epoch"].is_null()){reason="unknown_measurement_epoch";return {};}
    if(s["measurement_valid"]!=true){reason="not_valid_measured_wheel_feedback";return {};}
    const auto start=s["alignment_start_ns"].get<std::uint64_t>(),end=s["alignment_end_ns"].get<std::uint64_t>();
    if(start>=end){reason="invalid_wheel_interval";return {};}
    if(have_end_&&start<last_end_){reason="duplicate_or_out_of_order_wheel_interval";return {};}
    std::vector<const Json *> rates;
    for(const auto &g:rates_)if(g["mcu_epoch"]==s["mcu_epoch"])rates.push_back(&g);
    for(std::size_t i=1;i<rates.size();++i)if((*rates[i])["alignment_capture_ns"]<=(*rates[i-1])["alignment_capture_ns"]) {
        reason="nonmonotonic_gyro_capture_time";return {};
    }
    if(rates.empty()||rates.back()->at("alignment_capture_ns").get<std::uint64_t>()<end) {
        reason="waiting_for_bracketing_gyro";return {};
    }
    std::size_t first=0,last=0;
    if(rates.front()->at("alignment_capture_ns").get<std::uint64_t>()>start){reason="no_gyro_at_interval_start";return {};}
    while(first+1<rates.size()&&rates[first+1]->at("alignment_capture_ns").get<std::uint64_t>()<=start)++first;
    last=first;while(rates[last]->at("alignment_capture_ns").get<std::uint64_t>()<end)++last;
    double integral=0.;std::uint64_t receive=0,max_gap=0;
    Json selected=Json::array();
    for(std::size_t i=first;i<=last;++i) {
        const auto &a=*rates[i];receive=std::max(receive,a["receive_monotonic_ns"].get<std::uint64_t>());
        selected.push_back({{"mcu_epoch",a["mcu_epoch"]},{"alignment_capture_ns",a["alignment_capture_ns"]},
            {"yaw_rate_deg_s",a["yaw_rate_deg_s"]},{"receive_monotonic_ns",a["receive_monotonic_ns"]},{"rate_source_id",a["rate_source_id"]}});
        if(i==last)break;
        const auto &b=*rates[i+1];const auto ta=a["alignment_capture_ns"].get<std::uint64_t>(),tb=b["alignment_capture_ns"].get<std::uint64_t>();
        const auto lo=std::max(start,ta),hi=std::min(end,tb);
        const double va=a["yaw_rate_deg_s"],vb=b["yaw_rate_deg_s"];
        const double vlo=va+(vb-va)*double(lo-ta)/(tb-ta),vhi=va+(vb-va)*double(hi-ta)/(tb-ta);
        integral+=(vlo+vhi)*.5*(hi-lo);max_gap=std::max(max_gap,tb-ta);
    }
    const double omega=integral/double(end-start)*std::acos(-1.)/180.;
    const double vx=s["v_body_mps"][0],vy=s["v_body_mps"][1];
    if(std::hypot(vx,vy)>1.5||std::abs(omega)>8.){reason="implausible_planar_measured_velocity";return {};}
    const double lever_x=-omega*config_.camera_offset_left_m,lever_y=omega*config_.camera_offset_forward_m;
    const auto &f=config_.camera_robot_forward_axis;const auto &u=config_.camera_robot_up_axis;
    const std::array<double,3> left{u[1]*f[2]-u[2]*f[1],u[2]*f[0]-u[0]*f[2],u[0]*f[1]-u[1]*f[0]};
    std::array<double,3> velocity;
    for(int i=0;i<3;++i)velocity[i]=f[i]*(vx+lever_x)+left[i]*(vy+lever_y);
    last_end_=end;have_end_=true;
    Json packet{{"schema",2},{"session_id",session_},{"packet_id",next_packet_++},{"time_policy","received_feedback_no_age_gate_v1"},
        {"source","measured_wheel_feedback"},{"measurement_valid",true},{"mcu_epoch",s["mcu_epoch"]},
        {"sample_id",s["sample_id"]},{"capture_time_qualified",false},{"time_uncertainty_ns",nullptr},
        {"alignment_clock","mcu_event"},{"alignment_start_ns",start},{"alignment_end_ns",end},
        {"interval_start_ns",s["interval_start_ns"]},{"interval_end_ns",s["interval_end_ns"]},
        {"receive_monotonic_ns",s["receive_monotonic_ns"]},{"gyro_receive_ns",receive},
        {"alignment_gyro_start_ns",rates[first]->at("alignment_capture_ns")},{"alignment_gyro_end_ns",rates[last]->at("alignment_capture_ns")},
        {"gyro_max_gap_ns",max_gap},{"gyro_samples",selected},{"v_body_mps",s["v_body_mps"]},
        {"omega_body_rad_s",{0.,0.,omega}},{"lever_body_mps",{lever_x,lever_y,0.}},{"v_camera_mps",velocity},
        {"dependency_sources",{"measured_wheel_feedback","primary_gyro_rate"}},{"processing_owner","native_cpp_sdk_owner"}};
    packet["motion_wire_request_at_receive"]=s["motion_wire_request_at_receive"];
    // Keep the start bracket needed by the next non-overlapping interval.
    // Normal 100Hz operation stays O(new gyro samples), not a 128-row scan.
    while(rates_.size()>2 && rates_[1]["alignment_capture_ns"].get<std::uint64_t>()<=start)rates_.pop_front();
    return packet;
}

F405WheelMeasurements::Json F405WheelMeasurements::latest() {
    Json latest;unsigned coalesced=0;
    while(!pending_.empty()) {
        std::string reason;Json p=prepare(pending_.front(),reason);
        if(reason=="waiting_for_bracketing_gyro")break;
        if(!reason.empty())record_({{"event","wheel_native_rejected"},{"reason",reason},{"sample",pending_.front()}});
        pending_.pop_front();
        if(!p.is_null()) {if(!latest.is_null())++coalesced;latest=std::move(p);}
    }
    if(coalesced)record_({{"event","wheel_requests_coalesced"},{"samples",coalesced},{"processing_owner","native_cpp_sdk_owner"}});
    return latest;
}
} // namespace omni
