#include "raw_imu_csv.hpp"
#include <cassert>
#include <fstream>
#include <limits>
#include <sstream>
#include <cstdio>
int main(int argc, char **argv) {
    assert(argc == 2);
    omni::RawImuCsv log(argv[1]);
    log.submit(1, true, 17, 12345.125, 2, .1f, -.2f, .3f, true, 41.);
    log.submit(2, false, 1, 12400., 2, 0.f, -9.8f, std::numeric_limits<float>::quiet_NaN(), false, 0.);
    log.close();
    assert(log.samples()==2 && log.dropped()==0 && !log.failed());
    std::ifstream in(argv[1]);std::string header,a,b,extra;
    std::getline(in,header);std::getline(in,a);std::getline(in,b);
    assert(header=="sdk_run,stream,frame_number,capture_timestamp_ms,timestamp_domain,x,y,z,temperature_c");
    assert(a.find("1,gyro,17,12345.125000,2,")==0 && a.substr(a.size()-3)==",41");
    assert(b.find("2,accel,1,12400.000000,2,")==0 && b.find("nan,")!=std::string::npos);
    assert(!std::getline(in,extra));
    omni::RawImuCsv bad(std::string(argv[1])+"/missing/file");
    bad.submit(1,true,1,0,2,0,0,0,false,0);bad.close();
    assert(bad.failed() && bad.dropped()>0); // Log failure does not throw/stop control.
    std::remove(argv[1]);
}
