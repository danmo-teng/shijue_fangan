#include "test_uart_relay.hpp"
#include "f407_protocol.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace omni {
namespace {
std::uint64_t ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
void put(std::uint8_t *p,std::uint64_t n) {for(int i=0;i<8;++i)p[i]=n>>((7-i)*8);}
sockaddr_un target(const std::string &name) {
    if(name.empty()||name[0]!='@'||name.size()>=sizeof(sockaddr_un{}.sun_path))throw std::invalid_argument("short abstract test transport address required");
    sockaddr_un a{};a.sun_family=AF_UNIX;std::memcpy(a.sun_path+1,name.data()+1,name.size()-1);return a;
}
}
TestUartRelay::TestUartRelay(std::string address,Write write):address_(std::move(address)),write_(std::move(write)) {
    auto a=target(address_),rx=target(address_+"-rx");
    server_=::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    rx_server_=::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(server_<0||rx_server_<0 || ::bind(server_,reinterpret_cast<sockaddr *>(&a),offsetof(sockaddr_un,sun_path)+address_.size()) || ::listen(server_,1) ||
        ::bind(rx_server_,reinterpret_cast<sockaddr *>(&rx),offsetof(sockaddr_un,sun_path)+address_.size()+3) || ::listen(rx_server_,1)) {
        if(server_>=0)::close(server_);if(rx_server_>=0)::close(rx_server_);
        throw std::runtime_error("cannot bind native test UART transport");
    }
    thread_=std::thread([this]{run();});
}
TestUartRelay::~TestUartRelay() {
    stop_=true;if(thread_.joinable())thread_.join();
    if(server_>=0)::close(server_);if(rx_server_>=0)::close(rx_server_);
    if(rx_peer_>=0)::close(rx_peer_);
}
bool TestUartRelay::allowed(const std::uint8_t *p,std::size_t n) {
    if(n==15 && p[0]==0xa3 && p[1]==0xb3 && p[2]==0x19 && p[14]==0xc3)
        return modbus_crc16(p+2,10)==std::uint16_t(p[12]|(p[13]<<8));
    if(n<19||n>23||p[0]!=0xd4||p[1]!=0x4d||p[2]!=1||p[n-1]!=0xe4)return false;
    const auto length=(p[4]<<8)|p[5];
    return length+11==n && (p[3]==1||p[3]==3||p[3]==5||p[3]==9) &&
        length==(p[3]==5?12:8) && modbus_crc16(p+2,n-5)==std::uint16_t(p[n-3]|(p[n-2]<<8));
}
void TestUartRelay::publish(const std::uint8_t *p,std::size_t n,std::uint64_t rx) {
    if(n>4096)return;
    std::lock_guard<std::mutex> lock(rx_mutex_);
    if(rx_peer_<0)return;
    std::uint8_t message[4112];put(message,rx);put(message+8,drops_);std::memcpy(message+16,p,n);
    if(::send(rx_peer_,message,n+16,MSG_DONTWAIT|MSG_NOSIGNAL)!=static_cast<ssize_t>(n+16))++drops_;
}
void TestUartRelay::run() {
    int peer=-1;
    while(!stop_) {
        pollfd fds[]={{peer<0?server_:peer,POLLIN,0},{rx_server_,POLLIN,0}};
        const int ready=::poll(fds,2,50);
        if(ready<0&&errno==EINTR)continue;
        if(ready<=0)continue;
        if(fds[1].revents&POLLIN) {
            const int receiver=::accept4(rx_server_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
            if(receiver>=0) {
                const int budget=262144;::setsockopt(receiver,SOL_SOCKET,SO_SNDBUF,&budget,sizeof(budget));
                std::lock_guard<std::mutex> lock(rx_mutex_);
                if(rx_peer_>=0)::close(rx_peer_);rx_peer_=receiver;
            }
        }
        const auto &fd=fds[0];
        if(fd.revents&(POLLERR|POLLHUP|POLLNVAL)) {
            if(peer>=0)::close(peer);peer=-1;connected_=false;continue;
        }
        if(!(fd.revents&POLLIN))continue;
        if(peer<0) {peer=::accept4(server_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);connected_=peer>=0;continue;}
        std::uint8_t bytes[256];const auto size=::recv(peer,bytes,sizeof(bytes),MSG_TRUNC|MSG_DONTWAIT);
        if(size<=0){::close(peer);peer=-1;connected_=false;continue;}
        const auto before=ns();
        const bool written=size<=256 && allowed(bytes,size) && write_(bytes,size);
        const auto after=ns();
        // This local write result replaces serial.write's result. It is NOT
        // an MCU handshake or evidence that an action has been accepted/done.
        std::uint8_t result[17];result[0]=written;put(result+1,before);put(result+9,after);
        if(::send(peer,result,sizeof(result),MSG_DONTWAIT|MSG_NOSIGNAL)!=sizeof(result)) {
            ::close(peer);peer=-1;connected_=false;
        }
    }
    if(peer>=0)::close(peer);connected_=false;
}
} // namespace omni
