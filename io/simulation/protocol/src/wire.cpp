#include "rm_sim/adapters/wire.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <climits>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <cmath>

namespace rm_sim::adapters {
namespace {
void require(bool ok,const std::string& message) { if(!ok) throw std::invalid_argument(message); }
std::runtime_error error(const char* operation) {
  return std::runtime_error(std::string("External adapter ")+operation+": "+std::strerror(errno));
}
void wait_socket(int fd,short events,Deadline deadline) {
  for(;;) {
    auto now=std::chrono::steady_clock::now();
    if(now>=deadline) throw std::runtime_error("External adapter timeout");
    auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-now).count()+1;
    pollfd item{fd,events,0};
    int result=::poll(&item,1,int(std::min<int64_t>(remaining,INT_MAX)));
    if(result<0 && errno==EINTR) continue;
    if(result<0) throw error("poll");
    if(result==0) throw std::runtime_error("External adapter timeout");
    if(item.revents&POLLNVAL) throw std::runtime_error("External adapter invalid socket");
    return; // recv/send/getsockopt reports hangup and errors precisely.
  }
}
void low_latency(int fd) {
  int enabled=1;
  if(::setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&enabled,sizeof(enabled))<0)throw error("TCP_NODELAY");
}
sockaddr_in address(const std::string& host,int port) {
  validate_endpoint(host,port);
  sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_port=htons(uint16_t(port));
  ::inet_pton(AF_INET,host.c_str(),&addr.sin_addr);return addr;
}
}
Deadline deadline_after(int milliseconds) {
  require(milliseconds>=1 && milliseconds<=60000,"Adapter timeout must be in [1,60000] ms");
  return std::chrono::steady_clock::now()+std::chrono::milliseconds(milliseconds);
}
void validate_endpoint(const std::string& host,int port) {
  in_addr parsed{};
  require(::inet_pton(AF_INET,host.c_str(),&parsed)==1 && port>=1 && port<=65535,
    "Adapter endpoint needs a numeric IPv4 address and port in [1,65535]");
}
void validate_telemetry(const nlohmann::json& value) {
  require(value.is_object() && value.size()<=32,"Telemetry must have at most 32 scalar fields");
  for(auto it=value.begin();it!=value.end();++it){
    require(!it.key().empty() && it.key().size()<=64,"Invalid telemetry key");
    const auto& v=it.value();
    require(v.is_boolean() || (v.is_string() && v.get_ref<const std::string&>().size()<=128) ||
      (v.is_number() && std::isfinite(v.get<double>())),"Invalid telemetry scalar");
  }
}
Connection::~Connection(){if(fd_>=0)::close(fd_);}
Connection::Connection(Connection&& other) noexcept:fd_(std::exchange(other.fd_,-1)){}
Connection& Connection::operator=(Connection&& other) noexcept {
  if(this!=&other){if(fd_>=0)::close(fd_);fd_=std::exchange(other.fd_,-1);}return *this;
}
Connection Connection::connect(const std::string& host,int port,Deadline deadline) {
  auto addr=address(host,port);
  Connection socket(::socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0));
  if(socket.fd_<0)throw error("socket");
  low_latency(socket.fd_);
  int result=::connect(socket.fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr));
  if(result<0 && errno!=EINPROGRESS)throw error("connect");
  if(result<0){
    wait_socket(socket.fd_,POLLOUT,deadline);
    int status=0;socklen_t size=sizeof(status);
    if(::getsockopt(socket.fd_,SOL_SOCKET,SO_ERROR,&status,&size)<0)throw error("connect status");
    if(status){errno=status;throw error("connect");}
  }
  return socket;
}
void Connection::transfer(void* buffer,size_t size,bool writing,Deadline deadline) {
  auto* data=static_cast<uint8_t*>(buffer);
  while(size){
    wait_socket(fd_,writing?POLLOUT:POLLIN,deadline);
    ssize_t count=writing ? ::send(fd_,data,size,MSG_NOSIGNAL) : ::recv(fd_,data,size,0);
    if(count<0 && (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK))continue;
    if(count<0)throw error(writing?"send":"receive");
    if(count==0)throw std::runtime_error("External adapter peer disconnected");
    data+=count;size-=size_t(count);
  }
}
void Connection::send(const Message& message,Deadline deadline) {
  require(message.header.is_object() && message.header.value("protocol",std::string())==wire_version,
    "Invalid external protocol header");
  auto text=message.header.dump();
  require(!text.empty() && text.size()<=max_header_bytes && message.payload.size()<=max_payload_bytes,
    "External adapter message exceeds size limit");
  uint32_t prefix[3]{htonl(0x524d5331),htonl(uint32_t(text.size())),htonl(uint32_t(message.payload.size()))};
  transfer(prefix,sizeof(prefix),true,deadline);
  transfer(text.data(),text.size(),true,deadline);
  transfer(const_cast<uint8_t*>(message.payload.data()),message.payload.size(),true,deadline);
}
Message Connection::receive(Deadline deadline) {
  uint32_t prefix[3];transfer(prefix,sizeof(prefix),false,deadline);
  auto header_size=ntohl(prefix[1]),payload_size=ntohl(prefix[2]);
  require(ntohl(prefix[0])==0x524d5331,"Invalid external adapter frame magic");
  require(header_size>0 && header_size<=max_header_bytes && payload_size<=max_payload_bytes,
    "External adapter frame exceeds size limit");
  std::string text(header_size,'\0');transfer(text.data(),text.size(),false,deadline);
  Message message;message.header=nlohmann::json::parse(text);
  require(message.header.is_object() && message.header.value("protocol",std::string())==wire_version,
    "External protocol version mismatch");
  message.payload.resize(payload_size);transfer(message.payload.data(),message.payload.size(),false,deadline);
  return message;
}
Listener::Listener(const std::string& host,int port) {
  auto addr=address(host,port);
  fd_=::socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
  if(fd_<0)throw error("listen socket");
  int reuse=1;::setsockopt(fd_,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
  if(::bind(fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0 || ::listen(fd_,1)<0) {
    auto exception=error("listen");::close(fd_);fd_=-1;throw exception;
  }
}
Listener::~Listener(){if(fd_>=0)::close(fd_);}
Connection Listener::accept(Deadline deadline) {
  for(;;){
    wait_socket(fd_,POLLIN,deadline);
    int peer=::accept4(fd_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
    if(peer>=0){Connection result(peer);low_latency(peer);return result;}
    if(errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK)continue;
    throw error("accept");
  }
}
} // namespace rm_sim::adapters
