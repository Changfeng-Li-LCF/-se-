#pragma once
#include "blocking_stop.hpp"
#include <cerrno>
#include <poll.h>
#include <unistd.h>

namespace racecar_control {
// fd must be O_NONBLOCK. Complete a frame or return within the software budget.
inline BlockingStop::Result write_with_deadline(int fd,const unsigned char *data,
                                               size_t length,int timeout_ms=20) {
  const auto deadline=BlockingStop::Clock::now()+std::chrono::milliseconds(timeout_ms);
  size_t offset=0;
  while(offset<length) {
    if(BlockingStop::Clock::now()>=deadline)return BlockingStop::io_timeout;
    const auto written=::write(fd,data+offset,length-offset);
    if(written>0){offset+=static_cast<size_t>(written);continue;}
    if(written<0 && errno==EINTR)continue;
    if(written<0 && errno!=EAGAIN && errno!=EWOULDBLOCK)return BlockingStop::io_error;
    const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-BlockingStop::Clock::now()).count();
    if(remaining<=0)return BlockingStop::io_timeout;
    pollfd descriptor{fd,POLLOUT,0};
    const int status=::poll(&descriptor,1,static_cast<int>(remaining));
    if(status==0)return BlockingStop::io_timeout;
    if(status<0){if(errno==EINTR)continue;return BlockingStop::io_error;}
    if(descriptor.revents&(POLLERR|POLLHUP|POLLNVAL))return BlockingStop::io_error;
  }
  return BlockingStop::ok;
}
} // namespace racecar_control
