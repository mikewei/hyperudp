/* Copyright (c) 2016, Bin Wei <bin@vip.qq.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *     * The name of of its contributors may not be used to endorse or 
 * promote products derived from this software without specific prior 
 * written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "hyperudp/simple_udp_io.h"

namespace hudp {

SimpleUdpIO::SimpleUdpIO(const Env& env)
  : env_(env)
{
}

SimpleUdpIO::~SimpleUdpIO()
{
  Cleanup();
}

bool SimpleUdpIO::Init(const Addr& listen_addr, OnRecv on_recv)
{
  if (sock_ >= 0)
    return false;
  if ((sock_ = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    return false;
  if (bind(sock_, listen_addr.sockaddr_ptr(), listen_addr.sockaddr_len()) < 0)
    return false;
  struct timeval rto{0, 4000};
  if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (void*)&rto, sizeof(rto)) < 0)
    return false;

  on_recv_ = on_recv;

  stop_ = false;
  thread_ = std::thread([this] {
    char buf[65536];
    Addr addr{"0.0.0.0", 0};
    socklen_t addr_len = addr.sockaddr_len();
    while (!stop_) {
      int r = recvfrom(sock_, buf, sizeof(buf), 0, 
                       addr.sockaddr_ptr(), &addr_len);
      if (r > 0 && on_recv_)
        on_recv_({buf, static_cast<size_t>(r)}, addr);
    }
    close(sock_);
  });
  return true;
}

bool SimpleUdpIO::Send(const Buf& buf, const Addr& addr)
{
  int r = sendto(sock_, buf.ptr(), buf.len(), MSG_DONTWAIT, 
                 addr.sockaddr_ptr(), addr.sockaddr_len());
  return r >= 0;
}

void SimpleUdpIO::Cleanup()
{
  if (!stop_.exchange(true)) {
    thread_.join();
  }
}

}
