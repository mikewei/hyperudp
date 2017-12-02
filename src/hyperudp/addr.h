/* Copyright (c) 2016-2017, Bin Wei <bin@vip.qq.com>
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
 *     * The names of its contributors may not be used to endorse or 
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
#ifndef HUDP_ADDR_H_
#define HUDP_ADDR_H_

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>
#include <utility>

namespace hudp {

class Addr {
 public:
  Addr() = default;
  Addr(const sockaddr_in& sa) 
    : sa_(sa) {}
  Addr(const char* ip, uint16_t port) {
    sa_.sin_family = AF_INET;
    sa_.sin_addr.s_addr = inet_addr(ip);
    sa_.sin_port = htons(port);
  }
  Addr(uint32_t ip, uint16_t port) {
    sa_.sin_family = AF_INET;
    sa_.sin_addr.s_addr = htonl(ip);
    sa_.sin_port = htons(port);
  }
  Addr(const std::string& ip, uint16_t port)
    : Addr(ip.c_str(), port) {}
  Addr(const Addr& addr)
    : Addr(addr.sa_) {}

  // factory method
  static std::pair<bool, Addr> ParseFromString(const std::string& ip_port);

  bool operator == (const Addr& other) const {
    return (sa_.sin_family == other.sa_.sin_family
         && sa_.sin_addr.s_addr == other.sa_.sin_addr.s_addr
         && sa_.sin_port == other.sa_.sin_port);
  }
  bool operator != (const Addr& other) const {
    return !((*this) == other);
  }

  const sockaddr* sockaddr_ptr() const {
    return (const sockaddr*)&sa_;
  }
  sockaddr* sockaddr_ptr() {
    return (sockaddr*)&sa_;
  }
  socklen_t sockaddr_len() const {
    return sizeof(sa_);
  }
  uint32_t ip() const {
    return ntohl(sa_.sin_addr.s_addr);
  }
  uint16_t port() const {
    return ntohs(sa_.sin_port);
  }
  uint32_t n_ip() const {
    return sa_.sin_addr.s_addr;
  }
  uint16_t n_port() const {
    return sa_.sin_port;
  }
  const char* ip_str() const {
    static thread_local char buf[32];
    uint32_t ip = this->ip();
    int r = snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
        static_cast<unsigned>((ip >> 24) & 0xff),
        static_cast<unsigned>((ip >> 16) & 0xff),
        static_cast<unsigned>((ip >> 8) & 0xff),
        static_cast<unsigned>(ip & 0xff));
    if (r < 0 || static_cast<size_t>(r) >= sizeof(buf))
      return "<error>";
    return buf;
  }
  const char* str() const {
    static thread_local char buf[32];
    uint32_t ip = this->ip();
    int r = snprintf(buf, sizeof(buf), "%u.%u.%u.%u:%u",
        static_cast<unsigned>((ip >> 24) & 0xff),
        static_cast<unsigned>((ip >> 16) & 0xff),
        static_cast<unsigned>((ip >> 8) & 0xff),
        static_cast<unsigned>(ip & 0xff),
        static_cast<unsigned>(port()));
    if (r < 0 || static_cast<size_t>(r) >= sizeof(buf))
      return "<error>";
    return buf;
  }

private:
  sockaddr_in sa_;
};

}  // namespace hudp

namespace std {

template <>
struct hash<hudp::Addr> {
  std::size_t operator()(const hudp::Addr& key) const {
    // consider balance when used as ( hash() % 2/4/8 )
    return (key.ip() + key.port() + ((size_t)key.ip() << 32UL)
                                  + ((size_t)key.port() << 48UL));
  }
};

}  // namespace std

#endif  // HUDP_ADDR_H_
