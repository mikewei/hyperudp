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
#ifndef _HUDP_REUSE_PORT_UDP_IO_H
#define _HUDP_REUSE_PORT_UDP_IO_H

#include <atomic>
#include <thread>
#include <vector>
#include <memory>
#include "hyperudp/env.h"
#include "hyperudp/addr.h"
#include "hyperudp/buf.h"
#include "hyperudp/udp_io.h"

namespace hudp {

class ReusePortUdpIO : public UdpIO
{
public:
  ReusePortUdpIO(const Env& env);
  ~ReusePortUdpIO() override;
  bool Init(const Addr& listen_addr, OnRecv on_recv) override;
  bool Send(const Buf& buf, const Addr& addr) override;
  void Cleanup() override;

private:
  class ThreadContext;

  const Env& env_;
  OnRecv on_recv_;
  std::vector<std::unique_ptr<ThreadContext>> ctx_vec_;
  static std::atomic<size_t> cur_thread_index_s;
};

} // namespace hudp

#endif // _HUDP_REUSE_PORT_UDP_IO_H
