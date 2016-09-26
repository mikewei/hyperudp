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
#ifndef _HUDP_OPTIONS_H
#define _HUDP_OPTIONS_H

#include "hyperudp/hyperudp.h"

namespace hudp {

class Options
{
public:
  // Options is only copyable
  Options(const Options&) = default;
  Options& operator=(const Options&) = default;

  // Utility options
  int log_lv = 0;
  ccb::ClosureFunc<void(LogLevel, const char*)> log_f = nullptr;
  // WorkerGroup options
  size_t worker_num = 0;
  size_t worker_queue_size = 10000;
  // TxSessionManager options
  size_t max_tx_sessions = 0;
  size_t max_udp_pkt_size = 0;
  std::vector<uint32_t> retrans_timeouts;
  // TxBuffer options
  std::string tx_delay_algo_module;
  size_t max_tx_delay = 0;
  // RxFragCache options
  size_t max_rx_frag_cache_nodes = 0;
  uint32_t rx_frag_cache_timeout = 0;
  // RxDupCache options
  bool enable_rx_dup_cache = true;
  size_t rx_dup_cache_size = 0;
  uint32_t rx_dup_cache_timeout = 0;
  // HyperProto options
  ccb::ClosureFunc<Addr(const Addr&)> proxy_f = nullptr;
  // UdpIO options
  std::string udp_io_module;
  // AllocChunk options
  std::string chunk_alloc_module;

private:
  // non-copy contruction is private
  friend class OptionsBuilder;
  Options() = default;
};

}

#endif // _HUDP_OPTIONS_H
