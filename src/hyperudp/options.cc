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
#include <limits>
#include "gflags/gflags.h"
#include "hyperudp/options.h"
#include "hyperudp/constants.h"
#include "hyperudp/module_registry.h"

namespace hudp {

class UdpIO;
class ChunkAlloc;
class TxDelayAlgo;

#define GFLAGS_DEFINE(name, type, deft, desc) \
  DEFINE_##type(hudp_##name, deft, desc)
#define GFLAGS_DEFINE_U64(name, desc) GFLAGS_DEFINE(name, uint64, 0, desc)
#define GFLAGS_DEFINE_STR(name, desc) GFLAGS_DEFINE(name, string, "", desc)
#define GFLAGS_DEFINE_BOOL(name, desc) GFLAGS_DEFINE(name, bool, false, desc)

#define GFLAGS_MAY_OVERRIDE(name, setter) do { \
  google::CommandLineFlagInfo info; \
  if (google::GetCommandLineFlagInfo("hudp_" #name, &info) \
      && !info.is_default) { \
    setter(FLAGS_hudp_##name); \
  } \
} while(0)

// WorkerGroup options
GFLAGS_DEFINE_U64(worker_num, "number of worker threads");
GFLAGS_DEFINE_U64(worker_queue_size, "size of queue consumed by worker");
// TxSessionManager options
GFLAGS_DEFINE_U64(max_tx_sessions, "max number of pending tx sessions");
GFLAGS_DEFINE_U64(max_udp_pkt_size,
                  "max udp packet size without fragmentation");
// TxBuffer options
GFLAGS_DEFINE_STR(tx_delay_algo_module,
                  "module name of tx-delay-algorithm implementation");
GFLAGS_DEFINE_U64(max_tx_delay,
                  "max tx delay when using max-delay-algorithm family");
// RxFragCache options
GFLAGS_DEFINE_U64(max_rx_frag_cache_nodes,
                 "max nodes of rx-frag-cache for pending imcomplete packets");
GFLAGS_DEFINE_U64(rx_frag_cache_timeout,
                  "timeout of rx-frag-cache for pending imcomplete packets");
// RxDupCache options
GFLAGS_DEFINE_BOOL(enable_rx_dup_cache, "enable rx-dup-cache");
GFLAGS_DEFINE_U64(rx_dup_cache_size, "size of rx-dup-cache");
GFLAGS_DEFINE_U64(rx_dup_cache_timeout, "timeout of rx-dup-cache");
// UdpIO options
GFLAGS_DEFINE_STR(udp_io_module, "module name of udp_io implementation");
// AllocChunk options
GFLAGS_DEFINE_STR(chunk_alloc_module,
                  "module name of chunk_alloc implementation");

OptionsBuilder::OptionsBuilder()
  : opt_(new Options) {
  // WorkerGroup options
  WorkerNumber(2);
  WorkerQueueSize(10000);
  // TxSessionManager options
  MaxTxSessions(200000);
  MaxUdpPktSize(1400);
  RetransTimeouts({200, 1000});
  // TxBuffer options
  TxDelayAlgoModule("tb_max_tx_delay");
  MaxTxDelay(5);
  // RxFragCache options
  MaxRxFragCacheNodes(200000);
  RxFragCacheTimeout(2000);
  // RxDupCache options
  EnableRxDupCache(true);
  RxDupCacheSize(4000000);
  RxDupCacheTimeout(2000);
  // UdpIO options
  UdpIOModule("simple");
  // AllocChunk options
  ChunkAllocModule("simple");
}

OptionsBuilder::~OptionsBuilder() {
}

Options OptionsBuilder::Build() {
  GFLAGS_MAY_OVERRIDE(worker_num, WorkerNumber);
  GFLAGS_MAY_OVERRIDE(worker_queue_size, WorkerQueueSize);
  GFLAGS_MAY_OVERRIDE(max_tx_sessions, MaxTxSessions);
  GFLAGS_MAY_OVERRIDE(max_udp_pkt_size, MaxUdpPktSize);
  GFLAGS_MAY_OVERRIDE(tx_delay_algo_module, TxDelayAlgoModule);
  GFLAGS_MAY_OVERRIDE(max_tx_delay, MaxTxDelay);
  GFLAGS_MAY_OVERRIDE(max_rx_frag_cache_nodes, MaxRxFragCacheNodes);
  GFLAGS_MAY_OVERRIDE(rx_frag_cache_timeout, RxFragCacheTimeout);
  GFLAGS_MAY_OVERRIDE(enable_rx_dup_cache, EnableRxDupCache);
  GFLAGS_MAY_OVERRIDE(rx_dup_cache_size, RxDupCacheSize);
  GFLAGS_MAY_OVERRIDE(rx_dup_cache_timeout, RxDupCacheTimeout);
  GFLAGS_MAY_OVERRIDE(udp_io_module, UdpIOModule);
  GFLAGS_MAY_OVERRIDE(chunk_alloc_module, ChunkAllocModule);
  return Options(*opt_);
}

OptionsBuilder& OptionsBuilder::LogHandler(LogLevel lv, 
                     ccb::ClosureFunc<void(LogLevel, const char*)> f) {
  opt_->log_lv = lv;
  opt_->log_f  = std::move(f);
  return *this;
}

// WorkerGroup options

OptionsBuilder& OptionsBuilder::WorkerNumber(size_t num) {
  if (num <= 0 || num >= std::numeric_limits<uint16_t>::max()) {
    throw std::invalid_argument("Invalid value! expect [1, 65535]");
  }
  opt_->worker_num = num;
  return *this;
}

OptionsBuilder& OptionsBuilder::WorkerQueueSize(size_t num) {
  if (num <= 0) {
    throw std::invalid_argument("Invalid value!");
  }
  opt_->worker_queue_size = num;
  return *this;
}

// TxSessionManager options

OptionsBuilder& OptionsBuilder::MaxTxSessions(size_t num) {
  if (num < opt_->worker_num) {
    throw std::invalid_argument("Invalid value!");
  }
  opt_->max_tx_sessions = num;
  return *this;
}

OptionsBuilder& OptionsBuilder::MaxUdpPktSize(size_t num) {
  if (num > kMaxUdpPktSize) num = kMaxUdpPktSize;
  else if (num < kMinMaxUdpPktSize) num = kMinMaxUdpPktSize;
  opt_->max_udp_pkt_size = num;
  return *this;
}

OptionsBuilder& OptionsBuilder::RetransTimeouts(std::vector<uint32_t> vec) {
  opt_->retrans_timeouts = std::move(vec);
  return *this;
}

// TxBuffer options

OptionsBuilder& OptionsBuilder::TxDelayAlgoModule(std::string name) {
  if (!ModuleRegistry<TxDelayAlgo>::Get()->IsModuleAvailable(name)) {
    throw std::runtime_error("Unavailable TxDelayAlgo module: " + name);
  }
  opt_->tx_delay_algo_module = name;
  return *this;
}

OptionsBuilder& OptionsBuilder::MaxTxDelay(size_t ms) {
  if (ms > 60000) ms = 60000;
  opt_->max_tx_delay = ms;
  return *this;
}

// RxFragCache options

OptionsBuilder& OptionsBuilder::MaxRxFragCacheNodes(size_t num) {
  if (num < opt_->worker_num) {
    throw std::invalid_argument("Invalid value!");
  }
  opt_->max_rx_frag_cache_nodes = num;
  return *this;
}

OptionsBuilder& OptionsBuilder::RxFragCacheTimeout(size_t ms) {
  if (ms > std::numeric_limits<uint32_t>::max() || ms <= 0) {
    throw std::invalid_argument("Invalid timeout value!");
  }
  opt_->rx_frag_cache_timeout = ms;
  return *this;
}

// RxDupCache options

OptionsBuilder& OptionsBuilder::EnableRxDupCache(bool on) {
  opt_->enable_rx_dup_cache = on;
  return *this;
}

OptionsBuilder& OptionsBuilder::RxDupCacheSize(size_t num) {
  opt_->rx_dup_cache_size = num;
  return *this;
}

OptionsBuilder& OptionsBuilder::RxDupCacheTimeout(size_t ms) {
  if (ms > std::numeric_limits<uint32_t>::max() || ms <= 0) {
    throw std::invalid_argument("Invalid timeout value!");
  }
  opt_->rx_dup_cache_timeout = ms;
  return *this;
}

// HyperProto options
OptionsBuilder& OptionsBuilder::Proxy(ccb::ClosureFunc<Addr(const Addr&)> f) {
  opt_->proxy_f = f;
  return *this;
}

// UdpIO options

OptionsBuilder& OptionsBuilder::UdpIOModule(std::string name) {
  if (!ModuleRegistry<UdpIO>::Get()->IsModuleAvailable(name)) {
    throw std::runtime_error("Unavailable UdpIO module: " + name);
  }
  opt_->udp_io_module = name;
  return *this;
}

// AllocChunk options

OptionsBuilder& OptionsBuilder::ChunkAllocModule(std::string name) {
  if (!ModuleRegistry<ChunkAlloc>::Get()->IsModuleAvailable(name)) {
    throw std::runtime_error("Unavailable ChunkAlloc module: " + name);
  }
  opt_->chunk_alloc_module = name;
  return *this;
}

}
