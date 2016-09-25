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
#ifndef _HUDP_PEER_H
#define _HUDP_PEER_H

#include "ccbase/timer_wheel.h"
#include "ccbase/worker_group.h"
#include "hyperudp/addr.h"
#include "hyperudp/env.h"
#include "hyperudp/tx_buffer.h"
#include "hyperudp/tx_session_manager.h"
#include "hyperudp/rx_dup_cache.h"

namespace hudp {

class Peer
{
public:
  using OnFlush = TxBuffer::OnFlush;

  Peer(const Env& env, const Addr& addr, uint32_t init_seq,
       const OnFlush& on_flush);

  const Addr& addr() const {
    return addr_;
  }
  TxBuffer* tx_buffer() {
    return &tx_buffer_;
  }
  uint32_t next_seq() {
    return next_seq_++;
  }

  // recent used TxSessionManager node cache
  struct TxSessNode {
    TxSessionManager::Node* node = nullptr;
    uint32_t base_seq = 0;
  };
  TxSessNode* txs_add_cache() {
    return &txs_add_cache_;
  }
  TxSessNode* txs_ack_cache() {
    return &txs_ack_cache_;
  }

  // recent used RxDupCache node cache
  struct RDCNode {
    RxDupCache::Node* node = nullptr;
    uint32_t base_seq = 0;
  };
  RDCNode* rdc_cache() {
    return &rdc_cache_;
  }

private:
  const Env& env_;
  Addr addr_;
  uint32_t next_seq_;
  TxBuffer tx_buffer_;
  TxSessNode txs_add_cache_;
  TxSessNode txs_ack_cache_;
  RDCNode rdc_cache_;
};

} // namespace hudp

#endif // _HUDP_PEER_H
