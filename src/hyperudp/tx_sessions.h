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
#ifndef _HUDP_TX_SESSIONS_H
#define _HUDP_TX_SESSIONS_H

#include <vector>
#include "ccbase/worker_group.h"
#include "shmc/shm_hash_table.h"
#include "hyperudp/env.h"
#include "hyperudp/buf.h"
#include "hyperudp/addr.h"
#include "hyperudp/hyperudp.h"
#include "hyperudp/chunk_alloc.h"

namespace hudp {

class TxSessions
{
public:
  using OnSendFrag = ccb::ClosureFunc<void(const Buf&,
                                           const Addr&,
                                           uint32_t seq,
                                           uint16_t frag_count,
                                           uint16_t frag_index,
                                           void* ctx)>;
  using OnSessDone = ccb::ClosureFunc<void(Result, void* ctx)>;

  TxSessions(const Env& env)
    : env_(env) {}

  bool Init(size_t htable_size,
            uint32_t frag_len,
            std::vector<uint32_t> tmo_vec, 
            OnSendFrag on_send,
            OnSessDone on_done);

  bool AddSession(const Buf& buf, const Addr& addr, uint32_t seq, void* ctx);
  bool AckSession(const Addr& addr, uint32_t seq, uint16_t frag_count,
                  uint16_t frag_index);

private:
  struct Node;
  void OnRetransTimer(Node* node, uint16_t count);
  void DelSession(Node* node);
  ccb::TimerWheel* timerw() {
    if (!timerw_) {
      timerw_ = env_.timerw();
    }
    return timerw_;
  }

public:
  struct NodeKey {
    uint32_t ip;
    uint16_t port;
    uint32_t seq;

    bool operator==(const NodeKey& other) const {
      return ip == other.ip &&
             port == other.port &&
             seq == other.seq;
    }
  };

private:
  struct Node {
    NodeKey         key;
    uint32_t        pkt_len;
    const void*     pkt_ptr;
    uint8_t*        frag_bitmap;
    void*           ctx;
    ccb::TimerOwner timer_owner;

    std::pair<bool, NodeKey> Key() const volatile {
      return {key.port != 0, (const NodeKey&)key};
    }
  };

  shmc::ShmHashTable<NodeKey, Node, shmc::HEAP> hash_table_;
  const Env& env_;
  uint32_t frag_len_;
  std::vector<uint32_t> tmo_vec_;
  ccb::TimerWheel* timerw_;
  OnSendFrag send_frag_;
  OnSessDone sess_done_;
};

}

namespace std {

template <>
struct hash<hudp::TxSessions::NodeKey>
{
  std::size_t operator()(const hudp::TxSessions::NodeKey& key) const {
    return (key.ip + ((size_t)key.port << 16UL)
                   + ((size_t)key.seq << 32UL));
  }
};

}

#endif // _HUDP_TX_SESSIONS_H
