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
#ifndef _HUDP_TX_SESSION_MANAGER_H
#define _HUDP_TX_SESSION_MANAGER_H

#include <vector>
#include "ccbase/worker_group.h"
#include "shmc/shm_hash_table.h"
#include "hyperudp/env.h"
#include "hyperudp/buf.h"
#include "hyperudp/addr.h"
#include "hyperudp/chunk_alloc.h"

namespace hudp {

class Peer;

struct TxRequest
{
  uint32_t size;
  uint32_t ip;
  uint16_t port;
  uint16_t ref_count;
  Peer* peer;
  void* ctx;
  HyperUdp::OnSent on_sent;
  uint8_t data[0];
};

class TxSessionManager
{
public:
  using OnSendFrag = ccb::ClosureFunc<void(TxRequest*,
                                           const Buf&,
                                           uint32_t seq,
                                           uint16_t frag_count,
                                           uint16_t frag_index)>;
  using OnSessDone = ccb::ClosureFunc<void(TxRequest*, Result)>;

  TxSessionManager(const Env& env)
    : env_(env) {}

  bool Init(size_t sess_num,
            uint32_t frag_len,
            std::vector<uint32_t> tmo_vec, 
            OnSendFrag on_send,
            OnSessDone on_done);

  bool AddSession(TxRequest* req) {
    // garentee TxRequest is not mutable to TxSessionManager
    return AddSessionConst(req);
  }
  bool AckSession(Peer* peer, uint32_t seq, uint16_t frag_count,
                                            uint16_t frag_index);

public:
  struct NodeKey {
    uint32_t ip;
    uint16_t port;
    uint32_t base_seq;

    bool operator==(const NodeKey& other) const {
      return ip == other.ip &&
             port == other.port &&
             base_seq == other.base_seq;
    }
  };
  struct Session {
    const TxRequest* req; // also used as reusable flag
    uint8_t*         frag_bitmap;
    ccb::TimerOwner  timer_owner;
    uint16_t         retrans_count;
    uint16_t         frag_num;
  };
  struct Node {
    NodeKey          key;
    uint64_t         inuse_bitmap;
    Session          sessions[64];

    std::pair<bool, NodeKey> Key() const volatile {
      return {inuse_bitmap != 0, (const NodeKey&)key};
    }
  };

private:
  bool AddSessionConst(const TxRequest* req);
  void OnRetransTimer(Node* node, uint32_t seq_index);
  void DelSession(Node* node, uint32_t seq_index);
  ccb::TimerWheel* timerw() {
    if (!timerw_) {
      timerw_ = env_.timerw();
    }
    return timerw_;
  }

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
struct hash<hudp::TxSessionManager::NodeKey>
{
  std::size_t operator()(const hudp::TxSessionManager::NodeKey& key) const {
    return (key.ip + ((size_t)key.port << 16UL)
                   + ((size_t)key.base_seq << 32UL));
  }
};

}

#endif // _HUDP_TX_SESSION_MANAGER_H
