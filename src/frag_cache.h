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
#ifndef _HUDP_FRAG_CACHE_H
#define _HUDP_FRAG_CACHE_H

#include <vector>
#include "ccbase/worker_group.h"
#include "shmc/shm_hash_table.h"
#include "hyperudp/env.h"
#include "hyperudp/buf.h"
#include "hyperudp/addr.h"
#include "hyperudp/hyperudp.h"
#include "hyperudp/chunk_alloc.h"

namespace hudp {

class FragCache
{
public:
  using OnComplete = ccb::ClosureFunc<void(const Addr& addr,
                                           uint32_t proc_sess_id,
                                           uint32_t seq,
                                           uint16_t frag_count,
                                           void** frag_list,
                                           Result result)>;

  FragCache(const Env& env)
    : env_(env) {}

  bool Init(size_t htable_size,
            uint32_t timeout,
            OnComplete on_complete);

  bool AddFrag(const Addr& addr, uint32_t proc_sess_id, uint32_t seq,
               uint16_t frag_count, uint16_t frag_index, void* frag);

private:
  struct Node;
  void DeleteNode(Node* node);
  void OnNodeTimeout(Node* node);
  ccb::TimerWheel* timerw() {
    if (!timerw_) {
      timerw_ = env_.timerw();
    }
    return timerw_;
  }

public:
  struct NodeKey {
    uint32_t proc_sess_id;
    uint32_t seq;
    uint32_t ip;
    uint16_t port;

    bool operator==(const NodeKey& other) const {
      return proc_sess_id == other.proc_sess_id &&
             seq == other.seq &&
             ip == other.ip &&
             port == other.port;
    }
  };

private:
  struct Node {
    NodeKey         key;
    uint16_t        frag_count;
    uint16_t        cur_frags;
    void**          frag_list;
    void*           frag_buf[3];
    ccb::TimerOwner timer_owner;

    std::pair<bool, NodeKey> Key() const volatile {
      return {key.port != 0, (const NodeKey&)key};
    }
  };

  shmc::ShmHashTable<NodeKey, Node, shmc::HEAP> hash_table_;
  const Env& env_;
  uint32_t timeout_;
  OnComplete on_complete_;
  ccb::TimerWheel* timerw_;
};

} // namespace hudp

namespace std {

template <>
struct hash<hudp::FragCache::NodeKey>
{
  std::size_t operator()(const hudp::FragCache::NodeKey& key) const {
    return (key.ip + ((size_t)key.port << 16UL)
                   + ((size_t)key.seq << 32UL)
                   + ((size_t)key.proc_sess_id << 32UL));
  }
};

} // namespace std

#endif // _HUDP_FRAG_CACHE_H
