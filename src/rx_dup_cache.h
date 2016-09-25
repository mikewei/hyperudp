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
#ifndef _HUDP_RX_DUP_CACHE_H
#define _HUDP_RX_DUP_CACHE_H

#include "ccbase/timer_wheel.h"
#include "shmc/shm_hash_table.h"
#include "hyperudp/env.h"

namespace hudp {

class Peer;

class RxDupCache
{
public:
  RxDupCache(const Env& env)
    : env_(env) {}

  bool Init(bool enable, size_t htable_size, uint32_t timeout);
  bool CheckDup(Peer* peer, uint32_t proc_sess_id, uint32_t seq);

  struct NodeKey {
    uint32_t proc_sess_id;
    uint32_t base_seq;
    uint32_t ip;
    uint16_t port;

    bool operator==(const NodeKey& other) const {
      return proc_sess_id == other.proc_sess_id &&
             base_seq == other.base_seq &&
             ip == other.ip &&
             port == other.port;
    }
  };
  struct Node {
    NodeKey         key;
    uint64_t        seq_bitmap;
    ccb::tick_t     mtime;

    std::pair<bool, NodeKey> Key(const RxDupCache* rdc) const volatile {
      return {rdc->IsValidNode((const Node&)(*this)), (const NodeKey&)key};
    }
  };

private:
  bool IsValidNode(const Node& node) const;

  shmc::ShmHashTable<NodeKey, Node, shmc::HEAP> hash_table_;
  const Env& env_;
  bool enabled_ = false;
  uint32_t timeout_;
};

inline bool RxDupCache::IsValidNode(const Node& node) const
{
  return (node.key.port != 0 && 
          node.mtime + timeout_ > env_.timerw()->GetCurrentTick());
}

} // namespace hudp

namespace std {

template <>
struct hash<hudp::RxDupCache::NodeKey>
{
  std::size_t operator()(const hudp::RxDupCache::NodeKey& key) const {
    return (key.ip + ((size_t)key.port << 16UL)
                   + ((size_t)key.base_seq << 32UL)
                   + ((size_t)key.proc_sess_id << 32UL));
  }
};

} // namespace std

#endif // _HUDP_RX_DUP_CACHE_H
