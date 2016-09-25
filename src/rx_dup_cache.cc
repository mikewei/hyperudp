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
#include <assert.h>
#include "hyperudp/rx_dup_cache.h"
#include "hyperudp/peer.h"

namespace hudp {

bool RxDupCache::Init(bool enable, size_t htable_size, uint32_t timeout)
{
  if (!enable) {
    DLOG("RxDupCache: is disabled");
    return true;
  }
  if (!hash_table_.InitForWrite("", htable_size)) {
    ELOG("RxDupCache: hash table init failed! size = %lu", htable_size);
    return false;
  }
  enabled_ = true;
  timeout_ = timeout;
  return true;
}

bool RxDupCache::CheckDup(Peer* peer, uint32_t proc_sess_id, uint32_t seq)
{
  if (!enabled_) {
    // all packets are not dup
    return false;
  }
  NodeKey key = { proc_sess_id,
                  seq & ~0x3fU,
                  peer->addr().ip(),
                  peer->addr().port() };
  // first check cached node
  Node* node = peer->rdc_cache()->node;
  if (node &&
      key.base_seq == peer->rdc_cache()->base_seq &&
      key == node->key &&
      IsValidNode(*node)) {
    // cache hit
    DLOG("dup-cache node cache hit");
  } else {
    // cache miss, so hash search
    DLOG("dup-cache node cache miss");
    bool is_found;
    node = (Node*)hash_table_.FindOrAlloc(key, this, &is_found);
    if (!node) {
      // hash-table overflow
      WLOG("RxDupCache: hash-table overflow!");
      // return Not-Dup here is betten than Deny-Of-Service
      return false;
    } else if (!is_found) {
      // initialize new node
      node->key = key;
      node->seq_bitmap = 0UL;
      node->mtime = 0UL;
    }
    // update node cache
    peer->rdc_cache()->node = node;
    peer->rdc_cache()->base_seq = key.base_seq;
  }
  // check dup
  if (node->seq_bitmap & (1UL << (seq & 0x3fU))) {
    // dup DATA segment
    return true;
  } else {
    // valid DATA segment
    node->seq_bitmap |= (1UL << (seq & 0x3fU));
    node->mtime = env_.timerw()->GetCurrentTick();
    return false;
  }
}


} // namespace hudp

