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
#include "hyperudp/frag_cache.h"

namespace hudp {

bool FragCache::Init(size_t htable_size,
                     uint32_t timeout,
                     OnComplete on_complete)
{
  if (!hash_table_.InitForWrite("", htable_size)) {
    ELOG("FragCache: hash table init failed! size = %lu", htable_size);
    return false;
  }
  timeout_ = timeout;
  on_complete_ = std::move(on_complete);
  timerw_ = nullptr;
  return true;
}

bool FragCache::AddFrag(const Addr& addr, uint32_t proc_sess_id, uint32_t seq,
                        uint16_t frag_count, uint16_t frag_index, void* frag)
{
  assert(frag_index < frag_count);
  NodeKey key = {proc_sess_id, seq, addr.ip(), addr.port()};
  bool is_found;
  Node* node = (Node*)hash_table_.FindOrAlloc(key, &is_found);
  if (!node) {
    // overflow
    WLOG("FragCache: hash-table overflow!");
    return false;
  } else if (!is_found) {
    // initialize new node
    node->key = key;
    node->frag_count = frag_count;
    node->cur_frags = 0;
    if (frag_count <= sizeof(node->frag_buf)/sizeof(void*)) {
      node->frag_list = node->frag_buf;
    } else {
      node->frag_list = new void* [frag_count];
    }
    for (size_t i = 0; i < node->frag_count; i++) {
      node->frag_list[i] = nullptr;
    }
    new (&node->timer_owner) ccb::TimerOwner();
    timerw()->AddTimer(timeout_, [this, node] {
      OnNodeTimeout(node);
    }, &node->timer_owner);
  } else {
    // check existed node
    assert(node->cur_frags < node->frag_count);
    if (node->frag_count != frag_count) {
      ILOG("AddFrag: drop frag as frag_count dismatch");
      return false;
    }
    if (node->frag_list[frag_index]) {
      ILOG("AddFrag: drop dup frag");
      return false;
    }
  }
  // add frag to node
  node->frag_list[frag_index] = frag;
  if (++node->cur_frags == node->frag_count) {
    for (size_t i = 0; i < node->frag_count; i++) {
      assert(node->frag_list[i]);
    }
    on_complete_({node->key.ip, node->key.port},
                 node->key.proc_sess_id, node->key.seq, 
                 node->frag_count, node->frag_list, R_SUCCESS);
    DeleteNode(node);
  }

  return true;
}

void FragCache::OnNodeTimeout(Node* node)
{
  on_complete_({node->key.ip, node->key.port},
               node->key.proc_sess_id, node->key.seq, 
               node->frag_count, node->frag_list, R_TIMEOUT);
  DeleteNode(node);
}

void FragCache::DeleteNode(Node* node)
{
  node->timer_owner.~TimerOwner();
  if (node->frag_list != node->frag_buf)
    delete[] node->frag_list;
  node->frag_list = nullptr;
  node->key.port = 0;
}


} 

