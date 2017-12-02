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
#include <assert.h>
#include "hyperudp/tx_sessions.h"

namespace hudp {

bool TxSessions::Init(size_t htable_size,
                      uint32_t frag_len,
                      std::vector<uint32_t> tmo_vec, 
                      OnSendFrag on_send,
                      OnSessDone on_done) {

  if (!hash_table_.InitForWrite("", htable_size)) {
    ELOG("hash table init failed! size = %lu", htable_size);
    return false;
  }
  frag_len_ = frag_len;
  tmo_vec_ = std::move(tmo_vec);
  timerw_ = nullptr;
  send_frag_ = std::move(on_send);
  sess_done_ = std::move(on_done);
  return true;
}

bool TxSessions::AddSession(const Buf& buf, const Addr& addr, uint32_t seq,
                            void* ctx) {
  NodeKey key = {addr.ip(), addr.port(), seq};
  bool is_found;
  Node* node = (Node*)hash_table_.FindOrAlloc(key, &is_found);
  if (!node) {
    // overflow
    WLOG("TxSession hash-table overflow!");
    return false;
  } else if (is_found) {
    // dup seq
    WLOG("TxSession dup key found!");
    return false;
  }
  if (buf.len() <= 0) {
    return false;
  }
  // new hash-table node
  node->key = key;
  node->pkt_len = buf.len();
  node->pkt_ptr = buf.ptr();
  size_t frag_num = (node->pkt_len <= frag_len_ ? 1 :
                    (node->pkt_len + frag_len_ - 1) / frag_len_);
  if (frag_num >= 65536) {
    WLOG("data too large (%lu bytes, more than 65535 fragments)", buf.len());
    return false;
  } else if (frag_num <= 1) {
    node->frag_bitmap = nullptr;
  } else {
    size_t bitmap_len = (frag_num + 7) / 8;
    node->frag_bitmap = new uint8_t[bitmap_len];
    memset(node->frag_bitmap, 0xff, bitmap_len);
    node->frag_bitmap[bitmap_len-1] = (0xffU >> (bitmap_len*8-frag_num));
  }
  node->ctx = ctx;
  new (&node->timer_owner) ccb::TimerOwner();
  // set timer
  uint16_t count = 0;
  timerw()->AddTimer(tmo_vec_[count], [this, node, count] {
    OnRetransTimer(node, count);
  }, &node->timer_owner);
  // send all fragments
  size_t left = node->pkt_len;
  for (size_t i = 0; i < frag_num; i++) {
    void*  pkt = (void*)((uint8_t*)node->pkt_ptr + frag_len_ * i);
    size_t len = (left < frag_len_ ? left : frag_len_); 
    send_frag_({pkt, len}, {node->key.ip, node->key.port}, 
               seq, frag_num, i, node->ctx);
    left -= len;
  }

  return true;
}

bool TxSessions::AckSession(const Addr& addr, uint32_t seq, 
                            uint16_t frag_count, uint16_t frag_index) {
  NodeKey key = {addr.ip(), addr.port(), seq};
  Node* node = (Node*)hash_table_.Find(key);
  if (!node) {
    // not found
    DLOG("AckSession not found seq:%u", seq);
    // todo: add counter
    return false;
  }
  size_t frag_num = (node->pkt_len <= frag_len_ ? 1 :
                    (node->pkt_len + frag_len_ - 1) / frag_len_);
  if (frag_count != frag_num || frag_index >= frag_num) {
    // bad fragment index
    ILOG("AckSession bad param frag_count:%hu frag_index:%hu", 
                                      frag_count, frag_index);
    // todo: add counter
    return false;
  }
  if (!node->frag_bitmap) {
    if (frag_index == 0) {
      // single fragment packet acked
      sess_done_(R_SUCCESS, node->ctx);
      DelSession(node);
    }
  } else {
    node->frag_bitmap[frag_index / 8] &= ~(1UL << (frag_index % 8));
    // check if all fragments acked
    bool all_frags_acked = true;
    for (size_t i = 0; i < frag_num; i += 8) {
      if (node->frag_bitmap[i / 8]) {
        all_frags_acked = false;
        break;
      }
    }
    if (all_frags_acked) {
      sess_done_(R_SUCCESS, node->ctx);
      DelSession(node);
    }
  }
  return true;
}

void TxSessions::DelSession(Node* node) {
  node->timer_owner.~TimerOwner();
  node->key.port = 0;
  node->pkt_ptr = nullptr;
  node->pkt_len = 0;
  if (node->frag_bitmap) {
    delete[] node->frag_bitmap;
    node->frag_bitmap = nullptr;
  }
  node->ctx = nullptr;
}

void TxSessions::OnRetransTimer(TxSessions::Node* node, uint16_t count) {
  assert(node && node->key.port && node->pkt_ptr);
  count = count + 1;
  if (count < tmo_vec_.size()) {
    // retransmit
    if (!node->frag_bitmap) {
      // single fragment
      send_frag_({node->pkt_ptr, node->pkt_len}, 
                 {node->key.ip, node->key.port}, 
                  node->key.seq, 1, 0, node->ctx);
      ILOG("restrans DATA len:%u id:(-, -, %u, 1, 0)", 
                        node->pkt_len, node->key.seq);
    } else {
      // multiple fragments
      size_t frag_num = (node->pkt_len + frag_len_ - 1) / frag_len_;
      size_t send_count = 0;
      for (size_t i = 0; i < frag_num; ) {
        if ((i % 8) == 0 && node->frag_bitmap[i / 8] == 0) {
          i += 8;
          continue;
        }
        if (node->frag_bitmap[i / 8] & (1UL << (i % 8))) {
          void*  pkt = (void*)((uint8_t*)node->pkt_ptr + frag_len_ * i);
          size_t left = (uint8_t*)node->pkt_ptr + node->pkt_len - (uint8_t*)pkt;
          size_t len = (left < frag_len_ ? left : frag_len_); 
          send_frag_({pkt, len}, {node->key.ip, node->key.port}, 
                     node->key.seq, frag_num, i, node->ctx);
          ILOG("restrans DATA len:%u id:(%u, %lu, %lu)", 
                       len, node->key.seq, frag_num, i);
          send_count++;
        }
        i++;
      }
      assert(send_count > 0);
    }
    // set timer
    timerw()->AddTimer(tmo_vec_[count], [this, node, count] {
      OnRetransTimer(node, count);
    }, &node->timer_owner);
  } else {
    // fail
    sess_done_(R_TIMEOUT, node->ctx);
    DelSession(node);
  }
}

} 

