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
#include "ccbase/closure.h"
#include "hyperudp/peer.h"
#include "hyperudp/tx_session_manager.h"

namespace hudp {

bool TxSessionManager::Init(size_t sess_num,
                            uint32_t frag_len,
                            std::vector<uint32_t> tmo_vec, 
                            OnSendFrag on_send,
                            OnSessDone on_done) {

  size_t htable_size = sess_num / 64 + 1000;
  if (!hash_table_.InitForWrite("", htable_size)) {
    ELOG("TxSessionManager hash-table init failed! size:%lu", htable_size);
    return false;
  }
  frag_len_ = frag_len;
  tmo_vec_ = std::move(tmo_vec);
  timerw_ = nullptr;
  send_frag_ = std::move(on_send);
  sess_done_ = std::move(on_done);
  return true;
}

bool TxSessionManager::AddSessionConst(const TxRequest* req) {
  uint32_t seq = req->peer->next_seq();
  uint32_t base_seq = seq & ~0x3fU;
  uint32_t seq_index = seq & 0x3fU;
  NodeKey key = {req->ip, req->port, base_seq};

  // first check cached node
  Node* node = req->peer->txs_add_cache()->node;
  if (node && req->peer->txs_add_cache()->base_seq == base_seq
           && node->key == key) {
    // cache hit
    if (node->inuse_bitmap) {
      DLOG("tx-session add node cache hit (found)");
    } else {
      DLOG("tx-session add node cache hit (reuse)");
    }
  } else {
    // cache miss, so hash search
    DLOG("tx-session add node cache miss");
    bool is_found;
    node = (Node*)hash_table_.FindOrAlloc(key, &is_found);
    if (!node) {
      // overflow
      WLOG("TxSessionManager hash-table overflow!");
      return false;
    } else if (!is_found) {
      // new hash-table node
      node->key = key;
    }
    // update node cache
    req->peer->txs_add_cache()->node = node;
    req->peer->txs_add_cache()->base_seq = base_seq;
  }
  // check dup
  if (node->inuse_bitmap & (1UL << seq_index)) {
    WLOG("TxSessionManager dup key found!");
    return false;
  }
  // check data size
  size_t data_len = req->size - sizeof(TxRequest);
  if (static_cast<int64_t>(data_len) <= 0) {
    WLOG("TxSessionManager invalid data len:%lu!", data_len);
    return false;
  }
  // fill Session
  Session* sess = &node->sessions[seq_index];
  bool reusable = (sess->req != nullptr);
  sess->req = req;
  assert(sess->frag_bitmap == nullptr);
  sess->retrans_count = 0;

  size_t frag_num = (data_len <= frag_len_ ? 
                       1 : (data_len + frag_len_ - 1) / frag_len_);
  if (frag_num >= 65536) {
    WLOG("data too large (%lu bytes, more than 65535 fragments)", data_len);
    return false;
  } else if (frag_num <= 1) {
    sess->frag_bitmap = nullptr;
  } else {
    size_t bitmap_len = (frag_num + 7) / 8;
    sess->frag_bitmap = new uint8_t[bitmap_len];
    memset(sess->frag_bitmap, 0xff, bitmap_len);
    sess->frag_bitmap[bitmap_len-1] = (0xffU >> (bitmap_len*8-frag_num));
  }
  sess->frag_num = frag_num;
  // set session inuse
  node->inuse_bitmap |= (1UL << seq_index);
  // set timer
  if (!reusable) {
    new (&sess->timer_owner) ccb::TimerOwner();
    timerw()->AddTimer(tmo_vec_[sess->retrans_count], 
                       ccb::BindClosure(this,
                                        &TxSessionManager::OnRetransTimer,
                                        node, seq_index),
                       &sess->timer_owner);
  } else {
    // reuse timer objects allocated before for performance
    timerw()->ResetTimer(sess->timer_owner, tmo_vec_[sess->retrans_count]);
  }
  // send all fragments
  size_t left = data_len;
  for (size_t i = 0; i < frag_num; i++) {
    void*  pkt = (void*)((uint8_t*)req->data + frag_len_ * i);
    size_t len = (left < frag_len_ ? left : frag_len_); 
    send_frag_(const_cast<TxRequest*>(sess->req), {pkt,len}, seq, frag_num, i);
    left -= len;
  }

  return true;
}

bool TxSessionManager::AckSession(Peer* peer, uint32_t seq,
                                  uint16_t frag_count, uint16_t frag_index) {
  uint32_t base_seq = seq & ~0x3fU;
  uint32_t seq_index = seq & 0x3fU;
  NodeKey key = {peer->addr().ip(), peer->addr().port(), base_seq};

  // first check cached node
  Node* node = peer->txs_ack_cache()->node;
  if (node && peer->txs_ack_cache()->base_seq == base_seq
           && node->inuse_bitmap && node->key == key) {
    // cache hit
    DLOG("tx-session ack node cache hit");
  } else {
    // cache miss, so hash search
    DLOG("tx-session ack node cache miss");
    node = (Node*)hash_table_.Find(key);
    if (!node) {
      // hash-table node not found
      DLOG("AckSession hash-table not found seq:%u", seq);
      return false;
    }
    // update node cache
    peer->txs_ack_cache()->node = node;
    peer->txs_ack_cache()->base_seq = base_seq;
  }
  // check session valid
  if (!(node->inuse_bitmap & (1UL << seq_index))) {
    // session not found
    DLOG("AckSession session not found seq:%u", seq);
    return false;
  }
  Session* sess = &node->sessions[seq_index];
  size_t frag_num = sess->frag_num;
  if (frag_count != frag_num || frag_index >= frag_num) {
    // bad fragment index
    ILOG("AckSession bad param frag_count:%hu frag_index:%hu", 
                                      frag_count, frag_index);
    return false;
  }
  if (!sess->frag_bitmap) {
    if (frag_index == 0) {
      // single fragment packet acked
      sess_done_(const_cast<TxRequest*>(sess->req), R_SUCCESS);
      DelSession(node, seq_index);
    }
  } else {
    sess->frag_bitmap[frag_index / 8] &= ~(1UL << (frag_index % 8));
    // check if all fragments acked
    bool all_frags_acked = true;
    for (size_t i = 0; i < frag_num; i += 8) {
      if (sess->frag_bitmap[i / 8]) {
        all_frags_acked = false;
        break;
      }
    }
    if (all_frags_acked) {
      sess_done_(const_cast<TxRequest*>(sess->req), R_SUCCESS);
      DelSession(node, seq_index);
    }
  }
  return true;
}

void TxSessionManager::DelSession(Node* node, uint32_t seq_index) {
  Session* sess = &node->sessions[seq_index];
  sess->timer_owner.Cancel();
  if (sess->frag_bitmap) {
    delete[] sess->frag_bitmap;
    sess->frag_bitmap = nullptr;
  }
  // clear inuse bit, if inuse_bitmap reach 0 the node is deleted
  node->inuse_bitmap &= ~(1UL << seq_index);
}

void TxSessionManager::OnRetransTimer(Node* node, uint32_t seq_index) {
  assert(node && (node->inuse_bitmap & (1UL << seq_index)));
  Session* sess = &node->sessions[seq_index];
  uint32_t seq = node->key.base_seq + seq_index;
  size_t data_len = sess->req->size - sizeof(TxRequest);
  sess->retrans_count++;
  if (sess->retrans_count < tmo_vec_.size()) {
    // retransmit
    if (!sess->frag_bitmap) {
      // single fragment
      send_frag_(const_cast<TxRequest*>(sess->req),
                 {sess->req->data, data_len}, seq, 1, 0);
      ILOG("restrans DATA len:%u id:(-, -, %u, 1, 0)", data_len, seq);
    } else {
      // multiple fragments
      size_t frag_num = (data_len + frag_len_ - 1) / frag_len_;
      size_t send_count = 0;
      for (size_t i = 0; i < frag_num; ) {
        if ((i % 8) == 0 && sess->frag_bitmap[i / 8] == 0) {
          i += 8;
          continue;
        }
        if (sess->frag_bitmap[i / 8] & (1UL << (i % 8))) {
          void*  pkt = (void*)((uint8_t*)sess->req->data + frag_len_ * i);
          size_t left = (uint8_t*)sess->req->data + data_len - (uint8_t*)pkt;
          size_t len = (left < frag_len_ ? left : frag_len_); 
          send_frag_(const_cast<TxRequest*>(sess->req),
                     {pkt, len}, seq, frag_num, i);
          ILOG("restrans DATA len:%u id:(%u, %lu, %lu)",
                                  len, seq, frag_num, i);
          send_count++;
        }
        i++;
      }
      assert(send_count > 0);
    }
    // set timer
    timerw()->ResetTimer(sess->timer_owner, tmo_vec_[sess->retrans_count]);
  } else {
    // fail
    sess_done_(const_cast<TxRequest*>(sess->req), R_TIMEOUT);
    DelSession(node, seq_index);
  }
}

}  // namespace hudp

