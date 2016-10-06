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
#ifndef _HUDP_TX_BUFFER_H
#define _HUDP_TX_BUFFER_H

#include "ccbase/timer_wheel.h"
#include "ccbase/worker_group.h"
#include "hyperudp/addr.h"
#include "hyperudp/env.h"
#include "hyperudp/protocol.h"
#include "hyperudp/tx_delay_algo.h"

namespace hudp {

class Peer;
class TxRequest;

struct DataSegDesc
{
  uint32_t seq;
  uint16_t frag_count;
  uint16_t frag_index;
  const void* data;
  size_t len;
  TxRequest* req;
};

struct AckSegDesc
{
  uint32_t proc_sess_id;
  uint32_t base_seq;
  uint32_t seq_bitmap;
};

struct FragAckSegDesc
{
  uint32_t proc_sess_id;
  uint32_t seq;
  uint16_t frag_count;
  uint16_t frag_base_index;
  uint32_t frag_bitmap;
};

struct SegDesc
{
  SegmentType type;
  union {
    DataSegDesc data;
    AckSegDesc ack;
    FragAckSegDesc frag_ack;
  } val;
};

class TxBuffer
{
public:
  using OnFlush = ccb::ClosureFunc<void(Peer*, const SegDesc*, size_t)>;

  TxBuffer(const Env& env, Peer* peer, const OnFlush& on_flush,
           size_t flush_threshold = 0);
  ~TxBuffer();

  void AddData(uint32_t seq, uint16_t frag_count, uint16_t frag_index,
               const Buf& buf, TxRequest* req);
  void AddAck(uint32_t proc_sess_id, uint32_t seq, 
              uint32_t frag_count, uint32_t frag_index);

private:
  ccb::TimerWheel* timerw() {
    if (!timerw_) {
      timerw_ = env_.timerw();
    }
    return timerw_;
  }
  void CheckFlush(size_t new_seg_len);
  void OnBufTimeout();
  void Flush();

private:
  const Env& env_;
  Peer* peer_;
  OnFlush on_flush_;
  ccb::TimerWheel* timerw_;
  ccb::TimerOwner timer_owner_;
  size_t flush_threshold_;
  std::unique_ptr<TxDelayAlgo> delay_algo_;
  SegDesc segs_buf_[kMaxBufferedSegs];
  size_t segs_count_;
  size_t segs_len_;
  AckSegDesc* cur_ack_seg_;
  FragAckSegDesc* cur_frag_ack_seg_;
};

}

#endif // _HUDP_TX_BUFFER_H
