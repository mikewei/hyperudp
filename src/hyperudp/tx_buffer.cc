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
#include "hyperudp/tx_buffer.h"
#include "hyperudp/module_registry.h"

namespace hudp {

TxBuffer::TxBuffer(const Env& env, Peer* peer, const OnFlush& on_flush,
                   size_t flush_threshold)
  : env_(env)
  , peer_(peer)
  , on_flush_(on_flush)
  , timerw_(nullptr)
  , flush_threshold_(flush_threshold ? flush_threshold
                     : MaxSegmentsSize(env.opt().max_udp_pkt_size))
  , delay_algo_(GET_MODULE(TxDelayAlgo, env.opt().tx_delay_algo_module, env))
  , segs_count_(0)
  , segs_len_(0)
  , cur_ack_seg_(nullptr)
  , cur_frag_ack_seg_(nullptr)
{
  if (!timerw()->AddTimer(10, ccb::BindClosure(this, &TxBuffer::OnBufTimeout),
                                                     &timer_owner_)) {
    throw std::runtime_error("TxBuffer AddTimer failed!");
  };
}

TxBuffer::~TxBuffer()
{
}

void TxBuffer::AddData(uint32_t seq, uint16_t frag_count, uint16_t frag_index,
                       const Buf& buf, TxRequest* req)
{
  size_t seg_len = kMinDataSegSize + buf.len();
  CheckFlush(seg_len);
  SegDesc* seg = &segs_buf_[segs_count_];
  seg->type = SEG_DATA;
  seg->val.data.seq = seq;
  seg->val.data.frag_count = frag_count;
  seg->val.data.frag_index = frag_index;
  seg->val.data.data = buf.ptr();
  seg->val.data.len = buf.len();
  seg->val.data.req = req;
  segs_count_++;
  segs_len_ += seg_len;
  CheckFlush(kMinSegSize);

  auto action = delay_algo_->OnPendingSeg(segs_count_);
  if (action.type == TxDelayAlgo::kFlushPendings) {
    Flush();
  } else if (action.type == TxDelayAlgo::kResetFlushTimer) {
    timerw()->ResetTimer(timer_owner_, action.timeout);
  }
}

void TxBuffer::AddAck(uint32_t proc_sess_id, uint32_t seq, 
                      uint32_t frag_count, uint32_t frag_index)
{
  size_t seg_len;
  SegDesc* seg;
  if (frag_count == 1) {
    if (cur_ack_seg_ &&
        cur_ack_seg_->proc_sess_id == proc_sess_id &&
        cur_ack_seg_->base_seq <= seq && 
        cur_ack_seg_->base_seq + 32U > seq) {
      // merged into existed ACK segment
      cur_ack_seg_->seq_bitmap |= (0x1U << (seq - cur_ack_seg_->base_seq));
      return;
    }
    seg_len = kAckSegSize;
    CheckFlush(seg_len);
    seg = &segs_buf_[segs_count_];
    seg->type = SEG_ACK;
    seg->val.ack.proc_sess_id = proc_sess_id;
    seg->val.ack.base_seq = seq;
    seg->val.ack.seq_bitmap = 0x1;
    cur_ack_seg_ = &seg->val.ack;
  } else {
    if (cur_frag_ack_seg_ &&
        cur_frag_ack_seg_->proc_sess_id == proc_sess_id &&
        cur_frag_ack_seg_->seq == seq &&
        cur_frag_ack_seg_->frag_count == frag_count && 
        cur_frag_ack_seg_->frag_base_index <= frag_index && 
        cur_frag_ack_seg_->frag_base_index + 32U > frag_index) {
      // merged into existed FRAG-ACK segment
      cur_frag_ack_seg_->frag_bitmap |= 
        (0x1U << (frag_index - cur_frag_ack_seg_->frag_base_index));
      return;
    }
    seg_len = kFragAckSegSize;
    CheckFlush(seg_len);
    seg = &segs_buf_[segs_count_];
    seg->type = SEG_FRAG_ACK;
    seg->val.frag_ack.proc_sess_id = proc_sess_id;
    seg->val.frag_ack.seq = seq;
    seg->val.frag_ack.frag_count = frag_count;
    seg->val.frag_ack.frag_base_index = frag_index & ~0x1fU;
    seg->val.frag_ack.frag_bitmap = (0x1U << (frag_index & 0x1fU));
    cur_frag_ack_seg_ = &seg->val.frag_ack;
  }
  segs_count_++;
  segs_len_ += seg_len;
  CheckFlush(kMinSegSize);

  auto action = delay_algo_->OnPendingSeg(segs_count_);
  if (action.type == TxDelayAlgo::kFlushPendings) {
    Flush();
  } else if (action.type == TxDelayAlgo::kResetFlushTimer) {
    timerw()->ResetTimer(timer_owner_, action.timeout);
  }
}

void TxBuffer::CheckFlush(size_t new_seg_len)
{
  if (segs_len_ + new_seg_len > flush_threshold_) {
    Flush();
  } else if (segs_count_ >= kMaxBufferedSegs) {
    Flush();
  }
}

void TxBuffer::OnBufTimeout()
{
  if (segs_count_ > 0) {
    DLOG("flush tx-buffer as timeout");
    Flush();
  }
}

void TxBuffer::Flush()
{
  assert(segs_len_ > 0);
  on_flush_(peer_, segs_buf_, segs_count_);
  segs_count_ = 0;
  segs_len_ = 0;
  cur_ack_seg_ = nullptr;
  cur_frag_ack_seg_ = nullptr;
}

}
