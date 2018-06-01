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
#ifndef HUDP_TB_MAX_TX_DELAY_ALGO_H_
#define HUDP_TB_MAX_TX_DELAY_ALGO_H_

#include "ccbase/token_bucket.h"
#include "ccbase/timer_wheel.h"
#include "hyperudp/max_tx_delay_algo.h"
#include "hyperudp/env.h"

namespace hudp {

class TbMaxTxDelayAlgo : public MaxTxDelayAlgo {
 public:
  TbMaxTxDelayAlgo(const Env& env)
    : MaxTxDelayAlgo(env)
    , tick_(env.timerw()->GetCurrentTick())
    , tb_(1000/env.opt().max_tx_delay, // qps limit
          100/env.opt().max_tx_delay,  // bucket size
          100/env.opt().max_tx_delay,  // init size
          GetTimeNow(),
          false) {}

  virtual Action OnPendingSeg(size_t seg_count) override {
    if (seg_count == 0) {
      // if seg_count == 0, all flushed, so do nothing
      return {kNoOp, 0};
    }
    UpdateTickAndTB();
    // check token-bucket limit
    if (tb_.Get(1)) {
      return {kFlushPendings, 0};
    }
    // tx_max_delay algo default process
    return MaxTxDelayAlgo::OnPendingSeg(seg_count);
  }

 protected:
  void UpdateTickAndTB() {
    auto tick = env_.timerw()->GetCurrentTick();
    if (tick > tick_) {
      tick_ = tick;
      tb_.Gen(GetTimeNow());
    }
  }
  const struct timeval* GetTimeNow() {
    tv_.tv_sec = tick_/1000;
    tv_.tv_usec = (tick_%1000)*1000;
    return &tv_;
  }

  ccb::tick_t tick_;
  ccb::TokenBucket tb_;
  struct timeval tv_;
};

}  // namespace hudp

#endif  // HUDP_MAX_TX_DELAY_ALGO_H_
