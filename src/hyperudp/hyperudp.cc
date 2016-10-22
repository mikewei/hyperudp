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
#include <sys/time.h>
#include <assert.h>
#include "ccbase/worker_group.h"
#include "hyperudp/hyperudp.h"
#include "hyperudp/options.h"
#include "hyperudp/env.h"
#include "hyperudp/udp_io.h"
#include "hyperudp/chunk_alloc.h"
#include "hyperudp/module_registry.h"
#include "hyperudp/hyper_proto.h"
#include "hyperudp/module_registrars.h"

namespace hudp {

static size_t Hash(const Addr& addr, size_t range)
{
  return std::hash<Addr>()(addr) % range;
}

class HyperUdp::Impl
{
public:
  using OnSent = HyperUdp::OnSent;
  using OnRecv = HyperUdp::OnRecv;

  Impl(const Options& opt);
  ~Impl();

  bool Init(const Addr& addr, OnRecv on_recv, OnCtxSent on_ctx_sent);

  void Send(const Buf& buf, const Addr& addr, void* ctx, OnSent done);

  ccb::WorkerGroup* worker_group() const {
    return workers_.get();
  }

private:
  void OnUdpRecv(const Buf& buf, const Addr& addr);

private:
  Env env_;
  std::unique_ptr<UdpIO> udp_io_;
  std::vector<std::unique_ptr<HyperProto>> hyper_proto_;
  std::unique_ptr<ccb::WorkerGroup> workers_;
  OnRecv on_recv_;
  OnCtxSent on_ctx_sent_;
  bool is_initialized_;
};

HyperUdp::Impl::Impl(const Options& opt)
  : env_(opt)
  , udp_io_(HUDP_MODULE(UdpIO, env_.opt().udp_io_module, env_))
  , is_initialized_(false)
{
}

HyperUdp::Impl::~Impl()
{
}

bool HyperUdp::Impl::Init(const Addr& addr, OnRecv on_recv,
                          OnCtxSent on_ctx_sent)
{
  assert(!is_initialized_);
  on_recv_ = std::move(on_recv);
  on_ctx_sent_ = std::move(on_ctx_sent);
  size_t worker_num = env_.opt().worker_num;
  // initialize HyperProto
  HyperProto::OnUdpSend on_udp_send {
    [this](const Buf& buf, const Addr& peer) {
      if (!udp_io_->Send(buf, peer)) {
        WLOG("UdpIO::Send failed!");
      }
    }
  };
  for (size_t i = 0; i < worker_num; i++) {
    hyper_proto_.emplace_back(new HyperProto(env_));
    if (!hyper_proto_[i]->Init(on_udp_send, on_recv_, on_ctx_sent_)) {
      hyper_proto_.clear();
      return false;
    }
  }
  // initialize WorkerGroup
  workers_.reset(new ccb::WorkerGroup(worker_num,
                                      env_.opt().worker_queue_size));
  // initialize UdpIO
  if (!udp_io_->Init(addr, ccb::BindClosure(this,
                                            &HyperUdp::Impl::OnUdpRecv))) {
    // rollback
    workers_.reset();
    hyper_proto_.clear();
    return false;
  }
  is_initialized_ = true;
  return true;
}

void HyperUdp::Impl::OnUdpRecv(const Buf& buf, const Addr& addr)
{
  assert(is_initialized_);
  size_t wid = Hash(addr, workers_->size());
  RxRequest* req = hyper_proto_[wid]->NewRxRequest(buf, addr);
  if (!req) {
    WLOG("NewRxRequest failed");
    return;
  }
  if (!workers_->PostTask(wid, [this, req]{
    size_t id = ccb::Worker::self()->id();
    hyper_proto_[id]->StartRxRequest(req);
  })) {
    ILOG("OnUdpRecv: worker-queue overflow!");
    hyper_proto_[wid]->DelRxRequest(req);
  }
}

void HyperUdp::Impl::Send(const Buf& buf, const Addr& addr,
                          void* ctx, OnSent done)
{
  assert(is_initialized_);
  size_t wid = Hash(addr, workers_->size());
  TxRequest* req = hyper_proto_[wid]->NewTxRequest(buf, addr, ctx,
                                                   std::move(done));
  if (!req) {
    WLOG("NewTxRequest failed");
    return;
  }
  if (workers_->is_current_thread(wid)) {
    hyper_proto_[wid]->StartTxRequest(req);
  } else if (!workers_->PostTask(wid, [this, req] {
    size_t id = ccb::Worker::self()->id();
    hyper_proto_[id]->StartTxRequest(req);
  })) {
    // PostTask failed
    ILOG("Send: worker-queue overflow!");
    hyper_proto_[wid]->DelTxRequest(req, false);
  }
}

HyperUdp::HyperUdp()
  : HyperUdp(OptionsBuilder().Build())
{
}

HyperUdp::HyperUdp(const Options& opt)
  : pimpl_(new HyperUdp::Impl(opt))
{
}

HyperUdp::~HyperUdp()
{
}

bool HyperUdp::Init(const Addr& addr, OnRecv on_recv)
{
  return pimpl_->Init(addr, std::move(on_recv), nullptr);
}

bool HyperUdp::Init(const Addr& addr, OnRecv on_recv, OnCtxSent on_ctx_sent)
{
  return pimpl_->Init(addr, std::move(on_recv), std::move(on_ctx_sent));
}

void HyperUdp::Send(const Buf& buf, const Addr& addr)
{
  pimpl_->Send(buf, addr, nullptr, nullptr);
}

void HyperUdp::Send(const Buf& buf, const Addr& addr, OnSent done)
{
  pimpl_->Send(buf, addr, nullptr, std::move(done));
}

void HyperUdp::Send(const Buf& buf, const Addr& addr, void* ctx)
{
  pimpl_->Send(buf, addr, ctx, nullptr);
}

ccb::WorkerGroup* HyperUdp::GetWorkerGroup() const
{
  return pimpl_->worker_group();
}

} // namespace hudp
