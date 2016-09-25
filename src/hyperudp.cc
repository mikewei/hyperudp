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

  bool Init(const Addr& addr, OnRecv& on_recv);
  bool Init(uint16_t port, OnRecv& on_recv) {
    return Init({"0.0.0.0", port}, on_recv);
  }

  void Send(const Buf& buf, const Addr& addr, OnSent& done);

private:
  void OnUdpRecv(const Buf& buf, const Addr& addr);

private:
  Env env_;
  std::unique_ptr<UdpIO> udp_io_;
  std::vector<std::unique_ptr<HyperProto>> hyper_proto_;
  std::unique_ptr<ccb::WorkerGroup> workers_;
  OnRecv on_recv_;
};

HyperUdp::Impl::Impl(const Options& opt)
  : env_(opt)
  , udp_io_(GET_MODULE(UdpIO, env_.opt().udp_io_module, env_))
{
}

HyperUdp::Impl::~Impl()
{
}

bool HyperUdp::Impl::Init(const Addr& addr, OnRecv& on_recv)
{
  on_recv_ = std::move(on_recv);
  size_t worker_num = env_.opt().worker_num;
  // initialize HyperProto
  auto on_udp_send = [this](const Buf& buf, const Addr& peer) {
    udp_io_->Send(buf, peer);
  };
  auto on_usr_recv = [this](const Buf& buf, const Addr& peer) {
    if (on_recv_) on_recv_(buf, peer);
  };
  for (size_t i = 0; i < worker_num; i++) {
    hyper_proto_.emplace_back(new HyperProto(env_));
    if (!hyper_proto_[i]->Init(on_udp_send, on_usr_recv)) {
      return false;
    }
  }
  // initialize WorkerGroup
  workers_.reset(new ccb::WorkerGroup(worker_num, env_.opt().worker_queue_size));
  // initialize UdpIO
  udp_io_->Init(addr, [this](const Buf& buf, const Addr& peer) {
    this->OnUdpRecv(buf, peer);
  });
  return true;
}

void HyperUdp::Impl::OnUdpRecv(const Buf& buf, const Addr& addr)
{
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

void HyperUdp::Impl::Send(const Buf& buf, const Addr& addr, OnSent& done)
{
  size_t wid = Hash(addr, workers_->size());
  TxRequest* req = hyper_proto_[wid]->NewTxRequest(buf, addr, std::move(done));
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

bool HyperUdp::Init(uint16_t port, OnRecv on_recv)
{
  return pimpl_->Init(port, on_recv);
}

bool HyperUdp::Init(const Addr& addr, OnRecv on_recv)
{
  return pimpl_->Init(addr, on_recv);
}

void HyperUdp::Send(const Buf& buf, const Addr& addr, OnSent done)
{
  pimpl_->Send(buf, addr, done);
}

} // namespace hudp
