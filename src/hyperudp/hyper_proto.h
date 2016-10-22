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
#ifndef _HUDP_HYPER_PROTO_H
#define _HUDP_HYPER_PROTO_H

#include "hyperudp/env.h"

namespace hudp {

class TxRequest;
class RxRequest;
class PeerManager;
class Peer;
class TxSessionManager;
class RxFragCache;
class RxDupCache;
class SegDesc;

class HyperProto
{
public:
  using OnUdpSend = ccb::ClosureFunc<void(const Buf&, const Addr&)>;
  using OnUsrRecv = HyperUdp::OnRecv;
  using OnUsrSent = HyperUdp::OnSent;
  using OnUsrCtxSent = HyperUdp::OnCtxSent;

  HyperProto(const Env& env);
  virtual ~HyperProto();

  bool Init(OnUdpSend on_send, OnUsrRecv on_recv, OnUsrCtxSent on_ctx_sent);
  void OnUdpRecv(const Buf& buf, const Addr& addr);
  void OnUsrSend(const Buf& buf, const Addr& addr, void* ctx, OnUsrSent done);

  TxRequest* NewTxRequest(const Buf& buf, const Addr& addr,
                          void* ctx, OnUsrSent done);
  void StartTxRequest(TxRequest* req);
  void DelTxRequest(TxRequest* req, bool done = true);

  RxRequest* NewRxRequest(const Buf& buf, const Addr& addr);
  void StartRxRequest(RxRequest* req);
  void DelRxRequest(RxRequest* req);

private:
  // to be deleted
  void OnTxSessionsSendFrag(const Buf& buf,
                            const Addr& addr,
                            uint32_t seq,
                            uint16_t frag_count,
                            uint16_t frag_index,
                            void* ctx);
  // to be deleted
  void OnSessDone(Result res, void* ctx);
  void OnTxSessSendFrag(TxRequest* req,
                        const Buf& buf,
                        uint32_t seq,
                        uint16_t frag_count,
                        uint16_t frag_index);
  void OnTxSessDone(TxRequest* req, Result res);
  void ParseRxPacket(const Buf& buf, const Addr& addr, RxRequest* req);
  void SendAck(RxRequest* req, uint32_t proc_sess_id, uint32_t seq, 
               uint16_t frag_count, uint16_t frag_index);
  void OnRxFragCacheComplete(const Addr& addr,
                             uint32_t proc_sess_id,
                             uint32_t seq,
                             uint16_t frag_count,
                             void** frag_list,
                             Result result);
  void OnFlushTxBuffer(Peer* peer, const SegDesc* segs, size_t count);

private:
  const Env& env_;
  std::unique_ptr<PeerManager> peer_mgr_;
  std::unique_ptr<TxSessionManager> tx_sess_mgr_;
  std::unique_ptr<RxFragCache> frag_cache_;
  std::unique_ptr<RxDupCache> rx_dup_cache_;
  OnUdpSend on_udp_send_;
  OnUsrRecv on_usr_recv_;
  OnUsrCtxSent on_usr_ctx_sent_;
  uint32_t proc_sess_id_;
};

}

#endif // _HUDP_HYPER_PROTO_H
