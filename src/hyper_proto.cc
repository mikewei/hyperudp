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
#include <arpa/inet.h>
#include "hyperudp/hyper_proto.h"
#include "hyperudp/protocol.h"
#include "hyperudp/peer_manager.h"
#include "hyperudp/tx_session_manager.h"
#include "hyperudp/tx_sessions.h"
#include "hyperudp/frag_cache.h"
#include "hyperudp/rx_dup_cache.h"

namespace hudp {

struct RxRequest
{
  uint32_t size;
  Addr addr;
  Buf frag_buf;
  uint32_t ref_count;
  Peer* peer;
  uint8_t data[0];
};

HyperProto::HyperProto(const Env& env)
  : env_(env)
  , tx_sess_(new TxSessions(env))
  , tx_sess_mgr_(new TxSessionManager(env))
  , frag_cache_(new FragCache(env))
  , rx_dup_cache_(new RxDupCache(env))
  , proc_sess_id_(env_.Rand())
{
}

HyperProto::~HyperProto()
{
}

bool HyperProto::Init(OnUdpSend on_send, OnUsrRecv on_recv)
{
  using ccb::BindClosure;
  const Options& opt = env_.opt();
  // init TxSessions
  if (!tx_sess_->Init(opt.max_tx_sessions / opt.worker_num, 
                      FragDataSize(opt.max_udp_pkt_size), 
                      opt.retrans_timeouts,
                      BindClosure(this, &HyperProto::OnTxSessionsSendFrag),
                      BindClosure(this, &HyperProto::OnSessDone))) {
    ELOG("HyperProto: init TxSessions failed!");
    return false;
  }
  // init TxSessionManager
  if (!tx_sess_mgr_->Init(opt.max_tx_sessions / opt.worker_num, 
                          FragDataSize(opt.max_udp_pkt_size), 
                          opt.retrans_timeouts,
                          BindClosure(this, &HyperProto::OnTxSessSendFrag),
                          BindClosure(this, &HyperProto::OnTxSessDone))) {
    ELOG("HyperProto: init TxSessionManager failed!");
    return false;
  }
  // init FragCache
  if (!frag_cache_->Init(opt.max_frag_cache_nodes / opt.worker_num,
                         opt.frag_cache_timeout,
                         BindClosure(this, &HyperProto::OnFragCacheComplete)))
  {
    ELOG("HyperProto: init FragCache failed!");
    return false;
  }
  // init RxDupCache
  if (!rx_dup_cache_->Init(opt.enable_rx_dup_cache,
                           opt.rx_dup_cache_size / opt.worker_num / 64 + 1000,
                           opt.rx_dup_cache_timeout)) {
    ELOG("HyperProto: init RxDupCache failed!");
    return false;
  }

  on_udp_send_ = std::move(on_send);
  on_usr_recv_ = std::move(on_recv);
  // init PeerManager
  peer_mgr_.reset(new PeerManager(env_, 
                      BindClosure(this, &HyperProto::OnFlushTxBuffer)));

  return true;
}

void HyperProto::OnUdpRecv(const Buf& buf, const Addr& addr)
{
  //on_usr_recv_(buf, addr);
  RxRequest* req = NewRxRequest(buf, addr);
  if (!req) {
    WLOG("OnUdpRecv: NewRxRequest failed!");
    return;
  }
  StartRxRequest(req);
}

void HyperProto::OnUsrSend(const Buf& buf, const Addr& addr, OnUsrSent done)
{
  //on_udp_send_(buf, addr);
  TxRequest* req = NewTxRequest(buf, addr, std::move(done));
  if (!req) {
    WLOG("NewTxRequest failed!");
    return;
  }
  StartTxRequest(req);
}

TxRequest* HyperProto::NewTxRequest(const Buf& buf, const Addr& addr, 
                                    OnUsrSent done)
{
  size_t req_size = sizeof(TxRequest) + buf.len();
  TxRequest* req = static_cast<TxRequest*>(env_.alloc().Alloc(req_size));
  if (!req) {
    if (done) done(R_ERROR);
    return nullptr;
  }
  req->size = req_size;
  req->ip = addr.ip();
  req->port = addr.port();
  req->ref_count = 0;
  new (&req->on_sent) OnUsrSent(std::move(done));
  memcpy(req->data, buf.ptr(), buf.len());
  DLOG("NewTxRequest data_len:%lu", buf.len());
  return req;
}

void HyperProto::StartTxRequest(TxRequest* req)
{
  assert(req && req->port && !req->ref_count && 
         req->size > sizeof(TxRequest));
  req->ref_count++;
  req->peer = peer_mgr_->GetPeer({req->ip, req->port});
  /*
  if (!tx_sess_->AddSession({req->data, req->size - sizeof(TxRequest)},
                            {req->ip, req->port}, 
                            req->peer->next_seq(), (void*)req)) {
    if (req->on_sent) req->on_sent(R_ERROR);
    if (!--req->ref_count) DelTxRequest(req);
    WLOG("AddSession failed!");
    return;
  }*/
  if (!tx_sess_mgr_->AddSession(req)) {
    if (req->on_sent) req->on_sent(R_ERROR);
    if (!--req->ref_count) DelTxRequest(req);
    WLOG("AddSession failed!");
    return;
  }
}

void HyperProto::DelTxRequest(TxRequest* req, bool done)
{
  if (!done && req->on_sent) req->on_sent(R_ERROR);
  req->on_sent.~OnUsrSent();
  env_.alloc().Free(req, req->size);
}

RxRequest* HyperProto::NewRxRequest(const Buf& buf, const Addr& addr)
{
  size_t req_size = sizeof(RxRequest) + buf.len();
  RxRequest* req = static_cast<RxRequest*>(env_.alloc().Alloc(req_size));
  if (req) {
    req->size = req_size;
    new (&req->addr) Addr(addr);
    req->ref_count = 0;
    memcpy(req->data, buf.ptr(), buf.len());
  }
  return req;
}

void HyperProto::StartRxRequest(RxRequest* req)
{
  assert(req && req->size > sizeof(RxRequest) && !req->ref_count);
  req->ref_count++; // acquire local reference
  req->peer = peer_mgr_->GetPeer(req->addr);
  const Buf buf{req->data, req->size - sizeof(RxRequest)};
  ParseRxPacket(buf, req->addr, req);
  if (!--req->ref_count) DelRxRequest(req); // release local reference
}

void HyperProto::DelRxRequest(RxRequest* req)
{
  env_.alloc().Free(req, req->size);
}

// to be deleted
void HyperProto::OnTxSessionsSendFrag(const Buf& buf,
                                      const Addr& addr,
                                      uint32_t seq,
                                      uint16_t frag_count,
                                      uint16_t frag_index,
                                      void* ctx)
{
  TxRequest* req = static_cast<TxRequest*>(ctx);
  req->ref_count++;
  req->peer->tx_buffer()->AddData(seq, frag_count, frag_index, buf, req);
  DLOG("TxSessionSendFrag len:%lu id:(-, -, %u, %hu, %hu)", 
                                 buf.len(), seq, frag_count, frag_index);
}

// to be deleted
void HyperProto::OnSessDone(Result res, void* ctx)
{
  assert(ctx);
  TxRequest* req = static_cast<TxRequest*>(ctx);
  if (req->on_sent) req->on_sent(res);
  if (!--req->ref_count) {
    DelTxRequest(req);
  }
}

void HyperProto::OnTxSessSendFrag(TxRequest* req,
                                  const Buf& buf,
                                  uint32_t seq,
                                  uint16_t frag_count,
                                  uint16_t frag_index)
{
  req->ref_count++;
  req->peer->tx_buffer()->AddData(seq, frag_count, frag_index, buf, req);
  DLOG("TxSessionSendFrag len:%lu id:(-, -, %u, %hu, %hu)", 
                                 buf.len(), seq, frag_count, frag_index);
}

void HyperProto::OnTxSessDone(TxRequest* req, Result res)
{
  if (req->on_sent) req->on_sent(res);
  if (!--req->ref_count) {
    DelTxRequest(req);
  }
}

void HyperProto::ParseRxPacket(const Buf& buf, const Addr& addr, 
                               RxRequest* req)
{
  const uint8_t* ptr = static_cast<const uint8_t*>(buf.ptr());
  size_t left = buf.len();

  DLOG("ParseRxPacket len:%lu", buf.len());
  if (left <= kPktHeaderSize) WRET("bad pakcet len #1");
  PacketHeader* pkt_hdr = (PacketHeader*)ptr;
  if (pkt_hdr->hudp_tag != 'H') WRET("bad tag");
  if (pkt_hdr->hudp_ver != 1) WRET("bad ver");
  size_t pkt_len = ntohs(pkt_hdr->pkt_len);
  if (pkt_len != buf.len()) WRET("bad pakcet len #2");
  uint32_t proc_sess_id = ntohl(pkt_hdr->proc_sess_id);
  ptr += kPktHeaderSize;
  left -= kPktHeaderSize;

  bool first_frag = true;
  while (left > 0) {
    if (left < kSegHeaderSize) WRET("bad pakcet len #3");
    SegmentHeader* seg_hdr = (SegmentHeader*)ptr;
    uint16_t seg_type = ntohs(seg_hdr->seg_type);
    size_t seg_len = ntohs(seg_hdr->seg_len);

    // DATA segment
    if (seg_type == SEG_DATA) {
      if (seg_len <= kMinDataSegSize) 
        WRET("bad data seg len");
      if (left < seg_len) 
        WRET("bad packet len #4");
      DataSegment* data_seg = (DataSegment*)seg_hdr;
      uint32_t seq = ntohl(data_seg->seq);
      uint16_t frag_count = ntohs(data_seg->frag_count);
      uint16_t frag_index = ntohs(data_seg->frag_index);
      if (frag_index >= frag_count) {
        WRET("bad frag_index");
      }
      size_t data_len = seg_len - kMinDataSegSize;
      DLOG("parse DATA seg len:%lu id:(-, %u, %u, %hu, %hu)",
            data_len, proc_sess_id, seq, frag_count, frag_index);
      SendAck(req, proc_sess_id, seq, frag_count, frag_index);
      if (frag_count == 1) {
        // single-frag DATA segment
        if (!rx_dup_cache_->CheckDup(req->peer, proc_sess_id, seq)) {
          on_usr_recv_({data_seg->data, data_len}, addr);
        } else {
          ILOG("dup DATA seg len:%lu id:(-, %u, %u, 1, 0)",
                              data_len, proc_sess_id, seq);
        }
        // done
      } else {
        // multi-frag DATA segment
        RxRequest* frag_req = req;
        if (first_frag) {
          first_frag = false;
        } else {
          DLOG("second multi-frag len:%lu", data_len);
          // clone RxRequest
          frag_req = NewRxRequest(buf, addr);
          if (!frag_req) {
            WLOG("ParseRxPacket: NewRxRequest failed!");
          }
          frag_req->peer = req->peer;
        }
        frag_req->ref_count++; // acquire reference for FragCache
        new (&frag_req->frag_buf) Buf(data_seg->data, data_len);
        if (!frag_cache_->AddFrag(addr, proc_sess_id, seq, frag_count, 
                                  frag_index, (void*)frag_req)) {
          // release reference for FragCache
          if (!--frag_req->ref_count) DelRxRequest(frag_req);
          ILOG("ParseRxPacket: AddFrag failed!");
        }
      }

    // ACK segment
    } else if (seg_type == SEG_ACK) {
      if (seg_len < kAckSegSize) 
        WRET("bad ack seg len");
      if (left < seg_len) 
        WRET("bad packet len #5");
      AckSegment* ack_seg = (AckSegment*)seg_hdr;
      uint32_t ack_proc_sess_id = ntohl(ack_seg->proc_sess_id);
      uint32_t base_seq = ntohl(ack_seg->base_seq);
      uint32_t seq_bitmap = ntohl(ack_seg->seq_bitmap);
      if (ack_proc_sess_id != proc_sess_id_) {
        IRET("bad proc_sess_id (e.g. invalid old ack)");
      }
      DLOG("parse ACK seg ids:(-, %u, %u[0x%x])",
            ack_proc_sess_id, base_seq, seq_bitmap);

      uint32_t failed_seqs = 0;
      for (size_t i = 0; i < 32; ) {
        if ((i % 8) == 0 && !(seq_bitmap & (0xffUL << i))) {
          i += 8;
          continue;
        }
        if (seq_bitmap & (0x1UL << i)) {
          /*if (!tx_sess_->AckSession(addr, base_seq + i, 1, 0)) {*/
          if (!tx_sess_mgr_->AckSession(req->peer, base_seq + i, 1, 0)) {
            failed_seqs |= (0x1UL << i);
          }
        }
        i++;
      }
      if (failed_seqs) {
        ILOG("ACK AckSession failed ids:(-, %u, %u[0x%x])", 
              ack_proc_sess_id, base_seq, failed_seqs);
      }

    // Frag-ACK segment
    } else if (seg_type == SEG_FRAG_ACK) {
      if (seg_len < kFragAckSegSize) 
        WRET("bad frag ack seg len");
      if (left < seg_len) 
        WRET("bad packet len #6");
      FragAckSegment* ack_seg = (FragAckSegment*)seg_hdr;
      uint32_t ack_proc_sess_id = ntohl(ack_seg->proc_sess_id);
      uint32_t seq = ntohl(ack_seg->seq);
      uint16_t frag_count = ntohs(ack_seg->frag_count);
      uint16_t frag_base_index = ntohs(ack_seg->frag_base_index);
      uint32_t frag_bitmap = ntohl(ack_seg->frag_bitmap);
      if (ack_proc_sess_id != proc_sess_id_) {
        IRET("bad proc_sess_id (e.g. invalid old ack)");
      }
      DLOG("parse FRAG-ACK seg id:(-, %u, %u, %hu, %hu[0x%x])",
            ack_proc_sess_id, seq, frag_count, frag_base_index, frag_bitmap);

      uint32_t failed_seqs = 0;
      for (size_t i = 0; i < 32;) {
        if ((i % 8) == 0 && !(frag_bitmap & (0xffUL << i))) {
          i += 8;
          continue;
        }
        if (frag_bitmap & (0x1UL << i)) {
          size_t frag_index = (size_t)frag_base_index + i;
          if (frag_index >= frag_count) {
            IRET("bad frag_index (larger than frag_count");
          }
          /*if (!tx_sess_->AckSession(addr, seq, frag_count, */
          if (!tx_sess_mgr_->AckSession(req->peer, seq, frag_count,
                                              (uint16_t)frag_index)) {
            failed_seqs |= (0x1UL << i);
          }
        }
        i++;
      }
      if (failed_seqs) {
        ILOG("FRAG-ACK AckSession failed id:(-, %u, %u, %hu, %hu[0x%x])", 
              ack_proc_sess_id, seq, frag_count, frag_base_index, failed_seqs);
      }

    } else {
        WRET("unknow segment type %d", seg_type);
    }
    ptr += seg_len;
    left -= seg_len;
  }
}

void HyperProto::SendAck(RxRequest* req,
                         uint32_t proc_sess_id, 
                         uint32_t seq,
                         uint16_t frag_count,
                         uint16_t frag_index)
{
  req->peer->tx_buffer()->AddAck(proc_sess_id, seq, frag_count, frag_index);
  DLOG("SendAck id:(-, %u, %u, %hu, %hu)", 
                                 proc_sess_id, seq, frag_count, frag_index);
}

void HyperProto::OnFragCacheComplete(const Addr& addr, 
                                     uint32_t proc_sess_id,
                                     uint32_t seq, 
                                     uint16_t frag_count, 
                                     void** frag_list,
                                     Result result)
{
  Peer* peer = nullptr;
  std::string pkt;
  if (result == R_SUCCESS) {
    pkt.reserve(1500 * frag_count);
  }
  for (size_t i = 0; i < frag_count; i++) {
    RxRequest* req = static_cast<RxRequest*>(frag_list[i]);
    if (!req) continue;
    if (!peer) peer = req->peer;
    if (result == R_SUCCESS) {
      pkt.append(req->frag_buf.char_ptr(), req->frag_buf.len());
    }
    if (!--req->ref_count) DelRxRequest(req); // release ref by FragCache
  }
  if (result == R_SUCCESS) {
    if (!rx_dup_cache_->CheckDup(peer, proc_sess_id, seq)) {
      DLOG("recv complete multi-frags DATA seg len:%lu id:(-, %u, %u, %hu, -)",
                                    pkt.size(), proc_sess_id, seq, frag_count);
      on_usr_recv_({pkt.data(), pkt.size()}, addr);
    } else {
      ILOG("dup multi-frags DATA seg len:%lu id:(-, %u, %u, %hu, -)",
                          pkt.size(), proc_sess_id, seq, frag_count);
    }
  } else {
    ILOG("recv timeout multi-frags DATA seg len:%lu id:(-, %u, %u, %hu, -)",
                                 pkt.size(), proc_sess_id, seq, frag_count);
  }
}

void HyperProto::OnFlushTxBuffer(Peer* peer,
                                 const SegDesc* segs,
                                 size_t count)
{
  static thread_local uint8_t buffer[kMaxUdpPktSize];
  uint8_t* ptr = buffer;
  size_t left = sizeof(buffer);

  PacketHeader* pkt_hdr = (PacketHeader*)ptr;
  pkt_hdr->hudp_tag = 'H';
  pkt_hdr->hudp_ver = 1;
  pkt_hdr->pkt_len = 0;
  pkt_hdr->from_port = 0;
  pkt_hdr->to_port = 0;
  pkt_hdr->from_ip = 0;
  pkt_hdr->to_ip = 0;
  pkt_hdr->proc_sess_id = htonl(proc_sess_id_);
  ptr += kPktHeaderSize;
  left -= kPktHeaderSize;

  for (size_t i = 0; i < count; i++) {
    const SegDesc* seg = &segs[i];
    size_t seg_len;

    if (seg->type == SEG_DATA) {
      seg_len = kMinDataSegSize + seg->val.data.len;
      assert(left >= seg_len);
      DataSegment* data_seg = (DataSegment*)ptr;
      data_seg->hdr.seg_type = htons(SEG_DATA);
      data_seg->hdr.seg_len = htons(seg_len);
      data_seg->seq = htonl(seg->val.data.seq);
      data_seg->frag_count = htons(seg->val.data.frag_count);
      data_seg->frag_index = htons(seg->val.data.frag_index);
      memcpy(data_seg->data, seg->val.data.data, seg->val.data.len);
      // release TxRequest
      TxRequest* req = seg->val.data.req;
      if (!--req->ref_count) DelTxRequest(req);

    } else if (seg->type == SEG_ACK) {
      seg_len = kAckSegSize;
      assert(left >= seg_len);
      AckSegment* ack_seg = (AckSegment*)ptr;
      ack_seg->hdr.seg_type = htons(SEG_ACK);
      ack_seg->hdr.seg_len = htons(seg_len);
      ack_seg->proc_sess_id = htonl(seg->val.ack.proc_sess_id);
      ack_seg->base_seq = htonl(seg->val.ack.base_seq);
      ack_seg->seq_bitmap = htonl(seg->val.ack.seq_bitmap);

    } else if (seg->type == SEG_FRAG_ACK) {
      seg_len = kFragAckSegSize;
      assert(left >= seg_len);
      FragAckSegment* ack_seg = (FragAckSegment*)ptr;
      ack_seg->hdr.seg_type = htons(SEG_FRAG_ACK);
      ack_seg->hdr.seg_len = htons(seg_len);
      ack_seg->proc_sess_id = htonl(seg->val.ack.proc_sess_id);
      ack_seg->seq = htonl(seg->val.frag_ack.seq);
      ack_seg->frag_count = htons(seg->val.frag_ack.frag_count);
      ack_seg->frag_base_index = htons(seg->val.frag_ack.frag_base_index);
      ack_seg->frag_bitmap = htonl(seg->val.frag_ack.frag_bitmap);

    } else {
      assert(false);
    }

    ptr += seg_len;
    left -= seg_len;
  }

  size_t pkt_len = ptr - buffer;
  pkt_hdr->pkt_len = htons(pkt_len);
  DLOG("flush %lu segments to network", count);
  on_udp_send_({buffer, pkt_len}, peer->addr());
}

} // namespace hudp

