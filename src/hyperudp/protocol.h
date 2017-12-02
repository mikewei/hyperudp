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
#ifndef HUDP_PROTOCOL_H_
#define HUDP_PROTOCOL_H_

namespace hudp {

// ---------------- protocol definition ------------------

struct PacketHeader {
  uint8_t hudp_tag;
  uint8_t hudp_ver;
  uint16_t pkt_len;
  uint16_t from_port;
  uint16_t to_port;
  uint32_t from_ip;
  uint32_t to_ip;
  uint32_t proc_sess_id;
} __attribute__((__packed__));

struct SegmentHeader {
  uint16_t seg_type;
  uint16_t seg_len;
} __attribute__((__packed__));

enum SegmentType {
  SEG_DATA = 1, SEG_ACK = 2, SEG_FRAG_ACK = 3
};

struct DataSegment {
  SegmentHeader hdr;
  uint32_t seq;
  uint16_t frag_count;
  uint16_t frag_index;
  uint8_t data[0];
} __attribute__((__packed__));

struct AckSegment {
  SegmentHeader hdr;
  uint32_t proc_sess_id;
  uint32_t base_seq;
  uint32_t seq_bitmap;
} __attribute__((__packed__));

struct FragAckSegment {
  SegmentHeader hdr;
  uint32_t proc_sess_id;
  uint32_t seq;
  uint16_t frag_count;
  uint16_t frag_base_index;
  uint32_t frag_bitmap;
} __attribute__((__packed__));

}

#endif  // HUDP_PROTOCOL_H_
