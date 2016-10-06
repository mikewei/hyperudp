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
#ifndef _HUDP_CONSTANTS_H
#define _HUDP_CONSTANTS_H

#include <assert.h>
#include "hyperudp/protocol.h"

namespace hudp {

// constants definition
static constexpr size_t kMaxUdpPktSize = 4096;
static constexpr size_t kMaxBufferedSegs = 256;
static constexpr size_t kPktHeaderSize = sizeof(PacketHeader);
static constexpr size_t kSegHeaderSize = sizeof(SegmentHeader);
static constexpr size_t kMinDataSegSize = sizeof(DataSegment);
static constexpr size_t kAckSegSize = sizeof(AckSegment);
static constexpr size_t kFragAckSegSize = sizeof(FragAckSegment);
static constexpr size_t kMinSegSize = kAckSegSize;
static constexpr size_t kMaxAckSegSize = kFragAckSegSize;
static constexpr size_t kMinFragDataSize = 64;
static constexpr size_t kMinMaxUdpPktSize = kPktHeaderSize + kMinDataSegSize
                                            + kMinFragDataSize
                                            + kMaxAckSegSize * 2;
                                                          
static inline size_t FragDataSize(size_t max_udp_pkt_size)
{
  assert(max_udp_pkt_size >= kMinMaxUdpPktSize);
  return max_udp_pkt_size - kMinMaxUdpPktSize + kMinFragDataSize;
}

static inline size_t MaxSegmentsSize(size_t max_udp_pkt_size)
{
  assert(max_udp_pkt_size >= kMinMaxUdpPktSize);
  return max_udp_pkt_size - kPktHeaderSize;
}

}

#endif // _HUDP_CONSTANTS_H
