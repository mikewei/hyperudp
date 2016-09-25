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
#ifndef _HUDP_HYPERUDP_H
#define _HUDP_HYPERUDP_H

#include <memory>
#include <vector>
#include <functional> 
#include "ccbase/closure.h"
#include "hyperudp/addr.h"
#include "hyperudp/buf.h"

namespace hudp {

/* Log level definition of the library
 */
enum LogLevel {
  kError = 1, kWarning, kInfo, kDebug
};

/* Result definition of Send()
 */
enum Result {
  R_SUCCESS = 0, R_TIMEOUT, R_ERROR
};

class Options;

class OptionsBuilder
{
public:
  OptionsBuilder();
  ~OptionsBuilder();

  /* Build the Options object
   * @num     number of worker threads
   *
   * Build the Options object and return a unique_ptr point to it
   *
   * @return  created Options object
   */
  Options Build() const;

  /* Set the log level and log handler
   * @lv      only logs with level <= @lv will be handled
   * @f       callback to output logs
   *
   * This method allows you to use any logger library you want to ouput the 
   * log from HyperUdp. By default no log will be ouput in any way.
   *
   * @return  self reference as Builder-Pattern
   */
  OptionsBuilder& LogHandler(LogLevel lv, 
                             ccb::ClosureFunc<void(LogLevel, const char*)> f);

  /* Set number of worker threads
   * @num     number of worker threads
   *
   * Proper number of worker threads can get best permformance on modern 
   * multi-core system. Normally the best number is between 2 and CORES.
   *
   * @return  self reference as Builder-Pattern
   */
  OptionsBuilder& WorkerNumber(size_t num);
  OptionsBuilder& WorkerQueueSize(size_t num);

  OptionsBuilder& MaxTxSessions(size_t num);
  OptionsBuilder& MaxUdpPktSize(size_t num);
  OptionsBuilder& RetransTimeouts(std::vector<uint32_t> vec);

  OptionsBuilder& TxDelayAlgoModule(std::string name);
  OptionsBuilder& MaxTxDelay(size_t ms);

  OptionsBuilder& MaxFragCacheNodes(size_t num);
  OptionsBuilder& FragCacheTimeout(size_t ms);

  OptionsBuilder& EnableRxDupCache(bool on);
  OptionsBuilder& RxDupCacheSize(size_t num);
  OptionsBuilder& RxDupCacheTimeout(size_t num);

  OptionsBuilder& UdpIOModule(std::string name);
  OptionsBuilder& ChunkAllocModule(std::string name);

private:
  // not copyable and movable
  OptionsBuilder(const OptionsBuilder&) = delete;
  OptionsBuilder& operator=(const OptionsBuilder&) = delete;
  OptionsBuilder(OptionsBuilder&&) = delete;
  OptionsBuilder& operator=(OptionsBuilder&&) = delete;

  std::unique_ptr<Options> opt_;
};

class HyperUdp
{
public:
  using OnSent = ccb::ClosureFunc<void(Result)>;
  using OnRecv = ccb::ClosureFunc<void(const Buf&, const Addr&)>;

  HyperUdp();
  HyperUdp(const Options& opt);
  virtual ~HyperUdp();

  bool Init(uint16_t port, OnRecv on_recv);
  bool Init(const Addr& addr, OnRecv on_recv);
  void Send(const Buf& buf, const Addr& addr, OnSent done = nullptr);

private:
  // not copyable and movable
  HyperUdp(const HyperUdp&) = delete;
  HyperUdp& operator=(const HyperUdp&) = delete;
  HyperUdp(HyperUdp&&) = delete;
  HyperUdp& operator=(HyperUdp&&) = delete;

  class Impl;
  std::unique_ptr<HyperUdp::Impl> pimpl_;
};

} // namespace hudp

#include "hyperudp/options.h"

#endif // _HUDP_HYPERUDP_H
