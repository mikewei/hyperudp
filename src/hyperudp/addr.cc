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
#include <string>
#include <regex>
#include "hyperudp/addr.h"

namespace hudp {

std::pair<bool, Addr> Addr::ParseFromString(const std::string& ip_port) {
#if defined(__GNUC__) && defined(__GNUC_MINOR__) && \
    defined(__GNUC_PATCHLEVEL__) && \
    __GNUC__*10000000 + __GNUC_MINOR__*10000 + __GNUC_PATCHLEVEL__ >= 40090000
  // gcc >= 4.9.0 and std::regex is available
  std::regex re("([0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+):([0-9]+)");
  std::smatch match;
  if (!std::regex_match(ip_port, match, re)) {
    return std::make_pair(false, Addr(0U, 0));
  }
  std::string ip = match[1];
  unsigned long port = strtoul(match[2].str().c_str(), NULL, 10);
#else
  // naive string parse
  auto pos = ip_port.find(':');
  if (pos == std::string::npos || pos == 0 || pos == ip_port.size() - 1) {
    return std::make_pair(false, Addr(0U, 0));
  }
  std::string ip = ip_port.substr(0, pos);
  unsigned long port = strtoul(ip_port.c_str() + pos + 1, NULL, 10);
#endif
  if (port <= 0 || port > 65535) {
    return std::make_pair(false, Addr(0U, 0));
  }
  return std::make_pair(true, Addr(ip, (uint16_t)port));
}

}
