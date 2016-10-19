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
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include "hyperudp/env.h"
#include "hyperudp/chunk_alloc.h"
#include "hyperudp/module_registry.h"

namespace hudp {

static unsigned int GenerateSeed()
{
  unsigned int seed = static_cast<unsigned int>(time(nullptr));
  int f = open("/dev/urandom", O_RDONLY);
  if (f >= 0) read(f, &seed, sizeof(seed));
  return seed;
}

static unsigned int GetGlobalSeed()
{
  static unsigned int global_seed = GenerateSeed();
  return global_seed;
}

thread_local unsigned int Env::seed_tls = GetGlobalSeed();

Env::Env(const Options& opt, ccb::TimerWheel* tw)
  : BaseEnv(opt)
  , timerw_(tw)
  , alloc_(HUDP_MODULE(ChunkAlloc, BaseEnv::opt().chunk_alloc_module, *this))
{
  Log(kDebug, "initialized seed = %u", seed_tls);
}

Env::~Env()
{
}

uint32_t Env::Rand() const
{
  return static_cast<uint32_t>(rand_r(&seed_tls));
}

}
