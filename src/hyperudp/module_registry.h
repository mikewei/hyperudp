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
#ifndef HUDP_MODULE_REGISTRY_H_
#define HUDP_MODULE_REGISTRY_H_

#include <map>
#include "ccbase/closure.h"
#include "hyperudp/env.h"

#define HUDP_REGISTER_MODULE(type, name, func) \
  static ModuleRegistry<type>::Registrar registrar_##type##_##name{ \
    #name, func \
  }

#define HUDP_MODULE(type, name, ...) \
  ::hudp::ModuleRegistry<::hudp::type>::Get()->GetModule(name ,##__VA_ARGS__)

namespace hudp {

template <class ModuleType>
class ModuleRegistry {
 public:
  using FactoryFunc = ccb::ClosureFunc<ModuleType*(const Env&)>;
  class Registrar {
  public:
    Registrar(std::string name, FactoryFunc func) {
      ModuleRegistry::Get()->Register(std::move(name), std::move(func));
    }
  };
  static ModuleRegistry* Get() {
    static ModuleRegistry instance;
    return &instance;
  }
  void Register(std::string name, FactoryFunc func) {
    registry_[name] = std::move(func);
  }
  bool IsModuleAvailable(std::string name) {
    return registry_.find(name) != registry_.end();
  }
  ModuleType* GetModule(std::string name, const Env& env) {
    auto it = registry_.find(name);
    if (it == registry_.end()) {
      throw std::runtime_error("cannot find module: " + name);
      return nullptr;
    }
    return it->second(env);
  }

 private:
  std::map<std::string, FactoryFunc> registry_;
};

}  // namespace hudp

#endif  // HUDP_MODULE_REGISTRY_H_
