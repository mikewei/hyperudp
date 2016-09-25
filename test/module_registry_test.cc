#include "gtestx/gtestx.h"
#include "hyperudp/module_registry.h"

namespace hudp {
class UdpIO;
class FakeModule
{
public:
  virtual ~FakeModule() {}
  virtual void SomeFunction() = 0;
};

class FakeModuleImplA : public FakeModule
{
public:
  FakeModuleImplA(const Env&) {}
  virtual void SomeFunction() override {
  }
};

REGISTER_MODULE(FakeModule, impl_A, [](const Env& env) {
  return new FakeModuleImplA(env);
});

} // namespace hudp

using hudp::ModuleRegistry;
using hudp::FakeModule;
using hudp::OptionsBuilder;
using hudp::Env;

TEST(ModuleRegistryTest, GetModule)
{
  Env env{OptionsBuilder().Build()};
  ASSERT_TRUE(GET_MODULE(FakeModule, "impl_A", env));
  EXPECT_ANY_THROW(GET_MODULE(FakeModule, "impl_B", env));
}

TEST(ModuleRegistryTest, IsModuleAvailable)
{
  ASSERT_TRUE(ModuleRegistry<FakeModule>::Get()->IsModuleAvailable("impl_A"));
  ASSERT_FALSE(ModuleRegistry<FakeModule>::Get()->IsModuleAvailable("impl_B"));
}
