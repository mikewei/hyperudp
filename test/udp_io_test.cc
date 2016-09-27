#include <sys/types.h>
#include <sys/socket.h>
#include "gtestx/gtestx.h"
#include "hyperudp/env.h"
#include "hyperudp/simple_udp_io.h"
#include "hyperudp/reuse_port_udp_io.h"

template <class T>
class UdpIOTest : public testing::Test
{
protected:
  UdpIOTest()
    : env_(hudp::OptionsBuilder().LogHandler(hudp::kWarning,
                                  [](hudp::LogLevel, const char* s) {
                                    printf("%s\n", s);
                                  }).Build()) {}
  virtual ~UdpIOTest() {}
  virtual void SetUp() {
    udp_io_ = new T(env_);
  }
  virtual void TearDown() {
    delete udp_io_;
  }
  hudp::Env env_;
  hudp::UdpIO* udp_io_;
  std::atomic_int counter_{0};
};

using TestTypes = testing::Types<hudp::SimpleUdpIO
#ifdef SO_REUSEPORT
                                ,hudp::ReusePortUdpIO
#endif
                                >;
TYPED_TEST_CASE(UdpIOTest, TestTypes);

TYPED_TEST(UdpIOTest, Loop)
{
  hudp::Addr addr{"127.0.0.1", 13579};
  ASSERT_TRUE(this->udp_io_->Init(addr, [this](const hudp::Buf& buf, const hudp::Addr& addr) {
    assert(buf.len() == sizeof(int));
    assert(std::string(addr.str()) == "127.0.0.1:13579");
    assert(addr.port() == 13579);
    this->counter_ -= *static_cast<const int*>(buf.ptr());
  }));
  int n = 100;
  this->counter_ += n;
  ASSERT_TRUE(this->udp_io_->Send({&n, sizeof(n)}, addr));
  usleep(10000);
  ASSERT_EQ(0, this->counter_);
}

