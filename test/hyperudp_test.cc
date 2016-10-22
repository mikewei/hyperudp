#include "gtestx/gtestx.h"
#include "hyperudp/env.h"
#include "hyperudp/hyperudp.h"

using hudp::Addr;
using hudp::Buf;
using hudp::HyperUdp;

class HyperUdpTest : public testing::Test
{
protected:
  HyperUdpTest()
    : hyper_udp_(new HyperUdp(hudp::OptionsBuilder().Build())) {}
  virtual ~HyperUdpTest() {}
  virtual void SetUp() {
    ASSERT_TRUE(hyper_udp_->Init({"0.0.0.0", 13579}, 
           [this](const Buf& buf, const Addr& addr) {
             ASSERT_TRUE(buf.len() == sizeof(int));
             ASSERT_TRUE(std::string(addr.str()) == "127.0.0.1:13579");
             counter_ -= *static_cast<const int*>(buf.ptr());
           }));
  }
  virtual void TearDown() {
  }
  std::unique_ptr<HyperUdp> hyper_udp_;
  std::atomic_int counter_{0};
};

TEST_F(HyperUdpTest, Loop)
{
  int n = 100;
  counter_ += n;
  hyper_udp_->Send({&n, sizeof(n)}, {"127.0.0.1", 13579});
  usleep(1000*20);
  ASSERT_EQ(0, counter_);
}

