#include "gtestx/gtestx.h"
#include "ccbase/timer_wheel.h"
#include "hyperudp/peer_manager.h"

using hudp::Buf;
using hudp::Addr;
using hudp::Result;
using hudp::Peer;
using hudp::SegDesc;

class PeerManagerTest : public testing::Test
{
protected:
  PeerManagerTest()
    : tw_(1000, false)
    , env_(hudp::OptionsBuilder().Build(), &tw_)
    , pm_(env_, [](Peer*, const SegDesc*, size_t){}) {}
  virtual ~PeerManagerTest() {}

  virtual void SetUp() {
  }
  virtual void TearDown() {
  }

  ccb::TimerWheel tw_;
  hudp::Env env_;
  hudp::PeerManager pm_;
  hudp::Peer* peer_;
};

TEST_F(PeerManagerTest, GetPeer)
{
  peer_ = pm_.GetPeer({"127.0.0.1", 9999});
  ASSERT_TRUE(static_cast<bool>(peer_));
  ASSERT_EQ(peer_, pm_.GetPeer({"127.0.0.1", 9999}));
}

PERF_TEST_F(PeerManagerTest, GetPeerPerf)
{
  static uint16_t port = 0;
  peer_ = pm_.GetPeer({0x7f000001, static_cast<uint16_t>(++port % 4096)});
}
