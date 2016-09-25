#include "gtestx/gtestx.h"
#include "hyperudp/rx_dup_cache.h"
#include "hyperudp/peer.h"

using hudp::Buf;
using hudp::Addr;
using hudp::Result;

class RxDupCacheTest : public testing::Test
{
protected:
  RxDupCacheTest()
    : tw_(1000, false)
    , env_(hudp::OptionsBuilder().LogHandler(hudp::kWarning,
                                  [](hudp::LogLevel, const char* s) {
                                    printf("%s\n", s);
                                  }).Build(), &tw_)
    , rdc_(env_)
    , peer_(env_, {"127.0.0.1", 9999}, 0, nullptr) {}
  virtual ~RxDupCacheTest() {}

  virtual void SetUp() {
    ASSERT_TRUE(rdc_.Init(true, 100000, 2));
  }
  virtual void TearDown() {
  }

  ccb::TimerWheel tw_;
  hudp::Env env_;
  hudp::RxDupCache rdc_;
  hudp::Peer peer_;
};

TEST_F(RxDupCacheTest, CheckDup)
{
  ASSERT_FALSE(rdc_.CheckDup(&peer_, 0, 1));
  ASSERT_TRUE(rdc_.CheckDup(&peer_, 0, 1));
  ASSERT_TRUE(rdc_.CheckDup(&peer_, 0, 1));
  ASSERT_FALSE(rdc_.CheckDup(&peer_, 0, 2));
  ASSERT_TRUE(rdc_.CheckDup(&peer_, 0, 2));
  ASSERT_FALSE(rdc_.CheckDup(&peer_, 1, 1));
  ASSERT_TRUE(rdc_.CheckDup(&peer_, 1, 1));
  usleep(1000*10);
  tw_.MoveOn();
  ASSERT_FALSE(rdc_.CheckDup(&peer_, 0, 1));
  ASSERT_FALSE(rdc_.CheckDup(&peer_, 0, 2));
  ASSERT_FALSE(rdc_.CheckDup(&peer_, 1, 1));
}

PERF_TEST_F(RxDupCacheTest, CheckDupPerf)
{
  static uint32_t seq = 0;
  rdc_.CheckDup(&peer_, 0, seq++);
  if (seq % 8192 == 0) tw_.MoveOn();
}

PERF_TEST_F(RxDupCacheTest, CheckDupSkipPerf)
{
  static uint32_t seq = 0;
  rdc_.CheckDup(&peer_, 0, seq);
  seq += 64;
  if (seq % 8192 == 0) tw_.MoveOn();
}
