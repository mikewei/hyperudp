#include "gtestx/gtestx.h"
#include "hyperudp/tx_sessions.h"

using hudp::Buf;
using hudp::Addr;
using hudp::Result;

class TxSessionsTest : public testing::Test
{
protected:
  TxSessionsTest()
    : tw_(1000, false)
    , env_(hudp::OptionsBuilder().Build(), &tw_)
    , txs_(env_) {}
  virtual ~TxSessionsTest() {}

  virtual void SetUp() {
    txs_.Init(10000, 1500, {5, 5},
              [this](const Buf&, const Addr&, uint32_t, 
                     uint16_t, uint16_t, void*) {
                send_count_++;
              },
              [this](Result res, void*) {
                if (res == hudp::R_SUCCESS)
                  success_count_++;
                else if (res == hudp::R_TIMEOUT)
                  timeout_count_++;
              });
  }
  virtual void TearDown() {
  }

  ccb::TimerWheel tw_;
  hudp::Env env_;
  hudp::TxSessions txs_;
  size_t send_count_ = 0;
  size_t success_count_ = 0;
  size_t timeout_count_ = 0;
};

TEST_F(TxSessionsTest, SimpleAck)
{
  char dummy[256];
  ASSERT_EQ(0UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, 
                              {"127.0.0.1", 9999},
                              1000, (void*)dummy));
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, 
                              {"127.0.0.1", 9999},
                              1001, (void*)dummy));
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, 
                              {"127.0.0.1", 8888},
                              1000, (void*)dummy));
  ASSERT_FALSE(txs_.AddSession({dummy, sizeof(dummy)}, 
                              {"127.0.0.1", 9999},
                              1000, (void*)dummy));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AckSession({"127.0.0.1", 9999},
                              1000, 1, 0));
  ASSERT_TRUE(txs_.AckSession({"127.0.0.1", 9999},
                              1001, 1, 0));
  ASSERT_TRUE(txs_.AckSession({"127.0.0.1", 8888},
                              1000, 1, 0));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(3UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_FALSE(txs_.AckSession({"127.0.0.1", 9999},
                               1000, 1, 0));
}

TEST_F(TxSessionsTest, SimpleTimeout)
{
  char dummy[256];
  ASSERT_EQ(0UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, 
                              {"127.0.0.1", 9999},
                              1000, (void*)dummy));
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, 
                              {"127.0.0.1", 9999},
                              1001, (void*)dummy));
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, 
                              {"127.0.0.1", 8888},
                              1000, (void*)dummy));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  usleep(1000*50);
  this->tw_.MoveOn();
  ASSERT_EQ(6UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  usleep(1000*50);
  this->tw_.MoveOn();
  ASSERT_EQ(6UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(3UL, this->timeout_count_);
}

TEST_F(TxSessionsTest, FragsAck)
{
  char dummy[4000];
  ASSERT_EQ(0UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, 
                              {"127.0.0.1", 9999},
                              1000, (void*)dummy));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AckSession({"127.0.0.1", 9999},
                              1000, 3, 0));
  ASSERT_TRUE(txs_.AckSession({"127.0.0.1", 9999},
                              1000, 3, 1));
  ASSERT_TRUE(txs_.AckSession({"127.0.0.1", 9999},
                              1000, 3, 2));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(1UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
}

TEST_F(TxSessionsTest, FragsTimeout)
{
  char dummy[4000];
  ASSERT_EQ(0UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, 
                              {"127.0.0.1", 9999},
                              1000, (void*)dummy));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AckSession({"127.0.0.1", 9999},
                              1000, 3, 0));
  usleep(1000*50); this->tw_.MoveOn();
  ASSERT_EQ(5UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AckSession({"127.0.0.1", 9999},
                              1000, 3, 1));
  usleep(1000*50); this->tw_.MoveOn();
  ASSERT_EQ(5UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(1UL, this->timeout_count_);
  ASSERT_FALSE(txs_.AckSession({"127.0.0.1", 9999},
                              1000, 3, 2));
}

PERF_TEST_F(TxSessionsTest, SimpleAckPerf)
{
  static char dummy[256];
  static Addr addr{"127.0.0.1", 9999};
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, addr,
                              1000, (void*)dummy)) << PERF_ABORT;
  ASSERT_TRUE(txs_.AckSession(addr, 1000, 1, 0)) << PERF_ABORT;
}

PERF_TEST_F(TxSessionsTest, FragsAckPerf)
{
  static char dummy[4000];
  static Addr addr{"127.0.0.1", 9999};
  ASSERT_TRUE(txs_.AddSession({dummy, sizeof(dummy)}, addr,
                              1000, (void*)dummy)) << PERF_ABORT;
  ASSERT_TRUE(txs_.AckSession(addr, 1000, 3, 0)) << PERF_ABORT;
  ASSERT_TRUE(txs_.AckSession(addr, 1000, 3, 1)) << PERF_ABORT;
  ASSERT_TRUE(txs_.AckSession(addr, 1000, 3, 2)) << PERF_ABORT;
}

