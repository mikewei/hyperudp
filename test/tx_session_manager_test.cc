#include "gtestx/gtestx.h"
#include "hyperudp/peer.h"
#include "hyperudp/tx_session_manager.h"

using hudp::Buf;
using hudp::Addr;
using hudp::Result;
using hudp::TxRequest;

class TxSessionManagerTest : public testing::Test
{
protected:
  TxSessionManagerTest()
    : tw_(1000, false)
    , env_(hudp::OptionsBuilder().LogHandler(hudp::kError,
                                  [](hudp::LogLevel, const char* s) {
                                    printf("%s\n", s);
                                  }).Build(), &tw_)
    , txs_(env_) {}
  virtual ~TxSessionManagerTest() {}

  virtual void SetUp() {
    txs_.Init(10000, 1500, {5, 5},
              [this](TxRequest*, const Buf&, uint32_t, uint16_t, uint16_t) {
                send_count_++;
              },
              [this](TxRequest*, Result res) {
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
  hudp::TxSessionManager txs_;
  size_t send_count_ = 0;
  size_t success_count_ = 0;
  size_t timeout_count_ = 0;
};

TEST_F(TxSessionManagerTest, SimpleAck)
{
  constexpr size_t data_len = 256;
  hudp::Peer peer1{env_, {0U, 9999}, 1000, nullptr};
  hudp::Peer peer2{env_, {0U, 8888}, 1000, nullptr};
  hudp::Peer peer1_dup{env_, {0U, 9999}, 1000, nullptr};
  TxRequest req1{sizeof(TxRequest) + data_len, 0U, 9999, 0, &peer1, nullptr};
  TxRequest req2{sizeof(TxRequest) + data_len, 0U, 8888, 0, &peer2, nullptr};
  TxRequest req1_dup{sizeof(TxRequest) + data_len, 0U, 9999, 0, &peer1_dup, nullptr};

  ASSERT_EQ(0UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AddSession(&req1));
  ASSERT_TRUE(txs_.AddSession(&req1));
  ASSERT_TRUE(txs_.AddSession(&req2));
  ASSERT_FALSE(txs_.AddSession(&req1_dup));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AckSession(&peer1, 1000, 1, 0));
  ASSERT_TRUE(txs_.AckSession(&peer1, 1001, 1, 0));
  ASSERT_TRUE(txs_.AckSession(&peer2, 1000, 1, 0));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(3UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_FALSE(txs_.AckSession(&peer1, 1000, 1, 0));
}

TEST_F(TxSessionManagerTest, SimpleTimeout)
{
  constexpr size_t data_len = 256;
  hudp::Peer peer1{env_, {0U, 9999}, 1000, nullptr};
  hudp::Peer peer2{env_, {0U, 8888}, 1000, nullptr};
  TxRequest req1{sizeof(TxRequest) + data_len, 0U, 9999, 0, &peer1, nullptr};
  TxRequest req2{sizeof(TxRequest) + data_len, 0U, 8888, 0, &peer2, nullptr};

  ASSERT_EQ(0UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AddSession(&req1));
  ASSERT_TRUE(txs_.AddSession(&req1));
  ASSERT_TRUE(txs_.AddSession(&req2));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  usleep(1000*50);
  this->tw_.MoveOn(); // retrans here
  ASSERT_EQ(6UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  usleep(1000*50);
  this->tw_.MoveOn(); // timeout here
  ASSERT_EQ(6UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(3UL, this->timeout_count_);
}

TEST_F(TxSessionManagerTest, FragsAck)
{
  constexpr size_t data_len = 4000;
  hudp::Peer peer1{env_, {0U, 9999}, 1000, nullptr};
  TxRequest req1{sizeof(TxRequest) + data_len, 0U, 9999, 0, &peer1, nullptr};

  ASSERT_EQ(0UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AddSession(&req1));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AckSession(&peer1, 1000, 3, 0));
  ASSERT_TRUE(txs_.AckSession(&peer1, 1000, 3, 1));
  ASSERT_TRUE(txs_.AckSession(&peer1, 1000, 3, 2));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(1UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
}

TEST_F(TxSessionManagerTest, FragsTimeout)
{
  constexpr size_t data_len = 4000;
  hudp::Peer peer1{env_, {0U, 9999}, 1000, nullptr};
  TxRequest req1{sizeof(TxRequest) + data_len, 0U, 9999, 0, &peer1, nullptr};

  ASSERT_EQ(0UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AddSession(&req1));
  ASSERT_EQ(3UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AckSession(&peer1, 1000, 3, 0));
  usleep(1000*50); this->tw_.MoveOn();
  ASSERT_EQ(5UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(txs_.AckSession(&peer1, 1000, 3, 1));
  usleep(1000*50); this->tw_.MoveOn();
  ASSERT_EQ(5UL, this->send_count_);
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(1UL, this->timeout_count_);
  ASSERT_FALSE(txs_.AckSession(&peer1, 1000, 3, 2));
}

PERF_TEST_F(TxSessionManagerTest, SimpleAckPerf)
{
  constexpr size_t data_len = 256;
  static uint32_t seq = 1000;
  static hudp::Peer peer1{env_, {0U, 9999}, seq, nullptr};
  static TxRequest req1{sizeof(TxRequest) + data_len, 0U, 9999, 0, &peer1, nullptr};

  ASSERT_TRUE(txs_.AddSession(&req1)) << PERF_ABORT;
  ASSERT_TRUE(txs_.AckSession(&peer1, seq++, 1, 0)) << PERF_ABORT;
}

PERF_TEST_F(TxSessionManagerTest, FragsAckPerf)
{
  constexpr size_t data_len = 4000;
  static uint32_t seq = 1000;
  static hudp::Peer peer1{env_, {0U, 9999}, seq, nullptr};
  TxRequest req1{sizeof(TxRequest) + data_len, 0U, 9999, 0, &peer1, nullptr};
  ASSERT_TRUE(txs_.AddSession(&req1));
  ASSERT_TRUE(txs_.AckSession(&peer1, seq, 3, 0)) << PERF_ABORT;
  ASSERT_TRUE(txs_.AckSession(&peer1, seq, 3, 1)) << PERF_ABORT;
  ASSERT_TRUE(txs_.AckSession(&peer1, seq, 3, 2)) << PERF_ABORT;
  seq++;
}

