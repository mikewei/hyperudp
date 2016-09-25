#include "gtestx/gtestx.h"
#include "hyperudp/tx_buffer.h"

using hudp::Buf;
using hudp::Addr;
using hudp::Peer;
using hudp::SegDesc;
using hudp::kMinDataSegSize;
using hudp::kAckSegSize;
using hudp::kFragAckSegSize;
using hudp::kMinSegSize;

class TxBufferTest : public testing::Test
{
protected:
  TxBufferTest()
    : env_(hudp::OptionsBuilder().TxDelayAlgoModule("noop").Build(), &tw_)
    , tw_(1000, false)
    , tb_(env_, nullptr, 
          // OnFlush
          [this](Peer*, const SegDesc* segs, size_t count){
            for (size_t i = 0; i < count; i++) {
              switch (segs[i].type) {
              case hudp::SEG_DATA: flushed_data_segs_++; break;
              case hudp::SEG_ACK: flushed_ack_segs_++; break;
              case hudp::SEG_FRAG_ACK: flushed_fack_segs_++; break;
              }
            }
          }, 
          kMinDataSegSize + 100 + kAckSegSize + kFragAckSegSize) {}

  virtual ~TxBufferTest() {}

  virtual void SetUp() {
  }
  virtual void TearDown() {
  }

  hudp::Env env_;
  ccb::TimerWheel tw_;
  hudp::TxBuffer tb_;
  size_t flushed_data_segs_ = 0;
  size_t flushed_ack_segs_ = 0;
  size_t flushed_fack_segs_ = 0;
};

TEST_F(TxBufferTest, AddFlush)
{
  uint8_t dummy[100];
  tb_.AddAck(0, 0, 1, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddAck(0, 1, 2, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddData(0, 0, 0, {dummy, sizeof(dummy)}, nullptr);
  ASSERT_EQ(1UL, flushed_data_segs_);
  ASSERT_EQ(1UL, flushed_ack_segs_);
  ASSERT_EQ(1UL, flushed_fack_segs_);
}

TEST_F(TxBufferTest, AddNoFlush)
{
  uint8_t dummy[100 - kMinSegSize];
  tb_.AddAck(0, 0, 1, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddAck(0, 1, 2, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddData(0, 0, 0, {dummy, sizeof(dummy)}, nullptr);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
}

TEST_F(TxBufferTest, AddMergedNoFlush)
{
  uint8_t dummy[100 - kMinSegSize];
  tb_.AddAck(0, 0, 1, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddAck(0, 1, 2, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddData(0, 0, 0, {dummy, sizeof(dummy)}, nullptr);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  // merged ack
  tb_.AddAck(0, 2, 1, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
}

TEST_F(TxBufferTest, AddNotMergedFlush)
{
  uint8_t dummy[100 - kAckSegSize];
  tb_.AddAck(0, 0, 1, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddAck(0, 1, 2, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddData(0, 0, 0, {dummy, sizeof(dummy)}, nullptr);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  // not merged ack
  tb_.AddAck(0, 32, 1, 0);
  ASSERT_EQ(1UL, flushed_data_segs_);
  ASSERT_EQ(2UL, flushed_ack_segs_);
  ASSERT_EQ(1UL, flushed_fack_segs_);
}

TEST_F(TxBufferTest, AddMergedFragNoFlush)
{
  uint8_t dummy[100 - kMinSegSize];
  tb_.AddAck(0, 0, 1, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddAck(0, 1, 2, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddData(0, 0, 0, {dummy, sizeof(dummy)}, nullptr);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  // merged frag-ack
  tb_.AddAck(0, 1, 2, 1);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
}

TEST_F(TxBufferTest, AddNotMergedFragFlush)
{
  uint8_t dummy[100 - kFragAckSegSize];
  tb_.AddAck(0, 0, 1, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddAck(0, 1, 33, 0);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  tb_.AddData(0, 0, 0, {dummy, sizeof(dummy)}, nullptr);
  ASSERT_EQ(0UL, flushed_data_segs_);
  ASSERT_EQ(0UL, flushed_ack_segs_);
  ASSERT_EQ(0UL, flushed_fack_segs_);
  // not merged frag-ack
  tb_.AddAck(0, 1, 33, 32);
  ASSERT_EQ(1UL, flushed_data_segs_);
  ASSERT_EQ(1UL, flushed_ack_segs_);
  ASSERT_EQ(2UL, flushed_fack_segs_);
}
