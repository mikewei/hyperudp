#include "gtestx/gtestx.h"
#include "ccbase/timer_wheel.h"
#include "hyperudp/env.h"
#include "hyperudp/tb_max_tx_delay_algo.h"

template <class T>
class TxDelayAlgoTest : public testing::Test
{
protected:
  TxDelayAlgoTest()
    : tw_(1000, false)
    , env_(hudp::OptionsBuilder().Build(), &tw_) {}
  virtual ~TxDelayAlgoTest() {}
  virtual void SetUp() {
    algo_ = new T(env_);
  }
  virtual void TearDown() {
    delete algo_;
  }
  ccb::TimerWheel tw_;
  hudp::Env env_;
  hudp::TxDelayAlgo* algo_;
};

using TestTypes = testing::Types<hudp::MaxTxDelayAlgo,
                                 hudp::TbMaxTxDelayAlgo>;
TYPED_TEST_CASE(TxDelayAlgoTest, TestTypes);

TYPED_TEST(TxDelayAlgoTest, Test)
{
  using hudp::TxDelayAlgo;
  auto action = this->algo_->OnPendingSeg(0);
  ASSERT_EQ(TxDelayAlgo::kNoOp, action.type);
  action = this->algo_->OnPendingSeg(1);
  ASSERT_TRUE(TxDelayAlgo::kFlushPendings == action.type ||
              TxDelayAlgo::kResetFlushTimer == action.type);
}

