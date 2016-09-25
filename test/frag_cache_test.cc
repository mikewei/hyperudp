#include "gtestx/gtestx.h"
#include "hyperudp/frag_cache.h"

using hudp::Buf;
using hudp::Addr;
using hudp::Result;

class FragCacheTest : public testing::Test
{
protected:
  FragCacheTest()
    : env_(hudp::OptionsBuilder().Build(), &tw_)
    , fc_(env_)
    , tw_(1000, false) {}
  virtual ~FragCacheTest() {}

  virtual void SetUp() {
    fc_.Init(10000, 5, 
             [this](const Addr&, uint32_t, uint32_t, uint16_t, void**,
                    Result res) {
               if (res == hudp::R_SUCCESS)
                 success_count_++;
               else if (res == hudp::R_TIMEOUT)
                 timeout_count_++;
             });
  }
  virtual void TearDown() {
  }

  hudp::Env env_;
  hudp::FragCache fc_;
  ccb::TimerWheel tw_;
  size_t success_count_ = 0;
  size_t timeout_count_ = 0;
};

TEST_F(FragCacheTest, SingleFrag)
{
  char dummy[256];
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(fc_.AddFrag({"127.0.0.1", 9999}, 0, 1, 1, 0, (void*)dummy));
  ASSERT_EQ(1UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
}

TEST_F(FragCacheTest, MultiFragsNormal)
{
  char dummy[256];
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(fc_.AddFrag({"127.0.0.1", 9999}, 0, 1, 3, 0, (void*)dummy));
  ASSERT_TRUE(fc_.AddFrag({"127.0.0.1", 9999}, 0, 1, 3, 1, (void*)dummy));
  ASSERT_TRUE(fc_.AddFrag({"127.0.0.1", 9999}, 0, 1, 3, 2, (void*)dummy));
  ASSERT_EQ(1UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(fc_.AddFrag({"127.0.0.1", 9999}, 0, 2, 3, 2, (void*)dummy));
  ASSERT_TRUE(fc_.AddFrag({"127.0.0.1", 9999}, 0, 2, 3, 1, (void*)dummy));
  ASSERT_TRUE(fc_.AddFrag({"127.0.0.1", 9999}, 0, 2, 3, 0, (void*)dummy));
  ASSERT_EQ(2UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
}

TEST_F(FragCacheTest, MultiFragsTimeout)
{
  char dummy[256];
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(0UL, this->timeout_count_);
  ASSERT_TRUE(fc_.AddFrag({"127.0.0.1", 9999}, 0, 1, 3, 0, (void*)dummy));
  ASSERT_TRUE(fc_.AddFrag({"127.0.0.1", 9999}, 0, 1, 3, 1, (void*)dummy));
  usleep(10*1000); tw_.MoveOn();
  ASSERT_EQ(0UL, this->success_count_);
  ASSERT_EQ(1UL, this->timeout_count_);
}

PERF_TEST_F(FragCacheTest, MultiFragsPerf)
{
  static hudp::Addr addr{"127.0.0.1", 9999};
  char dummy[256];
  ASSERT_TRUE(fc_.AddFrag(addr, 0, 1, 3, 0, (void*)dummy));
  ASSERT_TRUE(fc_.AddFrag(addr, 0, 1, 3, 1, (void*)dummy));
  ASSERT_TRUE(fc_.AddFrag(addr, 0, 1, 3, 2, (void*)dummy));
}

