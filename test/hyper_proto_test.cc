#include "gtestx/gtestx.h"
#include "ccbase/timer_wheel.h"
#include "hyperudp/env.h"
#include "hyperudp/buf.h"
#include "hyperudp/addr.h"
#include "hyperudp/hyper_proto.h"

using hudp::Buf;
using hudp::Addr;
using hudp::Result;
using hudp::LogLevel;;

static constexpr LogLevel kCurLogLevel = hudp::kError;

class HyperProtoTest : public testing::TestWithParam<bool>
{
protected:
  HyperProtoTest()
    : pending_count_(0)
    , tw_(1000, false)
    , c_env_(hudp::OptionsBuilder().LogHandler(kCurLogLevel,
                           [this](LogLevel, const char* s) {
                             printf("[client] %s\n", s);
                           }).Proxy(GetProxy()).Build(), &tw_)
    , s_env_(hudp::OptionsBuilder().LogHandler(kCurLogLevel, 
                           [this](LogLevel, const char* s) {
                             printf("[server] %s\n", s);
                           }).Proxy(GetProxy()).Build(), &tw_)
    , client_(c_env_)
    , server_(s_env_)
    , c_recv_buf_len_(0) {}

  virtual ~HyperProtoTest() {}
  ccb::ClosureFunc<Addr(const Addr&)> GetProxy() const {
    if (!GetParam()) return nullptr;
    return [](const Addr& dst) {
      return dst;
    };
  }

  virtual void SetUp() {
    client_.Init([this](const Buf& buf, const Addr& addr) {
      c_env_.Log(hudp::kDebug, "--- send packet to server ---");
      server_.OnUdpRecv(buf, addr);
    }, [this](const Buf& buf, const Addr& addr) {
      c_env_.Log(hudp::kDebug, "--- recv echo response ---");
      pending_count_--;
    });
    server_.Init([this](const Buf& buf, const Addr& addr) {
      s_env_.Log(hudp::kDebug, "--- send packet to client ---");
      ASSERT_TRUE(c_recv_buf_len_ + sizeof(size_t) + buf.len() 
                                  <= sizeof(c_recv_buf_));
      *(size_t*)(c_recv_buf_ + c_recv_buf_len_) = buf.len();
      memcpy(c_recv_buf_ + c_recv_buf_len_ + sizeof(size_t), 
                                      buf.ptr(), buf.len());
      c_recv_buf_len_ += sizeof(size_t) + buf.len();
    }, [this](const Buf& buf, const Addr& addr) {
      s_env_.Log(hudp::kDebug, "--- recv echo request and send reponse ---");
      server_.OnUsrSend(buf, addr, [](Result){});
    });
  }

  virtual void TearDown() {}

  int pending_count_;
  ccb::TimerWheel tw_;
  hudp::Env c_env_;
  hudp::Env s_env_;
  hudp::HyperProto client_;
  hudp::HyperProto server_;
  uint8_t c_recv_buf_[8192];
  size_t c_recv_buf_len_;
};

INSTANTIATE_TEST_CASE_P(EnableProxy, HyperProtoTest, testing::Values(false, true));

PERF_TEST_P(HyperProtoTest, SmallPacket)
{
  static uint8_t pkt_data[100];
  static Addr addr{"127.0.0.1", 9999};
  pending_count_++;
  client_.OnUsrSend({pkt_data, sizeof(pkt_data)}, addr, [](Result){});
  ASSERT_LT(pending_count_, 100) << PERF_ABORT;
  for (size_t n = 0; n < c_recv_buf_len_; ) {
    size_t len = *(size_t*)(c_recv_buf_ + n);
    ASSERT_LE(n + sizeof(size_t) + len, c_recv_buf_len_) << PERF_ABORT;
    client_.OnUdpRecv({c_recv_buf_+ n + sizeof(size_t), len}, addr);
    n += sizeof(size_t) + len;
  }
  c_recv_buf_len_ = 0;
}

PERF_TEST_P(HyperProtoTest, BigPacket)
{
  static uint8_t pkt_data[2000];
  static Addr addr{"127.0.0.1", 9999};
  pending_count_++;
  client_.OnUsrSend({pkt_data, sizeof(pkt_data)}, addr, [](Result){});
  ASSERT_LT(pending_count_, 100) << PERF_ABORT;
  for (size_t n = 0; n < c_recv_buf_len_; ) {
    size_t len = *(size_t*)(c_recv_buf_ + n);
    ASSERT_LE(n + sizeof(size_t) + len, c_recv_buf_len_) << PERF_ABORT;
    client_.OnUdpRecv({c_recv_buf_+ n + sizeof(size_t), len}, addr);
    n += sizeof(size_t) + len;
  }
  c_recv_buf_len_ = 0;
}
