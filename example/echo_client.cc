#include <unistd.h>
#include <atomic>
#include "gflags/gflags.h"
#include "ccbase/token_bucket.h"
#include "ccbase/timer_wheel.h"
#include "hyperudp/hyperudp.h"

using hudp::HyperUdp;
using hudp::OptionsBuilder;
using hudp::LogLevel;
using hudp::Buf;
using hudp::Addr;
using hudp::Result;

DEFINE_string(server, "127.0.0.1:19933", "server address (ip:port)");
DEFINE_uint64(qps, 10000, "qps");
DEFINE_uint64(len, 100, "packet length");
DEFINE_uint64(port, 19944, "local port to bind");

void StartEchoClient()
{
  HyperUdp hyper_udp{
    OptionsBuilder().LogHandler(hudp::kError, [](LogLevel lv, const char* s) {
      printf("[%d] %s\n", (int)lv, s);
    }).Build()
  };

  if (!hyper_udp.Init({"0.0.0.0", static_cast<uint16_t>(FLAGS_port)},
                      [](const Buf& buf, const Addr& addr) {})) {
    fprintf(stderr, "init failed!\n");
    return;
  }

  auto ip_port = Addr::ParseFromString(FLAGS_server);
  if (!ip_port.first) {
    fprintf(stderr, "bad server addr!\n");
    return;
  }
  Addr server_addr = ip_port.second;

  std::atomic<size_t> send_count{0};
  std::atomic<size_t> fail_count{0};
  ccb::TimerWheel tw;
  tw.AddPeriodTimer(1000, [&] {
    printf("sending %lu qps, %lu failed\n", send_count.load(),
                                            fail_count.load());
    send_count.store(0);
    fail_count.store(0);
  });

  char buf[65536];
  uint32_t qps = static_cast<uint32_t>(FLAGS_qps);
  size_t len = (FLAGS_len < sizeof(buf) ? FLAGS_len : sizeof(buf));
  ccb::TokenBucket tb{qps, qps/10, 0};
  while (true) {
    if (!tb.Get(1)) {
      usleep(1000);
      tb.Gen();
      tw.MoveOn();
      continue;
    }
    send_count++;
    hyper_udp.Send({buf, len}, server_addr, [&](Result res) {
      if (res != hudp::R_SUCCESS) {
        fail_count++;
      }
    });
  }
}

int main(int argc, char* argv[])
{
  google::ParseCommandLineFlags(&argc, &argv, true);
  StartEchoClient();
  return 0;
}
