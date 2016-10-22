#include <unistd.h>
#include <stdio.h>
#include <atomic>
#include "gflags/gflags.h"
#include "hyperudp/hyperudp.h"

DEFINE_bool(echo, true, "whether echo request");

using hudp::HyperUdp;
using hudp::OptionsBuilder;
using hudp::LogLevel;
using hudp::Buf;
using hudp::Addr;
using hudp::Result;

void StartEchoServer()
{
  HyperUdp hyper_udp{
    OptionsBuilder().LogHandler(hudp::kError, [](LogLevel lv, const char* s) {
      printf("[%d] %s\n", (int)lv, s);
    }).Build()
  };

  std::atomic<size_t> recv_count{0};
  std::atomic<size_t> fail_count{0};
  hyper_udp.Init({"0.0.0.0", 19933}, [&](const Buf& buf, const Addr& addr) {
    recv_count++;
    if (FLAGS_echo) {
      hyper_udp.Send(buf, addr, [&](Result res) {
        if (res != hudp::R_SUCCESS) {
          fail_count++;
        }
      });
    }
  });

  while (true) {
    usleep(1000*1000);
    printf("recving %lu qps, %lu failed\n", recv_count.load(),
                                            fail_count.load());
    recv_count.store(0);
    fail_count.store(0);
  }
}

int main(int argc, char* argv[])
{
  google::ParseCommandLineFlags(&argc, &argv, true);
  StartEchoServer();
  return 0;
}
