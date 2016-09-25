#include "hyperudp/module_registry.h"
#include "hyperudp/simple_udp_io.h"
#include "hyperudp/reuse_port_udp_io.h"
#include "hyperudp/simple_chunk_alloc.h"
#include "hyperudp/noop_tx_delay_algo.h"
#include "hyperudp/max_tx_delay_algo.h"
#include "hyperudp/tb_max_tx_delay_algo.h"

namespace hudp {

// UdpIO modules

REGISTER_MODULE(UdpIO, simple, [](const Env& env) {
  return new SimpleUdpIO(env);
});

REGISTER_MODULE(UdpIO, reuse_port, [](const Env& env) {
  return new ReusePortUdpIO(env);
});

// ChunkAlloc modules

REGISTER_MODULE(ChunkAlloc, simple, [](const Env& env) {
  return new SimpleChunkAlloc(env);
});

// TxDelayAlgo modules

REGISTER_MODULE(TxDelayAlgo, noop, [](const Env& env) {
  return new NoopTxDelayAlgo(env);
});

REGISTER_MODULE(TxDelayAlgo, max_tx_delay, [](const Env& env) {
  return new MaxTxDelayAlgo(env);
});

REGISTER_MODULE(TxDelayAlgo, tb_max_tx_delay, [](const Env& env) {
  return new TbMaxTxDelayAlgo(env);
});

} // namespace hudp
