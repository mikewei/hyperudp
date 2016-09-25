#include "gtestx/gtestx.h"
#include "hyperudp/hyperudp.h"

TEST(AddrTest, Construct)
{
  hudp::Addr addr{"127.0.0.1", 8000};
  ASSERT_EQ(0x7f000001U, addr.ip());
  ASSERT_EQ(8000U, addr.port());
  auto sa = (sockaddr_in*)addr.sockaddr_ptr();
  ASSERT_EQ(AF_INET, sa->sin_family);
  ASSERT_EQ(inet_addr("127.0.0.1"), sa->sin_addr.s_addr);
  ASSERT_EQ(8000, ntohs(sa->sin_port));
  ASSERT_EQ("127.0.0.1:8000", std::string(addr.str()));
  ASSERT_EQ(hudp::Addr("127.0.0.1", 8000), addr);
}

TEST(AddrTest, Parse)
{
  ASSERT_FALSE(hudp::Addr::ParseFromString("").first);
  ASSERT_FALSE(hudp::Addr::ParseFromString("127.0.0.1").first);
  ASSERT_FALSE(hudp::Addr::ParseFromString("8000").first);
  ASSERT_FALSE(hudp::Addr::ParseFromString(":8000").first);
  ASSERT_FALSE(hudp::Addr::ParseFromString("127.0.0.1:abc").first);
  ASSERT_FALSE(hudp::Addr::ParseFromString("127.0.0.1:65536").first);
  auto res = hudp::Addr::ParseFromString("127.0.0.1:8000");
  ASSERT_TRUE(res.first);
  ASSERT_EQ(hudp::Addr("127.0.0.1", 8000), res.second);
}
