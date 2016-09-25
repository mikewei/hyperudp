#include "gtestx/gtestx.h"
#include "hyperudp/hyperudp.h"

TEST(BufTest, Construct)
{
  char buffer[16];
  hudp::Buf buf{buffer, sizeof(buffer)};
  ASSERT_EQ((void*)buffer, buf.ptr());
  ASSERT_EQ(sizeof(buffer), buf.len());
  hudp::Buf buf2(buf);
  ASSERT_EQ((void*)buffer, buf2.ptr());
  ASSERT_EQ(sizeof(buffer), buf2.len());
}
