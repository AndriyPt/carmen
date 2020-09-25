#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <string>
#include "communication.h"

TEST(TestSuite, positiveTestCase)
{
  communication_init();
  ASSERT_EQ(0, 0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
