#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "gmock-global/gmock-global.h"
#include <string>
#include "control_loop.h"
#include "osal.h"
#include "osal_control_loop.h"

using ::testing::NotNull;

MOCK_GLOBAL_FUNC2(osal_control_loop_create_thread, void(osal_control_loop_function_t, uint16_t));

MOCK_GLOBAL_FUNC1(osal_control_loop_queue_put, osal_control_loop_status_t(const osal_cl_message_t*));

MOCK_GLOBAL_FUNC2(osal_control_loop_queue_get, osal_control_loop_status_t(osal_cl_message_t*, uint32_t));

TEST(TestSuite, initializationTestCase)
{
  EXPECT_GLOBAL_CALL(osal_control_loop_create_thread, osal_control_loop_create_thread(NotNull(), 10)).Times(1);
  control_loop_init();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
