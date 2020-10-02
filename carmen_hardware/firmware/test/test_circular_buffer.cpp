#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "circular_buffer.h"

TEST(TestSuite, parametersValidation)
{
  const uint32_t buffer_length = 20;
  uint8_t buffer[buffer_length] = { 0 };
  const uint32_t circular_buffer_length = 20;
  uint8_t circular_buffer[circular_buffer_length] = { 0 };

  circular_buffer_t circular_buff_struct;
  circular_buff_struct.head_index = 0;
  circular_buff_struct.tail_index = 0;

  circular_buff_struct.buffer_size = 10;
  circular_buff_struct.p_buffer = NULL;

  ASSERT_ANY_THROW(circular_buffer_add(NULL, NULL, 10));
  ASSERT_ANY_THROW(circular_buffer_add(NULL, buffer, buffer_length));
  ASSERT_ANY_THROW(circular_buffer_add(&circular_buff_struct, buffer, buffer_length));

  ASSERT_ANY_THROW(circular_buffer_dequeue(NULL, NULL, 10));
  ASSERT_ANY_THROW(circular_buffer_dequeue(NULL, buffer, buffer_length));
  ASSERT_ANY_THROW(circular_buffer_dequeue(&circular_buff_struct, buffer, buffer_length));

  circular_buff_struct.p_buffer = circular_buffer;
  circular_buff_struct.buffer_size = 0;
  ASSERT_ANY_THROW(circular_buffer_add(&circular_buff_struct, buffer, buffer_length));
  ASSERT_ANY_THROW(circular_buffer_dequeue(&circular_buff_struct, buffer, buffer_length));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
