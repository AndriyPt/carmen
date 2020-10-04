#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "gmock-global/gmock-global.h"
#include "error.h"
#include "circular_buffer.h"
#include <stdexcept>

using ::testing::NotNull;
using ::testing::Gt;
using ::testing::Throw;

MOCK_GLOBAL_FUNC3(generate_error, void(error_types_t, uint8_t*, uint32_t));

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

  ON_GLOBAL_CALL(generate_error, generate_error(ERROR_SOFTWARE, NotNull(), Gt(0))).WillByDefault(Throw(
    std::exception()));
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


TEST(TestSuite, happyPath)
{
  uint8_t buffer[] = "Hello from test";
  const uint32_t circular_buffer_length = 2 * sizeof(buffer);
  uint8_t circular_buffer[circular_buffer_length] = { 0 };
  uint8_t output_buffer[circular_buffer_length] = { 0 };
  circular_buffer_t circular_buff_struct;
  uint32_t actual_size = 0;

  ON_GLOBAL_CALL(generate_error, generate_error(ERROR_SOFTWARE, NotNull(), Gt(0))).WillByDefault(Throw(
    std::exception()));
  circular_buffer_init(&circular_buff_struct, circular_buffer, circular_buffer_length);
  circular_buffer_add(&circular_buff_struct, buffer, sizeof(buffer));
  ASSERT_EQ(circular_buff_struct.tail_index - circular_buff_struct.head_index, sizeof(buffer));
  ASSERT_STREQ((char*)circular_buffer, (char*)buffer);

  actual_size = circular_buffer_dequeue(&circular_buff_struct, output_buffer, sizeof(buffer) + 2);
  ASSERT_EQ(circular_buff_struct.tail_index, circular_buff_struct.head_index);
  ASSERT_STREQ((char*)output_buffer, (char*)buffer);
  ASSERT_EQ(sizeof(buffer), actual_size);

  circular_buffer_add(&circular_buff_struct, buffer, sizeof(buffer));
  circular_buffer_add(&circular_buff_struct, buffer, sizeof(buffer));
  ASSERT_ANY_THROW(circular_buffer_add(&circular_buff_struct, buffer, 1));
  actual_size = circular_buffer_dequeue(&circular_buff_struct, output_buffer, sizeof(buffer) + 2);
  ASSERT_EQ(sizeof(buffer) + 2, actual_size);
  actual_size = circular_buffer_dequeue(&circular_buff_struct, output_buffer, sizeof(buffer) - 2);
  ASSERT_EQ(circular_buff_struct.tail_index, circular_buff_struct.head_index);
  ASSERT_EQ(sizeof(buffer) - 2, actual_size);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
