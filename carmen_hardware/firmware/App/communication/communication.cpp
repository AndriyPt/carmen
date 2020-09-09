//#include "communication.h"
//#include "osal_communication.h"
//#include "orion_protocol/orion_serial_port.h"
//#include "orion_protocol/orion_minor.h"
//#include "orion_protocol/orion_header.h"
//#include "carmen_hardware/protocol.h"
//
//#define COMMAND_BUFFER_SIZE (2048)
//
//static orion::ComPort com_port(USART1);
//static orion::Minor minor(&com_port);
//static uint8_t buffer[COMMAND_BUFFER_SIZE];
//
//static void thread_function(void);
//
//static void pid_read_callback(int32_t left_front_p, int32_t left_front_i, int32_t left_front_d);
//
//void communication_create_thread(void)
//{
//  osal_communication_create_thread(&thread_function);
//}
//
//void thread_function(void)
//{
//  while (true)
//  {
//    if (minor.receiveCommand(buffer, COMMAND_BUFFER_SIZE))
//    {
//      orion::CommandHeader *command_header = reinterpret_cast<orion::CommandHeader*>(buffer);
//      switch(command_header->common.message_id)
//      {
//        case carmen_hardware::HandshakeCommand:
//          carmen_hardware::HandshakeResult handshake_result;
//          handshake_result.header.common.sequence_id = command_header->common.sequence_id;
//          //TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
//          minor.sendResult(handshake_result);
//          break;
//
//        case carmen_hardware::ReadSettingsCommand:
//          eeprom_read_pid(&pid_read_callback);
//          break;
//
//        default:
//          // unknown command
//      }
//    }
//    HAL_Delay(100);
//  }
//}
//
//void pid_read_callback(int32_t left_front_p, int32_t left_front_i, int32_t left_front_d)
//{
//  //TODO: Determine how to pass corresponding sequence id to response
//  carmen_hardware::ReadSettingsResult result;
//  result.data.left_front_p = left_front_p;
//  result.data.left_front_i = left_front_i;
//  result.data.left_front_d = left_front_d;
//  minor.sendResult(result);
//}
