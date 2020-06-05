#include "orion_result.h"

class OrionMaster
{
public:
   OperationResult SendAndReceive(uint8_t *buffer, uint32_t size, uint32_t retryTimeout, uint8_t retryCount);
private:
   const uinit32_t BUFFER_SIZE = 500;
   uint8_t internal_send_buffer[BUFFER_SIZE];
   uint8_t internal_receive_buffer[BUFFER_SIZE];
   uinit32_t packetId = 0;
   OrionTransportInterface *transport;
}


OperationResult OrionMaster::SendAndReceive(uint8_t *buffer, uint32_t size, uint32_t retryTimeout, uint8_t retryCount)
{
  OrionRequestHeader requestHeader;
  requestHeader.packetId = ++(this->packetId);

  OperationResult opResult = {0. NULL};

  opResult.errorCode = Framer::encode(&requestHeader, sizeof(OrionRequestHeader), buffer, size,
    internal_send_buffer, BUFFER_SIZE);
  if (0 == opResult.errorCode)
  {
     bool allChecksPassed = false;
     do
     {
       time = now();
       transport->SendFrame(this->internal_send_buffer, BUFFER_SIZE, retryTimeout);
       // TODO: Add functionality to read all frames without waiting and check packet Id
       // if not found needed packet then resend message
       uint8_t result = transport->ReceiveFrame(this->internal_receive_buffer, BUFFER_SIZE, "??? Delimeter", size,
            retryTimeout - time + now());
       if (0 == result)
       {
            OrionResponse response = static_cast<OrionResponse>(this->internal_receive_buffer);
            if (response.header.packetId == requestHeader.packetId)
            {
               // TODO: Read message here
               // Check errorCode from header

            }
       }
     }
     while (0 != result || 0 != retryCount);


  }
  return opResult
}
