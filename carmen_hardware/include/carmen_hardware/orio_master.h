#include "orion_result.h"

class OrionMaster
{
public:
   OperationResult SendAndReceive(uint8_t *buffer, uint32_t size, uint32_t retryTimeout, uint8_t retryCount);
private:
   uint8_t internal_buffer[500];
   uinit32_t packetId = 1;
}


OperationResult OrionMaster::SendAndReceive(uint8_t *buffer, uint32_t size, uint32_t retryTimeout, uint8_t retryCount)
{
    uint8_t result = Framer::encode(buffer, size, )


}
