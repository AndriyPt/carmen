#ifndef VIRTUAL_COM_PORT_H
#define VIRTUAL_COM_PORT_H

#include <stdint.h>
#include <stdbool.h>
#include "orion_protocol/orion_communication.h"

namespace carmen_hardware
{

class VirtualComPort: public orion::Communication
{
public:
  VirtualComPort();
  virtual size_t receiveAvailableBuffer(uint8_t *buffer, uint32_t size);
  virtual size_t receiveBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout);
  virtual bool hasAvailableBuffer();
  virtual bool sendBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout);
  virtual ~VirtualComPort() = default;
};

}  // namespace carmen_hardware

#endif  // VIRTUAL_COM_PORT_H
