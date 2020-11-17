#include "virtual_com_port.h"
#include "usbd_cdc_if.h"
#include "error.h"
#include <stdint.h>
#include <stdbool.h>

namespace carmen_hardware
{

VirtualComPort::VirtualComPort()
{

}

size_t VirtualComPort::receiveAvailableBuffer(uint8_t *buffer, uint32_t size)
{
    size_t actual_size = dequeue_input_buffer(buffer, size);
    return (actual_size);
}

size_t VirtualComPort::receiveBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout)
{
    // TODO: Add logic to wait for timeout in case if no data is available
    size_t actual_size = this->receiveAvailableBuffer(buffer, size);
    return (actual_size);
}

bool VirtualComPort::hasAvailableBuffer()
{
    bool result = has_items_input_buffer();
    return (result);
}

bool VirtualComPort::sendBuffer(uint8_t *buffer, uint32_t size, uint32_t timeout)
{
    SOFTWARE_ASSERT(NULL != buffer);
    SOFTWARE_ASSERT(0 != size);
    SOFTWARE_ASSERT(size == (size & 0xFFFF));

    uint8_t status = CDC_Transmit_FS(buffer, (uint16_t)(size & 0xFFFF));
    return (USBD_OK == status);
}

}

