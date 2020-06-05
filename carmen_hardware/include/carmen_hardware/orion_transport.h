
class OrionTransportInterface
{
   virtual uint8_t SendFrame(uint8_t *buffer, uint32_t size, uint32_t timeout) = 0;
   virtual uint8_t ReceiveFrame(uint8_t *buffer, uint32_t size, uint8_t *frame_delimeter, uint32_t delimeter_size,
        uint32_t timeout) = 0;
}

class OrionUsartTransport
{
public:

   OrionUsartTransport(std::string portName)
   {
       usart.open(portName);
   }

   ~OrionUsartTransport()
   {
       usart.close();
   }

   uint8_t SendFrame(uint8_t *buffer, uint32_t size, uint32_t timeout)
   {
        TimeoutCheck time(timeout);
        uint32_t send_bytes = 0;
        uint8_t error = 0;
        while (send_bytes < size && time.hasTime() && !error)
        {
            error = usart.send(buffer[send_bytes++]);
        }
        return send_bytes - 1;
   }

   uint8_t ReceiveFrame(uint8_t *buffer, uint32_t size, uint8_t *frame_delimeter, uint32_t delimeter_size,
        uint32_t timeout)
   {
        TimeoutCheck time(timeout);
        int32_t index = buffer_.indexOf(frame_delimeter, delimeter_size);
        if (-1 == index)
        {
            buffer_.clear();
        }
        else
        {
            buffer_.clearBefore(index);
        }

        uint8_t result = 0;
        uint8_t value = 0;
        uint8_t error = 0;
        while (!result && time.hasTime() && !buffer_.isFull())
        {
            error = usart.receive(value);
            if (0 == error)
            {
                buffer_.append(value);
                if (buffer_.endsWith(frame_delimeter, delimeter_size))
                {
                    result = buffer_.size();
                    buffer_.copy(buffer, size);
                }
            }
        }
        return result;
   }

private:

  const uint32_t BUFFER_SIZE = 500;

  USART usart;
  CircularBuffer buffer_(BUFFER_SIZE);
}