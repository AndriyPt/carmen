

struct OperationResult
{
  uint8_t errorCode;
  uint8_t* data;
}

class CommunicationException
{
public:
   std::string GetErrorMessage();
   uint8_t GetErrorCode();
}